#include <chrono>
#include <algorithm>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <swarm_interfaces/msg/drone_state.hpp>

using namespace std::chrono_literals;

struct DroneRecord {
    bool ever_seen{false};
    bool armed{false};
    int recv_streak{0};
    float x{0.0f}, y{0.0f}, z{0.0f};
    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
};

enum class ManagerState {
    MESH_CHECK,
    ARMING,
    ARMED,
    FAULT
};

class SwarmArmingManager : public rclcpp::Node {
public:
    SwarmArmingManager() : Node("swarm_arming_manager"), mgr_state_(ManagerState::MESH_CHECK)
    {
        declare_parameter("num_drones", 3);
        declare_parameter("stability_msgs", 15);
        declare_parameter("arm_timeout_s", 15.0);
        declare_parameter("peer_timeout", 1.5);
        declare_parameter("safe_radius", 2.5);
        declare_parameter("position_uncertainty", 0.5);

        num_drones_ = get_parameter("num_drones").as_int();
        stability_msgs_ = get_parameter("stability_msgs").as_int();
        arm_timeout_s_ = get_parameter("arm_timeout_s").as_double();
        peer_timeout_s_ = get_parameter("peer_timeout").as_double();
        minimum_arm_separation_m_ =
            get_parameter("safe_radius").as_double() +
            get_parameter("position_uncertainty").as_double();
        if (num_drones_ < 1) {
            throw std::invalid_argument("num_drones must be positive");
        }
        active_drone_ids_.reserve(static_cast<size_t>(num_drones_));
        for (int id = 1; id <= num_drones_; ++id) {
            active_drone_ids_.push_back(id);
        }

        auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
        arm_clearance_pub_ = create_publisher<std_msgs::msg::Bool>("/swarm/arm_clearance", latched_qos);
        mission_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/swarm/mission_ready", latched_qos);

        auto px4_qos = rclcpp::QoS(1).best_effort().transient_local();
        const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        for (int i = 1; i <= num_drones_; ++i) {
            std::string cmd_topic = "/drone_" + std::to_string(i) + "/fmu/in/vehicle_command";
            cmd_pubs_[i] = create_publisher<px4_msgs::msg::VehicleCommand>(cmd_topic, px4_qos);

            drones_[i] = DroneRecord{};
            std::string state_topic = "/drone_" + std::to_string(i) + "/swarm/self_state";
            auto sub = create_subscription<swarm_interfaces::msg::DroneState>(
                state_topic, state_qos,
                [this, i](swarm_interfaces::msg::DroneState::SharedPtr m) {
                    if (m->drone_id != i ||
                        !std::isfinite(m->x) || !std::isfinite(m->y) ||
                        !std::isfinite(m->z)) {
                        RCLCPP_ERROR_THROTTLE(
                            get_logger(), *get_clock(), 1000,
                            "[ARM MGR] Rejecting invalid state on drone %d topic.", i);
                        return;
                    }
                    auto& rec = drones_[i];
                    rec.x = m->x;
                    rec.y = m->y;
                    rec.z = m->z;
                    if (!rec.ever_seen) {
                        rec.ever_seen = true;
                        float spawn_dist = std::sqrt(rec.x * rec.x + rec.y * rec.y + rec.z * rec.z);
                        RCLCPP_INFO(get_logger(), "[ARM MGR] Connected to Drone %d | Spawn Coords: (%.2f, %.2f, %.2f) | Spawn Dist to Origin: %.2fm",
                            i, rec.x, rec.y, rec.z, spawn_dist);
                    }
                    rec.recv_streak++;
                    rec.armed = (m->arming_state == 2);
                    rec.last_seen = now();
                });
            subs_.push_back(sub);
        }

        timer_ = create_wall_timer(200ms, std::bind(&SwarmArmingManager::monitor_loop, this));
        RCLCPP_INFO(get_logger(), "[ARM MGR] Node initialized for %d drones. Checking mesh connection...", num_drones_);
    }

private:
    void monitor_loop() {
        tick_++;

        switch (mgr_state_) {
        case ManagerState::MESH_CHECK:
            if (all_drones_stable()) {
                RCLCPP_INFO(get_logger(), "[ARM MGR] Mesh check PASSED! All drones connected. Publishing Arm Clearance...");
                mgr_state_ = ManagerState::ARMING;
                phase_start_ = now();
                send_arm_clearance_and_commands();
            } else if (tick_ % 10 == 0) {
                RCLCPP_INFO(get_logger(), "[ARM MGR] Waiting for mesh connectivity...");
            }
            break;

        case ManagerState::ARMING:
            if (all_drones_armed()) {
                RCLCPP_INFO(get_logger(),
                    "[ARM MGR] ALL ACTIVE DRONES ARMED! Publishing Mission Ready.");
                mgr_state_ = ManagerState::ARMED;

                for (const auto& [id, rec] : drones_) {
                    float spawn_dist = std::sqrt(rec.x * rec.x + rec.y * rec.y + rec.z * rec.z);
                    const bool active = std::binary_search(
                        active_drone_ids_.begin(), active_drone_ids_.end(), id);
                    RCLCPP_INFO(get_logger(),
                        "[ARM MGR]   -> Drone %d %s | Final Coords: (%.2f, %.2f, %.2f) | Dist to Origin: %.2fm",
                        id, active ? "ARMED ✓" : "IDLE", rec.x, rec.y, rec.z, spawn_dist);
                }

                std_msgs::msg::Bool ready_msg;
                ready_msg.data = true;
                mission_ready_pub_->publish(ready_msg);
            } else {
                if (tick_ % 10 == 0) {
                    send_arm_clearance_and_commands();
                }
                if ((now() - phase_start_).seconds() > arm_timeout_s_) {
                    RCLCPP_ERROR(get_logger(), "[ARM MGR] Timeout waiting for all drones to arm!");
                    mgr_state_ = ManagerState::FAULT;
                }
            }
            break;

        case ManagerState::ARMED:
            break;

        case ManagerState::FAULT:
            break;
        }
    }

    bool all_drones_stable() {
        if (static_cast<int>(drones_.size()) < num_drones_) return false;
        for (const auto& [id, rec] : drones_) {
            (void)id;
            if (!rec.ever_seen || rec.recv_streak < stability_msgs_ ||
                (now() - rec.last_seen).seconds() > peer_timeout_s_) {
                return false;
            }
        }

        for (auto first = drones_.begin(); first != drones_.end(); ++first) {
            for (auto second = std::next(first); second != drones_.end(); ++second) {
                const float dx = first->second.x - second->second.x;
                const float dy = first->second.y - second->second.y;
                const float dz = first->second.z - second->second.z;
                const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (distance < minimum_arm_separation_m_) {
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "[ARM MGR] Refusing arm: drones %d and %d are %.2fm apart; "
                        "minimum startup separation is %.2fm.",
                        first->first, second->first, distance,
                        minimum_arm_separation_m_);
                    return false;
                }
            }
        }
        return true;
    }

    bool all_drones_armed() const {
        for (const int id : active_drone_ids_) {
            const auto record_it = drones_.find(id);
            if (record_it == drones_.end() || !record_it->second.armed) return false;
        }
        return true;
    }

    void send_arm_clearance_and_commands() {
        std_msgs::msg::Bool msg;
        msg.data = true;
        arm_clearance_pub_->publish(msg);

        for (const int i : active_drone_ids_) {
            auto& rec = drones_[i];
            float spawn_dist = std::sqrt(rec.x * rec.x + rec.y * rec.y + rec.z * rec.z);
            RCLCPP_INFO(get_logger(),
                "[ARM MGR] Target Drone %d | Coords: (%.2f, %.2f, %.2f) | Spawn Dist to Origin: %.2fm | Arming State: %s",
                i, rec.x, rec.y, rec.z, spawn_dist, rec.armed ? "ARMED" : "ARMING");

            if (!rec.armed && cmd_pubs_.count(i)) {
                px4_msgs::msg::VehicleCommand cmd{};
                cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
                cmd.param1 = 1.0f; // Arm
                cmd.target_system = i + 1;
                cmd.target_component = 1;
                cmd.source_system = 255;
                cmd.from_external = true;
                cmd.timestamp = now().nanoseconds() / 1000;
                cmd_pubs_[i]->publish(cmd);
            }
        }
    }

    int num_drones_{3};
    int stability_msgs_{15};
    double arm_timeout_s_{15.0};
    double peer_timeout_s_{1.5};
    double minimum_arm_separation_m_{3.0};
    int tick_{0};
    std::vector<int> active_drone_ids_;

    ManagerState mgr_state_;
    std::map<int, DroneRecord> drones_;
    rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_clearance_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_ready_pub_;
    std::map<int, rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr> cmd_pubs_;
    std::vector<rclcpp::Subscription<swarm_interfaces::msg::DroneState>::SharedPtr> subs_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmArmingManager>());
    rclcpp::shutdown();
    return 0;
}
