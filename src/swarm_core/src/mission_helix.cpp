#include "swarm_core/mission_helix.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "swarm_core/cbf_avoidance.hpp"
#include "swarm_core/peer_tracker.hpp"
#include "swarm_core/px4_interface.hpp"
#include "swarm_core/swarm_types.hpp"

namespace swarm {

struct MissionHelix::Impl {
    enum class Phase : uint8_t {
        TAKEOFF,
        CONVERGE_TO_CENTER,
        WAIT_AT_CENTER,
        MOVE_TO_CIRCLE_START,
        WAIT_AT_CIRCLE_START,
        ROTATE_ONE_REVOLUTION,
        CLIMB_HELIX
    };

    Impl(rclcpp::Node& node,
         PX4Interface& px4,
         PeerTracker& peers,
         CbfAvoidance& cbf,
         uint8_t drone_id,
         int num_drones,
         float spawn_x,
         float spawn_y,
         float spawn_z);

    void load_parameters();
    void control_loop();
    void publish_safe_velocity_to(float target_x, float target_y, float target_z,
                                  float speed, float yaw = -1.5708f);
    bool reached_target(float target_x, float target_y, float target_z,
                        float tolerance) const;
    float world_x() const;
    float world_y() const;
    float world_z() const;
    float cruise_world_z(uint8_t id) const;
    float maximum_formation_climb() const;
    bool all_drones_at_formation_target(float target_x, float target_y,
                                        float climb_distance,
                                        float tolerance) const;
    void enter_phase(Phase next_phase);

    rclcpp::Node* node_{nullptr};
    PX4Interface* px4_{nullptr};
    PeerTracker* peers_{nullptr};
    CbfAvoidance* cbf_{nullptr};

    uint8_t drone_id_{1};
    int num_drones_{4};
    float spawn_x_{0.0f};
    float spawn_y_{0.0f};
    float spawn_z_{0.0f};

    float max_speed_mps_{3.0f};
    float reach_tol_{0.5f};
    bool preflight_check_{true};
    bool require_arm_clearance_{false};
    float stack_reference_z_{0.0f};
    std::vector<double> cruise_altitudes_m_{5.0, 10.0, 15.0, 20.0};
    float helix_center_x_{0.0f};
    float helix_center_y_{0.0f};
    float helix_radius_m_{3.0f};
    float helix_angular_speed_rad_s_{0.5f};
    float helix_climb_accel_mps2_{0.5f};
    float helix_climb_max_vel_mps_{1.0f};
    float helix_max_height_m_{30.0f};
    float formation_sync_hold_s_{2.0f};
    float formation_sync_tol_m_{0.8f};

    SwarmState state_{SwarmState::IDLE};
    Phase phase_{Phase::TAKEOFF};
    uint64_t tick_{0};
    bool preflight_done_{false};
    bool preflight_arm_seen_{false};
    bool arm_clearance_{false};
    rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};
    rclcpp::Time sync_start_{0, 0, RCL_ROS_TIME};
    bool sync_active_{false};

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_clearance_sub_;
};

MissionHelix::Impl::Impl(
    rclcpp::Node& node,
    PX4Interface& px4,
    PeerTracker& peers,
    CbfAvoidance& cbf,
    uint8_t drone_id,
    int num_drones,
    float spawn_x,
    float spawn_y,
    float spawn_z)
    : node_(&node),
      px4_(&px4),
      peers_(&peers),
      cbf_(&cbf),
      drone_id_(drone_id),
      num_drones_(num_drones),
      spawn_x_(spawn_x),
      spawn_y_(spawn_y),
      spawn_z_(spawn_z)
{
    load_parameters();

    if (require_arm_clearance_) {
        const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
        arm_clearance_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/swarm/arm_clearance", latched_qos,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                arm_clearance_ = msg->data;
                if (arm_clearance_) {
                    RCLCPP_INFO(node_->get_logger(),
                        "[MISSION HELIX] Global arm clearance received.");
                }
            });
    } else {
        arm_clearance_ = true;
    }

    RCLCPP_INFO(node_->get_logger(),
        "[MISSION HELIX] Drone %u cruise_z=%.1f center=(%.1f, %.1f) "
        "radius=%.1f omega=%.2f ceiling=%.1fm AGL",
        drone_id_, cruise_world_z(drone_id_), helix_center_x_, helix_center_y_,
        helix_radius_m_, helix_angular_speed_rad_s_, helix_max_height_m_);
}

void MissionHelix::Impl::load_parameters() {
    node_->declare_parameter("reach_tol", 0.5);
    node_->declare_parameter("preflight_check", true);
    node_->declare_parameter("require_arm_clearance", false);
    node_->declare_parameter("stack_reference_z", 0.0);
    node_->declare_parameter<std::vector<double>>(
        "cruise_altitudes_m", {5.0, 10.0, 15.0, 20.0});
    node_->declare_parameter("helix_center_x", 0.0);
    node_->declare_parameter("helix_center_y", 0.0);
    node_->declare_parameter("helix_radius_m", 3.0);
    node_->declare_parameter("helix_angular_speed_rad_s", 0.5);
    node_->declare_parameter("helix_climb_accel_mps2", 0.5);
    node_->declare_parameter("helix_climb_max_vel_mps", 1.0);
    node_->declare_parameter("helix_max_height_m", 30.0);
    node_->declare_parameter("formation_sync_hold_s", 2.0);
    node_->declare_parameter("formation_sync_tol_m", 0.8);

    max_speed_mps_ = static_cast<float>(
        node_->get_parameter("max_speed_mps").as_double());
    reach_tol_ = static_cast<float>(node_->get_parameter("reach_tol").as_double());
    preflight_check_ = node_->get_parameter("preflight_check").as_bool();
    require_arm_clearance_ =
        node_->get_parameter("require_arm_clearance").as_bool();
    stack_reference_z_ = static_cast<float>(
        node_->get_parameter("stack_reference_z").as_double());
    cruise_altitudes_m_ =
        node_->get_parameter("cruise_altitudes_m").as_double_array();
    helix_center_x_ = static_cast<float>(
        node_->get_parameter("helix_center_x").as_double());
    helix_center_y_ = static_cast<float>(
        node_->get_parameter("helix_center_y").as_double());
    helix_radius_m_ = std::max(0.1f, static_cast<float>(
        node_->get_parameter("helix_radius_m").as_double()));
    helix_angular_speed_rad_s_ = std::max(0.01f, static_cast<float>(
        node_->get_parameter("helix_angular_speed_rad_s").as_double()));
    helix_climb_accel_mps2_ = std::max(0.01f, static_cast<float>(
        node_->get_parameter("helix_climb_accel_mps2").as_double()));
    helix_climb_max_vel_mps_ = std::max(0.01f, static_cast<float>(
        node_->get_parameter("helix_climb_max_vel_mps").as_double()));
    helix_max_height_m_ = std::max(0.0f, static_cast<float>(
        node_->get_parameter("helix_max_height_m").as_double()));
    formation_sync_hold_s_ = std::max(0.0f, static_cast<float>(
        node_->get_parameter("formation_sync_hold_s").as_double()));
    formation_sync_tol_m_ = std::max(reach_tol_, static_cast<float>(
        node_->get_parameter("formation_sync_tol_m").as_double()));

    if (num_drones_ < 1 || drone_id_ < 1 || drone_id_ > num_drones_ ||
        cruise_altitudes_m_.size() < static_cast<size_t>(num_drones_)) {
        RCLCPP_FATAL(node_->get_logger(),
            "Invalid helix fleet configuration: drone_id=%u num_drones=%d altitude_count=%zu",
            drone_id_, num_drones_, cruise_altitudes_m_.size());
        throw std::invalid_argument(
            "cruise_altitudes_m must contain one entry per drone and drone_id must be in range");
    }

    for (int id = 0; id < num_drones_; ++id) {
        if (cruise_altitudes_m_[static_cast<size_t>(id)] <= 0.0) {
            throw std::invalid_argument("Every cruise_altitudes_m entry must be positive");
        }
    }

    const float tangential_speed = helix_radius_m_ * helix_angular_speed_rad_s_;
    const float combined_helix_speed = std::hypot(
        tangential_speed, helix_climb_max_vel_mps_);
    if (combined_helix_speed > max_speed_mps_) {
        RCLCPP_WARN(node_->get_logger(),
            "Helix path requests %.2f m/s but max_speed_mps is %.2f; tracking will be rate-limited.",
            combined_helix_speed, max_speed_mps_);
    }

    const auto highest_cruise_it = std::max_element(
        cruise_altitudes_m_.begin(),
        cruise_altitudes_m_.begin() + num_drones_);
    if (helix_max_height_m_ <= static_cast<float>(*highest_cruise_it)) {
        RCLCPP_WARN(node_->get_logger(),
            "helix_max_height_m=%.1f is not above the highest cruise altitude %.1f; "
            "helix climb is zero.",
            helix_max_height_m_, static_cast<float>(*highest_cruise_it));
    }
}

float MissionHelix::Impl::world_x() const {
    return px4_->pos_x() + spawn_x_;
}

float MissionHelix::Impl::world_y() const {
    return px4_->pos_y() + spawn_y_;
}

float MissionHelix::Impl::world_z() const {
    return px4_->pos_z() + spawn_z_;
}

void MissionHelix::Impl::publish_safe_velocity_to(
    float target_x, float target_y, float target_z, float speed, float yaw)
{
    const Vec3f safe_velocity = cbf_->compute_safe_velocity(
        target_x, target_y, target_z, speed,
        world_x(), world_y(), world_z(),
        px4_->vel_x(), px4_->vel_y(), px4_->vel_z(),
        *peers_);

    px4_->publish_velocity_setpoint(
        safe_velocity.x, safe_velocity.y, safe_velocity.z, yaw);
}

bool MissionHelix::Impl::reached_target(
    float target_x, float target_y, float target_z, float tolerance) const
{
    const float dx = world_x() - target_x;
    const float dy = world_y() - target_y;
    const float dz = world_z() - target_z;
    return std::sqrt(dx*dx + dy*dy + dz*dz) < tolerance;
}

float MissionHelix::Impl::cruise_world_z(uint8_t id) const {
    if (id == 0 || static_cast<size_t>(id) > cruise_altitudes_m_.size()) {
        return stack_reference_z_;
    }
    return stack_reference_z_ -
        static_cast<float>(cruise_altitudes_m_[static_cast<size_t>(id - 1)]);
}

float MissionHelix::Impl::maximum_formation_climb() const {
    const auto highest_cruise_it = std::max_element(
        cruise_altitudes_m_.begin(),
        cruise_altitudes_m_.begin() + num_drones_);
    return std::max(
        0.0f, helix_max_height_m_ - static_cast<float>(*highest_cruise_it));
}

bool MissionHelix::Impl::all_drones_at_formation_target(
    float target_x, float target_y, float climb_distance, float tolerance) const
{
    const float own_target_z = cruise_world_z(drone_id_) - climb_distance;
    if (!reached_target(target_x, target_y, own_target_z, tolerance)) {
        return false;
    }

    for (int id = 1; id <= num_drones_; ++id) {
        if (id == static_cast<int>(drone_id_)) continue;

        const auto peer_it = peers_->peers().find(static_cast<uint8_t>(id));
        if (peer_it == peers_->peers().end() ||
            !peers_->is_peer_valid(peer_it->second)) {
            return false;
        }

        const auto& peer = peer_it->second;

        const float dx = peer.x - target_x;
        const float dy = peer.y - target_y;
        const float dz = peer.z -
            (cruise_world_z(static_cast<uint8_t>(id)) - climb_distance);
        if (std::sqrt(dx*dx + dy*dy + dz*dz) >= tolerance) {
            return false;
        }
    }
    return true;
}

void MissionHelix::Impl::enter_phase(Phase next_phase) {
    phase_ = next_phase;
    phase_start_ = node_->now();
    sync_active_ = false;

    const char* phase_name = "UNKNOWN";
    switch (next_phase) {
    case Phase::TAKEOFF:               phase_name = "TAKEOFF"; break;
    case Phase::CONVERGE_TO_CENTER:    phase_name = "CONVERGE_TO_CENTER"; break;
    case Phase::WAIT_AT_CENTER:        phase_name = "WAIT_AT_CENTER"; break;
    case Phase::MOVE_TO_CIRCLE_START:  phase_name = "MOVE_TO_CIRCLE_START"; break;
    case Phase::WAIT_AT_CIRCLE_START:  phase_name = "WAIT_AT_CIRCLE_START"; break;
    case Phase::ROTATE_ONE_REVOLUTION: phase_name = "ROTATE_ONE_REVOLUTION"; break;
    case Phase::CLIMB_HELIX:           phase_name = "CLIMB_HELIX"; break;
    }
    RCLCPP_INFO(node_->get_logger(),
        "[MISSION HELIX] Drone %u entering %s", drone_id_, phase_name);
}

void MissionHelix::Impl::control_loop() {
    ++tick_;

    px4_->publish_offboard_control_mode();
    peers_->check_heartbeats();
    if (px4_->has_fresh_local_position() &&
        px4_->has_fresh_vehicle_status()) {
        peers_->broadcast_self_state(
            drone_id_, world_x(), world_y(), world_z(),
            px4_->vel_x(), px4_->vel_y(), px4_->vel_z(),
            px4_->arming_state(), state_, {});
    } else {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[MISSION HELIX] Local position stale; suppressing swarm heartbeat.");
    }
    peers_->publish_mission_status(
        drone_id_, state_, 0, {}, world_x(), world_y(), world_z());

    const float own_cruise_z = cruise_world_z(drone_id_);
    const float circle_start_x = helix_center_x_ + helix_radius_m_;
    const float circle_start_y = helix_center_y_;
    constexpr float kTwoPi = 6.28318530718f;

    switch (state_) {
    case SwarmState::IDLE:
        px4_->publish_velocity_setpoint(0.0f, 0.0f, 0.0f);

        if (preflight_check_ && !preflight_done_) {
            if (tick_ < 10) {
                break;
            }
            if (!preflight_arm_seen_) {
                if (!px4_->is_armed()) {
                    px4_->arm();
                    RCLCPP_INFO_THROTTLE(
                        node_->get_logger(), *node_->get_clock(), 1000,
                        "[PREFLIGHT] Drone %u motor arm check... World: (%.2f, %.2f, %.2f)",
                        drone_id_, world_x(), world_y(), world_z());
                    break;
                }
                preflight_arm_seen_ = true;
            }
            if (!px4_->is_disarmed()) {
                px4_->disarm();
                break;
            }
            preflight_done_ = true;
            RCLCPP_INFO(node_->get_logger(),
                "[PREFLIGHT] Drone %u arm check PASSED! World: (%.2f, %.2f, %.2f)",
                drone_id_, world_x(), world_y(), world_z());

            for (const auto& [peer_id, peer] : peers_->peers()) {
                if (peers_->is_peer_valid(peer)) {
                    const float dx = world_x() - peer.x;
                    const float dy = world_y() - peer.y;
                    const float dz = world_z() - peer.z;
                    const float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                    RCLCPP_INFO(node_->get_logger(),
                        "[PREFLIGHT]   -> Peer %u at (%.2f, %.2f, %.2f) dist=%.2fm",
                        peer_id, peer.x, peer.y, peer.z, distance);
                }
            }
            break;
        }

        if (arm_clearance_ || px4_->is_armed()) {
            if (!px4_->is_armed() || !px4_->is_in_offboard()) {
                px4_->engage_offboard();
                px4_->arm();
            } else {
                RCLCPP_INFO(node_->get_logger(),
                    "[MISSION HELIX] Drone %u ARMED & OFFBOARD — initiating takeoff.",
                    drone_id_);
                enter_phase(Phase::TAKEOFF);
                state_ = SwarmState::TAKEOFF;
            }
        }
        break;

    case SwarmState::TAKEOFF:
        publish_safe_velocity_to(spawn_x_, spawn_y_, own_cruise_z, max_speed_mps_);
        if (reached_target(spawn_x_, spawn_y_, own_cruise_z, reach_tol_)) {
            RCLCPP_INFO(node_->get_logger(),
                "[MISSION HELIX] Drone %u reached fixed cruise Z %.1f m.",
                drone_id_, own_cruise_z);
            enter_phase(Phase::CONVERGE_TO_CENTER);
            state_ = SwarmState::HOLD;
        }
        break;

    case SwarmState::HOLD:
        publish_safe_velocity_to(
            helix_center_x_, helix_center_y_, own_cruise_z, max_speed_mps_);

        if (phase_ == Phase::CONVERGE_TO_CENTER &&
            reached_target(helix_center_x_, helix_center_y_, own_cruise_z, reach_tol_)) {
            enter_phase(Phase::WAIT_AT_CENTER);
        }

        if (phase_ == Phase::WAIT_AT_CENTER) {
            if (all_drones_at_formation_target(
                    helix_center_x_, helix_center_y_, 0.0f, formation_sync_tol_m_)) {
                if (!sync_active_) {
                    sync_active_ = true;
                    sync_start_ = node_->now();
                    RCLCPP_INFO(node_->get_logger(),
                        "[MISSION HELIX] Drone %u sees the vertical column assembled.",
                        drone_id_);
                } else if ((node_->now() - sync_start_).seconds() >=
                           formation_sync_hold_s_) {
                    enter_phase(Phase::MOVE_TO_CIRCLE_START);
                    state_ = SwarmState::SURVEYING;
                }
            } else {
                sync_active_ = false;
            }
        }
        break;

    case SwarmState::SURVEYING:
        switch (phase_) {
        case Phase::MOVE_TO_CIRCLE_START:
            publish_safe_velocity_to(
                circle_start_x, circle_start_y, own_cruise_z, max_speed_mps_);
            if (reached_target(
                    circle_start_x, circle_start_y, own_cruise_z, reach_tol_)) {
                enter_phase(Phase::WAIT_AT_CIRCLE_START);
            }
            break;

        case Phase::WAIT_AT_CIRCLE_START:
            publish_safe_velocity_to(
                circle_start_x, circle_start_y, own_cruise_z, max_speed_mps_);
            if (all_drones_at_formation_target(
                    circle_start_x, circle_start_y, 0.0f, formation_sync_tol_m_)) {
                if (!sync_active_) {
                    sync_active_ = true;
                    sync_start_ = node_->now();
                } else if ((node_->now() - sync_start_).seconds() >=
                           formation_sync_hold_s_) {
                    enter_phase(Phase::ROTATE_ONE_REVOLUTION);
                }
            } else {
                sync_active_ = false;
            }
            break;

        case Phase::ROTATE_ONE_REVOLUTION:
            {
                const float revolution_time =
                    kTwoPi / helix_angular_speed_rad_s_;
                const float elapsed = std::max(0.0f, static_cast<float>(
                    (node_->now() - phase_start_).seconds()));
                const float path_time = std::min(elapsed, revolution_time);
                const float angle = helix_angular_speed_rad_s_ * path_time;
                const float target_x =
                    helix_center_x_ + helix_radius_m_ * std::cos(angle);
                const float target_y =
                    helix_center_y_ + helix_radius_m_ * std::sin(angle);
                const float tangential_speed =
                    helix_radius_m_ * helix_angular_speed_rad_s_;

                publish_safe_velocity_to(
                    target_x, target_y, own_cruise_z, tangential_speed);

                if (elapsed >= revolution_time &&
                    all_drones_at_formation_target(
                        circle_start_x, circle_start_y, 0.0f,
                        formation_sync_tol_m_)) {
                    if (!sync_active_) {
                        sync_active_ = true;
                        sync_start_ = node_->now();
                    } else if ((node_->now() - sync_start_).seconds() >=
                               formation_sync_hold_s_) {
                        enter_phase(Phase::CLIMB_HELIX);
                    }
                } else {
                    sync_active_ = false;
                }
            }
            break;

        case Phase::CLIMB_HELIX:
            {
                const float max_climb = maximum_formation_climb();
                const float ramp_time =
                    helix_climb_max_vel_mps_ / helix_climb_accel_mps2_;
                const float ramp_distance =
                    0.5f * helix_climb_accel_mps2_ * ramp_time * ramp_time;

                float climb_end_time = 0.0f;
                if (max_climb <= ramp_distance) {
                    climb_end_time = std::sqrt(
                        2.0f * max_climb / helix_climb_accel_mps2_);
                } else {
                    climb_end_time = ramp_time +
                        (max_climb - ramp_distance) / helix_climb_max_vel_mps_;
                }

                const float elapsed = std::max(0.0f, static_cast<float>(
                    (node_->now() - phase_start_).seconds()));
                const float path_time = std::min(elapsed, climb_end_time);

                float climbed = 0.0f;
                float climb_speed = 0.0f;
                if (path_time <= ramp_time) {
                    climbed =
                        0.5f * helix_climb_accel_mps2_ * path_time * path_time;
                    climb_speed = helix_climb_accel_mps2_ * path_time;
                } else {
                    climbed = ramp_distance +
                        helix_climb_max_vel_mps_ * (path_time - ramp_time);
                    climb_speed = helix_climb_max_vel_mps_;
                }
                climbed = std::min(climbed, max_climb);

                const float angle =
                    kTwoPi + helix_angular_speed_rad_s_ * path_time;
                const float target_x =
                    helix_center_x_ + helix_radius_m_ * std::cos(angle);
                const float target_y =
                    helix_center_y_ + helix_radius_m_ * std::sin(angle);
                const float target_z = own_cruise_z - climbed;
                const float tangential_speed =
                    helix_radius_m_ * helix_angular_speed_rad_s_;
                const float path_speed = std::max(
                    0.1f, std::hypot(tangential_speed, climb_speed));

                publish_safe_velocity_to(target_x, target_y, target_z, path_speed);

                if (elapsed >= climb_end_time &&
                    all_drones_at_formation_target(
                        target_x, target_y, max_climb, formation_sync_tol_m_)) {
                    if (!sync_active_) {
                        sync_active_ = true;
                        sync_start_ = node_->now();
                    } else if ((node_->now() - sync_start_).seconds() >=
                               formation_sync_hold_s_) {
                        RCLCPP_INFO(node_->get_logger(),
                            "[MISSION HELIX] Drone %u completed helix; "
                            "top-tier ceiling is %.1f m AGL.",
                            drone_id_, helix_max_height_m_);
                        state_ = SwarmState::RETURN;
                        sync_active_ = false;
                    }
                } else {
                    sync_active_ = false;
                }
            }
            break;

        default:
            enter_phase(Phase::MOVE_TO_CIRCLE_START);
            break;
        }
        break;

    case SwarmState::RETURN:
        {
            const float home_z = own_cruise_z;
            publish_safe_velocity_to(spawn_x_, spawn_y_, home_z, max_speed_mps_);

            if (reached_target(spawn_x_, spawn_y_, home_z, 1.0f)) {
                RCLCPP_INFO(node_->get_logger(),
                    "[MISSION HELIX] Drone %u returned above home (%.2f, %.2f); landing.",
                    drone_id_, spawn_x_, spawn_y_);
                state_ = SwarmState::LAND;
            }
        }
        break;

    case SwarmState::LAND:
        if (!px4_->is_in_auto_land()) {
            px4_->land();
        }
        if (px4_->is_disarmed()) {
            RCLCPP_INFO(node_->get_logger(),
                "[MISSION HELIX] Drone %u landed and disarmed safely.", drone_id_);
            state_ = SwarmState::DONE;
        }
        break;

    case SwarmState::DONE:
        break;
    }
}

MissionHelix::MissionHelix(
    rclcpp::Node& node,
    PX4Interface& px4,
    PeerTracker& peers,
    CbfAvoidance& cbf,
    uint8_t drone_id,
    int num_drones,
    float spawn_x,
    float spawn_y,
    float spawn_z)
    : impl_(std::make_unique<Impl>(
          node, px4, peers, cbf, drone_id, num_drones,
          spawn_x, spawn_y, spawn_z))
{
}

MissionHelix::~MissionHelix() = default;

void MissionHelix::control_loop() {
    impl_->control_loop();
}

} // namespace swarm
