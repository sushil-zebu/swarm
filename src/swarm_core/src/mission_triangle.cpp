#include "swarm_core/mission_triangle.hpp"

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

struct MissionTriangle::Impl {
    enum class Phase : uint8_t {
        WAIT_FOR_ARM,
        TAKEOFF,
        MOVE_TO_FORMATION,
        HOLD_FORMATION,
        SINE_PATH,
        HOLD_SINE_END,
        RTL
    };

    struct Target {
        float x;
        float y;
        float height_m;
    };

    struct WorldTarget {
        float x;
        float y;
        float z;
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
    bool is_active_drone(uint8_t id) const;
    Target formation_target(uint8_t id) const;
    WorldTarget wave_target(uint8_t id, float progress_m) const;
    bool all_active_drones_at_formation() const;
    bool all_active_drones_at_wave_target(float progress_m) const;
    void enter_phase(Phase next_phase);
    void publish_safe_velocity_to(float target_x, float target_y, float target_z,
                                  float speed, float yaw = -1.5708f);
    bool reached_target(float target_x, float target_y, float target_z,
                        float tolerance) const;
    float world_x() const;
    float world_y() const;
    float world_z() const;
    float world_z_from_height(float height_m) const;
    void publish_swarm_state();

    rclcpp::Node* node_{nullptr};
    PX4Interface* px4_{nullptr};
    PeerTracker* peers_{nullptr};
    CbfAvoidance* cbf_{nullptr};

    uint8_t drone_id_{1};
    int num_drones_{3};
    float spawn_x_{0.0f};
    float spawn_y_{0.0f};
    float spawn_z_{0.0f};

    std::vector<uint8_t> active_drone_ids_{1, 2, 3};
    uint8_t middle_drone_id_{2};
    float takeoff_height_m_{10.0f};
    float side_length_m_{5.0f};
    float center_x_{7.0f};
    float center_y_{0.0f};
    float reference_z_{0.0f};
    bool base_above_apex_{true};
    float max_speed_mps_{3.0f};
    float reach_tol_m_{0.5f};
    float sync_tol_m_{0.8f};
    bool preflight_check_{true};
    bool require_arm_clearance_{false};

    float formation_hold_s_{2.0f};
    float sine_travel_distance_m_{100.0f};
    float sine_wavelength_m_{20.0f};
    float sine_forward_speed_mps_{0.8f};
    float sine_tracking_gain_{1.0f};
    float sine_position_amplitude_m_{5.0f};
    float sine_duration_s_{66.6667f};
    float triangle_height_m_{4.330127f};
    float formation_center_z_{-12.886751f};
    float swap_rotation_direction_{-1.0f};

    Phase phase_{Phase::WAIT_FOR_ARM};
    SwarmState swarm_state_{SwarmState::IDLE};
    uint64_t tick_{0};
    bool preflight_done_{false};
    bool preflight_arm_seen_{false};
    bool arm_clearance_{false};
    bool formation_complete_logged_{false};
    bool formation_sync_active_{false};
    bool rtl_logged_{false};
    bool rtl_started_{false};
    int completed_swap_count_{0};
    rclcpp::Time formation_sync_start_{0, 0, RCL_ROS_TIME};
    rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_clearance_sub_;
};

MissionTriangle::Impl::Impl(
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
                        "[MISSION TRIANGLE] Drone %u received arm clearance.",
                        drone_id_);
                }
            });
        RCLCPP_INFO(node_->get_logger(),
            "[MISSION TRIANGLE] Drone %u waiting for centralized arm clearance.",
            drone_id_);
    } else {
        // Decentralized fleet launch: there is no arming-manager publisher.
        arm_clearance_ = true;
        RCLCPP_INFO(node_->get_logger(),
            "[MISSION TRIANGLE] Drone %u using autonomous mission arming.",
            drone_id_);
    }

    const Target target = formation_target(drone_id_);
    RCLCPP_INFO(node_->get_logger(),
        "[MISSION TRIANGLE] Drone %u target=(%.2f, %.2f, %.2fm AGL)",
        drone_id_, target.x, target.y, target.height_m);
}

void MissionTriangle::Impl::load_parameters() {
    node_->declare_parameter("triangle_middle_drone_id", 2);
    node_->declare_parameter("triangle_takeoff_height_m", 10.0);
    node_->declare_parameter("triangle_side_length_m", 5.0);
    node_->declare_parameter("triangle_center_x", 7.0);
    node_->declare_parameter("triangle_center_y", 0.0);
    node_->declare_parameter("triangle_reference_z", 0.0);
    node_->declare_parameter("triangle_base_above_apex", true);
    node_->declare_parameter("triangle_reach_tol_m", 0.5);
    node_->declare_parameter("triangle_sync_tol_m", 0.8);
    node_->declare_parameter("triangle_formation_hold_s", 2.0);
    node_->declare_parameter("triangle_sine_travel_distance_m", 100.0);
    node_->declare_parameter("triangle_sine_wavelength_m", 20.0);
    node_->declare_parameter("triangle_sine_forward_speed_mps", 0.8);
    node_->declare_parameter("preflight_check", true);
    node_->declare_parameter("require_arm_clearance", false);

    if (num_drones_ != 3) {
        throw std::invalid_argument("MissionTriangle requires num_drones=3");
    }
    active_drone_ids_ = {1, 2, 3};

    middle_drone_id_ = static_cast<uint8_t>(
        node_->get_parameter("triangle_middle_drone_id").as_int());
    takeoff_height_m_ = static_cast<float>(
        node_->get_parameter("triangle_takeoff_height_m").as_double());
    side_length_m_ = static_cast<float>(
        node_->get_parameter("triangle_side_length_m").as_double());
    center_x_ = static_cast<float>(
        node_->get_parameter("triangle_center_x").as_double());
    center_y_ = static_cast<float>(
        node_->get_parameter("triangle_center_y").as_double());
    reference_z_ = static_cast<float>(
        node_->get_parameter("triangle_reference_z").as_double());
    base_above_apex_ =
        node_->get_parameter("triangle_base_above_apex").as_bool();
    reach_tol_m_ = static_cast<float>(
        node_->get_parameter("triangle_reach_tol_m").as_double());
    sync_tol_m_ = std::max(reach_tol_m_, static_cast<float>(
        node_->get_parameter("triangle_sync_tol_m").as_double()));
    formation_hold_s_ = static_cast<float>(
        node_->get_parameter("triangle_formation_hold_s").as_double());
    sine_travel_distance_m_ = static_cast<float>(
        node_->get_parameter("triangle_sine_travel_distance_m").as_double());
    sine_wavelength_m_ = static_cast<float>(
        node_->get_parameter("triangle_sine_wavelength_m").as_double());
    sine_forward_speed_mps_ = static_cast<float>(
        node_->get_parameter("triangle_sine_forward_speed_mps").as_double());
    preflight_check_ = node_->get_parameter("preflight_check").as_bool();
    require_arm_clearance_ =
        node_->get_parameter("require_arm_clearance").as_bool();
    max_speed_mps_ = static_cast<float>(
        node_->get_parameter("max_speed_mps").as_double());
    sine_tracking_gain_ = static_cast<float>(
        node_->get_parameter("goal_gain").as_double());

    if (!is_active_drone(middle_drone_id_)) {
        throw std::invalid_argument(
            "triangle_middle_drone_id must be 1, 2, or 3");
    }
    if (takeoff_height_m_ <= 0.0f || side_length_m_ <= 0.0f) {
        throw std::invalid_argument(
            "Triangle takeoff height and side length must be positive");
    }
    if (formation_hold_s_ < 0.0f || sine_travel_distance_m_ <= 0.0f ||
        sine_wavelength_m_ <= 0.0f || sine_forward_speed_mps_ <= 0.0f ||
        sine_tracking_gain_ <= 0.0f) {
        throw std::invalid_argument(
            "Triangle sine distance, wavelength, speed, and tracking gain must be positive");
    }

    constexpr float kTwoPi = 6.28318530717958647692f;
    sine_position_amplitude_m_ = side_length_m_;
    const float spatial_frequency = kTwoPi / sine_wavelength_m_;
    const float peak_vertical_speed = sine_position_amplitude_m_ *
        spatial_frequency * sine_forward_speed_mps_;
    const float formation_radius = side_length_m_ / std::sqrt(3.0f);
    const float peak_rotation_speed = formation_radius * (4.0f / 3.0f) *
        spatial_frequency * sine_forward_speed_mps_;


    // Vertical sine velocity and the vertical component of formation rotation
    // can point the same way, so their magnitudes are conservatively added.
    const float peak_path_speed = std::hypot(
        sine_forward_speed_mps_,
        peak_vertical_speed + peak_rotation_speed);
    if (peak_path_speed > max_speed_mps_) {
        throw std::invalid_argument(
            "Triangle sine forward/vertical peak speed exceeds max_speed_mps; "
            "reduce triangle_sine_forward_speed_mps or increase wavelength");
    }

    sine_duration_s_ = sine_travel_distance_m_ / sine_forward_speed_mps_;

    triangle_height_m_ = 0.5f * std::sqrt(3.0f) * side_length_m_;
    const float base_height = takeoff_height_m_ +
        (base_above_apex_ ? triangle_height_m_ : -triangle_height_m_);
    if (base_height <= 0.0f) {
        throw std::invalid_argument(
            "Triangle base height would be at/below the altitude reference");
    }
    const float lowest_formation_height = std::min(takeoff_height_m_, base_height);
    if (lowest_formation_height - sine_position_amplitude_m_ <= 0.0f) {
        throw std::invalid_argument(
            "Triangle sine amplitude would command a drone at/below the altitude reference");
    }
    formation_center_z_ = (
        world_z_from_height(takeoff_height_m_) +
        2.0f * world_z_from_height(base_height)) / 3.0f;

    // Select the rotation direction from the configured geometry so the
    // mapping remains 1->2->3 even if triangle_middle_drone_id is changed.
    const Target slot_1 = formation_target(1);
    const Target slot_2 = formation_target(2);
    const float slot_1_dx = slot_1.x - center_x_;
    const float slot_1_dz =
        world_z_from_height(slot_1.height_m) - formation_center_z_;
    const float slot_2_dx = slot_2.x - center_x_;
    const float slot_2_dz =
        world_z_from_height(slot_2.height_m) - formation_center_z_;
    const float cross_1_to_2 = slot_1_dx * slot_2_dz -
        slot_1_dz * slot_2_dx;
    swap_rotation_direction_ = (cross_1_to_2 >= 0.0f) ? 1.0f : -1.0f;
}

bool MissionTriangle::Impl::is_active_drone(uint8_t id) const {
    return std::binary_search(
        active_drone_ids_.begin(), active_drone_ids_.end(), id);
}

MissionTriangle::Impl::Target
MissionTriangle::Impl::formation_target(uint8_t id) const {
    if (id == middle_drone_id_) {
        return {center_x_, center_y_, takeoff_height_m_};
    }

    std::vector<uint8_t> outer_ids;
    outer_ids.reserve(2);
    for (const uint8_t active_id : active_drone_ids_) {
        if (active_id != middle_drone_id_) {
            outer_ids.push_back(active_id);
        }
    }

    const float half_side = 0.5f * side_length_m_;
    const float x = (id == outer_ids.front())
        ? center_x_ - half_side
        : center_x_ + half_side;
    const float height = takeoff_height_m_ +
        (base_above_apex_ ? triangle_height_m_ : -triangle_height_m_);
    return {x, center_y_, height};
}

MissionTriangle::Impl::WorldTarget
MissionTriangle::Impl::wave_target(uint8_t id, float progress_m) const {
    constexpr float kTwoPi = 6.283185307179586f;
    const Target initial = formation_target(id);
    const float initial_z = world_z_from_height(initial.height_m);
    const float spatial_phase =
        kTwoPi * progress_m / sine_wavelength_m_;

    // A signed 120 degree formation rotation per +90 degree sine phase maps
    // slot 1 -> 2, slot 2 -> 3, and slot 3 -> 1. The direction was derived
    // from the configured geometry. Continuous rotation keeps the side length.
    const float rotation_angle = swap_rotation_direction_ *
        (4.0f / 3.0f) * spatial_phase;
    const float cosine = std::cos(rotation_angle);
    const float sine = std::sin(rotation_angle);
    const float initial_dx = initial.x - center_x_;
    const float initial_dz = initial_z - formation_center_z_;
    const float rotated_dx = cosine * initial_dx - sine * initial_dz;
    const float rotated_dz = sine * initial_dx + cosine * initial_dz;
    const float height_offset =
        sine_position_amplitude_m_ * std::sin(spatial_phase);

    return {
        center_x_ + rotated_dx,
        center_y_ + progress_m,
        formation_center_z_ + rotated_dz - height_offset};
}

float MissionTriangle::Impl::world_x() const {
    return px4_->pos_x() + spawn_x_;
}

float MissionTriangle::Impl::world_y() const {
    return px4_->pos_y() + spawn_y_;
}

float MissionTriangle::Impl::world_z() const {
    return px4_->pos_z() + spawn_z_;
}

float MissionTriangle::Impl::world_z_from_height(float height_m) const {
    return reference_z_ - height_m;
}

bool MissionTriangle::Impl::reached_target(
    float target_x, float target_y, float target_z, float tolerance) const
{
    const float dx = world_x() - target_x;
    const float dy = world_y() - target_y;
    const float dz = world_z() - target_z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) < tolerance;
}

void MissionTriangle::Impl::publish_safe_velocity_to(
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

bool MissionTriangle::Impl::all_active_drones_at_formation() const {
    for (const uint8_t id : active_drone_ids_) {
        const Target target = formation_target(id);
        const float target_z = world_z_from_height(target.height_m);

        if (id == drone_id_) {
            if (!reached_target(target.x, target.y, target_z, sync_tol_m_)) {
                return false;
            }
            continue;
        }

        const auto peer_it = peers_->peers().find(id);
        if (peer_it == peers_->peers().end() ||
            !peers_->is_peer_valid(peer_it->second)) {
            return false;
        }

        const PeerState& peer = peer_it->second;
        const float dx = peer.x - target.x;
        const float dy = peer.y - target.y;
        const float dz = peer.z - target_z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) >= sync_tol_m_) {
            return false;
        }
    }
    return true;
}

bool MissionTriangle::Impl::all_active_drones_at_wave_target(
    float progress_m) const
{
    for (const uint8_t id : active_drone_ids_) {
        const WorldTarget target = wave_target(id, progress_m);

        if (id == drone_id_) {
            if (!reached_target(target.x, target.y, target.z, sync_tol_m_)) {
                return false;
            }
            continue;
        }

        const auto peer_it = peers_->peers().find(id);
        if (peer_it == peers_->peers().end() ||
            !peers_->is_peer_valid(peer_it->second)) {
            return false;
        }

        const PeerState& peer = peer_it->second;
        const float dx = peer.x - target.x;
        const float dy = peer.y - target.y;
        const float dz = peer.z - target.z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) >= sync_tol_m_) {
            return false;
        }
    }
    return true;
}

void MissionTriangle::Impl::enter_phase(Phase next_phase) {
    phase_ = next_phase;
    phase_start_ = node_->now();
    formation_sync_active_ = false;

    const char* phase_name = "UNKNOWN";
    switch (next_phase) {
    case Phase::WAIT_FOR_ARM:          phase_name = "WAIT_FOR_ARM"; break;
    case Phase::TAKEOFF:               phase_name = "TAKEOFF"; break;
    case Phase::MOVE_TO_FORMATION:     phase_name = "MOVE_TO_FORMATION"; break;
    case Phase::HOLD_FORMATION:        phase_name = "HOLD_FORMATION"; break;
    case Phase::SINE_PATH:             phase_name = "SINE_PATH"; break;
    case Phase::HOLD_SINE_END:         phase_name = "HOLD_SINE_END"; break;
    case Phase::RTL:                   phase_name = "RTL"; break;
    }

    RCLCPP_INFO(node_->get_logger(),
        "[MISSION TRIANGLE] Drone %u entering %s",
        drone_id_, phase_name);
}

void MissionTriangle::Impl::publish_swarm_state() {
    peers_->check_heartbeats();
    // If PX4 localization disappears, stop refreshing the heartbeat. Peers
    // will then fail closed rather than trusting a frozen position forever.
    if (px4_->has_fresh_local_position() &&
        px4_->has_fresh_vehicle_status()) {
        peers_->broadcast_self_state(
            drone_id_, world_x(), world_y(), world_z(),
            px4_->vel_x(), px4_->vel_y(), px4_->vel_z(),
            px4_->arming_state(), swarm_state_, {});
    } else {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[MISSION TRIANGLE] Local position stale; suppressing swarm heartbeat.");
    }
    peers_->publish_mission_status(
        drone_id_, swarm_state_, 0, {}, world_x(), world_y(), world_z());
}

void MissionTriangle::Impl::control_loop() {
    ++tick_;
    if (phase_ != Phase::RTL) {
        px4_->publish_offboard_control_mode();
    }
    publish_swarm_state();

    const Target own_target = formation_target(drone_id_);
    const float formation_z = world_z_from_height(own_target.height_m);
    const float takeoff_z = world_z_from_height(takeoff_height_m_);

    switch (phase_) {
    case Phase::WAIT_FOR_ARM:
        swarm_state_ = SwarmState::IDLE;
        px4_->publish_velocity_setpoint(0.0f, 0.0f, 0.0f);

        if (preflight_check_ && !preflight_done_) {
            // Optional bench motor check. Wait for actual PX4 state changes so
            // command acknowledgement latency cannot race disarm against re-arm.
            if (tick_ < 10) {
                break;
            }
            if (!preflight_arm_seen_) {
                if (!px4_->is_armed()) {
                    px4_->arm();
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
                "[MISSION TRIANGLE] Drone %u preflight arm check complete.",
                drone_id_);
            break;
        }

        if (arm_clearance_ || px4_->is_armed()) {
            if (!px4_->is_armed() || !px4_->is_in_offboard()) {
                px4_->engage_offboard();
                px4_->arm();
            } else {
                swarm_state_ = SwarmState::TAKEOFF;
                enter_phase(Phase::TAKEOFF);
            }
        }
        break;

    case Phase::TAKEOFF:
        swarm_state_ = SwarmState::TAKEOFF;
        publish_safe_velocity_to(
            spawn_x_, spawn_y_, takeoff_z, max_speed_mps_);
        if (reached_target(spawn_x_, spawn_y_, takeoff_z, reach_tol_m_)) {
            swarm_state_ = SwarmState::SURVEYING;
            enter_phase(Phase::MOVE_TO_FORMATION);
        }
        break;

    case Phase::MOVE_TO_FORMATION:
        swarm_state_ = SwarmState::SURVEYING;
        publish_safe_velocity_to(
            own_target.x, own_target.y, formation_z, max_speed_mps_);
        if (reached_target(
                own_target.x, own_target.y, formation_z, reach_tol_m_)) {
            swarm_state_ = SwarmState::HOLD;
            enter_phase(Phase::HOLD_FORMATION);
        }
        break;

    case Phase::HOLD_FORMATION:
        swarm_state_ = SwarmState::HOLD;
        publish_safe_velocity_to(
            own_target.x, own_target.y, formation_z, max_speed_mps_);

        if (all_active_drones_at_formation()) {
            if (!formation_sync_active_) {
                formation_sync_active_ = true;
                formation_sync_start_ = node_->now();
            }

            if (!formation_complete_logged_) {
                formation_complete_logged_ = true;
                RCLCPP_INFO(node_->get_logger(),
                    "[MISSION TRIANGLE] All drones formed a %.2fm "
                    "equilateral triangle in the X-Z plane.",
                    side_length_m_);
            }

            if ((node_->now() - formation_sync_start_).seconds() >=
                formation_hold_s_) {
                enter_phase(Phase::SINE_PATH);
                RCLCPP_INFO(node_->get_logger(),
                    "[MISSION TRIANGLE] Drone %u starting rigid triangle sine: "
                    "Y distance=%.2fm, altitude amplitude=%.2fm, wavelength=%.2fm, "
                    "forward speed=%.2fm/s, waves=%.2f, swaps=%.0f, duration=%.2fs.",
                    drone_id_, sine_travel_distance_m_,
                    sine_position_amplitude_m_, sine_wavelength_m_,
                    sine_forward_speed_mps_,
                    sine_travel_distance_m_ / sine_wavelength_m_,
                    4.0f * sine_travel_distance_m_ / sine_wavelength_m_,
                    sine_duration_s_);
            }
        } else {
            formation_sync_active_ = false;
        }
        break;

    case Phase::SINE_PATH: {
        swarm_state_ = SwarmState::SURVEYING;
        const float elapsed_s = std::max(0.0f, static_cast<float>(
            (node_->now() - phase_start_).seconds()));

        constexpr float kTwoPi = 6.283185307179586f;
        const float progress_m = std::min(
            sine_travel_distance_m_, sine_forward_speed_mps_ * elapsed_s);
        const float spatial_frequency = kTwoPi / sine_wavelength_m_;
        const float phase = spatial_frequency * progress_m;
        const int swap_count = static_cast<int>(std::floor(
            (phase + 1.0e-4f) / (0.25f * kTwoPi)));
        if (swap_count > completed_swap_count_) {
            completed_swap_count_ = swap_count;
            const uint8_t occupied_slot = static_cast<uint8_t>(
                ((static_cast<int>(drone_id_) - 1 + swap_count) % 3) + 1);
            RCLCPP_INFO(node_->get_logger(),
                "[MISSION TRIANGLE] Drone %u completed swap %d at Y=%.2fm; "
                "now occupying moving slot %u.",
                drone_id_, swap_count, center_y_ + progress_m, occupied_slot);
        }

        if (elapsed_s >= sine_duration_s_) {
            enter_phase(Phase::HOLD_SINE_END);
            break;
        }

        const float rotation_rate = swap_rotation_direction_ * (4.0f / 3.0f) *
            spatial_frequency * sine_forward_speed_mps_;
        const WorldTarget desired = wave_target(drone_id_, progress_m);
        const float height_offset =
            sine_position_amplitude_m_ * std::sin(phase);
        const float rotated_dx = desired.x - center_x_;
        const float rotated_dz =
            desired.z - (formation_center_z_ - height_offset);
        const float feedforward_x = -rotation_rate * rotated_dz;
        const float sine_feedforward_z = -sine_position_amplitude_m_ *
            spatial_frequency * sine_forward_speed_mps_ * std::cos(phase);
        const float feedforward_z =
            rotation_rate * rotated_dx + sine_feedforward_z;

        // The translated, vertically oscillating triangle also rotates in its
        // X-Z plane. Continuous rotation completes one cyclic slot swap every
        // 90 degrees of sine phase without shrinking pairwise separation.
        const Vec3f nominal_velocity{
            feedforward_x + sine_tracking_gain_ * (desired.x - world_x()),
            sine_forward_speed_mps_ +
                sine_tracking_gain_ * (desired.y - world_y()),
            feedforward_z + sine_tracking_gain_ * (desired.z - world_z())};
        const Vec3f safe_velocity = cbf_->filter_velocity(
            nominal_velocity,
            world_x(), world_y(), world_z(),
            px4_->vel_x(), px4_->vel_y(), px4_->vel_z(),
            *peers_);
        px4_->publish_velocity_setpoint(
            safe_velocity.x, safe_velocity.y, safe_velocity.z);
        break;
    }

    case Phase::HOLD_SINE_END: {
        swarm_state_ = SwarmState::HOLD;
        const WorldTarget target = wave_target(
            drone_id_, sine_travel_distance_m_);
        publish_safe_velocity_to(
            target.x, target.y, target.z, max_speed_mps_);

        if (all_active_drones_at_wave_target(sine_travel_distance_m_)) {
            if (!formation_sync_active_) {
                formation_sync_active_ = true;
                formation_sync_start_ = node_->now();
            } else if ((node_->now() - formation_sync_start_).seconds() >=
                       formation_hold_s_) {
                swarm_state_ = SwarmState::RETURN;
                enter_phase(Phase::RTL);
            }
        } else {
            formation_sync_active_ = false;
        }
        break;
    }

    case Phase::RTL:
        swarm_state_ = SwarmState::RETURN;
        if (!rtl_logged_) {
            rtl_logged_ = true;
            RCLCPP_INFO(node_->get_logger(),
                "[MISSION TRIANGLE] Drone %u returning safely above home before RTL.",
                drone_id_);
        }

        if (!rtl_started_) {
            px4_->publish_offboard_control_mode();
            publish_safe_velocity_to(
                spawn_x_, spawn_y_, formation_z, max_speed_mps_);
            if (reached_target(spawn_x_, spawn_y_, formation_z, 1.0f)) {
                rtl_started_ = true;
                RCLCPP_INFO(node_->get_logger(),
                    "[MISSION TRIANGLE] Drone %u is above home; engaging PX4 RTL.",
                    drone_id_);
                px4_->rtl();
            }
        } else if (tick_ % 10 == 0) {
            px4_->rtl();
        }
        break;
    }
}

MissionTriangle::MissionTriangle(
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

MissionTriangle::~MissionTriangle() = default;

void MissionTriangle::control_loop() {
    impl_->control_loop();
}

}
