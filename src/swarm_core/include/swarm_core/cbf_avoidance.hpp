#pragma once

#include <rclcpp/rclcpp.hpp>
#include "swarm_core/swarm_types.hpp"
#include "swarm_core/peer_tracker.hpp"

namespace swarm {

// ── Configuration for the CBF collision avoidance ─────────────────────
// These are loaded from ROS2 parameters and passed in at setup time.
struct CbfConfig {
    float safe_radius{2.5f};          // Minimum center-to-center distance [m]
    float position_uncertainty{0.5f}; // GPS/sensor error allowance [m]
    float control_delay_s{0.15f};     // Control + network delay [s]
    float max_speed_mps{3.0f};        // Maximum allowed speed [m/s]
    float max_brake_mps2{2.0f};       // Maximum braking deceleration [m/s²]
    float goal_gain{1.0f};            // How aggressively to chase target [1/s]
    float cbf_alpha{1.5f};            // Barrier response gain [1/s]
    float peer_accel_uncertainty_mps2{2.0f}; // Unmodelled peer acceleration bound [m/s²]
    float constraint_tolerance_mps{0.02f};   // Numerical feasibility tolerance [m/s]
    int projection_iterations{80};           // Bounded convex solver iterations
    bool fail_closed_on_peer_loss{true};     // Hover when any expected peer is stale/missing
};

class CbfAvoidance {
public:
    // ── Setup ─────────────────────────────────────────────────────────
    void setup(rclcpp::Node& node, uint8_t drone_id, const CbfConfig& config);

    // ── Main Function: Compute Collision-Free Velocity ────────────────
    //
    // Inputs (all in SHARED WORLD FRAME):
    //   target_x/y/z     : where the drone wants to go
    //   requested_speed   : desired flight speed [m/s]
    //   self_wx/wy/wz     : drone's current world position
    //   self_vx/vy/vz     : drone's current velocity
    //   peers             : reference to PeerTracker (for peer positions)
    //
    // Returns:
    //   Safe velocity vector (may differ from desired if peers are nearby)
    //
    Vec3f compute_safe_velocity(
        float target_x, float target_y, float target_z,
        float requested_speed,
        float self_wx, float self_wy, float self_wz,
        float self_vx, float self_vy, float self_vz,
        const PeerTracker& peers);

    // Apply the same safety filter to a direct velocity command.  Missions
    // with velocity profiles (for example a sine wave) must use this instead
    // of publishing directly to PX4.
    Vec3f filter_velocity(
        const Vec3f& nominal_velocity,
        float self_wx, float self_wy, float self_wz,
        float self_vx, float self_vy, float self_vz,
        const PeerTracker& peers);

private:
    rclcpp::Node* node_{nullptr};
    uint8_t drone_id_{0};
    CbfConfig config_;
};

} // namespace swarm
