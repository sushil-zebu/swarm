/*
 * px4_interface.hpp — PX4 Flight Controller Interface
 *
 * ╔═══════════════════════════════════════════════════════════════════╗
 * ║  WHAT THIS FILE DOES                                            ║
 * ║                                                                 ║
 * ║  Handles ALL communication with the PX4 autopilot:              ║
 * ║  • Reads drone position, velocity, and arming status from PX4   ║
 * ║  • Sends velocity/position setpoints for offboard control       ║
 * ║  • Sends flight commands: arm, disarm, land, RTL, offboard      ║
 * ║                                                                 ║
 * ║  COORDINATE SYSTEM (NED = North-East-Down)                      ║
 * ║  • X = North, Y = East, Z = Down                                ║
 * ║  • Negative Z = above ground (z = -5.0 → 5 meters altitude)    ║
 * ║                                                                 ║
 * ║  USAGE                                                          ║
 * ║    px4_.setup(*this, mav_sys_id);                               ║
 * ║    px4_.arm();                                                  ║
 * ║    px4_.publish_velocity_setpoint(vx, vy, vz);                  ║
 * ╚═══════════════════════════════════════════════════════════════════╝
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <array>
#include <cstdint>
#include <unordered_map>

namespace swarm {

class PX4Interface {
public:
    enum class OffboardMode {
        Velocity,
        Position
    };

    // ── Setup (call once during node init) ─────────────────────────────
    // Creates all PX4 publishers and subscribers on the given ROS2 node.
    void setup(rclcpp::Node& node, int mav_sys_id);

    // ── Vehicle State (read-only, updated by PX4 subscriptions) ───────
    // Positions are in LOCAL NED frame (relative to this drone's launch point).
    float pos_x() const { return pos_x_; }
    float pos_y() const { return pos_y_; }
    float pos_z() const { return pos_z_; }
    float vel_x() const { return vel_x_; }
    float vel_y() const { return vel_y_; }
    float vel_z() const { return vel_z_; }

    // Raw PX4 state values. Prefer the fresh-state helpers below in control logic.
    uint8_t arming_state() const { return arming_state_; }
    uint8_t nav_state() const { return nav_state_; }

    // Collision avoidance is unsafe without current self position/velocity.
    bool has_fresh_local_position(double max_age_s = 0.5) const;
    bool has_fresh_vehicle_status(double max_age_s = -1.0) const;
    bool is_armed() const;
    bool is_disarmed() const;
    bool is_in_offboard() const;
    bool is_in_auto_land() const;
    bool is_in_auto_rtl() const;

    // ── Flight Commands ───────────────────────────────────────────────
    void arm();              // Arm motors (start spinning)
    void disarm();           // Disarm motors (stop spinning)
    void engage_offboard();  // Switch PX4 to offboard flight mode
    void land();             // Initiate autonomous landing
    void rtl();              // Return-to-launch

    // ── Setpoint Publishing ───────────────────────────────────────────
    // publish_offboard_control_mode() must be called at ≥2 Hz to keep
    // PX4 in offboard mode (PX4 requirement).
    void publish_offboard_control_mode(
        OffboardMode mode = OffboardMode::Velocity);

    // Send velocity command in NED frame (m/s).
    // yaw: heading in radians (default -90° = facing west).
    void publish_velocity_setpoint(float vx, float vy, float vz,
                                   float yaw = -1.5708f);

    // Send a fixed position/waypoint command in NED frame (metres). This also
    // publishes the matching position-mode heartbeat.
    void publish_position_setpoint(float x, float y, float z,
                                   float yaw = -1.5708f);

private:
    struct CommandRequestState {
        bool initialized{false};
        bool awaiting_ack{false};
        std::array<float, 3> params{};
        rclcpp::Time last_sent{0, 0, RCL_ROS_TIME};
        uint32_t attempts{0};
    };

    // Internal: send a PX4 command and track its acknowledgement.
    void send_vehicle_command(uint32_t command,
                              float param1 = 0.0f,
                              float param2 = 0.0f,
                              float param7 = 0.0f);
    void handle_vehicle_command_ack(
        const px4_msgs::msg::VehicleCommandAck::SharedPtr& msg);
    std::array<float, 3> limit_velocity(
        float vx, float vy, float vz, const rclcpp::Time& now);

    rclcpp::Node* node_{nullptr};   // Pointer to the parent ROS2 node
    int mav_sys_id_{2};             // MAVLink target system ID

    // Vehicle state (written by subscription callbacks)
    float   pos_x_{0.0f}, pos_y_{0.0f}, pos_z_{0.0f};
    float   vel_x_{0.0f}, vel_y_{0.0f}, vel_z_{0.0f};
    uint8_t arming_state_{1};   // 1 = DISARMED
    uint8_t nav_state_{0};
    bool local_position_valid_{false};
    rclcpp::Time last_local_position_{0, 0, RCL_ROS_TIME};
    bool vehicle_status_valid_{false};
    rclcpp::Time last_vehicle_status_{0, 0, RCL_ROS_TIME};

    // Final safety envelope. Mission and CBF limits should normally be lower.
    float max_velocity_mps_{7.0f};
    float max_acceleration_mps2_{3.0f};
    double vehicle_status_timeout_s_{1.0};
    double command_ack_timeout_s_{0.75};
    double command_retry_interval_s_{1.0};
    bool last_velocity_setpoint_valid_{false};
    std::array<float, 3> last_velocity_setpoint_{};
    rclcpp::Time last_velocity_setpoint_time_{0, 0, RCL_ROS_TIME};
    std::unordered_map<uint32_t, CommandRequestState> command_requests_;

    // ROS2 Publishers → PX4
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      command_pub_;

    // ROS2 Subscribers ← PX4
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr        status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr    command_ack_sub_;
};

} // namespace swarm
