/*
 * mission_executor.hpp — Mission & Waypoint Sequencing Engine
 *
 * ╔═══════════════════════════════════════════════════════════════════╗
 * ║  WHAT THIS FILE DOES                                            ║
 * ║                                                                 ║
 * ║  Manages the waypoint list that the drone is currently flying:  ║
 * ║  • Tracks which waypoint the drone is heading toward            ║
 * ║  • Handles hold timers at each waypoint                         ║
 * ║  • Advances to the next waypoint when done                      ║
 * ║  • Supports two input modes:                                    ║
 * ║    1. MISSION MODE: Direct WaypointSequence (e.g., helix path)  ║
 * ║    2. TASK MODE: Waypoints from the TaskAllocator               ║
 * ║  • Hosts the ROS2 Action Server for external mission control    ║
 * ║                                                                 ║
 * ║  TWO MODES OF OPERATION                                         ║
 * ║                                                                 ║
 * ║  Mission Mode (mission_mode_ = true):                           ║
 * ║    - Triggered by WaypointSequence messages                     ║
 * ║    - Waypoints advance sequentially (idx++)                     ║
 * ║    - Task allocator is bypassed                                 ║
 * ║    - Used for helical paths, formation flying, etc.             ║
 * ║                                                                 ║
 * ║  Task Mode (mission_mode_ = false):                             ║
 * ║    - Waypoints come from TaskAllocator each tick                ║
 * ║    - Index resets to 0 after each completion                    ║
 * ║    - Task allocator rebuilds the list each tick                 ║
 * ║                                                                 ║
 * ║  USAGE                                                          ║
 * ║    mission_.setup(*this, drone_id, cruise_alt, spawn_z, tol);   ║
 * ║    mission_.on_waypoint_sequence(msg);   // mission mode        ║
 * ║    mission_.load_task_waypoints(wps, tasks); // task mode       ║
 * ║    if (mission_.reached_current(wx, wy, wz)) ...               ║
 * ╚═══════════════════════════════════════════════════════════════════╝
 */
#pragma once

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <swarm_interfaces/msg/waypoint_item.hpp>
#include <swarm_interfaces/msg/waypoint_sequence.hpp>
#include <swarm_interfaces/msg/task_item.hpp>
#include <swarm_interfaces/action/execute_swarm_mission.hpp>

#include "swarm_core/swarm_types.hpp"

namespace swarm {

class MissionExecutor {
public:
    using ExecuteMissionAction  = swarm_interfaces::action::ExecuteSwarmMission;
    using GoalHandleExecMission = rclcpp_action::ServerGoalHandle<ExecuteMissionAction>;

    // ── Setup ─────────────────────────────────────────────────────────
    void setup(rclcpp::Node& node, uint8_t drone_id,
               float cruise_alt, float spawn_z, float reach_tol);

    // Optionally enable the ROS2 Action Server (currently disabled).
    // void setup_action_server(rclcpp::Node& node);

    // ═════════════════════════════════════════════════════════════════
    //  WAYPOINT INPUT
    // ═════════════════════════════════════════════════════════════════

    // Called when a WaypointSequence message is received.
    // Activates MISSION MODE (task allocator is bypassed).
    void on_waypoint_sequence(const swarm_interfaces::msg::WaypointSequence::SharedPtr msg);

    // Called by the main controller to feed task-allocated waypoints.
    // Does NOT activate mission mode (task allocator stays active).
    void load_task_waypoints(
        const std::vector<swarm_interfaces::msg::WaypointItem>& waypoints,
        const std::vector<swarm_interfaces::msg::TaskItem>& active_tasks);

    // Internally generates helical waypoints in C++ for this drone (no external python scripts).
    void generate_helical_mission(
        float radius, float angular_speed, float climb_accel, float climb_max_vel,
        float own_hover_alt, float max_alt,
        float offset_to_drone1_x, float offset_to_drone1_y
    );

    // ═════════════════════════════════════════════════════════════════
    //  STATE QUERIES
    // ═════════════════════════════════════════════════════════════════

    bool is_mission_mode() const { return mission_mode_; }
    bool has_waypoints()   const { return !waypoints_.empty(); }
    bool is_complete()     const { return waypoints_.empty() || current_wp_idx_ >= waypoints_.size(); }

    size_t current_index()   const { return current_wp_idx_; }
    size_t total_waypoints() const { return waypoints_.size(); }

    const std::string& mission_name() const { return mission_name_; }

    const std::vector<swarm_interfaces::msg::WaypointItem>& waypoints() const { return waypoints_; }

    // ═════════════════════════════════════════════════════════════════
    //  CURRENT TARGET (in World Frame)
    // ════════════════════════════════════════                                                ═════════════════════════

    // Get the world-frame target for the current waypoint.
    // If waypoint z is 0, defaults to cruise_alt + spawn_z.
    float target_x() const;
    float target_y() const;
    float target_z() const;

    // Get the speed for the current waypoint (falls back to default).
    float target_speed(float default_speed) const;

    // Get the hold time at the current waypoint (seconds).
    float target_hold_time() const;

    // Get the task_id of the current active task (0 if no task).
    uint32_t current_task_id() const;

    // The active tasks list (tasks claimed by this drone, matching waypoints_).
    const std::vector<swarm_interfaces::msg::TaskItem>& active_tasks() const {
        return active_claimed_tasks_;
    }

    // ═════════════════════════════════════════════════════════════════
    //  WAYPOINT REACHED & ADVANCEMENT
    // ═════════════════════════════════════════════════════════════════

    // Check if the drone has reached the current waypoint.
    bool reached_current(float world_x, float world_y, float world_z) const;

    // Hold management: hold position at waypoint for hold_time_s seconds.
    bool is_holding() const { return wp_holding_; }
    void start_hold(rclcpp::Time now);
    bool hold_complete(rclcpp::Time now) const;
    void clear_hold() { wp_holding_ = false; }

    // Mission mode: advance to the next waypoint in sequence.
    void advance_mission();

    // Task mode: reset index to 0 (task allocator rebuilds list each tick).
    void reset_task_index();

    // Called when mission is fully complete — clears mission_mode flag.
    void finish_mission();

    // Clear all waypoints and reset state.
    void clear();

private:
    rclcpp::Node* node_{nullptr};
    uint8_t drone_id_{0};
    float   cruise_alt_{-5.0f};
    float   spawn_z_{0.0f};
    float   reach_tol_{0.5f};

    // Mission mode flag: true = direct waypoint mission, false = task allocation
    bool mission_mode_{false};

    // Mission / waypoint state
    std::string mission_name_{"None"};
    std::vector<swarm_interfaces::msg::WaypointItem> waypoints_;
    std::vector<swarm_interfaces::msg::TaskItem>     active_claimed_tasks_;
    size_t current_wp_idx_{0};

    // Hold timer state
    bool         wp_holding_{false};
    rclcpp::Time wp_reached_time_{0, 0, RCL_ROS_TIME};

    // Action Server (currently disabled — uncomment to enable)
    // rclcpp_action::Server<ExecuteMissionAction>::SharedPtr action_server_;
};

} // namespace swarm
