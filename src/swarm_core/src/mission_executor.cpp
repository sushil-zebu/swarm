#include "swarm_core/mission_executor.hpp"
#include <cmath>

namespace swarm {

void MissionExecutor::setup(rclcpp::Node& node, uint8_t drone_id,
                            float cruise_alt, float spawn_z, float reach_tol) {
    node_ = &node;
    drone_id_ = drone_id;
    cruise_alt_ = cruise_alt;
    spawn_z_ = spawn_z;
    reach_tol_ = reach_tol;
}

void MissionExecutor::on_waypoint_sequence(
    const swarm_interfaces::msg::WaypointSequence::SharedPtr msg)
{
    // Filter by target drone ID (0 means broadcast to all)
    if (msg->drone_id != 0 && msg->drone_id != drone_id_) return;

    mission_name_ = msg->mission_name.empty() ? "Dynamic Mission" : msg->mission_name;
    waypoints_ = msg->waypoints;
    active_claimed_tasks_.clear();
    current_wp_idx_ = 0;
    wp_holding_ = false;
    mission_mode_ = true;  // Activate MISSION MODE (bypasses task allocation)

    RCLCPP_INFO(node_->get_logger(),
        "[SWARM CORE] Received WaypointSequence '%s' with %zu waypoints (mission_mode=ON).",
        mission_name_.c_str(), waypoints_.size());
}

void MissionExecutor::load_task_waypoints(
    const std::vector<swarm_interfaces::msg::WaypointItem>& waypoints,
    const std::vector<swarm_interfaces::msg::TaskItem>& active_tasks)
{
    waypoints_ = waypoints;
    active_claimed_tasks_ = active_tasks;
    // In task mode, the task list is rebuilt each tick, so we evaluate the first waypoint (index 0)
    current_wp_idx_ = 0;
}

void MissionExecutor::generate_helical_mission(
    float radius, float angular_speed, float climb_accel, float climb_max_vel,
    float own_hover_alt, float max_alt,
    float offset_to_drone1_x, float offset_to_drone1_y)
{
    waypoints_.clear();
    active_claimed_tasks_.clear();
    current_wp_idx_ = 0;
    wp_holding_ = false;
    mission_mode_ = true;
    mission_name_ = "Helical Formation";

    // Drone 1's ground coordinate, expressed in THIS drone's own local frame.
    // Since each drone's PX4 local origin is wherever IT spawned, and drones
    // spawn 7m apart, this offset is what lets every drone converge to the
    // exact same world point even though their (0,0) origins differ.
    float target_x = -offset_to_drone1_x;
    float target_y = -offset_to_drone1_y;

    float dt = 0.2f;  // waypoint spacing, matches original function's resolution

    // ---------- Phase 1: CONVERGE ----------
    // Single waypoint: move horizontally onto Drone 1's XY while 
    // (or descending) to THIS drone's own final hover altitude. reached_current()
    // already checks 3D distance, so one waypoint does both moves at once.
    {
        swarm_interfaces::msg::WaypointItem wp;
        wp.x = target_x;
        wp.y = target_y;
        wp.z = -own_hover_alt;   // NED: negative = up
        wp.speed = angular_speed * radius;  // reuse tangential speed as transit speed
        wp.hold_time_s = 0.0f;
        waypoints_.push_back(wp);
    }

    // ---------- Phase 2: ROTATE — exactly one full revolution ----------
    // NO phase_offset here — every drone computes the same angle at the same
    // waypoint index, so all 4 line up vertically and turn together instead
    // of spreading around the ring like the original 3-drone version did.
    {
        float omega = angular_speed;   // rad/s
        float total_angle = 2.0f * 3.14159265f;  // exactly one revolution
        float t = 0.0f;
        while (omega * t <= total_angle) {
            float angle = omega * t;   // starts at 0 — ring's "3 o'clock" point
            swarm_interfaces::msg::WaypointItem wp;
            wp.x = target_x + radius * std::cos(angle);
            wp.y = target_y + radius * std::sin(angle);
            wp.z = -own_hover_alt;           // constant altitude during pure rotation
            wp.speed = omega * radius;       // tangential speed
            wp.hold_time_s = 0.0f;
            waypoints_.push_back(wp);
            t += dt;
        }
    }

    // ---------- Phase 3: CLIMB_HELIX — rotation continues, altitude ramps up ----------
    // Velocity ramps 0 -> climb_max_vel at rate climb_accel, then holds constant,
    // same acceleration model discussed for the live-tick version — just baked
    // into the pre-generated waypoint list here instead of computed per-tick.
    {
        float omega = angular_speed;
        float t_ramp = climb_max_vel / climb_accel;  // time to reach max climb speed
        float t = 0.0f;

        while (true) {
            float climbed;
            if (t <= t_ramp) {
                climbed = 0.5f * climb_accel * t * t;             // accelerating
            } else {
                float climbed_during_ramp = 0.5f * climb_accel * t_ramp * t_ramp;
                climbed = climbed_during_ramp + climb_max_vel * (t - t_ramp); // constant vel
            }

            float z_world = -own_hover_alt - climbed;  // more negative = higher

            if (z_world <= max_alt) break;   // stop once past the ceiling (max_alt is negative)

            // Angle keeps advancing continuously from where ROTATE left off
            // (2*pi radians already completed), so the spin never stutters.
            float angle = omega * (2.0f * 3.14159265f / omega + t);
            swarm_interfaces::msg::WaypointItem wp;
            wp.x = target_x + radius * std::cos(angle);
            wp.y = target_y + radius * std::sin(angle);
            wp.z = z_world;
            wp.speed = omega * radius;
            wp.hold_time_s = 0.0f;
            waypoints_.push_back(wp);

            t += dt;
        }
    }

    RCLCPP_INFO(node_->get_logger(),
        "[MISSION EXECUTOR] Drone %u generated helical mission: %zu waypoints "
        "(hover_alt=%.1fm, climb ceiling=%.1fm).",
        drone_id_, waypoints_.size(), own_hover_alt, max_alt);
}

float MissionExecutor::target_x() const {
    if (is_complete()) return 0.0f;
    return waypoints_[current_wp_idx_].x;
}

float MissionExecutor::target_y() const {
    if (is_complete()) return 0.0f;
    return waypoints_[current_wp_idx_].y;
}

float MissionExecutor::target_z() const {
    if (is_complete()) return cruise_alt_ + spawn_z_;
    float z = waypoints_[current_wp_idx_].z;
    // Fall back to default cruise altitude if z is unspecified (0.0)
    return (z != 0.0f) ? z : cruise_alt_ + spawn_z_;
}

float MissionExecutor::target_speed(float default_speed) const {
    if (is_complete()) return default_speed;
    float s = waypoints_[current_wp_idx_].speed;
    return (s > 0.0f) ? s : default_speed;
}

float MissionExecutor::target_hold_time() const {
    if (is_complete()) return 0.0f;
    return waypoints_[current_wp_idx_].hold_time_s;
}

uint32_t MissionExecutor::current_task_id() const {
    if (current_wp_idx_ < active_claimed_tasks_.size()) {
        return active_claimed_tasks_[current_wp_idx_].task_id;
    }
    return 0;
}

bool MissionExecutor::reached_current(float world_x, float world_y, float world_z) const {
    if (is_complete()) return false;
    float dx = world_x - target_x();
    float dy = world_y - target_y();
    float dz = world_z - target_z();
    return std::sqrt(dx*dx + dy*dy + dz*dz) < reach_tol_;
}

void MissionExecutor::start_hold(rclcpp::Time now) {
    wp_holding_ = true;
    wp_reached_time_ = now;
}

bool MissionExecutor::hold_complete(rclcpp::Time now) const {
    if (!wp_holding_) return false;
    return (now - wp_reached_time_).seconds() >= target_hold_time();
}

void MissionExecutor::advance_mission() {
    wp_holding_ = false;
    current_wp_idx_++;
}

void MissionExecutor::reset_task_index() {
    wp_holding_ = false;
    current_wp_idx_ = 0;
}

void MissionExecutor::finish_mission() {
    mission_mode_ = false;
    RCLCPP_INFO(node_->get_logger(),
        "[MISSION COMPLETE] All %zu waypoints completed! Drone %u mission finished.",
        waypoints_.size(), drone_id_);
}

void MissionExecutor::clear() {
    waypoints_.clear();
    active_claimed_tasks_.clear();
    current_wp_idx_ = 0;
    wp_holding_ = false;
    mission_mode_ = false;
    mission_name_ = "None";
}

} // namespace swarm
