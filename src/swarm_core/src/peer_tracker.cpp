#include "swarm_core/peer_tracker.hpp"
#include <cmath>
#include <limits>

namespace swarm {

void PeerTracker::setup(rclcpp::Node& node, uint8_t drone_id, int num_drones, double peer_timeout_s) {
    node_ = &node;
    drone_id_ = drone_id;
    peer_timeout_s_ = peer_timeout_s;

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    swarm_pub_ = node_->create_publisher<swarm_interfaces::msg::DroneState>(
        "swarm/self_state", state_qos);

    mission_status_pub_ = node_->create_publisher<swarm_interfaces::msg::SwarmMissionStatus>(
        "swarm/mission_status", status_qos);

    for (int p_id = 1; p_id <= num_drones; ++p_id) {
        if (p_id == drone_id_) continue;
        peers_[static_cast<uint8_t>(p_id)] = PeerState{};

        std::string topic = "/drone_" + std::to_string(p_id) + "/swarm/self_state";
        auto sub = node_->create_subscription<swarm_interfaces::msg::DroneState>(
            topic, state_qos,
            [this, p_id](swarm_interfaces::msg::DroneState::SharedPtr msg) {
                on_peer_state(static_cast<uint8_t>(p_id), msg);
            });
        peer_subs_.push_back(sub);
        RCLCPP_INFO(node_->get_logger(), "[SWARM CORE] Subscribed to peer topic: %s", topic.c_str());
    }

    // will create subscription of mission state
}


//
bool PeerTracker::is_peer_valid(const PeerState& peer) const {
    if (!node_ || !peer.ever_seen || !peer.alive ||
        peer.last_seen.nanoseconds() <= 0) {
        return false;
    }

    const bool state_is_finite =
        std::isfinite(peer.x) && std::isfinite(peer.y) && std::isfinite(peer.z) &&
        std::isfinite(peer.vx) && std::isfinite(peer.vy) && std::isfinite(peer.vz);
    return state_is_finite && peer_age_s(peer) <= peer_timeout_s_;
}

// checking all Peer valid 
bool PeerTracker::all_peers_valid() const {
    for (const auto& [id, peer] : peers_) {
        (void)id;
        if (!is_peer_valid(peer)) return false;
    }
    return true;
}

double PeerTracker::peer_age_s(const PeerState& peer) const {
    if (!node_ || peer.last_seen.nanoseconds() <= 0) {
        return std::numeric_limits<double>::infinity();
    }

    const auto now = node_->now();
    const double receive_age = std::max(0.0, (now - peer.last_seen).seconds());
    if (peer.state_stamp.nanoseconds() <= 0) return receive_age;

    const double source_age = std::max(0.0, (now - peer.state_stamp).seconds());
    return std::max(receive_age, source_age);
}

void PeerTracker::on_peer_state(uint8_t peer_id, const swarm_interfaces::msg::DroneState::SharedPtr msg) {
    if (peers_.count(peer_id) == 0) return;

    if (msg->drone_id != peer_id) {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[SWARM SAFETY] Topic for drone %u carried drone_id=%u; rejecting state.",
            peer_id, msg->drone_id);
        return;
    }

    const bool finite =
        std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->z) &&
        std::isfinite(msg->vx) && std::isfinite(msg->vy) && std::isfinite(msg->vz);
    if (!finite) {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[SWARM SAFETY] Drone %u published non-finite state; rejecting it.", peer_id);
        return;
    }

    auto& p = peers_[peer_id];
    
    p.x = msg->x;  p.y = msg->y;  p.z = msg->z;

    p.vx = msg->vx; p.vy = msg->vy; p.vz = msg->vz;

    // p.altitude = msg->altitude;
    p.arming_state = msg->arming_state;
    p.swarm_state = msg->swarm_state;
    p.completed_task_ids = msg->completed_task_ids;
    p.alive = true;
    p.state_stamp = rclcpp::Time(msg->stamp);

    
    if (!p.ever_seen) {
        p.ever_seen = true;
        p.first_seen = node_->now();
        RCLCPP_INFO(node_->get_logger(), "[SWARM MESH] First contact with Peer Drone %u at World Coords: (%.2f, %.2f, %.2f)!",
            peer_id, p.x, p.y, p.z);
    }
    p.last_seen = node_->now();
}

void PeerTracker::check_heartbeats() {
    for (auto& [id, peer] : peers_) {
        if (!peer.ever_seen || !peer.alive) continue;
        const double elapsed = peer_age_s(peer);
        if (elapsed > peer_timeout_s_) {
            RCLCPP_WARN(node_->get_logger(),
                "[FAULT HANDLER] Peer Drone %u SILENT for %.1fs — marking OFFLINE!", id, elapsed);
            peer.alive = false;
        }
    }
}

// self state publish
void PeerTracker::broadcast_self_state(uint8_t drone_id,
    float world_x, float world_y, float world_z,
    float vel_x, float vel_y, float vel_z,
    uint8_t arming_state, SwarmState state,
    const std::vector<uint32_t>& completed_task_ids)
{
    swarm_interfaces::msg::DroneState msg;
    msg.drone_id           = drone_id;
    msg.x                  = world_x;
    msg.y                  = world_y;
    msg.z                  = world_z;
    msg.vx                 = vel_x;
    msg.vy                 = vel_y;
    msg.vz                 = vel_z;
    msg.arming_state       = arming_state;
    // msg.altitude           = altitude;
    msg.swarm_state        = static_cast<uint8_t>(state);
    msg.completed_task_ids = completed_task_ids;
    msg.stamp              = node_->now();
    swarm_pub_->publish(msg);
}

// mission status 
void PeerTracker::publish_mission_status(uint8_t drone_id, SwarmState state,
    size_t current_wp_idx,
    const std::vector<swarm_interfaces::msg::WaypointItem>& waypoints,
    float world_x, float world_y, float world_z)
{
    swarm_interfaces::msg::SwarmMissionStatus msg;
    msg.drone_id              = drone_id;
    msg.swarm_state           = static_cast<uint8_t>(state);
    msg.current_waypoint_idx  = static_cast<uint32_t>(current_wp_idx);
    msg.total_waypoints       = static_cast<uint32_t>(waypoints.size());

            if (!waypoints.empty() && current_wp_idx < waypoints.size()) {
        const auto& wp = waypoints[current_wp_idx];
        float dx = world_x - wp.x;
        float dy = world_y - wp.y;
        float dz = world_z - wp.z;
        msg.distance_to_target = std::sqrt(dx*dx + dy*dy + dz*dz);
        msg.progress_pct = (static_cast<float>(current_wp_idx) / static_cast<float>(waypoints.size())) * 100.0f;
    } else {
        msg.distance_to_target = 0.0f;
        msg.progress_pct = (state == SwarmState::DONE) ? 100.0f : 0.0f;

    msg.status_text = to_str(state);
    msg.stamp       = node_->now();
    mission_status_pub_->publish(msg);
}

}
}
/*
        if (!waypoints.empty() && current_wp_idx < waypoints.size()) {
        const auto& wp = waypoints[current_wp_idx];
        float dx = world_x - wp.x;
        float dy = world_y - wp.y;
        float dz = world_z - wp.z;
        msg.distance_to_target = std::sqrt(dx*dx + dy*dy + dz*dz);
        msg.progress_pct = (static_cast<float>(current_wp_idx) / static_cast<float>(waypoints.size())) * 100.0f;
    } else {
        msg.distance_to_target = 0.0f;
        msg.progress_pct = (state == SwarmState::DONE) ? 100.0f : 0.0f;
    } */
