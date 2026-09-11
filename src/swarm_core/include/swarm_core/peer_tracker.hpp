/*
 * peer_tracker.hpp — Peer Drone Heartbeat Monitoring & State Broadcasting
 *
 * ╔═══════════════════════════════════════════════════════════════════╗
 * ║  WHAT THIS FILE DOES                                            ║
 * ║                                                                 ║
 * ║  Manages communication with other drones in the swarm:          ║
 * ║  • Receives position & velocity updates from peer drones        ║
 * ║  • Detects when a peer goes offline (heartbeat timeout)         ║
 * ║  • Broadcasts this drone's own state to all peers               ║
 * ║  • Publishes mission status for monitoring dashboards           ║
 * ║                                                                 ║
 * ║  KEY CONCEPT: Shared World Frame                                ║
 * ║  Each drone's PX4 reports positions relative to ITS launch      ║
 * ║  point. To compare across drones, we convert to a shared world  ║
 * ║  frame using spawn offsets:                                     ║
 * ║    world_position = local_position + spawn_offset               ║
 * ║  All peer broadcasts use this shared world frame.               ║
 * ║                                                                 ║
 * ║  USAGE                                                          ║
 * ║    peers_.setup(*this, drone_id, num_drones, timeout);          ║
 * ║    peers_.check_heartbeats();     // call every tick             ║
 * ║    peers_.broadcast_self_state(...);                             ║
 * ║    auto& peer_map = peers_.peers();                             ║
 * ╚═══════════════════════════════════════════════════════════════════╝
 */
#pragma once

#include <map>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <swarm_interfaces/msg/drone_state.hpp>
#include <swarm_interfaces/msg/swarm_mission_status.hpp>
#include <swarm_interfaces/msg/waypoint_item.hpp>

#include "swarm_core/swarm_types.hpp"

namespace swarm {

class PeerTracker {
public:
    // ── Setup (call once during node init) ─────────────────────────────
    // Creates peer subscriptions and broadcast publishers.
    // Subscribes to /drone_{i}/swarm/self_state for every peer drone.
    void setup(rclcpp::Node& node, uint8_t drone_id,
               int num_drones, double peer_timeout_s);

    // ── Called Every Tick (10 Hz) ──────────────────────────────────────

    // Check if any peer has gone silent longer than the timeout.
    // Silent peers are marked OFFLINE so the CBF and task allocator
    // don't use stale data.
    void check_heartbeats();

    // Broadcast this drone's state to all peers via /swarm/self_state.
    void broadcast_self_state(uint8_t drone_id,
                              float world_x, float world_y, float world_z,
                              float vel_x, float vel_y, float vel_z,
                              uint8_t arming_state, SwarmState state,
                              const std::vector<uint32_t>& completed_task_ids);

    // Publish mission progress for monitoring dashboards.
    void publish_mission_status(uint8_t drone_id, SwarmState state,
                                size_t current_wp_idx,
                                const std::vector<swarm_interfaces::msg::WaypointItem>& waypoints,
                                float world_x, float world_y, float world_z);

    // ── Peer State Access ─────────────────────────────────────────────

    // Check if a peer's data is finite, current, and from a live connection.
    bool is_peer_valid(const PeerState& peer) const;

    // True only when every configured peer has a current, valid state.  Motion
    // controllers use this to fail closed instead of flying with a blind spot.
    bool all_peers_valid() const;

    // Age of the oldest evidence for this state (receive time or source stamp).
    double peer_age_s(const PeerState& peer) const;

    // Read-only access to the full peer state map.
    // Key = drone_id, Value = PeerState (position, velocity, alive status).
    const std::map<uint8_t, PeerState>& peers() const { return peers_; }

private:
    // ── Subscription Callback ─────────────────────────────────────────
    // Called when a peer drone publishes its state.
    void on_peer_state(uint8_t peer_id,
                       const swarm_interfaces::msg::DroneState::SharedPtr msg);

    rclcpp::Node* node_{nullptr};
    uint8_t drone_id_{0};
    double  peer_timeout_s_{1.5};

    // Map of peer_id → PeerState (position, velocity, alive flag, etc.)
    std::map<uint8_t, PeerState> peers_;

    // Publishers
    rclcpp::Publisher<swarm_interfaces::msg::DroneState>::SharedPtr          swarm_pub_;
    rclcpp::Publisher<swarm_interfaces::msg::SwarmMissionStatus>::SharedPtr  mission_status_pub_;

    // One subscriber per peer drone
    std::vector<rclcpp::Subscription<swarm_interfaces::msg::DroneState>::SharedPtr> peer_subs_;
};

} // namespace swarm
