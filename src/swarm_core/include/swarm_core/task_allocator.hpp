/*
 * task_allocator.hpp — Decentralized Task Allocation Engine
 *
 * ╔═══════════════════════════════════════════════════════════════════╗
 * ║  WHAT THIS FILE DOES                                            ║
 * ║                                                                 ║
 * ║  Decides WHICH tasks each drone should do — WITHOUT any central ║
 * ║  coordinator! Each drone independently runs the same algorithm  ║
 * ║  and arrives at the same conclusion about who does what.        ║
 * ║                                                                 ║
 * ║  HOW IT WORKS (Zero-Negotiation Proximity Bidding)              ║
 * ║  1. Receive a shared list of tasks from /swarm/task_list        ║
 * ║  2. Remove tasks already completed (by self or any peer)        ║
 * ║  3. For each remaining task, compute distance from self & peers ║
 * ║  4. The CLOSEST drone wins the task (bid)                       ║
 * ║  5. If two drones are equally close, use round-robin tiebreak   ║
 * ║  6. Convert won tasks into waypoints for the MissionExecutor    ║
 * ║                                                                 ║
 * ║  WHY IT'S DECENTRALIZED                                        ║
 * ║  Every drone sees the same task list and peer positions, so     ║
 * ║  every drone computes the same answer. No negotiation needed!   ║
 * ║                                                                 ║
 * ║  USAGE                                                          ║
 * ║    tasks_.setup(*this, drone_id, cruise_alt, spawn_z);          ║
 * ║    tasks_.on_task_list(msg);                                    ║
 * ║    auto result = tasks_.evaluate(wx, wy, wz, peers);           ║
 * ╚═══════════════════════════════════════════════════════════════════╝
 */
#pragma once

#include <vector>
#include <set>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <swarm_interfaces/msg/task_item.hpp>
#include <swarm_interfaces/msg/task_list.hpp>
#include <swarm_interfaces/msg/waypoint_item.hpp>

#include "swarm_core/swarm_types.hpp"
#include "swarm_core/peer_tracker.hpp"

namespace swarm {

class TaskAllocator {
public:
    // ── Setup ─────────────────────────────────────────────────────────
    void setup(rclcpp::Node& node, uint8_t drone_id,
               float cruise_alt, float spawn_z);

    // ── Receive shared task list from /swarm/task_list ─────────────────
    void on_task_list(const swarm_interfaces::msg::TaskList::SharedPtr msg);

    // ── Result of the allocation algorithm ────────────────────────────
    struct AllocationResult {
        std::vector<swarm_interfaces::msg::WaypointItem> waypoints;    // Claimed waypoints (nearest first)
        std::vector<swarm_interfaces::msg::TaskItem>     active_tasks; // Corresponding task items
        bool all_done{false};  // True if ALL tasks in the shared list are completed
    };

    // ── Run the bidding algorithm ─────────────────────────────────────
    // Call this every tick. Returns the waypoints this drone should fly.
    //
    // Inputs:
    //   world_x/y/z : this drone's position in shared world frame
    //   peers        : reference to PeerTracker for peer positions
    //
    AllocationResult evaluate(float world_x, float world_y, float world_z,
                              const PeerTracker& peers);

    // ── Task completion tracking ──────────────────────────────────────
    // Mark a task as completed by this drone. Broadcasts to peers
    // via the completed_task_ids vector in the drone state message.
    void mark_completed(uint32_t task_id);

    // Check if there are any unassigned tasks in the shared list.
    bool has_tasks() const { return !unassigned_tasks_.empty(); }

    // Get the list of task IDs completed by this drone.
    const std::vector<uint32_t>& completed_task_ids() const { return completed_task_ids_; }

private:
    rclcpp::Node* node_{nullptr};
    uint8_t drone_id_{0};
    float   cruise_alt_{-5.0f};
    float   spawn_z_{0.0f};

    // The shared task list received from /swarm/task_list
    std::vector<swarm_interfaces::msg::TaskItem> unassigned_tasks_;

    // Tasks completed by THIS drone
    std::vector<uint32_t> completed_task_ids_;
};

} // namespace swarm
