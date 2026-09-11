/*
 * task_allocator.cpp — Decentralized Task Allocation Engine
 *
 * Implements the TaskAllocator class defined in task_allocator.hpp.
 *
 * ── HOW THE ALGORITHM WORKS ──────────────────────────────────────────
 *
 * 1. Filter out already-completed tasks:
 *    We build a set of all completed task IDs. This includes task IDs completed
 *    by this drone (`completed_task_ids_`) as well as tasks completed by any
 *    valid peer tracker peer (`completed_task_ids`).
 *
 * 2. Find closest drone for each remaining task (Proximity Bidding):
 *    For every remaining task, we measure the distance from our own position
 *    and compare it to the distance from every other active peer drone.
 *    If our distance is shorter, we bid on and claim the task.
 *
 * 3. Tie-breaker rule (fair distribution):
 *    If two drones are roughly at the same distance (e.g. within 0.5m), we break
 *    the tie using a deterministic round-robin assignment based on the index
 *    of the task modulo the number of active drones, matching our sorted rank.
 *
 * 4. Sorting:
 *    Once we claim our subset of tasks, we sort them by distance (nearest first)
 *    so the drone naturally flies to the closest target first.
 */

#include "swarm_core/task_allocator.hpp"
#include <cmath>
#include <set>
#include <algorithm>

namespace swarm {

void TaskAllocator::setup(rclcpp::Node& node, uint8_t drone_id, float cruise_alt, float spawn_z) {
    node_ = &node;
    drone_id_ = drone_id;
    cruise_alt_ = cruise_alt;
    spawn_z_ = spawn_z;
}

void TaskAllocator::on_task_list(const swarm_interfaces::msg::TaskList::SharedPtr msg) {
    unassigned_tasks_ = msg->tasks;
    RCLCPP_INFO(node_->get_logger(),
        "[TASK ALLOCATOR] Received shared TaskList '%s' with %zu unassigned tasks.",
        msg->mission_name.c_str(), unassigned_tasks_.size());
}

void TaskAllocator::mark_completed(uint32_t task_id) {
    if (std::find(completed_task_ids_.begin(), completed_task_ids_.end(), task_id) == completed_task_ids_.end()) {
        completed_task_ids_.push_back(task_id);
        RCLCPP_INFO(node_->get_logger(),
            "[TASK ENGINE] Task %u COMPLETED by Drone %u! Broadcasting completion.",
            task_id, drone_id_);
    }
}

TaskAllocator::AllocationResult TaskAllocator::evaluate(
    float world_x, float world_y, float world_z,
    const PeerTracker& peers)
{
    AllocationResult result;
    if (unassigned_tasks_.empty()) return result;

    // ── STEP 1: Gather Completed Tasks (Self + Peers) ─────────────────
    std::set<uint32_t> completed_set;
    for (uint32_t tid : completed_task_ids_) {
        completed_set.insert(tid);
    }
    for (const auto& [id, peer] : peers.peers()) {
        if (peers.is_peer_valid(peer)) {
            for (uint32_t tid : peer.completed_task_ids) {
                completed_set.insert(tid);
            }
        }
    }

    // ── STEP 2: Filter Active Remaining Tasks ──────────────────────────
    std::vector<swarm_interfaces::msg::TaskItem> remaining_tasks;
    for (const auto& task : unassigned_tasks_) {
        if (completed_set.count(task.task_id) == 0) {
            remaining_tasks.push_back(task);
        }
    }

    if (remaining_tasks.empty()) {
        result.all_done = true;
        return result;
    }

    // ── STEP 3: Get Active Swarm Size and Rank for Tie-Breaker ─────────
    std::vector<uint8_t> active_ids = {drone_id_};
    for (const auto& [pid, p] : peers.peers()) {
        if (peers.is_peer_valid(p)) active_ids.push_back(pid);
    }
    std::sort(active_ids.begin(), active_ids.end());
    auto my_it = std::find(active_ids.begin(), active_ids.end(), drone_id_);
    size_t my_rank = (my_it != active_ids.end()) ? std::distance(active_ids.begin(), my_it) : 0;

    // ── STEP 4: Proximity Bidding ─────────────────────────────────────
    std::vector<swarm_interfaces::msg::TaskItem> claimed_by_self;

    for (size_t task_idx = 0; task_idx < remaining_tasks.size(); ++task_idx) {
        const auto& task = remaining_tasks[task_idx];
        
        // Calculate distance from self to task location
        float dx = world_x - task.x;
        float dy = world_y - task.y;
        float dz = world_z - task.z;
        float self_dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        bool self_wins = true;

        for (const auto& [peer_id, peer] : peers.peers()) {
            if (!peers.is_peer_valid(peer)) continue;

            // Distance from peer to task location
            float pdx = peer.x - task.x;
            float pdy = peer.y - task.y;
            float pdz = peer.z - task.z;
            float peer_dist = std::sqrt(pdx*pdx + pdy*pdy + pdz*pdz);

            // Proximity check
            if (peer_dist < self_dist - 0.5f) {
                self_wins = false;
                break;
            } else if (std::abs(self_dist - peer_dist) <= 0.5f) {
                // If distances are very close, perform modulo tie-break
                if ((task_idx % active_ids.size()) != my_rank) {
                    self_wins = false;
                    break;
                }
            }
        }

        if (self_wins) {
            claimed_by_self.push_back(task);
        }
    }

    // ── STEP 5: Sort Claimed Tasks (Nearest First) ────────────────────
    std::sort(claimed_by_self.begin(), claimed_by_self.end(),
        [world_x, world_y](const swarm_interfaces::msg::TaskItem& a,
                           const swarm_interfaces::msg::TaskItem& b) {
            float da = std::hypot(world_x - a.x, world_y - a.y);
            float db = std::hypot(world_x - b.x, world_y - b.y);
            return da < db;
        });

    // ── STEP 6: Convert to Waypoint sequence ──────────────────────────
    for (const auto& t : claimed_by_self) {
        swarm_interfaces::msg::WaypointItem wp;
        wp.x = t.x;
        wp.y = t.y;
        wp.z = (t.z != 0.0f ? t.z : cruise_alt_ + spawn_z_);
        wp.speed = 3.0f;
        wp.hold_time_s = t.hold_time_s;
        result.waypoints.push_back(wp);
    }
    result.active_tasks = claimed_by_self;

    return result;
}

} // namespace swarm
