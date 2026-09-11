#pragma once

#include <cstdint>
#include <memory>

#include "swarm_core/flight_mission.hpp"

namespace rclcpp {
class Node;
}

namespace swarm {

class CbfAvoidance;
class PeerTracker;
class PX4Interface;

// Public shell for the helix mission. Its complete implementation—including
// all parameters, phases, timers, subscriptions, and helpers—is private to
// mission_helix.cpp.
class MissionHelix final : public FlightMission {
public:
    MissionHelix(rclcpp::Node& node,
                 PX4Interface& px4,
                 PeerTracker& peers,
                 CbfAvoidance& cbf,
                 uint8_t drone_id,
                 int num_drones,
                 float spawn_x,
                 float spawn_y,
                 float spawn_z);
    ~MissionHelix() override;

    MissionHelix(const MissionHelix&) = delete;
    MissionHelix& operator=(const MissionHelix&) = delete;

    void control_loop() override;
    const char* name() const override { return "helix"; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace swarm
