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

// Minimal public shell. All triangle variables and control logic are private to
// mission_triangle.cpp, following the same structure as MissionHelix.
class MissionTriangle final : public FlightMission {
public:
    MissionTriangle(rclcpp::Node& node,
                    PX4Interface& px4,
                    PeerTracker& peers,
                    CbfAvoidance& cbf,
                    uint8_t drone_id,
                    int num_drones,
                    float spawn_x,
                    float spawn_y,
                    float spawn_z);
    ~MissionTriangle() override;

    MissionTriangle(const MissionTriangle&) = delete;
    MissionTriangle& operator=(const MissionTriangle&) = delete;

    void control_loop() override;
    const char* name() const override { return "triangle"; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace swarm
