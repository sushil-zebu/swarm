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

// Minimal public shell. The peer/arming test state machine stays private to
// cbf_test.cpp, following the same structure as the other missions.
class CbfTest final : public FlightMission {
public:
            CbfTest(rclcpp::Node& node,
                    PX4Interface& px4,
                    PeerTracker& peers,
                    CbfAvoidance& cbf,
                    uint8_t drone_id,
                    int num_drones,
                    float spawn_x,
                    float spawn_y,
                    float spawn_z);
    ~CbfTest() override;

    CbfTest(const CbfTest&) = delete;
    CbfTest& operator=(const CbfTest&) = delete;

    void control_loop() override;
    const char* name() const override { 
        return "cbftest"; 
    }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}
