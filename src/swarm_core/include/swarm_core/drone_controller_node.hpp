#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "swarm_core/cbf_avoidance.hpp"
#include "swarm_core/flight_mission.hpp"
#include "swarm_core/peer_tracker.hpp"
#include "swarm_core/px4_interface.hpp"

namespace swarm {

class DroneControllerNode : public rclcpp::Node {
public:
    explicit DroneControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~DroneControllerNode() override = default;

private:
    // Shared vehicle/communication setup. Mission-specific parameters are
    // declared and owned by the concrete FlightMission implementation.
    void load_parameters();

    // The main node deliberately has no mission state machine. It delegates
    // each 10 Hz tick through the generic FlightMission interface.
    void control_loop();

    PX4Interface px4_;
    PeerTracker peers_;
    CbfAvoidance cbf_;

    uint8_t drone_id_{1};
    int mav_sys_id_{2};
    int num_drones_{3};
    float spawn_x_{0.0f};
    float spawn_y_{0.0f};
    float spawn_z_{0.0f};
    double peer_timeout_s_{1.5};

    std::unique_ptr<FlightMission> active_mission_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace swarm
