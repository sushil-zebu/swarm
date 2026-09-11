#include "swarm_core/drone_controller_node.hpp"

#include "swarm_core/mission_helix.hpp"
#include "swarm_core/mission_triangle.hpp"
#include "swarm_core/cbf_test.hpp"

#include <stdexcept>

using namespace std::chrono_literals;

namespace swarm {

DroneControllerNode::DroneControllerNode(const rclcpp::NodeOptions& options)
    : Node("drone_controller_node", options)
{
    load_parameters();

    px4_.setup(*this, mav_sys_id_);
    peers_.setup(*this, drone_id_, num_drones_, peer_timeout_s_);

    CbfConfig cbf_config;
    cbf_config.safe_radius = static_cast<float>(
        get_parameter("safe_radius").as_double());

    cbf_config.position_uncertainty = static_cast<float>(
        get_parameter("position_uncertainty").as_double());

    cbf_config.control_delay_s = static_cast<float>(
        get_parameter("control_delay_s").as_double());

    cbf_config.max_speed_mps = static_cast<float>(
        get_parameter("max_speed_mps").as_double());

    cbf_config.max_brake_mps2 = static_cast<float>(
        get_parameter("max_brake_mps2").as_double());

    cbf_config.goal_gain = static_cast<float>(
        get_parameter("goal_gain").as_double());

    cbf_config.cbf_alpha = static_cast<float>(
        get_parameter("cbf_alpha").as_double());

    cbf_config.peer_accel_uncertainty_mps2 = static_cast<float>(
        get_parameter("peer_accel_uncertainty_mps2").as_double());

    cbf_config.constraint_tolerance_mps = static_cast<float>(
        get_parameter("cbf_constraint_tolerance_mps").as_double());

    cbf_config.projection_iterations = static_cast<int>(
        get_parameter("cbf_projection_iterations").as_int());
        
    cbf_config.fail_closed_on_peer_loss =
        get_parameter("cbf_fail_closed_on_peer_loss").as_bool();
    cbf_.setup(*this, drone_id_, cbf_config);

    // active_mission_ = std::make_unique<MissionTriangle>(
    //     *this, px4_, peers_, cbf_,
    //     drone_id_, num_drones_,
    //     spawn_x_, spawn_y_, spawn_z_
    // );

    // active_mission_ = std::make_unique<MissionHelix>(
    //     *this, px4_, peers_, cbf_,
    //     drone_id_, num_drones_,
    //     spawn_x_, spawn_y_, spawn_z_
    // );

    active_mission_ = std::make_unique<CbfTest>(
    *this,
    px4_,
    peers_,
    cbf_,
    drone_id_,
    num_drones_,
    spawn_x_,
    spawn_y_,
    spawn_z_
    );

    timer_ = create_wall_timer(
        100ms, std::bind(&DroneControllerNode::control_loop, this));
    RCLCPP_INFO(get_logger(),
        "[SWARM CORE] Drone %u online (MAV_SYS_ID=%d), mission=%s",
        drone_id_, mav_sys_id_, active_mission_->name());
    RCLCPP_INFO(get_logger(),
        "[SWARM CORE] World spawn offset: (%.2f, %.2f, %.2f)",
        spawn_x_, spawn_y_, spawn_z_);

}

void DroneControllerNode::load_parameters() {
    declare_parameter("drone_id", 1);
    declare_parameter("mav_sys_id", 2);
    declare_parameter("num_drones", 4);
    declare_parameter("spawn_x", 0.0);
    declare_parameter("spawn_y", 0.0);
    declare_parameter("spawn_z", 0.0);
    declare_parameter("peer_timeout", 1.5);
    declare_parameter("safe_radius", 2.5);
    declare_parameter("position_uncertainty", 0.5);
    declare_parameter("control_delay_s", 0.15);
    declare_parameter("max_speed_mps", 3.0);
    declare_parameter("max_brake_mps2", 2.0);
    declare_parameter("goal_gain", 1.0);
    declare_parameter("cbf_alpha", 1.5);
    declare_parameter("peer_accel_uncertainty_mps2", 2.0);
    declare_parameter("cbf_constraint_tolerance_mps", 0.02);
    declare_parameter("cbf_projection_iterations", 80);
    declare_parameter("cbf_fail_closed_on_peer_loss", true);
 
    drone_id_ = static_cast<uint8_t>(get_parameter("drone_id").as_int());
    mav_sys_id_ = get_parameter("mav_sys_id").as_int();
    num_drones_ = get_parameter("num_drones").as_int();
    spawn_x_ = static_cast<float>(get_parameter("spawn_x").as_double());
    spawn_y_ = static_cast<float>(get_parameter("spawn_y").as_double());
    spawn_z_ = static_cast<float>(get_parameter("spawn_z").as_double());
    peer_timeout_s_ = get_parameter("peer_timeout").as_double();
}

void DroneControllerNode::control_loop() {
    if (active_mission_) {
        active_mission_->control_loop();
    }
}

} 

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<swarm::DroneControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
