#include "swarm_core/drone_controller_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    
    options.append_parameter_override("drone_id", 2);
    options.append_parameter_override("mav_sys_id", 3);

    auto node = std::make_shared<swarm::DroneControllerNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
