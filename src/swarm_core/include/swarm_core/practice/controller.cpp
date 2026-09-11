#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <cmath>

using namespace std::chrono_literals;

enum class MissionState {
    INIT,       
    ARMING,      
    TAKEOFF,     
    WAYPOINT_1,  
    WAYPOINT_2,  
    WAYPOINT_3,  
    WAYPOINT_4,  
    LAND,        
    DISARM,      
    DONE
};

class SquareMissionNode : public rclcpp::Node
{
public:
    SquareMissionNode() : Node("square_mission_node")
    {

        //  we send to PX4
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // px4 tells us
        rclcpp::QoS qos(10);
        qos.best_effort();  

        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                current_pos_ = *msg;
                have_position_ = true;
            });
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status",
        qos,
        [this](px4_msgs::msg::VehicleStatus::SharedPtr msg){
            vehicle_status_ = *msg;
            have_vehicle_status_ = true;
        });
        timer_ = this->create_wall_timer(50ms, std::bind(&SquareMissionNode::tick, this));
    }

private:
    void tick(){
        if (!have_position_) return;

        publish_offboard_heartbeat();
        offboard_setpoint_counter_++;

        switch (state_) {
            case MissionState::INIT:
                publish_setpoint(0.0, 0.0, -takeoff_altitude_);
                if (offboard_setpoint_counter_ == 10) {
                    set_offboard_mode();
                    arm();
                    state_ = MissionState::ARMING;
                }
                break;

            case MissionState::ARMING:
                publish_setpoint(0.0, 0.0, -takeoff_altitude_);
                if (is_armed()) {
                    RCLCPP_INFO(this->get_logger(), "Armed. Climbing to altitude.");
                    state_ = MissionState::TAKEOFF;
                }
                break;

            case MissionState::TAKEOFF:
                publish_setpoint(0.0, 0.0, -takeoff_altitude_);
                if (reached(0.0, 0.0)) {
                    RCLCPP_INFO(this->get_logger(), "At altitude. Flying corner 1.");
                    state_ = MissionState::WAYPOINT_1;
                }
                break;

            case MissionState::WAYPOINT_1:
                publish_setpoint(square_side_, 0.0, -takeoff_altitude_);
                if (reached(square_side_, 0.0)) state_ = MissionState::WAYPOINT_2;
                break;

            case MissionState::WAYPOINT_2:
                {
                    float height_alt = takeoff_altitude_;
                    publish_setpoint(square_side_, square_side_, -height_alt);
                    if (reached(square_side_, square_side_)) state_ = MissionState::WAYPOINT_3;
                }
                break;

            case MissionState::WAYPOINT_3:
                publish_setpoint(0.0, square_side_, -takeoff_altitude_);
                if (reached(0.0, square_side_)) state_ = MissionState::WAYPOINT_4;
                break;

            case MissionState::WAYPOINT_4:
                publish_setpoint(0.0, 0.0, -takeoff_altitude_);
                if (reached(0.0, 0.0)) {
                    RCLCPP_INFO(this->get_logger(), "Back over home. Landing.");
                    state_ = MissionState::LAND;
                }
                break;

            case MissionState::LAND:
                land();
                if (is_landed()) state_ = MissionState::DISARM;
                break;

            case MissionState::DISARM:
                disarm();
                state_ = MissionState::DONE;
                break;

            case MissionState::DONE:
                break;
        }
    }

    bool reached(float x, float y){
        float dx = current_pos_.x - x;
        float dy = current_pos_.y - y;
        float dz = current_pos_.z - (-takeoff_altitude_);
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        return dist < position_tolerance_;
    }

    bool is_ready(){
        return have_vehicle_status_ && vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    }

    bool is_armed(){
        return have_vehicle_status_ && vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    }

    bool is_landed(){
        return std::abs(current_pos_.z) < 0.15;
    }

    void publish_offboard_heartbeat(){
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = now_us();
        offboard_mode_pub_->publish(msg);
    }

    void publish_setpoint(float x, float y, float z){
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {x, y, z};
        msg.timestamp = now_us();
        trajectory_pub_->publish(msg);
    }

    void set_offboard_mode(){
        send_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0, 6.0);
    }

    void arm(){
        send_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    }

    void disarm(){
        send_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    }

    void land(){
        send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }

    void send_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0){
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = now_us();
        vehicle_command_pub_->publish(msg);
    }

    uint64_t now_us(){
        return this->get_clock()->now().nanoseconds() / 1000;
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    px4_msgs::msg::VehicleLocalPosition current_pos_{};
    bool have_position_ = false;
    px4_msgs::msg::VehicleStatus vehicle_status_;
    bool have_vehicle_status_ = false;
    MissionState state_ = MissionState::INIT;
    uint64_t offboard_setpoint_counter_ = 0;

    float takeoff_altitude_ = 5.0;
    float square_side_ = 5.0;
    float position_tolerance_ = 0.4;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareMissionNode>());
    rclcpp::shutdown();
    return 0;
}