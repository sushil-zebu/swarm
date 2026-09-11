#include "swarm_core/px4_interface.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace swarm {

namespace {

constexpr float kAbsoluteVelocityCeilingMps = 7.0f;


// vehicle command sending log
const char* command_name(uint32_t command) {
    using Command = px4_msgs::msg::VehicleCommand;
    switch (command) {
    case Command::VEHICLE_CMD_COMPONENT_ARM_DISARM: return "ARM_DISARM";
    case Command::VEHICLE_CMD_DO_SET_MODE: return "SET_MODE";
    case Command::VEHICLE_CMD_NAV_LAND: return "LAND";
    case Command::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH: return "RTL";
    default: return "UNKNOWN";
    }
}

//ACK
const char* command_result_name(uint8_t result) {
    using Ack = px4_msgs::msg::VehicleCommandAck;
    switch (result) {
    case Ack::VEHICLE_CMD_RESULT_ACCEPTED: return "ACCEPTED";
    case Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: return "TEMPORARILY_REJECTED";
    case Ack::VEHICLE_CMD_RESULT_DENIED: return "DENIED";
    case Ack::VEHICLE_CMD_RESULT_UNSUPPORTED: return "UNSUPPORTED";
    case Ack::VEHICLE_CMD_RESULT_FAILED: return "FAILED";
    case Ack::VEHICLE_CMD_RESULT_IN_PROGRESS: return "IN_PROGRESS";
    case Ack::VEHICLE_CMD_RESULT_CANCELLED: return "CANCELLED";
    case Ack::VEHICLE_CMD_RESULT_COMMAND_LONG_ONLY: return "COMMAND_LONG_ONLY";
    case Ack::VEHICLE_CMD_RESULT_COMMAND_INT_ONLY: return "COMMAND_INT_ONLY";
    case Ack::VEHICLE_CMD_RESULT_UNSUPPORTED_MAV_FRAME: return "UNSUPPORTED_MAV_FRAME";
    default: return "UNKNOWN_RESULT";
    }
}


bool same_parameters(const std::array<float, 3>& lhs,
                     const std::array<float, 3>& rhs) {
    constexpr float kTolerance = 1.0e-6f;
    return std::fabs(lhs[0] - rhs[0]) <= kTolerance &&
           std::fabs(lhs[1] - rhs[1]) <= kTolerance &&
           std::fabs(lhs[2] - rhs[2]) <= kTolerance;
}

}

void PX4Interface::setup(rclcpp::Node& node, int mav_sys_id) {
    node_ = &node;
    mav_sys_id_ = mav_sys_id;

    const double requested_velocity_limit =
        node_->declare_parameter<double>("px4_max_velocity_mps", 7.0);
    const double requested_acceleration_limit =
        node_->declare_parameter<double>("px4_max_acceleration_mps2", 3.0);
    vehicle_status_timeout_s_ =
        node_->declare_parameter<double>("px4_status_timeout_s", 1.0);
    command_ack_timeout_s_ =
        node_->declare_parameter<double>("px4_command_ack_timeout_s", 0.75);
    command_retry_interval_s_ =
        node_->declare_parameter<double>("px4_command_retry_interval_s", 1.0);

    // here i wanted to a max velociy and accleration that we can limit
    max_velocity_mps_ = static_cast<float>(std::clamp(
        requested_velocity_limit, 0.1,
        static_cast<double>(kAbsoluteVelocityCeilingMps)));
    max_acceleration_mps2_ = static_cast<float>(std::max(
        0.1, requested_acceleration_limit));

    vehicle_status_timeout_s_ = std::max(0.1, vehicle_status_timeout_s_);
    command_ack_timeout_s_ = std::max(0.1, command_ack_timeout_s_);
    command_retry_interval_s_ = std::max(
        command_ack_timeout_s_, command_retry_interval_s_);

    if (requested_velocity_limit > kAbsoluteVelocityCeilingMps) {
        RCLCPP_WARN(node_->get_logger(),
            "[PX4 INTERFACE] Requested %.2fm/s exceeds the absolute 7.00m/s "
            "ceiling; using 7.00m/s.", requested_velocity_limit);
    }

    auto sensor_qos = rclcpp::SensorDataQoS();

    offboard_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "fmu/in/offboard_control_mode", sensor_qos);

    setpoint_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "fmu/in/trajectory_setpoint", sensor_qos);

    command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        "fmu/in/vehicle_command", sensor_qos);

    status_sub_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        "fmu/out/vehicle_status_v4", sensor_qos,
        [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            arming_state_ = msg->arming_state;
            nav_state_    = msg->nav_state;
            vehicle_status_valid_ =
                (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED ||
                 arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) &&
                nav_state_ < px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MAX;
            last_vehicle_status_ = node_->now();
        });

    pos_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "fmu/out/vehicle_local_position_v1", sensor_qos,
        [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            const bool finite =
                std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->z) &&
                std::isfinite(msg->vx) && std::isfinite(msg->vy) && std::isfinite(msg->vz);
            local_position_valid_ = msg->xy_valid && msg->z_valid &&
                msg->v_xy_valid && msg->v_z_valid && finite;
            if (local_position_valid_) {
                pos_x_ = msg->x; pos_y_ = msg->y; pos_z_ = msg->z;
                vel_x_ = msg->vx; vel_y_ = msg->vy; vel_z_ = msg->vz;
                last_local_position_ = node_->now();
            }
        });

    command_ack_sub_ = node_->create_subscription<px4_msgs::msg::VehicleCommandAck>(
        "fmu/out/vehicle_command_ack_v1", sensor_qos,
        [this](px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
            handle_vehicle_command_ack(msg);
        });

    RCLCPP_INFO(node_->get_logger(),
        "[PX4 INTERFACE] velocity mode default, max velocity=%.2fm/s, "
        "max acceleration=%.2fm/s^2, status timeout=%.2fs",
        max_velocity_mps_, max_acceleration_mps2_, vehicle_status_timeout_s_);
}

bool PX4Interface::has_fresh_local_position(double max_age_s) const {
    if (!node_ || !local_position_valid_ || last_local_position_.nanoseconds() <= 0) {
        return false;
    }
    const double age_s = (node_->now() - last_local_position_).seconds();
    return age_s >= 0.0 && age_s <= max_age_s;
}

bool PX4Interface::has_fresh_vehicle_status(double max_age_s) const {
    if (!node_ || !vehicle_status_valid_ ||
        last_vehicle_status_.nanoseconds() <= 0) {
        return false;
    }
    const double allowed_age =
        max_age_s > 0.0 ? max_age_s : vehicle_status_timeout_s_;
    const double age_s = (node_->now() - last_vehicle_status_).seconds();
    return age_s >= 0.0 && age_s <= allowed_age;
}

bool PX4Interface::is_armed() const {
    return has_fresh_vehicle_status() &&
        arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

bool PX4Interface::is_disarmed() const {
    return has_fresh_vehicle_status() &&
        arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
}

bool PX4Interface::is_in_offboard() const {
    return has_fresh_vehicle_status() &&
        nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

bool PX4Interface::is_in_auto_land() const {
    return has_fresh_vehicle_status() &&
        nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
}

bool PX4Interface::is_in_auto_rtl() const {
    return has_fresh_vehicle_status() &&
        nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL;
}

void PX4Interface::publish_offboard_control_mode(OffboardMode mode) {
    if (!has_fresh_local_position() || !has_fresh_vehicle_status()) {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[FLIGHT SAFETY] Withholding Offboard heartbeat: PX4 feedback is stale.");
        return;
    }

    px4_msgs::msg::OffboardControlMode m{};
    m.position  = mode == OffboardMode::Position;
    m.velocity  = mode == OffboardMode::Velocity;
    m.timestamp = node_->now().nanoseconds() / 1000;
    offboard_pub_->publish(m);
}

void PX4Interface::publish_position_setpoint(float x, float y, float z, float yaw) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !has_fresh_local_position() || !has_fresh_vehicle_status()) {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[FLIGHT SAFETY] Withholding position setpoint: target or local estimate is invalid.");
        return;
    }

    publish_offboard_control_mode(OffboardMode::Position);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint m{};
    m.position  = {x, y, z};
    m.velocity = {nan, nan, nan};
    m.acceleration = {nan, nan, nan};
    m.jerk = {nan, nan, nan};
    m.yaw       = yaw;
    m.yawspeed = nan;
    m.timestamp = node_->now().nanoseconds() / 1000;
    setpoint_pub_->publish(m);
}

void PX4Interface::publish_velocity_setpoint(float vx, float vy, float vz, float yaw) {
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz) ||
        !has_fresh_local_position() || !has_fresh_vehicle_status()) {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[FLIGHT SAFETY] Withholding velocity setpoint so PX4 Offboard-loss failsafe can act.");
        last_velocity_setpoint_valid_ = false;
        return;
    }

    publish_offboard_control_mode(OffboardMode::Velocity);

    const auto limited = limit_velocity(vx, vy, vz, node_->now());
    const float nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint m{};
    m.position     = {nan, nan, nan};
    m.velocity     = limited;
    m.acceleration = {nan, nan, nan};
    m.jerk         = {nan, nan, nan};
    m.yaw          = yaw;
    m.yawspeed     = nan;
    m.timestamp    = node_->now().nanoseconds() / 1000;
    setpoint_pub_->publish(m);
}

std::array<float, 3> PX4Interface::limit_velocity(
    float vx, float vy, float vz, const rclcpp::Time& now) {
    std::array<float, 3> target{vx, vy, vz};
    const float requested_speed = std::sqrt(vx * vx + vy * vy + vz * vz);

    if (requested_speed > max_velocity_mps_) {
        const float scale = max_velocity_mps_ / requested_speed;
        for (float& component : target) {
            component *= scale;
        }
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[FLIGHT SAFETY] Velocity %.2fm/s limited to %.2fm/s.",
            requested_speed, max_velocity_mps_);
    }

    if (!last_velocity_setpoint_valid_) {
        last_velocity_setpoint_ = {vel_x_, vel_y_, vel_z_};
        const float current_speed = std::sqrt(
            vel_x_ * vel_x_ + vel_y_ * vel_y_ + vel_z_ * vel_z_);
        if (current_speed > max_velocity_mps_) {
            const float scale = max_velocity_mps_ / current_speed;
            for (float& component : last_velocity_setpoint_) {
                component *= scale;
            }
        }
        last_velocity_setpoint_time_ = now;
        last_velocity_setpoint_valid_ = true;
    }

    double dt_s = (now - last_velocity_setpoint_time_).seconds();
    if (dt_s < 0.0) {
        dt_s = 0.0;
    } else if (dt_s > 0.5) {
        // A long publishing gap means the aircraft may no longer be following
        // our old setpoint. Restart the limiter from measured velocity.
        last_velocity_setpoint_ = {vel_x_, vel_y_, vel_z_};
        dt_s = 0.1;
    }
    dt_s = std::min(dt_s, 0.2);

    std::array<float, 3> delta{
        target[0] - last_velocity_setpoint_[0],
        target[1] - last_velocity_setpoint_[1],
        target[2] - last_velocity_setpoint_[2]};
    const float delta_speed = std::sqrt(
        delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
    const float max_delta = max_acceleration_mps2_ * static_cast<float>(dt_s);

    if (delta_speed > max_delta && delta_speed > 0.0f) {
        const float scale = max_delta / delta_speed;
        for (std::size_t i = 0; i < target.size(); ++i) {
            target[i] = last_velocity_setpoint_[i] + delta[i] * scale;
        }
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[FLIGHT SAFETY] Velocity change limited to %.2fm/s^2.",
            max_acceleration_mps2_);
    }

    const float limited_speed = std::sqrt(
        target[0] * target[0] + target[1] * target[1] + target[2] * target[2]);
    if (limited_speed > max_velocity_mps_) {
        const float scale = max_velocity_mps_ / limited_speed;
        for (float& component : target) {
            component *= scale;
        }
    }

    last_velocity_setpoint_ = target;
    last_velocity_setpoint_time_ = now;
    return target;
}

void PX4Interface::send_vehicle_command(
    uint32_t command, float param1, float param2, float param7) {
    const rclcpp::Time now = node_->now();
    const std::array<float, 3> params{param1, param2, param7};
    auto& request = command_requests_[command];

    if (!request.initialized || !same_parameters(request.params, params)) {
        request = CommandRequestState{};
        request.initialized = true;
        request.params = params;
    } else {
        const double since_send_s = (now - request.last_sent).seconds();
        if (request.awaiting_ack && since_send_s < command_ack_timeout_s_) {
            return;
        }
        if (request.awaiting_ack) {
            RCLCPP_WARN(node_->get_logger(),
                "[PX4 COMMAND] %s (%u) acknowledgement timed out after %.2fs; retrying.",
                command_name(command), command, since_send_s);
        } else if (since_send_s >= 0.0 &&
                   since_send_s < command_retry_interval_s_) {
            return;
        }
    }

    px4_msgs::msg::VehicleCommand m{};
    m.command          = command;
    m.param1           = param1;
    m.param2           = param2;
    m.param7           = param7;
    m.target_system    = mav_sys_id_;
    m.target_component = 1;
    m.source_system    = 1;
    m.source_component = 1;
    m.from_external    = true;
    m.timestamp        = now.nanoseconds() / 1000;
    command_pub_->publish(m);

    request.awaiting_ack = true;
    request.last_sent = now;
    ++request.attempts;

    RCLCPP_INFO(node_->get_logger(),
        "[PX4 COMMAND] Sent %s (%u), attempt %u.",
        command_name(command), command, request.attempts);
}

void PX4Interface::handle_vehicle_command_ack(
    const px4_msgs::msg::VehicleCommandAck::SharedPtr& msg) {
    const auto it = command_requests_.find(msg->command);
    if (it == command_requests_.end()) {
        RCLCPP_DEBUG(node_->get_logger(),
            "[PX4 COMMAND] Ignoring acknowledgement for untracked command %u.",
            msg->command);
        return;
    }

    auto& request = it->second;
    if (msg->result ==
        px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS) {
        request.awaiting_ack = true;
        request.last_sent = node_->now();
        RCLCPP_INFO(node_->get_logger(),
            "[PX4 COMMAND] %s (%u) is IN_PROGRESS (%u%%).",
            command_name(msg->command), msg->command, msg->result_param1);
        return;
    }

    request.awaiting_ack = false;
    if (msg->result ==
        px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
        RCLCPP_INFO(node_->get_logger(),
            "[PX4 COMMAND] %s (%u) ACCEPTED on attempt %u.",
            command_name(msg->command), msg->command, request.attempts);
        return;
    }

    RCLCPP_ERROR(node_->get_logger(),
        "[PX4 COMMAND] %s (%u) rejected: %s (result=%u, reason=%u, detail=%d, attempt=%u).",
        command_name(msg->command), msg->command,
        command_result_name(msg->result), msg->result,
        msg->result_param1, msg->result_param2, request.attempts);
}

void PX4Interface::arm() {
    if (!has_fresh_vehicle_status()) {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[FLIGHT SAFETY] Refusing to arm without fresh VehicleStatus.");
        return;
    }
    if (is_armed()) {
        command_requests_.erase(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM);
        return;
    }
    send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void PX4Interface::disarm() {
    if (is_disarmed()) {
        command_requests_.erase(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM);
        return;
    }
    send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void PX4Interface::engage_offboard() {
    if (!has_fresh_vehicle_status()) {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[FLIGHT SAFETY] Refusing Offboard request without fresh VehicleStatus.");
        return;
    }
    if (is_in_offboard()) {
        command_requests_.erase(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE);
        return;
    }
    send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void PX4Interface::land() {
    if (is_in_auto_land()) {
        command_requests_.erase(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        return;
    }
    send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

void PX4Interface::rtl() {
    if (is_in_auto_rtl()) {
        command_requests_.erase(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
        return;
    }
    send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
}

}
