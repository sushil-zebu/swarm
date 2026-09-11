#include "swarm_core/cbf_avoidance.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

float dot(const swarm::Vec3f& a, const swarm::Vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float norm(const swarm::Vec3f& v) {
    return std::sqrt(std::max(0.0f, dot(v, v)));
}

bool finite(const swarm::Vec3f& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

swarm::Vec3f clamp_to_ball(const swarm::Vec3f& value, float radius) {
    const float magnitude = norm(value);
    if (magnitude <= radius || magnitude <= 1.0e-6f) return value;
    return value * (radius / magnitude);
}

}  // namespace

namespace swarm {

void CbfAvoidance::setup(rclcpp::Node& node, uint8_t drone_id, const CbfConfig& config) {
    const bool values_are_finite =
        std::isfinite(config.safe_radius) &&
        std::isfinite(config.position_uncertainty) &&
        std::isfinite(config.control_delay_s) &&
        std::isfinite(config.max_speed_mps) &&
        std::isfinite(config.max_brake_mps2) &&
        std::isfinite(config.goal_gain) &&
        std::isfinite(config.cbf_alpha) &&
        std::isfinite(config.peer_accel_uncertainty_mps2) &&
        std::isfinite(config.constraint_tolerance_mps);
    if (!values_are_finite ||
        config.safe_radius <= 0.0f || config.position_uncertainty < 0.0f ||
        config.control_delay_s < 0.0f || config.max_speed_mps <= 0.0f ||
        config.max_brake_mps2 <= 0.0f || config.goal_gain <= 0.0f ||
        config.cbf_alpha <= 0.0f || config.peer_accel_uncertainty_mps2 < 0.0f ||
        config.constraint_tolerance_mps < 0.0f || config.projection_iterations < 1) {
        throw std::invalid_argument("CBF safety parameters must be finite and positive");
    }
    node_ = &node;
    drone_id_ = drone_id;
    config_ = config;
}

Vec3f CbfAvoidance::compute_safe_velocity(
    float target_x, float target_y, float target_z,
    float requested_speed,
    float self_wx, float self_wy, float self_wz,
    float self_vx, float self_vy, float self_vz,
    const PeerTracker& peers)
{
    const float gx = target_x - self_wx;
    const float gy = target_y - self_wy;
    const float gz = target_z - self_wz;
    const float goal_dist = std::sqrt(gx * gx + gy * gy + gz * gz);

    Vec3f nominal{0.0f, 0.0f, 0.0f};
    if (std::isfinite(goal_dist) && goal_dist > 0.10f &&
        std::isfinite(requested_speed) && requested_speed > 0.0f) {
        const float speed = std::min(
            {requested_speed, config_.max_speed_mps, config_.goal_gain * goal_dist});
        nominal = {speed * gx / goal_dist,
                   speed * gy / goal_dist,
                   speed * gz / goal_dist};
    }

    return filter_velocity(
        nominal, self_wx, self_wy, self_wz,
        self_vx, self_vy, self_vz, peers);
}

Vec3f CbfAvoidance::filter_velocity(
    const Vec3f& nominal_velocity,
    float self_wx, float self_wy, float self_wz,
    float self_vx, float self_vy, float self_vz,
    const PeerTracker& peers)
{
    constexpr float kMinDistance = 0.05f;
    constexpr float kTwoPi = 6.28318530718f;

    const Vec3f self_position{self_wx, self_wy, self_wz};
    const Vec3f self_velocity{self_vx, self_vy, self_vz};
    if (!node_ || !finite(nominal_velocity) || !finite(self_position) ||
        !finite(self_velocity)) {
        if (node_) {
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[CBF FAIL-CLOSED] Invalid self state or nominal command; hovering.");
        }
        return {};
    }

    if (config_.fail_closed_on_peer_loss && !peers.all_peers_valid()) {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[CBF FAIL-CLOSED] At least one expected peer is missing or stale; hovering.");
        return {};
    }

    struct PeerConstraint {
        uint8_t id;
        Vec3f normal;       // Unit vector from peer toward this drone
        float lower_bound; // normal dot command must be at least this value
        float distance;
        float r_eff;
    };

    std::vector<PeerConstraint> constraints;
    constraints.reserve(peers.peers().size());

    for (const auto& [id, peer] : peers.peers()) {
        if (!peers.is_peer_valid(peer)) {
            // Reaching here is possible only when fail_closed_on_peer_loss is
            // disabled. Never build a constraint from invalid data.
            continue;
        }

        const float age_s = static_cast<float>(peers.peer_age_s(peer));
        const Vec3f peer_velocity{peer.vx, peer.vy, peer.vz};
        const Vec3f peer_position{
            peer.x + peer.vx * age_s,
            peer.y + peer.vy * age_s,
            peer.z + peer.vz * age_s};

        if (!std::isfinite(age_s) || !finite(peer_position) ||
            !finite(peer_velocity)) {
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[CBF FAIL-CLOSED] Peer %u state cannot be extrapolated safely; hovering.",
                id);
            return {};
        }

        const Vec3f displacement = self_position - peer_position;
        const float measured_distance = norm(displacement);
        const float distance = std::max(measured_distance, kMinDistance);
        Vec3f normal{};

        if (measured_distance < 1.0e-4f) {
            // Both drones independently choose exact opposite directions for
            // the same ID pair. Store this direction in the constraint so it
            // is not lost during later projection sweeps.
            const uint8_t low_id = std::min(drone_id_, id);
            const uint8_t high_id = std::max(drone_id_, id);
            const float angle = kTwoPi *
                static_cast<float>((low_id * 31u + high_id * 17u) % 360u) / 360.0f;
            const float sign = (drone_id_ < id) ? 1.0f : -1.0f;
            normal = {sign * std::cos(angle), sign * std::sin(angle), 0.0f};
        } else {
            normal = displacement * (1.0f / measured_distance);
        }

        const Vec3f relative_velocity = self_velocity - peer_velocity;
        const float closing_speed = std::max(0.0f, -dot(normal, relative_velocity));

        // Constant-velocity extrapolation handles message age. This extra
        // term bounds unmodelled acceleration during that age.
        const float acceleration_uncertainty =
            0.5f * config_.peer_accel_uncertainty_mps2 * age_s * age_s;
        const float effective_radius =
            config_.safe_radius +
            config_.position_uncertainty +
            closing_speed * (age_s + config_.control_delay_s) +
            (closing_speed * closing_speed) /
                (2.0f * config_.max_brake_mps2) +
            acceleration_uncertainty;

        const float h = distance * distance - effective_radius * effective_radius;
        const float lower_bound =
            dot(normal, peer_velocity) - 0.5f * config_.cbf_alpha * h / distance;
        if (!finite(normal) || !std::isfinite(effective_radius) ||
            !std::isfinite(lower_bound)) {
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[CBF FAIL-CLOSED] Peer %u produced an invalid safety constraint; hovering.",
                id);
            return {};
        }
        constraints.push_back({id, normal, lower_bound, distance, effective_radius});
    }

    Vec3f command = clamp_to_ball(nominal_velocity, config_.max_speed_mps);
    if (constraints.empty()) return command;

    // Dykstra's algorithm projects the nominal command onto the intersection
    // of every CBF half-space AND the maximum-speed ball. Unlike a final speed
    // clamp, this preserves already-satisfied collision constraints.
    std::vector<Vec3f> residuals(constraints.size() + 1);
    for (int iter = 0; iter < config_.projection_iterations; ++iter) {
        for (size_t i = 0; i < constraints.size(); ++i) {
            const Vec3f input = command + residuals[i];
            Vec3f projected = input;
            const float violation = constraints[i].lower_bound -
                dot(constraints[i].normal, input);
            if (violation > 0.0f) {
                projected = input + constraints[i].normal * violation;
            }
            residuals[i] = input - projected;
            command = projected;
        }

        const size_t ball_index = constraints.size();
        const Vec3f ball_input = command + residuals[ball_index];
        const Vec3f ball_projection =
            clamp_to_ball(ball_input, config_.max_speed_mps);
        residuals[ball_index] = ball_input - ball_projection;
        command = ball_projection;
    }

    auto worst_violation = [&constraints](const Vec3f& candidate) {
        float worst = 0.0f;
        for (const auto& c : constraints) {
            worst = std::max(worst, c.lower_bound - dot(c.normal, candidate));
        }
        return worst;
    };

    float final_violation = worst_violation(command);
    if (final_violation > config_.constraint_tolerance_mps) {
        // The bounded constraint set is physically infeasible (or did not
        // converge). Find the bounded velocity that minimizes the worst CBF
        // violation. This normally becomes maximum braking/separation.
        Vec3f candidate = command;
        Vec3f best = command;
        float best_violation = final_violation;

        const Vec3f stopped{};
        const float stopped_violation = worst_violation(stopped);
        if (stopped_violation < best_violation) {
            best = stopped;
            best_violation = stopped_violation;
        }

        for (int iter = 1; iter <= 256; ++iter) {
            const PeerConstraint* worst_constraint = nullptr;
            float violation = -1.0f;
            for (const auto& c : constraints) {
                const float current = c.lower_bound - dot(c.normal, candidate);
                if (current > violation) {
                    violation = current;
                    worst_constraint = &c;
                }
            }
            if (!worst_constraint || violation <= config_.constraint_tolerance_mps) break;

            const float step = 0.35f * config_.max_speed_mps /
                std::sqrt(static_cast<float>(iter));
            candidate = clamp_to_ball(
                candidate + worst_constraint->normal * step,
                config_.max_speed_mps);
            const float candidate_violation = worst_violation(candidate);
            if (candidate_violation < best_violation) {
                best = candidate;
                best_violation = candidate_violation;
            }
        }

        command = best;
        final_violation = best_violation;
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[CBF EMERGENCY] Collision constraints are physically infeasible at "
            "the %.2fm/s speed limit; applying least-violating escape (residual=%.3fm/s).",
            config_.max_speed_mps, final_violation);
    }

    for (const auto& c : constraints) {
        if (c.lower_bound - dot(c.normal, nominal_velocity) >
            config_.constraint_tolerance_mps) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[CBF SEPARATION] peer=%u dist=%.2f R_eff=%.2f",
                c.id, c.distance, c.r_eff);
        }
    }

    if (!finite(command)) {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[CBF FAIL-CLOSED] Solver produced a non-finite command; hovering.");
        return {};
    }
    return clamp_to_ball(command, config_.max_speed_mps);
}

} // namespace swarm
