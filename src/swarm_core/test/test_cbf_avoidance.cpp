#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <swarm_interfaces/msg/drone_state.hpp>

#include "swarm_core/cbf_avoidance.hpp"
#include "swarm_core/peer_tracker.hpp"

namespace {

using namespace std::chrono_literals;
using swarm_interfaces::msg::DroneState;

float magnitude(const swarm::Vec3f& value) {
    return std::sqrt(
        value.x * value.x + value.y * value.y + value.z * value.z);
}

class CbfAvoidanceTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    void TearDown() override {
        publishers_.clear();
        if (peer_node_) executor_.remove_node(peer_node_);
        if (self_node_) executor_.remove_node(self_node_);
        peer_node_.reset();
        self_node_.reset();
    }

    void configure(int num_drones) {
        static int sequence = 0;
        ++sequence;
        self_node_ = std::make_shared<rclcpp::Node>(
            "cbf_test_self_" + std::to_string(sequence));
        peer_node_ = std::make_shared<rclcpp::Node>(
            "cbf_test_peer_" + std::to_string(sequence));

        tracker_.setup(*self_node_, 1, num_drones, 1.5);
        swarm::CbfConfig config;
        config.safe_radius = 4.0f;
        config.position_uncertainty = 0.5f;
        config.max_speed_mps = 3.0f;
        config.projection_iterations = 120;
        cbf_.setup(*self_node_, 1, config);

        executor_.add_node(self_node_);
        executor_.add_node(peer_node_);
    }

    void publish_peer(uint8_t id, float x, float y, float z,
                      float vx = 0.0f, float vy = 0.0f, float vz = 0.0f) {
        auto publisher = peer_node_->create_publisher<DroneState>(
            "/drone_" + std::to_string(id) + "/swarm/self_state",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
        publishers_.push_back(publisher);

        DroneState state;
        state.drone_id = id;
        state.x = x;
        state.y = y;
        state.z = z;
        state.vx = vx;
        state.vy = vy;
        state.vz = vz;
        state.stamp = peer_node_->now();

        for (int attempt = 0; attempt < 40 && !tracker_.all_peers_valid(); ++attempt) {
            publisher->publish(state);
            executor_.spin_some();
            std::this_thread::sleep_for(5ms);
        }
        executor_.spin_some();
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rclcpp::Node> self_node_;
    std::shared_ptr<rclcpp::Node> peer_node_;
    std::vector<rclcpp::Publisher<DroneState>::SharedPtr> publishers_;
    swarm::PeerTracker tracker_;
    swarm::CbfAvoidance cbf_;
};

TEST_F(CbfAvoidanceTest, MissingPeerFailsClosed) {
    configure(2);
    const auto command = cbf_.filter_velocity(
        {2.0f, 0.0f, 0.0f},
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        tracker_);

    EXPECT_FLOAT_EQ(command.x, 0.0f);
    EXPECT_FLOAT_EQ(command.y, 0.0f);
    EXPECT_FLOAT_EQ(command.z, 0.0f);
}

TEST_F(CbfAvoidanceTest, ApproachingPeerLimitsRadialVelocity) {
    configure(2);
    publish_peer(2, 5.0f, 0.0f, 0.0f);
    ASSERT_TRUE(tracker_.all_peers_valid());

    const auto command = cbf_.filter_velocity(
        {3.0f, 0.0f, 0.0f},
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        tracker_);

    EXPECT_LE(command.x, 0.75f);
    EXPECT_LE(magnitude(command), 3.0001f);
}

TEST_F(CbfAvoidanceTest, ExactOverlapProducesFiniteOppositeEscape) {
    configure(2);
    publish_peer(2, 0.0f, 0.0f, 0.0f);
    ASSERT_TRUE(tracker_.all_peers_valid());

    const auto command = cbf_.filter_velocity(
        {},
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        tracker_);

    constexpr float kTwoPi = 6.28318530718f;
    const float angle = kTwoPi * static_cast<float>((1u * 31u + 2u * 17u) % 360u) / 360.0f;
    const float outward_component =
        command.x * std::cos(angle) + command.y * std::sin(angle);

    EXPECT_TRUE(std::isfinite(command.x));
    EXPECT_TRUE(std::isfinite(command.y));
    EXPECT_TRUE(std::isfinite(command.z));
    EXPECT_GT(outward_component, 2.9f);
    EXPECT_LE(magnitude(command), 3.0001f);
}

TEST_F(CbfAvoidanceTest, MultiplePeersRemainInsideSpeedLimit) {
    configure(3);
    publish_peer(2, 4.7f, 0.0f, 0.0f);
    publish_peer(3, 0.0f, 4.7f, 0.0f);
    ASSERT_TRUE(tracker_.all_peers_valid());

    const auto command = cbf_.filter_velocity(
        {2.2f, 2.0f, 0.0f},
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        tracker_);

    EXPECT_TRUE(std::isfinite(command.x));
    EXPECT_TRUE(std::isfinite(command.y));
    EXPECT_TRUE(std::isfinite(command.z));
    EXPECT_LE(command.x, 0.35f);
    EXPECT_LE(command.y, 0.35f);
    EXPECT_LE(magnitude(command), 3.0001f);
}

}
