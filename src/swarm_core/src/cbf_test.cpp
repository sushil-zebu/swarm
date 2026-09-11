#include "swarm_core/cbf_test.hpp"

#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "swarm_core/cbf_avoidance.hpp"
#include "swarm_core/peer_tracker.hpp"
#include "swarm_core/px4_interface.hpp"
#include "swarm_core/swarm_types.hpp"

namespace swarm {

struct CbfTest::Impl {
    enum class Phase : uint8_t {
        WAIT_FOR_PEER,
        WAIT_FOR_ARM,
        ARM_HOLD,
        TAKEOFF,
        MOVE_X,
        HOLD,
        RTL
    };

    Impl(
    rclcpp::Node& node,
    PX4Interface& px4,
    PeerTracker& peers,
    CbfAvoidance& cbf,
    uint8_t drone_id,
    int num_drones,
    float spawn_x,
    float spawn_y,
    float spawn_z);

    void control_loop();

    void enter_phase(Phase next_phase); 

    void publish_safe_velocity_to(
    float target_x,
    float target_y,
    float target_z,
    float speed);


    bool reached_target(
    float target_x,
    float target_y,
    float target_z,
    float tolerance) const;

    float world_x() const;
    float world_y() const;
    float world_z() const;

    void publish_swarm_state();

    rclcpp::Node* node_{nullptr};

    PX4Interface* px4_{nullptr};

    PeerTracker* peers_{nullptr};

    CbfAvoidance* cbf_{nullptr};

    uint8_t drone_id_{1};
    int num_drones_{4};
    float spawn_x_{0.0f};
    float spawn_y_{0.0f};
    float spawn_z_{0.0f};

    float takeoff_height_m_{20.0f};
    float move_x_distance_m_{5.0f};
    float max_speed_mps_{1.0f};
    float reach_tolerance_m_{0.3f};
    

    Phase phase_{Phase::WAIT_FOR_ARM};

    SwarmState swarm_state_{SwarmState::IDLE};

    uint64_t tick_{0};
    uint64_t phase_tick_{0};

    bool peers_ready_logged_{false};
    bool armed_logged_{false};

};


// adding dependencies connection
CbfTest::Impl::Impl(
    rclcpp::Node& node,
    PX4Interface& px4,
    PeerTracker& peers,
    CbfAvoidance& cbf,
    uint8_t drone_id,
    int num_drones,
    float spawn_x,
    float spawn_y,
    float spawn_z)
    : node_(&node),
      px4_(&px4),
      peers_(&peers),
      cbf_(&cbf),
      drone_id_(drone_id),
      num_drones_(num_drones),
      spawn_x_(spawn_x),
      spawn_y_(spawn_y),
      spawn_z_(spawn_z)
{
    RCLCPP_INFO(
        node_->get_logger(),
        "[Cbf MOVE X] Drone %u initialized.",
        drone_id_);
}


// adding spawn position to local position

float CbfTest::Impl::world_x() const {
    return px4_->pos_x() + spawn_x_;
}

float CbfTest::Impl::world_y() const {
    return px4_->pos_y() + spawn_y_;
}

float CbfTest::Impl::world_z() const {
    return px4_->pos_z() + spawn_z_;
}



// checking is drone reached target.
bool CbfTest::Impl::reached_target(
    float target_x,
    float target_y,
    float target_z,
    float tolerance) const
{
    const float dx = world_x() - target_x;
    const float dy = world_y() - target_y;
    const float dz = world_z() - target_z;

    const float distance =
        std::sqrt(
            dx * dx +
            dy * dy +
            dz * dz);

    return distance < tolerance;
}





void CbfTest::Impl::enter_phase(Phase next_phase) {
    phase_ = next_phase;
    phase_tick_ = 0;

    const char* phase_name = "UNKNOWN";

    switch (phase_) {
    case Phase::WAIT_FOR_PEER:
        phase_name = "WAIT_FOR_PEER";
        break;
    case Phase::WAIT_FOR_ARM:
        phase_name = "WAIT_FOR_ARM";
        break;

    case Phase::ARM_HOLD:
        phase_name = "ARM_HOLD";
        break;

    case Phase::TAKEOFF:
        phase_name = "TAKEOFF";
        break;

    case Phase::MOVE_X:
        phase_name = "MOVE_X";
        break;

    case Phase::HOLD:
        phase_name = "HOLD";
        break;

    case Phase::RTL:
        phase_name = "RTL";
        break;
    }
    

    RCLCPP_INFO(
        node_->get_logger(),
        "[CBF TEST] Drone %u entering %s",
        drone_id_,
        phase_name);
}





void CbfTest::Impl::publish_swarm_state()
{
    // First check whether previously discovered peers have timed out.
    peers_->check_heartbeats();

    // Only broadcast our state when our own PX4 information is valid.
    if (px4_->has_fresh_local_position() &&
        px4_->has_fresh_vehicle_status())
    {
        peers_->broadcast_self_state(
            drone_id_,

            world_x(),
            world_y(),
            world_z(),

            px4_->vel_x(),
            px4_->vel_y(),
            px4_->vel_z(),

            px4_->arming_state(),

            swarm_state_,

            {}
        );
    }
    else {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "[CBF TEST] Drone %u waiting for valid PX4 state.",
            drone_id_);
    }
}





void CbfTest::Impl::publish_safe_velocity_to(
    float target_x,
    float target_y,
    float target_z,
    float speed)
{
    const Vec3f safe_velocity =
        cbf_->compute_safe_velocity(
            target_x,
            target_y,
            target_z,
            speed,

            world_x(),
            world_y(),
            world_z(),

            px4_->vel_x(),
            px4_->vel_y(),
            px4_->vel_z(),

            *peers_);

    px4_->publish_velocity_setpoint(
        safe_velocity.x,
        safe_velocity.y,
        safe_velocity.z);
}






void CbfTest::Impl::control_loop()
{
    ++tick_;
    ++phase_tick_;
    publish_swarm_state();

    switch (phase_) {

    case Phase::WAIT_FOR_PEER:
    {
        swarm_state_ = SwarmState::IDLE;

        px4_->publish_velocity_setpoint(
            0.0f,
            0.0f,
            0.0f          
        );

        if (!peers_->all_peers_valid()) {

            RCLCPP_INFO_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                2000,
                "[CBF TEST] Drone %u waiting for all swarm peers...",
                drone_id_);

            break;
        }

        if (!peers_ready_logged_) {
            peers_ready_logged_ = true;

            RCLCPP_INFO(
                node_->get_logger(), "[CBF TEST] Drone %u: ALL PEERS VALID.", drone_id_
            );
        }

        enter_phase(Phase::WAIT_FOR_ARM);

        break;

    }

    case Phase::WAIT_FOR_ARM:{
        swarm_state_ = SwarmState::IDLE;

        px4_->publish_velocity_setpoint(
            0.0f,
            0.0f,
            0.0f
        );


        if (!peers_->all_peers_valid()) {

            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                1000,
                "[CBF TEST] Drone %u lost peer communication before ARM.",
                drone_id_);

            enter_phase(Phase::WAIT_FOR_PEER);

            break;
        }

        if (phase_tick_ < 10 ) {
            break;
        }


        if (!px4_->is_in_offboard()) {
            px4_->engage_offboard();
        }


        if (!px4_->is_armed()) {
            px4_->arm();
        }


        if (px4_->is_armed() &&
            px4_->is_in_offboard())
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "[CBF TEST] Drone %u ARMED and OFFBOARD.",
                drone_id_);

            enter_phase(Phase::ARM_HOLD);
        }

        break;
    }

    case Phase::ARM_HOLD: {

        swarm_state_ = SwarmState::HOLD;

        px4_->publish_velocity_setpoint(
            0.0f,
            0.0f,
            0.0f);


        if (!armed_logged_) {

            armed_logged_ = true;

            RCLCPP_INFO(
                node_->get_logger(),
                "[CBF TEST] Drone %u arming test SUCCESS. Holding.",
                drone_id_);
        }

        // enter_phase(Phase::TAKEOFF)

        break;
    }
    case Phase::TAKEOFF:{
        break;
    }
    case Phase::MOVE_X:{
        break;
    }
    case Phase::HOLD:{
        break;
    }
    case Phase::RTL:{
        break;
    }



    // case Phase::TAKEOFF: {

    // swarm_state_ = SwarmState::TAKEOFF;

    //     px4_->publish_velocity_setpoint(
    //          spawn_x_ , spawn_y_ , takeoff_height_m_, reach_tolerance_m_);    
    // }



    // case Phase::MOVE_X: {
    // swarm_state_ = SwarmState::SURVEYING;

    //     px4_->publish_velocity_setpoint(
    //         move_x_distance_m_ , spawn_y_ , takeoff_height_m_, reach_tolerance_m_);    
    // }
    
    // case Phase::HOLD: {

    //     swarm_state_ = SwarmState::HOLD;
    //     px4_->publish_velocity_setpoint

    // }

}
}

CbfTest::CbfTest(
    rclcpp::Node& node,
    PX4Interface& px4,
    PeerTracker& peers,
    CbfAvoidance& cbf,
    uint8_t drone_id,
    int num_drones,
    float spawn_x,
    float spawn_y,
    float spawn_z)

    : impl_(
        std::make_unique<Impl>(
            node,
            px4,
            peers,
            cbf,
            drone_id,
            num_drones,
            spawn_x,
            spawn_y,
            spawn_z))
{
}


CbfTest::~CbfTest() = default;


void CbfTest::control_loop() {
    impl_->control_loop();
}

}