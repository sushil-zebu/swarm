#pragma once
#include <chrono>
#include <algorithm>
#include <map>
#include <vector>
#include <string>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <swarm_interfaces/msg/drone_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "swarm_types.hpp"
#include "collision_avoidance.hpp"

using namespace std::chrono_literals;

namespace swarm {

class SwarmControllerBase : public rclcpp::Node {
public:

    // ═══════════════════════════════════════════════════════════════
    //  Construction
    //
    //  @param node_name    ROS 2 node name  (e.g. "drone_controller_1")
    //  @param self_id      Swarm identity   (1, 2, 3, ...)
    //  @param mav_sys_id   PX4 MAVLink sys  (px4_instance + 1)
    //  @param peer_topics  Absolute ROS topic paths of peer DroneState publishers
    // ═══════════════════════════════════════════════════════════════
    SwarmControllerBase(
        const std::string&              node_name,
        uint8_t                         self_id,
        int                             mav_sys_id,
        const std::vector<std::string>& peer_topics)
    : Node(node_name)
    , self_id_(self_id)
    , mav_sys_id_(mav_sys_id)
    , state_(SwarmState::IDLE)
    {
        load_parameters();
        setup_publishers();
        setup_subscribers(peer_topics);

        timer_ = create_wall_timer(100ms,
            std::bind(&SwarmControllerBase::control_loop, this));

        RCLCPP_INFO(get_logger(),
            "[Drone %u] Swarm core ready | MAV_SYS_ID=%d | "
            "spawn=(%.1f,%.1f) | alt=%.1fm | safe_r=%.1fm | "
            "dead_thr=%d ticks | alive_thr=%d ticks",
            self_id_, mav_sys_id_,
            spawn_x_, spawn_y_, -target_altitude_,
            safe_radius_, dead_threshold_, alive_threshold_);
    }

protected:

    // ═══════════════════════════════════════════════════════════════
    // §1  PEER MESH
    //
    //  Fully decentralized: every drone broadcasts its own state at
    //  10 Hz on "swarm/self_state" (relative → resolved under its
    //  ROS namespace, e.g. /drone_1/swarm/self_state).
    //
    //  Every other drone subscribes to the absolute topic paths of
    //  its peers and calls on_peer_msg() on each received message.
    //
    //  No central relay, no shared memory, no coordinator needed.
    // ═══════════════════════════════════════════════════════════════

    // ── Receive a peer's DroneState broadcast ───────────────────────
    void on_peer_msg(const swarm_interfaces::msg::DroneState::SharedPtr& m) {
        auto& p = peers_[m->drone_id];

        // Update kinematics
        p.x            = m->x;   p.y  = m->y;   p.z  = m->z;
        p.vx           = m->vx;  p.vy = m->vy;  p.vz = m->vz;
        p.arming_state = m->arming_state;
        p.swarm_state  = m->swarm_state;
        p.last_seen    = now();

        // First contact with this peer
        if (!p.ever_seen) {
            p.ever_seen  = true;
            p.first_seen = now();
            RCLCPP_INFO(get_logger(),
                "[MESH] Drone %u appeared for the first time.", m->drone_id);
        }

        // ── Hysteresis: accumulate reception streak ─────────────────
        p.missed_count = 0;      // reset miss counter
        p.recv_streak++;         // extend consecutive receive streak

        // ── Link quality EMA (alpha=0.85, received this tick = 1.0) ─
        p.link_quality = LINK_ALPHA * p.link_quality + (1.0f - LINK_ALPHA) * 1.0f;

        // ── Recovery transition: DEAD → ALIVE ──────────────────────
        // Require alive_threshold_ consecutive messages before declaring
        // the peer healthy again. Prevents single-packet false-positive.
        if (!p.alive && p.recv_streak >= alive_threshold_) {
            p.alive = true;
            RCLCPP_INFO(get_logger(),
                "[MESH] Drone %u RECOVERED | quality=%.0f%% | streak=%d msgs",
                m->drone_id, p.link_quality * 100.0f, p.recv_streak);
        }
    }

    // ── Broadcast own position + state to the swarm mesh ────────────
    void broadcast_state() {
        swarm_interfaces::msg::DroneState msg;
        msg.drone_id     = self_id_;
        // Convert local PX4 frame to global NED (add spawn offset)
        msg.x            = self_pos_.x + spawn_x_;
        msg.y            = self_pos_.y + spawn_y_;
        msg.z            = self_pos_.z;
        msg.vx           = vel_.x;  msg.vy = vel_.y;  msg.vz = vel_.z;
        msg.arming_state = arming_state_;
        msg.swarm_state  = static_cast<uint8_t>(state_);
        msg.stamp        = now();
        swarm_pub_->publish(msg);
    }

    // ═══════════════════════════════════════════════════════════════
    // §2  HEARTBEAT — Robust Failure Detection
    //
    //  Called every control tick (10 Hz) BEFORE the state machine.
    //
    //  Design:
    //    • A peer is marked DEAD only after dead_threshold_ consecutive
    //      ticks with no message (default 5 = 500 ms). This tolerates
    //      brief RF dropouts and scheduling jitter without false alarms.
    //
    //    • link_quality decays toward 0.0 on every missed tick via EMA,
    //      giving a continuous signal health indicator even while the
    //      peer is still technically "alive".
    //
    //    • A peer that was never seen at all (ever_seen=false) is silently
    //      skipped — it may simply not be part of this mission.
    // ═══════════════════════════════════════════════════════════════

    void update_peer_heartbeats() {
        for (auto& [id, peer] : peers_) {
            // Skip peers we have never heard from
            if (!peer.ever_seen) continue;

            // Determine if a message was expected but missing this tick.
            // We use 1.5x the broadcast period (150 ms) as the jitter
            // tolerance window — accommodates OS scheduler latency.
            const bool msg_missing =
                (now() - peer.last_seen).seconds() > MISS_WINDOW_S;

            if (msg_missing) {
                // ── Count the miss ───────────────────────────────────
                peer.missed_count++;
                peer.recv_streak = 0;   // break consecutive receive streak

                // Link quality decays toward 0.0 (missed tick = 0.0)
                peer.link_quality =
                    LINK_ALPHA * peer.link_quality + (1.0f - LINK_ALPHA) * 0.0f;

                // ── Death transition: ALIVE → DEAD ───────────────────
                // Only trigger after dead_threshold_ consecutive misses.
                if (peer.alive && peer.missed_count >= dead_threshold_) {
                    peer.alive = false;
                    RCLCPP_WARN(get_logger(),
                        "[MESH] Drone %u LOST | %d consecutive missed ticks | "
                        "last seen %.2f s ago | quality=%.0f%%",
                        id, peer.missed_count,
                        (now() - peer.last_seen).seconds(),
                        peer.link_quality * 100.0f);
                }

            } else {
                // Message received this tick — handled in on_peer_msg().
                // Nothing extra needed here; the EMA and streak are
                // updated there to avoid double-counting.
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // §3  LEADER ELECTION — Bully Algorithm (fully decentralized)
    //
    //  Rule: the alive drone with the lowest swarm ID is the leader.
    //
    //  Every drone independently computes the same answer from its
    //  local peers_ map — no election messages, no coordinator,
    //  no consensus round needed.
    //
    //  When the leader fails (missed_count >= dead_threshold_), it
    //  is removed from alive_ids() and the next-lowest ID becomes
    //  leader automatically on the very next control tick.
    // ═══════════════════════════════════════════════════════════════

    // Returns sorted IDs of all alive drones (self always included).
    std::vector<uint8_t> alive_ids() const {
        std::vector<uint8_t> ids{self_id_};
        for (const auto& [id, peer] : peers_)
            if (peer.alive) ids.push_back(id);
        std::sort(ids.begin(), ids.end());
        return ids;
    }

    // Returns the ID of the current leader (lowest alive ID).
    uint8_t get_leader_id() const {
        auto ids = alive_ids();
        return ids.empty() ? self_id_ : ids.front();
    }

    // Returns true if this drone is currently the swarm leader.
    bool is_leader() const {
        return self_id_ == get_leader_id();
    }

    // Returns count of alive peers (excluding self).
    size_t peer_count() const {
        return alive_ids().size() - 1;
    }

    // ═══════════════════════════════════════════════════════════════
    // §4  COLLISION AVOIDANCE (APF)
    //
    //  Delegated entirely to collision_avoidance.hpp (pure math,
    //  no ROS, no state). Called every HOLD tick.
    //
    //  Returns a NED correction vector [m] to add to the raw setpoint.
    //  Only alive peers contribute repulsion.
    // ═══════════════════════════════════════════════════════════════

    Vec3f apf_correction() const {
        Vec3f self_global = {
            self_pos_.x + spawn_x_,
            self_pos_.y + spawn_y_,
            self_pos_.z
        };
        return compute_apf_avoidance(self_global, peers_, safe_radius_, apf_gain_);
    }

    // ═══════════════════════════════════════════════════════════════
    // §5  CORE FLIGHT STATE MACHINE  (10 Hz)
    //
    //   IDLE --(armed+offboard)--> TAKEOFF --(alt OK)--> HOLD
    //                                                       |
    //                                         on_hold_tick() hook
    //                                         (override in subclass)
    //                                               |
    //                                          SURVEYING (lawnmower)
    //                                               |
    //                                            LAND --> DONE
    //
    // ═══════════════════════════════════════════════════════════════

    // ── Mission extension hooks (override in subclasses) ─────────
    // Called every tick in HOLD state. Subclass may update hold_target_
    // or call transition() to move to SURVEYING.
    virtual void on_hold_tick() {}

    // Called every tick in SURVEYING state. Subclass drives the
    // lawnmower / waypoint sequence and eventually calls transition(LAND)
    // or transition(HOLD) when the survey is complete.
    virtual void on_survey_tick() {}

    void control_loop() {
        tick_++;

        // PX4 requires OffboardControlMode streamed at > 2 Hz.
        // Suppress during PX4-managed auto-land (nav_state=18) to
        // avoid toggling back to OFFBOARD mid-descent.
        if (state_ != SwarmState::LAND || nav_state_ != 18) {
            publish_offboard_heartbeat();
        }

        update_peer_heartbeats();   // §2 — failure detection every tick
        broadcast_state();          // §1 — send our state to the mesh

        // ── External disarm guard ────────────────────────────────────
        if (state_ != SwarmState::IDLE && arming_state_ != 2) {
            RCLCPP_WARN(get_logger(),
                "[Drone %u] Disarmed externally — resetting to IDLE.", self_id_);
            transition(SwarmState::IDLE);
            tick_ = 0;
        }

        // ── State machine ────────────────────────────────────────────
        switch (state_) {

        // ── IDLE: warm up PX4, wait for arm clearance, then arm ────
        // Arm clearance comes from swarm_arming_manager after it has
        // confirmed the mesh is healthy (all drones visible & stable).
        // Stream a zero setpoint for 1 s (10 ticks) first so PX4
        // accepts OFFBOARD mode when we request it.
        case SwarmState::IDLE:
            move_toward({0.0f, 0.0f, 0.0f});
            if (tick_ >= 10) {
                if (arming_state_ == 2 && nav_state_ == 14) {
                    RCLCPP_INFO(get_logger(),
                        "[Drone %u] Armed & OFFBOARD — starting takeoff.", self_id_);
                    transition(SwarmState::TAKEOFF);
                } else if (arm_cleared_ && tick_ % 20 == 0) {
                    // Arming manager has confirmed mesh is healthy.
                    // Retry every 2 s (too frequent → PX4 pre-flight disarm race).
                    RCLCPP_INFO(get_logger(),
                        "[Drone %u] IDLE: arm=%u nav=%u — arm clearance received, arming...",
                        self_id_, arming_state_, nav_state_);
                    engage_offboard();
                    arm();
                } else if (!arm_cleared_ && tick_ % 20 == 0) {
                    RCLCPP_INFO(get_logger(),
                        "[Drone %u] IDLE: waiting for arm clearance from swarm_arming_manager...",
                        self_id_);
                }
            }
            break;

        // ── TAKEOFF: rate-limited climb to target altitude ───────────
        case SwarmState::TAKEOFF:
            move_toward({0.0f, 0.0f, target_altitude_});
            if (reached({0.0f, 0.0f, target_altitude_})) {
                RCLCPP_INFO(get_logger(),
                    "[Drone %u] Reached hold altitude %.1f m.", self_id_, -target_altitude_);
                transition(SwarmState::HOLD);
            }
            break;

        // ── HOLD: hover with APF separation ─────────────────────────
        // hold_target_ defaults to takeoff position.
        // on_hold_tick() is called first so subclasses can update
        // hold_target_ or trigger a state transition to SURVEYING.
        case SwarmState::HOLD: {
            on_hold_tick();
            Vec3f avoid  = apf_correction();
            Vec3f target = hold_target_ + avoid;
            move_toward(target);
            break;
        }

        // ── SURVEYING: mission subclass drives the drone ─────────────
        // APF avoidance still active. on_survey_tick() advances the
        // lawnmower waypoint sequence, overrides hold_target_, and
        // transitions out (to HOLD or LAND) when survey is complete.
        case SwarmState::SURVEYING: {
            on_survey_tick();
            Vec3f avoid  = apf_correction();
            Vec3f target = hold_target_ + avoid;
            move_toward(target);
            break;
        }

        // ── LAND: delegate descent to PX4 NAV_LAND ──────────────────
        case SwarmState::LAND:
            if (nav_state_ != 17) land_cmd();
            if (arming_state_ == 1) {
                RCLCPP_INFO(get_logger(),
                    "[Drone %u] Disarmed after landing — DONE.", self_id_);
                transition(SwarmState::DONE);
            }
            break;

        case SwarmState::DONE:
        default:
            break;
        }

        // ── 2-second diagnostic log ──────────────────────────────────
        if (tick_ % 20 == 0) {
            log_swarm_status();
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  Diagnostic log — prints full peer table every 2 s
    // ═══════════════════════════════════════════════════════════════

    void log_swarm_status() const {
        RCLCPP_INFO(get_logger(),
            "[%s] pos=(%.1f,%.1f,%.1fm AGL) | arm=%u | leader=%u | peers=%zu",
            to_str(state_),
            self_pos_.x, self_pos_.y, -self_pos_.z,
            arming_state_, get_leader_id(), peer_count());

        for (const auto& [id, p] : peers_) {
            if (!p.ever_seen) continue;
            RCLCPP_INFO(get_logger(),
                "  Peer %u: %s | quality=%.0f%% | pos=(%.1f,%.1f,%.1fm) | "
                "arm=%u | state=%u | missed=%d | streak=%d",
                id,
                p.alive ? "ALIVE" : "DEAD ",
                p.link_quality * 100.0f,
                p.x, p.y, -p.z,
                p.arming_state, p.swarm_state,
                p.missed_count, p.recv_streak);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  Motion helpers
    // ═══════════════════════════════════════════════════════════════

    // True when 3-D distance to target < reach_tol_
    bool reached(const Vec3f& target) const {
        Vec3f d = self_pos_ - target;
        return std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z) < reach_tol_;
    }

    // Rate-limited setpoint: slides sp_ toward target at cruise_speed_ m/s.
    // Prevents PX4 from receiving a far-away waypoint and pitching to max speed.
    void move_toward(const Vec3f& target) {
        constexpr float DT       = 0.1f;
        const     float max_step = cruise_speed_ * DT;

        Vec3f delta = target - sp_;
        float dist  = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);

        if (dist <= max_step || dist < 0.001f) {
            sp_ = target;
        } else {
            float s = max_step / dist;
            sp_.x  += delta.x * s;
            sp_.y  += delta.y * s;
            sp_.z  += delta.z * s;
        }
        publish_setpoint(sp_);
    }

    // Elapsed seconds since a reference ROS time
    double elapsed(const rclcpp::Time& ref) const {
        return (now() - ref).seconds();
    }

    // Log and execute a state transition
    void transition(SwarmState next) {
        RCLCPP_INFO(get_logger(),
            "[Drone %u] %s -> %s", self_id_, to_str(state_), to_str(next));
        state_ = next;
    }

    // ═══════════════════════════════════════════════════════════════
    //  PX4 interface helpers
    // ═══════════════════════════════════════════════════════════════

    // Must be streamed at >2 Hz or PX4 exits OFFBOARD mode automatically
    void publish_offboard_heartbeat() {
        px4_msgs::msg::OffboardControlMode m{};
        m.position  = true;
        m.timestamp = now().nanoseconds() / 1000;
        offboard_pub_->publish(m);
    }

    // Publish a NED position setpoint to PX4
    void publish_setpoint(const Vec3f& sp) {
        px4_msgs::msg::TrajectorySetpoint m{};
        m.position  = {sp.x, sp.y, sp.z};
        m.yaw       = -1.5708f;   // face East; override in subclass if needed
        m.timestamp = now().nanoseconds() / 1000;
        setpoint_pub_->publish(m);
    }

    // Send any MAVLink VehicleCommand to PX4
    void send_cmd(uint16_t cmd, float p1 = 0.0f, float p2 = 0.0f) {
        px4_msgs::msg::VehicleCommand m{};
        m.command          = cmd;
        m.param1           = p1;
        m.param2           = p2;
        m.target_system    = mav_sys_id_;
        m.target_component = 1;
        m.source_system    = 1;
        m.source_component = 1;
        m.from_external    = true;
        m.timestamp        = now().nanoseconds() / 1000;
        command_pub_->publish(m);
    }

    void arm()             { send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1); }
    void disarm()          { send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0); }
    void engage_offboard() { send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); }
    void land_cmd()        { send_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); }

    // ═══════════════════════════════════════════════════════════════
    //  Identity — provided by each concrete drone subclass
    // ═══════════════════════════════════════════════════════════════
    const uint8_t self_id_;
    const int     mav_sys_id_;

    // ═══════════════════════════════════════════════════════════════
    //  Parameters — all loaded from mission.yaml at launch
    // ═══════════════════════════════════════════════════════════════
    float  target_altitude_{ -15.0f}; // NED Z hold altitude (negative=up) [m]
    float  safe_radius_    {   5.0f}; // APF safety radius [m]
    float  apf_gain_       {   3.0f}; // APF repulsion gain (tune up = stronger push)
    float  reach_tol_      {   0.5f}; // 3-D arrival tolerance [m]
    float  cruise_speed_   {   2.0f}; // max setpoint travel speed [m/s]
    float  spawn_x_        {   0.0f}; // North spawn offset [m] (global frame)
    float  spawn_y_        {   0.0f}; // East spawn offset [m]
    int    dead_threshold_ {   5   }; // consecutive missed ticks before DEAD (5=500ms)
    int    alive_threshold_{   3   }; // consecutive received msgs before ALIVE (3=300ms)

    // Set to true when swarm_arming_manager publishes arm clearance.
    // Drones will NOT attempt to arm until this is true.
    bool   arm_cleared_{false};

    // ═══════════════════════════════════════════════════════════════
    //  Runtime state
    // ═══════════════════════════════════════════════════════════════
    SwarmState               state_;
    std::map<uint8_t, PeerState> peers_;   // key = peer swarm ID

    Vec3f   self_pos_{};      // current local NED position (from PX4)
    Vec3f   vel_{};           // current NED velocity (from PX4)
    Vec3f   sp_{};            // active rate-limited setpoint being sent to PX4
    Vec3f   hold_target_{};   // hover position — overridable by plugins

    uint8_t arming_state_{0}; // PX4 arming state  (1=disarmed, 2=armed)
    uint8_t nav_state_{0};    // PX4 nav state     (14=offboard, 17=land, 18=auto-land)
    int     tick_{0};         // control loop tick counter

    // ═══════════════════════════════════════════════════════════════
    //  ROS 2 entities
    // ═══════════════════════════════════════════════════════════════
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      command_pub_;
    rclcpp::Publisher<swarm_interfaces::msg::DroneState>::SharedPtr  swarm_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                path_pub_;
    nav_msgs::msg::Path                                              path_msg_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    marker_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr        status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                 arm_clearance_sub_;

    std::vector<rclcpp::Subscription<
        swarm_interfaces::msg::DroneState>::SharedPtr> peer_subs_;

    rclcpp::TimerBase::SharedPtr timer_;

private:

    // ── Heartbeat constants ──────────────────────────────────────────
    // EMA smoothing factor for link quality (0.85 → ~700 ms time constant)
    static constexpr float  LINK_ALPHA   = 0.85f;
    // Miss detection window: 1.5x the 100ms broadcast period
    static constexpr double MISS_WINDOW_S = 0.15;

    // ── Constructor helpers ──────────────────────────────────────────

    void load_parameters() {
        declare_parameter("target_altitude",  -15.0);
        declare_parameter("safe_radius",        5.0);
        declare_parameter("apf_gain",           3.0);
        declare_parameter("reach_tolerance",    0.5);
        declare_parameter("cruise_speed",       2.0);
        declare_parameter("spawn_x",            0.0);
        declare_parameter("spawn_y",            0.0);
        declare_parameter("dead_threshold",     5);   // ticks (int)
        declare_parameter("alive_threshold",    3);   // ticks (int)

        target_altitude_ = static_cast<float>(get_parameter("target_altitude").as_double());
        safe_radius_     = static_cast<float>(get_parameter("safe_radius").as_double());
        apf_gain_        = static_cast<float>(get_parameter("apf_gain").as_double());
        reach_tol_       = static_cast<float>(get_parameter("reach_tolerance").as_double());
        cruise_speed_    = static_cast<float>(get_parameter("cruise_speed").as_double());
        spawn_x_         = static_cast<float>(get_parameter("spawn_x").as_double());
        spawn_y_         = static_cast<float>(get_parameter("spawn_y").as_double());
        dead_threshold_  = get_parameter("dead_threshold").as_int();
        alive_threshold_ = get_parameter("alive_threshold").as_int();

        hold_target_ = {0.0f, 0.0f, target_altitude_};
    }

    void setup_publishers() {
        auto px4_qos = rclcpp::QoS(1).best_effort().transient_local();

        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "fmu/in/offboard_control_mode", px4_qos);
        setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "fmu/in/trajectory_setpoint", px4_qos);
        command_pub_  = create_publisher<px4_msgs::msg::VehicleCommand>(
            "fmu/in/vehicle_command", px4_qos);

        swarm_pub_ = create_publisher<swarm_interfaces::msg::DroneState>(
            "swarm/self_state", rclcpp::SystemDefaultsQoS());

        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "visual_path", rclcpp::SystemDefaultsQoS());
    }

    void setup_subscribers(const std::vector<std::string>& peer_topics) {
        auto px4_qos = rclcpp::QoS(1).best_effort().transient_local();

        // PX4 vehicle status — arming & nav state
        status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "fmu/out/vehicle_status_v4", px4_qos,
            [this](px4_msgs::msg::VehicleStatus::SharedPtr m) {
                arming_state_ = m->arming_state;
                nav_state_    = m->nav_state;
            });

        // PX4 local NED position + velocity
        pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "fmu/out/vehicle_local_position_v1", px4_qos,
            [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
                self_pos_ = {m->x, m->y, m->z};
                vel_      = {m->vx, m->vy, m->vz};

                // Append current position to path message (transforming to global NED frame)
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = now();
                pose.header.frame_id = "map";
                pose.pose.position.x = m->x + spawn_x_;
                pose.pose.position.y = m->y + spawn_y_;
                pose.pose.position.z = m->z;
                pose.pose.orientation.w = 1.0;

                path_msg_.header.stamp = now();
                path_msg_.header.frame_id = "map";
                path_msg_.poses.push_back(pose);

                // Limit path length to 4000 points to prevent memory overflow
                if (path_msg_.poses.size() > 4000) {
                    path_msg_.poses.erase(path_msg_.poses.begin());
                }

                path_pub_->publish(path_msg_);
            });

        // Swarm mesh — one subscription per peer drone (absolute topics)
        for (const auto& topic : peer_topics) {
            auto sub = create_subscription<swarm_interfaces::msg::DroneState>(
                topic, rclcpp::SystemDefaultsQoS(),
                [this](swarm_interfaces::msg::DroneState::SharedPtr m) {
                    on_peer_msg(m);
                });
            peer_subs_.push_back(sub);
            RCLCPP_INFO(get_logger(), "[MESH] Subscribed to peer: %s", topic.c_str());
        }

        // Arm clearance — published by swarm_arming_manager once mesh is healthy.
        // Latched (transient_local) so late-joining drones get it immediately.
        auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
        arm_clearance_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/swarm/arm_clearance", latched_qos,
            [this](std_msgs::msg::Bool::SharedPtr m) {
                if (m->data && !arm_cleared_) {
                    arm_cleared_ = true;
                    RCLCPP_INFO(get_logger(),
                        "[Drone %u] ARM CLEARANCE received from arming manager!",
                        self_id_);
                }
            });
    }
};

}