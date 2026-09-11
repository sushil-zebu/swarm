#pragma once
#include <cstdint>
#include <vector>
#include <rclcpp/time.hpp>

namespace swarm {

struct Vec3f {
    float x{0.0f}, y{0.0f}, z{0.0f};

    Vec3f operator+(const Vec3f& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3f operator-(const Vec3f& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3f operator*(float s)        const { return {x*s,   y*s,   z*s};   }
};

struct PeerState {
    float   x{0.0f}, y{0.0f}, z{0.0f};    // local NED position [m]
    float   vx{0.0f}, vy{0.0f}, vz{0.0f}; // velocity [m/s]
    uint8_t arming_state{0};              // PX4 arming state
    uint8_t swarm_state{0};               // peer's SwarmState enum value
    std::vector<uint32_t> completed_task_ids; // tasks completed by peer

    bool         alive{false};            // true = peer considered healthy
    bool         ever_seen{false};        // true once we receive first message
    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};   // ROS time of last message
    rclcpp::Time first_seen{0, 0, RCL_ROS_TIME};  // ROS time of first message
    rclcpp::Time state_stamp{0, 0, RCL_ROS_TIME}; // source time carried by the message

    int  missed_count{0};   // consecutive ticks with no message (→ DEAD)
    int  recv_streak{0};    // consecutive ticks with a message  (→ ALIVE)

    float link_quality{0.0f};  // [0.0 = dead, 1.0 = perfect]
};

enum class SwarmState : uint8_t {
    IDLE      = 0,
    TAKEOFF   = 1,
    HOLD      = 2,
    SURVEYING = 3,
    RETURN    = 4,
    LAND      = 5,
    DONE      = 6
};

inline const char* to_str(SwarmState s) {
    static constexpr const char* names[] = {
        "IDLE", "TAKEOFF", "HOLD", "SURVEYING", "RETURN", "LAND", "DONE"
    };
    auto idx = static_cast<int>(s);
    if (idx < 0 || idx > 6) return "UNKNOWN";
    return names[idx];
}

}
