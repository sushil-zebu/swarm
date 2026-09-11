#pragma once
/*
 * collision_avoidance.hpp
 *
 * Standalone 3D Artificial Potential Field (APF) Collision Avoidance Library
 *
 * ── Theoretical Overview for Technical & Management Review ──────────
 *
 * 1. Artificial Potential Fields (APF):
 *    Each drone acts as a charged particle surrounded by a spherical virtual
 *    repulsion field of radius `safe_radius` (default 5.0m).
 *
 * 2. Repulsive Force Formula:
 *    When distance d_3d < safe_radius:
 *       F_rep = k_rep * (1 / d_3d - 1 / safe_radius) / (d_3d^2)
 *    This produces an inverse-square force pushing drones apart as they get closer.
 *
 * 3. 3D Altitude Deconfliction (Vertical Layering):
 *    Traditional 2D APF can deadlock if two drones meet head-on at identical (x,y) coordinates.
 *    To prevent deadlock, 3D Altitude Deconfliction dynamically separates drones vertically:
 *       - Smaller Drone ID (e.g. Drone 1) climbs UP   (-Z in NED frame)
 *       - Larger Drone ID  (e.g. Drone 2) descends DOWN (+Z in NED frame)
 *    This guarantees zero collision risk and smooth horizontal passage!
 */

#include "swarm_types.hpp"
#include <cmath>
#include <map>
#include <algorithm>

namespace swarm {

    /**
     * @brief Computes 3D APF Avoidance Vector for a drone relative to its active peers.
     * 
     * @param self_id     Swarm ID of this drone (1, 2, 3...)
     * @param self_pos    Current 3D position vector in local NED frame (metres)
     * @param peers       Map of active peer states
     * @param safe_radius Trigger distance threshold for repulsion (metres, default 5.0m)
     * @param k_rep       Repulsion strength gain multiplier (default 3.0)
     * @return Vec3f      Additive (dx, dy, dz) offset vector to apply to trajectory setpoint
     */
    inline Vec3f compute_apf_avoidance_3d(
        uint8_t self_id,
        const Vec3f& self_pos,
        const std::map<uint8_t, PeerState>& peers,
        float safe_radius = 5.0f,
        float k_rep = 3.0f)
    {
        Vec3f v{0.0f, 0.0f, 0.0f};

        for (const auto& [id, peer] : peers) {
            // Ignore offline or uninitialized peers
            if (!peer.ever_seen || !peer.alive) continue;

            float dx = self_pos.x - peer.x;
            float dy = self_pos.y - peer.y;
            float dz = self_pos.z - peer.z;
            float d_2d = std::hypot(dx, dy);
            float d_3d = std::sqrt(dx*dx + dy*dy + dz*dz);

            // Apply repulsion if within safe_radius
            if (d_3d < safe_radius && d_3d > 0.05f) {
                // 1. Horizontal Repulsion Force Vector
                float mag = k_rep * (1.0f / d_3d - 1.0f / safe_radius) / (d_3d * d_3d);
                v.x += mag * dx / d_3d;
                v.y += mag * dy / d_3d;

                // 2. 3D Altitude Deconfliction (Vertical Layering)
                if (d_2d < safe_radius) {
                    float vert_factor = 1.5f * (1.0f - d_2d / safe_radius);
                    if (self_id < id) {
                        v.z -= vert_factor; // Smaller Drone ID climbs UP (-z in NED)
                    } else {
                        v.z += vert_factor; // Larger Drone ID descends DOWN (+z in NED)
                    }
                } else {
                    v.z += mag * dz / d_3d;
                }
            }
        }

        return v;
    }

} // namespace swarm