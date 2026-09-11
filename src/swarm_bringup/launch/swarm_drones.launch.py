"""
swarm_drones.launch.py
======================
Top-level bringup launch file for the decentralized drone swarm.

Startup sequence (automatic, no manual steps needed):
  1. swarm_arming_manager starts first — monitors drone mesh health
  2. Four independent drone nodes start in parallel (namespaced /drone_N/)
  3. Each node executes the same decentralized vertical-stack helix controller.

To run:
  ros2 launch swarm_bringup swarm_drones.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('swarm_core'), 'config', 'swarm_params.yaml')

    # ── Arming manager ─────────────────────────────────────────────────────────
    arming_manager = Node(
        package='swarm_core',
        executable='swarm_arming_manager',
        name='swarm_arming_manager',
        output='screen',
        parameters=[params_file],
    )

    # ── Drone fleet (Gazebo World X spawns: 0m, 7m, 14m, 21m) ─────────────────
    fleet = [
        {'namespace': 'drone_1', 'executable': 'drone_1_node', 'drone_id': 1, 'mav_sys_id': 2, 'spawn_x': 0.0, 'spawn_y': 0.0, 'spawn_z': 0.0},
        {'namespace': 'drone_2', 'executable': 'drone_2_node', 'drone_id': 2, 'mav_sys_id': 3, 'spawn_x': 7.0, 'spawn_y': 0.0, 'spawn_z': 0.0},
        {'namespace': 'drone_3', 'executable': 'drone_3_node', 'drone_id': 3, 'mav_sys_id': 4, 'spawn_x': 14.0, 'spawn_y': 0.0, 'spawn_z': 0.0},
        {'namespace': 'drone_4', 'executable': 'drone_4_node', 'drone_id': 4, 'mav_sys_id': 5, 'spawn_x': 21.0, 'spawn_y': 0.0, 'spawn_z': 0.0},
    ]

    drone_nodes = [
        Node(
            package='swarm_core',
            executable=drone['executable'],
            namespace=drone['namespace'],
            name=f"drone_controller_{drone['drone_id']}",
            output='screen',
            parameters=[params_file, {
                'drone_id': drone['drone_id'],
                'mav_sys_id': drone['mav_sys_id'],
                'spawn_x': drone['spawn_x'],
                'spawn_y': drone['spawn_y'],
                'spawn_z': drone['spawn_z'],
            }],
        )
        for drone in fleet
    ]

    return LaunchDescription([arming_manager] + drone_nodes)
