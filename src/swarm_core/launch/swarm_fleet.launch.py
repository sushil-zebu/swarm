import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []
    params_file = os.path.join(
        get_package_share_directory('swarm_core'), 'config', 'swarm_params.yaml')

    # nodes.append(Node(
    #     package='swarm_core',
    #     executable='swarm_arming_manager',
    #     name='swarm_arming_manager',
    #     parameters=[params_file],
    #     output='screen'
    # ))

    spawn_offsets = {
        1: (0.0, 0.0, 0.0),
        2: (7.0, 0.0, 0.0),
        3: (14.0, 0.0, 0.0),
        4: (21.0, 0.0, 0.0)
    }

    for drone_id in [1, 2, 3, 4]:
        mav_sys_id = drone_id + 1
        sx, sy, sz = spawn_offsets.get(drone_id, (0.0, 0.0, 0.0))
        nodes.append(Node(
            package='swarm_core',
            executable=f'drone_{drone_id}_node',
            name=f'drone_controller_{drone_id}',
            namespace=f'drone_{drone_id}',
            parameters=[params_file, {
                'drone_id': drone_id,
                'mav_sys_id': mav_sys_id,
                'spawn_x': sx,
                'spawn_y': sy,
                'spawn_z': sz,
            }],
            output='screen'
        ))

    return LaunchDescription(nodes)
