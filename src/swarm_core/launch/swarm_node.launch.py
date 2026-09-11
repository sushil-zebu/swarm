import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('swarm_core'), 'config', 'swarm_params.yaml')

    drone_id_arg = DeclareLaunchArgument('drone_id', default_value='1', description='Drone Swarm ID')
    mav_sys_id_arg = DeclareLaunchArgument('mav_sys_id', default_value='2', description='MAVLink System ID')
    num_drones_arg = DeclareLaunchArgument('num_drones', default_value='3', description='Total Drones in Swarm')
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0', description='World Spawn Offset X')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0', description='World Spawn Offset Y')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.0', description='World Spawn Offset Z')

    drone_id = LaunchConfiguration('drone_id')
    mav_sys_id = LaunchConfiguration('mav_sys_id')
    num_drones = LaunchConfiguration('num_drones')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

    return LaunchDescription([
        drone_id_arg,
        mav_sys_id_arg,
        num_drones_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        Node(
            package='swarm_core',
            executable='drone_controller_node',
            name=['drone_controller_', drone_id],
            namespace=['drone_', drone_id],
            parameters=[params_file, {
                'drone_id': drone_id,
                'mav_sys_id': mav_sys_id,
                'num_drones': num_drones,
                'spawn_x': spawn_x,
                'spawn_y': spawn_y,
                'spawn_z': spawn_z,
            }],
            output='screen',
        )
    ])
