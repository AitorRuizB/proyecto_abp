import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav2_yaml_0 = os.path.join(get_package_share_directory('proyecto_abp'), 'config', 'robot_0_amcl_config.yaml')
    nav2_yaml_1 = os.path.join(get_package_share_directory('proyecto_abp'), 'config', 'robot_1_amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('proyecto_abp'), 'config', 'mapa.yaml')

    return LaunchDescription([
        # Map Server - serves the static map to all robots
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file}]
        ),
        # AMCL node for robot_0
        Node(
            namespace="robot_0",
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml_0],
            remappings=[
                ('/scan', '/robot_0/scan'),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        ),
        # AMCL node for robot_1
        Node(
            namespace="robot_1",
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml_1],
            remappings=[
                ('/scan', '/robot_1/scan'),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ]
        ),
        # Lifecycle Manager to manage the lifecycle of all nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'bond_timeout': 0.0},
                {'node_names': ['map_server', 'robot_0/amcl', 'robot_1/amcl']}
            ]
        )
    ])