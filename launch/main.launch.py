import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2', description='Numero de robots a instanciar'),
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))

    pkg         = get_package_share_directory('proyecto_abp')
    pkg_gz_sim  = get_package_share_directory('ros_gz_sim')
    urdf_file   = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    slam_yaml   = os.path.join(pkg, 'config', 'slam.yaml')

    nodes = []

    # 1. GAZEBO
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {os.path.join(pkg, "world", "laberinto_world.sdf")}'}.items(),
    ))

    # 2. BRIDGE GLOBAL
    global_bridge_path = os.path.join(tempfile.gettempdir(), 'global_bridge.yaml')
    with open(global_bridge_path, 'w') as f:
        yaml.dump([
            {'ros_topic_name': '/clock', 'gz_topic_name': '/clock', 'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': '/tf',    'gz_topic_name': '/tf',    'ros_type_name': 'tf2_msgs/msg/TFMessage',  'gz_type_name': 'gz.msgs.Pose_V', 'direction': 'GZ_TO_ROS'},
        ], f)

    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='global_bridge',
        parameters=[{'config_file': global_bridge_path}], output='screen'
    ))

    # 3. BUCLE POR ROBOT
    for i in range(1, num_robots + 1):
        robot_name = f'robot{i}'
        x_pose, y_pose = 0.0, float((i - 1) * 1.0)

        robot_desc = Command(['xacro ', urdf_file, ' prefix:=', robot_name, '/'])

        nodes.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=robot_name, parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
        ))

        nodes.append(Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', robot_name, '-string', robot_desc, '-x', str(x_pose), '-y', str(y_pose), '-z', '0.2']
        ))

        # Bridge Local
        local_bridge_path = os.path.join(tempfile.gettempdir(), f'bridge_{robot_name}.yaml')
        with open(local_bridge_path, 'w') as f:
            yaml.dump([
                {'ros_topic_name': f'/{robot_name}/cmd_vel', 'gz_topic_name': f'/{robot_name}/cmd_vel', 'ros_type_name': 'geometry_msgs/msg/Twist', 'gz_type_name': 'gz.msgs.Twist', 'direction': 'ROS_TO_GZ'},
                {'ros_topic_name': f'/{robot_name}/odom', 'gz_topic_name': f'/{robot_name}/odom', 'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'gz.msgs.Odometry', 'direction': 'GZ_TO_ROS'},
                {'ros_topic_name': f'/{robot_name}/scan', 'gz_topic_name': f'/{robot_name}/scan', 'ros_type_name': 'sensor_msgs/msg/LaserScan', 'gz_type_name': 'gz.msgs.LaserScan', 'direction': 'GZ_TO_ROS'},
                {'ros_topic_name': f'/{robot_name}/camera/image_raw', 'gz_topic_name': f'/{robot_name}/camera/image_raw', 'ros_type_name': 'sensor_msgs/msg/Image', 'gz_type_name': 'gz.msgs.Image', 'direction': 'GZ_TO_ROS'},
                {'ros_topic_name': f'/{robot_name}/camera/camera_info', 'gz_topic_name': f'/{robot_name}/camera/camera_info', 'ros_type_name': 'sensor_msgs/msg/CameraInfo', 'gz_type_name': 'gz.msgs.CameraInfo', 'direction': 'GZ_TO_ROS'},
                {'ros_topic_name': f'/{robot_name}/joint_states', 'gz_topic_name': f'/{robot_name}/joint_states', 'ros_type_name': 'sensor_msgs/msg/JointState', 'gz_type_name': 'gz.msgs.Model', 'direction': 'GZ_TO_ROS'},
            ], f)

        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'bridge_{robot_name}', namespace=robot_name, parameters=[{'config_file': local_bridge_path}]
        ))

        # SLAM Toolbox
        nodes.append(LifecycleNode(
            package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', namespace=robot_name,
            parameters=[slam_yaml, {'odom_frame': f'{robot_name}/odom', 'base_frame': f'{robot_name}/base_footprint', 'map_frame': f'{robot_name}/map', 'scan_topic': f'/{robot_name}/scan', 'use_sim_time': True}],
            remappings=[('/map', f'/{robot_name}/map'), ('/map_metadata', f'/{robot_name}/map_metadata'), ('/tf', '/tf'), ('/tf_static', '/tf_static')]
        ))

        nodes.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_slam', namespace=robot_name,
            parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['slam_toolbox'], 'bond_timeout': 0.0}]
        ))

        # Static TF: map → robot_i/map
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name=f'static_tf_{robot_name}',
            arguments=['--x', str(x_pose), '--y', str(y_pose), '--z', '0.0', '--frame-id', 'map', '--child-frame-id', f'{robot_name}/map']
        ))

    # 4. MAP MERGE
    nodes.append(TimerAction(
        period=20.0,
        actions=[Node(
            package='proyecto_abp', executable='map_merge', name='custom_map_merger',
            parameters=[{'use_sim_time': True, 'num_robots': num_robots}]
        )]
    ))

    # 5. RVIZ
    nodes.append(Node(
        package='rviz2', executable='rviz2', arguments=['-d', os.path.join(pkg, 'rviz', 'config.rviz')],
        parameters=[{'use_sim_time': True}]
    ))

    return nodes