import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2', description='Numero de robots a instanciar'),
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    num_robots_str = LaunchConfiguration('num_robots').perform(context)
    num_robots = int(num_robots_str)

    pkg_multirobot = get_package_share_directory('proyecto_abp')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_multirobot, 'world', 'world_new.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    nodes = [gazebo]

    bridge_config = []
    bridge_config.extend([
        {'ros_topic_name': '/clock', 'gz_topic_name': '/clock', 'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock', 'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': '/tf', 'gz_topic_name': '/tf', 'ros_type_name': 'tf2_msgs/msg/TFMessage', 'gz_type_name': 'gz.msgs.Pose_V', 'direction': 'GZ_TO_ROS'}
    ])

    urdf_file = os.path.join(pkg_multirobot, 'urdf', 'robot.urdf.xacro')

    # Poses iniciales para map_merge [x0,y0, x1,y1, ...]
    init_poses_flat = []

    for i in range(1, num_robots + 1):
        robot_name = f'robot{i}'
        prefix = f'{robot_name}/'
        y_pose = (i - 1) * 1.0

        init_poses_flat += [0.0, y_pose]

        bridge_config.extend([
            {'ros_topic_name': f'/{robot_name}/cmd_vel', 'gz_topic_name': f'/{robot_name}/cmd_vel', 'ros_type_name': 'geometry_msgs/msg/Twist', 'gz_type_name': 'gz.msgs.Twist', 'direction': 'ROS_TO_GZ'},
            {'ros_topic_name': f'/{robot_name}/odom', 'gz_topic_name': f'/{robot_name}/odom', 'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'gz.msgs.Odometry', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{robot_name}/scan', 'gz_topic_name': f'/{robot_name}/scan', 'ros_type_name': 'sensor_msgs/msg/LaserScan', 'gz_type_name': 'gz.msgs.LaserScan', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{robot_name}/camera/image_raw', 'gz_topic_name': f'/{robot_name}/camera/image_raw', 'ros_type_name': 'sensor_msgs/msg/Image', 'gz_type_name': 'gz.msgs.Image', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{robot_name}/camera/camera_info', 'gz_topic_name': f'/{robot_name}/camera/camera_info', 'ros_type_name': 'sensor_msgs/msg/CameraInfo', 'gz_type_name': 'gz.msgs.CameraInfo', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{robot_name}/joint_states', 'gz_topic_name': f'/{robot_name}/joint_states', 'ros_type_name': 'sensor_msgs/msg/JointState', 'gz_type_name': 'gz.msgs.Model', 'direction': 'GZ_TO_ROS'}
        ])

        robot_desc_cmd = Command(['xacro ', urdf_file, ' prefix:=', prefix])

        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher_{robot_name}',
            namespace=robot_name,
            parameters=[{'robot_description': robot_desc_cmd, 'use_sim_time': True}],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        )

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name,
                '-string', robot_desc_cmd,
                '-x', '0.0',
                '-y', str(y_pose),
                '-z', '0.2'
            ],
            output='screen'
        )

        # SLAM via nav2_bringup slam_launch.py (igual que Gemini, funciona)
        slam_yaml_path = os.path.join(tempfile.gettempdir(), f'nav2_slam_{robot_name}.yaml')
        slam_config = {
            'slam_toolbox': {
                'ros__parameters': {
                    'odom_frame': f'{robot_name}/odom',
                    'base_frame': f'{robot_name}/base_footprint',
                    'map_frame':  f'{robot_name}/map',
                    'scan_topic': f'/{robot_name}/scan',
                    'use_sim_time': True,
                    'mode': 'mapping',
                    'resolution': 0.05,
                    'max_laser_range': 10.0,
                    'minimum_time_interval': 0.5,
                    'transform_publish_period': 0.02,
                    'map_update_interval': 1.0,
                }
            }
        }
        with open(slam_yaml_path, 'w') as f:
            yaml.dump(slam_config, f, default_flow_style=False)

        nav2_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'slam_launch.py')
            ),
            launch_arguments={
                'namespace':   robot_name,
                'use_sim_time': 'true',
                'autostart':   'true',
                'params_file':  slam_yaml_path
            }.items()
        )

        # Static TF: map → robot_i/map
        static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{robot_name}',
            arguments=[
                '--x', '0.0',
                '--y', str(y_pose),
                '--z', '0.0',
                '--yaw', '0.0',
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', f'{robot_name}/map'
            ]
        )

        nodes.extend([rsp_node, spawn_node, nav2_slam, static_tf_node])

    # Bridge
    bridge_yaml_path = os.path.join(tempfile.gettempdir(), 'multirobot_bridge.yaml')
    with open(bridge_yaml_path, 'w') as f:
        yaml.dump(bridge_config, f, default_flow_style=False)

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml_path}],
        output='screen'
    )

    # Map merge propio (sustituye a multirobot_map_merge que no existe en Jazzy)
    map_merge_node = Node(
        package='proyecto_abp',
        executable='map_merge',
        name='map_merge_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'num_robots':   num_robots,
            'init_poses':   init_poses_flat,
            'map_origin_x': -12.5,
            'map_origin_y': -12.5,
            'map_width':    500,
            'map_height':   500,
            'merging_rate': 1.0,
        }]
    )

    # RViz
    rviz_config = os.path.join(pkg_multirobot, 'rviz', 'config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    nodes.extend([
        bridge_node,
        TimerAction(period=20.0, actions=[map_merge_node]),  # espera a que los SLAM publiquen mapas
        rviz_node
    ])

    return nodes