import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

ROBOT_XACRO = 'my_robot.xacro'
MAPA_WORLD_FILE = 'laberinto_v2_world.sdf'
RVIZ_FILE = 'robot_0.rviz'

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2', description='Numero de robots a instanciar'),
        DeclareLaunchArgument('goal', default_value='green', description='Objetivo a buscar (green, yellow, red, blue)'),
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    num_robots_str = LaunchConfiguration('num_robots').perform(context)
    num_robots = int(num_robots_str)
    goal = LaunchConfiguration('goal').perform(context)

    pkg_proyecto_abp = get_package_share_directory('proyecto_abp')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_proyecto_abp, 'world', MAPA_WORLD_FILE) 
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

    urdf_file = os.path.join(pkg_proyecto_abp, 'urdf', ROBOT_XACRO)

    for i in range(0, num_robots):
        robot_name = f'robot_{i}'
        prefix = f'{robot_name}/'
        y_pose = (i) * 2.5  

        bridge_config.extend([
            {'ros_topic_name': f'/{robot_name}/cmd_vel', 'gz_topic_name': f'/{robot_name}/cmd_vel', 'ros_type_name': 'geometry_msgs/msg/Twist', 'gz_type_name': 'gz.msgs.Twist', 'direction': 'ROS_TO_GZ'},
            {'ros_topic_name': f'/{robot_name}/odom', 'gz_topic_name': f'/{robot_name}/odom', 'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'gz.msgs.Odometry', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{robot_name}/scan', 'gz_topic_name': f'/{robot_name}/scan', 'ros_type_name': 'sensor_msgs/msg/LaserScan', 'gz_type_name': 'gz.msgs.LaserScan', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{robot_name}/camera/image_raw', 'gz_topic_name': f'/{robot_name}/camera/image_raw', 'ros_type_name': 'sensor_msgs/msg/Image', 'gz_type_name': 'gz.msgs.Image', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{robot_name}/camera/camera_info', 'gz_topic_name': f'/{robot_name}/camera/camera_info', 'ros_type_name': 'sensor_msgs/msg/CameraInfo', 'gz_type_name': 'gz.msgs.CameraInfo', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{robot_name}/joint_states', 'gz_topic_name': f'/{robot_name}/joint_states', 'ros_type_name': 'sensor_msgs/msg/JointState', 'gz_type_name': 'gz.msgs.Model', 'direction': 'GZ_TO_ROS'}
        ])

        robot_desc_cmd = Command(['xacro ', urdf_file, ' prefix:=', prefix])

        # CORRECCIÓN: Mapeos globales de TF agregados al robot_state_publisher
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
                '-name', robot_name, '-string', robot_desc_cmd,
                '-x', '0.0', '-y', str(y_pose), '-z', '1', '-Y', '2.7'
            ],
            output='screen'
        )

        # CORRECCIÓN: Mapeos globales de TF agregados
        static_tf_odom_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_odom_{robot_name}',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', 
                '--frame-id', f'{robot_name}/odom', '--child-frame-id', f'{robot_name}/base_footprint'
            ],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        )

        fsm_node = Node(
            package='proyecto_abp',
            executable='finite_state_machine',
            name=f'finite_state_machine_{robot_name}',
            namespace=robot_name,
            parameters=[{'goal': goal}],
            output='screen'
        )

        if i == 0:
            carpet_node = Node(
                package='proyecto_abp',
                executable='carpet_manager',
                name='carpet_color_manager',
                output='screen'
            )
            nodes.append(carpet_node)

        nodes.extend([rsp_node, spawn_node, fsm_node])

    bridge_yaml_path = os.path.join(tempfile.gettempdir(), 'multirobot_bridge.yaml')
    with open(bridge_yaml_path, 'w') as f:
        yaml.dump(bridge_config, f, default_flow_style=False)

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml_path}],
        output='screen'
    )

    rviz_config = os.path.join(pkg_proyecto_abp, 'rviz', RVIZ_FILE)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    nodes.extend([bridge_node, rviz_node])

    return nodes