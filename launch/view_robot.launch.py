import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

RVIZ_FILE = 'robot_0.rviz'
GZ_WORLD_FILE = 'laberinto_v1_world.sdf'

def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    
    robot_bringup_package_dir = get_package_share_directory('proyecto_abp')
    urdf_file_path = os.path.join(robot_bringup_package_dir, 'urdf', 'my_robot.xacro')
    
    launch_nodes = []
    
    # --- GAZEBO BRIDGE ---
    # Define all topics to be bridged
    bridge_topics = [
        {'ros_topic': 'cmd_vel',         'gz_topic': '/model/{name}/cmd_vel',     'ros_type': 'geometry_msgs/msg/Twist',      'gz_type': 'gz.msgs.Twist',       'direction': 'ROS_TO_GZ'},
        {'ros_topic': 'odom',            'gz_topic': '/{name}/odom',              'ros_type': 'nav_msgs/msg/Odometry',      'gz_type': 'gz.msgs.Odometry',    'direction': 'GZ_TO_ROS'},
        {'ros_topic': 'tf',              'gz_topic': '/model/{name}/tf',        'ros_type': 'tf2_msgs/msg/TFMessage',       'gz_type': 'gz.msgs.Pose_V',      'direction': 'GZ_TO_ROS'},
        {'ros_topic': 'joint_states',    'gz_topic': '/model/{name}/joint_states','ros_type': 'sensor_msgs/msg/JointState',   'gz_type': 'gz.msgs.Model',       'direction': 'GZ_TO_ROS'},
        {'ros_topic': 'scan',            'gz_topic': '/{name}/scan',              'ros_type': 'sensor_msgs/msg/LaserScan',    'gz_type': 'gz.msgs.LaserScan',   'direction': 'GZ_TO_ROS'},
        {'ros_topic': 'camera/image',    'gz_topic': '/{name}/camera/image_raw',  'ros_type': 'sensor_msgs/msg/Image',        'gz_type': 'gz.msgs.Image',       'direction': 'GZ_TO_ROS'},
        {'ros_topic': 'camera/camera_info', 'gz_topic': '/{name}/camera/camera_info', 'ros_type': 'sensor_msgs/msg/CameraInfo', 'gz_type': 'gz.msgs.CameraInfo', 'direction': 'GZ_TO_ROS'},
    ]

    # Initialize YAML content with the clock bridge
    bridge_params_content = """
# ROS2 <-> Gazebo bridge parameters
-
  ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
"""

    # Generate bridge parameters for each robot
    for i in range(num_robots):
        robot_name = f'robot_{i}'
        for topic_config in bridge_topics:
            # All TF messages go to the same topic
            ros_topic_name = f"/{robot_name}/{topic_config['ros_topic']}"
            if topic_config['ros_topic'] == 'tf':
                ros_topic_name = '/tf'

            bridge_params_content += f"""
-
  ros_topic_name: {ros_topic_name}
  gz_topic_name: {topic_config['gz_topic'].format(name=robot_name)}
  ros_type_name: {topic_config['ros_type']}
  gz_type_name: {topic_config['gz_type']}
  direction: {topic_config['direction']}
"""

        
        # 1. Robot State Publisher
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file_path, ' robot_name:=', robot_name]),
                'publish_frequency': 50.0,
                'frame_prefix': f'{robot_name}/',
                'use_sim_time': True
            }],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        )

        # 2. Joint State Publisher
        jsp = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{'publish_frequency': 50.0, 'use_sim_time': True}]
        )

        # 3. Spawn entity (Gazebo)
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-string', Command(['xacro ', urdf_file_path, ' robot_name:=', robot_name]),
                       '-name', robot_name,
                       '-x', '0.0', '-y', str(0.0 + i * 2.0), '-z', '0.1'],
            output='screen'
        )

        # 4. Differential Drive Node (The class you provided)
        """
        drive_node = Node(
            package='proyecto_abp',       # Replace with your actual package name
            executable='differential_drive', # matches the entry_point in setup.py
            namespace=robot_name,          # Launches in group /robot_i
            output='screen'
        )
        """

        # 4.5 Camera Subscriber Node (Monitor camera data)
        camera_sub_node = Node(
            package='proyecto_abp',
            executable='camera_subscriber',
            namespace=robot_name,
            output='screen'
        )

        # 4.6 Finite State Machine Node
        fsm_node = Node(
            package='proyecto_abp',
            executable='finite_state_machine',
            namespace=robot_name,
            output='screen'
        )

        # 5. Static TF (Connect world to robot_name/odom)
        # Offset the TF tree by the same amount the robot is spawned in Gazebo
        world_to_odom_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '-5.0', '--y', str(-8.0 + i * 2.0), '--z', '0.0',
                       '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                       '--frame-id', 'world', '--child-frame-id', f'{robot_name}/odom'],
            output='screen'
        )

        # --- NUEVO NODO PARA RECONECTAR LA CÁMARA Y APLICAR ROTACIÓN ÓPTICA ---
        camera_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            # Rotamos Yaw y Roll -1.5708 rad (-90 grados) para alinear la óptica
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                       '--yaw', '-1.5708', '--pitch', '0.0', '--roll', '-1.5708',
                       '--frame-id', f'{robot_name}/camera_link',
                       '--child-frame-id', f'{robot_name}/base_link/camera'],
            output='screen'
        )

        # --- NUEVO NODO PARA RECONECTAR EL LIDAR ---
        lidar_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            # El LiDAR coincide exactamente con su Link, así que todo es 0
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                       '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                       '--frame-id', f'{robot_name}/lidar_link',
                       '--child-frame-id', f'{robot_name}/base_link/gpu_lidar'],
            output='screen'
        )
        
        launch_nodes.append(TimerAction(period=2.0, actions=[rsp]))  # Delay RSP start
        launch_nodes.append(TimerAction(period=2.0, actions=[jsp]))  # Delay JSP start
        launch_nodes.append(TimerAction(period=5.0 + i*2.0, actions=[spawn]))  # Increased spawn delay
        #launch_nodes.append(TimerAction(period=8.0 + i*2.0, actions=[drive_node]))  # Increased drive node delay
        launch_nodes.append(TimerAction(period=9.0 + i*2.0, actions=[camera_sub_node]))  # Increased camera delay
        launch_nodes.append(fsm_node)
        launch_nodes.append(world_to_odom_tf)
        launch_nodes.append(camera_tf)
        launch_nodes.append(lidar_tf)

    generated_bridge_params_path = os.path.join(robot_bringup_package_dir, 'config', 'generated_bridge_params.yaml')
    with open(generated_bridge_params_path, 'w') as f:
        f.write(bridge_params_content)

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={generated_bridge_params_path}'],
        output='screen'
    )
    launch_nodes.append(ros_gz_bridge)
        
    return launch_nodes

def generate_launch_description():
    # Sanitize LD_LIBRARY_PATH
    if 'LD_LIBRARY_PATH' in os.environ:
        os.environ['LD_LIBRARY_PATH'] = os.pathsep.join([
            p for p in os.environ['LD_LIBRARY_PATH'].split(os.pathsep)
            if '/snap/' not in p
        ])

    robot_bringup_package_dir = get_package_share_directory('proyecto_abp')
    rviz_config_path = os.path.join(robot_bringup_package_dir, 'rviz', RVIZ_FILE)
    
    # IMPORTANTE: Definimos la ruta a tu mundo personalizado
    world_file_path = os.path.join(robot_bringup_package_dir, 'world', GZ_WORLD_FILE)

    # Set Gazebo resource path
    gazebo_resource_path = os.path.dirname(robot_bringup_package_dir)
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + gazebo_resource_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gazebo_resource_path

    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='3', description='Number of robots to spawn'
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen', arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r -v 4 {world_file_path}'}.items(),
    )

    return LaunchDescription([
        num_robots_arg,
        gz_sim,
        TimerAction(period=5.0, actions=[rviz_node]),  # Delay RViz to ensure Gazebo clock is ready
        OpaqueFunction(function=launch_setup)
    ])