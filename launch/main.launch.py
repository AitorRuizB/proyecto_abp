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

def generate_launch_description():
    return LaunchDescription([
        # Parámetro para elegir num de robots
        DeclareLaunchArgument('num_robots', default_value='2', description='Numero de robots a instanciar'),
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    # 1. Leer el número de robots
    num_robots_str = LaunchConfiguration('num_robots').perform(context)
    num_robots = int(num_robots_str)

    pkg_proyecto_abp = get_package_share_directory('proyecto_abp')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 2. Iniciar Gazebo
    world_file = os.path.join(pkg_proyecto_abp, 'world', 'laberinto_v2_world.sdf') 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    nodes = [gazebo]
    bridge_config = []

    # 3. Bucle para configurar cada robot (Visualización y Transformaciones)
    for i in range(num_robots):
        robot_name = f'robot_{i}'
        
        # Posición inicial (escalonada en Y para que no choquen al aparecer)
        y_pos = float(i) * -1.0 

        # Robot State Publisher (Carga el URDF con namespace)
        xacro_file = os.path.join(pkg_proyecto_abp, 'urdf', ROBOT_XACRO)
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file, ' prefix:=', robot_name, '/']),
                'use_sim_time': True,
                'frame_prefix': f'{robot_name}/'
            }]
        )

        # Aparecer el robot en Gazebo
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name,
                '-topic', f'/{robot_name}/robot_description',
                '-x', '0.0',
                '-y', str(y_pos),
                '-z', '0.1'
            ],
            output='screen'
        )

        # Transformación estática: map -> odom (Posicionamiento global)
        static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{robot_name}',
            arguments=['0', str(y_pos), '0', '0', '0', '0', 'map', f'{robot_name}/odom']
        )

        # Transformación estática: odom -> base_footprint (Para que RViz no de error de Status: Error)
        static_tf_odom_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_odom_{robot_name}',
            arguments=['0', '0', '0', '0', '0', '0', f'{robot_name}/odom', f'{robot_name}/base_footprint']
        )

        # Configuración del Bridge para este robot (Lidar, Cámara, Odometría, CMD_VEL)
        bridge_config.extend([
            {'ros_topic_name': f'/{robot_name}/scan', 'gz_topic_name': f'/world/default/model/{robot_name}/link/base_footprint/sensor/lidar_sensor/scan', 'ros_type': 'sensor_msgs/msg/LaserScan', 'gz_type': 'gz.msgs.LaserScan', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{robot_name}/cmd_vel', 'gz_topic_name': f'/model/{robot_name}/cmd_vel', 'ros_type': 'geometry_msgs/msg/Twist', 'gz_type': 'gz.msgs.Twist', 'direction': 'ROS_TO_GZ'},
            {'ros_topic_name': f'/{robot_name}/odom', 'gz_topic_name': f'/model/{robot_name}/odometry', 'ros_type': 'nav_msgs/msg/Odometry', 'gz_type': 'gz.msgs.Odometry', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{robot_name}/tf', 'gz_topic_name': f'/model/{robot_name}/tf', 'ros_type': 'tf2_msgs/msg/TFMessage', 'gz_type': 'gz.msgs.Pose_V', 'direction': 'GZ_TO_ROS'}
        ])

        nodes.extend([rsp_node, spawn_node, static_tf_node, static_tf_odom_node])

    # 4. Configurar el Bridge General
    bridge_yaml_path = os.path.join(tempfile.gettempdir(), 'multirobot_bridge.yaml')
    with open(bridge_yaml_path, 'w') as f:
        yaml.dump(bridge_config, f, default_flow_style=False)

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml_path}],
        output='screen'
    )

    # 5. Lanzar RViz
    rviz_config = os.path.join(pkg_proyecto_abp, 'rviz', 'config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    nodes.extend([bridge_node, rviz_node])
    return nodes