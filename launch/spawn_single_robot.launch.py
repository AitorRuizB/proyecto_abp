import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, LifecycleNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from nav2_common.launch import RewrittenYaml

def launch_setup(context, *args, **kwargs):
    # 1. Recibimos los argumentos dinámicos
    robot_name = LaunchConfiguration('robot_name').perform(context)
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)

    pkg_multirobot = get_package_share_directory('proyecto_abp')
    urdf_file = os.path.join(pkg_multirobot, 'urdf', 'robot.urdf.xacro')

    # 2. Robot State Publisher (Procesa el URDF con el prefijo)
    robot_desc_cmd = Command(['xacro ', urdf_file, ' prefix:=', robot_name, '/'])
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        parameters=[{'robot_description': robot_desc_cmd, 'use_sim_time': True}],
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
    )

    # 3. Spawner de Gazebo
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-string', robot_desc_cmd,
            '-x', x_pose, '-y', y_pose, '-z', '0.2'
        ],
        output='screen'
    )

    # 4. Bridge Local (Aislamos los topics de este robot en su propio YAML temporal)
    bridge_config = [
        {'ros_topic_name': f'/{robot_name}/cmd_vel', 'gz_topic_name': f'/{robot_name}/cmd_vel', 'ros_type_name': 'geometry_msgs/msg/Twist', 'gz_type_name': 'gz.msgs.Twist', 'direction': 'ROS_TO_GZ'},
        {'ros_topic_name': f'/{robot_name}/odom', 'gz_topic_name': f'/{robot_name}/odom', 'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'gz.msgs.Odometry', 'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'/{robot_name}/scan', 'gz_topic_name': f'/{robot_name}/scan', 'ros_type_name': 'sensor_msgs/msg/LaserScan', 'gz_type_name': 'gz.msgs.LaserScan', 'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'/{robot_name}/camera/image_raw', 'gz_topic_name': f'/{robot_name}/camera/image_raw', 'ros_type_name': 'sensor_msgs/msg/Image', 'gz_type_name': 'gz.msgs.Image', 'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'/{robot_name}/camera/camera_info', 'gz_topic_name': f'/{robot_name}/camera/camera_info', 'ros_type_name': 'sensor_msgs/msg/CameraInfo', 'gz_type_name': 'gz.msgs.CameraInfo', 'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'/{robot_name}/joint_states', 'gz_topic_name': f'/{robot_name}/joint_states', 'ros_type_name': 'sensor_msgs/msg/JointState', 'gz_type_name': 'gz.msgs.Model', 'direction': 'GZ_TO_ROS'}
    ]
    bridge_yaml_path = os.path.join(tempfile.gettempdir(), f'bridge_{robot_name}.yaml')
    with open(bridge_yaml_path, 'w') as f:
        yaml.dump(bridge_config, f, default_flow_style=False)

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{robot_name}',
        namespace=robot_name,
        parameters=[{'config_file': bridge_yaml_path}],
        output='screen'
    )

    # 5. SLAM Toolbox
    slam_yaml_path = os.path.join(pkg_multirobot, 'config', 'slam.yaml')
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot_name,
        # LA MAGIA ESTÁ AQUÍ: Mezclamos el YAML fijo con las variables dinámicas de este robot
        parameters=[
            slam_yaml_path,
            {
                'odom_frame': f'{robot_name}/odom',
                'base_frame': f'{robot_name}/base_footprint',
                'map_frame': f'{robot_name}/map',
                'scan_topic': f'/{robot_name}/scan'
            }
        ],
        remappings=[
            ('/map', f'/{robot_name}/map'),
            ('/map_metadata', f'/{robot_name}/map_metadata'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ],
        output='screen'
    )

    # 6. El Despertador del SLAM
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        namespace=robot_name,
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'], # Debe llamarse igual que el nombre del LifecycleNode
            'bond_timeout': 0.0
        }],
        output='screen'
    )

    # 7. Unión del mapa global con el local de este robot
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map',
        namespace=robot_name,
        arguments=[
            '--x', x_pose, '--y', y_pose, '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', f'{robot_name}/map'
        ]
    )
    # =================================================================
    # NAV2: El Cerebro de Navegación Autónomo
    # =================================================================
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2_yaml_path = os.path.join(pkg_multirobot, 'config', 'nav2.yaml')

    # La Magia de Nav2: Esta herramienta envuelve automáticamente 
    # el nav2.yaml con el namespace del robot para que no choquen.
    param_substitutions = {
        'use_sim_time': 'True',
        'autostart': 'True'
    }

    configured_params = RewrittenYaml(
        source_file=nav2_yaml_path,
        root_key=robot_name,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Invocamos el launch oficial de Nav2 (solo la navegación, sin su SLAM)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': robot_name,
            'use_namespace': 'True',
            'use_sim_time': 'True',
            'autostart': 'True',
            'params_file': configured_params,
            # Le decimos que se suscriba al topic /map de su propio namespace
            'map_subscribe_transient_local': 'true' 
        }.items()
    )

    return [rsp_node, spawn_node, bridge_node, slam_node, lifecycle_manager, static_tf_node,  nav2_launch]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', description='Nombre del namespace del robot'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='Posicion X en el mapa'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Posicion Y en el mapa'),
        OpaqueFunction(function=launch_setup)
    ])