import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_nav2_params(num_robots):
    """
    Genera un archivo de configuración de Nav2 independiente para cada robot
    SIN namespaces en las claves (navigation_launch.py se los pondrá automáticamente).
    """
    yaml_paths = []
    bt_path = os.path.join(
        get_package_share_directory('nav2_bt_navigator'), 
        'behavior_trees', 
        'navigate_to_pose_w_replanning_and_recovery.xml'
    )

    for i in range(num_robots):
        robot_id = f"robot_{i}"
        
        base_frame = f"{robot_id}/base_footprint"
        odom_frame = f"{robot_id}/odom"
        global_frame = "map"
        scan_topic = f"/{robot_id}/scan"
        map_topic = "/map"
        robot_radius = 0.20

        # Claves limpias (amcl, controller_server...)
        robot_config = {}

        robot_config["amcl"] = {
            "ros__parameters": {
                "use_sim_time": True,
                "base_frame_id": base_frame,
                "odom_frame_id": odom_frame,
                "global_frame_id": global_frame,
                "scan_topic": scan_topic,
                "laser_model_type": "likelihood_field"
            }
        }

        robot_config["local_costmap"] = {
            "local_costmap": {
                "ros__parameters": {
                    "use_sim_time": True,
                    "global_frame": odom_frame,
                    "robot_base_frame": base_frame,
                    "rolling_window": True,
                    "width": 4, "height": 4, "resolution": 0.05,
                    "robot_radius": robot_radius,
                    "plugins": ["obstacle_layer", "inflation_layer"],
                    "obstacle_layer": {
                        "plugin": "nav2_costmap_2d::ObstacleLayer",
                        "observation_sources": "lidar",
                        "lidar": {"topic": scan_topic, "data_type": "LaserScan", "clearing": True, "marking": True}
                    },
                    "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer", "inflation_radius": 0.5}
                }
            }
        }

        robot_config["global_costmap"] = {
            "global_costmap": {
                "ros__parameters": {
                    "use_sim_time": True,
                    "global_frame": global_frame,
                    "robot_base_frame": base_frame,
                    "resolution": 0.05,
                    "robot_radius": robot_radius,
                    "plugins": ["static_layer", "obstacle_layer", "inflation_layer"],
                    "static_layer": {"plugin": "nav2_costmap_2d::StaticLayer", "map_topic": map_topic, "subscribe_to_updates": True},
                    "obstacle_layer": {
                        "plugin": "nav2_costmap_2d::ObstacleLayer",
                        "observation_sources": "lidar",
                        "lidar": {"topic": scan_topic, "data_type": "LaserScan", "clearing": True, "marking": True}
                    },
                    "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer", "inflation_radius": 0.5}
                }
            }
        }

        robot_config["planner_server"] = {
            "ros__parameters": {
                "use_sim_time": True, 
                "planner_plugins": ["GridBased"], 
                "GridBased": {"plugin": "nav2_navfn_planner::NavfnPlanner"}
            }
        }

        robot_config["controller_server"] = {
            "ros__parameters": {
                "use_sim_time": True, 
                "controller_frequency": 20.0,
                "progress_checker_plugin": "progress_checker",
                "goal_checker_plugins": ["goal_checker"],
                "controller_plugins": ["FollowPath"],
                "FollowPath": {"plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController", "desired_linear_vel": 0.4, "lookahead_dist": 0.6},
                "progress_checker": {"plugin": "nav2_controller::SimpleProgressChecker", "required_movement_radius": 0.5, "movement_time_allowance": 10.0},
                "goal_checker": {"plugin": "nav2_controller::SimpleGoalChecker", "xy_goal_tolerance": 0.25, "yaw_goal_tolerance": 0.25, "stateful": True}
            }
        }

        robot_config["bt_navigator"] = {
            "ros__parameters": {
                "use_sim_time": True, 
                "default_bt_xml_filename": bt_path,
                "global_frame": global_frame,
                "robot_base_frame": base_frame,
                "odom_topic": f"/{robot_id}/odom"
            }
        }
        
        robot_config["behavior_server"] = {
            "ros__parameters": {
                "use_sim_time": True,
                "local_frame": odom_frame,
                "global_frame": global_frame,
                "robot_base_frame": base_frame
            }
        }

        robot_config["velocity_smoother"] = {
            "ros__parameters": {
                "use_sim_time": True,
                "odom_topic": f"/{robot_id}/odom"
            }
        }

        robot_config["waypoint_follower"] = {"ros__parameters": {"use_sim_time": True}}
        robot_config["smoother_server"] = {"ros__parameters": {"use_sim_time": True}}

        # Guardamos un YAML distinto por cada robot
        out_path = os.path.join(tempfile.gettempdir(), f'nav2_{robot_id}.yaml')
        with open(out_path, 'w') as f:
            yaml.dump(robot_config, f, default_flow_style=False, sort_keys=False)
        
        yaml_paths.append(out_path)
        
    return yaml_paths

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2', description='Número de robots a navegar'),
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    pkg_proyecto_abp = get_package_share_directory('proyecto_abp')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # 1. Generamos los N archivos YAML
    yaml_paths = generate_nav2_params(num_robots)

    nodes_to_launch = []

    # 2. MAP SERVER GLOBAL (1 solo para todos)
    map_yaml_file = os.path.join(pkg_proyecto_abp, 'config', 'mapa.yaml')
    
    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml_file}]
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
        output='screen', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}]
    )
    
    nodes_to_launch.extend([map_server_node, lifecycle_manager_map])

    # 3. NAV2 BRINGUP POR ROBOT
    for i in range(num_robots):
        robot_name = f'robot_{i}'
        
        # Le pasamos a cada robot SU propio archivo YAML generado
        nav2_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'namespace': robot_name,
                'use_sim_time': 'true',
                'params_file': yaml_paths[i],  # <- Aquí la magia
                'use_lifecycle_mgr': 'true',
                'map_subscribe_transient_local': 'true'
            }.items()
        )
        nodes_to_launch.append(nav2_group)

    return nodes_to_launch