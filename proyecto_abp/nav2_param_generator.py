#!/usr/bin/env python3
import yaml
import os
import sys

def generate_nav2_params(num_robots, output_path):
    """
    Genera un archivo YAML de configuración de Nav2 para N robots,
    inyectando los prefijos de namespace y los frames correctos.
    """
    
    # El diccionario raíz que contendrá la configuración de todos los robots
    multi_robot_config = {}

    for i in range(num_robots):
        robot_id = f"robot_{i}"
        
        # ---------------------------------------------------------
        # DEFINICIÓN DE FRAMES Y TOPICS DINÁMICOS
        # ---------------------------------------------------------
        base_frame = f"{robot_id}/base_footprint"
        odom_frame = f"{robot_id}/odom"
        global_frame = "map"  # El mapa es compartido por todos
        scan_topic = f"/{robot_id}/scan"
        map_topic = "/map"    # El topic del mapa estático es global
        
        # Radio del robot (Requisito: 0.20)
        robot_radius = 0.20

        # ---------------------------------------------------------
        # AMCL (Localización)
        # ---------------------------------------------------------
        multi_robot_config[f"{robot_id}/amcl"] = {
            "ros__parameters": {
                "use_sim_time": True, # En Gazebo debe ser True
                "base_frame_id": base_frame,
                "odom_frame_id": odom_frame,
                "global_frame_id": global_frame,
                "scan_topic": scan_topic,
                "laser_model_type": "likelihood_field",
                "min_particles": 500,
                "max_particles": 2000,
                "update_min_d": 0.1,
                "update_min_a": 0.2
            }
        }

        # ---------------------------------------------------------
        # LOCAL COSTMAP
        # ---------------------------------------------------------
        multi_robot_config[f"{robot_id}/local_costmap"] = {
            "local_costmap": {
                "ros__parameters": {
                    "use_sim_time": True,
                    "global_frame": odom_frame, # El global del local costmap suele ser Odom
                    "robot_base_frame": base_frame,
                    "rolling_window": True,
                    "width": 5,
                    "height": 5,
                    "resolution": 0.05,
                    "robot_radius": robot_radius,
                    "plugins": ["obstacle_layer", "inflation_layer"],
                    "obstacle_layer": {
                        "plugin": "nav2_costmap_2d::ObstacleLayer",
                        "observation_sources": "lidar",
                        "lidar": {
                            "topic": scan_topic,
                            "data_type": "LaserScan",
                            "clearing": True,
                            "marking": True
                        }
                    },
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "inflation_radius": 0.6
                    }
                }
            }
        }

        # ---------------------------------------------------------
        # GLOBAL COSTMAP
        # ---------------------------------------------------------
        multi_robot_config[f"{robot_id}/global_costmap"] = {
            "global_costmap": {
                "ros__parameters": {
                    "use_sim_time": True,
                    "global_frame": global_frame,
                    "robot_base_frame": base_frame,
                    "resolution": 0.05,
                    "robot_radius": robot_radius,
                    "plugins": ["static_layer", "obstacle_layer", "inflation_layer"],
                    "static_layer": {
                        "plugin": "nav2_costmap_2d::StaticLayer",
                        "map_topic": map_topic,
                        "subscribe_to_updates": True
                    },
                    "obstacle_layer": {
                        "plugin": "nav2_costmap_2d::ObstacleLayer",
                        "observation_sources": "lidar",
                        "lidar": {
                            "topic": scan_topic,
                            "data_type": "LaserScan",
                            "clearing": True,
                            "marking": True
                        }
                    },
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "inflation_radius": 0.6
                    }
                }
            }
        }

        # ---------------------------------------------------------
        # PLANNER, CONTROLLER & SMOOTHER
        # ---------------------------------------------------------
        multi_robot_config[f"{robot_id}/planner_server"] = {
            "ros__parameters": {
                "use_sim_time": True,
                "planner_plugins": ["GridBased"],
                "GridBased": {
                    "plugin": "nav2_navfn_planner::NavfnPlanner"
                }
            }
        }

        multi_robot_config[f"{robot_id}/controller_server"] = {
            "ros__parameters": {
                "use_sim_time": True,
                "controller_plugins": ["FollowPath"],
                "FollowPath": {
                    "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                    "desired_linear_vel": 0.4,
                    "lookahead_dist": 0.6
                }
            }
        }

        multi_robot_config[f"{robot_id}/smoother_server"] = {
            "ros__parameters": {
                "use_sim_time": True,
                "smoother_plugins": ["simple_smoother"],
                "simple_smoother": {
                    "plugin": "nav2_smoother::SimpleSmoother"
                }
            }
        }

        # ---------------------------------------------------------
        # BEHAVIOR TREE NAVIGATOR
        # ---------------------------------------------------------
        multi_robot_config[f"{robot_id}/bt_navigator"] = {
            "ros__parameters": {
                "use_sim_time": True,
                # Usa el default del sistema para evitar errores de ruta
                "default_bt_xml_filename": "/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
            }
        }

        # ---------------------------------------------------------
        # LIFECYCLE MANAGER
        # ---------------------------------------------------------
        multi_robot_config[f"{robot_id}/lifecycle_manager_navigation"] = {
            "ros__parameters": {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "amcl", 
                    "planner_server", 
                    "controller_server", 
                    "smoother_server", 
                    "bt_navigator"
                ]
            }
        }

    # Crear directorios si no existen y guardar YAML
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        yaml.dump(multi_robot_config, f, default_flow_style=False, sort_keys=False)
    
    print(f"✅ Archivo Nav2 multi-robot generado exitosamente en: {output_path}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Uso: python3 nav2_param_generator.py <num_robots> <ruta_salida.yaml>")
        sys.exit(1)
        
    n_robots = int(sys.argv[1])
    out_path = sys.argv[2]
    generate_nav2_params(n_robots, out_path)