#!/usr/bin/env python3
import sys
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory
from launch import LaunchService, LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch_ros.actions import LifecycleNode

# Importación de estados (Asumiendo que STATES es una lista exportada)
from proyecto_abp.finiteStateMachine import STATES

class SlamCoordinator(Node):
    def __init__(self, num_robots, launch_service):
        super().__init__('slam_coordinator')
        self.num_robots = num_robots
        self.launch_service = launch_service
        self.global_save_done = False
        self.transition_pubs = {}
        
        for i in range(num_robots):
            robot_name = f'robot_{i}'
            
            self.create_subscription(
                String, 
                f'/{robot_name}/state', 
                lambda msg, r=robot_name: self.state_callback(msg, r), 
                10
            )
            
            self.transition_pubs[robot_name] = self.create_publisher(
                String, f'/{robot_name}/transition', 10)

        self.get_logger().info(f"Coordinador iniciado. Guardaré el mapa GLOBAL cuando detecte {STATES[4]}.")

    def state_callback(self, msg, robot_name):
        if msg.data.strip() == STATES[4] and not self.global_save_done:
            self.global_save_done = True
            self.get_logger().info(f"[{robot_name}] ha llegado a {STATES[4]}. Guardando MAPA GLOBAL...")
            threading.Thread(target=self.save_global_map_procedure).start()

    def save_global_map_procedure(self):
        try:
            pkg_share = get_package_share_directory('proyecto_abp')
            config_dir = os.path.join(pkg_share, 'config')
            os.makedirs(config_dir, exist_ok=True)
            map_path = os.path.join(config_dir, 'map')
        except Exception as e:
            self.get_logger().error(f"Error de paquete: {e}")
            map_path = os.path.expanduser('~/map')

        command = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', map_path,
            '--ros-args', 
            '-p', 'save_map_timeout:=10000.0',
            '-p', 'map_subscribe_transient_local:=True',
            '-r', '/map:=/map' 
        ]
        
        try:
            self.get_logger().info(f"Ejecutando map_saver_cli en: {map_path}...")
            result = subprocess.run(command, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info("✅ ¡MAPA GLOBAL guardado con éxito en config/map!")
                
                msg = String()
                msg.data = 'GLOBAL_MAP_READY'
                for pub in self.transition_pubs.values():
                    pub.publish(msg)
                
                time.sleep(2.0)
                
                if self.launch_service:
                    self.get_logger().info("Apagando LaunchService de SLAM...")
                    self.launch_service.shutdown()
                
            else:
                self.get_logger().error(f"Error map_saver: {result.stderr}")
                self.global_save_done = False 
                
        except Exception as e:
            self.get_logger().error(f"Excepción: {str(e)}")
            self.global_save_done = False

def main():
    if len(sys.argv) < 2:
        print("Uso: ros2 run proyecto_abp start_slam <num_robots>")
        return

    num_robots = int(sys.argv[1])
    pkg = get_package_share_directory('proyecto_abp')
    slam_yaml = os.path.join(pkg, 'config', 'slam.yaml')

    nodes_to_launch = []

    for i in range(num_robots):
        robot_name = f'robot_{i}'
        
        # Configuración de SLAM Toolbox con los marcos perfectamente anclados
        nodes_to_launch.append(LifecycleNode(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', namespace=robot_name,
            parameters=[slam_yaml, {
                'odom_frame': f'{robot_name}/odom',
                'base_frame': f'{robot_name}/base_footprint',
                'map_frame': f'{robot_name}/map', 
                'scan_topic': f'/{robot_name}/scan',
                'use_sim_time': True,
                'transform_publish_period': 0.05
            }],
            # SLAM Toolbox publica localmente en /map, lo remapeamos a su namespace
            remappings=[('/map', f'/{robot_name}/map'), 
                        ('/scan', f'/{robot_name}/scan')]
        ))

        nodes_to_launch.append(LaunchNode(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_slam', namespace=robot_name,
            parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['slam_toolbox'], 'bond_timeout': 0.0}]
        ))

    # Lanzamos tu nuevo merger sin dependencias de odometría
    nodes_to_launch.append(LaunchNode(
        package='proyecto_abp', executable='map_merge',
        parameters=[{'use_sim_time': True, 'num_robots': num_robots}]
    ))

    ls = LaunchService()
    ls.include_launch_description(LaunchDescription(nodes_to_launch))

    rclpy.init()
    
    coordinator = SlamCoordinator(num_robots, launch_service=ls)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(coordinator)

    spin_thread = threading.Thread(target=lambda: executor.spin(), daemon=True)
    spin_thread.start()

    try:
        ls.run()
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()