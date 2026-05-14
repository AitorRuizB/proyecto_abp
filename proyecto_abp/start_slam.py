#!/usr/bin/env python3
import sys
import os
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService, LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch_ros.actions import LifecycleNode
import threading
import time

class SlamCoordinator(Node):
    def __init__(self, num_robots):
        super().__init__('slam_coordinator')
        self.num_robots = num_robots
        self.global_save_done = False
        self.transition_pubs = {}
        
        for i in range(num_robots):
            robot_name = f'robot_{i}'
            self.create_subscription(
                String, f'/{robot_name}/state', 
                lambda msg, r=robot_name: self.state_callback(msg, r), 10)
            
            self.transition_pubs[robot_name] = self.create_publisher(
                String, f'/{robot_name}/transition', 10)

        self.get_logger().info("Coordinador iniciado. Guardaré el mapa GLOBAL cuando detecte FINISH_SLAM.")

    def state_callback(self, msg, robot_name):
        if msg.data == 'FINISH_SLAM' and not self.global_save_done:
            self.global_save_done = True
            self.get_logger().info(f"[{robot_name}] ha llegado a FINISH_SLAM. Guardando MAPA GLOBAL...")
            threading.Thread(target=self.save_global_map_procedure).start()

    def save_global_map_procedure(self):
        map_path = os.path.expanduser('~/mapa_global_unificado')
        
        command = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', map_path,
            '--ros-args', '-p', 'save_map_timeout:=10000.0'
        ]
        
        try:
            self.get_logger().info("Ejecutando map_saver_cli para el tópico /map...")
            result = subprocess.run(command, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info("¡MAPA GLOBAL guardado con éxito en ~/mapa_global_unificado!")
                
                msg = String()
                msg.data = 'GLOBAL_MAP_READY'
                for pub in self.transition_pubs.values():
                    pub.publish(msg)
                
                time.sleep(2.0)
                # Apagado elegante
                os.kill(os.getpid(), signal.SIGINT)
            else:
                self.get_logger().error(f"Error al guardar mapa global: {result.stderr}")
                self.global_save_done = False 
                
        except Exception as e:
            self.get_logger().error(f"Excepción al ejecutar map_saver: {str(e)}")
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
        nodes_to_launch.append(LifecycleNode(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', namespace=robot_name,
            parameters=[slam_yaml, {
                'odom_frame': f'{robot_name}/odom',
                'base_frame': f'{robot_name}/base_footprint',
                'map_frame': 'map', 
                'scan_topic': f'/{robot_name}/scan',
                'use_sim_time': True,
                'transform_publish_period': 0.05
            }],
            remappings=[('/map', f'/{robot_name}/map'), 
                        ('/tf', '/tf'), 
                        ('/tf_static', '/tf_static'),
                        ('/scan', f'/{robot_name}/scan')
                        ]
        ))

        nodes_to_launch.append(LaunchNode(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_slam', namespace=robot_name,
            parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['slam_toolbox'], 'bond_timeout': 0.0}]
        ))

    nodes_to_launch.append(LaunchNode(
        package='proyecto_abp', executable='map_merge',
        parameters=[{'use_sim_time': True, 'num_robots': num_robots}]
    ))

    rclpy.init()
    coordinator = SlamCoordinator(num_robots)
    
    # CORRECCIÓN: Función auxiliar para capturar el error de apagado en el hilo
    def spin_coordinator():
        try:
            rclpy.spin(coordinator)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass

    spin_thread = threading.Thread(target=spin_coordinator, daemon=True)
    spin_thread.start()

    ls = LaunchService()
    ls.include_launch_description(LaunchDescription(nodes_to_launch))
    
    try:
        ls.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()