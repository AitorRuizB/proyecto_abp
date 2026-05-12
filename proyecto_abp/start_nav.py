#!/usr/bin/env python3
import sys
import os
import time
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

from ament_index_python.packages import get_package_share_directory
from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def generate_robot_nav_params(robot_name):
    """Lee la config default de Nav2 y le inyecta el namespace a los TFs (odom, base_link)"""
    nav2_pkg = get_package_share_directory('nav2_bringup')
    default_params_file = os.path.join(nav2_pkg, 'params', 'nav2_params.yaml')
    
    with open(default_params_file, 'r') as f:
        content = f.read()
        
    replacements = {
        ': odom': f': {robot_name}/odom',
        ': "odom"': f': "{robot_name}/odom"',
        ': base_link': f': {robot_name}/base_footprint',
        ': "base_link"': f': "{robot_name}/base_footprint"',
        ': base_footprint': f': {robot_name}/base_footprint',
        ': "base_footprint"': f': "{robot_name}/base_footprint"'
    }
    
    for old, new in replacements.items():
        content = content.replace(old, new)
        
    output_path = os.path.expanduser(f'~/nav2_params_{robot_name}.yaml')
    with open(output_path, 'w') as f:
        f.write(content)
        
    return output_path

class NavCoordinator(Node):
    def __init__(self, num_robots):
        super().__init__('nav_coordinator')
        self.num_robots = num_robots
        self.current_poses = {}
        self.nav_triggered = False
        
        for i in range(num_robots):
            robot_name = f'robot_{i}'
            self.current_poses[robot_name] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

            self.create_subscription(
                String, f'/{robot_name}/state', 
                lambda msg, r=robot_name: self.state_callback(msg, r), 10)
            
            self.create_subscription(
                Odometry, f'/{robot_name}/odom', 
                lambda msg, r=robot_name: self.odom_callback(msg, r), 10)

        self.get_logger().info(f"Coordinador Nav2 listo. Esperando transición a NAV2TARGET...")

    def odom_callback(self, msg, robot_name):
        if not self.nav_triggered:
            self.current_poses[robot_name]['x'] = msg.pose.pose.position.x
            self.current_poses[robot_name]['y'] = msg.pose.pose.position.y
            self.current_poses[robot_name]['yaw'] = get_yaw_from_quaternion(msg.pose.pose.orientation)

    def state_callback(self, msg, robot_name):
        if msg.data == 'NAV2TARGET' and not self.nav_triggered:
            self.nav_triggered = True
            self.get_logger().info(f"¡[{robot_name}] activó NAV2TARGET! Lanzando Nav2...")

    def send_goal_to_target(self, robot_name, target_x, target_y):
        """Cliente de acción para enviar la meta (goal) a Nav2"""
        nav_client = ActionClient(self, NavigateToPose, f'/{robot_name}/navigate_to_pose')
        
        self.get_logger().info(f"[{robot_name}] Esperando a que el servidor de acciones de Nav2 despierte...")
        nav_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Objetivo personalizado por robot
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"[{robot_name}] ¡Enviando robot a la base ({target_x}, {target_y})!")
        nav_client.send_goal_async(goal_msg)

def main():
    if len(sys.argv) < 2:
        print("Uso: ros2 run proyecto_abp start_nav <num_robots>")
        return

    num_robots = int(sys.argv[1])
    rclpy.init()
    coordinator = NavCoordinator(num_robots)
    
    spin_thread = threading.Thread(target=lambda: rclpy.spin(coordinator), daemon=True)
    spin_thread.start()

    while not coordinator.nav_triggered and rclpy.ok():
        time.sleep(0.1)

    if not rclpy.ok():
        return

    pkg_nav2 = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
    map_path = os.path.expanduser('~/mapa_global_unificado.yaml')

    nodes_to_launch = []

    for i in range(num_robots):
        robot_name = f'robot_{i}'
        pose = coordinator.current_poses[robot_name]
        
        custom_params_file = generate_robot_nav_params(robot_name)
        
        print(f"--- Configurando Nav2 para {robot_name} en X:{pose['x']:.2f}, Y:{pose['y']:.2f} ---")

        nav_group = GroupAction(actions=[
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    'map': map_path,
                    'use_sim_time': 'True',
                    'namespace': robot_name,
                    'use_namespace': 'True',
                    'autostart': 'True',
                    'params_file': custom_params_file,
                    'x': str(pose['x']),
                    'y': str(pose['y']),
                    'yaw': str(pose['yaw'])
                }.items()
            )
        ])
        nodes_to_launch.append(nav_group)

    def delayed_goal_sender():
        time.sleep(10.0) 
        
        # --- NUEVA LÓGICA: Destinos personalizados por robot ---
        for i in range(num_robots):
            robot_name = f'robot_{i}'
            # robot_0 va a (0, 0), robot_1 va a (1, 0), robot_2 va a (2, 0), etc.
            target_x = float(i) 
            target_y = 0.0
            coordinator.send_goal_to_target(robot_name, target_x=target_x, target_y=target_y)

    threading.Thread(target=delayed_goal_sender, daemon=True).start()

    ls = LaunchService()
    ls.include_launch_description(LaunchDescription(nodes_to_launch))
    
    try:
        ls.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()