#!/usr/bin/env python3
import sys
import os
import time
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

NAVIGATION_ST = 'NAV2TARGET'

def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class NavCoordinator(Node):
    def __init__(self, num_robots):
        super().__init__('nav_coordinator', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.num_robots = num_robots
        self.current_poses = {}
        self.robot_states = {}  
        self.nav_triggered = False
        self.initial_pose_pubs = {}  
        
        for i in range(num_robots):
            robot_name = f'robot_{i}'
            self.current_poses[robot_name] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            self.robot_states[robot_name] = 'WANDER'  
            
            self.initial_pose_pubs[robot_name] = self.create_publisher(
                PoseWithCovarianceStamped, f'/{robot_name}/initialpose', 10)
                
            self.create_subscription(String, f'/{robot_name}/state', lambda msg, r=robot_name: self.state_callback(msg, r), 10)
            self.create_subscription(Odometry, f'/{robot_name}/odom', lambda msg, r=robot_name: self.odom_callback(msg, r), 10)
            
        self.get_logger().info(f"Coordinador Nav2 listo. Esperando transiciones...")

    def odom_callback(self, msg, robot_name):
        self.current_poses[robot_name]['x'] = msg.pose.pose.position.x
        self.current_poses[robot_name]['y'] = msg.pose.pose.position.y
        self.current_poses[robot_name]['yaw'] = get_yaw_from_quaternion(msg.pose.pose.orientation)

    def state_callback(self, msg, robot_name):
        self.robot_states[robot_name] = msg.data  
        if msg.data == NAVIGATION_ST and not self.nav_triggered:
            self.nav_triggered = True
            self.get_logger().info(f"¡[{robot_name}] ENCONTRÓ EL OBJETIVO!")
    
    def get_winner_robots(self):
        return [name for name, state in self.robot_states.items() if state == NAVIGATION_ST]

    def get_moving_robots(self):
        return [name for name, state in self.robot_states.items() if state != NAVIGATION_ST]

    def send_goal_to_target(self, robot_name, target_x, target_y):
        if not rclpy.ok(): return 
        try:
            nav_client = ActionClient(self, NavigateToPose, f'/{robot_name}/navigate_to_pose')
            nav_client.wait_for_server()
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = rclpy.time.Time().to_msg() 
            goal_msg.pose.pose.position.x = float(target_x)
            goal_msg.pose.pose.position.y = float(target_y)
            goal_msg.pose.pose.orientation.w = 1.0
            nav_client.send_goal_async(goal_msg)
            self.get_logger().info(f"🎯 Meta enviada a {robot_name}: ({target_x}, {target_y})")
        except Exception as e:
            self.get_logger().warning(f"Error al enviar meta: {e}")

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

    if not rclpy.ok(): return
    time.sleep(3.0)
    
    movers = coordinator.get_moving_robots()
    if not movers:
        return

    pkg_proyecto_abp = get_package_share_directory('proyecto_abp')
    map_path = os.path.join(pkg_proyecto_abp, 'config', 'mapa.yaml')

    pkg_nav2 = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')

    nodes_to_launch = []

    for robot_name in movers:
        # AQUÍ CARGAMOS TU ARCHIVO DIRECTAMENTE SIN PARSEAR NADA
        custom_params_file = os.path.join(pkg_proyecto_abp, 'config', f'nav2_{robot_name}.yaml')
        
        # Verificamos si existe antes de lanzar para evitar crashes de ROS2
        if not os.path.exists(custom_params_file):
            coordinator.get_logger().error(f"⚠️ NO SE ENCUENTRA EL ARCHIVO: {custom_params_file}")
            continue

        coordinator.get_logger().info(f"🚀 Lanzando Nav2 para {robot_name} usando archivo estático: {custom_params_file}")

        nav_group = GroupAction(actions=[
            SetRemap(src='tf', dst='/tf'),
            SetRemap(src='tf_static', dst='/tf_static'),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            SetRemap(src='scan', dst=f'/{robot_name}/scan'),
            SetRemap(src='/scan', dst=f'/{robot_name}/scan'),
            SetRemap(src='map', dst='/map'),
            SetRemap(src='/map', dst='/map'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    'map': map_path,
                    'use_sim_time': 'True',
                    'namespace': robot_name,
                    'use_namespace': 'True',
                    'autostart': 'True',
                    'params_file': custom_params_file,
                    'use_composition': 'False'
                }.items()
            )
        ])
        nodes_to_launch.append(nav_group)

    def delayed_goal_sender():
        for _ in range(150):
            if not rclpy.ok(): return
            time.sleep(0.1)
        if not rclpy.ok(): return

        for robot_name in movers:
            try:
                t = coordinator.tf_buffer.lookup_transform('map', f'{robot_name}/base_footprint', rclpy.time.Time())
                init_pose = PoseWithCovarianceStamped()
                init_pose.header.frame_id = 'map'
                init_pose.header.stamp = rclpy.time.Time().to_msg()
                init_pose.pose.pose.position.x = t.transform.translation.x
                init_pose.pose.pose.position.y = t.transform.translation.y
                init_pose.pose.pose.orientation = t.transform.rotation
                init_pose.pose.covariance[0] = 0.25
                init_pose.pose.covariance[7] = 0.25
                init_pose.pose.covariance[35] = 0.068
                
                coordinator.initial_pose_pubs[robot_name].publish(init_pose)
                coordinator.get_logger().info(f"✅ Pose inicial cargada mediante TF para {robot_name}")
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                coordinator.get_logger().error(f"Error de TF al inyectar pose: {e}")

        time.sleep(3.0)

        for idx, robot_name in enumerate(movers):
            coordinator.send_goal_to_target(robot_name, target_x=2.0, target_y=2.0 + float(idx * 1.0))

    threading.Thread(target=delayed_goal_sender, daemon=True).start()

    ls = LaunchService()
    ls.include_launch_description(LaunchDescription(nodes_to_launch))
    try:
        ls.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()