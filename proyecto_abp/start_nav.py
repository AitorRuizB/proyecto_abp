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

from proyecto_abp.finiteStateMachine import STATES
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

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
        self.robot_states = {}
        self.robot_map_poses = {} # Cache para la última pose global conocida
        self.nav_triggered = False
        self.winner_robot = None
        self.initial_pose_pubs = {}
        
        for i in range(num_robots):
            robot_name = f'robot_{i}'
            self.robot_states[robot_name] = 'WANDER'
            self.robot_map_poses[robot_name] = None
            
            self.initial_pose_pubs[robot_name] = self.create_publisher(
                PoseWithCovarianceStamped, f'/{robot_name}/initialpose', 10)
                
            self.create_subscription(String, f'/{robot_name}/state', lambda msg, r=robot_name: self.state_callback(msg, r), 10)
        
        self.pose_update_timer = self.create_timer(0.5, self.update_robot_poses)
            
        self.get_logger().info(f"Coordinador Nav2 listo. Esperando transiciones...")

    def update_robot_poses(self):
        """
        Guarda periódicamente la última pose conocida de cada robot en el frame /map.
        Esto es crucial porque el arbol de TF del SLAM se apagará, y necesitamos
        esta información para inicializar AMCL en la fase de navegación.
        """
        if self.nav_triggered: # Dejar de actualizar cuando la navegación ha comenzado
            if self.pose_update_timer:
                self.pose_update_timer.cancel()
                self.pose_update_timer = None
            return

        for robot_name in self.robot_states.keys():
            try:
                # Buscamos la transformación desde el mapa global a la base del robot
                t = self.tf_buffer.lookup_transform('map', f'{robot_name}/base_footprint', rclpy.time.Time())
                self.robot_map_poses[robot_name] = t # Guardar la última transformación conocida
            except (LookupException, ConnectivityException, ExtrapolationException):
                # Es normal que al principio el TF no esté completo. No hacemos nada.
                pass

    def state_callback(self, msg, robot_name):
        self.robot_states[robot_name] = msg.data
        
        # Identifica al robot que encontró el objetivo y lo marca como "ganador"
        if msg.data == STATES[4]: # FINISH_SLAM
            if self.winner_robot is None:
                self.get_logger().info(f"¡Robot {robot_name} es el ganador! Ha encontrado el objetivo.")
                self.winner_robot = robot_name

        # Cuando el mapa está listo y las FSM transicionan, se activa el lanzamiento de Nav2
        if msg.data == STATES[5] and not self.nav_triggered: # NAV2TARGET
            self.nav_triggered = True
            self.get_logger().info(f"Transición a NAV2TARGET detectada por {robot_name}. Iniciando fase de navegación.")

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

def delayed_initial_pose_publisher(coordinator: NavCoordinator):
    """
    Espera a que los nodos de AMCL se inicien y luego publica la última pose conocida
    de cada robot para inicializar la localización.
    """
    coordinator.get_logger().info("Esperando a que los nodos de AMCL se inicien...")
    time.sleep(10.0)
    if not rclpy.ok(): return

    all_robots = coordinator.robot_states.keys()
    for robot_name in all_robots:
        t = coordinator.robot_map_poses.get(robot_name)
        if t:
            init_pose = PoseWithCovarianceStamped()
            init_pose.header.frame_id = 'map'
            init_pose.header.stamp = coordinator.get_clock().now().to_msg()
            init_pose.pose.pose.position.x = t.transform.translation.x
            init_pose.pose.pose.position.y = t.transform.translation.y
            init_pose.pose.pose.orientation = t.transform.rotation
            init_pose.pose.covariance[0] = 0.25
            init_pose.pose.covariance[7] = 0.25
            init_pose.pose.covariance[35] = 0.068
            
            coordinator.initial_pose_pubs[robot_name].publish(init_pose)
            coordinator.get_logger().info(f"✅ Pose inicial para {robot_name} publicada desde la última posición conocida.")
        else:
            coordinator.get_logger().error(f"⚠️ No se encontró pose en cache para {robot_name}. No se pudo inicializar AMCL.")
    
    coordinator.get_logger().info("--- Localización iniciada. Los robots están ahora localizados en el mapa. ---")

def delayed_goal_sender(coordinator: NavCoordinator):
    """
    Espera a que la pila de navegación esté activa y luego envía a los robots "seguidores"
    hacia la posición del robot "ganador".
    """
    coordinator.get_logger().info("Esperando a que los servidores de acción de Nav2 se inicien...")
    time.sleep(20.0) # Espera larga para asegurar que los servidores de acción estén listos
    if not rclpy.ok(): return

    winner = coordinator.winner_robot
    if not winner:
        coordinator.get_logger().error("No se pudo determinar el robot ganador. No se puede enviar la meta.")
        return

    target_pose_transform = coordinator.robot_map_poses.get(winner)
    if not target_pose_transform:
        coordinator.get_logger().error(f"No se encontró la pose guardada para el robot ganador '{winner}'. No se puede enviar la meta.")
        return

    target_x = target_pose_transform.transform.translation.x
    target_y = target_pose_transform.transform.translation.y
    followers = [r for r in coordinator.robot_states.keys() if r != winner]
    
    coordinator.get_logger().info(f"El robot ganador es {winner} en la pose ({target_x:.2f}, {target_y:.2f}).")
    coordinator.get_logger().info(f"Enviando a los robots seguidores {followers} hacia el objetivo.")

    for robot_name in followers:
        coordinator.send_goal_to_target(robot_name, target_x, target_y)

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
    
    # Espera para asegurar que los nodos de SLAM se han apagado correctamente
    time.sleep(3.0)
    
    # --- LANZAMIENTO DE LA LOCALIZACIÓN ---
    # Una vez que SLAM ha terminado y el mapa está guardado, lanzamos el sistema de localización
    # que usa ese mapa estático y AMCL para cada robot.
    pkg_proyecto_abp = get_package_share_directory('proyecto_abp')
    localization_launch_file = os.path.join(pkg_proyecto_abp, 'launch', 'multirobot_localization.launch.py')

    if not os.path.exists(localization_launch_file):
        coordinator.get_logger().error(f"No se encuentra el archivo de lanzamiento de localización: {localization_launch_file}")
        rclpy.shutdown()
        return

    nodes_to_launch = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch_file)
        )
    ]
    coordinator.get_logger().info(f"🚀 Lanzando el sistema de localización desde: {localization_launch_file}")

    # Inicia un hilo para publicar la pose inicial de cada robot para AMCL
    threading.Thread(target=delayed_initial_pose_publisher, args=(coordinator,), daemon=True).start()

    # Inicia un hilo para enviar a los robots seguidores hacia el ganador
    threading.Thread(target=delayed_goal_sender, args=(coordinator,), daemon=True).start()

    ls = LaunchService()
    ls.include_launch_description(LaunchDescription(nodes_to_launch))
    try:
        ls.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            coordinator.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()