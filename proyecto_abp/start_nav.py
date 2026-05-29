#!/usr/bin/env python3
import sys
import os
import time
import threading
import rclpy
import tempfile
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

from tf2_ros import Buffer, TransformListener # <-- IMPORTANTE
from proyecto_abp.finiteStateMachine import STATES

class NavCoordinator(Node):
    def __init__(self, num_robots):
        super().__init__('nav_coordinator', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.num_robots = num_robots
        self.robot_states = {}
        self.nav_triggered = False
        self.winner_robot = None
        self.initial_pose_pubs = {}
        self.robot_map_poses = {}
        
        # --- Listener para obtener la pose real y actual del ganador ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        for i in range(num_robots):
            robot_name = f'robot_{i}'
            self.robot_states[robot_name] = 'WANDER'
            self.initial_pose_pubs[robot_name] = self.create_publisher(PoseWithCovarianceStamped, f'/{robot_name}/initialpose', 10)
            self.create_subscription(String, f'/{robot_name}/state', lambda msg, r=robot_name: self.state_callback(msg, r), 10)
            
        self.get_logger().info("Coordinador Nav2 listo. Esperando transiciones...")

    def load_cached_poses(self):
        execution_path = os.getcwd()
        poses_file = os.path.join(execution_path, 'src', 'proyecto_abp', 'config', 'robot_poses.yaml')
        
        for _ in range(15): # Esperamos hasta 15s para que a SLAM le de tiempo a escribir
            if os.path.exists(poses_file):
                break
            time.sleep(1.0)
            
        if not os.path.exists(poses_file):
            self.get_logger().error(f"⚠️ Archivo de poses no encontrado en {poses_file}.")
            return False
            
        try:
            with open(poses_file, 'r') as f:
                self.robot_map_poses = yaml.safe_load(f)
            self.get_logger().info(f"✅ Poses iniciales cargadas correctamente desde el caché físico.")
            return True
        except Exception as e:
            self.get_logger().error(f"Error leyendo poses: {e}")
            return False

    def state_callback(self, msg, robot_name):
        self.robot_states[robot_name] = msg.data
        if msg.data == STATES[4]: 
            if self.winner_robot is None:
                self.get_logger().info(f"¡Robot {robot_name} es el ganador! Ha encontrado el objetivo.")
                self.winner_robot = robot_name

        if msg.data == STATES[5] and not self.nav_triggered: 
            self.nav_triggered = True
            self.get_logger().info(f"Transición a NAV2TARGET detectada por {robot_name}. Iniciando fase de navegación.")

    def send_goal_to_target(self, robot_name, target_x, target_y):
        if not rclpy.ok(): return 
        try:
            nav_client = ActionClient(self, NavigateToPose, f'/{robot_name}/navigate_to_pose')
            self.get_logger().info(f"⏳ Esperando al servidor de acción Nav2 para {robot_name}...")
            if not nav_client.wait_for_server(timeout_sec=15.0):
                self.get_logger().error(f"❌ Nav2 Action Server no disponible para {robot_name}. Revisa los logs.")
                return
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'  # Destino respecto al mapa global
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg() 
            goal_msg.pose.pose.position.x = float(target_x)
            goal_msg.pose.pose.position.y = float(target_y)
            goal_msg.pose.pose.orientation.w = 1.0
            nav_client.send_goal_async(goal_msg)
            self.get_logger().info(f"🎯 Meta enviada a {robot_name}: ({target_x:.2f}, {target_y:.2f}) respecto a 'map'")
        except Exception as e:
            self.get_logger().warning(f"Error al enviar meta: {e}")

def delayed_initial_pose_publisher(coordinator: NavCoordinator):
    coordinator.get_logger().info("Esperando a que los nodos de AMCL se inicien...")
    time.sleep(15.0) 

    if not coordinator.load_cached_poses():
        return

    for robot_name in coordinator.robot_states.keys():
        p = coordinator.robot_map_poses.get(robot_name)
        if p:
            init_pose = PoseWithCovarianceStamped()
            init_pose.header.frame_id = 'map'
            init_pose.header.stamp = coordinator.get_clock().now().to_msg()
            init_pose.pose.pose.position.x = float(p['x'])
            init_pose.pose.pose.position.y = float(p['y'])
            init_pose.pose.pose.position.z = float(p['z'])
            init_pose.pose.pose.orientation.x = float(p['qx'])
            init_pose.pose.pose.orientation.y = float(p['qy'])
            init_pose.pose.pose.orientation.z = float(p['qz'])
            init_pose.pose.pose.orientation.w = float(p['qw'])
            init_pose.pose.covariance[0] = 0.25
            init_pose.pose.covariance[7] = 0.25
            init_pose.pose.covariance[35] = 0.068
            
            coordinator.initial_pose_pubs[robot_name].publish(init_pose)
            coordinator.get_logger().info(f"✅ Pose inicial inyectada a AMCL para {robot_name}.")
        else:
            coordinator.get_logger().error(f"⚠️ No hay pose para {robot_name}.")
    
    coordinator.get_logger().info("--- Localización iniciada. Nodos desbloqueados. ---")

def delayed_goal_sender(coordinator: NavCoordinator):
    coordinator.get_logger().info("Esperando a que los servidores de acción de Nav2 se inicien...")
    time.sleep(30.0) 
    if not rclpy.ok(): return

    winner = coordinator.winner_robot
    if not winner: return

    # --- NUEVO: OBTENCIÓN DINÁMICA DE LA POSE DEL GANADOR RESPECTO A 'map' ---
    target_x, target_y = 0.0, 0.0
    try:
        # Escuchamos directamente el TF actual del ganador
        t = coordinator.tf_buffer.lookup_transform('map', f'{winner}/base_footprint', rclpy.time.Time())
        target_x = t.transform.translation.x
        target_y = t.transform.translation.y
        coordinator.get_logger().info(f"✅ Pose real capturada: {winner} está en X={target_x:.2f}, Y={target_y:.2f}")
    except Exception as e:
        coordinator.get_logger().error(f"⚠️ No se pudo obtener TF vivo de {winner}. Fallback a YAML. Error: {e}")
        # Solo en caso de fallo crítico recurrimos al YAML
        target_pose = coordinator.robot_map_poses.get(winner)
        if target_pose:
            target_x, target_y = target_pose['x'], target_pose['y']
        else:
            return

    followers = [r for r in coordinator.robot_states.keys() if r != winner]
    for robot_name in followers:
        coordinator.send_goal_to_target(robot_name, target_x, target_y)

def main():
    if len(sys.argv) < 2: return
    num_robots = int(sys.argv[1])
    rclpy.init()
    coordinator = NavCoordinator(num_robots)
    
    spin_thread = threading.Thread(target=lambda: rclpy.spin(coordinator), daemon=True)
    spin_thread.start()

    while not coordinator.nav_triggered and rclpy.ok(): time.sleep(0.1)
    if not rclpy.ok(): return
    time.sleep(3.0)
    
    pkg_proyecto_abp = get_package_share_directory('proyecto_abp')
    localization_launch_file = os.path.join(pkg_proyecto_abp, 'launch', 'multirobot_localization.launch.py')
    nodes_to_launch = [IncludeLaunchDescription(PythonLaunchDescriptionSource(localization_launch_file))]

    try:
        pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
        nav_launch_file = os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        
        template_params_path = os.path.join(pkg_proyecto_abp, 'config', 'nav2_params.yaml')
        with open(template_params_path, 'r') as f:
            template_content = f.read()
        
        for i in range(num_robots):
            robot_name = f'robot_{i}'
            robot_specific_content = template_content.replace('ROBOT_ID', robot_name)
            
            yaml_lines = [f"{robot_name}:"]
            for line in robot_specific_content.splitlines():
                yaml_lines.append("  " + line)
            final_yaml = "\n".join(yaml_lines)
            
            tmp_yaml_file = os.path.join(tempfile.gettempdir(), f'{robot_name}_nav2_params.yaml')
            with open(tmp_yaml_file, 'w') as f:
                f.write(final_yaml)
            
            nav_args = {'use_sim_time': 'True', 'autostart': 'True', 'params_file': tmp_yaml_file}
            nodes_to_launch.append(GroupAction(actions=[PushRosNamespace(robot_name), IncludeLaunchDescription(PythonLaunchDescriptionSource(nav_launch_file), launch_arguments=nav_args.items())]))
    except Exception as e:
         coordinator.get_logger().error(f"Error plantillas de Nav2: {e}")

    threading.Thread(target=delayed_initial_pose_publisher, args=(coordinator,), daemon=True).start()
    threading.Thread(target=delayed_goal_sender, args=(coordinator,), daemon=True).start()

    ls = LaunchService()
    ls.include_launch_description(LaunchDescription(nodes_to_launch))
    try: ls.run()
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            coordinator.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__': main()