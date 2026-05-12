#!/usr/bin/env python3
"""
Script que monitorea el estado del FSM y lanza Nav2 cuando se alcance MERGE_SLAM.
Úsalo así: ros2 run proyecto_abp launch_nav2_after_slam <robot_id>
Ejemplo:  ros2 run proyecto_abp launch_nav2_after_slam 0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time
import sys

class Nav2Launcher(Node):
    def __init__(self, robot_id):
        super().__init__('nav2_launcher')
        self.robot_id = f'robot_{robot_id}'
        self.nav2_launched = False
        
        # Suscribirse al estado del FSM
        self.create_subscription(String, f'/{self.robot_id}/state', self.state_callback, 10)
        
        self.get_logger().info(f"Esperando estado MERGE_SLAM para lanzar Nav2 de {self.robot_id}...")
    
    def state_callback(self, msg):
        """Cuando el estado sea MERGE_SLAM, lanzar Nav2."""
        if msg.data == 'MERGE_SLAM' and not self.nav2_launched:
            self.get_logger().info(f"🚀 ¡Estado MERGE_SLAM detectado! Lanzando Nav2 en 2 segundos...")
            time.sleep(2)
            self.launch_nav2()
            self.nav2_launched = True
    
    def launch_nav2(self):
        """Lanza Nav2 para el robot específico."""
        try:
            cmd = ['ros2', 'run', 'proyecto_abp', 'start_nav']
            self.get_logger().info(f"Ejecutando: {' '.join(cmd)}")
            
            # Lanzar en background para no bloquear
            subprocess.Popen(cmd)
            
            self.get_logger().info("✅ Nav2 lanzado correctamente")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error lanzando Nav2: {e}")

def main():
    if len(sys.argv) < 2:
        print("Uso: ros2 run proyecto_abp launch_nav2_after_slam <robot_id>")
        print("Ejemplo: ros2 run proyecto_abp launch_nav2_after_slam 0")
        return
    
    robot_id = sys.argv[1]
    rclpy.init(args=None)
    node = Nav2Launcher(robot_id)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
