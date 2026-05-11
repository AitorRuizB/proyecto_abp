#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class NavToPoseClient(Node):
    def __init__(self):
        super().__init__('nav_to_pose_client')

    def send_goal(self, robot_name, x, y, theta=0.0):
        # El action server de Nav2 bajo namespace es: /robot_name/navigate_to_pose
        action_server_name = f'/{robot_name}/navigate_to_pose'
        self._action_client = ActionClient(self, NavigateToPose, action_server_name)

        self.get_logger().info(f'Esperando al servidor de Nav2 para {robot_name}...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'Servidor {action_server_name} no disponible.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map" # Importante: usamos el mapa global
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Coordenadas
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        
        # Orientación (Simplificada para enviar solo el ángulo en Z)
        goal_msg.pose.pose.orientation.z = 0.0 
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Enviando objetivo a {robot_name}: x={x}, y={y}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Objetivo rechazado por Nav2')
            return

        self.get_logger().info('Objetivo aceptado, yendo hacia allá...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # Opcional: imprimir distancia restante
        # distance = feedback_msg.feedback.distance_remaining
        pass

    def get_result_callback(self, future):
        result = future.result().status
        if result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('¡Objetivo alcanzado con éxito!')
        else:
            self.get_logger().info(f'El objetivo falló con estado: {result}')
        
        # Cerramos el nodo al terminar
        rclpy.shutdown()

def main():
    if len(sys.argv) < 4:
        print("Uso: ros2 run proyecto_abp send_goal <nombre_robot> <x> <y>")
        return

    rclpy.init()
    
    robot_name = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]

    client = NavToPoseClient()
    client.send_goal(robot_name, x, y)
    rclpy.spin(client)

if __name__ == '__main__':
    main()