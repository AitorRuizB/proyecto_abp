#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
import math

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class DynamicMapMerger(Node):
    def __init__(self):
        super().__init__('custom_map_merger')
        
        self.declare_parameter('num_robots', 2)
        self.num_robots = self.get_parameter('num_robots').value

        self.tf_broadcaster = TransformBroadcaster(self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_msgs = {}   
        self.initial_poses = {} # Diccionario para guardar (x, y, yaw) dinámicos de Gazebo
        
        self.map_subs = []       
        self.odom_subs = []

        # Suscripciones dinámicas a Mapas y a Odometría (Ground Truth de Gazebo)
        for i in range(0, self.num_robots):
            robot_name = f'robot_{i}'
            self.map_msgs[robot_name] = None
            
            # Suscripción al mapa
            self.map_subs.append(
                self.create_subscription(
                    OccupancyGrid, 
                    f'/{robot_name}/map', 
                    self.make_map_callback(robot_name), 
                    map_qos
                )
            )
            
            # Suscripción a la odometría para pillar el spawn inicial tal cual lo escupe Gazebo
            self.odom_subs.append(
                self.create_subscription(
                    Odometry,
                    f'/{robot_name}/odom',
                    self.make_odom_callback(robot_name),
                    10
                )
            )

        self.pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.timer = self.create_timer(0.05, self.merge_and_publish)
        self.get_logger().info(f"Merger Dinámico Iniciado. Esperando posiciones de Gazebo para {self.num_robots} robots...")

    def make_map_callback(self, robot_name):
        return lambda msg: self.map_cb(robot_name, msg)

    def map_cb(self, robot_name, msg):
        self.map_msgs[robot_name] = msg

    def make_odom_callback(self, robot_name):
        return lambda msg: self.odom_cb(robot_name, msg)

    def odom_cb(self, robot_name, msg):
        # Solo atrapamos el primer mensaje para fijar la orientación nativa de Gazebo
        if robot_name not in self.initial_poses:
            # CORRECCIÓN: Tomamos la posición directa sin forzar el offset lateral manual en Y
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # Conversión de Quaternion a Yaw (Radianes)
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            self.initial_poses[robot_name] = (x, y, yaw)
            self.get_logger().info(f"📍 Origen matriz sincronizado para {robot_name}: X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f} rad")

    def merge_and_publish(self):
        if not rclpy.ok():
            return
            
        if len(self.initial_poses) < self.num_robots:
            return
            
        # =========================================================================
        # 1. EMISIÓN DE TFS EN PARALELO (Origen plano unificado para Nav2)
        # =========================================================================
        for robot_name, (_, _, _) in self.initial_poses.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = f'{robot_name}/map'
            
            # CORRECCIÓN CRÍTICA: Las TFs van alineadas a cero absoluto.
            # El submapa local de slam_toolbox ya contiene la traslación nativa del entorno.
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)

        # =========================================================================
        # 2. MOTOR DE FUSIÓN MATRICIAL VECTORIZADA CORREGIDA
        # =========================================================================
        base_msg = next((msg for msg in self.map_msgs.values() if msg is not None), None)
        if not base_msg:
            return

        res = base_msg.info.resolution
        canvas_size = 1500  
        canvas = np.full((canvas_size, canvas_size), -1, dtype=np.int8)
        center_px = canvas_size // 2

        for r_name, msg in self.map_msgs.items():
            if msg and r_name in self.initial_poses:
                w, h = msg.info.width, msg.info.height
                if w == 0 or h == 0: 
                    continue
                
                _, _, init_yaw = self.initial_poses[r_name]
                cos_y = math.cos(init_yaw)
                sin_y = math.sin(init_yaw)
                
                data = np.array(msg.data, dtype=np.int8).reshape((h, w))
                
                valid_y, valid_x = np.where(data != -1)
                vals = data[valid_y, valid_x]
                
                lx = msg.info.origin.position.x + (valid_x * res)
                ly = msg.info.origin.position.y + (valid_y * res)
                
                # Desplazamiento geométrico directo sin añadir el offset manual duplicado
                gx = (lx * cos_y) - (ly * sin_y)
                gy = (lx * sin_y) + (ly * cos_y)
                
                px = center_px + (gx / res).astype(int)
                py = center_px + (gy / res).astype(int)
                
                mask = (px >= 0) & (px < canvas_size) & (py >= 0) & (py < canvas_size)
                canvas[py[mask], px[mask]] = vals[mask]

        # =========================================================================
        # 3. PUBLICACIÓN DEL MENSAJE /MAP DEFINTIVO
        # =========================================================================
        merged_msg = OccupancyGrid()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = 'map'
        merged_msg.info.resolution = res
        merged_msg.info.width = canvas_size
        merged_msg.info.height = canvas_size
        merged_msg.info.origin.position.x = - (center_px * res)
        merged_msg.info.origin.position.y = - (center_px * res)
        merged_msg.data = canvas.flatten().tolist()

        try:
            self.pub.publish(merged_msg)
        except Exception:
            pass
def main(args=None):
    rclpy.init(args=args)
    node = DynamicMapMerger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()