#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
import math

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class DynamicMapMerger(Node):
    def __init__(self):
        super().__init__('custom_map_merger')
        
        self.declare_parameter('num_robots', 2)
        self.num_robots = self.get_parameter('num_robots').value

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- NUEVO: Listener de Transformaciones ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # -------------------------------------------

        self.map_msgs = {}   
        self.map_subs = []       

        for i in range(0, self.num_robots):
            robot_name = f'robot_{i}'
            self.map_msgs[robot_name] = None
            
            self.map_subs.append(
                self.create_subscription(
                    OccupancyGrid, 
                    f'/{robot_name}/map', 
                    self.make_map_callback(robot_name), 
                    map_qos
                )
            )

        self.pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.timer = self.create_timer(0.2, self.merge_and_publish) # Aumentado a 0.2s para no saturar CPU
        
        self.get_logger().info(f"✅ Merger TF Iniciado. Escuchando TFs globales para fusionar mapas...")

    def make_map_callback(self, robot_name):
        return lambda msg: self.map_cb(robot_name, msg)

    def map_cb(self, robot_name, msg):
        self.map_msgs[robot_name] = msg

    def merge_and_publish(self):
        if not rclpy.ok():
            return
            
        valid_maps = [(r, msg) for r, msg in self.map_msgs.items() if msg is not None]
        if not valid_maps:
            return

        res = valid_maps[0][1].info.resolution
        canvas_size = 1500  
        canvas = np.full((canvas_size, canvas_size), -1, dtype=np.int8)
        center_px = canvas_size // 2

        for r_name, msg in valid_maps:
            w, h = msg.info.width, msg.info.height
            if w == 0 or h == 0: 
                continue

            # 1. PREGUNTAR AL ÁRBOL TF DÓNDE ESTÁ ESTE MAPA RESPECTO AL MAPA GLOBAL
            try:
                # Buscamos la transformación desde el 'map' global hasta el frame del mapa local (ej. 'robot_0/map')
                t = self.tf_buffer.lookup_transform(
                    'map',
                    msg.header.frame_id,
                    rclpy.time.Time()
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                # Si la TF aún no existe, nos saltamos este robot temporalmente
                continue

            # 2. EXTRAER TRASLACIÓN Y ROTACIÓN (YAW)
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            # 3. EXTRAER DATOS MATRICIALES
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
            valid_y, valid_x = np.where(data != -1)
            vals = data[valid_y, valid_x]
            
            # Coordenadas locales dentro del submapa del robot
            lx = msg.info.origin.position.x + (valid_x * res)
            ly = msg.info.origin.position.y + (valid_y * res)
            
            # 4. APLICAR TRANSFORMACIÓN VECTORIZADA (Rotación + Traslación del launch)
            gx = tx + (lx * cos_yaw - ly * sin_yaw)
            gy = ty + (lx * sin_yaw + ly * cos_yaw)
            
            # Proyección sobre el lienzo central global
            px = center_px + (gx / res).astype(int)
            py = center_px + (gy / res).astype(int)
            
            # 5. DIBUJAR EN EL LIENZO
            mask = (px >= 0) & (px < canvas_size) & (py >= 0) & (py < canvas_size)
            
            # Priorizar obstáculos (100) sobre espacio libre (0) al solapar
            current_vals = canvas[py[mask], px[mask]]
            new_vals = vals[mask]
            
            # Si el pixel actual es desconocido (-1) o si el nuevo pixel es obstáculo, sobrescribimos
            canvas[py[mask], px[mask]] = np.where((current_vals == -1) | (new_vals > current_vals), new_vals, current_vals)

        # 6. PUBLICACIÓN
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