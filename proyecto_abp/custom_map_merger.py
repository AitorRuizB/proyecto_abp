#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class DynamicMapMerger(Node):
    def __init__(self):
        super().__init__('custom_map_merger')
        
        # 1. Declaramos y leemos el parámetro 'num_robots' (por defecto 2)
        self.declare_parameter('num_robots', 2)
        self.num_robots = self.get_parameter('num_robots').value

        # Diccionarios dinámicos para N robots
        self.map_msgs = {}   # Guarda el mapa de cada robot
        self.offsets_y = {}  # Guarda el desplazamiento Y de cada robot
        self.subs = []       # Guarda las suscripciones para que no mueran

        # 2. BUCLE MÁGICO: Creamos los suscriptores y offsets dinámicamente
        for i in range(1, self.num_robots + 1):
            robot_name = f'robot{i}'
            topic = f'/{robot_name}/map'
            
            # Calculamos la misma distancia matemática que usas en el Launch
            self.offsets_y[robot_name] = float(i - 1) * 1.0 
            self.map_msgs[robot_name] = None
            
            # Nos suscribimos usando una función generadora para no mezclar variables en el bucle
            self.subs.append(
                self.create_subscription(
                    OccupancyGrid, 
                    topic, 
                    self.make_callback(robot_name), 
                    10
                )
            )

        self.pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(0.5, self.merge_and_publish)
        
        self.get_logger().info(f"¡Cerebro Python INICIADO y escuchando a {self.num_robots} robots!")

    # Función factoría para crear callbacks independientes por robot
    def make_callback(self, robot_name):
        return lambda msg: self.map_cb(robot_name, msg)

    def map_cb(self, robot_name, msg):
        self.map_msgs[robot_name] = msg

    def merge_and_publish(self):
        # Buscamos el primer mapa que no sea nulo para sacar la resolución
        base_msg = next((msg for msg in self.map_msgs.values() if msg is not None), None)
        if not base_msg:
            return

        res = base_msg.info.resolution
        
        # Lienzo gigante (75x75 metros)
        canvas_size = 1500
        canvas = np.full((canvas_size, canvas_size), -1, dtype=np.int8)
        center_px = canvas_size // 2

        def paste_map(msg, offset_y_m):
            w, h = msg.info.width, msg.info.height
            if w == 0 or h == 0: return
            
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y + offset_y_m

            px_start_x = center_px + int(origin_x / res)
            px_start_y = center_px + int(origin_y / res)

            # Evitar salirnos del lienzo
            if px_start_x < 0 or px_start_y < 0 or px_start_x + w > canvas_size or px_start_y + h > canvas_size:
                return

            valid_mask = data != -1
            canvas[px_start_y:px_start_y+h, px_start_x:px_start_x+w][valid_mask] = data[valid_mask]

        # Pegamos TODO el diccionario de mapas en el lienzo
        for r_name, msg in self.map_msgs.items():
            if msg:
                paste_map(msg, self.offsets_y[r_name])

        # Publicamos
        merged_msg = OccupancyGrid()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = 'map'
        merged_msg.info.resolution = res
        merged_msg.info.width = canvas_size
        merged_msg.info.height = canvas_size
        merged_msg.info.origin.position.x = - (center_px * res)
        merged_msg.info.origin.position.y = - (center_px * res)
        merged_msg.data = canvas.flatten().tolist()

        self.pub.publish(merged_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicMapMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()