#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np

class DynamicMapMerger(Node):
    def __init__(self):
        super().__init__('custom_map_merger')
        
        self.declare_parameter('num_robots', 2)
        self.num_robots = self.get_parameter('num_robots').value

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_msgs = {}   
        self.offsets_y = {}  
        self.subs = []       

        for i in range(0, self.num_robots):
            robot_name = f'robot_{i}'
            topic = f'/{robot_name}/map'
            
            self.offsets_y[robot_name] = float(i) * 2.5 # Hardcoded bs
            self.map_msgs[robot_name] = None
            
            self.subs.append(
                self.create_subscription(
                    OccupancyGrid, 
                    topic, 
                    self.make_callback(robot_name), 
                    map_qos
                )
            )

        self.pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        
        self.timer = self.create_timer(1.0, self.merge_and_publish)
        self.get_logger().info(f"Merger configurado con QoS TRANSIENT_LOCAL para {self.num_robots} robots.")

    def make_callback(self, robot_name):
        return lambda msg: self.map_cb(robot_name, msg)

    def map_cb(self, robot_name, msg):
        self.map_msgs[robot_name] = msg

    def merge_and_publish(self):
        # CORRECCIÓN: Si el contexto ya no es válido por un apagado en curso, salimos para evitar la condición de carrera
        if not rclpy.ok():
            return
            
        base_msg = next((msg for msg in self.map_msgs.values() if msg is not None), None)
        if not base_msg:
            return

        res = base_msg.info.resolution
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

            if px_start_x < 0 or px_start_y < 0 or px_start_x + w > canvas_size or px_start_y + h > canvas_size:
                return

            valid_mask = data != -1
            canvas[px_start_y:px_start_y+h, px_start_x:px_start_x+w][valid_mask] = data[valid_mask]

        for r_name, msg in self.map_msgs.items():
            if msg:
                paste_map(msg, self.offsets_y[r_name])

        merged_msg = OccupancyGrid()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = 'map'
        merged_msg.info.resolution = res
        merged_msg.info.width = canvas_size
        merged_msg.info.height = canvas_size
        merged_msg.info.origin.position.x = - (center_px * res)
        merged_msg.info.origin.position.y = - (center_px * res)
        merged_msg.data = canvas.flatten().tolist()

        # CORRECCIÓN: Envolver la publicación en un try-except por si se destruye el contexto a medio camino
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