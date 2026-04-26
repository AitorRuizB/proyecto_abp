import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserSubscriber(Node):
    def __init__(self):
        super().__init__('laser_subscriber')
        
        # Get the namespace from the node
        namespace = self.get_namespace()
        
        # Subscribe to laser scan
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        
        self.scan_count = 0
        self.get_logger().info(f'Laser subscriber initialized for {namespace}')

    def laser_callback(self, msg: LaserScan):
        self.scan_count += 1
        """        
        if self.scan_count % 10 == 0:  # Log every 10 scans
            num_ranges = len(msg.ranges)
            min_range = min(msg.ranges) if msg.ranges else float('inf')
            max_range = max(msg.ranges) if msg.ranges else 0
                       self.get_logger().info(
                f'Received scan #{self.scan_count}: '
                f'{num_ranges} ranges, '
                f'min: {min_range:.2f}m, max: {max_range:.2f}m'
            )   
        """


def main(args=None):
    rclpy.init(args=args)
    node = LaserSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # COMPROBACIÓN AÑADIDA PARA EVITAR ERROR AL CERRAR CON CTRL+C
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()