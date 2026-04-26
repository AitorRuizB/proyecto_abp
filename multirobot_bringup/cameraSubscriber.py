import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Get the namespace from the node
        namespace = self.get_namespace()
        
        # Subscribe to camera image and camera info
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )
        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.bridge = cv_bridge.CvBridge()
        
        self.image_count = 0
        self.camera_info_count = 0
        
        self.get_logger().info(f'Camera subscriber initialized for {namespace}')

    def image_callback(self, msg: Image):
        self.image_count += 1
        """
        if self.image_count % 30 == 0:  # Log every 30 frames
            self.get_logger().info(
                f'Received image #{self.image_count}: '
                f'{msg.width}x{msg.height}, encoding: {msg.encoding}'
            )
        """

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info_count += 1
        """
        if self.camera_info_count == 1:  # Log only once
            self.get_logger().info(
                f'Received camera_info: '
                f'resolution: {msg.width}x{msg.height}, '
                f'fx: {msg.k[0]:.2f}, fy: {msg.k[4]:.2f}'
            )
        """
def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
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