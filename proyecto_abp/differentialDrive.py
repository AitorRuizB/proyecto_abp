import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DifferentialDrive(Node):
    def __init__(self):
        super().__init__('differential_drive_node')
        # CHANGE: removed '/' to allow namespacing (e.g. /robot_0/cmd_vel)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.get_logger().info('Differential Drive Node started')

    def publish_velocity(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0  # Forward velocity
        twist_msg.angular.z = 1.0  # Rotation velocity (1 rad/s)
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDrive()
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