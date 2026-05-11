import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        
        # Current robot index
        self.current_robot = 0
        self.robots = ['robot_0', 'robot_1', 'robot_2']
        
        # Velocities
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Step sizes
        self.linear_step = 0.2
        self.angular_step = 0.1
        
        # Publisher
        self.publisher_ = self.create_publisher(
            Twist, f'/{self.robots[self.current_robot]}/cmd_vel', 10
        )
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        # Control flag
        self.running = True
        
        self.get_logger().info(
            f'Teleop Node started. Controlling {self.robots[self.current_robot]}'
        )
        self.print_help()
        
    def print_help(self):
        """Print control instructions"""
        print("\n" + "="*40)
        print("        TELEOP CONTROL - Robot Driver")
        print("="*40)
        print(f"Current Robot: {self.robots[self.current_robot]}")
        print("-"*40)
        print("Controls:")
        print("  W  - Increase linear velocity (+0.2 m/s)")
        print("  S  - Decrease linear velocity (-0.2 m/s)")
        print("  A  - Increase angular velocity (+0.1 rad/s)")
        print("  D  - Decrease angular velocity (-0.1 rad/s)")
        print("  <-/->  - Switch between robots")
        print("  Q  - Quit (stops the robot)")
        print("  E  - Emergency Stop (zero velocities)")
        print("="*40 + "\n")
        
    def print_status(self):
        """Print current control status"""
        print(
            f"\rRobot: {self.robots[self.current_robot]:<10} | "
            f"linear.x: {self.linear_x:>6.2f} m/s | "
            f"angular.z: {self.angular_z:>6.2f} rad/s",
            end='',
            flush=True
        )
        
    def publish_velocity(self):
        """Publish current velocity command"""
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_x
        twist_msg.angular.z = self.angular_z
        self.publisher_.publish(twist_msg)
        
    def get_key(self):
        """Get key input without blocking"""
        if select.select([sys.stdin], [], [], 0.01)[0]:
            try:
                key = sys.stdin.read(1)
                # Handle escape sequences for arrow keys
                if key == '\x1b':
                    next_chars = sys.stdin.read(2)
                    key += next_chars
                return key
            except Exception as e:
                self.get_logger().error(f"Error reading key: {e}")
                return None
        return None
    
    def switch_robot(self, direction):
        """Switch to next or previous robot"""
        if direction == 'next':
            self.current_robot = (self.current_robot + 1) % len(self.robots)
        elif direction == 'prev':
            self.current_robot = (self.current_robot - 1) % len(self.robots)
        
        # Create new publisher for the new robot
        self.publisher_ = self.create_publisher(
            Twist, f'/{self.robots[self.current_robot]}/cmd_vel', 10
        )
        
        # Send zero velocity to previous robot before switching
        zero_msg = Twist()
        zero_msg.linear.x = 0.0
        zero_msg.angular.z = 0.0
        self.publisher_.publish(zero_msg)
        print(f"\n[INFO] Switched to {self.robots[self.current_robot]}")
        self.print_help()
        
    def handle_input(self):
        """Handle keyboard input"""
        key = self.get_key()
        
        if key is None:
            return True
        
        key_lower = key.lower()
        
        if key_lower == 'q':
            print("\n[INFO] Shutting down teleop node...")
            self.linear_x = 0.0
            self.angular_z = 0.0
            return False
            
        elif key_lower == 'w':
            self.linear_x += self.linear_step
            self.print_status()
            
        elif key_lower == 's':
            self.linear_x -= self.linear_step
            self.print_status()
            
        elif key_lower == 'a':
            self.angular_z += self.angular_step
            self.print_status()
            
        elif key_lower == 'd':
            self.angular_z -= self.angular_step
            self.print_status()
            
        elif key == '\x1b[C':  # Right arrow
            self.switch_robot('next')
            
        elif key == '\x1b[D':  # Left arrow
            self.switch_robot('prev')

        elif key == 'e':  # Emergency stop
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.publish_velocity()
            print("\n[EMERGENCY STOP] All velocities set to zero!")
            self.print_status()
        
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
    try:
        while rclpy.ok() and node.running:
            if not node.handle_input():
                node.running = False
                break
            # Small spin to process callbacks
            rclpy.spin_once(node, timeout_sec=0.1)
    
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt received")
        
    finally:
        # Send zero velocities before shutting down
        node.linear_x = 0.0
        node.angular_z = 0.0
        node.publish_velocity()
        rclpy.spin_once(node, timeout_sec=0.1)  # Ensure message is sent
        
        # Restore terminal settings
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        except Exception:
            pass
        
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        print("\n[INFO] Teleop node terminated")


if __name__ == '__main__':
    main()
