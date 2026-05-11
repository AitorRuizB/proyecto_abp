import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterEvent
from sensor_msgs.msg import Image, LaserScan


class SensorMonitor(Node):
    """Monitor camera and laser sensor data from robots"""
    
    def __init__(self, robot_name):
        super().__init__('sensor_monitor', namespace=robot_name)
        self.robot_name = robot_name
        
        # Subscribe to camera image
        self.camera_subscription = self.create_subscription(
            Image,
            'camera/image',
            self.camera_callback,
            10
        )
        
        # Subscribe to laser scan
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        
        self.image_count = 0
        self.scan_count = 0
        self.get_logger().info(f'Sensor monitor initialized for {robot_name}')

    def camera_callback(self, msg: Image):
        self.image_count += 1
        if self.image_count % 30 == 0:  # Log every 30 frames
            self.get_logger().info(
                f'[CAMERA] Received image #{self.image_count}: '
                f'{msg.width}x{msg.height}, encoding: {msg.encoding}'
            )

    def laser_callback(self, msg: LaserScan):
        self.scan_count += 1
        if self.scan_count % 10 == 0:  # Log every 10 scans
            num_ranges = len(msg.ranges)
            valid_ranges = [r for r in msg.ranges if r > 0]
            if valid_ranges:
                avg_range = sum(valid_ranges) / len(valid_ranges)
                self.get_logger().info(
                    f'[LASER] Received scan #{self.scan_count}: '
                    f'{num_ranges} ranges, avg: {avg_range:.2f}m'
                )


class RobotMonitorManager(Node):
    """Manages sensor monitors for all robots based on num_robots parameter"""
    
    def __init__(self, executor):
        super().__init__('robot_monitor_manager')
        self.executor = executor
        self.monitors = []
        self.num_robots = 3  # Default value
        
        # Declare parameter
        self.declare_parameter('num_robots', 3)
        
        # Get initial value
        self.num_robots = self.get_parameter('num_robots').value
        self.get_logger().info(f'Monitor Manager initialized with num_robots={self.num_robots}')
        
        # Subscribe to parameter changes
        self.param_subscriber = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.parameter_callback,
            10
        )
        
        # Create initial monitors
        self.create_monitors(self.num_robots)

    def parameter_callback(self, msg: ParameterEvent):
        """Handle parameter changes"""
        if msg.node == '/robot_monitor_manager':
            for param in msg.new_parameters:
                if param.name == 'num_robots':
                    new_num_robots = param.value.integer_value
                    if new_num_robots != self.num_robots:
                        self.get_logger().info(
                            f'Parameter update: num_robots changed from {self.num_robots} to {new_num_robots}'
                        )
                        self.update_monitors(new_num_robots)
                        self.num_robots = new_num_robots

    def create_monitors(self, num_robots):
        """Create sensor monitor nodes for all robots"""
        for i in range(num_robots):
            robot_name = f'robot_{i}'
            monitor = SensorMonitor(robot_name)
            self.monitors.append(monitor)
            self.executor.add_node(monitor)
            self.get_logger().info(f'Created sensor monitor for {robot_name}')

    def update_monitors(self, new_num_robots):
        """Update number of monitors based on new num_robots value"""
        current_count = len(self.monitors)
        
        if new_num_robots > current_count:
            # Add new monitors
            for i in range(current_count, new_num_robots):
                robot_name = f'robot_{i}'
                monitor = SensorMonitor(robot_name)
                self.monitors.append(monitor)
                self.executor.add_node(monitor)
                self.get_logger().info(f'Created sensor monitor for {robot_name}')
                
        elif new_num_robots < current_count:
            # Remove excess monitors
            monitors_to_remove = self.monitors[new_num_robots:]
            for monitor in monitors_to_remove:
                self.executor.remove_node(monitor)
                monitor.destroy_node()
                self.get_logger().info(f'Removed sensor monitor for {monitor.robot_name}')
            self.monitors = self.monitors[:new_num_robots]

    def destroy_all_monitors(self):
        """Clean up all monitor nodes"""
        for monitor in self.monitors:
            monitor.destroy_node()
        self.monitors.clear()


def main(args=None):
    rclpy.init(args=args)
    
    # Create executor with multiple threads
    executor = MultiThreadedExecutor()
    
    # Create monitor manager node
    monitor_manager = RobotMonitorManager(executor)
    
    # Add node to executor
    executor.add_node(monitor_manager)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Se detiene el ejecutor primero
        executor.shutdown()
        
        # Clean up monitors first
        monitor_manager.destroy_all_monitors()
        
        # Destroy main node
        monitor_manager.destroy_node()

        # COMPROBACIÓN AÑADIDA PARA EVITAR ERROR AL CERRAR CON CTRL+C
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()