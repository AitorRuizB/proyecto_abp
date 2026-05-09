# class to handle and publish the states of the robot
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

STATES = ['WANDER', 'NAVIGATING_HALLWAY', 'MERGE_SLAM', 'NAV2TARGET']
TRANSITIONS = ['HALLWAY_FOUND', 'TARGET_FOUND', 'GLOBAL_MAP_READY']

TRANSITION_TOPIC = '/transition'  # Topic para publicar transiciones de estado
STATE_TOPIC = '/state'  # Topic para publicar estados del robot

class FiniteStateMachine(Node):
    def __init__(self,robot_id='/robot_0'):
        super().__init__('finite_state_machine')
        self.robot_id = robot_id
        self.state_publisher = self.create_publisher(String, self.robot_id + STATE_TOPIC, 10)
        self.current_state = 'WANDER'  # Estado inicial

        # subcribe to transitions topics to update the state
        self.create_subscription(String, self.robot_id + TRANSITION_TOPIC, self.transition_callback, 10)

        # Create a timer to periodically publish the state at 12Hz
        self.create_timer(1.0 / 12.0, self.periodic_publish)

        self.get_logger().info(f"Finite State Machine for {self.robot_id} initialized.")

    def periodic_publish(self):
        """Called by a timer to periodically publish the current state."""
        self.publish_state(self.get_current_state())

    def publish_state(self, state):
        is_new_state = self.current_state != state
        self.current_state = state
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)
        if is_new_state:
            self.get_logger().info(f'State changed to: {state}')

    def get_current_state(self):
        return self.current_state  

    def transition_callback(self, msg):
        transition = msg.data
        if transition in TRANSITIONS:
            if transition == 'HALLWAY_FOUND' and self.get_current_state() != 'NAVIGATING_HALLWAY':
                self.publish_state('NAVIGATING_HALLWAY')
            elif transition == 'TARGET_FOUND' and self.get_current_state() != 'MERGE_SLAM':
                self.publish_state('MERGE_SLAM')
            elif transition == 'GLOBAL_MAP_READY' and self.get_current_state() != 'NAV2TARGET':
                self.publish_state('NAV2TARGET')

def main(args=None):
    rclpy.init(args=args)
    fsm_node = FiniteStateMachine()
    # The node now handles periodic state publishing internally via a timer.
    try:
        rclpy.spin(fsm_node)
    except KeyboardInterrupt:
        pass
    finally:
        fsm_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
