# class to handle and publish the states of the robot
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

FREQUENCY = 20.0  # Frecuencia de control del MRS en Hz (sincronizada con cámara 30Hz)

POSSIBLE_GOALS = ['green', 'yellow', 'red', 'blue']
STATES = ['WANDER', 'APPROACH_DOOR','NAVIGATING_HALLWAY','APPROACH_TARGET', 'FINISH_SLAM', 'NAV2TARGET']
TRANSITIONS = ['HALLWAY_FOUND', 'DOOR_PASSED','TARGET_APPROACH','TARGET_LOCATED', 'GLOBAL_MAP_READY']

GOAL_TOPIC = '/goal' # String indicando el objetivo a buscar
TRANSITION_TOPIC = '/transition'  # Topic para publicar transiciones de estado
STATE_TOPIC = '/state'  # Topic para publicar estados del robot

class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__('finite_state_machine')
        
        # Obtener el namespace dinámicamente
        self.robot_id = self.get_namespace()
        if self.robot_id == '/':
            self.robot_id = '/robot_0'  # Default si no hay namespace
        self.state_publisher = self.create_publisher(String, self.robot_id + STATE_TOPIC, 10)
        self.current_state = STATES[0]  # Estado inicial

        # subcribe to transitions topics to update the state
        self.create_subscription(String, self.robot_id + TRANSITION_TOPIC, self.transition_callback, 10)
        self.goal_publisher = self.create_publisher(String, self.robot_id + GOAL_TOPIC, 10)
        
        # Leer el parámetro goal y num_robots desde la configuración de ROS
        self.declare_parameter('goal', 'green')
        self.declare_parameter('num_robots', 2)
        self.current_goal = self.get_parameter('goal').value
        self.num_robots = self.get_parameter('num_robots').value

        # Crear publishers para notificar a OTROS robots
        self.other_robots_transition_pubs = []
        current_robot_name = self.robot_id.strip('/')
        for i in range(self.num_robots):
            other_robot_name = f'robot_{i}'
            if other_robot_name != current_robot_name:
                topic = f'/{other_robot_name}{TRANSITION_TOPIC}'
                self.other_robots_transition_pubs.append(
                    self.create_publisher(String, topic, 10)
                )

        # Create a timer to periodically publish the state at 12Hz
        self.create_timer(1.0 / FREQUENCY, self.periodic_publish)

        self.get_logger().info(f"FSM para {self.robot_id} inicializada. Objetivo: {self.current_goal}. Total de robots: {self.num_robots}.")

    def periodic_publish(self):
        """Called by a timer to periodically publish the current state and goal."""
        self.publish_state(self.get_current_state())
        self.publish_goal()

    def publish_state(self, state):
        is_new_state = self.current_state != state
        self.current_state = state
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)
        if is_new_state:
            self.get_logger().info(f'State changed to: {state}')

        # Si este robot alcanza FINISH_SLAM (STATES[4]), notifica al resto para que pasen a NAV2.
        if state == STATES[4]:
            self.get_logger().info(f"¡{self.robot_id.strip('/')} alcanzó FINISH_SLAM! Notificando a otros robots.")
            transition_msg = String()
            transition_msg.data = TRANSITIONS[4] # GLOBAL_MAP_READY
            for pub in self.other_robots_transition_pubs:
                pub.publish(transition_msg)

    def get_current_state(self):
        return self.current_state  

    def publish_goal(self):
        """Publica el objetivo actual en el topic /goal."""
        goal_msg = String()
        goal_msg.data = self.current_goal
        self.goal_publisher.publish(goal_msg)

    def transition_callback(self, msg):
        transition = msg.data
        if transition in TRANSITIONS and self.get_current_state() != STATES[TRANSITIONS.index(transition) + 1]:

            # Hallway found
            if transition == TRANSITIONS[0] and self.get_current_state() != STATES[1]: # switch to Approach Door
                self.publish_state(STATES[1])

            # Door Passed
            elif transition == TRANSITIONS[1] and self.get_current_state() != STATES[2]: # switch to NavHallway
                self.publish_state(STATES[2])
            
            # Target Approach
            elif transition == TRANSITIONS[2] and self.get_current_state() != STATES[3]: # switch to Approach Target
                self.publish_state(STATES[3])

            # Target Located
            elif transition == TRANSITIONS[3] and self.get_current_state() != STATES[4]: # switch to Finish SLAM
                self.publish_state(STATES[4])

            # Global Map Ready
            elif transition == TRANSITIONS[4] and self.get_current_state() in [STATES[0], STATES[1], STATES[2]]: # switch to Nav2Target except for the robot who found the target
                self.publish_state(STATES[5])

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
