import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

FREQUENCY = 20.0  

POSSIBLE_GOALS = ['green', 'yellow', 'red', 'blue']
STATES = ['WANDER', 'APPROACH_DOOR','NAVIGATING_HALLWAY','APPROACH_TARGET', 'FINISH_SLAM', 'NAV2TARGET']
TRANSITIONS = ['HALLWAY_FOUND', 'DOOR_PASSED','TARGET_APPROACH','TARGET_LOCATED', 'GLOBAL_MAP_READY']

GOAL_TOPIC = '/goal' 
TRANSITION_TOPIC = '/transition'  
STATE_TOPIC = '/state'  

class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__('finite_state_machine')
        
        self.robot_id = self.get_namespace()
        if self.robot_id == '/':
            self.robot_id = '/robot_0'
            
        self.state_publisher = self.create_publisher(String, self.robot_id + STATE_TOPIC, 10)
        self.current_state = STATES[0]  

        self.create_subscription(String, self.robot_id + TRANSITION_TOPIC, self.transition_callback, 10)
        self.goal_publisher = self.create_publisher(String, self.robot_id + GOAL_TOPIC, 10)
        
        self.declare_parameter('goal', 'green')
        self.declare_parameter('num_robots', 2)
        self.current_goal = self.get_parameter('goal').value
        self.num_robots = self.get_parameter('num_robots').value

        self.create_timer(1.0 / FREQUENCY, self.periodic_publish)
        self.get_logger().info(f"FSM para {self.robot_id} inicializada. Objetivo: {self.current_goal}")

    def periodic_publish(self):
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

        # ELIMINADO: Ya no disparamos GLOBAL_MAP_READY desde aquí.
        # Dejamos que el coordinador de start_slam.py lo haga cuando el archivo .yaml exista físicamente.

    def get_current_state(self):
        return self.current_state  

    def publish_goal(self):
        goal_msg = String()
        goal_msg.data = self.current_goal
        self.goal_publisher.publish(goal_msg)

    def transition_callback(self, msg):
        transition = msg.data
        if transition in TRANSITIONS and self.get_current_state() != STATES[TRANSITIONS.index(transition) + 1]:

            if transition == TRANSITIONS[0] and self.get_current_state() != STATES[1]: 
                self.publish_state(STATES[1])
            elif transition == TRANSITIONS[1] and self.get_current_state() != STATES[2]: 
                self.publish_state(STATES[2])
            elif transition == TRANSITIONS[2] and self.get_current_state() != STATES[3]: 
                self.publish_state(STATES[3])
            
            # El robot que encuentra el target pasa a FINISH_SLAM y espera pacientemente
            elif transition == TRANSITIONS[3] and self.get_current_state() != STATES[4]: 
                self.publish_state(STATES[4])

            # Cuando el coordinador avise de que el mapa está guardado, TODOS pasan a NAV2
            elif transition == TRANSITIONS[4] and self.get_current_state() != STATES[5]: 
                self.publish_state(STATES[5])

def main(args=None):
    rclpy.init(args=args)
    fsm_node = FiniteStateMachine()
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