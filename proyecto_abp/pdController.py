import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String
from proyecto_abp.cameraProcessor import ERROR_TOPIC as VISUAL_ERROR_TOPIC, HALLWAY_TOPIC
from proyecto_abp.laserProcessor import ERROR_TOPIC as LASER_ERROR_TOPIC, OBSTACLE_TOPIC
from proyecto_abp.finiteStateMachine import STATES, TRANSITIONS, TRANSITION_TOPIC, STATE_TOPIC, FREQUENCY


VELOCITY_TOPIC = '/cmd_vel'  # Topic para publicar comandos de velocidad
VCONS = 0.25
EPSILON = 25 # visual error in pixels admited
MIN_VISUAL_TRACK_ITER = 15 # iterations of the visual controller to consider it successful and switch to laser based control
class PDControllerParams():

    def __init__(self, kp, kd, sensor_type, is_steering):
        self.kp = kp
        self.kd = kd
        self.sensor_type = sensor_type  # 'visual' o 'laser'
        self.is_steering = is_steering  # True para control de dirección, False para control de velocidad
    
    def getKp(self):
        return self.kp  

    def getKd(self):
        return self.kd
    
    def setKp(self, kp):
        self.kp = kp    
    
    def setKd(self, kd):
        self.kd = kd
        
class PDController(Node):
    def __init__(self):
        super().__init__('pd_controller')
        # PD controller gains para visual y laser 
        self.visualPD_gains = PDControllerParams(kp=0.001, kd=0.0005, sensor_type='visual', is_steering=True)
        self.laserPD_gains = [
            PDControllerParams(kp=1.0, kd=0.1, sensor_type='laser', is_steering=False),
            PDControllerParams(kp=1.0, kd=0.2, sensor_type='laser', is_steering=True)
        ]

        self.previous_visual_error = 0.0
        self.previous_laser_error = 0.0
        self.controller_consecutive_actions_sent = 0 # detectar si ha conseguido minimizar el error visual
        self.fsm_st = STATES[0] # estado inicial de la FSM
        self.transition_hallway_sent = False  # Flag para evitar publicar múltiples transiciones
        self.transition_hallway_counter = 0  # Contador para hacer transición más robusta
        self.transition_target_located_sent = False  # Flag para evitar publicar múltiples transiciones a TARGET_LOCATED

        self.robot_id = self.get_namespace()
        if self.robot_id == '/':
            self.robot_id = '/robot_0'  # Default si no hay namespace
        
        # Suscripción a los topics de la cámara
        self.create_subscription(Float32, self.robot_id + VISUAL_ERROR_TOPIC, self.visual_error_callback, 10)
        self.create_subscription(Bool, self.robot_id + HALLWAY_TOPIC, self.hallway_callback, 10)
        self.create_subscription(Float32, self.robot_id + LASER_ERROR_TOPIC, self.laser_error_callback, 10) 
        self.create_subscription(Bool, self.robot_id + OBSTACLE_TOPIC, self.obstacle_callback, 10)
        self.create_subscription(String, self.robot_id + STATE_TOPIC, self.fsm_callback, 10) # Suscripción a transiciones de estado

        # Publicación de comandos
        self.cmd_vel_publisher = self.create_publisher(Twist, self.robot_id + VELOCITY_TOPIC, 1) # QoS de 1 para comandos de velocidad
        self.transition_publisher = self.create_publisher(String, self.robot_id + TRANSITION_TOPIC, 10) # Publicar transiciones de estado

        # Estado actual del pasillo y el objetivo
        self.hallway_detected = False
        self.visual_error = 0.0
        self.laser_error = 0.0
        self.there_is_obstacle = False
        self.visual_controller_success = False        # Timer para el bucle de control principal
        self.timer = self.create_timer(1.0 / FREQUENCY, self.control_loop) # Ejecutar el bucle de control a 10 Hz
        
        
        self.get_logger().info(f'PDController para {self.robot_id} inicializado.')

    def visual_error_callback(self, msg):
        """Callback para el error visual de la cámara."""
        self.visual_error = msg.data

    def laser_error_callback(self, msg):
        """Callback para el error del láser."""
        self.laser_error = msg.data
        # Solo resetear cuando ya estamos en el estado APPROACH_DOOR (transición completada)
        if self.visual_controller_success and self.fsm_st == STATES[1]:
            self.laser_error = 0.0 
            self.previous_laser_error = 0.0
            self.visual_controller_success = False
            self.transition_hallway_sent = False  # Reset para próxima transición

    def hallway_callback(self, msg):
        """Callback para el estado de detección del pasillo/puerta."""
        self.hallway_detected = msg.data

    def obstacle_callback(self, msg):
        """Callback para la detección de obstáculos."""
        self.there_is_obstacle = msg.data

    def fsm_callback(self, msg):
        """Callback para las transiciones de estado."""
        if msg.data in STATES:
            self.fsm_st = msg.data

    
    def control_loop(self):
        """Bucle de control principal que se ejecuta periódicamente."""
        if self.laser_error is None or self.visual_error is None:
            return
        
        cmd = Twist()
        cmd.linear.x = VCONS  # Usar la velocidad lineal del láser para evitar obstáculos
        control_law = 0.0

        # === MÁQUINA DE ESTADOS PARA TRANSICIÓN VISUAL → LASER ===
        if self.fsm_st == STATES[0]: # WANDER
            if not self.hallway_detected: # Searching hallway
                # Controlador proporcional de velocidad lineal
                cmd.linear.x = VCONS 
                # Controlador PD para steering
                control_law = -0.1
                self.get_logger().info('Buscando pasillo...')
                self.controller_consecutive_actions_sent = 0
                self.transition_hallway_sent = False  # Reset para próxima detección
                self.transition_hallway_counter = 0

            else: # Approach hallway with visual controller
                # Calcular el error y la derivada del error
                derivative = (self.visual_error - self.previous_visual_error) * FREQUENCY
                # Calcular la señal de control PD
                control_law = (self.visualPD_gains.getKp() * self.visual_error) + (self.visualPD_gains.getKd() * derivative)
                self.get_logger().info(f'Aproximando puerta (Visual) - Error: {self.visual_error:.2f}px')
                
                # DETECCIÓN ROBUSTA DE TRANSICIÓN: contador para evitar ruido
                if abs(self.visual_error) <= EPSILON and self.hallway_detected:
                    self.transition_hallway_counter += 1
                    if self.transition_hallway_counter >= 3:  # Requiere 3 iteraciones consecutivas (150ms @ 20Hz)
                        self.visual_controller_success = True
                        self.transition_hallway_counter = 0
                else:
                    self.transition_hallway_counter = 0

        elif self.fsm_st == STATES[1]: # APPROACH_DOOR
            # Controlador proporcional de velocidad lineal
            cmd.linear.x = VCONS * self.laserPD_gains[0].getKp()
            # Controlador PD para steering
            control_law = (self.laserPD_gains[1].getKp() * self.laser_error) + (self.laserPD_gains[1].getKd() * (self.laser_error - self.previous_laser_error) * FREQUENCY)
            self.get_logger().info(f'Aproximando puerta (Laser) - Error: {self.laser_error:.4f}')
        
        # Estado navegacion por el pasillo -> PD laser based control con nuevas ganancias y umbrales
        elif self.fsm_st == STATES[2]: # NAVIGATING_HALLWAY
            # Controlador proporcional de velocidad lineal
            cmd.linear.x = VCONS * self.laserPD_gains[0].getKp()
            self.laserPD_gains[1].setKp(1.2) # increase gain as quick turns are required
            self.laserPD_gains[1].setKd(0.19)
            # Controlador PD para steering
            control_law = (self.laserPD_gains[1].getKp() * self.laser_error) + (self.laserPD_gains[1].getKd() * (self.laser_error - self.previous_laser_error) * FREQUENCY)
            self.get_logger().info('Navegando pasillo...')

        elif self.fsm_st == STATES[3] and not self.transition_target_located_sent: # Approach target
            # Calcular el error y la derivada del error
            derivative = (self.visual_error - self.previous_visual_error) * FREQUENCY
            # Calcular la señal de control PD
            control_law = (self.visualPD_gains.getKp() * self.visual_error) + (self.visualPD_gains.getKd() * derivative)
            cmd.linear.x = VCONS * 0.5 # reducir velocidad para aproximación al objetivo
            self.get_logger().info(f'Aproximando objetivo (Visual) - Error: {self.visual_error:.2f}px')

            # Detección robusta de objetivo para transición a TARGET_LOCATED
            if abs(self.visual_error) <= EPSILON:
                self.transition_publisher.publish(String(data=TRANSITIONS[3])) # Publica TARGET_LOCATED
                self.transition_target_located_sent = True
                self.get_logger().info('✓ Transición publicada: TARGET_LOCATED')
                self.destroy_node=True
            else:
                self.transition_target_counter = 0 # Resetea el contador si el error es grande

        elif self.fsm_st == STATES[4]: # TARGET_LOCATED
            control_law = 0.0
            cmd.linear.x = 0.0 
            self.get_logger().info('Objetivo alcanzado')
            

        # === PUBLICAR TRANSICIÓN (UNA SOLA VEZ) ===
        if self.visual_controller_success and not self.transition_hallway_sent and self.fsm_st == STATES[0]: # Solo publicar transición si el controlador visual ha tenido éxito y no se ha publicado antes
            self.transition_publisher.publish(String(data=TRANSITIONS[1]))  # HALLWAY_FOUND
            self.transition_hallway_sent = True
            self.get_logger().info('✓ Transición publicada: HALLWAY_FOUND')
            
        # Asignar steering
        cmd.angular.z = -control_law
        
        #self.get_logger().info(f'Control Law: {control_law:.4f}, Linear Vel: {cmd.linear.x:.2f}, Angular Vel: {cmd.angular.z:.4f}')
        self.cmd_vel_publisher.publish(cmd)
        # actualizar error de los sensores
        self.previous_visual_error = self.visual_error
        self.previous_laser_error = self.laser_error


# -------------------------------- ZONA DE PRUEBAS DEL CONTROLADOR PD ------------------------------------------
def main(args=None):
    """Función principal para inicializar y ejecutar el nodo PDController."""
    rclpy.init(args=args)
    pd_controller = PDController()

    # rclpy.spin() se encargará de ejecutar el timer y los callbacks
    rclpy.spin(pd_controller)

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
