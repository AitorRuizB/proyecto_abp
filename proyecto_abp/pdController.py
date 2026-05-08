import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan
from cameraProcessor import ERROR_TOPIC as VISUAL_ERROR_TOPIC, HALLWAY_TOPIC
from laserProcessor import ERROR_TOPIC as LASER_ERROR_TOPIC, OBSTACLE_TOPIC

SCAN_TOPIC = '/scan'  # Topic del laser
VELOCITY_TOPIC = '/cmd_vel'  # Topic para publicar comandos de velocidad
FREQUENCY = 10.0  # Frecuencia de control en Hz
VCONS = 0.25

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
    def __init__(self, robot_id):
        super().__init__('pd_controller')
        
        # PD controller gains para visual y laser 
        self.visualPD_gains = PDControllerParams(kp=0.001, kd=0.0005, sensor_type='visual', is_steering=True)
        self.laserPD_gains = [
            PDControllerParams(kp=1.0, kd=0.0, sensor_type='laser', is_steering=False),
            PDControllerParams(kp=0.5, kd=0.01, sensor_type='laser', is_steering=True)
        ]

        self.previous_visual_error = 0.0
        self.previous_laser_error = 0.0
        self.controller_consecutive_actions_sent = 0 # detectar si ha conseguido minimizar el error visual
        self.controller_successful = False #flag para dejar de buscar pq se localizo el pasillo y el controlador funciono
        
        self.robot_id = robot_id # id is a namespace like '/robot_1'
        
        # Suscripción a los topics de la cámara
        self.create_subscription(Float32, self.robot_id + VISUAL_ERROR_TOPIC, self.visual_error_callback, 10)
        self.create_subscription(Bool, self.robot_id + HALLWAY_TOPIC, self.hallway_callback, 10)
        self.create_subscription(Float32, self.robot_id + LASER_ERROR_TOPIC, self.laser_error_callback, 10) 
        self.create_subscription(Bool, self.robot_id + OBSTACLE_TOPIC, self.obstacle_callback, 10)
         # Para leer el laser y evitar obstáculos
        
        # Publicación de comandos de velocidad
        self.cmd_vel_publisher = self.create_publisher(Twist, self.robot_id + VELOCITY_TOPIC, 1) # QoS de 1 para comandos de velocidad
        
        # Estado actual del pasillo y el objetivo
        self.hallway_detected = False
        self.visual_error = None
        self.laser_error = None
        self.there_is_obstacle = False
        # Timer para el bucle de control principal
        self.timer = self.create_timer(1.0 / FREQUENCY, self.control_loop) # Ejecutar el bucle de control a 10 Hz
        
        self.get_logger().info(f'PDController para {self.robot_id} inicializado.')

    def visual_error_callback(self, msg):
        """Callback para el error visual de la cámara."""
        self.visual_error = msg.data

    def laser_error_callback(self, msg):
        """Callback para el error del láser."""
        self.laser_error = msg.data

    def hallway_callback(self, msg):
        """Callback para el estado de detección del pasillo/puerta."""
        self.hallway_detected = msg.data

    def obstacle_callback(self, msg):
        """Callback para la detección de obstáculos."""
        self.there_is_obstacle = msg.data

    def read_laser_scan(self):
        """
        Calcular mediante Vfh la ley de control para direccion
        Retorna (velocidad_lineal, velocidad_angular_adicional_por_obstaculo).
        """
        
    
    def control_loop(self):
        """Bucle de control principal que se ejecuta periódicamente."""
        if self.laser_error is None or self.visual_error is None:
            return
        
        cmd = Twist()
        cmd.linear.x = VCONS  # Usar la velocidad lineal del láser para evitar obstáculos
        
        controller_success = self.controller_consecutive_actions_sent > 5 and abs(self.previous_visual_error) == 1.0

        if controller_success: 
            self.controller_successful = True

        # Estado de wander & search: usar ley de control del Laser
        if (not self.hallway_detected and not self.controller_successful) or self.there_is_obstacle:
            # Controlador proporcional de velocidad lineal
            cmd.linear.x = VCONS * self.laserPD_gains[0].getKp() # controlador porporcional de velocidad lineal
            # Controlador PD para steering
            control_law = (self.laserPD_gains[1].getKp() * self.laser_error) + (self.laserPD_gains[1].getKd() * (self.laser_error - self.previous_laser_error) * FREQUENCY)
            
            self.get_logger().info('Buscando pasillo...')
            self.controller_consecutive_actions_sent = 0
            
        # Estado identificacion pasillo -> PD visual based control
        else:
            # Calcular el error y la derivada del error
            derivative = (self.visual_error - self.previous_visual_error) * FREQUENCY
            # Calcular la señal de control PD
            control_law = (self.visualPD_gains.getKp() * self.visual_error) + (self.visualPD_gains.getKd() * derivative)
            
        # Asginar steering
        cmd.angular.z = -control_law
        self.get_logger().info(f"VLineal: {cmd.linear.x:.2f}, Angular: {cmd.angular.z:.2f}, Error Visual: {self.visual_error:.2f}, Error Laser: {self.laser_error:.2f}, Obstacle?: {self.there_is_obstacle}")
        self.controller_consecutive_actions_sent += 1

        self.cmd_vel_publisher.publish(cmd)
        # actualizar error de los sensores
        self.previous_visual_error = self.visual_error
        self.previous_laser_error = self.laser_error

# -------------------------------- ZONA DE PRUEBAS DEL CONTROLADOR PD ------------------------------------------
def main(args=None):
    """Función principal para inicializar y ejecutar el nodo PDController."""
    rclpy.init(args=args)
    robot_id = '/robot_0'  # Cambia esto según el robot que quieras controlar
    pd_controller = PDController(robot_id)

    # rclpy.spin() se encargará de ejecutar el timer y los callbacks
    rclpy.spin(pd_controller)

    pd_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
