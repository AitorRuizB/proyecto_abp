import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan
from cameraProcessor import ERROR_TOPIC, HALLWAY_TOPIC

SCAN_TOPIC = '/scan'  # Topic del laser
VELOCITY_TOPIC = '/cmd_vel'  # Topic para publicar comandos de velocidad
FREQUENCY = 10.0  # Frecuencia de control en Hz
THETA = 0.4 # rad/s
VCONS = 0.25
class PDController(Node):
    def __init__(self, robot_id):
        super().__init__('pd_controller')
        
        # Parámetros del controlador PD visual based para steering
        self.kpSteering = 0.001  # Ganancia proporcional
        self.kdSteering = 0.0005  # Ganancia derivativa
        # Controlador PD para velocidad lineal basado en el laser
        self.kpLineal = 0.05 # Ganancia para el control basado en el laser
        self.kdLineal = 0.01 # Ganancia derivativa para el control basado en el laser
        self.previous_visual_error = 0.0
        self.previous_laser_data = None
        self.controller_consecutive_actions_sent = 0 # detectar si ha conseguido minimizar el error visual
        self.controller_successful = False #flag para dejar de buscar pq se localizo el pasillo y el controlador funciono
        
        self.robot_id = robot_id # id is a namespace like '/robot_1'
        
        # Suscripción a los topics de la cámara
        self.create_subscription(Float32, self.robot_id + ERROR_TOPIC, self.error_callback, 10)
        self.create_subscription(Bool, self.robot_id + HALLWAY_TOPIC, self.hallway_callback, 10)
        self.create_subscription(LaserScan, self.robot_id + SCAN_TOPIC, self.laser_callback, 10)  # Para leer el laser y evitar obstáculos
        
        # Publicación de comandos de velocidad
        self.cmd_vel_publisher = self.create_publisher(Twist, self.robot_id + VELOCITY_TOPIC, 1) # QoS de 1 para comandos de velocidad
        
        # Estado actual del pasillo y el objetivo
        self.hallway_detected = False
        self.visual_error = None
        
        # Timer para el bucle de control principal
        self.timer = self.create_timer(1.0 / FREQUENCY, self.control_loop) # Ejecutar el bucle de control a 10 Hz
        
        self.get_logger().info(f'PDController para {self.robot_id} inicializado.')

    def error_callback(self, msg):
        """Callback para el error visual de la cámara."""
        self.visual_error = msg.data

    def hallway_callback(self, msg):
        """Callback para el estado de detección del pasillo/puerta."""
        self.hallway_detected = msg.data

    def laser_callback(self, msg):
        """Callback para los datos del sensor láser."""
        # Almacenar los rangos del láser para su procesamiento posterior
        self.previous_laser_data = msg.ranges

    def read_laser_scan(self):
        """
        Procesa los datos del láser para evitar obstáculos y determinar la velocidad lineal.
        Esta es una implementación básica y debe ser mejorada para una evitación de obstáculos robusta.
        Retorna (velocidad_lineal, velocidad_angular_adicional_por_obstaculo).
        """
        linear_velocity = 0.2 # Velocidad lineal por defecto
        angular_avoidance = 0.0 # Velocidad angular para evitar obstáculos
        
        if self.previous_laser_data is None or True:
            self.get_logger().warn('No se han recibido datos del láser aún.')
            return 0.0, 0.0 # Detener si no hay datos del láser
        
        # Definir un umbral de distancia para considerar un obstáculo
        obstacle_distance_threshold = 0.5 # metros
        
        # Buscar el obstáculo más cercano en el frente
        # Asumimos que los rangos del láser están ordenados de izquierda a derecha o viceversa
        # y que el frente está en el centro de los rangos.
        # Esto es una simplificación; un LiDAR real tiene un ángulo_min, ángulo_max y incremento.
        num_ranges = len(self.previous_laser_data)
        if num_ranges == 0:
            return linear_velocity, angular_avoidance
            
        # Considerar un segmento frontal para la detección de obstáculos
        front_segment_size = num_ranges // 4 # Por ejemplo, el 25% central de los rangos
        front_start_index = (num_ranges // 2) - (front_segment_size // 2)
        front_end_index = (num_ranges // 2) + (front_segment_size // 2)
        
        front_ranges = [r for r in self.previous_laser_data[front_start_index:front_end_index] if r > 0.0]
        
        if front_ranges:
            min_front_distance = min(front_ranges)
            
            if min_front_distance < obstacle_distance_threshold:
                self.get_logger().info(f'Obstáculo detectado a {min_front_distance:.2f}m. Reduciendo velocidad.')
                linear_velocity = 0.0 # Detenerse o reducir mucho la velocidad
                # Lógica simple para girar: si el obstáculo está más a la izquierda, girar a la derecha, y viceversa.
                # Esto requeriría analizar los rangos de izquierda y derecha por separado.
                # Por ahora, solo reducimos la velocidad.
                # angular_avoidance = ... (lógica más compleja aquí)
        
        return linear_velocity, angular_avoidance
    
    def control_loop(self):
        """Bucle de control principal que se ejecuta periódicamente."""
        cmd = Twist()
        
        # Obtener velocidades de evitación de obstáculos del láser
        laser_linear_v, laser_angular_w = self.read_laser_scan()
        controller_success = self.controller_consecutive_actions_sent > 5 and abs(self.previous_visual_error) == 1.0

        if controller_success: 
            self.controller_successful = True

        if not self.hallway_detected and not self.controller_successful:
            # Si no se detecta el pasillo, avanzar y girar lentamente para buscarlo
            cmd.linear.x = laser_linear_v # Usar la velocidad lineal del láser
            cmd.angular.z = THETA + laser_angular_w # Girar lentamente para buscar, más ajuste del láser
            self.cmd_vel_publisher.publish(cmd)
            self.get_logger().info('Buscando pasillo...')
            self.controller_consecutive_actions_sent = 0
            return
        
        # Calcular el error y la derivada del error
        derivative = (self.visual_error - self.previous_visual_error) * FREQUENCY
        
        # Calcular la señal de control PD
        control_signal = (self.kpSteering * self.visual_error) + (self.kdSteering * derivative)
        
        # Aplicar control PD visual para la velocidad angular y la velocidad lineal del láser
        cmd.linear.x = VCONS + laser_linear_v # Usar la velocidad lineal del láser para evitar obstáculos
        cmd.angular.z = -control_signal  # Girar en función del control PD visual, más ajuste del láser
        
        self.get_logger().info(f"VLineal: {cmd.linear.x:.2f}, Angular: {cmd.angular.z:.2f}, Error Visual: {self.visual_error:.2f}")
        self.controller_consecutive_actions_sent += 1

        self.cmd_vel_publisher.publish(cmd)
        self.previous_visual_error = self.visual_error

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
