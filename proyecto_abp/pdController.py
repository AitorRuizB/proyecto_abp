import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String
import csv
import os
from datetime import datetime
from cameraProcessor import ERROR_TOPIC as VISUAL_ERROR_TOPIC, HALLWAY_TOPIC
from laserProcessor import ERROR_TOPIC as LASER_ERROR_TOPIC, OBSTACLE_TOPIC
from finiteStateMachine import STATES, TRANSITIONS, TRANSITION_TOPIC, STATE_TOPIC, FREQUENCY

SCAN_TOPIC = '/scan'  # Topic del laser
VELOCITY_TOPIC = '/cmd_vel'  # Topic para publicar comandos de velocidad
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
            PDControllerParams(kp=1.0, kd=0.1, sensor_type='laser', is_steering=False),
            PDControllerParams(kp=1.0, kd=0.2, sensor_type='laser', is_steering=True)
        ]

        self.previous_visual_error = 0.0
        self.previous_laser_error = 0.0
        self.controller_consecutive_actions_sent = 0 # detectar si ha conseguido minimizar el error visual
        self.fsm_st = STATES[0] # estado inicial de la FSM

        self.robot_id = robot_id # id is a namespace like '/robot_1'
        
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
        self.visual_error = None
        self.laser_error = None
        self.there_is_obstacle = False
        self.visual_controller_success = False        # Timer para el bucle de control principal
        self.timer = self.create_timer(1.0 / FREQUENCY, self.control_loop) # Ejecutar el bucle de control a 10 Hz
        
        # --- INICIO: Configuración para guardado en CSV ---
        # Crear un nombre de archivo único para el log
        robot_name = self.robot_id.strip('/') # Eliminar la barra inicial para el nombre de archivo
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Directorio para guardar los CSV
        csv_dir = "csv"
        os.makedirs(csv_dir, exist_ok=True)
        csv_filepath = os.path.join(csv_dir, f"pd_controller_log_{robot_name}_{timestamp_str}.csv")
        
        # Abrir el archivo y crear el writer
        self.csv_file = open(csv_filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Escribir la cabecera
        self.csv_writer.writerow(['timestamp', 'visual_error', 'laser_error', 'control_law', 'fsm_state'])
        self.get_logger().info(f"Guardando datos de control en: {csv_filepath}")
        # --- FIN: Configuración para guardado en CSV ---
        
        self.get_logger().info(f'PDController para {self.robot_id} inicializado.')

    def visual_error_callback(self, msg):
        """Callback para el error visual de la cámara."""
        self.visual_error = msg.data

    def laser_error_callback(self, msg):
        """Callback para el error del láser."""
        self.laser_error = msg.data
        if self.visual_controller_success:
            # reset error as we switch heuristic in the PD laser controller
            self.laser_error = 0.0 
            self.previous_laser_error = 0.0
            self.visual_controller_success = False


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

    def csv_data_saving(self, timestamp, visual_error, laser_error, control_law, fsm_state):
        """Guarda una fila de datos en el archivo CSV."""
        self.csv_writer.writerow([timestamp, visual_error, laser_error, control_law, fsm_state])
    
    
    def control_loop(self):
        """Bucle de control principal que se ejecuta periódicamente."""
        if self.laser_error is None or self.visual_error is None:
            return
        
        cmd = Twist()
        cmd.linear.x = VCONS  # Usar la velocidad lineal del láser para evitar obstáculos
        
        # flag para detectar que el controlador visual llevo al robot por la puerta
        self.visual_controller_success = self.controller_consecutive_actions_sent > 50 and abs(self.previous_visual_error) == 1.0 and self.hallway_detected

        if self.visual_controller_success:
            self.transition_publisher.publish(String(data=TRANSITIONS[0])) # cambio de estado
            
        # Estado de wander & search: usar ley de control del Laser
        if self.fsm_st == STATES[0]: # WANDER
            if not self.hallway_detected: # Searching hallway
                # Controlador proporcional de velocidad lineal
                cmd.linear.x = VCONS * self.laserPD_gains[0].getKp() # controlador porporcional de velocidad lineal
                # Controlador PD para steering
                control_law = (self.laserPD_gains[1].getKp() * self.laser_error) + (self.laserPD_gains[1].getKd() * (self.laser_error - self.previous_laser_error) * FREQUENCY)
                self.get_logger().info('Buscando pasillo...')
                self.controller_consecutive_actions_sent = 0

            else: # Approach hallway
                # Calcular el error y la derivada del error
                derivative = (self.visual_error - self.previous_visual_error) * FREQUENCY
                # Calcular la señal de control PD
                control_law = (self.visualPD_gains.getKp() * self.visual_error) + (self.visualPD_gains.getKd() * derivative)
                self.get_logger().info('Aproximando puerta con Visual based PD Controller...')

        elif self.fsm_st == STATES[1]: # Aprox puerta
            # Controlador proporcional de velocidad lineal
            cmd.linear.x = VCONS * self.laserPD_gains[0].getKp() # controlador porporcional de velocidad lineal
            # Controlador PD para steering
            control_law = (self.laserPD_gains[1].getKp() * self.laser_error) + (self.laserPD_gains[1].getKd() * (self.laser_error - self.previous_laser_error) * FREQUENCY)
            self.get_logger().info('Aproximando puerta con Laser based PD Controller...')
        
        # Estado navegacion por el pasillo -> PD laser based control con nuevas ganancias y umbrales
        elif self.fsm_st == STATES[2]: # NAVIGATING_HALLWAY
            # Controlador proporcional de velocidad lineal
            cmd.linear.x = VCONS * self.laserPD_gains[0].getKp() # controlador porporcional de velocidad lineal
            # Controlador PD para steering
            control_law = (self.laserPD_gains[1].getKp() * self.laser_error) + (self.laserPD_gains[1].getKd() * (self.laser_error - self.previous_laser_error) * FREQUENCY)
            self.get_logger().info('Navegando pasillo...')

        # Asignar steering
        cmd.angular.z = -control_law

        # --- INICIO: Guardado de datos en CSV ---
        current_time = self.get_clock().now()
        timestamp_sec = current_time.nanoseconds / 1e9
        self.csv_data_saving(timestamp_sec, self.visual_error, self.laser_error, control_law, self.fsm_st)
        # --- FIN: Guardado de datos en CSV ---

        #self.get_logger().info(f"VLineal: {cmd.linear.x:.2f}, Angular: {cmd.angular.z:.2f}, Error Visual: {self.visual_error:.2f}, Error Laser: {self.laser_error:.2f}, Obstacle?: {self.there_is_obstacle}")
        self.controller_consecutive_actions_sent += 1

        self.cmd_vel_publisher.publish(cmd)
        # actualizar error de los sensores
        self.previous_visual_error = self.visual_error
        self.previous_laser_error = self.laser_error

    def destroy_node(self):
        """Limpia recursos, como cerrar el archivo CSV, antes de que el nodo se destruya."""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.get_logger().info("Cerrando archivo CSV.")
            self.csv_file.close()
        super().destroy_node()

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
