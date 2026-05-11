import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool, String
import numpy as np
import matplotlib.pyplot as plt
from proyecto_abp.finiteStateMachine import STATE_TOPIC, STATES


SCAN_TOPIC = '/scan'  # Topic del laser
ERROR_TOPIC = '/laser_error' # Topic para publicar el error del laser (distancia al obstáculo más cercano)
OBSTACLE_TOPIC = '/obstacle_detected' # Topic para publicar si se ha detectado un obstáculo cercano (bool)
OBSTACLE_THRESHOLD = 1.0 # Distancia umbral para considerar que hay un obstáculo cercano (en metros)
EPSILON = 0.005 # error de steering minimo para ejercer el controlador
PLOT_VFH_DATA = False # for debug purposes
# LaserPoint representa un punto detectado por el láser con su ángulo y distancia
class LaserPoint:
    def __init__(self, angle, distance):
        self.angle = angle # rad
        self.distance = distance 
    
    def getAngle(self):
        return self.angle
    
    def getDistance(self):
        return self.distance
    

# Vector Field Histogram (VFH) para evitar obstáculos basado en el láser
class Vfh:
    def __init__(self, dynamic_plot=PLOT_VFH_DATA):
        self.histogram = None
        self.laser_data = None # raw data del laser
        self.obstacle_threshold = OBSTACLE_THRESHOLD # init
        self.laser_points = np.empty((0,2)) # Array 2D of LaserPoints
        self.total_points = 0 # int 
        self.previous_direction = None # direccion seleccionada antes [-180º, +180º]
        self.there_is_obstacle = False # bool indicando si hay un obstáculo cercano según el umbral
        self.status = None # flag estado de la FSM
        self.neighbourhood_size = 20 # tamaño de la vecindad en grados para calcular apertura libre
        self.obstacle_probabilities = None # Array 2D of [angle, probability] para cada punto del laser, donde la probabilidad se calcula a partir de la distancia usando una función sigmoide
        self.prob_occupied_threshold = 0.5 # probabilidad minima para considerar un punto como ocupado (obstáculo) 
        self.G = 0.0 # funcion de coste a minimizar (error steering) 
        self.target_gain = 0.95 # ganancia para el término del objetivo (ir recto)
        self.previous_direction_gain = 0.9 # ganancia para el término de dirección previa (evitar cambios bruscos)
        self.goal_direction = 0.0 # dirección objetivo segura (en radianes), por defecto 0 para ir recto

        self.dynamic_plot = dynamic_plot
        if self.dynamic_plot:
            # Habilitar modo interactivo para que el plot no bloquee el programa
            plt.ion()
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111)

    def set_threshold(self, thres, p_thres):
        self.obstacle_threshold = thres
        self.prob_occupied_threshold = p_thres

    def set_gains(self, tG, pdG):
        self.target_gain = tG
        self.previous_direction_gain = pdG

    def set_neighbourhood_size(self, size):
        if(size != self.neighbourhood_size):
            self.neighbourhood_size = size # in degrees

    def set_laser_data(self, laser_data):
        self.laser_data = np.array(laser_data)
        self.total_points = int(len(laser_data))

    def set_status(self, status):
        self.status = status 
    
    def process_laser_data(self):
        if self.laser_data is None or self.total_points == 0:
            return

        N = self.total_points # N puntos detectados

        # 1. Generar array continuo asumiendo el barrido real del sensor.
        # El sensor empieza en +90º (Izquierda), pasa por 0º (Frente),
        # luego -90º (Derecha) y sigue hasta completar los 360º (-270º).
        angles = np.linspace(np.pi/2, -3 * np.pi/2, N, endpoint=False)

        # 2. Normalizar los ángulos al rango [-pi, pi] usando la operación módulo.
        # Esto automáticamente convierte el tramo que baja de -180º a -270º 
        # en sus correspondientes positivos de +180º bajando a +90º.
        angles = (angles + np.pi) % (2 * np.pi) - np.pi

        ranges = self.laser_data

        # 3. Ordenar puntos por ángulo de -pi a pi
        sort_indices = np.argsort(angles, kind='mergesort')
        sorted_angles = angles[sort_indices]
        sorted_ranges = ranges[sort_indices]

        # Actualizamos la variable de la clase. Cada fila es [ángulo, distancia]
        self.laser_points = np.column_stack((sorted_angles, sorted_ranges))

        if self.dynamic_plot:
            # 4. Actualizar el plot
            self.ax.clear()
    
            # Pasar de radianes a grados para una visualización más intuitiva
            sorted_angles_deg = np.degrees(sorted_angles)
            
            # Dibujar los datos usando directamente los arrays de NumPy
            self.ax.bar(sorted_angles_deg, sorted_ranges, width=1.0)
            self.ax.set_xlabel("Ángulo (grados)")
            self.ax.set_ylabel("Distancia estimada (m)")
            self.ax.set_title("Histograma de Visualización del Láser (Vista Frontal)")
            # Aumentar resolución para la vista frontal (-90º a +90º)
            self.ax.set_xlim(-90, 90)
            # Añadir barras verticales entorno a self.neighbourhood_size para visualizar la vecindad de análisis
            self.ax.axvline(x=np.degrees(self.goal_direction), color='r', linestyle='--', label='Dirección Objetivo')
            self.ax.axvline(x=np.degrees(self.goal_direction) - self.neighbourhood_size, color='k', linestyle='--', label='Vecindad Izquierda')
            self.ax.axvline(x=np.degrees(self.goal_direction) + self.neighbourhood_size, color='k', linestyle='--', label='Vecindad Derecha')
            self.ax.legend()
            # Aumentar resolución vertical para ver mejor los obstáculos cercanos
            self.ax.set_ylim(0, 5) # Mostrar hasta 5 metros
            self.ax.grid(True)

            # Redibujar el canvas de la figura de forma eficiente
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def calcular_probabilidades_ocupacion(self):
        """
        Esto convierte distancias cercanas a valores cercanos a 1 (ocupado)
        y distancias lejanas a valores cercanos a 0 (libre), con una transición suave alrededor del umbral.
        La función sigmoide se define como: S(d) = 1 / (1 + exp(-k*(d - d0)))
        donde k controla la pendiente de la transición y d0 es el punto medio (umbral).
        """
        if self.laser_points.size == 0:
            return False  
        k = 10 # pendiente de la función sigmoide
        d0 = self.obstacle_threshold # punto medio en el umbral
        probabilities = 1 / (1 + np.exp(-k * (self.laser_points[:,1] - d0)))
        # invertir para que valores altos de P represneten un obstaculo cercano
        probabilities = 1 - probabilities
        self.obstacle_probabilities = np.column_stack((self.laser_points[:,0], probabilities))

        # Heurística para encontrar la dirección más segura
        if not self.there_is_obstacle and self.status == STATES[0]:
            # Seleccionar en una vecindad self.neighbourhood_size el valor mediano con la probabilidad más baja
            print("Calcula heuristica direccion")

            # 1. Definir el tamaño de la ventana de análisis en número de puntos del láser
            angle_increment_rad = (2 * np.pi) / self.total_points if self.total_points > 0 else 0.1
            window_size_rad = np.radians(self.neighbourhood_size)
            window_size_points = int(window_size_rad / angle_increment_rad) if angle_increment_rad > 0 else 20
            
            # Asegurar que la ventana tenga un tamaño impar para tener un punto central
            if window_size_points % 2 == 0:
                window_size_points += 1

            # 2. Calcular la media móvil de las probabilidades para suavizar y encontrar la región más baja
            probabilities = self.obstacle_probabilities[:, 1]
            rolling_mean_probs = np.convolve(probabilities, np.ones(window_size_points) / window_size_points, mode='valid')

            if rolling_mean_probs.size > 0:
                # 3. Encontrar el índice de la ventana con la media de probabilidad más baja
                min_mean_idx = np.argmin(rolling_mean_probs)
                
                # 4. Obtener los ángulos de esa ventana y calcular su mediana para la selección final, que es más robusta
                start_idx = min_mean_idx
                end_idx = min_mean_idx + window_size_points
                best_window_angles = self.obstacle_probabilities[start_idx:end_idx, 0]
                self.goal_direction = np.median(best_window_angles)
            else:
                # Si no se pudo calcular, ir recto por defecto
                self.goal_direction = 0.0
        else:
            #print("Direccion RECTO")
            # Si hay un obstáculo cercano, el objetivo por defeto es ir recto para sortearlo
            self.goal_direction = 0.0

        if self.dynamic_plot:
            # Plotear las probabilidades de ocupación en un gráfico separado
            plt.figure(2)
            plt.clf()
            plt.plot(np.degrees(self.obstacle_probabilities[:,0]), self.obstacle_probabilities[:,1], marker='o')
            plt.xlabel("Ángulo (grados)")
            plt.ylabel("Probabilidad de Ocupación")
            plt.title("Probabilidades de Ocupación del Láser")
            plt.xlim(-90, 90)
            plt.ylim(0, 1)
            plt.grid(True)
            plt.draw()
            plt.pause(0.001)
        
        return True

    def compute_cost_function(self):# target en radianes, por defecto 0 para ir recto
        if self.laser_points.size == 0:
            self.G = 0.0
            return self.G

        # comprobar si hay puntos bajo del umbral para detectar obstáculos dentro de la vecindad
        self.there_is_obstacle = bool(np.any((self.laser_points[:,1] < self.obstacle_threshold) & (np.abs(self.laser_points[:,0] - self.goal_direction) < np.radians(self.neighbourhood_size))))
        if self.there_is_obstacle:
            print("Hay un obstaculo")
        # inicializar la direccion seleccionada al objetivo
        selected_direction = self.goal_direction
        # Calcular función de costo basada en probabilidad de ocupacion 
        if self.calcular_probabilidades_ocupacion():
            # 1. Encontrar índices de puntos con baja probabilidad de ocupación 
            low_prob_indices = np.where(self.obstacle_probabilities[:, 1] < self.prob_occupied_threshold)[0]

            if low_prob_indices.size > 0:
                # 2. Agrupar índices consecutivos en "huecos" o "valles"
                gaps = np.split(low_prob_indices, np.where(np.diff(low_prob_indices) != 1)[0] + 1)

                # 3. Filtrar los huecos que son suficientemente anchos
                angle_increment_rad = (2 * np.pi) / self.total_points if self.total_points > 0 else 0.1
                min_gap_width_rad = np.radians(2 * self.neighbourhood_size)
                min_gap_width_points = int(min_gap_width_rad / angle_increment_rad) if angle_increment_rad > 0 else 20

                valid_gaps = [g for g in gaps if len(g) >= min_gap_width_points]

                if valid_gaps:
                    # Encontrar el mejor hueco (el más cercano a la dirección objetivo)
                    min_angle_diff = float('inf')

                    for gap in valid_gaps:
                        gap_angles = self.obstacle_probabilities[gap, 0]
                        median_angle = np.median(gap_angles)
                        # Diferencia angular robusta (maneja el salto de -pi a +pi)
                        angle_diff = np.arctan2(np.sin(median_angle - self.goal_direction), np.cos(median_angle - self.goal_direction))
                        if abs(angle_diff) < min_angle_diff:
                            min_angle_diff = abs(angle_diff)
                            selected_direction = median_angle # Escoger el ángulo mediano como dirección

        # Inicializar la dirección previa si es la primera vez
        if self.previous_direction is None:
            self.previous_direction = selected_direction  # Por defecto, ir hacia el objetivo si no hay obstáculos o rutas seguras

        if abs(selected_direction) < EPSILON:
            #print("Error menor a EPSILON -> CONTROLLER OFF ############################")
            selected_direction = 0.0    
        #print(f"Selected direction: {selected_direction} ----------------------------")
        
        # Calcular la función de coste para un giro suave
        target_error = (selected_direction - self.goal_direction)
        continuity_error = (selected_direction - self.previous_direction)

        self.G = self.target_gain * target_error + self.previous_direction_gain * continuity_error
        #print(f"G cost function value : {self.G} <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
        self.previous_direction = selected_direction # Actualizar para la siguiente iteración
        return self.G

    def obstacle_detected(self):
        return self.there_is_obstacle

        
# ROS node to process laser scan data
class LaserProcessor(Node):
    def __init__(self, robot_id='/robot_0'):
        super().__init__('laser_processor')
        self.vfh = Vfh() # A LaserProcessor uses a Vfh
        self.robot_id = robot_id
        # Subscribe to laser scan
        self.laser_subscription = self.create_subscription(
            LaserScan,
            self.robot_id + SCAN_TOPIC,
            self.laser_callback,
            10
        )
        # subscribe to fsm node
        self.fsm_st = self.create_subscription(String, self.robot_id + STATE_TOPIC, self.fsm_callback, 10)

        # publisher
        self.laser_error_publisher = self.create_publisher(Float32, self.robot_id + ERROR_TOPIC, 10)
        self.obstacle_publisher = self.create_publisher(Bool, self.robot_id + OBSTACLE_TOPIC, 10)

        
        
        self.get_logger().info(f'Laser processor initialized for {self.robot_id}')


    def fsm_callback(self, msg):
        if msg.data in STATES:
            self.fsm_st = msg.data
    
    def laser_callback(self, msg: LaserScan):
        if not msg.ranges:
            self.get_logger().warn("Received empty laser scan.")
            return

        # procesado de datos del laser para evitar obstáculos
        self.vfh.set_laser_data(msg.ranges)
        self.vfh.process_laser_data()

        # check the current state of the fsm to assign parameters to the VFH
        if self.fsm_st == STATES[0] or self.fsm_st == STATES[1]: # wander & approach -> assign high threshold and min prob
            self.vfh.set_status(self.fsm_st)
            self.vfh.set_threshold(OBSTACLE_THRESHOLD, 0.5)
            self.vfh.set_gains(0.8,0.6)

        elif self.fsm_st == STATES[2]:# nav Hallway -> low thres and lower prob
            self.vfh.set_status(self.fsm_st)
            self.vfh.set_threshold(OBSTACLE_THRESHOLD + 0.15, 0.5)
            self.vfh.set_gains(0.99,0.8)

        else: # default values
            self.vfh.set_status(None)
            self.vfh.set_threshold(OBSTACLE_THRESHOLD, 0.5)
            self.vfh.set_gains(0.0,0.0)
            

        Float32_msg = Float32()
        Bool_msg = Bool()

        Float32_msg.data = self.vfh.compute_cost_function()
        Bool_msg.data = self.vfh.obstacle_detected()

        # publish the topics
        self.laser_error_publisher.publish(Float32_msg)
        self.obstacle_publisher.publish(Bool_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserProcessor()

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