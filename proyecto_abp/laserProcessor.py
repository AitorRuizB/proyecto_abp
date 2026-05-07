import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt



SCAN_TOPIC = '/scan'  # Topic del laser

# LaserPoint representa un punto detectado por el láser con su ángulo y distancia
class LaserPoint:
    def __init__(self, angle, distance):
        self.angle = angle
        self.distance = distance
    
    def getAngle(self):
        return self.angle
    
    def getDistance(self):
        return self.distance
    

# Vector Field Histogram (VFH) para evitar obstáculos basado en el láser
class Vfh:
    def __init__(self):
        self.histogram = None
        self.laser_data = None # raw data del laser

        self.laser_points = np.empty((0,2)) # Array 2D of LaserPoints
        self.total_points = 0 # int 
        # Habilitar modo interactivo para que el plot no bloquee el programa
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)

    def set_laser_data(self, laser_data):
        self.laser_data = np.array(laser_data)
        self.total_points = int(len(laser_data))
    
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

        # 4. Actualizar el plot
        self.ax.clear()
    
        # Pasar de radianes a grados para una visualización más intuitiva
        sorted_angles_deg = np.degrees(sorted_angles)
        
        # Dibujar los datos usando directamente los arrays de NumPy
        self.ax.bar(sorted_angles_deg, sorted_ranges, width=1.0)
        self.ax.set_xlabel("Ángulo (grados)")
        self.ax.set_ylabel("Distancia estimada (m)")
        self.ax.set_title("Histograma de Visualización del Láser")
        self.ax.set_xlim(sorted_angles_deg.min(), sorted_angles_deg.max())
        self.ax.grid(True)

        # Redibujar el canvas de la figura de forma eficiente
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

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
        
        
        self.get_logger().info(f'Laser processor initialized for {self.robot_id}')

    def laser_callback(self, msg: LaserScan):
        if not msg.ranges:
            self.get_logger().warn("Received empty laser scan.")
            return

        # procesado de datos del laser para evitar obstáculos
        self.vfh.set_laser_data(msg.ranges)
        self.vfh.process_laser_data()

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