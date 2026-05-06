import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np

class Recon:
    """
    ## Clase a bajo nivel ##
    Encargada de todos los algoritmos de visión y segmentación (OpenCV).
    """
    def __init__(self):
        # Rango de color morado en HSV
        self.lower_purple = np.array([125, 50, 50])
        self.upper_purple = np.array([155, 255, 255])
        self.frame = None

    def set_frame(self, frame):
        """Almacena el frame actual para su procesamiento."""
        self.frame = frame

    def detect_color(self):
        """Aplica los filtros de OpenCV y devuelve la máscara segmentada."""
        if self.frame is None:
            return None
        
        # 1. Aplicar Gaussian Blur para reducir el ruido
        blurred = cv2.GaussianBlur(self.frame, (9, 9), 0)
        
        # 2. Cambiar al espacio de color HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # 3. Umbralización para obtener la máscara binaria (Color Morado)
        mask = cv2.inRange(hsv, self.lower_purple, self.upper_purple)
        
        return mask
    
    def compute_mask(self, mask):
        """Logica de procesamiento de la imagen segmentada"""
        #TODO
        return mask

class CameraProcessing(Node):
    """
    ## Clase a alto nivel ##
    Administra el nodo y la suscripción a los topics de ROS2.
    """
    def __init__(self):
        super().__init__('camera_processor')
        
        self.recon = Recon()
        self.bridge = cv_bridge.CvBridge()
        self.image_subscription = None
        
        # Variable para almacenar el último frame procesado de forma segura
        self.latest_mask = None
        
        # 1. CREAMOS LA VENTANA UNA SOLA VEZ AQUÍ
        self.window_name = "Máscara de Color Morado"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        # Empleamos el setter en el constructor con el topic por defecto
        self.setCameraTopic('/robot_0/camera/image')
        
        # 2. CREAMOS UN TIMER PARA LA INTERFAZ GRÁFICA (~30 FPS)
        # Esto separa la recepción de imágenes del dibujado de OpenCV
        timer_period = 0.033  # segundos (1/30)
        self.display_timer = self.create_timer(timer_period, self.display_callback)
        
        self.get_logger().info('Nodo CameraProcessing inicializado y listo.')

    def setCameraTopic(self, topic: str):
        """
        Setter para el topic de la cámara. 
        Crea (o recrea) la suscripción al nuevo topic.
        """
        self.camera_topic = topic
        
        if self.image_subscription is not None:
            self.destroy_subscription(self.image_subscription)
            
        self.image_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Suscrito a la cámara en el topic: "{self.camera_topic}"')

    def image_callback(self, msg: Image):
        """Se ejecuta cada vez que llega una imagen. Solo procesa, NO dibuja."""
        try:
            # Convertir mensaje a OpenCV y procesar
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.recon.set_frame(cv_image)
            
            # Guardamos la máscara en la variable que lee el Timer
            self.latest_mask = self.recon.detect_color()

            self.result = self.compute_mask(self.latest_mask)
                
        except Exception as e:
            self.get_logger().error(f'Error en image_callback: {e}')

    def display_callback(self):
        """Se ejecuta constantemente dictado por el timer. Se encarga del refresco gráfico."""
        if self.latest_mask is not None:
            # 3. MOSTRAMOS LA IMAGEN EN LA VENTANA YA CREADA
            cv2.imshow(self.window_name, self.latest_mask)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = CameraProcessing()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Interrupción detectada. Cerrando nodo...')
    finally:
        # Limpieza de ventanas de OpenCV
        cv2.destroyAllWindows()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()