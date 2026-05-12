import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import os

class MultiCarpetManager(Node):
    def __init__(self):
        super().__init__('carpet_manager')
        
        # 1. Configuración de las 4 alfombras (Ajusta estas coordenadas a tu SDF)
        self.carpets = {
            'purple_carpet_north': {
                'x_range': (-0.5, 0.5), 
                'y_range': (4.0239, 5.0239), # Centro en 4.5239 +- 0.5
                'active': False
            },
            'purple_carpet_south': {
                'x_range': (-0.5, 0.5), 
                'y_range': (-5.0239, -4.0239), # Centro en -4.5239 +- 0.5
                'active': False
            },
            'purple_carpet_east': {
                'x_range': (4.0239, 5.0239), # Centro en 4.5239 +- 0.5
                'y_range': (-0.5, 0.5),
                'active': False
            },
            'purple_carpet_west': {
                'x_range': (-5.0239, -4.0239), # Centro en -4.5239 +- 0.5
                'y_range': (0.5, 1.5),
                'active': False
            }
        }

        # 2. Suscripciones
        self.create_subscription(Odometry, '/robot_0/odom', self.odom_callback, 10)
        self.create_subscription(String, '/robot_0/state', self.state_callback, 10)
        
        self.can_activate_carpets = False # Se activa tras pasar la puerta
        self.get_logger().info('Gestor de alfombras multizona iniciado. Esperando transición DOOR_PASSED...')

    def state_callback(self, msg):
        # Según tu finiteStateMachine.py, tras DOOR_PASSED el estado es NAVIGATING_HALLWAY
        if msg.data == 'NAVIGATING_HALLWAY':
            if not self.can_activate_carpets:
                self.get_logger().info('¡Puerta detectada! Activando sensores de alfombra.')
            self.can_activate_carpets = True

    def odom_callback(self, msg):
        if not self.can_activate_carpets:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        for name, data in self.carpets.items():
            is_inside = (data['x_range'][0] <= x <= data['x_range'][1] and 
                         data['y_range'][0] <= y <= data['y_range'][1])

            if is_inside and not data['active']:
                self.get_logger().info(f'¡Robot sobre {name}! Cambiando a VERDE')
                self.set_carpet_color(name, 0.0, 1.0, 0.0)
                data['active'] = True
            elif not is_inside and data['active']:
                # Volvemos a gris al salir
                self.set_carpet_color(name, 0.5, 0.5, 0.5)
                data['active'] = False

    def set_carpet_color(self, model_name, r, g, b):
        # Asumimos que en el SDF los links/visuals se llaman link_alfombra/visual_alfombra
        path = f"{model_name}/link/visual_alfombra"
        cmd = f"gz service -s /world/default/visual_config --reqtype gz.msgs.Visual --reptype gz.msgs.Boolean --timeout 300 --data 'name: \"{path}\", material: {{ diffuse: {{ r: {r}, g: {g}, b: {b}, a: 1 }} }}'"
        os.system(cmd)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MultiCarpetManager())
    rclpy.shutdown()