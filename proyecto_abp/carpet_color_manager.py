import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, ColorRGBA
from proyecto_abp.finiteStateMachine import STATES

class MultiCarpetManager(Node):
    def __init__(self):
        super().__init__('carpet_manager')
        
        # Coordenadas exactas basadas en tu SDF (Centro +- 0.5m)
        self.carpets = {
            'purple_carpet_north': {'x': (-0.5, 0.5), 'y': (4.02, 5.02), 'active': False},
            'purple_carpet_south': {'x': (-0.5, 0.5), 'y': (-5.02, -4.02), 'active': False},
            'purple_carpet_east':  {'x': (4.02, 5.02), 'y': (-0.5, 0.5), 'active': False},
            'purple_carpet_west':  {'x': (-5.02, -4.02), 'y': (0.5, 1.5), 'active': False}
        }

        # Crear publicadores para cada alfombra (Tópico: /model/<nombre_alfombra>/color)
        self.color_pubs = {}
        for carpet_name in self.carpets.keys():
            topic_name = f'/model/{carpet_name}/color'
            self.color_pubs[carpet_name] = self.create_publisher(ColorRGBA, topic_name, 10)

        # Suscripciones
        self.create_subscription(Odometry, '/robot_0/odom', self.odom_callback, 10)
        self.create_subscription(String, '/robot_0/state', self.state_callback, 10)
        
        self.can_activate_carpets = False 
        self.get_logger().info('Esperando transición DOOR_PASSED para activar alfombras...')

    def state_callback(self, msg):
        if msg.data == STATES[2]:  # Nav hallway
            if not self.can_activate_carpets:
                self.get_logger().info('Sensores de alfombra listos.')
            self.can_activate_carpets = True

    def odom_callback(self, msg):
        if not self.can_activate_carpets:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        for name, bounds in self.carpets.items():
            is_inside = (bounds['x'][0] <= x <= bounds['x'][1] and 
                         bounds['y'][0] <= y <= bounds['y'][1])

            if is_inside and not bounds['active']:
                self.get_logger().info(f'¡Robot sobre {name}! Cambiando a VERDE')
                self.set_carpet_color(name, 0.0, 1.0, 0.0) # Verde
                bounds['active'] = True
            elif not is_inside and bounds['active']:
                self.get_logger().info(f'Robot salió de {name}. Volviendo a PÚRPURA')
                self.set_carpet_color(name, 0.5, 0.0, 0.5) # Púrpura original
                bounds['active'] = False

    def set_carpet_color(self, model_name, r, g, b):
        """Publica el color en el tópico correspondiente a la alfombra."""
        msg = ColorRGBA()
        msg.r = float(r)
        msg.g = float(g)
        msg.b = float(b)
        msg.a = 1.0 # Opacidad completa
        
        # Publicar el mensaje
        self.color_pubs[model_name].publish(msg)
        self.get_logger().debug(f'Publicado color RGB({r}, {g}, {b}) en /model/{model_name}/color')

def main(args=None):
    rclpy.init(args=args)
    node = MultiCarpetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()