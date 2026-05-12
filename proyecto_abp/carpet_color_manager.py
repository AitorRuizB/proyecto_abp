import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import subprocess
import os
import json

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

        # Suscripciones
        self.create_subscription(Odometry, '/robot_0/odom', self.odom_callback, 10)
        self.create_subscription(String, '/robot_0/state', self.state_callback, 10)
        
        self.can_activate_carpets = False 
        self.get_logger().info('Esperando transición DOOR_PASSED para activar alfombras...')

    def state_callback(self, msg):
        # Según tu FSM, tras DOOR_PASSED el estado cambia a NAVIGATING_HALLWAY
        if msg.data == 'NAVIGATING_HALLWAY':
            if not self.can_activate_carpets:
                self.get_logger().info('¡Estado NAVIGATING_HALLWAY detectado! Sensores de alfombra listos.')
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
        """Cambia el color visual del modelo usando el servicio visual_config."""
        
        try:
            # Usar el servicio /world/laberinto_v2_world/visual_config
            # Tipo: gz.msgs.Visual
            # Formato: name: "<model_name>/link/visual", material: { diffuse: { r, g, b, a } }
            
            visual_data = f'''
name: "{model_name}/link/visual"
material {{
  diffuse {{
    r: {r}
    g: {g}
    b: {b}
    a: 1.0
  }}
}}
'''
            
            command = [
                'gz', 'service',
                '-s', '/world/laberinto_v2_world/visual_config',
                '--reqtype', 'gz.msgs.Visual',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--data', visual_data.strip()
            ]
            
            self.get_logger().debug(f'Enviando petición para {model_name}...')
            
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=5
            )
            
            self.get_logger().debug(f'Return code: {result.returncode}')
            if result.stdout.strip():
                self.get_logger().debug(f'Response: {result.stdout.strip()[:200]}')
            if result.stderr.strip():
                self.get_logger().debug(f'Error: {result.stderr.strip()[:200]}')
            
            if result.returncode == 0 or 'true' in result.stdout.lower():
                self.get_logger().info(f'✓ {model_name}: RGB({r:.1f},{g:.1f},{b:.1f})')
                return True
                
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Timeout enviando comando a Gazebo')
        except FileNotFoundError:
            self.get_logger().error('Comando "gz" no encontrado')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        
        return False

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