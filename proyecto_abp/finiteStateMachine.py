# class to handle and publish the states of the robot
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import shutil
from pathlib import Path

FREQUENCY = 25.0  # Frecuencia de control del MRS en Hz

POSSIBLE_GOALS = ['green', 'yellow', 'red', 'blue']
STATES = ['WANDER', 'APPROACH_DOOR','NAVIGATING_HALLWAY','APPROACH_TARGET', 'MERGE_SLAM', 'NAV2TARGET']
TRANSITIONS = ['HALLWAY_FOUND', 'DOOR_PASSED','TARGET_APPROACH','TARGET_LOCATED', 'GLOBAL_MAP_READY']

GOAL_TOPIC = '/goal' # String indicando el objetivo a buscar
TRANSITION_TOPIC = '/transition'  # Topic para publicar transiciones de estado
STATE_TOPIC = '/state'  # Topic para publicar estados del robot

class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__('finite_state_machine')
        
        # Obtener el namespace dinámicamente
        self.robot_id = self.get_namespace()
        if self.robot_id == '/':
            self.robot_id = '/robot_0'  # Default si no hay namespace
        self.state_publisher = self.create_publisher(String, self.robot_id + STATE_TOPIC, 10)
        self.current_state = 'WANDER'  # Estado inicial

        # subcribe to transitions topics to update the state
        self.create_subscription(String, self.robot_id + TRANSITION_TOPIC, self.transition_callback, 10)
        self.goal_publisher = self.create_publisher(String, self.robot_id + GOAL_TOPIC, 10)
        
        # Leer el parámetro goal desde la configuración de ROS
        self.declare_parameter('goal', 'green')
        self.current_goal = self.get_parameter('goal').value
        
        # Leer el número de robots (por defecto 2)
        self.declare_parameter('num_robots', 2)
        self.num_robots = self.get_parameter('num_robots').value

        # Create a timer to periodically publish the state at 12Hz
        self.create_timer(1.0 / FREQUENCY, self.periodic_publish)
        
        # Flag para rastrear si ya hemos guardado el mapa en esta sesión
        self.slam_saved = False
        self.slam_launched = False
        
        # Obtener la ruta al paquete para guardar mapas
        try:
            from ament_index_python.packages import get_package_share_directory
            self.pkg_path = get_package_share_directory('proyecto_abp')
            self.config_path = os.path.join(self.pkg_path, 'config')
        except:
            self.pkg_path = os.path.expanduser('~/multirobots_ros/src/proyecto_abp')
            self.config_path = os.path.join(self.pkg_path, 'config')

        self.get_logger().info(f"Finite State Machine for {self.robot_id} initialized with goal: {self.current_goal}.")
        self.get_logger().info(f"Número de robots: {self.num_robots}")
        self.get_logger().info(f"Mapa se guardará en: {self.config_path}")
        
        # Lanzar SLAM automáticamente al iniciar (solo desde el primer FSM)
        if self.robot_id == '/robot_0':
            self.get_logger().info("🎯 Iniciando SLAM automáticamente...")
            self._launch_slam()
        else:
            self.get_logger().info(f"FSM de {self.robot_id} listo (SLAM ya lanzado desde robot_0)")

    def periodic_publish(self):
        """Called by a timer to periodically publish the current state and goal."""
        self.publish_state(self.get_current_state())
        self.publish_goal()

    def publish_state(self, state):
        is_new_state = self.current_state != state
        self.current_state = state
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)
        if is_new_state:
            self.get_logger().info(f'State changed to: {state}')

    def get_current_state(self):
        return self.current_state  

    def publish_goal(self):
        """Publica el objetivo actual en el topic /goal."""
        goal_msg = String()
        goal_msg.data = self.current_goal
        self.goal_publisher.publish(goal_msg)

    def save_and_finalize_slam(self):
        """Guarda los mapas de SLAM, cierra los nodos SLAM y prepara para Nav2."""
        try:
            # Obtener el robot ID del namespace
            robot_namespace = self.robot_id.lstrip('/')
            
            self.get_logger().info(f"📍 Guardando mapas de SLAM para {robot_namespace}...")
            
            # 1. Guardar los mapas usando el servicio de SLAM Toolbox
            map_name = os.path.join(self.config_path, 'mapa')
            self._call_slam_save_service(robot_namespace, map_name)
            
            # 2. Esperar un poco para asegurar que los mapas se escribieron
            import time
            time.sleep(1)
            
            # 3. Verificar que los mapas existen
            pgm_file = f"{map_name}.pgm"
            yaml_file = f"{map_name}.yaml"
            
            if os.path.exists(pgm_file) and os.path.exists(yaml_file):
                self.get_logger().info(f"✅ Mapas guardados correctamente:")
                self.get_logger().info(f"   - {pgm_file}")
                self.get_logger().info(f"   - {yaml_file}")
            else:
                self.get_logger().warn(f"⚠️  Mapas no encontrados en la ubicación esperada")
                self.get_logger().warn(f"   Buscando en /tmp y rutas alternativas...")
                self._search_and_copy_maps(robot_namespace)
            
            # 4. Cerrar los nodos SLAM
            self.get_logger().info("🛑 Cerrando nodos de SLAM...")
            self._stop_slam_nodes(robot_namespace)
            
            self.get_logger().info("✅ SLAM finalizado. Listo para iniciar Nav2")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error al guardar/cerrar SLAM: {e}")
    
    def _call_slam_save_service(self, robot_namespace, map_name):
        """Llama al servicio de SLAM Toolbox para guardar el mapa."""
        try:
            # Usar ros2 service call para mayor compatibilidad
            service_name = f"/{robot_namespace}/slam_toolbox/save_map"
            
            # Comando: ros2 service call /robot_0/slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: mapa"
            cmd = [
                'ros2', 'service', 'call',
                service_name,
                'slam_toolbox/srv/SaveMap',
                f'{{name: "{map_name}"}}'
            ]
            
            self.get_logger().debug(f"Ejecutando: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                self.get_logger().info(f"✓ Servicio save_map respondió correctamente")
            else:
                self.get_logger().warn(f"Servicio devolvió código {result.returncode}: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            self.get_logger().error("Timeout al llamar al servicio save_map")
        except Exception as e:
            self.get_logger().error(f"Error llamando al servicio: {e}")
    
    def _search_and_copy_maps(self, robot_namespace):
        """Busca los mapas en ubicaciones alternativas y los copia."""
        possible_locations = [
            '/tmp',
            os.path.expanduser('~/.ros'),
            '/tmp/maps',
            os.path.join(self.config_path, '..', 'maps')
        ]
        
        for location in possible_locations:
            if os.path.exists(location):
                pgm_files = list(Path(location).glob('*.pgm'))
                yaml_files = list(Path(location).glob('*.yaml'))
                
                if pgm_files and yaml_files:
                    # Copiar el archivo más reciente
                    latest_pgm = max(pgm_files, key=os.path.getctime)
                    latest_yaml = max(yaml_files, key=os.path.getctime)
                    
                    self.get_logger().info(f"📋 Encontrados mapas en {location}")
                    self.get_logger().info(f"   Copiando: {latest_pgm.name}")
                    
                    shutil.copy(str(latest_pgm), os.path.join(self.config_path, 'mapa.pgm'))
                    shutil.copy(str(latest_yaml), os.path.join(self.config_path, 'mapa.yaml'))
                    
                    self.get_logger().info(f"✓ Mapas copiados a {self.config_path}")
                    return
        
        self.get_logger().error("❌ No se encontraron mapas en ubicaciones alternativas")
    
    def _launch_slam(self):
        """Lanza el nodo de SLAM automáticamente."""
        try:
            # Usar el número de robots del parámetro
            num_robots = str(self.num_robots)
            
            # Intentar lanzar start_slam en background
            cmd = ['ros2', 'run', 'proyecto_abp', 'start_slam', num_robots]
            self.get_logger().info(f"🚀 Lanzando SLAM para {num_robots} robots: {' '.join(cmd)}")
            
            # Lanzar en background sin esperar
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            self.slam_launched = True
            self.get_logger().info("✅ SLAM iniciado en background")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error lanzando SLAM: {e}")
    
    def _stop_slam_nodes(self, robot_namespace):
        """Detiene los nodos de SLAM usando ros2 node kill."""
        nodes_to_kill = [
            f"/{robot_namespace}/slam_toolbox",
            f"/{robot_namespace}/lifecycle_manager_slam"
        ]
        
        for node in nodes_to_kill:
            try:
                cmd = ['ros2', 'node', 'kill', node]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
                
                if result.returncode == 0 or 'not found' in result.stderr.lower():
                    self.get_logger().info(f"✓ Nodo {node} cerrado")
                else:
                    self.get_logger().warn(f"No se pudo cerrar {node}: {result.stderr}")
                    
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"Timeout cerrando {node}")
            except Exception as e:
                self.get_logger().warn(f"Error cerrando {node}: {e}")

    def transition_callback(self, msg):
        transition = msg.data
        if transition in TRANSITIONS:
            
            if transition == TRANSITIONS[0] and self.get_current_state() != STATES[1]:
                self.publish_state(STATES[1])

            elif transition == TRANSITIONS[1] and self.get_current_state() != STATES[2]:
                self.publish_state(STATES[2])
            
            elif transition == TRANSITIONS[2] and self.get_current_state() != STATES[3]:
                self.publish_state(STATES[3])

            elif transition == TRANSITIONS[3] and self.get_current_state() != STATES[4]:
                self.publish_state(STATES[4])

            elif transition == TRANSITIONS[4] and self.get_current_state() != STATES[5]:
                # TRANSICIÓN A MERGE_SLAM: Guardar mapas y cerrar SLAM
                if not self.slam_saved:
                    self.get_logger().info("🗺️  Transición a MERGE_SLAM: Guardando mapas...")
                    self.save_and_finalize_slam()
                    self.slam_saved = True
                self.publish_state(STATES[5])

def main(args=None):
    rclpy.init(args=args)
    fsm_node = FiniteStateMachine()
    # The node now handles periodic state publishing internally via a timer.
    try:
        rclpy.spin(fsm_node)
    except KeyboardInterrupt:
        pass
    finally:
        fsm_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
