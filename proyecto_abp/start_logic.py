#!/usr/bin/env python3
import sys
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node

class ProcessorsOrchestrator(Node):
    def __init__(self, num_robots):
        super().__init__('processors_orchestrator')
        self.num_robots = num_robots
        self.processes = []
        self.get_logger().info(f"Iniciando procesadores y controladores para {num_robots} robots...")

        # Lanzamos los subprocesos para cada robot
        for i in range(self.num_robots):
            robot_name = f'robot_{i}'
            self.launch_robot_nodes(robot_name)

    def launch_subprocesses(self, cmd, description):
        """Ejecuta un comando ros2 run en un hilo independiente para no bloquear el nodo"""
        def target():
            try:
                # stdout y stderr directos a la pantalla (output='screen')
                process = subprocess.Popen(cmd)
                self.processes.append(process)
                process.wait()
            except Exception as e:
                self.get_logger().error(f"Error en {description}: {str(e)}")

        thread = threading.Thread(target=target, daemon=True)
        thread.start()

    def launch_robot_nodes(self, robot_name):
        self.get_logger().info(f"[{robot_name}] Levantando cámara, láser y controlador PD...")

        # 1. CAMERA PROCESSOR (ros2 run proyecto_abp camera_processor ...)
        cmd_camera = [
            'ros2', 'run', 'proyecto_abp', 'camera_processor',
            '--ros-args',
            '-r', f'__node:=camera_processor_{robot_name}',
            '-r', f'__ns:=/{robot_name}'
        ]
        self.launch_subprocesses(cmd_camera, f"camera_processor_{robot_name}")

        # 2. LASER PROCESSOR (ros2 run proyecto_abp laser_processor ...)
        cmd_laser = [
            'ros2', 'run', 'proyecto_abp', 'laser_processor',
            '--ros-args',
            '-r', f'__node:=laser_processor_{robot_name}',
            '-r', f'__ns:=/{robot_name}'
        ]
        self.launch_subprocesses(cmd_laser, f"laser_processor_{robot_name}")

        # 3. PD CONTROLLER (ros2 run proyecto_abp pd_controller ...)
        cmd_pd = [
            'ros2', 'run', 'proyecto_abp', 'pd_controller',
            '--ros-args',
            '-r', f'__node:=pd_controller_{robot_name}',
            '-r', f'__ns:=/{robot_name}'
        ]
        self.launch_subprocesses(cmd_pd, f"pd_controller_{robot_name}")

    def shutdown_all(self):
        """Mata limpiamente todos los subprocesos abiertos al cerrar el nodo"""
        self.get_logger().info("Cerrando todos los procesadores de forma ordenada...")
        for p in self.processes:
            if p.poll() is None:  # Si sigue vivo, se le mata
                p.terminate()
        # Espera de cortesía para que liberen los puertos y topics de ROS 2
        time.sleep(0.5)


def main(args=None):
    # Verificación de argumentos por terminal
    if len(sys.argv) < 2:
        print("Uso: ros2 run proyecto_abp start_processors <num_robots>")
        return

    num_robots = int(sys.argv[1])

    rclpy.init(args=args)
    node = ProcessorsOrchestrator(num_robots)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Al pulsar Ctrl+C, matamos todos los ejecutables antes de salir
        node.shutdown_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()