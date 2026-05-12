#!/usr/bin/env python3
import sys
from launch import LaunchService, LaunchDescription
from launch_ros.actions import Node

def main():
    if len(sys.argv) < 2:
        print("Uso: ros2 run proyecto_abp start_logic <num_robots>")
        return

    num_robots = int(sys.argv[1])
    nodes_to_launch = []

    # 1. Instanciar los "cerebros" individuales por cada robot
    for i in range(num_robots):
        robot_name = f'robot_{i}'

        # Instancia de la Máquina de Estados individual
        nodes_to_launch.append(
            Node(
                package='proyecto_abp',
                executable='finite_state_machine', # Asegúrate de que así se llama en tu setup.py
                name='finite_state_machine',
                namespace=robot_name,
                output='screen'
            )
        )

        # Instancia del Controlador PD individual
        nodes_to_launch.append(
            Node(
                package='proyecto_abp',
                executable='pd_controller',
                name='pd_controller',
                namespace=robot_name,
                output='screen'
            )
        )

        # Instancia del Procesador de Láser individual
        nodes_to_launch.append(
            Node(
                package='proyecto_abp',
                executable='laser_processor',
                name='laser_processor',
                namespace=robot_name,
                output='screen'
            )
        )

    

    # 3. Lanzar todo el ecosistema de control
    print(f"--- Iniciando Lógica de Control (FSM, PD, Laser, Monitor) para {num_robots} robots ---")
    ls = LaunchService()
    ls.include_launch_description(LaunchDescription(nodes_to_launch))
    
    try:
        ls.run()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()