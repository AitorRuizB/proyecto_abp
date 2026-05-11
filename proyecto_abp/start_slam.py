#!/usr/bin/env python3
import sys
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService, LaunchDescription
from launch_ros.actions import Node, LifecycleNode

def main():
    # Comprobar que le pasamos el número de robots
    if len(sys.argv) < 2:
        print("Uso: ros2 run proyecto_abp start_slam <numero_de_robots>")
        return

    num_robots = int(sys.argv[1])
    pkg = get_package_share_directory('proyecto_abp')
    slam_yaml = os.path.join(pkg, 'config', 'slam.yaml')

    nodes = []

    # 1. BUCLE: Crear un nodo de SLAM por cada robot
    for i in range(1, num_robots + 1):
        robot_name = f'robot_{i}' # <--- ADAPTADO AL GUION BAJO
        
        # El nodo de SLAM para este robot específico
        nodes.append(LifecycleNode(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', namespace=robot_name,
            parameters=[
                slam_yaml,
                {
                    'odom_frame':   f'{robot_name}/odom',
                    'base_frame':   f'{robot_name}/base_link', # <--- CAMBIADO A base_link
                    'map_frame':    f'{robot_name}/map',
                    'scan_topic':   f'/{robot_name}/scan',
                    'use_sim_time': True,
                    'transform_publish_period': 0.0 # Para no pelear con el main.launch
                }
            ],
            remappings=[
                ('/map',          f'/{robot_name}/map'),
                ('/tf',           '/tf'),
                ('/tf_static',    '/tf_static'),
            ]
        ))

        # El manager para activar el SLAM de este robot
        nodes.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_slam', namespace=robot_name,
            parameters=[{
                'use_sim_time': True,
                'autostart':    True,
                'node_names':   ['slam_toolbox'],
                'bond_timeout': 0.0,
            }]
        ))

    # 2. El nodo que une todos los mapas al final
    nodes.append(Node(
        package='proyecto_abp',
        executable='map_merge',
        name='custom_map_merger',
        parameters=[{'use_sim_time': True, 'num_robots': num_robots}]
    ))

    # 3. Lanzar todo de golpe
    print(f"--- Iniciando Mapeo (SLAM) para {num_robots} robots ---")
    ld = LaunchDescription(nodes)
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

if __name__ == '__main__':
    main()