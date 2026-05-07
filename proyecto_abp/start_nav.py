#!/usr/bin/env python3
import sys
import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def main():
    if len(sys.argv) < 2:
        print("Uso: ros2 run proyecto_abp start_nav <nombre_robot>")
        return

    robot_name = sys.argv[1]
    pkg = get_package_share_directory('proyecto_abp')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    
    # 1. Leer el YAML base como TEXTO PLANO
    nav2_yaml_base = os.path.join(pkg, 'config', 'nav2.yaml')
    with open(nav2_yaml_base, 'r') as f:
        nav_text = f.read()
    
    # 2. Reemplazar comodines sin usar librerías yaml
    nav_text = nav_text.replace('<ROBOT>', robot_name)
    
    # 3. Guardar el archivo temporal exactamente con la misma estructura original
    nav2_robot_path = os.path.join(tempfile.gettempdir(), f'nav2_{robot_name}.yaml')
    with open(nav2_robot_path, 'w') as f:
        f.write(nav_text)

    # 4. Lanzar la navegación oficial
    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'namespace': robot_name,
                'use_namespace': 'True',
                'params_file': nav2_robot_path,
                'use_sim_time': 'True',
                'autostart': 'True'
            }.items()
        )
    ])

    print(f"--- Iniciando Navegación para {robot_name} ---")
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

if __name__ == '__main__':
    main()