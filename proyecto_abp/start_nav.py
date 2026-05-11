#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def main():
    pkg_proyecto = get_package_share_directory('proyecto_abp')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Apunta EXACTAMENTE a tu mapa.yaml y al yaml del robot1
    map_yaml_file = os.path.join(pkg_proyecto, 'config', 'mapa.yaml')
    params_file = os.path.join(pkg_proyecto, 'config', 'nav2.yaml')
    
    print("--- Lanzando Navegación EXCLUSIVAMENTE para robot1 ---")
    
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': 'robot1',
            'use_namespace': 'True',
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': 'True',
            'use_sim_time': 'True',
            
        }.items()
    )

    ld = LaunchDescription([nav_launch])
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

if __name__ == '__main__':
    main()