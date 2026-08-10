# requirements.md

Este documento detalla todas las dependencias de Python necesarias para ejecutar los nodos de visión, controladores, navegación (Nav2) y simulaciones (Gazebo y RViz) del proyecto en ROS2 Jazzy.

## Dependencias de Python

Librerías matemáticas y de visión artificial utilizadas en los scripts (`cameraProcessor.py`, `laserProcessor.py`, `custom_map_merger.py`). 

Puedes instalarlas usando `pip` con el siguiente contenido:

```text
numpy>=1.24.0
opencv-python>=4.8.0
matplotlib>=3.7.0
PyYAML>=6.0
