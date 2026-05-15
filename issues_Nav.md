# Historial de Problemas y Soluciones

## Problema principal: El Nav no hace moverse al robot y emplea un merge_map dependiente de la pose inicial de los robots. Además, el mapa resultante posee errores. 

**Posible Solución:**
- Emplear los paquetes `nav2_map_server`, `nav1_amcl` y `nav2_lifecycle_manager` para la localiación del robot.
- Seguir tutorial del siguiente enlace para cambiar el planteamiento del SLAM (https://www.youtube.com/watch?v=cGUueuIAFgw)
- Por tanto, se desea refactorizar la fase de Navegación y prescindir del uso de `start_slam.py` y `start_nav.py` dado que no funcionan correctamente para el MRS

## Otros problemas: Error de TF tree al pasar a NAV2TARGET no hay /map comun a todos los odom de cada robot [RESUELTO]

**Causa raíz problema:**
- **Condición de carrera en el lanzamiento:** Al ejecutar `start_nav.py`, los nodos que dependen del árbol de transformaciones (como `global_costmap` o `static_transform_publisher`) se iniciaban en paralelo al `map_server`.
- El `map_server` es el responsable de crear y publicar el frame `/map` inicial.
- Los otros nodos intentaban buscar transformaciones hacia o desde el frame `/map` antes de que este existiera, resultando en el error `Timed out waiting for transform ... tf error: Invalid frame ID "map"`.
- El uso de `bringup_launch.py` para cada robot, aunque funcional, lanzaba múltiples `map_server` redundantes y no permitía controlar el orden de arranque fácilmente.

##Problema con el start_slam.py [RESUELTO]
- Al hacer el merge del mapa se cierra el nodo con un error que revienta el TF tree.
**Solución propuesta:**
- Emplear un cierre seguro mediante un LaunchService contextualizado

