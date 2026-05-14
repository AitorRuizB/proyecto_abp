# Historial de Problemas y Soluciones

## Problema principal: Error de TF tree al pasar a NAV2TARGET no hay /map comun a todos los odom de cada robot [NO RESUELTO]

**Causa raíz problema:**
- **Condición de carrera en el lanzamiento:** Al ejecutar `start_nav.py`, los nodos que dependen del árbol de transformaciones (como `global_costmap` o `static_transform_publisher`) se iniciaban en paralelo al `map_server`.
- El `map_server` es el responsable de crear y publicar el frame `/map` inicial.
- Los otros nodos intentaban buscar transformaciones hacia o desde el frame `/map` antes de que este existiera, resultando en el error `Timed out waiting for transform ... tf error: Invalid frame ID "map"`.
- El uso de `bringup_launch.py` para cada robot, aunque funcional, lanzaba múltiples `map_server` redundantes y no permitía controlar el orden de arranque fácilmente.

**Problema con el start_slam.py**
- Al hacer el merge del mapa se cierra el nodo con un error que revienta el TF tree. Hay que asegurar que el map se cierra bien y se mapea como padre a todos los robot_x/odom 
