# Biblioteca de proyectos ROS y apuntes

Este repositorio centraliza la documentación y los modelos usados en el proyecto **Burger Delivery con ROS 2 Jazzy** junto con materiales de estudio (carpetas `IA/`, `PDS BIO/`, etc.). El foco principal es la celda donde un manipulador **Kinova Gen3** entrega bandejas a robots diferenciales coordinados mediante `tf2`, visión por AprilTags y micro-ROS.

## Estructura relevante

- `ROS/ros_burger_delivery.md`: guía completa del sistema (arquitectura, tf2, nodos, flujos).
- `ROS/visual/burger_delivery_gen3.urdf`: escena compuesta con la mesa, Kinova y robots móviles; sirve como fuente del árbol de frames.
- `ROS/joints.md`, `ROS/launch.md`, `ROS/ros.md`: notas auxiliares para joints URDF, lanzamientos y configuración general de ROS 2.
- `ROS/vendor/`: paquetes vendorizados (Kinova Gen3, Robotiq) referenciados por los URDF.
- Carpetas adicionales (`IA/`, `PDS BIO/`, etc.) contienen exámenes y apuntes que no interfieren con el stack ROS.

## Uso rápido del modelo ROS

1. **Publicar transformaciones estáticas** (mesa, staging, robots):
   ```bash
   ros2 run tf2_ros static_transform_publisher 0.80 -1.00 0.80 0 0 0 map table_link
   ros2 run tf2_ros static_transform_publisher 1.20 0.30 0.50 0 0 0 table_link kinova_base_link
   ros2 run tf2_ros static_transform_publisher 0.80 0.00 0.00 0 0 0 map staging_area
   ros2 run tf2_ros static_transform_publisher 0.00 0.40 0.50 0 0 0 staging_area delivery_slot_1
   ros2 run tf2_ros static_transform_publisher 0.20 0.30 0.00 0 0 0 table_link world
   ```
2. **State publisher**: lanzar `robot_state_publisher` con `ROS/visual/burger_delivery_gen3.urdf` para publicar la cadena completa `map → table_link → world → gen3_* → burger_grip_frame`.
3. **Visualización**: abrir RViz, agregar `RobotModel` y `TF` para inspeccionar la escena y validar las relaciones descritas en la guía.

## Visualizadores URDF

- [Online URDF Viewer](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html): Herramienta web para visualizar y manipular modelos URDF directamente en el navegador.

## Referencias

- Consulta `ROS/ros_burger_delivery.md` para detalles de topics, nodos, QoS y secuencia operativa del delivery.
- `ROS/ros_burger_delivery.pdf` ofrece la misma guía en formato imprimible.
- Diagrama del árbol tf2: `ROS/tf_tree_diagram.svg`.

> Mantén los URDF y la guía sincronizados: cualquier cambio en links/joints debe reflejarse tanto en `burger_delivery_gen3.urdf` como en el apartado de tf2 de la guía para evitar inconsistencias.
