# burger_delivery

Este repositorio centraliza la documentación, los modelos y los recursos usados en el proyecto **Burger Delivery con ROS 2 Jazzy**. El foco principal es la celda donde un manipulador **Kinova Gen3** entrega bandejas a robots diferenciales coordinados mediante `tf2`, visión por AprilTags y micro-ROS, optimizado para entornos inalámbricos de alta densidad.

Actualmente, el contenido implementado en este repositorio está centrado en el paquete `burger_description`, la visualización de la escena, scripts auxiliares y documentación técnica para Linux o WSL.

## Alcance actual

Este proyecto contiene hoy:

- Un paquete ROS 2 compilable: `burger_description`.
- Modelos URDF de la escena de entrega.
- Un `launch` para publicar TF, mover joints manualmente y abrir RViz.
- Mallas vendorizadas del Kinova Gen3, Robotiq y dos carros móviles.
- Documentación de arquitectura, instalación y red.
- Scripts auxiliares para build, lanzamiento y diagnóstico.

No contiene todavía un stack completo de operación autónoma en este repositorio: no hay paquetes propios de navegación, percepción, planeación o lógica de pedidos listos para compilar aquí.

## Estructura

- `burger_description/`: paquete ROS 2 principal.
- `burger_description/urdf/delivery_scene_fixed.urdf`: escena principal usada por el launch actual.
- `burger_description/urdf/burger_delivery_gen3.urdf`: variante adicional de escena con Gen3.
- `burger_description/launch/display.launch.py`: lanza `robot_state_publisher`, `joint_state_publisher_gui` y `rviz2`.
- `burger_description/rviz/default.rviz`: configuración base de RViz.
- `burger_description/visual/`: mallas auxiliares y modelos de los carros.
- `burger_description/vendor/`: recursos vendorizados de Kinova/Robotiq.
- `network_setup/`: scripts y guías de diagnóstico de red ROS 2 y micro-ROS.
- `ros2_setup/`: notas de instalación y verificación de ROS 2 Jazzy.
- `vision_setup/`: diagnóstico de conectividad visual, protocolos ópticos RTSP y uso de OpenCV.
- `scripts/`: herramientas vitales de parcheado de latencias del Kinova y testeos unitarios físicos CLI.
- `lanzar_robot.sh`: script rápido para abrir la visualización y `rqt`.
- `build_burger.sh`: script de compilación del paquete.

## Qué modela el URDF

La escena incluye:

- Frame raíz `map`.
- Mesa de trabajo `table_link`.
- Zona de staging `staging_area`.
- Un slot de entrega `delivery_slot_1`.
- Base auxiliar del manipulador `kinova_base_link`.
- Tool frame y grip frame (`kinova_tool_frame`, `burger_grip_frame`).
- Cámara aérea `overhead_camera_link`.
- Brazo Kinova Gen3 de 7 GDL.
- Gripper Robotiq 2F-85.
- Frames de cámara de muñeca.
- Dos robots móviles de referencia: `car1` y `car2`.
- Un bloque `ros2_control` para hardware Kinova con plugin `kortex2_driver/KortexMultiInterfaceHardware`.

## Requisitos

- Ubuntu con ROS 2 Jazzy instalado.
- `colcon`.
- Paquetes de escritorio de ROS 2 para usar RViz.
- Dependencias declaradas por el paquete:
  - `joint_state_publisher_gui`
  - `robot_state_publisher`
  - `rviz2`
  - `xacro`

Si necesitas instalar ROS 2 desde cero, revisa `install_ros2.sh` y la documentación en `ros2_setup/`.

## Compilación

Si este repositorio está dentro de `~/ros2_ws/src`, compila así:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select burger_description
source install/setup.bash
```

También existe el script:

```bash
./build_burger.sh
```

## Ejecución

Lanzamiento manual:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch burger_description display.launch.py
```

Ese launch abre:

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`

Script rápido:

```bash
./lanzar_robot.sh
```

Ese script además intenta abrir `rqt`.

## Notas de uso

- El launch actual carga `burger_description/urdf/delivery_scene_fixed.urdf`.
- En RViz conviene usar `map` como `Fixed Frame`.
- El paquete instalable es `burger_description`, aunque el repositorio se llama `burger_delivery`.
- `lanzar_robot.sh` usa una ruta absoluta al workspace del entorno actual: `/home/roncanciovl/ros2_ws/install/setup.bash`. Si mueves el proyecto a otra máquina, tendrás que ajustarla.

## Documentación adicional

- `burger_description/GUIA_DE_USO.md`: guía rápida de uso.
- `burger_description/README.md`: README específico del paquete.
- `ros_burger_delivery.md`: documento técnico de arquitectura y flujo propuesto.
- `launch.md`: notas para visualizar URDF en visor web.
- `network_setup/DIAGNOSTICO_RED.md`: guía de diagnóstico de red ROS 2.
- `network_setup/ROS2_NETWORK_CONFIG.md`: configuración de red recomendada.
- `ros2_setup/INSTALACION_KORTEX.md`: instalación del stack Kortex.
- `ros2_setup/verificar_ros2.md`: verificación de instalación de ROS 2 Jazzy.
- `MEJORAS_MOVIMIENTO_KINOVA.md`: guías de escalado MTC (5%) y explicativo del visualizador fantasma Ghost en RViz.
- `PRUEBAS_MOVIMIENTO.md`: manual unitario explicando cómo inyectar coordenadas de prueba cartesianas usando terminal.
- `vision_setup/VERIFICACION_CAMARA.md`: diagnóstico de cámara nativo.
- `vision_setup/DIAGNOSTICO_RED_VISION.md`: aislamiento de latencia visual (TCP vs ROS 2 Topic).

## Diagramas de referencia

Los siguientes diagramas en formato SVG sirven como referencia rápida de arquitectura, transformaciones y vision del sistema:

### Esquema de Localización por Visión

Esquema visual que mapea la posición del brazo respecto a la mesa de trabajo de los móviles y el marco estático de referencia de los AprilTags:

![Esquema de localizacion por vision](vision_localizacion_whiteboard.svg)

### Navegación de Movimiento (Nav2 + Visión)

Arquitectura Cliente/Servidor de ROS 2 Action y Lazo de Control Cinemático compensado para los comandos de entrega del TurtleBot:

![Arquitectura Nav2 Turtlebot](turtlebot_nav_service.svg)

### Pipeline de Manipulación Pick & Place

Jerarquía lógica y trayectoria física planificadas a través del framework MoveIt Task Constructor (MTC):

![Pipeline Pick and Place](pick_and_place_pipeline.svg)

### Árbol TF

![Árbol TF del proyecto](tf_tree_diagram.svg)

### Red ROS 2

![Diagrama de red ROS 2](network_setup/ros_network_diagram.svg)

### Dependencias

![Mapa de dependencias](ros2_setup/MAPA_DEPENDENCIAS.svg)

### Dependencias AI

![Mapa de dependencias AI](ros2_setup/MAPA_DEPENDENCIAS_AI.svg)

## Diagnóstico de red y micro-ROS

En `network_setup/` hay utilidades para revisar conectividad y rendimiento:

- `test_ros2_network.sh`
- `diagnostico_wifi.sh`
- `analisis_trafico_ros2.sh`
- `test_wan_access.sh`
- `diagnostico_microros.sh`
- `diagnostico_microros.ps1`

## Scripts de Depuración Física y Hardware
Alojados en `scripts/`, encontrarás potentes herramientas nativas en Python enfocadas en resolver barreras prácticas de la robótica y estabilizar el sistema:

- `apply_kinova_smooth_movement.py`: Parcheador inteligente que inyecta opciones de "Low Latency" a los drivers C++ del hardware, corrigiendo saltos y vibraciones inerciales (jittering).
- `test_kinova_pose.py`: Tester CLI de interacciones espaciales. Útil para verificar si una coordenada [X,Y,Z] rompe cinemática antes de llevarla al código final en el Pipeline de MoveIt.
- `test_kinova_camera.py`: Extractor Gstreamer/OpenCV por RTSP para diagnosticar la cámara y guardar *Datasets* sin invocar a los pesados tópicos ROS.

Estas herramientas son independientes y pueden usarse en demanda sin afectar el launch modular de simulaciones.
