# 🍔 Burger Delivery (ROS 2 Jazzy)

[🇪🇸 Español](README.md) | [🇬🇧 English](README_en.md)

![Pipeline Pick and Place](pick_and_place_pipeline.svg)

> **Contexto del Proyecto**: Este repositorio centraliza la documentación, los modelos y los recursos usados en un entorno avanzado de robótica colaborativa. El objetivo es simular y controlar una celda de trabajo donde un manipulador **Kinova Gen3** interactúa espacialmente para entregar bandejas a **robots móviles diferenciales** (estilo TurtleBot). Todo el ecosistema está coordinado mediante `tf2`, localización visual por **AprilTags**, framework de planificación **MoveIt 2** y protocolos **micro-ROS**, optimizado para entornos inalámbricos de alta densidad.

---

## 🎯 Alcance actual

Este repositorio funciona actualmente como el paquete de descripción (`burger_description`) y el centro neurálgico de documentación y configuración. Contiene:

- ✅ Un paquete ROS 2 compilable con los URDF de la escena de entrega.
- ✅ Mallas vendorizadas (Kinova Gen3, pinzas Robotiq, y carros móviles).
- ✅ Entorno de lanzamiento (`launch`) para TF dinámicos, `robot_state_publisher` y RViz2.
- ✅ Amplia documentación de arquitectura, instalación y diagnóstico de red.
- ✅ Scripts nativos avanzados para estabilización de hardware y parcheo de latencias.

*Nota: Aún no contiene un stack completo "out-of-the-box" de operación autónoma (no incluye paquetes finales de navegación o lógica de negocio compilables aquí, sino la base arquitectónica y geométrica para integrarlos).*

---

## 🚀 Inicio Rápido (Quick Start)

### Requisitos Previos
- Ubuntu con **ROS 2 Jazzy** instalado. (Si no lo tienes, revisa `install_ros2.sh` y la guía en `ros2_setup/`).
- Herramienta `colcon` y paquetes de escritorio ROS 2 (RViz2, joint_state_publisher_gui).

### Compilación
Asegúrate de clonar este repositorio dentro de la carpeta `src` de tu workspace (ej. `~/ros2_ws/src`):

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select burger_description
source install/setup.bash
```
*(Alternativa: usar el script `./build_burger.sh` incluido).*

### Ejecución y Visualización
Puedes lanzar la simulación base en RViz usando el script rápido:

```bash
./lanzar_robot.sh
```

O hacerlo manualmente:
```bash
ros2 launch burger_description display.launch.py
```

### 🌐 Monitor de Red y Telemetría ROS 2 (Dashboard Web)
Para supervisar dispositivos conectados al router (AX12), latencia/jitter WiFi, tráfico de dominios DDS y estado del agente Micro-ROS (puerto 8888) en tiempo real:

```bash
bash network_setup/iniciar_monitor.sh
```
*(Abre automáticamente la interfaz en `http://127.0.0.1:8080`).*

---

## 📂 Estructura del Proyecto

- `burger_description/`: Paquete ROS 2 principal (URDFs, meshes visuales/colisión, launch files).
  - `urdf/delivery_scene_fixed.urdf`: Escena estática principal.
  - `visual/` y `vendor/`: Mallas 3D y recursos de terceros (Kinova).
- `scripts/`: Herramientas vitales en Python para parcheado de latencias del Kinova y testeos unitarios físicos CLI.
- `network_setup/`: Diagnóstico avanzado de red, configuración para WiFi 6/AX12, herramientas de telemetría y el **Monitor de Red Web Híbrido** (`monitor_red/`).
- `vision_setup/`: Guías de conectividad RTSP de cámaras y localización con AprilTags.
- `ros2_setup/`: Notas de instalación, scripts y configuración del entorno Jazzy.

---

## 📚 Índice de Documentación (Categorizado)

Hemos organizado la extensa documentación técnica para facilitar tu aprendizaje y configuración:

### 📖 1. Guías Generales y Arquitectura
- [Guía Rápida de Uso](burger_description/GUIA_DE_USO.md)
- [Documento Técnico de Arquitectura (Burger Delivery)](ros_burger_delivery.md)
- [Proyecto Evaluable MoveIt2 & Delivery](PROYECTO_INTERMEDIO_MOVEIT2_DELIVERY.md)

### 🤖 2. Manipulación y Movimiento (Kinova & MoveIt)
- [Mejoras de Movimiento MTC y Ghost Visualizer](MEJORAS_MOVIMIENTO_KINOVA.md)
- [Pruebas de Movimiento y Coordenadas Cartesianas CLI](PRUEBAS_MOVIMIENTO.md)
- [Instalación del Stack Kortex](ros2_setup/INSTALACION_KORTEX.md)

### 👁️ 3. Visión Computacional y Localización
- [⭐ Arquitectura del Sistema de Localización AprilTag](vision_setup/LOCALIZACION_APRILTAG.md) *(Muy recomendado para entender el árbol TF)*
- [Verificación y Diagnóstico de Cámara Nativa](vision_setup/VERIFICACION_CAMARA.md)
- [Aislamiento de Latencia Visual (TCP vs ROS 2)](vision_setup/DIAGNOSTICO_RED_VISION.md)

### 🌐 4. Redes, Configuración y micro-ROS
- [⭐ Monitor de Red Híbrido (ROS 2 & Router AX12 Web Dashboard)](network_setup/iniciar_monitor.sh) *(Lanzador: `bash network_setup/iniciar_monitor.sh`)*
- [Diagnóstico de Red ROS 2](network_setup/DIAGNOSTICO_RED.md)
- [Configuración de Red Recomendada](network_setup/ROS2_NETWORK_CONFIG.md)
- [Guía de Configuración TP-Link Archer AX12](network_setup/router_tplink_ax12_config.md)
- [Verificación de Entorno ROS 2 Jazzy](ros2_setup/verificar_ros2.md)
- **Scripts de diagnóstico**: Encuentra herramientas como `test_ros2_network.sh`, `diagnostico_wifi.sh`, `diagnostico_microros.sh`, `analisis_trafico_ros2.sh` y más en la carpeta `network_setup/`.

---

## ⚙️ Notas de Uso Avanzado (TFs y Modos)

El comportamiento de los carritos móviles en la escena depende del sistema de **AprilTags**.

| Modo | Comando de Launch | Cuándo usarlo |
|---|---|---|
| **Producción** (Por defecto) | `ros2 launch burger_description display.launch.py` | Para operación real. Los carritos se acoplan dinámicamente al mapa cuando el nodo de localización AprilTag publica los TFs (`tag_mesa -> tag_carrito*`). |
| **Debug / Visualización** | `ros2 launch burger_description display.launch.py use_static_carts:=true` | Útil si el nodo de localización no está corriendo. Fuerza la publicación de TFs estáticos para visualizar dónde deberían estar los carritos. |

> ⚠️ **Advertencia de TF**: Nunca publiques `map -> car_base_link` si el sistema de AprilTag ya está publicando `tag_carrito -> car_base_link`. Romperá el árbol TF.

---

## 🛠️ Scripts de Depuración Física y Hardware
En `scripts/`, encontrarás potentes herramientas de parcheo que no afectan a la simulación estándar:
- `apply_kinova_smooth_movement.py`: Parcheador que inyecta "Low Latency" a los drivers C++, corrigiendo vibraciones inerciales (jittering).
- `test_kinova_pose.py`: Validador espacial para comprobar si una coordenada rompe la cinemática antes de probarla en MoveIt.
- `test_kinova_camera.py`: Extractor GStreamer/OpenCV para evaluar cámara sin la sobrecarga de ROS.

---

## 🖼️ Diagramas de Arquitectura Adicionales

### Esquema de Localización (AprilTag)
Mapeo posicional relativo entre el brazo, la mesa y el marco estático de referencia de las etiquetas.
![Esquema de localizacion AprilTag](vision_localizacion_whiteboard.svg)

### Navegación y Acciones (Turtlebot)
Bucle de control para la interacción y peticiones del robot de reparto.
![Arquitectura Nav2 Turtlebot](turtlebot_nav_service.svg)

### Árbol de Transformaciones (TF)
*(Para generar uno en tiempo real: con el launch corriendo, usa `ros2 run tf2_tools view_frames` en otra terminal).*
![Project TF Tree](tf_tree_diagram.svg)

### Dependencias de Red y Software
- [Diagrama de red ROS 2](network_setup/ros_network_diagram.svg)
- [Mapa de Dependencias](ros2_setup/MAPA_DEPENDENCIAS.svg)
- [Mapa de Dependencias AI](ros2_setup/MAPA_DEPENDENCIAS_AI.svg)
