# Biblioteca de proyectos ROS y apuntes

Este repositorio centraliza la documentación y los modelos usados en el proyecto **Burger Delivery con ROS 2 Jazzy**. El foco principal es la celda donde un manipulador **Kinova Gen3** entrega bandejas a robots diferenciales coordinados mediante `tf2`, visión por AprilTags y micro-ROS, optimizado para entornos inalámbricos de alta densidad.

## 📁 Estructura relevante

- `burger_description/`: **Paquete principal** con el URDF y archivos de lanzamiento.
  - `GUIA_DE_USO.md`: Guía de inicio rápido para visualización.
  - `urdf/delivery_scene_fixed.urdf`: Modelo URDF corregido (Kinova + Carritos + Escena).
- `network_setup/`: **Infraestructura de Red** (Configuración Crítica y Diagnóstico).
  - [`DIAGNOSTICO_RED.md`](network_setup/DIAGNOSTICO_RED.md): **Guía Maestra** de resolución de problemas (Niveles 1-4).
  - [`ROS2_NETWORK_CONFIG.md`](network_setup/ROS2_NETWORK_CONFIG.md): Configuración de RMW, Domain ID y Modo Mirrored de WSL.
  - `fastdds_discovery.xml`: Configuración para Discovery Server (evita problemas de Multicast).
- `ros_burger_delivery.md`: Documentación técnica de la arquitectura y flujos.
- `lanzar_robot.sh`: Script de automatización para lanzar la visualización completa.

## 🚀 Uso rápido del sistema

### Opción Rápida (Scripts de automatización)
Para lanzar la visualización en RViz con todos los componentes preconfigurados:
```bash
chmod +x lanzar_robot.sh
./lanzar_robot.sh
```

### Opción Manual (WSL/Linux)
1. **Compilar el paquete:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select burger_description
   source install/setup.bash
   ```
2. **Lanzar visualización:**
   ```bash
   ros2 launch burger_description display.launch.py
   ```
   *Incluye: Kinova Gen3, Gripper Robotiq, Robots móviles, y la mesa de entrega.*

---

## 📶 Diagnóstico y Configuración de Red ROS 2

Hemos implementado un sistema de diagnóstico multinivel para asegurar la estabilidad del sistema en redes WiFi 6 y entornos con micro-ROS.

### 1. Herramientas de Diagnóstico (Niveles 1-4)
Si experimentas lag, pérdida de tópicos o desconexión de las ESP32, usa los scripts en `network_setup/`:

| Nivel | Script | Objetivo | Diagnóstico |
| :--- | :--- | :--- | :--- |
| **1** | `test_ros2_network.sh` | **Salud Local** | Verifica variables de entorno y visibilidad de nodos. |
| **2** | `diagnostico_wifi.sh` | **Calidad Física** | Analiza latencia al router, RSSI y congestión WiFi. |
| **3** | `analisis_trafico_ros2.sh` | **Rendimiento** | Optimización de MTU, fragmentación y ancho de banda. |
| **4** | `test_wan_access.sh` | **Acceso WAN** | Verifica salida a internet (necesario para updates/drivers). |

### 2. Configuración Óptima (Recomendada)
Para máxima estabilidad en WiFi, configura tu `~/.bashrc` con:
```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```
*Nota: CycloneDDS gestiona mucho mejor la pérdida de paquetes que el RMW por defecto.*

### 3. Configuración Crítica del Router & Firewall
**⚠️ El ping puede funcionar pero ROS no si estas opciones están mal:**

- **AP Isolation:** Debe estar **DESACTIVADO** en el router para que los dispositivos se vean entre sí.
- **Reserva IP:** El PC Principal debe tener la IP `192.168.1.100` reservada mediante MAC.
- **Firewall:** Abre los puertos UDP `7400-7500` (DDS) y `8888` (micro-ROS).
  ```bash
  sudo ufw allow 7400:7500/udp
  sudo ufw allow 8888/udp
  ```

### 4. Modo Mirrored (WSL2)
Si usas Windows, es **fundamental** activar el `networkingMode=mirrored` en tu `.wslconfig` para que WSL comparta la IP real de tu PC y sea visible en la red física. Ver [guía detallada acá](network_setup/ROS2_NETWORK_CONFIG.md#💻-wsl-networking-configuration-windows-users).

---

## 🛠️ Visualizadores y Referencias

- **Visualización URDF:** [Online URDF Viewer](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html) para pruebas rápidas de modelos.
- **Captura Avanzada:** Si nada funciona, usa `tcpdump` o Wireshark con el filtro `udp.port == 8888` o `rtps`.
- **Árbol TF2:** Consulta `tf_tree_diagram.svg` para entender la jerarquía de coordenadas.

> **Importante:** Mantén los URDF y la documentación sincronizados. Cualquier cambio en joints/links debe reflejarse en la guía técnica para evitar errores de posicionamiento.

