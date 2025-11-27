# Biblioteca de proyectos ROS y apuntes

Este repositorio centraliza la documentación y los modelos usados en el proyecto **Burger Delivery con ROS 2 Jazzy** junto con materiales de estudio (carpetas `IA/`, `PDS BIO/`, etc.). El foco principal es la celda donde un manipulador **Kinova Gen3** entrega bandejas a robots diferenciales coordinados mediante `tf2`, visión por AprilTags y micro-ROS.

## Estructura relevante

- `ROS/burger_description/`: **Paquete principal** con la descripción del robot y archivos de lanzamiento.
  - `GUIA_DE_USO.md`: **¡LEER PRIMERO!** Guía de lanzamiento rápido (PowerShell/WSL).
  - `urdf/delivery_scene_fixed.urdf`: Modelo URDF final corregido (Kinova + Carritos + Escena).
- `ROS/ros_burger_delivery.md`: guía completa del sistema (arquitectura, tf2, nodos, flujos).
- `ROS/joints.md`, `ROS/launch.md`, `ROS/ros.md`: notas auxiliares.

## Uso rápido del modelo ROS (burger_description)

Hemos simplificado el lanzamiento del robot. Ver `ROS/burger_description/GUIA_DE_USO.md` para detalles.

**Opción Rápida (PowerShell):**
```powershell
lanzar_robot
```

**Opción Manual (WSL):**
1. **Compilar (si hay cambios):**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select burger_description
   source install/setup.bash
   ```
2. **Lanzar visualización:**
   ```bash
   ros2 launch burger_description display.launch.py
   ```
   Esto carga:
   - `robot_state_publisher` (TF)
   - `rviz2` (Visualización con configuración precargada)
   - `joint_state_publisher_gui` (Control manual de joints)

3. **Visualización en RViz:**
   - Fixed Frame: `map`
   - El modelo incluye: Brazo Kinova Gen3, Gripper Robotiq 2F-85, 2 Robots móviles (Carro2/Carro4), Mesa y zona de entrega.

## Visualizadores URDF

- [Online URDF Viewer](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html): Herramienta web para visualizar y manipular modelos URDF directamente en el navegador.

## Diagnóstico de Comunicación micro-ROS

Si experimentas problemas de comunicación entre las ESP32s y el agente micro-ROS (el ping funciona pero ROS no se comunica), sigue estos pasos:

### Script de Diagnóstico Automático

Ejecuta el script de diagnóstico según tu sistema operativo:

**Windows (PowerShell):**
```powershell
.\diagnostico_microros.ps1
```

**Linux (Bash):**
```bash
chmod +x diagnostico_microros.sh
./diagnostico_microros.sh
```

El script verificará automáticamente:
- IP y conectividad de red
- Estado del firewall
- Puerto UDP 8888 (agente micro-ROS)
- Conectividad a ESP32s
- Variables de entorno ROS
- **Router/Gateway** (nuevo) - Extrae información del router automáticamente
  - Gateway IP y conectividad
  - SSID conectado (verifica que sea 'ros2')
  - MAC Address del PC (necesaria para reserva DHCP)
  - Configuración DNS y rutas de red
  - Guía de configuración manual del router
- Tabla ARP

**Nota:** Si usas un **TP-Link Archer AX12**, consulta la guía específica: [`router_tplink_ax12_config.md`](router_tplink_ax12_config.md)

### Configuración del Firewall

El firewall puede bloquear el tráfico UDP necesario para micro-ROS. Para depuración, desactívalo temporalmente:

#### Windows

**Opción 1: Desactivar temporalmente (método rápido)**
```powershell
# Desactivar firewall para red privada
Set-NetFirewallProfile -Profile Private -Enabled False

# Para reactivarlo después de las pruebas
Set-NetFirewallProfile -Profile Private -Enabled True
```

**Opción 2: Crear regla específica (recomendado para producción)**
```powershell
New-NetFirewallRule -DisplayName "micro-ROS Agent UDP" `
  -Direction Inbound `
  -Protocol UDP `
  -LocalPort 8888 `
  -Action Allow
```

#### Linux (Ubuntu)

**Opción 1: Desactivar UFW temporalmente**
```bash
# Verificar estado
sudo ufw status

# Desactivar
sudo ufw disable

# Para reactivar después
sudo ufw enable
```

**Opción 2: Agregar regla específica (recomendado)**
```bash
# Permitir puerto 8888 UDP
sudo ufw allow 8888/udp

# Verificar reglas
sudo ufw status numbered
```

### Configuración Crítica del Router

**⚠️ IMPORTANTE:** Verifica en la configuración de tu router WiFi que:

1. **AP Isolation (Aislamiento de Clientes) esté DESACTIVADO**
   - Esta es la causa #1 cuando el ping funciona pero ROS no
   - Para TP-Link Archer AX12: Advanced → Wireless → Wireless Settings → Desmarca "Enable AP Isolation"
   
2. **IP fija reservada para el PC Principal** (`192.168.1.100`)
   - Router → DHCP → Reserva de IP → Vincular MAC del PC con 192.168.1.100
   - El script de diagnóstico te muestra tu MAC Address

3. **Todos los dispositivos en la misma subred** (`192.168.1.0/24`)

4. **Smart Connect desactivado** (si usas router WiFi 6 como el Archer AX12)
   - Puede causar problemas de conectividad con ESP32s

**Prueba de diagnóstico rápida:**
Si la comunicación **funciona con un hotspot de celular** pero **NO con el router WiFi**, confirma que el problema está en la configuración del router (típicamente AP Isolation).

### Verificación del Agente

Ejecuta el agente con logs verbosos para ver intentos de conexión:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

### Configuración Crítica del Router

**⚠️ IMPORTANTE:** Verifica en la configuración de tu router WiFi que:

1. **AP Isolation (Aislamiento de Clientes) esté DESACTIVADO**
   - Esta es la causa #1 cuando el ping funciona pero ROS no
   - Ubicación típica: Configuración WiFi → Seguridad → AP Isolation
   
2. **IP fija reservada para el PC Principal** (`192.168.1.100`)
   - Router → DHCP → Reserva de IP → Vincular MAC del PC con 192.168.1.100

3. **Todos los dispositivos en la misma subred** (`192.168.1.0/24`)

### Captura de Tráfico (Diagnóstico Avanzado)

Si los pasos anteriores no resuelven el problema, captura el tráfico UDP:

**Windows (Wireshark):**
1. Instalar Wireshark
2. Capturar en interfaz WiFi
3. Filtro: `udp.port == 8888`
4. Reiniciar ESP32 y observar paquetes

**Linux (tcpdump):**
```bash
sudo tcpdump -i wlan0 -n udp port 8888 -vv
```

Deberías ver paquetes desde las ESP32s hacia `192.168.1.100:8888`. Si no aparecen, el problema está en las ESP32s o el router.

## Referencias

- Consulta `ROS/ros_burger_delivery.md` para detalles de topics, nodos, QoS y secuencia operativa del delivery.
- `ROS/ros_burger_delivery.pdf` ofrece la misma guía en formato imprimible.
- `ROS/ros.md` contiene la arquitectura de red completa, configuración de micro-ROS y localización con AprilTags.
- Diagrama del árbol tf2: `ROS/tf_tree_diagram.svg`.

> Mantén los URDF y la guía sincronizados: cualquier cambio en links/joints debe reflejarse tanto en `burger_delivery_gen3.urdf` como en el apartado de tf2 de la guía para evitar inconsistencias.
