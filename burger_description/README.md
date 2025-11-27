# Burger Description Package

This package contains the URDF description for the Burger Delivery Robot.

## Prerequisites

- ROS 2 Jazzy
- `colcon` build tool

## Installation

1.  Clone or copy this package into your ROS 2 workspace `src` directory.
    ```bash
    mkdir -p ~/ros2_ws/src
    cp -r /path/to/burger_description ~/ros2_ws/src/
    ```

2.  Build the workspace.
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select burger_description
    ```

3.  Source the setup script.
    ```bash
    source install/setup.bash
    ```

## Usage

### Lanzar Visualización (Nodo 1: robot_state_publisher)

**Terminal 1 - Visualización:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch burger_description display.launch.py
```

Esto lanza automáticamente:
- `robot_state_publisher` - Publica TF
- `joint_state_publisher_gui` - GUI para mover articulaciones manualmente
- `rviz2` - Visualización 3D

**Configuración en RViz:**
- En el panel izquierdo "Global Options" → **Fixed Frame** → Selecciona `map`
- Verifica que estén habilitados: ✓ RobotModel, ✓ TF

### Lanzar Tu Nodo de Control (Nodo 2: Controlador)

**Opción A: Nodo Python simple**

1. Crea el archivo `~/ros2_ws/src/arm_controller.py`:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SimpleArmController(Node):
    def __init__(self):
        super().__init__('simple_arm_controller')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.move_arm)
        
    def move_arm(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gen3_joint_1', 'gen3_joint_2', 'gen3_joint_3', 
                    'gen3_joint_4', 'gen3_joint_5', 'gen3_joint_6', 'gen3_joint_7']
        msg.position = [0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SimpleArmController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

2. **Terminal 2 - Controlador:**
```bash
cd ~/ros2_ws/src
python3 arm_controller.py
```

**Opción B: Con MoveIt2 (recomendado)**

```bash
# Terminal 2 - MoveIt2
ros2 launch moveit_config demo.launch.py
```

### Workflow Completo

```bash
# Terminal 1: Visualización (siempre primero)
ros2 launch burger_description display.launch.py

# Terminal 2: Tu controlador (después de ver RViz)
python3 arm_controller.py

# Terminal 3 (opcional): Monitorear topics
ros2 topic echo /joint_states
```


This will launch:
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2` (with pre-configured displays for RobotModel and TF)

### RViz Configuration

**Important:** In RViz, set the **Fixed Frame** to `map` in the "Global Options" panel (left sidebar). This ensures the entire scene (robot arm, mobile robots, table, etc.) is visible.

## Robot Components

This URDF includes:
- **Kinova Gen3 (7 DOF)** - Ultra-lightweight robotic arm
- **Robotiq 2F-85** - Adaptive gripper
- **Scene elements** - Table, staging area, delivery slots, mobile robot bases

## Integration with Motion Control Nodes

### Architecture Overview: Two Independent Nodes

Your robot control system consists of **two separate nodes**:

```
┌─────────────────────────────────────────────────────────┐
│  NODO 1: robot_state_publisher (Ya incluido)           │
│  ────────────────────────────────────────────────────   │
│  Función: Publicar TF (transformaciones 3D)            │
│  Input:  /joint_states (ángulos de articulaciones)     │
│  Output: /tf (posiciones 3D de cada link)              │
│  Estado: ✓ Ya corriendo cuando lanzas display.launch   │
└─────────────────────────────────────────────────────────┘
                            ▲
                            │ lee /joint_states
                            │
┌─────────────────────────────────────────────────────────┐
│  NODO 2: Tu controlador de movimiento (Por crear)      │
│  ────────────────────────────────────────────────────   │
│  Función: Planificar y ejecutar trayectorias           │
│  Input:  Objetivo (ej: "agarrar hamburguesa en X,Y,Z") │
│  Output: /joint_states (ángulos calculados)            │
│  Estado: ✗ Debes crearlo                               │
└─────────────────────────────────────────────────────────┘
```

**Resumen:**
- **Nodo 1** (robot_state_publisher): Convierte ángulos → posiciones 3D (ya funciona)
- **Nodo 2** (tu controlador): Calcula qué ángulos enviar para mover el brazo

### 1. Opción Simple: Publicar Ángulos Directamente

Para pruebas rápidas, publica ángulos manualmente:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SimpleArmController(Node):
    def __init__(self):
        super().__init__('simple_arm_controller')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.move_arm)
        
    def move_arm(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gen3_joint_1', 'gen3_joint_2', 'gen3_joint_3', 
                    'gen3_joint_4', 'gen3_joint_5', 'gen3_joint_6', 'gen3_joint_7']
        msg.position = [0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0]  # Ángulos en radianes
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SimpleArmController()
    rclpy.spin(node)
```

**Limitación:** Debes calcular los ángulos manualmente (cinemática inversa).

### 2. Opción Recomendada: MoveIt2 (Planificador Avanzado)

MoveIt2 calcula automáticamente los ángulos necesarios para alcanzar una posición XYZ:

```bash
# Instalar MoveIt2
sudo apt install ros-jazzy-moveit

# Generar configuración para este robot
ros2 run moveit_setup_assistant moveit_setup_assistant
```

Luego carga el URDF desde: `~/ros2_ws/install/burger_description/share/burger_description/urdf/burger_delivery_gen3.urdf`

**Ventajas:**
- Cinemática inversa automática (das XYZ, MoveIt calcula ángulos)
- Evita colisiones
- Planifica trayectorias suaves

### 3. Control de Hardware Real (ros2_control)

El URDF ya incluye configuración `ros2_control` para el Kinova Gen3:
- **IP del robot:** 192.168.11.11 (definido en URDF línea 866)
- **Driver:** `kortex2_driver/KortexMultiInterfaceHardware`

Para conectar al robot real:
```bash
sudo apt install ros-jazzy-kortex-driver
ros2 launch kortex_driver kortex_driver.launch.py robot_ip:=192.168.11.11
```

### 4. Coordinate Frames para Pick-and-Place

Frames clave en el árbol TF (usa con `tf2` para obtener posiciones):
- `gen3_end_effector_link` - Punta del gripper
- `burger_grip_frame` - Punto de agarre optimizado (+18cm desde end effector)
- `delivery_slot_1` - Ubicación de entrega de hamburguesas
- `robot_a_tray_frame`, `robot_b_tray_frame` - Bandejas de robots móviles

**Ejemplo de uso:**
```python
from tf2_ros import Buffer, TransformListener

# Obtener posición del gripper respecto a la mesa
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)
transform = tf_buffer.lookup_transform('table_link', 'gen3_end_effector_link', rclpy.time.Time())
```

## Micro-ROS Debugging 📡

For issues related to micro-ROS communication (ESP32 connectivity, agent issues), we provide automatic diagnostic scripts for both operating systems.

**1. Windows (PowerShell):**
```powershell
.\diagnostico_microros.ps1
```

**2. Linux (Bash/WSL):**
```bash
chmod +x diagnostico_microros.sh
./diagnostico_microros.sh
```

For a detailed guide on interpreting results, firewall configuration, and router settings (AP Isolation), please refer to the root documentation:

[👉 Go to Detailed Micro-ROS Debugging Guide](../../README.md#diagnóstico-de-comunicación-micro-ros)


