# Guía de Instalación y Pruebas: Kinova Gen3 7DOF (ROS 2 Jazzy)

**Robot:** Kinova Gen3 · 7 grados de libertad · Pinza Robotiq 2F-85  
**Sistema:** Ubuntu 24.04 · ROS 2 Jazzy · IP del robot: `192.168.1.10`

> Esta guía cubre desde la instalación del driver hasta la primera prueba de movimiento real del brazo.

---

## Índice

1. [Requisitos Previos](#1-requisitos-previos)
2. [Clonar el Driver](#2-clonar-el-driver)
3. [Instalar Dependencias](#3-instalar-dependencias)
4. [Compilar](#4-compilar)
5. [Conectar y Lanzar](#5-conectar-y-lanzar)
6. [Verificación del Driver](#6-verificación-del-driver)
7. [Pruebas de Movimiento](#7-pruebas-de-movimiento)
8. [Solución de Problemas](#8-solución-de-problemas)

---

## 1. Requisitos Previos

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools git python3-colcon-common-extensions
```

---

## 2. Clonar el Driver

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Kinovarobotics/ros2_kortex.git
```

---

## 3. Instalar Dependencias

### 3.1 Inicializar rosdep (solo la primera vez)
```bash
sudo rosdep init
rosdep update
```

### 3.2 Instalación automática
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
```

### 3.3 Instalación manual (si el paso anterior falla)
```bash
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-hardware-interface \
  ros-jazzy-controller-manager \
  ros-jazzy-moveit \
  ros-jazzy-moveit-configs-utils \
  ros-jazzy-rqt-joint-trajectory-controller
```

### 3.4 Optimización de Hardware (Script de Movimiento Suave)
Antes de compilar, es vital inyectar los ajustes oficiales para suprimir lags de UDP (C++) y unificar el bus URDF de la pinza para así eliminar inercias.
```bash
python3 ~/ros2_ws/src/burger_delivery/scripts/apply_kinova_smooth_movement.py
```
> Esto parcheará directamente tus archivos internos de `ros2_kortex`. Reduce el Timeout de red interno a `200ms` forzando un ciclo de control libre de micro vibraciones y habilitando MTC de forma segura.

---

## 4. Compilar

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> El flag `--symlink-install` evita recompilar al editar archivos Python o de lanzamiento (launch).

---

## 5. Conectar y Lanzar

### 5.1 Verificar conectividad
Antes de lanzar el driver, confirma que tu PC alcanza al robot:
```bash
ping 192.168.1.10
```
**Esperado:** Respuestas con tiempos `< 5ms` (Ethernet).

### 5.2 Lanzar el driver (sin control interactivo)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch kortex_bringup gen3.launch.py robot_ip:=192.168.1.10
```
Esto abre RViz con el modelo del robot y activa los controladores de hardware.

### 5.3 Lanzar con MoveIt (control interactivo) ⭐
Si quieres planificar trayectorias visualmente con flechas interactivas:
```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.1.10
```
En RViz:
1. Arrastra las flechas en la punta del gripper para fijar una meta.
2. Clic en **Plan** → visualiza la trayectoria.
3. Clic en **Execute** → el robot real se mueve.

---

## 6. Verificación del Driver

Con el driver corriendo (paso 5.2), abre **otra terminal** y ejecuta estos comandos en orden.

### 6.1 Verificar tópicos
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```
**Esperado:** Deben aparecer al menos:
```
/joint_states
/joint_trajectory_controller/follow_joint_trajectory/_action/status
/robot_description
/tf
/tf_static
```

### 6.2 Ver datos en tiempo real
```bash
ros2 topic echo /joint_states
```
**Esperado:** Posiciones, velocidades y esfuerzos de las 7 articulaciones actualizándose. `Ctrl+C` para salir.

### 6.3 Verificar controladores activos
```bash
ros2 control list_controllers
```
**Esperado:**
```
joint_state_broadcaster    [joint_state_broadcaster/JointStateBroadcaster]          active
joint_trajectory_controller [joint_trajectory_controller/JointTrajectoryController] active
```

Si `joint_trajectory_controller` aparece como `inactive`:
```bash
ros2 control switch_controllers --activate joint_trajectory_controller
```

### Resumen rápido

| # | Qué verificar | Comando | ✅ OK si... |
| :--- | :--- | :--- | :--- |
| 1 | Red | `ping 192.168.1.10` | Respuesta `< 5ms` |
| 2 | Tópicos | `ros2 topic list` | Se ven `/joint_states`, `/tf` |
| 3 | Controladores | `ros2 control list_controllers` | `joint_trajectory_controller` → `active` |

---

## 7. Pruebas de Movimiento

> ⚠️ **Precaución:** Asegúrate de que el espacio alrededor del robot esté despejado antes de enviar cualquier comando.

### Opción A: Deslizadores gráficos (rqt)

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller --force-discover
```

En la ventana:
1. **Controller Manager** → `/controller_manager`
2. **Controller** → `joint_trajectory_controller`
3. Clic en el **botón rojo** (se pondrá verde).
4. Aparecen **7 deslizadores** (uno por articulación).
5. Mueve un deslizador lentamente → el robot se mueve en tiempo real.

### Opción B: Comando directo desde terminal

**Mover a una posición de prueba** (valores en radianes, 5 segundos de transición):
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    points: [{
      positions: [0.0, 0.3, 3.14, 1.3, 0.0, 0.5, 0.0],
      time_from_start: {sec: 5, nanosec: 0}
    }]
  }}"
```

**Volver a posición home (cero):**
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    points: [{
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 5, nanosec: 0}
    }]
  }}"
```

---

## 8. Solución de Problemas

| Error | Causa | Solución |
| :--- | :--- | :--- |
| `Package 'kortex_bringup' not found` | Solo se compiló el driver | Ejecutar `colcon build --symlink-install` (paso 4) |
| `No module named 'moveit_configs_utils'` | Falta MoveIt | `sudo apt install ros-jazzy-moveit ros-jazzy-moveit-configs-utils` |
| `Findhardware_interface.cmake not found` | Faltan libs de control | Ejecutar paso 3.3 |
| `Package 'kortex_moveit_config' not found` | Nombre de paquete incorrecto | Usar: `kinova_gen3_7dof_robotiq_2f_85_moveit_config` |
| `Overrun detected! (1000 Hz)` | PC no mantiene frecuencia | Bajar a 100Hz en `ros2_controllers.yaml`. Ver [ANOMALIAS_HARDWARE.md](../network_setup/ANOMALIAS_HARDWARE.md) |
| `no plugin matching` (rqt) | Cache de plugins desactualizada | Añadir `--force-discover` al comando |
| `Falla de ping` | No hay ruta de red | Verificar cable Ethernet y subred `192.168.1.x` |
