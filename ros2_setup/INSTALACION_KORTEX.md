# Guía de Instalación: Driver Kinova Kortex (ROS 2 Jazzy)

Esta guía detalla los pasos para instalar el controlador oficial de Kinova (`ros2_kortex`) desde el código fuente, optimizado para Ubuntu 24.04 y ROS 2 Jazzy.

---

## 1. Requisitos Previos

Antes de comenzar, asegúrate de tener instalado ROS 2 Jazzy Desktop y las herramientas de desarrollo básicas.

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools git python3-colcon-common-extensions
```

---

## 2. Preparación del Workspace

Si aún no tienes un workspace, créalo. Si ya tienes `~/ros2_ws`, simplemente entra en la carpeta `src`.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Clonar el Repositorio
Descarga el código fuente oficial de Kinova compatible con ROS 2:

```bash
git clone https://github.com/Kinovarobotics/ros2_kortex.git
```

---

## 3. Instalación de Dependencias

Este es el paso más crítico. El driver necesita componentes de `ros2_control` que no siempre vienen instalados por defecto.

### Paso A: Inicializar rosdep
Si es la primera vez que usas ROS en esta máquina:

```bash
sudo rosdep init
rosdep update
```

### Paso B: Instalación automática
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
```

### Paso C: Instalación manual (Si falla el anterior)
Si encuentras errores de "hardware_interface" o similares durante la compilación, instala estos paquetes manualmente:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-hardware-interface \
  ros-jazzy-controller-manager \
  ros-jazzy-moveit \
  ros-jazzy-moveit-configs-utils
```

> **Importante:** `ros-jazzy-moveit` y `ros-jazzy-moveit-configs-utils` son necesarios para el control interactivo del brazo desde RViz (planificación de trayectorias, flechas interactivas, etc.).

---

## 4. Compilación

Para asegurar que todas las dependencias cruzadas (especialmente entre el driver y los modelos 3D) se resuelvan correctamente, se recomienda compilar el workspace completo:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

*Nota: El flag `--symlink-install` evita tener que recompilar cada vez que edites un archivo de Python o de lanzamiento (launch).*

---

## 5. Uso y Verificación

### Cargar el entorno
```bash
source ~/ros2_ws/install/setup.bash
```

### Lanzar el Driver (Solo hardware, sin control interactivo)
El paquete de lanzamiento básico es **`kortex_bringup`**. Sustituye la IP por la de tu robot:

```bash
# Para Kinova Gen3
ros2 launch kortex_bringup gen3.launch.py robot_ip:=192.168.1.10

# Para Kinova Gen3 Lite
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=192.168.1.10
```

### Lanzar con Control Interactivo (MoveIt + RViz) ⭐ Recomendado
Este comando abre RViz con el panel de **Motion Planning**, flechas interactivas en el gripper y planificación automática de trayectorias:

```bash
# Para Kinova Gen3 7DOF con Robotiq 2F-85
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.1.10

# Para Gen3 6DOF
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.1.10

# Para Gen3 Lite
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=192.168.1.10
```

**Cómo usar MoveIt en RViz:**
1.  Arrastra las flechas/esferas interactivas en la punta del robot para fijar una meta.
2.  Haz clic en **`Plan`** para ver la trayectoria calculada.
3.  Haz clic en **`Execute`** para que el robot real se mueva.
---

## 6. Verificación del Driver (Paso a Paso)

Una vez que el driver esté corriendo (ver sección 5), sigue estos pasos en orden para verificar que todo funciona correctamente. **Cada paso se ejecuta en una terminal nueva** (no cierres la terminal del driver).

### Nivel 1: Verificar Conexión al Robot

Antes de lanzar nada, confirma que tu PC puede alcanzar al robot:

```bash
ping 192.168.1.10
```

**Resultado esperado:** Respuestas con tiempos bajos (`< 5ms` por Ethernet).
**Si falla:** Revisa que estés en la misma subred (`192.168.1.x`) y que el cable Ethernet esté conectado.

### Nivel 2: Verificar que el Driver Publica Tópicos

Con el driver corriendo (`gen3.launch.py`), abre una **segunda terminal** y ejecuta:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

**Resultado esperado:** Deberías ver al menos estos tópicos:
```
/joint_states
/joint_trajectory_controller/follow_joint_trajectory/_action/feedback
/joint_trajectory_controller/follow_joint_trajectory/_action/status
/robot_description
/tf
/tf_static
```

Para ver los **datos en tiempo real** de las articulaciones:
```bash
ros2 topic echo /joint_states
```

**Resultado esperado:** Verás las posiciones, velocidades y esfuerzos actuales de las 7 articulaciones actualizándose continuamente. Presiona `Ctrl+C` para salir.

### Nivel 3: Verificar Controladores Activos

```bash
ros2 control list_controllers
```

**Resultado esperado:**
```
joint_state_broadcaster [joint_state_broadcaster/JointStateBroadcaster] active
joint_trajectory_controller [joint_trajectory_controller/JointTrajectoryController] active
```

Si `joint_trajectory_controller` aparece como `inactive`, actívalo con:
```bash
ros2 control switch_controllers --activate joint_trajectory_controller
```

### Nivel 4: Prueba de Movimiento Básico

#### Opción A: Deslizadores Gráficos (rqt) — Más visual

Instala la herramienta (solo la primera vez):
```bash
sudo apt install ros-jazzy-rqt-joint-trajectory-controller
```

Ejecútala:
```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

En la ventana que se abre:
1.  **Controller Manager** → Selecciona `/controller_manager` del dropdown.
2.  **Controller** → Selecciona `joint_trajectory_controller`.
3.  Haz clic en el **botón rojo** (se pondrá **verde** = activo).
4.  Aparecerán **7 deslizadores**, uno por cada articulación del Gen3.
5.  **Mueve un deslizador lentamente** → El robot real se moverá en tiempo real.

> ⚠️ **Precaución:** Mueve los deslizadores lentamente y asegúrate de que el espacio alrededor del robot esté despejado.

#### Opción B: Comando Directo desde Terminal — Sin instalar nada

Envía una posición objetivo a las 7 articulaciones (valores en **radianes**):

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

**Qué hace:** Mueve el brazo a la posición indicada en **5 segundos**.

Para **volver a la posición de inicio** (home):
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

### Resumen de Verificación

| Nivel | Qué verifica | Comando clave | ✅ Resultado OK |
| :--- | :--- | :--- | :--- |
| **1** | Conectividad de red | `ping 192.168.1.10` | Respuesta `< 5ms` |
| **2** | Driver publica datos | `ros2 topic list` | Se ven `/joint_states` y `/tf` |
| **3** | Controladores activos | `ros2 control list_controllers` | `joint_trajectory_controller` → `active` |
| **4** | Robot se mueve | `rqt` o `ros2 action send_goal` | El brazo cambia de posición |


## Solución de Problemas Comunes

*   **Error: `Package 'kortex_bringup' not found`**: Ocurre si solo compilaste el driver. Ejecuta el paso **4** para compilar todo el workspace.
*   **Error: `No module named 'moveit_configs_utils'`**: Falta MoveIt. Instálalo con: `sudo apt install ros-jazzy-moveit ros-jazzy-moveit-configs-utils`.
*   **Error: `Findhardware_interface.cmake` not found**: Te faltan las librerías de control. Ejecuta los comandos del **Paso 3.C**.
*   **Error: `Package 'kortex_moveit_config' not found`**: No existe como paquete. Usa el nombre específico del modelo, ej: `kinova_gen3_7dof_robotiq_2f_85_moveit_config`.
*   **Advertencia: `Overrun detected! (1000 Hz)`**: El PC no logra mantener la frecuencia. Se puede bajar a 100Hz editando `ros2_controllers.yaml`. Ver [ANOMALIAS_HARDWARE.md](../network_setup/ANOMALIAS_HARDWARE.md).
*   **Falla de conexión (Ping)**: Verifica que tu PC esté en la misma subred que el robot. Intenta `ping 192.168.1.10`.
