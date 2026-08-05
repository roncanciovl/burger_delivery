# 🍔 Burger Delivery (ROS 2 Jazzy)

[🇪🇸 Español](README.md) | [🇬🇧 English](README_en.md)

![Pick and Place Pipeline](pick_and_place_pipeline.svg)

> **Project Context**: This repository centralizes the documentation, models, and resources used in an advanced collaborative robotics environment. The goal is to simulate and control a work cell where a **Kinova Gen3** manipulator spatially interacts to deliver trays to **differential mobile robots** (TurtleBot style). The entire ecosystem is coordinated using `tf2`, **AprilTags** visual localization, the **MoveIt 2** planning framework, and **micro-ROS** protocols, heavily optimized for high-density wireless environments.

---

## 🎯 Current Scope

This repository currently serves as the description package (`burger_description`) and the central hub for documentation and configuration. It contains:

- ✅ A buildable ROS 2 package with URDFs of the delivery scene.
- ✅ Vendored 3D meshes (Kinova Gen3, Robotiq grippers, and mobile carts).
- ✅ A `launch` environment for dynamic TFs, `robot_state_publisher`, and RViz2.
- ✅ Extensive architecture, installation, and network diagnostics documentation.
- ✅ Advanced native scripts for hardware stabilization and latency patching.

*Note: It does not yet contain a complete "out-of-the-box" autonomous operation stack (no final compilable navigation or business logic packages here, but rather the architectural and geometric foundation to integrate them).*

---

## 🚀 Quick Start

### Prerequisites
- Ubuntu with **ROS 2 Jazzy** installed. (If you don't have it, check `install_ros2.sh` and the guide in `ros2_setup/`).
- `colcon` build tool and ROS 2 desktop packages (RViz2, joint_state_publisher_gui).

### Compilation
Make sure to clone this repository inside the `src` folder of your workspace (e.g., `~/ros2_ws/src`):

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select burger_description
source install/setup.bash
```
*(Alternative: use the included `./build_burger.sh` script).*

### Execution & Visualization
You can launch the base simulation in RViz using the quick script:

```bash
./lanzar_robot.sh
```

Or run it manually:
```bash
ros2 launch burger_description display.launch.py
```

### 🌐 Network & ROS 2 Telemetry Monitor (Web Dashboard)
To monitor devices connected to the router (AX12), WiFi latency/jitter, DDS domain traffic, and Micro-ROS agent status (port 8888) in real-time:

```bash
bash network_setup/iniciar_monitor.sh
```
*(Automatically opens the dashboard at `http://localhost:8080`).*

---

## 📂 Project Structure

- `burger_description/`: Main ROS 2 package (URDFs, visual/collision meshes, launch files).
  - `urdf/delivery_scene_fixed.urdf`: Main static scene.
  - `visual/` & `vendor/`: 3D meshes and third-party resources (Kinova).
- `scripts/`: Vital Python tools for patching Kinova latencies and CLI physical unit testing.
- `network_setup/`: Advanced network diagnostics, WiFi 6/AX12 configuration, telemetry tools, and the **Hybrid Web Network Monitor** (`monitor_red/`).
- `vision_setup/`: RTSP camera connectivity guides and AprilTags localization.
- `ros2_setup/`: Jazzy environment setup notes, scripts, and configurations.

---

## 📚 Documentation Index (Categorized)

We have organized the extensive technical documentation to facilitate your learning and setup process:

### 📖 1. General Guides & Architecture
- [Quick Usage Guide](burger_description/GUIA_DE_USO.md)
- [Technical Architecture Document (Burger Delivery)](ros_burger_delivery.md)
- [Evaluable Project MoveIt2 & Delivery](PROYECTO_INTERMEDIO_MOVEIT2_DELIVERY.md)

### 🤖 2. Manipulation & Motion (Kinova & MoveIt)
- [MTC Motion Improvements and Ghost Visualizer](MEJORAS_MOVIMIENTO_KINOVA.md)
- [Motion Tests and CLI Cartesian Coordinates](PRUEBAS_MOVIMIENTO.md)
- [Kortex Stack Installation](ros2_setup/INSTALACION_KORTEX.md)

### 👁️ 3. Computer Vision & Localization
- [⭐ AprilTag Localization System Architecture](vision_setup/LOCALIZACION_APRILTAG.md) *(Highly recommended to understand the TF tree)*
- [Native Camera Verification & Diagnostics](vision_setup/VERIFICACION_CAMARA.md)
- [Visual Latency Isolation (TCP vs ROS 2)](vision_setup/DIAGNOSTICO_RED_VISION.md)

### 🌐 4. Networking, Setup & micro-ROS
- [⭐ Hybrid Network Monitor (ROS 2 & Router AX12 Web Dashboard)](network_setup/iniciar_monitor.sh) *(Launcher: `bash network_setup/iniciar_monitor.sh`)*
- [ROS 2 Network Diagnostics](network_setup/DIAGNOSTICO_RED.md)
- [Recommended Network Configuration](network_setup/ROS2_NETWORK_CONFIG.md)
- [TP-Link Archer AX12 Configuration Guide](network_setup/router_tplink_ax12_config.md)
- [ROS 2 Jazzy Environment Verification](ros2_setup/verificar_ros2.md)
- **Diagnostic Scripts**: Find tools like `test_ros2_network.sh`, `diagnostico_wifi.sh`, `diagnostico_microros.sh`, `analisis_trafico_ros2.sh` and more in the `network_setup/` folder.

---

## ⚙️ Advanced Usage Notes (TFs and Modes)

The behavior of the mobile carts in the scene depends on the **AprilTags** system.

| Mode | Launch Command | When to use it |
|---|---|---|
| **Production** (Default) | `ros2 launch burger_description display.launch.py` | For real operation. The carts dynamically attach to the map when the AprilTag localization node publishes the TFs (`tag_mesa -> tag_carrito*`). |
| **Debug / Visual** | `ros2 launch burger_description display.launch.py use_static_carts:=true` | Useful if the localization node is not running. It forces the publication of static TFs to visualize where the carts should be. |

> ⚠️ **TF Warning**: Never publish `map -> car_base_link` if the AprilTag system is already publishing `tag_carrito -> car_base_link`. It will break the TF tree.

---

## 🛠️ Hardware and Physical Debugging Scripts
In `scripts/`, you will find powerful patching tools that do not affect the standard simulation:
- `apply_kinova_smooth_movement.py`: Smart patcher that injects "Low Latency" options to the hardware's C++ drivers, correcting inertial jumps and vibrations (jittering).
- `test_kinova_pose.py`: Spatial validator to check if an [X,Y,Z] coordinate breaks kinematics before testing it in MoveIt.
- `test_kinova_camera.py`: GStreamer/OpenCV extractor to evaluate the camera without ROS overhead.

---

## 🖼️ Additional Architecture Diagrams

### Localization Scheme (AprilTag)
Relative positional mapping between the arm, the table, and the static reference frame of the tags.
![AprilTag Localization Scheme](vision_localizacion_whiteboard.svg)

### Navigation and Actions (Turtlebot)
Control loop for the delivery robot's interaction and requests.
![Turtlebot Nav2 Architecture](turtlebot_nav_service.svg)

### Transformation Tree (TF)
*(To generate a real-time one: with the launch running, execute `ros2 run tf2_tools view_frames` in another terminal).*
![Project TF Tree](tf_tree_diagram.svg)

### Network and Software Dependencies
- [ROS 2 Network Diagram](network_setup/ros_network_diagram.svg)
- [Dependencies Map](ros2_setup/MAPA_DEPENDENCIAS.svg)
- [AI Dependencies Map](ros2_setup/MAPA_DEPENDENCIAS_AI.svg)
