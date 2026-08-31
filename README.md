# 🍔 Burger Delivery (Burger-Cell) — ROS 2 Jazzy

**Título Formal del Proyecto:**  
> *Burger-Cell: Framework Abierto de Robótica Colaborativa en ROS 2 con Manipulación Kinova Gen3, Percepción AprilTag e Integración de Modelos VLM*

[🇪🇸 Español](README.md) | [🇬🇧 English](README_en.md)  
[![DOI Concept](https://zenodo.org/badge/DOI/10.5281/zenodo.21809949.svg)](https://doi.org/10.5281/zenodo.21809949)
[![DOI v1.0.0](https://zenodo.org/badge/DOI/10.5281/zenodo.21809950.svg)](https://doi.org/10.5281/zenodo.21809950)

![Pipeline Pick and Place](pick_and_place_pipeline.svg)

> **Contexto del Proyecto**: Framework abierto de **investigación aplicada y docencia avanzada** en robótica colaborativa basado en **ROS 2 Jazzy**. Centraliza modelos, entornos de simulación y herramientas de control para una celda de trabajo donde un manipulador **Kinova Gen3** interactúa espacialmente con **robots móviles diferenciales** (TurtleBots). El ecosistema integra transformaciones dinámicas con `tf2`, localización visual por **AprilTags**, planificación de movimiento con **MoveIt 2**, protocolos **micro-ROS** optimizados para redes inalámbricas y modelos de visión-lenguaje (**Gemini Robotics-ER 1.6**) para razonamiento espacial 3D.

---

## 🏛️ Pilares del Repositorio

| Pilar | Enfoque | Contenido Principal |
|---|---|---|
| 🔬 **Investigación Aplicada** | Banco de pruebas (*Testbed*) para Embodied AI, percepción 3D zero-shot y determinismo en redes DDS/micro-ROS. | [`docs/research/`](docs/research/), [`docs/architecture/`](docs/architecture/), [`docs/manipulation/`](docs/manipulation/), [`network_setup/`](network_setup/) |
| 🎓 **Ecosistema Docente (Living Lab)** | Formación de talento y validación experimental en celdas colaborativas alineada a criterios **ABET**. | [`education/syllabus/`](education/syllabus/), [`education/talleres/`](education/talleres/), [`education/guias_laboratorio/`](education/guias_laboratorio/) |

---

## 🎯 Alcance Actual

Este repositorio funciona como el paquete de descripción (`burger_description`), plataforma experimental y centro de recursos:

- ✅ **Paquete ROS 2 compilable** con los URDF de la escena colaborativa y robots móviles.
- ✅ **Mallas vendorizadas** (Kinova Gen3, pinzas Robotiq 85 y carros diferenciales).
- ✅ **Entorno de lanzamiento (`launch`)** para TF dinámicos, `robot_state_publisher` y RViz2.
- ✅ **Monitor de Red Web Híbrido** (`network_setup/monitor_red/`) para telemetría de QoS, jitter WiFi y micro-ROS en tiempo real.
- ✅ **Módulo de Percepción IA VLM** para inferencia semántica y de-projection 3D con Gemini Robotics.
- ✅ **Scripts nativos de hardware** para estabilización inercial y mitigación de latencia en drivers C++.
- ✅ **Currículo académico y guías de laboratorio** listas para docencia y semilleros de investigación.

---

## 🚀 Inicio Rápido (Quick Start)

### Requisitos Previos
- Ubuntu 24.04 con **ROS 2 Jazzy** instalado. (Si no lo tienes, revisa `install_ros2.sh` y la guía en `ros2_setup/`).
- Herramienta `colcon` y paquetes de escritorio ROS 2 (`rviz2`, `joint_state_publisher_gui`, `xacro`).

### Compilación
Clona este repositorio dentro de la carpeta `src` de tu workspace (ej. `~/ros2_ws/src`):

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select burger_description
source install/setup.bash
```
*(Alternativa: ejecutar `./build_burger.sh`).*

### Ejecución y Visualización
Lanza la escena base en RViz2:

```bash
./lanzar_robot.sh
```

O manualmente:
```bash
ros2 launch burger_description display.launch.py
```

### 🌐 Monitor de Red y Telemetría ROS 2 (Dashboard Web)
Para supervisar dispositivos conectados al router (AX12), latencia/jitter WiFi, tráfico de dominios DDS y estado del agente micro-ROS (puerto 8888) en tiempo real:

```bash
bash network_setup/iniciar_monitor.sh
```
*(Abre automáticamente la interfaz en `http://127.0.0.1:8080`).*

Consulta la [**guía de todos los botones, configuraciones e indicadores del monitor**](network_setup/MONITOR_RED_CONTROLES_Y_CONFIGURACION.md) antes de interpretar dominios DDS o grabar benchmarks.

---

## 📂 Estructura del Proyecto

```text
burger_delivery/
├── burger_description/            # Paquete ROS 2 principal (URDFs, meshes, launch files)
│   ├── urdf/                      # Escena fija y descripciones de carritos
│   ├── launch/                    # display.launch.py con switch de modos
│   └── rviz/                      # Configuraciones de visualización RViz2
├── scripts/                       # Herramientas Python de validación física y parcheo
├── network_setup/                 # Monitor de red híbrido, QoS DDS, telemetría y configs
├── vision_setup/                  # Pipeline visual RTSP y arquitectura AprilTag
├── ros2_setup/                    # Guías de entorno Jazzy, Kortex y mapas de dependencias
│
├── docs/                          # 🔬 Documentación Técnica y de Investigación
│   ├── research/                  # Razonamiento espacial 3D con Gemini Robotics
│   ├── architecture/              # Arquitectura global y fundamentos ROS 2
│   └── manipulation/              # Control MoveIt 2, MTC y cinemática Kinova
│
├── education/                     # 🎓 Ecosistema Pedagógico (Living Lab / ABET)
│   ├── syllabus/                  # Programa curricular oficial y formato ABET
│   ├── talleres/                  # Talleres de CLI ROS 2 y URDF/TF2
│   ├── guias_laboratorio/         # Guías de laboratorio y plantillas editables
│   ├── proyectos_evaluables/      # Contrato de integración MoveIt & Delivery
│   └── metodologias/              # Bitácora de experticia técnica (SuperStudent)
│
├── visual/ & vendor/              # Recursos gráficos y mallas 3D vendorizadas
├── conceptos_core/                # Herramientas interactivas HTML y visualizadores
└── git-fundamentals/              # Módulos interactivos web de control de versiones
```

---

## 📚 Índice de Documentación (Categorizado)

### 🔬 1. Investigación y Razonamiento Espacial IA
- [⭐ **Plan Estratégico de Publicaciones y Releases Zenodo**](docs/research/PLAN_PUBLICACIONES_Y_RELEASES_ZENODO.md) *(Cronograma editorial 2026-2027, gestión de datasets y reproducibilidad)*
- **Ruta institucional vigente:** aval en trámite con **GIDAM (Categoría A, UMNG)**; pendiente de decisión y formalización. Ver la [propuesta de vinculación y aval académico a GIDAM](docs/research/PROPUESTA_VINCULACION_GIDAM.md).
- [**Estudio de Razonamiento Espacial 3D con Gemini Robotics**](docs/research/EXPERIMENTO_IA_LOCALIZACION_GEMINI.md) *(Localización zero-shot y affordances)*
- [**Propuesta de Integración Gemini ER**](ros2_setup/PROPUESTA_GEMINI_ER.md)

### 📐 2. Arquitectura General y Cinemática
- [**Guía Rápida de Uso**](burger_description/GUIA_DE_USO.md)
- [**Documento Técnico de Arquitectura (Burger Delivery)**](docs/architecture/ros_burger_delivery.md)
- [**Fundamentos de ROS 2 y Redes**](docs/architecture/ros.md)
- [**Visualizador Web URDF**](docs/architecture/VISUALIZAR_URDF_WEB.md)

### 🦾 3. Manipulación y Movimiento (Kinova & MoveIt 2)
- [**Marco Conceptual MoveIt 2**](docs/manipulation/MARCO_CONCEPTUAL_MOVEIT2.md)
- [**Mejoras de Movimiento MTC y Ghost Visualizer**](docs/manipulation/MEJORAS_MOVIMIENTO_KINOVA.md)
- [**Pruebas de Movimiento y Coordenadas Cartesianas CLI**](docs/manipulation/PRUEBAS_MOVIMIENTO.md)
- [**Tipos de Joints y Cinemática**](docs/manipulation/joints.md)
- [**Instalación del Stack Kortex**](ros2_setup/INSTALACION_KORTEX.md)

### 👁️ 4. Visión Computacional y Localización
- [⭐ **Arquitectura del Sistema de Localización AprilTag**](vision_setup/LOCALIZACION_APRILTAG.md) *(Árbol TF dinámico y cancelación de perspectiva)*
- [**Verificación y Diagnóstico de Cámara Nativa**](vision_setup/VERIFICACION_CAMARA.md)
- [**Aislamiento de Latencia Visual (TCP vs ROS 2)**](vision_setup/DIAGNOSTICO_RED_VISION.md)

### 🌐 5. Redes, micro-ROS y Telemetría
- [⭐ **Monitor de Red Híbrido Web**](network_setup/iniciar_monitor.sh)
- [**Controles y Configuración del Monitor de Red**](network_setup/MONITOR_RED_CONTROLES_Y_CONFIGURACION.md)
- [**Diagnóstico de Red ROS 2**](network_setup/DIAGNOSTICO_RED.md)
- [**Configuración de Red Recomendada**](network_setup/ROS2_NETWORK_CONFIG.md)
- [**Guía TP-Link Archer AX12**](network_setup/router_tplink_ax12_config.md)

### 🎓 6. Ecosistema Docente y Formación (ABET)
- [**Hub de Educación**](education/README.md)
- [**Syllabus del Curso (Formato ABET)**](education/syllabus/SYLLABUS_ROS2_ROBOTICA.md)
- [**Taller URDF y TF2**](education/talleres/TALLER_URDF_TF.md)
- [**Taller CLI ROS 2**](education/talleres/TALLER_ROS2_CLI.md)
- [**Proyecto Evaluable MoveIt 2 & Delivery**](education/proyectos_evaluables/PROYECTO_INTERMEDIO_MOVEIT2_DELIVERY.md)
- [**Bitácora Metodológica SuperStudent**](education/metodologias/SKILL_SUPERSTUDENT.md)

### 🐙 7. Control de Versiones & Trabajo en Equipo (Git Fundamentals)
- [⭐ **Portal de Fundamentos Git & GitHub**](git-fundamentals/index.html) *(Animaciones interactivas SVG)*
- [🏢 **Guía GitHub Organizations, Trabajo en Equipo y Evidencia ABET**](git-fundamentals/equipos_organizaciones_abet.html) *(Repositorios privados con historia compartida, ramas `feat/...`, pull requests y entrega identificada por SHA)*

---

## ⚙️ Modos de Lanzamiento (TF Dinámico)

| Modo | Comando de Launch | Cuándo usarlo |
|---|---|---|
| **Producción** (Por defecto) | `ros2 launch burger_description display.launch.py` | Operación real. Los carritos se acoplan dinámicamente al mapa cuando el nodo de localización AprilTag publica los TFs (`tag_mesa -> tag_carrito*`). |
| **Debug / Visualización** | `ros2 launch burger_description display.launch.py use_static_carts:=true` | Visualización en RViz sin cámara. Publica TFs estáticos para ver la escena completa. |

> ⚠️ **Advertencia de TF**: Nunca publiques `map -> car_base_link` si el sistema de AprilTag ya está publicando `tag_carrito -> car_base_link`. Romperá el árbol TF.

---

## 🛠️ Scripts de Depuración Física
En `scripts/`:
- `apply_kinova_smooth_movement.py`: Inyecta low-latency en drivers C++, suprimiendo vibraciones inerciales (jittering).
- `test_kinova_pose.py`: Valida coordenadas cartesianas [X,Y,Z] contra límites cinemáticos antes de planificar en MoveIt 2.
- `test_kinova_camera.py`: Extractor GStreamer/OpenCV para evaluar cámara RTSP sin sobrecarga de ROS.

---

## 🖼️ Diagramas del Sistema

### 📐 Localización Fiduciaria con AprilTags y TF2
Mapeo posicional relativo entre el manipulador Kinova Gen3, la mesa de trabajo y los marcos de referencia fiduciarios de los robots móviles:

![Esquema de Localización AprilTag](vision_localizacion_whiteboard.svg)

### 🤖 Navegación Móvil y Solicitudes de Entrega (Turtlebot)
Bucle de control y arquitectura de servicios/acciones para la interacción con los robots de entrega:

![Arquitectura de Navegación Turtlebot](turtlebot_nav_service.svg)

### 🌳 Árbol de Transformaciones Cinemáticas (TF2)
![Árbol de Transformaciones TF](tf_tree_diagram.svg)

### 🌐 Mapas de Red y Dependencias
- [Diagrama de Red ROS 2 y QoS](network_setup/ros_network_diagram.svg)
- [Mapa de Dependencias del Sistema](ros2_setup/MAPA_DEPENDENCIAS.svg)
- [Mapa de Dependencias del Módulo de IA VLM](ros2_setup/MAPA_DEPENDENCIAS_AI.svg)

---

## 📄 Citación y DOI

Este proyecto cuenta con registros DOI persistentes asignados en **Zenodo / DataCite**:

| Tipo de DOI | Identificador | Badge | Propósito |
|---|---|---|---|
| 🌐 **Concept DOI** | `10.5281/zenodo.21809949` | [![DOI Concept](https://zenodo.org/badge/DOI/10.5281/zenodo.21809949.svg)](https://doi.org/10.5281/zenodo.21809949) | Apunta a **todas las versiones** del repositorio (siempre la más reciente). |
| 🏷️ **Release DOI (v1.0.0)** | `10.5281/zenodo.21809950` | [![DOI v1.0.0](https://zenodo.org/badge/DOI/10.5281/zenodo.21809950.svg)](https://doi.org/10.5281/zenodo.21809950) | Snapshot inmutable de la versión **v1.0.0** (Software Científico Registrado). |

Si utilizas este entorno en tu investigación o cursos académicos, por favor cítalo como:

```bibtex
@software{burger_cell_2026,
  author = {Roncancio Velandia, Henry Antonio},
  title = {Burger-Cell: An Open-Source Heterogeneous ROS 2 Framework for Kinova Gen3 Manipulation, AprilTag Perception, and VLM-Guided Spatial Reasoning},
  year = {2026},
  publisher = {Zenodo},
  version = {1.0.0},
  doi = {10.5281/zenodo.21809950},
  url = {https://doi.org/10.5281/zenodo.21809950}
}
```
*(Consulta también [CITATION.cff](CITATION.cff) y [.zenodo.json](.zenodo.json)).*
