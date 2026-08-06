# 📋 Roadmap & Oportunidades de Mejora — Burger-Cell (Burger Delivery)

Este documento centraliza las tareas pendientes, oportunidades de mejora identificadas y el plan de desarrollo para consolidar **Burger-Cell** como un producto de investigación y docencia de alto impacto.

---

## 🔬 1. Demostrador de Investigación (MVP Burger-Cell Bench)

- [ ] **Nodo de Percepción VLM en ROS 2 (`gemini_spatial_reasoning_node.py`)**:
  - [ ] Implementar captura asíncrona de frames desde la cámara del Kinova (RTSP/tópico ROS).
  - [ ] Integrar inferencia zero-shot con SDK `google-genai` (modelo `gemini-robotics-er-1.6-preview`).
  - [ ] Implementar desproyección cartesiana 3D (`image_geometry`) para calcular coordenadas `(X, Y, Z)`.
  - [ ] Publicar TF dinámico `/target_burger_box_frame` en el árbol de transformaciones.
- [ ] **Módulo de Benchmarking Comparativo**:
  - [ ] Desarrollar script de benchmark cuantitativo para ejecutar 10+ pruebas repetitivas.
  - [ ] Comparar error euclidiano 3D (mm) de Gemini vs AprilTag (Ground Truth).
  - [ ] Medir latencia de inferencia VLM (ms) y sobrecarga de ancho de banda.
  - [ ] Exportación automática de resultados a formato `.csv` y gráficas para publicaciones.
- [ ] **Borrador de Paper Científico / Extended Abstract**:
  - [ ] Redactar propuesta de paper académico (*"Burger-Cell: A Heterogeneous ROS 2 Testbed for VLM-Guided Spatial Manipulation and Real-Time QoS Analysis"*).
  - [ ] Estructurar metodología, resultados experimentales y discusión.

---

## 📦 2. Modularización de Paquetes ROS 2

- [ ] **Separación del Workspace en Paquetes Especializados**:
  - [ ] `burger_description`: Mantener exclusivamente URDFs, mallas 3D y configuraciones RViz.
  - [ ] `burger_perception`: Nodos de detección AprilTag y razonamiento espacial con IA.
  - [ ] `burger_control`: Scripts de MoveIt 2 Task Constructor (MTC) para pick & place colaborativo.
  - [ ] `burger_telemetry`: Dashboard web, agente micro-ROS y recolector de telemetría DDS.
  - [ ] `burger_bringup`: Launch files centralizados con argumentos de modo (`simulation:=true`, `use_vlm:=true`).

---

## 🚗 3. Navegación Móvil y Coordinación Multi-Robot (AGVs / TurtleBots)

- [ ] **Stack de Navegación Nav2**:
  - [ ] Pruebas de navegación autónoma y mapeo (SLAM) en el entorno de la celda de trabajo.
  - [ ] Definición de zonas de parada seguras (*delivery slots*) sincronizadas con el brazo Kinova.
- [ ] **Servidor de Acciones y Contratos de Integración**:
  - [ ] Implementar action server `/car/prepare_delivery_pose` con feedback de llegada.
  - [ ] Acople dinámico de TFs del carrito móvil (`tag_mesa -> tag_carrito -> car_base_link`) durante el movimiento.

---

## 🌐 4. Telemetría de Red y Determinismo DDS / micro-ROS

- [x] **Protocolo Experimental Formal de Telemetría QoS**:
  - [x] Documento metodológico y matemático: [EXPERIMENTO_QOS_TELEMETRIA.md](file:///home/roncanciovl/ros2_ws/src/burger_delivery/docs/research/EXPERIMENTO_QOS_TELEMETRIA.md).
  - [x] Definición de 3 escenarios de red controlados (Línea Base, Carga Multi-Robot, Estrés Severo).
- [x] **Módulo de Benchmarking en el Monitor de Red (`network_setup/monitor_red`)**:
  - [x] Grabación y muestreo de RTT, Jitter, Pérdida de Paquetes y Ancho de Banda DDS a 1 Hz.
  - [x] Endpoints API REST `/api/benchmark/start`, `/stop`, `/status`, `/download`.
  - [x] Panel visual interactivo en la UI con selectores de escenario, estado REC y descarga directa de CSV.
  - [x] Script de análisis estadístico y generación de figuras para papers: [analyze_telemetry_benchmark.py](file:///home/roncanciovl/ros2_ws/src/burger_delivery/scripts/analyze_telemetry_benchmark.py).
- [ ] **Inyección de Tráfico y Estrés de Red**:
  - [ ] Scripts para emular degradación de enlace WiFi (pérdida de paquetes, jitter, latencia artificial con `tc/netem`).
  - [ ] Evaluar estabilidad de trayectorias articulares del Kinova bajo congestión de red.

---

## 🎓 5. Ecosistema Docente y Living Lab (ABET)

- [ ] **Ampliación de Guías de Laboratorio (`education/guias_laboratorio/`)**:
  - [ ] Guía Lab 02: Modelado de robots y árboles TF con URDF/XACRO.
  - [ ] Guía Lab 03: Localización visual con AprilTags y cancelación de perspectiva.
  - [ ] Guía Lab 04: Planificación de trayectorias con MoveIt 2.
  - [ ] Guía Lab 05: Razonamiento espacial con IA multimodal (Gemini).
- [ ] **Integración Continua (CI/CD)**:
  - [ ] Configurar GitHub Actions para validación automática de URDFs (`xmllint`, `check_urdf`) en cada Pull Request.
  - [ ] Linteo automático de código Python y scripts bash.