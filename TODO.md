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
- [x] **Experimento A/B del enlace de la estación del driver (WiFi vs Ethernet)**:
  - [x] Instrumental reproducible: [`benchmark_enlace_kinova.sh`](file:///home/roncanciovl/ros2_ws/src/burger_delivery/burger_kinova_connection/scripts/benchmark_enlace_kinova.sh) y [`analizar_enlace.py`](file:///home/roncanciovl/ros2_ws/src/burger_delivery/burger_kinova_connection/scripts/analizar_enlace.py).
  - [x] Medición insesgada sobre el robot real: por WiFi, `p99 = 60.12 ms`, intervalo máximo `3251 ms`, 132 overruns y **2 pérdidas de telemetría** en 120 s; por cable, `p99 = 10.61 ms`, máximo `20.63 ms`, 6 overruns y **0 pérdidas**.
  - [x] Registro completo: [EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md](file:///home/roncanciovl/ros2_ws/src/burger_delivery/burger_kinova_connection/docs/EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md).

---

## 🩺 4.1 Deuda técnica de diagnóstico (hallazgos sin cerrar)

- [ ] **⚠ Verificar si el parche de "movimiento suave" del Kortex sigue siendo necesario**
      ([`apply_kinova_smooth_movement.py`](file:///home/roncanciovl/ros2_ws/src/burger_delivery/scripts/apply_kinova_smooth_movement.py)):
  - **Origen:** proviene de un experimento previo **no documentado**, motivado por un temblor del brazo durante el movimiento. Se atribuyó a la latencia interna del router UDP de la API Kortex y se redujo su timeout a 200 ms.
  - **Por qué hay que revisarlo:** el experimento A/B del enlace demostró que, **por WiFi**, la estación del driver introducía un `p99` de 60 ms y huecos de hasta 3.25 s en el ciclo de 100 Hz. Un jitter de esa magnitud sobre una sesión cíclica es una causa candidata del mismo temblor. Es posible que el parche estuviera tratando el síntoma de una causa distinta.
  - [ ] **Confirmar primero que el parche siquiera se aplica.** Hoy **no hace nada**: busca `router_udp_realtime_.SetMessageTimeout(500);` en `kortex_driver/src/hardware_interface.cpp`, patrón que ya no existe en la versión clonada de `ros2_kortex`, y aun así imprime `[x] Parcheado`. Cualquier conclusión previa basada en "con parche / sin parche" puede estar comparando dos veces lo mismo.
  - [ ] Corregir el script para que **falle ruidosamente** si el patrón no aparece, en lugar de reportar éxito.
  - [ ] Reproducir el temblor de forma controlada **con la estación por cable**, ejecutando una trayectoria articular lenta y grabando `/joint_states` en MCAP.
  - [ ] Comparar A/B con y sin parche, ya sobre enlace cableado, midiendo la desviación por articulación respecto de la trayectoria comandada (no a ojo).
  - [ ] Según el resultado: retirar el parche, o documentarlo con evidencia en [`INSTALACION_KORTEX.md`](file:///home/roncanciovl/ros2_ws/src/burger_delivery/ros2_setup/INSTALACION_KORTEX.md) §3.4 explicando qué mide y qué corrige.
- [x] **✅ RESUELTO: el Kinova es de 6 GDL con pinza Robotiq 2F-85** (2026-09-09):
  - Confirmado por el propietario y por el driver (`Actuator count reported by robot is '6'`). Con `dof:=6` el brazo publica `joint_1..joint_6` más `robotiq_85_left_knuckle_joint`, todas reales, y se activan los tres controladores incluido el del gripper.
  - Causa de que no se pudiera usar antes: el commit local `b4ae524` en `ros2_kortex` quitó los parámetros de simulación pero dejó los bloques `<xacro:if value="${}">` huérfanos, rompiendo la descripción de 6 GDL con `error: invalid syntax (<expression>, line 0)`. Corregido completando el refactor. Los estudiantes clonan `ros2_kortex` limpio, así que no les afectaba.
  - Ajustada la configuración del package, el enunciado del proyecto y la guía de instalación. Documentado el paso obligatorio de `update_rate` sobre `6dof/config/ros2_controllers.yaml`, que antes sólo se había aplicado al 7dof.
- [ ] **Revisar el resto de la documentación que aún declara 7 GDL** (descriptiva, no operativa): `docs/architecture/ros_burger_delivery.md`, `docs/research/`, `education/guias_laboratorio/GUIA_LAB_01`, `GUIA_LAB_02`, `education/talleres/TALLER_URDF_TF.md`, `education/evidencias_abet/INSTRUMENTO_ABET_*`, `conceptos_core/visor_web_urdf.md`, `burger_description/README.md`, `ros2_setup/MOVEIT_Y_ROS2_CONTROL.md`.
- [ ] **Antiguo bloqueante de PA-08 (resuelto por lo anterior)**: quedaba pendiente determinar si el brazo era 6 o 7 GDL:
  - El driver reporta `Actuator count reported by robot is '6'` de forma consistente, mientras todo el proyecto se configura con `dof:=7`. La séptima casilla nunca se escribe: se observó `joint_7 = 1.12e+277` en una sesión y `0.0` en otra, siempre bit-idéntica dentro de cada sesión mientras el resto muestra ruido de encoder.
  - [ ] Consultar la interfaz web del robot (`http://192.168.1.10`, puerto 80 activo, requiere credenciales) para distinguir **Gen3 de 6 GDL** de **Gen3 de 7 GDL con el actuador 7 fuera de línea o en falla**.
  - [ ] Según el resultado: corregir `dof`, `expected_joints`, `safe_joint_positions_rad`, `joint_min_rad` y `joint_max_rad` —todos dimensionados a 7—, o abrir la reparación del actuador.
  - [ ] Revisar el enunciado del proyecto y el resto de la documentación, que declaran 7 GDL en todas partes.
  - Detalle en [ANOMALIAS_HARDWARE.md](file:///home/roncanciovl/ros2_ws/src/burger_delivery/network_setup/ANOMALIAS_HARDWARE.md) §3.
- [ ] **Cuantificar el residuo que aporta WSL2**: con el enlace ya por cable persisten 6 overruns en 120 s y el driver sigue avisando `Could not enable FIFO RT scheduling policy`. Repetir la rama `ethernet` en Linux nativo para separar la contribución de la capa WSL2 de la del enlace.

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