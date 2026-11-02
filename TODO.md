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
  - [ ] `burger_kinova_reference` Convertir en el paquete principal para el movimiento del robot 
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
  - [x] Instrumental reproducible: [`benchmark_enlace_kinova.sh`](file:///home/roncanciovl/ros2_ws/src/burger_delivery/burger_kinova_reference/scripts/benchmark_enlace_kinova.sh) y [`analizar_enlace.py`](file:///home/roncanciovl/ros2_ws/src/burger_delivery/burger_kinova_reference/scripts/analizar_enlace.py).
  - [x] Medición insesgada sobre el robot real: por WiFi, `p99 = 60.12 ms`, intervalo máximo `3251 ms`, 132 overruns y **2 pérdidas de telemetría** en 120 s; por cable, `p99 = 10.61 ms`, máximo `20.63 ms`, 6 overruns y **0 pérdidas**.
  - [x] Registro completo: [EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md](file:///home/roncanciovl/ros2_ws/src/burger_delivery/burger_kinova_reference/docs/EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md).

---

## 🩺 4.1 Deuda técnica de diagnóstico (hallazgos sin cerrar)

- [x] **✅ RESUELTO: el parche de "movimiento suave" no era necesario** (2026-09-10):
  - El temblor venía del enlace de red de la estación, no de la latencia del router UDP: con cable el p99 de `/joint_states` cae de 60.12 ms a 10.61 ms. La prueba de movimiento con el anfitrión por cable salió limpia.
  - Aquel script además **no aplicaba nada**: buscaba un patrón que ya no existe e imprimía "Parcheado" igual.
  - Reemplazado por [`aplicar_compatibilidad_kortex.py`](file:///home/roncanciovl/ros2_ws/src/burger_delivery/scripts/aplicar_compatibilidad_kortex.py), que sí hace lo necesario, **verifica el resultado** y falla ruidosamente. Validado sobre un clon prístino de upstream: deja los URDF generándose en 6 y 7 GDL, y es idempotente.
  - Confirmado que **ninguno de estos ajustes toca el árbol TF**: con `use_internal_bus_gripper_comm` en true y en false, links y joints son idénticos.
- [ ] **Cuantificar el residuo que aporta WSL2**: con el enlace ya por cable persisten 6 overruns en 120 s y el driver sigue avisando `Could not enable FIFO RT scheduling policy`. Repetir la rama `ethernet` en Linux nativo para separar la contribución de la capa WSL2 de la del enlace.
- [x] **✅ Re-vendorizar `burger_description` a 6 GDL** (2026-10-14): `scripts/revendorizar_gen3_6dof.py --vision` escribe la cadena oficial de 6 GDL con la pinza Robotiq 2F-85 y el módulo de visión (`gen3_camera_link`, `gen3_camera_color_frame`, `gen3_camera_depth_frame`), y regenera la copia del visor web `visual/burger_delivery_gen3.urdf`. `TALLER_URDF_TF.md` §11 enseña la cadena de la cámara para el Lab IA 01. Historial de la tarea:
  - El URDF vendorizado modela un Gen3 de **siete** articulaciones (`gen3_joint_1`…`gen3_joint_7`, cadena `gen3_half_arm_1_link`/`gen3_half_arm_2_link`, mallas de `arms/gen3/7dof/`), mientras el brazo real es de 6 GDL (cadena `bicep_link`).
  - **No afecta al RViz que abre el driver**: ese usa `kortex_description` generado en vivo con `dof:=6` y sí refleja la posición real. El modelo vendorizado sólo lo consume `burger_description/launch/display.launch.py`, un visor sin robot.
  - Mientras tanto, los cuatro documentos que lo describen (`burger_description/README.md`, `conceptos_core/visor_web_urdf.md`, `docs/architecture/ros_burger_delivery.md`, `education/talleres/TALLER_URDF_TF.md`) llevan un aviso explicando la diferencia entre los dos modelos.
  - [x] Sustituir descripción y mallas por las de `arms/gen3/6dof/`. La escena (mesa, tags, `burger_grip_frame`) no cambia: cuelga de frames que siguen existiendo.
  - [x] Revisar `TALLER_URDF_TF.md` y su formato ABET (v1.1, evidencia E7 de la cámara).
  - [ ] Validar en el robot real el eslabón `end_effector_link → camera_link` (el driver no carga `vision:=true`) y el sentido de los ejes de `camera_color_frame` frente a la imagen. Procedimiento: `TALLER_URDF_TF.md` §11, Ejercicio 3 (predecir con TF2 el píxel de un marcador medido sobre la mesa y compararlo con la imagen). Según el URDF de Kinova el marco es óptico (Z hacia donde mira, X a la derecha, Y hacia abajo); falta confirmarlo.
- [ ] **Pendientes de la revisión de talleres (2026-09-15)**, documentados en `TROUBLESHOOTING.md` §4:
  - [x] `scripts/flight_recorder_telemetry_demo.py` publica ahora los mismos nombres que el robot real: `joint_1..joint_6` y `robotiq_85_left_knuckle_joint` (2026-10-14). Cada `JointState` crece 24 bytes; el taller de rosbag explica el efecto estimado en sus tamaños de referencia.
  - [ ] Volver a medir con ROS 2 Jazzy los tamaños de referencia del taller de rosbag (MCAP+Zstd y SQLite3) con el emulador nuevo.
  - [x] `scripts/apriltag_fixed_camera_localizer.py`: el modo real ya se suscribe a `/camera/color/image_raw/compressed`, es compatible con OpenCV 4.6 (antes el detector quedaba en `None` en silencio) y mide en el plano de `tag_mesa` por homografía. Validado con escena sintética en perspectiva (error ≈ 3 mm y 0.55°).
  - [ ] Validar el localizador con la cámara real y los tags de la mesa: requiere tags 36h11 en la mesa (en la verificación del 2026-09-15 no había tags instalados; la cámara sí transmitía, 1920×1080 a ≈ 29 FPS) y que la anfitriona lleve el brazo a la pose de observación. Documentar IDs y tamaño de los tags físicos, que no figuran en el repositorio.
  - [ ] `display.launch.py` publica `/joint_states`, `/tf` y `/robot_description` sin namespace: hoy sólo lo protege la regla de dominio de los talleres. Valorar un namespace o un aviso al arrancar.

---

## 🎓 5. Ecosistema Docente y Living Lab (ABET)

- [ ] **Ampliación de Guías de Laboratorio (`education/guias_laboratorio/`)**:
  - [ ] Guía Lab 04: Modelado de robots y árboles TF con URDF/XACRO.
  - [x] Guía Lab 05: Localización visual con AprilTags y cancelación de perspectiva → publicada como [Guía 05](education/guias_laboratorio/GUIA_LAB_05_TF_DRON_CARRITO_MICROROS.md) (2026-10-28): el dron publica `tag_mesa → tag_carrito1` con `apriltag_fixed_camera_localizer.py --ros-args -p publish_tf:=true` y el carrito la usa en un ESP32 con micro-ROS (relevo tf2 y `/tf` directo), con su instrumento ABET.
    - [ ] Compilar `carrito_tf_control.ino` para el ESP32 con `micro_ros_arduino` (jazzy) y probarlo con un carrito: sólo se revisó contra las cabeceras de micro-ROS y se simuló la ley de control en la PC.
    - [ ] Regenerar `micro_ros_arduino` con `tf2_msgs` (la biblioteca precompilada no lo trae) y comprobar la Fase B; dejar la biblioteca regenerada disponible para los estudiantes.
    - [ ] Probar `ros2 run image_transport republish` con `in_transport`/`out_transport` en Jazzy y llevar a ROS 2 la cámara de cada dron del curso.
  - [ ] Guía Lab 06: Planificación de trayectorias con MoveIt 2.
  - [x] Guía Lab 07: Razonamiento espacial con IA multimodal (Gemini) → publicada como [Guía IA 01](education/guias_laboratorio/GUIA_LAB_IA_01_EMBODIED_AI_GEMINI_ER2_KINOVA.md) para la asignatura de IA, con `gemini-robotics-er-2-preview`, ruta sin ROS 2 y plantilla de código/informe.
    - [ ] Validar en el robot real antes de la sesión: llamada a la API, `leer_calibracion_kortex.py` y el nodo ROS 2 contra `kinova_vision`.
    - [ ] Corregir o reemplazar `default_color_calib_1920x1080.ini` de `kinova_vision` (declara `cy = 96.97 px`): afecta a todo nodo ROS que desproyecte con `/camera/color/camera_info` a 1080p.
- [ ] **Integración Continua (CI/CD)**:
  - [ ] Configurar GitHub Actions para validación automática de URDFs (`xmllint`, `check_urdf`) en cada Pull Request.
  - [ ] Linteo automático de código Python y scripts bash.
