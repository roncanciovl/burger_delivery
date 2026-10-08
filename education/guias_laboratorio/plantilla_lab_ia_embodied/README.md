# Plantilla — Laboratorio IA 01: Embodied AI con Gemini Robotics-ER 2 y la cámara RGB-D del Kinova

Código base de la [Guía de Laboratorio IA 01](../GUIA_LAB_IA_01_EMBODIED_AI_GEMINI_ER2_KINOVA.md).
La plantilla ya resuelve la captura, la llamada a la API, el registro de la profundidad,
las evidencias y el CSV; usted completa las piezas de **IA y geometría** marcadas con `TODO`.

| Archivo | Para qué sirve | ¿Lo modifica? |
|---|---|---|
| `embodied_comun.py` | Núcleo común: calibración, prompts, llamada a Gemini, registro de profundidad, desproyección | **Sí: TODO 1 a 4** |
| `embodied_sin_ros.py` | **Opción A (sin ROS 2)**: RTSP directo + OpenCV. Subcomandos `calibrar`, `capturar`, `consultar`, `exito` | No |
| `embodied_ros2_nodo.py` | **Opción B (con ROS 2)**: nodo que consume `kinova_vision` y publica punto 3D y TF | **Sí: TODO 5** |
| `config_camara_kinova.json` | Intrínsecos de color y profundidad y extrínsecos entre ambos sensores | Sí, al calibrar |
| `leer_calibracion_kortex.py` | Lee la calibración real del robot con la Kortex API (lo ejecuta el docente una vez) | No |
| `tests/test_geometria.py` | Autoevaluación de los TODO 2 a 4 con una escena RGB-D sintética | No |

## Los TODO

| TODO | Función | Qué evalúa |
|---|---|---|
| 1 | `PROMPTS["propio"]` | Ingeniería de prompts: razonamiento espacial/estado del objeto con salida estructurada |
| 2 | `normalizado_a_pixel` | Formato `[y, x]` 0-1000 de Gemini → píxel `(u, v)` |
| 3 | `profundidad_robusta` | Estadística robusta sobre una ventana con huecos y reflejos |
| 4 | `desproyectar` | Modelo pinhole inverso: `(u, v, z)` → `(X, Y, Z)` |
| 5 | `NodoEmbodied._publicar` | (Solo Opción B) `PointStamped` y TF del objetivo |

Cuando termine los TODO 2 a 4 **todas** las pruebas deben pasar:

```bash
python3 -m pytest tests -q        # antes de completar: 9 fallan con NotImplementedError
```

## Instalación (Ubuntu 24.04)

```bash
# OpenCV del sistema (con GStreamer) y plugins RTSP, igual que en el Laboratorio 02
sudo apt update && sudo apt install -y python3-opencv python3-numpy python3-venv \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav

# Entorno virtual que SÍ ve el OpenCV del sistema
python3 -m venv --system-site-packages ~/venv_embodied
source ~/venv_embodied/bin/activate
pip install -r requirements.txt

# Comprobación: debe imprimir YES
python3 -c "import cv2; print([l for l in cv2.getBuildInformation().splitlines() if 'GStreamer' in l][0])"
```

No ejecute `pip install opencv-python` dentro del entorno: taparía el OpenCV del sistema con
uno sin GStreamer y el stream de profundidad dejaría de abrir.

## Clave de la API

```bash
export GEMINI_API_KEY="su-clave"     # https://aistudio.google.com/  -> Get API key
```

Nunca la escriba en el código ni la suba al repositorio. Sin clave (o sin cuota) todo el
pipeline se puede probar pasando la respuesta a mano con `--respuesta '[{"point": [500, 500], "label": "centro"}]'`.

## Uso rápido — Opción A (sin ROS 2)

```bash
cd ~/ros2_ws/src/burger_delivery/education/guias_laboratorio/plantilla_lab_ia_embodied

# En su turno frente al robot (un solo consumidor RTSP a la vez):
python3 embodied_sin_ros.py capturar --escena regla --nota "hoja A4 a 60 cm"
python3 embodied_sin_ros.py capturar --escena escena01 --nota "dos cajas, una con termo encima"

# Después, en cualquier lugar:
python3 embodied_sin_ros.py calibrar --escena regla --u1 <px> --u2 <px> --ancho-real-m 0.297 --distancia-m 0.60
python3 embodied_sin_ros.py consultar --escena escena01 --tarea puntos --objeto "la caja de hamburguesa" --grupo G03
python3 embodied_sin_ros.py consultar --escena escena01 --tarea propio --objeto "" --thinking high --grupo G03
python3 embodied_sin_ros.py consultar --escena escena01 --tarea puntos --objeto "la caja" --sin-registro --grupo G03
```

Cada consulta deja una imagen anotada `capturas/<escena>_<tarea>_resultado.png` y una fila en
`resultados/resultados.csv`, que son la evidencia del informe. Ambas carpetas están en
`.gitignore`: adjúntelas al informe, no al repositorio del curso.

## Uso rápido — Opción B (ROS 2)

La estación anfitriona lanza la cámara **una sola vez** para todos los grupos:

```bash
ros2 launch kinova_vision kinova_vision.launch.py device:=192.168.1.10 \
    max_color_pub_rate:=10.0 max_depth_pub_rate:=5.0
```

Cada grupo, con el mismo `ROS_DOMAIN_ID` y el perfil CycloneDDS del Laboratorio 02:

```bash
source /opt/ros/jazzy/setup.bash && source ~/venv_embodied/bin/activate
python3 embodied_ros2_nodo.py --ros-args -p grupo:=G03 -p thinking:=low

# En otra terminal:
ros2 topic pub --once /embodied/consulta std_msgs/msg/String \
  "{data: '{\"tarea\": \"puntos\", \"objeto\": \"la caja de hamburguesa\"}'}"
ros2 topic echo /embodied/respuesta --once
ros2 run image_view image_view --ros-args -r image:=/embodied/anotada -p image_transport:=compressed

# Guardar la escena actual con el mismo formato que la Opción A
ros2 topic pub --once /embodied/consulta std_msgs/msg/String \
  "{data: '{\"tarea\": \"guardar\", \"escena\": \"semantica1\"}'}"
```

Las escenas guardadas así se analizan después con `embodied_sin_ros.py consultar`, igual que
en la Opción A.

## Estado de validación

- Geometría, parser, registro de profundidad y CLI: verificados con la escena sintética de
  `tests/` (error 3D < 5 mm) y con respuestas de Gemini simuladas. La lógica del nodo ROS 2
  (hilo de consulta, `guardar`, `PointStamped`, TF y CSV) se verificó con `rclpy` simulado.
- Captura RTSP: reutiliza los pipelines de `scripts/test_kinova_camera.py`, validados en el
  Laboratorio 02 (color 1920×1080, profundidad 480×270 en mm).
- **Pendiente de validar en el robot real antes de la sesión**: la llamada real a
  `gemini-robotics-er-2-preview` (modelo en *preview*: nombre, cuotas y precios pueden
  cambiar), `leer_calibracion_kortex.py` y el nodo ROS 2 contra el driver en vivo.
