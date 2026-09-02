# PRUEBAS DE VISIÓN Y CÁMARA (KINOVA)

Se ha integrado un módulo de pruebas directas para el sensor óptico del Kinova. Este visor es una herramienta crítica de diagnóstico porque **conecta directamente vía RTSP/RTP (Protocolo de Transmisión de Tiempo Real)** hacia la IP del brazo, de la misma forma que lo hacen los sistemas de seguridad, saltándose por completo a ROS 2. 

Esto ayuda enormemente a comprobar si el sensor del hardware de cámara está operativo antes de integrarlo a un pipeline complejo de Computer Vision y *AprilTags*.

**Directorio del ejecutable**: `scripts/test_kinova_camera.py`

---

## 1. Visualizar Stream Principal a Color
Para probar la transmisión estandar RGB del brazo, corre en cualquier terminal (no requiere hacer un `source` de MoveIt):

```bash
python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_camera.py --ip admin:admin@192.168.1.10 --stream color
```
*(Puedes presionar la tecla `q` sobre la ventana para cerrar o cancelar).*

## 2. Visualizar Flujo de Profundidad (Depth Mappings)

> [!IMPORTANT]
> **Antes de la primera prueba de profundidad, instala los plugins de GStreamer.**
> Ubuntu 24.04 trae solo `gstreamer1.0-plugins-base`, que **no** incluye
> `rtspsrc` ni `rtpgstdepay`:
> ```bash
> sudo apt update && sudo apt install -y \
>     gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
>     gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
> gst-inspect-1.0 rtspsrc && gst-inspect-1.0 rtpgstdepay   # verificación
> ```

Si tu brazo Kinova porta el módulo Infrarrojo o de Profundidad (útil en algoritmos 3D y nubes de puntos de ROS):
```bash
python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_camera.py --ip 192.168.1.10 --stream depth
```

### Por qué `color` y `depth` no se abren igual
El módulo de visión expone dos servicios RTSP con **formatos distintos**:

| Stream | Transporte real | Backend que sirve | Formato entregado |
|---|---|---|---|
| `rtsp://<ip>/color` | RTP + **H.264** estándar | FFMPEG **o** GStreamer | `BGR` 8 bits |
| `rtsp://<ip>/depth` | RTP + payload **X-GST** de GStreamer | **Solo** GStreamer (`rtpgstdepay`) | `GRAY16_LE` 480x270, **milímetros** |

FFmpeg no implementa el depayloader `X-GST`, así que en `--stream depth` el
script **omite FFMPEG a propósito** y va directo a GStreamer. Los antiguos
mensajes `[WARN] FFMPEG TCP no funcionó` en profundidad no indicaban una falla
de red ni del sensor.

Como la profundidad son 16 bits métricos y no una imagen RGB, el visor la
colorea con `COLORMAP_JET` (negro = píxel sin dato) y rotula la distancia del
píxel central. Con la tecla `s` se guardan **dos** archivos:
`kinova_capture_N.png` (vista coloreada, para el informe) y
`kinova_capture_N_raw16.png` (16 bits en mm, el que sirve para medir).
Usa `--max-depth 2000` para reescalar el color en escenas cercanas.

### Características de Testeo Añadidas:
* **Autocaptura**: Presionando la tecla `s` mientras ves la ventana, el script disparará una fotografía PNG limpia (útil para recolectar datasets de entrenamiento artificial de hamburguesas o calibraciones intrínsecas de tablero de ajedrez).
* **Gestión de Latencia Intensa**: En `color` el script prueba FFMPEG/TCP, luego FFMPEG/UDP y finalmente GStreamer, con `nobuffer` y `low_delay`. En `depth` va directo a GStreamer, que es el único backend capaz de leer ese stream.
* **Diagnóstico por capas previo**: Antes de abrir el stream comprueba el puerto TCP 554 del robot y la presencia de los plugins GStreamer necesarios, indicando el comando `apt` exacto si faltan.

---

## Solución de Fallos

### A. `No URI handler implemented for "rtsp"` (falla más reportada)
Traza típica en los equipos de los estudiantes:
```
GStreamer warning: your GStreamer installation is missing a required plugin
Embedded video playback halted; module uridecodebin0 reported:
    No URI handler implemented for "rtsp".
CAP_IMAGES: can't find starting number (in the name of file): rtsp://192.168.1.10/depth
```
**Causa:** OpenCV se compiló *con* soporte GStreamer (`GStreamer: YES`), pero en
el sistema solo está `gstreamer1.0-plugins-base`. Sin `rtspsrc` no hay manejador
para el esquema `rtsp://`. Al agotarse los backends, OpenCV cae al lector
`CAP_IMAGES` e intenta leer la URL como una secuencia de imágenes numeradas: de
ahí el último error, que despista.

**Solución:**
```bash
sudo apt update && sudo apt install -y \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
```
Comprobación rápida del estado de un equipo:
```bash
python3 -c "import cv2; print([l for l in cv2.getBuildInformation().splitlines() if 'GStreamer' in l])"
ls /usr/lib/x86_64-linux-gnu/gstreamer-1.0/ | wc -l   # ~31 = solo 'base'; >100 = completo
gst-inspect-1.0 rtspsrc && gst-inspect-1.0 rtpgstdepay
```

### B. La ventana de profundidad se ve casi negra
Ocurría con el pipeline antiguo, que forzaba `videoconvert ! video/x-raw,format=BGR`
sobre datos `GRAY16_LE`: la conversión descarta el byte bajo, y una escena a
1000 mm quedaba en el nivel 3 de 255. El visor actual conserva los 16 bits y
colorea en Python. Si aún se ve plano, ajusta el rango con `--max-depth`.

### C. El script se queda colgado sin imprimir nada
`rtspsrc` sin *timeout* bloquea indefinidamente. El pipeline actual fija
`timeout=5000000 tcp-timeout=5000000` (5 s) y `protocols=tcp`, de modo que falla
con mensaje en lugar de congelar la terminal.

### D. `No se pudo conectar a rtsp://192.168.1.10/...`
1. El script hace primero una prueba de puerto TCP 554. Si esa prueba falla, el
   problema es de **capa de red**: revisa `ping 192.168.1.10` y que la NIC
   cableada esté en `192.168.1.0/24`.
2. Verifica que la cámara esté habilitada en la [Web App del Kinova](http://192.168.1.10).
3. Si el sistema dice "No OpenCV bindings": `sudo apt install -y python3-opencv`
   (preferible a `pip install opencv-python` en un entorno con ROS 2 Jazzy).
4. Para saltarse los chequeos previos: `--skip-checks`.
