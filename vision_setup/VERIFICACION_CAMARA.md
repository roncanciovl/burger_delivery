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
Si tu brazo Kinova porta el módulo Infrarrojo o de Profundidad (útil en algoritmos 3D y nubes de puntos de ROS):
```bash
python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_camera.py --ip admin:admin@192.168.1.10 --stream depth
```

### Características de Testeo Añadidas:
* **Autocaptura**: Presionando la tecla `s` mientras ves la ventana, el script disparará una fotografía PNG limpia (útil para recolectar datasets de entrenamiento artificial de hamburguesas o calibraciones intrínsecas de tablero de ajedrez).
* **Gestión de Latencia Intensa**: El script intenta hacer passthrough mediante `GStreamer`, y si este falla, intenta forzar transporte por TCP y finalmente por UDP bajando el `frames_delay` a 0 para corroborar la red.

---

## Solución de Fallos
Si el script reporta `No se pudo conectar a rtsp://192.168.1.10/...`:
1. Verifica que la base administrativa del Kinova tenga habilitado el RTSP. Ingresa en tu navegador a la [pestaña Web App del Kinova](http://192.168.1.10) y verifica si la cámara está encendida.
2. Si el sistema dice "No OpenCV bindings" verifica: `pip3 install opencv-python cv-bridge`.
