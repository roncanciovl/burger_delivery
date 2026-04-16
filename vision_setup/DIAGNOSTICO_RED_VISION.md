# Diagnóstico de Conectividad de la Cámara Kinova

A diferencia de los comandos mecánicos del robot que se enrutan típicamente por protocolos ligeros (RTPS/UDP de bajo peso en ROS 2), el flujo de una matriz óptica de visión requiere una **alta demanda de red TCP y un ancho de banda considerable**.

Antes de intentar correr algoritmos visuales complejos o abrir RViz (que demanda RAM), debes usar el protocolo de verificación de red que detallamos aquí para asegurar que la cámara transmite fotogramas sanos sin latencia ni cuellos de botella.

---

## 1. Verificación en Capa de Red (Ping)
Desde la computadora (Linux/Ubuntu), envía paquetes al sistema cerrado de la cámara (dentro del IP de fábrica del brazo Kinova).

```bash
ping 192.168.1.10 -c 10
```
**Regla de oro Logística:**
* `time < 5 ms`: Perfecto, óptimo para MTC y *AprilTags*.
* `time > 20 ms` o oscilaciones: Hay cuellos de botella en tu Switch o Router Wi-Fi. Las imágenes por *RTSP* llegarán entrecortadas (Drop Frames).
* `Destination Host Unreachable`: La interfaz de red de Ubuntu (eth0/wlan0) no está en la misma subred (ej: 192.168.1.x) que el robot.

---

## 2. Diagnóstico de Passthrough Visual (RTSP)
Para comprobar que el servidor óptico interno del brazo no está petrificado, usamos el script unificado de pruebas sin lanzar ROS 2:

```bash
python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_camera.py --ip 192.168.1.10 --stream color
```

Si este testeo falla, verifica los cortafuegos internos de tu sistema:
1. El protocolo utiliza por defecto **FFMPEG TCP Streamer**.
2. Verifica mediante la GUI que la cámara no requiera credenciales o modifica el script para usar *RTSP con Password* `rtsp://admin:admin@192.168.1.10/color`.

---

## 3. Comprobando Carga sobre Tópicos ROS 2
Si tu red y script RTSP pasaron el test satisfactoriamente, significa que el entorno base conectivo es ideal. Es tiempo de usar `ros2_kortex_vision`. Una vez lanzado tu archivo `robot.launch.py`, comprueba que Ubuntu esté decodificando las matrices.

```bash
ros2 topic hz /camera/color/image_raw
```
Deberías ver una constancia cercana a los `15_hz` a `30_hz`. Si el valor es de *1 hz*, la latencia de tu decodificador sobre ROS 2 podría colapsar a MoveIt. Utiliza la documentación en `VERIFICACION_CAMARA.md` para aislar el diagnóstico del flujo usando solo la Terminal y FFMPEG nativo en ese caso.
