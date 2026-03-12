# Diagnóstico de Anomalías: Kinova Kortex & ROS 2 Control

Este documento analiza los mensajes de advertencia y errores comunes observados durante la ejecución del driver `ros2_kortex`.

---

## 1. Advertencia: "Overrun detected! / Missed desired rate"

### El Problema
```text
[ros2_control_node-1] [WARN] [controller_manager]: Overrun detected! The controller manager missed its desired rate of 1000 Hz. The loop took 10.107760 ms.
```

### Análisis Técnico
*   **Frecuencia Objetivo:** El sistema intenta ejecutarse a 1000 Hz (ciclos de 1ms).
*   **Tiempo Real:** El bucle está tomando más de 10ms por ciclo.
*   **Causa Detectada:** El log indica que el `Write time` es el culpable (~10,000 us). Esto ocurre cuando la comunicación de red entre el PC y el brazo es lenta.

### Causas Probables
1.  **Entorno No-RealTime:** WSL (Windows Subsystem for Linux) y Ubuntu estándar no garantizan latencias menores a 1ms.
2.  **Conexión WiFi:** Si el robot está conectado vía WiFi, las fluctuaciones de latencia (jitter) causarán overruns constantes.
3.  **Carga del Agente:** El procesamiento de la API de Kinova es intensivo.

### Sugerencias
*   **Ethernet Directo:** Conectar el brazo mediante cable Ethernet categoría 6 directo al PC.
*   **Bajar Frecuencia:** Si la navegación o el control no requieren precisión quirúrgica, reducir la tasa de `1000Hz` a `100Hz` o `200Hz` en `ros2_controllers.yaml`.
*   **Ignorar en Pruebas:** Si el robot se mueve aceptablemente, estas advertencias pueden ignorarse durante la fase de desarrollo.

---

## 2. Error: "Segmentation fault (exit code -11)"

### El Problema
```text
[ros2_control_node-1] #0 Object "/home/roncanciovl/ros2_ws/build/kortex_driver/libkortex_driver.so"
[ros2_control_node-1] Segmentation fault (Address not mapped to object)
```

### Análisis Técnico
*   **Cuándo ocurre:** Únicamente al cerrar el nodo con `Ctrl+C`.
*   **Causa:** Una condición de carrera (race condition) en el driver de Kinova. Los objetos de la API de Kortex intentan liberar memoria mientras el `ResourceManager` de ROS ya los está destruyendo.

### Sugerencias
*   **Ignorar:** Este error ocurre durante el **apagado**. No afecta la ejecución, el movimiento ni la seguridad del robot mientras está operando. Es una anomalía estética del driver actual.

---

## 3. Resumen de Salud del Sistema

| Anomalía | Gravedad | Impacto | Acción |
| :--- | :--- | :--- | :--- |
| **Overrun (1000Hz)** | Media | Movimiento a tirones (jitter) | Usar Ethernet o bajar frecuencia. |
| **Segfault al Cerrar** | Muy Baja | Mensaje de error al salir | Ninguna (Es un bug del driver). |
| **Write Time > 10ms** | Alta | Latencia en el mando/joypad | Revisar infraestructura de red. |
