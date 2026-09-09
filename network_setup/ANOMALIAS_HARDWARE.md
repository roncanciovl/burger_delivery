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

## 3. Anomalía: `joint_7` fabricado — el robot reporta 6 actuadores con `dof:=7`

### El Problema

Con el brazo conectado y el driver lanzado con `dof:=7`, `/joint_states` publica siete
articulaciones, pero la séptima **no proviene del robot**:

```text
[KortexMultiInterfaceHardware]: Actuator count reported by robot is '6'
```

El driver expone siete interfaces y el brazo sólo alimenta seis. La casilla de `joint_7`
nunca se escribe y conserva lo que hubiera en memoria. Observado en dos sesiones
distintas del 2026-09-09, con el mismo recuento de actuadores en ambas:

| Sesión | `joint_7` position | `joint_7` effort |
| :--- | ---: | ---: |
| Primera | `1.1207224803148005e+277` | `0.0` |
| Segunda | `0.0` | `0.0` |

### Análisis Técnico

Un valor que cambia de sesión a sesión pero permanece **constante dentro de cada una** es
la firma de memoria sin inicializar, no de un fallo de sensor. Se confirma comparando el
ruido por articulación en seis muestras consecutivas con el robot quieto:

```text
  joint_2      joint_3      joint_7
 +0.137103794 +2.035091432 +0.000000000
 +0.137104327 +2.035091831 +0.000000000   <- joint_2 y joint_3 fluctúan
 +0.137103794 +2.035091831 +0.000000000   <- joint_7 es bit-idéntico
```

Las articulaciones reales muestran ruido de encoder; `joint_7` no varía **ni un bit**. Su
`effort` es exactamente `0.0` mientras el resto reporta pares de gravedad de hasta 11.9 N·m.

### Por qué es peligroso

El caso `1.12e+277` es escandaloso y se detecta. El caso `0.0` **no**: es finito, está
dentro de todos los límites articulares y supera cualquier validación razonable. El
monitor del proyecto llegó a reportar `telemetría saludable, 7/7 articulaciones` mientras
una de las siete era ficción.

Peor aún, `safe_trajectory_client` calcularía el desplazamiento de `joint_7` contra esa
posición inventada y podría enviar una meta para una articulación que no existe.

### Sugerencias

1. **Determinar primero qué es el robot.** Su interfaz web responde en el puerto 80
   (`http://192.168.1.10`) y requiere credenciales. Hay que distinguir dos casos con
   consecuencias muy distintas:
   - Es un **Gen3 de 6 GDL**, y toda la configuración del proyecto —que asume 7— está mal.
   - Es un **Gen3 de 7 GDL con el actuador 7 fuera de línea o en falla**, y es una
     reparación pendiente.
2. **Mientras tanto, no dar por buena la telemetría de `joint_7`.** No es un valor
   degradado: no existe.
3. **No ejecutar una meta articular de 7 elementos** hasta resolver el punto 1.
4. Revisar `dof` en `config/kinova_connection.yaml` y `expected_joints`,
   `safe_joint_positions_rad`, `joint_min_rad` y `joint_max_rad`, todos dimensionados a 7.

### Mitigación parcial ya implementada

`burger_kinova_connection` rechaza posiciones finitas pero físicamente imposibles
(`max_plausible_joint_rad`, 100 rad por defecto), lo que captura el caso `1.12e+277`.
**No captura el caso `0.0`**, y por diseño no puede: un cero es indistinguible de una
articulación legítimamente en el origen. La detección robusta requeriría comparar el
recuento de actuadores que anuncia el robot contra las articulaciones esperadas, o
detectar valores estancados bit a bit mientras el resto muestra ruido.

---

## 4. Resumen de Salud del Sistema

| Anomalía | Gravedad | Impacto | Acción |
| :--- | :--- | :--- | :--- |
| **Overrun (1000Hz)** | Media | Movimiento a tirones (jitter) | Usar Ethernet o bajar frecuencia. |
| **Segfault al Cerrar** | Muy Baja | Mensaje de error al salir | Ninguna (Es un bug del driver). |
| **Write Time > 10ms** | Alta | Latencia en el mando/joypad | Revisar infraestructura de red. |
