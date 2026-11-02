# GUÍA DE LABORATORIO 05 (CORTE 3): LA TRANSFORMACIÓN DEL DRON COMO TELEMETRÍA DEL CARRITO — TF2, MICRO-ROS Y CONTROL DE POSICIÓN EN LAZO CERRADO

---

| FACULTAD | PROGRAMA | ASIGNATURA | SEMESTRE | CÓDIGO GUÍA | REVISIÓN |
|:---|:---|:---|:---:|:---:|:---:|
| Facultad de Ingeniería | Ingeniería Mecatrónica | ROBOT OPERATING SYSTEM - ROS | VIII – IX | GL-AA-F-1 / LAB-05 (Corte 3) | 1.0 (2026-2) |

> [!IMPORTANT]
> **Actualización del Repositorio Privado del Equipo:**
> Antes de iniciar o continuar con este laboratorio, asegúrese de haber sincronizado su repositorio privado de equipo con los últimos cambios de la base del curso. Consulte la [Guía Oficial de Sincronización y Actualizaciones](../proyectos_evaluables/ACTUALIZACIONES_BASE_CORTE_1.md) (`education/proyectos_evaluables/ACTUALIZACIONES_BASE_CORTE_1.md`) para realizar este proceso correctamente.

---

## 1. CONTROL DE CAMBIOS

| Descripción del Cambio | Justificación | Fecha |
|---|---|:---:|
| Creación e implementación de la Guía de Laboratorio 05 | Integra dos equipos en un mismo lazo: el equipo del dron publica en la red ROS 2 la transformación `tag_mesa → tag_carrito1` y el equipo del carrito la usa como telemetría de un control de posición en un ESP32 con micro-ROS. La lectura de la TF se hace en dos fases: a través de un relevo en la PC (Fase A) y directamente desde `/tf` (Fase B). | 28/10/2026 |
| Publicación de TF en el localizador AprilTag | `scripts/apriltag_fixed_camera_localizer.py` acepta `publish_tf:=true` y publica `tag_mesa → tag_carrito1` con la estampa de la imagen. Su comportamiento por defecto, el del taller de localización, no cambia. | 28/10/2026 |

---

## 2. INTRODUCCIÓN

### 2.1. Un lazo con dos equipos

En el [taller de localización con AprilTag](../talleres/TALLER_LOCALIZACION_APRILTAG_KINOVA_MICROROS.md) la cámara del Kinova localizaba el carrito y un ESP32 calculaba su distancia a una meta. En el [Proyecto del Segundo Corte](../proyectos_evaluables/PROYECTO_CORTE_2_CONTROL_POSICION_ACCIONES.md) se diseñó la ley de control uniciclo en `turtlesim`. Este laboratorio une las dos cosas con un robot real y **dos equipos que sólo se comunican a través de ROS 2**:

| Equipo | Responsabilidad | Entrega a la red |
|---|---|---|
| **Dron** | Llevar la imagen de su cámara a ROS 2, detectar el `tag_mesa` y el tag del carrito, y publicar dónde está el carrito respecto a la mesa | La transformación `tag_mesa → tag_carrito1` en `/tf`, con la estampa de la imagen |
| **Carrito** | Leer esa transformación, convertirla en $(x, y, \theta)$ y cerrar un lazo de control de posición en el ESP32 con micro-ROS | Comandos aplicados, distancia a la meta, edad de la pose y tasa de mensajes recibidos |

Ninguno de los dos equipos ve el código del otro: lo único que comparten es un **contrato** (sección 7, Fase 0) con los nombres de los marcos, el dominio, la estampa y las unidades. Si el contrato está bien escrito, el carrito no necesita saber qué dron, qué cámara ni qué detector se usan.

```text
 Equipo del dron                                    Equipo del carrito
 ───────────────                                    ──────────────────
 cámara del dron ──► /dron/camara/image_raw/compressed
                          │
                          ▼
            apriltag_fixed_camera_localizer
              (publish_tf:=true)
                          │
                          ▼  /tf : tag_mesa → tag_carrito1  (estampa de la imagen)
        ┌─────────────────┴──────────────────────┐
        │ Fase A                                  │ Fase B
        ▼                                         ▼
 relevo_tf_pose (PC del carrito)          micro_ros_agent ──► ESP32 suscrito a /tf
        │ /burger_car_01/pose_mesa                 (filtra tag_mesa → tag_carrito1)
        ▼
 micro_ros_agent ──► ESP32 ──► control uniciclo ──► motores
```

### 2.2. Por qué una transformación y no una `Pose2D`

En el taller, el localizador publicaba `geometry_msgs/Pose2D`: tres números sin marco ni estampa. Una **transformación** (`geometry_msgs/TransformStamped` dentro de `/tf`) dice además **respecto a qué marco** está expresada la pose (`tag_mesa`), **de qué objeto** es (`tag_carrito1`) y **cuándo** se vio. Con eso:

- Cualquier nodo puede encadenarla con el resto del árbol: `table_link → tag_mesa → tag_carrito1 → car1_base_link` (ver [`LOCALIZACION_APRILTAG.md`](../../vision_setup/LOCALIZACION_APRILTAG.md)).
- El receptor puede **medir la edad** de la pose y descartarla si es vieja.
- La pose es independiente de dónde esté el dron: el localizador mide el carrito **en el plano del `tag_mesa`**, así que si el dron se mueve o vibra, la pose del carrito no cambia.

El precio es el tamaño: un `TransformStamped` lleva dos cadenas de texto, y `/tf` lleva las transformaciones de **todos** los publicadores del dominio. Para un ESP32 eso es memoria y ancho de banda. Por eso la práctica tiene dos fases:

| | Fase A — relevo en la PC | Fase B — `/tf` directo en el ESP32 |
|---|---|---|
| Qué suscribe el ESP32 | `/burger_car_01/pose_mesa` (`PoseStamped`), un solo publicador | `/tf` (`tf2_msgs/TFMessage`), todos los publicadores del dominio |
| Quién filtra | `relevo_tf_pose.py` con tf2 | El firmware compara `frame_id` y `child_frame_id` |
| Biblioteca micro-ROS | La estándar | Regenerada con `tf2_msgs` (no viene incluida) |
| Qué se aprende | El patrón habitual: el embebido recibe un tópico compacto | El costo real de leer TF en un microcontrolador |

### 2.3. La transformación que publica el dron

El localizador usa la homografía del taller: las cuatro esquinas del `tag_mesa`, de tamaño conocido, llevan cualquier píxel del plano de la mesa a metros. La TF que publica es:

| Campo | Valor |
|---|---|
| `header.frame_id` | `tag_mesa` (origen en el centro del sticker; eje $x$ de su esquina 0 a la 1; eje $z$ hacia arriba, fuera de la mesa) |
| `child_frame_id` | `tag_carrito1` |
| `header.stamp` | Estampa de la imagen que originó la detección |
| `translation` | $(x, y, h)$: posición del centro del tag del carrito; $h$ es `tag_height_m`, la altura medida del tag sobre la mesa |
| `rotation` | Giro $\theta$ alrededor de $z$: $q = (0,\ 0,\ \sin(\theta/2),\ \cos(\theta/2))$ |

El receptor recupera $\theta$ con la guiñada del cuaternión:

$$\theta = \operatorname{atan2}\big(2(q_w q_z + q_x q_y),\ 1 - 2(q_y^2 + q_z^2)\big)$$

> [!WARNING]
> **Paralaje.** La homografía es exacta para puntos **en el plano de la mesa**, pero el tag del carrito está a la altura de su techo. Con la cámara del dron inclinada, eso desplaza la pose medida. La Fase 2 lo mide; `tag_height_m` sólo fija la coordenada $z$ de la TF, no corrige $x$ ni $y$.

### 2.4. La ley de control en el ESP32

El firmware aplica la ley en dos fases del Proyecto del Segundo Corte, a 20 Hz, con la pose más reciente:

$$\rho = \sqrt{\Delta x^2 + \Delta y^2},\qquad \alpha = \operatorname{wrap}\big(\operatorname{atan2}(\Delta y, \Delta x) - \theta\big),\qquad \theta_e = \operatorname{wrap}(\theta_d - \theta)$$

| Fase | Condición | Comando |
|---|---|---|
| 1. Rumbo y avance | $\rho > 0.03$ m | $v = \operatorname{clip}(K_\rho\,\rho\,\cos\alpha,\ 0,\ v_{\max})$, $\ \omega = \operatorname{clip}(K_\alpha\,\alpha,\ \pm\omega_{\max})$ |
| 2. Giro en el sitio | $\rho \le 0.03$ m y $\lvert\theta_e\rvert > 0.10$ rad | $v = 0$, $\ \omega = \operatorname{clip}(K_\theta\,\theta_e,\ \pm\omega_{\max})$ |
| Meta alcanzada | $\rho \le 0.03$ m y $\lvert\theta_e\rvert \le 0.10$ rad | Motores detenidos |

Las velocidades de rueda salen de la cinemática diferencial, $v_{izq,der} = v \mp \omega L/2$, con $L = 0.1605$ m (separación de ruedas de `car1_apriltag.urdf`). Con $K_\rho = 0.8$, $K_\alpha = 2.0$, $v_{\max} = 0.10$ m/s y $\omega_{\max} = 1.2$ rad/s, una simulación del firmware con un uniciclo ideal y pose a 10 Hz llega de $(0, 0, 0)$ a $(0.30, 0.15, 0)$ m en 5 s.

**Dos salvaguardas no negociables** viven en el firmware: si no llega una pose nueva en `POSE_TIMEOUT_MS` = 300 ms, o si la pose llega con más de `EDAD_MAX_MS` = 500 ms de edad, los motores se detienen.

---

## 3. OBJETIVOS

### 3.1. Objetivo General

Cerrar un lazo de control de posición de un carrito con ESP32 y micro-ROS usando como única telemetría la transformación `tag_mesa → tag_carrito1` que otro equipo publica en la red ROS 2 desde la cámara de un dron, y comparar la lectura de esa transformación a través de un relevo en la PC con la lectura directa de `/tf` en el microcontrolador.

### 3.2. Objetivos Específicos

1. Acordar y documentar entre los dos equipos un contrato de integración: marcos, dominio, estampa, tópicos y unidades.
2. **(Dron)** Llevar la imagen de la cámara del dron a ROS 2 y publicar la TF `tag_mesa → tag_carrito1` con la estampa de la imagen, midiendo su exactitud y el error de paralaje.
3. **(Carrito)** Convertir la TF en $(x, y, \theta)$ y cerrar el lazo uniciclo en el ESP32, primero a través de un relevo tf2 (Fase A) y después suscribiendo `/tf` directamente (Fase B).
4. Medir la edad de la pose, la tasa de mensajes y el error final de llegada en las dos fases, y el efecto de otros publicadores de `/tf` sobre el ESP32.
5. Verificar experimentalmente que el carrito se detiene cuando la pose se pierde, envejece o se cae el agente.

---

## 4. DESCRIPCIÓN DE LA PRÁCTICA

Cada **pareja** está formada por un equipo de dron y un equipo de carrito, y trabaja en su propio dominio: `ROS_DOMAIN_ID = 40 + NN`, donde `NN` es el número de la pareja (pareja 01 → `41`).

| Fase | Contenido | Equipo | Duración |
|:---:|---|---|:---:|
| 0 | Entorno, dominio de la pareja y contrato de integración | Ambos | 15 min |
| 1 | Imagen del dron en ROS 2 y tags de la escena | Dron | 20 min |
| 2 | Publicar la TF `tag_mesa → tag_carrito1` y medir su exactitud | Dron | 30 min |
| 3 | Puesta a punto del carrito: motores y firmware | Carrito | 30 min (en paralelo con 1 y 2) |
| 4 | Lazo cerrado con relevo (Fase A) | Ambos | 30 min |
| 5 | Lazo cerrado con `/tf` directo (Fase B) y carga en `/tf` | Ambos | 30 min |
| 6 | Robustez: pérdida de pose, de agente y de visión | Ambos | 20 min |

La cámara del dron se usa en tres modos, de menor a mayor riesgo:

| Modo | Cámara | Para qué |
|---|---|---|
| **S · Simulado** | Ninguna: el localizador genera una pose en círculo | Probar la cadena TF → relevo → ESP32 con las ruedas levantadas |
| **F · Fija** | La cámara del dron (o el dron apagado) en un soporte sobre la mesa | Medir exactitud y cerrar el lazo sin vuelo |
| **V · Vuelo** | El dron en vuelo estacionario sobre la mesa | Repetir el lazo con la cámara en movimiento |

### 4.1. Resultados de Aprendizaje Evaluables (RAE) y Ponderación

| Criterio | RAE / Indicador de Desempeño | SO | Ponderación |
|---|---|:---:|:---:|
| **C1. Transformación `tag_mesa → tag_carrito1`** | **1.1.** Aplica principios matemáticos (transformaciones homogéneas, homografías y cuaterniones) para resolver problemas espaciales en robótica. | SO1 | 25% |
| **C2. Integración TF2 – micro-ROS** | **2.2.** Selecciona interfaces y mensajes respetando las restricciones de ancho de banda y memoria del ESP32, y justifica sus limitaciones. | SO2 | 20% |
| **C3. Control de posición y experimentación** | **6.1.** Diseña y ejecuta pruebas de lazo cerrado, midiendo la exactitud de la pose, su edad y frecuencia, y la respuesta del nodo embebido. | SO6 | 25% |
| **C4. Seguridad del vuelo, del carrito y de la celda** | **4.1.** Identifica riesgos de seguridad física, paradas de emergencia y celdas robóticas. | SO4 | 20% |
| **C5. Contrato, trazabilidad y trabajo entre equipos** | **3.1 - 3.3.** Elabora documentación técnica reproducible y registra evidencias.<br>**5.1.** Define roles técnicos y coordina la ejecución en equipo. | SO3 / SO5 | 10% |
| **Total** | | | **100%** |

Cada criterio se califica en cinco niveles sobre la escala de 0 a 500. **N3 es el umbral individual de logro** y la meta de cohorte es que al menos el 70 % de los estudiantes alcance N3 o superior en cada indicador.

| Nivel | Intervalo | Interpretación |
|:---:|:---:|---|
| **N5** | 475–500 | Excelente: evidencia completa y reproducible; explica cada diferencia entre fases con datos. |
| **N4** | 400–474 | Bueno: desempeño correcto con omisiones menores que no comprometen la validez de las mediciones. |
| **N3** | 300–399 | Aceptable: completa las fases con evidencia verificable (umbral de logro). |
| **N2** | 150–299 | Parcial: evidencia incompleta, contrato incumplido o lazo sin medir. |
| **N1** | 0–149 | No cumple: evidencia mínima, o vuelo o movimiento del carrito fuera del protocolo. |

> [!NOTE]
> Los indicadores 1.1, 2.2, 3.1 - 3.3, 4.1, 5.1 y 6.1 se citan como aparecen en los instrumentos de las prácticas anteriores. Verifíquelos contra el syllabus vigente.

---

## 5. MATERIALES Y EQUIPOS

### 5.1. Equipos del Laboratorio
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Mesa de la celda con el `tag_mesa` (AprilTag 36h11) instalado | 1 | Unidad |
| Router TP-Link AX12 (SSID `ros2`, subred `192.168.1.0/24`) | 1 | Unidad |
| Zona de vuelo delimitada sobre la mesa y protecciones de hélices | 1 | Juego |
| Soporte para la cámara del dron a la altura de vuelo (Modo F) | 1 | Por pareja |
| Cinta métrica, transportador y marcas adhesivas | 1 | Juego |

### 5.2. Equipos de los Estudiantes
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| **Dron:** dron con cámara y un medio para llevar su imagen a ROS 2 como `sensor_msgs/CompressedImage` | 1 | Por pareja |
| **Dron:** portátil con Ubuntu 24.04, ROS 2 Jazzy, `python3-opencv` y el repositorio del curso | 1 | Por pareja |
| **Carrito:** carrito diferencial con ESP32, puente H, dos motores y el AprilTag 36h11 en el techo | 1 | Por pareja |
| **Carrito:** portátil con ROS 2 Jazzy, `micro_ros_agent` y Arduino IDE con `micro_ros_arduino` (jazzy) | 1 | Por pareja |
| **Carrito, Fase B:** Docker para regenerar la biblioteca micro-ROS con `tf2_msgs`, o la biblioteca regenerada que entregue el docente | 1 | Por pareja |
| Instrumento de entrega del laboratorio (DOCX) con las Tablas 1 a 7, más la bolsa de la Fase 4 | 1 | Por equipo |

El código base está en [`plantilla_lab_05_dron_carrito/`](plantilla_lab_05_dron_carrito/README.md): el relevo `relevo_tf_pose.py` y el firmware `carrito_tf_control.ino`. El localizador es `scripts/apriltag_fixed_camera_localizer.py`.

---

## 6. SEGURIDAD EN EL LABORATORIO

> [!WARNING]
> 1. **Vuelo sólo en Modo V y con autorización.** El dron vuela únicamente dentro de la zona delimitada, con protecciones de hélices, a no más de 1.5 m sobre la mesa y con un piloto que tiene a mano el aterrizaje o el corte de motores. Nadie se ubica bajo el dron. Batería revisada antes de cada vuelo. Si el laboratorio no autoriza el vuelo, la práctica completa se hace en Modo F.
> 2. **El Kinova no participa.** El brazo permanece en reposo, sin driver en ningún dominio de las parejas y con su parada de emergencia accesible. El dron no vuela sobre el brazo.
> 3. **Ruedas levantadas primero.** El firmware se prueba con el carrito sobre un soporte (Fase 3 y Modo S) antes de apoyarlo en la mesa.
> 4. **El carrito no sale de la zona de trabajo.** Las metas quedan a más de 0.15 m de los bordes de la mesa; un integrante del equipo del carrito permanece junto a él para levantarlo. `V_MAX` no se aumenta por encima de 0.10 m/s.
> 5. **Las salvaguardas del firmware no se desactivan.** `POSE_TIMEOUT_MS`, `EDAD_MAX_MS` y el paro por pérdida del agente se verifican en la Fase 6 y no se modifican para «hacer pasar» una prueba.
> 6. **Un solo publicador de la TF por carrito.** Sólo el localizador del equipo del dron publica `tag_mesa → tag_carrito1` en el dominio de la pareja. Dos publicadores darían al carrito poses alternadas.

---

## 7. PROCEDIMIENTO EXPERIMENTAL

### Fase 0: Entorno, Dominio de la Pareja y Contrato de Integración

1. En **todas** las terminales de las dos PC, fije el dominio de la pareja y reinicie el daemon de la CLI:
   ```bash
   export ROS_DOMAIN_ID=$((40 + NN))       # NN = número de pareja, p. ej. 41 para la pareja 01
   timeout 5s ros2 daemon stop; ros2 daemon start
   ros2 node list                           # debe salir vacío: el dominio es sólo de la pareja
   ```
2. Midan juntos, con regla o calibrador, el **lado del cuadro negro** del `tag_mesa` (`tag_size_m`) y la **altura del tag del carrito** sobre la mesa (`tag_height_m`). Identifiquen la esquina 0 del `tag_mesa` y la dirección de la esquina 0 a la 1: es el eje $x$ del marco de medición.
3. Completen la **Tabla 1 (contrato)** y fírmenla los dos equipos. Ningún equipo cambia un valor del contrato sin avisar al otro.

---

### Fase 1: Imagen del Dron en ROS 2 y Tags de la Escena (equipo del dron)

1. Lleve la imagen de la cámara del dron a ROS 2 como `sensor_msgs/CompressedImage` en `/dron/camara/image_raw/compressed`. Cómo hacerlo depende del dron (driver del fabricante, `v4l2_camera` con una cámara USB, un flujo RTSP…). Si su driver publica la imagen sin comprimir, comprímala con `image_transport`:
   ```bash
   ros2 run image_transport republish --ros-args -p in_transport:=raw -p out_transport:=compressed \
       -r in:=/dron/camara/image_raw -r out/compressed:=/dron/camara/image_raw/compressed
   ```
2. Verifique el tópico y anote frecuencia y ancho de banda en la **Tabla 2**:
   ```bash
   ros2 topic info /dron/camara/image_raw/compressed
   ros2 topic hz /dron/camara/image_raw/compressed
   ros2 topic bw /dron/camara/image_raw/compressed
   ros2 topic echo /dron/camara/image_raw/compressed --once --field header
   ```
   **La estampa no puede ser cero:** es la que heredará la TF y con la que el carrito medirá la edad de la pose.
3. Lance el localizador sin referencia para ver qué IDs detecta (tape un tag con la mano para saber cuál es cuál):
   ```bash
   python3 scripts/apriltag_fixed_camera_localizer.py --ros-args -r __node:=localizador_dron \
       -p image_topic:=/dron/camara/image_raw/compressed -p tag_id:=0 -p publish_pose2d:=false
   ```

---

### Fase 2: Publicar la TF y Medir su Exactitud (equipo del dron)

1. Con los valores de la Tabla 1 (el ejemplo usa `tag_mesa` = 1, carrito = 5, lado de 0.10 m y tag del carrito a 0.15 m), publique la TF:
   ```bash
   python3 scripts/apriltag_fixed_camera_localizer.py --ros-args -r __node:=localizador_dron \
       -p image_topic:=/dron/camara/image_raw/compressed \
       -p tag_id:=5 -p reference_tag_id:=1 -p tag_size_m:=0.10 \
       -p publish_tf:=true -p publish_pose2d:=false \
       -p tf_parent_frame:=tag_mesa -p tf_child_frame:=tag_carrito1 -p tag_height_m:=0.15
   ```
   Al arrancar debe confirmar `Publicando en: /tf (tag_mesa -> tag_carrito1, z = 0.15 m)`.
2. Verifíquela desde **la PC del carrito**, que es quien la va a consumir:
   ```bash
   ros2 run tf2_ros tf2_echo tag_mesa tag_carrito1
   ros2 topic hz /tf
   ros2 run tf2_tools view_frames            # genera frames_<fecha>.pdf
   ```
   La frecuencia de `/tf` debe ser la de la imagen mientras ambos tags estén visibles. Si se tapa un tag, el localizador avisa `… no visible: no se publica pose` y la TF deja de actualizarse.
3. **Exactitud (Modo F).** Con la cámara en el soporte, ubique el tag del carrito en 3 posiciones marcadas con cinta respecto al `tag_mesa`. En cada una registre 20 lecturas de `tf2_echo` y calcule media y desviación de $x$, $y$ y $\theta$ (**Tabla 3**).
4. **Paralaje.** Repita una posición con el tag del carrito apoyado en la mesa y luego sobre el techo del carrito. La diferencia es el error de paralaje (**Tabla 3**, última fila).
5. **Modo S para el equipo del carrito.** Mientras el carrito se prepara, publique una TF de prueba sin cámara (círculo de 0.35 m a 10 Hz):
   ```bash
   python3 scripts/apriltag_fixed_camera_localizer.py --ros-args -r __node:=localizador_dron \
       -p simulated_mode:=true -p publish_tf:=true -p publish_pose2d:=false -p tag_height_m:=0.15
   ```

---

### Fase 3: Puesta a Punto del Carrito (equipo del carrito, en paralelo)

1. **Motores, con las ruedas levantadas.** Ajuste en `carrito_tf_control.ino` los pines del puente H. Con un sketch de prueba o con `aplicar(v, 0)`, encuentre el PWM mínimo que mueve las ruedas (`PWM_MIN`) y mida la velocidad de rueda con PWM máximo (`V_RUEDA_MAX`, en m/s, contando vueltas en 10 s). Compruebe que $v > 0$ mueve el carrito hacia su eje $x$, el del tag del carrito. Registre los valores en la **Tabla 4**.
2. **Firmware de la Fase A.** Deje `#define FUENTE_POSE FUENTE_RELEVO`, ajuste `AGENT_IP` (la IP de la PC del carrito, `ip -brief addr`), `DOMINIO` (el de la pareja), `FRAME_PADRE` y `FRAME_HIJO`, y cárguelo.
3. **Agente y relevo** en la PC del carrito, con el dominio de la pareja:
   ```bash
   # Terminal 1
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   # Terminal 2
   python3 education/guias_laboratorio/plantilla_lab_05_dron_carrito/relevo_tf_pose.py --ros-args \
       -p robot_namespace:=burger_car_01 -p parent_frame:=tag_mesa -p child_frame:=tag_carrito1
   ```
4. **Prueba de banco (Modo S, ruedas levantadas):**
   ```bash
   ros2 node list                                   # /burger_car_01/control_posicion
   ros2 topic hz /burger_car_01/pose_mesa
   ros2 topic echo /burger_car_01/cmd_aplicado
   ros2 topic echo /burger_car_01/edad_pose_ms
   ```
   Las ruedas deben girar persiguiendo la pose simulada. Pida al equipo del dron que detenga el localizador: las ruedas deben **detenerse en menos de 0.3 s**. Anote el tiempo en la **Tabla 4**.

> [!NOTE]
> **El dominio lo fija el ESP32.** `rclc_support_init_with_options()` pide `DOMINIO`; si la PC está en otro dominio, no verá los tópicos del carrito aunque el agente muestre la sesión.

---

### Fase 4: Lazo Cerrado con Relevo (Fase A)

1. Apoye el carrito en la mesa, dentro de la zona de trabajo, con el dron en **Modo F** y el localizador de la Fase 2 publicando.
2. Grabe la evidencia en la PC del carrito:
   ```bash
   ros2 bag record -s mcap -o lab05_parNN_faseA \
       --topics /tf /burger_car_01/pose_mesa /burger_car_01/meta /burger_car_01/distance_to_goal \
                /burger_car_01/cmd_aplicado /burger_car_01/edad_pose_ms /burger_car_01/mensajes_por_s
   ```
3. Envíe 3 metas dentro de la zona de trabajo, en el marco del `tag_mesa`:
   ```bash
   ros2 topic pub --once /burger_car_01/meta geometry_msgs/msg/Pose2D "{x: 0.30, y: 0.15, theta: 0.0}"
   ```
   Para cada meta registre en la **Tabla 5**: tiempo hasta que el LED queda fijo, error final medido **con cinta** (no con la TF), edad media de la pose y mensajes por segundo.
4. Repita una meta en **Modo V** si el vuelo está autorizado, y compare el error final con el de Modo F.

---

### Fase 5: Lazo Cerrado con `/tf` Directo (Fase B)

1. **Regenerar `micro_ros_arduino` con `tf2_msgs`.** La biblioteca precompilada de Jazzy no lo incluye (su archivo `built_packages` no lista `geometry2`). El docente puede entregarla ya regenerada; si no, en la PC del carrito:
   ```bash
   cd ~/Arduino/libraries
   git clone -b jazzy https://github.com/micro-ROS/micro_ros_arduino.git micro_ros_arduino_tf
   git clone -b jazzy --depth 1 https://github.com/ros2/geometry2.git /tmp/geometry2
   cp -r /tmp/geometry2/tf2_msgs micro_ros_arduino_tf/extras/library_generation/extra_packages/
   cd micro_ros_arduino_tf
   docker pull microros/micro_ros_static_library_builder:jazzy
   docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras \
       microros/micro_ros_static_library_builder:jazzy -p esp32
   ls src/tf2_msgs/msg/tf_message.h          # debe existir
   ```
   Deje en `~/Arduino/libraries` una sola biblioteca micro-ROS (mueva la estándar fuera de la carpeta) para que el IDE no mezcle las dos.
2. Cambie a `#define FUENTE_POSE FUENTE_TF`, compile y cargue. Detenga `relevo_tf_pose.py`: en esta fase no se usa.
3. Repita las 3 metas de la Fase 4 en Modo F y complete la **Tabla 5** (columna Fase B). Grabe la bolsa `lab05_parNN_faseB` con los mismos tópicos.
4. **Carga en `/tf`.** Simule a otro publicador de TF, como el driver del Kinova, que no tiene nada que ver con el carrito:
   ```bash
   ros2 topic pub -r 100 /tf tf2_msgs/msg/TFMessage \
       "{transforms: [{header: {frame_id: base_link}, child_frame_id: ruido, transform: {rotation: {w: 1.0}}}]}"
   ```
   Repita una meta y registre en la **Tabla 6** `mensajes_por_s`, la edad media de la pose y el error final, con y sin este publicador. Repita lo mismo en la Fase A (con el relevo) y compare.

> [!IMPORTANT]
> **`/tf`, no `tf`.** El nodo del ESP32 vive en el namespace `burger_car_01`: el nombre relativo `tf` se resolvería a `/burger_car_01/tf`, donde no publica nadie. Por eso el firmware suscribe el nombre absoluto `/tf`.

---

### Fase 6: Robustez — Pérdida de Pose, de Agente y de Visión

Con el carrito avanzando hacia una meta lejana (Fase A o B), provoque cada falla y mida con la bolsa o con `ros2 topic echo /burger_car_01/cmd_aplicado` el tiempo hasta que los motores se detienen (**Tabla 7**):

| N.º | Falla | Cómo provocarla | Debe ocurrir |
|:---:|---|---|---|
| 6.1 | Oclusión del tag del carrito | Tape el tag del carrito con la mano 5 s | Paro en menos de 0.3 s; retoma al destaparlo |
| 6.2 | Oclusión del `tag_mesa` | Tape el `tag_mesa` 5 s | Igual que 6.1 |
| 6.3 | Pose vieja | Fase B: detenga el localizador y publique la TF con estampa 0 (comando bajo la tabla) | `edad_pose_ms` muy grande y el carrito no se mueve: el ESP32 descarta poses con más de 500 ms |
| 6.4 | Pérdida del agente | `Ctrl+C` en el agente | Paro inmediato; reconexión sin reiniciar el ESP32 al relanzarlo |
| 6.5 | Pérdida de la cámara del dron | Modo V: el dron aterriza; Modo F: se retira la cámara del soporte | Paro en menos de 0.3 s |

Para la prueba 6.3, con el localizador detenido (es el único momento en que otro nodo publica la TF del carrito):
```bash
ros2 topic pub -r 10 /tf tf2_msgs/msg/TFMessage \
    "{transforms: [{header: {stamp: {sec: 0}, frame_id: tag_mesa}, child_frame_id: tag_carrito1, transform: {translation: {x: 0.10}, rotation: {w: 1.0}}}]}"
```

---

## 8. RESULTADOS DE LA PRÁCTICA

Las tablas se diligencian en el instrumento de entrega, [`INSTRUMENTO_ABET_LAB_05_TF_DRON_CARRITO_MICROROS.docx`](../evidencias_abet/INSTRUMENTO_ABET_LAB_05_TF_DRON_CARRITO_MICROROS.docx), que cada equipo entrega como `C3_L05_G<grupo>_<codigo1>_<codigo2>_v1.docx`. Las Tablas 1 y 5 a 7 son de la pareja: los dos equipos entregan los mismos valores. Con el instrumento se adjuntan la salida de `ros2 bag info` de las bolsas de las Fases 4 y 5 y el `frames_<fecha>.pdf` de la Fase 2. Aquí se reproducen como referencia para la sesión.

### Tabla 1: Contrato de Integración (Fase 0)
| Elemento | Valor acordado |
|---|---|
| Pareja `NN` y `ROS_DOMAIN_ID` | |
| ID y `tag_size_m` del `tag_mesa` | |
| ID del tag del carrito y `tag_height_m` | |
| Marco padre y marco hijo de la TF | `tag_mesa` → `tag_carrito1` |
| Tópico de la imagen del dron y estampa usada | |
| Tópico de la pose para el ESP32 (Fase A) y del de la meta | `/burger_car_01/pose_mesa`, `/burger_car_01/meta` |
| Edad máxima aceptada y timeout de pose | 300 ms (relevo), 500 ms y 300 ms (ESP32) |
| Firmas del equipo del dron y del equipo del carrito | |

### Tabla 2: Imagen del Dron (Fase 1)
| Ítem | Valor |
|---|---|
| Dron, cámara y cómo llega la imagen a ROS 2 | |
| Resolución, frecuencia (Hz) y ancho de banda de la imagen comprimida | |
| ¿La estampa de la imagen es distinta de cero? | |
| IDs detectados y cuál es el `tag_mesa` | |

### Tabla 3: Exactitud de la TF (Fase 2, Modo F)
| Posición | Real $(x, y, \theta)$ medida con cinta | Media de la TF $(x, y, \theta)$ | Desviación $(x, y, \theta)$ | Error de posición (mm) |
|:---:|---|---|---|:---:|
| P1 | | | | |
| P2 | | | | |
| P3 | | | | |
| Paralaje (P1, tag en la mesa frente a tag en el techo) | | | | |

### Tabla 4: Puesta a Punto del Carrito (Fase 3)
| Ítem | Valor |
|---|---|
| Puente H, pines y sentido de giro verificado | |
| `PWM_MIN` (zona muerta) y `V_RUEDA_MAX` (m/s) | |
| Prueba de banco: tiempo hasta el paro al detener la pose (ms) | |

### Tabla 5: Lazo Cerrado, Fase A frente a Fase B (Fases 4 y 5)
| Meta $(x, y, \theta)$ | Fase | Modo de cámara | Tiempo de llegada (s) | Error final con cinta (mm, °) | Edad media de la pose (ms) | Mensajes por s en el ESP32 |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| | A | F | | | | |
| | A | F | | | | |
| | A | F | | | | |
| | A | V | | | | |
| | B | F | | | | |
| | B | F | | | | |
| | B | F | | | | |

### Tabla 6: Carga en `/tf` (Fase 5)
| Fase | Publicador de ruido a 100 Hz | Mensajes por s en el ESP32 | Edad media de la pose (ms) | ¿Llegó a la meta? | Error final (mm) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| A | No | | | | |
| A | Sí | | | | |
| B | No | | | | |
| B | Sí | | | | |

### Tabla 7: Robustez (Fase 6)
| N.º | Falla | Tiempo hasta el paro (ms) | ¿Retomó solo? | Observación |
|:---:|---|:---:|:---:|---|
| 6.1 | Oclusión del tag del carrito | | | |
| 6.2 | Oclusión del `tag_mesa` | | | |
| 6.3 | Pose vieja | | | |
| 6.4 | Pérdida del agente | | | |
| 6.5 | Pérdida de la cámara del dron | | | |

---

## 9. ANÁLISIS DE RESULTADOS

1. **Análisis 1 (Exactitud y paralaje):** Con la Tabla 3, separe el error de la TF en sus fuentes: medición de `tag_size_m`, colocación del `tag_mesa`, paralaje por la altura del carrito y ruido del detector. ¿Cuál domina y cómo se reduciría?
2. **Análisis 2 (Fase A frente a Fase B):** Con las Tablas 5 y 6, compare la edad de la pose, la tasa de mensajes y el error final en las dos fases. ¿Qué gana y qué pierde el ESP32 al leer `/tf` directamente?
3. **Análisis 3 (Edad de la pose):** ¿De qué tramos se compone la edad que mide el ESP32 (captura, red del dron, detección, DDS, agente, WiFi)? ¿Cuáles cambian entre las fases y cuáles entre los modos F y V?
4. **Análisis 4 (Seguridad del lazo):** Con la Tabla 7, ¿qué falla tarda más en detener el carrito y por qué? ¿Qué distancia recorre el carrito a $v_{\max}$ durante ese tiempo?

---

## 10. CONCLUSIONES

1. Conclusión sobre el contrato de integración: qué tuvo que fijarse para que dos equipos que no comparten código cerraran un lazo de control.
2. Conclusión cuantitativa sobre la exactitud del lazo (Tablas 3 y 5).
3. Conclusión sobre cuándo conviene que un microcontrolador lea `/tf` directamente y cuándo a través de un relevo (Tablas 5 y 6).

---

## 11. PREGUNTAS PARA LA DISCUSIÓN

1. **Pregunta 1:** El localizador expresa el carrito en el marco del `tag_mesa`. ¿Por qué la pose del carrito no cambia cuando el dron se mueve, y qué pasaría si se publicara en el marco de la cámara del dron?
2. **Pregunta 2:** ¿Por qué la TF lleva la estampa de la imagen y no la hora en que el localizador terminó de calcularla? ¿Qué mediría el ESP32 si fuera la segunda?
3. **Pregunta 3:** El firmware reserva memoria para 4 transformaciones por `TFMessage` y cadenas de 32 caracteres. ¿Qué le pasa a un mensaje de `/tf` con 10 transformaciones, como los del driver del Kinova? ¿Y a uno con un `child_frame_id` de 40 caracteres?
4. **Pregunta 4:** ¿Qué haría falta para que el ESP32 midiera la edad de la pose sin `rmw_uros_sync_session()`? ¿Qué error introduciría no sincronizar el reloj?
5. **Pregunta 5:** Si dos parejas compartieran el dominio, ¿qué vería el ESP32 de cada carrito en `/tf` en la Fase B? ¿Qué cambiaría en el contrato para que pudieran compartirlo?

---

## 12. BIBLIOGRAFÍA

1. Foote, T. (2013). tf: The transform library. *IEEE International Conference on Technologies for Practical Robot Applications (TePRA)*, 1–6.
2. micro-ROS. *micro_ros_arduino (jazzy): precompiled library and custom packages.* https://github.com/micro-ROS/micro_ros_arduino
3. Olson, E. (2011). AprilTag: A robust and flexible visual fiducial system. *IEEE International Conference on Robotics and Automation (ICRA)*, 3400–3407.
4. Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2.ª ed.). MIT Press.
5. Proyecto `burger_delivery`. `TALLER_LOCALIZACION_APRILTAG_KINOVA_MICROROS.md`, `PROYECTO_CORTE_2_CONTROL_POSICION_ACCIONES.md` y `vision_setup/LOCALIZACION_APRILTAG.md`.

---

## 13. APROBACIÓN DE LA GUÍA DE LABORATORIO

| Elaborado por: | Revisado por: | Aprobado por: |
|:---:|:---:|:---:|
| **Ing. Henry Roncancio**<br>Docente Asignatura ROS | **Director de Programa**<br>Ingeniería Mecatrónica | **Decano(a)**<br>Facultad de Ingeniería |
