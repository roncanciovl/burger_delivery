# 🎓 Guía Paso a Paso: Localización Visual 2D (Cámara Fija) y Navegación con micro-ROS

> [!IMPORTANT]
> **Actualización del Repositorio Privado del Equipo:**
> Antes de iniciar o continuar con este taller, asegúrese de haber sincronizado su repositorio privado de equipo con los últimos cambios de la base del curso. Consulte la [Guía Oficial de Sincronización y Actualizaciones](../proyectos_evaluables/ACTUALIZACIONES_BASE_CORTE_1.md) para realizar este proceso correctamente.

¡Bienvenido! En este taller práctico aprenderás a implementar un sistema de **localización visual 2D** utilizando una **cámara en posición fija** (la cámara del Kinova estacionada en su posición *Home* o una cámara cenital sobre la mesa) para detectar marcadores visuales (**AprilTags / ArUco**) y transmitir la pose del robot en tiempo real hacia nodos embebidos en **micro-ROS (ESP32)**.

Este taller está diseñado para trabajar con los conceptos esenciales de **ROS 2 del Primer Corte** (Nodos, Tópicos, Publicadores, Suscriptores y micro-ROS), **sin requerir el uso de árboles de transformadas TF2**, los cuales se abordarán formalmente en el Segundo Corte.

---

## 🎯 Resultado de Aprendizaje Evaluable (RAE)

**RAE 1 (Primer Corte):** *Comprender la arquitectura distribuida, redes, comunicación técnica y experimentación en el ecosistema ROS 2.*

### Indicadores ABET asociados:
* **Indicador 1.1 (SO1 - Resolución de problemas):** Formula y conecta el flujo de datos entre un nodo de percepción visual en la PC y un nodo de control embebido en el microcontrolador.
* **Indicador 2.2 (SO2 - Diseño de ingeniería):** Selecciona interfaces y mensajes compactos (`geometry_msgs/msg/Pose2D`) respetando las restricciones de ancho de banda y memoria del ESP32.
* **Indicador 6.1 (SO6 - Experimentación y análisis):** Diseña y ejecuta pruebas de lazo cerrado, midiendo la frecuencia de actualización de la pose y la respuesta del nodo embebido.

---

## 🧠 Contexto: ¿Cómo funciona la Localización con Cámara Fija?

En esta etapa del curso, la cámara se encuentra **estacionaria** apuntando directamente hacia el plano de la mesa de trabajo:

```mermaid
graph TD
    subgraph Celda de Trabajo [Plano 2D de la Mesa]
        Cam_Fija["Cámara en Posición Fija<br>(Kinova en Home o Cámara Cenital)"]
        Tag_Movil["Marcador AprilTag / ArUco<br>(En el techo del carrito móvil)"]
        Cam_Fija -.->|Captura Visual| Tag_Movil
    end

    subgraph PC Principal [ROS 2 Jazzy Linux]
        Cam_Fija --> Localizer["Nodo de Visión 2D<br>(apriltag_fixed_camera_localizer.py)"]
        Localizer -->|Publica /burger_car_01/pose2d<br>geometry_msgs/Pose2D (x, y, theta)| Agent["micro-ros-agent<br>(UDP :8888)"]
    end

    subgraph Robot Móvil Embebido [ESP32 con micro-ROS]
        Agent == "WiFi UDP" ==> ESP32["Cliente micro-ROS ESP32<br>(visual_navigator)"]
        ESP32 --> Control["Cálculo de Error:<br>Distancia a la Meta"]
        Control -->|Publica /burger_car_01/distance_to_goal| Agent
        Control --> Actuadores["LEDs / Salidas de Motor"]
    end
```

### 📐 Principio de Medición en el Plano:
1. **Centro Óptico como Origen $(0, 0)$:** El centro de la imagen capturada representa el origen $(x=0, y=0)$ de la mesa.
2. **Posición $(x, y)$:** Al detectar el marcador en la imagen, se calcula la distancia en píxeles desde el centro óptico y se multiplica por una escala conocida ($\text{píxeles por metro}$).
3. **Orientación $\theta$ (Yaw):** Se calcula a partir del ángulo de inclinación del vector entre las esquinas del marcador.
4. **Mensaje Estándar:** La pose calculada se envía como un mensaje estándar `geometry_msgs/msg/Pose2D` $(x, y, \theta)$.

---

## 📋 Estructura de Trabajo del Taller

1. **Fase 1:** Ejecución y verificación del nodo localizador de cámara fija en la PC.
2. **Fase 2:** Programación del firmware en el ESP32 para suscribirse a la pose visual y calcular errores de seguimiento.
3. **Fase 3:** Integración del lazo cerrado distribuido mediante el agente micro-ROS.
4. **Fase 4:** Pruebas experimentales de respuesta y análisis de oclusión visual.

---

## 1. Fase 1: Nodo Localizador de Cámara Fija en ROS 2

### 🧠 El Concepto
El nodo [`scripts/apriltag_fixed_camera_localizer.py`](file:///home/roncanciovl/ros2_ws/src/burger_delivery/scripts/apriltag_pose_projector.py) procesa las coordenadas del marcador y publica periódicamente la pose 2D en el tópico `/{robot_namespace}/pose2d`.

Cuenta además con un **modo de simulación sintética** que permite realizar todas las pruebas de software y comunicación micro-ROS sin necesidad de tener la cámara física conectada en ese instante.

### 🛠️ Ejercicio: Probar el Publicador de Pose

Abre una terminal en tu PC y ejecuta el nodo:

```bash
cd ~/ros2_ws/src/burger_delivery
source /opt/ros/jazzy/setup.bash

# Lanzar en modo simulación (para pruebas directas en laboratorio):
python3 scripts/apriltag_fixed_camera_localizer.py --ros-args -p simulated_mode:=true -p robot_namespace:=burger_car_01
```

En una segunda terminal, inspecciona la publicación de datos:
```bash
ros2 topic list
# Debe aparecer: /burger_car_01/pose2d

ros2 topic echo /burger_car_01/pose2d
```

Observa cómo las coordenadas $(x, y, \theta)$ se actualizan continuamente a 10 Hz.

---

## 2. Fase 2: Firmware micro-ROS en ESP32 (Consumidor de Pose)

### 🧠 El Concepto
El ESP32 se conecta por Wi-Fi UDP a la PC. Mediante la librería `rclc`, crea un nodo que se **suscribe** al tópico `/burger_car_01/pose2d`.

Cada vez que llega una nueva pose visual:
1. Lee las coordenadas actuales $(x_{\text{actual}}, y_{\text{actual}}, \theta_{\text{actual}})$.
2. Calcula la distancia euclidiana hacia un punto meta $(x_{\text{meta}}, y_{\text{meta}})$:
   $$e_d = \sqrt{(x_{\text{meta}} - x_{\text{actual}})^2 + (y_{\text{meta}} - y_{\text{actual}})^2}$$
3. Publica la distancia calculada en `/burger_car_01/distance_to_goal` (`std_msgs/msg/Float32`).
4. Modula el LED indicador: si la distancia es menor a $5\text{ cm}$, el LED queda fijo (meta alcanzada); si no, parpadea.

### 🛠️ Ejercicio: Cargar el Código en el ESP32

Abre Arduino IDE o PlatformIO y carga el siguiente código en el ESP32:

```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/pose2_d.h>
#include <std_msgs/msg/float32.h>
#include <math.h>

// ==========================================
// CONFIGURACIÓN DE RED
// ==========================================
const char* SSID = "ros2";
const char* PASSWORD = "ros12345";
IPAddress AGENT_IP(192, 168, 1, 100);
const size_t AGENT_PORT = 8888;

#define ROBOT_NAMESPACE "burger_car_01"
#define PIN_STATUS_LED 2

// Coordenadas objetivo en el plano de la mesa (en metros)
const float GOAL_X = 0.30;
const float GOAL_Y = 0.00;

// Variables de micro-ROS
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t pose_sub;
rcl_publisher_t dist_pub;

geometry_msgs__msg__Pose2D msg_pose;
std_msgs__msg__Float32 msg_dist;

bool is_connected = false;

// Callback: Ejecutado automáticamente al recibir una nueva pose de la cámara
void pose_callback(const void * msgin) {
  const geometry_msgs__msg__Pose2D * pose = (const geometry_msgs__msg__Pose2D *)msgin;

  float current_x = pose->x;
  float current_y = pose->y;
  float current_theta = pose->theta;

  // Cálculo del error de distancia
  float dx = GOAL_X - current_x;
  float dy = GOAL_Y - current_y;
  float distance = sqrt(dx * dx + dy * dy);

  msg_dist.data = distance;
  rcl_publish(&dist_pub, &msg_dist, NULL);

  // Indicador visual de proximidad
  if (distance < 0.05) {
    digitalWrite(PIN_STATUS_LED, HIGH); // Meta alcanzada (< 5 cm)
  } else {
    digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED)); // Navegando
  }
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Crear nodo en el ESP32
  rclc_node_init_default(&node, "visual_navigator", ROBOT_NAMESPACE, &support);

  // Suscriptor a la pose2d de la cámara fija
  rclc_subscription_init_default(
    &pose_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
    "pose2d");

  // Publicador del error de distancia hacia ROS 2
  rclc_publisher_init_default(
    &dist_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "distance_to_goal");

  // Ejecutor con 1 suscripción
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &pose_sub, &msg_pose, &pose_callback, ON_NEW_DATA);

  return true;
}

void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  set_microros_wifi_transports((char*)SSID, (char*)PASSWORD, AGENT_IP, AGENT_PORT);
}

void loop() {
  if (!is_connected) {
    if (rmw_uros_ping_agent(500, 2) == RMW_RET_OK) {
      if (create_entities()) {
        is_connected = true;
      }
    }
  } else {
    if (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) {
      is_connected = false;
    } else {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
    }
  }
}
```

---

## 3. Fase 3: Integración del Lazo Cerrado con el Agente

### 🛠️ Ejercicio: Puesta en Marcha del Sistema Completo

1. **Terminal 1:** Inicia el agente micro-ROS en la PC principal:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```
2. **Terminal 2:** Lanza el localizador visual:
   ```bash
   python3 scripts/apriltag_fixed_camera_localizer.py --ros-args -p simulated_mode:=true -p robot_namespace:=burger_car_01
   ```
3. **Enciende el ESP32:**
   Observa en la Terminal 1 cómo el agente reconoce la conexión del cliente:
   ```text
   [micro_ros_agent] Client connected with session ID: ...
   ```
4. **Terminal 3:** Verifica el tráfico bidireccional en ROS 2:
   ```bash
   # Ver la pose enviada por la PC al ESP32:
   ros2 topic echo /burger_car_01/pose2d

   # Ver la respuesta de distancia calculada por el ESP32:
   ros2 topic echo /burger_car_01/distance_to_goal
   ```
5. **Inspección con rqt_graph:**
   ```bash
   rqt_graph
   ```
   *Verás la conexión directa entre el nodo de visión en Python y el nodo del ESP32 a través de los tópicos de ROS 2.*

---

## 4. Fase 4: Pruebas Experimentales y Robustez

### 🛠️ Ejercicio: Validación de Robustez
1. **Medición de Tasa de Publicación:**
   ```bash
   ros2 topic hz /burger_car_01/pose2d
   ros2 topic hz /burger_car_01/distance_to_goal
   ```
   Verifica que ambas tasas se mantengan estables a $\approx 10\text{ Hz}$.
2. **Prueba de Desconexión del Agente:**
   Detén el agente con `Ctrl+C` y reinícialo. Comprueba que el ESP32 reconecte automáticamente sin necesidad de presionar el botón de reset físico.

---

## 📦 Entregables del Taller para la Bitácora ABET

Cada equipo debe incluir en su informe técnico:
1. **Captura del Grafo de Nodos (rqt_graph):** Mostrando la interacción entre el nodo localizador de la PC y el nodo `visual_navigator` del ESP32.
2. **Gráfica o Registro de Datos:** Comparativa temporal entre las coordenadas $(x, y)$ de la pose visual y el valor calculado de `distance_to_goal`.
3. **Análisis de Desempeño:** Tasa de refresco promedio y estabilidad del enlace Wi-Fi UDP.
