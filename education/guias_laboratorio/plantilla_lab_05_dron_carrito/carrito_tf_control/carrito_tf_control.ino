// =============================================================================
// Laboratorio 05 — Carrito: control de posición con la TF tag_mesa -> tag_carrito
// ESP32 + micro-ROS (micro_ros_arduino, distribución jazzy), transporte WiFi UDP.
//
// FUENTE_POSE elige de dónde sale la pose del carrito:
//   FUENTE_RELEVO  (Fase A): /<ns>/pose_mesa, geometry_msgs/PoseStamped, que publica
//                            relevo_tf_pose.py en la PC del equipo del carrito.
//   FUENTE_TF      (Fase B): /tf, tf2_msgs/TFMessage, directamente; el firmware se queda
//                            con la transformación PADRE -> HIJO y descarta las demás.
//                            La biblioteca precompilada micro_ros_arduino (jazzy) NO trae
//                            tf2_msgs: hay que regenerarla con ese paquete (guía, Fase 5).
//
// En ambos casos el control es el uniciclo en dos fases del Proyecto Corte 2 y los
// motores se detienen si la pose deja de llegar (POSE_TIMEOUT_MS) o si llega vieja
// (EDAD_MAX_MS, medida con el reloj sincronizado con el agente).
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/pose2_d.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <math.h>
#include <string.h>

// ==========================================
// CONFIGURACIÓN (ajústela antes de cargar)
// ==========================================
#define FUENTE_RELEVO 0
#define FUENTE_TF     1
#define FUENTE_POSE   FUENTE_RELEVO        // ← Fase A: FUENTE_RELEVO · Fase B: FUENTE_TF

#if FUENTE_POSE == FUENTE_TF
#include <tf2_msgs/msg/tf_message.h>      // sólo existe en la biblioteca regenerada
#endif

const char* SSID = "ros2";
const char* PASSWORD = "ros12345";
IPAddress AGENT_IP(192, 168, 1, 100);     // ← IP de la PC que ejecuta el agente del carrito
const size_t AGENT_PORT = 8888;
const size_t DOMINIO = 41;                // ← ROS_DOMAIN_ID de la pareja: 40 + NN (pareja 01 -> 41)

#define ROBOT_NAMESPACE "burger_car_01"
#define FRAME_PADRE     "tag_mesa"
#define FRAME_HIJO      "tag_carrito1"

// Meta inicial en el marco del tag_mesa (m, rad). Se cambia en vivo con /<ns>/meta.
float meta_x = 0.30, meta_y = 0.15, meta_theta = 0.0;

// Ley de control (Proyecto Corte 2) y límites del carrito
const float K_RHO = 0.8, K_ALPHA = 2.0, K_THETA = 1.5;
const float V_MAX = 0.10;                 // m/s
const float W_MAX = 1.20;                 // rad/s
const float TOL_XY = 0.03;                // m
const float TOL_THETA = 0.10;             // rad
const float SEPARACION_RUEDAS = 0.1605;   // m, car1_apriltag.urdf: ruedas en y = ±0.08025

// Seguridad
const uint32_t POSE_TIMEOUT_MS = 300;     // sin pose nueva -> motores detenidos
const int64_t EDAD_MAX_MS = 500;          // pose más vieja que esto -> se descarta
const uint32_t PERIODO_CONTROL_MS = 50;   // 20 Hz

// Puente H (dos motores): un pin PWM y dos de dirección por motor. Ajuste a su carrito.
const int PIN_PWM_IZQ = 25, PIN_IN1_IZQ = 26, PIN_IN2_IZQ = 27;
const int PIN_PWM_DER = 14, PIN_IN1_DER = 12, PIN_IN2_DER = 13;
const int PWM_FREQ = 20000, PWM_RES = 10;           // 20 kHz, 10 bits (0..1023)
const int PWM_MIN = 300;                            // zona muerta medida en la Fase 3
const float V_RUEDA_MAX = 0.25;                     // m/s de la rueda con PWM máximo (medido)
#define PIN_STATUS_LED 2

// ==========================================
// Estado
// ==========================================
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t pose_sub, meta_sub;
rcl_publisher_t dist_pub, cmd_pub, edad_pub, tasa_pub;

#if FUENTE_POSE == FUENTE_TF
tf2_msgs__msg__TFMessage msg_entrada;
#else
geometry_msgs__msg__PoseStamped msg_entrada;
#endif
geometry_msgs__msg__Pose2D msg_meta;
std_msgs__msg__Float32 msg_dist, msg_edad, msg_tasa;
geometry_msgs__msg__Twist msg_cmd;

bool is_connected = false;
bool hay_pose = false, meta_alcanzada = false;
float pose_x = 0, pose_y = 0, pose_theta = 0;
uint32_t t_ultima_pose = 0, t_ultimo_control = 0, t_ultima_tasa = 0, t_ultimo_ping = 0;
uint32_t mensajes_recibidos = 0;

// ==========================================
// Utilidades
// ==========================================
float wrap_to_pi(float a) {
  while (a > M_PI) a -= 2.0f * M_PI;
  while (a < -M_PI) a += 2.0f * M_PI;
  return a;
}

float clip(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Guiñada (giro alrededor de z) de un cuaternión
float yaw_de(const geometry_msgs__msg__Quaternion &q) {
  return atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

bool texto_igual(const rosidl_runtime_c__String &s, const char *c) {
  return s.size == strlen(c) && memcmp(s.data, c, s.size) == 0;
}

// Una pose válida actualiza el estado; la edad sólo se mide si el reloj está sincronizado.
void aceptar_pose(const builtin_interfaces__msg__Time &stamp, float x, float y, float theta) {
  if (rmw_uros_epoch_synchronized()) {
    int64_t estampa_ns = (int64_t)stamp.sec * 1000000000LL + stamp.nanosec;
    float edad_ms = (rmw_uros_epoch_nanos() - estampa_ns) / 1e6f;
    msg_edad.data = edad_ms;
    rcl_publish(&edad_pub, &msg_edad, NULL);
    if (edad_ms > EDAD_MAX_MS) return;     // pose vieja: no se usa
  }
  pose_x = x; pose_y = y; pose_theta = theta;
  hay_pose = true;
  t_ultima_pose = millis();
}

// ==========================================
// Motores
// ==========================================
void configurar_pwm(int pin, int canal) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void) canal;
  ledcAttach(pin, PWM_FREQ, PWM_RES);
#else
  ledcSetup(canal, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, canal);
#endif
}

void escribir_pwm(int pin, int canal, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void) canal;
  ledcWrite(pin, duty);
#else
  (void) pin;
  ledcWrite(canal, duty);
#endif
}

// Velocidad de rueda (m/s) -> sentido y PWM, con zona muerta
void mover_rueda(float v, int pin_pwm, int canal, int in1, int in2) {
  int maximo = (1 << PWM_RES) - 1;
  int duty = 0;
  if (fabsf(v) > 1e-3f) {
    duty = PWM_MIN + (int)((maximo - PWM_MIN) * clip(fabsf(v) / V_RUEDA_MAX, 0.0f, 1.0f));
  }
  digitalWrite(in1, v >= 0 ? HIGH : LOW);
  digitalWrite(in2, v >= 0 ? LOW : HIGH);
  escribir_pwm(pin_pwm, canal, duty);
}

void aplicar(float v, float w) {
  float v_izq = v - w * SEPARACION_RUEDAS / 2.0f;
  float v_der = v + w * SEPARACION_RUEDAS / 2.0f;
  mover_rueda(v_izq, PIN_PWM_IZQ, 0, PIN_IN1_IZQ, PIN_IN2_IZQ);
  mover_rueda(v_der, PIN_PWM_DER, 1, PIN_IN1_DER, PIN_IN2_DER);
  msg_cmd.linear.x = v;
  msg_cmd.angular.z = w;
}

void detener() { aplicar(0.0f, 0.0f); }

// ==========================================
// Callbacks
// ==========================================
#if FUENTE_POSE == FUENTE_TF
void entrada_callback(const void * msgin) {
  const tf2_msgs__msg__TFMessage * m = (const tf2_msgs__msg__TFMessage *)msgin;
  mensajes_recibidos++;
  // /tf trae las transformaciones de TODOS los publicadores del dominio: filtrar
  for (size_t i = 0; i < m->transforms.size; i++) {
    const geometry_msgs__msg__TransformStamped &t = m->transforms.data[i];
    if (texto_igual(t.header.frame_id, FRAME_PADRE) && texto_igual(t.child_frame_id, FRAME_HIJO)) {
      aceptar_pose(t.header.stamp, t.transform.translation.x, t.transform.translation.y,
                   yaw_de(t.transform.rotation));
    }
  }
}
#else
void entrada_callback(const void * msgin) {
  const geometry_msgs__msg__PoseStamped * m = (const geometry_msgs__msg__PoseStamped *)msgin;
  mensajes_recibidos++;
  aceptar_pose(m->header.stamp, m->pose.position.x, m->pose.position.y, yaw_de(m->pose.orientation));
}
#endif

void meta_callback(const void * msgin) {
  const geometry_msgs__msg__Pose2D * m = (const geometry_msgs__msg__Pose2D *)msgin;
  meta_x = m->x; meta_y = m->y; meta_theta = m->theta;
  meta_alcanzada = false;
}

// Uniciclo en dos fases (Proyecto Corte 2, §4)
void paso_de_control() {
  if (!hay_pose || millis() - t_ultima_pose > POSE_TIMEOUT_MS) {
    detener();                               // sin pose fresca no hay movimiento
    digitalWrite(PIN_STATUS_LED, LOW);
  } else if (meta_alcanzada) {
    detener();
    digitalWrite(PIN_STATUS_LED, HIGH);
  } else {
    float dx = meta_x - pose_x, dy = meta_y - pose_y;
    float rho = sqrtf(dx * dx + dy * dy);
    float alpha = wrap_to_pi(atan2f(dy, dx) - pose_theta);
    float theta_e = wrap_to_pi(meta_theta - pose_theta);
    msg_dist.data = rho;
    rcl_publish(&dist_pub, &msg_dist, NULL);
    if (rho > TOL_XY) {                      // Fase 1: rumbo y avance
      aplicar(clip(K_RHO * rho * cosf(alpha), 0.0f, V_MAX), clip(K_ALPHA * alpha, -W_MAX, W_MAX));
    } else if (fabsf(theta_e) > TOL_THETA) { // Fase 2: giro en el sitio
      aplicar(0.0f, clip(K_THETA * theta_e, -W_MAX, W_MAX));
    } else {
      meta_alcanzada = true;
      detener();
    }
    digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
  }
  rcl_publish(&cmd_pub, &msg_cmd, NULL);
}

// ==========================================
// Entidades micro-ROS
// ==========================================
bool create_entities() {
  allocator = rcl_get_default_allocator();
  // El dominio lo fija el ESP32: debe coincidir con el de la pareja dron-carrito
  init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
  if (rcl_init_options_set_domain_id(&init_options, DOMINIO) != RCL_RET_OK) return false;
  if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "control_posicion", ROBOT_NAMESPACE, &support) != RCL_RET_OK) return false;

  // Los mensajes con cadenas y secuencias necesitan memoria reservada de antemano
  micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_string_capacity = 32;               // frame_id y child_frame_id
  conf.max_ros2_type_sequence_capacity = 4;    // transformaciones por TFMessage
  conf.max_basic_type_sequence_capacity = 4;

#if FUENTE_POSE == FUENTE_TF
  micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), &msg_entrada, conf);
  // Nombre ABSOLUTO: "tf" sin barra se resolvería a /burger_car_01/tf
  if (rclc_subscription_init_best_effort(
        &pose_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf") != RCL_RET_OK) return false;
#else
  micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped), &msg_entrada, conf);
  if (rclc_subscription_init_default(
        &pose_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped), "pose_mesa") != RCL_RET_OK) return false;
#endif
  if (rclc_subscription_init_default(
        &meta_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "meta") != RCL_RET_OK) return false;
  rclc_publisher_init_default(&dist_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "distance_to_goal");
  rclc_publisher_init_default(&cmd_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_aplicado");
  rclc_publisher_init_default(&edad_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "edad_pose_ms");
  rclc_publisher_init_default(&tasa_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "mensajes_por_s");

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &pose_sub, &msg_entrada, &entrada_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &meta_sub, &msg_meta, &meta_callback, ON_NEW_DATA);

  // Reloj común con el agente: permite medir la edad de cada pose
  rmw_uros_sync_session(1000);
  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&dist_pub, &node);
  rcl_publisher_fini(&cmd_pub, &node);
  rcl_publisher_fini(&edad_pub, &node);
  rcl_publisher_fini(&tasa_pub, &node);
  rcl_subscription_fini(&pose_sub, &node);
  rcl_subscription_fini(&meta_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  rcl_init_options_fini(&init_options);
}

void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_IN1_IZQ, OUTPUT); pinMode(PIN_IN2_IZQ, OUTPUT);
  pinMode(PIN_IN1_DER, OUTPUT); pinMode(PIN_IN2_DER, OUTPUT);
  configurar_pwm(PIN_PWM_IZQ, 0);
  configurar_pwm(PIN_PWM_DER, 1);
  detener();
  set_microros_wifi_transports((char*)SSID, (char*)PASSWORD, AGENT_IP, AGENT_PORT);
}

void loop() {
  if (!is_connected) {
    detener();
    if (rmw_uros_ping_agent(500, 2) == RMW_RET_OK && create_entities()) {
      is_connected = true;
      t_ultima_tasa = millis();
    }
    return;
  }
  uint32_t ahora = millis();
  if (ahora - t_ultimo_ping >= 500) {          // comprobar el agente dos veces por segundo
    t_ultimo_ping = ahora;
    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      detener();                               // sin agente no hay pose: parar ya
      destroy_entities();
      digitalWrite(PIN_STATUS_LED, LOW);
      hay_pose = false;
      is_connected = false;
      return;
    }
  }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  if (ahora - t_ultimo_control >= PERIODO_CONTROL_MS) {
    t_ultimo_control = ahora;
    paso_de_control();
  }
  if (ahora - t_ultima_tasa >= 1000) {         // mensajes de entrada recibidos por segundo
    msg_tasa.data = mensajes_recibidos * 1000.0f / (ahora - t_ultima_tasa);
    rcl_publish(&tasa_pub, &msg_tasa, NULL);
    mensajes_recibidos = 0;
    t_ultima_tasa = ahora;
  }
}
