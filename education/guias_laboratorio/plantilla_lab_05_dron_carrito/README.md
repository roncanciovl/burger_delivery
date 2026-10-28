# Plantilla — Laboratorio 05: TF del dron como telemetría del carrito con micro-ROS

Código base de la [Guía de Laboratorio 05](../GUIA_LAB_05_TF_DRON_CARRITO_MICROROS.md).

| Archivo | Equipo | Qué hace |
|---|---|---|
| [`scripts/apriltag_fixed_camera_localizer.py`](../../../scripts/apriltag_fixed_camera_localizer.py) (en la raíz del repositorio) | Dron | Detecta `tag_mesa` y el tag del carrito en la imagen del dron y, con `publish_tf:=true`, publica `tag_mesa → tag_carrito1` en `/tf` con la estampa de la imagen |
| [`relevo_tf_pose.py`](relevo_tf_pose.py) | Carrito (Fase A) | Lee la TF con tf2 y la reenvía como `geometry_msgs/PoseStamped` en `/<ns>/pose_mesa`; no publica si la TF tiene más de `max_age_s` |
| [`carrito_tf_control/carrito_tf_control.ino`](carrito_tf_control/carrito_tf_control.ino) | Carrito | Firmware del ESP32: control uniciclo en dos fases (Proyecto Corte 2), motores por PWM y paro si la pose no llega. `FUENTE_POSE` elige la Fase A (relevo) o la Fase B (`/tf` directo) |

## Contrato entre los dos equipos

| Elemento | Valor por defecto | Quién lo fija |
|---|---|---|
| `ROS_DOMAIN_ID` de la pareja | `40 + NN` (pareja 01 → `41`) | Docente; el ESP32 lo fija en `DOMINIO` |
| Transformación | `tag_mesa → tag_carrito1` | Dron publica; carrito lee |
| Estampa | La de la imagen que originó la detección | Dron |
| Altura del tag del carrito (`tag_height_m`) | Medida con cinta | Carrito la mide; dron la configura |
| Pose para el ESP32 (Fase A) | `/burger_car_01/pose_mesa` (`PoseStamped`) | Carrito |
| Meta | `/burger_car_01/meta` (`Pose2D`, marco del `tag_mesa`) | Carrito |

## Fase B: la biblioteca de micro-ROS no trae `tf2_msgs`

La biblioteca precompilada `micro_ros_arduino` de Jazzy no compila el paquete `tf2_msgs`
(revise `built_packages` en su repositorio). Para suscribir `/tf` en el ESP32 hay que
regenerarla con ese paquete, como indica la guía (Fase 5). La Fase A funciona con la
biblioteca estándar.

## Estado de validación

- Localizador con `publish_tf` y relevo: probados sin ROS, con `rclpy` y `tf2_ros` simulados y
  una escena sintética de AprilTags vista en perspectiva: la TF salió en
  `(0.306, 0.149) m` y `29.0°` para un carrito en `(0.30, 0.15) m` y `30°`, con la estampa de la imagen.
- Firmware: revisado contra las cabeceras de `rclc`, `rcl`, `rmw_microxrcedds` y
  `micro_ros_utilities` (rama `jazzy`) y compilado en la PC con cabeceras equivalentes. La ley de
  control se simuló con un uniciclo: llega a `(0.30, 0.15)` en 5 s y detiene los motores 300 ms
  después de la última pose. **No se ha compilado para el ESP32 ni probado con el carrito.**
- Pendiente con hardware: la cámara de cada dron en ROS 2, la regeneración de la biblioteca con
  `tf2_msgs` y los valores `PWM_MIN` y `V_RUEDA_MAX` de cada carrito.
