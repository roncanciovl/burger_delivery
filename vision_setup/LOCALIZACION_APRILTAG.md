# Guía de Localización con AprilTag (Sin Odometría)

Esta guía explica cómo vincular la detección visual de los AprilTags (usando la cámara externa o del Kinova) para posicionar automáticamente los carritos móviles (`car1_base_link` y `car2_base_link`) en el ecosistema ROS 2 y RViz, **sin depender de odometría de ruedas**.

---

## 1. Arquitectura del Árbol de Transformaciones (TF)

El modelo URDF ha sido actualizado para soportar localización visual pura mediante un esquema de "Tracking Dinámico por Tags".

La estructura jerárquica ahora funciona así:

![Diagrama de Localización AprilTag](LOCALIZACION_APRILTAG_DIAGRAM.svg)

### Versión en texto (por si no carga el gráfico):
- **map / world** (Raíz)
  - $\rightarrow$ **table_link** (Mesa física)
    - $\rightarrow$ **tag_mesa** (Tag de referencia en la esquina)
      - $\dashrightarrow$ *[Nodo de Localización AprilTag]* $\dashrightarrow$ **tag_carrito1** (Tag en el techo del carro 1)
        - $\rightarrow$ **car1_base_link** (Centro del Carro 1)
      - $\dashrightarrow$ *[Nodo de Localización AprilTag]* $\dashrightarrow$ **tag_carrito2** (Tag en el techo del carro 2)
        - $\rightarrow$ **car2_base_link** (Centro del Carro 2)

---

### ¿Por qué esta arquitectura es superior (Especialmente usando la cámara del Kinova)?
Si utilizas el módulo de visión integrado en la muñeca del Kinova Gen3 (`camera_color_optical_frame`), la cámara estará **en constante movimiento**. 
Si vincularas el carrito directamente a la cámara (como se hace tradicionalmente), cualquier mínima vibración del brazo robótico o error en la calibración de las articulaciones haría que el carrito pareciera temblar o deslizarse erráticamente por el mapa.

Al conectar matemáticamente el `tag_carrito` directamente con el `tag_mesa`, logramos que **la cámara del Kinova actúe como un observador externo neutral**. Las coordenadas absolutas de la cámara, sus vibraciones y sus errores cinemáticos **se anulan por completo en la ecuación**. El carrito siempre se dibujará sólidamente anclado a la mesa en tu mundo 3D.
---

## 2. Instrucciones para tu Nodo de Localización AprilTag

El nodo de localización AprilTag que tienes (en Python o C++) necesita estar suscrito a los tópicos de las detecciones de la librería `apriltag_ros` o tu detector custom.

El objetivo de ese nodo será **publicar la transformación relativa (TF)** entre los tags detectados.

### ¿Por qué necesitamos un TF Listener al frame de la cámara?

Cuando la librería `apriltag_ros` detecta un tag, reporta su posición **relativa al frame óptico de la cámara** (`camera_color_optical_frame`). Ese frame cambia con cada movimiento del brazo Kinova, por lo que la detección viene "distorsionada" por la perspectiva actual de la cámara.

El truco del listener es el siguiente:
1. **Leemos** dónde ve la cámara el `tag_mesa` → `T(cam → mesa)`
2. **Leemos** dónde ve la cámara el `tag_carrito` → `T(cam → carrito)`
3. **Cancelamos** la perspectiva multiplicando la inversa: `T(mesa → carrito) = T(cam → mesa)⁻¹ × T(cam → carrito)`

El resultado es una transformación **independiente de la posición de la cámara**: no importa si el brazo vibra, gira o se mueve — el carrito siempre queda anclado geométricamente a la mesa.

```python
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np # (o tf_transformations)

class AprilTagLocalizationNode(Node):
    def __init__(self):
        super().__init__('apriltag_localization')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        # Listener al arbol de TF para leer la posicion de la camara
        # y de los tags detectados respecto a ella
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer para calcular la posicion a 30Hz
        self.timer = self.create_timer(0.033, self.publish_dynamic_tf)

    def publish_dynamic_tf(self):
        try:
            # 1. Leer donde vio la camara la MESA
            #    -> Esto nos da la perspectiva actual de la camara (incluye vibraciones del brazo)
            tf_cam_to_mesa = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame', 'tag_mesa_detection', rclpy.time.Time()
            )
            
            # 2. Leer donde vio la camara el CARRITO
            tf_cam_to_carrito = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame', 'tag_carrito_detection', rclpy.time.Time()
            )
            
            # 3. Matematica: Multiplicar la Inversa
            # T(mesa -> carrito) = Inv(T(cam -> mesa)) * T(cam -> carrito)
            # Esto cancela completamente la perspectiva de la camara
            tf_mesa_to_carrito = self.calculate_relative_transform(tf_cam_to_mesa, tf_cam_to_carrito)
            
            # 4. Publicar al Arbol de ROS 2
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'tag_mesa'      # <-- PADRE
            t.child_frame_id = 'tag_carrito1'   # <-- HIJO
            
            t.transform = tf_mesa_to_carrito
            
            self.tf_broadcaster.sendTransform(t)
            
        except tf2_ros.TransformException:
            pass # Si en este frame la camara no ve ambos tags simultaneamente, ignorar.

    def calculate_relative_transform(self, tf1, tf2):
        # Aqui usas librerias matriciales estandar de ROS 2 para multiplicar
        # la matriz inversa de tf1 por tf2.
        pass
```

---

## 3. Consideraciones de Implementación

1. 📌 **Arquitectura TF vs URDF (importante entender la separación):**
   Los frames `tag_carrito1` y `tag_carrito2` no deben vivir como `links` fijos de la escena global. En producción existen en el árbol de TF en tiempo de ejecución, publicados dinámicamente por el nodo de localización AprilTag.

   La geometría rígida del carrito sí puede tener su propio URDF separado. En ese patrón, el URDF del carrito cuelga de su tag:
   `tag_carrito1 -> car1_base_link -> car1_delivery_tray_frame`.

   El archivo `delivery_scene_fixed.urdf` describe la escena fija: mesa, Kinova, cámara y `tag_mesa`. Los carritos están en URDFs separados (`car1_apriltag.urdf`, `car2_apriltag.urdf`) para que puedan colgar del `frame` dinámico publicado por el nodo de localización AprilTag. No publiques otro parent para `car1_base_link` o `car2_base_link`.

2. **Launch File**: Para depuración de la escena fija, lanza:
   ```bash
   ros2 launch burger_description display.launch.py
   ```

   El `launch` principal ya carga los URDFs de los carritos separados:
   `burger_description/urdf/car1_apriltag.urdf` y `burger_description/urdf/car2_apriltag.urdf`.

   Para simular la salida del nodo de localización sin cámara activa:
   ```bash
   ros2 run tf2_ros static_transform_publisher \
     --x 0.35 --y 0.10 --z 0.00 \
     --roll 0 --pitch 0 --yaw 0 \
     --frame-id tag_mesa \
     --child-frame-id tag_carrito1
   ```

   O usa el modo integrado del launch:
   ```bash
   ros2 launch burger_description display.launch.py use_static_carts:=true
   ```

3. ⚠️ **Calibración de Cámara Pendiente**: Queda pendiente verificar y calibrar la posición exacta de la cámara con respecto a su *parent frame* en el URDF físico (proceso conocido como calibración *Eye-in-Hand*). Si bien la matemática relativa anula el movimiento global del brazo, es vital que el marco de referencia óptico de la cámara coincida de forma precisa con el hardware para evitar sesgos espaciales en la proyección de las distancias.

4. ⚠️ **Ajuste Físico del tag_mesa**: Es crítico que las coordenadas `xyz` y `rpy` del joint `table_to_tag_mesa` en el URDF coincidan exactamente con el lugar donde pegues físicamente el sticker en la mesa. Si el tag está rotado 90 grados o desplazado 5cm, toda la localización de los carritos tendrá ese mismo error sistemático. Se recomienda medir con cinta métrica o calibre y actualizar el URDF antes de operar.

5. ⚠️ **Offset Tag-Carrito**: El nodo de localización AprilTag debe conocer la distancia física desde el centro del AprilTag en el techo del carrito hasta el `base_link` (centro del eje de las ruedas). Ese offset se debe incluir en la transformación que publica el nodo (`tag_carrito -> car_base_link`). Si el tag no está perfectamente centrado y no se compensa, el brazo Kinova intentará dejar la hamburguesa en una posición desplazada.
