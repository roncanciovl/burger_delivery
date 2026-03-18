# Propuesta de Integración: Gemini Robotics ER 1.5 Preview 🧠+🤖

La incorporación del modelo de razonamiento físico **`gemini-robotics-er-1.5-preview`** (VLM) transformará el proyecto *Burger Delivery* de una arquitectura de hardware "ciego" operado por coordenadas manuales, a un sistema autónomo impulsado por Lenguaje Natural e Inteligencia Artificial Visual.

---

## 1. Arquitectura del Sistema (Flujo de Mando)

La propuesta añade una nueva "Capa Cognitiva" por encima de la "Capa de Aplicación" actual, utilizando la API de Google Vertex AI.

```text
[ Humano (Lenguaje Natural) ]
         │
         ▼
[ GEMINI ER 1.5 (VLM) ] ◀─── [ 📷 Cámara RGB-D (Intel RealSense/Webcam) ]
         │ (Extrae: Tarea estructurada + Coordenadas de la hamburguesa)
         ▼
[ Nodo ROS 2: Gestor Lógico (Python) ]
         │ (Traduce Píxeles 2D → TF Coordenadas 3D Reales)
         ▼                                         ▼
[ Acción MoveIt 2 (Brazo Kinova) ]          [ Nodos Micro-ROS (ESP32) ]
(Calcula la trayectoria y agarra)           (Despacha los robots móviles)
```

---

## 2. Requerimientos Técnicos

### 2.1 Hardware Necesario
1.  **Cámara de Visión Artificial:** Idealmente la cámara integrada en el Kinova Gen3 (Intel RealSense) o cualquier cámara USB montada en un soporte fijo mirando la escena (top-down view).
2.  **Brazo Kinova Gen3 7DOF** (Ya operativo).
3.  **Robots Móviles ESP32** (Ya operativos).

### 2.2 Software & Paquetes
Se debe crear un nuevo paquete ROS 2 en Python en el workspace llamado `burger_ai_brain_pkg`. Este paquete requerirá:
*   `google-genai` / `vertexai`: Librería oficial para consumir el modelo `gemini-robotics-er-1.5-preview`.
*   `cv_bridge`: Para transformar la imagen que publica la cámara en ROS a una imagen JPEG para Gemini.
*   `tf2_ros` y `image_geometry`: Para la "Retroproyección" (pasar del pixel `[X=400, Y=250]` donde Gemini dice que está el pan, al metro `[X=0.45m, Y=0.10m, Z=0.0m]` de la base del Kinova).

---

## 3. Ejemplo Práctico: El Flujo de Código

### El Promt que enviará el sistema
Cuando el usuario diga *"Robot, atiende al pedido A sirviendo la hamburguesa"* por voz o chat, tu nodo ROS tomará una foto y enviará el siguiente paquete a Gemini ER:

> **Imagen Adjunta:** (Foto actual de la mesa con la hamburguesa, el brazo y el Robot Móvil A).
> **Prompt del Sistema:** "Eres el cerebro de un brazo Kinova en un entorno de ROS 2. Identifica en la imagen los objetos clave: 'hamburguesa', 'zona de agarre', 'bandeja_robot_a'. Retorna las coordenadas bounding-box 2D de la hamburguesa y una secuencia lógica de sub-tareas a realizar."

### Respuesta Estructurada de Gemini ER (JSON)
El modelo `er-1.5` está altamente optimizado para devolver coordenadas espaciales exactas y precisas (Pointing Accuracy):

```json
{
  "scene_understanding": {
    "hamburguesa_box": [y_min, x_min, y_max, x_max],
    "bandeja_a_box": [y_min, x_min, y_max, x_max]
  },
  "action_plan": [
    {"action": "move_to", "target": "hamburguesa_box"},
    {"action": "close_gripper", "force": 30},
    {"action": "move_to", "target": "bandeja_a_box"},
    {"action": "open_gripper"}
  ]
}
```

### Ejecución de MoveIt 2
Tu nodo en Python leerá el `action_plan` e irá ejecutándolo paso a paso:
1. Extraerá el centro de la caja `hamburguesa_box`.
2. Hará el match geométrico en TF2 para saber la coordenada 3D en el mundo real.
3. Lanzará el *ActionClient* de `FollowJointTrajectory` enviando la meta `.pose.position.x = 0.45`, etc., usando MoveIt para que el robot esquive cualquier obstáculo en el trayecto.

---

## 4. Retos Abiertos y Próximos Pasos

Para hacer esto realidad, los pasos que hay que desarrollar en el código (`TODOs`) son:
1.  **Calibración Extrínseca:** Configurar la transformación de coordenadas estática (`tf_static`) entre el lente de la cámara y la base del Kinova (`world -> camera_link`). Sin esto, el brazo Kinova agarrará el aire porque los píxeles de Gemini no coincidirán con las medidas del robot.
2.  **Gestión de Costos:** Las llamadas al VLM tienen costo y latencia (2 a 5 segundos por foto). La arquitectura debe llamar a Gemini ER solo 1 vez al iniciar la tarea, y dejar que el ciclo ciego (MoveIt) ejecute el movimiento. Gemini no servirá como un ciclo de control cerrado a 1000Hz como `ros2_control`.
3.  **Cámara RGB:** Conseguir un nodo ROS de la cámara (`v4l2_camera` o `realsense2_camera`) para publicar imágenes al tópico `/image_raw`.

---
*Fin de la Propuesta Técnica. Fase: Proof of Concept (PoC).*
