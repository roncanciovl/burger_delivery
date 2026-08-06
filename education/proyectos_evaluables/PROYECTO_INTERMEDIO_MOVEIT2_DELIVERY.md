# Proyecto de Integración: Celda de Manufactura y Logística Autónoma

## 1. Introducción y Contexto del Workspace

El presente proyecto constituye la evaluación integradora de nivel avanzado para el curso de Robótica. El objetivo es diseñar, implementar y validar una celda de manufactura flexible donde un manipulador **Kinova Gen3** interactúa de manera autónoma con una unidad de transporte (**carrito móvil**) mediante **ROS 2 Jazzy**.

El desarrollo del proyecto se centraliza en el workspace del curso:
`~/ros2_ws/src/burger_delivery/`

Para simular un entorno de desarrollo profesional, la clase se dividirá en **dos grandes equipos de ingeniería**. Cada equipo desarrollará sus nodos y archivos de configuración dentro de los paquetes de este workspace. Ambos equipos deben definir y respetar un **contrato de integración** (interfaces de software y transformaciones espaciales) para lograr el éxito del proyecto.

---

## 2. Contrato de Integración (Interfaces Compartidas)

El éxito del proyecto depende de que ambos equipos respeten las siguientes interfaces. Ningún equipo debe depender de "coordenadas quemadas" (hardcoded); todo debe calcularse dinámicamente.

### 2.1. Árbol de Transformaciones (TF2)
Debe existir una cadena continua entre los sistemas de referencia de ambos equipos:
- **Punto de anclaje compartido:** `map` -> `table_link` -> `tag_mesa`
- **Generado por Equipo Carrito:** `tag_mesa` -> `tag_carrito1` -> `car1_base_link` -> `car1_delivery_tray_frame`
- **Generado por Equipo Kinova:** `map` / `world` -> `gen3_base_link` -> ... -> `burger_grip_frame`

> [!IMPORTANT]
> El destino de la descarga (*Place*) ejecutado por el Kinova debe calcularse dinámicamente consultando el frame `car1_delivery_tray_frame` expuesto por el carrito.

### 2.2. Servicio de Coordinación
El Equipo Kinova (Cliente) solicitará posicionamiento al Equipo Carrito (Servidor) a través de este servicio:
- **Servicio:** `/car1/prepare_delivery_pose`
- **Tipo de Interfaz (Custom o std_srvs):**
  - **Request:** `geometry_msgs/PoseStamped target_pose` (Pose requerida) y `float64 tolerance_xy` (Tolerancia).
  - **Response:** `bool success` y `string status_message` (Diagnóstico).

---

## 3. Asignación: Equipo Kinova (Manipulador)

**Objetivo:** Desarrollar el pipeline de manipulación autónoma para recoger la orden y depositarla en la bandeja del carrito móvil de forma segura.

**Componentes y Archivos a Cargo en el Workspace:**
1. **Paquete de Lógica de Tarea:** Crear un paquete (ej. `kinova_delivery_task`) que contenga el nodo orquestador cliente (`delivery_coordinator_node`).
2. **MoveIt 2 / MTC:** Archivos de configuración o scripts C++/Python para la planificación del brazo.
3. **Manejo de Escena:** Actualizar y mantener el archivo `delivery_scene_fixed.urdf` reflejando las dimensiones reales de la mesa de trabajo y la ubicación exacta de los marcadores fiduciarios (tags).

**Responsabilidades Clave:**
- **Sincronización Geométrica:** Ajustar la geometría de colisión en el URDF para que coincida con el entorno físico (mesa, obstáculos fijos y posición de los tags).
- **Posicionamiento de Visión:** Mover el manipulador a la "Pose de Observación" definida por el equipo carrito al inicio del proceso para garantizar que la cámara tenga línea de visión con los tags.
- **Orquestación:** Llamar al servicio `/car1/prepare_delivery_pose` y esperar el `success` antes de moverse hacia el carrito.
- **Pick and Place Seguro:**
  - Aproximación y agarre (Pre-grasp & Grasp).
  - Usar `AttachedCollisionObject` para fusionar el objeto con el gripper.
  - Calcular dinámicamente la pose de entrega (Place) leyendo el frame `car1_delivery_tray_frame` vía TF2.
  - Planificar esquivando al carrito (actualizando la escena dinámicamente) y retornar a *Home*.

### Rúbrica Específica - Equipo Kinova (50 Puntos)
| Criterio | Puntos | Indicador de Logro |
| :--- | :---: | :--- |
| **Pipeline MoveIt 2** | 20 | Pick-Lift-Move-Place-Retreat ejecutado sin errores cinemáticos. |
| **Manejo de Colisiones** | 10 | Uso correcto de la Planning Scene y del `AttachedCollisionObject`. |
| **TF2 Dinámico** | 10 | El *Place* se adapta si el frame del carrito cambia de posición. |
| **Cliente de Servicio** | 10 | Llama al servicio correctamente y maneja timeouts o errores del servidor. |

---

## 4. Asignación: Equipo Carrito (Unidad Móvil)

**Objetivo:** Proveer localización dinámica de la plataforma móvil y responder a los comandos de orquestación, asegurando que el URDF de la bandeja se posicione correctamente en el mundo.

**Componentes y Archivos a Cargo en el Workspace:**
1. **Paquete `burger_description`:** Modificar y mantener el archivo `car1_apriltag.urdf` asegurando que el `car1_delivery_tray_frame` esté en el centro de la bandeja de carga.
2. **Paquete de Localización:** Crear o configurar un nodo (`cart_localization_node`) dentro de la carpeta `vision_setup/` o un nuevo paquete que publique la TF `tag_mesa -> tag_carrito1`.
3. **Servidor de Servicio:** Desarrollar el paquete `cart_delivery_service` que contenga el nodo servidor para `/car1/prepare_delivery_pose`.

**Responsabilidades Clave:**
- **Definición de Pose de Observación:** Determinar y comunicar al equipo Kinova la configuración de joints o pose del end-effector necesaria para que la cámara del brazo visualice los AprilTags de la mesa y el carrito.
- **Localización Activa:** Procesar el stream de la cámara del Kinova para publicar la TF `tag_mesa -> tag_carrito1` sin saltos bruscos en RViz.
- **Validación del Servidor:** El servicio no debe devolver `success = True` instantáneamente. Debe simular (o ejecutar) el movimiento y verificar internamente mediante TF que el carrito está dentro de la `tolerance_xy` requerida antes de responder al Kinova.

### Rúbrica Específica - Equipo Carrito (50 Puntos)
| Criterio | Puntos | Indicador de Logro |
| :--- | :---: | :--- |
| **Robustez del Servicio** | 20 | El servidor valida la pose final mediante TF antes de confirmar el `success`. |
| **Precisión de Localización** | 15 | El TF `tag_mesa -> tag_carrito1` es estable, continuo y usa timestamps correctos. |
| **Estructura URDF/TF2** | 15 | La cadena `car1_base_link -> car1_delivery_tray_frame` es dimensionalmente correcta. |

---

## 5. El Reto de "Ingeniería en Tiempo Real" (Sustentación)

Durante la sustentación (que evaluará los 50 puntos de cada equipo), el equipo docente introducirá una **modificación imprevista** en el workspace. Los estudiantes deberán diagnosticar la falla en vivo (15-20 min).

**Ejemplos del Reto (Por Equipo):**
- **Equipo Kinova:** "El objeto ahora pesa más y cambia el centro de gravedad, modifiquen los límites de aceleración de MoveIt en `joint_limits.yaml`." O "El carrito 1 se averió, enruten la entrega dinámicamente al `/car2/prepare_delivery_pose`".
- **Equipo Carrito:** "El soporte de la cámara se aflojó (el tag rotó 15 grados). Apliquen un `static_transform_publisher` en su launch file para corregir el offset." O "El cliente envía coordenadas fuera del alcance del carrito; hagan que el servicio devuelva un error controlado en `status_message` sin crashear".

**Protocolo de Respuesta al Reto:**
1. **Identificar Capa:** ¿Falla en URDF/Config, en Transformaciones (TF2), en Servicios o en Planificación?
2. **Recolectar Evidencia:** Mostrar el fallo en terminal (`ros2 topic echo`, `ros2 run tf2_tools view_frames`, logs).
3. **Aplicar Parche:** Modificar el código, el URDF o el Launch dentro del workspace y volver a lanzar (`colcon build` si es necesario).

---

## 6. Entregables Finales y Lanzamiento

Para demostrar la integración final en `~/ros2_ws/src/burger_delivery/`:

1. **Launch de Integración Conjunta:** Debe existir un archivo `launch/system_integration.launch.py` (o scripts bash como `lanzar_robot.sh` actualizados) que levante ambos subsistemas simultáneamente.
2. **Limpieza del Repositorio:** El paquete no debe tener código basura. Los `package.xml` deben tener las dependencias correctas.
3. **Reporte (Markdown):** Un breve análisis justificando parámetros de control (timeouts, IK solver, tolerancias del carrito).
4. **Demostración Práctica:** Flujo completo visualizado en **RViz** mostrando la coordinación de los árboles de TF y la evasión de colisiones.

> *"En la robótica colaborativa, los sistemas no fallan por defecto en sus partes, sino en las interfaces que los unen."*
