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

> [!NOTE]
> **Evaluación ABET**: Los criterios de evaluación para el **Equipo Kinova** han sido unificados con los del Equipo Carrito bajo la rúbrica institucional ABET. Consulta la **Sección 5** para ver los detalles técnicos y los indicadores evaluados.

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

> [!NOTE]
> **Evaluación ABET**: Los criterios de evaluación para el **Equipo Carrito** han sido unificados con los del Equipo Kinova bajo la rúbrica institucional ABET. Consulta la **Sección 5** para ver los detalles técnicos y los indicadores evaluados.

---

## 5. Sistema de Evaluación y Rúbrica Unificada ABET (100 Puntos)

La evaluación de este proyecto integrador está estructurada bajo el marco de acreditación internacional **ABET** (Student Outcomes SO1, SO2, SO5 y SO6) y alineada estrictamente con la **Rúbrica Oficial Ver. 1.7** del curso. Para simplificar y enfocar la evaluación en los elementos nucleares del proyecto de robótica, ambos equipos son evaluados bajo los mismos **4 indicadores de logro clave**, con una ponderación simétrica de **25 puntos cada uno** (Total: **100 Puntos por equipo**).

### 5.1. Cuadro Resumen de Criterios y Pesos

| Dimensión ABET | Peso | Indicador Oficial Evaluado (Rúbrica Ver. 1.7) | Foco del Criterio en el Proyecto |
| :--- | :---: | :--- | :--- |
| **SO1: Resolución de Problemas** | 25 pts | **1.1** Analiza las relaciones entre fenómenos físicos y modelos matemáticos de orientación y transformaciones, validando el árbol de transformaciones (TF2) en sistemas robóticos distribuidos. | Coherencia, estabilidad y transformaciones dinámicas del árbol de TFs globales y locales. |
| **SO2: Diseño de Soluciones** | 25 pts | **2.3** Implementa y valida trayectorias libres de colisiones configurando pipelines de planificación de movimiento en MoveIt 2 sobre entornos industriales modelados experimentalmente. | Trayectorias robustas Pick-and-Place y lógica interactiva del servicio de coordinación. |
| **SO5: Trabajo en Equipo** | 25 pts | **5.1** Reconoce habilidades técnicas y define interfaces y roles dentro del equipo de trabajo para el desarrollo distribuido de los nodos de la celda de automatización. | Definición y cumplimiento del **Contrato de Integración** (TF, servicios, Git Flow compartido). |
| **SO6: Experimentación y Análisis** | 25 pts | **6.4** Interpreta fallas y diagnósticos experimentales aplicando protocolos de diagnóstico por capas (Sintaxis -> TF -> Red -> Lógica) para aislar errores en hardware y software. | Validación de tolerancias, pruebas del sistema y defensa del **Reto en Tiempo Real** en vivo. |

---

### 5.2. Desglose de Criterios y Tareas Técnicas Específicas

#### Criterio 1: Coherencia Cinematográfica y TF2 Dinámico (SO1 - Indicador 1.1) - [25 Puntos]
* **Descripción:** Evalúa la capacidad de comprender, formular y depurar las relaciones espaciales (orientación, translación) entre los múltiples sistemas de referencia del entorno distribuido.
* **Tareas Técnicas - Equipo Kinova:**
  - Configurar las transformaciones de la celda de manipulación (`map` -> `gen3_base_link` -> `burger_grip_frame`) y garantizar que el gripper sea capaz de leer dinámicamente y con precisión el frame de la bandeja del carrito.
  - Asegurar la calibración de la posición del marcador fiduciario en la mesa (`tag_mesa`) con respecto a la base del robot.
* **Tareas Técnicas - Equipo Carrito:**
  - Estructurar cinemáticamente el URDF de la bandeja (`car1_base_link -> car1_delivery_tray_frame`).
  - Desarrollar el nodo de visión que publica de forma activa y sin ruido la TF `tag_mesa -> tag_carrito1` usando la cámara del robot y marcadores AprilTags, con marcas de tiempo coherentes.

#### Criterio 2: Pipeline MoveIt 2 y Trayectorias Seguras (SO2 - Indicador 2.3) - [25 Puntos]
* **Descripción:** Evalúa el diseño e implementación del flujo lógico de automatización, la validación de trayectorias libres de colisiones y la robustez cinemática del robot manipulador interactuando con la unidad móvil.
* **Tareas Técnicas - Equipo Kinova:**
  - Programar y ejecutar de forma autónoma el flujo cinemático *Pick-Lift-Move-Place-Retreat* en MoveIt 2/MTC, esquivando colisiones y actualizando la `PlanningScene` al sujetar el objeto (`AttachedCollisionObject`).
  - Implementar la lógica de cliente que llama a `/car1/prepare_delivery_pose` con manejo de timeouts.
* **Tareas Técnicas - Equipo Carrito:**
  - Diseñar e implementar el servidor de servicio `/car1/prepare_delivery_pose` con QoS y validaciones geométricas.
  - Programar la validación interna que verifique vía TF2 que la bandeja del carro se encuentre físicamente dentro de `tolerance_xy` antes de responder `success = True`, garantizando una respuesta controlada si las coordenadas están fuera de límites.

#### Criterio 3: Contrato de Integración e Interfaces (SO5 - Indicador 5.1) - [25 Puntos]
* **Descripción:** Evalúa la coordinación interdisciplinaria para definir interfaces de software y hardware claras, y el desarrollo colaborativo empleando herramientas profesionales de gestión de configuración.
* **Tareas Técnicas (Colaboración de Ambos Equipos):**
  - Definir, documentar y respetar el **Contrato de Integración**: ningún equipo debe alterar unilateralmente la interfaz del servicio `/car1/prepare_delivery_pose` o el árbol de TF compartido (`map` -> `table_link` -> `tag_mesa`).
  - Desarrollar colaborativamente el Launch file conjunto (`system_integration.launch.py`) y gestionar el repositorio compartido en GitHub usando Git Flow (ramas limpias, Pull Requests obligatorios y revisión cruzada de código).

#### Criterio 4: Diagnóstico de Fallas por Capas - Reto Real-Time (SO6 - Indicador 6.4) - [25 Puntos]
* **Descripción:** Evalúa la capacidad de experimentación, aislamiento de errores y toma de decisiones basadas en datos técnicos mediante la defensa en vivo del sistema bajo condiciones imprevistas de falla.
* **Tareas Técnicas (Prueba en vivo - Reto en Tiempo Real):**
  - Enfrentar y resolver de manera exitosa el **Reto de Ingeniería en Tiempo Real** durante la sustentación. Cada equipo debe diagnosticar de forma sistemática y en caliente una falla introducida aleatoriamente por el docente en menos de 20 minutos (ej. offsets de visión de AprilTags, límites de aceleración o cambio de nombres de servicios), aplicando el protocolo por capas (Sintaxis -> TF -> Red -> Lógica) y justificando sus decisiones basadas en el análisis de datos recopilados (tópicos, logs, visualizadores).

---

### 5.3. Niveles de Desempeño Cualitativo (Alineados a Escala UMNG)

| Nivel de Desempeño | Rango Equivalente (Total) | Rango por Indicador | Desempeño Cualitativo (ABET) |
| :--- | :---: | :---: | :--- |
| **Excelente** | 95 - 100 pts | 23.8 - 25.0 pts | Desarrolla las tareas técnicas completas superando los requerimientos de robustez, optimización y modularidad. Deja trazabilidad absoluta en Git y documentación, justificando decisiones con juicio de ingeniería avanzado. Resuelve el reto en vivo con rapidez y precisión diagnóstica. |
| **Bueno** | 80 - 94 pts | 20.0 - 23.7 pts | Cumple de manera consistente con las tareas técnicas del respectivo equipo. El código es reproducible, las TFs son estables y el servicio valida correctamente. Resuelve el reto de ingeniería en tiempo real requiriendo mínimas guías. |
| **Aceptable con Riesgo Menor** | 60 - 79 pts | 15.0 - 19.9 pts | Implementa las tareas principales de software, pero presenta deficiencias menores en estabilidad de TF, control de colisiones o manejo de excepciones en servicios. Requiere asistencia parcial para aislar fallas o solucionar el reto de integración. |
| **Cumplimiento Parcial** | 30 - 59 pts | 7.5 - 14.9 pts | Integración incompleta. Existen fallas recurrentes de comunicación DDS, colisiones frecuentes en la trayectoria de MoveIt 2 o falta de validaciones geométricas en el servicio. La documentación y trazabilidad de Git es insuficiente. |
| **No Cumple** | 0 - 29 pts | 0.0 - 7.4 pts | Desconocimiento de los conceptos de ROS 2, cinemática o interfaces. Ausencia de nodos funcionales, colisiones críticas no resueltas o falta de colaboración demostrable para la integración final. |

---

## 6. Entregables Finales y Trazabilidad de Evidencias ABET

Para demostrar formalmente la integración final y proveer evidencias reproducibles para los auditores de acreditación ABET, se debe entregar en el repositorio la siguiente estructura dentro del paquete `burger_delivery`:

1. **Launch de Integración Conjunta (Evidencia SO5 - Indicador 5.1):** Archivo `launch/system_integration.launch.py` que configure y levante simultáneamente la localización por AprilTags, el servidor de control del carrito, y el pipeline y mallas de colisiones del manipulador Kinova Gen3.
2. **Historial de GitHub y Ramas (Evidencia SO5 - Indicador 5.1):** Ramas de desarrollo separadas y Pull Requests con comentarios técnicos que validen el trabajo colaborativo e interdisciplinar de los equipos de ingeniería.
3. **Reporte Técnico en Markdown (Evidencia SO1, SO2 & SO6):** Archivo `README.md` detallado en la raíz del paquete que contenga:
   - Diagrama completo de la arquitectura del software ROS 2 (nodos, tópicos, servicios) y QoS DDS (Evidencia SO2).
   - Diagrama detallado del árbol cinemático global de TF2 (Evidencia SO1 - Indicador 1.1).
   - Análisis y protocolo de experimentación utilizado para calibrar cámaras y tolerancias geométricas (Evidencia SO6).
4. **Defensa en Vivo y Resolución del Reto (Evidencia SO6 - Indicador 6.4):** Demostración práctica del ciclo autónomo Pick-and-Place y resolución exitosa del reto sorpresa de diagnóstico en vivo en la sustentación.

> *"En la robótica colaborativa, los sistemas no fallan por defecto en sus partes, sino en las interfaces que los unen."*

