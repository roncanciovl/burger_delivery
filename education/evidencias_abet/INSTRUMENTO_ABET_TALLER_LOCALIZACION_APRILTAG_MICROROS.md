# Instrumento de Evidencia y Calificación — Taller: Localización Visual 2D con AprilTags (Kinova) y Lazo de Pose con micro-ROS

> **Documento Único de Entrega Estudiantil y Evaluación ABET.** Asociado a `education/talleres/TALLER_LOCALIZACION_APRILTAG_KINOVA_MICROROS.md`.  
> Este formato integra la entrega formal por equipos: registro de evidencias, tablas de datos experimentales llenadas durante la práctica (Modos A y B), cuestionario de análisis técnico con respuestas justificadas, Anexo A de comprobación individual para sustentar el logro ABET de cada estudiante y rúbricas analíticas docentes.

---

## 1. Identificación y Control

| Campo | Registro Oficial |
|---|---|
| **Programa Académico** | Ingeniería Mecatrónica |
| **Asignatura** | ROBOT OPERATING SYSTEM - ROS |
| **Periodo Académico** | 2026-2 |
| **Corte / Instrumento** | Segundo Corte / Talleres y tareas |
| **Actividad Evaluada** | Taller — Localización Visual 2D con Cámara Kinova y Lazo de Pose con micro-ROS (Modos A y B) |
| **Número de Grupo / Subgrupo** | |
| **Estudiante 1 (Nombre y Código)** | |
| **Estudiante 2 (Nombre y Código)** | |
| **Estudiante 3 (si aplica)** | |
| **Namespace del Robot Asignado** | `/burger_car_` &nbsp;&nbsp;&nbsp;&nbsp; *(ej. `/burger_car_01`)* |
| **Modos Evaluados** | Modo A (Simulado) ☐ &nbsp;&nbsp;&nbsp;&nbsp; Modo B (Real Kinova) ☐ |
| **Nombre del Archivo de Entrega** | `C2_T_APRILTAG_G<grupo>_<codigo1>_<codigo2>_v1.docx` |
| **Fecha de Realización en Laboratorio** | |
| **Fecha de Entrega del Documento** | |
| **Docente Evaluador** | Ing. Henry Roncancio |
| **Versión del Instrumento** | Versión 1.0 (2026-2) |
| **Unidad de Análisis / Captura** | Equipo colaborativo con comprobación individual (Anexo A) |

---

## 2. Parámetros de Assessment

| Parámetro | Regla Institucional y Metodológica Adoptada |
|---|---|
| **Población o cohorte** | Censo completo de estudiantes matriculados que presentan el taller de localización visual y micro-ROS en 2026-2. |
| **Momento de medición** | Segundo corte, tras la ejecución de las fases de detección visual, homografía y lazo embebido en ESP32. |
| **Evaluador** | Docente titular de la asignatura ROBOT OPERATING SYSTEM - ROS. |
| **Umbral individual de logro** | Nivel **N3 o superior** (puntaje mínimo de 300 sobre 500) en cada uno de los indicadores evaluados. |
| **Meta de cohorte** | Al menos el **70% de los estudiantes evaluables** debe alcanzar el nivel N3 o superior en cada indicador. |
| **Regla de muestreo** | Censo 100%: no se utiliza muestreo; se evalúa la totalidad de los estudiantes y equipos exigibles. |
| **Evidencia faltante** | Entrega exigible sin evidencia obligatoria verificable: se califica en N1 con puntaje 0. Retiro oficial: `NA / no evaluado`. |
| **Regla de trabajo en equipo** | Informe único por equipo con tablas experimentales llenas más Anexo A individual obligatorio para cada integrante. |

---

## 3. Niveles de Desempeño para Zubatronic/SGDE

| Nivel | Intervalo Zubatronic | Valor Guía | Interpretación y Criterio de Logro |
|---|---:|:---:|---|
| **N5** | 475–500 | 500 | **Excelente:** Solución y evidencia completas, precisas, reproducibles y explicadas con profundidad analítica y matemática sobresaliente. |
| **N4** | 400–474 | 450 | **Bueno:** Desempeño correcto con omisiones menores que no comprometen la operación, reproducibilidad ni diagnóstico. |
| **N3** | 300–399 | 350 | **Aceptable:** Demuestra el desempeño esencial con evidencia verificable. **Es el umbral individual de logro.** |
| **N2** | 150–299 | 250 | **Cumplimiento parcial:** Evidencia incompleta, métricas faltantes o errores conceptuales en homografía o micro-ROS. |
| **N1** | 0–149 | 100 | **No cumple:** Evidencia mínima, fragmentaria o no funcional. Sin evidencia obligatoria se registra 0. |

---

## 4. Alineación de Criterios, RAE y Student Outcomes

| Criterio | Peso | Student Outcome Principal | Indicador de Desempeño Literal del Programa | Evidencia Directa Obligatoria |
|---|:---:|:---:|---|---|
| **C1. Arquitectura y lazo distribuido XRCE-DDS / DDS** | 25% | **SO1** | **1.1.** Formula y conecta el flujo de datos entre la cámara del Kinova, un nodo de localización en PC y un nodo de control embebido en microcontrolador. | Grafo `rqt_graph` completo, ejecución de `micro_ros_agent` en PC, suscripción en ESP32 (`Subscription count: 1`) y Tabla 1 llena. |
| **C2. Selección de interfaces compactas y homografía** | 25% | **SO2** | **2.2.** Selecciona interfaces y mensajes compactos (imagen comprimida, `geometry_msgs/msg/Pose2D`) respetando restricciones de ancho de banda y memoria del ESP32, y justifica sus limitaciones. | Justificación de `image_transport`, cálculo de homografía sobre `tag_mesa` sin TF2, dimensiones exactas del tag con pie de rey y respuestas técnicas. |
| **C3. Exactitud de localización, paralaje y telemetría** | 25% | **SO6** | **6.1.** Diseña y ejecuta pruebas de lazo cerrado, midiendo la exactitud de la pose, su frecuencia de actualización y la respuesta del nodo embebido. | Tabla 2 de exactitud y paralaje diligenciada (5 puntos), medición de tasas `ros2 topic hz` (Tabla 3) y activación de LED a $<5\text{ cm}$. |
| **C4. Robustez del lazo, oclusión y resiliencia** | 15% | **SO6** | **6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos por capas para aislar errores en hardware y software. | Prueba de oclusión de tags (Tabla 4), diagnóstico de congelamiento de pose, propuesta de timeout con `millis()` y reconexión tras caída del agente. |
| **C5. Trazabilidad en rosbag MCAP y trabajo en equipo** | 10% | **SO3 / SO5** | **3.1 - 3.3.** Documentación técnica reproducible y telemetría.<br>**5.1.** Define roles técnicos y coordina la ejecución en equipo. | Dataset MCAP `dataset_lazo_burger_car_NN` con metadatos válidos, gráficas de $x, y, d$ vs tiempo, informe técnico y Anexo A individual. |
| **TOTAL** | **100%** | | | |

---

## 5. Registro de Evidencias de la Práctica (E1–E8)

| Código | Evidencia Requerida | Localizador en el Documento / Repositorio / Archivo |
|---|---|---|
| **E1** | Captura de `rqt_graph` en Modo B mostrando `/camera/kinova_vision_color`, `/apriltag_fixed_camera_localizer` y `/burger_car_NN/visual_navigator`. | |
| **E2** | Configuración de tags: IDs, medición con pie de rey de `tag_size_m`, orientación de ejes del `tag_mesa` y cálculo a mano del Ejercicio 3.1. | |
| **E3** | Captura de terminal con `ros2 topic info /burger_car_NN/pose2d` mostrando `Subscription count: 1` (ESP32 conectado a través del agente). | |
| **E4** | Captura de `ros2 topic echo /burger_car_NN/distance_to_goal` mostrando convergencia a $< 0.05\text{ m}$ y evidencia fotográfica del LED encendido fijo. | |
| **E5** | Medición de tasas con `ros2 topic hz` para imagen comprimida, pose2d y distance_to_goal, demostrando sincronismo del lazo. | |
| **E6** | Captura del comportamiento del lazo ante oclusión del tag del carrito y del tag de referencia (congelamiento de telemetría y LED). | |
| **E7** | Cronometraje y captura de reconexión autónoma del ESP32 tras detener y relanzar el agente UDP (sin reset físico del microcontrolador). | |
| **E8** | Salida de `ros2 bag info dataset_lazo_burger_car_NN` y gráfica de trayectoria ($x, y$) y distancia a la meta frente al tiempo de recepción. | |

---

## 6. Tablas de Registro Experimental (Diligenciadas por el Equipo)

### Tabla 1: Configuración de Tags, Dispositivos y Parámetros del Localizador

| Parámetro | Valor Configurado / Medido | Método de Verificación |
|---|---|---|
| **Familia de AprilTags utilizada** | Tag36h11 | Inspección visual del tag impreso |
| **ID del Tag de Referencia (`tag_mesa`)** | | Parámetro `-p reference_tag_id:=` |
| **ID del Tag del Carrito (`tag_id`)** | | Parámetro `-p tag_id:=` |
| **Tamaño real del cuadro negro (`tag_size_m`)** | m (ej. 0.100 m) | Medición con pie de rey |
| **Resolución del flujo visual** | píxeles | `ros2 topic echo /camera/color/camera_info --once` |
| **Namespace del robot móvil** | `/burger_car_` | Parámetro `-p robot_namespace:=` |
| **Dirección IP y puerto del Agente micro-ROS** | UDP puerto 8888 | Configuración de firmware y PC |
| **Dirección IP asignada al ESP32** | | Monitor Serie (`IP del ESP32: ...`) |

---

### Tabla 2: Exactitud de Localización 2D y Medición de Paralaje (Modo B)

> Realice 5 mediciones colocando el tag sobre el plano de la mesa (altura $z=0$) y luego en el techo del carrito ($z \approx h_{carro}$) para evaluar el error de paralaje por perspectiva.

| Punto de Prueba | Coordenada Real en Mesa ($x_{real}, y_{real}$) [m] | Medición en Plano ($x_{plano}, y_{plano}$) [m] | Error Plano ($e_{plano}$) [m] | Medición Techo Carrito ($x_{techo}, y_{techo}$) [m] | Error con Carrito ($e_{techo}$) [m] | Error de Paralaje Observado ($\Delta e$) [m] |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **P1 (Cerca origen)** | `(0.00, 0.00)` | | | | | |
| **P2 (Zona central)** | `(0.20, 0.15)` | | | | | |
| **P3 (Extremo X)** | `(0.40, 0.00)` | | | | | |
| **P4 (Extremo Y)** | `(0.00, 0.30)` | | | | | |
| **P5 (Diagonal)** | `(0.35, 0.25)` | | | | | |

---

### Tabla 3: Medición de Tasas de Frecuencia y Sincronismo del Lazo Distribuido

| Tópico del Lazo | Tasa Nominal / Esperada | Tasa Media Medida (`ros2 topic hz`) | Mínimo (s) | Máximo (s) | Desviación Estándar (`std dev`) |
|---|:---:|:---:|:---:|:---:|:---:|
| `/camera/color/image_raw/compressed` | 15–30 Hz | Hz | | | |
| `/burger_car_NN/pose2d` | Igual a la imagen | Hz | | | |
| `/burger_car_NN/distance_to_goal` | Igual a la pose | Hz | | | |

---

### Tabla 4: Pruebas de Robustez, Oclusión y Resiliencia del Enlace

| Escenario Experimental | Acción Ejecutada | Respuesta Inmediata del Localizador | Respuesta del ESP32 y Estado del LED | Tiempo de Recuperación tras Normalizar |
|---|---|---|---|---|
| **Oclusión Tag Carrito** | Tapar el tag del carrito 10 s | Avisa tag no visible, cesa `/pose2d` | | |
| **Oclusión Tag Mesa** | Tapar el `tag_mesa` 10 s | Avisa ref no visible, cesa `/pose2d` | | |
| **Caída del Agente micro-ROS** | `Ctrl+C` al agente en PC | Sigue publicando `/pose2d` | LED se apaga / pierde sesión | |
| **Reinicio del Agente** | Relanzar agente UDP en PC | Continúa normal | Reconecta automáticamente | s (sin reset físico) |

---

## 7. Cuestionario de Análisis Técnico (Respondido por el Equipo)

### Pregunta 1: Ausencia del Agente micro-ROS en rqt_graph
*¿Por qué el nodo `micro_ros_agent` no aparece como un nodo visible en `rqt_graph` ni en `ros2 node list`, mientras que el nodo embebido `/burger_car_NN/visual_navigator` sí aparece en el grafo DDS a pesar de estar ejecutándose físicamente dentro del microcontrolador ESP32? Explique el rol de puente XRCE-DDS ↔ DDS.*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 2: Justificación de geometry_msgs/msg/Pose2D y Ausencia de Header
*Justifique por qué en este lazo embebido se seleccionó el mensaje compacto `geometry_msgs/msg/Pose2D` en lugar de `geometry_msgs/msg/PoseStamped`. ¿Qué ventajas aporta a la memoria y ancho de banda del ESP32 y qué desventaja crítica introduce la ausencia del campo `std_msgs/Header` (sello temporal y marco de coordenadas)?*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 3: Origen Físico y Mitigación del Error de Paralaje
*A partir de los datos registrados en la Tabla 2, explique físicamente por qué la estimación de la pose del carrito difiere cuando el tag está colocado sobre la mesa frente a cuando está en el techo del vehículo ($z > 0$). Si la homografía matemática asume un plano bidimensional estricto ($z=0$), ¿cómo influye el ángulo de inclinación de la cámara del Kinova en este error de paralaje y cómo se corregiría formalmente con matrices de transformación TF2?*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 4: Diagnóstico del LED Congelado y Necesidad de Timeout de Pose
*Durante la prueba de oclusión (Ejercicio 6.2), el LED del ESP32 se congela en su último estado cuando se tapa el tag. Explique por qué un LED encendido fijo NO garantiza que el carrito esté realmente en la meta. Proponga cómo implementar un mecanismo de seguridad por software (*Watchdog / Timeout*) en el firmware utilizando `millis()` y qué acción debería tomar un robot móvil real ante la pérdida de pose.*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 5: Comparativa Metodológica: Modo A (Simulado) vs. Modo B (Real)
*Compare las fortalezas y limitaciones de validar el lazo en Modo A frente a Modo B. ¿Qué componentes de la cadena de ingeniería valida con certeza el Modo A y cuáles enmascara por completo? ¿Por qué es una buena práctica de ingeniería robótica validar primero en Modo A antes de interactuar con la celda física del Kinova?*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 6: Uso de la Marca Temporal de Recepción en rosbag2
*En el Ejercicio 6.4 se grabó el dataset MCAP con `ros2 bag record`. Dado que `Pose2D` no incluye `header.stamp`, ¿cómo permite `rosbag2` reconstruir la relación temporal y latencia entre la publicación de la pose en la PC y la respuesta de distancia emitida por el ESP32?*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

## 8. Anexo A — Comprobación Individual de Desempeño

> **Propósito ABET:** Este anexo verifica la autoría, comprensión técnica y contribución directa individual de cada estudiante, sustentando la asignación del logro individual en los Student Outcomes evaluados.

### Estudiante 1: __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas en la práctica (ej. configuración de OpenCV, calibración y medición de tags, lanzamiento del agente, programación/carga del firmware en ESP32 o análisis de rosbag). | |
| **2. Comprensión de la homografía:** Explique con sus propias palabras qué hace la función de homografía con las esquinas del `tag_mesa` y cómo transforma píxeles de cámara a metros en el plano de trabajo. | |
| **3. Cadena de transporte XRCE-DDS:** Explique cómo viaja el mensaje desde que OpenCV calcula la pose en la PC hasta que la función callback del ESP32 recibe los datos y actualiza el LED. | |
| **4. Análisis de una anomalía resuelta:** Describa un error o fallo enfrentado durante la práctica (ej. iluminación, tags no detectados, colisión de namespaces, fallo de Wi-Fi) y cómo lo resolvió. | |
| **5. Autoría y reproducibilidad:** Indique qué secciones del código, tablas, capturas de pantalla o gráficas del informe fueron elaboradas directamente por usted. | |

**Estado de comprobación individual Estudiante 1:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐  
**Firma del Estudiante 1:** ________________________________________

---

### Estudiante 2: __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas en la práctica (ej. configuración de OpenCV, calibración y medición de tags, lanzamiento del agente, programación/carga del firmware en ESP32 o análisis de rosbag). | |
| **2. Comprensión de la homografía:** Explique con sus propias palabras qué hace la función de homografía con las esquinas del `tag_mesa` y cómo transforma píxeles de cámara a metros en el plano de trabajo. | |
| **3. Cadena de transporte XRCE-DDS:** Explique cómo viaja el mensaje desde que OpenCV calcula la pose en la PC hasta que la función callback del ESP32 recibe los datos y actualiza el LED. | |
| **4. Análisis de una anomalía resuelta:** Describa un error o fallo enfrentado durante la práctica (ej. iluminación, tags no detectados, colisión de namespaces, fallo de Wi-Fi) y cómo lo resolvió. | |
| **5. Autoría y reproducibilidad:** Indique qué secciones del código, tablas, capturas de pantalla o gráficas del informe fueron elaboradas directamente por usted. | |

**Estado de comprobación individual Estudiante 2:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐  
**Firma del Estudiante 2:** ________________________________________

---

## 9. Selección del Nivel Alcanzado por Criterio (Rúbricas Docentes)

Marque con una **X** una sola casilla por criterio y registre el valor entero (0 a 500) dentro de la banda correspondiente.

### C1. Arquitectura y lazo distribuido XRCE-DDS / DDS — Peso 25% — Student Outcome SO1

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, analiza la estructura de tramas XRCE-DDS, optimiza la memoria del microcontrolador ajustando el número de handles del executor y justifica arquitectónicamente la ausencia del agente en el grafo. |
| ☐ | **N4 — 400–474** | Establece el lazo bidireccional completo entre la cámara, el localizador, el agente UDP y el ESP32, verificando `Subscription count: 1` y documentando el flujo en `rqt_graph`. |
| ☐ | **N3 — 300–399** | Configura el localizador y el agente micro-ROS, logrando que el ESP32 reciba `/pose2d` y responda con `/distance_to_goal` en Modo A o B. |
| ☐ | **N2 — 150–299** | El lazo opera de forma intermitente, presenta problemas de direccionamiento IP o no logra que el ESP32 publique la distancia calculada. |
| ☐ | **N1 — 0–149** | No logra establecer la comunicación entre la PC y el ESP32 o carece de evidencias funcionales obligatorias. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C2. Selección de interfaces compactas y homografía — Peso 25% — Student Outcome SO2

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, calcula analíticamente la matriz de homografía $3 \times 3$, demuestra la invarianza proyectiva ante cambios de perspectiva y propone el empaquetado de metadatos temporales mínimos en interfaces embebidas. |
| ☐ | **N4 — 400–474** | Calibra con pie de rey el tamaño exacto del tag, orienta correctamente los ejes de `tag_mesa`, justifica el uso de `Pose2D` y calcula a mano la pose de prueba (Ejercicio 3.1). |
| ☐ | **N3 — 300–399** | Configura los parámetros de tag y referencia en el script localizador, obteniendo poses métricas coherentes en el plano de trabajo. |
| ☐ | **N2 — 150–299** | Presenta errores de escala por mala medición de `tag_size_m` o desorientación de los ejes del marco de referencia. |
| ☐ | **N1 — 0–149** | No implementa la homografía, confunde marcos de coordenadas o carece de justificación de interfaces. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C3. Exactitud de localización, paralaje y telemetría — Peso 25% — Student Outcome SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, modela matemáticamente el error de paralaje en función de la altura del vehículo y el ángulo visual de la cámara, proponiendo una compensación geométrica directa en el localizador. |
| ☐ | **N4 — 400–474** | Diligencia completamente la Tabla 2 con 5 puntos de prueba, cuantifica el error en plano vs techo del carrito, mide las frecuencias del lazo (Tabla 3) y valida el LED fijo a $< 5\text{ cm}$. |
| ☐ | **N3 — 300–399** | Realiza mediciones de exactitud en al menos 3 posiciones, verifica el cambio de estado del LED y registra las tasas de los tópicos. |
| ☐ | **N2 — 150–299** | Mediciones incompletas en la Tabla 2, omisión de la evaluación de paralaje o tasas de tópicos inestables sin justificación. |
| ☐ | **N1 — 0–149** | No realiza mediciones experimentales o afirma exactitud sin datos verificables. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C4. Robustez del lazo, oclusión y resiliencia — Peso 15% — Student Outcome SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, implementa y valida en el firmware el mini-reto del *Watchdog* de pose con `millis()`, demostrando apagado seguro automático ante oclusiones $> 500\text{ ms}$ y reconexión inmediata. |
| ☐ | **N4 — 400–474** | Ejecuta las pruebas de oclusión de tags (Tabla 4), explica el síntoma de pose congelada, mide el tiempo de reconexión autónoma del agente y formula la solución por timeout. |
| ☐ | **N3 — 300–399** | Prueba la oclusión de tags y la desconexión del agente, documentando las respuestas observadas en la Tabla 4. |
| ☐ | **N2 — 150–299** | Confunde la oclusión con desconexión de red o requiere reset físico forzado del microcontrolador para recuperar el lazo. |
| ☐ | **N1 — 0–149** | No ejecuta pruebas de robustez o no presenta evidencia de manejo de fallos. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C5. Trazabilidad en rosbag MCAP y trabajo en equipo — Peso 10% — Student Outcome SO3 / SO5

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, presenta un script de extracción programática (`rosbag2_py`) que grafica la correlación temporal y el desfase exacto entre pose y distancia, con sustentación individual sobresaliente en el Anexo A. |
| ☐ | **N4 — 400–474** | Graba el dataset MCAP con metadatos válidos, genera gráficas claras de evolución temporal ($x, y, d$), entrega el informe técnico estructurado y el Anexo A evidencia autoría individual. |
| ☐ | **N3 — 300–399** | Entrega el dataset grabado con `ros2 bag info`, informe con tablas llenas y Anexo A con respuestas verificables. |
| ☐ | **N2 — 150–299** | Dataset con pérdida de mensajes, tablas incompletas o Anexo A con respuestas genéricas sin evidencia de autoría. |
| ☐ | **N1 — 0–149** | No entrega rosbag ni informe, o el Anexo A demuestra ausencia de participación individual. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

## 10. Consolidado Final de Calificación Académica

| Criterio Evaluado | Peso Oficial (%) | Nivel Marcado | Valor Obtenido (0–500) | Aporte Ponderado |
|---|:---:|:---:|:---:|:---:|
| **C1. Arquitectura y lazo distribuido XRCE-DDS / DDS** | 25% | | | |
| **C2. Selección de interfaces compactas y homografía** | 25% | | | |
| **C3. Exactitud de localización, paralaje y telemetría** | 25% | | | |
| **C4. Robustez del lazo, oclusión y resiliencia** | 15% | | | |
| **C5. Trazabilidad en rosbag MCAP y trabajo en equipo** | 10% | | | |
| **TOTAL CONSOLIDADO** | **100%** | | | **________ / 500** |

```text
Nota Académica sobre 5,0 = Nota Consolidada sobre 500 ÷ 100
Aporte a Talleres del Segundo Corte: T₂ = Nota Académica sobre 5,0 (Componente T_C2)
```

| Resultado Oficial de la Actividad | Registro |
|---|---|
| **Nota de Taller AprilTag sobre 500 puntos** | __________ / 500 |
| **Nota Académica Oficial sobre 5,0** | __________ / 5,0 |
| **Número de Criterios en N3 o superior (Umbral Individual)** | _____ / 5 |
| **¿Cumple el Umbral Individual de Logro ABET (todos en N3+)?** | SÍ ☐ &nbsp;&nbsp;&nbsp;&nbsp; NO ☐ |

---

## 11. Consolidación ABET y Cierre de Mejora Continua

| Student Outcome / Indicador Evaluado | N Estudiantes Evaluables | N en Nivel N3 o Superior | Porcentaje de Logro (%) | Meta Institucional | Hallazgo Docente |
|---|:---:|:---:|:---:|:---:|---|
| **SO1 / Indicador 1.1** (Resolución de Problemas y Lazo) | | | | 70% | |
| **SO2 / Indicador 2.2** (Diseño e Interfaces Compactas) | | | | 70% | |
| **SO6 / Indicador 6.1 y 6.4** (Experimentación y Diagnóstico) | | | | 70% | |
| **SO3 / Indicador 3.1 y 3.3** (Documentación Técnica y Rosbag) | | | | 70% | |
| **SO5 / Indicador 5.1** (Trabajo en Equipo y Roles) | | | | 70% | |

| Campo de Cierre de Ciclo de Mejora Continua | Registro de Gestión Pedagógica |
|---|---|
| **Decisión derivada del hallazgo** | |
| **Acción de mejora continua pedagógica y técnica** | |
| **Responsable de la acción** | Ing. Henry Roncancio |
| **Fecha prevista de seguimiento** | |
| **Evidencia de seguimiento** | |
| **Resultado observado en el segundo ciclo** | |

---

## 12. Cierre y Firmas de Conformidad

**Docente Evaluador:** Ing. Henry Roncancio &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; **Fecha de Evaluación:** ____________________

**Firma del Docente Evaluador:** __________________________________________________

**Observaciones Finales y Retroalimentación Pedagógica:**  
________________________________________________________________________________________________________________________  
________________________________________________________________________________________________________________________  
