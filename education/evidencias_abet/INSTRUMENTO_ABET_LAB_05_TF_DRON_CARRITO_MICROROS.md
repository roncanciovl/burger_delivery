# Instrumento de Evidencia y Calificación — Laboratorio 05: La Transformación del Dron como Telemetría del Carrito (TF2, micro-ROS y Control de Posición en Lazo Cerrado)

> **Documento Único de Entrega y Evaluación ABET.** Asociado a `education/guias_laboratorio/GUIA_LAB_05_TF_DRON_CARRITO_MICROROS.docx`.
> Este documento reúne la entrega integral de cada equipo (dron o carrito): registro de evidencias, tablas de datos experimentales diligenciadas en el laboratorio, respuestas al cuestionario de análisis técnico, Anexo A de comprobación individual y las rúbricas analíticas para la calificación docente y la consolidación ABET. Las Tablas 1, 5, 6 y 7 son de la pareja: los dos equipos registran los mismos valores.

---

## 1. Identificación y Control

| Campo | Registro Oficial |
|---|---|
| **Programa Académico** | Ingeniería Mecatrónica |
| **Asignatura** | ROBOT OPERATING SYSTEM - ROS |
| **Periodo Académico** | 2026-2 |
| **Corte / Instrumento** | Tercer Corte / Laboratorios y evidencias experimentales |
| **Actividad Evaluada** | Laboratorio 05 — La transformación del dron como telemetría del carrito: TF2, micro-ROS y control de posición en lazo cerrado |
| **Número de Grupo / Subgrupo** | |
| **Estudiante 1 (Nombre y Código)** | |
| **Estudiante 2 (Nombre y Código)** | |
| **Estudiante 3 (si aplica)** | |
| **Rol del Equipo en la Pareja** | Equipo del dron ☐ &nbsp;&nbsp;&nbsp;&nbsp; Equipo del carrito ☐ |
| **Pareja (`NN`), `ROS_DOMAIN_ID` y grupo del otro equipo** | `NN` = ____ &nbsp;&nbsp;&nbsp;&nbsp; `ROS_DOMAIN_ID` = ______ &nbsp;&nbsp;&nbsp;&nbsp; Grupo: ____ |
| **Modos de Cámara Usados** | S · Simulado ☐ &nbsp;&nbsp;&nbsp;&nbsp; F · Fija ☐ &nbsp;&nbsp;&nbsp;&nbsp; V · Vuelo ☐ |
| **Repositorio y Commit Evaluado (SHA)** | |
| **Nombre del Archivo de Entrega** | `C3_L05_G<grupo>_<codigo1>_<codigo2>_v1.docx` |
| **Anexos Obligatorios del Envío** | Salida de `ros2 bag info` de `lab05_parNN_faseA` y `lab05_parNN_faseB`, y el `frames_<fecha>.pdf` de `view_frames` |
| **Fecha de Realización en Laboratorio** | |
| **Fecha de Entrega del Documento** | |
| **Docente Evaluador** | Ing. Henry Roncancio |
| **Versión del Instrumento** | Versión 1.0 (2026-2) |
| **Unidad de Análisis / Captura** | Equipo colaborativo con comprobación individual (Anexo A) |

---

## 2. Parámetros de Assessment

| Parámetro | Regla Institucional y Metodológica Adoptada |
|---|---|
| **Población o cohorte** | Censo completo de estudiantes matriculados que presentan el Laboratorio 05 en el periodo 2026-2. |
| **Momento de medición** | Sesión experimental presencial por parejas dron–carrito (Fases 0 a 6) y entrega documental única por equipo con tablas llenas, información de las bolsas de las Fases 4 y 5 y el árbol de TF. |
| **Evaluador** | Docente titular de la asignatura ROBOT OPERATING SYSTEM - ROS. |
| **Umbral individual de logro** | Nivel **N3 o superior** (puntaje mínimo de 300 sobre 500) en cada uno de los indicadores de desempeño evaluados. |
| **Meta de cohorte** | Al menos el **70% de los estudiantes evaluables** debe alcanzar el nivel N3 o superior en cada indicador. |
| **Regla de muestreo** | Censo 100%: no se utiliza muestreo; se evalúa la totalidad de los estudiantes y equipos exigibles. |
| **Evidencia faltante** | Entrega exigible sin evidencia obligatoria verificable: se califica en N1 con puntaje 0. Retiro oficial o causa justificada autorizada: `NA / no evaluado` (se excluye del denominador ABET). |
| **Regla de trabajo colaborativo** | Informe único por equipo complementado obligatoriamente con el Anexo A de comprobación individual. Cada equipo se califica por las evidencias de su rol y por las de la pareja. Sin comprobación individual suficiente no se infiere logro individual ABET. |
| **Regla de datos** | Toda cifra de las Tablas 3 a 7 debe poder rastrearse a una bolsa, a una salida de `tf2_echo` o de `ros2 topic echo`, o a una medición física con cinta. El error final del carrito se mide con cinta, no con la TF. Un dato que no pueda rastrearse se considera evidencia no verificable. |

---

## 3. Niveles de Desempeño para Zubatronic/SGDE

| Nivel | Intervalo Zubatronic | Valor Guía | Interpretación y Criterio de Logro |
|---|---:|:---:|---|
| **N5** | 475–500 | 500 | **Excelente:** Evidencia completa y reproducible; explica cada diferencia entre fases con datos. |
| **N4** | 400–474 | 450 | **Bueno:** Desempeño correcto con omisiones menores que no comprometen la validez de las mediciones ni la seguridad. |
| **N3** | 300–399 | 350 | **Aceptable:** Completa las fases con evidencia verificable. **Es el umbral individual de logro.** |
| **N2** | 150–299 | 250 | **Cumplimiento parcial:** Evidencia incompleta, contrato incumplido o lazo sin medir. |
| **N1** | 0–149 | 100 | **No cumple:** Evidencia mínima, o vuelo o movimiento del carrito fuera del protocolo. Sin evidencia obligatoria se registra 0. |

---

## 4. Alineación de Criterios, RAE y Student Outcomes

| Criterio | Peso | Student Outcome Principal | Indicador de Desempeño Literal del Programa | Evidencia Directa Obligatoria |
|---|:---:|:---:|---|---|
| **C1. Transformación `tag_mesa → tag_carrito1`** | 25% | **SO1** | **1.1.** Aplica principios matemáticos (transformaciones homogéneas, homografías y cuaterniones) para resolver problemas espaciales en robótica. | **Dron:** TF publicada con la estampa de la imagen, verificada desde la PC del carrito, y exactitud y paralaje medidos (E3, E4; Tabla 3). **Carrito:** conversión del cuaternión a $\theta$ comparada con `tf2_echo` (E6). |
| **C2. Integración TF2 – micro-ROS** | 20% | **SO2** | **2.2.** Selecciona interfaces y mensajes respetando las restricciones de ancho de banda y memoria del ESP32, y justifica sus limitaciones. | Pose recibida en el ESP32 por el relevo (Fase A) y por `/tf` con la biblioteca regenerada (Fase B), y efecto de la carga en `/tf` (E6, E7; Tabla 6). |
| **C3. Control de posición y experimentación** | 25% | **SO6** | **6.1.** Diseña y ejecuta pruebas de lazo cerrado, midiendo la exactitud de la pose, su edad y frecuencia, y la respuesta del nodo embebido. | Tres metas por fase con error final medido con cinta, edad de la pose y mensajes por segundo, y bolsas de las Fases 4 y 5 (E6, E7; Tablas 5 y 6). |
| **C4. Seguridad del vuelo, del carrito y de la celda** | 20% | **SO4** | **4.1.** Identifica riesgos de seguridad física, paradas de emergencia y celdas robóticas. | Prueba de banco con ruedas levantadas, metas en la zona de trabajo, vuelo autorizado y las cinco fallas de la Tabla 7 con su tiempo de paro (E5, E8, E9). |
| **C5. Contrato, trazabilidad y trabajo entre equipos** | 10% | **SO3 / SO5** | **3.1 - 3.3.** Elabora documentación técnica reproducible y registra evidencias.<br>**5.1.** Define roles técnicos y coordina la ejecución en equipo. | Contrato de la Tabla 1 firmado por los dos equipos, bolsas, árbol de TF y Anexo A individual (E1, E6, E7). |
| **TOTAL** | **100%** | | | |

---

## 5. Registro de Evidencias de la Práctica (E1–E9)

| Código | Evidencia Requerida | Localizador en el Documento / Repositorio / Archivo |
|---|---|---|
| **E1** | **Pareja:** Tabla 1 firmada por los dos equipos y `ros2 node list` vacío en el dominio de la pareja al empezar. | |
| **E2** | **Dron:** `ros2 topic info`, `hz`, `bw` y `echo --field header` de la imagen del dron, con la estampa distinta de cero. | |
| **E3** | **Dron:** log de arranque del localizador (`Publicando en: /tf (tag_mesa -> tag_carrito1, …)`), `tf2_echo tag_mesa tag_carrito1` ejecutado desde la PC del carrito y el `frames_<fecha>.pdf`. | |
| **E4** | **Dron:** foto de las tres posiciones marcadas con cinta, las 20 lecturas de cada una y la medición del paralaje (Tabla 3). | |
| **E5** | **Carrito:** Tabla 4 y video o capturas de la prueba de banco: ruedas girando con la pose simulada y paro al detener el localizador. | |
| **E6** | **Pareja, Fase A:** `ros2 bag info lab05_parNN_faseA`, capturas de `distance_to_goal`, `edad_pose_ms` y `mensajes_por_s`, y foto de la medición del error final con cinta. | |
| **E7** | **Pareja, Fase B:** `ls src/tf2_msgs/msg/tf_message.h` en la biblioteca regenerada, `ros2 bag info lab05_parNN_faseB` y los datos de la Tabla 6 con y sin el publicador de ruido. | |
| **E8** | **Pareja:** registro de las cinco fallas de la Tabla 7 (salida de `cmd_aplicado` o gráfica de la bolsa con el instante de la falla y el del paro). | |
| **E9** | **Pareja:** autorización de vuelo y lista de chequeo de seguridad de la sesión (zona delimitada, protecciones, Kinova en reposo); si no hubo vuelo, la constancia de que todo se hizo en Modo F. | |

---

## 6. Tablas de Registro Experimental (Diligenciadas por el Equipo)

> **Instrucciones para el equipo:** Diligencie de forma completa y precisa cada una de las siguientes siete tablas con los datos reales obtenidos durante la sesión de laboratorio. Las Tablas 2 y 3 las mide el equipo del dron y la Tabla 4 el del carrito; las Tablas 1, 5, 6 y 7 son de la pareja y deben coincidir en los dos instrumentos.

### Tabla 1: Contrato de Integración (Fase 0)

| Elemento | Valor Acordado |
|---|---|
| **Pareja `NN` y `ROS_DOMAIN_ID`** | |
| **ID y `tag_size_m` del `tag_mesa`** | |
| **ID del tag del carrito y `tag_height_m`** | |
| **Marco padre y marco hijo de la TF** | `tag_mesa` → `tag_carrito1` |
| **Tópico de la imagen del dron y estampa usada** | |
| **Tópico de la pose para el ESP32 (Fase A) y de la meta** | `/burger_car_01/pose_mesa`, `/burger_car_01/meta` |
| **Edad máxima aceptada y timeout de pose** | 300 ms (relevo), 500 ms y 300 ms (ESP32) |
| **Firmas del equipo del dron y del equipo del carrito** | |

---

### Tabla 2: Imagen del Dron (Fase 1)

| Ítem | Valor |
|---|---|
| **Dron, cámara y cómo llega la imagen a ROS 2** | |
| **Resolución, frecuencia (Hz) y ancho de banda de la imagen comprimida** | |
| **¿La estampa de la imagen es distinta de cero?** | |
| **IDs detectados y cuál es el `tag_mesa`** | |

---

### Tabla 3: Exactitud de la TF (Fase 2, Modo F)

| Posición | Real $(x, y, \theta)$ Medida con Cinta | Media de la TF $(x, y, \theta)$ | Desviación $(x, y, \theta)$ | Error de Posición (mm) |
|:---:|---|---|---|:---:|
| **P1** | | | | |
| **P2** | | | | |
| **P3** | | | | |
| **Paralaje** (P1, tag en la mesa frente a tag en el techo) | | | | |

---

### Tabla 4: Puesta a Punto del Carrito (Fase 3)

| Ítem | Valor |
|---|---|
| **Puente H, pines y sentido de giro verificado** | |
| **`PWM_MIN` (zona muerta) y `V_RUEDA_MAX` (m/s)** | |
| **Prueba de banco: tiempo hasta el paro al detener la pose (ms)** | |

---

### Tabla 5: Lazo Cerrado, Fase A frente a Fase B (Fases 4 y 5)

| Meta $(x, y, \theta)$ | Fase | Modo de Cámara | Tiempo de Llegada (s) | Error Final con Cinta (mm, °) | Edad Media de la Pose (ms) | Mensajes por s en el ESP32 |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| | A | F | | | | |
| | A | F | | | | |
| | A | F | | | | |
| | A | V | | | | |
| | B | F | | | | |
| | B | F | | | | |
| | B | F | | | | |

---

### Tabla 6: Carga en `/tf` (Fase 5)

| Fase | Publicador de Ruido a 100 Hz | Mensajes por s en el ESP32 | Edad Media de la Pose (ms) | ¿Llegó a la Meta? | Error Final (mm) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| A | No | | | | |
| A | Sí | | | | |
| B | No | | | | |
| B | Sí | | | | |

---

### Tabla 7: Robustez (Fase 6)

| N.º | Falla | Tiempo hasta el Paro (ms) | ¿Retomó Solo? | Observación |
|:---:|---|:---:|:---:|---|
| **6.1** | Oclusión del tag del carrito | | | |
| **6.2** | Oclusión del `tag_mesa` | | | |
| **6.3** | Pose vieja (estampa 0) | | | |
| **6.4** | Pérdida del agente | | | |
| **6.5** | Pérdida de la cámara del dron | | | |

---

## 7. Cuestionario de Análisis y Discusión Técnica (Respondido por el Equipo)

### Pregunta 1: Exactitud y Paralaje
*Con la Tabla 3, separe el error de la TF en sus fuentes: medición de `tag_size_m`, colocación del `tag_mesa`, paralaje por la altura del carrito y ruido del detector. ¿Cuál domina y cómo se reduciría?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 2: Fase A frente a Fase B
*Con las Tablas 5 y 6, compare la edad de la pose, la tasa de mensajes y el error final en las dos fases. ¿Qué gana y qué pierde el ESP32 al leer `/tf` directamente?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 3: Composición de la Edad de la Pose
*¿De qué tramos se compone la edad que mide el ESP32 (captura, red del dron, detección, DDS, agente, WiFi)? ¿Cuáles cambian entre las fases A y B y cuáles entre los modos F y V?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 4: Seguridad del Lazo
*Con la Tabla 7, ¿qué falla tarda más en detener el carrito y por qué? ¿Qué distancia recorre el carrito a $v_{\max}$ durante ese tiempo?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 5: Marco de Referencia de la Pose
*El localizador expresa el carrito en el marco del `tag_mesa`. ¿Por qué la pose del carrito no cambia cuando el dron se mueve, y qué pasaría si se publicara en el marco de la cámara del dron?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 6: Estampa de la Transformación
*¿Por qué la TF lleva la estampa de la imagen y no la hora en que el localizador terminó de calcularla? ¿Qué mediría el ESP32 si fuera la segunda?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 7: Memoria Reservada en el ESP32
*El firmware reserva memoria para 4 transformaciones por `TFMessage` y cadenas de 32 caracteres. ¿Qué le pasa a un mensaje de `/tf` con 10 transformaciones, como los del driver del Kinova? ¿Y a uno con un `child_frame_id` de 40 caracteres?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 8: Sincronización del Reloj
*¿Qué haría falta para que el ESP32 midiera la edad de la pose sin `rmw_uros_sync_session()`? ¿Qué error introduciría no sincronizar el reloj?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 9: Dos Parejas en un Mismo Dominio
*Si dos parejas compartieran el dominio, ¿qué vería el ESP32 de cada carrito en `/tf` en la Fase B? ¿Qué cambiaría en el contrato para que pudieran compartirlo?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 10: Conclusiones del Equipo
*Redacte tres conclusiones: (1) sobre el contrato de integración, qué tuvo que fijarse para que dos equipos que no comparten código cerraran un lazo de control; (2) una conclusión cuantitativa sobre la exactitud del lazo (Tablas 3 y 5); y (3) sobre cuándo conviene que un microcontrolador lea `/tf` directamente y cuándo a través de un relevo (Tablas 5 y 6).*

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
| **1. Rol y tareas técnicas desarrolladas:** Indique su equipo (dron o carrito) y describa sus responsabilidades específicas (ej. imagen del dron en ROS 2, localizador y TF, motores, firmware, relevo, biblioteca con `tf2_msgs`, mediciones o bolsas). | |
| **2. El contrato de integración:** Elija un elemento de la Tabla 1 que dependía de usted y explique qué le habría pasado al otro equipo si lo hubiera cambiado sin avisar. | |
| **3. La transformación `tag_mesa → tag_carrito1`:** Explique el marco padre, el marco hijo, la estampa y el cuaternión de la TF, y cómo se recupera $\theta$ en el ESP32. | |
| **4. Fase A frente a Fase B:** Con datos de las Tablas 5 y 6, explique una diferencia medida entre leer la pose a través del relevo y leer `/tf` directamente. | |
| **5. Autoría y reproducibilidad:** Identifique qué comandos, filas de tablas, partes del firmware o del localizador, capturas o bolsas fueron ejecutados y documentados directamente por usted. | |

**Estado de comprobación individual Estudiante 1:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐
**Firma del Estudiante 1:** ________________________________________

---

### Estudiante 2: __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Indique su equipo (dron o carrito) y describa sus responsabilidades específicas (ej. imagen del dron en ROS 2, localizador y TF, motores, firmware, relevo, biblioteca con `tf2_msgs`, mediciones o bolsas). | |
| **2. El contrato de integración:** Elija un elemento de la Tabla 1 que dependía de usted y explique qué le habría pasado al otro equipo si lo hubiera cambiado sin avisar. | |
| **3. La transformación `tag_mesa → tag_carrito1`:** Explique el marco padre, el marco hijo, la estampa y el cuaternión de la TF, y cómo se recupera $\theta$ en el ESP32. | |
| **4. Fase A frente a Fase B:** Con datos de las Tablas 5 y 6, explique una diferencia medida entre leer la pose a través del relevo y leer `/tf` directamente. | |
| **5. Autoría y reproducibilidad:** Identifique qué comandos, filas de tablas, partes del firmware o del localizador, capturas o bolsas fueron ejecutados y documentados directamente por usted. | |

**Estado de comprobación individual Estudiante 2:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐
**Firma del Estudiante 2:** ________________________________________

---

## 9. Selección del Nivel Alcanzado por Criterio (Rúbricas Docentes)

Marque con una **X** una sola casilla por criterio y registre el valor entero (0 a 500) dentro de la banda correspondiente. En los criterios C1 y C4 la evidencia se juzga según el rol del equipo (dron o carrito).

### C1. Transformación `tag_mesa → tag_carrito1` — Peso 25% — Student Outcome SO1

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, relaciona la homografía del localizador con la TF publicada, separa con datos el error de paralaje del resto de fuentes y propone cómo corregirlo (por ejemplo, con la altura del tag y la pose de la cámara). |
| ☐ | **N4 — 400–474** | **Dron:** publica la TF con la estampa de la imagen, la verifica con `tf2_echo` y `view_frames` desde la PC del carrito y completa la Tabla 3 con las tres posiciones y el paralaje. **Carrito:** recupera $\theta$ del cuaternión y comprueba que $(x, y, \theta)$ del ESP32 coincide con `tf2_echo`. |
| ☐ | **N3 — 300–399** | La TF `tag_mesa → tag_carrito1` se publica y se lee con los marcos del contrato, y la Tabla 3 (dron) o la conversión a $(x, y, \theta)$ (carrito) están completas. |
| ☐ | **N2 — 150–299** | La TF usa marcos distintos a los del contrato, no tiene estampa de la imagen, o la Tabla 3 está incompleta o sin medición con cinta. |
| ☐ | **N1 — 0–149** | No se publica o no se lee la transformación, o no hay evidencia verificable de la Fase 2. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C2. Integración TF2 – micro-ROS — Peso 20% — Student Outcome SO2

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, explica con la Tabla 6 cómo la carga de `/tf` afecta al ESP32 y justifica qué fase usaría en el robot del proyecto y por qué. |
| ☐ | **N4 — 400–474** | El ESP32 recibe la pose en las dos fases: por el relevo (`/burger_car_01/pose_mesa`) y suscribiendo `/tf` con la biblioteca regenerada con `tf2_msgs`; explica la reserva de memoria y el nombre absoluto `/tf`. |
| ☐ | **N3 — 300–399** | El lazo funciona en la Fase A y se intenta la Fase B, documentando la regeneración de la biblioteca y su resultado. |
| ☐ | **N2 — 150–299** | Sólo se completa parte de la Fase A, o la PC y el ESP32 quedan en dominios distintos sin diagnosticarlo. |
| ☐ | **N1 — 0–149** | El ESP32 no recibe la pose por ninguna de las dos vías. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C3. Control de posición y experimentación — Peso 25% — Student Outcome SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, descompone la edad de la pose en sus tramos, compara los modos F y V y relaciona el error final con la exactitud de la TF (Tablas 3 y 5). |
| ☐ | **N4 — 400–474** | Llega a las 3 metas en cada fase, mide el error final con cinta, registra la edad de la pose y los mensajes por segundo, y completa las Tablas 5 y 6 con las bolsas de las Fases 4 y 5. |
| ☐ | **N3 — 300–399** | El carrito llega a las metas en la Fase A con el error final medido con cinta y la Tabla 5 completa. |
| ☐ | **N2 — 150–299** | El lazo funciona sin mediciones, o el error final se toma de la TF en lugar de medirse con cinta. |
| ☐ | **N1 — 0–149** | El lazo no se cierra o no hay registro de las pruebas. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C4. Seguridad del vuelo, del carrito y de la celda — Peso 20% — Student Outcome SO4

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, calcula la distancia que recorre el carrito antes de detenerse en cada falla de la Tabla 7 y propone una mejora concreta a las salvaguardas. |
| ☐ | **N4 — 400–474** | Prueba primero con las ruedas levantadas, mantiene metas dentro de la zona de trabajo y $v_{\max}$ = 0.10 m/s, vuela sólo con autorización en la zona delimitada y verifica las cinco fallas de la Tabla 7 con paro en menos de 0.3 s. |
| ☐ | **N3 — 300–399** | Respeta el protocolo de seguridad (ruedas levantadas, zona de trabajo, Kinova en reposo) y verifica al menos el paro por oclusión y por pérdida del agente. |
| ☐ | **N2 — 150–299** | Omite la prueba de banco, deja metas cerca del borde o completa sólo parte de la Tabla 7. |
| ☐ | **N1 — 0–149** | Vuela o mueve el carrito fuera del protocolo, desactiva o modifica las salvaguardas del firmware, o publica en el dominio del Kinova. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C5. Contrato, trazabilidad y trabajo entre equipos — Peso 10% — Student Outcome SO3 / SO5

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, el contrato anticipa los cambios que exigiría compartir el dominio con otra pareja y el Anexo A demuestra un dominio técnico individual sobresaliente. |
| ☐ | **N4 — 400–474** | La Tabla 1 está firmada por los dos equipos y se cumplió en toda la sesión; se entregan las bolsas de las Fases 4 y 5 y el `frames_<fecha>.pdf`, y el Anexo A evidencia autoría y comprensión. |
| ☐ | **N3 — 300–399** | Entrega el instrumento con las Tablas 1 a 7, la información de las bolsas y el Anexo A con respuestas verificables. |
| ☐ | **N2 — 150–299** | El contrato no se firmó o se incumplió sin registrarlo, faltan bolsas o el Anexo A tiene respuestas genéricas. |
| ☐ | **N1 — 0–149** | No entrega el instrumento ni las bolsas, o el Anexo A demuestra ausencia de participación. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

## 10. Consolidado Final de Calificación Académica

| Criterio Evaluado | Peso Oficial (%) | Nivel Marcado | Valor Obtenido (0–500) | Aporte Ponderado |
|---|:---:|:---:|:---:|:---:|
| **C1. Transformación `tag_mesa → tag_carrito1`** | 25% | | | |
| **C2. Integración TF2 – micro-ROS** | 20% | | | |
| **C3. Control de posición y experimentación** | 25% | | | |
| **C4. Seguridad del vuelo, del carrito y de la celda** | 20% | | | |
| **C5. Contrato, trazabilidad y trabajo entre equipos** | 10% | | | |
| **TOTAL CONSOLIDADO** | **100%** | | | **________ / 500** |

```text
Nota Académica sobre 5,0 = Nota Consolidada sobre 500 ÷ 100
Aporte a Laboratorios del Tercer Corte: L₃ = Nota Académica sobre 5,0
```

| Resultado Oficial de la Actividad | Registro |
|---|---|
| **Nota de Laboratorio 05 sobre 500 puntos** | __________ / 500 |
| **Nota Académica Oficial sobre 5,0** | __________ / 5,0 |
| **Número de Criterios en N3 o superior (Umbral Individual)** | _____ / 5 |
| **¿Cumple el Umbral Individual de Logro ABET (todos en N3+)?** | SÍ ☐ &nbsp;&nbsp;&nbsp;&nbsp; NO ☐ |

---

## 11. Consolidación ABET y Cierre de Mejora Continua

| Student Outcome / Indicador Evaluado | N Estudiantes Evaluables | N en Nivel N3 o Superior | Porcentaje de Logro (%) | Meta Institucional | Hallazgo Docente |
|---|:---:|:---:|:---:|:---:|---|
| **SO1 / Indicador 1.1** (Transformación y Localización Visual) | | | | 70% | |
| **SO2 / Indicador 2.2** (Integración TF2 – micro-ROS) | | | | 70% | |
| **SO6 / Indicador 6.1** (Control de Posición y Experimentación) | | | | 70% | |
| **SO4 / Indicador 4.1** (Seguridad del Vuelo, del Carrito y de la Celda) | | | | 70% | |
| **SO3 / Indicador 3.1 - 3.3** (Documentación y Trazabilidad) | | | | 70% | |
| **SO5 / Indicador 5.1** (Trabajo entre Equipos y Roles) | | | | 70% | |

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
