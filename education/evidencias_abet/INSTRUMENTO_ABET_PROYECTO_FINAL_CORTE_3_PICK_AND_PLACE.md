# Instrumento de Evidencia y Calificación — Proyecto Final del Tercer Corte: Pick and Place de la Caja de Hamburguesa con MoveIt 2, la Pinza y la Visión del Kinova Gen3

> **Documento Único de Entrega y Evaluación ABET.** Asociado a `education/proyectos_evaluables/PROYECTO_FINAL_CORTE_3_PICK_AND_PLACE_MOVEIT.md`.
> Este documento reúne la entrega integral del grupo: registro de evidencias, tablas de datos experimentales, respuestas al cuestionario de análisis técnico, Anexo A de comprobación individual y las rúbricas analíticas para la calificación docente y la consolidación ABET.

---

## 1. Identificación y Control

| Campo | Registro Oficial |
|---|---|
| **Programa Académico** | Ingeniería Mecatrónica |
| **Asignatura** | ROBOT OPERATING SYSTEM - ROS |
| **Periodo Académico** | 2026-2 |
| **Corte / Instrumento** | Tercer Corte / Entrega final y sustentación técnica (`E_C3` — 30%) |
| **Actividad Evaluada** | Proyecto Final — Pick and place de la caja de hamburguesa con MoveIt 2, la pinza y la visión del Kinova Gen3 |
| **Número de Grupo / Subgrupo** | |
| **Estudiante 1 (Nombre y Código)** | |
| **Estudiante 2 (Nombre y Código)** | |
| **Estudiante 3 (si aplica)** | |
| **Paquete y Commit Evaluado (SHA)** | `burger_mision_g____` &nbsp;&nbsp;&nbsp;&nbsp; SHA: |
| **Nombre del Archivo de Entrega** | `C3_E03_G<grupo>_<codigo1>_<codigo2>_v1.docx` |
| **Anexos Obligatorios del Envío** | Video de un ensayo completo, `ros2 bag info` de las bolsas de los tres ensayos, informes del ejecutor (`*.jsonl`) y `metas_mision_gNN.yaml` de cada ensayo |
| **Fecha de Realización / Sustentación** | |
| **Fecha de Entrega del Documento** | |
| **Docente Evaluador** | Ing. Henry Roncancio |
| **Versión del Instrumento** | Versión 1.0 (2026-2) |
| **Unidad de Análisis / Captura** | Equipo colaborativo con comprobación individual (Anexo A) |

---

## 2. Parámetros de Assessment

| Parámetro | Regla Institucional y Metodológica Adoptada |
|---|---|
| **Población o cohorte** | Censo completo de estudiantes matriculados que presentan el proyecto final del Corte 3 en el periodo 2026-2. |
| **Momento de medición** | Ensayos con el robot real por turnos (precedidos de ensayo con el driver fake), entrega documental única y sustentación con reto en vivo. |
| **Evaluador** | Docente titular de la asignatura ROBOT OPERATING SYSTEM - ROS. |
| **Umbral individual de logro** | Nivel **N3 o superior** (puntaje mínimo de 300 sobre 500) en cada uno de los indicadores de desempeño evaluados. |
| **Meta de cohorte** | Al menos el **70% de los estudiantes evaluables** debe alcanzar el nivel N3 o superior en cada indicador. |
| **Regla de muestreo** | Censo 100%: no se utiliza muestreo; se evalúa la totalidad de los estudiantes y equipos exigibles. |
| **Evidencia faltante** | Entrega exigible sin evidencia obligatoria verificable: se califica en N1 con puntaje 0. Retiro oficial o causa justificada autorizada: `NA / no evaluado` (se excluye del denominador ABET). |
| **Regla de trabajo colaborativo** | Informe único por grupo complementado obligatoriamente con el Anexo A de comprobación individual. Sin comprobación individual suficiente no se infiere logro individual ABET. |
| **Regla de datos** | Toda cifra de las Tablas 2 a 6 (incluida la 3b) debe poder rastrearse a una bolsa, a un informe del ejecutor, a un YAML de metas o a una medición física con cinta. El error de colocación se mide con cinta, no con la TF. |

---

## 3. Niveles de Desempeño para Zubatronic/SGDE

| Nivel | Intervalo Zubatronic | Valor Guía | Interpretación y Criterio de Logro |
|---|---:|:---:|---|
| **N5** | 475–500 | 500 | **Excelente:** Evidencia completa y reproducible; explica cada resultado con sus causas y datos. |
| **N4** | 400–474 | 450 | **Bueno:** Desempeño correcto con omisiones menores que no comprometen la validez de las mediciones ni la seguridad. |
| **N3** | 300–399 | 350 | **Aceptable:** Misión ejecutada con evidencia verificable. **Es el umbral individual de logro.** |
| **N2** | 150–299 | 250 | **Cumplimiento parcial:** Misión incompleta, coordenadas escritas a mano o pruebas sin medir. |
| **N1** | 0–149 | 100 | **No cumple:** Evidencia mínima, o operación del robot fuera del protocolo. Sin evidencia obligatoria se registra 0. |

---

## 4. Alineación de Criterios, RAE y Student Outcomes

| Criterio | Peso | Student Outcome Principal | Indicador de Desempeño Literal del Programa | Evidencia Directa Obligatoria |
|---|:---:|:---:|---|---|
| **C1. Percepción RGB-D, corrección de perspectiva con tf2 y árbol de transformaciones** | 25% | **SO1** | **1.1.** Analiza las relaciones entre fenómenos físicos y modelos matemáticos de orientación y transformaciones, validando el árbol de transformaciones (TF2) en sistemas robóticos distribuidos. | Pose de observación, verificación del `tag_mesa`, pose 3D de los tags con RGB-D y tf2, TF del carrito y pose de la bandeja en `base_link` (E2, E3, E4; Tablas 1, 2, 3 y 3b). |
| **C2. Pick and place con MoveIt 2 y la pinza** | 25% | **SO2** | **2.3.** Implementa y valida trayectorias libres de colisiones configurando pipelines de planificación de movimiento en MoveIt 2 sobre entornos industriales modelados experimentalmente. | Escena con la caja y el carrito, misión en modo seco y en el robot real, tres ensayos (E5, E6, E7; Tablas 4 y 5). |
| **C3. Validación experimental y diagnóstico** | 20% | **SO6** | **6.1.** Diseña y ejecuta pruebas de lazo cerrado, midiendo exactitud y respuesta del sistema.<br>**6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos por capas. | Pruebas P1 a P6, bolsas e informes, reto en vivo (E8, E9, E10; Tablas 5 a 7). |
| **C4. Seguridad en la operación del robot real** | 20% | **SO4** | **4.1.** Identifica riesgos de seguridad física, paradas de emergencia y celdas robóticas. | Modo seco de cada ensayo revisado por el docente, abortos seguros en P4 a P6, límites sin modificar (E6, E9; Tablas 4 y 6). |
| **C5. Trazabilidad y trabajo en equipo** | 10% | **SO3 / SO5** | **3.1 - 3.3.** Elabora documentación técnica reproducible y registra evidencias.<br>**5.1.** Define roles técnicos y coordina la ejecución en equipo. | Paquete con `README.md`, commits de todos, video, bolsas y Anexo A individual (E1, E7, E8). |
| **TOTAL** | **100%** | | | |

> **Nota de alineación:** los indicadores se citan como aparecen en los instrumentos de las prácticas anteriores y en el proyecto integrador de MoveIt del repositorio. Verifíquelos contra el syllabus vigente.

---

## 5. Registro de Evidencias de la Práctica (E1–E10)

| Código | Evidencia Requerida | Localizador en el Documento / Repositorio / Archivo |
|---|---|---|
| **E1** | Enlace al paquete `burger_mision_gNN` y al commit evaluado, con el `README.md` (arquitectura, árbol de TF y cómo reproducir la misión). | |
| **E2** | Las seis posiciones articulares de la pose de observación y una imagen de la cámara desde esa pose con el `tag_mesa` y el tag del carrito visibles. | |
| **E3** | Salida de la verificación del `tag_mesa`: pose medida con la cámara en `base_link`, pose del URDF y error (Tabla 2). | |
| **E4** | Imagen RGB con las esquinas detectadas de los dos tags y la profundidad registrada en falso color de la misma escena; `tf2_echo tag_mesa_detectado tag_carrito1_detectado`, `tf2_echo base_link car1_delivery_tray_frame`, el `frames_<fecha>.pdf` de `view_frames` con la cadena completa, las 20 lecturas de la Tabla 3 y las mediciones de la Tabla 3b. | |
| **E5** | Captura de RViz con la caja en la zona de recogida y el carrito como obstáculos (en el modo hoja, sólo la caja), y otra con la caja adjunta a la pinza durante el traslado. | |
| **E6** | Informe del ejecutor en modo seco de la misión completa de cada ensayo, revisado por el docente. | |
| **E7** | Video de un ensayo completo con el robot real, desde la pose de observación hasta `home`. | |
| **E8** | `ros2 bag info`, informe del ejecutor (`*.jsonl`) y `metas_mision_gNN.yaml` de cada uno de los tres ensayos de la P3. | |
| **E9** | Salida de la misión en las pruebas de falla P4, P5 y P6: estado en que abortó y mensaje (Tabla 6). | |
| **E10** | Registro del reto en vivo: falla introducida, capa diagnosticada, evidencia usada y tiempo de resolución (Tabla 7). | |

---

## 6. Tablas de Registro Experimental (Diligenciadas por el Equipo)

> **Instrucciones para el equipo:** Diligencie de forma completa y precisa cada una de las siguientes tablas con los datos reales. Antes de cada tabla se explica qué significa cada columna y de dónde sale cada valor.

### Tabla 1: Configuración de la Celda y de la Misión

Registra los valores medidos y elegidos por el grupo que la misión usa como parámetros. Cada fila es un dato:

- **IDs y tamaños de los tags:** IDs detectados del `tag_mesa` y del tag del carrito, y lado del cuadro negro de cada uno, medido con calibrador.
- **Registro de la profundidad:** traslación de `tf2_echo camera_color_frame camera_depth_frame` y el `frame_id` de `/camera/depth_registered/image_rect`, que debe ser `camera_color_frame`.
- **Modo de destino:** `destino:=carrito` o `destino:=hoja` (tag del carrito impreso en una hoja sobre la mesa, sección 4.1 de la especificación).
- **Altura del tag del carrito:** `tag_height_m`, distancia vertical del tag a la cara de la mesa, medida con cinta. En el modo hoja es el grosor de la hoja, prácticamente 0.
- **Caja:** largo, ancho (lado de agarre) y alto, en mm.
- **Zona de recogida:** centro de la zona en el marco del `tag_mesa`, medido con cinta.
- **Pose de observación:** las seis posiciones articulares en radianes.
- **Alturas de la secuencia:** distancia de pre-agarre y de pre-entrega sobre la caja y el destino (bandeja u hoja), y altura de suelta.

| Ítem | Valor |
|---|---|
| **IDs del `tag_mesa` y del tag del carrito; lado de cada uno (m)** | |
| **`camera_color_frame → camera_depth_frame` (m) y `frame_id` de la profundidad registrada** | |
| **Modo de destino (`carrito` / `hoja`)** | |
| **`tag_height_m` del tag del carrito (m)** | |
| **Dimensiones de la caja: largo × ancho de agarre × alto (mm)** | |
| **Centro de la zona de recogida en el marco del `tag_mesa` (m)** | |
| **Pose de observación: `joint_1` a `joint_6` (rad)** | |
| **Alturas de pre-agarre, pre-entrega y suelta (m)** | |

---

### Tabla 2: Verificación del `tag_mesa` (Prueba P1)

Compara dónde dice el URDF que está el `tag_mesa` con dónde lo mide la cámara. Cada fila es una componente de la pose en `base_link`:

- **URDF:** el valor de `tag_mesa` en `base_link` según la escena (`tf2_echo base_link tag_mesa`).
- **Medido con la cámara:** mediana de 20 lecturas de `tf2_echo base_link tag_mesa_detectado`: la pose RGB-D del `tag_mesa` llevada a `base_link` con tf2.
- **Diferencia:** medido menos URDF. Si el error de posición supera 2 cm, se corrige el URDF midiendo la mesa y se repite.

| Componente | URDF | Medido con la Cámara | Diferencia |
|:---:|:---:|:---:|:---:|
| **x (m)** | | | |
| **y (m)** | | | |
| **z (m)** | | | |
| **Giro alrededor de z (°)** | | | |
| **Error de posición (mm)** | — | — | |

---

### Tabla 3: Estabilidad de la TF del Carrito (Prueba P2, parte a)

Con el carrito quieto, mide cuánto varía la pose que entrega la visión. Las tres primeras filas son de `tag_mesa → tag_carrito1`; la última, de la meta de entrega en `base_link` (la bandeja, o el tag en el modo hoja), que es la que usa la misión:

- **Media y desviación estándar:** sobre 20 lecturas consecutivas de `tf2_echo`.
- **Mediana:** el valor que usa la misión para la meta de entrega.

| Magnitud | Media | Desviación Estándar | Mediana |
|:---:|:---:|:---:|:---:|
| **x del carrito en `tag_mesa` (m)** | | | |
| **y del carrito en `tag_mesa` (m)** | | | |
| **θ del carrito en `tag_mesa` (°)** | | | |
| **Meta de entrega en `base_link`: (x, y, z) (m)** | | | |

---

### Tabla 3b: Exactitud de la Pose RGB-D del Carrito (Prueba P2, parte b)

Mide qué tan bien ubica el método RGB-D con tf2 al tag del carrito en el marco de la mesa. En cada fila el tag del carrito está en una posición marcada con cinta sobre la mesa:

- **Real (cinta):** posición del centro del tag del carrito en el marco del `tag_mesa`, medida con cinta.
- **RGB-D con tf2:** $(x, y)$ de `tf2_echo tag_mesa_detectado tag_carrito1_detectado`, mediana de 20 lecturas.
- **Error (mm):** distancia entre las dos posiciones.
- **$z$ medido y `tag_height_m`:** la altura del tag que da la pose RGB-D y la medida con cinta; deben coincidir.
- **Píxeles válidos (%) y residuo del plano (mm):** calidad de la profundidad dentro del tag del carrito en esa medición.

| Posición | Real (cinta) (x, y) (m) | RGB-D con tf2 (x, y) (m) | Error (mm) | $z$ Medido / `tag_height_m` (m) | Píxeles Válidos (%) | Residuo del Plano (mm) |
|:---:|---|---|:---:|:---:|:---:|:---:|
| **1** | | | | | | |
| **2** | | | | | | |
| **3** | | | | | | |

---

### Tabla 4: Secuencia de la Misión (Modo Seco y Ejecución Real)

Una fila por estado de la secuencia del proyecto (sección 5) que mueve el brazo o la pinza. Los datos salen del informe del ejecutor:

- **Estado:** número y nombre del estado.
- **Tipo de meta:** articular, pose, cartesiana o pinza.
- **Puntos y duración (s):** del plan validado por el ejecutor (no aplica a la pinza).
- **Velocidad pico (rad/s):** la mayor del plan; no puede superar la del laboratorio.
- **Modo seco:** «OK» o el mensaje de bloqueo.
- **Real:** «OK», el `error_code` o, en la pinza, la posición final reportada.

| Estado | Tipo de Meta | Puntos | Duración (s) | Velocidad Pico (rad/s) | Modo Seco | Real |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| 1. Pose de observación | articular | | | | | |
| 3. Abrir pinza | pinza | — | — | — | | |
| 4. Pre-agarre | pose | | | | | |
| 5. Descenso | cartesiana | | | | | |
| 6. Cerrar pinza | pinza | — | — | — | | |
| 8. Ascenso | cartesiana | | | | | |
| 9. Pre-entrega | pose | | | | | |
| 10. Descenso de entrega | cartesiana | | | | | |
| 11. Abrir pinza | pinza | — | — | — | | |
| 13. Retirada | cartesiana | | | | | |
| 14. `home` | articular | | | | | |

---

### Tabla 5: Ensayos de Pick and Place (Prueba P3)

Una fila por ensayo; en cada uno el docente ubica el carrito (o la hoja) en una posición distinta de la zona de entrega:

- **Modo:** `carrito` u `hoja`.

- **Posición del carrito:** la pose del tag del carrito en el marco del `tag_mesa` (mediana de la TF).
- **Agarre y entrega:** «sí» si la caja quedó en la pinza y, después, dentro de la bandeja (o sobre el contorno dibujado en la hoja).
- **Error de colocación (mm):** distancia, medida con cinta, entre el centro de la base de la caja y el centro de la bandeja (en el modo hoja, el centro del tag).
- **Tiempo de ciclo (s):** desde que sale de la pose de observación hasta que vuelve a `home`, según la bolsa.

| Ensayo | Modo | Posición del Carrito (x, y, θ) en `tag_mesa` | ¿Agarre? | ¿Entrega? | Error de Colocación (mm) | Tiempo de Ciclo (s) |
|:---:|:---:|---|:---:|:---:|:---:|:---:|
| **1** | | | | | | |
| **2** | | | | | | |
| **3** | | | | | | |

---

### Tabla 6: Pruebas de Falla (Pruebas P4 a P6)

Una fila por falla provocada. Lo esperado es que la misión se detenga en un estado seguro:

- **Estado en que se detuvo:** el número del estado de la secuencia.
- **Mensaje:** el que imprime la misión o el ejecutor, copiado tal cual.
- **Dato medido:** en P5, la posición final de la pinza; en P6, el tiempo hasta la detención.

| Prueba | Falla Provocada | Estado en que se Detuvo | Mensaje | Dato Medido |
|:---:|---|:---:|---|:---:|
| **P4** | Tag del carrito tapado | | | |
| **P5** | Zona de recogida vacía | | | |
| **P6** | Ctrl+C durante el traslado | | | |

---

### Tabla 7: Reto en Vivo de la Sustentación

Registra la falla que introdujo el docente y cómo la diagnosticó el grupo:

- **Capa:** red y dominio, TF, percepción, planificación o ejecución.
- **Evidencia usada:** el comando, tópico, log o visualización que llevó a la causa.

| Falla Introducida | Capa Diagnosticada | Evidencia Usada | Corrección | Tiempo (min) |
|---|---|---|---|:---:|
| | | | | |

---

## 7. Cuestionario de Análisis y Discusión Técnica (Respondido por el Equipo)

### Pregunta 1: Por Qué Verificar el `tag_mesa`
*La percepción mide el carrito respecto al `tag_mesa`, no respecto a la cámara. Explique por qué entonces un error de 3 cm en la posición del `tag_mesa` del URDF produce un error de 3 cm en la entrega, aunque la cámara esté perfectamente calibrada. Use su resultado de la Tabla 2.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 2: Mediana de la TF del Carrito
*¿Por qué la pose de entrega usa la mediana de 20 lecturas y no la última? Con los datos de la Tabla 3, estime cuánto variaría la colocación si se usara una sola lectura.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 3: Fuentes del Error de Colocación
*Con la Tabla 5, descomponga el error de colocación en sus fuentes: verificación del `tag_mesa`, localización del carrito con RGB-D (registro de la profundidad y calidad del plano del tag), agarre (dónde quedó la caja entre los dedos) y suelta. ¿Cuál domina?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 4: Adjuntar la Caja a la Pinza
*¿Qué podría pasar durante el traslado si la caja no se adjunta a la pinza en la escena de MoveIt? ¿Y si no se separa al soltarla? Relaciónelo con la planificación del retorno a `home`.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 5: Detección del Agarre por la Pinza
*La misión decide si hay caja entre los dedos por la posición final de la pinza (prueba P5). ¿Qué otras señales podrían confirmarlo y qué fallas no detecta este criterio?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 6: Planificar Ahora, Ejecutar Después
*El nodo de misión genera las metas en `base_link` antes de ejecutarlas. ¿Qué ocurre si el carrito se mueve entre la percepción (estado 2) y la entrega (estado 9)? Proponga cómo detectarlo antes de soltar la caja.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 7: Abortar como Respuesta Segura
*Con la Tabla 6, justifique para cada falla por qué abortar en un estado seguro es mejor que intentar completar la misión. ¿En qué estado de la secuencia es más peligroso abortar?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 8: Diagnóstico del Reto en Vivo
*Describa la falla introducida en el reto en vivo, la capa donde la encontraron (red y dominio, TF, percepción, planificación o ejecución) y la evidencia que los llevó a ella.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 9: Corrección de Perspectiva con RGB-D y tf2
*Explique por qué la profundidad debe transformarse del marco del sensor de profundidad al de la cámara de color antes de usar las esquinas del RGB, y qué comprobó para saber que estaba registrada. Explique por qué la cámara inclinada impide pasar de píxeles a metros con una escala fija, y cómo la profundidad registrada y el ajuste de plano convierten cada tag en una pose 3D. Luego explique por qué `lookup_transform('tag_mesa_detectado', 'tag_carrito1_detectado')` no depende de dónde estaba la cámara, por qué las detecciones no pueden llamarse `tag_mesa` y `tag_carrito1`, y qué le indica que $z$ coincida (o no) con `tag_height_m` en la Tabla 3b.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 10: Conclusiones del Equipo
*Redacte tres conclusiones sustentadas en sus datos: (1) sobre la exactitud de la entrega y su fuente dominante de error (Tablas 2, 3 y 5); (2) sobre la arquitectura de la misión: percepción, metas calculadas y ejecución validada; y (3) sobre la seguridad: qué fallas detiene el sistema y cuáles todavía no (Tabla 6).*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

## 8. Anexo A — Comprobación Individual de Desempeño

> **Propósito ABET:** Este anexo verifica la autoría, comprensión técnica y contribución directa individual de cada estudiante, sustentando la asignación del logro individual en los Student Outcomes evaluados.

### Estudiante 1: __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas (pose de observación, verificación del `tag_mesa`, percepción RGB-D, nodo de misión, escena de MoveIt, pinza, pruebas o registros). | |
| **2. El árbol de transformaciones:** Recorra la cadena de `base_link` a `car1_delivery_tray_frame` e indique qué tramo sale del URDF, cuál de la visión y cuál del driver. | |
| **3. La meta de entrega:** Explique cómo se calcula la pose de pre-entrega a partir de la TF del carrito y por qué no tiene coordenadas escritas en el código. | |
| **4. Seguridad:** Elija una falla de la Tabla 6 que usted provocó y explique en qué estado abortó la misión y qué la detuvo. | |
| **5. Autoría y reproducibilidad:** Identifique qué archivos del paquete, commits, filas de tablas, bolsas o partes del video son su trabajo directo. | |

**Estado de comprobación individual Estudiante 1:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐
**Firma del Estudiante 1:** ________________________________________

---

### Estudiante 2: __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas (pose de observación, verificación del `tag_mesa`, percepción RGB-D, nodo de misión, escena de MoveIt, pinza, pruebas o registros). | |
| **2. El árbol de transformaciones:** Recorra la cadena de `base_link` a `car1_delivery_tray_frame` e indique qué tramo sale del URDF, cuál de la visión y cuál del driver. | |
| **3. La meta de entrega:** Explique cómo se calcula la pose de pre-entrega a partir de la TF del carrito y por qué no tiene coordenadas escritas en el código. | |
| **4. Seguridad:** Elija una falla de la Tabla 6 que usted provocó y explique en qué estado abortó la misión y qué la detuvo. | |
| **5. Autoría y reproducibilidad:** Identifique qué archivos del paquete, commits, filas de tablas, bolsas o partes del video son su trabajo directo. | |

**Estado de comprobación individual Estudiante 2:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐
**Firma del Estudiante 2:** ________________________________________

---

## 9. Selección del Nivel Alcanzado por Criterio (Rúbricas Docentes)

Marque con una **X** una sola casilla por criterio y registre el valor entero (0 a 500) dentro de la banda correspondiente.

### C1. Percepción RGB-D, corrección de perspectiva con tf2 y árbol de transformaciones — Peso 25% — Student Outcome SO1

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, analiza cómo cada tramo del árbol (URDF, visión RGB-D, driver) y la calidad de la profundidad se trasladan al error de colocación, y lo sustenta con las Tablas 2, 3, 3b y 5. |
| ☐ | **N4 — 400–474** | Verifica el `tag_mesa` con la cámara (error menor que 2 cm o URDF corregido con medición), obtiene la pose 3D de los dos tags con RGB-D y la del carrito en el marco de la mesa con tf2, la compara con la cinta y con `tag_height_m` (Tabla 3b) con criterios de calidad de la profundidad, y filtra la pose de la bandeja en `base_link` con la mediana de 20 lecturas. |
| ☐ | **N3 — 300–399** | La pose 3D de los tags se obtiene con RGB-D, la del carrito en el marco de la mesa con `lookup_transform`, y la pose de la bandeja en `base_link` con tf2. |
| ☐ | **N2 — 150–299** | La pose del carrito se obtiene sin profundidad o sin tf2 (por ejemplo, con una escala fija en píxeles), la entrega usa coordenadas escritas a mano, o no se verificó el `tag_mesa`. |
| ☐ | **N1 — 0–149** | No hay localización del carrito por visión o no hay evidencia verificable. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C2. Pick and place con MoveIt 2 y la pinza — Peso 25% — Student Outcome SO2

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, extiende la misión con criterio propio (por ejemplo, metas resueltas con tf2 dentro del ejecutor o detección de movimiento del carrito) y lo documenta en el `README.md`. |
| ☐ | **N4 — 400–474** | Tres ensayos exitosos con el carrito o la hoja en posiciones distintas, la caja adjunta y separada en la escena, el carrito como obstáculo en el modo carrito y la detección de agarre fallido por la posición de la pinza. |
| ☐ | **N3 — 300–399** | La misión completa se ejecuta en el robot real al menos una vez, con todas las trayectorias a través de `moveit_safe_executor`. |
| ☐ | **N2 — 150–299** | La misión se completa sólo con el driver fake, o en el robot real necesitó intervención manual en algún estado. |
| ☐ | **N1 — 0–149** | No hay misión funcional, o se enviaron trayectorias al controlador sin pasar por el ejecutor. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C3. Validación experimental y diagnóstico — Peso 20% — Student Outcome SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, explica el error de colocación por sus fuentes con datos y resuelve el reto en vivo en menos de 10 minutos con un diagnóstico por capas documentado. |
| ☐ | **N4 — 400–474** | Ejecuta P1 a P6 con sus métricas, entrega las bolsas, informes y YAML de cada ensayo, y resuelve el reto en vivo en menos de 20 minutos. |
| ☐ | **N3 — 300–399** | Ejecuta P1 a P3 con sus métricas y registra las bolsas de los ensayos. |
| ☐ | **N2 — 150–299** | Pruebas incompletas, métricas sin medición física o bolsas que no corresponden a los ensayos reportados. |
| ☐ | **N1 — 0–149** | No hay pruebas registradas. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C4. Seguridad en la operación del robot real — Peso 20% — Student Outcome SO4

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, identifica el estado más peligroso para abortar y propone una mejora concreta de la misión o del protocolo. |
| ☐ | **N4 — 400–474** | Modo seco de la misión completa antes de cada ejecución real con revisión del docente; la misión aborta en un estado seguro en P4, P5 y P6, y ningún límite del ejecutor ni el esfuerzo de la pinza se modificó. |
| ☐ | **N3 — 300–399** | Modo seco de la misión antes de cada ejecución real, turnos y parada de emergencia custodiada. |
| ☐ | **N2 — 150–299** | Ejecuta sin modo seco previo en algún ensayo o sin revisión del docente. |
| ☐ | **N1 — 0–149** | Opera fuera del protocolo, amplía límites del ejecutor o lanza un segundo driver o ejecutor. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C5. Trazabilidad y trabajo en equipo — Peso 10% — Student Outcome SO3 / SO5

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, el `README.md` permite que otro grupo reproduzca la misión sin ayuda y el Anexo A demuestra un dominio técnico individual sobresaliente. |
| ☐ | **N4 — 400–474** | Repositorio con commits de todos los integrantes, paquete con `README.md` (arquitectura, árbol de TF y reproducción), video y bolsas trazables a cada ensayo, y Anexo A con autoría verificable. |
| ☐ | **N3 — 300–399** | Instrumento diligenciado, paquete con `README.md` y Anexo A individual completo. |
| ☐ | **N2 — 150–299** | Faltan el video, las bolsas o el `README.md`, o el Anexo A es genérico. |
| ☐ | **N1 — 0–149** | No entrega el instrumento ni el paquete, o el Anexo A demuestra ausencia de participación. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

## 10. Consolidado Final de Calificación Académica

| Criterio Evaluado | Peso Oficial (%) | Nivel Marcado | Valor Obtenido (0–500) | Aporte Ponderado |
|---|:---:|:---:|:---:|:---:|
| **C1. Percepción RGB-D, corrección de perspectiva con tf2 y árbol de transformaciones** | 25% | | | |
| **C2. Pick and place con MoveIt 2 y la pinza** | 25% | | | |
| **C3. Validación experimental y diagnóstico** | 20% | | | |
| **C4. Seguridad en la operación del robot real** | 20% | | | |
| **C5. Trazabilidad y trabajo en equipo** | 10% | | | |
| **TOTAL CONSOLIDADO** | **100%** | | | **________ / 500** |

```text
Nota Académica sobre 5,0 = Nota Consolidada sobre 500 ÷ 100
Aporte a la Nota del Corte 3: E₃ = Nota Académica × 0,30 (30% de C3)
```

| Resultado Oficial de la Actividad | Registro |
|---|---|
| **Nota del Proyecto Final sobre 500 puntos** | __________ / 500 |
| **Nota Académica Oficial sobre 5,0** | __________ / 5,0 |
| **Número de Criterios en N3 o superior (Umbral Individual)** | _____ / 5 |
| **¿Cumple el Umbral Individual de Logro ABET (todos en N3+)?** | SÍ ☐ &nbsp;&nbsp;&nbsp;&nbsp; NO ☐ |

---

## 11. Consolidación ABET y Cierre de Mejora Continua

| Student Outcome / Indicador Evaluado | N Estudiantes Evaluables | N en Nivel N3 o Superior | Porcentaje de Logro (%) | Meta Institucional | Hallazgo Docente |
|---|:---:|:---:|:---:|:---:|---|
| **SO1 / Indicador 1.1** (Percepción RGB-D, Corrección de Perspectiva con tf2 y Árbol de Transformaciones) | | | | 70% | |
| **SO2 / Indicador 2.3** (Pick and Place con MoveIt 2) | | | | 70% | |
| **SO6 / Indicadores 6.1 y 6.4** (Validación y Diagnóstico) | | | | 70% | |
| **SO4 / Indicador 4.1** (Seguridad en el Robot Real) | | | | 70% | |
| **SO3 / Indicador 3.1 - 3.3** (Documentación y Trazabilidad) | | | | 70% | |
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
