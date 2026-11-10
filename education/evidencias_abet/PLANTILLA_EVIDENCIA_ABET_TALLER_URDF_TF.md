# Formato de entrega, evidencias y calificación — Taller URDF, robot_description y TF2

> Documento `PLANTILLA_EVIDENCIA_ABET_TALLER_URDF_TF`, versión 1.1. Registra evidencia, puntuaciones por criterio y nota de la actividad. No consolida ni interpreta datos de cohorte.

## 1. Identificación

| Campo | Registro |
|---|---|
| Programa | Ingeniería Mecatrónica |
| Asignatura | ELECTIVA ELECTRONICA ( ROBOT OPERATING SYSTEM |
| Periodo | 2026-2 |
| Corte / momento | Tercer corte (ubicación administrativa); evidencia curricular de RAE 2 y del RAE 4 transversal |
| Actividad | Taller individual — URDF, robot_description y TF2 en Burger Delivery |
| Evaluador | Docente responsable de ROBOT OPERATING SYSTEM - ROS |
| Versión del instrumento | 1.1 |
| Nombre completo del estudiante |  |
| Código institucional / STUDENT_ID |  |
| Fecha y hora de entrega |  |
| Fecha y franja de sustentación oral |  |
| Enlace o ubicación de la entrega |  |
| Repositorio y commit evaluado |  |
| ROS_DOMAIN_ID utilizado |  |
| Evaluador |  |

## 2. Uso del formato

1. La persona participante registra los localizadores de las evidencias requeridas.
2. Quien evalúa marca una sola banda por criterio y registra un valor exacto entre 0 y 500 dentro del intervalo demostrado.
3. La entrega exigible sin evidencia se procesa conforme a la regla académica declarada; `NA / no evaluado` se reserva para la población excluida por una regla autorizada.
4. El archivo se nombra `C3_T_URDF_TF_<codigo>_<apellido>_v1.docx`.
5. El código institucional del documento debe coincidir con la identidad de la entrega y con la exportación de notas.

| Nivel | Intervalo / valor guía | Interpretación |
|---|---:|---|
| N5 | 475–500 / guía 500 | Excelente |
| N4 | 400–474 / guía 450 | Bueno |
| N3 | 300–399 / guía 350 | Aceptable / umbral individual de logro |
| N2 | 150–299 / guía 250 | Parcial |
| N1 | 0–149 / guía 100 | No cumple |

### 2.1. Parámetros de captura

| Parámetro | Regla adoptada |
|---|---|
| Unidad de captura | Individual |
| Momento de medición | Tercer corte (ubicación administrativa); evidencia curricular de RAE 2 y del RAE 4 transversal |
| Evaluador | Docente responsable de ROBOT OPERATING SYSTEM - ROS |
| Evidencia faltante | Entrega exigible sin documento = N1 con valor académico 0. Si existe documento pero no se presenta la sustentación oral, C4 y C5 se registran en 0 y no se infiere logro en esos criterios. Retiro oficial o exclusión autorizada = NA / no evaluado para ABET. |

### 2.2. Alineación de criterios

Cada criterio produce una medida distinguible para un único indicador principal.

| Criterio | Peso | Student Outcome (SO) principal | Indicador de desempeño literal | Evidencia directa | Unidad |
|---|---:|---|---|---|---|
| C1. Modelado URDF y parámetros cinemáticos | 25% | SO1 | Indicador 1.2 (RAE 2): Identifica variables, parámetros cinemáticos y restricciones de articulación para formular y simular modelos de robots y entornos industriales en URDF/XACRO. | E1, E2 y E3 | Individual |
| C2. Coherencia espacial y validación del árbol TF2 | 20% | SO1 | Indicador 1.1 (RAE 1): Analiza las relaciones entre fenómenos físicos y modelos matemáticos de orientación y transformaciones, validando el árbol de transformaciones (TF2) en sistemas robóticos distribuidos. | E4, E5, E6 y E7 | Individual |
| C3. Evidencia técnica reproducible y trazabilidad | 20% | SO3 | Indicador 3.2 (RAE 2): Sustenta decisiones de arquitectura, integración de hardware y modelado URDF/XACRO usando evidencias verificables de simulación, laboratorio y repositorio. | E1 a E8 | Individual |
| C4. Sustentación oral individual de decisiones URDF/TF2 | 20% | SO3 | Indicador 3.2 (RAE 2): Sustenta decisiones de arquitectura, integración de hardware y modelado URDF/XACRO usando evidencias verificables de simulación, laboratorio y repositorio. | E9, E10 y E11 | Individual |
| C5. Diagnóstico técnico en vivo durante la sustentación | 15% | SO6 | Indicador 6.4 (RAE 4 transversal): Interpreta fallas y diagnósticos experimentales aplicando protocolos de diagnóstico por capas (Sintaxis -> TF -> Red -> Lógica) para aislar errores en hardware y software. | E12 | Individual |

## 3. Registro mínimo de evidencias

| Código | Evidencia requerida | Archivo, enlace, página o ubicación |
|---|---|---|
| E1 | Registro de preparación, compilación con colcon y verificación de los recursos instalados del paquete burger_description. |  |
| E2 | Evidencia de robot_description, robot_state_publisher, joint_states, tf y tf_static con comandos y salidas legibles. |  |
| E3 | Análisis individual de links y joints: visual, collision, inertial, origin, axis, límites y mimic de la cadena de 6 GDL del Gen3 y de la pinza Robotiq 2F-85, y su correspondencia con el modelo que publica el driver. |  |
| E4 | Resultados de xmllint/check_urdf y grafo o reporte generado por view_frames, con interpretación de la raíz y jerarquía. |  |
| E5 | Capturas de RViz y consultas tf2_echo que relacionen valores observados con origin xyz/rpy del modelo. |  |
| E6 | Explicación verificable de carritos como URDF separados, uso de use_static_carts y diferencia frente a la localización dinámica con AprilTags. |  |
| E7 | Cadena TF de la cámara de muñeca (taller §11): tf2_echo gen3_base_link gen3_camera_color_frame en la pose cero, transformación de un punto del marco óptico a gen3_base_link a mano y con TF2, y explicación del eslabón end_effector_link -> camera_link que necesita el robot real para el Laboratorio IA 01. |  |
| E8 | Enlace al repositorio o ubicación reproducible, commit evaluado, versiones y conclusión técnica individual. |  |
| E9 | Registro del evaluador de la sustentación oral individual: asistencia, verificación de identidad y autoría, fecha, duración y observaciones. |  |
| E10 | Respuesta oral individual: explique el flujo URDF/XACRO -> robot_description -> robot_state_publisher -> tf/tf_static y localice la evidencia correspondiente en su entrega. |  |
| E11 | Respuesta oral individual: interprete una relación entre frames, origin xyz/rpy, links/joints y publicación estática o dinámica; justifique una decisión del modelo Burger Delivery. |  |
| E12 | Reto de diagnóstico en vivo: registre el escenario planteado, hipótesis, prueba propuesta, resultado interpretado y corrección siguiendo Sintaxis -> TF -> Red -> Lógica. |  |

## 4. Selección del nivel alcanzado

Marque una sola banda por criterio y registre un valor exacto de 0 a 500. `NA` no es una banda de desempeño ni produce una nota.

### C1. Modelado URDF y parámetros cinemáticos — peso 25%

**SO / indicador principal:** SO1 — Indicador 1.2 (RAE 2): Identifica variables, parámetros cinemáticos y restricciones de articulación para formular y simular modelos de robots y entornos industriales en URDF/XACRO.

**Evidencia:** E1, E2 y E3

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 / guía 500 | Además de N4, compara alternativas de modelado, anticipa consecuencias en simulación y propone una mejora verificable sin confundir el modelo didáctico con el hardware real. |
| ☐ | N4 — 400–474 / guía 450 | Además de N3, justifica parámetros cinemáticos y restricciones mediante referencias concretas al URDF/XACRO y a su efecto observable. |
| ☐ | N3 — 300–399 / guía 350 | Identifica y explica links, joints, origin, axis, límites y mimic; relaciona la cadena de 6 GDL y la pinza Robotiq 2F-85 del modelo con el hardware del laboratorio. |
| ☐ | N2 — 150–299 / guía 250 | Identifica algunos elementos del URDF, pero omite relaciones cinemáticas, límites o restricciones relevantes. |
| ☐ | N1 — 0–149 / guía 100 | No presenta un modelo verificable o confunde links, joints y parámetros esenciales. |
| ☐ | NA — No evaluado | Aplicar sólo conforme a la regla documentada de población o exclusión. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp; **Valor exacto (0–500):** ________

**Localizador y observación de la evidencia:** ___________________________________________

### C2. Coherencia espacial y validación del árbol TF2 — peso 20%

**SO / indicador principal:** SO1 — Indicador 1.1 (RAE 1): Analiza las relaciones entre fenómenos físicos y modelos matemáticos de orientación y transformaciones, validando el árbol de transformaciones (TF2) en sistemas robóticos distribuidos.

**Evidencia:** E4, E5, E6 y E7

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 / guía 500 | Además de N4, demuestra consistencia de extremo a extremo, analiza incertidumbre o limitaciones y propone una validación adicional reproducible. |
| ☐ | N4 — 400–474 / guía 450 | Además de N3, contrasta valores TF con origin xyz/rpy del modelo, transforma un punto del marco óptico de la cámara a gen3_base_link y explica de forma causal las discrepancias encontradas. |
| ☐ | N3 — 300–399 / guía 350 | Valida la jerarquía esperada con check_urdf, view_frames, tf2_echo y RViz, incluida la cadena hasta la cámara de muñeca; distingue transformaciones estáticas y dinámicas. |
| ☐ | N2 — 150–299 / guía 250 | Genera evidencia parcial del árbol, pero deja frames desconectados o interpreta de forma incorrecta origen, orientación o temporalidad. |
| ☐ | N1 — 0–149 / guía 100 | No produce un árbol TF verificable ni interpreta las transformaciones observadas. |
| ☐ | NA — No evaluado | Aplicar sólo conforme a la regla documentada de población o exclusión. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp; **Valor exacto (0–500):** ________

**Localizador y observación de la evidencia:** ___________________________________________

### C3. Evidencia técnica reproducible y trazabilidad — peso 20%

**SO / indicador principal:** SO3 — Indicador 3.2 (RAE 2): Sustenta decisiones de arquitectura, integración de hardware y modelado URDF/XACRO usando evidencias verificables de simulación, laboratorio y repositorio.

**Evidencia:** E1 a E8

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 / guía 500 | Además de N4, deja una reproducción completa, concisa y auditable, con relación explícita entre decisión, evidencia y conclusión. |
| ☐ | N4 — 400–474 / guía 450 | Además de N3, organiza la evidencia por hipótesis y resultado, explica limitaciones y permite repetir casi toda la validación. |
| ☐ | N3 — 300–399 / guía 350 | Registra comandos, resultados legibles, versión del entorno y commit o ubicación reproducible de los artefactos evaluados. |
| ☐ | N2 — 150–299 / guía 250 | Documenta parte del procedimiento, pero faltan comandos, resultados, versiones o trazabilidad suficiente. |
| ☐ | N1 — 0–149 / guía 100 | La evidencia es fragmentaria, ilegible o no permite localizar la versión evaluada. |
| ☐ | NA — No evaluado | Aplicar sólo conforme a la regla documentada de población o exclusión. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp; **Valor exacto (0–500):** ________

**Localizador y observación de la evidencia:** ___________________________________________

### C4. Sustentación oral individual de decisiones URDF/TF2 — peso 20%

**SO / indicador principal:** SO3 — Indicador 3.2 (RAE 2): Sustenta decisiones de arquitectura, integración de hardware y modelado URDF/XACRO usando evidencias verificables de simulación, laboratorio y repositorio.

**Evidencia:** E9, E10 y E11

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 / guía 500 | Además de N4, comunica con precisión y síntesis, compara alternativas y defiende sus conclusiones ante casos nuevos. |
| ☐ | N4 — 400–474 / guía 450 | Además de N3, responde preguntas de seguimiento, reconoce limitaciones y relaciona parámetros con efectos en RViz y TF2. |
| ☐ | N3 — 300–399 / guía 350 | Explica de manera individual el flujo URDF a robot_description y TF2, y sustenta decisiones con evidencia de su entrega. |
| ☐ | N2 — 150–299 / guía 250 | Responde de forma parcial o memorística y depende del documento sin relacionar decisiones con resultados observables. |
| ☐ | N1 — 0–149 / guía 100 | No presenta la sustentación o no puede explicar la autoría, las decisiones ni la evidencia entregada. |
| ☐ | NA — No evaluado | Aplicar sólo conforme a la regla documentada de población o exclusión. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp; **Valor exacto (0–500):** ________

**Localizador y observación de la evidencia:** ___________________________________________

### C5. Diagnóstico técnico en vivo durante la sustentación — peso 15%

**SO / indicador principal:** SO6 — Indicador 6.4 (RAE 4 transversal): Interpreta fallas y diagnósticos experimentales aplicando protocolos de diagnóstico por capas (Sintaxis -> TF -> Red -> Lógica) para aislar errores en hardware y software.

**Evidencia:** E12

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 / guía 500 | Además de N4, diseña una comprobación mínima y concluyente, anticipa efectos colaterales y verifica la corrección de extremo a extremo. |
| ☐ | N4 — 400–474 / guía 450 | Además de N3, descarta causas con evidencia, interpreta el resultado de la prueba y propone una corrección segura. |
| ☐ | N3 — 300–399 / guía 350 | Aplica la secuencia Sintaxis -> TF -> Red -> Lógica, formula una causa probable y propone una prueba pertinente. |
| ☐ | N2 — 150–299 / guía 250 | Propone hipótesis aisladas o pruebas por ensayo y error sin separar las capas del problema. |
| ☐ | N1 — 0–149 / guía 100 | No identifica una ruta de diagnóstico ni propone una prueba verificable. |
| ☐ | NA — No evaluado | Aplicar sólo conforme a la regla documentada de población o exclusión. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp; **Valor exacto (0–500):** ________

**Localizador y observación de la evidencia:** ___________________________________________

## 5. Captura y cálculo

```text
Aporte del criterio = valor exacto × peso / 100
Nota de la actividad en captura = suma de aportes (máximo 500)
Nota académica = total en captura × 5 / 500
```

Los valores guía agilizan la valoración, pero el valor registrado debe pertenecer al intervalo realmente demostrado.

## 6. Consolidado final de calificación

| Criterio | Peso | Nivel marcado | Valor exacto | Aporte ponderado |
|---|---:|:---:|---:|---:|
| C1. Modelado URDF y parámetros cinemáticos | 25% |  |  |  |
| C2. Coherencia espacial y validación del árbol TF2 | 20% |  |  |  |
| C3. Evidencia técnica reproducible y trazabilidad | 20% |  |  |  |
| C4. Sustentación oral individual de decisiones URDF/TF2 | 20% |  |  |  |
| C5. Diagnóstico técnico en vivo durante la sustentación | 15% |  |  |  |
| **TOTAL** | **100%** |  |  | **________ / 500** |

| Resultado final | Registro |
|---|---|
| Nota de la actividad sobre 500 |  |
| Nota académica sobre 5 |  |
| Observaciones |  |

## 7. Uso posterior en la nota del corte

| Variable | Registro |
|---|---|
| Código de la actividad o componente | T_URDF_TF |
| Peso dentro de Componente de talleres y tareas T_C3 del tercer corte | 28% |
| Nota producida por este documento | Nota de T_URDF_TF sobre 5 |

`I = sumatoria(p_i * v_i) / 100; N_T_URDF_TF = I / 100; la actividad alimenta el promedio T_C3 y T_C3 aporta 28% a C3`

Este instrumento entrega la nota de la actividad. El consolidado docente integra después todas las fuentes del corte y prepara el registro oficial en Univex.

## 8. Cierre

| Campo | Registro |
|---|---|
| Nombre de quien evalúa |  |
| Fecha |  |
| Versión aplicada |  |
| Ubicación del archivo maestro |  |
| Revisión o segundo evaluador |  |
| Observaciones |  |
