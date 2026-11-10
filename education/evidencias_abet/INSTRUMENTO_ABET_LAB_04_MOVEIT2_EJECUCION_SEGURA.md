# Instrumento de Evidencia y Calificación — Laboratorio 04: Planificación con MoveIt 2 y Ejecución Supervisada del Kinova Gen3 sobre la Escena de burger_description

> **Documento Único de Entrega y Evaluación ABET.** Asociado a `education/guias_laboratorio/GUIA_LAB_04_MOVEIT2_EJECUCION_SEGURA.docx`.
> Este documento reúne la entrega integral del equipo: registro de evidencias, tablas de datos experimentales diligenciadas en el laboratorio, respuestas al cuestionario de análisis técnico, Anexo A de comprobación individual y las rúbricas analíticas para la calificación docente y la consolidación ABET.

---

## 1. Identificación y Control

| Campo | Registro Oficial |
|---|---|
| **Programa Académico** | Ingeniería Mecatrónica |
| **Asignatura** | ROBOT OPERATING SYSTEM - ROS |
| **Periodo Académico** | 2026-2 |
| **Corte / Instrumento** | Tercer Corte / Laboratorios y evidencias experimentales |
| **Actividad Evaluada** | Laboratorio 04 — Planificación con MoveIt 2 y ejecución supervisada del Kinova Gen3 sobre la escena de `burger_description` |
| **Número de Grupo / Subgrupo** | |
| **Estudiante 1 (Nombre y Código)** | |
| **Estudiante 2 (Nombre y Código)** | |
| **Estudiante 3 (si aplica)** | |
| **Grupo (`eqNN`) y `ROS_DOMAIN_ID` de las Fases 0 a 5** | `eq____` &nbsp;&nbsp;&nbsp;&nbsp; `ROS_DOMAIN_ID` = ______ |
| **Rol del Equipo en la Fase 6** | Grupo en turno ☐ &nbsp;&nbsp;&nbsp;&nbsp; Grupo anfitrión ☐ |
| **Repositorio y Commit Evaluado (SHA)** | |
| **Nombre del Archivo de Entrega** | `C3_L04_G<grupo>_<codigo1>_<codigo2>_v1.docx` |
| **Anexos Obligatorios del Envío** | `seco.jsonl`, `fake.jsonl`, `real.jsonl`, `metas_eqNN.yaml` y la salida de `ros2 bag info lab04_eqNN_real` |
| **Fecha de Realización en Laboratorio** | |
| **Fecha de Entrega del Documento** | |
| **Docente Evaluador** | Ing. Henry Roncancio |
| **Versión del Instrumento** | Versión 1.0 (2026-2) |
| **Unidad de Análisis / Captura** | Equipo colaborativo con comprobación individual (Anexo A) |

---

## 2. Parámetros de Assessment

| Parámetro | Regla Institucional y Metodológica Adoptada |
|---|---|
| **Población o cohorte** | Censo completo de estudiantes matriculados que presentan el Laboratorio 04 en el periodo 2026-2. |
| **Momento de medición** | Sesión experimental presencial (Fases 0 a 5 con driver fake en el dominio del grupo, Fase 6 con el robot real por turnos) y entrega documental única con tablas llenas, registros `*.jsonl` y la bolsa de la Fase 6. |
| **Evaluador** | Docente titular de la asignatura ROBOT OPERATING SYSTEM - ROS. |
| **Umbral individual de logro** | Nivel **N3 o superior** (puntaje mínimo de 300 sobre 500) en cada uno de los indicadores de desempeño evaluados. |
| **Meta de cohorte** | Al menos el **70% de los estudiantes evaluables** debe alcanzar el nivel N3 o superior en cada indicador. |
| **Regla de muestreo** | Censo 100%: no se utiliza muestreo; se evalúa la totalidad de los estudiantes y equipos exigibles. |
| **Evidencia faltante** | Entrega exigible sin evidencia obligatoria verificable: se califica en N1 con puntaje 0. Retiro oficial o causa justificada autorizada: `NA / no evaluado` (se excluye del denominador ABET). |
| **Regla de trabajo colaborativo** | Informe único por equipo complementado obligatoriamente con el Anexo A de comprobación individual. Sin comprobación individual suficiente no se infiere logro individual ABET. |
| **Regla de datos** | Toda cifra de las Tablas 3 a 6 debe poder rastrearse a una línea de `seco.jsonl`, `fake.jsonl` o `real.jsonl`, o a una medición física registrada. Un dato que no pueda rastrearse se considera evidencia no verificable. |

---

## 3. Niveles de Desempeño para Zubatronic/SGDE

| Nivel | Intervalo Zubatronic | Valor Guía | Interpretación y Criterio de Logro |
|---|---:|:---:|---|
| **N5** | 475–500 | 500 | **Excelente:** Evidencia completa y reproducible; explica cada bloqueo con su causa y su regla. |
| **N4** | 400–474 | 450 | **Bueno:** Desempeño correcto con omisiones menores que no comprometen la validez de las mediciones ni la seguridad. |
| **N3** | 300–399 | 350 | **Aceptable:** Completa las fases con evidencia verificable. **Es el umbral individual de logro.** |
| **N2** | 150–299 | 250 | **Cumplimiento parcial:** Evidencia incompleta o bloqueos sin explicar. |
| **N1** | 0–149 | 100 | **No cumple:** Evidencia mínima, o ejecuta en el robot fuera del protocolo. Sin evidencia obligatoria se registra 0. |

---

## 4. Alineación de Criterios, RAE y Student Outcomes

| Criterio | Peso | Student Outcome Principal | Indicador de Desempeño Literal del Programa | Evidencia Directa Obligatoria |
|---|:---:|:---:|---|---|
| **C1. Escena, cinemática y planificación** | 25% | **SO1** | **1.1.** Aplica principios matemáticos (transformaciones homogéneas, cinemática directa e inversa) para resolver problemas espaciales en robótica. | Mesa de `burger_description` cargada en `base_link` y verificada en RViz, cálculo de la cara superior de la mesa en las dos versiones de la escena, rechazo de la escena con z = 0 (código 1) y predicción de la pose de `aproximacion_eqNN` (E2, E4, E8; Tablas 2 y 5). |
| **C2. Integración de MoveIt 2 con el paquete de referencia** | 20% | **SO2** | **2.1.** Diseña soluciones de software integrando contratos ROS 2 (servicios, acciones, QoS) en sistemas robóticos reales. | `move_group` con la ejecución deshabilitada, secuencia por defecto en modo seco y con driver fake a través de `moveit_safe_executor`, y Tabla 3 completa (E3, E5, E6). |
| **C3. Validación experimental y diagnóstico de bloqueos** | 25% | **SO6** | **6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos por capas para aislar errores de planificación, validación y ejecución. | Las ocho pruebas 4.1 a 4.8 con la regla o capa que actuó, el mensaje exacto, el código de salida y su explicación, incluida la prueba 4.8 sin la mesa en la escena (E7; Tabla 4). |
| **C4. Seguridad en la ejecución real** | 20% | **SO4** | **4.1.** Identifica riesgos de seguridad física, paradas de emergencia y celdas robóticas. | Modo seco con el robot real revisado por el docente, ejecución supervisada con autorización verbal, parada de emergencia custodiada, radio libre de 1.2 m y un único ejecutor en el dominio 0; altura de las puntas medida y Tabla 6 completa (E9). |
| **C5. Trazabilidad y trabajo en equipo** | 10% | **SO3 / SO5** | **3.1 - 3.3.** Elabora documentación técnica reproducible y registra evidencias.<br>**5.1.** Define roles técnicos y coordina la ejecución en equipo. | Tabla 1 con commit y resultado de `colcon test`, registros `seco.jsonl`, `fake.jsonl` y `real.jsonl`, bolsa `lab04_eqNN_real` y Anexo A individual (E1, E9). |
| **TOTAL** | **100%** | | | |

---

## 5. Registro de Evidencias de la Práctica (E1–E9)

| Código | Evidencia Requerida | Localizador en el Documento / Repositorio / Archivo |
|---|---|---|
| **E1** | Salida de `colcon test-result --verbose` con `test_trajectory_validation.py`, `test_escena_urdf.py` y `test_metas_moveit.py` aprobadas, y `echo $ROS_DOMAIN_ID` mostrando el dominio del grupo (`10 + NN`). | |
| **E2** | Log de `escena_moveit` con la caja `table_link` en `base_link` y `1 obstáculo(s) cargados`, y captura de RViz (*Scene Objects*) con la mesa bajo el robot. | |
| **E3** | Captura del aviso de `move_group` al pulsar **Execute** en el panel MotionPlanning, que demuestra que MoveIt no puede mover el robot. | |
| **E4** | Salida de `escena_moveit` con la escena modificada a z = 0, que se niega a cargarla y termina con código 1, junto con el cálculo a mano de la altura de la cara superior de la mesa en las dos versiones. | |
| **E5** | Informe del ejecutor en modo seco para la secuencia por defecto (`seco.jsonl`): puntos, duración, recorrido y velocidad pico de cada meta, y la pose objetivo de `sobre_tag_mesa` en `base_link`. | |
| **E6** | Ejecución de la secuencia con el driver fake (`fake.jsonl`): confirmación con `si`, movimiento en RViz y desvío final máximo de cada meta. | |
| **E7** | Mensaje exacto y código de salida (`echo $?`) de cada prueba de bloqueo 4.1 a 4.8, con captura de la prueba 4.8 y de la recarga de la escena al terminarla. | |
| **E8** | Archivo `metas_eqNN.yaml` con `aproximacion_eqNN`, `descenso_eqNN` y la secuencia del grupo, y sus informes en modo seco y con driver fake. | |
| **E9** | Fase 6: informe en modo seco con el robot real revisado por el docente, ejecución supervisada (`real.jsonl`), salida de `ros2 bag info lab04_eqNN_real` y foto de la medición con cinta métrica de la altura de las puntas sobre la mesa. | |

---

## 6. Tablas de Registro Experimental (Diligenciadas por el Equipo)

> **Instrucciones para el equipo:** Diligencie de forma completa y precisa cada una de las siguientes seis tablas con los datos reales obtenidos durante la sesión de laboratorio. Copie los valores de los informes del ejecutor sin redondearlos.

### Tabla 1: Entorno (Fase 0)

| Ítem | Valor |
|---|---|
| **Grupo (`eqNN`) y `ROS_DOMAIN_ID` de las Fases 0 a 5** | |
| **Versión de MoveIt** (`ros2 pkg xml moveit_ros_move_group \| grep version`) | |
| **Resultado de `colcon test` de `burger_kinova_reference`** (aprobadas / fallidas) | |
| **Commit del repositorio del grupo (SHA)** | |

---

### Tabla 2: Escena y Ejecución Deshabilitada (Fase 1)

| Comprobación | Resultado Observado | Explicación |
|---|---|---|
| **Aviso de `move_group` al pulsar Execute** | | |
| **Caja `table_link` en `base_link`** (centro y tamaño) | | |
| **Cara superior de la mesa en `base_link`, escena corregida** (z, calculado) | | |
| **Cara superior de la mesa en `base_link`, escena con z = 0** (z, calculado) | | |
| **Salida y código de `escena_moveit` con la escena z = 0** | | |

---

### Tabla 3: Secuencia por Defecto (Fases 2 y 3, modo seco y driver fake)

| Meta | Tipo | Puntos | Duración (s) | Articulación de Mayor Recorrido (rad) | Velocidad Pico (rad/s) | Desvío Final Fake (rad) |
|---|---|:---:|:---:|:---:|:---:|:---:|
| `home` | articular | | | | | |
| `sobre_tag_mesa` | pose | | | | | |
| `bajar_5cm` | cartesiana | | | | | |
| `subir_5cm` | cartesiana | | | | | |
| `home` | articular | | | | | |

---

### Tabla 4: Bloqueos Provocados (Fase 4)

| N.º | Prueba | Regla o Capa que Actuó | Mensaje Exacto | Código de Salida | ¿Por Qué Ocurre? |
|:---:|---|---|---|:---:|---|
| **4.1** | Velocidad (`max_velocity_rad_s:=0.05`) | | | | |
| **4.2** | Velocidad declarada por MoveIt (`velocity_scaling:=0.6`) | | | | |
| **4.3** | Arranque desde otra pose (`source:=rviz`) | | | | |
| **4.4** | Meta inalcanzable (`delivery_slot_1`) | | | | |
| **4.5** | Meta dentro de la mesa (`en_la_mesa`) | | | | |
| **4.6** | Telemetría perdida | | | | |
| **4.7** | Cancelación del operador (Ctrl+C) | | | | |
| **4.8** | Sin la mesa en la escena | | | | |

---

### Tabla 5: Metas Propias (Fase 5, modo seco y driver fake)

| Meta | Pose Predicha en `base_link` (m) | Pose Impresa por el Ejecutor (m) | Duración (s) | Velocidad Pico (rad/s) | Resultado |
|---|---|---|:---:|:---:|---|
| `aproximacion_eqNN` | | | | | |
| `descenso_eqNN` | — | — | | | |

---

### Tabla 6: Ejecución en el Robot Real (Fase 6)

| Meta | Duración Planificada (s) | Desvío Final (rad) | Error de Seguimiento Máximo del Feedback (rad) | Altura Medida de las Puntas (m) | Resultado |
|---|:---:|:---:|:---:|:---:|---|
| `home` | | | | — | |
| `aproximacion_eqNN` | | | | — | |
| `descenso_eqNN` | | | | | |
| `home` (regreso) | | | | — | |

---

## 7. Cuestionario de Análisis y Discusión Técnica (Respondido por el Equipo)

### Pregunta 1: Capas de Protección
*Con la Tabla 4, clasifique cada prueba según la capa que la detuvo: escena de planificación, planificador (MoveIt), validador de trayectoria, habilitación del operador o vigilancia durante la ejecución. ¿Qué riesgo cubre cada capa que las demás no cubren?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 2: Geometría Compartida
*A partir de la Fase 1 y la prueba 4.8, explique por qué un error de 25 mm en un URDF que «sólo se usaba para visualizar» terminó siendo un problema de seguridad del robot real.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 3: Velocidad entre Puntos y Velocidad Declarada
*Compare las pruebas 4.1 y 4.2. ¿Por qué el ejecutor revisa tanto la velocidad entre puntos como la declarada por MoveIt, si MoveIt ya aplica `velocity_scaling`?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 4: Simulado frente a Real
*Compare el desvío final y la duración de la Tabla 3 (driver fake) con los de la Tabla 6 (robot real). ¿Qué fuentes de diferencia existen y cuáles pertenecen al enlace de red (Laboratorio 03, `EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md`)?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 5: Planificar desde la Pose Medida
*¿Por qué el ejecutor pide cada plan desde la pose **medida** y no deja que MoveIt use la que conoce? Describa un escenario del laboratorio donde ambas difieran.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 6: De `max_joint_delta_rad` a las Reglas sobre la Trayectoria Completa
*`max_joint_delta_rad` (0.35 rad) no se aplica a los planes de MoveIt. ¿Qué regla lo sustituye y por qué la regla anterior no tiene sentido para una trayectoria de varios puntos?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 7: Metas Expresadas en `tag_mesa`
*Una meta de tipo `pose` en `tag_mesa` sigue a la mesa si el URDF cambia. ¿Qué pasaría si el `tag_mesa` físico se mueve 5 cm y el URDF no se actualiza? ¿Qué componente del proyecto podría detectarlo?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 8: Alcance del Modo Seco
*En modo seco la secuencia se planifica desde la pose final del plan anterior. ¿Qué garantiza y qué **no** garantiza un modo seco exitoso sobre la ejecución real?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 9: Validar También la Geometría
*¿Qué haría falta para que `moveit_safe_executor` validara también la geometría (colisiones) del plan, y por qué no basta con confiar en MoveIt?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 10: Conclusiones del Equipo
*Redacte tres conclusiones: (1) sobre la separación entre planificar y ejecutar, sustentada en las Tablas 2 y 4; (2) sobre el papel de `burger_description` como contrato geométrico de la celda; y (3) una conclusión cuantitativa sobre la fidelidad de la ejecución real frente al plan (Tabla 6).*

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
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas en la práctica (ej. escena y cálculo de la mesa, ejecución del modo seco, pruebas de bloqueo, metas propias, turno en el robot real o registros). | |
| **2. Planificar frente a ejecutar:** Explique por qué `move_group` se lanza con `allow_trajectory_execution: false` y qué comprueba `moveit_safe_executor` que MoveIt no comprueba. | |
| **3. Lectura de un informe de validación:** Tome una meta de la Tabla 3 y explique cada dato del informe (puntos, duración, recorrido y velocidad pico) y la regla o el parámetro que lo limita. | |
| **4. Diagnóstico de un bloqueo:** Elija una prueba de la Tabla 4 que usted ejecutó, identifique la capa que la detuvo y explique por qué esa capa, y no otra, actuó. | |
| **5. Autoría y reproducibilidad:** Identifique qué comandos, filas de tablas, metas de `metas_eqNN.yaml`, capturas o registros `*.jsonl` fueron ejecutados y documentados directamente por usted. | |

**Estado de comprobación individual Estudiante 1:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐
**Firma del Estudiante 1:** ________________________________________

---

### Estudiante 2: __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas en la práctica (ej. escena y cálculo de la mesa, ejecución del modo seco, pruebas de bloqueo, metas propias, turno en el robot real o registros). | |
| **2. Planificar frente a ejecutar:** Explique por qué `move_group` se lanza con `allow_trajectory_execution: false` y qué comprueba `moveit_safe_executor` que MoveIt no comprueba. | |
| **3. Lectura de un informe de validación:** Tome una meta de la Tabla 3 y explique cada dato del informe (puntos, duración, recorrido y velocidad pico) y la regla o el parámetro que lo limita. | |
| **4. Diagnóstico de un bloqueo:** Elija una prueba de la Tabla 4 que usted ejecutó, identifique la capa que la detuvo y explique por qué esa capa, y no otra, actuó. | |
| **5. Autoría y reproducibilidad:** Identifique qué comandos, filas de tablas, metas de `metas_eqNN.yaml`, capturas o registros `*.jsonl` fueron ejecutados y documentados directamente por usted. | |

**Estado de comprobación individual Estudiante 2:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐
**Firma del Estudiante 2:** ________________________________________

---

## 9. Selección del Nivel Alcanzado por Criterio (Rúbricas Docentes)

Marque con una **X** una sola casilla por criterio y registre el valor entero (0 a 500) dentro de la banda correspondiente.

### C1. Escena, cinemática y planificación — Peso 25% — Student Outcome SO1

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, deriva con transformaciones homogéneas la pose de `table_link` y de `tag_mesa` en `base_link` a partir de las articulaciones fijas de la escena, explica por qué 25 mm de error bastan para que MoveIt declare en colisión el estado inicial, y su predicción de `aproximacion_eqNN` coincide con la pose impresa por el ejecutor. |
| ☐ | **N4 — 400–474** | Verifica la mesa en el log de `escena_moveit` y en RViz, calcula correctamente la cara superior de la mesa en las dos versiones de la escena, reproduce el rechazo de la escena con z = 0 (código 1) y predice la pose de `aproximacion_eqNN` explicando las diferencias. Tablas 2 y 5 completas. |
| ☐ | **N3 — 300–399** | Carga la escena, comprueba la caja en RViz, completa la Tabla 2 con el cálculo de la cara superior de la mesa y registra en la Tabla 5 la pose predicha y la impresa. |
| ☐ | **N2 — 150–299** | Da por cargada la escena sólo por el código de salida, sin comprobar la caja en RViz; el cálculo de la altura de la mesa es erróneo, o falta la predicción de la Tabla 5. |
| ☐ | **N1 — 0–149** | No carga la escena de `burger_description` o no presenta evidencia verificable de la Fase 1. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C2. Integración de MoveIt 2 con el paquete de referencia — Peso 20% — Student Outcome SO2

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, explica el flujo plan → validación → confirmación → controlador con los nodos, servicios y acciones que intervienen, y justifica con un caso concreto por qué el ejecutor pide cada plan desde la pose medida. |
| ☐ | **N4 — 400–474** | Demuestra que **Execute** en RViz no mueve el robot, ejecuta la secuencia por defecto en modo seco y con driver fake, relaciona `sobre_tag_mesa` con la pose de `tag_mesa` y completa la Tabla 3 con velocidades pico de hasta 0.35 rad/s y los desvíos finales. |
| ☐ | **N3 — 300–399** | Levanta MoveIt en modo sólo planificación con el driver fake, ejecuta la secuencia por defecto en modo seco y con driver fake, y completa la Tabla 3. |
| ☐ | **N2 — 150–299** | Ejecuta sólo parte de la secuencia, omite el modo seco antes de habilitar el movimiento o deja la Tabla 3 incompleta. |
| ☐ | **N1 — 0–149** | No integra MoveIt con `moveit_safe_executor`, o intenta ejecutar los planes directamente desde RViz o `move_group`. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C3. Validación experimental y diagnóstico de bloqueos — Peso 25% — Student Outcome SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, clasifica las ocho pruebas por capa (escena, planificador, validador, habilitación del operador y vigilancia durante la ejecución), argumenta con la prueba 4.8 qué riesgo no cubre el validador y propone una salvaguarda adicional que pueda verificarse. |
| ☐ | **N4 — 400–474** | Ejecuta las ocho pruebas 4.1 a 4.8 reduciendo límites, nunca aumentándolos; registra el mensaje exacto y el código de salida de cada una, explica su causa y vuelve a cargar la escena al terminar la prueba 4.8. Tabla 4 completa. |
| ☐ | **N3 — 300–399** | Ejecuta al menos seis de las ocho pruebas, incluida la 4.8, con mensaje, código de salida y explicación en la Tabla 4. |
| ☐ | **N2 — 150–299** | Ejecuta menos de seis pruebas, parafrasea los mensajes en lugar de copiarlos o sus explicaciones no identifican la regla que actuó. |
| ☐ | **N1 — 0–149** | No provoca los bloqueos, o amplía límites para «hacer pasar» un plan. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C4. Seguridad en la ejecución real — Peso 20% — Student Outcome SO4

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, compara cuantitativamente el desvío final y el error de seguimiento del robot real con los del driver fake (Tablas 3 y 6) y propone una mejora concreta al protocolo de turnos con MoveIt. |
| ☐ | **N4 — 400–474** | Ejecuta en el robot real sólo después del modo seco con el robot en la misma pose y revisado por el docente, con autorización verbal, parada de emergencia custodiada, radio libre de 1.2 m y un único ejecutor en el dominio 0; mide la altura de las puntas y completa la Tabla 6. |
| ☐ | **N3 — 300–399** | Cumple el protocolo de turnos (modo seco revisado, autorización verbal y confirmación con `si`) y registra la Tabla 6. |
| ☐ | **N2 — 150–299** | Ejecuta sin que el docente revise el modo seco, con el robot en una pose distinta a la del modo seco, o deja la Tabla 6 incompleta. |
| ☐ | **N1 — 0–149** | Ejecuta en el robot fuera del protocolo, lanza un segundo ejecutor o un driver fake en el dominio 0, o modifica límites para el robot real. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C5. Trazabilidad y trabajo en equipo — Peso 10% — Student Outcome SO3 / SO5

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, reconstruye el turno de la Fase 6 cruzando `real.jsonl` con la bolsa `lab04_eqNN_real`, y el Anexo A demuestra un dominio técnico individual sobresaliente. |
| ☐ | **N4 — 400–474** | Entrega `seco.jsonl`, `fake.jsonl`, `real.jsonl`, `metas_eqNN.yaml` y la información de la bolsa `lab04_eqNN_real`; la Tabla 1 incluye el commit y el resultado de `colcon test`, y el Anexo A evidencia autoría y comprensión. |
| ☐ | **N3 — 300–399** | Entrega el informe con las Tablas 1 a 6, los registros `*.jsonl` y el Anexo A con respuestas verificables. |
| ☐ | **N2 — 150–299** | Faltan registros `*.jsonl` o la bolsa, la Tabla 1 está incompleta o el Anexo A tiene respuestas genéricas. |
| ☐ | **N1 — 0–149** | No entrega el informe ni los registros, o el Anexo A demuestra ausencia de participación. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

## 10. Consolidado Final de Calificación Académica

| Criterio Evaluado | Peso Oficial (%) | Nivel Marcado | Valor Obtenido (0–500) | Aporte Ponderado |
|---|:---:|:---:|:---:|:---:|
| **C1. Escena, cinemática y planificación** | 25% | | | |
| **C2. Integración de MoveIt 2 con el paquete de referencia** | 20% | | | |
| **C3. Validación experimental y diagnóstico de bloqueos** | 25% | | | |
| **C4. Seguridad en la ejecución real** | 20% | | | |
| **C5. Trazabilidad y trabajo en equipo** | 10% | | | |
| **TOTAL CONSOLIDADO** | **100%** | | | **________ / 500** |

```text
Nota Académica sobre 5,0 = Nota Consolidada sobre 500 ÷ 100
Aporte a Laboratorios del Tercer Corte: L₃ = Nota Académica sobre 5,0
```

| Resultado Oficial de la Actividad | Registro |
|---|---|
| **Nota de Laboratorio 04 sobre 500 puntos** | __________ / 500 |
| **Nota Académica Oficial sobre 5,0** | __________ / 5,0 |
| **Número de Criterios en N3 o superior (Umbral Individual)** | _____ / 5 |
| **¿Cumple el Umbral Individual de Logro ABET (todos en N3+)?** | SÍ ☐ &nbsp;&nbsp;&nbsp;&nbsp; NO ☐ |

---

## 11. Consolidación ABET y Cierre de Mejora Continua

| Student Outcome / Indicador Evaluado | N Estudiantes Evaluables | N en Nivel N3 o Superior | Porcentaje de Logro (%) | Meta Institucional | Hallazgo Docente |
|---|:---:|:---:|:---:|:---:|---|
| **SO1 / Indicador 1.1** (Escena, Cinemática y Planificación) | | | | 70% | |
| **SO2 / Indicador 2.1** (Integración de MoveIt 2 y Contratos ROS 2) | | | | 70% | |
| **SO6 / Indicador 6.4** (Diagnóstico de Bloqueos por Capas) | | | | 70% | |
| **SO4 / Indicador 4.1** (Seguridad en la Ejecución Real) | | | | 70% | |
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
