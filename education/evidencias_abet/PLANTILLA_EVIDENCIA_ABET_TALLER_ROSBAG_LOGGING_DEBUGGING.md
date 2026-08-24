# Formato de evidencias y calificación — Taller: rosbag2, Logging y Depuración Avanzada

> Instrumento calificable asociado a `education/talleres/TALLER_ROSBAG_LOGGING_DEBUGGING.md`. El taller conserva las instrucciones de aprendizaje; este formato permite registrar las evidencias E1–E10, marcar el nivel alcanzado en cada criterio RAE/SO y obtener la nota académica consolidada de forma automática.

## 1. Identificación y control

| Campo | Registro |
|---|---|
| **Programa** | Ingeniería Mecatrónica |
| **Asignatura** | ROBOT OPERATING SYSTEM - ROS |
| **Periodo** | 2026-2 |
| **Corte / instrumento** | Primer corte / Talleres y tareas |
| **Actividad** | Taller — rosbag2, Logging y Depuración Avanzada (MCAP, Flight Recorder, rosbag2_py) |
| **Nombre completo del estudiante** | |
| **Código institucional / STUDENT_ID** | |
| **Grupo / Subgrupo** | |
| **Fecha de entrega** | |
| **Evaluador** | Ing. Henry Roncancio |
| **Versión del instrumento** | 1.0 |
| **Enlace o ubicación de la entrega** | |
| **Commit evaluado, si aplica** | |
| **Unidad de análisis / Unidad de captura** | Individual |

## 2. Uso del formato y reglas de calificación

1. La persona participante registra las evidencias obligatorias E1–E10 en la Sección 3 y responde las preguntas de la Sección 4.
2. Quien evalúa marca **una sola banda de desempeño por criterio** (N1 a N5) y registra un valor exacto de 0 a 500 dentro del intervalo correspondiente.
3. Los niveles N3, N4 y N5 representan una progresión de dominio; N1 y N2 son bandas para evidencia insuficiente o cumplimiento parcial.
4. Si falta una evidencia obligatoria en una entrega exigible, se marca N1 y se registra 0 para la calificación académica. Un retiro oficial o exclusión autorizada se registra como `NA / no evaluado`, se excluye del denominador ABET y no se convierte en cero.
5. Zubatronic/SGDE calcula el aporte ponderado de cada criterio. En cálculo manual o respaldo: `Aporte = Valor × Peso / 100`.
6. La nota académica sobre 5,0 se obtiene dividiendo la nota sobre 500 entre 100 (`Nota 5,0 = Nota sobre 500 ÷ 100`).
7. La entrega es individual y debe nombrarse `C1_T_ROSBAG_<codigo>_<apellido>_v1.docx`. El código del archivo, el código en el documento y la entrega deben coincidir.

| Nivel | Intervalo Zubatronic / Valor Guía | Interpretación y Desempeño |
|---|---:|---|
| **N5** | 475–500 / guía 500 | Excelente: evidencia completa, precisa, reproducible y explicada con profundidad técnica y rigor analítico. |
| **N4** | 400–474 / guía 450 | Bueno: desempeño correcto con omisiones menores que no impiden verificar la operación, reproducibilidad ni diagnóstico. |
| **N3** | 300–399 / guía 350 | Aceptable: demuestra el desempeño esencial con evidencia verificable. Es el umbral individual de logro. |
| **N2** | 150–299 / guía 250 | Cumplimiento parcial: evidencia incompleta, métricas faltantes o errores conceptuales en el diagnóstico. |
| **N1** | 0–149 / guía 100 | No cumple: evidencia mínima, fragmentaria o no funcional. Sin evidencia obligatoria se registra 0. |

### 2.1. Parámetros de assessment

| Parámetro | Regla adoptada |
|---|---|
| **Población o cohorte** | Censo de estudiantes matriculados que deben presentar el taller en 2026-2. |
| **Momento de medición** | Primer corte, tras la práctica de logging estructurado, rosbag2 y depuración. |
| **Evaluador** | Docente responsable de la asignatura ROBOT OPERATING SYSTEM - ROS. |
| **Umbral individual** | Nivel **N3 o superior** (umbral individual de logro mínimo 300/500) en cada indicador. |
| **Meta de cohorte** | Al menos el **70% de los estudiantes evaluables** alcanza N3 o superior en cada indicador. |
| **Regla de muestreo** | No se usa muestra: se evalúa el censo completo de entregas exigibles. |
| **Evidencia faltante** | Entrega exigible sin evidencia obligatoria: N1 con valor 0. Retiro oficial o exclusión autorizada: `NA / no evaluado`. |
| **Unidad de captura** | Individual (un registro y una puntuación por estudiante). |

### 2.2. Alineación de Criterios, RAE y Student Outcomes para Zubatronic/SGDE

Cada criterio evalúa un único Student Outcome (SO) principal e indicador de desempeño literal del programa. La nota académica y el logro ABET se interpretan por separado. No se calcula un nivel ABET global agregado.

| Criterio | Peso | SO principal | Indicador de desempeño literal del programa | Evidencia directa obligatoria |
|---|---:|:---:|---|---|
| **C1. Control dinámico de logging, verbosidad y trazas en caliente** | 20% | **SO1** | **1.1. Diagnostica anomalías de sincronización, jittering y fallos lógicos en sistemas robóticos mediante el análisis estructurado de trazas de logging y datos registrados.** | E1, E2 y respuestas de análisis de severidad de logs |
| **C2. Grabación quirúrgica, almacenamiento MCAP y compresión Zstd** | 25% | **SO6** | **6.1. Diseña esquemas de grabación selectiva (filtros, compresión y QoS) y ejecuta reproducción determinista para validar hipótesis experimentales sin degradar el rendimiento del robot.** | E3, E4 (dataset MCAP, inspección con ros2 bag info y compresión) |
| **C3. Reproducción determinista, control interactivo y reloj de simulación** | 20% | **SO6** | **6.1. Diseña esquemas de grabación selectiva (filtros, compresión y QoS) y ejecuta reproducción determinista para validar hipótesis experimentales sin degradar el rendimiento del robot.** | E5, E6 (reproducción con rate, controles interactivos, sim_time y remapping) |
| **C4. Patrón Flight Recorder (Caja Negra) y análisis post-mortem de fallas** | 20% | **SO1** | **1.1. Diagnostica anomalías de sincronización, jittering y fallos lógicos en sistemas robóticos mediante el análisis estructurado de trazas de logging y datos registrados.** | E7, E8 (inyección de anomalía de jitter, monitoreo en rqt_console y volcado del ring buffer) |
| **C5. Extracción programática y analítica de datos con rosbag2_py** | 15% | **SO7** | **7.1. Integra herramientas modernas de la industria (estándar de almacenamiento MCAP, visualizadores Foxglove/PlotJuggler y API programática rosbag2_py) para acelerar el ciclo de desarrollo robótico.** | E9, E10 (script Python con rosbag2_py, deserialización CDR, métricas de jitter y respuestas conceptuales) |

## 3. Registro mínimo de evidencias obligatorias

Las capturas deben ser legibles y mostrar el comando ejecutado junto con el resultado obtenido.

| Código | Evidencia Requerida | Archivo, enlace, página o ubicación verificable |
|---|---|---|
| **E1** | Captura de terminal con el nodo de telemetría corriendo en INFO y cambio dinámico a DEBUG con ros2 param set, mostrando flujo cinemático en vivo. | |
| **E2** | Captura del formateo avanzado de consola mediante RCUTILS_CONSOLE_OUTPUT_FORMAT y RCUTILS_COLORIZED_OUTPUT con severidad, tiempo, función y línea. | |
| **E3** | Captura de ejecución de la grabación quirúrgica con ros2 bag record -s mcap --compression-format zstd de los 4 tópicos clave de telemetría. | |
| **E4** | Salida completa de ros2 bag info dataset_telemetria_kinova, evidenciando plugin mcap, conteo de mensajes, compresión zstd y tipos de mensaje. | |
| **E5** | Captura de reproducción determinista a velocidad reducida (--rate 0.5) y evidencia de uso de controles interactivos (pausa con Espacio, single-step con 's'). | |
| **E6** | Captura del Mini-Reto 1: reproducción con remapping de tópicos (--remap /burger/kinova/joint_states:=/burger/kinova/joint_states_replay) verificado con ros2 topic list/echo. | |
| **E7** | Captura de rqt_console con filtro de severidad en ERROR mostrando la alerta de falla tras invocar /burger/kinova/trigger_anomaly. | |
| **E8** | Captura del volcado del Flight Recorder tras invocar /burger/kinova/dump_flight_recorder en nivel DEBUG, mostrando muestras del ring buffer previas a la falla. | |
| **E9** | Código y ejecución del script scripts/read_mcap_telemetry.py sobre el dataset MCAP, mostrando catálogo de tópicos, total de mensajes y jitter promedio/máximo. | |
| **E10** | Respuestas técnicas a las preguntas conceptuales (comparativa MCAP vs SQLite3, throttling de logs y caso de uso de Flight Recorder en robótica). | |

Ubicación del repositorio o registro reproducible de comandos: ________________________________________________

## 4. Respuestas técnicas y análisis conceptual de la persona participante

### E10.1. Comparativa Técnica: SQLite3 (`.db3`) frente a MCAP (`.mcap`)

| Característica Técnica | Plugin SQLite3 (`.db3`) | Plugin MCAP (`.mcap`) — Estado del Arte |
|---|---|---|
| **Definiciones de esquemas de mensajes** | | |
| **Resistencia a cierres abruptos (corte de energía)** | | |
| **Rendimiento de I/O y tasa de datos (cámaras/LiDAR)** | | |
| **Compatibilidad con visores modernos (Foxglove/PlotJuggler)** | | |

### E10.2. Diagnóstico de Logging y Throttling en Tiempo Real

Explique por qué emitir `get_logger().info()` en un bucle de control a alta frecuencia (ej. 1000 Hz) degrada el rendimiento del robot y cómo la macro de throttling (`throttle_duration_sec`) soluciona este problema:

________________________________________________________________________________

________________________________________________________________________________

### E10.3. Arquitectura Flight Recorder (Caja Negra)

Describa el principio de funcionamiento de un Ring Buffer en memoria RAM para diagnóstico post-mortem ante fallas intermitentes o críticas:

________________________________________________________________________________

________________________________________________________________________________

## 5. Selección del nivel alcanzado por criterio

Marque con una **X** una sola casilla por criterio y registre el valor exacto (escala 0 a 500) que se capturará en Zubatronic. Si no existe evidencia obligatoria verificable, marque N1 y registre 0. `NA` no es una banda de desempeño ni produce nota.

### C1. Control dinámico de logging, verbosidad y trazas en caliente — peso 20% — SO1

**SO / Indicador de desempeño:** 1.1. Diagnostica anomalías de sincronización, jittering y fallos lógicos en sistemas robóticos mediante el análisis estructurado de trazas de logging y datos registrados.

**Evidencia directa:** E1, E2 y respuestas de análisis de severidad de logs

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, personaliza variables de entorno (RCUTILS_CONSOLE_OUTPUT_FORMAT, colorización), aísla logs por función y línea, y explica el impacto del throttling de logs en la estabilidad en tiempo real. |
| ☐ | **N4 — 400–474** | Inspecciona /rosout, modifica dinámicamente el nivel de registro a DEBUG/INFO en tiempo de ejecución con ros2 param set y explica la jerarquía de severidad (DEBUG a FATAL). |
| ☐ | **N3 — 300–399** | Lanza el nodo con diferentes niveles de log (--log-level), observa mensajes en consola y cambia la verbosidad mediante la CLI. |
| ☐ | **N2 — 150–299** | Modifica parámetros de log pero requiere reiniciar el nodo o confunde la jerarquía de severidad en /rosout. |
| ☐ | **N1 — 0–149** | No demuestra control dinámico de logs o no presenta la evidencia obligatoria verificable. |
| ☐ | **NA — No evaluado** | Aplicar únicamente conforme a la regla documentada de población o exclusión autorizada. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

**Localizador y observación de la evidencia:**

________________________________________________________________________________

### C2. Grabación quirúrgica, almacenamiento MCAP y compresión Zstd — peso 25% — SO6

**SO / Indicador de desempeño:** 6.1. Diseña esquemas de grabación selectiva (filtros, compresión y QoS) y ejecuta reproducción determinista para validar hipótesis experimentales sin degradar el rendimiento del robot.

**Evidencia directa:** E3, E4 (dataset MCAP, inspección con ros2 bag info y compresión)

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, justifica las ventajas arquitectónicas del formato MCAP (cero corrupción, esquemas embebidos, streaming zero-copy) frente a SQLite3 y diseña esquemas de compresión óptimos para datos heterogéneos. |
| ☐ | **N4 — 400–474** | Graba quirúrgicamente utilizando el plugin mcap, compresión zstd, división por tiempo/tamaño y filtros por expresiones regulares (-e), verificando metadatos con ros2 bag info. |
| ☐ | **N3 — 300–399** | Graba datasets en formato MCAP seleccionando tópicos específicos (/joint_states, /diagnostics, /joint_jitter, /system_health) y genera metadatos válidos. |
| ☐ | **N2 — 150–299** | Graba tópicos pero utiliza configuraciones por defecto (sqlite3), sin compresión o saturando el almacenamiento con grabaciones indiscriminadas (-a). |
| ☐ | **N1 — 0–149** | No genera archivos de grabación válidos o carece de evidencia verificable obligatoria. |
| ☐ | **NA — No evaluado** | Aplicar únicamente conforme a la regla documentada de población o exclusión autorizada. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

**Localizador y observación de la evidencia:**

________________________________________________________________________________

### C3. Reproducción determinista, control interactivo y reloj de simulación — peso 20% — SO6

**SO / Indicador de desempeño:** 6.1. Diseña esquemas de grabación selectiva (filtros, compresión y QoS) y ejecuta reproducción determinista para validar hipótesis experimentales sin degradar el rendimiento del robot.

**Evidencia directa:** E5, E6 (reproducción con rate, controles interactivos, sim_time y remapping)

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, analiza la sincronización temporal con reloj simulado (/clock y use_sim_time), ejecuta remapeo de tópicos en caliente (--remap) y demuestra avance paso a paso determinista. |
| ☐ | **N4 — 400–474** | Utiliza controles interactivos en consola (Espacio para pausa, 's' para single step, '+' / '-' para velocidad) y remapea tópicos para pruebas sin colisión en la red. |
| ☐ | **N3 — 300–399** | Reproduce datasets a velocidad modificada (--rate) y verifica la publicación de mensajes en tópicos esperados. |
| ☐ | **N2 — 150–299** | Reproduce bolsas únicamente a velocidad estándar sin controles interactivos o confunde la sincronización de tiempo simulado. |
| ☐ | **N1 — 0–149** | No logra reproducir el dataset grabado o no presenta evidencia verificable. |
| ☐ | **NA — No evaluado** | Aplicar únicamente conforme a la regla documentada de población o exclusión autorizada. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

**Localizador y observación de la evidencia:**

________________________________________________________________________________

### C4. Patrón Flight Recorder (Caja Negra) y análisis post-mortem de fallas — peso 20% — SO1

**SO / Indicador de desempeño:** 1.1. Diagnostica anomalías de sincronización, jittering y fallos lógicos en sistemas robóticos mediante el análisis estructurado de trazas de logging y datos registrados.

**Evidencia directa:** E7, E8 (inyección de anomalía de jitter, monitoreo en rqt_console y volcado del ring buffer)

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, explica la arquitectura de buffers circulares (Ring Buffers en RAM), diagnostica la causa raíz del jitter/anomalía inyectada e interpreta los registros post-mortem para mitigación en producción. |
| ☐ | **N4 — 400–474** | Inyecta la falla con el servicio /trigger_anomaly, monitorea alertas de ERROR en rqt_console y ejecuta el vaciado del Flight Recorder (/dump_flight_recorder) extrayendo el historial de telemetría. |
| ☐ | **N3 — 300–399** | Identifica la alerta de falla en consola, invoca el servicio de vaciado y visualiza las muestras extraídas. |
| ☐ | **N2 — 150–299** | Observa la falla pero no logra invocar los servicios de control o el volcado de memoria está incompleto. |
| ☐ | **N1 — 0–149** | No ejecuta el procedimiento de inyección y diagnóstico o no presenta evidencia de la caja negra. |
| ☐ | **NA — No evaluado** | Aplicar únicamente conforme a la regla documentada de población o exclusión autorizada. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

**Localizador y observación de la evidencia:**

________________________________________________________________________________

### C5. Extracción programática y analítica de datos con rosbag2_py — peso 15% — SO7

**SO / Indicador de desempeño:** 7.1. Integra herramientas modernas de la industria (estándar de almacenamiento MCAP, visualizadores Foxglove/PlotJuggler y API programática rosbag2_py) para acelerar el ciclo de desarrollo robótico.

**Evidencia directa:** E9, E10 (script Python con rosbag2_py, deserialización CDR, métricas de jitter y respuestas conceptuales)

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, extiende el script para generar análisis estadístico completo (promedios, máximos, detección automática de anomalías) y visualización o exportación sin depender de reproducción en tiempo real. |
| ☐ | **N4 — 400–474** | Implementa un script en Python utilizando rosbag2_py (SequentialReader, StorageOptions, ConverterOptions) para abrir el MCAP, deserializar mensajes CDR y extraer datos cuantitativos. |
| ☐ | **N3 — 300–399** | Ejecuta el script read_mcap_telemetry.py sobre el dataset y reporta el conteo de mensajes y métricas de jitter. |
| ☐ | **N2 — 150–299** | El script presenta errores de deserialización de tipos de mensaje o lee parcialmente los datos del bag. |
| ☐ | **N1 — 0–149** | No implementa la extracción con la API de Python o carece de evidencia verificable obligatoria. |
| ☐ | **NA — No evaluado** | Aplicar únicamente conforme a la regla documentada de población o exclusión autorizada. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

**Localizador y observación de la evidencia:**

________________________________________________________________________________

## 6. Captura y cálculo automático de la calificación

Capture en Zubatronic el valor exacto de cada criterio. La plataforma o la planilla calcula automáticamente:

```text
Aporte del criterio = Valor exacto (0–500) × Peso (%) / 100
Nota Taller sobre 500 = Sumatoria de los cinco aportes ponderados
Nota Académica sobre 5,0 = Nota sobre 500 ÷ 100
```

Los valores guía 500, 450, 350, 250 y 100 agilizan una valoración discreta, pero puede usarse cualquier entero dentro del intervalo demostrado. La ausencia de evidencia obligatoria se registra con 0 dentro de N1.

## 7. Consolidado final de calificación académica

Esta tabla consolida el cálculo automático de la nota académica del taller a partir de los niveles y valores registrados:

| Criterio | Peso | Nivel marcado | Valor exacto (0–500) | Aporte ponderado |
|---|---:|:---:|---:|---:|
| C1. Control dinámico de logging, verbosidad y trazas en caliente | 20% | | | |
| C2. Grabación quirúrgica, almacenamiento MCAP y compresión Zstd | 25% | | | |
| C3. Reproducción determinista, control interactivo y reloj de simulación | 20% | | | |
| C4. Patrón Flight Recorder (Caja Negra) y análisis post-mortem de fallas | 20% | | | |
| C5. Extracción programática y analítica de datos con rosbag2_py | 15% | | | |
| **TOTAL PONDERADO** | **100%** | | | **________ / 500** |

| Resultado final de la actividad | Registro oficial |
|---|---|
| **Nota Taller rosbag2 sobre 500 puntos** | __________ |
| **Nota Académica sobre 5,0 (total ÷ 100)** | __________ |
| **Criterios en Nivel N3 o superior (Umbral de Logro)** | __________ |
| **Criterios por debajo de N3** | __________ |
| **Observaciones del evaluador** | |

## 8. Uso posterior en la nota del primer corte

| Variable de evaluación | Registro y ponderación oficial |
|---|---:|
| Código de actividad / componente | `T_ROSBAG` |
| Peso dentro del acumulado de talleres `T₁` | 100% para el primer corte 2026-2 |
| Aporte del taller a `T₁` | Nota Taller rosbag2 × peso ÷ 100 |
| Fórmula de calificación aprobada del Corte 1 | `Nota Corte 1 = 0,28 T₁ + 0,42 L₁ + 0,30 E₁` |

```text
T₁ = Nota Taller rosbag2
Nota Corte 1 = 0,28 T₁ + 0,42 L₁ + 0,30 E₁
```

Este documento entrega la **Nota del Taller** y los resultados desagregados por criterio. El consolidado docente integra todas las fuentes del corte y prepara el registro oficial en Univex. Zubatronic genera por separado los reportes de logro RAE/SO.

## 9. Consolidación ABET y plan de mejora continua

La calificación académica anterior no se interpreta como un nivel ABET global agregado. Para cada indicador de desempeño se consolidan por separado los estudiantes evaluables de la cohorte.

| Indicador de desempeño / SO | N evaluable | N en N3 o superior | % de logro | Meta de cohorte | Hallazgo |
|---|---:|---:|---:|---:|---|
| **1.1 / SO1** — Resolución de problemas y diagnóstico | | | | 70% | |
| **6.1 / SO6** — Grabación y experimentación determinista | | | | 70% | |
| **7.1 / SO7** — Integración MCAP y rosbag2_py | | | | 70% | |

| Campo de cierre de ciclo de mejora continua | Registro de gestión |
|---|---|
| **Decisión derivada del hallazgo** | |
| **Acción de mejora continua** | |
| **Responsable** | Ing. Henry Roncancio |
| **Fecha prevista de seguimiento** | |
| **Evidencia de seguimiento** | |
| **Resultado observado en el segundo ciclo** | |

## 10. Cierre y control

Nombre de quien evalúa: ____________________________________ &nbsp;&nbsp; Fecha: __________________

Firma o visto bueno del evaluador: ______________________________________________________

Observaciones generales: ________________________________________________________________
