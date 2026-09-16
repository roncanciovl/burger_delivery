# Instrumento de Evidencia y Calificación — Proyecto Integrador Corte 2: Control de Posición en Lazo Cerrado con ROS 2 Actions (Turtlesim / micro-ROS)

> **Documento Único de Entrega Estudiantil y Evaluación ABET.** Asociado a `education/proyectos_evaluables/PROYECTO_CORTE_2_CONTROL_POSICION_ACCIONES.md`.  
> Este formato integra la entrega formal por equipos: registro de evidencias, tablas de datos experimentales diligenciadas durante las pruebas, cuestionario de análisis técnico con respuestas justificadas, Anexo A de comprobación individual para sustentar el logro ABET de cada estudiante y rúbricas analíticas docentes.

---

## 1. Identificación y Control

| Campo | Registro Oficial |
|---|---|
| **Programa Académico** | Ingeniería Mecatrónica |
| **Asignatura** | ROBOT OPERATING SYSTEM - ROS |
| **Periodo Académico** | 2026-2 |
| **Corte / Instrumento** | Segundo Corte / Entrega final y sustentación técnica (`E_C2` — 30%) |
| **Actividad Evaluada** | Proyecto Integrador Corte 2 — Control de Posición con ROS 2 Actions (Turtlesim / micro-ROS) |
| **Número de Grupo / Subgrupo** | |
| **Estudiante 1 (Nombre y Código)** | |
| **Estudiante 2 (Nombre y Código)** | |
| **Estudiante 3 (si aplica)** | |
| **Plataforma de Validación** | Simulador Turtlesim ☐ &nbsp;&nbsp;&nbsp;&nbsp; micro-ROS ESP32 Físico (Bono) ☐ |
| **Nombre del Archivo de Entrega** | `C2_E02_G<grupo>_<codigo1>_<codigo2>_v1.docx` |
| **Fecha de Realización / Sustentación** | |
| **Fecha de Entrega del Documento** | |
| **Docente Evaluador** | Ing. Henry Roncancio |
| **Versión del Instrumento** | Versión 1.1 — Enfoque Profundo en Acciones (2026-2) |
| **Unidad de Análisis / Captura** | Equipo colaborativo con comprobación individual (Anexo A) |

---

## 2. Parámetros de Assessment

| Parámetro | Regla Institucional y Metodológica Adoptada |
|---|---|
| **Población o cohorte** | Censo completo de estudiantes matriculados que presentan el proyecto integrador del Corte 2 en el periodo 2026-2. |
| **Momento de medición** | Segundo corte (Semana 13), tras la implementación de la interfaz de acción, servidor cinemático y pruebas en lazo cerrado. |
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
| **N5** | 475–500 | 500 | **Excelente:** Dominio exhaustivo de ROS 2 Actions (contratos DDS, ciclo de vida formal, preempción en caliente y cancelación en seco), control cinemático sin singularidades y sustentación sobresaliente. |
| **N4** | 400–474 | 450 | **Bueno:** Desempeño técnico correcto: la acción ejecuta metas, emite feedback a 10 Hz y cancela adecuadamente, con omisiones menores en preempción o en el análisis de jitter. |
| **N3** | 300–399 | 350 | **Aceptable:** Demuestra el desempeño esencial con evidencia verificable: el servidor acepta la meta, calcula control y frena en tolerancia. **Es el umbral individual de logro.** |
| **N2** | 150–299 | 250 | **Cumplimiento parcial:** Evidencia incompleta, acción implementada sin soporte de cancelación, inestabilidad en la trayectoria o sin introspección de tópicos internos. |
| **N1** | 0–149 | 100 | **No cumple:** Implementación no funcional, sin arquitectura de acciones (uso de tópicos/servicios simples) o entrega fragmentaria. Sin evidencia obligatoria se registra 0. |

---

## 4. Alineación de Criterios, RAE y Student Outcomes (Ponderación con Foco en Actions)

| Criterio | Peso | Student Outcome Principal | Indicador de Desempeño Literal del Programa | Evidencia Directa Obligatoria |
|---|:---:|:---:|---|---|
| **C1. Arquitectura y Contrato de Acción ROS 2** | 30% | **SO2** | **2.1 / 2.3.** Diseña contratos de comunicación robustos integrando interfaces customizadas de acciones y arquitectura cliente-servidor. | Interfaz `GoToPose.action` compilada, introspección de los 5 canales DDS (`ros2 action info`, `ros2 topic/service list`), validación de límites en `goal_callback`. |
| **C2. Ciclo de Vida, Máquina de Estados y Preempción** | 25% | **SO6** | **6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos de diagnóstico por capas para aislar errores en hardware y software. | Transiciones formales del `GoalStatus`, emisión de feedback a 10 Hz, soporte de cancelación con parada en seco y política de reemplazo de meta en caliente (*Preemption*). |
| **C3. Modelado Cinemático en Lazo Cerrado** | 20% | **SO1** | **1.1 / 1.2.** Analiza relaciones entre modelos matemáticos y orientación, calculando el error de rumbo y posición sin singularidades angulares. | Implementación de $\rho, \alpha, \theta_e$, función `wrap_to_pi`, saturaciones ($v_{\max}, \omega_{\max}$), atenuación en curvas cerradas y convergencia en dos fases. |
| **C4. Misión Multi-Waypoint y Análisis con Rosbag MCAP** | 15% | **SO6** | **6.1 / 6.2.** Diseña y ejecuta pruebas de lazo cerrado, midiendo tiempos de convergencia y errores en estado estacionario. | Dataset MCAP `dataset_proyecto_corte2_G<equipo>.mcap`, ejecución de misión de patrullaje multi-meta encadenada y gráficas de trayectoria y velocidad. |
| **C5. Trazabilidad Técnica, Trabajo en Equipo y Sim-to-Real** | 10% | **SO3 / SO5** | **3.1 - 3.3.** Documentación técnica reproducible.<br>**5.1.** Define roles técnicos y coordina la ejecución en equipo. | Informe técnico estructurado, repositorio Git con commits colaborativos, Anexo A individual diligenciado y sustentación oral (con reto micro-ROS opcional). |
| **TOTAL** | **100%** | | | |

---

## 5. Registro de Evidencias Obligatorias (E1–E8)

| Código | Evidencia Requerida | Localizador en el Repositorio / Documento |
|---|---|---|
| **E1** | Captura de `ros2 interface show <paquete>/action/GoToPose` mostrando la estructura completa de Goal, Result y Feedback. | |
| **E2** | Salida de terminal de la introspección de los 5 canales DDS: `ros2 topic list | grep go_to_pose` y `ros2 service list | grep go_to_pose`. |
| **E3** | Captura de terminal del servidor mostrando aceptación de metas válidas y rechazo de metas fuera de límites ($x < 0.5$ o $y > 10.5$). | |
| **E4** | Captura de terminal del cliente de acción mostrando recepción periódica de `Feedback` a 10 Hz con distancia restante y error angular. | |
| **E5** | Evidencia de la prueba de **Preempción en caliente**: llegada de nueva meta mientras ejecutaba, aborto ordenado de la anterior y transición fluida. | |
| **E6** | Evidencia de la prueba de **Cancelación voluntaria**: cliente envía `cancel_goal()`, el robot frena en seco y el servidor reporta `STATUS_CANCELED`. | |
| **E7** | Captura de pantalla de `turtlesim` con el rastro de la **Misión Multi-Waypoint (Patrullaje)** completando 4 objetivos secuenciales. | |
| **E8** | Salida de `ros2 bag info dataset_proyecto_corte2_G<equipo>.mcap` y gráficas de trayectoria 2D $(x, y)$, error vs tiempo y velocidades $v, \omega$. | |

---

## 6. Tablas de Registro Experimental

### Tabla 1: Introspección de los Canales DDS de la Acción

| Canal Generado por ROS 2 | Tipo de Primitiva (Topic / Service) | Tipo de Mensaje / Servicio | Función en el Ciclo de Vida |
|---|---|---|---|
| `/go_to_pose/_action/send_goal` | Servicio | `<paquete>/action/GoToPose_SendGoal` | Validación y aceptación/rechazo de la meta |
| `/go_to_pose/_action/cancel_goal` | Servicio | `action_msgs/srv/CancelGoal` | Solicitud de interrupción en caliente |
| `/go_to_pose/_action/get_result` | Servicio | `<paquete>/action/GoToPose_GetResult` | Espera asíncrona del desenlace final |
| `/go_to_pose/_action/feedback` | Tópico | `<paquete>/action/GoToPose_FeedbackMessage` | Publicación periódica del avance (10 Hz) |
| `/go_to_pose/_action/status` | Tópico | `action_msgs/msg/GoalStatusArray` | Array con estados de los `GoalHandle` activos |

---

### Tabla 2: Resultados Cuantitativos de los Escenarios Experimentales

| Escenario | Pose Inicial $[x_0, y_0, \theta_0]$ | Pose Meta $[x_d, y_d, \theta_d]$ | Error Final $\rho_f$ [m] | Error Angular $\theta_{e,f}$ [rad] | Tiempo [s] | Estado Final Reportado |
|---|---|---|---|---|---|---|
| **E1: Rectilíneo** | $[5.54, 5.54, 0.00]$ | $[9.50, 5.54, 0.00]$ | | | | `STATUS_SUCCEEDED` ☐ |
| **E2: Rotación** | $[5.54, 5.54, 0.00]$ | $[5.54, 5.54, 3.14]$ | | | | `STATUS_SUCCEEDED` ☐ |
| **E3: Patrulla (W1 a W4)** | Misión 4 Waypoints | Ruta Poligonal | | | | Todos `SUCCEEDED` ☐ |
| **E4: Cancelación en Caliente** | $[2.00, 2.00, 0.00]$ | $[10.0, 10.0, 0.00]$ | N/A (Cancelado) | N/A (Cancelado) | | `STATUS_CANCELED` ☐ |
| **E5: Preempción (Reemplazo)** | Meta 1 $\to$ Meta 2 en vuelo | $M_1 \to M_2$ | | | | Meta 1 `ABORTED`, Meta 2 `SUCCEEDED` ☐ |

---

### Tabla 3: Verificación de Robustez y Manejo de Estados

| Prueba de Robustez | Entrada Aplicada | Comportamiento Esperado de la Acción | Comportamiento Real Observado | ¿Aprobado? |
|---|---|---|---|:---:|
| **Rechazo fuera de límites** | $x = 12.0, \, y = 5.0$ | Rechazo en `goal_callback` (`GoalResponse.REJECT`) | | SÍ ☐ &nbsp; NO ☐ |
| **Parada en cancelación** | Llamada a `cancel_goal()` | $v=0, \omega=0$ inmediato, estado `CANCELED` | | SÍ ☐ &nbsp; NO ☐ |
| **Preempción concurrente** | Envío de Meta 2 a $t=2\text{ s}$ | Aborto de Meta 1, navegación fluida a Meta 2 | | SÍ ☐ &nbsp; NO ☐ |
| **Cadencia de Feedback** | Medición con `ros2 topic hz` | $10\text{ Hz} \pm 1\text{ Hz}$ en `/go_to_pose/_action/feedback` | | SÍ ☐ &nbsp; NO ☐ |

---

## 7. Cuestionario de Análisis Técnico y Justificación de Ingeniería

1. **La Tríada de Comunicación:** Compare conceptual y arquitectónicamente Tópicos, Servicios y Acciones en ROS 2. ¿Bajo qué criterios de ingeniería se decide que una tarea robótica debe implementarse como una Acción en lugar de un Servicio o un Tópico?
2. **Los 5 Canales DDS:** Explique en detalle qué sucede en la capa de red cuando un cliente envía una meta con `send_goal_async()`. ¿Por qué la Acción genera 3 servicios y 2 tópicos en lugar de uno solo?
3. **Máquina de Estados y Preempción:** Describa los estados del `GoalStatus` de ROS 2. Explique cómo su nodo servidor gestiona la llegada de una nueva meta mientras otra se encuentra en estado `STATUS_EXECUTING` y por qué esta política es vital en robots de navegación autónoma (Nav2).
4. **Manejo de Cancelación Asíncrona:** ¿Cuál es la diferencia entre que el middleware acepte la cancelación en `cancel_callback` y que el hilo de ejecución confirme la detención física mediante `goal_handle.canceled()`? ¿Por qué nunca debe matarse el proceso bruscamente?
5. **Portabilidad Sim-to-Real con micro-ROS:** Justifique técnicamente cómo la independencia de transporte de ROS 2 permite que el mismo servidor de acciones comande al robot móvil físico con ESP32 simplemente remapeando el tópico `cmd_vel`.

---

## 8. Anexo A: Comprobación Individual del Logro ABET (Obligatorio)

Cada estudiante del equipo debe diligenciar su sección de comprobación individual:

### Estudiante 1: _____________________________________________ Código: ______________
- **Rol técnico asumido en el proyecto:**  
  Arquitectura de acción y callbacks ☐ &nbsp;&nbsp;&nbsp;&nbsp; Control cinemático y preempción ☐ &nbsp;&nbsp;&nbsp;&nbsp; Cliente interactivo y waypoints ☐ &nbsp;&nbsp;&nbsp;&nbsp; Rosbag y métricas ☐
- **Contribución concreta al código (archivos, commits y funciones desarrolladas):**  
  ___________________________________________________________________________________________________
- **Pregunta de sustentación individual asignada por el docente:**  
  ___________________________________________________________________________________________________
- **Respuesta y justificación técnica del estudiante:**  
  ___________________________________________________________________________________________________
- **Nivel de logro individual asignado por el docente:** N1 ☐ &nbsp;&nbsp; N2 ☐ &nbsp;&nbsp; N3 ☐ &nbsp;&nbsp; N4 ☐ &nbsp;&nbsp; N5 ☐

---

### Estudiante 2: _____________________________________________ Código: ______________
- **Rol técnico asumido en el proyecto:**  
  Arquitectura de acción y callbacks ☐ &nbsp;&nbsp;&nbsp;&nbsp; Control cinemático y preempción ☐ &nbsp;&nbsp;&nbsp;&nbsp; Cliente interactivo y waypoints ☐ &nbsp;&nbsp;&nbsp;&nbsp; Rosbag y métricas ☐
- **Contribución concreta al código (archivos, commits y funciones desarrolladas):**  
  ___________________________________________________________________________________________________
- **Pregunta de sustentación individual asignada por el docente:**  
  ___________________________________________________________________________________________________
- **Respuesta y justificación técnica del estudiante:**  
  ___________________________________________________________________________________________________
- **Nivel de logro individual asignado por el docente:** N1 ☐ &nbsp;&nbsp; N2 ☐ &nbsp;&nbsp; N3 ☐ &nbsp;&nbsp; N4 ☐ &nbsp;&nbsp; N5 ☐

---

## 9. Consolidación de Calificación Docente (Escala Zubatronic 0–500)

| Criterio Evaluado | Peso Oficial (%) | Nivel Marcado | Valor Obtenido (0–500) | Aporte Ponderado |
|---|:---:|:---:|:---:|:---:|
| **C1. Arquitectura y Contrato de Acción ROS 2** | 30% | | | |
| **C2. Ciclo de Vida, Máquina de Estados y Preempción** | 25% | | | |
| **C3. Modelado Cinemático en Lazo Cerrado** | 20% | | | |
| **C4. Misión Multi-Waypoint y Rosbag MCAP** | 15% | | | |
| **C5. Trazabilidad Técnica, Trabajo en Equipo y Sim-to-Real** | 10% | | | |
| **TOTAL CONSOLIDADO** | **100%** | | | **________ / 500** |

```text
Nota Académica sobre 5,0 = Nota Consolidada sobre 500 ÷ 100
Aporte a la Nota del Corte 2: E₂ = Nota Académica × 0,30 (30% de la nota de C2)
```

| Resultado Oficial de la Actividad | Registro Oficial |
|---|---|
| **Nota del Proyecto Integrador Corte 2 sobre 500 puntos** | __________ / 500 |
| **Nota Académica Oficial sobre 5,0** | __________ / 5,0 |
| **Número de Criterios en Nivel N3 o superior (Umbral Individual)** | _____ / 5 |
| **¿Cumple el Umbral Individual de Logro ABET (todos en N3+)?** | SÍ ☐ &nbsp;&nbsp;&nbsp;&nbsp; NO ☐ |
