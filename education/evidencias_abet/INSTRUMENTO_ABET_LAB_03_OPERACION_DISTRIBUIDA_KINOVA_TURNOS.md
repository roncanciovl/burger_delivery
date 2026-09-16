# Instrumento de Evidencia y Calificación — Laboratorio 03: Operación Distribuida del Kinova Gen3 (Estación Anfitriona, Monitores, RQT y Envío de Trayectorias por Turnos)

> **Documento Único de Entrega y Evaluación ABET.** Asociado a `education/guias_laboratorio/GUIA_LAB_03_OPERACION_DISTRIBUIDA_KINOVA_TURNOS.docx`.  
> Este documento reúne la entrega integral del equipo: registro de evidencias, tablas de datos experimentales diligenciadas en el laboratorio, respuestas al cuestionario de análisis técnico, Anexo A de comprobación individual y las rúbricas analíticas para la calificación docente y consolidación ABET.

---

## 1. Identificación y Control

| Campo | Registro Oficial |
|---|---|
| **Programa Académico** | Ingeniería Mecatrónica |
| **Asignatura** | ROBOT OPERATING SYSTEM - ROS |
| **Periodo Académico** | 2026-2 |
| **Corte / Instrumento** | Segundo Corte / Laboratorios y evidencias experimentales |
| **Actividad Evaluada** | Laboratorio 03 — Operación Distribuida del Kinova Gen3: Estación Anfitriona, Monitores, RQT y Envío de Trayectorias por Turnos |
| **Número de Grupo / Subgrupo** | |
| **Estudiante 1 (Nombre y Código)** | |
| **Estudiante 2 (Nombre y Código)** | |
| **Estudiante 3 (si aplica)** | |
| **Rol de Estación del Equipo** | Anfitriona (Ethernet) ☐ &nbsp;&nbsp;&nbsp;&nbsp; Monitora Wi-Fi ☐ |
| **Nombre del Archivo de Entrega** | `C2_L03_G<grupo>_<codigo1>_<codigo2>_v1.docx` |
| **Fecha de Realización en Laboratorio** | |
| **Fecha de Entrega del Documento** | |
| **Docente Evaluador** | Ing. Henry Roncancio |
| **Versión del Instrumento** | Versión 1.0 (2026-2) |
| **Unidad de Análisis / Captura** | Equipo colaborativo con comprobación individual (Anexo A) |

---

## 2. Parámetros de Assessment

| Parámetro | Regla Institucional y Metodológica Adoptada |
|---|---|
| **Población o cohorte** | Censo completo de estudiantes matriculados que presentan el Laboratorio 03 en el periodo 2026-2. |
| **Momento de medición** | Sesión experimental presencial y entrega documental única con tablas llenas, dataset rosbag MCAP y sustentación técnica. |
| **Evaluador** | Docente titular de la asignatura ROBOT OPERATING SYSTEM - ROS. |
| **Umbral individual de logro** | Nivel **N3 o superior** (puntaje mínimo de 300 sobre 500) en cada uno de los indicadores de desempeño evaluados. |
| **Meta de cohorte** | Al menos el **70% de los estudiantes evaluables** debe alcanzar el nivel N3 o superior en cada indicador. |
| **Regla de muestreo** | Censo 100%: no se utiliza muestreo; se evalúa la totalidad de los estudiantes y equipos exigibles. |
| **Evidencia faltante** | Entrega exigible sin evidencia obligatoria verificable: se califica en N1 con puntaje 0. Retiro oficial o causa justificada autorizada: `NA / no evaluado` (se excluye del denominador ABET). |
| **Regla de trabajo colaborativo** | Informe único por equipo complementado obligatoriamente con el Anexo A de comprobación individual. Sin comprobación individual suficiente no se infiere logro individual ABET. |

---

## 3. Niveles de Desempeño para Zubatronic/SGDE

| Nivel | Intervalo Zubatronic | Valor Guía | Interpretación y Criterio de Logro |
|---|---:|:---:|---|
| **N5** | 475–500 | 500 | **Excelente:** Evidencia completa, rigurosa, reproducible y explicada con profundidad analítica y matemática sobresaliente. |
| **N4** | 400–474 | 450 | **Bueno:** Desempeño técnico correcto con omisiones menores que no comprometen la operación, trazabilidad ni seguridad. |
| **N3** | 300–399 | 350 | **Aceptable:** Demuestra el desempeño esencial con evidencia verificable. **Es el umbral individual de logro.** |
| **N2** | 150–299 | 250 | **Cumplimiento parcial:** Evidencia incompleta, métricas faltantes, errores en modo seco o fallas de protocolo. |
| **N1** | 0–149 | 100 | **No cumple:** Evidencia mínima, fragmentaria o no funcional. Sin evidencia obligatoria se registra 0. |

---

## 4. Alineación de Criterios, RAE y Student Outcomes

| Criterio | Peso | Student Outcome Principal | Indicador de Desempeño Literal del Programa | Evidencia Directa Obligatoria |
|---|:---:|:---:|---|---|
| **C1. Red DDS distribuida y conectividad (ROS_DOMAIN_ID=0)** | 25% | **SO2** | **2.1.** Diseña soluciones de software para control y monitoreo de robots, integrando contratos de comunicación (QoS) y redes DDS robustas.<br>**2.2.** Incorpora restricciones de red, latencia, ancho de banda y seguridad en hardware heterogéneo. | Ping ICMP a `192.168.1.10` con RTT $< 2\text{ ms}$, `ROS_DOMAIN_ID=0` verificado en todos los terminales, Tabla 1 diligenciada e inspección de topología en `rqt_graph`. |
| **C2. Estación anfitriona, sesión Kortex y telemetría** | 25% | **SO2 / SO6** | **2.1.** Control y monitoreo de robots en tiempo real.<br>**6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos por capas para aislar errores en hardware y software. | Lanzamiento exitoso del driver Kortex (puerto 10000 TCP), tasa de `/joint_states` a 100 Hz, Tabla 2 diligenciada y verificación de no duplicidad de drivers. |
| **C3. Monitoreo remoto e introspección gráfica (RQT / RViz)** | 20% | **SO6 / SO3** | **6.4.** Diagnóstico experimental por capas.<br>**3.1.** Elabora documentación técnica reproducible del sistema ROS 2 y comunica resultados de inspección de grafos y tópicos. | Nodo `kinova_monitor_eqNN` activo, visualización en RViz sobre Wi-Fi vía `TRANSIENT_LOCAL`, auditoría de logs `/rosout` en `rqt_console` y Tabla 3 diligenciada. |
| **C4. Protocolo de turnos, modo seco y seguridad física** | 20% | **SO4 / SO6** | **4.1.** Identifica y mitiga riesgos de seguridad física, paradas de emergencia y protocolos de operación en celdas robóticas.<br>**6.4.** Validación experimental en modo seco. | Validación matemática obligatoria en modo seco (`dry_run:=true`, código 0), autorización verbal, rotación visible de `joint_6` ($\pm 0.05$ a $\pm 0.08$ rad) y Tabla 4 llena. |
| **C5. Trazabilidad en rosbag, trabajo en equipo y cierre** | 10% | **SO3 / SO5** | **3.1 - 3.3.** Documentación técnica y registro de telemetría reproducible.<br>**5.1.** Define roles técnicos y coordina la ejecución colaborativa en equipo. | Dataset MCAP `sesion_turnos_eqNN` con metadatos válidos, desconexión limpia del driver, registro de transición OK->ERROR, Tabla 5 llena y Anexo A individual. |
| **TOTAL** | **100%** | | | |

---

## 5. Registro de Evidencias de la Práctica (E1–E8)

| Código | Evidencia Requerida | Localizador en el Documento / Repositorio / Archivo |
|---|---|---|
| **E1** | Captura de terminal con ping ICMP exitoso al Kinova (`192.168.1.10`) y entre estaciones en la subred `192.168.1.0/24`. | |
| **E2** | Captura de verificación de entorno (`echo $ROS_DOMAIN_ID`) mostrando `0` y captura de `rqt_graph` mostrando la topología de nodos. | |
| **E3** | Captura de la estación anfitriona con sesión Kortex activa (puerto TCP 10000) y `ros2 topic hz /joint_states` registrando $\approx 100\text{ Hz}$. | |
| **E4** | Captura de la estación monitora remota ejecutando `kinova_monitor_eqNN` y modelo del robot en RViz visualizado sobre Wi-Fi. | |
| **E5** | Captura de `rqt_console` filtrando mensajes del dominio 0 y demostrando auditoría centralizada de eventos y advertencias. | |
| **E6** | Captura de ejecución del envío de trayectoria en Modo Seco (`dry_run:=true`) mostrando salida exitosa (código 0) y validación de límites. | |
| **E7** | Captura de envío real al Kinova Gen3, mostrando el cambio angular en `joint_6` y confirmación de recepción en el robot físico. | |
| **E8** | Salida de `ros2 bag info` del dataset MCAP grabado y captura de la monitora mostrando la transición de estado OK -> ERROR al apagar el driver. | |

---

## 6. Tablas de Registro Experimental (Diligenciadas por el Equipo)

> **Instrucciones para el equipo:** Diligencie de forma completa y precisa cada una de las siguientes cinco tablas con los datos reales obtenidos durante la sesión de laboratorio.

### Tabla 1: Inventario de Interfaces de Red y Rol de Equipo

| Equipo / Grupo | Interfaz de Red (eth0 / wlan0) | Dirección IP Asignada | Máscara / Gateway | Rol Asignado (Anfitriona / Monitora) | Ping a Kinova 192.168.1.10 (RTT ms / Pérdida) |
|---|---|---|---|---|---|
| **Estación Anfitriona (Gateway)** | | | | Anfitriona (Driver Kortex) | RTT: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ms &nbsp; Pérdida: |
| **Equipo Evaluado (Propio)** | | | | | RTT: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ms &nbsp; Pérdida: |
| **Estación Remota G01** | | | | Monitora Wi-Fi | RTT: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ms &nbsp; Pérdida: |
| **Estación Remota G02** | | | | Monitora Wi-Fi | RTT: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ms &nbsp; Pérdida: |
| **Estación Remota G03** | | | | Monitora Wi-Fi | RTT: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ms &nbsp; Pérdida: |
| **Estación Remota G04** | | | | Monitora Wi-Fi | RTT: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ms &nbsp; Pérdida: |

---

### Tabla 2: Estado y Telemetría de la Estación Anfitriona (Fase 1)

| Parámetro de la Estación Anfitriona | Valor Medido / Estado Experimental | Método de Verificación / Comando Ejecutado |
|---|---|---|
| **Hostname e IP de la estación anfitriona** | | `hostname -I` |
| **Tipo de conexión física al router / robot** | Cable Ethernet directo / Switch | Inspección visual de hardware |
| **RTT promedio de ping a 192.168.1.10** | ms | `ping -c 10 192.168.1.10` |
| **PID y estado del nodo driver Kortex** | PID: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Estado: | `ps aux \| grep kortex_bringup` |
| **Frecuencia medida de `/joint_states`** | Hz (nominal 100 Hz) | `ros2 topic hz /joint_states` |
| **Frecuencia de `/burger/kinova/joint_states`** | Hz | `ros2 topic hz /burger/kinova/joint_states` |
| **Estado de sesión Kortex (puerto TCP 10000)** | Conectada / Estable | `ss -tulpn \| grep 10000` |
| **Archivo rosbag MCAP de registro general** | | `ros2 bag info <nombre_bag>` |

---

### Tabla 3: Despliegue y Estado de Estaciones Monitoras (Fases 2 y 4)

| Grupo | Nombre de Nodo Monitor | IP Estación (Wi-Fi) | Parámetro `rol_estacion` | Frecuencia `/joint_states` (Hz) | Estado de Movimiento (`/burger/kinova/habilitacion`) | RViz / rqt_graph OK? | Logs en rqt_console | Tiempo transición OK->ERROR al apagar driver |
|:---:|---|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **G01** | `kinova_monitor_eq01` | | | | | | | |
| **G02** | `kinova_monitor_eq02` | | | | | | | |
| **G03** | `kinova_monitor_eq03` | | | | | | | |
| **G04** | `kinova_monitor_eq04` | | | | | | | |
| **G05** | `kinova_monitor_eq05` | | | | | | | |

---

### Tabla 4: Registro de Turnos de Movimiento Articular y Prueba Final (Fase 3)

| Turno # / Prueba | Grupo | Hora Inicio | Pose / joint_6 Inicial (rad) | Meta Solicitada | Validación Modo Seco (Código) | Envío Real al Robot (Resultado) | Pose Final (joint_6) | Diferencia Real vs Esp | Hora Cierre |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **1** | | | | *(+0.05 a +0.08)* | Código: | | | | |
| **2** | | | | *(-0.05 a -0.08)* | Código: | | | | |
| **3** | | | | *(+0.05 a +0.08)* | Código: | | | | |
| **4** | | | | *(-0.05 a -0.08)* | Código: | | | | |
| **5** | | | | *(+0.05 a +0.08)* | Código: | | | | |
| **Prueba Final** | Todos | | Origen autodescubierto | Coreografía 25 deltas | Código 0 | SUCCESSFUL | Retorno a origen | 0.000 rad | OK |

---

### Tabla 5: Incidentes, Anomalías y Diagnóstico Metódico por Capas

| Momento / Fase | Síntoma Observado | Capa Diagnosticada (1:Red, 2:DDS/QoS, 3:Driver, 4:Lógica/Acción, 5:Protocolo) | Procedimiento de Inspección Ejecutado | Acción Correctiva Aplicada y Resultado |
|---|---|---|---|---|
| **Incidente 1** | | | | |
| **Incidente 2** | | | | |
| **Incidente 3** | | | | |

---

## 7. Cuestionario de Análisis y Discusión Técnica (Respondido por el Equipo)

### Pregunta 1: Unicidad del Driver y Consulta de Identidad en el Grafo
*Con base en la Tabla 2 y la identidad anunciada en el grafo (`rqt_graph` o `/burger/kinova/identidad_estacion`), argumente cómo la convención de estación anfitriona única y el dominio compartido permiten responder '¿quién tiene el control del robot?' desde cualquier estación sin necesidad de coordinación verbal previa.*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 2: Emisores Simultáneos y Política de Cancelación en follow_joint_trajectory
*En ROS 2 y `ros2_control`, el servidor de acción `/joint_trajectory_controller/follow_joint_trajectory` acepta metas de cualquier nodo en el dominio 0. Si dos estaciones envían una meta de trayectoria simultáneamente, ¿qué le ocurre a la primera meta y por qué? ¿Por qué el protocolo colaborativo de turnos es indispensable cuando todos comparten el `ROS_DOMAIN_ID=0`?*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 3: Compromiso de Enlace Ethernet (Anfitriona) vs. Wi-Fi (Monitoras)
*Compare cuantitativamente la frecuencia y el jitter de `/joint_states` medidos en la estación anfitriona (cable Ethernet) frente a las estaciones monitoras (Wi-Fi). ¿Por qué la estación monitora puede operar sobre Wi-Fi mientras que la anfitriona debe estar obligatoriamente conectada por cable? Considere el bucle de control a 1 kHz del API Kortex.*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 4: Trazabilidad y Reconstrucción Cronológica a partir del rosbag MCAP
*A partir del archivo rosbag grabado (`sesion_turnos_eqNN`) y los registros de la Tabla 4, reconstruya la cronología exacta de un turno: demuestre qué nodo (por su sufijo `_eqNN`) emitió la meta, el instante en que el controlador la aceptó y el instante en que finalizó el movimiento.*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 5: Propuesta de Garantía de Turnos por Software vs. Protocolo Humano
*El nodo monitor publica el estado de 'habilitación de movimiento', pero el cliente CLI no lo consulta obligatoriamente antes de enviar. Proponga una arquitectura de software (por ejemplo, un servicio ROS 2 de concesión de turnos con Mutex distribuido en la anfitriona). ¿Qué ventajas operativas tendría y qué nuevos modos de falla (ej. deadlocks por desconexión) introduciría?*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 6: Pérdida de Enlace Durante el Movimiento Físico
*Si durante la ejecución de una trayectoria articular se cae repentinamente el enlace Wi-Fi de la estación que envió la meta, ¿se detiene inmediatamente el robot? Razone su respuesta identificando dónde reside físicamente el controlador de trayectoria y dónde reside el cliente de acción.*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 7: Aislamiento vs. Colaboración en DDS (Impacto del ROS_DOMAIN_ID)
*¿Qué ocurriría si un equipo olvida configurar el `ROS_DOMAIN_ID=0` y ejecuta su monitor en el dominio por defecto (`ROS_DOMAIN_ID=10`)? ¿Podría recibir telemetría o enviar trayectorias al Kinova? Explique por qué el dominio común es condición matemática necesaria para la operación distribuida.*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 8: Posicionamiento Absoluto vs. Deltas Relativos y Autodescubrimiento de Pose
*Compare la operación de `safe_trajectory_client` frente a `safe_sequence_client`. ¿Por qué en el cliente de trayectoria individual fue estrictamente necesario descubrir las posiciones absolutas reales de `/joint_states` antes de formular la meta para evitar el bloqueo por `max_joint_delta_rad` (0.10 rad), mientras que el cliente de secuencia pudo ejecutarse desde cualquier pose sin transcribir coordenadas a mano? ¿Qué riesgos y ventajas de seguridad introduce cada enfoque en entornos industriales colaborativos?*

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
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas en la práctica (ej. configuración de red, ejecución de la anfitriona, monitoreo RQT, cálculo de trayectorias, validación en modo seco o gestión de rosbag). | |
| **2. Justificación de ROS_DOMAIN_ID=0:** Explique por qué todos los equipos debieron converger al dominio 0 y qué sucede en el descubrimiento DDS cuando coinciden los dominios en la misma LAN. | |
| **3. Explicación del Modo Seco y deltas angulares:** Justifique técnicamente qué valida el cliente en modo seco (`dry_run:=true`) y por qué se restringió el desplazamiento de `joint_6` a un rango de $\pm 0.05$ a $\pm 0.08$ rad. | |
| **4. Diagnóstico por capas de una anomalía:** Describa un incidente o advertencia observado durante la sesión (en `rqt_console`, consola o RViz) y cómo lo analizó mediante el protocolo por capas. | |
| **5. Autoría y reproducibilidad:** Identifique qué comandos, filas de tablas, capturas o secciones del rosbag fueron ejecutados y documentados directamente por usted. | |

**Estado de comprobación individual Estudiante 1:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐  
**Firma del Estudiante 1:** ________________________________________

---

### Estudiante 2: __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas en la práctica (ej. configuración de red, ejecución de la anfitriona, monitoreo RQT, cálculo de trayectorias, validación en modo seco o gestión de rosbag). | |
| **2. Justificación de ROS_DOMAIN_ID=0:** Explique por qué todos los equipos debieron converger al dominio 0 y qué sucede en el descubrimiento DDS cuando coinciden los dominios en la misma LAN. | |
| **3. Explicación del Modo Seco y deltas angulares:** Justifique técnicamente qué valida el cliente en modo seco (`dry_run:=true`) y por qué se restringió el desplazamiento de `joint_6` a un rango de $\pm 0.05$ a $\pm 0.08$ rad. | |
| **4. Diagnóstico por capas de una anomalía:** Describa un incidente o advertencia observado durante la sesión (en `rqt_console`, consola o RViz) y cómo lo analizó mediante el protocolo por capas. | |
| **5. Autoría y reproducibilidad:** Identifique qué comandos, filas de tablas, capturas o secciones del rosbag fueron ejecutados y documentados directamente por usted. | |

**Estado de comprobación individual Estudiante 2:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐  
**Firma del Estudiante 2:** ________________________________________

---

## 9. Selección del Nivel Alcanzado por Criterio (Rúbricas Docentes)

Marque con una **X** una sola casilla por criterio y registre el valor entero (0 a 500) dentro de la banda correspondiente.

### C1. Red DDS distribuida y conectividad (ROS_DOMAIN_ID=0) — Peso 25% — Student Outcome SO2

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, automatiza la verificación de red con scripts reproducibles, analiza la asignación de puertos RTPS en el dominio 0 con CycloneDDS y justifica los mecanismos de aislamiento y descubrimiento multicast en Wi-Fi vs Ethernet. |
| ☐ | **N4 — 400–474** | Configura `ROS_DOMAIN_ID=0`, verifica ping al Kinova (`192.168.1.10`) con RTT $< 2\text{ ms}$, diligencia completamente la Tabla 1 y valida el descubrimiento global de nodos mediante `rqt_graph`. |
| ☐ | **N3 — 300–399** | Configura la IP en la subred `192.168.1.0/24`, exporta `ROS_DOMAIN_ID=0`, comprueba ping al robot y visualiza los tópicos principales del dominio. |
| ☐ | **N2 — 150–299** | Presenta conflictos de conectividad por discrepancia en variables de entorno (`ROS_DOMAIN_ID` incorrecto) o llena de forma incompleta la Tabla 1. |
| ☐ | **N1 — 0–149** | No logra comunicación con la red del robot o no presenta evidencias obligatorias verificables. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C2. Estación anfitriona, sesión Kortex y telemetría — Peso 25% — Student Outcome SO2 / SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, analiza cuantitativamente el jitter de `/joint_states` a 100 Hz frente al ciclo Kortex a 1 kHz, y diseña protocolos de contingencia ante desconexión de hardware o fallos en el socket TCP 10000. |
| ☐ | **N4 — 400–474** | Levanta correctamente la estación anfitriona (o se coordina con ella), verifica la sesión TCP Kortex activa a 1000 Hz, mide la tasa estable de `/joint_states` a 100 Hz y diligencia con exactitud la Tabla 2. |
| ☐ | **N3 — 300–399** | Participa en la puesta en marcha de la anfitriona, confirma la publicación de `/joint_states` y llena los parámetros esenciales de la Tabla 2. |
| ☐ | **N2 — 150–299** | Intenta levantar un driver duplicado compitiendo por el puerto Kortex o no logra verificar la frecuencia de telemetría articular. |
| ☐ | **N1 — 0–149** | No comprende la convención de anfitriona única, bloquea la controladora Kortex o carece de evidencias de telemetría. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C3. Monitoreo remoto e introspección gráfica (RQT / RViz) — Peso 20% — Student Outcome SO6 / SO3

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, correlaciona eventos transitorios en `rqt_console` con métricas de QoS en Wi-Fi, explica la persistencia de `/robot_description` vía `TRANSIENT_LOCAL` y optimiza la configuración de RViz para bajo consumo de ancho de banda. |
| ☐ | **N4 — 400–474** | Despliega el monitor con identidad única (`kinova_monitor_eqNN`), visualiza el robot en RViz sin lanzar drivers locales, audita logs en `rqt_console` y llena completamente la Tabla 3. |
| ☐ | **N3 — 300–399** | Ejecuta el nodo monitor en su estación, visualiza el robot en RViz en el dominio 0 y documenta su estado en la Tabla 3. |
| ☐ | **N2 — 150–299** | Lanza el monitor con colisión de nombres de nodo o presenta fallos de visualización en RViz por falta de recepción de transformaciones TF. |
| ☐ | **N1 — 0–149** | No logra ejecutar el monitoreo remoto o carece de capturas de RQT y RViz. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C4. Protocolo de turnos, modo seco y seguridad física — Peso 20% — Student Outcome SO4 / SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, justifica matemáticamente los límites de interpolación del controlador (`max_joint_delta_rad`), propone mecanismos de exclusión mutua por software y demuestra liderazgo en la coordinación segura de la celda. |
| ☐ | **N4 — 400–474** | Ejecuta rigurosamente la validación en modo seco (`dry_run:=true`, código 0), solicita autorización verbal, supervisa la parada de emergencia, ejecuta el movimiento en `joint_6` ($\pm 0.05$ a $\pm 0.08$ rad) y llena la Tabla 4. |
| ☐ | **N3 — 300–399** | Realiza la validación en modo seco antes del envío real, respeta el turno asignado, ejecuta el movimiento dentro de límites y diligencia la Tabla 4. |
| ☐ | **N2 — 150–299** | Envía metas al robot sin validación previa en modo seco, excede el ángulo sugerido o no coordina el turno verbalmente. |
| ☐ | **N1 — 0–149** | Incurre en actos inseguros en la celda robótica, envía trayectorias no autorizadas o carece de registros de turnos. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C5. Trazabilidad en rosbag, trabajo en equipo y cierre — Peso 10% — Student Outcome SO3 / SO5

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, el rosbag MCAP incluye filtros quirúrgicos de tópicos, el Anexo A demuestra una complementariedad y dominio técnico individual sobresaliente, y propone mejoras al protocolo de apagado del robot. |
| ☐ | **N4 — 400–474** | Graba y documenta el dataset MCAP con metadatos válidos, registra la transición OK->ERROR en el cierre ordenado, diligencia la Tabla 5 y el Anexo A individual evidencia autoría y comprensión. |
| ☐ | **N3 — 300–399** | Entrega el informe con tablas llenas, registra el cierre del driver y aporta el Anexo A con respuestas verificables. |
| ☐ | **N2 — 150–299** | Documento incompleto, tablas con datos faltantes, cierre abrupto del sistema o Anexo A con respuestas genéricas. |
| ☐ | **N1 — 0–149** | No entrega el informe, no aporta rosbag ni evidencias de cierre, o el Anexo A demuestra ausencia de participación. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

## 10. Consolidado Final de Calificación Académica

| Criterio Evaluado | Peso Oficial (%) | Nivel Marcado | Valor Obtenido (0–500) | Aporte Ponderado |
|---|:---:|:---:|:---:|:---:|
| **C1. Red DDS distribuida y conectividad (ROS_DOMAIN_ID=0)** | 25% | | | |
| **C2. Estación anfitriona, sesión Kortex y telemetría** | 25% | | | |
| **C3. Monitoreo remoto e introspección gráfica (RQT / RViz)** | 20% | | | |
| **C4. Protocolo de turnos, modo seco y seguridad física** | 20% | | | |
| **C5. Trazabilidad en rosbag, trabajo en equipo y cierre** | 10% | | | |
| **TOTAL CONSOLIDADO** | **100%** | | | **________ / 500** |

```text
Nota Académica sobre 5,0 = Nota Consolidada sobre 500 ÷ 100
Aporte a Laboratorios del Corte: L₂ = Nota Académica sobre 5,0
```

| Resultado Oficial de la Actividad | Registro |
|---|---|
| **Nota de Laboratorio 03 sobre 500 puntos** | __________ / 500 |
| **Nota Académica Oficial sobre 5,0** | __________ / 5,0 |
| **Número de Criterios en N3 o superior (Umbral Individual)** | _____ / 5 |
| **¿Cumple el Umbral Individual de Logro ABET (todos en N3+)?** | SÍ ☐ &nbsp;&nbsp;&nbsp;&nbsp; NO ☐ |

---

## 11. Consolidación ABET y Cierre de Mejora Continua

| Student Outcome / Indicador Evaluado | N Estudiantes Evaluables | N en Nivel N3 o Superior | Porcentaje de Logro (%) | Meta Institucional | Hallazgo Docente |
|---|:---:|:---:|:---:|:---:|---|
| **SO2 / Indicador 2.1 y 2.2** (Red DDS y Hardware) | | | | 70% | |
| **SO6 / Indicador 6.4** (Diagnóstico por Capas) | | | | 70% | |
| **SO3 / Indicador 3.1 y 3.3** (Documentación y Trazabilidad) | | | | 70% | |
| **SO4 / Indicador 4.1** (Seguridad Física y Celdas Robóticas) | | | | 70% | |
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
