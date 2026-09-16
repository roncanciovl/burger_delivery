# GUÍA DE LABORATORIO 03 (CORTE 2): OPERACIÓN DISTRIBUIDA DEL KINOVA GEN3 — CONVENCIÓN DE ESTACIÓN ANFITRIONA, MONITORES MULTI-DISPOSITIVO Y PROTOCOLO DE TURNOS DE TRAYECTORIAS ARTICULARES

---

| FACULTAD | PROGRAMA | ASIGNATURA | SEMESTRE | CÓDIGO GUÍA | REVISIÓN |
|:---|:---|:---|:---:|:---:|:---:|
| Facultad de Ingeniería | Ingeniería Mecatrónica | ROBOT OPERATING SYSTEM - ROS | VIII – IX | GL-AA-F-1 / LAB-03 (Corte 2) | 1.0 (2026-2) |

---

## 1. CONTROL DE CAMBIOS

| Descripción del Cambio | Justificación | Fecha |
|---|---|:---:|
| Creación e implementación de la Guía de Laboratorio 03 | Diseño del protocolo de operación distribuida del robot manipulador Kinova Gen3 (6-DOF) en LAN única `192.168.1.0/24`, definiendo la convención de estación anfitriona por cable Ethernet y estaciones monitoras por Wi-Fi. | 16/09/2026 |
| Reestructuración de la Fase 0 y eliminación de simulación en dominios separados | Se unifica a todos los grupos en `ROS_DOMAIN_ID=0` desde el inicio para interacción directa con el robot físico real en la IP `192.168.1.10`. | 16/09/2026 |
| Integración de herramientas RQT y profundización en Modo Seco | Incorporación de `rqt_graph` para el grafo distribuido, `rqt_console` para logging centralizado y formalización del concepto de modo seco (*dry run*). | 16/09/2026 |

---

## 2. INTRODUCCIÓN

### 2.1. Contexto Teórico y Desafío Multi-Estación

En celdas de manufactura y robótica industrial colaborativa, un único robot manipulador de alta gama debe ser compartido y operado por múltiples estaciones de ingeniería. El laboratorio cuenta con **un robot manipulador Kinova Gen3 (6 GDL) con pinza Robotiq 2F-85** y varios equipos de trabajo estudiantiles.

La controladora del Kinova Gen3 (API Kortex) admite **una sola sesión de control en tiempo real** a través del puerto TCP `10000` (ciclo a 1 kHz). Si dos o más computadores intentan lanzar el driver `kortex_bringup` simultáneamente, compiten por el socket TCP de tiempo real, provocando caídas de enlace y disparando paradas de seguridad (*Safety Faults*).

Para resolver esta restricción física sin aislar a los grupos, se implementa la **convención de estación anfitriona** ([`TROUBLESHOOTING.md`](../../TROUBLESHOOTING.md) §2.0):
1. **Dominio Común:** Todas las estaciones del laboratorio operan estrictamente en `ROS_DOMAIN_ID=0`.
2. **Estación Anfitriona Única:** Un computador designado, conectado físicamente por **cable Ethernet** al router, es el **único** autorizado a ejecutar el driver del robot (`start_driver:=true robot_ip:=192.168.1.10`).
3. **Estaciones Monitoras:** Los computadores de los estudiantes se conectan por **Wi-Fi** al router y operan como monitoras: escuchan la telemetría `/joint_states` y publican su propio diagnóstico, sin driver local (`start_driver:=false`).
4. **Protocolo de Turnos entre Personas:** Debido a que el servidor de acción del controlador de ROS 2 (`FollowJointTrajectory`) acepta metas de cualquier nodo en el dominio, el movimiento físico se gestiona mediante un estricto protocolo de turnos supervisado entre los integrantes del laboratorio.

```text
                 Router TP-Link AX12 — 192.168.1.0/24 — ROS_DOMAIN_ID=0
          │                          │                                │
  [ Kinova Gen3 6 GDL ]      [ Grupo anfitrión ]              [ Grupos monitores ]
     192.168.1.10          Ethernet, 1 sola estación          WiFi, N estaciones
          │                          │                                │
          │◄── sesión Kortex ────────┤  kortex_bringup (driver)       │
          │    (TCP 10000, 1 kHz)    │  kinova_monitor                 │  kinova_monitor_eqNN
          │                          │                                │
          │                          ├── DDS: /joint_states ─────────►│
          │                          ├── DDS: /burger/kinova/diagnostics ►│
          │                          │◄─ acción FollowJointTrajectory ─┤  safe_trajectory_client_eqNN
          │                          │    (sólo el grupo con el TURNO)  │  (uno a la vez)
```

### 2.2. El Paquete de Referencia y Seguridad por Software

El paquete [`burger_kinova_reference`](../../burger_kinova_reference/README.md) implementa la capa de enlace y seguridad:
- `kinova_monitor`: Valida la salud de `/joint_states` (> 20 Hz, latencia < 1.0 s), audita el estado de los controladores y publica en `/burger/kinova/diagnostics` la **identidad verificada de la estación** (`anfitriona` o `cliente`).
- `safe_trajectory_client`: Cliente de acción seguro que valida límites articulares, velocidad, delta máximo de desplazamiento (`max_joint_delta_rad = 0.10 rad`), vigencia de telemetría y soporte de **Modo Seco (*Dry Run*)** antes de emitir cualquier trayectoria física.

### 2.3. Lo que ninguna capa de software impide

El servidor de acción del controlador en `ros2_control` acepta metas de cualquier nodo en el dominio `0`. Si dos estaciones envían trayectorias simultáneas, la segunda meta **reemplaza y cancela** inmediatamente a la primera:

```text
[grupo A] [RESULTADO] Meta ACEPTADA por el controlador.
[grupo A] [RESULTADO] Meta CANCELADA | error_code=SUCCESSFUL | error_string=""     ← código de salida 3
[grupo B] [RESULTADO] Meta ACEPTADA por el controlador.
[grupo B] [RESULTADO] Trayectoria completada con éxito | error_code=SUCCESSFUL
```

Por este motivo, **el turno es un protocolo de ingeniería riguroso entre personas**, y este laboratorio entrena dicha competencia operativa.

---

## 3. OBJETIVOS

### 3.1. Objetivo General
Operar el robot manipulador Kinova Gen3 real desde múltiples estaciones de trabajo en red bajo la convención de estación anfitriona y dominio compartido (`ROS_DOMAIN_ID=0`), auditando la salud del enlace DDS mediante RQT y ejecutando el protocolo colaborativo de turnos de trayectorias articulares con validación obligatoria en modo seco.

### 3.2. Objetivos Específicos
1. **Configurar el entorno de red y dominio común** (`ROS_DOMAIN_ID=0`, CycloneDDS y conectividad IP con el Kinova real `192.168.1.10`) en todos los equipos del laboratorio.
2. **Poner en marcha la estación anfitriona** con el driver del robot real, verificando la sesión TCP Kortex a 1 kHz, los controladores activos y la telemetría a 100 Hz.
3. **Desplegar estaciones monitoras con identidad propia** en el grafo, comprobando la identidad de la anfitriona y analizando la topología y logs mediante RQT (`rqt_graph` y `rqt_console`).
4. **Visualizar el robot real en RViz** desde estaciones remotas sin ejecutar drivers locales, comprendiendo la durabilidad `TRANSIENT_LOCAL` en `/robot_description`.
5. **Ejecutar el protocolo de turnos de trayectoria**, aplicando validación matemática previa en **Modo Seco** (`dry_run:=true`), autorización verbal y supervisión con parada de emergencia física.
6. **Cerrar ordenadamente la sesión Kortex** y observar desde las estaciones monitoras la transición de pérdida de enlace en tiempo real.

---

## 4. DESCRIPCIÓN DE LA PRÁCTICA

### 4.1. Fases de la Práctica

```text
  +---------------------------------------------------------------------------------------+
  |  FASE 0: CONFIGURACIÓN DE RED Y ROS_DOMAIN_ID=0 (todos los grupos, IP: 192.168.1.10)  |
  |                                        ↓                                              |
  |  FASE 1: PUESTA EN MARCHA DE LA ESTACIÓN ANFITRIONA (grupo anfitrión, robot real)     |
  |                                        ↓                                              |
  |  FASE 2: ESTACIONES MONITORAS, RQT Y RVIZ (grupos monitores por Wi-Fi)                |
  |                                        ↓                                              |
  |  FASE 3: ENVÍO DE TRAYECTORIAS POR TURNOS (modo seco previo y envío supervisado)      |
  |                                        ↓                                              |
  |  FASE 4: CIERRE ORDENADO Y OBSERVACIÓN DE LA PÉRDIDA DE ENLACE                        |
  +---------------------------------------------------------------------------------------+
```

### 4.2. Resultados de Aprendizaje Evaluables (RAE), Ponderación y Ubicación en el Corte 2

> [!IMPORTANT]
> **Asignación Académica Oficial — Corte 2:**  
> Este laboratorio corresponde a la evaluación experimental del **Segundo Corte (Corte 2)**:
> - **Componente Univex:** Alimenta el componente agregado `L_C2` (Laboratorios y evidencias experimentales), el cual tiene una ponderación del **42%** dentro de la nota del Corte 2.
> - **Regla de calificación:** La nota del componente se consolida promediando los laboratorios del corte: $L_2 = \text{promedio}(N_{\text{L02}}, N_{\text{L03}})$, donde $N_j = \text{Puntaje}/100 \in [0.0, 5.0]$.
> - **Escala:** 500 puntos (escala estándar Zubatronic / SGDE).
> - **Modalidad de entrega:** Entrega grupal por equipo. Un solo integrante (el entregante designado) sube el archivo único `C2_L03_G<grupo>_<codigo1>_<codigo2>_v1.docx` con las 5 tablas llenas, preguntas respondidas y el Anexo A de comprobación individual.

| Criterio | RAE / Indicador Oficial del Syllabus | Student Outcome | Ponderación |
|---|---|:---:|:---:|
| **C1. Red DDS distribuida y conectividad (`ROS_DOMAIN_ID=0`)** | **2.1.** Diseña soluciones de software integrando contratos QoS y redes DDS robustas.<br>**2.2.** Incorpora restricciones de red, latencia y seguridad en hardware heterogéneo. | SO2 | 25% |
| **C2. Estación anfitriona, sesión Kortex y telemetría** | **2.1.** Control y monitoreo de robots en tiempo real.<br>**6.4.** Diagnóstico experimental de capas de comunicación y controladores. | SO2 / SO6 | 25% |
| **C3. Monitoreo remoto e introspección gráfica (RQT / RViz)** | **6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos por capas.<br>**3.1.** Inspección de grafos, tópicos y contratos TF. | SO6 / SO3 | 20% |
| **C4. Protocolo de turnos, modo seco y seguridad física** | **4.1.** Identifica riesgos de seguridad física, paradas de emergencia y celdas robóticas.<br>**6.4.** Validación experimental en modo seco antes de energizar actuadores. | SO4 / SO6 | 20% |
| **C5. Trazabilidad en rosbag, trabajo en equipo y cierre** | **3.1 - 3.3.** Elabora documentación técnica reproducible y registra datasets.<br>**5.1.** Define roles técnicos y coordina la ejecución en equipo. | SO3 / SO5 | 10% |
| **TOTAL** | | | **100% (5.0)** |

---

### 4.3. Rúbrica Detallada por Niveles de Desempeño

#### C1. Red DDS distribuida y conectividad (`ROS_DOMAIN_ID=0`) — Peso: 25% (SO2)
- **N5 (475–500):** Configura y automatiza la exportación de variables de entorno y perfiles XML en todas las terminales; justifica con rigor técnico la mitigación de fragmentación UDP en Wi-Fi con CycloneDDS y explica la segmentación lógica por dominios en la LAN compartida.
- **N4 (400–474):** Configura correctamente `ROS_DOMAIN_ID=0`, `rmw_cyclonedds_cpp`, `cyclonedds.xml`, reinicia el daemon de ROS 2 y demuestra conectividad fluida mediante ping a `192.168.1.10`.
- **N3 (300–399) [Umbral de Logro]:** Configura el dominio 0 y el middleware indicado; comprueba conectividad con el robot real y el estado del daemon sin errores.
- **N2 (150–299):** Presenta inconsistencias de dominio en alguna terminal, olvida reiniciar el daemon o no comprueba la conectividad IP previa.
- **N1 (0–149):** No logra conectividad con la red `192.168.1.0/24` o utiliza un dominio diferente quedando aislado del sistema.

#### C2. Estación anfitriona, sesión Kortex y telemetría — Peso: 25% (SO2 / SO6)
- **N5 (475–500):** Demuestra la unicidad del driver mediante inspección de sockets TCP (`ss -tanp`), justifica por qué la anfitriona debe conectarse por cable Ethernet (1 kHz determinista) comparando con pérdidas en Wi-Fi, y audita la estabilidad de `/joint_states` a 100 Hz y controladores activos.
- **N4 (400–474):** Despliega el launch de la anfitriona con hardware real (`start_driver:=true`), verifica la sesión TCP Kortex en el puerto 10000 y comprueba la identidad anunciada como `anfitriona` en el diagnóstico.
- **N3 (300–399) [Umbral de Logro]:** Ejecuta la estación anfitriona con hardware real; verifica que los controladores estén activos y que `/joint_states` publique a frecuencia nominal.
- **N2 (150–299):** Lanza el driver sin verificar previamente si el robot estaba ocupado, o presenta inestabilidad en los controladores.
- **N1 (0–149):** No logra establecer sesión Kortex o causa bloqueos por lanzar drivers duplicados en hardware real.

#### C3. Monitoreo remoto e introspección gráfica (RQT / RViz) — Peso: 20% (SO6 / SO3)
- **N5 (475–500):** Justifica la durabilidad `TRANSIENT_LOCAL` en `/robot_description` explicando por qué RViz visualiza la pose sin relanzar el modelo; analiza en `rqt_graph` las relaciones entre publicadores/suscriptores y filtra logs en `rqt_console` diagnosticando la salud global de la red.
- **N4 (400–474):** Lanza el monitor del grupo con nombre único (`kinova_monitor_eqNN`), visualiza el robot en RViz en su PC sin driver local, inspecciona la topología en `rqt_graph` y filtra mensajes en `rqt_console`.
- **N3 (300–399) [Umbral de Logro]:** Despliega el monitor con identidad propia, abre RViz observando el robot real e inspecciona nodos y logs en RQT.
- **N2 (150–299):** Ejecuta el monitor sin renombrar causando advertencias de nodos duplicados, o no logra visualizar el modelo en RViz.
- **N1 (0–149):** No despliega el nodo monitor o no realiza la introspección con herramientas gráficas.

#### C4. Protocolo de turnos, modo seco y seguridad física — Peso: 20% (SO4 / SO6)
- **N5 (475–500):** Explica en profundidad la necesidad del protocolo de turnos ante la preemptibilidad de acciones en ROS 2; valida la meta en Modo Seco interpretando el reporte de deltas, custodia la parada de emergencia y ejecuta el movimiento físico con exactitud milimétrica.
- **N4 (400–474):** Construye la meta articular absoluta para `joint_6`, ejecuta el modo seco con código de salida `0`, solicita autorización y realiza el envío físico registrando la pose final.
- **N3 (300–399) [Umbral de Logro]:** Cumple el protocolo de turnos: modo seco previo aprobado, confirmación interactiva y verificación de la pose final.
- **N2 (150–299):** Intenta enviar trayectorias sin modo seco previo, o redondea erróneamente las articulaciones generando advertencias de delta.
- **N1 (0–149):** Envía trayectorias fuera de turno, viola los límites de seguridad o vulnera las normas de seguridad del laboratorio.

#### C5. Trazabilidad en rosbag, trabajo en equipo y cierre — Peso: 10% (SO3 / SO5)
- **N5 (475–500):** Registra la sesión en rosbag MCAP; analiza con precisión la pérdida de enlace sincronizada en las monitoras al cerrar la sesión; completa todas las tablas con datos rigurosos y el Anexo A demuestra una contribución técnica sobresaliente de cada integrante.
- **N4 (400–474):** Graba la evidencia en rosbag, observa la transición `OK -> ERROR` al cerrar el driver, completa las tablas de la guía y responde las preguntas de análisis con solvencia.
- **N3 (300–399) [Umbral de Logro]:** Cierra limpiamente la sesión, verifica la liberación del puerto Kortex, llena las tablas y demuestra trabajo en equipo en el Anexo A.
- **N2 (150–299):** Cierra el driver de forma abrupta (`kill -9`) dejando la sesión bloqueada, o la documentación del anexo individual es insuficiente.
- **N1 (0–149):** No entrega el documento con las tablas llenas o no presenta evidencia de comprobación individual.

---

## 5. MATERIALES Y EQUIPOS

### 5.1. Equipos del Laboratorio
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Brazo manipulador Kinova Gen3 (6-DOF) con pinza Robotiq 2F-85 (**IP real: `192.168.1.10`**, puerto Kortex `10000`) | 1 | Unidad |
| Router TP-Link AX12 (SSID `ros2`, subred `192.168.1.0/24`, Gateway `192.168.1.1`) | 1 | Unidad |
| Estación anfitriona con Ubuntu 24.04, ROS 2 Jazzy y **cable Ethernet** al router | 1 | Unidad |
| Pulsador de parada de emergencia física, accesible desde la estación anfitriona | 1 | Unidad |

### 5.2. Equipos del Estudiante (por grupo)
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Portátil con Ubuntu 24.04 LTS y ROS 2 Jazzy conectado por **WiFi** al SSID `ros2` (o por cable) | 1 | Unidad |
| Workspace `~/ros2_ws` con `burger_delivery` y `ros2_kortex` compilados | 1 | Workspace |
| Formato de entrega del laboratorio diligenciable (DOCX) | 1 | Documento |

---

## 6. SEGURIDAD EN EL LABORATORIO

> [!WARNING]
> 1. **Un solo driver en hardware real.** Ninguna estación distinta de la anfitriona autorizada puede ejecutar `start_driver:=true` con el robot real.
> 2. **Custodia de Parada de Emergencia.** Un integrante del grupo anfitrión debe permanecer junto al pulsador físico de parada de emergencia durante **todo** turno de envío.
> 3. **Área de Barrido de Seguridad.** Mantener un radio libre de 1.2 metros alrededor de la base del robot. Nadie debe ingresar al área de operación mientras haya un turno activo.
> 4. **Movimiento seguro y acotado.** En este ejercicio sólo se mueve `joint_6` (muñeca), típicamente **±0.05 a ±0.08 rad (2.9° a 4.6°)** por turno y en **5 s**. El cliente rechaza estrictamente cualquier articulación que alcance o supere `max_joint_delta_rad = 0.10 rad` (5.7°) respecto a su posición actual; **no** se modifica ese límite.
> 5. **Modo seco obligatorio.** Ningún envío real puede autorizarse sin una ejecución previa en modo seco con código de salida `0`.
> 6. **No limpiar fallas a ciegas.** Ante cualquier error o paro de emergencia, se detiene la sesión y se informa de inmediato al docente.

---

## 7. PROCEDIMIENTO EXPERIMENTAL

### Fase 0: Configuración de Red y ROS_DOMAIN_ID=0 (todos los grupos)

#### 🧠 Concepto
Para que todas las estaciones puedan comunicarse e interactuar con la anfitriona y el robot físico, **todos los grupos deben configurar y compartir exactamente el mismo dominio: `ROS_DOMAIN_ID=0`**.

- **Robot Kinova Gen3:** IP física **`192.168.1.10`** (puerto API Kortex: `10000`).
- **Router del Laboratorio:** TP-Link AX12 (SSID Wi-Fi: `ros2`, subred: `192.168.1.0/24`).
- **Estación Anfitriona:** Cable Ethernet al router (garantiza bucle de control determinista a 1 kHz).
- **Estaciones Monitoras:** Conexión Wi-Fi al SSID `ros2`.

#### 🛠️ Ejercicio 0.1: Conexión a la red y verificación de IP propia
```bash
ip -brief addr
```
Confirme que su interfaz de red activa (`wlan0` o `enp...`) tenga una dirección en el rango `192.168.1.xx/24`.

#### 🛠️ Ejercicio 0.2: Probar conectividad con el robot real (`192.168.1.10`)
```bash
ping -c 4 192.168.1.10
```
Debe obtener `0% packet loss` y tiempos RTT típicos menores a 5 ms.

#### 🛠️ Ejercicio 0.3: Configuración del entorno ROS 2 (`ROS_DOMAIN_ID=0`)
En **todas** las terminales de trabajo:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/ros2_ws/src/burger_delivery/network_setup/cyclonedds.xml"
export CFG=$(ros2 pkg prefix burger_kinova_reference)/share/burger_kinova_reference/config/kinova_connection.yaml
```

#### 🛠️ Ejercicio 0.4: Reinicio limpio del daemon de ROS 2
```bash
timeout 5s ros2 daemon stop
ros2 daemon start
ros2 daemon status
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID | RMW: $RMW_IMPLEMENTATION"
```

#### 🛠️ Ejercicio 0.5: Verificación de ejecutables
```bash
ros2 pkg executables burger_kinova_reference
```
Salida esperada: `kinova_monitor`, `safe_sequence_client`, `safe_trajectory_client`.

---

### Fase 1: Puesta en Marcha de la Estación Anfitriona (grupo anfitrión)

#### 🛠️ Ejercicio 1.1: Comprobar que nadie más tiene el robot
En la estación anfitriona (por cable Ethernet, `ROS_DOMAIN_ID=0`):
```bash
ip -brief addr
ping -c 4 192.168.1.10
ss -tanp | grep 192.168.1.10                  # debe estar vacío
timeout 15s ros2 node list | grep -E "controller_manager|kinova_vision"   # debe estar vacío
```

#### 🛠️ Ejercicio 1.2: Lanzar el driver sin movimiento
**Terminal A1 — Grabación de evidencia:**
```bash
ros2 bag record -s mcap -o sesion_turnos_eqNN \
  --topics /joint_states /burger/kinova/diagnostics /rosout
```

**Terminal A2 — Driver y monitor:**
```bash
ros2 launch burger_kinova_reference kinova_connection.launch.py \
  start_driver:=true robot_ip:=192.168.1.10 use_fake_hardware:=false \
  enable_motion:=false launch_rviz:=true
```

**Terminal A3 — Verificación de telemetría:**
```bash
ss -tanp | grep 192.168.1.10
ros2 topic hz /joint_states
ros2 topic echo /burger/kinova/diagnostics --once | grep -A8 "identidad de la estación"
```
Salida esperada: sesión TCP Kortex en estado `ESTAB`, `/joint_states` a ~99.96 Hz y `rol_estacion: anfitriona`.

---

### Fase 2: Estaciones Monitoras, RQT y RViz (grupos monitores)

#### 🛠️ Ejercicio 2.1: Lanzar el monitor del grupo
En la portátil del grupo monitor (Wi-Fi `ros2`, `ROS_DOMAIN_ID=0`):
```bash
ros2 run burger_kinova_reference kinova_monitor --ros-args --params-file $CFG \
  -r __node:=kinova_monitor_eqNN \
  -p start_driver:=false -p use_fake_hardware:=false -p robot_ip:=192.168.1.10
```

#### 🛠️ Ejercicio 2.2: Auditar el estado del robot y roles anunciados
```bash
ros2 node list | grep kinova_monitor
ros2 topic hz /joint_states
ros2 topic echo /burger/kinova/diagnostics | grep -A8 "identidad de la estación"
```

#### 🛠️ Ejercicio 2.3: Visualizar el robot en RViz sin lanzar driver
```bash
rviz2 -d $(ros2 pkg prefix kortex_description)/share/kortex_description/rviz/view_robot.rviz
```
El modelo aparece gracias a que `robot_state_publisher` en la anfitriona publica en `/robot_description` con durabilidad `TRANSIENT_LOCAL`.

#### 🛠️ Ejercicio 2.4: Inspección gráfica con RQT (Grafo y Logs)
**1. Grafo computacional (`rqt_graph`):**
```bash
rqt_graph
```
Seleccione `Nodes/Topics (all)`. Observe el nodo `/controller_manager`, el tópico `/joint_states` y todos los monitores `kinova_monitor_eqNN` convergiendo en `/burger/kinova/diagnostics`.

**2. Consola de logs (`rqt_console`):**
```bash
ros2 run rqt_console rqt_console
```
Filtre por nodo (`kinova_monitor_eqNN` o `kinova_monitor`) y observe los eventos en tiempo real transmitidos por `/rosout`.

---

### Fase 3: Envío de Trayectorias por Turnos y Prueba Final Integradora

#### 3.1. Fundamentos y Reglas de Seguridad en Envío de Trayectorias
##### ¿Por qué se mueve únicamente `joint_6` en los turnos individuales?
- `joint_6` rota la pinza Robotiq 2F-85 sobre su propio eje longitudinal. No cambia el alcance del brazo, no desplaza el codo ni altera el centro de masa del manipulador.
- $\Delta = \pm 0.05\text{ rad}$ ($2.86^\circ$) produce un arco de $\sim 7.5\text{ mm}$ en las puntas de la pinza.
- Para mayor visibilidad desde las mesas de trabajo de los estudiantes, se puede usar hasta **`±0.08 rad`** ($4.58^\circ$, desplazamiento $\sim 12\text{ mm}$).
- El techo estricto e infranqueable de software es `max_joint_delta_rad = 0.10 rad` ($5.73^\circ$).

##### ¿Qué es el Modo Seco (`dry_run:=true`)?
Consiste en ejecutar toda la validación matemática sobre la telemetría viva de `/joint_states` (límites de carrera, deltas por articulación, tasa de actualización y frescura temporal), pero **inhibiendo por software el envío de la meta al servidor de acción**. El robot físico permanece 100% inmóvil y el cliente retorna en Linux el código de éxito `0` (`echo $?` = 0). Es un requisito mandatorio de seguridad antes de autorizar cualquier movimiento en hardware real.

---

#### 3.2. Descubrimiento Obligatorio del Estado Actual del Robot (Telemetría Viva)

> [!CAUTION]
> **Error típico de seguridad:** Copiar valores numéricos de un ejemplo o guía sin leer el estado real del robot provocará que `safe_trajectory_client` **bloquee la meta inmediatamente** con el mensaje:
> `[SEGURIDAD] Meta bloqueada: joint_N: desplazamiento X.XXXX rad supera max_joint_delta_rad=0.1000 rad`.  
> Esto ocurre porque el cliente exige posiciones articulares absolutas y calcula la diferencia matemática contra la pose física viva del robot en ese instante.

Para descubrir la pose actual exacta del manipulador en tu mesa:

```bash
# Opción 1: Extraer directamente el vector numérico de las 6 articulaciones
ros2 topic echo /joint_states --once --field position

# Opción 2: Ver el desglose completo de nombres y posiciones
ros2 topic echo /joint_states --once
```

Al ejecutar la lectura, obtendrás las 6 posiciones vivas en radianes `[j1, j2, j3, j4, j5, j6]` (por ejemplo: `[-0.1944, -0.5010, -1.9547, -0.0058, -0.6803, -1.5982]`).

**Construcción de la Meta Articular:**
1. Mantén las primeras 5 articulaciones (`joint_1` a `joint_5`) **exactamente idénticas** a las leídas.
2. Modifica **únicamente** la última articulación (`joint_6`), sumando o restando entre $+0.05$ y $+0.08\text{ rad}$.
   - Ejemplo: si `joint_6` actual es `-1.5982`, la meta será `-1.5982 + 0.0600 = -1.5382`.
   - Vector meta resultante: `[-0.1944, -0.5010, -1.9547, -0.0058, -0.6803, -1.5382]`.

---

#### 3.3. Protocolo de un Turno Individual (`safe_trajectory_client`)

1. **Solicitud de Turno:** El grupo solicita turno; el grupo anfitrión lo concede y registra la hora de inicio en la Tabla 4.
2. **Seguridad en Celda:** El anfitrión confirma que el área de operación esté despejada y que un integrante esté posicionado junto a la parada de emergencia física.
3. **Lectura y Formulación:** El grupo descubre la pose viva actual y prepara la meta articular de `joint_6`.
4. **Ensayo Obligatorio en Modo Seco:** El grupo ejecuta el cliente con `dry_run:=true`:
   ```bash
   ros2 run burger_kinova_reference safe_trajectory_client --ros-args --params-file $CFG \
     -r __node:=safe_trajectory_client_eqNN \
     -p use_fake_hardware:=false -p enable_motion:=true -p dry_run:=true \
     -p "safe_joint_positions_rad:=[j1_actual, j2_actual, j3_actual, j4_actual, j5_actual, j6_meta]"
   echo $?
   ```
   Compruebe en la consola que todas las filas reporten `✓ OK`, que el delta de `joint_6` esté entre 0.05 y 0.08 rad y que `echo $?` retorne **`0`**. Muestre el reporte al anfitrión.
5. **Autorización Verbal:** El anfitrión autoriza en voz alta el envío físico.
6. **Envío al Hardware Real:**
   ```bash
   ros2 run burger_kinova_reference safe_trajectory_client --ros-args --params-file $CFG \
     -r __node:=safe_trajectory_client_eqNN \
     -p use_fake_hardware:=false -p enable_motion:=true -p dry_run:=false \
     -p "safe_joint_positions_rad:=[j1_actual, j2_actual, j3_actual, j4_actual, j5_actual, j6_meta]"
   ```
   Escriba `"si"` ante la confirmación interactiva:
   ```text
   ¿Enviar meta al servidor de acción? [si/no]: si
   ```
   Observe el movimiento físico de la pinza y el cierre con código `SUCCESSFUL`.
7. **Verificación y Liberación:** Todos los grupos verifican la nueva pose en RViz y el anfitrión asienta el cierre del turno en la Tabla 4.

---

#### 3.4. Prueba Final Integradora: Secuencia Autónoma con Autodescubrimiento de Pose (`safe_sequence_client`)

Como prueba final integradora de la práctica, se dispone del nodo autónomo `safe_sequence_client`.

##### ¿En qué se diferencia de la prueba individual?
A diferencia de `safe_trajectory_client` (que exige descubrir y transcribir coordenadas absolutas a mano), `safe_sequence_client` implementa el **autodescubrimiento dinámico del punto de origen**:
- Al arrancar, se suscribe a `/joint_states` y audita la frecuencia y frescura del enlace durante 2 segundos.
- Captura la pose en la que se encuentre el robot en ese instante y la fija como vector `origen` de referencia.
- Ejecuta una coreografía articular fluida de **25 tramos relativos** preaprobados en `kinova_connection.yaml` (`sequence_deltas_rad`), coordinando base (`joint_1`), hombro (`joint_2`) y muñeca (`joint_6`).
- Al terminar el ciclo, regresa automáticamente a la pose de origen exacta (`return_to_origin:=true`).

> [!WARNING]
> **Protocolo de Seguridad para la Secuencia Autónoma:**  
> Debido a que la secuencia autónoma desplaza articulaciones mayores (base y hombro hasta $\pm 31^\circ$), es **estrictamente obligatorio**:
> 1. Despejar un radio perimetral mínimo de **1.2 metros** alrededor de la base del robot.
> 2. Mantener un operador con la **mano directamente sobre la parada de emergencia física**.

##### Procedimiento de la Prueba Final:

**Paso A: Ensayo de la Secuencia en Modo Seco**
```bash
ros2 run burger_kinova_reference safe_sequence_client --ros-args \
  --params-file $CFG \
  -r __node:=safe_sequence_client_eqNN \
  -p dry_run:=true
```
Compruebe en la terminal que se capture la pose de origen a 100 Hz, que valide los 25 tramos punto por punto y finalice con éxito (`EXIT_OK = 0`).

**Paso B: Ejecución Real de la Secuencia en el Robot Físico**
Con el área despejada y parada de emergencia lista, ejecute:
```bash
ros2 run burger_kinova_reference safe_sequence_client --ros-args \
  --params-file $CFG \
  -r __node:=safe_sequence_client_eqNN \
  -p dry_run:=false -p enable_motion:=true
```
Observe en el robot real y en RViz el ciclo continuo de movimientos de hombro y rotación de muñeca, finalizando con el retorno suave a la pose original. Registre el evento en la Tabla 4.

---

### Fase 4: Cierre Ordenado y Observación de la Pérdida de Enlace

#### 🛠️ Ejercicio 4.1: Cierre desde la anfitriona
Con los monitores de los grupos en marcha, el grupo anfitrión detiene el driver con `Ctrl+C` en la Terminal A2 y comprueba:
```bash
ss -tanp | grep 192.168.1.10
```
La sesión Kortex pasa a `TIME-WAIT` y desaparece limpiamente. Luego detiene la grabación en la Terminal A1 (`Ctrl+C`) y verifica el bag:
```bash
ros2 bag info sesion_turnos_eqNN
```

#### 🛠️ Ejercicio 4.2: La pérdida vista desde las monitoras
Cada grupo monitor observa en la consola de su `kinova_monitor_eqNN` la transición:
```text
[TRANSICIÓN] OK -> ERROR | telemetría vencida: ... s sin mensaje válido (límite 1.00 s)
```
Anote el tiempo de reacción en la Tabla 3 y detenga su monitor con `Ctrl+C`.

---

## 8. REGISTRO DEL LABORATORIO

### Tabla 1: Verificación de Entorno y Red (Fase 0, todos los grupos)
| Verificación | Comando ejecutado | Resultado esperado | Resultado obtenido |
|---|---|---|:---:|
| IP propia en subred `192.168.1.0/24` | `ip -brief addr` | IP asignada en `192.168.1.xx` | |
| Ping al robot Kinova (`192.168.1.10`) | `ping -c 4 192.168.1.10` | 0% packet loss, RTT < 5 ms | |
| Dominio común ROS 2 | `echo $ROS_DOMAIN_ID` | `0` | |
| Middleware DDS optimizado | `echo $RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | |
| Estado del Daemon de ROS 2 | `ros2 daemon status` | `The daemon is running` | |
| Ejecutables de referencia disponibles | `ros2 pkg executables burger_kinova_reference` | 3 ejecutables listados | |

### Tabla 2: Verificación de la Estación Anfitriona (Fase 1)
| Verificación | Comando | Resultado |
|---|---|---|
| Nadie tenía el robot antes | `ss -tanp`, `ros2 node list` | |
| Sesión Kortex establecida | `ss -tanp \| grep 192.168.1.10` | |
| Identidad anunciada | `rol_estacion` / `rol_verificado` | |
| Frecuencia `/joint_states` (cable) | `ros2 topic hz` | |
| Controladores activos | `ros2 control list_controllers` | |

### Tabla 3: Estaciones Monitoras (Fases 2 y 4, una fila por grupo)
| Grupo | Nodo | IP | `rol_estacion` | Frecuencia `/joint_states` (WiFi) | Estado inicial | RViz / `rqt_graph` OK | Logs en `rqt_console` | Tiempo hasta `OK -> ERROR` |
|:---:|---|---|---|:---:|---|:---:|:---:|:---:|
| | `kinova_monitor_eq__` | | | | | | | |
| | `kinova_monitor_eq__` | | | | | | | |
| | `kinova_monitor_eq__` | | | | | | | |

### Tabla 4: Registro de Turnos y Prueba Final (Fase 3)
| Turno / Prueba | Grupo | Inicio | Pose / `joint_6` inicial | Meta solicitada | Modo seco (código) | Envío (código / `error_code`) | Pose final (`joint_6`) | Cierre |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 1 | | | | (+0.05 a +0.08) | | | | |
| 2 | | | | (-0.05 a -0.08) | | | | |
| 3 | | | | (+0.05 a +0.08) | | | | |
| 4 | | | | (-0.05 a -0.08) | | | | |
| **Prueba Final (Secuencia)** | | | Origen autodescubierto | Coreografía 25 deltas | Código 0 | SUCCESSFUL | Retorno a origen OK | |

### Tabla 5: Incidentes y Diagnóstico
| Momento | Síntoma observado | Capa (red / DDS / driver / validación / protocolo) | Verificación realizada | Acción |
|---|---|---|---|---|
| | | | | |

---

## 9. PREGUNTAS DE ANÁLISIS

1. **Pregunta 1 (Unicidad del driver):** Con la Tabla 2 y la identidad publicada, argumente cómo la estación anfitriona única y el dominio compartido permiten responder *"¿quién tiene el robot?"* desde cualquier estación.
2. **Pregunta 2 (Emisores simultáneos y unicidad de acción):** En ROS 2 y `ros2_control`, el servidor de acción del controlador (`/joint_trajectory_controller/follow_joint_trajectory`) acepta metas de cualquier nodo que opere en el dominio `0`. Si dos estaciones envían una meta de trayectoria simultáneamente, ¿qué le ocurre a la primera meta y por qué? ¿Por qué el protocolo de turnos es indispensable cuando todos comparten el `ROS_DOMAIN_ID=0`?
3. **Pregunta 3 (Enlace por cable y por WiFi):** Compare la frecuencia de `/joint_states` en la anfitriona y en las monitoras (Tablas 2 y 3). ¿Por qué la monitora puede ir por WiFi y la anfitriona no?
4. **Pregunta 4 (Trazabilidad):** Con el bag `sesion_turnos_eqNN` y la Tabla 4, reconstruya la cronología de un turno: qué nodo (por su nombre `_eqNN`) envió, cuándo se aceptó la meta y cuándo terminó.
5. **Pregunta 5 (Garantía por software):** El monitor publica *"habilitación de movimiento"*, pero el cliente no lo consulta antes de enviar. Proponga un diseño en el que el turno quede **garantizado por software** (por ejemplo, un servicio de concesión de turno en la anfitriona). ¿Qué nuevas fallas introduciría?
6. **Pregunta 6 (Pérdida de enlace durante el movimiento):** Si durante un turno se cae el WiFi de la estación que envió la meta, ¿se detiene el robot? Razone con la arquitectura: dónde vive el controlador y dónde vive el cliente de acción.
7. **Pregunta 7 (Aislamiento vs. Colaboración en DDS):** ¿Qué ocurriría durante este laboratorio si un grupo deja accidentalmente su `ROS_DOMAIN_ID` en un valor distinto de `0` (por ejemplo `10`)? ¿Podría ver la telemetría del robot o participar en los turnos? ¿Por qué es fundamental que todas las estaciones acuerden exactamente el mismo `ROS_DOMAIN_ID=0`?
8. **Pregunta 8 (Posicionamiento Absoluto vs. Deltas Relativos y Autodescubrimiento):** Compare la operación de `safe_trajectory_client` frente a `safe_sequence_client`. ¿Por qué en el cliente de trayectoria individual fue estrictamente necesario descubrir las posiciones absolutas reales de `/joint_states` antes de formular la meta para evitar el bloqueo por `max_joint_delta_rad`, mientras que el cliente de secuencia pudo ejecutarse desde cualquier pose sin transcribir coordenadas a mano? ¿Qué riesgos y ventajas de seguridad introduce cada enfoque en entornos industriales colaborativos?

---

## 10. REFERENCIAS

1. Kinova Robotics. (2024). *Kinova Gen3 Ultra lightweight robot User Guide.* Kinova Inc.
2. ros2_control. (2024). *joint_trajectory_controller — Documentation.* https://control.ros.org/
3. ROS 2 Documentation. (2024). *Understanding actions.* https://docs.ros.org/en/jazzy/
4. Proyecto `burger_delivery`. [`burger_kinova_reference/README.md`](../../burger_kinova_reference/README.md), [`VALIDACION_CORTE_1.md`](../../burger_kinova_reference/docs/VALIDACION_CORTE_1.md) y [`TROUBLESHOOTING.md`](../../TROUBLESHOOTING.md) §2.
