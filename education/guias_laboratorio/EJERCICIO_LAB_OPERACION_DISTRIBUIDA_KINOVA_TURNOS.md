# EJERCICIO DE LABORATORIO: OPERACIÓN DISTRIBUIDA DEL KINOVA GEN3 — ESTACIÓN ANFITRIONA, MONITORES Y ENVÍO DE TRAYECTORIAS POR TURNOS

> [!NOTE]
> **Ejercicio práctico sin calificación.** No forma parte de la evaluación del curso ni de las
> evidencias ABET. Su propósito es que los grupos practiquen la operación compartida del robot
> real antes de usarlo en sus proyectos.
>
> Asignatura: ROBOT OPERATING SYSTEM - ROS · Ingeniería Mecatrónica · Versión 1.0 (16/09/2026).
> Los comandos de configuración de red, nodos de diagnóstico y visualización en RViz se validaron
> sobre ROS 2 Jazzy; los datos del robot real provienen de
> [`VALIDACION_CORTE_1.md`](../../burger_kinova_reference/docs/VALIDACION_CORTE_1.md).

---

## 1. INTRODUCCIÓN

### 1.1. Un robot, muchas estaciones

El laboratorio tiene **un** Kinova Gen3 de 6 GDL con pinza Robotiq 2F-85 y varios grupos de
trabajo. El robot admite **una sola sesión de control en tiempo real** (API Kortex, puerto
`10000`): si dos computadores lanzan el driver a la vez, compiten por esa sesión y provocan
desconexiones y paradas de seguridad. En el Laboratorio 02 ocurrió algo equivalente con la
cámara, y de ahí nació la **convención de estación anfitriona** que este ejercicio pone en
funcionamiento ([`TROUBLESHOOTING.md`](../../TROUBLESHOOTING.md) §2.0):

- **Todas** las estaciones trabajan en el mismo dominio: `ROS_DOMAIN_ID=0`.
- **Una sola** estación, la **anfitriona**, conectada por **cable Ethernet**, ejecuta el driver.
- Las demás son **monitoras**: observan el robot por DDS, sin driver propio.
- Mover el robot es un **turno**: sólo un grupo a la vez envía una trayectoria.

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

### 1.2. El package del ejercicio

[`burger_kinova_reference`](../../burger_kinova_reference/README.md) es la implementación de
referencia del corte 1. No reimplementa el driver: trabaja sobre las interfaces ROS 2 que
publica `ros2_kortex`.

| Ejecutable | Qué hace | Quién lo usa en este ejercicio |
|---|---|---|
| `kinova_connection.launch.py` | Lanza el driver (`start_driver:=true`) y el monitor | Grupo anfitrión |
| `kinova_monitor` | Valida `/joint_states` y los controladores, y publica `/burger/kinova/diagnostics`, incluida la **identidad de la estación** | Todos los grupos |
| `safe_trajectory_client` | Envía **una** meta articular tras validar límites, delta máximo, que la telemetría esté vigente (no vencida) y habilitación explícita | El grupo con el turno |

### 1.3. Lo que ninguna capa de software impide

El servidor de acción del controlador **acepta metas de cualquier estación del dominio**. Si
dos grupos envían a la vez, la segunda meta **cancela** la primera:

```text
[grupo A] [RESULTADO] Meta ACEPTADA por el controlador.
[grupo A] [RESULTADO] Meta CANCELADA | error_code=SUCCESSFUL | error_string=""     ← código de salida 3
[grupo B] [RESULTADO] Meta ACEPTADA por el controlador.
[grupo B] [RESULTADO] Trayectoria completada con éxito | error_code=SUCCESSFUL
```

Tampoco lo impide el monitor: su estado *"habilitación de movimiento"* es **informativo** y
`safe_trajectory_client` no lo consulta. **El turno es un protocolo entre personas**, y este
ejercicio lo entrena.

---

## 2. OBJETIVOS

### 2.1. Objetivo General
Operar el Kinova Gen3 real desde varias estaciones ROS 2 bajo la convención de estación
anfitriona, verificando por evidencia quién tiene el robot, monitoreando la salud del enlace
desde estaciones remotas y enviando trayectorias articulares supervisadas mediante un
protocolo de turnos.

### 2.2. Objetivos Específicos
1. **Configurar el entorno de red y dominio común** (`ROS_DOMAIN_ID=0`, CycloneDDS y conectividad con la IP real del Kinova `192.168.1.10`) en todos los equipos del laboratorio, asegurando la interoperabilidad distribuida.
2. **Poner en marcha la estación anfitriona** con el driver del robot real, verificando la sesión Kortex, los controladores y la frecuencia de la telemetría.
3. **Desplegar estaciones monitoras** con identidad propia en el grafo, comprobando desde cada una quién es la anfitriona y analizando el grafo distribuido y los logs con herramientas RQT (`rqt_graph` y `rqt_console`).
4. **Ejecutar el protocolo de turnos**: modo seco, autorización, envío supervisado y liberación del turno, con registro trazable de cada movimiento.
5. **Cerrar la sesión de forma ordenada** y observar desde las monitoras la pérdida de telemetría.

---

## 3. DESCRIPCIÓN DEL EJERCICIO

```text
  +---------------------------------------------------------------------------------------+
  |  FASE 0: CONFIGURACIÓN DE RED Y ROS_DOMAIN_ID=0 (todos los grupos, IP: 192.168.1.10)  |
  |                                        ↓                                              |
  |  FASE 1: PUESTA EN MARCHA DE LA ESTACIÓN ANFITRIONA (grupo anfitrión, robot real)     |
  |                                        ↓                                              |
  |  FASE 2: ESTACIONES MONITORAS (grupos monitores)                                      |
  |                                        ↓                                              |
  |  FASE 3: ENVÍO DE TRAYECTORIAS POR TURNOS (un grupo a la vez)                         |
  |                                        ↓                                              |
  |  FASE 4: CIERRE ORDENADO Y OBSERVACIÓN DE LA PÉRDIDA DE ENLACE                        |
  +---------------------------------------------------------------------------------------+
```

**Roles de la sesión.** El docente asigna un **grupo anfitrión** por sesión; el resto son
**grupos monitores**. En la Fase 3 **todos** los grupos, incluido el anfitrión, reciben al
menos un turno de envío. En sesiones posteriores el rol de anfitrión rota.

| Rol | Estación | Responsabilidades |
|---|---|---|
| **Grupo anfitrión** | PC fija conectada por **Ethernet** | Lanza y detiene el driver; custodia la **parada de emergencia**; concede y cierra los turnos; graba la evidencia de la sesión |
| **Grupo monitor** | Portátil del grupo, por WiFi | Monitorea el enlace con un nodo propio; solicita turno; en su turno, envía y documenta una trayectoria |

**Nomenclatura obligatoria.** Cada grupo usa su número de dos dígitos `NN` en los nombres de
sus nodos: `kinova_monitor_eqNN` y `safe_trajectory_client_eqNN`. Así cada línea de `/rosout`
queda asociada a un grupo.

---

## 4. MATERIALES Y EQUIPOS

### 4.1. Equipos del Laboratorio
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Brazo manipulador Kinova Gen3 (6-DOF) con pinza Robotiq 2F-85 (**IP real: `192.168.1.10`**, puerto Kortex `10000`) | 1 | Unidad |
| Router TP-Link AX12 (SSID `ros2`, subred `192.168.1.0/24`, Gateway `192.168.1.1`) | 1 | Unidad |
| Estación anfitriona con Ubuntu 24.04, ROS 2 Jazzy y **cable Ethernet** al router (IP en `192.168.1.0/24`) | 1 | Unidad |
| Pulsador de parada de emergencia física, accesible desde la estación anfitriona | 1 | Unidad |

### 4.2. Equipos del Estudiante (por grupo)
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Portátil con Ubuntu 24.04 LTS y ROS 2 Jazzy conectado por **WiFi** al SSID `ros2` (o por cable) | 1 | Unidad |
| Workspace `~/ros2_ws` con `burger_delivery` compilado | 1 | Workspace |
| Hoja de registro de turnos (Tabla 4) | 1 | Documento |

---

## 5. SEGURIDAD EN EL LABORATORIO

> [!WARNING]
> 1. **Un solo driver.** Ninguna estación distinta de la anfitriona lanza `kortex_bringup` ni `start_driver:=true` con el robot real.
> 2. **Parada de emergencia.** Un integrante del grupo anfitrión permanece junto al pulsador durante **todo** turno de envío.
> 3. **Área de barrido.** Radio libre de 1.2 m alrededor del robot antes de cada envío. Nadie dentro del área mientras hay un turno activo.
> 4. **Movimiento seguro y acotado.** En este ejercicio sólo se mueve `joint_6` (muñeca), típicamente **±0.05 a ±0.08 rad (2.9° a 4.6°)** por turno y en **5 s**. El cliente rechaza estrictamente cualquier articulación que alcance o supere `max_joint_delta_rad = 0.10` rad (5.7°) respecto a su posición actual; **no** se modifica ese límite.
> 5. **Modo seco primero.** Ningún envío real sin un modo seco previo con código de salida `0`.
> 6. **No se limpian fallas para continuar.** Ante un error, se detiene la sesión y se avisa al docente.

---

## 6. PROCEDIMIENTO EXPERIMENTAL

### Fase 0: Configuración de Red y ROS_DOMAIN_ID=0 (todos los grupos)

#### 🧠 Concepto
Para que todas las estaciones del laboratorio puedan descubrirse entre sí por DDS y comunicarse
con el robot real a través de la estación anfitriona, **todos los grupos deben configurar y
compartir exactamente el mismo dominio: `ROS_DOMAIN_ID=0`**.

Si una estación usa un `ROS_DOMAIN_ID` distinto (o un middleware DDS incompatible), quedará en un
universo DDS aislado: no verá la telemetría del robot (`/joint_states`), no podrá inspeccionar los
diagnósticos (`/burger/kinova/diagnostics`) ni podrá enviar metas de trayectoria en su turno.

**Topología de red e IP real del robot:**
- **Robot Kinova Gen3 (6 GDL):** Su dirección IP física configurada en el laboratorio es **`192.168.1.10`** (puerto API Kortex: `10000`).
- **Router del Laboratorio:** TP-Link AX12 (SSID Wi-Fi: `ros2`, subred: `192.168.1.0/24`, Gateway: `192.168.1.1`).
- **Estación Anfitriona:** Se conecta por **cable Ethernet** al router para garantizar latencia determinista y evitar pérdidas en el bucle de control a 1 kHz.
- **Estaciones Monitoras (estudiantes):** Se conectan por **Wi-Fi** a la red `ros2` (o por cable de red si hay puestos disponibles).

```text
                  Router TP-Link AX12 (Subred 192.168.1.0/24)
                               │
       ┌───────────────────────┼───────────────────────┐
       │ (Cable Ethernet)      │ (Cable Ethernet)      │ (Wi-Fi SSID: ros2)
[ Kinova Gen3 ]         [ PC Anfitriona ]       [ Portátiles Monitores ]
 IP: 192.168.1.10        IP: 192.168.1.XX        IP: 192.168.1.YY
 Puerto: 10000           ROS_DOMAIN_ID=0         ROS_DOMAIN_ID=0
```

---

#### 🛠️ Ejercicio 0.1: Conexión a la red del laboratorio y verificación de IP propia

Conéctese a la red del laboratorio (cable Ethernet si es anfitrión, o Wi-Fi `ros2` si es monitor)
y compruebe la dirección IP asignada a su tarjeta de red:

```bash
ip -brief addr
```

**Salida esperada:** Localice su interfaz activa (`wlan0`, `enp...` o `eth...`) y confirme que
tenga una dirección en la subred del robot:
```text
wlan0            UP             192.168.1.45/24 fe80::...
```

---

#### 🛠️ Ejercicio 0.2: Probar conectividad con el robot Kinova real (`192.168.1.10`)

Antes de iniciar cualquier software de ROS 2, verifique que su computador puede comunicarse
directamente con el robot manipulador en su IP real:

```bash
ping -c 4 192.168.1.10
```

**Salida esperada:**
```text
4 packets transmitted, 4 received, 0% packet loss, time 3004ms
rtt min/avg/max/mdev = 1.842/2.510/3.120/0.450 ms
```

> [!WARNING]
> Si obtiene `Destination Host Unreachable` o `100% packet loss`:
> 1. Verifique que su Wi-Fi esté conectado a la red `ros2` y **no** a otra red (como eduroam o datos móviles).
> 2. Verifique que el Kinova esté encendido (anillo LED de la base en color verde o azul continuo).
> 3. No continúe a las siguientes fases hasta que el comando `ping` responda con `0% packet loss`.

---

#### 🛠️ Ejercicio 0.3: Configuración del entorno ROS 2 (`ROS_DOMAIN_ID=0`)

En **todas** las terminales que abra durante el laboratorio, ejecute este bloque para configurar el
dominio común y el middleware optimizado:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/ros2_ws/src/burger_delivery/network_setup/cyclonedds.xml"
export CFG=$(ros2 pkg prefix burger_kinova_reference)/share/burger_kinova_reference/config/kinova_connection.yaml
```

**¿Qué hace cada una de estas instrucciones?**
- `source ...`: Carga el entorno base de ROS 2 Jazzy y los paquetes de su workspace.
- `export ROS_DOMAIN_ID=0`: **Regla crítica del laboratorio.** Establece el dominio 0 para que todas las estaciones compartan el mismo grafo ROS 2.
- `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`: Selecciona CycloneDDS como capa de transporte.
- `export CYCLONEDDS_URI=...`: Aplica la configuración de red del proyecto para evitar la fragmentación de datagramas UDP en la red inalámbrica.
- `export CFG=...`: Define un atajo a la ruta del archivo de configuración YAML con los parámetros de seguridad.

---

#### 🛠️ Ejercicio 0.4: Reinicio limpio del daemon de ROS 2

El daemon de introspección de ROS 2 guarda en memoria el dominio anterior. Para asegurar que la CLI
opere limpiamente sobre el dominio `0`, reinicie el daemon y compruebe su estado:

```bash
timeout 5s ros2 daemon stop
ros2 daemon start
ros2 daemon status
```

**Salida esperada:**
```text
The daemon is running
```

Compruebe que las variables de entorno quedaron correctamente exportadas:
```bash
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID | RMW: $RMW_IMPLEMENTATION"
```
Salida esperada: `ROS_DOMAIN_ID: 0 | RMW: rmw_cyclonedds_cpp`.

---

#### 🛠️ Ejercicio 0.5: Verificación de compilación del paquete de referencia

Compruebe que los ejecutables del laboratorio están listos para usarse:

```bash
ros2 pkg executables burger_kinova_reference
```

**Salida esperada:**
```text
burger_kinova_reference kinova_monitor
burger_kinova_reference safe_sequence_client
burger_kinova_reference safe_trajectory_client
```

Si no aparecen, o si aún no los ha compilado en su máquina:
```bash
python3 ~/ros2_ws/src/burger_delivery/scripts/aplicar_compatibilidad_kortex.py --check
cd ~/ros2_ws && colcon build --packages-select burger_kinova_reference --symlink-install
source ~/ros2_ws/install/setup.bash
```

#### ✅ Criterios de éxito de la Fase 0
- Su estación cuenta con una IP asignada en el segmento `192.168.1.0/24`.
- El comando `ping -c 4 192.168.1.10` responde con `0% packet loss`.
- El comando `echo $ROS_DOMAIN_ID` devuelve `0` en todas sus terminales.
- El daemon de ROS 2 está activo (`The daemon is running`) y utiliza `rmw_cyclonedds_cpp`.
- Los tres ejecutables de `burger_kinova_reference` están compilados y listos en el sistema.

---

### Fase 1: Puesta en Marcha de la Estación Anfitriona (grupo anfitrión)

#### 🧠 Concepto
La anfitriona es la **única** estación con sesión Kortex. Su monitor lo **verifica localmente**:
comprueba en `/proc/net/tcp` si este computador tiene una conexión establecida con
`192.168.1.10:10000` y lo anuncia en el diagnóstico. Ninguna otra estación puede verificarlo
por ella.

#### 🛠️ Ejercicio 1.1: Comprobar que nadie más tiene el robot

En la estación anfitriona, con el entorno del curso (`ROS_DOMAIN_ID=0`) cargado y el daemon
reiniciado:

```bash
ip -brief addr                                # la NIC por CABLE debe estar en 192.168.1.0/24
ping -c 4 192.168.1.10
ss -tanp | grep 192.168.1.10                  # debe estar vacío
timeout 15s ros2 node list | grep -E "controller_manager|kinova_vision"   # debe estar vacío
```

Si alguna de las dos últimas devuelve algo, **no lance el driver**: ya hay una anfitriona. Siga
[`TROUBLESHOOTING.md`](../../TROUBLESHOOTING.md) §2.2.

#### 🛠️ Ejercicio 1.2: Lanzar el driver sin movimiento

**Terminal A1 — evidencia de la sesión** (antes del driver, para no perder el arranque):
```bash
ros2 bag record -s mcap -o sesion_turnos_eqNN \
  --topics /joint_states /burger/kinova/diagnostics /rosout
```

**Terminal A2 — driver y monitor:**
```bash
ros2 launch burger_kinova_reference kinova_connection.launch.py \
  start_driver:=true robot_ip:=192.168.1.10 use_fake_hardware:=false \
  enable_motion:=false launch_rviz:=true
```

Salida esperada:
```text
[KortexMultiInterfaceHardware]: Actuator count reported by robot is '6'
[kinova_monitor]: [CONTROLADORES] joint_state_broadcaster=active, joint_trajectory_controller=active
[kinova_monitor]: [TRANSICIÓN] … -> OK | telemetría saludable: 100.0 Hz, … 6/6 articulaciones
```

**Terminal A3 — verificación:**
```bash
ss -tanp | grep 192.168.1.10
ros2 topic hz /joint_states
ros2 topic echo /burger/kinova/diagnostics --once | grep -A8 "identidad de la estación"
```

Referencia medida sobre este robot por cable: `/joint_states` a **99.96 Hz**, y en la identidad:
```text
ESTAB  192.168.1.xx:57423  192.168.1.10:10000  users:(("ros2_control_no",pid=…))
message: <HOSTNAME> (192.168.1.xx) — anfitriona
  rol_estacion    : anfitriona
  rol_verificado  : si
  rol_evidencia   : sesión TCP establecida con 192.168.1.10:10000
```

#### ✅ Criterios de éxito
- La persona participante puede demostrar que no había otra anfitriona antes de lanzar el driver.
- La persona participante puede validar la sesión Kortex con `ss` y relacionarla con `rol_estacion: anfitriona`.
- La persona participante puede justificar por qué la anfitriona va por cable (por WiFi, medido: 132 desbordamientos y 2 pérdidas de telemetría en 120 s; [`EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md`](../../burger_kinova_reference/docs/EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md)).

---

### Fase 2: Estaciones Monitoras (grupos monitores)

#### 🧠 Concepto
Una monitora **consume** la telemetría y el diagnóstico por DDS, y publica **su propio**
diagnóstico. Su monitor no se conecta al robot: `robot_ip` sólo le sirve para comprobar que
**este** computador **no** tiene sesión con el robot, y anunciarse como `cliente`.

#### 🛠️ Ejercicio 2.1: Lanzar el monitor del grupo

Con el entorno del curso (`ROS_DOMAIN_ID=0`, CycloneDDS, perfil del proyecto), la variable
`CFG` del Ejercicio 0.2 y el daemon reiniciado:

```bash
ros2 run burger_kinova_reference kinova_monitor --ros-args --params-file $CFG \
  -r __node:=kinova_monitor_eqNN \
  -p start_driver:=false -p use_fake_hardware:=false -p robot_ip:=192.168.1.10
```

Salida esperada: `[TRANSICIÓN] INICIO -> ERROR` durante el primer segundo (aún no ha llegado
telemetría ni se han consultado los controladores) y, enseguida, `ERROR -> OK`.

#### 🛠️ Ejercicio 2.2: ¿Quién tiene el robot y en qué estado está?

```bash
ros2 node list | grep kinova_monitor                      # un monitor por grupo, nombres distintos
ros2 topic info /burger/kinova/diagnostics                # Publisher count = número de monitores
ros2 topic hz /joint_states                               # frecuencia recibida por WiFi
ros2 topic echo /burger/kinova/diagnostics | grep -A8 "identidad de la estación"
```

En el flujo de diagnósticos aparecen **todas** las estaciones: debe haber exactamente **una**
`anfitriona` verificada, y la propia como `cliente`:
```text
message: <HOSTNAME_ANFITRIONA> (192.168.1.xx) — anfitriona
message: <HOSTNAME_PROPIO> (192.168.1.yy) — cliente
```

Complete la **Tabla 3** con los datos de su estación.

> [!NOTE]
> Este ejercicio completa la prueba de aceptación **PA-09 (DDS distribuido)**, que en
> [`VALIDACION_CORTE_1.md`](../../burger_kinova_reference/docs/VALIDACION_CORTE_1.md) figura como
> pendiente: driver en una estación, monitor en otra, descubrimiento por la subred y dominio
> compartido.

#### 🛠️ Ejercicio 2.3: Ver el robot en RViz desde la estación del grupo

Cada grupo puede ver en su propio PC el modelo del robot moviéndose, **sin lanzar ningún driver**.
Se usa exactamente la misma configuración de RViz que abre `kortex_bringup` en la anfitriona
(`kortex_control.launch.py` carga `kortex_description/rviz/view_robot.rviz`):

```bash
rviz2 -d $(ros2 pkg prefix kortex_description)/share/kortex_description/rviz/view_robot.rviz
```

**Revisión real obligatoria.** Verifique de dónde sale lo que ve RViz:

```bash
ros2 topic info -v /robot_description | grep -E "Node name|Endpoint type|Durability"
ros2 topic info -v /tf | grep -E "Node name|Endpoint type"
```

- El **modelo** lo publica el `robot_state_publisher` de la **anfitriona** en `/robot_description`: es el `gen3` de 6 GDL que genera `kortex_description` con `dof:=6`, el mismo que usa el driver.
- Se publica **una sola vez**, con durabilidad `TRANSIENT_LOCAL`, y RViz se suscribe también con `TRANSIENT_LOCAL`. Por eso el modelo aparece **aunque abra RViz después** de que la anfitriona arrancó.
- La **pose** llega por `/tf` y `/tf_static`, que publica ese mismo `robot_state_publisher` a partir de `/joint_states`. Con RViz abierto aparece un suscriptor nuevo a `/tf` llamado `transform_listener_impl_…`: es RViz.
- El `Fixed Frame` de esa configuración es `base_link`.

Resultado esperado: el Kinova en la pose actual del robot real. Durante los turnos de la Fase 3,
el grupo ve moverse la muñeca (`joint_6`) en su propia pantalla. En la validación de referencia, un
movimiento de `joint_6` de `0.00` a `0.10` rad cambió la orientación de `end_effector_link`
respecto a `base_link` de −180.00° a 174.27°, es decir 0.10 rad.

> [!WARNING]
> **No use otra vía para visualizar el robot en una estación monitora:**
> - `kinova_connection.launch.py … launch_rviz:=true` o `kortex_bringup` **lanzan el driver**: sólo la anfitriona los ejecuta.
> - `burger_description/display.launch.py` muestra **otro modelo** (el vendorizado de 7 GDL de la escena) y publica su propio `/joint_states` y `/robot_description` **en el dominio del robot**, mezclándose con la telemetría real ([`TALLER_URDF_TF.md`](../talleres/TALLER_URDF_TF.md)).

#### 🛠️ Ejercicio 2.4: Inspección gráfica con RQT — Grafo distribuido y Logs en tiempo real

RQT es la suite oficial de herramientas gráficas de ROS 2. En una sesión multi-estación sobre
`ROS_DOMAIN_ID=0`, RQT permite verificar visualmente cómo se descubren los nodos de todos los grupos
y monitorear los mensajes de log globales (`/rosout`).

##### 1. Visualización del Grafo Distribuido (`rqt_graph`)

En una terminal con el entorno configurado (`ROS_DOMAIN_ID=0`), ejecute:

```bash
rqt_graph
```
*(También accesible desde el lanzador maestro ejecutando `rqt` y seleccionando en el menú superior: `Plugins` → `Introspection` → `Node Graph`).*

**Configuración recomendada de la vista en `rqt_graph`:**
1. En el selector desplegable superior izquierdo, cambie de `Nodes only` a **`Nodes/Topics (all)`** o **`Nodes/Topics (active)`**.
2. Desmarque la casilla `Debug` y desmarque `Leaf topics` si desea ver todos los enlaces.
3. Haga clic en el botón de actualización (icono de flechas circulares azules 🔄 en la esquina superior izquierda).

**Qué debe observar en el grafo:**
- **Nodos de la anfitriona:** Localice `/robot_state_publisher` y `/controller_manager`. Verá flechas saliendo hacia tópicos globales como `/joint_states` y `/tf`.
- **Nodos de los grupos monitores:** Observe cómo en el diagrama aparecen representados en rectángulos los nodos `kinova_monitor_eqNN` de cada equipo. Todos apuntan con flechas hacia el tópico central `/burger/kinova/diagnostics`.
- **Acción de trayectoria:** Se visualizan las conexiones hacia `/joint_trajectory_controller/follow_joint_trajectory`.

##### 2. Inspección centralizada de Logs (`rqt_console`)

Todos los nodos de ROS 2 en el dominio publican sus mensajes de logging hacia el tópico distribuido
`/rosout`. Para filtrarlos e inspeccionarlos visualmente:

```bash
ros2 run rqt_console rqt_console
```
*(O desde `rqt`: menú `Plugins` → `Logging` → `Console`).*

**Qué debe observar y cómo filtrar:**
- **Filtro por nodo:** En la sección de filtros (icono `+` verde o barra inferior), agregue una regla con `Node` = `kinova_monitor_eqNN` para aislar los mensajes generados por la estación de su grupo, o `Node` = `kinova_monitor` para ver los de la anfitriona.
- **Filtro por severidad:** Puede filtrar por nivel `INFO`, `WARN` o `ERROR`.
- **Mensajes clave observables:**
  - Los mensajes `[TRANSICIÓN] ERROR -> OK` anunciando telemetría saludable y frecuencia de recepción.
  - La identidad de la estación anunciada periódicamente (`anfitriona` o `cliente`).
  - Durante la Fase 3, podrá ver los logs del cliente de trayectoria (`safe_trajectory_client_eqNN`) y la aceptación o feedback del controlador de movimiento.

#### ✅ Criterios de éxito
- La persona participante puede visualizar en su PC el modelo básico del robot y explicar por qué aparece aunque RViz se abra después de la anfitriona.
- La persona participante puede identificar la estación anfitriona desde su propia estación, sin preguntar a nadie.
- La persona participante puede inspeccionar el grafo distribuido en `rqt_graph`, ubicando el nodo de su equipo y los de la anfitriona.
- La persona participante puede filtrar e interpretar los logs de diagnóstico en `rqt_console` a través del tópico `/rosout`.
- La persona participante puede explicar por qué hay tantos publicadores en `/burger/kinova/diagnostics` como monitores.
- La persona participante puede comparar la frecuencia de `/joint_states` en la anfitriona (cable) y en su estación (WiFi).

---

### Fase 3: Envío de Trayectorias por Turnos

#### 🧠 Concepto
Un turno convierte una operación peligrosa en una secuencia verificable. El grupo anfitrión
**concede** el turno; el grupo en turno **valida**, **pide autorización**, **envía** y
**libera**. Como el controlador acepta metas de cualquiera en el dominio compartido (§1.3), el protocolo es la
única garantía de que un solo grupo mueve el robot.

##### ¿Por qué se mueve únicamente `joint_6` y qué tan visible es el ángulo?
- **Seguridad geométrica:** `joint_6` es el último eslabón (la muñeca que rota la pinza Robotiq 2F-85 sobre su propio eje longitudinal). Al girar solo esta articulación, **el brazo no cambia su alcance, no altera su centro de masa ni barre volumen hacia los lados**, eliminando cualquier riesgo de colisión contra mesas, computadores o personas.
- **Magnitud del ángulo y visibilidad física:**
  - El valor base sugerido es **`±0.05 rad`** ($\approx 2.86^\circ$). A una distancia radial de $\sim 15\text{ cm}$ hasta las puntas de los dedos de la pinza, este giro produce un arco físico de:
    $$s = r \cdot \Delta\theta \approx 150\text{ mm} \times 0.05\text{ rad} \approx 7.5\text{ mm}$$
    En RViz y en la telemetría numérica (`/joint_states`), este cambio es **100% evidente y cuantificable**. En el robot real, un observador atento ve rotar los dedos de la pinza $\sim 7.5\text{ mm}$.
  - **Margen para mayor visibilidad:** Si el grupo desea un movimiento más conspicuo y evidente a simple vista desde las mesas de trabajo (a 2 metros de distancia), puede utilizar una meta con un $\Delta$ de hasta **`±0.08 rad`** ($\approx 4.58^\circ$, que desplaza las puntas $\sim 12\text{ mm}$, más de un centímetro).
  - **Techo rígido de seguridad:** El parámetro del sistema es `max_joint_delta_rad = 0.10 rad` ($\approx 5.73^\circ$). Si ingresa un delta igual o superior a `0.10 rad` (o si confunde la posición actual), el cliente **rechazará la meta automáticamente** (código de salida 1).

Cada turno mueve sólo `joint_6`, **alternando** el sentido: los turnos impares giran en sentido positivo (`+0.05` a `+0.08` rad) y los pares en sentido negativo (`-0.05` a `-0.08` rad). Así el robot oscila alrededor de su pose inicial y queda tal como se encontró.

#### 🛠️ Protocolo de un turno

| Paso | Quién | Acción | Evidencia |
|---|---|---|---|
| 1 | Grupo en turno | Solicita el turno en voz alta; el anfitrión lo **concede** y lo anota en la Tabla 4 | Hora de inicio |
| 2 | Anfitrión | Confirma área despejada y un integrante **junto a la parada de emergencia** | — |
| 3 | Grupo en turno | Lee la pose actual y construye la meta: **igual a la actual** salvo `joint_6 ± 0.05` (o hasta `±0.08`) | Pose inicial |
| 4 | Grupo en turno | Ejecuta el **modo seco** y muestra el resumen al anfitrión | Código `0` |
| 5 | Anfitrión | **Autoriza** el envío en voz alta | — |
| 6 | Grupo en turno | Envía, escribe `si` en la confirmación y observa el movimiento | Código de salida y `error_code` |
| 7 | Todos | Verifican la pose final desde sus estaciones | Pose final |
| 8 | Grupo en turno | **Libera** el turno; el anfitrión lo cierra en la Tabla 4 | Hora de cierre |

#### 🛠️ Ejercicio 3.1: Leer la pose y construir la meta (paso 3)

```bash
ros2 topic echo /joint_states --once
```

`/joint_states` del robot real incluye `joint_1` … `joint_6` **y** la articulación de la pinza
(`robotiq_85_left_knuckle_joint`), que se ignora. Copie las seis posiciones **por nombre**, no
por orden de aparición. Ejemplo con la pose registrada en la validación del robot:

| Articulación | Actual [rad] | Meta [rad] (turno impar: +0.05 rad) | Meta alternativa [rad] (turno impar: +0.08 rad) |
|---|---:|---:|---:|
| `joint_1` | -3.0294 | -3.0294 | -3.0294 |
| `joint_2` | -0.2658 | -0.2658 | -0.2658 |
| `joint_3` | +1.8683 | +1.8683 | +1.8683 |
| `joint_4` | +0.6188 | +0.6188 | +0.6188 |
| `joint_5` | -0.7071 | -0.7071 | -0.7071 |
| `joint_6` | -2.0610 | **-2.0110** | **-1.9810** |

> [!IMPORTANT]
> **Use las posiciones del momento del turno, con al menos 4 decimales.** La meta es
> **absoluta**: si copia la pose de otro turno, o redondea `joint_1` a `-3.03`, el cliente
> detecta un delta distinto al esperado o mueve articulaciones que no pretendía mover.

#### 🛠️ Ejercicio 3.2: Modo seco — ¿Qué es y cómo funciona? (paso 4)

##### 🔍 ¿Qué es el "Modo Seco" (*Dry Run*)?
El término proviene de la práctica industrial de ingeniería ("ensayo en seco" o simulación en frío):
consiste en **ejecutar todo el pipeline de cómputo, diagnóstico y validación matemática sobre telemetría viva del robot, pero inhibiendo por completo el envío físico de comandos a los motores**.

##### ⚙️ ¿Qué hace internamente `safe_trajectory_client` en Modo Seco (`dry_run:=true`)?
1. **Captura telemetría real:** Se suscribe a `/joint_states` y verifica que el robot esté publicando a frecuencia adecuada (> 20 Hz) y que los datos no estén vencidos (< 1.0 s).
2. **Inspección de límites:** Comprueba que la pose deseada esté dentro de los límites articulares mecánicos (`joint_min` / `joint_max`).
3. **Validación del desplazamiento ($\Delta$):** Calcula $\Delta = |\text{meta} - \text{actual}|$ para cada articulación. Si alguna supera `max_joint_delta_rad` (0.10 rad), la bloquea.
4. **Validación de duración y habilitación:** Exige que la trayectoria dure al menos 5.0 segundos y que `enable_motion:=true` esté presente.
5. **Generación de reporte en pantalla:** Imprime una tabla comparativa con cada articulación, el delta detectado y el resultado (`✓` o `✗`).
6. **Inhibición de hardware (Punto clave):** Al estar en `dry_run:=true`, el nodo **termina inmediatamente aquí**: **NO** se conecta al servidor de acción (`FollowJointTrajectory`), **NO** envía ningún paquete al controlador y el robot permanece completamente inmóvil.
7. **Código de salida en Linux:** Si todo es seguro, el proceso devuelve código `0` (`$? = 0`). Si hay un error de tipeo o un delta excesivo, devuelve código `1` (`EXIT_BLOCKED`).

```bash
ros2 run burger_kinova_reference safe_trajectory_client --ros-args --params-file $CFG \
  -r __node:=safe_trajectory_client_eqNN \
  -p use_fake_hardware:=false -p enable_motion:=true -p dry_run:=true \
  -p "safe_joint_positions_rad:=[-3.0294,-0.2658,1.8683,0.6188,-0.7071,-2.0110]"
echo $?
```

**Salida esperada (reporte de validación):**
```text
[safe_trajectory_client]: Telemetría disponible: 6 articulaciones a 100.0 Hz.
[safe_trajectory_client]: ======================================================================
[safe_trajectory_client]: VALIDACIÓN DE META ARTICULAR (safe_trajectory_client)
[safe_trajectory_client]: ======================================================================
[safe_trajectory_client]: Articulación    Actual [rad]    Meta [rad]     Delta [rad]   Estado
[safe_trajectory_client]: ----------------------------------------------------------------------
[safe_trajectory_client]: joint_1              -3.0294       -3.0294          0.0000     ✓ OK
[safe_trajectory_client]: joint_2              -0.2658       -0.2658          0.0000     ✓ OK
[safe_trajectory_client]: joint_3               1.8683        1.8683          0.0000     ✓ OK
[safe_trajectory_client]: joint_4               0.6188        0.6188          0.0000     ✓ OK
[safe_trajectory_client]: joint_5              -0.7071       -0.7071          0.0000     ✓ OK
[safe_trajectory_client]: joint_6              -2.0610       -2.0110          0.0500     ✓ OK
[safe_trajectory_client]: ----------------------------------------------------------------------
[safe_trajectory_client]: Duración trayectoria : 5.00 s (mínimo permitido: 5.00 s)     ✓ OK
[safe_trajectory_client]: Habilitación física  : enable_motion=True                      ✓ OK
[safe_trajectory_client]: [MODO SECO] dry_run=true: la validación terminó aquí y NO se contactó el servidor de acción.
```

Revise en el resumen que **sólo** `joint_6` tiene `Δ > 0` y que no aparece ningún `✗`.
El comando `echo $?` debe retornar **`0`**. Muestre este reporte al grupo anfitrión como requisito previo para solicitar la autorización de envío.

> [!WARNING]
> `use_fake_hardware:=false` es obligatorio con el robot real: el YAML trae `true` por
> defecto. Compruebe la línea de arranque del cliente antes de seguir:
> `safe_trajectory_client iniciado | … | modo=HARDWARE REAL | dry_run=True | enable_motion=True`.

#### 🛠️ Ejercicio 3.3: Envío supervisado (pasos 5 y 6)

Sólo tras la autorización del anfitrión:

```bash
ros2 run burger_kinova_reference safe_trajectory_client --ros-args --params-file $CFG \
  -r __node:=safe_trajectory_client_eqNN \
  -p use_fake_hardware:=false -p enable_motion:=true -p dry_run:=false \
  -p "safe_joint_positions_rad:=[-3.0294,-0.2658,1.8683,0.6188,-0.7071,-2.0110]"
echo $?
```

La confirmación debe decir **HARDWARE REAL**:
```text
¿Confirmas el envío de la meta al HARDWARE REAL? Escribe "si" para continuar: si
```

Referencia medida sobre este robot con el mismo movimiento (`joint_6` +0.05 rad en 5 s):
```text
[ENVÍO] Enviando meta de 1 punto(s) a /joint_trajectory_controller/follow_joint_trajectory con duración 5.00 s.
[RESULTADO] Meta ACEPTADA por el controlador.
[FEEDBACK] t=2.040 s | error máximo por articulación=0.00034 rad
[RESULTADO] Trayectoria completada con éxito | error_code=SUCCESSFUL
```

#### 🛠️ Ejercicio 3.4: Verificación desde todas las estaciones (paso 7)

```bash
ros2 topic echo /joint_states --once
```

Cada grupo anota en la Tabla 4 la pose final de `joint_6` que ve desde **su** estación.
Si tiene RViz abierto (Ejercicio 2.3), confirme también que el movimiento se vio en su pantalla.
Todas deben coincidir con la meta en menos de `0.001` rad.

#### ✅ Criterios de éxito
- La persona participante puede construir una meta absoluta a partir de la pose actual, cambiando sólo la articulación autorizada.
- La persona participante puede validar en modo seco que el único `Δ` es el previsto y que el modo es `HARDWARE REAL`.
- La persona participante puede ejecutar el protocolo completo sin enviar fuera de su turno.
- La persona participante puede explicar qué protege el cliente (límites, delta, telemetría vigente, habilitación) y qué **no** protege (metas simultáneas de otras estaciones).

---

### Fase 4: Cierre Ordenado y Observación de la Pérdida de Enlace

#### 🧠 Concepto
Detener el driver sin `Ctrl+C`, por ejemplo con `kill -9`, deja la sesión Kortex abierta y el
siguiente arranque falla sin motivo aparente. Además, la pérdida de telemetría es un evento que
**todas** las monitoras deben registrar.

#### 🛠️ Ejercicio 4.1: Cierre desde la anfitriona

Con los monitores de los grupos **todavía en marcha**, el grupo anfitrión detiene el driver con
`Ctrl+C` en la Terminal A2 y comprueba:

```bash
ss -tanp | grep 192.168.1.10      # sin ESTAB: la sesión pasa a TIME-WAIT y desaparece
```

Salida esperada en la anfitriona:
```text
KortexMultiInterfaceHardware successfully deactivated!
```

Después detiene la grabación (`Ctrl+C` en la Terminal A1) y verifica:
```bash
ros2 bag info sesion_turnos_eqNN
```

#### 🛠️ Ejercicio 4.2: La pérdida vista desde las monitoras

Cada grupo monitor observa en la consola de su `kinova_monitor_eqNN` la transición provocada:
```text
[TRANSICIÓN] OK -> ERROR   | telemetría vencida: … s sin mensaje válido (límite 1.00 s)
```

Anote en la Tabla 3 el tiempo entre el `Ctrl+C` de la anfitriona y la transición, y después
detenga su monitor con `Ctrl+C`.

#### ✅ Criterios de éxito
- La persona participante puede verificar que la sesión Kortex se liberó antes de dar por cerrado el ejercicio.
- La persona participante puede relacionar el `joint_state_timeout_s` (1.0 s) con el tiempo observado hasta `OK -> ERROR`.

---

## 7. REGISTRO DEL EJERCICIO

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

### Tabla 4: Registro de Turnos (Fase 3)
| Turno | Grupo | Inicio | `joint_6` inicial | Meta `joint_6` | Modo seco (código) | Envío (código / `error_code`) | `joint_6` final | Cierre |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 1 | | | | (+0.05) | | | | |
| 2 | | | | (-0.05) | | | | |
| 3 | | | | (+0.05) | | | | |
| 4 | | | | (-0.05) | | | | |

### Tabla 5: Incidentes y Diagnóstico
| Momento | Síntoma observado | Capa (red / DDS / driver / validación / protocolo) | Verificación realizada | Acción |
|---|---|---|---|---|
| | | | | |

---

## 8. PREGUNTAS DE ANÁLISIS

1. **Pregunta 1 (Unicidad del driver):** Con la Tabla 2 y la identidad publicada, argumente cómo la estación anfitriona única y el dominio compartido permiten responder *"¿quién tiene el robot?"* desde cualquier estación.
2. **Pregunta 2 (Emisores simultáneos y unicidad de acción):** En ROS 2 y `ros2_control`, el servidor de acción del controlador (`/joint_trajectory_controller/follow_joint_trajectory`) acepta metas de cualquier nodo que opere en el dominio `0`. Si dos estaciones envían una meta de trayectoria simultáneamente, ¿qué le ocurre a la primera meta y por qué? ¿Por qué el protocolo de turnos es indispensable cuando todos comparten el `ROS_DOMAIN_ID=0`?
3. **Pregunta 3 (Enlace por cable y por WiFi):** Compare la frecuencia de `/joint_states` en la anfitriona y en las monitoras (Tablas 2 y 3). ¿Por qué la monitora puede ir por WiFi y la anfitriona no?
4. **Pregunta 4 (Trazabilidad):** Con el bag `sesion_turnos_eqNN` y la Tabla 4, reconstruya la cronología de un turno: qué nodo (por su nombre `_eqNN`) envió, cuándo se aceptó la meta y cuándo terminó.
5. **Pregunta 5:** El monitor publica *"habilitación de movimiento"*, pero el cliente no lo consulta antes de enviar. Proponga un diseño en el que el turno quede **garantizado por software** (por ejemplo, un servicio de concesión de turno en la anfitriona). ¿Qué nuevas fallas introduciría?
6. **Pregunta 6:** Si durante un turno se cae el WiFi de la estación que envió la meta, ¿se detiene el robot? Razone con la arquitectura: dónde vive el controlador y dónde vive el cliente de acción.
7. **Pregunta 7 (Aislamiento vs. Colaboración en DDS):** ¿Qué ocurriría durante este laboratorio si un grupo deja accidentalmente su `ROS_DOMAIN_ID` en un valor distinto de `0` (por ejemplo `10`)? ¿Podría ver la telemetría del robot o participar en los turnos? ¿Por qué es fundamental que todas las estaciones acuerden exactamente el mismo `ROS_DOMAIN_ID=0`?

---

## 9. REFERENCIAS

1. Kinova Robotics. (2024). *Kinova Gen3 Ultra lightweight robot User Guide.* Kinova Inc.
2. ros2_control. (2024). *joint_trajectory_controller — Documentation.* https://control.ros.org/
3. ROS 2 Documentation. (2024). *Understanding actions.* https://docs.ros.org/en/jazzy/
4. Proyecto `burger_delivery`. [`burger_kinova_reference/README.md`](../../burger_kinova_reference/README.md), [`VALIDACION_CORTE_1.md`](../../burger_kinova_reference/docs/VALIDACION_CORTE_1.md) y [`TROUBLESHOOTING.md`](../../TROUBLESHOOTING.md) §2.

