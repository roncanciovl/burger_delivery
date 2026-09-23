# Troubleshooting del entorno ROS 2 del proyecto

Esta guía cubre los fallos que más tiempo consumen en el laboratorio:

| Síntoma | Sección |
| :--- | :--- |
| `ros2 node list`, `topic list` o `param set` se bloquean o terminan en `TimeoutError` | [1. Bloqueo del daemon de la CLI](#1-bloqueo-del-daemon-de-ros-2-en-wsl) |
| El driver no arranca, o hay telemetría duplicada, porque **el robot ya está en uso** | [2. Hardware ocupado: un solo robot, varias estaciones](#2-hardware-ocupado-un-solo-robot-y-varias-estaciones) |
| El robot se mueve a tirones y el log del driver muestra `WRONG_SERVOING_MODE` o `Fault was not recognized ... combination of Control Mode and Active State` | [2.6 Otra estación lanzó un segundo driver](#26-otra-estación-lanzó-un-segundo-driver-y-le-quitó-el-control-al-primero) |
| No sé quién tiene el robot ocupado | [2.0 La configuración recomendada](#20-la-configuración-recomendada-léela-antes-que-el-resto) |
| Mi código parece correcto y aun así el driver no arranca, o publica datos absurdos | [3. Fallos de plataforma ajenos a tu código](#3-fallos-de-plataforma-ajenos-a-tu-código) |
| La cámara del Kinova da `Failed to start stream`, o `kinova_vision` no se detiene con `Ctrl+C` | [3.4 El driver de visión no se detiene limpiamente](#34-el-driver-de-visión-no-se-detiene-limpiamente-y-deja-la-cámara-bloqueada) |
| Un comando de un taller "funciona" (no da error) pero no produce lo que la guía promete | [4. Fallos silenciosos al ejecutar los talleres](#4-fallos-silenciosos-al-ejecutar-los-talleres) |

---

## 1. Bloqueo del daemon de ROS 2 en WSL

Esta sección resuelve un fallo recurrente en WSL/WSL2: comandos de introspección como `ros2 node list`, `ros2 topic list`, `ros2 service list` o `ros2 daemon stop` quedan bloqueados, tardan minutos o terminan con errores como `TimeoutError`, `ConnectionRefusedError` o `RemoteDisconnected`.

### 1.1 Recuperación rápida

Ejecuta estos pasos dentro de WSL. No uses `sudo`: el daemon pertenece al usuario que ejecuta ROS 2.

```bash
# 1. Intento normal, limitado a cinco segundos para no bloquear la terminal
timeout 5s ros2 daemon stop

# 2. Si se agotó el tiempo, localiza exclusivamente el daemon de ROS 2
pgrep -af '(_ros2_daemon|--name ros2-daemon)'

# 3. Sustituye <PID> por el identificador mostrado y solicita un cierre limpio
kill <PID>

# 4. Comprueba que terminó
ps -p <PID> -o pid,stat,cmd
```

Si `ps` todavía muestra el mismo proceso después de unos segundos, fuerza su terminación como último recurso:

```bash
kill -KILL <PID>
```

Luego inicia una instancia nueva con el entorno correcto y valida la CLI:

```bash
ros2 daemon start
timeout 5s ros2 daemon status
timeout 15s ros2 node list
timeout 15s ros2 topic list
```

El resultado esperado es que `daemon status`, `node list` y `topic list` finalicen sin quedar suspendidos. Las listas pueden estar vacías si no hay nodos ejecutándose; una lista vacía no es, por sí sola, un fallo del daemon.

> **Atajo:** `killall _ros2_daemon` se ha usado para recuperar versiones de ROS 2 cuyo `ros2 daemon stop` tampoco responde. La secuencia `pgrep` + `kill <PID>` es preferible porque permite verificar exactamente qué proceso se terminará. No empieces con `kill -9`/`kill -KILL`: esa señal no permite que el proceso haga una salida ordenada.

### 1.2 Qué se está bloqueando

El daemon de ROS 2 no es un `roscore` ni un servidor DDS central. Es un proceso que mantiene información del grafo ROS para que las consultas de la CLI respondan con rapidez. Se inicia automáticamente cuando una herramienta de introspección lo necesita y atiende a la CLI mediante una conexión local en `127.0.0.1`.

Esto separa dos rutas que conviene diagnosticar:

```text
ros2 node/topic list ── XML-RPC local ── daemon ── RMW/DDS ── grafo ROS 2
```

- Si falla la conexión local entre la CLI y el daemon, los comandos pueden bloquearse aunque los nodos sigan publicando y recibiendo datos.
- Si la conexión local funciona pero DDS no descubre otros participantes, la CLI responde, aunque su lista puede estar incompleta o vacía.

Cada daemon queda asociado al `ROS_DOMAIN_ID` y al RMW activos al iniciarse. ROS 2 utiliza instancias distintas para dominios distintos. Por eso el comando que detiene el daemon debe ejecutarse con el mismo `ROS_DOMAIN_ID`; además, después de cambiar `RMW_IMPLEMENTATION` o un perfil DDS se debe detener la instancia anterior antes de consultar otra vez el grafo.

### 1.3 Por qué aparece de forma recurrente en WSL

Existe un reporte reproducible del proyecto `ros2cli` con ROS 2 Humble, Ubuntu 22.04 en WSL2 y `networkingMode=mirrored`: una consulta que necesita el daemon espera cerca de dos minutos y termina con `TimeoutError` al intentar conectarse al servicio local. El caso demuestra que una falla del camino local CLI-daemon puede coexistir con una red DDS que sí funciona; no implica que todas las instalaciones WSL tengan el defecto.

En la práctica, revisa el daemon después de cualquiera de estos cambios:

- WSL fue suspendido, apagado o reanudado.
- Windows cambió de Wi-Fi, VPN o interfaz de red.
- Se alternó entre red NAT y modo reflejado.
- Cambió `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, `ROS_DISCOVERY_SERVER` o un perfil de Fast DDS.
- Diferentes terminales cargaron archivos `setup.bash` o variables incompatibles.

No es correcto atribuir todo bloqueo a una "caché de IP antiguas". El síntoma también puede proceder de la conexión XML-RPC local, de un daemon iniciado con otro entorno o de la capa DDS. El siguiente diagnóstico permite distinguirlos.

### 1.4 Diagnóstico paso a paso

#### 1.4.1 Confirma el entorno de la terminal

Antes de reiniciar el daemon, carga la distribución y el workspace que realmente se utilizarán:

```bash
source /opt/ros/<distro>/setup.bash
source ~/ros2_ws/install/setup.bash

printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-0}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-predeterminado}"
printf 'ROS_AUTOMATIC_DISCOVERY_RANGE=%s\n' "${ROS_AUTOMATIC_DISCOVERY_RANGE:-predeterminado}"
```

Sustituye `<distro>` por la distribución instalada; en este proyecto es `jazzy`. Las máquinas participantes deben compartir el dominio configurado —`ROS_DOMAIN_ID=0`, el dominio de pruebas del docente y valor por defecto de ROS 2— y utilizar la misma implementación RMW (`rmw_cyclonedds_cpp`).

#### 1.4.2 Compara la consulta con y sin daemon

```bash
timeout 15s ros2 node list
timeout 15s ros2 node list --no-daemon
```

Interpreta el resultado:

| Resultado | Capa más probable | Acción siguiente |
| :--- | :--- | :--- |
| La consulta normal se bloquea y `--no-daemon` responde | Camino CLI-daemon local | Reinicia el daemon; usa `kill` si `stop` no responde. |
| Ambas consultas responden, pero muestran grafos diferentes | Daemon iniciado con otro entorno o información desactualizada | Revisa dominio/RMW/perfiles y reinicia el daemon. |
| Ambas consultas se bloquean o no descubren nodos remotos | RMW/DDS, multicast, firewall o red WSL | Continúa con la [configuración de red ROS 2](network_setup/ROS2_NETWORK_CONFIG.md). |
| La lista está vacía y no hay nodos activos | Comportamiento normal | Inicia un nodo y repite la prueba. |

`--no-daemon` es una herramienta temporal de aislamiento, no una garantía de descubrimiento correcto. Su disponibilidad depende del subcomando y de la versión de `ros2cli`; consulta `ros2 <comando> --help` si la opción no es reconocida.

#### 1.4.3 Detén la instancia correcta

Primero prueba la interfaz normal:

```bash
timeout 5s ros2 daemon status
timeout 5s ros2 daemon stop
```

El código de salida `124` de `timeout` significa que el comando superó el límite. En ese caso inspecciona los procesos:

```bash
pgrep -af '(_ros2_daemon|--name ros2-daemon)'
```

La línea suele incluir tanto el dominio como el RMW, por ejemplo:

```text
1234 /usr/bin/python3 /opt/ros/humble/bin/_ros2_daemon --rmw-implementation rmw_fastrtps_cpp --ros-domain-id 42
```

Según la versión de `ros2cli`, la línea también puede contener `--name ros2-daemon` en lugar del ejecutable `_ros2_daemon`. Comprueba los argumentos antes de ejecutar `kill <PID>`. Puede haber más de una instancia si se trabajó con varios dominios; termina solamente la que corresponda o detén cada PID de forma explícita si quieres reiniciar todo el entorno ROS 2 del usuario.

#### 1.4.4 Inicia y valida una instancia limpia

Con las variables correctas ya cargadas:

```bash
ros2 daemon start
timeout 5s ros2 daemon status
timeout 15s ros2 node list
```

Como prueba observable, inicia un publicador en otra terminal con el mismo entorno:

```bash
ros2 topic pub /daemon_test std_msgs/msg/String "{data: 'ok'}" -r 1
```

Y valida en la terminal recuperada:

```bash
timeout 15s ros2 topic list | grep '^/daemon_test$'
```

La recuperación queda validada cuando aparece `/daemon_test` y las consultas repetidas terminan sin demora anormal.

### 1.5 Si `kill` no basta en WSL

Si ya no existe `_ros2_daemon`, pero la CLI continúa agotando el tiempo al conectarse por localhost, reinicia completamente la VM de WSL. Guarda primero cualquier trabajo abierto y ejecuta en PowerShell:

```powershell
wsl --shutdown
```

Abre WSL de nuevo, carga `/opt/ros/<distro>/setup.bash` y el workspace, confirma las variables y repite la validación. Microsoft documenta que WSL2 usa NAT de forma predeterminada y que el modo reflejado cambia la arquitectura de red; reiniciar WSL es necesario después de modificar `.wslconfig`.

### 1.6 Prevención

- Detén el daemon antes de cambiar `ROS_DOMAIN_ID`, RMW o perfiles DDS.
- Carga el mismo archivo de configuración ROS 2 en todas las terminales del mismo equipo.
- Después de cambiar la red de Windows o reanudar WSL, reinicia el daemon antes de diagnosticar DDS.
- Usa `timeout` en los comandos de diagnóstico para conservar el control de la terminal.
- Mantén WSL y los paquetes de la distribución ROS 2 actualizados dentro de la versión compatible con el proyecto.
- Usa `--no-daemon` para aislar la falla, no como sustituto permanente de una configuración coherente.

### 1.7 Criterio de resolución

La persona que realiza el diagnóstico puede distinguir el servicio local del daemon de la comunicación DDS, identificar el PID y su dominio/RMW antes de terminarlo, recuperar la CLI sin reiniciar todo Windows y demostrar el resultado con un tópico observable.


---

## 2. Hardware ocupado: un solo robot y varias estaciones

### 2.0 La configuración recomendada (léela antes que el resto)

El verdadero cuello de botella del laboratorio no es que el robot esté ocupado —con un
solo brazo y varios equipos, **estará ocupado casi siempre**—. El cuello de botella es
*saber quién lo tiene*. Y esa pregunta no tiene respuesta técnica: **ROS 2 no expone en
qué máquina corre un nodo.** `ros2 topic info --verbose` devuelve el GID del participante
DDS, no un hostname ni una IP.

Con el dominio compartido, el package sí responde esa pregunta: cada `kinova_monitor`
verifica localmente si su máquina tiene la sesión con el robot y lo publica en
`/burger/kinova/diagnostics` (sección 2.2, paso 2). Pero eso **presupone** el dominio
común, así que la base sigue siendo una **convención**:

> ### 📌 Convención del laboratorio
>
> 1. **Un único computador ejecuta el driver.** Siempre el mismo, designado y anotado
>    junto al robot. Es la *estación anfitriona*. Hospeda todos los drivers que hablan con
>    el hardware: el brazo (`kortex_bringup`) y el de visión, que comparten la misma IP.
> 2. **Esa estación va por cable Ethernet.** Por WiFi la sesión de control se rompe; está
>    medido, no es una preferencia (sección 2.3, opción B).
> 3. **Todos los demás usan el mismo `ROS_DOMAIN_ID`** que la estación anfitriona, y
>    lanzan con `start_driver:=false`. Pueden ir por WiFi.

La convención nació de la experiencia: en el Laboratorio 02 (cámara del Kinova) la mayoría
de los grupos completó la práctica, pero el problema principal fue el **conflicto al usar la
cámara desde varios computadores a la vez**, cada uno con su propio driver de visión, agravado
porque ese driver no se detenía limpiamente y dejaba la cámara bloqueada (sección 3.4). Por eso
los talleres posteriores que usan el driver del robot —como el de
[localización con AprilTag](education/talleres/TALLER_LOCALIZACION_APRILTAG_KINOVA_MICROROS.md)—
parten de una sola anfitriona que publica la imagen comprimida, y todas las estaciones en el
mismo `ROS_DOMAIN_ID`.

Esta convención es la que **hace desaparecer el cuello de botella**, y conviene ver por
qué: si todos comparten el dominio, *"¿quién tiene el robot?"* deja de ser una pregunta
incontestable y pasa a resolverse con un comando:

```bash
# ¿Hay un driver corriendo?
timeout 15s ros2 node list | grep controller_manager
# ¿En qué máquina?
ros2 topic echo /burger/kinova/diagnostics --once | grep -A8 "identidad de la estación"
```

Si aparece, el driver está corriendo, sabes en qué computador, y debes conectarte a él en
lugar de lanzar el tuyo. Si cada equipo usa un dominio distinto, esa misma consulta devuelve vacío
aunque el robot esté plenamente en uso, y sólo queda barrer dominios a ciegas
(sección 2.2, paso 3) o preguntar en voz alta.

Dicho de otro modo: **el dominio compartido no es un detalle de configuración, es el
mecanismo de coordinación.** El resto de esta sección es qué hacer cuando la convención
no se siguió, o cuando algo quedó a medias.

---

### 2.1 El error de concepto que hay que quitarse primero

Lo que está ocupado **no es el driver: es el robot**. Y esa ocupación vive en una capa
**por debajo** de ROS 2:

```text
  Estación A ──── TCP/UDP Kortex (puerto 10000) ────> Kinova Gen3
                  ▲ UN solo modo de servo del brazo, compartido por
                  │ todas las sesiones. Esta capa NO sabe qué es ROS_DOMAIN_ID

  Estación A ◄─── DDS (aislado por ROS_DOMAIN_ID) ──► Estación B
                  Aquí sí manda el dominio
```

De ahí la regla que más confusión causa:

> ⚠️ **Cambiar tu `ROS_DOMAIN_ID` NO libera el robot ni te habilita a lanzar tu propio
> driver.** El dominio aísla el grafo ROS 2, no la sesión con la controladora. Y la
> controladora **no rechaza** a un segundo driver: lo acepta, ese driver cambia el modo de
> servo del brazo y le quita el control al primero, que sigue corriendo sin enterarse.
> Resultado: `WRONG_SERVOING_MODE`, movimiento a tirones y ambos drivers "vivos"
> (sección 2.6).

El aislamiento por dominio sólo sirve para el caso **simulado** (`use_fake_hardware:=true`),
donde no hay robot físico que compartir.

Son, por tanto, dos síntomas distintos con causas distintas:

| Síntoma | Capa | Qué significa |
| :--- | :--- | :--- |
| El driver no arranca; la conexión al robot no progresa | Kortex TCP/UDP | El robot no es alcanzable, o una sesión matada en duro aún no expira |
| Ambos drivers arrancan, pero el primero registra `WRONG_SERVOING_MODE` y el brazo se mueve a tirones | Kortex (modo de servo) | Dos drivers sobre el mismo robot, **en cualquier dominio** (sección 2.6) |
| Arranca, pero hay `/joint_states` duplicado, `/controller_manager` con respuestas erráticas o metas aceptadas y abortadas a la vez | Grafo ROS 2 / DDS | Hay **dos drivers en tu mismo dominio** |

### 2.2 Diagnóstico: tres preguntas en orden

#### Paso 1 · ¿La sesión la tiene mi propia máquina?

Empieza por aquí: lo más frecuente es un proceso huérfano tuyo, no un compañero.

```bash
ROBOT_IP=192.168.1.10          # ajusta a la IP verificada del laboratorio
ss -tanp | grep "$ROBOT_IP"
```

Interpretación:

| Salida | Significado |
| :--- | :--- |
| *(vacío)* | Tu máquina no tiene ninguna sesión abierta con el robot |
| `ESTAB ... 192.168.1.10:10000 users:(("ros2_control_no",pid=NNNN,...))` | **Tú tienes la sesión.** Ese PID es el driver |
| `SYN-SENT ... 192.168.1.10:10000` | Estás **intentando** conectar y el robot no responde: no es "ocupado", es inalcanzable → ve al paso 4 |

Si aparece un `ros2_control_node` que tú no lanzaste conscientemente, es un huérfano de una
sesión anterior (típicamente por haber matado el launch con `kill -9` en vez de `SIGINT`).
Ciérralo de forma ordenada:

```bash
kill -INT <PID>      # nunca -9 de entrada: la sesión Kortex debe cerrarse limpiamente
sleep 10
ss -tanp | grep "$ROBOT_IP" || echo "sesión liberada"
```

#### Paso 2 · ¿Hay un driver publicando en MI dominio?

```bash
echo "mi dominio: ${ROS_DOMAIN_ID:-0}"
timeout 15s ros2 node list
timeout 15s ros2 topic info /joint_states
```

Si ves `/controller_manager` y `/joint_states` **con un publicador**, hay un driver activo y
visible para ti. No lances otro: **conéctate a él** (sección 2.3, opción A).

> **Qué NO vas a obtener por aquí:** el nombre de la máquina que lo ejecuta.
> `ros2 topic info /joint_states --verbose` muestra el GID del participante DDS, no un
> hostname ni una IP. **ROS 2 no expone en qué máquina corre un nodo.**

Para eso el package publica la identidad de cada estación en el diagnóstico. Cada
`kinova_monitor` comprueba **localmente** si su propia máquina mantiene la sesión TCP con
la controladora, y lo anuncia. Desde cualquier estación del mismo dominio:

```bash
ros2 topic echo /burger/kinova/diagnostics --once | grep -A8 "identidad de la estación"
```

```text
message: PC-LAB-01 (192.168.1.42) — anfitriona
  rol_estacion    : anfitriona
  rol_verificado  : si
  rol_evidencia   : sesión TCP establecida con 192.168.1.10:10000
```

| `rol_estacion` | `rol_verificado` | Significado |
| :--- | :---: | :--- |
| `anfitriona` | `si` | **Esa máquina tiene el robot.** Hay sesión TCP establecida: es prueba directa, no una suposición |
| `cliente` | `si` | Esa máquina no tiene el robot. Es el estado normal de una estación cliente |
| `desconocido` | `no` | Lanzó el driver pero aún no hay sesión, o el robot está en `SYN-SENT` (no responde) |

Cada monitor sólo afirma sobre sí mismo, que es lo único que puede verificar: en una red
conmutada nadie ve las conexiones TCP de otro equipo. Por eso **la anfitriona se anuncia**
en vez de intentar detectarse desde fuera. No hay IP que configurar, y si mañana la
anfitriona es otro computador el diagnóstico lo refleja solo.

> ⚠ Esto sólo funciona si compartes el `ROS_DOMAIN_ID` con la estación anfitriona —otra
> razón para la convención de la sección 2.0—. Si no lo compartes, sigue valiendo el `ss`
> del paso 1 para **tu propia** máquina, y el monitor de red del párrafo siguiente, que
> no depende del dominio.

##### Sin abrir una terminal: el monitor de red

La misma información aparece en la tabla de dispositivos del monitor web, con un
distintivo verde `🔒 ANFITRIONA · driver Kinova` en la fila de la máquina que lo tiene:

```bash
./network_setup/iniciar_monitor.sh        # y abrir http://<ip-del-monitor>:8080
```

Llega por un canal distinto —broadcast UDP en el puerto `45455`, no DDS—, así que
**funciona aunque no compartas el `ROS_DOMAIN_ID`**. Es la vía recomendada para quien
llega al laboratorio y sólo quiere saber si el robot está libre. Detalle en
[`MONITOR_UI_GUIA.md`](network_setup/MONITOR_UI_GUIA.md) §3.7.

Las tres superficies muestran el mismo dato verificado, y ninguna sustituye a las otras:

| Vía | Alcance | Cuándo usarla |
| :--- | :--- | :--- |
| `ss -tanp` (paso 1) | Sólo tu máquina | Sospechas de un huérfano tuyo |
| `/burger/kinova/diagnostics` | Estaciones de tu dominio | Ya estás trabajando en ROS 2 |
| Monitor de red | Toda la subred, sin ROS | Llegas al lab y quieres saber si está libre |

> Si estos comandos se bloquean, el problema es el daemon, no el robot: ve a la
> [sección 1](#1-bloqueo-del-daemon-de-ros-2-en-wsl) antes de seguir.

#### Paso 3 · ¿Y si no veo nada, pero el robot igual está ocupado?

Es el caso que más despista, y tiene una explicación exacta: **los dominios DDS son
mutuamente invisibles**. Si un compañero corre el driver en `ROS_DOMAIN_ID=7` y tú estás en
el `0`, `ros2 node list` te devuelve una lista vacía aunque el robot esté plenamente en uso.

No existe un comando de ROS 2 que responda *"¿en qué dominio está el driver?"*, porque para
preguntarlo ya tendrías que estar en ese dominio.

Si estás aquí, es que la convención de la sección 2.0 no se siguió: con un dominio único
para todo el laboratorio este paso no existiría. Hay dos vías para salir del paso:

1. **Preguntar.** En la práctica es lo más rápido y lo que evita accidentes. El dominio de
   trabajo del curso es `ROS_DOMAIN_ID=0` salvo acuerdo explícito del equipo.
2. **Barrer dominios**, si no hay nadie a quien preguntar:

   ```bash
   for d in $(seq 0 12); do
     encontrados=$(ROS_DOMAIN_ID=$d timeout 8s ros2 node list --no-daemon 2>/dev/null \
                   | grep -c controller_manager)
     [ "$encontrados" -gt 0 ] && echo "  → driver activo en ROS_DOMAIN_ID=$d"
   done
   echo "barrido terminado"
   ```

   `--no-daemon` es lo que hace viable el barrido: consulta el grafo directamente y así no
   hay que levantar, cambiar y matar un daemon por cada dominio (recuerda que cada daemon
   queda atado al `ROS_DOMAIN_ID` con el que arrancó, sección 1.4.3). Aun así el barrido es
   lento —unos segundos por dominio— y sólo alcanza los dominios que enumeres.

#### Paso 4 · Si nadie lo tiene y aun así no conecta

Entonces no es ocupación, es alcance. Verifica en este orden:

```bash
ping -c 4 "$ROBOT_IP"                  # ¿está en la red y responde?
ip route get "$ROBOT_IP"               # ¿sales por la interfaz que crees?
ip -4 addr show | grep inet            # ¿estás en la misma subred que el robot?
```

Una sesión Kortex matada en duro puede tardar en expirar dentro de la controladora. Si el
robot responde a `ping` pero rechaza la sesión, espera a que expire o reinicia el brazo.

### 2.3 Qué hacer según el resultado

Las dos salidas **no son alternativas equivalentes**: la primera es lo que haces hoy para
trabajar, la segunda es cómo debe quedar montado el laboratorio.

#### Opción A · Conectarte al driver que ya está corriendo *(lo normal)*

Es el modo previsto por el proyecto, no un plan B. Tu estación consume la telemetría y el
servidor de acción por DDS:

```bash
export ROS_DOMAIN_ID=<el mismo de la estación que tiene el driver>
ros2 launch burger_kinova_connection kinova_connection.launch.py   start_driver:=false   enable_motion:=false
```

Requisitos, ambos obligatorios:

- **Mismo `ROS_DOMAIN_ID`** que la estación del driver.
- **Misma subred**, con el descubrimiento en su valor por defecto (`SUBNET`). No lo cambies
  a `LOCALHOST`: eso aislaría tu estación y es justo lo contrario de lo que necesitas.

Tu estación cliente **puede estar por WiFi**: sólo consume telemetría, no sostiene el ciclo
de control del robot.

#### Opción B · Montar el laboratorio para que "ocupado" deje de ser un error

Es la convención de la sección 2.0, con su justificación. No resuelve un incidente: define
quién puede ocupar el robot, y de paso convierte *"¿quién lo tiene?"* en una consulta de
un solo comando.

- **Una única estación anfitriona** ejecuta *todos* los drivers que hablan con el hardware:
  el brazo (`kortex_bringup`) y el de visión. El módulo de visión del Gen3 vive en la misma
  IP, así que comparte enlace y conviene que comparta también estación.
- **Esa estación va por cable Ethernet, no por WiFi.** No es una preferencia de
  rendimiento: medido sobre el robot real, por WiFi el ciclo de control se rompe —132
  desbordamientos, un hueco de 3.25 s y 2 pérdidas de telemetría en 120 s— mientras que por
  cable el peor intervalo fue de 20.6 ms y no hubo ninguna pérdida. La medición completa
  está en
  [`EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md`](burger_kinova_reference/docs/EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md).
- **Todas las demás estaciones son clientes** (opción A), y pueden ir por WiFi.
- El equipo acuerda **un solo `ROS_DOMAIN_ID`** y lo anota donde todos lo vean.

#### Opción C · Necesitas el robot y lo tiene otro

Se coordina entre personas, no por software. Quien lo tenga cierra su driver con `SIGINT`
(nunca `kill -9`) y verifica que la controladora liberó la sesión:

```bash
# En la estación que lo tiene:
# Ctrl+C sobre el launch, y confirmar en su log:
#   KortexMultiInterfaceHardware successfully deactivated!
ss -tanp | grep 192.168.1.10 || echo "sesión liberada"
```

Sólo entonces la siguiente estación lanza con `start_driver:=true`.

### 2.4 Prevención

- **Aplica la convención de la sección 2.0**: una estación anfitriona designada, por cable,
  y un único `ROS_DOMAIN_ID` para todo el laboratorio. Es lo que convierte "¿quién tiene el
  robot?" en un comando en vez de una búsqueda.
- Cierra siempre el driver con `Ctrl+C` / `SIGINT`. Un `kill -9` deja la sesión Kortex
  abierta y el siguiente arranque falla sin motivo aparente.
- Lanza el driver **siempre** con `kinova_connection.launch.py` de
  `burger_kinova_reference`: antes de incluir `kortex_bringup` busca un driver activo y, si
  lo encuentra, emite un `WARNING` y no lo lanza (sección 2.6). Para comprobarlo a mano:
  ```bash
  ss -tanp | grep 192.168.1.10; timeout 15s ros2 node list | grep controller_manager
  ```
  Si ambas salidas están vacías, el camino está libre **desde tu punto de vista**: `ss`
  sólo ve tu máquina, y `ros2 node list` sólo tu dominio.
- Anota el `ROS_DOMAIN_ID` acordado del equipo junto al robot. Es más barato que barrer
  dominios.
- En modo simulado (`use_fake_hardware:=true`) sí conviene que **cada equipo use un
  `ROS_DOMAIN_ID` distinto**, para no interferirse entre sí.

### 2.5 Criterio de resolución

La persona que diagnostica distingue la sesión Kortex del grafo DDS, sabe que cambiar de
dominio no libera el robot, identifica con `ss` qué proceso tiene la sesión, y decide con
criterio entre conectarse como cliente o pedir que liberen el hardware. Y entiende por qué
el dominio compartido no es un detalle de configuración sino el mecanismo con el que el
equipo se coordina alrededor de un único robot.

### 2.6 Otra estación lanzó un segundo driver y le quitó el control al primero

Incidente real del [Laboratorio 03](education/guias_laboratorio/GUIA_LAB_03_OPERACION_DISTRIBUIDA_KINOVA_TURNOS.md)
(16/09/2026): con la anfitriona operando, otro equipo lanzó `start_driver:=true` desde su
portátil por WiFi. El robot empezó a moverse a tirones y la anfitriona perdió el control,
aunque su driver **nunca se cayó**.

#### Síntomas en el log de la estación que ya tenía el robot

```text
[KortexMultiInterfaceHardware]: Kortex exception: Device error, Error sub type=WRONG_SERVOING_MODE => <srv: 3, fct: 1, msgType: 3>
description: Wrong servoing mode, must be low level servoing mode
[KortexMultiInterfaceHardware]: Fault was not recognized on the robot but combination of Control Mode and Active State are not supported!   ← repetido a 100 Hz
```

Y en `ros2 topic hz /joint_states`, desde cualquier estación del dominio:

```text
average rate: 105.441
        min: 0.000s max: 0.030s      ← ~105 Hz e intervalo 0.000 s: DOS publicadores intercalados
average rate: 85.675
        min: 0.002s max: 16.530s     ← hueco de 16 s sin telemetría
```

Las metas de trayectoria pueden seguir terminando en `Goal reached, success!`: el
controlador de ROS 2 no sabe que el brazo obedeció a otro.

#### Causa: el modo de servo es del robot, no de la sesión

La controladora acepta varias sesiones Kortex a la vez. Lo que es único es el **modo de
servo** del brazo, y el driver `kortex_driver` lo cambia sin coordinarse con nadie
([`hardware_interface.cpp`](../ros2_kortex/kortex_driver/src/hardware_interface.cpp)):

| Momento del segundo driver | Qué le hace al robot | Qué ve el primer driver |
| :--- | :--- | :--- |
| Al configurarse (líneas 204–225) | `SINGLE_LEVEL_SERVOING` → `ClearFaults` → `LOW_LEVEL_SERVOING` | Sus comandos fallan con `WRONG_SERVOING_MODE`; mientras el robot no está en bajo nivel entra en la rama de la línea 907 (`Fault was not recognized…`) y **no comanda nada** |
| Mientras ambos corren | Los dos envían consignas de posición a 100 Hz | El brazo alterna entre las dos consignas: movimiento a tirones |
| Al cerrarse (líneas 727–731) | `SINGLE_LEVEL_SERVOING` | El primero vuelve a quedar sin control, otra vez con `WRONG_SERVOING_MODE` |

El primer driver guarda en memoria `arm_mode_ = LOW_LEVEL_SERVOING` y nunca lo vuelve a
consultar, así que no se recupera solo ni se detiene.

> ⚠ **El `ROS_DOMAIN_ID` no protege.** El modo de servo vive en la capa Kortex. Un driver
> en *otro* dominio produce exactamente el mismo daño; la única diferencia es que no lo
> verás en `ros2 node list` ni en `/joint_states`.

#### Qué hace ahora el package: la guarda previa al driver

`kinova_connection.launch.py` ejecuta
[`driver_guard.py`](burger_kinova_reference/burger_kinova_reference/driver_guard.py) antes
de incluir `kortex_bringup`, y combina tres vías porque ninguna sola ve todos los casos:

| Vía | Detecta | Depende del dominio |
| :--- | :--- | :---: |
| Sesión TCP local (`/proc/net/tcp`) | Un driver en **esta** máquina: otra terminal o un huérfano | No |
| Grafo DDS | `/controller_manager` o publicadores de `/joint_states` en cualquier máquina | Sí |
| Anuncio UDP `45455` | Un `kinova_monitor` que se declara anfitriona verificada del mismo robot | No (misma subred) |

Si encuentra evidencia, **no lanza el driver**, emite `WARNING` y la estación continúa como
cliente (sólo el monitor):

```text
[INFO] [launch.user]: Comprobando que no haya otro driver activo para 192.168.1.10 (hasta 6.0 s)...
[WARNING] [launch.user]: NO se lanza kortex_bringup: ya hay un driver activo para este robot. ...
[WARNING] [launch.user]:   evidencia: ya existe el nodo /controller_manager en ROS_DOMAIN_ID=0: otra estación tiene el driver corriendo
[WARNING] [launch.user]:   evidencia: la estación PC-LAB-01 (192.168.1.42) se anuncia como ANFITRIONA de 192.168.1.10: sesión TCP establecida con 192.168.1.10:10000
[WARNING] [launch.user]: Esta estación continúa como CLIENTE (start_driver:=false). ...
```

| Argumento | Defecto | Uso |
| :--- | :--- | :--- |
| `check_existing_driver` | `true` | `false` omite la comprobación. Sólo para diagnóstico, nunca en el laboratorio |
| `driver_check_timeout_s` | `6.0` | Ventana máxima: cubre el descubrimiento DDS entre máquinas y un periodo del anuncio (5 s). Si hay evidencia, termina antes |

Si la evidencia es un driver **tuyo** (sesión TCP local), es un huérfano: ciérralo con
`kill -INT <PID>` (sección 2.2, paso 1) y relanza.

#### Límites de la guarda: lo que no puede ver

- **Un driver en otro PC, en otro dominio y sin `kinova_monitor`** (por ejemplo,
  `ros2 launch kortex_bringup gen3.launch.py` a mano). No deja rastro en tu dominio ni se
  anuncia.
- **Dos estaciones que lanzan a la vez**: ambas comprueban antes de que la otra exista.
- **Broadcast bloqueado**: en WSL2 con red NAT, o si el router aísla los clientes WiFi, el
  anuncio UDP no llega y sólo quedan las otras dos vías.
- **Quien ignora la guarda** con `check_existing_driver:=false` o sin usar este launch.

Por eso la guarda **complementa** la convención de la sección 2.0, no la reemplaza.

#### Protección definitiva: en el robot, no en ROS

La única barrera que ningún launch puede saltarse está en la controladora:

1. **Crear un usuario Kortex exclusivo para la anfitriona.** Los drivers entran con
   `admin`/`admin`, que viene fijo en
   `kortex_description/arms/gen3/6dof/urdf/gen3_macro.xacro` (`password:=admin`) y
   `gen3.launch.py` **no lo expone como argumento**. Si en la aplicación web del robot
   (`http://192.168.1.10`) se cambia la contraseña de `admin`, cualquier driver con las
   credenciales por defecto fallará al crear la sesión en vez de quitarle el control a
   nadie. El costo: la anfitriona necesita esas credenciales en su URDF, lo que exige una
   copia local del xacro o un parche en `ros2_kortex`, que este proyecto no modifica. Es
   una decisión del responsable del laboratorio, no del estudiante.
2. **Aislar el robot de la red compartida.** Conectarlo por cable directo a una segunda
   tarjeta de red de la anfitriona, en otra subred. Las estaciones cliente siguen viendo
   todo por DDS a través de la anfitriona, pero ningún otro PC alcanza el puerto `10000`.

#### Si ya ocurrió

1. **Pulsa la parada de emergencia** si el brazo se mueve de forma errática.
2. En la estación intrusa: `Ctrl+C` sobre su launch. Verás de nuevo `WRONG_SERVOING_MODE` en
   la anfitriona: su cierre devuelve el robot a `SINGLE_LEVEL_SERVOING`.
3. En la anfitriona: **reinicia también su driver** (`Ctrl+C` y relanzar). Su modo en
   memoria ya no coincide con el del robot y no se corrige solo.
4. Confirma con `ros2 topic info /joint_states` que queda **un** publicador y registra el
   incidente en la Tabla 5 de la guía.

#### Otros mensajes del mismo log que NO son la causa

| Mensaje | Qué es |
| :--- | :--- |
| `Could not enable FIFO RT scheduling` y `Overrun detected ... Write time : 10–38 ms` | Anfitriona en WSL2 sin planificación de tiempo real. Añade temblor al ciclo, pero no quita el control |
| `Loader for controller 'twist_controller' / 'fault_controller' ... not found` | Faltan los plugins de PickNik. No afecta a los laboratorios con `joint_trajectory_controller` |
| `Segmentation fault` en `~GripperCyclic::Command` al hacer `Ctrl+C` | Defecto de `kortex_driver` al destruirse, **después** de `successfully deactivated!`: la sesión ya se cerró bien |
| `ddsi_udp_conn_write ... failed with retcode -1` | La estación perdió la interfaz de red (WSL, WiFi o cable). Es de la capa DDS, no del robot |


---

## 3. Fallos de plataforma ajenos a tu código

Esta sección recoge problemas detectados durante la validación sobre el robot real: los tres
primeros el 2026-09-09 y el del driver de visión (3.4) el 2026-09-15. Todos hacen fallar
**cualquier implementación correcta**, así que si tu proyecto o tu práctica no funcionó,
empieza por descartarlos antes de revisar tu código.

### 3.1 El brazo es de 6 GDL, y el enunciado decía 7

Hasta el 2026-09-09 el enunciado del proyecto y las guías declaraban un Kinova Gen3 de
**siete** grados de libertad. El brazo del laboratorio tiene **seis**, con pinza Robotiq
2F-85. El propio driver lo dice en cada arranque:

```text
[KortexMultiInterfaceHardware]: Actuator count reported by robot is '6'
```

Lo traicionero es que **lanzarlo con `dof:=7` no produce ningún error**. El driver expone
una séptima articulación que el robot nunca alimenta, y esa casilla publica lo que hubiera
en memoria:

| Sesión | `joint_7` position | `joint_7` effort |
| :--- | ---: | ---: |
| Una | `1.1207224803148005e+277` | `0.0` |
| Otra | `0.0` | `0.0` |

Las consecuencias sobre un proyecto bien hecho:

- Tu monitor reporta `7/7 articulaciones` y telemetría saludable, porque el valor **es**
  finito y, cuando cae en `0.0`, está dentro de todos los límites articulares. Ninguna
  validación de rango puede distinguirlo de una articulación legítimamente en el origen.
- Tu cliente de trayectoria calcula el desplazamiento de `joint_7` contra una posición
  inventada, y una meta de siete elementos comanda una articulación que no existe.

**Cómo detectarlo.** Compara el ruido por articulación con el robot quieto: las reales
fluctúan en los últimos decimales, la fabricada es bit-idéntica.

```bash
for i in 1 2 3 4 5; do ros2 topic echo /joint_states --once | grep -A8 "^position:"; done
```

**Qué hacer.** Usar `dof:=6` y seis articulaciones en `expected_joints`,
`safe_joint_positions_rad`, `joint_min_rad` y `joint_max_rad`. La articulación de la pinza
(`robotiq_85_left_knuckle_joint`) también aparece en `/joint_states` y debe **ignorarse**
sin invalidar el mensaje. Detalle en
[`ANOMALIAS_HARDWARE.md`](network_setup/ANOMALIAS_HARDWARE.md) §3.

### 3.2 Con pinza, el launch no arranca: `Invalid parameter "mock_sensor_commands"`

```text
error: Invalid parameter "mock_sensor_commands"
  when instantiating macro: robotiq_gripper (/opt/ros/jazzy/share/robotiq_description/...)
```

El `kortex_description` que viene de Kinova pasa al macro de la pinza argumentos que el
`robotiq_description` instalado por apt en Jazzy **no acepta**. El macro instalado admite
`sim_ignition`, `sim_isaac`, `use_fake_hardware`, `fake_sensor_commands`,
`include_ros2_control` y `com_port`; el de Kinova le envía además `mock_sensor_commands`,
`sim_gazebo`, `isaac_joint_commands` e `isaac_joint_states`.

No es un fallo de tu launch ni de tus parámetros: el xacro no llega a generarse, así que
**ningún nodo arranca**. Procedimiento de corrección en
[`INSTALACION_KORTEX.md`](ros2_setup/INSTALACION_KORTEX.md) §3.4.b.

### 3.3 `error: invalid syntax (<expression>, line 0)` al generar el URDF

Aparece si alguien retiró de un xacro los parámetros de simulación **pero dejó los bloques
`<xacro:if>` que los usaban**, quedando expresiones `${}` vacías:

```xml
<xacro:if value="${}">          <!-- ← expresión vacía: xacro no puede evaluarla -->
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</xacro:if>
```

El mensaje no señala el archivo ni la variable, sólo `line 0`. Para localizarlo:

```bash
grep -rn '\${}' ~/ros2_ws/src/ros2_kortex/kortex_description/
```

La corrección es eliminar también esos bloques, no reponer las variables. Ocurrió en la
máquina del docente y dejó inutilizable la descripción de 6 GDL, que es la de este robot;
por eso se venía usando `dof:=7` y se llegó al problema 3.1.

### 3.4 El driver de visión no se detiene limpiamente y deja la cámara bloqueada

Fue uno de los problemas más reportados en el Laboratorio 02. Al detener
`kinova_vision.launch.py` con `Ctrl+C` el launch muestra:

```text
[kinova_vision_node-2] terminate called after throwing an instance of 'std::system_error'
[ERROR] [kinova_vision_node-2]: process has died [pid …, exit code -11, cmd '…kinova_vision_node …']
[ERROR] [kinova_vision_node-1]: process[kinova_vision_node-1] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'
```

y, al volver a lanzarlo, la cámara **rechaza el stream** durante varios segundos:

```text
[camera.kinova_vision_color]: [color]: Failed to start stream
[camera.kinova_vision_color]: [color]: Trying to connect... (attempt #1)
```

Medido sobre el robot real (driver `ros2_kortex_vision`, commit `d1d0213`, color y
profundidad, relanzando 3 s después de cada parada):

| Driver | Parada con `Ctrl+C` | Cerrar la terminal (`SIGHUP`) | Relanzar tras la parada |
| :--- | :--- | :--- | :--- |
| Original | **0 de 3 limpias**: segfault (`-11`), aborto (`-6`) o escalado a `SIGTERM`; 3–7 s | Muerte abrupta de todos los procesos | La cámara rechazó el stream **más de 12 s** (2 de 3 intentos) |
| Con el parche | **11 de 11 limpias** (`process has finished cleanly`), ≈ 1.5 s | Parada limpia, ≈ 1.6 s | Conecta sin reintentos |

**Por qué pasa.** Son tres defectos del driver, no de tu estación:

1. `vision_node.cpp` instala su propio manejador de `SIGINT` que llama a `quit()`,
   `rclcpp::shutdown()` y `exit()` **desde dentro de la señal**, mientras el hilo principal
   sigue usando GStreamer. Ninguna de esas funciones es segura en un manejador de señal: de
   ahí el segfault, el aborto o el bloqueo.
2. Al salir, el pipeline de GStreamer nunca pasa a `NULL`, así que la sesión RTSP con la
   cámara no se cierra de forma ordenada y la cámara tarda en aceptar otra.
3. El lazo principal crea un ejecutor temporal y duerme con `Rate::sleep()` en cada
   iteración; si el apagado llega en medio, ambas llamadas lanzan excepción
   (`failed to create guard condition`, `context cannot be slept with because it's invalid`).

`rclcpp` tampoco gestiona `SIGHUP`, que es lo que llega al **cerrar la ventana de la
terminal**, así que ese gesto tan habitual mataba el driver sin cerrar la sesión RTSP.

**La consecuencia en el laboratorio.** Un proceso `kinova_vision_node` huérfano (por ejemplo,
si el launch muere antes que sus nodos) **sigue con la sesión RTSP abierta**: medido, con
`ESTAB` hacia `192.168.1.10:554` e ignorando `SIGINT`. Si otra estación intenta usar la
cámara, o la misma relanza el driver, se encuentra con `Failed to start stream`. Junto con la
falta de una estación anfitriona única (sección 2.0), explica el conflicto al usar la cámara
desde varios computadores.

**Cómo detectarlo** en la estación que lanzó el driver:

```bash
pgrep -a -x kinova_vision_n          # el nombre del proceso se trunca a 15 caracteres
ss -tanp | grep "192.168.1.10:554"   # sesiones RTSP abiertas desde esta estación
```

Si tras detener el launch queda algún `kinova_vision_n` o alguna línea `ESTAB`, hay un
huérfano. Con el driver original **`SIGINT` no basta**: usa `kill -TERM <PID>`. No uses
`pkill -f kinova_vision` desde un script (sección 4.3).

**Qué hacer.** En la estación anfitriona, un solo comando:

```bash
bash ~/ros2_ws/src/burger_delivery/scripts/aplicar_parche_kinova_vision.sh           # prepara y compila
bash ~/ros2_ws/src/burger_delivery/scripts/aplicar_parche_kinova_vision.sh --check   # sólo comprueba
```

El script es independiente de `aplicar_compatibilidad_kortex.py`. Hace todo el proceso:
- clona `ros2_kortex_vision` si falta;
- avisa si faltan paquetes del sistema;
- aplica el parche
  [`ros2_setup/parches/kinova_vision_parada_limpia.patch`](ros2_setup/parches/kinova_vision_parada_limpia.patch)
  sin repetirlo si ya está;
- verifica el contenido de los fuentes y compila sólo `kinova_vision`.

Si el parche no aplica (cambios locales en el clon, o un upstream distinto de `d1d0213`), se
detiene y dice por qué, en lugar de forzarlo. Resumen para estudiantes en el
[README](README.md#-preparar-los-paquetes-de-kinova-antes-de-compilar).

Qué cambia el parche, uno por cada defecto descrito arriba:

| Defecto | Cambio en el driver |
| :--- | :--- |
| 1. Manejador de `SIGINT` inseguro | Se elimina. `rclcpp` gestiona `SIGINT` y `SIGTERM`, y `quit()` se llama desde un callback `rclcpp::on_shutdown()`, que corre fuera del manejador de señal y aun así desbloquea la espera de imagen de GStreamer |
| 2. El pipeline nunca pasa a `NULL` | `run()` llama a `stop()` al salir del lazo, así `rtspsrc` cierra la sesión RTSP con la cámara |
| 3. Excepciones durante el apagado | Un único `SingleThreadedExecutor` para todo el lazo. Si `spin_some()` o `sleep()` lanzan **porque el contexto ya se apagó**, se sale del lazo; cualquier otro error se relanza |
| `SIGHUP` al cerrar la terminal | Se redirige a `SIGINT` con `raise()`, que es seguro dentro de un manejador de señal |

El parche se verificó sobre un clon limpio del commit `d1d0213` (aplica y compila) y pasa los
hooks de `pre-commit` del upstream. La corrección, con estas mediciones, se envió a Kinova como
[Kinovarobotics/ros2_kortex_vision#2](https://github.com/Kinovarobotics/ros2_kortex_vision/pull/2):
si se integra, el parche deja de ser necesario.

> Lo único que ningún código puede atender es `SIGKILL` o un corte de energía: en ese caso
> la sesión RTSP queda sin cerrar y la cámara puede rechazar streams nuevos durante más de
> 12 s. Espera antes de relanzar, o relanza y deja que el driver reintente.

### 3.5 Si tu proyecto del corte 1 no funcionó

Antes de dar por malo tu código, comprueba en este orden:

1. `ros2 topic echo /joint_states --once` — ¿aparecen seis articulaciones más la de la
   pinza, o siete `joint_N`? Si son siete, estabas contra el problema 3.1.
2. ¿El launch llegaba a arrancar con pinza? Si no, era el problema 3.2.
3. ¿La estación que ejecutaba el driver estaba por **cable**? Por WiFi la sesión de
   control se rompe: medido, 132 desbordamientos y 2 pérdidas de telemetría en 120 s
   ([`EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md`](burger_kinova_reference/docs/EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md)).
4. ¿Había otra estación con el driver abierto? Ver la [sección 2](#2-hardware-ocupado-un-solo-robot-y-varias-estaciones).
5. Si usabas la cámara: ¿el driver de visión tenía el parche de parada limpia, y no quedaban
   procesos `kinova_vision_n` huérfanos? Ver el problema 3.4.

Sólo si los cinco salen limpios tiene sentido revisar la lógica de tu package.

---

## 4. Fallos silenciosos al ejecutar los talleres

**Origen.** El taller de rosbag2 tuvo que reescribirse porque su versión anterior no corría.
Eso motivó revisar el resto de los talleres de `education/talleres/`, la Guía de Laboratorio
02 y `TEORIA_LOGGING_ROS2.md` contra lo que realmente hacen el código y la instalación.

**Entorno de la revisión (2026-09-15):**
- ROS 2 Jazzy sobre Ubuntu 24.04 en WSL2 con red `mirrored`.
- `rmw_cyclonedds_cpp` con `network_setup/cyclonedds.xml`.
- OpenCV 4.6.0, el de `python3-opencv`.
- Las prácticas simuladas se ejecutaron en un dominio aislado para no interferir con el
  laboratorio.
- Las partes de cámara se probaron sobre el Kinova real (`192.168.1.10`) desde una estación
  por WiFi.

**Qué se ejecutó y qué sólo se revisó.** No todos los talleres se pudieron correr de punta a punta:

| Taller o documento | Cómo se verificó |
| :--- | :--- |
| rosbag2, logging y depuración | **Ejecutado completo**: nodo emulador, niveles de log, `ros2 param set`, servicios de logger, grabación con y sin compresión, parada desde scripts con cada señal, `ros2 bag play`, volcado del *flight recorder* y lectura con `rosbag2_py` |
| URDF y TF2 | **Ejecutado en parte**: compilación limpia de `burger_description`, resolución de las mallas `package://` y comportamiento de `ln -s`. RViz y `display.launch.py` no se lanzaron |
| Localización AprilTag | **Ejecutado**: localizador simulado, modo real con una escena sintética en perspectiva y, en la Guía 02, cámara y driver `kinova_vision` reales. **Sin tags físicos** en la mesa |
| Guía de Laboratorio 02 | **Ejecutadas las Fases 1 a 3** sobre el robot real, con mediciones del monitor de red |
| micro-ROS en ESP32 | **Revisado contra el código y la instalación**. Se comprobó el comportamiento de `ros2 topic pub --once`. El agente no estaba instalado y los firmwares **no se compilaron** (no hay toolchain de ESP32) |
| CLI de ROS 2 y TF2 con turtlesim | **Revisados contra la instalación**: nombres de los plugins de `rqt`, formato de salida de `tf2_echo`. `turtle_tf2_py` no estaba instalado, así que la demo de TF2 no se ejecutó |

**El patrón común.** Casi ningún fallo producía un error: **el comando termina, pero no hace
lo que la guía promete**. Aparecieron cinco formas concretas:

1. **Código de salida 0 fallando.** `ros2 param set … log_level` y `ln -s` sobre un enlace
   existente no hacen lo pedido y aun así salen con 0 (4.2, 4.5).
2. **Omisión silenciosa.** `ros2 bag record` ignora sin avisar un tópico que nadie publica, y
   un `kill -INT` a una grabación en segundo plano no la detiene (4.3).
3. **Datos falsos con apariencia normal.** El lector de bags reportaba jitter `0.00000`, el
   monitor de red contaba el loopback como WiFi y `ros2 topic bw` daba 1.88 MB/s para un
   video de ≈ 162 MB/s (4.4, 4.8).
4. **Un entorno que oculta el fallo.** Las mallas funcionaban sólo porque existía un
   `install/` antiguo, y las prácticas simuladas "funcionaban" mezclando datos de otras
   estaciones del mismo dominio (4.1, 4.5).
5. **La guía contradice a la herramienta.** Teclas de `ros2 bag play`, nombres de archivo y
   patrones regex que ya no corresponden a Jazzy (4.6).

Cada subsección da el síntoma exacto, la causa medida y cómo detectarlo. La 4.7 resume el
método que evita repetir estos fallos al escribir o actualizar una guía.

| Síntoma | Sección |
| :--- | :--- |
| Ves nodos o tópicos que no lanzaste, o varios publicadores en un tópico de tu práctica | [4.1](#41-qué-dominio-usa-cada-taller) |
| `ros2 param set <nodo> log_level DEBUG` no cambia nada | [4.2](#42-ros2-param-set--log_level-no-es-estándar-y-su-fallo-sale-con-código-0) |
| `ros2 bag record` no se detiene desde un script, o el bag queda sin `metadata.yaml` | [4.3](#43-la-grabación-no-se-detiene-o-queda-sin-metadatos) |
| El análisis de un bag comprimido no abre, o da métricas en cero | [4.4](#44-bags-comprimidos-leídos-con-el-lector-equivocado) |
| La mesa sale roja o invisible en RViz en un equipo recién clonado | [4.5](#45-mallas-que-sólo-existen-en-un-install-antiguo) |
| Teclas de `bag play`, regex o volumen de logs que no coinciden con la guía | [4.6](#46-desajustes-menores-con-jazzy) |
| El monitor de red o `ros2 topic bw` dan un ancho de banda que no cuadra | [4.8](#48-mediciones-de-red-que-no-miden-lo-que-parecen) |

### 4.1 Qué dominio usa cada taller

La convención de la sección 2.0 —`ROS_DOMAIN_ID=0` compartido, **una** estación anfitriona
conectada por Ethernet al router que ejecuta el driver, y las demás con `start_driver:=false`—
existe para que nunca haya dos drivers compitiendo por el robot. Por eso aplica **sólo a los
talleres que usan el driver del robot**. Los que trabajan con emuladores o simuladores no tienen
driver que proteger y siguen la sección 2.4:

| Tipo de taller | Talleres | Dominio | ¿Estación anfitriona? |
| :--- | :--- | :--- | :--- |
| **Usa el driver del robot** (brazo o módulo de visión del Kinova) | Localización AprilTag con la cámara real del Kinova | `0`, compartido con la anfitriona | **Sí**: sólo la anfitriona lanza el driver (§2.0) |
| **Sin driver, con nombres fijos** (emulador o simulador) | CLI (turtlesim), TF2 turtlesim, URDF/TF (`display.launch.py`), rosbag2 (emulador) | Uno distinto por equipo cuando varias estaciones practican a la vez (§2.4) | No aplica |
| **Sin driver, con namespace por robot** | micro-ROS en ESP32, AprilTag en modo simulado | `0` del curso: el namespace de cada carrito (`/burger_car_01`, …) ya evita las colisiones | No aplica |

Por qué los talleres sin driver necesitan su propio dominio en clase: publican en los **mismos
nombres** en todas las estaciones, y varios reproducen nombres del sistema real.

| Taller | Publica sin namespace | Se mezcla con |
| :--- | :--- | :--- |
| rosbag2 (`flight_recorder_telemetry_demo.py`) | `/burger/kinova/*` y sus servicios | los emuladores de las demás estaciones y, en una sesión con robot, el `kinova_monitor` (`/burger/kinova/diagnostics`) |
| URDF/TF (`display.launch.py`) | `/joint_states`, `/tf`, `/robot_description` (salvo con `namespace:=<nombre>`, que los mueve todos bajo `/<nombre>/` y avisa al arrancar si no se usa en el dominio 0) | los visores de las demás estaciones y, en una sesión con robot, el driver |
| TF2 turtlesim, CLI | `/tf`, `/turtle1/cmd_vel` | las demos de las demás estaciones |

Nada de esto da error: las grabaciones mezclan datos de varias estaciones, `tf2_echo` alterna
entre tortugas ajenas y un `trigger_anomaly` lo atienden todos los emuladores. Al cambiar de
dominio, hazlo en todas las terminales y reinicia el daemon (sección 1.6). Se aísla el dominio; el
rango de descubrimiento se deja en `SUBNET`, **nunca** `LOCALHOST`. Al terminar la práctica
simulada, vuelve al dominio `0` del curso antes de un taller con el robot.

Cómo detectarlo antes de grabar o medir:

```bash
ros2 topic info /burger/kinova/joint_states   # Publisher count: 1  → sólo tu emulador
ros2 node list | grep controller_manager      # si aparece, compartes dominio con un driver
```

### 4.2 `ros2 param set … log_level` no es estándar, y su fallo sale con código 0

```text
Setting parameter failed: Invalid access to undeclared parameter(s): []
```

`log_level` **no es un parámetro de ROS 2**. Existe sólo si el nodo lo declara y le asocia un
callback, como `kinova_monitor` ([`TEORIA_LOGGING_ROS2.md`](burger_kinova_reference/docs/TEORIA_LOGGING_ROS2.md)).
El emulador del taller no lo declara, y aun así su mensaje de arranque recomendaba ese comando
(corregido). Dos
agravantes medidos: el comando **devuelve código de salida 0**, así que un script no detecta el
fallo; y la explicación que circuló (`enable_logger_service=False`) es de otro mecanismo, los
servicios `~/set_logger_levels`, que `rclpy` sólo crea con `enable_logger_service=True`.

Caminos que sí funcionan, en orden de preferencia para depurar un nodo concreto:

```bash
# Al arrancar, sólo el logger del nodo:
... --ros-args --log-level <nombre_del_nodo>:=debug

# En caliente, si el nodo habilitó los servicios de logger (level 10 = DEBUG):
ros2 service call /<nodo>/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: '<nodo>', level: 10}]}"
```

Evita `--log-level DEBUG` a secas: sube también `rcl` y `rmw_cyclonedds_cpp`. Medido: ≈ 490
líneas en 5 s, de ellas ≈ 400 internas, frente a ≈ 90 con el logger nombrado.

Relacionado: un *flight recorder* que vuelca su buffer con `get_logger().debug()` responde
`success=True` y no muestra nada si el nodo está en `INFO`. **Relanzarlo en DEBUG no lo arregla**:
el buffer vive en RAM y se pierde con el proceso. Hay que arrancar en DEBUG antes de la falla.

### 4.3 La grabación no se detiene, o queda sin metadatos

`rosbag2` escribe `metadata.yaml` al recibir la señal de parada; sin él, `ros2 bag info` y
`ros2 bag play` rechazan la carpeta. Medido en Jazzy, lanzando `ros2 bag record … &` desde un
script:

| Señal | Resultado |
| :--- | :--- |
| `SIGINT` (`kill -INT`) | **Ignorada**: sigue grabando |
| `SIGTERM` (`kill -TERM`) | Parada limpia, con `metadata.yaml` |
| `SIGINT` con `set -m` activo antes del `&` | Parada limpia |
| `SIGKILL` | `.mcap` sin cerrar; `ros2 bag reindex` falla con `No storage could be initialized` |

La causa no es `rosbag2`: en una shell **no interactiva**, los comandos en segundo plano heredan
`SIGINT` y `SIGQUIT` ignorados cuando no hay control de trabajos (comportamiento documentado de
bash). El aviso `stdin is not a terminal device. Keyboard handling disabled.` sólo informa de que
se desactivaron los atajos de teclado; no explica el fallo.

Dos trampas más, ambas vistas en esta revisión:

- **`pkill -f 'ros2 bag record'` dentro de un script mata la propia shell**, porque su línea de
  comandos contiene el patrón. Guarda el PID.
- El recorder de Jazzy **no** expone un servicio de parada (`rosbag2_interfaces/srv/Stop` es del
  player).

```bash
ros2 bag record -s mcap -o mi_bag --topics /t1 /t2 > record.log 2>&1 &
REC_PID=$!
# ... experimento ...
kill -TERM "$REC_PID"; wait "$REC_PID"
test -f mi_bag/metadata.yaml && echo "bag cerrado correctamente"
```

### 4.4 Bags comprimidos leídos con el lector equivocado

La CLI (`info`, `play`) descomprime sola; la API de Python no. Hay que elegir el lector según los
metadatos, y el error puede ser silencioso:

| Bolsa | `SequentialReader` | `SequentialCompressionReader` |
| :--- | :--- | :--- |
| Sin compresión | ✅ | ❌ `should not be initialized with NONE compression mode` |
| `--compression-mode file` (`.mcap.zstd`) | ❌ `invalid magic bytes in Header: 0x28B52FFD…` | ✅ |
| `--compression-mode message` | ⚠️ **abre y entrega payloads comprimidos** | ✅ |

La fila `message` es la grave: cada deserialización falla, y `read_mcap_telemetry.py` las atrapaba
con `except: pass`, así que reportaba **el conteo de mensajes correcto y jitter `0.00000`**, un
resultado falso con apariencia de sistema nominal. La guía anterior además afirmaba que ese modo
"sigue siendo legible por la API", y proponía descomprimir a mano el `.mcap.zstd`, lo que deja un
`metadata.yaml` que sigue declarando compresión.

Solución aplicada en `scripts/read_mcap_telemetry.py`, la misma que ya usaba
`burger_kinova_reference/scripts/analizar_enlace.py`: leer `compression_mode` en `metadata.yaml`,
usar `SequentialCompressionReader` si no es `NONE`, y **contar y avisar** de los mensajes que no
deserializan en lugar de silenciarlos.

### 4.5 Mallas que sólo existen en un `install/` antiguo

El URDF de `burger_description` referencia 55 veces `package://burger_description/meshes/...`,
pero el `CMakeLists.txt` instalaba `visual` y no `meshes` (un enlace simbólico a `visual/meshes`).
En una compilación limpia `install/.../share/burger_description/meshes` **no existía**. En el equipo
del docente funcionaba porque su `install/` conservaba una carpeta `meshes` de noviembre de 2025.
Corregido instalando `visual/meshes` como `meshes`; verificado con compilación limpia normal y con
`--symlink-install`, sin ninguna referencia `package://` sin resolver.

Dos lecciones: un `install/` viejo **oculta** errores de empaquetado, así que valida desde cero
(`--build-base` e `--install-base` en un directorio temporal); y el taller pedía crear el enlace
con `ln -s visual/meshes meshes` aunque ya viene versionado. Sobre un enlace existente, `ln` **no
falla**: crea otro enlace roto dentro del directorio (`visual/meshes/meshes`).

### 4.6 Desajustes menores con Jazzy

| Guía decía | Realidad medida en Jazzy |
| :--- | :--- |
| En `bag play`, `s` avanza un mensaje y `+`/`-` cambian la velocidad | `Flecha Derecha` avanza; `Flecha Arriba`/`Abajo` cambian ±10 %. El player lo anuncia al arrancar (`Press CURSOR_RIGHT for Play Next Message`) |
| El archivo comprimido se llama `ds_mcap_zstd_0.mcap.zstd` | `<carpeta de -o>_0.mcap.zstd` |
| `-e "/burger/kinova/.*"` graba todos los brazos | Sólo `/burger/kinova/`; para `/burger/kinova_2/` hace falta `-e "/burger/kinova[^/]*/.*"` |
| Pedir un tópico que nadie publica hace fallar la grabación | La grabación sigue y lo omite; sólo `ros2 bag info` lo revela |
| Tópicos posicionales en `ros2 bag record` | Deprecados; usa `--topics` |

### 4.7 Cómo validar una guía antes de entregarla

La revisión dejó un método que evita repetir esto:

1. **Ejecutar, no leer.** Cada bloque de comandos, en el orden de la guía. Si el taller no usa
   el driver, en un dominio propio para no mezclarse con otras estaciones (§4.1).
2. **Comprobar el efecto, no el código de salida.** `param set` y `ln -s` salen con 0 fallando;
   `bag record` no avisa de tópicos ausentes; el lector de bags imprimía métricas plausibles.
   Verifica el artefacto: que exista `metadata.yaml`, el `Publisher count`, los conteos de
   `ros2 bag info`, que el valor medido tenga el orden de magnitud esperado.
3. **Contrastar con lo que la herramienta anuncia.** El player imprime sus teclas; el nodo imprime
   su nivel y su dominio. Si la guía contradice esa salida, la guía está mal.
4. **Compilar desde cero** antes de afirmar que un paso de instalación basta.
5. **Buscar el mismo error en el resto del repositorio**: las teclas de `bag play` y el lector sin
   descompresión estaban también en `TEORIA_LOGGING_ROS2.md`.

### 4.8 Mediciones de red que no miden lo que parecen

Al repetir el Laboratorio 02 sobre el robot real (2026-09-15, estación por WiFi, WSL en modo
`mirrored`) tres herramientas dieron cifras plausibles pero falsas. Se contrastaron contra los
contadores de **cada interfaz** (`psutil.net_io_counters(pernic=True)`):

| Herramienta | Qué reportó | Qué ocurría realmente | Por qué |
| :--- | :--- | :--- | :--- |
| Monitor de red: tráfico total, recibido y enviado | Driver de visión más un suscriptor local de la imagen comprimida: **143 Mbps recibidos y 60 Mbps enviados** | Por el WiFi (`eth1`): **83.1 Mbps recibidos y 0 enviados**. Los otros ≈ 59 Mbps en cada sentido iban por `lo` | `psutil.net_io_counters()` suma **todas** las interfaces, incluido el loopback (`lo`, y `loopback0` en WSL `mirrored`). El DDS entre nodos de la misma PC cuenta como tráfico de red |
| Monitor de red: reparto TCP / UDP / DDS | Driver encendido y sin suscriptores: **65 Mbps "DDS"** | No salía DDS por la red (TX = 0). Eran los streams RTP de la cámara llegando por WiFi | El reparto no se mide: se aplican proporciones fijas según haya sockets DDS o micro-ROS ([`MONITOR_RED_CONTROLES_Y_CONFIGURACION.md`](network_setup/MONITOR_RED_CONTROLES_Y_CONFIGURACION.md)) |
| `ros2 topic bw /camera/color/image_raw` | **1.88 MB/s** (menos que el comprimido a calidad 80) | Un suscriptor fiable recibió **26 imágenes/s de 6.22 MB ≈ 162 MB/s**; `ros2 topic hz` sobre el mismo tópico ni siquiera llegó a reportar | `bw` se suscribe siempre con `qos_profile_sensor_data` (*best effort*), sin opción para cambiarlo. Con `FragmentSize` de 1344 B, cada imagen cruda son miles de fragmentos, y perder uno descarta la imagen entera |

**Cómo medir bien:**

```bash
# Tráfico real de UNA interfaz (sustituye eth1 por la NIC conectada a la red ros2)
IF=eth1; R1=$(cat /sys/class/net/$IF/statistics/rx_bytes); sleep 10; \
R2=$(cat /sys/class/net/$IF/statistics/rx_bytes); echo "RX: $(( (R2-R1)*8/10/1000000 )) Mbps"
```

- **Ancho de banda del video crudo:** calcúlalo en lugar de medirlo con `bw`:
  ancho × alto × 3 bytes × FPS. Para 1920 × 1080 RGB8 son 6.22 MB por imagen; la frecuencia
  tómala de `ros2 topic hz` sobre el tópico **comprimido**, que sale de la misma cámara.
- **Tópicos comprimidos:** `ros2 topic bw` sí es utilizable, porque cada imagen pesa unos cientos
  de KB. Contrasta su media por mensaje con `hz`.
- **Monitor de red:** úsalo para el RTT, el jitter y la pérdida hacia el router y el robot,
  que sí son mediciones (`ping`). No uses sus Mbps como tráfico de WiFi cuando hay nodos
  comunicándose dentro de la misma PC.
- Si el puerto 8080 está ocupado (en WSL `mirrored` puede estarlo del lado de Windows), el
  monitor usa el siguiente libre y lo anuncia al arrancar (`Interfaz Web disponible en:
  http://localhost:8081`).

**Valores de referencia medidos** (driver `kinova_vision` en una estación por WiFi, 60 s por
escenario):

| Escenario | WiFi recibido (`eth1`) | RTT al router, medio / máx | Jitter | Pérdida |
| :--- | ---: | :--- | ---: | ---: |
| Driver apagado | 0 Mbps | 6.6 / 23 ms | 1.6 ms | 0 % |
| Driver encendido, color y profundidad | **83.5 Mbps** | 11.1 / 61 ms | 4.5 ms | 0 % |
| Driver sólo color (`launch_depth:=false`) | **20.9 Mbps** | — | — | — |
| Driver más un suscriptor local de la imagen comprimida | 83.1 Mbps | 13.9 / 101 ms | 6.7 ms | 0 % |

La profundidad viaja **sin comprimir** (≈ 62.6 Mbps de los 83.5) y la imagen comprimida con la
calidad JPEG por defecto (95) pesa ≈ 295 KB, unos **61 Mbps por cada suscriptor remoto**. Son
dos razones cuantitativas para la convención de la sección 2.0: la anfitriona por Ethernet, y
el menor número posible de estaciones suscritas a la imagen. Los CSV del monitor de estas
pruebas están en `network_setup/monitor_red/benchmark_logs/` (`lab02_A/B/C`).

## Fuentes técnicas

- [Documentación oficial de ROS 2: servicio de descubrimiento en segundo plano](https://github.com/ros2/ros2_documentation/blob/rolling/source/Developer-Tools/Introspection-and-analysis/About-Command-Line-Tools.rst)
- [Documentación oficial de ROS 2: cambio entre implementaciones RMW](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html)
- [Código fuente oficial de `ros2cli`: inicio y detención del daemon](https://github.com/ros2/ros2cli/blob/rolling/ros2cli/ros2cli/node/daemon.py)
- [Reporte oficial `ros2cli` #934: timeout del daemon en WSL2 con red reflejada](https://github.com/ros2/ros2cli/issues/934)
- [Reporte oficial `ros2cli` #702: `daemon stop` no responde y recuperación con `killall`](https://github.com/ros2/ros2cli/issues/702)
- [Microsoft Learn: arquitectura de red NAT y modo reflejado de WSL](https://learn.microsoft.com/en-us/windows/wsl/networking)
- [Microsoft Learn: reinicio de WSL con `wsl --shutdown`](https://learn.microsoft.com/en-us/windows/wsl/basic-commands#shutdown)
- [Documentación oficial de ROS 2: `ROS_DOMAIN_ID` y aislamiento del grafo](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html)
- Medición propia del enlace sobre el robot real: [`EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md`](burger_kinova_reference/docs/EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md)
- Regla de unicidad del driver y arquitectura distribuida: [`PROYECTO_CORTE_1_CONEXION_KINOVA.md`](education/proyectos_evaluables/PROYECTO_CORTE_1_CONEXION_KINOVA.md) §5
- [GNU Bash Reference Manual: señales en comandos asíncronos sin control de trabajos](https://www.gnu.org/software/bash/manual/html_node/Signals.html)
- Revisión de talleres ejecutada sobre ROS 2 Jazzy (2026-09-15): [`TALLER_ROSBAG_LOGGING_DEBUGGING.md`](education/talleres/TALLER_ROSBAG_LOGGING_DEBUGGING.md), §4 de esta guía
