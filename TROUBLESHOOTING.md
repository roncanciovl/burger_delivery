# Troubleshooting del entorno ROS 2 del proyecto

Esta guía cubre los dos fallos que más tiempo consumen en el laboratorio:

| Síntoma | Sección |
| :--- | :--- |
| `ros2 node list`, `topic list` o `param set` se bloquean o terminan en `TimeoutError` | [1. Bloqueo del daemon de la CLI](#1-bloqueo-del-daemon-de-ros-2-en-wsl) |
| El driver no arranca, o hay telemetría duplicada, porque **el robot ya está en uso** | [2. Hardware ocupado: un solo robot, varias estaciones](#2-hardware-ocupado-un-solo-robot-y-varias-estaciones) |
| No sé quién tiene el robot ocupado | [2.0 La configuración recomendada](#20-la-configuración-recomendada-léela-antes-que-el-resto) |

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

Como no hay comando que lo responda, la respuesta tiene que ser una **convención**:

> ### 📌 Convención del laboratorio
>
> 1. **Un único computador ejecuta el driver.** Siempre el mismo, designado y anotado
>    junto al robot. Es la *estación anfitriona*. Hospeda todos los drivers que hablan con
>    el hardware: el brazo (`kortex_bringup`) y el de visión, que comparten la misma IP.
> 2. **Esa estación va por cable Ethernet.** Por WiFi la sesión de control se rompe; está
>    medido, no es una preferencia (sección 2.3, opción B).
> 3. **Todos los demás usan el mismo `ROS_DOMAIN_ID`** que la estación anfitriona, y
>    lanzan con `start_driver:=false`. Pueden ir por WiFi.

Esta convención es la que **hace desaparecer el cuello de botella**, y conviene ver por
qué: si todos comparten el dominio, *"¿quién tiene el robot?"* deja de ser una pregunta
incontestable y pasa a resolverse con un comando:

```bash
timeout 15s ros2 node list | grep controller_manager
```

Si aparece, el driver está corriendo y ya sabes que debes conectarte a él en lugar de
lanzar el tuyo. Si cada equipo usa un dominio distinto, esa misma consulta devuelve vacío
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
                  ▲ UNA sola sesión de control en tiempo real
                  │ Esta capa NO sabe qué es ROS_DOMAIN_ID

  Estación A ◄─── DDS (aislado por ROS_DOMAIN_ID) ──► Estación B
                  Aquí sí manda el dominio
```

De ahí la regla que más confusión causa:

> ⚠️ **Cambiar tu `ROS_DOMAIN_ID` NO libera el robot ni te habilita a lanzar tu propio
> driver.** El dominio aísla el grafo ROS 2, no la sesión TCP con la controladora. Si otra
> estación ya tiene la sesión, tu driver fallará igual —o peor: las dos competirán por
> ella, provocando desconexiones por *timeout* de heartbeat y paradas de seguridad en el
> robot.

El aislamiento por dominio sólo sirve para el caso **simulado** (`use_fake_hardware:=true`),
donde no hay robot físico que compartir.

Son, por tanto, dos síntomas distintos con causas distintas:

| Síntoma | Capa | Qué significa |
| :--- | :--- | :--- |
| El driver no arranca; la conexión al robot no progresa | Kortex TCP/UDP | Alguien más tiene la sesión, o el robot no es alcanzable |
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
> hostname ni una IP. Para saber *quién* lo tiene, la vía es la convención de la sección
> 2.0 —una estación anfitriona designada y anotada— o preguntar. Cada persona sí puede
> comprobar **su propia** máquina con el `ss` del paso 1.

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
  [`EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md`](burger_kinova_connection/docs/EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md).
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
- Antes de lanzar con `start_driver:=true`, comprueba en un solo paso que nadie lo tiene:
  ```bash
  ss -tanp | grep 192.168.1.10; timeout 15s ros2 node list | grep controller_manager
  ```
  Si ambas salidas están vacías, el camino está libre desde tu punto de vista.
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

## Fuentes técnicas

- [Documentación oficial de ROS 2: servicio de descubrimiento en segundo plano](https://github.com/ros2/ros2_documentation/blob/rolling/source/Developer-Tools/Introspection-and-analysis/About-Command-Line-Tools.rst)
- [Documentación oficial de ROS 2: cambio entre implementaciones RMW](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html)
- [Código fuente oficial de `ros2cli`: inicio y detención del daemon](https://github.com/ros2/ros2cli/blob/rolling/ros2cli/ros2cli/node/daemon.py)
- [Reporte oficial `ros2cli` #934: timeout del daemon en WSL2 con red reflejada](https://github.com/ros2/ros2cli/issues/934)
- [Reporte oficial `ros2cli` #702: `daemon stop` no responde y recuperación con `killall`](https://github.com/ros2/ros2cli/issues/702)
- [Microsoft Learn: arquitectura de red NAT y modo reflejado de WSL](https://learn.microsoft.com/en-us/windows/wsl/networking)
- [Microsoft Learn: reinicio de WSL con `wsl --shutdown`](https://learn.microsoft.com/en-us/windows/wsl/basic-commands#shutdown)
- [Documentación oficial de ROS 2: `ROS_DOMAIN_ID` y aislamiento del grafo](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html)
- Medición propia del enlace sobre el robot real: [`EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md`](burger_kinova_connection/docs/EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md)
- Regla de unicidad del driver y arquitectura distribuida: [`PROYECTO_CORTE_1_CONEXION_KINOVA.md`](education/proyectos_evaluables/PROYECTO_CORTE_1_CONEXION_KINOVA.md) §5
