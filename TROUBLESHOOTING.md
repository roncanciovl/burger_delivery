# Troubleshooting: bloqueo del daemon de ROS 2 en WSL

Esta guía resuelve un fallo recurrente en WSL/WSL2: comandos de introspección como `ros2 node list`, `ros2 topic list`, `ros2 service list` o `ros2 daemon stop` quedan bloqueados, tardan minutos o terminan con errores como `TimeoutError`, `ConnectionRefusedError` o `RemoteDisconnected`.

## Recuperación rápida

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

## Qué se está bloqueando

El daemon de ROS 2 no es un `roscore` ni un servidor DDS central. Es un proceso que mantiene información del grafo ROS para que las consultas de la CLI respondan con rapidez. Se inicia automáticamente cuando una herramienta de introspección lo necesita y atiende a la CLI mediante una conexión local en `127.0.0.1`.

Esto separa dos rutas que conviene diagnosticar:

```text
ros2 node/topic list ── XML-RPC local ── daemon ── RMW/DDS ── grafo ROS 2
```

- Si falla la conexión local entre la CLI y el daemon, los comandos pueden bloquearse aunque los nodos sigan publicando y recibiendo datos.
- Si la conexión local funciona pero DDS no descubre otros participantes, la CLI responde, aunque su lista puede estar incompleta o vacía.

Cada daemon queda asociado al `ROS_DOMAIN_ID` y al RMW activos al iniciarse. ROS 2 utiliza instancias distintas para dominios distintos. Por eso el comando que detiene el daemon debe ejecutarse con el mismo `ROS_DOMAIN_ID`; además, después de cambiar `RMW_IMPLEMENTATION` o un perfil DDS se debe detener la instancia anterior antes de consultar otra vez el grafo.

## Por qué aparece de forma recurrente en WSL

Existe un reporte reproducible del proyecto `ros2cli` con ROS 2 Humble, Ubuntu 22.04 en WSL2 y `networkingMode=mirrored`: una consulta que necesita el daemon espera cerca de dos minutos y termina con `TimeoutError` al intentar conectarse al servicio local. El caso demuestra que una falla del camino local CLI-daemon puede coexistir con una red DDS que sí funciona; no implica que todas las instalaciones WSL tengan el defecto.

En la práctica, revisa el daemon después de cualquiera de estos cambios:

- WSL fue suspendido, apagado o reanudado.
- Windows cambió de Wi-Fi, VPN o interfaz de red.
- Se alternó entre red NAT y modo reflejado.
- Cambió `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, `ROS_DISCOVERY_SERVER` o un perfil de Fast DDS.
- Diferentes terminales cargaron archivos `setup.bash` o variables incompatibles.

No es correcto atribuir todo bloqueo a una "caché de IP antiguas". El síntoma también puede proceder de la conexión XML-RPC local, de un daemon iniciado con otro entorno o de la capa DDS. El siguiente diagnóstico permite distinguirlos.

## Diagnóstico paso a paso

### 1. Confirma el entorno de la terminal

Antes de reiniciar el daemon, carga la distribución y el workspace que realmente se utilizarán:

```bash
source /opt/ros/<distro>/setup.bash
source ~/ros2_ws/install/setup.bash

printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-0}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-predeterminado}"
printf 'ROS_AUTOMATIC_DISCOVERY_RANGE=%s\n' "${ROS_AUTOMATIC_DISCOVERY_RANGE:-predeterminado}"
```

Sustituye `<distro>` por la distribución instalada, por ejemplo `humble` o `jazzy`. En este proyecto, las máquinas participantes deben compartir el dominio configurado —actualmente `ROS_DOMAIN_ID=42`— y utilizar una configuración RMW compatible.

### 2. Compara la consulta con y sin daemon

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

### 3. Detén la instancia correcta

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

### 4. Inicia y valida una instancia limpia

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

## Si `kill` no basta en WSL

Si ya no existe `_ros2_daemon`, pero la CLI continúa agotando el tiempo al conectarse por localhost, reinicia completamente la VM de WSL. Guarda primero cualquier trabajo abierto y ejecuta en PowerShell:

```powershell
wsl --shutdown
```

Abre WSL de nuevo, carga `/opt/ros/<distro>/setup.bash` y el workspace, confirma las variables y repite la validación. Microsoft documenta que WSL2 usa NAT de forma predeterminada y que el modo reflejado cambia la arquitectura de red; reiniciar WSL es necesario después de modificar `.wslconfig`.

## Prevención

- Detén el daemon antes de cambiar `ROS_DOMAIN_ID`, RMW o perfiles DDS.
- Carga el mismo archivo de configuración ROS 2 en todas las terminales del mismo equipo.
- Después de cambiar la red de Windows o reanudar WSL, reinicia el daemon antes de diagnosticar DDS.
- Usa `timeout` en los comandos de diagnóstico para conservar el control de la terminal.
- Mantén WSL y los paquetes de la distribución ROS 2 actualizados dentro de la versión compatible con el proyecto.
- Usa `--no-daemon` para aislar la falla, no como sustituto permanente de una configuración coherente.

## Criterio de resolución

La persona que realiza el diagnóstico puede distinguir el servicio local del daemon de la comunicación DDS, identificar el PID y su dominio/RMW antes de terminarlo, recuperar la CLI sin reiniciar todo Windows y demostrar el resultado con un tópico observable.

## Fuentes técnicas

- [Documentación oficial de ROS 2: servicio de descubrimiento en segundo plano](https://github.com/ros2/ros2_documentation/blob/rolling/source/Developer-Tools/Introspection-and-analysis/About-Command-Line-Tools.rst)
- [Documentación oficial de ROS 2: cambio entre implementaciones RMW](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html)
- [Código fuente oficial de `ros2cli`: inicio y detención del daemon](https://github.com/ros2/ros2cli/blob/rolling/ros2cli/ros2cli/node/daemon.py)
- [Reporte oficial `ros2cli` #934: timeout del daemon en WSL2 con red reflejada](https://github.com/ros2/ros2cli/issues/934)
- [Reporte oficial `ros2cli` #702: `daemon stop` no responde y recuperación con `killall`](https://github.com/ros2/ros2cli/issues/702)
- [Microsoft Learn: arquitectura de red NAT y modo reflejado de WSL](https://learn.microsoft.com/en-us/windows/wsl/networking)
- [Microsoft Learn: reinicio de WSL con `wsl --shutdown`](https://learn.microsoft.com/en-us/windows/wsl/basic-commands#shutdown)
