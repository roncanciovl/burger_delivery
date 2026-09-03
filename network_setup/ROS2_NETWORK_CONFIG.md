# Guía de Análisis y Configuración de Red en ROS 2

Este documento proporciona un camino lógico, paso a paso, para configurar, comprender y solucionar problemas en tu red de ROS 2, especialmente en entornos mixtos (Linux, Windows, WSL).

---

## 0. Puesta en marcha de una máquina nueva

Los cinco pasos que hacen falta para que un computador recién instalado hable con el resto. Ninguno depende del hardware, del nombre de la interfaz ni de la IP concreta del equipo.

| # | Paso | Dónde |
|---|---|---|
| 1 | Conectar el equipo a la red del router ROS (SSID `ros2`) y confirmar que la IP quedó en la subred ROS | `ip -brief addr` · [§6.2](#62-política-recomendada-para-la-red-del-laboratorio) |
| 2 | Solo en WSL: activar `networkingMode=mirrored` y verificarlo | [§2](#2-configuración-del-host-resolviendo-el-aislamiento-de-red-en-wsl) |
| 3 | Exportar las cuatro variables de entorno en `~/.bashrc` | [§3](#3-aplicando-configuraciones-de-entorno) |
| 4 | Abrir el firewall a UDP desde la subred ROS (UFW en Linux; dos reglas en Windows + Hyper-V) | [§6](#6-configuración-del-firewall-para-una-red-ros-2-distribuida) |
| 5 | Validar contra otro computador: multicast, luego grafo y datos | [§4](#4-verificación-y-pruebas) y [§6.6](#66-validación-funcional-entre-dos-computadores) |

Comprobación de un vistazo, en la máquina nueva:

```bash
bash ~/ros2_ws/src/burger_delivery/network_setup/test_ros2_network.sh
```

El script revisa variables de entorno, perfil DDS, modo de red de WSL, arranque real del middleware y puertos del dominio; termina con código `0` solo si la configuración local sirve para comunicarse con otros equipos. Equivalente mínimo si prefieres hacerlo a mano:

```bash
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_AUTOMATIC_DISCOVERY_RANGE CYCLONEDDS_URI
timeout 15s ros2 topic list --no-daemon
```

Si el último comando responde en segundos, la configuración local es correcta y cualquier problema restante es de red o de firewall. Si falla, el problema es local y se resuelve en la [§3](#3-aplicando-configuraciones-de-entorno) antes de tocar nada más.

**Qué es igual en todas las máquinas:** dominio `0`, `SUBNET`, `rmw_cyclonedds_cpp`, la ruta del perfil `cyclonedds.xml` y los nombres de las reglas de firewall.
**Qué cambia por máquina:** su dirección IP y, únicamente en equipos dual-homed, el nombre de la interfaz fijado en el perfil DDS.

---

## 1. Conceptos Fundamentales de Red

Antes de configurar la red, es importante comprender cómo se comunican los nodos y las tres variables fundamentales que controlan esto en ROS 2:

**El Rol del UDP Multicast en DDS:**
ROS 2 utiliza el estándar DDS (Data Distribution Service) como su capa subyacente. Para que los nodos se encuentren entre sí automáticamente en una red sin necesidad de un servidor central (como el antiguo `roscore`), DDS emplea **UDP Multicast**. Esto significa que cuando un nodo se inicia, "grita" su presencia enviando paquetes a una dirección IP de Multicast dedicada. Todos los demás nodos de ROS 2 en la red "escuchan" en esa dirección, lo que les permite descubrirse mutuamente y establecer conexiones (peer-to-peer). Si tu enrutador (router) o firewall bloquea el tráfico UDP Multicast, los nodos jamás se verán, sin importar cuán bien esté configurado ROS 2.

1. **`ROS_DOMAIN_ID=0`**
   - Crea un "grupo de red" aislado para tus nodos de ROS.
   - Solo los nodos con el **mismo ID de dominio** pueden comunicarse entre sí.
   - **Este proyecto usa el dominio `0`**: es el dominio de pruebas del docente y, además, el valor por defecto de ROS 2, de modo que cualquier equipo recién instalado queda en el dominio correcto sin configurar nada.
   - Rango seguro en Linux: `0-101`. El protocolo admite hasta `232` (es el máximo que observa el monitor de red), pero por encima de `101` el bloque de puertos RTPS puede solaparse con el rango efímero del sistema.
   - **Aislamiento por grupos:** si el docente asigna un dominio distinto a tu grupo para evitar interferencias, expórtalo en **todas** las máquinas del grupo. Un solo equipo con un dominio diferente queda invisible para el resto, aunque la red física funcione.

2. **`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`**
   - `SUBNET` = Descubre cualquier nodo alcanzable a través de Multicast (por defecto).
   - `LOCALHOST` = Solo descubre nodos en la misma máquina física.
   - `OFF` = No descubre ningún otro nodo.
   - *(Esta variable reemplaza a la antigua y obsoleta `ROS_LOCALHOST_ONLY`)*

3. **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`**
   - **¿Por qué usarlo?** CycloneDDS es significativamente más estable en entornos Wi-Fi. Genera menos "ruido" durante el descubrimiento de nodos y maneja la pérdida de paquetes y la latencia mucho mejor que Fast DDS (el predeterminado).
   - **Verificación:** Ejecuta `printenv RMW_IMPLEMENTATION`. Debería retornar `rmw_cyclonedds_cpp`.

---

## 2. Configuración del Host: Resolviendo el Aislamiento de Red en WSL

Si estás utilizando Windows Subsystem for Linux (WSL), por defecto utiliza una red virtual (NAT) que oculta tus nodos de ROS del resto de la red física. Debes solucionar esto primero.

### Opción A: Modo Espejo (Mirrored Mode) (Recomendado ⭐)
Este modo hace que WSL comparta la misma dirección IP que tu host de Windows, haciendo que todos los nodos de ROS sean visibles instantáneamente en la red física.

1. Abre el Explorador de Windows y ve a `%USERPROFILE%` (ej. `C:\Users\TuUsuario`).
2. Crea o edita un archivo llamado `.wslconfig`.
3. Añade las siguientes líneas:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
4. Reinicia WSL desde PowerShell:
   ```powershell
   wsl --shutdown
   ```
5. Verifica en WSL: El comando `ip addr` ahora debería mostrar tu IP de Windows.

### Opción B: Si no puedes usar el Modo Espejo

> [!WARNING]
> `netsh interface portproxy` **no es una alternativa válida aquí**: solo reenvía **TCP**, mientras que DDS y el agente micro-ROS (`udp4 --port 8888`) usan **UDP**. Tampoco reenvía el multicast que DDS necesita para el descubrimiento. Un portproxy no hará visible tu WSL en la red ROS.

En modo NAT (el predeterminado de WSL2) las opciones reales son:

1. **Preferida:** habilitar `networkingMode=mirrored` (Opción A). Requiere Windows 11 22H2 o superior; comprueba tu versión con `winver`.
2. **Ejecutar ROS 2 en Ubuntu nativo** o en una máquina virtual con adaptador en modo puente.
3. **Discovery Server unicast** (Sección 7) más `Peers` explícitos: evita depender del multicast, pero sigue necesitando que la IP de WSL sea alcanzable desde la subred ROS, lo que NAT no garantiza.

Verifica en qué modo estás antes de continuar:
```bash
wslinfo --networking-mode   # resultado esperado: mirrored
```

---

## 3. Aplicando Configuraciones de Entorno

Una vez que la red del host es accesible, aplica las variables principales a todas las PCs que participarán.

Estas cuatro líneas son **idénticas en todas las máquinas** del proyecto. No contienen nombres de interfaz ni direcciones IP, por lo que funcionan igual en Ubuntu nativo, en WSL2 reflejado y en equipos con una o varias NIC.

### En Linux / WSL (Ubuntu)
Añade al archivo `~/.bashrc`, **después** de la línea `source /opt/ros/<distro>/setup.bash`:
```bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/ros2_ws/src/burger_delivery/network_setup/cyclonedds.xml
```
Luego ejecuta: `source ~/.bashrc`

### El perfil `cyclonedds.xml`

`CYCLONEDDS_URI` carga [`network_setup/cyclonedds.xml`](cyclonedds.xml), que ajusta el tamaño de mensaje y de fragmento a la MTU de Wi-Fi (evita que la pérdida de un fragmento descarte una imagen completa) y amplía el buffer de escritura.

El perfil **selecciona la interfaz de red automáticamente** (`<NetworkInterface autodetermine="true"/>`), así que no hay que editarlo para cambiar de equipo. Solo existe un caso que exige tocarlo: un computador **dual-homed** —una NIC hacia el Kinova y otra hacia la subred ROS— donde la autodetección puede elegir la NIC equivocada. Ahí se comenta el bloque automático y se fija el nombre real de la interfaz, que obtienes con:

```bash
ip -brief addr
```

> [!WARNING]
> Nunca dejes en el perfil un nombre de interfaz que no exista en la máquina. CycloneDDS aborta la creación del nodo con `<nombre>: does not match an available interface` y **ningún** comando de ROS 2 funcionará hasta corregirlo.

### En Windows (Nativo)
Solo si ejecutas ROS 2 compilado para Windows. Si tus nodos viven dentro de WSL, **no** configures estas variables en Windows: no las lee nadie y crean confusión.
```powershell
[System.Environment]::SetEnvironmentVariable('ROS_DOMAIN_ID', '0', 'User')
[System.Environment]::SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET', 'User')
[System.Environment]::SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp', 'User')
```
Reinicia tu terminal para aplicar los cambios.

### Verificación inmediata (obligatoria en cada máquina nueva)

```bash
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_AUTOMATIC_DISCOVERY_RANGE CYCLONEDDS_URI
test -r "${CYCLONEDDS_URI#file://}" && echo "perfil legible"
timeout 15s ros2 topic list --no-daemon
```

`ros2 topic list --no-daemon` debe responder en segundos con al menos `/parameter_events` y `/rosout`. Si en cambio imprime un error de `rmw_cyclonedds_cpp`, el problema está en el perfil (interfaz inexistente o ruta ilegible), no en la red.

---

## 4. Verificación y Pruebas

Verifica que tus nodos puedan comunicarse a través de la red.

### Diagnósticos Automatizados
- **Lista de Verificación de Red y ROS 2:** `bash test_ros2_network.sh`
- **Estabilidad Wi-Fi:** `bash diagnostico_wifi.sh`

### Prueba Manual 1: Validación de Multicast a Nivel de Red (Muy Recomendado)
Esta prueba utiliza una herramienta nativa (`ros2 multicast`) que envía un paquete UDP Multicast "crudo", evadiendo por completo la capa de DDS. Si falla, el problema radica en tu red física, firewall o router, y no en tus configuraciones del dominio de ROS 2.

> [!IMPORTANT]
> `ros2 multicast` usa el grupo `225.0.0.1` en el puerto `49150`, **no** el grupo `239.255.0.1` ni los puertos `7400 + 250 × ROS_DOMAIN_ID` que emplea DDS. Es una prueba del camino multicast, no del descubrimiento real:
> - Con la política de firewall de la Sección 6 (todo UDP desde la subred ROS) ambos caminos quedan cubiertos y la prueba es representativa.
> - Con una regla acotada a `7400-7500`, esta prueba **falla aunque DDS funcione**, y a la inversa: puede pasar aunque el router filtre el grupo que DDS necesita.
>
> Para forzar el mismo grupo y puerto que usa tu dominio, añade `--group 239.255.0.1 --port <7400 + 250 × ID>` en ambos extremos.

**PC 1 (Receptor):**
```bash
ros2 multicast receive
```
*(Se quedará esperando recibir datos...)*

**PC 2 (Emisor):**
```bash
ros2 multicast send
```
Si la red está permitiendo tráfico multicast, la PC 1 mostrará el mensaje: `Received from <IP_PC2>: 'Hello World!'`.
*Nota: Si esta prueba falla, detente aquí y revisa la Sección 6 (Firewall) o intenta usar el Discovery Server (Sección 7).*

### Prueba Manual 2: Publicador / Suscriptor a Nivel de Nodos
Si la prueba de multicast fue exitosa, ahora verificamos que DDS pueda establecer la comunicación formal de tópicos.

**PC 1 (Publicador):**
```bash
ros2 topic pub /test std_msgs/String "data: 'Hola desde PC1'"
```
**PC 2 (Suscriptor):**
```bash
ros2 topic echo /test
```

---

## 5. Bloqueo del daemon de ROS 2 en WSL

Los comandos `ros2 node list`, `ros2 topic list` y otras consultas de introspección utilizan un daemon local que mantiene información del grafo ROS 2. Este proceso se comunica con la CLI por `127.0.0.1` y se inicia con el `ROS_DOMAIN_ID`, RMW y perfiles DDS activos en ese momento.

En WSL2 se ha reportado un caso reproducible en el que esa conexión local queda esperando hasta terminar en `TimeoutError`. El reporte original de `ros2cli` es sobre ROS 2 Humble con red reflejada; el síntoma y la recuperación son idénticos en Jazzy, la distribución de este proyecto. También aparecen inconsistencias cuando se cambia de dominio, RMW, perfil DDS, Wi-Fi o VPN sin reiniciar el daemon. Esto es distinto de una falla de multicast: el daemon puede estar bloqueado aunque los nodos continúen intercambiando datos.

### Diagnóstico rápido

```bash
timeout 15s ros2 node list
timeout 15s ros2 node list --no-daemon
```

Si solo se bloquea la primera consulta, la ruta local CLI-daemon es la sospecha principal. Si ambas fallan, continúa revisando RMW, DDS, multicast y firewall.

### Recuperación

```bash
# Intento de cierre normal
timeout 5s ros2 daemon stop

# Si no responde, inspecciona el PID exacto
pgrep -af '(_ros2_daemon|--name ros2-daemon)'

# Sustituye <PID>; SIGTERM permite una salida ordenada
kill <PID>

# Solo si el mismo proceso continúa vivo
kill -KILL <PID>

# Inicia con el entorno actual y valida
ros2 daemon start
timeout 5s ros2 daemon status
timeout 15s ros2 node list
```

No uses `sudo` ni empieces con `kill -9`. Verifica en la salida de `pgrep` que el dominio y el RMW sean los esperados antes de terminar el proceso.

La explicación completa, el flujo de decisión, la recuperación de WSL y las fuentes oficiales están en [TROUBLESHOOTING.md](../TROUBLESHOOTING.md).

---

## 6. Configuración del firewall para una red ROS 2 distribuida

### 6.1. Por qué `7400-7500` no cubre todo DDS

La expresión `7400 + (250 × ROS_DOMAIN_ID)` permite ubicar el bloque de puertos RTPS calculados para un dominio. Sin embargo, no describe necesariamente todos los sockets utilizados durante la comunicación. CycloneDDS puede crear puertos unicast asignados dinámicamente por el sistema operativo; otras implementaciones DDS también pueden usar locators o rangos diferentes.

Por tanto, una regla limitada a `7400-7500` puede dejar pasar el anuncio multicast y bloquear después el descubrimiento de endpoints o los datos. No es una restricción que impida usar CycloneDDS de forma distribuida: es una consecuencia del modelo DDSI y de la asignación de puertos por proceso. Consulta la [documentación oficial de puertos de CycloneDDS](https://cyclonedds.io/docs/cyclonedds/latest/config/port_numbers.html).

### 6.2. Política recomendada para la red del laboratorio

**Primero identifica la subred ROS de tu equipo.** La del laboratorio es `192.168.1.0/24` (router TP-Link AX12 en `192.168.1.1`), y es el valor por defecto de todas las herramientas del proyecto. Confirma la tuya con:

```bash
ip -brief addr        # busca la NIC conectada al router ROS, p. ej. 192.168.1.185/24
ip route | grep default
```

Si tu red ROS **no** es `192.168.1.0/24`, sustituye la subred en todos los comandos de esta sección y declárala una vez para que el monitor valide contra ella en lugar de la del laboratorio:

```bash
export ROS_LAN_SUBNET=10.42.0.0/24   # solo si tu red ROS es distinta a la del laboratorio
```

La configuración adoptada por el proyecto es:

1. Permitir **todo UDP entrante hacia WSL**.
2. Aceptarlo **solamente desde la subred ROS de confianza** (`192.168.1.0/24` por defecto).
3. Mantener `DefaultInboundAction = Block` para cualquier otra red.
4. Mantener la salida permitida; no se requieren reglas de salida adicionales en la configuración estándar de Windows/Hyper-V.
5. Aplicar la misma política en cada computador que ejecute nodos ROS 2 dentro de WSL.

Esta excepción cubre multicast, los puertos RTPS calculados y los puertos UDP dinámicos sin exponer WSL a equipos externos a la subred ROS. No habilita TCP ni desactiva el firewall.

> [!WARNING]
> Utiliza esta política únicamente en una LAN o VLAN ROS administrada. Si `192.168.1.0/24` comparte dispositivos no confiables, limita `RemoteAddresses` a las IP concretas de los robots y estaciones autorizadas.

### 6.3. Linux o Ubuntu nativo con UFW

En una máquina Linux conectada exclusivamente a la subred ROS:

```bash
sudo ufw allow from 192.168.1.0/24 proto udp   # sustituye por tu subred ROS
sudo ufw status numbered
```

La regla no permite UDP desde otras subredes. No deshabilites UFW para realizar la práctica.

Si el equipo también expone el monitor de red a otros computadores, añade su puerto TCP:

```bash
sudo ufw allow from 192.168.1.0/24 proto tcp to any port 8080
```

### 6.4. Windows 11 y WSL2 en modo reflejado

En `networkingMode=mirrored`, Windows Defender Firewall y el firewall de Hyper-V son capas distintas. Es necesario crear una excepción en ambas. Microsoft identifica WSL con este `VMCreatorId`:

```powershell
$wslId = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
$rosSubnet = '192.168.1.0/24'   # sustituye por tu subred ROS si es distinta
```

El `VMCreatorId` es el mismo en todas las instalaciones de WSL: no depende de tu equipo ni de la distribución. Lo único que cambia de una máquina a otra es `$rosSubnet`.

Ejecuta una sola vez en PowerShell como administrador:

```powershell
New-NetFirewallHyperVRule `
  -Name 'ROS2-Distributed-LAN-HyperV' `
  -DisplayName 'ROS 2 Distributed LAN WSL' `
  -Direction Inbound `
  -VMCreatorId $wslId `
  -Protocol UDP `
  -RemoteAddresses $rosSubnet `
  -Action Allow

New-NetFirewallRule `
  -Name 'ROS2-Distributed-LAN-Windows' `
  -DisplayName 'ROS 2 Distributed LAN WSL' `
  -Direction Inbound `
  -Protocol UDP `
  -LocalPort Any `
  -RemoteAddress $rosSubnet `
  -Profile Any `
  -Action Allow
```

Al omitir `LocalPorts` en la regla Hyper-V se permiten todos los puertos UDP locales. La seguridad se conserva mediante `VMCreatorId`, el sentido `Inbound`, el protocolo UDP y la subred remota. La documentación oficial de Microsoft describe estas reglas en [red reflejada y firewall de Hyper-V para WSL](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking).

Si la regla ya existe, no la dupliques. Inspecciónala y corrígela con `Set-NetFirewallHyperVRule` o `Set-NetFirewallRule` según corresponda.

#### Auditar reglas ROS anteriores

Una regla antigua con `RemoteAddress = Any` puede anular el aislamiento pretendido, aunque la regla nueva esté correctamente limitada. Revísalas con:

```powershell
Get-NetFirewallRule -PolicyStore ActiveStore |
  Where-Object { $_.DisplayName -match 'ROS.?2|DDS' -and $_.Direction -eq 'Inbound' } |
  ForEach-Object {
    $rule = $_
    $port = $rule | Get-NetFirewallPortFilter
    $addr = $rule | Get-NetFirewallAddressFilter
    [PSCustomObject]@{
      Name          = $rule.Name
      DisplayName   = $rule.DisplayName
      Enabled       = $rule.Enabled
      Protocol      = $port.Protocol
      LocalPort     = $port.LocalPort
      RemoteAddress = $addr.RemoteAddress
    }
  } | Format-Table -AutoSize
```

Corrige o deshabilita únicamente reglas ROS creadas previamente cuyo origen sea `Any`. No modifiques reglas del sistema o de otras aplicaciones sin evaluar su función.

### 6.5. Verificación obligatoria

Dentro de WSL:

```bash
wslinfo --networking-mode
```

Resultado esperado: `mirrored`.

En PowerShell:

```powershell
$wslId = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'

# Perfil Hyper-V de WSL (filtrado por VMCreatorId, como lo evalúa el monitor)
Get-NetFirewallHyperVProfile -PolicyStore ActiveStore |
  Where-Object { $_.Name -eq $wslId } |
  Select-Object Name, Enabled, DefaultInboundAction, DefaultOutboundAction

# Perfiles de Windows: los tres deben bloquear entrada y permitir salida
Get-NetFirewallProfile -PolicyStore ActiveStore |
  Select-Object Name, Enabled, DefaultInboundAction, DefaultOutboundAction

Get-NetFirewallHyperVRule -Name 'ROS2-Distributed-LAN-HyperV' |
  Select-Object Name, Enabled, Direction, Action, Protocol, LocalPorts, RemoteAddresses |
  Format-List

$rule = Get-NetFirewallRule -Name 'ROS2-Distributed-LAN-Windows'
$rule | Select-Object Name, Enabled, Direction, Action, Profile
$rule | Get-NetFirewallPortFilter | Select-Object Protocol, LocalPort
$rule | Get-NetFirewallAddressFilter | Select-Object RemoteAddress
```

El criterio de aceptación —el mismo que aplica el monitor de red— es:

| Capa | Entrada predeterminada | Salida predeterminada | Excepción ROS 2 | Origen permitido |
|---|:---:|:---:|:---:|:---:|
| Hyper-V / WSL (perfil `$wslId`) | `Block` | `Allow` | `Inbound`, `Allow`, UDP, puertos `Any` | subred ROS |
| Windows (los tres perfiles) | `Block` | `Allow` | `Inbound`, `Allow`, UDP, puertos `Any`, perfil `Any` | subred ROS |

Además, **ninguna** regla ROS/DDS heredada puede quedar habilitada con origen remoto `Any`: una sola de ellas anula el aislamiento aunque las dos reglas nuevas estén correctas (ver «Auditar reglas ROS anteriores»).

Cada fila corresponde a un control del panel del monitor: `bloqueo entrante predeterminado de Hyper-V`, `salida predeterminada de Hyper-V`, `políticas predeterminadas de Windows`, `regla Hyper-V`, `regla Windows` y `reglas ROS/DDS heredadas`. Si el panel indica un control pendiente, es exactamente la fila que falta aquí.

### 6.6. Validación funcional entre dos computadores

Primero valida el transporte multicast. En el PC receptor:

```bash
ros2 multicast receive
```

En el PC emisor:

```bash
ros2 multicast send
```

Después valida el grafo y los datos DDS con un `talker` remoto activo:

```bash
ros2 topic info /chatter --verbose --no-daemon
ros2 topic echo /chatter std_msgs/msg/String --once
```

El criterio de aceptación es observar `Publisher count: 1` y recibir una muestra. Si multicast funciona pero el publicador continúa en `0`, verifica en ambos computadores:

```bash
printenv ROS_DOMAIN_ID
printenv RMW_IMPLEMENTATION
printenv ROS_AUTOMATIC_DISCOVERY_RANGE
printenv ROS_LOCALHOST_ONLY
printenv CYCLONEDDS_URI
```

Los equipos deben usar el mismo dominio (`0` salvo asignación distinta del docente), `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` y, durante el diagnóstico, la misma implementación RMW. La prueba `ros2 multicast` utiliza un datagrama de prueba en otro grupo y puerto (ver Sección 4) y no demuestra por sí sola que el proceso DDS esté anunciándose correctamente.

Recorre estas causas en orden; son las que explican casi todos los `Publisher count: 0` con multicast funcionando:

| Síntoma | Causa probable | Comprobación |
|---|---|---|
| `ros2 topic list` falla con `rmw_cyclonedds_cpp` | Interfaz inexistente en `cyclonedds.xml` | Sección 3, `ip -brief addr` |
| Un equipo no ve a nadie, el resto sí | `ROS_DOMAIN_ID` distinto | `printenv ROS_DOMAIN_ID` en ambos |
| Solo se ven los nodos locales | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` o `ROS_LOCALHOST_ONLY=1` heredada | `printenv` de ambas variables |
| Se ven los nodos pero no llegan datos | RMW distinto entre equipos | `printenv RMW_IMPLEMENTATION` |
| Nada llega desde un solo equipo | Firewall de ese equipo | Sección 6.5 o panel del monitor |
| La IP cambió de subred | El Wi-Fi saltó a la red institucional | Sección 8.B |
| La CLI se cuelga pero los nodos publican | Daemon bloqueado | Sección 5 |

---

## 7. Avanzado: Servidor de Descubrimiento (Fast DDS Discovery Server)

Si tienes problemas persistentes con el **multicast** (común en redes corporativas o de universidades), puedes utilizar el *Discovery Server*. Esto reemplaza el sistema automático en el que los nodos "gritan" su existencia por toda la red, centralizándolo en una "agenda telefónica" única (el Servidor).

### 1. El Archivo de Configuración (`fastdds_discovery.xml`)
Crea un archivo XML que configure tus nodos de ROS para que actúen como **Super Clientes**. En lugar de buscar por toda la red usando multicast, se conectarán directamente a la IP estática del Discovery Server.

> [!IMPORTANT]
> Es un modo **alternativo y excluyente**, no un añadido: usa Fast DDS, así que todas las máquinas deben cambiar de RMW a la vez. Un equipo en CycloneDDS y otro en Fast DDS no se comunican nunca, aunque compartan dominio, red y firewall. Después de cambiar de RMW o de perfil, detén el daemon en cada equipo (`ros2 daemon stop`, Sección 5) antes de volver a consultar el grafo.

### 2. Cómo Utilizarlo

**A. Iniciar el Servidor (solo en la PC principal, con IP fija o reservada por DHCP):**
```bash
fastdds discovery --server-id 0 --ip-address <IP_DEL_SERVIDOR> --port 11811
```
En el laboratorio esa IP es `192.168.1.100`. Si usas otra, **debe** coincidir con la del `<address>` en [`fastdds_discovery.xml`](fastdds_discovery.xml); son dos lugares distintos y desincronizarlos es el error más habitual de esta sección.

**B. Configurar los Clientes (en todas las PCs, incluida la del servidor):**
Añade estas variables a tu `~/.bashrc`, y comenta las de CycloneDDS:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/ros2_ws/src/burger_delivery/network_setup/fastdds_discovery.xml
```
El perfil declara `SUPER_CLIENT`, lo que mantiene funcional la introspección (`ros2 node list`, `ros2 topic list`) además del intercambio de datos.

> [!WARNING]
> La alternativa corta `export ROS_DISCOVERY_SERVER="192.168.1.100:11811"` crea un participante `CLIENT`, no `SUPER_CLIENT`: los datos fluyen, pero `ros2 topic list` y `ros2 node list` muestran listas incompletas o vacías. Si la usas, añade también `export ROS_SUPER_CLIENT=true`.

---

## 8. Diagnóstico y Diagnósticos con el Monitor de Red (`monitor_red`)

El proyecto incluye un monitor web interactivo (`network_setup/iniciar_monitor.sh`) diseñado para visualizar en tiempo real la topología de la red, los nodos activos en el `ROS_DOMAIN_ID` y las métricas de calidad de enlace.

### A. Observación de dominios RTPS sin privilegios de administrador
Para asociar una IP con un `ROS_DOMAIN_ID` sin ejecutar el monitor con `sudo`:
- El monitor escucha pasivamente los anuncios SPDP multicast y valida que contengan una cabecera RTPS.
- Atribuye el dominio usando el puerto multicast estándar:
  $$\text{Puerto Base} = 7400 + (250 \times \text{ROS\_DOMAIN\_ID})$$
- Por ejemplo, el anuncio SPDP del **Dominio 0** —el del proyecto— llega al puerto `7400`; el del **Dominio 15** al `11150` y el del **Dominio 42** al `17900`.

El dominio configurado localmente no se presenta como actividad. Para computadores remotos, si no llega un anuncio RTPS válido, la interfaz muestra **Domain desconocido**. El uso exclusivo de Discovery Server, descubrimiento unicast, `LOCALHOST`, aislamiento Wi-Fi o filtrado multicast puede impedir la observación remota.

El observador tampoco abre puertos dentro del rango UDP efímero de Linux. En WSL considera además el rango dinámico de Windows, ya que esa capa puede reservar puertos que no aparecen en `ss` dentro de Linux.

### Verificación de la política de firewall desde el monitor

En WSL, el panel **Observación RTPS y servicios locales** consulta cada 30 segundos el estado de Windows Defender Firewall y del firewall de Hyper-V. Debe mostrar **Firewall distribuido verificado** cuando:

- ambas reglas `ROS2-Distributed-LAN-*` están habilitadas para UDP entrante, puertos `Any` y origen igual a la subred ROS esperada (`192.168.1.0/24`, o la que hayas declarado en `ROS_LAN_SUBNET` antes de lanzar el monitor);
- Windows y Hyper-V conservan `Block` como acción entrante predeterminada y `Allow` para salida;
- no existen reglas ROS/DDS heredadas habilitadas con origen remoto `Any`.

La comprobación es de solo lectura: el monitor no solicita elevación ni modifica el firewall. El mismo resultado está disponible en `GET /api/firewall`. Si aparece **Firewall distribuido no alineado**, aplica nuevamente las verificaciones y comandos de la sección 6 desde PowerShell como administrador.

### B. Diagnóstico de Subred: Red del Robot (`192.168.1.x`) vs Red de Respaldo (`10.0.28.x`)
Por defecto, la red del proyecto y del enjambre de robots opera en la subred **`192.168.1.x`** (con el Router TP-Link AX12 en `192.168.1.1`).
- **Si el monitor o `ifconfig` reportan una subred `10.0.28.x` / `10.0.29.x`:** Indica un **problema de conexión física con el router del robot** (ej. la Wi-Fi se desconectó y la PC conmutó a la red de la institución/edificio).
- **Solución:** Verifica que el adaptador Wi-Fi de Windows o el cable Ethernet estén conectados a la red del router del robot (SSID: `ros2`). Con el **Modo Espejo (`networkingMode=mirrored`)** en `.wslconfig`, Ubuntu pasará automáticamente a la red `192.168.1.x` al restablecerse el enlace con el router.


### C. Métricas de Calidad de Enlace: Latencia, Jitter y Pérdida de Paquetes

Al evaluar la comunicación en vivo entre el PC de control, el robot (Kinova Gen3) y los microcontroladores (ESP32 con Micro-ROS), el monitor mide tres métricas críticas:

1. **Latencia (RTT - Round Trip Time):** 
   - Mide el tiempo total en milisegundos que tarda un paquete de datos en ir desde el emisor hasta el receptor y regresar.
   - En una red local Wi-Fi / Ethernet dedicada para ROS 2, la latencia ideal debe ser $< 5\text{ ms}$.

2. **¿Qué es el Jitter (Variación de Retardo)?**
   - **Definición:** El **Jitter** mide la *inestabilidad o fluctuación* en el tiempo de llegada entre paquetes de datos consecutivos en la red.
   - **Ejemplo Práctico en Robótica:** Si un nodo en el robot publica la odometría o los comandos de velocidad `/cmd_vel` de manera uniforme a 10 Hz (un paquete cada 100 ms), pero debido a la saturación o interferencia de la red Wi-Fi los paquetes llegan en lapsos irregulares (ej. 70 ms, 140 ms, 60 ms, 130 ms), esa desviación representa un **Jitter elevado**.
   - **Impacto:** En ROS 2, un Jitter alto provoca tirones en los motores, desincronización de transformadas TF2, retardos en los filtros de estimación de estado (EKF) y desconexiones intermitentes en Micro-ROS.

3. **Pérdida de Paquetes (`Packet Loss`):**
   - Mide el porcentaje de paquetes UDP que no llegaron a su destino durante la ráfaga de transmisión.
   - Como DDS utiliza UDP para la transmisión de baja latencia, una pérdida de paquetes mayor al 2% causará caídas de mensajes en tópicos Best Effort o retrasos por retransmisión en tópicos Reliable.

### D. Modo Experimento y Benchmark de Telemetría Científica
El monitor cuenta con un panel integrado para **grabar datasets experimentales** en tiempo real bajo 3 condiciones controladas (Línea Base WiFi 6, Carga Multi-Robot y Estrés Severo):
- **Iniciar/Detener:** Permite iniciar ensayos catalogados (ej. `ensayo_01_kinova`) y detenerlos generando un dataset estructurado en `.csv`.
- **Exportación Directa:** Descarga inmediata desde el botón `📥 Descargar CSV`.
- **Análisis Estadístico:** Procesamiento automático con [analyze_telemetry_benchmark.py](file:///home/roncanciovl/ros2_ws/src/burger_delivery/scripts/analyze_telemetry_benchmark.py).

> 📘 **Manual Completo de la Interfaz Web:**  
> Para una explicación detallada de todos los componentes visuales, KPIs, gráficas Canvas y endpoints de la API, consulta la [Guía completa de la interfaz del monitor](MONITOR_UI_GUIA.md).
> Para el diseño metodológico del experimento y pruebas estadísticas, consulta [EXPERIMENTO_QOS_TELEMETRIA.md](file:///home/roncanciovl/ros2_ws/src/burger_delivery/docs/research/EXPERIMENTO_QOS_TELEMETRIA.md).
