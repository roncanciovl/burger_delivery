# Guía de Análisis y Configuración de Red en ROS 2

Este documento proporciona un camino lógico, paso a paso, para configurar, comprender y solucionar problemas en tu red de ROS 2, especialmente en entornos mixtos (Linux, Windows, WSL).

---

## 1. Conceptos Fundamentales de Red

Antes de configurar la red, es importante comprender cómo se comunican los nodos y las tres variables fundamentales que controlan esto en ROS 2:

**El Rol del UDP Multicast en DDS:**
ROS 2 utiliza el estándar DDS (Data Distribution Service) como su capa subyacente. Para que los nodos se encuentren entre sí automáticamente en una red sin necesidad de un servidor central (como el antiguo `roscore`), DDS emplea **UDP Multicast**. Esto significa que cuando un nodo se inicia, "grita" su presencia enviando paquetes a una dirección IP de Multicast dedicada. Todos los demás nodos de ROS 2 en la red "escuchan" en esa dirección, lo que les permite descubrirse mutuamente y establecer conexiones (peer-to-peer). Si tu enrutador (router) o firewall bloquea el tráfico UDP Multicast, los nodos jamás se verán, sin importar cuán bien esté configurado ROS 2.

1. **`ROS_DOMAIN_ID=42`**
   - Crea un "grupo de red" aislado para tus nodos de ROS.
   - Solo los nodos con el **mismo ID de dominio** pueden comunicarse entre sí.
   - Rango: 0-101 (Utilizamos `42` para este proyecto).

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

### Opción B: Proxy de Puertos (Alternativa)
Si no puedes usar el modo Espejo, debes redirigir los puertos DDS/micro-ROS desde Windows hacia WSL usando PowerShell (como Administrador):
```powershell
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8888 connectaddress=<IP_DE_WSL> connectport=8888
```

---

## 3. Aplicando Configuraciones de Entorno

Una vez que la red del host es accesible, aplica las variables principales a todas las PCs que participarán.

### En Linux / WSL (Ubuntu)
Añade al archivo `~/.bashrc`:
```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
Luego ejecuta: `source ~/.bashrc`

### En Windows (Nativo)
Configura las variables de entorno (PowerShell como Administrador):
```powershell
[System.Environment]::SetEnvironmentVariable('ROS_DOMAIN_ID', '42', 'User')
[System.Environment]::SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET', 'User')
[System.Environment]::SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp', 'User')
```
Reinicia tu terminal para aplicar los cambios.

---

## 4. Verificación y Pruebas

Verifica que tus nodos puedan comunicarse a través de la red.

### Diagnósticos Automatizados
- **Lista de Verificación de Red y ROS 2:** `bash test_ros2_network.sh`
- **Estabilidad Wi-Fi:** `bash diagnostico_wifi.sh`

### Prueba Manual 1: Validación de Multicast a Nivel de Red (Muy Recomendado)
Esta prueba es altamente viable y crucial para diagnosticar problemas, ya que utiliza una herramienta nativa (`ros2 multicast`) que envía un paquete UDP Multicast "crudo", evadiendo por completo la compleja capa de DDS. Si esto falla, el problema radica estrictamente en tu red física, Firewall o Router, y no en tus configuraciones del dominio de ROS 2.

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

En WSL2 se ha reportado un caso reproducible en el que esa conexión local queda esperando hasta terminar en `TimeoutError`, especialmente con ROS 2 Humble y red reflejada. También aparecen inconsistencias cuando se cambia de dominio, RMW, perfil DDS, Wi-Fi o VPN sin reiniciar el daemon. Esto es distinto de una falla de multicast: el daemon puede estar bloqueado aunque los nodos continúen intercambiando datos.

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

## 6. Configuración del Firewall

Si los nodos aún no son visibles en la red, es muy probable que tu Firewall esté bloqueando los puertos UDP de DDS. DDS asigna puertos dinámicamente basándose en tu `ROS_DOMAIN_ID`.
Fórmula: `7400 + (250 * ROS_DOMAIN_ID)`
Para `ROS_DOMAIN_ID=42`, el rango de puertos UDP es **17900 a 18150**.

### Linux / Ubuntu (UFW)
```bash
sudo ufw allow 17900:18150/udp
```

### Windows Firewall (Configuración en Host de Windows)

Para permitir el paso de datos desde/hacia tu máquina Windows (y por ende WSL), tienes dos opciones principales.

#### Opción A: Reglas Específicas por Dominio (Recomendado y Seguro)
Esta opción abre **únicamente** los puertos estrictamente necesarios para tu ID de dominio.
Ejecuta en PowerShell (como Administrador):
```powershell
# Para el ROS_DOMAIN_ID=42 (Puertos 17900-18150)
New-NetFirewallRule -DisplayName "ROS 2 DDS UDP (Domain 42)" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 17900-18150
New-NetFirewallRule -DisplayName "ROS 2 DDS TCP (Domain 42)" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 17900-18150
New-NetFirewallRule -DisplayName "ROS 2 DDS Outbound (Domain 42)" -Direction Outbound -Action Allow -Protocol UDP -LocalPort 17900-18150
```
*Si alguna vez cambias tu `ROS_DOMAIN_ID`, deberás recalcular los puertos y actualizar estas reglas.*

#### Opción B: Desbloqueo General de Puertos UDP/TCP (Rápido, pero de Alto Riesgo)
⚠️ **ADVERTENCIA CRÍTICA DE CIBERSEGURIDAD:** Esta opción abre el tráfico de **todos los puertos UDP y TCP** a través del firewall para ROS 2. 
- **¿Por qué hacerlo?** Si trabajas con múltiples dominios a la vez o cambias constantemente de `ROS_DOMAIN_ID` y no quieres recalcular puertos, esto garantiza que la red siempre funcionará de inmediato.
- **Riesgos de Seguridad:** Al permitir todo el tráfico entrante UDP/TCP sin restricción de puertos, tu sistema queda expuesto en la red local. Si te conectas a redes públicas (como una cafetería, aeropuerto o Wi-Fi público de la universidad) o si tu red está comprometida, atacantes podrían aprovechar servicios vulnerables, realizar escaneos completos de puertos, lanzar ataques de denegación de servicio (DDoS) a nivel de red, o interceptar e inyectar paquetes maliciosos hacia servicios que ahora son accesibles en tu sistema.
- **Veredicto:** Solo utiliza esta opción en **redes locales privadas, confiables y completamente aisladas** (ej. la red dedicada de tu laboratorio de robótica), NUNCA en redes públicas o corporativas donde no tengas control absoluto de los dispositivos conectados.

Si entiendes los riesgos y decides aplicarlo, ejecuta en PowerShell (como Administrador):
```powershell
New-NetFirewallRule -DisplayName "ROS 2 DDS ALL UDP" -Direction Inbound -Action Allow -Protocol UDP
New-NetFirewallRule -DisplayName "ROS 2 DDS ALL TCP" -Direction Inbound -Action Allow -Protocol TCP
New-NetFirewallRule -DisplayName "ROS2 DDS Discovery (UDP)" -Direction Inbound -Action Allow -Protocol UDP
New-NetFirewallRule -DisplayName "ROS 2 DDS Outbound" -Direction Outbound -Action Allow
```

---

## 7. Avanzado: Servidor de Descubrimiento (Fast DDS Discovery Server)

Si tienes problemas persistentes con el **multicast** (común en redes corporativas o de universidades), puedes utilizar el *Discovery Server*. Esto reemplaza el sistema automático en el que los nodos "gritan" su existencia por toda la red, centralizándolo en una "agenda telefónica" única (el Servidor).

### 1. El Archivo de Configuración (`fastdds_discovery.xml`)
Crea un archivo XML que configure tus nodos de ROS para que actúen como **Super Clientes**. En lugar de buscar por toda la red usando multicast, se conectarán directamente a la IP estática del Discovery Server.

### 2. Cómo Utilizarlo

**A. Iniciar el Servidor (Solo en la PC Principal):**
```bash
fastdds discovery --server-id 0 --ip-address 192.168.1.100 --port 11811
```

**B. Configurar los Clientes (En todas las PCs):**
Añade estas variables a tu `~/.bashrc`:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/src/burger_delivery/network_setup/fastdds_discovery.xml
```
*(Nota: También puedes usar una configuración más simple como `export ROS_DISCOVERY_SERVER="192.168.1.100:11811"`, pero el archivo XML te otorga mucho más control sobre la calidad de servicio y el comportamiento del descubrimiento).*

---

## 8. Diagnóstico y Diagnósticos con el Monitor de Red (`monitor_red`)

El proyecto incluye un monitor web interactivo (`network_setup/iniciar_monitor.sh`) diseñado para visualizar en tiempo real la topología de la red, los nodos activos en el `ROS_DOMAIN_ID` y las métricas de calidad de enlace.

### A. Observación de dominios RTPS sin privilegios de administrador
Para asociar una IP con un `ROS_DOMAIN_ID` sin ejecutar el monitor con `sudo`:
- El monitor escucha pasivamente los anuncios SPDP multicast y valida que contengan una cabecera RTPS.
- Atribuye el dominio usando el puerto multicast estándar:
  $$\text{Puerto Base} = 7400 + (250 \times \text{ROS\_DOMAIN\_ID})$$
- Por ejemplo, el anuncio SPDP del **Dominio 0** llega al puerto `7400` y el del **Dominio 42** al `17900`.

El dominio configurado localmente no se presenta como actividad. Para computadores remotos, si no llega un anuncio RTPS válido, la interfaz muestra **Domain desconocido**. El uso exclusivo de Discovery Server, descubrimiento unicast, `LOCALHOST`, aislamiento Wi-Fi o filtrado multicast puede impedir la observación remota.

El observador tampoco abre puertos dentro del rango UDP efímero de Linux. En WSL considera además el rango dinámico de Windows, ya que esa capa puede reservar puertos que no aparecen en `ss` dentro de Linux.

### B. Diagnóstico de Subred: Red del Robot (`192.168.1.x`) vs Red de Respaldo (`10.0.28.x`)
Por defecto, la red del proyecto y del enjambre de robots opera en la subred **`192.168.1.x`** (con el Router TP-Link AX12 en `192.168.1.1`).
- **Si el monitor o `ifconfig` reportan una subred `10.0.28.x` / `10.0.29.x`:** Indica un **problema de conexión física con el router del robot** (ej. la Wi-Fi se desconectó y la PC conmutó a la red de la institución/edificio).
- **Solución:** Verifica que el adaptador Wi-Fi de Windows o el cable Ethernet estén conectados a la red del router del robot (SSID: `ros`). Con el **Modo Espejo (`networkingMode=mirrored`)** en `.wslconfig`, Ubuntu pasará automáticamente a la red `192.168.1.x` al restablecerse el enlace con el router.


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
> Para una explicación detallada de todos los componentes visuales, KPIs, gráficas Canvas y endpoints de la API, consulta la [Guía Completa de la Interfaz del Monitor (MONITOR_UI_GUIA.md)](file:///home/roncanciovl/ros2_ws/src/burger_delivery/network_setup/MONITOR_UI_GUIA.md).  
> Para el diseño metodológico del experimento y pruebas estadísticas, consulta [EXPERIMENTO_QOS_TELEMETRIA.md](file:///home/roncanciovl/ros2_ws/src/burger_delivery/docs/research/EXPERIMENTO_QOS_TELEMETRIA.md).
