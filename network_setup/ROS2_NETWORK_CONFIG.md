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

## 5. Análisis: El Problema del Bloqueo del Daemon de ROS 2

**Introducción: Comandos de Inspección en ROS 2**
Para comprender qué está sucediendo en tu red de ROS 2, típicamente utilizas dos comandos fundamentales de la interfaz de línea de comandos (CLI):
- `ros2 node list`: Muestra todos los nodos activos que se han descubierto en tu red actual.
- `ros2 topic list`: Muestra todos los canales de comunicación (tópicos) por los cuales los nodos están enviando o recibiendo datos.
Estos comandos son esenciales para observar el estado de tu red, pero dependen de un proceso en segundo plano que a veces falla.

**¿En qué consiste el problema?**
A menudo, al ejecutar comandos de inspección como los mencionados arriba (`ros2 topic list` o `ros2 node list`), la terminal se queda congelada (bloqueada) o tarda demasiado en responder sin mostrar los datos reales de la red.

**Análisis de la causa:**
ROS 2 utiliza un proceso en segundo plano llamado **Daemon** (`_ros2_daemon`). Su propósito es mantener un caché de la topología de la red (nodos, tópicos, servicios) para responder rápidamente a los comandos de la CLI, evitando tener que redescubrir toda la red cada vez que escribes un comando.

El problema ocurre porque este daemon es susceptible a **cambios e inconsistencias en la red**:
1. **Cambios de IP / Interfaz:** Si pasas de una red Wi-Fi a otra, enciendes una VPN, o WSL cambia su IP, el daemon mantiene en caché las rutas antiguas. Al intentar comunicarse con las direcciones cacheadas, los paquetes se pierden y provoca un "timeout" o bloqueo.
2. **Problemas de Multicast:** En redes Wi-Fi saturadas o corporativas, los paquetes UDP Multicast (usados para descubrimiento) se pierden. El daemon se queda esperando respuestas de nodos que sabe que existen pero que no puede alcanzar.
3. **Conflictos del RMW:** Fast DDS (el default) a veces genera bloqueos internos de descubrimiento (deadlocks) si hay mucho ruido en la red o si la red es inestable.

**Alternativas y Soluciones:**

1. **Evitar el Daemon (Bypass CLI):**
   Puedes forzar a la CLI a no usar el caché del daemon añadiendo la bandera `--no-daemon`. Esto hará que el comando tarde un par de segundos más (porque descubre la red desde cero), pero garantiza que mostrará la información real y evitará el bloqueo.
   ```bash
   ros2 topic list --no-daemon
   ros2 node list --no-daemon
   ```

2. **Reiniciar el Daemon (Hard Reset):**
   Si la red cambió (ej. activaste el Modo Espejo o cambiaste de red), debes purgar el daemon para que vuelva a escanear.
   ```bash
   ros2 daemon stop
   ros2 daemon start
   
   # Si el proceso no se detiene correctamente, fuérzalo:
   pkill -f _ros2_daemon
   ```

3. **Cambiar el RMW a CycloneDDS:**
   Como configuramos en el Paso 1, usar `rmw_cyclonedds_cpp` reduce drásticamente los problemas de bloqueo del daemon. CycloneDDS maneja la memoria y el descubrimiento de forma mucho más eficiente en redes inalámbricas frente a caídas de paquetes.

4. **Usar Discovery Server (Ver sección 7):**
   Si el multicast sigue bloqueando el daemon, cambiar a un servidor de descubrimiento centralizado elimina el problema de raíz al erradicar la dependencia del tráfico Multicast.

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
