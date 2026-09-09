# 🖥️ Manual Completo de la Interfaz Web y Arquitectura del Monitor de Red (`monitor_red`)

> **Burger-Cell Living Lab & Telemetry System**  
> *Herramienta de Diagnóstico en Tiempo Real, Visualización de Topología DDS y Grabación de Telemetría Científica (QoS Benchmarking)*

---

## 1. Visión General y Arquitectura

El **Monitor de Red de Burger-Cell** es una aplicación web ligera, reactiva y desacoplada construida específicamente para entornos de robótica distribuida bajo **ROS 2** y **micro-ROS**.

```
                           ┌─────────────────────────────────────────┐
                           │            Navegador Web (UI)           │
                           │  - Dashboard HTML5 / Vanilla CSS        │
                           │  - Gráficas Canvas 60fps                │
                           │  - Panel Benchmark & Control de Ensayos │
                           └────────────────────▲────────────────────┘
                                                │ REST API / Polling 1 Hz
                                                │ (JSON)
                           ┌────────────────────▼────────────────────┐
                           │          server.py (HTTP Server)        │
                           │  - /api/traffic, /api/scan, /api/firewall│
                           │  - Endpoints /api/benchmark/*           │
                           │  - Servidor estático sin dependencias   │
                           └──────────────┬─────────────┬────────────┘
                                          │             │
                    ┌─────────────────────▼─┐         ┌─▼─────────────────────┐
                    │   traffic_sniffer.py  │         │   device_scanner.py   │
                    │ - Observador SPDP     │         │ - Escaneo ARP/ICMP    │
                    │ - Medición RTT/Jitter │         │ - Mapeo de IPs y MACs │
                    │ - Data Logger CSV     │         │ - Detección de roles  │
                    └───────────────────────┘         └───────────────────────┘
                                                ┌────────────────────────────┐
                                                │ firewall_status.py         │
                                                │ - Consulta de solo lectura │
                                                │ - Reglas Windows/Hyper-V   │
                                                └────────────────────────────┘
```

### Características Principales:
1. **Cero Dependencias Pesadas:** Backend en Python puro (`http.server`, `socket`, `subprocess`) y frontend en HTML5/Vanilla CSS/Canvas sin frameworks pesados (Node.js o React no requeridos).
2. **Observación sin privilegios (`No sudo`):** Escucha pasivamente los anuncios SPDP multicast y valida la cabecera RTPS antes de asociar una IP con un dominio.
3. **Grabador Científico de Telemetría:** Registra métricas de QoS (latencia, jitter, pérdida, ancho de banda DDS) con exportación automática a formato `.csv`.
4. **Resiliente a Subredes Mixtas:** Detecta automáticamente si el host está en la red del robot (`192.168.1.x`) o en redes institucionales (`10.0.28.x`), alertando desajustes en tiempo real.
5. **Verificación de firewall en WSL:** Comprueba en modo de solo lectura que Windows y Hyper-V permitan UDP entrante exclusivamente desde la subred ROS y conserven el bloqueo predeterminado para las demás redes.

---

## 2. Puesta en Marcha

Para iniciar el servidor y abrir el monitor:

```bash
# Opción 1: Script centralizado
bash network_setup/iniciar_monitor.sh

# Opción 2: Ejecución directa en Python
python3 network_setup/monitor_red/server.py --port 8080
```

Acceder desde cualquier navegador web en:  
🌐 **`http://localhost:8080`** (o `http://<IP_DE_TU_PC>:8080` desde otra máquina en la misma red Wi-Fi).

---

## 3. Recorrido Detallado por Componentes de la Interfaz (UI)

```
┌──────────────────────────────────────────────────────────────────────────────────────┐
│ [📡 Burger-Cell Network Monitor]       [🟢 Conectado 1Hz]  [🔍 Escanear]  [📶 Multicast] │
├──────────────────────────────────────────────────────────────────────────────────────┤
│ 📡 Banner de Sincronización: Calibrando telemetría...              [Acumulado: 12s]  │
├──────────────────────────────────────────────────────────────────────────────────────┤
│ 🔬 MODO EXPERIMENTO: Registro de Telemetría QoS & ROS 2                             │
│ Escenario: [1. Línea Base (WiFi 6) ▼]  Ensayo: [ensayo_01_kinova] [▶ Iniciar Grabación]│
│ [⏱ 00:34] [📊 34 muestras] [📶 RTT: 1.8 ms] [〰 Jitter: 0.4 ms] [📉 Pérdida: 0.0%]  │
├──────────────────────────────────────────────────────────────────────────────────────┤
│ [ KPI 1: Gateway AX12 ] [ KPI 2: ROS 2 DDS ] [ KPI 3: micro-ROS ] [ KPI 4: Tráfico ]│
├──────────────────────────────────────────────────────────────────────────────────────┤
│ [ 📈 Gráfica Temporal en Vivo (Canvas) ]    │ [ 🗺️ Topología Visual de la Celda ]   │
├──────────────────────────────────────────────────────────────────────────────────────┤
│ [ 📋 Tabla de Dispositivos Conectados ]     │ [ 🔌 Sockets RTPS & Puertos DDS ]      │
└──────────────────────────────────────────────────────────────────────────────────────┘
```

---

### 3.1. Barra Superior (Header) y Herramientas Globales

- **Badge de Estado en Vivo (`sse-badge`):** Indica la conexión activa con el servidor (`Conectado en Vivo (1 Hz)`). Si el servidor se apaga, cambia a rojo indicando reconexión.
- **Botón `🔍 Escanear Red` (`btn-scan`):** Ejecuta un barrido ARP y de ping ICMP en la subred local mediante `device_scanner.py` para descubrir nuevos dispositivos conectados (ESP32, Kinova, PC secundarias).
- **Botón `📶 Test Multicast UDP` (`btn-test-multicast`):** Ejecuta un eco local en `225.0.0.1:49150`, separado de los puertos DDS. Confirma el envío y recepción multicast dentro del host, pero no valida el router, el firewall entre equipos ni el descubrimiento distribuido. Para eso se requieren dos computadores.

---

### 3.2. Banner de Sincronización y Calibración (`sync-banner`)

Al abrir la interfaz, este banner muestra el progreso de acumulación de paquetes y estabilización de métricas de red:
- **Fase de Calibración (0 a 5 segundos):** Recopila muestras iniciales de RTT y Jitter con el router TP-Link AX12.
- **Fase Estabilizada (> 5 segundos):** Cambia a verde (`Telemetría sincronizada y calibrada`), confirmando que las lecturas son estadísticamente representativas.

---

### 3.3. Panel de Experimento: Registro de Telemetría QoS & DDS (`benchmark-card`)

Este módulo fue diseñado para investigaciones cuantitativas y publicaciones científicas:

#### Controles:
1. **Selector de Escenario (`benchmark-scenario-select`):**
   - `1. Línea Base (WiFi 6 AX12 Limpio)`: Red sin tráfico concurrente.
   - `2. Carga Típica (Multi-Robot / Tráfico Concurrente)`: Simula operación de enjambre.
   - `3. Estrés Severo (Congestión / Pérdida Forzada)`: Condiciones críticas de interferencia.
   - `4. Ensayo Personalizado`: Para pruebas ad-hoc.
2. **Identificador del Ensayo (`benchmark-session-input`):** Nombre personalizado para catalogar la prueba (ej. `ensayo_01_kinova_vlm`).
3. **Botón `▶ Iniciar Grabación` / `⏹ Detener y Exportar` (`btn-benchmark-toggle`):**
   - Al activarse, cambia a color rojo parpadeante con la etiqueta `🔴 GRABANDO TELEMETRÍA EN VIVO`.
   - Comienza a registrar cada segundo en memoria los valores de RTT, Jitter, Pérdida de paquetes y tasas de bits.
   - Al detenerse, guarda el archivo físico `.csv` en `network_setup/monitor_red/benchmark_logs/`.
4. **Botón `📥 Descargar CSV` (`btn-benchmark-download`):** Descarga directamente en el navegador el dataset del último ensayo finalizado.

#### Barra de Métricas en Vivo:
- **⏱ Tiempo:** Cronómetro de duración del ensayo en formato `mm:ss`.
- **📊 Muestras:** Contador incremental de filas registradas.
- **📶 RTT Promedio:** Latencia media acumulada de la sesión.
- **〰 Jitter Promedio:** Desviación estándar del retardo acumulada (`mdev`).
- **📉 Pérdida:** Porcentaje de pérdida de paquetes experimentado en el ensayo.
- **📁 Archivo:** Nombre del archivo `.csv` generado con timestamp ISO.

---

### 3.4. Tarjetas Rápidas de Telemetría (KPIs)

| Tarjeta KPI | Métrica Principal | Submétricas e Indicadores | Diagnóstico / Alerta |
|---|---|---|---|
| **Gateway (Router AX12)** | IP del Gateway (ej. `192.168.1.1`) | - RTT Latencia (ms)<br>- Jitter de dispersión (ms)<br>- Pérdida de paquetes (%) | Si RTT $> 10\text{ ms}$ o Pérdida $> 2\%$, la tarjeta muestra advertencia visual. |
| **Canal ROS 2 DDS** | Dominios RTPS observados (ej. `42` o `0`) | - Dominio configurado localmente<br>- Estado del observador y Discovery Server | Distingue configuración local de actividad DDS realmente observada. |
| **micro-ROS Agent** | Estado del Agente (`ONLINE` / `OFFLINE`) | - Puerto UDP `8888`<br>- PID del proceso daemon en ejecución | Alerta si el agente para los ESP32 no está corriendo en segundo plano. |
| **Tráfico de Red Global** | Tasa total de transferencia ($\text{KB/s}$) | - Tráfico TCP ($\text{KB/s}$)<br>- Tráfico UDP / DDS ($\text{KB/s}$) | Permite monitorear el consumo de ancho de banda en ráfagas de control. |

---

### 3.5. Gráfica Temporal en Vivo (Canvas 2D)

Renderizada a $60\text{ fps}$ mediante la API nativa de Canvas:
- **Línea Cian:** Tráfico TCP.
- **Línea Naranja:** Tráfico UDP general.
- **Línea Morada (Área sombreada):** Tráfico específico de **ROS 2 DDS**.
- **Línea Verde:** Tráfico del agente **micro-ROS** (puerto 8888).
- **Interactividad:** Haz clic en los chips de la leyenda para ocultar o mostrar flujos individuales.

---

### 3.6. Topología Visual de la Celda Robótica

Representa de manera intuitiva el mapa físico de la celda de trabajo:
- **🌐 Router Principal (TP-Link Archer AX12):** Nodo central de la topología inalámbrica (`192.168.1.1`).
- **💻 Host PC (Workstation / WSL2):** Estación de control de ROS 2.
- **🦾 Brazo Kinova Gen3:** Conexión del manipulador industrial (`192.168.1.10`).
- **🤖 ESP32 / micro-ROS:** Microcontroladores embebidos (`192.168.1.50+`).
- **🚗 Mobile Carts / AGVs:** Plataformas móviles de transporte.

Cada nodo muestra un punto de estado:
- 🟢 **Verde:** Dispositivo respondiendo a ping y con tráfico activo.
- 🟡 **Amarillo:** Latencia elevada o respuesta intermitente.
- 🔴 **Rojo:** Dispositivo desconectado o inalcanzable.

---

### 3.7. Tabla de Dispositivos e Inspección de Sockets

- **Filtros Rápidos:** Botones para filtrar la tabla por `Todos`, `ROS 2`, `micro-ROS` e `Infraestructura`.
- **Observaciones RTPS:** Muestra los dominios para los cuales llegaron anuncios RTPS válidos y el puerto multicast SPDP utilizado para atribuirlos:
  $$\text{Puerto} = 7400 + (250 \times \text{ROS\_DOMAIN\_ID})$$

La tabla diferencia tres estados:

- **Observado:** se recibió tráfico RTPS válido desde la IP indicada; el dominio es evidencia de ejecución.
- **Configurado:** aplica solamente al PC que ejecuta el monitor; expresa `ROS_DOMAIN_ID`, pero no afirma que exista un nodo activo.
- **Desconocido:** el dispositivo está en red, pero no se observó RTPS. El monitor no sustituye ese dato con el dominio local.

La detección remota requiere anuncios SPDP multicast visibles. Si el RMW usa exclusivamente un Discovery Server, descubrimiento unicast, `LOCALHOST`, una VPN que filtra multicast o el punto de acceso aísla clientes, el dominio remoto se mostrará como desconocido en vez de inferirse.

El observador evita los dominios cuyo bloque DDS se cruza con el rango UDP efímero configurado en `/proc/sys/net/ipv4/ip_local_port_range`. En WSL también considera el rango dinámico de Windows. Un dominio en cualquiera de esos rangos se marca como no observable para no bloquear puertos de otras aplicaciones.

El panel de servicios incluye un indicador de firewall actualizado cada 30 segundos. En WSL valida las reglas `ROS2-Distributed-LAN-HyperV` y `ROS2-Distributed-LAN-Windows`, la subred remota esperada (`192.168.1.0/24`, o la declarada en `ROS_LAN_SUBNET` antes de lanzar el servidor), UDP en cualquier puerto, las políticas predeterminadas de bloqueo entrante y posibles reglas ROS/DDS heredadas abiertas a `Any`. Esta comprobación nunca crea ni modifica reglas. En Linux nativo aparece como no aplicable.

Las entradas RTPS corresponden a los puertos multicast SPDP calculados. No representan todos los sockets de datos: CycloneDDS puede negociar puertos UDP dinámicos, que el panel no enumera como una lista fija.

#### Distintivo `🔒 ANFITRIONA · driver Kinova`

Con un solo robot y varias estaciones, la pregunta que más tiempo hace perder es **qué máquina tiene ocupado el Kinova**. La tabla la responde de un vistazo: la fila de esa estación lleva un distintivo verde adicional, y las que se anuncian sin tener el robot llevan uno gris de `cliente ROS 2`.

Ese dato **no se obtiene observando la red**, y conviene entender por qué:

- El tráfico entre la estación que ejecuta el driver y la controladora es **unicast**. En una red conmutada el switch lo entrega únicamente a esos dos puertos, así que ninguna tercera máquina —el monitor incluido— puede verlo.
- Esnifar ARP tampoco sirve: exigiría `CAP_NET_RAW`, privilegio que este monitor evita a propósito, y Linux refresca la entrada ARP con sondas **unicast**, de modo que sólo sería visible el instante inicial en que arranca el driver.
- Sondear el puerto de control del robot para comprobar si está ocupado significaría intentar abrir una sesión Kortex contra un brazo que puede estar en movimiento, con riesgo de perturbar la sesión real y disparar una parada de seguridad.

Por eso **la estación que tiene el robot se anuncia**. Cada `kinova_monitor` comprueba en su propio `/proc/net/tcp` si mantiene la sesión TCP con la controladora y difunde el resultado por broadcast UDP (puerto `45455`). El monitor sólo escucha: [`station_listener.py`](monitor_red/station_listener.py), sin privilegios y sin configurar ninguna dirección, porque la IP de origen se toma del propio `recvfrom` y se correlaciona con la lista de dispositivos.

| Campo | Significado |
| :--- | :--- |
| `is_driver_host` | La estación declara tener el robot **y** lo verificó con una sesión TCP establecida |
| `station_role` | `anfitriona`, `cliente` o `desconocido`, según lo que la máquina comprobó de sí misma |
| `station_evidence` | La evidencia concreta, p. ej. `sesión TCP establecida con 192.168.1.10:10000` |
| `station_age_s` | Antigüedad del último anuncio recibido |

Los anuncios caducan a los 20 segundos sin recibirse, así que si la anfitriona se apaga el distintivo desaparece solo: no hace falta un mensaje de despedida que un apagón nunca llegaría a enviar.

> **Es un anuncio, no una autoridad.** Refleja lo que esa máquina *declara* de sí misma tras verificarlo localmente. Cualquier equipo de la red podría emitir uno falso; en una red de laboratorio cerrada resulta aceptable, pero el dato no es una prueba criptográfica. El único campo que un anuncio no puede falsear sin suplantar la dirección es la IP, porque la fija el socket y no el contenido del mensaje.

Endpoint asociado: `GET /api/estaciones` devuelve la anfitriona y todas las estaciones que se anuncian, sin necesidad de escanear la red. La misma información viaja embebida en `GET /api/devices`, y en ROS 2 está en `/burger/kinova/diagnostics` (ver [`TROUBLESHOOTING.md`](../TROUBLESHOOTING.md) §2).

---

## 4. Estructura del Dataset de Telemetría (`.csv`)

Los datasets generados por el panel de Benchmark se almacenan con el siguiente esquema:

```csv
timestamp_iso,elapsed_sec,session_name,scenario,total_kbps,dds_kbps,microros_kbps,tcp_kbps,udp_kbps,bytes_recv_rate,bytes_sent_rate,packets_recv_rate,packets_sent_rate,gateway_latency_ms,gateway_jitter_ms,gateway_loss_percent,dds_latency_ms,dds_jitter_ms,dds_loss_percent,active_dds_domains,microros_active
2026-08-06T13:36:16Z,0.85,ensayo_01_kinova,Linea_Base_WiFi6,514.86,411.89,0.0,102.97,411.89,128450.0,119200.0,160.0,152.0,1.2,0.3,0.0,0.5,0.2,0.0,42,1
```

---

## 5. Análisis Automatizado y Gráficas de Publicación

Una vez recolectados los archivos CSV, puedes procesarlos automáticamente con el script incluido:

```bash
# Analizar el archivo más reciente y generar gráficas
python3 scripts/analyze_telemetry_benchmark.py

# Analizar un archivo específico
python3 scripts/analyze_telemetry_benchmark.py --file network_setup/monitor_red/benchmark_logs/mi_experimento.csv
```

### Salidas Generadas:
1. **Resumen Estadístico en Terminal:** Media, Desviación Estándar, Mínimo, Máximo y Percentil 95 ($p_{95}$) de cada variable de red.
2. **Figura de Alta Resolución ($300\text{ DPI}$):** Guardada en `docs/research/figures/telemetry_benchmark_plot.png`, lista para ser insertada en artículos de IEEE, Sensors o reportes de investigación.

---

## 6. Referencias Cruzadas

- 📄 **Protocolo Experimental Completo:** [EXPERIMENTO_QOS_TELEMETRIA.md](file:///home/roncanciovl/ros2_ws/src/burger_delivery/docs/research/EXPERIMENTO_QOS_TELEMETRIA.md)
- ⚙️ **Configuración de Red y Firewall:** [ROS2_NETWORK_CONFIG.md](ROS2_NETWORK_CONFIG.md)
- 🐍 **Script de Análisis Estadístico:** [analyze_telemetry_benchmark.py](file:///home/roncanciovl/ros2_ws/src/burger_delivery/scripts/analyze_telemetry_benchmark.py)
