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
                           │  - Endpoints /api/traffic, /api/scan    │
                           │  - Endpoints /api/benchmark/*           │
                           │  - Servidor estático sin dependencias   │
                           └──────────────┬─────────────┬────────────┘
                                          │             │
                    ┌─────────────────────▼─┐         ┌─▼─────────────────────┐
                    │   traffic_sniffer.py  │         │   device_scanner.py   │
                    │ - /proc/net/udp parser│         │ - Escaneo ARP/ICMP    │
                    │ - Medición RTT/Jitter │         │ - Mapeo de IPs y MACs │
                    │ - Data Logger CSV     │         │ - Detección de roles  │
                    └───────────────────────┘         └───────────────────────┘
```

### Características Principales:
1. **Cero Dependencias Pesadas:** Backend en Python puro (`http.server`, `socket`, `subprocess`) y frontend en HTML5/Vanilla CSS/Canvas sin frameworks pesados (Node.js o React no requeridos).
2. **Inspección sin Privilegios (`No sudo`):** Lee sockets RTPS directamente del kernel Linux (`/proc/net/udp` y `/proc/net/udp6`).
3. **Grabador Científico de Telemetría:** Registra métricas de QoS (latencia, jitter, pérdida, ancho de banda DDS) con exportación automática a formato `.csv`.
4. **Resiliente a Subredes Mixtas:** Detecta automáticamente si el host está en la red del robot (`192.168.1.x`) o en redes institucionales (`10.0.28.x`), alertando desajustes en tiempo real.

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
- **Botón `📶 Test Multicast UDP` (`btn-test-multicast`):** Abre un modal que envía un paquete UDP crudo a la dirección de Multicast DDS `239.255.0.1:7400`. Valida instantáneamente si el router permite el descubrimiento automático de ROS 2.

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
| **Canal ROS 2 DDS** | `ROS_DOMAIN_ID` Activo (ej. `42` o `0`) | - Número de sockets RTPS abiertos<br>- Estado de Discovery Server (`Activo`/`Inactivo`) | Confirma si los procesos locales están escuchando en el dominio correcto. |
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
- **Inspección de Sockets RTPS:** Muestra los puertos UDP locales abiertos por el RMW (`rmw_cyclonedds_cpp` o `rmw_fastrtps_cpp`), calculando el dominio DDS al que pertenece cada socket mediante la fórmula:
  $$\text{Puerto} = 7400 + (250 \times \text{ROS\_DOMAIN\_ID})$$

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
- ⚙️ **Configuración de Red y Firewall:** [ROS2_NETWORK_CONFIG.md](file:///home/roncanciovl/ros2_ws/src/burger_delivery/network_setup/ROS2_NETWORK_CONFIG.md)
- 🐍 **Script de Análisis Estadístico:** [analyze_telemetry_benchmark.py](file:///home/roncanciovl/ros2_ws/src/burger_delivery/scripts/analyze_telemetry_benchmark.py)
