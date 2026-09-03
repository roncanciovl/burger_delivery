# Monitor de red ROS 2: controles, configuración e interpretación

Esta guía explica todos los controles visibles, configuraciones de ejecución, indicadores y límites del monitor ubicado en `network_setup/monitor_red/`. Está basada en la implementación actual de `index.html`, `app.js`, `server.py`, `device_scanner.py` y `traffic_sniffer.py`.

El objetivo es que la persona participante pueda operar el dashboard y, sobre todo, distinguir entre un valor configurado, una observación de red y una estimación.

## 1. Inicio y detención

Desde la raíz del repositorio:

```bash
bash network_setup/iniciar_monitor.sh
```

El puerto predeterminado es `8080`. Para solicitar otro puerto:

```bash
bash network_setup/iniciar_monitor.sh 8090
```

También puede iniciarse directamente:

```bash
python3 network_setup/monitor_red/server.py --host 127.0.0.1 --port 8080
```

Para detenerlo cuando está en primer plano, use `Ctrl+C` en su terminal. El lanzador elimina primero otras instancias de `monitor_red/server.py`, por lo que debe evitarse si se pretende conservar otra sesión del monitor.

Si el puerto solicitado está ocupado, `server.py` prueba hasta diez puertos consecutivos. La dirección efectiva se muestra en la consola del servidor.

> **Seguridad:** el lanzador usa `--host 0.0.0.0`, el servidor no implementa autenticación y permite CORS. En una red no confiable, prefiera `--host 127.0.0.1` o aplique reglas de firewall.

## 2. Qué observa y qué no modifica

El monitor combina cuatro mecanismos:

1. ARP, vecinos IP, resolución de hostname y `ping` para encontrar dispositivos.
2. Escucha pasiva de anuncios SPDP multicast para asociar una IP con un `ROS_DOMAIN_ID` observado.
3. Inspección de los puertos locales `8888` y `11811` para detectar Micro-ROS Agent y DDS Discovery Server.
4. Contadores globales de interfaces de red y pruebas `ping` para producir telemetría.

El monitor no cambia el dominio de nodos ROS 2 que ya se estén ejecutando en otros procesos o computadores. Tampoco configura el router, el firewall, CycloneDDS ni Fast DDS. En WSL consulta en modo de solo lectura las reglas de Windows/Hyper-V y muestra si coinciden con la política distribuida del proyecto.

### Estados de dominio

| Estado mostrado | Significado | Evidencia disponible |
|---|---|---|
| `observed` / “DDS observado” | Llegó una cabecera RTPS válida desde esa IP al puerto SPDP del dominio. | Tráfico de ejecución visible por multicast. |
| `configured` | Es el `ROS_DOMAIN_ID` del proceso del monitor local. | Configuración, pero no necesariamente un nodo ROS 2 activo. |
| `unknown` / “Domain desconocido” | El equipo está presente en la red, pero no se recibió RTPS atribuible. | No hay evidencia suficiente para afirmar un dominio. |

Una observación expira después de 180 segundos sin nuevos paquetes RTPS.

## 3. Barra superior

### Campo `DOMAIN_ID`

Acepta enteros entre `0` y `232`. El valor inicial procede de `ROS_DOMAIN_ID`; si la variable no existe, ROS 2 y el monitor usan `0`.

### Botón `Aplicar`

Al pulsarlo, o al presionar `Enter` dentro del campo:

- valida que el número esté entre `0` y `232`;
- actualiza `ROS_DOMAIN_ID` dentro del proceso del monitor;
- actualiza el dominio configurado del PC local;
- inicia un nuevo escaneo corto de dispositivos.

No hace lo siguiente:

- no cambia el entorno de la terminal que lanzó el monitor;
- no persiste el valor después de reiniciar el servidor;
- no reinicia ni reconfigura nodos ROS 2 existentes;
- no cambia el dominio de otros computadores.

Para iniciar nodos reales en el mismo dominio, configure la terminal antes de arrancarlos:

```bash
export ROS_DOMAIN_ID=0   # dominio del proyecto; use el asignado por el docente si es otro
```

Si el bloque de puertos DDS del dominio se cruza con el rango UDP efímero de Linux o Windows/WSL, la interfaz informa que ese dominio no es observable por este mecanismo pasivo.

### Indicador `Conectado en Vivo`

Indica que la página inicializó su ciclo de consultas HTTP. La interfaz consulta tráfico y benchmark cada segundo, y dispositivos cada cuatro segundos.

Este indicador no demuestra que exista comunicación DDS, que el multicast funcione o que haya nodos ROS 2 activos. Aunque el elemento conserva el nombre interno `sse-badge`, la implementación actual usa polling HTTP y no Server-Sent Events.

### Botón `Escanear Red`

Ejecuta un barrido completo de las subredes `/24` activas detectadas:

- consulta ARP y vecinos del kernel;
- prueba direcciones mediante ICMP `ping`;
- incorpora IP que hayan emitido RTPS;
- actualiza tabla y topología.

Durante el proceso cambia a `Escaneando...`. El barrido genera tráfico ICMP y puede tardar varios segundos, especialmente si existen varias interfaces o VPN.

El servidor también realiza un escaneo inicial y refrescos abreviados aproximadamente cada diez segundos, por lo que el botón se reserva para forzar un descubrimiento inmediato y más amplio.

### Botón `Test Multicast UDP`

Abre un receptor local, envía un mensaje a `225.0.0.1:49150` y muestra:

- éxito o fallo;
- descripción del resultado;
- tiempo de retorno local.

El grupo y puerto se separan deliberadamente de SPDP/DDS para no confundir esta prueba con actividad ROS 2. Un resultado exitoso valida únicamente que el host puede enviar y recibir localmente en ese grupo multicast. No prueba que otro computador reciba el paquete ni descarta AP Isolation, filtrado Wi-Fi o reglas de firewall entre equipos. Para validar la red distribuida se necesita una prueba desde dos hosts.

### Botón `×` del modal

Cierra la ventana de resultados del test multicast. No cancela ninguna prueba ya terminada ni modifica la configuración.

## 4. Barra de sincronización

La barra muestra los segundos transcurridos desde que se cargó la página. Después de cuatro segundos cambia visualmente a “Telemetría en Vivo Sincronizada”.

Es un indicador de tiempo de interfaz, no una certificación estadística ni una confirmación de descubrimiento DDS. Los dominios deben verificarse en el KPI DDS y en la tabla de dispositivos.

## 5. Modo experimento y benchmark

El benchmark registra una fila aproximadamente cada segundo mientras el recolector está activo. Los archivos se guardan en:

```text
network_setup/monitor_red/benchmark_logs/
```

### Selector `Escenario de Red`

| Opción | Valor guardado en CSV | Uso recomendado |
|---|---|---|
| Línea Base | `Linea_Base_WiFi6` | Red con baja carga para construir referencia. |
| Carga Típica | `Carga_MultiRobot` | Ensayo mientras operan varios nodos o robots. |
| Estrés Severo | `Estres_Severo` | Ensayo durante una condición de congestión creada externamente. |
| Personalizado | `Personalizado` | Situación no cubierta por las etiquetas anteriores. |

El selector solamente etiqueta el experimento. No crea tráfico, no simula robots y no aplica pérdida, latencia o congestión.

### Campo `Identificador del Ensayo`

Define el nombre lógico de la sesión. Si queda vacío se usa `ensayo_qos`. Al iniciar, cualquier carácter distinto de letras, números, guion o guion bajo se reemplaza por `_`.

Ejemplo:

```text
ensayo_01_kinova
```

### Botón `Iniciar Grabación`

Inicia una sesión nueva, limpia las muestras anteriores en memoria y bloquea el escenario y el identificador hasta detenerla. El botón cambia a `Detener y Exportar` y el estado muestra `GRABANDO TELEMETRÍA EN VIVO`.

La barra en vivo informa:

- tiempo transcurrido;
- cantidad de muestras;
- RTT promedio al gateway;
- jitter promedio del gateway;
- pérdida promedio;
- último nombre de archivo disponible.

### Botón `Detener y Exportar`

Finaliza el registro y crea un CSV si existe al menos una muestra. El nombre sigue este patrón:

```text
benchmark_<escenario>_<ensayo>_<AAAAMMDD_HHMMSS>.csv
```

El CSV contiene marcas de tiempo, tasas estimadas TCP/UDP/DDS/Micro-ROS, contadores RX/TX, RTT, jitter, pérdida, dominios observados y estado del agente Micro-ROS.

### Botón `Descargar CSV`

Permanece deshabilitado hasta que el proceso actual haya generado un archivo. Descarga el último CSV producido en esa ejecución.

Después de reiniciar el monitor, los CSV anteriores siguen en `benchmark_logs/`, pero el botón no recupera automáticamente cuál fue el último archivo de una ejecución anterior.

## 6. Tarjetas KPI

### `Gateway de red activo`

Muestra el gateway de la ruta predeterminada, RTT, jitter y pérdida obtenidos mediante dos paquetes `ping` por ciclo.

Interpretación:

- RTT y jitter bajos sugieren un enlace estable hacia el gateway;
- pérdida distinta de cero requiere revisar señal Wi-Fi, congestión o firewall;
- un valor cero también puede aparecer cuando la medición no obtiene respuesta, por lo que debe contrastarse con un `ping` manual.

### `Tráfico Total en Tiempo Real`

Resume contadores agregados de las interfaces del host y muestra RX/TX. No representa exclusivamente la interfaz del robot ni exclusivamente ROS 2.

La implementación convierte el delta de bytes a bits y divide por `1024`, aunque la interfaz usa la etiqueta `KB/s`. Por ello este panel es más fiable para comparar tendencias relativas que para afirmar un caudal exacto en kilobytes por segundo.

### `ROS 2 DDS & Dominios`

Puede mostrar:

- `Observados: ...` cuando llegaron anuncios RTPS válidos;
- `Sin RTPS observado (local: N)` cuando solo se conoce la configuración local;
- un error del observador;
- una advertencia si el dominio local cae en un rango UDP efímero no observable.

El valor `DDS estimado` no proviene de captura completa de paquetes DDS. El backend reparte el tráfico global mediante proporciones fijas según detecte DDS o Micro-ROS. Debe usarse como indicador exploratorio, no como medición científica aislada.

Las métricas denominadas jitter y pérdida DDS se obtienen actualmente mediante `ping` al Kinova `192.168.1.10` cuando el gateway pertenece a `192.168.x.x`; fuera de esa red se usa el gateway. No son métricas tomadas del protocolo DDS ni de sus QoS.

### `Micro-ROS Agent (UDP 8888)`

Muestra `Agente Activo` cuando existe un socket local en el puerto UDP `8888`. Si `psutil` permite resolverlo, también muestra el PID.

Este estado confirma el socket del agente en el computador del monitor; no garantiza que un ESP32 esté conectado, sincronizado o intercambiando mensajes.

## 7. Topología de subred

El diagrama presenta el gateway y hasta doce dispositivos. Los colores representan roles inferidos y dominios observados:

- gateway;
- PC host;
- robot Kinova/Raspberry Pi;
- ESP32/Micro-ROS;
- Domain 0;
- Domain 42;
- otros dominios con el color genérico DDS.

Las líneas y partículas animadas son una representación visual del estado de la tabla. No muestran rutas físicas, saltos IP ni cada paquete individual.

La clasificación de roles usa IP, hostname, fabricante inferido por OUI y patrones conocidos; debe considerarse una ayuda diagnóstica, no identificación criptográfica del dispositivo.

## 8. Tabla de dispositivos y filtros

### Columnas

| Columna | Significado |
|---|---|
| Rol / Dispositivo | Clasificación inferida del equipo. El símbolo `⚡` aparece cuando se observó DDS. |
| Dirección IP | IP desde la que se encontró o se observó el dispositivo. |
| MAC Address | Dirección obtenida de ARP; puede ser `N/A` si no está disponible. |
| Fabricante | OUI conocido y hostname, cuando puede resolverse. |
| Canal / Protocolo ROS 2 | Dominio observado, configuración local o estado desconocido. |
| Latencia | Resultado de `ping`; puede usar un valor de reserva si solo existe evidencia ARP/RTPS. |
| Estado | `Online` significa que el equipo fue incluido por ping, ARP o evidencia RTPS reciente. |

### Botones de filtro

| Botón | Elementos mostrados |
|---|---|
| `Todos` | Todos los dispositivos presentes en la tabla. |
| `Todos DDS` | Solo equipos con RTPS observado. |
| `Domain 0` | Equipos activos observados específicamente en Domain 0. |
| `Domain 42` | Equipos activos observados específicamente en Domain 42. |
| `Robots` | Elementos clasificados con rol `robot`, tengan o no DDS observado. |
| `Micro-ROS` | Elementos clasificados como `esp32`, tengan o no agente activo. |

Los filtros no ejecutan un nuevo escaneo; solo cambian la tabla y topología usando los datos ya cargados.

## 9. Gráfica de tráfico

La gráfica conserva aproximadamente las últimas 25 muestras del navegador y dibuja cuatro series:

- UDP general;
- ROS 2 DDS estimado;
- Micro-ROS estimado;
- TCP estimado.

Los valores bajo la gráfica son indicadores, no botones. La gráfica se redimensiona con la ventana y se actualiza desde el polling de un segundo.

Las series TCP/UDP/DDS/Micro-ROS son una distribución ponderada del tráfico total, no una clasificación de flujo basada en inspección profunda de paquetes.

## 10. Panel de puertos y servicios

El panel muestra como máximo quince entradas relevantes:

- puerto SPDP `7400 + 250 × Domain ID` cuando se observó RTPS;
- Micro-ROS Agent en UDP `8888`;
- DDS Discovery Server en `11811`.

También presenta el estado de la política distribuida. En WSL verifica, sin modificar el sistema, que las reglas `ROS2-Distributed-LAN-HyperV` y `ROS2-Distributed-LAN-Windows` permitan UDP entrante en cualquier puerto solamente desde `192.168.1.0/24`; comprueba además el bloqueo entrante predeterminado y alerta sobre reglas ROS/DDS heredadas abiertas a `Any`. El estado se refresca cada 30 segundos.

`OBSERVED` significa que llegó tráfico RTPS. `LISTENING` significa que se detectó un socket local. Son evidencias diferentes. El listado SPDP no intenta enumerar los puertos UDP dinámicos que CycloneDDS puede asignar para datos.

El observador evita abrir bloques DDS que se crucen con rangos UDP efímeros del sistema. En WSL también considera el rango dinámico habitual de Windows para no bloquear puertos de otras aplicaciones.

## 11. Tips de rendimiento

Las tarjetas de CycloneDDS y AP Isolation son recomendaciones estáticas:

- no cambian `RMW_IMPLEMENTATION`;
- no ingresan al panel del router;
- no modifican AP Isolation.

La persona participante debe aplicar esos cambios en el entorno ROS 2 o en la administración del router y después repetir las pruebas.

## 12. Configuraciones del proceso

### Argumentos

| Argumento | Predeterminado | Efecto |
|---|---:|---|
| `--host` | `0.0.0.0` | Interfaz donde escucha el servidor web. Use `127.0.0.1` para acceso exclusivamente local. |
| `--port` | `8080` | Primer puerto HTTP que intenta utilizar. |

### Variables de entorno

| Variable | Valor mostrado si no existe | Uso real |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | Configuración local del monitor y valor inicial del campo Domain ID. |
| `RMW_IMPLEMENTATION` | Texto recomendado `rmw_cyclonedds_cpp` | Se informa en `/api/status`; el monitor no selecciona ni inicia el RMW. |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | Texto `SUBNET` | Se informa en `/api/status`; el monitor no reconfigura nodos existentes. |
| `CYCLONEDDS_URI` | — | Perfil DDS del proyecto. El monitor no lo usa, pero los nodos ROS 2 sí. |
| `ROS_LAN_SUBNET` | `192.168.1.0/24` | Subred ROS contra la que se validan las reglas de firewall. Declárela solo si su red no es la del laboratorio; debe exportarse **antes** de lanzar el servidor. |

Los textos predeterminados de `RMW_IMPLEMENTATION` y `ROS_AUTOMATIC_DISCOVERY_RANGE` son referencias de la API, no variables exportadas automáticamente.

Ejemplo de arranque configurado:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export CYCLONEDDS_URI=file://$HOME/ros2_ws/src/burger_delivery/network_setup/cyclonedds.xml
python3 network_setup/monitor_red/server.py --host 127.0.0.1 --port 8080
```

## 13. Frecuencias y retención

| Función | Frecuencia o retención |
|---|---:|
| Recolección backend | Aproximadamente 1 segundo. |
| Consulta de tráfico del navegador | 1 segundo. |
| Consulta de benchmark | 1 segundo. |
| Consulta de dispositivos | 4 segundos. |
| Escaneo abreviado en segundo plano | 10 segundos. |
| Historial backend | 30 muestras. |
| Historial visible del navegador | 25 muestras. |
| Vigencia de una observación IP/dominio | 180 segundos. |

## 14. API disponible

| Método | Ruta | Función |
|---|---|---|
| `GET` | `/api/status` | Entorno y red detectada. |
| `GET` | `/api/firewall` | Cumplimiento de la política Windows/Hyper-V, en modo de solo lectura. |
| `GET` | `/api/devices` | Dispositivos y dominios por IP. |
| `GET` | `/api/traffic` | Métricas actuales e historial. |
| `POST` | `/api/scan` | Barrido completo de red. |
| `POST` | `/api/test_multicast` | Prueba multicast local. |
| `POST` | `/api/config` | Actualiza el Domain ID del monitor. |
| `GET` | `/api/benchmark/status` | Estado y promedios de la grabación. |
| `POST` | `/api/benchmark/start` | Inicia una grabación. |
| `POST` | `/api/benchmark/stop` | Detiene y guarda el CSV. |
| `GET` | `/api/benchmark/download` | Descarga el último CSV de la ejecución. |

## 15. Ejercicio de validación en la red del proyecto

1. Conecte los computadores a la misma subred del proyecto.
2. Exporte el mismo `ROS_DOMAIN_ID` en cada terminal antes de iniciar los nodos.
3. Inicie al menos un nodo ROS 2 en cada computador.
4. Abra el monitor y confirme `Firewall distribuido verificado`.
5. Espere varios anuncios SPDP.
6. Pulse `Escanear Red`.
7. Compruebe que cada PC aparezca con `domain_source=observed` representado como “DDS observado”.
8. Use el filtro del dominio y confirme que solo permanezcan los equipos esperados.
9. Compare con `ros2 node list` ejecutado bajo el mismo dominio.

Criterio: la persona participante puede explicar por qué un PC conectado pero sin tráfico RTPS aparece con dominio desconocido, y por qué un Domain ID configurado no demuestra actividad.

## 16. Diagnóstico rápido

| Síntoma | Interpretación probable | Acción |
|---|---|---|
| PC remoto con `Domain desconocido` | No llegó SPDP multicast, no hay nodo activo o el descubrimiento es unicast/Discovery Server. | Verifique nodos, dominio, firewall y rango de descubrimiento. |
| `Sin RTPS observado` en el PC local | El dominio está configurado, pero no se vio un anuncio válido. | Inicie un nodo ROS 2 y revise multicast. |
| Test multicast exitoso, pero no aparecen otros PCs | La prueba solo confirmó el loop local. | Ejecute una prueba multicast entre dos hosts y revise AP Isolation. |
| `Firewall distribuido no alineado` | Falta una regla, cambió la política predeterminada o existe una regla ROS/DDS abierta a cualquier red. | Consulte el detalle del indicador y aplique la sección 6 de `ROS2_NETWORK_CONFIG.md` desde PowerShell como administrador. |
| Gateway o subred inesperados | El host está conectado a otra red o una VPN cambió la ruta. | Revise `ip route` e `ip -o -4 addr`. |
| Domain “no observable” | Su bloque DDS cruza un rango UDP efímero. | Seleccione un dominio seguro para el sistema operativo. |
| Micro-ROS Agent inactivo | No se detectó UDP 8888 local. | Inicie el agente y confirme con `ss -lunp | rg ':8888'`. |
| Descarga deshabilitada | No se generó un CSV en esta ejecución. | Inicie y detenga un benchmark con al menos una muestra. |
| Métricas en cero | No hubo respuesta o no existe tráfico suficiente. | Contraste con `ping`, `ss`, `ros2 node list` y herramientas de captura. |

## 17. Archivos relacionados

| Archivo | Responsabilidad |
|---|---|
| `network_setup/iniciar_monitor.sh` | Lanzador y selección inicial de puerto. |
| `network_setup/monitor_red/server.py` | Servidor HTTP y endpoints. |
| `network_setup/monitor_red/device_scanner.py` | Descubrimiento IP, roles y clasificación de dominios. |
| `network_setup/monitor_red/traffic_sniffer.py` | Observación RTPS, métricas y CSV. |
| `network_setup/monitor_red/static/index.html` | Controles y estructura visual. |
| `network_setup/monitor_red/static/app.js` | Interacción, polling, filtros y gráfica. |
| `network_setup/MONITOR_UI_GUIA.md` | Descripción general de arquitectura y recorrido visual. |
| `network_setup/ROS2_NETWORK_CONFIG.md` | Configuración recomendada de ROS 2, DDS y firewall. |
