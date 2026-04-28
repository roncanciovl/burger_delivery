# Guía de Diagnóstico de Red - Burger Delivery

Esta guía proporciona un flujo de trabajo paso a paso para diagnosticar problemas de conectividad, latencia y rendimiento en el sistema ROS 2, desde chequeos básicos hasta análisis profundo de paquetes.

---

## 1. Niveles de Diagnóstico

Para resolver un problema de red, sigue este orden de herramientas locales:

| Nivel | Herramienta | Objetivo | Cuándo usarlo |
| :--- | :--- | :--- | :--- |
| **1** | `test_ros2_network.sh` | **Salud de ROS 2** | Si no ves tópicos o nodos en la lista. |
| **2** | `diagnostico_wifi.sh` / `.ps1` | **Calidad Física** | Si crees que la señal WiFi es débil. |
| **3** | `analisis_trafico_ros2.sh` | **Rendimiento** | Si hay lag, retraso o interferencia de otros dominios. |
| **4** | `test_wan_access.sh` | **Acceso WAN** | Si no puedes descargar paquetes o navegar. |

### Cómo elegir el script adecuado:
*   Usa **`test_ros2_network.sh`** para verificar que ROS 2 esté bien instalado, los puertos DDS locales estén abiertos y puedas publicar mensajes básicos.
*   Usa **`diagnostico_wifi.sh`** para medir estabilidad del link, latencia al router y congestión de canales (Linux nativo).
    *   **Nota para usuarios de WSL (Windows):** Debido a que WSL usa un adaptador de red virtual, no puede leer el estado físico del WiFi. Usa el script de PowerShell ejecutando: `powershell.exe -ExecutionPolicy Bypass -File diagnostico_wifi.ps1` desde la terminal de WSL (o directamente en Windows).
*   Usa **`analisis_trafico_ros2.sh`** para escanear la subred en busca de otros robots e identificar cuellos de botella de ancho de banda.
*   Usa **`test_wan_access.sh`** si tienes "falsa conexión" (tienes IP pero el router bloquea la salida a Internet o HTTPS).

---

## 2. Análisis y Pruebas en `analisis_trafico_ros2.sh`

El script `analisis_trafico_ros2.sh` es la herramienta de nivel 3 principal para explorar el rendimiento y latencia dentro de tu red local ROS 2. A continuación, se explican las pruebas que realiza:

### 0. Verificación de Variables de Entorno ROS 2
Comprueba la configuración de entorno para asegurar que utilices los parámetros correctos. Evalúa si la variable `RMW_IMPLEMENTATION` está ajustada a `rmw_cyclonedds_cpp` (recomendada para redes Wi-Fi) en lugar del default en Jazzy/Humble, y detecta tus identificadores de dominio (`ROS_DOMAIN_ID`) y rangos de descubrimiento.

### 1. Escaneo de Dominios Activos (DDS Discovery Scan)
Utiliza la herramienta de Linux `ss` para identificar qué puertos UDP del rango estándar DDS (7400 a 32000) están recibiendo tráfico. A partir de los puertos activos, calcula (con la fórmula inversa `(puerto - 7400)/250`) de manera explícita cuáles son los `DOMAIN_ID` circulando en la red. 
**Importancia**: Operar múltiples dominios en la misma red Wi-Fi genera problemas graves de estabilidad debido a la amplificación y cruzamiento excesivos del tráfico Multicast de descubrimiento.

### 2. Medición de Ancho de Banda (Bandwidth)
Muestra una lista de los tópicos ROS 2 activos y ejecuta automáticamente una evaluación de ancho de banda y consistencia de envío temporal (`ros2 topic bw`) sobre el primer tópico de usuario disponible.
**Importancia**: Determina con precisión práctica si los mensajes se propagan al ritmo ideal y si la infraestructura Wi-Fi local está soportando todo el *throughput* inyectado en la subred.

### 3. Diagnóstico de Causas de Latencia Física
Ejecuta verificaciones elementales a nivel estructural y de hardware en torno al router Wi-Fi:
- **Medición Ping hasta Gateway:** Computa el ping promedio a la puerta de enlace de red e ilustra el porcentaje acumulado de pérdida de paquetes, alertando sobre posible congestión inalámbrica del canal o interferencias perjudiciales al rango robótico.
- **Validación de Hardware de Red (Wi-Fi 6):** Identifica el driver controlador local (empleando `iw` nativamente en Linux o mediante `netsh.exe` bajo WSL) validando si existe soporte total para hardware bajo el robusto estándar IEEE 802.11ax, evitando así asimetrías como que tu equipo portátil contenga un transmisor anticuado interactuando contra el avanzado router Archer AX12 que limita el alcance.

### 4. Guía de Mitigación de Latencia y Recomendaciones
Extiende sugerencias paramétricas exactas, derivando en cómo anular los choques descritos previamente por medio de acciones como:
* Exportar tu entorno a CycloneDDS para lograr optimizaciones.
* Implementar mecanismos unicast potentes y enfocados al router (como Discovery Server) para erradicar las fallas iniciales causadas por ráfagas multicast de DDS.
* Alertar contra el Smart Connect o Band-Steering a través del portal LAN de tu Archer AX.
* **Manejo de Imágenes por Wi-Fi**: Las transmisiones de cámara (imágenes RAW) son la principal causa del colapso de colas UDP (saturando el MTU). Para solucionarlo, aquí hay sugerencias directas:
  1. **Compresión Activa**: Publica y suscríbete siempre usando compresión provista por `image_transport` (tópicos terminados en `/compressed` de tipo jpeg/png o formato `/theora`), NUNCA en texto o formato crudo (`sensor_msgs/Image`).
  2. **Perfil QoS a `Best Effort`**: Cambia la configuración de Quality of Service para que la red no intente retransmitir frenéticamente frames grandes perdidos, lo cual solo causa lag retrasado.
  3. **Límite de Resolución y FPS**: Desde los parámetros del driver de la cámara, dismunuye al mínimo estrictamente útil la resolución base y baja la tasa a, por ejemplo, 10-15 FPS antes de que pase a la pila de red.

---

## 3. Diagnóstico Visual y de Terceros

Si las herramientas locales no resuelven el problema, utiliza interfaces gráficas especializadas:
*   **Foxglove Studio**: Monitorización moderna de mensajes y estado del sistema.  
    [Sitio Web](https://foxglove.dev/)
*   **Wireshark (GUI)**: Inspección de paquetes con disectores específicos para DDS/RTPS.

---

## 4. Pruebas Avanzadas (Sniffing con tshark)

Cuando necesites ver exactamente qué bits están viajando por el aire, utiliza `tshark`.

### Instalación
```bash
sudo apt update && sudo apt install -y tshark
```
*(Selecciona **<Yes>** cuando pregunte por permisos de captura para usuarios no-root).*

### Comandos Críticos de Análisis
1. **Escaneo de Dominios Activos**:
   ```bash
   sudo tshark -i any -f "udp portrange 7400-8000" -T fields -e ip.src -e udp.dstport -e rtps.domain_id
   ```
2. **Ancho de Banda por Dispositivo**:
   ```bash
   sudo tshark -i any -f "udp portrange 7400-8000" -q -z conv,udp
   ```
3. **Inspección de Paquetes RTPS (Verbose)**:
   ```bash
   sudo tshark -i any -f "udp portrange 7400-8000" -Y "rtps" -V
   ```

### Síntomas a Detectar
*   **Conflictos de Dominio**: Tráfico en múltiples puertos base (7400, 7650, etc.) indica que otros usuarios están usando ROS 2 en la misma WiFi.
*   **Tormentas de Multicast**: Una IP enviando ráfagas masivas de paquetes de descubrimiento sin datos reales.
*   **Retransmisiones (ACKNACK)**: Muchos mensajes de este tipo en el modo Verboso indican que el WiFi está perdiendo paquetes físicamente.

---

## 5. Cómo Evaluar si las Optimizaciones Funcionan

Para comprobar empíricamente que los ajustes para mitigar latencia (compresión, QoS, FPS) y los cambios en la red (Discovery Server / CycloneDDS) fueron efectivos, usa estas 3 métricas integradas de ROS 2:

1. **Ahorro en Ancho de Banda (Bandwidth):**
   ```bash
   ros2 topic bw /image_raw/compressed
   ```
   *Éxito:* Deberías observar que el consumo final disminuyó drásticamente (ej. pasar de 60 MB/s a ~1.5 MB/s) y se mantiene moderadamente constante.

2. **Estabilidad de la Frecuencia (FPS Reales):**
   ```bash
   ros2 topic hz /image_raw/compressed
   ```
   *Éxito:* Verás una tasa promedio (`rate`) constante que coincide con tu límite. Si los hz fluctúan alocadamente o los mensajes llegan retrasados en ráfagas (efecto acordeón), significa que la red tiene caídas físicas y colas llenas.

3. **Latencia Absoluta del Mensaje (Delay):**
   *(Nota: Requiere que todos los equipos compartan la hora sincronizada mediante `chrony` o `ntp`)*
   ```bash
   ros2 topic delay /image_raw/compressed
   ```
   *Éxito:* Calcula la diferencia entre el instante en el robot captura la imagen (`header.stamp`) y cuando tu PC la desempaqueta. Si los promedios bajan sustancialmente y los picos de *Max delay* desaparecen, la optimización ha sido un éxito rotundo.

---

## 6. Resumen de Flujo de Trabajo
Si el robot se mueve con lag o video retrasado:
1. Ejecuta `./diagnostico_wifi.sh` (o `powershell.exe -ExecutionPolicy Bypass -File diagnostico_wifi.ps1` si estás en WSL) para descartar mala señal (nivel físico).
2. Ejecuta `./analisis_trafico_ros2.sh` para localizar cuellos de botella por dominios/multicast.
3. Utiliza la sección 5 (`ros2 topic hz/bw`) para evaluar la estabilidad de los mensajes críticos como el video.
4. Si aún se sufre, audita los paquetes puros con `tshark` para evidenciar interferencia o retransmisión de hardware en la señal Wi-Fi.
