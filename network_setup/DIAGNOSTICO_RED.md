# Guía de Diagnóstico de Red - Burger Delivery

Esta guía proporciona un flujo de trabajo paso a paso para diagnosticar problemas de conectividad, latencia y rendimiento en el sistema ROS 2, desde chequeos básicos hasta análisis profundo de paquetes.

---

## 1. Niveles de Diagnóstico

Para resolver un problema de red, sigue este orden de herramientas locales:

| Nivel | Herramienta | Objetivo | Cuándo usarlo |
| :--- | :--- | :--- | :--- |
| **1** | `test_ros2_network.sh` | **Salud de ROS 2** | Si no ves tópicos o nodos en la lista. |
| **2** | `diagnostico_wifi.sh` | **Calidad Física** | Si crees que la señal WiFi es débil. |
| **3** | `analisis_trafico_ros2.sh` | **Rendimiento** | Si hay lag, retraso o interferencia de otros dominios. |
| **4** | `test_wan_access.sh` | **Acceso WAN** | Si no puedes descargar paquetes o navegar. |

### Cómo elegir el script adecuado:
*   Usa **`test_ros2_network.sh`** para verificar que ROS 2 esté bien instalado, los puertos DDS locales estén abiertos y puedas publicar mensajes básicos.
*   Usa **`diagnostico_wifi.sh`** para medir estabilidad del link, latencia al router y congestión de canales.
*   Usa **`analisis_trafico_ros2.sh`** para escanear la subred en busca de otros robots e identificar cuellos de botella de ancho de banda.
*   Usa **`test_wan_access.sh`** si tienes "falsa conexión" (tienes IP pero el router bloquea la salida a Internet o HTTPS).

---

## 2. Diagnóstico Visual y de Terceros

Si las herramientas locales no resuelven el problema, utiliza interfaces gráficas especializadas:.//fastdds/monitor/monitor.html)
*   **Foxglove Studio**: Monitorización moderna de mensajes y estado del sistema.  
    [Sitio Web](https://foxglove.dev/)
*   **Wireshark (GUI)**: Inspección de paquetes con disectores específicos para DDS/RTPS.

---

## 3. Pruebas Avanzadas (Sniffing con tshark)

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

## 4. Resumen de Flujo de Trabajo
Si el robot se mueve con lag:
1. Ejecuta `./diagnostico_wifi.sh` para descartar mala señal.
2. Ejecuta `./analisis_trafico_ros2.sh` para ver interferencia de otros dominios.
3. Si el problema persiste, usa `tshark` para ver si hay pérdida de paquetes (retransmisiones).
