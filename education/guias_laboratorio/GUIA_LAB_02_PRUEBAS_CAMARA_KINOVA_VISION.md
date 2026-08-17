# GUÍA DE LABORATORIO 02: PRUEBAS, COMPRESIÓN DE VIDEO Y DIAGNÓSTICO DISTRIBUIDO DEL MÓDULO DE VISIÓN KINOVA GEN3 CON CYCLONEDDS

---

| FACULTAD | PROGRAMA | ASIGNATURA | SEMESTRE | CÓDIGO GUÍA | REVISIÓN |
|:---|:---|:---|:---:|:---:|:---:|
| Facultad de Ingeniería | Ingeniería Mecatrónica | ROBOT OPERATING SYSTEM - ROS | VIII – IX | GL-AA-F-1 / LAB-02 | 1.2 (2026-2) |

---

## 1. CONTROL DE CAMBIOS

| Descripción del Cambio | Justificación | Fecha |
|---|---|:---:|
| Creación e integración de compresión de video, CycloneDDS y justificación arquitectónica de RTSP | Diseño de la práctica experimental para pruebas de red, streaming RTSP directo en Gateway local, compresión con `image_transport`, transporte distribuido con CycloneDDS sobre Wi-Fi y diagnóstico por capas. | 17/08/2026 |

---

## 2. INTRODUCCIÓN

### 2.1. Contexto Teórico y Arquitectura de Flujo de Datos
En celdas robóticas colaborativas, la arquitectura de procesamiento visual suele distribuirse entre múltiples nodos de cómputo: una estación fija conectada físicamente al brazo robótico actúa como pasarela (*Gateway*) de sensores y actuadores, mientras que una estación remota (por ejemplo, una laptop o unidad móvil de cómputo conectada por Wi-Fi) procesa los algoritmos pesados de percepción visual (detección de *AprilTags*, segmentación semántica o estimación de pose 3D).

#### ¿Por qué el Kinova utiliza RTSP y cómo se integra con ROS 2?
El módulo de visión integrado en la muñeca del robot **Kinova Gen3 no es un nodo de ROS 2 nativo**. Internamente, el brazo opera un servidor multimedia embebido que transmite los fotogramas ópticos en formato H.264 mediante el protocolo estándar de la industria de video: **RTSP (Real-Time Streaming Protocol, puerto 554)** sobre las URLs `rtsp://192.168.1.10/color` y `rtsp://192.168.1.10/depth`.

Para que el sistema de percepción robótica opere de forma coordinada, la arquitectura divide el flujo en dos enlaces con propósitos y protocolos diferenciados:

```
+---------------------------------------------------------------------------------------------------+
|                            ARQUITECTURA DISTRIBUIDA DEL FLUJO DE VISIÓN                           |
+---------------------------------------------------------------------------------------------------+

 [ Robot Kinova Gen3 ]            [ Dispositivo A: Gateway ]             [ Dispositivo B: Wi-Fi ]
 (Servidor RTSP H.264)            (Conectado por Ethernet)               (Estación Procesamiento)
     192.168.1.10                      192.168.1.100                          192.168.50.20
          |                                  |                                      |
          |  === 1. Enlace RTSP / H.264 ===> |                                      |
          |      (Cable Ethernet Gigabit)    |                                      |
          |                                  |                                      |
          |                       [ ros2_kortex_vision ]                            |
          |                                  ↓ (Decodifica H.264)                   |
          |                       [ Ingesta de Stamps y TF2 ]                       |
          |                                  ↓ (Añade camera_color_frame)           |
          |                       [ image_transport ]                               |
          |                                  ↓ (Comprime dinámico JPEG ~1.5 MB/s)   |
          |                       [ CycloneDDS Publisher ]                          |
          |                                  |                                      |
          |                                  |  === 2. CycloneDDS / Wi-Fi ===>      |
          |                                  |     (/camera/.../compressed)         |
          |                                  |     (/camera/camera_info)            |
          |                                  |     (/tf, /joint_states)             |
          |                                  |                                      |
          |                                  |                          [ CycloneDDS Subscriber ]
          |                                  |                                      ↓
          |                                  |                          [ Detección AprilTags / IA ]
```

#### Comparativa de Roles en la Arquitectura

| Componente / Mecanismo | Dónde Opera | Protocolo / Transporte | Función Técnica en la Celda |
|---|---|---|---|
| **RTSP directo (`test_kinova_camera.py`)** | Kinova $\rightarrow$ Dispositivo A | TCP en puerto 554 (H.264 nativo) | **Diagnóstico de Base:** Valida que el sensor físico y el enlace Ethernet local están 100% operativos sin depender de ROS 2, RViz2 ni Wi-Fi. |
| **Driver ROS 2 (`ros2_kortex_vision`)** | En Dispositivo A (Gateway) | Puente RTSP $\rightarrow$ ROS 2 | **Ingesta y Sincronización:** Decodifica H.264, añade estampas de tiempo (`header.stamp`) y vincula el marco de coordenadas al árbol cinemático (`TF2`). |
| **Adaptador `image_transport`** | En Dispositivo A (Gateway) | Compresión JPEG dinámica | **Optimización:** Reduce el consumo de ancho de banda de $\sim 25\text{ MB/s}$ a $< 1.5\text{ MB/s}$ ($>90\%$ de ahorro). |
| **CycloneDDS (`rmw_cyclonedds_cpp`)** | Dispositivo A $\rightarrow$ Dispositivo B | DDS RTPS sobre Wi-Fi (`cyclonedds.xml`) | **Distribución Robótica:** Transporta el video comprimido, la calibración (`CameraInfo`) y transformaciones (`TF2`) hacia la estación remota de forma tolerante a caídas de paquetes. |

---

### 2.2. ¿Por qué NO se envía RTSP directo por Wi-Fi al Dispositivo B?
En una celda de manufactura real, enviar el flujo RTSP directamente por Wi-Fi a la estación de procesamiento sería un error arquitectónico por tres razones críticas:
1. **Pérdida de Sincronización Temporal (`header.stamp`):** Un stream RTSP plano no contiene la estampa de tiempo del reloj del sistema de control del robot. Sin esto, el Dispositivo B no puede sincronizar el instante exacto de captura con la posición de las articulaciones (`/joint_states`) para planificar movimientos con MoveIt 2.
2. **Pérdida del Árbol de Transformaciones (TF2) y Calibración:** Los algoritmos de localización (*AprilTags*, estimación 3D) requieren la matriz de calibración intrínseca (`sensor_msgs/msg/CameraInfo`) y la transformación extrínseca respecto a la base del robot (`base_link -> bracelet_link -> camera_color_frame`). El transporte ROS 2 sobre CycloneDDS garantiza que la imagen viaje vinculada a su contexto espacial.
3. **Seguridad y Aislamiento de Red:** El Kinova debe residir en una subred industrial cableada y protegida (`192.168.1.0/24`). Exponer el puerto RTSP del robot directamente a la red Wi-Fi general (`192.168.50.0/24`) introduce vulnerabilidades de seguridad y congestión innecesaria en el procesador embebido del brazo.

### 2.3. Importancia del Diagnóstico por Capas
Cuando la estación remota en Wi-Fi experimenta congelamiento o descarte de fotogramas, existen múltiples causas posibles (hardware del sensor, cable Ethernet, decodificador de ROS 2, compresión de imagen, multidifusión de DDS o sobrecarga en el algoritmo de visión). **El visor RTSP directo (`test_kinova_camera.py`) es la herramienta metodológica para aislar las capas 1 y 2 en 5 segundos**, permitiendo comprobar el hardware y la red física antes de investigar capas superiores de middleware o procesamiento.

---

## 3. OBJETIVOS

### 3.1. Objetivo General
Validar, optimizar y diagnosticar experimentalmente el flujo óptico del robot Kinova Gen3, integrando pruebas de streaming RTSP directo en la pasarela local, compresión de video mediante `image_transport`, configuración robusta de CycloneDDS para transmisión inalámbrica multi-dispositivo y aplicación de un protocolo de depuración por capas ante fallas inducidas.

### 3.2. Objetivos Específicos
1. **Caracterizar la capa de red cableada e inalámbrica:** Medir cuantitativamente RTT, jitter y pérdida de paquetes en el enlace local (Dispositivo A $\rightarrow$ Kinova) y en el canal Wi-Fi (Dispositivo B $\rightarrow$ Dispositivo A).
2. **Validar streaming RTSP directo con OpenCV:** Ejecutar el visor directo sin ROS (`test_kinova_camera.py`) en color y profundidad, evaluando transporte FFMPEG TCP vs UDP y capturando imágenes para calibración.
3. **Implementar y evaluar compresión de video en ROS 2:** Configurar `image_transport` para publicar `/camera/color/image_raw/compressed`, comparando cuantitativamente el consumo de ancho de banda (`ros2 topic bw`) y la tasa de cuadros (`ros2 topic hz`) frente al flujo crudo.
4. **Desplegar y verificar CycloneDDS en entorno multi-PC sobre Wi-Fi:** Configurar `rmw_cyclonedds_cpp` y `cyclonedds.xml` en ambos dispositivos, verificando la recepción remota, descompresión y sincronización del video en el nodo de procesamiento.
5. **Diagnosticar fallas inducidas por capas:** Aislar y resolver fallas en red física, streaming RTSP, middleware CycloneDDS, compresión de imagen y configuración del sensor en la interfaz web de Kinova.

---

## 4. DESCRIPCIÓN DE LA PRÁCTICA

La práctica se organiza en cinco fases de experimentación y validación técnica:

```
  +---------------------------------------------------------------------------------------+
  |                                FASES DE LA PRÁCTICA                                   |
  +---------------------------------------------------------------------------------------+
  |  FASE 1: DIAGNÓSTICO EN CAPA DE RED (ICMP Ping, RTT, Subred y Enlace Wi-Fi)          |
  |                                        ↓                                              |
  |  FASE 2: PASSTHROUGH VISUAL DIRECTO (RTSP Color/Depth, FFMPEG TCP vs UDP, Capturas)   |
  |                                        ↓                                              |
  |  FASE 3: COMPRESIÓN DE VIDEO EN ROS 2 (image_transport, Ahorro de Ancho de Banda)    |
  |                                        ↓                                              |
  |  FASE 4: PROCESAMIENTO DISTRIBUIDO SOBRE WI-FI CON CYCLONEDDS (Multi-Dispositivo)     |
  |                                        ↓                                              |
  |  FASE 5: DIAGNÓSTICO METÓDICO POR CAPAS ANTE FALLAS INDUCIDAS Y RECUPERACIÓN          |
  +---------------------------------------------------------------------------------------+
```

### 4.1. Resultados de Aprendizaje Evaluables (RAE) y Ponderación

| Criterio | RAE / Indicador Oficial | SO | Ponderación |
|---|---|:---:|:---:|
| **C1. Conectividad, compresión de video y ancho de banda** | **2.2.** Incorpora restricciones de red, latencia, ancho de banda y seguridad en la integración de hardware heterogéneo (brazos robóticos, sensores, micro-ROS). | SO2 | 25% |
| **C2. Diagnóstico experimental de visión y protocolos por capas** | **6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos de diagnóstico por capas (Sintaxis -> TF -> Red -> Lógica) para aislar errores en hardware y software.<br>*(Apoyo en **6.2** variables de red y parámetros de cámara)* | SO6 | 30% |
| **C3. Arquitectura distribuida con CycloneDDS y QoS** | **2.1.** Diseña soluciones de software para control y monitoreo de robots, integrando contratos de comunicación (QoS, interfaces customizadas) y redes DDS robustas. | SO2 | 20% |
| **C4. Documentación técnica, reproducibilidad y capturas** | **3.1.** Elabora documentación técnica reproducible del sistema ROS 2...<br>**3.3.** Comunica resultados experimentales de percepción, calibración y planificación... | SO3 | 15% |
| **C5. Seguridad, ética en captura visual y trabajo en equipo** | **4.3.** Aplica buenas prácticas de responsabilidad profesional en el uso de datos de cámara, licencias de software, operación segura del robot y documentación de limitaciones.<br>**5.1.** Reconoce habilidades técnicas y define roles... | SO4 / SO5 | 10% |
| **Total** | | | **100%** |

---

## 5. MATERIALES Y EQUIPOS

### 5.1. Equipos del Laboratorio (por puesto de trabajo / grupo)

| DESCRIPCIÓN (Material, reactivo, instrumento, software, hardware, equipo) | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Brazo manipulador Kinova Gen3 (7-DOF) con módulo de visión integrado en la muñeca | 1 | Unidad |
| Switch Gigabit Ethernet + Access Point Router Wi-Fi 6 con soporte multicast | 1 | Unidad |
| Estación fija (Dispositivo A: Gateway Kinova) con Ubuntu 24.04 LTS y ROS 2 Jazzy | 1 | Unidad |
| Pulsador de parada de emergencia física y cableado de alimentación | 1 | Unidad |

### 5.2. Equipos del Estudiante (por grupo)

| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Laptop (Dispositivo B: Estación de Procesamiento Wi-Fi) con Ubuntu 24.04 LTS y ROS 2 | 1 | Unidad |
| Cable de red UTP Categoría 6 (mínimo 2 metros) | 1 | Unidad |
| Repositorio `burger_delivery` con paquetes `image_transport`, `image_transport_plugins` y `rmw_cyclonedds_cpp` | 1 | Repositorio |

---

## 6. SEGURIDAD EN EL LABORATORIO

> [!WARNING]
> **Normas de Seguridad Física y Operacional:**
> 1. **Área de Barrido:** Asegure un radio de seguridad de 1.2 m libre de obstáculos alrededor del brazo robótico.
> 2. **Parada de Emergencia:** Verifique el funcionamiento del botón de parada de emergencia antes de energizar los actuadores.
> 3. **Gestión de Cables:** Compruebe que los cables de alimentación y Ethernet no se enrollen ni tensionen con las articulaciones de la muñeca.
> 4. **Tratamiento Ético de Imágenes:** Las capturas ópticas deben limitarse estrictamente a los objetos de prueba del laboratorio.

---

## 7. PROCEDIMIENTO EXPERIMENTAL

### Fase 1: Diagnóstico en Capa de Red e Interfaces (ICMP & Enlace Wi-Fi)

1. **Configuración del Dispositivo A (Gateway):**
   - Interfaz cableada (`eth0`): IP estática `192.168.1.100/24` (comunicación directa con Kinova en `192.168.1.10`).
   - Interfaz inalámbrica (`wlan0`): Conectada a la red Wi-Fi del laboratorio (ej. `192.168.50.10/24`).
2. **Configuración del Dispositivo B (Procesamiento Wi-Fi):**
   - Interfaz inalámbrica (`wlan0`): Conectada a la misma red Wi-Fi (ej. `192.168.50.20/24`).
3. **Prueba de Latencia ICMP:**
   ```bash
   # En Dispositivo A -> Hacia el Kinova:
   ping 192.168.1.10 -c 10

   # En Dispositivo B -> Hacia Dispositivo A (Wi-Fi):
   ping 192.168.50.10 -c 10
   ```
   *Criterios cuantitativos:* RTT Ethernet $< 5\text{ ms}$; RTT Wi-Fi $< 15\text{ ms}$ con jitter $< 3\text{ ms}$.

---

### Fase 2: Passthrough Óptico Directo por RTSP (Sin ROS 2)

1. **Prueba de Video RGB y Depth en Gateway Local:**
   En el Dispositivo A, ejecute el script directo de validación óptica:
   ```bash
   python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_camera.py --ip 192.168.1.10 --stream color
   ```
   - Verifique fluidez ($\ge 25\text{ FPS}$) y transporte FFMPEG TCP (`rtsp_transport=tcp`, `nobuffer`, `low_delay`).
   - Pruebe el stream de profundidad con `--stream depth`.
   - Realice 3 capturas fotográficas de calibración con la tecla **`s`** (`kinova_capture_*.png`) y salga con **`q`**.

---

### Fase 3: Compresión de Video en ROS 2 (`image_transport`) y Ahorro de Ancho de Banda

1. **Lanzamiento del Nodo Publicador de Visión con Compresión:**
   En el Dispositivo A, inicie el driver de visión asegurando la carga de los plugins de transporte de imagen:
   ```bash
   ros2 launch burger_delivery robot.launch.py
   ```
2. **Inspección de Tópicos Crudos vs. Comprimidos:**
   Verifique la disponibilidad del flujo crudo y del flujo comprimido:
   ```bash
   ros2 topic list | grep camera
   # Debe mostrar:
   # /camera/color/image_raw
   # /camera/color/image_raw/compressed
   ```
3. **Comparativa Cuantitativa de Ancho de Banda (`ros2 topic bw`):**
   ```bash
   # Medición del flujo crudo sin comprimir:
   ros2 topic bw /camera/color/image_raw

   # Medición del flujo comprimido en JPEG:
   ros2 topic bw /camera/color/image_raw/compressed
   ```
   - Anote los valores en la **Tabla 3**. Calcule el porcentaje exacto de ahorro de ancho de banda:
     $$\text{Ahorro (\%)} = \left( 1 - \frac{\text{BW}_{\text{comprimido}}}{\text{BW}_{\text{crudo}}} \right) \times 100\%$$
4. **Ajuste Dinámico de Parámetros de Compresión JPEG:**
   Ajuste la calidad JPEG dinámicamente y evalúe el impacto en el ancho de banda y la calidad visual:
   ```bash
   ros2 param set /camera/camera_node_driver format "jpeg"
   ros2 param set /camera/camera_node_driver jpeg_quality 80
   ros2 param set /camera/camera_node_driver jpeg_quality 30
   ```

---

### Fase 4: Despliegue y Verificación Distribuida sobre Wi-Fi con CycloneDDS

1. **Configuración de CycloneDDS en Ambos Dispositivos:**
   En **Dispositivo A** y **Dispositivo B**, exporte las variables de entorno para usar CycloneDDS en el mismo dominio:
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export ROS_DOMAIN_ID=15  # Utilice el ID asignado a su equipo
   ```
2. **Configuración de `cyclonedds.xml` para Enlace Wi-Fi:**
   Cree un archivo de configuración `cyclonedds.xml` para forzar la interfaz Wi-Fi y optimizar los búferes de red:
   ```xml
   <?xml version="1.0" encoding="UTF-8" ?>
   <CycloneDDS xmlns="https://cdds.io/config">
       <Domain id="any">
           <General>
               <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
               <AllowMulticast>true</AllowMulticast>
               <MaxMessageSize>65500B</MaxMessageSize>
           </General>
           <Discovery>
               <ParticipantIndex>auto</ParticipantIndex>
           </Discovery>
       </Domain>
   </CycloneDDS>
   ```
   Exporte la configuración en ambos equipos:
   ```bash
   export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
   ```
3. **Recepción, Descompresión y Visualización en Dispositivo B:**
   Desde el Dispositivo B (conectado por Wi-Fi), verifique la recepción del video comprimido:
   ```bash
   # Medición de frecuencia remota:
   ros2 topic hz /camera/color/image_raw/compressed

   # Descompresión y visualización remota con image_view:
   ros2 run image_view image_view --ros-args --remap image:=/camera/color/image_raw _image_transport:=compressed
   ```
   - Verifique que la tasa de cuadros en el Dispositivo B se mantenga estable ($\ge 20\text{ Hz}$) sin congelamiento.

---

### Fase 5: Protocolo de Diagnóstico Metódico por Capas ante Fallas Inducidas

Cada equipo debe inducir y resolver las siguientes fallas sistemáticas, registrando el diagnóstico en la **Tabla 5**:

```
+-----------------------------------------------------------------------------------------+
|                      PROTOCOLO DE DIAGNÓSTICO METÓDICO POR CAPAS                        |
+-----------------------------------------------------------------------------------------+
| [CAPA 1: RED / ENLACE]     -> ¿Hay enlace Ethernet/Wi-Fi? ¿Responde ping ICMP?          |
| [CAPA 2: RTSP STREAMING]   -> ¿Responde el puerto 554? ¿Las credenciales son válidas?   |
| [CAPA 3: CYCLONEDDS / RMW] -> ¿Mismo ROS_DOMAIN_ID? ¿RMW_IMPLEMENTATION coincide?       |
| [CAPA 4: COMPRESIÓN / QoS] -> ¿El tópico comprimido existe? ¿QoS SensorData compatible? |
| [CAPA 5: LÓGICA / SENSOR]  -> ¿El sensor está habilitado en la Web App del Kinova?      |
+-----------------------------------------------------------------------------------------+
```

1. **Falla 1 (Capa 1 - Red):** Desconecte el enlace Wi-Fi o asigne una IP fuera de subred en Dispositivo B.
2. **Falla 2 (Capa 2 - RTSP):** Ingrese credenciales erróneas en el script de prueba.
3. **Falla 3 (Capa 3 - CycloneDDS):** Configure un `ROS_DOMAIN_ID` distinto o RMW diferente (`rmw_fastrtps_cpp` vs `rmw_cyclonedds_cpp`) entre Dispositivo A y B. Observe el bloqueo de descubrimiento de tópicos.
4. **Falla 4 (Capa 4 - Compresión):** Suscríbase al flujo crudo `/camera/color/image_raw` sobre Wi-Fi y observe la caída drástica de FPS y saturación de ancho de banda frente al flujo comprimido.
5. **Falla 5 (Capa 5 - Web App):** Desactive la cámara desde `http://192.168.1.10` y verifique los logs de error del driver.

---

## 8. RESULTADOS DE LA PRÁCTICA

### Tabla 1: Caracterización de Enlaces de Red (ICMP Ping)
| Enlace Evaluado | IP Origen / Destino | Paquetes (Tx/Rx) | RTT Mínimo (ms) | RTT Promedio (ms) | RTT Máximo (ms) | Jitter (mdev) | Estado |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Cableado (Disp. A -> Kinova)** | `192.168.1.100` -> `192.168.1.10` | 10 / ___ | | | | | |
| **Inalámbrico (Disp. B -> Disp. A)** | `192.168.50.20` -> `192.168.50.10` | 10 / ___ | | | | | |

### Tabla 2: Rendimiento del Visor RTSP Directo (`test_kinova_camera.py`)
| Stream / Configuración | Backend Activo | FPS Medido | Latencia Percibida | Estabilidad Visual |
|---|:---:|:---:|:---:|:---:|
| **Color RGB (`--stream color`)** | FFMPEG TCP | | | |
| **Profundidad (`--stream depth`)** | FFMPEG TCP | | | |
| **Color con transporte UDP** | FFMPEG UDP | | | |
| **Captura Guardada (PNG)** | Archivo: `kinova_capture_1.png` | Resolución: | Tamaño: ____ KB | |

### Tabla 3: Comparativa de Ancho de Banda: Video Crudo vs. Video Comprimido
| Formato de Video en ROS 2 | Nombre del Tópico | Ancho de Banda Medido (`ros2 topic bw`) | Tasa de Cuadros (`ros2 topic hz`) | Ahorro de Ancho de Banda (%) |
|---|---|:---:|:---:|:---:|
| **Video Crudo (RGB8)** | `/camera/color/image_raw` | | | $0\%$ (Referencia) |
| **Comprimido JPEG ($q=80$)** | `/camera/color/image_raw/compressed` | | | |
| **Comprimido JPEG ($q=30$)** | `/camera/color/image_raw/compressed` | | | |

### Tabla 4: Despliegue Distribuido con CycloneDDS sobre Wi-Fi (Dispositivo B)
| Parámetro en Dispositivo B | Valor Configurado / Medido | Comportamiento Observado |
|---|:---:|---|
| RMW Seleccionado | `rmw_cyclonedds_cpp` | |
| `ROS_DOMAIN_ID` | | |
| Tópico Remoto Suscrito | `/camera/color/image_raw/compressed` | |
| Frecuencia Remota Recibida (`ros2 topic hz`) | | |
| Fluidez de Video Descomprimido | | |

### Tabla 5: Registro del Protocolo de Diagnóstico ante Fallas Inducidas
| Falla Inducida | Capa Afectada | Síntoma en Consola / GUI | Método de Aislamiento | Acción Correctiva y Verificación |
|---|:---:|---|---|---|
| **Falla 1: Falla de red / IP** | Capa 1 (Red) | | | |
| **Falla 2: Credenciales RTSP** | Capa 2 (RTSP) | | | |
| **Falla 3: Conflicto RMW / Dominio**| Capa 3 (CycloneDDS) | | | |
| **Falla 4: Video crudo en Wi-Fi** | Capa 4 (Compresión) | | | |
| **Falla 5: Sensor apagado Web App** | Capa 5 (Sensor/Lógica)| | | |

---

## 9. ANÁLISIS DE RESULTADOS

1. **Análisis 1 (Impacto de la Compresión):** Explique con base en los datos de la Tabla 3 por qué transmitir video crudo en Wi-Fi colapsa el canal y cómo la compresión JPEG permite una tasa de cuadros estable sin degradar significativamente la detección de marcadores.
2. **Análisis 2 (Ventajas de CycloneDDS en Wi-Fi):** Compare el comportamiento de CycloneDDS frente a FastDDS en redes con descarte de paquetes inalámbricos. ¿Por qué es crítico configurar `NetworkInterfaceAddress` en `cyclonedds.xml`?
3. **Análisis 3 (Latencia de Pipeline Distribuido):** Analice la cadena de retardos: Captura Kinova $\rightarrow$ Codificación H.264 $\rightarrow$ RTSP $\rightarrow$ ROS 2 Gateway $\rightarrow$ Compresión JPEG $\rightarrow$ Transmisión Wi-Fi $\rightarrow$ Descompresión en Dispositivo B.
4. **Análisis 4 (Metodología de Diagnóstico):** Demuestre cómo el protocolo por capas y el uso de RTSP directo en la pasarela permitieron aislar fallas de middleware (DDS) de fallas de streaming y del sensor físico.

---

## 10. CONCLUSIONES

1. Conclusión técnica sobre la reducción de ancho de banda y viabilidad de streaming visual sobre Wi-Fi mediante `image_transport`.
2. Conclusión sobre la estabilidad y configuración de CycloneDDS en arquitecturas robóticas distribuidas multi-dispositivo.
3. Conclusión metodológica sobre el papel del diagnóstico directo por RTSP en el aislamiento de fallas físicas antes de la integración en ROS 2.

---

## 11. PREGUNTAS PARA LA DISCUSIÓN

1. **Pregunta 1:** Si se incrementa la resolución a 1080p, ¿qué compromiso existe entre la carga de CPU de compresión en el Gateway y el ancho de banda consumido en Wi-Fi?
2. **Pregunta 2:** ¿Por qué en CycloneDDS es fundamental ajustar `MaxMessageSize` cuando se transmiten paquetes UDP de video comprimido?
3. **Pregunta 3:** ¿Por qué en una arquitectura robótica distribuida NO se recomienda consumir el stream RTSP directamente desde la estación Wi-Fi sin pasar por el nodo de ROS 2 en el Gateway? *(Considere estampas de tiempo `header.stamp`, marcos `TF2` y aislamiento de subredes).*

---

## 12. BIBLIOGRAFÍA

1. Kinova Robotics. (2024). *Kinova Gen3 Ultra lightweight robot User Guide.* Kinova Inc.
2. Eclipse Foundation. (2024). *Eclipse Cyclone DDS Documentation and Configuration Guide.* https://cyclonedds.io/
3. ROS 2 Design & Documentation. (2024). *image_transport and Compressed Image Transport in ROS 2.* https://docs.ros.org/
4. Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming Robots with ROS.* O'Reilly Media.
5. Lentin, J. (2024). *ROS 2 Robotics Developer Guide: Real-world robotics projects with ROS 2 Jazzy.* Packt Publishing.

---

## 13. APROBACIÓN DE LA GUÍA DE LABORATORIO

| Elaborado por: | Revisado por: | Aprobado por: |
|:---:|:---:|:---:|
| **Ing. Henry Roncancio**<br>Docente Asignatura ROS | **Director de Programa**<br>Ingeniería Mecatrónica | **Decano(a)**<br>Facultad de Ingeniería |
