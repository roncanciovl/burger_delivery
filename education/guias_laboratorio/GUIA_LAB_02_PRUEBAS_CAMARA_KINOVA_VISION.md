# GUÍA DE LABORATORIO 02: PRUEBAS, COMPRESIÓN DE VIDEO Y DIAGNÓSTICO DISTRIBUIDO DEL MÓDULO DE VISIÓN KINOVA GEN3 CON CYCLONEDDS Y MONITOR DE RED

---

| FACULTAD | PROGRAMA | ASIGNATURA | SEMESTRE | CÓDIGO GUÍA | REVISIÓN |
|:---|:---|:---|:---:|:---:|:---:|
| Facultad de Ingeniería | Ingeniería Mecatrónica | ROBOT OPERATING SYSTEM - ROS | VIII – IX | GL-AA-F-1 / LAB-02 | 1.3 (2026-2) |

---

## 1. CONTROL DE CAMBIOS

| Descripción del Cambio | Justificación | Fecha |
|---|---|:---:|
| Integración de Monitor de Red Híbrido y Grabación del Experimento | Incorporación del panel web de telemetría en tiempo real (`monitor_red`), exportación de logs CSV de QoS y protocolo de grabación audiovisual obligatoria del experimento distribuido. | 17/08/2026 |

---

## 2. INTRODUCCIÓN

### 2.1. Contexto Teórico y Arquitectura de Flujo de Datos
En celdas robóticas colaborativas, la arquitectura de procesamiento visual se distribuye entre dos nodos: una estación fija conectada físicamente al brazo robótico que actúa como pasarela (**Dispositivo A: Gateway**) para adquisición de hardware, y una estación remota (**Dispositivo B: Procesamiento Wi-Fi**) que ejecuta algoritmos de visión por computador (detección de *AprilTags*, segmentación semántica o estimación de pose 3D).

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
          |                       [ Monitor de Red :8080 ]                          |
          |                       (Sniffer RTPS / Telemetría)                       |
          |                                  |                                      |
          |                                  |  === 2. CycloneDDS / Wi-Fi ===>      |
          |                                  |     (/camera/.../compressed)         |
          |                                  |     (/camera/camera_info)            |
          |                                  |     (/tf, /joint_states)             |
          |                                  |     Dashboard Web: http://IP:8080    |
          |                                  |                                      |
          |                                  |                          [ CycloneDDS Subscriber ]
          |                                  |                                      ↓
          |                                  |                          [ Detección AprilTags / IA ]
```

#### ¿Por qué el Kinova utiliza RTSP y cómo se integra con ROS 2?
El sensor óptico en la muñeca del Kinova Gen3 opera como un servidor RTSP nativo (puerto 554) transmitiendo video H.264. En el Gateway local (Dispositivo A), el driver decodifica este flujo, le estampa el reloj del sistema (`header.stamp`) y lo asocia al árbol cinemático (`TF2`). Luego, `image_transport` comprime dinámicamente el video a JPEG ($< 1.5\text{ MB/s}$, reduciendo $>90\%$ del ancho de banda) y lo distribuye mediante CycloneDDS sobre Wi-Fi.

#### Medición y Telemetría Científica con el Monitor de Red (`monitor_red`)
El proyecto incorpora un **Monitor de Red Híbrido** (`network_setup/iniciar_monitor.sh`) que levanta un servidor web en `http://localhost:8080`. Este monitor escucha pasivamente los anuncios SPDP de descubrimiento multicast, audita el tráfico RTPS de puertos DDS (7400–8000), grafica la latencia RTT y el jitter en tiempo real, y registra la telemetría experimental en archivos `.csv` para análisis cuantitativo riguroso.

---

## 3. OBJETIVOS

### 3.1. Objetivo General
Validar, optimizar y diagnosticar experimentalmente el flujo óptico del robot Kinova Gen3, integrando compresión de video (`image_transport`), configuración robusta de CycloneDDS para transmisión inalámbrica multi-dispositivo, auditoría de telemetría de red con el Monitor Web (`monitor_red`), grabación audiovisual del experimento y aplicación de un protocolo de depuración por capas.

### 3.2. Objetivos Específicos
1. **Auditar la calidad de red con el Monitor Web:** Lanzar `iniciar_monitor.sh` y evaluar en tiempo real métricas de tráfico RTPS, paquetes perdidos, latencia y dominios DDS activos, exportando los logs de telemetría en `.csv`.
2. **Validar streaming RTSP directo con OpenCV:** Ejecutar `test_kinova_camera.py` en color y profundidad para certificar que el hardware óptico y el enlace cableado están sanos antes de cargar el middleware.
3. **Implementar y evaluar compresión de video:** Configurar `image_transport` para publicar `/camera/color/image_raw/compressed`, cuantificando con `ros2 topic bw` el ahorro de ancho de banda frente al flujo crudo.
4. **Desplegar CycloneDDS sobre Wi-Fi:** Configurar `rmw_cyclonedds_cpp` y `cyclonedds.xml` en ambos dispositivos, verificando recepción remota fluida y visualización en tiempo real.
5. **Grabar y documentar el experimento:** Registrar un video continuo que demuestre la operación distribuida, las métricas del monitor web y la resolución de una falla inducida.

---

## 4. DESCRIPCIÓN DE LA PRÁCTICA

La práctica se estructura en seis fases:

```
  +---------------------------------------------------------------------------------------+
  |                                FASES DE LA PRÁCTICA                                   |
  +---------------------------------------------------------------------------------------+
  |  FASE 1: DIAGNÓSTICO DE RED Y PUESTA EN MARCHA DEL MONITOR DE RED (Dashboard :8080)   |
  |                                        ↓                                              |
  |  FASE 2: PASSTHROUGH VISUAL DIRECTO (RTSP Color/Depth, FFMPEG TCP vs UDP, Capturas)   |
  |                                        ↓                                              |
  |  FASE 3: COMPRESIÓN DE VIDEO EN ROS 2 (image_transport, Ahorro de Ancho de Banda)    |
  |                                        ↓                                              |
  |  FASE 4: PROCESAMIENTO DISTRIBUIDO SOBRE WI-FI CON CYCLONEDDS (Multi-Dispositivo)     |
  |                                        ↓                                              |
  |  FASE 5: PROTOCOLO DE DIAGNÓSTICO METÓDICO ANTE FALLAS INDUCIDAS POR CAPAS           |
  |                                        ↓                                              |
  |  FASE 6: GRABACIÓN DEL EXPERIMENTO Y EXPORTACIÓN DE TELEMETRÍA CSV                    |
  +---------------------------------------------------------------------------------------+
```

### 4.1. Resultados de Aprendizaje Evaluables (RAE) y Ponderación

| Criterio | RAE / Indicador Oficial | SO | Ponderación |
|---|---|:---:|:---:|
| **C1. Conectividad, compresión de video y ancho de banda** | **2.2.** Incorpora restricciones de red, latencia, ancho de banda y seguridad en hardware heterogéneo. | SO2 | 25% |
| **C2. Diagnóstico experimental de visión y protocolos por capas** | **6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos de diagnóstico por capas. | SO6 | 30% |
| **C3. Arquitectura distribuida con CycloneDDS y QoS** | **2.1.** Diseña soluciones de software integrando contratos QoS y redes DDS robustas. | SO2 | 20% |
| **C4. Documentación técnica, telemetría y grabación de video** | **3.1.** Elabora documentación técnica reproducible...<br>**3.3.** Comunica resultados experimentales con métricas y demostraciones... | SO3 | 15% |
| **C5. Seguridad, ética en captura visual y trabajo en equipo** | **4.3.** Aplica buenas prácticas en el uso de datos de cámara, operación segura y roles de equipo.<br>**5.1.** Reconoce habilidades técnicas y define roles... | SO4 / SO5 | 10% |
| **Total** | | | **100%** |

---

## 5. MATERIALES Y EQUIPOS

### 5.1. Equipos del Laboratorio (por grupo)
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Brazo manipulador Kinova Gen3 (7-DOF) con módulo de visión integrado en la muñeca | 1 | Unidad |
| Switch Gigabit Ethernet + Router Wi-Fi 6 | 1 | Unidad |
| Estación fija (Dispositivo A: Gateway Kinova) con Ubuntu 24.04 y ROS 2 Jazzy | 1 | Unidad |
| Pulsador de parada de emergencia física y cableado de alimentación | 1 | Unidad |

### 5.2. Equipos del Estudiante (por grupo)
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Laptop (Dispositivo B: Procesamiento Wi-Fi) con Ubuntu 24.04 LTS y ROS 2 Jazzy | 1 | Unidad |
| Cable de red UTP Categoría 6 (mínimo 2 metros) | 1 | Unidad |
| Repositorio `burger_delivery` con `image_transport`, `rmw_cyclonedds_cpp` y `monitor_red` | 1 | Repositorio |
| Software de captura de video de pantalla (OBS Studio, SimpleScreenRecorder o grabador de sistema) | 1 | Herramienta |

---

## 6. SEGURIDAD EN EL LABORATORIO

> [!WARNING]
> 1. **Área de Barrido:** Asegure un radio libre de 1.2 m alrededor del robot antes de energizarlo.
> 2. **Parada de Emergencia:** Compruebe la parada de emergencia antes de iniciar pruebas cinemáticas.
> 3. **Gestión de Cables:** Evite tensiones o torsiones en el cableado durante el movimiento del efector.
> 4. **Tratamiento Ético de Imágenes:** Las capturas de video deben restringirse a los objetos de calibración del laboratorio.

---

## 7. PROCEDIMIENTO EXPERIMENTAL

### Fase 1: Diagnóstico de Red y Puesta en Marcha del Monitor de Red

1. **Configuración de Enlaces:**
   - Dispositivo A (Gateway): Ethernet `192.168.1.100/24` (Kinova `192.168.1.10`) y Wi-Fi `192.168.50.10/24`.
   - Dispositivo B (Wi-Fi): Wi-Fi `192.168.50.20/24`.
2. **Lanzamiento del Monitor de Red Híbrido:**
   En el Dispositivo A, ejecute:
   ```bash
   bash ~/ros2_ws/src/burger_delivery/network_setup/iniciar_monitor.sh
   ```
3. **Acceso al Dashboard Web:**
   Abra el navegador en `http://localhost:8080` (en PC A) o `http://192.168.50.10:8080` (desde PC B).
   - Verifique el estado de la topología DDS, las interfaces detectadas y el tráfico RTPS en tiempo real.
   - En el panel de **Benchmark / Data Logger**, inicie una sesión de registro para capturar la telemetría de la práctica.

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
   En el Dispositivo A, inicie el driver de visión:
   ```bash
   ros2 launch burger_delivery robot.launch.py
   ```
2. **Inspección y Comparativa de Ancho de Banda (`ros2 topic bw`):**
   ```bash
   # Flujo crudo:
   ros2 topic bw /camera/color/image_raw

   # Flujo comprimido JPEG:
   ros2 topic bw /camera/color/image_raw/compressed
   ```
   - Calcule el ahorro porcentual ($> 90\%$) y anótelo en la **Tabla 3**.
   - Ajuste la calidad dinámicamente con `ros2 param set /camera/camera_node_driver jpeg_quality 80`.

---

### Fase 4: Despliegue y Verificación Distribuida sobre Wi-Fi con CycloneDDS

1. **Configuración de CycloneDDS en Ambos Equipos:**
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export ROS_DOMAIN_ID=15
   export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
   ```
2. **Recepción, Descompresión y Visualización en Dispositivo B:**
   Desde el Dispositivo B (conectado por Wi-Fi):
   ```bash
   # Medición de frecuencia remota:
   ros2 topic hz /camera/color/image_raw/compressed

   # Descompresión y visualización remota con image_view:
   ros2 run image_view image_view --ros-args --remap image:=/camera/color/image_raw _image_transport:=compressed
   ```
   - Observe en el **Monitor de Red** cómo se comporta la tasa de paquetes y el jitter durante el streaming.

---

### Fase 5: Protocolo de Diagnóstico Metódico por Capas ante Fallas Inducidas

Induzca y resuelva las siguientes fallas sistemáticas registrando el aislamiento en la **Tabla 5**:
1. **Falla 1 (Capa 1 - Red):** Desconexión o subred incorrecta en Wi-Fi.
2. **Falla 2 (Capa 2 - RTSP):** Credenciales erróneas en `test_kinova_camera.py`.
3. **Falla 3 (Capa 3 - CycloneDDS):** Conflicto de `ROS_DOMAIN_ID` o RMW diferente entre PC A y B.
4. **Falla 4 (Capa 4 - Compresión):** Suscripción al tópico crudo en Wi-Fi y saturación del canal.
5. **Falla 5 (Capa 5 - Web App):** Desactivación de la cámara en `http://192.168.1.10`.

---

### Fase 6: Grabación del Experimento y Exportación de Telemetría CSV

1. **Exportación del Log CSV del Monitor de Red:**
   - Desde el panel web (`http://localhost:8080`), detenga la sesión de grabación de telemetría y descargue el archivo `.csv` generado (`telemetria_red_lab02.csv`).
2. **Grabación Audiovisual del Experimento:**
   - Grabe un video continuo (duración sugerida: **2 a 4 minutos**) donde se muestre:
     - **Pantalla Dispositivo A:** Terminales del driver ROS 2, `image_transport` y el servidor del monitor de red.
     - **Pantalla Dispositivo B:** `ros2 topic hz`, ventana de `image_view` recibiendo el video por Wi-Fi y el dashboard web del monitor abierto.
     - **Demostración de Falla:** Inyección en vivo de una falla (ej. caída de frecuencia al suscribir video crudo o desconexión de dominio) y su posterior recuperación.
     - **Sustentación:** Breve explicación oral de los resultados obtenidos por los integrantes.
   - Aloje el video en OneDrive institucional o YouTube (enlace no listado) e incluya la URL en el informe y en el Instrumento ABET.

---

## 8. RESULTADOS DE LA PRÁCTICA

### Tabla 1: Caracterización de Enlaces de Red (ICMP Ping & Monitor de Red)
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

### Tabla 4: Despliegue Distribuido con CycloneDDS y Monitor de Red (Dispositivo B)
| Parámetro / Métrica | Valor Configurado / Medido en Dispositivo B | Comportamiento Observado en Dashboard Web |
|---|:---:|---|
| RMW Seleccionado | `rmw_cyclonedds_cpp` | Confirmado en `/api/status` |
| `ROS_DOMAIN_ID` | | Dominio único activo en sniffer |
| Tópico Remoto Suscrito | `/camera/color/image_raw/compressed` | Flujo estable detectado |
| Frecuencia Remota Recibida (`ros2 topic hz`) | | |
| Jitter Promedio Reportado en Monitor | | Gráfica de canvas estable |
| Archivo CSV de Telemetría Exportado | `telemetria_red_lab02.csv` | Registros totales: _____ |
| Enlace del Video del Experimento | URL: ____________________________ | Duración: _____ min |

### Tabla 5: Registro del Protocolo de Diagnóstico ante Fallas Inducidas
| Falla Inducida | Capa Afectada | Síntoma en Consola / Monitor Web | Método de Aislamiento | Acción Correctiva y Verificación |
|---|:---:|---|---|---|
| **Falla 1: Falla de red / IP** | Capa 1 (Red) | | | |
| **Falla 2: Credenciales RTSP** | Capa 2 (RTSP) | | | |
| **Falla 3: Conflicto RMW / Dominio**| Capa 3 (CycloneDDS) | | | |
| **Falla 4: Video crudo en Wi-Fi** | Capa 4 (Compresión) | | | |
| **Falla 5: Sensor apagado Web App** | Capa 5 (Sensor/Lógica)| | | |

---

## 9. ANÁLISIS DE RESULTADOS

1. **Análisis 1 (Compresión y Telemetría):** Con base en la Tabla 3 y las gráficas del Monitor de Red, analice cómo la compresión JPEG reduce el tráfico RTPS y mitiga el jitter en la red Wi-Fi.
2. **Análisis 2 (CycloneDDS en Wi-Fi):** Compare el comportamiento de CycloneDDS frente a FastDDS en redes inalámbricas utilizando los datos de retransmisiones y pérdida de paquetes registrados en el monitor.
3. **Análisis 3 (Latencia de Pipeline Distribuido):** Analice la cadena de retardos: Captura Kinova $\rightarrow$ Codificación H.264 $\rightarrow$ RTSP $\rightarrow$ ROS 2 Gateway $\rightarrow$ Compresión JPEG $\rightarrow$ Transmisión Wi-Fi $\rightarrow$ Descompresión en Dispositivo B.
4. **Análisis 4 (Diagnóstico Metódico):** Demuestre cómo el protocolo por capas y el uso de RTSP directo en la pasarela permitieron aislar fallas de middleware (DDS) de fallas de streaming y del sensor físico.

---

## 10. CONCLUSIONES

1. Conclusión técnica sobre la reducción de ancho de banda y viabilidad de streaming visual sobre Wi-Fi mediante `image_transport`.
2. Conclusión sobre la estabilidad y configuración de CycloneDDS en arquitecturas robóticas distribuidas multi-dispositivo.
3. Conclusión metodológica sobre la utilidad del Monitor de Red y la grabación audiovisual en la validación experimental reproducible.

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
