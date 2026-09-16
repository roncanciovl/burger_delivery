# Instrumento de Evidencia y Calificación — Taller: Integración de micro-ROS en ESP32 para Plataformas Móviles y Drones

> **Documento Único de Entrega Estudiantil y Evaluación ABET.** Asociado a `education/talleres/TALLER_MICROROS_ESP32_ROBOTICA_MOVIL.md`.  
> Este formato integra la entrega formal por equipos: registro de evidencias, tablas de datos experimentales diligenciadas durante la práctica (transportes Serial y Wi-Fi UDP), cuestionario de análisis técnico con respuestas justificadas, Anexo A de comprobación individual para sustentar el logro ABET de cada estudiante y rúbricas analíticas docentes.

---

## 1. Identificación y Control

| Campo | Registro Oficial |
|---|---|
| **Programa Académico** | Ingeniería Mecatrónica |
| **Asignatura** | ROBOT OPERATING SYSTEM - ROS |
| **Periodo Académico** | 2026-2 |
| **Corte / Instrumento** | Primer Corte / Talleres y tareas |
| **Actividad Evaluada** | Taller — Integración de micro-ROS en ESP32 para Plataformas Móviles y Drones (Serial y WiFi UDP) |
| **Número de Grupo / Subgrupo** | |
| **Estudiante 1 (Nombre y Código)** | |
| **Estudiante 2 (Nombre y Código)** | |
| **Estudiante 3 (si aplica)** | |
| **Namespace Asignado al Robot** | `/burger_car_` &nbsp;&nbsp;&nbsp;&nbsp; *(ej. `/burger_car_01`)* |
| **Hardware Embebido Utilizado** | ESP32 NodeMCU ☐ &nbsp;&nbsp;&nbsp;&nbsp; ESP32-S3 ☐ &nbsp;&nbsp;&nbsp;&nbsp; ESP32-WROOM ☐ |
| **Nombre del Archivo de Entrega** | `C1_T_MICROROS_ESP32_G<grupo>_<codigo1>_<codigo2>_v1.docx` |
| **Fecha de Realización en Laboratorio** | |
| **Fecha de Entrega del Documento** | |
| **Docente Evaluador** | Ing. Henry Roncancio |
| **Versión del Instrumento** | Versión 1.0 (2026-2) |
| **Unidad de Análisis / Captura** | Equipo colaborativo con comprobación individual (Anexo A) |

---

## 2. Parámetros de Assessment

| Parámetro | Regla Institucional y Metodológica Adoptada |
|---|---|
| **Población o cohorte** | Censo completo de estudiantes matriculados que presentan el taller de micro-ROS en ESP32 en el periodo 2026-2. |
| **Momento de medición** | Primer corte, tras la ejecución de las fases de transporte serial, WiFi UDP, teleoperación y diagnóstico de red. |
| **Evaluador** | Docente titular de la asignatura ROBOT OPERATING SYSTEM - ROS. |
| **Umbral individual de logro** | Nivel **N3 o superior** (puntaje mínimo de 300 sobre 500) en cada uno de los indicadores evaluados. |
| **Meta de cohorte** | Al menos el **70% de los estudiantes evaluables** debe alcanzar el nivel N3 o superior en cada indicador. |
| **Regla de muestreo** | Censo 100%: no se utiliza muestreo; se evalúa la totalidad de los estudiantes y equipos exigibles. |
| **Evidencia faltante** | Entrega exigible sin evidencia obligatoria verificable: se califica en N1 con puntaje 0. Retiro oficial: `NA / no evaluado`. |
| **Regla de trabajo en equipo** | Informe único por equipo con tablas experimentales llenas más Anexo A individual obligatorio para cada integrante. |

---

## 3. Niveles de Desempeño para Zubatronic/SGDE

| Nivel | Intervalo Zubatronic | Valor Guía | Interpretación y Criterio de Logro |
|---|---:|:---:|---|
| **N5** | 475–500 | 500 | **Excelente:** Solución y evidencia completas, precisas, reproducibles y explicadas con profundidad analítica y técnica sobresaliente. |
| **N4** | 400–474 | 450 | **Bueno:** Desempeño técnico correcto con omisiones menores que no comprometen la operación, reproducibilidad ni diagnóstico. |
| **N3** | 300–399 | 350 | **Aceptable:** Demuestra el desempeño esencial con evidencia verificable. **Es el umbral individual de logro.** |
| **N2** | 150–299 | 250 | **Cumplimiento parcial:** Evidencia incompleta, métricas faltantes o errores conceptuales en XRCE-DDS o redes. |
| **N1** | 0–149 | 100 | **No cumple:** Evidencia mínima, fragmentaria o no funcional. Sin evidencia obligatoria se registra 0. |

---

## 4. Alineación de Criterios, RAE y Student Outcomes

| Criterio | Peso | Student Outcome Principal | Indicador de Desempeño Literal del Programa | Evidencia Directa Obligatoria |
|---|:---:|:---:|---|---|
| **C1. Arquitectura Micro XRCE-DDS y agentes micro-ROS** | 25% | **SO1** | **1.1.** Identifica y selecciona los requerimientos para la arquitectura de software distribuido del robot mediante nodos de ROS 2 y micro-ROS. | Ejecución del agente (`-v6`), creación de cliente/sesión, validación de transporte serial y WiFi UDP, Tabla 1 y Tabla 2. |
| **C2. Namespaces, interfaces y teleoperación en tiempo real** | 25% | **SO2** | **2.2.** Incorpora restricciones de red, latencia, ancho de banda y seguridad en la integración de hardware heterogéneo (micro-ROS en microcontroladores). | Nodo `/burger_car_NN/base_controller`, suscripción a `cmd_vel` (`Twist`), publicación de `battery_voltage`, teleoperación remapeada y LED físico. |
| **C3. Diagnóstico por tramos de red y métricas cuantitativas** | 25% | **SO6** | **6.1.** Diseña y ejecuta pruebas de conectividad, jitter, pérdida de paquetes y latencia en enlaces inalámbricos XRCE-DDS. | Script `diagnostico_microros.sh`, ping de 100 paquetes (Tabla 3: RTT, jitter `mdev`, pérdida) y estabilidad de telemetría a 1, 5 y 10 Hz (Tabla 4). |
| **C4. Resiliencia, máquina de estados y reconexión** | 15% | **SO6** | **6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos por capas para aislar errores en hardware y software. | Simetría `create_entities()` / `destroy_entities()`, cronometraje de reconexión autónoma tras parada de agente (Tabla 5) y manejo seguro de fallos. |
| **C5. Trazabilidad técnica y trabajo en equipo** | 10% | **SO3 / SO5** | **3.1 - 3.3.** Documentación técnica reproducible.<br>**5.1.** Define roles técnicos y coordina la ejecución en equipo. | Grafo `rqt_graph` comentado, informe técnico estructurado con comandos reproducibles y Anexo A individual diligenciado. |
| **TOTAL** | **100%** | | | |

---

## 5. Registro de Evidencias de la Práctica (E1–E8)

| Código | Evidencia Requerida | Localizador en el Documento / Repositorio / Archivo |
|---|---|---|
| **E1** | Captura de ejecución del agente micro-ROS con opción `-v6` mostrando creación de cliente, sesión XRCE-DDS y participantes DDS. | |
| **E2** | Salida de terminal del Mini-Reto 1 (serial): medición con `ros2 topic hz` del heartbeat a 2 Hz y 10 Hz. | |
| **E3** | Captura de `ros2 node list` (`/burger_car_NN/base_controller`) y `ros2 topic info /burger_car_NN/cmd_vel` (`Subscription count: 1`). | |
| **E4** | Captura de teleoperación con `ros2 topic pub --once` y `teleop_twist_keyboard` remapeado, mostrando activación física del LED. | |
| **E5** | Salida de la ejecución de `diagnostico_microros.sh` mostrando agente, puertos, ping al router y al ESP32. | |
| **E6** | Resultados cuantitativos del ping de 100 paquetes (`-i 0.2`) al ESP32: cerca y lejos del router (RTT min/avg/max/mdev y pérdida). | |
| **E7** | Registro de estabilidad de telemetría con `ros2 topic hz /burger_car_NN/battery_voltage -w 50` para periodos de 1000, 200 y 100 ms. | |
| **E8** | Cronometraje y captura de reconexión autónoma del ESP32 tras detener y relanzar el agente UDP (sin reset físico del microcontrolador). | |

---

## 6. Tablas de Registro Experimental (Diligenciadas por el Equipo)

### Tabla 1: Configuración de Dispositivos, Interfaces y Parámetros de Red

| Parámetro | Valor Configurado / Asignado | Método de Verificación |
|---|---|---|
| **Modelo de placa ESP32 utilizada** | | Inspección física del chip |
| **Puerto serial USB de programación** | `/dev/ttyUSB` o `/dev/ttyACM` | `ls /dev/tty*` |
| **Red Wi-Fi (SSID)** | `ros2` (o la del laboratorio) | Configuración de firmware |
| **Dirección IP de la PC (Host Agente)** | | `hostname -I` |
| **Puerto UDP del Agente micro-ROS** | `8888` | Parámetro `--port 8888` |
| **Dirección IP asignada al ESP32** | | Monitor Serie (`IP del ESP32: ...`) |
| **Dirección MAC del ESP32** | | Monitor Serie / router |
| **Namespace oficial del robot móvil** | `/burger_car_` | Macro `ROBOT_NAMESPACE` |

---

### Tabla 2: Métricas del Heartbeat Serial (Mini-Reto 1, Fase 1)

| Configuración de Frecuencia | Periodo Configurado (`HEARTBEAT_PERIOD_MS`) | Frecuencia Media Medida (`ros2 topic hz`) | Mínimo (s) | Máximo (s) | Desviación Estándar (`std dev`) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **Nominal 2 Hz** | 500 ms | Hz | | | |
| **Nominal 10 Hz** | 100 ms | Hz | | | |

---

### Tabla 3: Calidad de Enlace Inalámbrico hacia el ESP32 (`ping -c 100 -i 0.2 <IP_ESP32>`)

| Ubicación del Robot / ESP32 | Distancia Aprox. al Router | RTT Mínimo (ms) | RTT Promedio (ms) | RTT Máximo (ms) | Jitter (`mdev`) (ms) | Paquetes Perdidos (%) |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Cerca del Router (< 2 m, línea de vista)** | $< 2\text{ m}$ | | | | | |
| **Lejos del Router (> 8 m o con obstáculos)** | $> 8\text{ m}$ | | | | | |

---

### Tabla 4: Estabilidad de la Telemetría según Periodo de Publicación (`ros2 topic hz -w 50`)

| Periodo (`TELEMETRY_PERIOD_MS`) | Tasa Teórica | Tasa Media Medida (Hz) | Mínimo (s) | Máximo (s) | Desviación Estándar (`std dev`) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **1000 ms** | 1.0 Hz | Hz | | | |
| **200 ms** | 5.0 Hz | Hz | | | |
| **100 ms** | 10.0 Hz | Hz | | | |

---

### Tabla 5: Registro de Resiliencia y Máquina de Estados de Reconexión

| Evento Experimental | Acción Ejecutada | Respuesta del Firmware en ESP32 | Estado de `ros2 node list` | Tiempo Cronometrado de Recuperación |
|---|---|---|---|---|
| **Pérdida del Agente** | `Ctrl+C` al agente en PC | Falla ping, entra a `destroy_entities()` | Nodo desaparece del grafo | — |
| **Relanzamiento del Agente** | Relanzar agente UDP :8888 | Detecta agente, ejecuta `create_entities()` | Nodo reaparece en el grafo | s (sin reset físico) |
| **Pérdida de Señal Wi-Fi** | Apagar Wi-Fi / alejar | Entra a reconexión Wi-Fi | Nodo desaparece del grafo | s tras reconectar |

---

## 7. Cuestionario de Análisis Técnico (Respondido por el Equipo)

### Pregunta 1: Ausencia del Agente en ros2 node list y Rol de XRCE-DDS
*¿Por qué al ejecutar `ros2 node list` aparece directamente `/burger_car_NN/base_controller` pero NO aparece ningún nodo denominado `micro_ros_agent`? Explique técnicamente cómo opera el Agente micro-ROS como puente (*Bridge*) entre el protocolo ligero Micro XRCE-DDS del microcontrolador y el estándar DDS del resto del sistema.*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 2: Distinción Crítica entre Métricas del Monitor Web y Ping al ESP32
*Compare lo que mide el Dashboard del Monitor de Red web (`http://localhost:8080`) frente a lo que mide el comando `ping <IP_ESP32>`. ¿Por qué una medición de latencia baja hacia el router no garantiza que la comunicación con el microcontrolador esté libre de jitter o pérdidas? Explique los tramos de red involucrados.*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 3: Necesidad del Remapeo en teleop_twist_keyboard
*¿Por qué el nodo estándar `teleop_twist_keyboard` requiere obligatoriamente el remapeo `cmd_vel:=/burger_car_NN/cmd_vel` para operar el carrito? ¿Qué ocurriría en un laboratorio con múltiples carritos si todos los equipos operaran publicando directamente en el tópico raíz `/cmd_vel`?*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 4: Simetría de Entidades y Gestión de Memoria en C / C++ Embebido
*En el firmware, analice por qué es fundamental que `create_entities()` y `destroy_entities()` sean exactamente simétricas. Dado que el ESP32 cuenta con memoria SRAM restringida (520 KB) y no dispone de recolección de basura (*Garbage Collector*), ¿qué problema fatal ocurriría si la función de destrucción no liberara el suscriptor o el executor antes de reintentar la conexión?*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 5: Impacto de la Tasa de Telemetría sobre Jitter y Uso de CPU
*A partir de los datos registrados en la Tabla 4, analice el comportamiento de la desviación estándar (`std dev`) y el jitter al pasar de 1 Hz (1000 ms) a 10 Hz (100 ms). ¿Por qué en un sistema embebido con FreeRTOS y Wi-Fi no es conveniente elevar la frecuencia de telemetría más allá de lo necesario para el lazo de control?*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

### Pregunta 6: Seguridad Funcional ante Pérdida de Comunicación
*En el Mini-Reto 3 se propone conectar un puente H para accionar motores reales. Si el robot móvil se encuentra avanzando a $0.5\text{ m/s}$ y repentinamente se cae la red Wi-Fi o se apaga la PC del agente, ¿qué le ocurriría a los motores si el firmware no implementa una parada en `destroy_entities()`? Formule una política de parada segura (*Fail-Safe*).*

> **Respuesta Técnica del Equipo:**  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  
> ________________________________________________________________________________________________________________________  

---

## 8. Anexo A — Comprobación Individual de Desempeño

> **Propósito ABET:** Este anexo verifica la autoría, comprensión técnica y contribución directa individual de cada estudiante, sustentando la asignación del logro individual en los Student Outcomes evaluados.

### Estudiante 1: __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas en la práctica (ej. compilación/ejecución del agente, conexión y pruebas seriales, configuración de Wi-Fi, pruebas de teleoperación o script de diagnóstico). | |
| **2. Arquitectura micro-ROS:** Explique con sus propias palabras qué hace la capa `rclc` y el executor en el firmware del ESP32 frente al cliente XRCE-DDS. | |
| **3. Máquina de estados de reconexión:** Explique cómo el firmware detecta que se perdió el agente y qué pasos sigue para reconectarse sin necesidad de pulsar el botón reset. | |
| **4. Análisis de una falla resuelta:** Describa un problema técnico enfrentado (ej. puerto ocupado en Windows/WSL, error en `AGENT_IP`, firewall, error de compilación de bibliotecas) y cómo lo solucionó. | |
| **5. Autoría y reproducibilidad:** Indique qué comandos, mediciones de tablas, capturas o secciones del firmware fueron elaborados directamente por usted. | |

**Estado de comprobación individual Estudiante 1:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐  
**Firma del Estudiante 1:** ________________________________________

---

### Estudiante 2: __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas en la práctica (ej. compilación/ejecución del agente, conexión y pruebas seriales, configuración de Wi-Fi, pruebas de teleoperación o script de diagnóstico). | |
| **2. Arquitectura micro-ROS:** Explique con sus propias palabras qué hace la capa `rclc` y el executor en el firmware del ESP32 frente al cliente XRCE-DDS. | |
| **3. Máquina de estados de reconexión:** Explique cómo el firmware detecta que se perdió el agente y qué pasos sigue para reconectarse sin necesidad de pulsar el botón reset. | |
| **4. Análisis de una falla resuelta:** Describa un problema técnico enfrentado (ej. puerto ocupado en Windows/WSL, error en `AGENT_IP`, firewall, error de compilación de bibliotecas) y cómo lo solucionó. | |
| **5. Autoría y reproducibilidad:** Indique qué comandos, mediciones de tablas, capturas o secciones del firmware fueron elaborados directamente por usted. | |

**Estado de comprobación individual Estudiante 2:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐  
**Firma del Estudiante 2:** ________________________________________

---

## 9. Selección del Nivel Alcanzado por Criterio (Rúbricas Docentes)

Marque con una **X** una sola casilla por criterio y registre el valor entero (0 a 500) dentro de la banda correspondiente.

### C1. Arquitectura Micro XRCE-DDS y agentes micro-ROS — Peso 25% — Student Outcome SO1

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, analiza la serialización CDR en tramas XRCE-DDS, optimiza el tamaño de los buffers del middleware y compara el overhead de transporte serial vs UDP demostrando uso eficiente de la memoria estática. |
| ☐ | **N4 — 400–474** | Configura y ejecuta exitosamente el agente micro-ROS en PC (Docker o nativo), verifica la creación de clientes y sesiones con `-v6`, y valida la comunicación tanto por puerto Serial como por Wi-Fi UDP. |
| ☐ | **N3 — 300–399** | Pone en marcha el agente micro-ROS, establece comunicación con el ESP32 y documenta la salida en las Tablas 1 y 2. |
| ☐ | **N2 — 150–299** | Presenta fallos de conexión por configuración de puertos, discrepancia de bibliotecas o no logra verificar el flujo serial. |
| ☐ | **N1 — 0–149** | No logra ejecutar el agente micro-ROS o carece de evidencias funcionales obligatorias. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C2. Namespaces, interfaces y teleoperación en tiempo real — Peso 25% — Student Outcome SO2

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, implementa el Mini-Reto 3 accionando motores reales mediante PWM con saturación segura y parada automática de hardware programada en `destroy_entities()`. |
| ☐ | **N4 — 400–474** | Configura el namespace `/burger_car_NN`, valida que los tópicos sean relativos, verifica `Subscription count: 1` en `cmd_vel`, y demuestra teleoperación interactiva con `teleop_twist_keyboard` y respuesta en LED. |
| ☐ | **N3 — 300–399** | Asigna namespace al nodo, recibe comandos de velocidad encendiendo el LED y visualiza la telemetría sintética de batería. |
| ☐ | **N2 — 150–299** | Opera con tópicos en el espacio global sin namespace causando colisiones o requiere modificar código para cambiar metas. |
| ☐ | **N1 — 0–149** | No logra recibir comandos de velocidad en el microcontrolador o carece de justificación de interfaces. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C3. Diagnóstico por tramos de red y métricas cuantitativas — Peso 25% — Student Outcome SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, analiza la distribución estadística del jitter (`mdev`), correlaciona las fluctuaciones de latencia con la saturación del espectro Wi-Fi y propone umbrales de QoS para navegación autónoma en interiores. |
| ☐ | **N4 — 400–474** | Ejecuta el script `diagnostico_microros.sh`, mide el ping de 100 paquetes cerca y lejos del router (Tabla 3), analiza la estabilidad de telemetría a 1, 5 y 10 Hz (Tabla 4) y diferencia claramente los tramos de red. |
| ☐ | **N3 — 300–399** | Realiza mediciones de ping hacia el ESP32, reporta latencias y registra la estabilidad de telemetría a dos frecuencias distintas. |
| ☐ | **N2 — 150–299** | Mediciones incompletas en las Tablas 3 y 4 o confusión entre las mediciones del monitor web (router) y las del ESP32. |
| ☐ | **N1 — 0–149** | No realiza mediciones cuantitativas de red o reporta valores ficticios sin captura de terminal verificable. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C4. Resiliencia, máquina de estados y reconexión — Peso 15% — Student Outcome SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, instrumenta el firmware con telemetría de fallos, cuantifica el consumo de heap en reconexiones sucesivas demostrando cero fugas de memoria (*zero memory leaks*) y formula un diagrama de estados formal. |
| ☐ | **N4 — 400–474** | Demuestra la simetría entre `create_entities()` y `destroy_entities()`, cronometra la reconexión autónoma tras detener y relanzar el agente (Tabla 5) y verifica que no requiere reset físico. |
| ☐ | **N3 — 300–399** | Realiza la prueba de apagado del agente, documenta la pérdida de sesión y logra la reconexión tras reiniciar el agente. |
| ☐ | **N2 — 150–299** | El ESP32 se bloquea al perder la conexión con el agente, requiere reset por pulsador físico o presenta fugas de memoria. |
| ☐ | **N1 — 0–149** | No implementa máquina de estados de reconexión o carece de evidencias de resiliencia. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C5. Trazabilidad técnica y trabajo en equipo — Peso 10% — Student Outcome SO3 / SO5

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, el informe técnico incluye diagramas de secuencia UML de la negociación XRCE-DDS y el Anexo A demuestra una división de roles técnicos y dominio individual sobresaliente de ambos integrantes. |
| ☐ | **N4 — 400–474** | Presenta el informe técnico completo con comandos reproducibles, capturas de `rqt_graph` comentadas, tablas llenas con rigor y el Anexo A evidencia autoría individual. |
| ☐ | **N3 — 300–399** | Entrega el informe con tablas diligenciadas, capturas de terminal legibles y Anexo A con respuestas verificables. |
| ☐ | **N2 — 150–299** | Informe incompleto, capturas sin contexto, tablas con datos faltantes o Anexo A con respuestas genéricas. |
| ☐ | **N1 — 0–149** | No entrega el informe técnico o el Anexo A demuestra ausencia de participación individual. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

## 10. Consolidado Final de Calificación Académica

| Criterio Evaluado | Peso Oficial (%) | Nivel Marcado | Valor Obtenido (0–500) | Aporte Ponderado |
|---|:---:|:---:|:---:|:---:|
| **C1. Arquitectura Micro XRCE-DDS y agentes micro-ROS** | 25% | | | |
| **C2. Namespaces, interfaces y teleoperación en tiempo real** | 25% | | | |
| **C3. Diagnóstico por tramos de red y métricas cuantitativas** | 25% | | | |
| **C4. Resiliencia, máquina de estados y reconexión** | 15% | | | |
| **C5. Trazabilidad técnica y trabajo en equipo** | 10% | | | |
| **TOTAL CONSOLIDADO** | **100%** | | | **________ / 500** |

```text
Nota Académica sobre 5,0 = Nota Consolidada sobre 500 ÷ 100
Aporte a Talleres del Primer Corte: T₁ = Nota Académica sobre 5,0
```

| Resultado Oficial de la Actividad | Registro |
|---|---|
| **Nota de Taller micro-ROS ESP32 sobre 500 puntos** | __________ / 500 |
| **Nota Académica Oficial sobre 5,0** | __________ / 5,0 |
| **Número de Criterios en N3 o superior (Umbral Individual)** | _____ / 5 |
| **¿Cumple el Umbral Individual de Logro ABET (todos en N3+)?** | SÍ ☐ &nbsp;&nbsp;&nbsp;&nbsp; NO ☐ |

---

## 11. Consolidación ABET y Cierre de Mejora Continua

| Student Outcome / Indicador Evaluado | N Estudiantes Evaluables | N en Nivel N3 o Superior | Porcentaje de Logro (%) | Meta Institucional | Hallazgo Docente |
|---|:---:|:---:|:---:|:---:|---|
| **SO1 / Indicador 1.1** (Arquitectura Distribuida y Agente) | | | | 70% | |
| **SO2 / Indicador 2.2** (Diseño con Hardware Heterogéneo) | | | | 70% | |
| **SO6 / Indicador 6.1 y 6.4** (Experimentación y Diagnóstico) | | | | 70% | |
| **SO3 / Indicador 3.1 y 3.3** (Documentación y Trazabilidad) | | | | 70% | |
| **SO5 / Indicador 5.1** (Trabajo en Equipo y Roles) | | | | 70% | |

| Campo de Cierre de Ciclo de Mejora Continua | Registro de Gestión Pedagógica |
|---|---|
| **Decisión derivada del hallazgo** | |
| **Acción de mejora continua pedagógica y técnica** | |
| **Responsable de la acción** | Ing. Henry Roncancio |
| **Fecha prevista de seguimiento** | |
| **Evidencia de seguimiento** | |
| **Resultado observado en el segundo ciclo** | |

---

## 12. Cierre y Firmas de Conformidad

**Docente Evaluador:** Ing. Henry Roncancio &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; **Fecha de Evaluación:** ____________________

**Firma del Docente Evaluador:** __________________________________________________

**Observaciones Finales y Retroalimentación Pedagógica:**  
________________________________________________________________________________________________________________________  
________________________________________________________________________________________________________________________  
