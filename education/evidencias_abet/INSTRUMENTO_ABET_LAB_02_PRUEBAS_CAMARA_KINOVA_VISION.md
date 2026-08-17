# Instrumento de evidencia y calificación — Laboratorio 02: pruebas de cámara, compresión de video, CycloneDDS y Monitor de Red

> Instrumento asociado a `education/guias_laboratorio/GUIA_LAB_02_PRUEBAS_CAMARA_KINOVA_VISION.docx`. La práctica se realiza en parejas de trabajo en arquitectura distribuida (Dispositivo A: Gateway Kinova con Monitor de Red y Dispositivo B: Estación de Procesamiento Wi-Fi). El informe común reúne las evidencias técnicas del equipo, telemetría CSV y enlace de grabación de video; el Anexo A verifica la autoría, comprensión y contribución técnica individual de cada estudiante para sustentar el logro ABET.

---

## 1. Identificación

| Campo | Registro |
|---|---|
| **Programa** | Ingeniería Mecatrónica |
| **Asignatura** | ROBOT OPERATING SYSTEM - ROS |
| **Periodo** | 2026-2 |
| **Corte / instrumento** | Laboratorios y evidencias experimentales |
| **Actividad** | Laboratorio 02 — Pruebas de conectividad, compresión de video (`image_transport`), transmisión con CycloneDDS, telemetría con Monitor de Red y grabación audiovisual |
| **Estudiante evaluado / código** | |
| **Pareja / grupo** | |
| **Archivo de entrega** | `C1_L02_G<grupo>_<codigo1>_<codigo2>_v1.docx` |
| **Fecha de realización de la práctica** | |
| **Fecha de evaluación** | |
| **Evaluador** | Ing. Henry Roncancio |
| **Versión del instrumento** | Versión 1.2 |
| **Unidad de análisis / Unidad de captura** | Pareja con comprobación individual |

---

## 2. Parámetros de assessment

| Parámetro | Regla adoptada |
|---|---|
| **Población o cohorte** | Censo de estudiantes matriculados que deben presentar el Laboratorio 02 en 2026-2. |
| **Momento de medición** | Sesión experimental y entrega documental del laboratorio con telemetría CSV y grabación de video. |
| **Evaluador** | Docente responsable de la asignatura ROBOT OPERATING SYSTEM - ROS. |
| **Umbral individual** | Nivel **N3 o superior** (umbral individual de logro mínimo 300/500) en cada indicador evaluado. |
| **Meta de cohorte** | Al menos el **70% de los estudiantes evaluables** alcanza N3 o superior en cada indicador. |
| **Regla de muestreo** | No se usa muestra: se evalúa el censo completo de entregas exigibles. |
| **Evidencia faltante** | Entrega exigible sin evidencia obligatoria: N1 con valor 0. Retiro oficial o exclusión autorizada: `NA / no evaluado` (excluido del denominador). |
| **Regla para trabajo en pareja** | El informe común se complementa obligatoriamente con el Anexo A individual. Sin comprobación individual suficiente no se infiere logro ABET individual del estudiante. |

---

## 3. Niveles de Desempeño para Zubatronic/SGDE

| Nivel | Intervalo | Interpretación |
|---|---:|---|
| **N5** | 475–500 | **Excelente:** Solución y evidencia completas, precisas, reproducibles y explicadas con profundidad técnica y rigor analítico. |
| **N4** | 400–474 | **Bueno:** Desempeño correcto con omisiones menores que no impiden verificar la operación, reproducibilidad ni diagnóstico. |
| **N3** | 300–399 | **Aceptable:** Demuestra el desempeño esencial con evidencia verificable. **Es el umbral individual de logro.** |
| **N2** | 150–299 | **Cumplimiento parcial:** Evidencia incompleta, métricas faltantes o errores conceptuales en el diagnóstico. |
| **N1** | 0–149 | **No cumple:** Evidencia mínima, fragmentaria o no funcional. Sin evidencia obligatoria se registra 0. |

---

## 4. Alineación de Criterios, RAE y Student Outcomes

| Criterio | Peso | Student Outcome | Indicador de desempeño literal | Evidencia directa obligatoria |
|---|:---:|:---:|---|---|
| **C1. Conectividad, compresión y ancho de banda** | 25% | **SO2** | **Indicador de desempeño 2.2:** Incorpora restricciones de red, latencia, ancho de banda y seguridad en la integración de hardware heterogéneo (brazos robóticos, sensores, micro-ROS). | Pruebas de ping ICMP, RTT $< 5\text{ ms}$, compresión JPEG con `image_transport`, medición de `ros2 topic bw` crudo vs comprimido y cálculo de ahorro porcentual ($> 90\%$). |
| **C2. Diagnóstico experimental de visión y protocolos por capas** | 30% | **SO6** | **Indicador de desempeño 6.4:** Interpreta fallas y diagnósticos experimentales aplicando protocolos de diagnóstico por capas (Sintaxis -> TF -> Red -> Lógica) para aislar errores en hardware y software.<br>*(Apoyado en **6.2** variables de red y parámetros de cámara)* | Tabla de diagnóstico ante 5 fallas inducidas (Red, RTSP, CycloneDDS, Compresión, Sensor Web App), medición de FPS y verificación de recuperación. |
| **C3. Arquitectura distribuida con CycloneDDS y Monitor de Red** | 20% | **SO2** | **Indicador de desempeño 2.1:** Diseña soluciones de software para control y monitoreo de robots, integrando contratos de comunicación (QoS, interfaces customizadas) y redes DDS robustas. | Configuración de `rmw_cyclonedds_cpp` en Dispositivo A y B, archivo `cyclonedds.xml` (`wlan0`), dashboard web del Monitor de Red (`:8080`) y exportación del log CSV de telemetría. |
| **C4. Documentación técnica, telemetría y grabación de video** | 15% | **SO3** | **Indicador de desempeño 3.1 - 3.3:** Elabora documentación técnica reproducible del sistema ROS 2 y comunica resultados experimentales de percepción, calibración y planificación... | Informe estructurado con comandos exactos, capturas PNG calibradas (tecla 's'), archivo CSV del monitor y video continuo del experimento (2–4 min) con demostración y sustentación. |
| **C5. Seguridad, ética en captura visual y trabajo en equipo** | 10% | **SO4 / SO5** | **Indicador de desempeño 4.3 - 5.1:** Aplica buenas prácticas de responsabilidad profesional en el uso de datos de cámara y reconoce habilidades técnicas definiendo roles... | Protocolo de seguridad física del manipulador, respeto a privacidad en datos visuales, roles definidos (Gateway vs Procesamiento Wi-Fi) y Anexo A individual. |
| **Total** | **100%** | | | |

---

## 5. Registro de Evidencias de la Práctica

| Código | Evidencia Requerida | Archivo, página, comando o ubicación |
|---|---|---|
| **E1** | Registro cuantitativo de Ping ICMP, RTT promedio y jitter en enlace Ethernet y enlace Wi-Fi. | |
| **E2** | Captura y registro de FPS en tiempo real del visor directo RGB en color (`test_kinova_camera.py --stream color`). | |
| **E3** | Captura y registro de respuesta del visor directo de profundidad (`test_kinova_camera.py --stream depth`). | |
| **E4** | Archivos de imagen PNG capturados con tecla 's' (`kinova_capture_*.png`) con metadatos de resolución y tamaño. | |
| **E5** | Medición comparativa de ancho de banda (`ros2 topic bw`) entre `/camera/color/image_raw` y `/camera/color/image_raw/compressed`, con cálculo de ahorro porcentual. | |
| **E6** | Captura del Dashboard del Monitor de Red (`http://localhost:8080`) y archivo de telemetría exportado (`telemetria_red_lab02.csv`). | |
| **E7** | Archivo de configuración `cyclonedds.xml` y captura de verificación remota de recepción y descompresión en Dispositivo B sobre Wi-Fi. | |
| **E8** | Matriz diligenciada del diagnóstico por capas ante las cinco fallas inducidas y su procedimiento de recuperación. | |
| **E9** | Video continuo grabado del experimento (2–4 min) que muestra terminales en PC A, visor en PC B, monitor de red web y sustentación oral. | |
| **E10** | Informe técnico estructurado con comandos exactos, diagramas de arquitectura multi-PC, análisis de resultados y discusión técnica. | |

---

## 6. Anexo A — Comprobación Individual de Desempeño

> Este anexo permite atribuir el logro individual a cada integrante de la pareja evaluada.

| Pregunta Individual de Verificación | Respuesta / Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa las tareas específicas realizadas (ej. configuración del Gateway en Dispositivo A, compresión `image_transport`, configuración de CycloneDDS, ejecución del Monitor de Red en Dispositivo B o grabación de video). | |
| **2. Justificación de compresión de video en Wi-Fi:** Explique técnicamente por qué es inviable transmitir `/camera/color/image_raw` sin comprimir a través de una red Wi-Fi y cómo la compresión JPEG mitiga el retardo. | |
| **3. Configuración de CycloneDDS y Monitor de Red:** Justifique por qué se configuró `rmw_cyclonedds_cpp` y `NetworkInterfaceAddress` en `cyclonedds.xml` y qué anomalías de red permite detectar el monitor web. | |
| **4. Aislamiento metódico de fallas:** Describa cómo procedió para aislar y resolver una de las cinco fallas inducidas (Capa 1 Red, Capa 2 RTSP, Capa 3 CycloneDDS, Capa 4 Compresión o Capa 5 Sensor Web App). | |
| **5. Autoría y reproducibilidad:** Identifique los comandos, tablas, capturas, logs de telemetría o secciones del video de su autoría o sustentación directa. | |

**Estado de la comprobación individual:** Verificada ☐ &nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp; NA autorizado ☐  
**Localizador del acta, formulario o entrega:** __________________________________________________

---

## 7. Selección del Nivel Alcanzado por Criterio

Marque una sola casilla por criterio con una **X** y registre el valor entero (escala 0 a 500) dentro de la banda demostrada.

### C1. Conectividad, compresión y ancho de banda — Peso 25% — Student Outcome SO2

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, automatiza la configuración de red y compresión, calcula analíticamente la tasa de compresión y justifica los trade-offs de latencia vs calidad JPEG ($q=80$ vs $q=30$) demostrando un ahorro $> 90\%$ sin pérdida perceptible de precisión. |
| ☐ | **N4 — 400–474** | Configura `image_transport`, mide el ancho de banda con `ros2 topic bw` para video crudo vs comprimido, demuestra reducción a $< 2.5\text{ MB/s}$ y verifica fluidez $\ge 25\text{ FPS}$ en el stream RTSP. |
| ☐ | **N3 — 300–399** | Configura la interfaz de red, comprueba conectividad con ping, ejecuta el publicador de video comprimido `/camera/color/image_raw/compressed` y registra mediciones de ancho de banda. |
| ☐ | **N2 — 150–299** | Intenta la compresión pero transmite video crudo sobre Wi-Fi causando saturación, o las mediciones de ancho de banda son incompletas o no justificadas. |
| ☐ | **N1 — 0–149** | No logra comunicación de red con el robot, no implementa compresión de video o carece de evidencia obligatoria verificable. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C2. Diagnóstico experimental de visión y protocolos por capas — Peso 30% — Student Outcome SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, cuantifica y compara el impacto de cada falla inducida sobre la tasa de cuadros y el jitter, relaciona síntomas con logs detallados del Monitor de Red y FFMPEG, y formula un árbol de decisión para diagnóstico rápido transferible a producción. |
| ☐ | **N4 — 400–474** | Aísla con precisión las cinco fallas inducidas siguiendo estrictamente el protocolo por capas (Red -> RTSP -> CycloneDDS -> Compresión -> Web App), documentando el síntoma, método de detección y recuperación verificada. |
| ☐ | **N3 — 300–399** | Aplica el protocolo de diagnóstico por capas ante las fallas inducidas, documenta los síntomas observados y demuestra la recuperación del flujo de video. |
| ☐ | **N2 — 150–299** | Resuelve las fallas por método de ensayo y error sin seguir el protocolo por capas, o la documentación del aislamiento y recuperación es incompleta. |
| ☐ | **N1 — 0–149** | No logra diagnosticar ni recuperar el sistema ante fallas inducidas, o afirma operatividad sin registrar datos ni protocolo verificable. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C3. Arquitectura distribuida con CycloneDDS y Monitor de Red — Peso 20% — Student Outcome SO2

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, analiza la estructura de paquetes RTPS en Wi-Fi, optimiza parámetros avanzados en `cyclonedds.xml` (`MaxMessageSize`, fragmentación UDP), exporta telemetría CSV completa desde el Monitor de Red y justifica las políticas QoS `SensorData`. |
| ☐ | **N4 — 400–474** | Configura `rmw_cyclonedds_cpp` en ambos equipos, define `cyclonedds.xml` con interfaz `wlan0`, lanza el Monitor de Red (`:8080`) y verifica recepción remota fluida ($\ge 20\text{ Hz}$) en Dispositivo B. |
| ☐ | **N3 — 300–399** | Establece comunicación distribuida con CycloneDDS en el mismo `ROS_DOMAIN_ID`, lanza el monitor de red y recibe el tópico de video comprimido en el Dispositivo B. |
| ☐ | **N2 — 150–299** | La comunicación distribuida es intermitente debido a conflictos de RMW o no se registran métricas del monitor de red. |
| ☐ | **N1 — 0–149** | No logra comunicación distribuida entre Dispositivo A y B o carece de evidencia sobre el middleware y telemetría. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C4. Documentación técnica, telemetría y grabación de video — Peso 15% — Student Outcome SO3

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, el informe incluye diagramas de arquitectura impecables, telemetría CSV estructurada con análisis estadístico de jitter/latencia, y un video continuo de demostración (2–4 min) con excelente audio, edición clara y sustentación técnica fluida de ambos estudiantes. |
| ☐ | **N4 — 400–474** | Informe técnico ordenado y trazable con comandos reproducibles, tablas diligenciadas, capturas PNG, log CSV del monitor de red y video continuo demostrando la operación distribuida y recuperación ante fallas. |
| ☐ | **N3 — 300–399** | Entrega el informe con comandos esenciales, tablas de datos diligenciadas, capturas fotográficas, log CSV y enlace al video del experimento. |
| ☐ | **N2 — 150–299** | Documento incompleto sin telemetría CSV o video inaccesible/incompleto sin demostración de la operación distribuida. |
| ☐ | **N1 — 0–149** | Informe fragmentario, sin video del experimento o sin evidencia funcional verificable. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C5. Seguridad, ética en captura visual y trabajo en equipo — Peso 10% — Student Outcome SO4 / SO5

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, propone protocolos de seguridad física y ciberseguridad para cámaras en celdas industriales (VLANs, gestión de credenciales) y el Anexo A junto con el video demuestran una coordinación y dominio técnico individual sobresaliente entre Gateway y estación remota. |
| ☐ | **N4 — 400–474** | Cumple rigurosamente las normas de seguridad del robot, respeta las directrices éticas de captura óptica (sin datos personales) y el Anexo A evidencia tareas complementarias y dominio individual. |
| ☐ | **N3 — 300–399** | Aplica las normas básicas de seguridad en el laboratorio, toma capturas exclusivas de calibración y demuestra contribución individual mediante el Anexo A. |
| ☐ | **N2 — 150–299** | Omite precauciones de seguridad o el Anexo A muestra una distribución desequilibrada de tareas y comprensión parcial. |
| ☐ | **N1 — 0–149** | Incurre en actos inseguros en el laboratorio, vulnera pautas éticas de captura o el Anexo A no demuestra participación del estudiante. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

## 8. Consolidado de Calificación Académica

La calificación académica del laboratorio se consolida a partir de los aportes por criterio y su posterior ponderación dentro de la nota del corte:

| Criterio | Peso | Nivel Marcado | Valor Capturado (0–500) | Aporte Ponderado ($\text{Valor} \times \text{Peso} / 100$) |
|---|:---:|:---:|:---:|:---:|
| **C1. Conectividad y compresión de video** | 25% | | | |
| **C2. Diagnóstico por capas** | 30% | | | |
| **C3. Arquitectura CycloneDDS y Monitor de Red**| 20% | | | |
| **C4. Documentación, telemetría y video** | 15% | | | |
| **C5. Seguridad, ética y equipo** | 10% | | | |
| **Nota académica consolidada (Escala 0–500)** | **100%** | | | **________ / 500** |
| **Nota académica consolidada (Escala 0,0–5,0)** | | | | **________ / 5,0** |

```text
Cálculo: Nota académica sobre 5,0 = Nota sobre 500 / 100
Aporte a la nota del corte: L = Nota académica consolidada de laboratorios en el acumulado del corte.
```

---

## 9. Consolidación ABET y Cierre de Mejora Continua

> La nota académica no se interpreta como un nivel ABET global. Se consolida cada indicador por separado únicamente con estudiantes evaluables que cuenten con comprobación individual suficiente.

| Student Outcome / Indicador de desempeño | N Evaluable | N en N3 o superior | % de Logro | Meta de Cohorte | Hallazgo Docente |
|---|:---:|:---:|:---:|:---:|---|
| **SO2 / Indicador 2.2** (Redes, Hardware y Compresión) | | | | 70% | |
| **SO6 / Indicador 6.4** (Diagnóstico por Capas) | | | | 70% | |
| **SO2 / Indicador 2.1** (CycloneDDS y Telemetría QoS) | | | | 70% | |
| **SO3 / Indicador 3.1 - 3.3** (Comunicación y Video) | | | | 70% | |
| **SO4 - SO5 / Indicador 4.3 - 5.1** (Ética / Equipo)| | | | 70% | |

### Registro de Mejora Continua y Cierre de Ciclo

| Campo de Cierre de Ciclo | Registro |
|---|---|
| **Decisión derivada del hallazgo** | |
| **Acción de mejora pedagógica y técnica** | |
| **Responsable de la acción** | |
| **Fecha prevista de seguimiento** | |
| **Evidencia de seguimiento** | |
| **Resultado del segundo ciclo de evaluación** | |

---

## 10. Cierre y Firmas

**Evaluador:** ____________________________________ &nbsp;&nbsp;&nbsp;&nbsp; **Fecha:** __________________

**Observaciones finales:**  
___________________________________________________________________________________________________  
___________________________________________________________________________________________________
