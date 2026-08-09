# Instrumento de evidencia y calificación — Laboratorio 01: red ROS 2 distribuida

> Instrumento asociado a `education/guias_laboratorio/GUIA_LAB_01_RED_ROS2_TALKER_LISTENER.docx`. La práctica se realizó en parejas antes de publicar esta versión normalizada. Sólo se valoran actividades y evidencias exigidas por la guía aplicada; el anexo individual verifica autoría y comprensión de ese trabajo, sin introducir un experimento técnico nuevo.

## 1. Identificación

| Campo | Registro |
|---|---|
| Programa | Ingeniería Mecatrónica |
| Asignatura | ROBOT OPERATING SYSTEM - ROS |
| Periodo | 2026-2 |
| Corte / instrumento | Primer corte / Laboratorios y evidencias experimentales |
| Actividad | Laboratorio 01 — Red ROS 2 distribuida Talker–Listener |
| Estudiante evaluado / código |  |
| Pareja / grupo |  |
| Archivo de entrega |  |
| Fecha original de la práctica |  |
| Fecha de evaluación |  |
| Evaluador |  |
| Versión del instrumento | 1.1 — normalización posterior a la práctica |
| Unidad de análisis | Pareja con comprobación individual |

## 2. Parámetros de assessment

| Parámetro | Regla adoptada |
|---|---|
| Población | Censo de estudiantes matriculados que debían presentar el Laboratorio 01 en 2026-2. |
| Momento de medición | Primer corte, sobre la práctica ya realizada y su entrega documental. |
| Evaluador | Docente responsable de ROBOT OPERATING SYSTEM - ROS. |
| Umbral individual | N3 o superior en cada indicador evaluado. |
| Meta de cohorte | Al menos 70% de estudiantes evaluables alcanza N3 o superior en cada indicador. |
| Muestreo | No se usa muestra: se evalúa el censo de entregas exigibles. |
| Evidencia faltante | Entrega exigible sin evidencia: N1 con 0. Retiro oficial o exclusión autorizada: `NA / no evaluado`. |
| Regla para trabajo en pareja | El informe común se complementa con el Anexo A individual. Sin comprobación individual suficiente no se infiere logro ABET del estudiante. |

## 3. Niveles para Zubatronic/SGDE

| Nivel | Intervalo | Interpretación |
|---|---:|---|
| N5 | 475–500 | Excelente: evidencia completa, precisa, reproducible y explicada con profundidad. |
| N4 | 400–474 | Bueno: desempeño correcto con omisiones menores que no impiden verificar el resultado. |
| N3 | 300–399 | Aceptable: demuestra el desempeño esencial con evidencia verificable. Es el umbral de logro. |
| N2 | 150–299 | Cumplimiento parcial: evidencia incompleta o con errores importantes. |
| N1 | 0–149 | No cumple: evidencia mínima, fragmentaria o no funcional. Sin evidencia obligatoria se registra 0. |

## 4. Alineación y pesos

El indicador 6.2 no se usa en este laboratorio porque el syllabus lo ubica en el segundo corte. El diagnóstico experimental realizado se asocia al indicador transversal 6.4, cuyo texto coincide con el protocolo aplicado.

| Criterio | Peso | SO principal | Indicador de desempeño literal | Evidencia directa |
|---|---:|---|---|---|
| C1. Arquitectura de software y red DDS | 30% | SO2 | **2.1.** Diseña soluciones de software para control y monitoreo de robots, integrando contratos de comunicación (QoS, interfaces customizadas) y redes DDS robustas. | Configuración de ambos dispositivos, variables DDS, conectividad e introspección remota. |
| C2. Diagnóstico experimental de comunicación | 30% | SO6 | **6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos de diagnóstico por capas (Sintaxis -> TF -> Red -> Lógica) para aislar errores en hardware y software. | Frecuencia, latencia/pérdida disponible, prueba QoS, falla inducida, causa y recuperación. |
| C3. Documentación técnica reproducible | 20% | SO3 | **3.1.** Elabora documentación técnica reproducible del sistema ROS 2, incluyendo diagramas de nodos, tópicos, servicios, acciones, frames TF y contratos de comunicación DDS. | DOCX entregado, comandos, tablas, diagrama y conclusiones trazables. |
| C4. Seguridad y aislamiento de red | 10% | SO4 | **4.1.** Identifica riesgos de seguridad física, ciberseguridad y operación colaborativa en celdas robóticas, considerando límites, zonas de trabajo, paradas de emergencia y permisos de red. | Dominio asignado, reglas de red, restauración del firewall y análisis de riesgo DDS. |
| C5. Coordinación y responsabilidad individual | 10% | SO5 | **5.1.** Reconoce habilidades técnicas y define interfaces y roles dentro del equipo de trabajo para el desarrollo distribuido de los nodos de la celda de automatización. | Roles Talker/Listener, contribución verificable y Anexo A individual. |
| **Total** | **100%** |  |  |  |

## 5. Registro de la evidencia común

| Código | Evidencia ya exigida por la guía | Archivo, página o ubicación |
|---|---|---|
| E1 | IP y conectividad bidireccional de los dos dispositivos. |  |
| E2 | `ROS_DOMAIN_ID`, rango de descubrimiento y `RMW_IMPLEMENTATION`. |  |
| E3 | Talker y Listener distribuidos, `/talker`, `/listener` y `/chatter`. |  |
| E4 | Medición de frecuencia y datos de latencia o pérdida disponibles. |  |
| E5 | Comparación o prueba de compatibilidad QoS. |  |
| E6 | Falla inducida de dominio o firewall, diagnóstico por capas y recuperación. |  |
| E7 | Tabla de resultados, análisis, conclusiones y respuestas de discusión. |  |
| E8 | Diagrama o descripción reproducible de la arquitectura. |  |

## 6. Anexo A — comprobación individual posterior

Este anexo verifica la atribución del trabajo ya realizado. Puede diligenciarse dentro del DOCX, mediante un formulario institucional o en una sustentación breve registrada por el docente.

| Pregunta individual | Respuesta / evidencia |
|---|---|
| Rol desempeñado y tareas concretas realizadas |  |
| Explique cómo comprobó que ambos equipos pertenecían al mismo dominio DDS |  |
| Interprete una medición o un síntoma de falla registrado por la pareja |  |
| Indique una decisión de seguridad o aislamiento aplicada y su justificación |  |
| Identifique las páginas, comandos o artefactos de su autoría o verificación directa |  |

Estado de comprobación individual: **Verificada ☐ / Insuficiente ☐ / NA autorizado ☐**

Localizador del formulario, acta o registro: ________________________________________________

## 7. Selección del nivel alcanzado

Marque una sola banda y registre un valor entero dentro del intervalo demostrado. Los descriptores se aplican por estudiante utilizando la evidencia común y el Anexo A.

### C1. Arquitectura de software y red DDS — 30% — indicador 2.1 / SO2

| Marque | Nivel | Evidencia observable |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, automatiza o documenta de forma inequívoca la configuración en ambos dispositivos y justifica las decisiones de RMW, dominio y red. |
| ☐ | N4 — 400–474 | Configura correctamente subred, dominio, CycloneDDS e introspección remota; la comunicación es estable y reproducible. |
| ☐ | N3 — 300–399 | Establece la comunicación Talker–Listener entre dispositivos y verifica nodos, tópico y variables principales. |
| ☐ | N2 — 150–299 | Obtiene comunicación parcial o intermitente; faltan variables, verificaciones o pasos reproducibles. |
| ☐ | N1 — 0–149 | No demuestra comunicación distribuida funcional o no presenta evidencia obligatoria verificable. |

**Nivel C1:** ________ &nbsp;&nbsp; **Valor Zubatronic:** ________

### C2. Diagnóstico experimental de comunicación — 30% — indicador 6.4 / SO6

| Marque | Nivel | Evidencia observable |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, compara cuantitativamente condiciones, explica variaciones y delimita las conclusiones con base en los datos disponibles. |
| ☐ | N4 — 400–474 | Relaciona frecuencia, latencia/pérdida o QoS con síntomas; aísla la causa por capas y demuestra recuperación. |
| ☐ | N3 — 300–399 | Registra frecuencia, ejecuta una falla inducida y documenta síntoma, corrección y verificación posterior. |
| ☐ | N2 — 150–299 | Presenta mediciones o diagnóstico parciales, sin causa demostrada o sin comprobar la recuperación. |
| ☐ | N1 — 0–149 | Afirma que la red funciona o falla sin datos ni protocolo verificable, o no presenta evidencia obligatoria. |

**Nivel C2:** ________ &nbsp;&nbsp; **Valor Zubatronic:** ________

### C3. Documentación técnica reproducible — 20% — indicador 3.1 / SO3

| Marque | Nivel | Evidencia observable |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, integra diagramas claros, scripts o registros estructurados y permite repetir el procedimiento sin aclaraciones externas. |
| ☐ | N4 — 400–474 | Documento ordenado y atribuible con arquitectura, comandos, resultados, incidencias y conclusiones casi completamente reproducibles. |
| ☐ | N3 — 300–399 | Incluye comandos esenciales, tabla de resultados, análisis y evidencia suficiente para verificar la práctica. |
| ☐ | N2 — 150–299 | Contiene capturas o resultados aislados; faltan datos, secuencia, interpretación o trazabilidad. |
| ☐ | N1 — 0–149 | Documento fragmentario, no atribuible o sin evidencia funcional verificable. |

**Nivel C3:** ________ &nbsp;&nbsp; **Valor Zubatronic:** ________

### C4. Seguridad y aislamiento de red — 10% — indicador 4.1 / SO4

| Marque | Nivel | Evidencia observable |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, analiza riesgos residuales de DDS sin cifrado y propone una mitigación viable para un entorno institucional. |
| ☐ | N4 — 400–474 | Justifica dominio, permisos de red y restauración segura del firewall; documenta consecuencias de una configuración incorrecta. |
| ☐ | N3 — 300–399 | Usa un dominio asignado, evita interferencias y verifica que cualquier regla temporal de firewall quede restaurada. |
| ☐ | N2 — 150–299 | Reconoce el riesgo, pero deja verificaciones incompletas o una restauración ambigua. |
| ☐ | N1 — 0–149 | Interfiere otros grupos, conserva una regla insegura o no presenta evidencia del control aplicado. |

**Nivel C4:** ________ &nbsp;&nbsp; **Valor Zubatronic:** ________

### C5. Coordinación y responsabilidad individual — 10% — indicador 5.1 / SO5

| Marque | Nivel | Evidencia observable |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, explica decisiones compartidas, resolución conjunta de una falla y contribuciones verificables de ambos roles. |
| ☐ | N4 — 400–474 | Define y alterna responsabilidades cuando es necesario; el Anexo A demuestra comprensión y contribución técnica individual. |
| ☐ | N3 — 300–399 | Identifica roles Talker/Listener, coordina la ejecución y demuestra una contribución individual verificable. |
| ☐ | N2 — 150–299 | La distribución de tareas existe, pero la coordinación o la atribución individual es incompleta. |
| ☐ | N1 — 0–149 | El trabajo se concentra en una persona o no existe evidencia verificable de participación. |

**Nivel C5:** ________ &nbsp;&nbsp; **Valor Zubatronic:** ________

## 8. Consolidado académico

| Criterio | Peso | Nivel | Valor 0–500 | Aporte ponderado |
|---|---:|:---:|---:|---:|
| C1. Arquitectura DDS | 30% |  |  |  |
| C2. Diagnóstico experimental | 30% |  |  |  |
| C3. Documentación reproducible | 20% |  |  |  |
| C4. Seguridad y aislamiento | 10% |  |  |  |
| C5. Coordinación individual | 10% |  |  |  |
| **Total** | **100%** |  |  | **________ / 500** |

```text
L₁ = Nota Laboratorio 01
Nota Corte 1 = 0,28 T₁ + 0,42 L₁ + 0,30 E₁
```

Para 2026-2, Laboratorio 01 constituye el 100% de `L₁` y aporta 42% a la nota del primer corte.

## 9. Consolidación ABET y mejora continua

La nota académica no se interpreta como un único nivel ABET. Se consolida cada indicador por separado y únicamente con estudiantes evaluables y comprobación individual suficiente.

| Indicador | N evaluable | N en N3 o superior | Porcentaje de logro | Meta | Hallazgo |
|---|---:|---:|---:|---:|---|
| 2.1 / SO2 |  |  |  | 70% |  |
| 6.4 / SO6 |  |  |  | 70% |  |
| 3.1 / SO3 |  |  |  | 70% |  |
| 4.1 / SO4 |  |  |  | 70% |  |
| 5.1 / SO5 |  |  |  | 70% |  |

| Campo de cierre de ciclo | Registro |
|---|---|
| Decisión derivada del hallazgo |  |
| Acción de mejora |  |
| Responsable |  |
| Fecha prevista de seguimiento |  |
| Evidencia de seguimiento |  |
| Resultado observado en el siguiente ciclo |  |

## 10. Cierre

Nombre de quien evalúa: ____________________________________  Fecha: __________________

Observaciones: __________________________________________________________________________
