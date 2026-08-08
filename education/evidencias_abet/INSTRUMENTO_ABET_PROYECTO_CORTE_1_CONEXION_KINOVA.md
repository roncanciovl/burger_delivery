# Instrumento ABET — Proyecto del primer corte: conexión con Kinova Gen3

> Instrumento de evidencia y calificación asociado a `education/proyectos_evaluables/PROYECTO_CORTE_1_CONEXION_KINOVA.md`. La especificación técnica define el proyecto; este documento registra exclusivamente las evidencias, el nivel alcanzado y la nota de la entrega final del primer corte.

## 1. Identificación

| Campo | Registro |
|---|---|
| Asignatura | ROBOT OPERATING SYSTEM - ROS |
| Periodo | 2026-2 |
| Corte / instrumento | Primer corte / Entrega final y sustentación técnica |
| Proyecto | Package `burger_kinova_connection` |
| Estudiante evaluado / código |  |
| Equipo / integrantes |  |
| Grupo |  |
| Repositorio |  |
| Rama y commit evaluados |  |
| Fecha de demostración |  |
| Evaluador |  |
| Versión del instrumento | 1.0 |
| Estación conectada al Kinova |  |
| Estación cliente DDS |  |
| ROS_DOMAIN_ID |  |

## 2. Resultado de este instrumento

Este instrumento produce la **Nota Proyecto Corte 1**, denominada `E₁` en el syllabus, y el resultado desagregado de cada criterio para su captura en Zubatronic/SGDE. `E₁` representa el 30% de la nota del primer corte; los niveles por criterio alimentan por separado los reportes RAE/SO.

| Criterio | RAE / indicador | Student Outcome | Peso | Máximo sobre 5,0 |
|---|---|---|---:|---:|
| C1. Arquitectura e integración funcional con Kortex | 2.1 | SO2 — Diseño de soluciones | 35% | 1,75 |
| C2. Validación experimental y diagnóstico de comunicación | 6.4 transversal | SO6 — Experimentación y análisis | 25% | 1,25 |
| C3. Seguridad física, de software y de red | 4.1 | SO4 — Responsabilidad profesional | 15% | 0,75 |
| C4. Documentación reproducible y sustentación técnica | 3.1 | SO3 — Comunicación efectiva | 15% | 0,75 |
| C5. Trabajo en equipo y contrato de integración | 5.1 | SO5 — Trabajo en equipo | 10% | 0,50 |
| **Total** |  |  | **100%** | **5,00** |

## 3. Forma de uso

1. El equipo registra las evidencias E1–E10 antes de la sustentación y se diligencia un registro por estudiante.
2. Quien evalúa marca **una sola banda de desempeño por criterio** y registra un valor de 0 a 500 dentro del intervalo correspondiente.
3. Los descriptores muestran progresión de desempeño. N3, N4 y N5 pueden incorporar capacidades de niveles anteriores, mientras N1 y N2 describen bandas alternativas de desempeño insuficiente o parcial.
4. Si falta una evidencia obligatoria, se marca N1 y se registra 0. Una exclusión autorizada o retiro oficial no se convierte en nota: se gestiona según la regla institucional de población.
5. Zubatronic/SGDE calcula automáticamente el aporte ponderado. En una copia manual se usa `aporte = valor × peso / 100` y la nota sobre 5,0 es el total sobre 500 dividido entre 100.

| Nivel | Intervalo Zubatronic / valor guía | Interpretación |
|---|---:|---|
| N5 | 475–500 / guía 500 | Excelente: solución completa, robusta, reproducible y explicada con precisión. |
| N4 | 400–474 / guía 450 | Bueno: solución correcta con omisiones menores que no impiden la operación. |
| N3 | 300–399 / guía 350 | Aceptable: cumple el funcionamiento esencial y presenta evidencia verificable. Este es el umbral de logro. |
| N2 | 150–299 / guía 250 | Parcial: funcionamiento o evidencia incompletos, con errores importantes. |
| N1 | 0–149 / guía 100 | No cumple: evidencia mínima o fragmentaria. Sin evidencia obligatoria se registra 0. |

## 4. Registro de evidencias

| Código | Evidencia requerida | Archivo, enlace, comando o ubicación |
|---|---|---|
| E1 | Árbol del package, `package.xml`, configuración y resultado de `colcon build` y `colcon test`. |  |
| E2 | Launch en modo fake y launch sobre hardware real, con parámetros visibles y sin editar código. |  |
| E3 | `/joint_states` con `joint_1` a `joint_7`, frecuencia medida y edad del último mensaje. |  |
| E4 | Respuesta de `/controller_manager/list_controllers` y verificación de controladores activos. |  |
| E5 | `/burger/kinova/diagnostics` en estados saludable, degradado y perdido. |  |
| E6 | Prueba de pérdida y recuperación del driver o de la conexión DDS, con tiempos y diagnóstico. |  |
| E7 | Validación de tres metas bloqueadas: movimiento deshabilitado, articulación faltante y límite excedido. |  |
| E8 | Trayectoria segura autorizada: meta, feedback, resultado y verificación del estado final. |  |
| E9 | Despliegue distribuido: estación A con Kortex, estación B con el cliente y contrato de red/DDS. |  |
| E10 | README, diagrama del grafo, protocolo de seguridad, roles, commits y sustentación reproducible. |  |

### Registro mínimo de la sesión evaluada

| Variable | Registro |
|---|---|
| Fecha y hora |  |
| Commit |  |
| Distribución ROS 2 |  |
| IP del robot |  |
| ROS_DOMAIN_ID |  |
| Frecuencia `/joint_states` |  |
| Timeout observado |  |
| Controlador de trayectoria |  |
| Resultado de la acción |  |
| Incidente o limitación encontrada |  |

## 5. Selección del nivel alcanzado

Marque con una **X** una sola banda en cada criterio y escriba el valor exacto que se capturará en Zubatronic. Si no existe evidencia obligatoria verificable, marque N1 y registre 0.

### C1. Arquitectura e integración funcional con Kortex — 35% — RAE 2.1 / SO2

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, la arquitectura separa correctamente driver y aplicación, admite operación local y distribuida, maneja todos los estados de error previstos y no requiere cambios en `ros2_kortex`. |
| ☐ | N4 — 400–474 | Además de N3, el launch parametriza los modos fake, real y cliente DDS; el monitor y el action client tienen contratos claros y manejo consistente de errores. |
| ☐ | N3 — 300–399 | El package compila, se instala y demuestra conexión real: recibe siete articulaciones, consulta controladores y envía una trayectoria autorizada mediante la acción estándar. |
| ☐ | N2 — 150–299 | Solo funciona en modo fake o requiere pasos manuales no documentados; falta alguna interfaz esencial o la ejecución real es intermitente. |
| ☐ | N1 — 0–149 | Existen scripts o fragmentos, pero no un package instalable ni una comunicación funcional verificable con el Kinova; incluye ausencia de evidencia obligatoria. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

### C2. Validación experimental y diagnóstico de comunicación — 25% — RAE 6.4 / SO6

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, compara cuantitativamente estados nominales y degradados, explica las variaciones y demuestra que otra persona puede repetir el protocolo y obtener conclusiones equivalentes. |
| ☐ | N4 — 400–474 | Además de N3, aísla correctamente fallas de driver, controlador, red DDS y lógica; relaciona síntomas, mediciones, causa y recuperación. |
| ☐ | N3 — 300–399 | Mide frecuencia y timeout, demuestra pérdida y recuperación, y registra diagnósticos saludables, degradados y de error. |
| ☐ | N2 — 150–299 | Presenta mediciones parciales o capturas aisladas; identifica el síntoma pero no demuestra causa, recuperación o repetibilidad. |
| ☐ | N1 — 0–149 | Reporta que la comunicación funciona o falla sin datos, protocolo ni diagnóstico verificable; incluye ausencia de evidencia obligatoria. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

### C3. Seguridad física, de software y de red — 15% — RAE 4.1 / SO4

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, analiza riesgos residuales, justifica límites y permisos de red, y demuestra mediante pruebas que una reconexión no rehabilita movimiento automáticamente. |
| ☐ | N4 — 400–474 | Además de N3, bloquea metas inválidas o vencidas, protege configuración sensible y documenta respuesta ante fallas, zonas de trabajo y parada de emergencia. |
| ☐ | N3 — 300–399 | Inicia con movimiento deshabilitado, valida primero en fake, exige habilitación explícita y cumple el protocolo físico de operación supervisada. |
| ☐ | N2 — 150–299 | Aplica algunas medidas, pero conserva valores inseguros por defecto, validaciones incompletas o un protocolo ambiguo. |
| ☐ | N1 — 0–149 | La solución permite movimiento sin controles suficientes, expone configuración sensible o no presenta evidencia obligatoria verificable. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

### C4. Documentación reproducible y sustentación técnica — 15% — RAE 3.1 / SO3

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, la sustentación explica decisiones, límites y resultados sin depender de lectura literal; responde preguntas reproduciendo evidencia del sistema. |
| ☐ | N4 — 400–474 | Además de N3, documenta modos local y distribuido, QoS, fallas comunes, seguridad y resultados cuantitativos con trazabilidad al commit. |
| ☐ | N3 — 300–399 | El README permite compilar, lanzar, interpretar diagnósticos y repetir las pruebas; incluye grafo, interfaces y comandos. |
| ☐ | N2 — 150–299 | La documentación cubre la ejecución básica, pero omite dependencias, resultados esperados, diagnóstico o decisiones de arquitectura. |
| ☐ | N1 — 0–149 | Presenta capturas o instrucciones fragmentarias que no permiten reproducir el proyecto, o no presenta evidencia obligatoria. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

### C5. Trabajo en equipo y contrato de integración — 10% — RAE 5.1 / SO5

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, el equipo demuestra revisión cruzada, decisiones técnicas compartidas y resolución documentada de conflictos o fallas de integración. |
| ☐ | N4 — 400–474 | Además de N3, los commits y revisiones muestran contribuciones equilibradas; las interfaces se acordaron antes de integrar y se respetaron durante las pruebas. |
| ☐ | N3 — 300–399 | Define roles, contrato de interfaces y responsabilidades; integra monitor, launch, action client y documentación en una demostración común. |
| ☐ | N2 — 150–299 | La distribución de tareas existe, pero la integración es tardía, las interfaces cambian sin control o las contribuciones son difíciles de verificar. |
| ☐ | N1 — 0–149 | El trabajo se concentra en una persona, no existe evidencia de coordinación técnica o falta la comprobación individual obligatoria. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

## 6. Captura y cálculo

Capture en Zubatronic el valor exacto de cada criterio. La plataforma calcula el aporte ponderado con la fórmula:

```text
Aporte del criterio = valor del criterio × peso / 100
Nota del instrumento sobre 500 = suma de los cinco aportes
Nota del instrumento sobre 5,0 = nota sobre 500 / 100
```

Los valores guía 500, 450, 350, 250 y 100 agilizan una valoración discreta, pero puede usarse cualquier entero dentro del intervalo demostrado. La ausencia de evidencia obligatoria se registra con 0 dentro de N1.

## 7. Consolidado final

Zubatronic calcula este consolidado automáticamente. La tabla permite verificar la captura o registrar el resultado en una copia de respaldo.

| Criterio | Peso | Nivel marcado | Valor del nivel | Aporte ponderado |
|---|---:|:---:|---:|---:|
| C1. Arquitectura e integración | 35% |  |  |  |
| C2. Validación y diagnóstico | 25% |  |  |  |
| C3. Seguridad | 15% |  |  |  |
| C4. Documentación y sustentación | 15% |  |  |  |
| C5. Trabajo en equipo | 10% |  |  |  |
| **Nota Proyecto Corte 1 — escala 0–500** | **100%** |  |  | **________ / 500** |
| **Nota Proyecto Corte 1 — escala 0,0–5,0** |  |  |  | **________ / 5,0** |

| Resultado complementario | Registro |
|---|---|
| Criterios en N3 o superior |  |
| Criterios por debajo de N3 |  |
| Fortalezas observadas |  |
| Mejora prioritaria |  |
| Evidencia individual verificada | Sí ☐ / No ☐ |

## 8. Integración con la nota del primer corte

```text
E₁ = Nota Proyecto Corte 1
Nota Corte 1 = 0,28 T₁ + 0,42 L₁ + 0,30 E₁
```

Este instrumento genera `E₁` y los resultados desagregados por criterio. Los promedios `T₁` y `L₁` provienen de sus instrumentos respectivos; Zubatronic consolida los reportes RAE/SO a partir de la captura por estudiante.

## 9. Cierre

Nombre de quien evalúa: ____________________________________  Fecha: __________________

Firma o visto bueno: __________________________________________________________________

Observaciones finales:

________________________________________________________________________________

________________________________________________________________________________
