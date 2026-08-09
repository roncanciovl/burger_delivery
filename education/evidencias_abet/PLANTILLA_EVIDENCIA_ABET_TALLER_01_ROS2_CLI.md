# Formato de evidencias y calificación — Taller 01: CLI de ROS 2

> Instrumento calificable asociado a `education/talleres/TALLER_ROS2_CLI.md`. El taller conserva las instrucciones de aprendizaje; este formato permite registrar la evidencia, marcar el nivel alcanzado en cada criterio y obtener una única nota consolidada.

## 1. Identificación

| Campo | Registro |
|---|---|
| Asignatura | ROBOT OPERATING SYSTEM - ROS |
| Periodo | 2026-2 |
| Corte / instrumento | Primer corte / Talleres y tareas |
| Actividad | Taller 01 — CLI de ROS 2 |
| Nombre completo |  |
| Código / grupo |  |
| Fecha de entrega |  |
| Evaluador |  |
| Versión del instrumento | 1.1 |
| Enlace o ubicación de la entrega |  |
| Commit evaluado, si aplica |  |
| Unidad de análisis | Individual |

## 2. Uso del formato

1. La persona participante registra las evidencias E1–E9.
2. Quien evalúa marca **una sola banda de desempeño por criterio** y registra un valor de 0 a 500 dentro del intervalo correspondiente.
3. Los niveles N3, N4 y N5 representan una progresión de dominio; N1 y N2 son bandas alternativas para evidencia insuficiente o parcial.
4. Si falta una evidencia obligatoria en una entrega exigible, se marca N1 y se registra 0 para la nota académica. Un retiro oficial o una exclusión autorizada se registra como `NA / no evaluado`, se excluye del denominador ABET y no se convierte en cero.
5. Zubatronic/SGDE calcula el aporte ponderado de cada criterio. En una copia manual se usa `aporte = valor × peso / 100`.
6. La entrega es individual y debe nombrarse `C1_T01_<codigo>_<apellido>_v1.docx`. El código del archivo, el código registrado en el documento y la identidad de la entrega deben coincidir.

| Nivel | Intervalo Zubatronic / valor guía | Interpretación |
|---|---:|---|
| N5 | 475–500 / guía 500 | Excelente: desempeño completo, preciso y reproducible. |
| N4 | 400–474 / guía 450 | Bueno: desempeño correcto con omisiones menores. |
| N3 | 300–399 / guía 350 | Aceptable: logra lo esencial con explicación parcial. Este es el umbral de logro. |
| N2 | 150–299 / guía 250 | Parcial: evidencia incompleta o errores importantes. |
| N1 | 0–149 / guía 100 | No cumple: evidencia mínima, fragmentaria o no funcional. Sin evidencia obligatoria se registra 0. |

### 2.1. Parámetros de assessment

| Parámetro | Regla adoptada |
|---|---|
| Población | Censo de estudiantes matriculados que deben presentar el Taller 01 en 2026-2. |
| Momento de medición | Primer corte, después de la práctica guiada de CLI de ROS 2. |
| Evaluador | Docente responsable de ROBOT OPERATING SYSTEM - ROS. |
| Umbral individual | N3 o superior en cada indicador evaluado. |
| Meta de cohorte | Al menos 70% de estudiantes evaluables alcanza N3 o superior en cada indicador. |
| Muestreo | No se usa muestra: se evalúa el censo de entregas exigibles. |
| Evidencia faltante | Entrega exigible sin evidencia: N1 con 0. Retiro oficial o exclusión autorizada: `NA / no evaluado`. |

### 2.2. Alineación de criterios para Zubatronic/SGDE

Cada criterio tiene un único indicador principal. Los demás aprendizajes que aparezcan en la evidencia son complementarios y no producen una segunda medición indistinguible.

| Criterio | Peso | SO principal | Indicador de desempeño literal | Evidencia directa |
|---|---:|---|---|---|
| C1. Ejecución e introspección del grafo ROS 2 | 25% | SO2 | **2.1.** Diseña soluciones de software para control y monitoreo de robots, integrando contratos de comunicación (QoS, interfaces customizadas) y redes DDS robustas. | E1 y explicación individual del grafo. |
| C2. Operación e interpretación de tópicos y mensajes | 20% | SO2 | **2.1.** Diseña soluciones de software para control y monitoreo de robots, integrando contratos de comunicación (QoS, interfaces customizadas) y redes DDS robustas. | E2 y explicación del flujo publicador/suscriptor. |
| C3. Servicios, parámetros y rqt | 20% | SO2 | **2.1.** Diseña soluciones de software para control y monitoreo de robots, integrando contratos de comunicación (QoS, interfaces customizadas) y redes DDS robustas. | E3–E7 y resultados observables. |
| C4. Decisión arquitectónica para `burger_delivery` | 20% | SO2 | **2.1.** Diseña soluciones de software para control y monitoreo de robots, integrando contratos de comunicación (QoS, interfaces customizadas) y redes DDS robustas. | E8: selección justificada de interfaces. |
| C5. Evidencia reproducible y diagnóstico | 15% | SO3 | **3.1.** Elabora documentación técnica reproducible del sistema ROS 2, incluyendo diagramas de nodos, tópicos, servicios, acciones, frames TF y contratos de comunicación DDS. | E9, registro reproducible y trazabilidad de la entrega. |

## 3. Registro mínimo de evidencias

Las capturas deben ser legibles y mostrar el resultado obtenido, no solamente el comando escrito.

| Código | Evidencia requerida | Archivo, enlace, página o ubicación |
|---|---|---|
| E1 | `ros2 node list` antes y después de detener `/teleop_turtle`, y `ros2 node info /turtlesim`. |  |
| E2 | Publicación de `geometry_msgs/msg/Twist`, `ros2 topic echo /turtle1/cmd_vel` y trazo de la inicial. |  |
| E3 | Invocación de `/clear` y efecto observable en `turtlesim`. |  |
| E4 | Cambio de `background_r`, con valor aplicado y efecto observable. |  |
| E5 | Invocación de `/turtle1/set_pen` desde rqt, con color y ancho 5. |  |
| E6 | Captura con `turtlesim`, rqt Node Graph y `topic echo` activos. |  |
| E7 | Diferencia entre tópico y servicio: flujo, asincronía y confirmación. |  |
| E8 | Dos tópicos y dos servicios viables para `burger_delivery`, con productor/consumidor o servidor/cliente. |  |
| E9 | Diagnóstico real: síntoma, causa, corrección y verificación. |  |

Ubicación del registro reproducible de comandos: ________________________________________________

## 4. Respuestas breves de la persona participante

### E7. Tópico frente a servicio

Explique la diferencia en máximo cinco líneas:

________________________________________________________________________________

________________________________________________________________________________

### E8. Aplicación a `burger_delivery`

| Interfaz | Nombre propuesto | Productor/servidor | Consumidor/cliente | Justificación breve |
|---|---|---|---|---|
| Tópico 1 |  |  |  |  |
| Tópico 2 |  |  |  |  |
| Servicio 1 |  |  |  |  |
| Servicio 2 |  |  |  |  |

### E9. Diagnóstico

| Síntoma | Causa encontrada | Corrección aplicada | Verificación posterior |
|---|---|---|---|
|  |  |  |  |

## 5. Selección del nivel alcanzado

Marque con una **X** una sola banda en cada criterio y escriba el valor exacto que se capturará en Zubatronic. Si no existe evidencia obligatoria verificable, marque N1 y registre 0.

### C1. Ejecución e introspección del grafo ROS 2 — peso 25%

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, explica con precisión productores, consumidores y cambios del grafo; la ejecución es reproducible. |
| ☐ | N4 — 400–474 | Además de N3, identifica correctamente tipos, relaciones y cambios del grafo, con omisiones menores. |
| ☐ | N3 — 300–399 | Ejecuta el entorno e identifica nodos e interfaces principales mediante comandos de introspección. |
| ☐ | N2 — 150–299 | Logra una ejecución parcial; faltan salidas o confunde algunas responsabilidades del grafo. |
| ☐ | N1 — 0–149 | Presenta evidencia fragmentaria, no funcional o sin interpretación; incluye ausencia de evidencia obligatoria. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

### C2. Operación e interpretación de tópicos y mensajes — peso 20%

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, interpreta campos, unidades y efecto de `linear.x` y `angular.z` con precisión. |
| ☐ | N4 — 400–474 | Además de N3, inspecciona el tópico y explica correctamente el flujo publicador/suscriptor. |
| ☐ | N3 — 300–399 | Publica un `Twist`, produce el trazo solicitado y obtiene `topic echo`. |
| ☐ | N2 — 150–299 | Obtiene movimiento o datos parciales, pero mantiene errores de YAML o de interpretación. |
| ☐ | N1 — 0–149 | Presenta intentos sin una publicación funcional verificable o no presenta la evidencia obligatoria. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

### C3. Servicios, parámetros y rqt — peso 20%

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, distingue con precisión solicitud–respuesta, configuración y herramienta gráfica. |
| ☐ | N4 — 400–474 | Además de N3, identifica tipos y campos y explica el efecto de cada operación. |
| ☐ | N3 — 300–399 | Ejecuta correctamente `/clear`, cambia `background_r` y usa rqt con `/turtle1/set_pen`. |
| ☐ | N2 — 150–299 | Completa solamente una o dos operaciones o conserva errores de sintaxis. |
| ☐ | N1 — 0–149 | Presenta intentos sin operaciones funcionales verificables o no presenta la evidencia obligatoria. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

### C4. Decisión arquitectónica para `burger_delivery` — peso 20%

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, analiza correctamente una consecuencia de escoger la interfaz contraria. |
| ☐ | N4 — 400–474 | Además de N3, identifica datos o contratos y justifica cada elección. |
| ☐ | N3 — 300–399 | Propone dos tópicos y dos servicios pertinentes con sus participantes. |
| ☐ | N2 — 150–299 | Presenta menos de cuatro casos válidos o confunde tópico y servicio. |
| ☐ | N1 — 0–149 | Las propuestas son genéricas, no corresponden a la comunicación requerida o no se presenta la evidencia obligatoria. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

### C5. Evidencia reproducible y diagnóstico — peso 15%

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, el diagnóstico conecta síntoma, causa, corrección y prueba posterior de forma técnica. |
| ☐ | N4 — 400–474 | Además de N3, la evidencia es ordenada, atribuible y permite reproducir casi todos los resultados. |
| ☐ | N3 — 300–399 | Las evidencias permiten verificar el funcionamiento general y documentan una dificultad real. |
| ☐ | N2 — 150–299 | Hay capturas aisladas, registros incompletos o no se verifica la corrección. |
| ☐ | N1 — 0–149 | La evidencia es fragmentaria, ilegible, no atribuible o está ausente. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

## 6. Captura y cálculo

Capture en Zubatronic el valor exacto de cada criterio. La plataforma calcula:

```text
Aporte del criterio = valor del criterio × peso / 100
Nota del taller sobre 500 = suma de los cinco aportes
Nota del taller sobre 5,0 = nota sobre 500 / 100
```

Los valores guía 500, 450, 350, 250 y 100 agilizan una valoración discreta, pero puede usarse cualquier entero dentro del intervalo demostrado. La ausencia de evidencia obligatoria se registra con 0 dentro de N1.

## 7. Consolidado final de calificación

Zubatronic calcula este consolidado automáticamente. La tabla permite verificar la captura o registrar un respaldo manual; no depende de campos de fórmula de Word.

| Criterio | Peso | Nivel marcado | Valor 0–500 | Aporte ponderado |
|---|---:|:---:|---:|---:|
| C1. Grafo ROS 2 | 25% |  |  |  |
| C2. Tópicos y mensajes | 20% |  |  |  |
| C3. Servicios, parámetros y rqt | 20% |  |  |  |
| C4. Arquitectura `burger_delivery` | 20% |  |  |  |
| C5. Evidencia y diagnóstico | 15% |  |  |  |
| **TOTAL** | **100%** |  |  | **________ / 500** |

| Resultado final | Registro |
|---|---|
| Nota Taller 01 sobre 500 | __________ |
| Nota Taller 01 sobre 5,0 = total ÷ 100 | __________ |
| Criterios en N3 o superior | __________ |
| Criterios por debajo de N3 | __________ |
| Observaciones |  |

## 8. Uso posterior en la nota del corte

| Variable | Registro |
|---|---:|
| Peso del Taller 01 dentro del acumulado `T₁` | 100% para el primer corte 2026-2 |
| Aporte del Taller 01 a `T₁` | Nota Taller 01 × peso ÷ 100 |

```text
T₁ = Nota Taller 01
Nota Corte 1 = 0,28 T₁ + 0,42 L₁ + 0,30 E₁
```

Este documento entrega la **Nota Taller 01** y los resultados desagregados por criterio para su captura. En 2026-2, Taller 01 constituye el 100% de `T₁`; cualquier cambio exige publicar una nueva versión antes de recibir entregas. Zubatronic genera por separado los reportes RAE/SO.

## 9. Consolidación ABET y mejora continua

La nota académica anterior no se interpreta como un único nivel ABET. Para cada indicador se consolidan por separado los estudiantes evaluables.

| Indicador | N evaluable | N en N3 o superior | Porcentaje de logro | Meta | Hallazgo |
|---|---:|---:|---:|---:|---|
| 2.1 / SO2 |  |  |  | 70% |  |
| 3.1 / SO3 |  |  |  | 70% |  |

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

Firma o visto bueno: __________________________________________________________________
