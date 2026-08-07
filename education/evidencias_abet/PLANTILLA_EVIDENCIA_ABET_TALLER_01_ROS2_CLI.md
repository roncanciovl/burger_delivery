# Formato de evidencias y calificación — Taller 01: CLI de ROS 2

> Instrumento calificable asociado a `education/talleres/TALLER_ROS2_CLI.md`. El taller conserva las instrucciones de aprendizaje; este formato permite registrar la evidencia, marcar el nivel alcanzado en cada criterio y obtener una única nota consolidada.

## 1. Identificación

| Campo | Registro |
|---|---|
| Asignatura | ROBOT OPERATING SYSTEM - ROS |
| Corte / instrumento | Primer corte / Talleres y tareas |
| Actividad | Taller 01 — CLI de ROS 2 |
| Nombre completo |  |
| Código / grupo |  |
| Fecha de entrega |  |
| Enlace o ubicación de la entrega |  |
| Commit evaluado, si aplica |  |

## 2. Uso del formato

1. La persona participante registra las evidencias E1–E9.
2. Quien evalúa marca **una sola casilla por criterio**, correspondiente al nivel más alto demostrado. El nivel es acumulativo: para marcarlo también deben cumplirse los niveles anteriores.
3. En el consolidado final se copia el aporte indicado en la tabla rápida y se suman los cinco aportes. No se asignan puntajes intermedios.

| Nivel | Valor fijo | Interpretación |
|---|---:|---|
| N5 | 500 | Excelente: desempeño completo, preciso y reproducible. |
| N4 | 450 | Bueno: desempeño correcto con omisiones menores. |
| N3 | 350 | Aceptable: logra lo esencial con explicación parcial. |
| N2 | 250 | Parcial: evidencia incompleta o errores importantes. |
| N1 | 100 | Inicial: evidencia mínima, fragmentaria o no funcional. |
| N0 | 0 | Sin evidencia verificable. |

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

Marque con una **X** una sola casilla en cada criterio. Si no existe evidencia verificable, marque N0.

### C1. Ejecución e introspección del grafo ROS 2 — peso 25%

| Marque | Nivel | Evidencia observable acumulativa |
|:---:|---|---|
| ☐ | N5 — 500 | Además de N4, explica con precisión productores, consumidores y cambios del grafo; la ejecución es reproducible. |
| ☐ | N4 — 450 | Además de N3, identifica correctamente tipos, relaciones y cambios del grafo, con omisiones menores. |
| ☐ | N3 — 350 | Ejecuta el entorno e identifica nodos e interfaces principales mediante comandos de introspección. |
| ☐ | N2 — 250 | Logra una ejecución parcial; faltan salidas o confunde algunas responsabilidades del grafo. |
| ☐ | N1 — 100 | Presenta evidencia fragmentaria, no funcional o sin interpretación. |
| ☐ | N0 — 0 | No presenta evidencia verificable para el criterio. |

**Nivel C1 marcado:** ________

### C2. Operación e interpretación de tópicos y mensajes — peso 20%

| Marque | Nivel | Evidencia observable acumulativa |
|:---:|---|---|
| ☐ | N5 — 500 | Además de N4, interpreta campos, unidades y efecto de `linear.x` y `angular.z` con precisión. |
| ☐ | N4 — 450 | Además de N3, inspecciona el tópico y explica correctamente el flujo publicador/suscriptor. |
| ☐ | N3 — 350 | Publica un `Twist`, produce el trazo solicitado y obtiene `topic echo`. |
| ☐ | N2 — 250 | Obtiene movimiento o datos parciales, pero mantiene errores de YAML o de interpretación. |
| ☐ | N1 — 100 | Presenta intentos sin una publicación funcional verificable. |
| ☐ | N0 — 0 | No presenta evidencia verificable para el criterio. |

**Nivel C2 marcado:** ________

### C3. Servicios, parámetros y rqt — peso 20%

| Marque | Nivel | Evidencia observable acumulativa |
|:---:|---|---|
| ☐ | N5 — 500 | Además de N4, distingue con precisión solicitud–respuesta, configuración y herramienta gráfica. |
| ☐ | N4 — 450 | Además de N3, identifica tipos y campos y explica el efecto de cada operación. |
| ☐ | N3 — 350 | Ejecuta correctamente `/clear`, cambia `background_r` y usa rqt con `/turtle1/set_pen`. |
| ☐ | N2 — 250 | Completa solamente una o dos operaciones o conserva errores de sintaxis. |
| ☐ | N1 — 100 | Presenta intentos sin operaciones funcionales verificables. |
| ☐ | N0 — 0 | No presenta evidencia verificable para el criterio. |

**Nivel C3 marcado:** ________

### C4. Decisión arquitectónica para `burger_delivery` — peso 20%

| Marque | Nivel | Evidencia observable acumulativa |
|:---:|---|---|
| ☐ | N5 — 500 | Además de N4, analiza correctamente una consecuencia de escoger la interfaz contraria. |
| ☐ | N4 — 450 | Además de N3, identifica datos o contratos y justifica cada elección. |
| ☐ | N3 — 350 | Propone dos tópicos y dos servicios pertinentes con sus participantes. |
| ☐ | N2 — 250 | Presenta menos de cuatro casos válidos o confunde tópico y servicio. |
| ☐ | N1 — 100 | Las propuestas son genéricas o no corresponden a la comunicación requerida. |
| ☐ | N0 — 0 | No presenta evidencia verificable para el criterio. |

**Nivel C4 marcado:** ________

### C5. Evidencia reproducible y diagnóstico — peso 15%

| Marque | Nivel | Evidencia observable acumulativa |
|:---:|---|---|
| ☐ | N5 — 500 | Además de N4, el diagnóstico conecta síntoma, causa, corrección y prueba posterior de forma técnica. |
| ☐ | N4 — 450 | Además de N3, la evidencia es ordenada, atribuible y permite reproducir casi todos los resultados. |
| ☐ | N3 — 350 | Las evidencias permiten verificar el funcionamiento general y documentan una dificultad real. |
| ☐ | N2 — 250 | Hay capturas aisladas, registros incompletos o no se verifica la corrección. |
| ☐ | N1 — 100 | La evidencia es fragmentaria, ilegible o no atribuible a la ejecución solicitada. |
| ☐ | N0 — 0 | No presenta evidencia verificable para el criterio. |

**Nivel C5 marcado:** ________

## 6. Tabla rápida de aportes

Después de marcar cada nivel, copie en el consolidado el aporte correspondiente. Los valores ya incluyen el peso del criterio.

| Criterio | Peso | N0 | N1 | N2 | N3 | N4 | N5 |
|---|---:|---:|---:|---:|---:|---:|---:|
| C1 | 25% | 0 | 25 | 62,5 | 87,5 | 112,5 | 125 |
| C2 | 20% | 0 | 20 | 50 | 70 | 90 | 100 |
| C3 | 20% | 0 | 20 | 50 | 70 | 90 | 100 |
| C4 | 20% | 0 | 20 | 50 | 70 | 90 | 100 |
| C5 | 15% | 0 | 15 | 37,5 | 52,5 | 67,5 | 75 |

## 7. Consolidado final de calificación

Copie únicamente los cinco aportes de la tabla rápida. En Word, use **Ctrl+A y F9** para actualizar automáticamente el total y su equivalente sobre 5,0.

| Criterio | Peso | Nivel marcado | Valor del nivel | Aporte según tabla rápida |
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
| Nivel global | __________ |
| Observaciones |  |

## 8. Uso posterior en la nota del corte

| Variable | Registro |
|---|---:|
| Peso del Taller 01 dentro del acumulado `T₁` | ______ % |
| Aporte del Taller 01 a `T₁` | Nota Taller 01 × peso ÷ 100 |

```text
T₁ = Σ(Nota de cada taller o tarea × su peso) / 100
Nota Corte 1 = 0,28 T₁ + 0,42 L₁ + 0,30 E₁
```

Este documento entrega directamente la **Nota Taller 01**. Esa nota pasa al consolidado de talleres y tareas del primer corte.

## 9. Cierre

Nombre de quien evalúa: ____________________________________  Fecha: __________________

Firma o visto bueno: __________________________________________________________________
