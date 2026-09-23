# GUÍA DE LABORATORIO 07: RAZONAMIENTO ESPACIAL CON IA MULTIMODAL (GEMINI ROBOTICS-ER)

| FACULTAD | PROGRAMA | ASIGNATURA | SEMESTRE | CÓDIGO GUÍA | REVISIÓN |
|:---|:---|:---|:---:|:---:|:---:|
| Facultad de Ingeniería | Ingeniería Mecatrónica | ROBOT OPERATING SYSTEM - ROS | VIII – IX | GL-AA-F-1 / LAB-07 | 0.1 (borrador 2026-2) |

> [!NOTE]
> **Estado:** borrador, sin ejecutar con estudiantes. Usa el nodo
> `gemini_spatial_reasoning_node` y el benchmark `benchmark_gemini_apriltag` de
> `burger_perception` (branches `claude/todo-gemini-vlm-node` y
> `claude/todo-benchmark-gemini-apriltag`). Cada llamada al modelo tiene costo: el docente
> define la cuota de llamadas por grupo antes de la sesión.

---

## 1. CONTROL DE CAMBIOS

| Descripción del Cambio | Justificación | Fecha |
|---|---|:---:|
| Creación de la guía (borrador) | Pendiente de `TODO.md` §5 («Guía Lab 05: Razonamiento espacial con IA multimodal»), renumerada a 07 | 23/09/2026 |

---

## 2. INTRODUCCIÓN

Un detector clásico encuentra *todas* las cajas. Un modelo de visión y lenguaje con
razonamiento espacial puede señalar *la* caja que cumple una condición («la que está libre, no la
que tiene un termo encima») y devolver un punto en la imagen. Para que el robot la recoja, ese
punto se lleva a 3D con la profundidad y la geometría de la cámara, y se publica como TF
(`base_link -> target_burger_box_frame`) para `burger_control`.

Esta práctica separa lo que aporta el modelo (entender la escena) de lo que aporta la geometría
(convertir un píxel en metros), y mide ambas cosas contra una referencia AprilTag.

> [!WARNING]
> **Error frecuente al desproyectar.** `projectPixelTo3dRay` devuelve un rayo **unitario**. El
> punto correcto es `ray * z / ray[2]`, no `ray * z`: lo segundo acorta el punto fuera del eje
> óptico (ver `docs/research/EXPERIMENTO_IA_LOCALIZACION_GEMINI.md`).

## 3. OBJETIVOS

### 3.1. Objetivo General
Integrar un VLM de razonamiento espacial en el pipeline ROS 2 de la celda y evaluar su exactitud,
latencia y costo frente a un localizador fiducial.

### 3.2. Objetivos Específicos
1. Diseñar prompts con restricciones espaciales y evaluar su discriminación.
2. Obtener el punto 3D del objeto con profundidad registrada y verificarlo con cinta métrica.
3. Medir error 3D, latencia y bytes enviados frente a AprilTag.
4. Argumentar dónde puede y dónde no puede ir un VLM en el lazo de control.

## 4. DESCRIPCIÓN DE LA PRÁCTICA

| Fase | Contenido | Duración |
|---|---|:---:|
| 1 | Clave, nodo y primera llamada | 20 min |
| 2 | Prompts con restricciones espaciales (2D) | 30 min |
| 3 | De píxel a metros: verificación con cinta | 30 min |
| 4 | Benchmark contra AprilTag | 40 min |

### 4.1. RAE y ponderación

| Criterio | Indicador | SO | Peso |
|---|---|:---:|:---:|
| C1. Integración correcta y segura de la clave | 2.2 Restricciones de seguridad en sistemas heterogéneos | SO2 | 15% |
| C2. Diseño y evaluación de prompts | 7.1 Adquiere y aplica nuevas herramientas | SO7 | 25% |
| C3. Geometría de desproyección verificada | 1.1 Modelado matemático | SO1 | 25% |
| C4. Benchmark y análisis costo-exactitud | 6.4 Análisis experimental | SO6 | 35% |

## 5. MATERIALES Y EQUIPOS

| DESCRIPCIÓN | CANTIDAD |
|---|:---:|
| Cámara del Kinova publicando color y profundidad registrada (`depth_registration:=true`) | 1 |
| Clave de API de Gemini provista por el docente, con cuota | 1 por grupo |
| Dos cajas de hamburguesa, un termo u objeto distractor | 1 juego |
| Tag 36h11 de 50 mm pegado al centro de la tapa de una caja | 1 |

## 6. SEGURIDAD EN EL LABORATORIO

La clave **nunca** va en un archivo del repositorio ni como parámetro ROS (quedaría en bags y en
`ros2 param dump`): sólo `export GEMINI_API_KEY=...` en la terminal del nodo. El brazo no se
mueve en esta práctica; la cámara ya está en la pose de observación. La inferencia es bajo
demanda (`~/locate`), nunca periódica sin permiso del docente.

## 7. PROCEDIMIENTO EXPERIMENTAL

### Fase 1: Clave, nodo y primera llamada

```bash
pip install google-genai
export GEMINI_API_KEY=...        # la del grupo
ros2 launch burger_perception gemini.launch.py
ros2 service call /gemini_spatial_reasoning/locate std_srvs/srv/Trigger
```

Anota la respuesta, la latencia y los KB enviados que imprime el nodo. Revisa `~/inference`
(`ros2 topic echo /gemini_spatial_reasoning/inference --once`).

### Fase 2: Prompts con restricciones espaciales

Escenas (de `EXPERIMENTO_IA_LOCALIZACION_GEMINI.md` §7): dos cajas, una con un termo encima.
Cambia `target` y `context` (`--ros-args -p target:=... -p context:=...` o en `config/gemini.yaml`)
y registra en la **Tabla 1**, para 5 llamadas por prompt, cuántas veces señala la caja correcta:

1. `target: box` sin contexto.
2. `target: cardboard burger box`, `context: It must be free, with nothing on top of it.`
3. El mismo, moviendo el termo a la otra caja.

### Fase 3: De píxel a metros

1. Con la caja libre, llama a `~/locate` y lee `ros2 run tf2_ros tf2_echo base_link target_burger_box_frame`.
2. Mide con cinta la posición de la caja respecto a la base del robot.
3. Repite con `depth_mode: plane` y `plane_distance_m` = distancia cámara–mesa medida. Explica la
   diferencia con el modo `registered`.

### Fase 4: Benchmark contra AprilTag

```bash
ros2 run burger_perception benchmark_gemini_apriltag --tag-id 7 --tag-size 0.05 \
    --ensayos 10 --interactivo --salida bench_eqNN
```

Mueve la caja entre ensayos. Con `resumen.json` y `figura_benchmark.png` completa la **Tabla 2**:
error 3D (media, p95), error en píxeles, latencia media y KB por imagen. Usa `error_px` para
separar los errores del modelo de los de la profundidad (`docs/research/BENCHMARK_GEMINI_APRILTAG.md`).

**Pregunta de cierre:** con la latencia medida, ¿a qué frecuencia máxima podría actualizarse el
objetivo? ¿Por qué el VLM se consulta una vez por tarea y el lazo cerrado lo hace MoveIt con el
TF ya fijado?

## 8. ENTREGABLES

- Tablas 1 y 2, `resumen.json`, la figura y tres imágenes anotadas (una con error grande explicado).
- Argumento escrito (media página) sobre costo, latencia y exactitud frente a AprilTag.

## 9. REFERENCIAS

- `docs/research/EXPERIMENTO_IA_LOCALIZACION_GEMINI.md` y `ros2_setup/PROPUESTA_GEMINI_ER.md`.
- `docs/research/BENCHMARK_GEMINI_APRILTAG.md`.
- Gemini Robotics Team, «Gemini Robotics: Bringing AI into the Physical World», arXiv:2503.20020, 2025.
