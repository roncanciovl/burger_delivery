# GUÍA DE LABORATORIO 06: PLANIFICACIÓN DE TRAYECTORIAS CON MOVEIT 2 Y MOVEIT TASK CONSTRUCTOR

| FACULTAD | PROGRAMA | ASIGNATURA | SEMESTRE | CÓDIGO GUÍA | REVISIÓN |
|:---|:---|:---|:---:|:---:|:---:|
| Facultad de Ingeniería | Ingeniería Mecatrónica | ROBOT OPERATING SYSTEM - ROS | VIII – IX | GL-AA-F-1 / LAB-06 | 0.1 (borrador 2026-2) |

> [!NOTE]
> **Estado:** borrador, sin ejecutar con estudiantes. Usa el paquete `burger_control`
> (branch `claude/todo-modularizacion-paquetes`) y el paquete de Kinova
> `kinova_gen3_6dof_robotiq_2f_85_moveit_config` (`sudo apt install
> ros-jazzy-kinova-gen3-6dof-robotiq-2f-85-moveit-config ros-jazzy-moveit-task-constructor-core`).

---

## 1. CONTROL DE CAMBIOS

| Descripción del Cambio | Justificación | Fecha |
|---|---|:---:|
| Creación de la guía (borrador) | Pendiente de `TODO.md` §5 («Guía Lab 04: Planificación de trayectorias con MoveIt 2»), renumerada a 06 | 23/09/2026 |

---

## 2. INTRODUCCIÓN

La Guía 03 movía el brazo articulación por articulación con metas validadas a mano. Para
recoger una caja hace falta otra cosa: **planificar** en el espacio de la tarea, evitando la mesa,
con agarre y liberación coordinados con la pinza. MoveIt 2 resuelve el movimiento libre (OMPL),
el movimiento cartesiano (aproximación, elevación) y la cinemática inversa; MoveIt Task
Constructor (MTC) encadena esas piezas en **etapas** y busca una solución completa antes de
mover nada.

El nodo `burger_control/pick_place_node` ya implementa la tarea para el Gen3 de 6 GDL. En esta
práctica se recorre, se modifica y se mide, **primero sin robot** (hardware simulado) y sólo al
final, con el docente, sobre el brazo real.

## 3. OBJETIVOS

### 3.1. Objetivo General
Planificar y ejecutar de forma segura una tarea de pick & place con MoveIt Task Constructor en el
Kinova Gen3 de la celda.

### 3.2. Objetivos Específicos
1. Identificar grupos de planificación, estados con nombre y efector final en el SRDF del Gen3.
2. Interpretar cada etapa de la tarea MTC y sus fallos en RViz.
3. Medir el efecto del escalado de velocidad y de la pose de la caja sobre el éxito y la duración.
4. Ejecutar el plan en el robot real bajo el protocolo de seguridad.

## 4. DESCRIPCIÓN DE LA PRÁCTICA

| Fase | Contenido | Duración |
|---|---|:---:|
| 1 | SRDF, grupos y estados con nombre | 20 min |
| 2 | La tarea MTC en hardware simulado | 40 min |
| 3 | Experimentos de planificación | 40 min |
| 4 | Ejecución supervisada en el robot (docente) | 20 min |

### 4.1. RAE y ponderación

| Criterio | Indicador | SO | Peso |
|---|---|:---:|:---:|
| C1. Lectura del SRDF y de la tarea | 1.1 Modelado de sistemas robóticos | SO1 | 20% |
| C2. Diagnóstico de etapas fallidas | 6.4 Diagnóstico experimental | SO6 | 25% |
| C3. Experimentos y análisis | 6.4 Análisis de datos | SO6 | 35% |
| C4. Seguridad en la ejecución real | 4.1 Riesgos y paradas de emergencia | SO4 | 20% |

## 5. MATERIALES Y EQUIPOS

| DESCRIPCIÓN | CANTIDAD |
|---|:---:|
| Portátil con ROS 2 Jazzy, MoveIt 2, MTC y el workspace compilado | 1 por grupo |
| Caja de hamburguesa (≈ 0.10 × 0.10 × 0.06 m) | 1 |
| Kinova Gen3 6 GDL + Robotiq 2F-85, estación anfitriona y parada de emergencia (Fase 4) | 1 |

## 6. SEGURIDAD EN EL LABORATORIO

Fases 1 a 3: **sin robot**, en el dominio del grupo, con el driver en modo fake. Fase 4: sólo el
docente lanza `execute:=true`, desde la anfitriona, con la parada de emergencia en la mano, el
área despejada y `velocity_scaling: 0.1`. Nunca se ejecuta un plan que no se haya revisado
completo en RViz.

## 7. PROCEDIMIENTO EXPERIMENTAL

### Fase 1: SRDF, grupos y estados con nombre

```bash
SRDF=$(ros2 pkg prefix kinova_gen3_6dof_robotiq_2f_85_moveit_config)/share/kinova_gen3_6dof_robotiq_2f_85_moveit_config/config/gen3.srdf
grep -v disable_collisions "$SRDF"
```

**Tabla 1:** grupos (`manipulator`, `gripper`), efector final y estados (`Home`, `Retract`,
`Vertical`, `Open`, `Close`) con sus valores. Relaciona cada uno con un parámetro de
`burger_control/config/pick_place.yaml`.

### Fase 2: La tarea MTC en hardware simulado

```bash
# Terminal 1 — driver simulado
ros2 launch kortex_bringup gen3.launch.py dof:=6 gripper:=robotiq_2f_85 \
    robot_ip:=0.0.0.0 use_fake_hardware:=true launch_rviz:=false
# Terminal 2 — move_group + tarea (sólo planifica)
ros2 launch burger_control pick_place.launch.py
```

En RViz añade el panel **Motion Planning Tasks**. Recorre las etapas: `open hand`,
`move to pick`, `pick object` (aproximación, IK del agarre, cierre, *attach*, elevación),
`move to place`, `place object` y `return home`. Para cada una, anota cuántas soluciones generó y
cuántas fallaron.

### Fase 3: Experimentos de planificación

Modifica `pick_place.yaml` (o pasa parámetros con `--ros-args -p`) y registra en la **Tabla 2**
éxito/fallo, número de soluciones y duración de la trayectoria de la mejor solución:

1. `object_xyz` en cinco posiciones: centro del alcance, al límite (0.75 m), detrás de la base,
   pegada a la mesa (z = 0.03) y elevada (z = 0.20).
2. `velocity_scaling` en 0.05, 0.1 y 0.3 con la misma caja.
3. `tcp_offset_m` en 0.12, 0.15 y 0.18: explica qué etapa falla primero y por qué.

**Pregunta:** ¿por qué la etapa `generate grasp pose` genera varias orientaciones
(`setAngleDelta(π/12)`) y qué pierde un brazo de 6 GDL frente a uno de 7 en esa búsqueda?

### Fase 4: Ejecución supervisada (docente)

Con el driver real en la anfitriona y la caja en una posición validada en la Fase 3:

```bash
ros2 launch burger_control pick_place.launch.py use_fake_hardware:=false          # revisar el plan
ros2 launch burger_control pick_place.launch.py use_fake_hardware:=false execute:=true
```

Graba `/joint_states` y `/rosout` durante la ejecución (`burger_kinova_reference/scripts/record_kinova_bag.sh`).
Compara la duración real con la planificada.

## 8. ENTREGABLES

- Tablas 1 y 2 y las capturas del panel de MTC con al menos una etapa fallida explicada.
- Bag de la ejecución real (Fase 4) y la comparación de duraciones.

## 9. REFERENCIAS

- `docs/architecture/PAQUETES_ROS2.md` (sección `burger_control`).
- `docs/manipulation/MARCO_CONCEPTUAL_MOVEIT2.md`.
- M. Görner et al., «MoveIt! Task Constructor for Task-Level Motion Planning», IEEE ICRA, 2019.
- Tutorial oficial *Pick and Place with MoveIt Task Constructor* (moveit.picknik.ai, Jazzy).
