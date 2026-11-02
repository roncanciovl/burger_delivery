# GUÍA DE LABORATORIO 04 (CORTE 3): PLANIFICACIÓN CON MOVEIT 2 Y EJECUCIÓN SUPERVISADA DEL KINOVA GEN3 SOBRE LA ESCENA DE BURGER_DESCRIPTION

---

| FACULTAD | PROGRAMA | ASIGNATURA | SEMESTRE | CÓDIGO GUÍA | REVISIÓN |
|:---|:---|:---|:---:|:---:|:---:|
| Facultad de Ingeniería | Ingeniería Mecatrónica | ROBOT OPERATING SYSTEM - ROS | VIII – IX | GL-AA-F-1 / LAB-04 (Corte 3) | 1.1 (2026-2) |

> [!IMPORTANT]
> **Actualización del Repositorio Privado del Equipo:**
> Antes de iniciar o continuar con este laboratorio, asegúrese de haber sincronizado su repositorio privado de equipo con los últimos cambios de la base del curso. Consulte la [Guía Oficial de Sincronización y Actualizaciones](../proyectos_evaluables/ACTUALIZACIONES_BASE_CORTE_1.md) (`education/proyectos_evaluables/ACTUALIZACIONES_BASE_CORTE_1.md`) para realizar este proceso correctamente.

---

## 1. CONTROL DE CAMBIOS

| Descripción del Cambio | Justificación | Fecha |
|---|---|:---:|
| Creación e implementación de la Guía de Laboratorio 04 | Lleva MoveIt 2 al Kinova Gen3 real sin perder las salvaguardas validadas en los cortes 1 y 2: `move_group` sólo planifica y `burger_kinova_reference` valida y ejecuta cada trayectoria (`moveit_safe_executor`). La mesa y los marcos de la celda se toman de `burger_description`. | 19/10/2026 |
| Base del Gen3 apoyada sobre la cara superior de la mesa en `burger_description` | Al cargar la escena en MoveIt, la mesa invadía 25 mm la base del robot (`table_to_world` en z = 0 con la mesa centrada en `table_link`) y MoveIt declaraba en colisión el estado inicial. `table_to_world` pasa a z = 0.025 m, la misma altura de `tag_mesa`. La Fase 1 reproduce el fallo a propósito. | 19/10/2026 |
| Instrumento ABET de entrega | Se crea `INSTRUMENTO_ABET_LAB_04_MOVEIT2_EJECUCION_SEGURA` (Markdown y Word) con las Tablas 1 a 6, las evidencias E1 a E9, el cuestionario, el Anexo A y las rúbricas de C1 a C5. Las secciones 5.2 y 8 lo citan como formato de entrega. | 21/10/2026 |
| Aviso de sincronización con la base del curso | Igual que en los talleres, la guía empieza recordando sincronizar el repositorio privado del equipo con la base del curso (`base-latest`) antes del laboratorio. | 21/10/2026 |
| Ctrl+C cancela la trayectoria en `moveit_safe_executor` | Al validar la guía, Ctrl+C terminaba el ejecutor pero no cancelaba la meta: el brazo completaba la trayectoria. Corregido en `burger_kinova_reference`. La prueba 4.7 indica ahora el mensaje esperado. | 22/10/2026 |
| Anfitriona por cable, sin WiFi, y RViz con prioridad baja | Con el WiFi conectado junto al cable, o con RViz compitiendo por la CPU, el ciclo de control del driver se retrasa y el brazo vibra durante la trayectoria (`TROUBLESHOOTING.md` §3.7). La Fase 6 indica cómo dejar la anfitriona sólo por cable. `moveit_planificacion.launch.py` abre RViz con `nice -n 19` cuando el hardware es real. | 22/10/2026 |
| Fase 6: llevar el brazo a Home antes del modo seco | Desde una pose con la muñeca girada media vuelta, la meta `home` exige recorrer ~3.1 rad en `joint_4` y `joint_6`, y `max_joint_travel_rad` (3.0) la bloquea. El modo seco sirve de comprobación. | 22/10/2026 |
| Pruebas 4.1, 4.6 y 4.8 | Tras la Fase 3 el brazo ya está en `home`: la 4.1 lo advierte y la 4.6 se provoca durante `sobre_tag_mesa`. En la 4.8 el planificador es aleatorio y el resultado varía de una corrida a otra. Se pide repetirla y se explica qué demuestra cada resultado. | 22/10/2026 |

---

## 2. INTRODUCCIÓN

### 2.1. Del punto articular a la tarea en el espacio

En el Laboratorio 03 cada turno enviaba **una** meta articular cercana: ninguna articulación podía moverse más de `max_joint_delta_rad` (0.35 rad) desde la pose medida. Para llevar la pinza a un punto de la mesa hace falta otra cosa: **planificar**. Hay que encontrar una configuración articular que ponga el efector donde se pide (cinemática inversa), un camino que no atraviese la mesa (detección de colisiones) y una ley de tiempo que respete los límites de velocidad y aceleración (parametrización temporal). Eso es MoveIt 2:

| Pieza de MoveIt 2 | Qué resuelve en esta práctica |
|---|---|
| `move_group` | Nodo central: recibe las peticiones de planificación y mantiene la escena |
| SRDF (`kinova_gen3_6dof_robotiq_2f_85_moveit_config`) | Grupos `manipulator` y `gripper`, efector `end_effector_link`, estados `Home`, `Retract`, `Vertical` |
| OMPL | Planificador por muestreo para movimientos libres (metas articulares y de pose) |
| `compute_cartesian_path` | Tramos en línea recta del efector (aproximación y retirada) |
| Escena de planificación | Obstáculos que el planificador debe evitar: aquí, la mesa |

### 2.2. Quién planifica y quién ejecuta

Un plan de MoveIt es una trayectoria de decenas de puntos con tiempos. Si `move_group` la ejecuta directamente, la envía al `joint_trajectory_controller` y se salta todo lo que el paquete de referencia validó en el robot: `enable_motion`, modo seco, confirmación del operador, telemetría fresca y límites aprobados. En esta práctica la responsabilidad se separa:

```text
 burger_description ──(mesa)──► escena_moveit ──► move_group  (allow_trajectory_execution: false)
                                                     ▲   │ plan
 metas_moveit.yaml / botón Plan de RViz ──► moveit_safe_executor ──(valida, confirma)──► joint_trajectory_controller
                                                     ▲
                                              /joint_states
```

- **`move_group` no puede mover el robot.** Se lanza con `allow_trajectory_execution: false`: el botón *Execute* de RViz no hace nada.
- **`moveit_safe_executor` es la única puerta.** Pide cada plan **desde la pose medida**, valida la trayectoria completa, muestra un informe, pide confirmación escribiendo `si` y, durante el movimiento, cancela la meta si la telemetría se pierde o si alguien pulsa Ctrl+C.

### 2.3. Reglas sobre la trayectoria completa

`validate_goal` (cortes 1 y 2) no sirve para un plan: un plan puede recorrer más de un radián rodeando la mesa. `trajectory_validation.py` aplica estas reglas, todas a la vez:

| Regla | Parámetro | Por qué |
|---|---|---|
| El primer punto coincide con la pose medida | `start_tolerance_rad` = 0.02 | Un plan calculado sobre una pose vieja movería el brazo desde donde ya no está |
| Todos los puntos dentro de los límites aprobados | `joint_min_rad`, `joint_max_rad` | Los mismos de los cortes anteriores; nunca se amplían |
| Velocidad entre puntos y declarada | `max_velocity_rad_s` = 0.35 | Tope del laboratorio, independiente del escalado que se pida a MoveIt |
| Sin saltos de 2π en articulaciones continuas | `continuous_joints` | Un paso de +3.1 a −3.1 rad es un cambio de representación, no un giro corto |
| Recorrido acumulado y duración acotados | `max_joint_travel_rad` = 3.0, `max_trajectory_duration_s` = 60 | Un plan que serpentea es una señal de alarma |
| Telemetría fresca y `enable_motion` | igual que en el Laboratorio 03 | Sin estado medido no hay ejecución segura |

> [!IMPORTANT]
> **El validador no ve la geometría.** Comprueba articulaciones y tiempos, no colisiones. La única protección contra atravesar la mesa es que la mesa **esté en la escena** y **en el lugar correcto**. Por eso la escena se toma de `burger_description`, el mismo modelo que comparten todos los equipos, y por eso la Fase 1 empieza verificándola.

### 2.4. La escena de `burger_description` en el marco del robot

En `delivery_scene_fixed.urdf` el eslabón `world` es donde se monta la base del Gen3, es decir, el `base_link` del driver. `escena_moveit` recorre las articulaciones fijas de la escena y expresa cada caja de colisión en ese marco:

| Elemento de la escena | Pose en `base_link` (m) | Uso en la práctica |
|---|---|---|
| `table_link` (caja 1.2 × 0.8 × 0.05) | centro en (−0.20, −0.30, −0.025) | Obstáculo de MoveIt |
| `tag_mesa` | (0.30, 0.00, 0.00) | Marco de las metas tipo `pose` |
| `delivery_slot_1` | (−0.20, 1.10, −0.30) | Fuera del alcance; ejemplo de meta inalcanzable |

Una meta de tipo `pose` se escribe **en un marco de la escena**. Por ejemplo, «0.30 m sobre `tag_mesa`, con la pinza hacia abajo» se escribe `marco: tag_mesa`, `posicion: [0, 0, 0.30]`, `rpy: [π, 0, 0]`. Si mañana se corrige la posición de la mesa en el URDF, la meta se corrige sola.

---

## 3. OBJETIVOS

### 3.1. Objetivo General

Planificar con MoveIt 2 movimientos del Kinova Gen3 sobre la escena de `burger_description` y ejecutarlos en el robot real a través de la validación de trayectorias de `burger_kinova_reference`, verificando experimentalmente cada salvaguarda.

### 3.2. Objetivos Específicos

1. Levantar MoveIt 2 en modo sólo planificación sobre el driver de la estación anfitriona (o el driver fake) y cargar la mesa de `burger_description` como obstáculo.
2. Interpretar el informe de validación de una trayectoria: puntos, duración, recorrido y velocidad pico por articulación.
3. Provocar y explicar cada regla de bloqueo del validador y cada fallo de planificación.
4. Diseñar metas propias expresadas en marcos de la celda (`tag_mesa`) y validarlas en modo seco.
5. Ejecutar la secuencia en el robot real bajo el protocolo de turnos y comparar lo ejecutado con lo planificado.

---

## 4. DESCRIPCIÓN DE LA PRÁCTICA

| Fase | Contenido | Dónde | Duración |
|:---:|---|---|:---:|
| 0 | Entorno, compilación y dominio del grupo | Portátil del grupo | 15 min |
| 1 | MoveIt sólo planificación y escena de `burger_description` | Driver fake, dominio del grupo | 25 min |
| 2 | Secuencia en modo seco: leer el informe de validación | Driver fake | 20 min |
| 3 | Ejecución en hardware simulado | Driver fake | 15 min |
| 4 | Provocar cada bloqueo | Driver fake | 30 min |
| 5 | Metas propias sobre la celda | Driver fake | 20 min |
| 6 | Ejecución en el robot real por turnos | Anfitriona, dominio 0 | 10 min por grupo |

Las fases 0 a 5 no tocan el robot: cada grupo trabaja con su propio driver fake en **su** dominio. Sólo la Fase 6 usa el Kinova real, con un único `move_group` en la estación anfitriona y un grupo ejecutando a la vez.

### 4.1. Resultados de Aprendizaje Evaluables (RAE) y Ponderación

| Criterio | RAE / Indicador de Desempeño | SO | Ponderación |
|---|---|:---:|:---:|
| **C1. Escena, cinemática y planificación** | **1.1.** Aplica principios matemáticos (transformaciones homogéneas, cinemática directa e inversa) para resolver problemas espaciales en robótica. | SO1 | 25% |
| **C2. Integración de MoveIt 2 con el paquete de referencia** | **2.1.** Diseña soluciones de software integrando contratos ROS 2 (servicios, acciones, QoS) en sistemas robóticos reales. | SO2 | 20% |
| **C3. Validación experimental y diagnóstico de bloqueos** | **6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos por capas para aislar errores de planificación, validación y ejecución. | SO6 | 25% |
| **C4. Seguridad en la ejecución real** | **4.1.** Identifica riesgos de seguridad física, paradas de emergencia y celdas robóticas. | SO4 | 20% |
| **C5. Trazabilidad y trabajo en equipo** | **3.1 - 3.3.** Elabora documentación técnica reproducible y registra evidencias.<br>**5.1.** Define roles técnicos y coordina la ejecución en equipo. | SO3 / SO5 | 10% |
| **Total** | | | **100%** |

Cada criterio se califica en cinco niveles sobre la escala de 0 a 500. **N3 es el umbral individual de logro** y la meta de cohorte es que al menos el 70 % de los estudiantes alcance N3 o superior en cada indicador.

| Nivel | Intervalo | Interpretación |
|:---:|:---:|---|
| **N5** | 475–500 | Excelente: evidencia completa y reproducible; explica cada bloqueo con su causa y su regla. |
| **N4** | 400–474 | Bueno: desempeño correcto con omisiones menores que no comprometen la validez de las mediciones. |
| **N3** | 300–399 | Aceptable: completa las fases con evidencia verificable (umbral de logro). |
| **N2** | 150–299 | Parcial: evidencia incompleta o bloqueos sin explicar. |
| **N1** | 0–149 | No cumple: evidencia mínima, o ejecuta en el robot fuera del protocolo. |

> [!NOTE]
> Los indicadores 1.1, 2.1, 3.1 - 3.3, 4.1, 5.1 y 6.4 se citan como aparecen en los instrumentos de las prácticas anteriores. Verifíquelos contra el syllabus vigente.

---

## 5. MATERIALES Y EQUIPOS

### 5.1. Equipos del Laboratorio
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Brazo Kinova Gen3 (6 GDL) con pinza Robotiq 2F-85 (IP `192.168.1.10`) | 1 | Unidad |
| Estación anfitriona con Ubuntu 24.04, ROS 2 Jazzy, MoveIt 2 y **cable Ethernet** al router (WiFi desconectado durante la práctica) | 1 | Unidad |
| Router TP-Link AX12 (SSID `ros2`, subred `192.168.1.0/24`) | 1 | Unidad |
| Pulsador de parada de emergencia física, accesible desde la anfitriona | 1 | Unidad |
| Mesa de la celda con `tag_mesa` instalado; cinta métrica | 1 | Juego |

### 5.2. Equipos del Estudiante
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Portátil con Ubuntu 24.04, ROS 2 Jazzy y MoveIt 2 (`ros-jazzy-moveit`, `ros-jazzy-kinova-gen3-6dof-robotiq-2f-85-moveit-config`) | 1 | Por grupo |
| Workspace `~/ros2_ws` con `burger_delivery` y `ros2_kortex` compilados | 1 | Workspace |
| Instrumento de entrega del laboratorio (DOCX) con las Tablas 1 a 6, más los registros `*.jsonl` | 1 | Documento |

---

## 6. SEGURIDAD EN EL LABORATORIO

> [!WARNING]
> 1. **Fases 0 a 5 sin robot.** Cada grupo usa su driver fake en su propio `ROS_DOMAIN_ID`. Un driver fake en el dominio 0 publicaría `/joint_states` y `/joint_trajectory_controller` con los mismos nombres que el robot real.
> 2. **Un solo `move_group` y un solo ejecutor en el dominio 0.** En la Fase 6 la anfitriona lanza el driver real y el único `move_group`. Sólo el grupo en turno corre `moveit_safe_executor`. Dos ejecutores en el dominio enviarían metas al mismo controlador.
> 3. **Modo seco primero, siempre.** Ninguna meta se ejecuta en el robot si antes no pasó en modo seco **con el robot en la misma pose** y el docente no revisó el informe.
> 4. **Custodia de la parada de emergencia.** Un integrante de la anfitriona permanece junto al pulsador durante todo el turno. Radio libre de 1.2 m alrededor de la base.
> 5. **No ampliar límites.** `joint_min_rad`, `joint_max_rad`, `max_velocity_rad_s` y `start_tolerance_rad` no se modifican para «hacer pasar» un plan. En la Fase 4 se **reducen** a propósito para provocar bloqueos; nunca se aumentan.
> 6. **Ctrl+C cancela.** En el ejecutor, Ctrl+C cancela la trayectoria en curso y el controlador detiene el brazo. El ejecutor lo confirma con `[SEGURIDAD] Cancelando la trayectoria en curso`. Si ese mensaje no aparece, la base del grupo está desactualizada: use la parada de emergencia y sincronice antes de seguir. La parada de emergencia física sigue siendo la protección última.
> 7. **La anfitriona va sólo por cable.** El driver se lanza con el WiFi de la anfitriona desconectado. Si el WiFi sigue conectado en la misma subred, el ciclo de control del driver se retrasa aunque la ruta al robot salga por el cable, y el brazo vibra durante la trayectoria (`TROUBLESHOOTING.md` §3.7). Las estaciones de los grupos sí pueden ir por WiFi: sólo consumen telemetría.

---

## 7. PROCEDIMIENTO EXPERIMENTAL

### Fase 0: Entorno, Compilación y Dominio del Grupo

1. Actualice el repositorio del grupo con la base del curso ([Guía de Sincronización](../proyectos_evaluables/ACTUALIZACIONES_BASE_CORTE_1.md)) y compile:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select burger_description burger_kinova_reference
   source install/setup.bash
   colcon test --packages-select burger_kinova_reference
   colcon test-result --verbose --test-result-base build/burger_kinova_reference
   ```
   Las pruebas de `test_trajectory_validation.py`, `test_escena_urdf.py` y `test_metas_moveit.py` deben pasar. Registre el resultado en la **Tabla 1**.
2. Fije el dominio del grupo **en todas las terminales** (convención `eqNN` del Laboratorio 03):
   ```bash
   export ROS_DOMAIN_ID=$((10 + NN))      # NN = número de grupo, p. ej. 13 para eq03
   timeout 5s ros2 daemon stop; ros2 daemon start
   ```

---

### Fase 1: MoveIt Sólo Planificación y Escena de `burger_description`

1. **Terminal 1 — driver fake** (sin robot; la pinza se omite en modo fake):
   ```bash
   ros2 launch burger_kinova_reference kinova_connection.launch.py \
       start_driver:=true robot_ip:=0.0.0.0 use_fake_hardware:=true
   ```
2. **Terminal 2 — MoveIt sólo planificación + mesa:**
   ```bash
   ros2 launch burger_kinova_reference moveit_planificacion.launch.py
   ```
   En el log de `escena_moveit` debe aparecer `table_link: caja [1.2, 0.8, 0.05] en base_link (-0.200, -0.300, -0.025) m` y `1 obstáculo(s) cargados`. En RViz, la mesa aparece como objeto de colisión bajo el robot.
3. **Compruebe que MoveIt no puede ejecutar.** En el panel MotionPlanning elija *Goal State: Home*, pulse **Plan** y luego **Execute**. Copie en la **Tabla 2** el aviso de `move_group` y explique por qué es deseable.
4. **Reproduzca el error de modelado** que motivó el cambio de `burger_description` (ver Control de Cambios):
   ```bash
   cp $(ros2 pkg prefix burger_description)/share/burger_description/urdf/delivery_scene_fixed.urdf /tmp/escena_z0.urdf
   sed -i '/name="table_to_world"/,/<\/joint>/s/0.20 0.30 0.025/0.20 0.30 0.00/' /tmp/escena_z0.urdf
   ros2 run burger_kinova_reference escena_moveit --ros-args -p scene_urdf:=/tmp/escena_z0.urdf
   echo "código de salida: $?"
   ```
   `escena_moveit` debe negarse a cargar la escena y terminar con código 1. Con la cinemática directa, calcule a mano a qué altura queda la cara superior de la mesa respecto a `base_link` en cada versión y anótelo en la **Tabla 2**.

> [!TIP]
> **Comprobar el efecto, no el código de salida** (`TROUBLESHOOTING.md` §4.7). Que `escena_moveit` termine con 0 no basta: confirme en RViz (*Scene Objects*) que la caja existe y está bajo el robot.

---

### Fase 2: Secuencia en Modo Seco — Leer el Informe de Validación

1. **Terminal 3 — ejecutor en modo seco** (valor por defecto):
   ```bash
   SHARE=$(ros2 pkg prefix burger_kinova_reference)/share/burger_kinova_reference
   ros2 run burger_kinova_reference moveit_safe_executor --ros-args \
       --params-file $SHARE/config/kinova_connection.yaml \
       -p goals_file:=$SHARE/config/metas_moveit.yaml \
       -p report_file:=$HOME/lab04_eqNN/seco.jsonl
   ```
   La secuencia por defecto es `home → sobre_tag_mesa → bajar_5cm → subir_5cm → home`. En modo seco el brazo no se mueve: cada meta se planifica desde la pose **final del plan anterior** y el log lo indica.
2. Para cada meta, copie del informe a la **Tabla 3**: tipo, número de puntos, duración, articulación de mayor recorrido y velocidad pico. Verifique que ninguna velocidad pico supera 0.35 rad/s.
3. Abra `metas_moveit.yaml` y relacione la pose objetivo que imprime el ejecutor para `sobre_tag_mesa` (`Pose objetivo en base_link: (+0.300, +0.000, +0.300) m`) con la pose de `tag_mesa` de la sección 2.4.

---

### Fase 3: Ejecución en Hardware Simulado

1. Relance el ejecutor habilitando el movimiento (sigue siendo el driver fake):
   ```bash
   ros2 run burger_kinova_reference moveit_safe_executor --ros-args \
       --params-file $SHARE/config/kinova_connection.yaml \
       -p goals_file:=$SHARE/config/metas_moveit.yaml \
       -p dry_run:=false -p enable_motion:=true \
       -p report_file:=$HOME/lab04_eqNN/fake.jsonl
   ```
   Confirme cada meta escribiendo `si`. Observe el movimiento en RViz.
2. Al final de cada meta el ejecutor compara la pose medida con la final del plan (`desvío final máximo`). Anótelo en la **Tabla 3**.

---

### Fase 4: Provocar Cada Bloqueo

Ejecute cada prueba en modo seco o con el driver fake y anote en la **Tabla 4** la regla que actuó, el mensaje exacto, el código de salida (`echo $?`) y su explicación. **Sólo se reducen límites; nunca se aumentan.**

Primero copie las metas a la carpeta del grupo; las pruebas 4.4 y 4.5 añaden metas a esa copia, que se pasa con `-p goals_file:=$HOME/lab04_eqNN/metas_eqNN.yaml`:

```bash
mkdir -p $HOME/lab04_eqNN && cp $SHARE/config/metas_moveit.yaml $HOME/lab04_eqNN/metas_eqNN.yaml
```

| N.º | Prueba | Cómo provocarla |
|:---:|---|---|
| 4.1 | Velocidad | `-p max_velocity_rad_s:=0.05`. Si el brazo quedó en `home` tras la Fase 3, `home` pasa (el plan apenas se mueve) y el bloqueo aparece en `sobre_tag_mesa` |
| 4.2 | Velocidad declarada por MoveIt | `-p velocity_scaling:=0.6` (deja `max_velocity_rad_s` en 0.35) |
| 4.3 | Arranque desde otra pose | `-p source:=rviz -p dry_run:=false -p enable_motion:=true`; en RViz elija *Start State: random valid*, *Goal State: Home* y pulse **Plan** |
| 4.4 | Meta inalcanzable | Añada `inalcanzable`: tipo `pose`, `marco: delivery_slot_1`, `posicion: [0, 0, 0.2]`, `rpy: [3.14159, 0, 0]`; ejecute con `-p sequence:="['home', 'inalcanzable']"` |
| 4.5 | Meta dentro de la mesa | Añada `en_la_mesa`: tipo `pose`, `marco: tag_mesa`, `posicion: [0, 0, -0.05]`, `rpy: [3.14159, 0, 0]`; ejecute con `-p sequence:="['home', 'en_la_mesa']"` |
| 4.6 | Telemetría perdida | Ejecute como en la Fase 3 con `-p "sequence:=['sobre_tag_mesa', 'home']"` y, mientras el brazo se mueve hacia `sobre_tag_mesa`, detenga el driver fake con Ctrl+C en la Terminal 1. Tras la Fase 3 el brazo ya está en `home`, por eso no sirve provocarla durante `home` |
| 4.7 | Cancelación del operador | Durante la ejecución, pulse Ctrl+C en la Terminal 3. Debe aparecer `[SEGURIDAD] Cancelando la trayectoria en curso` y, en RViz, el brazo queda detenido a mitad de camino |
| 4.8 | Sin la mesa en la escena | `ros2 run burger_kinova_reference escena_moveit --ros-args -p remove:=true` y repita 4.5 en modo seco |

> [!IMPORTANT]
> **La prueba 4.8 es la más importante de la práctica.** Sin la mesa, MoveIt **encuentra** un plan hacia un punto **dentro** de la mesa, cuando en la 4.5 no lo encontraba, y el validador no tiene ninguna regla que mencione la mesa, porque no ve la geometría.
>
> El planificador (OMPL) es aleatorio, así que el resultado cambia de una corrida a otra. Al validar la guía, de 7 corridas 2 terminaron en `PERMITIDA` y las otras 5 las bloqueó otra regla: el recorrido acumulado (`max_joint_travel_rad`) o los límites de ±3.14 rad. Repita la prueba hasta obtener un `PERMITIDA`, con un máximo de 5 corridas, y anote en la Tabla 4 el resultado y la regla de cada una. Un bloqueo por recorrido no protege de la mesa: con otro plan, el mismo punto pasa.
>
> Explique en el análisis qué capa protege de qué y vuelva a cargar la escena antes de continuar: `ros2 run burger_kinova_reference escena_moveit`.

---

### Fase 5: Metas Propias sobre la Celda

1. En `~/lab04_eqNN/metas_eqNN.yaml` (la copia de la Fase 4) añada:
   - `aproximacion_eqNN`: tipo `pose`, `marco: tag_mesa`, una posición distinta de la de `sobre_tag_mesa` dentro de un cuadrado de ±0.10 m en x e y, a 0.30 m de altura, con la pinza hacia abajo.
   - `descenso_eqNN`: tipo `cartesiana`, que baje la pinza entre 0.03 y 0.08 m.
   - Una `secuencia` que vaya de `home` a sus metas y regrese a `home`.
2. Valídela en modo seco y después ejecútela con el driver fake (Fase 3). Copie los informes a la **Tabla 5**.
3. **Predicción:** antes de ejecutarla, calcule con la sección 2.4 la posición del efector en `base_link` para `aproximacion_eqNN`. Compárela con la que imprime el ejecutor.

---

### Fase 6: Ejecución en el Robot Real por Turnos

**Estación anfitriona** (docente o grupo anfitrión), dominio 0, **sólo por cable**:

1. **Antes de lanzar el driver**, desconecte el WiFi de la anfitriona y compruebe que sólo queda la interfaz del cable. En WSL, desconéctelo desde Windows (en PowerShell: `netsh wlan disconnect interface="<nombre del adaptador>"`).
   ```bash
   ip -brief -4 addr                  # sólo lo y la interfaz del cable
   ip route get 192.168.1.10          # la ruta al robot sale por esa interfaz
   ping -c 50 -i 0.05 -q 192.168.1.10 # mdev ≲ 0.5 ms; con WiFi activo rondaba 2.8 ms
   ```
   Si el driver, la escena o el daemon de ROS se lanzaron con el WiFi activo, relánzalos: la sesión del driver y Cyclone DDS quedan atados a la interfaz del WiFi (`TROUBLESHOOTING.md` §3.7).
2. Lance el driver y MoveIt. Con `use_fake_hardware:=false`, `moveit_planificacion.launch.py` abre RViz con `nice -n 19` para que no retrase el ciclo de control del driver.
   ```bash
   export ROS_DOMAIN_ID=0
   ros2 launch burger_kinova_reference kinova_connection.launch.py \
       start_driver:=true robot_ip:=192.168.1.10 use_fake_hardware:=false
   ros2 launch burger_kinova_reference moveit_planificacion.launch.py use_fake_hardware:=false
   ss -tanp | grep 192.168.1.10       # la sesión sale desde la IP del cable
   ```
3. **Lleve el brazo a su posición Home** con la Web App de Kinova o con el mando, antes del primer turno y siempre que un grupo lo deje en otra pose. La secuencia empieza por `home`, y el validador limita el recorrido de cada articulación a `max_joint_travel_rad` = 3.0 rad. Desde una pose con la muñeca girada media vuelta, `home` exige ~3.1 rad en `joint_4` y `joint_6` y el modo seco la bloquea:
   ```text
   - joint_4: recorrido acumulado 3.094 rad supera max_joint_travel_rad=3.000
   - joint_6: recorrido acumulado 3.128 rad supera max_joint_travel_rad=3.000
   ```
   Ese bloqueo es correcto: **no amplíe el límite**. Lleve el brazo a Home y repita el modo seco.

**Grupo en turno**, dominio 0, en su portátil:
1. Grabe la evidencia durante todo el turno:
   ```bash
   export ROS_DOMAIN_ID=0
   SHARE=$(ros2 pkg prefix burger_kinova_reference)/share/burger_kinova_reference
   bash $SHARE/scripts/record_kinova_bag.sh lab04_eqNN_real 120
   ```
2. **Modo seco con el robot real** (obligatorio):
   ```bash
   ros2 run burger_kinova_reference moveit_safe_executor --ros-args \
       --params-file $SHARE/config/kinova_connection.yaml \
       -p use_fake_hardware:=false \
       -p goals_file:=$HOME/lab04_eqNN/metas_eqNN.yaml \
       -p report_file:=$HOME/lab04_eqNN/real.jsonl
   ```
   El docente revisa el informe de cada meta en pantalla.
3. **Ejecución supervisada** (con autorización verbal, la parada de emergencia custodiada y el área despejada):
   ```bash
   ros2 run burger_kinova_reference moveit_safe_executor --ros-args \
       --params-file $SHARE/config/kinova_connection.yaml \
       -p use_fake_hardware:=false -p dry_run:=false -p enable_motion:=true \
       -p goals_file:=$HOME/lab04_eqNN/metas_eqNN.yaml \
       -p report_file:=$HOME/lab04_eqNN/real.jsonl
   ```
4. Con cinta métrica, mida la altura de las puntas de la pinza sobre la mesa al final de `descenso_eqNN` y compárela con la esperada. Complete la **Tabla 6** y cierre el turno deteniendo el ejecutor y la grabación.

---

## 8. RESULTADOS DE LA PRÁCTICA

Las tablas se diligencian en el instrumento de entrega, [`INSTRUMENTO_ABET_LAB_04_MOVEIT2_EJECUCION_SEGURA.docx`](../evidencias_abet/INSTRUMENTO_ABET_LAB_04_MOVEIT2_EJECUCION_SEGURA.docx), que se entrega como `C3_L04_G<grupo>_<codigo1>_<codigo2>_v1.docx`. Con él se adjuntan `metas_eqNN.yaml`, los archivos `seco.jsonl`, `fake.jsonl` y `real.jsonl` que escribe el ejecutor (`report_file`) y la salida de `ros2 bag info` de la bolsa de la Fase 6. Aquí se reproducen como referencia para la sesión.

### Tabla 1: Entorno
| Ítem | Valor |
|---|---|
| Grupo (`eqNN`) y `ROS_DOMAIN_ID` de las fases 0 a 5 | |
| Versión de MoveIt (`ros2 pkg xml moveit_ros_move_group \| grep version`) | |
| Resultado de `colcon test` de `burger_kinova_reference` (aprobadas / fallidas) | |
| Commit del repositorio del grupo (SHA) | |

### Tabla 2: Escena y Ejecución Deshabilitada
| Comprobación | Resultado observado | Explicación |
|---|---|---|
| Aviso de `move_group` al pulsar Execute | | |
| Caja `table_link` en `base_link` (centro y tamaño) | | |
| Cara superior de la mesa en `base_link`, escena corregida (z, calculado) | | |
| Cara superior de la mesa en `base_link`, escena con z = 0 (z, calculado) | | |
| Salida y código de `escena_moveit` con la escena z = 0 | | |

### Tabla 3: Secuencia por Defecto (modo seco y driver fake)
| Meta | Tipo | Puntos | Duración (s) | Articulación de mayor recorrido (rad) | Velocidad pico (rad/s) | Desvío final fake (rad) |
|---|---|:---:|:---:|:---:|:---:|:---:|
| `home` | articular | | | | | |
| `sobre_tag_mesa` | pose | | | | | |
| `bajar_5cm` | cartesiana | | | | | |
| `subir_5cm` | cartesiana | | | | | |
| `home` | articular | | | | | |

### Tabla 4: Bloqueos Provocados
| N.º | Regla o capa que actuó | Mensaje exacto | Código de salida | ¿Por qué ocurre? |
|:---:|---|---|:---:|---|
| 4.1 | | | | |
| 4.2 | | | | |
| 4.3 | | | | |
| 4.4 | | | | |
| 4.5 | | | | |
| 4.6 | | | | |
| 4.7 | | | | |
| 4.8 | | | | |

### Tabla 5: Metas Propias (modo seco y fake)
| Meta | Pose predicha en `base_link` (m) | Pose impresa por el ejecutor (m) | Duración (s) | Velocidad pico (rad/s) | Resultado |
|---|---|---|:---:|:---:|---|
| `aproximacion_eqNN` | | | | | |
| `descenso_eqNN` | — | — | | | |

### Tabla 6: Ejecución en el Robot Real
| Meta | Duración planificada (s) | Desvío final (rad) | Error de seguimiento máximo del feedback (rad) | Altura medida de las puntas (m) | Resultado |
|---|:---:|:---:|:---:|:---:|---|
| | | | | | |

---

## 9. ANÁLISIS DE RESULTADOS

1. **Análisis 1 (Capas de protección):** Con la Tabla 4, clasifique cada prueba según la capa que la detuvo: escena de planificación, planificador (MoveIt), validador de trayectoria, habilitación del operador o vigilancia durante la ejecución. ¿Qué riesgo cubre cada capa que las demás no cubren?
2. **Análisis 2 (Geometría compartida):** A partir de la Fase 1 y la prueba 4.8, explique por qué un error de 25 mm en un URDF que «sólo se usaba para visualizar» terminó siendo un problema de seguridad del robot real.
3. **Análisis 3 (Velocidad):** Compare las pruebas 4.1 y 4.2. ¿Por qué el ejecutor revisa tanto la velocidad entre puntos como la declarada por MoveIt, si MoveIt ya aplica `velocity_scaling`?
4. **Análisis 4 (Simulado frente a real):** Compare el desvío final y la duración de la Tabla 3 (fake) con los de la Tabla 6 (real). ¿Qué fuentes de diferencia existen y cuáles pertenecen al enlace de red (Laboratorio 03, `EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md`)?

---

## 10. CONCLUSIONES

1. Conclusión sobre la separación entre planificar y ejecutar, sustentada en las Tablas 2 y 4.
2. Conclusión sobre el papel de `burger_description` como contrato geométrico de la celda.
3. Conclusión cuantitativa sobre la fidelidad de la ejecución real frente al plan (Tabla 6).

---

## 11. PREGUNTAS PARA LA DISCUSIÓN

1. **Pregunta 1:** ¿Por qué el ejecutor pide cada plan desde la pose **medida** y no deja que MoveIt use la que conoce? Describa un escenario del laboratorio donde ambas difieran.
2. **Pregunta 2:** `max_joint_delta_rad` (0.35 rad) no se aplica a los planes de MoveIt. ¿Qué regla lo sustituye y por qué la regla anterior no tiene sentido para una trayectoria de varios puntos?
3. **Pregunta 3:** Una meta de tipo `pose` en `tag_mesa` sigue a la mesa si el URDF cambia. ¿Qué pasaría si el `tag_mesa` físico se mueve 5 cm y el URDF no se actualiza? ¿Qué componente del proyecto podría detectarlo?
4. **Pregunta 4:** En modo seco la secuencia se planifica desde la pose final del plan anterior. ¿Qué garantiza y qué **no** garantiza un modo seco exitoso sobre la ejecución real?
5. **Pregunta 5:** ¿Qué haría falta para que `moveit_safe_executor` validara también la geometría (colisiones) del plan, y por qué no basta con confiar en MoveIt?

---

## 12. BIBLIOGRAFÍA

1. MoveIt 2 Documentation (Jazzy). *Concepts: move_group, planning scene, trajectory processing.* https://moveit.picknik.ai/jazzy/
2. Kinova Robotics. (2024). *ros2_kortex: ROS 2 driver and MoveIt configuration for Kinova Gen3.* https://github.com/Kinovarobotics/ros2_kortex
3. Sucan, I. A., Moll, M., & Kavraki, L. E. (2012). The Open Motion Planning Library. *IEEE Robotics & Automation Magazine, 19*(4), 72–82.
4. Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control.* Cambridge University Press.
5. Proyecto `burger_delivery`. `burger_kinova_reference/README.md` §13, `burger_description/urdf/delivery_scene_fixed.urdf` y `TROUBLESHOOTING.md` §2 y §3.7.

---

## 13. APROBACIÓN DE LA GUÍA DE LABORATORIO

| Elaborado por: | Revisado por: | Aprobado por: |
|:---:|:---:|:---:|
| **Ing. Henry Roncancio**<br>Docente Asignatura ROS | **Director de Programa**<br>Ingeniería Mecatrónica | **Decano(a)**<br>Facultad de Ingeniería |
