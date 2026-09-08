# Validación del corte 1 — `burger_kinova_connection`

Registro de las pruebas de aceptación PA-01 a PA-10.

| Campo | Valor |
|---|---|
| Fecha de esta ejecución | 2026-09-08 |
| Entorno | Ubuntu 24.04 sobre WSL2 · ROS 2 Jazzy |
| Middleware | `rmw_cyclonedds_cpp` · `ROS_DOMAIN_ID=0` · `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` (por defecto) |
| Hardware | **Simulado** (`use_fake_hardware:=true`) con el driver oficial `kortex_bringup` |
| Estaciones | Una sola estación (A y B en el mismo equipo) |
| Robot físico | **No disponible en esta sesión** |

> Las filas marcadas **PENDIENTE (robot real)** requieren el Kinova Gen3 conectado.
> Sigue la *Lista de verificación para el Kinova real* (sección 11 del README) y anota
> aquí los resultados con fecha, comandos y salidas.

---

## Preparación previa (obligatoria en WSL)

```bash
timeout 5s ros2 daemon stop
pgrep -af '_ros2_daemon'        # si quedó vivo: kill <PID>
ros2 daemon start
timeout 5s ros2 daemon status   # -> The daemon is running

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/ros2_ws/src/burger_delivery/network_setup/cyclonedds.xml"
```

---

## PA-01 · Compilación limpia — ✅

```bash
cd ~/ros2_ws
colcon build --packages-select burger_kinova_connection --symlink-install
colcon test --packages-select burger_kinova_connection
colcon test-result --test-result-base build/burger_kinova_connection --verbose
```

```text
Summary: 79 tests, 0 errors, 0 failures, 0 skipped
```

Incluye `ament_flake8`, `ament_pep257` y `ament_copyright` además de las pruebas
unitarias de validación de mensajes, timeouts, límites de meta, caja negra, subsistema
de logging y resolución de la pinza en el launch.

---

## PA-02 · Grafo en modo fake — ✅

```bash
ros2 launch burger_kinova_connection kinova_connection.launch.py \
  start_driver:=true robot_ip:=0.0.0.0 use_fake_hardware:=true enable_motion:=false
```

```text
[launch.user] ⚠ gripper:=robotiq_2f_85 se omite en modo fake. […] Con hardware real la
              pinza SÍ se transfiere.
[spawner_joint_state_broadcaster]      Configured and activated joint_state_broadcaster
[spawner_joint_trajectory_controller]  Configured and activated joint_trajectory_controller
[kinova_monitor] [TRANSICIÓN] INICIO -> ERROR | no se ha recibido ningún /joint_states válido
[kinova_monitor] [CONTROLADORES] joint_state_broadcaster=active, joint_trajectory_controller=active
[kinova_monitor] [TRANSICIÓN] ERROR -> OK | telemetría saludable: 100.0 Hz, edad 0.003 s,
                 7/7 articulaciones | todos los controladores requeridos están activos
```

Grafo observado:

```text
/controller_manager  /joint_state_broadcaster  /joint_trajectory_controller
/kortexmultiinterfacehardware  /robot_state_publisher  /kinova_monitor
```

> **Nota registrada:** el spawner de `twist_controller --inactive` termina con código 1.
> Es un comportamiento del propio `kortex_bringup`, ajeno a este package, y no impide que
> el broadcaster ni el controlador de trayectoria queden activos.

---

## PA-03 · Telemetría — ✅ en simulación · **PENDIENTE (robot real)**

```bash
ros2 topic hz /joint_states
```

```text
average rate: 99.976   min: 0.009s  max: 0.011s  std dev: 0.00035s  window: 102
average rate: 99.998   min: 0.009s  max: 0.011s  std dev: 0.00030s  window: 203
```

Siete articulaciones `joint_1`…`joint_7`, sin interrupciones, muy por encima del mínimo
configurado (`min_joint_state_hz: 20.0`). Falta repetir la medición durante 60 s
continuos sobre el robot físico.

---

## PA-04 · Controladores — ✅

```bash
ros2 control list_controllers
```

```text
joint_trajectory_controller  joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
```

El nodo lo confirma por su cuenta mediante `/controller_manager/list_controllers`, y el
diagnóstico publica el estado individual de cada controlador.

---

## PA-05 · Pérdida de enlace — ✅

Procedimiento: monitor con `start_driver:=false` en un proceso, `kortex_bringup` en otro;
se termina el proceso del driver.

```text
[ERROR] [SEGURIDAD] Movimiento bloqueado por: pérdida de telemetría articular. Tras
        restablecer el enlace, rehabilita de forma explícita con:
        ros2 service call /kinova_monitor/rehabilitar_movimiento std_srvs/srv/Trigger
[ERROR] [TRANSICIÓN] OK -> ERROR | telemetría vencida: 1.05 s sin mensaje válido
        (límite 1.00 s) | todos los controladores requeridos están activos
```

El monitor **no se cerró**: siguió publicando diagnóstico durante toda la caída.

---

## PA-06 · Recuperación — ✅

Se relanza el driver, sin reiniciar el monitor:

```text
[INFO] [RECUPERACIÓN] /joint_states volvió a fluir tras 1 interrupción(es).
       El movimiento sigue bloqueado hasta una nueva habilitación explícita.
[WARN] [TRANSICIÓN] ERROR -> WARN | muestras insuficientes para estimar la frecuencia
       (20 muestras en 0.19 s; se requieren al menos 0.50 s de observación continua)
[INFO] [TRANSICIÓN] WARN -> OK | telemetría saludable: 100.0 Hz, edad 0.003 s,
       7/7 articulaciones | todos los controladores requeridos están activos
```

Rehabilitación explícita del movimiento (requisito de seguridad §11.8):

```bash
ros2 service call /kinova_monitor/rehabilitar_movimiento std_srvs/srv/Trigger
```

```text
success=True, message='movimiento rehabilitado (bloqueo previo: pérdida de telemetría articular)'
```

El paso intermedio por `WARN` es deliberado: tras una interrupción la ventana de
frecuencia se reinicia y el enlace no se declara saludable hasta reobservarlo.

---

## PA-07 · Validación de meta — ✅ (los tres casos bloqueados)

Todos en modo seco: **ninguno contactó el servidor de acción**.

**a) Movimiento deshabilitado**

```bash
ros2 run burger_kinova_connection safe_trajectory_client --ros-args \
  --params-file <config> -p dry_run:=true
```
```text
Telemetría disponible: 7 articulaciones a 100.0 Hz.
Estado: META BLOQUEADA antes de contactar el servidor de acción:
  ✗ movimiento deshabilitado (enable_motion=false). En modo fake habilítalo de forma
    explícita para completar la prueba
```

**b) Límite excedido** — `-p "safe_joint_positions_rad:=[9.0,0.0,0.0,0.0,0.0,0.0,0.0]"`

```text
[CONFIGURACIÓN INSEGURA] la pose aprobada para joint_1 (9.0000 rad) queda fuera de
                         [-3.1400, 3.1400]
```

**c) Pose incompleta** — `-p "safe_joint_positions_rad:=[0.0,0.0,0.0]"`

```text
[CONFIGURACIÓN INSEGURA] safe_joint_positions_rad tiene 3 elementos y se esperaban 7
                         (uno por articulación de expected_joints)
```

Códigos de salida: `1` (meta bloqueada por seguridad) en los tres casos.

---

## PA-08 · Movimiento autorizado — ✅ en simulación · **PENDIENTE (robot real)**

```bash
ros2 run burger_kinova_connection safe_trajectory_client --ros-args \
  --params-file <config> -p dry_run:=false -p enable_motion:=true \
  -p "safe_joint_positions_rad:=[0.05,0.05,0.0,0.0,0.0,0.0,0.0]"
```

```text
safe_trajectory_client iniciado | dry_run=False | enable_motion=True
  Estado: VALIDACIÓN SUPERADA — la meta puede enviarse.
[ENVÍO] Enviando meta de 1 punto(s) a /joint_trajectory_controller/follow_joint_trajectory
        con duración 5.00 s.
[RESULTADO] Meta ACEPTADA por el controlador.
[FEEDBACK] t=0.040 s | error máximo por articulación=0.00000 rad
[FEEDBACK] t=2.040 s | error máximo por articulación=0.00000 rad
[FEEDBACK] t=4.090 s | error máximo por articulación=0.00000 rad
[RESULTADO] Trayectoria completada con éxito | error_code=SUCCESSFUL
```

Verificación independiente de que el brazo alcanzó la meta:

```bash
ros2 topic echo /joint_states --once
# position: [0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
```

Código de salida `0`. El feedback aparece limitado en frecuencia (`log_throttle_period_s`),
tal como exige la regla de throttling del taller de logging.

> Sobre el robot real esta prueba requiere espacio despejado, parada de emergencia
> accesible y autorización del responsable del laboratorio.

---

## PA-09 · DDS distribuido — **PENDIENTE (dos estaciones)**

Validado parcialmente: monitor y driver como **procesos independientes** descubriéndose
por DDS en la misma máquina (ver PA-05/PA-06). Falta ejecutarlo con la estación A
conectada al robot y la estación B en otro computador de la misma subred, con
`ROS_DOMAIN_ID` acordado y sin cambiar el rango de descubrimiento.

---

## PA-10 · Reproducibilidad — ✅ parcial

El README permite compilar, lanzar en modo fake y repetir PA-01, PA-02, PA-04, PA-05,
PA-06, PA-07 y PA-08 desde un workspace limpio. Queda pendiente que otra persona lo
repita de forma independiente sobre el robot real.

---

## Incidencias encontradas durante la validación

| # | Incidencia | Capa | Estado |
|---|---|---|---|
| 1 | `ros2_control_node` aborta con `Activated mimic joints cannot have command interfaces` al usar la pinza 2F-85 en modo fake | `ros2_kortex` vs ROS 2 Jazzy | **Evitado** en el launch: la pinza se omite en modo fake y se avisa. Con hardware real el mismo xacro no declara `command_interface` sobre las `mimic`, así que la pinza sí se transfiere |
| 2 | `ros2 node/topic/param` terminan en `TimeoutError` | Daemon de la CLI en WSL | Procedimiento en [`TROUBLESHOOTING.md`](../../TROUBLESHOOTING.md); se reinicia el daemon antes de cada sesión |
| 3 | `-p dry_run:=false` se ignoraba en silencio | Precedencia de parámetros de ROS 2 | **Corregido**: los parámetros sobrescribibles desde la CLI se movieron a la sección `/**` del YAML |
| 4 | La frecuencia se estimaba en miles de Hz al suscribirse | Ráfaga de mensajes encolados por DDS | **Corregido**: `min_rate_observation_s` exige observación continua antes de creer la estimación |
| 5 | Spawner de `twist_controller --inactive` termina con código 1 | `kortex_bringup` | Ajeno a este package; no impide activar broadcaster ni controlador de trayectoria |

---

## Evidencia grabada

```bash
./scripts/record_kinova_bag.sh dataset_validacion_corte1 60
ros2 bag info dataset_validacion_corte1
```

Graba `/joint_states`, `/burger/kinova/diagnostics`, `/rosout`, `/tf` y `/tf_static` en
MCAP con compresión Zstd. Incluir `/rosout` es lo que deja la caída de la telemetría y la
línea `ERROR` que la explica en el mismo archivo, con marcas de tiempo correlacionadas.
