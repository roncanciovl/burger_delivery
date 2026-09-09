# Base de correcciones del corte 1 — qué trae y por qué

Esta base **no modifica tu package**. Entra limpia en el repositorio de tu equipo: ninguna
de las rutas que corrige fue tocada por ningún equipo, así que el merge no produce
conflictos.

## Por qué se publica

Durante la validación sobre el Kinova real del laboratorio aparecieron tres problemas de
plataforma que **hacen fallar a cualquier implementación correcta**. Si el proyecto de tu
equipo no funcionó, es muy probable que la causa esté aquí y no en tu código.

### 1. El brazo es de 6 GDL, no de 7

El enunciado declaraba siete grados de libertad. El brazo tiene **seis**, con pinza
Robotiq 2F-85. El driver lo dice en cada arranque:

```text
[KortexMultiInterfaceHardware]: Actuator count reported by robot is '6'
```

Lo traicionero es que **lanzarlo con `dof:=7` no produce ningún error**: el driver expone
una séptima articulación que el robot nunca alimenta, y esa casilla publica memoria sin
inicializar. Se observó `joint_7` valiendo `1.12e+277` rad en una sesión y `0.0` en otra.

Con `0.0` tu monitor reporta `7/7 articulaciones` y telemetría saludable, porque el valor
es finito y cae dentro de todos los límites. **Ninguna validación de rango puede
distinguirlo** de una articulación legítimamente en el origen.

**Qué revisar en tu package:** `dof`, `expected_joints`, `safe_joint_positions_rad`,
`joint_min_rad` y `joint_max_rad` pasan a seis elementos. La articulación de la pinza
(`robotiq_85_left_knuckle_joint`) también aparece en `/joint_states` y debe **ignorarse**
sin invalidar el mensaje.

### 2. Con pinza, el launch no llega a arrancar

```text
error: Invalid parameter "mock_sensor_commands"
  when instantiating macro: robotiq_gripper (/opt/ros/jazzy/share/robotiq_description/...)
```

El `kortex_description` de Kinova envía al macro de la pinza argumentos que el
`robotiq_description` que instala apt en Jazzy no acepta. El xacro no se genera, así que
**ningún nodo arranca**. No es un fallo de tu launch. Corrección en
[`ros2_setup/INSTALACION_KORTEX.md`](ros2_setup/INSTALACION_KORTEX.md) §3.4.

### 3. La estación del driver debe ir por cable

Medido sobre el robot real, con la misma máquina y cambiando sólo el enlace:

| | WiFi | Cable |
|---|---:|---:|
| Frecuencia media de `/joint_states` | 72.24 Hz | **99.96 Hz** |
| Intervalo p99 | 60.12 ms | **10.61 ms** |
| Intervalo máximo | 3251 ms | **20.63 ms** |
| Pérdidas de telemetría en 120 s | **2** | **0** |

La mediana era idéntica en las dos: por WiFi el ciclo se cumplía la mitad del tiempo. Lo
que rompe una sesión de control cíclica no es el promedio, es la cola. Las estaciones
**cliente** sí pueden ir por WiFi.

## Qué contiene esta base

| Archivo | Cambio |
| :--- | :--- |
| `education/proyectos_evaluables/PROYECTO_CORTE_1_CONEXION_KINOVA.md` | Plataforma corregida a 6 GDL |
| `ros2_setup/INSTALACION_KORTEX.md` | Ajustes obligatorios sobre `ros2_kortex` recién clonado |
| `TROUBLESHOOTING.md` | §2 hardware ocupado · §3 fallos de plataforma ajenos a tu código |
| `network_setup/ANOMALIAS_HARDWARE.md` | §3 la articulación fabricada |
| `network_setup/monitor_red/` | Distintivo de estación anfitriona en la lista de dispositivos |
| `education/guias_laboratorio/GUIA_LAB_02_*` | Direccionamiento IP real del laboratorio |

## Cómo sincronizarla

```bash
git fetch upstream --tags
git switch -c sync/base-2026-2-corte1-fix origin/main
git merge --no-ff refs/tags/base-2026-2-corte1-fix
git push -u origin sync/base-2026-2-corte1-fix
```

Se integra a `main` mediante pull request, como cualquier otro cambio.

## Antes de dar por malo tu código

1. `ros2 topic echo /joint_states --once` — ¿seis articulaciones más la de la pinza, o
   siete `joint_N`? Si son siete, era el problema 1.
2. ¿El launch arrancaba con pinza? Si no, era el problema 2.
3. ¿La estación del driver estaba por cable?
4. ¿Había otra estación con el driver abierto? Ver `TROUBLESHOOTING.md` §2.

Sólo si los cuatro salen limpios tiene sentido revisar la lógica de tu package.

---

La implementación de referencia del docente se publica por separado, en el tag
`referencia-corte1`, bajo el nombre `burger_kinova_reference` para que no colisione con
el `burger_kinova_connection` de tu equipo.
