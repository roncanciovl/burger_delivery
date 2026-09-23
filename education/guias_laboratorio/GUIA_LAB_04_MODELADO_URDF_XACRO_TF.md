# GUÍA DE LABORATORIO 04: MODELADO DE ROBOTS Y ÁRBOLES TF CON URDF/XACRO

| FACULTAD | PROGRAMA | ASIGNATURA | SEMESTRE | CÓDIGO GUÍA | REVISIÓN |
|:---|:---|:---|:---:|:---:|:---:|
| Facultad de Ingeniería | Ingeniería Mecatrónica | ROBOT OPERATING SYSTEM - ROS | VIII – IX | GL-AA-F-1 / LAB-04 | 0.1 (borrador 2026-2) |

> [!NOTE]
> **Numeración.** `TODO.md` §5 la llamaba «Guía Lab 02»; los números 01 a 03 ya están
> ocupados por red DDS, cámara y operación distribuida, así que las guías nuevas van de la
> 04 a la 07. **Estado:** borrador, sin ejecutar con estudiantes. Aplicar el método de
> `TROUBLESHOOTING.md` §4.7 (ejecutar cada bloque y comprobar el efecto) antes de publicarla.

---

## 1. CONTROL DE CAMBIOS

| Descripción del Cambio | Justificación | Fecha |
|---|---|:---:|
| Creación de la guía (borrador) | Pendiente de `TODO.md` §5: práctica de modelado con URDF/XACRO y TF sobre la escena real de Burger-Cell, complementaria al taller `TALLER_URDF_TF.md` | 23/09/2026 |

---

## 2. INTRODUCCIÓN

El taller `TALLER_URDF_TF.md` enseña a **leer** la escena de `burger_description`. Esta práctica
pide **construir**: parametrizar un carrito con XACRO, verificar su árbol con herramientas de
línea de comandos y medir en el laboratorio las transformaciones que el URDF dice tener.

Tres ideas que se evalúan:

1. **URDF describe; XACRO genera.** Un `.xacro` con propiedades y macros evita copiar a mano
   `car1_apriltag.urdf` y `car2_apriltag.urdf`, que hoy difieren sólo en el prefijo.
2. **El árbol TF es un contrato.** `tag_mesa -> tag_carritoN -> carN_base_link` lo comparten el
   equipo del carrito y el del Kinova (`PROYECTO_INTERMEDIO_MOVEIT2_DELIVERY.md` §2.1). Cambiar un
   nombre de frame rompe al otro equipo sin error de compilación.
3. **Un modelo se valida contra el mundo.** Las cotas del URDF se contrastan con cinta métrica;
   la diferencia se reporta, no se esconde.

> [!IMPORTANT]
> El modelo vendorizado del Gen3 en `burger_description` es de 7 GDL en `main` y de 6 GDL tras
> fusionar `claude/todo-burger-description-6dof`. Verifica cuál tienes con
> `rg -c 'gen3_joint_7' burger_description/urdf/delivery_scene_fixed.urdf` antes de la Fase 3.

## 3. OBJETIVOS

### 3.1. Objetivo General
Modelar un carrito diferencial de la celda con XACRO, integrarlo al árbol TF de Burger-Cell y
validar el modelo con herramientas de ROS 2 y mediciones físicas.

### 3.2. Objetivos Específicos
1. Convertir `car1_apriltag.urdf` en una macro XACRO parametrizada por prefijo y altura del tag.
2. Generar el URDF con `xacro` y validarlo con `check_urdf` y `scripts/ci/validar_urdf.py`.
3. Publicar el modelo con `robot_state_publisher` y documentar el árbol con `view_frames`.
4. Medir con `tf2_echo` y con cinta métrica tres transformaciones de la escena y reportar el error.

## 4. DESCRIPCIÓN DE LA PRÁCTICA

| Fase | Contenido | Duración |
|---|---|:---:|
| 1 | Anatomía de un URDF: links, joints, `origin`, `axis`, `limit` | 20 min |
| 2 | Del URDF al XACRO: propiedades, macros y argumentos | 40 min |
| 3 | Validación y árbol TF | 30 min |
| 4 | Medición física contra el modelo | 30 min |

### 4.1. RAE y ponderación

| Criterio | Indicador | SO | Peso |
|---|---|:---:|:---:|
| C1. XACRO parametrizado y correcto | 1.1 Modela sistemas robóticos con representaciones estándar | SO1 | 30% |
| C2. Validación automática (sin errores) | 6.4 Verifica experimentalmente modelos y configuraciones | SO6 | 25% |
| C3. Árbol TF documentado y coherente con el contrato | 3.1 Documentación técnica | SO3 | 20% |
| C4. Medición física y análisis del error | 6.4 Diagnóstico experimental | SO6 | 25% |

## 5. MATERIALES Y EQUIPOS

| DESCRIPCIÓN | CANTIDAD |
|---|:---:|
| Portátil con Ubuntu 24.04, ROS 2 Jazzy, `xacro`, `liburdfdom-tools`, `ros-jazzy-tf2-tools` | 1 por grupo |
| Carrito de la celda con su tag, y la mesa con el `tag_mesa` | 1 |
| Cinta métrica o flexómetro (resolución 1 mm) | 1 |

## 6. SEGURIDAD EN EL LABORATORIO

Esta práctica no usa el driver del robot. Trabaja en el **dominio de tu grupo** (`export
ROS_DOMAIN_ID=<11, 12, …>`) para no mezclar tu `/tf` y tu `/joint_states` con el robot ni con
otros grupos (`TROUBLESHOOTING.md` §4.1). Si no puedes cambiar de dominio, usa
`display.launch.py namespace:=visor_eqNN` (disponible con el branch
`claude/todo-display-launch-namespace`).

## 7. PROCEDIMIENTO EXPERIMENTAL

### Fase 1: Anatomía de un URDF

```bash
cd ~/ros2_ws/src/burger_delivery
rg -n '<link name|<joint name|<origin|<parent|<child' burger_description/urdf/car1_apriltag.urdf
```

Completa la **Tabla 1** con cada joint: tipo, padre, hijo y `origin`. Responde: ¿por qué
`tag_carrito1 -> car1_base_link` tiene `z = -0.12`? ¿Qué ocurre con el árbol si el tag no está
en el techo sino en el frente del carrito?

### Fase 2: Del URDF al XACRO

1. Crea `burger_description/urdf/carrito.urdf.xacro` con:
   - `<xacro:property name="altura_tag" value="0.12"/>`;
   - una macro `<xacro:macro name="carrito" params="prefijo numero altura_tag">` que genere
     `tag_carrito${numero}`, `${prefijo}_base_link`, ruedas y rueda castor;
   - un archivo por carrito que sólo invoque la macro.
2. Genera y compara con el original:

   ```bash
   xacro burger_description/urdf/carrito1.urdf.xacro > /tmp/car1_generado.urdf
   diff <(check_urdf burger_description/urdf/car1_apriltag.urdf) <(check_urdf /tmp/car1_generado.urdf)
   ```

   **Criterio:** `check_urdf` debe listar los mismos links y la misma jerarquía. Explica cada
   diferencia que quede.

### Fase 3: Validación y árbol TF

```bash
python3 scripts/ci/validar_urdf.py /tmp/car1_generado.urdf      # branch claude/todo-ci-urdf-lint
ros2 run robot_state_publisher robot_state_publisher --ros-args \
    -p robot_description:="$(cat /tmp/car1_generado.urdf)"
ros2 run joint_state_publisher joint_state_publisher --ros-args \
    -p robot_description:="$(cat /tmp/car1_generado.urdf)"
ros2 run tf2_tools view_frames
```

Adjunta el PDF de `view_frames`. **Comprueba el efecto, no el código de salida:** que existan
los frames `tag_carrito1` y `car1_base_link` y que ninguna malla quede sin resolver.

### Fase 4: Medición física contra el modelo

1. Lanza la escena: `ros2 launch burger_description display.launch.py use_static_carts:=true`.
2. Con `ros2 run tf2_ros tf2_echo <padre> <hijo>` anota tres transformaciones:
   `table_link -> tag_mesa`, `tag_carrito1 -> car1_base_link` y `map -> delivery_slot_1`.
3. Mide las mismas distancias en la celda real con cinta métrica.
4. **Tabla 2:** valor del modelo, valor medido, diferencia (mm). Si alguna supera 10 mm, propone
   el cambio al URDF, pero **no lo apliques sin acordarlo con el equipo del Kinova**: es el
   contrato compartido.

## 8. ENTREGABLES

- `carrito.urdf.xacro` y los archivos por carrito.
- Salida de `check_urdf` y de `validar_urdf.py` sin errores.
- PDF de `view_frames`.
- Tablas 1 y 2 con el análisis del error de modelado.

## 9. REFERENCIAS

- `education/talleres/TALLER_URDF_TF.md` (taller de lectura de la escena).
- ROS 2 Jazzy: *URDF* y *Using Xacro to clean up your code* (docs.ros.org).
- `TROUBLESHOOTING.md` §4.1 (dominios) y §4.5 (mallas que sólo existen en un `install/` antiguo).
