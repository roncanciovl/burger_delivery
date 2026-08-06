# Tipos de joints en ROS 2 (URDF/Xacro)

Este documento resume los tipos de juntas (joints) disponibles en URDF para ROS 2, su propósito, implicaciones en TF, control con `ros2_control` y ejemplos mínimos. Puedes visualizar ejemplos en `ROS/joints_demo.urdf` (compatible con el visor web; notar limitaciones indicadas más abajo).

## Visión general
- `fixed`: sin movimiento; une rígidamente dos links.
- `revolute`: rotación con límites alrededor de un eje.
- `continuous`: rotación sin límites (360°) alrededor de un eje.
- `prismatic`: traslación lineal con límites a lo largo de un eje.
- `planar`: 3 DoF (x, y, yaw) en un plano perpendicular al eje dado.
- `floating`: 6 DoF (x, y, z, roll, pitch, yaw).

Notas de compatibilidad (visores): algunos visores web (incluido `urdf-loaders`) no renderizan `planar` ni `floating`. Para demostraciones visuales usa `fixed` en su lugar, manteniendo la misma pose inicial. En RViz/ROS 2 funcionan correctamente.

---

## fixed
- Uso: soportes, estructuras rígidas, frames de referencia.
- TF: se publica una única transform estática entre padre e hijo.
- Control: no aplica.

Ejemplo URDF:
```xml
<joint name="base_to_sensor" type="fixed">
  <parent link="base_link"/>
  <child  link="sensor_link"/>
  <origin xyz="0.10 0.00 0.20" rpy="0 0 0"/>
  <!-- sin <axis> ni <limit> -->
  <!-- sin interfaces de control -->
</joint>
```

---

## revolute
- Uso: bisagras con tope; articulaciones de brazos con límites mecánicos.
- TF: un DoF rotacional (θ). En ROS 2, el estado proviene de `/joint_states`.
- Control: típicamente interfaces `position`, `velocity`, `effort` + `<limit>`.

Ejemplo URDF:
```xml
<joint name="shoulder" type="revolute">
  <parent link="link_a"/>
  <child  link="link_b"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis   xyz="0 0 1"/>
  <limit  lower="-1.5708" upper="1.5708" effort="30" velocity="1.5"/>
</joint>
```

---

## continuous
- Uso: uniones sin tope (ruedas, torretas, juntas 360°).
- TF: un DoF rotacional periódico (θ ≡ θ ± 2π).
- Control: similar a `revolute`, pero sin `<limit lower/upper>`.

Ejemplo URDF:
```xml
<joint name="wheel_yaw" type="continuous">
  <parent link="chassis"/>
  <child  link="wheel_base"/>
  <origin xyz="0.2 0 0.05" rpy="0 0 0"/>
  <axis   xyz="0 0 1"/>
</joint>
```

---

## prismatic
- Uso: guías lineales, actuadores lineales.
- TF: un DoF de traslación (d) a lo largo del eje.
- Control: `position`/`velocity`/`effort` con `<limit lower/upper>` en metros.

Ejemplo URDF:
```xml
<joint name="linear_slide" type="prismatic">
  <parent link="base"/>
  <child  link="carriage"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis   xyz="1 0 0"/>
  <limit  lower="0.0" upper="0.50" effort="200" velocity="0.2"/>
</joint>
```

---

## planar
- Uso: plataformas móviles en un plano (x, y, yaw), mesas XY.
- TF: 3 DoF (x, y, yaw) definidos por el eje (típicamente `0 0 1`).
- Control: no estándar con `ros2_control`; comúnmente se maneja a nivel cinemática/odometría.
- Visor web: usualmente no soportado; usa `fixed` para visualizar.

Ejemplo URDF (para RViz/ROS):
```xml
<joint name="map_to_base" type="planar">
  <parent link="map"/>
  <child  link="robot_base"/>
  <origin xyz="1.2 0.8 0.0" rpy="0 0 0"/>
  <axis   xyz="0 0 1"/>
</joint>
```

---

## floating
- Uso: pose libre 6 DoF (robots voladores, objetos manipulados).
- TF: 6 DoF (x, y, z, roll, pitch, yaw) determinados por el estado del joint.
- Control: no hay interfaz genérica directa; se actualiza por estimación/odometría.
- Visor web: usualmente no soportado; usa `fixed` para visualizar.

Ejemplo URDF (para RViz/ROS):
```xml
<joint name="world_to_object" type="floating">
  <parent link="world"/>
  <child  link="object"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

---

## mimic (relación entre joints)
- Uso: acoplar el movimiento de un joint “esclavo” a otro “maestro”. Muy útil en garras.
- Sintaxis: `<mimic joint="master" multiplier="k" offset="b"/>` con `q_slave = k*q_master + b`.

Ejemplo URDF (dedos de garra):
```xml
<joint name="left_inner_knuckle_joint" type="continuous">
  <parent link="base"/>
  <child  link="left_inner_knuckle"/>
  <mimic joint="left_knuckle_joint"/>
</joint>
```

---

## Consideraciones de TF y `ros2_control`
- Cada joint define una transform en TF entre `parent` y `child`; para joints móviles, su valor proviene de `/joint_states` (`sensor_msgs/JointState`).
- Límites: define `<limit lower upper effort velocity>` y, si corresponde, fricción/retroceso (`<dynamics>`). Para `continuous`, omite `lower/upper`.
- Control: en ROS 2 se configuran interfaces en el controlador (`ros2_control`) y el hardware publica/consume estados y comandos.
- Inerciales y colisión: aunque aquí nos centramos en joints, incluye `<inertial>`, `<collision>` y `<visual>` en links para simulaciones y planificación.

## Ejemplos en este repo
- `ROS/joints_demo.urdf`: escena pequeña con ejemplos de cada tipo. Para compatibilidad de visores web, `planar` y `floating` se muestran como `fixed` (manteniendo la pose). En ROS/RViz puedes cambiarlos a sus tipos reales.
- `ROS/visual/burger_delivery_gen3.urdf`: integra joints reales del Kinova Gen3 + gripper.

## Depuración rápida
- Si “no se ve nada” en un visor: valida que no haya links duplicados y que todas las rutas de malla sean relativas y correctas.
- En RViz: agrega `TF` y `RobotModel`, revisa `/tf` y `/tf_static`, y verifica que el `Fixed Frame` sea `map` (o el apropiado).

