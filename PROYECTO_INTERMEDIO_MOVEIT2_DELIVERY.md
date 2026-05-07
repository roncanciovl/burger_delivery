# Proyecto Intermedio: MoveIt 2, Localización y Entrega en Carrito

## 1. Objetivo

Implementar una prueba intermedia donde el Kinova Gen3 ubica un carrito mediante un nodo de localización, solicita al carrito ubicarse en una pose conveniente y luego ejecuta un pick and place de una caja de hamburguesa usando MoveIt 2 o MoveIt Task Constructor.

El proyecto evalúa integración robótica, no solo visualización:

- URDF y geometría de colisión,
- `robot_description`, TF y frames dinámicos,
- nodo de localización,
- servicio ROS 2 del carrito,
- coordinación entre carrito y manipulador,
- Planning Scene,
- pipeline de pick and place,
- validación y diagnóstico.

Estrategia de manipulación de referencia:

![Pipeline Pick and Place](pick_and_place_pipeline.svg)

## 2. Flujo esperado

1. El sistema carga la escena fija, el Kinova y los URDFs separados de los carritos.
2. Un nodo de localización publica la pose dinámica del carrito:

```text
tag_mesa -> tag_carrito1
```

3. El sistema calcula una pose conveniente para recibir la caja.
4. El coordinador llama un servicio del carrito para pedirle que se ubique allí.
5. Cuando el carrito confirma llegada, se valida su pose usando TF.
6. MoveIt 2 planifica el pick de la caja desde la mesa.
7. MoveIt 2 transporta la caja evitando colisiones.
8. El destino de place se calcula desde:

```text
car1_delivery_tray_frame
```

9. El Kinova deposita la caja, hace detach y se retira.

## 3. Arquitectura mínima

| Responsabilidad | Componente | Interfaz principal |
|---|---|---|
| Escena fija | `delivery_scene_fixed.urdf` | `robot_description`, `/tf_static` |
| Modelo del carrito | `car1_apriltag.urdf` | `/car1/robot_description` |
| Localización | `cart_localization_node` | publica `tag_mesa -> tag_carrito1` |
| Selección de pose | `delivery_pose_selector` | produce `PoseStamped` objetivo |
| Servicio del carrito | `/car1/prepare_delivery_pose` | request/response |
| Manipulación | `kinova_delivery_task_node` | MoveIt 2 / MTC |

Principio de diseño:

```text
El URDF describe estructura rígida.
TF describe poses en tiempo de ejecución.
El servicio coordina intención con el carrito.
MoveIt 2 planifica solo cuando TF y la Planning Scene son coherentes.
```

## 4. Frames obligatorios

La persona participante debe poder generar `view_frames` y justificar estas cadenas:

```text
map -> table_link -> tag_mesa -> tag_carrito1 -> car1_base_link -> car1_delivery_tray_frame
map -> table_link -> world -> gen3_base_link -> ... -> burger_grip_frame
```

Restricción crítica:

El place no debe usar una coordenada fija escrita a mano. Debe depender del frame del carrito:

```text
car1_delivery_tray_frame
```

## 5. Servicio del carrito

Servicio sugerido:

```text
/car1/prepare_delivery_pose
```

Interfaz conceptual:

```text
Request:
  target_pose: geometry_msgs/PoseStamped
  tray_frame: string
  tolerance_xy: float64
  tolerance_yaw: float64
  timeout_sec: float64

Response:
  accepted: bool
  reached: bool
  message: string
  final_pose: geometry_msgs/PoseStamped
```

El servicio puede ser simulado al inicio. En ese caso debe quedar claro que simula la llegada del carrito, no la localización real ni la navegación real.

## 6. Pipeline de MoveIt 2

El pipeline debe seguir las etapas de `pick_and_place_pipeline.svg`:

| Etapa | Acción | Fundamento |
|---|---|---|
| 0 | Estado inicial | TF, Planning Scene, joints |
| 1 | aproximación a caja | IK y pregrasp |
| 2 | cerrar gripper + attach | objeto adherido |
| 3 | levantar | movimiento cartesiano seguro |
| 4 | conectar hacia carrito | planificación anti-colisión |
| 5 | descender al destino | pose dependiente de TF |
| 6 | abrir gripper + detach | objeto vuelve al mundo |
| 7 | retirada | salida segura |

La caja debe existir como objeto de colisión antes del pick. Después del attach, debe tratarse como `AttachedCollisionObject`. Después del detach, debe quedar en la Planning Scene sobre la bandeja.

## 7. Entregables

La persona participante debe entregar:

1. Diagrama de nodos, tópicos, servicios y TF.
2. `view_frames` con el carrito conectado por localización.
3. Servicio `/car1/prepare_delivery_pose` definido y probado.
4. Nodo de localización simulado o real para `tag_mesa -> tag_carrito1`.
5. Nodo coordinador que llame el servicio antes del pick and place.
6. Pipeline MoveIt 2 o MTC con aproximación, attach, lift, connect, place, detach y retreat.
7. Evidencia en RViz de carrito, caja, Planning Scene y trayectoria.
8. Registro de una prueba exitosa y un fallo diagnosticado.

## 8. Pruebas mínimas

| Prueba | Evidencia |
|---|---|
| URDF válido | `check_urdf burger_description/urdf/delivery_scene_fixed.urdf` |
| TF del carrito | `view_frames` muestra `tag_mesa -> tag_carrito1 -> car1_base_link` |
| Servicio activo | `ros2 service list` muestra `/car1/prepare_delivery_pose` |
| Servicio funcional | una llamada responde `accepted/reached` o falla con razón clara |
| Place dependiente de TF | mover el carrito cambia el destino de place |
| Planning Scene | mesa, carrito y caja se consideran como colisiones |
| Attach/Detach | la caja cambia de mundo a gripper y vuelve al mundo |
| Seguridad | la trayectoria no atraviesa mesa ni carrito |

## 9. Evaluación

Puntaje sugerido: 100 puntos.

| Área | Puntaje | Evidencia |
|---|---:|---|
| Arquitectura ROS 2 | 10 | responsabilidades, nodos, servicios y tópicos claros |
| URDF, TF y frames | 15 | distingue escena fija, carrito móvil y TF dinámico |
| Localización del carrito | 10 | publica y valida `tag_mesa -> tag_carrito1` sin conflictos |
| Servicio del carrito | 10 | define interfaz, tolerancias, timeout y fallos |
| Coordinación del flujo | 10 | localiza, pide pose, verifica y manipula en orden correcto |
| MoveIt 2 / MTC | 15 | ejecuta pipeline pick and place con attach/detach |
| Planning Scene y colisiones | 10 | caja, mesa y carrito afectan la planificación |
| Validación y diagnóstico | 10 | usa RViz, CLI, logs y TF para justificar resultados |
| Documentación técnica | 5 | explica decisiones y límites |
| Problema desconocido | 5 | transfiere fundamentos a una situación nueva |

## 10. Evaluación adicional: problema desconocido

La evaluación adicional no debe preguntar por una etapa ya implementada exactamente como en el proyecto. Debe presentar un problema nuevo, no ensayado, que obligue a transferir los mismos fundamentos.

La persona participante debe explicar cómo lo resolvería, qué verificaría primero, qué comandos usaría y en qué capa corregiría el problema.

Formato de respuesta esperado:

```text
Problema:
Hipótesis inicial:
Capa probable: URDF / TF / localización / servicio / MoveIt / RViz / hardware
Evidencia a recolectar:
Comandos o herramientas:
Corrección propuesta:
Cómo se valida:
Qué no se debe compensar en otro nivel:
```

Ejemplos de problemas desconocidos:

| Problema nuevo | Qué se evalúa |
|---|---|
| El sistema debe entregar en `car2` en vez de `car1` sin duplicar lógica | generalización de frames, servicios y parámetros |
| El carrito queda visible en RViz pero MoveIt lo atraviesa | diferencia entre visual y collision / Planning Scene |
| El detector ve el tag, pero el carrito aparece desplazado 8 cm | calibración, offset tag-carrito y `tag_mesa` |
| El servicio responde `reached=true`, pero TF no cambió | separación entre respuesta de servicio y evidencia espacial |
| El place funciona con carrito quieto, pero falla cuando el carrito cambia de pose | uso correcto de TF dinámico y timestamp |
| La caja se mueve con el gripper en RViz pero MoveIt no evita chocarla | estado attached mal actualizado |
| Al activar RViz aparece error de `robot_description_semantic` | plugin MoveIt cargado sin SRDF o stack incompleto |
| Dos nodos publican `tag_carrito1` | conflicto de autoridad TF |

Rúbrica del problema desconocido:

| Criterio | Puntaje | Evidencia |
|---|---:|---|
| Identifica la capa correcta | 1 | no confunde síntomas con causa |
| Propone validación concreta | 1 | usa comandos, RViz o logs verificables |
| Corrige en el nivel correcto | 1 | no compensa en URDF si el fallo es TF, ni al revés |
| Explica impacto robótico | 1 | relaciona el fallo con seguridad, planificación o localización |
| Generaliza la solución | 1 | la respuesta sirve para casos similares |

## 11. Criterio de aprobación global

El proyecto se considera aprobado si la persona participante puede demostrar el flujo principal y además resolver conceptualmente un problema desconocido usando razonamiento por capas.

La evidencia más fuerte de aprendizaje es poder afirmar:

```text
El problema está en esta capa del sistema.
Lo verifico con esta herramienta.
Lo corrijo en este nodo, archivo o configuración.
No lo compenso en otro nivel porque produciría un error sistemático.
```
