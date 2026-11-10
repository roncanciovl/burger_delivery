# Branches con los pendientes de TODO.md (actualizado el 2026-11-02)

Estos branches se crearon el 2026-09-23 para los pendientes abiertos de `TODO.md`. El detalle
original de cada uno (qué cambia, cómo probarlo, qué cuenta como éxito) está en el documento
compartido «Burger Delivery — Branches de pendientes del TODO».

## Branches vigentes

El orden sigue la prueba sugerida: de lo que no necesita robot a lo que mueve el brazo y los
carritos.

| Branch | Pendiente del TODO | Base | Necesita robot |
| --- | --- | --- | --- |
| `claude/todo-paper-borrador` | §1 borrador de paper | `main` | No |
| `claude/todo-guias-lab` | §5 guías de laboratorio 04–07 | `main` | No |
| `claude/todo-apriltag-validacion-real` | §4.1 validar localizador con cámara real | `main` | Sí (cámara) |
| `claude/todo-modularizacion-paquetes` | §2 separación en paquetes | `main` | Para MTC |
| `claude/todo-gemini-vlm-node` | §1 nodo de percepción VLM | `claude/todo-modularizacion-paquetes` | Sí (cámara) |
| `claude/todo-benchmark-gemini-apriltag` | §1 benchmark comparativo | `claude/todo-gemini-vlm-node` | Sí (cámara) |
| `claude/todo-netem-estres-red` | §4 inyección de tráfico y estrés | `main` | Sí |
| `claude/todo-nav2-delivery` | §3 Nav2, slots y acción de entrega | `main` | Sí (carritos) |

Los ocho van 55 commits por detrás de `main`. La prueba de integración del 2026-09-23 (fusión
sin conflictos y 113 pruebas unitarias sobre el árbol combinado) se hizo sobre el `main` de
entonces y no se ha repetido. `burger_control` (de `claude/todo-modularizacion-paquetes`) no se
pudo compilar sin MoveIt en esa prueba.

**Lab 07:** su actualización sigue pendiente. El borrador está en `claude/todo-guias-lab`
(`GUIA_LAB_07_RAZONAMIENTO_ESPACIAL_GEMINI.md`). Lo que puede aportar al Lab IA 01 es el benchmark
contra AprilTag, que necesita `claude/todo-benchmark-gemini-apriltag` y sus dos branches base, y la
regla de no pasar la clave de API como parámetro ROS. Conserva estos branches hasta decidirlo.

La entrega del carrito (`claude/todo-nav2-delivery`) queda como acción, no como servicio (decidido
el 2026-09-23).

## Branches retirados

| Branch | Último commit | Motivo | Pendiente en `TODO.md` |
| --- | --- | --- | --- |
| `claude/todo-burger-description-6dof` | ya en `main` | Fusionado | Cerrado (2026-10-14) |
| `claude/todo-flight-recorder-6dof` | `7432284` | Superado: `main` ya publica 6 articulaciones | — |
| `claude/todo-display-launch-namespace` | `bdb3616` | Descartado (2026-11-02): `display.launch.py` cambió mucho desde entonces | Abierto: namespace en `display.launch.py` |
| `claude/todo-wsl2-linux-nativo` | `0a3ff68` | Descartado (2026-11-02) | Abierto: residuo de WSL2 |
| `claude/todo-ci-urdf-lint` | `14c0352` | Descartado (2026-11-02) | Abierto: validación de URDF y linteo en CI |

Los tres pendientes abiertos ya no tienen branch: si se retoman, se rehacen sobre el `main` actual.

`TODO.md` no se modificó en ningún branch: marca cada casilla en el PR cuya prueba salga bien.
