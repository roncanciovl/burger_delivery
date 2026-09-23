# Branches con los pendientes de TODO.md (2026-09-23)

Cada pendiente abierto de `TODO.md` tiene su propio branch, listo para probar con el robot
físico. El detalle de cada uno (qué cambia, cómo probarlo, qué cuenta como éxito) está en el
documento compartido «Burger Delivery — Branches de pendientes del TODO».

La numeración sigue el orden de prueba sugerido: de lo que no necesita robot a lo que
mueve el brazo y los carritos.

| # | Branch | Pendiente del TODO | Base | Necesita robot |
| --- | --- | --- | --- | --- |
| 1 | `claude/todo-ci-urdf-lint` | §5 CI: validación de URDF y linteo | `main` | No |
| 2 | `claude/todo-flight-recorder-6dof` | §4.1 emulador con 7 articulaciones | `main` | No |
| 3 | `claude/todo-paper-borrador` | §1 borrador de paper | `main` | No |
| 4 | `claude/todo-guias-lab` | §5 guías de laboratorio 04–07 | `main` | No |
| 5 | `claude/todo-display-launch-namespace` | §4.1 `display.launch.py` sin namespace | `main` | Opcional |
| 6 | `claude/todo-burger-description-6dof` | §4.1 re-vendorizar a 6 GDL | `main` | Para comparar |
| 7 | `claude/todo-apriltag-validacion-real` | §4.1 validar localizador con cámara real | `main` | Sí (cámara) |
| 8 | `claude/todo-wsl2-linux-nativo` | §4.1 residuo de WSL2 | `main` | Sí (quieto) |
| 9 | `claude/todo-modularizacion-paquetes` | §2 separación en paquetes | `main` | Para MTC |
| 10 | `claude/todo-gemini-vlm-node` | §1 nodo de percepción VLM | branch 9 | Sí (cámara) |
| 11 | `claude/todo-benchmark-gemini-apriltag` | §1 benchmark comparativo | branch 10 | Sí (cámara) |
| 12 | `claude/todo-netem-estres-red` | §4 inyección de tráfico y estrés | `main` | Sí |
| 13 | `claude/todo-nav2-delivery` | §3 Nav2, slots y acción de entrega | `main` | Sí (carritos) |

Ninguno se ha probado con el robot todavía. Los 13 se fusionan sin conflictos respetando la
cadena 9 → 10 → 11; sobre el árbol combinado pasan 113 pruebas unitarias, `ruff`,
`shellcheck` y la validación de URDF. `burger_control` (branch 9) no se pudo compilar sin
MoveIt: lo compilará el CI al abrir el PR.

La entrega del carrito (branch 13) queda como acción, no como servicio (decidido el
2026-09-23).

`TODO.md` no se modificó en ningún branch: marcar cada casilla en el PR cuya prueba salga bien.
