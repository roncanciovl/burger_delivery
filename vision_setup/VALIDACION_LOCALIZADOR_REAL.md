# Validación del localizador AprilTag con la cámara real del Kinova

**Estado:** pendiente de ejecutar en el laboratorio (TODO.md §4.1). El localizador ya se validó
con una escena sintética en perspectiva (error ≈ 3 mm y 0.55°); falta la cámara real con tags
físicos, que el 2026-09-15 no estaban instalados.

**Qué se entrega al terminar:** `vision_setup/tags_fisicos.yaml` con IDs y lados medidos, la
imagen anotada de la pose de observación y `validacion_apriltag/poses.csv` con la tabla de la
sección 4, más la fila correspondiente en la tabla de resultados de abajo.

## 1. Material

| Qué | Cómo |
| :--- | :--- |
| Tag de la mesa (36h11) | `python3 scripts/generar_tags_apriltag.py --ids 1 --lado-mm 100 --salida tags/` |
| Tags de los carritos | `python3 scripts/generar_tags_apriltag.py --ids 5 6 --lado-mm 60 --salida tags/` |
| Impresión | Al 100 %, sin "ajustar a la página", en papel mate. Medir la regla de 50 mm impresa: si no mide 50.0 ± 0.3 mm, la impresora escaló y hay que repetir |
| Medición | Calibrador para el lado del cuadro negro; cinta métrica para las posiciones |

Pega el `tag_mesa` plano y sin burbujas: una ondulación de 1 mm en el tag de referencia se
convierte en error de escala de toda la medición.

## 2. Registro de los tags

1. Mide con calibrador el lado del cuadro negro de cada tag **ya pegado** y anótalo en
   `vision_setup/tags_fisicos.yaml` (parámetros y tabla de inventario).
2. Con la anfitriona publicando la cámara y el brazo en la pose de observación:

   ```bash
   python3 scripts/validar_localizador_apriltag.py imagen --salida validacion_apriltag/
   ```

   Debe listar el `tag_mesa` y el del carrito con estado `OK` (≥ 30 px de lado). Si un tag
   sale `PEQUEÑO`, acerca la pose de observación o usa tags más grandes. Una asimetría
   mayor al 20 % indica una cámara muy inclinada: la homografía la corrige, pero el ruido crece.
3. Guarda la pose articular de observación (`ros2 topic echo --once /joint_states`) en la
   tabla de resultados: sin ella la prueba no es repetible.

## 3. Arranque del localizador

```bash
python3 scripts/apriltag_fixed_camera_localizer.py --ros-args \
    --params-file vision_setup/tags_fisicos.yaml
ros2 topic hz /burger_car_01/pose2d     # debe igualar la frecuencia de la imagen comprimida
```

## 4. Exactitud y repetibilidad

Marca con cinta tres posiciones del **centro del tag del carrito** respecto al centro del
`tag_mesa`, en sus ejes. Para cada una, con el tag **sobre la mesa** (sin paralaje):

```bash
python3 scripts/validar_localizador_apriltag.py pose --esperado 0.30 0.15 0 \
    --etiqueta P1_mesa --muestras 100 --csv validacion_apriltag/poses.csv
```

Repite P1 con el tag sobre el techo del carrito (`--etiqueta P1_techo`): la diferencia entre
`P1_mesa` y `P1_techo` es el error de paralaje por la altura del carrito.

| Etiqueta | Esperado (x, y, θ) | Qué mide |
| :--- | :--- | :--- |
| `P1_mesa` | (0.30, 0.15, 0°) | Exactitud cerca del tag de referencia |
| `P2_mesa` | (0.60, -0.20, 90°) | Exactitud lejos del tag de referencia |
| `P3_mesa` | (0.15, 0.40, -45°) | Orientación no alineada |
| `P1_techo` | (0.30, 0.15, 0°) | Paralaje por la altura del carrito |

## 5. Criterios de aceptación

| Métrica | Umbral propuesto | Motivo |
| :--- | ---: | :--- |
| Error de posición en `P*_mesa` | ≤ 10 mm | 3 mm en sintético; se admite el ruido de la cámara real |
| Error de orientación en `P*_mesa` | ≤ 2° | 0.55° en sintético |
| Desviación de x e y | ≤ 3 mm | Repetibilidad; más indica desenfoque o iluminación |
| Frecuencia de la pose | = frecuencia de la imagen comprimida | El nodo publica una pose por imagen |
| Poses recibidas | 100 de 100 | Si faltan, algún tag deja de verse |

Los umbrales son una propuesta para la primera corrida; ajústalos con los datos.

## 6. Resultados

| Fecha | Tags (mesa / carrito, lado) | Pose de observación | P1 | P2 | P3 | Paralaje P1 | ¿Pasa? |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| | | | | | | | |
