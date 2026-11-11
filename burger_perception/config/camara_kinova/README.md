# Calibración del módulo de visión del Kinova Gen3 (laboratorio)

Estos archivos sustituyen la calibración de fábrica de la cámara de color que trae `kinova_vision`,
que no corresponde a la cámara del laboratorio. Los usa
`ros2 launch burger_perception camera.launch.py`. Para volver a los valores de fábrica, por
ejemplo para comparar, se lanza con `calibracion:=kinova`.

| Archivo | Qué contiene |
| :--- | :--- |
| `color_1920x1080.yaml` | Intrínsecos y distorsión de la cámara de color en 1920 × 1080 (medidos) |
| `extrinsecos_profundidad_color.yaml` | Posición del sensor de profundidad respecto del de color (los de fábrica, validados) |

La profundidad (480 × 270) conserva los intrínsecos de fábrica (`fx` = 360), que son correctos.

## Qué estaba mal en la calibración de fábrica

| | Fábrica | Laboratorio |
| :--- | :--- | :--- |
| Color `fx`, `fy` | 2612, 2609 | 2531, 2531 |
| Color `cx`, `cy` | 946, **97** | 961, **200** |

Con `cy` = 97, la profundidad registrada cae unos 140 px por encima del color en 1080p (unos
4 cm a 0.75 m). La Kortex API del robot (`leer_calibracion_kortex.py`) devuelve para el color los
mismos valores de fábrica, así que leerlos del robot no lo corrige. Además devuelve unos
extrínsecos de (−27.1, −10.0, −4.7) mm y una `fx` de profundidad de 338.7; ninguno de los dos
reproduce las medidas de abajo.

## Cómo se midió (2026-11-10 y 2026-11-11)

1. **Color.** 68 tomas de un tablero de 9 × 7 cuadros de 25 mm (8 × 6 esquinas), a 0.5-1.35 m,
   con inclinaciones de 12° a 74°, procesadas con OpenCV `calibrateCamera`. Se eligió el modelo
   con distorsión radial y tangencial: el tablero solo no distingue entre modelos (error de
   reproyección 0.35 px en todos), pero este es el que mejor alinea la caja de validación.
2. **Validación independiente** con `verify_rgbd_alignment` y una caja de 240 × 303 mm, en
   capturas que no se usaron en el ajuste. Distancia entre los extrínsecos del sistema y los que
   mejor explican la imagen:

| Captura | Distancia cámara-caja | Fábrica | Color nuevo + extrínsecos de fábrica |
| :--- | :--- | :--- | :--- |
| 2026-11-10 13:39 | 0.73 m | 26 mm | 4-6 mm |
| 2026-11-10 13:55 | 0.74 m | 39 mm | 11-17 mm |
| 2026-11-11 15:27, 15:32, 15:39 (en vivo, misma escena) | 1.03-1.04 m | 42-46 mm | 9-12, 5-12 y 33-36 mm |

   **La componente vertical quedó resuelta**: es la que fallaba con la calibración de fábrica, y en
   las tres capturas en vivo su residuo se mantuvo entre −7 y −9 mm. **La horizontal no se puede
   afirmar por debajo de unos ±15 mm con esta caja a 1 m**: sin que nada se moviera (las imágenes
   de las tres capturas coinciden a 0.3 px), la mejor solución horizontal saltó entre −13 y +15 mm.
   El contorno de la tapa que da la profundidad sale unos 45 px más angosto que la tapa real y calza
   casi igual de bien pegado a cualquiera de sus lados, con ayuda de los bordes impresos en la caja.

   Con la `fx` de profundidad de fábrica (360), la tapa de la caja midió 238-245 × 302-304 mm. Con
   la del robot (338.7), 249-259 × 320-324 mm.

## Mediciones que no se adoptaron

- **Extrínsecos por planos.** Con 17 tomas del tablero con color y profundidad sincronizadas,
  comparando el plano visto por cada cámara, salen (−34.0, −11.3, +4.4) mm y un giro de unos 1.8°
  (discrepancia entre planos: 4.3 mm, frente a 15 mm con los del robot). La validación con la caja
  no los distingue de los de fábrica (5-18 mm frente a 4-17 mm, dentro del ruido del método), así
  que se conservaron los de fábrica por ser los documentados. Archivo: `~/alineacion_rgbd/extrinsecos_planos_*.json` de la
  estación donde se midió.
- **Modelo de color sólo radial** (`cy` = 152): 12-34 mm en la misma validación.

## Límites

- **La desalineación horizontal restante no está medida con precisión**: entre 0.4 y 3.6 cm según
  la captura, por el límite del método descrito arriba. Alcanza para localizar una caja de este
  tamaño con Gemini, no para objetos pequeños. Para medirla mejor hace falta un objeto de bordes
  nítidos y sin dibujo, a 0.6-0.8 m. `verify_rgbd_alignment` responde `DESALINEADA` con su
  tolerancia por defecto de 3 mm.
- **La imagen no se rectifica.** La distorsión del color mueve los píxeles hasta unos 10 px en el
  borde inferior; el registro de profundidad y el nodo Gemini la ignoran.
- **El color tiene enfoque automático**, y la distancia focal cambia algo con el enfoque. La
  calibración se hizo entre 0.5 y 1.35 m.
- **Sólo vale para 1920 × 1080.** Con otra resolución, `kinova_vision` avisa que la calibración no
  coincide con el stream.
- **Luz:** sin sol directo ni sombras duras (TROUBLESHOOTING.md §3.9).

Para repetirla se necesita el tablero impreso al 100 % sobre una base rígida y los scripts de
captura y ajuste, que todavía no están en el repositorio (ver TODO.md).
