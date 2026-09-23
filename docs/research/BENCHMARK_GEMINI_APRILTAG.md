# Benchmark: localización 3D de Gemini frente a AprilTag

**Estado:** instrumental listo, pendiente de ejecutar en el laboratorio (TODO.md §1,
«Módulo de Benchmarking Comparativo»).

**Pregunta.** ¿Con qué error y a qué costo (latencia, ancho de banda) localiza Gemini
Robotics-ER la caja en 3D, comparado con un AprilTag, que sirve de referencia?

## Montaje

- Un AprilTag 36h11 de lado conocido (p. ej. 50 mm, ID 7) pegado al **centro de la cara
  superior** de la caja. Imprimirlo con `scripts/generar_tags_apriltag.py` y medir el lado.
- Cámara del Kinova publicando color y profundidad **registrada**:
  `ros2 launch kinova_vision kinova_vision.launch.py device:=192.168.1.10 depth_registration:=true`.
- `GEMINI_API_KEY` exportada en la terminal del benchmark.

Así la referencia (PnP sobre el tag) y Gemini (píxel señalado + profundidad) miden el mismo
punto físico en la misma imagen y en el mismo marco óptico.

> El tag está a la vista del modelo y podría ayudarle a señalar. Para una medición más
> exigente, repetir con un tag del mismo tamaño en la cara lateral visible y corregir el
> desplazamiento conocido; queda como variante.

## Ejecución

```bash
colcon build --packages-select burger_perception && source install/setup.bash
# Repetibilidad y latencia: misma escena, 10 ensayos
ros2 run burger_perception benchmark_gemini_apriltag --tag-id 7 --tag-size 0.05 \
    --ensayos 10 --salida bench_repetibilidad
# Exactitud: se mueve la caja entre ensayos (posiciones y orientaciones distintas)
ros2 run burger_perception benchmark_gemini_apriltag --tag-id 7 --tag-size 0.05 \
    --ensayos 15 --interactivo --salida bench_exactitud
```

Cada carpeta contiene `ensayos.csv` (una fila por ensayo), `resumen.json`,
`ensayo_NN.jpg` (círculo verde: tag; cruz roja: Gemini) y `figura_benchmark.png`.

## Métricas

| Métrica | Dónde | Definición |
| :--- | :--- | :--- |
| Error 3D | `error_3d_mm` | Distancia euclidiana entre el centro del tag (PnP) y el punto de Gemini, en mm |
| Error en imagen | `error_px` | Distancia entre el píxel señalado y la proyección del centro del tag |
| Latencia | `latencia_ms` | Tiempo de `generate_content`, de la petición a la respuesta |
| Carga subida | `jpeg_bytes`, `subida_gemini_Bps` | Tamaño de cada imagen enviada (reducida a 1024 px de lado) y media por segundo |
| Carga de la cámara | `camara_comprimida_Bps` | Bytes/s del tópico comprimido que recibe la estación, para comparar |
| Tasa de éxito | `exito_tag`, `exito_gemini` | Ensayos con referencia y con punto válido |

`resumen.json` trae media, desviación, mediana, p95, RMSE y máximo de cada serie.

## Separar las fuentes de error

`error_px` alto con `error_3d_mm` alto: el modelo señala mal. `error_px` bajo con
`error_3d_mm` alto: el problema es la profundidad (bordes, reflejos, registro). Para
comprobar lo segundo, repetir con `--depth-mode plane --distancia-plano <z medida>`.

## Resultados

| Fecha | Escena | n | Error 3D media / p95 (mm) | Error px media | Latencia media (ms) | KB por imagen |
| :--- | :--- | ---: | :--- | ---: | ---: | ---: |
| | | | | | | |
