# GUÍA DE LABORATORIO 05: LOCALIZACIÓN VISUAL CON APRILTAGS Y CANCELACIÓN DE PERSPECTIVA

| FACULTAD | PROGRAMA | ASIGNATURA | SEMESTRE | CÓDIGO GUÍA | REVISIÓN |
|:---|:---|:---|:---:|:---:|:---:|
| Facultad de Ingeniería | Ingeniería Mecatrónica | ROBOT OPERATING SYSTEM - ROS | VIII – IX | GL-AA-F-1 / LAB-05 | 0.1 (borrador 2026-2) |

> [!NOTE]
> **Estado:** borrador, sin ejecutar con estudiantes. Requiere la cámara del Kinova publicando
> (Guía 02) y tags 36h11 instalados en la mesa. Usa las herramientas del branch
> `claude/todo-apriltag-validacion-real` (`generar_tags_apriltag.py`,
> `validar_localizador_apriltag.py`, `vision_setup/tags_fisicos.yaml`).

---

## 1. CONTROL DE CAMBIOS

| Descripción del Cambio | Justificación | Fecha |
|---|---|:---:|
| Creación de la guía (borrador) | Pendiente de `TODO.md` §5 («Guía Lab 03: Localización visual con AprilTags y cancelación de perspectiva»), renumerada a 05 | 23/09/2026 |

---

## 2. INTRODUCCIÓN

La cámara del Kinova va en la muñeca: se mueve con el brazo. Si el carrito se localizara
respecto a la cámara, cada vibración del brazo lo haría «temblar» en el mapa. La celda lo evita
midiendo el carrito **respecto al `tag_mesa`**: las cuatro esquinas de un tag de tamaño conocido
definen una homografía imagen → plano de la mesa que cancela la perspectiva, esté donde esté la
cámara (`TALLER_LOCALIZACION_APRILTAG_KINOVA_MICROROS.md` §3).

El taller explica la teoría y el código; esta guía **mide**: exactitud, repetibilidad, efecto
del paralaje y sensibilidad a un error en el tamaño declarado del tag.

## 3. OBJETIVOS

### 3.1. Objetivo General
Caracterizar experimentalmente el localizador AprilTag de la celda con la cámara real del Kinova.

### 3.2. Objetivos Específicos
1. Imprimir y registrar tags a tamaño controlado.
2. Medir exactitud y repetibilidad en tres posiciones conocidas.
3. Cuantificar el error de paralaje por la altura del tag del carrito.
4. Demostrar el efecto de declarar mal `tag_size_m`.

## 4. DESCRIPCIÓN DE LA PRÁCTICA

| Fase | Contenido | Duración |
|---|---|:---:|
| 1 | Tags a tamaño exacto y registro | 20 min |
| 2 | Pose de observación y diagnóstico de imagen | 20 min |
| 3 | Exactitud y repetibilidad | 40 min |
| 4 | Paralaje y error de escala | 30 min |

### 4.1. RAE y ponderación

| Criterio | Indicador | SO | Peso |
|---|---|:---:|:---:|
| C1. Registro de tags y verificación de escala | 6.4 Verificación experimental | SO6 | 20% |
| C2. Exactitud y repetibilidad con estadística | 6.4 Análisis de datos experimentales | SO6 | 35% |
| C3. Paralaje y escala: modelo frente a medida | 1.1 Modelado matemático | SO1 | 30% |
| C4. Informe y trazabilidad (CSV, imágenes) | 3.1 Documentación técnica | SO3 | 15% |

## 5. MATERIALES Y EQUIPOS

| DESCRIPCIÓN | CANTIDAD |
|---|:---:|
| Kinova Gen3 con módulo de visión y la estación anfitriona publicando la cámara | 1 |
| Tags 36h11 impresos: uno para la mesa (100 mm), uno para el carrito (60 mm) | 2 |
| Calibrador (0.1 mm) y cinta métrica (1 mm) | 1 c/u |
| Carrito de la celda | 1 |

## 6. SEGURIDAD EN EL LABORATORIO

Sólo la estación anfitriona lanza el driver (`TROUBLESHOOTING.md` §2.0). El brazo se lleva a
la pose de observación **una vez**, desde la anfitriona, con la parada de emergencia a mano;
durante las mediciones el brazo permanece quieto. Las estaciones de los grupos sólo se
suscriben a la imagen comprimida: cada suscriptor remoto cuesta ≈ 61 Mbps de WiFi
(`TROUBLESHOOTING.md` §4.8); **un** suscriptor por grupo.

## 7. PROCEDIMIENTO EXPERIMENTAL

### Fase 1: Tags a tamaño exacto

```bash
python3 scripts/generar_tags_apriltag.py --ids 1 --lado-mm 100 --salida tags/
python3 scripts/generar_tags_apriltag.py --ids 5 --lado-mm 60 --salida tags/
```

Imprime al 100 %. Mide con calibrador la regla de 50 mm y el lado del cuadro negro de cada tag
pegado. Registra los valores medidos en `vision_setup/tags_fisicos.yaml`.

**Pregunta:** si la impresora escaló al 97 %, ¿qué error produce en una posición a 0.40 m del
`tag_mesa`? Justifica con la homografía.

### Fase 2: Diagnóstico de imagen

```bash
python3 scripts/validar_localizador_apriltag.py imagen --salida validacion_eqNN/
```

Cada tag debe aparecer con estado `OK` (≥ 30 px de lado). Anota la asimetría: indica cuánto
está inclinada la cámara respecto a la mesa.

### Fase 3: Exactitud y repetibilidad

1. Lanza el localizador: `python3 scripts/apriltag_fixed_camera_localizer.py --ros-args --params-file vision_setup/tags_fisicos.yaml`.
2. Marca con cinta tres posiciones del centro del tag del carrito (sobre la mesa, sin el carrito)
   respecto al centro del `tag_mesa` y en sus ejes.
3. En cada posición:

   ```bash
   python3 scripts/validar_localizador_apriltag.py pose --esperado 0.30 0.15 0 \
       --etiqueta P1_mesa --muestras 100 --csv validacion_eqNN/poses.csv
   ```

4. **Tabla 1:** esperado, media, desviación y error de x, y, θ por posición.

### Fase 4: Paralaje y error de escala

1. **Paralaje.** Repite P1 con el tag sobre el techo del carrito (`--etiqueta P1_techo`). La
   homografía supone que el tag medido está en el plano de la mesa: un tag a altura h aparece
   desplazado hacia afuera del punto principal. Estima ese desplazamiento con un modelo de
   cámara pinhole (distancia cámara–mesa medida con cinta) y compáralo con la diferencia
   `P1_techo − P1_mesa`.
2. **Escala.** Relanza el localizador declarando `tag_size_m` 10 % mayor y repite P2. Verifica
   que la posición escala por el mismo factor y explica por qué θ no cambia.

## 8. ENTREGABLES

- `tags_fisicos.yaml` con IDs y lados medidos.
- `poses.csv` y las imágenes anotadas de la Fase 2.
- Tablas y el análisis de paralaje y escala (modelo frente a medida).

## 9. REFERENCIAS

- `education/talleres/TALLER_LOCALIZACION_APRILTAG_KINOVA_MICROROS.md` §2–§3.
- `vision_setup/VALIDACION_LOCALIZADOR_REAL.md` (protocolo y criterios de aceptación).
- E. Olson, «AprilTag: A robust and flexible visual fiducial system», IEEE ICRA, 2011.
