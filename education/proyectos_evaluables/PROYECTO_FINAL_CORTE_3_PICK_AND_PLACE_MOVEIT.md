# Proyecto Final del Tercer Corte — Pick and Place de la Caja de Hamburguesa con MoveIt 2, la Pinza y la Visión del Kinova Gen3

> **Documento Oficial de Guía Técnica y Evaluación Sumativa — Corte 3 (30% de la Nota del Corte).**
> **Asignatura:** ROBOT OPERATING SYSTEM - ROS (Periodo 2026-2)
> **Código de Actividad:** `E_C3` | **Componente Univex:** Entrega Final del Tercer Corte (30%)
> **Modalidad:** Cada grupo implementa la misión completa; el robot real se usa por turnos. Anexo A de comprobación individual obligatorio.
> **Puntuación Oficial:** 500 puntos Zubatronic / SGDE ($N_3 = \text{Score} / 100 \in [0.0, 5.0]$).
> **Instrumento Asociado:** [`education/evidencias_abet/INSTRUMENTO_ABET_PROYECTO_FINAL_CORTE_3_PICK_AND_PLACE.docx`](../evidencias_abet/INSTRUMENTO_ABET_PROYECTO_FINAL_CORTE_3_PICK_AND_PLACE.docx).

> [!IMPORTANT]
> **Actualización del Repositorio Privado del Equipo:**
> Antes de iniciar o continuar con este proyecto, asegúrese de haber sincronizado su repositorio privado de equipo con los últimos cambios de la base del curso. Consulte la [Guía Oficial de Sincronización y Actualizaciones](ACTUALIZACIONES_BASE_CORTE_1.md) para realizar este proceso correctamente.

---

## 1. La Misión

El Kinova Gen3 debe **entregar una caja de hamburguesa en el carrito**, sin que nadie le diga dónde está el carrito.

**Todo ocurre sobre la mesa.** La caja y el carrito están sobre el tablero, en el mismo plano que el `tag_mesa`. El brazo está montado sobre la mesa y no alcanza el piso, 0.92 m más abajo. Por eso la pose del carrito se expresa **en el marco de la mesa** (el del `tag_mesa`), y esa corrección de la perspectiva de la cámara al marco de la mesa se hace con la **imagen RGB-D** (color y profundidad) y **tf2** (sección 6.3).

La misión tiene cinco pasos:

1. **Observar.** Desde una pose de observación, la cámara de la muñeca ve el `tag_mesa` (el AprilTag fijo de la mesa) y el AprilTag del techo del carrito (el «QR» del carrito).
2. **Localizar.** Con esas detecciones el sistema calcula dónde está la bandeja del carrito respecto a la base del robot, y comprueba que el `tag_mesa` está donde dice el modelo de la celda.
3. **Recoger.** El brazo va a la **zona de recogida**, una zona fija de la mesa definida en el marco del `tag_mesa`, abre la pinza, baja, agarra la caja y la levanta.
4. **Entregar.** Lleva la caja sobre la bandeja del carrito, baja, **abre la pinza** para soltarla y se retira.
5. **Volver** a `home`.

El carrito lo ubica el docente en una posición distinta en cada ensayo, dentro de la zona de entrega (sección 4). **No hay coordenadas del carrito escritas en el código**: si se mueve el carrito, la misión se recalcula sola. Esa es la prueba del reto en vivo (sección 9).

---

## 2. Qué Entrega el Curso y Qué Construye el Grupo

Todo lo de la columna izquierda ya existe en el repositorio y se usó en los laboratorios. El proyecto consiste en integrarlo y completarlo con lo de la columna derecha.

| Ya existe en `burger_delivery` | Lo construye el grupo |
|---|---|
| Driver del Kinova y MoveIt en modo sólo planificación: `kinova_connection.launch.py` y `moveit_planificacion.launch.py` ([Laboratorio 04](../guias_laboratorio/GUIA_LAB_04_MOVEIT2_EJECUCION_SEGURA.md)) | La **pose de observación** desde la que la cámara ve los dos tags |
| `moveit_safe_executor`: metas articulares, de pose, cartesianas y de pinza, con modo seco, confirmación y validación de cada trayectoria | Un **nodo de misión** que calcula las metas a partir de las TF y las ejecuta en orden a través de `moveit_safe_executor` |
| Escena de colisión de `burger_description` (mesa con sus medidas reales y `tag_mesa`) cargada por `escena_moveit` | La **caja** como objeto de colisión en la escena de MoveIt, unida a la pinza al agarrarla y separada al soltarla |
| Driver `kinova_vision` con `depth_registration:=true`: imagen de color y **profundidad registrada al marco de color** (transformada del marco del sensor de profundidad al de la cámara de color), con sus `camera_info`. El modelo pinhole para pasar de un píxel con profundidad a un punto 3D está en el [Laboratorio IA 01](../guias_laboratorio/GUIA_LAB_IA_01_EMBODIED_AI_GEMINI_ER2_KINOVA.md) §2.4 | El **nodo de percepción RGB-D con tf2**: pose 3D de cada tag a partir de sus esquinas en la imagen y de la profundidad, publicada como TF desde la cámara, y la transformación al marco de la mesa calculada con tf2 (sección 6.3) |
| | La **verificación del `tag_mesa`**: su pose medida con la cámara frente a la del URDF, también con tf2 (sección 6.2) |
| URDF del carrito: `tag_carrito1 → car1_base_link → car1_delivery_tray_frame` (`car1_apriltag.urdf`) | El **launch de la misión**, el registro de las pruebas y el `README.md` del paquete |
| Cadena de la cámara de muñeca y su eslabón con el brazo ([`TALLER_URDF_TF.md`](../talleres/TALLER_URDF_TF.md) §11) | |

El grupo trabaja en un paquete ROS 2 propio dentro de su repositorio privado, `burger_mision_gNN` (`NN` = número de grupo), con al menos el nodo de misión, la gestión de la caja en la escena y un launch.

---

## 3. Árbol de Transformaciones de la Misión

```text
base_link (= world de la escena)
 ├── … ── end_effector_link ── camera_link ── camera_depth_frame        (driver kinova_vision: sensor de profundidad)
 │                                         └── camera_color_frame        (driver + eslabón del Taller URDF §11)
 │                                                 ├── tag_mesa_detectado       (VISIÓN RGB-D, nodo del grupo)
 │                                                 └── tag_carrito1_detectado   (VISIÓN RGB-D, nodo del grupo)
 └── table_link ── tag_mesa                                              (URDF, medido en la mesa real)
                      ├── zona_recogida      (fija: la define el grupo en el marco del tag_mesa)
                      └── tag_carrito1       (nodo del grupo, con tf2: tag_mesa_detectado → tag_carrito1_detectado)
                            └── car1_base_link ── car1_delivery_tray_frame   (URDF del carrito)
```

- La **meta de agarre** se expresa en el marco del `tag_mesa` (zona de recogida).
- La **meta de entrega** se calcula en cada ensayo leyendo con tf2 la pose de `car1_delivery_tray_frame` en `base_link`.
- El carrito se mide **respecto al `tag_mesa`**, no respecto a la cámara: tf2 compone las dos detecciones y la pose de la cámara se cancela, así que la pose del carrito no depende de dónde esté el brazo al observar. Por eso es crítico que el `tag_mesa` del URDF coincida con el sticker real: un error ahí se traslada entero a la entrega. La verificación de la sección 6.2 lo comprueba antes de mover el brazo.

---

## 4. Celda, Caja y Zonas de Trabajo

| Elemento | Especificación | Por qué |
|---|---|---|
| **Caja de hamburguesa** | Caja de prueba entregada por el docente, con un lado de agarre de **máximo 75 mm** y menos de 300 g | La Robotiq 2F-85 abre 85 mm: una caja más ancha no cabe entre los dedos |
| **Zona de recogida** | Rectángulo marcado con cinta, con centro en $(0.00,\ +0.25)$ m en el marco del `tag_mesa` (el grupo mide y documenta el valor real) | Fija y conocida: el agarre no depende de la visión |
| **Zona de entrega** | El docente ubica el carrito con el centro de su tag dentro de $x \in [0.10,\ 0.35]$ m e $y \in [-0.30,\ -0.18]$ m en el marco del `tag_mesa`, con su lado largo (0.25 m) a menos de 30° del eje $x$ del `tag_mesa` | Dentro del alcance del brazo y del campo de visión; el carrito (0.25 × 0.20 m) queda al menos 5 cm dentro del borde de la mesa ($y = -0.45$ m en `base_link`) y sin tapar el `tag_mesa` |
| **Carrito** | Detenido y con los motores apagados durante la misión; AprilTag 36h11 en el techo | El carrito es el destino, no participa del movimiento |
| **Alturas** | La caja se suelta con su base a no más de 3 cm de la bandeja | Una caída mayor puede volcarla fuera de la bandeja |

El grupo mide con cinta y calibrador las dimensiones de la caja, el lado del cuadro negro de cada tag, la altura del tag del carrito sobre la mesa (`tag_height_m`) y la de la bandeja, y las registra en la Tabla 1 del instrumento.

---

## 5. Secuencia de Estados de la Misión

El nodo de misión ejecuta esta secuencia. **Cada movimiento pasa por `moveit_safe_executor`**: MoveIt planifica y el ejecutor valida, pide confirmación y ejecuta. Ningún nodo del grupo envía trayectorias directamente al controlador.

| N.º | Estado | Tipo de meta | Condición para avanzar |
|:---:|---|---|---|
| 0 | `home` | articular | El brazo está en `home` |
| 1 | Pose de observación | articular | La cámara ve los dos tags |
| 2 | Percepción | — (sin movimiento) | `tag_mesa` verificado (error menor que 2 cm) y TF del carrito estable (20 lecturas con desviación menor que 5 mm) |
| 3 | Abrir pinza | pinza, 0.0 | La pinza reporta posición cercana a 0 |
| 4 | Pre-agarre | pose, 0.15 m sobre la caja, pinza hacia abajo | Llegó sin bloqueos del validador |
| 5 | Descenso | cartesiana, hasta la altura de agarre | — |
| 6 | Cerrar pinza | pinza, 0.8 con esfuerzo limitado | La posición final es **menor** que la de cierre completo: hay caja entre los dedos. Si cierra completa, la misión aborta |
| 7 | Adjuntar caja | — (escena de MoveIt) | La caja pasa a ser parte de la pinza en la escena |
| 8 | Ascenso | cartesiana, +0.15 m | — |
| 9 | Pre-entrega | pose sobre `car1_delivery_tray_frame`, calculada con tf2 | Llegó sin bloqueos del validador |
| 10 | Descenso de entrega | cartesiana, hasta 3 cm sobre la bandeja | — |
| 11 | Abrir pinza | pinza, 0.0 | La caja queda en la bandeja |
| 12 | Separar caja | — (escena de MoveIt) | La caja vuelve a ser un objeto de la escena, ahora sobre la bandeja |
| 13 | Retirada | cartesiana, +0.15 m | — |
| 14 | `home` | articular | Fin de la misión |

**Abortar es una respuesta válida.** Si el carrito no se ve, si el `tag_mesa` no coincide con el URDF, si el carrito está fuera de la zona de entrega o si la pinza cierra sin caja, la misión se detiene en un estado seguro, lo informa y no improvisa.

> [!TIP]
> **Cómo ejecutar metas calculadas en tiempo de ejecución.** `moveit_safe_executor` lee las metas de un YAML (`goals_file`) y acepta poses en el marco de planificación (`base_link`). Una forma sencilla: el nodo de misión lee las TF, escribe `metas_mision_gNN.yaml` con las poses ya expresadas en `base_link` y lanza el ejecutor con ese archivo, primero en modo seco. Extender el ejecutor para que resuelva marcos de tf2 directamente es una alternativa de nivel avanzado.

---

## 6. Requisitos Técnicos

### 6.1. Pose de observación
- Una pose **articular** desde la que la cámara de color ve completos el `tag_mesa` y el tag del carrito en cualquier punto de la zona de entrega. Se documenta con las seis posiciones articulares y una imagen de la cámara.

### 6.2. Verificación del `tag_mesa`
- Con la detección `camera_color_frame → tag_mesa_detectado` de la sección 6.3, pedir a tf2 la pose de `tag_mesa_detectado` en `base_link`. Esto exige el eslabón `end_effector_link → camera_link` del [Taller URDF](../talleres/TALLER_URDF_TF.md) §11 y unos ejes de cámara correctos (Ejercicio 3 de ese taller).
- Compararla con la pose de `tag_mesa` del URDF (`base_link → tag_mesa`, también con tf2). Si difieren en más de **2 cm**, la misión no arranca: se mide la mesa y se corrige `table_to_tag_mesa` en el URDF, no se compensa en el código.

### 6.3. Localización del carrito: corrección de perspectiva al marco de la mesa con RGB-D y tf2

**El problema.** En la pose de observación la cámara mira la mesa **inclinada**, no desde arriba. En la imagen, el `tag_mesa`, que es un cuadrado, se ve como un trapecio, y un centímetro de mesa ocupa más píxeles cerca de la cámara que lejos. Por eso no se puede pasar de píxeles a metros con una escala fija: hay que **corregir la perspectiva**, es decir, expresar lo que ve la cámara en el marco de la mesa. Aquí se hace en dos partes: la **profundidad** convierte cada tag en una pose 3D en la cámara, y **tf2** lleva el carrito al marco de la mesa.

**Paso 0 — Llevar la profundidad al marco de la cámara de color (registro).** El módulo de visión del Gen3 tiene **dos cámaras**: la de color, con su marco `camera_color_frame`, y el sensor de profundidad D410, con su marco `camera_depth_frame`, montado a unos 3 cm de la de color (en el URDF de la escena, `gen3_camera_depth_frame` está 2.75 cm a un lado y 1 cm más arriba en la imagen que `gen3_camera_color_frame`, con la misma orientación). Cada una tiene sus propios intrínsecos y su propia resolución. Por eso un píxel $(u, v)$ de la imagen RGB **no** es el mismo píxel en la imagen de profundidad cruda, y las esquinas del tag, que salen del RGB, no se pueden usar directamente sobre ella. Antes de todo hay que transformar la profundidad del marco del sensor de profundidad al marco de la cámara de color. Para cada píxel de la profundidad cruda:

1. Se lleva a un punto 3D en `camera_depth_frame` con el modelo pinhole inverso y los intrínsecos **de profundidad**.
2. Se transforma a `camera_color_frame` con la transformación fija $T_{\text{color},\text{depth}}$, que el driver publica en tf2.
3. Se proyecta en la imagen de color con los intrínsecos **de color**: ese es su píxel en la imagen registrada.

Ese proceso se llama **registro de la profundidad**, y no hay que programarlo: la estación anfitriona lanza `kinova_vision` con `depth_registration:=true`, y el driver lo hace con tf2 y publica el resultado en `/camera/depth_registered/image_rect`. Esa imagen está en `camera_color_frame` y tiene la resolución del color: el píxel $(u, v)$ de la profundidad registrada corresponde al mismo píxel de la imagen RGB, y su $Z$ ya está medido desde la cámara de color. El grupo **comprueba** el registro antes de usarlo:

- `ros2 run tf2_ros tf2_echo camera_color_frame camera_depth_frame` da una traslación de pocos centímetros y una rotación nula. Ese valor se anota en la Tabla 1 y se compara con el del URDF.
- El `header.frame_id` de `/camera/depth_registered/image_rect` es `camera_color_frame`, y su ancho y su alto son los de `/camera/color/image_raw` (`ros2 topic echo --once … --field header` y `--field width`).
- Se usan pares de imágenes de color y de profundidad con estampas cercanas.

> [!NOTE]
> **Por qué importa.** A 0.75 m, los 3 cm entre las dos cámaras corren la imagen unos 25 píxeles, y el tag del carrito mide unos 50 píxeles. En una simulación de la escena, con la profundidad registrada el tag del carrito quedó a 6 mm de su posición real. Con la profundidad sin registrar, el error fue de 6 mm si el techo del carrito era mucho más grande que el tag, porque la máscara corrida seguía cayendo sobre el mismo plano, pero de **13 cm** cuando el techo era apenas más grande que el tag: la máscara caía sobre la mesa y el ajuste de plano se arruinaba. Además, en el robot real las dos imágenes tienen resoluciones distintas, y sin registro ni siquiera coinciden los índices de los píxeles.

**Paso 1 — Esquinas del tag en la imagen RGB.** El detector AprilTag 36h11 de OpenCV entrega las cuatro esquinas de cada tag en píxeles, en orden: 0 arriba a la izquierda, 1 arriba a la derecha, 2 abajo a la derecha y 3 abajo a la izquierda.

**Paso 2 — El plano del tag con la profundidad.** Cada píxel dentro del tag (máscara con `cv2.fillConvexPoly`) que tenga profundidad válida se lleva a un punto 3D en `camera_color_frame` con el modelo pinhole inverso:

$$X = \frac{(u - c_x)\,Z}{f_x}, \qquad Y = \frac{(v - c_y)\,Z}{f_y}$$

Con esos puntos se ajusta el **plano del tag** por mínimos cuadrados (SVD), descartando los atípicos: se obtiene su normal $\mathbf{n}$ y su distancia $d$ a la cámara ($\mathbf{n} \cdot \mathbf{X} = d$). Se usa un plano y no la profundidad de cada esquina porque las esquinas están justo en el borde del tag, donde la profundidad es menos confiable.

**Paso 3 — Las cuatro esquinas en 3D y el marco del tag.** El rayo de cada esquina, $\mathbf{r} = K^{-1}(u, v, 1)^\top$, se intersecta con el plano: $\mathbf{P} = \frac{d}{\mathbf{n} \cdot \mathbf{r}}\,\mathbf{r}$. Con las cuatro esquinas 3D se arma el marco del tag:

- **Origen:** el centro de las cuatro esquinas.
- **Eje $x$:** del punto medio del borde izquierdo (esquinas 0 y 3) al del borde derecho (esquinas 1 y 2).
- **Eje $z$:** la normal del plano, apuntando hacia la cámara (fuera de la mesa).
- **Eje $y$:** $z \times x$.

La misma convención para los dos tags hace que sus marcos sean comparables. Esta pose ya contiene la inclinación de la cámara: es ahí donde se resuelve la perspectiva.

**Paso 4 — Publicar las dos detecciones como TF.** Con un `TransformBroadcaster` y la **estampa de la imagen**:

- `camera_color_frame → tag_mesa_detectado`
- `camera_color_frame → tag_carrito1_detectado`

Los nombres llevan `_detectado` porque `tag_mesa` y `tag_carrito1` ya existen en el árbol con otro padre: un frame no puede tener dos padres en tf2.

**Paso 5 — tf2 lleva el carrito al marco de la mesa.**
```python
t = tf_buffer.lookup_transform('tag_mesa_detectado', 'tag_carrito1_detectado', stamp_imagen)
```
tf2 calcula $T_{\text{mesa},\text{carrito}} = T_{\text{cam},\text{mesa}}^{-1}\,T_{\text{cam},\text{carrito}}$. Como los dos tags salen de la misma imagen, **la pose de la cámara se cancela**: el resultado es la pose del carrito en el marco de la mesa, sin importar dónde quedó el brazo ni cuánto se inclinó la cámara. Como es una pose **3D**, la altura del tag del carrito queda en $z$ y debe coincidir con `tag_height_m` medida con cinta: es la primera comprobación de que la profundidad está bien registrada.

**Paso 6 — Publicar el carrito en el árbol oficial.** El nodo publica `tag_mesa → tag_carrito1` con la $x$, la $y$ y el giro alrededor de $z$ del paso 5 (el techo del carrito es paralelo a la mesa) y $z$ igual a la altura medida. Así el carrito queda colgado del `tag_mesa` del URDF, y tf2 encadena el URDF del carrito hasta `car1_delivery_tray_frame` y entrega su pose en `base_link` para la meta de entrega. Se usa la **mediana** de 20 lecturas para filtrar el ruido. Si el tag del carrito queda fuera de la zona de entrega, la misión aborta.

> [!WARNING]
> **Cuándo falla la profundidad.** El D410 mide con luz infrarroja: superficies negras brillantes, reflejos y bordes devuelven 0 (sin dato). Imprima los tags en papel mate, mantenga la cámara a más de 0.3 m de la mesa, y exija un mínimo de píxeles válidos dentro de cada tag (por ejemplo, el 60 %) y un residuo del ajuste de plano pequeño (por ejemplo, menos de 5 mm). Si no se cumplen, el nodo no publica la detección y la misión aborta en el estado 2.

**Validación.**
- Con tres posiciones del tag del carrito marcadas con cinta, se compara la posición del método RGB-D con la de la cinta, y $z$ con `tag_height_m` (prueba P2).
- `ros2 run tf2_ros tf2_echo tag_mesa_detectado tag_carrito1_detectado` y el `view_frames` deben mostrar las dos detecciones colgando de `camera_color_frame`.
- Evidencia visual: la imagen RGB con las esquinas detectadas de los dos tags y la profundidad registrada en falso color de la misma escena.

> [!NOTE]
> En una simulación con la cámara a 0.60 m de altura, el carrito a unos 0.60 m en horizontal y ruido de 3 mm en la profundidad, este método ubicó el tag del carrito con un error de 2.9 mm en el marco de la mesa ($z$ = 0.153 m para un tag a 0.150 m). Con el robot real hay que medirlo: el ruido del D410 crece con la distancia.

### 6.4. Escena de MoveIt y pinza
- La caja se agrega a la escena de planificación como una caja de colisión con sus medidas reales, en la zona de recogida.
- Al cerrar la pinza, la caja se **adjunta** al efector (`AttachedCollisionObject`, con los eslabones de la pinza como `touch_links`) para que MoveIt la tenga en cuenta al planificar el traslado. Al soltarla se **separa** y queda en la escena sobre la bandeja.
- El carrito se agrega a la escena como obstáculo (caja envolvente en `car1_base_link`), para que la trayectoria de entrega no lo atraviese.

### 6.5. Seguridad (no negociable)
- Las reglas del [Laboratorio 03](../guias_laboratorio/GUIA_LAB_03_OPERACION_DISTRIBUIDA_KINOVA_TURNOS.md) y del [Laboratorio 04](../guias_laboratorio/GUIA_LAB_04_MOVEIT2_EJECUCION_SEGURA.md): un solo driver y un solo `move_group` en la estación anfitriona por cable, un solo ejecutor en el dominio, turnos, parada de emergencia custodiada y radio libre de 1.2 m.
- **Modo seco de la misión completa** antes de cada ejecución real, con el carrito ya ubicado, y revisión del docente.
- Los límites de `moveit_safe_executor` no se amplían para «hacer pasar» un plan; `gripper_max_effort` no se aumenta.
- Primero se ensaya con el driver fake en el dominio del grupo. En modo fake no hay pinza: las metas de pinza se validan en modo seco.

---

## 7. Protocolo Experimental y Registro

| Prueba | Configuración | Comportamiento esperado | Métrica a reportar |
|---|---|---|---|
| **P1. Verificación del `tag_mesa`** | Pose de observación, 20 lecturas | Pose medida cercana a la del URDF | Error de posición (mm) y de orientación (°) |
| **P2. Localización del carrito** | (a) Carrito quieto, 20 lecturas; (b) tres posiciones del tag del carrito marcadas con cinta | (a) Lecturas consistentes; (b) la pose RGB-D coincide con la cinta en $x$ e $y$, y $z$ con la altura medida | (a) Media y desviación de $x$, $y$, $\theta$; (b) error frente a la cinta (mm), $z$ frente a `tag_height_m` y residuo del ajuste de plano |
| **P3. Misión completa** | Tres posiciones del carrito elegidas por el docente | Caja dentro de la bandeja en los tres ensayos | Éxito del agarre y de la entrega, error de colocación con cinta (mm), tiempo de ciclo (s) |
| **P4. Carrito no visible** | Tag del carrito tapado | La misión aborta en el estado 2 sin mover el brazo hacia la entrega | Estado en que abortó y mensaje |
| **P5. Caja ausente** | Zona de recogida vacía | La pinza cierra completa y la misión aborta | Posición final reportada por la pinza y estado alcanzado |
| **P6. Cancelación del operador** | Ctrl+C durante el traslado | El brazo se detiene | Tiempo hasta la detención |

El **error de colocación** es la distancia, medida con cinta, entre el centro de la base de la caja y el centro de la bandeja.

### Grabación oficial de cada ensayo de la P3
```bash
ros2 bag record -s mcap -o dataset_proyecto_final_gNN_ensayoK \
  /tf /tf_static /joint_states \
  /robotiq_gripper_controller/gripper_cmd/_action/status \
  /joint_trajectory_controller/follow_joint_trajectory/_action/status
```
Con cada bolsa se entregan el informe del ejecutor (`report_file`, `*.jsonl`) y el `metas_mision_gNN.yaml` generado en ese ensayo.

---

## 8. Entregables

1. **Paquete `burger_mision_gNN`** en el repositorio del grupo, con el nodo de misión, la gestión de la caja en la escena, el launch de la misión y un `README.md` con la arquitectura (nodos, tópicos, acciones), el árbol de TF y cómo reproducir la misión.
2. **Instrumento de entrega** diligenciado: `C3_E03_G<grupo>_<codigo1>_<codigo2>_v1.docx`.
3. **Bolsas, informes del ejecutor y YAML de metas** de los tres ensayos de la P3.
4. **Video** de un ensayo completo de la misión con el robot real (desde la pose de observación hasta `home`).
5. **Sustentación** con el reto en vivo (sección 9).

---

## 9. Reto en Vivo de la Sustentación

El docente ubica el carrito en una posición nueva de la zona de entrega y, a su elección, introduce **una** falla: un tag tapado, un `tag_size_m` equivocado, la caja corrida 3 cm o el eslabón de la cámara sin publicar. El grupo debe:

1. Ejecutar la misión **sin editar el código**: sólo puede relanzar nodos y ajustar parámetros documentados.
2. Si la misión aborta, diagnosticar la causa por capas (red y dominio → TF → percepción → planificación → ejecución), corregirla y repetir, en menos de 20 minutos.

---

## 10. Rúbrica Analítica de Evaluación ABET (500 Puntos Zubatronic)

```text
Nota Académica sobre 5,0 = Nota Consolidada sobre 500 ÷ 100
Aporte a la Nota del Corte 3: E₃ = Nota Académica × 0,30 (30% de C3)
```

| Criterio | Peso | SO Principal | Indicador | Nivel N3 (Umbral de Logro: 300–399 pts) | Nivel N5 (Excelente: 475–500 pts) |
|---|:---:|:---:|---|---|---|
| **C1. Percepción RGB-D, corrección de perspectiva con tf2 y árbol de transformaciones** | 25% | **SO1** | **1.1** | La pose 3D de los dos tags se obtiene con la imagen RGB-D, las detecciones se publican como TF desde `camera_color_frame` y la pose del carrito en el marco de la mesa se obtiene con `lookup_transform`; la pose de la bandeja en `base_link` se obtiene con tf2. | Verificación del `tag_mesa` con error menor que 2 cm, comparación con la cinta, $z$ coherente con la altura medida, criterios de calidad de la profundidad (píxeles válidos y residuo del plano), filtrado con mediana y análisis del efecto de cada error del árbol sobre la colocación. |
| **C2. Pick and place con MoveIt 2 y la pinza** | 25% | **SO2** | **2.3** | La misión completa se ejecuta en el robot real al menos una vez, con todas las trayectorias a través de `moveit_safe_executor`. | Tres ensayos exitosos con el carrito en posiciones distintas, caja adjunta y separada en la escena, carrito como obstáculo y detección de agarre fallido por la posición de la pinza. |
| **C3. Validación experimental y diagnóstico** | 20% | **SO6** | **6.1 / 6.4** | Se ejecutan P1 a P3 con sus métricas y se registran las bolsas. | P1 a P6 completas, error de colocación explicado por sus fuentes y reto en vivo resuelto por diagnóstico por capas. |
| **C4. Seguridad en la operación del robot real** | 20% | **SO4** | **4.1** | Modo seco de la misión antes de cada ejecución real, turnos y parada de emergencia custodiada. | Además, la misión aborta en un estado seguro ante cada falla de P4 a P6 y ningún límite del ejecutor se modificó. |
| **C5. Trazabilidad y trabajo en equipo** | 10% | **SO3 / SO5** | **3.1–3.3 / 5.1** | Instrumento diligenciado, paquete con `README.md` y Anexo A individual completo. | Repositorio con commits de todos los integrantes, video y bolsas trazables a cada ensayo y sustentación individual sobresaliente. |
| **TOTAL** | **100%** | | | | **500 Puntos Máximos** |

Las rúbricas completas, con los cinco niveles de cada criterio, están en el instrumento asociado.
