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

**Todo ocurre sobre la mesa.** La caja y el carrito están sobre el tablero, en el mismo plano que el `tag_mesa`. El brazo está montado sobre la mesa y no alcanza el piso, 0.92 m más abajo. Que todo esté en un plano conocido es lo que permite localizar el carrito con una **corrección de perspectiva al plano de la mesa** (sección 6.3).

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
| Localizador AprilTag con `publish_tf:=true`: corrige la perspectiva con una homografía al plano del `tag_mesa` y publica `tag_mesa → tag_carrito1` ([Laboratorio 05](../guias_laboratorio/GUIA_LAB_05_TF_DRON_CARRITO_MICROROS.md)) | La **verificación del `tag_mesa`**: su pose medida con la cámara frente a la del URDF |
| | La **corrección de paralaje** por la altura del tag del carrito sobre la mesa (sección 6.3), que el localizador no hace |
| URDF del carrito: `tag_carrito1 → car1_base_link → car1_delivery_tray_frame` (`car1_apriltag.urdf`) | El **launch de la misión**, el registro de las pruebas y el `README.md` del paquete |
| Cadena de la cámara de muñeca y su eslabón con el brazo ([`TALLER_URDF_TF.md`](../talleres/TALLER_URDF_TF.md) §11) | |

El grupo trabaja en un paquete ROS 2 propio dentro de su repositorio privado, `burger_mision_gNN` (`NN` = número de grupo), con al menos el nodo de misión, la gestión de la caja en la escena y un launch.

---

## 3. Árbol de Transformaciones de la Misión

```text
base_link (= world de la escena)
 ├── … ── end_effector_link ── camera_link ── camera_color_frame        (driver + eslabón del Taller URDF §11)
 └── table_link ── tag_mesa                                              (URDF, medido en la mesa real)
                      ├── zona_recogida      (fija: la define el grupo en el marco del tag_mesa)
                      └── tag_carrito1       (VISIÓN: localizador AprilTag, dinámica)
                            └── car1_base_link ── car1_delivery_tray_frame   (URDF del carrito)
```

- La **meta de agarre** se expresa en el marco del `tag_mesa` (zona de recogida).
- La **meta de entrega** se calcula en cada ensayo leyendo con tf2 la pose de `car1_delivery_tray_frame` en `base_link`.
- El localizador mide el carrito **respecto al `tag_mesa`**, no respecto a la cámara: la pose del carrito no depende de dónde esté el brazo al observar. Por eso es crítico que el `tag_mesa` del URDF coincida con el sticker real: un error ahí se traslada entero a la entrega. La verificación de la sección 6.2 lo comprueba antes de mover el brazo.

---

## 4. Celda, Caja y Zonas de Trabajo

| Elemento | Especificación | Por qué |
|---|---|---|
| **Caja de hamburguesa** | Caja de prueba entregada por el docente, con un lado de agarre de **máximo 75 mm** y menos de 300 g | La Robotiq 2F-85 abre 85 mm: una caja más ancha no cabe entre los dedos |
| **Zona de recogida** | Rectángulo marcado con cinta, con centro en $(0.00,\ +0.25)$ m en el marco del `tag_mesa` (el grupo mide y documenta el valor real) | Fija y conocida: el agarre no depende de la visión |
| **Zona de entrega** | El docente ubica el carrito con el centro de su tag dentro de $x \in [0.10,\ 0.35]$ m e $y \in [-0.30,\ -0.18]$ m en el marco del `tag_mesa`, con su lado largo (0.25 m) a menos de 30° del eje $x$ del `tag_mesa` | Dentro del alcance del brazo y del campo de visión; el carrito (0.25 × 0.20 m) queda al menos 5 cm dentro del borde de la mesa ($y = -0.45$ m en `base_link`) y sin tapar el `tag_mesa` |
| **Carrito** | Detenido y con los motores apagados durante la misión; AprilTag 36h11 en el techo | El carrito es el destino, no participa del movimiento |
| **Alturas** | La caja se suelta con su base a no más de 3 cm de la bandeja | Una caída mayor puede volcarla fuera de la bandeja |

El grupo mide con cinta y calibrador las dimensiones de la caja, la altura del tag del carrito sobre la mesa (`tag_height_m`) y la de la bandeja, y las registra en la Tabla 1 del instrumento.

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
- Con la cámara, los intrínsecos de color y el tamaño del tag, estimar la pose del `tag_mesa` en `camera_color_frame` (por ejemplo con `cv2.solvePnP` y `SOLVEPNP_IPPE_SQUARE`) y llevarla a `base_link` con tf2. Esto exige el eslabón `end_effector_link → camera_link` del [Taller URDF](../talleres/TALLER_URDF_TF.md) §11 y unos ejes de cámara correctos (Ejercicio 3 de ese taller).
- Compararla con la pose de `tag_mesa` del URDF. Si difieren en más de **2 cm**, la misión no arranca: se mide la mesa y se corrige `table_to_tag_mesa` en el URDF, no se compensa en el código.

### 6.3. Localización del carrito: corrección de perspectiva al plano de la mesa

**El problema.** En la pose de observación la cámara mira la mesa **inclinada**, no desde arriba. En la imagen, el `tag_mesa`, que es un cuadrado, se ve como un trapecio, y un centímetro de mesa ocupa más píxeles cerca de la cámara que lejos. Por eso no se puede pasar de píxeles a metros con una escala fija: hay que **corregir la perspectiva**, es decir, llevar la imagen al plano de la mesa, como si se mirara desde arriba.

**Paso 1 — Homografía al plano del `tag_mesa`.** El `tag_mesa` es un cuadrado de lado $s$ conocido, pegado sobre la mesa. Sus cuatro esquinas se conocen en dos sistemas: en píxeles (las da el detector) y en metros, en su propio marco:

$$\text{esquina } 0 \to (-\tfrac{s}{2}, +\tfrac{s}{2}),\quad 1 \to (+\tfrac{s}{2}, +\tfrac{s}{2}),\quad 2 \to (+\tfrac{s}{2}, -\tfrac{s}{2}),\quad 3 \to (-\tfrac{s}{2}, -\tfrac{s}{2})$$

Cuatro correspondencias definen una **homografía** $H$ (matriz de 3 × 3) que lleva **cualquier píxel del plano de la mesa** a metros en el marco del `tag_mesa`, aunque la cámara esté inclinada:

$$\begin{bmatrix} x' \\ y' \\ w \end{bmatrix} = H \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}, \qquad (x, y) = \left(\frac{x'}{w}, \frac{y'}{w}\right)$$

Aplicando $H$ a las cuatro esquinas del tag del carrito se obtiene su centro $(x_0, y_0)$ y su orientación $\theta$ (eje de la esquina 0 a la 1) en el marco del `tag_mesa`. $H$ se recalcula en **cada imagen**, así que el resultado no depende de dónde quedó el brazo. Es lo que hace el localizador del curso (`_to_reference_plane`, con `cv2.getPerspectiveTransform` y `cv2.perspectiveTransform`; ver el [taller de localización con AprilTag](../talleres/TALLER_LOCALIZACION_APRILTAG_KINOVA_MICROROS.md), Fase 3).

**Paso 2 — Corrección de paralaje.** La homografía sólo es exacta para puntos **sobre** la mesa. El tag del carrito está a una altura $h$ (`tag_height_m`) sobre el tablero, así que el punto $(x_0, y_0)$ que entrega es donde el rayo de la cámara que pasa por el tag **corta la mesa**, no la posición real del tag: queda desplazado hacia afuera, alejándose de la cámara. Se corrige con la posición de la cámara $C = (C_x, C_y, C_z)$ en el marco del `tag_mesa` y triángulos semejantes a lo largo de ese rayo:

$$(x, y) = (C_x, C_y) + \frac{C_z - h}{C_z}\big((x_0, y_0) - (C_x, C_y)\big)$$

$C$ sale de la misma detección del `tag_mesa`: con `cv2.solvePnP` se obtiene la pose del tag en la cámara $(R, t)$, y la cámara en el marco del tag es $C = -R^{\top} t$. La orientación $\theta$ no cambia: el techo del carrito es paralelo a la mesa.

> [!WARNING]
> **El paralaje no es un detalle.** Con la cámara a $C_z = 0.60$ m sobre la mesa, un tag a $h = 0.15$ m y el carrito a 0.30 m (en horizontal) del punto bajo la cámara, el error sin corregir es $0.30 \cdot 0.15 / 0.60 = 7.5$ cm: la caja caería fuera de la bandeja. El error crece con la distancia horizontal: en una simulación con la cámara a 0.60 m de altura y el carrito a unos 0.60 m en horizontal, la homografía sola lo ubicó a 21 cm de su posición real, y la corrección lo llevó a menos de 1 mm. El localizador del curso publica $z = h$ en la TF, pero **no corrige** $x$ ni $y$. Esta corrección la implementa el grupo, en su propio nodo o extendiendo el localizador.

**Paso 3 — Pose de entrega.**
- La TF corregida `tag_mesa → tag_carrito1` se encadena con el URDF del carrito hasta `car1_delivery_tray_frame`, y con tf2 se obtiene su pose en `base_link`.
- Se usa la **mediana** de 20 lecturas para filtrar el ruido del detector.
- Si el tag del carrito queda fuera de la zona de entrega, la misión aborta.

**Validación.** Con tres posiciones del carrito marcadas con cinta, se compara la posición medida con la homografía sola y con la corrección de paralaje (prueba P2). Como evidencia, se genera además la **vista cenital rectificada** de la mesa con `cv2.warpPerspective` y la homografía: en ella el `tag_mesa` debe verse cuadrado y los bordes de la mesa, paralelos.

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
| **P2. Localización del carrito** | (a) Carrito quieto, 20 lecturas; (b) tres posiciones del tag del carrito marcadas con cinta | (a) Lecturas consistentes; (b) la corrección de paralaje acerca la medida a la cinta | (a) Media y desviación de $x$, $y$, $\theta$; (b) error frente a la cinta sin y con corrección de paralaje (mm) |
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
| **C1. Percepción, corrección de perspectiva y árbol de transformaciones** | 25% | **SO1** | **1.1** | La TF `tag_mesa → tag_carrito1` se obtiene con la homografía al plano de la mesa y la corrección de paralaje, y la pose de la bandeja en `base_link` se obtiene con tf2. | Verificación del `tag_mesa` con error menor que 2 cm, vista cenital rectificada, corrección de paralaje con su efecto medido frente a la cinta, filtrado con mediana y análisis del efecto de cada error del árbol sobre la colocación. |
| **C2. Pick and place con MoveIt 2 y la pinza** | 25% | **SO2** | **2.3** | La misión completa se ejecuta en el robot real al menos una vez, con todas las trayectorias a través de `moveit_safe_executor`. | Tres ensayos exitosos con el carrito en posiciones distintas, caja adjunta y separada en la escena, carrito como obstáculo y detección de agarre fallido por la posición de la pinza. |
| **C3. Validación experimental y diagnóstico** | 20% | **SO6** | **6.1 / 6.4** | Se ejecutan P1 a P3 con sus métricas y se registran las bolsas. | P1 a P6 completas, error de colocación explicado por sus fuentes y reto en vivo resuelto por diagnóstico por capas. |
| **C4. Seguridad en la operación del robot real** | 20% | **SO4** | **4.1** | Modo seco de la misión antes de cada ejecución real, turnos y parada de emergencia custodiada. | Además, la misión aborta en un estado seguro ante cada falla de P4 a P6 y ningún límite del ejecutor se modificó. |
| **C5. Trazabilidad y trabajo en equipo** | 10% | **SO3 / SO5** | **3.1–3.3 / 5.1** | Instrumento diligenciado, paquete con `README.md` y Anexo A individual completo. | Repositorio con commits de todos los integrantes, video y bolsas trazables a cada ensayo y sustentación individual sobresaliente. |
| **TOTAL** | **100%** | | | | **500 Puntos Máximos** |

Las rúbricas completas, con los cinco niveles de cada criterio, están en el instrumento asociado.
