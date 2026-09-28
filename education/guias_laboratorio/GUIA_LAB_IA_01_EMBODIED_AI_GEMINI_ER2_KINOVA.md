# GUÍA DE LABORATORIO IA 01: EMBODIED AI CON GEMINI ROBOTICS-ER 2 Y LA CÁMARA RGB-D DEL KINOVA GEN3

---

| FACULTAD | PROGRAMA | ASIGNATURA | SEMESTRE | CÓDIGO GUÍA | REVISIÓN |
|:---|:---|:---|:---:|:---:|:---:|
| Facultad de Ingeniería | Ingeniería Mecatrónica | INTELIGENCIA ARTIFICIAL | Por definir | GL-AA-F-1 / LAB-IA-01 | 1.0 (2026-2) |

---

## 1. CONTROL DE CAMBIOS

| Descripción del Cambio | Justificación | Fecha |
|---|---|:---:|
| Versión inicial de la práctica y de su plantilla de código e informe | Lleva a laboratorio la propuesta de razonamiento espacial con Gemini Robotics-ER (`docs/research/EXPERIMENTO_IA_LOCALIZACION_GEMINI.md`, `ros2_setup/PROPUESTA_GEMINI_ER.md`), actualizada al modelo `gemini-robotics-er-2-preview`. Ofrece dos rutas equivalentes: **A sin ROS 2** (RTSP directo) y **B con ROS 2** (driver `kinova_vision`). Incorpora el registro de la profundidad al marco de color, que la propuesta original omitía, y la calibración de la cámara de color, porque el archivo por defecto del driver para 1920×1080 trae un punto principal imposible (`cy = 96.97 px`). | 28/09/2026 |

---

## 2. INTRODUCCIÓN

### 2.1. ¿Qué es *Embodied AI*?
Un modelo de lenguaje responde preguntas sobre texto; un agente **corporizado** (*embodied*) tiene que responderlas **en el espacio físico**: percibe con sensores, razona sobre objetos reales y propone acciones que un cuerpo (aquí, el brazo Kinova Gen3) podría ejecutar. El ciclo es:

```
   PERCEPCIÓN                  RAZONAMIENTO                     ACCIÓN
 cámara RGB-D  ───►  VLM: "¿dónde está?, ¿se puede   ───►  punto 3D, plan de agarre,
 (color + mm)         agarrar?, ¿se cumplió la tarea?"       verificación de éxito
       ▲                                                             │
       └──────────────────────── el mundo cambia ◄──────────────────┘
```

La dificultad central no es reconocer el objeto, sino **aterrizar** (*grounding*) la respuesta del modelo: pasar de "la caja libre, a la izquierda del termo" a un punto `(X, Y, Z)` en metros que un planificador de movimiento pueda usar.

### 2.2. Gemini Robotics-ER 2
`gemini-robotics-er-2-preview` es el modelo visión-lenguaje (VLM) de Google DeepMind especializado en **razonamiento corporizado** (*Embodied Reasoning*). Recibe imágenes y texto y devuelve texto, normalmente JSON. Sus capacidades relevantes para esta práctica son:

| Capacidad | Qué devuelve | Tarea en la plantilla |
|---|---|---|
| Señalamiento (*pointing*) | `[{"point": [y, x], "label": "..."}]` | `puntos`, `propio` |
| Cajas delimitadoras | `[{"box_2d": [ymin, xmin, ymax, xmax], "label": "..."}]` | `cajas` |
| *Affordances* y seguridad | Punto de agarre, o lista vacía si no es seguro | `agarre` |
| Orquestación de tareas | Plan de pasos estructurado | `plan` |
| Detección de éxito | Veredicto a partir de imágenes antes/después | `exito` |

Tres detalles de la API que causan la mayoría de los errores:

1. **Las coordenadas vienen normalizadas a 0-1000 y en orden `[y, x]`** (fila, columna), sin importar la resolución de la imagen enviada. Por eso se puede enviar una imagen reducida a 1280 px y aun así ubicar el punto en la imagen original de 1920×1080.
2. **El nivel de razonamiento se controla con `thinking_level`** (`low`, `high`, ...). Más razonamiento suele mejorar las preguntas difíciles a costa de latencia y tokens: es una de las variables del experimento.
3. **Es un modelo *preview***: nombre, cuotas y precios pueden cambiar. Cada llamada tarda segundos, así que **nunca** se usa dentro de un lazo de control; se consulta una vez por decisión.

> [!NOTE]
> ¿Por qué no YOLO? Un detector entrenado encuentra *todas* las cajas; no sabe cuál está libre, cuál está abierta ni cuál "pertenece" a quién. La comparación completa está en [`docs/research/EXPERIMENTO_IA_LOCALIZACION_GEMINI.md`](../../docs/research/EXPERIMENTO_IA_LOCALIZACION_GEMINI.md) §2.

### 2.3. La cámara del Kinova: dos sensores, no uno
El módulo de visión de la muñeca tiene **dos** sensores separados unos milímetros ([Laboratorio 02](GUIA_LAB_02_PRUEBAS_CAMARA_KINOVA_VISION.md)):

| Sensor | Stream RTSP | Formato medido en el laboratorio |
|---|---|---|
| Color (Omnivision OV5640) | `rtsp://192.168.1.10/color` (H.264) | 1920×1080 RGB, ≈ 30 FPS |
| Profundidad (Intel RealSense D410) | `rtsp://192.168.1.10/depth` (RTP `X-GST`, solo GStreamer) | 480×270 `GRAY16_LE`, **milímetros** por píxel, 0 = sin dato |

Gemini señala un píxel **en la imagen de color**, pero la distancia está **en la imagen de profundidad**, que tiene otra resolución, otro campo de visión y otro origen. Tomar "el mismo píxel escalado" es un error que se mide en la Fase 5.

### 2.4. Geometría: del punto de Gemini al punto 3D

**Paso 1 — normalizado a píxel** (imagen de color de ancho $W$ y alto $H$):

$$u = \frac{x_n}{1000}(W-1), \qquad v = \frac{y_n}{1000}(H-1)$$

**Paso 2 — registro de la profundidad** al marco de color. Cada píxel de profundidad $(u_d, v_d)$ con distancia $z_d$ se lleva a 3D con los intrínsecos de profundidad $K_d$, se traslada al marco de color con los extrínsecos $[R \mid t]$ y se proyecta con los intrínsecos de color $K_c$:

$$P_d = z_d\,K_d^{-1}\begin{bmatrix}u_d\\ v_d\\ 1\end{bmatrix}, \qquad P_c = R\,P_d + t, \qquad \begin{bmatrix}u_c\\ v_c\\ 1\end{bmatrix} \sim K_c\,P_c$$

Es lo que hace `depth_image_proc::RegisterNode` en ROS 2; la plantilla lo implementa en `registrar_profundidad()` con numpy para que ambas rutas usen el mismo código. En el Kinova, `R = I` y `t = (-19.5, -5, 0) mm` según el `kinova_vision.launch.py` oficial.

**Paso 3 — profundidad robusta**: como la profundidad tiene ~16 veces menos píxeles que el color, la imagen registrada queda con huecos. Se toma la **mediana** de los valores no nulos en una ventana alrededor de $(u, v)$.

**Paso 4 — desproyección** con el modelo pinhole ($f_x, f_y, c_x, c_y$ del sensor de color):

$$X = \frac{(u - c_x)\,Z}{f_x}, \qquad Y = \frac{(v - c_y)\,Z}{f_y}, \qquad Z = z$$

El resultado está en el **marco óptico** de la cámara de color: X a la derecha, Y hacia abajo, Z hacia delante.

> [!WARNING]
> **Los intrínsecos de color por defecto del driver para 1920×1080 no sirven.** El archivo `default_color_calib_1920x1080.ini` de `kinova_vision` declara `cy = 96.97 px` en una imagen de 1080 filas (el centro está cerca de 540) y una focal incompatible con la de 640×480. Con esos valores cada punto 3D sale desplazado decenas de centímetros. Por eso la Fase 2 calibra la cámara, y la plantilla **advierte** si el punto principal cae lejos del centro.

---

## 3. OBJETIVOS

### 3.1. Objetivo General
Construir y evaluar experimentalmente un pipeline de *Embodied AI* que, a partir de una instrucción en lenguaje natural y una captura RGB-D del Kinova Gen3, use Gemini Robotics-ER 2 para identificar objetos por razonamiento espacial y los localice en metros, cuantificando su exactitud semántica, su error geométrico y el costo del razonamiento.

### 3.2. Objetivos Específicos
1. **Adquirir escenas RGB-D** del módulo de visión del Kinova por RTSP directo (Ruta A) o desde el driver ROS 2 (Ruta B).
2. **Calibrar y verificar** los intrínsecos de la cámara de color, comprobando la calibración con una medida conocida.
3. **Implementar** la conversión de coordenadas de Gemini, la lectura robusta de profundidad y la desproyección pinhole, validándolas con pruebas automáticas.
4. **Diseñar prompts** que obliguen al modelo a razonar sobre relaciones espaciales, estado y seguridad de los objetos, con salida JSON verificable.
5. **Evaluar** exactitud semántica, error 3D, latencia y tokens en función del nivel de razonamiento, y el efecto del registro de profundidad (ablación).
6. **Cerrar el ciclo corporizado** con un plan de manipulación validado (no ejecutado) y una detección de éxito antes/después.

---

## 4. DESCRIPCIÓN DE LA PRÁCTICA

La práctica se realiza en **grupos de 2 o 3 personas**. Cada grupo elige una ruta; las fases 0, 2 y 4 a 7 son idénticas en ambas y producen las mismas evidencias.

| | **Ruta A — sin ROS 2** | **Ruta B — con ROS 2** |
|---|---|---|
| Para quién | Quienes aún no han visto ROS 2 | Quienes ya aprobaron los laboratorios 01 y 02 de ROS |
| Cómo llega la imagen | `embodied_sin_ros.py capturar` abre el RTSP del robot con OpenCV + GStreamer | El nodo `embodied_ros2_nodo.py` se suscribe al driver `kinova_vision` de la estación anfitriona |
| Dónde se trabaja | Capturas guardadas en disco: el análisis no necesita el robot | En vivo, con el mismo `ROS_DOMAIN_ID` que la anfitriona |
| TODO a completar | 1, 2, 3, 4 | 1, 2, 3, 4 y 5 |

```
  +---------------------------------------------------------------------------------------+
  |                                FASES DE LA PRÁCTICA                                   |
  +---------------------------------------------------------------------------------------+
  |  FASE 0: ENTORNO, CLAVE DE API Y PRUEBA SIN ROBOT                                     |
  |  FASE 1: ADQUISICIÓN RGB-D  (Ruta A: RTSP por turnos · Ruta B: driver + nodo ROS 2)   |
  |  FASE 2: CALIBRACIÓN DE LA CÁMARA DE COLOR Y VERIFICACIÓN                             |
  |  FASE 3: IMPLEMENTACIÓN DE LOS TODO Y AUTOEVALUACIÓN (pytest)                         |
  |  FASE 4: EXPERIMENTO 1 — RAZONAMIENTO SEMÁNTICO-ESPACIAL                              |
  |  FASE 5: EXPERIMENTO 2 — EXACTITUD 3D Y ABLACIÓN DEL REGISTRO DE PROFUNDIDAD          |
  |  FASE 6: EXPERIMENTO 3 — COSTO DEL RAZONAMIENTO (thinking_level)                      |
  |  FASE 7: EXPERIMENTO 4 — CICLO CORPORIZADO: AGARRE, PLAN Y DETECCIÓN DE ÉXITO         |
  +---------------------------------------------------------------------------------------+
```

**Duración:** una sesión presencial de 3 horas (turnos de captura de 10 minutos por grupo en la Ruta A) y trabajo autónomo para los experimentos que no requieren el robot.

**Código base:** [`plantilla_lab_ia_embodied/`](plantilla_lab_ia_embodied/README.md). **Informe:** [`PLANTILLA_INFORME_LAB_IA_01_EMBODIED_AI.docx`](plantilla_lab_ia_embodied/PLANTILLA_INFORME_LAB_IA_01_EMBODIED_AI.docx).

### 4.1. Resultados de Aprendizaje Evaluables (RAE) y Ponderación

| Criterio | Evidencia observable | SO | Ponderación |
|---|---|:---:|:---:|
| **C1. Implementación del pipeline percepción → razonamiento → geometría** | TODO completos, `pytest` en verde, CSV generado por el código del grupo | SO1 | 30% |
| **C2. Diseño experimental y medición** | Protocolo de las fases 4 a 7 con repeticiones, verdad de terreno medida y variables controladas | SO6 | 25% |
| **C3. Análisis crítico del modelo** | Métricas (tasa de acierto, error 3D, latencia, tokens), ablación y discusión de fallas del VLM | SO6 | 20% |
| **C4. Informe técnico** | Plantilla de informe completa, figuras anotadas y conclusiones sustentadas en datos | SO3 | 15% |
| **C5. Uso responsable de IA, seguridad y trabajo en equipo** | Manejo de la clave, datos enviados a terceros, respeto de la zona del robot, roles declarados | SO4 / SO5 | 10% |
| **Total** | | | **100%** |

> [!NOTE]
> Los códigos SO siguen la numeración ABET usada en las demás guías del repositorio; ajústelos a los indicadores del syllabus vigente de Inteligencia Artificial.

---

## 5. MATERIALES Y EQUIPOS

### 5.1. Equipos del Laboratorio
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Brazo Kinova Gen3 (6 GDL) con módulo de visión en la muñeca, en **pose de observación fija** mirando la mesa | 1 | Unidad |
| Estación anfitriona (Ubuntu 24.04, ROS 2 Jazzy, `kinova_vision`) cableada al router `192.168.1.1` | 1 | Unidad |
| Router TP-Link AX12, SSID `ros2`, subred `192.168.1.0/24` | 1 | Unidad |
| Parada de emergencia física verificada | 1 | Unidad |
| Objetos de escena: 2 cajas de hamburguesa (o recipientes), termo o botella, 2 cuadernos, estuche y esferos, bandeja | 1 | Juego |
| Cinta métrica y regla de 30 cm; hoja A4 blanca con bordes marcados | 1 | Juego |

### 5.2. Equipos del Estudiante
| DESCRIPCIÓN | CANTIDAD | UNIDAD DE MEDIDA |
|---|:---:|:---:|
| Portátil con Ubuntu 24.04 (Ruta B: además ROS 2 Jazzy y el perfil CycloneDDS del Laboratorio 02) | 1 | Por grupo |
| Clave personal de la Gemini API (Google AI Studio) | 1 | Por grupo |
| Repositorio `burger_delivery` actualizado (carpeta `plantilla_lab_ia_embodied`) | 1 | Por grupo |

---

## 6. SEGURIDAD, ÉTICA Y USO RESPONSABLE DE IA

> [!WARNING]
> 1. **El robot NO se mueve durante la práctica.** La estación anfitriona lo deja en la pose de observación antes de empezar. El código de los estudiantes solo lee la cámara: no abre sesión de control ni publica trayectorias.
> 2. **Zona de trabajo:** para reorganizar la escena, avise en voz alta y confirme que nadie está operando la anfitriona. Mantenga libre el radio de 1.2 m del brazo cuando esté energizado.
> 3. **Las imágenes salen del laboratorio.** Cada consulta envía la foto a servidores de Google. No capture personas, rostros, carnés ni documentos. En la capa gratuita, los términos de la API permiten que el contenido se use para mejorar los productos: revise los términos vigentes antes de enviar cualquier imagen.
> 4. **La clave de API es personal.** Úsela solo como variable de entorno (`GEMINI_API_KEY`). Una clave subida a GitHub debe revocarse de inmediato en AI Studio.
> 5. **Un VLM puede equivocarse con total seguridad.** Ningún resultado de esta práctica se envía al robot sin validación humana y geométrica (Fase 7).

---

## 7. PROCEDIMIENTO EXPERIMENTAL

### Fase 0: Entorno, Clave de API y Prueba sin Robot

1. **Instale el entorno** (una vez por portátil) siguiendo [`plantilla_lab_ia_embodied/README.md`](plantilla_lab_ia_embodied/README.md) §Instalación. El punto clave: OpenCV debe ser el de Ubuntu (`python3-opencv`), porque el de `pip` no trae GStreamer y sin él el stream de profundidad no abre (Laboratorio 02, Fase 2).
   ```bash
   python3 -m venv --system-site-packages ~/venv_embodied
   source ~/venv_embodied/bin/activate
   cd ~/ros2_ws/src/burger_delivery/education/guias_laboratorio/plantilla_lab_ia_embodied
   pip install -r requirements.txt
   ```
2. **Genere la clave** en [Google AI Studio](https://aistudio.google.com/) y expórtela en cada terminal:
   ```bash
   export GEMINI_API_KEY="su-clave"
   ```
3. **Prueba de humo sin robot ni API** (valida la instalación):
   ```bash
   python3 -m pytest tests -q   # debe mostrar 9 fallos con NotImplementedError (sus TODO) y 6 aprobadas
   ```
   Sin cuota de API, cualquier consulta de las fases siguientes puede ejecutarse con una respuesta escrita a mano, por ejemplo `--respuesta '[{"point": [500, 500], "label": "centro"}]'`, para depurar la geometría sin gastar llamadas.
   Registre la versión de `google-genai` (`pip show google-genai`) en la **Tabla 1**.

---

### Fase 1: Adquisición RGB-D

La estación anfitriona lleva el brazo a la **pose de observación** (cámara a unos 50-70 cm de la mesa, mirando hacia abajo) y **no lo mueve** durante la sesión.

#### Ruta A — sin ROS 2 (captura por turnos)

> [!IMPORTANT]
> **Un solo consumidor RTSP a la vez.** Como se vio en el Laboratorio 02, la cámara puede rechazar streams nuevos si otro proceso la mantiene abierta. Los grupos de la Ruta A capturan **por turnos de 10 minutos**, con el driver ROS 2 de la anfitriona **detenido**, y analizan después con los archivos guardados.

1. Verifique el enlace y la cámara con el visor del Laboratorio 02 (cierre con `q`):
   ```bash
   ping -c 3 192.168.1.10
   python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_camera.py --ip 192.168.1.10 --stream depth
   ```
2. Capture **todas** las escenas que necesitará en las fases 2 a 7 (lista en la **Tabla 2**). Cada captura toma unos segundos y guarda color, profundidad de 16 bits y metadatos:
   ```bash
   python3 embodied_sin_ros.py capturar --escena regla    --nota "hoja A4 horizontal, centrada"
   python3 embodied_sin_ros.py capturar --escena semantica1 --nota "dos cajas, termo sobre una"
   # ... una captura por escena de la Tabla 2
   ```
3. Revise en cada `capturas/<escena>_meta.json` el porcentaje de píxeles de profundidad válidos y el desfase entre color y profundidad. Una escena con menos del 60 % de píxeles válidos se repite (superficies negras o brillantes devuelven 0).

#### Ruta B — con ROS 2 (en vivo)

1. La **anfitriona** lanza el driver con color y profundidad, limitando la frecuencia para no saturar el Wi-Fi (la imagen de profundidad cruda pesa 259 KB por cuadro):
   ```bash
   ros2 launch kinova_vision kinova_vision.launch.py device:=192.168.1.10 \
       max_color_pub_rate:=10.0 max_depth_pub_rate:=5.0
   ```
2. Cada grupo, con el `ROS_DOMAIN_ID`, el RMW y el `CYCLONEDDS_URI` del Laboratorio 02, comprueba los tópicos:
   ```bash
   ros2 topic hz /camera/color/image_raw/compressed
   ros2 topic hz /camera/depth/image_raw
   ros2 topic echo /camera/depth/camera_info --once --field k
   ```
3. Lance el nodo del grupo y envíe una consulta de prueba:
   ```bash
   python3 embodied_ros2_nodo.py --ros-args -p grupo:=G03 -p thinking:=low
   ros2 topic pub --once /embodied/consulta std_msgs/msg/String \
     "{data: '{\"tarea\": \"puntos\", \"objeto\": \"la caja de hamburguesa\"}'}"
   ros2 topic echo /embodied/respuesta --once
   ```
   La llamada a Gemini corre en un **hilo aparte** (`_cb_consulta` → `_trabajar`): explique en el informe qué ocurriría con `/camera/...` si se hiciera dentro del *callback*.
4. **Guarde las escenas** de la Tabla 2 con la tarea especial `guardar`. El nodo escribe los mismos archivos que `capturar` de la Ruta A (`capturas/<escena>_color.png`, `_depth.png` de 16 bits y `_meta.json`), así que desde aquí las fases 4 a 7 se ejecutan **igual en las dos rutas**, con `embodied_sin_ros.py consultar` sobre las escenas guardadas:
   ```bash
   ros2 topic pub --once /embodied/consulta std_msgs/msg/String \
     "{data: '{\"tarea\": \"guardar\", \"escena\": \"semantica1\", \"nota\": \"dos cajas, termo sobre una\"}'}"
   ```
   Las consultas en vivo por `/embodied/consulta` son equivalentes y también escriben en `resultados/resultados.csv` (columna `opcion = B-ROS2`).

---

### Fase 2: Calibración de la Cámara de Color y Verificación

`config_camara_kinova.json` trae solo los valores del driver para 640×480 (color) y 480×270 (profundidad). Para la resolución real del robot (1920×1080) hay dos caminos:

1. **Calibración oficial (docente o monitor, una vez):** `leer_calibracion_kortex.py` lee con la Kortex API los intrínsecos de ambos sensores y los extrínsecos, y los escribe en el JSON. El JSON resultante se comparte con todos los grupos.
2. **Calibración rápida (cada grupo):** semejanza de triángulos con la escena `regla`. Con una hoja A4 horizontal (0.297 m) paralela a la imagen, centrada, a una distancia $Z$ medida con cinta desde el frente del módulo de visión:
   $$f_x \approx \frac{(u_2 - u_1)\,Z}{0.297}, \qquad f_y \approx f_x, \qquad c_x \approx W/2, \qquad c_y \approx H/2$$
   Lea las columnas $u_1$ y $u_2$ de los bordes de la hoja con cualquier visor que muestre coordenadas del puntero (GIMP, por ejemplo) y ejecute:
   ```bash
   python3 embodied_sin_ros.py calibrar --escena regla --u1 <px> --u2 <px> \
       --ancho-real-m 0.297 --distancia-m <Z medida>
   ```
   El comando agrega la resolución `1920x1080` al JSON e informa el campo de visión horizontal resultante.
3. **Verificación obligatoria (cierra el lazo):** pida a Gemini las dos esquinas superiores de la hoja y compare la distancia 3D calculada con 0.297 m:
   ```bash
   python3 embodied_sin_ros.py consultar --escena regla --tarea puntos \
       --objeto "la esquina superior izquierda y la esquina superior derecha de la hoja blanca"
   ```
   Un error mayor al 5 % indica una calibración o una distancia $Z$ mal medida: repita antes de seguir. Registre todo en la **Tabla 3**.

> [!TIP]
> En la Ruta B, el nodo usa el mismo JSON (parámetro `fuente_intrinsecos`). Con `-p fuente_intrinsecos:=camera_info` usaría los del driver: pruébelo y observe la advertencia sobre `cy`.

---

### Fase 3: Implementación de los TODO y Autoevaluación

Complete, en este orden, las funciones de `embodied_comun.py` (y `embodied_ros2_nodo.py` en la Ruta B):

| TODO | Función | Pista |
|---|---|---|
| 2 | `normalizado_a_pixel(punto_yx, ancho, alto)` | El primer elemento es **y**. Valide el rango 0-1000 y lance `ValueError` si no se cumple. |
| 3 | `profundidad_robusta(depth_mm, u, v, ventana)` | Recorte una ventana centrada **sin salirse de la imagen**, descarte los ceros, devuelva la mediana en **metros** o `None`. |
| 4 | `desproyectar(u, v, z_m, k)` | Ecuaciones del paso 4 de la sección 2.4. |
| 5 | `NodoEmbodied._publicar(...)` (Ruta B) | `PointStamped` para el primer objeto y un `TransformStamped` por objeto con `child_frame_id = objetivo_gemini_<i>` y rotación identidad. |
| 1 | `PROMPTS["propio"]` | Se escribe en la Fase 4, cuando ya conozca las escenas. |

```bash
python3 -m pytest tests -q     # meta: 15 passed
```

Las pruebas usan una **escena RGB-D sintética** (mesa a 0.80 m, caja a 0.60 m) con un punto 3D exacto conocido: `test_pipeline_recupera_la_caja` exige error < 5 mm y `test_sin_registro_falla_en_el_borde` demuestra por qué hace falta el registro. Anote el resultado en la **Tabla 1**.

---

### Fase 4: Experimento 1 — Razonamiento Semántico-Espacial

Monte (o use las capturas de) las escenas de la **Tabla 4**, tomadas de `docs/research/EXPERIMENTO_IA_LOCALIZACION_GEMINI.md` §7, más una escena de la celda *Burger Delivery*. Para cada escena:

1. Escriba **antes** de consultar cuál es la respuesta correcta (verdad de terreno) marcando el objeto en la imagen.
2. Consulte con `--tarea puntos` y la instrucción en lenguaje natural, **3 repeticiones** con `--thinking low`:
   ```bash
   python3 embodied_sin_ros.py consultar --escena semantica1 --tarea puntos --grupo G03 \
       --objeto "la caja de hamburguesa que está libre y segura para tomar desde arriba, ignorando la que tiene un objeto encima"
   ```
3. Una respuesta es **correcta** si el punto cae dentro del objeto esperado en la imagen anotada. Cuente también las respuestas con JSON inválido.
4. **TODO 1:** escriba en `PROMPTS["propio"]` un prompt para la escena *Burger Delivery* que exija razonamiento relacional (p. ej. "la caja más cercana a la bandeja del carrito") y conserve el formato de salida. Compárelo con una versión ingenua ("señala la caja").

---

### Fase 5: Experimento 2 — Exactitud 3D y Ablación del Registro

La posición absoluta de la cámara es difícil de medir con cinta, pero **dos magnitudes no dependen del marco de referencia** y sí se miden bien:

- **Distancia entre dos objetos** (centros de sus caras superiores), medida con cinta.
- **Altura de un objeto** sobre la mesa, medida con regla, frente a $Z_{mesa} - Z_{objeto}$ con la cámara mirando hacia abajo.

1. Ubique dos objetos en **5 configuraciones** distintas (cerca y lejos del centro de la imagen) y mida la distancia real $d$ entre ellos.
2. Consulte pidiendo los dos objetos a la vez; la plantilla imprime `Distancia 3D` entre los dos primeros:
   ```bash
   python3 embodied_sin_ros.py consultar --escena dist1 --tarea puntos \
       --objeto "el centro de la tapa del termo y el centro de la tapa de la caja"
   ```
3. Repita **la misma escena** con `--sin-registro` (ablación: la profundidad se reescala sin registrar).
4. Complete la **Tabla 5** con el error absoluto $|d_{est} - d_{real}|$ y el error relativo, con y sin registro.

---

### Fase 6: Experimento 3 — Costo del Razonamiento

Con **una** escena difícil de la Fase 4 (la que peor resultado dio), consulte **5 veces** por nivel:

```bash
for nivel in low high; do for i in 1 2 3 4 5; do
  python3 embodied_sin_ros.py consultar --escena semantica2 --tarea puntos --thinking $nivel --grupo G03 \
      --objeto "el celular que pertenece a la persona que dejó sus llaves al lado"
done; done
```

Los niveles `minimal` y `medium` son opcionales: si el modelo rechaza alguno, registre el error como resultado. El CSV ya contiene latencia y tokens de entrada, salida y pensamiento; complete la **Tabla 6** con media y desviación estándar por nivel y la tasa de acierto.

---

### Fase 7: Experimento 4 — Ciclo Corporizado: Agarre, Plan y Detección de Éxito

1. **Affordance:** consulte `--tarea agarre` en una escena segura y en una insegura (objeto encima, o una mano cerca). Un buen resultado devuelve un punto de agarre en la primera y **lista vacía** en la segunda.
2. **Plan:** consulte `--tarea plan --objeto "llevar la caja libre a la bandeja"`. La plantilla imprime los pasos **sin ejecutarlos**. Valide el plan con tres reglas y regístrelo en la **Tabla 7**: (a) solo usa acciones del vocabulario permitido; (b) cada `objetivo` existe en `objetos`; (c) el punto 3D del objeto cae dentro del volumen de trabajo de la mesa (defínalo en el informe).
3. **Detección de éxito:** capture la escena **antes**; una persona del grupo realiza **a mano** la tarea (o la hace mal a propósito) y capture el **después**. Pregunte:
   ```bash
   python3 embodied_sin_ros.py exito --antes exito_antes --despues exito_despues \
       --objeto "la caja libre quedó dentro de la bandeja"
   ```
   Haga al menos un caso exitoso y uno fallido.

> [!NOTE]
> **Extensión opcional (sin calificación, solo con el docente):** con la Ruta B y el TF `objetivo_gemini_0` publicado, el docente puede transformar el punto a `base_link` y enviar la pose de aproximación con los clientes seguros de `burger_kinova_reference` (Laboratorio 03). Ningún grupo envía trayectorias por su cuenta.

---

## 8. RESULTADOS DE LA PRÁCTICA

### Tabla 1: Entorno y Autoevaluación
| Ítem | Valor |
|---|---|
| Ruta elegida (A sin ROS 2 / B con ROS 2) | |
| Versión de `google-genai` y de Python | |
| Modelo usado (`gemini-robotics-er-2-preview`) y fecha de las consultas | |
| Resultado de `python3 -m pytest tests -q` (passed / failed) | |
| Commit del repositorio del grupo con los TODO resueltos (SHA) | |

### Tabla 2: Registro de Escenas Capturadas
| Escena | Fase | Descripción | % píxeles de profundidad válidos | Desfase color-profundidad (ms) |
|---|:---:|---|:---:|:---:|
| `regla` | 2 | Hoja A4 horizontal centrada, Z = ____ m | | |
| `semantica1` | 4 | Dos cajas, termo sobre una | | |
| `semantica2` | 4 | Dos celulares, llaves junto a uno | | |
| `semantica3` | 4 | Cuaderno abierto y cuaderno cerrado | | |
| `semantica4` | 4 | Esfero dentro del estuche y dos sueltos | | |
| `burger` | 4 | Escena de la celda: cajas, bandeja del carrito | | |
| `dist1` … `dist5` | 5 | Dos objetos a distancia conocida | | |
| `exito_antes` / `exito_despues` | 7 | Antes y después de la manipulación manual | | |

### Tabla 3: Calibración de la Cámara de Color
| Parámetro | Valor | Fuente |
|---|:---:|---|
| Resolución de color | | `capturas/regla_meta.json` |
| $f_x$ / $f_y$ (px) | | Kortex API / calibración rápida |
| $c_x$ / $c_y$ (px) | | |
| Campo de visión horizontal (°) | | Salida de `calibrar` |
| Distancia 3D entre esquinas de la hoja (m) | | `consultar --escena regla` |
| Error de verificación respecto a 0.297 m (%) | | |

### Tabla 4: Experimento 1 — Razonamiento Semántico-Espacial (`thinking = low`, 3 repeticiones)
| Escena | Instrucción (resumen) | Respuesta esperada | Aciertos / 3 | JSON inválidos | Latencia media (s) | Observación del error |
|---|---|---|:---:|:---:|:---:|---|
| `semantica1` | Caja libre, segura para agarre superior | | | | | |
| `semantica2` | Celular del dueño de las llaves | | | | | |
| `semantica3` | Cuaderno abierto | | | | | |
| `semantica4` | Esfero dentro del estuche | | | | | |
| `burger` (ingenuo) | "Señala la caja" | | | | | |
| `burger` (TODO 1) | Prompt propio del grupo | | | | | |

### Tabla 5: Experimento 2 — Exactitud 3D y Ablación del Registro
| Config. | $d_{real}$ (m) | $d_{est}$ con registro (m) | Error (mm) | $d_{est}$ sin registro (m) | Error (mm) | Observación |
|:---:|:---:|:---:|:---:|:---:|:---:|---|
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |
| 4 | | | | | | |
| 5 | | | | | | |
| **Media ± σ** | | | | | | |

Altura de un objeto sobre la mesa: real ______ m · estimada ($Z_{mesa} - Z_{objeto}$) ______ m · error ______ mm.

### Tabla 6: Experimento 3 — Costo del Razonamiento (5 repeticiones por nivel)
| `thinking_level` | Aciertos / 5 | Latencia media ± σ (s) | Tokens de pensamiento (media) | Tokens totales (media) | ¿El modelo aceptó el nivel? |
|:---:|:---:|:---:|:---:|:---:|:---:|
| `low` | | | | | |
| `high` | | | | | |
| `minimal` (opcional) | | | | | |
| `medium` (opcional) | | | | | |

### Tabla 7: Experimento 4 — Ciclo Corporizado
| Prueba | Escena | Resultado del modelo | Resultado esperado | ¿Correcto? | Comentario |
|---|---|---|---|:---:|---|
| Agarre en escena segura | | | Punto sobre el objeto | | |
| Agarre en escena insegura | | | Lista vacía | | |
| Plan: vocabulario permitido | | | Solo acciones válidas | | |
| Plan: objetivos existentes | | | Todos en `objetos` | | |
| Plan: punto dentro del volumen de trabajo | | | Dentro de los límites | | |
| Éxito (tarea bien hecha) | | | `exito: true` | | |
| Éxito (tarea mal hecha) | | | `exito: false` | | |

---

## 9. ANÁLISIS DE RESULTADOS

1. **Análisis 1 (Semántica):** Con la Tabla 4, ¿en qué tipo de relación (seguridad, pertenencia, estado, contención) falló más el modelo? Proponga una hipótesis y un cambio de prompt que la pruebe.
2. **Análisis 2 (Geometría):** Con la Tabla 5, descomponga el error 3D en sus fuentes: error de señalamiento de Gemini (píxeles), calibración, profundidad y registro. ¿Cuál domina? Estime cuántos milímetros produce un error de 10 px a 0.6 m con su $f_x$.
3. **Análisis 3 (Ablación):** ¿Dónde el registro importa más: en el centro o en los bordes de los objetos? Relacione la respuesta con la separación de 19.5 mm entre sensores y con la diferencia de campo de visión.
4. **Análisis 4 (Costo):** Con la Tabla 6, ¿vale la pena `high` para esta tarea? Calcule cuántas decisiones por minuto permite cada nivel y discuta por qué un VLM no puede cerrar un lazo de control de 1 kHz como `ros2_control`.
5. **Análisis 5 (Corporización):** Con la Tabla 7, ¿qué validaciones deterministas son imprescindibles entre la salida de un VLM y un actuador real?

---

## 10. CONCLUSIONES

1. Conclusión sobre la capacidad de razonamiento espacial *zero-shot* del modelo frente a un detector clásico, sustentada en la Tabla 4.
2. Conclusión cuantitativa sobre el error 3D alcanzable con la cámara del Kinova y el papel del registro y la calibración.
3. Conclusión sobre el compromiso latencia-exactitud del nivel de razonamiento y sus implicaciones de arquitectura.

---

## 11. PREGUNTAS PARA LA DISCUSIÓN

1. **Pregunta 1:** ¿Por qué la plantilla envía la imagen reducida a 1280 px y aun así ubica el punto en la imagen de 1920×1080 sin pérdida de exactitud geométrica? ¿Qué sí se pierde al reducirla?
2. **Pregunta 2:** `profundidad_robusta` usa la mediana y no el promedio. Construya un ejemplo numérico de 9 píxeles con un reflejo en el que el promedio falle.
3. **Pregunta 3:** ¿Qué pasa con el punto 3D si Gemini señala el borde de la caja en lugar de su centro? ¿Cómo lo detectaría automáticamente con la imagen de profundidad?
4. **Pregunta 4:** La salida está en el marco **óptico** de la cámara. ¿Qué transformación falta para expresarla en `base_link`, y de qué depende que esa transformación sea correcta mientras el brazo se mueve?
5. **Pregunta 5 (ética):** ¿Qué información de la escena podría revelar una imagen enviada a un servicio externo aunque no aparezcan personas? ¿Qué alternativa técnica existiría para no enviar imágenes fuera del laboratorio?

---

## 12. BIBLIOGRAFÍA

1. Google DeepMind. (2026). *Gemini Robotics ER 2 — Model card.* https://deepmind.google/models/model-cards/gemini-robotics-er-2/
2. Google. (2026). *Gemini Robotics ER — Gemini API documentation.* https://ai.google.dev/gemini-api/docs/robotics-overview
3. Kinova Robotics. (2024). *ros2_kortex_vision: ROS 2 driver for the Kinova Gen3 vision module.* https://github.com/Kinovarobotics/ros2_kortex_vision
4. Kinova Robotics. (2024). *Kinova Gen3 Ultra lightweight robot User Guide.* Kinova Inc.
5. Hartley, R., & Zisserman, A. (2004). *Multiple View Geometry in Computer Vision* (2nd ed.). Cambridge University Press.
6. Russell, S., & Norvig, P. (2021). *Artificial Intelligence: A Modern Approach* (4th ed.). Pearson.

---

## 13. APROBACIÓN DE LA GUÍA DE LABORATORIO

| Elaborado por: | Revisado por: | Aprobado por: |
|:---:|:---:|:---:|
| **Ing. Henry Roncancio**<br>Docente Asignatura Inteligencia Artificial | **Director de Programa**<br>Ingeniería Mecatrónica | **Decano(a)**<br>Facultad de Ingeniería |
