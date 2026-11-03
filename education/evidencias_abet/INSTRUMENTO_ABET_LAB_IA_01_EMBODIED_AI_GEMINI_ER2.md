# Instrumento de Evidencia y Calificación — Laboratorio IA 01: Embodied AI con Gemini Robotics-ER 2 y la Cámara RGB-D del Kinova Gen3

> **Documento Único de Entrega y Evaluación ABET.** Asociado a `education/guias_laboratorio/GUIA_LAB_IA_01_EMBODIED_AI_GEMINI_ER2_KINOVA.docx`.
> Este documento reúne la entrega integral del equipo: registro de evidencias, tablas de datos experimentales, respuestas al cuestionario de análisis técnico, declaración de uso de IA generativa, Anexo A de comprobación individual y las rúbricas analíticas para la calificación docente y la consolidación ABET.

---

## 1. Identificación y Control

| Campo | Registro Oficial |
|---|---|
| **Programa Académico** | Ingeniería Mecatrónica |
| **Asignatura** | INTELIGENCIA ARTIFICIAL |
| **Periodo Académico** | 2026-2 |
| **Corte / Instrumento** | Tercer Corte / Laboratorios y evidencias experimentales |
| **Actividad Evaluada** | Laboratorio IA 01 — Embodied AI: razonamiento espacial con Gemini Robotics-ER 2 y localización 3D con la cámara RGB-D del Kinova Gen3 |
| **Número de Grupo / Subgrupo** | |
| **Estudiante 1 (Nombre y Código)** | |
| **Estudiante 2 (Nombre y Código)** | |
| **Estudiante 3 (si aplica)** | |
| **Ruta del Equipo** | Ruta A — sin ROS 2 ☐ &nbsp;&nbsp;&nbsp;&nbsp; Ruta B — con ROS 2 ☐ |
| **Repositorio y Commit con los TODO Resueltos (SHA)** | |
| **Nombre del Archivo de Entrega** | `C3_LIA01_G<grupo>_<codigo1>_<codigo2>_v1.docx` |
| **Anexos Obligatorios del Envío** | `resultados.csv` y las imágenes `capturas/*_resultado.png` citadas en el documento |
| **Fecha de Realización en Laboratorio** | |
| **Fecha de Entrega del Documento** | |
| **Docente Evaluador** | Ing. Henry Roncancio |
| **Versión del Instrumento** | Versión 1.0 (2026-2) |
| **Unidad de Análisis / Captura** | Equipo colaborativo con comprobación individual (Anexo A) |

---

## 2. Parámetros de Assessment

| Parámetro | Regla Institucional y Metodológica Adoptada |
|---|---|
| **Población o cohorte** | Censo completo de estudiantes matriculados en Inteligencia Artificial que presentan el Laboratorio IA 01 en el periodo 2026-2. |
| **Momento de medición** | Sesión experimental presencial (captura de escenas) y entrega documental única con tablas llenas, `resultados.csv`, imágenes anotadas y código en el repositorio del equipo. |
| **Evaluador** | Docente titular de la asignatura INTELIGENCIA ARTIFICIAL. |
| **Umbral individual de logro** | Nivel **N3 o superior** (puntaje mínimo de 300 sobre 500) en cada uno de los indicadores de desempeño evaluados. |
| **Meta de cohorte** | Al menos el **70% de los estudiantes evaluables** debe alcanzar el nivel N3 o superior en cada indicador. |
| **Regla de muestreo** | Censo 100%: no se utiliza muestreo; se evalúa la totalidad de los estudiantes y equipos exigibles. |
| **Evidencia faltante** | Entrega exigible sin evidencia obligatoria verificable: se califica en N1 con puntaje 0. Retiro oficial o causa justificada autorizada: `NA / no evaluado` (se excluye del denominador ABET). |
| **Regla de trabajo colaborativo** | Informe único por equipo complementado obligatoriamente con el Anexo A de comprobación individual. Sin comprobación individual suficiente no se infiere logro individual ABET. |
| **Regla de datos** | Toda cifra de las tablas debe poder rastrearse a una fila de `resultados.csv` o a una medición física registrada. Un dato que no pueda rastrearse se considera evidencia no verificable. |

---

## 3. Niveles de Desempeño para Zubatronic/SGDE

| Nivel | Intervalo Zubatronic | Valor Guía | Interpretación y Criterio de Logro |
|---|---:|:---:|---|
| **N5** | 475–500 | 500 | **Excelente:** Evidencia completa, rigurosa, reproducible y explicada con profundidad analítica y matemática sobresaliente. |
| **N4** | 400–474 | 450 | **Bueno:** Desempeño técnico correcto con omisiones menores que no comprometen la validez de las mediciones, la trazabilidad ni la seguridad. |
| **N3** | 300–399 | 350 | **Aceptable:** Demuestra el desempeño esencial con evidencia verificable. **Es el umbral individual de logro.** |
| **N2** | 150–299 | 250 | **Cumplimiento parcial:** Evidencia incompleta, métricas faltantes, pruebas sin repeticiones o conclusiones sin sustento en los datos. |
| **N1** | 0–149 | 100 | **No cumple:** Evidencia mínima, fragmentaria o no funcional. Sin evidencia obligatoria se registra 0. |

---

## 4. Alineación de Criterios, RAE y Student Outcomes

| Criterio | Peso | Student Outcome Principal | Indicador de Desempeño del Programa | Evidencia Directa Obligatoria |
|---|:---:|:---:|---|---|
| **C1. Implementación del pipeline percepción → razonamiento → geometría** | 30% | **SO1** | **1.1.** Aplica principios matemáticos (matrices de transformación homogénea, rotaciones y cuaterniones) para resolver problemas espaciales en robótica. | TODO 2 a 4 (y 5 en la Ruta B) implementados, `python3 -m pytest tests -q` con 15 aprobadas, commit identificado por SHA y `resultados.csv` generado por el código del equipo (E1, E5). |
| **C2. Diseño experimental y medición** | 25% | **SO6** | **6.1.** Diseña y ejecuta pruebas experimentales con verdad de terreno medida, repeticiones y variables controladas para cuantificar exactitud, latencia y repetibilidad. | Escenas registradas (Tabla 2), calibración verificada con error < 5% (Tabla 3), 3 repeticiones por escena semántica, 5 configuraciones de distancia y 5 repeticiones por nivel de razonamiento (Tablas 4 a 6). |
| **C3. Análisis crítico del modelo** | 20% | **SO6** | **6.4.** Interpreta fallas y diagnósticos experimentales aplicando protocolos por capas para aislar errores en hardware y software. | Descomposición del error 3D por fuentes (señalamiento, calibración, profundidad, registro), ablación con y sin registro (Tabla 5), análisis de fallas semánticas del VLM y compromiso latencia-exactitud (Preguntas 1 a 5 y 9). |
| **C4. Informe técnico y comunicación de resultados** | 15% | **SO3** | **3.1 - 3.3.** Elabora documentación técnica reproducible y comunica resultados experimentales con métricas y demostraciones. | Instrumento completo, imágenes anotadas de aciertos y fallos (E4), tablas con media y desviación estándar, conclusiones sustentadas en datos (Pregunta 10) y anexos trazables. |
| **C5. Uso responsable de IA, seguridad y trabajo en equipo** | 10% | **SO4 / SO5** | **4.3.** Aplica buenas prácticas en el uso de datos de cámara, operación segura y roles de equipo.<br>**5.1.** Define roles técnicos y coordina la ejecución colaborativa en equipo. | Clave de API fuera del repositorio, escenas sin personas ni documentos, respeto de la zona del robot y de los turnos RTSP, Tabla 8 (declaración de IA generativa) y Anexo A individual. |
| **TOTAL** | **100%** | | | |

> **Nota de alineación:** los indicadores 1.1, 3.1 - 3.3, 4.3, 5.1 y 6.4 se citan como aparecen en los instrumentos del programa usados en las demás prácticas del repositorio (en 1.1, el modelo pinhole y el registro de profundidad son las transformaciones evaluadas). El indicador 6.1 se redactó para el contexto de un modelo de IA. Antes de cargar las notas, verifique los códigos contra el syllabus vigente de Inteligencia Artificial.

---

## 5. Registro de Evidencias de la Práctica (E1–E9)

| Código | Evidencia Requerida | Localizador en el Documento / Repositorio / Archivo |
|---|---|---|
| **E1** | Captura de `python3 -m pytest tests -q` con **15 passed** y enlace al commit (SHA) del repositorio del equipo con los TODO resueltos. | |
| **E2** | Archivos `capturas/<escena>_meta.json` de todas las escenas (Ruta A) o salida de `ros2 topic hz` de color y profundidad más las escenas guardadas con la tarea `guardar` (Ruta B). | |
| **E3** | Salida del comando `calibrar` (o JSON leído con la Kortex API) y salida de `consultar --escena regla` con la distancia 3D entre las esquinas de la hoja. | |
| **E4** | Imágenes anotadas `capturas/*_resultado.png` de las escenas semánticas: como mínimo un acierto y un fallo del modelo. | |
| **E5** | Archivo `resultados.csv` completo generado por la plantilla (columnas de latencia, tokens, píxel y punto 3D). | |
| **E6** | Imágenes anotadas de una misma escena de distancia con registro y sin registro (`--sin-registro`). | |
| **E7** | Filas del CSV de las 5 repeticiones por nivel de razonamiento (`thinking = low` y `high`, como mínimo). | |
| **E8** | Salida de la tarea `plan` (pasos impresos sin ejecutar), imágenes de agarre seguro e inseguro, y capturas antes/después con la respuesta de `exito` para un caso exitoso y uno fallido. | |
| **E9** | Texto del prompt propio (`PROMPTS["propio"]`, TODO 1) y de la versión ingenua con la que se comparó. | |

---

## 6. Tablas de Registro Experimental (Diligenciadas por el Equipo)

> **Instrucciones para el equipo:** Diligencie de forma completa y precisa cada una de las siguientes tablas con los datos reales obtenidos. Toda cifra debe coincidir con `resultados.csv` o con una medición física anotada en la sesión.

#### Diseño experimental: qué se varía y qué se mide

Las Tablas 1 a 3 registran el entorno, las capturas y la calibración: no son experimentos, pero los experimentos dependen de ellas. Las Tablas 4 a 7 son los cuatro experimentos. Para leerlas conviene saber qué cambia el grupo a propósito en cada uno y qué mide:

- **Factor:** la condición que el grupo cambia a propósito para observar su efecto.
- **Niveles:** los valores que toma el factor en el experimento.
- **Unidad experimental:** sobre qué se hace cada medición: una escena capturada o una configuración de objetos.
- **Repeticiones:** cuántas veces se consulta el modelo con el mismo nivel sobre la misma unidad. Gemini no responde siempre igual a la misma pregunta; por eso se repite y se reporta el número de aciertos, no un solo intento.
- **Variable de respuesta:** lo que se mide y se anota en la tabla.
- **Se mantiene fijo:** lo que no debe cambiar entre mediciones para que la comparación sea justa.

| Experimento (tabla) | Factor | Niveles | Unidad experimental | Repeticiones | Variable de respuesta | Se mantiene fijo |
|---|---|---|---|:---:|---|---|
| 1. Razonamiento semántico-espacial (Tabla 4) | La escena y su instrucción; en `burger`, la redacción del prompt | 4 escenas `semantica1` a `semantica4`; `burger` con el prompt ingenuo y con el prompt propio (TODO 1) | Una escena capturada | 3 por escena | Aciertos de 3, respuestas con JSON inválido, latencia | `--thinking low` y la misma captura en las 3 repeticiones |
| 2. Exactitud 3D y ablación (Tabla 5) | El registro de la profundidad al marco de color | Con registro (por defecto) y sin registro (`--sin-registro`) | Una configuración de dos objetos: `dist1` a `dist5` | 1 por nivel y configuración | Error \|d_est − d_real\| en mm | La misma captura y la misma instrucción en los dos niveles |
| 3. Costo del razonamiento (Tabla 6) | El nivel de razonamiento `thinking_level` | `low` y `high` (obligatorios); `minimal` y `medium` (opcionales) | La escena con peor resultado en la Tabla 4 | 5 por nivel | Aciertos de 5, latencia media ± σ, tokens | La escena y la instrucción |
| 4. Ciclo corporizado (Tabla 7) | El tipo de prueba | Agarre en escena segura e insegura, las 3 reglas del plan, éxito con la tarea bien y mal hecha | Una escena por prueba | 1 | ¿Correcto? (sí o no) | La instrucción de cada prueba y el nivel de razonamiento |


### Tabla 1: Entorno y Autoevaluación

Registra con qué se hizo la práctica, para que otro grupo pueda reproducirla. Cada fila es un dato:

- **Ruta elegida:** A, si capturó con `embodied_sin_ros.py`; B, si usó el nodo ROS 2.
- **Versiones:** salida de `pip show google-genai` (línea `Version`) y de `python3 --version`.
- **Modelo y fecha:** el modelo está en *preview*: con la fecha se sabe con qué versión se obtuvieron los resultados.
- **Resultado de `pytest`:** la línea final, por ejemplo `15 passed`. Con algún `failed`, indique cuál prueba falla.
- **Commit (SHA):** salida de `git rev-parse --short HEAD` en el repositorio del grupo, con los TODO ya resueltos.


| Ítem | Valor Registrado |
|---|---|
| **Ruta elegida (A sin ROS 2 / B con ROS 2)** | |
| **Versión de `google-genai` y de Python** | |
| **Modelo usado y fecha de las consultas** | `gemini-robotics-er-2-preview` — |
| **Resultado de `python3 -m pytest tests -q` (passed / failed)** | |
| **Commit del repositorio del equipo con los TODO resueltos (SHA)** | |

### Tabla 2: Registro de Escenas Capturadas

Una **escena** es una captura con nombre: la imagen de color, la de profundidad de 16 bits y un archivo `_meta.json`, guardados en `capturas/` con ese nombre. Es el nombre que después se pasa a `--escena`. Cada fila propuesta es una escena que necesitará más adelante; capture todas en su turno de cámara.

- **Escena:** el nombre exacto de la captura. Use los propuestos: el CSV y las demás tablas se cruzan por ese nombre.
- **Fase:** la fase de esta guía en la que se usa la escena: 2 (calibración), 4 (razonamiento), 5 (exactitud 3D) o 7 (ciclo corporizado).
- **Descripción:** qué objetos hay y cómo están dispuestos. Respete la disposición propuesta: cada escena pone a prueba una capacidad distinta del modelo (lista de abajo).
- **% píxeles de profundidad válidos:** `pixeles_validos_pct` del `_meta.json`: porcentaje de píxeles con profundidad distinta de 0. Por debajo de 60 % la escena se repite; las superficies negras o brillantes devuelven 0.
- **Desfase color-profundidad (ms):** `desfase_color_profundidad_ms` del `_meta.json`: tiempo entre la toma de color y la de profundidad. Si algo se movió en ese intervalo, las dos imágenes no muestran lo mismo.

Qué pone a prueba cada escena propuesta:

- `regla`: una hoja A4 (0.297 m de ancho) a una distancia Z medida con cinta, para calibrar la cámara (Fase 2).
- `semantica1`: el **estado físico**: una caja tiene algo encima y no se puede tomar desde arriba.
- `semantica2`: una **relación** entre objetos: de quién es el celular según dónde están las llaves.
- `semantica3`: el **estado** de un objeto: abierto o cerrado.
- `semantica4`: la **contención**: dentro o fuera del estuche.
- `burger`: la escena real de la celda, con la bandeja del carrito; es la del prompt propio (TODO 1).
- `dist1` a `dist5`: cinco disposiciones de dos objetos con la distancia entre ellos medida con cinta; alterne posiciones cerca y lejos del centro de la imagen.
- `exito_antes` y `exito_despues`: la misma escena antes y después de hacer la tarea a mano.


| Escena | Fase | Descripción | % Píxeles de Profundidad Válidos | Desfase Color-Profundidad (ms) |
|---|:---:|---|:---:|:---:|
| `regla` | 2 | Hoja A4 horizontal centrada, Z = ____ m | | |
| `semantica1` | 4 | Dos cajas, termo sobre una | | |
| `semantica2` | 4 | Dos celulares, llaves junto a uno | | |
| `semantica3` | 4 | Cuaderno abierto y cuaderno cerrado | | |
| `semantica4` | 4 | Esfero dentro del estuche y dos sueltos | | |
| `burger` | 4 | Escena de la celda: cajas y bandeja del carrito | | |
| `dist1` … `dist5` | 5 | Dos objetos a distancia conocida | | |
| `exito_antes` / `exito_despues` | 7 | Antes y después de la manipulación manual | | |

### Tabla 3: Calibración de la Cámara de Color

Registra los parámetros de la cámara de color con los que se desproyectan todos los puntos 3D, y la prueba que los valida. Cada fila es un parámetro:

- **Resolución de color:** ancho × alto en píxeles de las capturas (en el `_meta.json`).
- **f_x / f_y (px):** distancias focales en píxeles. Con la calibración rápida, f_y se toma igual a f_x.
- **c_x / c_y (px):** el punto principal, donde el eje óptico corta la imagen. En la calibración rápida es el centro de la imagen.
- **Campo de visión horizontal (°):** lo informa el comando `calibrar`; sirve para comprobar que f_x es razonable.
- **Distancia 3D entre esquinas de la hoja (m):** la que calcula `consultar --escena regla` con los puntos de Gemini.
- **Error de verificación (%):** |distancia − 0.297| / 0.297 × 100. Por encima de 5 %, repita la calibración antes de seguir.

La columna **Fuente** indica de dónde sale cada valor: la Kortex API (calibración oficial del docente), la calibración rápida del grupo o la salida de un comando.


| Parámetro | Valor | Fuente |
|---|:---:|---|
| **Resolución de color** | | `capturas/regla_meta.json` |
| **fx / fy (px)** | | Kortex API / calibración rápida |
| **cx / cy (px)** | | |
| **Campo de visión horizontal (°)** | | Salida de `calibrar` |
| **Distancia 3D entre esquinas de la hoja (m)** | | `consultar --escena regla` |
| **Error de verificación respecto a 0.297 m (%)** | | |

### Tabla 4: Experimento 1 — Razonamiento Semántico-Espacial (`thinking = low`, 3 repeticiones)

Cada fila es una escena; el experimento compara qué tan bien razona el modelo en cada tipo de pregunta (ver el diseño experimental al inicio de esta sección).

- **Escena:** la captura usada (Tabla 2). `burger` aparece dos veces porque se consulta con el prompt ingenuo y con el prompt propio del grupo.
- **Instrucción (resumen):** la pregunta en pocas palabras; la instrucción completa queda en el CSV, columna `objeto`.
- **Respuesta esperada:** el objeto correcto, decidido por el grupo **antes** de consultar (verdad de terreno).
- **Aciertos / 3:** cuántas de las 3 repeticiones pusieron el punto dentro del objeto esperado en la imagen anotada.
- **JSON inválidos:** cuántas respuestas no se pudieron interpretar como la lista de puntos pedida.
- **Latencia media (s):** promedio de la columna `latencia_s` del CSV para esas 3 consultas.
- **Observación del error:** qué señaló el modelo cuando falló y una hipótesis del porqué.


| Escena | Instrucción (resumen) | Respuesta Esperada | Aciertos / 3 | JSON Inválidos | Latencia Media (s) | Observación del Error |
|---|---|---|:---:|:---:|:---:|---|
| `semantica1` | Caja libre, segura para agarre superior | | | | | |
| `semantica2` | Celular del dueño de las llaves | | | | | |
| `semantica3` | Cuaderno abierto | | | | | |
| `semantica4` | Esfero dentro del estuche | | | | | |
| `burger` (ingenuo) | "Señala la caja" | | | | | |
| `burger` (TODO 1) | Prompt propio del equipo | | | | | |

### Tabla 5: Experimento 2 — Exactitud 3D y Ablación del Registro de Profundidad

Cada fila es una **configuración**: una disposición de dos objetos (`dist1` a `dist5`) con la distancia real entre ellos medida con cinta. Cada configuración se consulta dos veces sobre la **misma captura**: con registro de la profundidad y sin él (la ablación). Comparar las dos columnas de error muestra cuánto aporta el registro.

- **Config.:** el número de la configuración (1 = `dist1`, etc.).
- **d_real (m):** distancia medida con cinta entre los centros de las caras superiores de los dos objetos.
- **d_est con registro (m):** la `Distancia 3D` que imprime `consultar`, con el registro activo (por defecto).
- **d_est sin registro (m):** la misma consulta con `--sin-registro`.
- **Error (mm):** |d_est − d_real| × 1000 para cada caso.
- **Observación:** por ejemplo, si un objeto quedó en el borde de la imagen o sobre una superficie brillante.
- **Media ± σ:** promedio y desviación estándar de cada columna sobre las 5 configuraciones. En la columna **Observación** de esta fila anote el **error relativo medio** con y sin registro: el promedio de error / d_real, en %.


| Configuración | d real (m) | d estimada con registro (m) | Error (mm) | d estimada sin registro (m) | Error (mm) | Observación |
|:---:|:---:|:---:|:---:|:---:|:---:|---|
| **1** | | | | | | |
| **2** | | | | | | |
| **3** | | | | | | |
| **4** | | | | | | |
| **5** | | | | | | |
| **Media ± σ** | | | | | | |

Altura de un objeto sobre la mesa: real ______ m · estimada (Z mesa − Z objeto) ______ m · error ______ mm.

### Tabla 6: Experimento 3 — Costo del Razonamiento (5 repeticiones por nivel)

Cada fila es un **nivel** del factor `thinking_level` aplicado a la misma escena y la misma instrucción, 5 veces. El experimento mide qué se gana en aciertos y qué se paga en tiempo y tokens al dejar razonar más al modelo.

- **`thinking_level`:** el nivel pasado con `--thinking`. `low` y `high` son obligatorios; `minimal` y `medium`, opcionales.
- **Aciertos / 5:** cuántas de las 5 respuestas señalaron el objeto correcto.
- **Latencia media ± σ (s):** promedio y desviación estándar de `latencia_s` en el CSV.
- **Tokens de pensamiento (media):** promedio de `tokens_pensamiento`: lo que el modelo "pensó" antes de responder.
- **Tokens totales (media):** promedio de `tokens_entrada + tokens_salida + tokens_pensamiento`.
- **¿El modelo aceptó el nivel?:** «no» si la API devolvió un error para ese nivel; ese error también es un resultado.


| `thinking_level` | Aciertos / 5 | Latencia Media ± σ (s) | Tokens de Pensamiento (media) | Tokens Totales (media) | ¿El Modelo Aceptó el Nivel? |
|:---:|:---:|:---:|:---:|:---:|:---:|
| `low` | | | | | |
| `high` | | | | | |
| `minimal` (opcional) | | | | | |
| `medium` (opcional) | | | | | |

### Tabla 7: Experimento 4 — Ciclo Corporizado

Cada fila es una **prueba** del ciclo corporizado con su resultado esperado ya escrito: el grupo compara lo que respondió el modelo con ese resultado.

- **Prueba:** qué se evalúa. Las de agarre miden si el modelo reconoce cuándo **no** es seguro tomar un objeto. Las del plan aplican las tres reglas de la Fase 7. Las de éxito comparan una captura de antes con una de después.
- **Escena:** la captura usada (Tabla 2).
- **Resultado del modelo:** lo que devolvió: un punto, una lista vacía, los pasos del plan o el valor de `exito`.
- **Resultado esperado:** ya propuesto en cada fila. «Lista vacía» significa que el modelo se abstiene de proponer un agarre inseguro.
- **¿Correcto?:** sí, si el resultado del modelo coincide con el esperado.
- **Comentario:** la causa de un error, o el volumen de trabajo usado para la regla del punto dentro de los límites.


| Prueba | Escena | Resultado del Modelo | Resultado Esperado | ¿Correcto? | Comentario |
|---|---|---|---|:---:|---|
| **Agarre en escena segura** | | | Punto sobre el objeto | | |
| **Agarre en escena insegura** | | | Lista vacía | | |
| **Plan: vocabulario permitido** | | | Solo acciones válidas | | |
| **Plan: objetivos existentes** | | | Todos en `objetos` | | |
| **Plan: punto dentro del volumen de trabajo** | | | Dentro de los límites | | |
| **Éxito (tarea bien hecha)** | | | `exito: true` | | |
| **Éxito (tarea mal hecha)** | | | `exito: false` | | |

### Tabla 8: Declaración de Uso de Herramientas de IA Generativa

| Herramienta | Para qué se usó (código, redacción, depuración…) | Cómo se verificó el resultado |
|---|---|---|
| **Gemini Robotics-ER 2** | Objeto de estudio de la práctica | Tablas 4 a 7 |
| | | |
| | | |

---

## 7. Cuestionario de Análisis y Discusión Técnica (Respondido por el Equipo)

### Pregunta 1: Fallas Semánticas del Modelo
*Con base en la Tabla 4, ¿en qué tipo de relación (seguridad, pertenencia, estado o contención) falló más el modelo? Formule una hipótesis sobre la causa y proponga un cambio de prompt que la ponga a prueba. Compare el prompt propio (TODO 1) con la versión ingenua.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 2: Descomposición del Error 3D por Fuentes
*Con la Tabla 5, descomponga el error 3D en sus fuentes: error de señalamiento de Gemini (píxeles), calibración, profundidad y registro. ¿Cuál domina? Estime cuántos milímetros produce un error de 10 px a 0.6 m con la fx de la Tabla 3.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 3: Ablación del Registro de Profundidad
*¿Dónde importa más el registro: en el centro o en los bordes de los objetos? Relacione la respuesta con la separación de 19.5 mm entre los sensores y con la diferencia de campo de visión entre la cámara de color y la de profundidad.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 4: Costo del Razonamiento y Arquitectura
*Con la Tabla 6, ¿vale la pena `thinking = high` para esta tarea? Calcule cuántas decisiones por minuto permite cada nivel y explique por qué un VLM no puede cerrar un lazo de control de 1 kHz como `ros2_control`.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 5: Validaciones entre el VLM y el Actuador
*Con la Tabla 7, ¿qué validaciones deterministas son imprescindibles entre la salida de un VLM y un actuador real? Indique cuáles detectaron errores en su práctica y cuáles faltarían para ejecutar el plan en el robot.*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 6: Resolución de la Imagen Enviada a la API
*La plantilla reduce la imagen a 1280 px antes de enviarla y aun así ubica el punto en la imagen de 1920×1080. ¿Por qué no se pierde exactitud geométrica en la conversión de coordenadas? ¿Qué sí se pierde al reducirla?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 7: Del Marco Óptico a la Base del Robot
*El punto 3D está en el marco óptico de la cámara de color. ¿Qué transformación falta para expresarlo en `base_link` y de qué depende que esa transformación sea correcta mientras el brazo se mueve?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 8: Ética y Privacidad de los Datos Enviados
*¿Qué información de la escena podría revelar una imagen enviada a un servicio externo aunque no aparezcan personas? ¿Qué alternativa técnica permitiría no enviar imágenes fuera del laboratorio y qué se perdería con ella?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 9: Punto en el Borde del Objeto
*¿Qué pasa con el punto 3D si Gemini señala el borde de la caja en lugar de su centro? ¿Cómo lo detectaría automáticamente con la imagen de profundidad?*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

### Pregunta 10: Conclusiones del Equipo
*Redacte tres conclusiones, cada una sustentada en sus datos: (1) la capacidad de razonamiento espacial *zero-shot* del modelo frente a un detector clásico (Tabla 4); (2) el error 3D alcanzable con la cámara del Kinova y el papel del registro y la calibración (Tablas 3 y 5); y (3) el compromiso latencia-exactitud del nivel de razonamiento y sus implicaciones de arquitectura (Tabla 6).*

> **Respuesta Técnica del Equipo:**
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________
> ________________________________________________________________________________________________________________________

---

## 8. Anexo A — Comprobación Individual de Desempeño

> **Propósito ABET:** Este anexo verifica la autoría, comprensión técnica y contribución directa individual de cada estudiante, sustentando la asignación del logro individual en los Student Outcomes evaluados.

### Estudiante 1: __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas (captura de escenas, calibración, implementación de un TODO, diseño de prompts, análisis de datos o redacción). | |
| **2. Conversión de coordenadas:** Gemini responde `[y, x] = [250, 750]` sobre una imagen de 1920×1080. Calcule (u, v) y explique por qué el orden importa. | |
| **3. Profundidad robusta:** Construya un ejemplo de 9 píxeles de profundidad (en mm) con un reflejo en el que el promedio falle y la mediana no. | |
| **4. Falla observada del modelo:** Describa un fallo concreto de Gemini en su práctica (escena, instrucción, respuesta) y explique su causa más probable. | |
| **5. Autoría y reproducibilidad:** Identifique qué commits, funciones, filas del CSV, escenas o secciones de este documento produjo directamente usted. | |

**Estado de comprobación individual Estudiante 1:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐

**Firma del Estudiante 1:** ________________________________________

---

### Estudiante 2: __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas (captura de escenas, calibración, implementación de un TODO, diseño de prompts, análisis de datos o redacción). | |
| **2. Conversión de coordenadas:** Gemini responde `[y, x] = [250, 750]` sobre una imagen de 1920×1080. Calcule (u, v) y explique por qué el orden importa. | |
| **3. Profundidad robusta:** Construya un ejemplo de 9 píxeles de profundidad (en mm) con un reflejo en el que el promedio falle y la mediana no. | |
| **4. Falla observada del modelo:** Describa un fallo concreto de Gemini en su práctica (escena, instrucción, respuesta) y explique su causa más probable. | |
| **5. Autoría y reproducibilidad:** Identifique qué commits, funciones, filas del CSV, escenas o secciones de este documento produjo directamente usted. | |

**Estado de comprobación individual Estudiante 2:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐

**Firma del Estudiante 2:** ________________________________________

---

### Estudiante 3 (si aplica): __________________________________________________ Código: ____________________

| Pregunta Individual de Verificación | Respuesta y Evidencia Directa del Estudiante |
|---|---|
| **1. Rol y tareas técnicas desarrolladas:** Describa sus responsabilidades específicas (captura de escenas, calibración, implementación de un TODO, diseño de prompts, análisis de datos o redacción). | |
| **2. Conversión de coordenadas:** Gemini responde `[y, x] = [250, 750]` sobre una imagen de 1920×1080. Calcule (u, v) y explique por qué el orden importa. | |
| **3. Profundidad robusta:** Construya un ejemplo de 9 píxeles de profundidad (en mm) con un reflejo en el que el promedio falle y la mediana no. | |
| **4. Falla observada del modelo:** Describa un fallo concreto de Gemini en su práctica (escena, instrucción, respuesta) y explique su causa más probable. | |
| **5. Autoría y reproducibilidad:** Identifique qué commits, funciones, filas del CSV, escenas o secciones de este documento produjo directamente usted. | |

**Estado de comprobación individual Estudiante 3:** Verificada ☐ &nbsp;&nbsp;&nbsp; Insuficiente ☐ &nbsp;&nbsp;&nbsp; NA autorizado ☐

**Firma del Estudiante 3:** ________________________________________

---

## 9. Selección del Nivel Alcanzado por Criterio (Rúbricas Docentes)

Marque con una **X** una sola casilla por criterio y registre el valor entero (0 a 500) dentro de la banda correspondiente.

### C1. Implementación del pipeline percepción → razonamiento → geometría — Peso 30% — Student Outcome SO1

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, extiende el pipeline con una mejora verificable (p. ej. rechazo de puntos en bordes con la profundidad, intrínsecos con distorsión o validación automática del JSON) y la justifica matemáticamente. |
| ☐ | **N4 — 400–474** | Implementa correctamente los TODO 2 a 4 (y 5 en Ruta B), 15 pruebas aprobadas, código legible en un commit identificado y `resultados.csv` generado sin edición manual. |
| ☐ | **N3 — 300–399** | Implementa los TODO con las pruebas esenciales aprobadas (conversión de coordenadas y desproyección) y obtiene puntos 3D en al menos una escena real. |
| ☐ | **N2 — 150–299** | Implementación incompleta: pruebas fallidas, confusión del orden `[y, x]` o profundidad sin conversión a metros; resultados obtenidos con la respuesta escrita a mano. |
| ☐ | **N1 — 0–149** | No implementa los TODO, no aporta commit verificable o carece de evidencias obligatorias. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C2. Diseño experimental y medición — Peso 25% — Student Outcome SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, controla variables adicionales (iluminación, posición en la imagen, tamaño del objeto) o amplía las repeticiones, e informa incertidumbre de la verdad de terreno. |
| ☐ | **N4 — 400–474** | Ejecuta el protocolo completo: calibración verificada con error < 5%, 3 repeticiones por escena semántica, 5 configuraciones de distancia con y sin registro y 5 repeticiones por nivel de razonamiento; Tablas 2 a 7 completas y trazables al CSV. |
| ☐ | **N3 — 300–399** | Ejecuta las fases 4 a 6 con verdad de terreno medida y el número mínimo de repeticiones, con omisiones menores en las tablas. |
| ☐ | **N2 — 150–299** | Mediciones sin repeticiones, sin verdad de terreno o sin verificar la calibración; tablas incompletas o no trazables al CSV. |
| ☐ | **N1 — 0–149** | No ejecuta los experimentos o presenta datos no verificables. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C3. Análisis crítico del modelo — Peso 20% — Student Outcome SO6

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, cuantifica la contribución de cada fuente de error con cálculos propios, contrasta hipótesis sobre las fallas del VLM con experimentos adicionales y propone una arquitectura justificada en los datos. |
| ☐ | **N4 — 400–474** | Descompone el error 3D por fuentes, interpreta la ablación del registro, explica las fallas semánticas observadas y discute el compromiso latencia-exactitud con media y desviación estándar. |
| ☐ | **N3 — 300–399** | Responde las preguntas de análisis con referencia explícita a las tablas e identifica la fuente dominante del error. |
| ☐ | **N2 — 150–299** | Análisis descriptivo o genérico, sin relación con los datos propios, o conclusiones que contradicen las tablas. |
| ☐ | **N1 — 0–149** | Sin análisis o con análisis ajeno a los resultados obtenidos. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C4. Informe técnico y comunicación de resultados — Peso 15% — Student Outcome SO3

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, presenta figuras de calidad de publicación (p. ej. error frente a posición en la imagen o latencia por nivel) y un documento que otro equipo podría reproducir sin ayuda. |
| ☐ | **N4 — 400–474** | Instrumento completo, evidencias E1–E9 localizadas, imágenes anotadas de aciertos y fallos, unidades correctas y conclusiones sustentadas en los datos. |
| ☐ | **N3 — 300–399** | Instrumento completo con las evidencias obligatorias y conclusiones coherentes con los datos. |
| ☐ | **N2 — 150–299** | Documento incompleto, evidencias no localizables, tablas sin unidades o conclusiones sin sustento. |
| ☐ | **N1 — 0–149** | No entrega el documento o entrega evidencia fragmentaria. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

### C5. Uso responsable de IA, seguridad y trabajo en equipo — Peso 10% — Student Outcome SO4 / SO5

| Marque | Nivel | Evidencia Observable del Nivel |
|:---:|---|---|
| ☐ | **N5 — 475–500** | Además de N4, propone controles concretos de privacidad o seguridad (anonimización, procesamiento local, validaciones antes del actuador) y el Anexo A muestra complementariedad sobresaliente de roles. |
| ☐ | **N4 — 400–474** | Clave fuera del repositorio, escenas sin personas ni documentos, respeto de turnos RTSP y de la zona del robot, Tabla 8 completa y Anexo A con evidencia individual verificable. |
| ☐ | **N3 — 300–399** | Cumple las reglas de seguridad y privacidad de la guía, declara el uso de IA generativa y aporta el Anexo A con respuestas verificables. |
| ☐ | **N2 — 150–299** | Declaración de IA incompleta, Anexo A con respuestas genéricas o incumplimiento menor de turnos o zona de trabajo. |
| ☐ | **N1 — 0–149** | Publica la clave de API, envía imágenes de personas o documentos, incurre en actos inseguros o el Anexo A demuestra ausencia de participación. |

**Nivel C5 marcado:** ________ &nbsp;&nbsp;&nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---

## 10. Consolidado Final de Calificación Académica

| Criterio Evaluado | Peso Oficial (%) | Nivel Marcado | Valor Obtenido (0–500) | Aporte Ponderado |
|---|:---:|:---:|:---:|:---:|
| **C1. Implementación del pipeline percepción → razonamiento → geometría** | 30% | | | |
| **C2. Diseño experimental y medición** | 25% | | | |
| **C3. Análisis crítico del modelo** | 20% | | | |
| **C4. Informe técnico y comunicación de resultados** | 15% | | | |
| **C5. Uso responsable de IA, seguridad y trabajo en equipo** | 10% | | | |
| **TOTAL CONSOLIDADO** | **100%** | | | **________ / 500** |

```text
Nota Académica sobre 5,0 = Nota Consolidada sobre 500 ÷ 100
Aporte a Laboratorios del Tercer Corte: L₃ = Nota Académica sobre 5,0
```

| Resultado Oficial de la Actividad | Registro |
|---|---|
| **Nota de Laboratorio IA 01 sobre 500 puntos** | __________ / 500 |
| **Nota Académica Oficial sobre 5,0** | __________ / 5,0 |
| **Número de Criterios en N3 o superior (Umbral Individual)** | _____ / 5 |
| **¿Cumple el Umbral Individual de Logro ABET (todos en N3+)?** | SÍ ☐ &nbsp;&nbsp;&nbsp;&nbsp; NO ☐ |

---

## 11. Consolidación ABET y Cierre de Mejora Continua

| Student Outcome / Indicador Evaluado | N Estudiantes Evaluables | N en Nivel N3 o Superior | Porcentaje de Logro (%) | Meta Institucional | Hallazgo Docente |
|---|:---:|:---:|:---:|:---:|---|
| **SO1 / Indicador 1.1** (Modelado matemático del pipeline) | | | | 70% | |
| **SO6 / Indicador 6.1** (Diseño experimental y medición) | | | | 70% | |
| **SO6 / Indicador 6.4** (Análisis e interpretación de fallas) | | | | 70% | |
| **SO3 / Indicador 3.1 y 3.3** (Documentación y comunicación) | | | | 70% | |
| **SO4 / Indicador 4.3** (Uso responsable de datos y operación segura) | | | | 70% | |
| **SO5 / Indicador 5.1** (Trabajo en equipo y roles) | | | | 70% | |

| Campo de Cierre de Ciclo de Mejora Continua | Registro de Gestión Pedagógica |
|---|---|
| **Decisión derivada del hallazgo** | |
| **Acción de mejora continua pedagógica y técnica** | |
| **Responsable de la acción** | Ing. Henry Roncancio |
| **Fecha prevista de seguimiento** | |
| **Evidencia de seguimiento** | |
| **Resultado observado en el segundo ciclo** | |

---

## 12. Cierre y Firmas de Conformidad

**Docente Evaluador:** Ing. Henry Roncancio &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; **Fecha de Evaluación:** ____________________

**Firma del Docente Evaluador:** __________________________________________________

**Observaciones Finales y Retroalimentación Pedagógica:**
________________________________________________________________________________________________________________________
________________________________________________________________________________________________________________________
