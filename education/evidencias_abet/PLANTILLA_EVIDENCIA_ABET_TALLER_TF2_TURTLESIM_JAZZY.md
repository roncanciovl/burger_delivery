# Formato de evidencias y calificación — Taller: TF2 Turtlesim Jazzy

> Instrumento calificable asociado a `education/talleres/taller_turtlesim_tf2/TALLER_TF2_TURTLESIM_JAZZY.md`. El taller conserva las instrucciones de aprendizaje; este formato permite registrar la evidencia, marcar el nivel alcanzado en cada criterio y obtener una única nota consolidada.

## 1. Identificación

| Campo | Registro |
|---|---|
| Asignatura | ROBOT OPERATING SYSTEM - ROS |
| Periodo | 2026-2 |
| Corte / instrumento | Primer corte / Talleres y tareas |
| Actividad | Taller — TF2 Turtlesim Jazzy |
| Nombre completo |  |
| Código / grupo |  |
| Fecha de entrega |  |
| Evaluador |  |
| Versión del instrumento | 1.0 |
| Enlace o ubicación de la entrega |  |
| Commit evaluado, si aplica |  |
| Unidad de análisis | Individual |

## 2. Uso del formato

1. La persona participante registra las evidencias requeridas (E1–E4).
2. Quien evalúa marca **una sola banda de desempeño por criterio** y registra un valor de 0 a 500 dentro del intervalo correspondiente.
3. Los niveles N3, N4 y N5 representan una progresión de dominio; N1 y N2 son bandas alternativas para evidencia insuficiente o parcial.
4. Si falta una evidencia obligatoria en una entrega exigible, se marca N1 y se registra 0 para la nota académica. Un retiro oficial o una exclusión autorizada se registra como `NA / no evaluado`, se excluye del denominador ABET y no se convierte en cero.
5. Zubatronic/SGDE calcula el aporte ponderado de cada criterio. En una copia manual se usa `aporte = valor × peso / 100`.
6. La entrega es individual y debe nombrarse `C1_TALLER_TF2_TURTLESIM_<codigo>_<apellido>_v1.docx`. El código del archivo, el código registrado en el documento y la identidad de la entrega deben coincidir.

| Nivel | Intervalo Zubatronic / valor guía | Interpretación |
|---|---:|---|
| N5 | 475–500 / guía 500 | Excelente: desempeño completo, preciso y reproducible. |
| N4 | 400–474 / guía 450 | Bueno: desempeño correcto con omisiones menores. |
| N3 | 300–399 / guía 350 | Aceptable: logra lo esencial con explicación parcial. Este es el umbral de logro. |
| N2 | 150–299 / guía 250 | Parcial: evidencia incompleta o errores importantes. |
| N1 | 0–149 / guía 100 | No cumple: evidencia mínima, fragmentaria o no funcional. Sin evidencia obligatoria se registra 0. |

### 2.1. Parámetros de assessment

| Parámetro | Regla adoptada |
|---|---|
| Población | Censo de estudiantes matriculados que deben presentar el Taller de TF2 en 2026-2. |
| Momento de medición | Primer corte, después de la introducción a transformaciones espaciales en ROS 2. |
| Evaluador | Docente responsable de ROBOT OPERATING SYSTEM - ROS. |
| Umbral individual | N3 o superior en cada indicador evaluado. |
| Meta de cohorte | Al menos 70% de estudiantes evaluables alcanza N3 o superior en cada indicador. |
| Muestreo | No se usa muestra: se evalúa el censo de entregas exigibles. |
| Evidencia faltante | Entrega exigible sin evidencia: N1 con 0. Retiro oficial o exclusión autorizada: `NA / no evaluado`. |

### 2.2. Alineación de criterios para Zubatronic/SGDE

| Criterio | Peso | SO principal | Indicador de desempeño literal | Evidencia directa |
|---|---:|---|---|---|
| C1. Ejecución del simulador y lectura de matriz (traslación) | 30% | SO6 | **6.4.** Conduce experimentos en simulación utilizando herramientas modernas para validar empíricamente parámetros espaciales. | E1 y explicación del vector de traslación extraído. |
| C2. Interpretación matemática de la rotación (Cuaterniones a Yaw) | 30% | SO1 | **1.1.** Aplica principios matemáticos (matrices de transformación homogénea, rotaciones y cuaterniones) para resolver problemas espaciales en robótica. | E2, con cálculos y explicación teórica demostrable del ángulo Yaw. |
| C3. Comprobación empírica de composición de transformaciones y Árbol TF | 30% | SO1 | **1.1.** Aplica principios matemáticos (matrices de transformación homogénea, rotaciones y cuaterniones) para resolver problemas espaciales en robótica. | E3: Resultados del Jupyter Notebook vs salida de `tf2_echo`, sumado al análisis del `frames.pdf`. |
| C4. Evidencia reproducible y documentación | 10% | SO3 | **3.1.** Elabora documentación técnica reproducible del sistema ROS 2 y comunica resultados experimentales. | E4, registro reproducible de la práctica con pantallazos y comandos legibles. |

## 3. Registro mínimo de evidencias

Las capturas deben ser legibles y mostrar el resultado obtenido, no solamente el comando escrito.

| Código | Evidencia requerida | Archivo, enlace, página o ubicación |
|---|---|---|
| E1 | Captura de `turtlesim` activo y salida de `ros2 run tf2_ros tf2_echo world turtle1` demostrando los valores de traslación. |  |
| E2 | Cálculo explícito (manual o en script Python) para convertir el cuaternión entregado por `tf2_echo` al ángulo $\theta$ en 2D. |  |
| E3 | Captura del PDF generado por `view_frames` y del cuaderno Jupyter comparando $T_{T1}^{T2}$ calculado vs el que arroja ROS. |  |
| E4 | Enlace a la carpeta o repositorio donde constan los comandos y notebooks del experimento de forma organizada. |  |

Ubicación del registro reproducible de comandos: ________________________________________________

## 4. Respuestas breves de la persona participante

### E2. Conversión de Cuaternión a Rotación plana

Muestre el cuaternión arrojado por la terminal (w, x, y, z) y explique paso a paso cómo obtiene el ángulo $\theta$ (Yaw).

________________________________________________________________________________

________________________________________________________________________________

### E3. Composición de Transformaciones

Describa cómo verificó matemáticamente la matriz inversa multiplicada y cómo comparó el resultado con la transformación en tiempo real provista por el Árbol TF.

________________________________________________________________________________

________________________________________________________________________________

## 5. Selección del nivel alcanzado

Marque con una **X** una sola banda en cada criterio y escriba el valor exacto que se capturará en Zubatronic. Si no existe evidencia obligatoria verificable, marque N1 y registre 0.

### C1. Ejecución del simulador y lectura de matriz (traslación) — peso 30%

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, explica con exactitud la relación entre el vector de traslación del simulador y la representación matricial 2D. |
| ☐ | N4 — 400–474 | Además de N3, identifica y documenta correctamente todos los componentes de traslación (`x, y`). |
| ☐ | N3 — 300–399 | Ejecuta el entorno, lanza el listener TF y presenta la lectura del vector de traslación. |
| ☐ | N2 — 150–299 | Logra ejecutar parcialmente el simulador o hay discrepancias en la lectura de TF. |
| ☐ | N1 — 0–149 | Evidencia fragmentaria o falta del uso de `tf2_echo` para extraer la traslación. |

**Nivel C1 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

### C2. Interpretación matemática de la rotación (Cuaterniones a Yaw) — peso 30%

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, comprende el modelo tridimensional subyacente y lo reduce formalmente a una rotación planar pura 2D matemáticamente demostrable. |
| ☐ | N4 — 400–474 | Además de N3, calcula correctamente el ángulo Yaw y justifica los valores usando el script o ecuación atan2 de manera correcta. |
| ☐ | N3 — 300–399 | Identifica el cuaternión en la salida e intenta la conversión a ángulo. |
| ☐ | N2 — 150–299 | Confunde las dimensiones del cuaternión o realiza un cálculo con un resultado inconsistente al ángulo observable del simulador. |
| ☐ | N1 — 0–149 | Omite por completo el cálculo del ángulo Yaw de rotación plana. |

**Nivel C2 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

### C3. Comprobación empírica de composición de transformaciones y Árbol TF — peso 30%

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | Además de N4, relaciona formalmente por qué el árbol generado por `view_frames` es equivalente a la multiplicación de las matrices de transformación homogénea inversas y directas. |
| ☐ | N4 — 400–474 | Además de N3, la comparación entre el cálculo en Jupyter y la salida empírica de TF2 coinciden, y se genera un análisis claro sobre ello. |
| ☐ | N3 — 300–399 | Genera el archivo `frames.pdf`, ejecuta la composición en Python y hace el `tf2_echo` de las dos tortugas. |
| ☐ | N2 — 150–299 | Faltan cálculos en el cuaderno o hay errores matemáticos que impiden que los valores cuadren con los de la terminal TF. |
| ☐ | N1 — 0–149 | Falta la comprobación empírica matemática y de salida de la terminal. |

**Nivel C3 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

### C4. Evidencia reproducible y documentación — peso 10%

| Marque | Nivel | Evidencia observable del nivel |
|:---:|---|---|
| ☐ | N5 — 475–500 | La documentación es excelente, todos los comandos se muestran en orden causal y los pantallazos son autodescriptivos. |
| ☐ | N4 — 400–474 | Además de N3, la estructura de la entrega cumple al 100% las normativas de nombrado y el repositorio es legible. |
| ☐ | N3 — 300–399 | Presenta un conjunto de comandos y pantallazos verificables. |
| ☐ | N2 — 150–299 | La evidencia es confusa, desordenada o el nombrado no sigue las normas de la clase. |
| ☐ | N1 — 0–149 | No entrega documentación reproducible o el repositorio de respaldo es ilegible. |

**Nivel C4 marcado:** ________ &nbsp;&nbsp; **Valor Zubatronic (0–500):** ________

---
_Fin del instrumento_
