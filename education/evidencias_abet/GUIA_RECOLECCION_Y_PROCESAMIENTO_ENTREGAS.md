# Recolección y procesamiento de entregas — Corte 1, 2026-2

## Propósito

Este flujo permite conservar la evidencia original, relacionarla inequívocamente con cada estudiante y preparar una matriz de revisión compatible con los instrumentos del Taller 01 y el Laboratorio 01. Las entregas estudiantiles contienen datos personales y **no deben almacenarse dentro del repositorio `burger_delivery` ni distribuirse en los repositorios de los equipos**.

## Canal recomendado: Microsoft Forms institucional

Crear un formulario por actividad desde la cuenta institucional y configurarlo para que solo respondan personas de la organización. Agregar una pregunta obligatoria de carga de archivo y permitir únicamente archivos Word. Forms conserva las respuestas estructuradas y almacena los archivos recibidos en OneDrive para la Empresa.

Campos obligatorios:

1. Actividad: `Taller 01` o `Laboratorio 01`.
2. Correo institucional, capturado por Forms.
3. Código estudiantil.
4. Apellidos y nombres.
5. Grupo o equipo.
6. Códigos de los integrantes, cuando la evidencia sea común.
7. Usuario de GitHub y URL del repositorio o del commit, si aplica.
8. Archivo Word de la entrega.
9. Declaración: “Confirmo que el archivo corresponde a mi trabajo o al de mi equipo y que las contribuciones declaradas son verificables”.

Configuración mínima:

- una respuesta por persona para el Taller 01;
- un archivo `.docx`, máximo 20 MB;
- fecha y hora de cierre visibles;
- registro institucional de identidad;
- recibo de respuesta habilitado, si la política institucional lo permite;
- no aceptar enlaces editables como sustituto del archivo cerrado.

Para el Laboratorio 01 ya realizado, solicitar:

- **una entrega común por pareja** con el documento original producido durante la práctica, sin pedir que se reescriba;
- **una comprobación individual breve** por estudiante usando el Anexo A de `INSTRUMENTO_ABET_LAB_01_RED_ROS2_DISTRIBUIDA.docx` o las mismas preguntas trasladadas a Forms;
- una nota expresa si el documento original no existe o no puede recuperarse. La ausencia de evidencia no debe reemplazarse por evidencia inventada después de la práctica.

Para el Taller 01, que aún no ha sido publicado, entregar desde el inicio la plantilla oficial y solicitar un archivo individual con el nombre:

`C1_T01_<codigo>_<apellido>_v1.docx`

Nombre para la evidencia común del laboratorio:

`C1_L01_G<grupo>_<codigo1>_<codigo2>_v1.docx`

Nombre para un anexo individual del laboratorio, si se recibe como Word:

`C1_L01_ANEXO_<codigo>_<apellido>_v1.docx`

### Alternativa

Si Forms no está disponible, usar **Solicitar archivos** de OneDrive/SharePoint sobre una carpeta exclusiva para la actividad. Este mecanismo evita que los estudiantes vean o modifiquen entregas ajenas, pero debe acompañarse con un listado institucional, porque un nombre escrito por una persona no autenticada no prueba su identidad.

## Carpeta privada de trabajo

Crear esta estructura fuera del repositorio, por ejemplo en el OneDrive institucional del curso:

```text
ROS_ENTREGAS_2026-2/
└── corte_1/
    ├── taller_01/
    │   ├── 00_originales/
    │   ├── 01_manifiesto/
    │   ├── 02_texto_extraido/
    │   ├── 03_revision_docente/
    │   └── 04_resultados/
    └── laboratorio_01/
        ├── 00_originales/
        ├── 01_anexos_individuales/
        ├── 02_manifiesto/
        ├── 03_texto_extraido/
        ├── 04_revision_docente/
        └── 05_resultados/
```

Los archivos de `00_originales` no se editan. Toda corrección, anotación o conversión se guarda en otra carpeta. Al cerrar la recepción se calcula un hash SHA-256 por archivo y se registra en el manifiesto.

## Regla para reentregas

- conservar todas las versiones;
- nunca sobrescribir el original;
- incrementar el sufijo: `_v1`, `_v2`, etc.;
- registrar fecha, hora, motivo y versión válida para calificar;
- después del cierre, aceptar una versión nueva solo mediante la regla académica definida por el curso.

## Procesamiento asistido

Una vez descargada la carpeta, el procesamiento puede dividirse en cinco etapas:

1. **Inventario:** comparar códigos y correos con la lista oficial; detectar faltantes, duplicados, nombres incorrectos y archivos dañados.
2. **Preservación:** copiar los originales a la carpeta de trabajo, calcular SHA-256 y producir el manifiesto CSV.
3. **Extracción:** leer párrafos, tablas, títulos, leyendas e imágenes incrustadas de cada `.docx`; generar una copia de texto por estudiante o equipo.
4. **Preclasificación:** localizar evidencia candidata para cada criterio, señalar páginas o tablas, y marcar evidencia ausente o ambigua. Esta etapa no asigna automáticamente el nivel.
5. **Decisión docente:** el evaluador revisa la evidencia localizada, marca N1–N5 en el instrumento, registra retroalimentación y consolida la nota académica y el logro ABET por separado.

El procesamiento automático debe producir, como mínimo:

- `inventario_entregas.csv`;
- `matriz_revision.xlsx` o `.csv`, con una fila por estudiante y columnas C1–C5;
- `incidencias.csv`, para faltantes, duplicados, archivos inválidos o identidad no conciliada;
- un resumen por indicador ABET con `N evaluable`, `N ≥ N3`, porcentaje de logro y meta;
- un archivo de carga para Zubatronic únicamente cuando se conozca y valide su estructura exacta.

La automatización puede localizar evidencia y aplicar fórmulas, pero **la selección del nivel N1–N5 queda en manos del docente**. No debe inferir autoría, logro o fraude solo a partir del texto extraído.

## Campos del manifiesto

Usar `MANIFIESTO_ENTREGAS_TEMPLATE.csv` como encabezado base. Un registro representa un archivo recibido. Para el laboratorio, los integrantes comparten el identificador de equipo, pero la comprobación individual tiene su propia fila.

## Cierre y custodia

1. Bloquear o cerrar el formulario al vencer el plazo.
2. Exportar las respuestas y conservar el archivo de respuestas junto con los originales.
3. Verificar que cada calificación tenga trazabilidad a archivo, estudiante, criterio y versión del instrumento.
4. Restringir el acceso a las personas responsables del curso y del assessment.
5. Aplicar la política institucional de retención y eliminación de datos; Git no se usa como archivo de entregas estudiantiles.

