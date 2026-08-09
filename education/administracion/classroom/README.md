# Controlador de Google Classroom — ROBOT OPERATING SYSTEM - ROS

Este controlador se instala **una sola vez** en Google Apps Script. Después, la información variable se administra desde una hoja privada: curso, estudiantes, equipos, actividades, fechas, invitaciones y entregas. No es necesario pegar un script por cada operación.

La fuente versionada es [`apps_script/Controlador.gs`](apps_script/Controlador.gs). El archivo [`apps_script/appsscript.json`](apps_script/appsscript.json) declara el servicio avanzado de Classroom y permite sincronizar posteriormente el proyecto con `clasp`.

## Instalación única

1. Abra el proyecto de Apps Script usado para la prueba.
2. Elimine el contenido y los archivos de las pruebas anteriores para evitar funciones duplicadas.
3. Cree un archivo de secuencia de comandos llamado `Controlador`.
4. Pegue allí todo el contenido de [`apps_script/Controlador.gs`](apps_script/Controlador.gs). No lo pegue dentro de otra función.
5. Compruebe que **Google Classroom API** continúe agregado en **Servicios**.
6. Guarde y ejecute `instalarControladorROS`.
7. Autorice los permisos solicitados.
8. Abra el enlace `enlacePanel` mostrado en el registro de ejecución.

`instalarControladorROS` crea una hoja privada en el Drive de la cuenta que ejecuta el script. Si se vuelve a ejecutar, reutiliza la misma hoja y no borra sus datos.

En una instalación existente, reemplace primero el archivo `Controlador` por la
versión actual y vuelva a ejecutar `instalarControladorROS`. La migración agrega
las hojas y columnas faltantes sin borrar identificadores ni registros previos.

## Preparación del panel

### Hoja `Estudiantes`

Copie desde `Lista de Estudiantes ROS.xlsx` únicamente las columnas:

```text
CODIGO | APELLIDOS | NOMBRES | CORREO
```

Péguelas desde la fila 2. Las columnas `ESTADO_CLASSROOM` y `OBSERVACIONES` son administradas por el controlador.

### Hoja `Actividades`

El panel incluye inicialmente:

- `T01`: Taller 01 individual, 500 puntos.
- `L01`: evidencia común del laboratorio por pareja, 500 puntos.
- `L01-IND`: comprobación individual, sin nota académica separada.
- `E01`: proyecto del primer corte, inicialmente inactivo.

Las columnas de asignación admiten:

- `TODOS`: actividad individual o comprobación dirigida a toda la clase;
- `ENTREGANTES`: una sola persona designada por cada equipo recibe la entrega
  común. `EQUIPOS_OBJETIVO` puede contener una lista separada por comas; vacío
  significa todos los equipos activos.

### Hoja `Equipos`

La hoja contiene una fila por estudiante y es la fuente única para Classroom y
GitHub. Complete `EQUIPO_ID`, `GITHUB_USER`, `ROL_INICIAL` y `ENTREGANTE`.
Debe existir exactamente un entregante por equipo. `REPOSITORIO` y
`GITHUB_TEAM` se derivan del identificador del equipo.

Para incorporar altas posteriores sin sobrescribir la conformación existente,
ejecute `sincronizarEquiposDesdeEstudiantesROS`. Antes de activar una actividad
grupal ejecute `vistaPreviaEquiposROS`.

Complete las fechas con el formato `AAAA-MM-DD` y las horas con `HH:MM`. Una fecha vacía crea el borrador sin vencimiento. Todas las actividades se crean como `DRAFT`; el controlador no las publica automáticamente.

## Ejecución normal

1. Ejecute `vistaPreviaControladorROS`.
2. Corrija cualquier incidencia mostrada en estudiantes o actividades.
3. En `Configuracion`, cambie `MODO` de `VISTA_PREVIA` a `APLICAR`.
4. Ejecute `configurarTodoROS`.
5. El controlador crea o reutiliza el curso, crea los temas, envía únicamente las invitaciones faltantes y crea únicamente los borradores faltantes.
6. Al finalizar, `MODO` vuelve automáticamente a `VISTA_PREVIA`.
7. Ejecute `auditarControladorROS` para comprobar el resultado.

Las funciones son idempotentes: conservan identificadores en el panel, consultan Classroom antes de crear y evitan duplicar el curso, los temas, las invitaciones y las actividades.

La réplica de la hoja hacia los repositorios privados se realiza con el
sincronizador documentado en [`../github/README.md`](../github/README.md).

## Recepción de entregas

Después de que los estudiantes entreguen trabajos, ejecute `sincronizarEntregasROS`. La hoja `Entregas` registrará actividad, estudiante, estado, entrega tardía, notas existentes y enlaces a los adjuntos.

Esta sincronización no descarga todavía los `.docx`, no asigna niveles N1–N5 y no calcula automáticamente logro ABET. Es el inventario trazable para el procesador documental y la decisión posterior del docente.

## Seguridad

- No guarde correos estudiantiles dentro de `Controlador.gs` ni en Git.
- La hoja creada por el controlador debe mantenerse privada.
- No comparta `COURSE_ID`, códigos de clase ni enlaces de adjuntos en repositorios públicos.
- La palabra `APLICAR` funciona como confirmación explícita y se restablece después de cada configuración.
- El controlador nunca publica actividades; la publicación se revisa manualmente en Classroom.

## Sincronización futura con `clasp`

La carpeta `apps_script/` ya está preparada para `clasp`. Cuando se configure la herramienta oficial de Google, las actualizaciones podrán enviarse desde esta fuente local al proyecto de Apps Script con `clasp push`, sin copiar y pegar nuevamente. Los archivos `.clasp.json` y `.clasprc.json` están excluidos de Git porque contienen la vinculación y las credenciales locales.
