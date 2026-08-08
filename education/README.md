# Education — ROBOT OPERATING SYSTEM - ROS

Esta carpeta reúne el material docente asociado al proyecto `burger_delivery` para la asignatura **ROBOT OPERATING SYSTEM - ROS**. Su propósito es mantener en el mismo repositorio el diseño curricular, las actividades que realizan los estudiantes, las guías experimentales y los instrumentos utilizados para producir las calificaciones y evidencias ABET.

`education` contiene documentación; el código ejecutable de los paquetes ROS 2 continúa en las demás carpetas del repositorio. Los talleres y laboratorios deben apuntar a archivos, comandos e interfaces reales del proyecto.

## Quién utiliza cada material

- La persona participante consulta `talleres/`, los PDF publicados en `guias_laboratorio/rendered/` y las especificaciones de `proyectos_evaluables/`.
- El equipo docente mantiene el syllabus, las fuentes editables de las guías y los instrumentos de `evidencias_abet/`.
- Los formatos ABET registran evidencia y calificación, pero no sustituyen las instrucciones pedagógicas del taller o laboratorio.

## Estructura

```text
education/
├── syllabus/              # Único syllabus oficial y archivo deprecated
├── talleres/              # Actividades guiadas realizadas por estudiantes
├── guias_laboratorio/     # Guías editables, plantillas y PDF publicados
├── evidencias_abet/       # Formatos para evidencia y calificación
├── proyectos_evaluables/  # Entregas integradoras y criterios de aceptación
└── metodologias/          # Orientaciones de aprendizaje y documentación
```

## Fuente oficial y archivos derivados

Para evitar versiones incompatibles, cada responsabilidad debe tener una sola fuente oficial.

| Responsabilidad | Fuente oficial | Archivo derivado o de consulta |
|---|---|---|
| Syllabus institucional | [`syllabus/Syllabus_Robotica_ROS2_Experimental.docx`](syllabus/Syllabus_Robotica_ROS2_Experimental.docx) | Las versiones anteriores están identificadas en [`syllabus/deprecated/`](syllabus/deprecated/README.md) y no deben usarse para docencia |
| Taller CLI de ROS 2 | [`talleres/TALLER_ROS2_CLI.md`](talleres/TALLER_ROS2_CLI.md) | Instrumento ABET separado en `evidencias_abet/` |
| Taller URDF y TF2 | [`talleres/TALLER_URDF_TF.md`](talleres/TALLER_URDF_TF.md) | [`talleres/TALLER_URDF_TF.pdf`](talleres/TALLER_URDF_TF.pdf) |
| Guía de laboratorio 01 | [`guias_laboratorio/GUIA_LAB_01_RED_ROS2_TALKER_LISTENER.docx`](guias_laboratorio/GUIA_LAB_01_RED_ROS2_TALKER_LISTENER.docx) | [`guias_laboratorio/rendered/GUIA_LAB_01_RED_ROS2_TALKER_LISTENER.pdf`](guias_laboratorio/rendered/GUIA_LAB_01_RED_ROS2_TALKER_LISTENER.pdf) |
| Evidencia y nota del Taller 01 | [`evidencias_abet/PLANTILLA_EVIDENCIA_ABET_TALLER_01_ROS2_CLI.docx`](evidencias_abet/PLANTILLA_EVIDENCIA_ABET_TALLER_01_ROS2_CLI.docx) | [`evidencias_abet/PLANTILLA_EVIDENCIA_ABET_TALLER_01_ROS2_CLI.md`](evidencias_abet/PLANTILLA_EVIDENCIA_ABET_TALLER_01_ROS2_CLI.md), representación auxiliar no normativa |
| Proyecto del primer corte | [`proyectos_evaluables/PROYECTO_CORTE_1_CONEXION_KINOVA.md`](proyectos_evaluables/PROYECTO_CORTE_1_CONEXION_KINOVA.md) | No tiene archivo derivado; esta especificación es la fuente oficial |
| Evidencia y nota del Proyecto Corte 1 | [`evidencias_abet/INSTRUMENTO_ABET_PROYECTO_CORTE_1_CONEXION_KINOVA.docx`](evidencias_abet/INSTRUMENTO_ABET_PROYECTO_CORTE_1_CONEXION_KINOVA.docx) | [`evidencias_abet/INSTRUMENTO_ABET_PROYECTO_CORTE_1_CONEXION_KINOVA.md`](evidencias_abet/INSTRUMENTO_ABET_PROYECTO_CORTE_1_CONEXION_KINOVA.md), representación auxiliar |

## Recursos disponibles

### Syllabus

- [Syllabus oficial de ROBOT OPERATING SYSTEM - ROS](syllabus/Syllabus_Robotica_ROS2_Experimental.docx).

### Talleres

- [Taller 01 — CLI de ROS 2](talleres/TALLER_ROS2_CLI.md): nodos, tópicos, servicios, parámetros, acciones e introspección del grafo.
- [Taller — URDF y TF2](talleres/TALLER_URDF_TF.md): descripción del robot, frames, transformaciones y validación en tiempo de ejecución.

### Laboratorios

- [Guía 01 — Red ROS 2, DDS y comunicación talker/listener](guias_laboratorio/rendered/GUIA_LAB_01_RED_ROS2_TALKER_LISTENER.pdf).
- [Plantilla institucional para nuevas guías](guias_laboratorio/templates/Formato_Guias_de_Laboratorio.docx).

### Proyectos y Control de Versiones

- [Proyecto del primer corte — Conexión segura con Kinova Gen3](proyectos_evaluables/PROYECTO_CORTE_1_CONEXION_KINOVA.md).
- [Guía de configuración de repositorios por equipo y arquitectura multi-remoto](proyectos_evaluables/CONFIGURACION_REPOSITORIOS_EQUIPOS.md).
- [⭐ Guía Interactiva GitHub Organizations, Trabajo en Equipo y ABET](../git-fundamentals/equipos_organizaciones_abet.html): repositorios privados con historia compartida, ramas individuales `feat/...`, pull requests y entrega identificada por commit SHA.


### Evidencias

- [Formato de evidencia y calificación del Taller 01](evidencias_abet/PLANTILLA_EVIDENCIA_ABET_TALLER_01_ROS2_CLI.docx).
- [Instrumento ABET del Proyecto Corte 1 — Conexión con Kinova](evidencias_abet/INSTRUMENTO_ABET_PROYECTO_CORTE_1_CONEXION_KINOVA.docx).

## Flujo de actualización

1. Modifique únicamente la fuente oficial indicada en la tabla.
2. Actualice o regenere los archivos derivados correspondientes.
3. Compruebe que los comandos, nombres de nodos, tópicos, servicios, frames y rutas coincidan con el proyecto real.
4. Verifique que cada instrumento de evaluación indique evidencia observable, criterio, peso y forma de consolidar la nota.
5. Registre en el mismo cambio la fuente y sus derivados.

No cree copias activas con sufijos como `_final`, `_nuevo` o `_consistente`. El historial de Git conserva las versiones anteriores; cualquier documento legado que deba retenerse se mueve a una carpeta `deprecated` y se identifica como no vigente.

## Relación con la carpeta `ROS` de Windows

En el entorno local, `ROS/education` está configurada como un enlace a esta carpeta del repositorio. Ambas rutas muestran los mismos archivos: un cambio realizado desde Windows o Ubuntu modifica la misma copia versionada por Git. No deben mantenerse dos carpetas `education` independientes ni copiar archivos manualmente entre ellas.
