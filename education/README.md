# 🎓 Ecosistema Pedagógico y Docente — Burger Delivery

Bienvenido a la sección pedagógica del repositorio **Burger Delivery**. Este espacio articula el material curricular, talleres prácticos, guías de laboratorio y metodologías de aprendizaje diseñadas para cursos avanzados y electivas de profundización en **Robótica Aplicada, ROS 2 e Inteligencia Artificial**.

---

## 🏛️ Integración con Investigación Formativa (Living Lab)
Este ecosistema docente está estructurado como un **banco de pruebas vivo (*Living Lab*)**. Las actividades prácticas de los estudiantes no son ejercicios aislados, sino que alimentan directamente la validación experimental y estrés de la celda de trabajo robótica:
- Recolección y validación de métricas de red DDS y latencia WiFi.
- Evaluación de precisión y repetibilidad del árbol de transformaciones (TF2) con AprilTags.
- Benchmarking de algoritmos de percepción espacial 3D con modelos VLM (Gemini Robotics).

---

## 📂 Estructura del Directorio

```text
education/
├── syllabus/                    # Diseño curricular y programas de curso
│   ├── SYLLABUS_ROS2_ROBOTICA.md
│   ├── Syllabus_ABET_Mecatronica_2026I_V2.docx
│   └── Syllabus_Robotica_ROS2_Experimental.docx
├── talleres/                    # Talleres prácticos paso a paso
│   ├── TALLER_ROS2_CLI.md
│   ├── TALLER_URDF_TF.md
│   └── TALLER_URDF_TF.pdf
├── guias_laboratorio/           # Guías formales de laboratorio y plantillas
│   ├── GUIA_LAB_01_RED_ROS2_TALKER_LISTENER.docx
│   ├── normalizar_guia_lab_01.py
│   ├── rendered/
│   │   └── GUIA_LAB_01_RED_ROS2_TALKER_LISTENER.pdf
│   └── templates/
│       └── Formato_Guias_de_Laboratorio.docx
├── proyectos_evaluables/        # Contratos de integración y rúbricas de corte
│   └── PROYECTO_INTERMEDIO_MOVEIT2_DELIVERY.md
└── metodologias/                # Guías de buenas prácticas y habilidades de aprendizaje
    └── SKILL_SUPERSTUDENT.md
```

---

## 📚 Índice de Recursos

### 1. Programas y Diseño Curricular (`syllabus/`)
- [**Syllabus — Robótica Aplicada con ROS 2 (Formato ABET)**](syllabus/SYLLABUS_ROS2_ROBOTICA.md): Programa de 16 semanas estructurado en 6 módulos de aprendizaje por competencias.

### 2. Talleres Guiados (`talleres/`)
- [**Taller CLI ROS 2**](talleres/TALLER_ROS2_CLI.md): Manejo avanzado de introspección por consola (tópicos, nodos, servicios, acciones).
- [**Taller URDF y TF2**](talleres/TALLER_URDF_TF.md): Guía de 11 secciones para modelado geométrico, cinemática y coherencia del árbol TF.

### 3. Guías de Laboratorio (`guias_laboratorio/`)
- [**Guía Lab 01: Red ROS 2 y Comunicación Pub/Sub**](guias_laboratorio/rendered/GUIA_LAB_01_RED_ROS2_TALKER_LISTENER.pdf): Configuración de entorno y validación de red distribuida.
- Plantilla institucional editable: `guias_laboratorio/templates/Formato_Guias_de_Laboratorio.docx`.

### 4. Proyectos de Evaluación (`proyectos_evaluables/`)
- [**Proyecto Intermedio: MoveIt 2 & Delivery**](proyectos_evaluables/PROYECTO_INTERMEDIO_MOVEIT2_DELIVERY.md): Rúbricas y especificación del contrato de integración entre el manipulador Kinova y los robots móviles.

### 5. Metodología y Buenas Prácticas (`metodologias/`)
- [**Skill SuperStudent**](metodologias/SKILL_SUPERSTUDENT.md): Bitácora de experiencia técnica, lecciones aprendidas y resolución de fallos típicos en robótica colaborativa.
