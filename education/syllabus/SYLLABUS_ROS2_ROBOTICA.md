# SYLLABUS — Robótica Aplicada con ROS 2

> **Formato:** ABET — Programa de Ingeniería Mecatrónica  
> **Período:** 2026-I  
> **Universidad Militar Nueva Granada**

---

## 1. INFORMACIÓN GENERAL

| Campo | Detalle |
|:---|:---|
| **Nombre de la Asignatura** | Robótica Aplicada con ROS 2 |
| **Código** | *[Asignar según malla curricular]* |
| **Tipo de Curso** | Teórico-Práctico (Electiva de Profundización) |
| **Semestre** | VIII – IX |
| **Prerrequisitos** | Control Automático, Programación Orientada a Objetos (C++/Python), Cinemática de Mecanismos |
| **Correquisitos** | Ninguno |
| **Créditos Académicos** | 3 |
| **Horas Trabajo Directo** | 4 h/semana (2 h teoría + 2 h laboratorio) |
| **Horas Trabajo Independiente** | 5 h/semana |
| **Modalidad** | Presencial con componente de laboratorio obligatorio |
| **Fecha de Elaboración** | Mayo 2026 |
| **Coordinador / Jefe de Área** | *[Nombre del coordinador del programa]* |
| **Docente(s) / Correo** | *[Nombre y correo institucional del docente]* |

---

## 2. DESCRIPCIÓN DEL CURSO

Este curso proporciona al estudiante de Ingeniería Mecatrónica las competencias necesarias para diseñar, implementar y validar sistemas robóticos colaborativos utilizando el framework **ROS 2 (Robot Operating System 2)** en su distribución **Jazzy Jalisco**. El enfoque pedagógico es eminentemente práctico: a lo largo de 16 semanas, el estudiante construye progresivamente los subsistemas de una **celda de automatización colaborativa** donde un manipulador industrial de 7 grados de libertad interactúa espacialmente con unidades móviles autónomas, coordinados mediante transformaciones geométricas en tiempo real, localización visual por marcadores fiduciarios y planificación de trayectorias libre de colisiones.

El curso cubre desde los fundamentos de comunicación distribuida (nodos, tópicos, servicios y acciones) hasta temas avanzados como percepción robótica con inteligencia artificial multimodal, integración de hardware industrial con drivers ROS 2, y diseño de contratos de integración entre subsistemas heterogéneos. La solución integradora final replica patrones de automatización directamente transferibles a **celdas de manufactura flexible, logística intramuros, robótica de almacén y sistemas de inspección industrial**.

---

## 3. OBJETIVOS DE FORMACIÓN

### 3.1. Objetivo General
Desarrollar en el estudiante la capacidad de diseñar, implementar y validar arquitecturas de software para sistemas robóticos colaborativos multi-agente usando ROS 2, integrando manipulación, movilidad, percepción y coordinación en un entorno de automatización industrial.

### 3.2. Objetivos Específicos
1. Comprender la arquitectura distribuida de ROS 2 (nodos, tópicos, servicios, acciones, parámetros) y aplicarla al control de robots reales y simulados.
2. Modelar robots y escenas industriales mediante URDF/XACRO, garantizando coherencia espacial a través de árboles de transformaciones (TF2).
3. Configurar y operar el framework MoveIt 2 para planificación de trayectorias, evasión de colisiones y manipulación autónoma con brazos industriales.
4. Implementar sistemas de localización visual basados en marcadores fiduciarios (AprilTags) y técnicas de percepción con IA multimodal.
5. Diseñar e implementar contratos de integración (TF, servicios, interfaces) entre subsistemas robóticos desarrollados por equipos independientes.
6. Diagnosticar y resolver problemas en sistemas robóticos distribuidos usando metodologías de depuración por capas.

---

## 4. COMPETENCIAS DISCIPLINARES

| ID | Competencia | Nivel Bloom |
|:---|:---|:---|
| **CD-1** | Diseñar la arquitectura de software de un sistema robótico distribuido utilizando el paradigma de grafos computacionales de ROS 2 (publicador/suscriptor, cliente/servidor, acciones). | Aplicación |
| **CD-2** | Modelar la geometría y cinemática de robots industriales y escenas de trabajo mediante URDF/XACRO, validando la coherencia del árbol de transformaciones con herramientas estándar. | Análisis |
| **CD-3** | Configurar y ejecutar pipelines de planificación de movimiento (MoveIt 2 / MTC) para manipuladores con evasión de colisiones en escenas dinámicas. | Aplicación |
| **CD-4** | Implementar sistemas de percepción robótica que integren localización visual (AprilTags, cámaras calibradas) y modelos de IA para razonamiento espacial. | Síntesis |
| **CD-5** | Integrar subsistemas heterogéneos (manipulador + móvil + visión) en una celda colaborativa funcional, respetando contratos de interfaz y protocolos de comunicación definidos. | Evaluación |

---

## 5. COMPETENCIAS NO DISCIPLINARES

| ID | Competencia |
|:---|:---|
| **CND-1** | **Trabajo colaborativo en equipos de ingeniería:** Distribuir responsabilidades, definir interfaces de integración y coordinar entregas técnicas entre sub-equipos con responsabilidades complementarias. |
| **CND-2** | **Gestión de configuración y versionamiento:** Utilizar Git y GitHub como herramientas profesionales de control de versiones, branching, pull requests y documentación técnica. |
| **CND-3** | **Comunicación técnica:** Documentar arquitecturas, decisiones de diseño y procedimientos de diagnóstico de forma clara, reproducible y orientada a pares de ingeniería. |
| **CND-4** | **Resolución metódica de problemas:** Aplicar protocolos de diagnóstico por capas (sintaxis → TF → red → lógica) para aislar y resolver fallas en sistemas distribuidos. |

---

## 6. RESULTADOS DE APRENDIZAJE E INDICADORES DEL CURSO

| RA | Resultado de Aprendizaje | Indicadores de Logro |
|:---|:---|:---|
| **RA-1** | El estudiante configura un entorno ROS 2 funcional y demuestra la comunicación entre nodos mediante tópicos, servicios y acciones. | • Compila un workspace con `colcon` sin errores. • Implementa un par publicador/suscriptor funcional. • Invoca un servicio desde CLI y desde código. |
| **RA-2** | El estudiante modela un robot y su escena operativa en URDF, visualiza correctamente en RViz2 y valida el árbol TF. | • El URDF pasa `xmllint` y `check_urdf`. • El árbol de `view_frames` coincide con el diseño. • Distingue visual vs. collision. |
| **RA-3** | El estudiante configura MoveIt 2 para un manipulador y ejecuta operaciones de pick & place con evasión de colisiones. | • Plan punto a punto sin colisiones. • Usa `AttachedCollisionObject`. • Destino dinámico vía TF. |
| **RA-4** | El estudiante implementa un sistema de localización visual funcional usando AprilTags integrado al árbol TF de la celda. | • TF estable y continua. • Cancelación de perspectiva funcional. • Error < 3 cm. |
| **RA-5** | El estudiante integra exitosamente su subsistema con el de otro equipo respetando contratos de interfaz predefinidos. | • Integración sin parches ad-hoc. • Diagnóstico por capas aplicado. |

---

## 7. CONTENIDO TEMÁTICO (16 Semanas)

### Módulo 0 — Fundamentos de Entorno y Herramientas (Semanas 1–2)

| Sem. | Tema | Contenido | Actividad Práctica |
|:---:|:---|:---|:---|
| 1 | Ecosistema Linux y terminal | Navegación, permisos, variables de entorno, `bash`, VS Code. Instalación de ROS 2 Jazzy. | Lab: Instalar ROS 2, verificar con `ros2 doctor`. |
| 2 | Git y GitHub para robótica | Repositorios, commits, branches, merges, PRs, `.gitignore`, resolución de conflictos. | Taller: Clonar repo del curso, crear branch, hacer PR. |

### Módulo 1 — Fundamentos de ROS 2 (Semanas 3–5)

| Sem. | Tema | Contenido | Actividad Práctica |
|:---:|:---|:---|:---|
| 3 | Arquitectura de ROS 2 | Grafo computacional: nodos, tópicos (pub/sub), DDS, QoS. Workspaces, paquetes, `colcon`. | Lab: Crear paquete, nodo pub/sub en Python. |
| 4 | Servicios, parámetros y Launch | Cliente/servidor. Parámetros dinámicos. Launch files Python. Composición de nodos. | Lab: Servicio custom. Launch file multi-nodo. |
| 5 | Acciones y CLI avanzado | Action servers/clients (feedback, result, cancel). Introspección con herramientas CLI. | Lab: Action server con feedback de progreso. |

### Módulo 2 — Modelado Robótico: URDF, TF2 y Visualización (Semanas 6–8)

| Sem. | Tema | Contenido | Actividad Práctica |
|:---:|:---|:---|:---|
| 6 | URDF: links, joints y geometría | XML, Links (visual/collision/inertial), Joints (fixed/revolute/continuous/prismatic), ejes, límites, `mimic`, meshes STL. | Lab: Robot 3-DOF en URDF. `check_urdf`. |
| 7 | RSP, TF2 y RViz2 | Pipeline URDF → `robot_description` → RSP → `/tf`. Árbol TF. `tf2_echo`, `view_frames`. JSP GUI. | **Taller URDF/TF:** Escena industrial, validar árbol, calibrar offsets. |
| 8 | XACRO y escenas multi-robot | Macros, parámetros, includes. URDFs separados por fuente de posición. Múltiples RSP con remapping. | Lab: Escena manipulador + móviles separados. **Primer Corte.** |

### Módulo 3 — Percepción y Localización Visual (Semanas 9–10)

| Sem. | Tema | Contenido | Actividad Práctica |
|:---:|:---|:---|:---|
| 9 | Visión en ROS 2 y AprilTags | Nodos de cámara, `image_transport`, calibración. AprilTags. Publicación TF desde detección. | Lab: Detectar tags, publicar TF `camera → tag`. |
| 10 | Localización relativa | `T(ref→obj) = T(cam→ref)⁻¹ × T(cam→obj)`. Integración con TF tree. Filtrado y estabilidad. | Lab: Localizar móvil respecto a referencia fija con cámara eye-in-hand. |

### Módulo 4 — Manipulación con MoveIt 2 (Semanas 11–13)

| Sem. | Tema | Contenido | Actividad Práctica |
|:---:|:---|:---|:---|
| 11 | Fundamentos de MoveIt 2 | `move_group`, IK (KDL, TRAC-IK), Planning Scene, SRDF, Setup Assistant. | Lab: MoveIt para manipulador, plan punto a punto. |
| 12 | Colisiones y tool frames | Objetos de colisión (add/remove/attach). TCP. Velocity/acceleration scaling. Planificación cartesiana. | Lab: Pick & place con collision objects. |
| 13 | MoveIt Task Constructor | Stages: generators, propagators, connectors. Pipelines multi-etapa. | Lab: Pipeline MTC pick-and-place. **Segundo Corte.** |

### Módulo 5 — Integración Multi-Robot (Semanas 14–15)

| Sem. | Tema | Contenido | Actividad Práctica |
|:---:|:---|:---|:---|
| 14 | Redes, micro-ROS y robótica distribuida | DDS multi-PC, `DOMAIN_ID`, QoS WiFi, micro-ROS en ESP32, agente UDP, diagnóstico de red. | Lab: Comunicación multi-PC. ESP32 vía micro-ROS. |
| 15 | Contratos de integración | Interfaces compartidas (TF, servicios, mensajes custom). Modos switchable (debug/producción). | Lab: Integración manipulador + móvil. Pruebas de contrato. |

### Módulo 6 — IA Aplicada y Proyecto Final (Semana 16)

| Sem. | Tema | Contenido | Actividad Práctica |
|:---:|:---|:---|:---|
| 16 | Percepción IA y sustentación | Modelos visión-lenguaje zero-shot. Razonamiento 3D. De-projection. Integración asíncrona. | **Sustentación proyecto + reto en vivo.** |

---

## 8. PROYECTO INTEGRADOR: Celda de Automatización Colaborativa

### 8.1. Descripción
El proyecto vertebral es el diseño e implementación de una **celda de automatización colaborativa** donde un manipulador industrial interactúa con unidades de transporte móviles. Esta arquitectura replica patrones industriales reales de logística intramuros, celdas de manufactura flexible, sistemas de inspección y robótica de servicio.

### 8.2. División en Equipos
La clase se organiza en **dos equipos de ingeniería** que trabajan en paralelo con un contrato de integración compartido (punto de anclaje TF + servicio ROS de coordinación).

| Equipo Manipulador | Equipo Móvil |
|:---|:---|
| Pipeline MoveIt 2, escena de colisiones, orquestación | Localización visual, URDF del móvil, servidor de posicionamiento |
| Publica frames del brazo y escena fija | Publica TF dinámica `ref_tag → obj_tag` |
| Consume frame de entrega del móvil vía TF2 | Consume solicitudes de posicionamiento vía servicio |

### 8.3. Reto de Sustentación
Modificación imprevista (15–20 min) diagnosticada con protocolo por capas:
1. ¿Sintaxis? → `xmllint`, `check_urdf`
2. ¿TF? → `view_frames`, `tf2_echo`  
3. ¿Red? → `ros2 topic list/hz`
4. ¿Lógica? → Logs, estado del action server

---

## 9. METODOLOGÍA

| Estrategia | Descripción |
|:---|:---|
| **Clase magistral contextualizada** | Máximo 45 min, vinculada al componente del proyecto que se implementará en laboratorio. |
| **Laboratorios guiados** | Guías paso a paso con criterios de éxito verificables antes de avanzar. |
| **Talleres de integración** | Ambos equipos conectan subsistemas e identifican fallas de interfaz. |
| **Code Review (PRs)** | Revisión de código entre pares en GitHub, simulando la industria. |
| **Depuración por capas** | Protocolo transversal: sintaxis → TF → red → lógica. |
| **Documentación como entregable** | Markdown técnico en el repositorio, evaluado como parte integral. |

**Plataformas:** Ubuntu 24.04 + ROS 2 Jazzy, RViz2, MoveIt 2, Gazebo, micro-ROS, Git/GitHub.

---

## 10. SISTEMA DE EVALUACIÓN

| Componente | Peso | Semana | Descripción |
|:---|:---:|:---:|:---|
| Laboratorios y talleres | 20% | Continua | Entregas semanales con criterios de éxito. |
| Primer Corte | 20% | 8 | Módulos 0–2: entorno, ROS 2 core, URDF/TF2. |
| Segundo Corte | 20% | 13 | Módulos 3–4: percepción + MoveIt 2. |
| Proyecto Integrador Final | 30% | 16 | Sustentación + reto en vivo + documentación. |
| Documentación y participación | 10% | Continua | Markdown, code reviews, trabajo colaborativo. |

> **Nota:** Todas las actividades están amparadas por el reglamento estudiantil vigente de la UMNG.

---

## 11. BIBLIOGRAFÍA

### Principal
1. Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming Robots with ROS.* O'Reilly.
2. Lentin, J. (2024). *ROS 2 Robotics Developer Guide.* Packt.
3. Corke, P. (2023). *Robotics, Vision and Control* (3rd ed.). Springer.
4. Lynch, K. M., & Park, F. C. (2017). *Modern Robotics.* Cambridge UP. [modernrobotics.org]
5. Siciliano, B., et al. (2009). *Robotics: Modelling, Planning and Control.* Springer.

### Complementaria
6. Documentación ROS 2 Jazzy: [docs.ros.org/en/jazzy](https://docs.ros.org/en/jazzy/)
7. Documentación MoveIt 2: [moveit.picknik.ai](https://moveit.picknik.ai/)
8. Repositorio del curso (`burger_delivery`): documentación técnica interna.
9. The Construct ROS 2 Courses: [theconstructsim.com](https://www.theconstructsim.com/)

---

## 12. MATERIAL COMPLEMENTARIO

| Recurso | Ubicación | Descripción |
|:---|:---|:---|
| Arquitectura del Sistema | `docs/architecture/ros_burger_delivery.md` | Red, nodos, TF tree, secuencia operativa. |
| Taller URDF y TF2 | `education/talleres/TALLER_URDF_TF.md` | Guía con ejercicios y criterios de éxito. |
| Taller CLI ROS 2 | `education/talleres/TALLER_ROS2_CLI.md` | Comandos básicos, nodos y tópicos en consola. |
| Marco Conceptual MoveIt 2 | `docs/manipulation/MARCO_CONCEPTUAL_MOVEIT2.md` | `move_group`, MTC, Planning Scene. |
| Proyecto Evaluable | `education/proyectos_evaluables/PROYECTO_INTERMEDIO_MOVEIT2_DELIVERY.md` | Rúbricas y contrato de integración. |
| Experimento IA | `docs/research/EXPERIMENTO_IA_LOCALIZACION_GEMINI.md` | Percepción zero-shot, de-projection 3D. |
| Guía de Redes | `docs/architecture/ros.md` | Topología, micro-ROS, localización 2D. |
| Fundamentos Git | `git-fundamentals/` | Tutoriales interactivos HTML. |
| Diagnóstico de Red | `network_setup/` | Scripts diagnóstico WiFi y DDS. |

---

## 13. COMPETENCIA DEL DOCENTE

- **Formación:** Posgrado en Mecatrónica, Robótica, Automatización o afines.
- **Experiencia técnica:** ROS/ROS 2, manipulación, visión computacional, sistemas embebidos.
- **Experiencia práctica:** Manipuladores industriales, MoveIt 2, integración multi-robot.
- **Pedagógica:** ABP, evaluación por competencias, tutorización de proyectos en equipo.

---

## 14. CONTROL DE CAMBIOS

| Cambio Realizado | Justificación | Acta |
|:---|:---|:---|
| Creación del syllabus | Nuevo curso electiva de profundización en robótica con ROS 2. | *Pendiente* |

---

> *"En la robótica colaborativa, los sistemas no fallan por defecto en sus partes, sino en las interfaces que los unen."*
