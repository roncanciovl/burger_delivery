# 🔬 Documentación Técnica y de Investigación — Burger-Cell (Burger Delivery)

> **Título Formal del Proyecto:**  
> *Burger-Cell: An Open-Source Heterogeneous ROS 2 Framework for Kinova Gen3 Manipulation, AprilTag Perception, and VLM-Guided Spatial Reasoning*

Bienvenido a la sección de **documentación técnica, papers y reportes de investigación** del proyecto **Burger-Cell / Burger Delivery**.

---

## 📂 Estructura del Directorio

```text
docs/
├── research/                    # Experimentos, benchmarks e investigación en IA y Redes
│   ├── EXPERIMENTO_IA_LOCALIZACION_GEMINI.md
│   ├── EXPERIMENTO_IA_LOCALIZACION_GEMINI.pdf
│   ├── EXPERIMENTO_QOS_TELEMETRIA.md
│   ├── PLAN_PUBLICACIONES_Y_RELEASES_ZENODO.md
│   ├── PLAN_PUBLICACIONES_Y_RELEASES_ZENODO.pdf
│   ├── PROPUESTA_VINCULACION_GIDAM.md
│   ├── PROPUESTA_VINCULACION_GIDAM.pdf
│   └── EXPEDIENTE_AVAL_GIDAM.md
├── architecture/                # Especificaciones formales del sistema y cinemática
│   ├── ros_burger_delivery.md
│   ├── ros_burger_delivery.pdf
│   ├── ros.md
│   ├── ros.pdf
│   └── VISUALIZAR_URDF_WEB.md
└── manipulation/                # Control cinemático, MoveIt 2 y optimización
    ├── MARCO_CONCEPTUAL_MOVEIT2.md
    ├── MEJORAS_MOVIMIENTO_KINOVA.md
    ├── PRUEBAS_MOVIMIENTO.md
    └── joints.md
```

---

## 📑 Artículos y Reportes Destacados

### 🤖 1. Investigación en Inteligencia Artificial, QoS y Telemetría (`research/`)
- [**Plan Estratégico de Publicaciones y Releases Zenodo (2026-2027) [Markdown]**](research/PLAN_PUBLICACIONES_Y_RELEASES_ZENODO.md) | [**[Versión PDF]**](research/PLAN_PUBLICACIONES_Y_RELEASES_ZENODO.pdf): Cronograma editorial, gestión de datasets, reproducibilidad y ruta de aval en trámite con GIDAM (Categoría A, UMNG).
- [**Propuesta de Vinculación y Aval Académico — GIDAM [Markdown]**](research/PROPUESTA_VINCULACION_GIDAM.md) | [**[Versión PDF]**](research/PROPUESTA_VINCULACION_GIDAM.pdf): solicitud vigente de aval académico con GIDAM (Categoría A, UMNG), en trámite.
- [**Expediente de Soporte para Aval — GIDAM**](research/EXPEDIENTE_AVAL_GIDAM.md): evidencia disponible, límites y condiciones de cierre para S15.
- [**Razonamiento Espacial 3D y Localización con Gemini Robotics**](research/EXPERIMENTO_IA_LOCALIZACION_GEMINI.md): Estudio y arquitectura para localización *zero-shot* 3D basada en VLM (`gemini-robotics-er-1.6-preview`), affordances de agarre y triangulación multi-vista como alternativa a modelos clásicos como YOLO.
- [**Protocolo Experimental: QoS, Jitter y Telemetría Robótica**](research/EXPERIMENTO_QOS_TELEMETRIA.md): Metodología cuantitativa y matemática para evaluar el impacto de la degradación inalámbrica en el seguimiento articular del Kinova Gen3 y la latencia de percepción espacial.

### 📐 2. Arquitectura del Sistema y Redes (`architecture/` & `network_setup/`)
- [**Documento Técnico de Arquitectura (Burger Delivery)**](architecture/ros_burger_delivery.md): Descripción global de la celda colaborativa, árbol TF completo, nodos y protocolos de comunicación.
- [**Guía de Configuración de Red y Firewall ROS 2**](../network_setup/ROS2_NETWORK_CONFIG.md): Configuración de dominio, CycloneDDS, modo reflejado en WSL2, UDP dinámico limitado a la subred ROS y diagnóstico de multicast.
- [**Controles y Configuración del Monitor de Red**](../network_setup/MONITOR_RED_CONTROLES_Y_CONFIGURACION.md): Referencia operativa de todos los botones, filtros, selectores, indicadores, variables de entorno, API y límites de interpretación.
- [**Manual Completo de la UI del Monitor de Red**](../network_setup/MONITOR_UI_GUIA.md): Explicación exhaustiva del dashboard web, KPIs, modo experimento de telemetría y visualizador de topología.
- [**Fundamentos de ROS 2 y Redes**](architecture/ros.md): Guía de referencia de topología de red, micro-ROS y localización 2D.
- [**Visualizador Web URDF**](architecture/VISUALIZAR_URDF_WEB.md): Métodos para renderizar modelos robóticos URDF en navegadores web.

### 🦾 3. Manipulación y Control Kinova Gen3 (`manipulation/`)
- [**Marco Conceptual MoveIt 2**](manipulation/MARCO_CONCEPTUAL_MOVEIT2.md): Conceptos de `move_group`, Planning Scene y MoveIt Task Constructor (MTC).
- [**Mejoras de Movimiento y Ghost Visualizer**](manipulation/MEJORAS_MOVIMIENTO_KINOVA.md): Técnicas de mitigación de latencia y visualización de trayectorias.
- [**Pruebas de Movimiento y Coordenadas Cartesianas CLI**](manipulation/PRUEBAS_MOVIMIENTO.md): Validación cinemática y límites articulares.
- [**Tipos de Joints y Cinemática**](manipulation/joints.md): Análisis cinemático de articulaciones en ROS 2.
