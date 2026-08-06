# 🔬 Documentación Técnica y de Investigación — Burger-Cell (Burger Delivery)

> **Título Formal del Proyecto:**  
> *Burger-Cell: An Open-Source Heterogeneous ROS 2 Framework for Kinova Gen3 Manipulation, AprilTag Perception, and VLM-Guided Spatial Reasoning*

Bienvenido a la sección de **documentación técnica, papers y reportes de investigación** del proyecto **Burger-Cell / Burger Delivery**.

---

## 📂 Estructura del Directorio

```text
docs/
├── research/                    # Experimentos, benchmarks e investigación en IA
│   ├── EXPERIMENTO_IA_LOCALIZACION_GEMINI.md
│   └── EXPERIMENTO_IA_LOCALIZACION_GEMINI.pdf
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

### 🤖 1. Investigación en Inteligencia Artificial y Razonamiento Espacial (`research/`)
- [**Razonamiento Espacial 3D y Localización con Gemini Robotics**](research/EXPERIMENTO_IA_LOCALIZACION_GEMINI.md): Estudio y arquitectura para localización *zero-shot* 3D basada en VLM (`gemini-robotics-er-1.6-preview`), affordances de agarre y triangulación multi-vista como alternativa a modelos clásicos como YOLO.

### 📐 2. Arquitectura del Sistema (`architecture/`)
- [**Documento Técnico de Arquitectura (Burger Delivery)**](architecture/ros_burger_delivery.md): Descripción global de la celda colaborativa, árbol TF completo, nodos y protocolos de comunicación.
- [**Fundamentos de ROS 2 y Redes**](architecture/ros.md): Guía de referencia de topología de red, micro-ROS y localización 2D.
- [**Visualizador Web URDF**](architecture/VISUALIZAR_URDF_WEB.md): Métodos para renderizar modelos robóticos URDF en navegadores web.

### 🦾 3. Manipulación y Control Kinova Gen3 (`manipulation/`)
- [**Marco Conceptual MoveIt 2**](manipulation/MARCO_CONCEPTUAL_MOVEIT2.md): Conceptos de `move_group`, Planning Scene y MoveIt Task Constructor (MTC).
- [**Mejoras de Movimiento y Ghost Visualizer**](manipulation/MEJORAS_MOVIMIENTO_KINOVA.md): Técnicas de mitigación de latencia y visualización de trayectorias.
- [**Pruebas de Movimiento y Coordenadas Cartesianas CLI**](manipulation/PRUEBAS_MOVIMIENTO.md): Validación cinemática y límites articulares.
- [**Tipos de Joints y Cinemática**](manipulation/joints.md): Análisis cinemático de articulaciones en ROS 2.
