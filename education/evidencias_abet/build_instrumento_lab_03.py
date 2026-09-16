#!/usr/bin/env python3
"""
Constructor para INSTRUMENTO_ABET_LAB_03_OPERACION_DISTRIBUIDA_KINOVA_TURNOS.docx
Genera el instrumento Word único de captura de evidencia, tablas de datos experimentales,
cuestionario de análisis técnico, evaluación y consolidación ABET para el Laboratorio 03:
Operación Distribuida del Kinova Gen3 (Estación Anfitriona, Monitores, RQT y Envío de Trayectorias por Turnos).
"""

from __future__ import annotations

from pathlib import Path

from docx import Document
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

ROOT = Path(__file__).resolve().parents[2]
TARGET = ROOT / "education" / "evidencias_abet" / "INSTRUMENTO_ABET_LAB_03_OPERACION_DISTRIBUIDA_KINOVA_TURNOS.docx"

FONT = "Arial"
BLUE = "1F4E79"
LIGHT_BLUE = "D9EAF7"
LIGHT_GRAY = "F2F2F2"

INDICATOR_21 = (
    "Indicador de desempeño 2.1: Diseña soluciones de software para control y monitoreo de robots, "
    "integrando contratos de comunicación (QoS, interfaces customizadas) y redes DDS robustas."
)
INDICATOR_22 = (
    "Indicador de desempeño 2.2: Incorpora restricciones de red, latencia, ancho de banda y seguridad "
    "en la integración de hardware heterogéneo (brazos robóticos, sensores, micro-ROS)."
)
INDICATOR_64 = (
    "Indicador de desempeño 6.4: Interpreta fallas y diagnósticos experimentales aplicando protocolos "
    "de diagnóstico por capas (Sintaxis -> TF -> Red -> Lógica) para aislar errores en hardware y software."
)
INDICATOR_31_33 = (
    "Indicador de desempeño 3.1 - 3.3: Elabora documentación técnica reproducible del sistema ROS 2 y "
    "comunica resultados experimentales de percepción, calibración y planificación de movimiento."
)
INDICATOR_41 = (
    "Indicador de desempeño 4.1: Identifica y mitiga riesgos de seguridad física, paradas de emergencia "
    "y protocolos de operación en celdas robóticas e industriales."
)
INDICATOR_51 = (
    "Indicador de desempeño 5.1: Define roles técnicos, coordina la ejecución en equipo y fomenta un "
    "entorno de trabajo colaborativo e inclusivo."
)

LEVELS = [
    ("N5", "475–500", "Excelente: evidencia completa, rigurosa, reproducible y explicada con profundidad analítica y matemática sobresaliente."),
    ("N4", "400–474", "Bueno: desempeño técnico correcto con omisiones menores que no comprometen la operación, trazabilidad ni seguridad."),
    ("N3", "300–399", "Aceptable: demuestra el desempeño esencial con evidencia verificable. Es el umbral individual de logro."),
    ("N2", "150–299", "Cumplimiento parcial: evidencia incompleta, métricas faltantes, errores en modo seco o fallas de protocolo."),
    ("N1", "0–149", "No cumple: evidencia mínima, fragmentaria o no funcional. Sin evidencia obligatoria se registra 0."),
]

LAB_CRITERIA = [
    {
        "code": "C1",
        "name": "Red DDS distribuida y conectividad (ROS_DOMAIN_ID=0)",
        "weight": 25,
        "so": "Student Outcome SO2",
        "indicator": f"{INDICATOR_21} / {INDICATOR_22}",
        "descriptors": [
            "Además de N4, automatiza la verificación de red con scripts reproducibles, analiza la asignación de puertos RTPS en el dominio 0 con CycloneDDS y justifica los mecanismos de aislamiento y descubrimiento multicast en Wi-Fi vs Ethernet.",
            "Configura ROS_DOMAIN_ID=0, verifica ping al Kinova (192.168.1.10) con RTT < 2 ms, diligencia completamente la Tabla 1 y valida el descubrimiento global de nodos mediante rqt_graph.",
            "Configura la IP en la subred 192.168.1.0/24, exporta ROS_DOMAIN_ID=0, comprueba ping al robot y visualiza los tópicos principales del dominio.",
            "Presenta conflictos de conectividad por discrepancia en variables de entorno (ROS_DOMAIN_ID incorrecto) o llena de forma incompleta la Tabla 1.",
            "No logra comunicación con la red del robot o no presenta evidencias obligatorias verificables.",
        ],
    },
    {
        "code": "C2",
        "name": "Estación anfitriona, sesión Kortex y telemetría",
        "weight": 25,
        "so": "Student Outcome SO2 / SO6",
        "indicator": f"{INDICATOR_21} / {INDICATOR_64}",
        "descriptors": [
            "Además de N4, analiza cuantitativamente el jitter de /joint_states a 100 Hz frente al ciclo Kortex a 1 kHz, y diseña protocolos de contingencia ante desconexión de hardware o fallos en el socket TCP 10000.",
            "Levanta correctamente la estación anfitriona (o se coordina con ella), verifica la sesión TCP Kortex activa a 1000 Hz, mide la tasa estable de /joint_states a 100 Hz y diligencia con exactitud la Tabla 2.",
            "Participa en la puesta en marcha de la anfitriona, confirma la publicación de /joint_states y llena los parámetros esenciales de la Tabla 2.",
            "Intenta levantar un driver duplicado compitiendo por el puerto Kortex o no logra verificar la frecuencia de telemetría articular.",
            "No comprende la convención de anfitriona única, bloquea la controladora Kortex o carece de evidencias de telemetría.",
        ],
    },
    {
        "code": "C3",
        "name": "Monitoreo remoto e introspección gráfica (RQT / RViz)",
        "weight": 20,
        "so": "Student Outcome SO6 / SO3",
        "indicator": f"{INDICATOR_64} / {INDICATOR_31_33}",
        "descriptors": [
            "Además de N4, correlaciona eventos transitorios en rqt_console con métricas de QoS en Wi-Fi, explica la persistencia de /robot_description vía TRANSIENT_LOCAL y optimiza la configuración de RViz para bajo consumo de ancho de banda.",
            "Despliega el monitor con identidad única (kinova_monitor_eqNN), visualiza el robot en RViz sin lanzar drivers locales, audita logs en rqt_console y llena completamente la Tabla 3.",
            "Ejecuta el nodo monitor en su estación, visualiza el robot en RViz en el dominio 0 y documenta su estado en la Tabla 3.",
            "Lanza el monitor con colisión de nombres de nodo o presenta fallos de visualización en RViz por falta de recepción de transformaciones TF.",
            "No logra ejecutar el monitoreo remoto o carece de capturas de RQT y RViz.",
        ],
    },
    {
        "code": "C4",
        "name": "Protocolo de turnos, modo seco y seguridad física",
        "weight": 20,
        "so": "Student Outcome SO4 / SO6",
        "indicator": f"{INDICATOR_41} / {INDICATOR_64}",
        "descriptors": [
            "Además de N4, justifica matemáticamente los límites de interpolación del controlador (max_joint_delta_rad), propone mecanismos de exclusión mutua por software y demuestra liderazgo en la coordinación segura de la celda.",
            "Ejecuta rigurosamente la validación en modo seco (dry_run:=true, código 0), solicita autorización verbal, supervisa la parada de emergencia, ejecuta el movimiento en joint_6 (±0.05 a ±0.08 rad) y llena la Tabla 4.",
            "Realiza la validación en modo seco antes del envío real, respeta el turno asignado, ejecuta el movimiento dentro de límites y diligencia la Tabla 4.",
            "Envía metas al robot sin validación previa en modo seco, excede el ángulo sugerido o no coordina el turno verbalmente.",
            "Incurre en actos inseguros en la celda robótica, envía trayectorias no autorizadas o carece de registros de turnos.",
        ],
    },
    {
        "code": "C5",
        "name": "Trazabilidad en rosbag, trabajo en equipo y cierre",
        "weight": 10,
        "so": "Student Outcome SO3 / SO5",
        "indicator": f"{INDICATOR_31_33} / {INDICATOR_51}",
        "descriptors": [
            "Además de N4, el rosbag MCAP incluye filtros quirúrgicos de tópicos, el Anexo A demuestra una complementariedad y dominio técnico individual sobresaliente, y propone mejoras al protocolo de apagado del robot.",
            "Graba y documenta el dataset MCAP con metadatos válidos, registra la transición OK->ERROR en el cierre ordenado, diligencia la Tabla 5 y el Anexo A individual evidencia autoría y comprensión.",
            "Entrega el informe con tablas llenas, registra el cierre del driver y aporta el Anexo A con respuestas verificables.",
            "Documento incompleto, tablas con datos faltantes, cierre abrupto del sistema o Anexo A con respuestas genéricas.",
            "No entrega el informe, no aporta rosbag ni evidencias de cierre, o el Anexo A demuestra ausencia de participación.",
        ],
    },
]


def shade(cell, fill: str) -> None:
    tc_pr = cell._tc.get_or_add_tcPr()
    node = tc_pr.find(qn("w:shd"))
    if node is None:
        node = OxmlElement("w:shd")
        tc_pr.append(node)
    node.set(qn("w:fill"), fill)


def format_cell(cell, *, bold: bool = False, size: float = 8, color: str | None = None, align: WD_ALIGN_PARAGRAPH | None = None) -> None:
    cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
    for paragraph in cell.paragraphs:
        if align is not None:
            paragraph.alignment = align
        paragraph.paragraph_format.space_after = Pt(0)
        paragraph.paragraph_format.space_before = Pt(0)
        for run in paragraph.runs:
            run.font.name = FONT
            run.font.size = Pt(size)
            run.bold = bold
            if color:
                run.font.color.rgb = RGBColor.from_string(color)


def style_table(table, *, header: bool = True, font_size: float = 8) -> None:
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.style = "Table Grid"
    for i, row in enumerate(table.rows):
        for cell in row.cells:
            if header and i == 0:
                shade(cell, BLUE)
                format_cell(cell, bold=True, size=font_size, color="FFFFFF", align=WD_ALIGN_PARAGRAPH.CENTER)
            else:
                if i % 2 == 0:
                    shade(cell, LIGHT_GRAY)
                format_cell(cell, size=font_size)


def add_heading(doc: Document, text: str, level: int = 1):
    p = doc.add_heading(text, level=level)
    for run in p.runs:
        run.font.name = FONT
        run.font.color.rgb = RGBColor.from_string(BLUE)
    p.paragraph_format.space_before = Pt(8)
    p.paragraph_format.space_after = Pt(3)
    return p


def add_table(doc: Document, headers: list[str], rows: list[list[str]], widths=None, alignments=None):
    table = doc.add_table(rows=1, cols=len(headers))
    for j, value in enumerate(headers):
        table.cell(0, j).text = str(value)
    for i, row in enumerate(rows):
        cells = table.add_row().cells
        for j, value in enumerate(row):
            cells[j].text = str(value)
    style_table(table, font_size=7.5)
    if widths:
        for row in table.rows:
            for j, width in enumerate(widths):
                if j < len(row.cells):
                    row.cells[j].width = Cm(width)
    if alignments:
        for row in table.rows[1:]:
            for j, align in enumerate(alignments):
                if j < len(row.cells):
                    row.cells[j].paragraphs[0].alignment = align
    return table


def add_note(doc: Document, text: str) -> None:
    table = doc.add_table(rows=1, cols=1)
    table.style = "Table Grid"
    shade(table.cell(0, 0), LIGHT_BLUE)
    table.cell(0, 0).text = text
    format_cell(table.cell(0, 0), size=8)


def add_response_box(doc: Document, prompt: str, lines: int = 4) -> None:
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(2)
    run = p.add_run(prompt)
    run.font.name = FONT
    run.font.size = Pt(8.5)
    run.font.italic = True

    box = doc.add_table(rows=1, cols=1)
    box.alignment = WD_TABLE_ALIGNMENT.CENTER
    box.style = "Table Grid"
    cell = box.cell(0, 0)
    shade(cell, "FAFAFA")
    cell.width = Cm(17.5)
    cp = cell.paragraphs[0]
    cp.paragraph_format.space_before = Pt(2)
    cp.paragraph_format.space_after = Pt(2)
    r = cp.add_run("Respuesta Técnica del Equipo:\n" + ("\n" * (lines - 1)))
    r.font.name = FONT
    r.font.size = Pt(8)
    r.font.color.rgb = RGBColor.from_string("555555")


def set_doc_defaults(doc: Document) -> None:
    section = doc.sections[0]
    section.top_margin = Cm(1.6)
    section.bottom_margin = Cm(1.6)
    section.left_margin = Cm(1.7)
    section.right_margin = Cm(1.7)
    styles = doc.styles
    styles["Normal"].font.name = FONT
    styles["Normal"].font.size = Pt(9)


def add_page_number_footer(doc: Document) -> None:
    p = doc.sections[0].footer.paragraphs[0]
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = p.add_run("ROB-ROS · Instrumento Único de Evidencia y Evaluación ABET — Lab 03 · 2026-2")
    run.font.name = FONT
    run.font.size = Pt(8)


def build_instrument():
    doc = Document()
    set_doc_defaults(doc)

    title = doc.add_paragraph()
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = title.add_run(
        "INSTRUMENTO DE EVIDENCIA Y CALIFICACIÓN ACADÉMICA\n"
        "LABORATORIO 03 — OPERACIÓN DISTRIBUIDA DEL KINOVA GEN3:\n"
        "ESTACIÓN ANFITRIONA, MONITORES, RQT Y ENVÍO DE TRAYECTORIAS POR TURNOS"
    )
    run.bold = True
    run.font.name = FONT
    run.font.size = Pt(13)
    run.font.color.rgb = RGBColor.from_string(BLUE)

    add_note(
        doc,
        "DOCUMENTO ÚNICO DE ENTREGA Y EVALUACIÓN ABET — Versión 1.0 (2026-2). "
        "Asociado a la Guía de Laboratorio 03 (GL-AA-F-1). Este instrumento contiene el informe técnico integral: "
        "las 5 tablas de registro experimental llenadas por el equipo, el cuestionario de análisis técnico respondido, "
        "el Anexo A de comprobación individual para certificar el logro de cada integrante y la rúbrica analítica "
        "docente para calificación sobre 500 y 5,0. La nota académica no se interpreta como nivel ABET agregado.",
    )

    # 1. Identificación
    add_heading(doc, "1. Identificación y Control")
    add_table(
        doc,
        ["Campo", "Registro Oficial"],
        [
            ["Programa Académico", "Ingeniería Mecatrónica"],
            ["Asignatura", "ROBOT OPERATING SYSTEM - ROS"],
            ["Periodo Académico", "2026-2"],
            ["Corte / Instrumento", "Segundo Corte / Laboratorios y evidencias experimentales"],
            ["Actividad Evaluada", "Laboratorio 03 — Operación Distribuida del Kinova Gen3 (Turnos, RQT y Modo Seco)"],
            ["Número de Grupo / Subgrupo", ""],
            ["Estudiante 1 (Nombre y Código)", ""],
            ["Estudiante 2 (Nombre y Código)", ""],
            ["Estudiante 3 (si aplica)", ""],
            ["Rol de Estación del Equipo", "Anfitriona (Cable Ethernet) ☐        Monitora Wi-Fi ☐"],
            ["Archivo de Entrega", "C2_L03_G<grupo>_<codigo1>_<codigo2>_v1.docx"],
            ["Fecha de Realización en Laboratorio", ""],
            ["Fecha de Entrega del Documento", ""],
            ["Docente Evaluador", "Ing. Henry Roncancio"],
            ["Versión del Instrumento", "Versión 1.0 (2026-2)"],
            ["Unidad de Análisis / Captura", "Equipo colaborativo con comprobación individual (Anexo A)"],
        ],
        [5, 12],
    )

    # 2. Parámetros de assessment
    add_heading(doc, "2. Parámetros de Assessment")
    add_table(
        doc,
        ["Parámetro", "Regla Institucional y Metodológica Adoptada"],
        [
            ["Población o cohorte", "Censo completo de estudiantes matriculados que presentan el Laboratorio 03 en el periodo 2026-2."],
            ["Momento de medición", "Sesión experimental presencial y entrega documental única con tablas llenas, rosbag MCAP y sustentación técnica."],
            ["Evaluador", "Docente titular de la asignatura ROBOT OPERATING SYSTEM - ROS."],
            ["Umbral individual de logro", "Nivel N3 o superior (puntaje mínimo de 300 sobre 500) en cada indicador evaluado."],
            ["Meta de cohorte", "Al menos el 70% de los estudiantes evaluables debe alcanzar el nivel N3 o superior en cada indicador."],
            ["Regla de muestreo", "Censo 100%: no se utiliza muestreo; se evalúa la totalidad de los estudiantes y equipos exigibles."],
            ["Evidencia faltante", "Entrega exigible sin evidencia obligatoria verificable: N1 con valor 0. Retiro oficial: NA / no evaluado."],
            ["Regla de trabajo colaborativo", "Informe único por equipo con tablas llenas más Anexo A individual obligatorio."],
        ],
        [5, 12],
    )

    # 3. Niveles Zubatronic
    add_heading(doc, "3. Niveles de Desempeño para Zubatronic/SGDE")
    add_table(doc, ["Nivel", "Intervalo", "Interpretación y Criterio de Logro"], [list(x) for x in LEVELS], [2, 3, 12])

    # 4. Alineación Criterios
    add_heading(doc, "4. Alineación de Criterios, RAE y Student Outcomes")
    add_table(
        doc,
        ["Criterio", "Peso", "Student Outcome Principal", "Indicador Literal del Programa", "Evidencia Directa Obligatoria"],
        [
            [
                f"{c['code']}. {c['name']}",
                f"{c['weight']}%",
                c["so"],
                c["indicator"],
                "Ping ICMP, ROS_DOMAIN_ID=0, rqt_graph, Tabla 1" if c['code'] == 'C1' else
                ("Sesión Kortex 1 kHz, /joint_states 100 Hz, Tabla 2" if c['code'] == 'C2' else
                ("kinova_monitor_eqNN, RViz Wi-Fi, rqt_console, Tabla 3" if c['code'] == 'C3' else
                ("Modo seco (dry_run:=true, cód 0), joint_6 visible, Tabla 4" if c['code'] == 'C4' else
                "Dataset MCAP, cierre ordenado OK->ERROR, Tabla 5, Anexo A")))
            ]
            for c in LAB_CRITERIA
        ],
        [4.2, 1.3, 2.5, 5.0, 4.5],
    )

    # 5. Registro de Evidencias
    add_heading(doc, "5. Registro de Evidencias de la Práctica (E1–E8)")
    add_table(
        doc,
        ["Código", "Evidencia Requerida", "Localizador en Documento / Repositorio / Archivo"],
        [
            ["E1", "Captura de terminal con ping ICMP exitoso al Kinova (192.168.1.10) y entre estaciones en la subred 192.168.1.0/24.", ""],
            ["E2", "Captura de verificación de entorno (echo $ROS_DOMAIN_ID) mostrando 0 y captura de rqt_graph mostrando topología.", ""],
            ["E3", "Captura de la anfitriona con sesión Kortex activa (puerto TCP 10000) y ros2 topic hz /joint_states a ~100 Hz.", ""],
            ["E4", "Captura de la monitora remota ejecutando kinova_monitor_eqNN y modelo del robot en RViz visualizado sobre Wi-Fi.", ""],
            ["E5", "Captura de rqt_console filtrando mensajes del dominio 0 y demostrando auditoría centralizada de eventos.", ""],
            ["E6", "Captura de ejecución del envío en Modo Seco (dry_run:=true) mostrando salida exitosa (código 0) y validación de límites.", ""],
            ["E7", "Captura de envío real al Kinova Gen3, mostrando el cambio angular en joint_6 (±0.05 a ±0.08 rad) verificado.", ""],
            ["E8", "Salida de ros2 bag info del dataset MCAP y captura de la monitora mostrando la transición de estado OK -> ERROR al apagar driver.", ""],
        ],
        [2, 11, 4.5],
    )

    # 6. Tablas Experimentales Llenas
    add_heading(doc, "6. Tablas de Registro Experimental (Diligenciadas por el Equipo)")
    add_note(doc, "Instrucciones: Registre en cada una de las tablas los datos cuantitativos y cualitativos obtenidos durante la sesión de laboratorio.")

    p = doc.add_paragraph("Tabla 1: Inventario de Interfaces de Red y Rol de Equipo")
    p.runs[0].bold = True
    add_table(
        doc,
        ["Equipo / Grupo", "Interfaz (eth0/wlan0)", "Dirección IP Asignada", "Máscara / Gateway", "Rol Asignado", "Ping Kinova 192.168.1.10 (RTT ms / Pérdida)"],
        [
            ["Estación Anfitriona", "eth0", "192.168.1.xxx", "255.255.255.0 / 192.168.1.1", "Anfitriona (Driver)", "RTT:        ms | Pérdida: 0%"],
            ["Equipo Propio", "", "", "255.255.255.0 / 192.168.1.1", "", "RTT:        ms | Pérdida:"],
            ["Remoto G01", "wlan0", "", "255.255.255.0 / 192.168.1.1", "Monitora Wi-Fi", "RTT:        ms | Pérdida:"],
            ["Remoto G02", "wlan0", "", "255.255.255.0 / 192.168.1.1", "Monitora Wi-Fi", "RTT:        ms | Pérdida:"],
            ["Remoto G03", "wlan0", "", "255.255.255.0 / 192.168.1.1", "Monitora Wi-Fi", "RTT:        ms | Pérdida:"],
            ["Remoto G04", "wlan0", "", "255.255.255.0 / 192.168.1.1", "Monitora Wi-Fi", "RTT:        ms | Pérdida:"],
        ],
        [3.0, 2.5, 3.0, 3.5, 2.5, 3.0],
    )

    p = doc.add_paragraph("Tabla 2: Estado y Telemetría de la Estación Anfitriona (Fase 1)")
    p.runs[0].bold = True
    add_table(
        doc,
        ["Parámetro de la Estación Anfitriona", "Valor Medido / Estado Experimental", "Método de Verificación / Comando Ejecutado"],
        [
            ["Hostname e IP de la anfitriona", "", "hostname -I"],
            ["Tipo de conexión física", "Cable Ethernet directo / Switch", "Inspección física de hardware"],
            ["RTT promedio de ping a 192.168.1.10", "ms", "ping -c 10 192.168.1.10"],
            ["PID y estado del nodo driver Kortex", "PID:              | Estado: Activo", "ps aux | grep kortex_bringup"],
            ["Frecuencia medida de /joint_states", "Hz (nominal 100 Hz)", "ros2 topic hz /joint_states"],
            ["Frecuencia de /burger/kinova/joint_states", "Hz", "ros2 topic hz /burger/kinova/joint_states"],
            ["Estado de sesión Kortex (puerto TCP 10000)", "Conectada / Estable", "ss -tulpn | grep 10000"],
            ["Archivo rosbag MCAP de registro", "", "ros2 bag info <nombre_bag>"],
        ],
        [6.0, 5.5, 6.0],
    )

    p = doc.add_paragraph("Tabla 3: Despliegue y Estado de Estaciones Monitoras (Fases 2 y 4)")
    p.runs[0].bold = True
    add_table(
        doc,
        ["Grupo", "Nodo Monitor", "IP Estación (Wi-Fi)", "rol_estacion", "Freq /joint_states (Hz)", "Habilitación Mov.", "RViz / rqt_graph", "Logs rqt_console", "Tiempo OK->ERROR"],
        [
            ["G01", "kinova_monitor_eq01", "", "monitor", "", "TRUE / FALSE", "OK", "OK", "s"],
            ["G02", "kinova_monitor_eq02", "", "monitor", "", "TRUE / FALSE", "OK", "OK", "s"],
            ["G03", "kinova_monitor_eq03", "", "monitor", "", "TRUE / FALSE", "OK", "OK", "s"],
            ["G04", "kinova_monitor_eq04", "", "monitor", "", "TRUE / FALSE", "OK", "OK", "s"],
            ["G05", "kinova_monitor_eq05", "", "monitor", "", "TRUE / FALSE", "OK", "OK", "s"],
        ],
        [1.5, 3.2, 2.5, 2.0, 2.2, 1.8, 1.8, 1.5, 1.0],
    )

    p = doc.add_paragraph("Tabla 4: Registro de Turnos de Movimiento Articular y Prueba Final (Fase 3)")
    p.runs[0].bold = True
    add_table(
        doc,
        ["Turno #", "Grupo", "Hora", "joint_6 Inicial (rad)", "Meta joint_6 (rad)", "Modo Seco (Código)", "Envío Real (Código)", "joint_6 Final (rad)", "Diff Real vs Esp", "Cierre"],
        [
            ["1", "G01", "", "", "(+0.05 a +0.08)", "0 (Éxito)", "0 (Éxito)", "", "", ""],
            ["2", "G02", "", "", "(-0.05 a -0.08)", "0 (Éxito)", "0 (Éxito)", "", "", ""],
            ["3", "G03", "", "", "(+0.05 a +0.08)", "0 (Éxito)", "0 (Éxito)", "", "", ""],
            ["4", "G04", "", "", "(-0.05 a -0.08)", "0 (Éxito)", "0 (Éxito)", "", "", ""],
            ["5", "G05", "", "", "(+0.05 a +0.08)", "0 (Éxito)", "0 (Éxito)", "", "", ""],
            ["Prueba Final", "Todos", "", "Origen autodescubierto", "Coreografía 25 deltas", "0 (Éxito)", "SUCCESSFUL", "Retorno origen", "0.000", "OK"],
        ],
        [1.2, 1.3, 1.4, 2.3, 2.4, 2.0, 2.0, 2.2, 1.7, 1.0],
    )

    p = doc.add_paragraph("Tabla 5: Incidentes, Anomalías y Diagnóstico Metódico por Capas")
    p.runs[0].bold = True
    add_table(
        doc,
        ["Momento / Fase", "Síntoma Observado", "Capa Diagnosticada (1:Red, 2:DDS, 3:Driver, 4:Lógica, 5:Protocolo)", "Procedimiento de Inspección", "Acción Correctiva y Resultado"],
        [
            ["Incidente 1", "", "", "", ""],
            ["Incidente 2", "", "", "", ""],
            ["Incidente 3", "", "", "", ""],
        ],
        [2.2, 4.0, 3.8, 3.8, 3.7],
    )

    # 7. Cuestionario de Análisis Técnico
    add_heading(doc, "7. Cuestionario de Análisis y Discusión Técnica (Respondido por el Equipo)")

    add_response_box(
        doc,
        "Pregunta 1: Unicidad del Driver y Consulta de Identidad en el Grafo\n"
        "Con base en la Tabla 2 y la identidad anunciada en el grafo (rqt_graph o /burger/kinova/identidad_estacion), "
        "argumente cómo la convención de estación anfitriona única y el dominio compartido permiten responder "
        "'¿quién tiene el control del robot?' desde cualquier estación sin necesidad de coordinación verbal previa.",
        lines=4,
    )

    add_response_box(
        doc,
        "Pregunta 2: Emisores Simultáneos y Política de Cancelación en follow_joint_trajectory\n"
        "En ROS 2 y ros2_control, el servidor de acción /joint_trajectory_controller/follow_joint_trajectory acepta metas "
        "de cualquier nodo en el dominio 0. Si dos estaciones envían una meta de trayectoria simultáneamente, ¿qué le ocurre "
        "a la primera meta y por qué? ¿Por qué el protocolo colaborativo de turnos es indispensable cuando todos comparten ROS_DOMAIN_ID=0?",
        lines=4,
    )

    add_response_box(
        doc,
        "Pregunta 3: Compromiso de Enlace Ethernet (Anfitriona) vs. Wi-Fi (Monitoras)\n"
        "Compare cuantitativamente la frecuencia y el jitter de /joint_states medidos en la estación anfitriona (cable Ethernet) "
        "frente a las estaciones monitoras (Wi-Fi). ¿Por qué la estación monitora puede operar sobre Wi-Fi mientras que la anfitriona "
        "debe estar obligatoriamente conectada por cable? Considere el bucle de control a 1 kHz del API Kortex.",
        lines=4,
    )

    add_response_box(
        doc,
        "Pregunta 4: Trazabilidad y Reconstrucción Cronológica a partir del rosbag MCAP\n"
        "A partir del archivo rosbag grabado (sesion_turnos_eqNN) y los registros de la Tabla 4, reconstruya la cronología exacta "
        "de un turno: demuestre qué nodo (por su sufijo _eqNN) emitió la meta, el instante en que el controlador la aceptó y el "
        "instante en que finalizó el movimiento.",
        lines=4,
    )

    add_response_box(
        doc,
        "Pregunta 5: Propuesta de Garantía de Turnos por Software vs. Protocolo Humano\n"
        "El nodo monitor publica el estado de 'habilitación de movimiento', pero el cliente CLI no lo consulta obligatoriamente "
        "antes de enviar. Proponga una arquitectura de software (por ejemplo, un servicio ROS 2 de concesión de turnos con Mutex "
        "distribuido en la anfitriona). ¿Qué ventajas operativas tendría y qué nuevos modos de falla introduciría?",
        lines=4,
    )

    add_response_box(
        doc,
        "Pregunta 6: Pérdida de Enlace Durante el Movimiento Físico\n"
        "Si durante la ejecución de una trayectoria articular se cae repentinamente el enlace Wi-Fi de la estación que envió la meta, "
        "¿se detiene inmediatamente el robot? Razone su respuesta identificando dónde reside físicamente el controlador de "
        "trayectoria y dónde reside el cliente de acción.",
        lines=4,
    )

    add_response_box(
        doc,
        "Pregunta 7: Aislamiento vs. Colaboración en DDS (Impacto del ROS_DOMAIN_ID)\n"
        "¿Qué ocurriría si un equipo olvida configurar el ROS_DOMAIN_ID=0 y ejecuta su monitor en el dominio por defecto (ROS_DOMAIN_ID=10)? "
        "¿Podría recibir telemetría o enviar trayectorias al Kinova? Explique por qué el dominio común es condición matemática necesaria.",
        lines=4,
    )

    add_response_box(
        doc,
        "Pregunta 8: Posicionamiento Absoluto vs. Deltas Relativos y Autodescubrimiento de Pose\n"
        "Compare la operación de safe_trajectory_client frente a safe_sequence_client. ¿Por qué en el cliente individual "
        "fue estrictamente necesario descubrir las posiciones absolutas reales de /joint_states antes de formular la meta "
        "para evitar el bloqueo por max_joint_delta_rad (0.10 rad), mientras que el cliente de secuencia pudo ejecutarse desde "
        "cualquier pose sin transcribir coordenadas a mano? ¿Qué riesgos y ventajas de seguridad introduce cada enfoque en entornos industriales colaborativos?",
        lines=4,
    )

    # 8. Anexo A
    add_heading(doc, "8. Anexo A — Comprobación Individual de Desempeño")
    add_note(doc, "Propósito ABET: Verifica la autoría, comprensión técnica y contribución directa individual de cada estudiante.")

    for est_num in [1, 2]:
        p = doc.add_paragraph(f"Estudiante {est_num}: __________________________________________________ Código: ____________________")
        p.runs[0].bold = True
        add_table(
            doc,
            ["Pregunta Individual de Verificación", "Respuesta / Evidencia Directa del Estudiante"],
            [
                ["1. Rol y tareas técnicas desarrolladas: Describa sus responsabilidades específicas en la práctica (ej. configuración de red, ejecución de la anfitriona, monitoreo RQT, cálculo de trayectorias, modo seco o rosbag).", ""],
                ["2. Justificación de ROS_DOMAIN_ID=0: Explique por qué todos los equipos debieron converger al dominio 0 y qué sucede en el descubrimiento DDS cuando coinciden los dominios en la misma LAN.", ""],
                ["3. Explicación del Modo Seco y deltas angulares: Justifique técnicamente qué valida el cliente en modo seco (dry_run:=true) y por qué se restringió el desplazamiento de joint_6 a un rango de ±0.05 a ±0.08 rad.", ""],
                ["4. Diagnóstico por capas de una anomalía: Describa un incidente o advertencia observado durante la sesión (en rqt_console, consola o RViz) y cómo lo analizó mediante el protocolo por capas.", ""],
                ["5. Autoría y reproducibilidad: Identifique qué comandos, filas de tablas, capturas o secciones del rosbag fueron ejecutados y documentados directamente por usted.", ""],
            ],
            [7.5, 10.0],
        )
        p = doc.add_paragraph(f"Estado de comprobación individual Estudiante {est_num}: Verificada ☐     Insuficiente ☐     NA autorizado ☐     Firma: ________________________")
        p.paragraph_format.space_after = Pt(6)

    # 9. Rúbricas Docentes
    add_heading(doc, "9. Selección del Nivel Alcanzado por Criterio (Rúbricas Docentes)")
    for criterion in LAB_CRITERIA:
        add_heading(
            doc,
            f"{criterion['code']}. {criterion['name']} — {criterion['weight']}% — {criterion['so']}",
            level=2,
        )
        rows = []
        for level, descriptor in zip(LEVELS, criterion["descriptors"]):
            rows.append(["☐", f"{level[0]} — {level[1]}", descriptor])
        add_table(doc, ["Marque", "Nivel", "Evidencia observable"], rows, [1.5, 3, 12.5])
        doc.add_paragraph(f"Nivel {criterion['code']} marcado: ________     Valor Zubatronic (0–500): ________")

    # 10. Consolidado Final
    add_heading(doc, "10. Consolidado Final de Calificación Académica")
    add_table(
        doc,
        ["Criterio", "Peso Oficial (%)", "Nivel Marcado", "Valor Obtenido (0–500)", "Aporte Ponderado"],
        [
            [f"{c['code']}. {c['name']}", f"{c['weight']}%", "", "", ""] for c in LAB_CRITERIA
        ] + [["TOTAL CONSOLIDADO", "100%", "", "", "________ / 500"]],
        [6.5, 2.5, 2.2, 3.3, 3.0],
    )
    add_note(
        doc,
        "Nota Académica sobre 5,0 = Nota Consolidada sobre 500 ÷ 100. "
        "Aporte a Laboratorios del Segundo Corte: L₂ = Nota Académica sobre 5,0.",
    )

    add_table(
        doc,
        ["Resultado Oficial de la Actividad", "Registro Oficial"],
        [
            ["Nota de Laboratorio 03 sobre 500 puntos", "__________ / 500"],
            ["Nota Académica Oficial sobre 5,0", "__________ / 5,0"],
            ["Número de Criterios en Nivel N3 o superior (Umbral Individual)", "_____ / 5"],
            ["¿Cumple el Umbral Individual de Logro ABET (todos en N3+)?", "SÍ ☐         NO ☐"],
        ],
        [8.5, 9.0],
    )

    # 11. Consolidación ABET
    add_heading(doc, "11. Consolidación ABET y Cierre de Mejora Continua")
    add_table(
        doc,
        ["Student Outcome / Indicador Evaluado", "N Evaluables", "N en N3+", "% Logro", "Meta", "Hallazgo Docente"],
        [
            ["SO2 / Indicador 2.1 y 2.2 (Red DDS y Hardware)", "", "", "", "70%", ""],
            ["SO6 / Indicador 6.4 (Diagnóstico por Capas)", "", "", "", "70%", ""],
            ["SO3 / Indicador 3.1 y 3.3 (Documentación y Trazabilidad)", "", "", "", "70%", ""],
            ["SO4 / Indicador 4.1 (Seguridad Física y Celdas)", "", "", "", "70%", ""],
            ["SO5 / Indicador 5.1 (Trabajo en Equipo y Roles)", "", "", "", "70%", ""],
        ],
        [5.0, 2.0, 2.0, 2.0, 1.8, 4.7],
    )

    add_table(
        doc,
        ["Campo de Cierre de Ciclo de Mejora Continua", "Registro de Gestión Pedagógica"],
        [
            ["Decisión derivada del hallazgo", ""],
            ["Acción de mejora continua pedagógica y técnica", ""],
            ["Responsable de la acción", "Ing. Henry Roncancio"],
            ["Fecha prevista de seguimiento", ""],
            ["Evidencia de seguimiento", ""],
            ["Resultado observado en el segundo ciclo", ""],
        ],
        [6.0, 11.5],
    )

    # 12. Cierre y Firmas
    add_heading(doc, "12. Cierre y Firmas de Conformidad")
    doc.add_paragraph("Docente Evaluador: Ing. Henry Roncancio           Fecha de Evaluación: ____________________")
    doc.add_paragraph("Firma del Docente Evaluador: __________________________________________________")
    doc.add_paragraph("Observaciones Finales y Retroalimentación Pedagógica:\n________________________________________________________________________________\n________________________________________________________________________________")

    add_page_number_footer(doc)
    doc.core_properties.title = "Instrumento ABET Laboratorio 03 — Operación Distribuida Kinova Gen3"
    doc.core_properties.subject = "ROBOT OPERATING SYSTEM - ROS · 2026-2"
    doc.core_properties.author = "Ing. Henry Roncancio"
    doc.core_properties.comments = "Versión 1.0. Documento único de entrega estudiantil y evaluación ABET con tablas experimentales llenas, cuestionario técnico y Anexo A."
    TARGET.parent.mkdir(parents=True, exist_ok=True)
    doc.save(str(TARGET))
    print(f"Instrumento ABET Word generado exitosamente en: {TARGET}")


if __name__ == "__main__":
    build_instrument()
