#!/usr/bin/env python3
"""
Constructor para GUIA_LAB_03_OPERACION_DISTRIBUIDA_KINOVA_TURNOS.docx
Genera la Guía de Laboratorio 03 basada en la plantilla institucional Formato_Guias_de_Laboratorio.docx (GL-AA-F-1),
incorporando la arquitectura distribuida del Kinova Gen3, estación anfitriona, monitores por Wi-Fi,
herramientas RQT (rqt_graph y rqt_console), protocolo de turnos y validación en Modo Seco.
"""

from __future__ import annotations

import re
from copy import deepcopy
from pathlib import Path

from docx import Document
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

ROOT = Path(__file__).resolve().parents[2]
TEMPLATE = ROOT / "education" / "guias_laboratorio" / "templates" / "Formato_Guias_de_Laboratorio.docx"
TARGET = ROOT / "education" / "guias_laboratorio" / "GUIA_LAB_03_OPERACION_DISTRIBUIDA_KINOVA_TURNOS.docx"

FONT = "Arial"
BLUE = "1F4E79"
LIGHT_BLUE = "D9EAF7"
LIGHT_GRAY = "F2F2F2"


def clean(text: str) -> str:
    text = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r"\1", text)
    text = text.replace("*", "").replace("`", "")
    text = re.sub(r"^\s*>\s?", "", text)
    text = re.sub(r"^\s*#+\s*", "", text)
    text = re.sub(r"^\s*\*\s+", "", text)
    return text.strip()


def set_cell_shading(cell, fill: str):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_margins(cell, top=90, start=90, bottom=90, end=90):
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for margin, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{margin}"))
        if node is None:
            node = OxmlElement(f"w:{margin}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def set_repeat_header(row):
    tr_pr = row._tr.get_or_add_trPr()
    marker = OxmlElement("w:tblHeader")
    marker.set(qn("w:val"), "true")
    tr_pr.append(marker)


def prevent_row_split(row):
    tr_pr = row._tr.get_or_add_trPr()
    tr_pr.append(OxmlElement("w:cantSplit"))


def replace_text_preserving_cell(cell, text: str, *, bold=False, size=8, align=None, color=None):
    cell.text = ""
    p = cell.paragraphs[0]
    if align is not None:
        p.alignment = align
    p.paragraph_format.space_after = Pt(0)
    p.paragraph_format.space_before = Pt(0)
    r = p.add_run(clean(text))
    r.bold = bold
    r.font.name = FONT
    r.font.size = Pt(size)
    if color:
        r.font.color.rgb = RGBColor.from_string(color)
    cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
    set_cell_margins(cell)


def add_field(run, instruction: str):
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = instruction
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    text = OxmlElement("w:t")
    text.text = "1"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run._r.extend([begin, instr, separate, text, end])


def configure_footer(doc):
    footer = doc.sections[0].footer
    footer.is_linked_to_previous = False
    for table in footer.tables:
        table._element.getparent().remove(table._element)
    for p in footer.paragraphs[1:]:
        p._element.getparent().remove(p._element)
    p = footer.paragraphs[0]
    p.clear()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_after = Pt(2)
    r = p.add_run("Página ")
    r.font.name = FONT
    r.font.size = Pt(8)
    add_field(r, "PAGE")
    r = p.add_run(" de ")
    r.font.name = FONT
    r.font.size = Pt(8)
    add_field(r, "NUMPAGES")
    r = p.add_run(
        "\nEl uso no autorizado de su contenido, así como su reproducción total o parcial, "
        "estará en contra de los derechos de autor."
    )
    r.font.name = FONT
    r.font.size = Pt(7)


def clear_body(doc):
    body = doc._element.body
    sect_pr = body.sectPr
    for child in list(body):
        if child is not sect_pr:
            body.remove(child)


def add_page_break(doc):
    p = doc.add_paragraph()
    p.add_run().add_break(WD_BREAK.PAGE)


def add_body(doc, text: str, *, bold=False, italic=False, align=WD_ALIGN_PARAGRAPH.JUSTIFY,
             size=9, left=0, first_line=0, keep=False):
    p = doc.add_paragraph(style="Normal")
    p.alignment = align
    pf = p.paragraph_format
    pf.space_before = Pt(0)
    pf.space_after = Pt(4)
    pf.line_spacing_rule = WD_LINE_SPACING.SINGLE
    pf.left_indent = Cm(left)
    pf.first_line_indent = Cm(first_line)
    pf.keep_with_next = keep
    r = p.add_run(clean(text))
    r.font.name = FONT
    r.font.size = Pt(size)
    r.bold = bold
    r.italic = italic
    return p


def add_section_heading(doc, text: str, *, page_break=False):
    if page_break:
        add_page_break(doc)
    p = doc.add_paragraph(style="Normal")
    p.paragraph_format.space_before = Pt(8)
    p.paragraph_format.space_after = Pt(5)
    p.paragraph_format.keep_with_next = True
    r = p.add_run(clean(text).upper())
    r.font.name = FONT
    r.font.size = Pt(10)
    r.bold = True
    r.font.color.rgb = RGBColor.from_string(BLUE)
    return p


def add_subheading(doc, text: str):
    p = doc.add_paragraph(style="Normal")
    p.paragraph_format.space_before = Pt(5)
    p.paragraph_format.space_after = Pt(3)
    p.paragraph_format.keep_with_next = True
    r = p.add_run(clean(text))
    r.font.name = FONT
    r.font.size = Pt(9)
    r.bold = True
    r.font.color.rgb = RGBColor.from_string(BLUE)
    return p


def add_list_item(doc, text: str, number: int | None = None, level=0):
    marker = f"{number}. " if number is not None else "• "
    p = add_body(doc, marker + clean(text), align=WD_ALIGN_PARAGRAPH.LEFT, left=0.55 + 0.45 * level)
    p.paragraph_format.first_line_indent = Cm(-0.45)
    return p


def add_code_box(doc, lines: list[str]):
    table = doc.add_table(rows=1, cols=1)
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = True
    cell = table.cell(0, 0)
    set_cell_shading(cell, LIGHT_GRAY)
    set_cell_margins(cell, 100, 140, 100, 140)
    cell.text = ""
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(0)
    for i, line in enumerate(lines):
        if i:
            p.add_run("\n")
        r = p.add_run(clean(line))
        r.font.name = "Courier New"
        r.font.size = Pt(8)
    add_body(doc, "", size=2)


def add_callout(doc, label: str, text: str):
    table = doc.add_table(rows=1, cols=1)
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    cell = table.cell(0, 0)
    set_cell_shading(cell, "FFF2CC" if label == "ADVERTENCIA" else LIGHT_BLUE)
    set_cell_margins(cell, 120, 140, 120, 140)
    cell.text = ""
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(0)
    r = p.add_run(label + ": ")
    r.bold = True
    r.font.name = FONT
    r.font.size = Pt(8)
    r = p.add_run(clean(text))
    r.font.name = FONT
    r.font.size = Pt(8)
    add_body(doc, "", size=2)


def apply_model_table_properties(table, model):
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = True
    old_pr = table._tbl.tblPr
    table._tbl.remove(old_pr)
    table._tbl.insert(0, deepcopy(model._tbl.tblPr))
    for row in table.rows:
        prevent_row_split(row)
        for cell in row.cells:
            set_cell_margins(cell)


def add_grid_table(doc, headers: list[str], rows: list[list[str]], *, widths: list[float] | None = None,
                   alignments: list[WD_ALIGN_PARAGRAPH] | None = None, model_table=None):
    table = doc.add_table(rows=len(rows) + 1, cols=len(headers))
    if model_table is not None:
        apply_model_table_properties(table, model_table)
    else:
        table.alignment = WD_TABLE_ALIGNMENT.CENTER
        table.autofit = True
        for row in table.rows:
            prevent_row_split(row)
            for cell in row.cells:
                set_cell_margins(cell)

    set_repeat_header(table.rows[0])
    for j, h in enumerate(headers):
        cell = table.cell(0, j)
        set_cell_shading(cell, BLUE)
        align = alignments[j] if alignments else WD_ALIGN_PARAGRAPH.CENTER
        replace_text_preserving_cell(cell, h, bold=True, size=8, align=align, color="FFFFFF")

    for i, row in enumerate(rows, start=1):
        bg = LIGHT_GRAY if i % 2 == 1 else "FFFFFF"
        for j, val in enumerate(row):
            cell = table.cell(i, j)
            if bg != "FFFFFF":
                set_cell_shading(cell, bg)
            align = alignments[j] if alignments else (WD_ALIGN_PARAGRAPH.CENTER if j == 0 else WD_ALIGN_PARAGRAPH.LEFT)
            replace_text_preserving_cell(cell, val, size=8, align=align)

    if widths:
        for row in table.rows:
            for j, w in enumerate(widths):
                if j < len(row.cells):
                    row.cells[j].width = Cm(w)

    add_body(doc, "", size=2)
    return table


def main():
    doc = Document(str(TEMPLATE))

    cover_header = deepcopy(doc.tables[0]._tbl)
    cover_id = deepcopy(doc.tables[1]._tbl)
    cover_sign = deepcopy(doc.tables[2]._tbl)
    changes_model = deepcopy(doc.tables[3]._tbl)
    model_table = doc.tables[4]
    approval_model = deepcopy(doc.tables[6]._tbl)

    clear_body(doc)
    body = doc._element.body
    sect_pr = body.sectPr

    # 1. Página inicial institucional
    body.insert(body.index(sect_pr), cover_header)
    cover = doc.tables[-1]
    replace_text_preserving_cell(cover.cell(0, 1), "Fecha Emisión:\n2026/09/16", bold=True, size=8, align=WD_ALIGN_PARAGRAPH.CENTER)
    replace_text_preserving_cell(cover.cell(1, 1), "Revisión No.:\n1", bold=True, size=8, align=WD_ALIGN_PARAGRAPH.CENTER)
    page_cell = cover.cell(1, 2)
    page_cell.text = ""
    page_p = page_cell.paragraphs[0]
    page_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    page_p.paragraph_format.space_after = Pt(0)
    page_r = page_p.add_run("Página ")
    page_r.font.name = FONT
    page_r.font.size = Pt(8)
    page_r.bold = True
    add_field(page_r, "PAGE")
    page_r = page_p.add_run(" de ")
    page_r.font.name = FONT
    page_r.font.size = Pt(8)
    page_r.bold = True
    add_field(page_r, "NUMPAGES")
    add_body(doc, "", size=4)

    body.insert(body.index(sect_pr), cover_id)
    identification = doc.tables[-1]
    replace_text_preserving_cell(
        identification.cell(0, 0),
        "Laboratorio de: ROBOT OPERATING SYSTEM - ROS",
        bold=True, size=9,
    )
    replace_text_preserving_cell(
        identification.cell(1, 0),
        "Título de Laboratorio: Práctica 3 (Segundo Corte). Operación distribuida del Kinova Gen3: "
        "Estación anfitriona, monitores, RQT y envío de trayectorias por turnos",
        bold=True, size=9,
    )
    add_body(doc, "", size=6)

    body.insert(body.index(sect_pr), cover_sign)
    signatures = doc.tables[-1]
    replace_text_preserving_cell(signatures.cell(0, 0), "Elaborado por:\n\nIng. Henry Roncancio\nDocente Asignatura ROS", size=8, align=WD_ALIGN_PARAGRAPH.CENTER)
    replace_text_preserving_cell(signatures.cell(0, 1), "Revisado por:\n\nDirector de Programa\nIngeniería Mecatrónica", size=8, align=WD_ALIGN_PARAGRAPH.CENTER)
    replace_text_preserving_cell(signatures.cell(0, 2), "Aprobado por:\n\nDecano(a)\nFacultad de Ingeniería", size=8, align=WD_ALIGN_PARAGRAPH.CENTER)

    add_page_break(doc)
    add_section_heading(doc, "Control de Cambios de la Guía de práctica")
    body.insert(body.index(sect_pr), changes_model)
    changes = doc.tables[-1]
    for row in changes.rows[1:]:
        for cell in row.cells:
            replace_text_preserving_cell(cell, "", size=8)
    change_rows = [
        [
            "Creación e implementación de la Guía de Laboratorio 03",
            "Diseño del protocolo de operación distribuida del manipulador Kinova Gen3 (6-DOF) en LAN única 192.168.1.0/24, definiendo la convención de estación anfitriona por cable Ethernet y monitoras por Wi-Fi.",
            "16/09/2026",
        ],
        [
            "Reestructuración de Fase 0 y unificación en ROS_DOMAIN_ID=0",
            "Se elimina la simulación en dominios separados y se unifica a todos los grupos en el dominio 0 para interactuar con el robot físico en la IP 192.168.1.10.",
            "16/09/2026",
        ],
        [
            "Guarda contra segundo driver y convención eqNN",
            "Incidente del 16/09/2026: una estación monitora lanzó un segundo driver y le quitó el control a la anfitriona (WRONG_SERVOING_MODE, movimiento a tirones). Se corrige la explicación de la sesión Kortex, se documenta la comprobación automática del launch (check_existing_driver) con sus límites, se define cómo se configura el sufijo eqNN y se corrige la frecuencia del ciclo de control (100 Hz).",
            "17/09/2026",
        ],
        [
            "Ampliación del rango del turno individual: de joint_6 a joint_1",
            "Con joint_6 acotado a 0.10 rad el movimiento (~12 mm en las puntas de la pinza) no era visible desde las mesas de trabajo. El turno pasa a mover joint_1 (giro de base) con 0.20 a 0.30 rad, arco de ~150 mm a 4 °/s, y max_joint_delta_rad sube de 0.10 a 0.35 rad. Los límites de carrera del URDF y la secuencia autónoma no se modifican.",
            "17/09/2026",
        ],
        [
            "Integración de RQT y formalización del Modo Seco",
            "Inclusión de rqt_graph para topología, rqt_console para logs /rosout y profundización de la validación matemática de trayectorias previas al envío físico.",
            "16/09/2026",
        ],
    ]
    for i, values in enumerate(change_rows, start=1):
        if i >= len(changes.rows):
            changes.add_row()
        for j, value in enumerate(values):
            replace_text_preserving_cell(changes.cell(i, j), value, size=8)
    set_repeat_header(changes.rows[0])

    add_page_break(doc)
    for label, value in [
        ("FACULTAD O UNIDAD ACADÉMICA:", "Facultad de Ingeniería"),
        ("PROGRAMA:", "Ingeniería Mecatrónica"),
        ("ASIGNATURA:", "ROBOT OPERATING SYSTEM - ROS"),
        ("SEMESTRE:", "VIII – IX"),
    ]:
        p = add_body(doc, "", keep=True)
        r = p.add_run(label + " ")
        r.font.name = FONT
        r.font.size = Pt(9)
        r.bold = True
        r = p.add_run(value)
        r.font.name = FONT
        r.font.size = Pt(9)

    # 2. INTRODUCCIÓN
    add_section_heading(doc, "2. INTRODUCCIÓN")
    add_subheading(doc, "2.1. Contexto Teórico y Arquitectura Multi-Estación")
    add_body(doc,
        "En celdas de automatización colaborativa, un único robot manipulador de alta gama debe ser operado por múltiples estaciones de ingeniería. "
        "El laboratorio cuenta con un manipulador Kinova Gen3 de 6 GDL con pinza Robotiq 2F-85 y varios grupos de trabajo. "
        "La controladora Kortex (TCP 10000 y UDP 10001 de tiempo real) acepta varias sesiones a la vez y NO rechaza a un segundo driver. "
        "Lo que es único es el modo de servo del brazo, y kortex_driver lo cambia al arrancar y al cerrarse sin coordinarse con nadie. "
        "Si un segundo computador lanza kortex_bringup contra el mismo robot, le quita el control al driver que ya estaba trabajando: "
        "sus comandos fallan con WRONG_SERVOING_MODE y el brazo se mueve a tirones mientras ambos envían consignas a 100 Hz (incidente del 16/09/2026). "
        "El ROS_DOMAIN_ID no protege contra esto, porque el modo de servo vive por debajo de ROS 2."
    )
    add_body(doc,
        "Para operar el robot de forma segura y compartida, se implementa la convención de estación anfitriona:\n"
        "1. Dominio Común: Todas las estaciones operan estrictamente en ROS_DOMAIN_ID=0.\n"
        "2. Estación Anfitriona Única: Una sola PC conectada por cable Ethernet ejecuta el driver del robot real (start_driver:=true robot_ip:=192.168.1.10). Como respaldo, el launch del paquete busca durante hasta 6 s un driver ya activo (check_existing_driver) y, si lo encuentra, emite WARNING y NO lanza el driver.\n"
        "3. Estaciones Monitoras: Las computadoras de los estudiantes se conectan por Wi-Fi como monitoras (start_driver:=false), consumiendo /joint_states y publicando diagnóstico.\n"
        "4. Protocolo de Turnos: Como el controlador acepta metas de cualquier estación del dominio 0, el movimiento físico se disciplina mediante un protocolo colaborativo entre personas."
    )

    add_callout(doc, "NOTA IMPORTANTE",
        "El servidor de acción /joint_trajectory_controller/follow_joint_trajectory acepta metas de cualquier nodo en el dominio 0. "
        "Si dos grupos envían metas simultáneas, la segunda cancela de inmediato a la primera. Por tanto, el turno es un protocolo estricto entre personas."
    )

    # 3. OBJETIVOS
    add_section_heading(doc, "3. OBJETIVOS")
    add_subheading(doc, "3.1. Objetivo General")
    add_body(doc,
        "Operar el robot manipulador Kinova Gen3 real desde múltiples estaciones en red bajo la convención de estación anfitriona y dominio compartido (ROS_DOMAIN_ID=0), "
        "auditando la salud del enlace DDS mediante RQT y ejecutando el protocolo colaborativo de turnos de trayectorias articulares con validación obligatoria en modo seco."
    )
    add_subheading(doc, "3.2. Objetivos Específicos")
    add_list_item(doc, "Configurar el entorno de red y dominio común (ROS_DOMAIN_ID=0, CycloneDDS y conectividad IP con el Kinova real 192.168.1.10) en todos los equipos.", 1)
    add_list_item(doc, "Poner en marcha la estación anfitriona con el driver del robot real, verificando la sesión TCP Kortex, el ciclo del controlador a 100 Hz y un único publicador de /joint_states.", 2)
    add_list_item(doc, "Desplegar estaciones monitoras con identidad propia en el grafo, analizando la topología y logs mediante RQT (rqt_graph y rqt_console).", 3)
    add_list_item(doc, "Visualizar el robot real en RViz desde estaciones remotas sin ejecutar drivers locales, comprendiendo la durabilidad TRANSIENT_LOCAL en /robot_description.", 4)
    add_list_item(doc, "Ejecutar el protocolo de turnos aplicando validación previa en Modo Seco (dry_run:=true), autorización verbal y supervisión con parada de emergencia física.", 5)
    add_list_item(doc, "Cerrar ordenadamente la sesión Kortex y observar desde las estaciones monitoras la transición de pérdida de enlace en tiempo real.", 6)

    # 4. DESCRIPCIÓN DE LA PRÁCTICA
    add_section_heading(doc, "4. DESCRIPCIÓN DE LA PRÁCTICA")
    add_subheading(doc, "4.1. Fases de la Práctica")
    add_body(doc,
        "La práctica se desarrolla secuencialmente en cinco fases:\n"
        "• Fase 0: Configuración de red LAN (192.168.1.0/24), ping al Kinova (192.168.1.10) y exportación de ROS_DOMAIN_ID=0 en todos los equipos.\n"
        "• Fase 1: Puesta en marcha de la estación anfitriona por cable Ethernet con sesión Kortex y grabación de rosbag MCAP.\n"
        "• Fase 2: Despliegue de estaciones monitoras por Wi-Fi (kinova_monitor_eqNN), visualización en RViz e introspección gráfica con rqt_graph y rqt_console.\n"
        "• Fase 3: Envío de trayectorias por turnos en joint_1 (±0.20 a ±0.30 rad), validación en modo seco (código 0) y envío supervisado al robot real.\n"
        "• Fase 4: Cierre ordenado de la sesión Kortex y registro de la transición de desconexión (OK -> ERROR) en las monitoras."
    )

    add_subheading(doc, "4.2. Resultados de Aprendizaje Evaluables (RAE) y Ponderación — Corte 2 (L_C2: 42%)")
    add_callout(
        doc,
        "ASIGNACIÓN ACADÉMICA OFICIAL (CORTE 2)",
        "Este laboratorio corresponde a la evaluación experimental del Segundo Corte (Corte 2). Alimenta el componente agregado L_C2 (Laboratorios y evidencias experimentales, 42% del Corte 2), promediándose con el Laboratorio 02: L_2 = promedio(N_L02, N_L03), con escala 0–500 puntos (N_L03 = Puntos / 100 en escala 0.0–5.0). Entrega grupal única en documento C2_L03_G<grupo>_<cod1>_<cod2>_v1.docx."
    )
    add_grid_table(doc,
        ["Criterio", "RAE / Indicador Oficial del Syllabus", "SO", "Ponderación"],
        [
            ["C1. Red DDS distribuida y conectividad (ROS_DOMAIN_ID=0)", "2.1. Diseña soluciones de software integrando contratos QoS y redes DDS robustas.\n2.2. Restricciones de red y latencia en hardware heterogéneo.", "SO2", "25%"],
            ["C2. Estación anfitriona, sesión Kortex y telemetría", "2.1. Control y monitoreo de robots en tiempo real.\n6.4. Diagnóstico experimental de capas de comunicación y controladores.", "SO2 / SO6", "25%"],
            ["C3. Monitoreo remoto e introspección gráfica (RQT / RViz)", "6.4. Interpreta fallas aplicando protocolos por capas.\n3.1. Inspección de grafos, tópicos y contratos TF.", "SO6 / SO3", "20%"],
            ["C4. Protocolo de turnos, modo seco y seguridad física", "4.1. Identifica riesgos de seguridad física, paradas de emergencia y celdas robóticas.\n6.4. Validación experimental en modo seco.", "SO4 / SO6", "20%"],
            ["C5. Trazabilidad en rosbag, trabajo en equipo y cierre", "3.1 - 3.3. Documentación técnica reproducible y registro de telemetría.\n5.1. Define roles técnicos y coordina la ejecución en equipo.", "SO3 / SO5", "10%"],
            ["TOTAL", "", "", "100%"]
        ],
        widths=[5.0, 8.0, 2.0, 2.0],
        model_table=model_table
    )

    add_subheading(doc, "4.3. Rúbrica Analítica por Niveles de Desempeño (SGDE)")
    rubric_rows = [
        ["C1 (25%) Red DDS y Dominio 0", "Configura y automatiza variables en todas las terminales; justifica mitigación de fragmentación UDP en Wi-Fi con CycloneDDS y explica la segmentación lógica por dominios.", "Configura correctamente ROS_DOMAIN_ID=0, CycloneDDS, perfil XML, reinicia daemon y demuestra conectividad fluida mediante ping a 192.168.1.10.", "Configura dominio 0 y middleware indicado; comprueba conectividad con robot real y estado del daemon sin errores.", "Presenta inconsistencias de dominio en alguna terminal, olvida reiniciar daemon o no comprueba conectividad IP previa.", "No logra conectividad con la red 192.168.1.0/24 o utiliza un dominio diferente quedando aislado."],
        ["C2 (25%) Anfitriona y Kortex", "Demuestra unicidad del driver mediante ss -tanp, justifica conexión Ethernet determinista frente a Wi-Fi, explica qué detecta y qué NO detecta la comprobación del launch, y audita estabilidad de /joint_states a 100 Hz con un solo publicador y controladores activos.", "Despliega anfitriona con hardware real (start_driver:=true), verifica sesión TCP Kortex en puerto 10000 y comprueba identidad anunciada como anfitriona.", "Ejecuta anfitriona con hardware real; verifica controladores activos y que /joint_states publique a frecuencia nominal.", "Lanza driver sin verificar si el robot estaba ocupado, desactiva la comprobación del launch o continúa pese al WARNING.", "No logra establecer sesión Kortex o causa bloqueos por lanzar drivers duplicados."],
        ["C3 (20%) Monitores, RQT y RViz", "Justifica durabilidad TRANSIENT_LOCAL en /robot_description explicando por qué RViz visualiza la pose sin relanzar el modelo; analiza en rqt_graph y filtra logs en rqt_console diagnosticando la red.", "Lanza monitor con nombre único (kinova_monitor_eqNN), visualiza robot en RViz sin driver local, inspecciona topología en rqt_graph y filtra logs en rqt_console.", "Despliega monitor con identidad propia, abre RViz observando robot real e inspecciona nodos y logs en RQT.", "Ejecuta monitor sin renombrar causando nodos duplicados, o no logra visualizar el modelo en RViz.", "No despliega el nodo monitor o no realiza la introspección con herramientas gráficas."],
        ["C4 (20%) Turnos y Modo Seco", "Explica en profundidad el protocolo de turnos ante preemptibilidad de acciones en ROS 2; valida en Modo Seco interpretando reporte de deltas, custodia parada de emergencia y ejecuta movimiento exacto.", "Construye meta absoluta en joint_1, ejecuta modo seco con código 0, solicita autorización y realiza envío físico registrando pose final.", "Cumple protocolo de turnos: modo seco previo aprobado, confirmación interactiva y verificación de pose final.", "Intenta enviar trayectorias sin modo seco previo, o redondea erróneamente articulaciones.", "Envía trayectorias fuera de turno, viola límites de seguridad o normas del laboratorio."],
        ["C5 (10%) Rosbag y Trabajo Equipo", "Registra sesión en rosbag MCAP; analiza pérdida de enlace sincronizada al cerrar driver; completa tablas con rigor y Anexo A demuestra contribución sobresaliente de cada integrante.", "Graba evidencia en rosbag, observa transición OK -> ERROR al cerrar driver, completa tablas y responde preguntas de análisis con solvencia.", "Cierra limpiamente la sesión, verifica liberación del puerto Kortex, llena tablas y demuestra trabajo en equipo.", "Cierra driver de forma abrupta (kill -9) dejando sesión bloqueada, o anexo individual es insuficiente.", "No entrega documento con tablas llenas o no presenta evidencia de comprobación individual."]
    ]
    add_grid_table(doc,
        ["Criterio", "N5 (475–500)", "N4 (400–474)", "N3 (300–399) [Umbral]", "N2 (150–299)", "N1 (0–149)"],
        rubric_rows,
        widths=[2.8, 2.9, 2.9, 2.9, 2.8, 2.7],
        model_table=model_table
    )

    # 5. MATERIALES Y EQUIPOS
    add_section_heading(doc, "5. MATERIALES Y EQUIPOS")
    add_subheading(doc, "5.1. Equipos del Laboratorio")
    add_grid_table(doc,
        ["Descripción", "Cantidad", "Unidad de Medida"],
        [
            ["Brazo manipulador Kinova Gen3 (6-DOF) con pinza Robotiq 2F-85 (IP real: 192.168.1.10, puerto Kortex 10000)", "1", "Unidad"],
            ["Router TP-Link AX12 (SSID ros2, subred 192.168.1.0/24, Gateway 192.168.1.1)", "1", "Unidad"],
            ["Estación anfitriona con Ubuntu 24.04, ROS 2 Jazzy y cable Ethernet al router", "1", "Unidad"],
            ["Pulsador de parada de emergencia física, accesible desde la estación anfitriona", "1", "Unidad"]
        ],
        widths=[11.0, 3.0, 3.0],
        model_table=model_table
    )
    add_subheading(doc, "5.2. Equipos del Estudiante (por grupo)")
    add_grid_table(doc,
        ["Descripción", "Cantidad", "Unidad de Medida"],
        [
            ["Portátil con Ubuntu 24.04 LTS y ROS 2 Jazzy conectado por WiFi al SSID ros2 (o por cable)", "1", "Unidad"],
            ["Workspace ~/ros2_ws con burger_delivery y ros2_kortex compilados", "1", "Workspace"],
            ["Formato de entrega del laboratorio diligenciable (DOCX)", "1", "Documento"]
        ],
        widths=[11.0, 3.0, 3.0],
        model_table=model_table
    )

    # 6. SEGURIDAD
    add_section_heading(doc, "6. SEGURIDAD EN EL LABORATORIO")
    add_callout(doc, "ADVERTENCIA DE SEGURIDAD FÍSICA Y OPERATIVA",
        "1. Un solo driver en hardware real: Ninguna estación distinta de la anfitriona autorizada puede ejecutar start_driver:=true con el robot real, tampoco desde otro ROS_DOMAIN_ID.\n"
        "2. Custodia de Parada de Emergencia: Un integrante del grupo anfitrión debe permanecer junto al pulsador físico durante todo turno de envío.\n"
        "3. Área de Barrido: Mantener un radio libre de 1.2 m alrededor del robot. Nadie debe ingresar mientras haya un turno activo.\n"
        "4. Movimiento seguro y acotado: En este ejercicio sólo se mueve joint_1 (giro de base), típicamente ±0.20 a ±0.30 rad (11.5° a 17.2°) por turno en 5 s, unos 4 °/s. El cliente rechaza estrictamente cualquier articulación que supere max_joint_delta_rad = 0.35 rad (20.1°). Como joint_1 mueve el brazo completo, el radio libre de 1.2 m debe verificarse antes de CADA envío.\n"
        "5. Modo seco obligatorio: Ningún envío real puede autorizarse sin una ejecución previa en modo seco con código de salida 0.\n"
        "6. No limpiar fallas a ciegas: Ante cualquier anomalía, detener la sesión e informar de inmediato al docente.\n"
        "7. Síntoma de un segundo driver: Si el brazo se mueve a tirones o aparece WRONG_SERVOING_MODE, parada de emergencia, cerrar el driver intruso y REINICIAR también el driver de la anfitriona, que no se recupera solo."
    )

    # 7. PROCEDIMIENTO EXPERIMENTAL
    add_section_heading(doc, "7. PROCEDIMIENTO EXPERIMENTAL")

    add_subheading(doc, "Fase 0: Configuración de Red y ROS_DOMAIN_ID=0 (todos los grupos)")
    add_body(doc,
        "Todos los equipos del laboratorio deben compartir el dominio ROS_DOMAIN_ID=0. "
        "Topología: Robot Kinova en IP 192.168.1.10; Router TP-Link en 192.168.1.1 (SSID ros2); Anfitriona por Ethernet; Monitores por Wi-Fi."
    )
    add_body(doc, "Paso 0.1: Verificar IP asignada en la tarjeta de red (debe estar en 192.168.1.xx):")
    add_code_box(doc, ["ip -brief addr"])
    add_body(doc, "Paso 0.2: Probar conectividad con el robot Kinova real:")
    add_code_box(doc, ["ping -c 4 192.168.1.10"])
    add_body(doc, "Paso 0.3: Configurar el entorno en todas las terminales de trabajo:")
    add_code_box(doc, [
        "source /opt/ros/jazzy/setup.bash",
        "source ~/ros2_ws/install/setup.bash",
        "export ROS_DOMAIN_ID=0",
        "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        "export CYCLONEDDS_URI=\"file://$HOME/ros2_ws/src/burger_delivery/network_setup/cyclonedds.xml\"",
        "export CFG=$(ros2 pkg prefix burger_kinova_reference)/share/burger_kinova_reference/config/kinova_connection.yaml",
        "export EQ=eq03   # <-- sustituya 03 por SU numero de grupo, siempre con dos digitos",
        "[[ \"$EQ\" =~ ^eq[0-9]{2}$ ]] && echo \"EQ valido: $EQ\" || echo \"EQ MAL ESCRITO\""
    ])
    add_body(doc,
        "Convención eqNN: NN es el número de grupo asignado por el docente, siempre con dos dígitos (eq03, no eq3), "
        "en minúsculas y sin guiones, porque los nombres de nodo de ROS 2 sólo admiten letras, dígitos y guion bajo. "
        "Todos los integrantes de un mismo grupo usan el mismo sufijo. Como todas las estaciones comparten el dominio 0, "
        "dos nodos con el mismo nombre se mezclan en rqt y en el bag y deja de saberse qué grupo envió cada meta. "
        "El grupo anfitrión también tiene su eqNN y lo usa en el nombre del bag y de sus clientes, pero su monitor se llama "
        "kinova_monitor sin sufijo: eso es lo que identifica a la anfitriona. "
        "Todos los comandos de las fases siguientes usan ${EQ}: con esta variable exportada se copian y pegan sin editar. "
        "Para no repetirlo en cada terminal, añada la línea export EQ=eqNN al final de ~/.bashrc."
    )
    add_body(doc, "Paso 0.4: Reiniciar limpiamente el daemon de ROS 2 y comprobar variables:")
    add_code_box(doc, [
        "timeout 5s ros2 daemon stop",
        "ros2 daemon start",
        "ros2 daemon status",
        "echo \"ROS_DOMAIN_ID: $ROS_DOMAIN_ID | RMW: $RMW_IMPLEMENTATION\""
    ])
    add_body(doc, "Paso 0.5: Comprobar ejecutables compilados:")
    add_code_box(doc, ["ros2 pkg executables burger_kinova_reference"])

    add_subheading(doc, "Fase 1: Puesta en Marcha de la Estación Anfitriona (grupo anfitrión)")
    add_body(doc, "Paso 1.1: Comprobar que nadie más tiene sesión con el robot:")
    add_code_box(doc, [
        "ip -brief addr",
        "ping -c 4 192.168.1.10",
        "ss -tanp | grep 192.168.1.10",
        "timeout 15s ros2 node list | grep -E \"controller_manager|kinova_vision\""
    ])
    add_body(doc, "Paso 1.2: Terminal A1 — Grabación de evidencia en rosbag MCAP:")
    add_code_box(doc, ["ros2 bag record -s mcap -o sesion_turnos_eqNN --topics /joint_states /burger/kinova/diagnostics /rosout"])
    add_body(doc, "Paso 1.3: Terminal A2 — Lanzar driver con hardware real sin movimiento:")
    add_code_box(doc, [
        "ros2 launch burger_kinova_reference kinova_connection.launch.py \\",
        "  start_driver:=true robot_ip:=192.168.1.10 use_fake_hardware:=false \\",
        "  enable_motion:=false launch_rviz:=true"
    ])
    add_body(doc, "Paso 1.4: Terminal A3 — Verificar sesión TCP Kortex y telemetría a 100 Hz:")
    add_code_box(doc, [
        "ss -tanp | grep 192.168.1.10",
        "ros2 topic hz /joint_states",
        "ros2 topic echo /burger/kinova/diagnostics --once | grep -A8 \"identidad de la estación\""
    ])

    add_subheading(doc, "Fase 2: Estaciones Monitoras, RQT y RViz (grupos monitores)")
    add_body(doc, "Paso 2.1: Lanzar el monitor del grupo con nombre único (NN = número de grupo):")
    add_code_box(doc, [
        "ros2 run burger_kinova_reference kinova_monitor --ros-args --params-file $CFG \\",
        "  -r __node:=kinova_monitor_eqNN \\",
        "  -p start_driver:=false -p use_fake_hardware:=false -p robot_ip:=192.168.1.10"
    ])
    add_body(doc, "Paso 2.2: Auditar el estado del robot y roles anunciados:")
    add_code_box(doc, [
        "ros2 node list | grep kinova_monitor",
        "ros2 topic hz /joint_states",
        "ros2 topic echo /burger/kinova/diagnostics | grep -A8 \"identidad de la estación\""
    ])
    add_body(doc, "Paso 2.3: Visualizar el robot en RViz sin lanzar driver:")
    add_code_box(doc, ["rviz2 -d $(ros2 pkg prefix kortex_description)/share/kortex_description/rviz/view_robot.rviz"])
    add_body(doc, "Paso 2.4: Inspección gráfica con RQT — Grafo distribuido (rqt_graph) y Logs centralizados (rqt_console):")
    add_code_box(doc, [
        "# Terminal 1: Ver el grafo computacional distribuido",
        "rqt_graph",
        "# Terminal 2: Ver logs centralizados de /rosout filtrando por nodo",
        "ros2 run rqt_console rqt_console"
    ])

    add_subheading(doc, "Fase 3: Envío de Trayectorias por Turnos y Prueba Final Integradora")
    add_body(doc,
        "Análisis de ángulo en joint_1: joint_1 gira toda la estructura del brazo alrededor del eje vertical de la base. "
        "El movimiento es horizontal: no cambia la altura de la pinza, no flexiona el codo y no altera la carga por gravedad sobre los actuadores. "
        "El brazo de palanca es lo que hace visible el movimiento: con la pinza a ~0.6 m del eje de la base, un delta de 0.25 rad (14.3°) "
        "desplaza la pinza un arco de ~150 mm, visible desde cualquier mesa del laboratorio. "
        "Rango de trabajo: ±0.20 a ±0.30 rad por turno en 5 s. El límite estricto de seguridad es max_joint_delta_rad = 0.35 rad (20.1°), "
        "que en 5 s equivale a 0.07 rad/s (4 °/s), muy por debajo de la velocidad nominal del Gen3. "
        "Hasta el 16/09/2026 el turno movía joint_6 (giro de pinza) acotado a 0.10 rad: ~12 mm en las puntas, indistinguible del ruido desde las mesas. "
        "ATENCIÓN: joint_1 barre volumen y joint_6 no. Al girar la base, todo el brazo se desplaza lateralmente."
    )
    add_body(doc,
        "¿Qué es el Modo Seco (dry_run:=true)?\n"
        "Consiste en ejecutar toda la validación matemática sobre telemetría viva (/joint_states), "
        "comprobando límites articulares, deltas de desplazamiento y vigencia temporal, pero INHIBIENDO el contacto con el servidor de acción. "
        "El robot permanece inmóvil y el cliente retorna código 0 en Linux ($? = 0). Es obligatorio antes de autorizar el envío físico."
    )
    add_callout(doc, "PREVENCIÓN DE BLOQUEO DE SEGURIDAD",
        "Copiar valores numéricos de un ejemplo sin leer el robot real provocará que la meta sea BLOQUEADA inmediatamente "
        "por superar max_joint_delta_rad (0.35 rad). Es estrictamente obligatorio descubrir la pose actual viva antes de formular la meta."
    )
    add_body(doc, "Paso 3.1: Descubrir la pose articular actual real del robot en la mesa:")
    add_code_box(doc, [
        "# Ver las 6 posiciones articulares actuales en radianes [j1, j2, j3, j4, j5, j6]",
        "ros2 topic echo /joint_states --once --field position"
    ])
    add_body(doc,
        "Construcción de la meta: Conserve los 5 valores restantes (joint_2 a joint_6) idénticos a los descubiertos en el robot, "
        "con todos sus decimales, y sume o reste entre 0.20 y 0.30 rad únicamente a joint_1. "
        "Antes de enviar, anticipe hacia qué lado va a girar el brazo y confirme que ese lado esté despejado."
    )
    add_body(doc,
        "Ejemplo completo. Si la lectura de SU robot fue [-0.1944, -0.5010, -1.9547, -0.0058, -0.6803, -1.5982] "
        "y se elige un delta de +0.25 rad sobre joint_1 (-0.1944 + 0.2500 = 0.0556), el vector meta es "
        "[0.0556, -0.5010, -1.9547, -0.0058, -0.6803, -1.5982]: sólo cambia el primer número; los otros cinco se copian tal cual."
    )
    add_callout(doc, "j1_meta, j2_actual... NO SON VARIABLES: SON CASILLAS QUE USTED DEBE RELLENAR",
        "En los comandos siguientes esos nombres aparecen sólo para indicar qué va en cada posición del vector. "
        "La terminal no los conoce y no los sustituye por nada: si copia el comando tal cual, envía texto donde el cliente espera seis números. "
        "Antes de pulsar Enter, dentro de los corchetes no puede quedar ni una sola letra. "
        "MAL: safe_joint_positions_rad:=[j1_meta, j2_actual, j3_actual, j4_actual, j5_actual, j6_actual]   "
        "BIEN: safe_joint_positions_rad:=[0.0556, -0.5010, -1.9547, -0.0058, -0.6803, -1.5982]   "
        "Los seis números son los de SU robot en ESE instante: los del ejemplo provocarán el bloqueo por max_joint_delta_rad. "
        "En cambio ${EQ} sí es una variable de entorno y se sustituye sola, siempre que la haya exportado en la Fase 0."
    )
    add_body(doc, "Paso 3.2: Ejecutar Modo Seco en safe_trajectory_client y verificar código de salida 0. Reemplace las seis casillas por los números de su lectura:")
    add_code_box(doc, [
        "ros2 run burger_kinova_reference safe_trajectory_client --ros-args --params-file $CFG \\",
        "  -r __node:=safe_trajectory_client_eqNN \\",
        "  -p use_fake_hardware:=false -p enable_motion:=true -p dry_run:=true \\",
        "  -p \"safe_joint_positions_rad:=[j1_meta, j2_actual, j3_actual, j4_actual, j5_actual, j6_actual]\"",
        "echo $?"
    ])
    add_body(doc, "Con los valores del ejemplo anterior el comando quedaría así (los seis números serán OTROS en su mesa):")
    add_code_box(doc, [
        "ros2 run burger_kinova_reference safe_trajectory_client --ros-args --params-file $CFG \\",
        "  -r __node:=safe_trajectory_client_eqNN \\",
        "  -p use_fake_hardware:=false -p enable_motion:=true -p dry_run:=true \\",
        "  -p \"safe_joint_positions_rad:=[0.0556, -0.5010, -1.9547, -0.0058, -0.6803, -1.5982]\"",
        "echo $?"
    ])
    add_body(doc, "Paso 3.3: Tras autorización verbal, enviar al Hardware Real confirmando con 'si':")
    add_code_box(doc, [
        "ros2 run burger_kinova_reference safe_trajectory_client --ros-args --params-file $CFG \\",
        "  -r __node:=safe_trajectory_client_eqNN \\",
        "  -p use_fake_hardware:=false -p enable_motion:=true -p dry_run:=false \\",
        "  -p \"safe_joint_positions_rad:=[j1_meta, j2_actual, j3_actual, j4_actual, j5_actual, j6_actual]\""
    ])
    add_body(doc, "Paso 3.4: Verificar pose final y liberar el turno:")
    add_code_box(doc, ["ros2 topic echo /joint_states --once"])

    add_body(doc,
        "Paso 3.5: PRUEBA FINAL INTEGRADORA — Secuencia Autónoma con Autodescubrimiento de Pose (safe_sequence_client)\n"
        "A diferencia del cliente individual (que requiere formular coordenadas absolutas a mano), safe_sequence_client "
        "implementa autodescubrimiento dinámico de la pose de origen: al arrancar, lee /joint_states, fija el punto actual "
        "como origen y ejecuta una coreografía continua de 25 tramos relativos regresando al punto de inicio."
    )
    add_callout(doc, "ADVERTENCIA",
        "La secuencia autónoma desplaza hombro (joint_2), base (joint_1) y muñeca (joint_6) hasta ±31°. "
        "Exige despejar un radio de 1.2 m alrededor del robot y un operador con la mano en la parada de emergencia."
    )
    add_body(doc, "Ensayo en Modo Seco de la secuencia:")
    add_code_box(doc, [
        "ros2 run burger_kinova_reference safe_sequence_client --ros-args --params-file $CFG \\",
        "  -r __node:=safe_sequence_client_eqNN -p dry_run:=true"
    ])
    add_body(doc, "Ejecución real de la Prueba Final en hardware:")
    add_code_box(doc, [
        "ros2 run burger_kinova_reference safe_sequence_client --ros-args --params-file $CFG \\",
        "  -r __node:=safe_sequence_client_eqNN -p dry_run:=false -p enable_motion:=true"
    ])

    add_subheading(doc, "Fase 4: Cierre Ordenado y Observación de Pérdida de Enlace")
    add_body(doc, "Paso 4.1: En anfitriona, detener driver con Ctrl+C y verificar liberación de socket Kortex:")
    add_code_box(doc, [
        "ss -tanp | grep 192.168.1.10",
        "ros2 bag info sesion_turnos_eqNN"
    ])
    add_body(doc, "Paso 4.2: En monitoras, registrar tiempo hasta transición OK -> ERROR en la consola:")
    add_code_box(doc, ["[TRANSICIÓN] OK -> ERROR | telemetría vencida: ... s sin mensaje válido (límite 1.00 s)"])

    # 8. REGISTRO
    add_section_heading(doc, "8. REGISTRO DEL LABORATORIO")
    add_body(doc, "Los grupos deben registrar experimentalmente las siguientes tablas en el documento único de entrega:")

    add_subheading(doc, "Tabla 1: Verificación de Entorno y Red (Fase 0, todos los grupos)")
    add_grid_table(doc,
        ["Verificación", "Comando Ejecutado", "Resultado Esperado", "Resultado Obtenido"],
        [
            ["IP propia en subred 192.168.1.0/24", "ip -brief addr", "IP asignada en 192.168.1.xx", ""],
            ["Ping al robot Kinova (192.168.1.10)", "ping -c 4 192.168.1.10", "0% packet loss, RTT < 5 ms", ""],
            ["Dominio común ROS 2", "echo $ROS_DOMAIN_ID", "0", ""],
            ["Middleware DDS optimizado", "echo $RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp", ""],
            ["Sufijo de grupo configurado", "echo $EQ", "eqNN con el numero del grupo (p. ej. eq03)", ""],
            ["Estado del Daemon de ROS 2", "ros2 daemon status", "The daemon is running", ""],
            ["Ejecutables de referencia listos", "ros2 pkg executables burger_kinova_reference", "3 ejecutables listados", ""]
        ],
        widths=[5.0, 5.5, 4.0, 2.5],
        model_table=model_table
    )

    add_subheading(doc, "Tabla 2: Verificación de la Estación Anfitriona (Fase 1)")
    add_grid_table(doc,
        ["Verificación", "Comando", "Resultado"],
        [
            ["Nadie tenía el robot antes", "ss -tanp, ros2 node list", ""],
            ["Sesión Kortex establecida", "ss -tanp | grep 192.168.1.10", ""],
            ["Identidad anunciada", "rol_estacion / rol_verificado", ""],
            ["Frecuencia /joint_states (cable)", "ros2 topic hz", ""],
            ["Controladores activos", "ros2 control list_controllers", ""]
        ],
        widths=[6.0, 6.0, 5.0],
        model_table=model_table
    )

    add_subheading(doc, "Tabla 3: Estaciones Monitoras (Fases 2 y 4, una fila por grupo)")
    add_grid_table(doc,
        ["Grupo", "Nodo", "IP", "rol_estacion", "Frecuencia /joint_states", "Estado inicial", "RViz / rqt_graph OK", "Logs rqt_console", "Tiempo OK->ERROR"],
        [
            ["G01", "kinova_monitor_eq01", "", "", "", "", "", "", ""],
            ["G02", "kinova_monitor_eq02", "", "", "", "", "", "", ""],
            ["G03", "kinova_monitor_eq03", "", "", "", "", "", "", ""]
        ],
        widths=[1.5, 3.2, 2.0, 2.0, 2.2, 1.8, 1.8, 1.5, 1.0],
        model_table=model_table
    )

    add_subheading(doc, "Tabla 4: Registro de Turnos y Prueba Final (Fase 3)")
    add_grid_table(doc,
        ["Turno / Prueba", "Grupo", "Inicio", "Pose / joint_1 inicial", "Meta solicitada", "Modo seco (código)", "Envío (código / error)", "Pose final (joint_1)", "Cierre"],
        [
            ["1", "", "", "", "(+0.20 a +0.30)", "", "", "", ""],
            ["2", "", "", "", "(-0.20 a -0.30)", "", "", "", ""],
            ["3", "", "", "", "(+0.20 a +0.30)", "", "", "", ""],
            ["4", "", "", "", "(-0.20 a -0.30)", "", "", "", ""],
            ["Prueba Final (Secuencia)", "", "", "Origen autodescubierto", "Coreografía 25 deltas", "Código 0", "SUCCESSFUL", "Retorno origen OK", ""]
        ],
        widths=[1.5, 1.3, 1.8, 2.2, 2.4, 2.2, 2.5, 2.0, 1.2],
        model_table=model_table
    )

    add_subheading(doc, "Tabla 5: Incidentes y Diagnóstico por Capas")
    add_grid_table(doc,
        ["Momento", "Síntoma observado", "Capa (Red/DDS/Driver/Lógica/Protocolo)", "Verificación realizada", "Acción Correctiva"],
        [
            ["", "", "", "", ""],
            ["", "", "", "", ""]
        ],
        widths=[2.0, 4.0, 4.0, 3.5, 3.5],
        model_table=model_table
    )

    # 9. PREGUNTAS
    add_section_heading(doc, "9. PREGUNTAS DE ANÁLISIS")
    add_body(doc,
        "1. Unicidad del driver: Con la Tabla 2 y la identidad anunciada, argumente cómo la estación anfitriona única y el dominio compartido permiten responder '¿quién tiene el robot?' desde cualquier estación sin preguntar verbalmente.\n"
        "2. Emisores simultáneos y unicidad de acción: En ROS 2 y ros2_control, el servidor de acción /joint_trajectory_controller/follow_joint_trajectory acepta metas de cualquier nodo en el dominio 0. Si dos estaciones envían una meta de trayectoria simultáneamente, ¿qué le ocurre a la primera meta y por qué? ¿Por qué el protocolo de turnos es indispensable cuando todos comparten el ROS_DOMAIN_ID=0?\n"
        "3. Enlace por cable y por Wi-Fi: Compare la frecuencia de /joint_states en la anfitriona (cable) y en las monitoras (Wi-Fi). ¿Por qué la monitora puede ir por Wi-Fi y la anfitriona obligatoriamente por cable?\n"
        "4. Trazabilidad: Con el bag sesion_turnos_eqNN y la Tabla 4, reconstruya la cronología de un turno: qué nodo (por su nombre _eqNN) envió, cuándo se aceptó la meta y cuándo terminó.\n"
        "5. Garantía por software: El monitor publica 'habilitación de movimiento', pero el cliente no lo consulta antes de enviar. Proponga un diseño en el que el turno quede garantizado por software (por ejemplo, un servicio de concesión de turno en la anfitriona). ¿Qué nuevas fallas introduciría?\n"
        "6. Pérdida de enlace durante el movimiento: Si durante un turno se cae el Wi-Fi de la estación que envió la meta, ¿se detiene el robot? Razone con la arquitectura: dónde vive el controlador y dónde vive el cliente de acción.\n"
        "7. Aislamiento vs. Colaboración en DDS: ¿Qué ocurriría si un grupo deja accidentalmente su ROS_DOMAIN_ID en 10? ¿Podría ver la telemetría del robot o participar en los turnos? ¿Por qué es fundamental acordar exactamente el mismo ROS_DOMAIN_ID=0?\n"
        "8. Posicionamiento Absoluto vs. Deltas Relativos y Autodescubrimiento: Compare la operación de safe_trajectory_client frente a safe_sequence_client. ¿Por qué en el cliente individual fue estrictamente necesario descubrir las posiciones absolutas reales de /joint_states antes de formular la meta para evitar el bloqueo por max_joint_delta_rad, mientras que el cliente de secuencia pudo ejecutarse desde cualquier pose sin transcribir coordenadas a mano? ¿Qué riesgos y ventajas de seguridad introduce cada enfoque en entornos industriales colaborativos?"
    )

    # 10. REFERENCIAS
    add_section_heading(doc, "10. REFERENCIAS")
    add_body(doc,
        "1. Kinova Robotics. (2024). Kinova Gen3 Ultra lightweight robot User Guide. Kinova Inc.\n"
        "2. ros2_control. (2024). joint_trajectory_controller — Documentation. https://control.ros.org/\n"
        "3. ROS 2 Documentation. (2024). Understanding actions. https://docs.ros.org/en/jazzy/\n"
        "4. Proyecto burger_delivery. burger_kinova_reference/README.md, VALIDACION_CORTE_1.md y TROUBLESHOOTING.md §2."
    )

    body.insert(body.index(sect_pr), approval_model)
    approval = doc.tables[-1]
    replace_text_preserving_cell(approval.cell(2, 0), "Ing. Henry Roncancio\nDocente Asignatura ROS", size=8, align=WD_ALIGN_PARAGRAPH.CENTER)
    replace_text_preserving_cell(approval.cell(2, 1), "Director de Programa\nIngeniería Mecatrónica", size=8, align=WD_ALIGN_PARAGRAPH.CENTER)
    replace_text_preserving_cell(approval.cell(2, 2), "Decano(a)\nFacultad de Ingeniería", size=8, align=WD_ALIGN_PARAGRAPH.CENTER)

    configure_footer(doc)
    doc.core_properties.title = "Guía de Laboratorio 03 – Operación Distribuida del Kinova Gen3"
    doc.core_properties.subject = "ROBOT OPERATING SYSTEM - ROS"
    doc.core_properties.author = "Universidad Militar Nueva Granada"
    doc.core_properties.comments = "Documento construido sobre el formato institucional GL-AA-F-1 con soporte de estación anfitriona, monitores por Wi-Fi, RQT, protocolo de turnos y validación en modo seco."
    TARGET.parent.mkdir(parents=True, exist_ok=True)
    doc.save(str(TARGET))
    print(f"Guía Word LAB-03 generada exitosamente en: {TARGET}")


if __name__ == "__main__":
    main()
