#!/usr/bin/env python3
"""Prepara una carga verificable de ROS para Zubatronic/SGDE.

El modo de este programa es siempre VISTA_PREVIA. No abre el navegador, no
escribe en Firestore y no modifica Zubatronic. Usa solamente la biblioteca
estándar de Python para que pueda ejecutarse en el repositorio sin instalar
dependencias.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from zipfile import ZipFile


COURSE_NAME = "ELECTIVA ELECTRONICA ( ROBOT OPERATING SYSTEM"
SGDE_COURSE_KEY = "ELECTIVA ELECTRONICA (  ROBOT OPERATING SYSTEM)|A|2026-2"
SGDE_ASIGNATURA = SGDE_COURSE_KEY.split("|")[0]
TERM = "2026-2"
OFFICIAL_COURSE_CODE = "171552"
RUBRIC_NAMESPACE = "18326"

INSTRUMENT_SOURCES = (
    {
        "code": "T01",
        "name": "Talleres y tareas",
        "type": "Taller",
        "weight": 28,
        "description": "Taller 01 — CLI de ROS 2 e introspección del grafo.",
        "source": "evidencias_abet/PLANTILLA_EVIDENCIA_ABET_TALLER_01_ROS2_CLI.md",
    },
    {
        "code": "L01",
        "name": "Laboratorios y evidencias experimentales",
        "type": "Laboratorio",
        "weight": 42,
        "description": "Laboratorio 01 — red ROS 2 distribuida, DDS y talker/listener.",
        "source": "evidencias_abet/INSTRUMENTO_ABET_LAB_01_RED_ROS2_DISTRIBUIDA.md",
    },
    {
        "code": "E01",
        "name": "Entrega final y sustentación técnica",
        "type": "Parcial",
        "weight": 30,
        "description": "Proyecto del corte 1 — package de conexión segura con Kinova Gen3.",
        "source": "evidencias_abet/INSTRUMENTO_ABET_PROYECTO_CORTE_1_CONEXION_KINOVA.md",
    },
)

LEVEL_CODES = ("N5", "N4", "N3", "N2", "N1")
WORD_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
SHEET_NS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
REL_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
PKG_REL_NS = "http://schemas.openxmlformats.org/package/2006/relationships"


class PreviewError(RuntimeError):
    """Error que invalida la vista previa."""


def plain_markdown(value: str) -> str:
    value = value.strip()
    value = value.replace("**", "").replace("`", "")
    value = value.replace("&nbsp;", " ")
    return re.sub(r"\s+", " ", value).strip()


def table_cells(line: str) -> list[str]:
    return [plain_markdown(cell) for cell in line.strip().strip("|").split("|")]


def parse_instrument(source: Path, metadata: dict[str, Any]) -> dict[str, Any]:
    if not source.exists():
        raise PreviewError(f"No existe la fuente: {source}")

    lines = source.read_text(encoding="utf-8").splitlines()
    summaries: dict[str, dict[str, Any]] = {}

    for line in lines:
        if not line.lstrip().startswith("|"):
            continue
        cells = table_cells(line)
        if not cells or not re.fullmatch(r"C\d+\..+", cells[0]):
            continue
        match = re.match(r"(C\d+)\.\s*(.+)", cells[0])
        if not match:
            continue
        code, name = match.groups()
        summary: dict[str, Any] = {"code": code, "name": name}

        # Taller/Laboratorio: criterio | peso | SO | indicador | evidencia
        if len(cells) >= 4 and re.fullmatch(r"\d+(?:[.,]\d+)?%", cells[1]):
            summary["weight"] = float(cells[1].replace("%", "").replace(",", "."))
            so_match = re.search(r"SO\s*(\d+)", cells[2], re.IGNORECASE)
            indicator_match = re.match(r"(\d+\.\d+)\.?(?:\s+)(.+)", cells[3])
            if so_match:
                summary["student_outcome"] = f"SO{so_match.group(1)}"
            if indicator_match:
                summary["indicator"] = indicator_match.group(1)
                summary["rae_text"] = f"{indicator_match.group(1)} {indicator_match.group(2)}"

        # Proyecto: criterio | indicador | SO | peso | máximo
        elif len(cells) >= 4 and re.fullmatch(r"\d+(?:[.,]\d+)?%", cells[3]):
            indicator_match = re.search(r"\d+\.\d+", cells[1])
            so_match = re.search(r"SO\s*(\d+)", cells[2], re.IGNORECASE)
            summary["weight"] = float(cells[3].replace("%", "").replace(",", "."))
            if indicator_match:
                summary["indicator"] = indicator_match.group(0)
            if so_match:
                summary["student_outcome"] = f"SO{so_match.group(1)}"

        if "weight" in summary:
            # Los documentos repiten C1..C5 en una tabla final de consolidado,
            # pero esa tabla no vuelve a declarar RAE/SO. Conservar siempre la
            # fila informativa más completa.
            previous = summaries.get(code)
            if previous is None or (summary.get("indicator") and not previous.get("indicator")):
                summaries[code] = summary

    criteria: dict[str, dict[str, Any]] = {}
    current_code: str | None = None
    heading_pattern = re.compile(
        r"^###\s+(C\d+)\.\s+(.+?)\s+—\s+(?:peso\s+)?(\d+(?:[.,]\d+)?)%",
        re.IGNORECASE,
    )
    level_pattern = re.compile(r"^N([1-5])\s+—\s+.+$", re.IGNORECASE)

    for line in lines:
        heading = heading_pattern.match(line.strip())
        if heading:
            code, heading_name, heading_weight = heading.groups()
            summary = summaries.get(code, {})
            criteria[code] = {
                "code": code,
                "name": f"{code}. {summary.get('name', plain_markdown(heading_name))}",
                "weight": float(summary.get("weight", heading_weight.replace(",", "."))),
                "indicator": summary.get("indicator", ""),
                "student_outcome": summary.get("student_outcome", ""),
                "rae_text": summary.get("rae_text", ""),
                "levels": {},
            }
            current_code = code
            continue

        if current_code and line.lstrip().startswith("|"):
            cells = table_cells(line)
            if len(cells) >= 3:
                level = level_pattern.match(cells[1])
                if level:
                    criteria[current_code]["levels"][f"N{level.group(1)}"] = cells[2]

    ordered = [criteria[key] for key in sorted(criteria, key=lambda item: int(item[1:]))]
    for criterion in ordered:
        summary = summaries.get(criterion["code"], {})
        if not criterion["indicator"] and summary.get("indicator"):
            criterion["indicator"] = summary["indicator"]
        if not criterion["student_outcome"] and summary.get("student_outcome"):
            criterion["student_outcome"] = summary["student_outcome"]
        if not criterion["rae_text"] and criterion["indicator"]:
            criterion["rae_text"] = criterion["indicator"]

    result = dict(metadata)
    result["source"] = str(source)
    result["criteria"] = ordered
    return result


def docx_paragraphs(path: Path) -> list[str]:
    with ZipFile(path) as archive:
        root = ET.fromstring(archive.read("word/document.xml"))
    paragraphs: list[str] = []
    for paragraph in root.iter(f"{{{WORD_NS}}}p"):
        text = "".join(node.text or "" for node in paragraph.iter(f"{{{WORD_NS}}}t"))
        if text.strip():
            paragraphs.append(text.strip())
    return paragraphs


def extract_raes_from_syllabus(path: Path) -> list[dict[str, Any]]:
    pattern = re.compile(
        r"Indicador\s+(\d+)\.(\d+)\s+\(SO(\d+)\s*-[^)]+\):\s*(.+)",
        re.IGNORECASE,
    )
    raes: dict[str, dict[str, Any]] = {}
    for paragraph in docx_paragraphs(path):
        match = pattern.search(paragraph)
        if not match:
            continue
        major, minor, so, text = match.groups()
        indicator = f"{major}.{minor}"
        raes[indicator] = {
            "id": int(f"{RUBRIC_NAMESPACE}{major}{minor}"),
            "asignatura": SGDE_ASIGNATURA,
            "so": int(so),
            "rae": f"{indicator} {plain_markdown(text)}",
        }
    return [raes[key] for key in sorted(raes, key=lambda item: tuple(map(int, item.split("."))))]


def validate_preview(manifest: dict[str, Any], raes: list[dict[str, Any]]) -> list[str]:
    errors: list[str] = []
    instruments = manifest["instruments"]
    if sum(item["weight"] for item in instruments) != 100:
        errors.append("Los instrumentos no suman 100%.")
    if [item["weight"] for item in instruments] != [28, 42, 30]:
        errors.append("Los instrumentos no conservan la distribución 28/42/30.")

    known_raes = {item["rae"].split()[0]: item for item in raes}
    seen_names: set[str] = set()
    for instrument in instruments:
        criteria = instrument["criteria"]
        if not criteria:
            errors.append(f"{instrument['code']}: no tiene criterios.")
            continue
        if sum(item["weight"] for item in criteria) != 100:
            errors.append(f"{instrument['code']}: los criterios no suman 100%.")
        for criterion in criteria:
            key = f"{instrument['code']}:{criterion['name']}"
            if key in seen_names:
                errors.append(f"Criterio duplicado: {key}.")
            seen_names.add(key)
            missing_levels = [level for level in LEVEL_CODES if not criterion["levels"].get(level)]
            if missing_levels:
                errors.append(
                    f"{instrument['code']} {criterion['code']}: faltan {', '.join(missing_levels)}."
                )
            if criterion["indicator"] not in known_raes:
                errors.append(
                    f"{instrument['code']} {criterion['code']}: RAE {criterion['indicator']} no está en el syllabus."
                )
            else:
                official = known_raes[criterion["indicator"]]
                criterion["rae_id"] = official["id"]
                criterion["rae_text"] = official["rae"]
                if criterion["student_outcome"] != f"SO{official['so']}":
                    errors.append(
                        f"{instrument['code']} {criterion['code']}: SO no coincide con el syllabus."
                    )
    return errors


def write_rae_csv(path: Path, raes: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream, delimiter=";")
        writer.writerow(("ID", "ASIGNATURA", "SO", "RAE"))
        for item in raes:
            writer.writerow((item["id"], item["asignatura"], item["so"], item["rae"]))


def excel_column_index(reference: str) -> int:
    match = re.match(r"[A-Z]+", reference)
    if not match:
        raise PreviewError(f"Referencia de celda inválida: {reference}")
    number = 0
    for character in match.group(0):
        number = number * 26 + ord(character) - 64
    return number - 1


def read_first_xlsx_sheet(path: Path) -> list[list[str]]:
    namespace = {"m": SHEET_NS, "r": REL_NS, "pr": PKG_REL_NS}
    with ZipFile(path) as archive:
        shared: list[str] = []
        if "xl/sharedStrings.xml" in archive.namelist():
            root = ET.fromstring(archive.read("xl/sharedStrings.xml"))
            shared = [
                "".join(node.text or "" for node in item.iter(f"{{{SHEET_NS}}}t"))
                for item in root.findall("m:si", namespace)
            ]
        workbook = ET.fromstring(archive.read("xl/workbook.xml"))
        relations = ET.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
        targets = {
            item.attrib["Id"]: item.attrib["Target"]
            for item in relations.findall("pr:Relationship", namespace)
        }
        first_sheet = next(iter(workbook.find("m:sheets", namespace)))
        relation_id = first_sheet.attrib[f"{{{REL_NS}}}id"]
        sheet_path = "xl/" + targets[relation_id].lstrip("/")
        root = ET.fromstring(archive.read(sheet_path))

    rows: list[list[str]] = []
    for row in root.findall(".//m:row", namespace):
        values: dict[int, str] = {}
        for cell in row.findall("m:c", namespace):
            index = excel_column_index(cell.attrib["r"])
            value_element = cell.find("m:v", namespace)
            value = ""
            if cell.attrib.get("t") == "s" and value_element is not None and value_element.text:
                value = shared[int(value_element.text)]
            elif cell.attrib.get("t") == "inlineStr":
                value = "".join(
                    node.text or "" for node in cell.iter(f"{{{SHEET_NS}}}t")
                )
            elif value_element is not None:
                value = value_element.text or ""
            values[index] = value.strip()
        if values:
            width = max(values) + 1
            rows.append([values.get(index, "") for index in range(width)])
    return rows


def write_student_csv(source: Path, target: Path) -> dict[str, int]:
    rows = read_first_xlsx_sheet(source)
    if not rows:
        raise PreviewError("La lista de estudiantes está vacía.")
    headers = [value.upper() for value in rows[0]]
    required = ("CODIGO", "APELLIDOS", "NOMBRES", "CORREO")
    try:
        indexes = {header: headers.index(header) for header in required}
    except ValueError as error:
        raise PreviewError(f"Falta una columna obligatoria en la lista: {error}") from error

    students = []
    for row in rows[1:]:
        padded = row + [""] * (len(headers) - len(row))
        record = {header: padded[indexes[header]].strip() for header in required}
        if any(record.values()):
            students.append(record)

    codes = [item["CODIGO"] for item in students if item["CODIGO"]]
    emails = [item["CORREO"].lower() for item in students if item["CORREO"]]
    summary = {
        "students": len(students),
        "missing_code": sum(not item["CODIGO"] for item in students),
        "missing_name": sum(not (item["APELLIDOS"] and item["NOMBRES"]) for item in students),
        "missing_email": sum(not item["CORREO"] for item in students),
        "duplicate_codes": sum(count - 1 for count in Counter(codes).values() if count > 1),
        "duplicate_emails": sum(count - 1 for count in Counter(emails).values() if count > 1),
        "institutional_emails": sum(email.endswith("@unimilitar.edu.co") for email in emails),
    }
    if any(summary[key] for key in ("missing_code", "missing_name", "missing_email", "duplicate_codes", "duplicate_emails")):
        raise PreviewError(f"La lista estudiantil no pasa validación: {summary}")

    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=required, delimiter=";")
        writer.writeheader()
        writer.writerows(students)
    return summary


def build_report(manifest: dict[str, Any], raes: list[dict[str, Any]], student_summary: dict[str, int] | None) -> str:
    lines = [
        "# Vista previa de carga — ROS — Zubatronic/SGDE",
        "",
        f"- Modo: **{manifest['mode']}**",
        f"- Curso oficial: **{COURSE_NAME}**",
        f"- Curso objetivo SGDE: `{SGDE_COURSE_KEY}`",
        f"- Periodo: **{TERM}**",
        "- Estado actual observado: tres instrumentos existentes, sin criterios, sin estudiantes y sin vínculo Classroom.",
        "- Acción externa ejecutada: **ninguna**.",
        "",
        "## Instrumentos preparados",
        "",
        "| Código | Instrumento existente | Tipo | Peso | Criterios | Suma criterios |",
        "|---|---|---|---:|---:|---:|",
    ]
    for item in manifest["instruments"]:
        lines.append(
            f"| {item['code']} | {item['name']} | {item['type']} | {item['weight']}% | "
            f"{len(item['criteria'])} | {sum(c['weight'] for c in item['criteria']):g}% |"
        )
    lines.extend(
        [
            "",
            "## Contenido que se aplicaría",
            "",
            "| Instrumento | Criterio | Peso | RAE | SO | N1–N5 completos |",
            "|---|---|---:|---|---|:---:|",
        ]
    )
    for instrument in manifest["instruments"]:
        for criterion in instrument["criteria"]:
            complete = all(criterion["levels"].get(level) for level in LEVEL_CODES)
            lines.append(
                f"| {instrument['code']} | {criterion['name']} | {criterion['weight']:g}% | "
                f"{criterion['indicator']} | {criterion['student_outcome']} | {'Sí' if complete else 'No'} |"
            )
    lines.extend(
        [
            "",
            "## Dependencia administrativa",
            "",
            f"El curso no tiene RAEs disponibles en SGDE. Se preparó un CSV con {len(raes)} RAE del syllabus ",
            "para que un administrador lo importe. La columna `ASIGNATURA` usa exactamente el nombre interno del curso; ",
            "así funciona el fallback por nombre de la versión actual de SGDE.",
            "",
            f"El código oficial del syllabus es `{OFFICIAL_COURSE_CODE}`, mientras la rúbrica maestra usa el espacio de nombres `{RUBRIC_NAMESPACE}`. ",
            "No se fusionan: el primero identifica la asignatura y el segundo sólo genera IDs RAE estables y no colisionantes.",
        ]
    )
    if student_summary:
        lines.extend(
            [
                "",
                "## Lista privada preparada",
                "",
                f"- Estudiantes: {student_summary['students']}",
                f"- Correos institucionales: {student_summary['institutional_emails']}",
                "- Faltantes o duplicados: 0",
                "- El CSV contiene datos personales y permanece fuera del repositorio.",
            ]
        )
    lines.extend(
        [
            "",
            "## Orden recomendado de aplicación",
            "",
            "1. Un administrador importa el CSV de RAE.",
            "2. Se verifica que los 21 RAE aparezcan en el selector del curso.",
            "3. Se importan los 19 estudiantes desde el CSV privado.",
            "4. Se agregan los 15 criterios a los tres instrumentos existentes.",
            "5. Se relee el curso y se comprueba 28/42/30, 100% por instrumento, N1–N5 y RAE/SO.",
            "6. No se usa el botón Crear en Classroom; el curso de Classroom existente se mantiene por separado.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    education_root = script_dir.parents[1]
    parser = argparse.ArgumentParser(description="Prepara la vista previa de ROS para Zubatronic.")
    parser.add_argument("--output-dir", type=Path, default=script_dir / "preview")
    parser.add_argument("--student-xlsx", type=Path)
    parser.add_argument("--student-output", type=Path)
    args = parser.parse_args()

    if bool(args.student_xlsx) != bool(args.student_output):
        parser.error("--student-xlsx y --student-output deben usarse juntos.")

    instruments = []
    for metadata in INSTRUMENT_SOURCES:
        source = education_root / metadata["source"]
        instruments.append(parse_instrument(source, dict(metadata)))

    syllabus = education_root / "syllabus/Syllabus_Electiva_ROS.docx"
    raes = extract_raes_from_syllabus(syllabus)
    manifest = {
        "schema_version": "1.0",
        "mode": "VISTA_PREVIA",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "course": {
            "official_name": COURSE_NAME,
            "official_code": OFFICIAL_COURSE_CODE,
            "term": TERM,
            "sgde_course_key": SGDE_COURSE_KEY,
            "rubric_namespace": RUBRIC_NAMESPACE,
        },
        "policy": {
            "update_existing_instruments": True,
            "create_missing_instruments": False,
            "delete_extra_data": False,
            "publish_to_classroom": False,
            "require_rae_preload": True,
        },
        "instruments": instruments,
    }
    errors = validate_preview(manifest, raes)
    if len(raes) != 21:
        errors.append(f"Se esperaban 21 RAE del syllabus y se extrajeron {len(raes)}.")
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    student_summary = None
    if args.student_xlsx and args.student_output:
        student_summary = write_student_csv(args.student_xlsx, args.student_output)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output_dir / "ros_corte1_zubatronic.preview.json"
    rae_path = args.output_dir / "rae_ros_2026-2_admin.csv"
    report_path = args.output_dir / "RESUMEN_PREVIO_CARGA_ROS.md"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_rae_csv(rae_path, raes)
    report_path.write_text(build_report(manifest, raes, student_summary), encoding="utf-8")

    print(f"Vista previa válida: {len(instruments)} instrumentos, {sum(len(i['criteria']) for i in instruments)} criterios, {len(raes)} RAE.")
    print(f"Manifiesto: {manifest_path}")
    print(f"RAE para administrador: {rae_path}")
    print(f"Reporte: {report_path}")
    if args.student_output:
        print(f"Lista privada: {args.student_output} ({student_summary['students']} estudiantes)")
    print("No se realizaron escrituras en Zubatronic/SGDE.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
