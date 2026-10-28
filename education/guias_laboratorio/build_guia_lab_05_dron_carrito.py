#!/usr/bin/env python3
"""
Constructor del Word de la Guía de Laboratorio 05 (TF del dron como telemetría del carrito con micro-ROS).

La fuente única es GUIA_LAB_05_TF_DRON_CARRITO_MICROROS.md. Reutiliza el conversor
Markdown -> formato institucional GL-AA-F-1 de la Guía IA 01
(build_guia_lab_ia_01_embodied.py), cambiando sólo los datos de esta guía, para que
todas las guías generadas desde Markdown se vean igual.

Ejecución:  python3 education/guias_laboratorio/build_guia_lab_05_dron_carrito.py
"""

from __future__ import annotations

import sys
from pathlib import Path

from docx import Document

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import build_guia_lab_ia_01_embodied as base  # noqa: E402

base.FUENTE = AQUI / "GUIA_LAB_05_TF_DRON_CARRITO_MICROROS.md"
base.GUIA = AQUI / "GUIA_LAB_05_TF_DRON_CARRITO_MICROROS.docx"
base.ASIGNATURA = "ROBOT OPERATING SYSTEM - ROS"
base.TITULO = ("Laboratorio 05. La transformación del dron como telemetría del carrito: TF2, "
               "micro-ROS y control de posición en lazo cerrado")
base.FECHA_EMISION = "2026/10/28"


def main():
    secs = base.secciones(base.FUENTE.read_text(encoding="utf-8"))
    base.construir_guia(secs, "Ing. Henry Roncancio\nDocente Asignatura ROS")
    doc = Document(base.GUIA)
    cp = doc.core_properties
    cp.title = "Guía de Laboratorio 05 – TF del dron como telemetría del carrito con micro-ROS"
    cp.comments = f"Generado desde {base.FUENTE.name} con {Path(__file__).name}."
    doc.save(base.GUIA)


if __name__ == "__main__":
    main()
