#!/usr/bin/env python3
"""
Constructor para INSTRUMENTO_ABET_LAB_05_TF_DRON_CARRITO_MICROROS.docx

La fuente única es INSTRUMENTO_ABET_LAB_05_TF_DRON_CARRITO_MICROROS.md. Reutiliza el
conversor Markdown -> Word del instrumento del Laboratorio IA 01
(build_instrumento_lab_ia_01.py), con el estilo de los instrumentos de los laboratorios
de ROS, cambiando sólo los datos de este instrumento.

Ejecución:  python3 education/evidencias_abet/build_instrumento_lab_05.py
"""

from __future__ import annotations

import sys
from pathlib import Path

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import build_instrumento_lab_ia_01 as base  # noqa: E402

base.FUENTE = AQUI / "INSTRUMENTO_ABET_LAB_05_TF_DRON_CARRITO_MICROROS.md"
base.DESTINO = AQUI / "INSTRUMENTO_ABET_LAB_05_TF_DRON_CARRITO_MICROROS.docx"
base.PIE = "ROB-ROS · Instrumento Único de Evidencia y Evaluación ABET — Lab 05 · 2026-2"
base.TITULO_DOC = "Instrumento ABET – Laboratorio 05 – TF del dron como telemetría del carrito con micro-ROS"
base.ASIGNATURA = "ROBOT OPERATING SYSTEM - ROS"
base.CONSTRUCTOR = Path(__file__).name

if __name__ == "__main__":
    base.construir()
