#!/usr/bin/env python3
"""
Constructor para INSTRUMENTO_ABET_PROYECTO_FINAL_CORTE_3_PICK_AND_PLACE.docx

La fuente única es INSTRUMENTO_ABET_PROYECTO_FINAL_CORTE_3_PICK_AND_PLACE.md. Reutiliza el
conversor Markdown -> Word del instrumento del Laboratorio IA 01
(build_instrumento_lab_ia_01.py), con el estilo de los instrumentos de los laboratorios
de ROS, cambiando sólo los datos de este instrumento.

Ejecución:  python3 education/evidencias_abet/build_instrumento_proyecto_final_corte3.py
"""

from __future__ import annotations

import sys
from pathlib import Path

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import build_instrumento_lab_ia_01 as base  # noqa: E402

base.FUENTE = AQUI / "INSTRUMENTO_ABET_PROYECTO_FINAL_CORTE_3_PICK_AND_PLACE.md"
base.DESTINO = AQUI / "INSTRUMENTO_ABET_PROYECTO_FINAL_CORTE_3_PICK_AND_PLACE.docx"
base.PIE = "ROB-ROS · Instrumento Único de Evidencia y Evaluación ABET — Proyecto Final C3 · 2026-2"
base.TITULO_DOC = "Instrumento ABET – Proyecto Final Corte 3 – Pick and place con MoveIt 2, pinza y visión"
base.ASIGNATURA = "ROBOT OPERATING SYSTEM - ROS"
base.CONSTRUCTOR = Path(__file__).name

if __name__ == "__main__":
    base.construir()
