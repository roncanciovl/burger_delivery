# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Cliente mínimo de Gemini Robotics-ER para señalar objetos en una imagen.

La clave se lee de ``GEMINI_API_KEY`` (o ``GOOGLE_API_KEY``), como hace el SDK
``google-genai``; nunca de un parámetro ROS, para que no acabe en un bag ni en
``ros2 param dump``. El SDK se importa al crear el cliente: el resto del paquete funciona
y se prueba sin él.
"""

import os
import time
from typing import Dict, List, Tuple

from burger_perception.vlm_geometry import parse_points

DEFAULT_MODEL = 'gemini-robotics-er-1.6-preview'

PROMPT_TEMPLATE = (
    'Point to the {target}. {context}\n'
    'Answer ONLY with a JSON array like [{{"point": [y, x], "label": "<name>"}}], '
    'one entry per matching object, most relevant first. '
    'The points are in [y, x] format normalized to 0-1000. '
    'If there is no such object, answer [].'
)


def build_prompt(target: str, context: str = '') -> str:
    """Construir el prompt de señalamiento para ``target`` con restricciones opcionales."""
    return PROMPT_TEMPLATE.format(target=target, context=context.strip())


class GeminiPointer:
    """Envoltorio de ``google.genai.Client`` que devuelve puntos y latencia."""

    def __init__(self, model: str = DEFAULT_MODEL, temperature: float = 0.0,
                 thinking_budget: int = 0):
        """Crear el cliente; falla con un mensaje claro si falta el SDK o la clave."""
        if not (os.environ.get('GEMINI_API_KEY') or os.environ.get('GOOGLE_API_KEY')):
            raise RuntimeError('Falta GEMINI_API_KEY en el entorno del nodo '
                               '(export GEMINI_API_KEY=...; nunca como parámetro ROS)')
        try:
            from google import genai
            from google.genai import types
        except ImportError as error:
            raise RuntimeError('Falta el SDK: pip install google-genai') from error
        self._types = types
        self._client = genai.Client()
        self.model = model
        self._config = types.GenerateContentConfig(
            temperature=temperature,
            thinking_config=types.ThinkingConfig(thinking_budget=thinking_budget))

    def point(self, jpeg: bytes, prompt: str) -> Tuple[List[Dict], float, str]:
        """
        Pedir al modelo los puntos del objeto descrito en ``prompt``.

        :param jpeg: imagen codificada en JPEG.
        :returns: ``(puntos, latencia_ms, texto_crudo)``.
        """
        start = time.monotonic()
        response = self._client.models.generate_content(
            model=self.model,
            contents=[self._types.Part.from_bytes(data=jpeg, mime_type='image/jpeg'), prompt],
            config=self._config)
        latency_ms = (time.monotonic() - start) * 1000.0
        text = response.text or ''
        return parse_points(text), latency_ms, text
