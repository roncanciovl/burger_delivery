# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""Pruebas de la lógica pura del nodo Gemini: respuesta, profundidad y desproyección."""

from burger_perception.gemini_client import build_prompt
from burger_perception.vlm_geometry import (
    deproject,
    depth_at,
    normalized_to_pixel,
    parse_points,
    ray_from_intrinsics,
    scale_pixel,
    scale_ray_to_depth,
)
import numpy as np
import pytest


def test_parse_respuesta_con_bloque_json():
    """El modelo suele envolver el JSON en un bloque de código."""
    texto = 'Aquí está:\n```json\n[{"point": [420, 610], "label": "burger box"}]\n```'
    puntos = parse_points(texto)
    assert puntos == [{'y': 420.0, 'x': 610.0, 'label': 'burger box'}]


def test_parse_descarta_puntos_invalidos():
    """Fuera de 0-1000, con una sola coordenada o sin 'point' no son puntos."""
    texto = ('[{"point": [1200, 10]}, {"point": [5]}, {"label": "x"}, '
             '{"point": [100, 200], "label": "ok"}]')
    assert [p['label'] for p in parse_points(texto)] == ['ok']


@pytest.mark.parametrize('texto', ['', 'no veo ninguna caja', '[]', '[{"point": ]'])
def test_parse_respuestas_vacias_o_rotas(texto):
    """Una respuesta sin arreglo válido no produce puntos ni excepción."""
    assert parse_points(texto) == []


def test_orden_y_x_de_gemini():
    """[y, x] = [250, 750] en 1920×1080 es u = 1440, v = 270."""
    assert normalized_to_pixel({'y': 250, 'x': 750}, 1920, 1080) == (1440.0, 270.0)


def test_escala_de_pixel_entre_resoluciones():
    """El centro del color 1920×1080 cae en el centro de la profundidad 480×270."""
    assert scale_pixel(960, 540, (1920, 1080), (480, 270)) == (240.0, 135.0)


def test_profundidad_por_mediana_ignora_ceros():
    """Los ceros del sensor (sin medida) no cuentan; 16UC1 está en milímetros."""
    depth = np.zeros((10, 10), dtype=np.uint16)
    depth[4:7, 4:7] = 800
    depth[5, 5] = 0
    assert depth_at(depth, 5, 5, window=3) == pytest.approx(0.8)


def test_profundidad_float_en_metros_y_sin_datos():
    """32FC1 ya viene en metros; una ventana sin datos da None."""
    depth = np.full((4, 4), np.nan, dtype=np.float32)
    assert depth_at(depth, 1, 1, window=3, encoding='32FC1') is None
    depth[1, 1] = 0.65
    assert depth_at(depth, 1, 1, window=3, encoding='32FC1') == pytest.approx(0.65)
    assert depth_at(depth, 50, 1) is None


def test_rayo_unitario_se_escala_por_z_y_no_por_la_norma():
    """Con el rayo unitario, ray * z acorta el punto; ray * z / ray_z es correcto."""
    fx = fy = 600.0
    cx, cy = 320.0, 240.0
    u, v, z = 620.0, 40.0, 0.9
    esperado = deproject(u, v, z, fx, fy, cx, cy)
    ray = ray_from_intrinsics(u, v, fx, fy, cx, cy)
    assert scale_ray_to_depth(ray, z) == pytest.approx(esperado)
    ingenuo = tuple(r * z for r in ray)
    assert ingenuo[2] < z - 0.05


def test_prompt_incluye_objetivo_y_contexto():
    """El prompt nombra el objeto, el contexto y el formato [y, x]."""
    prompt = build_prompt('burger box', 'On the table.')
    assert 'burger box' in prompt and 'On the table.' in prompt and '[y, x]' in prompt


def test_cliente_exige_la_clave_en_el_entorno(monkeypatch):
    """Sin GEMINI_API_KEY el cliente falla con un mensaje claro, antes de importar el SDK."""
    from burger_perception.gemini_client import GeminiPointer
    monkeypatch.delenv('GEMINI_API_KEY', raising=False)
    monkeypatch.delenv('GOOGLE_API_KEY', raising=False)
    with pytest.raises(RuntimeError, match='GEMINI_API_KEY'):
        GeminiPointer()
