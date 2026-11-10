"""
Autoevaluación de los TODO 2, 3 y 4 con una escena RGB-D sintética (no requiere
robot, cámara ni clave de API):

    cd education/guias_laboratorio/plantilla_lab_ia_embodied
    python3 -m pytest tests -q

La escena es una mesa plana a 0.80 m con una caja cuya cara superior está a 0.60 m.
Se conoce el punto 3D exacto de la caja, así que se puede medir el error del pipeline.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import embodied_comun as ec  # noqa: E402

ANCHO_C, ALTO_C = 1280, 720


@pytest.fixture
def cfg() -> ec.ConfigCamara:
    color = ec.Intrinsecos(900.0, 900.0, 640.0, 360.0, ANCHO_C, ALTO_C, "sintética")
    prof = ec.Intrinsecos(360.0, 360.0, 240.0, 135.0, 480, 270, "sintética")
    return ec.ConfigCamara(color, prof, np.eye(3), np.array([-0.0195, -0.005, 0.0]))


def escena_sintetica(cfg: ec.ConfigCamara, caja_xyz_color=(0.10, 0.05, 0.60), lado=0.12):
    """Profundidad (mm) vista desde el sensor de profundidad: mesa a 0.8 m y una caja.

    ``caja_xyz_color`` es el centro de la cara superior, en el marco de COLOR.
    """
    kd = cfg.profundidad
    t = cfg.traslacion_depth_a_color
    caja_d = np.asarray(caja_xyz_color) - t  # mismo punto en el marco de profundidad
    v, u = np.mgrid[0:kd.alto, 0:kd.ancho].astype(float)
    rayo_x, rayo_y = (u - kd.cx) / kd.fx, (v - kd.cy) / kd.fy
    z = np.full(u.shape, 0.80)
    x_en_caja, y_en_caja = rayo_x * caja_d[2], rayo_y * caja_d[2]
    sobre_caja = (np.abs(x_en_caja - caja_d[0]) < lado / 2) & (np.abs(y_en_caja - caja_d[1]) < lado / 2)
    z[sobre_caja] = caja_d[2]
    depth = np.round(z * 1000).astype(np.uint16)
    depth[::7, ::11] = 0  # huecos sin dato, como en el sensor real
    return depth


def punto_gemini(xyz, k: ec.Intrinsecos):
    """Lo que respondería un Gemini perfecto: [y, x] normalizado 0-1000."""
    u = k.fx * xyz[0] / xyz[2] + k.cx
    v = k.fy * xyz[1] / xyz[2] + k.cy
    return [v / (k.alto - 1) * 1000, u / (k.ancho - 1) * 1000]


# --- TODO 2 ---------------------------------------------------------------
def test_normalizado_a_pixel_orden_yx():
    u, v = ec.normalizado_a_pixel([250, 750], 1001, 501)
    assert (u, v) == pytest.approx((750.0, 125.0))


def test_normalizado_a_pixel_rechaza_fuera_de_rango():
    with pytest.raises(ValueError):
        ec.normalizado_a_pixel([1200, 10], 640, 480)


# --- TODO 3 ---------------------------------------------------------------
def test_profundidad_robusta_ignora_ceros_y_atipicos():
    depth = np.full((50, 50), 600, dtype=np.uint16)
    depth[20:30, 20:30:2] = 0          # huecos
    depth[25, 25] = 4000               # reflejo atípico
    assert ec.profundidad_robusta(depth, 25, 25, ventana=9) == pytest.approx(0.600)


def test_profundidad_robusta_sin_datos():
    assert ec.profundidad_robusta(np.zeros((20, 20), np.uint16), 10, 10, ventana=5) is None


def test_profundidad_robusta_en_el_borde():
    depth = np.full((10, 10), 500, dtype=np.uint16)
    assert ec.profundidad_robusta(depth, 0, 9, ventana=7) == pytest.approx(0.5)


# --- TODO 4 ---------------------------------------------------------------
def test_desproyectar_punto_principal(cfg):
    assert ec.desproyectar(640.0, 360.0, 0.7, cfg.color) == pytest.approx((0.0, 0.0, 0.7))


def test_desproyectar_ida_y_vuelta(cfg):
    k = cfg.color
    x, y, z = ec.desproyectar(900.0, 100.0, 0.55, k)
    assert k.fx * x / z + k.cx == pytest.approx(900.0)
    assert k.fy * y / z + k.cy == pytest.approx(100.0)


# --- Pipeline completo -----------------------------------------------------
def test_pipeline_recupera_la_caja(cfg):
    caja = (0.10, 0.05, 0.60)
    depth = escena_sintetica(cfg, caja)
    color = np.zeros((ALTO_C, ANCHO_C, 3), np.uint8)
    det = [{"label": "caja", "point": punto_gemini(caja, cfg.color)}]
    loc = ec.localizar(det, color, depth, cfg, registrar=True)[0]
    error = np.linalg.norm(np.subtract(loc["xyz_m"], caja))
    assert error < 0.005, f"error 3D {error * 1000:.1f} mm"


def test_sin_registro_falla_en_el_borde(cfg):
    """Ablación: cerca de los bordes, no registrar confunde la caja con la mesa."""
    caja, lado = (0.10, 0.05, 0.60), 0.12
    depth = escena_sintetica(cfg, caja, lado)
    color = np.zeros((ALTO_C, ANCHO_C, 3), np.uint8)
    errores_con, errores_sin = 0, 0
    for mm in [d for d in range(-30, 31, 2) if abs(d) >= 8]:  # lejos de la ventana de la mediana
        x = caja[0] - lado / 2 + mm / 1000         # alrededor del borde izquierdo de la caja
        esperado = 0.60 if mm > 0 else 0.80
        punto = (x, caja[1], esperado)
        det = [{"label": "p", "point": punto_gemini(punto, cfg.color)}]
        con = ec.localizar(det, color, depth, cfg, registrar=True)[0]["z_m"]
        sin = ec.localizar(det, color, depth, cfg, registrar=False)[0]["z_m"]
        errores_con += abs(con - esperado) > 0.05
        errores_sin += abs(sin - esperado) > 0.05
    assert errores_con == 0
    assert errores_sin >= 3


def test_distancia_entre_objetos(cfg):
    a, b = {"xyz_m": [0.0, 0.0, 0.6]}, {"xyz_m": [0.3, 0.4, 0.6]}
    assert ec.distancia_entre(a, b) == pytest.approx(0.5)


# --- Utilidades ya implementadas ------------------------------------------
def test_extraer_json_con_cercas_markdown():
    texto = 'Claro:\n```json\n[{"point": [10, 20], "label": "a"}]\n```'
    assert ec.extraer_json(texto) == [{"point": [10, 20], "label": "a"}]


def test_detecciones_desde_cajas():
    d = ec.detecciones_desde_respuesta([{"box_2d": [100, 200, 300, 400], "label": "x"}])
    assert d[0]["point"] == [200, 300]


def test_prompts_admiten_objeto():
    for tarea in ec.PROMPTS:
        assert "{objeto}" not in ec.construir_prompt(tarea, "la caja")


def test_config_por_defecto_y_escalado():
    cfg = ec.cargar_config(None, 1280, 960)  # 4:3 -> se escala desde 640x480
    assert cfg.color.ancho == 1280 and cfg.color.fx == pytest.approx(2 * 653.68229)
    with pytest.raises(KeyError):
        ec.cargar_config(None, 1234, 567)


def test_detecta_intrinsecos_absurdos():
    # Punto principal del archivo por defecto de kinova_vision para 1920x1080.
    malo = ec.Intrinsecos(2612.3, 2609.5, 946.4, 96.97, 1920, 1080)
    assert any("cy" in a for a in ec.advertencias_intrinsecos(malo))
