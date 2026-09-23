# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""Pruebas de la referencia AprilTag por PnP y de la estadística del benchmark."""

from burger_perception.benchmark_stats import error_3d_mm, summarize
from burger_perception.tag_geometry import tag_center_in_camera
import cv2
import numpy as np
import pytest

K = np.array([[900.0, 0.0, 960.0], [0.0, 900.0, 540.0], [0.0, 0.0, 1.0]])
DIST = np.zeros(5)


def test_pnp_recupera_el_centro_del_tag():
    """Proyectar un tag conocido y recuperar su centro por PnP."""
    h = 0.03
    objeto = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    rvec = np.array([0.4, -0.2, 0.1])
    tvec = np.array([0.12, -0.05, 0.65])
    esquinas, _ = cv2.projectPoints(objeto, rvec, tvec, K, DIST)
    xyz, (u, v) = tag_center_in_camera(esquinas.reshape(4, 2), 0.06, K, DIST)
    assert xyz == pytest.approx(tvec, abs=1e-4)
    proyectado, _ = cv2.projectPoints(np.zeros((1, 3)), rvec, tvec, K, DIST)
    assert (u, v) == pytest.approx(tuple(proyectado.ravel()), abs=1e-3)


def test_error_3d_en_milimetros():
    """10 mm en x y 0 en el resto."""
    assert error_3d_mm((0.1, 0.2, 0.5), (0.11, 0.2, 0.5)) == pytest.approx(10.0)


def test_resumen_ignora_ausentes():
    """Los ensayos fallidos (None) no cuentan en la estadística."""
    r = summarize([3.0, None, 4.0, float('nan')])
    assert r['n'] == 2
    assert r['media'] == pytest.approx(3.5)
    assert r['rmse'] == pytest.approx((12.5) ** 0.5)
    assert summarize([]) == {'n': 0}
