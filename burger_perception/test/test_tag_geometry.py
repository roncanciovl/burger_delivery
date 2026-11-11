# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""Pruebas de la geometría de localización sobre una escena sintética en perspectiva."""

import math

from burger_perception.tag_geometry import (
    make_detector,
    pose_from_corners,
    to_image_center,
    to_reference_plane,
)
import cv2
import numpy as np
import pytest

PX_POR_M = 1000.0      # resolución de la "mesa" dibujada antes de la perspectiva
LADO_MESA = 1200       # px


def _tag(tag_id, lado_px):
    diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(diccionario, tag_id, lado_px)
    return cv2.aruco.drawMarker(diccionario, tag_id, lado_px)


def _pegar(mesa, tag, cx_m, cy_m, theta_deg):
    """Pegar un tag (con margen blanco) centrado en (cx, cy) m, girado theta."""
    margen = tag.shape[0] // 4
    con_margen = cv2.copyMakeBorder(tag, margen, margen, margen, margen,
                                    cv2.BORDER_CONSTANT, value=255)
    n = con_margen.shape[0]
    centro = (LADO_MESA / 2 + cx_m * PX_POR_M, LADO_MESA / 2 - cy_m * PX_POR_M)
    m = cv2.getRotationMatrix2D((n / 2, n / 2), theta_deg, 1.0)
    m[0, 2] += centro[0] - n / 2
    m[1, 2] += centro[1] - n / 2
    mascara = cv2.warpAffine(np.full_like(con_margen, 255), m, mesa.shape[::-1])
    pegado = cv2.warpAffine(con_margen, m, mesa.shape[::-1], borderValue=255)
    mesa[mascara > 0] = pegado[mascara > 0]


def _escena():
    """Tag 1 de 0.10 m en el origen y tag 5 en (0.30, 0.15) m a 30°, vistos inclinados."""
    mesa = np.full((LADO_MESA, LADO_MESA), 255, dtype=np.uint8)
    _pegar(mesa, _tag(1, 100), 0.0, 0.0, 0.0)
    _pegar(mesa, _tag(5, 60), 0.30, 0.15, 30.0)
    origen = np.float32([[0, 0], [LADO_MESA, 0], [LADO_MESA, LADO_MESA], [0, LADO_MESA]])
    destino = np.float32([[150, 80], [1050, 120], [1180, 1100], [20, 1000]])
    return cv2.warpPerspective(mesa, cv2.getPerspectiveTransform(origen, destino),
                               (LADO_MESA, LADO_MESA), borderValue=255)


def test_pose_en_el_plano_de_la_mesa_con_perspectiva():
    """La homografía del tag_mesa recupera posición y orientación del carrito."""
    gris = _escena()
    corners, ids, _ = make_detector()(gris)
    vistos = [int(i) for i in ids.ravel()]
    assert sorted(vistos) == [1, 5]
    referencia = corners[vistos.index(1)].reshape(4, 2)
    objetivo = corners[vistos.index(5)].reshape(4, 2)
    x, y, theta = pose_from_corners(to_reference_plane(referencia, objetivo, 0.10))
    assert x == pytest.approx(0.30, abs=0.01)
    assert y == pytest.approx(0.15, abs=0.01)
    assert math.degrees(theta) == pytest.approx(30.0, abs=2.0)


def test_pose_de_un_cuadrado_alineado():
    """Esquinas en el orden del detector: theta = 0 y centro exacto."""
    h = 0.05
    esquinas = np.array([[1 - h, 2 + h], [1 + h, 2 + h], [1 + h, 2 - h], [1 - h, 2 - h]])
    assert pose_from_corners(esquinas) == pytest.approx((1.0, 2.0, 0.0))


def test_modelo_simple_invierte_el_eje_v():
    """Sin referencia, subir en la imagen es y positiva."""
    esquinas = np.array([[310.0, 190.0], [330.0, 190.0], [330.0, 210.0], [310.0, 210.0]])
    puntos = to_image_center((400, 640), esquinas, 100.0)
    x, y, _ = pose_from_corners(puntos)
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)
