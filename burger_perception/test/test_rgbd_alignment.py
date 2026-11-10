# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""Pruebas de la verificación RGB-D sobre una escena sintética con extrínsecos conocidos."""

from burger_perception.rgbd_alignment import (
    ALIGNED,
    edge_distance_field,
    evaluate_alignment,
    INCONCLUSIVE,
    MISALIGNED,
    project_points,
    search_shift,
    silhouette_points,
    top_face_contour,
)
import cv2
import numpy as np
import pytest

DEPTH_K = (360.0, 360.0, 240.0, 135.0)
COLOR_K = (2612.0, 2609.0, 946.0, 540.0)
TRUE_T = (-0.0275, -0.0096, 0.0)
BOX = (200, 360, 80, 200)     # u0, u1, v0, v1 en la imagen de profundidad
BOX_Z, TABLE_Z = 0.47, 0.56


def _depth_scene():
    depth = np.full((270, 480), TABLE_Z, dtype=np.float32)
    u0, u1, v0, v1 = BOX
    depth[v0:v1, u0:u1] = BOX_Z
    return depth


def _color_scene(t=TRUE_T):
    u0, u1, v0, v1 = BOX
    corners_u = np.array([u0 - 0.5, u1 - 0.5], dtype=np.float64)
    corners_v = np.array([v0 - 0.5, v1 - 0.5], dtype=np.float64)
    z = np.full(2, BOX_Z)
    us, vs = project_points(corners_u, corners_v, z, DEPTH_K, COLOR_K, t)
    gray = np.full((1080, 1920), 200, dtype=np.uint8)
    cv2.rectangle(gray, (int(us[0]), int(vs[0])), (int(us[1]), int(vs[1])), 60, -1)
    return gray


def _evaluate(depth, gray, reference_t):
    u, v, z = silhouette_points(depth)
    field = edge_distance_field(gray)
    return evaluate_alignment(field, u, v, z, DEPTH_K, COLOR_K,
                              {'sistema': reference_t}, 'sistema')


def test_la_silueta_queda_en_el_objeto_cercano():
    """Los puntos del borde pertenecen a la caja, no a la mesa."""
    _, _, z = silhouette_points(_depth_scene())
    assert z.size > 0
    assert z == pytest.approx(BOX_Z)


def test_salta_la_franja_sin_medida_junto_al_objeto():
    """La sombra del estéreo no oculta la silueta: se compara con el otro lado del hueco."""
    depth = _depth_scene()
    u0, u1, v0, v1 = BOX
    depth[v0:v1, u1:u1 + 12] = 0.0
    u, _, z = silhouette_points(depth)
    assert (u == u1 - 1).any()
    assert z == pytest.approx(BOX_Z)


def test_un_hueco_en_la_misma_superficie_no_es_silueta():
    """Una zona sin medida rodeada de mesa (como la pinza) no genera puntos."""
    depth = np.full((270, 480), TABLE_Z, dtype=np.float32)
    depth[240:265, 100:160] = 0.0
    assert silhouette_points(depth)[0].size == 0


def test_recupera_los_extrinsecos_de_la_escena():
    """La búsqueda encuentra la traslación con la que se generó la imagen de color."""
    report = _evaluate(_depth_scene(), _color_scene(), TRUE_T)
    assert report['best_t_mm'][0] == pytest.approx(TRUE_T[0] * 1000.0, abs=1.5)
    assert report['best_t_mm'][1] == pytest.approx(TRUE_T[1] * 1000.0, abs=1.5)


def test_veredicto_alineada_con_los_extrinsecos_correctos():
    """Los extrínsecos reales dan ALINEADA."""
    report = _evaluate(_depth_scene(), _color_scene(), TRUE_T)
    assert report['verdict'] == ALIGNED


def test_ignora_la_silueta_fuera_del_campo_del_color():
    """La profundidad ve más que el color: lo que cae fuera no entra en el puntaje."""
    depth = _depth_scene()
    depth[0:30, 440:480] = BOX_Z
    report = _evaluate(depth, _color_scene(), TRUE_T)
    assert report['points_outside_color'] > 0
    assert report['verdict'] == ALIGNED


def test_veredicto_desalineada_con_los_extrinsecos_del_otro_origen():
    """Usar 19.5/5 mm cuando la escena es de 27.5/9.6 mm se detecta."""
    report = _evaluate(_depth_scene(), _color_scene(), (-0.0195, -0.005, 0.0))
    assert report['verdict'] == MISALIGNED
    assert report['hypotheses']['sistema']['error_mm'] > 3.0


def test_sin_siluetas_no_se_concluye():
    """Una escena plana no permite afirmar nada."""
    depth = np.full((270, 480), TABLE_Z, dtype=np.float32)
    report = _evaluate(depth, np.full((1080, 1920), 200, dtype=np.uint8), TRUE_T)
    assert report['verdict'] == INCONCLUSIVE


def test_un_solo_borde_recto_no_permite_concluir():
    """Con una sola arista sólo se mide lo perpendicular a ella: no concluyente."""
    depth = np.full((270, 480), TABLE_Z, dtype=np.float32)
    depth[:120, :] = BOX_Z
    gray = np.full((1080, 1920), 200, dtype=np.uint8)
    gray[:500, :] = 60
    report = _evaluate(depth, gray, TRUE_T)
    assert report['verdict'] == INCONCLUSIVE
    assert report['direction_spread'] < 0.2


def test_cara_superior_con_la_mesa_inclinada():
    """Con la cámara inclinada 18°, la caja se separa por su altura sobre el plano de la mesa."""
    fx, fy, cx, cy = DEPTH_K
    rows, cols = np.mgrid[0:270, 0:480].astype(np.float64)
    rays = np.stack([(cols - cx) / fx, (rows - cy) / fy, np.ones_like(cols)], axis=-1)
    tilt = np.radians(18.0)
    normal = np.array([0.0, np.sin(tilt), -np.cos(tilt)])   # mesa vista desde arriba, inclinada
    depth = (0.70 / -(rays @ normal)).astype(np.float32)    # z de la mesa: n·p + 0.70 = 0
    box = (rows >= 90) & (rows < 180) & (cols >= 180) & (cols < 320)
    depth[box] = (0.62 / -(rays[box] @ normal)).astype(np.float32)   # tapa 8 cm más alta
    u, v, z = top_face_contour(depth, DEPTH_K)
    assert u.size > 300
    assert u.min() == 180 and u.max() == 319 and v.min() == 90 and v.max() == 179


def test_sin_objeto_sobre_la_mesa_no_hay_contorno():
    """Una mesa vacía no produce contorno."""
    depth = np.full((270, 480), TABLE_Z, dtype=np.float32)
    assert top_face_contour(depth, DEPTH_K)[0].size == 0


def test_desplazamiento_residual_en_pixeles():
    """Una silueta corrida 20 y -15 píxeles se detecta en la imagen de color."""
    gray = np.full((1080, 1920), 200, dtype=np.uint8)
    cv2.rectangle(gray, (600, 300), (1400, 800), 60, -1)
    field = edge_distance_field(gray)
    top = np.column_stack([np.arange(600.0, 1400.0, 4.0), np.full(200, 300.0)])
    side = np.column_stack([np.full(125, 600.0), np.arange(300.0, 800.0, 4.0)])
    points = np.vstack([top, side]) - np.array([20.0, -15.0])
    shift = search_shift(field, points[:, 0], points[:, 1])
    assert shift['du'] == pytest.approx(20, abs=2)
    assert shift['dv'] == pytest.approx(-15, abs=2)
    assert shift['score'] < shift['score_at_zero']
