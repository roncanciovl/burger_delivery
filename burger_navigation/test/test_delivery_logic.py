# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""Pruebas de la geometría SE(2) y de las zonas de entrega."""

import math

from burger_navigation.delivery_logic import (
    compose,
    inverse,
    map_to_odom,
    parse_slots,
    pose_error,
    quaternion_from_yaw,
    resolve_tolerance,
    within_tolerance,
    yaw_from_quaternion,
)
import pytest


def test_cuaternion_ida_y_vuelta():
    """Pasar de yaw a cuaternión y de vuelta conserva el ángulo."""
    for yaw in (-3.0, -0.5, 0.0, 1.2, 3.1):
        assert yaw_from_quaternion(*quaternion_from_yaw(yaw)) == pytest.approx(yaw)


def test_componer_con_la_inversa_es_identidad():
    """Componer una transformación con su inversa da la identidad."""
    a = (0.4, -0.2, 1.0)
    assert compose(a, inverse(a)) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_componer_gira_la_traslacion():
    """Avanzar 1 m en un marco girado 90° es moverse en +y del padre."""
    assert compose((1.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0)) == pytest.approx(
        (1.0, 1.0, math.pi / 2))


def test_correccion_map_odom_cierra_la_cadena():
    """map->odom ∘ odom->base debe reproducir map->base."""
    map_base = (2.0, 1.0, 0.3)
    odom_base = (0.5, -0.1, -0.2)
    assert compose(map_to_odom(map_base, odom_base), odom_base) == pytest.approx(map_base)


def test_error_de_pose_con_cruce_de_pi():
    """179° frente a -179° son 2° de error, no 358°."""
    _, dyaw = pose_error((0, 0, math.radians(179)), (0, 0, math.radians(-179)))
    assert math.degrees(dyaw) == pytest.approx(2.0)


def test_tolerancias():
    """Dentro en posición y fuera en ángulo no es llegada."""
    assert within_tolerance((0, 0, 0), (0.02, 0, 0.05), 0.03, 0.1)
    assert not within_tolerance((0, 0, 0), (0.02, 0, 0.2), 0.03, 0.1)


def test_slots_validos_y_faltantes():
    """Los grados pasan a radianes y un campo faltante se nombra en el error."""
    slots = parse_slots({'s1': {'frame': 'tag_mesa', 'x': 0.35, 'y': 0.1, 'yaw_deg': 90,
                                'tolerance_xy': 0.02}})
    assert slots['s1']['pose'][2] == pytest.approx(math.pi / 2)
    assert slots['s1']['tol_yaw'] == 0.0
    with pytest.raises(ValueError, match='yaw_deg'):
        parse_slots({'s2': {'frame': 'map', 'x': 0, 'y': 0}})


def test_prioridad_de_tolerancias():
    """Pedida > slot > por defecto."""
    assert resolve_tolerance(0.05, 0.02, 0.03) == 0.05
    assert resolve_tolerance(0.0, 0.02, 0.03) == 0.02
    assert resolve_tolerance(0.0, 0.0, 0.03) == 0.03
