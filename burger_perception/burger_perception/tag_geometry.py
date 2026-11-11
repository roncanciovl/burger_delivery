# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Geometría de la localización AprilTag en el plano de la mesa, sin ROS.

Separada del nodo para probarla con ``colcon test`` sobre imágenes sintéticas. Ver la
teoría en ``education/talleres/TALLER_LOCALIZACION_APRILTAG_KINOVA_MICROROS.md`` §3.
"""

import math
from typing import Callable, Tuple

import cv2
import numpy as np


def make_detector() -> Callable:
    """Devolver un detector AprilTag 36h11 compatible con OpenCV 4.6 (Ubuntu 24.04) y 4.7+."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'ArucoDetector'):          # OpenCV >= 4.7
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return detector.detectMarkers
    parameters = cv2.aruco.DetectorParameters_create()  # OpenCV 4.6
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


def to_reference_plane(reference_px: np.ndarray, target_px: np.ndarray,
                       tag_size_m: float) -> np.ndarray:
    """
    Llevar las esquinas del tag objetivo al plano del tag de referencia, en metros.

    El marco del ``tag_mesa`` tiene origen en su centro, x hacia la derecha del tag
    impreso (de la esquina 0 a la 1) e y hacia arriba (de la 3 a la 0).

    :param reference_px: 4 esquinas del tag de referencia en píxeles, orden del detector.
    :param target_px: 4 esquinas del tag objetivo en píxeles.
    :param tag_size_m: lado del cuadro negro del tag de referencia.
    """
    h = tag_size_m / 2.0
    reference_m = np.array([[-h, h], [h, h], [h, -h], [-h, -h]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(reference_px.astype(np.float32), reference_m)
    return cv2.perspectiveTransform(
        target_px.reshape(1, 4, 2).astype(np.float32), homography)[0]


def to_image_center(shape: Tuple[int, int], target_px: np.ndarray,
                    pixels_per_meter: float) -> np.ndarray:
    """Modelo simple: origen en el centro de la imagen, escala fija, eje y invertido."""
    rows, cols = shape[:2]
    x = (target_px[:, 0] - cols / 2.0) / pixels_per_meter
    y = -(target_px[:, 1] - rows / 2.0) / pixels_per_meter
    return np.stack([x, y], axis=1)


def pose_from_corners(points: np.ndarray) -> Tuple[float, float, float]:
    """
    Pose 2D (x, y, theta) de un tag a partir de sus 4 esquinas en el plano métrico.

    La posición es el centro de las esquinas; theta, el ángulo del vector que va del
    centro al punto medio del borde derecho (esquinas 1 y 2), es decir, el eje x del tag.
    """
    center = points.mean(axis=0)
    forward = (points[1] + points[2]) / 2.0
    theta = math.atan2(forward[1] - center[1], forward[0] - center[0])
    return float(center[0]), float(center[1]), float(theta)
