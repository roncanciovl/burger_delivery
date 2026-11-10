# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Verificación de la alineación entre la cámara de color y la de profundidad, sin ROS.

Un borde físico de un objeto debe verse en el mismo lugar en las dos cámaras. Se comparan así:

1. **Silueta en profundidad**: píxeles del objeto cercano en los saltos de distancia. Se salta
   la franja sin medida que el sensor deja junto al objeto (sombra del estéreo).
2. **Bordes en color**: Canny y transformada de distancia (cuántos píxeles hay hasta el borde
   más cercano).
3. **Proyección**: cada punto de la silueta se lleva a la imagen de color con los extrínsecos
   candidatos. El puntaje (*chamfer*) es la distancia media a un borde de color; cuanto menor,
   mejor alineadas están las imágenes.

Se buscan los extrínsecos con menor puntaje y se comparan con los que usa el sistema. La
búsqueda sólo es fiable si la escena tiene siluetas claras en varias direcciones: por eso
existe el veredicto ``NO CONCLUYENTE``.
"""

from typing import Dict, Sequence, Tuple

import cv2
import numpy as np

Intrinsics = Tuple[float, float, float, float]  # fx, fy, cx, cy
Translation = Tuple[float, float, float]

ALIGNED = 'ALINEADA'
MISALIGNED = 'DESALINEADA'
INCONCLUSIVE = 'NO CONCLUYENTE'


def _mark_line(depth: np.ndarray, valid: np.ndarray, mask: np.ndarray,
               jump_m: float, max_gap_px: int) -> None:
    """Marcar en ``mask`` el lado cercano de los saltos de una fila o columna."""
    indices = np.flatnonzero(valid)
    if indices.size < 2:
        return
    first, second = indices[:-1], indices[1:]
    close = (second - first) <= max_gap_px + 1
    delta = depth[second] - depth[first]
    mask[first[close & (delta > jump_m)]] = True
    mask[second[close & (-delta > jump_m)]] = True


def silhouette_points(depth_m: np.ndarray, jump_m: float = 0.02, max_gap_px: int = 25,
                      min_component_px: int = 8) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Extraer la silueta de los objetos cercanos en una imagen de profundidad.

    Un hueco sólo cuenta si al otro lado hay una medida más lejana. Así no se toma por silueta
    el borde de una zona sin medida rodeada de la misma superficie (por ejemplo, los dedos de
    la pinza, demasiado cerca para el sensor).

    :param depth_m: profundidad en metros; 0 o NaN significa sin medida.
    :param jump_m: salto mínimo de distancia entre los dos lados del borde.
    :param max_gap_px: ancho máximo de la franja sin medida que se puede saltar.
    :param min_component_px: los trozos de silueta más pequeños se descartan por ruido.
    :returns: columnas, filas y profundidades (m) de los puntos de la silueta.
    """
    valid = np.isfinite(depth_m) & (depth_m > 0)
    mask = np.zeros(depth_m.shape, dtype=bool)
    for row in range(depth_m.shape[0]):
        _mark_line(depth_m[row], valid[row], mask[row], jump_m, max_gap_px)
    for col in range(depth_m.shape[1]):
        _mark_line(depth_m[:, col], valid[:, col], mask[:, col], jump_m, max_gap_px)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8)
    large = np.zeros(count, dtype=bool)
    large[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_component_px
    rows, cols = np.nonzero(large[labels])
    return cols.astype(np.float64), rows.astype(np.float64), depth_m[rows, cols].astype(np.float64)


def _cloud(depth_m: np.ndarray, k: Intrinsics):
    """Puntos 3D (marco de la cámara) de los píxeles con medida, y sus columnas y filas."""
    fx, fy, cx, cy = k
    rows, cols = np.nonzero(np.isfinite(depth_m) & (depth_m > 0))
    z = depth_m[rows, cols].astype(np.float64)
    return cols, rows, np.column_stack([(cols - cx) * z / fx, (rows - cy) * z / fy, z])


def fit_table_plane(points: np.ndarray, iters: int = 400, tol_m: float = 0.008,
                    seed: int = 0) -> Tuple[np.ndarray, float, float]:
    """
    Ajustar por RANSAC el plano dominante (la mesa) y refinarlo con sus puntos de apoyo.

    :returns: normal unitaria orientada hacia la cámara, ``d`` (``n·p + d`` es la altura sobre
        el plano) y la fracción de puntos que lo apoyan.
    """
    rng = np.random.default_rng(seed)
    sample = points[rng.choice(len(points), min(len(points), 20000), replace=False)]
    best_count, best = 0, None
    for _ in range(iters):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            continue
        normal /= norm
        count = int((np.abs(sample @ normal - normal @ a) < tol_m).sum())
        if count > best_count:
            best_count, best = count, (normal, -normal @ a)
    normal, d = best
    support = sample[np.abs(sample @ normal + d) < tol_m]
    center = support.mean(axis=0)
    normal = np.linalg.svd(support - center, full_matrices=False)[2][2]
    d = -normal @ center
    if d < 0:  # la cámara (origen) queda del lado positivo: altura > 0 sobre la mesa
        normal, d = -normal, -d
    return normal, float(d), best_count / len(sample)


def top_face_contour(depth_m: np.ndarray, k: Intrinsics, min_height_m: float = 0.03,
                     band_m: float = 0.015, min_area_px: int = 300
                     ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Contorno de la cara superior del objeto más grande sobre la mesa.

    Separa el objeto por su altura sobre el plano de la mesa, así que funciona aunque la cámara
    esté inclinada y los costados del objeto se vean en rampa (sin salto de distancia).

    :param k: intrínsecos de la imagen ``depth_m`` (los del color si está registrada).
    :returns: columnas, filas y profundidades (m) del borde de la cara superior; vacíos si no
        hay un objeto que sobresalga ``min_height_m``.
    """
    empty = (np.array([]), np.array([]), np.array([]))
    cols, rows, points = _cloud(depth_m, k)
    if len(points) < 100:
        return empty
    normal, d, _ = fit_table_plane(points)
    height = np.zeros(depth_m.shape, dtype=np.float64)
    height[rows, cols] = points @ normal + d
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (height > min_height_m).astype(np.uint8), connectivity=8)
    if count < 2 or stats[1:, cv2.CC_STAT_AREA].max() < min_area_px:
        return empty
    raised = labels == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    top = raised & (np.abs(height - np.median(height[raised])) < band_m)
    top = cv2.morphologyEx(top.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(top, connectivity=8)
    if count < 2:
        return empty
    top = labels == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    edge = top & ~cv2.erode(top.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    r, c = np.nonzero(edge)
    return c.astype(np.float64), r.astype(np.float64), depth_m[r, c].astype(np.float64)


def edge_distance_field(gray: np.ndarray, canny_low: int = 40,
                        canny_high: int = 120) -> np.ndarray:
    """Devolver, para cada píxel, la distancia en píxeles al borde de color más cercano."""
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), canny_low, canny_high)
    return cv2.distanceTransform((edges == 0).astype(np.uint8), cv2.DIST_L2, 5)


def project_points(u: np.ndarray, v: np.ndarray, z: np.ndarray, depth_k: Intrinsics,
                   color_k: Intrinsics, t: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """Llevar puntos de la imagen de profundidad a la de color: ``p_color = p_depth + t``."""
    fx, fy, cx, cy = depth_k
    x = (u - cx) * z / fx + t[0]
    y = (v - cy) * z / fy + t[1]
    zc = z + t[2]
    fx, fy, cx, cy = color_k
    return x * fx / zc + cx, y * fy / zc + cy


def chamfer_score(field: np.ndarray, u: np.ndarray, v: np.ndarray, cap_px: float) -> float:
    """Distancia media a un borde; se trunca en ``cap_px`` y lo que cae fuera vale ``cap_px``."""
    rows, cols = field.shape
    ui, vi = np.rint(u).astype(np.int64), np.rint(v).astype(np.int64)
    inside = (ui >= 0) & (ui < cols) & (vi >= 0) & (vi < rows)
    distance = np.full(u.shape, cap_px, dtype=np.float64)
    distance[inside] = np.minimum(field[vi[inside], ui[inside]], cap_px)
    return float(distance.mean())


def _subsample(max_points: int, *arrays):
    """Reducir arreglos de igual longitud a ``max_points`` elementos equiespaciados."""
    size = arrays[0].size
    if size <= max_points:
        return arrays
    index = np.linspace(0, size - 1, max_points).astype(np.int64)
    return tuple(a[index] for a in arrays)


def search_translation(field: np.ndarray, u: np.ndarray, v: np.ndarray, z: np.ndarray,
                       depth_k: Intrinsics, color_k: Intrinsics,
                       x_range_m: Tuple[float, float] = (-0.06, 0.02),
                       y_range_m: Tuple[float, float] = (-0.04, 0.03),
                       cap_px: float = 30.0, step_m: float = 0.001,
                       refine_step_m: float = 0.00025) -> Dict:
    """
    Buscar la traslación profundidad → color (z = 0) con menor puntaje.

    Primero recorre una rejilla y luego afina alrededor del mínimo.

    :returns: diccionario con ``t``, ``score``, y la rejilla ``xs``, ``ys``, ``scores``.
    """
    u, v, z = _subsample(4000, u, v, z)

    def score(tx, ty):
        return chamfer_score(field, *project_points(u, v, z, depth_k, color_k, (tx, ty, 0.0)),
                             cap_px)

    xs = np.arange(x_range_m[0], x_range_m[1] + step_m / 2.0, step_m)
    ys = np.arange(y_range_m[0], y_range_m[1] + step_m / 2.0, step_m)
    scores = np.array([[score(x, y) for x in xs] for y in ys])
    j, i = np.unravel_index(np.argmin(scores), scores.shape)

    best = (float(scores[j, i]), float(xs[i]), float(ys[j]))
    offsets = np.arange(-step_m, step_m + refine_step_m / 2.0, refine_step_m)
    for dy in offsets:
        for dx in offsets:
            candidate = score(xs[i] + dx, ys[j] + dy)
            if candidate < best[0]:
                best = (candidate, float(xs[i] + dx), float(ys[j] + dy))
    return {'t': (best[1], best[2], 0.0), 'score': best[0], 'xs': xs, 'ys': ys, 'scores': scores}


def search_shift(field: np.ndarray, u: np.ndarray, v: np.ndarray, range_px: int = 90,
                 cap_px: float = 30.0) -> Dict:
    """Buscar el desplazamiento en píxeles que mejor lleva una silueta a los bordes de color."""
    u, v = _subsample(4000, u, v)

    def score(du, dv):
        return chamfer_score(field, u + du, v + dv, cap_px)

    best = min(((score(du, dv), du, dv) for dv in range(-range_px, range_px + 1, 3)
                for du in range(-range_px, range_px + 1, 3)), key=lambda item: item[0])
    best = min(((score(best[1] + du, best[2] + dv), best[1] + du, best[2] + dv)
                for dv in range(-3, 4) for du in range(-3, 4)), key=lambda item: item[0])
    return {'du': best[1], 'dv': best[2], 'score': best[0], 'score_at_zero': score(0, 0)}


def direction_spread(u: np.ndarray, v: np.ndarray) -> float:
    """
    Medir si la silueta tiene bordes en varias direcciones (0 = una sola recta, 1 = disco).

    Con una sola recta sólo se mide el desplazamiento perpendicular a ella.
    """
    if u.size < 3:
        return 0.0
    eigen = np.linalg.eigvalsh(np.cov(np.vstack([u, v])))
    return float(np.sqrt(max(eigen[0], 0.0) / eigen[1])) if eigen[1] > 0 else 0.0


def _ambiguity(search: Dict, exclusion_m: float) -> float:
    """Puntaje del mejor mínimo lejano dividido entre el del global (cerca de 1 = ambiguo)."""
    x_grid, y_grid = np.meshgrid(search['xs'], search['ys'])
    far = np.hypot(x_grid - search['t'][0], y_grid - search['t'][1]) > exclusion_m
    if not far.any() or search['score'] <= 0.0:
        return float('inf')
    return float(search['scores'][far].min() / search['score'])


def evaluate_alignment(field: np.ndarray, u: np.ndarray, v: np.ndarray, z: np.ndarray,
                       depth_k: Intrinsics, color_k: Intrinsics,
                       hypotheses: Dict[str, Translation], reference: str,
                       tol_mm: float = 3.0, min_points: int = 200, min_contrast: float = 0.25,
                       min_ambiguity: float = 1.05, exclusion_m: float = 0.005,
                       min_spread: float = 0.2, margin_px: int = 40) -> Dict:
    """
    Comparar los extrínsecos candidatos con los que mejor explican la imagen.

    :param hypotheses: extrínsecos a evaluar, por nombre (metros).
    :param reference: nombre de la hipótesis que usa el sistema; define el veredicto.
    :param tol_mm: diferencia máxima aceptada entre la referencia y la mejor estimación.
    :param margin_px: margen del borde de la imagen de color que no se usa.
    :returns: diccionario con el veredicto, su motivo, la estimación y cada hipótesis.
    """
    # La profundidad ve un campo más amplio que el color (en 1080p, unos 67° frente a 40°):
    # sólo cuenta la silueta que cae dentro de la imagen de color.
    pu, pv = project_points(u, v, z, depth_k, color_k, hypotheses[reference])
    rows, cols = field.shape
    inside = ((pu >= margin_px) & (pu < cols - margin_px)
              & (pv >= margin_px) & (pv < rows - margin_px))
    u, v, z = u[inside], v[inside], z[inside]
    report = {'points': int(u.size), 'points_outside_color': int((~inside).sum()),
              'verdict': INCONCLUSIVE, 'reason': '', 'hypotheses': {}}
    if u.size < min_points:
        report['reason'] = (f'silueta insuficiente: {u.size} puntos (mínimo {min_points}); '
                            'coloque un objeto de bordes rectos con salto de altura')
        return report

    search = search_translation(field, u, v, z, depth_k, color_k)
    median_z = float(np.median(z))
    fx = color_k[0]
    mean_score = float(search['scores'].mean())
    report.update({
        'median_depth_m': median_z,
        'best_t_mm': [round(c * 1000.0, 2) for c in search['t']],
        'best_chamfer_px': search['score'],
        'contrast': (mean_score - search['score']) / mean_score if mean_score > 0 else 0.0,
        'ambiguity': _ambiguity(search, exclusion_m),
        'direction_spread': direction_spread(u, v),
    })
    for name, t in hypotheses.items():
        error_m = float(np.hypot(t[0] - search['t'][0], t[1] - search['t'][1]))
        projected = project_points(u, v, z, depth_k, color_k, t)
        report['hypotheses'][name] = {
            't_mm': [round(c * 1000.0, 2) for c in t],
            'chamfer_px': chamfer_score(field, *projected, 30.0),
            'error_mm': error_m * 1000.0,
            'error_px_at_median_depth': error_m * fx / median_z,
        }

    if report['direction_spread'] < min_spread:
        report['reason'] = (f'la silueta es casi una sola recta (dispersión '
                            f'{report["direction_spread"]:.2f}, mínimo {min_spread:.2f}): ponga '
                            'un objeto completo a la vista, con bordes en dos direcciones')
    elif report['contrast'] < min_contrast:
        report['reason'] = (f'el puntaje casi no cambia con los extrínsecos (contraste '
                            f'{report["contrast"]:.2f}, mínimo {min_contrast:.2f})')
    elif report['ambiguity'] < min_ambiguity:
        report['reason'] = (f'hay otro mínimo casi igual de bueno (razón '
                            f'{report["ambiguity"]:.2f}, mínimo {min_ambiguity:.2f}); use un '
                            'objeto sin textura impresa y con bordes en dos direcciones')
    else:
        error_mm = report['hypotheses'][reference]['error_mm']
        report['verdict'] = ALIGNED if error_mm <= tol_mm else MISALIGNED
        report['reason'] = (f'"{reference}" difiere {error_mm:.1f} mm de la mejor estimación '
                            f'(tolerancia {tol_mm:.1f} mm)')
    return report
