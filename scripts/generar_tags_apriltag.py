#!/usr/bin/env python3
# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Genera AprilTags 36h11 imprimibles a tamaño exacto para la mesa y los carritos.

El localizador (``apriltag_fixed_camera_localizer.py``) mide en metros a partir del lado
del cuadro negro del ``tag_mesa`` (``tag_size_m``): si el tag impreso no mide lo que se
declara, todas las posiciones salen escaladas por el mismo factor. Por eso este script
escribe cada tag como PNG con la resolución (DPI) embebida, de modo que al imprimirlo
al 100 % (sin "ajustar a la página") el cuadro negro mida exactamente ``--lado-mm``.

Cada imagen incluye:
* el margen blanco de un módulo que exige el detector (sin él no se detecta);
* una regla de 50 mm bajo el tag, para comprobar la escala de la impresión con un
  calibrador antes de pegarlo;
* el ID y el lado nominal en texto.

Uso::

    python3 scripts/generar_tags_apriltag.py --ids 1 --lado-mm 100 --salida tags/   # tag_mesa
    python3 scripts/generar_tags_apriltag.py --ids 5 6 --lado-mm 60 --salida tags/  # carritos

Luego registra los IDs y lados MEDIDOS (no los nominales) en ``vision_setup/tags_fisicos.yaml``.
"""

import argparse
import os
import struct
import sys
import zlib

import cv2
import numpy as np

MM_POR_PULGADA = 25.4
# Un tag 36h11 son 8×8 módulos en su borde negro (6×6 de datos más el borde de un
# módulo); el detector necesita además un módulo blanco alrededor.
MODULOS_NEGROS = 8


def tag_36h11(tag_id: int, lado_px: int) -> np.ndarray:
    """Devolver el tag (sólo el cuadro negro) como imagen de ``lado_px`` × ``lado_px``."""
    diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'generateImageMarker'):          # OpenCV >= 4.7
        return cv2.aruco.generateImageMarker(diccionario, tag_id, lado_px)
    return cv2.aruco.drawMarker(diccionario, tag_id, lado_px)  # OpenCV 4.6


def lamina(tag_id: int, lado_mm: float, dpi: int) -> np.ndarray:
    """Componer la lámina imprimible: tag, margen blanco, regla de 50 mm y rótulo."""
    px_mm = dpi / MM_POR_PULGADA
    # Múltiplo de 8 para que cada módulo tenga un número entero de píxeles.
    lado_px = int(round(lado_mm * px_mm / MODULOS_NEGROS)) * MODULOS_NEGROS
    modulo = lado_px // MODULOS_NEGROS
    tag = tag_36h11(tag_id, lado_px)

    regla_mm = 50.0
    regla_px = int(round(regla_mm * px_mm))
    alto_extra = int(round(22 * px_mm))
    ancho = max(lado_px + 2 * modulo, regla_px + 2 * modulo)
    alto = lado_px + 2 * modulo + alto_extra
    hoja = np.full((alto, ancho), 255, dtype=np.uint8)

    x0 = (ancho - lado_px) // 2
    hoja[modulo:modulo + lado_px, x0:x0 + lado_px] = tag

    # Regla: barra de 50 mm con marcas cada 10 mm.
    y = lado_px + 2 * modulo + int(round(4 * px_mm))
    xr = (ancho - regla_px) // 2
    grosor = max(1, int(round(0.4 * px_mm)))
    cv2.line(hoja, (xr, y), (xr + regla_px, y), 0, grosor)
    for k in range(6):
        xk = xr + int(round(k * 10 * px_mm))
        cv2.line(hoja, (xk, y - int(2 * px_mm)), (xk, y + int(2 * px_mm)), 0, grosor)

    texto = f'36h11 ID {tag_id} | lado {lado_mm:g} mm | regla 50 mm | imprimir al 100 %'
    escala = 0.35 * px_mm / 10.0
    cv2.putText(hoja, texto, (modulo, alto - int(round(6 * px_mm))),
                cv2.FONT_HERSHEY_SIMPLEX, escala, 0, max(1, grosor // 2), cv2.LINE_AA)
    real_mm = lado_px / px_mm
    if abs(real_mm - lado_mm) > 0.2:
        print(f'  aviso: con {dpi} DPI el lado queda en {real_mm:.2f} mm '
              f'(pedido {lado_mm:g} mm); sube --dpi para acercarlo')
    return hoja


def guardar_png(ruta: str, imagen: np.ndarray, dpi: int) -> None:
    """Guardar el PNG con el chunk pHYs, que fija el tamaño físico al imprimir."""
    ok, datos = cv2.imencode('.png', imagen)
    if not ok:
        raise RuntimeError(f'no se pudo codificar {ruta}')
    datos = bytearray(datos.tobytes())
    ppm = int(round(dpi / 0.0254))
    cuerpo = b'pHYs' + struct.pack('>IIB', ppm, ppm, 1)
    chunk = struct.pack('>I', 9) + cuerpo + struct.pack('>I', zlib.crc32(cuerpo) & 0xFFFFFFFF)
    fin_ihdr = 8 + 8 + 13 + 4   # firma + (longitud, tipo) + datos IHDR + CRC
    datos[fin_ihdr:fin_ihdr] = chunk
    with open(ruta, 'wb') as salida:
        salida.write(datos)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--ids', type=int, nargs='+', required=True)
    parser.add_argument('--lado-mm', type=float, required=True,
                        help='lado del cuadro negro, sin el margen blanco')
    parser.add_argument('--dpi', type=int, default=600)
    parser.add_argument('--salida', default='tags')
    args = parser.parse_args()

    os.makedirs(args.salida, exist_ok=True)
    for tag_id in args.ids:
        if not 0 <= tag_id < 587:
            print(f'ID {tag_id} fuera de la familia 36h11 (0..586)', file=sys.stderr)
            return 1
        ruta = os.path.join(args.salida, f'tag36h11_id{tag_id}_{args.lado_mm:g}mm.png')
        guardar_png(ruta, lamina(tag_id, args.lado_mm, args.dpi), args.dpi)
        print(f'{ruta}')
    print('Imprime al 100 % y mide el lado del cuadro negro y la regla con calibrador.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
