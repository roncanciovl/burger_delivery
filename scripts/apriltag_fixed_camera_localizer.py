#!/usr/bin/env python3
"""
Envoltorio de compatibilidad del localizador AprilTag.

El nodo se trasladó al paquete ``burger_perception`` (ejecutable ``apriltag_localizer``,
módulo ``burger_perception/apriltag_localizer.py``). Este archivo se conserva para que los
comandos de los talleres (``python3 scripts/apriltag_fixed_camera_localizer.py ...``) sigan
funcionando: usa el paquete instalado y, si no está compilado, el código fuente del repo.
"""

import os
import sys

try:
    from burger_perception.apriltag_localizer import main
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'burger_perception'))
    from burger_perception.apriltag_localizer import main

if __name__ == '__main__':
    main()
