#!/usr/bin/env python3
"""
Leer del robot los intrínsecos (color y profundidad) y los extrínsecos del módulo de
visión con la Kortex API, y escribirlos en config_camara_kinova.json.

Lo ejecuta UNA persona (docente o monitor), una vez, coordinada con la estación
anfitriona; el JSON resultante se comparte con todos los grupos. No abre sesión
cíclica ni mueve el robot: solo lee la configuración del módulo de visión.

Instalación (entorno virtual aparte; la Kortex API fija versiones antiguas de protobuf):

    python3 -m venv ~/venv_kortex && source ~/venv_kortex/bin/activate
    pip install --no-deps kortex_api-2.6.0.post3-py3-none-any.whl   # descargado de Kinova
    pip install "protobuf==3.20.3"

Uso:
    python3 leer_calibracion_kortex.py --ip 192.168.1.10 --usuario admin
"""

from __future__ import annotations

import argparse
import getpass
import json
import re
from pathlib import Path

AQUI = Path(__file__).resolve().parent


def resolucion(nombre_enum: str) -> tuple[int, int]:
    m = re.search(r"(\d+)x(\d+)", nombre_enum)
    if not m:
        raise ValueError(f"Resolución no reconocida: {nombre_enum}")
    return int(m.group(1)), int(m.group(2))


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", default="192.168.1.10")
    p.add_argument("--usuario", default="admin")
    p.add_argument("--salida", default=str(AQUI / "config_camara_kinova.json"))
    args = p.parse_args()

    from kortex_api.RouterClient import RouterClient
    from kortex_api.SessionManager import SessionManager
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
    from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient
    from kortex_api.autogen.messages import DeviceConfig_pb2, Session_pb2, VisionConfig_pb2

    transporte = TCPTransport()
    router = RouterClient(transporte, lambda e: print(f"[kortex] {e}"))
    transporte.connect(args.ip, 10000)
    sesion = Session_pb2.CreateSessionInfo()
    sesion.username = args.usuario
    sesion.password = getpass.getpass(f"Contraseña de {args.usuario}@{args.ip}: ")
    sesion.session_inactivity_timeout = 60000
    sesion.connection_inactivity_timeout = 2000
    gestor = SessionManager(router)
    gestor.CreateSession(sesion)
    try:
        dispositivos = DeviceManagerClient(router).ReadAllDevices()
        ids = [d.device_identifier for d in dispositivos.device_handle
               if d.device_type == DeviceConfig_pb2.VISION]
        if not ids:
            raise RuntimeError("El robot no reporta módulo de visión")
        vision = VisionConfigClient(router)

        def intrinsecos(sensor) -> dict:
            sid = VisionConfig_pb2.SensorIdentifier()
            sid.sensor = sensor
            k = vision.GetIntrinsicParameters(sid, ids[0])
            ancho, alto = resolucion(VisionConfig_pb2.Resolution.Name(k.resolution))
            return {"fx": k.focal_length_x, "fy": k.focal_length_y, "cx": k.principal_point_x,
                    "cy": k.principal_point_y, "ancho": ancho, "alto": alto,
                    "fuente": f"Kortex API GetIntrinsicParameters ({args.ip})"}

        color = intrinsecos(VisionConfig_pb2.SENSOR_COLOR)
        prof = intrinsecos(VisionConfig_pb2.SENSOR_DEPTH)
        e = vision.GetExtrinsicParameters(ids[0])
        r = e.rotation
        rot = [r.row1.column1, r.row1.column2, r.row1.column3,
               r.row2.column1, r.row2.column2, r.row2.column3,
               r.row3.column1, r.row3.column2, r.row3.column3]
        tras = [e.translation.t_x, e.translation.t_y, e.translation.t_z]
    finally:
        gestor.CloseSession()
        transporte.disconnect()

    salida = Path(args.salida)
    datos = json.loads(salida.read_text(encoding="utf-8")) if salida.exists() else {"color": {}}
    datos["color"][f"{color['ancho']}x{color['alto']}"] = color
    datos["profundidad"] = prof
    datos["extrinsecos_depth_a_color"] = {
        "_comentario": f"Kortex API GetExtrinsicParameters ({args.ip}). Verifique las unidades: "
                       "si la traslación supera 0.1, probablemente viene en milímetros.",
        "rotacion": rot, "traslacion_m": tras}
    salida.write_text(json.dumps(datos, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"color": color, "profundidad": prof, "R": rot, "t": tras}, indent=2))
    print(f"[OK] Escrito en {salida}")


if __name__ == "__main__":
    main()
