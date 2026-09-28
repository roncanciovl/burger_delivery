#!/usr/bin/env python3
"""
Opción A — Embodied AI SIN ROS 2: cámara del Kinova por RTSP + Gemini Robotics-ER 2.

Para quienes todavía no han visto ROS 2. Solo necesita Python, OpenCV con GStreamer,
numpy y el SDK google-genai. Cuatro subcomandos:

    # 0) Una vez por resolución: calibración rápida de la cámara de color
    #    (antes, capture la escena 'regla' con el subcomando 1)
    python3 embodied_sin_ros.py calibrar --escena regla --u1 700 --u2 1220 \\
        --ancho-real-m 0.297 --distancia-m 0.60

    # 1) En su turno frente al robot: capturar color + profundidad (unos segundos)
    python3 embodied_sin_ros.py capturar --ip 192.168.1.10 --escena escena01

    # 2) Donde quiera (no necesita el robot): preguntar a Gemini y localizar en 3D
    python3 embodied_sin_ros.py consultar --escena escena01 --tarea puntos \\
        --objeto "la caja de hamburguesa" --thinking low

    # 3) Detección de éxito con dos capturas (antes / después)
    python3 embodied_sin_ros.py exito --antes escena01 --despues escena02 \\
        --objeto "la caja quedó dentro de la bandeja"

Sin clave de API o sin cuota puede probar toda la geometría pasando la respuesta
a mano:  --respuesta '[{"point": [500, 500], "label": "centro"}]'
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import embodied_comun as ec

AQUI = Path(__file__).resolve().parent
REPO = AQUI.parents[2]
# Los pipelines GStreamer se reutilizan del visor validado en el Laboratorio 02.
sys.path.insert(0, str(REPO / "scripts"))

CARPETA_CAPTURAS = AQUI / "capturas"
CSV_RESULTADOS = AQUI / "resultados" / "resultados.csv"


# ---------------------------------------------------------------------------
# Captura
# ---------------------------------------------------------------------------
def abrir_stream(ip: str, tipo: str, latencia_ms: int):
    from test_kinova_camera import build_pipeline  # scripts/test_kinova_camera.py

    cap = cv2.VideoCapture(build_pipeline(f"rtsp://{ip}/{tipo}", tipo, latencia_ms), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError(
            f"No se abrió rtsp://{ip}/{tipo}. Verifique primero con "
            f"'python3 scripts/test_kinova_camera.py --ip {ip} --stream {tipo}' (Laboratorio 02)."
        )
    return cap


def leer_ultimo(cap, descartar: int = 10):
    """Descartar cuadros viejos del búfer y devolver el más reciente."""
    frame = None
    for _ in range(descartar):
        ok, f = cap.read()
        if ok:
            frame = f
    if frame is None:
        raise RuntimeError("El stream abrió pero no entregó cuadros")
    return frame, time.time()


def cmd_capturar(args) -> None:
    CARPETA_CAPTURAS.mkdir(parents=True, exist_ok=True)
    base = CARPETA_CAPTURAS / args.escena
    if base.with_name(base.name + "_color.png").exists() and not args.sobrescribir:
        sys.exit(f"[ERROR] {base}_color.png ya existe. Use otro --escena o --sobrescribir.")

    cap_c = abrir_stream(args.ip, "color", args.latencia)
    cap_d = abrir_stream(args.ip, "depth", args.latencia) if not args.solo_color else None
    try:
        color, t_c = leer_ultimo(cap_c)
        depth, t_d = leer_ultimo(cap_d) if cap_d else (None, None)
        # Volver a leer el color justo después para acercar las dos estampas de tiempo.
        color, t_c = leer_ultimo(cap_c, descartar=2)
    finally:
        cap_c.release()
        if cap_d:
            cap_d.release()

    cv2.imwrite(str(base) + "_color.png", color)
    meta = {"escena": args.escena, "ip": args.ip, "fecha": datetime.now().isoformat(timespec="seconds"),
            "color": {"ancho": color.shape[1], "alto": color.shape[0]}}
    if depth is not None:
        depth = depth if depth.ndim == 2 else depth[:, :, 0]
        cv2.imwrite(str(base) + "_depth.png", depth.astype(np.uint16))
        validos = depth[depth > 0]
        meta["profundidad"] = {"ancho": depth.shape[1], "alto": depth.shape[0], "unidad": "mm",
                               "pixeles_validos_pct": round(100 * validos.size / depth.size, 1),
                               "mediana_mm": int(np.median(validos)) if validos.size else None}
        meta["desfase_color_profundidad_ms"] = round(abs(t_c - t_d) * 1000, 1)
    if args.nota:
        meta["nota"] = args.nota
    Path(str(base) + "_meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))
    print(json.dumps(meta, indent=2, ensure_ascii=False))
    print(f"[OK] Escena guardada en {base}_*.png")


# ---------------------------------------------------------------------------
# Consulta
# ---------------------------------------------------------------------------
def cargar_escena(nombre: str):
    base = Path(nombre)
    if not base.name.endswith("_color.png") and not base.exists():
        base = CARPETA_CAPTURAS / nombre
    ruta_color = Path(str(base).removesuffix("_color.png") + "_color.png")
    ruta_depth = Path(str(base).removesuffix("_color.png") + "_depth.png")
    color = cv2.imread(str(ruta_color), cv2.IMREAD_COLOR)
    if color is None:
        sys.exit(f"[ERROR] No existe {ruta_color}")
    depth = cv2.imread(str(ruta_depth), cv2.IMREAD_UNCHANGED) if ruta_depth.exists() else None
    if depth is not None and depth.dtype != np.uint16:
        sys.exit(f"[ERROR] {ruta_depth} no es de 16 bits: se guardó como imagen coloreada, no métrica.")
    return ruta_color, color, depth


def obtener_respuesta(args, imagenes, prompt) -> dict:
    if args.respuesta:
        return {"texto": args.respuesta, "latencia_s": 0.0}
    cliente = ec.crear_cliente()
    return ec.consultar_gemini(cliente, imagenes, prompt, modelo=args.modelo,
                               thinking=args.thinking, temperatura=args.temperatura)


def cmd_consultar(args) -> None:
    ruta_color, color, depth = cargar_escena(args.escena)
    cfg = ec.cargar_config(args.config, color.shape[1], color.shape[0])
    for aviso in ec.advertencias_intrinsecos(cfg.color):
        print(f"[AVISO] Intrínsecos de color sospechosos: {aviso}")
    if depth is None:
        print("[AVISO] La escena no tiene profundidad: solo se reportarán píxeles.")

    prompt = ec.construir_prompt(args.tarea, args.objeto)
    print(f"[INFO] Prompt:\n{prompt}\n")
    r = obtener_respuesta(args, [color], prompt)
    print(f"[INFO] Respuesta en {r['latencia_s']} s:\n{r['texto']}\n")

    try:
        datos = ec.extraer_json(r["texto"])
    except json.JSONDecodeError as e:
        sys.exit(f"[ERROR] La respuesta no es JSON válido ({e}). Ajuste el prompt.")
    if args.tarea == "plan" and isinstance(datos, dict):
        print("[INFO] Pasos propuestos (NO se ejecutan):")
        for i, paso in enumerate(datos.get("pasos", []), 1):
            print(f"   {i}. {paso}")
        datos = datos.get("objetos", [])

    detecciones = ec.detecciones_desde_respuesta(datos)
    localizados = ec.localizar(detecciones, color, depth, cfg, registrar=not args.sin_registro)

    for det in localizados:
        print(f"  - {det['label']}: píxel=({det['u']}, {det['v']})  xyz={det.get('xyz_m')}  "
              f"rango={det.get('rango_m')} m")
    if len(localizados) >= 2:
        d = ec.distancia_entre(localizados[0], localizados[1])
        if d is not None:
            print(f"[INFO] Distancia 3D '{localizados[0]['label']}' <-> '{localizados[1]['label']}': {d:.3f} m")

    salida = ruta_color.with_name(ruta_color.name.replace("_color.png", f"_{args.tarea}_resultado.png"))
    cv2.imwrite(str(salida), ec.dibujar(color, localizados))
    print(f"[OK] Imagen anotada: {salida}")

    comunes = {
        "fecha": datetime.now().isoformat(timespec="seconds"), "grupo": args.grupo, "opcion": "A-sin-ROS",
        "escena": ruta_color.name.removesuffix("_color.png"), "tarea": args.tarea, "objeto": args.objeto,
        "modelo": "manual" if args.respuesta else args.modelo, "thinking": args.thinking,
        "registro": "no" if args.sin_registro else "si", "latencia_s": r["latencia_s"],
        "tokens_entrada": r.get("tokens_entrada"), "tokens_salida": r.get("tokens_salida"),
        "tokens_pensamiento": r.get("tokens_pensamiento"), "respuesta": r["texto"].replace("\n", " "),
    }
    for det in localizados or [{}]:
        xyz = det.get("xyz_m") or [None, None, None]
        ec.guardar_fila_csv(args.csv, {**comunes, "label": det.get("label"), "punto_yx": det.get("point"),
                                       "u": det.get("u"), "v": det.get("v"), "z_m": det.get("z_m"),
                                       "x_m": xyz[0], "y_m": xyz[1], "rango_m": det.get("rango_m")})
    print(f"[OK] Fila(s) agregada(s) a {args.csv}")


def cmd_exito(args) -> None:
    _, antes, _ = cargar_escena(args.antes)
    _, despues, _ = cargar_escena(args.despues)
    prompt = ec.construir_prompt("exito", args.objeto)
    r = obtener_respuesta(args, [antes, despues], prompt)
    print(f"[INFO] Respuesta en {r['latencia_s']} s:\n{r['texto']}")
    ec.guardar_fila_csv(args.csv, {
        "fecha": datetime.now().isoformat(timespec="seconds"), "grupo": args.grupo, "opcion": "A-sin-ROS",
        "escena": f"{args.antes}->{args.despues}", "tarea": "exito", "objeto": args.objeto,
        "modelo": "manual" if args.respuesta else args.modelo, "thinking": args.thinking,
        "latencia_s": r["latencia_s"], "tokens_entrada": r.get("tokens_entrada"),
        "tokens_salida": r.get("tokens_salida"), "tokens_pensamiento": r.get("tokens_pensamiento"),
        "respuesta": r["texto"].replace("\n", " "),
    })


def cmd_calibrar(args) -> None:
    """Calibración rápida: un objeto de ancho conocido, paralelo a la imagen y centrado.

    Por semejanza de triángulos  fx = ancho_en_pixeles * Z / ancho_real.
    Se asume fy = fx y el punto principal en el centro de la imagen: es una
    aproximación (error típico de unos pocos %), suficiente para esta práctica.
    """
    _, color, _ = cargar_escena(args.escena)
    alto, ancho = color.shape[:2]
    fx = abs(args.u2 - args.u1) * args.distancia_m / args.ancho_real_m
    entrada = {"fx": round(fx, 2), "fy": round(fx, 2), "cx": ancho / 2, "cy": alto / 2,
               "ancho": ancho, "alto": alto,
               "fuente": f"calibración rápida ({args.escena}: {args.ancho_real_m} m a {args.distancia_m} m)"}
    ruta = Path(args.config)
    datos = json.loads(ruta.read_text(encoding="utf-8"))
    datos["color"][f"{ancho}x{alto}"] = entrada
    ruta.write_text(json.dumps(datos, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    fov = 2 * np.degrees(np.arctan(ancho / (2 * fx)))
    print(json.dumps(entrada, indent=2, ensure_ascii=False))
    print(f"[OK] Campo de visión horizontal resultante: {fov:.1f}°. Guardado en {ruta}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("capturar", help="Guardar una captura color + profundidad del Kinova")
    c.add_argument("--ip", default="192.168.1.10")
    c.add_argument("--escena", required=True, help="Nombre de la escena, p. ej. escena01")
    c.add_argument("--latencia", type=int, default=30, help="Latencia RTSP en ms")
    c.add_argument("--solo-color", action="store_true")
    c.add_argument("--nota", default="", help="Descripción breve de la escena")
    c.add_argument("--sobrescribir", action="store_true")
    c.set_defaults(func=cmd_capturar)

    def comunes(sp):
        sp.add_argument("--objeto", required=True, help="Objeto o instrucción en lenguaje natural")
        sp.add_argument("--modelo", default=ec.MODELO_POR_DEFECTO)
        sp.add_argument("--thinking", default="low", choices=ec.NIVELES_THINKING)
        sp.add_argument("--temperatura", type=float, default=0.5)
        sp.add_argument("--respuesta", default="", help="JSON a mano en lugar de llamar a la API")
        sp.add_argument("--grupo", default="G00")
        sp.add_argument("--csv", default=str(CSV_RESULTADOS))

    q = sub.add_parser("consultar", help="Preguntar a Gemini ER 2 y localizar en 3D")
    q.add_argument("--escena", required=True, help="Nombre en capturas/ o ruta a *_color.png")
    q.add_argument("--tarea", default="puntos", choices=list(ec.PROMPTS))
    q.add_argument("--config", default=str(ec.CONFIG_POR_DEFECTO))
    q.add_argument("--sin-registro", action="store_true",
                   help="Ablación: usar la profundidad sin registrarla al marco de color")
    comunes(q)
    q.set_defaults(func=cmd_consultar)

    e = sub.add_parser("exito", help="Detección de éxito con dos capturas")
    e.add_argument("--antes", required=True)
    e.add_argument("--despues", required=True)
    comunes(e)
    e.set_defaults(func=cmd_exito)

    k = sub.add_parser("calibrar", help="Calibración rápida de fx con un objeto de ancho conocido")
    k.add_argument("--escena", required=True)
    k.add_argument("--u1", type=float, required=True, help="Columna (píxel) del borde izquierdo")
    k.add_argument("--u2", type=float, required=True, help="Columna (píxel) del borde derecho")
    k.add_argument("--ancho-real-m", type=float, required=True)
    k.add_argument("--distancia-m", type=float, required=True, help="Distancia cámara-objeto medida")
    k.add_argument("--config", default=str(ec.CONFIG_POR_DEFECTO))
    k.set_defaults(func=cmd_calibrar)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
