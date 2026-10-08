#!/usr/bin/env python3
"""
Núcleo común del laboratorio de Embodied AI con Gemini Robotics-ER 2 y Kinova Gen3.

Lo usan las dos rutas de la práctica:

* ``embodied_sin_ros.py``   -> Opción A: RTSP directo + OpenCV, sin ROS 2.
* ``embodied_ros2_nodo.py`` -> Opción B: nodo ROS 2 que consume el driver kinova_vision.

La cadena completa, de la foto al punto 3D, es:

    imagen color ──► Gemini ER 2 ──► [y, x] normalizado 0-1000 ──► píxel (u, v)
                                                                     │
    imagen profundidad (mm) ──► registro al marco de color ─────────►│
                                                                     ▼
                                   profundidad robusta z ──► desproyección ──► (X, Y, Z) en metros

Las funciones marcadas con TODO son las que usted debe implementar. Las demás ya
funcionan y conviene leerlas: la guía hace preguntas sobre ellas.
"""

from __future__ import annotations

import csv
import json
import os
import re
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

MODELO_POR_DEFECTO = "gemini-robotics-er-2-preview"
NIVELES_THINKING = ("minimal", "low", "medium", "high")
AQUI = Path(__file__).resolve().parent
CONFIG_POR_DEFECTO = AQUI / "config_camara_kinova.json"


# ---------------------------------------------------------------------------
# 1. Calibración de la cámara
# ---------------------------------------------------------------------------
@dataclass
class Intrinsecos:
    """Modelo pinhole de un sensor: focales y punto principal en píxeles."""

    fx: float
    fy: float
    cx: float
    cy: float
    ancho: int
    alto: int
    fuente: str = "desconocida"

    def matriz(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]])

    def escalar_a(self, ancho: int, alto: int) -> "Intrinsecos":
        """Reescalar a otra resolución CON LA MISMA relación de aspecto."""
        if (ancho, alto) == (self.ancho, self.alto):
            return self
        if abs(ancho / alto - self.ancho / self.alto) > 1e-3:
            raise ValueError(
                f"La calibración es de {self.ancho}x{self.alto} y la imagen de {ancho}x{alto}: "
                "cambió la relación de aspecto, no basta con escalar. Obtenga los intrínsecos "
                "de esa resolución (leer_calibracion_kortex.py o calibración rápida de la guía)."
            )
        s = ancho / self.ancho
        return Intrinsecos(self.fx * s, self.fy * s, self.cx * s, self.cy * s,
                           ancho, alto, self.fuente + f" (escalada x{s:.3f})")


def advertencias_intrinsecos(k: Intrinsecos) -> list[str]:
    """Pruebas de cordura baratas que detectan calibraciones absurdas."""
    avisos = []
    if not (0.3 * k.ancho < k.cx < 0.7 * k.ancho):
        avisos.append(f"cx={k.cx:.1f} está lejos del centro ({k.ancho / 2:.0f}).")
    if not (0.3 * k.alto < k.cy < 0.7 * k.alto):
        avisos.append(f"cy={k.cy:.1f} está lejos del centro ({k.alto / 2:.0f}).")
    if k.fx > 0 and abs(k.fx - k.fy) / k.fx > 0.05:
        avisos.append(f"fx={k.fx:.1f} y fy={k.fy:.1f} difieren más de 5 %.")
    return avisos


@dataclass
class ConfigCamara:
    color: Intrinsecos
    profundidad: Intrinsecos
    rotacion_depth_a_color: np.ndarray   # 3x3
    traslacion_depth_a_color: np.ndarray  # metros, 3


def _intrinsecos_desde_dict(d: dict, fuente: str) -> Intrinsecos:
    return Intrinsecos(float(d["fx"]), float(d["fy"]), float(d["cx"]), float(d["cy"]),
                       int(d["ancho"]), int(d["alto"]), d.get("fuente", fuente))


def cargar_config(ruta: str | Path | None, ancho_color: int | None = None,
                  alto_color: int | None = None) -> ConfigCamara:
    """Leer config_camara_kinova.json y elegir los intrínsecos de color de la resolución pedida."""
    ruta = Path(ruta) if ruta else CONFIG_POR_DEFECTO
    datos = json.loads(ruta.read_text(encoding="utf-8"))

    colores = datos["color"]
    clave = f"{ancho_color}x{alto_color}" if ancho_color else next(iter(colores))
    if clave in colores:
        color = _intrinsecos_desde_dict(colores[clave], str(ruta))
    else:
        # Intentar escalar desde una calibración con la misma relación de aspecto.
        color = None
        for d in colores.values():
            candidato = _intrinsecos_desde_dict(d, str(ruta))
            try:
                color = candidato.escalar_a(ancho_color, alto_color)
                break
            except ValueError:
                continue
        if color is None:
            raise KeyError(
                f"{ruta} no tiene intrínsecos de color para {clave} ni una resolución con la "
                f"misma relación de aspecto. Disponibles: {', '.join(colores)}."
            )

    ext = datos["extrinsecos_depth_a_color"]
    return ConfigCamara(
        color=color,
        profundidad=_intrinsecos_desde_dict(datos["profundidad"], str(ruta)),
        rotacion_depth_a_color=np.array(ext["rotacion"], dtype=float).reshape(3, 3),
        traslacion_depth_a_color=np.array(ext["traslacion_m"], dtype=float).reshape(3),
    )


# ---------------------------------------------------------------------------
# 2. Del formato de Gemini a píxeles
# ---------------------------------------------------------------------------
def extraer_json(texto: str):
    """Gemini a veces envuelve el JSON en ```json ... ```; esta función lo limpia."""
    limpio = texto.strip()
    bloque = re.search(r"```(?:json)?\s*(.*?)```", limpio, flags=re.DOTALL)
    if bloque:
        limpio = bloque.group(1).strip()
    return json.loads(limpio)


def normalizado_a_pixel(punto_yx, ancho: int, alto: int) -> tuple[float, float]:
    """Convertir un punto de Gemini ``[y, x]`` (0-1000) a píxeles ``(u, v)``.

    OJO con el orden: Gemini entrega primero la fila (y) y después la columna (x).
    En geometría de cámara ``u`` es la columna y ``v`` la fila.
    """
    # TODO 2: implemente la conversión y valide que el punto esté dentro de 0-1000.
    raise NotImplementedError("TODO 2: complete esta función (ver la guía)")


def caja_a_pixeles(caja_yxyx, ancho: int, alto: int) -> tuple[float, float, float, float]:
    """``box_2d = [ymin, xmin, ymax, xmax]`` (0-1000) -> ``(u_min, v_min, u_max, v_max)``."""
    u0, v0 = normalizado_a_pixel((caja_yxyx[0], caja_yxyx[1]), ancho, alto)
    u1, v1 = normalizado_a_pixel((caja_yxyx[2], caja_yxyx[3]), ancho, alto)
    return u0, v0, u1, v1


def detecciones_desde_respuesta(datos) -> list[dict]:
    """Normalizar la respuesta a una lista de ``{"label", "point"}``.

    Acepta puntos (``point``) o cajas (``box_2d``); de una caja toma el centro.
    """
    if isinstance(datos, dict):
        datos = [datos]
    salida = []
    for item in datos:
        if not isinstance(item, dict):
            continue
        etiqueta = str(item.get("label", "objeto"))
        if "point" in item:
            salida.append({"label": etiqueta, "point": list(item["point"])})
        elif "box_2d" in item:
            y0, x0, y1, x1 = item["box_2d"]
            salida.append({"label": etiqueta, "point": [(y0 + y1) / 2, (x0 + x1) / 2],
                           "box_2d": list(item["box_2d"])})
    return salida


# ---------------------------------------------------------------------------
# 3. Profundidad: registro, lectura robusta y desproyección
# ---------------------------------------------------------------------------
def registrar_profundidad(depth_mm: np.ndarray, cfg: ConfigCamara,
                          ancho_color: int, alto_color: int) -> np.ndarray:
    """Reproyectar la imagen de profundidad al marco de la cámara de color.

    El Kinova tiene DOS sensores: el de color (OV5640) y el de profundidad
    (Intel RealSense D410), separados unos milímetros y con resoluciones y
    campos de visión distintos. El píxel (u, v) de color NO es el mismo
    píxel en la imagen de profundidad. Esta función hace lo mismo que
    ``depth_image_proc::RegisterNode`` en ROS 2:

        1. cada píxel de profundidad -> punto 3D en el marco de profundidad,
        2. punto 3D -> marco de color con [R | t],
        3. punto 3D -> píxel de color con los intrínsecos de color.

    Devuelve una imagen del tamaño de la de color, en milímetros (0 = sin dato).
    Como la profundidad tiene menos resolución, quedan huecos: por eso después
    se usa una ventana robusta en lugar de un único píxel.
    """
    kd, kc = cfg.profundidad, cfg.color
    if depth_mm.shape[1] != kd.ancho or depth_mm.shape[0] != kd.alto:
        kd = kd.escalar_a(depth_mm.shape[1], depth_mm.shape[0])
    if (kc.ancho, kc.alto) != (ancho_color, alto_color):
        kc = kc.escalar_a(ancho_color, alto_color)

    v_d, u_d = np.nonzero(depth_mm)
    z = depth_mm[v_d, u_d].astype(np.float64) / 1000.0
    puntos_d = np.stack([(u_d - kd.cx) * z / kd.fx, (v_d - kd.cy) * z / kd.fy, z])
    puntos_c = cfg.rotacion_depth_a_color @ puntos_d + cfg.traslacion_depth_a_color[:, None]

    delante = puntos_c[2] > 1e-3
    puntos_c = puntos_c[:, delante]
    u_c = np.round(kc.fx * puntos_c[0] / puntos_c[2] + kc.cx).astype(np.int64)
    v_c = np.round(kc.fy * puntos_c[1] / puntos_c[2] + kc.cy).astype(np.int64)
    dentro = (u_c >= 0) & (u_c < ancho_color) & (v_c >= 0) & (v_c < alto_color)

    registrada = np.full((alto_color, ancho_color), np.iinfo(np.uint16).max, dtype=np.uint16)
    z_mm = np.round(puntos_c[2, dentro] * 1000.0).astype(np.uint16)
    # Si dos puntos caen en el mismo píxel gana el más cercano (z-buffer).
    np.minimum.at(registrada, (v_c[dentro], u_c[dentro]), z_mm)
    registrada[registrada == np.iinfo(np.uint16).max] = 0
    return registrada


def profundidad_sin_registro(depth_mm: np.ndarray, u: float, v: float,
                             ancho_color: int, alto_color: int) -> np.ndarray:
    """Aproximación ingenua (para el experimento de ablación de la guía).

    Supone que ambos sensores ven exactamente lo mismo y solo reescala la imagen
    de profundidad al tamaño de la de color. Es rápido y ERRÓNEO en los bordes
    de los objetos: la guía le pide medir cuánto.
    """
    del u, v
    return cv2.resize(depth_mm, (ancho_color, alto_color), interpolation=cv2.INTER_NEAREST)


def profundidad_robusta(depth_mm: np.ndarray, u: float, v: float,
                        ventana: int | None = None) -> float | None:
    """Profundidad en METROS alrededor de (u, v), o ``None`` si no hay datos válidos.

    Use una ventana cuadrada centrada en (u, v), descarte los ceros (sin dato) y
    devuelva la MEDIANA. ¿Por qué la mediana y no el promedio? (pregunta de la guía).
    """
    if ventana is None:
        ventana = max(5, int(0.012 * depth_mm.shape[1]) | 1)
    # TODO 3: implemente la lectura robusta.
    raise NotImplementedError("TODO 3: complete esta función (ver la guía)")


def desproyectar(u: float, v: float, z_m: float, k: Intrinsecos) -> tuple[float, float, float]:
    """Píxel (u, v) + profundidad z (m) -> punto (X, Y, Z) en el marco óptico de color.

    Convención óptica: X a la derecha, Y hacia abajo, Z hacia delante.
    """
    # TODO 4: implemente el modelo pinhole inverso.
    raise NotImplementedError("TODO 4: complete esta función (ver la guía)")


def localizar(detecciones: list[dict], color_bgr: np.ndarray, depth_mm: np.ndarray | None,
              cfg: ConfigCamara, registrar: bool = True) -> list[dict]:
    """Completar cada detección con píxel, profundidad y punto 3D."""
    alto, ancho = color_bgr.shape[:2]
    depth_color = None
    if depth_mm is not None:
        depth_color = (registrar_profundidad(depth_mm, cfg, ancho, alto) if registrar
                       else profundidad_sin_registro(depth_mm, 0, 0, ancho, alto))
    k = cfg.color if (cfg.color.ancho, cfg.color.alto) == (ancho, alto) else cfg.color.escalar_a(ancho, alto)
    resultado = []
    for det in detecciones:
        u, v = normalizado_a_pixel(det["point"], ancho, alto)
        item = dict(det, u=round(u, 1), v=round(v, 1))
        if depth_color is not None:
            z = profundidad_robusta(depth_color, u, v)
            item["z_m"] = z
            if z is not None:
                x, y, z = desproyectar(u, v, z, k)
                item["xyz_m"] = [round(x, 4), round(y, 4), round(z, 4)]
                item["rango_m"] = round(float(np.linalg.norm([x, y, z])), 4)
        resultado.append(item)
    return resultado


def distancia_entre(a: dict, b: dict) -> float | None:
    """Distancia 3D entre dos objetos localizados; no depende del marco de referencia."""
    if "xyz_m" not in a or "xyz_m" not in b:
        return None
    return float(np.linalg.norm(np.subtract(a["xyz_m"], b["xyz_m"])))


# ---------------------------------------------------------------------------
# 4. Prompts: aquí está buena parte de la "IA" de la práctica
# ---------------------------------------------------------------------------
FORMATO_PUNTOS = (
    'Responde SOLO con una lista JSON con el formato '
    '[{"point": [y, x], "label": "<nombre>"}]. '
    "Los puntos van en formato [y, x] normalizado a 0-1000."
)
FORMATO_CAJAS = (
    'Responde SOLO con una lista JSON con el formato '
    '[{"box_2d": [ymin, xmin, ymax, xmax], "label": "<nombre>"}]. '
    "Las coordenadas van normalizadas a 0-1000."
)

PROMPTS = {
    # Señalar un objeto nombrado en lenguaje natural.
    "puntos": "Señala {objeto} en la imagen. " + FORMATO_PUNTOS,
    # Cajas delimitadoras de todos los objetos de la escena.
    "cajas": "Detecta como máximo 10 objetos sobre la mesa. " + FORMATO_CAJAS,
    # Affordance: dónde agarrar, no solo dónde está.
    "agarre": (
        "Un brazo robótico con pinza paralela debe tomar {objeto} desde arriba. "
        "Señala el mejor punto de agarre sobre el objeto y, si el agarre NO es seguro "
        "(algo encima, objeto inestable, mano humana cerca), devuelve una lista vacía. "
        + FORMATO_PUNTOS
    ),
    # Planificación (orquestación): el plan se valida, NO se ejecuta.
    "plan": (
        "Eres el planificador de un brazo Kinova Gen3 con pinza. Objetivo: {objeto}. "
        "Devuelve SOLO JSON con el formato "
        '{"objetos": [{"point": [y, x], "label": "<nombre>"}], '
        '"pasos": [{"accion": "mover_sobre|bajar|cerrar_pinza|subir|abrir_pinza", '
        '"objetivo": "<label o null>"}]}. '
        "Los puntos van en [y, x] normalizado a 0-1000. Usa solo esas acciones."
    ),
    # Detección de éxito: se envían DOS imágenes (antes y después).
    "exito": (
        "La primera imagen es ANTES y la segunda DESPUÉS de una manipulación. "
        "Tarea: {objeto}. ¿Se cumplió la tarea? Responde SOLO JSON "
        '{"exito": true|false, "confianza": 0-1, "evidencia": "<frase corta>"}.'
    ),
    # TODO 1: escriba su propio prompt para la escena de su grupo (Fase 4).
    # Debe obligar a Gemini a razonar sobre relaciones espaciales o estado del
    # objeto, y seguir exigiendo el formato de puntos.
    "propio": (
        "ESCRIBA AQUÍ SU PROMPT (TODO 1). {objeto}. " + FORMATO_PUNTOS
    ),
}


def construir_prompt(tarea: str, objeto: str) -> str:
    """Insertar el objeto en la plantilla. Se usa replace() y no format() porque
    los prompts contienen llaves literales del JSON de salida."""
    if tarea not in PROMPTS:
        raise KeyError(f"Tarea desconocida '{tarea}'. Opciones: {', '.join(PROMPTS)}")
    return PROMPTS[tarea].replace("{objeto}", objeto)


# ---------------------------------------------------------------------------
# 5. Llamada a Gemini Robotics-ER 2
# ---------------------------------------------------------------------------
def preparar_imagen(bgr: np.ndarray, lado_max: int = 1280, calidad: int = 90) -> bytes:
    """Reducir y codificar a JPEG. Las coordenadas son normalizadas, así que
    reducir la imagen NO cambia el píxel final en la imagen original."""
    alto, ancho = bgr.shape[:2]
    escala = min(1.0, lado_max / max(alto, ancho))
    if escala < 1.0:
        bgr = cv2.resize(bgr, (int(ancho * escala), int(alto * escala)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, calidad])
    if not ok:
        raise RuntimeError("No se pudo codificar la imagen a JPEG")
    return buf.tobytes()


def crear_cliente():
    """El SDK lee la clave de GEMINI_API_KEY (o GOOGLE_API_KEY). Nunca la escriba en el código."""
    if not (os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")):
        raise RuntimeError(
            "Falta la variable de entorno GEMINI_API_KEY. Genere la clave en "
            "https://aistudio.google.com/ y ejecútela así: export GEMINI_API_KEY=\"...\""
        )
    from google import genai  # importación diferida: el resto funciona sin el SDK
    return genai.Client()


def consultar_gemini(cliente, imagenes_bgr: list[np.ndarray], prompt: str,
                     modelo: str = MODELO_POR_DEFECTO, thinking: str = "low",
                     temperatura: float = 0.5, timeout_s: float = 60.0) -> dict:
    """Enviar una o varias imágenes + prompt. Devuelve texto, latencia y tokens."""
    from google.genai import types

    partes = [types.Part.from_bytes(data=preparar_imagen(img), mime_type="image/jpeg")
              for img in imagenes_bgr]
    config = types.GenerateContentConfig(
        temperature=temperatura,
        thinking_config=types.ThinkingConfig(thinking_level=thinking),
        http_options=types.HttpOptions(timeout=int(timeout_s * 1000)),
    )
    t0 = time.perf_counter()
    respuesta = cliente.models.generate_content(model=modelo, contents=[*partes, prompt], config=config)
    latencia = time.perf_counter() - t0
    uso = getattr(respuesta, "usage_metadata", None)
    return {
        "texto": respuesta.text or "",
        "latencia_s": round(latencia, 3),
        "tokens_entrada": getattr(uso, "prompt_token_count", None),
        "tokens_salida": getattr(uso, "candidates_token_count", None),
        "tokens_pensamiento": getattr(uso, "thoughts_token_count", None),
    }


# ---------------------------------------------------------------------------
# 6. Evidencias: imagen anotada y CSV de resultados
# ---------------------------------------------------------------------------
def dibujar(color_bgr: np.ndarray, localizados: list[dict]) -> np.ndarray:
    salida = color_bgr.copy()
    grosor = max(2, salida.shape[1] // 640)
    for det in localizados:
        u, v = int(det["u"]), int(det["v"])
        if "box_2d" in det:
            u0, v0, u1, v1 = caja_a_pixeles(det["box_2d"], salida.shape[1], salida.shape[0])
            cv2.rectangle(salida, (int(u0), int(v0)), (int(u1), int(v1)), (255, 180, 0), grosor)
        cv2.drawMarker(salida, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 12 * grosor, grosor)
        texto = det["label"]
        if det.get("xyz_m"):
            texto += " ({:.3f}, {:.3f}, {:.3f}) m".format(*det["xyz_m"])
        elif "z_m" in det:
            texto += " (sin profundidad)"
        cv2.putText(salida, texto, (u + 8, max(20, v - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5 * grosor, (0, 255, 255), grosor)
    return salida


COLUMNAS_CSV = ["fecha", "grupo", "opcion", "escena", "tarea", "objeto", "modelo", "thinking",
                "registro", "latencia_s", "tokens_entrada", "tokens_salida", "tokens_pensamiento",
                "label", "punto_yx", "u", "v", "z_m", "x_m", "y_m", "rango_m", "respuesta"]


def guardar_fila_csv(ruta: str | Path, fila: dict) -> None:
    ruta = Path(ruta)
    ruta.parent.mkdir(parents=True, exist_ok=True)
    nuevo = not ruta.exists()
    with ruta.open("a", newline="", encoding="utf-8") as f:
        escritor = csv.DictWriter(f, fieldnames=COLUMNAS_CSV, extrasaction="ignore")
        if nuevo:
            escritor.writeheader()
        escritor.writerow(fila)
