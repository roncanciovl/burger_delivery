# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Metas del ejecutor seguro de MoveIt, leídas de un YAML (sin ROS).

Formato (``config/metas_moveit.yaml``)::

    metas:
      home:          {tipo: articular, posiciones: [0.0, 0.26, -2.27, 0.0, 0.96, 1.57]}
      sobre_tag:     {tipo: pose, marco: tag_mesa, posicion: [0.0, 0.0, 0.20],
                      rpy: [3.14159, 0.0, 0.0]}
      bajar_5cm:     {tipo: cartesiana, desplazamiento: [0.0, 0.0, -0.05]}
      cerrar_pinza:  {tipo: pinza, posicion: 0.8}
    secuencia: [home, sobre_tag, bajar_5cm, home]

* ``articular``: posiciones de las seis articulaciones, en radianes.
* ``pose``: posición y orientación (roll-pitch-yaw) del efector final **en un marco de la
  escena de burger_description** (``tag_mesa``, ``delivery_slot_1``...) o en el marco de
  planificación. Así las metas siguen a la celda: si se corrige la escena, se corrigen.
* ``cartesiana``: desplazamiento en línea recta del efector, en el marco de planificación,
  conservando su orientación.
* ``pinza``: apertura de la Robotiq 2F-85 (0 = abierta, 0.8 = cerrada).
"""

import math
from typing import Dict, List, Optional, Sequence

from burger_kinova_reference.escena_urdf import matmul, pose_matrix, SceneModel, to_pose

TIPOS = ('articular', 'pose', 'cartesiana', 'pinza')


class Meta:
    """Una meta del ejecutor, ya validada en su forma."""

    def __init__(self, nombre: str, tipo: str, **datos):
        """Guardar el nombre, el tipo y los datos propios del tipo."""
        self.nombre = nombre
        self.tipo = tipo
        self.posiciones: List[float] = datos.get('posiciones', [])
        self.marco: str = datos.get('marco', '')
        self.posicion: List[float] = datos.get('posicion', [])
        self.rpy: List[float] = datos.get('rpy', [])
        self.desplazamiento: List[float] = datos.get('desplazamiento', [])
        self.pinza: float = datos.get('pinza', 0.0)

    def __repr__(self) -> str:
        """Representación breve para los mensajes."""
        return f'Meta({self.nombre}, {self.tipo})'


def _vector(spec: Dict, key: str, n: int, nombre: str) -> List[float]:
    valor = spec.get(key)
    if not isinstance(valor, (list, tuple)) or len(valor) != n:
        raise ValueError(f'meta "{nombre}": "{key}" debe ser una lista de {n} números')
    try:
        numeros = [float(v) for v in valor]
    except (TypeError, ValueError) as error:
        raise ValueError(f'meta "{nombre}": "{key}" tiene valores no numéricos') from error
    if not all(math.isfinite(v) for v in numeros):
        raise ValueError(f'meta "{nombre}": "{key}" tiene valores no finitos')
    return numeros


def parse_metas(data: Dict, n_joints: int = 6) -> Dict[str, Meta]:
    """
    Validar la sección ``metas`` del YAML.

    :raises ValueError: con el nombre de la meta y el campo que falla.
    """
    metas = {}
    for nombre, spec in ((data or {}).get('metas') or {}).items():
        if not isinstance(spec, dict) or spec.get('tipo') not in TIPOS:
            raise ValueError(f'meta "{nombre}": "tipo" debe ser uno de {TIPOS}')
        tipo = spec['tipo']
        if tipo == 'articular':
            metas[nombre] = Meta(nombre, tipo,
                                 posiciones=_vector(spec, 'posiciones', n_joints, nombre))
        elif tipo == 'pose':
            metas[nombre] = Meta(nombre, tipo, marco=str(spec.get('marco', '')),
                                 posicion=_vector(spec, 'posicion', 3, nombre),
                                 rpy=_vector(spec, 'rpy', 3, nombre))
        elif tipo == 'cartesiana':
            desplazamiento = _vector(spec, 'desplazamiento', 3, nombre)
            if math.hypot(*desplazamiento) > 0.30:
                raise ValueError(f'meta "{nombre}": un tramo cartesiano de más de 0.30 m no '
                                 'es un ajuste fino; usa una meta de tipo pose')
            metas[nombre] = Meta(nombre, tipo, desplazamiento=desplazamiento)
        else:
            try:
                pinza = float(spec.get('posicion'))
            except (TypeError, ValueError) as error:
                raise ValueError(f'meta "{nombre}": "posicion" de la pinza debe ser un '
                                 'número') from error
            metas[nombre] = Meta(nombre, tipo, pinza=pinza)
    return metas


def parse_secuencia(data: Dict, metas: Dict[str, Meta],
                    pedidas: Optional[Sequence[str]] = None) -> List[Meta]:
    """
    Elegir la secuencia a ejecutar: ``pedidas`` si se da, si no la del YAML.

    :raises ValueError: si se nombra una meta inexistente o la secuencia queda vacía.
    """
    nombres = list(pedidas) if pedidas else list((data or {}).get('secuencia') or [])
    desconocidas = [n for n in nombres if n not in metas]
    if desconocidas:
        raise ValueError(f'metas desconocidas en la secuencia: {desconocidas}; '
                         f'disponibles: {sorted(metas)}')
    if not nombres:
        raise ValueError('la secuencia está vacía')
    return [metas[n] for n in nombres]


def pose_en_marco_de_planificacion(meta: Meta, modelo: Optional[SceneModel], anchor: str,
                                   planning_frame: str):
    """
    Calcular la pose objetivo del efector en el marco de planificación.

    Devuelve ``((x, y, z), (qx, qy, qz, qw))``.

    Si ``meta.marco`` es el marco de planificación (o está vacío), la pose se toma tal cual.
    Si es un marco de la escena, se compone con la transformación ``anchor -> marco`` de
    burger_description, donde ``anchor`` (``world``) coincide con la base del brazo.

    :raises ValueError: si el marco no existe o no hay modelo de escena.
    """
    local = pose_matrix(meta.posicion, meta.rpy)
    if meta.marco in ('', planning_frame, anchor):
        return to_pose(local)
    if modelo is None:
        raise ValueError(f'meta "{meta.nombre}": el marco "{meta.marco}" requiere la escena '
                         'de burger_description (parámetro scene_urdf)')
    try:
        return to_pose(matmul(modelo.transform(anchor, meta.marco), local))
    except KeyError as error:
        raise ValueError(f'meta "{meta.nombre}": {error}') from error
