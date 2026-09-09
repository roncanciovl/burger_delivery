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
Validación de seguridad previa al envío de una meta articular.

Toda la lógica vive aquí, separada del cliente de acción, por dos razones:

1. **Se puede probar sin robot.** ``colcon test`` ejercita los límites articulares, el
   delta máximo y el bloqueo por telemetría vencida sin levantar un grafo ROS 2.
2. **Se bloquea antes de tocar la red.** Ninguna meta rechazada llega a contactar el
   servidor de acción (prueba de aceptación PA-07).

Reglas implementadas (§11 de la especificación):

* ``enable_motion`` arranca siempre en ``false``; sin habilitación explícita no se envía
  nada al hardware real.
* Si el estado articular está vencido, incompleto o fuera de límites, la meta se bloquea.
* Los límites articulares configurados nunca se amplían para "hacer pasar" una prueba.
* Tras una pérdida de comunicación el movimiento permanece deshabilitado hasta una nueva
  habilitación explícita.
"""

import ipaddress
import math
from typing import Dict, List, Optional, Sequence


class GoalValidation:
    """Resultado de validar una meta articular antes de enviarla."""

    def __init__(
        self,
        allowed: bool,
        blocks: Optional[List[str]] = None,
        warnings: Optional[List[str]] = None,
        deltas: Optional[Dict[str, float]] = None,
    ):
        """
        Construir el resultado.

        :param allowed: ``True`` si la meta puede enviarse.
        :param blocks: motivos que impiden el envío.
        :param warnings: observaciones que no bloquean.
        :param deltas: desplazamiento por articulación respecto al estado actual.
        """
        self.allowed = allowed
        self.blocks = blocks or []
        self.warnings = warnings or []
        self.deltas = deltas or {}

    @property
    def reason(self) -> str:
        """Motivos de bloqueo concatenados en una sola línea."""
        return '; '.join(self.blocks)

    def __repr__(self) -> str:
        """Representación compacta útil en los mensajes de las pruebas."""
        return f'GoalValidation(allowed={self.allowed}, blocks={self.blocks})'


def validate_safety_config(
    expected_joints: Sequence[str],
    safe_positions: Sequence[float],
    joint_min: Sequence[float],
    joint_max: Sequence[float],
    max_joint_delta_rad: float,
    trajectory_duration_s: float,
) -> List[str]:
    """
    Validar la configuración de seguridad al arrancar el nodo.

    Un valor ausente, inválido o fuera de rango debe impedir el movimiento y producir un
    mensaje de error claro, en lugar de descubrirse a mitad de una prueba física.

    :param expected_joints: articulaciones esperadas (define la longitud obligatoria).
    :param safe_positions: meta articular aprobada.
    :param joint_min: límite inferior por articulación.
    :param joint_max: límite superior por articulación.
    :param max_joint_delta_rad: desplazamiento máximo permitido por articulación.
    :param trajectory_duration_s: duración mínima de la trayectoria de prueba.
    :returns: lista de errores; vacía si la configuración es utilizable.
    """
    errors: List[str] = []
    n = len(expected_joints)
    if n == 0:
        return ['expected_joints está vacío: no hay articulaciones que validar']

    for name, values in (
        ('safe_joint_positions_rad', safe_positions),
        ('joint_min_rad', joint_min),
        ('joint_max_rad', joint_max),
    ):
        if len(values) != n:
            errors.append(
                f'{name} tiene {len(values)} elementos y se esperaban {n} '
                f'(uno por articulación de expected_joints)'
            )
            continue
        for index, value in enumerate(values):
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                errors.append(
                    f'{name}[{index}] no es un número finito: {value!r}'
                )

    if len(joint_min) == n and len(joint_max) == n:
        for index, (low, high) in enumerate(zip(joint_min, joint_max)):
            if low >= high:
                errors.append(
                    f'límite inválido en {expected_joints[index]}: '
                    f'joint_min_rad[{index}]={low} no es menor que joint_max_rad[{index}]={high}'
                )

    if len(safe_positions) == n and len(joint_min) == n and len(joint_max) == n:
        for index, target in enumerate(safe_positions):
            low, high = joint_min[index], joint_max[index]
            if low < high and not (low <= target <= high):
                errors.append(
                    f'la pose aprobada para {expected_joints[index]} '
                    f'({target:.4f} rad) queda fuera de [{low:.4f}, {high:.4f}]'
                )

    if not isinstance(max_joint_delta_rad, (int, float)) or max_joint_delta_rad <= 0.0:
        errors.append(
            f'max_joint_delta_rad debe ser positivo; se recibió {max_joint_delta_rad!r}'
        )
    if not isinstance(trajectory_duration_s, (int, float)) or trajectory_duration_s <= 0.0:
        errors.append(
            f'trajectory_duration_s debe ser positivo; se recibió {trajectory_duration_s!r}'
        )
    return errors


def validate_robot_ip(robot_ip: str, start_driver: bool, use_fake_hardware: bool) -> List[str]:
    """
    Validar la dirección del robot según el modo de arranque.

    Con ``start_driver:=true`` y ``use_fake_hardware:=false`` el valor ``0.0.0.0`` debe
    rechazarse y exigirse una IP real; en modo fake ``0.0.0.0`` es el valor seguro.

    :param robot_ip: dirección configurada.
    :param start_driver: si esta estación inicia el driver.
    :param use_fake_hardware: si se opera con hardware simulado.
    :returns: lista de errores; vacía si la dirección es aceptable en ese modo.
    """
    errors: List[str] = []
    if not start_driver:
        return errors
    if not isinstance(robot_ip, str) or not robot_ip.strip():
        return ['robot_ip está vacío y start_driver es true']
    candidate = robot_ip.strip()
    try:
        address = ipaddress.IPv4Address(candidate)
    except (ipaddress.AddressValueError, ValueError):
        return [f"robot_ip no es una dirección IPv4 válida: '{candidate}'"]
    if not use_fake_hardware:
        if address == ipaddress.IPv4Address('0.0.0.0'):
            errors.append(
                'robot_ip=0.0.0.0 no es válido con hardware real: indica la IP verificada '
                'del Kinova el día de la práctica (por ejemplo robot_ip:=192.168.1.10)'
            )
        elif address.is_loopback or address.is_multicast:
            errors.append(f"robot_ip='{candidate}' no puede alcanzar un robot físico")
    return errors


def validate_goal(
    expected_joints: Sequence[str],
    target_positions: Sequence[float],
    current_positions: Dict[str, float],
    joint_min: Sequence[float],
    joint_max: Sequence[float],
    max_joint_delta_rad: float,
    enable_motion: bool,
    use_fake_hardware: bool,
    telemetry_fresh: bool,
    duration_s: float,
    min_duration_s: float,
) -> GoalValidation:
    """
    Decidir si una meta articular puede enviarse al servidor de acción.

    Se acumulan **todos** los motivos de bloqueo en lugar de cortar en el primero: para
    la evidencia de laboratorio es más útil un informe completo que un rechazo parcial.

    :param expected_joints: articulaciones esperadas, en el orden de la configuración.
    :param target_positions: meta articular aprobada, en radianes.
    :param current_positions: última posición válida conocida por articulación.
    :param joint_min: límite inferior aprobado por articulación.
    :param joint_max: límite superior aprobado por articulación.
    :param max_joint_delta_rad: desplazamiento máximo permitido por articulación.
    :param enable_motion: habilitación explícita del operador.
    :param use_fake_hardware: ``True`` si se opera con hardware simulado.
    :param telemetry_fresh: ``True`` si ``/joint_states`` está dentro del *timeout*.
    :param duration_s: duración solicitada para la trayectoria.
    :param min_duration_s: duración mínima admitida por la configuración.
    :returns: el resultado de la validación.
    """
    blocks: List[str] = []
    warnings: List[str] = []
    deltas: Dict[str, float] = {}
    n = len(expected_joints)

    if len(target_positions) != n:
        blocks.append(
            f'la meta tiene {len(target_positions)} valores y se esperaban {n} '
            f'(pose incompleta)'
        )
    if len(joint_min) != n or len(joint_max) != n:
        blocks.append(
            f'los límites articulares no cubren las {n} articulaciones esperadas '
            f'(joint_min_rad={len(joint_min)}, joint_max_rad={len(joint_max)})'
        )

    if not telemetry_fresh:
        blocks.append(
            'el estado articular está vencido o ausente: no se puede construir una meta '
            'segura sin conocer la posición actual'
        )

    missing = [joint for joint in expected_joints if joint not in current_positions]
    if missing:
        blocks.append(
            f"faltan posiciones actuales de: {', '.join(missing)}"
        )

    if len(target_positions) == n and len(joint_min) == n and len(joint_max) == n:
        for index, joint in enumerate(expected_joints):
            target = float(target_positions[index])
            if not math.isfinite(target):
                blocks.append(f'la meta para {joint} no es un número finito: {target!r}')
                continue
            low, high = float(joint_min[index]), float(joint_max[index])
            if not (low <= target <= high):
                blocks.append(
                    f'{joint}: la meta {target:.4f} rad excede el límite aprobado '
                    f'[{low:.4f}, {high:.4f}]'
                )
            if joint in current_positions:
                current = float(current_positions[joint])
                delta = target - current
                deltas[joint] = delta
                if abs(delta) > max_joint_delta_rad:
                    blocks.append(
                        f'{joint}: desplazamiento {abs(delta):.4f} rad supera '
                        f'max_joint_delta_rad={max_joint_delta_rad:.4f} rad'
                    )
                if not (low <= current <= high):
                    warnings.append(
                        f'{joint}: la posición ACTUAL {current:.4f} rad ya está fuera de '
                        f'[{low:.4f}, {high:.4f}]; revisa los límites configurados'
                    )

    if duration_s < min_duration_s:
        blocks.append(
            f'la duración solicitada {duration_s:.2f} s es menor que la mínima aprobada '
            f'{min_duration_s:.2f} s: un tramo más corto implica mayor velocidad'
        )

    if not enable_motion:
        if use_fake_hardware:
            blocks.append(
                'movimiento deshabilitado (enable_motion=false). En modo fake habilítalo '
                'de forma explícita para completar la prueba'
            )
        else:
            blocks.append(
                'movimiento deshabilitado (enable_motion=false) sobre HARDWARE REAL: '
                'requiere autorización del responsable del laboratorio, espacio despejado '
                'y parada de emergencia accesible'
            )

    return GoalValidation(allowed=not blocks, blocks=blocks, warnings=warnings, deltas=deltas)


def format_goal_report(
    expected_joints: Sequence[str],
    target_positions: Sequence[float],
    current_positions: Dict[str, float],
    validation: GoalValidation,
    duration_s: float,
) -> List[str]:
    """
    Formatear el resumen de la trayectoria que se muestra al operador.

    :param expected_joints: articulaciones esperadas.
    :param target_positions: meta articular solicitada.
    :param current_positions: posición actual conocida.
    :param validation: resultado de :func:`validate_goal`.
    :param duration_s: duración de la trayectoria.
    :returns: líneas del informe, listas para registrar o imprimir.
    """
    lines = ['── Resumen de la trayectoria de prueba ──',
             f'  Duración solicitada: {duration_s:.2f} s',
             '  articulación        actual [rad]     meta [rad]      Δ [rad]']
    for index, joint in enumerate(expected_joints):
        target = (
            float(target_positions[index]) if index < len(target_positions) else float('nan')
        )
        current = current_positions.get(joint, float('nan'))
        delta = validation.deltas.get(joint, target - current)
        lines.append(
            f'  {joint:<18} {current:>12.4f} {target:>14.4f} {delta:>+12.4f}'
        )
    for warning in validation.warnings:
        lines.append(f'  ⚠ {warning}')
    if validation.allowed:
        lines.append('  Estado: VALIDACIÓN SUPERADA — la meta puede enviarse.')
    else:
        lines.append('  Estado: META BLOQUEADA antes de contactar el servidor de acción:')
        for block in validation.blocks:
            lines.append(f'    ✗ {block}')
    return lines
