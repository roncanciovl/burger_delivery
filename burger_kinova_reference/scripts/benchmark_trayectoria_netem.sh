#!/usr/bin/env bash
# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
#
# Una corrida del experimento de estabilidad de trayectorias bajo degradación de red
# (TODO.md §4, PI-1 de docs/research/EXPERIMENTO_QOS_TELEMETRIA.md).
#
# Supone que el driver YA corre en la estación anfitriona (TROUBLESHOOTING.md §2.0) y que
# esta estación es un cliente del mismo dominio. Por cada corrida:
#
#   1. registra las condiciones (entorno.txt) y el estado de netem (netem.txt);
#   2. aplica el perfil con network_setup/netem/perfil_netem.sh (pide sudo);
#   3. graba /joint_states, el estado del joint_trajectory_controller, /rosout y el
#      diagnóstico del monitor;
#   4. ejecuta safe_sequence_client (la coreografía de config/kinova_connection.yaml),
#      o sólo observa con --sin-movimiento;
#   5. retira netem SIEMPRE, también con Ctrl+C, y cierra la bolsa.
#
# Uso:
#   ./benchmark_trayectoria_netem.sh <etiqueta> <E1|E2|E3> <iface> [--destino IP]
#                                    [--sin-movimiento] [--segundos N]
#
#   ./benchmark_trayectoria_netem.sh cliente_E2 E2 wlan0 --destino 192.168.1.20
#
# Analiza con:  python3 scripts/analizar_trayectoria.py trayectoria_cliente_E1 trayectoria_cliente_E2 ...
#
# SEGURIDAD: con movimiento, el brazo ejecuta la secuencia validada de siempre (deltas de
# 0.09 rad, excursión máxima 0.60 rad) y el cliente pide confirmación por teclado. Ten la
# parada de emergencia a mano. La secuencia se detiene sola si el enlace se degrada lo
# bastante para que el monitor lo marque en ERROR: ese es también un resultado.

set -uo pipefail

ETIQUETA="${1:?Falta la etiqueta (ej. cliente_E2)}"
PERFIL="${2:?Falta el perfil (E1, E2 o E3)}"
IFACE="${3:?Falta la interfaz a degradar (ej. wlan0)}"
shift 3
DESTINO=""
MOVIMIENTO=1
SEGUNDOS=90
while [ $# -gt 0 ]; do
  case "$1" in
    --destino) DESTINO=$2; shift 2 ;;
    --sin-movimiento) MOVIMIENTO=0; shift ;;
    --segundos) SEGUNDOS=$2; shift 2 ;;
    *) echo "opción desconocida: $1" >&2; exit 1 ;;
  esac
done

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ERROR: carga primero el entorno ROS 2 y el workspace." >&2
  exit 1
fi
AQUI=$(cd "$(dirname "$0")" && pwd)
# El script de netem vive en el repositorio, no en install/: BURGER_REPO lo ubica si el
# repositorio no está en la ruta habitual del workspace.
NETEM=""
for repo in "${BURGER_REPO:-}" "$AQUI/../.." "$HOME/ros2_ws/src/burger_delivery"; do
  if [ -n "$repo" ] && [ -x "$repo/network_setup/netem/perfil_netem.sh" ]; then
    NETEM=$(readlink -f "$repo/network_setup/netem/perfil_netem.sh")
    break
  fi
done
[ -n "$NETEM" ] || { echo "ERROR: no encuentro perfil_netem.sh; exporta BURGER_REPO" >&2; exit 1; }
CONFIG=$(ros2 pkg prefix burger_kinova_reference)/share/burger_kinova_reference/config/kinova_connection.yaml

SALIDA="trayectoria_${ETIQUETA}"
mkdir -p "$SALIDA" && cd "$SALIDA" || exit 1

ARGS_NETEM=()
[ -n "$DESTINO" ] && ARGS_NETEM+=(--destino "$DESTINO" --ambos-sentidos)

limpiar() {
  [ -n "${BAG_PGID:-}" ] && kill -INT -- "-$BAG_PGID" 2>/dev/null
  for _ in $(seq 1 15); do [ -f bag/metadata.yaml ] && break; sleep 1; done
  sudo "$NETEM" quitar "$IFACE" >/dev/null 2>&1
  echo "netem retirado de $IFACE"
}
trap limpiar EXIT
trap 'exit 130' INT TERM

echo "=== 1/4 Condiciones ==="
{
  echo "etiqueta          : $ETIQUETA"
  echo "perfil            : $PERFIL"
  echo "interfaz          : $IFACE"
  echo "destino           : ${DESTINO:-(todo el tráfico)}"
  echo "movimiento        : $MOVIMIENTO"
  echo "fecha             : $(date -Is)"
  echo "host              : $(uname -srm)"
  echo "wsl               : ${WSL_DISTRO_NAME:-(no es WSL)}"
  echo "ROS_DOMAIN_ID     : ${ROS_DOMAIN_ID:-0 (por defecto)}"
  echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-(por defecto)}"
} | tee entorno.txt

echo "=== 2/4 Aplicando $PERFIL en $IFACE ==="
sudo "$NETEM" aplicar "$IFACE" "$PERFIL" "${ARGS_NETEM[@]}" --auto-quitar $((SEGUNDOS + 120)) \
  | tee netem.txt || exit 1
if [ -n "$DESTINO" ]; then
  ping -c 50 -i 0.2 -W 1 "$DESTINO" 2>&1 | tail -3 | tee -a netem.txt
fi

echo "=== 3/4 Grabando ==="
setsid ros2 bag record --storage mcap --output bag \
  --topics /joint_states /joint_trajectory_controller/controller_state \
  /burger/kinova/diagnostics /rosout > bag.log 2>&1 &
BAG_PGID=$(ps -o pgid= -p $! | tr -d ' ')
sleep 3

if [ "$MOVIMIENTO" -eq 1 ]; then
  echo "=== 4/4 Secuencia con movimiento (confirma por teclado) ==="
  ros2 run burger_kinova_reference safe_sequence_client --ros-args \
    --params-file "$CONFIG" -p dry_run:=false -p enable_motion:=true \
    -p use_fake_hardware:=false 2>&1 | tee secuencia.txt
  echo "codigo_salida: ${PIPESTATUS[0]}" >> secuencia.txt
else
  echo "=== 4/4 Observación sin movimiento durante ${SEGUNDOS} s ==="
  sleep "$SEGUNDOS"
fi
"$NETEM" estado "$IFACE" >> netem.txt 2>&1
echo "Artefactos en $(pwd)"
