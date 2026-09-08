#!/usr/bin/env bash
# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
#
# Mide UNA rama de un experimento A/B sobre el enlace con el Kinova Gen3.
#
# Se ejecuta igual en cada configuración que se quiera comparar (WiFi contra cable,
# WSL2 contra Linux nativo, un RMW contra otro) y produce artefactos homogéneos, para
# que la comparación sea entre números y no entre impresiones.
#
# Uso:
#   ./benchmark_enlace_kinova.sh <etiqueta> [ip_robot] [segundos]
#
#   ./benchmark_enlace_kinova.sh wifi_wsl2      192.168.1.10 120
#   ./benchmark_enlace_kinova.sh ethernet_wsl2  192.168.1.10 120
#
# Produce en ./benchmark_<etiqueta>/:
#   entorno.txt   condiciones exactas de la corrida (SO, RMW, dominio, ruta al robot)
#   ping.txt      latencia al robot y al gateway, SIN sesión Kortex abierta
#   driver.txt    log completo del launch (overruns y timeouts de la API Kortex)
#   bag/          bolsa MCAP con /joint_states, /rosout y el diagnóstico
#
# SEGURIDAD: arranca con enable_motion:=false y sin pinza. No envía ninguna meta y el
# robot no se mueve. Aun así, sólo esta estación puede tener el driver: la sesión de
# control en tiempo real de la API Kortex es única.

set -uo pipefail

ETIQUETA="${1:?Falta la etiqueta de la corrida (ej. wifi_wsl2)}"
ROBOT_IP="${2:-192.168.1.10}"
SEGUNDOS="${3:-120}"
SALIDA="benchmark_${ETIQUETA}"

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ERROR: carga primero el entorno ROS 2 y el workspace." >&2
  exit 1
fi

mkdir -p "$SALIDA"
cd "$SALIDA" || exit 1

echo "=== 1/4 Registrando condiciones de la corrida ==="
{
  echo "etiqueta          : $ETIQUETA"
  echo "fecha             : $(date -Is)"
  echo "host              : $(uname -srm)"
  echo "wsl               : ${WSL_DISTRO_NAME:-(no es WSL)}"
  echo "ROS_DISTRO        : ${ROS_DISTRO}"
  echo "ROS_DOMAIN_ID     : ${ROS_DOMAIN_ID:-0 (por defecto)}"
  echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-(por defecto)}"
  echo "CYCLONEDDS_URI    : ${CYCLONEDDS_URI:-(no definido)}"
  echo "descubrimiento    : ${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET (por defecto)}"
  echo "robot_ip          : $ROBOT_IP"
  echo "duración          : ${SEGUNDOS} s"
  echo "--- ruta al robot ---"
  ip route get "$ROBOT_IP" 2>&1
  echo "--- interfaces ---"
  ip -4 addr show 2>/dev/null | grep -E "inet " | grep -v 127.0.0.1
} | tee entorno.txt

echo
echo "=== 2/4 Latencia del enlace (sin sesión Kortex abierta) ==="
GATEWAY=$(ip route | awk '/^default/{print $3; exit}')
{
  echo "### robot $ROBOT_IP (500 paquetes @ 20 ms)"
  ping -c 500 -i 0.02 -W 1 "$ROBOT_IP" 2>&1 | tail -3
  if [ -n "$GATEWAY" ]; then
    echo
    echo "### gateway $GATEWAY (500 paquetes @ 20 ms) — control del mismo primer salto"
    ping -c 500 -i 0.02 -W 1 "$GATEWAY" 2>&1 | tail -3
  fi
} | tee ping.txt

echo
echo "=== 3/4 Driver + monitor durante ${SEGUNDOS} s (SIN movimiento) ==="
# El launch arranca en su propio grupo de procesos (setsid) para poder señalizar a
# TODO el árbol —driver, spawners, monitor— y no dejar huérfanos.
setsid ros2 launch burger_kinova_connection kinova_connection.launch.py \
    start_driver:=true robot_ip:="$ROBOT_IP" use_fake_hardware:=false \
    enable_motion:=false gripper:=none launch_rviz:=false > driver.txt 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')

sleep 15   # margen para que el driver active los controladores

# Sin compresión a propósito: es una bolsa de análisis de dos minutos, y comprimir
# retrasa el cierre del archivo justo cuando se interrumpe la grabación.
#
# `setsid` es obligatorio: `ros2 bag record` sólo atiende SIGINT cuando lo recibe como
# grupo de procesos (que es lo que hace Ctrl+C en una terminal). Señalizar al PID suelto
# desde un script NO lo detiene —sigue grabando— y la bolsa acaba conteniendo también las
# corridas siguientes, con lo que la comparación A/B queda contaminada en silencio.
setsid ros2 bag record --storage mcap \
    --output bag /joint_states /burger/kinova/diagnostics /rosout > bag.log 2>&1 &
BAG_PID=$!
BAG_PGID=$(ps -o pgid= -p "$BAG_PID" 2>/dev/null | tr -d ' ')

sleep "$SEGUNDOS"

kill -INT -- "-${BAG_PGID:-$BAG_PID}" 2>/dev/null
for _ in $(seq 1 15); do
  [ -f bag/metadata.yaml ] && break
  sleep 1
done
if [ ! -f bag/metadata.yaml ]; then
  echo "  ⚠ el grabador no cerró la bolsa; se fuerza su terminación"
  kill -TERM -- "-${BAG_PGID:-$BAG_PID}" 2>/dev/null
  sleep 3
  kill -9 -- "-${BAG_PGID:-$BAG_PID}" 2>/dev/null
fi
# Verificación explícita: ningún grabador debe sobrevivir a esta rama del experimento.
if pgrep -f 'bag[ ]record' > /dev/null; then
  echo "  ⚠ ATENCIÓN: quedó un 'ros2 bag record' vivo. Mátalo antes de la siguiente"
  echo "    corrida o grabará también sobre esta bolsa y contaminará la comparación."
fi
# SIGINT, nunca SIGKILL: el driver debe cerrar la sesión Kortex de forma ordenada o el
# robot puede quedar reportando "Session already in use" en el siguiente arranque.
kill -INT -- "-${LAUNCH_PGID:-$LAUNCH_PID}" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null

# Se espera al cierre ordenado y se VERIFICA que el driver soltó el brazo. Matar en duro
# una sesión de la API Kortex deja al robot reportando "Session already in use" y el
# siguiente arranque falla, así que aquí sólo se recurre a SIGKILL como último extremo.
for _ in $(seq 1 30); do
  grep -q "successfully deactivated" driver.txt && break
  sleep 1
done

if grep -q "successfully deactivated" driver.txt; then
  echo "  ✓ sesión Kortex cerrada de forma ordenada"
else
  echo "  ⚠ el driver no reportó el cierre de la sesión Kortex."
  echo "    Si el próximo arranque falla con 'Session already in use', espera a que"
  echo "    expire la sesión en la controladora o reinicia el brazo."
fi

pkill -9 -g "${LAUNCH_PGID:-0}" 2>/dev/null

echo
echo "=== 4/4 Listo ==="
echo "Artefactos en $(pwd)"
echo "Analiza con:"
echo "    python3 <ruta>/scripts/analizar_enlace.py $(pwd)"
