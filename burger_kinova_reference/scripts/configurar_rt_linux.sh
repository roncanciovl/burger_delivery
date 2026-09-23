#!/usr/bin/env bash
# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
#
# Habilita la planificación de tiempo real (SCHED_FIFO) para el driver del Kinova en una
# anfitriona con Linux NATIVO, siguiendo la guía de ros2_control:
#   https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html
#
# Sin esto, ros2_control_node avisa al arrancar:
#   Could not enable FIFO RT scheduling policy: Operation not permitted
# y el lazo de control compite con cualquier otro proceso por la CPU.
#
# Qué hace (idempotente):
#   1. crea el grupo 'realtime' si no existe;
#   2. añade al usuario indicado (por defecto, quien invoca sudo);
#   3. escribe /etc/security/limits.d/99-ros2-realtime.conf con rtprio 99 y memlock
#      ilimitado para ese grupo.
#
# Hay que cerrar sesión y volver a entrar para que el límite aplique. Comprobación:
#   ulimit -r      # debe decir 99
#
# En WSL2 no sirve: el kernel de WSL2 no concede SCHED_FIFO a los procesos del invitado
# de forma fiable. Ese es precisamente el residuo que mide la rama ethernet_nativo_rt.
#
# Uso:  sudo ./configurar_rt_linux.sh [usuario]

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: ejecuta con sudo." >&2
  exit 1
fi
if grep -qi microsoft /proc/version 2>/dev/null; then
  echo "AVISO: esto es WSL2; la configuración se escribe, pero el kernel de WSL2 puede" >&2
  echo "       seguir negando SCHED_FIFO. La comparación útil es en Linux nativo." >&2
fi

USUARIO="${1:-${SUDO_USER:-}}"
[ -n "$USUARIO" ] || { echo "ERROR: indica el usuario: sudo $0 <usuario>" >&2; exit 1; }
LIMITES=/etc/security/limits.d/99-ros2-realtime.conf

getent group realtime >/dev/null || groupadd realtime
usermod -aG realtime "$USUARIO"
cat > "$LIMITES" <<'CONF'
# Escrito por burger_kinova_reference/scripts/configurar_rt_linux.sh
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
CONF

echo "✓ $USUARIO en el grupo realtime; límites en $LIMITES"
echo "  Cierra sesión y vuelve a entrar; luego 'ulimit -r' debe decir 99."
echo "  Kernel actual: $(uname -v | grep -o 'PREEMPT[_A-Z]*' | head -1 || echo 'sin PREEMPT')"
echo "  (Un kernel PREEMPT_RT —p. ej. 'sudo pro enable realtime-kernel' en Ubuntu— mejora"
echo "   aún más la latencia, pero no es necesario para esta comparación.)"
