#!/usr/bin/env bash
# Parche de parada limpia para el driver de visión del Kinova (ros2_kortex_vision).
#
# Sólo lo necesita la ESTACIÓN ANFITRIONA, la única que ejecuta el driver de visión.
# Las estaciones cliente se suscriben a la imagen y no instalan kinova_vision.
#
# En un solo paso: clona el driver si falta, revisa las dependencias, aplica el parche
# (sin repetirlo si ya está), verifica que quedó aplicado y compila kinova_vision.
#
# Uso:
#   bash scripts/aplicar_parche_kinova_vision.sh           # prepara y compila
#   bash scripts/aplicar_parche_kinova_vision.sh --check   # sólo comprueba, no modifica nada
#
# Variables opcionales:
#   WS_SRC=~/otro_ws/src bash scripts/aplicar_parche_kinova_vision.sh
#
# Por qué hace falta y qué corrige: TROUBLESHOOTING.md §3.4.

set -uo pipefail

MODO="aplicar"
case "${1:-}" in
  --check) MODO="check" ;;
  "")      ;;
  *) echo "Opción desconocida: $1"; echo "Usa: (sin opciones) | --check"; exit 2 ;;
esac

if [[ -t 1 ]]; then
  ROJO=$'\e[31m'; VERDE=$'\e[32m'; AMAR=$'\e[33m'; NEG=$'\e[1m'; FIN=$'\e[0m'
else
  ROJO=""; VERDE=""; AMAR=""; NEG=""; FIN=""
fi
ok()     { echo "  ${VERDE}OK${FIN}      $1"; }
aviso()  { echo "  ${AMAR}AVISO${FIN}   $1"; }
fallar() { echo "  ${ROJO}FALLA${FIN}   $1"; shift; for linea in "$@"; do echo "          $linea"; done; exit 1; }
titulo() { echo; echo "${NEG}$1${FIN}"; }

RAIZ_PROYECTO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARCHE="$RAIZ_PROYECTO/ros2_setup/parches/kinova_vision_parada_limpia.patch"
WS_SRC="${WS_SRC:-$HOME/ros2_ws/src}"
WS="$(dirname "$WS_SRC")"
DRIVER="$WS_SRC/ros2_kortex_vision"
URL_UPSTREAM="https://github.com/Kinovarobotics/ros2_kortex_vision.git"

echo "${NEG}=== Parche de parada limpia para kinova_vision (modo: $MODO) ===${FIN}"

# ------------------------------------------------------------------- 1. driver
titulo "1. Driver de visión"
[[ -f "$PARCHE" ]] || fallar "No se encuentra el parche: $PARCHE"
if [[ ! -d "$DRIVER/.git" ]]; then
  if [[ "$MODO" == "check" ]]; then
    fallar "No existe $DRIVER" "Ejecuta el script sin --check para clonarlo."
  fi
  aviso "No existe $DRIVER: se clona"
  git clone -b ros2 "$URL_UPSTREAM" "$DRIVER" \
    || fallar "No se pudo clonar $URL_UPSTREAM" "Revisa la conexión a Internet."
fi
ok "Driver en $DRIVER"

# ------------------------------------------------------------- 2. dependencias
titulo "2. Dependencias"
FALTAN=()
for paquete in ros-jazzy-camera-calibration-parsers ros-jazzy-camera-info-manager \
               ros-jazzy-image-transport-plugins libgstreamer1.0-dev \
               libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-good gstreamer1.0-libav; do
  dpkg-query -W -f='${db:Status-Abbrev}' "$paquete" 2>/dev/null | grep -q '^ii' || FALTAN+=("$paquete")
done
if (( ${#FALTAN[@]} )); then
  fallar "Faltan paquetes del sistema: ${FALTAN[*]}" \
         "Instálalos y vuelve a ejecutar el script:" \
         "sudo apt install -y ${FALTAN[*]}"
fi
ok "Paquetes del sistema instalados"

# ------------------------------------------------------------------ 3. parche
titulo "3. Parche"
if git -C "$DRIVER" apply --reverse --check "$PARCHE" 2>/dev/null; then
  ok "El parche ya estaba aplicado"
elif git -C "$DRIVER" apply --check "$PARCHE" 2>/dev/null; then
  if [[ "$MODO" == "check" ]]; then
    fallar "El parche NO está aplicado" "Ejecuta el script sin --check para aplicarlo."
  fi
  git -C "$DRIVER" apply "$PARCHE" || fallar "git apply falló al aplicar el parche"
  ok "Parche aplicado"
else
  fallar "El parche no aplica sobre este clon" \
         "O el driver tiene cambios locales, o el upstream cambió desde el commit d1d0213." \
         "Cambios locales: $(git -C "$DRIVER" status --short | tr '\n' ' ')" \
         "No lo fuerces: revisa TROUBLESHOOTING.md §3.4."
fi

# Verificación del contenido: no basta con que git no se queje.
NODO="$DRIVER/src/vision_node.cpp"
VISION="$DRIVER/src/vision.cpp"
grep -q "rclcpp::on_shutdown" "$NODO"                   || fallar "vision_node.cpp no usa rclcpp::on_shutdown"
! grep -q "sigintHandler" "$NODO"                       || fallar "vision_node.cpp conserva el manejador de SIGINT inseguro"
grep -q "std::signal(SIGHUP" "$NODO"                    || fallar "vision_node.cpp no gestiona SIGHUP"
grep -q "SingleThreadedExecutor executor" "$VISION"     || fallar "vision.cpp no usa un ejecutor único"
ok "Contenido verificado (on_shutdown, SIGHUP, ejecutor único, sin sigintHandler)"

if [[ "$MODO" == "check" ]]; then
  echo; echo "${VERDE}Todo en orden.${FIN} Si no has compilado desde que se aplicó el parche:"
  echo "  cd $WS && colcon build --packages-select kinova_vision --symlink-install"
  exit 0
fi

# ---------------------------------------------------------------- 4. compilar
titulo "4. Compilación"
if ! command -v colcon >/dev/null || [[ -z "${ROS_DISTRO:-}" ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash 2>/dev/null || fallar "No se pudo cargar /opt/ros/jazzy/setup.bash"
fi
( cd "$WS" && colcon build --packages-select kinova_vision --symlink-install ) \
  || fallar "La compilación de kinova_vision falló" "Revisa el log de colcon en $WS/log/latest_build"
ok "kinova_vision compilado en $WS/install"

echo
echo "${VERDE}Listo.${FIN} En cada terminal nueva: source $WS/install/setup.bash"
echo "Para comprobar la parada limpia: lanza el driver, detenlo con Ctrl+C y verifica que ambos"
echo "nodos terminen con 'process has finished cleanly' y que 'pgrep -a -x kinova_vision_n' no muestre nada."
