#!/usr/bin/env bash
# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
#
# Degradación controlada del enlace con tc/netem (TODO.md §4, "Inyección de tráfico").
#
# Aplica sobre UNA interfaz los escenarios de docs/research/EXPERIMENTO_QOS_TELEMETRIA.md §4:
#
#   E1  línea base       sin degradación (sólo quita cualquier netem previo)
#   E2  carga típica     retardo 20 ms ± 8 ms, pérdida 4 %
#   E3  estrés severo    retardo 60 ms ± 25 ms, pérdida 15 %
#   custom               --retardo-ms, --jitter-ms, --perdida-pct a mano
#
# netem sólo actúa sobre el tráfico SALIENTE de la interfaz. Con --ambos-sentidos el
# entrante se redirige a una interfaz ifb y se degrada igual. Con --destino IP sólo se
# degrada el tráfico hacia (y, con --ambos-sentidos, desde) esa IP; el resto —SSH, el
# monitor de red, otras estaciones— queda intacto.
#
# SEGURIDAD: la degradación se retira sola a los --auto-quitar segundos (600 por defecto),
# para que una sesión abandonada no deje la red del laboratorio degradada. Aplicarla sobre
# el enlace entre la anfitriona y el robot degrada el lazo de control en tiempo real de la
# API Kortex: ver network_setup/netem/README.md antes de hacerlo.
#
# Uso:
#   sudo ./perfil_netem.sh aplicar <iface> <E1|E2|E3|custom> [opciones]
#   sudo ./perfil_netem.sh quitar  <iface>
#        ./perfil_netem.sh estado  <iface>
#
# Ejemplos:
#   sudo ./perfil_netem.sh aplicar eth0 E2 --destino 192.168.1.20 --ambos-sentidos
#   sudo ./perfil_netem.sh aplicar wlan0 custom --retardo-ms 40 --jitter-ms 10 --perdida-pct 2
#   sudo ./perfil_netem.sh quitar eth0

set -euo pipefail

IFB=ifb_netem0
YO=$(readlink -f "$0")

uso() {
  sed -n '/^# Uso:/,/^$/p' "$0" | sed 's/^# \{0,1\}//'
  exit 1
}

requiere_root() {
  if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: $1 necesita root (sudo)." >&2
    exit 1
  fi
}

parametros_netem() {
  # Imprime los parámetros netem del perfil pedido.
  local perfil=$1 retardo=$2 jitter=$3 perdida=$4
  case "$perfil" in
    E2) echo "delay 20ms 8ms distribution normal loss 4%" ;;
    E3) echo "delay 60ms 25ms distribution normal loss 15%" ;;
    custom)
      local p="delay ${retardo}ms"
      if [ "$jitter" != "0" ]; then p="$p ${jitter}ms distribution normal"; fi
      echo "$p loss ${perdida}%"
      ;;
    *) echo "ERROR: perfil desconocido '$perfil'" >&2; exit 1 ;;
  esac
}

quitar() {
  local iface=$1
  tc qdisc del dev "$iface" root 2>/dev/null || true
  tc qdisc del dev "$iface" ingress 2>/dev/null || true
  if ip link show "$IFB" >/dev/null 2>&1; then
    tc qdisc del dev "$IFB" root 2>/dev/null || true
    ip link del "$IFB" 2>/dev/null || true
  fi
  pkill -f "perfil_netem_autoquitar_${iface}" 2>/dev/null || true
}

# Instala netem en <dev>: en toda la interfaz o, con destino, sólo para esa IP.
instalar() {
  local dev=$1 params=$2 destino=$3 campo=$4
  if [ -z "$destino" ]; then
    # shellcheck disable=SC2086
    tc qdisc add dev "$dev" root netem $params
  else
    # prio con 3 bandas normales + una cuarta con netem; el filtro manda a la banda 4
    # sólo los paquetes cuyo campo (dst o src) coincide con el destino.
    tc qdisc add dev "$dev" root handle 1: prio bands 4 priomap 1 2 2 2 1 2 0 0 1 1 1 1 1 1 1 1
    # shellcheck disable=SC2086
    tc qdisc add dev "$dev" parent 1:4 handle 40: netem $params
    tc filter add dev "$dev" parent 1: protocol ip prio 1 u32 \
      match ip "$campo" "$destino"/32 flowid 1:4
  fi
}

aplicar() {
  local iface=$1 perfil=$2; shift 2
  local destino="" ambos=0 auto=600 retardo=0 jitter=0 perdida=0
  while [ $# -gt 0 ]; do
    case "$1" in
      --destino) destino=$2; shift 2 ;;
      --ambos-sentidos) ambos=1; shift ;;
      --auto-quitar) auto=$2; shift 2 ;;
      --retardo-ms) retardo=$2; shift 2 ;;
      --jitter-ms) jitter=$2; shift 2 ;;
      --perdida-pct) perdida=$2; shift 2 ;;
      *) echo "ERROR: opción desconocida $1" >&2; uso ;;
    esac
  done

  ip link show "$iface" >/dev/null 2>&1 || { echo "ERROR: no existe la interfaz $iface" >&2; exit 1; }
  quitar "$iface"
  if [ "$perfil" = "E1" ]; then
    echo "E1 (línea base): $iface sin degradación."
    return
  fi

  local params
  params=$(parametros_netem "$perfil" "$retardo" "$jitter" "$perdida")

  if ! instalar "$iface" "$params" "$destino" dst 2>/tmp/perfil_netem.err; then
    echo "ERROR: tc no pudo instalar netem en $iface:" >&2
    cat /tmp/perfil_netem.err >&2
    echo "  ¿Falta el módulo del kernel? Prueba 'sudo modprobe sch_netem'. El kernel de" >&2
    echo "  WSL2 no siempre lo trae: en ese caso aplica netem en una máquina Linux nativa." >&2
    quitar "$iface"
    exit 1
  fi

  if [ "$ambos" -eq 1 ]; then
    modprobe ifb 2>/dev/null || true
    ip link add "$IFB" type ifb 2>/dev/null || true
    ip link set "$IFB" up
    tc qdisc add dev "$iface" handle ffff: ingress
    tc filter add dev "$iface" parent ffff: protocol ip u32 match u32 0 0 \
      action mirred egress redirect dev "$IFB"
    instalar "$IFB" "$params" "$destino" src
  fi

  # Retirada automática en segundo plano; el nombre del proceso permite cancelarla.
  if [ "$auto" -gt 0 ]; then
    nohup bash -c "exec -a perfil_netem_autoquitar_${iface} bash -c 'sleep $auto; \"$YO\" quitar $iface'" \
      >/dev/null 2>&1 &
  fi

  echo "Perfil $perfil aplicado en $iface: netem $params"
  [ -n "$destino" ] && echo "  sólo tráfico con $destino"
  [ "$ambos" -eq 1 ] && echo "  en ambos sentidos (entrada vía $IFB)"
  [ "$auto" -gt 0 ] && echo "  se retira solo en ${auto} s (o: sudo $0 quitar $iface)"
  return 0
}

estado() {
  local iface=$1
  echo "== $iface =="
  tc -s qdisc show dev "$iface"
  if ip link show "$IFB" >/dev/null 2>&1; then
    echo "== $IFB (entrada) =="
    tc -s qdisc show dev "$IFB"
  fi
}

[ $# -ge 2 ] || uso
accion=$1; iface=$2; shift 2
case "$accion" in
  aplicar) requiere_root aplicar; [ $# -ge 1 ] || uso; aplicar "$iface" "$@" ;;
  quitar) requiere_root quitar; quitar "$iface"; echo "$iface sin netem." ;;
  estado) estado "$iface" ;;
  *) uso ;;
esac
