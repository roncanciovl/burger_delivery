#!/usr/bin/env bash
# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
#
# Grabación quirúrgica de la evidencia del enlace con el Kinova Gen3.
#
# Aplica las reglas del taller de rosbag2/logging:
#   * almacenamiento MCAP (esquemas embebidos: la bolsa se abre sin el workspace del curso),
#   * compresión Zstandard por archivo,
#   * fragmentación por duración para que un corte de energía no invalide la sesión,
#   * lista explícita de tópicos: NUNCA "-a", NUNCA imágenes crudas,
#   * /rosout incluido, para que la telemetría y el log que la explica queden
#     correlacionados en el mismo archivo.
#
# Uso:
#   ./record_kinova_bag.sh [nombre_del_dataset] [duracion_max_por_fragmento_s]
#
# Ejemplo (evidencia de PA-03, telemetría real durante 60 s):
#   ./record_kinova_bag.sh dataset_pa03_telemetria_real 60

set -euo pipefail

DATASET="${1:-dataset_conexion_kinova_$(date +%Y%m%d_%H%M%S)}"
SPLIT_S="${2:-60}"

TOPICS=(
  /joint_states
  /burger/kinova/diagnostics
  /rosout
  /tf
  /tf_static
)

echo "── Grabación de evidencia del enlace Kinova ──"
echo "  Dataset          : ${DATASET}"
echo "  Almacenamiento   : mcap (compresión zstd por archivo)"
echo "  Fragmentación    : ${SPLIT_S} s"
echo "  ROS_DOMAIN_ID    : ${ROS_DOMAIN_ID:-0 (por defecto)}"
echo "  Tópicos          : ${TOPICS[*]}"
echo
echo "  Detén la grabación con Ctrl+C. Después inspecciona con:"
echo "      ros2 bag info ${DATASET}"
echo

exec ros2 bag record \
  --storage mcap \
  --compression-mode file \
  --compression-format zstd \
  --max-bag-duration "${SPLIT_S}" \
  --output "${DATASET}" \
  "${TOPICS[@]}"
