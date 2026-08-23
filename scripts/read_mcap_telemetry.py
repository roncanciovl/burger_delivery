#!/usr/bin/env python3
"""
read_mcap_telemetry.py
Script de extracción y análisis programático de datasets rosbag2 (MCAP / SQLite3)
usando la API directa `rosbag2_py` sin reproducir tráfico en la red DDS.

Uso:
    python3 scripts/read_mcap_telemetry.py <path_al_bag>
    Ejemplo: python3 scripts/read_mcap_telemetry.py dataset_telemetria_burger
"""

import os
import sys
from typing import Dict, List

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def analyze_bag(bag_path: str):
    if not os.path.exists(bag_path):
        print(f"❌ Error: La ruta '{bag_path}' no existe.")
        sys.exit(1)

    # Detectar o configurar plugin de almacenamiento (MCAP o sqlite3)
    storage_id = 'mcap'
    # Si hay archivos .db3 en el directorio, usar sqlite3 como fallback
    if any(f.endswith('.db3') for f in os.listdir(bag_path)):
        storage_id = 'sqlite3'

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"❌ Error al abrir la bolsa con plugin '{storage_id}': {e}")
        return

    # Catálogo de tópicos y tipos de mensajes
    topics_and_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topics_and_types}

    print("=" * 70)
    print(f"📦 ANÁLISIS PROGRAMÁTICO DE DATASET: {bag_path}")
    print(f"🔌 Plugin de Almacenamiento: {storage_id.upper()}")
    print("=" * 70)
    print(f"📌 Tópicos registrados ({len(type_map)}):")
    for topic_name, msg_type in type_map.items():
        print(f"   • {topic_name}  [{msg_type}]")
    print("-" * 70)

    msg_counts: Dict[str, int] = {topic: 0 for topic in type_map}
    jitter_values: List[float] = []
    fault_counter = 0

    while reader.has_next():
        (topic, data, timestamp_ns) = reader.read_next()
        msg_counts[topic] = msg_counts.get(topic, 0) + 1

        try:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)

            # Procesamiento específico de telemetría si existe el tópico de jitter
            if "joint_jitter" in topic and hasattr(msg, "data"):
                val = float(msg.data)
                jitter_values.append(val)
                if abs(val) > 0.3:
                    fault_counter += 1

            elif "system_health" in topic and hasattr(msg, "data"):
                if "FAULT" in str(msg.data):
                    fault_counter += 1

        except Exception as err:
            pass

    total_msgs = sum(msg_counts.values())
    print(f"📊 Desglose de Mensajes por Tópico:")
    for topic_name, count in msg_counts.items():
        print(f"   • {topic_name}: {count} mensajes")
    print("-" * 70)
    print(f"✅ Total general de mensajes procesados: {total_msgs}")

    if jitter_values:
        avg_jitter = sum(jitter_values) / len(jitter_values)
        max_jitter = max(jitter_values)
        min_jitter = min(jitter_values)
        print(f"📈 Métricas de Calidad Cinemática (Jitter):")
        print(f"   • Promedio: {avg_jitter:.5f}")
        print(f"   • Máximo:   {max_jitter:.5f}")
        print(f"   • Mínimo:   {min_jitter:.5f}")
        print(f"   • Eventos Críticos de Falla Detectados: {fault_counter}")
    print("=" * 70)


def main():
    if len(sys.argv) < 2:
        print("Uso: python3 scripts/read_mcap_telemetry.py <path_al_directorio_del_bag>")
        print("Ejemplo: python3 scripts/read_mcap_telemetry.py dataset_telemetria_burger")
    else:
        analyze_bag(sys.argv[1])


if __name__ == '__main__':
    main()
