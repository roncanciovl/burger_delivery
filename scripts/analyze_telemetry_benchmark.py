#!/usr/bin/env python3
"""
analyze_telemetry_benchmark.py
Herramienta de análisis estadístico y generación de gráficas de publicación (IEEE/Sensors)
a partir de los datasets CSV recolectados por el Monitor de Red de Burger-Cell.

Uso:
    python3 scripts/analyze_telemetry_benchmark.py
    python3 scripts/analyze_telemetry_benchmark.py --file network_setup/monitor_red/benchmark_logs/mi_benchmark.csv
    python3 scripts/analyze_telemetry_benchmark.py --input-dir network_setup/monitor_red/benchmark_logs/
"""

import argparse
import csv
import glob
import os
import sys
from typing import Dict, List, Any

# Verificar disponibilidad de librerías gráficas
try:
    import matplotlib
    matplotlib.use("Agg") # Modo sin display para entornos headless/servidor
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


def load_csv_dataset(filepath: str) -> List[Dict[str, Any]]:
    """Carga y parsea un archivo CSV de telemetría de red"""
    records = []
    if not os.path.exists(filepath):
        return records

    with open(filepath, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            parsed = {}
            for k, v in row.items():
                try:
                    if "." in v:
                        parsed[k] = float(v)
                    elif v.isdigit() or (v.startswith("-") and v[1:].isdigit()):
                        parsed[k] = int(v)
                    else:
                        parsed[k] = v
                except Exception:
                    parsed[k] = v
            records.append(parsed)
    return records


def calculate_statistics(records: List[Dict[str, Any]], field: str) -> Dict[str, float]:
    """Calcula métricas estadísticas descriptivas (Media, Desv. Estándar, Min, Max, Percentil 95)"""
    vals = [r[field] for r in records if field in r and isinstance(r[field], (int, float))]
    if not vals:
        return {"count": 0, "mean": 0.0, "std": 0.0, "min": 0.0, "max": 0.0, "p95": 0.0}

    vals.sort()
    n = len(vals)
    mean = sum(vals) / n
    variance = sum((x - mean) ** 2 for x in vals) / max(n - 1, 1)
    std = variance ** 0.5
    p95_idx = int(0.95 * n)
    p95 = vals[min(p95_idx, n - 1)]

    return {
        "count": n,
        "mean": round(mean, 2),
        "std": round(std, 2),
        "min": round(vals[0], 2),
        "max": round(vals[-1], 2),
        "p95": round(p95, 2)
    }


def print_summary_table(dataset_name: str, records: List[Dict[str, Any]]):
    """Imprime una tabla formateada en Markdown con las estadísticas del ensayo"""
    print(f"\n=======================================================================")
    print(f"📊 RESUMEN ESTADÍSTICO DE TELEMETRÍA: {dataset_name}")
    print(f"=======================================================================")
    print(f"Total de muestras: {len(records)} seg | Escenario: {records[0].get('scenario', 'N/A') if records else 'N/A'}")
    print(f"-----------------------------------------------------------------------")
    print(f"{'Métrica':<26} | {'Media':<10} | {'Desv.Std':<10} | {'Min':<8} | {'Max':<8} | {'p95':<8}")
    print(f"-----------------------------------------------------------------------")

    metrics_to_show = [
        ("Gateway Latencia (ms)", "gateway_latency_ms"),
        ("Gateway Jitter (ms)", "gateway_jitter_ms"),
        ("Gateway Pérdida (%)", "gateway_loss_percent"),
        ("DDS Latencia (ms)", "dds_latency_ms"),
        ("DDS Jitter (ms)", "dds_jitter_ms"),
        ("DDS Pérdida (%)", "dds_loss_percent"),
        ("Tráfico Total (KB/s)", "total_kbps"),
        ("Tráfico DDS (KB/s)", "dds_kbps"),
        ("Tráfico micro-ROS (KB/s)", "microros_kbps"),
    ]

    for label, field in metrics_to_show:
        stats = calculate_statistics(records, field)
        print(f"{label:<26} | {stats['mean']:<10} | {stats['std']:<10} | {stats['min']:<8} | {stats['max']:<8} | {stats['p95']:<8}")

    print(f"-----------------------------------------------------------------------\n")


def generate_publication_plots(records: List[Dict[str, Any]], output_path: str):
    """Genera figuras de alta resolución (300 DPI) para papers académicos"""
    if not MATPLOTLIB_AVAILABLE:
        print("⚠️ Matplotlib no está disponible en este entorno. Instalar con: pip install matplotlib")
        return

    if not records:
        print("⚠️ No hay registros suficientes para graficar.")
        return

    time_series = [r.get("elapsed_sec", i) for i, r in enumerate(records)]
    rtt = [r.get("gateway_latency_ms", 0.0) for r in records]
    jitter = [r.get("gateway_jitter_ms", 0.0) for r in records]
    dds_kbps = [r.get("dds_kbps", 0.0) for r in records]
    loss = [r.get("gateway_loss_percent", 0.0) for r in records]

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, dpi=300)
    plt.subplots_adjust(hspace=0.25)

    # Gráfica 1: RTT y Jitter
    axes[0].plot(time_series, rtt, color="#0284c7", linewidth=2.0, label="RTT Gateway (ms)")
    axes[0].plot(time_series, jitter, color="#9333ea", linewidth=1.5, linestyle="--", label="Jitter mdev (ms)")
    axes[0].set_ylabel("Latencia / Jitter (ms)", fontsize=10, fontweight="bold")
    axes[0].set_title(f"Telemetría Temporal de Red - Escenario: {records[0].get('scenario', 'Test')}", fontsize=12, fontweight="bold")
    axes[0].grid(True, linestyle=":", alpha=0.6)
    axes[0].legend(loc="upper right", frameon=True)

    # Gráfica 2: Tráfico DDS / micro-ROS
    axes[1].fill_between(time_series, dds_kbps, color="#a855f7", alpha=0.3)
    axes[1].plot(time_series, dds_kbps, color="#7c3aed", linewidth=2.0, label="Tráfico ROS 2 DDS (KB/s)")
    axes[1].set_ylabel("Ancho de Banda (KB/s)", fontsize=10, fontweight="bold")
    axes[1].grid(True, linestyle=":", alpha=0.6)
    axes[1].legend(loc="upper right", frameon=True)

    # Gráfica 3: Tasa de Pérdida de Paquetes
    axes[2].bar(time_series, loss, width=0.8, color="#ef4444", alpha=0.7, label="Pérdida de Paquetes (%)")
    axes[2].set_ylabel("Pérdida (%)", fontsize=10, fontweight="bold")
    axes[2].set_xlabel("Tiempo Transcurrido (segundos)", fontsize=10, fontweight="bold")
    axes[2].grid(True, linestyle=":", alpha=0.6)
    axes[2].legend(loc="upper right", frameon=True)

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    plt.savefig(output_path, bbox_inches="tight")
    plt.close()
    print(f"✅ Figura de publicación generada exitosamente en: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Analizador estadístico de datasets de telemetría QoS de Burger-Cell.")
    parser.add_argument("--file", "-f", type=str, help="Ruta directa al archivo CSV a analizar.")
    parser.add_argument("--input-dir", "-d", type=str, default="network_setup/monitor_red/benchmark_logs",
                        help="Directorio donde buscar archivos CSV.")
    parser.add_argument("--output-plot", "-o", type=str, default="docs/research/figures/telemetry_benchmark_plot.png",
                        help="Ruta donde guardar la gráfica generada.")

    args = parser.parse_args()

    target_file = args.file
    if not target_file:
        csv_files = glob.glob(os.path.join(args.input_dir, "*.csv"))
        if csv_files:
            csv_files.sort(key=os.path.getmtime, reverse=True)
            target_file = csv_files[0]
            print(f"📂 Seleccionado dataset más reciente: {target_file}")
        else:
            print(f"ℹ️ No se encontraron archivos CSV en '{args.input_dir}'.")
            print(f"   Inicia una sesión de prueba desde la interfaz web o pasa un archivo con --file.")
            return

    records = load_csv_dataset(target_file)
    if not records:
        print(f"❌ No se pudieron cargar registros de {target_file}")
        return

    print_summary_table(os.path.basename(target_file), records)
    generate_publication_plots(records, args.output_plot)


if __name__ == "__main__":
    main()
