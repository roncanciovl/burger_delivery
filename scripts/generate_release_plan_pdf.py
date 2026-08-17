#!/usr/bin/env python3
"""
Script to generate a high-impact, professional academic PDF for the Zenodo Release & Publication Plan (2026-2027).
Uses WeasyPrint for pixel-perfect PDF rendering with modern CSS styling, official research lines,
verified DataCite DOI references, and interactive hyperlinks.
"""

import os
import weasyprint

HTML_CONTENT = """<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<style>
  @page {
    size: A4;
    margin: 1.6cm 1.5cm 1.6cm 1.5cm;
    @bottom-right {
      content: "Página " counter(page) " de " counter(pages);
      font-family: 'Liberation Sans', Helvetica, Arial, sans-serif;
      font-size: 8pt;
      color: #64748b;
    }
    @bottom-left {
      content: "Plan Estratégico de Publicaciones y Releases Zenodo — Grupo VOLTA (A1 - MinCiencias)";
      font-family: 'Liberation Sans', Helvetica, Arial, sans-serif;
      font-size: 8pt;
      color: #64748b;
    }
  }

  body {
    font-family: 'Liberation Sans', Helvetica, Arial, sans-serif;
    color: #1e293b;
    line-height: 1.40;
    font-size: 9.0pt;
  }

  a {
    color: #0369a1;
    text-decoration: none;
  }

  .header-card {
    border-left: 5px solid #0284c7;
    background: linear-gradient(135deg, #f0f9ff 0%, #e0f2fe 100%);
    padding: 14px 18px;
    border-radius: 6px;
    margin-bottom: 14px;
  }

  .header-title {
    font-size: 14pt;
    font-weight: 700;
    color: #0c4a6e;
    margin: 0 0 6px 0;
    text-transform: uppercase;
    letter-spacing: 0.5px;
  }

  .header-subtitle {
    font-size: 9.5pt;
    font-weight: 600;
    color: #0369a1;
    margin: 0 0 10px 0;
  }

  .meta-grid {
    display: table;
    width: 100%;
    margin-top: 6px;
    font-size: 8.5pt;
  }

  .meta-row {
    display: table-row;
  }

  .meta-cell {
    display: table-cell;
    padding: 2px 6px;
  }

  .badge-doi {
    display: inline-block;
    background: #0284c7;
    color: white;
    font-size: 7.5pt;
    font-weight: 700;
    padding: 2px 8px;
    border-radius: 4px;
    margin-top: 4px;
  }

  h2 {
    font-size: 11pt;
    color: #0f172a;
    border-bottom: 1.5px solid #0284c7;
    padding-bottom: 3px;
    margin-top: 14px;
    margin-bottom: 8px;
    text-transform: uppercase;
    letter-spacing: 0.3px;
  }

  h3 {
    font-size: 9.5pt;
    color: #0369a1;
    margin-top: 10px;
    margin-bottom: 4px;
  }

  p {
    margin: 0 0 8px 0;
    text-align: justify;
  }

  table {
    width: 100%;
    border-collapse: collapse;
    margin: 10px 0;
    font-size: 8.2pt;
  }

  th {
    background-color: #0c4a6e;
    color: white;
    font-weight: 600;
    padding: 6px 8px;
    text-align: left;
    border: 1px solid #0c4a6e;
  }

  td {
    padding: 5px 8px;
    border: 1px solid #cbd5e1;
    vertical-align: top;
  }

  tr:nth-child(even) td {
    background-color: #f8fafc;
  }

  .highlight-box {
    background-color: #f8fafc;
    border-left: 4px solid #3b82f6;
    padding: 8px 12px;
    margin: 8px 0;
    border-radius: 0 4px 4px 0;
    font-size: 8.5pt;
  }

  .checklist {
    list-style-type: none;
    padding-left: 0;
    margin: 6px 0;
  }

  .checklist li {
    position: relative;
    padding-left: 18px;
    margin-bottom: 4px;
    font-size: 8.5pt;
  }

  .checklist li::before {
    content: "✓";
    position: absolute;
    left: 0;
    color: #0284c7;
    font-weight: bold;
  }

  .signatures-table {
    width: 100%;
    margin-top: 20px;
    border: none;
  }

  .signatures-table td {
    border: none;
    text-align: center;
    padding: 10px 15px;
    background-color: transparent !important;
  }

  .sig-line {
    border-top: 1px solid #475569;
    margin-bottom: 5px;
    padding-top: 4px;
    font-size: 8.5pt;
    font-weight: bold;
  }
</style>
</head>
<body>

<div class="header-card">
  <div class="header-title">Plan Estratégico de Publicaciones y Releases Zenodo</div>
  <div class="header-subtitle">Gestión de Datasets, Reproducibilidad de Software y Cronograma Editorial (2026 - 2027)</div>
  <div class="meta-grid">
    <div class="meta-row">
      <div class="meta-cell"><strong>Proyecto:</strong> Burger-Cell (Framework ROS 2 Colaborativo)</div>
      <div class="meta-cell"><strong>Grupo:</strong> VOLTA (A1 - MinCiencias) — Línea 2: Diseños Mecatrónicos</div>
    </div>
    <div class="meta-row">
      <div class="meta-cell"><strong>Investigador Principal:</strong> Prof. Henry Antonio Roncancio Velandia</div>
      <div class="meta-cell"><strong>Institución:</strong> Universidad Militar Nueva Granada (Campus Cajicá)</div>
    </div>
  </div>
  <a href="https://doi.org/10.5281/zenodo.21809949" target="_blank">
    <div class="badge-doi">Concept DOI Persistente: 10.5281/zenodo.21809949</div>
  </a>
</div>

<h2>1. Justificación y Fundamentos de los Releases Científicos</h2>
<p>
En la investigación de vanguardia en robótica, el repositorio digital constituye un <strong>instrumento de reproducibilidad científica y un producto de desarrollo tecnológico verificable</strong> ante organismos de acreditación (MinCiencias, Scopus, DataCite y ABET). A diferencia del desarrollo de software comercial, los releases en <strong>Zenodo / DataCite</strong> conllevan responsabilidades editoriales fundamentales:
</p>
<div class="highlight-box">
  <strong>• Inmutabilidad en Servidores del CERN:</strong> Cada versión con DOI emitido congela un snapshot (.zip) con hash criptográfico que no puede ser eliminado ni alterado.<br>
  <strong>• Trazabilidad de Citas (Citable Snapshots):** Los pares evaluadores de papers o comités acceden exactamente al código y datos usados en los experimentos reportados.<br>
  <strong>• Concept DOI vs Version DOI:</strong> El <em>Concept DOI (10.5281/zenodo.21809949)</em> resuelve permanentemente a la versión más reciente para difusión global, mientras los <em>Version DOIs</em> preservan la historia experimental.
</div>

<h2>2. Cronograma Maestro de Releases y Publicaciones (2026 - 2027)</h2>
<table>
  <thead>
    <tr>
      <th style="width: 12%;">Versión</th>
      <th style="width: 14%;">Fecha Meta</th>
      <th style="width: 32%;">Hito Científico Asociado</th>
      <th style="width: 18%;">DOI Zenodo</th>
      <th style="width: 24%;">Productos Entregables</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong>v1.0.0</strong></td>
      <td><strong>Ago 2026</strong><br><em>(Ejecutado)</em></td>
      <td><strong>Línea Base del Framework y Registro Tecnológico</strong></td>
      <td><code>10.5281/zenodo.21809950</code></td>
      <td>• Paquete ROS 2 Jazzy y cinemática.<br>• Monitor de Red QoS CycloneDDS.<br>• Syllabus y guías docentes ABET.</td>
    </tr>
    <tr>
      <td><strong>v1.1.0</strong></td>
      <td><strong>Nov 2026</strong></td>
      <td><strong>Dataset Abierto de Telemetría e Inferencia VLM</strong></td>
      <td><em>Nuevo DOI Asignado</em></td>
      <td>• Pipeline Gemini Robotics + MoveIt 2.<br>• Dataset 500+ ciclos (DDS vs VLM).<br>• Cierre y evidencias ABET 2026-2.</td>
    </tr>
    <tr>
      <td><strong>v1.2.0</strong></td>
      <td><strong>Mar 2027</strong></td>
      <td><strong>Snapshot Oficial: Envío de Flagship Paper Q1</strong></td>
      <td><em>Nuevo DOI Asignado</em></td>
      <td>• Código congelado para reproducibilidad.<br>• Scripts de análisis automatizado.<br>• Manuscrito enviado (IEEE T-ASE/RA-L).</td>
    </tr>
    <tr>
      <td><strong>v2.0.0</strong></td>
      <td><strong>Jul 2027</strong></td>
      <td><strong>Arquitectura Multi-Agente Autónoma y Tesis</strong></td>
      <td><em>Nuevo DOI Asignado</em></td>
      <td>• micro-ROS en ESP32-S3 físico.<br>• Percepción multi-cámara 3D.<br>• Tesis de Maestría y ponencia IEEE.</td>
    </tr>
  </tbody>
</table>

<h2>3. Detalle de Hitos Editoriales y Articulación VOLTA</h2>

<h3>• Hito 1: Versión v1.0.0 — Software Científico Base (Completado)</h3>
<p>
Publicación de la plataforma experimental completa (Kinova Gen3 7-DOF, Robotiq 85, carritos TurtleBot, AprilTags dinámicos y dashboard de telemetría QoS a 1 Hz). Registrado con el DOI <strong>10.5281/zenodo.21809950</strong> y reportable en GrupLAC como Producto Tecnológico.
</p>

<h3>• Hito 2: Versión v1.1.0 — Dataset Abierto Unificado (Noviembre 2026)</h3>
<p>
Liberación del primer gran banco de datos experimentales que correlaciona latencia RTT, jitter de red inalámbrica, tiempo de inferencia VLM zero-shot (Gemini ER) y error de seguimiento articular (RMSE). Publicado en <strong>IEEE DataPort / Zenodo Data</strong> con DOI de dataset abierto.
</p>

<h3>• Hito 3: Versión v1.2.0 — Reproducibilidad del Flagship Paper Q1 (Marzo 2027)</h3>
<p>
Snapshot inmutable citado en la sección de disponibilidad de código del artículo principal: <em>"VLM-Guided 3D Spatial Reasoning Under Network QoS Uncertainty: A Real-Time Telemetry and Manipulation Benchmark in Heterogeneous ROS 2 Robotic Cells"</em>, enviado a revistas IEEE Q1/Q2.
</p>

<h3>• Hito 4: Versión v2.0.0 — Celda Multi-Agente y Cierre de Posgrados (Julio 2027)</h3>
<p>
Evolución a arquitectura distribuida con micro-ROS en microcontroladores ESP32-S3, coordinación multi-robot y sustentación de tesis de Maestría en Ingeniería Mecatrónica, con postulación a ponencias en conferencias IEEE CASE / IROS.
</p>

<h2>4. Protocolo Operativo y Checklist Técnico Pre-Release</h2>
<ul class="checklist">
  <li><strong>Validación Técnica:</strong> Compilación limpia (<code>colcon build</code>) y validación cinemática y de cámara (<code>scripts/test_kinova_*.py</code>).</li>
  <li><strong>Metadatos CITATION.cff:</strong> Actualizar <code>version</code>, <code>date-released</code> y confirmación de autores con afiliación UMNG.</li>
  <li><strong>Metadatos .zenodo.json:</strong> Sincronizar descripción del hito, palabras clave tecnológicas y colaboradores estudiantiles.</li>
  <li><strong>Tag & Release en GitHub:</strong> Crear tag firmado y publicar release con Changelog Científico y Académico formal.</li>
  <li><strong>Verificación Zenodo:</strong> Confirmar ingestión por webhook y registrar el nuevo <em>Version DOI</em> asignado.</li>
  <li><strong>Actualización Curricular:</strong> Actualizar READMEs y sincronizar producción en CvLAC / GrupLAC de MinCiencias.</li>
</ul>

<table class="signatures-table">
  <tr>
    <td style="width: 50%;">
      <div class="sig-line">Prof. Henry Antonio Roncancio Velandia</div>
      Investigador Proponente — Programa Ingeniería Mecatrónica<br>Universidad Militar Nueva Granada
    </td>
    <td style="width: 50%;">
      <div class="sig-line">Dr. William Gómez Rivera, Ph.D. / Dr. William Aperador</div>
      Docente Investigador / Líder Grupo de Investigación VOLTA (A1)<br>Facultad de Ingeniería — UMNG
    </td>
  </tr>
</table>

</body>
</html>
"""

def main():
    output_pdf = "/home/roncanciovl/ros2_ws/src/burger_delivery/docs/research/PLAN_PUBLICACIONES_Y_RELEASES_ZENODO.pdf"
    print("Generating academic PDF for Zenodo Release Plan...")
    weasyprint.HTML(string=HTML_CONTENT).write_pdf(output_pdf)
    print(f"✅ PDF successfully generated at: {output_pdf}")

if __name__ == "__main__":
    main()
