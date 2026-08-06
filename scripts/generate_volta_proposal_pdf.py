#!/usr/bin/env python3
"""
Script to generate a high-impact, professional academic PDF proposal for Grupo VOLTA (UMNG).
Uses WeasyPrint for pixel-perfect PDF rendering with modern CSS styling and clickable hyperlinks.
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
    margin: 1.7cm 1.5cm 1.7cm 1.5cm;
    @bottom-right {
      content: "Página " counter(page) " de " counter(pages);
      font-family: 'Liberation Sans', Helvetica, Arial, sans-serif;
      font-size: 8pt;
      color: #64748b;
    }
    @bottom-left {
      content: "Propuesta de Vinculación Científica — Grupo VOLTA (A1 - MinCiencias)";
      font-family: 'Liberation Sans', Helvetica, Arial, sans-serif;
      font-size: 8pt;
      color: #64748b;
    }
  }

  body {
    font-family: 'Liberation Sans', Helvetica, Arial, sans-serif;
    color: #1e293b;
    line-height: 1.42;
    font-size: 9.2pt;
  }

  a {
    color: #1d4ed8;
    text-decoration: none;
    font-weight: 600;
  }

  a:hover {
    text-decoration: underline;
  }

  /* Header banner */
  .header-table {
    width: 100%;
    border-bottom: 3px solid #1e3a8a;
    padding-bottom: 10px;
    margin-bottom: 14px;
  }

  .header-title {
    font-size: 14pt;
    font-weight: 800;
    color: #1e3a8a;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    margin: 0;
  }

  .header-subtitle {
    font-size: 9pt;
    color: #475569;
    margin-top: 3px;
    font-weight: 600;
  }

  .badge-container {
    text-align: right;
  }

  .badge {
    display: inline-block;
    padding: 3px 8px;
    border-radius: 4px;
    font-size: 7.8pt;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.4px;
  }

  .badge-a1 {
    background-color: #fef3c7;
    color: #92400e;
    border: 1px solid #f59e0b;
  }

  .badge-doi {
    background-color: #e0f2fe;
    color: #0369a1;
    border: 1px solid #38bdf8;
    margin-top: 4px;
  }

  .badge-github {
    background-color: #f1f5f9;
    color: #0f172a;
    border: 1px solid #94a3b8;
    margin-top: 4px;
  }

  /* Addressees box */
  .destinatarios-box {
    background-color: #f8fafc;
    border: 1px solid #e2e8f0;
    border-left: 4px solid #1e3a8a;
    border-radius: 4px;
    padding: 9px 12px;
    margin-bottom: 14px;
  }

  .destinatarios-table {
    width: 100%;
  }

  .destinatarios-table td {
    vertical-align: top;
    font-size: 8.8pt;
  }

  /* Project Title Box */
  .title-card {
    background: linear-gradient(135deg, #1e3a8a 0%, #1e40af 100%);
    color: #ffffff;
    padding: 12px 14px;
    border-radius: 6px;
    margin-bottom: 14px;
  }

  .title-card h2 {
    font-size: 10.5pt;
    margin: 0 0 6px 0;
    color: #f8fafc;
    font-weight: 700;
    line-height: 1.35;
  }

  .title-meta {
    font-size: 8.2pt;
    color: #cbd5e1;
  }

  .title-meta span {
    margin-right: 14px;
  }

  .title-meta a {
    color: #93c5fd;
    text-decoration: underline;
  }

  /* Section Headings */
  h3 {
    font-size: 10pt;
    color: #1e3a8a;
    border-bottom: 1.5px solid #cbd5e1;
    padding-bottom: 3px;
    margin-top: 12px;
    margin-bottom: 6px;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.3px;
  }

  p {
    margin-top: 0;
    margin-bottom: 6px;
    text-align: justify;
  }

  /* Tables */
  table.data-table {
    width: 100%;
    border-collapse: collapse;
    margin: 8px 0 12px 0;
    font-size: 8.3pt;
  }

  table.data-table th {
    background-color: #1e3a8a;
    color: #ffffff;
    padding: 5px 8px;
    text-align: left;
    font-weight: 700;
    border: 1px solid #1e3a8a;
  }

  table.data-table td {
    padding: 5px 8px;
    border: 1px solid #cbd5e1;
    vertical-align: top;
  }

  table.data-table tr:nth-child(even) {
    background-color: #f8fafc;
  }

  /* Lists */
  ul, ol {
    margin-top: 0;
    margin-bottom: 6px;
    padding-left: 16px;
  }

  li {
    margin-bottom: 3px;
  }

  /* Callout box */
  .callout {
    background-color: #eff6ff;
    border: 1px solid #bfdbfe;
    border-left: 3px solid #2563eb;
    padding: 7px 10px;
    border-radius: 4px;
    margin: 8px 0;
    font-size: 8.6pt;
  }

  /* Signatures */
  .signature-table {
    width: 100%;
    margin-top: 20px;
    border-collapse: collapse;
  }

  .signature-table td {
    width: 50%;
    text-align: center;
    vertical-align: top;
    padding: 8px 16px;
    font-size: 8.6pt;
  }

  .signature-line {
    border-top: 1px solid #475569;
    margin-top: 36px;
    padding-top: 5px;
    font-weight: 600;
  }

  .page-break {
    page-break-before: always;
  }
</style>
</head>
<body>

  <!-- Header -->
  <table class="header-table">
    <tr>
      <td style="width: 65%;">
        <div class="header-title">Propuesta de Vinculación y Plan Científico</div>
        <div class="header-subtitle">Universidad Militar Nueva Granada — Facultad de Ingeniería (Campus Cajicá)</div>
      </td>
      <td class="badge-container" style="width: 35%;">
        <div class="badge badge-a1">Grupo VOLTA (A1 MinCiencias)</div><br>
        <a href="https://github.com/roncanciovl/burger_delivery" target="_blank"><div class="badge badge-github">GitHub: roncanciovl/burger_delivery</div></a><br>
        <a href="https://doi.org/10.5281/zenodo.21809949" target="_blank"><div class="badge badge-doi">DOI: 10.5281/zenodo.21809949</div></a>
      </td>
    </tr>
  </table>

  <!-- Addressees -->
  <div class="destinatarios-box">
    <table class="destinatarios-table">
      <tr>
        <td style="width: 50%; padding-right: 10px;">
          <strong style="color: #1e3a8a;">Para revisión y aval de:</strong><br>
          <strong>Prof. William Gómez Rivera, M.Sc.</strong><br>
          <a href="mailto:william.gomezr@unimilitar.edu.co" style="font-size: 8.2pt;">william.gomezr@unimilitar.edu.co</a><br>
          Docente Investigador — Grupo VOLTA (A1)<br>
          Programa de Ingeniería Mecatrónica — Campus Cajicá
        </td>
        <td style="width: 50%; border-left: 1px solid #cbd5e1; padding-left: 10px;">
          <strong style="color: #1e3a8a;">Dirigida a:</strong><br>
          <strong>Dr. William Arnulfo Aperador Chaparro</strong><br>
          Líder del Grupo de Investigación VOLTA (Categoría A1)<br>
          Comité de Investigaciones de Mecatrónica — UMNG
        </td>
      </tr>
    </table>
  </div>

  <!-- Title Card -->
  <div class="title-card">
    <h2>Burger-Cell: Framework Abierto y Banco Experimental de Manufactura Flexible Heterogénea en ROS 2 para Manipulación Robótica (Kinova Gen3), Telemetría de Red QoS y Razonamiento Espacial con Inteligencia Artificial</h2>
    <div class="title-meta">
      <span><strong>GitHub:</strong> <a href="https://github.com/roncanciovl/burger_delivery" target="_blank">github.com/roncanciovl/burger_delivery</a></span>
      <span><strong>DOI:</strong> <a href="https://doi.org/10.5281/zenodo.21809949" target="_blank">10.5281/zenodo.21809949</a></span>
      <span><strong>TRL:</strong> 4 - 5</span>
      <span><strong>Sede:</strong> Campus Cajicá</span>
    </div>
  </div>

  <!-- 1. Executive Summary -->
  <h3>1. Resumen Ejecutivo y Justificación Estratégica</h3>
  <p>
    La presente propuesta formaliza la vinculación del suscrito <strong>Docente de Cátedra e Investigador</strong> al <strong>Grupo de Investigación VOLTA (Categoría A1 - MinCiencias)</strong>, aportando una plataforma que <strong>ya cuenta con un proyecto de software abierto y activo en GitHub (<a href="https://github.com/roncanciovl/burger_delivery" target="_blank">github.com/roncanciovl/burger_delivery</a>) en fase de pruebas experimentales de laboratorio</strong>, registrado y respaldado formalmente con identificador digital persistente <strong>DOI en Zenodo (<a href="https://doi.org/10.5281/zenodo.21809949" target="_blank">10.5281/zenodo.21809949</a>)</strong>.
  </p>
  <p>
    La celda articula sinérgicamente la <strong>robótica de manipulación industrial (Kinova Gen3 7-DOF)</strong>, los <strong>sistemas embebidos distribuidos (micro-ROS en ESP32)</strong>, la <strong>telemetría ciber-física en tiempo real</strong> y la <strong>Inteligencia Artificial Multimodal (Embodied AI con Gemini Robotics)</strong>. Al ser un desarrollo en ejecución y con datos experimentales en curso, ofrece al grupo VOLTA un retorno científico inmediato sin tiempos muertos de prototipado inicial.
  </p>

  <!-- 2. Alignment Table -->
  <h3>2. Alineación con las Líneas de Investigación de VOLTA</h3>
  <table class="data-table">
    <thead>
      <tr>
        <th style="width: 38%;">Línea de Investigación VOLTA</th>
        <th style="width: 62%;">Contribución Específica de Burger-Cell</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>1. Diseños Mecatrónicos y Aplicaciones Industriales</strong></td>
        <td>Celda colaborativa de pick & place de alta precisión con cinemática inversa MoveIt 2, servoing visual 3D con AprilTags y control con reducción de jerk inercial.</td>
      </tr>
      <tr>
        <td><strong>2. Sistemas Embebidos y Telemetría Ciber-Física</strong></td>
        <td>Medición y registro continuo a 1 Hz de latencia RTT, Jitter y pérdida de paquetes en buses DDS/RTPS de ROS 2 y micro-ROS (ESP32) sobre WiFi 6.</td>
      </tr>
      <tr>
        <td><strong>3. Inteligencia Artificial Aplicada a Mecatrónica</strong></td>
        <td>Razonamiento espacial 3D <em>zero-shot</em> con <strong>Gemini Robotics VLM</strong> (<code>gemini-robotics-er-1.6-preview</code>) para grasping semántico multi-vista y affordances físicas.</td>
      </tr>
    </tbody>
  </table>

  <!-- Page Break for clean layout -->
  <div class="page-break"></div>

  <!-- 3. Scientific Commitments -->
  <h3>3. Compromisos de Producción Científica para el GrupLAC (2026 - 2027)</h3>
  <p>
    Para maximizar los indicadores de productividad del grupo VOLTA y optimizar los tiempos de ejecución, se focaliza el esfuerzo en <strong>un Artículo Insignia (Flagship Paper) de Categoría Q1/Q2</strong> que fusiona la IA Multimodal con la Telemetría de Red:
  </p>

  <div class="callout">
    <strong>Artículo Principal Propuesto (MinCiencias Tipo A1):</strong><br>
    <em>"VLM-Guided 3D Spatial Reasoning Under Network QoS Uncertainty: A Real-Time Telemetry and Manipulation Benchmark in Heterogeneous ROS 2 Robotic Cells"</em><br>
    <strong>Target Journals:</strong> <em>IEEE Transactions on Automation Science and Engineering (T-ASE)</em>, <em>IEEE Robotics and Automation Letters (RA-L)</em>, <em>MDPI Sensors (Q1/Q2)</em> o <em>Elsevier Mechatronics</em>.
  </div>

  <ul>
    <li><strong>Software Científico Registrado:</strong> Registro formal ante MinCiencias del repositorio <a href="https://github.com/roncanciovl/burger_delivery" target="_blank">Burger-Cell en GitHub</a> con DOI persistente <a href="https://doi.org/10.5281/zenodo.21809949" target="_blank">10.5281/zenodo.21809949</a>.</li>
    <li><strong>Dataset Abierto Unificado:</strong> Publicación en <em>IEEE DataPort / Zenodo</em> correlacionando métricas DDS (1 Hz), latencias VLM y precisión cinemática articular (RMSE).</li>
    <li><strong>Formación de Talento en Campus Cajicá:</strong> Dirección de 1 a 2 trabajos de grado para la <strong>Maestría en Ingeniería Mecatrónica</strong> y talleres para el semillero de robótica.</li>
  </ul>

  <!-- 4. MVP Demonstration -->
  <h3>4. Demostración Tecnológica en Laboratorio (MVP Campus Cajicá)</h3>
  <p>
    Se ofrece una sesión de demostración funcional (10 minutos) en los laboratorios de Mecatrónica del Campus Nueva Granada:
  </p>
  <ul>
    <li><strong>Demostrador 1 (Pipeline Unificado IA + Cinemática):</strong> Captura multi-vista, inferencia espacial con Gemini Robotics y ejecución de trayectoria MoveIt 2 con el Kinova Gen3.</li>
    <li><strong>Demostrador 2 (Telemetría de Red en Vivo):</strong> Monitoreo en tiempo real (<code>http://localhost:8080</code>) de ráfagas DDS y exportación de telemetría a dataset CSV.</li>
    <li><strong>Demostrador 3 (Análisis Estadístico Automatizado):</strong> Ejecución de <code>analyze_telemetry_benchmark.py</code> generando figuras vectoriales para publicación.</li>
  </ul>

  <!-- 5. Requested Resources -->
  <h3>5. Recursos Solicitados y Apoyo Institucional</h3>
  <p>
    Se solicita el respaldo y aval del grupo VOLTA para gestionar ante la Vicerrectoría de Investigaciones (VRI) los siguientes rubros:
  </p>
  <ol>
    <li><strong>Hardware Embebido:</strong> Adquisición de tarjetas de desarrollo <strong>ESP32 / ESP32-S3 (x3 unidades)</strong> para nodos de micro-ROS y sensores, además de una cámara de profundidad RGB-D (Intel RealSense) para el efector final.</li>
    <li><strong>Fondo de Publicación (APC Open Access):</strong> Cobertura de cargos de procesamiento para el Flagship Paper Q1 al ser aceptado.</li>
    <li><strong>Créditos de Cómputo e Inferencia IA:</strong> Asignación de tokens API para inferencia en la nube con Gemini Robotics durante los ensayos.</li>
    <li><strong>Vinculación Contractual de Investigación (Modalidad Docente de Cátedra):</strong> Gestión institucional para formalizar un contrato de investigación adicional / remuneración por horas de proyecto (4 a 6 horas semanales dedicadas a investigación) con cargo al proyecto o fondos de fortalecimiento del grupo VOLTA.</li>
    <li><strong>Estudiante Auxiliar:</strong> Asignación de 1 auxiliar de investigación de pregrado o maestría para apoyo en calibraciones de laboratorio en Campus Cajicá.</li>
  </ol>

  <!-- Signatures -->
  <table class="signature-table">
    <tr>
      <td>
        <div class="signature-line">
          <strong>Prof. Henry Roncancio</strong><br>
          Docente de Cátedra / Investigador<br>
          Programa de Ingeniería Mecatrónica<br>
          Campus Nueva Granada — UMNG
        </div>
      </td>
      <td>
        <div class="signature-line">
          <strong>Prof. William Gómez Rivera, M.Sc.</strong><br>
          Docente Investigador — Grupo VOLTA (A1)<br>
          Programa de Ingeniería Mecatrónica<br>
          Campus Nueva Granada — UMNG
        </div>
      </td>
    </tr>
  </table>

</body>
</html>
"""

def generate_pdf():
    output_pdf_path = "/home/roncanciovl/ros2_ws/src/burger_delivery/docs/research/PROPUESTA_VINCULACION_GRUPO_VOLTA.pdf"
    print(f"Generating PDF with WeasyPrint: {output_pdf_path} ...")
    html_obj = weasyprint.HTML(string=HTML_CONTENT)
    html_obj.write_pdf(output_pdf_path)
    print(f"PDF successfully created at: {output_pdf_path} (Size: {os.path.getsize(output_pdf_path)} bytes)")

if __name__ == "__main__":
    generate_pdf()
