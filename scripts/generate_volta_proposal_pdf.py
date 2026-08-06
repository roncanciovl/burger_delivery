#!/usr/bin/env python3
"""
Script to generate a high-impact, professional academic PDF proposal for Grupo VOLTA (UMNG).
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
      content: "Propuesta de Vinculación Científica — Grupo VOLTA (Categoría A1 - MinCiencias)";
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
    padding-bottom: 8px;
    margin-bottom: 12px;
  }

  .header-title {
    font-size: 13.5pt;
    font-weight: 800;
    color: #1e3a8a;
    text-transform: uppercase;
    letter-spacing: 0.4px;
    margin: 0;
  }

  .header-subtitle {
    font-size: 8.8pt;
    color: #475569;
    margin-top: 2px;
    font-weight: 600;
  }

  .badge-container {
    text-align: right;
  }

  .badge {
    display: inline-block;
    padding: 2.5px 7px;
    border-radius: 4px;
    font-size: 7.5pt;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.3px;
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
    margin-top: 3px;
  }

  .badge-github {
    background-color: #f1f5f9;
    color: #0f172a;
    border: 1px solid #94a3b8;
    margin-top: 3px;
  }

  /* Addressees box */
  .destinatarios-box {
    background-color: #f8fafc;
    border: 1px solid #e2e8f0;
    border-left: 4px solid #1e3a8a;
    border-radius: 4px;
    padding: 8px 12px;
    margin-bottom: 12px;
  }

  .destinatarios-table {
    width: 100%;
  }

  .destinatarios-table td {
    vertical-align: top;
    font-size: 8.6pt;
  }

  /* Project Title Box */
  .title-card {
    background: linear-gradient(135deg, #1e3a8a 0%, #1e40af 100%);
    color: #ffffff;
    padding: 10px 13px;
    border-radius: 5px;
    margin-bottom: 12px;
  }

  .title-card h2 {
    font-size: 10pt;
    margin: 0 0 5px 0;
    color: #f8fafc;
    font-weight: 700;
    line-height: 1.32;
  }

  .title-meta {
    font-size: 8.0pt;
    color: #cbd5e1;
  }

  .title-meta span {
    margin-right: 12px;
  }

  .title-meta a {
    color: #93c5fd;
    text-decoration: underline;
  }

  /* Section Headings */
  h3 {
    font-size: 9.6pt;
    color: #1e3a8a;
    border-bottom: 1.5px solid #cbd5e1;
    padding-bottom: 2px;
    margin-top: 10px;
    margin-bottom: 5px;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.3px;
  }

  p {
    margin-top: 0;
    margin-bottom: 5px;
    text-align: justify;
  }

  /* Tables */
  table.data-table {
    width: 100%;
    border-collapse: collapse;
    margin: 6px 0 10px 0;
    font-size: 8.1pt;
  }

  table.data-table th {
    background-color: #1e3a8a;
    color: #ffffff;
    padding: 5px 7px;
    text-align: left;
    font-weight: 700;
    border: 1px solid #1e3a8a;
  }

  table.data-table td {
    padding: 4px 7px;
    border: 1px solid #cbd5e1;
    vertical-align: top;
  }

  table.data-table tr:nth-child(even) {
    background-color: #f8fafc;
  }

  /* Lists */
  ul, ol {
    margin-top: 0;
    margin-bottom: 5px;
    padding-left: 15px;
  }

  li {
    margin-bottom: 2.5px;
  }

  /* Callout box */
  .callout {
    background-color: #eff6ff;
    border: 1px solid #bfdbfe;
    border-left: 3px solid #2563eb;
    padding: 6px 9px;
    border-radius: 4px;
    margin: 7px 0;
    font-size: 8.4pt;
  }

  /* Signatures */
  .signature-table {
    width: 100%;
    margin-top: 18px;
    border-collapse: collapse;
  }

  .signature-table td {
    width: 50%;
    text-align: center;
    vertical-align: top;
    padding: 6px 14px;
    font-size: 8.4pt;
  }

  .signature-line {
    border-top: 1px solid #475569;
    margin-top: 32px;
    padding-top: 4px;
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
        <div class="header-subtitle">Universidad Militar Nueva Granada — Facultad de Ingeniería (Campus Nueva Granada)</div>
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
          <strong style="color: #1e3a8a;">Para consideración previa de:</strong><br>
          <strong>Dr. William Gómez Rivera, Ph.D.</strong><br>
          <a href="mailto:william.gomezr@unimilitar.edu.co" style="font-size: 8.0pt;">william.gomezr@unimilitar.edu.co</a><br>
          Docente Investigador — Grupo VOLTA (Categoría A1)<br>
          Programa de Ingeniería Mecatrónica — Campus Nueva Granada
        </td>
        <td style="width: 50%; border-left: 1px solid #cbd5e1; padding-left: 10px;">
          <strong style="color: #1e3a8a;">Dirigida a:</strong><br>
          <strong>Dr. William Arnulfo Aperador Chaparro</strong><br>
          Líder del Grupo de Investigación VOLTA (Categoría A1)<br>
          Comité de Investigaciones — Facultad de Ingeniería
        </td>
      </tr>
    </table>
  </div>

  <!-- Title Card -->
  <div class="title-card">
    <h2>Burger-Cell: Framework Abierto y Banco Experimental de Manufactura Flexible Heterogénea en ROS 2 para Manipulación Robótica (Kinova Gen3), Telemetría de Red QoS y Razonamiento Espacial con Inteligencia Artificial</h2>
    <div class="title-meta">
      <span><strong>GitHub:</strong> <a href="https://github.com/roncanciovl/burger_delivery" target="_blank">github.com/roncanciovl/burger_delivery</a></span>
      <span><strong>DOI Zenodo:</strong> <a href="https://doi.org/10.5281/zenodo.21809949" target="_blank">10.5281/zenodo.21809949</a></span>
      <span><strong>TRL:</strong> 4 - 5</span>
      <span><strong>Sede:</strong> Campus Nueva Granada</span>
    </div>
  </div>

  <!-- 1. Executive Summary -->
  <h3>1. Resumen Ejecutivo y Justificación Estratégica</h3>
  <p>
    La presente propuesta formaliza la postulación del suscrito <strong>Docente e Investigador</strong> para vincularse al <strong>Grupo de Investigación VOLTA (Categoría A1 - MinCiencias)</strong>, aportando una plataforma de investigación aplicada que <strong>ya cuenta con un proyecto de software funcional y abierto en GitHub (<a href="https://github.com/roncanciovl/burger_delivery" target="_blank">github.com/roncanciovl/burger_delivery</a>)</strong>, validado experimentalmente en laboratorio y registrado formalmente en <strong>DataCite / Zenodo con DOI persistente (<a href="https://doi.org/10.5281/zenodo.21809949" target="_blank">10.5281/zenodo.21809949</a>)</strong>.
  </p>
  <p>
    La plataforma articula de forma sinérgica la <strong>robótica de manipulación industrial (Kinova Gen3 7-DOF)</strong>, los <strong>sistemas embebidos distribuidos (micro-ROS en ESP32)</strong>, la <strong>telemetría ciber-física en tiempo real</strong> y la <strong>Inteligencia Artificial Multimodal (Embodied AI con Gemini Robotics)</strong>. Al tratarse de una celda con protocolos de medición terminados y toma de datos activa, ofrece a VOLTA un retorno científico inmediato sin tiempos muertos de desarrollo inicial.
  </p>

  <!-- 2. Alignment Table -->
  <h3>2. Articulación Estratégica con las Líneas de Investigación de VOLTA</h3>
  <p>
    Frente a las tres líneas oficiales de VOLTA (<em>1. Energías Renovables</em>, <em>2. Diseños Mecatrónicos</em>, <em>3. Materiales y Procesos de Manufactura</em>), el proyecto se integra en el núcleo de la <strong>Línea 2</strong> e incorpora dos extensiones de frontera tecnológica ausentes hoy en su GrupLAC:
  </p>
  <table class="data-table">
    <thead>
      <tr>
        <th style="width: 32%;">Línea Oficial VOLTA</th>
        <th style="width: 25%;">Estado en VOLTA</th>
        <th style="width: 43%;">Aporte y Extensión con Burger-Cell</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>1. Energías Renovables</strong></td>
        <td>Consolidada</td>
        <td>—</td>
      </tr>
      <tr>
        <td><strong>2. Diseños Mecatrónicos</strong></td>
        <td><strong>Núcleo de la Propuesta</strong></td>
        <td><strong>Aporte Directo:</strong> Celda colaborativa de pick & place de alta precisión con manipulador Kinova Gen3 (7-DOF), cinemática inversa en MoveIt 2, servoing visual 3D (AprilTags) y control de trayectoria suave (<em>jerk reduction</em>).</td>
      </tr>
      <tr>
        <td><em>Extensión 2.A (Nueva Capacidad)</em></td>
        <td><em>Ausente en GrupLAC</em></td>
        <td><strong>Telemetría Ciber-Física y QoS en Tiempo Real:</strong> Medición y registro a 1 Hz de latencia RTT, Jitter y pérdida de paquetes en buses DDS/RTPS de ROS 2 y micro-ROS (ESP32) sobre WiFi 6.</td>
      </tr>
      <tr>
        <td><em>Extensión 2.B (Nueva Capacidad)</em></td>
        <td><em>Ausente en GrupLAC</em></td>
        <td><strong>Inteligencia Artificial Física (Embodied AI):</strong> Razonamiento espacial 3D <em>zero-shot</em> con Modelos de Visión-Lenguaje (<strong>Gemini Robotics <code>gemini-robotics-er-1.6-preview</code></strong>) para grasping semántico multi-vista y affordances.</td>
      </tr>
      <tr>
        <td><strong>3. Materiales y Procesos de Manufactura</strong></td>
        <td>Consolidada</td>
        <td>Potencial sinergia futura en caracterización de agarre sobre probetas manufacturadas.</td>
      </tr>
    </tbody>
  </table>

  <!-- Page Break for clean layout -->
  <div class="page-break"></div>

  <!-- 3. Scientific Commitments -->
  <h3>3. Compromisos de Producción Científica para el GrupLAC (2026 - 2027)</h3>
  <p>
    Para maximizar los indicadores de productividad de VOLTA y focalizar los esfuerzos, se plantea <strong>un Artículo Insignia (Flagship Paper) de Categoría Q1/Q2</strong> que fusiona la IA Multimodal con la Telemetría de Red:
  </p>

  <div class="callout">
    <strong>Artículo Principal Propuesto (MinCiencias Tipo A1):</strong><br>
    <em>"VLM-Guided 3D Spatial Reasoning Under Network QoS Uncertainty: A Real-Time Telemetry and Manipulation Benchmark in Heterogeneous ROS 2 Robotic Cells"</em><br>
    <strong>Target Journals:</strong> <em>IEEE Transactions on Automation Science and Engineering (T-ASE)</em>, <em>IEEE Robotics and Automation Letters (RA-L)</em>, <em>MDPI Sensors (Q1/Q2)</em> o <em>Elsevier Mechatronics</em>.
  </div>

  <ul>
    <li><strong>Software Científico Registrado:</strong> Registro formal ante MinCiencias del repositorio <a href="https://github.com/roncanciovl/burger_delivery" target="_blank">Burger-Cell en GitHub</a> con DOI persistente <a href="https://doi.org/10.5281/zenodo.21809949" target="_blank">10.5281/zenodo.21809949</a>.</li>
    <li><strong>Dataset Abierto Unificado:</strong> Publicación en <em>IEEE DataPort / Zenodo</em> correlacionando métricas DDS (1 Hz), latencias VLM y precisión cinemática articular (RMSE).</li>
    <li><strong>Formación de Talento Humano:</strong> Dirección de 1 a 2 trabajos de grado para la <strong>Maestría en Ingeniería Mecatrónica</strong> y vinculación de estudiantes del semillero de robótica.</li>
  </ul>

  <!-- 4. MVP Demonstration -->
  <h3>4. Demostración Tecnológica en Laboratorio (MVP Campus Nueva Granada)</h3>
  <p>
    Se ofrece una sesión de demostración funcional (10 minutos) en los laboratorios de la Facultad de Ingeniería:
  </p>
  <ul>
    <li><strong>Demostrador 1 (Pipeline Unificado IA + Cinemática):</strong> Captura multi-vista, inferencia espacial con Gemini Robotics y ejecución de trayectoria MoveIt 2 con el Kinova Gen3.</li>
    <li><strong>Demostrador 2 (Telemetría de Red en Vivo):</strong> Monitoreo en tiempo real (<code>http://localhost:8080</code>) de ráfagas DDS y exportación de telemetría a dataset CSV.</li>
    <li><strong>Demostrador 3 (Análisis Estadístico Automatizado):</strong> Ejecución de <code>analyze_telemetry_benchmark.py</code> generando figuras vectoriales para publicación.</li>
  </ul>

  <!-- 5. Requested Resources -->
  <h3>5. Recursos y Apoyo Solicitado al Grupo VOLTA / VRI</h3>
  <p>
    Para consolidar la infraestructura experimental y acelerar los tiempos de publicación, se solicita el respaldo de VOLTA para tramitar ante la Vicerrectoría de Investigaciones (VRI):
  </p>
  <ol>
    <li><strong>Equipamiento Embebido y Sensores:</strong> Adquisición de tarjetas de desarrollo <strong>ESP32 / ESP32-S3 (x3 unidades)</strong> para nodos de micro-ROS y una cámara de profundidad RGB-D (Intel RealSense) para el manipulador Kinova Gen3.</li>
    <li><strong>Fondo de Publicación (APC Open Access):</strong> Respaldo para la cobertura de cargos de procesamiento para el Flagship Paper Q1 al momento de su aceptación.</li>
    <li><strong>Créditos de Cómputo e Inferencia IA:</strong> Soporte de tokens API para inferencia en la nube con Gemini Robotics durante las campañas experimentales.</li>
    <li><strong>Articulación de Estudiantes:</strong> Asignación de 1 estudiante auxiliar de pregrado o maestría para soporte en calibración de hardware y pruebas de laboratorio.</li>
  </ol>

  <!-- Signatures -->
  <table class="signature-table">
    <tr>
      <td>
        <strong style="color: #1e3a8a;">Postula:</strong>
        <div class="signature-line">
          <strong>Prof. Henry Antonio Roncancio Velandia</strong><br>
          Docente e Investigador — Programa de Ingeniería Mecatrónica<br>
          Facultad de Ingeniería — Campus Nueva Granada<br>
          <span style="font-size: 7.8pt; color: #475569;">ORCID: 0009-0009-9954-9813 | henry.roncancio@unimilitar.edu.co</span>
        </div>
      </td>
      <td>
        <strong style="color: #1e3a8a;">Visto Bueno / Aval (Grupo VOLTA):</strong>
        <div class="signature-line">
          <strong>Dr. William Gómez Rivera, Ph.D.</strong><br>
          Docente Investigador — Grupo de Investigación VOLTA (A1)<br>
          Programa de Ingeniería Mecatrónica — Campus Nueva Granada<br>
          <span style="font-size: 7.8pt; color: #475569;">william.gomezr@unimilitar.edu.co</span>
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
