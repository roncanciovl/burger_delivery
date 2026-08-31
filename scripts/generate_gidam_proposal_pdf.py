#!/usr/bin/env python3
"""Genera la propuesta vigente de vinculación y aval académico para GIDAM."""

from pathlib import Path

import weasyprint


HTML_CONTENT = """<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<style>
  @page {
    size: A4;
    margin: 16mm 16mm 15mm;
    @bottom-left { content: "Propuesta de vinculación y aval académico — GIDAM"; font: 8pt sans-serif; color: #64748b; }
    @bottom-right { content: "Página " counter(page) " de " counter(pages); font: 8pt sans-serif; color: #64748b; }
  }
  body { font-family: Arial, sans-serif; color: #172033; font-size: 9.2pt; line-height: 1.43; }
  h1 { color: #173b7a; font-size: 17pt; margin: 0 0 4px; }
  h2 { color: #173b7a; font-size: 11pt; border-bottom: 1px solid #8ca9d8; padding-bottom: 3px; margin: 16px 0 7px; }
  p { margin: 0 0 8px; text-align: justify; }
  a { color: #0c5ca8; text-decoration: none; }
  .header { background: #edf4ff; border-left: 5px solid #1d4f91; padding: 16px 18px; border-radius: 5px; }
  .meta { display: table; width: 100%; margin-top: 9px; font-size: 8.6pt; }
  .meta div { display: table-row; }
  .meta span { display: table-cell; padding: 2px 10px 2px 0; }
  .status { margin: 12px 0; padding: 10px 12px; border-left: 4px solid #c47a00; background: #fff8e7; }
  table { width: 100%; border-collapse: collapse; margin: 8px 0; font-size: 8.3pt; }
  th { background: #173b7a; color: #fff; text-align: left; padding: 6px 7px; }
  td { border: 1px solid #c8d2e2; padding: 6px 7px; vertical-align: top; }
  tr:nth-child(even) td { background: #f7f9fc; }
  ul { margin: 5px 0 8px; padding-left: 20px; }
  li { margin: 4px 0; }
  .signature { margin-top: 26px; padding-top: 8px; border-top: 1px solid #46566f; width: 48%; }
</style>
</head>
<body>
  <section class="header">
    <h1>Propuesta de vinculación y aval académico</h1>
    <strong>Grupo de Investigación GIDAM — Categoría A, MinCiencias</strong>
    <div class="meta">
      <div><span><strong>Institución:</strong> Universidad Militar Nueva Granada</span><span><strong>Proyecto:</strong> Burger-Cell / burger_delivery</span></div>
      <div><span><strong>Proponente:</strong> Henry Antonio Roncancio Velandia</span><span><strong>ORCID:</strong> 0009-0009-9954-9813</span></div>
      <div><span><strong>DOI de concepto:</strong> 10.5281/zenodo.21809949</span><span><strong>Estado:</strong> solicitud en trámite</span></div>
    </div>
  </section>

  <div class="status"><strong>Estado al 31-ago-2026:</strong> la solicitud fue remitida a William Gómez Rivera y escalada al director de GIDAM. Permanece pendiente de decisión y formalización; no equivale todavía a aceptación, aval GrupLAC ni registro CvLAC.</div>

  <h2>1. Solicitud</h2>
  <p>Se solicita concepto sobre el encaje científico de Burger-Cell con GIDAM y, si es favorable, orientación para formalizar su vinculación y aval académico. Esta solicitud no compromete recursos, contratación, firma de terceros ni representación institucional.</p>

  <h2>2. Estado verificable del proyecto</h2>
  <p>Burger-Cell es un proyecto de software ROS 2 en existencia documentada. El snapshot v1.0.0 está preservado en Zenodo con DOI de versión <a href="https://doi.org/10.5281/zenodo.21809950">10.5281/zenodo.21809950</a>, licencia Apache-2.0 y metadatos de citación.</p>
  <p>La auditoría del 31-ago-2026 verificó un paquete ROS 2 compilable, cuatro modelos URDF válidos y 224 referencias de malla resueltas. Las pruebas ejecutables, la trazabilidad de requisitos, la titularidad y la validación experimental aún deben consolidarse antes de presentar el proyecto como producto institucional cerrado.</p>

  <h2>3. Encaje propuesto con GIDAM</h2>
  <p>El encaje se formula frente a los tópicos publicados por GIDAM. Sus denominaciones formales en GrupLAC se confirmarán con el grupo antes de declararlas como líneas oficiales.</p>
  <table>
    <tr><th>Tópico publicado</th><th>Aporte propuesto</th><th>Estado de evidencia</th></tr>
    <tr><td>Percepción y procesamiento de señales</td><td>Evaluación de percepción visual en una celda de manipulación.</td><td>Trayectoria de Henry en percepción robótica; integración de producción pendiente.</td></tr>
    <tr><td>Control y programación de arquitecturas</td><td>ROS 2, modelos cinemáticos y experimentos de telemetría QoS.</td><td>URDF y documentación auditados; capacidades integradas aún en construcción.</td></tr>
    <tr><td>Aprendizaje e interacción con el entorno</td><td>Hipótesis experimental sobre VLM, telemetría y manipulación colaborativa.</td><td>Propuesta de investigación, no resultado experimental cerrado.</td></tr>
  </table>

  <h2>4. Productos y compromisos propuestos</h2>
  <ul>
    <li>Consolidar una versión reproducible con pruebas, documentación y evidencia experimental verificable.</li>
    <li>Preparar el registro de software en CvLAC cuando existan aceptación, soportes y autorización del titular.</li>
    <li>Gestionar por separado la asociación o aval del producto en GrupLAC, sin confundir el DOI con dicho aval.</li>
    <li>Formular artículos, datasets o proyectos de grado solo con el procedimiento y las aprobaciones institucionales aplicables.</li>
  </ul>

  <h2>5. Seguimiento</h2>
  <table>
    <tr><th>Hito</th><th>Estado</th></tr>
    <tr><td>Solicitud remitida y escalada a dirección de GIDAM</td><td>En trámite</td></tr>
    <tr><td>Decisión de vinculación/aval</td><td>Pendiente</td></tr>
    <tr><td>Registro CvLAC del software</td><td>Pendiente, sujeto a soportes y autorización</td></tr>
    <tr><td>Asociación o aval en GrupLAC</td><td>Pendiente de decisión y constancia verificable</td></tr>
  </table>

  <div class="signature"><strong>Henry Antonio Roncancio Velandia</strong><br>Profesor de Cátedra — Categoría Asociado<br>Programa de Ingeniería Mecatrónica, UMNG</div>
</body>
</html>
"""


def main() -> None:
    output = Path(__file__).resolve().parents[1] / "docs/research/PROPUESTA_VINCULACION_GIDAM.pdf"
    weasyprint.HTML(string=HTML_CONTENT).write_pdf(output)
    print(f"PDF generado: {output}")


if __name__ == "__main__":
    main()
