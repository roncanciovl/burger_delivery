#!/usr/bin/env python3
"""Compara o aplica el manifiesto ROS mediante la interfaz abierta de SGDE.

No usa credenciales propias ni escribe directamente en Firestore. Se conecta a
una pestaña de Chrome ya autenticada mediante Chrome DevTools Protocol (CDP).
El modo predeterminado es VISTA_PREVIA; APLICAR requiere una confirmación
literal y todos los RAE previamente cargados por un administrador.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

from playwright.sync_api import Page, sync_playwright


LEVEL_FIELDS = ("n5", "n4", "n3", "n2", "n1")


class SyncError(RuntimeError):
    """La sincronización no puede continuar de forma segura."""


def read_manifest(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("mode") != "VISTA_PREVIA":
        raise SyncError("El manifiesto debe haberse generado en VISTA_PREVIA.")
    return data


def read_student_count(path: Path | None) -> int | None:
    if path is None:
        return None
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream, delimiter=";"))
    required = {"CODIGO", "APELLIDOS", "NOMBRES", "CORREO"}
    if not rows or not required.issubset(rows[0]):
        raise SyncError("El CSV estudiantil no tiene el esquema esperado.")
    if len({row["CODIGO"].strip() for row in rows}) != len(rows):
        raise SyncError("El CSV estudiantil contiene códigos duplicados.")
    return len(rows)


def find_sgde_page(browser: Any) -> Page:
    pages = [page for context in browser.contexts for page in context.pages]
    matches = [page for page in pages if "sgde-umng-mecat.web.app" in page.url]
    if len(matches) != 1:
        raise SyncError(f"Se esperaba una pestaña SGDE abierta y se encontraron {len(matches)}.")
    return matches[0]


def safe_snapshot(page: Page, course_key: str) -> dict[str, Any]:
    return page.evaluate(
        """courseKey => {
          if(typeof currentUser==='undefined' || !currentUser) return {authenticated:false};
          const mine=(typeof getData==='function' ? getData()?.[currentUser.docenteKey] : null)||{};
          const cd=mine[courseKey];
          if(!cd) return {authenticated:true,hasTeacherLink:!!currentUser.docenteKey,courseFound:false};
          const asig=courseKey.split('|')[0];
          return {
            authenticated:true,
            hasTeacherLink:!!currentUser.docenteKey,
            courseFound:true,
            studentCount:(getSharedStudents(courseKey)||[]).length,
            raes:(getRaes(asig)||[]).map(r=>({id:String(r.id),so:String(r.so),rae:String(r.rae)})),
            hasClassroomCourse:!!cd._clsCourseId,
            instruments:(cd.instrumentos||[]).map(i=>({
              id:i.id,name:i.nombre||'',type:i.tipo||'',weight:+i.peso||0,
              criteria:(i.criterios||[]).map(c=>({
                id:c.id,name:c.nombre||'',weight:+c.peso||0,raeId:String(c.raeId||''),
                n5:c.n5||'',n4:c.n4||'',n3:c.n3||'',n2:c.n2||'',n1:c.n1||''
              }))
            }))
          };
        }""",
        course_key,
    )


def desired_criterion(raw: dict[str, Any]) -> dict[str, Any]:
    return {
        "name": raw["name"],
        "weight": float(raw["weight"]),
        "raeId": str(raw["rae_id"]),
        "n5": raw["levels"]["N5"],
        "n4": raw["levels"]["N4"],
        "n3": raw["levels"]["N3"],
        "n2": raw["levels"]["N2"],
        "n1": raw["levels"]["N1"],
    }


def same_criterion(existing: dict[str, Any], desired: dict[str, Any]) -> bool:
    return all(existing.get(key) == desired.get(key) for key in desired)


def build_diff(manifest: dict[str, Any], snapshot: dict[str, Any], student_count: int | None) -> dict[str, Any]:
    if not snapshot.get("authenticated"):
        raise SyncError("La pestaña SGDE no está autenticada.")
    if not snapshot.get("hasTeacherLink"):
        raise SyncError("La cuenta no está vinculada con un perfil docente.")
    if not snapshot.get("courseFound"):
        raise SyncError("No se encontró el curso objetivo en el perfil docente.")

    available_rae_ids = {item["id"] for item in snapshot["raes"]}
    required_rae_ids = {
        str(criterion["rae_id"])
        for instrument in manifest["instruments"]
        for criterion in instrument["criteria"]
    }
    missing_rae_ids = sorted(required_rae_ids - available_rae_ids)

    current_instruments = {item["name"]: item for item in snapshot["instruments"]}
    missing_instruments: list[str] = []
    create_count = update_count = unchanged_count = 0
    extra_criteria = 0
    criterion_actions: list[dict[str, Any]] = []

    for desired_instrument in manifest["instruments"]:
        current = current_instruments.get(desired_instrument["name"])
        if not current:
            missing_instruments.append(desired_instrument["name"])
            continue
        current_by_name = {item["name"]: item for item in current["criteria"]}
        desired_names = {item["name"] for item in desired_instrument["criteria"]}
        extra_criteria += len(set(current_by_name) - desired_names)
        for raw in desired_instrument["criteria"]:
            desired = desired_criterion(raw)
            existing = current_by_name.get(desired["name"])
            if existing is None:
                action = "CREATE"
                create_count += 1
            elif same_criterion(existing, desired):
                action = "UNCHANGED"
                unchanged_count += 1
            else:
                action = "UPDATE"
                update_count += 1
            criterion_actions.append(
                {
                    "instrument": desired_instrument["name"],
                    "instrument_id": current["id"],
                    "criterion": desired,
                    "existing_id": existing["id"] if existing else None,
                    "action": action,
                }
            )

    desired_students = student_count
    current_students = snapshot["studentCount"]
    return {
        "ready": not missing_rae_ids and not missing_instruments,
        "missing_rae_ids": missing_rae_ids,
        "missing_instruments": missing_instruments,
        "criteria": {
            "create": create_count,
            "update": update_count,
            "unchanged": unchanged_count,
            "extra_not_deleted": extra_criteria,
        },
        "students": {
            "current": current_students,
            "source": desired_students,
            "import_requested": student_count is not None,
        },
        "classroom_link_present": snapshot["hasClassroomCourse"],
        "criterion_actions": criterion_actions,
    }


def import_students(page: Page, csv_path: Path, expected_count: int) -> None:
    page.evaluate("() => openModalImport()")
    page.locator("#imp-tab-csv").click()
    page.locator("#imp-csv-file").set_input_files(csv_path)
    page.wait_for_timeout(500)
    detected = page.evaluate(
        "() => (typeof _impCSVLines!=='undefined' && Array.isArray(_impCSVLines)) ? _impCSVLines.length : -1"
    )
    if detected != expected_count:
        page.evaluate("() => closeModal('modalImport')")
        raise SyncError(f"SGDE detectó {detected} estudiantes; se esperaban {expected_count}.")
    page.evaluate("() => importStudents()")


def apply_criteria(page: Page, course_key: str, actions: list[dict[str, Any]]) -> None:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for action in actions:
        if action["action"] != "UNCHANGED":
            grouped.setdefault(action["instrument"], []).append(action)

    for instrument_name, instrument_actions in grouped.items():
        instrument_id = instrument_actions[0]["instrument_id"]
        page.evaluate(
            """([courseKey,instrumentId]) => {
              openCourse(courseKey);
              openInstrumentDetail(instrumentId);
            }""",
            [course_key, instrument_id],
        )
        for action in instrument_actions:
            criterion = action["criterion"]
            if action["action"] == "UPDATE":
                page.evaluate("id => openEditCriterio(id)", action["existing_id"])
            else:
                page.evaluate("() => openModalCriterio()")
            page.locator("#mc-nombre").fill(criterion["name"])
            page.locator("#mc-peso").fill(str(criterion["weight"]))
            page.locator("#mc-rae").select_option(criterion["raeId"])
            for field in LEVEL_FIELDS:
                page.locator(f"#mc-{field}").fill(criterion[field])
            page.evaluate("() => saveCriterio()")


def verify_applied(manifest: dict[str, Any], snapshot: dict[str, Any], student_count: int | None) -> None:
    diff = build_diff(manifest, snapshot, student_count)
    if diff["criteria"]["create"] or diff["criteria"]["update"]:
        raise SyncError(f"La verificación detectó criterios pendientes: {diff['criteria']}.")
    if student_count is not None and snapshot["studentCount"] < student_count:
        raise SyncError(
            f"La verificación encontró {snapshot['studentCount']} estudiantes; se esperaban al menos {student_count}."
        )


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Sincroniza el manifiesto ROS con la pestaña SGDE.")
    parser.add_argument(
        "--manifest",
        type=Path,
        default=script_dir / "preview/ros_corte1_zubatronic.preview.json",
    )
    parser.add_argument("--student-csv", type=Path)
    parser.add_argument("--cdp-url", default="http://127.0.0.1:9222")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if args.apply and args.confirm != "APLICAR":
        parser.error("APLICAR requiere --confirm APLICAR.")

    manifest = read_manifest(args.manifest)
    student_count = read_student_count(args.student_csv)
    course_key = manifest["course"]["sgde_course_key"]

    with sync_playwright() as playwright:
        browser = playwright.chromium.connect_over_cdp(args.cdp_url)
        page = find_sgde_page(browser)
        page.on("dialog", lambda dialog: dialog.accept())
        snapshot = safe_snapshot(page, course_key)
        diff = build_diff(manifest, snapshot, student_count)

        public_diff = {key: value for key, value in diff.items() if key != "criterion_actions"}
        print(json.dumps(public_diff, ensure_ascii=False, indent=2))
        if not diff["ready"]:
            print("BLOQUEADO: faltan prerrequisitos; no se realizó ninguna escritura.")
            return 0
        if not args.apply:
            print("VISTA_PREVIA completa. Use --apply --confirm APLICAR sólo después de revisar el resultado.")
            return 0

        if args.student_csv is not None:
            import_students(page, args.student_csv, student_count or 0)
        apply_criteria(page, course_key, diff["criterion_actions"])
        final_snapshot = safe_snapshot(page, course_key)
        verify_applied(manifest, final_snapshot, student_count)
        print(
            f"APLICADO Y VERIFICADO: {final_snapshot['studentCount']} estudiantes y "
            f"{sum(len(i['criteria']) for i in final_snapshot['instruments'])} criterios."
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, json.JSONDecodeError, SyncError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
