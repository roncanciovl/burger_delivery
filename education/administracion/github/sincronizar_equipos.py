#!/usr/bin/env python3
"""Sincroniza la hoja privada Equipos con GitHub Teams y repositorios.

VISTA_PREVIA es el modo predeterminado. APLICAR requiere una confirmación
explícita y sólo agrega/actualiza accesos; nunca retira integrantes.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable
from urllib.parse import quote


DEFAULT_ORG = "umng-mecatronica-ros"
DEFAULT_PERIOD = "2026-2"
TEAM_RE = re.compile(r"^equipo-(0[1-9]|10)$")
USER_RE = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9-]{0,37}[A-Za-z0-9])?$")
REQUIRED_COLUMNS = {
    "ACTIVO",
    "EQUIPO_ID",
    "CODIGO",
    "CORREO",
    "GITHUB_USER",
    "ENTREGANTE",
    "REPOSITORIO",
    "GITHUB_TEAM",
}


class SyncError(RuntimeError):
    """Error seguro y comunicable al usuario."""


@dataclass(frozen=True)
class Member:
    row: int
    team: str
    code: str
    email: str
    github_user: str
    submitter: bool
    repository: str
    github_team: str


@dataclass(frozen=True)
class Action:
    kind: str
    target: str
    detail: str


def normalize_header(value: str) -> str:
    return str(value or "").strip().upper()


def is_yes(value: str) -> bool:
    return str(value or "").strip().upper() == "SI"


def load_members(path: Path) -> list[Member]:
    if not path.is_file():
        raise SyncError(f"No existe el CSV: {path}")

    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise SyncError("El CSV no contiene encabezados.")
        header_map = {name: normalize_header(name) for name in reader.fieldnames}
        available = set(header_map.values())
        missing = sorted(REQUIRED_COLUMNS - available)
        if missing:
            raise SyncError("Faltan columnas requeridas: " + ", ".join(missing))

        members: list[Member] = []
        for row_number, raw in enumerate(reader, start=2):
            row = {header_map[key]: (value or "").strip() for key, value in raw.items()}
            if not is_yes(row.get("ACTIVO", "")):
                continue
            members.append(
                Member(
                    row=row_number,
                    team=row.get("EQUIPO_ID", "").lower(),
                    code=row.get("CODIGO", ""),
                    email=row.get("CORREO", "").lower(),
                    github_user=row.get("GITHUB_USER", ""),
                    submitter=is_yes(row.get("ENTREGANTE", "")),
                    repository=row.get("REPOSITORIO", ""),
                    github_team=row.get("GITHUB_TEAM", ""),
                )
            )
    return members


def validate_members(members: Iterable[Member]) -> dict[str, list[Member]]:
    rows = list(members)
    issues: list[str] = []
    by_team: dict[str, list[Member]] = defaultdict(list)
    codes: set[str] = set()
    users: set[str] = set()

    if not rows:
        issues.append("No hay filas activas.")

    for member in rows:
        if not TEAM_RE.fullmatch(member.team):
            issues.append(f"Fila {member.row}: EQUIPO_ID inválido o vacío.")
        if not member.code:
            issues.append(f"Fila {member.row}: CODIGO vacío.")
        elif member.code in codes:
            issues.append(f"Fila {member.row}: CODIGO duplicado {member.code}.")
        codes.add(member.code)

        user_key = member.github_user.lower()
        if not USER_RE.fullmatch(member.github_user):
            issues.append(f"Fila {member.row}: GITHUB_USER inválido o vacío.")
        elif user_key in users:
            issues.append(f"Fila {member.row}: GITHUB_USER duplicado {member.github_user}.")
        users.add(user_key)

        expected_repo = f"burger-kinova-{member.team}"
        if member.repository != expected_repo:
            issues.append(
                f"Fila {member.row}: REPOSITORIO debe ser {expected_repo}."
            )
        if member.github_team != member.team:
            issues.append(
                f"Fila {member.row}: GITHUB_TEAM debe ser {member.team}."
            )
        if member.team:
            by_team[member.team].append(member)

    for team, team_members in sorted(by_team.items()):
        if len(team_members) not in (2, 3):
            issues.append(f"{team}: debe tener 2 o 3 integrantes.")
        submitters = sum(member.submitter for member in team_members)
        if submitters != 1:
            issues.append(f"{team}: debe tener exactamente un ENTREGANTE=SI.")

    if issues:
        raise SyncError("Validación del CSV fallida:\n- " + "\n- ".join(issues))
    return dict(by_team)


def run_gh(arguments: list[str], *, allow_not_found: bool = False) -> Any:
    command = [
        "gh",
        "api",
        "-H",
        "Accept: application/vnd.github+json",
        "-H",
        "X-GitHub-Api-Version: 2022-11-28",
        *arguments,
    ]
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        output = (completed.stderr or completed.stdout).strip()
        if allow_not_found and ("HTTP 404" in output or "Not Found" in output):
            return None
        raise SyncError("Falló gh api: " + output)
    text = completed.stdout.strip()
    return json.loads(text) if text else {}


def path_segment(value: str) -> str:
    return quote(value, safe="")


def inspect_github(
    org: str, by_team: dict[str, list[Member]]
) -> tuple[list[Action], list[str]]:
    viewer = run_gh(["user"])
    login = viewer.get("login", "")
    membership = run_gh([
        f"user/memberships/orgs/{path_segment(org)}"
    ])
    if membership.get("state") != "active" or membership.get("role") != "admin":
        raise SyncError(
            f"La cuenta {login or 'actual'} no es administradora activa de {org}."
        )

    actions: list[Action] = []
    warnings: list[str] = []
    for team, members in sorted(by_team.items()):
        repo = members[0].repository
        team_info = run_gh([
            f"orgs/{path_segment(org)}/teams/{path_segment(team)}"
        ], allow_not_found=True)
        if not team_info:
            raise SyncError(f"No existe el GitHub Team {team}.")

        repo_info = run_gh([
            f"repos/{path_segment(org)}/{path_segment(repo)}"
        ], allow_not_found=True)
        if not repo_info:
            raise SyncError(f"No existe el repositorio {org}/{repo}.")
        if repo_info.get("private") is not True:
            raise SyncError(f"El repositorio {org}/{repo} no es privado.")

        team_repo = run_gh([
            f"orgs/{path_segment(org)}/teams/{path_segment(team)}/repos/"
            f"{path_segment(org)}/{path_segment(repo)}"
        ], allow_not_found=True)
        push_enabled = bool((team_repo or {}).get("permissions", {}).get("push"))
        if not push_enabled:
            actions.append(Action("PERMISO_PUSH", f"{team} -> {repo}", "crear/actualizar"))

        expected = {member.github_user.lower() for member in members}
        current_members = run_gh([
            f"orgs/{path_segment(org)}/teams/{path_segment(team)}/members",
            "--paginate",
        ])
        for current in current_members or []:
            current_login = str(current.get("login", ""))
            if current_login and current_login.lower() not in expected:
                warnings.append(
                    f"{team}: integrante adicional no administrado automáticamente: {current_login}"
                )

        for member in members:
            account = run_gh([
                f"users/{path_segment(member.github_user)}"
            ], allow_not_found=True)
            if not account:
                raise SyncError(
                    f"Fila {member.row}: no existe la cuenta GitHub {member.github_user}."
                )
            state = run_gh([
                f"orgs/{path_segment(org)}/teams/{path_segment(team)}/memberships/"
                f"{path_segment(member.github_user)}"
            ], allow_not_found=True)
            if not state or state.get("state") != "active" or state.get("role") != "member":
                detail = "invitar" if not state else f"estado actual: {state.get('state')}"
                actions.append(
                    Action("MEMBRESIA", f"{member.github_user} -> {team}", detail)
                )
    return actions, warnings


def apply_actions(org: str, by_team: dict[str, list[Member]]) -> list[str]:
    results: list[str] = []
    for team, members in sorted(by_team.items()):
        repo = members[0].repository
        run_gh([
            "-X",
            "PUT",
            f"orgs/{path_segment(org)}/teams/{path_segment(team)}/repos/"
            f"{path_segment(org)}/{path_segment(repo)}",
            "-f",
            "permission=push",
        ])
        results.append(f"Permiso push confirmado: {team} -> {repo}")
        for member in members:
            response = run_gh([
                "-X",
                "PUT",
                f"orgs/{path_segment(org)}/teams/{path_segment(team)}/memberships/"
                f"{path_segment(member.github_user)}",
                "-f",
                "role=member",
            ])
            state = response.get("state", "desconocido")
            results.append(
                f"Membresía {state}: {member.github_user} -> {team}"
            )
    return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sincroniza el CSV privado de Equipos con GitHub."
    )
    parser.add_argument("--csv", required=True, type=Path, help="CSV exportado de Equipos")
    parser.add_argument("--org", default=DEFAULT_ORG)
    parser.add_argument("--periodo", default=DEFAULT_PERIOD)
    parser.add_argument(
        "--modo",
        choices=("VISTA_PREVIA", "APLICAR"),
        default="VISTA_PREVIA",
    )
    parser.add_argument(
        "--confirmar",
        default="",
        help="Para APLICAR: ORG/PERIODO, por ejemplo umng-mecatronica-ros/2026-2",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        by_team = validate_members(load_members(args.csv))
        actions, warnings = inspect_github(args.org, by_team)
        print(f"Equipos válidos: {len(by_team)}")
        print(f"Estudiantes válidos: {sum(len(rows) for rows in by_team.values())}")
        print(f"Acciones pendientes: {len(actions)}")
        for action in actions:
            print(f"- {action.kind}: {action.target} ({action.detail})")
        for warning in warnings:
            print(f"ADVERTENCIA: {warning}")

        if args.modo == "VISTA_PREVIA":
            print("No se modificó GitHub. Revise y ejecute nuevamente con --modo APLICAR.")
            return 0

        expected_confirmation = f"{args.org}/{args.periodo}"
        if args.confirmar != expected_confirmation:
            raise SyncError(
                "Confirmación incorrecta. Para aplicar use: "
                f"--confirmar {expected_confirmation}"
            )
        for line in apply_actions(args.org, by_team):
            print(line)
        print("Sincronización aplicada. Las invitaciones pendientes deben ser aceptadas.")
        return 0
    except SyncError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
