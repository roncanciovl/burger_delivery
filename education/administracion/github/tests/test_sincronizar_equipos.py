import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "sincronizar_equipos.py"
SPEC = importlib.util.spec_from_file_location("sincronizar_equipos", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


HEADERS = [
    "ACTIVO",
    "EQUIPO_ID",
    "CODIGO",
    "CORREO",
    "GITHUB_USER",
    "ENTREGANTE",
    "REPOSITORIO",
    "GITHUB_TEAM",
]


class CsvValidationTest(unittest.TestCase):
    def write_csv(self, rows):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "equipos.csv"
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=HEADERS)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_accepts_pair_with_one_submitter(self):
        rows = [
            {
                "ACTIVO": "SI",
                "EQUIPO_ID": "equipo-01",
                "CODIGO": "1",
                "CORREO": "a@example.edu",
                "GITHUB_USER": "student-a",
                "ENTREGANTE": "SI",
                "REPOSITORIO": "burger-kinova-equipo-01",
                "GITHUB_TEAM": "equipo-01",
            },
            {
                "ACTIVO": "SI",
                "EQUIPO_ID": "equipo-01",
                "CODIGO": "2",
                "CORREO": "b@example.edu",
                "GITHUB_USER": "student-b",
                "ENTREGANTE": "NO",
                "REPOSITORIO": "burger-kinova-equipo-01",
                "GITHUB_TEAM": "equipo-01",
            },
        ]
        result = MODULE.validate_members(MODULE.load_members(self.write_csv(rows)))
        self.assertEqual(len(result["equipo-01"]), 2)

    def test_rejects_team_without_submitter(self):
        rows = [
            {
                "ACTIVO": "SI",
                "EQUIPO_ID": "equipo-01",
                "CODIGO": str(index),
                "CORREO": f"s{index}@example.edu",
                "GITHUB_USER": f"student-{index}",
                "ENTREGANTE": "NO",
                "REPOSITORIO": "burger-kinova-equipo-01",
                "GITHUB_TEAM": "equipo-01",
            }
            for index in (1, 2)
        ]
        with self.assertRaises(MODULE.SyncError):
            MODULE.validate_members(MODULE.load_members(self.write_csv(rows)))


if __name__ == "__main__":
    unittest.main()
