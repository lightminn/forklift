"""M-a1 replay plan R2: the runner's drive part (the initial plans on) runs without Isaac -- its module imports with
Isaac refused, and drive() itself imports nothing from Isaac (the Isaac work goes through the setup's adapters)."""

import ast
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "sim/isaac/run_transport.py"
ISAAC = ("isaacsim", "omni", "pxr", "carb", "scene")


def test_the_runner_module_imports_with_isaac_refused(monkeypatch):
    for name in ISAAC:
        monkeypatch.setitem(sys.modules, name, None)  # any import of it raises ImportError
    monkeypatch.syspath_prepend(str(SCRIPT.parent))
    spec = importlib.util.spec_from_file_location("run_transport_cpu", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert callable(module.drive) and callable(module.run)


def test_drive_imports_nothing_from_isaac_and_run_hands_over_at_the_initial_plans():
    tree = ast.parse(SCRIPT.read_text())
    drive = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "drive")
    imported = set()
    for n in ast.walk(drive):
        if isinstance(n, ast.Import):
            imported |= {a.name.split(".")[0] for a in n.names}
        elif isinstance(n, ast.ImportFrom) and n.module:
            imported.add(n.module.split(".")[0])
    assert not imported & set(ISAAC), imported & set(ISAAC)
    run = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "run")
    assert ast.unparse(run.body[-1]) == "return drive(_Setup(**locals()), isaac)"
    # drive begins where the initial plans begin (its moved helper closures first)
    statements = [ast.unparse(s) for s in drive.body if not isinstance(s, ast.FunctionDef)]
    begin = next(i for i, text in enumerate(statements) if text == "state['phase'] = 'planning'")
    assert all(text.startswith(("'", '"', "_bound", "if '", "if boundary is not None")) for text in statements[:begin])


def test_drive_reads_no_isaac_helper_names():
    """Isaac-typed helpers (USD math, articulation actions) are reached through ``isaac`` only (Codex R2 review:
    the chase camera's Gf look-at ran inside drive)."""
    tree = ast.parse(SCRIPT.read_text())
    drive = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "drive")
    names = {n.id for n in ast.walk(drive) if isinstance(n, ast.Name)}
    assert not names & {"Gf", "ArticulationAction", "UsdGeom", "UsdPhysics", "omni", "Camera", "World"}
