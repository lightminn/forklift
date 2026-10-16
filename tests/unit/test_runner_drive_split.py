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
    assert all(text.startswith(("'", '"', "_bound", "if '", "if boundary is not None", "early_ports = ", "if early_ports"))
               for text in statements[:begin])


def test_drive_reads_no_isaac_helper_names():
    """Isaac-typed helpers (USD math, articulation actions) are reached through ``isaac`` only (Codex R2 review:
    the chase camera's Gf look-at ran inside drive)."""
    tree = ast.parse(SCRIPT.read_text())
    drive = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "drive")
    names = {n.id for n in ast.walk(drive) if isinstance(n, ast.Name)}
    assert not names & {"Gf", "ArticulationAction", "UsdGeom", "UsdPhysics", "omni", "Camera", "World"}


def test_the_end_state_projection_hides_the_output_directory_and_lists_recorded_only_keys(tmp_path):
    import numpy as np

    from forklift_core.mission.state import MissionState

    spec = importlib.util.spec_from_file_location("run_transport_projection", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ms = MissionState(phase="return_home")
    a = module.final_projection({"video": str(tmp_path / "a" / "v.mp4"), "lift": np.float64(0.1),
                                 "setup_snapshot": {"files": 3}}, ms, tmp_path / "a")
    b = module.final_projection({"video": str(tmp_path / "b" / "v.mp4"), "lift": np.float64(0.1)}, ms, tmp_path / "b")
    assert a["result"] == b["result"] and set(a["result"]) == {"video", "lift"}
    assert a["recorded_only"] == ["setup_snapshot"] and a["mission"] is ms
    c = module.final_projection({"video": "x", "lift": np.float64(0.2)}, ms, tmp_path / "a")
    assert c["result"]["lift"] != a["result"]["lift"]


def test_the_stdout_observer_records_printed_lines_as_sink_echo_and_still_prints(tmp_path, capsys):
    from forklift_core.mission.boundary_record import BoundaryRecorder, RecordingGate
    from forklift_core.mission.replay import read_ticks

    spec = importlib.util.spec_from_file_location("run_transport_print", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    module._BOUNDARY_RECORD.update(gate=gate, print=print)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    module._recording_print("SAMPLE", '{"t": 1}', flush=True)
    module._recording_print("to stderr", file=sys.stderr)
    rec.tick_end(outputs={}, branches=[])
    module._recording_print("outside a tick")
    rec.close()
    module._BOUNDARY_RECORD.update(gate=None, print=None)
    calls = next(iter(read_ticks(tmp_path)))["calls"]
    assert [(c["port"], c["method"]) for c in calls] == [("Sink", "echo")]
    out = capsys.readouterr()
    assert out.out == 'SAMPLE {"t": 1}\noutside a tick\n' and out.err == "to stderr\n"


def test_main_closes_the_record_with_what_left_run_and_the_harness_does_the_same():
    source = SCRIPT.read_text()
    assert "close_boundary_record(state, failure_exc)" in source and "failure_exc = exc" in source
    harness = (ROOT / "tools/boundary_replay.py").read_text()
    assert "module.close_boundary_record(S.state, raised)" in harness
    assert 'S.state["failure_reason"] = str(raised)' in harness
