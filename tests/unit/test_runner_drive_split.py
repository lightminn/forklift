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


def test_the_end_state_projection_covers_the_containers_by_logical_service_and_hides_the_output(tmp_path):
    import numpy as np

    from forklift_core.mission.boundary_record import BoundaryRecorder, RecordingGate, RecordingProxy
    from forklift_core.mission.state import MissionState

    spec = importlib.util.spec_from_file_location("run_transport_projection", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ms = MissionState(phase="return_home")
    gate = RecordingGate(BoundaryRecorder(tmp_path / "b", value_format=2))

    def containers(out, tick_value, link):
        return {"state": {"video": str(out / "v.mp4"), "lift": np.float64(0.1), "setup_snapshot": {"files": 3}},
                "near": {"ticks": [{"d": tick_value}], "tracker": RecordingProxy(object(), "NearTracking", gate)},
                "slam": {"link": link}, "paths": None}

    a = module.final_projection(containers(tmp_path / "a", 1.0, object()), ms, tmp_path / "a")
    b = module.final_projection(containers(tmp_path / "b", 1.0, object()), ms, tmp_path / "b")
    assert a == b and "setup_snapshot" not in a["containers"]["state"] and a["containers"]["paths"] is None
    c = module.final_projection(containers(tmp_path / "a", 2.0, object()), ms, tmp_path / "a")
    assert c["containers"]["near"]["ticks"] != a["containers"]["near"]["ticks"]  # an old log item changed
    assert c["containers"]["near"]["tracker"] == a["containers"]["near"]["tracker"]


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


def test_a_print_whose_end_changed_is_a_different_sink_payload(tmp_path):
    from forklift_core.mission.boundary_record import payload_digest

    spec = importlib.util.spec_from_file_location("run_transport_print_end", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    seen = []
    module.sink_effect = lambda gate, method, path, payload, write=None: (seen.append(payload), write())[1]
    module._BOUNDARY_RECORD.update(gate=object(), print=lambda *a, **k: None)
    module._recording_print("SAMPLE")
    module._recording_print("SAMPLE", end="")
    module._BOUNDARY_RECORD.update(gate=None, print=None)
    assert seen == ["SAMPLE\n", "SAMPLE"] and payload_digest(seen[0]) != payload_digest(seen[1])


def test_main_closes_the_record_with_what_left_run_and_the_harness_does_the_same():
    source = SCRIPT.read_text()
    assert "close_boundary_record(state, failure_exc)" in source and "failure_exc = exc" in source
    harness = (ROOT / "tools/boundary_replay.py").read_text()
    assert "module.close_boundary_record(S.state, raised)" in harness
    assert 'S.state["failure_reason"] = str(raised)' in harness


def test_the_projection_encodes_cpu_state_by_value_refuses_unknown_objects_and_hides_both_output_roots(tmp_path):
    import pytest

    from forklift_core.mission.state import MissionState

    spec = importlib.util.spec_from_file_location("run_transport_projection2", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class RearHistory:  # by name, as the near-field helper
        def __init__(self, rows):
            self._rows = rows

    ms = MissionState()
    a = module.final_projection({"near": {"rear": RearHistory([1.0])}}, ms, tmp_path)
    b = module.final_projection({"near": {"rear": RearHistory([2.0])}}, ms, tmp_path)
    assert a["containers"]["near"]["rear"] != b["containers"]["near"]["rear"]
    with pytest.raises(TypeError, match=r"near\['thing'\]"):
        module.final_projection({"near": {"thing": object()}}, ms, tmp_path)
    recorded = module.final_projection({"state": {"arguments": {"output": "/runs/a"}, "f": "/runs/a/x.json"}},
                                       ms, Path("/runs/a"))
    replayed = module.final_projection({"state": {"arguments": {"output": "/runs/a"}, "f": "/runs/a/x.json"}},
                                       ms, Path("/tmp/replay_out"))
    assert recorded == replayed


def test_main_writes_failure_txt_outside_the_record_and_capture_images_are_sinks():
    source = SCRIPT.read_text()
    assert '(Path(str(args.output)) / "failure.txt").write_text(traceback.format_exc())' in source
    assert source.count("save_image(Image.fromarray(") == 2


def test_the_projection_handles_the_slam_container_and_refuses_a_socket(tmp_path):
    import socket

    import numpy as np
    import pytest

    from forklift_core.mission.state import MissionState

    spec = importlib.util.spec_from_file_location("run_transport_projection3", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class Odometry:
        def __init__(self, x):
            self.pose = np.array([x, 0.0])
            self.callback = print

    slam = {"module": np, "noise": np.random.default_rng(3), "odometry": Odometry(1.0), "link": object(),
            "holds": {("a", 1)}, "estimate": Odometry(1.0), "rng": np.random.default_rng(4)}
    a = module.final_projection({"slam": slam}, MissionState(), tmp_path)
    slam["estimate"] = Odometry(2.0)
    b = module.final_projection({"slam": slam}, MissionState(), tmp_path)
    assert a["containers"]["slam"]["module"] == b["containers"]["slam"]["module"]
    assert a["containers"]["slam"]["estimate"] != b["containers"]["slam"]["estimate"]
    with socket.socket() as sock, pytest.raises(TypeError, match=r"slam\['sock'\]"):
        module.final_projection({"slam": {"sock": sock}}, MissionState(), tmp_path)


def test_service_owned_slam_state_is_compared_by_id_and_native_objects_refuse(tmp_path):
    import pytest

    from forklift_core.mission.state import MissionState

    spec = importlib.util.spec_from_file_location("run_transport_projection4", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class Stop:
        def __init__(self, n):
            self.count = n

    a = module.final_projection({"slam": {"stop": Stop(1), "stop_now": False}}, MissionState(), tmp_path)
    b = module.final_projection({"slam": {"stop": Stop(9), "stop_now": False}}, MissionState(), tmp_path)
    assert a == b  # the step's own detector: by its logical id
    with open(SCRIPT, "rb") as fh, pytest.raises(TypeError, match=r"near\['file'\]"):
        module.final_projection({"near": {"file": fh}}, MissionState(), tmp_path)


def test_a_c_backed_generator_is_projected_by_its_state_and_foreign_objects_refuse(tmp_path):
    import random
    from fractions import Fraction  # a Python class from outside the project

    import pytest

    from forklift_core.mission.state import MissionState

    spec = importlib.util.spec_from_file_location("run_transport_projection5", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rng = random.Random(3)
    a = module.final_projection({"near": {"rng": rng}}, MissionState(), tmp_path)
    rng.random()
    b = module.final_projection({"near": {"rng": rng}}, MissionState(), tmp_path)
    assert a != b
    with pytest.raises(TypeError, match=r"near\['f'\]"):
        module.final_projection({"near": {"f": Fraction(1, 3)}}, MissionState(), tmp_path)
