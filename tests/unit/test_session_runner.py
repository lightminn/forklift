"""The runner's session pieces (docs/plans/2026-10-22-rigid-session-pallets.md 2.3-2.8), its functions taken out of
run_transport.py by AST and run on stand-ins (as the start-check tests do)."""
import ast
import importlib.util
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "sim/isaac/run_transport.py"
sys.path.insert(0, str(ROOT / "sim/isaac"))
SPEC = importlib.util.spec_from_file_location("session_pallets", ROOT / "sim/isaac/session_pallets.py")
S = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(S)
sys.modules.setdefault("session_pallets", S)


def _functions(*names):
    tree = ast.parse(SCRIPT.read_text())
    found = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name in names]
    assert sorted(n.name for n in found) == sorted(names)
    return found


class _H:
    def __init__(self, x, y, yaw=0.0, z=0.0):
        self.set(x, y, yaw, z)

    def set(self, x, y, yaw=0.0, z=0.0):
        self.pose = (np.array([x, y, z]), np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]))

    def get_world_pose(self):
        return self.pose


def _yaw_and_tilt(q):
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)), 0.0


class _Trace:
    made = []

    def __init__(self):
        self.ground_at = None
        _Trace.made.append(self)

    def ground(self, pallet):
        self.ground_at = pallet


def _tick_scope(carrying=None):
    """session_tick_head with three pallets, P-1 the mission's; its scene_require records instead of raising."""
    from forklift_core.planning.geometry import Rectangle

    handles = {"P-1": _H(1.0, 0.0), "P-2": _H(1.0, 3.0), "P-3": _H(1.0, 6.0)}
    sw = S.PalletSwitch(handles, "P-1")
    rest = S.RestWatch()
    for pid, h in handles.items():
        rest.rest(pid, *h.get_world_pose())
    damage = []
    old_trace = _Trace()
    scope = {
        "pallet": sw, "session_handles": handles, "session_rest": rest,
        "session": SimpleNamespace(pallets=[SimpleNamespace(pallet_id=p) for p in ("P-1", "P-2", "P-3")]),
        "core": SimpleNamespace(cargo=SimpleNamespace(pallet_id=carrying, relation="carrying" if carrying else "free")),
        "far_runtime": SimpleNamespace(truth_instance=0), "trace_evidence": old_trace,
        "live_records": SimpleNamespace(TraceEvidence=_Trace), "state": {},
        "scene_require": lambda t, ok, reason, cargo=False: damage.append(reason) if not ok else None,
        "Rectangle": Rectangle, "geometry": SimpleNamespace(pallet_depth_m=0.6, pallet_width_m=0.8),
        "yaw_and_tilt": _yaw_and_tilt, "session_truth_rects_": [],
    }
    (fn,) = _functions("session_tick_head")
    # session_tick_head uses nonlocal: run it as a closure inside a wrapper that owns those names
    wrapper = ast.FunctionDef(
        name="_wrap", args=ast.arguments(posonlyargs=[], args=[], kwonlyargs=[], kw_defaults=[], defaults=[]),
        body=[ast.parse("trace_evidence = scope['trace_evidence']").body[0],
              ast.parse("session_truth_rects_ = []").body[0], ast.parse("session_truth_ids_ = []").body[0], fn,
              ast.parse("return session_tick_head, (lambda: (trace_evidence, session_truth_rects_, "
                        "session_truth_ids_))").body[0]],
        decorator_list=[], returns=None, type_params=[])
    module = ast.fix_missing_locations(ast.Module([wrapper], []))
    env = dict(scope, scope=scope)
    exec(compile(module, str(SCRIPT), "exec"), env)
    head, read = env["_wrap"]()
    return head, read, sw, handles, rest, damage, scope, old_trace


def test_a_requested_switch_is_made_at_the_next_tick_head_with_its_records():
    head, read, sw, handles, rest, damage, scope, old_trace = _tick_scope()
    head(10, 1.0)
    trace, rects, ids = read()
    assert trace is old_trace and sorted((r.x_m, r.y_m) for r in rects) == [(1.0, 3.0), (1.0, 6.0)]
    assert [(i, r.y_m) for i, r in zip(ids, rects)] == [("P-2", 3.0), ("P-3", 6.0)]  # ids in the outlines' order
    handles["P-1"].set(3.0, -8.9)  # delivered at its slot, then P-2 adopted
    sw.request("P-2")
    head(11, 1.1)
    trace, rects, ids = read()
    assert sw.current == "P-2" and scope["far_runtime"].truth_instance == 1
    assert scope["state"]["session_switches"] == [{"step": 11, "t": 1.1, "from": "P-1", "to": "P-2"}]
    assert trace is not old_trace and trace.ground_at == pytest.approx((1.0, 3.0, 0.0))  # a new pallet's evidence
    assert rest.poses["P-1"][0] == pytest.approx((3.0, -8.9, 0.0))  # rested where it left
    assert sorted((r.x_m, r.y_m) for r in rects) == [(1.0, 6.0), (3.0, -8.9)]  # the others where they stand
    assert dict(zip(ids, ((r.x_m, r.y_m) for r in rects))) == {"P-1": (3.0, -8.9), "P-3": (1.0, 6.0)}
    assert damage == []


def test_a_resubmit_of_the_same_pallet_keeps_its_trace_evidence():
    head, read, sw, *_rest, old_trace = _tick_scope()
    sw.request("P-1")
    head(5, 0.5)
    assert read()[0] is old_trace and sw.switches == []


def test_a_moved_neighbour_is_scene_damage_and_its_outline_where_it_is_now():
    head, read, sw, handles, rest, damage, scope, _ = _tick_scope()
    handles["P-2"].set(1.0, 3.009)  # within 1 cm: no damage, the outline follows it (Codex plan round 2 P2)
    head(1, 0.1)
    assert damage == [] and any(abs(r.y_m - 3.009) < 1e-9 for r in read()[1])
    handles["P-2"].set(1.0, 3.012)
    head(2, 0.2)
    assert damage and damage[0].startswith("Session pallet displaced: P-2")


def test_the_carried_pallet_is_neither_watched_nor_left_by_the_switch():
    head, read, sw, handles, rest, damage, scope, _ = _tick_scope(carrying="P-1")
    handles["P-1"].set(5.0, 5.0)  # on the forks, far from its rest
    head(1, 0.1)
    assert damage == []
    sw.request("P-2")
    with pytest.raises(RuntimeError, match="session_pallet_switch_while_carrying"):
        head(2, 0.2)


def test_the_lidar_leaves_out_only_the_carried_session_pallet():
    (fn,) = _functions("session_self_prefixes")
    sw = S.PalletSwitch({"P-1": _H(0, 0), "P-2": _H(0, 3)}, "P-2")
    for relation, cargo_id, loaded, want in [
            ("carrying", "P-1", False, ("/World/Forklift", "/World/SessionPallets/P_1/")),  # teleop carrying
            ("free", None, True, ("/World/Forklift", "/World/SessionPallets/P_2/")),  # the mission's, loaded phase
            ("free", None, False, ("/World/Forklift",))]:
        env = {"core": SimpleNamespace(cargo=SimpleNamespace(relation=relation, pallet_id=cargo_id)), "pallet": sw,
               "planar_lidar": SimpleNamespace(SELF_PREFIX="/World/Forklift")}
        exec(compile(ast.Module([fn], []), str(SCRIPT), "exec"), env)
        assert env["session_self_prefixes"](loaded) == want


def test_no_far_frame_or_step_is_taken_while_a_switch_waits():
    (fn,) = _functions("far_observing")
    sw = S.PalletSwitch({"P-1": _H(0, 0), "P-2": _H(0, 3)}, "P-1")
    env = {"far_live": object(), "session": object(), "pallet": sw,
           "frr": SimpleNamespace(live_observing=lambda status, obs, stopped: obs),
           "core": SimpleNamespace(commands=SimpleNamespace(status="running")),
           "live": SimpleNamespace(stopped_in_tick=False)}
    exec(compile(ast.Module([fn], []), str(SCRIPT), "exec"), env)
    assert env["far_observing"](True) is True
    sw.request("P-2")
    assert env["far_observing"](True) is False


def test_a_displaced_session_pallet_is_scene_damage_with_console_text():
    from forklift_core.mission.failures import SCENE, classify

    assert classify("Session pallet displaced: P-2 (0.012 m, 0.000 rad)") == SCENE
    assert '"Session pallet displaced"' in (ROOT / "console/web/js/results.js").read_text()


def _session_args(tmp_path, *extra):
    import run_transport

    from tools import riibotics_isaac_scenes as tool

    path = tmp_path / "line_session.json"
    path.write_text(json.dumps(tool.session_record("line")))
    argv = sys.argv
    sys.argv = ["run_transport.py", "--base-scene", "scene.usda", "--output", "out", "--seed", "0",
                "--forklift-urdf", str(ROOT / "sim/models/dls08_measured/forklift.urdf"),
                "--pallet-urdf", str(ROOT / "sim/models/epal6_pallet/pallet.urdf"),
                "--pallet-geometry", str(ROOT / "config/pallet_geometry_epal6.yaml"),
                "--settings", str(ROOT / "config/isaac_transport_measured.yaml"),
                "--use-perception", "--planning-target", "perception", "--record-slam",
                "--pallet-prior", str(ROOT / "config/pallet_prior_epal6.yaml"),
                "--obstacle-layer", str(ROOT / "config/obstacle_layer_single.yaml"), "--obstacle-act",
                "--layout", "scene", "--scene-file", str(path), *extra]
    try:
        return run_transport.arguments()
    finally:
        sys.argv = argv


def test_a_session_scene_is_a_live_console_run_with_the_redraw_always(tmp_path, capsys):
    args = _session_args(tmp_path, "--live", "--near-capture-redraw", "always")
    assert args.session and args.observation_waypoints  # the first mission's candidates (P-1)
    for extra, message in [((), "needs --live"),
                           (("--live",), "needs --near-capture-redraw always"),
                           (("--live", "--near-capture-redraw", "always", "--bottom-deck-check", "on"),
                            "--bottom-deck-check on is a batch run"),  # refused by its own rule already
                           (("--live", "--near-capture-redraw", "always", "--record-pocket-frames"),
                            "--record-pocket-frames")]:  # its truth names no pallet (Codex PR 3 impl round 1 P3)
        with pytest.raises(SystemExit):
            _session_args(tmp_path, *extra)
        assert message in capsys.readouterr().err


def test_the_near_straight_meets_a_neighbour_its_box_check_cannot_see():
    """Codex PR 2 round 1 counterexample: a 2.1 m straight started 0.049 m / 0.049 rad off passes the neighbour's
    planning outline (nominal (1.2, 1.09, 0) + 0.25 m); the sweep check sees it, a clear one passes."""
    from forklift_core.mission.core import MissionCore
    from forklift_core.planning.geometry import Footprint, Rectangle

    core = MissionCore.__new__(MissionCore)
    core.geometry = SimpleNamespace(unloaded_footprint=Footprint(1.29, 0.17, 0.36))
    yaw = 0.049
    trajectory = [(0.0 + d * math.cos(yaw), 0.049 + d * math.sin(yaw), yaw) for d in np.linspace(0.0, 2.1, 85)]
    near = Rectangle(1.2, 1.09, 0.6 + 0.5, 0.8 + 0.5, 0.0)
    assert not core._sweep_clear(trajectory, [near])
    assert core._sweep_clear(trajectory, [Rectangle(1.2, 2.5, 1.1, 1.3, 0.0)])


def test_the_gap_to_a_neighbour_is_the_outlines_distance_and_its_smallest_is_kept():
    from forklift_core.planning.geometry import Rectangle

    truck = S.footprint_corners((0.0, 0.0, 0.0), 1.0, 0.2, 0.3)  # x -0.2..1.0, y -0.3..0.3
    ahead = S.rect_corners(Rectangle(1.5, 0.0, 0.6, 0.8, 0.0))  # x 1.2..1.8
    beside = S.rect_corners(Rectangle(0.4, 1.0, 0.6, 0.8, 0.0))  # y 0.6..1.4
    turned = S.rect_corners(Rectangle(2.0, 0.0, 1.0, 1.0, math.pi / 4))  # its corner at x 2 - 0.7071
    assert S.polygon_gap(truck, ahead) == pytest.approx(0.2)
    assert S.polygon_gap(truck, beside) == pytest.approx(0.3)
    assert S.polygon_gap(truck, turned) == pytest.approx(2.0 - math.sqrt(0.5) - 1.0)
    assert S.polygon_gap(truck, S.rect_corners(Rectangle(0.9, 0.0, 0.6, 0.8, 0.0))) == 0.0  # overlapping
    gap = S.NeighbourGap()
    gap.update(1.0, "approach", "P-1", truck, {"P-2": ahead, "P-3": beside})
    gap.update(2.0, "transport", "P-1", S.footprint_corners((0.1, 0.0, 0.0), 1.0, 0.2, 0.3), {"P-2": ahead,
                                                                                             "P-3": beside})
    assert gap.smallest["P-2"] == {"m": pytest.approx(0.1), "t": 2.0, "phase": "transport", "mission_pallet": "P-1"}
    assert gap.smallest["P-3"]["t"] == 1.0  # the same gap later does not replace the first


def test_the_runner_wires_the_session_records_to_its_truth():
    """Codex PR 3 impl round 1: the docking dry run checks the true outlines, the first mission's far truth instance is
    its pallet's, the gap is recorded every tick, the video log names its pallet."""
    source = SCRIPT.read_text()
    assert 'session_extra_["fn"] = lambda: list(session_truth_rects_)' in source
    assert "far_runtime.truth_instance = [p_.pallet_id for p_ in session.pallets].index(session.sequence[0][0])" in source
    assert 'state["session_neighbour_gap"] = session_gap.smallest' in source
    assert '**({} if session is None else {"pallet_id": pallet.current})' in source
