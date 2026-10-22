"""The runner's transport exit prefix (docs/plans/2026-10-22-riibotics-trailer-exit.md D1, tests 7, 8, 11):
``--exit-clearance-m`` only on a scene run, the scene's tall walls as its obstacles, the near-field contract's key,
and the same ExitPrefix reaching every transport planner -- the hooks, the worker's jobs and a replay's setup."""

import json
import math
import re
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core.planning import Pose2D
from forklift_core.planning.geometry import Rectangle
from forklift_core.planning.pallet_mission import ExitPrefix

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from scene_file import exit_walls, load_scene  # noqa: E402

from tools import riibotics_isaac_scenes as tool  # noqa: E402

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402

SCRIPT = ROOT / "sim/isaac/run_transport.py"
BAND_BOTTOM = 1.044  # config/obstacle_layer_single.yaml


@pytest.fixture(scope="module")
def trailer(tmp_path_factory):
    path = tmp_path_factory.mktemp("scenes") / "trailer.json"
    path.write_text(json.dumps(tool.scene_record("trailer", tool.scenes()["trailer"])))
    return path


def test_the_exit_clearance_is_a_scene_runs_and_positive(trailer):
    scene = ["--layout", "scene", "--scene-file", str(trailer)]
    assert _runner_args(scene).exit_clearance_m is None  # off unless given
    assert _runner_args(scene + ["--exit-clearance-m", "0.30"]).exit_clearance_m == 0.30
    for extra in (["--exit-clearance-m", "0.30"],  # the bay: no scene walls
                  scene + ["--exit-clearance-m", "0"], scene + ["--exit-clearance-m", "-0.1"],
                  scene + ["--exit-clearance-m", "nan"], scene + ["--exit-clearance-m", "inf"]):
        with pytest.raises(SystemExit):
            _runner_args(extra)


def test_the_exit_obstacles_are_the_scenes_walls_that_reach_the_band(trailer):
    scene = load_scene(trailer)
    walls = exit_walls(scene, BAND_BOTTOM)
    assert len(walls) == 3  # both side walls and the front wall; floors and the pallet copies are not walls
    assert {(round(w.x_m, 3), round(w.y_m, 3)) for w in walls} == {(3.34, 0.718), (3.34, -0.718), (4.64, 0.0)}
    assert exit_walls(scene, 1.5) == ()  # 1.36 m walls under a higher band are not met


def test_the_near_field_contract_knows_the_exit_clearance():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["exit_clearance_m"] is None


def _hook(name: str) -> str:
    source = SCRIPT.read_text()
    m = re.search(rf"\n\s+{name}=(lambda \*a, \*\*k: {name}\(\n?\s*\*a, \*\*k, \*\*\(\{{\}} if transport_exit is None "
                  r"else \{\"exit_prefix\": transport_exit\}\)\))", source)
    assert m is not None, name
    return m.group(1)


@pytest.mark.parametrize("name", ["plan_transport_leg", "plan_transport"])
def test_the_hooks_transport_planners_carry_the_runs_exit_prefix(name):
    exit_ = ExitPrefix((Rectangle(3.34, 0.718, 0.08, 2.68),), 0.30)
    calls = []
    planner = lambda *a, **k: calls.append((a, k)) or "R"  # noqa: E731
    on = eval(_hook(name), {name: planner, "transport_exit": exit_})  # noqa: S307
    off = eval(_hook(name), {name: planner, "transport_exit": None})  # noqa: S307
    assert on("W", "S", geometry="G") == "R" and off("W", "S", geometry="G") == "R"
    assert calls == [(("W", "S"), {"geometry": "G", "exit_prefix": exit_}), (("W", "S"), {"geometry": "G"})]


def test_the_hooks_hold_the_exit_prefix_and_drive_binds_it_from_the_setup():
    source = SCRIPT.read_text()
    assert "\n            exit_prefix=transport_exit,\n" in source  # the core's hooks.exit_prefix
    import replay_setup

    spec = __import__("importlib.util").util.spec_from_file_location("run_transport_exit", SCRIPT)
    module = __import__("importlib.util").util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert "transport_exit" in replay_setup.drive_inputs(module.drive)
    # the loaded delivery's worker job and the first mission plan
    loaded = source[source.index('"occupancy": grid_["occupancy"], "deadline": time.monotonic() + 3 * 20.0'):][:200]
    assert '**({} if transport_exit is None else {"exit_prefix": transport_exit})' in loaded
    first = source[source.index('        plans = plan_transport(\n            grid_world(scenario),'):][:420]
    assert '**({} if transport_exit is None else {"exit_prefix": transport_exit})' in first


def _core(exit_):
    from forklift_core.mission.core import MissionCore

    core = MissionCore.__new__(MissionCore)
    core.hooks = SimpleNamespace(exit_prefix=exit_)
    core.options = SimpleNamespace(planner_config=SimpleNamespace(clearance_m=0.1), travel_config=None,
                                   return_to_pose=Pose2D(0, 0, 0))
    core.geometry = "GEOMETRY"
    return core


def test_the_workers_transport_leg_jobs_carry_it_and_the_return_legs_do_not(monkeypatch):
    import forklift_core.mission.core as core_module

    monkeypatch.setattr(core_module, "replace", lambda obj, **k: SimpleNamespace(**{**vars(obj), **k}))
    exit_ = ExitPrefix((Rectangle(3.34, 0.718, 0.08, 2.68),), 0.30)
    leg = _core(exit_)._leg_payload("transport", "W", Pose2D(0, 0, 0), {"occupancy": "O"}, retry_on=("no_path",))
    assert leg["kwargs"]["exit_prefix"] is exit_ and leg["tight_kwargs"]["exit_prefix"] is exit_
    back = _core(exit_)._leg_payload("return", "W", Pose2D(0, 0, 0), {}, retry_on=())
    assert "exit_prefix" not in back["kwargs"] and "exit_prefix" not in back["tight_kwargs"]
    off = _core(None)._leg_payload("transport", "W", Pose2D(0, 0, 0), {}, retry_on=())
    assert "exit_prefix" not in off["kwargs"] and "exit_prefix" not in off["tight_kwargs"]


def test_the_workers_mission_plan_carries_it():
    core = (ROOT / "src/forklift_core/mission/core.py").read_text()
    block = core[core.index("kwargs_ = dict(geometry=geometry, target_pickup=planning_pickup"):][:420]
    assert "**self._exit_prefix_kwargs())" in block


def test_the_setup_snapshot_restores_the_same_exit_prefix(tmp_path):
    import replay_setup

    from forklift_core.mission.replay_codec import decode

    exit_ = ExitPrefix((Rectangle(3.34, 0.718, 0.08, 2.68, -math.pi / 2), Rectangle(4.64, 0.0, 1.516, 0.08)), 0.30)
    audit = replay_setup.write_setup_snapshot({"transport_exit": exit_}, tmp_path / "boundary", [],
                                              wanted={"transport_exit"})
    assert audit["opaque"] == []
    snap = json.loads((tmp_path / "boundary/setup_snapshot.json").read_text())
    item = snap["names"]["transport_exit"]
    assert item["kind"] == "value"
    restored = decode(item["node"], {})
    assert restored == exit_ and isinstance(restored, ExitPrefix)


def test_the_final_state_projection_leaves_an_unset_exit_prefix_out():
    # Codex code review round 2 P2: a record's final-state hash was taken before the field existed
    import numpy as np

    import run_transport

    from forklift_core.planning.hybrid_astar import PlanResult

    plain = PlanResult(True, "success", np.zeros((2, 3)), np.ones(2, dtype=np.int8), np.zeros(2), 0.1, 3)
    marked = PlanResult(True, "success", np.zeros((2, 3)), np.ones(2, dtype=np.int8), np.zeros(2), 0.1, 3,
                        exit_prefix={"length_m": 0.35})
    assert "exit_prefix" not in run_transport.projection_node(plain, {})["attrs"]
    assert "exit_prefix" in run_transport.projection_node(marked, {})["attrs"]
