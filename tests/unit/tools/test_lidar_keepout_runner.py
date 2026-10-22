"""The runner's P1c switch (docs/plans/2026-10-23-planner-permission-clearance.md): ``--lidar-keepout-m`` off by
default, finite and non-negative, not with grid planning; the near-field contract's key; the geometry it builds and
records (unchanged while off)."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402


def test_the_keepout_is_off_unless_given_and_must_be_a_non_negative_length():
    assert _runner_args([]).lidar_keepout_m == 0.0
    assert _runner_args(["--lidar-keepout-m", "0.15"]).lidar_keepout_m == 0.15
    for bad in ("-0.01", "nan", "inf"):
        with pytest.raises(SystemExit):
            _runner_args(["--lidar-keepout-m", bad])


def test_the_keepout_is_refused_with_grid_planning():
    with pytest.raises(SystemExit):
        _runner_args(["--lidar-keepout-m", "0.1", "--grid-planning"])


def test_the_near_field_contract_knows_the_keepout():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["lidar_keepout_m"] == 0.0


def test_the_recorded_geometry_gains_the_keepout_only_when_it_is_set():
    from dataclasses import asdict

    from forklift_core.mission.replay_codec import recorded_fields
    from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

    def record(g):  # as run_transport records state["geometry"]
        kept = {f.name for f in recorded_fields(g)}
        return {k: v for k, v in asdict(g).items() if k in kept}

    off = record(SyntheticMissionGeometry())
    assert "lidar_keepout_m" not in off and "lidar_plane_m" not in off
    assert set(off) == set(asdict(SyntheticMissionGeometry())) - {"lidar_keepout_m", "lidar_plane_m"}
    assert record(SyntheticMissionGeometry(lidar_keepout_m=0.15))["lidar_keepout_m"] == 0.15


SCRIPT = ROOT / "sim/isaac/run_transport.py"


def test_the_run_hands_the_keepouts_prefix_on_as_its_transport_exit_before_the_hooks_take_it():
    # Codex P1c review P1: with the prefix only inside the planner, exit_prefix_m read 0 and a short transport's
    # docking-straight window could reach back into the prefix, where the permission then did not act
    source = SCRIPT.read_text()
    block = source.index("auto_ = keepout_exit_prefix(scenario, geometry, search_clearance_)")
    assert source.index("transport_exit = auto_", block) < source.index("exit_prefix=transport_exit,")


def test_the_runs_prefix_is_the_one_the_planner_would_build_and_plans_the_same():
    import numpy as np

    from forklift_core.planning.pallet_mission import keepout_exit_prefix, plan_transport_leg

    sys.path.insert(0, str(ROOT / "tests/unit/planning"))
    import test_lidar_keepout as t

    geometry = t.keepout(0.10)
    explicit = keepout_exit_prefix(t.scenario(), geometry, t.CONFIG.clearance_m)
    a = plan_transport_leg(t.scenario(), t.EXTRACTED, t.CONFIG, geometry=geometry)
    b = plan_transport_leg(t.scenario(), t.EXTRACTED, t.CONFIG, geometry=geometry, exit_prefix=explicit)
    assert a.status == b.status == "success" and np.array_equal(a.poses, b.poses) and a.exit_prefix == b.exit_prefix


def test_the_core_reads_the_keepouts_prefix_from_the_tracker():
    from types import SimpleNamespace

    from forklift_core.control.path_tracking import RearAxlePathTracker, TrackerConfig
    from forklift_core.mission.core import MissionCore
    from forklift_core.planning.pallet_mission import keepout_exit_prefix, plan_transport_leg

    sys.path.insert(0, str(ROOT / "tests/unit/planning"))
    import test_lidar_keepout as t

    geometry = t.keepout(0.10)
    exit_ = keepout_exit_prefix(t.scenario(), geometry, t.CONFIG.clearance_m)
    plan = plan_transport_leg(t.scenario(), t.EXTRACTED, t.CONFIG, geometry=geometry, exit_prefix=exit_)
    core = MissionCore.__new__(MissionCore)
    core.hooks = SimpleNamespace(exit_prefix=exit_)
    core.geometry = geometry
    core.trackers = {"transport": RearAxlePathTracker(plan.poses, plan.directions, plan.curvatures_inv_m,
                                                      TrackerConfig())}
    assert core.exit_prefix_m() == pytest.approx(plan.exit_prefix["length_m"]) and core.exit_prefix_m() > 0


def test_the_scene_tool_names_every_keepout_apart_and_refuses_another_keepouts_result(tmp_path, monkeypatch):
    # Codex P1c review P2: 0.151 and 0.154 shared "keep0.15"; a worker took the other value's result
    from tools import riibotics_isaac_scenes as tool

    assert tool._result_name("trailer", 0.0, 0.151) != tool._result_name("trailer", 0.0, 0.154)
    assert tool._result_name("trailer", 0.0, 0.0) == "trailer.plan.json"  # unchanged without a keepout
    import hashlib
    import json

    scene = tmp_path / "trailer.json"
    scene.write_text("{}")
    out = tmp_path / "out"
    out.mkdir()

    class Done:  # a child that "wrote" a result for another keepout
        exitcode = 0

        def __init__(self, target, args):
            path, out_, grow, keep = args
            (out_ / tool._result_name("trailer", grow, keep)).write_text(json.dumps(
                {"scene_sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "lidar_keepout_m": 0.2,
                 "outcome": "success"}))

        def start(self): pass

        def join(self, timeout=None): pass

        def is_alive(self): return False

    monkeypatch.setattr(tool.multiprocessing, "get_context", lambda _: type("C", (), {"Process": Done}))
    assert tool._plan_worker((scene, out, 10.0, 0.0, 0.15))[1] == "stale_result"
