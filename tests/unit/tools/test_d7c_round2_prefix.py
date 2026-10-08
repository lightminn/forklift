"""Plan D7c (Codex D7 Isaac review P1, re-review P1): the whole docking straight can pass its
dry run and its sweep while the path actually driven -- the straight cut at the round-2 stop,
judged at the stop's tolerance -- fails either. Isaac seed 1 (--d7-planner): 0.118 m lateral,
accepted, then a heading stop at round 2."""

import importlib.util
import sys
from dataclasses import replace

import numpy as np
import yaml

from forklift_core.control.rollout import bicycle_rollout
from forklift_core.planning import Bounds, Footprint, Pose2D, Rectangle, collision_free_pose
from forklift_core.planning.pallet_mission import straight_from_pose
from tools.d7c_reapproach_sweep import ROOT, stop_tracker_config

SPEC = importlib.util.spec_from_file_location("docking_retry", ROOT / "sim/isaac/docking_retry.py")
RETRY = importlib.util.module_from_spec(SPEC)
sys.modules["docking_retry"] = RETRY
SPEC.loader.exec_module(RETRY)

SETTINGS = yaml.safe_load((ROOT / "config/isaac_transport_measured.yaml").read_text())
STOP = stop_tracker_config(SETTINGS)
TRANSPORT = replace(STOP, position_tolerance_m=0.008, yaw_tolerance_rad=0.02)
LINE, GOAL = Pose2D(0.0, 0.0, 0.0), Pose2D(1.5, 0.0, 0.0)
LOADED = Footprint(1.56, 0.17, 0.40)  # dls08_measured loaded outline
BOUNDS = Bounds(-10, 10, -10, 10)


def straight_from(current):
    straight, _ = straight_from_pose(current, LINE, GOAL, max_lateral_m=0.12, max_yaw_rad=0.08, min_length_m=0.3)
    return straight, (current.x_m, current.y_m, current.yaw_rad)


def clear_of(cells):
    return lambda pose: collision_free_pose(np.asarray(pose), cells, LOADED, BOUNDS)


def test_a_heading_miss_at_the_stop_is_refused_where_the_whole_straight_passes():
    straight, start = straight_from(Pose2D(-0.0284, -0.1181, 0.0273))
    whole = bicycle_rollout(straight.poses, straight.directions, straight.curvatures_inv_m, TRANSPORT, start)
    assert whole.status == "arrived" and abs(whole.yaw_error_rad) <= 0.75 * 0.02
    ok, record = RETRY.round_two_stop_check(straight, start, STOP, 0.6)
    assert not ok and record["status"] == "failed" and abs(record["yaw_error_rad"]) > 0.05
    assert record["length_m"] == pytest_approx(0.9, 0.01)


def test_a_sweep_hit_on_the_stop_path_is_refused_where_the_whole_sweep_is_clear():
    # Codex re-review counterexample: a 5 cm grid cell (corner 2.22620976, 0.45107184) the cut
    # path's front corner grazes at 0.034 rad. The two sweeps differ by millimetres here, so
    # the case is a graze by construction -- what matters is that the stop path is swept.
    straight, start = straight_from(Pose2D(-0.0284, -0.05, 0.0))
    clear = clear_of([Rectangle(2.22620976 + 0.025, 0.45107184 + 0.025, 0.05, 0.05, 0.0)])
    whole = bicycle_rollout(straight.poses, straight.directions, straight.curvatures_inv_m, TRANSPORT, start)
    assert whole.status == "arrived" and all(clear(tuple(p)) for p in whole.trajectory)
    ok, record = RETRY.round_two_stop_check(straight, start, STOP, 0.6, clear=clear)
    assert not ok and record["status"] == "arrived" and not record["swept_clear"]


def test_a_small_offset_passes():
    straight, start = straight_from(Pose2D(-0.03, -0.05, 0.0))
    ok, record = RETRY.round_two_stop_check(straight, start, STOP, 0.6, clear=clear_of([]))
    assert ok and record["swept_clear"] and abs(record["yaw_error_rad"]) <= 0.05


def test_a_straight_without_a_round_two_stop_is_not_judged():
    straight, start = straight_from(Pose2D(1.0, 0.0, 0.0))  # 0.5 m left: no 0.6 m prefix
    assert RETRY.round_two_stop_check(straight, start, STOP, 0.6) == (True, None)


def test_the_runner_refuses_on_it_only_with_the_flag():
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    call = source.index("DOCKING_RETRY.round_two_stop_check(")
    guard = source.rindex("if args.d7_docking and round_number == 1 and result.accepted:", 0, call)
    window = source[guard : call + 400]
    assert "clear=pose_clear" in window and "if not stop_ok:\n                        path = None" in window
    # The whole straight's sweep and the stop path's use one pose check.
    assert "swept_clear = all(pose_clear(pose) for pose in dry.trajectory)" in source
    assert "final_keep = ROUND2_KEEP_M" in source


def pytest_approx(value, tol):
    import pytest

    return pytest.approx(value, abs=tol)
