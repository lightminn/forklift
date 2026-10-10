import math

import numpy as np
import pytest

from tools import d8c_replay as RP


class Latest:
    def __init__(self, held, width=0.2275, bound=None, stamp_s=1.0):
        self.held = held
        self.stamp_s = stamp_s
        self.source = "front"
        self.bound = bound or {"wall_m": 0.001, "yaw_rad": 0.001}

        class Obs:
            pass

        self.observation = Obs()
        for side in ("left", "right"):
            pocket = Obs()
            pocket.width_m = width
            setattr(self.observation, side, pocket)


class Bounds:
    max_latency_s = 0.1

    def odometry(self, age):
        return None if age > 10 else (0.002, 0.001)


def truth_at(x, half=0.18625, width=0.2275, yaw=0.0):
    axis = np.array([math.cos(yaw), math.sin(yaw)])
    lateral = np.array([-axis[1], axis[0]])
    out = {"yaw": yaw}
    for side, sign in (("left", 1.0), ("right", -1.0)):
        centre = np.array([x, 0.0]) + sign * half * lateral
        out[side] = np.r_[centre, 0.061]
        out[f"{side}_walls"] = {
            "outer": np.r_[centre + sign * width / 2 * lateral, 0.061],
            "inner": np.r_[centre - sign * width / 2 * lateral, 0.061],
        }
    return out


def held_at(x, dy=0.0, half=0.18625, yaw=0.0):
    return {"left": (x, half + dy, 0.061), "right": (x, -half + dy, 0.061), "yaw": yaw}


def test_an_exact_estimate_has_no_wall_error():
    w = RP.wall_check(
        Latest(held_at(1.5)),
        np.zeros(3),
        truth_at(1.5),
        (0.0, 0.33),
        Bounds(),
        1.0,
        -0.34,
        5e-5,
    )
    assert w["ratio"] == pytest.approx(0.0, abs=1e-12)


def test_a_lateral_offset_is_judged_against_the_erosion_at_the_farthest_wall_point():
    # 2 mm offset; b_t = 1 mm wall + depth x 1 mrad + 2 mm + rho_max x 1 mrad + 0.05 mm
    depths = (0.0, 0.33)
    w = RP.wall_check(
        Latest(held_at(1.5, dy=0.002)),
        np.zeros(3),
        truth_at(1.5),
        depths,
        Bounds(),
        1.0,
        -0.34,
        5e-5,
    )
    rho_max = math.hypot(1.5 + 0.33 + 0.34, 0.18625 + 0.2275 / 2 + 0.002)
    assert w["rho_m"] == pytest.approx(rho_max)
    assert w["error_m"] == pytest.approx(0.002)
    assert w["allowed_m"] == pytest.approx(
        0.001 + 0.002 + rho_max * 0.001 + 5e-5
    )  # the face point is the worst


def test_a_yaw_error_grows_with_depth():
    yaw = 0.01
    w = RP.wall_check(
        Latest(held_at(1.5, yaw=yaw)),
        np.zeros(3),
        truth_at(1.5),
        (0.33,),
        Bounds(),
        1.0,
        -0.34,
        5e-5,
    )
    assert w["error_m"] == pytest.approx(
        0.33 * math.sin(yaw) + 0.2275 / 2 * (1 - math.cos(yaw)), abs=2e-4
    )


def test_an_age_past_the_table_is_reported_unbounded():
    w = RP.wall_check(
        Latest(held_at(1.5), stamp_s=0.0),
        np.zeros(3),
        truth_at(1.5),
        (0.0,),
        Bounds(),
        20.0,
        -0.34,
        5e-5,
    )
    assert w["unbounded"]


def test_the_carried_estimate_follows_the_base_pose():
    walls = RP.estimate_walls(Latest(held_at(2.0)), np.array([0.5, 0.0, 0.0]))
    assert walls["left"]["outer"][0] == pytest.approx(1.5)


def test_the_verdict_needs_every_check():
    good = {
        "mismatches": 0,
        "handoff": {"index": 1},
        "handoff_ok": True,
        "provenance": {"speed_mps": 0.08},
        "lost_ticks_after_arm": 0,
        "armed": {"lost": False},
        "wall_violations": 0,
        "wall_unbounded_ticks": 0,
        "start": {"ok": True},
    }
    assert RP.verdict([good])["pass"]
    for key, value in (
        ("mismatches", 1),
        ("handoff", None),
        ("lost_ticks_after_arm", 2),
        ("wall_violations", 1),
        ("start", {"ok": False}),
        ("armed", None),
    ):
        assert not RP.verdict([dict(good, **{key: value})])["pass"]
    fast = dict(good, provenance={"speed_mps": 0.3}, lost_ticks_after_arm=5)
    assert RP.verdict([fast])["pass"]  # (c) only for the operating speeds


def test_budget_stops_count_gaps_across_arming_and_at_the_end():
    # Codex S3 review counterexample: arrivals 1.0, 1.2, 1.3 s, armed at 1.00833, end 1.49167
    assert (
        RP.budget_stops([1.0, 1.2, 1.3], 1.00833, 1.49167, 0.1) == 2
    )  # 1.0 -> 1.2 and 1.3 -> end
    assert RP.budget_stops([1.0, 1.1, 1.2], 1.05, 1.25, 0.1) == 0
    assert RP.budget_stops([1.0, 1.2], None, 1.3, 0.1) == 0  # never armed
    # the near capture's arrival leads the list: a gap before the first replayed acceptance
    assert RP.budget_stops([0.9], 1.0, 1.5, 0.1) == 1
    assert RP.budget_stops([0.9, 1.4, 1.5], 1.0, 1.55, 0.1) == 1
    # Codex S3 3rd review: a gap closing at the arming tick is not inside the section, and
    # an age already old at entry keeps counting to the end
    assert RP.budget_stops([0.9, 1.2, 1.3], 1.2, 1.35, 0.1) == 0
    assert RP.budget_stops([0.9], 1.1, 1.15, 0.1) == 1


def test_the_provenance_covers_every_loaded_repository_module():
    import json as _json
    import os
    import subprocess
    import sys
    from pathlib import Path

    sources = RP.repo_sources()
    assert (
        "src/forklift_core/sensors/rgbd.py" in sources
        and "tools/scene_rig.py" in sources
    )
    assert "tools/d8c_replay.py" in RP.uncovered_modules(
        {}
    )  # an empty provenance covers nothing
    # A clean process that loads what a replay loads: nothing left uncovered (other tests
    # in this process load their own tools, so the check runs apart).
    root = Path(RP.__file__).resolve().parents[1]
    code = (
        "import json\n"
        "from tools import d8c_replay as RP\n"
        "from forklift_core.perception import near_field_bounds, near_field_tracking, pallet_geometry\n"
        "from forklift_core.perception import pallet_prior, pocket_detector, roof_tracking\n"
        "from sim.isaac import insertion_geometry, perception_adapter\n"
        "print(json.dumps(RP.uncovered_modules(RP.repo_sources())))\n"
    )
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(root), str(root / "src")]))
    out = subprocess.run(
        [sys.executable, "-c", code],
        cwd=root,
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )
    assert _json.loads(out.stdout.strip().splitlines()[-1]) == []


def test_the_estimate_errors_use_the_truth_axes():
    e = RP.estimate_errors(Latest(held_at(1.5, dy=0.003)), np.zeros(3), truth_at(1.502))
    assert e["lateral_m"] == pytest.approx(0.003) and e["along_m"] == pytest.approx(
        0.002
    )


def test_the_capture_pose_is_recovered_from_the_map_estimate():
    from forklift_core.perception.pocket_observation import Pocket, PocketObservation
    from sim.isaac import perception_adapter as PA

    obs = PocketObservation(
        1,
        "synthetic",
        "base_link",
        "synthetic",
        "valid",
        Pocket((3.2, 0.19, 0.061), 0.23, 0.078),
        Pocket((3.21, -0.18, 0.061), 0.23, 0.078),
        0.027,
        None,
        None,
        None,
    )
    pose = (-0.314, 1.89, -0.1507)
    site = PA.estimate_world_pallet_site(
        PA.estimate_pallet_center_m(obs, 0.6), obs.insertion_yaw_rad, pose[:2], pose[2]
    )
    got = RP.capture_pose(
        obs, 0.6, {"x_m": site.x_m, "y_m": site.y_m, "yaw_rad": site.yaw_rad}
    )
    assert got == pytest.approx(pose, abs=1e-12)
    assert RP.start_check(got, (pose[0] + 0.0008, pose[1], pose[2]))["ok"]
    assert not RP.start_check(got, (pose[0] + 0.002, pose[1], pose[2]))["ok"]
