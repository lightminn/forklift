"""Plan 2026-10-21 task B: --near-capture-redraw always -- the near capture's stage-1 re-draw without the near-field
tracking (its correction-point check has nothing to check there); `on` keeps today's near-tracking-only behaviour."""

from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning import Pose2D
from forklift_core.planning.pallet_mission import SyntheticMissionGeometry


@dataclass
class Cfg:
    cruise_speed_mps: float = 0.6


@dataclass
class Side:
    width_m: float = 0.23


@dataclass
class Obs:
    left: Side
    right: Side
    insertion_yaw_rad: float = 0.05


def core(*, near, redraw):
    calls = []
    plans = {"approach": "A", "insert": "I", "extract": "E"}

    def traced(world, rear_pose, target, config, **kw):
        calls.append(("redraw_docking_traced", kw.get("max_yaw_rad")))
        return plans, {"refused": None}, ["TRAJ1", "TRAJ2"]

    hooks = SimpleNamespace(
        correction_grid=lambda *a: calls.append(("correction_grid",)) or {},
        grid_world=lambda sc: "WORLD", redraw_docking_traced=traced,
        tracking_frame=lambda: calls.append(("tracking_frame",)) or SimpleNamespace(fork_tip_x_m=0.9))
    near_tracking = SimpleNamespace(
        CORRECTION_MAX_YAW_RAD=0.12,
        rollout_face_distance=lambda *a: calls.append(("rollout_face_distance",)) or 1.0,
        correction_points_check=lambda *a: calls.append(("correction_points_check",)) or ([], True))
    adapter = SimpleNamespace(
        estimate_pallet_center_m=lambda obs, depth: (2.0, 0.0),
        estimate_world_pallet_site=lambda xy, yaw, base_xy, base_yaw: Pose2D(3.0, 0.0, 0.0))
    options = SimpleNamespace(near_tracking=near_tracking, adapter=adapter, pickup_bounds=None, planner_config="CFG",
                              scenario="SC", rear_axle_offset_m=0.37, near_capture_redraw=redraw)
    c = MissionCore(MissionState(phase="observe", phase_started=0.0), state={}, obstacle=None, slam=None,
                    pocket={"planning_pickup": Pose2D(3.0, 0.0, 0.0)}, paths={}, trackers={},
                    geometry=SyntheticMissionGeometry(), settings={}, pallet_geometry=None, hooks=hooks,
                    near=near, options=options)
    return c, calls


@pytest.mark.parametrize("near, redraw, allowed", [
    (None, "on", False), (None, "always", True), (None, "off", False),
    ({}, "on", True), ({}, "always", True), ({}, "off", False)])
def test_who_may_redraw_at_the_near_capture(near, redraw, allowed):
    c, _ = core(near=near, redraw=redraw)
    assert c._near_capture_redraw_allowed() is allowed


def test_without_the_near_field_tracking_the_redraw_skips_the_correction_points():
    c, calls = core(near=None, redraw="always")
    obs = Obs(Side(), Side())
    plans, entry = c.near_capture_redraw(np.array([0.0, 0.0, 0.05]), (0.37, 0.0, 0.05), obs, 4, Cfg(), Cfg(), 50.0)
    assert plans == {"approach": "A", "insert": "I", "extract": "E"}
    names = [n[0] for n in calls]
    assert "redraw_docking_traced" in names and ("redraw_docking_traced", 0.12) in calls
    for absent in ("tracking_frame", "rollout_face_distance", "correction_points_check"):
        assert absent not in names, absent
    assert entry["tracking"] is False and "correction_points" not in entry and entry["accepted"] is True
