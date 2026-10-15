"""M-a1 slice 4b-iii: an observation without a detection goes to the next candidate, moved from the runner
(behaviour, state, port requests and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning import PlanResult

REAR = np.array([1.0, 2.0, 0.0])


@dataclass(frozen=True)
class Config:
    clearance_m: float = 0.3


def plan(success=True, status=None):
    if not success:
        return PlanResult(False, status or "no_path", np.empty((0, 3)), np.empty(0, np.int8), np.empty(0), 0.0, 0)
    return PlanResult(True, "success", np.array([[1.0, 2.0, 0.0], [3.0, 2.0, 0.0]]), np.ones(2, np.int8),
                      np.zeros(2), 2.0, 0, analytic_expansion_interval=3, search_attempts=((3, "success"),))


class World:
    def __init__(self, *, plans=(), waypoints=((5.0, 1.0, 0.0), (6.0, 1.0, 0.0)), cursor=0, near=None,
                 repeat=0):
        self.calls = []
        plans_ = list(plans)
        clock = iter(range(1000, 2000))

        def log(name):
            return lambda *a, **k: self.calls.append((name, a, k))

        def require(ok, reason):
            self.calls.append(("require", (bool(ok), reason), {}))
            if not ok:
                raise RuntimeError(reason)

        self.hooks = SimpleNamespace(
            require=require, wall=lambda: float(next(clock)), grid_world=lambda sc: ("world", sc),
            grid_kwargs=lambda *a: {}, set_inset=log("set_inset"),
            plan_observation_leg=lambda *a, **k: self.calls.append(("plan_observation_leg", a, k)) or plans_.pop(0),
            apply_tracker_profile=log("apply_tracker_profile"), realign_install=log("realign_install"),
            record_tracker_configs=log("record_tracker_configs"), echo=lambda *a: None,
            record_json=lambda v: str(v), snapshot=lambda *a: None,
        )
        self.ms = MissionState(phase="observe", phase_started=0.0, next_candidate_index=cursor)
        self.state = {"transitions": [], "observation_candidates": [], "planning_wall_s": 0.5}
        if near is not None:
            self.state["near_capture"] = {"status": near}
        settings = {"tracker_curvature_inv_m": 0.45, "drive_acceleration_mps2": 0.5}
        self.core = MissionCore(
            self.ms, state=self.state, obstacle=None, slam=None, pocket={"check": None}, paths={}, trackers={},
            geometry="G", settings=settings, pallet_geometry=None, hooks=self.hooks,
            options=SimpleNamespace(planner_config=Config(), scenario="SC", pickup_bounds="B",
                                    speeds={"observe": 0.35}, cusp_heading_hold_rad=0.08,
                                    observation_waypoints=list(waypoints), repeat_captures=repeat))
        # Moved into MissionCore (M-a1 slice 6e): this test's stand-ins go on the instance.
        for name_ in ['control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
            if hasattr(self.hooks, name_):
                setattr(self.core, name_, getattr(self.hooks, name_))

    def names(self):
        return [c[0] for c in self.calls if c[0] != "require"]

    def fail(self, t=30.0):
        attempt = {}
        stop = self.core.observe_failed(t, REAR, attempt, 2, SimpleNamespace(status="no_detection", reason="few"))
        return stop, attempt


def test_the_first_reachable_candidate_is_installed_as_the_observe_leg():
    w = World(plans=[plan(False), plan()])
    stop, attempt = w.fail()
    assert stop is False and attempt["retry_reason"] == "perception_no_detection:few"
    assert w.names() == ["set_inset", "plan_observation_leg", "plan_observation_leg", "apply_tracker_profile",
                         "realign_install", "record_tracker_configs"]
    assert w.calls[0][1] == ("인식 실패 #2 · 재관측 이동", "사유: few")
    _, args, kwargs = [c for c in w.calls if c[0] == "plan_observation_leg"][1]
    assert args[0] == ("world", "SC") and (args[1].x_m, args[1].y_m) == (6.0, 1.0) and args[2] == Config()
    assert kwargs["start_rear"].x_m == 1.0 and kwargs["extended"] is False and kwargs["pickup_bounds"] == "B"
    assert [c["candidate_index"] for c in w.state["observation_candidates"]] == [0, 1]
    assert w.state["observation_candidates"][1]["search_attempts"] == [[3, "success"]]
    assert w.state["observation_waypoint_selected"] == [6.0, 1.0, 0.0] and w.ms.next_candidate_index == 2
    assert w.state["planning_status"] == "success" and w.state["planning_wall_s"] == 1.5
    tracker = w.core.trackers["observe"]
    assert tracker.config.cruise_speed_mps == 0.35 and tracker.config.cusp_heading_pending_rad == 0.08
    assert tracker.config.yaw_tolerance_rad == 0.05 and tracker.config.max_cross_track_error_m == 0.35
    assert w.ms.phase_started == 30.0 and w.calls[-2][1] == ("observe_candidate",)


def test_a_near_capture_without_a_detection_fails_outright():
    w = World(near="pending")
    with pytest.raises(RuntimeError, match="near_capture_failed:perception_no_detection:few"):
        w.fail()
    assert w.state["near_capture"]["status"] == "failed" and w.names() == []


def test_every_candidate_exhausted_ends_the_run_with_their_statuses():
    w = World(plans=[plan(False, "no_path"), plan(False, "invalid_goal")])
    with pytest.raises(RuntimeError, match="observe:all_candidates_exhausted:perception_no_detection:few:no_path;invalid_goal"):
        w.fail()
    assert w.state["planning_status"] == "all_candidates_exhausted"
    w = World(cursor=2)  # nothing left to try
    with pytest.raises(RuntimeError, match="all_candidates_exhausted"):
        w.fail()


def test_the_repeat_diagnostic_ends_the_loop_when_no_candidate_is_reachable():
    w = World(plans=[plan(False), plan(False)], repeat=5)
    stop, _ = w.fail()
    assert stop is True and w.ms.phase == "repeat_target_not_reached"
    assert w.state["repeat_capture"] == {"status": "repeat_target_not_reached", "attempts_made": 2,
                                         "reason": "candidates_exhausted"}


def test_the_search_stops_at_the_first_reachable_candidate():
    w = World(plans=[plan(), plan()])
    w.fail()
    assert w.ms.next_candidate_index == 1 and len(w.state["observation_candidates"]) == 1
    assert w.state["observation_waypoint_selected"] == [5.0, 1.0, 0.0]
