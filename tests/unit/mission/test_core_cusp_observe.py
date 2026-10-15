"""M-a1 slice 4a-iii: the cusp and observe-leg recoveries moved from the runner (behaviour, state, port requests
and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

import forklift_core.mission.core as core_module
from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning import PlanResult

REAR = np.array([1.0, 2.0, 0.0])


@dataclass(frozen=True)
class Tracking:
    status: str = "tracking"
    speed_mps: float = 0.0
    failure: str | None = None
    at_cusp: bool = False
    off_path: bool = False
    cusp_pending: bool = False
    position_error_m: float = 0.01
    yaw_error_rad: float = 0.06
    progress_m: float = 0.4


@dataclass(frozen=True)
class Config:
    clearance_m: float = 0.3


def plan(success=True):
    if not success:
        return PlanResult(False, "no_path", np.empty((0, 3)), np.empty(0, np.int8), np.empty(0), 0.0, 0)
    return PlanResult(True, "success", np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]), np.ones(2, np.int8),
                      np.zeros(2), 1.0, 0)


class HoldTracker:
    def __init__(self, calls, remaining=0.0, window=0.05, release=True):
        self.calls, self.remaining, self.window, self.release = calls, remaining, window, release
        self.leg_index, self.config = 1, TrackerConfig()

    def leg_remaining_m(self):
        return self.remaining

    def leg_end_window_m(self):
        return self.window

    def release_cusp(self, ok):
        self.calls.append(("release_cusp", (ok,), {}))
        return self.release

    def fail_pending(self):
        self.calls.append(("fail_pending", (), {}))

    def update(self, rear, speed, dt):
        self.calls.append(("update", (tuple(rear),), {}))
        return Tracking(status="tracking", speed_mps=0.1)

    def remaining_to_goal_m(self):
        return 2.0


class World:
    def __init__(self, phase="transport", *, slam_mode="tracking", stop_now=True, docking=None, cusp_replans=0,
                 plans=(), stopped=True, realign=False, observe_replans=0, holds_observe=False, slam=True,
                 velocity=(0.0, 0.0), yaw_rate=0.0):
        self.calls = []
        plans_ = list(plans)
        clock = iter(range(1000, 2000))
        self.runner_rear = REAR

        def log(name, value=None):
            def call(*args, **kwargs):
                self.calls.append((name, args, kwargs))
                return value() if callable(value) else value
            return call

        def require(ok, reason):
            self.calls.append(("require", (bool(ok), reason), {}))
            if not ok:
                raise RuntimeError(reason)

        self.holds = SimpleNamespace(held=log("held", lambda: {}), decide=log("decide"), tick=log("holds.tick"))
        self.slam = {"tracker": SimpleNamespace(mode=slam_mode), "stop_now": stop_now, "docking": dict(docking or {}),
                     "control": [[0.0, 0.0, 0.0, 0.0]], "transport_scenario": "SC_T"} if slam else None
        self.hooks = SimpleNamespace(
            require=require, wall=lambda: float(next(clock)), path_record=lambda p: {"status": p.status},
            write_paths=log("write_paths"), show_path=log("show_path"),
            start_docking_retry=log("start_docking_retry"), slam_rear=log("slam_rear", lambda: np.array([3.0, 4.0, 0.1])),
            set_rear=lambda v: setattr(self, "runner_rear", v), arm_docking=log("arm_docking"),
            grid_world=lambda sc: ("world", sc),
            grid_kwargs=lambda *a: self.calls.append(("grid_kwargs", a, {"own": tuple(self.runner_rear)})) or {},
            plan_transport_leg=lambda *a, **k: self.calls.append(("plan_transport_leg", a, k)) or plans_.pop(0),
            plan_observation_leg=lambda *a, **k: self.calls.append(("plan_observation_leg", a, k)) or plans_.pop(0),
            realign_active=lambda: realign, realign_install=log("realign_install"),
            cusp_holds=lambda: self.holds,
            cusp_monitor=lambda: SimpleNamespace(holds_observe=lambda: holds_observe),
            cusp_heading_motion=lambda v: {"speed": v},
            truth_velocity=lambda: np.array([*velocity, 0.0]), truth_yaw_rate=lambda: yaw_rate,
        )
        self.tracker = HoldTracker(self.calls)
        self.ms = MissionState(phase=phase, phase_started=0.0)
        self.ms.last_tracking = Tracking()  # follow() sets it every tick before these rules run
        self.state = {"transitions": [], "cusp_replans": [{}] * cusp_replans, "paths": {},
                      "observe_replans": [{}] * observe_replans, "observe_replan_leg": {"start": 0, "serial": 7}}
        self.obstacle = {"progress": {"x": 1}}
        self.core = MissionCore(
            self.ms, state=self.state, obstacle=self.obstacle, slam=self.slam, pocket={"check": None},
            paths={phase: plan()}, trackers={phase: self.tracker}, geometry="G", settings={}, pallet_geometry=None,
            hooks=self.hooks,
            options=SimpleNamespace(planner_config=Config(), travel_config=None, scenario="SC", pickup_bounds="B",
                                    max_cusp_replans=2, cusp_stop_needed=12, cusp_stop_yaw_rate_radps=0.02,
                                    cusp_heading_hold_phases=("observe", "return_home"),
                                    cusp_heading_hold_position_m=0.03, cusp_heading_hold_rad=0.08,
                                    observe_replans_per_leg=2))
        # Docking/withdrawal helpers are MissionCore methods since slice 6b; this test's stand-ins go on the instance (6c: the release and escape helpers too).
        for name_ in ['dock_at_delivery_straight', 'arm_docking', 'release_and_carry', 'transport_replan', 'start_docking_retry', 'release_and_stand', 'release_after_backoff', 'slam_release', 'escape_leg', 'find_escape', 'hold_count', 'near_tick', 'near_redraw', 'near_capture_redraw', 'near_protection_prediction', 'near_install', 'start_pickup_retry', 'control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
            if hasattr(self.hooks, name_):
                setattr(self.core, name_, getattr(self.hooks, name_))
        # The cusp rules are MissionCore methods since slice 6a; stand-ins on the instance as before.
        self.core.cusp_heading_end = lambda tr: {"end": tr.status}
        self.core.cusp_heading_offset = lambda tracker, progress, rear: (0.0, 0.0)
        self.core.cusp_stopped = lambda: stopped
        self.core.cusp_end_recovered = lambda goal: True
        self.core.cusp_leg_checker = lambda: None
        self.core.cusp_sensors = lambda *a: None

    def names(self):
        return [c[0] for c in self.calls if c[0] not in ("require", "grid_kwargs")]

    def tick(self, tracking, *, d7_docking=None, d7_owned=False, t=50.0, step=600):
        return self.core.cusp_and_observe_recovery(t, REAR, tracking, 0.0, 1 / 120, d7_docking or {}, d7_owned, step)


CUSP_MISS = Tracking(status="failed", failure="endpoint_heading", at_cusp=True)


def test_a_transport_cusp_miss_brakes_until_stopped_for_12_ticks_then_replans_the_leg():
    w = World(plans=[plan()])
    for _ in range(11):
        out = w.tick(CUSP_MISS)
        assert out.tracking.status == "braking"
    assert w.ms.cusp_stop_ticks == 11 and w.names() == []
    out = w.tick(CUSP_MISS)
    assert w.names() == ["plan_transport_leg", "write_paths", "show_path", "arm_docking"]
    _, args, kwargs = [c for c in w.calls if c[0] == "plan_transport_leg"][0]
    assert args[0] == ("world", "SC_T") and args[1].x_m == 1.0 and args[2] == Config()
    rec = w.state["cusp_replans"][-1]
    assert (rec["loop_step"], rec["planning_wall_s"], rec["status"]) == (600, 1.0, "success")
    assert rec["path"] == {"status": "success"} and rec["replaced_path"] == {"status": "success"}
    assert w.state["planning_wall_s"] == 1.0 and w.obstacle["progress"] == {} and w.ms.phase_started == 50.0
    assert w.ms.cusp_stop_ticks == 0 and w.core.trackers["transport"] is not w.tracker
    assert w.calls[[c[0] for c in w.calls].index("show_path")][1][1:] == ("Transport", (1.0, 0.65, 0.04))


def test_a_failed_cusp_replan_keeps_the_path_and_two_is_the_budget_unless_d7_owns_it():
    w = World(plans=[plan(False)])
    w.ms.cusp_stop_ticks = 11
    out = w.tick(CUSP_MISS)
    assert w.state["cusp_replans"][-1]["path"] is None and "write_paths" not in w.names()
    assert w.core.trackers["transport"] is w.tracker and out.tracking == CUSP_MISS
    w = World(cusp_replans=2)
    assert w.tick(CUSP_MISS).tracking == CUSP_MISS and w.ms.cusp_stop_ticks == 0  # no budget: not this rule


def test_without_slam_the_stop_is_judged_on_the_true_speed_and_yaw_rate():
    for velocity, yaw_rate, counts in (((0.0, 0.0), 0.0, 1), ((0.2, 0.0), 0.0, 0), ((0.0, 0.0), 0.05, 0)):
        w = World(slam=False, velocity=velocity, yaw_rate=yaw_rate)
        w.tick(CUSP_MISS)
        assert w.ms.cusp_stop_ticks == counts
    w = World(slam=False)
    w.tick(Tracking(status="failed", failure="endpoint_heading", at_cusp=True, speed_mps=0.1))
    assert w.ms.cusp_stop_ticks == 0  # still commanded: not standing


def test_under_d7_a_cusp_miss_is_a_docking_retry_and_the_runner_sees_the_released_pose():
    w = World(docking={"reapproach": True}, cusp_replans=5)
    w.ms.cusp_stop_ticks = 11
    out = w.tick(CUSP_MISS, d7_docking={"reapproach": True}, d7_owned=True)
    assert w.names() == ["start_docking_retry", "slam_rear", "update"]
    assert w.calls[0][1] == (50.0, "d", "transport_recovery_after_docking")
    np.testing.assert_array_equal(out.rear, [3.0, 4.0, 0.1])
    np.testing.assert_array_equal(w.runner_rear, [3.0, 4.0, 0.1])
    assert w.slam["control"][-1] == [0.0, 3.0, 4.0, 0.1] and w.ms.last_tracking == out.tracking


def test_a_cusp_miss_after_a_docking_match_without_d7_ends_the_run():
    w = World(docking={"accepted_any": True})
    w.ms.cusp_stop_ticks = 11
    with pytest.raises(RuntimeError, match="transport_recovery_after_docking"):
        w.tick(CUSP_MISS)


PENDING = Tracking(status="tracking", cusp_pending=True)


@pytest.mark.parametrize("judged, release, decision", [
    (SimpleNamespace(pose_ok=True, sensor_ok=True, prefix_occupied=False, record={"r": 1}), True, "released"),
    (SimpleNamespace(pose_ok=True, sensor_ok=True, prefix_occupied=False, record={"r": 1}), False, "release_refused"),
    (SimpleNamespace(pose_ok=False, sensor_ok=True, prefix_occupied=False, record={"r": 1}), True, "judged_out"),
    (SimpleNamespace(pose_ok=True, sensor_ok=True, prefix_occupied=True, record={"r": 1}), True, "judged_out"),
])
def test_a_held_cusp_heading_miss_is_judged_from_here_and_released_or_latched(monkeypatch, judged, release, decision):
    seen = {}
    monkeypatch.setattr(core_module.CUSP_STALL, "judge",
                        lambda tracker, rear, tracking, extra, **k: seen.update(k) or judged)
    w = World("return_home")
    w.tracker.release = release
    out = w.tick(PENDING)
    assert seen["bounds"] == (0.03, 0.08)
    (decide,) = [c for c in w.calls if c[0] == "decide"]
    assert decide[1][1] == decision and decide[1][2]["decided_rear"] == [1.0, 2.0, 0.0]
    assert ("fail_pending" in w.names()) is (decision != "released")
    assert ("holds.tick" in w.names()) is (decision == "released")
    assert out.tracking.status == "tracking" and w.ms.last_tracking == out.tracking
    held = [c for c in w.calls if c[0] == "held"][0]
    assert held[1][1][0] == "return_home" and held[1][2]["rear"] == [1.0, 2.0, 0.0]


def test_the_cusp_heading_hold_waits_standing_and_inside_its_window():
    w = World("observe", stopped=False)
    out = w.tick(PENDING)
    assert out.tracking == PENDING and "decide" not in w.names()  # not standing yet: still held
    w = World("observe")
    w.tracker.remaining = 0.2  # outside the leg end window
    w.tick(PENDING)
    assert [c[1][1] for c in w.calls if c[0] == "decide"] == ["window"] and "fail_pending" in w.names()
    w = World("observe", slam_mode="holding", realign=True)
    w.tick(PENDING)
    assert [c[1][1] for c in w.calls if c[0] == "decide"] == ["realign_held"]
    w = World("transport")
    w.tick(PENDING)
    assert "held" not in w.names()  # the transport does not hold a cusp heading miss


OBSERVE_MISS = Tracking(status="failed", failure="endpoint_heading")


def test_an_observe_heading_miss_brakes_and_once_stopped_replans_the_leg_to_the_same_target():
    w = World("observe", stop_now=False, plans=[plan()])
    out = w.tick(OBSERVE_MISS)
    assert out.tracking.status == "braking" and out.tracking.speed_mps == 0.0 and w.names() == []
    w.slam["stop_now"] = True
    w.ms.last_tracking = OBSERVE_MISS
    out = w.tick(OBSERVE_MISS)
    assert w.names() == ["plan_observation_leg", "write_paths", "show_path", "realign_install"]
    _, args, kwargs = [c for c in w.calls if c[0] == "plan_observation_leg"][0]
    assert (args[1].x_m, args[1].y_m) == (1.0, 0.0) and kwargs["start_rear"].x_m == 1.0 and kwargs["extended"] is False
    assert w.state["observe_replans"][-1] == {"time_s": 50.0, "rear_pose": [1.0, 2.0, 0.0], "yaw_error_rad": 0.06,
                                             "status": "success", "planning_wall_s": 1.0, "leg_serial": 7,
                                             "budget_left": 1}
    assert w.ms.phase_started == 50.0 and w.calls[-1][1] == ("observe_replan",)


def test_an_observe_cusp_stall_counts_120_standing_ticks_and_the_budget_is_two_per_leg():
    w = World("observe", plans=[plan()])
    stall = Tracking(status="tracking", at_cusp=True)
    for _ in range(119):
        w.tick(stall)
    assert w.ms.observe_stall_ticks == 119 and "plan_observation_leg" not in w.names()
    w.tick(stall)
    assert "plan_observation_leg" in w.names() and w.ms.observe_stall_ticks == 0
    w = World("observe", holds_observe=True)
    w.tick(stall)
    assert w.ms.observe_stall_ticks == 0  # the cusp stall handling holds it: not this rule
    w = World("observe", observe_replans=2)
    assert w.tick(OBSERVE_MISS).tracking == OBSERVE_MISS  # the leg's budget is spent


def test_an_observe_recovery_held_in_a_realignment_ends_the_run_and_a_failed_replan_too():
    w = World("observe", slam_mode="holding", realign=True)
    with pytest.raises(RuntimeError, match="near_capture_realign_held_recovery"):
        w.tick(OBSERVE_MISS)
    w = World("observe", plans=[plan(False)])
    with pytest.raises(RuntimeError, match="observe_replan_failed:no_path"):
        w.tick(OBSERVE_MISS)
    assert w.state["observe_replans"][-1]["status"] == "no_path"
