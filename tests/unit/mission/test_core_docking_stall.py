"""M-a1 slice 4a-ii: the D7c docking stop, the SLAM goal heading miss and the stall replan, moved from the runner
(behaviour, state, port requests and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

from dataclasses import dataclass, replace
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning import PlanResult


@dataclass(frozen=True)
class Tracking:
    status: str = "tracking"
    speed_mps: float = 0.0
    failure: str | None = None
    at_cusp: bool = False
    off_path: bool = False
    position_error_m: float = 0.02
    yaw_error_rad: float = 0.04


@dataclass(frozen=True)
class Config:
    clearance_m: float = 0.3


def plan(success=True):
    if not success:
        return PlanResult(False, "no_path", np.empty((0, 3)), np.empty(0, np.int8), np.empty(0), 0.0, 0)
    return PlanResult(True, "success", np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]), np.ones(2, np.int8),
                      np.zeros(2), 1.0, 0)


def _next_plan(plans_, name):
    assert plans_, f"an unexpected planning call: {name}"  # the fake port expects its calls (M-a2)
    return plans_.pop(0)


class World:
    def __init__(self, phase="transport", *, docking=None, mode="holding", stop_now=True, remaining=0.0,
                 stall_replans=0, plans=(), obstacle=None):
        self.calls = []
        plans_ = list(plans)
        clock = iter(range(1000, 2000))
        self.runner_rear = np.array([1.0, 2.0, 0.0])  # run()'s rear, which the runner's closures read

        def log(name, value=None):
            def call(*args, **kwargs):
                self.calls.append((name, args, kwargs))
                return value() if callable(value) else value
            return call

        def require(ok, reason):
            self.calls.append(("require", (bool(ok), reason), {}))
            if not ok:
                raise RuntimeError(reason)

        tracker_ = SimpleNamespace(mode=mode, applied=(np.array([0.1, 0.0, 0.0]),),
                                   release=lambda **k: self.calls.append(("slam.release", (), k)) or (0.05, 0.01))
        self.slam = {"tracker": tracker_, "stop_now": stop_now, "docking": dict(docking or {}), "holds": [],
                     "control": [[0.0, 0.0, 0.0, 0.0]], "transport_scenario": "SC_T", "return_scenario": "SC_R"}
        self.hooks = SimpleNamespace(
            require=require, wall=lambda: float(next(clock)), path_stats=lambda p: {"length_m": p.length_m},
            path_record=lambda p: {"status": p.status}, write_paths=log("write_paths"), show_path=log("show_path"),
            dock_at_delivery_straight=log("dock_at_delivery_straight"), start_docking_retry=log("start_docking_retry"),
            slam_rear=log("slam_rear", lambda: np.array([3.0, 4.0, 0.1])), slam_odom_base=lambda: "ODOM",
            arm_docking=log("arm_docking"), now_s=lambda: 77.0, grid_world=lambda sc: ("world", sc),
            grid_kwargs=lambda *a: self.calls.append(("grid_kwargs", a, {"own_pose": tuple(self.runner_rear)})) or {},
            set_rear=lambda value: setattr(self, "runner_rear", value),
            plan_transport_leg=lambda *a, **k: self.calls.append(("plan_transport_leg", a, k)) or _next_plan(plans_, "plan_transport_leg"),
            plan_return_leg=lambda *a, **k: self.calls.append(("plan_return_leg", a, k)) or _next_plan(plans_, "plan_return_leg"),
        )
        self.tracker = SimpleNamespace(config=TrackerConfig(), remaining_to_goal_m=lambda: remaining,
                                       update=lambda rear, v, dt: self.calls.append(("update", (tuple(rear),), {}))
                                       or Tracking(status="tracking", speed_mps=0.2))
        self.ms = MissionState(phase=phase, phase_started=0.0)
        self.state = {"transitions": [], "stall_replans": [{}] * stall_replans, "paths": {}}
        self.obstacle = obstacle
        self.core = MissionCore(
            self.ms, state=self.state, obstacle=obstacle, slam=self.slam, pocket={"check": None}, paths={},
            trackers={phase: self.tracker}, geometry="G", settings={}, pallet_geometry=None, hooks=self.hooks,
            options=SimpleNamespace(d7_docking=True, planner_config=Config(), travel_config=None, scenario="SC",
                                    return_to_pose="HOME"))
        # Docking/withdrawal helpers are MissionCore methods since slice 6b; this test's stand-ins go on the instance (6c: the release and escape helpers too).
        for name_ in ['dock_at_delivery_straight', 'arm_docking', 'release_and_carry', 'transport_replan', 'start_docking_retry', 'release_and_stand', 'release_after_backoff', 'slam_release', 'escape_leg', 'find_escape', 'hold_count', 'near_tick', 'near_redraw', 'near_capture_redraw', 'near_protection_prediction', 'near_install', 'start_pickup_retry', 'control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
            if hasattr(self.hooks, name_):
                setattr(self.core, name_, getattr(self.hooks, name_))

    def names(self):
        return [c[0] for c in self.calls if c[0] not in ("require", "grid_kwargs")]

    def tick(self, tracking, t=50.0, speed=0.0):
        return self.core.docking_and_stall(t, np.array([1.0, 2.0, 0.0]), tracking, speed, 1 / 120, True)


def test_a_reapproach_hold_exit_due_brakes_then_runs_on_the_stand():
    """Re-approach hold plan E1 (Codex plan round 2: the order against arrival and the stall rules): moving, an exit
    due brakes to zero with no stall counted; standing, the exit runs before the docking stop and the stall rules;
    an arrival goes on to the docking as before."""
    w = World(docking={"reapproach": True, "status": "armed"}, stop_now=False, remaining=1.0)
    w.core.reapproach_hold_exit_due = lambda t: "correction_gap"
    w.core.reapproach_hold_exit = lambda t, reason: w.calls.append(("hold_exit", (reason,), {}))
    out = w.tick(Tracking(status="tracking", speed_mps=0.2), speed=0.2)
    assert (out.tracking.status, out.tracking.speed_mps) == ("braking", 0.0) and "hold_exit" not in w.names()
    assert w.ms.slam_stall_ticks == 0
    w.slam["stop_now"] = True
    w.tick(Tracking(status="tracking", at_cusp=True))
    assert w.names()[:3] == ["hold_exit", "slam_rear", "update"]
    assert "start_docking_retry" not in w.names() and w.ms.slam_stall_ticks == 0
    assert w.slam["control"][-1][1:4] == [3.0, 4.0, 0.1]
    # an arrival is not stopped for an exit
    w = World(docking={"reapproach": True, "status": "armed"}, stop_now=True)
    w.core.reapproach_hold_exit_due = lambda t: "hold_time"
    w.core.reapproach_hold_exit = lambda t, reason: w.calls.append(("hold_exit", (reason,), {}))
    out = w.tick(Tracking(status="arrived"))
    assert "hold_exit" not in w.names() and out.tracking.status == "arrived"


def test_no_reapproach_hold_exit_leaves_the_tick_as_it_was():
    w = World(docking={"reapproach": True, "status": "armed"}, stop_now=False, remaining=1.0)
    w.core.reapproach_hold_exit_due = lambda t: None
    out = w.tick(Tracking(status="tracking", speed_mps=0.2), speed=0.2)
    assert out.tracking.status == "tracking" and out.tracking.speed_mps == 0.2


MISS = Tracking(status="failed", failure="endpoint_heading")


def test_the_docking_stop_brakes_and_after_a_standing_second_docks_here_as_an_arrival_would():
    w = World(docking={"status": "armed", "round": 2})
    for _ in range(119):
        out = w.tick(MISS)
        assert out.tracking.status == "braking" and out.tracking.speed_mps == 0.0
    assert w.ms.dock_stop_ticks == 119 and w.names() == []
    out = w.tick(MISS)
    assert w.names() == ["dock_at_delivery_straight", "slam_rear", "update"]
    assert w.state["docking_retries"] == [{"trigger": "a", "time_s": 50.0, "counted": False, "round": 2,
                                           "reapproach": False, "position_error_m": 0.02, "yaw_error_rad": 0.04}]
    np.testing.assert_array_equal(out.rear, [3.0, 4.0, 0.1])
    assert w.slam["control"][-1] == [0.0, 3.0, 4.0, 0.1]
    assert out.tracking.status == "tracking" and w.ms.last_tracking == out.tracking and w.ms.dock_stop_ticks == 0


def test_the_docking_stop_counts_only_standing_and_resets_once_the_stop_no_longer_applies():
    w = World(docking={"status": "armed"}, stop_now=False)
    w.tick(MISS)
    assert w.ms.dock_stop_ticks == 0
    w.slam["stop_now"] = True
    w.tick(MISS)
    assert w.ms.dock_stop_ticks == 1
    w.tick(Tracking(status="failed", failure="endpoint_heading", at_cusp=True))  # at a cusp: not the stop
    assert w.ms.dock_stop_ticks == 0
    w = World(docking={"status": "armed"}, remaining=0.0)
    w.tick(Tracking(status="tracking"))  # standing at the end of the straight counts too
    assert w.ms.dock_stop_ticks == 1


def test_a_heading_miss_at_the_goal_brakes_and_replans_after_a_single_standing_tick():
    w = World("return_home", mode="tracking", plans=[plan()])
    out = w.tick(MISS)  # the heading miss jumps the stall count to 119: one more standing tick is 120
    assert w.names() == ["plan_return_leg", "write_paths", "show_path"]  # then the new tracker's first update
    assert w.core.trackers["return_home"] is not w.tracker and w.core.paths["return_home"].status == "success"
    _, args, kwargs = [c for c in w.calls if c[0] == "plan_return_leg"][0]
    assert args[0] == ("world", "SC_R") and args[2:] == ("HOME", Config()) and kwargs["geometry"] == "G"
    rec = w.state["stall_replans"][-1]
    assert rec["phase"] == "return_home" and rec["status"] == "success" and rec["planning_wall_s"] == 1.0
    assert rec["rear_pose"] == [1.0, 2.0, 0.0] and rec["length_m"] == 1.0
    assert w.ms.phase_started == 50.0 and w.ms.slam_stall_ticks == 0 and w.ms.last_tracking is out.tracking  # the new follower's first answer
    assert w.calls[[c[0] for c in w.calls].index("show_path")][1][1:] == ("Return", (0.55, 0.2, 0.85))


def test_a_plain_stall_at_the_path_end_needs_120_standing_ticks():
    w = World("transport", mode="tracking", plans=[plan()], remaining=0.0)
    for _ in range(119):
        w.tick(Tracking(status="tracking", speed_mps=0.0))
    assert w.ms.slam_stall_ticks == 119 and "plan_transport_leg" not in w.names()
    w.tick(Tracking(status="tracking", speed_mps=0.0))
    assert w.names() == ["plan_transport_leg", "write_paths", "show_path", "arm_docking"]
    assert [c for c in w.calls if c[0] == "plan_transport_leg"][0][1][0] == ("world", "SC_T")
    w.slam["stop_now"] = False
    w.tick(Tracking(status="tracking", speed_mps=0.0))
    assert w.ms.slam_stall_ticks == 0


def test_two_stall_replans_at_most_and_none_after_a_docking_match():
    w = World("transport", mode="tracking", stall_replans=2, remaining=0.0)
    out = w.tick(MISS)
    assert out.tracking == MISS  # no third: the heading miss is not braked for a replan any more
    w = World("transport", mode="tracking", docking={"accepted_any": True}, plans=[plan()])
    w.core.options.d7_docking = False  # without D7 the docking does not own the stall
    with pytest.raises(RuntimeError, match="transport_recovery_after_docking"):
        w.tick(MISS)


def test_under_d7_a_stall_after_docking_is_a_docking_retry():
    w = World("transport", mode="tracking", docking={"accepted_any": True, "status": "done"})
    out = w.tick(MISS)
    assert out.d7_owned is True and w.names() == ["start_docking_retry", "slam_rear", "update"]
    assert w.calls[0][1] == (50.0, "c", "transport_recovery_after_docking")
    w = World("transport", mode="tracking", docking={"reapproach": True})
    w.tick(Tracking(status="tracking", speed_mps=0.0, at_cusp=True))  # (d): a re-approach stopped at a cusp
    for _ in range(119):
        w.tick(Tracking(status="tracking", speed_mps=0.0, at_cusp=True))
    assert [c[1][1] for c in w.calls if c[0] == "start_docking_retry"] == ["d"]


def test_a_held_stall_lets_slam_back_in_and_reprojects_the_grid_before_planning():
    layer = SimpleNamespace(refresh=lambda *a, **k: refreshed.append((a, k)))
    refreshed = []
    obstacle = {"layer": layer, "applied": (0.0, 0.0, 0.0), "version": 4,
                "backoff": {"phase": "return_home", "config": TrackerConfig(cruise_speed_mps=0.3)},
                "replan_wait": {"return_home": True}, "progress": {"x": 1}}
    w = World("return_home", mode="holding", plans=[plan()], obstacle=obstacle)
    out = w.tick(MISS)
    assert w.names()[:3] == ["slam.release", "slam_rear", "plan_return_leg"]
    assert w.slam["holds"] == [{"phase": "return_home", "time_s": 50.0, "event": "release_before_stall_replan",
                                "jump_m": 0.05, "jump_rad": 0.01}]
    assert [c[1] for c in w.calls if c[0] == "slam_rear"] == [(77.0,)]  # the physics time, not the tick's
    (args, kwargs), = refreshed
    assert args == (50.0, (0.1, 0.0, 0.0), 5) and kwargs["current_pose"] == (3.0, 4.0, 0.1) and kwargs["loaded"]
    np.testing.assert_array_equal(out.rear, [3.0, 4.0, 0.1])
    assert [c for c in w.calls if c[0] == "plan_return_leg"][0][1][1].x_m == 3.0
    assert "backoff" not in obstacle and obstacle["replan_wait"] == {} and obstacle["progress"] == {}
    assert w.core.trackers["return_home"].config == TrackerConfig(cruise_speed_mps=0.3)  # the leg's own config back


def test_a_failed_stall_replan_ends_the_run_after_recording_it():
    w = World("transport", mode="tracking", plans=[plan(False)])
    with pytest.raises(RuntimeError, match="stall_replan_failed:no_path"):
        w.tick(MISS)
    assert w.state["stall_replans"][-1]["status"] == "no_path" and "write_paths" not in w.names()


def test_the_docking_stop_needs_an_armed_docking():
    w = World(docking={"status": "done"}, stall_replans=2)
    out = w.tick(MISS)
    assert w.ms.dock_stop_ticks == 0 and out.tracking == MISS  # no stop, and no stall replan left either


def test_the_held_stall_replans_grid_sees_the_released_pose():
    """Codex slice-3 P2 (same shape here): the stall replan's grid_kwargs reads run()'s rear after the release."""
    w = World("return_home", mode="holding", plans=[plan()])
    w.tick(MISS)
    (grid,) = [c for c in w.calls if c[0] == "grid_kwargs"]
    assert grid[2]["own_pose"] == (3.0, 4.0, 0.1)
    np.testing.assert_array_equal(w.runner_rear, [3.0, 4.0, 0.1])


# M-a2 core sweep (tools/ma2_core_sweep.py --all): conditions the kept source tests read that no behaviour test held.
def test_an_off_path_heading_miss_is_neither_a_docking_stop_nor_a_goal_heading_miss():
    off = Tracking(status="failed", failure="endpoint_heading", off_path=True)
    w = World(docking={"status": "armed"})
    out = w.tick(off)
    assert out.tracking.status == "failed" and w.ms.dock_stop_ticks == 0
    w = World("return_home", mode="tracking")
    out = w.tick(off)
    assert out.tracking.status == "failed" and w.ms.slam_stall_ticks == 0


def test_a_goal_heading_miss_brakes_on_the_travel_legs_only():
    w = World("return_home", mode="tracking", stop_now=False)
    out = w.tick(MISS)
    assert (out.tracking.status, out.tracking.speed_mps) == ("braking", 0.0)  # brakes, waits for the stop
    w = World("observe", mode="tracking", stop_now=False)
    assert w.tick(MISS).tracking.status == "failed"  # an observe leg's miss is another rule's


def test_the_stall_replan_is_for_the_travel_legs_standing_on_a_zero_command():
    w = World("observe", mode="tracking")
    for _ in range(121):
        w.tick(Tracking(status="tracking", speed_mps=0.0))
    assert w.names() == [] and w.ms.slam_stall_ticks == 0
    w = World("transport", mode="tracking")
    for _ in range(121):
        w.tick(Tracking(status="tracking", speed_mps=0.1))  # standing, but still commanded to move: no stall
    assert w.names() == [] and w.ms.slam_stall_ticks == 0


def test_a_reapproach_stalls_at_a_cusp_only():
    w = World(docking={"reapproach": True}, mode="tracking", remaining=1.0)
    for _ in range(121):
        w.tick(Tracking(status="tracking", speed_mps=0.0))  # standing mid-leg, not at a cusp
    assert "start_docking_retry" not in w.names() and w.ms.slam_stall_ticks == 0


def test_a_plain_stall_without_a_stall_replan_left_does_not_replan():
    w = World(mode="tracking", stall_replans=2)
    for _ in range(121):
        w.tick(Tracking(status="tracking", speed_mps=0.0))
    assert "plan_transport_leg" not in w.names()


def test_a_stall_replan_keeps_another_legs_backoff():
    from forklift_core.control.path_tracking import TrackerConfig as TC

    backoff = {"phase": "return_home", "config": TC(cruise_speed_mps=0.15)}
    w = World(mode="tracking", plans=[plan()], obstacle={"backoff": dict(backoff)})
    w.tracker.config = TC(cruise_speed_mps=0.5)
    for _ in range(120):
        w.tick(Tracking(status="tracking", speed_mps=0.0))
    assert "plan_transport_leg" in w.names()
    assert w.obstacle["backoff"]["phase"] == "return_home"  # not this leg's: neither popped nor used
    assert w.core.trackers["transport"].config.cruise_speed_mps == 0.5


def test_a_real_time_docking_stop_that_asks_its_match_stands_braking_without_the_old_follower():
    """Codex D9 impl P2: the match asked from the docking stop -- that tick brakes, the old follower is not run
    again (it would fail the endpoint heading the stop is there for)."""
    w = World(docking={"status": "armed", "round": 2})
    w.core.jobs = SimpleNamespace()
    w.core.pending = None
    asked = []

    def ask(t):
        asked.append(t)
        w.core.pending = SimpleNamespace(site="dock")

    w.core.dock_at_delivery_straight = ask
    for _ in range(119):
        w.tick(MISS)
    out = w.tick(MISS)
    assert asked == [50.0] and "update" not in w.names() and "slam_rear" not in w.names()
    assert out.tracking.status == "braking" and out.tracking.speed_mps == 0.0
