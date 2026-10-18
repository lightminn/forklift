"""Real-time plan V8-1 (docs/plans/2026-10-17-realtime-sim.md, D8b-D8c): the mission core's resume boundaries for
the long plans of a real-time run -- the request, the ticks that stand for it, the plan taken on the current
state (or dropped and asked again), the failures, and the work clock that does not count the wait. A fake worker
lets each test decide when a plan is done."""

from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState


@dataclass(frozen=True)
class Config:
    clearance_m: float = 0.3


@dataclass
class Plan:
    status: str
    poses: np.ndarray = field(default_factory=lambda: np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
    directions: np.ndarray = field(default_factory=lambda: np.array([1, 1]))
    curvatures_inv_m: np.ndarray = field(default_factory=lambda: np.zeros(2))
    turnaround: object = None

    @property
    def success(self):
        return self.status == "success"


class FakeJobs:
    def __init__(self):
        self.submitted, self.answers, self.discarded = [], {}, []

    def submit(self, name, payload):
        self.submitted.append((name, payload))
        return len(self.submitted)

    def poll(self, job):
        return self.answers.pop(job, None)

    def discard(self, job):
        self.discarded.append(job)


def make(phase="transport", *, applied=(0.1, 0.0, 0.0)):
    calls = []

    def require(ok, reason):
        calls.append(("require", bool(ok), reason))
        if not ok:
            raise RuntimeError(reason)

    hooks = SimpleNamespace(
        require=require, wall=lambda: 100.0, record_json=str,
        grid_world=lambda sc: ("world", sc),
        grid_kwargs=lambda *a, **k: calls.append(("grid_kwargs", a, k)) or {"grid": a},
        path_record=lambda path: {"status": path.status}, write_paths=lambda: calls.append(("write_paths",)),
        slam_rear=lambda t: np.array([1.0, 2.0, 0.5]), stage=lambda: "STAGE", truth_rear=lambda: (1.0, 2.0, 0.5),
        add_path_display=lambda *a: calls.append(("display", a[2])),
    )
    refreshes = []
    layer = SimpleNamespace(refresh=lambda *a, **k: refreshes.append((a, k)), permission=SimpleNamespace(path_check=None))
    obstacle = {"layer": layer, "applied": applied, "version": 3}
    tracker = SimpleNamespace(config=TrackerConfig(), leg_ahead=lambda: ("LEG", 1))
    slam = {"tracker": SimpleNamespace(applied=(applied, 0), mode="tracking"), "stop_now": True, "holds": [],
            "transport_scenario": "SC_T"}
    ms = MissionState(phase=phase, phase_started=1.0)
    state = {"transitions": [], "paths": {}}
    options = SimpleNamespace(grid_planning=True, return_turnaround=False, planner_config=Config(),
                              travel_config=Config(), scenario=SimpleNamespace(destination="DEST"),
                              return_to_pose="HOME")
    core = MissionCore(ms, state=state, obstacle=obstacle, slam=slam, pocket={"check": None}, paths={},
                       trackers={phase: tracker} if phase in ("transport", "return_home") else {},
                       geometry="G", settings={}, pallet_geometry=None, hooks=hooks, options=options)
    core.arm_docking = lambda: calls.append(("arm_docking",))
    core.jobs = FakeJobs()
    core.commands.status = "running"
    return core, ms, state, obstacle, slam, calls, refreshes


REAR = np.array([1.0, 2.0, 0.5])


def test_the_first_live_leg_plan_is_asked_stands_and_is_taken_on_the_current_state():
    core, ms, state, obstacle, slam, calls, refreshes = make("transport")
    assert core.live_leg(20.0, REAR, True) is True  # asked: this tick stands
    name, payload = core.jobs.submitted[0]
    assert name == "leg" and payload["kind"] == "transport" and payload["retry_on"] == ("invalid_start", "no_path")
    assert payload["args"][0] == ("world", "SC_T") and payload["tight_args"][2] == Config(clearance_m=0.0)
    assert payload["kwargs"]["travel_config"] == Config() and payload["tight_kwargs"]["travel_config"] == Config(0.0)
    assert obstacle["deadline"] is None and core.job_pending()
    for t in (20.5, 30.0):  # waiting: stands, the work clock stands still
        assert core.job_hold(t, REAR, 0.0, False).hold
    assert core.commands.work_elapsed(1.0, 30.0) == pytest.approx(19.0)
    core.jobs.answers[1] = (True, {"result": Plan("success"), "tight": False})
    assert core.job_hold(37.6, REAR + [0.005, 0.0, 0.0], 0.0, False).hold  # taken: this tick stands too
    assert not core.job_pending() and core.paths["transport"].status == "success"
    assert ms.phase_started == 20.0  # as the asking tick would have set it ...
    assert core.commands.work_elapsed(ms.phase_started, 40.0) == pytest.approx(2.4)  # ... the wait not counted
    assert "arm_docking" in [c[0] for c in calls]
    # the permission's path check on the new leg, from where the truck stands now, before the first move
    assert np.array_equal(refreshes[-1][1]["path_ahead"][:, :2], Plan("success").poses[:, :2])
    assert refreshes[-1][1]["current_pose"] == (1.005, 2.0, 0.5)
    assert obstacle["live_plans"][-1]["async"] and obstacle["live_plans"][-1]["time_s"] == 20.0
    record = state["async_plans"][-1]  # what it was asked on and taken on
    assert record["asked"]["rear"] == [1.0, 2.0, 0.5] and record["taken"]["rear"] == [1.005, 2.0, 0.5]
    assert record["asked"]["truth_rear"] == [1.0, 2.0, 0.5] and record["asked"]["mode"] == "tracking"
    assert not core.job_hold(38.0, REAR, 0.0, False).hold


def test_a_leg_is_asked_only_once_the_truck_stands():
    """Isaac V8-4 d8c_6: asked while still rolling out of the last phase, the plan came back dropped as moving."""
    core, ms, state, obstacle, slam, calls, _ = make("transport")
    slam["stop_now"] = False
    for t in (20.0, 20.1):
        assert core.live_leg(t, REAR, True) is True  # stands, asks nothing, counts nothing
        assert not core.jobs.submitted and not core.job_pending()
        assert obstacle["live_replanned"]["transport"] is False and obstacle["deadline"] is None
    assert "async_plan_drops" not in state
    slam["stop_now"] = True
    assert core.live_leg(20.2, REAR, True) is True and core.job_pending()
    assert len(core.jobs.submitted) == 1


@pytest.mark.parametrize("change,why", [
    (lambda core, slam: slam["tracker"].__setattr__("applied", ((0.2, 0.0, 0.0), 0)), "correction"),
    (lambda core, slam: slam["tracker"].__setattr__("mode", "holding"), "slam_mode"),
    (lambda core, slam: core.ms.__setattr__("phase", "return_home"), "phase"),
])
def test_a_plan_is_dropped_when_the_state_it_was_made_for_changed(change, why):
    core, ms, state, obstacle, slam, calls, _ = make("transport")
    core.live_leg(20.0, REAR, True)
    change(core, slam)
    core.jobs.answers[1] = (True, {"result": Plan("success"), "tight": False})
    assert core.job_hold(25.0, REAR, 0.0, False).hold
    assert state["async_plans"][-1]["dropped"] == why and "transport" not in core.paths
    assert obstacle["live_replanned"]["transport"] is False  # live_leg asks again on its next call
    assert core.commands.waits[-1] == [20.0, 25.0]


def test_a_plan_waits_for_the_stop_detector_and_is_dropped_only_if_it_never_confirms():
    """Isaac d8e_p: the answer came 0.18 s after the ask, 0.3 s before the detector confirmed the stand."""
    core, ms, state, obstacle, slam, calls, _ = make("transport")
    core.live_leg(20.0, REAR, True)
    core.jobs.answers[1] = (True, {"result": Plan("success"), "tight": False})
    slam["stop_now"] = False
    for t in (20.2, 20.5, 22.0):  # kept, judged again each tick, the wait still open
        assert core.job_hold(t, REAR, 0.0, False).hold and core.job_pending()
    assert "async_plan_drops" not in state and core.commands.waits[-1][1] is None
    slam["stop_now"] = True
    assert core.job_hold(22.1, REAR, 0.0, False).hold and not core.job_pending()
    assert core.paths["transport"].status == "success" and state["async_plans"][-1]["dropped"] is None
    # never confirmed: dropped (and counted) once ASYNC_STAND_CONFIRM_S has passed since the answer
    core, ms, state, obstacle, slam, calls, _ = make("transport")
    core.live_leg(20.0, REAR, True)
    core.jobs.answers[1] = (True, {"result": Plan("success"), "tight": False})
    slam["stop_now"] = False
    assert core.job_hold(20.2, REAR, 0.0, False).hold and core.job_pending()
    assert core.job_hold(22.21, REAR, 0.0, False).hold and not core.job_pending()
    assert state["async_plans"][-1]["dropped"] == "moving" and state["async_plan_drops"] == {"live_leg": 1}


def test_drift_drops_the_plan_and_the_fourth_drop_ends_the_run():
    core, ms, state, obstacle, slam, calls, _ = make("transport")
    for k in range(3):
        core.live_leg(20.0 + k, REAR, True)
        core.jobs.answers[k + 1] = (True, {"result": Plan("success"), "tight": False})
        core.job_hold(20.5 + k, REAR + [0.03, 0.0, 0.0], 0.0, False)  # 3 cm off: dropped
        assert state["async_plans"][-1]["dropped"] == "drift"
    core.live_leg(30.0, REAR, True)
    core.jobs.answers[4] = (True, {"result": Plan("success"), "tight": False})
    with pytest.raises(RuntimeError, match="async_plan_discarded:live_leg:drift"):
        core.job_hold(30.5, REAR + [0.0, 0.0, 0.03], 0.0, False)  # 1.7 deg: dropped once more


def test_a_pause_drops_the_plan_without_counting_and_nothing_is_asked_while_paused():
    core, ms, state, obstacle, slam, calls, _ = make("transport")
    core.live_leg(20.0, REAR, True)
    assert not core.job_hold(21.0, REAR, 0.0, True).hold  # paused: the paused tick runs instead
    assert core.jobs.discarded == [1] and not core.job_pending() and "async_plan_drops" not in state
    assert obstacle["live_replanned"]["transport"] is False


def test_a_worker_failure_is_the_tick_plans_failure_and_a_failed_plan_retries_a_second_from_now():
    core, ms, state, obstacle, slam, calls, _ = make("transport")
    core.live_leg(20.0, REAR, True)
    core.jobs.answers[1] = (True, {"result": Plan("no_path"), "tight": True})
    core.job_hold(29.0, REAR, 0.0, False)
    assert obstacle["live_retry_after_s"] == 30.0 and obstacle["live_replanned"]["transport"] is False
    core.live_leg(30.0, REAR, True)
    core.jobs.answers[2] = (False, "ValueError: broken\ntraceback")
    with pytest.raises(RuntimeError, match="async_plan_failed:ValueError: broken"):
        core.job_hold(31.0, REAR, 0.0, False)


def test_a_truck_moving_while_it_waits_ends_the_run_after_a_second():
    core, *_ = make("transport")
    core.live_leg(20.0, REAR, True)
    core.job_hold(20.1, REAR, 0.08, False)
    core.job_hold(20.9, REAR, 0.08, False)
    with pytest.raises(RuntimeError, match="planning_wait_moving"):
        core.job_hold(21.2, REAR, 0.08, False)


def test_a_release_replan_is_asked_with_its_released_correction_and_asked_again_after_a_drop():
    core, ms, state, obstacle, slam, calls, refreshes = make("settle")
    core.trackers["return_home"] = SimpleNamespace(config=TrackerConfig(), leg_ahead=lambda: ("LEG", 1))
    core.paths["return_home"] = Plan("success")
    event = {"event": "release"}
    obstacle["applied"] = (0.0, 0.0, 0.0)  # the grid not yet re-projected: the plan's own correction decides
    core._release_request(20.0, {"leg": "return_home", "event": event, "after": (1.0, 2.0, 0.5),
                                 "return_scenario": "SC_R", "redo": False})
    name, payload = core.jobs.submitted[0]
    assert payload["kind"] == "return" and payload["args"][0] == ("world", "SC_R")
    assert core.pending.applied == (0.1, 0.0, 0.0)  # the applied (released) correction, not obstacle["applied"]
    core.jobs.answers[1] = (True, {"result": Plan("success"), "tight": True})
    slam["tracker"].applied = ((0.3, 0.0, 0.0), 0)  # a newer correction came in: dropped
    core.job_hold(21.0, REAR, 0.0, False)
    assert core.release_replan_needed is not None and not core.job_pending()
    assert core.job_hold(21.1, REAR, 0.0, False).hold  # asked again from where the truck stands
    assert core.pending.applied == (0.3, 0.0, 0.0) and len(core.jobs.submitted) == 2
    core.jobs.answers[2] = (True, {"result": Plan("success"), "tight": True})
    core.job_hold(22.0, REAR, 0.0, False)
    assert event["replanned"] and event["tight_retry"] and ("display", "Return") in calls


def refreshes_after(core):
    return core._refreshed


def test_the_observation_is_finished_with_its_capture_kept_apart_from_where_the_truck_stands():
    core, ms, state, obstacle, slam, calls, _ = make("observe")
    core._refreshed = []
    core._refresh_for_new_leg = lambda t, rear: core._refreshed.append((t, tuple(rear)))
    seen = {}
    core._observe_install = lambda plans, trace, start, attempt: seen.update(install=(plans, trace, attempt)) or "NEAR"

    def near(t, rear, base, yaw, observation, attempt_number, near_leg, acquired=None):
        seen.update(near=(t, tuple(rear), tuple(base), yaw, observation, near_leg, acquired))
        state["pocket_check"] = {"built_s": t}

    core.near_leg_or_recapture = near
    core._request("observe", "plan_transport", {"args": (), "kwargs": {}}, 40.0, REAR, (0.1, 0.0, 0.0),
                  ctx={"base": (2.0, 3.0, 0.1), "yaw": 0.4, "observation": "OBS", "attempt_number": 2,
                       "planning_start": 99.0, "acquired_s": 40.3})
    core.jobs.answers[1] = (True, {"result": "PLANS", "trace": ["t"]})
    now = REAR + [0.01, 0.0, 0.0]
    core.job_hold(57.6, now, 0.0, False)
    assert seen["install"] == ("PLANS", ["t"], 2)
    t, rear, base, yaw, observation, near_leg, acquired = seen["near"]
    assert t == 57.6 and rear == tuple(now) and base == (2.0, 3.0, 0.1) and yaw == 0.4 and observation == "OBS"
    assert near_leg == "NEAR" and acquired == (40.3, tuple(REAR))  # the capture's own time and pose
    assert core.fresh_depth_wait == (57.6, 40.3)  # a fresh pocket frame must follow the build
    assert refreshes_after(core)  # the new leg's path check before the first move


def test_without_a_fresh_pocket_frame_after_an_async_build_the_run_ends():
    core, *_ = make("approach")
    core.fresh_depth_wait = (50.0, 40.0)
    core.pocket["check"] = SimpleNamespace(last_new_s=40.0)  # only the capture's own frame
    assert not core.job_hold(51.0, REAR, 0.0, False).hold
    with pytest.raises(RuntimeError, match="async_near_no_fresh_depth"):
        core.job_hold(52.5, REAR, 0.0, False)
    core.fresh_depth_wait = (50.0, 40.0)
    core.pocket["check"] = SimpleNamespace(last_new_s=40.1)  # newer than the capture but 10 s old: not enough
    core.job_hold(50.6, REAR, 0.0, False)
    assert core.fresh_depth_wait is not None
    core.pocket["check"] = SimpleNamespace(last_new_s=50.5)  # a new pocket frame, fresh by the check's own rule
    core.job_hold(50.6, REAR, 0.0, False)
    assert core.fresh_depth_wait is None


def test_one_plan_at_a_time_a_pending_release_goes_before_the_legs_plan():
    """Codex real-time D8 implementation P1: the transition's tick asks no leg plan while a release is pending."""
    core, ms, state, obstacle, slam, calls, _ = make("transport")
    slam["pending_release"] = "transport"
    assert core.live_leg(20.0, REAR, True) is True and not core.jobs.submitted  # stands, asks nothing
    assert obstacle["live_replanned"]["transport"] is False  # asked once the release is done
    core._request("live_leg", "leg", {}, 20.0, REAR, (0.1, 0.0, 0.0), ctx={})
    core._request("release", "leg", {}, 20.1, REAR, (0.1, 0.0, 0.0), ctx={"need": {"leg": "transport"}})
    assert core.jobs.discarded == [1] and core.pending.site == "release"  # the first one superseded, not stacked
    assert core.commands.waits[-2][1] == 20.1 and core.commands.waits[-1] == [20.1, None]


def test_a_batch_core_has_no_worker_and_plans_in_the_tick():
    core, ms, state, obstacle, slam, calls, _ = make("transport")
    core.jobs = None
    core.hooks.plan_transport_leg = lambda *a, **k: Plan("success")
    core.hooks.arm_docking = lambda: None
    assert core.live_leg(20.0, REAR, True) is True
    assert not core.job_pending() and core.paths["transport"].status == "success" and ms.phase_started == 20.0
