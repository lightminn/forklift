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


# --- real-time plan D9c: the capture over control ticks and the observation's next-tick check ---------------------
@dataclass
class FakeCaptureDiagnostics:
    attempts: int = 2


class CaptureFailure(Exception):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason
        self.diagnostics = FakeCaptureDiagnostics()


class FakeSession:
    """Accepts a frame after ``steps`` render steps; ``fail_at`` raises at that step."""

    def __init__(self, steps=3, fail_at=None):
        self.steps, self.fail_at, self.n, self.closed, self.result = steps, fail_at, 0, False, None

    def start(self):
        return False

    def stepped(self, error=None):
        self.n += 1
        if self.fail_at == self.n:
            raise CaptureFailure("pose_changed")
        if self.n >= self.steps:
            self.result = ("SCENE", "FRAME_DIAG", self.n)
            return True
        return False

    def close(self):
        self.closed = True


def capture_core(session):
    core, ms, state, obstacle, slam, calls, _ = make("observe")
    state.update(observation_attempts=[], observation_waypoint_selected=[1.0, 2.0, 0.0])
    ms.next_candidate_index = 1
    core.options.adapter = SimpleNamespace(CaptureFailure=CaptureFailure)
    core.options.perception_max_attempts = 20
    core.hooks.perception_capture = lambda: SimpleNamespace(session=lambda max_attempts: session)
    core.hooks.live_abort = lambda: False
    core.hooks.now_s = lambda: 50.25
    return core, state, slam, calls


def test_a_capture_runs_one_render_step_per_tick_and_holds_its_frame_for_the_next_tick():
    session = FakeSession(steps=3)
    core, state, slam, calls = capture_core(session)
    assert core.capture_begin(50.0, REAR, 6000) is None and core.capture_live is not None
    assert state["observation_attempts"][0]["t_before_capture_s"] == 50.0
    for _ in range(2):
        assert core.capture_stepped(False) is None and core.capture_live.raw is None
    assert core.capture_stepped(False) is None and core.capture_live.raw == ("SCENE", "FRAME_DIAG", 3)


def test_a_stop_at_a_render_step_or_a_pause_drops_the_capture_and_a_failure_is_the_capture_s_failure():
    session = FakeSession(steps=5)
    core, state, slam, calls = capture_core(session)
    core.capture_begin(50.0, REAR, 6000)
    core.capture_stepped(False)
    assert core.capture_stepped(True).aborted and session.closed and core.capture_live is None
    assert state["observation_attempts"][-1]["aborted"] == "live_stop"
    session = FakeSession(steps=5)
    core, state, slam, calls = capture_core(session)
    core.capture_begin(50.0, REAR, 6000)
    core.capture_abandon()
    assert session.closed and core.capture_live is None and state["observation_attempts"][-1]["aborted"] == "paused"
    core, state, slam, calls = capture_core(FakeSession(steps=5, fail_at=2))
    core.capture_begin(50.0, REAR, 6000)
    core.capture_stepped(False)
    with pytest.raises(RuntimeError, match="perception_capture_failed:pose_changed"):
        core.capture_stepped(False)


@pytest.mark.parametrize("change,why", [
    (None, None),
    (lambda core, slam: slam["tracker"].__setattr__("applied", ((0.2, 0.0, 0.0), 0)), "correction"),
    (lambda core, slam: setattr(core.commands, "mission_gen", core.commands.mission_gen + 1), "mission"),
    (lambda core, slam: setattr(core.commands, "status", "cancelled"), "mission"),
    (lambda core, slam: core.ms.__setattr__("phase", "approach"), "phase"),
])
def test_an_observation_is_taken_on_the_next_tick_only_on_what_it_was_detected_on(change, why):
    core, state, slam, calls = capture_core(FakeSession())
    core.observation_freeze(50.3, "CAPTURED")
    if change is not None:
        change(core, slam)
    taken = core.observation_take(50.31)
    if why is None:
        assert taken == "CAPTURED" and core.capture_acquired_s == 50.25  # the time at detection, not now
    else:
        assert taken is None and state["observation_drops"][-1]["why"] == why


def test_a_fourth_dropped_observation_ends_the_run():
    core, state, slam, calls = capture_core(FakeSession())
    for k in range(3):
        core.observation_freeze(50.0 + k, "CAPTURED")
        slam["tracker"].applied = ((0.2 + k, 0.0, 0.0), 0)
        assert core.observation_take(50.5 + k) is None
    core.observation_freeze(60.0, "CAPTURED")
    slam["tracker"].applied = ((9.0, 0.0, 0.0), 0)
    with pytest.raises(RuntimeError, match="observation_discarded:correction"):
        core.observation_take(60.5)


# --- real-time plan D9a: SLAM replies applied at a tick's start ----------------------------------------------------
def test_replies_are_applied_in_order_and_a_new_correction_re_projects_the_grid_in_that_tick():
    from forklift_core.localization.slam_link import Reply
    from forklift_core.localization.slam_pose import SlamPoseTracker

    core, ms, state, obstacle, slam, calls, refreshes = make("transport", applied=(0.0, 0.0, 0.0))
    tracker = SlamPoseTracker(max_age_s=1.25, hold_limit_m=15.0)
    tracker.receive(4, 0.4, "processed", (0.0, 0.0, 0.0))  # tracking, as after the warm-up
    slam.update(tracker=tracker, records=[])
    obstacle["applied"] = (0.0, 0.0, 0.0)
    core.hooks.slam_rear = lambda t: np.array([1.0, 2.0, 0.5])
    meta = lambda i: {"scan_id": i, "stamp_s": 0.1 * i, "phase": "transport", "scan_correction": [0.0, 0.0, 0.0]}
    core.slam_replies(5.0, [(Reply(5, 0.5, "skipped", (0.0, 0.0, 0.0), 1, 0.0), meta(5), 1.0, 1.01)], True)
    assert refreshes == [] and slam["records"][-1]["link"] == "async"  # a skipped reply changes nothing
    core.slam_replies(5.1, [(Reply(6, 0.6, "skipped", (0.0, 0.0, 0.0), 1, 0.0), meta(6), 1.1, 1.11),
                            (Reply(7, 0.7, "processed", (0.03, 0.0, 0.0), 2, 0.2), meta(7), 1.11, 1.35)], True)
    rec = slam["records"][-1]
    assert [r["scan_id"] for r in slam["records"]] == [5, 6, 7]
    assert rec["applied_after"] == [0.03, 0.0, 0.0] and rec["scan_correction"] == [0.0, 0.0, 0.0]
    assert (rec["sent_mono"], rec["received_mono"], rec["applied_tick_s"]) == (1.11, 1.35, 5.1)
    assert obstacle["applied"] == (0.03, 0.0, 0.0) and obstacle["version"] == 4 and len(refreshes) == 1
    assert refreshes[0][1]["current_pose"] == (1.0, 2.0, 0.5)  # the pose with the new correction
    tracker.hold(odom_from_base=(0.0, 0.0, 0.0))  # held: a new correction is received, not applied
    core.slam_replies(5.2, [(Reply(8, 0.8, "processed", (0.05, 0.0, 0.0), 3, 0.2), meta(8), 1.2, 1.4)], True)
    assert len(refreshes) == 1 and slam["records"][-1]["received_after"] == [0.05, 0.0, 0.0]
    assert slam["records"][-1]["applied_after"] == [0.03, 0.0, 0.0]


# --- the persistent console's mission end (docs/plans/2026-10-18-persistent-live-console.md D1 step 2) -------------
@pytest.mark.parametrize("phase", ["transport", "lower"])
def test_abandon_jobs_drops_the_waited_plan_without_the_drops_side_effects(phase):
    core, ms, state, obstacle, slam, calls, _ = make(phase)
    if phase == "transport":
        assert core.live_leg(20.0, REAR, True) is True  # a live leg plan is waited for
    else:
        core._request("release", "leg", {}, 20.0, REAR, None, {"need": {"leg": "x"}})
    job = core.pending.job
    closed = []
    core.capture_live = SimpleNamespace(session=SimpleNamespace(close=lambda: closed.append(True)), attempt={})
    core.observation_live, core.capture_acquired_s, core.fresh_depth_wait = "OBS", 19.0, (18.0, 17.5)
    core.release_replan_needed = {"leg": "older"}
    core.abandon_jobs(22.0)
    assert core.pending is None and core.jobs.discarded == [job]
    assert core.commands.waits[-1] == [20.0, 22.0]
    assert state["async_plans"][-1]["finished_s"] == 22.0 and state["async_plans"][-1]["dropped"] == "abandoned"
    assert core.release_replan_needed is None and "async_plan_drops" not in state
    assert obstacle.get("live_replanned", {}).get(phase) is not False  # _drop's live-leg undo is not made
    assert closed == [True] and core.capture_live is None
    assert core.observation_live is None and core.capture_acquired_s is None and core.fresh_depth_wait is None
    core.abandon_jobs(23.0)  # nothing left: nothing happens
    assert core.jobs.discarded == [job] and len(state["async_plans"]) == 1


# --- persistent console plan D2: a live mission prepared from where the truck stands ------------------------------
def preparing_core():
    from forklift_core.mission.baseline import MissionBaseline
    from forklift_core.mission.commands import Submit
    from forklift_core.mission.targets import Pose

    core, ms, state, obstacle, slam, calls, _ = make("complete")
    core.commands.status = "idle"
    core.targets.add_pallet("P-1", "epal6", Pose(3.1, 0.65, 0.0))
    core.targets.add_destination("D-1", Pose(-5.0, -21.4, -1.57))
    core.options.observation_waypoints = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
    core.options.pickup_bounds, core.options.home_rear = "BAY", "HOME"
    core.options.speeds, core.options.cusp_heading_hold_rad, core.options.tracker_profile = {"observe": 0.3}, 0.05, None
    core.settings.update(tracker_curvature_inv_m=0.4, drive_acceleration_mps2=0.3)
    core.hooks.lift_position = lambda: 0.004
    started = []
    core.hooks.live_mission_start = lambda t, inputs, plan, records: started.append((t, inputs, plan, records))
    core.hooks.live_mission_ready = lambda t, inputs: None
    state["observation_attempts"] = []
    core.baseline = MissionBaseline.capture(core)
    core.commands.queue(Submit("P-1", "D-1", core.targets.revision))
    core.commands.apply(10.0)
    inputs = {"scenario": "SC_NEW", "wait_pose": "WAIT", "waypoints": [[5.0, 5.0, 1.0], [6.0, 5.0, 1.0]],
              "pickup_bounds": None, "pickup_zone": "ZONE"}
    return core, ms, state, started, inputs


def ladder(result=None, selected=None, timed_out=False, statuses=("no_path",)):
    return {"result": result, "selected": selected, "timed_out": timed_out, "wall_s": 0.1,
            "candidates": [{"candidate_index": i, "pose": [5.0 + i, 5.0, 1.0], "status": st, "success": st == "success"}
                           for i, st in enumerate(statuses)]}


def test_a_prepared_mission_is_adopted_with_its_start_processing():
    core, ms, state, started, inputs = preparing_core()
    ms.observe_stall_ticks, core.last_permission = 9, (0.2, "slow")
    core.prepare_request(11.0, REAR, inputs, {"waypoints": inputs["waypoints"]})
    assert core.commands.status == "preparing" and core.pending.site == "prepare"
    assert core.jobs.submitted[-1][0] == "observe_ladder" and core.prepare_attempts == 1
    assert core.job_hold(11.5, REAR, 0.0, False).hold and core.commands.status == "preparing"  # still planning
    plan = Plan("success")
    core.jobs.answers[core.pending.job] = (True, ladder(plan, 1, statuses=("no_path", "success")))
    assert core.job_hold(12.0, REAR, 0.0, False).hold
    assert core.commands.status == "running" and core.commands.adopted_id == core.commands.mission_id
    assert ms.phase == "observe" and ms.phase_started == 12.0 and ms.next_candidate_index == 2
    assert ms.observe_stall_ticks == 0 and core.last_permission is None and ms.lift_command == 0.004
    assert core.paths == {"observe": plan} and state["planning_status"] == "success"
    assert list(core.trackers) == ["observe"] and core.trackers["observe"].config.cruise_speed_mps == 0.3
    assert state["observation_waypoint_selected"] == [6.0, 5.0, 1.0]
    assert [c["status"] for c in state["observation_candidates"]] == ["no_path", "success"]
    o = core.options
    assert (o.scenario, o.return_to_pose, o.home_rear, o.pickup_bounds, o.observe_bounds, o.pickup_zone) == (
        "SC_NEW", "WAIT", "WAIT", None, None, "ZONE")
    assert o.observation_waypoints == inputs["waypoints"]
    assert started and started[0][0] == 12.0 and started[0][2] is plan
    assert state["transitions"][-1] == {"from": "complete", "to": "observe", "time_s": 12.0,
                                        "mission_id": core.commands.mission_id}


@pytest.mark.parametrize("given", [{"adoption_zone": "THIS_MISSIONS"}, {}])
def test_the_adoption_takes_this_missions_rectangle_or_none(given):
    """Plan 2026-10-28-adoption-zone-check section 2: fixed at preparation, never the last mission's."""
    core, ms, state, started, inputs = preparing_core()
    core.options.adoption_zone = "LAST_MISSIONS"
    core.prepare_request(11.0, REAR, {**inputs, **given}, {"waypoints": inputs["waypoints"]})
    core.jobs.answers[core.pending.job] = (True, ladder(Plan("success"), 0, statuses=("success",)))
    core.job_hold(12.0, REAR, 0.0, False)
    assert ms.phase == "observe" and core.options.adoption_zone == given.get("adoption_zone")


@pytest.mark.parametrize("answer, reason", [
    ((True, ladder(timed_out=True)), "prepare:timeout"),
    ((True, ladder(statuses=("no_path", "invalid_start"))), "prepare:all_candidates_failed:no_path;invalid_start"),
    ((False, "Traceback ...\nValueError: x"), "prepare:error:Traceback ..."),
])
def test_a_preparation_that_plans_nothing_fails_the_mission_and_changes_nothing(answer, reason):
    core, ms, state, started, inputs = preparing_core()
    core.prepare_request(11.0, REAR, inputs, {})
    core.jobs.answers[core.pending.job] = answer
    core.job_hold(12.0, REAR, 0.0, False)
    assert core.commands.status == "failed" and core.commands.end_reason == reason
    assert ms.phase == "complete" and not started and core.options.scenario.destination == "DEST"


def test_a_blocked_start_carries_the_workers_advice_to_the_missions_end():
    """Persistent console plan D13 item 6: the worker's advice reaches the result, nothing else gets one."""
    core, ms, state, started, inputs = preparing_core()
    core.prepare_request(11.0, REAR, inputs, {})
    answer = ladder(statuses=("invalid_start", "invalid_start"))
    answer["advice"] = {"backoff_m": 0.1, "backoff_max_m": 0.4, "from_rear": [1.0, 2.0, 0.0]}
    core.jobs.answers[core.pending.job] = (True, answer)
    core.job_hold(12.0, REAR, 0.0, False)
    assert core.commands.status == "failed" and core.commands.end_advice == answer["advice"]
    core2, *_ = preparing_core()
    core2.prepare_request(11.0, REAR, inputs, {})
    core2.jobs.answers[core2.pending.job] = (True, ladder(timed_out=True))
    core2.job_hold(12.0, REAR, 0.0, False)
    assert core2.commands.end_advice is None


@pytest.mark.parametrize("change, why", [("drift", "drift"), ("targets", "targets")])
def test_a_preparation_whose_inputs_changed_is_asked_again_once_then_fails(change, why):
    from forklift_core.mission.targets import Pose

    core, ms, state, started, inputs = preparing_core()
    for attempt in (1, 2):
        core.prepare_request(11.0 + attempt, REAR, inputs, {})
        core.jobs.answers[core.pending.job] = (True, ladder(Plan("success"), 0, statuses=("success",)))
        if change == "drift":
            rear = REAR + [0.05, 0.0, 0.0]
        else:
            rear = REAR
            core.targets.add_destination(f"D-{attempt + 5}", Pose(0.0, 0.0, 0.0))
        assert core.job_hold(11.5 + attempt, rear, 0.0, False).hold
        assert core.pending is None and "async_plan_drops" not in state
        assert core.commands.status == ("preparing" if attempt == 1 else "failed")
    assert core.commands.end_reason == f"prepare:{why}" and not started


def test_the_destinations_reference_is_made_before_anything_changes_and_its_failure_fails_only_the_preparation():
    """Codex stage 2 impl P2-6: the cast ran after the start processing had changed the options, paths and trackers."""
    core, ms, state, started, inputs = preparing_core()
    order = []
    core.hooks.live_mission_ready = lambda t, inputs: order.append(("ready", core.options.scenario)) or "REF"
    core.hooks.live_mission_start = lambda t, inputs, plan, ready: order.append(("start", ready))
    core.prepare_request(11.0, REAR, inputs, {})
    core.jobs.answers[core.pending.job] = (True, ladder(Plan("success"), 0, statuses=("success",)))
    core.job_hold(12.0, REAR, 0.0, False)
    assert order[0][0] == "ready" and order[0][1].destination == "DEST"  # before the options changed
    assert order[1] == ("start", "REF") and core.commands.status == "running"

    core, ms, state, started, inputs = preparing_core()

    def broken(t, inputs):
        raise RuntimeError("cast failed")

    core.hooks.live_mission_ready = broken
    core.prepare_request(11.0, REAR, inputs, {})
    core.jobs.answers[core.pending.job] = (True, ladder(Plan("success"), 0, statuses=("success",)))
    core.job_hold(12.0, REAR, 0.0, False)
    assert core.commands.status == "failed" and core.commands.end_reason == "prepare:reference:cast failed"
    assert ms.phase == "complete" and core.options.scenario.destination == "DEST" and not started


def test_the_adoption_keeps_this_missions_own_preparation_records():
    """Codex stage 2 impl P2-5: the baseline is restored at the mission's end (D1 step 8); at the adoption what was
    recorded since -- this mission's preparation -- stays with it."""
    core, ms, state, started, inputs = preparing_core()
    core.hooks.live_mission_ready = lambda t, inputs: None
    core.prepare_request(11.0, REAR, inputs, {})
    core.jobs.answers[core.pending.job] = (True, ladder(Plan("success"), 0, statuses=("success",)))
    core.job_hold(12.0, REAR, 0.0, False)
    assert [r["site"] for r in state["async_plans"]] == ["prepare"]


def test_the_end_closes_the_pocket_check_and_hands_the_mission_records_back():
    """Codex stage 2 impl P1-1: a cancel in the approach left the pocket check, its exemption and support."""
    core, ms, state, started, inputs = preparing_core()
    layer = core.obstacle["layer"]
    layer.exempt, layer.depth_support = "EXEMPT", "SUPPORT"
    core.pocket["check"] = SimpleNamespace(summary=lambda: {"frames": 3})
    core.pocket["frames_read"] = 3
    state["async_plans"] = [{"site": "leg"}]
    records = core.end_reset(20.0)
    assert core.pocket["check"] is None and layer.exempt is None and layer.depth_support is None
    assert records["state"]["pocket_check"] == {"frames": 3, "frames_read": 3} and "pocket_check" not in state
    assert records["state"]["async_plans"] == [{"site": "leg"}] and "async_plans" not in state


@pytest.mark.parametrize("site", ["leg", "prepare"])
def test_a_dead_worker_ends_the_session_not_the_mission(site):
    """Codex D6 P2-4: a worker that could not be started again fails every job at once; that is not 'no path'."""
    if site == "prepare":
        core, ms, state, started, inputs = preparing_core()
        core.prepare_request(11.0, REAR, inputs, {})
    else:
        core, ms, state, obstacle, slam, calls, _ = make("transport")
        core.live_leg(20.0, REAR, True)
    core.jobs._dead = "plan worker restart failed: boom"
    core.jobs.answers[core.pending.job] = (False, core.jobs._dead)
    with pytest.raises(RuntimeError, match="^plan_worker_dead: plan worker restart failed: boom"):
        core.job_hold(30.0, REAR, 0.0, False)


# --- the end SLAM release (plan D1 step 4; stage 3 design, 4th edition) --------------------------------------------
class HeldTracker:
    """A SLAM tracker holding a correction: a release moves the estimate by +0.20 m in x."""

    def __init__(self):
        self.mode, self.applied, self.released = "holding", ((0.0, 0.0, 0.0), 0), 0

    def release(self, odom_from_base=None):
        self.mode, self.applied, self.released = "tracking", ((0.2, 0.0, 0.0), 0), self.released + 1
        return 0.2, 0.0

    def hold(self, odom_from_base=None):
        self.mode = "holding"

    def may_drive(self):
        return True


def held_core(phase="approach"):
    from forklift_core.mission.targets import Pose

    core, ms, state, started, inputs = preparing_core()
    ms.phase = phase
    slam = core.slam
    slam["tracker"] = HeldTracker()
    slam.update(stop_now=True, pending_release="transport", release_wait_from=5.0, control=[[0.0, 1.0, 2.0, 0.5]])
    pose = {"rear": np.array([1.0, 2.0, 0.5])}
    core.hooks.slam_rear = lambda t: pose["rear"] + ([0.2, 0.0, 0.0] if slam["tracker"].mode == "tracking" else 0.0)
    core.hooks.slam_odom_base = lambda: None
    core.hooks.set_rear = lambda rear: None
    core.commands.adopted_id = core.commands.mission_id  # the mission ran
    from forklift_core.planning.pallet_mission import PalletSite

    core.pocket["planning_pickup"] = PalletSite(3.0, 0.5, 0.0)  # seen in the hold
    core.obstacle["pickup_estimate"] = PalletSite(3.0, 0.5, 0.0)
    slam["transport_scenario"] = Scenario(PalletSite(-5.0, -21.4, -1.57))
    return core, slam, Pose


@dataclass(frozen=True)
class Scenario:
    destination: object


def test_an_end_standing_releases_the_hold_and_carries_only_the_held_pallet_estimate():
    core, slam, Pose = held_core("approach")
    dest = core.targets.destinations["D-1"]
    version = core.targets.correction_version
    core.end_reset(20.0)
    assert slam["tracker"].released == 1 and slam["pending_release"] is None and slam["release_wait_from"] is None
    pose = core.targets.pallets["P-1"].pose  # carried by the release, then settled
    assert (pose.x_m, pose.y_m, pose.yaw_rad) == pytest.approx((3.2, 0.5, 0.0))
    assert core.targets.destinations["D-1"] == dest  # a map goal: not moved
    assert core.targets.correction_version == version + 1
    assert slam["holds"][-1]["reason"] == "mission_end" and core.obstacle["applied"] == (0.2, 0.0, 0.0)
    assert core.pending is None and not core.jobs.submitted  # no plan, no path (U28)


def test_an_end_while_moving_releases_once_standing_and_carries_the_pallet_target():
    core, slam, Pose = held_core("observe")
    slam["stop_now"] = False
    core.end_reset(20.0)
    assert slam["tracker"].released == 0 and slam["pending_release"] is None
    before = core.targets.pallets["P-1"].pose
    core.release_gate(21.0, np.array([1.0, 2.0, 0.5]))  # still moving: waits, no 10 s limit
    assert slam["tracker"].released == 0
    slam["stop_now"] = True
    core.release_gate(22.0, np.array([1.0, 2.0, 0.5]))
    assert slam["tracker"].released == 1
    after = core.targets.pallets["P-1"].pose
    assert (after.x_m - before.x_m) == pytest.approx(0.2) and core.targets.destinations["D-1"].x_m == -5.0
    core.release_gate(23.0, np.array([1.0, 2.0, 0.5]))
    assert slam["tracker"].released == 1  # once


def cargo_to(core, phase):
    """The session's fork relation as a mission that reached ``phase`` left it (plan D5a: driven by transitions)."""
    order = ["observe", "approach", "insert", "lift", "extract", "transport", "lower", "withdraw", "settle"]
    for a, b in zip(order, order[1:order.index(phase) + 1]):
        core.cargo.on_transition(a, b, "P-1")


@pytest.mark.parametrize("phase", ["insert", "lift", "lower", "withdraw"])
def test_an_end_with_the_forks_in_a_pocket_keeps_the_hold(phase):
    core, slam, Pose = held_core(phase)
    cargo_to(core, phase)
    core.end_reset(20.0)
    assert slam["tracker"].mode == "holding" and slam["tracker"].released == 0
    assert slam["pending_release"] is None  # the mission's own reservation is gone either way


def test_an_end_while_carrying_releases_the_hold_for_the_teleop_with_the_load():
    """Plan D5a (Codex round 1 P2-6): a hold left while carrying freezes the correction under the loaded teleop the
    user allowed, until LocalizationStale at 15 m."""
    core, slam, Pose = held_core("transport")
    cargo_to(core, "transport")
    core.end_reset(20.0)
    assert slam["tracker"].released == 1


@pytest.mark.parametrize("standing", [True, False])
def test_an_end_after_docking_carries_the_docked_destination_and_the_pallet_delivered_there(standing):
    """Codex D6 design round 4 P2: ended in settle with the docking hold still on; both the immediate and the deferred
    release move the docked destination and its delivered pallet together, by the frame change, once."""
    from forklift_core.mission.targets import Pose
    from forklift_core.planning.pallet_mission import PalletSite

    core, slam, _ = held_core("settle")
    core.pocket["planning_pickup"] = core.obstacle["pickup_estimate"] = None
    slam["transport_scenario"] = Scenario(PalletSite(5.1, 1.0, 0.0))  # the prior
    slam["tracker"].mode = "tracking"
    core.hold_slam(10.0, held_frame_pallet=False)  # the transport's arrival hold
    slam["tracker"].mode = "holding"
    slam["docking"] = {"accepted_any": True}
    slam["transport_scenario"] = Scenario(PalletSite(5.0, 1.0, 0.0))  # the docking match, in the held frame
    core.targets.carried("P-1")
    core.targets.put_down("P-1", Pose(5.0, 1.0, 0.0))
    slam["stop_now"] = standing
    core.end_reset(20.0)
    if not standing:
        slam["stop_now"] = True
        core.release_gate(21.0, np.array([1.0, 2.0, 0.5]))
    assert slam["tracker"].released == 1
    pallet, dest = core.targets.pallets["P-1"].pose, core.targets.destinations["D-1"]
    assert (pallet.x_m, pallet.y_m) == pytest.approx((5.2, 1.0)) and (dest.x_m, dest.y_m) == pytest.approx((5.2, 1.0))


@pytest.mark.parametrize("phase, adopted, key", [
    ("withdraw", False, "recovery_required"), ("insert", False, "recovery_required"),
    ("transport", False, "cargo_held"), ("settle", False, None), ("observe", False, None),
    ("transport", True, None),  # a mission running (a batch run's is always adopted)
])
def test_the_safety_state_says_what_an_ended_missions_forks_refuse(phase, adopted, key):
    """Plan D5a: from the session's fork relation, not the phase name (which a batch run's state still uses)."""
    core, ms, state, obstacle, slam, calls, _ = make(phase)
    cargo_to(core, phase)
    core.commands.adopted_id = "m1" if adopted else None
    core.commands.mission_id = "m1"
    assert not {"recovery_required", "cargo_held"} & set(core.safety_state(10.0))  # not live: as before (P3)
    core.baseline = object()  # a live session
    s = core.safety_state(10.0)
    for k in ("recovery_required", "cargo_held"):
        assert (k in s) is (k == key) and s.get(k, True) is True


def test_a_live_safety_state_publishes_the_fork_relation_and_a_batch_one_the_phase():
    core, ms, state, obstacle, slam, calls, _ = make("transport")
    cargo_to(core, "transport")
    batch = core.safety_state(10.0)
    assert batch["load"] == {"loaded": True, "basis": "phase", "lift_target_m": 0.0}
    assert batch["fork_in_pocket"] == {"value": False, "basis": "phase"}
    core.baseline = object()  # a live session (the runner captures the baseline only there)
    live = core.safety_state(10.0)
    assert live["load"] == {"loaded": True, "basis": "phase+cargo", "pallet_id": "P-1", "state": "confirmed",
                            "lift_target_m": 0.0}
    assert live["fork_in_pocket"] == {"value": True, "basis": "phase+cargo", "relation": "carrying"}


def test_the_cores_transitions_drive_the_session_cargo():
    core, ms, state, obstacle, slam, calls, _ = make("lift")
    core.commands.mission = SimpleNamespace(pallet_id="P-7", destination_id="D-1")
    core.hooks.echo, core.hooks.snapshot = (lambda *a: None), (lambda *a: None)
    obstacle["layer"].known_enabled, obstacle["layer"].memory = False, None
    core.transition("extract", 5.0)
    assert (core.cargo.relation, core.cargo.load, core.cargo.pallet_id) == ("carrying", "confirmed", "P-7")


def test_an_arrival_hold_carries_only_what_it_found_itself():
    """Codex D6 round 5: a hold at the return's arrival must not move the destination the settle release already
    carried; a hold at the observation's arrival, before any capture, must not move the pallet's map prior."""
    from forklift_core.mission.targets import Pose
    from forklift_core.planning.pallet_mission import PalletSite

    # the return's arrival hold, after the delivery: the destination is a map goal already
    core, slam, _ = held_core("home_settle")
    core.pocket["planning_pickup"] = core.obstacle["pickup_estimate"] = None
    slam["docking"] = {"accepted_any": True}
    slam["return_scenario"] = Scenario(PalletSite(5.0, 1.0, 0.0))
    core.targets.carried("P-1")
    core.targets.put_down("P-1", Pose(5.0, 1.0, 0.0))
    slam["tracker"].mode = "tracking"
    core.hold_slam(30.0, held_frame_pallet=False)  # the arrival hold (follow's end of leg)
    slam["tracker"].mode = "holding"
    core.end_reset(40.0)
    assert core.targets.destinations["D-1"].x_m == pytest.approx(5.0)
    assert core.targets.pallets["P-1"].pose.x_m == pytest.approx(5.0)

    # the observation's arrival hold, before the capture; ended while moving, released once standing
    core, slam, _ = held_core("observe")
    core.pocket["planning_pickup"] = core.obstacle["pickup_estimate"] = None
    slam["tracker"].mode = "tracking"
    core.hold_slam(10.0, held_frame_pallet=False)
    slam["tracker"].mode = "holding"
    slam["stop_now"] = False
    before = core.targets.pallets["P-1"].pose
    core.end_reset(12.0)
    slam["stop_now"] = True
    core.release_gate(13.0, np.array([1.0, 2.0, 0.5]))
    assert slam["tracker"].released == 1 and core.targets.pallets["P-1"].pose == before


def test_a_capture_made_under_the_held_correction_is_carried():
    """The approach hold begins at the accepted capture, whose estimate was made with the correction it holds."""
    core, slam, _ = held_core("approach")
    slam["tracker"].mode = "tracking"
    core.hold_slam(10.0, held_frame_pallet=True)
    slam["tracker"].mode = "holding"
    core.end_reset(12.0)
    assert core.targets.pallets["P-1"].pose.x_m == pytest.approx(3.2)


@pytest.mark.parametrize("observed, snapshot_x", [(False, 3.2), (True, 3.0)])
def test_a_deferred_release_carries_a_put_down_made_in_its_hold_but_not_an_older_one(observed, snapshot_x):
    """Plan 2026-10-28-adoption-zone-check (Codex implementation round 3 P2): a manual recovery's ground verdict put
    the pallet down in this hold -- its snapshot is that hold's estimate and goes with the pose; a pose an observation
    rewrote since is a new estimate, and the snapshot stays where it was put down."""
    core, slam, Pose = held_core("observe")
    core.targets.carried("P-1")
    core.targets.put_down("P-1", Pose(3.0, 0.5, 0.0), state="available")
    if observed:
        core.targets.update_pose("P-1", Pose(3.05, 0.5, 0.0), basis="recognised", uncertainty_m=0.05)
    core.end_release_due = {"pallet_id": "P-1", "destination_id": "D-1", "pallet_held": True,
                            "destination_held": False}
    core.end_release(20.0, deferred=True)
    pallet = core.targets.pallets["P-1"]
    assert pallet.pose.x_m == pytest.approx(3.25 if observed else 3.2)
    assert pallet.put_down_pose.x_m == pytest.approx(snapshot_x)


def test_a_deferred_end_release_is_dropped_when_the_cargo_turns_unknown_before_standing():
    """Codex D5a impl round 1 P2: the release was reserved while carrying; the pallet dropped before the truck stood
    -- unknown keeps the hold."""
    core, slam, Pose = held_core("transport")
    cargo_to(core, "transport")
    slam["stop_now"] = False
    core.end_reset(20.0)
    assert core.end_release_due is not None and slam["tracker"].released == 0
    core.cargo.invalidate("Pallet dropped during transport")
    slam["stop_now"] = True
    core.release_gate(21.0, np.array([1.0, 2.0, 0.5]))
    assert slam["tracker"].released == 0 and slam["tracker"].mode == "holding" and core.end_release_due is None


@pytest.mark.parametrize("phase", ["recover_withdraw", "clear_out"])
def test_an_end_release_after_a_recovery_carries_the_put_back_pallet(phase):
    """Plan D4b item 4 (Codex 6b-2 round 1 P2-1): the recovery put the pallet back at the hold's estimate; the
    targets list no longer follows the core there, so the end release carries the list's pallet itself -- the
    truck and the pallet keep their gap across the frame change."""
    core, slam, Pose = held_core(phase)
    slam["tracker"].mode = "tracking"
    core.hold_slam(10.0, held_frame_pallet=True)  # the approach hold: the capture's estimate is in its frame
    slam["tracker"].mode = "holding"
    core.targets.carried("P-1")
    core.restore_pallet_target()  # the ground verdict
    assert core.targets.pallets["P-1"].pose.x_m == pytest.approx(3.0)
    core.end_reset(20.0)
    assert slam["tracker"].released == 1
    assert core.targets.pallets["P-1"].pose.x_m == pytest.approx(3.2)


def test_an_end_release_after_a_new_hold_keeps_the_put_back_pallet():
    """A hold the back-out began itself found nothing: the pallet's estimate is a map estimate and stays."""
    core, slam, Pose = held_core("clear_out")
    slam["tracker"].mode = "tracking"
    core.hold_slam(10.0, held_frame_pallet=False)
    slam["tracker"].mode = "holding"
    core.targets.carried("P-1")
    core.restore_pallet_target()
    core.end_reset(20.0)
    assert core.targets.pallets["P-1"].pose.x_m == pytest.approx(3.0)


# --- the live far recognition's stop decision (docs/plans/2026-10-22-live-far-recognition.md T1 (A), tasks 4, 10) ---
def far_core():
    """A capture core whose far stop is handed over: ``answers`` is what the poll gives, in turn."""
    core, state, slam, calls = capture_core(FakeSession())
    answers, cancels, ends = [], [], []

    def poll(handle):
        got = answers.pop(0)
        if isinstance(got, Exception):
            raise got
        return got

    core.hooks.far_recognition_poll = poll
    core.hooks.far_recognition_cancel = cancels.append
    core.hooks.far_recognition_mission_end = ends.append
    return core, state, slam, answers, cancels, ends


def far_captured():
    return SimpleNamespace(far_pending="HANDLE", attempt={"observation_source": "pending"}, observation="DETECTOR")


@dataclass
class Obs:
    status: str
    reason: str | None = None


def test_a_far_stop_is_waited_for_with_the_work_clocks_standing_and_taken_with_its_decision():
    core, state, slam, answers, cancels, ends = far_core()
    captured = far_captured()
    core.observation_freeze(50.3, captured)
    assert core.commands.waits[-1] == [50.3, None]  # the wait opens with the freeze
    answers.append(SimpleNamespace(state="waiting"))
    assert core.observation_take(50.4) is None and core.observation_live is not None  # still frozen
    answers.append(SimpleNamespace(state="done", observation=Obs("valid"), record={"source": "fused", "leg": 0}))
    taken = core.observation_take(50.5)
    assert taken is captured and taken.observation == Obs("valid")
    assert taken.attempt["observation_source"] == "fused" and taken.attempt["recognition"]["leg"] == 0
    assert taken.attempt["pocket_observation"] == {"status": "valid", "reason": None}
    assert core.commands.waits[-1] == [50.3, 50.5] and core.observation_live is None
    assert core.capture_acquired_s == 50.25  # the frozen acquisition, not the decision's time


def test_a_far_decision_is_dropped_when_the_correction_changed_while_it_was_worked_on():
    core, state, slam, answers, cancels, ends = far_core()
    core.observation_freeze(50.3, far_captured())
    slam["tracker"].applied = ((0.4, 0.0, 0.0), 0)  # a correction while the worker decides
    answers.append(SimpleNamespace(state="done", observation=Obs("valid"), record={"source": "fused"}))
    assert core.observation_take(50.5) is None
    assert state["observation_drops"][-1]["why"] == "correction" and core.commands.waits[-1][1] == 50.5
    assert state["recognition_discarded"][-1]["why"] == "correction"  # counted as a far drop too


def test_a_leg_ended_before_the_decision_captures_again_once_then_fails_the_observation():
    core, state, slam, answers, cancels, ends = far_core()
    core.observation_freeze(50.3, far_captured())
    answers.append(SimpleNamespace(state="invalid", why="leg", observation=Obs("no_pallet", "far_leg")))
    assert core.observation_take(50.5) is None and core.observation_live is None  # the arrival captures again
    assert state["recognition_discarded"][-1]["why"] == "leg" and core.far_leg_recaptures == 1
    core.observation_freeze(51.0, far_captured())
    answers.append(SimpleNamespace(state="invalid", why="leg", observation=Obs("no_pallet", "far_leg")))
    failed = core.observation_take(51.2)
    assert failed.observation == Obs("no_pallet", "far_leg") and failed.attempt["recognition"]["status"] == "invalid"
    assert core.far_leg_recaptures == 0 and core.commands.waits[-1] == [51.0, 51.2]


def test_a_decision_done_resets_the_recapture_count():
    core, state, slam, answers, cancels, ends = far_core()
    core.observation_freeze(50.3, far_captured())
    answers.append(SimpleNamespace(state="invalid", why="leg", observation=Obs("no_pallet", "far_leg")))
    core.observation_take(50.5)
    core.observation_freeze(51.0, far_captured())
    answers.append(SimpleNamespace(state="done", observation=Obs("valid"), record={"source": "single"}))
    assert core.observation_take(51.2) is not None and core.far_leg_recaptures == 0


def test_a_timeout_raises_and_the_missions_end_closes_the_wait_and_drops_the_far_work():
    core, state, slam, answers, cancels, ends = far_core()
    core.observation_freeze(50.3, far_captured())
    answers.append(RuntimeError("recognition_timeout"))
    with pytest.raises(RuntimeError, match="recognition_timeout"):
        core.observation_take(80.4)
    core.abandon_jobs(80.5)
    assert core.observation_live is None and core.commands.waits[-1] == [50.3, 80.5]
    assert cancels == ["mission_end"] and ends == [80.5]
    assert state["recognition_discarded"][-1]["why"] == "mission_end"


def test_a_pause_drops_the_frozen_far_observation_and_its_work():
    core, state, slam, answers, cancels, ends = far_core()
    core.observation_freeze(50.3, far_captured())
    core.observation_drop(50.6, "paused")
    assert core.observation_live is None and cancels == ["paused"] and core.commands.waits[-1] == [50.3, 50.6]
    core.observation_drop(50.7, "paused")  # nothing left: nothing happens
    assert cancels == ["paused"]


def test_a_far_wait_does_not_open_a_second_wait_over_one_already_open():
    core, state, slam, answers, cancels, ends = far_core()
    core.commands.begin_wait(50.0)
    core.observation_freeze(50.3, far_captured())
    answers.append(SimpleNamespace(state="done", observation=Obs("valid"), record={"source": "fused"}))
    core.observation_take(50.5)
    assert core.commands.waits == [[50.0, None]]  # not its wait to close


@dataclass(frozen=True)
class Sc:
    start_rear: object = None


def observe_failed_core(waypoints=((5.0, 1.0, 0.0), (6.0, 1.0, 0.0), (7.0, 1.0, 0.0))):
    core, state, slam, calls = capture_core(FakeSession())
    core.options.observation_waypoints = [list(w) for w in waypoints]
    core.options.repeat_captures = 0
    core.options.scenario = Sc()
    core.options.planner_config = Config()
    core.options.observe_bounds = None
    core.options.pickup_bounds = "B"
    core.options.speeds, core.options.cusp_heading_hold_rad = {"observe": 0.35}, 0.08
    core.hooks.set_inset = lambda *a: None
    core.hooks.plan_observation_leg = lambda *a, **k: pytest.fail("a live run plans on the worker, not in the tick")
    installed = []
    core.observe_tracker = lambda plan: ("TRACKER", plan.status)
    core.apply_tracker_profile = lambda: installed.append("profile")
    core.realign_install = lambda why: installed.append(why)
    core.record_tracker_configs = lambda: installed.append("configs")
    state.update(observation_candidates=[], planning_wall_s=0.0)
    return core, state, slam, installed


def test_a_live_observe_failure_asks_the_worker_for_the_remaining_candidates_and_installs_the_first():
    core, state, slam, installed = observe_failed_core()
    core.ms.next_candidate_index = 1
    attempt = {}
    assert core.observe_failed(60.0, REAR, attempt, 2, Obs("no_pallet", "far_leg")) is False
    name, payload = core.jobs.submitted[-1]
    assert name == "observe_ladder" and payload["waypoints"] == [(6.0, 1.0, 0.0), (7.0, 1.0, 0.0)]
    assert payload["ladders"] == (False,) and payload["pickup_bounds"] is None  # a live run: the whole hall (D2)
    assert (payload["world"][1].start_rear.x_m, payload["world"][1].start_rear.yaw_rad) == (1.0, 0.5)
    assert core.commands.waits[-1] == [60.0, None]
    cand = lambda i, ok: {"candidate_index": i, "pose": [6.0 + i, 1.0, 0.0], "success": ok,
                          "status": "success" if ok else "no_path", "start_rear": list(REAR)}
    core.jobs.answers[1] = (True, {"result": Plan("success"), "selected": 1, "timed_out": False, "wall_s": 0.3,
                                   "candidates": [cand(0, False), cand(1, True)]})
    assert core.job_hold(60.5, REAR, 0.0, False).hold is True
    assert [c["candidate_index"] for c in state["observation_candidates"]] == [1, 2]
    assert core.ms.next_candidate_index == 3 and core.trackers["observe"] == ("TRACKER", "success")
    assert state["observation_waypoint_selected"] == [7.0, 1.0, 0.0] and installed[-2] == "observe_candidate"
    assert core.ms.phase_started == 60.5 and core.commands.waits[-1] == [60.0, 60.5]


def test_a_live_observe_failure_with_no_reachable_candidate_ends_the_mission_by_name():
    core, state, slam, installed = observe_failed_core()
    core.ms.next_candidate_index = 2
    core.observe_failed(60.0, REAR, {}, 2, Obs("no_pallet", "few"))
    core.jobs.answers[1] = (True, {"result": None, "selected": None, "timed_out": False, "wall_s": 0.1,
                                   "candidates": [{"candidate_index": 0, "pose": [7.0, 1.0, 0.0], "status": "no_path",
                                                   "success": False, "start_rear": list(REAR)}]})
    with pytest.raises(RuntimeError, match="^observe:all_candidates_exhausted:perception_no_pallet:few:no_path$"):
        core.job_hold(60.5, REAR, 0.0, False)


def test_a_dropped_re_observation_plan_is_asked_again_from_where_the_truck_stands():
    core, state, slam, installed = observe_failed_core()
    core.ms.next_candidate_index = 1
    core.observe_failed(60.0, REAR, {}, 2, Obs("no_pallet", "few"))
    slam["tracker"].applied = ((0.3, 0.0, 0.0), 0)  # a correction while it was planned
    core.jobs.answers[1] = (True, {"result": Plan("success"), "selected": 0, "timed_out": False, "candidates": []})
    core.job_hold(60.5, REAR, 0.0, False)
    assert core.observe_failed_needed == {"first": 1, "retry_reason": "perception_no_pallet:few"}
    core.job_hold(60.6, REAR, 0.0, False)
    assert len(core.jobs.submitted) == 2 and core.observe_failed_needed is None
    assert core.jobs.submitted[-1][1]["waypoints"] == [(6.0, 1.0, 0.0), (7.0, 1.0, 0.0)]
