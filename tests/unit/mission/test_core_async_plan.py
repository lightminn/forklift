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
