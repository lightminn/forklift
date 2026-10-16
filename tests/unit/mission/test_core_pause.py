"""M-a3b: the paused tick, the permission's watch, work time and the resume's general checks
(docs/plans/2026-10-16-m-a3-mission-commands.md E3, E4, E5, E6)."""

from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control import RearAxlePathTracker
from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.commands import Cancel, Pause, Resume, Submit
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState

REAR = np.array([0.0, 0.0, 0.0])  # at the start of the leg: a follower that has not moved rejoins here


def straight(x0=0.0, length=4.0):
    xs = np.linspace(x0, x0 + length, 41)
    poses = np.column_stack((xs, np.zeros_like(xs), np.zeros_like(xs)))
    return RearAxlePathTracker(poses, np.ones(len(xs)), np.zeros(len(xs)), TrackerConfig())


class Layer:
    def __init__(self, allowed=0.5, why="ok", blocked=None, scans=None, timeout=100.0):
        self.allowed, self.why, self.blocked = allowed, why, blocked
        self.calls = []
        self.last_scan_s = scans if scans is not None else {"front": 9.9}
        self.permission = SimpleNamespace(config=SimpleNamespace(sensor_timeout_s=timeout))
        self.known_enabled, self.known_config = False, {}

    def limit(self, t, **k):
        self.calls.append(("limit", k))
        return self.allowed, self.why

    def refresh(self, *a, **k):
        self.calls.append(("refresh", a, k))

    def check_candidate(self, leg, **k):
        self.calls.append(("check_candidate", k))
        return SimpleNamespace(blocked=self.blocked)


def make(phase="transport", *, speed=0.0, lift=0.07, layer=None, tracker=None, obstacle=True, monitor=None):
    calls = []

    def require(ok, reason):
        if not ok:
            raise RuntimeError(reason)

    hooks = SimpleNamespace(require=require, truth_speed=lambda: speed, lift_position=lambda: lift,
                            echo=lambda text: None, record_json=lambda v, indent=None: "{}", snapshot=lambda phase: None,
                            cusp_monitor=lambda: monitor, pallet_z=lambda: 0.1)
    trackers = {phase: tracker or straight()} if phase in ("observe", "approach", "transport", "return_home") else {}
    obs = None
    if obstacle:
        obs = {"layer": layer or Layer(), "applied": (0.0, 0.0, 0.0), "version": 1, "path_blocked_ticks": 3,
               "blocked_ticks": 7, "progress": {"phase": phase, "best_m": 2.0, "since_s": 1.0}}
    core = MissionCore(MissionState(phase=phase, phase_started=0.0), state={"transitions": []}, obstacle=obs,
                       slam=None, pocket={"check": None}, paths={phase: SimpleNamespace(status="success")},
                       trackers=trackers, geometry=SimpleNamespace(alignment_straight_m=1.0, delivery_straight_m=1.5),
                       settings={"lift_target_m": 0.2, "lift_rate_mps": 0.05}, pallet_geometry=None, hooks=hooks,
                       options=SimpleNamespace(obstacle_act=True, dt=1 / 120, pocket_check=False))
    from forklift_core.mission.targets import Pose

    core.targets.add_pallet("P", "epal6", Pose(5.0, 0.0, 0.0))
    core.targets.add_destination("D", Pose(9.0, 0.0, 0.0))
    core.commands.queue(Submit("P", "D", core.targets.revision))
    core.commands.apply(0.0)
    core.commands.start()
    return core, obs, calls


def pause(core, t):
    core.commands.queue(Pause())
    assert core.commands.apply(t)[0]["accepted"]


def resume(core, t):
    core.commands.queue(Resume())
    assert core.commands.apply(t)[0]["accepted"]


def test_the_paused_tick_asks_for_zero_follows_the_lift_and_keeps_the_permissions_cap():
    core, obs, _ = make(layer=Layer(allowed=0.0, why="occupied"), speed=0.3)
    pause(core, 10.0)
    core.ms.lift_command = 0.2
    out = core.paused_tick(10.0, REAR, 0.1, True)
    assert out.requested_speed == 0.0 and out.stop is False
    assert core.ms.lift_command == 0.07  # synchronised to where the lift is, not frozen at the target
    assert obs["permission_cap"] == (10.0, 0.0) and out.obstacle_hold is True  # an obstacle still steps to zero
    (_, limit_kwargs), = [c for c in obs["layer"].calls if c[0] == "limit"]
    assert limit_kwargs["cap_mps"] == pytest.approx(0.3) and limit_kwargs["direction"] == 1


def test_the_permission_watch_counts_no_work():
    core, obs, _ = make(layer=Layer(allowed=0.0, why="occupied"))
    pause(core, 10.0)
    for k in range(5):
        core.paused_tick(10.0 + k / 120, REAR, 0.0, True)
    assert obs["path_blocked_ticks"] == 3 and obs["blocked_ticks"] == 7
    assert obs["progress"] == {"phase": "transport", "best_m": 2.0, "since_s": 1.0}


def test_a_correction_change_while_paused_without_a_follower_reprojects_at_the_pose():
    core, obs, _ = make("settle")
    core.slam = {"tracker": SimpleNamespace(applied=(np.array([0.1, 0.0, 0.0]),), mode="tracking"), "holds": []}
    core.slam["stop_now"] = True
    pause(core, 10.0)
    core.paused_tick(10.0, REAR, 0.0, False)
    (_, args, kwargs), = [c for c in obs["layer"].calls if c[0] == "refresh"]
    assert args[1] == (0.1, 0.0, 0.0) and kwargs["path_ahead"].tolist() == [[0.0, 0.0, 0.0]]
    assert kwargs["direction"] == 0 and obs["permission_cap"] is None  # nothing to act on without a follower


def test_a_cancel_ends_once_standing():
    core, _, _ = make(speed=0.2)
    core.commands.queue(Cancel())
    core.commands.apply(10.0)
    assert core.paused_tick(10.0, REAR, 0.0, True).stop is False and core.commands.status == "cancelling"
    core.hooks.truth_speed = lambda: 0.0
    assert core.paused_tick(11.0, REAR, 0.0, True).stop is True
    assert core.commands.status == "cancelled" and core.commands.pauses == [[10.0, 11.0]]


def test_work_time_does_not_count_a_pause():
    core, _, _ = make("lift", obstacle=False)
    pause(core, 1.0)
    resume(core, 4.0)
    core.paused_tick(5.0, REAR, 0.0, False)  # standing, no follower in lift: resumed at 5
    assert core.commands.status == "running" and core.commands.pauses == [[1.0, 5.0]]
    assert core.lift_and_timers(8.4, 1 / 120) is False and core.ms.phase == "lift"  # 4.4 s of work
    core.lift_and_timers(8.6, 1 / 120)
    assert core.ms.phase == "extract"


def test_a_resume_waits_for_standing_then_rejoins_and_moves_the_deadlines():
    shifted = []
    monitor = SimpleNamespace(shift_time=lambda length: shifted.append(length))
    core, obs, _ = make(speed=0.2, monitor=monitor)
    obs["live_retry_after_s"] = 12.0
    pause(core, 10.0)
    resume(core, 11.0)
    core.paused_tick(11.0, REAR, 0.0, True)
    assert core.commands.status == "resuming" and core.commands.resume_check["step"] == "stop_confirmed"
    core.hooks.truth_speed = lambda: 0.0
    core.paused_tick(13.0, REAR, 0.0, True)
    assert core.commands.status == "running" and core.commands.resume_check["rejoined"] is True
    assert obs["live_retry_after_s"] == 15.0 and shifted == [3.0]


def test_a_resume_with_a_silent_sensor_stays_resuming():
    core, _, _ = make(layer=Layer(scans={"front": 5.0}, timeout=0.5))
    pause(core, 10.0)
    resume(core, 11.0)
    core.paused_tick(11.0, REAR, 0.0, True)
    assert core.commands.status == "resuming" and core.commands.resume_check["reason"] == "path_blocked:sensor_silent"


def test_a_resume_off_its_path_replans_from_here_before_running():
    core, obs, _ = make()
    pause(core, 10.0)
    resume(core, 11.0)
    off = np.array([0.0, 1.0, 0.0])  # a metre beside the leg: the follower would fail
    replanned = []

    def blocked_replan(t, rear, kappa, requested):
        replanned.append(t)
        core.paths["transport"] = SimpleNamespace(status="success")
        core.trackers["transport"] = straight(0.0, 4.0)
        core.trackers["transport"]._poses[:, 1] = 1.0  # the new path starts where the truck stands
        return SimpleNamespace(rear=rear, requested_speed=0.0)

    core.blocked_replan = blocked_replan
    core.paused_tick(11.0, off, 0.0, True)
    assert replanned == [11.0] and core.commands.status == "resuming"
    assert core.commands.resume_check["reason"] == "replanned"
    core.paused_tick(11.1, off, 0.0, True)
    assert core.commands.status == "running"


def test_a_resume_whose_replan_installs_nothing_waits_a_second():
    core, _, _ = make(layer=Layer(blocked="occupied"))
    pause(core, 10.0)
    resume(core, 11.0)
    asked = []
    core.blocked_replan = lambda t, rear, kappa, requested: asked.append(t) or SimpleNamespace(rear=rear,
                                                                                              requested_speed=0.0)
    for t in (11.0, 11.5, 12.0):
        core.paused_tick(t, REAR, 0.0, True)
    assert asked == [11.0, 12.0] and core.commands.resume_check["reason"] == "replan_waiting"


def test_a_resume_onto_a_recovery_path_runs_it():
    core, _, _ = make(layer=Layer(blocked="occupied"))
    pause(core, 10.0)
    resume(core, 11.0)

    def backoff(t, rear, kappa, requested):
        core.paths["transport"] = SimpleNamespace(status="backoff")
        return SimpleNamespace(rear=rear, requested_speed=0.0)

    core.blocked_replan = backoff
    core.paused_tick(11.0, REAR, 0.0, True)
    assert core.commands.status == "running" and core.commands.resume_check["reason"] == "recovery_installed"


def test_the_watch_keeps_the_pocket_checks_acting_phases_capped():
    """Codex M-a3b review: with the D5 check the insertion acts, so a zero from the layer still caps a pause."""
    core, obs, _ = make("insert", layer=Layer(allowed=0.0, why="sensor_silent:front"), speed=0.2,
                        tracker=straight())
    core.trackers = {"insert": core.trackers.get("insert") or straight()}
    core.options.pocket_check = True
    pause(core, 10.0)
    out = core.paused_tick(10.0, REAR, 0.0, False)
    assert obs["permission_cap"] == (10.0, 0.0) and out.obstacle_hold is True


@pytest.mark.parametrize("blocked", ["unknown", "edge"])
def test_a_resume_waits_while_the_path_ahead_is_unknown_or_at_an_edge(blocked):
    core, _, _ = make(layer=Layer(blocked=blocked))
    pause(core, 10.0)
    resume(core, 11.0)
    core.paused_tick(11.0, REAR, 0.0, True)
    assert core.commands.status == "resuming" and core.commands.resume_check["reason"] == f"path_blocked:{blocked}"


def test_a_resume_off_its_path_outside_the_free_travel_legs_is_not_replanned():
    """Codex M-a3b review: the common recovery's last branch plans home -- not for an approach."""
    core, _, _ = make("approach")
    core.blocked_replan = lambda *a: pytest.fail("the common recovery ran on an approach")
    pause(core, 10.0)
    resume(core, 11.0)
    core.paused_tick(11.0, np.array([0.0, 1.0, 0.0]), 0.0, False)
    assert core.commands.status == "paused" and core.commands.resume_check["reason"] == "no_resume:rejoin_failed"


def test_a_cancel_ends_only_once_the_wheels_are_commanded_to_zero():
    """Codex M-a3b review: standing is not enough while the slew still steps down."""
    core, _, _ = make()
    core.state["applied_command"] = (0.05, [0.0, 0.0])
    core.commands.queue(Cancel())
    core.commands.apply(10.0)
    assert core.paused_tick(10.0, REAR, 0.0, True).stop is False and core.commands.status == "cancelling"
    core.state["applied_command"] = (0.0, [0.0, 0.0])
    assert core.paused_tick(10.1, REAR, 0.0, True).stop is True and core.commands.status == "cancelled"


def test_a_resume_off_its_path_replans_even_when_the_old_path_is_hidden():
    """Codex M-a3b review: an unknown old path does not keep a displaced truck from replanning from here."""
    core, _, _ = make(layer=Layer(blocked="unknown"))
    pause(core, 10.0)
    resume(core, 11.0)
    asked = []
    core.blocked_replan = lambda t, rear, kappa, requested: asked.append(t) or SimpleNamespace(rear=rear,
                                                                                              requested_speed=0.0)
    core.paused_tick(11.0, np.array([0.0, 1.0, 0.0]), 0.0, True)
    assert asked == [11.0]


# --- M-a3c: the resume table's rows a general check cannot decide (M-a0.5 part 2, section 8) ---
def held(core, *, holds=1, docking=None):
    core.slam = {"tracker": SimpleNamespace(mode="holding", applied=None), "holds": [{"event": "hold"}] * holds,
                 "stop_now": True, "docking": dict(docking or {})}
    return core.slam


def test_a_held_realignment_does_not_resume():
    core, _, _ = make("observe")
    held(core)
    core.realign_active = lambda: True
    pause(core, 10.0)
    resume(core, 11.0)
    core.paused_tick(11.0, REAR, 0.0, False)
    assert core.commands.status == "paused" and core.commands.resume_check["reason"] == "no_resume:realign_held"


def test_a_new_hold_episode_while_paused_does_not_resume():
    core, _, _ = make("approach")
    slam = held(core, holds=1)
    pause(core, 10.0)
    core.paused_tick(10.0, REAR, 0.0, False)  # the stop's first tick records hold episode 1
    slam["holds"] = slam["holds"] + [{"event": "release"}, {"event": "hold"}]
    resume(core, 11.0)
    core.paused_tick(11.0, REAR, 0.0, False)
    assert core.commands.resume_check["reason"] == "state_verified:hold_episode_changed"


def docking_core(*, d7=True, retries=()):
    core, _, _ = make("transport")
    held(core, docking={"status": "armed"})
    core.options.d7_docking = d7
    core.state["docking_retries"] = list(retries)
    asked = []
    core.start_docking_retry = lambda t, trigger, failure: asked.append((t, trigger, failure))
    core.hooks.slam_rear = lambda t: np.array([0.05, 0.0, 0.0])
    pause(core, 10.0)
    core.paused_tick(10.0, REAR, 0.0, False)
    resume(core, 11.0)
    return core, asked


def test_a_docking_straight_resumes_where_it_stopped():
    core, asked = docking_core()
    core.paused_tick(11.0, REAR + np.array([0.005, 0.0, 0.0]), 0.0, False)  # within the stop's tolerance
    assert asked == [] and core.commands.status == "running"


def test_a_docking_straight_moved_while_paused_retries_the_docking_from_there():
    core, asked = docking_core()
    core.paused_tick(11.0, REAR + np.array([0.05, 0.0, 0.0]), 0.0, False)
    assert asked == [(11.0, "resume", "docking_moved_while_paused")]
    assert core.commands.status == "resuming" and core.commands.resume_check["reason"] == "docking_retry"
    assert core.commands.stop_pose == (0.05, 0.0, 0.0)  # the re-approach starts here: not moved again


@pytest.mark.parametrize("d7, retries, reason", [
    (False, (), "no_resume:docking_moved"),
    (True, ({"counted": True}, {"counted": True}), "no_resume:docking_retry_budget"),
])
def test_a_moved_docking_straight_without_d7_or_its_budget_does_not_resume(d7, retries, reason):
    core, asked = docking_core(d7=d7, retries=retries)
    core.paused_tick(11.0, REAR + np.array([0.05, 0.0, 0.0]), 0.0, False)
    assert asked == [] and core.commands.resume_check["reason"] == reason



# --- Codex M-a3c review: the rows' own scope ---
class Pocket:
    def __init__(self, allowed):
        self.allowed = allowed

    def limit(self, *a):
        return self.allowed, "frame_old" if self.allowed == 0.0 else "ok"


@pytest.mark.parametrize("allowed, status", [(0.0, "resuming"), (0.3, "running")])
def test_r4n_with_the_depth_check_resumes_only_on_its_answer(allowed, status):
    core, _, _ = make("approach", tracker=straight(0.0, 0.5))  # inside the 1 m alignment straight
    core.options.pocket_check = True
    core.pocket = {"check": Pocket(allowed)}
    pause(core, 10.0)
    resume(core, 11.0)
    core.paused_tick(11.0, REAR, 0.0, False)
    assert core.commands.status == status


def test_a_refused_resume_waits_for_a_new_resume_even_when_the_cause_goes():
    core, asked = docking_core(d7=False)
    core.paused_tick(11.0, REAR + np.array([0.05, 0.0, 0.0]), 0.0, False)
    assert core.commands.status == "paused"
    core.paused_tick(11.5, REAR, 0.0, False)  # back where it stood: still paused, no resume without a command
    assert core.commands.status == "paused"


def test_a_stop_that_began_held_does_not_resume_once_the_hold_is_gone():
    core, _, _ = make("approach")
    slam = held(core)
    pause(core, 10.0)
    core.paused_tick(10.0, REAR, 0.0, False)
    slam["tracker"].mode = "tracking"  # released while paused, no release was due
    resume(core, 11.0)
    core.paused_tick(11.0, REAR, 0.0, False)
    assert core.commands.status == "paused"
    assert core.commands.resume_check["reason"] == "state_verified:hold_state_mismatch"


def test_a_due_release_during_the_pause_is_not_a_lost_hold():
    core, _, _ = make("transport")
    slam = held(core)
    slam["pending_release"] = "transport"
    pause(core, 10.0)
    core.paused_tick(10.0, REAR, 0.0, False)
    slam["tracker"].mode, slam["pending_release"] = "tracking", None  # the release gate released it
    resume(core, 11.0)
    core.paused_tick(11.0, REAR, 0.0, False)
    assert core.commands.status == "running"


def test_a_held_near_segment_blocked_ahead_is_not_replanned():
    core, _, _ = make("observe", layer=Layer(blocked="occupied"))
    held(core)
    core.realign_active = lambda: False
    core.blocked_replan = lambda *a: pytest.fail("a held observe segment was replanned")
    pause(core, 10.0)
    resume(core, 11.0)
    core.paused_tick(11.0, REAR, 0.0, False)
    assert core.commands.status == "paused" and core.commands.resume_check["reason"] == "no_resume:path_blocked:occupied"


@pytest.mark.parametrize("phase", ["extract", "withdraw"])
def test_a_leg_without_the_permission_resumes_whatever_the_grid_says(phase):
    core, _, _ = make("transport", layer=Layer(blocked="unknown"))
    core.trackers = {phase: straight()}
    core.ms.phase = phase
    pause(core, 10.0)
    resume(core, 11.0)
    core.paused_tick(11.0, REAR, 0.0, True)
    assert core.commands.status == "running"


def test_a_re_observation_stand_resumes_to_capture_again():
    core, _, _ = make("observe", layer=Layer(blocked="unknown"))
    core.near = {"retry_stand": True, "tracker": None}
    pause(core, 10.0)
    resume(core, 11.0)
    core.paused_tick(11.0, REAR, 0.0, False)
    assert core.commands.status == "running"



def test_after_its_own_docking_retry_the_resume_takes_the_release_as_expected():
    """Codex M-a3c review: the retry releases the hold; the next tick must not call that a lost hold."""
    core, _ = docking_core()
    slam = core.slam

    def retry(t, trigger, failure):
        slam["tracker"].mode = "tracking"  # the retry released the correction
        slam["holds"].append({"event": "release"})
        slam["docking"]["reapproach"] = True

    core.start_docking_retry = retry
    core.paused_tick(11.0, REAR + np.array([0.05, 0.0, 0.0]), 0.0, False)
    assert core.commands.resume_check["reason"] == "docking_retry"
    core.paused_tick(11.1, np.array([0.05, 0.0, 0.0]), 0.0, False)
    assert core.commands.resume_check.get("reason") != "state_verified:hold_state_mismatch"
    assert core.commands.status in ("resuming", "running")



def test_a_release_the_resumes_replan_makes_is_not_a_lost_hold():
    """Codex M-a3c review: a re-approach or backoff replan inside the resume releases the hold on purpose."""
    core, _, _ = make("transport")
    slam = held(core, docking={"reapproach": True})
    pause(core, 10.0)
    core.paused_tick(10.0, REAR, 0.0, False)
    resume(core, 11.0)

    def replan(t, rear, kappa, requested):
        slam["tracker"].mode = "tracking"
        slam["holds"].append({"event": "release"})
        core.paths["transport"] = SimpleNamespace(status="success")
        core.trackers["transport"] = straight(0.0, 4.0)
        core.trackers["transport"]._poses[:, 1] = 1.0
        return SimpleNamespace(rear=rear, requested_speed=0.0)

    core.blocked_replan = replan
    core.paused_tick(11.0, np.array([0.0, 1.0, 0.0]), 0.0, False)  # off its path: replanned
    core.paused_tick(11.1, np.array([0.0, 1.0, 0.0]), 0.0, False)
    assert core.commands.status == "running"


def test_a_backoff_on_the_transport_is_not_the_docking_straight():
    """Codex M-a3c review: an active backoff with the docking still armed is a free leg (R10), not R11."""
    core, asked = docking_core(d7=False)
    core.obstacle["backoff"] = {"phase": "transport"}
    core.paused_tick(11.0, REAR + np.array([0.05, 0.0, 0.0]), 0.0, False)
    assert core.commands.resume_check.get("reason") != "no_resume:docking_moved"


# --- M-a3e: the safety state (plan E8) ---
def test_the_safety_state_reports_the_lifecycle_the_load_and_the_command_without_a_port_call():
    core, obs, _ = make("transport")
    core.hooks = SimpleNamespace()  # any port call would fail
    core.state["applied_command"] = (0.25, [0.1, 0.12])
    obs["permission_cap"] = (10.0, 0.4)
    out = core.safety_state(10.0)
    assert out["mission"]["status"] == "running" and out["mission"]["id"] == "m1"
    assert out["mission"]["phase"] == "transport" and out["mission"]["phase_elapsed_s"] == 10.0
    assert out["load"] == {"loaded": True, "basis": "phase", "lift_target_m": 0.0}
    assert out["fork_in_pocket"] == {"value": False, "basis": "phase"}
    assert out["command"] == {"applied_speed_mps": 0.25, "steering_rad": [0.1, 0.12]}
    assert out["blocked"] == {"cap": 0.4, "reason": None} and out["frame"]["correction_version"] == 1 and out["near"] is None
    import json

    json.dumps(out)  # one JSON line


def test_the_safety_state_shows_a_refused_resume_and_the_pause_in_work_time():
    core, _, _ = make()
    pause(core, 10.0)
    core.paused_tick(10.0, REAR, 0.0, True)
    resume(core, 11.0)
    core.commands.resume_check = {"time_s": 11.0, "step": "state_verified", "reason": "no_resume:x"}
    out = core.safety_state(12.0)
    assert out["mission"]["status"] == "resuming" and out["mission"]["resume"]["reason"] == "no_resume:x"
    assert out["mission"]["phase_elapsed_s"] == pytest.approx(10.0)  # 12 s, less the 2 s paused so far


def test_a_stop_not_seen_standing_for_ten_seconds_is_reported_and_the_reason_with_the_cap():
    """Codex M-a3d/e review: not_stopped and the permission's reason reach the safety state."""
    core, _, _ = make(speed=0.2, layer=Layer(allowed=0.0, why="sensor_silent:front"))
    pause(core, 10.0)
    core.paused_tick(15.0, REAR, 0.0, True)
    assert core.safety_state(15.0)["mission"]["not_stopped"] is False
    core.paused_tick(20.0, REAR, 0.0, True)
    out = core.safety_state(20.0)
    assert out["mission"]["not_stopped"] is True and out["blocked"]["reason"] == "sensor_silent:front"


def test_the_safety_state_before_the_first_tick_has_no_work_time():
    core, _, _ = make()
    assert core.safety_state(None)["mission"]["phase_elapsed_s"] is None
