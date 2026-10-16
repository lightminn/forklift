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
                       options=SimpleNamespace(obstacle_act=True, dt=1 / 120))
    core.commands.queue(Submit("P", "D"))
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
    core.slam = {"tracker": SimpleNamespace(applied=(np.array([0.1, 0.0, 0.0]),))}
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
