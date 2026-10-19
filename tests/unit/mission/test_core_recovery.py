"""Persistent console plan D4 (6b, 6th ed.): a live cancel that stood with the forks in a pocket backs them out --
recover_lower (lift down, on the floor), then recover_withdraw (the insert trace retraced to its margin) -- and ends
cancelled; refused, timed out or without its evidence it ends recovery_required."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.commands import Cancel, Pause, Resume

from test_core_pause import Layer, make  # noqa: E402  (the paused tick's fixtures)


def inserted_core(*, lift=0.0, pallet_z=0.0, layer=None, depth=0.30):
    core, obs, _ = make("insert", lift=lift, layer=layer or Layer(verified_m=10.0))  # a clear path, verified whole
    core.hooks.pallet_z = lambda: pallet_z
    core.hooks.slam_rear = lambda t: None  # no SLAM in these cores
    core.hooks.path_record = lambda path: {"status": path.status}
    core.baseline = object()  # a live session
    from dataclasses import replace as _replace

    from test_core_pause import straight

    withdraw_ = straight()  # the mission plan's withdraw tracker: its speed and tolerance are the retrace's
    withdraw_.config = _replace(withdraw_.config, cruise_speed_mps=0.1, position_tolerance_m=0.008)
    core.trackers["withdraw"] = withdraw_
    core.settings.update(withdraw_speed_mps=0.1)
    core.state["applied_command"] = (0.0, (0.0, 0.0))
    core.cargo.on_transition("approach", "insert", "P")
    core.cargo.observe("insert", (0.0, 0.0, 0.0))  # the anchor: the approach's end
    for s in np.arange(0.0, depth + 0.10 + 1e-9, 0.01):
        core.cargo.observe("insert", (s, 0.0, 0.0))
    rear = np.array([depth + 0.10, 0.0, 0.0])
    core.commands.adopted_id = core.commands.mission_id
    core.commands.queue(Cancel())
    core.commands.apply(5.0)
    return core, rear


def stand(core, t, rear):
    return core.paused_tick(t, rear, 0.0, False)


def test_a_cancel_standing_with_the_forks_in_a_pocket_starts_the_recovery():
    core, rear = inserted_core()
    out = stand(core, 6.0, rear)
    assert not out.stop and core.commands.status == "recovering" and core.ms.phase == "recover_lower"


def test_the_lowered_truck_retraces_the_insert_and_ends_cancelled():
    core, rear = inserted_core()
    stand(core, 6.0, rear)
    assert not core.lift_and_timers(6.5, 1 / 120)  # under a second: the verdict waits
    core.lift_and_timers(7.2, 1 / 120)
    assert core.ms.phase == "recover_withdraw" and core.cargo.load == "none"
    path = core.paths["recover_withdraw"]
    assert path.poses[0][0] == pytest.approx(rear[0], abs=0.011) and np.all(path.directions == -1)
    assert path.poses[-1][0] == pytest.approx(-core.cargo.band(rear), abs=1e-6)
    assert "recover_withdraw" in core.trackers
    acting, _ = core.permission_acting()
    assert acting  # the permission acts on the recovery's reverse
    out = core.arrive(9.0, SimpleNamespace(status="arrived"))
    assert not out.stop and core.commands.status == "cancelled" and core.cargo.relation == "free"


def test_a_blocked_retrace_is_recovery_required():
    core, rear = inserted_core(layer=Layer(blocked="occupied", verified_m=0.05))
    stand(core, 6.0, rear)
    core.lift_and_timers(7.2, 1 / 120)
    assert core.commands.status == "recovery_required" and core.commands.end_reason.startswith("recovery:blocked")
    assert core.cargo.relation == "constrained"


def test_a_lift_that_does_not_come_down_times_out():
    core, rear = inserted_core(lift=0.10, pallet_z=0.05)
    stand(core, 6.0, rear)
    core.lift_and_timers(20.0, 1 / 120)
    assert core.commands.status == "recovering"
    core.lift_and_timers(26.5, 1 / 120)
    assert core.commands.status == "recovery_required" and "lower" in core.commands.end_reason


def test_an_invalid_trace_refuses_before_moving():
    core, rear = inserted_core()
    core.cargo.trace_valid = False
    out = stand(core, 6.0, rear)
    assert core.commands.status == "recovery_required" and out.stop


def test_a_pause_in_the_retrace_resumes_from_the_depth_now():
    core, rear = inserted_core()
    stand(core, 6.0, rear)
    core.lift_and_timers(7.2, 1 / 120)
    core.commands.queue(Pause())
    core.commands.apply(8.0)
    assert core.commands.status == "paused"
    now = np.array([rear[0] - 0.15, 0.0, 0.0])  # backed 0.15 m before the pause
    core.cargo.observe("recover_withdraw", tuple(now))
    core.commands.queue(Resume())
    core.commands.apply(9.0)
    core.resume_step(9.0, now, False, True)
    assert core.commands.status == "recovering"
    assert core.paths["recover_withdraw"].poses[0][0] == pytest.approx(now[0], abs=0.011)


def lowered(core, rear):
    stand(core, 6.0, rear)
    core.lift_and_timers(7.2, 1 / 120)
    assert core.ms.phase == "recover_withdraw"


def test_the_retrace_runs_at_the_withdraw_speed_with_its_tolerance():
    """Codex 6b impl round 1 P1-2: the first tracker of a live session is the observation's (0.60 m/s, 0.03 m)."""
    core, rear = inserted_core()
    core.settings.update(tracker_curvature_inv_m=0.4, drive_acceleration_mps2=0.3)
    lowered(core, rear)
    config = core.trackers["recover_withdraw"].config
    assert config.cruise_speed_mps == pytest.approx(0.1) and config.position_tolerance_m == pytest.approx(0.008)


def test_a_resume_checks_the_trace_and_the_floor_again():
    """Codex 6b impl round 1 P1-3: paused in the retrace, the evidence lost or the pallet not down: no blind retrace."""
    core, rear = inserted_core()
    lowered(core, rear)
    core.commands.queue(Pause())
    core.commands.apply(8.0)
    core.cargo.trace_valid = False
    core.commands.queue(Resume())
    core.commands.apply(9.0)
    core.resume_step(9.0, rear, False, True)
    assert core.commands.status == "recovery_required"
    core2, rear2 = inserted_core()
    lowered(core2, rear2)
    core2.commands.queue(Pause())
    core2.commands.apply(8.0)
    core2.hooks.lift_position, core2.hooks.pallet_z = (lambda: 0.08), (lambda: 0.04)
    core2.commands.queue(Resume())
    core2.commands.apply(9.0)
    core2.resume_step(9.0, rear2, False, True)
    assert core2.ms.phase == "recover_lower"


def test_a_pause_in_the_lowering_holds_the_lift_where_it_is():
    core, rear = inserted_core(lift=0.10, pallet_z=0.05)
    stand(core, 6.0, rear)
    core.ms.lift_command = 0.03
    core.commands.queue(Pause())
    core.commands.apply(7.0)
    core.paused_tick(7.1, rear, 0.0, False)
    assert core.ms.lift_command == pytest.approx(0.10)


def test_a_resume_already_out_ends_the_recovery_cleanly():
    """Codex 6b impl round 1 P2-6: recovered while resuming raised (a session failure)."""
    core, rear = inserted_core()
    lowered(core, rear)
    core.commands.queue(Pause())
    core.commands.apply(8.0)
    out = np.array([-0.30, 0.0, 0.0])  # backed past the end by hand-free coasting
    core.commands.queue(Resume())
    core.commands.apply(9.0)
    core.resume_step(9.0, out, False, True)
    assert core.commands.status == "cancelled" and core.cargo.relation == "free"


def test_the_recovery_path_is_a_full_path_record():
    """Codex 6b impl round 1 P2-7: path_record reads expanded_nodes (a tracking failure's dump calls it first)."""
    from forklift_core.mission.core import RecoveryPath

    path = RecoveryPath(np.zeros((2, 3)), np.array([-1, -1]), np.zeros(2), 0.02)
    for field in ("success", "status", "poses", "directions", "curvatures_inv_m", "length_m", "expanded_nodes"):
        assert hasattr(path, field)


def test_a_finished_recovery_puts_the_pallet_back_in_the_targets():
    """Codex 6b impl round 1 P1-1: a recovery after the lift left the pallet carried with no pose."""
    from forklift_core.planning.pallet_mission import PalletSite

    core, rear = inserted_core()
    core.obstacle["pickup_estimate"] = PalletSite(0.75, 0.0, 0.0)
    core.targets.carried("P")
    lowered(core, rear)
    core.arrive(9.0, SimpleNamespace(status="arrived"))
    pallet = core.targets.pallets["P"]
    assert pallet.state == "available" and pallet.pose is not None and pallet.pose.x_m == pytest.approx(0.75)


def test_an_evidence_lost_after_the_install_stops_before_any_move():
    """Codex 6b impl round 2 P1: the tick's start ends it -- before the follower and the command run."""
    core, rear = inserted_core()
    lowered(core, rear)
    core.cargo.trace_valid = False
    core.release_gate(8.0, rear)
    assert core.commands.status == "recovery_required" and core.commands.end_reason == "recovery:trace_invalid"


def test_a_resume_waiting_installs_the_retrace_once():
    """Codex 6b impl round 2 P2: every tick of a waiting resume installed it again (records without bound)."""
    core, rear = inserted_core(layer=Layer(verified_m=10.0, scans={"front": 0.0}, timeout=0.1))  # silent: it waits
    lowered(core, rear)
    core.commands.queue(Pause())
    core.commands.apply(8.0)
    core.commands.queue(Resume())
    core.commands.apply(9.0)
    for k in range(10):
        core.resume_step(9.0 + 0.01 * k, rear, False, True)
    assert len(core.state["recoveries"]) == 2  # the first install and this resume's
    from forklift_core.mission.baseline import MISSION_KEYS

    assert "recoveries" in MISSION_KEYS["state"]  # a mission's record, handed over at its end


# --- plan D4b (6b-2, 4th ed.): the cancel ends where the next preparation can start -----------------------------------
def gate_core(ok_after, **k):
    """A recovery core whose preparation start check passes once the truck is ``ok_after`` m behind its pose now."""
    core, rear = inserted_core(**k)
    calls = []

    def start_ok(pose, *, extra_m=0.0, preview=True, t=None):
        assert t is not None  # the release time the preview builds its grid at (Codex D4b impl round 2 P2-1)
        calls.append((tuple(round(v, 4) for v in pose), extra_m, preview))
        return float(pose[0]) <= core._gate_origin - ok_after - extra_m

    core._gate_origin = float(rear[0])
    core.hooks.prepare_start_ok = start_ok
    return core, rear, calls


def test_a_recovery_ending_inside_the_preparations_clearance_backs_out_further():
    core, rear, calls = gate_core(ok_after=0.70)  # the retrace ends ~0.44 m back: not yet clear
    lowered(core, rear)
    end = core.paths["recover_withdraw"].poses[-1]
    core._gate_origin = float(rear[0])
    core.arrive(9.0, SimpleNamespace(status="arrived"))
    assert core.commands.status == "recovering" and core.ms.phase == "clear_out"
    path = core.paths["clear_out"]
    assert np.all(path.directions == -1)
    assert path.poses[-1][0] <= core._gate_origin - 0.70 - 0.08 + 1e-6  # the first sample clear with the margin
    assert any(extra == pytest.approx(0.08) for _, extra, _ in calls)
    assert "clear_out" in core.trackers and core.permission_acting()[0]


def test_a_clear_out_ends_cancelled_when_the_final_check_passes_or_extends_by_steps():
    core, rear, calls = gate_core(ok_after=0.70)
    lowered(core, rear)
    core.arrive(9.0, SimpleNamespace(status="arrived"))  # -> clear_out
    core._gate_origin -= 0.10  # at the arrival the final check (no margin) still fails (the release moved it)
    core.cargo.last_rear = tuple(core.paths["clear_out"].poses[-1])
    core.arrive(10.0, SimpleNamespace(status="arrived"))
    assert core.ms.phase == "clear_out" and core.commands.status == "recovering"  # extended 0.05 m
    assert core.paths["clear_out"].length_m == pytest.approx(0.05, abs=1e-6)
    core.cargo.last_rear = tuple(core.paths["clear_out"].poses[-1])
    core.arrive(11.0, SimpleNamespace(status="arrived"))
    assert core.commands.status == "cancelled"


def test_no_clear_start_within_the_budget_is_recovery_required():
    core, rear, _ = gate_core(ok_after=5.0)
    lowered(core, rear)
    core.arrive(9.0, SimpleNamespace(status="arrived"))
    assert core.commands.status == "recovery_required" and "clear_out" in core.commands.end_reason


def test_a_cancel_standing_clear_of_the_next_start_ends_at_once_but_a_near_one_clears_out():
    # free (approach) cancel: the gate decides, not the phase name (Codex 6b-2 round 1 P2-3)
    core, _, _ = make("approach")
    core.baseline = object()
    core.hooks.prepare_start_ok = lambda pose, extra_m=0.0, preview=True, t=None: True
    core.state["applied_command"] = (0.0, (0.0, 0.0))
    core.commands.adopted_id = core.commands.mission_id
    core.commands.queue(Cancel())
    core.commands.apply(5.0)
    core.paused_tick(6.0, np.array([0.0, 0.0, 0.0]), 0.0, False)
    assert core.commands.status == "cancelled"


def test_the_recovered_pallet_is_back_in_the_targets_at_the_ground_verdict():
    """Codex 6b-2 round 2 P2-2: the recovery's end check needs it there before the mission ends."""
    from forklift_core.planning.pallet_mission import PalletSite

    core, rear = inserted_core()
    core.obstacle["pickup_estimate"] = PalletSite(0.75, 0.0, 0.0)
    core.targets.carried("P")
    stand(core, 6.0, rear)
    core.lift_and_timers(7.2, 1 / 120)
    assert core.commands.status == "recovering" and core.targets.pallets["P"].state == "available"


def approach_cancel(gate, trace=None):
    """A live approach cancelled standing; ``trace``: the odometry rear poses the core saw on the way."""
    core, _, _ = make("approach", layer=Layer(verified_m=10.0))
    core.baseline = object()
    core.hooks.prepare_start_ok = gate
    core.hooks.path_record = lambda path: {"status": path.status}
    core.state["applied_command"] = (0.0, (0.0, 0.0))
    core.settings.update(withdraw_speed_mps=0.1)
    for pose in trace or []:
        core.observe_motion(pose)
    core.commands.adopted_id = core.commands.mission_id
    core.commands.queue(Cancel())
    core.commands.apply(5.0)
    return core


def test_a_near_cancel_after_driving_forward_retraces_the_odometry_trajectory():
    # an arc driven forward: the back-out runs back over it, newest pose first, until the start check passes
    trace = [(2.5 * math.sin(a), 2.5 * (1 - math.cos(a)), a) for a in np.linspace(0.0, 0.32, 41)]  # 0.8 m, kappa 0.4
    gate = lambda pose, extra_m=0.0, preview=True, t=None: pose[2] <= 0.16 - 1e-9  # clear once back past the arc's middle
    core = approach_cancel(gate, trace)
    rear = np.array(trace[-1])
    core.paused_tick(6.0, rear, 0.0, False)
    assert core.commands.status == "recovering" and core.ms.phase == "clear_out"
    path = core.paths["clear_out"]
    assert np.all(path.directions == -1)
    assert 0.12 < path.poses[-1][2] <= 0.16  # the first trace pose back past the middle (0.02 m sampling)
    on_arc = [abs(math.hypot(p[0], p[1] - 2.5) - 2.5) for p in path.poses]
    assert max(on_arc) < 1e-6
    # the arc's own curvature (radius 2.5 m) as the tracker's feed-forward, in the planner's convention
    # (turn = signed distance x curvature): reversing back over a left arc is still curvature +0.4
    assert np.allclose(path.curvatures_inv_m[1:], 0.4, atol=0.005)


def test_a_near_cancel_after_reversing_backs_straight_along_the_heading():
    trace = [(1.0 - s, 0.0, 0.0) for s in np.arange(0.0, 0.3, 0.02)]  # the last motion went backward
    gate = lambda pose, extra_m=0.0, preview=True, t=None: pose[0] <= 0.40 - extra_m
    core = approach_cancel(gate, trace)
    rear = np.array(trace[-1])
    core.paused_tick(6.0, rear, 0.0, False)
    path = core.paths["clear_out"]
    assert core.ms.phase == "clear_out" and np.allclose(path.poses[:, 1:], 0.0)
    assert path.poses[-1][0] <= 0.40 - 0.08 + 1e-9 and path.poses[-1][0] > 0.40 - 0.08 - 0.021


def test_a_paused_clear_out_resumes_on_its_stored_path_and_budget():
    trace = [(1.0 - s, 0.0, 0.0) for s in np.arange(0.0, 0.3, 0.02)]
    gate = lambda pose, extra_m=0.0, preview=True, t=None: pose[0] <= 0.10 - extra_m
    core = approach_cancel(gate, trace)
    rear = np.array(trace[-1])
    core.paused_tick(6.0, rear, 0.0, False)
    stored = core.paths["clear_out"].poses.copy()
    used = core.clear_out_used_m
    core.commands.queue(Pause())
    core.commands.apply(7.0)
    now = np.array([rear[0] - 0.20, 0.0, 0.0])  # 0.20 m along it before the pause
    core.commands.queue(Resume())
    core.commands.apply(8.0)
    core.resume_step(8.0, now, False, True)
    assert core.commands.status == "recovering" and core.ms.phase == "clear_out"
    rest = core.paths["clear_out"].poses
    assert rest[0][0] == pytest.approx(now[0], abs=0.011) and rest[-1][0] == pytest.approx(stored[-1][0])
    assert core.clear_out_used_m == pytest.approx(used, abs=0.011)  # the budget is not spent twice


def test_a_carrying_cancel_keeps_the_cargo_without_a_back_out():
    """Codex D4b impl round 1 P1-1: the gate and the back-out are for empty forks; a loaded cancel keeps the cargo
    (plan D5a: cargo_held), whatever the start check says."""
    core = approach_cancel(lambda pose, extra_m=0.0, preview=True, t=None: False)
    core.cargo.relation, core.cargo.load = "carrying", "confirmed"
    core.paused_tick(6.0, np.array([0.0, 0.0, 0.0]), 0.0, False)
    assert core.commands.status == "cancelled" and core.ms.phase != "clear_out"
    assert (core.cargo.relation, core.cargo.load) == ("carrying", "confirmed")


def test_a_trace_with_gear_changes_retraces_only_its_last_forward_run_then_backs_straight():
    """Codex D4b impl round 1 P1-2: forward 0 -> 0.30, back to 0.10, forward to 0.20 -- the whole trace reversed
    had forward segments marked reverse. Only the last forward run is retraced, then straight back."""
    xs = list(np.arange(0.0, 0.30 + 1e-9, 0.02)) + list(np.arange(0.28, 0.10 - 1e-9, -0.02)) + list(
        np.arange(0.12, 0.20 + 1e-9, 0.02))
    gate = lambda pose, extra_m=0.0, preview=True, t=None: pose[0] <= -0.20 - extra_m
    core = approach_cancel(gate, [(x, 0.0, 0.0) for x in xs])
    core.paused_tick(6.0, np.array([0.20, 0.0, 0.0]), 0.0, False)
    path = core.paths["clear_out"]
    assert core.ms.phase == "clear_out" and np.all(path.directions == -1)
    assert np.all(np.diff(path.poses[:, 0]) < 0)  # every step backward along the heading
    assert path.poses[-1][0] <= -0.28 + 1e-9


def test_a_resume_continues_from_the_progress_made_not_the_nearest_sample():
    """Codex D4b impl round 1 P1-2 (resume): the rest starts where the follower got to."""
    trace = [(1.0 - s, 0.0, 0.0) for s in np.arange(0.0, 0.3, 0.02)]
    core = approach_cancel(lambda pose, extra_m=0.0, preview=True, t=None: pose[0] <= 0.10 - extra_m, trace)
    rear = np.array(trace[-1])
    core.paused_tick(6.0, rear, 0.0, False)
    tracker = core.trackers["clear_out"]
    tracker.remaining_to_goal_m = lambda: core.paths["clear_out"].length_m - 0.30  # 0.30 m done
    core.commands.queue(Pause())
    core.commands.apply(7.0)
    core.commands.queue(Resume())
    core.commands.apply(8.0)
    used = core.clear_out_used_m
    core.resume_step(8.0, rear, False, True)  # standing back at the start (a hand push): still from 0.30 m on
    rest = core.paths["clear_out"].poses
    assert rest[1][0] <= rear[0] - 0.30 + 0.06
    # Codex D4b impl round 2 P2-2: the way back onto the path is part of it -- from where the truck stands, checked
    # with the rest by the permission -- and the budget takes what it adds beyond the rest left (round 3): the push
    assert tuple(rest[0]) == pytest.approx(tuple(rear))
    assert core.clear_out_used_m == pytest.approx(used + 0.30, abs=1e-6)


def test_a_resume_whose_way_back_onto_the_path_exceeds_the_budget_is_recovery_required():
    trace = [(1.0 - s, 0.0, 0.0) for s in np.arange(0.0, 0.3, 0.02)]
    core = approach_cancel(lambda pose, extra_m=0.0, preview=True, t=None: pose[0] <= -0.20 - extra_m, trace)
    rear = np.array(trace[-1])
    core.paused_tick(6.0, rear, 0.0, False)
    assert core.clear_out_used_m == pytest.approx(1.0, abs=0.021)  # the whole budget planned
    core.trackers["clear_out"].remaining_to_goal_m = lambda: core.paths["clear_out"].length_m - 0.50
    core.commands.queue(Pause())
    core.commands.apply(7.0)
    core.commands.queue(Resume())
    core.commands.apply(8.0)
    core.resume_step(8.0, rear, False, True)  # pushed 0.50 m back up the path: the budget cannot cover it again
    assert core.commands.status == "recovery_required" and "budget" in core.commands.end_reason


def test_a_clear_out_whose_hold_was_released_while_paused_is_recovery_required():
    """Codex D4b impl round 1 P2-5: the general hold check refused (paused) before the back-out's own."""
    trace = [(1.0 - s, 0.0, 0.0) for s in np.arange(0.0, 0.3, 0.02)]
    core = approach_cancel(lambda pose, extra_m=0.0, preview=True, t=None: pose[0] <= 0.10 - extra_m, trace)
    tracker = SimpleNamespace(mode="tracking", applied=((0.0, 0.0, 0.0), 0), received=((0.0, 0.0, 0.0), 0, 0))
    tracker.hold = lambda odom_from_base=None: setattr(tracker, "mode", "holding")
    core.slam = {"tracker": tracker, "holds": [], "docking": {}, "stop_now": True}
    core.hooks.slam_odom_base = lambda: (0.0, 0.0, 0.0)
    rear = np.array(trace[-1])
    core.paused_tick(6.0, rear, 0.0, False)
    assert core.ms.phase == "clear_out" and tracker.mode == "holding"
    core.commands.queue(Pause())
    core.commands.apply(7.0)
    tracker.mode = "tracking"  # released from outside
    core.commands.queue(Resume())
    core.commands.apply(8.0)
    core.resume_step(8.0, rear, False, True)
    assert core.commands.status == "recovery_required" and "hold_released" in core.commands.end_reason


@pytest.mark.parametrize("done", [0.305, 0.315])
def test_a_resume_standing_between_samples_on_a_full_budget_path_goes_on(done):
    """Codex D4b impl round 3-4 P2: stopped between samples of a 1.0 m back-out and not moved, the rest starts at
    the truck's projection on the path in its order -- never back to a sample already passed -- and the budget
    takes nothing more."""
    trace = [(1.0 - s, 0.0, 0.0) for s in np.arange(0.0, 0.3, 0.02)]
    core = approach_cancel(lambda pose, extra_m=0.0, preview=True, t=None: pose[0] <= -0.20 - extra_m, trace)
    rear = np.array(trace[-1])
    core.paused_tick(6.0, rear, 0.0, False)
    length = core.paths["clear_out"].length_m
    used = core.clear_out_used_m
    core.trackers["clear_out"].remaining_to_goal_m = lambda: length - done
    core.commands.queue(Pause())
    core.commands.apply(7.0)
    core.commands.queue(Resume())
    core.commands.apply(8.0)
    stood = np.array([rear[0] - done, 0.0, 0.0])  # between the 0.30 and 0.32 samples
    core.resume_step(8.0, stood, False, True)
    assert core.commands.status == "recovering" and core.ms.phase == "clear_out"
    assert core.clear_out_used_m == pytest.approx(used, abs=1e-6)
    rest = core.paths["clear_out"]
    assert rest.length_m == pytest.approx(length - done, abs=1e-6)
    assert np.all(np.diff(rest.poses[:, 0]) < 0)  # every step backward: no way back to a passed sample
