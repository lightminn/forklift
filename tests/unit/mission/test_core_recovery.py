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
