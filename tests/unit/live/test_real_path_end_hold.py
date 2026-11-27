"""U3e-1b (docs/plans/2026-11-27-real-path-end-hold.md 4): the real truck standing just short of a path end, one tick
on real parts -- the encoder safety speed from two STATEs (``real_teleop.encoder_speed``), the obstacle layer's
``DrivePermission`` on an empty grid and a short path, ``MissionCore.permission`` with the D5 bypass check.

The standing reading's error width (two directions, about 0.021 m/s at a 27.8 ms delay bound) is above the path-end
deceleration (``max(speed_for(path_left + margin), end_creep 0.02)``, about 0.0208 m/s 11.26 mm before the end): the
operating end-to-end run stalled there (ws1 21634). In the D5 stages the possible motion -- not the new command -- may
stop within the trackers' arrival tolerance (8 mm) past the end; every other stage, and the new command, as before.
Codex's counterexamples (a moving truck, a two-direction reading of one, a stale standing reading) stay held."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.live import real_teleop as rt
from forklift_core.live.real_assembly import BypassPocketCheck
from forklift_core.mission.core import MissionCore
from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot
from forklift_core.planning.geometry import Footprint
from forklift_core.plant.bringup import Sample
from forklift_core.safety.truth_access import SafetySpeed

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission"))
from test_core_truth_split import permission_core  # noqa: E402

FOOT = Footprint(1.29, 0.17, 0.36)  # the measured chassis outline (obstacle_layer_real.yaml bridge.footprint)
BODY = Footprint(0.94, 0.17, 0.36)
# the operating stop model as the run builds it from obstacle_layer_real.yaml (latency 0.30 + the scan timing, 0.52)
STOP = StoppingModel(latency_s=0.52, decel_mps2=0.5, margin_m=0.05)
CONFIG = PermissionConfig(STOP, envelope_offset_m=0.01, evidence_max_age_s=0.35, envelope_ramp_m=0.0,
                          sensor_timeout_s=0.35)
LEFT = 0.01126  # rear to the path end in the failing run (approach end 0.910, stalled at 0.89874)
T = 10.0
R, CPR, TRACK = 0.125, 327680.0, 0.53  # config/real/vehicle.yaml
P = rt.EncoderSpeedParams(radius_m=(R, R), counts_per_rev=(CPR, CPR), track_m=TRACK, kappa_max=0.366,
                          axle_accel_mps2=0.5, max_age_s=0.1, dt_min_s=0.02, dt_max_s=0.2)  # real_run's values


def reading(delay_s=0.0278, dt_ms=20, moved_counts=0):
    """Two STATEs ``dt_ms`` apart, each wheel ``moved_counts`` on, the newest received now, its delay bound
    ``delay_s``."""
    now = 50_000_000_000

    def st(mcu_ms, rx_ns, c):
        return Sample(rx_ns, McuState(int(mcu_ms * 1000), 7, 1, 0, Mode.ARMED, RejectReason.NONE, FaultFlag(0), c, c,
                                      0.0, 0.0, 0.0, 0.0, (0,) * 4, (0,) * 4, (0xC0,) * 4, 0, 0, StateFlag(0)))

    return rt.encoder_speed([st(1000, now - dt_ms * 1_000_000, 1000), st(1000 + dt_ms, now, 1000 + moved_counts)],
                            now, P, lambda q: True, delay_s).speed


def standing_speed(delay_s=0.0278):
    """Two STATEs 20 ms apart with the same wheel counts."""
    speed = reading(delay_s)
    assert speed.valid and speed.directions == (1, -1) and speed.signed_mps == 0.0
    return speed


class PermissionLayer:
    """The core's layer port on a real ``DrivePermission`` (the RealLayer's own call, without its worker)."""

    known_enabled = False
    known_config = {}

    def __init__(self, perm):
        self.permission = perm
        self.calls = []

    @property
    def snapshot(self):  # the core's diagnostics dump of an occupied stop reads it
        return self.permission.snapshot

    def limit(self, t, *, current_pose, curvature_inv_m, direction, loaded, cap_mps, evidence_s=None,
              path_end_slack_m=0.0, path_end_ahead_m=None):
        self.calls.append((direction, cap_mps, path_end_slack_m))
        ev = t if evidence_s is None else evidence_s
        kw = {"path_end_slack_m": path_end_slack_m, "path_end_ahead_m": path_end_ahead_m} if path_end_slack_m else {}
        return self.permission.allowed_speed(ev, {"low": T}, current_pose=current_pose,
                                             curvature_inv_m=curvature_inv_m, direction=direction, footprint=FOOT,
                                             own_footprint=BODY, speed_cap_mps=cap_mps, **kw)


def grid(*occupied):
    state = np.full((200, 120), FREE, dtype=np.uint8)  # x -1..9, y -3..3
    free = np.full(state.shape, T)
    for x, y in occupied:
        i, j = int(math.floor((x + 1.0) / 0.05)), int(math.floor((y + 3.0) / 0.05))
        state[i, j], free[i, j] = OCCUPIED, np.nan
    return GridSnapshot(state, free, T, (0.0, 0.0, 0.0), 0, 1, {"low": T}, -1.0, -3.0, 0.05)


class LegTo:
    """A tracker stand-in whose current leg ends at ``end`` (the core's other reads: as test_core_truth_split's)."""

    def __init__(self, end, direction=1):
        self.end, self.direction = end, direction

    def leg_ahead(self):
        return np.array([self.end]), self.direction

    def remaining_to_goal_m(self):
        return 5.0

    def progress_m(self):
        return 1.0


def core_at_end(speed, *occupied, phase="insert", left=LEFT, leg_left=None):
    """The truck at the origin, the layer's path ``left`` long; the tracker's leg ends ``leg_left`` ahead (default:
    the same end)."""
    perm = DrivePermission(CONFIG)
    path = np.array([[0.0, 0.0, 0.0], [left, 0.0, 0.0]])
    perm.update(grid(*occupied), path, FOOT, BODY, current_pose=(0.0, 0.0, 0.0), direction=1)
    c, _ = permission_core(speed, phase=phase, pocket_check=BypassPocketCheck((0.0, 0.0), 1.8, 0.8, 0.0))
    c.trackers[phase] = LegTo((left if leg_left is None else leg_left, 0.0, 0.0))
    layer = PermissionLayer(perm)
    c.obstacle["layer"] = layer
    return c, layer, perm


def path_end_limit(left=LEFT):
    return max(STOP.speed_for(left + STOP.margin_m), CONFIG.end_creep_mps)


def test_the_failing_ticks_numbers():
    """The two values the stall is made of: the standing width above the path-end deceleration."""
    assert standing_speed().bound_mps > path_end_limit()
    assert standing_speed(0.0183).bound_mps < path_end_limit()  # the passing run's delay bound at the same spot


@pytest.mark.parametrize("phase", ["insert", "approach"])
def test_a_standing_truck_just_short_of_the_end_creeps_on(phase):
    c, layer, _ = core_at_end(standing_speed(), phase=phase)
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert not out.obstacle_hold and c.state.get("motion_hold") is None, (out, c.state.get("motion_hold"))
    assert 0.0 < out.requested_speed <= path_end_limit() + 1e-9, out
    assert any(sl == MissionCore.POCKET_END_SLACK_M for _, _, sl in layer.calls)  # the width asked with the slack


def test_a_blocked_rear_arc_still_holds():
    c, _, _ = core_at_end(standing_speed(), (-0.20, 0.0))  # just behind the rear (0.17) within the reverse stop arc
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.obstacle_hold


def test_a_blocked_front_arc_still_holds():
    c, _, perm = core_at_end(standing_speed(), (1.36, 0.0))  # beyond the path end's front, within the stop arc
    assert perm.path_check.blocked is None and perm.path_check.reached_end
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.obstacle_hold


def test_no_request_checks_both_ways_on_the_arc_and_sends_nothing():
    c, layer, _ = core_at_end(standing_speed())
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.0, False, None)
    assert out.requested_speed == 0.0 and c.state.get("motion_hold") is None
    assert {d for d, _, sl in layer.calls if sl == MissionCore.POCKET_END_SLACK_M} == {1, -1}  # both, with the slack


def test_no_request_with_one_arc_blocked_holds():
    c, _, _ = core_at_end(standing_speed(), (-0.20, 0.0))
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.0, False, None)
    assert out.requested_speed == 0.0 and c.state.get("motion_hold") == "occupied"  # obstacle_hold needs a request


def test_a_stale_standing_reading_still_holds():
    """Codex U3e-1b round 3 P1: a standing reading 89 ms old (bound about 0.054) 8.1 mm before the insert end -- its
    stop does not fit the end and the tolerance: held, as before."""
    speed = standing_speed(0.089)
    assert speed.bound_mps > 0.05
    c, _, _ = core_at_end(speed, left=0.0081)
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.obstacle_hold


def test_a_slow_creep_within_the_tolerance_goes_on():
    """Moving at 5 mm/s (both directions at a 27.8 ms delay bound) 9 mm before the end, its command and output 0.005:
    the possible motion stops within the end and the tolerance -- the creep goes on, at most the path-end limit."""
    speed = reading(moved_counts=42)  # 42 counts in 20 ms: about 5 mm/s
    assert speed.directions == (1, -1) and 0.004 < speed.signed_mps < 0.006
    c, _, _ = core_at_end(speed, left=0.009)
    c.state.update(command_speed=0.005, outstanding_motion={1: 0.005})
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert not out.obstacle_hold and 0.0 < out.requested_speed <= path_end_limit(0.009) + 1e-9, out


def test_a_two_direction_reading_of_a_moving_truck_is_still_bound_by_the_path_end():
    """Codex U3e-1b plan round 2 P1: 292 counts in 30 ms (signed 0.0233, bound 0.0469, both directions at a 27.8 ms
    delay bound) with a 0.018 command and output, 8.1 mm before the insert end -- held, as before."""
    speed = reading(dt_ms=30, moved_counts=292)
    assert speed.directions == (1, -1) and speed.signed_mps > 0.02
    c, _, _ = core_at_end(speed, left=0.0081)
    c.state.update(command_speed=0.018, outstanding_motion={1: 0.018})
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.obstacle_hold


def test_outside_the_d5_stages_the_width_stays_bound_by_the_path_end():
    """Codex U3e-1b plan round 2 P2: transport is not acting -- the layer's 0 need not reach the output there."""
    c, _, _ = core_at_end(standing_speed(), phase="transport")
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.obstacle_hold


def test_a_moving_reading_is_still_bound_by_the_path_end():
    """Codex U3e-1b plan round 1 counterexample: moving at a measured 0.0392 (bound 0.060, one direction) with an
    unconfirmed 0.018 output, 11.26 mm before the insert end -- held, as before."""
    c, _, _ = core_at_end(SafetySpeed(0.0392, 0.060, (1,)))
    c.state["outstanding_motion"] = {1: 0.018}
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.obstacle_hold


def test_a_command_in_effect_beyond_the_tolerance_holds():
    c, _, _ = core_at_end(standing_speed())
    c.state["command_speed"] = 0.04  # stops about 22 mm on: past the end and the tolerance
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.obstacle_hold


def test_no_request_on_an_unexpired_output_holds():
    """Codex U3e-1b round 4 P2: a 0.030 forward command and unexpired output, nothing asked, 11.26 mm before the end --
    the loop's own output is held to the path end (no slack): a hold (the loop's wait frame, real_loop.py)."""
    c, _, _ = core_at_end(standing_speed())
    c.state.update(command_speed=0.03, outstanding_motion={1: 0.03})
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.0, False, None)
    assert out.requested_speed == 0.0 and c.state.get("motion_hold")


def test_a_forward_request_on_an_unexpired_reverse_output_holds():
    c, _, _ = core_at_end(standing_speed())  # an empty grid: the output's own bound is what holds
    c.state.update(command_speed=-0.03, outstanding_motion={-1: 0.03})
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.obstacle_hold


def test_the_distance_to_the_end_is_the_legs_end_pose_from_the_rear():
    """Codex U3e-1b rounds 4 P1, 5 P2: the end's distance is measured from the leg's end pose along its heading and
    the leg's direction (negative past it), not taken from the layer's path_left alone."""
    c, _, _ = core_at_end(standing_speed())
    slack = MissionCore.POCKET_END_SLACK_M
    assert c._end_slack(LegTo((0.0, 0.0, 0.0)), (-0.02, 0.0, 0.0)) == {"path_end_slack_m": slack,
                                                                       "path_end_ahead_m": pytest.approx(0.02)}
    assert c._end_slack(LegTo((0.0, 0.0, 0.0)), (0.005, 0.0, 0.0))["path_end_ahead_m"] == pytest.approx(-0.005)
    assert c._end_slack(LegTo((0.0, 0.0, 0.0), -1), (-0.005, 0.0, 0.0))["path_end_ahead_m"] == pytest.approx(-0.005)
    assert c._end_slack(None, (0.0, 0.0, 0.0)) == {}


def test_a_layer_path_left_older_than_the_pose_does_not_widen_the_slack():
    """Codex U3e-1b round 5 P2: the snapshot's path_left (8.395 mm, asked before this tick's progress) is longer than
    the truck's own distance to the insert end (8.100 mm). A standing width of about 0.0296 stops within 8.395 + 8 mm
    but not within 8.100 + 8: held, measured from the nearer."""
    speed = standing_speed(0.0438)
    assert 0.0294 < speed.bound_mps < 0.0298
    assert STOP.speed_for(0.008100 + 0.008 + STOP.margin_m) < speed.bound_mps < STOP.speed_for(0.008395 + 0.008 + STOP.margin_m)
    c, _, _ = core_at_end(speed, left=0.008395, leg_left=0.0081)
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.obstacle_hold
    c, _, _ = core_at_end(speed, left=0.008395)  # the same path_left with the leg agreeing: it creeps on
    assert not c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None).obstacle_hold


def test_past_the_end_the_new_command_is_zero():
    c, _, perm = core_at_end(standing_speed())
    perm.advance(0.02)  # 8.7 mm past the end
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0


# --- the transition on the real loop (Codex U3e-1b round 3 P2) -----------------------------------------------------
@pytest.mark.parametrize("slack", [MissionCore.POCKET_END_SLACK_M, 0.0])
def test_the_bypassed_s10_reaches_the_lift_at_a_27_8_ms_delay_bound(slack, monkeypatch):
    """The real loop rig with the real layer at the operating stop model (scan period 0.12 -> latency 0.52 s), the
    real encoder model's two directions (no direction evidence) and the delay bound the encoder speed adds fixed at
    27.8 ms (``_speed_delay_s``, run0's phase):
    with the slack the bypassed S10 goes approach -> insert -> lift; without it (the code before U3e-1b) it stalls short
    of an end and ends in a pocket stop."""
    from forklift_core.live import real_autonomy as ra

    from real_loop_rig import Rig
    from test_real_loop_console import ask, run_until
    from test_real_loop_d5 import submit

    monkeypatch.setattr(MissionCore, "POCKET_END_SLACK_M", slack)
    r = Rig(real_requests=True, real_layer=True, direction_evidence=False, scan_period_s=0.12)
    r.loop._speed_delay_s = lambda: 0.0278
    r.start()
    r.console_out.clear()
    reply, _ = ask(r, "check_bypass", check="d5_pocket", on=True)
    assert reply["t"] == "ack", reply
    r.tick()
    submit(r)
    run_until(r, lambda rec: (r.core.operator_open() is not None or r.permit.state == ra.LATCHED
                              or r.core.commands.status in ("cancelled", "failed", "recovery_required", "done")),
              30000)
    if slack:
        op = r.core.operator_open()
        assert op is not None and op["kind"] == "lifted", (r.core.commands.end_reason, r.permit.reason)
    else:
        assert "pocket_stop" in str(r.core.commands.end_reason) + str(r.permit.reason), (
            r.core.commands.end_reason, r.permit.reason)
