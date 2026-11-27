"""U3e-1b (docs/plans/2026-11-27-real-path-end-hold.md 4): the real truck standing just short of a path end, one tick
on real parts -- the encoder safety speed from two STATEs (``real_teleop.encoder_speed``), the obstacle layer's
``DrivePermission`` on an empty grid and a short path, ``MissionCore.permission`` with the D5 bypass check.

The standing reading's error width (two directions, about 0.021 m/s at a 27.8 ms delay bound) is above the path-end
deceleration (``max(speed_for(path_left + margin), end_creep 0.02)``, about 0.0208 m/s 11.26 mm before the end): the
operating end-to-end run stalled there (ws1 21634). On a measured stand (the loop's ``measured_stand``) in the D5
stages the width is judged by the stop arc only; a command in effect, an unconfirmed output, a moving reading (two
directions included) and every other stage stay bound by the path-end deceleration."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.live import real_teleop as rt
from forklift_core.live.real_assembly import BypassPocketCheck
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
              path_end_target=True):
        self.calls.append((direction, cap_mps, path_end_target))
        ev = t if evidence_s is None else evidence_s
        kw = {} if path_end_target else {"path_end_target": False}
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


def core_at_end(speed, *occupied, phase="insert", left=LEFT, stand=True):
    perm = DrivePermission(CONFIG)
    path = np.array([[0.0, 0.0, 0.0], [left, 0.0, 0.0]])
    perm.update(grid(*occupied), path, FOOT, BODY, current_pose=(0.0, 0.0, 0.0), direction=1)
    c, _ = permission_core(speed, phase=phase, pocket_check=BypassPocketCheck((0.0, 0.0), 1.8, 0.8, 0.0))
    layer = PermissionLayer(perm)
    c.obstacle["layer"] = layer
    c.state["measured_stand"] = stand  # the real loop's measured stand (outputs settled, counts still)
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
    assert any(not pet for _, _, pet in layer.calls)  # the width was asked on the stop arc


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
    assert {d for d, _, pet in layer.calls if pet is False} == {1, -1}  # both widths asked on the stop arc


def test_no_request_with_one_arc_blocked_holds():
    c, _, _ = core_at_end(standing_speed(), (-0.20, 0.0))
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.0, False, None)
    assert out.requested_speed == 0.0 and c.state.get("motion_hold") == "occupied"  # obstacle_hold needs a request


def test_without_a_measured_stand_the_width_stays_bound_by_the_path_end():
    c, _, _ = core_at_end(standing_speed(), stand=False)
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.obstacle_hold


def test_a_two_direction_reading_of_a_moving_truck_is_still_bound_by_the_path_end():
    """Codex U3e-1b plan round 2 P1: 292 counts in 30 ms (signed 0.0233, bound 0.0469, both directions at a 27.8 ms
    delay bound) with a 0.018 command and output, 8.1 mm before the insert end -- held, as before. (The loop's
    measured stand is false while counts move; the test sets it true to pin the reading's own part too.)"""
    speed = reading(dt_ms=30, moved_counts=292)
    assert speed.directions == (1, -1) and speed.signed_mps > 0.02
    c, _, _ = core_at_end(speed, left=0.0081, stand=False)
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


def test_a_command_in_effect_above_the_path_end_holds():
    c, _, _ = core_at_end(standing_speed())
    c.state["command_speed"] = 0.03
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.obstacle_hold


def test_past_the_end_the_new_command_is_zero():
    c, _, perm = core_at_end(standing_speed())
    perm.advance(0.02)  # 8.7 mm past the end
    out = c.permission(T, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0
