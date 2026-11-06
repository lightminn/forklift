"""The unconfirmed position state machine (docs/plans/2026-11-02-unconfirmed-runner.md 2.2): checking from the first
live tick, the start result, the straight-reverse strokes (net displacement, D_STROKE and D_TOTAL in the brake), the
re-match request and its results, the SLAM agreement on every promotion, and the monitor after confirmation -- every
comparison on the base link and the rear axle both."""

import math

import pytest

from forklift_core.localization import pose_check as P
from forklift_core.localization.slam_pose import compose, invert

BOUNDS = P.Bounds(odom=(0.02, 0.02, 0.005, 0.01), e_still=(0.03, 0.01), e_slam=(0.05, 0.025))
I = (0.0, 0.0, 0.0)


def _pc():
    return P.PoseCheck(BOUNDS, rear_offset_m=0.34, stop_m=0.068)


def _back(o, m):
    """``o`` moved ``m`` straight back along its heading."""
    return compose(o, (-m, 0.0, 0.0))


def _tick(pc, o, C=I, **kw):
    kw.setdefault("standing", True)
    kw.setdefault("deadman", False)
    return pc.update(0.0, o, C, **kw)


def _unconfirmed(o0=I):
    pc = _pc()
    pc.start(0.0, confirmed=False, r_scan=o0, o_scan=o0, C=I, o_now=o0, reasons=("competitor",))
    return pc


def _drive(pc, o, metres, step=0.01):
    """Reverse with the deadman held until the stroke closes or ``metres`` are covered; returns the odometry."""
    moved = 0.0
    while moved < metres - 1e-9:
        view = _tick(pc, o, standing=False, deadman=True)
        if not view.drive_ready:
            break
        o = _back(o, step)
        moved += step
    return o


def _stand(pc, o):
    _tick(pc, o, standing=False, deadman=False)
    return _tick(pc, o, standing=True, deadman=False)


def test_it_is_checking_from_the_first_tick_and_nothing_opens():
    pc = _pc()
    view = _tick(pc, I)
    assert pc.status == "checking" and not view.drive_ready and not view.want_rematch


def test_a_confirmed_start_needs_the_slam_agreement_on_the_base_and_on_the_rear_axle():
    pc = _pc()
    pc.start(0.0, confirmed=True, r_scan=(1.0, -0.02, 0.0), o_scan=I, C=(1.0, 0.0, 0.0), o_now=I, reasons=())
    assert pc.status == "confirmed"
    pc = _pc()  # Codex draft 2 P1: base 0.049 m, 0.024 rad apart -- 0.057 m at the rear axle, outside 0.05 m
    pc.start(0.0, confirmed=True, r_scan=(1.0, -0.090, 0.0), o_scan=I, C=(1.0, -0.139, 0.024), o_now=I, reasons=())
    assert pc.status == "ended:pose_slam_disagree"


def test_c_int_comes_from_the_evidence_and_its_own_odometry():
    # Codex draft 1 P2-5: confirmed 0.3 m after the scan, the monitor sees no gap
    pc = _pc()
    o_scan, o_now = (2.0, 0.0, 0.0), (1.7, 0.0, 0.0)
    r_scan = (5.0, 1.0, 0.0)
    C = compose(r_scan, invert(o_scan))
    pc.start(0.0, confirmed=True, r_scan=r_scan, o_scan=o_scan, C=C, o_now=o_now, reasons=())
    assert pc.status == "confirmed"
    assert _tick(pc, o_now, C).delta_int == pytest.approx((0.0, 0.0), abs=1e-12)


def test_an_unconfirmed_start_opens_only_the_straight_reverse_stroke():
    pc = _unconfirmed()
    view = _tick(pc, I)
    assert pc.status == "unconfirmed" and view.drive_ready and view.stroke == "ready"


def test_a_stroke_closes_short_of_d_stroke_by_the_stopping_distance():
    pc = _unconfirmed()
    o = _drive(pc, I, 2.0)
    s = math.hypot(o[0], o[1])
    assert abs(s - (P.D_STROKE - 0.068)) <= 0.011  # closed at the first tick within the stopping distance
    view = _tick(pc, o, standing=False, deadman=True)
    assert not view.drive_ready and view.stroke == "closed" and view.reason == "pose_unconfirmed:stroke"
    view = _tick(pc, o, standing=True, deadman=True)  # standing but still held: not yet
    assert view.stroke == "closed"
    assert _stand(pc, o).stroke == "ready"


def test_partial_strokes_never_take_the_total_past_d_total():
    # Codex draft 1 P2-3: four strokes of 0.7 m; the fifth's brake counts the total too
    pc = _unconfirmed()
    o = I
    for _ in range(5):
        o = _drive(pc, o, 0.7)
        _stand(pc, o)
    used = math.hypot(o[0], o[1])
    assert used <= P.D_TOTAL - 0.068 + 0.011  # one 0.01 m step past the closing tick at most
    assert _tick(pc, o).stroke == "spent" and not _tick(pc, o).drive_ready
    assert _tick(pc, o).reason == "pose_unconfirmed:total"


def test_standing_wheel_noise_does_not_use_up_the_stroke():
    # agy 9: the stroke is the net displacement, not the path length
    pc = _unconfirmed()
    o = I
    for k in range(2000):
        jitter = (0.0005 if k % 2 else -0.0005, 0.0, 0.0)
        _tick(pc, compose(o, jitter), standing=True, deadman=True)
    view = _tick(pc, o, standing=False, deadman=True)
    assert view.drive_ready and view.stroke_left_m == pytest.approx(P.D_STROKE, abs=1e-3)


def test_the_rematch_is_asked_once_per_stand_after_d_min():
    pc = _unconfirmed()
    o = _drive(pc, I, 0.3)
    assert not _stand(pc, o).want_rematch  # 0.3 m < D_MIN
    o = _drive(pc, o, 0.3)
    view = _stand(pc, o)
    assert view.want_rematch
    pc.begin_rematch(0.0, epoch=1, control_gen=4)
    assert pc.status == "checking" and not _tick(pc, o).drive_ready
    pc.rematch_result(0.0, "unconfirmed", None, o, o_now=o, C=I, epoch=1, control_gen=4)
    assert pc.status == "unconfirmed" and not _tick(pc, o).want_rematch  # not again at this stand
    o = _drive(pc, o, 0.1)
    assert _stand(pc, o).want_rematch  # the next stand


@pytest.mark.parametrize("outcome, status", [("twin_preferred", "ended:pose_twin_preferred"),
                                             ("out_of_set", "ended:pose_out_of_set"),
                                             ("too_short", "unconfirmed"), ("unconfirmed", "unconfirmed")])
def test_the_rematch_outcomes(outcome, status):
    pc = _unconfirmed()
    o = _drive(pc, I, 0.6)
    _stand(pc, o)
    pc.begin_rematch(0.0, epoch=1, control_gen=4)
    pc.rematch_result(0.0, outcome, (9.0, 9.0, 0.0), o, o_now=o, C=I, epoch=1, control_gen=4)
    assert pc.status == status


def test_a_rematch_confirm_promotes_through_the_slam_agreement():
    pc = _unconfirmed()
    o = _drive(pc, I, 0.6)
    _stand(pc, o)
    pc.begin_rematch(0.0, epoch=1, control_gen=4)
    g = compose((3.0, 2.0, 0.1), o)
    C = (3.0, 2.0, 0.1)
    pc.rematch_result(0.0, "confirm", g, o, o_now=o, C=C, epoch=1, control_gen=4)
    assert pc.status == "confirmed"
    pc = _unconfirmed()
    o = _drive(pc, I, 0.6)
    _stand(pc, o)
    pc.begin_rematch(0.0, epoch=1, control_gen=4)
    pc.rematch_result(0.0, "confirm", g, o, o_now=o, C=(3.2, 2.0, 0.1), epoch=1, control_gen=4)
    assert pc.status == "ended:pose_slam_disagree"


def test_an_unconfirmed_rematch_survives_a_control_boundary_but_not_a_move_or_a_new_epoch():
    # agy 2: letting go of the deadman can preempt and raise control_gen; the result stands, motion does not
    pc = _unconfirmed()
    o = _drive(pc, I, 0.6)
    _stand(pc, o)
    pc.begin_rematch(0.0, epoch=1, control_gen=4)
    pc.rematch_result(0.0, "twin_preferred", None, o, o_now=o, C=I, epoch=1, control_gen=5)
    assert pc.status == "ended:pose_twin_preferred"
    for kw in ({"o_now": _back(o, 0.003), "epoch": 1}, {"o_now": o, "epoch": 2}):
        pc = _unconfirmed()
        o = _drive(pc, I, 0.6)
        _stand(pc, o)
        pc.begin_rematch(0.0, epoch=1, control_gen=4)
        pc.rematch_result(0.0, "twin_preferred", None, o, C=I, control_gen=4, **kw)
        assert pc.status == "unconfirmed" and pc.events[-1]["kind"] == "rematch_discarded"


def _confirmed(C=I, o=I):
    pc = _pc()
    pc.start(0.0, confirmed=True, r_scan=compose(C, o), o_scan=o, C=C, o_now=o, reasons=())
    assert pc.status == "confirmed"
    return pc


def test_the_monitor_revokes_on_the_rear_axle_too():
    # Codex draft 2 P1: the base inside the limit, the rear axle past it
    pc = _confirmed()
    C = compose((0.0, -0.150, 0.049), I)
    view = _tick(pc, I, C, standing=False)
    assert pc.status == "ended:pose_revoked:window", view


def _walk(pc, metres, per_m, step=0.1, t0=0.0, dt=0.1):
    """Drive forward with the correction moving ``per_m`` sideways per metre; returns (o, C, t)."""
    o, C, t = I, I, t0
    for k in range(round(metres / step)):
        o, C, t = (o[0] + step, 0.0, 0.0), (0.0, C[1] + per_m * step, 0.0), t + dt
        pc.update(t, o, C, standing=False, deadman=False)
    return o, C, t


def test_a_correction_drifting_over_many_metres_is_not_revoked():
    # the revision plan 9: the drift a correction absorbs passed the cumulative gate in every correct mission
    pc = _confirmed()
    _walk(pc, 40.0, 0.05)  # 2 m of 0.05 m/m: 0.10 m in the window, 2.0 m in all
    assert pc.status == "confirmed"


def test_a_jump_inside_the_window_is_revoked_whatever_the_path_before():
    pc = _confirmed()
    o, C, t = _walk(pc, 40.0, 0.0)
    pc.update(t + 0.1, o, (0.0, 0.16, 0.0), standing=False, deadman=False)
    assert pc.status == "ended:pose_revoked:window"
    assert [e for e in pc.events if e["kind"] == "revoke"][0]["gate"] == "window"


def test_a_jump_right_after_the_confirmation_is_seen_with_less_than_the_window_behind():
    pc = _confirmed()
    pc.update(0.1, (0.3, 0.0, 0.0), I, standing=False, deadman=False)
    pc.update(0.2, (0.6, 0.0, 0.0), (0.0, 0.2, 0.0), standing=False, deadman=False)
    assert pc.status == "ended:pose_revoked:window"


def test_the_time_cap_lets_a_long_stand_go_on():
    # a stand does not grow the path: without the cap a correction turning 0.002 rad/s ends a correct session at 51 s
    bounds = P.Bounds(odom=(0.02, 0.02, 0.005, 0.01), e_still=(0.20, 0.05), e_slam=(0.05, 0.025))
    pc = P.PoseCheck(bounds, rear_offset_m=0.34, stop_m=0.068)
    pc.start(0.0, confirmed=True, r_scan=I, o_scan=I, C=I, o_now=I)
    for k in range(1, 12000):  # 120 s at 0.002 rad/s: 0.24 rad in all, 0.02 rad in the cap; a new stand each 10 s
        C = (0.0, 0.0, 0.00002 * k)
        pc.update(k * 0.01, I, C, standing=k % 1000 != 0, deadman=False)
    assert pc.status == "confirmed"


def test_the_stand_reference_is_taken_again_at_a_hold_release():
    # the revision plan 9.4: the release's own move is not the stand's
    pc = _confirmed()
    _tick(pc, I, I, standing=False)
    _tick(pc, I, I, standing=True, holding=True)
    pc.update(1.0, I, (0.0, 0.0, 0.04), standing=True, deadman=False, holding=False)  # released: 0.04 rad, over 0.01
    pc.update(1.1, I, (0.0, 0.0, 0.041), standing=True, deadman=False, holding=False)
    assert pc.status == "confirmed"
    pc.update(1.5, I, (0.0, 0.0, 0.041), standing=True, deadman=False, holding=False)
    pc.update(1.6, I, (0.0, 0.0, 0.056), standing=True, deadman=False, holding=False)  # past the release: the stand's
    assert pc.status == "ended:pose_revoked:still"


def test_the_stand_gate_is_not_taken_again_without_a_release():
    pc = _confirmed()
    _tick(pc, I, I, standing=False)
    pc.update(1.0, I, I, standing=True, deadman=False)
    pc.update(1.1, I, (0.0, 0.0, 0.04), standing=True, deadman=False)
    assert pc.status == "ended:pose_revoked:still"


def test_the_window_limit_is_a_development_value():
    with pytest.raises(ValueError):
        P.Bounds(odom=(0.0, 0.0, 0.0, 0.0), e_still=(0.03, 0.01), e_slam=(0.05, 0.025), e_window=(0.1,))
    assert P.Bounds(odom=(0.0, 0.0, 0.0, 0.0), e_still=(0.03, 0.01), e_slam=(0.05, 0.025),
                    e_window=(0.2, 0.2)).e_window == (0.2, 0.2)


def test_the_still_monitor_and_its_reference_per_stand():
    pc = _confirmed()
    _tick(pc, I, I, standing=False)
    _tick(pc, I, (0.02, 0.0, 0.0), standing=True)  # C_still taken at the stand's first tick
    _tick(pc, I, (0.04, 0.0, 0.0), standing=True)  # 0.02 m from C_still: inside E_STILL 0.03
    assert pc.status == "confirmed"
    _tick(pc, I, (0.06, 0.0, 0.0), standing=True)  # 0.04 m: out
    assert pc.status == "ended:pose_revoked:still"


def test_the_confirmed_rematch_waits_for_a_stand_without_a_mission_or_a_near_section():
    pc = _confirmed()
    C = (0.12, 0.0, 0.0)
    assert not _tick(pc, I, C, standing=True, mission_active=True).want_rematch
    assert not _tick(pc, I, C, standing=True, near=True).want_rematch
    assert _tick(pc, I, C, standing=True).want_rematch


@pytest.mark.parametrize("g, outcome, status, moved", [
    ((0.12, 0.0, 0.0), "confirm", "confirmed", True),  # evidence agreeing with c: a new C_int
    ((0.30, 0.0, 0.0), "confirm", "ended:pose_revoked:rematch", False),  # evidence against c
    ((0.30, 0.0, 0.0), "none", "confirmed", False),  # no evidence: C_int kept
])
def test_the_confirmed_rematch_contract(g, outcome, status, moved):
    pc = _confirmed()
    C = (0.12, 0.0, 0.0)
    _tick(pc, I, C, standing=True)
    pc.begin_rematch(0.0, epoch=1, control_gen=4)
    pc.rematch_result(0.0, outcome, g, I, o_now=I, C=C, epoch=1, control_gen=4)
    assert pc.status == status
    if status == "confirmed":
        assert (_tick(pc, I, C).delta_int[0] < 1e-9) is moved


def test_a_confirmed_rematch_is_discarded_on_a_control_boundary():
    pc = _confirmed()
    C = (0.12, 0.0, 0.0)
    _tick(pc, I, C, standing=True)
    pc.begin_rematch(0.0, epoch=1, control_gen=4)
    pc.rematch_result(0.0, "confirm", (0.30, 0.0, 0.0), I, o_now=I, C=C, epoch=1, control_gen=5)
    assert pc.status == "confirmed" and pc.events[-1]["kind"] == "rematch_discarded"


def test_an_ended_check_stays_ended():
    pc = _unconfirmed()
    pc.end(0.0, "pose_rear_blocked")
    assert pc.status == "ended:pose_rear_blocked"
    _tick(pc, I)
    pc.start(0.0, confirmed=True, r_scan=I, o_scan=I, C=I, o_now=I, reasons=())
    assert pc.status == "ended:pose_rear_blocked" and not _tick(pc, I).drive_ready


def test_the_odometry_bound_grows_with_d():
    assert BOUNDS.dp(0.0) == pytest.approx(0.02) and BOUNDS.dp(1.0) == pytest.approx(0.04)
    assert BOUNDS.dth(2.0) == pytest.approx(0.025)
    with pytest.raises(ValueError):
        P.Bounds(odom=(0.02, -0.1, 0.0, 0.0), e_still=(0.03, 0.01), e_slam=(0.05, 0.025))


# --- Codex piece 2a review ---
def test_a_confirmed_rematch_locks_the_controls_while_the_monitor_goes_on():
    pc = _confirmed()
    C = (0.12, 0.0, 0.0)
    _tick(pc, I, C, standing=True)
    pc.begin_rematch(0.0, epoch=1, control_gen=4)
    view = _tick(pc, I, C, standing=True)
    assert view.status == "checking" and view.reason == "pose_checking" and not view.drive_ready
    _tick(pc, I, (0.29, 0.0, 0.0), standing=True)  # the monitor still runs: past the window's limit it ends
    assert pc.status == "ended:pose_revoked:window"


def test_a_stroke_never_passes_d_stroke_with_its_stopping_distance():
    # Codex piece 2a P2: closing after the threshold left the last sample's move; the brake counts one tick's travel
    pc = P.PoseCheck(BOUNDS, rear_offset_m=0.34, stop_m=0.0683333333, tick_m=0.01)
    pc.start(0.0, confirmed=False, r_scan=I, o_scan=I, C=I, o_now=I)
    o = I
    for _ in range(3):
        o = _drive(pc, o, 2.0)
        o = _back(o, 0.0683333333)  # the stop after the close
        _stand(pc, o)
    assert pc.used_m <= P.D_TOTAL + 1e-9
    assert all(e["s_m"] + 0.0683333333 <= P.D_STROKE + 1e-9 for e in pc.events if e["kind"] == "stroke_close")


def test_a_rematch_result_is_discarded_when_the_rear_axle_moved():
    # Codex piece 2a P2: base 0.0019 m, rear axle 0.0025 m
    pc = _unconfirmed()
    o = _drive(pc, I, 0.6)
    _stand(pc, o)
    pc.begin_rematch(0.0, epoch=1, control_gen=4)
    pc.rematch_result(0.0, "twin_preferred", None, o, o_now=compose(o, (0.0, -0.0019, 0.0019)), C=I, epoch=1,
                      control_gen=4)
    assert pc.status == "unconfirmed" and pc.events[-1]["kind"] == "rematch_discarded"


def test_the_stroke_brake_holds_on_the_quantized_encoder_odometry():
    # Codex piece 2a re-review P2: team_spec_quantized moves up to 5 counts (0.96 mm) in a tick at 0.10 m/s, more than
    # 0.10 / 120; the brake allows for the larger of the given tick bound and the largest step seen
    import numpy as np

    from forklift_core.localization.encoders import (
        MT6701_COUNTS_PER_REV, TeamEncoderModel, encoder_model_spec, sensor_phases)
    from forklift_core.localization.slam_pose import IncrementalWheelOdometry
    from forklift_core.localization.wheel_odometry import AckermannOdometryGeometry

    stop_m, r, dt, v = 0.0683333333, 0.125, 1 / 120, 0.10
    quantum = 2 * math.pi / MT6701_COUNTS_PER_REV

    def closed_plus_stop(a0, tick_m):
        model = TeamEncoderModel(encoder_model_spec("team_spec_quantized"), sensor_phases(3))
        odo = IncrementalWheelOdometry(AckermannOdometryGeometry(0.66, 0.53, 0.125))
        pc = P.PoseCheck(BOUNDS, rear_offset_m=0.34, stop_m=stop_m, tick_m=tick_m)
        pc.start(0.0, confirmed=False, r_scan=I, o_scan=I, C=I, o_now=I)
        angle, o = a0, I
        model.read(np.array([angle, angle]), np.array([0.0, 0.0]))
        for k in range(1, 3000):
            if not pc.update(k * dt, o, I, standing=False, deadman=True).drive_ready:
                break
            angle -= v * dt / r
            inc, steer, _ = model.read(np.array([angle, angle]), np.array([0.0, 0.0]))
            o = odo.update_increments(k * dt, inc, steer)
        return next(e["s_m"] for e in pc.events if e["kind"] == "stroke_close") + stop_m

    starts = [quantum * j / 10 for j in range(40)]  # where in a count the wheel starts
    # V_REV_U x dt alone: the plan's first figure; Codex's 1.0000349 m at a start 3.6 counts in
    assert max(closed_plus_stop(a, v * dt) for a in starts) > P.D_STROKE
    # one count more (P.tick_bound): never past D_STROKE
    assert max(closed_plus_stop(a, P.tick_bound(v, dt, r, MT6701_COUNTS_PER_REV)) for a in starts) <= P.D_STROKE
