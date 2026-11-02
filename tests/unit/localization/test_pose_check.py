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
    # Codex draft 2 P1: base 0.199 m (inside 0.20 m) is 0.216 m at the rear axle
    pc = _confirmed()
    C = compose((0.0, -0.199, 0.049), I)
    view = _tick(pc, I, C, standing=False)
    assert pc.status == "ended:pose_revoked:int", view


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
    _tick(pc, I, (0.25, 0.0, 0.0), standing=True)  # the monitor still runs: past 0.20 m it ends
    assert pc.status == "ended:pose_revoked:int"


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
