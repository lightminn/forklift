"""S1–S6 bring-up calculations (docs/design/2026-11-12-bringup-host-tools.md 4)."""

import pytest

from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag
from forklift_core.plant.bringup import (
    Sample,
    count_settle,
    rolling_distance_m,
    step_metrics,
    wheel_cpr,
    wheel_rate_cap,
)


def st(t_us, l=0, r=0, rate_l=0.0, rate_r=0.0, boot=7, faults=FaultFlag(0)):
    s = McuState(mcu_time_us=t_us, boot_id=boot, arm_gen=1, last_rx_seq=0, mode=Mode.DISARMED,
                 reject_reason=RejectReason.NONE, fault_flags=faults, wheel_count_left=l, wheel_count_right=r,
                 wheel_rate_left_rad_s=rate_l, wheel_rate_right_rad_s=rate_r, steering_rad=0.0, lift_m=0.0,
                 current_ma=(0, 0, 0, 0), duty_e4=(0, 0, 0, 0), driver_status=(0, 0, 0, 0), rx_errors=0,
                 cycle_overruns=0, flags=StateFlag(0))
    return Sample(host_ns=t_us * 1000 + 5, state=s)


# --- wheel CPR -------------------------------------------------------------------------------------------------------

def test_cpr_sums_signed_modular_differences_across_the_wrap():
    cpr = 327_680
    start = 2**31 - 1000  # the left count wraps past +2^31 during the turn
    samples = [st(0, l=start, r=0)]
    for k in range(1, 11):  # ten turns, one sample per turn (missing samples between are fine: counts accumulate)
        samples.append(st(k * 100_000, l=(start + k * cpr + 2**31) % 2**32 - 2**31, r=-k * cpr))
    res = wheel_cpr(samples, (10, 10), (+1, -1), motor_counts_confirmed=True)
    assert res.valid and res.cpr == (cpr, cpr) and res.gear == (80.0, 80.0) and res.difference == 0


def test_cpr_without_the_motor_encoder_confirmed_gives_no_gear():
    res = wheel_cpr([st(0), st(100, l=4096 * 80)], (1, 0), (+1, +1))
    assert res.valid and res.cpr[0] == 4096 * 80 and res.cpr[1] is None and res.gear == (None, None)


@pytest.mark.parametrize("bad,reason", [
    ([st(0), st(100, l=500, boot=8)], "boot_changed"),
    ([st(100), st(50, l=500)], "time_back"),
    ([st(0), st(100, l=500, faults=FaultFlag.ENCODER_JUMP_L)], "continuity_cut"),
    ([st(0), st(100, l=500, faults=FaultFlag.ENCODER_GAP)], "continuity_cut"),
    ([st(0), st(100, l=-500)], "direction_mismatch"),
    ([st(0), st(100, l=0)], "direction_mismatch"),
    ([st(0)], "too_few_samples"),
])
def test_cpr_is_invalid_when_the_count_cannot_be_trusted(bad, reason):
    res = wheel_cpr(bad, (1, 0), (+1, +1))
    assert not res.valid and res.reason == reason


def test_a_jump_on_the_wheel_not_turned_does_not_void_the_other():
    res = wheel_cpr([st(0), st(100, l=1000, faults=FaultFlag.ENCODER_JUMP_R)], (1, 0), (+1, +1))
    assert res.valid and res.cpr[0] == 1000


# --- step response ---------------------------------------------------------------------------------------------------

def first_order(target, initial=0.0, tau=0.1, dt_us=10_000, n=300, step_at=0):
    import math
    out = []
    for k in range(n):
        t = k * dt_us
        y = initial if t < step_at else target + (initial - target) * math.exp(-(t - step_at) / 1e6 / tau)
        out.append(st(t, rate_l=y))
    return out


def test_a_first_order_step_rises_settles_and_does_not_overshoot():
    m = step_metrics(first_order(2.0), 0, 0, 0.0, 2.0)
    lo, hi = m.rise_s
    assert lo <= 0.2197 <= hi + 0.01 and (hi - lo) <= 0.02  # tau ln 9 = 0.2197 s
    assert m.overshoot_pct == 0.0 and not m.unsettled and m.settle_s[1] <= 0.31


def test_a_reverse_step_from_a_residual_rate_is_normalised_the_same():
    m = step_metrics(first_order(-1.5, initial=0.5), 0, 0, 0.5, -1.5)
    assert m.rise_s is not None and not m.unsettled


def test_a_zero_step_is_refused_and_an_unsettled_one_reported():
    with pytest.raises(ValueError):
        step_metrics(first_order(1.0), 0, 0, 1.0, 1.0)
    m = step_metrics(first_order(1.0, tau=5.0), 0, 0, 0.0, 1.0)
    assert m.unsettled and m.settle_s is None


def test_a_gap_makes_the_crossing_an_interval():
    samples = [s for s in first_order(2.0) if not (150_000 <= s.state.mcu_time_us <= 300_000)]
    m = step_metrics(samples, 0, 0, 0.0, 2.0)
    assert m.gaps and m.rise_s[1] - m.rise_s[0] >= 0.1


# --- count settle ----------------------------------------------------------------------------------------------------

def test_the_counts_settle_once_still_for_the_hold():
    samples = [st(k * 10_000, l=100 * min(k, 20), r=-50 * min(k, 20)) for k in range(80)]
    s = count_settle(samples)
    assert s.mcu_us == 200_000 and s.reason is None


def test_a_still_run_shorter_than_the_hold_is_undetermined():
    samples = [st(k * 10_000, l=100 * min(k, 70)) for k in range(80)]
    assert count_settle(samples).reason == "undetermined"


def test_a_boot_change_breaks_the_observation():
    samples = [st(k * 10_000) for k in range(10)] + [st(100_000 + k * 10_000, boot=9) for k in range(40)]
    assert count_settle(samples).reason == "observation_broken"


def test_a_continuity_cut_is_never_a_settle():
    samples = [st(k * 10_000, faults=FaultFlag.ENCODER_JUMP_R) for k in range(60)]
    s = count_settle(samples)
    assert s.mcu_us is None and s.reason == "undetermined"


def test_rolling_distance_and_the_rate_cap():
    import math
    assert rolling_distance_m(327_680, 327_680, 0.125) == pytest.approx(2 * math.pi * 0.125)
    assert wheel_rate_cap(8.0, 0.125) == pytest.approx(1.6) and wheel_rate_cap(8.0, None) == 1.0
    assert wheel_rate_cap(0.5, 0.125) == 0.5


def test_a_settle_is_never_held_across_a_gap_nor_on_a_stuck_encoder():
    gap = [st(0), st(1_000_000)]  # two equal counts a second apart: not a 0.3 s hold
    assert count_settle(gap).reason == "undetermined"
    stuck = [st(k * 10_000, faults=FaultFlag.ENCODER_STUCK_L) for k in range(60)]
    assert count_settle(stuck).reason == "undetermined"


def test_a_step_settle_is_not_held_across_a_gap():
    samples = [st(k * 10_000, rate_l=1.0) for k in range(5)] + [st(1_100_000, rate_l=1.0)]
    m = step_metrics(samples, 0, 0, 0.0, 1.0, window_s=2.0)
    assert m.unsettled


def test_a_response_already_under_way_at_the_first_accepting_state_keeps_its_crossing():
    samples = [st(0, rate_l=0.0)] + [st(10_000 + k * 10_000, rate_l=min(1.0, 0.2 + 0.1 * k)) for k in range(40)]
    m = step_metrics(samples, 0, 10_000, 0.0, 1.0)  # the first accepting STATE already reads 20 %
    assert m.rise_s is not None and m.rise_s[1] - m.rise_s[0] >= 0.01  # the 10 % crossing is an interval before it
