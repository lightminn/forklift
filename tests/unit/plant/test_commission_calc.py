"""S3 commissioning result calculations (docs/design/2026-11-12-bringup-commission-tool.md 4): synthetic STATE
samples, no link."""

import math

import pytest

from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag
from forklift_core.plant.bringup import Sample
from forklift_core.plant.commission import commission_metrics, sign_verdict

BOOT = 7


def st(t_ms, *, counts=(0, 0), rates=(0.0, 0.0), steer=0.0, lift=0.0, duty=(0, 0, 0, 0), cur=(0, 0, 0, 0),
       faults=FaultFlag(0), boot=BOOT, rx=0):
    return Sample(t_ms * 1_000_000, McuState(t_ms * 1000, boot, 1, rx, Mode.ARMED, RejectReason.NONE, faults,
                                             counts[0], counts[1], rates[0], rates[1], steer, lift, cur, duty,
                                             (0xC0,) * 4, 0, 0, StateFlag.COMMISSION_BUILD))


def wheel_run(direction=1, sign_e4=1, n_on=10, faults_at=None):
    """Samples every 10 ms: 2 before, ``n_on`` driven on axis 0, 3 coasting. Counts advance ``direction`` x 10."""
    out, c = [], 0
    for i in range(2):
        out.append(st(i * 10))
    for i in range(n_on):
        c += 10 * direction
        f = faults_at[1] if faults_at and i >= faults_at[0] else FaultFlag(0)
        out.append(st(20 + i * 10, counts=(c, 0), rates=(direction * (1.0 + 0.1 * i), 0.0),
                      duty=(1000 * sign_e4, 0, 0, 0), cur=(300 + i, 0, 5, 0), faults=f))
    for i in range(3):
        out.append(st(20 + (n_on + i) * 10, counts=(c, 0)))
    return out


def test_wheel_run_sign_rate_current_and_driven_time():
    s = wheel_run()
    m = commission_metrics(s, axis=0, duty_e4=1000, on_index=2, off_index=12)
    assert m.valid and m.measured_sign == "pos" and m.measured_delta == 100
    assert m.duty_sign_in_build == 1 and m.applied_max_e4 == 1000
    assert m.driven_ms_min == pytest.approx(90.0) and m.driven_ms_max == pytest.approx(110.0)
    # rate: the back half of the driven samples (i = 5..9 -> 1.5..1.9)
    assert m.rate_mean_rad_s == pytest.approx(1.7) and m.rate_peak_rad_s == pytest.approx(1.9)
    assert m.current_peak_ma == 309 and m.current_mean_ma == pytest.approx(304.5) and m.others_peak_ma == 5
    assert m.value is None


def test_reported_duty_against_the_command_gives_the_build_duty_sign():
    m = commission_metrics(wheel_run(sign_e4=-1), axis=0, duty_e4=1000, on_index=2, off_index=12)
    assert m.duty_sign_in_build == -1
    m = commission_metrics(wheel_run(sign_e4=1), axis=0, duty_e4=-1000, on_index=2, off_index=12)
    assert m.duty_sign_in_build == -1


def test_motion_under_the_noise_floor_is_none():
    s = [st(i * 10, counts=(i % 2 * 3, 0), duty=(1000, 0, 0, 0)) for i in range(12)]
    m = commission_metrics(s, axis=0, duty_e4=1000, on_index=2, off_index=10)
    assert m.measured_sign == "none"


def test_the_delta_starts_from_the_last_sample_before_acceptance():
    s = wheel_run()
    m = commission_metrics(s, axis=0, duty_e4=1000, on_index=3, off_index=12)
    assert m.measured_delta == 90  # from sample 2 (count 10, the last before acceptance) to the end (100)


def test_a_boot_change_invalidates():
    s = wheel_run()
    s[8] = Sample(s[8].host_ns, McuState(**{**s[8].state.__dict__, "boot_id": BOOT + 1}))
    m = commission_metrics(s, axis=0, duty_e4=1000, on_index=2, off_index=12)
    assert not m.valid and m.reason == "boot_changed"


def test_a_continuity_bit_on_the_wheel_invalidates_the_count():
    s = wheel_run(faults_at=(4, FaultFlag.ENCODER_JUMP_L))
    m = commission_metrics(s, axis=0, duty_e4=1000, on_index=2, off_index=12)
    assert not m.valid and m.reason == "encoder_continuity"


def test_new_encoder_bits_are_listed_with_their_first_time_and_bits_at_start_apart():
    s = wheel_run(faults_at=(3, FaultFlag.ENCODER_POLARITY_L))
    s[0] = Sample(s[0].host_ns, McuState(**{**s[0].state.__dict__, "fault_flags": FaultFlag.ENCODER_STUCK_R}))
    s[1] = Sample(s[1].host_ns, McuState(**{**s[1].state.__dict__, "fault_flags": FaultFlag.ENCODER_STUCK_R}))
    m = commission_metrics(s, axis=0, duty_e4=1000, on_index=2, off_index=12)
    assert m.encoder_bits_at_start == ["ENCODER_STUCK_R"]
    assert m.encoder_bits_new == [("ENCODER_POLARITY_L", 50_000)]


def test_steering_run_reports_the_value_track():
    s = [st(0, steer=0.10), st(10, steer=0.10)]
    s += [st(20 + 10 * i, steer=0.10 + 0.01 * (i + 1), duty=(0, 0, 800, 0), cur=(0, 0, 200, 0)) for i in range(5)]
    s += [st(70, steer=0.15), st(80, steer=0.15)]
    m = commission_metrics(s, axis=2, duty_e4=800, on_index=2, off_index=7)
    assert m.valid and m.measured_sign == "pos" and m.measured_delta == pytest.approx(0.05)
    assert m.value == pytest.approx({"start": 0.10, "end": 0.15, "min": 0.10, "max": 0.15})
    assert m.rate_mean_rad_s is None


def test_an_invalid_axis_value_invalidates():
    s = [st(0, steer=0.1), st(10, steer=0.1), st(20, steer=float("nan"), duty=(0, 0, 800, 0)), st(30, steer=0.12)]
    m = commission_metrics(s, axis=2, duty_e4=800, on_index=2, off_index=3)
    assert not m.valid and m.reason == "value_invalid"


def test_no_acceptance_is_not_a_measurement():
    m = commission_metrics(wheel_run(), axis=0, duty_e4=1000, on_index=None, off_index=None)
    assert not m.valid and m.reason == "not_accepted"


@pytest.mark.parametrize("cmd,measured,observed,duty_ok,sensor_ok,note", [
    (1000, "pos", "pos", True, True, None),
    (1000, "neg", "neg", False, True, None),  # wired backwards, sensor right
    (1000, "neg", "pos", True, False, None),  # turns right, sensor reversed
    (-1000, "neg", "neg", True, True, None),
    (-1000, "pos", "neg", True, False, None),
    (1000, "pos", "unknown", None, None, "no_observation"),
    (1000, "none", "pos", None, None, "inconsistent"),
    (1000, "pos", "none", None, None, "inconsistent"),
    (1000, "none", "none", None, None, "insufficient_motion"),
])
def test_sign_verdict(cmd, measured, observed, duty_ok, sensor_ok, note):
    v = sign_verdict(cmd, measured, observed, duty_sign_in_build=1)
    assert (v["duty_sign_ok"], v["sensor_sign_ok"], v["note"]) == (duty_ok, sensor_ok, note)
    if duty_ok is not None:
        assert v["suggested_duty_sign"] == (1 if duty_ok else -1)


def test_sign_verdict_suggests_against_the_build_value():
    v = sign_verdict(1000, "neg", "neg", duty_sign_in_build=-1)
    assert v["suggested_duty_sign"] == 1
    v = sign_verdict(1000, "neg", "neg", duty_sign_in_build=None)  # the output never left 0
    assert v["suggested_duty_sign"] is None


def test_nan_rates_are_skipped():
    s = wheel_run()
    s[10] = Sample(s[10].host_ns, McuState(**{**s[10].state.__dict__, "wheel_rate_left_rad_s": math.nan}))
    m = commission_metrics(s, axis=0, duty_e4=1000, on_index=2, off_index=12)
    assert math.isfinite(m.rate_mean_rad_s)


def test_an_unknown_end_of_driving_is_not_a_measurement():
    m = commission_metrics(wheel_run(), axis=0, duty_e4=1000, on_index=2, off_index=None)
    assert not m.valid and m.reason == "end_unknown"


def test_an_end_sample_cut_off_the_record_is_an_unknown_end():
    s = wheel_run()
    m = commission_metrics(s, axis=0, duty_e4=1000, on_index=2, off_index=len(s))
    assert not m.valid and m.reason == "end_unknown"
