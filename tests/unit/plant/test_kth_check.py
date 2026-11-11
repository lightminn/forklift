"""S2's KTH CRC variant check from recorded STATEs (mcu-link/3 contract 3.3): adjacent-STATE unsigned modular deltas
summed per axis, a span the counters cannot be restored over (a boot change, the MCU clock going back, a gap past
30 s, a restored count past the frames) is invalid, the symmetric verdict needs distinguishing frames, and a HIL
build's counters are synthetic."""

import dataclasses
import math

import pytest

from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag
from forklift_core.plant.bringup import Sample
from forklift_core.plant.kth_check import GAP_MAX_S, kth_check

BASE = McuState(mcu_time_us=0, boot_id=7, arm_gen=0, last_rx_seq=0, mode=Mode.DISARMED,
                reject_reason=RejectReason.NONE, fault_flags=FaultFlag(0), wheel_count_left=0, wheel_count_right=0,
                wheel_rate_left_rad_s=0.0, wheel_rate_right_rad_s=0.0, steering_rad=0.1, lift_m=0.05,
                current_ma=(0,) * 4, duty_e4=(0,) * 4, driver_status=(0,) * 4, rx_errors=0, cycle_overruns=0,
                flags=StateFlag(0))
CONF = {"M": "g704_bytes", "H": "g704_bytes"}


def run(steps, *, start=(0, 0, 0, 0, 0, 0), dt_us=10_000, **state_kw):
    """``steps``: per STATE, the counters' rise (frames M, H, errors M, H, alt M, H); the counters wrap at 2^16."""
    c = list(start)
    out = []
    t = 1_000_000
    for k, rise in enumerate([(0,) * 6] + list(steps)):
        c = [(a + b) & 0xFFFF for a, b in zip(c, rise)]
        st = dataclasses.replace(BASE, mcu_time_us=t, kth_frames=(c[0], c[1]), kth_crc_errors=(c[2], c[3]),
                                 kth_alt_crc_errors=(c[4], c[5]), **state_kw)
        out.append(Sample(k * 10_000_000, st))
        t += dt_us
    return out


def good(n=200):
    # 10 frames per STATE per axis, no configured errors, alt fails on 9 of 10 (the axes moving)
    return [(10, 10, 0, 0, 9, 9)] * n


def test_the_configured_variant_is_confirmed_with_distinguishing_frames():
    res = kth_check(run(good()), configured=CONF, image="fw@abc")
    assert res["result"] == "ok" and res["synthetic"] is False and res["image"] == "fw@abc"
    for ax in ("M", "H"):
        a = res["axes"][ax]
        assert (a["frames"], a["crc_errors"], a["alt_crc_errors"]) == (2000, 0, 1800)
        assert a["verdict"] == "configured_ok" and a["configured"] == "g704_bytes"


def test_the_other_variant_is_named_symmetrically():
    res = kth_check(run([(10, 10, 9, 9, 0, 0)] * 200), configured=CONF, image="x")
    assert all(res["axes"][ax]["verdict"] == "other_variant" for ax in ("M", "H"))


def test_a_still_axis_at_an_angle_both_variants_pass_is_undecided():
    """No distinguishing frame: zero errors prove nothing (a stuck-low MISO reads 0x0000, which both pass)."""
    res = kth_check(run([(10, 10, 0, 0, 0, 0)] * 200), configured=CONF, image="x")
    for ax in ("M", "H"):
        assert res["axes"][ax]["verdict"] == "undecided" and "move" in res["axes"][ax]["why"]


def test_one_frame_is_never_enough_either_way():
    """Codex v3 design P2-2: the old 50 % rule named the other variant from one frame."""
    res = kth_check(run([(1, 1, 1, 1, 0, 0)]), configured=CONF, image="x")
    assert all(res["axes"][ax]["verdict"] == "undecided" for ax in ("M", "H"))


def test_a_wrap_between_adjacent_states_is_restored_and_first_minus_last_is_not_used():
    """Codex v3 design P2-1: (66536, 65536, 1000) read first-to-last is (1000, 0, 1000) -- a false configured_ok."""
    steps = [(6000, 6000, 6000, 6000, 100, 100)] * 11 + [(536, 536, 36, 36, 0, 0)]  # 66,536 frames, 66,036 errors
    res = kth_check(run(steps, dt_us=6_000_000), configured=CONF, image="x")
    a = res["axes"]["M"]
    assert (a["frames"], a["crc_errors"], a["alt_crc_errors"]) == (66536, 66036, 1100)
    assert a["verdict"] == "undecided" and "both" in a["why"]  # never configured_ok


def test_counters_starting_near_the_wrap_still_count():
    res = kth_check(run(good(), start=(0xFFF0, 0xFFF0, 0xFFFF, 0xFFFF, 0xFFF8, 0xFFF8)), configured=CONF, image="x")
    assert res["axes"]["M"]["frames"] == 2000 and res["axes"]["M"]["verdict"] == "configured_ok"


@pytest.mark.parametrize("mutate, why", [
    (lambda s: s[:50] + [Sample(s[50].host_ns, dataclasses.replace(s[50].state, boot_id=8))] + s[51:], "boot"),
    (lambda s: s[:50] + [Sample(s[50].host_ns, dataclasses.replace(s[50].state, mcu_time_us=5))] + s[51:], "time"),
    (lambda s: s[:50] + [Sample(s[50].host_ns, dataclasses.replace(
        s[50].state, mcu_time_us=s[49].state.mcu_time_us + int((GAP_MAX_S + 1) * 1e6)))]
     + [Sample(x.host_ns, dataclasses.replace(x.state, mcu_time_us=x.state.mcu_time_us + int((GAP_MAX_S + 1) * 1e6)))
        for x in s[51:]], "gap"),
])
def test_a_span_the_counters_cannot_be_restored_over_is_invalid(mutate, why):
    res = kth_check(mutate(run(good())), configured=CONF, image="x")
    assert res["result"] == "invalid" and why in res["why"]


def test_more_errors_than_frames_is_invalid():
    res = kth_check(run([(1, 1, 5, 0, 0, 0)] * 10), configured=CONF, image="x")
    assert res["result"] == "invalid" and "errors" in res["why"]


def test_a_short_state_loss_is_restored_from_the_cumulative_counters():
    s = run(good())
    kept = s[:40] + s[45:]  # four STATEs lost: 40 ms of MCU time
    res = kth_check(kept, configured=CONF, image="x")
    assert res["result"] == "ok" and res["axes"]["M"]["frames"] == 2000


def test_a_hil_build_is_synthetic_and_never_an_s2_confirmation():
    res = kth_check(run(good(), flags=StateFlag.HIL_PLANT | StateFlag.OUTPUTS_LOCKED), configured=CONF, image="x")
    assert res["synthetic"] is True
    assert all(res["axes"][ax]["verdict"] == "synthetic" for ax in ("M", "H"))
    assert all(res["axes"][ax]["computed"] == "configured_ok" for ax in ("M", "H"))


@pytest.mark.parametrize("kw, ax", [
    ({"steering_rad": math.nan}, "M"),
    ({"lift_m": math.nan}, "H"),
    ({"fault_flags": FaultFlag.STEER_OUT_OF_RANGE}, "M"),
    ({"fault_flags": FaultFlag.LIFT_SENSOR_INVALID}, "H"),
])
def test_an_axis_not_valid_and_in_range_at_the_end_is_not_confirmed(kw, ax):
    res = kth_check(run(good(), **kw), configured=CONF, image="x")
    assert res["axes"][ax]["verdict"] == "undecided" and "axis" in res["axes"][ax]["why"]
    other = "H" if ax == "M" else "M"
    assert res["axes"][other]["verdict"] == "configured_ok"


def test_the_configured_variants_and_the_image_are_required():
    with pytest.raises(ValueError):
        kth_check(run(good()), configured={"M": "g704_bytes"}, image="x")
    with pytest.raises(ValueError):
        kth_check(run(good()), configured={"M": "crc8", "H": "g704_bytes"}, image="x")
    with pytest.raises(ValueError):
        kth_check(run(good()), configured=CONF, image="")
