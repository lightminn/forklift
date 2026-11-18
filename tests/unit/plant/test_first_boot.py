"""S2 first-boot STATE check (arrival-day procedure 6.1): the drivers answer with 11 in the top bits and no fault, the
current inputs read near zero with no motor, no unexpected fault bit, both KTH axes valid and counting, the
untouched wheels still, the link and cycle clean -- over the first seconds of one boot."""

import dataclasses

import pytest

from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag
from forklift_core.plant.bringup import Sample
from forklift_core.plant.first_boot import CURRENT_ZERO_MA, first_boot_check

OK = McuState(mcu_time_us=0, boot_id=7, arm_gen=0, last_rx_seq=0, mode=Mode.DISARMED,
              reject_reason=RejectReason.NONE, fault_flags=FaultFlag(0), wheel_count_left=100,
              wheel_count_right=-5, wheel_rate_left_rad_s=0.0, wheel_rate_right_rad_s=0.0, steering_rad=0.1,
              lift_m=0.05, current_ma=(3, -2, 0, 1), duty_e4=(0,) * 4, driver_status=(0xC0,) * 4, rx_errors=0,
              cycle_overruns=0, flags=StateFlag.DRIVERS_READY | StateFlag.OUTPUTS_LOCKED)


def record(n=1200, dt_us=10_000, **change):
    """n STATEs 10 ms apart; the KTH frame counters rise 10 per STATE and wrap; ``change`` applies from STATE 600."""
    out = []
    for k in range(n):
        st = dataclasses.replace(OK, mcu_time_us=1_000_000 + k * dt_us,
                                 kth_frames=((65000 + 10 * k) & 0xFFFF, (10 * k) & 0xFFFF))
        if k >= 600 and change:
            st = dataclasses.replace(st, **change)
        out.append(Sample(k * dt_us * 1000, st))
    return out


def failed(res):
    return sorted(name for name, c in res["checks"].items() if not c["ok"])


def test_a_clean_first_boot_passes_and_reports_the_build():
    res = first_boot_check(record())
    assert res["result"] == "pass", res
    assert res["report"]["reset_cause"] == "POWER" and "OUTPUTS_LOCKED" in res["report"]["flags"]
    assert res["checks"]["kth"]["frames"] == {"M": 10000, "H": 10000}  # 10 s window: 1000 intervals of 10 frames, across the wrap


@pytest.mark.parametrize("change, name", [
    (dict(driver_status=(0xC0, 0x00, 0xC0, 0xC0)), "drivers"),         # top bits not 11: SPI wiring or no VM
    (dict(driver_status=(0xC0, 0xC0, 0xC2, 0xC0)), "drivers"),         # a fault bit (VMUV) on M
    (dict(flags=StateFlag.OUTPUTS_LOCKED), "drivers"),                  # drivers_ready gone
    (dict(current_ma=(0, 0, CURRENT_ZERO_MA + 1, 0)), "current"),       # an offset past the zero band
    (dict(fault_flags=FaultFlag.CURRENT_SENSE_INVALID), "current"),
    (dict(fault_flags=FaultFlag.ENCODER_GAP), "faults"),
    (dict(steering_rad=float("nan")), "kth"),
    (dict(wheel_count_left=103), "wheels_still"),
    (dict(rx_errors=1), "link"),
    (dict(cycle_overruns=2), "link"),
])
def test_each_problem_fails_its_own_check(change, name):
    res = first_boot_check(record(**change))
    assert res["result"] == "fail" and failed(res) == [name], res


def test_a_stuck_kth_counter_fails_kth():
    rec = record()
    rec = [dataclasses.replace(s, state=dataclasses.replace(s.state, kth_frames=(s.state.kth_frames[0], 0)))
           for s in rec]
    res = first_boot_check(rec)
    assert failed(res) == ["kth"] and res["checks"]["kth"]["frames"]["H"] == 0


def test_lift_absent_excuses_only_the_lift_axis():
    nan_lift = record(lift_m=float("nan"), fault_flags=FaultFlag.LIFT_SENSOR_INVALID)
    assert failed(first_boot_check(nan_lift)) == ["faults", "kth"]
    assert first_boot_check(nan_lift, lift_absent=True)["result"] == "pass"
    nan_steer = record(steering_rad=float("nan"), fault_flags=FaultFlag.STEER_SENSOR_INVALID)
    assert first_boot_check(nan_steer, lift_absent=True)["result"] == "fail"


def test_only_the_window_is_judged():
    # the operator starts turning the wheels after the hands-off window: not a failure
    rec = record()
    late = [dataclasses.replace(s, state=dataclasses.replace(s.state, wheel_count_left=500)) if k >= 1100 else s
            for k, s in enumerate(rec)]
    assert first_boot_check(late, window_s=10.0)["result"] == "pass"


def test_unusable_records_are_invalid_not_pass():
    assert first_boot_check(record(n=30))["result"] == "invalid"            # ends before the window does
    assert first_boot_check(record(n=900))["result"] == "invalid"           # 9 s: still short of 10
    assert first_boot_check(record(boot_id=8))["result"] == "invalid"      # a reboot inside the window


def _edit(rec, k0, k1, **change):
    return [dataclasses.replace(s, state=dataclasses.replace(s.state, **change)) if k0 <= k < k1 else s
            for k, s in enumerate(rec)]


def test_a_stall_or_clock_or_counter_fault_inside_the_window_is_invalid():
    rec = record()
    # 50 good STATEs, then nothing for 3 s: not a pass (Codex P2-1)
    stalled = rec[:50] + [dataclasses.replace(s, state=dataclasses.replace(
        s.state, mcu_time_us=s.state.mcu_time_us + 3_000_000)) for s in rec[50:]]
    assert first_boot_check(stalled)["result"] == "invalid"
    back = _edit(rec, 500, 1200, mcu_time_us=1_000)                          # the MCU clock jumps back (P2-4)
    assert first_boot_check(back)["result"] == "invalid"
    reset = [dataclasses.replace(s, state=dataclasses.replace(s.state, kth_frames=(0, 0))) if k == 500 else s
             for k, s in enumerate(rec)]                                      # a counter reset reads as a wrap (P2-4)
    assert first_boot_check(reset)["result"] == "invalid"


def test_errors_past_frames_are_invalid():
    rec = record()
    bad = [dataclasses.replace(s, state=dataclasses.replace(s.state, kth_crc_errors=(5 * k, 0))) for k, s in
           enumerate(rec)]  # frames +10 per STATE but errors +5 is fine; +11 is not
    assert first_boot_check(bad)["result"] == "pass"
    worse = [dataclasses.replace(s, state=dataclasses.replace(s.state, kth_alt_crc_errors=(0, 11 * k))) for k, s in
             enumerate(rec)]
    assert first_boot_check(worse)["result"] == "invalid"


def test_a_reboot_after_the_window_is_not_judged():
    rec = record(n=1300)
    later = [dataclasses.replace(s, state=dataclasses.replace(s.state, boot_id=9, mcu_time_us=k * 10_000))
             if k >= 1200 else s for k, s in enumerate(rec)]                  # reboot at 12 s, its clock from 0 (P2-2)
    assert first_boot_check(later)["result"] == "pass"


def test_a_hil_or_synthetic_record_is_never_a_pass():
    hil = record(flags=StateFlag.DRIVERS_READY | StateFlag.HIL_PLANT)
    assert first_boot_check(hil)["result"] == "synthetic"
    assert first_boot_check(record(), synthetic_input=True)["result"] == "synthetic"
