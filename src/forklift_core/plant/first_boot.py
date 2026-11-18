"""S2's first-boot STATE check (arrival-day procedure 6.1): read the first seconds of one boot's record and say
whether the board came up as it should before anyone turns a wheel.

- drivers: every STATE has ``drivers_ready`` and each DRV8244 status byte has 11 in its top two bits (else the SPI
  wiring or VM is wrong -- with no VM the drivers answer 0x0000) and no fault bit in the low six;
- current: no ``current_sense_invalid`` and every motor current within ``CURRENT_ZERO_MA`` of zero (no motor is
  connected in S2, so this is the ADC zero; one ADC count is about 0.24 mA with R_IPROPI 1 kOhm);
- faults: no fault bit outside the allowed set (the lift axis's two when ``lift_absent``);
- kth: steering (and lift unless absent) never NaN, and each axis's fresh-frame counter rising;
- wheels_still: the wheel counts move at most ``WHEEL_STILL_COUNTS`` -- nobody touches the truck in the window;
- link: no rise in ``rx_errors`` or ``cycle_overruns`` and no ``cycle_overrun`` bit.

The thresholds are working values for a first look, not calibrations. A HIL plant's or a synthetic record's STATEs
are never a pass."""

from __future__ import annotations

import math
from collections.abc import Sequence

from forklift_core.mcu_link.messages import FaultFlag, StateFlag, kth_counter_delta, wheel_count_delta

WINDOW_S = 10.0
MIN_STATES = 50  # half a second of STATEs at 100 Hz
CURRENT_ZERO_MA = 50
WHEEL_STILL_COUNTS = 2
STATUS_TOP = 0xC0  # the DRV8244 SPI response's top bits are always 11 (SLVSG24C table 8-21)
LIFT_BITS = FaultFlag.LIFT_SENSOR_INVALID | FaultFlag.LIFT_OUT_OF_RANGE


def first_boot_check(samples: Sequence, *, window_s: float = WINDOW_S, lift_absent: bool = False,
                     synthetic_input: bool = False) -> dict:
    """``samples``: the record's STATEs in receipt order (bring-up record rows read back)."""
    base = {"window_s": window_s, "lift_absent": lift_absent,
            "thresholds": {"current_zero_ma": CURRENT_ZERO_MA, "wheel_still_counts": WHEEL_STILL_COUNTS}}
    if not samples:
        return {**base, "result": "invalid", "why": "no STATE in the record"}
    t0 = samples[0].state.mcu_time_us
    win = [s.state for s in samples if s.state.mcu_time_us - t0 <= window_s * 1e6]
    if len(win) < MIN_STATES:
        return {**base, "result": "invalid", "why": f"{len(win)} STATEs in the window (need {MIN_STATES})"}
    if any(st.boot_id != win[0].boot_id for st in win):
        return {**base, "result": "invalid", "why": "the board rebooted inside the window: look at reset_cause"}
    first, last = win[0], win[-1]
    checks = {}

    bad_status = sorted({f"{'LRMH'[i]}=0x{v:02X}" for st in win for i, v in enumerate(st.driver_status)
                         if (v & 0xC0) != STATUS_TOP or (v & 0x3F)})
    not_ready = sum(1 for st in win if not st.flags & StateFlag.DRIVERS_READY)
    checks["drivers"] = {"ok": not bad_status and not not_ready, "bad_status": bad_status,
                         "states_not_ready": not_ready}

    peak = [max(abs(st.current_ma[i]) for st in win) for i in range(4)]
    sense_bad = any(st.fault_flags & FaultFlag.CURRENT_SENSE_INVALID for st in win)
    checks["current"] = {"ok": max(peak) <= CURRENT_ZERO_MA and not sense_bad,
                         "peak_abs_ma": dict(zip("LRMH", peak)), "current_sense_invalid": sense_bad}

    allowed = LIFT_BITS if lift_absent else FaultFlag(0)
    seen = FaultFlag(0)
    for st in win:
        seen |= st.fault_flags
    unexpected = seen & ~allowed & ~FaultFlag.CURRENT_SENSE_INVALID  # that one belongs to "current"
    checks["faults"] = {"ok": not unexpected, "unexpected": [f.name for f in FaultFlag if f in unexpected],
                        "seen": [f.name for f in FaultFlag if f in seen]}

    axes = ("M",) if lift_absent else ("M", "H")
    frames = {ax: sum(kth_counter_delta(b.kth_frames[i], a.kth_frames[i]) for a, b in zip(win, win[1:]))
              for i, ax in enumerate(("M", "H"))}
    nan = {"M": sum(1 for st in win if not math.isfinite(st.steering_rad)),
           "H": sum(1 for st in win if not math.isfinite(st.lift_m))}
    checks["kth"] = {"ok": all(frames[ax] > 0 and nan[ax] == 0 for ax in axes), "frames": frames,
                     "nan_states": nan}

    moved = {side: max(abs(wheel_count_delta(getattr(st, f"wheel_count_{side}"), getattr(first, f"wheel_count_{side}")))
                       for st in win) for side in ("left", "right")}
    checks["wheels_still"] = {"ok": max(moved.values()) <= WHEEL_STILL_COUNTS, "max_counts_moved": moved}

    rx_rise = (last.rx_errors - first.rx_errors) & 0xFFFF
    overrun_rise = (last.cycle_overruns - first.cycle_overruns) & 0xFFFF
    overrun_bit = any(st.fault_flags & FaultFlag.CYCLE_OVERRUN for st in win)
    checks["link"] = {"ok": rx_rise == 0 and overrun_rise == 0 and not overrun_bit, "rx_errors_rise": rx_rise,
                      "cycle_overruns_rise": overrun_rise}

    report = {"reset_cause": first.reset_cause.name, "mode": last.mode.name, "states": len(win),
              "flags": [f.name for f in StateFlag if f in last.flags]}
    computed = "pass" if all(c["ok"] for c in checks.values()) else "fail"
    if synthetic_input or any(st.flags & StateFlag.HIL_PLANT for st in win):
        return {**base, "result": "synthetic", "computed": computed, "checks": checks, "report": report}
    return {**base, "result": computed, "checks": checks, "report": report}
