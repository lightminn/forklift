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

The window is the receipt-order prefix of one boot from the first STATE: it must reach ``window_s`` on the MCU clock
with no STATE gap over ``GAP_MAX_S``, the clock only moving forward and each KTH counter rising at most once a
millisecond (mcu-link/3 3.3) -- otherwise the record cannot be judged (invalid), never a pass.

The thresholds are working values for a first look, not calibrations. A HIL plant's or a synthetic record's STATEs
are never a pass."""

from __future__ import annotations

import math
from collections.abc import Sequence

from forklift_core.mcu_link.messages import FaultFlag, StateFlag, kth_counter_delta, wheel_count_delta

WINDOW_S = 10.0
GAP_MAX_S = 0.1  # STATE every 10 ms (board design 5): a tenth of a second without one is a stall, not a first boot
KTH_RATE_PER_MS = 1  # contract 3.3: each counter rises at most once a control cycle, the cycle >= 1 ms
CURRENT_ZERO_MA = 50
WHEEL_STILL_COUNTS = 2
STATUS_TOP = 0xC0  # the DRV8244 SPI response's top bits are always 11 (SLVSG24C table 8-21)
LIFT_BITS = FaultFlag.LIFT_SENSOR_INVALID | FaultFlag.LIFT_OUT_OF_RANGE
STEER_BITS = FaultFlag.STEER_SENSOR_INVALID | FaultFlag.STEER_OUT_OF_RANGE


def first_boot_check(samples: Sequence, *, window_s: float = WINDOW_S, lift_absent: bool = False,
                     steer_absent: bool = False, synthetic_input: bool = False) -> dict:
    """``samples``: the record's STATEs in receipt order (bring-up record rows read back). ``lift_absent`` /
    ``steer_absent``: that axis's KTH7801 is not fitted -- its sensor bits are allowed and it is not judged
    (2026-11-25 partial-parts bench; the caller marks a steering-absent result not for a gate)."""
    base = {"window_s": window_s, "lift_absent": lift_absent, "steer_absent": steer_absent,
            "thresholds": {"current_zero_ma": CURRENT_ZERO_MA, "wheel_still_counts": WHEEL_STILL_COUNTS}}
    if not samples:
        return {**base, "result": "invalid", "why": "no STATE in the record"}
    # the window in receipt order: stop at its end; a reboot, the clock going back, a gap or a counter rising faster
    # than the contract allows before the end makes the record unjudgeable (Codex first-boot review P2-1, -2, -4)
    first = samples[0].state
    win = [first]
    fitted = [i for i, absent in enumerate((steer_absent, lift_absent)) if not absent]  # M, H counter indices
    end_us = first.mcu_time_us + window_s * 1e6
    for s in samples[1:]:
        st, prev = s.state, win[-1]
        if st.boot_id != first.boot_id:
            return {**base, "result": "invalid", "why": "the board rebooted inside the window: look at reset_cause"}
        dt_us = st.mcu_time_us - prev.mcu_time_us
        if dt_us <= 0:
            return {**base, "result": "invalid", "why": "the MCU clock went back or stood inside the window"}
        if dt_us > GAP_MAX_S * 1e6:
            return {**base, "result": "invalid", "why": f"a {dt_us / 1e6:.2f} s gap between STATEs inside the window"}
        cap = KTH_RATE_PER_MS * (dt_us // 1000 + 1)
        rises = [kth_counter_delta(getattr(st, f)[i], getattr(prev, f)[i])
                 for f in ("kth_frames", "kth_crc_errors", "kth_alt_crc_errors") for i in range(2)]
        # an absent axis's counters are not read (Codex PR-A P3): only the fitted axes' can make the record invalid
        rises = [r if k % 2 in fitted else 0 for k, r in enumerate(rises)]
        if max(rises) > cap:
            return {**base, "result": "invalid", "why": "a KTH counter rose faster than once a millisecond (reset?)"}
        # per axis: errors and other-variant errors are counted among the fresh frames (contract 3.3, Codex P2)
        if any(rises[2 + i] > rises[i] or rises[4 + i] > rises[i] for i in range(2)):
            return {**base, "result": "invalid", "why": "KTH error counts rose past the frame count (contract 3.3)"}
        win.append(st)
        if st.mcu_time_us >= end_us:
            break
    else:
        return {**base, "result": "invalid",
                "why": f"the record ends {(win[-1].mcu_time_us - first.mcu_time_us) / 1e6:.2f} s into the "
                       f"{window_s:g} s window: record longer"}
    last = win[-1]
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

    allowed = (LIFT_BITS if lift_absent else FaultFlag(0)) | (STEER_BITS if steer_absent else FaultFlag(0))
    seen = FaultFlag(0)
    for st in win:
        seen |= st.fault_flags
    unexpected = seen & ~allowed & ~FaultFlag.CURRENT_SENSE_INVALID  # that one belongs to "current"
    checks["faults"] = {"ok": not unexpected, "unexpected": [f.name for f in FaultFlag if f in unexpected],
                        "seen": [f.name for f in FaultFlag if f in seen]}

    axes = tuple(ax for ax, absent in (("M", steer_absent), ("H", lift_absent)) if not absent)
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
