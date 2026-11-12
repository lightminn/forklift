"""S2's KTH7801 CRC variant check from recorded STATEs (mcu-link/3 contract 3.3; plan S2, arrival-day procedure).

Each axis (M steering, H lift) carries three 16-bit modular counters: fresh frames, frames failing the configured
CRC variant, frames that would fail the other variant. The two variants agree on 256 of the 4096 angle codes, so zero
configured errors prove the variant only with frames that tell the variants apart (``alt`` > 0). The counters are
read as adjacent-STATE unsigned deltas summed as integers -- first-minus-last would lose wraps -- over one boot, with
the MCU clock moving forward and no gap past ``GAP_MAX_S`` (each counter rises at most 1000/s by contract, so 30 s
stays under 2^16). A HIL build's counters come from the model: synthetic, never an S2 confirmation.

The thresholds are working values, not a validated confidence level; ``alt`` evidence shows distinguishing frames,
not that the whole range was covered (S2 observes the range itself)."""

from __future__ import annotations

import math
from collections.abc import Sequence

from forklift_core.mcu_link.messages import FaultFlag, StateFlag, kth_counter_delta

GAP_MAX_S = 30.0
RATE_MAX_HZ = 1000.0  # contract 3.3: each counter rises at most once a control cycle, the cycle >= 1 ms
MIN_FRAMES = 1000
MIN_DISTINGUISHING = 100
VARIANTS = ("g704_bytes", "reflected_msb_first")  # firmware/core/include/fw/crc.h kth_crc_variant
AXES = ("M", "H")
_AXIS_BAD = {"M": FaultFlag.STEER_SENSOR_INVALID | FaultFlag.STEER_OUT_OF_RANGE,
             "H": FaultFlag.LIFT_SENSOR_INVALID | FaultFlag.LIFT_OUT_OF_RANGE}


def _axis_ok(state, ax: str) -> bool:
    value = state.steering_rad if ax == "M" else state.lift_m
    return math.isfinite(value) and not (state.fault_flags & _AXIS_BAD[ax])


def kth_check(samples: Sequence, *, configured: dict, image: str) -> dict:
    """``samples``: the record's STATEs in receipt order. ``configured``: each axis's variant in the image under
    test; ``image``: that image's identity -- both carried into the result so the verdict names what it judged."""
    if set(configured) != set(AXES) or any(v not in VARIANTS for v in configured.values()):
        raise ValueError(f"configured: a variant for each of {AXES}, one of {VARIANTS}")
    if not isinstance(image, str) or not image.strip():
        raise ValueError("image: the firmware image under test")
    base = {"configured": dict(configured), "image": image, "gap_max_s": GAP_MAX_S,
            "thresholds": {"frames": MIN_FRAMES, "distinguishing": MIN_DISTINGUISHING},
            "note": "alt counts show frames that tell the variants apart, not range coverage (S2 observes that)"}
    if len(samples) < 2:
        return {**base, "result": "invalid", "why": "fewer than two STATEs"}
    synthetic = any(s.state.flags & StateFlag.HIL_PLANT for s in samples)
    sums = {ax: [0, 0, 0] for ax in AXES}
    for a, b in zip(samples, samples[1:]):
        sa, sb = a.state, b.state
        if sb.boot_id != sa.boot_id:
            return {**base, "result": "invalid", "why": "boot changed: record again in one boot"}
        dt = (sb.mcu_time_us - sa.mcu_time_us) / 1e6
        if dt <= 0:
            return {**base, "result": "invalid", "why": "MCU time went back or stood"}
        if dt > GAP_MAX_S:
            return {**base, "result": "invalid", "why": f"gap of {dt:.1f} s between STATEs (over {GAP_MAX_S} s)"}
        # each interval on its own, before any sum (Codex v3 impl P2-1): a rise past the contract's rate is a reset or
        # a wrap the gap hid; errors past frames inside one interval is a contradiction the sums would balance away
        rise_max = math.ceil(dt * RATE_MAX_HZ) + 1  # one cycle of slack at either end
        for i, ax in enumerate(AXES):
            df = kth_counter_delta(sb.kth_frames[i], sa.kth_frames[i])
            de = kth_counter_delta(sb.kth_crc_errors[i], sa.kth_crc_errors[i])
            da = kth_counter_delta(sb.kth_alt_crc_errors[i], sa.kth_alt_crc_errors[i])
            if df > rise_max:
                return {**base, "result": "invalid",
                        "why": f"axis {ax}: frames rise {df} in {dt * 1e3:.1f} ms (over {rise_max}): a reset or a hidden wrap"}
            if de > df or da > df:
                return {**base, "result": "invalid",
                        "why": f"axis {ax}: errors past frames within one interval ({df}, {de}, {da})"}
            sums[ax][0] += df
            sums[ax][1] += de
            sums[ax][2] += da
    for ax in AXES:
        frames, errors, alt = sums[ax]
        if errors > frames or alt > frames:
            return {**base, "result": "invalid", "why": f"axis {ax}: more errors than frames (restore failed)"}
    last = samples[-1].state
    axes = {}
    for ax in AXES:
        frames, errors, alt = sums[ax]
        if frames < MIN_FRAMES:
            verdict, why = "undecided", f"fewer than {MIN_FRAMES} frames"
        elif errors == 0 and alt >= MIN_DISTINGUISHING:
            verdict, why = ("configured_ok", None) if _axis_ok(last, ax) else (
                "undecided", "axis not valid and in range at the end")
        elif alt == 0 and errors >= MIN_DISTINGUISHING:
            verdict, why = "other_variant", None
        elif errors == 0 and alt == 0:
            verdict, why = "undecided", "no frame tells the variants apart: move the axis through its range"
        else:
            verdict, why = "undecided", "errors under both variants: wiring, noise or a third variant"
        entry = {"configured": configured[ax], "frames": frames, "crc_errors": errors, "alt_crc_errors": alt,
                 "verdict": "synthetic" if synthetic else verdict, "why": why}
        if synthetic:
            entry["computed"] = verdict
        axes[ax] = entry
    return {**base, "result": "ok", "synthetic": synthetic, "axes": axes}


__all__ = ["AXES", "GAP_MAX_S", "MIN_DISTINGUISHING", "MIN_FRAMES", "VARIANTS", "kth_check"]
