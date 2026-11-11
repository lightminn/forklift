"""S3 axis-by-axis commissioning results (docs/design/2026-11-12-bringup-commission-tool.md 4): pure functions of
recorded STATE samples. Software evidence only: the physical direction is the operator's observation, and nothing
here writes a configuration -- it proposes a ``duty_sign`` and a sensor sign with their grounds."""

from __future__ import annotations

import math
from dataclasses import dataclass

from forklift_core.mcu_link.messages import FaultFlag, McuState, wheel_count_delta
from forklift_core.plant.bringup import _CUT, _ENCODER_ANY, Sample

AXES = {"L": 0, "R": 1, "M": 2, "H": 3}
COMMISSION_DUTY_MAX = 0.15  # the tool's own bound: the plan's proposed build limit (the build may be lower)
COMMISSION_HOLD_MAX_S = 1.5  # under the proposed 2 s build time limit, so a sound tool never meets it
# a motion smaller than this is "none": wheels in counts, steering in rad, lift in m
NOISE = {0: 4, 1: 4, 2: 0.005, 3: 0.0005}


@dataclass(frozen=True)
class CommissionMetrics:
    valid: bool
    reason: str | None
    axis: int
    duty_e4: int  # commanded
    driven_ms: float | None  # MCU time from the first sample accepting the nonzero command to the one accepting 0
    applied_max_e4: int  # largest |reported duty| on the axis while driven
    duty_sign_in_build: int | None  # reported duty sign / command sign (STATE.duty_e4 carries duty_sign)
    current_peak_ma: int
    current_mean_ma: float | None
    others_peak_ma: int  # the three other channels
    measured_delta: float | None  # wheel counts, or the steering/lift value change
    measured_sign: str  # "pos", "neg", "none", "invalid"
    rate_mean_rad_s: float | None  # wheels: the back half of the driven samples
    rate_peak_rad_s: float | None
    value: dict | None  # steering/lift: start, end, min, max
    encoder_bits_at_start: list
    encoder_bits_new: list  # (name, first MCU time in us) of bits that came on from acceptance on


def _value(s: McuState, axis: int) -> float:
    return s.steering_rad if axis == 2 else s.lift_m


def _bits(flags: FaultFlag) -> list[str]:
    return [f.name for f in FaultFlag if f in _ENCODER_ANY and f in flags]


def _sign(delta: float, axis: int) -> str:
    if abs(delta) < NOISE[axis]:
        return "none"
    return "pos" if delta > 0 else "neg"


def commission_metrics(samples: list[Sample], *, axis: int, duty_e4: int, on_index: int | None,
                       off_index: int | None) -> CommissionMetrics:
    """``on_index``: the first sample accepting the nonzero COMMISSION; ``off_index``: the first accepting the zero
    after it (or the end of driving). The motion runs from the last sample before acceptance to the last sample
    (the coast included); currents, duties and rates over the driven samples only."""
    empty = dict(driven_ms=None, applied_max_e4=0, duty_sign_in_build=None, current_peak_ma=0, current_mean_ma=None,
                 others_peak_ma=0, measured_delta=None, measured_sign="invalid", rate_mean_rad_s=None,
                 rate_peak_rad_s=None, value=None, encoder_bits_at_start=[], encoder_bits_new=[])
    if on_index is None or not samples or on_index >= len(samples):
        return CommissionMetrics(False, "not_accepted", axis, duty_e4, **empty)
    off = len(samples) if off_index is None else min(off_index, len(samples))
    start = max(on_index - 1, 0)
    span = samples[start:]
    driven = samples[on_index:off]
    at_start = FaultFlag(0)
    for s in samples[:on_index]:
        at_start |= s.state.fault_flags & _ENCODER_ANY
    new, seen = [], set(_bits(at_start))
    for s in samples[on_index:]:
        for name in _bits(s.state.fault_flags):
            if name not in seen:
                seen.add(name)
                new.append((name, s.state.mcu_time_us))
    applied = [s.state.duty_e4[axis] for s in driven]
    peak_applied = max(applied, key=abs, default=0)
    duty_sign = None if peak_applied == 0 or duty_e4 == 0 else (1 if (peak_applied > 0) == (duty_e4 > 0) else -1)
    cur = [s.state.current_ma[axis] for s in driven]
    others = [c for s in driven for i, c in enumerate(s.state.current_ma) if i != axis]
    base = dict(driven_ms=(samples[off].state.mcu_time_us - samples[on_index].state.mcu_time_us) / 1000
                if off < len(samples) else None,
                applied_max_e4=abs(peak_applied), duty_sign_in_build=duty_sign,
                current_peak_ma=max(cur, default=0), current_mean_ma=sum(cur) / len(cur) if cur else None,
                others_peak_ma=max(others, default=0), encoder_bits_at_start=_bits(at_start),
                encoder_bits_new=new)

    def bad(reason: str) -> CommissionMetrics:
        return CommissionMetrics(False, reason, axis, duty_e4, **{**empty, **base})

    boot = span[0].state.boot_id
    if any(s.state.boot_id != boot for s in span):
        return bad("boot_changed")
    rate_mean = rate_peak = value = None
    if axis < 2:
        if any(s.state.fault_flags & _CUT[axis] for s in span):
            return bad("encoder_continuity")
        get = (lambda s: s.wheel_count_left) if axis == 0 else (lambda s: s.wheel_count_right)
        delta = sum(wheel_count_delta(get(b.state), get(a.state)) for a, b in zip(span, span[1:]))
        rates = [r for s in driven[len(driven) // 2:]
                 if math.isfinite(r := (s.state.wheel_rate_left_rad_s if axis == 0 else s.state.wheel_rate_right_rad_s))]
        if rates:
            rate_mean, rate_peak = sum(rates) / len(rates), max(rates, key=abs)
    else:
        vals = [_value(s.state, axis) for s in span]
        if not all(math.isfinite(v) for v in vals):
            return bad("value_invalid")
        delta = vals[-1] - vals[0]
        value = {"start": vals[0], "end": vals[-1], "min": min(vals), "max": max(vals)}
    return CommissionMetrics(True, None, axis, duty_e4, **{**empty, **base, "measured_delta": delta,
                                                            "measured_sign": _sign(delta, axis),
                                                            "rate_mean_rad_s": rate_mean,
                                                            "rate_peak_rad_s": rate_peak, "value": value})


def sign_verdict(duty_e4: int, measured_sign: str, observed: str, *, duty_sign_in_build: int | None) -> dict:
    """Design 4: with the operator's physical direction for this command (``pos``: wheel rolling the truck forward,
    steering left, lift up), whether the channel's ``duty_sign`` and the axis sensor's sign are right, and the
    ``duty_sign`` to use (against the build's, read from the reported duty). No observation, no proposal."""
    out = {"duty_sign_ok": None, "sensor_sign_ok": None, "suggested_duty_sign": None, "note": None}
    if observed not in ("pos", "neg", "none"):
        return {**out, "note": "no_observation"}
    if measured_sign == "invalid":
        return {**out, "note": "measurement_invalid"}
    if observed == "none" and measured_sign == "none":
        return {**out, "note": "no_motion"}
    if (observed == "none") != (measured_sign == "none"):
        return {**out, "note": "inconsistent"}
    duty_ok = observed == ("pos" if duty_e4 > 0 else "neg")
    return {"duty_sign_ok": duty_ok, "sensor_sign_ok": measured_sign == observed,
            "suggested_duty_sign": None if duty_sign_in_build is None else duty_sign_in_build * (1 if duty_ok else -1),
            "note": None}
