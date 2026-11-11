"""S1–S6 bring-up calculations (docs/design/2026-11-12-bringup-host-tools.md 4): wheel counts per revolution, a
step's response, the count settle after a stop, and the drive limits. Pure functions of recorded STATE samples --
software evidence only: none of them proves the physical output stopped or the body stopped (design 1)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from forklift_core.mcu_link.messages import FaultFlag, McuState, wheel_count_delta

MOTOR_COUNTS_PER_REV = 4096  # the motor encoder (MT6701 ABZ x4) -- G follows only once the operator confirmed it
_CUT = {  # the bits that break a wheel's count continuity (contract 3.3, host reading)
    0: FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_JUMP_L,
    1: FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_JUMP_R,
}
# every encoder fault: a count settle never counts on an encoder judged stuck or reversed either
_ENCODER_ANY = (FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_STUCK_L | FaultFlag.ENCODER_STUCK_R
                | FaultFlag.ENCODER_POLARITY_L | FaultFlag.ENCODER_POLARITY_R | FaultFlag.ENCODER_JUMP_L
                | FaultFlag.ENCODER_JUMP_R)
GAP_US = 50_000  # two STATEs further apart than this are an observation gap: no hold is counted across it


@dataclass(frozen=True)
class Sample:
    """One STATE as received: the host monotonic time it was read at and the message."""

    host_ns: int
    state: McuState


def _count(s: McuState, wheel: int) -> int:
    return s.wheel_count_left if wheel == 0 else s.wheel_count_right


def _rate(s: McuState, wheel: int) -> float:
    return s.wheel_rate_left_rad_s if wheel == 0 else s.wheel_rate_right_rad_s


@dataclass(frozen=True)
class CprResult:
    valid: bool
    reason: str | None
    counts: tuple[int, int]  # signed sums, left and right
    cpr: tuple[float | None, float | None]  # counts per wheel revolution
    gear: tuple[float | None, float | None]  # cpr / 4096, only when the motor encoder was confirmed

    @property
    def difference(self) -> float | None:
        a, b = self.cpr
        return None if a is None or b is None else abs(a) - abs(b)


def wheel_cpr(samples: list[Sample], turns: tuple[float, float], direction: tuple[int, int], *,
              motor_counts_confirmed: bool = False) -> CprResult:
    """Counts per wheel revolution from a hand turn: ``turns`` and ``direction`` (+1 forward rolling, -1 back) are
    the operator's, per wheel (0 turns: that wheel not turned). Signed modular sums within one boot; invalid on a boot
    change, a time going back, a continuity bit on a turned wheel, or a sign against the operator's direction."""
    if len(samples) < 2:
        return CprResult(False, "too_few_samples", (0, 0), (None, None), (None, None))
    sums = [0, 0]
    boot = samples[0].state.boot_id
    for prev, cur in zip(samples, samples[1:]):
        if cur.state.boot_id != boot:
            return CprResult(False, "boot_changed", tuple(sums), (None, None), (None, None))
        if cur.state.mcu_time_us < prev.state.mcu_time_us:
            return CprResult(False, "time_back", tuple(sums), (None, None), (None, None))
        for w in (0, 1):
            if turns[w] and cur.state.fault_flags & _CUT[w]:
                return CprResult(False, "continuity_cut", tuple(sums), (None, None), (None, None))
            sums[w] += wheel_count_delta(_count(cur.state, w), _count(prev.state, w))
    cpr: list[float | None] = [None, None]
    for w in (0, 1):
        if not turns[w]:
            continue
        if sums[w] == 0 or (sums[w] > 0) != (direction[w] > 0):
            return CprResult(False, "direction_mismatch", tuple(sums), (None, None), (None, None))
        cpr[w] = abs(sums[w]) / float(turns[w])
    gear = tuple(None if c is None or not motor_counts_confirmed else c / MOTOR_COUNTS_PER_REV for c in cpr)
    return CprResult(True, None, tuple(sums), tuple(cpr), gear)


@dataclass(frozen=True)
class StepMetrics:
    rise_s: tuple[float, float] | None  # 10 % -> 90 % (an interval when a crossing falls in a gap)
    overshoot_pct: float | None
    settle_s: tuple[float, float] | None  # first time inside +-5 % held for hold_s (None: unsettled)
    unsettled: bool
    gaps: list = field(default_factory=list)  # MCU-time gaps longer than gap_us (crossing times become intervals)


def _crossing(points, level: float):
    """The MCU-time interval [t_before, t_at] in which the normalised value first reaches ``level``."""
    for (t0, y0), (t1, y1) in zip(points, points[1:]):
        if y0 < level <= y1:
            return (t0, t1)
    return None


def step_metrics(samples: list[Sample], wheel: int, step_mcu_us: int, initial: float, target: float, *,
                 window_s: float = 3.0, hold_s: float = 0.2, band: float = 0.05,
                 gap_us: int = GAP_US) -> StepMetrics:
    """A wheel rate step's response from ``step_mcu_us`` (the first STATE seen to accept the step), normalised from
    ``initial`` to ``target`` (either sign). Times are MCU-time seconds from that STATE. The last sample before it is
    kept (negative time): the step was applied somewhere in between, so a response already under way at the first
    accepting STATE still has its crossing as an interval. A zero step is refused (ValueError)."""
    span = target - initial
    if span == 0:
        raise ValueError("a zero step has no response")
    before = [s for s in samples if s.state.mcu_time_us < step_mcu_us]
    window = [s for s in samples if step_mcu_us <= s.state.mcu_time_us <= step_mcu_us + window_s * 1e6]
    pts = [((s.state.mcu_time_us - step_mcu_us) / 1e6, (_rate(s.state, wheel) - initial) / span)
           for s in before[-1:] + window]
    gaps = [(a[0], b[0]) for a, b in zip(pts, pts[1:]) if (b[0] - a[0]) * 1e6 > gap_us]
    if not pts:
        return StepMetrics(None, None, None, True, gaps)
    c10, c90 = _crossing(pts, 0.1), _crossing(pts, 0.9)
    rise = None if c10 is None or c90 is None else (c90[0] - c10[1], c90[1] - c10[0])
    peak = max(y for _, y in pts)
    overshoot = max(0.0, (peak - 1.0) * 100.0) if c90 is not None else None
    settle = None
    for i, (t, y) in enumerate(pts):
        if abs(y - 1.0) > band:
            continue
        j = i
        while (j + 1 < len(pts) and abs(pts[j + 1][1] - 1.0) <= band
               and (pts[j + 1][0] - pts[j][0]) * 1e6 <= gap_us):  # never held across an observation gap
            j += 1
        if pts[j][0] - t >= hold_s:
            prev_t = pts[i - 1][0] if i > 0 else t
            settle = (prev_t, t)
            break
    return StepMetrics(rise, overshoot, settle, settle is None, gaps)


@dataclass(frozen=True)
class Settle:
    host_ns: int | None  # the received time of the first sample of the settled run (None: undetermined)
    mcu_us: int | None
    reason: str | None  # why undetermined


def count_settle(samples: list[Sample], start_index: int = 0, *, tolerance: int = 2, hold_s: float = 0.3,
                 gap_us: int = GAP_US) -> Settle:
    """The first time both wheel counts stayed within ``tolerance`` counts for ``hold_s`` (MCU time) -- a count
    settle, not a body stop. A boot change, any encoder fault or a time going back is never a settle, and a hold is
    never counted across an observation gap."""
    seq = samples[start_index:]
    for i, first in enumerate(seq):
        boot = first.state.boot_id
        ok = True
        prev_us = first.state.mcu_time_us
        for cur in seq[i:]:
            st = cur.state
            if st.boot_id != boot or st.mcu_time_us < first.state.mcu_time_us:
                return Settle(None, None, "observation_broken")
            if st.fault_flags & _ENCODER_ANY or st.mcu_time_us - prev_us > gap_us:
                ok = False
                break
            prev_us = st.mcu_time_us
            if any(abs(wheel_count_delta(_count(st, w), _count(first.state, w))) > tolerance for w in (0, 1)):
                ok = False
                break
            if (st.mcu_time_us - first.state.mcu_time_us) / 1e6 >= hold_s:
                return Settle(first.host_ns, first.state.mcu_time_us, None)
        if not ok:
            continue
        break  # the observation ended inside a still run shorter than hold_s
    return Settle(None, None, "undetermined")


def rolling_distance_m(counts: int, cpr: float, radius_m: float) -> float:
    """The wheel's rolling estimate (on the stand, the circumference travelled) -- never the body's stopping
    distance (backlash, slip, a locked wheel sliding): the floor distance is an external measurement."""
    return counts / cpr * 2.0 * math.pi * radius_m


def wheel_rate_cap(rate_limit_rad_s: float, radius_m: float | None, *, ground_speed_mps: float = 0.2,
                   unknown_radius_cap: float = 1.0) -> float:
    """The bring-up drive commands' wheel rate bound: the operating limit and the rate of 0.2 m/s on the wheel's
    measured radius (1.0 rad/s until the radius is measured)."""
    by_speed = unknown_radius_cap if radius_m is None else ground_speed_mps / radius_m
    return min(float(rate_limit_rad_s), by_speed)
