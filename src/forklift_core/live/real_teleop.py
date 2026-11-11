"""The S6 teleop bridge's decisions, without I/O (docs/design/2026-11-12-teleop-bridge.md): console -> arbiter ->
encoder speed cap -> McuLinkHost, outside the Isaac live runtime.

This module holds the parts that decide; ``tools/teleop_bridge.py`` opens the device and the console server and runs
the wall-clock loop. Nothing here reads simulation truth: the speed a safety check reads is the host's difference of
two STATEs' encoder counts (design 2), never a dead-reckoning pose or the true speed.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from pathlib import Path

from forklift_core.mcu_link.messages import FaultFlag, wheel_count_delta
from forklift_core.plant.bringup import Sample
from forklift_core.safety.teleop_drive import TeleopLimits
from forklift_core.safety.truth_access import SafetySpeed

SCHEMA = "teleop_bridge/1"
STANDING_MPS = 0.01  # the stop confirmation's speed (MissionCore.STANDING_MPS)
_SPEED_BITS = (FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_JUMP_L | FaultFlag.ENCODER_JUMP_R
               | FaultFlag.ENCODER_STUCK_L | FaultFlag.ENCODER_STUCK_R
               | FaultFlag.ENCODER_POLARITY_L | FaultFlag.ENCODER_POLARITY_R)


# --------------------------------------------------------------------------------------------- encoder speed (design 2)
@dataclass(frozen=True)
class EncoderSpeedParams:
    """The values the encoder speed's error width rests on. ``axle_accel_mps2`` and ``state_latency_s`` are the
    premises A1 and A2 (provisional, plan's provisional table): the bound holds only while they do."""

    radius_m: tuple[float, float]
    counts_per_rev: tuple[float, float]
    track_m: float
    kappa_max: float  # the largest curvature the steering map can hold
    axle_accel_mps2: float  # A1: the axle's acceleration bound (each wheel's is this x (1 + kappa_max track / 2))
    state_latency_s: float  # A2: a STATE's measurement-to-receipt delay bound
    max_age_s: float  # the newest STATE older than this (host receipt) is too old for a safety speed
    dt_min_s: float  # the two STATEs differenced are at least this far apart (MCU time) ...
    dt_max_s: float  # ... and at most this

    def __post_init__(self) -> None:
        for name in ("track_m", "axle_accel_mps2", "state_latency_s", "max_age_s", "dt_min_s", "dt_max_s"):
            v = getattr(self, name)
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
                raise ValueError(f"{name} must be a positive finite number")
        if not (math.isfinite(self.kappa_max) and self.kappa_max >= 0):
            raise ValueError("kappa_max must be finite and >= 0")
        if self.dt_min_s >= self.dt_max_s:
            raise ValueError("dt_min_s must be below dt_max_s")
        for pair in (self.radius_m, self.counts_per_rev):
            if len(pair) != 2 or not all(math.isfinite(x) and x > 0 for x in pair):
                raise ValueError("radius_m and counts_per_rev are two positive finite values")

    @property
    def turn_factor(self) -> float:
        """The outer wheel's speed over the axle's at the largest curvature."""
        return 1.0 + self.kappa_max * self.track_m / 2.0

    @property
    def wheel_accel_mps2(self) -> float:
        return self.axle_accel_mps2 * self.turn_factor

    def error(self, wheel: int, dt_s: float, age_s: float) -> float:
        """A wheel's speed error width: the acceleration over half the interval, the age and the latency, plus one
        count at each end of the interval."""
        quant = 2.0 * math.pi * self.radius_m[wheel] / (self.counts_per_rev[wheel] * dt_s)
        return self.wheel_accel_mps2 * (dt_s / 2.0 + age_s + self.state_latency_s) + quant

    def worst_error(self) -> float:
        """The largest error width a valid reading can have: the longest interval and age, the shortest interval's
        quantization."""
        return max(self.wheel_accel_mps2 * (self.dt_max_s / 2.0 + self.max_age_s + self.state_latency_s)
                   + 2.0 * math.pi * self.radius_m[w] / (self.counts_per_rev[w] * self.dt_min_s) for w in (0, 1))


@dataclass(frozen=True)
class EncoderReading:
    speed: SafetySpeed  # valid=False with reason when it cannot be read
    yaw_rate: float | None
    reason: str | None
    dt_s: float | None = None
    age_s: float | None = None
    steering_rad: float | None = None


_INVALID = SafetySpeed(0.0, 0.0, (1, -1), valid=False)


def _invalid(reason: str, *, age_s=None) -> EncoderReading:
    return EncoderReading(replace(_INVALID, age_s=age_s), None, reason, age_s=age_s)


def encoder_speed(samples: Sequence[Sample], now_ns: int, p: EncoderSpeedParams,
                  steering_ok: Callable[[float], bool]) -> EncoderReading:
    """The safety speed from the newest STATE and the latest earlier STATE of the same boot at least ``dt_min_s``
    before it (MCU time), at most ``dt_max_s``: per wheel ``v = 2 pi r dc / (CPR dt)``, the mean, the yaw rate from
    the wheel difference, the bound on the outer wheel, the directions from ``signed +- e`` (design 2)."""
    if not samples:
        return _invalid("no_state")
    cur = samples[-1]
    s1 = cur.state
    age = (now_ns - cur.host_ns) / 1e9
    if age > p.max_age_s or age < -1e-3:
        return _invalid("stale", age_s=age)
    age = max(age, 0.0)
    prev = None
    for smp in reversed(samples[:-1]):
        if smp.state.boot_id != s1.boot_id:
            break
        dt = (s1.mcu_time_us - smp.state.mcu_time_us) / 1e6
        if dt < p.dt_min_s:
            continue
        if dt <= p.dt_max_s:
            prev = smp
        break
    if prev is None:
        return _invalid("no_interval", age_s=age)
    s0 = prev.state
    if (s0.fault_flags | s1.fault_flags) & _SPEED_BITS:
        return _invalid("encoder_fault", age_s=age)
    q = float(s1.steering_rad)
    if not (math.isfinite(q) and steering_ok(q)):
        return _invalid("steering_invalid", age_s=age)
    dt = (s1.mcu_time_us - s0.mcu_time_us) / 1e6
    v, e = [], []
    for w, (c1, c0) in enumerate(((s1.wheel_count_left, s0.wheel_count_left),
                                  (s1.wheel_count_right, s0.wheel_count_right))):
        dc = wheel_count_delta(c1, c0)
        v.append(2.0 * math.pi * p.radius_m[w] * dc / (p.counts_per_rev[w] * dt))
        e.append(p.error(w, dt, age))
    signed = (v[0] + v[1]) / 2.0
    err = max(e)
    bound = max(abs(v[0]) + e[0], abs(v[1]) + e[1])
    lo, hi = signed - err, signed + err
    directions = (1, -1) if (lo < 0.0 < hi) else ((1,) if lo >= 0.0 else (-1,))
    return EncoderReading(SafetySpeed(signed, bound, directions, valid=True, age_s=age),
                          (v[1] - v[0]) / p.track_m, None, dt_s=dt, age_s=age, steering_rad=q)


# ------------------------------------------------------------------------------------------------ the cap (design 3)
def command_cap_mps(limit_mps: float, p: EncoderSpeedParams) -> float:
    """The teleop command cap that keeps a normal command's encoder bound at or under ``limit_mps`` at the largest
    curvature, the oldest reading and the longest interval: ``(V - M) / (1 + kappa_max track / 2)``."""
    cap = (float(limit_mps) - p.worst_error()) / p.turn_factor
    if not cap > 0.0:
        raise ValueError(f"no command cap fits under {limit_mps} m/s with an error width of {p.worst_error():.4f}")
    return cap


def real_teleop_limits(base: TeleopLimits, cap_mps: float) -> TeleopLimits:
    """Every teleop speed limit cut to ``cap_mps`` (never raised)."""
    names = ("video_forward_mps", "video_reverse_mps", "direct_forward_mps", "direct_reverse_mps", "loaded_mps",
             "recovery_reverse_mps")
    return replace(base, **{n: min(getattr(base, n), float(cap_mps)) for n in names})


# ------------------------------------------------------------------------------------- the direction rule (design 8)
def direction_limits(limit_fn, speed: SafetySpeed, direction, cap_mps: float, commanded_mps: float):
    """The core's real-truck rule (``MissionCore._direction_limits``, 4d-1), with the command still in effect given:
    every direction the measured speed may have at the bound, and the command in effect at its magnitude; the
    driving check is ``direction`` at ``cap_mps`` (none asked: every motion direction at the largest of its speeds);
    any other motion direction that does not allow its own speed stops the truck (a hold). Returns
    ``(allowed, why, hold, driving, unsure)``. Tests pin it to the core's."""
    motion = {}
    if speed.bound_mps > 0.0:
        for d_ in speed.directions:
            motion[d_] = speed.bound_mps
    if commanded_mps != 0.0:
        c_ = 1 if commanded_mps > 0 else -1
        motion[c_] = max(motion.get(c_, 0.0), abs(commanded_mps))
    possible = set(motion) | ({direction} if direction is not None else set())
    if direction is not None:
        driving, cap = [direction], cap_mps
    else:
        driving = list(motion) or [speed.directions[0]]
        cap = max([cap_mps, *motion.values()])
    allowed, why = min((limit_fn(d_, cap) for d_ in driving), key=lambda r: r[0])
    hold = False
    for d_, v_ in motion.items():
        if d_ == direction:
            continue
        a_, w_ = limit_fn(d_, v_)
        if a_ < v_:
            allowed, why, hold = 0.0, (w_ if w_ != "ok" else "direction_unsure"), True
    return allowed, why, hold, driving, len(possible) > 1


# ---------------------------------------------------------------------------------------- the safety state (design 7)
CONFIRMATIONS = ("empty_forks", "forks_clear", "lift_low", "no_open_recovery")


def bench_safety(confirmed) -> dict | None:
    """The standalone S6 run's safety picture: unknown (None -- the policy refuses) unless the operator confirmed all
    four separately; then empty forks out of any pocket, no SLAM, no mission."""
    if not set(CONFIRMATIONS) <= set(confirmed):
        return None
    return {"load": {"loaded": False}, "fork_in_pocket": {"value": False}, "near": None,
            "frame": {"hold": "none", "slam": False}, "mission": {"phase": "idle"}}


def enforce_real(safety: dict | None) -> dict | None:
    """The real truck's refusals put before the policy, whatever the provider gave (design 7): loaded teleop, a
    recovery still required, forks in a pocket or a recovery context (the policy would open its straight reverse;
    the real truck recovers by hand -- field recovery decision (B))."""
    if safety is None:
        return None
    s = dict(safety)
    if (s.get("load") or {}).get("loaded"):
        s["teleop_refused"] = "real_loaded"
    elif s.get("recovery_required"):
        s["teleop_refused"] = "real_recovery_required"
    elif (s.get("fork_in_pocket") or {}).get("value") or s.get("recovery"):
        s["teleop_refused"] = "real_in_pocket"
    return s


# ------------------------------------------------------------------------------------------------------ config file
def load_bridge_params(path: str | Path, vehicle) -> EncoderSpeedParams:
    """``config/real/teleop_bridge.yaml`` (strict: exactly its keys, each block with a provenance) with the vehicle's
    radii, counts per revolution, track and steering map."""
    import yaml

    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or set(data) != {"schema", "encoder_speed"} or data["schema"] != SCHEMA:
        raise ValueError(f"teleop bridge config must hold schema {SCHEMA!r} and encoder_speed only")
    es = data["encoder_speed"]
    keys = {"axle_accel_mps2", "state_latency_s", "max_age_s", "dt_min_s", "dt_max_s"}
    if not isinstance(es, dict) or set(es) != keys | {"provenance"}:
        raise ValueError(f"encoder_speed must hold {sorted(keys)} and provenance")
    prov = es["provenance"]
    if not isinstance(prov, dict) or not {"source", "measured"} <= set(prov):
        raise ValueError("encoder_speed.provenance needs source and measured")
    return EncoderSpeedParams(radius_m=tuple(vehicle.wheel_radius_m), counts_per_rev=tuple(vehicle.counts_per_wheel_rev),
                              track_m=vehicle.track_m, kappa_max=max(abs(k) for k in vehicle.steering.curvature_range),
                              **{k: float(es[k]) for k in keys})


__all__ = ["CONFIRMATIONS", "EncoderReading", "EncoderSpeedParams", "STANDING_MPS", "bench_safety", "command_cap_mps",
           "direction_limits", "encoder_speed", "enforce_real", "load_bridge_params", "real_teleop_limits"]
