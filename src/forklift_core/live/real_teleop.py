"""The S6 teleop bridge's decisions, without I/O (docs/design/2026-11-12-teleop-bridge.md): console -> arbiter ->
encoder speed cap -> McuLinkHost, outside the Isaac live runtime.

This module holds the parts that decide; ``tools/teleop_bridge.py`` opens the device and the console server and runs
the wall-clock loop. Nothing here reads simulation truth: the speed a safety check reads is the host's difference of
two STATEs' encoder counts (design 2), never a dead-reckoning pose or the true speed.
"""

from __future__ import annotations

import itertools
import math
import secrets
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from forklift_core.mcu_link.messages import FaultFlag, Mode, wheel_count_delta
from forklift_core.plant.bringup import Sample
from forklift_core.plant.real_vehicle import DriveRefusal
from forklift_core.safety.arbiter import Feedback
from forklift_core.safety.teleop_drive import TeleopLimits, bundle
from forklift_core.safety.truth_access import SafetySpeed

SCHEMA = "teleop_bridge/1"
STANDING_MPS = 0.01  # the stop confirmation's speed (MissionCore.STANDING_MPS)
_SPEED_BITS = (FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_JUMP_L | FaultFlag.ENCODER_JUMP_R
               | FaultFlag.ENCODER_STUCK_L | FaultFlag.ENCODER_STUCK_R
               | FaultFlag.ENCODER_POLARITY_L | FaultFlag.ENCODER_POLARITY_R)


# --------------------------------------------------------------------------------------------- encoder speed (design 2)
@dataclass(frozen=True)
class EncoderSpeedParams:
    """The values the encoder speed's error width rests on. ``axle_accel_mps2`` is premise A1 (provisional, plan's
    provisional table): the bound holds only while it does. A STATE's delay is not a premise: it is measured by
    round trip (``DelayBound``) and counted in its age."""

    radius_m: tuple[float, float]
    counts_per_rev: tuple[float, float]
    track_m: float
    kappa_max: float  # the largest curvature the steering map can hold
    axle_accel_mps2: float  # A1: the axle's acceleration bound (each wheel's is this x (1 + kappa_max track / 2))
    max_age_s: float  # the newest STATE older than this (host receipt) is too old for a safety speed
    dt_min_s: float  # the two STATEs differenced are at least this far apart (MCU time) ...
    dt_max_s: float  # ... and at most this

    def __post_init__(self) -> None:
        for name in ("track_m", "axle_accel_mps2", "max_age_s", "dt_min_s", "dt_max_s"):
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
        """A wheel's speed error width: the acceleration over half the interval and the age (its receipt age plus
        its measured delay), plus one count at each end of the interval."""
        quant = 2.0 * math.pi * self.radius_m[wheel] / (self.counts_per_rev[wheel] * dt_s)
        return self.wheel_accel_mps2 * (dt_s / 2.0 + age_s) + quant

    def worst_error(self) -> float:
        """The largest error width a valid reading can have: the longest interval and age, the shortest interval's
        quantization."""
        return max(self.wheel_accel_mps2 * (self.dt_max_s / 2.0 + self.max_age_s)
                   + 2.0 * math.pi * self.radius_m[w] / (self.counts_per_rev[w] * self.dt_min_s) for w in (0, 1))


@dataclass(frozen=True)
class EncoderReading:
    speed: SafetySpeed  # valid=False with reason when it cannot be read
    yaw_rate: float | None
    reason: str | None
    dt_s: float | None = None
    age_s: float | None = None
    steering_rad: float | None = None
    wheel_mps: tuple | None = None  # each wheel's measured speed (left, right), wheel forward positive
    wheel_err: tuple | None = None  # each wheel's error width


_INVALID = SafetySpeed(0.0, 0.0, (1, -1), valid=False)


def _invalid(reason: str, *, age_s=None) -> EncoderReading:
    return EncoderReading(replace(_INVALID, age_s=age_s), None, reason, age_s=age_s)


def encoder_speed(samples: Sequence[Sample], now_ns: int, p: EncoderSpeedParams,
                  steering_ok: Callable[[float], bool], delay_s: float | None = 0.0) -> EncoderReading:
    """The safety speed from the newest STATE and the latest earlier STATE of the same boot at least ``dt_min_s``
    before it (MCU time), at most ``dt_max_s``: per wheel ``v = 2 pi r dc / (CPR dt)``, the mean, the yaw rate from
    the wheel difference, the bound on the outer wheel, the directions from ``signed +- e`` (design 2)."""
    if not samples:
        return _invalid("no_state")
    cur = samples[-1]
    s1 = cur.state
    # the receipt age plus the STATE's own measurement-to-receipt delay (an upper bound by round trip, design 2);
    # none known: no safety speed
    if delay_s is None:
        return _invalid("no_delay_bound")
    age = (now_ns - cur.host_ns) / 1e9 + max(0.0, float(delay_s))
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
                          (v[1] - v[0]) / p.track_m, None, dt_s=dt, age_s=age, steering_rad=q,
                          wheel_mps=(v[0], v[1]), wheel_err=(e[0], e[1]))


class DelayBound:
    """An upper bound on each STATE's delay from its measurement to its receipt, by round trip (design 2; Codex
    re-review 3 P1: no offset baseline survives a clock rate difference -- a fast MCU clock hid a growing delay).

    The firmware runs one cycle at a time: inputs read, the link's frames handled, a STATE sent (``app.c``). When a
    STATE's ``last_rx_seq`` moves to ``k`` -- a frame this session wrote (the host starts each session at a random
    seq: re-review 5 P1, an earlier session's late STATE) -- ``k`` was handled in that STATE's cycle or before. The
    **next** STATE of the boot was measured in a later cycle, so after ``k`` was handled and so after ``k`` was
    written: its delay is at most ``its receipt - write of k`` whatever either clock's rate or how long a cycle took
    (re-review 5 P2: the overrun count is updated a cycle late; re-review 4 P1: the initial seq 0 is no
    acknowledgement). That STATE is an anchor ``(offset, receipt, MCU time, bound)``, ``offset = receipt - MCU time``.
    Another STATE ``j`` of the same boot is bounded from an anchor ``a`` by
    ``bound_a + (offset_j - offset_a) + drift |MCU_j - MCU_a| / (1 - drift)`` -- the measurement interval taken from the
    MCU clock (re-review 4 P2) -- over at most ``anchor_max_age_s`` (premise A3); the smallest counts. No recent
    anchor: no bound, no safety speed.

    **Session check** (re-review 6 P1: a random start seq still meets an earlier session's consecutive seqs). No
    anchor is taken until a STATE shows ``last_rx_seq`` step from ``a`` to ``b`` for a pair this session wrote a random
    distance apart (``expect_step``, a step of 1024 to 64512) -- an earlier session's late STATEs step by one, or by a
    few where STATEs went missing or frames were refused; a step of 1024 or more needs 20 s of its frames between two
    of its STATEs, or its own session check's step -- matching this one's start and step: two random 16-bit values,
    about 2^-32 (Codex re-review 7 P3). The host accepts STATEs in increasing MCU time only, so once
    such a STATE is accepted no earlier one can follow it: everything after it is this session's. A write is forgotten after ``anchor_max_age_s``, long before a 16-bit seq comes
    round (22 min at 50 Hz)."""

    def __init__(self, *, anchor_max_age_s: float = 2.0, drift_max: float = 100e-6, anchors_max: int = 32) -> None:
        self.anchor_max_age_ns = int(anchor_max_age_s * 1e9)
        self.drift_max = float(drift_max)
        self.anchors_max = int(anchors_max)
        self._writes: dict[int, int] = {}  # seq -> write host ns (this session's frames)
        self._anchors: list[tuple[int, int, int, int, int]] = []  # (offset_ns, receipt_ns, mcu_ns, bound_ns, boot)
        self._prev: tuple[int, int] | None = None  # the previous accepted STATE: (boot, last_rx_seq)
        self._pending: tuple[int, int, int] | None = None  # an acknowledgement seen: (write ns, its STATE's MCU ns, boot)
        self._step: tuple[int, int] | None = None  # the session check's (a, b)
        self.verified_boot: int | None = None  # the boot the session check passed on

    def written(self, seq: int, write_ns: int) -> None:
        self._writes[int(seq) & 0xFFFF] = int(write_ns)
        if len(self._writes) > 4 * self.anchors_max:
            cut = write_ns - self.anchor_max_age_ns
            self._writes = {q: t for q, t in self._writes.items() if t >= cut}

    def restart_check(self) -> None:
        """Forget the session check and every anchor (a lost link)."""
        self.verified_boot, self._step, self._pending, self._prev = None, None, None, None
        self._anchors = []

    def expect_step(self, a: int, b: int) -> None:
        self._step = (int(a) & 0xFFFF, int(b) & 0xFFFF)

    @property
    def verified(self) -> bool:
        return self.verified_boot is not None

    def add(self, sample: Sample) -> None:
        st = sample.state
        seq = int(st.last_rx_seq) & 0xFFFF
        mcu_ns = st.mcu_time_us * 1000
        rx = sample.host_ns
        prev, self._prev = self._prev, (st.boot_id, seq)
        if prev is None or prev[0] != st.boot_id:
            self._anchors = [a for a in self._anchors if a[4] == st.boot_id]
            self._pending = None
            if self.verified_boot != st.boot_id:
                self.verified_boot = None  # a new boot: check the session again
            return  # nothing seen before on this boot: no acknowledgement can be told from the initial value
        if self.verified_boot != st.boot_id:
            if self._step is not None and (prev[1], seq) == self._step and seq in self._writes:
                self.verified_boot = st.boot_id  # this STATE and every later one are this session's
            else:
                return
        # a STATE after an acknowledging one: measured after the acknowledged frame's handling -> an anchor
        p = self._pending
        if p is not None and p[2] == st.boot_id and mcu_ns > p[1] and rx >= p[0] \
                and rx - p[0] <= self.anchor_max_age_ns:
            self._pending = None
            self._anchors.append((rx - mcu_ns, rx, mcu_ns, rx - p[0], st.boot_id))
            cut = rx - self.anchor_max_age_ns
            self._anchors = [a for a in self._anchors if a[1] >= cut and a[4] == st.boot_id][-self.anchors_max:]
        if prev[1] != seq:
            w = self._writes.pop(seq, None)
            if w is not None and rx >= w:
                self._pending = (w, mcu_ns, st.boot_id)

    def delay_s(self, sample: Sample) -> float | None:
        mcu_ns = sample.state.mcu_time_us * 1000
        off = sample.host_ns - mcu_ns
        k = self.drift_max / (1.0 - self.drift_max)
        best = None
        for a_off, a_rx, a_mcu, a_b, boot in self._anchors:
            if boot != sample.state.boot_id or abs(sample.host_ns - a_rx) > self.anchor_max_age_ns:
                continue
            b = a_b + (off - a_off) + k * abs(mcu_ns - a_mcu)
            best = b if best is None else min(best, b)
        return None if best is None else max(0.0, best / 1e9)


STILL_WINDOW_S = 0.2  # a stand before an MCU ARM: counts still over at least this much of contiguous STATEs ...
STILL_COUNTS = 2  # ... within this many counts per wheel


def counts_still(samples: Sequence[Sample], p: EncoderSpeedParams, *, window_s: float = STILL_WINDOW_S,
                 tol_counts: int = STILL_COUNTS) -> bool:
    """A measured stand (design 4): the newest STATEs of one boot, contiguous (no gap over ``dt_max_s``) and fault
    free, cover ``window_s`` of MCU time, and no wheel's count moved more than ``tol_counts`` over them. Not the
    encoder bound: its acceleration term (premises A1, A2) keeps it above any stand threshold by construction."""
    if not samples:
        return False
    last = samples[-1].state
    span = 0.0
    for a, b in zip(reversed(samples[:-1]), reversed(samples[1:])):
        sa, sb = a.state, b.state
        if sa.boot_id != last.boot_id or (sa.fault_flags | sb.fault_flags) & _SPEED_BITS:
            return False
        dt = (sb.mcu_time_us - sa.mcu_time_us) / 1e6
        if not 0.0 < dt <= p.dt_max_s:
            return False
        for c_last, c_a in ((last.wheel_count_left, sa.wheel_count_left),
                            (last.wheel_count_right, sa.wheel_count_right)):
            if abs(wheel_count_delta(c_last, c_a)) > tol_counts:
                return False
        span += dt
        if span >= window_s:
            return True
    return False


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


def output_direction_check(limit_fn, speed: SafetySpeed, requested_mps: float, output_mps: float,
                           commanded_mps: float) -> tuple[float, str | None]:
    """Design 8 on this cycle's output: the driving direction is the operator's request (before the slew -- a slew
    still on the old command's side must not take the residual motion out of the unasked check, Codex round 3 P1),
    none when nothing is asked; the output passes only when ``direction_limits`` neither holds nor limits it below
    its magnitude. Returns (speed, why cut or None)."""
    direction = None if requested_mps == 0.0 else (1 if requested_mps > 0 else -1)
    allowed, why, hold, _drv, _uns = direction_limits(limit_fn, speed, direction, abs(output_mps), commanded_mps)
    if hold or allowed < abs(output_mps):
        return 0.0, why
    return output_mps, None


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
    keys = {"axle_accel_mps2", "max_age_s", "dt_min_s", "dt_max_s"}
    if not isinstance(es, dict) or set(es) != keys | {"provenance"}:
        raise ValueError(f"encoder_speed must hold {sorted(keys)} and provenance")
    prov = es["provenance"]
    if not isinstance(prov, dict) or not {"source", "measured"} <= set(prov):
        raise ValueError("encoder_speed.provenance needs source and measured")
    return EncoderSpeedParams(radius_m=tuple(vehicle.wheel_radius_m), counts_per_rev=tuple(vehicle.counts_per_wheel_rev),
                              track_m=vehicle.track_m, kappa_max=max(abs(k) for k in vehicle.steering.curvature_range),
                              **{k: float(es[k]) for k in keys})


# ------------------------------------------------------------------------------------- the bridge's cycle (design 4-6)
STEP_MIN = 1024  # the session check's step: an earlier session's ACKs only step this far after 20 s of missed frames
STEP_RETRY_NS = 1_000_000_000  # a session check not passed within this is started again
MIN_LIFE_NS = 1_000_000  # a command with less life left than the link's 1 ms validity step is not followed
OUT_LIFE_NS = 100_000_000  # a DRIVE's life from its decision, before the console's own deadlines cut it
STOPPING_WAIT_NS = 1_000_000_000  # an MCU-originated STOPPING runs out before the DISARM, for at most this
IDLE, ARMING, READY, DISARMING, HELD = "idle", "arming", "ready", "disarming", "held"

# --------------------------------------------------------------- the wall escape (standing-direction-rule design 4)
ESCAPE_MPS = 0.05  # the escape's command cap (the physical speed is watched per wheel)
ESCAPE_WHEEL_MARGIN_MPS = 0.02  # a wheel over ESCAPE_MPS + this is a STOP
ESCAPE_DISTANCE_M = 0.20  # the operating budget: per wheel cumulative absolute travel, to the stand
ESCAPE_TIME_S = 4.0
ESCAPE_BRAKE_MIN_MPS2 = 0.3  # b_min: provisional, set by S5's stop measurement (plan's provisional table)
ESCAPE_REVERSE_COUNTS = 3  # back from a wheel's best progress; re-set only from S6's count jitter (with a mm cap)
ESCAPE_YAW_RAD_S = 0.02
ESCAPE_SETTLE_S = 5.0  # the travel after an end is followed this long at most for the record


def escape_stop_reach_m(travel_m: float, reading: EncoderReading, p: EncoderSpeedParams, *, cycle_s: float) -> float:
    """Design 4.3 (5th ed.): the travel so far plus the farthest the truck may still go once a stop is decided --
    ``v T + A T^2 / 2 + (v + A T)^2 / (2 b_min)``, with ``v = max_w(|v_w| + e_w)`` (the safety bound, never signed),
    ``A`` premise A1's wheel acceleration and ``T`` the reading's age (its delay included), a cycle and a DRIVE's
    life."""
    if reading.wheel_mps is not None and reading.wheel_err is not None:
        v = max(abs(w) + e for w, e in zip(reading.wheel_mps, reading.wheel_err))
    else:
        v = reading.speed.bound_mps
    a = p.wheel_accel_mps2
    t = (reading.age_s or 0.0) + cycle_s + OUT_LIFE_NS / 1e9
    return travel_m + v * t + 0.5 * a * t * t + (v + a * t) ** 2 / (2.0 * ESCAPE_BRAKE_MIN_MPS2)


class TeleopBridge:
    """One cycle of the S6 bridge (design 5, step 2): read the encoder speed, run the arbiter through ``link``, keep
    the MCU authorization paired with the arbiter's ``arm_gen`` and put this cycle's DRIVE (or none) on the port.
    The caller receives before and transmits after (design 5, steps 1 and 3), and feeds every STATE received to
    ``samples`` (a log tap on the host, as the bring-up tools do).

    ``floor_permit(curvatures, direction, check_speed) -> (limit, reason)`` is the obstacle layer's (the CLI's has no
    scan source: always 0); ``safety_provider(now_mono) -> dict | None`` the safety picture (``bench_safety`` here,
    the mission core's ``safety_state`` in 4e-2) -- the real truck's refusals are put on it here (``enforce_real``)."""

    def __init__(self, *, link, host, port, kinematics, params: EncoderSpeedParams, limit_mps: float, floor_permit,
                 safety_provider, cycle_s: float = 0.02, samples_max: int = 64) -> None:
        self.link, self.arbiter = link, link.a
        self.host, self.port, self.kin, self.params = host, port, kinematics, params
        self.limit_mps = float(limit_mps)
        self.floor_permit, self.safety_provider = floor_permit, safety_provider
        self.cycle_s = float(cycle_s)
        self.samples: list[Sample] = []
        self.samples_max = int(samples_max)
        self.delays = DelayBound()
        self._newest_mcu: dict[int, int] = {}
        self.dropped_old = 0
        self._disconnects_seen = 0
        self._step_b: int | None = None
        self._step_since = 0
        self.last_ping_ns: int | None = None
        self.phase = IDLE
        self.bound_gen: int | None = None  # the arbiter arm_gen the MCU authorization belongs to
        self.used_gens: set[int] = set()  # arm_gens an MCU ARM was asked for (once each, never again)
        self.seen_control_gen = self.arbiter.control_gen
        self.q_cmd: float | None = None  # the steering command the slew runs from
        self.holding = True  # the steering is held (not following the teleop curvature)
        self.last_cmd: tuple[float, int] = (0.0, 0)  # (speed, expiry ns) of the last DRIVE queued
        self.mcu_wait: dict | None = None  # an MCU-originated STOPPING: {"control_gen", "until_ns"}
        self._urgent = False  # this cycle took a higher-priority stop than an MCU STOPPING
        self.reading: EncoderReading | None = None
        self.records: list[dict] = []
        # the wall escape (standing-direction-rule design 4): the current offer, the approval waiting to start, the
        # escape running, every step's record; the arbiter takes escape_approve only through this handler
        self.offer: dict | None = None
        self._approved: int | None = None
        self.escape: dict | None = None
        self.escape_log: list[dict] = []
        self._events_out: list[dict] = []  # logged since the last cycle record (the JSONL and the console's events)
        self._tail: dict | None = None  # an ended escape's travel followed to the stand or the hand-over
        self._offer_ids = itertools.count(secrets.randbelow(1 << 20) + 1)
        self.arbiter.escape = self.approve_escape

    # ------------------------------------------------------------------ inputs
    def add_sample(self, sample: Sample) -> None:
        """A STATE the host accepted. The host forgets its last STATE on a disconnect, so after a reconnect it can
        accept an older one; the bridge keeps each boot's newest MCU time across reconnects and drops anything not
        newer (Codex re-review 7 P2: a 200 ms regression read as fresh) -- the session check's order argument rests
        on it."""
        st = sample.state
        newest = self._newest_mcu.get(st.boot_id)
        if newest is not None and st.mcu_time_us <= newest:
            self.dropped_old += 1
            return
        self._newest_mcu[st.boot_id] = st.mcu_time_us
        self.delays.add(sample)
        self.samples.append(sample)

    def frame_written(self, seq: int, write_ns: int) -> None:
        """Every frame the link wrote (the host's log): the round-trip anchors' start times."""
        self.delays.written(seq, write_ns)
        if len(self.samples) > self.samples_max:
            del self.samples[: len(self.samples) - self.samples_max]

    def _steering_ok(self, q: float) -> bool:
        return self.kin.cfg.steering.curvature(q) is not None

    def _deadline_ns(self, now_ns: int, teleop_cut: bool) -> int:
        """Design 6: decision + 100 ms, the teleop command's deadline (nonzero) and the lease's."""
        a = self.arbiter
        ends = [now_ns + OUT_LIFE_NS]
        if teleop_cut and a.teleop is not None and a.teleop.get("deadline") is not None:
            ends.append(int(a.teleop["deadline"] * 1e9))
        if a.lease is not None:
            ends.append(int((a.lease.renewed_mono + a.config.lease_timeout_s) * 1e9))
        return min(ends)

    def _commanded(self, now_ns: int) -> float:
        speed, expires = self.last_cmd
        return speed if now_ns < expires else 0.0

    # ------------------------------------------------------------------ the cycle
    def cycle(self, now_ns: int) -> dict:
        a, now = self.arbiter, now_ns / 1e9
        rec: dict = {"now_ns": now_ns}
        if self.host.disconnects != self._disconnects_seen:
            self._disconnects_seen = self.host.disconnects
            self.delays.restart_check()  # a lost link: check the session again before any anchor
            self._step_b = None
        delay = self.delays.delay_s(self.samples[-1]) if self.samples else None
        reading = encoder_speed(self.samples, now_ns, self.params, self._steering_ok, delay)
        rec["delay_s"] = delay
        self.reading = reading
        st = self.host.state
        was_driving = a.mode == "TELEOP" and a.armed  # before observe can disarm it (design 4)
        lift = st.lift_m if st is not None and math.isfinite(st.lift_m) else None
        feedback = None
        if reading.speed.valid:
            feedback = Feedback(self.samples[-1].host_ns / 1e9, reading.speed.signed_mps, reading.yaw_rate or 0.0,
                                lift_m=lift, steering_rad=abs(reading.steering_rad))
        gen_before = a.control_gen
        self.link.gate(now)
        self.link.begin(now, feedback)
        safety = enforce_real(self.safety_provider(now))
        drive = None
        if reading.speed.valid:
            q = reading.steering_rad
            kappa = self.kin.held_curvature((q,))
            target = self.kin.steering_targets(a.drive.kappa)
            drive = {"measured_mps": reading.speed.signed_mps, "measured_kappa": 0.0 if kappa is None else kappa,
                     "steering_error_rad": math.inf if target is None else abs(target[0] - q),
                     "permit": self.floor_permit}
        out = self.link.end(now, [], safety, (0.0, 0.0), drive)
        # a boundary the arbiter took this cycle (an operator STOP, a lease or feedback loss, a disarm): it outranks
        # an MCU STOPPING first seen in the same cycle (Codex impl P1-3)
        self._urgent = a.control_gen != gen_before
        rec.update(mode=a.mode, armed=a.armed, arm_gen=a.arm_gen, target_mps=out.speed_mps,
                   target_kappa=out.curvature_inv_m, policy=out.policy.reason,
                   speed=None if not reading.speed.valid else
                   {"signed": reading.speed.signed_mps, "bound": reading.speed.bound_mps,
                    "dirs": list(reading.speed.directions)},
                   speed_invalid=reading.reason)
        # design 3/4: an invalid speed while driving, or a bound over the limit, is a STOP (not only the latch)
        if was_driving and not reading.speed.valid:
            self._stop(now_ns, "speed_invalid", rec)
            self._urgent = True
        elif a.armed and reading.speed.valid and reading.speed.bound_mps > self.limit_mps:
            self._stop(now_ns, "overspeed", rec)
            self._urgent = True
        if a.control_gen != self.seen_control_gen:
            self.seen_control_gen = a.control_gen
            self.port.new_control_generation()  # queued DRIVEs go before this cycle's transmit
        self._arm_machine(now_ns, reading, rec)
        self._ping(now_ns, rec)
        applied = self._output(now_ns, out, reading, rec)
        if self._tail is not None and self.escape is None:
            self._tail_step(now_ns, applied, rec)
        if self._events_out:
            rec["escape_events"], self._events_out = self._events_out, []
        a.last_applied = applied  # what was put on the port, not the arbiter's target (Codex P1-3)
        rec["applied_mps"] = applied
        rec["phase"] = self.phase
        q = self.q_cmd
        rec["applied"] = {"speed_mps": applied, "steering_rad": None if q is None else [q],
                          "curvature_inv_m": None if q is None else self.kin.held_curvature((q,))}
        self.records.append(rec)
        # the state after every stop this cycle took (Codex impl P3): the console reads the final mode and gen
        stopped = a.mode == "STOPPED" or a.preempt_latch is not None
        rec.update(mode=a.mode, armed=a.armed)
        rec["_out"] = replace(out, mode=a.mode, armed=a.armed, control_gen=a.control_gen, speed_mps=applied,
                              reason=a.preempt_latch,
                              **({"stop_kind": "preempt", "source": "zero"} if stopped else {}))
        return rec

    # ------------------------------------------------------------------ design 4
    def _want(self) -> bool:
        a = self.arbiter
        return a.mode == "TELEOP" and a.armed and a.preempt_latch is None

    def _disarm(self, now_ns: int, why: str, rec: dict) -> None:
        self.host.drop_drives(why)
        self.host.request_disarm(now_ns)
        self.phase, self.bound_gen = DISARMING, None
        self.last_cmd = (0.0, 0)
        rec.setdefault("disarm", why)

    def _stop(self, now_ns: int, why: str, rec: dict) -> None:
        self.arbiter.stop(now_ns / 1e9, why)
        self.seen_control_gen = self.arbiter.control_gen  # our own boundary: the port's generation follows below
        self.port.new_control_generation()
        rec.setdefault("stop", why)

    def _arm_machine(self, now_ns: int, reading: EncoderReading, rec: dict) -> None:
        a, h = self.arbiter, self.host
        st = h.state
        mcu_mode = None if st is None else st.mode
        # an MCU-originated STOPPING being let run out (design 4, DISARM priority 2)
        if self.mcu_wait is not None:
            operator_event = a.control_gen != self.mcu_wait["control_gen"]
            if mcu_mode != Mode.STOPPING or operator_event or now_ns >= self.mcu_wait["until_ns"]:
                why = "mcu_stopping_over" if mcu_mode != Mode.STOPPING else (
                    "operator_during_mcu_stopping" if operator_event else "mcu_stopping_timeout")
                self.mcu_wait = None
                self._disarm(now_ns, why, rec)
            return
        if self.phase in (ARMING, READY) and not h.authorized and not h.arming:
            # the host's authorization ended on its own: MCU STOPPING / FAULT / ESTOP / boot / link / build
            reason = "link" if mcu_mode is None or not h.health(now_ns).fresh else mcu_mode.name.lower()
            what = "mcu_arm_refused" if self.phase == ARMING else f"mcu_{reason}"
            self._stop(now_ns, what, rec)
            if mcu_mode == Mode.STOPPING and h.health(now_ns).fresh and not self._urgent:
                self.mcu_wait = {"control_gen": a.control_gen, "until_ns": now_ns + STOPPING_WAIT_NS}
                self.phase, self.bound_gen = HELD, None
                self.host.drop_drives("mcu_stopping")
                return
            self._disarm(now_ns, what, rec)
            return
        want, g = self._want(), a.arm_gen
        if self.phase in (ARMING, READY) and (not want or g != self.bound_gen):
            self._disarm(now_ns, "arbiter_disarmed" if not want else "new_arm_gen", rec)
            return
        if self.phase == ARMING and h.ready:
            self.phase = READY
            self.q_cmd, self.holding = reading.steering_rad, True
            rec["ready"] = g
        if self.phase in (DISARMING, HELD):
            if st is not None and h.health(now_ns).fresh and mcu_mode in (Mode.DISARMED, Mode.FAULT, Mode.ESTOP) \
                    and not h.authorized:
                self.phase = IDLE
        if self.phase == IDLE and want and g not in self.used_gens:
            why = self._arm_blocker(now_ns, reading)
            rec["arm_wait"] = why
            if why is None:
                rec["arm_still"] = counts_still(self.samples, self.params)
                self.used_gens.add(g)
                blocked = h.request_arm(now_ns)
                if blocked is not None:
                    self._stop(now_ns, f"mcu_arm_refused:{blocked.reason}", rec)
                    self.phase = HELD
                else:
                    self.phase, self.bound_gen = ARMING, g
                    rec["arm"] = g

    PING_NS = 500_000_000
    PING_FAST_NS = 100_000_000

    def _ping(self, now_ns: int, rec: dict) -> None:
        """While the MCU is not armed and nothing else goes out, a DISARM every 0.5 s: it changes nothing on a
        disarmed MCU but is acknowledged (``last_rx_seq``), which keeps a round-trip delay anchor -- so the safety
        speed (the stand check, the arbiter's feedback) stays measurable when no DRIVE flows."""
        h, st = self.host, self.host.state
        if self.phase not in (IDLE, HELD) or self.mcu_wait is not None or h.authorized or h.arming:
            return
        if st is None or st.mode == Mode.STOPPING:
            return
        # faster while there is no delay bound at all (start, a new boot): the first acknowledgement needs a seq that
        # moves, and seq 0 cannot be told from the MCU's initial value
        every = self.PING_NS if self.samples and self.delays.delay_s(self.samples[-1]) is not None \
            else self.PING_FAST_NS
        if self.last_ping_ns is not None and now_ns - self.last_ping_ns < every:
            return
        if not self.delays.verified and self._step_b is None:
            # the session check (DelayBound): this ping is ``a``; the next one goes out a random distance after it
            a = h._seq
            h.request_disarm(now_ns)
            gap = STEP_MIN + secrets.randbelow(0x10000 - 2 * STEP_MIN)
            b = h.skip_seq(gap - 1)
            self.delays.expect_step(a, b)
            self._step_b, self._step_since = b, now_ns
        else:
            h.request_disarm(now_ns)
            if self._step_b is not None and (self.delays.verified or now_ns - self._step_since > STEP_RETRY_NS):
                self._step_b = None  # passed, or lost (a frame dropped): the next check starts afresh
        self.last_ping_ns = now_ns
        rec["ping"] = True

    def _arm_blocker(self, now_ns: int, reading: EncoderReading) -> str | None:
        """Design 4: every MCU ARM waits for a DISARMED MCU, no older nonzero DRIVE that could act, and a fresh
        confirmed stand (the encoder bound and the arbiter's stop detector)."""
        st = self.host.state
        if st is None or not self.host.health(now_ns).fresh:
            return "no_state"
        if st.mode != Mode.DISARMED:
            if self.host.authorized:
                return "mcu_armed"
            return f"mcu_{st.mode.name.lower()}"
        if self.port.older_nonzero_may_act(now_ns):
            return "older_drive_may_act"
        if not (reading.speed.valid and counts_still(self.samples, self.params) and self.arbiter.stop_confirmed):
            return "not_standing"
        return None

    # ------------------------------------------------------------------ design 6
    def _output(self, now_ns: int, out, reading: EncoderReading, rec: dict) -> float:
        a = self.arbiter
        if self.escape is not None and self._escape_lost(now_ns, rec):
            return 0.0
        if self.phase != READY or not self.host.ready:
            self._drop_offer()
            return 0.0  # arming: the host owns the zero; STOPPING/unauthorized: observe only
        q_meas = reading.steering_rad
        # a teleop command counts until its deadline, not at it (Codex re-review P2: the arbiter expires only past
        # the deadline -- at it, the old steering target must not be renewed)
        active = a.teleop is not None and int(a.teleop["deadline"] * 1e9) - now_ns >= MIN_LIFE_NS
        follow = (out.source == "teleop" and out.policy.drive_allowed and a.armed and a.preempt_latch is None
                  and active and bool(a.teleop.get("deadman")) and reading.speed.valid)
        requested = a.drive.target(a.teleop, loaded=out.policy.loaded)[0] if a.teleop is not None else 0.0
        if self.escape is not None:
            return self._escape_output(now_ns, out, reading, rec, follow=follow, active=active, requested=requested)
        if not follow:
            self._drop_offer()
            if not self.holding and q_meas is not None:
                self.q_cmd = q_meas  # entering the hold: the steering stays where it is measured
            self.holding = True
        else:
            self.holding = False
        if self.q_cmd is None:
            if q_meas is None:
                self._stop(now_ns, "steering_invalid", rec)
                self._disarm(now_ns, "steering_invalid", rec)
                return 0.0
            self.q_cmd = q_meas
        speed = float(out.speed_mps) if follow else 0.0
        q_cmd = [self.q_cmd]
        if follow:
            target = self.kin.steering_targets(out.curvature_inv_m)
            if target is None:
                self._stop(now_ns, "steering_invalid", rec)
                self._disarm(now_ns, "steering_invalid", rec)
                return 0.0
            arr = np.array([self.q_cmd], dtype=float)
            self.kin.slew_steering(arr, np.array([target[0]], dtype=float), self.cycle_s)
            self.q_cmd = float(arr[0])
            q_cmd = [self.q_cmd]
        if speed != 0.0:
            # the bundle TeleopDrive checks (measured curvature to the output's, with the margin), widened to the
            # curvature the submitted steering holds (Codex impl P1-2)
            k_meas = self.kin.held_curvature((q_meas,)) if q_meas is not None else None
            k_cmd = self.kin.held_curvature(q_cmd)
            ends = [k for k in (k_meas, k_cmd, float(out.curvature_inv_m)) if k is not None]
            curvs = bundle(min(ends), max(ends), a.config.teleop_limits.curvature_margin_inv_m)
            limit_fn = lambda d, v: self.floor_permit(curvs, d, v)  # noqa: E731
            commanded = self._commanded(now_ns)
            out_speed = speed
            speed, why = output_direction_check(limit_fn, reading.speed, requested, speed, commanded)
            if why is not None:
                rec["direction_cut"] = why
            started = self._offer_or_start(now_ns, reading, rec, limit_fn, requested, out_speed, commanded, why)
            if started:
                return self._escape_output(now_ns, out, reading, rec, follow=follow, active=active,
                                           requested=requested)
        else:
            self._drop_offer()
        return self._submit(now_ns, speed, q_cmd, q_meas, follow, rec)

    def _submit(self, now_ns: int, speed: float, q_cmd: list, q_meas, follow: bool, rec: dict,
                deadline_ns: int | None = None) -> float:
        if speed == 0.0:
            drive = self.kin.stopped(q_cmd)
        else:
            raw = self.kin.command(speed, self.kin.held_curvature(q_cmd) or 0.0)
            if isinstance(raw, DriveRefusal):
                self._stop(now_ns, f"port_refused:{raw.reason}", rec)
                self._disarm(now_ns, "port_refused", rec)
                return 0.0
            drive = self.kin.finalize(raw, q_cmd)
            speed = drive.speed_mps
        expires = self._deadline_ns(now_ns, follow)  # an active teleop command, wheels or steering only
        if deadline_ns is not None:
            expires = min(expires, deadline_ns)  # the escape's end (design 4.3)
        if expires <= now_ns:
            # the lease's end has come: nothing renews the old target -- the steering held where it is measured
            if q_meas is not None and deadline_ns is None:
                self.q_cmd, self.holding = q_meas, True
            q_cmd = [self.q_cmd]
            speed, drive = 0.0, self.kin.stopped(q_cmd)
            expires = now_ns + OUT_LIFE_NS if self.arbiter.lease is None else self._deadline_ns(now_ns, False)
            if expires <= now_ns:
                return 0.0  # no lease life left: no frame at all (the MCU's own expiry stops it)
        cmd = self.port.make_command(now_ns, expires, drive, q_cmd, 0.0, "keep")
        res = self.port.command(cmd, now_ns)
        rec["port"] = res.kind if res.reason is None else f"{res.kind}:{res.reason}"
        if res.kind != "queued":
            self._stop(now_ns, f"port_{res.kind}:{res.reason}", rec)
            self._disarm(now_ns, "port_not_queued", rec)
            return 0.0
        if speed != 0.0:
            self.last_cmd = (speed, expires)  # a nonzero DRIVE may act until its expiry, whatever follows it
            if self.escape is not None:
                rec.setdefault("escape", {})["expires_ns"] = expires
        return speed

    # ------------------------------------------------------- the wall escape (standing-direction-rule design 4)
    def _log_escape(self, now_ns: int, event: str, **fields) -> None:
        entry = {"now_ns": now_ns, "event": event, **fields}
        self.escape_log.append(entry)
        self._events_out.append(entry)  # the cycle's record carries it (Codex impl review P2-2)

    def _follow(self, track: dict, d: int | None = None) -> str | None:
        """Every STATE since ``track["last_t"]``: each wheel's cumulative absolute travel and, while escaping (``d``),
        the count reverse against the best progress and the steering on every STATE (Codex impl review P2-1)."""
        for smp in self.samples:
            st = smp.state
            if st.mcu_time_us <= track["last_t"]:
                continue
            if st.boot_id != track["boot"]:
                return "boot"
            c = (st.wheel_count_left, st.wheel_count_right)
            why = None
            for w in (0, 1):
                step = wheel_count_delta(c[w], track["last_c"][w])
                track["travel"][w] += abs(step) * 2.0 * math.pi * self.params.radius_m[w] / self.params.counts_per_rev[w]
                if d is not None:
                    prog = wheel_count_delta(c[w], track["c0"][w]) * d
                    track["best"][w] = max(track["best"][w], prog)
                    if track["best"][w] - prog > ESCAPE_REVERSE_COUNTS:
                        why = why or "wheel_reverse"
            track["last_c"], track["last_t"] = c, st.mcu_time_us
            if d is not None and why is None:
                q = float(st.steering_rad)
                if not (math.isfinite(q) and abs(q) <= self.kin.straight_threshold_rad):
                    why = "steering"
            if why is not None:
                return why
        return None

    def _tail_step(self, now_ns: int, applied: float, rec: dict) -> None:
        """Design 4.3's record after an end: the travel from the escape's start to the stand, or to the general rule
        driving again (the hand-over), or for ESCAPE_SETTLE_S at most."""
        t = self._tail
        if self._follow(t) == "boot":
            event = "settle_lost"
        elif applied != 0.0:
            event = "handed_over"
        elif counts_still(self.samples, self.params):
            event = "settled"
        elif now_ns - t["end_ns"] > ESCAPE_SETTLE_S * 1e9:
            event = "settle_timeout"
        else:
            return
        self._tail = None
        self._log_escape(now_ns, event, offer_id=t["offer_id"], travel_m=[round(x, 5) for x in t["travel"]],
                         after_end_s=round((now_ns - t["end_ns"]) / 1e9, 3))

    def _drop_offer(self) -> None:
        self.offer, self._approved = None, None

    def offer_view(self) -> dict | None:
        """The state message's ``escape_offer`` (design 4.1)."""
        o = self.offer
        if o is None or self.escape is not None:
            return None
        return {k: o[k] for k in ("offer_id", "direction", "blocked", "reason")}

    def escape_view(self) -> dict | None:
        """The state message's ``escape``: the escape running."""
        e = self.escape
        if e is None:
            return None
        return {"offer_id": e["offer_id"], "direction": e["d"], "travel_m": [round(t, 4) for t in e["travel"]],
                "elapsed_s": round((e["last_ns"] - e["start_ns"]) / 1e9, 3), "budget_m": ESCAPE_DISTANCE_M,
                "time_s": ESCAPE_TIME_S}

    def approve_escape(self, msg: dict, now_mono: float) -> str | None:
        """The arbiter's ``escape_approve`` handler (design 4.2): the current offer, of its generations, once."""
        now_ns = int(now_mono * 1e9)
        why = None
        o = self.offer
        if self.escape is not None:
            why = "escape_active"
        elif o is None or msg["offer_id"] != o["offer_id"]:
            why = "escape_no_offer"
        elif msg["control_gen"] != o["control_gen"] or msg["arm_gen"] != o["arm_gen"]:
            why = "escape_stale"
        if why is not None:
            self._log_escape(now_ns, "refused", offer_id=msg["offer_id"], reason=why)
            return why
        self._approved = o["offer_id"]
        self._log_escape(now_ns, "approved", offer_id=o["offer_id"])
        return None

    def _offer_or_start(self, now_ns, reading, rec, limit_fn, requested, out_speed, commanded, cut) -> bool:
        """Design 4.1: an offer while the requested direction's own permit allows the output and only the unasked
        direction holds it, at a fresh stand with nothing nonzero in effect and the steering straight; an approval
        of the offer starts the escape (returns True)."""
        a = self.arbiter
        d = 0 if requested == 0.0 else (1 if requested > 0 else -1)
        ok = False
        if cut is not None and d != 0 and (out_speed > 0) == (d > 0):
            _al, why, hold, _drv, _uns = direction_limits(limit_fn, reading.speed, d, abs(out_speed), commanded)
            own, _w = limit_fn(d, abs(out_speed))
            q = reading.steering_rad
            ok = (hold and own >= abs(out_speed) and counts_still(self.samples, self.params) and a.stop_confirmed
                  and not self.port.older_nonzero_may_act(now_ns) and commanded == 0.0
                  and q is not None and abs(q) <= self.kin.straight_threshold_rad)
        if not ok:
            self._drop_offer()
            return False
        o = self.offer
        if o is None or (o["direction"], o["control_gen"], o["arm_gen"]) != (d, a.control_gen, a.arm_gen):
            self._approved = None
            o = self.offer = {"offer_id": next(self._offer_ids), "direction": d, "blocked": -d, "reason": why,
                              "control_gen": a.control_gen, "arm_gen": a.arm_gen}
            self._log_escape(now_ns, "offer", **{k: o[k] for k in ("offer_id", "direction", "blocked", "reason")})
        rec["escape_offer"] = o["offer_id"]
        if self._approved != o["offer_id"]:
            return False
        last = self.samples[-1].state
        self.escape = {"offer_id": o["offer_id"], "d": d, "q": float(reading.steering_rad), "start_ns": now_ns,
                       "last_ns": now_ns, "deadline_ns": now_ns + int(ESCAPE_TIME_S * 1e9),
                       "control_gen": a.control_gen, "arm_gen": a.arm_gen, "disconnects": self.host.disconnects,
                       "boot": last.boot_id, "c0": (last.wheel_count_left, last.wheel_count_right),
                       "last_c": (last.wheel_count_left, last.wheel_count_right), "last_t": last.mcu_time_us,
                       "best": [0, 0], "travel": [0.0, 0.0]}
        self._drop_offer()
        self.q_cmd, self.holding = self.escape["q"], True
        self._log_escape(now_ns, "start", offer_id=o["offer_id"], direction=d, steering_rad=self.escape["q"])
        return True

    def _escape_end(self, now_ns: int, reason: str, rec: dict, *, stop: bool) -> None:
        e = self.escape
        self.escape = None
        self._drop_offer()
        self.host.drop_drives(f"escape_{reason}")
        self._log_escape(now_ns, "end", offer_id=e["offer_id"], reason=reason, stop=stop,
                         travel_m=[round(t, 5) for t in e["travel"]],
                         elapsed_s=round((now_ns - e["start_ns"]) / 1e9, 3), steering_rad=e["q"])
        rec["escape_end"] = reason
        self._tail = {"offer_id": e["offer_id"], "boot": e["boot"], "last_c": e["last_c"], "last_t": e["last_t"],
                      "travel": list(e["travel"]), "end_ns": now_ns}
        if stop:
            if self.arbiter.mode != "STOPPED":
                self._stop(now_ns, f"escape:{reason}", rec)
            if self.phase in (ARMING, READY):
                self._disarm(now_ns, f"escape_{reason}", rec)

    def _escape_lost(self, now_ns: int, rec: dict) -> bool:
        """Design 4.3: an operator STOP, a lease loss, a disarm, a generation change or a reconnect ends it at once."""
        a, e = self.arbiter, self.escape
        if (self.phase == READY and self.host.ready and a.control_gen == e["control_gen"]
                and a.arm_gen == e["arm_gen"] and self.host.disconnects == e["disconnects"]):
            return False
        self._escape_end(now_ns, "generation", rec, stop=True)
        return True

    def _escape_watch(self, reading: EncoderReading) -> str | None:
        """Every STATE since the last cycle (``_follow``: travel, count reverse, steering); then the yaw and per-wheel
        overspeed on the reading (design 4.3)."""
        why = self._follow(self.escape, self.escape["d"])
        if why is not None:
            return why
        if reading.yaw_rate is not None and abs(reading.yaw_rate) > ESCAPE_YAW_RAD_S:
            return "yaw"
        if reading.wheel_mps is not None and max(abs(v) for v in reading.wheel_mps) > \
                ESCAPE_MPS + ESCAPE_WHEEL_MARGIN_MPS:
            return "wheel_overspeed"
        return None

    def _escape_output(self, now_ns, out, reading, rec, *, follow, active, requested) -> float:
        """Design 4.3: straight in the approved direction at ESCAPE_MPS at most, the requested direction's permit
        as usual; the immediate stops, then the normal ends (distance by the measured formula, time, permit)."""
        a, e = self.arbiter, self.escape
        e["last_ns"] = now_ns
        d = e["d"]
        rec["escape"] = {"offer_id": e["offer_id"], "deadline_ns": e["deadline_ns"]}
        why = None
        if not reading.speed.valid:
            why = "speed_invalid"
        elif a.teleop is None or not active or not a.teleop.get("deadman"):
            why = "deadman"
        elif requested == 0.0 or (1 if requested > 0 else -1) != d:
            why = "request"
        elif not follow:
            why = "policy"
        elif reading.steering_rad is None or abs(reading.steering_rad) > self.kin.straight_threshold_rad:
            why = "steering"
        else:
            why = self._escape_watch(reading)
        if why is not None:
            self._escape_end(now_ns, why, rec, stop=True)
            return 0.0
        q_cmd = [e["q"]]
        self.q_cmd, self.holding = e["q"], True
        speed = d * min(abs(float(out.speed_mps)), ESCAPE_MPS) if (out.speed_mps > 0) == (d > 0) else 0.0
        reach = escape_stop_reach_m(max(e["travel"]), reading, self.params, cycle_s=self.cycle_s)
        rec["escape"].update(travel_m=list(e["travel"]), reach_m=reach)
        end = None
        if now_ns >= e["deadline_ns"]:
            end = "time"
        elif reach >= ESCAPE_DISTANCE_M:
            end = "distance"
        elif speed == 0.0:
            end = "permit"  # the arbiter's own target went to zero (its permit) -- it was nonzero at the offer
        else:
            k = [k for k in (self.kin.held_curvature(q_cmd), self.kin.held_curvature((reading.steering_rad,)), 0.0)
                 if k is not None]
            curvs = bundle(min(k), max(k), a.config.teleop_limits.curvature_margin_inv_m)
            allowed, _w = self.floor_permit(curvs, d, abs(speed))
            if allowed < abs(speed):
                end = "permit"
        if end is not None:
            self._escape_end(now_ns, end, rec, stop=False)
            return self._submit(now_ns, 0.0, q_cmd, reading.steering_rad, False, rec)
        return self._submit(now_ns, speed, q_cmd, reading.steering_rad, True, rec, deadline_ns=e["deadline_ns"])


__all__ = ["CONFIRMATIONS", "EncoderReading", "EncoderSpeedParams", "STANDING_MPS", "bench_safety", "command_cap_mps",
           "direction_limits", "encoder_speed", "enforce_real", "load_bridge_params", "real_teleop_limits", "TeleopBridge", "counts_still", "DelayBound", "output_direction_check", "ESCAPE_MPS", "ESCAPE_DISTANCE_M",
           "ESCAPE_TIME_S", "ESCAPE_BRAKE_MIN_MPS2", "escape_stop_reach_m"]
