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
                  steering_ok: Callable[[float], bool], extra_delay_s: float = 0.0) -> EncoderReading:
    """The safety speed from the newest STATE and the latest earlier STATE of the same boot at least ``dt_min_s``
    before it (MCU time), at most ``dt_max_s``: per wheel ``v = 2 pi r dc / (CPR dt)``, the mean, the yaw rate from
    the wheel difference, the bound on the outer wheel, the directions from ``signed +- e`` (design 2)."""
    if not samples:
        return _invalid("no_state")
    cur = samples[-1]
    s1 = cur.state
    # the receipt age plus how much later than the link's best case this STATE arrived (design 2, A2 split): the
    # premise L covers only the base delay
    age = (now_ns - cur.host_ns) / 1e9 + max(0.0, float(extra_delay_s))
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


class DelayTracker:
    """Each STATE's delay over the link's best case (design 2): the offset ``host receipt - MCU time`` of a STATE
    minus the smallest offset seen this boot. The base never rises: one-way timing cannot tell a slow MCU clock from
    a delay that grew, and a rising base turned a sustained 80 ms delay fresh (Codex impl P1-1, re-review P1). A
    slow MCU clock therefore ages every STATE a little more as the boot goes on -- the fail-safe side: past
    ``max_age_s`` the speed is invalid and the truck stops; the real drift is measured in P0-9's long HIL run.
    The base delay itself is the premise L's."""

    def __init__(self) -> None:
        self._base: tuple[int, int] | None = None  # (offset_ns, boot)

    def add(self, sample: Sample) -> None:
        off = sample.host_ns - sample.state.mcu_time_us * 1000
        boot = sample.state.boot_id
        if self._base is None or self._base[1] != boot or off < self._base[0]:
            self._base = (off, boot)  # a new boot is a new MCU clock

    def excess_s(self, sample: Sample) -> float:
        if self._base is None or self._base[1] != sample.state.boot_id:
            return 0.0
        off = sample.host_ns - sample.state.mcu_time_us * 1000
        return max(0.0, (off - self._base[0]) / 1e9)


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
    keys = {"axle_accel_mps2", "state_latency_s", "max_age_s", "dt_min_s", "dt_max_s"}
    if not isinstance(es, dict) or set(es) != keys | {"provenance"}:
        raise ValueError(f"encoder_speed must hold {sorted(keys)} and provenance")
    prov = es["provenance"]
    if not isinstance(prov, dict) or not {"source", "measured"} <= set(prov):
        raise ValueError("encoder_speed.provenance needs source and measured")
    return EncoderSpeedParams(radius_m=tuple(vehicle.wheel_radius_m), counts_per_rev=tuple(vehicle.counts_per_wheel_rev),
                              track_m=vehicle.track_m, kappa_max=max(abs(k) for k in vehicle.steering.curvature_range),
                              **{k: float(es[k]) for k in keys})


# ------------------------------------------------------------------------------------- the bridge's cycle (design 4-6)
MIN_LIFE_NS = 1_000_000  # a command with less life left than the link's 1 ms validity step is not followed
OUT_LIFE_NS = 100_000_000  # a DRIVE's life from its decision, before the console's own deadlines cut it
STOPPING_WAIT_NS = 1_000_000_000  # an MCU-originated STOPPING runs out before the DISARM, for at most this
IDLE, ARMING, READY, DISARMING, HELD = "idle", "arming", "ready", "disarming", "held"


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
        self.delays = DelayTracker()
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

    # ------------------------------------------------------------------ inputs
    def add_sample(self, sample: Sample) -> None:
        self.delays.add(sample)
        self.samples.append(sample)
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
        extra = self.delays.excess_s(self.samples[-1]) if self.samples else 0.0
        reading = encoder_speed(self.samples, now_ns, self.params, self._steering_ok, extra)
        rec["late_s"] = extra
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
        applied = self._output(now_ns, out, reading, rec)
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
        if self.phase != READY or not self.host.ready:
            return 0.0  # arming: the host owns the zero; STOPPING/unauthorized: observe only
        q_meas = reading.steering_rad
        # a teleop command counts until its deadline, not at it (Codex re-review P2: the arbiter expires only past
        # the deadline -- at it, the old steering target must not be renewed)
        active = a.teleop is not None and int(a.teleop["deadline"] * 1e9) - now_ns >= MIN_LIFE_NS
        follow = (out.source == "teleop" and out.policy.drive_allowed and a.armed and a.preempt_latch is None
                  and active and bool(a.teleop.get("deadman")) and reading.speed.valid)
        if not follow:
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
            requested = a.drive.target(a.teleop, loaded=out.policy.loaded)[0] if a.teleop is not None else 0.0
            # the bundle TeleopDrive checks (measured curvature to the output's, with the margin), widened to the
            # curvature the submitted steering holds (Codex impl P1-2)
            k_meas = self.kin.held_curvature((q_meas,)) if q_meas is not None else None
            k_cmd = self.kin.held_curvature(q_cmd)
            ends = [k for k in (k_meas, k_cmd, float(out.curvature_inv_m)) if k is not None]
            curvs = bundle(min(ends), max(ends), a.config.teleop_limits.curvature_margin_inv_m)
            speed, why = output_direction_check(lambda d, v: self.floor_permit(curvs, d, v), reading.speed,
                                                requested, speed, self._commanded(now_ns))
            if why is not None:
                rec["direction_cut"] = why
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
        if expires <= now_ns:
            # the lease's end has come: nothing renews the old target -- the steering held where it is measured
            if q_meas is not None:
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
        return speed


__all__ = ["CONFIRMATIONS", "EncoderReading", "EncoderSpeedParams", "STANDING_MPS", "bench_safety", "command_cap_mps",
           "direction_limits", "encoder_speed", "enforce_real", "load_bridge_params", "real_teleop_limits", "TeleopBridge", "counts_still", "DelayTracker", "output_direction_check"]
