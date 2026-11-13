"""Open-loop drive profiles for the M1-M3 floor measurements (docs/design/2026-11-14-profile-drive-and-m-analysis.md 2,
G6).

A profile replaces only the operator's joystick: the runner rewrites the ``speed_ratio`` and ``curvature_ratio`` of each
``teleop`` message the console sends, and nothing else -- it never makes a message, so the deadman, the request
tokens, the lease, the command age, the arbiter, ``TeleopDrive``, the floor permit, the safety speed, the steering
slew and the MCU stop the truck as they do under the hand. What it asks for is a request; what was driven is in the
record (the bridge's applied command and the measured q), and a trial the gates reshaped is void.

Segments: ``presteer`` (speed 0 until the measured q sits at the target), ``hold`` (speed 0 for a time: the bias
windows), ``drive`` (a speed over an encoder distance with a curvature shape: constant, a sine merge, or a q step in
time). The encoder distance repeats the command; it is not the true distance.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import NamedTuple

SCHEMA = "drive_profile/1"
PRESTEER_TOL_RAD = 0.02  # the measured q within this of the target ...
PRESTEER_SETTLE_S = 0.5  # ... for this long ends a presteer
PRESTEER_TIMEOUT_S = 5.0
STILL_START_S = 1.0  # a trial starts only after this long standing (encoder speed under the stop speed)
RAMP_SLACK_S = 0.1  # the speed check starts this long after the planned acceleration ends
OVERTIME_FACTOR = 1.5
OVERTIME_SLACK_S = 1.0
UNDER_REACH_TOL_RAD = 0.02


@dataclass(frozen=True)
class ProfileLimits:
    cap_mps: float  # the teleop command cap (forward and reverse)
    kappa_max: float  # the teleop curvature limit
    kappa_rate: float  # the teleop curvature rate (1/m/s)
    q_rate: float  # the host's steering slew on q (rad/s)
    accel_mps2: float
    q_of_kappa: Callable[[float], float]
    kappa_of_q: Callable[[float], float]
    max_total_s: float = 600.0


@dataclass(frozen=True)
class Segment:
    label: str
    kind: str  # presteer | hold | drive
    hold_s: float = 0.0
    speed_mps: float = 0.0
    distance_m: float = 0.0
    kappa: float = 0.0  # presteer target, or a drive's constant / starting curvature
    merge: tuple | None = None  # (peak, length)
    step: tuple | None = None  # (to_kappa, after_s)
    q_step_ramp_s: float | None = None
    at_max: bool = False  # a constant drive at the teleop curvature limit (under-reach is checked)

    @property
    def peak_kappa(self) -> float | None:
        return None if self.merge is None else self.merge[0]

    def kappa_at(self, s_m: float, t_s: float) -> float:
        """The requested curvature at encoder distance ``s_m`` and ``t_s`` after the drive began."""
        if self.merge is not None:
            peak, length = self.merge
            return peak * math.sin(2 * math.pi * min(max(s_m, 0.0), length) / length)
        if self.step is not None and t_s >= self.step[1]:
            return self.step[0]
        return self.kappa


@dataclass(frozen=True)
class Profile:
    name: str
    segments: tuple
    planned_s: float


def _kappa(value, lim: ProfileLimits, where: str) -> float:
    if value == "max":
        return lim.kappa_max
    if value == "-max":
        return -lim.kappa_max
    k = float(value)
    if not math.isfinite(k) or abs(k) > lim.kappa_max + 1e-12:
        raise ValueError(f"{where}: curvature {value} outside the teleop limit {lim.kappa_max} 1/m")
    return k


def _dq_dkappa(lim: ProfileLimits) -> float:
    """The largest |dq/dkappa| over the teleop range (numerical)."""
    n, k = 200, lim.kappa_max
    h = k / n
    return max(abs(lim.q_of_kappa(-k + (i + 1) * h) - lim.q_of_kappa(-k + i * h)) / h for i in range(2 * n))


def parse_profile(data: dict, lim: ProfileLimits) -> Profile:
    """Read and check a profile (design 2.1). Anything the gates would reshape is refused, so the command the record
    shows follows the profile when no gate intervenes."""
    if not isinstance(data, dict) or data.get("schema") != SCHEMA:
        raise ValueError(f"a profile needs schema {SCHEMA!r}")
    if data.get("load", "empty") != "empty":
        raise ValueError("only empty-truck profiles: loaded teleop is refused on the real truck (lead decision (a))")
    raw = data.get("segments")
    if not isinstance(raw, list) or not raw:
        raise ValueError("a profile needs segments")
    dq = _dq_dkappa(lim)
    segs, current, planned = [], None, 0.0
    for i, r in enumerate(raw):
        label = str(r.get("label", f"seg{i}"))
        where = f"segment {i} ({label})"
        if "presteer_kappa_inv_m" in r or "presteer_q_rad" in r:
            k = (_kappa(r["presteer_kappa_inv_m"], lim, where) if "presteer_kappa_inv_m" in r
                 else _kappa(lim.kappa_of_q(float(r["presteer_q_rad"])), lim, where))
            segs.append(Segment(label, "presteer", kappa=k))
            current, planned = k, planned + PRESTEER_TIMEOUT_S
            continue
        if current is None:
            raise ValueError(f"{where}: a profile starts with a presteer (the standing q is not known)")
        if "hold_s" in r:
            h = float(r["hold_s"])
            if not h > 0:
                raise ValueError(f"{where}: hold_s must be positive")
            segs.append(Segment(label, "hold", hold_s=h, kappa=current))
            planned += h
            continue
        v = float(r.get("speed_mps", 0.0))
        if not (math.isfinite(v) and 0.0 < abs(v) <= lim.cap_mps + 1e-12):
            raise ValueError(f"{where}: speed {v} m/s must be nonzero and within the teleop cap {lim.cap_mps:.3f}")
        merge = step = ramp = None
        at_max = False
        if "s_merge" in r:
            off, length = float(r["s_merge"]["offset_m"]), float(r["s_merge"]["length_m"])
            peak = 2 * math.pi * off / length ** 2
            _kappa(abs(peak), lim, where)
            rate = abs(peak) * 2 * math.pi / length * abs(v)
            if rate > lim.kappa_rate + 1e-12:
                raise ValueError(f"{where}: curvature rate {rate:.3f} 1/m/s over the teleop rate {lim.kappa_rate}")
            if rate * dq > lim.q_rate + 1e-12:
                raise ValueError(f"{where}: q rate {rate * dq:.3f} rad/s over the steering slew {lim.q_rate}")
            d = float(r.get("distance_m", length))
            if abs(d - length) > 1e-9:
                raise ValueError(f"{where}: a merge's distance is its length")
            merge, k0 = (peak, length), 0.0
        else:
            k0 = (_kappa(r["kappa_inv_m"], lim, where) if "kappa_inv_m" in r
                  else _kappa(lim.kappa_of_q(float(r["q_rad"])), lim, where))
            at_max = r.get("kappa_inv_m") in ("max", "-max")
            d = float(r["distance_m"])
            if "q_step" in r:
                to = _kappa(lim.kappa_of_q(float(r["q_step"]["to_rad"])), lim, where)
                after = float(r["q_step"]["after_s"])
                if after < abs(v) / lim.accel_mps2 + 0.5:
                    raise ValueError(f"{where}: q_step after_s must leave the speed settled (>= ramp + 0.5 s)")
                step = (to, after)
                ramp = abs(lim.q_of_kappa(to) - lim.q_of_kappa(k0)) / lim.q_rate
        if not d > 0:
            raise ValueError(f"{where}: distance_m must be positive")
        if abs(k0 - current) > 1e-9:
            raise ValueError(f"{where}: curvature {k0} does not continue from {current} (add a presteer)")
        seg = Segment(label, "drive", speed_mps=v, distance_m=d, kappa=k0, merge=merge, step=step,
                      q_step_ramp_s=ramp, at_max=at_max)
        segs.append(seg)
        current = seg.kappa_at(d, math.inf)
        planned += d / abs(v) + abs(v) / lim.accel_mps2
    if planned > lim.max_total_s:
        raise ValueError(f"planned total {planned:.0f} s over {lim.max_total_s:.0f} s")
    return Profile(str(data.get("name", "")), tuple(segs), planned)


class Observation(NamedTuple):
    """One bridge cycle as the runner sees it, after the core decided (design 2: request vs what was driven)."""

    dt_s: float
    speed_valid: bool
    speed_mps: float  # the encoder's signed speed
    meas_q: float | None
    armed: bool
    mode: str
    target_mps: float  # the arbiter's output
    applied_mps: float  # what the bridge put on the port
    applied_q: float | None
    permit: tuple | None  # (limit, reason) of the arc bundle
    port: str | None
    direction_cut: object
    stop: object


@dataclass
class ProfileRunner:
    profile: Profile
    limits: ProfileLimits
    tolerance_mps: float = 0.005
    state: str = "idle"  # idle | running | zeroing
    trial: int = 0
    result: str | None = None
    under_reached: list = field(default_factory=list)

    def __post_init__(self) -> None:
        self._marks: list = []
        self._deadman = False
        self._row_s: float | None = None
        self._reset()

    def _reset(self) -> None:
        self.i = 0
        self.seg_t = 0.0
        self.s_m = 0.0
        self.moving = False  # the drive segment has begun (the deadman was held)
        self.drive_t = 0.0
        self.settle_t = 0.0
        self.reached = False
        self.req_mps = 0.0
        self.req_kappa = 0.0

    @property
    def segment(self) -> Segment | None:
        return self.profile.segments[self.i] if self.state == "running" else None

    def take_marks(self) -> list:
        out, self._marks = self._marks, []
        return out

    def _enter(self, host_ns: int) -> None:
        self.seg_t = self.s_m = self.drive_t = self.settle_t = 0.0
        self.moving = self.reached = False
        self._marks.append({"label": f"profile:{self.trial}:{self.segment.label}:start", "host_ns": host_ns})

    def start(self, now_ns: int, *, standstill_s: float, armed: bool) -> str | None:
        """None when a trial starts; else why not (busy, not_armed, not_still)."""
        if self.state != "idle":
            return "busy"
        if not armed:
            return "not_armed"
        if standstill_s < STILL_START_S:
            return "not_still"
        self.trial += 1
        self.result, self.under_reached = None, []
        self._reset()
        self.state = "running"
        self._enter(now_ns)
        return None

    def rewrite(self, msg: dict) -> dict:
        """The operator's teleop message with the profile's two ratios (idle: untouched; ended: zeros until the
        deadman is let go). Never makes a message."""
        if msg.get("t") != "teleop":
            return msg
        self._deadman = msg.get("deadman") is True
        if self.state == "idle":
            return msg
        if self.state == "zeroing":
            return {**msg, "speed_ratio": 0.0, "curvature_ratio": 0.0}
        return {**msg, "speed_ratio": self.req_mps / self.limits.cap_mps,
                "curvature_ratio": self.req_kappa / self.limits.kappa_max}

    def _presteer_check(self, q: float | None, target: float) -> bool:
        return q is not None and abs(q - self.limits.q_of_kappa(target)) <= PRESTEER_TOL_RAD

    def _end(self, result: str) -> None:
        self.result = result
        self.state = "zeroing"
        self.req_mps = self.req_kappa = 0.0

    def _next(self, host_ns: int) -> None:
        seg = self.segment
        self._row_s = self.s_m  # the row of this cycle shows where the finished segment ended
        if seg.kind == "drive" and seg.at_max and not self.reached:
            self.under_reached.append(seg.label)
        self.i += 1
        if self.i >= len(self.profile.segments):
            self._end("done")
        else:
            self._enter(host_ns)

    def step(self, now_ns: int, obs: Observation) -> dict | None:
        """Advance one bridge cycle; returns the profile row (None when idle)."""
        if self.state == "zeroing":
            self._row_s = None
            row = self._row(now_ns, obs, None)
            if not self._deadman:
                self.state = "idle"
            return row
        if self.state != "running":
            return None
        seg = self.segment
        self._row_s = None
        if not obs.armed or obs.mode != "TELEOP":
            self._end("void:disarmed")
            return self._row(now_ns, obs, seg)
        if obs.stop:
            self._end(f"void:stopped:{obs.stop}")
            return self._row(now_ns, obs, seg)
        self.seg_t += obs.dt_s
        if seg.kind == "presteer":
            self.req_mps, self.req_kappa = 0.0, seg.kappa
            self.settle_t = self.settle_t + obs.dt_s if self._presteer_check(obs.meas_q, seg.kappa) else 0.0
            if self.settle_t >= PRESTEER_SETTLE_S:
                self._next(now_ns)
            elif self.seg_t >= PRESTEER_TIMEOUT_S:
                self._end("void:presteer_timeout")
        elif seg.kind == "hold":
            self.req_mps, self.req_kappa = 0.0, seg.kappa
            if self.seg_t >= seg.hold_s:
                self._next(now_ns)
        else:
            self._drive(now_ns, obs, seg)
        return self._row(now_ns, obs, seg)

    def _drive(self, now_ns: int, obs: Observation, seg: Segment) -> None:
        lim = self.limits
        if not self.moving:
            self.req_mps, self.req_kappa = 0.0, seg.kappa_at(0.0, 0.0)
            if not self._deadman:
                return  # waits in place for the deadman (design 2: a drive starts only while it is held)
            self.moving = True
        elif not self._deadman:
            self._end("void:deadman")
            return
        sign = 1.0 if seg.speed_mps > 0 else -1.0
        if obs.speed_valid:
            self.s_m += obs.speed_mps * sign * obs.dt_s
        self.drive_t += obs.dt_s
        ramp_s = abs(seg.speed_mps) / lim.accel_mps2
        if self.drive_t > ramp_s + RAMP_SLACK_S:
            expected = abs(seg.speed_mps)
            if abs(obs.applied_mps) < expected - self.tolerance_mps:
                why = obs.direction_cut or (obs.port if obs.port not in (None, "queued") else None) or (
                    obs.permit[1] if obs.permit else None) or "speed"
                self._end(f"void:limited:{why}")
                return
        if self.drive_t > (seg.distance_m / abs(seg.speed_mps) + ramp_s) * OVERTIME_FACTOR + OVERTIME_SLACK_S:
            self._end("void:overtime")
            return
        if seg.at_max and obs.meas_q is not None and \
                abs(obs.meas_q - lim.q_of_kappa(seg.kappa)) <= UNDER_REACH_TOL_RAD:
            self.reached = True
        if self.s_m >= seg.distance_m:
            self._next(now_ns)
            return
        self.req_mps, self.req_kappa = seg.speed_mps, seg.kappa_at(self.s_m, self.drive_t)

    def _row(self, now_ns: int, obs: Observation, seg: Segment | None) -> dict:
        s_m = self.s_m if self._row_s is None else self._row_s
        return {"t": "profile", "host_ns": now_ns, "trial": self.trial, "state": self.state,
                "seg": self.i, "label": None if seg is None else seg.label, "s_m": s_m,
                "req_mps": self.req_mps, "req_kappa": self.req_kappa,
                "arb_mps": obs.target_mps, "arb_limit": None if obs.permit is None else list(obs.permit),
                "drive_mps": obs.applied_mps, "drive_q": obs.applied_q, "port": obs.port,
                "direction_cut": obs.direction_cut, "meas_q": obs.meas_q, "result": self.result}
