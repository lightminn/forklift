"""The real truck's safety watch (docs/design/2026-11-16-real-safety-watch.md, 4e-2).

The simulation judged scene damage (tilt, slip, forbidden contact) from truth and in effect used it as a safety
watch; the real truck has no truth. These are the host-side stand-ins, kept small (a school-scale minimum):

- motor current: any channel (L, R, M, H) over its limit for the hold -> ``watch_current``;
- each rear wheel: the target of the DRIVE last queued against the MCU-measured rate, apart by more than the
  tolerance for the hold (both under the floor: not judged) -> ``watch_wheel``;
- tilt: only when an IMU owner hands acceleration samples -- the angle from the gravity direction taken at the arm,
  over the limit for the hold -> ``watch_tilt``; samples expected but older than ``max_age_s`` -> ``watch_imu_stale``.

Judged only while armed. Current and wheels count only on a fresh, newly received STATE: a stale link clears their
holds (the link's own stops -- MCU expiry, ``speed_invalid`` -- take that case), a cycle with no new STATE judges
nothing. A trip restarts that check's hold and is counted per reason. No clock is read: the caller passes ``t_s``.
Thresholds come from ``config/real/safety_watch.yaml`` (all provisional, see their provenance)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

REASONS = ("watch_current", "watch_wheel", "watch_tilt", "watch_imu_stale")
# The longest gap between two judged STATEs a hold may run across: longer, and the hold starts again on the new STATE
# (a gap is not evidence the condition held; Codex code review P2). Twice the link's 50 ms freshness age.
STATE_GAP_MAX_S = 0.1


@dataclass(frozen=True)
class WatchConfig:
    current_limit_a: tuple[float, float, float, float]
    current_hold_s: float
    wheel_tol_mps: float
    wheel_floor_mps: float
    wheel_hold_s: float
    tilt_limit_deg: float
    tilt_hold_s: float
    imu_max_age_s: float


def load_watch_config(path) -> WatchConfig:
    """The watch's thresholds; every part must carry a provenance (they are provisional)."""
    import yaml

    raw = yaml.safe_load(Path(path).read_text())
    if raw.get("schema") != "safety_watch/1":
        raise ValueError(f"{path}: schema {raw.get('schema')!r}, not 'safety_watch/1'")
    for part in ("current", "wheel", "tilt"):
        if not isinstance(raw.get(part), dict) or "provenance" not in raw[part]:
            raise ValueError(f"{path}: {part} needs a provenance")
    c, w, t = raw["current"], raw["wheel"], raw["tilt"]
    limits = tuple(float(v) for v in c["limit_a"])
    if len(limits) != 4 or any(not (v > 0 and math.isfinite(v)) for v in limits):
        raise ValueError(f"{path}: current.limit_a needs four positive values (L, R, M, H)")
    cfg = WatchConfig(limits, float(c["hold_s"]), float(w["tol_mps"]), float(w["floor_mps"]), float(w["hold_s"]),
                      float(t["limit_deg"]), float(t["hold_s"]), float(t["max_age_s"]))
    for name in ("current_hold_s", "wheel_tol_mps", "wheel_floor_mps", "wheel_hold_s", "tilt_limit_deg",
                 "tilt_hold_s", "imu_max_age_s"):
        v = getattr(cfg, name)
        if not (v > 0 and math.isfinite(v)):
            raise ValueError(f"{path}: {name} must be positive, not {v!r}")
    return cfg


@dataclass
class WatchResult:
    trip: str | None
    values: dict = field(default_factory=dict)


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return None if not (n > 0 and math.isfinite(n)) else tuple(x / n for x in v)


class SafetyWatch:
    def __init__(self, cfg: WatchConfig, *, wheel_radius_m: tuple[float, float]) -> None:
        self.cfg = cfg
        self.radius = tuple(float(r) for r in wheel_radius_m)
        self.counts: dict[str, int] = {}
        self._since: dict[str, float | None] = {r: None for r in REASONS}
        self._baseline = None  # gravity direction at the arm (tilt)
        self._armed = False
        self._last_state_t: float | None = None  # the time of the last STATE judged (current, wheels)

    def _clear(self, *reasons) -> None:
        for r in reasons:
            self._since[r] = None

    def _hold(self, reason: str, bad: bool, t_s: float, hold_s: float) -> bool:
        if not bad:
            self._since[reason] = None
            return False
        if self._since[reason] is None:
            self._since[reason] = t_s
        if t_s - self._since[reason] >= hold_s - 1e-9:
            self._since[reason] = None  # the hold restarts after a trip
            self.counts[reason] = self.counts.get(reason, 0) + 1
            return True
        return False

    def step(self, t_s: float, *, armed: bool, fresh: bool, new_state: bool, currents_a=None,
             wheel_targets_rad_s=None, wheel_rates_rad_s=None, accel=None, accel_t_s=None,
             imu_expected: bool = False) -> WatchResult:
        """One judgement. ``accel``: the newest acceleration sample (x, y, z in m/s^2) and ``accel_t_s`` its time on
        the same clock as ``t_s`` (None: now); ``imu_expected``: an IMU owner promised samples."""
        values: dict = {}
        if currents_a is not None and not all(math.isfinite(float(x)) for x in currents_a):
            currents_a = None  # not judged
        if wheel_rates_rad_s is not None and not all(math.isfinite(float(x)) for x in wheel_rates_rad_s):
            wheel_rates_rad_s = None
        if not armed:
            self._armed = False
            self._baseline = None
            self._clear(*REASONS)
            self._last_state_t = None
            return WatchResult(None, values)
        if not self._armed:
            self._armed = True
            self._clear(*REASONS)
        trip = None
        c = self.cfg
        if not fresh:
            self._clear("watch_current", "watch_wheel")
            self._last_state_t = None
        elif new_state:
            if self._last_state_t is None or t_s - self._last_state_t > STATE_GAP_MAX_S:
                self._clear("watch_current", "watch_wheel")
            self._last_state_t = t_s
            if currents_a is not None:
                mags = [abs(float(x)) for x in currents_a]
                ch = max(range(len(mags)), key=lambda i: mags[i])
                values.update(current_max_a=mags[ch], current_channel=ch)
                over = any(m > lim for m, lim in zip(mags, c.current_limit_a))
                if self._hold("watch_current", over, t_s, c.current_hold_s):
                    trip = trip or "watch_current"
            if wheel_rates_rad_s is not None:
                targets = wheel_targets_rad_s if wheel_targets_rad_s is not None else (0.0, 0.0)
                errs, judged = [], []
                for i in (0, 1):
                    r = self.radius[i]
                    tgt, meas = float(targets[i]) * r, float(wheel_rates_rad_s[i]) * r
                    errs.append(abs(tgt - meas))
                    judged.append(not (abs(tgt) < c.wheel_floor_mps and abs(meas) < c.wheel_floor_mps))
                wi = max((0, 1), key=lambda i: errs[i])
                values.update(wheel_error_mps=errs[wi], wheel=wi)
                bad = any(j and e > c.wheel_tol_mps for j, e in zip(judged, errs))
                if self._hold("watch_wheel", bad, t_s, c.wheel_hold_s):
                    trip = trip or "watch_wheel"
        if accel is not None or imu_expected:
            sample_t = t_s if accel_t_s is None else float(accel_t_s)
            stale = imu_expected and (t_s - sample_t > c.imu_max_age_s)
            if stale:
                if self._hold("watch_imu_stale", True, t_s, 0.0):
                    trip = trip or "watch_imu_stale"
            else:
                self._clear("watch_imu_stale")
            u = _unit(accel) if accel is not None else None
            if u is not None and not stale:
                if self._baseline is None:
                    self._baseline = u
                cos = max(-1.0, min(1.0, sum(a * b for a, b in zip(u, self._baseline))))
                tilt = math.degrees(math.acos(cos))
                values["tilt_deg"] = tilt
                if self._hold("watch_tilt", tilt > c.tilt_limit_deg, t_s, c.tilt_hold_s):
                    trip = trip or "watch_tilt"
        pending = [r for r, since in self._since.items() if since is not None]
        if pending:
            values["pending"] = pending  # a condition met, its hold not yet reached: recorded, not told
        return WatchResult(trip, values)
