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


# --- the IMU watch of the real autonomy (docs/design/2026-11-19-real-safety-wiring.md 4, stage 5b) -----------------
GRAVITY_MPS2 = 9.81
IMU_REASONS = ("watch_imu_stale", "watch_imu_gap", "watch_imu_moved", "watch_tilt")


@dataclass(frozen=True)
class ImuConfig:
    limit_deg: float  # the tilt from the baseline over which a sample counts toward the hold
    hold_s: float  # how long consecutive tilted samples must last
    max_age_s: float  # the newest valid sample older than this (host clock): watch_imu_stale
    gap_max_s: float  # two consecutive valid samples further apart than this, armed: watch_imu_gap
    baseline_min: int  # valid samples taken in the current stand the baseline needs
    baseline_spread_deg: float  # every baseline sample within this of their mean direction
    baseline_wait_s: float  # a stand shorter than this may still be gathering its baseline (the ARM waits)
    norm_tol_frac: float  # |‖a‖ - g| over this fraction of g: not a valid sample
    level_ref: tuple = (0.0, 0.0, 1.0)  # gravity's direction in the IMU frame on the level floor (the mount)
    level_tol_deg: float = 5.0  # the baseline within this of level_ref, or the stand is not level
    window_s: float = 2.0  # how long valid samples are kept

    def __post_init__(self) -> None:
        for name in ("limit_deg", "hold_s", "max_age_s", "gap_max_s", "baseline_spread_deg", "baseline_wait_s",
                     "norm_tol_frac", "level_tol_deg", "window_s"):
            v = getattr(self, name)
            try:
                ok = not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) and v > 0
            except OverflowError:
                ok = False  # an integer too large for a float (5b round 9 P3)
            if not ok:
                raise ValueError(f"imu_config:{name}")
        if isinstance(self.baseline_min, bool) or not (isinstance(self.baseline_min, int) and self.baseline_min >= 2):
            raise ValueError("imu_config:baseline_min")
        # only settings the watch can judge with (5b round 1 P2-4)
        if not self.limit_deg < 90.0:
            raise ValueError("imu_config:limit_deg")
        if not self.baseline_spread_deg < self.limit_deg:
            raise ValueError("imu_config:baseline_spread_deg")
        if not self.gap_max_s <= self.max_age_s:
            raise ValueError("imu_config:gap_max_s")
        if not self.hold_s > self.gap_max_s:
            raise ValueError("imu_config:hold_s")
        if not self.norm_tol_frac < 1.0:
            raise ValueError("imu_config:norm_tol_frac")
        if not self.level_tol_deg < self.limit_deg:
            raise ValueError("imu_config:level_tol_deg")
        # three finite numbers in a sequence: a mapping's keys or a string's characters are not a direction
        # (5b round 7 P2-2)
        raw = self.level_ref
        try:
            ok = isinstance(raw, (list, tuple)) and len(raw) == 3 and all(
                isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in raw)
            ok = ok and _unit(tuple(float(v) for v in raw)) is not None
        except (OverflowError, ValueError, TypeError):
            ok = False  # an integer too large for a float is not a direction either (5b round 8 P3)
        if not ok:
            raise ValueError("imu_config:level_ref")


def load_imu_config(path) -> ImuConfig:
    """The ``imu:`` part of ``config/real/safety_watch.yaml`` (it must carry a provenance: provisional values)."""
    import yaml

    raw = yaml.safe_load(Path(path).read_text())
    part = raw.get("imu") if isinstance(raw, dict) else None
    if not isinstance(part, dict) or "provenance" not in part:
        raise ValueError("imu_config:missing")
    names = ("limit_deg", "hold_s", "max_age_s", "gap_max_s", "baseline_min", "baseline_spread_deg",
             "baseline_wait_s", "norm_tol_frac", "level_ref", "level_tol_deg")
    for name in names:
        if name not in part:
            raise ValueError(f"imu_config:{name}")
    values = {k: part[k] for k in names}
    values["level_ref"] = tuple(values["level_ref"]) if isinstance(values["level_ref"], list) else values["level_ref"]
    return ImuConfig(**values)


class ImuWatch:
    """The tilt and the IMU's loss while armed, on distinct valid samples (design 4; the SafetyWatch tilt part counted
    a re-handed sample as continuing and took a missing time as now). Every time is host monotonic ns; the caller
    passes the clock."""

    def __init__(self, cfg: ImuConfig) -> None:
        if not isinstance(cfg, ImuConfig):
            raise TypeError("ImuConfig expected")
        cfg.__post_init__()
        self.cfg = cfg
        self.samples: list = []  # (ns, unit vector) of valid samples, oldest first, within window_s
        self.counts = {"valid": 0, "not_newer": 0, "future": 0, "not_finite": 0, "norm": 0}
        self._baseline = None  # the unit gravity direction frozen before the ARM
        self._prev_ns = None  # the newest sample judged
        self._arm_until = None
        self._tilt_start = None  # the first sample of the current tilted run
        self._trip = None

    def add(self, sample_ns: int, accel, now_ns: int) -> str | None:
        """One sample from the source: None when valid, else why it was not counted."""
        why = None
        try:
            v = tuple(float(x) for x in accel)
        except (TypeError, ValueError):
            v = ()
        n = 0.0
        if len(v) != 3 or not all(math.isfinite(x) for x in v):
            why = "not_finite"
        elif self.samples and int(sample_ns) <= self.samples[-1][0]:
            why = "not_newer"
        elif int(sample_ns) > int(now_ns):
            why = "future"
        else:
            n = math.sqrt(sum(x * x for x in v))
            if abs(n - GRAVITY_MPS2) > self.cfg.norm_tol_frac * GRAVITY_MPS2:
                why = "norm"
        if why is not None:
            self.counts[why] += 1
            return why
        self.counts["valid"] += 1
        self.samples.append((int(sample_ns), tuple(x / n for x in v)))
        cut = int(sample_ns) - int(self.cfg.window_s * 1e9)
        while len(self.samples) > 1 and self.samples[0][0] < cut:
            self.samples.pop(0)
        return None

    def baseline(self, still_since_ns):
        """``((direction, last_ns), None)`` from the valid samples taken in the current stand -- ``last_ns`` the
        newest one used; the stand's latest gapless run --, or ``(None, reason)``: ``imu_not_still``,
        ``imu_no_baseline``, ``imu_baseline_unsteady``."""
        if still_since_ns is None:
            return None, "imu_not_still"
        c = self.cfg
        stand = [(ns, u) for ns, u in self.samples if ns >= int(still_since_ns)]
        # the latest gapless run of the stand: an earlier gap only means the run started later
        part = stand
        for k in range(len(stand) - 1, 0, -1):
            if stand[k][0] - stand[k - 1][0] > c.gap_max_s * 1e9:
                part = stand[k:]
                break
        if len(part) < c.baseline_min:
            return None, "imu_no_baseline"
        mean = tuple(sum(u[k] for _, u in part) / len(part) for k in range(3))
        size = math.sqrt(sum(x * x for x in mean))
        if size < 0.9:
            return None, "imu_baseline_unsteady"
        d = tuple(x / size for x in mean)
        if any(_angle_deg(u, d) > c.baseline_spread_deg for _, u in part):
            return None, "imu_baseline_unsteady"
        # the stand is on the level floor (5b round 6: an absolute check instead of the stand's own history -- a
        # pose changed while standing, behind a gap or a lost stand, is caught by where it is, not by when)
        if _angle_deg(d, _unit(tuple(float(x) for x in c.level_ref))) > c.level_tol_deg:
            return None, "imu_not_level"
        return (d, part[-1][0]), None

    def arm(self, baseline) -> None:
        """Freeze ``(direction, last_ns)`` from ``baseline()`` before the ARM is asked (kept while armed); the
        judgement starts at ``last_ns``: the first sample after it is measured from it (13th draft, round 2 P2-1)."""
        direction, last_ns = baseline
        d = _unit(tuple(float(x) for x in direction))
        if d is None:
            raise ValueError("imu baseline direction")
        self._baseline, self._trip = d, None
        self._prev_ns = int(last_ns)  # the newest sample judged (the baseline's last at first)
        self._tilt_start = None
        self._arm_until = None  # samples taken up to this instant meet the ARMING rule (None: all, still arming)

    def settled(self, boundary_ns: int) -> None:
        """The ARM was answered (the permit seen READY at ``boundary_ns``): samples taken up to then meet the ARMING
        rule, later ones the tilt hold -- by acquisition time, whichever read brings them (5b round 8 P2). The first
        call sets it; later calls change nothing."""
        if self._arm_until is None:
            self._arm_until = int(boundary_ns)

    def disarm(self) -> None:
        self._baseline = self._trip = self._prev_ns = self._tilt_start = None
        self._arm_until = None

    @property
    def armed(self) -> bool:
        return self._baseline is not None

    def judge(self, now_ns: int) -> str | None:
        """With a frozen baseline: ``watch_imu_stale``, ``watch_imu_gap`` or ``watch_tilt`` (each kept until the
        disarm), else None. The samples received since the last call are taken in order into an incremental state
        (the previous sample's time, the tilt hold's start): a hold of any length is measured, whatever the window
        (13th draft, round 2 P1-1)."""
        if self._baseline is None:
            return None
        if self._trip is not None:
            return self._trip
        c = self.cfg
        for ns, u in self.samples:
            if ns <= self._prev_ns:
                continue  # judged already (or the baseline's own)
            if ns - self._prev_ns > c.gap_max_s * 1e9:
                self._trip = "watch_imu_gap"  # sparse samples cannot carry a hold: the watch is lost
                return self._trip
            self._prev_ns = ns
            arming = self._arm_until is None or ns <= self._arm_until
            if arming and _angle_deg(u, self._baseline) > c.baseline_spread_deg:
                # the truck moved between the baseline and the ARM's answer -- the ARM's own first byte included
                # (5b round 6 P2-1): the baseline no longer says where it stands
                self._trip = "watch_imu_moved"
                return self._trip
            if _angle_deg(u, self._baseline) > c.limit_deg:
                self._tilt_start = ns if self._tilt_start is None else self._tilt_start
                if ns - self._tilt_start >= c.hold_s * 1e9:
                    self._trip = "watch_tilt"
                    return self._trip
            else:
                self._tilt_start = None
        if int(now_ns) - self._prev_ns > c.max_age_s * 1e9:
            self._trip = "watch_imu_stale"
        return self._trip


def _angle_deg(u, v) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(u, v))))))
