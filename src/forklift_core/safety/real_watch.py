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
             imu_expected: bool = False, skip=frozenset()) -> WatchResult:
        """One judgement. ``accel``: the newest acceleration sample (x, y, z in m/s^2) and ``accel_t_s`` its time on
        the same clock as ``t_s`` (None: now); ``imu_expected``: an IMU owner promised samples. ``skip``: checks the
        operator bypassed (``watch_current`` / ``watch_wheel``; real safety wiring design 5) -- not judged at all, so
        the other check's hold is untouched (empty by default: the teleop bridge unchanged)."""
        values: dict = {}
        if "watch_current" in skip:
            currents_a = None
            self._clear("watch_current")
        if "watch_wheel" in skip:
            wheel_rates_rad_s = None
            self._clear("watch_wheel")
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


_NO_GENERATION = object()  # no sample or generation noted yet


class ImuBaseline(tuple):
    """``(direction, last_ns)`` as ``baseline()`` hands it out, carrying the last sample's ``lo``, serial and
    generation so ``arm`` resumes from that very sample -- even once the window has dropped it -- and refuses a
    baseline of an older generation (Codex 7b round 1 P1-6)."""

    def __new__(cls, direction, last_ns, *, lo, serial, gen):
        self = super().__new__(cls, (direction, last_ns))
        self.lo, self.serial, self.gen = lo, serial, gen
        return self


class ImuWatch:
    """The tilt and the IMU's loss while armed, on distinct valid samples (design 4; the SafetyWatch tilt part counted
    a re-handed sample as continuing and took a missing time as now). Every time is host monotonic ns; the caller
    passes the clock.

    A sample is an acquisition interval ``[lo, hi]`` with its producer's generation (PR 7 design 5, 7b), judged at
    its worst: freshness from ``lo``, a gap as the next ``hi`` minus the previous ``lo``, the ARMING rule for any
    sample whose ``lo`` is not after READY, a tilt hold at its longest possible length (the run's first ``lo`` to its
    last ``hi``). A new generation is never joined to the old one: armed, it is a lost watch (``watch_imu_gap``) even
    when the samples are 5 ms apart; not armed, the old samples are dropped and the baseline gathers again. ``add``
    is a point sample (``lo == hi``) with no generation -- the 5b contract, unchanged."""

    def __init__(self, cfg: ImuConfig) -> None:
        if not isinstance(cfg, ImuConfig):
            raise TypeError("ImuConfig expected")
        cfg.__post_init__()
        self.cfg = cfg
        # (hi, unit vector, lo, generation, serial) of valid samples, oldest first, within window_s of the newest hi
        self.samples: list = []
        self.counts = {"valid": 0, "not_newer": 0, "future": 0, "not_finite": 0, "norm": 0, "bad_interval": 0,
                       "generation": 0}
        self._gen = _NO_GENERATION  # the generation of the samples kept (none seen yet)
        self._serial = 0
        self._baseline = None  # the unit gravity direction frozen before the ARM
        self._prev = None  # the newest sample judged: (serial or None, lo, hi)
        self._arm_until = None
        self._tilt_start = None  # the first ``lo`` of the current tilted run
        self._tilt_n = 0  # the tilted samples in it
        self._trip = None

    def add(self, sample_ns: int, accel, now_ns: int) -> str | None:
        """One point sample from the source: None when valid, else why it was not counted."""
        return self.add_interval(None, sample_ns, sample_ns, accel, now_ns)

    def add_interval(self, gen, lo_ns: int, hi_ns: int, accel, now_ns: int) -> str | None:
        """One sample taken somewhere in ``[lo_ns, hi_ns]`` by producer generation ``gen``: None when valid, else why
        it was not counted. A new generation is noted first (``note_generation``), whatever the sample's fate."""
        self.note_generation(gen)
        why = None
        try:
            v = tuple(float(x) for x in accel)
        except (TypeError, ValueError):
            v = ()
        lo, hi = int(lo_ns), int(hi_ns)
        n = 0.0
        if len(v) != 3 or not all(math.isfinite(x) for x in v):
            why = "not_finite"
        elif lo > hi:
            why = "bad_interval"
        elif self.samples and hi <= self.samples[-1][0]:
            why = "not_newer"
        elif hi > int(now_ns):
            why = "future"
        else:
            n = math.sqrt(sum(x * x for x in v))
            if abs(n - GRAVITY_MPS2) > self.cfg.norm_tol_frac * GRAVITY_MPS2:
                why = "norm"
        if why is not None:
            self.counts[why] += 1
            return why
        self.counts["valid"] += 1
        self._serial += 1
        self.samples.append((hi, tuple(x / n for x in v), lo, gen, self._serial))
        cut = hi - int(self.cfg.window_s * 1e9)
        while len(self.samples) > 1 and self.samples[0][0] < cut:
            self.samples.pop(0)
        return None

    def note_generation(self, gen) -> None:
        """The source's current generation, with or without a new sample (design 5: a restart is seen even when no
        sample follows it). A change drops every kept sample; armed, it is ``watch_imu_gap`` until the disarm."""
        if self._gen is _NO_GENERATION:
            self._gen = gen
            return
        if gen == self._gen:
            return
        self.counts["generation"] += 1
        self._gen = gen
        self.samples.clear()
        if self._baseline is not None and self._trip is None:
            self._trip = "watch_imu_gap"  # the old generation's baseline and hold do not carry over

    def baseline(self, still_since_ns):
        """``((direction, last_ns), None)`` from the valid samples taken in the current stand (``lo`` not before it)
        -- ``last_ns`` the ``hi`` of the newest one used; the stand's latest gapless run --, or ``(None, reason)``:
        ``imu_not_still``, ``imu_no_baseline``, ``imu_baseline_unsteady``, ``imu_not_level``."""
        if still_since_ns is None:
            return None, "imu_not_still"
        c = self.cfg
        # the latest gapless run of samples that all started in the stand, taken back from the newest: a sample that
        # may predate the stand (or a gap) ends it -- the samples on either side are never joined around it (Codex
        # 7b round 1 P1-5: a wide sample in the middle filtered out would join two runs)
        part: list = []
        for s in reversed(self.samples):
            if s[2] < int(still_since_ns) or (part and part[0][0] - s[2] > c.gap_max_s * 1e9):
                break
            part.insert(0, s)
        if len(part) < c.baseline_min:
            return None, "imu_no_baseline"
        mean = tuple(sum(s[1][k] for s in part) / len(part) for k in range(3))
        size = math.sqrt(sum(x * x for x in mean))
        if size < 0.9:
            return None, "imu_baseline_unsteady"
        d = tuple(x / size for x in mean)
        if any(_angle_deg(s[1], d) > c.baseline_spread_deg for s in part):
            return None, "imu_baseline_unsteady"
        # the stand is on the level floor (5b round 6: an absolute check instead of the stand's own history -- a
        # pose changed while standing, behind a gap or a lost stand, is caught by where it is, not by when)
        if _angle_deg(d, _unit(tuple(float(x) for x in c.level_ref))) > c.level_tol_deg:
            return None, "imu_not_level"
        last = part[-1]
        return ImuBaseline(d, last[0], lo=last[2], serial=last[4], gen=last[3]), None

    def arm(self, baseline) -> None:
        """Freeze ``(direction, last_ns)`` from ``baseline()`` before the ARM is asked (kept while armed); the
        judgement starts at that sample: the first one after it is measured from its ``lo`` (13th draft, round 2
        P2-1). A baseline this watch handed out resumes from its own last sample; one of an older generation than
        the samples now kept is a lost watch at once. Any other ``(direction, last_ns)``: the kept sample ending
        there, else a point sample at ``last_ns``."""
        direction, last_ns = baseline
        d = _unit(tuple(float(x) for x in direction))
        if d is None:
            raise ValueError("imu baseline direction")
        self._baseline, self._trip = d, None
        last = int(last_ns)
        if isinstance(baseline, ImuBaseline):
            self._prev = (baseline.serial, int(baseline.lo), last)
            if self._gen is not _NO_GENERATION and baseline.gen != self._gen:
                self._trip = "watch_imu_gap"  # the generation changed after the baseline was taken
        else:
            kept = next((s for s in reversed(self.samples) if s[0] == last), None)
            self._prev = (None, last, last) if kept is None else (kept[4], kept[2], kept[0])
        self._tilt_start, self._tilt_n = None, 0
        self._arm_until = None  # samples taken up to this instant meet the ARMING rule (None: all, still arming)

    def settled(self, boundary_ns: int) -> None:
        """The ARM was answered (the permit seen READY at ``boundary_ns``): samples that may be taken up to then meet
        the ARMING rule, later ones the tilt hold -- by acquisition time, whichever read brings them (5b round 8 P2).
        The first call sets it; later calls change nothing."""
        if self._arm_until is None:
            self._arm_until = int(boundary_ns)

    def disarm(self) -> None:
        self._baseline = self._trip = self._prev = self._tilt_start = None
        self._tilt_n = 0
        self._arm_until = None

    @property
    def armed(self) -> bool:
        return self._baseline is not None

    def judge(self, now_ns: int) -> str | None:
        """With a frozen baseline: ``watch_imu_stale``, ``watch_imu_gap`` or ``watch_tilt`` (each kept until the
        disarm), else None. The samples received since the last call are taken in order into an incremental state
        (the previous sample, the tilt hold's start): a hold of any length is measured, whatever the window
        (13th draft, round 2 P1-1)."""
        if self._baseline is None:
            return None
        if self._trip is not None:
            return self._trip
        c = self.cfg
        for hi, u, lo, _gen, serial in self.samples:
            p_serial, p_lo, p_hi = self._prev
            if (serial <= p_serial) if p_serial is not None else (hi <= p_hi):
                continue  # judged already (or the baseline's own)
            if hi - p_lo > c.gap_max_s * 1e9:
                self._trip = "watch_imu_gap"  # sparse samples cannot carry a hold: the watch is lost
                return self._trip
            self._prev = (serial, lo, hi)
            arming = self._arm_until is None or lo <= self._arm_until
            if arming and _angle_deg(u, self._baseline) > c.baseline_spread_deg:
                # the truck moved between the baseline and the ARM's answer -- the ARM's own first byte included
                # (5b round 6 P2-1): the baseline no longer says where it stands
                self._trip = "watch_imu_moved"
                return self._trip
            if _angle_deg(u, self._baseline) > c.limit_deg:
                self._tilt_start = lo if self._tilt_start is None else self._tilt_start
                self._tilt_n += 1
                # a hold needs two distinct tilted samples (5b 4): one wide sample alone never makes it (Codex 7b
                # round 1 P2)
                if self._tilt_n >= 2 and hi - self._tilt_start >= c.hold_s * 1e9:
                    self._trip = "watch_tilt"
                    return self._trip
            else:
                self._tilt_start, self._tilt_n = None, 0
        if int(now_ns) - self._prev[1] > c.max_age_s * 1e9:
            self._trip = "watch_imu_stale"
        return self._trip


def _angle_deg(u, v) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(u, v))))))
