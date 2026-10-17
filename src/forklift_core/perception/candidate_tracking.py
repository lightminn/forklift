# src/forklift_core/perception/candidate_tracking.py
"""Far-range candidate tracking and multi-frame fusion (design §6, §8; stage 2B-3 plan).

Frames carry their acquisition time and the odometry pose at that time. Counted
candidates are moved into odom and associated with the *last accepted observation*
of each live track, with the near-field tracker's gate (`near_field_tracking.py`):
both single-frame bounds, the odometry bound for the elapsed time, its lever and
the axis-mixing term. A component of the gate graph is applied only when its
maximum matching is unique; otherwise its tracks are held (not refreshed) and its
observations start no track. A track is active (output, selectable) up to 0.5 s
after its last accepted observation, then stale up to keep_s (hidden, but a
re-observation of the same pallet that passes the gate revives it with its id --
design §8's re-observation, not a replacement), then deleted. Active tracks are
matched first; observations tied to them never go to stale tracks. A parametric
reading may join any track; a known reading must match the track's first known
hypothesis; each pocket's face coordinate must overlap the interval every accepted
observation allows (2B-3b plan).

Fusion is over the rigid pocket-pair pose -- the pair midpoint and the insertion
yaw -- in a window W, weighted by 1/(sigma^2 + o^2) with o the assumed odometry
noise at the observation's age (weight_model; the worst-case bound if absent). The latest reading's local layout (pocket spacing, face,
centre) is kept and moved by one rigid transform, so every field still describes
one reading. The budget is the weighted mean of each observation's bound plus its
odometry terms: an empirical budget built from p95 rows, not a probability bound.
"""

from __future__ import annotations

import bisect
import itertools
import math
from dataclasses import dataclass, field, replace

import numpy as np

from forklift_core.geometry import RigidTransform
from forklift_core.perception.pocket_candidate import PocketCandidate

FAMILIES = ("known", "parametric")
MAX_COMPONENT = 4  # larger gate components are held rather than enumerated


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class BoundRow:
    """Single-frame p95 errors over one range bin [lo_m, hi_m) (plan §4)."""

    lo_m: float
    hi_m: float
    lateral_m: float  # each pocket, used by the gate
    along_m: float
    centre_lateral_m: float  # the pocket-pair midpoint, used by the budget
    centre_along_m: float
    centre_height_m: float
    yaw_rad: float
    support_m: float  # the support height, used by the gate only
    layout_m: float  # each pocket's offset from the pair midpoint along the face
    sigma_m: float  # random part of one observation: the fusion weight

    def __post_init__(self) -> None:
        values = [getattr(self, f) for f in self.__dataclass_fields__]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("bound rows must be finite")
        if not 0 <= self.lo_m < self.hi_m:
            raise ValueError("a bound row needs 0 <= lo_m < hi_m")
        if min(values[2:]) < 0 or self.sigma_m <= 0:
            raise ValueError("bounds must be non-negative and sigma_m positive")


@dataclass(frozen=True)
class TrackBounds:
    rows: dict
    ages_s: tuple
    e_m: tuple
    psi_rad: tuple
    rear_axle_xy_m: tuple
    window_s: float
    max_observations: int = 10
    expiry_s: float = 0.5
    keep_s: float = 1.0  # 2B-3b: a track is stale (hidden, revivable) up to keep_s, then deleted
    # 2B-3d: a new track born on a yaw / support near miss of the active selected track is a suspected
    # duplicate. Development values from seven calibration misses (yaw <= 1.23 x, support <= 2.01 x);
    # 30 deg because far-range yaw gates are already ~26 deg; 45 mm a safety cap, not a layer guarantee
    near_miss_yaw_ratio: float = 1.5
    near_miss_yaw_cap_rad: float = math.radians(30.0)
    near_miss_support_ratio: float = 2.5
    near_miss_support_cap_m: float = 0.045
    # each pocket within 0.2 m of the selected track's last reading on both axes: the odometry-widened
    # gate reaches 0.5 m at far range, while a real neighbour (0.65 m true spacing, minus two far p95
    # errors) reads >= 0.41 m away when both errors are within p95; the six active-track misses moved
    # about 54 mm at most (Codex)
    near_miss_position_cap_m: float = 0.2
    # Fusion weights only: the assumed odometry noise as random walks (position_m_per_sqrt_s,
    # yaw_rad_per_sqrt_s). None weighs with the worst-case bound, which leaves older observations
    # almost no weight (stage 2B-3 calibration). The budget always uses the bound.
    weight_model: dict | None = None

    def __post_init__(self) -> None:
        if set(self.rows) != set(FAMILIES):
            raise ValueError(f"rows must cover {FAMILIES}")
        for family, rows in self.rows.items():
            rows = tuple(rows)
            if any(b.lo_m < a.hi_m for a, b in itertools.pairwise(rows)):  # empty: that family goes untracked
                raise ValueError(f"{family} rows must be sorted and non-overlapping")
            self.rows[family] = rows
        ages = tuple(float(a) for a in self.ages_s)
        scalars = (*ages, *self.e_m, *self.psi_rad, *self.rear_axle_xy_m, self.window_s, self.expiry_s, self.keep_s,
                   self.near_miss_yaw_ratio, self.near_miss_yaw_cap_rad, self.near_miss_support_ratio,
                   self.near_miss_support_cap_m, self.near_miss_position_cap_m)
        if not all(math.isfinite(float(v)) for v in scalars) or len(self.rear_axle_xy_m) != 2:
            raise ValueError("odometry bounds, the rear axle, window_s and expiry_s must be finite")
        if not ages or ages[0] <= 0 or any(b <= a for a, b in itertools.pairwise(ages)):
            raise ValueError("odometry ages must be positive and increase")
        for name in ("e_m", "psi_rad"):
            values = tuple(float(v) for v in getattr(self, name))
            if len(values) != len(ages) or any(v < 0 for v in values) or any(b < a for a, b in itertools.pairwise(values)):
                raise ValueError(f"odometry {name} must be a non-negative running maximum over the ages")
            object.__setattr__(self, name, values)
        object.__setattr__(self, "ages_s", ages)
        if self.weight_model is not None:
            wm = {k: float(self.weight_model[k]) for k in ("position_m_per_sqrt_s", "yaw_rad_per_sqrt_s")}
            if not all(math.isfinite(v) and v >= 0 for v in wm.values()):
                raise ValueError("weight_model values must be finite and non-negative")
            object.__setattr__(self, "weight_model", wm)
        if not (self.window_s >= 0 and self.keep_s >= self.expiry_s > 0 and self.max_observations >= 1
                and self.near_miss_yaw_ratio >= 1 and self.near_miss_support_ratio >= 1
                and self.near_miss_yaw_cap_rad >= 0 and self.near_miss_support_cap_m >= 0
                and self.near_miss_position_cap_m >= 0):
            raise ValueError("window_s >= 0, expiry_s > 0 and max_observations >= 1 are required")

    @classmethod
    def from_dict(cls, data: dict) -> TrackBounds:
        odo = data["odometry"]
        rows = {family: tuple(BoundRow(**r) for r in data["rows"][family]) for family in FAMILIES}
        return cls(rows=rows, ages_s=tuple(odo["ages_s"]), e_m=tuple(odo["e_m"]), psi_rad=tuple(odo["psi_rad"]),
                   rear_axle_xy_m=tuple(data["rear_axle_xy_m"]), window_s=float(data["window_s"]),
                   max_observations=int(data["max_observations"]), expiry_s=float(data["expiry_s"]),
                   weight_model=data.get("weight_model"), keep_s=float(data.get("keep_s", 1.0)),
                   near_miss_yaw_ratio=float(data.get("near_miss_yaw_ratio", 1.5)),
                   near_miss_yaw_cap_rad=float(data.get("near_miss_yaw_cap_rad", math.radians(30.0))),
                   near_miss_support_ratio=float(data.get("near_miss_support_ratio", 2.5)),
                   near_miss_support_cap_m=float(data.get("near_miss_support_cap_m", 0.045)),
                   near_miss_position_cap_m=float(data.get("near_miss_position_cap_m", 0.2)))

    @classmethod
    def load(cls, path) -> TrackBounds:
        import yaml  # noqa: PLC0415 -- only the config loader needs it

        with open(path, encoding="utf-8") as fh:
            return cls.from_dict(yaml.safe_load(fh))

    def row(self, family: str, distance_m: float) -> BoundRow | None:
        for r in self.rows[family]:
            if r.lo_m <= distance_m < r.hi_m:
                return r
        return None

    def odometry(self, age_s: float) -> tuple[float, float] | None:
        """The bound at the first tabulated age >= age_s (the near-field lookup), None past the table."""
        if not math.isfinite(age_s) or age_s < 0:
            raise ValueError("age must be finite and non-negative")
        if age_s == 0:
            return 0.0, 0.0
        i = bisect.bisect_left(self.ages_s, age_s - 1e-9)
        if i >= len(self.ages_s):
            return None
        return self.e_m[i], self.psi_rad[i]

    def lever_m(self, point_base) -> float:
        return math.hypot(point_base[0] - self.rear_axle_xy_m[0], point_base[1] - self.rear_axle_xy_m[1])


@dataclass(frozen=True)
class Frame:
    stamp_ns: int
    clock_domain: str
    odom_epoch: int
    odom_from_base: RigidTransform
    candidates: tuple


@dataclass(frozen=True)
class FrameReport:
    accepted: dict = field(default_factory=dict)  # candidate_id -> track id
    new: int = 0
    held: int = 0  # tracks not refreshed because their gate component had no unique matching
    untracked: int = 0  # uncounted candidates and distances outside the bound table
    ignored: str | None = None
    cleared: bool = False
    revived: int = 0  # stale tracks that took an observation and became active again
    absorbed: int = 0  # suspected duplicates removed when the selected track took their observation (2B-3e)


@dataclass(frozen=True)
class TrackedCandidate:
    track_id: int
    candidate: PocketCandidate  # in base_link at the query time
    observations: int  # observations fused (in the window)
    budget: dict  # lateral_m, along_m (each pocket), yaw_rad, height_m
    age_s: float  # query time minus the last accepted observation's acquisition time
    resets: int
    holds: int
    revivals: int = 0


def _planar_yaw(t: RigidTransform) -> float:
    r = np.asarray(t.rotation)
    if abs(r[2, 2] - 1) > 1e-9 or np.abs(r[2, :2]).max() > 1e-9 or np.abs(r[:2, 2]).max() > 1e-9:
        raise ValueError("odom_from_base must be planar (a rotation about z)")
    return math.atan2(r[1, 0], r[0, 0])


def _apply(t: RigidTransform, p) -> np.ndarray:
    return np.asarray(t.rotation) @ np.asarray(p, float) + np.asarray(t.translation_m)


def _inverse(t: RigidTransform, p) -> np.ndarray:
    return np.asarray(t.rotation).T @ (np.asarray(p, float) - np.asarray(t.translation_m))


@dataclass(frozen=True)
class _Observation:
    stamp_ns: int
    candidate: PocketCandidate
    odom_from_base: RigidTransform
    family: str
    key: str
    row: BoundRow
    left: np.ndarray  # odom
    right: np.ndarray
    mid: np.ndarray
    yaw: float
    support_z: float
    spacing: float
    offsets: tuple  # each pocket's signed coordinate along the face from the face centre (left, right)

    @classmethod
    def make(cls, c: PocketCandidate, frame: Frame, bounds: TrackBounds):
        family = "parametric" if c.hypothesis.startswith("parametric") else "known"
        row = bounds.row(family, c.range_m)
        if not c.counted or row is None:
            return None
        t = frame.odom_from_base
        left, right = _apply(t, c.left.centre_m), _apply(t, c.right.centre_m)
        lb, rb = np.asarray(c.left.centre_m), np.asarray(c.right.centre_m)
        n = np.array([-math.sin(c.insertion_yaw_rad), math.cos(c.insertion_yaw_rad)])
        face = np.asarray(c.face_centre_m[:2])
        return cls(frame.stamp_ns, c, t, family, "parametric" if family == "parametric" else c.hypothesis, row,
                   left, right, (left + right) / 2, _wrap(c.insertion_yaw_rad + _planar_yaw(t)),
                   float(_apply(t, (*c.face_centre_m[:2], c.support_z_m))[2]),
                   float(np.linalg.norm(lb[:2] - rb[:2])),
                   (float((lb[:2] - face) @ n), float((rb[:2] - face) @ n)))


class _Track:
    def __init__(self, track_id: int, obs: _Observation):
        self.id = track_id
        self.history = [obs]
        self.resets = 0
        self.holds = 0
        self.revivals = 0
        # the first known-shape hypothesis this track accepted: a known reading must match it, a
        # parametric reading of the same pocket pair may join any track (2B-3b)
        self.known = obs.key if obs.family == "known" else None
        # each pocket's face coordinate interval every accepted observation allows: a new observation
        # must overlap all of them, so a pocket cannot creep to another opening through small steps
        # (2B-3b; kept across history resets)
        self.layout = [(x - obs.row.layout_m, x + obs.row.layout_m) for x in obs.offsets]
        # the selected track this one may duplicate: born where the selected track's pose gate holds
        # but split off by a reading test (2B-3c)
        self.duplicate_of: int | None = None
        # why: "reading" (split off by family / layout -- carries a reading the selected track refuses) or
        # "near_miss" (yaw / support near miss -- the same reading); only near misses are absorbed (2B-3e)
        self.duplicate_kind: str | None = None
        # 2B-3e: an absorbed duplicate becomes a shadow -- hidden from output and selection, never accepts,
        # but until keep_s it still holds any reading another track would take from it (its reading and pose
        # hypotheses are kept for ambiguity; deleting them handed readings to neighbours -- Codex)
        self.shadow = False

    def accept(self, obs: _Observation) -> None:
        self.history.append(obs)
        if self.known is None and obs.family == "known":
            self.known = obs.key
        b = obs.row.layout_m
        self.layout = [(max(lo, x - b), min(hi, x + b)) for (lo, hi), x in zip(self.layout, obs.offsets)]

    def layout_overlaps(self, obs: _Observation) -> bool:
        b = obs.row.layout_m
        return all(max(lo, x - b) <= min(hi, x + b) for (lo, hi), x in zip(self.layout, obs.offsets))

    @property
    def last(self) -> _Observation:
        return self.history[-1]


def _layout_fits(a: _Observation, b: _Observation) -> bool:
    layout = a.row.layout_m + b.row.layout_m
    return all(abs(x - y) <= layout for x, y in zip(a.offsets, b.offsets))


def _gate(track: _Track, new: _Observation, bounds: TrackBounds) -> bool:
    """The reading tests (family compatibility, each pocket's face coordinate against the last
    observation and the interval every accepted observation allows) and the pose tests."""
    return _reading_fits(track, new) and _pose_fits(track, new, bounds)


def _reading_fits(track: _Track, new: _Observation) -> bool:
    if new.family == "known" and track.known is not None and new.key != track.known:
        return False
    return _layout_fits(track.last, new) and track.layout_overlaps(new)


def _pose_terms(track: _Track, new: _Observation, bounds: TrackBounds):
    """(position fits, yaw difference, yaw gate, support difference, support gate, largest pocket step
    along either axis) against the track's last accepted observation, or None past the odometry table."""
    ref = track.last
    odometry = bounds.odometry((new.stamp_ns - ref.stamp_ns) / 1e9)
    if odometry is None:
        return None
    e, psi = odometry
    a = np.array([math.cos(ref.yaw), math.sin(ref.yaw)])
    n = np.array([-a[1], a[0]])
    mixing = ref.row.yaw_rad + psi
    position, step = True, 0.0
    for side in ("left", "right"):
        d = getattr(new, side)[:2] - getattr(ref, side)[:2]
        lever = bounds.lever_m(getattr(new.candidate, side).centre_m)
        t_lat = ref.row.lateral_m + new.row.lateral_m + e + lever * psi
        t_along = ref.row.along_m + new.row.along_m + e + lever * psi
        if abs(float(d @ n)) > t_lat + t_along * mixing or abs(float(d @ a)) > t_along + t_lat * mixing:
            position = False
        step = max(step, abs(float(d @ n)), abs(float(d @ a)))
    return (position, abs(_wrap(new.yaw - ref.yaw)), ref.row.yaw_rad + new.row.yaw_rad + psi,
            abs(new.support_z - ref.support_z), ref.row.support_m + new.row.support_m, step)


def _pose_fits(track: _Track, new: _Observation, bounds: TrackBounds) -> bool:
    """The near-field comparison (near_field_tracking.py `_compare`) against the track's last accepted
    observation: both bounds, the odometry bound for the elapsed time, its lever, the axis mixing,
    yaw and support."""
    terms = _pose_terms(track, new, bounds)
    if terms is None:
        return False
    position, dyaw, gyaw, dsup, gsup, _ = terms
    return position and dyaw <= gyaw and dsup <= gsup


def _near_miss(track: _Track, new: _Observation, bounds: TrackBounds) -> bool:
    """Position and reading fit, and yaw or support (at least one) misses its gate only slightly: yaw within
    min(ratio x gate, cap), support within ratio x gate and under its cap (2B-3d). The caller requires the
    track to be active -- a stale track's gate is widened by the odometry bound."""
    terms = _pose_terms(track, new, bounds)
    if terms is None or not _reading_fits(track, new):
        return False
    position, dyaw, gyaw, dsup, gsup, step = terms
    eps = 1e-12
    return (position and step <= bounds.near_miss_position_cap_m + eps and (dyaw > gyaw or dsup > gsup)
            and dyaw <= min(bounds.near_miss_yaw_ratio * gyaw, bounds.near_miss_yaw_cap_rad) + eps
            and dsup <= bounds.near_miss_support_ratio * gsup + eps and dsup < bounds.near_miss_support_cap_m)


def _matchings(tracks, obs, edges):
    """Every matching of tracks to observations within `edges` (a set of (track, obs) index pairs)."""
    out = []

    def rec(i, used, chosen):
        if i == len(tracks):
            out.append(frozenset(chosen))
            return
        rec(i + 1, used, chosen)
        for j in obs:
            if j not in used and (tracks[i], j) in edges:
                rec(i + 1, used | {j}, chosen + [(tracks[i], j)])

    rec(0, frozenset(), [])
    return out


class CandidateTracker:
    def __init__(self, bounds: TrackBounds):
        self.bounds = bounds
        self._tracks: list[_Track] = []
        self._next_id = 0
        self._last_stamp: int | None = None
        self._clock: str | None = None
        self._epoch: int | None = None
        self.selected: int | None = None  # the track target selection follows (2B-3c, user decision B)

    def select(self, track_id: int | None) -> None:
        """Name the track target selection follows; it is matched before every other track, stale or
        not. None clears it. A deleted selected track clears the selection (no automatic replacement)."""
        if track_id is not None and all(t.id != track_id or t.shadow for t in self._tracks):
            raise ValueError(f"no live or stale track {track_id}")
        self.selected = track_id

    def _prune(self, now_ns: int) -> None:
        self._tracks = self._kept(now_ns)
        if self.selected is not None and all(t.id != self.selected for t in self._tracks):
            self.selected = None

    # Fusion ------------------------------------------------------------------------------------
    def _window(self, track: _Track) -> list[_Observation]:
        latest = track.last.stamp_ns
        inside = [o for o in track.history if (latest - o.stamp_ns) / 1e9 <= self.bounds.window_s + 1e-12]
        return inside[-self.bounds.max_observations:]

    def _fuse(self, track: _Track, now_ns: int, base_now: RigidTransform):
        """The fused pair pose (odom midpoint, odom yaw), the observations used and the budget."""
        rows = []
        for o in self._window(track):
            odometry = self.bounds.odometry((now_ns - o.stamp_ns) / 1e9)
            if odometry is None:
                continue
            e, psi = odometry
            lever = self.bounds.lever_m(_inverse(base_now, o.mid))
            reach = e + lever * psi
            wm = self.bounds.weight_model
            if wm is None:
                spread = reach
            else:
                root = math.sqrt((now_ns - o.stamp_ns) / 1e9)
                spread = (wm["position_m_per_sqrt_s"] + lever * wm["yaw_rad_per_sqrt_s"]) * root
            w = 1.0 / (o.row.sigma_m ** 2 + spread ** 2)
            rows.append((o, w, reach, psi))
        if not rows:
            return None
        latest = track.last
        wsum = sum(w for _, w, _, _ in rows)
        mid = sum(w * o.mid for o, w, _, _ in rows) / wsum
        yaw = _wrap(latest.yaw + sum(w * _wrap(o.yaw - latest.yaw) for o, w, _, _ in rows) / wsum)
        lat_c = sum(w * (o.row.centre_lateral_m + r) for o, w, r, _ in rows) / wsum
        along_c = sum(w * (o.row.centre_along_m + r) for o, w, r, _ in rows) / wsum
        b_yaw = sum(w * (o.row.yaw_rad + p) for o, w, _, p in rows) / wsum
        b_h = sum(w * o.row.centre_height_m for o, w, _, _ in rows) / wsum
        half = latest.spacing / 2
        budget = {
            "centre_lateral_m": lat_c,
            "centre_along_m": along_c,
            "lateral_m": lat_c + latest.row.layout_m + half * (1 - math.cos(b_yaw)),
            "along_m": along_c + half * math.sin(min(b_yaw, math.pi / 2)),
            "yaw_rad": b_yaw,
            "height_m": b_h,
        }
        return mid, yaw, len(rows), budget

    def _conflicts(self, track: _Track, new: _Observation) -> bool:
        fused = self._fuse(track, new.stamp_ns, new.odom_from_base)
        if fused is None:
            return True
        mid, yaw, _, b = fused
        a = np.array([math.cos(new.yaw), math.sin(new.yaw)])
        n = np.array([-a[1], a[0]])
        d = new.mid - mid
        t_lat = new.row.centre_lateral_m + b["centre_lateral_m"]
        t_along = new.row.centre_along_m + b["centre_along_m"]
        mixing = new.row.yaw_rad + b["yaw_rad"]  # the comparison axis against each estimate's own axis (as the gate)
        return (abs(float(d[:2] @ n)) > t_lat + t_along * mixing
                or abs(float(d[:2] @ a)) > t_along + t_lat * mixing
                or abs(_wrap(new.yaw - yaw)) > new.row.yaw_rad + b["yaw_rad"]
                or abs(float(d[2])) > new.row.centre_height_m + b["height_m"])

    # Frames ------------------------------------------------------------------------------------
    def _age(self, track: _Track, now_ns: int) -> float:
        return (now_ns - track.last.stamp_ns) / 1e9

    def _live(self, now_ns: int) -> list[_Track]:
        """Active tracks: the ones selection may read (design §6)."""
        return [t for t in self._tracks if self._age(t, now_ns) <= self.bounds.expiry_s]

    def _kept(self, now_ns: int) -> list[_Track]:
        """Active and stale tracks; anything older is deleted for good."""
        return [t for t in self._tracks if self._age(t, now_ns) <= self.bounds.keep_s]

    def _match(self, tracks, obs, free, skip=frozenset()):
        """Unique maximum matching per gate component over the observations in `free`; (track id, obs
        index) pairs in `skip` get no edge. Returns (matched (track, obs index) pairs, held tracks, gated
        observation indices)."""
        edges = {(i, j) for i, t in enumerate(tracks) for j in free
                 if (t.id, j) not in skip and _gate(t, obs[j], self.bounds)}
        parent = {("t", i): ("t", i) for i in range(len(tracks))} | {("o", j): ("o", j) for j in free}

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for i, j in edges:
            parent[find(("t", i))] = find(("o", j))
        components: dict = {}
        for node in parent:
            components.setdefault(find(node), []).append(node)
        pairs, held, gated = [], [], set()
        for nodes in components.values():
            ts = sorted(i for k, i in nodes if k == "t")
            os_ = sorted(j for k, j in nodes if k == "o")
            if not ts or not os_:
                continue
            best = None
            if len(ts) <= MAX_COMPONENT and len(os_) <= MAX_COMPONENT:
                ms = _matchings(ts, os_, edges)
                size = max(len(m) for m in ms)
                top = [m for m in ms if len(m) == size]
                best = top[0] if len(top) == 1 else None
            gated.update(os_)  # gated observations never start a track
            if best is None:
                held += [tracks[i] for i in ts]
                for i in ts:
                    tracks[i].holds += 1
                continue
            pairs += [(tracks[i], j) for i, j in sorted(best)]
        return pairs, held, gated

    def update(self, frame: Frame) -> FrameReport:
        ids = [c.candidate_id for c in frame.candidates]
        if len(set(ids)) != len(ids):
            raise ValueError("candidate ids must be unique within a frame")
        if any(c.stamp_ns != frame.stamp_ns for c in frame.candidates):
            raise ValueError("every candidate must carry the frame's acquisition stamp")
        _planar_yaw(frame.odom_from_base)
        cleared = False
        if self._clock is not None and (frame.clock_domain != self._clock or frame.odom_epoch != self._epoch):
            self._tracks, self._last_stamp, cleared = [], None, True
        if self._last_stamp is not None and frame.stamp_ns <= self._last_stamp:
            return FrameReport(ignored="not_newer")
        self._last_stamp, self._clock, self._epoch = frame.stamp_ns, frame.clock_domain, frame.odom_epoch
        self._prune(frame.stamp_ns)  # older than keep_s: deleted for good

        obs, untracked = [], 0
        for c in frame.candidates:
            o = _Observation.make(c, frame, self.bounds)
            if o is None:
                untracked += 1
            else:
                obs.append(o)
        # stages: the selected track first (2B-3c), then the other active tracks, then the remaining
        # observations against the other stale tracks, so a stale track's wide gate never holds an
        # active track's observation (2B-3b, agy review)
        chosen = next((tr for tr in self._tracks if tr.id == self.selected), None)
        # judged against the selected track as it was before this frame's matching moves it: inside its
        # pose gate but refused by a reading test (Codex: judging after the update marks real neighbours)
        chosen_active = chosen is not None and self._age(chosen, frame.stamp_ns) <= self.bounds.expiry_s
        split_off = {}
        for j, o in enumerate(obs):
            if chosen is None:
                break
            if _pose_fits(chosen, o, self.bounds) and not _reading_fits(chosen, o):
                split_off[j] = "reading"
            elif chosen_active and _near_miss(chosen, o, self.bounds):  # 2B-3d
                split_off[j] = "near_miss"
        # 2B-3e: before matching, which of the selected track's suspected duplicates each reading fits
        duplicates = [] if chosen is None else [tr for tr in self._tracks
                                                if tr.duplicate_of == chosen.id and not tr.shadow]
        dup_fits = {j: [tr for tr in duplicates if _gate(tr, o, self.bounds)] for j, o in enumerate(obs)}
        taken_by_chosen, held_now = [], set()
        free = set(range(len(obs)))
        accepted, held, revived, blocked = {}, 0, 0, set()
        priority = None
        if chosen is not None:
            fits = sorted(j for j in free if _gate(chosen, obs[j], self.bounds))
            if len(fits) == 1:
                rivals = [tr for tr in self._tracks if tr is not chosen and _gate(tr, obs[fits[0]], self.bounds)]
                if all(tr.duplicate_of == chosen.id for tr in rivals):
                    priority = fits[0]  # the selected track wins only against its own suspected duplicates
        # 2B-3e shadows compete in the active stage like tracks but never accept: a reading a unique
        # matching gives to a shadow is held. A shadow never competes with the selected track it was
        # absorbed into for a reading that track fits (only its owner is exempt -- switching the selection
        # keeps an old target's shadows competing); every other competition stays as in 2B-3b
        shadows = [tr for tr in self._tracks if tr.shadow]
        chosen_fits = set() if chosen is None else {j for j, o in enumerate(obs) if _gate(chosen, o, self.bounds)}
        skip = {(sh.id, j) for sh in shadows if sh.duplicate_of == self.selected for j in chosen_fits}
        shadow_fits = {j for j, o in enumerate(obs) if any(_gate(sh, o, self.bounds) for sh in shadows)}
        live = [tr for tr in self._live(frame.stamp_ns) if not tr.shadow]
        if priority is None:  # the selected track (if any) takes its natural stage under the 2B-3b rules
            active = live
            stages = []
        else:
            active = [tr for tr in live if tr is not chosen]
            stages = [("selected", [chosen])]
        stale = [tr for tr in self._tracks if tr not in active and not tr.shadow
                 and not (priority is not None and tr is chosen)]
        stages += [("active", active + shadows), ("stale", stale)]
        for stage, tracks in stages:
            # the selected stage sees only its one observation; the others see what is still free
            pairs, h, gated = self._match(tracks, obs, {priority} if stage == "selected" else free, skip=skip)
            held += len(h)
            held_now.update(tr.id for tr in h)
            blocked |= gated
            free -= gated
            for track, j in pairs:
                if track.shadow:  # a shadow never accepts: the reading is held
                    held += 1
                    held_now.add(track.id)
                    continue
                o = obs[j]
                was_stale = self._age(track, frame.stamp_ns) > self.bounds.expiry_s
                if self._conflicts(track, o):
                    track.history = []
                    track.resets += 1
                track.accept(o)
                track.history = self._window(track)  # the stored history never outgrows W and the count
                accepted[o.candidate.candidate_id] = track.id
                if track is chosen:
                    taken_by_chosen.append(j)
                if was_stale:
                    track.revivals += 1
                    revived += 1

        took_other = set(accepted.values())  # a duplicate that took another reading this frame tracks something
        absorbed = {tr.id for j in taken_by_chosen for tr in dup_fits[j]
                    if tr is not chosen and tr.duplicate_kind == "near_miss"
                    and tr.id not in took_other and tr.id not in held_now}
        for tr in self._tracks:
            if tr.id in absorbed:
                tr.shadow = True

        new = 0
        fresh = sorted((j for j in range(len(obs)) if j not in blocked and j not in shadow_fits),
                       key=lambda j: (round(float(obs[j].mid[0]), 9), round(float(obs[j].mid[1]), 9), obs[j].key))
        for j in fresh:
            o = obs[j]
            track = _Track(self._next_id, o)
            if j in split_off:
                track.duplicate_of = chosen.id  # where the selected track was, split off by a reading test
                track.duplicate_kind = split_off[j]
            self._tracks.append(track)
            accepted[o.candidate.candidate_id] = self._next_id
            self._next_id += 1
            new += 1
        return FrameReport(accepted=accepted, new=new, held=held, untracked=untracked, cleared=cleared,
                           revived=revived, absorbed=len(absorbed))

    # Output ------------------------------------------------------------------------------------
    def stale_ids(self, now_ns: int) -> tuple:
        """Tracks hidden from selection but still revivable at now_ns (for diagnostics)."""
        return tuple(t.id for t in self._kept(now_ns) if self._age(t, now_ns) > self.bounds.expiry_s and not t.shadow)

    def tracks(self, now_ns: int, odom_from_base_now: RigidTransform) -> tuple:
        if self._last_stamp is not None and now_ns < self._last_stamp:
            raise ValueError("the query time is before the last processed frame")
        robot_yaw = _planar_yaw(odom_from_base_now)
        self._prune(now_ns)  # a query deletes only past keep_s; stale tracks stay revivable
        out = []
        for track in sorted((t for t in self._live(now_ns) if not t.shadow), key=lambda t: t.id):
            fused = self._fuse(track, now_ns, odom_from_base_now)
            if fused is None:  # every observation is older than the odometry table
                continue
            mid, yaw, used, budget = fused
            latest = track.last
            dyaw = _wrap(yaw - latest.yaw)
            c_, s_ = math.cos(dyaw), math.sin(dyaw)
            rz = np.array([[c_, -s_, 0.0], [s_, c_, 0.0], [0.0, 0.0, 1.0]])

            def move(p_base, t_latest=latest.odom_from_base):
                p = rz @ (_apply(t_latest, p_base) - latest.mid) + mid
                return tuple(float(v) for v in _inverse(odom_from_base_now, p))

            c = latest.candidate
            dz = float(mid[2] - latest.mid[2])
            moved = replace(
                c,
                left=replace(c.left, centre_m=move(c.left.centre_m)),
                right=replace(c.right, centre_m=move(c.right.centre_m)),
                face_centre_m=move(c.face_centre_m),
                pallet_centre_m=move(c.pallet_centre_m),
                centre_segment_m=tuple(move(p) for p in c.centre_segment_m),
                alternative_centres_m=tuple(move(p) for p in c.alternative_centres_m),
                observed_centre_xy=(None if c.observed_centre_xy is None
                                    else move((*c.observed_centre_xy, 0.0))[:2]),
                insertion_yaw_rad=_wrap(yaw - robot_yaw),
                support_z_m=float(_inverse(odom_from_base_now, (0.0, 0.0, latest.support_z + dz))[2]),
            )
            out.append(TrackedCandidate(track.id, moved, used, budget, (now_ns - latest.stamp_ns) / 1e9,
                                        track.resets, track.holds, track.revivals))
        return tuple(out)


__all__ = ["BoundRow", "CandidateTracker", "Frame", "FrameReport", "TrackBounds", "TrackedCandidate"]
