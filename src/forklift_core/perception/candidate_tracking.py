# src/forklift_core/perception/candidate_tracking.py
"""Far-range candidate tracking and multi-frame fusion (design §6, §8; stage 2B-3 plan).

Frames carry their acquisition time and the odometry pose at that time. Counted
candidates are moved into odom and associated with the *last accepted observation*
of each live track, with the near-field tracker's gate (`near_field_tracking.py`):
both single-frame bounds, the odometry bound for the elapsed time, its lever and
the axis-mixing term. A component of the gate graph is applied only when its
maximum matching is unique; otherwise its tracks are held (not refreshed) and its
observations start no track. A track expires 0.5 s after its last accepted
observation and is never revived, so a lost target is not silently replaced.

Fusion is over the rigid pocket-pair pose -- the pair midpoint and the insertion
yaw -- in a window W, weighted by 1/(sigma^2 + o^2) with o the odometry bound at
the observation's age. The latest reading's local layout (pocket spacing, face,
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

    def __post_init__(self) -> None:
        if set(self.rows) != set(FAMILIES):
            raise ValueError(f"rows must cover {FAMILIES}")
        for family, rows in self.rows.items():
            rows = tuple(rows)
            if not rows or any(b.lo_m < a.hi_m for a, b in itertools.pairwise(rows)):
                raise ValueError(f"{family} rows must be non-empty, sorted and non-overlapping")
            self.rows[family] = rows
        ages = tuple(float(a) for a in self.ages_s)
        scalars = (*ages, *self.e_m, *self.psi_rad, *self.rear_axle_xy_m, self.window_s, self.expiry_s)
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
        if not (self.window_s >= 0 and self.expiry_s > 0 and self.max_observations >= 1):
            raise ValueError("window_s >= 0, expiry_s > 0 and max_observations >= 1 are required")

    @classmethod
    def from_dict(cls, data: dict) -> TrackBounds:
        odo = data["odometry"]
        rows = {family: tuple(BoundRow(**r) for r in data["rows"][family]) for family in FAMILIES}
        return cls(rows=rows, ages_s=tuple(odo["ages_s"]), e_m=tuple(odo["e_m"]), psi_rad=tuple(odo["psi_rad"]),
                   rear_axle_xy_m=tuple(data["rear_axle_xy_m"]), window_s=float(data["window_s"]),
                   max_observations=int(data["max_observations"]), expiry_s=float(data["expiry_s"]))

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


@dataclass(frozen=True)
class TrackedCandidate:
    track_id: int
    candidate: PocketCandidate  # in base_link at the query time
    observations: int  # observations fused (in the window)
    budget: dict  # lateral_m, along_m (each pocket), yaw_rad, height_m
    age_s: float  # query time minus the last accepted observation's acquisition time
    resets: int
    holds: int


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

    @property
    def last(self) -> _Observation:
        return self.history[-1]


def _gate(ref: _Observation, new: _Observation, bounds: TrackBounds) -> bool:
    """The near-field comparison (near_field_tracking.py `_compare`) plus family, layout and support."""
    if ref.key != new.key:
        return False
    odometry = bounds.odometry((new.stamp_ns - ref.stamp_ns) / 1e9)
    if odometry is None:
        return False
    e, psi = odometry
    a = np.array([math.cos(ref.yaw), math.sin(ref.yaw)])
    n = np.array([-a[1], a[0]])
    mixing = ref.row.yaw_rad + psi
    for side in ("left", "right"):
        d = getattr(new, side)[:2] - getattr(ref, side)[:2]
        lever = bounds.lever_m(getattr(new.candidate, side).centre_m)
        t_lat = ref.row.lateral_m + new.row.lateral_m + e + lever * psi
        t_along = ref.row.along_m + new.row.along_m + e + lever * psi
        if abs(float(d @ n)) > t_lat + t_along * mixing or abs(float(d @ a)) > t_along + t_lat * mixing:
            return False
    if abs(_wrap(new.yaw - ref.yaw)) > ref.row.yaw_rad + new.row.yaw_rad + psi:
        return False
    if abs(new.support_z - ref.support_z) > ref.row.support_m + new.row.support_m:
        return False
    layout = ref.row.layout_m + new.row.layout_m
    return all(abs(a - b) <= layout for a, b in zip(new.offsets, ref.offsets))


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
            w = 1.0 / (o.row.sigma_m ** 2 + reach ** 2)
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
    def _live(self, now_ns: int) -> list[_Track]:
        return [t for t in self._tracks if (now_ns - t.last.stamp_ns) / 1e9 <= self.bounds.expiry_s]

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
        self._tracks = self._live(frame.stamp_ns)  # expired tracks are never revived

        obs, untracked = [], 0
        for c in frame.candidates:
            o = _Observation.make(c, frame, self.bounds)
            if o is None:
                untracked += 1
            else:
                obs.append(o)
        tracks = self._tracks
        edges = {(i, j) for i, t in enumerate(tracks) for j, o in enumerate(obs) if _gate(t.last, o, self.bounds)}

        # gate components
        parent = {("t", i): ("t", i) for i in range(len(tracks))} | {("o", j): ("o", j) for j in range(len(obs))}

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

        accepted, held, blocked = {}, 0, set()
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
            blocked.update(os_)  # gated observations never start a track
            if best is None:
                held += len(ts)
                for i in ts:
                    tracks[i].holds += 1
                continue
            for i, j in sorted(best):
                track, o = tracks[i], obs[j]
                if self._conflicts(track, o):
                    track.history = []
                    track.resets += 1
                track.history.append(o)
                track.history = self._window(track)  # the stored history never outgrows W and the count
                accepted[o.candidate.candidate_id] = track.id

        new = 0
        fresh = sorted((o for j, o in enumerate(obs) if j not in blocked),
                       key=lambda o: (round(float(o.mid[0]), 9), round(float(o.mid[1]), 9), o.key))
        for o in fresh:
            self._tracks.append(_Track(self._next_id, o))
            accepted[o.candidate.candidate_id] = self._next_id
            self._next_id += 1
            new += 1
        return FrameReport(accepted=accepted, new=new, held=held, untracked=untracked, cleared=cleared)

    # Output ------------------------------------------------------------------------------------
    def tracks(self, now_ns: int, odom_from_base_now: RigidTransform) -> tuple:
        if self._last_stamp is not None and now_ns < self._last_stamp:
            raise ValueError("the query time is before the last processed frame")
        robot_yaw = _planar_yaw(odom_from_base_now)
        self._tracks = self._live(now_ns)  # an expiry seen by a query is permanent: a late frame cannot revive it
        out = []
        for track in sorted(self._tracks, key=lambda t: t.id):
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
                                        track.resets, track.holds))
        return tuple(out)


__all__ = ["BoundRow", "CandidateTracker", "Frame", "FrameReport", "TrackBounds", "TrackedCandidate"]
