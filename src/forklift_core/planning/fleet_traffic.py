"""Central traffic control for trucks on planned routes: claims, ranks, safe stops.

Plan: docs/plans/2026-10-08-fleet-five-forklifts.md. Every truck follows its
own planned route (all its remaining stages, end to end); nothing in the
planner knows about the other trucks. This module keeps them apart by fixed
rules, without replanning:

- **Claims.** A truck owns the region its footprint (grown by a margin)
  sweeps along its route from where it is now to an arc length ``granted_m``,
  and may drive only inside it. A grant never shrinks while the route stays
  the same, so a truck can always drive to the end of what it was given.
- **Rank.** Each truck has a fixed rank (lower first). Each update, trucks
  extend their grants in rank order; extension stops at the first sample
  whose footprint would touch another truck's body or claim.
- **Safe stops.** A grant may end only where the truck's footprint is clear
  of the remaining route of every higher-ranked truck -- there and
  ``moving_m`` short of it, since a truck held at the end of its grant may
  come to rest a little before it (it then waits). A truck that would
  have to stop on such a route does not enter it: it waits before it until it
  can claim through to a safe stop, as a car keeps out of a junction it
  cannot clear.

Why this cannot lock: a waiting truck stands clear of every higher-ranked
truck's remaining route, so the highest-ranked unfinished truck can be held
up only by trucks driving through, or working inside, their own grants, and
those always finish. It therefore finishes, and then the next one by the
same argument. That needs every truck's start pose to be clear of the
higher-ranked trucks' routes and its final pose to be clear of the
lower-ranked ones' (``route_conflicts`` lists any that are not), and the
trucks to stay inside their grants: the margin has to cover the tracking and
localization error.

``deadlocks`` still reports waiting cycles and long stalls, for the record;
under these rules a reported cycle means a broken precondition or a bug.

``advisories`` adds resolution on top (plan 2026-10-10, TCAS-like): a truck
held by a standing truck for ``ta_s`` gets a TA -- plan round that truck's
body; a waiting cycle (after ``cycle_s``) or a wait of ``ra_s`` gets an RA for
the lower-ranked truck of the pair -- plan round the higher-ranked truck's
nearby remaining route as a wall (``wall``), or back off it (``back_to``).
The higher-ranked truck never gets an RA, so it keeps going. The caller
plans and changes the route; ``resolved`` marks a truck in RA until it is
clear again (a "cc" advisory). These are advice, not a no-lock argument.
A new route of a truck must keep clear of where the lower-ranked trucks may
stop on what they already hold (``lower_stops``, ``path_conflicts``), or it
breaks the precondition above for them.

Poses are what each truck believes (SLAM estimates). Rear-axle x, y, yaw in
metres and radians; arc lengths along the truck's route.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, hypot, sin

import numpy as np

from .geometry import Bounds, Footprint, FootprintCollisionChecker, Pose2D, Rectangle

_HUGE = Bounds(-1e6, 1e6, -1e6, 1e6)


def footprint_rectangle(pose, footprint: Footprint, margin_m: float = 0.0) -> Rectangle:
    """The footprint at a rear-axle (x, y, yaw) as a Rectangle grown by margin_m."""
    x, y, yaw = (
        (float(v) for v in (pose.x_m, pose.y_m, pose.yaw_rad))
        if isinstance(pose, Pose2D)
        else (float(v) for v in pose)
    )
    offset = (footprint.front_m - footprint.rear_m) / 2
    return Rectangle(
        x + offset * cos(yaw),
        y + offset * sin(yaw),
        footprint.front_m + footprint.rear_m + 2 * margin_m,
        2 * (footprint.half_width_m + margin_m),
        yaw,
    )


def arc_lengths(poses: np.ndarray) -> np.ndarray:
    """Cumulative distance along a (N, 3) path, 0 at the first pose."""
    poses = np.asarray(poses, dtype=float)
    if len(poses) < 2:
        return np.zeros(len(poses))
    return np.concatenate(
        ([0.0], np.cumsum(np.hypot(*np.diff(poses[:, :2], axis=0).T)))
    )


def pose_at(poses: np.ndarray, s: np.ndarray, distance_m: float) -> np.ndarray:
    """Pose at arc length distance_m (clamped), interpolating position and yaw."""
    distance_m = float(np.clip(distance_m, 0.0, s[-1]))
    i = int(np.searchsorted(s, distance_m, side="right")) - 1
    i = min(max(i, 0), len(s) - 2) if len(s) > 1 else 0
    if len(s) == 1 or s[i + 1] == s[i]:
        return poses[i].copy()
    t = (distance_m - s[i]) / (s[i + 1] - s[i])
    a, b = poses[i], poses[i + 1]
    turn = (b[2] - a[2] + np.pi) % (2 * np.pi) - np.pi
    return np.array(
        [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * turn]
    )


def oriented_rects(poses, footprints, margin_m: float) -> np.ndarray:
    """(N, 6) centre x, y, half length, half width, cos, sin of grown footprints."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    if isinstance(footprints, Footprint):
        footprints = [footprints] * len(poses)
    front = np.array([f.front_m for f in footprints])
    rear = np.array([f.rear_m for f in footprints])
    half = np.array([f.half_width_m for f in footprints])
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    offset = (front - rear) / 2
    return np.column_stack(
        (
            poses[:, 0] + offset * c,
            poses[:, 1] + offset * s,
            (front + rear) / 2 + margin_m,
            half + margin_m,
            c,
            s,
        )
    )


def _reach(rects: np.ndarray) -> np.ndarray:
    return np.hypot(rects[:, 2], rects[:, 3])


def _near(rects: np.ndarray, others: np.ndarray) -> np.ndarray:
    """Indices of rects that can touch any of others (bounding-box test)."""
    if not len(rects) or not len(others):
        return np.zeros(0, dtype=int)
    r = _reach(others)
    lo = (others[:, :2] - r[:, None]).min(axis=0)
    hi = (others[:, :2] + r[:, None]).max(axis=0)
    own = _reach(rects)[:, None]
    inside = np.all((rects[:, :2] + own >= lo) & (rects[:, :2] - own <= hi), axis=1)
    return np.nonzero(inside)[0]


def overlap_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """(len(a), len(b)) closed-rectangle intersections by the separating-axis test."""
    out = np.zeros((len(a), len(b)), dtype=bool)
    if not len(a) or not len(b):
        return out
    dx = b[None, :, 0] - a[:, None, 0]
    dy = b[None, :, 1] - a[:, None, 1]
    reach = np.hypot(a[:, 2], a[:, 3])[:, None] + np.hypot(b[:, 2], b[:, 3])[None, :]
    i, j = np.nonzero(dx * dx + dy * dy <= reach * reach)
    if not len(i):
        return out
    p, q, dx, dy = a[i], b[j], dx[i, j], dy[i, j]
    cos_rel = np.abs(p[:, 4] * q[:, 4] + p[:, 5] * q[:, 5])
    sin_rel = np.abs(p[:, 5] * q[:, 4] - p[:, 4] * q[:, 5])
    apart = (
        (
            np.abs(dx * p[:, 4] + dy * p[:, 5])
            > p[:, 2] + q[:, 2] * cos_rel + q[:, 3] * sin_rel
        )
        | (
            np.abs(dy * p[:, 4] - dx * p[:, 5])
            > p[:, 3] + q[:, 2] * sin_rel + q[:, 3] * cos_rel
        )
        | (
            np.abs(dx * q[:, 4] + dy * q[:, 5])
            > q[:, 2] + p[:, 2] * cos_rel + p[:, 3] * sin_rel
        )
        | (
            np.abs(dy * q[:, 4] - dx * q[:, 5])
            > q[:, 3] + p[:, 2] * sin_rel + p[:, 3] * cos_rel
        )
    )
    out[i[~apart], j[~apart]] = True
    return out


@dataclass(frozen=True)
class Permit:
    allowed_m: float
    granted_m: float
    blocked_by: str | None


@dataclass(frozen=True)
class Advisory:
    """Advice from ``advisories``: ``truck`` acts, against ``other``.

    kind "ta": plan round other's standing body (why "stall"); "ra": clear
    other's remaining route (why "cycle", "long_wait" or "in_the_way");
    "cc": truck, in RA against other, is clear again (why "clear").
    """

    kind: str
    truck: str
    other: str
    why: str


def tile_rects(rects: np.ndarray, piece_m: float) -> np.ndarray:
    """Cut (N, 6) rects into pieces about piece_m long and wide, so the part
    near a truck can be left out of a keep-out without dropping the rest."""
    out = []
    for cx, cy, hl, hw, c, s in np.asarray(rects, dtype=float).reshape(-1, 6):
        nl = max(1, int(np.ceil(2 * hl / piece_m)))
        nw = max(1, int(np.ceil(2 * hw / piece_m)))
        u = (np.arange(nl) + 0.5) * (2 * hl / nl) - hl
        v = (np.arange(nw) + 0.5) * (2 * hw / nw) - hw
        uu, vv = (a.ravel() for a in np.meshgrid(u, v))
        out.append(
            np.column_stack(
                (
                    cx + uu * c - vv * s,
                    cy + uu * s + vv * c,
                    np.full(len(uu), hl / nl),
                    np.full(len(uu), hw / nw),
                    np.full(len(uu), c),
                    np.full(len(uu), s),
                )
            )
        )
    return np.vstack(out) if out else np.zeros((0, 6))


@dataclass
class _Truck:
    name: str
    rank: int
    footprint: Footprint
    g: np.ndarray  # arc length of each route sample
    rects: np.ndarray  # (n, 6) the route samples' footprints, grown
    rest: np.ndarray  # (n, 6) the same, moving_m back along the route
    progress_m: float = 0.0
    granted_m: float = 0.0
    pose: tuple = (0.0, 0.0, 0.0)
    waiting_on: str | None = None
    waiting_since_s: float | None = None
    held_by_body: bool = False  # waiting_on's body or claim is on the way
    moving: bool = True
    claim: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))

    @property
    def key(self):
        return (self.rank, self.name)


class TrafficManager:
    """Claims along planned routes under fixed ranks; see the module docstring."""

    def __init__(
        self,
        *,
        margin_m: float = 0.3,
        lookahead_m: float = 3.0,
        sample_m: float = 0.2,
        moving_m: float = 0.05,
        scan_m: float = 80.0,
        cycle_s: float = 5.0,
        stall_s: float = 20.0,
        ta_s: float | None = None,
        ra_s: float | None = None,
        cooldown_s: float = 60.0,
    ) -> None:
        if min(lookahead_m, sample_m, moving_m, scan_m) <= 0 or margin_m < 0:
            raise ValueError("lookahead, sample, moving and scan lengths must be positive")
        if scan_m < lookahead_m:
            raise ValueError("scan_m must reach at least lookahead_m")
        self.margin_m = margin_m
        self.lookahead_m = lookahead_m
        self.sample_m = sample_m
        self.moving_m = moving_m
        self.scan_m = scan_m
        self.cycle_s = cycle_s
        self.stall_s = stall_s
        self.ta_s = ta_s
        self.ra_s = ra_s
        self.cooldown_s = cooldown_s
        self.ra_against: dict[str, str] = {}  # truck in RA -> the truck it cleared
        self._advised: dict = {}  # (kind, actor, other, why) -> the wait it was for
        self._last_advice: dict[str, float] = {}
        self.trucks: dict[str, _Truck] = {}
        # (truck, higher-ranked truck) -> per route sample of the first: the
        # last arc length on the second's route whose footprint touches it
        # (at the sample or moving_m short of it).
        self._cover: dict[tuple[str, str], np.ndarray] = {}

    # -- configuration -------------------------------------------------------
    def add(self, name: str, rank: int, pose, footprint: Footprint) -> None:
        """A truck standing at pose, with no route yet (it stays there)."""
        pose = np.asarray(pose, dtype=float).reshape(1, 3)
        self.trucks[name] = _Truck(
            name,
            rank,
            footprint,
            np.zeros(1),
            oriented_rects(pose, footprint, self.margin_m),
            oriented_rects(pose, footprint, self.margin_m),
            pose=tuple(pose[0]),
        )
        self._refresh_cover(name)

    def set_route(self, name: str, chunks, rank: int | None = None) -> list[float]:
        """The truck's whole route from where it is, as (poses, footprint) per
        stage in driving order; its old claim is released. ``rank`` replaces
        the truck's rank (a fleet that keeps giving out jobs ranks them in
        the order it gave them).

        Returns each chunk's starting arc length on the route; progress passed
        to ``update`` is measured the same way (chunk start + distance along
        the chunk's own poses).
        """
        truck = self.trucks[name]
        if rank is not None:
            truck.rank = rank
        offsets, g, poses, rests, footprints = [], [], [], [], []
        start = 0.0
        for chunk, footprint in chunks:
            chunk = np.asarray(chunk, dtype=float).reshape(-1, 3)
            s = arc_lengths(chunk)
            marks = np.arange(0.0, s[-1], self.sample_m)
            marks = np.append(marks, s[-1]) if len(marks) else np.array([s[-1]])
            offsets.append(start)
            g.extend(start + marks)
            poses.extend(pose_at(chunk, s, d) for d in marks)
            rests.extend(pose_at(chunk, s, max(d - self.moving_m, 0.0)) for d in marks)
            footprints.extend([footprint] * len(marks))
            start += float(s[-1])
        truck.g = np.asarray(g)
        truck.rects = oriented_rects(np.asarray(poses), footprints, self.margin_m)
        truck.rest = oriented_rects(np.asarray(rests), footprints, self.margin_m)
        truck.progress_m = truck.granted_m = 0.0
        truck.waiting_on = truck.waiting_since_s = None
        self._refresh_cover(name)
        return offsets

    def set_footprint(self, name: str, footprint: Footprint) -> None:
        """The body the truck has now (loaded or not)."""
        self.trucks[name].footprint = footprint

    def _refresh_cover(self, name: str) -> None:
        truck = self.trucks[name]
        for other in self.trucks.values():
            if other is truck:
                continue
            self._cover.pop((truck.name, other.name), None)
            self._cover.pop((other.name, truck.name), None)
            low, high = (truck, other) if other.key < truck.key else (other, truck)
            cover = np.full(len(low.g), -np.inf)
            for own in (low.rects, low.rest):  # at the sample, and where it may rest
                rows = _near(own, high.rects)
                cols = _near(high.rects, own[rows])
                if len(rows) and len(cols):
                    touch = overlap_matrix(own[rows], high.rects[cols])
                    cover[rows] = np.maximum(
                        cover[rows],
                        np.where(touch, high.g[cols][None, :], -np.inf).max(axis=1),
                    )
            self._cover[(low.name, high.name)] = cover

    # -- per update ----------------------------------------------------------
    def _remaining(self, truck: _Truck) -> float:
        """Route samples at or past this arc length are still ahead of the truck."""
        return truck.progress_m - self.sample_m

    def _higher(self, truck: _Truck) -> list[_Truck]:
        return [o for o in self.trucks.values() if o.key < truck.key]

    def _unsafe(self, truck: _Truck) -> np.ndarray:
        """Route samples of this truck covered by a higher-ranked remaining route."""
        unsafe = np.zeros(len(truck.g), dtype=bool)
        for other in self._higher(truck):
            unsafe |= self._cover[(truck.name, other.name)] >= self._remaining(other)
        return unsafe

    def _route_owner(self, truck: _Truck, index: int) -> str | None:
        if index >= len(truck.g):
            return None
        for other in sorted(self._higher(truck), key=lambda o: o.key):
            if self._cover[(truck.name, other.name)][index] >= self._remaining(other):
                return other.name
        return None

    def _claim(self, truck: _Truck) -> np.ndarray:
        body = oriented_rects(np.array([truck.pose]), truck.footprint, self.margin_m)
        ahead = (truck.g > truck.progress_m) & (truck.g <= truck.granted_m + 1e-9)
        return np.vstack((body, truck.rects[ahead]))

    def _extend(self, truck: _Truck) -> tuple[str | None, bool]:
        """Move the grant on to a safe stop; returns whom the truck waits for,
        and whether that truck's body or claim is in the way (rather than its
        route only)."""
        target = min(truck.g[-1], truck.progress_m + self.lookahead_m)
        if truck.granted_m >= target - 1e-9:
            return None, False
        first = int(np.searchsorted(truck.g, truck.granted_m + 1e-9, side="left"))
        last = int(
            np.searchsorted(truck.g, truck.progress_m + self.scan_m, side="right")
        )
        others = [o for o in self.trucks.values() if o is not truck]
        obstacles = np.vstack([np.zeros((0, 6))] + [o.claim for o in others])
        owner = np.concatenate(
            [np.zeros(0, dtype=int)]
            + [np.full(len(o.claim), k) for k, o in enumerate(others)]
        )
        ahead = truck.rects[first:last]
        nearby = _near(obstacles, ahead)
        obstacles, owner = obstacles[nearby], owner[nearby]
        touch = overlap_matrix(ahead, obstacles)
        hit_rows = np.nonzero(touch.any(axis=1))[0]
        free = int(hit_rows[0]) if len(hit_rows) else last - first
        g = truck.g[first : first + free]
        safe = ~self._unsafe(truck)[first : first + free]
        far = np.nonzero(safe & (g >= target - 1e-9))[0]
        near = np.nonzero(safe)[0]
        end = far[0] if len(far) else near[-1] if len(near) else None
        if end is not None:
            truck.granted_m = max(truck.granted_m, float(g[end]))
        if truck.granted_m >= target - 1e-9:
            return None, False
        if len(hit_rows):
            return others[int(owner[np.nonzero(touch[free])[0][0]])].name, True
        return self._route_owner(truck, first + (0 if end is None else end + 1)), False

    def update(self, now_s: float, poses: dict, progress: dict) -> dict[str, Permit]:
        """Trucks' believed rear-axle poses and arc lengths along their routes -> permits."""
        for name, truck in self.trucks.items():
            truck.pose = tuple(float(v) for v in poses[name])
            truck.progress_m = float(np.clip(progress[name], 0.0, truck.g[-1]))
            truck.granted_m = max(truck.granted_m, truck.progress_m)
        for truck in self.trucks.values():
            truck.claim = self._claim(truck)
        permits = {}
        for truck in sorted(self.trucks.values(), key=lambda t: t.key):
            blocked_by, by_body = self._extend(truck)
            truck.claim = self._claim(truck)
            allowed = truck.granted_m - truck.progress_m
            at_end = truck.progress_m >= truck.g[-1] - 1e-9
            waiting = blocked_by is not None and allowed < self.moving_m and not at_end
            if waiting:
                if truck.waiting_on != blocked_by:
                    truck.waiting_on, truck.waiting_since_s = blocked_by, now_s
                truck.held_by_body = by_body
            else:
                truck.waiting_on, truck.waiting_since_s = None, None
            truck.moving = not waiting and not at_end
            permits[truck.name] = Permit(allowed, truck.granted_m, blocked_by)
        return permits

    def deadlocks(self, now_s: float) -> list[tuple[str, str]]:
        """(truck, blocker) pairs: a waiting cycle after cycle_s, or a wait of
        stall_s on a blocker that does not move. For the record only."""
        out = []
        for truck in self.trucks.values():
            if truck.waiting_on is None:
                continue
            waited = now_s - truck.waiting_since_s
            blocker = self.trucks[truck.waiting_on]
            cycle = blocker.waiting_on == truck.name
            if cycle and waited >= self.cycle_s:
                lower = max((truck, blocker), key=lambda t: t.key)
                pair = (lower.name, (blocker if lower is truck else truck).name)
                if pair not in out:
                    out.append(pair)
            elif not cycle and not blocker.moving and waited >= self.stall_s:
                out.append((truck.name, blocker.name))
        return out

    # -- TA/RA resolution (plan 2026-10-10) ----------------------------------
    def advisories(self, now_s: float) -> list[Advisory]:
        """TA, RA and CC advice after ``update``; see the module docstring.

        Each wait gets each piece of advice once (an RA again after ``failed``),
        and a truck gets no TA or RA within ``cooldown_s`` of its last one. A
        truck at the end of its route (parked) is never asked to act.
        """
        out = []
        for name, other in list(self.ra_against.items()):
            truck = self.trucks[name]
            if truck.waiting_on is None and (
                other not in self.trucks or not self.on_route(name, other)
            ):
                del self.ra_against[name]
                out.append(Advisory("cc", name, other, "clear"))
        for truck in sorted(self.trucks.values(), key=lambda t: t.key):
            if truck.waiting_on is None:
                continue
            blocker = self.trucks[truck.waiting_on]
            waited = now_s - truck.waiting_since_s
            lower, higher = (truck, blocker) if blocker.key < truck.key else (blocker, truck)
            cycle = blocker.waiting_on == truck.name
            if cycle and waited >= self.cycle_s:
                self._advise(out, now_s, "ra", lower, higher, "cycle", lower.waiting_since_s)
            # A cycle the RA did not clear: after ta_s either truck may go round
            # the other's body (the lower one once its cooldown is over). Only
            # a body in the way: a truck kept off a route gains nothing by it.
            since = truck.waiting_since_s
            stall = not blocker.moving and truck.held_by_body
            if self.ta_s is not None and stall and waited >= self.ta_s:
                self._advise(out, now_s, "ta", truck, blocker, "stall", since)
            if cycle:
                continue
            if self.ra_s is not None and waited >= self.ra_s:
                if lower is truck:
                    self._advise(out, now_s, "ra", truck, blocker, "long_wait", since)
                elif not blocker.moving and self.on_route(blocker.name, truck.name):
                    self._advise(out, now_s, "ra", blocker, truck, "in_the_way", since)
        return out

    def _advise(self, out, now_s, kind, actor, other, why, since) -> None:
        key = (kind, actor.name, other.name, why)
        if self._advised.get(key) == since or actor.progress_m >= actor.g[-1] - 1e-9:
            return
        if now_s - self._last_advice.get(actor.name, -np.inf) < self.cooldown_s:
            return
        self._advised[key] = since
        self._last_advice[actor.name] = now_s
        out.append(Advisory(kind, actor.name, other.name, why))

    def resolved(self, name: str, other: str) -> None:
        """name changed its route to clear other's: in RA until clear."""
        self.ra_against[name] = other

    def failed(self, advice: Advisory) -> None:
        """advice found no way out. An RA may come again, after the cooldown,
        in the same wait (other trucks will have moved by then); a TA does
        not, or it would take the cooldown each time before the RA's turn."""
        if advice.kind == "ra":
            self._advised.pop((advice.kind, advice.truck, advice.other, advice.why), None)

    def on_route(self, name: str, other: str) -> bool:
        """Does name's body, grown by the margin, touch other's remaining route?"""
        truck, route = self.trucks[name], self.trucks[other]
        here = oriented_rects(np.array([truck.pose]), truck.footprint, self.margin_m)
        ahead = route.rects[route.g >= self._remaining(route)]
        return bool(len(ahead)) and bool(overlap_matrix(here, ahead).any())

    def _keep_clear(self, rects: np.ndarray, name: str, keep_m: float, piece_m: float):
        """rects in pieces, without the pieces within keep_m of name's body: a
        planner starting there would find its start inside the keep-out."""
        truck = self.trucks[name]
        pieces = tile_rects(rects, piece_m)
        near = oriented_rects(np.array([truck.pose]), truck.footprint, keep_m)
        return pieces[~overlap_matrix(pieces, near).any(axis=1)]

    def wall(
        self,
        name: str,
        other: str,
        radius_m: float,
        keep_m: float,
        piece_m: float = 0.6,
        every: int = 3,
    ) -> np.ndarray:
        """RA keep-out for name: other's body and remaining route within
        radius_m of name, grown by twice the margin (what the safe-stop rule
        counts as off the route), as (N, 6) pieces clear of name's own spot."""
        truck, route = self.trucks[name], self.trucks[other]
        ahead = route.rects[route.g >= self._remaining(route)][::every]
        body = oriented_rects(np.array([route.pose]), route.footprint, self.margin_m)
        rects = np.vstack((ahead, body))
        rects = rects[np.hypot(*(rects[:, :2] - truck.pose[:2]).T) <= radius_m].copy()
        rects[:, 2:4] += self.margin_m
        return self._keep_clear(rects, name, keep_m, piece_m)

    def body_keepout(self, name: str, other: str, keep_m: float, piece_m: float = 0.4):
        """TA keep-out for name: other's body grown by twice the margin, in
        pieces clear of name's own spot."""
        route = self.trucks[other]
        body = oriented_rects(np.array([route.pose]), route.footprint, 2 * self.margin_m)
        return self._keep_clear(body, name, keep_m, piece_m)

    def lower_stops(self, name: str) -> tuple[list[str], np.ndarray]:
        """Where the trucks ranked below name may stop on what they hold now:
        the end of each one's grant, and the body of one that is not moving,
        grown by the margin. A new route for name over one of these would
        leave that truck standing on a higher-ranked route -- the no-lock
        precondition broken."""
        truck = self.trucks[name]
        names, rects = [], []
        for other in self.trucks.values():
            if other.key <= truck.key:
                continue
            end = int(np.searchsorted(other.g, other.granted_m + 1e-9, side="right")) - 1
            stops = [other.rects[max(end, 0)][None]]
            if not other.moving:
                stops.append(oriented_rects(np.array([other.pose]), other.footprint, self.margin_m))
            rects.extend(stops)
            names.extend([other.name] * len(stops))
        return names, np.vstack(rects) if rects else np.zeros((0, 6))

    def stop_keepout(self, name: str, keep_m: float, piece_m: float = 0.6) -> np.ndarray:
        """Keep-out for a new route of name: lower_stops grown by the margin
        once more, in pieces clear of name's own spot."""
        rects = self.lower_stops(name)[1].copy()
        rects[:, 2:4] += self.margin_m
        return self._keep_clear(rects, name, keep_m, piece_m)

    def keepout(self, name: str, rects, keep_m: float, piece_m: float = 0.6) -> np.ndarray:
        """Fixed places, (N, 6) rects, as a keep-out for a new route of name,
        in pieces clear of name's own spot."""
        return self._keep_clear(np.asarray(rects, dtype=float).reshape(-1, 6), name, keep_m, piece_m)

    def _path_rects(self, name: str, poses) -> np.ndarray:
        poses = np.asarray(poses, dtype=float).reshape(-1, 3)
        s = arc_lengths(poses)
        marks = np.append(np.arange(0.0, s[-1], self.sample_m), s[-1])
        samples = np.array([pose_at(poses, s, d) for d in marks])
        return oriented_rects(samples, self.trucks[name].footprint, self.margin_m)

    def path_conflicts(self, name: str, poses, other: str | None = None) -> list[str]:
        """Why name may not take poses as its new path, by name's body now:
        "covers_lower" (it runs over a lower_stops place) and, with other,
        "meets_other" (it touches other's claim, so name would wait for other
        again). Empty if neither."""
        rects = self._path_rects(name, poses)
        out = []
        stops = self.lower_stops(name)[1]
        if len(stops) and overlap_matrix(rects, stops).any():
            out.append("covers_lower")
        if other is not None and overlap_matrix(rects, self.trucks[other].claim).any():
            out.append("meets_other")
        return out

    def back_to(self, name: str, lowest_m: float, highest_m: float) -> float | None:
        """The furthest-along arc length in [lowest_m, highest_m] of name's
        route where it may stop (clear of every higher-ranked remaining
        route): where to back off to. None if there is none."""
        truck = self.trucks[name]
        ok = (~self._unsafe(truck)) & (truck.g >= lowest_m) & (truck.g <= highest_m)
        found = np.nonzero(ok)[0]
        return float(truck.g[found[-1]]) if len(found) else None

    def route_conflicts(self) -> list[dict]:
        """Breaks of the no-lock precondition: a truck's current pose on a
        higher-ranked truck's remaining route, or its final pose on a
        lower-ranked one's."""
        out = []
        for truck in self.trucks.values():
            here = oriented_rects(np.array([truck.pose]), truck.footprint, self.margin_m)
            for other in self.trucks.values():
                if other is truck:
                    continue
                ahead = other.rects[other.g >= self._remaining(other)]
                kind, rect = (
                    ("current", here)
                    if other.key < truck.key
                    else ("final", truck.rects[-1:])
                )
                if overlap_matrix(rect, ahead).any():
                    out.append(
                        {"truck": truck.name, "pose": kind, "route_of": other.name}
                    )
        return out

    def gap_m(self, a: str, b: str) -> float:
        """Centre distance of two trucks' believed bodies (for logging)."""
        pa, pb = self.trucks[a].pose, self.trucks[b].pose
        return hypot(pa[0] - pb[0], pa[1] - pb[1])


def rectangles_overlap(a: Rectangle, b: Rectangle) -> bool:
    """Closed-rectangle intersection by the separating-axis test."""
    half = a.length_m / 2
    return not FootprintCollisionChecker(
        [b], Footprint(half, half, a.width_m / 2), _HUGE
    ).free((a.x_m, a.y_m, a.yaw_rad))
