"""A session's liftable pallets in the Isaac runner (docs/plans/2026-10-22-rigid-session-pallets.md); no Isaac imports.

- ``pallet_prim(id)``: the one map from a pallet id (``P-1``) to its prim path (``/World/SessionPallets/P_1``; USD takes
  no ``-``) -- for its creation, its physics root, its semantic label and the LiDAR's self filter.
- ``PalletSwitch``: the runner's ``pallet`` in a session -- every read and write goes to the mission's pallet; a mission
  adopted mid-tick is switched to at the next tick's head (``request`` / ``apply_pending``), so no tick reads two.
- ``RestWatch``: where the other pallets rest and whether one moved (scene damage) -- rest poses fixed at the session's
  start and when a pallet stops being the mission's.
"""
from __future__ import annotations

import math
import re

ROOT = "/World/SessionPallets"
_ID = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
RESTING_M, RESTING_RAD = 0.01, 0.01  # how far a pallet nobody handles may move before the scene counts as damaged


def prim_name(pallet_id: str) -> str:
    if not isinstance(pallet_id, str) or not _ID.fullmatch(pallet_id):
        raise ValueError(f"pallet id {pallet_id!r} cannot name a prim")
    return pallet_id.replace("-", "_")


def pallet_prim(pallet_id: str) -> str:
    return f"{ROOT}/{prim_name(pallet_id)}"


def check_prim_names(pallet_ids) -> None:
    """Every id names a prim and no two name the same one (``P-1`` and ``P_1`` would)."""
    names = [prim_name(i) for i in pallet_ids]
    if len(set(names)) != len(names):
        raise ValueError(f"pallet ids {list(pallet_ids)} name the same prim twice")


def self_filter_prefix(pallet_id: str) -> str:
    """The LiDAR's self prefix for a carried session pallet: its own prim and nothing else (the trailing slash keeps
    ``P_1`` from matching ``P_10``)."""
    return pallet_prim(pallet_id) + "/"


class PalletSwitch:
    """The mission's pallet among a session's rigid pallets. Reads and writes go to the current one; ``handle(id)``
    reads another directly. A switch is asked for at an adoption (``request``) and made at the next tick's head
    (``apply_pending``), before anything of that tick reads the pallet."""

    def __init__(self, handles: dict, current: str):
        if current not in handles:
            raise ValueError(f"no pallet {current!r} in this session")
        self.handles = dict(handles)
        self.current = current
        self.pending: str | None = None
        self.switches: list = []  # [{"tick", "from", "to"}]

    def handle(self, pallet_id: str):
        return self.handles[pallet_id]

    # the reads and writes the runner makes of its one pallet
    def get_world_pose(self):
        return self.handles[self.current].get_world_pose()

    def set_world_pose(self, *args, **kwargs):
        return self.handles[self.current].set_world_pose(*args, **kwargs)

    def get_mass(self):
        return self.handles[self.current].get_mass()

    def get_linear_velocity(self):
        return self.handles[self.current].get_linear_velocity()

    def get_angular_velocity(self):
        return self.handles[self.current].get_angular_velocity()

    def request(self, pallet_id: str) -> None:
        if pallet_id not in self.handles:
            raise ValueError(f"no pallet {pallet_id!r} in this session")
        self.pending = pallet_id

    def apply_pending(self, tick: int, *, carried_id: str | None = None):
        """The switch asked for, at this tick's head: (old id, new id), or None when none is pending or it is the same
        pallet. Refused when the pallet leaving is on the forks (a submit for another pallet is refused while carrying,
        so this would be a broken contract)."""
        target, self.pending = self.pending, None
        if target is None or target == self.current:
            return None
        if carried_id is not None and carried_id == self.current:
            raise RuntimeError(f"session_pallet_switch_while_carrying:{self.current}->{target}")
        old, self.current = self.current, target
        self.switches.append({"tick": int(tick), "from": old, "to": target})
        return old, target


def _rotation_between(q0, q1) -> float:
    """The whole rotation angle from q0 to q1 (w, x, y, z): a tilt counts as a turn does."""
    dot = abs(sum(float(a) * float(b) for a, b in zip(q0, q1)))
    return 2 * math.acos(min(1.0, dot))


def _corners_rect(x, y, length, width, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = length / 2, width / 2
    return [(x + c * u - s * v, y + s * u + c * v) for u, v in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]


def footprint_corners(rear, front_m, rear_m, half_width_m):
    """The planner's outline (rear axle frame: front_m ahead, rear_m behind, half_width_m aside) at ``rear``."""
    x, y, yaw = (float(v) for v in rear)
    c, s = math.cos(yaw), math.sin(yaw)
    local = ((front_m, half_width_m), (-rear_m, half_width_m), (-rear_m, -half_width_m), (front_m, -half_width_m))
    return [(x + c * u - s * v, y + s * u + c * v) for u, v in local]


def rect_corners(r) -> list:
    return _corners_rect(float(r.x_m), float(r.y_m), float(r.length_m), float(r.width_m), float(r.yaw_rad))


def _separated(a, b) -> bool:
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            ex, ey = poly[(i + 1) % n][0] - poly[i][0], poly[(i + 1) % n][1] - poly[i][1]
            ax, ay = -ey, ex
            pa = [px * ax + py * ay for px, py in a]
            pb = [px * ax + py * ay for px, py in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return True
    return False


def _point_segment(p, a, b) -> float:
    abx, aby = b[0] - a[0], b[1] - a[1]
    t = ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / max(abx * abx + aby * aby, 1e-18)
    t = min(1.0, max(0.0, t))
    return math.hypot(p[0] - (a[0] + t * abx), p[1] - (a[1] + t * aby))


def polygon_gap(a, b) -> float:
    """The gap between two convex outlines (corner lists); 0 when they touch or overlap."""
    if not _separated(a, b):
        return 0.0
    best = math.inf
    for poly, other in ((a, b), (b, a)):
        n = len(other)
        for p in poly:
            for i in range(n):
                best = min(best, _point_segment(p, other[i], other[(i + 1) % n]))
    return best


class NeighbourGap:
    """The smallest gap seen between the truck's outline and each other session pallet's true outline (CT-1 D1's
    record: the real distance to the neighbours, plan 2026-10-22-rigid-session-pallets section 4)."""

    def __init__(self):
        self.smallest: dict = {}  # id -> {"m", "t", "phase", "mission_pallet"}

    def update(self, t: float, phase: str, mission_pallet: str, truck_corners, others: dict) -> None:
        for pallet_id, corners in others.items():
            gap = polygon_gap(truck_corners, corners)
            seen = self.smallest.get(pallet_id)
            if seen is None or gap < seen["m"]:
                self.smallest[pallet_id] = {"m": gap, "t": float(t), "phase": phase, "mission_pallet": mission_pallet}


class RestWatch:
    """Where the pallets nobody handles rest, and the largest move seen. ``rest(id, position, quaternion)`` fixes one;
    ``check(id, position, quaternion)`` returns the move ((metres in 3D, radians of the whole rotation), whether it
    passes the bounds) -- a pallet lifted or tipped on one edge moves as one slid or turned does (Codex round 3)."""

    def __init__(self, bound_m: float = RESTING_M, bound_rad: float = RESTING_RAD):
        self.bound_m, self.bound_rad = float(bound_m), float(bound_rad)
        self.poses: dict = {}
        self.largest: dict = {}  # id -> [metres, radians] the largest move seen

    def rest(self, pallet_id: str, position, quaternion) -> None:
        self.poses[pallet_id] = (tuple(float(v) for v in position[:3]), tuple(float(v) for v in quaternion))

    def check(self, pallet_id: str, position, quaternion):
        p0, q0 = self.poses[pallet_id]
        moved_m = math.dist(p0, tuple(float(v) for v in position[:3]))
        moved_rad = _rotation_between(q0, quaternion)
        seen = self.largest.setdefault(pallet_id, [0.0, 0.0])
        seen[0], seen[1] = max(seen[0], moved_m), max(seen[1], moved_rad)
        return (moved_m, moved_rad), (moved_m <= self.bound_m and moved_rad <= self.bound_rad)
