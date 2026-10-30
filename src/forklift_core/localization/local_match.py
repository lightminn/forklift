"""F1's local re-match around an operator-placed start pose (docs/plans/2026-10-31-operator-placed-start.md 1.3).

The global matcher's lattice search runs only where the laser may stand for a base pose inside the window -- within
``radius_m`` of the placed base position and ``yaw_rad`` of its heading -- and returns every leaf scoring at least
``keep_ratio`` of the window's best (before any suppression). The top leaf is refined (the same point-to-line ICP as
the global refinement), and confirmed only when

1. refinement did not walk away from its lattice pose (``refine_moved``),
2. its endpoint fraction is at least E_MIN (``low_fit``),
3. no leaf outside the success tolerance of the refined base pose scores R_AMB of the lattice best or more
   (``ambiguous``: a slide along an aisle, a near heading, another peak -- no shape is assumed),
4. the refined base pose lies inside the window by more than its edge bands (``edge``: the truth may be outside).
Ambiguity is checked before the edge: it is the more telling refusal (a closer click would not resolve it).

A leaf of the window's best lying outside the window proper (the search mask is the window's laser disc, a little
larger) is an ``edge`` refusal too, so the window's best is the masked search's best and the leaf set is complete
down to ``keep_ratio`` of it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.localization import global_match as G
from forklift_core.localization.relocalization_samples import LASER_X_M
from forklift_core.localization.slam_map import Grid


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Window:
    base: tuple  # the operator's placed base_link pose (x, y, yaw), map
    radius_m: float = 2.0
    yaw_rad: float = 0.6
    edge_m: float = 0.3
    edge_rad: float = 0.1


@dataclass(frozen=True)
class LocalResult:
    status: str  # ok | uninformative | empty | overflow | timeout | cancelled
    confirmed: bool
    reason: str  # confirmed | refine_moved | low_fit | edge | ambiguous | <status>
    base: tuple | None = None  # the refined base pose (the estimate when confirmed)
    lattice_base: tuple | None = None
    endpoint_fraction: float | None = None
    amb_ratio: float | None = None  # the best leaf outside the success tolerance / the lattice best
    leaves: int = 0
    work: int = 0


def _bases(lasers: np.ndarray) -> np.ndarray:
    """Laser poses (n, 3) to base_link poses: the laser stands LASER_X_M ahead of base_link, same heading."""
    yaw = lasers[:, 2]
    return np.stack([lasers[:, 0] - LASER_X_M * np.cos(yaw), lasers[:, 1] - LASER_X_M * np.sin(yaw), yaw], axis=1)


def _laser(base) -> tuple:
    x, y, yaw = base
    return (x + LASER_X_M * math.cos(yaw), y + LASER_X_M * math.sin(yaw), yaw)


def _yaw_gaps(yaws: np.ndarray, yaw0: float) -> np.ndarray:
    return np.abs(np.arctan2(np.sin(yaws - yaw0), np.cos(yaws - yaw0)))


def ambiguity_ratio(bases: np.ndarray, scores: np.ndarray, *, best: float, refined, success_m: float,
                    success_rad: float) -> float:
    """The highest score / ``best`` among leaves (base poses) outside the success tolerance of the refined base pose
    -- distance and heading from the refined pose, the ratio against the lattice best (Codex F1 v2 review); 0 if none."""
    bases = np.asarray(bases, float).reshape(-1, 3)
    if not len(bases) or best <= 0:
        return 0.0
    d = np.hypot(bases[:, 0] - refined[0], bases[:, 1] - refined[1])
    out = (d > success_m) | (_yaw_gaps(bases[:, 2], refined[2]) > success_rad)
    return float(np.max(np.asarray(scores, float)[out]) / best) if out.any() else 0.0


def _laser_disc(grid: Grid, centre, radius_m: float) -> np.ndarray:
    h, w = grid.data.shape
    ox, oy, _ = grid.origin
    res = grid.resolution_m
    cols = ox + (np.arange(w) + 0.5) * res
    rows = oy + (np.arange(h) + 0.5) * res
    return (rows[:, None] - centre[1]) ** 2 + (cols[None, :] - centre[0]) ** 2 <= radius_m ** 2


def local_match(grid: Grid, points: np.ndarray, window: Window, cfg: G.MatchConfig | None = None, *, e_min: float,
                r_amb: float, success_m: float = 0.10, success_rad: float = 0.05, index: G.MapIndex | None = None,
                clock=None, deadline: float | None = None, cancel=None) -> LocalResult:
    cfg = cfg or G.MatchConfig()
    G._check_grid(grid)
    pts = G._thin(np.asarray(points, float).reshape(-1, 2), grid.resolution_m)
    if len(pts) < cfg.min_points or not grid.occupied().any():
        return LocalResult("uninformative", False, "uninformative")
    x0, y0, yaw0 = (float(v) for v in window.base)
    # the laser of a base inside the window stands within radius + |LASER_X_M| of the placed base position
    cells = _laser_disc(grid, (x0, y0), window.radius_m + abs(LASER_X_M))
    n_yaw = max(1, int(round(2 * math.pi / cfg.yaw_step_rad)))
    yaws = np.arange(n_yaw) * (2 * math.pi / n_yaw)
    yaw_index = np.flatnonzero(_yaw_gaps(yaws, yaw0) <= window.yaw_rad)
    budget = G._Budget(cfg, clock, deadline, cancel)
    try:
        s = G._Search(grid, pts, cfg, index, budget, cells=cells, yaw_index=yaw_index)
        if not s.cand.any() or not len(yaw_index):
            return LocalResult("empty", False, "empty", work=budget.work)
        h, r, c, sc = G._lattice_leaves(s, cfg)
    except G._Stop as stop:
        return LocalResult(str(stop), False, str(stop), work=budget.work)
    if not len(sc):
        return LocalResult("empty", False, "empty", work=budget.work)
    lasers = np.array([s.pose(h[k], r[k], c[k]) for k in range(len(h))])
    bases = _bases(lasers)
    inside = (np.hypot(bases[:, 0] - x0, bases[:, 1] - y0) <= window.radius_m) & (
        _yaw_gaps(bases[:, 2], yaw0) <= window.yaw_rad)
    top = int(np.argmax(sc))
    best = float(sc[top])
    lattice_base = tuple(float(v) for v in bases[top])
    common = dict(lattice_base=lattice_base, leaves=int(inside.sum()), work=budget.work)
    if not inside[top]:  # the masked search's best lies just outside the window proper: at its edge
        # ambiguity is the more telling refusal (a closer click would not help), read here from the lattice top
        ratio = ambiguity_ratio(bases[inside], sc[inside], best=best, refined=lattice_base, success_m=success_m,
                                success_rad=success_rad)
        return LocalResult("ok", False, "ambiguous" if ratio >= r_amb else "edge", amb_ratio=ratio, **common)
    cand = G.Candidate(tuple(float(v) for v in lasers[top]), best, math.nan)
    refined = G.refine(grid, G.GlobalMatch("ok", (cand,), work=budget.work), points, cfg, index=index, clock=clock,
                       deadline=deadline, cancel=cancel)
    if refined.status != "ok":
        return LocalResult(refined.status, False, refined.status, **common)
    rc = refined.candidates[0]
    base = tuple(float(v) for v in _bases(np.array([rc.pose]))[0])
    common.update(base=base, endpoint_fraction=float(rc.endpoint_fraction))
    lp = rc.search_pose
    if math.hypot(rc.pose[0] - lp[0], rc.pose[1] - lp[1]) >= cfg.sep_m or abs(wrap(rc.pose[2] - lp[2])) >= cfg.sep_rad:
        return LocalResult("ok", False, "refine_moved", **common)
    if not (rc.endpoint_fraction >= e_min):  # NaN fails
        return LocalResult("ok", False, "low_fit", **common)
    ratio = ambiguity_ratio(bases[inside], sc[inside], best=best, refined=base, success_m=success_m,
                            success_rad=success_rad)
    common["amb_ratio"] = ratio
    if ratio >= r_amb:  # before the edge: a closer click would not resolve it
        return LocalResult("ok", False, "ambiguous", **common)
    if (math.hypot(base[0] - x0, base[1] - y0) > window.radius_m - window.edge_m
            or abs(wrap(base[2] - yaw0)) > window.yaw_rad - window.edge_rad):
        return LocalResult("ok", False, "edge", **common)
    if cancel is not None and cancel():
        return LocalResult("cancelled", False, "cancelled", **common)
    if deadline is not None and clock is not None and clock() > deadline:
        return LocalResult("timeout", False, "timeout", **common)
    return LocalResult("ok", True, "confirmed", **common)


__all__ = ["LocalResult", "Window", "ambiguity_ratio", "local_match", "wrap"]
