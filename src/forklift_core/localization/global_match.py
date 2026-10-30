"""The global matcher (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md B1-B2).

Where on a survey map's grid a laser scan was taken, without a prior: a multi-resolution correlative search with
branch and bound over every free cell the laser may stand in (``candidate_cells``: free, with ``clearance_m`` to the
nearest occupied cell) and every heading ``yaw_step_rad`` apart. The score of a lattice pose is the mean, over the
scan's endpoints (``scan_points``: finite, inside the sensor's limits, at most ``match_range_m`` away, thinned to one
per map cell), of a likelihood field exp(-d^2 / 2 sigma^2) of the distance d to the nearest occupied cell. Level k of
the pyramid holds the maximum over the 2^k x 2^k cells from each cell, so a block's score at level k bounds every
pose in it (the laser stands on cell centres, so endpoint cells shift by whole cells). The search keeps every leaf
scoring at least ``keep_ratio`` of the best, so the competitors are exact, not the survivors of a greedy descent.

The leaves are reduced by non-maximum suppression with the wrong-confirmation tolerance (``suppress``: a leaf is
dropped only when it lies within both ``sep_m`` and ``sep_rad`` of a better kept one), refined each by point-to-line
ICP against the occupied cells (``refine``: every kept candidate the same way; it places the pose, while the
competitor ratio stays the exact lattice one), and judged by ``decide``: the top not moved by its refinement beyond the
tolerance, its endpoint fraction at least ``e_min``, the best competitor's lattice score ratio at most ``r_max``, and
no evenly spaced row of competitors (``periodic``: shelf bays). An observation with too few endpoints is ``uninformative`` and a search whose frontier outgrows ``max_nodes``
is ``overflow``; neither is ever confirmed, and no NaN reaches a comparison.

Poses are the laser's in the map frame; the caller converts to and from the base. numpy and scipy only.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace

import numpy as np

from forklift_core.localization.scan_match import match_scans, transform_points
from forklift_core.localization.slam_map import Grid, dilated


@dataclass(frozen=True)
class MatchConfig:
    yaw_step_rad: float = math.radians(0.5)
    match_range_m: float = 8.0  # plan B1 R_MATCH: 0.5 deg is 0.105 m at 12 m
    sigma_m: float = 0.05
    levels: int = 5  # the coarsest block is 2^levels cells (1.6 m at 5 cm)
    # every leaf scoring >= this share of the best is kept. Plan B4b: 0.80 = the lowest r_struct of the selection grid
    # (decide reads only the top, a competitor above r_max > r_struct and candidates >= r_struct); B4a had 0.40
    keep_ratio: float = 0.8
    sep_m: float = 0.3  # plan B1: the wrong-confirmation tolerance
    sep_rad: float = 0.1
    clearance_m: float = 0.3  # the laser stands where the truck can: this far from any occupied cell
    min_points: int = 30  # fewer endpoints is no information
    max_candidates: int = 8
    # a level's frontier larger than this is an overflow (no answer, never a confirmation); a frontier guard, not a
    # time bound. B4a: 400,000 -> 4,000,000 (dev smoke overflow); B4b: 1,000,000 (dev maximum ~89,000 with the beam)
    max_nodes: int = 1_000_000
    endpoint_dilate_cells: int = 1  # the endpoint fraction's tolerance, as slam_map.endpoint_fraction
    beam_width: int = 64  # the floor's descent keeps this many bounds per level (B4b; 1 is the old greedy leaf)
    max_work: int | None = None  # node x endpoint scores per search before overflow (B4b: set on dev, then frozen)
    deadline_s: float | None = 30.0  # relocalize(): no confirmation after this many seconds (B4b, console wait)


@dataclass(frozen=True)
class Candidate:
    pose: tuple  # laser (x, y, yaw) in the map
    score: float  # mean likelihood over the endpoints
    endpoint_fraction: float
    refined: bool = False
    lattice_pose: tuple | None = None  # where the exact lattice search put it (None: the pose is the lattice pose)
    lattice_score: float | None = None

    @property
    def search_score(self) -> float:
        """The exact lattice score: what the competitor ratio and the periodic check compare."""
        return self.score if self.lattice_score is None else self.lattice_score

    @property
    def search_pose(self) -> tuple:
        return self.pose if self.lattice_pose is None else self.lattice_pose


@dataclass(frozen=True)
class GlobalMatch:
    status: str  # ok | uninformative | overflow | timeout | cancelled
    candidates: tuple = ()  # the best first, then the competitors beyond the separation
    work: int = 0  # node x endpoint scores the search spent


@dataclass(frozen=True)
class Decision:
    confirmed: bool
    reason: str  # confirmed | uninformative | overflow | timeout | cancelled | refine_moved | low_fit | competitor | periodic
    ratio: float | None = None  # best competitor's score / the top's


def scan_points(ranges, angle_min_rad: float, angle_increment_rad: float, *, range_min_m: float,
                range_max_m: float, match_range_m: float = 8.0, cell_m: float | None = None) -> np.ndarray:
    """(N, 2) endpoints in the laser frame: finite ranges within the sensor's limits and ``match_range_m``; with
    ``cell_m``, thinned to one per cell (their mean)."""
    r = np.asarray(ranges, float)
    a = angle_min_rad + angle_increment_rad * np.arange(len(r))
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(r) & (r >= range_min_m) & (r <= min(range_max_m, match_range_m))
    pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1)
    return _thin(pts, cell_m) if cell_m else pts


def _thin(pts: np.ndarray, cell_m: float) -> np.ndarray:
    if not len(pts):
        return pts
    keys = np.floor(pts / cell_m).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    out = np.zeros((inverse.max() + 1, 2))
    np.add.at(out, inverse, pts)
    return out / np.bincount(inverse)[:, None]


def stack_scans(scans, *, range_min_m: float, range_max_m: float, match_range_m: float = 8.0,
                cell_m: float = 0.05) -> np.ndarray:
    """Endpoints of several scans in the first laser's frame: ``scans`` = [(laser pose in the first laser's frame
    (odometry, or identity standing still), ranges, angle_min, angle_increment)], thinned to one per ``cell_m``."""
    parts = [transform_points(scan_points(r, a0, inc, range_min_m=range_min_m, range_max_m=range_max_m,
                                          match_range_m=match_range_m), pose) for pose, r, a0, inc in scans]
    pts = np.vstack(parts) if parts else np.zeros((0, 2))
    return _thin(pts, cell_m)


def _check_grid(grid: Grid) -> None:
    if abs(grid.origin[2]) > 1e-12:
        raise ValueError("the global matcher takes grids with an unrotated origin")


def likelihood_field(grid: Grid, sigma_m: float) -> np.ndarray:
    from scipy.ndimage import distance_transform_edt  # noqa: PLC0415

    occ = grid.occupied()
    if not occ.any():
        return np.zeros(occ.shape)
    d = distance_transform_edt(~occ) * grid.resolution_m
    return np.exp(-(d * d) / (2.0 * sigma_m * sigma_m))


def candidate_cells(grid: Grid, clearance_m: float) -> np.ndarray:
    """Free cells (survey value 0) at least ``clearance_m`` from every occupied cell: where the laser may stand."""
    from scipy.ndimage import distance_transform_edt  # noqa: PLC0415

    occ = grid.occupied()
    free = grid.data == 0
    if not occ.any():
        return free
    d = distance_transform_edt(~occ) * grid.resolution_m
    return free & (d >= clearance_m)


def _pyramid(base: np.ndarray, levels: int, op) -> list:
    """Level k at (r, c): ``op`` (max / or) over the 2^k x 2^k cells from (r, c); beyond the grid counts as zero."""
    out = [base]
    for k in range(1, levels + 1):
        prev, h = out[-1], 1 << (k - 1)
        nxt = prev.copy()
        nxt[:-h, :] = op(nxt[:-h, :], prev[h:, :])
        shifted = nxt.copy()
        shifted[:, :-h] = op(shifted[:, :-h], nxt[:, h:])
        out.append(shifted)
    return out


def _reach_cells(reach_m: float, res: float) -> int:
    return int(math.ceil(reach_m / res)) + 2


class MapIndex:
    """What a search needs from one survey grid, built once per bundle (plan B4b): the likelihood field, its max
    pyramid on a field padded for endpoints up to ``match_range_m``, the candidate cells and their pyramid, and the
    occupied cell centres the refinement fits to. Searches with or without it give the same poses and scores."""

    def __init__(self, grid: Grid, cfg: MatchConfig | None = None):
        cfg = cfg or MatchConfig()
        _check_grid(grid)
        self.grid, self.cfg = grid, cfg
        self.field = likelihood_field(grid, cfg.sigma_m)
        self.cand = candidate_cells(grid, cfg.clearance_m)
        self.reach = _reach_cells(cfg.match_range_m, grid.resolution_m)
        self.pad = self.reach + (1 << cfg.levels) + 1
        self.L = _pyramid(np.pad(self.field, self.pad), cfg.levels, np.maximum)
        self.C = _pyramid(self.cand, cfg.levels, np.logical_or)
        self.walls = grid.cell_centres(grid.occupied())


class _Stop(Exception):
    """The search's budget ran out: ``overflow`` (work) or ``timeout`` (the deadline)."""


class _Budget:
    """The work cap (node x endpoint scores, deterministic) and the wall deadline (``clock`` seconds), checked per
    scoring chunk and by the refinement around each ICP (plan B4b)."""

    def __init__(self, cfg: MatchConfig, clock=None, deadline: float | None = None, cancel=None):
        self.max_work, self.clock, self.deadline, self.work = cfg.max_work, clock, deadline, 0
        self.cancel = cancel

    def check(self) -> None:
        if self.cancel is not None and self.cancel():  # B-P2: STOP or session end while a request runs
            raise _Stop("cancelled")
        if self.deadline is not None and self.clock is not None and self.clock() > self.deadline:
            raise _Stop("timeout")

    def spend(self, n: int) -> None:
        self.work += n
        if self.max_work is not None and self.work > self.max_work:
            raise _Stop("overflow")
        self.check()


class _Search:
    def __init__(self, grid: Grid, pts: np.ndarray, cfg: MatchConfig, index: MapIndex | None = None,
                 budget: _Budget | None = None, *, cells: np.ndarray | None = None, yaw_index=None):
        """``cells`` (a mask of the grid's shape) and ``yaw_index`` restrict where the laser may stand and which
        lattice headings are searched (F1's local re-match); without them, every candidate cell and heading."""
        self.grid, self.cfg, self.res = grid, cfg, grid.resolution_m
        self.budget = budget or _Budget(replace(cfg, max_work=None))
        self.n_yaw = max(1, int(round(2 * math.pi / cfg.yaw_step_rad)))
        self.yaws = np.arange(self.n_yaw) * (2 * math.pi / self.n_yaw)
        self.yaw_index = np.arange(self.n_yaw) if yaw_index is None else np.asarray(yaw_index, np.int64)
        # an endpoint's distance, not its largest coordinate: a heading can turn it onto one axis (Codex B-P1b review)
        reach = _reach_cells(float(np.hypot(pts[:, 0], pts[:, 1]).max()), self.res) if len(pts) else 2
        if index is not None:
            if reach > index.reach:
                raise ValueError("endpoints beyond the map index's reach (match_range_m)")
            self.pad, self.L, self.C, self.cand = index.pad, index.L, index.C, index.cand
        else:
            field = likelihood_field(grid, cfg.sigma_m)
            cand = candidate_cells(grid, cfg.clearance_m)
            self.pad = reach + (1 << cfg.levels) + 1
            # pad first: a block starting outside the grid must still pool the cells inside it (B-P1b; pooling before
            # padding read such blocks as 0 and the bound fell below leaves near walls on the grid's first rows/columns)
            self.L = _pyramid(np.pad(field, self.pad), cfg.levels, np.maximum)
            self.C = _pyramid(cand, cfg.levels, np.logical_or)
            self.cand = cand
        if cells is not None:
            self.cand = self.cand & cells
            self.C = _pyramid(self.cand, cfg.levels, np.logical_or)
        c, s = np.cos(self.yaws)[:, None], np.sin(self.yaws)[:, None]
        qx = c * pts[:, 0] - s * pts[:, 1]
        qy = s * pts[:, 0] + c * pts[:, 1]
        # the endpoint's cell relative to the laser's cell when the laser stands on a cell centre
        self.oc = np.floor(0.5 + qx / self.res).astype(np.int64)
        self.orow = np.floor(0.5 + qy / self.res).astype(np.int64)

    def scores(self, level: int, h, r, c, chunk: int = 4096) -> np.ndarray:
        out = np.empty(len(h))
        lv = self.L[level]
        for a in range(0, len(h), chunk):
            hh, rr, cc = h[a:a + chunk], r[a:a + chunk], c[a:a + chunk]
            vals = lv[rr[:, None] + self.orow[hh] + self.pad, cc[:, None] + self.oc[hh] + self.pad]
            out[a:a + chunk] = vals.mean(axis=1)
            self.budget.spend(len(hh) * self.oc.shape[1])
        return out

    def children(self, level: int, h, r, c):
        half = 1 << (level - 1)
        hh = np.repeat(h, 4)
        rr = np.repeat(r, 4) + np.tile([0, 0, half, half], len(r))
        cc = np.repeat(c, 4) + np.tile([0, half, 0, half], len(c))
        shape = self.cand.shape
        ok = (rr < shape[0]) & (cc < shape[1])
        hh, rr, cc = hh[ok], rr[ok], cc[ok]
        keep = self.C[level - 1][rr, cc]
        return hh[keep], rr[keep], cc[keep]

    def top(self):
        step = 1 << self.cfg.levels
        rs, cs = np.meshgrid(np.arange(0, self.cand.shape[0], step), np.arange(0, self.cand.shape[1], step),
                             indexing="ij")
        rs, cs = rs.ravel(), cs.ravel()
        keep = self.C[self.cfg.levels][rs, cs]
        rs, cs = rs[keep], cs[keep]
        h = np.repeat(self.yaw_index, len(rs))
        return h, np.tile(rs, len(self.yaw_index)), np.tile(cs, len(self.yaw_index))

    def pose(self, h, r, c) -> tuple:
        ox, oy, _ = self.grid.origin
        return (ox + (c + 0.5) * self.res, oy + (r + 0.5) * self.res, float(self.yaws[h]))


def _beam_floor(s: _Search, h, r, c, bounds, width: int) -> float:
    """The best leaf score reached by descending the ``width`` highest bounds at every level: a real leaf's score, so
    at most the best, and with a wide beam usually the best itself (plan B4b; one greedy leaf could sit far below)."""
    i = np.argsort(-bounds, kind="stable")[:width]
    gh, gr, gc = h[i], r[i], c[i]
    for level in range(s.cfg.levels, 0, -1):
        gh, gr, gc = s.children(level, gh, gr, gc)
        b = s.scores(level - 1, gh, gr, gc)
        j = np.argsort(-b, kind="stable")[:width]
        gh, gr, gc, b = gh[j], gr[j], gc[j], b[j]
    return float(b.max())


def _lattice_leaves(s: _Search, cfg: MatchConfig):
    """Every lattice leaf scoring >= ``keep_ratio`` of the best one -- complete down to that share, since the descent
    prunes below ``keep_ratio`` of a real leaf's score (the beam floor) with upper bounds. Raises _Stop on the work
    cap, the deadline, a cancel, or a frontier over ``max_nodes`` (``overflow``)."""
    h, r, c = s.top()
    bounds = s.scores(cfg.levels, h, r, c)
    if not len(bounds):
        return h, r, c, bounds
    floor = _beam_floor(s, h, r, c, bounds, max(1, cfg.beam_width))
    thr = cfg.keep_ratio * floor
    for level in range(cfg.levels, 0, -1):
        keep = bounds >= thr
        h, r, c = h[keep], r[keep], c[keep]
        if len(h) * 4 > cfg.max_nodes:
            raise _Stop("overflow")
        h, r, c = s.children(level, h, r, c)
        bounds = s.scores(level - 1, h, r, c)
    best = float(bounds.max()) if len(bounds) else floor
    keep = bounds >= cfg.keep_ratio * best
    return h[keep], r[keep], c[keep], bounds[keep]


def global_match(grid: Grid, points: np.ndarray, cfg: MatchConfig | None = None, *, index: MapIndex | None = None,
                 clock=None, deadline: float | None = None, cancel=None) -> GlobalMatch:
    """The kept candidates (lattice poses, before refinement) or why there are none. With ``clock`` and ``deadline``
    a search still running at the deadline ends as ``timeout``; ``max_work`` ends it as ``overflow``."""
    cfg = cfg or MatchConfig()
    _check_grid(grid)
    pts = _thin(np.asarray(points, float).reshape(-1, 2), grid.resolution_m)
    cand = index.cand if index is not None else candidate_cells(grid, cfg.clearance_m)
    if len(pts) < cfg.min_points or not cand.any() or not grid.occupied().any():
        return GlobalMatch("uninformative")
    budget = _Budget(cfg, clock, deadline, cancel)
    try:
        s = _Search(grid, pts, cfg, index, budget)
        h, r, c, sc = _lattice_leaves(s, cfg)
    except _Stop as stop:
        return GlobalMatch(str(stop), work=budget.work)
    order = np.argsort(-sc, kind="stable")
    leaves = [Candidate(s.pose(h[k], r[k], c[k]), float(sc[k]), math.nan) for k in order]
    kept = suppress(leaves, sep_m=cfg.sep_m, sep_rad=cfg.sep_rad)[:cfg.max_candidates]
    return GlobalMatch("ok", tuple(_with_fraction(grid, pts, k, cfg) for k in kept), work=budget.work)


def brute_force_best(grid: Grid, points: np.ndarray, cfg: MatchConfig | None = None) -> float:
    """The best lattice score by exhaustive search (a test oracle for the branch and bound)."""
    cfg = cfg or MatchConfig()
    pts = _thin(np.asarray(points, float).reshape(-1, 2), grid.resolution_m)
    s = _Search(grid, pts, cfg)
    rows, cols = np.nonzero(s.cand)
    h = np.repeat(np.arange(s.n_yaw), len(rows))
    return float(s.scores(0, h, np.tile(rows, s.n_yaw), np.tile(cols, s.n_yaw)).max())


def _yaw_gap(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def suppress(candidates, *, sep_m: float, sep_rad: float) -> list:
    """Non-maximum suppression in score order: a candidate is dropped only within both ``sep_m`` and ``sep_rad`` of
    a better kept one; anything farther in position or heading stays a competitor."""
    kept: list = []
    for cand in sorted(candidates, key=lambda k: -k.score):
        if all(math.hypot(cand.pose[0] - k.pose[0], cand.pose[1] - k.pose[1]) >= sep_m
               or _yaw_gap(cand.pose[2], k.pose[2]) >= sep_rad for k in kept):
            kept.append(cand)
    return kept


def _field_score(grid: Grid, field: np.ndarray, pts: np.ndarray, pose) -> float:
    world = transform_points(pts, pose)
    rows, cols, inside = grid.cells_of(world)
    vals = np.where(inside, field[rows, cols], 0.0)
    return float(vals.mean()) if len(vals) else 0.0


def endpoint_fraction(grid: Grid, pts: np.ndarray, pose, dilate_cells: int = 1) -> float:
    """The share of endpoints on the occupied cells (dilated); NaN for no endpoints."""
    if not len(pts):
        return math.nan
    rows, cols, inside = grid.cells_of(transform_points(pts, pose))
    walls = dilated(grid.occupied(), dilate_cells)
    return float((inside & walls[rows, cols]).sum()) / len(pts)


def _with_fraction(grid: Grid, pts: np.ndarray, cand: Candidate, cfg: MatchConfig) -> Candidate:
    return replace(cand, endpoint_fraction=endpoint_fraction(grid, pts, cand.pose, cfg.endpoint_dilate_cells))


def refine(grid: Grid, match: GlobalMatch, points: np.ndarray, cfg: MatchConfig | None = None, *,
           index: MapIndex | None = None, clock=None, deadline: float | None = None, cancel=None) -> GlobalMatch:
    """Point-to-line ICP of every kept candidate against the occupied cells near it, each rescored on the same field
    at its refined pose (an unaccepted ICP keeps the lattice pose, unrefined). The order and the lattice scores stay:
    the competitor ratio is read from the exact lattice search, whose kept set is complete down to ``keep_ratio``;
    refinement only places the pose (Codex B4a re-review P2: an ICP from a weak lattice pose can climb, and
    candidates can converge together, so refined scores would not be a complete competitor set). With ``clock`` and
    ``deadline`` the deadline is checked before and after each ICP; past it the match is a ``timeout``."""
    cfg = cfg or MatchConfig()
    if match.status != "ok":
        return match
    pts = _thin(np.asarray(points, float).reshape(-1, 2), grid.resolution_m)
    field = index.field if index is not None else likelihood_field(grid, cfg.sigma_m)
    walls = index.walls if index is not None else grid.cell_centres(grid.occupied())
    budget = _Budget(replace(cfg, max_work=None), clock, deadline, cancel)
    out = []
    try:
        for cand in match.candidates:
            budget.check()
            near = walls[np.hypot(walls[:, 0] - cand.pose[0], walls[:, 1] - cand.pose[1]) <= cfg.match_range_m + 0.5]
            res = match_scans(near, pts, cand.pose, max_correspondence_m=cfg.sep_m)
            budget.check()
            pose = tuple(float(v) for v in res.pose) if res.accepted else cand.pose
            out.append(_with_fraction(grid, pts, Candidate(pose, _field_score(grid, field, pts, pose), math.nan,
                                                           res.accepted, cand.search_pose, cand.search_score), cfg))
    except _Stop as stop:
        return GlobalMatch(str(stop), work=match.work)
    return GlobalMatch("ok", tuple(out), work=match.work)


def relocalize(grid: Grid, points: np.ndarray, cfg: MatchConfig | None = None, *, e_min: float, r_max: float,
               r_struct: float | None = None, index: MapIndex | None = None, clock=time.monotonic,
               deadline_at: float | None = None, cancel=None):
    """One relocalization request (plan B4b): search, refine, decide, all against one deadline -- ``deadline_at``
    (a ``clock`` time shared by several requests, B-P2's session deadline) or else ``cfg.deadline_s`` after the call.
    A confirmation is checked against the deadline (and ``cancel``) right before it is returned, so none comes out
    late: past the deadline the request is a ``timeout``, a cancelled one ``cancelled``; neither is confirmed.
    Returns (refined match, decision)."""
    cfg = cfg or MatchConfig()
    deadline = deadline_at if deadline_at is not None else (
        None if cfg.deadline_s is None else clock() + cfg.deadline_s)
    match = global_match(grid, points, cfg, index=index, clock=clock, deadline=deadline, cancel=cancel)
    match = refine(grid, match, points, cfg, index=index, clock=clock, deadline=deadline, cancel=cancel)
    decision = decide(match, e_min=e_min, r_max=r_max, r_struct=r_struct, sep_m=cfg.sep_m, sep_rad=cfg.sep_rad)
    if cancel is not None and cancel():
        return GlobalMatch("cancelled", work=match.work), Decision(False, "cancelled")
    if deadline is not None and clock() > deadline:
        match = GlobalMatch("timeout", work=match.work)
        decision = Decision(False, "timeout")
    return match, decision


def periodic(candidates, *, ratio_min: float, line_tol_m: float = 0.15, spacing_tol_m: float = 0.15) -> bool:
    """Whether the top and at least two competitors scoring >= ``ratio_min`` of it lie on one line at even spacing
    (multiples of one period within ``spacing_tol_m``): a repeated structure such as shelf bays."""
    if len(candidates) < 3 or candidates[0].search_score <= 0:
        return False
    top = candidates[0]
    strong = [c for c in candidates[1:] if c.search_score / top.search_score >= ratio_min]
    if len(strong) < 2:
        return False
    vec = np.array([[c.search_pose[0] - top.search_pose[0], c.search_pose[1] - top.search_pose[1]] for c in strong])
    for a in range(len(vec)):
        u = vec[a] / (np.linalg.norm(vec[a]) or 1.0)
        along = vec @ u
        across = np.abs(vec @ np.array([-u[1], u[0]]))
        on = (across <= line_tol_m) & (np.abs(along) > spacing_tol_m)
        if on.sum() < 2:
            continue
        period = np.min(np.abs(along[on]))
        multiples = np.abs(along[on]) / period
        if np.all(np.abs(multiples - np.round(multiples)) * period <= spacing_tol_m):
            return True
    return False


def decide(match: GlobalMatch, *, e_min: float, r_max: float, r_struct: float | None = None,
           sep_m: float = 0.3, sep_rad: float = 0.1) -> Decision:
    """Plan B2's confirmation: the top's endpoint fraction (at its refined pose) >= ``e_min``, the best competitor's
    lattice score ratio <= ``r_max``, (with ``r_struct``) no evenly spaced row of competitors scoring >= ``r_struct``
    of the top, and the refinement not having moved the top beyond the wrong-confirmation tolerance from its lattice
    pose (a top that walked away is not the candidate the search scored)."""
    if match.status != "ok" or not match.candidates:
        return Decision(False, match.status if match.status != "ok" else "uninformative")
    top = match.candidates[0]
    ratio = (match.candidates[1].search_score / top.search_score
             if len(match.candidates) > 1 and top.search_score > 0 else 0.0)
    lp = top.search_pose
    if math.hypot(top.pose[0] - lp[0], top.pose[1] - lp[1]) >= sep_m or _yaw_gap(top.pose[2], lp[2]) >= sep_rad:
        return Decision(False, "refine_moved", ratio)
    if not (top.endpoint_fraction >= e_min):  # NaN fails
        return Decision(False, "low_fit", ratio)
    if not (ratio <= r_max):
        return Decision(False, "competitor", ratio)
    if r_struct is not None and periodic(match.candidates, ratio_min=r_struct):
        return Decision(False, "periodic", ratio)
    return Decision(True, "confirmed", ratio)


__all__ = ["Candidate", "Decision", "GlobalMatch", "MapIndex", "MatchConfig", "brute_force_best", "candidate_cells", "decide",
           "endpoint_fraction", "global_match", "likelihood_field", "periodic", "refine", "relocalize", "scan_points",
           "stack_scans", "suppress"]
