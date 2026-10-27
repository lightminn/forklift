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
    keep_ratio: float = 0.4  # every leaf scoring >= this share of the best is kept: 0.10 below the lowest r_max (B4a)
    sep_m: float = 0.3  # plan B1: the wrong-confirmation tolerance
    sep_rad: float = 0.1
    clearance_m: float = 0.3  # the laser stands where the truck can: this far from any occupied cell
    min_points: int = 30  # fewer endpoints is no information
    max_candidates: int = 8
    max_nodes: int = 4_000_000  # a frontier larger than this is an overflow (no answer, never a confirmation);
    # 400,000 overflowed 2 of the 3 dev smoke starts on line_m (plan B4a settings history)
    endpoint_dilate_cells: int = 1  # the endpoint fraction's tolerance, as slam_map.endpoint_fraction


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
    status: str  # ok | uninformative | overflow
    candidates: tuple = ()  # the best first, then the competitors beyond the separation


@dataclass(frozen=True)
class Decision:
    confirmed: bool
    reason: str  # confirmed | uninformative | overflow | refine_moved | low_fit | competitor | periodic
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


class _Search:
    def __init__(self, grid: Grid, pts: np.ndarray, cfg: MatchConfig):
        self.grid, self.cfg, self.res = grid, cfg, grid.resolution_m
        self.n_yaw = max(1, int(round(2 * math.pi / cfg.yaw_step_rad)))
        self.yaws = np.arange(self.n_yaw) * (2 * math.pi / self.n_yaw)
        field = likelihood_field(grid, cfg.sigma_m)
        cand = candidate_cells(grid, cfg.clearance_m)
        reach = int(math.ceil(np.abs(pts).max() / self.res)) + 2 if len(pts) else 2
        self.pad = reach + (1 << cfg.levels) + 1
        self.L = [np.pad(lv, self.pad) for lv in _pyramid(field, cfg.levels, np.maximum)]
        self.C = _pyramid(cand, cfg.levels, np.logical_or)
        self.cand = cand
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
        h = np.repeat(np.arange(self.n_yaw), len(rs))
        return h, np.tile(rs, self.n_yaw), np.tile(cs, self.n_yaw)

    def pose(self, h, r, c) -> tuple:
        ox, oy, _ = self.grid.origin
        return (ox + (c + 0.5) * self.res, oy + (r + 0.5) * self.res, float(self.yaws[h]))


def global_match(grid: Grid, points: np.ndarray, cfg: MatchConfig | None = None) -> GlobalMatch:
    """The kept candidates (lattice poses, before refinement) or why there are none."""
    cfg = cfg or MatchConfig()
    _check_grid(grid)
    pts = _thin(np.asarray(points, float).reshape(-1, 2), grid.resolution_m)
    if len(pts) < cfg.min_points or not candidate_cells(grid, cfg.clearance_m).any() or not grid.occupied().any():
        return GlobalMatch("uninformative")
    s = _Search(grid, pts, cfg)
    h, r, c = s.top()
    bounds = s.scores(cfg.levels, h, r, c)
    # a first leaf by greedy descent gives a floor for the threshold
    i = int(np.argmax(bounds))
    gh, gr, gc = h[i:i + 1], r[i:i + 1], c[i:i + 1]
    for level in range(cfg.levels, 0, -1):
        gh, gr, gc = s.children(level, gh, gr, gc)
        b = s.scores(level - 1, gh, gr, gc)
        j = int(np.argmax(b))
        gh, gr, gc = gh[j:j + 1], gr[j:j + 1], gc[j:j + 1]
    floor = float(s.scores(0, gh, gr, gc)[0])
    thr = cfg.keep_ratio * floor
    for level in range(cfg.levels, 0, -1):
        keep = bounds >= thr
        h, r, c = h[keep], r[keep], c[keep]
        if len(h) * 4 > cfg.max_nodes:
            return GlobalMatch("overflow")
        h, r, c = s.children(level, h, r, c)
        bounds = s.scores(level - 1, h, r, c)
    best = float(bounds.max()) if len(bounds) else floor
    keep = bounds >= cfg.keep_ratio * best
    h, r, c, sc = h[keep], r[keep], c[keep], bounds[keep]
    order = np.argsort(-sc, kind="stable")
    leaves = [Candidate(s.pose(h[k], r[k], c[k]), float(sc[k]), math.nan) for k in order]
    kept = suppress(leaves, sep_m=cfg.sep_m, sep_rad=cfg.sep_rad)[:cfg.max_candidates]
    return GlobalMatch("ok", tuple(_with_fraction(grid, pts, k, cfg) for k in kept))


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


def refine(grid: Grid, match: GlobalMatch, points: np.ndarray, cfg: MatchConfig | None = None) -> GlobalMatch:
    """Point-to-line ICP of every kept candidate against the occupied cells near it, each rescored on the same field
    at its refined pose (an unaccepted ICP keeps the lattice pose, unrefined). The order and the lattice scores stay:
    the competitor ratio is read from the exact lattice search, whose kept set is complete down to ``keep_ratio``;
    refinement only places the pose (Codex B4a re-review P2: an ICP from a weak lattice pose can climb, and
    candidates can converge together, so refined scores would not be a complete competitor set)."""
    cfg = cfg or MatchConfig()
    if match.status != "ok":
        return match
    pts = _thin(np.asarray(points, float).reshape(-1, 2), grid.resolution_m)
    field = likelihood_field(grid, cfg.sigma_m)
    walls = grid.cell_centres(grid.occupied())
    out = []
    for cand in match.candidates:
        near = walls[np.hypot(walls[:, 0] - cand.pose[0], walls[:, 1] - cand.pose[1]) <= cfg.match_range_m + 0.5]
        res = match_scans(near, pts, cand.pose, max_correspondence_m=cfg.sep_m)
        pose = tuple(float(v) for v in res.pose) if res.accepted else cand.pose
        out.append(_with_fraction(grid, pts, Candidate(pose, _field_score(grid, field, pts, pose), math.nan,
                                                       res.accepted, cand.search_pose, cand.search_score), cfg))
    return GlobalMatch("ok", tuple(out))


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


__all__ = ["Candidate", "Decision", "GlobalMatch", "MatchConfig", "brute_force_best", "candidate_cells", "decide",
           "endpoint_fraction", "global_match", "likelihood_field", "periodic", "refine", "scan_points",
           "stack_scans", "suppress"]
