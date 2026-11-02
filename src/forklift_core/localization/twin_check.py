"""The start check of the unconfirmed position plan (docs/plans/2026-11-02-unconfirmed-position.md 1.2).

A believed start b (base_link, map) is confirmed only when no pose outside b's success tolerance could explain the
scan as well, judged over a hypothesis set -- the matcher's pool, the last global search's pre-suppression leaves and
synthetic twins (bay shifts along the map axes and turnarounds) -- and judged conservatively:

- every lattice leaf in each hypothesis's region (+-REGION_M, +-REGION_RAD), not one refined pose;
- endpoints falling on unknown survey cells count as hits, and the laser may stand on unknown cells: unknown cannot
  lower a hypothesis's score (the plan's second-match counterexample: unknown hid the true bay);
- a region is impossible, and recorded as such, only when no laser cell in it is possible (free or unknown, the
  matcher's clearance from occupied cells).

b itself is the F1 local re-match around b on the real survey grid: its refined pose, the score there s(b) and F1's
ambiguity ratio a(b). Residual ambiguity: a(b) >= R_RESID, any competing leaf u >= R_RESID * s(b), or a search that
could not finish. Pure: no runner, no clock but the one passed in.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from forklift_core.localization import global_match as G
from forklift_core.localization import local_match as L
from forklift_core.localization import start_reloc as R
from forklift_core.localization.relocalization_samples import LASER_X_M
from forklift_core.localization.slam_map import Grid
from forklift_core.localization.slam_pose import compose, invert

R_RESID = 0.98  # fixed before results (the lead's decision 2)
SUCCESS_M, SUCCESS_RAD = 0.10, 0.05  # B-P2 4.0's success tolerance
REGION_M, REGION_RAD = 0.30, 0.10  # a hypothesis's region (F1's refine_moved)
BAY_M = 6.0  # this hall's repeated bay pitch (plan 1.2; set per hall)
D_MIN = 0.5  # plan 3: the straight reverse (odometry) before the re-match
MULTIPLES = (1, 2, 3)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _gap(a, b) -> tuple:
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(wrap(a[2] - b[2]))


def synthetic_twins(b, bay_m: float = BAY_M, multiples=MULTIPLES) -> list:
    """b shifted by +-k bay pitches along the map's x and y, each also turned around, and b turned around."""
    x, y, yaw = (float(v) for v in b)
    out = []
    for k in multiples:
        for dx, dy, name in ((k * bay_m, 0.0, f"x+{k}"), (-k * bay_m, 0.0, f"x-{k}"),
                             (0.0, k * bay_m, f"y+{k}"), (0.0, -k * bay_m, f"y-{k}")):
            out.append(((x + dx, y + dy, yaw), f"shift {name}"))
            out.append(((x + dx, y + dy, wrap(yaw + math.pi)), f"turn {name}"))
    out.append(((x, y, wrap(yaw + math.pi)), "turn"))
    return out


def hypotheses(b, *, pool=(), leaves=(), synthetic=()) -> list:
    """(pose, source) of every hypothesis other than b; poses within b's success tolerance are b, and two within each
    other's success tolerance are one (only those: a pose further away is another hypothesis)."""
    out = []
    items = [(tuple(float(v) for v in p), "pool") for p in pool] + \
            [(tuple(float(v) for v in p), "leaf") for p in leaves] + \
            [(tuple(float(v) for v in p), src) for p, src in synthetic]
    for pose, src in items:
        d, a = _gap(pose, b)
        if d <= SUCCESS_M and a <= SUCCESS_RAD:
            continue
        if any(_gap(pose, q)[0] <= SUCCESS_M and _gap(pose, q)[1] <= SUCCESS_RAD for q, _ in out):
            continue
        out.append((pose, src))
    return out


class ConservativeIndex:
    """The global matcher's index on a field where unknown cells are hits, with laser cells on free or unknown cells
    (the matcher's clearance from occupied cells): no unknown cell can lower a score or rule a pose out."""

    def __init__(self, grid: Grid, cfg: G.MatchConfig):
        from scipy.ndimage import distance_transform_edt  # noqa: PLC0415

        G._check_grid(grid)
        data = np.asarray(grid.data)
        hits = Grid(np.where(data < 0, 100, data).astype(data.dtype), grid.origin, grid.resolution_m)
        base = G.MapIndex(hits, cfg)
        self.grid, self.cfg, self.field = grid, cfg, base.field
        self.reach, self.pad, self.L, self.walls = base.reach, base.pad, base.L, base.walls
        occ = grid.occupied()
        possible = (data == 0) | (data < 0)
        d = distance_transform_edt(~occ) * grid.resolution_m if occ.any() else np.full(occ.shape, np.inf)
        self.cand = possible & (d >= cfg.clearance_m)
        self.C = G._pyramid(self.cand, cfg.levels, np.logical_or)


@dataclass
class Residual:
    confirmed: bool
    reasons: tuple
    base: tuple | None = None  # b refined by the F1 local re-match
    s_b: float | None = None
    a_b: float | None = None
    competitors: list = field(default_factory=list)  # (pose, conservative score, source)
    impossible: list = field(default_factory=list)  # (pose, source)
    regions: int = 0
    work: int = 0
    wall_s: float = 0.0


def _region_leaves(grid: Grid, pts, cfg, cons: ConservativeIndex, region, budget):
    """Every conservative lattice leaf whose base lies in ``region`` (+-REGION_M, +-REGION_RAD): (bases, scores), or
    None when no laser cell in the region is possible."""
    x0, y0, yaw0 = region
    cells = L._laser_disc(grid, (x0, y0), REGION_M + abs(LASER_X_M))
    n_yaw = max(1, int(round(2 * math.pi / cfg.yaw_step_rad)))
    yaws = np.arange(n_yaw) * (2 * math.pi / n_yaw)
    yaw_index = np.flatnonzero(L._yaw_gaps(yaws, yaw0) <= REGION_RAD + 1e-9)
    s = G._Search(grid, pts, cfg, cons, budget, cells=cells, yaw_index=yaw_index)
    if not s.cand.any() or not len(yaw_index):
        return None
    h, r, c, sc = G._lattice_leaves(s, cfg)
    if not len(sc):
        return np.zeros((0, 3)), np.zeros(0)
    bases = L._bases(np.array([s.pose(h[k], r[k], c[k]) for k in range(len(h))]))
    inside = (np.hypot(bases[:, 0] - x0, bases[:, 1] - y0) <= REGION_M) & (L._yaw_gaps(bases[:, 2], yaw0) <= REGION_RAD)
    return bases[inside], sc[inside]


def residual_ambiguity(grid: Grid, points, b, hyps, cfg: G.MatchConfig | None = None, *, e_min: float,
                       r_resid: float = R_RESID, index: G.MapIndex | None = None,
                       cons: ConservativeIndex | None = None, clock=None, deadline: float | None = None) -> Residual:
    """The plan's 1.2 check of b against ``hyps`` ((pose, source), b excluded)."""
    cfg = cfg or G.MatchConfig()
    t0 = time.monotonic()
    pts = G._thin(np.asarray(points, float).reshape(-1, 2), grid.resolution_m)
    index = index or G.MapIndex(grid, cfg)
    window = L.Window(base=tuple(float(v) for v in b), radius_m=REGION_M, yaw_rad=REGION_RAD, edge_m=0.0, edge_rad=0.0)
    own = L.local_match(grid, points, window, cfg, e_min=e_min, r_amb=2.0, index=index, clock=clock, deadline=deadline)
    out = Residual(False, (), base=own.base, a_b=own.amb_ratio, work=own.work)
    if not own.confirmed:
        out.reasons, out.wall_s = (f"believed:{own.reason}",), time.monotonic() - t0
        return out
    s_b = G._field_score(grid, index.field, pts, R.laser_from_base(own.base))
    out.s_b = s_b
    reasons = []
    if own.amb_ratio is not None and own.amb_ratio >= r_resid:
        reasons.append("believed:ambiguous")
    cons = cons or ConservativeIndex(grid, cfg)
    budget = G._Budget(cfg, clock, deadline)
    regions = [(own.base, "believed")] + list(hyps)
    try:
        for pose, src in regions:
            got = _region_leaves(grid, pts, cfg, cons, pose, budget)
            out.regions += 1
            if got is None:
                out.impossible.append((pose, src))
                continue
            bases, scores = got
            for k in range(len(scores)):
                d, a = _gap(bases[k], own.base)
                if (d > SUCCESS_M or a > SUCCESS_RAD) and scores[k] >= r_resid * s_b:
                    out.competitors.append((tuple(float(v) for v in bases[k]), float(scores[k]), src))
    except G._Stop as stop:
        reasons.append(f"search:{stop}")
    if out.competitors:
        reasons.append("competitor")
    out.work += budget.work
    out.reasons, out.confirmed, out.wall_s = tuple(reasons), not reasons, time.monotonic() - t0
    return out






# --- the re-match after a straight reverse (plan 1.5) ---------------------------------------------------------------
def moved(pose, o0, ot) -> tuple:
    """``pose`` carried by the odometry step O(0) -> O(t), expressed in its own frame: pose o O(0)^-1 o O(t) (a
    turned-around hypothesis moves the other way on the map)."""
    x, y, yaw = compose(pose, compose(invert(o0), ot))
    return (x, y, wrap(yaw))


def e_agree(d: float, dp: float = 0.0, dth: float = 0.0) -> tuple:
    """Plan 1.5: two estimates each within the success tolerance, the odometry bound and b's heading error turning the
    move: 0.20 m + dp + 0.05 d, 0.10 rad + dth."""
    return 0.20 + dp + 0.05 * d, 0.10 + dth


@dataclass
class Rematch:
    outcome: str  # too_short | unconfirmed | confirm | twin_preferred | out_of_set
    d: float
    matched: str | None = None  # the hypothesis source a twin_preferred re-match landed on
    agree_m: float | None = None  # g's distance from the moved start
    agree_rad: float | None = None
    residual: Residual | None = None


def rematch_decision(grid: Grid, points, b, hyps, o0, ot, *, decision: str, g, cfg: G.MatchConfig | None = None,
                     e_min: float, d: float | None = None, dp=lambda d: 0.0, dth=lambda d: 0.0, d_min: float = D_MIN,
                     index: G.MapIndex | None = None, cons: ConservativeIndex | None = None, clock=None,
                     deadline: float | None = None) -> Rematch:
    """Plan 1.5 after the truck stood: ``decision`` and ``g`` are the frozen relocalizer's on this place's scans
    (``points``); ``hyps`` the start's hypothesis set. ``confirm`` still needs the SLAM agreement (piece 2);
    ``twin_preferred`` and ``out_of_set`` end the session; ``unconfirmed`` waits for another stroke."""
    if d is None:
        d = math.hypot(ot[0] - o0[0], ot[1] - o0[1])
    if d < d_min:
        return Rematch("too_short", d)
    if decision != "confirmed" or g is None:
        return Rematch("unconfirmed", d)
    g = tuple(float(v) for v in g)
    tol_m, tol_rad = e_agree(d, dp(d), dth(d))
    b_now = moved(b, o0, ot)
    dm, da = _gap(g, b_now)
    out = Rematch("unconfirmed", d, agree_m=dm, agree_rad=da)
    if dm <= tol_m and da <= tol_rad:
        # the conservative check at this place too: an unattended confirmation alone misses a twin unknown hides
        here = hypotheses(g, synthetic=[(moved(h, o0, ot), src) for h, src in hyps])
        out.residual = residual_ambiguity(grid, points, g, here, cfg, e_min=e_min, index=index, cons=cons,
                                          clock=clock, deadline=deadline)
        out.outcome = "confirm" if out.residual.confirmed else "unconfirmed"
        return out
    for h, src in hyps:
        hm, ha = _gap(g, moved(h, o0, ot))
        if hm <= tol_m and ha <= tol_rad:
            out.outcome, out.matched = "twin_preferred", src
            return out
    out.outcome = "out_of_set"
    return out

__all__ = ["BAY_M", "ConservativeIndex", "D_MIN", "R_RESID", "REGION_M", "REGION_RAD", "Rematch", "Residual",
           "SUCCESS_M", "SUCCESS_RAD", "e_agree", "hypotheses", "moved", "rematch_decision", "residual_ambiguity",
           "synthetic_twins", "wrap"]
