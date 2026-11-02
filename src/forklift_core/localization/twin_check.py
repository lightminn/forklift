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


def _same(a, b) -> bool:
    d, a_ = _gap(a, b)
    return d <= 1e-9 and a_ <= 1e-9


def hypotheses(b, *, pool=(), leaves=(), synthetic=()) -> list:
    """(pose, source) of every hypothesis other than b. Only exact repeats go: two centres within each other's success
    tolerance still search different +-REGION_M regions, and the check must cover their union (Codex piece 1
    re-review)."""
    out = []
    items = [(tuple(float(v) for v in p), "pool") for p in pool] + \
            [(tuple(float(v) for v in p), "leaf") for p in leaves] + \
            [(tuple(float(v) for v in p), src) for p, src in synthetic]
    for pose, src in items:
        if _same(pose, b) or any(_same(pose, q) for q, _ in out):
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
    searched: list = field(default_factory=list)  # (centre, source) of every region searched, in order
    work: int = 0
    wall_s: float = 0.0


def _leaves_above(s: G._Search, cfg: G.MatchConfig, thr: float):
    """Every lattice leaf scoring >= ``thr`` (an absolute score): the descent prunes only nodes whose upper bound is
    below ``thr``, so none is lost to a better leaf elsewhere (Codex piece 1 review: G._lattice_leaves prunes relative
    to its own best, which may stand outside the region). Raises G._Stop on the budget or a frontier over max_nodes."""
    h, r, c = s.top()
    bounds = s.scores(cfg.levels, h, r, c)
    for level in range(cfg.levels, 0, -1):
        keep = bounds >= thr
        h, r, c = h[keep], r[keep], c[keep]
        if len(h) * 4 > cfg.max_nodes:
            raise G._Stop("overflow")
        h, r, c = s.children(level, h, r, c)
        bounds = s.scores(level - 1, h, r, c)
    keep = bounds >= thr
    return h[keep], r[keep], c[keep], bounds[keep]


def _region_leaves(grid: Grid, pts, cfg, cons: ConservativeIndex, region, budget, thr: float):
    """Every conservative lattice leaf scoring >= ``thr`` whose base lies in ``region`` (+-REGION_M, +-REGION_RAD):
    (bases, scores), or None when no laser cell in the region is possible."""
    x0, y0, yaw0 = region
    cells = L._laser_disc(grid, (x0, y0), REGION_M + abs(LASER_X_M))
    n_yaw = max(1, int(round(2 * math.pi / cfg.yaw_step_rad)))
    yaws = np.arange(n_yaw) * (2 * math.pi / n_yaw)
    yaw_index = np.flatnonzero(L._yaw_gaps(yaws, yaw0) <= REGION_RAD + 1e-9)
    s = G._Search(grid, pts, cfg, cons, budget, cells=cells, yaw_index=yaw_index)
    if not s.cand.any() or not len(yaw_index):
        return None
    h, r, c, sc = _leaves_above(s, cfg, thr)
    if not len(sc):
        return np.zeros((0, 3)), np.zeros(0)
    bases = L._bases(np.array([s.pose(h[k], r[k], c[k]) for k in range(len(h))]))
    inside = (np.hypot(bases[:, 0] - x0, bases[:, 1] - y0) <= REGION_M) & \
             (L._yaw_gaps(bases[:, 2], yaw0) <= REGION_RAD + 1e-9)
    return bases[inside], sc[inside]


def residual_ambiguity(grid: Grid, points, b, hyps, cfg: G.MatchConfig | None = None, *, e_min: float,
                       r_resid: float = R_RESID, index: G.MapIndex | None = None,
                       cons: ConservativeIndex | None = None, clock=None, deadline: float | None = None,
                       hyp_status: str = "ok", cancel=None) -> Residual:
    """The plan's 1.2 check of b against ``hyps`` ((pose, source), b excluded). ``hyp_status`` is how the hypothesis
    set's own search ended: anything but ``ok`` leaves b unconfirmed (plan 1.2, a set that could not be generated)."""
    cfg = cfg or G.MatchConfig()
    t0 = time.monotonic()
    pts = G._thin(np.asarray(points, float).reshape(-1, 2), grid.resolution_m)
    index = index or G.MapIndex(grid, cfg)
    window = L.Window(base=tuple(float(v) for v in b), radius_m=REGION_M, yaw_rad=REGION_RAD, edge_m=0.0, edge_rad=0.0)
    own = L.local_match(grid, points, window, cfg, e_min=e_min, r_amb=2.0, index=index, clock=clock, deadline=deadline,
                        cancel=cancel)
    out = Residual(False, (), base=own.base, a_b=own.amb_ratio, work=own.work)
    early = () if hyp_status == "ok" else (f"hypotheses:{hyp_status}",)
    if not own.confirmed:
        out.reasons, out.wall_s = early + (f"believed:{own.reason}",), time.monotonic() - t0
        return out
    s_b = G._field_score(grid, index.field, pts, R.laser_from_base(own.base))
    out.s_b = s_b
    reasons = list(early)
    if own.amb_ratio is not None and own.amb_ratio >= r_resid:
        reasons.append("believed:ambiguous")
    cons = cons or ConservativeIndex(grid, cfg)
    budget = G._Budget(cfg, clock, deadline, cancel)
    b = tuple(float(v) for v in b)
    # b refined and b as believed: different centres, so both regions (Codex piece 1 re-review)
    regions = [(own.base, "believed")] + ([] if _same(b, own.base) else [(b, "believed-start")])
    regions += [(p, src) for p, src in hyps if not any(_same(p, q) for q, _ in regions)]
    try:
        for pose, src in regions:
            got = _region_leaves(grid, pts, cfg, cons, pose, budget, r_resid * s_b)
            out.regions += 1
            out.searched.append((pose, src))
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


# --- piece 2: the start check's entry and the pallet zones in odometry coordinates -------------------------------------
def last_leaves(grid: Grid, points, cfg: G.MatchConfig, index: G.MapIndex, r_resid: float = R_RESID, *, clock=None,
                deadline: float | None = None, cancel=None) -> tuple:
    """(status, leaves): one global lattice search on ``points`` (the frozen matcher's own search, not changed) and its
    pre-suppression leaves scoring >= ``r_resid`` of the best, as base poses. Deterministic for the same grid, points
    and settings once it finishes; the deadline and cancel reach the whole search (Codex piece 2 draft 1 P2-7)."""
    pts = G._thin(np.asarray(points, float).reshape(-1, 2), grid.resolution_m)
    if len(pts) < cfg.min_points or not index.cand.any() or not grid.occupied().any():
        return "uninformative", []
    try:
        s = G._Search(grid, pts, cfg, index, G._Budget(cfg, clock, deadline, cancel))
        h, r, c, sc = G._lattice_leaves(s, cfg)
    except G._Stop as stop:
        return str(stop), []
    if not len(sc):
        return "ok", []
    keep = np.flatnonzero(sc >= r_resid * float(sc.max()))
    return "ok", [tuple(float(v) for v in p) for p in L._bases(np.array([s.pose(h[k], r[k], c[k]) for k in keep]))]


@dataclass
class StartCheck:
    residual: Residual
    hyps: list  # (pose, source) at the live scan's time, the believed start excluded
    statuses: dict  # the leaf searches' ends


def start_check(grid: Grid, points, b, *, pool=(), gen_points=None, carry=None, cfg: G.MatchConfig | None = None,
                e_min: float, r_resid: float = R_RESID, index: G.MapIndex | None = None,
                cons: ConservativeIndex | None = None, clock=None, deadline: float | None = None,
                cancel=None) -> StartCheck:
    """The start check after the live start (plan 2.2): the believed start ``b`` and the live scan set ``points`` at
    O_scan; the generation's pool and the leaves of its last global search (``gen_points``) carried to O_scan's time by
    ``carry``; the live set's own leaves added; the synthetic twins of ``b``. One deadline for all of it."""
    cfg = cfg or G.MatchConfig()
    index = index or G.MapIndex(grid, cfg)
    carry = carry or (lambda p: tuple(float(v) for v in p))
    kw = {"clock": clock, "deadline": deadline, "cancel": cancel}
    st_live, live = last_leaves(grid, points, cfg, index, r_resid, **kw)
    st_gen, gen = ("ok", []) if gen_points is None else last_leaves(grid, gen_points, cfg, index, r_resid, **kw)
    extra = [(carry(p), "gen-leaf") for p in gen] + [(p, "leaf") for p in live]
    hyps = hypotheses(b, pool=[carry(p) for p in pool], synthetic=extra + synthetic_twins(b))
    status = next((st for st in (st_gen, st_live) if st != "ok"), "ok")
    res = residual_ambiguity(grid, points, b, hyps, cfg, e_min=e_min, r_resid=r_resid, index=index, cons=cons,
                             hyp_status=status, **kw)
    return StartCheck(res, hyps, {"generation": st_gen, "live": st_live})


def odom_zones(zones, hyps, o_s) -> list:
    """(zone, rho0) per hypothesis and pallet zone: Z_h = O_s o h_s^-1 o z in odometry coordinates (plan 3.3) --
    where the pallet is if h_s (a base pose at O_s's time) is true; it does not move with the odometry. rho0: the
    farthest corner's distance from the truck's base link at O_s."""
    out = []
    for k, h in enumerate(hyps):
        rel = compose(tuple(o_s), invert(tuple(h)))
        for z in zones:
            x, y, yaw = compose(rel, (z.x_m, z.y_m, z.yaw_rad))
            c, s_ = math.cos(yaw), math.sin(yaw)
            rho0 = max(math.hypot(x + c * a - s_ * w - o_s[0], y + s_ * a + c * w - o_s[1])
                       for a in (-z.length_m / 2, z.length_m / 2) for w in (-z.width_m / 2, z.width_m / 2))
            out.append((type(z)(f"{z.zone_id}@h{k}", x, y, z.length_m, z.width_m, wrap(yaw)), rho0))
    return out


def grown_zones(zones_rho, d_upper: float, bounds) -> list:
    """Each zone grown by design 1.4's (0.30 + dp(d) + 2 d sin 0.05) + 2 rho sin((0.10 + dth(d)) / 2), at the upper
    end ``d_upper`` of the net displacement's 0.05 m bin and rho <= rho0 + d_upper (the triangle inequality: d is the
    net displacement from O_s, Codex piece 2 draft 3 P2)."""
    d = float(d_upper)
    out = []
    for z, rho0 in zones_rho:
        g = (REGION_M + bounds.dp(d) + 2 * d * math.sin(REGION_RAD / 2)
             + 2 * (rho0 + d) * math.sin((REGION_RAD + bounds.dth(d)) / 2))
        out.append(type(z)(z.zone_id, z.x_m, z.y_m, z.length_m + 2 * g, z.width_m + 2 * g, z.yaw_rad))
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
        # b' keeps its own region: g agrees with it only within E_AGREE (Codex piece 1 review)
        here = hypotheses(g, synthetic=[(b_now, "moved-start")] + [(moved(h, o0, ot), src) for h, src in hyps])
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
           "SUCCESS_M", "SUCCESS_RAD", "StartCheck", "e_agree", "grown_zones", "hypotheses", "last_leaves", "moved",
           "odom_zones", "rematch_decision", "residual_ambiguity", "start_check", "synthetic_twins", "wrap"]
