"""S1 of the mass follow-up (docs/plans/2026-11-06-mass-along-followup-intervals-map.md, section 3): a rigid alignment of
the SLAM's published map to truth scan endpoints in a common region around the delivery reference, with every
reported value given as an interval over the near-optimal transforms, and the synthetic precision tests that fix the
cost tolerance and the method floor.

    python tools/mass_along_map.py precision --json PRECISION   # before the real records

The pair alignment is called from tools/mass_along_followup.py."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage, optimize

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mass_along_judge as J  # noqa: E402
import mass_along_mechanism as M  # noqa: E402
from forklift_core.localization.slam_pose import compose, invert  # noqa: E402

CUT_M = 0.15  # truncated square loss
SEARCH_T_M = 0.30
SEARCH_PSI = math.radians(2.0)
COARSE = (0.02, 0.02, math.radians(0.1))
FINE = (0.005, 0.005, math.radians(0.025))  # the final box size (three levels from COARSE)
ANGLE_SCALE_M = 2.0
REGION_M = 4.0
THIN_M = 0.05
SUPPORT_MARGIN_M = 0.10
MIN_SUPPORT_POINTS = 100
MIN_SUPPORT_SPREAD_M = 0.10
OCCUPIED = 100
FLOOR_DISTANCES_M = (0.0, 5.0, 10.0)
FLOOR_ROTATIONS = (0.0, math.radians(45.0), math.radians(90.0))
CHUNK = 4_000_000  # transform x point products per batch


# ---------------------------------------------------------------- geometry


def se2_apply(pose, pts: np.ndarray) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.column_stack([pose[0] + c * pts[:, 0] - s * pts[:, 1], pose[1] + s * pts[:, 0] + c * pts[:, 1]])


def se2_inv_apply(pose, pts: np.ndarray) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = pts[:, 0] - pose[0], pts[:, 1] - pose[1]
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


def world_transform(q, g) -> tuple[float, float, float]:
    """F = Q o G o Q^-1 for G = (a, l, psi) about Q."""
    return compose(compose(tuple(q), tuple(g)), invert(tuple(q)))


def along_at(f, t) -> float:
    """The along component of T^-1 o F o T: the estimate error in T's frame if the map were F applied to truth."""
    return float(compose(invert(tuple(t)), compose(tuple(f), tuple(t)))[0])


def along_batch(q, gs: np.ndarray, t) -> np.ndarray:
    """along_at(world_transform(q, g), t) for many G at once."""
    # T^-1 F T: the point T moved by F, seen in T's frame, plus the rotation part (which has no along component at T's
    # origin); along = x-component of T^-1 (F(T.p)) since T^-1 F T maps T's origin to T^-1 F(T.p)
    tq = se2_inv_apply(q, np.array([[t[0], t[1]]]))[0]  # T's position in Q's frame
    c, s = np.cos(gs[:, 2]), np.sin(gs[:, 2])
    mx = gs[:, 0] + c * tq[0] - s * tq[1]
    my = gs[:, 1] + s * tq[0] + c * tq[1]
    w = se2_apply(q, np.column_stack([mx, my]))
    d = w - np.array([t[0], t[1]])
    return d[:, 0] * math.cos(t[2]) + d[:, 1] * math.sin(t[2])


class MapGrid:
    """An int8 occupancy grid (-1 unknown, 0 free, 100 occupied); cell (r, c) centre at origin o (r, c) + 0.5 cells."""

    def __init__(self, data: np.ndarray, origin, resolution: float):
        self.data = np.asarray(data)
        self.origin = tuple(float(v) for v in (list(origin) + [0.0] * 3)[:3])
        self.res = float(resolution)
        occ = self.data >= OCCUPIED
        self.dist = (ndimage.distance_transform_edt(~occ) * self.res if occ.any()
                     else np.full(self.data.shape, np.inf))
        known = np.pad(self.data >= 0, 1, constant_values=False)
        self.dist_unknown = ndimage.distance_transform_edt(known)[1:-1, 1:-1] * self.res

    def cells(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        loc = se2_inv_apply(self.origin, pts)
        return loc[:, 1] / self.res - 0.5, loc[:, 0] / self.res - 0.5  # row, col

    def distance(self, pts: np.ndarray) -> np.ndarray:
        r, c = self.cells(pts)
        return ndimage.map_coordinates(self.dist, [r, c], order=1, mode="nearest")

    def clearance_to_unknown(self, pts: np.ndarray) -> np.ndarray:
        """Distance from the point's nearest cell centre to the nearest unknown or off-map cell centre, less the
        point's offset from that centre (so every cell centre within the returned radius of the point is known)."""
        r, c = self.cells(pts)
        ri, ci = np.rint(r).astype(int), np.rint(c).astype(int)
        inside = (ri >= 0) & (ri < self.data.shape[0]) & (ci >= 0) & (ci < self.data.shape[1])
        out = np.zeros(len(pts))
        off = np.hypot(r - ri, c - ci) * self.res
        out[inside] = self.dist_unknown[ri[inside], ci[inside]] - off[inside]
        return out


def costs(grid: MapGrid, q, local: np.ndarray, gs: np.ndarray, half: tuple | None = None):
    """Mean truncated square cost of the points (in Q's frame) under each G, against the grid. With ``half`` (half a
    lattice cell: a, l, psi) also each vertex's slack: the most the cost can drop anywhere in the cell around it,
    mean_i min(2 m_i e_i, m_i^2) with m_i = min(d_i, c) at the vertex and e_i = sqrt(2) (hypot(half a, half l)
    + r_i half psi) the point's largest move times the interpolated distance transform's Lipschitz bound."""
    out = np.empty(len(gs))
    sl = np.empty(len(gs)) if half is not None else None
    if half is not None:
        e = math.sqrt(2.0) * (math.hypot(half[0], half[1]) + np.hypot(local[:, 0], local[:, 1]) * half[2])
    step = max(1, CHUNK // max(1, len(local)))
    c2 = CUT_M * CUT_M
    for i in range(0, len(gs), step):
        g = gs[i:i + step]
        c, s = np.cos(g[:, 2])[:, None], np.sin(g[:, 2])[:, None]
        mx = g[:, 0][:, None] + c * local[:, 0] - s * local[:, 1]
        my = g[:, 1][:, None] + s * local[:, 0] + c * local[:, 1]
        w = se2_apply(q, np.column_stack([mx.ravel(), my.ravel()]))
        d = grid.distance(w).reshape(mx.shape)
        out[i:i + step] = np.mean(np.minimum(d * d, c2), axis=1)
        if half is not None:
            m = np.minimum(d, CUT_M)
            sl[i:i + step] = np.mean(np.minimum(2.0 * m * e, m * m), axis=1)
    return (out, sl) if half is not None else out


def lattice(step: tuple, lo=(-SEARCH_T_M, -SEARCH_T_M, -SEARCH_PSI), hi=(SEARCH_T_M, SEARCH_T_M, SEARCH_PSI)):
    axes = [np.arange(round(lo[k] / step[k]), round(hi[k] / step[k]) + 1) for k in range(3)]
    return np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)


def half(step: tuple) -> tuple:
    return tuple(v / 2 for v in step)


def align(grid: MapGrid, q, local: np.ndarray, delta: float) -> dict:
    """Box refinement: the search box tiled by coarse boxes (centre, half size); a box survives when its centre's cost
    less its slack (the most the cost can drop inside it) is within min + delta; survivors split into eight half-size
    boxes, down to the final level. S = the final survivors: every transform whose cost is within min + delta lies in
    one of them (the slack is a bound). 'min' is the least cost seen (any centre, and the continuous optimum started
    from the best coarse centre). Returns the optimum, S and whether S reaches the search box's edge."""
    radius = float(np.max(np.hypot(local[:, 0], local[:, 1]))) if len(local) else 0.0
    centres = lattice(COARSE) * np.array(COARSE)
    h = np.array(half(COARSE))
    cc, sc = costs(grid, q, local, centres, tuple(h))
    i0 = int(np.argmin(cc))
    scale = np.array([1.0, 1.0, ANGLE_SCALE_M])
    bounds = [(-SEARCH_T_M, SEARCH_T_M), (-SEARCH_T_M, SEARCH_T_M), (-SEARCH_PSI * ANGLE_SCALE_M, SEARCH_PSI * ANGLE_SCALE_M)]
    res = optimize.minimize(lambda x: float(costs(grid, q, local, (x / scale)[None, :])[0]), centres[i0] * scale,
                            method="Powell", bounds=bounds, options={"xtol": 1e-5, "ftol": 1e-10})
    g_opt = res.x / scale
    c_opt = float(costs(grid, q, local, g_opt[None, :])[0])
    cmin = min(c_opt, float(cc[i0]))
    coarse_order = np.argsort(cc)
    gap = float(cc[coarse_order[1]] - cc[coarse_order[0]]) if len(cc) > 1 else None
    counts = []
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)
    while True:
        keep = cc - sc <= cmin + delta
        counts.append(int(np.count_nonzero(keep)))
        centres, cc, sc = centres[keep], cc[keep], sc[keep]
        if np.all(2 * h <= np.array(FINE) + 1e-15):
            break
        centres = (centres[:, None, :] + signs[None, :, :] * (h / 2)).reshape(-1, 3)
        h = h / 2
        cc, sc = costs(grid, q, local, centres, tuple(h))
        cmin = min(cmin, float(np.min(cc)))
    keep = cc - sc <= cmin + delta
    S = centres[keep]
    edge = np.array([SEARCH_T_M, SEARCH_T_M, SEARCH_PSI]) - h
    touches = bool(np.any(np.abs(S) >= edge - 1e-12))
    d = grid.distance(se2_apply(q, local))
    d_opt = grid.distance(se2_apply(world_transform(q, g_opt), se2_apply(q, local)))
    return {"g_opt": [float(v) for v in g_opt], "cost_opt": c_opt, "cost_min": cmin, "radius_m": radius,
            "S": S, "S_size": int(len(S)), "survivors_per_level": counts, "touches_box": touches,
            "inlier_share": float(np.mean(d_opt < CUT_M)), "inlier_share_identity": float(np.mean(d < CUT_M)),
            "coarse_gap_top2": gap,
            "S_range": {k: [float(np.min(S[:, i])), float(np.max(S[:, i]))] for i, k in enumerate(("a", "l", "psi"))}
            if len(S) else None}


def value_interval(q, al: dict, t) -> dict:
    """A reported along value at pose T: at the optimum, and its interval over S widened by the fine half-cell's
    largest effect at T (half the translation diagonal plus T's distance from Q times half the yaw step)."""
    r = math.hypot(t[0] - q[0], t[1] - q[1])
    margin = math.hypot(FINE[0] / 2, FINE[1] / 2) + r * FINE[2] / 2
    vals = along_batch(q, al["S"], t)
    return {"opt": along_at(world_transform(q, al["g_opt"]), t), "lo": float(np.min(vals)) - margin,
            "hi": float(np.max(vals)) + margin, "distance_from_q_m": r}


def mean_interval(q, al: dict, poses: list) -> dict | None:
    """The mean along value over several poses: the interval of the mean over S (not the mean of intervals)."""
    if not poses:
        return None
    vals = np.mean([along_batch(q, al["S"], t) for t in poses], axis=0)
    rmax = max(math.hypot(t[0] - q[0], t[1] - q[1]) for t in poses)
    margin = math.hypot(FINE[0] / 2, FINE[1] / 2) + rmax * FINE[2] / 2
    f = world_transform(q, al["g_opt"])
    return {"opt": float(np.mean([along_at(f, t) for t in poses])), "lo": float(np.min(vals)) - margin,
            "hi": float(np.max(vals)) + margin, "distance_from_q_m": rmax, "n": len(poses)}


def pair_difference(a: dict | None, b: dict | None, floor: float | None) -> dict | None:
    """B - A over S_A x S_B: [lo_B - hi_A, hi_B - lo_A], widened by the method floor; decidable when it excludes 0."""
    if a is None or b is None:
        return None
    lo, hi = b["lo"] - a["hi"], b["hi"] - a["lo"]
    f = floor or 0.0
    return {"opt": b["opt"] - a["opt"], "lo": lo - f, "hi": hi + f, "floor": floor,
            "sign": (1 if lo - f > 0 else -1 if hi + f < 0 else 0) if floor is not None else None}


# ---------------------------------------------------------------- records


def scan_points(log: dict, laser: dict, stamps: set) -> np.ndarray:
    """Truth endpoints of the scans whose stamp is in ``stamps``: finite ranges with range_min < r < range_max, from
    the truth laser pose (2D)."""
    st = np.asarray(log["scan_stamps_s"], float)
    rng = np.asarray(log["scan_ranges_m"], float)
    pose = np.asarray(log["laser_pose_world"], float)
    ang = float(laser["angle_min_rad"]) + float(laser["angle_increment_rad"]) * np.arange(rng.shape[1])
    pts = []
    for i in np.nonzero(np.isin(st, np.array(sorted(stamps), float)))[0]:
        r = rng[i]
        ok = np.isfinite(r) & (r > float(laser["range_min_m"])) & (r < float(laser["range_max_m"]))
        a = pose[i, 2] + ang[ok]
        pts.append(np.column_stack([pose[i, 0] + r[ok] * np.cos(a), pose[i, 1] + r[ok] * np.sin(a)]))
    return np.concatenate(pts) if pts else np.zeros((0, 2))


def thin(pts: np.ndarray, cell: float = THIN_M) -> np.ndarray:
    """One point per world cell (the cell's mean), so repeated observations do not weigh more."""
    if not len(pts):
        return pts
    key = np.floor(pts / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    n = np.bincount(inv)
    return np.column_stack([np.bincount(inv, pts[:, 0]) / n, np.bincount(inv, pts[:, 1]) / n])


def support(points: np.ndarray, q, grids: list[MapGrid]) -> np.ndarray:
    """Points whose whole reach under the search box (sqrt(2) x 0.30 m + r x 2 deg + margin) is known in every grid."""
    r = np.hypot(points[:, 0] - q[0], points[:, 1] - q[1])
    need = math.hypot(SEARCH_T_M, SEARCH_T_M) + r * SEARCH_PSI + SUPPORT_MARGIN_M
    ok = np.ones(len(points), bool)
    for g in grids:
        ok &= g.clearance_to_unknown(points) >= need
    return ok


def support_problem(local: np.ndarray) -> str | None:
    if len(local) < MIN_SUPPORT_POINTS:
        return f"{len(local)} support points (< {MIN_SUPPORT_POINTS})"
    ev = np.linalg.eigvalsh(np.cov(local.T))
    if math.sqrt(max(ev[0], 0.0)) < MIN_SUPPORT_SPREAD_M:
        return f"support spread {math.sqrt(max(ev[0], 0.0)):.3f} m (< {MIN_SUPPORT_SPREAD_M})"
    return None


def load_maps(bridge: Path) -> tuple[list[dict], dict]:
    maps = json.loads((bridge / "maps.json").read_text())
    z = np.load(bridge / "maps.npz")
    return maps, z


def map_for(maps: list[dict], arrays, scan_id: int | None) -> tuple[MapGrid, dict] | None:
    """The last map whose after_scan_id is at most ``scan_id``."""
    if scan_id is None:
        return None
    ok = [m for m in maps if int(m["after_scan_id"]) <= scan_id]
    if not ok:
        return None
    m = max(ok, key=lambda x: int(x["index"]))
    return MapGrid(arrays[f"map_{int(m['index']):04d}"], m["origin"], float(m["resolution_m"])), m


def event_poses(events: list[tuple], records: list[dict], control: np.ndarray, offset: float) -> list[dict]:
    """The D1 events (M.correction_effects: the same selection) with the truth rear pose each is evaluated at:
    a processed scan's own truth_base moved to the rear axle, a release's first control row before the next event."""
    out = []
    times = [e[0] for e in events]
    for i, (t, kind, before, after, _jump) in enumerate(events):
        if before is None:
            continue
        nxt = times[i + 1] if i + 1 < len(times) else math.inf
        if kind == "processed":
            rec = next((r for r in records if r.get("status") == "processed" and abs(float(r["stamp_s"]) - t) < 1e-9
                        and r.get("odom_base") is not None and r.get("truth_base") is not None), None)
            if rec is None:
                continue
            odom = tuple(float(v) for v in rec["odom_base"])
            truth = J.rear_of(rec["truth_base"], offset)
            post = M.err3(truth, J.rear_of(compose(tuple(after), odom), offset))
        else:
            k = J.control_row_at_or_after(control, t)
            if k is None or control[k, 0] >= nxt:
                continue
            truth = tuple(control[k, 4:7])
            post = M.err3(truth, tuple(control[k, 1:4]))
        out.append({"t": t, "kind": kind, "truth": tuple(float(v) for v in truth), "post_along": float(post[0])})
    return out


def run_inputs(run: dict, split: dict) -> dict:
    """What S1 needs from one arm: records, the transport's processed stamps, the D1 events with poses, t_c's pose and
    e_c, the map choices and the delivery reference."""
    folder: Path = run["_folder"]
    rd = folder / "run"
    result = json.loads((rd / "result.json").read_text())
    control = np.load(rd / "slam_control.npy")
    records = json.loads((rd / "slam_records.json").read_text())
    z = np.load(rd / "slam_log.npz")
    log = {k: z[k] for k in z.files}
    meta = json.loads((rd / "meta.json").read_text())
    geom = meta["odometry_geometry"]
    offset = float((result.get("arguments") or {}).get("rear_axle_offset_m", geom.get("rear_axle_x_in_base_m")))
    t_m = run["_t_m"]
    windows = M.phase_windows(result, t_m, float(control[-1, 0]))
    tw, ew = windows["transport"], windows["approach_empty"]
    holds = (result.get("slam_summary") or {}).get("holds") or []
    events, bad = J.correction_events(records, holds)
    ev = [] if bad else [e for e in event_poses(events, records, control, offset) if tw and tw[0] <= e["t"] <= tw[1]]
    proc = lambda w: {float(r["stamp_s"]) for r in records  # noqa: E731
                      if r.get("status") == "processed" and w and w[0] <= float(r["stamp_s"]) <= w[1]}
    before_tm = [r for r in records if t_m is not None and float(r["stamp_s"]) < t_m]
    insert_t = ew[1] if ew else None
    before_ins = [r for r in records if insert_t is not None and float(r["stamp_s"]) < insert_t]
    out = {"log": log, "laser": meta["laser"], "transport_stamps": proc(tw), "approach_stamps": proc(ew), "events": ev,
           "replay_error": bad,
           "scan_id_processed": max((int(r["scan_id"]) for r in before_tm if r.get("status") == "processed"), default=None),
           "scan_id_any": max((int(r["scan_id"]) for r in before_tm), default=None),
           "scan_id_insert": max((int(r["scan_id"]) for r in before_ins if r.get("status") == "processed"), default=None),
           "t_m": t_m}
    if split.get("t_c") is not None:
        ic = J.control_row_at_or_after(control, float(split["t_c"]))
        if ic is not None:
            out["t_c"] = float(split["t_c"])
            out["truth_c"] = tuple(float(v) for v in control[ic, 4:7])
            out["e_c_along"] = M.err3(control[ic, 4:7], control[ic, 1:4])[0]
    try:
        dm = np.load(rd / "delivery_match_1.npz")
        out["reference_rear"] = tuple(float(v) for v in dm["reference_rear"])
    except (OSError, KeyError, ValueError):
        out["reference_rear"] = None
    out["maps"] = load_maps(folder / "bridge") if (folder / "bridge" / "maps.json").exists() else None
    return out


def floor_for(precision: dict | None, kind: str, distance: float) -> float | None:
    """The method floor for a value kind at a distance from Q: the smallest tested band at least that far (the
    farthest band, flagged by the caller, beyond it)."""
    if precision is None:
        return None
    bands = precision["floor"][kind]
    for d in sorted(float(k) for k in bands):
        if distance <= d + 1e-9:
            return float(bands[f"{d:g}"])
    return float(bands[f"{max(float(k) for k in bands):g}"])


def arm_values(q, al: dict, inp: dict) -> dict:
    ev = inp["events"]
    proc = [e for e in ev if e["kind"] == "processed"]
    out = {"all": mean_interval(q, al, [e["truth"] for e in ev]),
           "last3": mean_interval(q, al, [e["truth"] for e in ev[-3:]]),
           "processed": mean_interval(q, al, [e["truth"] for e in proc]),
           "d1_all_post_along": float(np.mean([e["post_along"] for e in ev])) if ev else None,
           "d1_last3_post_along": float(np.mean([e["post_along"] for e in ev[-3:]])) if ev else None,
           "d1_processed_post_along": float(np.mean([e["post_along"] for e in proc])) if proc else None,
           "events": len(ev), "processed_events": len(proc), "at_q_a": value_interval(q, al, q)}
    if "truth_c" in inp:
        out["at_t_c"] = value_interval(q, al, inp["truth_c"])
        out["e_c_along"] = inp["e_c_along"]
    return out


def align_choice(a_inp: dict, b_inp: dict, q, key: str, stamps_key: str, precision: dict | None) -> dict:
    delta = precision["delta"] if precision else 0.0
    ga, gb = map_for(*a_inp["maps"], a_inp[key]), map_for(*b_inp["maps"], b_inp[key])
    if ga is None or gb is None:
        return {"missing": "no map at or before the scan id"}
    pts = []
    for inp in (a_inp, b_inp):
        pts.append(scan_points(inp["log"], inp["laser"], inp[stamps_key]))
    allp = np.concatenate(pts) if pts else np.zeros((0, 2))
    allp = allp[np.hypot(allp[:, 0] - q[0], allp[:, 1] - q[1]) <= REGION_M]
    th = thin(allp)
    ok = support(th, q, [ga[0], gb[0]]) if len(th) else np.zeros(0, bool)
    local = se2_inv_apply(q, th[ok]) if np.any(ok) else np.zeros((0, 2))
    out = {"maps": {"A": {k: ga[1][k] for k in ("index", "after_scan_id", "stamp_ns")},
                    "B": {k: gb[1][k] for k in ("index", "after_scan_id", "stamp_ns")}},
           "points_region": int(len(th)), "points_support": int(len(local)),
           "support_share": float(len(local) / len(th)) if len(th) else None}
    why = support_problem(local)
    if why:
        return {**out, "not_measurable": f"common support too small: {why}"}
    res = {}
    for arm, g in (("A", ga[0]), ("B", gb[0])):
        al = align(g, q, local, delta)
        res[arm] = al
        out[arm] = {k: v for k, v in al.items() if k != "S"}
    if res["A"]["touches_box"] or res["B"]["touches_box"]:
        out["not_measurable"] = "candidate cells touch the search box"
    out["_aligned"] = res
    return out


def pair_alignment(a: dict, b: dict, split: dict, precision: dict | None = None) -> dict:
    """S1 for one pair (both arms reached the match): the two map choices, each aligned on the common support, every
    reported value as an interval and the pair differences against the D1 residuals."""
    if not (a["reached"] and b["reached"]):
        return {"missing": "an arm did not reach the match"}
    ai, bi = run_inputs(a, split.get("A") or {}), run_inputs(b, split.get("B") or {})
    if ai["maps"] is None or bi["maps"] is None:
        return {"missing": "no bridge maps"}
    if ai["reference_rear"] is None or bi["reference_rear"] is None:
        return {"missing": "no delivery reference"}
    q = ai["reference_rear"]
    out = {"reference_gap_m": math.hypot(ai["reference_rear"][0] - bi["reference_rear"][0],
                                         ai["reference_rear"][1] - bi["reference_rear"][1]),
           "replay_error": {"A": ai["replay_error"], "B": bi["replay_error"]}}
    for name, key, stamps in (("last_processed", "scan_id_processed", "transport_stamps"),
                              ("last_any", "scan_id_any", "transport_stamps"),
                              ("before_insert", "scan_id_insert", "approach_stamps")):
        ch = align_choice(ai, bi, q, key, stamps, precision)
        aligned = ch.pop("_aligned", None)
        if aligned is not None and name != "before_insert":
            va, vb = arm_values(q, aligned["A"], ai), arm_values(q, aligned["B"], bi)
            ch["values"] = {"A": va, "B": vb}
            d = {}
            for k in ("all", "last3", "processed", "at_q_a", "at_t_c"):
                kind = "at_q" if k == "at_q_a" else "event"
                dist = max((v[k] or {}).get("distance_from_q_m", 0.0) for v in (va, vb) if v.get(k))\
                    if va.get(k) and vb.get(k) else 0.0
                d[k] = pair_difference(va.get(k), vb.get(k), floor_for(precision, kind, dist))
            for k in ("d1_all_post_along", "d1_last3_post_along", "d1_processed_post_along", "e_c_along"):
                d[k] = vb[k] - va[k] if va.get(k) is not None and vb.get(k) is not None else None
            ch["differences"] = d
        elif aligned is not None:
            ch["values"] = {arm: {"at_q_a": value_interval(q, aligned[arm], q)} for arm in ("A", "B")}
        out[name] = ch
    return out


# ---------------------------------------------------------------- synthetic precision tests


def segments_for(name: str) -> list[tuple]:
    """Test geometries in Q's frame (segments x0, y0, x1, y1)."""
    rect = [(-3.0, -2.5, 3.5, -2.5), (3.5, -2.5, 3.5, 2.5), (3.5, 2.5, -3.0, 2.5), (-3.0, 2.5, -3.0, -2.5)]
    if name == "room":
        return rect + [(1.0, 1.0, 1.0, 2.5)]
    if name == "angled":
        c30, s30 = math.cos(math.radians(30)), math.sin(math.radians(30))
        return [(-3.0, -2.0, -3.0 + 6 * c30, -2.0 + 6 * s30), (2.0, -3.0, 2.0 - 4 * 0.7071, -3.0 + 4 * 0.7071),
                (-3.0, 2.5, 3.0, 2.5)]
    if name == "racks":
        cols = []
        for i in range(-3, 4):
            x = 1.0 * i
            cols += [(x - 0.05, 1.5, x + 0.05, 1.5), (x + 0.05, 1.5, x + 0.05, 1.6), (x + 0.05, 1.6, x - 0.05, 1.6),
                     (x - 0.05, 1.6, x - 0.05, 1.5)]
        return cols + [(-3.5, -2.0, 3.5, -2.0), (3.5, -2.0, 3.5, 2.0)]
    if name == "repeat":  # columns 0.25 m apart and nothing else along x: competing minima inside the box
        cols = []
        for i in range(-12, 13):
            x = 0.25 * i
            cols += [(x - 0.03, 1.0, x + 0.03, 1.0), (x - 0.03, -1.0, x + 0.03, -1.0)]
        return cols + [(-3.2, 1.5, -3.2, -1.5)]
    if name == "parallel":
        return [(-3.5, -1.5, 3.5, -1.5), (-3.5, 1.5, 3.5, 1.5)]
    raise ValueError(name)


def sample_points(segs: list[tuple], rng: np.random.Generator, spacing: float = 0.02, drop: float = 0.2) -> np.ndarray:
    pts = []
    for x0, y0, x1, y1 in segs:
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / spacing))
        u = np.linspace(0, 1, n)
        p = np.column_stack([x0 + u * (x1 - x0), y0 + u * (y1 - y0)])
        # occlusion: drop random runs
        keep = np.ones(n, bool)
        for _ in range(int(drop * n / 20)):
            s = rng.integers(0, n)
            keep[s:s + rng.integers(5, 30)] = False
        pts.append(p[keep] + rng.normal(0, 0.005, (np.count_nonzero(keep), 2)))
    return np.concatenate(pts)


def rasterize(segs: list[tuple], f, phase: tuple[float, float], res: float = 0.05, size: float = 12.0,
              unknown: list[tuple] | None = None) -> MapGrid:
    """Occupancy of the segments moved by F (truth -> map), on a grid whose cell lattice is offset by ``phase``;
    ``unknown`` boxes (x0, y0, x1, y1, in the map frame) are marked -1; everything else known free."""
    n = int(size / res)
    origin = (-size / 2 + phase[0], -size / 2 + phase[1], 0.0)
    data = np.zeros((n, n), np.int8)
    for x0, y0, x1, y1 in segs:
        m = max(2, int(math.hypot(x1 - x0, y1 - y0) / (res / 4)))
        u = np.linspace(0, 1, m)
        p = se2_apply(f, np.column_stack([x0 + u * (x1 - x0), y0 + u * (y1 - y0)]))
        c = np.floor((p[:, 0] - origin[0]) / res).astype(int)
        r = np.floor((p[:, 1] - origin[1]) / res).astype(int)
        ok = (r >= 0) & (r < n) & (c >= 0) & (c < n)
        data[r[ok], c[ok]] = OCCUPIED
    for x0, y0, x1, y1 in unknown or []:
        c0, c1 = int((x0 - origin[0]) / res), int((x1 - origin[0]) / res)
        r0, r1 = int((y0 - origin[1]) / res), int((y1 - origin[1]) / res)
        data[max(r0, 0):r1, max(c0, 0):c1] = -1
    return MapGrid(data, origin, res)


TRUE_TRANSFORMS = [(0.0, 0.0, 0.0), (0.01, 0.0, 0.0), (-0.02, 0.0, 0.0), (0.03, 0.0, 0.0), (0.0, 0.02, 0.0),
                   (0.0, -0.03, 0.0), (0.0, 0.0, math.radians(0.5)), (0.0, 0.0, -math.radians(0.5)),
                   (0.013, -0.027, math.radians(0.33)), (-0.0255, 0.0175, -math.radians(0.21))]  # last two off-lattice
PHASES = [(0.0, 0.0), (0.017, 0.031), (0.041, 0.009)]


def contains(s: np.ndarray, g) -> bool:
    if not len(s):
        return False
    half = np.array(FINE) / 2 + 1e-12
    return bool(np.any(np.all(np.abs(s - np.asarray(g)) <= half, axis=1)))


def precision(seed: int = 0) -> dict:
    """The plan's synthetic tests: recovery and S containment (fixing delta), B - A method error between rasters that
    differ only in phase or in a known mask (the method floor per value kind and distance), the degenerate and
    competing-minimum geometries, and the support refusal."""
    rng = np.random.default_rng(seed)
    q = (0.0, 0.0, 0.0)
    rec = []
    floor_rows = {"event": {d: [] for d in FLOOR_DISTANCES_M}, "at_q": {d: [] for d in FLOOR_DISTANCES_M}}
    for geo in ("room", "angled", "racks"):
        segs = segments_for(geo)
        pts = sample_points(segs, rng)
        local = pts[np.hypot(pts[:, 0], pts[:, 1]) <= REGION_M]
        local = thin(local)
        for g in TRUE_TRANSFORMS:
            f = world_transform(q, g)
            als = []
            for ph in PHASES:
                grid = rasterize(segs, f, ph)
                al = align(grid, q, local, 0.0)
                # S provably holds every transform within min + delta, so the true transform is in S whenever its
                # cost exceeds the least cost by at most delta: the needed delta per case
                excess = max(0.0, float(costs(grid, q, local, np.array([g]))[0]) - al["cost_min"])
                rec.append({"geometry": geo, "true": list(g), "phase": list(ph), "found": al["g_opt"],
                            "error": [al["g_opt"][k] - g[k] for k in range(3)], "cost_excess_of_truth": excess})
                als.append(al)
            # mask variant: B has the far wall unknown (one arm saw it, the other not)
            grid_m = rasterize(segs, f, PHASES[1], unknown=[(2.5, -3.0, 6.0, 3.0)])
            ok = support(se2_apply(q, local), q, [rasterize(segs, f, PHASES[0]), grid_m])
            if np.count_nonzero(ok) >= MIN_SUPPORT_POINTS:
                als_m = [align(rasterize(segs, f, PHASES[0]), q, local[ok], 0.0), align(grid_m, q, local[ok], 0.0)]
            else:
                als_m = None
            pairs = [(als[i], als[j]) for i in range(len(als)) for j in range(len(als)) if i < j]
            if als_m:
                pairs.append(tuple(als_m))
            for aa, bb in pairs:
                for dist in FLOOR_DISTANCES_M:
                    for rot in FLOOR_ROTATIONS:
                        t = (dist * math.cos(rot), dist * math.sin(rot), rot)
                        va = along_at(world_transform(q, aa["g_opt"]), t)
                        vb = along_at(world_transform(q, bb["g_opt"]), t)
                        floor_rows["event"][dist].append(abs(vb - va))
                    floor_rows["at_q"][dist].append(abs(bb["g_opt"][0] - aa["g_opt"][0]))
    excess = np.array([r["cost_excess_of_truth"] for r in rec])
    delta = float(np.percentile(excess, 95))
    floor = {k: {f"{d:g}": float(np.percentile(v, 95)) for d, v in rows.items()} for k, rows in floor_rows.items()}
    # degenerate and competing minima
    extra = {}
    for geo in ("parallel", "repeat"):
        segs = segments_for(geo)
        local = thin(sample_points(segs, rng))
        local = local[np.hypot(local[:, 0], local[:, 1]) <= REGION_M]
        al = align(rasterize(segs, q, PHASES[1]), q, local, delta)
        iv = value_interval(q, al, (0.0, 0.0, 0.0))
        extra[geo] = {"S_range": al["S_range"], "touches_box": al["touches_box"], "S_size": al["S_size"],
                      "at_q_interval": [iv["lo"], iv["hi"]], "g_opt": al["g_opt"]}
    few = thin(sample_points([(0.0, 1.0, 0.5, 1.0)], rng))
    extra["support"] = {"few": support_problem(few), "line": support_problem(thin(sample_points([(-3, 1, 3, 1)], rng)))}
    err = np.array([r["error"] for r in rec])
    return {"delta": delta, "containment_share": float(np.mean(excess <= delta)),
            "cost_excess_of_truth": {"median": float(np.median(excess)), "max": float(np.max(excess))},
            "recovery_abs_max": {"a": float(np.max(np.abs(err[:, 0]))), "l": float(np.max(np.abs(err[:, 1]))),
                                 "psi": float(np.max(np.abs(err[:, 2])))},
            "recovery_abs_p95": {"a": float(np.percentile(np.abs(err[:, 0]), 95)),
                                 "l": float(np.percentile(np.abs(err[:, 1]), 95)),
                                 "psi": float(np.percentile(np.abs(err[:, 2]), 95))},
            "floor": floor, "extra": extra, "recovery": rec}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("precision")
    p.add_argument("--json", type=Path, required=True)
    p.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)
    res = precision(args.seed)
    args.json.write_text(json.dumps(res, indent=1, default=str) + "\n")
    print(json.dumps({k: v for k, v in res.items() if k != "recovery"}, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
