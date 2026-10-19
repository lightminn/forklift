# src/forklift_core/perception/bottom_deck_evidence.py
"""Bottom-deck side evidence: is there a bottom board under the fork openings? (plan 2026-10-19 task 1)

From the entry face a pallet and its "grounded" lookalike (no bottom boards, block columns down to the floor) look
the same. Bottom boards run along model x under the columns, so a model +-y face shows them in the gaps between the
block rows. One side observation is judged here:

1. the side plane is refitted from block-height points near the face the camera sees;
2. heights are taken from the local floor in front of it -- the dominant RANSAC plane, refused when steeper than
   ``max_floor_slope_deg`` or when a second surface lies just above or below it -- not from the candidate's
   support or the nominal extrinsic;
3. the block rows are located from the data (a solid / open profile at block height along the face, limited to the
   target's extent), and exactly one arrangement of the spec's rows and gaps must match;
4. the pallet's own height marker -- the open-to-solid edge in the gaps, the top boards' underside at
   deck_bottom + opening + stringer -- must agree with the floor plane;
5. every gap is judged on its own: rays at board height are board (on the plane, measured where they hit), open
   (behind the plane or on the floor) or occluded (in front). A gap that reads board must also be flush with the
   board band under the rows and topped at the bottom-board thickness.

The face reads ``present`` only when every gap is board and passes both checks, ``absent`` when every gap is open
(or, seen squarely, there are no block rows at all: a shelf), and ``unobserved`` otherwise. The plan's section 5
lists what this cannot tell apart (a separate board-height object in each gap, a slab).
"""

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.perception.instance_fit import camera_xyz, pixel_rays
from forklift_core.perception.pallet_model import PalletSpec

VERDICTS = ("present", "absent", "unobserved")


@dataclass(frozen=True)
class BottomDeckTarget:
    centre_xy: tuple            # pallet centre in the base / map frame of the scene
    model_yaw_rad: float        # model +x direction (the hypothesis' axis: x entry -> entry yaw, y entry -> -90 deg)
    spec: PalletSpec
    support_z_m: float = 0.0    # the support height the far estimate claims


@dataclass(frozen=True)
class BottomDeckParams:
    board_band: tuple = (0.4, 0.85)         # x bottom-board thickness: the rays judged per gap
    floor_fraction: float = 0.3             # x thickness: measured points lower than this are floor
    block_band: tuple = (0.2, 0.8)          # x opening height, above the bottom board: the row profile
    side_window_m: float = 0.05             # band points within this of the nominal face refit the plane
    side_min_points: int = 30
    plane_tol_m: float = 0.015              # on the plane
    behind_m: float = 0.03                  # behind the plane by more: open
    front_m: float = 0.03                   # in front by more: occluded
    floor_offset_m: tuple = (0.03, 0.45)    # floor points: this far in front of the face
    floor_max_z_m: float = 0.06
    floor_min_points: int = 50
    ransac_iters: int = 200
    ransac_tol_m: float = 0.004
    ransac_min_support: int = 30
    max_floor_slope_deg: float = 1.5        # both the RANSAC samples and the refitted plane
    second_surface_m: tuple = (0.008, 0.06)
    second_surface_fraction: float = 0.1
    bin_m: float = 0.01
    profile_pad_m: float = 0.15             # the profile spans the target's depth +- this
    row_length: tuple = (0.8, 1.5)          # x row depth (+ row_pad_m on the upper end)
    row_pad_m: float = 0.03
    gap_length: tuple = (0.4, 1.3)          # x nominal gap (+- gap_pad_m)
    gap_pad_m: float = 0.01
    extent: tuple = (0.85, 1.25)            # x overall depth
    centre_tol_m: float = 0.12
    no_rows_max_theta_face_deg: float = 50.0
    no_rows_solid_slack_m: float = 0.05
    gap_margin_m: float = 0.012             # trimmed off both ends of a located gap
    marker_window_m: tuple = (-0.04, 0.03)
    marker_tol_m: float = 0.004
    marker_min_points: int = 20
    marker_min_each: int = 5
    min_gap_rays: int = 8
    max_occluded_fraction: float = 0.3
    present_fraction: float = 0.6
    absent_fraction: float = 0.3
    flush_tol_m: float = 0.016
    top_window_m: tuple = (-0.003, 0.008)   # board top minus thickness (reads high: mixed edge pixels)
    top_min_points: int = 8
    top_cluster_m: float = 0.004
    top_cluster_fraction: float = 0.1
    top_cluster_min: int = 4
    max_support_m: float = 0.03
    seed: int = 0


@dataclass(frozen=True)
class GapEvidence:
    state: str                  # board | open | mixed | unseen
    board: int
    open: int
    occluded: int
    fraction: float = math.nan
    flush_m: float = math.nan   # board band in the gap minus the band under the rows
    top_m: float = math.nan     # board top above the floor
    n_top: int = 0


@dataclass(frozen=True)
class FloorFit:
    coef: tuple                 # z = a + b s + c off
    support: int
    ambiguous: bool
    tilted: bool = False        # the refitted plane is steeper than the limit (its samples were not)


@dataclass(frozen=True)
class BottomDeckEvidence:
    verdict: str
    reason: str
    face: int = 0               # +1 / -1: the model y face judged
    theta_face_deg: float = math.nan
    range_m: float = math.nan
    gaps: tuple = ()
    floor: tuple = ()
    plane: tuple = ()           # the refitted face: (point x, point y, normal x, normal y)
    marker_m: float = math.nan
    profile: str = ""


def nominal_gaps(spec: PalletSpec) -> tuple:
    """Gap widths between consecutive block rows along model x."""
    rows = sorted(spec.row_centres_x_m)
    return tuple((b - a) - spec.row_depth_m for a, b in zip(rows, rows[1:]))


def dominant_floor(s, off, z, params: BottomDeckParams):
    """RANSAC dominant plane z = a + b s + c off over floor candidates; None when no sample plane lies within the
    slope limit, ``tilted`` when the plane refitted on its support does not."""
    s, off, z = (np.asarray(v, float) for v in (s, off, z))
    if len(z) < params.floor_min_points:
        return None
    a_mat = np.column_stack([np.ones(len(z)), s, off])
    tan_max = math.tan(math.radians(params.max_floor_slope_deg))
    rng = np.random.default_rng(params.seed)
    best, best_n = None, -1
    for _ in range(params.ransac_iters):
        idx = rng.choice(len(z), 3, replace=False)
        try:
            cf = np.linalg.solve(a_mat[idx], z[idx])
        except np.linalg.LinAlgError:
            continue
        if math.hypot(cf[1], cf[2]) > tan_max:
            continue
        n_in = int((np.abs(a_mat @ cf - z) <= params.ransac_tol_m).sum())
        if n_in > best_n:
            best, best_n = cf, n_in
    if best is None or best_n < params.ransac_min_support:
        return None
    inl = np.abs(a_mat @ best - z) <= params.ransac_tol_m
    coef = np.linalg.lstsq(a_mat[inl], z[inl], rcond=None)[0]
    tilted = math.hypot(coef[1], coef[2]) > tan_max   # the plane actually used, not only its samples
    resid = np.abs(z - a_mat @ coef)
    lo, hi = params.second_surface_m
    ambiguous = bool(((resid >= lo) & (resid <= hi)).sum() > params.second_surface_fraction * len(z))
    return FloorFit(tuple(float(v) for v in coef), int(inl.sum()), ambiguous, tilted)


def stand_angle_deg(camera_xy, target: BottomDeckTarget, face: int) -> float:
    """theta_e: the horizontal angle between camera - centre and the outward normal of model face ``face``."""
    c, s = math.cos(target.model_yaw_rad), math.sin(target.model_yaw_rad)
    normal = np.array([-s, c]) * (1 if face > 0 else -1)
    v = np.asarray(camera_xy, float)[:2] - np.asarray(target.centre_xy, float)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(normal @ v / np.linalg.norm(v))))))


ARRIVAL = {"x": (25.0, 45.0), "y": (0.0, 35.0)}   # theta_e ranges per hypothesis axis
ARRIVAL_RANGE_M = (1.1, 1.45)


def arrival_ok(scene, target: BottomDeckTarget, *, face: int, axis: str) -> bool:
    """The stand actually reached: camera-centre range and theta_e on the leg's target face."""
    cam = camera_xyz(scene)[:2]
    rng_m = float(np.linalg.norm(cam - np.asarray(target.centre_xy, float)))
    lo, hi = ARRIVAL[axis]
    return ARRIVAL_RANGE_M[0] <= rng_m <= ARRIVAL_RANGE_M[1] and lo <= stand_angle_deg(cam, target, face) <= hi


def _runs(profile):
    runs = []
    for i, st in enumerate(profile):
        if runs and runs[-1][0] == st:
            runs[-1][2] = i + 1
        else:
            runs.append([st, i, i + 1])
    return runs


def _top_height(z, params: BottomDeckParams) -> float:
    """The highest height with a dense cluster just below it (stray points above it do not count)."""
    zf = np.sort(z)
    need = max(params.top_cluster_min, int(params.top_cluster_fraction * len(zf)))
    cnt = np.searchsorted(zf, zf, side="right") - np.searchsorted(zf, zf - params.top_cluster_m, side="left")
    ok = cnt >= need
    return float(zf[ok].max()) if ok.any() else math.nan


def evaluate_side(scene, target: BottomDeckTarget, *, face: int | None = None,
                  params: BottomDeckParams = BottomDeckParams()) -> BottomDeckEvidence:
    """Judge the model y face the camera sees (``face`` given: it must be that one)."""
    spec = target.spec
    support = float(target.support_z_m)
    if not math.isfinite(support) or abs(support) > params.max_support_m:   # NaN must not read as "on the floor"
        return BottomDeckEvidence("unobserved", "not_on_floor")
    rays = pixel_rays(scene)
    cam = camera_xyz(scene)
    depth = np.asarray(scene.depth_m, float)
    valid = np.isfinite(depth) & (depth > 0)
    pts = cam + np.where(valid, depth, 0.0)[..., None] * rays
    c, s = math.cos(target.model_yaw_rad), math.sin(target.model_yaw_rad)
    ex, ey = np.array([c, s, 0.0]), np.array([-s, c, 0.0])
    centre = np.array([float(target.centre_xy[0]), float(target.centre_xy[1]), 0.0])
    seen = 1 if (cam - centre) @ ey >= 0 else -1
    range_m = float(np.linalg.norm((cam - centre)[:2]))
    if face is not None and face != seen:
        return BottomDeckEvidence("unobserved", "wrong_face", face=seen, range_m=range_m)
    n = ey * seen
    half_y, half_x = spec.overall_width_m / 2, spec.overall_depth_m / 2
    p0 = centre + n * half_y
    db, bh = spec.deck_bottom_m, spec.opening_height_m
    blk = (db + params.block_band[0] * bh, db + params.block_band[1] * bh)

    def out(verdict, reason, **kw):
        return BottomDeckEvidence(verdict, reason, face=seen, range_m=range_m, **kw)

    # 1. the side plane from block-height points near the nominal face
    rel = pts - p0
    band = valid & (np.abs(rel @ n) <= params.side_window_m) & (pts[..., 2] >= blk[0]) & (pts[..., 2] <= blk[1]) \
        & (np.abs(rel @ ex) <= half_x + params.profile_pad_m)
    if band.sum() < params.side_min_points:
        return out("unobserved", "no_side_points")
    xy = pts[band][:, :2]
    m = np.median(xy, axis=0)
    _, vec = np.linalg.eigh((xy - m).T @ (xy - m))
    line = vec[:, 1]
    nn = np.array([-line[1], line[0], 0.0])
    if nn @ n < 0:
        nn = -nn
    r = (xy - m) @ nn[:2]
    inl = np.abs(r - np.median(r)) <= params.plane_tol_m
    m = m + nn[:2] * np.median(r[inl])
    n, p0 = nn, np.array([m[0], m[1], 0.0])
    ex = np.array([line[0], line[1], 0.0]) * (1.0 if line @ np.array([c, s]) >= 0 else -1.0)
    origin_s = float((centre - p0) @ ex)
    to_cam = (cam - p0)[:2]
    theta_face = math.degrees(math.acos(max(-1.0, min(1.0, float(n[:2] @ to_cam / np.linalg.norm(to_cam))))))

    denom = rays @ n
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((p0 - cam) @ n) / denom
    hit = cam + t[..., None] * rays
    s_hit = (hit - p0) @ ex
    off = (pts - p0) @ n
    s_meas = (pts - p0) @ ex
    facing = (t > 0) & (denom < 0) & valid

    # 2. the local floor in front of the face
    near_floor = valid & (off > params.floor_offset_m[0]) & (off < params.floor_offset_m[1]) \
        & (np.abs(s_meas - origin_s) <= half_x + params.profile_pad_m + 0.05) & (pts[..., 2] < params.floor_max_z_m)
    if near_floor.sum() < params.floor_min_points:
        return out("unobserved", "no_floor", theta_face_deg=theta_face)
    floor = dominant_floor(s_meas[near_floor], off[near_floor], pts[..., 2][near_floor], params)
    plane = (float(p0[0]), float(p0[1]), float(n[0]), float(n[1]))
    if floor is None:
        return out("unobserved", "no_floor", theta_face_deg=theta_face, plane=plane)
    if floor.tilted:
        return out("unobserved", "floor_tilted", theta_face_deg=theta_face, plane=plane, floor=floor.coef)
    if floor.ambiguous:
        return out("unobserved", "floor_ambiguous", theta_face_deg=theta_face, plane=plane, floor=floor.coef)
    fa, fb, fc = floor.coef
    z_hit = hit[..., 2] - (fa + fb * s_hit)
    zm = pts[..., 2] - (fa + fb * s_meas + fc * off)
    on = facing & (np.abs(off) <= params.plane_tol_m)
    behind = facing & (off < -params.behind_m)
    front = facing & (off > params.front_m)
    common = dict(theta_face_deg=theta_face, floor=floor.coef, plane=plane)

    # 3. the block rows along the face, over the target's extent only
    step = params.bin_m
    lo = origin_s - half_x - params.profile_pad_m
    edges = np.arange(lo, origin_s + half_x + params.profile_pad_m + step, step)
    zb = facing & (z_hit >= blk[0]) & (z_hit <= blk[1])
    prof = []
    for a, b in zip(edges[:-1], edges[1:]):
        sel = zb & (s_hit >= a) & (s_hit < b)
        k_on, k_be, k_fr = int((sel & on).sum()), int((sel & behind).sum()), int((sel & front).sum())
        tot = k_on + k_be + k_fr
        prof.append("?" if tot < 3 or k_fr > 0.3 * tot else "S" if k_on >= 0.7 * tot else "O" if k_be >= 0.7 * tot
                    else "?")
    profile = "".join(prof)
    solids = [r for r in _runs(prof) if r[0] == "S" and (r[2] - r[1]) >= 3]
    rd = spec.row_depth_m
    rows = [r for r in solids
            if params.row_length[0] * rd <= (r[2] - r[1]) * step <= params.row_length[1] * rd + params.row_pad_m]
    gaps_nom = nominal_gaps(spec)
    k = len(gaps_nom) + 1

    def gap_ok(a, b, width):
        span = (b[1] - a[2]) * step
        return (params.gap_length[0] * width - params.gap_pad_m <= span <= params.gap_length[1] * width
                + params.gap_pad_m) and "O" in prof[a[2]:b[1]]

    matches = []
    for i in range(len(rows) - k + 1):
        seq = rows[i:i + k]
        extent = (seq[-1][2] - seq[0][1]) * step
        mid = (edges[seq[0][1]] + edges[seq[-1][2]]) / 2
        if all(gap_ok(a, b, w) for a, b, w in zip(seq, seq[1:], gaps_nom)) \
                and params.extent[0] * spec.overall_depth_m <= extent <= params.extent[1] * spec.overall_depth_m \
                and abs(mid - origin_s) <= params.centre_tol_m:
            matches.append(seq)
    if len(matches) > 1:
        return out("unobserved", "ambiguous_rows", profile=profile, **common)
    if not matches:
        solid_total = max(((r[2] - r[1]) * step for r in solids), default=0.0)
        if solid_total >= spec.overall_depth_m - params.no_rows_solid_slack_m \
                and theta_face <= params.no_rows_max_theta_face_deg:
            return out("absent", "no_block_rows", profile=profile, **common)
        return out("unobserved", "rows_not_located", profile=profile, **common)
    seq = matches[0]
    gaps = [(edges[a[2]], edges[b[1]]) for a, b in zip(seq, seq[1:])]
    gm = params.gap_margin_m

    # 4. the pallet's own height marker in the gaps must agree with the floor plane
    edge_nom = db + bh + spec.stringer_m
    in_gap_hit = np.zeros(t.shape, bool)
    for a, b in gaps:
        in_gap_hit |= (s_hit >= a + gm) & (s_hit <= b - gm)
    win = facing & in_gap_hit & (z_hit >= edge_nom + params.marker_window_m[0]) \
        & (z_hit <= edge_nom + params.marker_window_m[1])
    zz, is_on = z_hit[win], on[win]
    keep = is_on | behind[win]
    zz, is_on = zz[keep], is_on[keep]
    if len(zz) < params.marker_min_points or is_on.sum() < params.marker_min_each \
            or (~is_on).sum() < params.marker_min_each:
        return out("unobserved", "marker_unseen", profile=profile, **common)
    cands = np.sort(zz)
    score = [((zz < cv) & ~is_on).sum() + ((zz >= cv) & is_on).sum() for cv in cands]
    marker = float(cands[int(np.argmax(score))]) - edge_nom
    common["marker_m"] = marker
    if abs(marker) > params.marker_tol_m:
        return out("unobserved", "floor_marker_disagree", profile=profile, **common)

    # 5. every gap on its own
    tb = facing & (z_hit >= params.board_band[0] * db) & (z_hit <= params.board_band[1] * db)
    meas_in = np.zeros(t.shape, bool)
    for a, b in gaps:
        meas_in |= (s_meas >= a + gm) & (s_meas <= b - gm)
    floor_pt = facing & (zm < params.floor_fraction * db)
    board_pt = on & meas_in & (zm >= params.floor_fraction * db) & (np.abs(zm - z_hit) <= max(0.004, 0.35 * db))
    in_rows = np.zeros(t.shape, bool)
    for r in seq:
        in_rows |= (s_meas >= edges[r[1]] + gm) & (s_meas <= edges[r[2]] - gm)
    ref_sel = facing & in_rows & tb & (np.abs(off) <= params.plane_tol_m)
    off_ref = float(np.median(off[ref_sel])) if ref_sel.sum() >= 15 else math.nan
    per = []
    for a, b in gaps:
        ing = (s_hit >= a + gm) & (s_hit <= b - gm)
        k_on = int((tb & ing & board_pt).sum())
        k_be = int((tb & ing & (behind | floor_pt) & ~board_pt).sum())
        k_fr = int((tb & ing & front & ~floor_pt).sum())
        if k_on + k_be < params.min_gap_rays or k_fr > params.max_occluded_fraction * (k_on + k_be + k_fr):
            per.append(GapEvidence("unseen", k_on, k_be, k_fr))
            continue
        frac = k_on / (k_on + k_be)
        state = "board" if frac >= params.present_fraction else "open" if frac <= params.absent_fraction else "mixed"
        if state != "board":
            per.append(GapEvidence(state, k_on, k_be, k_fr, frac))
            continue
        flush = float(np.median(off[tb & ing & board_pt])) - off_ref
        q = (b - a) / 4      # the middle half: the bin-quantised gap ends let the next block's face in
        mid = (s_meas >= (a + b) / 2 - q) & (s_meas <= (a + b) / 2 + q)
        top_sel = facing & mid & (np.abs(off - off_ref) <= params.plane_tol_m) & (zm >= params.floor_fraction * db) \
            & (zm <= db + 0.02)
        n_top = int(top_sel.sum())
        top = _top_height(zm[top_sel], params) if n_top >= params.top_min_points else math.nan
        per.append(GapEvidence(state, k_on, k_be, k_fr, frac, flush, top, n_top))
    gap_ev = tuple(per)
    states = [gp.state for gp in per]
    if "unseen" in states:
        return out("unobserved", "board_band_unseen", gaps=gap_ev, profile=profile, **common)
    if all(st == "open" for st in states):
        return out("absent", "open_gaps", gaps=gap_ev, profile=profile, **common)
    if not all(st == "board" for st in states):
        return out("unobserved", "gaps_disagree", gaps=gap_ev, profile=profile, **common)
    if any(not math.isfinite(gp.flush_m) or abs(gp.flush_m) > params.flush_tol_m for gp in per):
        return out("unobserved", "board_not_flush", gaps=gap_ev, profile=profile, **common)
    if any(not math.isfinite(gp.top_m) for gp in per):
        return out("unobserved", "board_edge_unseen", gaps=gap_ev, profile=profile, **common)
    lo_t, hi_t = params.top_window_m
    if any(not (db + lo_t <= gp.top_m <= db + hi_t) for gp in per):
        return out("unobserved", "board_height_mismatch", gaps=gap_ev, profile=profile, **common)
    return out("present", "board_in_gaps", gaps=gap_ev, profile=profile, **common)
