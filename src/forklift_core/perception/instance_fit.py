# src/forklift_core/perception/instance_fit.py
"""Instance geometric fit (design §7): one proposal mask -> one PocketCandidate.

Evidence ownership (§7.2): members are fitted from the proposal's own pixels
(eroded), and a member edge counts only when the face-side rays belong to the
proposal; openings are judged from every raw-depth ray that meets the face
plane inside the hypothesised opening. A member that is not seen costs nothing
(damage, occlusion); material where a hypothesis has an opening does. Rays
never add score, so a hypothesis cannot earn score over empty background.

Faces are vertical planes in the base frame. Face coordinates: u along the
left axis (insertion direction turned +90 deg), h = base z. FitParams values
were chosen on dev scenes (docs/plans/2026-10-16-recognition-stage2a-instance-fit.md).
"""

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.perception.fork_fit import (
    M_REQ_PROVISIONAL_M, PAYLOAD_LIMIT_KG, REASON_CODES, SUPPORT_LATERAL_MARGIN_M, SUPPORT_LONGITUDINAL_MARGIN_M, fork_fit,
)
from forklift_core.perception.instance_proposals import InstanceProposal
from forklift_core.perception.pallet_geometry import target_insertion_depth_m
from forklift_core.perception.pallet_model import FaceLayout, ObservedPalletShape, PalletSpec, face_layout, pair_margin, spec_boxes
from forklift_core.perception.pocket_candidate import PocketCandidate, PocketEstimate
from forklift_core.perception.scene_dataset import SceneInput

BEHIND, FACE, FRONT, UNOBSERVED = 0, 1, 2, 3
HEIGHT_REASONS = ("lift_alignment_needed",)  # fork-fit reasons that read the support height
# The far-range path never permits a pick: the near-range re-observation (§9.1) verifies these first.
# A known template fills them, but a non-standard pallet can match a template's face and differ in
# its lane height or top (S7 dev 2026-10-16: T11-like lanes 3-5 mm lower, EPAL-like flat tops).
FAR_PENDING = ("depth", "lane_bounds", "top_layout")
# A non-standard pallet with openings on two sides shows no far-range evidence of which face is the
# entry face (Codex review A round 3): the near range checks it on the top-board direction.
PARAMETRIC_PENDING = FAR_PENDING + ("entry_axis",)
# A known shape's entry face does not tell a pallet from its grounded lookalike (no bottom boards): the side
# evidence (plan 2026-10-19 task 1) does, and the insert permission checks it (task 3).
KNOWN_FAR_PENDING = FAR_PENDING + ("bottom_deck",)


@dataclass(frozen=True)
class FitParams:
    erode_px: int = 3                 # mask edge pixels carry flying depth and a 2 px RGB shift
    min_points: int = 60
    ransac_iters: int = 300
    ransac_tol_m: float = 0.03
    max_faces: int = 2
    min_face_span_m: float = 0.15
    top_exclude_m: float = 0.015      # planes are fitted without the top surface
    irls_iters: int = 15
    exterior_margin_m: float = 0.03
    exterior_max_fraction: float = 0.10
    corner_tol_rad: float = math.radians(10.0)  # a second face must meet the first at 90 deg +- this
    max_face_scale_m: float = 0.01    # a face's robust scale may not exceed max(this, ray_margin_k * reach^2 / 2)
    bin_m: float = 0.005
    support_window_m: float = 0.03
    ray_margin_m: float = 0.03        # ray class margin: max(ray_margin_m, ray_margin_k * t^2)
    ray_margin_k: float = 0.015
    hole_point_weight: float = 1.0
    outside_weight: float = 0.5
    face_ray_weight: float = 0.5
    seen_through_weight: float = 0.25
    edge_window_m: float = 0.015
    edge_min_hits: int = 3
    min_score: float = 0.3
    shape_outside_max: float = 0.08    # face points outside the shape's rectangle (dev 2026-10-16)
    shape_seen_through_max: float = 0.12  # rays passing through the shape's material (dev 2026-10-16)
    ambiguity_margin: float = 0.05
    separate_placement_m: float = 0.05  # another local maximum this far along the face is a different placement
    peak_window_m: float = 0.02         # a local maximum beats every placement within this distance
    equivalence_m: float = 0.020
    equivalence_yaw_rad: float = math.radians(2.0)
    parametric: bool = True              # read non-standard faces when no known shape explains them (§7.1)
    parametric_row_fill: float = 0.65    # rows of the top band are at least this full (boards leave gaps)
    parametric_member_fill: float = 0.5  # a member column is at least this full below the lane ceiling
    parametric_top_skip_m: float = 0.03  # board ends (S7 top boards 12-25 mm, along x) and a ragged edge above the deck run
    min_opening_m: float = 0.05          # narrower gaps are member splits, not fork lanes (a split block is 5-15 mm)
    min_lane_ceiling_m: float = 0.03
    end_window_m: tuple = (0.005, 0.03)  # rays this far beyond an end decide whether the end is physical
    nonstandard_depth_m: tuple = (0.50, 1.20)  # depth range of non-standard pallets (design §5.1): unobserved here
    # The far range stops a non-standard pick for fork spacing only below M_REQ minus this; in between
    # the lane bounds wait for the near range, which applies M_REQ itself. An empirical dev margin
    # (provisional noise, 2026-10-16: far margins read up to 42.5 mm low at the high level, pickable
    # S7 pallets sit 4-6 mm above M_REQ), not a measured error bound -- revisit with the D435i noise.
    parametric_spacing_slack_m: float = 0.030
    seed: int = 0


@dataclass(frozen=True)
class FacePlane:
    centre_xy: np.ndarray
    normal_xy: np.ndarray  # unit, pointing toward the camera
    scale_m: float
    index: np.ndarray      # indices of the proposal points on this face

    @property
    def insertion_xy(self):
        return -self.normal_xy

    @property
    def left_xy(self):
        a = self.insertion_xy
        return np.array([-a[1], a[0]])


def erode(mask, radius: int):
    out = np.asarray(mask, bool).copy()
    for _ in range(radius):
        m = out.copy()
        m[1:, :] &= out[:-1, :]
        m[:-1, :] &= out[1:, :]
        m[:, 1:] &= out[:, :-1]
        m[:, :-1] &= out[:, 1:]
        m[0, :] = m[-1, :] = False
        m[:, 0] = m[:, -1] = False
        out = m
    return out


def pixel_rays(scene: SceneInput):
    """Base-frame ray per pixel with unit optical z: point = camera + axial_depth * ray."""
    k = scene.intrinsics
    v, u = np.indices((k.height, k.width))
    optical = np.stack(((u - k.cx) / k.fx, (v - k.cy) / k.fy, np.ones(u.shape)), axis=-1)
    return optical @ np.asarray(scene.base_from_optical.rotation, float).T


def camera_xyz(scene: SceneInput):
    return np.asarray(scene.base_from_optical.translation_m, float)


def proposal_points(scene: SceneInput, rays, mask, erode_px: int):
    kept = erode(mask, erode_px) & np.isfinite(scene.depth_m)
    v, u = np.nonzero(kept)
    return camera_xyz(scene) + scene.depth_m[v, u, None] * rays[v, u], np.column_stack((v, u))


def _irls_line(points_xy, centre, normal, camera_xy, iters):
    """Huber-weighted total least squares line, started at the RANSAC line (centre, normal).

    The face is the object's outer layer: lane walls and a neighbouring face's corner lie
    only behind it. Points more than 2.5 scales behind the line (away from the camera) get
    no weight, so they cannot pull the line inward; the scale is a low quantile of |r|
    (|r| < 0.3186 sigma for 25 % of a Gaussian), which stays on the face layer.
    """
    if (camera_xy - centre) @ normal < 0:
        normal = -normal
    scale = 0.002
    for _ in range(iters):
        r = (points_xy - centre) @ normal          # > 0: toward the camera
        scale = max(float(np.percentile(np.abs(r), 25)) / 0.3186, 0.002)
        a = np.abs(r) / (1.345 * scale)
        w = np.where(a <= 1.0, 1.0, 1.0 / np.maximum(a, 1e-12))
        w[(np.abs(r) > 4.0 * scale) | (r < -2.5 * scale)] = 0.0
        if w.sum() <= 0:
            break
        centre = (points_xy * w[:, None]).sum(0) / w.sum()
        d = points_xy - centre
        n = np.linalg.eigh((d * w[:, None]).T @ d / w.sum())[1][:, 0]
        normal = n if n @ normal >= 0 else -n
    r = (points_xy - centre) @ normal
    kept = np.abs(r[(r >= -2.5 * scale) & (r <= 4.0 * scale)])
    return centre, normal, max(float(np.percentile(kept, 25)) / 0.3186 if len(kept) else scale, 0.002)


def _snap_to_layer(points_xy, centre, normal, tol):
    """Move a line onto the densest thin layer of points near it and re-aim it along that layer.

    Twice: histogram the offsets within +-tol, shift to the densest bin, then refit the
    direction from the points within a thin band of it (a few bins wide).
    """
    step = max(0.002, tol / 15)
    for _ in range(2):
        r = (points_xy - centre) @ normal
        r = r[np.abs(r) < tol]
        if len(r) < 2:
            break
        counts, edges = np.histogram(r, bins=np.arange(-tol, tol + step, step))
        k = int(np.argmax(np.convolve(counts, np.ones(3), mode="same")))
        centre = centre + normal * float((edges[k] + edges[k + 1]) / 2)
        band = points_xy[np.abs((points_xy - centre) @ normal) < 1.5 * step]
        if len(band) < 2:
            break
        centre = band.mean(0)
        d = band - centre
        n = np.linalg.eigh(d.T @ d)[1][:, 0]
        normal = n if n @ normal >= 0 else -n
    return centre, normal


def _reach(camera, centre_xy) -> float:
    return float(np.hypot(*(camera[:2] - centre_xy)))


def _margin(params, reach) -> float:
    return max(params.ray_margin_m, params.ray_margin_k * reach * reach)


def fit_faces(points, camera, params: FitParams, rng) -> list[FacePlane]:
    """Up to max_faces exterior vertical planes, largest first.

    RANSAC on two-point vertical lines and a Huber refit, both on the points below the
    top surface (seen from above, the top surface would otherwise give the largest
    line). Each face then takes every remaining point within its band, top included.
    A plane whose robust scale exceeds the noise allowance is a loose bundle of points;
    a plane with more than exterior_max_fraction of the object's points on the camera
    side of it is an interior plane (block sides seen through a lane); a second plane that
    does not meet the first at a right angle is neither of a box's two visible faces. All
    three are dropped.
    """
    faces = []
    if len(points) < params.min_points:
        return faces
    zmax = float(points[:, 2].max())
    body = points[:, 2] < zmax - params.top_exclude_m
    # The inlier band widens with the noise at the object's range (sigma grows with range^2).
    reach0 = float(np.median(np.hypot(*(points[:, :2] - camera[:2]).T)))
    tol = max(params.ransac_tol_m, 0.5 * params.ray_margin_k * reach0 * reach0)
    remaining = np.arange(len(points))
    for _ in range(params.max_faces + 2):
        if len(faces) == params.max_faces:
            break
        fit_from = remaining[body[remaining]]
        if len(fit_from) < params.min_points:
            break
        sub = points[fit_from, :2]
        best = None
        for _ in range(params.ransac_iters):
            i, j = rng.choice(len(sub), 2, replace=False)
            d = sub[j] - sub[i]
            length = float(np.hypot(*d))
            if length < 0.05:
                continue
            n = np.array([-d[1], d[0]]) / length
            inliers = np.abs((sub - sub[i]) @ n) < tol
            count = int(inliers.sum())
            if best is None or count > best[0]:
                best = (count, inliers, sub[i], n)
        if best is None or best[0] < params.min_points:
            break
        _, inliers, c0, n0 = best
        # The line with the most points in the band can sit inside the face, slightly tilted,
        # covering the face and the first centimetres of the lane walls behind it.
        c0, n0 = _snap_to_layer(sub, c0, n0, tol)
        near = np.abs((sub - c0) @ n0) < tol
        centre, normal, scale = _irls_line(sub[near], c0, n0, camera[:2], params.irls_iters)
        on = np.abs((points[remaining, :2] - centre) @ normal) < max(tol, 2.5 * scale)
        index = remaining[on]
        if len(index) < params.min_points:
            break
        remaining = remaining[~on]
        if (camera[:2] - centre) @ normal < 0:
            normal = -normal
        reach = _reach(camera, centre)
        if scale > max(params.max_face_scale_m, 0.5 * params.ray_margin_k * reach * reach):
            continue
        margin = max(params.exterior_margin_m, 3 * scale, params.ray_margin_k * reach * reach)
        if ((points[:, :2] - centre) @ normal > margin).mean() > params.exterior_max_fraction:
            continue
        # A pallet is a box: a second visible face meets the first at a right angle.
        if faces and abs(abs(float(normal @ faces[0].normal_xy)) - 0.0) > math.sin(params.corner_tol_rad):
            continue
        faces.append(FacePlane(centre, normal, scale, index))
    return faces


def face_coords(face: FacePlane, points):
    d = points[:, :2] - face.centre_xy
    return d @ face.left_xy, points[:, 2]


def ray_samples(scene: SceneInput, rays, face: FacePlane, u_range, h_range, params: FitParams, mask=None):
    """(u, h, class, owned) of every pixel whose ray meets the face plane inside the window.

    The class compares the measured axial depth d with the plane crossing t:
    BEHIND if d > t + m, FRONT if d < t - m, FACE otherwise, UNOBSERVED if d is NaN,
    with m = max(ray_margin_m, ray_margin_k * t^2) (the noise grows with t^2).
    owned marks the pixels of the proposal mask (all False without a mask).
    """
    cam = camera_xyz(scene)
    n = face.normal_xy
    den = rays[..., :2] @ n
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((face.centre_xy - cam[:2]) @ n) / den
    hit = np.isfinite(t) & (t > 0)
    p = cam + np.where(hit, t, 0.0)[..., None] * rays
    u = (p[..., :2] - face.centre_xy) @ face.left_xy
    h = p[..., 2]
    inside = hit & (u >= u_range[0]) & (u <= u_range[1]) & (h >= h_range[0]) & (h <= h_range[1])
    d, tt = scene.depth_m[inside], t[inside]
    margin = np.maximum(params.ray_margin_m, params.ray_margin_k * tt * tt)
    cls = np.full(len(tt), FACE, np.int8)
    cls[d > tt + margin] = BEHIND
    cls[d < tt - margin] = FRONT
    cls[~np.isfinite(d)] = UNOBSERVED
    owned = np.zeros(len(tt), bool) if mask is None else np.asarray(mask, bool)[inside]
    return u[inside], h[inside], cls, owned


def face_raster(spec: PalletSpec, axis: str, bin_m: float, *, mirror: bool = False):
    """Occupancy of an entry face seen along its axis (every box projected through the depth).

    Rows are h from 0 at the support surface; columns are u from -W/2 (right) to +W/2
    (left). Entering an x face, left is +y; entering a y face, left is -x. Lanes are
    through-holes, so the projection is what the face shows.
    """
    layout = face_layout(spec, axis)
    width = layout.width_m
    nu, nh = int(round(width / bin_m)), int(round(spec.overall_height_m / bin_m))
    uc = -width / 2 + (np.arange(nu) + 0.5) * bin_m
    hc = (np.arange(nh) + 0.5) * bin_m
    occ = np.zeros((nh, nu), bool)
    for box in spec_boxes(spec):
        (cx, cy, cz), (sx, sy, sz) = box.centre_m, box.size_m
        lo, hi = (cy - sy / 2, cy + sy / 2) if axis == "x" else (-cx - sx / 2, -cx + sx / 2)
        rows = (hc >= cz - sz / 2) & (hc <= cz + sz / 2)
        cols = (uc >= lo) & (uc <= hi)
        occ[np.ix_(rows, cols)] = True
    return occ[:, ::-1] if mirror else occ


def damageable_rows(spec: PalletSpec, bin_m: float):
    """Raster rows that allowed damage may empty: the top-deck band (design §5.2 removes 1-2 top
    boards, edge boards included, or breaks one off). Seen-through rays there are not evidence of
    a different shape; the blocks, stringers and bottom boards stay (chips and splits are small)."""
    nh = int(round(spec.overall_height_m / bin_m))
    hc = (np.arange(nh) + 0.5) * bin_m
    return hc >= spec.overall_height_m - spec.top_thickness_m


def _hist(u, h, u0, h0, nu, nh, bin_m):
    iu = np.floor((u - u0) / bin_m).astype(int)
    ih = np.floor((h - h0) / bin_m).astype(int)
    ok = (iu >= 0) & (iu < nu) & (ih >= 0) & (ih < nh)
    out = np.zeros((nh, nu))
    np.add.at(out, (ih[ok], iu[ok]), 1.0)
    return out


def _correlate_rows(hist, kernel, row_shift):
    total = None
    for i in range(kernel.shape[0]):
        row = np.correlate(hist[row_shift + i], kernel[i], mode="valid")
        total = row if total is None else total + row
    return total


@dataclass(frozen=True)
class Placement2D:
    score: float
    u_centre: float
    support_z: float
    runner_up_score: float = -math.inf   # best separate local maximum of the same raster
    runner_up_u: float = math.nan
    close_maxima: tuple = ()             # (score, u_centre, support_z) of every separate local maximum
                                         # within ambiguity_margin of this placement's score


def best_placement(occ, pts_u, pts_h, rays, params: FitParams, h_lo: float, n_shift: int,
                   damageable=None) -> Placement2D:
    """Exhaustive search of a face raster over u and support height.

    Per mask point: +1 on material, -hole_point_weight in a hole, -outside_weight outside
    the raster. Per raw ray: -face_ray_weight if it stops on the face inside a hole,
    -seen_through_weight if it passes behind the face where the raster has material outside the
    damageable rows (a missing or broken top board is allowed damage, §5.2; §7.4 does not
    penalise it).
    Rays never add. The sum is divided by the number of face points. The support height
    runs over h_lo + s * bin_m for s < n_shift.
    """
    b = params.bin_m
    nh_t, nu_t = occ.shape
    u0 = float(pts_u.min()) - nu_t * b
    nu = int(math.ceil((float(pts_u.max()) - float(pts_u.min())) / b)) + 2 * nu_t + 1
    nh = nh_t + n_shift + 8
    ru, rh, rc = rays[0], rays[1], rays[2]
    pm = _hist(pts_u, pts_h, u0, h_lo, nu, nh, b)
    rb = _hist(ru[rc == BEHIND], rh[rc == BEHIND], u0, h_lo, nu, nh, b)
    rf = _hist(ru[rc == FACE], rh[rc == FACE], u0, h_lo, nu, nh, b)
    k_pts = np.where(occ, 1.0 + params.outside_weight, params.outside_weight - params.hole_point_weight)
    solid = occ if damageable is None else occ & ~np.asarray(damageable)[:, None]
    k_b = np.where(solid, -params.seen_through_weight, 0.0)
    k_f = np.where(occ, 0.0, -params.face_ray_weight)
    n_pts = len(pts_u)
    grid = np.array([(_correlate_rows(pm, k_pts, s) + _correlate_rows(rb, k_b, s) + _correlate_rows(rf, k_f, s)
                      - params.outside_weight * n_pts) / n_pts for s in range(n_shift)])
    profile = grid.max(axis=0)                     # best support height for each lateral placement
    j = int(np.argmax(profile))
    s = int(np.argmax(grid[:, j]))
    # The same raster can fit nearly as well one member pitch away (Codex review A 2026-10-16:
    # 0.7173 against 0.7154, 286 mm apart): keep the best separate local maximum as a rival.
    w = max(1, int(round(params.peak_window_m / b)))
    padded = np.pad(profile, w, constant_values=-np.inf)
    local = np.array([profile[i] >= padded[i:i + 2 * w + 1].max() for i in range(len(profile))])
    far = np.abs(np.arange(len(profile)) - j) * b > params.separate_placement_m
    others = np.nonzero(local & far)[0]
    k = int(others[np.argmax(profile[others])]) if len(others) else None
    # Every separate maximum within the ambiguity margin is kept, not only the runner-up (Codex
    # review A round 3: a third placement near the true centre was lost and a target looked unique).
    close = sorted(((float(profile[i]), u0 + (i + nu_t / 2) * b, h_lo + int(np.argmax(grid[:, i])) * b)
                    for i in others if profile[i] >= profile[j] - params.ambiguity_margin), reverse=True)
    return Placement2D(float(profile[j]), u0 + (j + nu_t / 2) * b, h_lo + s * b,
                       float(profile[k]) if k is not None else -math.inf,
                       u0 + (k + nu_t / 2) * b if k is not None else math.nan, tuple(close))


@dataclass(frozen=True)
class Hypothesis:
    label: str             # "<shape>:<axis>" with ":m" for a mirrored raster
    spec: PalletSpec
    axis: str
    mirror: bool
    members: tuple         # (lo, hi) in u relative to the face centre, left = +u
    band: tuple            # pocket band (z0, z1) above the support
    placement: Placement2D
    face: int


def _mirrored(intervals, mirror):
    return tuple(sorted((-hi, -lo) for lo, hi in intervals)) if mirror else tuple(intervals)


def known_hypotheses(specs: dict, face_index: int, pts_u, pts_h, rays, params, h_lo, n_shift):
    out = []
    for name in sorted(specs):
        spec = specs[name]
        for axis in spec.entry_axes:
            layout = face_layout(spec, axis)
            base = face_raster(spec, axis, params.bin_m)
            for mirror in ((False,) if np.array_equal(base, base[:, ::-1]) else (False, True)):
                occ = base[:, ::-1] if mirror else base
                out.append(Hypothesis(
                    f"{spec.name}:{axis}" + (":m" if mirror else ""), spec, axis, mirror,
                    _mirrored(layout.member_intervals_m, mirror), layout.pocket_band_m,
                    best_placement(occ, pts_u, pts_h, rays, params, h_lo, n_shift,
                                   damageable_rows(spec, params.bin_m)), face_index))
    return out


def observed_members(hyp: Hypothesis, rays, params: FitParams, window_m: float) -> int:
    """Members with a side edge seen in the pocket band: rays owned by the proposal stop on
    the face inside the member, raw rays pass behind it just outside (§7.2: a neighbour's
    face pixels are not this instance's evidence)."""
    zs = hyp.placement.support_z
    ru, rh, rc, owned = rays[:4]
    band = (rh >= zs + hyp.band[0]) & (rh <= zs + hyp.band[1])
    on_face, behind = band & (rc == FACE) & owned, band & (rc == BEHIND)
    seen = 0
    for lo, hi in hyp.members:
        edges = 0
        for edge, inward in ((hyp.placement.u_centre + lo, 1.0), (hyp.placement.u_centre + hi, -1.0)):
            off = inward * (ru - edge)
            inside = on_face & (off >= 0) & (off <= window_m)
            outside = behind & (off < 0) & (off >= -window_m)
            edges += int(inside.sum() >= params.edge_min_hits and outside.sum() >= params.edge_min_hits)
        seen += edges > 0
    return seen


def support_state(scene: SceneInput, rays, mask, face: FacePlane, support_z: float, params: FitParams):
    """How the surface under the object's lower edge was seen (design §7.3).

    Only columns whose lowest object pixel lies on the face at the support height are used
    (under an open lane the lowest pixel is a lane wall behind the face). Two pixels below that
    edge and five pixels below it are compared: on a horizontal surface the height stays the
    same, on a vertical one (a stacked pallet's face) it drops by the pixel step times the range.
    visible -- a level surface at the support height, seen from above it (its measured height
      is returned, §7.3: a visible support gives z_s);
    coplanar -- the face continues down below the edge: the edge may be a stack joint;
    physical_edge -- the rays pass behind the face (nothing under the edge in view);
    uncertain -- too few rays, or something nearer hides the edge.
    Returns (state, measured height or None). Codex 2026-10-16: at 1.3 m the floor just below the
    edge is only 22 mm in front of the face, so a distance band alone took it for the face.
    """
    cam = camera_xyz(scene)
    margin = _margin(params, _reach(cam, face.centre_xy))
    near_rows, far_rows = [], []
    for c in np.nonzero(mask.any(axis=0))[0]:
        bottom = int(np.nonzero(mask[:, c])[0].max())
        d0 = scene.depth_m[bottom, c]
        if not np.isfinite(d0) or bottom + 6 >= mask.shape[0] or mask[bottom + 2:bottom + 7, c].any():
            continue
        p0 = cam + d0 * rays[bottom, c]
        if abs(p0[2] - support_z) > params.support_window_m or abs((p0[:2] - face.centre_xy) @ face.normal_xy) > margin:
            continue
        near_rows.append((bottom + 2, c))
        far_rows.append((bottom + 6, c))
    if len(near_rows) < 5:
        return "uncertain", None
    (v1, c1), (v2, c2) = np.array(near_rows).T, np.array(far_rows).T
    d1, d2 = scene.depth_m[v1, c1], scene.depth_m[v2, c2]
    ok = np.isfinite(d1) & np.isfinite(d2)
    if ok.mean() < 0.5:
        return "uncertain", None
    p1 = cam + d1[ok, None] * rays[v1[ok], c1[ok]]
    p2 = cam + d2[ok, None] * rays[v2[ok], c2[ok]]
    ahead = (p1[:, :2] - face.centre_xy) @ face.normal_xy
    drop = p1[:, 2] - p2[:, 2]                                   # height lost over four pixel rows
    vertical_drop = 4.0 * _reach(cam, face.centre_xy) / scene.intrinsics.fy
    level = np.abs(p1[:, 2] - support_z) <= params.support_window_m
    horizontal = np.median(np.abs(drop)) < 0.4 * vertical_drop
    if horizontal and level.mean() > 0.5 and np.median(ahead) > -margin and cam[2] > support_z + params.support_window_m:
        return "visible", float(np.median(p1[level, 2]))
    if np.median(np.abs(ahead)) <= margin:
        return "coplanar", None
    if np.median(ahead) < -margin:
        return "physical_edge", None
    return "uncertain", None


def shape_evidence(hyp: Hypothesis, pts_u, pts_h, rays, params: FitParams) -> dict:
    """How well a known shape explains the face beyond its score (design §7.4-4, §7.5).

    outside: share of face points outside the shape's face rectangle;
    top_gap_m: observed top of the face (98th percentile of h) minus the shape's top (recorded);
    seen_through: share of the classified rays on the shape's material that pass behind it, the
      damageable top-deck band left out (missing or broken top boards are allowed damage, §5.2;
      Codex 2026-10-16: a missing edge board, or two, must not read as another shape).
    On the stage 1B Isaac dev scenes (fitted candidates), known shapes stayed at outside p99
    <= 0.06 and seen_through p99 <= 0.09 up to the mid noise level (damaged S5 included),
    while 70 % of the non-standard S7 pallets fitted as a known shape exceeded the limits.
    """
    layout = face_layout(hyp.spec, hyp.axis)
    zs, uc = hyp.placement.support_z, hyp.placement.u_centre
    du = pts_u - uc
    w, top = layout.width_m, zs + hyp.spec.overall_height_m
    outside = float(((np.abs(du) > w / 2 + 2 * params.bin_m) | (pts_h > top + 0.02)).mean())
    occ = face_raster(hyp.spec, hyp.axis, params.bin_m, mirror=hyp.mirror)
    ru, rh, rc = rays[0], rays[1], rays[2]
    occ = occ & ~damageable_rows(hyp.spec, params.bin_m)[:, None]
    iu = np.floor((ru - (uc - w / 2)) / params.bin_m).astype(int)
    ih = np.floor((rh - zs) / params.bin_m).astype(int)
    ok = (iu >= 0) & (iu < occ.shape[1]) & (ih >= 0) & (ih < occ.shape[0]) & ((rc == FACE) | (rc == BEHIND))
    mat = np.zeros(len(ru), bool)
    mat[ok] = occ[ih[ok], iu[ok]]
    seen = int(mat.sum())
    return {"outside": outside, "top_gap_m": float(np.percentile(pts_h, 98)) - top,
            "seen_through": float((mat & (rc == BEHIND)).sum() / seen) if seen else 1.0}


def _pockets(hyp: Hypothesis, face: FacePlane, setup, load):
    fit = fork_fit(hyp.spec, hyp.axis, setup, load, support_z_m=max(hyp.placement.support_z, 0.0))
    layout = face_layout(hyp.spec, hyp.axis)
    pockets = []
    for i in fit.pair:
        lo, hi = layout.opening_intervals_m[i]
        pockets.append(((-(lo + hi) if hyp.mirror else (lo + hi)) / 2, hi - lo))
    (r_lat, r_w), (l_lat, l_w) = sorted(pockets)
    centre = face.centre_xy + face.left_xy * hyp.placement.u_centre
    z = hyp.placement.support_z + (hyp.band[0] + hyp.band[1]) / 2
    height = hyp.band[1] - hyp.band[0]
    left = PocketEstimate((*(centre + face.left_xy * l_lat), z), l_w, height, "known_shape", "known_shape")
    right = PocketEstimate((*(centre + face.left_xy * r_lat), z), r_w, height, "known_shape", "known_shape")
    return fit, centre, left, right


def _identity(proposal: InstanceProposal, threshold) -> str:
    if proposal.source == "oracle":
        return "confirmed"
    if proposal.source == "learned" and threshold is not None and proposal.score >= threshold:
        return "confirmed"
    return "unconfirmed"


def _runs(flags):
    out, start = [], None
    for i, f in enumerate(list(flags) + [False]):
        if f and start is None:
            start = i
        if not f and start is not None:
            out.append((start, i))
            start = None
    return out


@dataclass(frozen=True)
class ParametricReading:
    """A non-standard entry face read off the data (§7.1): member and opening layout across the
    face, the lane ceiling above the support, and whether each end is the object's physical end.
    The depth is not read, and the top layout is left to the near range: from the front it is a few
    pixels this reader does not use (§3.2), not something that cannot be seen."""
    layout: FaceLayout
    u_centre: float
    support_z: float
    top_m: float
    ends_physical: tuple


def read_parametric_face(face: FacePlane, pts_u, pts_h, rays, support_z: float, params: FitParams, rows=None):
    """Members and openings from the face's own points; ends from the raw rays it owns.

    The top band is the contiguous run of rows from the top that are at least parametric_row_fill
    full, counted on rows=(u, h) -- the proposal's un-eroded pixels on the face plane: erosion
    removes most of a thin deck band at range (mixed-depth edge pixels fall off the plane anyway).
    Below it, a column is a member if it is at least parametric_member_fill full, and a gap is
    an opening only if it is min_opening_m wide and rays pass behind the face through it. Fork
    lanes run down to the support (bottom boards only under the members), so the lane floor is
    the support and the bottom-board thickness is not observable from the front. Returns
    (reading, None) or (None, reason).
    """
    b = params.bin_m
    ru, rh, rc, owned = rays
    top = float(np.percentile(pts_h, 99))
    on = owned & (rc == FACE) & (rh > support_z + 0.005) & (rh < top)
    if on.sum() < 20:
        return None, "few_face_rays"
    umin, umax = float(np.percentile(ru[on], 0.5)), float(np.percentile(ru[on], 99.5))
    nu, nh = int(math.ceil((umax - umin) / b)), int(math.ceil((top - support_z) / b))
    if nu < 10 or nh < 4:
        return None, "too_small"
    occ = np.zeros((nh, nu), bool)
    occ[np.clip(((pts_h - support_z) / b).astype(int), 0, nh - 1), np.clip(((pts_u - umin) / b).astype(int), 0, nu - 1)] = True
    row_u, row_h = (pts_u, pts_h) if rows is None else rows
    keep = (row_u >= umin) & (row_u <= umax) & (row_h >= support_z)
    rocc = np.zeros((nh, nu), bool)
    rocc[np.clip(((row_h[keep] - support_z) / b).astype(int), 0, nh - 1),
         np.clip(((row_u[keep] - umin) / b).astype(int), 0, nu - 1)] = True
    row_fill = rocc.mean(axis=1)
    # Above the full deck run there can be rows that are not full: a ragged top edge (mixed-depth
    # pixels, the 99th-percentile top cuts through a row) and, where top boards run along the
    # insertion axis, their ends with gaps between them (S7 fill 0.64). Skip up to
    # parametric_top_skip_m of them before the deck run starts, else the run is empty and the
    # ceiling lands on the top, reading the whole deck as openings (dev 2026-10-16).
    k = nh - 1
    while k > 0 and row_fill[k] < params.parametric_row_fill and (nh - 1 - k) * b < params.parametric_top_skip_m:
        k -= 1
    if row_fill[k] < params.parametric_row_fill:
        # No full row near the top: there is no deck band to bound the lanes. Inventing a ceiling
        # at the top read free posts and a deck-less cut as a pallet (Codex review A round 3).
        return None, "no_deck_band"
    while k > 0 and row_fill[k] >= params.parametric_row_fill:
        k -= 1
    ceiling = (k + 1) * b
    if ceiling < params.min_lane_ceiling_m:
        return None, "no_lane_band"
    col_fill = occ[1:k + 1].mean(axis=0) if k >= 1 else occ[:1].mean(axis=0)
    band = (rh > support_z + 0.01) & (rh < support_z + ceiling - 0.005)
    sel = band & (rc == BEHIND) & (ru >= umin) & (ru < umax)
    behind = np.zeros(nu)
    np.add.at(behind, np.clip(((ru[sel] - umin) / b).astype(int), 0, nu - 1), 1)
    member = col_fill >= params.parametric_member_fill
    openings = []
    for lo, hi in _runs(~member):
        if (hi - lo) * b >= params.min_opening_m and behind[lo:hi].sum() >= params.edge_min_hits:
            openings.append((lo, hi))
        else:
            member[lo:hi] = True
    members = _runs(member)
    if len(members) < 3 or len(openings) < 2:
        return None, f"layout:{len(members)}m{len(openings)}o"
    c = (umin + umax) / 2
    deck = (rh > support_z + ceiling) & (rh < top)
    ends = []
    for edge, side in ((umin, -1.0), (umax, 1.0)):
        off = side * (ru - edge)
        window = (off > params.end_window_m[0]) & (off < params.end_window_m[1])
        beyond = band & window
        # The face plane must not go on beyond the end under another owner: a mask cut through
        # touching pallets has the neighbour's deck continuing in the plane (Codex review
        # 2026-10-16; rig: 54 face rays, none behind). It is judged at deck height, where a real
        # end has far background -- in the lane band, floor hits just behind the plane read as
        # face under noise (dev mid: up to 2.25 face per behind ray at true ends). Deck-band face
        # rays outnumbering the see-through ones flagged 0 of 166 true ends clean, 2-4 of ~150 noisy.
        cont = deck & window & (rc == FACE) & ~owned
        seen_behind = (deck & window & (rc == BEHIND)).sum()
        continued = cont.sum() >= params.edge_min_hits and cont.sum() >= seen_behind
        # Seeing nothing at deck height is not seeing an end: missing depth there could hide a
        # continuing deck (Codex review A round 2). Dev cost: 1 of ~490 true ends.
        # Occluded at deck height is not observed either (Codex review A round 3).
        missing = (deck & window & ((rc == UNOBSERVED) | (rc == FRONT))).sum()
        unseen = missing >= params.edge_min_hits and missing >= seen_behind
        # And open space must be seen at deck height: lane-height see-through alone can be a
        # neighbour's openings beyond a cut (Codex review A round 4). Dev cost: 10-13 of ~125 true ends.
        # Continuation and unseen rays are both evidence against an end: split between the two they
        # must not outweigh the open space either (Codex review A round 5: behind 24, face 18,
        # missing 18 passed each check alone), and a tie is not an end (round 6: 30 against 30).
        # With these ray counts a wider margin costs true ends: under the provisional noise they
        # reach 1.15-2.4 against per behind ray at the tail (dev 2026-10-16) -- a trade-off of this
        # statistic, not a limit of every single-frame method. Cut masks belong to the split
        # hypothesis of design 7.4-3 (stage 2B-2, with the proposer that produces them).
        against = cont.sum() + missing
        ends.append(bool(((beyond & (rc == BEHIND)).sum() >= params.edge_min_hits)
                         and ((beyond & (rc == FRONT)).sum() < params.edge_min_hits)
                         and seen_behind >= params.edge_min_hits
                         and not continued and not unseen
                         and not (against >= params.edge_min_hits and against >= seen_behind)))
    layout = FaceLayout("x", umax - umin, math.nan,
                        tuple((lo * b + umin - c, hi * b + umin - c) for lo, hi in members),
                        tuple((lo * b + umin - c, hi * b + umin - c) for lo, hi in openings),
                        (0.0, ceiling), 0.0, ceiling)
    return ParametricReading(layout, c, support_z, top, (ends[0], ends[1])), None


def _layout_raster(layout: FaceLayout, height: float, bin_m: float):
    """Occupancy of a parametric face: members up to the lane ceiling, the full width above."""
    nu, nh = int(round(layout.width_m / bin_m)), max(1, int(round(height / bin_m)))
    uc = -layout.width_m / 2 + (np.arange(nu) + 0.5) * bin_m
    hc = (np.arange(nh) + 0.5) * bin_m
    occ = np.zeros((nh, nu), bool)
    occ[hc >= layout.lane_ceiling_m, :] = True
    for lo, hi in layout.member_intervals_m:
        occ[:, (uc >= lo) & (uc <= hi)] = True
    return occ


def _parametric_fit(layout: FaceLayout, setup, load, depth_max_m: float,
                    spacing_slack_m: float = FitParams.parametric_spacing_slack_m):
    """What the face and the instruction decide (§9.3): the fork spacing, the support and the payload.

    Each opening pair is judged on its lateral margin and its support together, like fork_fit: the
    widest-margin pair may fail lateral support while another passes (Codex 2026-10-16). Only the
    instruction's load counts: without it the support is unverified, as in fork_fit -- a non-standard
    pallet has no template mass and the near range cannot supply one. The depth is not seen, so the
    longitudinal support uses the largest insertion target any depth up to depth_max_m allows -- a
    load centre beyond it fails at every depth. The lift is not decided: the
    far reading's 5 mm rows do not bound the lane tightly enough for a 2.5 mm margin, so the lane
    bounds wait for the near range (FAR_PENDING). The lateral margin stops the pick only below
    M_REQ - spacing_slack_m: the far reading's margins can be well under the true ones, and between
    the two the near range decides on M_REQ. Returns (pair, pickable False | None, reasons)."""
    target_max = target_insertion_depth_m(depth_max_m, setup.carriage_limit_m, setup.reserve_m)
    options = []
    n = len(layout.opening_intervals_m)
    for i in range(n):
        for j in range(i + 1, n):
            margin, centre = pair_margin(layout, (i, j), setup.spacing_m, setup.blade_width_m)
            reasons = set()
            if margin < M_REQ_PROVISIONAL_M - spacing_slack_m:
                reasons.add("fork_spacing_adjust")
            if load is None or load.source != "instruction" or (
                    target_max < load.com_x_max_m + SUPPORT_LONGITUDINAL_MARGIN_M
                    or abs(centre) + load.com_y_max_m > setup.spacing_m / 2 - SUPPORT_LATERAL_MARGIN_M):
                reasons.add("support_unverified")
            options.append(((-round(margin, 9), round(abs(centre), 9)), (i, j), reasons))
    options.sort(key=lambda o: o[0])
    _, pair, reasons = next((o for o in options if not o[2]), options[0])
    if load is not None and load.pallet_mass_kg + load.load_mass_kg > PAYLOAD_LIMIT_KG:
        reasons = reasons | {"payload_limit"}
    ordered = tuple(code for code in REASON_CODES if code in reasons)
    return pair, (False if ordered else None), ordered


def _parametric_candidate(scene, proposal, cam, face, fi, pts_u, pts_h, rays_f, reading, setup, load_for, params,
                          candidate_id, identity_threshold, competing, state, seen, extent):
    layout = reading.layout
    occ = _layout_raster(layout, reading.top_m - reading.support_z, params.bin_m)
    # The top band was read as one block, but boards leave gaps the rays pass through: like the
    # damageable band of a known shape, it is not checked for seen-through rays.
    top_band = (np.arange(occ.shape[0]) + 0.5) * params.bin_m >= layout.lane_ceiling_m
    placement = best_placement(occ, pts_u, pts_h, rays_f, params, reading.support_z, 1, top_band)
    hyp = Hypothesis("parametric:x", None, "x", False, layout.member_intervals_m, layout.pocket_band_m, placement, fi)
    reach = _reach(cam, face.centre_xy + face.left_xy * placement.u_centre)
    geometry, reasons = "fitted", []
    if placement.score < params.min_score:
        geometry, reasons = "unfit", ["weak_fit"]
    elif placement.score - placement.runner_up_score < params.ambiguity_margin:
        geometry, reasons = "ambiguous", ["competing:parametric:x@shifted"]
    elif pts_u.max() - pts_u.min() < params.min_face_span_m:
        geometry, reasons = "ambiguous", ["yaw_unconstrained"]
    elif observed_members(hyp, rays_f, params, max(params.edge_window_m, 4 * reach / scene.intrinsics.fx)) < 2:
        geometry, reasons = "ambiguous", ["lateral_unconstrained"]
    elif not all(reading.ends_physical):
        # A non-standard reading must see both physical ends: the inner part of two touching
        # pallets looks like one wide non-standard pallet (§7.4-5c).
        geometry, reasons = "ambiguous", ["end_unobserved"]
    pair, pickable, fit_reasons = _parametric_fit(layout, setup, load_for(None), params.nonstandard_depth_m[1],
                                                  params.parametric_spacing_slack_m)
    centre = face.centre_xy + face.left_xy * placement.u_centre
    lats = sorted(((layout.opening_intervals_m[i][0] + layout.opening_intervals_m[i][1]) / 2,
                   layout.opening_intervals_m[i][1] - layout.opening_intervals_m[i][0]) for i in pair)
    # The lane floor is not read, so the pocket centre sits at the support + half the ceiling: below
    # the truth convention (deck_bottom + ceiling) / 2 by deck_bottom / 2 (S7 dev height p95 12.8 mm
    # against 5 mm, Codex 2026-10-16). The near range re-reads both lane bounds and regenerates the
    # pockets (stage 4); until then lane_bounds is pending.
    z = reading.support_z + layout.lane_ceiling_m / 2
    right = PocketEstimate((*(centre + face.left_xy * lats[0][0]), z), lats[0][1], layout.lane_ceiling_m,
                           "observed", "observed")
    left = PocketEstimate((*(centre + face.left_xy * lats[1][0]), z), lats[1][1], layout.lane_ceiling_m,
                          "observed", "observed")
    d_lo, d_hi = params.nonstandard_depth_m
    # The depth is not seen: the centre may lie anywhere on the segment from half the shallowest to
    # half the deepest depth behind the face; selection tests the whole segment (Codex 2026-10-16:
    # five samples missed an instruction between two of them).
    pallet_centre = centre + face.insertion_xy * (d_lo + d_hi) / 4
    segment = tuple((float(p[0]), float(p[1]), float(reading.support_z))
                    for p in (centre + face.insertion_xy * d_lo / 2, centre + face.insertion_xy * d_hi / 2))
    alternatives = tuple(competing) + segment
    if state == "uncertain":
        reasons = list(reasons) + ["support_uncertain"]
    shape = ObservedPalletShape(None, layout, (("face", "observed"), ("depth", "unobserved"), ("top", "unobserved"),
                                               ("support", "unobserved" if state == "uncertain" else "observed")))
    yaw = math.atan2(face.insertion_xy[1], face.insertion_xy[0])
    return PocketCandidate(
        candidate_id, proposal.proposal_id, proposal.source, scene.stamp_ns, "parametric:x", "x", (), yaw,
        (float(centre[0]), float(centre[1]), float(reading.support_z)),
        (float(pallet_centre[0]), float(pallet_centre[1]), float(reading.support_z)), left, right,
        reading.support_z, state, state == "visible" and abs(reading.support_z) <= params.support_window_m, reach,
        placement.score, geometry, _identity(proposal, identity_threshold), tuple(reasons), pickable, fit_reasons,
        shape, alternatives, (float(seen[0]), float(seen[1])), extent, PARAMETRIC_PENDING, segment)


def _decide(hyps, faces, evidence, pts, scene, setup, load_for, params):
    """Rank the hypotheses and apply every gate: weak fit, rivals, face span, member edges, shape."""
    cam = camera_xyz(scene)
    scored = []
    for hyp in hyps:
        face = faces[hyp.face]
        fit, centre, left, right = _pockets(hyp, face, setup, load_for(hyp.spec))
        scored.append((hyp, fit, centre, left, right, math.atan2(face.insertion_xy[1], face.insertion_xy[0])))
    # Faces compete by total evidence (a face with few points must not win on a normalised
    # score); readings of the same face compete by normalised score.
    sizes = [len(f.index) for f in faces]
    scored.sort(key=lambda s: -s[0].placement.score * sizes[s[0].face])
    best = scored[0]

    def equivalent(a, b):
        # Only the same reading of the same face merges: an x and a y reading of a square T11
        # face act differently (lane height, near tracking) and must compete (design §7.4-7).
        d = max(np.linalg.norm(np.subtract(a[3].centre_m, b[3].centre_m)),
                np.linalg.norm(np.subtract(a[4].centre_m, b[4].centre_m)))
        dyaw = abs((a[5] - b[5] + math.pi) % (2 * math.pi) - math.pi)
        return (a[0].axis == b[0].axis and a[1].reasons == b[1].reasons
                and d <= params.equivalence_m and dyaw <= params.equivalence_yaw_rad)

    rivals = sorted((s for s in scored[1:] if s[0].face == best[0].face and not equivalent(best, s)),
                    key=lambda s: -s[0].placement.score)
    hyp = best[0]
    u, rays_f = evidence[hyp.face]
    reach = _reach(cam, best[2])
    geometry, reasons = "fitted", []
    if hyp.placement.score < params.min_score:
        geometry, reasons = "unfit", ["weak_fit"]
    elif rivals and hyp.placement.score - rivals[0][0].placement.score < params.ambiguity_margin:
        geometry, reasons = "ambiguous", [f"competing:{rivals[0][0].label}"]
    elif hyp.placement.score - hyp.placement.runner_up_score < params.ambiguity_margin:
        geometry, reasons = "ambiguous", [f"competing:{hyp.label}@shifted"]
    elif u.max() - u.min() < params.min_face_span_m:
        geometry, reasons = "ambiguous", ["yaw_unconstrained"]
    elif observed_members(hyp, rays_f, params, max(params.edge_window_m, 4 * reach / scene.intrinsics.fx)) < 2:
        geometry, reasons = "ambiguous", ["lateral_unconstrained"]
    else:
        ev = shape_evidence(hyp, u, pts[faces[hyp.face].index, 2], rays_f, params)
        if ev["outside"] > params.shape_outside_max or ev["seen_through"] > params.shape_seen_through_max:
            # The best known shape does not explain this face: a candidate for the parametric
            # (non-standard) reading of stage 2B, never a confident known-shape fit.
            geometry, reasons = "ambiguous", ["shape_mismatch"]
    return best, rivals, geometry, reasons


def fit_instance(scene: SceneInput, proposal: InstanceProposal, *, specs: dict, setup, load_for,
                 params: FitParams = FitParams(), rng=None, candidate_id: str = "c0", rays=None,
                 identity_threshold=None) -> PocketCandidate | None:
    """Best known-shape hypothesis for one proposal, or None when no face can be fitted.

    When the support under the chosen face is seen, its measured height fixes z_s (§7.3) and the
    whole decision -- every hypothesis on that face, the ranking and all gates -- is made again
    on it, so the returned placement and its verdict always belong together (Codex 2026-10-16);
    if the new decision moves to another face, that face's support is fixed in turn.
    """
    rng = np.random.default_rng(params.seed) if rng is None else rng
    rays = pixel_rays(scene) if rays is None else rays
    cam = camera_xyz(scene)
    pts, _ = proposal_points(scene, rays, proposal.mask, params.erode_px)
    faces = fit_faces(pts, cam, params, rng)
    if not faces:
        return None
    wmax = max(face_layout(s, a).width_m for s in specs.values() for a in s.entry_axes)
    hmax = max(s.overall_height_m for s in specs.values())
    n_shift = int(round(2 * params.support_window_m / params.bin_m)) + 1
    evidence, lows = [], []
    for face in faces:
        u, h = face_coords(face, pts[face.index])
        h_lo = float(np.percentile(h, 2)) - params.support_window_m
        rays_f = ray_samples(scene, rays, face, (u.min() - wmax - 0.05, u.max() + wmax + 0.05),
                             (h_lo, h_lo + hmax + 2 * params.support_window_m + 0.1), params, mask=proposal.mask)
        evidence.append((u, rays_f))
        lows.append(h_lo)
    fixed = {}
    # Each pass fixes the support of the face it chose; it ends when the chosen face is already
    # fixed or has no measured support. A face is fixed at most once, so len(faces) + 1 passes
    # always end on a decision made with every fix applied (Codex 2026-10-16: with two passes a
    # switch to the other face returned that face uncorrected, 90 deg off and the wrong fork fit).
    for _ in range(len(faces) + 1):
        hyps = []
        for fi, face in enumerate(faces):
            u, rays_f = evidence[fi]
            h_lo, shifts = (fixed[fi], 1) if fi in fixed else (lows[fi], n_shift)
            hyps += known_hypotheses(specs, fi, u, pts[face.index, 2], rays_f, params, h_lo, shifts)
        best, rivals, geometry, reasons = _decide(hyps, faces, evidence, pts, scene, setup, load_for, params)
        hyp = best[0]
        face = faces[hyp.face]
        state, measured = support_state(scene, rays, proposal.mask, face, hyp.placement.support_z, params)
        if hyp.face in fixed or measured is None or abs(measured - hyp.placement.support_z) <= params.bin_m / 2:
            break
        fixed[hyp.face] = measured
    hyp, fit, centre, left, right, yaw = best
    reach = _reach(cam, centre)
    fit_reasons = tuple(fit.reasons)
    if state == "uncertain":
        # An unseen support may not decide anything that reads the height (§7.3).
        fit_reasons = tuple(r for r in fit_reasons if r not in HEIGHT_REASONS)
        reasons = list(reasons) + ["support_uncertain"]
    # Far range: a reason the face decides stops the pick; otherwise it waits for the near range.
    pickable = False if fit_reasons else None
    layout = face_layout(hyp.spec, hyp.axis)
    pallet_centre = centre + face.insertion_xy * layout.depth_m / 2
    # Centres of the readings close enough to make this one ambiguous: the shifted placement of the
    # same raster and the rival hypotheses. Target selection treats an ambiguous candidate as in an
    # instruction's radius if any of them is (Codex review A round 2).
    competing = []
    close = hyp.placement.score - params.ambiguity_margin
    for h in [hyp] + [r[0] for r in rivals]:
        depth = face_layout(h.spec, h.axis).depth_m
        f = faces[h.face]
        options = list(h.placement.close_maxima)
        if h is not hyp:
            options.append((h.placement.score, h.placement.u_centre, h.placement.support_z))
        for score, u_c, z in options:
            if score >= close:
                c = f.centre_xy + f.left_xy * u_c + f.insertion_xy * depth / 2
                competing.append((float(c[0]), float(c[1]), float(z)))
    shape = ObservedPalletShape(hyp.spec, layout, (("face", "known_shape"), ("depth", "known_shape"),
                                                    ("support", "unobserved" if state == "uncertain" else "observed")))
    zs = hyp.placement.support_z
    seen = np.median(pts[:, :2], axis=0)
    extent = float(np.max(np.hypot(*(pts[:, :2] - seen).T)))
    if params.parametric and geometry != "fitted" and reasons and reasons[0].split(":")[0] in ("shape_mismatch", "weak_fit"):
        # No known shape explains this object (shape mismatch or a weak fit): read the face as a
        # non-standard pallet (§7.1), on the faces in order of evidence. A known reading that is only
        # under-constrained (occluded member edges) or ambiguous between known shapes is left as it
        # is -- trying the parametric reading there read partly hidden known pallets as narrower
        # non-standard ones (dev 2026-10-16). The known reading's centre stays among the competing ones.
        competing_all = tuple(competing) + ((float(pallet_centre[0]), float(pallet_centre[1]), float(zs)),)
        raw, _ = proposal_points(scene, rays, proposal.mask, 0)
        order = sorted(range(len(faces)), key=lambda k: -len(faces[k].index))
        readings = []
        for fi in order:
            f = faces[fi]
            u_f, rays_f = evidence[fi]
            h_f = pts[f.index, 2]
            low = float(np.percentile(h_f, 2))
            st, measured = support_state(scene, rays, proposal.mask, f, low, params)
            on_plane = np.abs((raw[:, :2] - f.centre_xy) @ f.normal_xy) < max(params.ransac_tol_m, 2.5 * f.scale_m)
            rows = face_coords(f, raw[on_plane])
            reading, _ = read_parametric_face(f, u_f, h_f, rays_f, measured if measured is not None else low, params, rows)
            if reading is not None:
                readings.append((fi, f, u_f, h_f, rays_f, reading, st))
        if readings:
            # The first reading in order of evidence. Which face is the entry face is not known at far
            # range: every non-standard candidate leaves entry_axis to the near range (user decision
            # 2026-10-17 -- approach allowed, pick only after the axis is confirmed), so a side face that
            # reads is not refused here (the S7 sides are entry faces; a blocked side that reads as one
            # is still stopped by the fork fit and the near-range check).
            fi, f, u_f, h_f, rays_f, reading, st = readings[0]
            return _parametric_candidate(scene, proposal, cam, f, fi, u_f, h_f, rays_f, reading, setup, load_for,
                                         params, candidate_id, identity_threshold, competing_all, st, seen, extent)
    return PocketCandidate(
        candidate_id, proposal.proposal_id, proposal.source, scene.stamp_ns, hyp.label, hyp.axis,
        tuple((s[0].label, round(s[0].placement.score, 4)) for s in rivals[:3]), yaw,
        (float(centre[0]), float(centre[1]), float(zs)), (float(pallet_centre[0]), float(pallet_centre[1]), float(zs)),
        left, right, zs, state, state == "visible" and abs(zs) <= params.support_window_m, reach,
        hyp.placement.score, geometry, _identity(proposal, identity_threshold), tuple(reasons), pickable,
        fit_reasons, shape, tuple(competing), (float(seen[0]), float(seen[1])), extent, KNOWN_FAR_PENDING)


@dataclass(frozen=True)
class FitFailure:
    proposal_id: str
    reason: str                  # no_face, or exception:<type> (§10)
    centre_xy: tuple | None      # median base-frame xy of the proposal's depth points, if any
    extent_m: float = 0.0        # largest distance of those points from centre_xy


@dataclass(frozen=True)
class SceneFit:
    candidates: tuple
    failures: tuple              # FitFailure: an object that was seen but not fitted


def fit_scene(scene: SceneInput, proposals, *, specs: dict, setup, load_for, params: FitParams = FitParams(),
              identity_threshold=None) -> SceneFit:
    """Fit every proposal; one proposal's failure never stops the others."""
    rng = np.random.default_rng(params.seed)
    rays = pixel_rays(scene)
    candidates, failures = [], []
    for k, proposal in enumerate(proposals):
        if proposal.mask.shape != scene.depth_m.shape:
            failures.append(FitFailure(proposal.proposal_id, "bad_mask:shape", None))
            continue
        try:
            cand = fit_instance(scene, proposal, specs=specs, setup=setup, load_for=load_for, params=params,
                                rng=rng, candidate_id=f"c{k}", rays=rays, identity_threshold=identity_threshold)
            reason = None if cand is not None else "no_face"
        except Exception as exc:  # noqa: BLE001 -- isolated per proposal by design (§10)
            cand, reason = None, f"exception:{type(exc).__name__}"
        if cand is not None:
            candidates.append(cand)
            continue
        try:
            pts, _ = proposal_points(scene, rays, proposal.mask, 0)
        except Exception:  # noqa: BLE001 -- a failure must never stop the scene (Codex review A)
            pts = np.zeros((0, 3))
        if len(pts):
            c = np.median(pts[:, :2], axis=0)
            failures.append(FitFailure(proposal.proposal_id, reason, (float(c[0]), float(c[1])),
                                       float(np.max(np.hypot(*(pts[:, :2] - c).T)))))
        else:
            failures.append(FitFailure(proposal.proposal_id, reason, None))
    return SceneFit(tuple(candidates), tuple(failures))
