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

from forklift_core.perception.fork_fit import fork_fit
from forklift_core.perception.instance_proposals import InstanceProposal
from forklift_core.perception.pallet_model import ObservedPalletShape, PalletSpec, face_layout, spec_boxes
from forklift_core.perception.pocket_candidate import PocketCandidate, PocketEstimate
from forklift_core.perception.scene_dataset import SceneInput

BEHIND, FACE, FRONT, UNOBSERVED = 0, 1, 2, 3
HEIGHT_REASONS = ("lift_alignment_needed",)  # fork-fit reasons that read the support height


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
    equivalence_m: float = 0.020
    equivalence_yaw_rad: float = math.radians(2.0)
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
    best = None
    for s in range(n_shift):
        score = (_correlate_rows(pm, k_pts, s) + _correlate_rows(rb, k_b, s) + _correlate_rows(rf, k_f, s)
                 - params.outside_weight * n_pts) / n_pts
        j = int(np.argmax(score))
        if best is None or score[j] > best.score:
            best = Placement2D(float(score[j]), u0 + (j + nu_t / 2) * b, h_lo + s * b)
    return best
