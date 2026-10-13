"""D5 v11: depth pocket check referenced to the tracked pallet (no Isaac imports).

docs/plans/2026-10-13-d5-v11-pallet-referenced-evidence.md. The v10 check
(pocket_check.PocketCheck) places its evidence in the control frame fixed at the
near capture and lets it expire after 20 s. Here the evidence lives in the
**pallet frame** of the frame that produced it: origin at the midpoint of the two
observed pocket front centres, x along the insertion axis (away from the truck),
z from the floor. A depth frame certifies only when the near-field tracker
accepted an observation in it (V1), and its rays go camera -> base_link -> pallet
through that observation, so the base pose cancels and only the observation error
(the measured bound, per axis) and the residual alignment motion m_k remain (V2).
Certification is eroded per axis by the frame's own error before it is merged
(V4); there is no lifetime, the store is dropped on invalidation events (V6). A
truck box asked at a tick is placed in the pallet frame through the tracker's
carried estimate and inflated per axis by e_d (V3). The body is not asked within
one voxel of the face (V8). V is the two pocket interiors plus the body band in
front of the face (V7).

The v10 path is untouched: this module adds the v11 check beside it.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field

import numpy as np

from forklift_core.control.drive_permission import StoppingModel, arc_poses
from forklift_core.perception.insertion_clearance import (
    Box,
    InsertionVolume,
    overlapping_voxels,
)
from forklift_core.perception.pallet_geometry import pallet_boxes
from forklift_core.perception.pocket_clearance import DepthCamera

RULES = "d5_v11"
H_DET_M = 0.17  # body band top (D0 detection band), as v10
SCOPE_FLOOR_M = 0.03  # D0 scope floor (user decision 2026-10-05), as v10
FORK_Z_M = (SCOPE_FLOOR_M, 0.072)  # lowered blades 0.028-0.052 +0.01, from the scope floor, as v10
PALLET_TERM_M = 0.00005  # the pallet-stillness term the S4 tracker carries (near_tracking.PALLET_TERM_M)


@dataclass
class V11Config:
    voxel_m: float = 0.01
    band_ahead_m: float = 0.10  # pocket columns start this far in front of the face (as v10)
    band_along_m: float = 0.60  # body band length in front of the face (as v10)
    band_across_m: float = 0.20  # the region's width beyond the pallet (as v10)
    body_lateral_m: float = 0.026  # v10's column/band allowance: estimate 0.01 + tracking 0.006 + envelope 0.01
    body_slack_m: float = 0.05  # as v10
    real_stop_extra_m: float = 0.02  # pocket columns reach the insertion depth + the real stop
    demand_extra_m: float = 0.03  # ... and this e_d room (V7)
    envelope_m: float = 0.01
    sample_m: float = 0.01
    surface_estimate_m: float = 0.02  # surface tolerance's estimate share (D5 delta, 2026-10-05)
    halo_m: float = 0.06  # V is evaluated this far around itself (x, y); wider erosion is refused
    face_cut_m: float = 0.01  # V8: the body is not asked within this of the estimated face
    # The truck's front below h_det is the carriage alone: the chassis box ends carriage_depth_m
    # behind the carriage face and the carriage is carriage_half_width_m wide (the dls08_measured
    # URDF: chassis front 0.099 m behind the carriage face, carriage |y| 0.235 m -- model values,
    # not measurements: the carriage width is estimated and the body front computed, README). None
    # keeps one body box to the carriage face (the plan's v4 model).
    carriage_depth_m: float | None = None
    carriage_half_width_m: float = 0.235
    frame_max_age_s: float = 0.2  # newest new frame's pixels at most this old (with L_hi)


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_rotation(axis, angle: float) -> np.ndarray:
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * (kx @ kx)


def _clip(poly: list, axis: int, c: float, keep_below: bool) -> list:
    """Sutherland-Hodgman: the part of a convex polygon (list of (x, y)) with coordinate `axis`
    <= c (keep_below) or >= c."""
    out = []
    n = len(poly)
    for k in range(n):
        a, b = poly[k], poly[(k + 1) % n]
        ina = a[axis] <= c if keep_below else a[axis] >= c
        inb = b[axis] <= c if keep_below else b[axis] >= c
        if ina:
            out.append(a)
        if ina != inb:
            f = (c - a[axis]) / (b[axis] - a[axis])
            q = (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
            out.append((c, q[1]) if axis == 0 else (q[0], c))
    return out


def _hull(points) -> list:
    """Convex hull (monotone chain) of 2-D points, counter-clockwise."""
    pts = sorted(set((round(float(px), 12), round(float(py), 12)) for px, py in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for q in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    for q in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return lower[:-1] + upper[:-1]


def truck_split_from_urdf(urdf_path, rear_axle_offset_m: float, body_rear_m: float, body_half_width_m: float,
                          carriage_face_m: float, carriage_half_width_m: float, z_band=(SCOPE_FLOOR_M, H_DET_M),
                          steer_samples: int = 61, rim_points: int = 48) -> dict:
    """V14 (plan 6판): the chassis/carriage split read from the URDF collision shapes. Every
    shape that reaches the D0 band (z0 < band top and z1 > band bottom) is sampled over the whole
    allowed range of the revolute joints (the steering envelope; each sample's offset from the
    joint axis circumscribed by 1/cos(half the sample step), so the envelope between samples is
    covered), prismatic joints at 0 and continuous joints at 0 -- the model is refused unless
    every prismatic joint moves its child straight up in base_link (the lift: up only), every
    continuous joint's child carries only cylinders about that joint's axis (a spinning wheel is
    the same shape) and no revolute joint sits under another (independent subtrees, so one
    sample angle for all is every combination). A shape counts when it meets the band at any
    height its prismatic ancestors allow (the lift's whole travel). Boxes by their corners, cylinders by rim
    polygons circumscribed by their radius. The chassis box ends at the frontmost point of any
    shape wider than the carriage half-width; the carriage box runs from there to the carriage
    face, as wide as the widest non-blade point there (at least the carriage front corners).
    Containment is exact for each shape's footprint (its convex hull, clipped at the chassis
    front into the part each box must hold), else ValueError (the model is refused). Lengths in
    the rear-axle frame (x forward from the rear axle)."""
    import xml.etree.ElementTree as ET

    root = ET.parse(urdf_path).getroot()

    def origin(el):
        o = el.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        return np.asarray(xyz), _rpy(*rpy)

    joints, raw_limits = {}, {}
    for j in root.findall("joint"):
        axis = j.find("axis")
        limit = j.find("limit")
        kind = j.get("type")
        if limit is not None:
            raw_limits[j.find("child").get("link")] = (float(limit.get("lower", "nan")), float(limit.get("upper", "nan")))
        joints[j.find("child").get("link")] = (
            j.find("parent").get("link"), *origin(j), kind,
            np.asarray([float(v) for v in axis.get("xyz").split()] if axis is not None else [1.0, 0.0, 0.0]),
            (float(limit.get("lower")), float(limit.get("upper"))) if limit is not None and kind == "revolute"
            else (0.0, 0.0))

    def chain_of(link):
        chain = []
        while link in joints:
            chain.append(joints[link])
            link = joints[link][0]
        return list(reversed(chain))

    def static_rotation(link):  # base <- link at all joints 0
        r = np.eye(3)
        for joint in chain_of(link):
            r = r @ joint[2]
        return r

    # The supported joint set (else refuse): see the docstring.
    children = {}
    for child, joint in joints.items():
        children.setdefault(joint[0], []).append(child)

    def subtree(link):
        out, stack = [], [link]
        while stack:
            k = stack.pop()
            out.append(k)
            stack.extend(children.get(k, []))
        return out

    for child, (_parent, _xyz, _rot, kind, axis, _lim) in joints.items():
        if kind == "prismatic":
            up = static_rotation(child) @ (axis / np.linalg.norm(axis))
            q_lo, q_hi = raw_limits.get(child, (-math.inf, math.inf))
            if (abs(abs(up[2]) - 1) > 1e-9 or not (math.isfinite(q_lo) and math.isfinite(q_hi))
                    or (up[2] > 0 and q_lo < 0) or (up[2] < 0 and q_hi > 0)
                    or any(c[3] != "fixed" for c in chain_of(child)[:-1])):
                raise ValueError(f"{child}: a prismatic joint that does not move only upward in base_link "
                                 "(or sits under a moving joint)")
        elif kind == "continuous":
            # Every shape the spin carries -- the child and every link fixed under it -- must be a
            # cylinder about the spin axis (the same shape at any spin angle).
            a = axis / np.linalg.norm(axis)
            for link_name in subtree(child):
                r_c, t_c = np.eye(3), np.zeros(3)  # child <- link
                k = link_name
                while k != child:
                    j = joints[k]
                    if j[3] != "fixed":
                        raise ValueError(f"{link_name}: a moving joint under a spinning link")
                    r_c, t_c = j[2] @ r_c, j[2] @ t_c + j[1]
                    k = j[0]
                for col in root.find(f"link[@name='{link_name}']").findall("collision"):
                    xyz, crot = origin(col)
                    if col.find("geometry/cylinder") is None:
                        raise ValueError(f"{link_name}: a spinning link carries a non-cylinder collision")
                    ca = r_c @ crot @ np.array([0.0, 0.0, 1.0])
                    centre = r_c @ xyz + t_c
                    off = centre - np.dot(centre, a) * a
                    if abs(abs(float(np.dot(ca, a))) - 1) > 1e-9 or np.linalg.norm(off) > 1e-9:
                        raise ValueError(f"{link_name}: a cylinder not about its joint's spin axis")
        elif kind == "revolute" and any(c[3] == "revolute" for c in chain_of(child)[:-1]):
            raise ValueError(f"{child}: a revolute joint under another revolute joint")
        elif kind not in ("fixed", "revolute", "prismatic", "continuous"):
            raise ValueError(f"{child}: joint type {kind} is not read")

    limits = [v[5] for v in joints.values() if v[3] == "revolute"]
    lo = min((a for a, _ in limits), default=0.0)
    hi = max((b for _, b in limits), default=0.0)
    angles = np.linspace(lo, hi, steer_samples) if hi > lo else np.array([0.0])
    theta_pad = 1.0 / math.cos((hi - lo) / max(steer_samples - 1, 1) / 2) if hi > lo else 1.0

    def to_base(link, local, angle, scale):
        pts = local
        for _parent, xyz, rot, kind, axis, (jlo, jhi) in reversed(chain_of(link)):
            if kind == "revolute":
                a = axis / np.linalg.norm(axis)
                along = np.outer(pts @ a, a)
                pts = along + (pts - along) * scale
                pts = pts @ _axis_rotation(a, min(max(angle, jlo), jhi)).T
            pts = pts @ rot.T + xyz
        return pts

    def lift_range(link):  # the base-z travel the link's prismatic ancestors allow (upward only, checked)
        lo_z = hi_z = 0.0
        k = link
        while k in joints:
            j = joints[k]
            if j[3] == "prismatic":
                up_z = (static_rotation(k) @ (j[4] / np.linalg.norm(j[4])))[2]
                ends = sorted(up_z * q for q in raw_limits[k])
                lo_z, hi_z = lo_z + ends[0], hi_z + ends[1]
            k = j[0]
        return lo_z, hi_z

    pad = 1.0 / math.cos(math.pi / rim_points)
    shapes = []  # (footprint points (x_rear, y), is_blade): per shape, the swept footprint of each angle step
    for link in root.findall("link"):
        steered = any(c[3] == "revolute" for c in chain_of(link.get("name")))
        z_lo, z_hi = lift_range(link.get("name"))
        for col in link.findall("collision"):
            geom = col.find("geometry")
            xyz, rot = origin(col)
            if geom.find("box") is not None:
                hx, hy, hz = (float(v) / 2 for v in geom.find("box").get("size").split())
                local = np.array([[a * hx, b * hy, c * hz] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])
            elif geom.find("cylinder") is not None:
                rad = float(geom.find("cylinder").get("radius")) * pad
                half = float(geom.find("cylinder").get("length")) / 2
                th = np.linspace(0.0, 2 * math.pi, rim_points, endpoint=False)
                ring = np.column_stack([rad * np.cos(th), rad * np.sin(th)])
                local = np.vstack([np.column_stack([ring, np.full(rim_points, z)]) for z in (-half, half)])
            else:
                raise ValueError(f"{col.get('name')}: only box and cylinder collisions are read")
            blade = (col.get("name") or "").endswith("_fork_collision")
            local = local @ rot.T + xyz
            if not steered:
                sweeps = [to_base(link.get("name"), local, 0.0, 1.0)]
            else:
                # Each step between adjacent sample angles: the points at both ends as they are and
                # circumscribed (1/cos(step/2) from the axis) -- every point's arc between the two
                # samples lies between those chords, so the step's hull holds the swept shape.
                at = [np.vstack([to_base(link.get("name"), local, float(g), 1.0),
                                 to_base(link.get("name"), local, float(g), theta_pad)]) for g in angles]
                sweeps = [np.vstack([at[k], at[k + 1]]) for k in range(len(at) - 1)] or at
            for pts in sweeps:
                # The lift moves the shape straight up through [z_lo, z_hi]: in the band at any height?
                if pts[:, 2].min() + z_lo >= z_band[1] or pts[:, 2].max() + z_hi <= z_band[0]:
                    continue
                shapes.append((np.column_stack([pts[:, 0] + rear_axle_offset_m, pts[:, 1]]), blade))
    tol = 1e-9
    hulls = [(_hull(fp), blade) for fp, blade in shapes]
    # The chassis front: the frontmost point of any footprint where it is wider than the carriage
    # (each convex footprint clipped to y >= half and to y <= -half -- edges, not only corners).
    wide = []
    for hull, _ in hulls:
        if len(hull) < 3:
            continue
        edge = carriage_half_width_m + 1e-7  # strictly wider: the carriage itself is exactly half wide
        for side in (_clip(hull, 1, edge, False), _clip(hull, 1, -edge, True)):
            if side:
                wide.append(max(q[0] for q in side))
    if not wide:
        raise ValueError("no shape wider than the carriage reaches the D0 band")
    front = max(wide)
    if front > carriage_face_m + tol:
        raise ValueError(f"a shape wider than the carriage reaches {front:.4f} m, ahead of the carriage face")
    ahead = [max(abs(q[1]) for q in _clip(hull, 0, front, False)) for hull, blade in hulls
             if not blade and len(hull) > 2 and max(q[0] for q in hull) > front + tol]
    half = max([carriage_half_width_m, *ahead])
    outside = []
    for hull, blade in hulls:
        if blade or len(hull) < 3:
            continue
        # Behind the front the chassis box holds the footprint; ahead of it the carriage box
        # (points on the line x = front belong to the chassis box).
        bad = [q for q in _clip(hull, 0, front, True)
               if not (q[0] >= -body_rear_m - tol and abs(q[1]) <= body_half_width_m + tol)]
        bad += [q for q in _clip(hull, 0, front, False) if q[0] > front + tol
                and not (q[0] <= carriage_face_m + tol and abs(q[1]) <= half + tol)]
        outside += [(round(q[0], 4), round(q[1], 4)) for q in bad]
    if outside:
        raise ValueError(f"the truck boxes do not cover the URDF collision shapes: {outside[:5]}")
    return {"chassis_front_m": front, "carriage_depth_m": carriage_face_m - front,
            "carriage_half_width_m": half, "steer_range_rad": [float(lo), float(hi)], "footprints": len(shapes)}


def pallet_from_base(observation) -> tuple[float, float, float]:
    """(ox, oy, yaw): the pallet frame in base_link -- origin at the midpoint of the two
    pocket front centres, x along the observation's insertion yaw."""
    left, right = observation.left.center_m, observation.right.center_m
    return (left[0] + right[0]) / 2, (left[1] + right[1]) / 2, float(observation.insertion_yaw_rad)


def _to_frame(frame, x, y):
    """Points (x, y) expressed in a planar frame (ox, oy, yaw) given in their own frame."""
    ox, oy, yaw = frame
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = np.asarray(x, dtype=float) - ox, np.asarray(y, dtype=float) - oy
    return dx * c + dy * s, -dx * s + dy * c


def pose_in_frame(frame, pose) -> tuple[float, float, float]:
    """A planar pose (x, y, yaw) expressed in frame (ox, oy, yaw), both in one parent."""
    x, y = _to_frame(frame, pose[0], pose[1])
    return float(x), float(y), math.atan2(math.sin(pose[2] - frame[2]), math.cos(pose[2] - frame[2]))


def box_filter_xy(mask: np.ndarray, kx: int, ky: int) -> np.ndarray:
    """Erosion of a boolean grid over a (2kx+1) x (2ky+1) rectangle in x, y (V4: per axis);
    outside the grid counts as not free."""
    out = mask.copy()
    for axis, k in ((0, kx), (1, ky)):
        if k <= 0:
            continue
        n = out.shape[axis]
        pad = [(0, 0)] * out.ndim
        pad[axis] = (k, k)
        padded = np.pad(out, pad, constant_values=False)
        out = np.logical_and.reduce([np.take(padded, range(i, i + n), axis=axis) for i in range(2 * k + 1)])
    return out


def bound_ok(bound) -> bool:
    """A usable observation bound: lateral, along and yaw present, finite and not negative."""
    try:
        vals = [float(bound[k]) for k in ("lateral_m", "along_m", "yaw_rad")]
    except (KeyError, TypeError, ValueError):
        return False
    return all(math.isfinite(v) and v >= 0.0 for v in vals)


def all_finite(*values) -> bool:
    return bool(np.all(np.isfinite(np.asarray(values, dtype=float))))


def axis_errors(bound: dict, x_extent: float, y_extent: float, extra_m: float) -> tuple[float, float]:
    """(e_x, e_y): a per-axis error at the farthest |x|, |y| of a region -- V2/V3's
    e_.,x = along + |y|·yaw + term, e_.,y = lateral + |x|·yaw + term."""
    yaw = float(bound["yaw_rad"])
    return (float(bound["along_m"]) + y_extent * yaw + extra_m,
            float(bound["lateral_m"]) + x_extent * yaw + extra_m)


@dataclass
class PalletPocketCheck:
    pallet_geometry: object
    blades_rear: tuple  # rear-axle frame (x0, x1, y0, y1) per blade
    body_front_m: float
    body_rear_m: float
    body_half_width_m: float
    insertion_depth_m: float
    base_z_m: float
    base_from_optical: tuple  # (R, t) optical -> base_link at lift 0
    rear_axle_offset_m: float  # base_link x of the rear axle is -offset
    camera: DepthCamera
    stopping: StoppingModel
    bounds: object  # forklift_core.perception.near_field_bounds.NearFieldBounds
    config: V11Config = field(default_factory=V11Config)
    latency_band_s: tuple | None = None  # (L_lo, L_hi) of the bounds file, for Δ_L (V2)
    noise_seed: int = 0
    obstacle: bool = False
    obstacle_points: list = field(default_factory=list)
    records: list = field(default_factory=list)
    events: list = field(default_factory=list)
    last_hash: str | None = None
    last_seen_s: float = -math.inf  # the latest new render stamp (newness)
    last_new_s: float = -math.inf  # the latest frame placed and checked (freshness: placed frames only)

    def __post_init__(self) -> None:
        g, cfg = self.pallet_geometry, self.config
        half_depth = g.overall_depth_m / 2
        # Pallet solids in the pallet frame (face at x = 0).
        self.solids = [Box((b.centre_m[0] + half_depth, b.centre_m[1], b.centre_m[2]),
                           tuple(float(v) / 2 for v in b.size_m)) for b in pallet_boxes(g)]
        self.floor = Box((0.0, 0.0, -0.5), (50.0, 50.0, 0.5))
        parts = self._pocket_parts() + [self._body_band()]
        self.volume = InsertionVolume(parts, voxel_m=cfg.voxel_m, halo_m=cfg.halo_m)
        self._reset_store()
        self.region = (-cfg.band_along_m, g.overall_depth_m + cfg.band_along_m,
                       -(g.overall_width_m / 2 + cfg.band_across_m), g.overall_width_m / 2 + cfg.band_across_m)
        self.rng = np.random.default_rng([self.noise_seed, 13])
        self.real_stop = StoppingModel(self.stopping.latency_s, self.stopping.decel_mps2, cfg.envelope_m)
        self.l_hi = float(self.bounds.max_latency_s)
        self.align_s = float(self.bounds.align_latency_s)
        low = float(self.latency_band_s[0]) if self.latency_band_s else self.align_s
        self.delta_l = max(self.l_hi - self.align_s, self.align_s - low)
        v = cfg.voxel_m
        idx, centres = self.volume.centres(evaluated=True)
        in_v = self.volume.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
        self._x_extent = float(np.abs(centres[in_v, 0]).max() + v / 2)  # V's farthest |x|, |y| (V2, V4)
        self._y_extent = float(np.abs(centres[in_v, 1]).max() + v / 2)

    # V ----------------------------------------------------------------------
    def _pocket_parts(self) -> list:
        """V7: each pocket between its walls (only voxels wholly inside), from band_ahead in
        front of the face to the insertion depth + the real stop + the e_d room."""
        g, cfg, v = self.pallet_geometry, self.config, self.config.voxel_m
        ys = g.block_centres_y_m()
        widths = g.block_widths_m
        x0 = -cfg.band_ahead_m
        x1 = self.insertion_depth_m + cfg.real_stop_extra_m + cfg.demand_extra_m
        parts = []
        for k in range(len(ys) - 1):
            lo = ys[k] + widths[k] / 2
            hi = ys[k + 1] - widths[k + 1] / 2
            lo_in = math.ceil(lo / v - 1e-9) * v  # whole voxels inside the walls
            hi_in = math.floor(hi / v + 1e-9) * v
            if hi_in - lo_in < v - 1e-12:
                continue
            parts.append(Box(((x0 + x1) / 2, (lo_in + hi_in) / 2, sum(FORK_Z_M) / 2),
                             ((x1 - x0) / 2, (hi_in - lo_in) / 2, (FORK_Z_M[1] - FORK_Z_M[0]) / 2)))
        return parts

    def _body_band(self) -> Box:
        cfg = self.config
        x0 = -cfg.band_along_m
        half_w = self.body_half_width_m + cfg.body_lateral_m + cfg.body_slack_m
        return Box((x0 / 2, 0.0, (SCOPE_FLOOR_M + H_DET_M) / 2),
                   (-x0 / 2, half_w, (H_DET_M - SCOPE_FLOOR_M) / 2))

    def _reset_store(self) -> None:
        shape = self.volume.shape
        self.certified = np.zeros(shape, dtype=bool)
        self.best_k = np.full(shape + (2,), 10**6, dtype=np.int64)  # (kx, ky) of the smallest certification
        self.cert_frame = np.full(shape, -1, dtype=np.int64)
        self.last_cert_s = np.full(shape, -np.inf)
        self.last_code = np.zeros(shape, dtype=np.int8)

    # Frames -------------------------------------------------------------------
    def optical_from_pallet(self, observation, lift_m: float):
        """(R, t) pallet frame -> optical through the frame's own observation (V1: the base
        pose cancels)."""
        ox, oy, yaw = pallet_from_base(observation)
        c, s = math.cos(yaw), math.sin(yaw)
        r_bp = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # pallet -> base (rotation)
        t_bp = np.array([ox, oy, -self.base_z_m])  # pallet z from the floor, base z from base_link
        r_bo, t_bo = (np.asarray(a, dtype=float) for a in self.base_from_optical)
        t_bo = t_bo + np.array([0.0, 0.0, lift_m])
        r_ob, t_ob = r_bo.T, -r_bo.T @ t_bo
        return r_ob @ r_bp, r_ob @ t_bp + t_ob

    def frame_errors(self, bound: dict, motion: tuple) -> tuple[float, float]:
        """(e_c,x, e_c,y) at V's farthest point: the observation bound per axis, the pallet
        term and m_k = Δ_L (v̄ + ω̄ ρ) (V2), ρ the rear axle to V's farthest point."""
        vbar, wbar, rho = (float(v) for v in motion)
        m_k = self.delta_l * (abs(vbar) + abs(wbar) * rho)
        return axis_errors(bound, self._x_extent, self._y_extent, PALLET_TERM_M + m_k)

    def add_frame(self, stamp_s: float, depth_m: np.ndarray, observation, bound: dict | None, lift_m: float,
                  motion: tuple = (0.0, 0.0, 0.0), truck_pallet=None, carried=None, standing=None) -> dict:
        """One depth frame. observation/bound: the tracker's accepted observation in this frame
        (None: the frame is only an obstacle check, V5 -- then truck_pallet, the base pose in the
        pallet frame by the carried estimate, places it and carried = (the latest accepted
        observation's bound, its stamp) gives that estimate's e_d; standing = (d0, φ0) adds the
        residual-motion bound of a frame placed by the anchor pose while standing, V13). motion:
        (v̄, ω̄, ρ) over the frame's pixel-time window. A non-finite observation, bound or motion
        certifies nothing."""
        # A frame is new by its render stamp: a re-read of the same render carries the same stamp,
        # and a standing camera may render identical pixels at a new time.
        if not math.isfinite(float(stamp_s)) or float(stamp_s) <= self.last_seen_s + 1e-9:
            rec = {"time_s": float(stamp_s), "new": False}
            self.records.append(rec)
            return rec
        raw = np.ascontiguousarray(np.asarray(depth_m, dtype=np.float32))
        self.last_hash = hashlib.sha256(raw.tobytes()).hexdigest()
        self.last_seen_s = float(stamp_s)
        depth = np.asarray(depth_m, dtype=float).copy()
        finite = np.isfinite(depth) & (depth > 0)
        k_sigma = self.camera.sigma_k
        depth[finite] += np.clip(self.rng.normal(0.0, 1.0, int(finite.sum())), -k_sigma, k_sigma) * (
            self.camera.sigma_a * depth[finite] ** 2)
        accepted = (observation is not None and bound_ok(bound) and all_finite(*motion, lift_m)
                    and all_finite(*pallet_from_base(observation)))
        errors_at = None
        if accepted:
            rot, trans = self.optical_from_pallet(observation, lift_m)
            e_x, e_y = self.frame_errors(bound, motion)
            errors_at = self._frame_errors_at(bound, motion)
        elif truck_pallet is not None and all_finite(*truck_pallet, lift_m) and carried is not None:
            errors_at = self._carried_errors_at(truck_pallet, carried, stamp_s, standing)
        if errors_at is None:  # not placed: no obstacle check, so not fresh either
            rec = {"time_s": float(stamp_s), "new": True, "accepted": False, "placed": False}
            self.records.append(rec)
            return rec
        self.last_new_s = float(stamp_s)
        if not accepted:
            rot, trans = self._optical_from_truck(truck_pallet, lift_m)
        idx, centres = self.volume.centres(evaluated=True)
        in_v = self.volume.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
        pts = centres @ rot.T + trans
        z = pts[:, 2]
        cam = self.camera
        with np.errstate(divide="ignore", invalid="ignore"):
            u = np.round(cam.fx * pts[:, 0] / z + cam.cx)
            w = np.round(cam.fy * pts[:, 1] / z + cam.cy)
        inside = (z > 0) & (u >= 0) & (u < cam.width) & (w >= 0) & (w < cam.height)
        ui = np.where(inside, u, 0).astype(np.int64)
        wi = np.where(inside, w, 0).astype(np.int64)
        measured = np.where(inside, depth[wi, ui], np.nan)
        valid = inside & np.isfinite(measured) & (measured > 0)
        margin = cam.margin_m(z)
        free = valid & (measured > z + margin)
        front = valid & (measured < z - margin) & in_v
        latched = self.obstacle
        obstacle_count = self._obstacle_check(stamp_s, rot, trans, ui, wi, measured, front, errors_at,
                                              observation if accepted else None, truck_pallet, lift_m)
        rec = {"time_s": float(stamp_s), "new": True, "accepted": accepted, "obstacle": obstacle_count}
        if self.obstacle and not latched:
            self.invalidate("obstacle", stamp_s)  # V6: the latch drops the store; nothing certifies after it
        if accepted and not self.obstacle:
            v = self.config.voxel_m
            kx = max(1, math.ceil(e_x / v - 1e-9))
            ky = max(1, math.ceil(e_y / v - 1e-9))
            rec.update(e_c=[e_x, e_y], k=[kx, ky])
            if max(kx, ky) * v > self.config.halo_m + 1e-12:
                rec["refused"] = "erosion_beyond_halo"
            else:
                free_grid = np.zeros(self.volume.shape, dtype=bool)
                free_grid[idx[free, 0], idx[free, 1], idx[free, 2]] = True
                cert = box_filter_xy(free_grid, kx, ky) & self.volume.in_v
                frame_no = sum(1 for r in self.records if r.get("accepted"))
                better = cert & ((kx + ky) < self.best_k.sum(axis=-1))
                self.best_k[better] = (kx, ky)
                self.cert_frame[better] = frame_no
                self.certified |= cert
                self.last_cert_s[cert] = float(stamp_s)
                rec["certified"] = int(cert.sum())
            seen = inside & in_v
            code = np.full(len(idx), 5, dtype=np.int8)
            code[free] = 2
            code[front] = 3
            si = idx[seen]
            self.last_code[si[:, 0], si[:, 1], si[:, 2]] = code[seen]
            if "certified" in rec:
                self.last_code[cert] = 1
        self.records.append(rec)
        return rec

    def _optical_from_truck(self, truck_pallet, lift_m: float):
        """(R, t) pallet -> optical with the truck's base pose in the pallet frame (V5's frames
        without an accepted observation)."""
        x, y, yaw = truck_pallet
        c, s = math.cos(yaw), math.sin(yaw)
        r_pb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # base -> pallet
        r_bp = r_pb.T
        t_bp = -r_bp @ np.array([x, y, self.base_z_m])
        r_bo, t_bo = (np.asarray(a, dtype=float) for a in self.base_from_optical)
        t_bo = t_bo + np.array([0.0, 0.0, lift_m])
        r_ob, t_ob = r_bo.T, -r_bo.T @ t_bo
        return r_ob @ r_bp, r_ob @ t_bp + t_ob

    def _frame_errors_at(self, bound: dict, motion: tuple):
        """Per-point e_c (V2) of an accepted frame: along + |y|·yaw, lateral + |x|·yaw, + term + m_k."""
        vbar, wbar, rho = (float(v) for v in motion)
        extra = PALLET_TERM_M + self.delta_l * (abs(vbar) + abs(wbar) * rho)
        yaw = float(bound["yaw_rad"])

        def at(pts):
            return (float(bound["along_m"]) + np.abs(pts[:, 1]) * yaw + extra,
                    float(bound["lateral_m"]) + np.abs(pts[:, 0]) * yaw + extra)
        return at

    def _carried_errors_at(self, truck_pallet, carried, stamp_s: float, standing=None):
        """Per-point e_d (V3) of the carried estimate that places a frame without an accepted
        observation: the latest bound + e(a) + ρ_p ψ(a) + term, ρ_p from the rear axle; plus
        d0 + ρ_p φ0 for a frame placed by the anchor pose while standing (V13 -- standing =
        (d0, φ0)); None when the bound or the standing bound is unusable or the age has no
        odometry row (the frame cannot be placed)."""
        bound, latest_stamp_s = carried
        if not bound_ok(bound) or not all_finite(latest_stamp_s):
            return None
        d0, phi0 = standing if standing is not None else (0.0, 0.0)
        if not (all_finite(d0, phi0) and d0 >= 0.0 and phi0 >= 0.0):
            return None
        odo = self.demand_terms(float(stamp_s) - (float(latest_stamp_s) - self.l_hi))
        if odo is None:
            return None
        e, psi = odo
        x, y, yaw_t = truck_pallet
        rear = (x - self.rear_axle_offset_m * math.cos(yaw_t), y - self.rear_axle_offset_m * math.sin(yaw_t))
        yaw = float(bound["yaw_rad"])

        def at(pts):
            rho = np.hypot(pts[:, 0] - rear[0], pts[:, 1] - rear[1])
            extra = e + rho * psi + PALLET_TERM_M + d0 + rho * phi0
            return (float(bound["along_m"]) + np.abs(pts[:, 1]) * yaw + extra,
                    float(bound["lateral_m"]) + np.abs(pts[:, 0]) * yaw + extra)
        return at

    def _obstacle_check(self, stamp_s, rot, trans, ui, wi, measured, front, errors_at, observation, truck_pallet,
                        lift_m) -> int:
        """V5: a return in front of a V voxel that lies off the pallet surface (placed by this
        frame's observation with e_c, or by the carried estimate with e_d, both at the return's
        own position) and off the floor and the truck's own parts is an obstacle (latched).
        Tolerance kσ + the estimate share + the error along the surface normal (front face e_x,
        side faces e_y, none for horizontal)."""
        if not front.any():
            return 0
        cam = self.camera
        ret_opt = np.column_stack(((ui[front] - cam.cx) / cam.fx, (wi[front] - cam.cy) / cam.fy,
                                   np.ones(int(front.sum())))) * measured[front][:, None]
        ret = (ret_opt - trans) @ rot  # pallet frame
        base_tol = cam.margin_m(measured[front]) + self.config.surface_estimate_m
        e_x, e_y = errors_at(ret)
        on = np.zeros(len(ret), dtype=bool)
        for b in self.solids:
            tx, ty, tz = base_tol + e_x, base_tol + e_y, base_tol
            on |= ((np.abs(ret[:, 0] - b.center[0]) <= b.half[0] + tx)
                   & (np.abs(ret[:, 1] - b.center[1]) <= b.half[1] + ty)
                   & (np.abs(ret[:, 2] - b.center[2]) <= b.half[2] + tz))
        on |= ret[:, 2] <= base_tol  # the floor
        truck = truck_pallet if truck_pallet is not None else self._truck_from_observation(observation)
        for b in self.own_boxes(truck, lift_m):
            c, s = math.cos(b.yaw), math.sin(b.yaw)
            dx, dy = ret[:, 0] - b.center[0], ret[:, 1] - b.center[1]
            on |= ((np.abs(dx * c + dy * s) <= b.half[0] + base_tol) & (np.abs(-dx * s + dy * c) <= b.half[1] + base_tol)
                   & (np.abs(ret[:, 2] - b.center[2]) <= b.half[2] + base_tol))
        count = int((~on).sum())
        if count:
            self.obstacle = True
            self.obstacle_points.append({"time_s": float(stamp_s), "points": ret[~on][:20].tolist()})
        return count

    def _truck_from_observation(self, observation):
        """The truck's base pose in the pallet frame of an observation (base_link origin)."""
        return pose_in_frame(pallet_from_base(observation), (0.0, 0.0, 0.0))

    def invalidate(self, reason: str, now_s: float) -> None:
        """V6: drop the whole store (mismatch, unbounded, loss end, a new episode, an obstacle)."""
        self._reset_store()
        self.events.append({"time_s": float(now_s), "event": "invalidated", "reason": reason})

    # Per tick ----------------------------------------------------------------
    def own_boxes(self, pose_p, lift_m: float) -> list:
        """The blades and the carriage face (0.02 m deep) at a base pose in the pallet frame."""
        x, y, yaw = pose_p
        c, s = math.cos(yaw), math.sin(yaw)
        off = self.rear_axle_offset_m
        out = []
        for x0, x1, y0, y1 in self.blades_rear:
            u, v = (x0 + x1) / 2 - off, (y0 + y1) / 2
            out.append(Box((x + u * c - v * s, y + u * s + v * c, 0.04 + lift_m),
                           ((x1 - x0) / 2, (y1 - y0) / 2, 0.012), yaw))
        u = self.body_front_m - 0.01 - off
        out.append(Box((x + u * c, y + u * s, 0.25 + lift_m), (0.01, 0.235, 0.21), yaw))
        return out

    def demand_terms(self, age_s: float):
        """(e(a), ψ(a)) of the odometry table for the carried estimate's age, or None when the
        table has no row for it (V3: pocket_unbounded)."""
        return self.bounds.odometry(max(0.0, age_s))

    def demand_errors(self, bound: dict, odo, rear_p, rect) -> tuple[float, float]:
        """(e_d,x, e_d,y) of one truck box (V3): at each corner (x_p, y_p) of the rear-axle frame
        rectangle rect = (x0, x1, y0, y1) placed at rear_p in the pallet frame, e_d,x = along +
        |y_p|·yaw + e(a) + ρ_p ψ(a) + term and e_d,y = lateral + |x_p|·yaw + the same, ρ_p the
        corner's distance from the rear axle; the largest over the corners per axis."""
        e, psi = odo
        x, y, yaw_t = rear_p
        c, s = math.cos(yaw_t), math.sin(yaw_t)
        yaw = float(bound["yaw_rad"])
        x0, x1, y0, y1 = rect
        e_x = e_y = -math.inf
        for u in (x0, x1):
            for w in (y0, y1):
                xp, yp = x + u * c - w * s, y + u * s + w * c
                extra = e + math.hypot(u, w) * psi + PALLET_TERM_M
                e_x = max(e_x, float(bound["along_m"]) + abs(yp) * yaw + extra)
                e_y = max(e_y, float(bound["lateral_m"]) + abs(xp) * yaw + extra)
        return e_x, e_y

    def truck_rects(self) -> list:
        """(name, rear-axle frame rectangle (x0, x1, y0, y1), z range, is body) of the truck's boxes
        below h_det: the body (chassis to the carriage when carriage_depth_m is set, else to
        the carriage face), the carriage and the blades (z at lift 0)."""
        depth = self.config.carriage_depth_m
        chassis_front = self.body_front_m - (depth or 0.0)
        hw = self.body_half_width_m
        out = [("body", (-self.body_rear_m, chassis_front, -hw, hw), True)]
        if depth:
            cw = self.config.carriage_half_width_m
            out.append(("carriage", (self.body_front_m - depth, self.body_front_m, -cw, cw), True))
        for x0, x1, y0, y1 in self.blades_rear:
            out.append(("blade_left" if (y0 + y1) > 0 else "blade_right", (x0, x1, y0, y1), False))
        return out

    def truck_boxes(self, rear_p, lift_m: float, bound: dict, odo) -> tuple[list, list]:
        """Body (scope floor .. h_det), carriage and blades at the lift around a rear-axle pose in
        the pallet frame, each inflated per axis by its own e_d (V3, at the box's corners) and
        sideways by the envelope (along the axis the envelope is in the stop length). Returns
        (boxes, their e_d)."""
        x, y, yaw = rear_p
        c, s = math.cos(yaw), math.sin(yaw)
        sin_t = abs(math.sin(yaw))
        env = self.config.envelope_m
        boxes, errors = [], []
        z_blade = (max(0.028 + lift_m - 0.01, SCOPE_FLOOR_M), 0.052 + lift_m + 0.01)
        for _, rect, body in self.truck_rects():
            e_d = self.demand_errors(bound, odo, rear_p, rect)
            e_x, e_y = e_d
            along, across = e_x + e_y * sin_t, e_y + e_x * sin_t  # V3 as written (>= the exact projection)
            x0, x1, y0, y1 = rect
            u, w = (x0 + x1) / 2, (y0 + y1) / 2
            z0, z1 = (SCOPE_FLOOR_M, H_DET_M) if body else z_blade
            boxes.append(Box((x + u * c - w * s, y + u * s + w * c, (z0 + z1) / 2),
                             ((x1 - x0) / 2 + along, (y1 - y0) / 2 + across + env, (z1 - z0) / 2), yaw))
            errors.append(e_d)
        return boxes, errors

    def contained(self, box: Box, body: bool) -> bool:
        """Every voxel the box overlaps inside the region is in V and certified; the body is
        not asked within face_cut_m of the face, inside the face width (V8)."""
        if self.obstacle:
            return False
        if not all_finite(*box.center, *box.half, box.yaw):
            return False
        if not self._meets_region(box):
            return True  # wholly outside the region: the grid's business (as v10)
        idx, leaves = overlapping_voxels(self.volume, box, self.region)
        v = self.volume.voxel_m
        lo = self.volume.lo
        vx0 = lo[0] + idx[:, 0] * v
        vy0 = lo[1] + idx[:, 1] * v
        x0, x1, y0, y1 = self.region
        keep = (vx0 + v > x0) & (vx0 < x1) & (vy0 + v > y0) & (vy0 < y1)
        if body:
            half_face = self.pallet_geometry.overall_width_m / 2
            cut = (vx0 + v > -self.config.face_cut_m) & (np.abs(vy0 + v / 2) <= half_face)
            keep &= ~cut
        idx = idx[keep]
        if leaves and self._box_leaves_grid(box):
            return False
        if not len(idx):
            return True
        in_v = self.volume.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
        cert = self.certified[idx[:, 0], idx[:, 1], idx[:, 2]]
        return bool((in_v & cert).all())

    def _meets_region(self, box: Box) -> bool:
        """Whether the box's xy footprint overlaps the region (2D SAT, as v10)."""
        x0, x1, y0, y1 = self.region
        c, s = math.cos(box.yaw), math.sin(box.yaw)
        hx, hy, _ = box.half
        ex, ey = abs(c) * hx + abs(s) * hy, abs(s) * hx + abs(c) * hy
        if box.center[0] + ex < x0 or box.center[0] - ex > x1 or box.center[1] + ey < y0 or box.center[1] - ey > y1:
            return False
        rx, ry = (x1 - x0) / 2, (y1 - y0) / 2
        dx, dy = box.center[0] - (x0 + x1) / 2, box.center[1] - (y0 + y1) / 2
        return (abs(dx * c + dy * s) <= hx + rx * abs(c) + ry * abs(s)
                and abs(-dx * s + dy * c) <= hy + rx * abs(s) + ry * abs(c))

    def _explain(self, box: Box, body: bool) -> dict:
        """The asked voxels a box fails on: outside V or never certified, with examples."""
        idx, leaves = overlapping_voxels(self.volume, box, self.region)
        v, lo = self.volume.voxel_m, self.volume.lo
        vx0 = lo[0] + idx[:, 0] * v
        vy0 = lo[1] + idx[:, 1] * v
        x0, x1, y0, y1 = self.region
        keep = (vx0 + v > x0) & (vx0 < x1) & (vy0 + v > y0) & (vy0 < y1)
        if body:
            keep &= ~((vx0 + v > -self.config.face_cut_m)
                      & (np.abs(vy0 + v / 2) <= self.pallet_geometry.overall_width_m / 2))
        idx = idx[keep]
        in_v = self.volume.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
        cert = self.certified[idx[:, 0], idx[:, 1], idx[:, 2]]
        bad = idx[~(in_v & cert)]
        centres = lo + (bad + 0.5) * v
        return {"leaves": bool(leaves), "asked": int(len(idx)), "outside_v": int((~in_v).sum()),
                "uncertified": int((in_v & ~cert).sum()), "examples": np.round(centres[:12], 3).tolist(),
                "examples_code": [int(self.last_code[tuple(b)]) for b in bad[:12]]}

    def _box_leaves_grid(self, box: Box) -> bool:
        """Whether the part of the box inside the region leaves the voxel grid in x, y (1e-9
        tolerance). Height is not asked: below the scope floor and above h_det nothing is in
        scope (D0), and the grid spans exactly that band."""
        c, s = math.cos(box.yaw), math.sin(box.yaw)
        hx, hy, _ = box.half
        corners = [(box.center[0] + a * hx * c - b * hy * s, box.center[1] + a * hx * s + b * hy * c)
                   for a in (-1, 1) for b in (-1, 1)]
        x0, x1, y0, y1 = self.region
        xs = np.clip([p[0] for p in corners], x0, x1)
        ys = np.clip([p[1] for p in corners], y0, y1)
        vol = self.volume
        top = vol.lo + np.array(vol.shape) * vol.voxel_m
        tol = 1e-9
        return not (xs.min() >= vol.lo[0] - tol and xs.max() <= top[0] + tol
                    and ys.min() >= vol.lo[1] - tol and ys.max() <= top[1] + tol)

    def limit(self, now_s: float, rear_p, latest_bound: dict, latest_stamp_s: float, curvature_inv_m: float,
              direction: int, speed_mps: float, lift_m: float) -> tuple[float, str]:
        """The speed the stop arc allows: rear_p is the rear axle in the pallet frame by the
        tracker's carried estimate, latest_bound/latest_stamp_s its latest accepted observation.
        A non-finite pose, bound or motion input is unbounded: nothing can be placed."""
        if self.obstacle:
            return 0.0, "pocket_obstacle"
        if not all_finite(now_s, *rear_p, latest_stamp_s, curvature_inv_m, speed_mps, lift_m) or not bound_ok(latest_bound):
            return 0.0, "pocket_unbounded"
        if now_s - (self.last_new_s - self.l_hi) > self.config.frame_max_age_s:
            return 0.0, "depth_stale"
        age = now_s - (latest_stamp_s - self.l_hi)
        odo = self.demand_terms(age)
        if odo is None:
            return 0.0, "pocket_unbounded"
        length = self.real_stop.distance_m(abs(speed_mps)) + self.config.sample_m
        samples, arc = arc_poses(tuple(rear_p), curvature_inv_m, 1 if direction >= 0 else -1, length,
                                 self.config.sample_m)
        verified = 0.0
        rects = self.truck_rects()
        for pose, s in zip(samples, arc, strict=True):
            boxes, errors = self.truck_boxes(pose, lift_m, latest_bound, odo)
            failing = [k for k, b in enumerate(boxes) if not self.contained(b, body=rects[k][2])]
            if failing:
                k = failing[0]
                self.last_fail = {"box": rects[k][0], "arc_s": float(s),
                                  "pose_pallet": [round(float(c), 4) for c in pose],
                                  "e_d": list(errors[k]), "age_s": float(age),
                                  **self._explain(boxes[k], rects[k][2])}
                break
            verified = float(s)
        else:
            return math.inf, "ok"
        allowed = self.real_stop.speed_for(verified)
        return allowed, "ok" if allowed > 0 else "pocket_unverified"

    def summary(self) -> dict:
        accepted = [r for r in self.records if r.get("accepted")]
        best = self.best_k[self.certified]
        return {
            "rules": RULES,
            "obstacle": self.obstacle,
            "obstacle_points": self.obstacle_points[:5],
            "last_fail": getattr(self, "last_fail", None),
            "frames": len(self.records),
            "new_frames": sum(1 for r in self.records if r.get("new")),
            "accepted_frames": len(accepted),
            "refused_frames": sum(1 for r in accepted if r.get("refused")),
            "certified_voxels": int(self.certified.sum()),
            "voxels": int(self.volume.in_v.sum()),
            "best_erosion": {f"{kx},{ky}": int(n) for (kx, ky), n in
                             zip(*np.unique(best, axis=0, return_counts=True), strict=True)} if len(best) else {},
            "events": self.events,
            "config": self.config.__dict__,
        }


__all__ = ["PalletPocketCheck", "RULES", "V11Config", "box_filter_xy", "pallet_from_base", "pose_in_frame"]
