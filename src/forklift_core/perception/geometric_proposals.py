# src/forklift_core/perception/geometric_proposals.py
"""The geometric fallback proposer: one mask per depth-connected object (stage 3 plan, task 2; design §4, §10).

Depth only (no RGB, no truth): valid depth back-projected to base_link within ``range_max_m``; the floor band
z <= a + b r^2 removed (r the camera's horizontal range; depth noise grows with r^2); 8-neighbour connected
components linked where neighbouring axial depths differ by at most c + d * depth; components without a vertical
face (instance_fit.fit_faces) dropped; invalid-depth holes enclosed by a component (up to ``hole_radius_px``)
filled so the fitter's erosion does not grow interior dropout into the mask (Codex); a component wider than
``width_max_m`` is reported as an ``oversize`` failure (an object is there, so a target in reach asks to look
again), not proposed. At most ``max_proposals`` (design §8), largest first; the rest are counted.

Depth cannot split touching or stacked pallets (Codex), so every geometric proposal carries the warning
``geometric_unsplit``: its candidates stay ``unconfirmed`` (instance_fit._identity), never selected automatically
(design §7.5) -- the learned proposer decides identity and splits instances.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from forklift_core.perception.instance_fit import FitFailure, FitParams, camera_xyz, fit_faces, pixel_rays
from forklift_core.perception.instance_proposals import InstanceProposal

WARNING = "geometric_unsplit"


@dataclass(frozen=True)
class GeometricParams:
    range_max_m: float = 4.5
    floor_a_m: float = 0.015         # floor band z <= a + b r^2
    floor_b_per_m: float = 0.002
    jump_c_m: float = 0.03           # depth link c + d * depth
    jump_d: float = 0.02
    min_pixels: int = 150
    width_max_m: float = 2.0
    hole_radius_px: int = 4
    max_proposals: int = 8


@dataclass(frozen=True)
class GeometricResult:
    proposals: list
    failures: list                   # FitFailure(reason "oversize")
    dropped: int = 0                 # components past max_proposals
    warnings: dict = field(default_factory=dict)  # proposal_id -> [warning]


def _components(keep, depth, params):
    """Labels (-1 outside) of keep's 8-neighbour components linked by the depth rule (numpy union-find)."""
    h, w = keep.shape
    idx = np.arange(h * w).reshape(h, w)
    tol = params.jump_c_m + params.jump_d * np.where(np.isfinite(depth), depth, 0.0)
    a_list, b_list = [], []
    for dv, du in ((0, 1), (1, 0), (1, 1), (1, -1)):
        v0, v1 = (0, h - dv)
        u0, u1 = (max(0, -du), w - max(0, du))
        a = (slice(v0, v1), slice(u0, u1))
        b = (slice(v0 + dv, v1 + dv), slice(u0 + du, u1 + du))
        link = keep[a] & keep[b] & (np.abs(depth[a] - depth[b]) <= np.minimum(tol[a], tol[b]))
        a_list.append(idx[a][link])
        b_list.append(idx[b][link])
    ea, eb = np.concatenate(a_list), np.concatenate(b_list)
    parent = np.arange(h * w)
    while True:
        lo = np.minimum(parent[ea], parent[eb])
        before = parent.copy()
        np.minimum.at(parent, parent[ea], lo)
        np.minimum.at(parent, parent[eb], lo)
        while True:  # pointer jumping
            jumped = parent[parent]
            if np.array_equal(jumped, parent):
                break
            parent = jumped
        if np.array_equal(parent, before):
            break
    labels = np.where(keep.ravel(), parent, -1).reshape(h, w)
    return labels


def _fill_holes(mask, invalid, radius):
    """Invalid-depth pixels inside the mask's outline (within ``radius`` px of it on both sides of a row or
    column) join the mask; valid-depth pixels (a pocket seen through) never do."""
    if radius <= 0:
        return mask
    out = mask.copy()
    for axis in (0, 1):
        m = np.moveaxis(mask, axis, 0)
        inv = np.moveaxis(invalid, axis, 0)
        fill = np.zeros_like(m)
        for k in range(1, 2 * radius + 1):
            # a run of invalid pixels up to 2*radius long bounded by mask pixels on both ends
            left = np.zeros_like(m)
            left[k:] = m[:-k]
            right = np.zeros_like(m)
            right[:-k] = m[k:]
            fill |= left & right & inv
        out |= np.moveaxis(fill, 0, axis)
    return out


def _dilate(mask):
    out = mask.copy()
    out[1:] |= mask[:-1]
    out[:-1] |= mask[1:]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    return out


def _grow_into_band(comp, band, points, faces, cam, params, max_px=15):
    """Floor-band pixels on one of the component's vertical faces (the bottom deck the floor band cut off), joined
    to the component through each other; the floor in front of a face is off its plane and stays out."""
    xy = points[..., :2]
    r = np.hypot(xy[..., 0] - cam[0], xy[..., 1] - cam[1])
    tol = params.floor_a_m + params.floor_b_per_m * r * r
    on_face = np.zeros_like(band)
    comp_xy = xy[comp]
    for f in faces:
        rel = xy - f.centre_xy
        normal = np.abs(rel @ f.normal_xy)
        lateral = rel @ f.left_xy
        span = (comp_xy - f.centre_xy) @ f.left_xy  # the component's own extent along the face
        on_face |= band & (normal <= tol) & (lateral >= span.min() - 0.05) & (lateral <= span.max() + 0.05)
    grown = comp.copy()
    for _ in range(max_px):
        nxt = grown | (_dilate(grown) & on_face)
        if np.array_equal(nxt, grown):
            break
        grown = nxt
    return grown


def _attach_interior(comp, faces, labels, small, points, max_depth_m=1.3, margin_m=0.05):
    """Small components seen through the component's openings -- behind one of its faces by at most a pallet
    depth and inside its lateral extent -- are the same object (the fitter's evidence ownership needs them)."""
    v, u = np.nonzero(comp)
    v0, v1, u0, u1 = v.min(), v.max(), u.min(), u.max()
    comp_xy = points[comp][:, :2]
    out = comp.copy()
    for lab in small:
        part = labels == lab
        pv, pu = np.nonzero(part)
        if pv.min() < v0 or pv.max() > v1 or pu.min() < u0 or pu.max() > u1:
            continue
        xy = points[part][:, :2]
        for f in faces:
            rel = xy - f.centre_xy
            behind = -(rel @ f.normal_xy)  # normal points to the camera
            span = (comp_xy - f.centre_xy) @ f.left_xy
            lateral = rel @ f.left_xy
            if (np.all(behind >= -margin_m) and np.all(behind <= max_depth_m)
                    and np.all(lateral >= span.min() - margin_m) and np.all(lateral <= span.max() + margin_m)):
                out |= part
                break
    return out


def propose(scene, params: GeometricParams = GeometricParams()) -> GeometricResult:
    depth = np.asarray(scene.depth_m, dtype=float)
    valid = np.isfinite(depth) & (depth > 0)
    rays = pixel_rays(scene)
    cam = camera_xyz(scene)
    points = cam + np.where(valid, depth, 0.0)[..., None] * rays
    r = np.hypot(points[..., 0] - cam[0], points[..., 1] - cam[1])
    floor = params.floor_a_m + params.floor_b_per_m * r * r
    near = valid & (r <= params.range_max_m)
    keep = near & (points[..., 2] > floor)
    band = near & ~keep & (points[..., 2] > -0.05)
    labels = _components(keep, depth, params)
    found, failures = [], []
    all_labels, sizes = np.unique(labels[labels >= 0], return_counts=True)
    small = [lab for lab, n in zip(all_labels, sizes) if n < params.min_pixels]
    for lab, n in zip(all_labels, sizes):
        n = int(n)
        if n < params.min_pixels:
            continue
        comp = labels == lab
        pts = points[comp]
        xy = pts[:, :2]
        # horizontal width: the extent across the camera's view of the component's points
        centre = np.median(xy, axis=0)
        extent = float(np.max(np.linalg.norm(xy - centre, axis=1)))
        bearing = np.arctan2(xy[:, 1] - cam[1], xy[:, 0] - cam[0])
        width = float(np.median(np.hypot(xy[:, 0] - cam[0], xy[:, 1] - cam[1])) * (bearing.max() - bearing.min()))
        faces = fit_faces(pts, cam, FitParams(), np.random.default_rng(0))
        if not faces:
            continue
        if width > params.width_max_m:
            failures.append(FitFailure(f"g_oversize_{len(failures)}", "oversize",
                                       (float(centre[0]), float(centre[1])), extent))
            continue
        comp = _grow_into_band(comp, band, points, faces, cam, params)
        comp = _attach_interior(comp, faces, labels, small, points)
        v, u = np.nonzero(comp)
        found.append((n, int(v.min()), int(u.min()), comp))
    found.sort(key=lambda f: (-f[0], f[1], f[2]))
    kept, dropped = found[: params.max_proposals], max(0, len(found) - params.max_proposals)
    kept.sort(key=lambda f: (f[1], f[2]))  # stable reading order
    proposals, warnings = [], {}
    for k, (n, _, _, comp) in enumerate(kept):
        mask = _fill_holes(comp, ~valid, params.hole_radius_px)
        pid = f"g{k}"
        proposals.append(InstanceProposal(pid, mask, min(1.0, n / 5000.0), "geometric"))
        warnings[pid] = [WARNING]
    return GeometricResult(proposals, failures, dropped, warnings)


__all__ = ["GeometricParams", "GeometricResult", "WARNING", "propose"]
_ = math
