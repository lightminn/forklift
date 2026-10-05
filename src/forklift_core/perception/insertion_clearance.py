"""Depth-certified insertion volume (priority-5 plan D5, with the 2026-10-05 body delta).

The grid cannot tell the pallet from a foreign object inside its pockets or
right in front of its face, so the docking exemption (the estimated pallet
rectangle plus its axial band) is earned with depth instead:

* the volume V, in the insertion frame I (origin at the estimated pallet
  centre, x along the insertion heading, z up), is two fork columns (the
  blades' sweep to the insertion depth plus the stop, widened by the lateral
  margin) and the body band in front of the face (the body's width, between
  the floor and h_det) -- user approval 2026-10-05 for the body part;
* V must not meet the estimated pallet solids: a margin that does is a
  planned collision, refused, never certified (Codex L3c P1);
* every depth frame certifies the 1 cm voxels of V its rays passed through
  (pocket_clearance.check_pocket_clearance), with the frame's measurement
  time; a frame that sees anything off the pallet surface inside V is an
  obstacle, latched;
* a box of the truck's stop volume (body, blades) is contained when every
  voxel its oriented box overlaps (exact z-interval + 2D SAT, Codex L3c P1)
  is in V and was certified within the lifetime; a box part outside the
  exemption region is the grid's business and not asked here.

Frames come from the stand-off capture and the 10 Hz frames on the way in:
near the face the far capture cannot separate a voxel from the face within
the depth noise, a closer frame can (Codex L3c P1).
numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, sin

import numpy as np

from forklift_core.perception.pocket_clearance import DepthCamera, on_surface, surface_tolerance_m


@dataclass(frozen=True)
class Box:
    """Box in the insertion frame: centre, half extents, yaw about z."""

    center: tuple[float, float, float]
    half: tuple[float, float, float]
    yaw: float = 0.0


@dataclass
class InsertionVolume:
    boxes: list  # Box parts of V (axis aligned, yaw 0)
    voxel_m: float = 0.01
    halo_m: float = 0.0  # evaluated around V (x, y) so a moving frame's certification can be eroded
    lo: np.ndarray = field(init=False)
    shape: tuple = field(init=False)
    in_v: np.ndarray = field(init=False)
    in_eval: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        h = np.array([self.halo_m, self.halo_m, 0.0])
        mins = np.min([np.subtract(b.center, b.half) for b in self.boxes], axis=0) - h
        maxs = np.max([np.add(b.center, b.half) for b in self.boxes], axis=0) + h
        v = self.voxel_m
        self.lo = np.floor(mins / v) * v
        self.shape = tuple(int(n) for n in np.ceil((maxs - self.lo) / v - 1e-9))
        self.in_v = np.zeros(self.shape, dtype=bool)
        for b in self.boxes:
            if b.yaw != 0.0:
                raise ValueError("V parts are axis aligned in the insertion frame")
            # Every voxel the part overlaps belongs to V (a superset of the part).
            i0 = np.floor((np.subtract(b.center, b.half) - self.lo) / v + 1e-9).astype(int)
            i1 = np.ceil((np.add(b.center, b.half) - self.lo) / v - 1e-9).astype(int)
            self.in_v[i0[0]:i1[0], i0[1]:i1[1], i0[2]:i1[2]] = True
        k = int(np.ceil(self.halo_m / v - 1e-9))
        self.in_eval = _box_filter(self.in_v, k, np.max) if k > 0 else self.in_v.copy()

    def centres(self, *, evaluated: bool = False) -> tuple[np.ndarray, np.ndarray]:
        """(indices (N, 3), centres (N, 3)) of the voxels of V (or V and its halo)."""
        idx = np.argwhere(self.in_eval if evaluated else self.in_v)
        return idx, self.lo + (idx + 0.5) * self.voxel_m


def _box_filter(mask: np.ndarray, k: int, op) -> np.ndarray:
    """max (dilation) or min (erosion) of a boolean grid over a (2k+1)^2 square in x, y."""
    out = mask.copy()
    for axis in (0, 1):
        n = out.shape[axis]
        pad = [(0, 0)] * out.ndim
        pad[axis] = (k, k)
        padded = np.pad(out, pad, constant_values=False)  # outside the grid: neither free nor marked
        stack = [np.take(padded, range(i, i + n), axis=axis) for i in range(2 * k + 1)]
        out = np.logical_and.reduce(stack) if op is np.min else np.logical_or.reduce(stack)
    return out


def overlapping_voxels(volume: InsertionVolume, box: Box, region=None) -> tuple[np.ndarray, bool]:
    """(indices (k, 3) of every grid voxel the box overlaps, whether it leaves the grid).

    region (x0, x1, y0, y1): only voxels overlapping it are enumerated (the
    rest are not asked); 'leaves' still reports the whole box."""
    v = volume.voxel_m
    c, s = cos(box.yaw), sin(box.yaw)
    hx, hy, hz = box.half
    ex = abs(c) * hx + abs(s) * hy
    ey = abs(s) * hx + abs(c) * hy
    lo = np.array([box.center[0] - ex, box.center[1] - ey, box.center[2] - hz])
    hi = np.array([box.center[0] + ex, box.center[1] + ey, box.center[2] + hz])
    i0 = np.floor((lo - volume.lo) / v + 1e-9).astype(int)
    i1 = np.ceil((hi - volume.lo) / v - 1e-9).astype(int)
    leaves = bool((i0 < 0).any() or (i1 > np.array(volume.shape)).any())
    i0 = np.maximum(i0, 0)
    i1 = np.minimum(i1, volume.shape)
    if region is not None:
        r0 = np.floor((np.array([region[0], region[2]]) - volume.lo[:2]) / v + 1e-9).astype(int)
        r1 = np.ceil((np.array([region[1], region[3]]) - volume.lo[:2]) / v - 1e-9).astype(int)
        i0[:2] = np.maximum(i0[:2], r0)
        i1[:2] = np.minimum(i1[:2], r1)
    if (i1 <= i0).any():
        return np.zeros((0, 3), dtype=int), leaves
    ii, jj, kk = np.meshgrid(*(np.arange(a, b) for a, b in zip(i0, i1)), indexing="ij")
    idx = np.column_stack((ii.ravel(), jj.ravel(), kk.ravel()))
    # Exact 2D SAT between each voxel square and the box rectangle (z is an
    # interval test, done by the index range above).
    cx = volume.lo[0] + (idx[:, 0] + 0.5) * v - box.center[0]
    cy = volume.lo[1] + (idx[:, 1] + 0.5) * v - box.center[1]
    half = v / 2
    proj = half * (abs(c) + abs(s))
    keep = (
        (np.abs(cx) <= ex + half)
        & (np.abs(cy) <= ey + half)
        & (np.abs(cx * c + cy * s) <= hx + proj)
        & (np.abs(-cx * s + cy * c) <= hy + proj)
    )
    return idx[keep], leaves


@dataclass
class ClearanceMemory:
    volume: InsertionVolume
    lifetime_s: float
    certified_s: np.ndarray = field(init=False)  # measurement time of the latest certifying frame
    obstacle: bool = False
    obstacle_points: list = field(default_factory=list)
    frames: list = field(default_factory=list)

    def __post_init__(self) -> None:
        self.certified_s = np.full(self.volume.shape, -np.inf)

    def solid_conflict(self, solids: list) -> int:
        """Voxels of V overlapping any estimated pallet solid (a planned collision)."""
        hits = np.zeros(self.volume.shape, dtype=bool)
        for b in solids:
            idx, _ = overlapping_voxels(self.volume, b)
            if len(idx):
                hits[idx[:, 0], idx[:, 1], idx[:, 2]] = True
        return int((hits & self.volume.in_v).sum())

    def add_frame(self, stamp_s: float, depth_m: np.ndarray, camera: DepthCamera, optical_from_insertion,
                  solids: list, surface_extra_m: float = 0.0, pose_uncertainty_m: float = 0.0) -> dict:
        """Certify V voxels from one depth frame; latch an obstacle if one is seen.

        optical_from_insertion: (R (3, 3), t (3,)) taking insertion-frame points
        to the camera's optical frame at the frame's measurement time.
        """
        rot, trans = (np.asarray(a, dtype=float) for a in optical_from_insertion)
        idx, centres = self.volume.centres(evaluated=True)
        in_v = self.volume.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
        pts = centres @ rot.T + trans
        z = pts[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = np.round(camera.fx * pts[:, 0] / z + camera.cx)
            w = np.round(camera.fy * pts[:, 1] / z + camera.cy)
        inside = (z > 0) & (u >= 0) & (u < camera.width) & (w >= 0) & (w < camera.height)
        ui = np.where(inside, u, 0).astype(np.int64)
        wi = np.where(inside, w, 0).astype(np.int64)
        measured = np.where(inside, np.asarray(depth_m, dtype=float)[wi, ui], np.nan)
        valid = inside & np.isfinite(measured) & (measured > 0)
        # A frame taken while moving is placed with the pose of its reported
        # time, but its pixels may be up to a render period older (L3c v7: 3.5
        # cm at 0.5 m/s): that much more depth before a voxel counts as passed,
        # and that much more distance from a surface before a return is foreign.
        margin = camera.margin_m(z) + pose_uncertainty_m
        free = valid & (measured > z + margin)
        front = valid & (measured < z - margin) & in_v
        obstacle_count = 0
        if front.any():
            # The return lies on the pixel's own ray, not on the voxel's: at a
            # grazing face half a pixel moves it centimetres.
            ret = np.column_stack(((ui[front] - camera.cx) / camera.fx, (wi[front] - camera.cy) / camera.fy,
                                   np.ones(int(front.sum())))) * measured[front][:, None]
            solids_optical = [
                (np.asarray(b.center) @ rot.T + trans, np.asarray(b.half),
                 rot @ np.array([[cos(b.yaw), -sin(b.yaw), 0.0], [sin(b.yaw), cos(b.yaw), 0.0], [0.0, 0.0, 1.0]]))
                for b in solids
            ]
            # The surfaces are where the estimate puts them: a return also counts
            # as on them within the estimate's error (L3c v6: the front
            # stringer's underside 1.7 cm behind its estimate).
            pallet = on_surface(ret, solids_optical,
                                surface_tolerance_m(camera, ret) + surface_extra_m + pose_uncertainty_m)
            obstacle_count = int((~pallet).sum())
            if obstacle_count:
                self.obstacle = True
                self.obstacle_points.append({"time_s": float(stamp_s), "points": ret[~pallet][:20].tolist()})
        # A moving frame's rays may sit up to the pose uncertainty beside where
        # they are placed, and an old ray beside an object passes it: a voxel
        # is certified only when every voxel within that distance (x, y) was
        # passed too (Codex checkpoint P1) -- the halo around V lets V's own
        # edge be judged.
        free_grid = np.zeros(self.volume.shape, dtype=bool)
        free_grid[idx[free, 0], idx[free, 1], idx[free, 2]] = True
        k = int(np.ceil(pose_uncertainty_m / self.volume.voxel_m - 1e-9))
        if k > 0:
            free_grid = _box_filter(free_grid, k, np.min)
        certified = free_grid & self.volume.in_v
        self.certified_s[certified] = float(stamp_s)
        record = {"time_s": float(stamp_s), "voxels": int(in_v.sum()), "free": int(certified.sum()),
                  "in_view": int((inside & in_v).sum()), "obstacle": obstacle_count, "erosion_voxels": k}
        self.frames.append(record)
        return record

    def contained(self, box: Box, now_s: float, region: tuple | None = None) -> bool:
        """Every voxel the box overlaps (inside region, an x/y AABB in I) is in V
        and certified within the lifetime; False once an obstacle was seen."""
        if self.obstacle:
            return False
        if region is not None and not self._box_meets_region(box, region):
            return True  # wholly outside the region: the grid's business
        idx, leaves = overlapping_voxels(self.volume, box, region)
        if region is not None:
            x0, x1, y0, y1 = region
            v = self.volume.voxel_m
            vx0 = self.volume.lo[0] + idx[:, 0] * v
            vy0 = self.volume.lo[1] + idx[:, 1] * v
            # Voxels wholly outside the region are not asked; any overlap is.
            inside = (vx0 + v > x0) & (vx0 < x1) & (vy0 + v > y0) & (vy0 < y1)
            idx = idx[inside]
            if leaves and not self._box_inside_grid_xy(box, region):
                return False
        elif leaves:
            return False
        if not len(idx):
            return True
        in_v = self.volume.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
        fresh = now_s - self.certified_s[idx[:, 0], idx[:, 1], idx[:, 2]] <= self.lifetime_s
        return bool((in_v & fresh).all())

    @staticmethod
    def _box_meets_region(box: Box, region) -> bool:
        """Whether the box's xy footprint overlaps the region (2D SAT)."""
        x0, x1, y0, y1 = region
        c, s = cos(box.yaw), sin(box.yaw)
        hx, hy, _ = box.half
        ex = abs(c) * hx + abs(s) * hy
        ey = abs(s) * hx + abs(c) * hy
        if box.center[0] + ex < x0 or box.center[0] - ex > x1 or box.center[1] + ey < y0 or box.center[1] - ey > y1:
            return False
        rx, ry = (x1 - x0) / 2, (y1 - y0) / 2
        dx, dy = box.center[0] - (x0 + x1) / 2, box.center[1] - (y0 + y1) / 2
        return abs(dx * c + dy * s) <= hx + rx * abs(c) + ry * abs(s) and \
            abs(-dx * s + dy * c) <= hy + rx * abs(s) + ry * abs(c)

    def _box_inside_grid_xy(self, box: Box, region) -> bool:
        """Whether the part of the box inside region stays within the voxel grid."""
        c, s = cos(box.yaw), sin(box.yaw)
        hx, hy, hz = box.half
        corners = [(box.center[0] + a * hx * c - b * hy * s, box.center[1] + a * hx * s + b * hy * c)
                   for a in (-1, 1) for b in (-1, 1)]
        xs = np.clip([p[0] for p in corners], region[0], region[1])
        ys = np.clip([p[1] for p in corners], region[2], region[3])
        v = self.volume
        top = v.lo + np.array(v.shape) * v.voxel_m
        zlo, zhi = box.center[2] - hz, box.center[2] + hz
        return bool(xs.min() >= v.lo[0] and xs.max() <= top[0] and ys.min() >= v.lo[1] and ys.max() <= top[1]
                    and zlo >= v.lo[2] and zhi <= top[2])


__all__ = ["Box", "ClearanceMemory", "InsertionVolume", "overlapping_voxels"]
