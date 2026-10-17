"""Pocket interior free check from one depth image (priority-5 plan D5).

docs/plans/2026-10-04-lidar-obstacle-map.md: a 2D grid cannot tell a foreign
object inside a fork pocket from the pallet, so before insertion the carriage
depth camera must see every 1 cm voxel of the fork stopping volume freed by a
ray that went past it. Voxels are given in the camera's optical frame (z
forward, x right, y down; metres). Per voxel, the pixel its centre projects to
is read:

* depth beyond the voxel (by more than the noise margin): the ray passed
  through -- free;
* depth in front of the voxel: something is between -- if that return lies on
  the estimated pallet surface (within the noise margin) the voxel is merely
  occluded (unobserved); otherwise it is an obstacle;
* no return, an invalid pixel or a projection outside the image: unobserved.

Any obstacle or unobserved voxel rejects the insertion; there is no size
threshold. The noise margin is k * sigma(z) with sigma(z) = a * z^2 (the D435i
datasheet form, assumed). Ray spacing sets the smallest detectable object.
numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DepthCamera:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    sigma_a: float = 0.0036  # sigma(z) = sigma_a * z^2 (assumed, P0a)
    sigma_k: float = 3.0

    def margin_m(self, z):
        return self.sigma_k * self.sigma_a * np.asarray(z, dtype=float) ** 2


@dataclass
class ClearanceResult:
    accepted: bool
    reason: str
    voxels: int
    free: int
    occluded: int
    unobserved: int
    obstacle: int
    obstacle_points: np.ndarray  # (k, 3) optical-frame returns that are not pallet


def box_voxels(center, half, rotation=np.eye(3), voxel_m: float = 0.01) -> np.ndarray:
    """Voxel centres (N, 3) filling an oriented box (centre, half extents, rotation)."""
    half = np.asarray(half, dtype=float)
    axes = [np.arange(-h + voxel_m / 2, h, voxel_m) if h > voxel_m / 2 else np.array([0.0]) for h in half]
    g = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    return g @ np.asarray(rotation).T + np.asarray(center, dtype=float)


def surface_tolerance_m(camera: DepthCamera, returns: np.ndarray) -> np.ndarray:
    """How far a return may lie from the surface it came from: k sigma at the
    true depth (up to measured + k sigma(measured)), stretched from the optical
    axis onto the pixel's ray."""
    r = np.asarray(returns, dtype=float).reshape(-1, 3)
    z = r[:, 2]
    stretch = np.linalg.norm(r, axis=1) / np.where(z > 0, z, 1.0)
    return camera.margin_m(z + camera.margin_m(z)) * stretch


def on_surface(points: np.ndarray, boxes, tolerance) -> np.ndarray:
    """True for points within tolerance (per point) of any box surface or inside it."""
    out = np.zeros(len(points), dtype=bool)
    tol = np.broadcast_to(np.asarray(tolerance, dtype=float), (len(points),))
    for center, half, rotation in boxes:
        local = (points - np.asarray(center)) @ np.asarray(rotation)
        excess = np.abs(local) - np.asarray(half)
        outside = np.linalg.norm(np.maximum(excess, 0.0), axis=1)
        out |= outside <= tol
    return out


def check_pocket_clearance(
    depth_m: np.ndarray,
    camera: DepthCamera,
    voxels_optical: np.ndarray,
    pallet_boxes_optical,
) -> ClearanceResult:
    """Free / occluded / unobserved / obstacle per voxel of the fork volume."""
    v = np.asarray(voxels_optical, dtype=float).reshape(-1, 3)
    depth = np.asarray(depth_m, dtype=float)
    z = v[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.round(camera.fx * v[:, 0] / z + camera.cx).astype(np.int64)
        w = np.round(camera.fy * v[:, 1] / z + camera.cy).astype(np.int64)
    inside = (z > 0) & (u >= 0) & (u < camera.width) & (w >= 0) & (w < camera.height)
    measured = np.full(len(v), np.nan)
    measured[inside] = depth[w[inside], u[inside]]
    valid = inside & np.isfinite(measured) & (measured > 0)
    margin = camera.margin_m(z)
    free = valid & (measured > z + margin)
    front = valid & (measured < z - margin)
    # The return in front of the voxel, back in 3D along the same pixel ray.
    ret = np.zeros((len(v), 3))
    # On the pixel's own ray (not the voxel's): at a grazing face half a pixel
    # moves the point centimetres.
    ret[front] = np.column_stack(((u[front] - camera.cx) / camera.fx, (w[front] - camera.cy) / camera.fy,
                                  np.ones(int(front.sum())))) * measured[front][:, None]
    pallet = np.zeros(len(v), dtype=bool)
    if front.any():
        pallet[front] = on_surface(ret[front], pallet_boxes_optical, surface_tolerance_m(camera, ret[front]))
    occluded = front & pallet
    obstacle = front & ~pallet
    # A return within the margin of the voxel itself cannot be told apart: unobserved.
    unobserved = ~(free | occluded | obstacle)
    n_obs, n_un = int(obstacle.sum()), int(unobserved.sum()) + int(occluded.sum())
    if n_obs:
        reason = "obstacle"
    elif n_un:
        reason = "unobserved"
    else:
        reason = "ok"
    return ClearanceResult(
        reason == "ok", reason, len(v), int(free.sum()), int(occluded.sum()),
        int(unobserved.sum()), n_obs, ret[obstacle],
    )


__all__ = ["ClearanceResult", "DepthCamera", "box_voxels", "check_pocket_clearance", "on_surface",
           "surface_tolerance_m"]
