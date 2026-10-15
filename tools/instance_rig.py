# tools/instance_rig.py
"""CPU first-hit renderer that also reports the instance and base-frame point each
pixel hit. Reuses tools.scene_rig's slab intersection unchanged (that file's hash
is part of recorded Isaac runs). Depth is clean and unquantised;
forklift_core.perception.depth_noise adds the sensor model afterwards."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from forklift_core.perception.scene_dataset import SceneInput
from tools import scene_rig

PERCEPTION_CAMERA = scene_rig.Camera((0.75, 0.0, 0.27), 0.0)
BACKGROUND, DISTRACTOR = -1, -2


@dataclass(frozen=True)
class RenderedScene:
    scene: SceneInput
    instance_ids: NDArray[np.int32]
    points_base: NDArray[np.float64]


def render_instances(instances: Sequence[Sequence[scene_rig.Box]], *, distractors=(), camera=PERCEPTION_CAMERA,
                     intrinsics=None, floor=True, back_wall=True, stamp_ns=2_000_000_000) -> RenderedScene:
    spec = scene_rig._rig_intrinsics() if intrinsics is None else intrinsics
    transform = camera.base_from_optical()
    origin = np.asarray(transform.translation_m, dtype=float)
    rays = scene_rig._rays(spec, transform.rotation)
    depth = np.full((spec.height, spec.width), np.inf)
    ids = np.full(depth.shape, BACKGROUND, dtype=np.int32)
    with np.errstate(divide="ignore", invalid="ignore"):
        if floor:
            t = -origin[2] / rays[..., 2]
            depth = np.fmin(depth, np.where((rays[..., 2] < 0) & (t > 0), t, np.inf))
        if back_wall:
            t = (scene_rig.BACK_WALL_X_M - origin[0]) / rays[..., 0]
            depth = np.fmin(depth, np.where((rays[..., 0] > 0) & (t > 0), t, np.inf))
    labelled = [(DISTRACTOR, b) for b in distractors] + [(k, b) for k, boxes in enumerate(instances) for b in boxes]
    for label, box in labelled:
        hit = scene_rig._box_depth(origin, rays, np.asarray(box.centre_m, float), box.size_m,
                                   scene_rig._yaw_matrix(box.yaw_rad))
        closer = hit < depth
        depth = np.where(closer, hit, depth)
        ids = np.where(closer, label, ids)
    depth = np.where(np.isfinite(depth), depth, np.nan)
    points = origin + depth[..., None] * rays
    scene = SceneInput(rgb=np.zeros((spec.height, spec.width, 3), dtype=np.uint8), depth_m=depth,
                       intrinsics=spec, base_from_optical=transform, stamp_ns=stamp_ns,
                       clock_domain="ros_sim", source_provenance="synthetic")
    return RenderedScene(scene, ids, points)
