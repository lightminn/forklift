# tools/recognition_truth.py
"""Ground truth for recognition scenes: per-face pockets, face exposure, fit truth."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import asdict, dataclass, replace

import numpy as np

from forklift_core.perception.fork_fit import ForkSetup, danger_threshold_m, fork_fit, lane_reachable
from forklift_core.perception.pallet_damage import apply_damage
from forklift_core.perception.pallet_model import PalletSpec, face_layout, spec_boxes
from tools import instance_rig, scene_rig

EXPOSURE_MIN = 0.5
FACE_PLANE_TOL_M = 0.001
SUPPORT_TOL_M = 0.002


def _wrap(angle: float) -> float:
    wrapped = (angle + math.pi) % (2 * math.pi) - math.pi
    return math.pi if wrapped == -math.pi else wrapped


@dataclass(frozen=True)
class Placement:
    x_m: float
    y_m: float
    yaw_rad: float
    support_z_m: float


def instance_boxes(spec: PalletSpec, damage):
    return apply_damage(spec_boxes(spec), damage)


def placed_boxes(boxes, placement: Placement) -> list[scene_rig.Box]:
    lifted = [scene_rig.Box((b.centre_m[0], b.centre_m[1], b.centre_m[2] + placement.support_z_m), tuple(b.size_m))
              for b in boxes]
    return scene_rig.place(lifted, x_m=placement.x_m, y_m=placement.y_m, yaw_rad=placement.yaw_rad)


@dataclass(frozen=True)
class FaceTruth:
    axis: str
    sign: int
    insertion_yaw_rad: float
    left_m: tuple
    right_m: tuple
    pocket_width_m: tuple
    pocket_height_m: float
    accessible: bool
    exposure: float
    fit: dict
    danger_threshold_m: float


@dataclass(frozen=True)
class InstanceTruth:
    instance_id: int
    spec: dict
    known: bool
    layer: int
    support_z_m: float
    support_visible: bool
    lane_reachable: bool
    faces: tuple
    eligible: bool
    damage: list

    def to_json(self) -> dict:
        obj = asdict(self)
        obj["faces"] = [asdict(f) for f in self.faces]
        return obj


def _local(points, placement: Placement):
    c, s = math.cos(placement.yaw_rad), math.sin(placement.yaw_rad)
    d = points - np.array([placement.x_m, placement.y_m, placement.support_z_m])
    return np.stack((c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1], d[..., 2]), axis=-1)


def _face_mask(local, spec: PalletSpec, axis: str, sign: int):
    """Hits on the entry-face plane itself, not on the top or sides near its edges."""
    if axis == "x":
        plane = -spec.overall_depth_m / 2 if sign == 1 else spec.overall_depth_m / 2
        return np.abs(local[..., 0] - plane) <= FACE_PLANE_TOL_M
    plane = -spec.overall_width_m / 2 if sign == 1 else spec.overall_width_m / 2
    return np.abs(local[..., 1] - plane) <= FACE_PLANE_TOL_M


def _top_visible(rendered, m, boxes, top_z) -> bool:
    """Hits on the interior of an upward face of one of instance m's boxes at top_z."""
    pts = rendered.points_base
    inside = np.zeros(pts.shape[:2], bool)
    for box in boxes:
        if abs(box.centre_m[2] + box.size_m[2] / 2 - top_z) > SUPPORT_TOL_M:
            continue
        c, s = math.cos(box.yaw_rad), math.sin(box.yaw_rad)
        dx, dy = pts[..., 0] - box.centre_m[0], pts[..., 1] - box.centre_m[1]
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        inside |= (np.abs(lx) < box.size_m[0] / 2 - SUPPORT_TOL_M) & (np.abs(ly) < box.size_m[1] / 2 - SUPPORT_TOL_M)
    on_top = np.abs(pts[..., 2] - top_z) <= SUPPORT_TOL_M
    # An upward-facing surface can only be hit by a descending ray, so a camera at
    # or below it sees board sides there, never the surface itself.
    camera_z = float(rendered.scene.base_from_optical.translation_m[2])
    descending = rendered.points_base[..., 2] < camera_z
    return bool(((rendered.instance_ids == m) & inside & on_top & descending).any())


def face_load(spec, axis: str, load):
    """The load declaration as seen from the face it is judged on.

    A declaration is measured from the x face: the centre of mass lies at most com_x from it and at
    most com_y to the side. From a y face of a non-standard pallet that is up to width/2 + com_y along
    the forks and up to max(depth/2, com_x - depth/2) to the side -- the declaration gives no lower
    bound on the x distance. Masses and source are kept (dropping the load lost payload_limit, Codex
    2026-10-17). Known shapes are exempt from the support check, so their load is left as given."""
    if load is None or axis == "x" or spec.known:
        return load
    return replace(load, com_x_max_m=spec.overall_width_m / 2 + load.com_y_max_m,
                   com_y_max_m=max(spec.overall_depth_m / 2, load.com_x_max_m - spec.overall_depth_m / 2))


def _faces(spec, placement, setup, load, scene_local, scene_ids, k, solo_local, solo_ids):
    camera = np.asarray(instance_rig.PERCEPTION_CAMERA.xyz_m, dtype=float)
    out = []
    for axis in spec.entry_axes:
        layout = face_layout(spec, axis)
        fit = fork_fit(spec, axis, setup, face_load(spec, axis, load), support_z_m=placement.support_z_m)
        intervals = [layout.opening_intervals_m[i] for i in fit.pair]
        z0, z1 = layout.pocket_band_m
        z = placement.support_z_m + (z0 + z1) / 2
        base = placement.yaw_rad + (0.0 if axis == "x" else math.pi / 2)
        for sign in (1, -1):
            yaw = _wrap(base + (0.0 if sign == 1 else math.pi))
            a = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            left_axis = np.array([-a[1], a[0], 0.0])
            face_centre = np.array([placement.x_m, placement.y_m, 0.0]) - a * layout.depth_m / 2
            # The opposite face sees the layout mirrored: its left axis is the nominal right.
            (r_lat, r_w), (l_lat, l_w) = sorted((sign * (lo + hi) / 2, hi - lo) for lo, hi in intervals)
            left, right = face_centre + left_axis * l_lat, face_centre + left_axis * r_lat
            seen = int((_face_mask(scene_local, spec, axis, sign) & (scene_ids == k)).sum())
            solo = int((_face_mask(solo_local, spec, axis, sign) & (solo_ids == 0)).sum())
            out.append(FaceTruth(
                axis, sign, yaw, (float(left[0]), float(left[1]), z), (float(right[0]), float(right[1]), z),
                (l_w, r_w), z1 - z0, float(-a @ (camera - face_centre)) > 0,
                seen / solo if solo else 0.0, fit.to_json(), danger_threshold_m(fit),
            ))
    return tuple(out)


def scene_truths(instances, setup: ForkSetup, *, distractors: Sequence[scene_rig.Box] = (), layers=None,
                 intrinsics=None):
    placed = [placed_boxes(instance_boxes(s, d), p) for s, p, d, _ in instances]
    rendered = instance_rig.render_instances(placed, distractors=distractors, intrinsics=intrinsics)
    layers = [0] * len(instances) if layers is None else list(layers)
    truths = []
    for k, (spec, placement, damage, load) in enumerate(instances):
        solo = instance_rig.render_instances([placed[k]], floor=False, back_wall=False, intrinsics=intrinsics)
        faces = _faces(spec, placement, setup, load, _local(rendered.points_base, placement), rendered.instance_ids,
                       k, _local(solo.points_base, placement), solo.instance_ids)
        support_visible = layers[k] == 0 or any(
            _top_visible(rendered, m, placed[m], placement.support_z_m)
            for m in range(len(instances)) if layers[m] == layers[k] - 1)
        first = next((f for f in faces if f.accessible), faces[0])
        truths.append(InstanceTruth(
            k, spec.to_json(), spec.known, layers[k], placement.support_z_m, support_visible,
            lane_reachable(spec, first.axis, setup, support_z_m=placement.support_z_m), faces,
            any(f.accessible and f.exposure >= EXPOSURE_MIN for f in faces), [d.to_json() for d in damage],
        ))
    return truths, rendered
