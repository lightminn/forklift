"""Why did the pocket detector fail on a saved capture? (G3, closeout plan)

Plan: docs/plans/2026-10-01-g3-detection-diagnosis.md. For each saved
observation attempt of an Isaac run this replays the real detector first,
refuses to go on unless it reproduces the recorded result, and only then joins
the ground-truth pallet to say where the evidence was lost: visibility before
preprocessing (per-pixel expected depth from the pallet's own boxes), the
workspace filter, each plane-extraction round, every candidate plane and
pattern with its final verification, and where the lower-deck points came
from. The family rules are the plan's, fixed before any capture was classified.

The detector never sees the ground truth: ``diagnose`` runs ``detect_pockets``
on the scene alone, and the truth is joined afterwards.

    python -m tools.diagnose_detection artifacts/<run>/seed_1 [more runs] \\
        --json out.json
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
from forklift_core.perception import pocket_detector as det
from forklift_core.perception.pallet_prior import PalletPrior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.scene_dataset import SceneInput
from forklift_core.sensors.rgbd import PinholeIntrinsics
from tools import scene_rig

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PALLET_URDF = ROOT / "sim/models/epal6_pallet/pallet.urdf"
VISIBLE_TOLERANCE_M = 0.01
PART_MARGIN_M = 0.01
FRONT_DISTANCE_M = 0.05
FRONT_ANGLE_DEG = 15.0
CHAIN_AGREEMENT = 0.95
POSE_CONSTANT_M = 1e-4
POSE_CONSTANT_RAD = 1e-4
DETECTION_SOURCES = (
    "forklift_core/perception/pocket_detector.py",
    "forklift_core/perception/pallet_prior.py",
    "forklift_core/perception/pallet_geometry.py",
    "forklift_core/perception/scene_dataset.py",
    "forklift_core/sensors/rgbd.py",
    "forklift_core/geometry.py",
    "sim/isaac/perception_adapter.py",
    "tools/scene_rig.py",
)
# Detector check order for patterns (plan: the furthest-reaching pattern decides).
STAGES = ("no_gap_pair", "spacer", "evidence", "upper", "final", "valid")


# --- ground truth -----------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class PalletTruth:
    """The pallet as it stands: world x, y, yaw and its named boxes."""

    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    depth_m: float
    parts: tuple[
        tuple[str, tuple[float, float, float], tuple[float, float, float]], ...
    ]


def pallet_parts(urdf: Path):
    """Named collision boxes of the pallet link: (name, local centre, size)."""
    root = ET.parse(urdf).getroot()
    parts = []
    for collision in root.iter("collision"):
        box = collision.find("geometry/box")
        if box is None:
            continue
        origin = collision.find("origin")
        xyz = (0.0, 0.0, 0.0) if origin is None else origin.get("xyz", "0 0 0")
        if origin is not None and any(
            float(v) for v in origin.get("rpy", "0 0 0").split()
        ):
            raise ValueError(f"rotated pallet box {collision.get('name')}")
        centre = tuple(float(v) for v in (xyz.split() if isinstance(xyz, str) else xyz))
        size = tuple(float(v) for v in box.get("size").split())
        parts.append((collision.get("name") or "", centre, size))
    if not parts:
        raise ValueError("pallet URDF has no collision boxes")
    return tuple(parts)


def part_group(name: str) -> str:
    for group in ("top_board", "stringer", "block", "bottom_board"):
        if group in name:
            return group
    return "other_pallet"


def _yaw(q_wxyz) -> float:
    w, x, y, z = q_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def base_transform(accepted_pose) -> tuple[np.ndarray, np.ndarray]:
    """world_from_base rotation and translation from (position, wxyz)."""
    position, q = (np.asarray(v, dtype=float) for v in accepted_pose)
    rotation = rotation_matrix_from_quaternion_xyzw(np.roll(q, -1))
    return rotation, position


def truth_in_base(truth: PalletTruth, accepted_pose):
    """Pallet boxes as base-frame scene_rig boxes (plus names), and the front plane."""
    rotation, position = base_transform(accepted_pose)
    tilt = math.degrees(math.acos(min(1.0, abs(rotation[2, 2]))))
    if tilt > 0.06:  # 1e-3 rad: boxes are yawed only, so the base must be level
        raise ValueError(f"base tilt {tilt:.3f} deg too large for yaw-only boxes")
    base_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    c, s = math.cos(truth.yaw_rad), math.sin(truth.yaw_rad)
    boxes, names = [], []
    for name, centre, size in truth.parts:
        world = np.array(
            [
                truth.x_m + c * centre[0] - s * centre[1],
                truth.y_m + s * centre[0] + c * centre[1],
                truth.z_m + centre[2],
            ]
        )
        local = rotation.T @ (world - position)
        boxes.append(scene_rig.Box(tuple(local), size, truth.yaw_rad - base_yaw))
        names.append(name)
    # The approach face is the pallet's local -x face, facing the truck.
    front_world = np.array(
        [truth.x_m - c * truth.depth_m / 2, truth.y_m - s * truth.depth_m / 2, 0.0]
    )
    normal_world = -np.array([c, s, 0.0])
    front_point = rotation.T @ (front_world - position)
    front_normal = rotation.T @ normal_world
    return boxes, names, front_point, front_normal


def _inside(points: np.ndarray, box: scene_rig.Box, margin: float) -> np.ndarray:
    c, s = math.cos(box.yaw_rad), math.sin(box.yaw_rad)
    delta = points - np.asarray(box.centre_m)
    local = np.column_stack(
        (
            c * delta[:, 0] + s * delta[:, 1],
            -s * delta[:, 0] + c * delta[:, 1],
            delta[:, 2],
        )
    )
    half = np.asarray(box.size_m) / 2 + margin
    return np.all(np.abs(local) <= half, axis=1)


def label_points(points: np.ndarray, boxes, names) -> np.ndarray:
    """Part group per point: a group name, 'outside', or 'ambiguous'."""
    labels = np.full(len(points), "outside", dtype=object)
    hits = np.zeros(len(points), dtype=int)
    for box, name in zip(boxes, names, strict=True):
        inside = _inside(points, box, PART_MARGIN_M)
        labels[inside & (hits == 0)] = part_group(name)
        labels[inside & (hits > 0)] = "ambiguous"  # in two boxes: no single source
        hits += inside
    return labels


def _count_labels(labels) -> dict:
    values, counts = np.unique(np.asarray(labels, dtype=str), return_counts=True)
    return {str(v): int(n) for v, n in zip(values, counts, strict=True)}


# --- per-pixel expected depth ------------------------------------------------


def raycast(boxes, intrinsics: PinholeIntrinsics, base_from_optical: RigidTransform):
    """Expected optical depth of the pallet boxes alone, and which box each pixel hits."""
    origin = np.asarray(base_from_optical.translation_m, dtype=float)
    rays = scene_rig._rays(intrinsics, np.asarray(base_from_optical.rotation))
    depth = np.full((intrinsics.height, intrinsics.width), np.inf)
    index = np.full(depth.shape, -1, dtype=int)
    for i, box in enumerate(boxes):
        hit = scene_rig._box_depth(
            origin,
            rays,
            np.asarray(box.centre_m, dtype=float),
            box.size_m,
            scene_rig._yaw_matrix(box.yaw_rad),
        )
        closer = hit < depth
        depth[closer] = hit[closer]
        index[closer] = i
    return depth, index, rays, origin


def visibility(
    measured, expected, index, names, rays, origin, front_point, front_normal, boxes
):
    on_pallet = np.isfinite(expected)
    valid = np.isfinite(measured) & (measured > 0)
    with np.errstate(invalid="ignore"):
        delta = measured - expected
    visible = on_pallet & valid & (np.abs(delta) <= VISIBLE_TOLERANCE_M)
    nearer = on_pallet & valid & (delta < -VISIBLE_TOLERANCE_M)
    other = on_pallet & ~visible & ~nearer
    hit = origin + rays * np.where(on_pallet, expected, 0.0)[..., None]
    # A front pixel is one whose ray meets a box's local -x face (the face
    # turned towards the truck) where that face lies on the approach plane.
    front = np.zeros(on_pallet.shape, dtype=bool)
    for i, box in enumerate(boxes):
        mask = index == i
        if not mask.any():
            continue
        c, s = math.cos(box.yaw_rad), math.sin(box.yaw_rad)
        delta = hit[mask] - np.asarray(box.centre_m)
        local_x = c * delta[:, 0] + s * delta[:, 1]
        on_face = np.abs(local_x + box.size_m[0] / 2) <= 1e-6
        on_plane = np.abs((hit[mask] - front_point) @ front_normal) <= PART_MARGIN_M
        front[mask] = on_face & on_plane
    groups = {}
    for i, name in enumerate(names):
        group = part_group(name)
        mask = index == i
        entry = groups.setdefault(group, {"pixels": 0, "visible": 0})
        entry["pixels"] += int(mask.sum())
        entry["visible"] += int((mask & visible).sum())
    total = int(on_pallet.sum())
    return {
        "pallet_pixels_in_view": total,
        "visible": int(visible.sum()),
        "occluded_nearer": int(nearer.sum()),
        "farther_or_missing": int(other.sum()),
        "agreement": (int(visible.sum()) / total) if total else None,
        "front_face_pixels": int(front.sum()),
        "front_face_visible": int((front & visible).sum()),
        "front_face_nearer": int((front & nearer).sum()),
        "front_face_farther_or_missing": int((front & other).sum()),
        "by_part": groups,
    }


# --- detector replays --------------------------------------------------------


def extraction_rounds(points, camera, params):
    """_vertical_plane_candidates, instrumented: the same loop with each round's outcome."""
    rng = np.random.default_rng(params.seed)
    remaining = points
    planes, rounds = [], []
    end = "budget_exhausted"
    for _ in range(params.max_plane_candidates):
        if len(remaining) < max(3, params.min_plane_points):
            end = "too_few_remaining"
            break
        best_mask, best_count = None, 0
        for _ in range(params.ransac_iterations):
            sample = remaining[rng.choice(len(remaining), 3, replace=False)]
            normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-12:
                continue
            normal /= norm
            if abs(normal[2]) >= 0.1:
                continue
            residual = np.abs((remaining - sample[0]) @ normal)
            mask = residual <= params.plane_inlier_m
            count = int(mask.sum())
            if count > best_count:
                best_mask, best_count = mask, count
        if best_count < params.min_plane_points:
            end = "no_vertical_hypothesis" if best_count == 0 else "best_below_min"
            rounds.append(
                {"remaining": int(len(remaining)), "best_count": best_count, "end": end}
            )
            break
        plane = det._refit_vertical(
            remaining[best_mask], camera, median_offset=params.median_plane_offset
        )
        residual = np.abs((points - plane.point) @ plane.normal)
        full_mask = residual <= params.plane_inlier_m
        if np.count_nonzero(full_mask) < params.min_plane_points:
            end = "refit_below_min"
            rounds.append(
                {"remaining": int(len(remaining)), "best_count": best_count, "end": end}
            )
            break
        plane = det._Plane(
            plane.point,
            plane.normal,
            points[full_mask],
            float(np.percentile(residual[full_mask], 95)),
        )
        planes.append(plane)
        rounds.append(
            {
                "remaining": int(len(remaining)),
                "best_count": best_count,
                "inliers": int(len(plane.points)),
            }
        )
        remaining_residual = np.abs((remaining - plane.point) @ plane.normal)
        remaining = remaining[remaining_residual > params.plane_inlier_m]
    return planes, rounds, end


def selection_matches(
    winner, result, replayed_observation, replayed_rejections
) -> bool:
    """Whether a replayed winner is the detector's.

    Every recorded field must agree: the rejected patterns with their absolute
    bounds, the selected plane's inliers and residual, the pattern's evidence
    and widths, and the observation the winner produces (its absolute pocket
    centres when valid, its reason when not).
    """
    diagnostics = result.diagnostics
    if _normalise(replayed_rejections) != _normalise(diagnostics.rejected_patterns):
        return False
    if winner is None:
        return (
            diagnostics.plane_inlier_count == 0 and diagnostics.selected_lower is None
        )
    if _normalise(replayed_observation) != _normalise(result.observation):
        return False
    plane, pattern = winner[4], winner[5]
    widths = tuple(high - low for low, high in reversed(pattern.gaps))
    return (
        len(plane.points) == diagnostics.plane_inlier_count
        and plane.residual_p95_m == diagnostics.plane_residual_p95_m
        and pattern.support_min == diagnostics.selected_support_min
        and pattern.lower == diagnostics.selected_lower
        and pattern.upper_left == diagnostics.selected_upper_left
        and pattern.upper_right == diagnostics.selected_upper_right
        and widths == tuple(diagnostics.opening_width_raw_m or ())
    )


def same_planes(a, b) -> bool:
    return len(a) == len(b) and all(
        np.array_equal(x.point, y.point)
        and np.array_equal(x.normal, y.normal)
        and np.array_equal(x.points, y.points)
        and x.residual_p95_m == y.residual_p95_m
        for x, y in zip(a, b, strict=True)
    )


def _jsonable(value):
    if dataclasses.is_dataclass(value):
        return _jsonable(dataclasses.asdict(value))
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    return value


def _hash(*arrays) -> str:
    digest = hashlib.sha256()
    for array in arrays:
        array = np.ascontiguousarray(np.asarray(array))
        digest.update(str(array.dtype).encode() + str(array.shape).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


def detection_record(result) -> dict:
    """The detector's own output, with the wall-clock field removed (plan gate 3)."""
    record = _jsonable(
        {"observation": result.observation, "diagnostics": result.diagnostics}
    )
    record["diagnostics"].pop("elapsed_s", None)
    return record


# --- the diagnosis -----------------------------------------------------------


def diagnose(
    scene: SceneInput,
    prior: PalletPrior,
    params: DetectorParams,
    truth: PalletTruth | None = None,
    accepted_pose=None,
) -> dict:
    """Detect first; then, with the truth, say where the pallet's evidence went."""
    result = detect_pockets(scene, prior, params)
    out = {"detection": detection_record(result)}
    points, rays = det._base_points(scene)
    camera = np.asarray(scene.base_from_optical.translation_m)
    workspace = det._filter_workspace(points, camera, prior, params)
    before = _hash(scene.depth_m, points, workspace)

    real = det._vertical_plane_candidates(workspace, camera, params)
    planes, rounds, end = extraction_rounds(workspace, camera, params)
    out["extraction"] = {
        "replica_matches": same_planes(real, planes),
        "rounds": rounds,
        "end": end,
        "planes": len(planes),
        "budget": params.max_plane_candidates,
    }
    if not out["extraction"]["replica_matches"]:
        out["family"] = {"family": "undecidable", "why": "extraction_replica_mismatch"}
        return out

    candidates, plane_records = [], []
    for index, plane in enumerate(planes):
        record = {
            "index": index,
            "inliers": int(len(plane.points)),
            "residual_p95_m": plane.residual_p95_m,
        }
        if plane.residual_p95_m > params.max_plane_residual_m:
            record["rejected"] = "vertical_refit_residual"
            plane_records.append(record)
            continue
        patterns, rejections = det._opening_candidates(
            plane, prior, params, workspace, plane_index=index
        )
        record["rejections"] = _jsonable(rejections)
        record["rejection_objects"] = list(rejections)
        record["patterns"] = []
        for pattern in patterns:
            opening_rays = {
                side: det._classify_opening_rays(scene, rays, plane, gap, prior, params)
                for side, gap in zip(("right", "left"), pattern.gaps, strict=True)
            }
            score = pattern.support_count + pattern.deck_count
            score *= 1 + sum(count.behind_fraction for count in opening_rays.values())
            distance = float(np.linalg.norm((plane.point - camera)[:2]))
            if not pattern.upper_ok:
                final = {
                    "stage": "upper",
                    "reason": det._upper_deck_reason(pattern, params),
                }
            else:
                observation = det._build_observation(
                    scene, prior, params, plane, pattern, opening_rays
                )
                final = {
                    "stage": "valid" if observation.status == "valid" else "final",
                    "reason": observation.reason,
                }
            entry = {
                "gaps": _jsonable(pattern.gaps),
                "score": float(score),
                "distance_m": distance,
                "final": final,
            }
            record["patterns"].append(entry)
            candidates.append(
                (score, -distance, index, len(record["patterns"]) - 1, plane, pattern)
            )
        plane_records.append(record)
    winner = max(candidates, key=lambda c: c[:2]) if candidates else None
    out["selection"] = {
        "candidates": len(candidates),
        "winner": None
        if winner is None
        else {"plane": winner[2], "pattern": winner[3]},
    }
    # The detector's own winner, from its diagnostics, must be the replay's:
    # the same plane and the same pattern evidence, not just an inlier count.
    replayed_observation = None
    if winner is not None:
        _, _, _, _, chosen_plane, chosen_pattern = winner
        if chosen_pattern.upper_ok:
            chosen_rays = {
                side: det._classify_opening_rays(
                    scene, rays, chosen_plane, gap, prior, params
                )
                for side, gap in zip(
                    ("right", "left"), chosen_pattern.gaps, strict=True
                )
            }
            replayed_observation = det._build_observation(
                scene, prior, params, chosen_plane, chosen_pattern, chosen_rays
            )
        else:
            replayed_observation = det._status_observation(
                scene, "invalid", det._upper_deck_reason(chosen_pattern, params)
            )
    all_rejections = [
        rejection
        for record in plane_records
        for rejection in record.get("rejection_objects", [])
    ]
    out["selection"]["replica_matches"] = selection_matches(
        winner, result, replayed_observation, all_rejections
    )
    if not out["selection"]["replica_matches"]:
        for record in plane_records:
            record.pop("rejection_objects", None)
        out["planes"] = plane_records
        out["family"] = {"family": "undecidable", "why": "selection_replica_mismatch"}
        return out

    if truth is not None:
        boxes, names, front_point, front_normal = truth_in_base(truth, accepted_pose)
        expected, index, ray_grid, origin = raycast(
            boxes, scene.intrinsics, scene.base_from_optical
        )
        out["visibility"] = visibility(
            np.asarray(scene.depth_m, dtype=float),
            expected,
            index,
            names,
            ray_grid,
            origin,
            front_point,
            front_normal,
            boxes,
        )
        finite = np.isfinite(points).all(axis=1)
        all_labels = label_points(points[finite], boxes, names)
        work_labels = label_points(workspace, boxes, names)
        horizontal = np.hypot(points[:, 0] - camera[0], points[:, 1] - camera[1])
        reasons = {
            "nonfinite": ~finite,
            "at_or_below_floor": finite & ~(points[:, 2] > params.floor_z_m),
            "above_slab": finite & ~(points[:, 2] <= prior.height_m + 0.10),
            "nearer_than_range_min": finite & ~(horizontal >= params.range_min_m),
            "farther_than_range_max": finite & ~(horizontal <= params.range_max_m),
        }
        point_labels = np.full(len(points), "outside", dtype=object)
        point_labels[finite] = all_labels
        removed = {
            name: {
                "points": int(mask.sum()),
                "pallet": int(np.sum(mask & (point_labels != "outside"))),
            }
            for name, mask in reasons.items()
        }
        out["preprocessing"] = {
            "removed_by": removed,
            "valid_points": int(finite.sum()),
            "workspace_points": int(len(workspace)),
            "pallet_points_before": int(np.sum(~np.isin(all_labels, ["outside"]))),
            "pallet_points_workspace": int(np.sum(~np.isin(work_labels, ["outside"]))),
            "workspace_by_part": _count_labels(work_labels),
        }
        for round_record, plane in zip(
            [r for r in rounds if "inliers" in r], planes, strict=True
        ):
            labels = label_points(plane.points, boxes, names)
            round_record["purity"] = float(np.mean(labels != "outside"))
        for record, plane in zip(plane_records, planes, strict=True):
            labels = label_points(plane.points, boxes, names)
            record["purity"] = float(np.mean(labels != "outside"))
            unit = front_normal / np.linalg.norm(front_normal)
            record["front_distance_m"] = float(
                np.median((plane.points - front_point) @ unit)
            )
            cosine = abs(float(np.dot(plane.normal, unit)))
            record["front_angle_deg"] = math.degrees(math.acos(min(1.0, cosine)))
            record["front"] = (
                abs(record["front_distance_m"]) <= FRONT_DISTANCE_M
                and record["front_angle_deg"] <= FRONT_ANGLE_DEG
            )
            if record["front"]:
                record["lower_provenance"] = lower_provenance(
                    plane, record, prior, params, workspace, boxes, names
                )
        out["family"] = classify(out, params, plane_records)
    for record in plane_records:
        record.pop("rejection_objects", None)
    out["planes"] = plane_records
    after = _hash(scene.depth_m, points, workspace)
    out["unmodified"] = before == after and same_planes(real, planes)
    return out


def lower_provenance(plane, record, prior, params, workspace, boxes, names) -> list:
    """Each pattern's lower-band conditions applied one at a time, by part."""
    out = []
    lateral = workspace @ plane.left_axis
    depth = -(workspace - plane.point) @ plane.normal
    labels = label_points(workspace, boxes, names)
    height = np.abs(workspace[:, 2] - prior.deck_bottom_m) <= params.deck_evidence_tol_m
    ahead = depth >= 0
    within = depth <= prior.overall_depth_m + params.plane_inlier_m
    gap_sets = [
        r["gaps"]
        for r in record.get("rejections", [])
        if r.get("gaps") and len(r["gaps"]) == 2
    ]
    gap_sets += [p["gaps"] for p in record.get("patterns", [])]
    seen = set()
    for gaps in gap_sets:
        key = tuple(map(tuple, gaps))
        if key in seen:
            continue
        seen.add(key)
        left, right = gaps[0][0], gaps[1][1]
        window = (lateral >= left) & (lateral <= right)
        stages = {"dropped_by_height": _count_labels(labels[~height])}
        mask = height.copy()
        stages["height"] = _count_labels(labels[mask])
        stages["dropped_by_depth_ahead"] = _count_labels(labels[mask & ~ahead])
        mask &= ahead
        stages["depth_ahead"] = _count_labels(labels[mask])
        stages["dropped_by_depth_cap"] = _count_labels(labels[mask & ~within])
        mask &= within
        stages["depth_cap"] = _count_labels(labels[mask])
        stages["dropped_by_window"] = _count_labels(labels[mask & ~window])
        mask &= window
        stages["window"] = _count_labels(labels[mask])
        out.append({"gaps": gaps, "stages": stages, "lower": int(mask.sum())})
    return out


def _pattern_stage(item: dict) -> tuple[int, str, list]:
    """(stage rank, stage name, failed items) of a rejection or a kept pattern."""
    if "final" in item:
        stage = item["final"]["stage"]
        return STAGES.index(stage), stage, [item["final"].get("reason")]
    stage = item.get("stage")
    if stage == "no_gap_pair":
        return 0, "no_gap_pair", list(item.get("failed_items") or ())
    if stage == "spacer_out_of_range":
        return 1, "spacer", list(item.get("failed_items") or ())
    if stage == "insufficient_evidence":
        return 2, "evidence", list(item.get("failed_items") or ())
    if stage == "insufficient_upper":
        return 3, "upper", list(item.get("failed_items") or ())
    return -1, str(stage), []


def classify(out: dict, params: DetectorParams, plane_records: list) -> dict:
    """The plan's family rules, applied in order A -> P -> B -> R -> C/D."""
    if out["detection"]["observation"]["status"] == "valid":
        return {"family": "OK"}
    minimum = params.min_plane_points
    vis = out["visibility"]
    # The detector needs a vertical front plane; top boards seen from above
    # do not make one, so A counts the approach face's visible pixels.
    if vis["front_face_visible"] < minimum:
        if vis["front_face_pixels"] == 0:
            why = "outside_view"
        elif vis["front_face_nearer"] and (
            vis["front_face_nearer"] >= vis["front_face_farther_or_missing"]
        ):
            why = "occluded"
        elif vis["front_face_farther_or_missing"]:
            why = "missing"
        else:
            # Every front pixel in view is seen; there are just too few of them.
            why = "partial_view"
        return {"family": "A", "why": why}
    if out["preprocessing"]["pallet_points_workspace"] < minimum:
        return {"family": "P"}
    front = [r for r in plane_records if r.get("front")]
    if not front:
        return {"family": "B", "why": out["extraction"]["end"]}
    usable = [r for r in front if r.get("rejected") != "vertical_refit_residual"]
    if not usable:
        return {"family": "R"}
    items = []
    for record in usable:
        items += [(record["index"], r) for r in record.get("rejections", [])]
        items += [(record["index"], p) for p in record.get("patterns", [])]
    if not items:
        return {"family": "undecidable", "why": "front_plane_without_patterns"}
    ranked = [(_pattern_stage(item), plane, item) for plane, item in items]
    best = max(rank for (rank, _, _), _, _ in ranked)
    reached = [
        (name, failed, plane)
        for (rank, name, failed), plane, _ in ranked
        if rank == best
    ]
    name = STAGES[best] if best >= 0 else "unknown"
    failed = sorted({f for _, items_failed, _ in reached for f in items_failed if f})
    if name == "valid":
        return {"family": "C", "why": "selection", "stage": name}
    if name == "evidence":
        lower_only = all(set(f) == {"lower"} for _, f, _ in reached)
        return {
            "family": "D" if lower_only else "C",
            "why": "evidence",
            "failed": failed,
        }
    sub = {
        "no_gap_pair": "opening",
        "spacer": "opening",
        "upper": "upper",
        "final": "final",
    }
    return {"family": "C", "why": sub.get(name, name), "failed": failed}


# --- run records -------------------------------------------------------------


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


MOUNT_SNIPPETS = (
    "perception_mount = adapter.default_base_from_optical()",
    "translation=np.asarray(perception_mount.translation_m),",
    "orientation=np.asarray(adapter.xyzw_to_wxyz(rig.OPTICAL_QUATERNION_XYZW)),",
    "camera_axes=args.perception_camera_axes,",
)
# Runners that record each capture's mount (2026-10-03 carriage mount adoption):
# the mount comes from the adapter's named table and is written per attempt.
MOUNT_SNIPPETS_NAMED = (
    "perception_mount = adapter.mount_base_from_optical(args.perception_mount)",
    "perception_mount = adapter.default_base_from_optical()",
    'attempt["base_from_optical"] = {',
    "camera_axes=args.perception_camera_axes,",
)
_RUNNER_CACHE: dict[tuple, str | None] = {}


def runner_mount_commit(
    script_sha256: str | None, snippets: tuple = MOUNT_SNIPPETS
) -> str | None:
    """The commit whose run_transport.py has this hash and applies the nominal mount.

    The run record keeps only the runner's hash, so the mount the runner applied
    is checked in git history: the matching revision must set the perception
    camera from the adapter's default_base_from_optical with the rig's optical
    quaternion. None when no revision matches or the mount code differs.
    """
    import subprocess

    if script_sha256 is None:
        return None
    if (script_sha256, snippets) in _RUNNER_CACHE:
        return _RUNNER_CACHE[(script_sha256, snippets)]
    found = None
    try:
        commits = subprocess.run(
            ["git", "log", "--format=%H", "--", "sim/isaac/run_transport.py"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.split()
        for commit in commits:
            blob = subprocess.run(
                ["git", "show", f"{commit}:sim/isaac/run_transport.py"],
                cwd=ROOT,
                capture_output=True,
                check=True,
            ).stdout
            if hashlib.sha256(blob).hexdigest() == script_sha256:
                text = blob.decode()
                if all(snippet in text for snippet in snippets):
                    found = commit
                break
    except (OSError, subprocess.CalledProcessError):
        found = None
    _RUNNER_CACHE[(script_sha256, snippets)] = found
    return found


def _adapter():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "diagnose_perception_adapter", ROOT / "sim/isaac/perception_adapter.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def recorded_mount_matches(result: dict, attempt: dict) -> bool:
    """The attempt's recorded base<-optical is the named mount the run asked for."""
    recorded = attempt.get("base_from_optical")
    name = result["arguments"].get("perception_mount", "legacy")
    if recorded is None:
        return False
    try:
        planned = _adapter().mount_base_from_optical(name)
    except KeyError:
        return False
    return bool(
        np.allclose(recorded["rotation"], planned.rotation, rtol=0, atol=1e-9)
        and np.allclose(
            recorded["translation_m"], planned.translation_m, rtol=0, atol=1e-9
        )
    )


def depth_array_sha256(depth: np.ndarray) -> str:
    """The run records' depth hash (sim/isaac/g2_records.py depth_sha256)."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "g2_records", ROOT / "sim/isaac/g2_records.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.depth_sha256(depth)


def observe_pallet_pose(result: dict):
    """Mean pallet pose over observe-phase samples, if it stayed constant."""
    samples = [s for s in result.get("samples", []) if s.get("phase") == "observe"]
    if not samples:
        return None, "no_observe_samples"
    xy = np.array([s["pallet_position_m"] for s in samples], dtype=float)
    yaw = np.array([s["pallet_yaw_rad"] for s in samples], dtype=float)
    spread = float(np.max(np.linalg.norm(xy - xy[0], axis=1)))
    yaw_spread = float(
        np.max(np.abs(np.arctan2(np.sin(yaw - yaw[0]), np.cos(yaw - yaw[0]))))
    )
    if spread > POSE_CONSTANT_M or yaw_spread > POSE_CONSTANT_RAD:
        return None, f"pallet_moved_during_observe:{spread:.2e}m,{yaw_spread:.2e}rad"
    mean = xy.mean(axis=0)
    mean_yaw = math.atan2(float(np.mean(np.sin(yaw))), float(np.mean(np.cos(yaw))))
    return (float(mean[0]), float(mean[1]), float(mean[2]), mean_yaw), None


def attempt_scene(result: dict, attempt: dict, depth: np.ndarray) -> SceneInput:
    perception_adapter = _adapter()
    # The runner's mount: the recorded named mount when the run recorded one
    # (checked by the runner_mount gate), else the adapter's nominal mount.
    if "base_from_optical" in attempt:
        from forklift_core.geometry import RigidTransform

        recorded = attempt["base_from_optical"]
        default = perception_adapter.default_base_from_optical()
        mount = RigidTransform(
            default.source_frame,
            default.target_frame,
            np.asarray(recorded["rotation"], dtype=float),
            np.asarray(recorded["translation_m"], dtype=float),
        )
    else:
        mount = perception_adapter.default_base_from_optical()
    if attempt.get("depth_quantize_mm"):
        depth = perception_adapter.quantize_depth_mm(depth)

    k = attempt["capture_diagnostics"]["intrinsics"]["integer_index"]["matrix"]
    intrinsics = PinholeIntrinsics(
        640, 480, k[0][0], k[1][1], k[0][2], k[1][2], "camera_optical_frame"
    )
    observation = attempt["pocket_observation"]
    return SceneInput(
        rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        depth_m=np.asarray(depth, dtype=float),
        intrinsics=intrinsics,
        base_from_optical=mount,
        stamp_ns=int(observation["stamp_ns"]),
        clock_domain="synthetic",
        source_provenance="synthetic",
    )


def _normalise(value):
    return json.loads(json.dumps(_jsonable(value)))


FLOAT_TOLERANCE = 1e-9


def max_float_difference(a, b) -> float | None:
    """Largest float difference between two JSON values, or None if they differ
    in structure, type or any non-float value. ws1 and a local machine can
    disagree in the last bits of a float (different BLAS), nothing else."""
    if isinstance(a, dict) and isinstance(b, dict):
        if set(a) != set(b):
            return None
        values = [max_float_difference(a[key], b[key]) for key in a]
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return None
        values = [max_float_difference(x, y) for x, y in zip(a, b, strict=True)]
    elif isinstance(a, float) and isinstance(b, float):
        if math.isfinite(a) and math.isfinite(b):
            return abs(a - b)
        # NaN or infinite on either side: equal only as the same non-finite value.
        same = (math.isnan(a) and math.isnan(b)) or a == b
        return 0.0 if same else None
    else:
        return 0.0 if a == b and type(a) is type(b) else None
    if any(v is None for v in values):
        return None
    return max(values, default=0.0)


def diagnose_attempt(run_dir: Path, number: int, pallet_urdf: Path) -> dict:
    """Gates of the plan, then ``diagnose``; a failed gate stops with its reason."""
    from forklift_core.perception.pallet_prior import PalletPrior as Prior

    result = json.loads((run_dir / "result.json").read_text())
    attempt = result["observation_attempts"][number - 1]
    depth = np.load(run_dir / f"perception_capture_{number}_depth_m.npy")
    out = {
        "run": str(run_dir),
        "attempt": number,
        "candidate": attempt["candidate_index"],
        "gates": {},
    }
    recorded = attempt.get("depth_sha256")
    out["gates"]["depth_hash"] = (
        recorded is None or depth_array_sha256(depth) == recorded
    )
    sources = result.get("source_sha256", {})
    mismatched = []
    for name in DETECTION_SOURCES:
        key = next((k for k in sources if k.endswith(name)), None)
        local = ROOT / ("src/" + name if name.startswith("forklift_core") else name)
        if key is None or sources[key] != _sha256_file(local):
            mismatched.append(name)
    out["gates"]["sources"] = not mismatched
    out["gates"]["sources_mismatched"] = mismatched
    out["gates"]["camera_axes"] = (
        result["arguments"].get("perception_camera_axes") == "ros"
    )
    if "base_from_optical" in attempt:
        out["runner_commit"] = runner_mount_commit(
            result.get("script_sha256"), MOUNT_SNIPPETS_NAMED
        )
        out["gates"]["runner_mount"] = out[
            "runner_commit"
        ] is not None and recorded_mount_matches(result, attempt)
    else:
        out["runner_commit"] = runner_mount_commit(result.get("script_sha256"))
        out["gates"]["runner_mount"] = out["runner_commit"] is not None
    out["gates"]["pallet_urdf"] = result.get("pallet_urdf_sha256") == _sha256_file(
        pallet_urdf
    )
    prior = Prior(**result["arguments"]["pallet_prior_loaded"])
    if result.get("detector_params"):
        params = DetectorParams(**result["detector_params"])
        out["params_source"] = "recorded"
    else:
        params = DetectorParams.derived_for(prior)
        out["params_source"] = "derived_for(prior)"
    if not all(v for k, v in out["gates"].items() if k != "sources_mismatched"):
        out["family"] = {"family": "undecidable", "why": "gate_failed"}
        return out

    pose, why = observe_pallet_pose(result)
    truth = None
    if pose is not None:
        geometry = result["arguments"]["pallet_geometry_loaded"]
        truth = PalletTruth(
            *pose, geometry["overall_depth_m"], pallet_parts(pallet_urdf)
        )
        nominal = result["scenario"]["pickup"]
        out["truth"] = {
            "pose": pose,
            "minus_nominal_m": math.hypot(
                pose[0] - nominal["x_m"], pose[1] - nominal["y_m"]
            ),
            "minus_nominal_yaw_rad": pose[3] - nominal["yaw_rad"],
        }
    else:
        out["truth"] = {"unavailable": why}
    scene = attempt_scene(result, attempt, depth)
    diagnosis = diagnose(
        scene, prior, params, truth, attempt["capture_diagnostics"]["accepted_pose"]
    )
    recorded_detection = {
        "observation": attempt["pocket_observation"],
        "diagnostics": {
            k: v
            for k, v in attempt["detection_diagnostics"].items()
            if k != "elapsed_s"
        },
    }
    difference = max_float_difference(
        _normalise(diagnosis["detection"]), _normalise(recorded_detection)
    )
    out["detection_max_float_difference"] = difference
    out["gates"]["detection_reproduced"] = (
        difference is not None and difference <= FLOAT_TOLERANCE
    )
    out.update(diagnosis)
    if not out["gates"]["detection_reproduced"]:
        out["family"] = {"family": "undecidable", "why": "detection_not_reproduced"}
    elif truth is None:
        out["family"] = {"family": "undecidable", "why": out["truth"]["unavailable"]}
    return out


def chain_check(records: Sequence[dict]) -> dict:
    """Per run configuration, at least one attempt must agree with the truth chain."""
    by_config: dict[tuple, list] = {}
    for record in records:
        result = json.loads((Path(record["run"]) / "result.json").read_text())
        # The same runner can now run several mounts and depth roundings: each
        # is its own configuration (Codex review, 2026-10-04).
        key = (
            result.get("script_sha256"),
            result.get("pallet_urdf_sha256"),
            result["arguments"].get("perception_mount", "legacy"),
            int(result["arguments"].get("depth_quantize_mm", 0) or 0),
        )
        by_config.setdefault(key, []).append(record)
    out = {}
    for key, group in by_config.items():
        best = max(((r.get("visibility") or {}).get("agreement") or 0.0) for r in group)
        label = str(key[0])[:12]
        if key[2] != "legacy" or key[3]:
            label += f"/{key[2]}/q{key[3]}"
        out[label] = {
            "attempts": len(group),
            "best_agreement": best,
            "ok": best >= CHAIN_AGREEMENT,
        }
        if best < CHAIN_AGREEMENT:
            for r in group:
                r["family"] = {"family": "undecidable", "why": "truth_chain_unverified"}
    return out


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--pallet-urdf", type=Path, default=DEFAULT_PALLET_URDF)
    parser.add_argument("--json", type=Path, default=None)
    args = parser.parse_args(argv)
    records = []
    for run in args.runs:
        result = json.loads((run / "result.json").read_text())
        for number in range(1, len(result.get("observation_attempts", [])) + 1):
            records.append(diagnose_attempt(run, number, args.pallet_urdf))
    chains = chain_check([r for r in records if "visibility" in r])
    print(f"# truth chain per configuration: {json.dumps(chains)}")
    print(
        f"{'run':>34s} {'att':>3s} {'cand':>4s} {'status':>9s} {'family':>12s} "
        f"{'visible':>8s} {'occl':>6s} {'ws_pal':>6s} {'planes':>6s} {'end':>18s} front"
    )
    for r in records:
        obs = r.get("detection", {}).get("observation", {})
        vis = r.get("visibility", {})
        fam = r.get("family", {})
        fronts = [
            f"{p['index']}:{p.get('front_distance_m', 0):+.3f}m/{p.get('front_angle_deg', 0):.1f}deg"
            for p in r.get("planes", [])
            if p.get("front")
        ]
        print(
            f"{str(Path(r['run']).name):>34s} {r['attempt']:3d} {r['candidate']:4d} "
            f"{obs.get('status', '-'):>9s} "
            f"{fam.get('family', '-') + ('/' + fam['why'] if fam.get('why') else ''):>12s} "
            f"{vis.get('visible', '-'):>8} {vis.get('occluded_nearer', '-'):>6} "
            f"{r.get('preprocessing', {}).get('pallet_points_workspace', '-'):>6} "
            f"{r.get('extraction', {}).get('planes', '-'):>6} "
            f"{r.get('extraction', {}).get('end', '-'):>18s} {' '.join(fronts) or '-'}"
        )
    if args.json:
        args.json.write_text(json.dumps(_jsonable(records), indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
