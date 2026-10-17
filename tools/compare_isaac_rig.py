#!/usr/bin/env python3
"""Check noise-free Isaac renders against the CPU rig rendered at the scene's own K.

Per target (instance or the distractor group): IoU, valid-depth fraction, depth error
on pixels both renders give the target, and the mismatch fraction -- rig target pixels
that Isaac leaves without depth or puts more than 5 mm away -- which catches a missing
or misplaced part whose pixels drop out of the same-id comparison. A pixel Isaac gives
another label at the same depth is a tie (touching pallets share a face, and either
surface may win it); ties are counted, and IoU still bounds wrong labels. Background: distance of Isaac's back-projected points to the floor (z = 0)
or the back wall (x = BACK_WALL_X_M). Plus the K convention and the camera pose.
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import yaml

from forklift_core.perception.recognition_dataset import load_recognition_sample
from tools import instance_rig, scene_rig
from tools import recognition_scenes as rs
from tools.generate_recognition_scenes import load_site, recipe_record

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SETTINGS = ROOT / "config/isaac_perception_camera.yaml"
IOU_MIN, VALID_MIN, DEPTH_MEDIAN_MAX_M, DEPTH_P95_MAX_M = 0.97, 0.99, 0.001, 0.003
BACKGROUND_P95_MAX_M, POSE_MM_MAX, POSE_DEG_MAX, K_TOL_PX, MIN_PIXELS = 0.002, 1.0, 0.01, 0.01, 200
MISMATCH_MAX, MISMATCH_DEPTH_M = 0.005, 0.005


def load_settings(path) -> dict:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _k_contract(k, settings) -> bool:
    width, height = settings["resolution"]
    f = float(settings["focal_px"])
    if (k.width, k.height) != (width, height):
        return False
    want = (f, f, width / 2 - 0.5, height / 2 - 0.5)
    return all(abs(a - b) <= K_TOL_PX for a, b in zip((k.fx, k.fy, k.cx, k.cy), want))


def _target(depth_a, depth_b, a, b) -> dict:
    union = np.logical_or(a, b).sum()
    valid = float(np.isfinite(depth_a[b]).mean()) if b.any() else 1.0
    both = a & b & np.isfinite(depth_a) & np.isfinite(depth_b)
    diff = np.abs(depth_a[both] - depth_b[both])
    with np.errstate(invalid="ignore"):
        same_depth = np.isfinite(depth_a) & (np.abs(depth_a - depth_b) <= MISMATCH_DEPTH_M)
    mismatch = float((b & ~same_depth).sum() / b.sum()) if b.any() else 0.0
    ties = float((b & same_depth & ~a).sum() / b.sum()) if b.any() else 0.0
    report = {"pixels": int(b.sum()), "iou": float(np.logical_and(a, b).sum() / union) if union else 1.0,
              "valid": valid, "mismatch": mismatch, "ties": ties, "depth_median_m": float(np.median(diff)) if diff.size else None,
              "depth_p95_m": float(np.percentile(diff, 95)) if diff.size else None}
    report["pass"] = bool(report["iou"] >= IOU_MIN and valid >= VALID_MIN and mismatch <= MISMATCH_MAX and diff.size
                          and report["depth_median_m"] <= DEPTH_MEDIAN_MAX_M
                          and report["depth_p95_m"] <= DEPTH_P95_MAX_M)
    return report


def compare_scene(sample, recipe, settings) -> dict:
    scene = sample.input.scene
    rig = recipe_record(recipe, load_site(), intrinsics=scene.intrinsics).rendered
    ids_a, ids_b = sample.instance_ids, rig.instance_ids
    da, db = scene.depth_m, rig.scene.depth_m
    targets, small = {}, {}
    for k in sorted(set(np.unique(ids_b).tolist()) - {-1}):
        report = _target(da, db, ids_a == k, ids_b == k)
        (targets if report["pixels"] >= MIN_PIXELS else small)[str(k)] = report
    bg = (ids_b == -1) & np.isfinite(db)
    use = bg & (ids_a == -1) & np.isfinite(da)
    bg_valid = float(use.sum() / bg.sum()) if bg.any() else 1.0
    v, u = np.nonzero(use)
    k = scene.intrinsics
    z = da[use]
    optical = np.stack(((u - k.cx) / k.fx * z, (v - k.cy) / k.fy * z, z), axis=1)
    tf = scene.base_from_optical
    points = optical @ np.asarray(tf.rotation).T + np.asarray(tf.translation_m)
    on_floor = rig.points_base[use][:, 2] < 1e-6
    dist = np.where(on_floor, np.abs(points[:, 2]), np.abs(points[:, 0] - scene_rig.BACK_WALL_X_M))
    background = {"pixels": int(use.sum()), "valid": bg_valid,
                  "surface_p95_m": float(np.percentile(dist, 95)) if dist.size else None}
    background["pass"] = bool(dist.size and bg_valid >= VALID_MIN
                              and background["surface_p95_m"] <= BACKGROUND_P95_MAX_M)
    want = instance_rig.PERCEPTION_CAMERA.base_from_optical()
    dt = float(np.linalg.norm(np.asarray(tf.translation_m) - np.asarray(want.translation_m)) * 1000)
    cos = (np.trace(np.asarray(tf.rotation).T @ np.asarray(want.rotation)) - 1) / 2
    dr = float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))
    report = {"targets": targets, "small": small, "background": background, "k_contract": _k_contract(k, settings),
              "pose_translation_mm": dt, "pose_rotation_deg": dr}
    report["pass"] = bool(all(t["pass"] for t in targets.values()) and background["pass"] and report["k_contract"]
                          and dt <= POSE_MM_MAX and dr <= POSE_DEG_MAX)
    return report


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--isaac", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--settings", type=Path, default=DEFAULT_SETTINGS)
    a = p.parse_args(argv)
    settings = load_settings(a.settings)
    recipes = {r.scene_id: r for r in rs.catalogue()}
    reports = {d.parent.name: compare_scene(load_recognition_sample(d.parent), recipes[d.parent.name], settings)
               for d in sorted(a.isaac.glob("*/scene.json"))}
    targets = [t for r in reports.values() for t in r["targets"].values()]
    summary = {
        "scenes": len(reports), "targets": len(targets),
        "iou_min": min((t["iou"] for t in targets), default=None),
        "valid_min": min((t["valid"] for t in targets), default=None),
        "mismatch_max": max((t["mismatch"] for t in targets), default=None),
        "ties_max": max((t["ties"] for t in targets), default=None),
        "depth_p95_max_m": max((t["depth_p95_m"] or 0.0 for t in targets), default=None),
        "background_p95_max_m": max((r["background"]["surface_p95_m"] or 0.0 for r in reports.values()),
                                    default=None),
        "failed": sorted(name for name, r in reports.items() if not r["pass"]),
    }
    summary["pass"] = bool(reports) and not summary["failed"]
    a.out.write_text(json.dumps({"summary": summary, "scenes": reports}, indent=1))
    print(json.dumps(summary))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
