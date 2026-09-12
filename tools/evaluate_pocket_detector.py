#!/usr/bin/env python3
"""Evaluate one synthetic scene split and save observations and display artifacts."""

import argparse
import csv
import hashlib
import json
import platform
import shutil
import subprocess
import sys
import time
import traceback
from collections.abc import Sequence
from dataclasses import asdict, fields
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from forklift_core.perception.evaluation import (
    NEGATIVE_CATEGORIES,
    POSITION_TOLERANCE_M,
    POSITIVE_CATEGORIES,
    YAW_TOLERANCE_RAD,
    evaluate_scene,
    summarize,
)
from forklift_core.perception.overlay import draw_scene_overlay
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.pocket_observation import PocketObservation
from forklift_core.perception.scene_dataset import load_scene_sample

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_COLUMNS = (
    "scene_id",
    "category",
    "split",
    "truth_status",
    "estimate_status",
    "outcome",
    "left_error_m",
    "right_error_m",
    "position_error_m",
    "yaw_error_rad",
    "reason",
    "elapsed_s",
    "plane_residual_p95_m",
    "plane_inlier_count",
    "left_front_frac",
    "left_behind_frac",
    "right_front_frac",
    "right_behind_frac",
)


def _utc_now():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path, data):
    path.write_text(
        json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )


def _load_params(path):
    if path is None:
        return DetectorParams()
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError("Malformed detector parameter YAML") from exc
    if not isinstance(data, dict):
        raise ValueError("Detector parameters must be a YAML mapping")
    unknown = data.keys() - {field.name for field in fields(DetectorParams)}
    if unknown:
        raise ValueError(f"Unknown detector parameter keys: {list(unknown)!r}")
    return DetectorParams(**data)


def _select_scenes(dataset, split, selection):
    scenes_dir = dataset / "scenes"
    if not scenes_dir.is_dir():
        raise ValueError(f"Dataset scenes directory is missing: {scenes_dir}")
    available = {}
    for path in sorted(scenes_dir.iterdir()):
        if not path.is_dir():
            continue
        scene = json.loads((path / "scene.json").read_text(encoding="utf-8"))
        if not isinstance(scene, dict) or scene.get("scene_id") != path.name:
            raise ValueError(f"Scene ID must match its directory: {path}")
        if scene.get("split") not in ("dev", "eval"):
            raise ValueError(f"Unsupported scene split: {path}")
        if scene["split"] == split:
            available[scene["scene_id"]] = path
    if selection is None:
        selected = sorted(available)
    else:
        selected = [scene_id.strip() for scene_id in selection.split(",")]
        if any(not scene_id or scene_id not in available for scene_id in selected):
            raise ValueError(
                "Scene selection must name nonempty IDs in the chosen split"
            )
        if len(set(selected)) != len(selected):
            raise ValueError("Scene selection contains duplicate IDs")
        selected.sort()
    if not selected:
        raise ValueError(f"No scenes selected for split {split!r}")
    return [available[scene_id] for scene_id in selected]


def _git_state():
    def git(*args):
        return subprocess.run(
            ["git", "--no-optional-locks", "-C", str(REPO_ROOT), *args],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()

    try:
        return git("rev-parse", "HEAD"), bool(git("status", "--porcelain"))
    except (OSError, subprocess.CalledProcessError):
        # A source export has no Git metadata; unknown is never claimed clean.
        return None, None


def _diagnostics_json(diagnostics):
    data = asdict(diagnostics)
    for side in ("left", "right"):
        rays = diagnostics.opening_rays.get(side)
        for label in ("front", "behind"):
            fraction = getattr(rays, f"{label}_fraction") if rays is not None else None
            data[f"{side}_{label}_frac"] = fraction
            if rays is not None:
                data["opening_rays"][side][f"{label}_fraction"] = fraction
    return data


def _invalid_observation(scene_input, exc):
    return PocketObservation(
        stamp_ns=scene_input.stamp_ns,
        clock_domain=scene_input.clock_domain,
        frame_id="base_link",
        source_provenance=scene_input.source_provenance,
        status="invalid",
        left=None,
        right=None,
        insertion_yaw_rad=None,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason=f"exception:{type(exc).__name__}",
    )


def _evaluate_sample(sample, prior, params):
    start = time.perf_counter()
    try:
        detected = detect_pockets(sample.input, prior, params)
        observation = detected.observation
        diagnostics = _diagnostics_json(detected.diagnostics)
        result = evaluate_scene(
            sample, observation, elapsed_s=detected.diagnostics.elapsed_s
        )
    except Exception as exc:
        diagnostics = {"exception_traceback": traceback.format_exc()}
        observation = _invalid_observation(sample.input, exc)
        result = evaluate_scene(
            sample, observation, elapsed_s=time.perf_counter() - start
        )
    return result, observation, diagnostics


def _save_overlay(path, sample, observation, result, *, video):
    from PIL import Image

    position = (
        f"{result.position_error_m:.4f} m"
        if result.position_error_m is not None
        else "unknown"
    )
    yaw = (
        f"{result.yaw_error_rad:.4f} rad"
        if result.yaw_error_rad is not None
        else "unknown"
    )
    caption = (
        f"{result.scene_id} {result.outcome.value} ({observation.status})\n"
        f"position error: {position}; yaw error: {yaw}\n"
        "truth: green; estimate: magenta"
    )
    if video:
        caption += "\nIndependent scenes; not continuous observation"
    pixels = draw_scene_overlay(
        sample.input.rgb,
        truth=sample.ground_truth,
        estimate=observation,
        intrinsics=sample.input.intrinsics,
        base_from_optical=sample.input.base_from_optical,
        caption=caption,
    )
    Image.fromarray(pixels).save(path)


def _make_video(output, no_overlay):
    if no_overlay:
        return "Video requires overlay PNGs; --no-overlay was supplied"
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        return "ffmpeg is not available; overlay PNGs were retained"
    try:
        subprocess.run(
            [
                ffmpeg,
                "-nostdin",
                "-n",
                "-loglevel",
                "error",
                "-framerate",
                "5",
                "-pattern_type",
                "glob",
                "-i",
                str(output / "overlay" / "*.png"),
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(output / "overlay.mp4"),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        return f"{type(exc).__name__}: {exc}\n{getattr(exc, 'stderr', '') or ''}"
    return None


def _run(args):
    started = _utc_now()
    dataset = args.dataset.resolve()
    prior_path = args.prior.resolve()
    params_path = args.params.resolve() if args.params is not None else None
    prior = load_pallet_prior(prior_path)
    params = _load_params(params_path)
    # This is a configuration error, before the per-scene recovery boundary.
    if 2 * params.band_margin_m >= prior.opening_height_m:
        raise ValueError("band_margin_m must leave a nonempty opening height band")
    manifest_hash = _sha256(dataset / "manifest.json")
    paths = _select_scenes(dataset, args.split, args.scenes)
    revision, dirty = _git_state()
    record = {
        "dataset_dir": str(dataset),
        "dataset_manifest_sha256": manifest_hash,
        "split": args.split,
        "scene_ids": [],
        "scene_count": 0,
        "prior_path": str(prior_path),
        "prior_sha256": _sha256(prior_path),
        "params": asdict(params),
        "params_path": str(params_path) if params_path is not None else None,
        "position_tolerance_m": POSITION_TOLERANCE_M,
        "yaw_tolerance_rad": YAW_TOLERANCE_RAD,
        "git_revision": revision,
        "git_dirty": dirty,
        "started_at_utc": started,
        "python_version": platform.python_version(),
        "numpy_version": np.__version__,
    }
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "observations").mkdir()
    if not args.no_overlay:
        (output / "overlay").mkdir()
    results = []
    with (output / "scenes.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=SCENE_COLUMNS)
        writer.writeheader()
        for path in paths:
            # Input I/O failures are tool failures, not detector observations.
            sample = load_scene_sample(path)
            if sample.scene["category"] not in (
                *POSITIVE_CATEGORIES,
                *NEGATIVE_CATEGORIES,
            ):
                raise ValueError(f"Unsupported scene category: {path}")
            result, observation, diagnostics = _evaluate_sample(sample, prior, params)
            scene_id = result.scene_id
            _write_json(
                output / "observations" / f"{scene_id}.json",
                {**observation.to_json(), "diagnostics": diagnostics},
            )
            row = asdict(result)
            row["outcome"] = result.outcome.value
            row.update(
                (name, diagnostics.get(name))
                for name in SCENE_COLUMNS
                if name not in row
            )
            writer.writerow(row)
            if not args.no_overlay:
                _save_overlay(
                    output / "overlay" / f"{scene_id}.png",
                    sample,
                    observation,
                    result,
                    video=args.video,
                )
            results.append(result)
    metrics = summarize(results)
    record["scene_ids"] = [result.scene_id for result in results]
    record["scene_count"] = len(results)
    if args.video:
        video_error = _make_video(output, args.no_overlay)
        if video_error is not None:
            record["video_error"] = video_error
    record["ended_at_utc"] = _utc_now()
    _write_json(output / "run.json", record)
    _write_json(output / "metrics.json", {**metrics, "run": record})
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """Run one split; configuration and required I/O errors raise with diagnostics."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset", type=Path, required=True, help="v1 dataset directory"
    )
    parser.add_argument("--split", choices=("dev", "eval"), required=True)
    parser.add_argument("--prior", type=Path, required=True, help="pallet prior YAML")
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="new artifacts/<UTC>_pocket_eval_<split>_NN directory",
    )
    parser.add_argument("--params", type=Path, help="DetectorParams overrides as YAML")
    parser.add_argument(
        "--scenes", help="comma-separated scene IDs within the chosen split"
    )
    parser.add_argument(
        "--video", action="store_true", help="encode overlay PNGs at 5 fps"
    )
    parser.add_argument("--no-overlay", action="store_true", help="omit overlay PNGs")
    args = parser.parse_args(argv)
    try:
        return _run(args)
    except Exception as exc:
        print(
            json.dumps(
                {
                    "status": "invalid",
                    "reason": f"exception:{type(exc).__name__}",
                    "diagnostics": {"exception_traceback": traceback.format_exc()},
                }
            ),
            file=sys.stderr,
        )
        raise


if __name__ == "__main__":
    try:
        code = main()
    except Exception:
        code = 1
    raise SystemExit(code)
