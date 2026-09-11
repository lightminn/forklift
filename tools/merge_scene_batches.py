#!/usr/bin/env python3
"""Validate complete scene batches and copy their verified data into a fresh set."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
from collections import Counter
from collections.abc import Sequence
from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml

from forklift_core.perception.scene_dataset import load_scene_sample

SCENE_FILES = frozenset(
    {
        "rgb.png",
        "depth_mm.png",
        "depth_preview.png",
        "depth_meta.json",
        "camera_info.json",
        "tf.json",
        "ground_truth.json",
        "scene.json",
    }
)
METADATA_FIELDS = (
    "catalogue_version",
    "catalogue_sha256",
    "camera",
    "image_id",
    "source_snapshot_sha256",
)
CATEGORY_STATUS = {
    "positive": "valid",
    "occluded": "valid",
    "negative_no_pallet": "no_pallet",
    "negative_lookalike": "no_pallet",
}


def _directory(path: Path) -> None:
    if path.is_symlink() or not path.is_dir():
        raise ValueError(f"expected a real directory, not a symlink: {path}")


def _file(path: Path) -> None:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"expected a regular file, not a symlink: {path}")


def _sha256(path: Path) -> str:
    _file(path)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_json(path: Path) -> dict:
    _file(path)
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _match(value: dict, expected: dict, label: str) -> None:
    for key, required in expected.items():
        if key not in value or value[key] != required:
            raise ValueError(f"{label}: {key} mismatch")


def _scene_ids(values: Any, label: str) -> set[str]:
    if not isinstance(values, list) or not values:
        raise ValueError(f"{label} must contain scene IDs")
    if any(not isinstance(v, str) or not re.fullmatch(r"s[0-9]{3}", v) for v in values):
        raise ValueError(f"{label} contains invalid scene IDs")
    if len(set(values)) != len(values):
        raise ValueError(f"{label} contains duplicate scene IDs")
    return set(values)


def _catalogue(path: Path) -> tuple[dict, dict[str, dict], str]:
    _file(path)
    raw = path.read_bytes()
    try:
        data = yaml.safe_load(raw)
    except yaml.YAMLError as error:
        raise ValueError(f"invalid catalogue YAML: {error}") from error
    if not isinstance(data, dict) or not isinstance(data.get("scenes"), list):
        raise ValueError("catalogue must contain scenes")
    if not isinstance(data.get("catalogue_version"), str) or not isinstance(
        data.get("camera"), dict
    ):
        raise ValueError("catalogue requires catalogue_version and camera")
    entries = data["scenes"]
    if any(not isinstance(entry, dict) for entry in entries):
        raise ValueError("catalogue scenes must be objects")
    _scene_ids([entry.get("scene_id") for entry in entries], "catalogue")
    for entry in entries:
        category = entry.get("category")
        if not isinstance(category, str) or category not in CATEGORY_STATUS:
            raise ValueError("catalogue category is unsupported")
        if entry.get("split") not in ("dev", "eval"):
            raise ValueError("catalogue split is unsupported")
        if not isinstance(entry.get("ground_truth"), dict):
            raise ValueError("catalogue ground_truth must be an object")
        if entry["ground_truth"].get("status") != CATEGORY_STATUS[category]:
            raise ValueError("catalogue ground_truth status does not match category")
    return data, {e["scene_id"]: e for e in entries}, hashlib.sha256(raw).hexdigest()


def _verify_scene(
    scene_dir: Path, result: dict, entry: dict, batch: dict
) -> dict[str, str]:
    scene_id = entry["scene_id"]
    if (
        not isinstance(result, dict)
        or result.get("passed") is not True
        or result.get("error")
    ):
        raise ValueError(f"failed scene or scene error: {scene_id}")
    files = result.get("files")
    if not isinstance(files, dict) or set(files) != SCENE_FILES:
        raise ValueError(f"scene files must name all eight capture files: {scene_id}")
    for name, digest in files.items():
        if _sha256(scene_dir / name) != digest:
            raise ValueError(f"scene file hash mismatch: {scene_id}/{name}")
    scene = _read_json(scene_dir / "scene.json")
    _match(
        scene,
        {
            "scene_id": scene_id,
            "category": entry["category"],
            "split": entry["split"],
            "catalogue_version": batch["catalogue_version"],
            "camera": batch["camera"],
            "image_id": batch["image_id"],
            "source_snapshot_sha256": batch["source_snapshot_sha256"],
            "run_id": batch["run_id"],
            "clock_domain": "ros_sim",
            "source_provenance": "synthetic",
        },
        f"scene {scene_id}",
    )
    if "stamp_ns" not in scene:
        raise ValueError(f"scene {scene_id}: missing stamp_ns")
    expected_truth = deepcopy(entry["ground_truth"])
    expected_truth.update(stamp_ns=scene["stamp_ns"], clock_domain="ros_sim")
    if _read_json(scene_dir / "ground_truth.json") != expected_truth:
        raise ValueError(f"ground_truth differs from catalogue: {scene_id}")
    # The actual loader checks PNG bit depth/shape, calibration, TF and timestamps.
    load_scene_sample(scene_dir)
    return dict(sorted(files.items()))


def merge_batches(*, catalogue: Path, batches: Sequence[Path], output: Path) -> dict:
    """Validate complete catalogue coverage before copying the eight data files.

    Existing outputs are never overwritten. Logs/world/runtime files stay in the
    collected batch. Failed batches must be rerun in full with a new run ID.
    """
    output = Path(output)
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"output already exists: {output}")
    data, entries, catalogue_sha256 = _catalogue(Path(catalogue))
    if not batches:
        raise ValueError("at least one batch is required")
    expected_metadata = {
        "catalogue_version": data["catalogue_version"],
        "catalogue_sha256": catalogue_sha256,
        "camera": data["camera"],
    }
    records: dict[str, dict] = {}
    sources: dict[str, Path] = {}
    for batch_path in batches:
        batch_path = Path(batch_path)
        _directory(batch_path)
        capture = batch_path / "scene_capture"
        _directory(capture)
        batch = _read_json(capture / "manifest.json")
        _match(batch, expected_metadata, f"batch {batch_path}")
        for key in ("image_id", "source_snapshot_sha256", "run_id"):
            if not isinstance(batch.get(key), str) or not batch[key]:
                raise ValueError(f"batch requires {key}")
        expected_metadata = {key: batch[key] for key in METADATA_FIELDS}
        if (
            type(batch.get("failed_count")) is not int
            or batch["failed_count"] != 0
            or batch.get("error")
        ):
            raise ValueError(f"failed batch or batch error: {batch_path}")
        requested = _scene_ids(batch.get("requested_scenes"), "requested_scenes")
        results = batch.get("scenes")
        if not isinstance(results, dict) or set(results) != requested:
            raise ValueError("requested scene IDs and result keys differ")
        if requested - entries.keys():
            raise ValueError("requested scene IDs are absent from catalogue")
        if requested & records.keys():
            raise ValueError("duplicate scene IDs across batches")
        scene_root = capture / "scenes"
        _directory(scene_root)
        children = list(scene_root.iterdir())
        if {p.name for p in children} != requested:
            raise ValueError("scene directory names and requested IDs differ")
        for child in children:
            _directory(child)
        for scene_id in sorted(requested):
            scene_dir = scene_root / scene_id
            files = _verify_scene(
                scene_dir, results[scene_id], entries[scene_id], batch
            )
            records[scene_id] = {"batch_run_id": batch["run_id"], "files": files}
            sources[scene_id] = scene_dir
    missing = entries.keys() - records.keys()
    if missing:
        raise ValueError(f"missing catalogue scene coverage: {sorted(missing)}")
    manifest = {
        **expected_metadata,
        "scenes": dict(sorted(records.items())),
        "category_counts": dict(
            sorted(Counter(e["category"] for e in entries.values()).items())
        ),
        "split_counts": dict(
            sorted(Counter(e["split"] for e in entries.values()).items())
        ),
    }
    output.mkdir(parents=True, exist_ok=False)
    try:
        for scene_id, record in manifest["scenes"].items():
            destination = output / "scenes" / scene_id
            destination.mkdir(parents=True)
            for name, digest in record["files"].items():
                shutil.copyfile(sources[scene_id] / name, destination / name)
                if _sha256(destination / name) != digest:
                    raise ValueError(f"copied file hash mismatch: {scene_id}/{name}")
        (output / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    except BaseException:
        # Only this invocation's newly created output is removed on copy failure.
        shutil.rmtree(output)
        raise
    return manifest


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalogue", required=True, type=Path)
    parser.add_argument(
        "--batches",
        required=True,
        nargs="+",
        type=Path,
        help="collected batch roots containing scene_capture/",
    )
    parser.add_argument(
        "--output", required=True, type=Path, help="new dataset directory"
    )
    args = parser.parse_args(argv)
    manifest = merge_batches(
        catalogue=args.catalogue, batches=args.batches, output=args.output
    )
    print(
        json.dumps(
            {
                "output": str(args.output),
                "scene_count": len(manifest["scenes"]),
                "category_counts": manifest["category_counts"],
                "split_counts": manifest["split_counts"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2) from error
