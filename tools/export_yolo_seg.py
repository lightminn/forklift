#!/usr/bin/env python3
"""Training renders -> a YOLO segmentation dataset (stage 3 plan, task 3).

Reads <renders>/<scene>/{rgb.png, instances.png, meta.json} (sim/isaac/render_recognition_scenes.py
--catalogue train) and writes <out>/images/{train,val}/<scene>.png, <out>/labels/{train,val}/<scene>.txt and
<out>/data.yaml. One class, ``pallet``; distractors (65535) and background (0) are background.

One instance is one label row (Codex): an instance split by occlusion into several parts keeps its parts
joined into one polygon by thin bridges between their nearest points (the Ultralytics COCO converter's
``merge_multi_segment`` idea); holes (background seen through pockets) are not representable in a YOLO polygon
and are left out. The split into train / val is by a hash of the scene id (val ``--val-percent``).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np

MIN_AREA_PX = 30


def instance_polygons(codes: np.ndarray, *, min_area_px: int = MIN_AREA_PX) -> list:
    """[(instance id, polygon (n, 2) float pixel coordinates)] -- one polygon per instance (parts joined)."""
    import cv2  # noqa: PLC0415

    out = []
    for code in sorted(int(c) for c in np.unique(codes) if 0 < c < 65535):
        mask = (codes == code).astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        parts = [c.reshape(-1, 2).astype(float) for c in contours if cv2.contourArea(c) >= min_area_px or len(c) >= 3]
        parts = [p for p in parts if len(p) >= 3]
        if not parts or int(mask.sum()) < min_area_px:
            continue
        out.append((code - 1, join_parts(parts)))
    return out


def join_parts(parts: list) -> np.ndarray:
    """Join polygon parts into one ring: walk each part, bridging to the next part at their nearest points and
    coming back the same way (zero-width bridges), largest part first."""
    if len(parts) == 1:
        return parts[0]
    parts = sorted(parts, key=lambda p: -len(p))
    ring = parts[0]
    for part in parts[1:]:
        d = np.linalg.norm(ring[:, None, :] - part[None, :, :], axis=2)
        i, j = np.unravel_index(int(np.argmin(d)), d.shape)
        part_rolled = np.roll(part, -j, axis=0)
        ring = np.concatenate([ring[: i + 1], part_rolled, part_rolled[:1], ring[i:]])
    return ring


def label_lines(codes: np.ndarray) -> list[str]:
    h, w = codes.shape
    lines = []
    for _, poly in instance_polygons(codes):
        xy = poly / np.array([w, h])
        lines.append("0 " + " ".join(f"{v:.6f}" for v in np.clip(xy, 0, 1).ravel()))
    return lines


def split_of(scene_id: str, val_percent: int) -> str:
    return "val" if int(hashlib.sha256(scene_id.encode()).hexdigest(), 16) % 100 < val_percent else "train"


def export(renders, out: Path, *, val_percent: int = 10) -> dict:
    """Render folders (one or a list of shards) -> one YOLO dataset in a new ``out``. An existing export is refused
    (Codex review 1: re-exporting at another val share left a scene in both splits), and so is a scene id found in
    two shards."""
    from PIL import Image  # noqa: PLC0415

    shards = [Path(renders)] if isinstance(renders, (str, Path)) else [Path(r) for r in renders]
    scenes = [p for shard in shards for p in sorted(shard.iterdir()) if (p / "rgb.png").exists()]
    names = [p.name for p in scenes]
    if len(set(names)) != len(names):
        raise ValueError("a scene id appears in more than one render shard")
    if (out / "images").exists() or (out / "labels").exists():
        raise FileExistsError(f"{out} already holds an export; export into a new folder")
    counts = {"train": 0, "val": 0, "instances": 0, "scenes_without_instances": 0}
    for split in ("train", "val"):
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)
    for scene in scenes:
        codes = np.asarray(Image.open(scene / "instances.png")).astype(np.int64)
        lines = label_lines(codes)
        split = split_of(scene.name, val_percent)
        shutil.copyfile(scene / "rgb.png", out / "images" / split / f"{scene.name}.png")
        (out / "labels" / split / f"{scene.name}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
        counts[split] += 1
        counts["instances"] += len(lines)
        counts["scenes_without_instances"] += not lines
    train = {p.stem for p in (out / "images" / "train").iterdir()}
    val = {p.stem for p in (out / "images" / "val").iterdir()}
    if train & val or len(train) != counts["train"] or len(val) != counts["val"]:
        raise RuntimeError("the export's splits do not match the scenes written")
    (out / "data.yaml").write_text(f"path: {out.resolve()}\ntrain: images/train\nval: images/val\nnames:\n  0: pallet\n")
    (out / "export.json").write_text(json.dumps({"renders": [str(r) for r in shards], "val_percent": val_percent,
                                                 **counts}, indent=1))
    return counts


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("renders", type=Path, nargs="+", help="render shard folders")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--val-percent", type=int, default=10)
    args = ap.parse_args(argv)
    print(json.dumps(export(args.renders, args.out, val_percent=args.val_percent)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
