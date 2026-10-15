"""Pure helpers for rendering recognition recipes in Isaac (no Isaac imports).

The prims reproduce tools.instance_rig's world exactly -- the recipe's boxes, a
floor whose top is z = 0 and a back wall whose front face is x = BACK_WALL_X_M --
so the CPU-rig truth (stage 1A) applies to the Isaac image. Every label carries a
per-scene token: a frame that still shows the previous scene's labels is stale,
and an annotator that has not produced a buffer yet is "not ready", not a failure.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools import recognition_scenes as rs  # noqa: E402
from tools import scene_rig  # noqa: E402

WOOD, GREY, CONCRETE, PAINT = (0.62, 0.45, 0.28), (0.5, 0.5, 0.52), (0.55, 0.55, 0.55), (0.85, 0.85, 0.8)


@dataclass(frozen=True)
class PrimSpec:
    path: str
    centre_m: tuple
    size_m: tuple
    yaw_rad: float
    base_label: str
    colour: tuple


@dataclass(frozen=True)
class SemanticFrame:
    instance_ids: np.ndarray | None
    ready: bool
    status: str
    stale_pixels: int


def prim_specs(recipe) -> list[PrimSpec]:
    specs = [
        PrimSpec("/World/Scene/floor", (0.0, 0.0, -0.05), (40.0, 40.0, 0.1), 0.0, "floor", CONCRETE),
        PrimSpec("/World/Scene/wall", (scene_rig.BACK_WALL_X_M + 0.05, 0.0, 5.0), (0.1, 40.0, 10.0), 0.0,
                 "wall", PAINT),
    ]
    for k in range(len(recipe.instances)):
        for i, box in enumerate(rs.recipe_boxes(recipe, k)):
            specs.append(PrimSpec(f"/World/Scene/inst_{k}/box_{i}", tuple(box.centre_m), tuple(box.size_m),
                                  float(box.yaw_rad), f"inst_{k}", WOOD))
    for i, box in enumerate(recipe.distractors):
        specs.append(PrimSpec(f"/World/Scene/distractor/box_{i}", tuple(box.centre_m), tuple(box.size_m),
                              float(box.yaw_rad), "distractor", GREY))
    return specs


def scene_label(base: str, token: int) -> str:
    return f"{base}_t{token}"


def parse_label(label: str):
    base, sep, token = str(label).rpartition("_t")
    if not sep or not base or not token.isdigit():
        return None
    return base, int(token)


def _class_names(value) -> list[str]:
    classes = value.get("class", []) if isinstance(value, dict) else []
    if isinstance(classes, str):
        classes = [c for c in classes.split(",") if c]
    return list(classes)


def _not_ready(status: str) -> SemanticFrame:
    return SemanticFrame(None, False, status, 0)


def decode_semantic(payload, token: int, shape=(480, 640)) -> SemanticFrame:
    if not isinstance(payload, dict) or payload.get("data") is None:
        return _not_ready("missing_buffer")
    array = np.asarray(payload["data"])
    if not array.size:
        return _not_ready("empty_buffer")
    info = payload.get("info")
    labels = info.get("idToLabels") if isinstance(info, dict) else None
    if not isinstance(labels, dict) or not labels:
        return _not_ready("missing_label_map")
    if array.ndim == 3 and array.shape[-1] == 1:
        array = array[..., 0]
    if array.dtype.kind not in "ui":
        raise ValueError("semantic segmentation must be an uncolorized integer grid")
    if array.shape != tuple(shape):
        return _not_ready("shape_mismatch")
    ids = np.full(array.shape, -1, dtype=np.int32)
    stale, floor = 0, 0
    for key, value in labels.items():
        names = _class_names(value)
        if len(names) > 1:
            raise ValueError(f"semantic id {key} carries several labels: {names}")
        parsed = parse_label(names[0]) if names else None
        if parsed is None:
            continue
        base, tok = parsed
        hit = array == int(key)
        if tok != token:
            stale += int(hit.sum())
        elif base == "distractor":
            ids[hit] = -2
        elif base.startswith("inst_"):
            ids[hit] = int(base.split("_", 1)[1])
        elif base == "floor":
            floor += int(hit.sum())
    ready = floor > 0 and stale == 0
    return SemanticFrame(ids, ready, "ready" if ready else ("stale" if stale else "floor_missing"), stale)


def clean_depth(z) -> np.ndarray:
    depth = np.asarray(z, dtype=float)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    depth = depth.copy()
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    return depth
