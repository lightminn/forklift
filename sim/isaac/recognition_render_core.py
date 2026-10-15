"""Pure helpers for rendering recognition recipes in Isaac (no Isaac imports).

The prims reproduce tools.instance_rig's world exactly -- the recipe's boxes, a
floor whose top is z = 0 and a back wall whose front face is x = BACK_WALL_X_M --
so the CPU-rig truth (stage 1A) applies to the Isaac image.

Every scene is built under its own root (/World/Scene_t<token>) and its labels carry
the scene token. On the 2026-10-15 renders Isaac kept the previous scene's id->label
map for the same 34 of 1,020 scenes across all captures (their depth was the new
scene's); app updates, fresh prim paths and fixed class labels did not help, while one
capture with no labelled prims between scenes did. The token check is what exposes a
kept map, and tools/compare_isaac_rig.py checks every scene against the rig. Fixed
class labels (scene_label(base) without a token) still decode. An annotator that has
not produced a buffer yet is "not ready", not a failure.
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
    stale_tokens: tuple = ()


def scene_root(token: int) -> str:
    return f"/World/Scene_t{token}"


def prim_specs(recipe, root: str = "/World/Scene") -> list[PrimSpec]:
    specs = [
        PrimSpec(f"{root}/floor", (0.0, 0.0, -0.05), (40.0, 40.0, 0.1), 0.0, "floor", CONCRETE),
        PrimSpec(f"{root}/wall", (scene_rig.BACK_WALL_X_M + 0.05, 0.0, 5.0), (0.1, 40.0, 10.0), 0.0,
                 "wall", PAINT),
    ]
    for k in range(len(recipe.instances)):
        for i, box in enumerate(rs.recipe_boxes(recipe, k)):
            specs.append(PrimSpec(f"{root}/inst_{k}/box_{i}", tuple(box.centre_m), tuple(box.size_m),
                                  float(box.yaw_rad), f"inst_{k}", WOOD))
    for i, box in enumerate(recipe.distractors):
        specs.append(PrimSpec(f"{root}/distractor/box_{i}", tuple(box.centre_m), tuple(box.size_m),
                              float(box.yaw_rad), "distractor", GREY))
    return specs


def scene_label(base: str, token: int | None = None) -> str:
    return base if token is None else f"{base}_t{token}"


def parse_label(label: str):
    """(base, token) for token labels, (base, None) for fixed class labels, None for anything else."""
    text = str(label)
    base, sep, token = text.rpartition("_t")
    if sep and base and token.isdigit():
        return base, int(token)
    if text in ("floor", "wall", "distractor") or (text.startswith("inst_") and text[5:].isdigit()):
        return text, None
    return None


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
    stale, floor, stale_tokens = 0, 0, set()
    for key, value in labels.items():
        names = _class_names(value)
        if len(names) > 1:
            raise ValueError(f"semantic id {key} carries several labels: {names}")
        parsed = parse_label(names[0]) if names else None
        if parsed is None:
            continue
        base, tok = parsed
        hit = array == int(key)
        if token is not None and tok != token:  # token None: fixed labels, or any token for diagnosis
            count = int(hit.sum())
            stale += count
            if count:
                stale_tokens.add(tok)
        elif base == "distractor":
            ids[hit] = -2
        elif base.startswith("inst_"):
            ids[hit] = int(base.split("_", 1)[1])
        elif base == "floor":
            floor += int(hit.sum())
    ready = floor > 0 and stale == 0
    return SemanticFrame(ids, ready, "ready" if ready else ("stale" if stale else "floor_missing"), stale,
                         tuple(sorted(stale_tokens)))


def clean_depth(z) -> np.ndarray:
    depth = np.asarray(z, dtype=float)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    depth = depth.copy()
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    return depth


def status_exit_code(path) -> int:
    """Job exit code from status.json: Isaac's close path may end the process before main returns."""
    import json

    try:
        status = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return 1
    return 0 if status.get("rendered") and not status.get("failed") and "fatal" not in status else 1


if __name__ == "__main__":
    sys.exit(status_exit_code(sys.argv[1]))
