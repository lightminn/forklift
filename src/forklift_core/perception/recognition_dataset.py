# src/forklift_core/perception/recognition_dataset.py
"""Scene dataset schema version 2: multi-instance truth beside the v1 input files.

Input files keep the v1 encodings and checks. tf.json names one of TF_ORIGINS
and a rendered pose is named as rendered, never as a received /tf_static
(ADR 0004 D5 amendment, 2026-10-15). instruction.json holds the system input
only; whether an instruction is ambiguous and which instance it means are
evaluator truth in truth.json. The input loader never opens truth.json or the
instance image.
"""

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from forklift_core._validation import _finite_scalar
from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.fork_fit import LoadDeclaration
from forklift_core.perception.pocket_observation import CLOCK_DOMAINS, PROVENANCES
from forklift_core.perception.scene_dataset import (
    SceneInput, _nonnegative_int, _read_json, _read_png, _require_fields, decode_depth_mm, intrinsics_from_camera_info,
)

SCHEMA_VERSION = 2
TF_ORIGINS = ("received_tf_static", "isaac_render_pose", "rig_render_pose")
ORIGIN_SOURCE = {"rig_render_pose": "rig", "isaac_render_pose": "isaac", "received_tf_static": "capture"}
_DISTRACTOR = 65535


@dataclass(frozen=True)
class RecognitionInput:
    scene: SceneInput
    instruction: dict


@dataclass(frozen=True)
class RecognitionSample:
    input: RecognitionInput
    origin: str
    meta: dict
    truth: dict
    instruction: dict
    instance_ids: np.ndarray


def _quaternion_xyzw(rotation) -> list[float]:
    """Shepperd's method: branch on the largest of w, x, y, z."""
    m = np.asarray(rotation, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    i = int(np.argmax((trace, m[0, 0], m[1, 1], m[2, 2])))
    if i == 0:
        s = 2.0 * np.sqrt(1.0 + trace)
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4)
    elif i == 1:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = (s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif i == 2:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s)
    q = np.asarray(q) / np.linalg.norm(q)
    return [float(v) for v in q]


INSTRUCTION_FIELDS = ("present", "x_m", "y_m", "radius_m", "layer", "load")


def check_instruction(obj) -> dict:
    """System-input instruction only: no evaluator truth (kind, target, ...) may ride along."""
    if not isinstance(obj, dict) or set(obj) - set(INSTRUCTION_FIELDS) or "present" not in obj:
        raise ValueError(f"instruction input may hold only {INSTRUCTION_FIELDS}, with present")
    if not isinstance(obj["present"], bool):
        raise ValueError("instruction present must be a bool")
    if obj["present"]:
        for name in ("x_m", "y_m"):
            _finite_scalar(obj.get(name), f"instruction {name}")
        if _finite_scalar(obj.get("radius_m"), "instruction radius_m") <= 0:
            raise ValueError("instruction radius_m must be positive")
        layer = obj.get("layer")
        if layer is not None and (isinstance(layer, bool) or not isinstance(layer, int) or layer < 0):
            raise ValueError("instruction layer must be a nonnegative int or null")
    if obj.get("load") is not None:
        LoadDeclaration(**obj["load"])  # validates names, finiteness and source
    return obj


def _png(path: Path, array) -> None:
    from PIL import Image

    Image.fromarray(np.ascontiguousarray(array)).save(path, format="PNG")


def _check_meta(origin: str, meta: dict) -> None:
    if origin not in TF_ORIGINS:
        raise ValueError(f"tf origin must be one of {TF_ORIGINS}")
    expected = ORIGIN_SOURCE.get(origin)
    if expected is None or meta.get("render_source") != expected:
        raise ValueError(f"render_source must be {expected!r} for origin {origin!r}")


def write_recognition_scene(scene_dir, scene: SceneInput, instance_ids, *, origin, meta, truth, instruction) -> None:
    _check_meta(origin, meta)
    check_instruction(instruction)
    scene_dir = Path(scene_dir)
    scene_dir.mkdir(parents=True, exist_ok=True)
    depth = scene.depth_m
    ok = np.isfinite(depth) & (depth > 0)
    if np.any(depth[ok] > 65.535):
        raise ValueError("Depth exceeds the uint16 millimetre range")
    raw = np.zeros(depth.shape, dtype=np.uint16)
    raw[ok] = np.rint(depth[ok] * 1000).astype(np.uint16)
    ids = np.asarray(instance_ids)
    codes = np.where(ids >= 0, ids + 1, np.where(ids == -2, _DISTRACTOR, 0)).astype(np.uint16)
    i = scene.intrinsics
    k = [i.fx, 0.0, i.cx, 0.0, i.fy, i.cy, 0.0, 0.0, 1.0]
    files = {
        "depth_meta.json": {"unit": "mm", "meters_per_unit": 0.001, "unknown_value": 0, "kind": "optical_axis_z"},
        "camera_info.json": {
            "frame_id": i.frame_id, "stamp_ns": scene.stamp_ns, "width": i.width, "height": i.height,
            "distortion_model": "plumb_bob", "d": [0.0] * 5, "k": k, "r": np.eye(3).ravel().tolist(),
            "p": np.column_stack((np.array(k).reshape(3, 3), np.zeros(3))).ravel().tolist(),
            "binning_x": 0, "binning_y": 0,
            "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False},
        },
        "tf.json": {
            "target_frame": scene.base_from_optical.target_frame, "source_frame": scene.base_from_optical.source_frame,
            "translation_m": [float(v) for v in scene.base_from_optical.translation_m],
            "quaternion_xyzw": _quaternion_xyzw(scene.base_from_optical.rotation), "origin": origin,
        },
        "scene.json": {**meta, "schema_version": SCHEMA_VERSION, "stamp_ns": scene.stamp_ns,
                       "clock_domain": scene.clock_domain, "source_provenance": scene.source_provenance},
        "truth.json": truth,
        "instruction.json": instruction,
    }
    for name, obj in files.items():
        (scene_dir / name).write_text(json.dumps(obj, indent=1, sort_keys=True), encoding="utf-8")
    _png(scene_dir / "rgb.png", np.asarray(scene.rgb, dtype=np.uint8))
    _png(scene_dir / "depth_mm.png", raw)
    _png(scene_dir / "instances.png", codes)


def read_meta(scene_dir) -> dict:
    meta = _read_json(Path(scene_dir) / "scene.json")
    if meta.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("scene.json schema_version must be 2")
    _require_fields(meta, {"scene_id", "case", "split", "render_source", "stamp_ns", "clock_domain",
                           "source_provenance"}, "scene", allow_extra=True)
    meta["stamp_ns"] = _nonnegative_int(meta["stamp_ns"], "scene.stamp_ns")
    for name, allowed in (("clock_domain", CLOCK_DOMAINS), ("source_provenance", PROVENANCES)):
        if not isinstance(meta[name], str) or meta[name] not in allowed:
            raise ValueError(f"Unsupported scene {name}: {meta[name]!r}")
    return meta


def _load_input(scene_dir: Path):
    meta = read_meta(scene_dir)
    info = _read_json(scene_dir / "camera_info.json")
    intrinsics = intrinsics_from_camera_info(info)
    tf = _read_json(scene_dir / "tf.json")
    _require_fields(tf, {"target_frame", "source_frame", "translation_m", "quaternion_xyzw", "origin"}, "tf")
    _check_meta(tf["origin"], meta)
    if tf["target_frame"] != "base_link" or tf["source_frame"] != "camera_optical_frame":
        raise ValueError("TF must map camera_optical_frame to base_link")
    if intrinsics.frame_id != tf["source_frame"]:
        raise ValueError("CameraInfo frame must match TF source_frame")
    if _nonnegative_int(info["stamp_ns"], "camera_info.stamp_ns") != meta["stamp_ns"]:
        raise ValueError("CameraInfo and scene acquisition stamps must match")
    transform = RigidTransform(source_frame=tf["source_frame"], target_frame=tf["target_frame"],
                               rotation=rotation_matrix_from_quaternion_xyzw(tf["quaternion_xyzw"]),
                               translation_m=tf["translation_m"])
    rgb = _read_png(scene_dir / "rgb.png", depth=False)
    depth = decode_depth_mm(_read_png(scene_dir / "depth_mm.png", depth=True), _read_json(scene_dir / "depth_meta.json"))
    shape = (intrinsics.height, intrinsics.width)
    if rgb.shape != (*shape, 3) or depth.shape != shape:
        raise ValueError("RGB and depth shapes must match CameraInfo width/height")
    scene = SceneInput(rgb=rgb, depth_m=depth, intrinsics=intrinsics, base_from_optical=transform,
                       stamp_ns=int(meta["stamp_ns"]), clock_domain=meta["clock_domain"],
                       source_provenance=meta["source_provenance"])
    return RecognitionInput(scene, check_instruction(_read_json(scene_dir / "instruction.json"))), tf["origin"], meta


def load_recognition_input(scene_dir) -> RecognitionInput:
    given, _, _ = _load_input(Path(scene_dir))
    return given


def load_recognition_sample(scene_dir) -> RecognitionSample:
    scene_dir = Path(scene_dir)
    given, origin, meta = _load_input(scene_dir)
    codes = _read_png(scene_dir / "instances.png", depth=True).astype(np.int64)
    if codes.shape != given.scene.depth_m.shape:
        raise ValueError("instances.png must match the depth grid")
    ids = np.where(codes == _DISTRACTOR, -2, codes - 1).astype(np.int32)
    return RecognitionSample(given, origin, meta, _read_json(scene_dir / "truth.json"), given.instruction, ids)


def scene_digest(scene_dir) -> str:
    digest = hashlib.sha256()
    for path in sorted(Path(scene_dir).iterdir()):
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()
