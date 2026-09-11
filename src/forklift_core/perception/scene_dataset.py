"""Load v1 synthetic PNG/JSON scenes without ROS or hardware SDK dependencies.

Depth is optical-axis z in metres (float64, NaN unknown). Downstream deprojection
must use meters_per_unit=1.0. SceneInput carries no evaluation labels.
"""

import json
from dataclasses import dataclass
from numbers import Integral
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from forklift_core._validation import _finite_scalar, _real_array
from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.pocket_observation import (
    CLOCK_DOMAINS,
    OBSERVATION_FRAME,
    PROVENANCES,
    PocketObservation,
    pocket_observation_from_json,
)
from forklift_core.sensors.rgbd import PinholeIntrinsics


@dataclass(frozen=True)
class SceneInput:
    """Recognizer input: RGB uint8[H,W,3], metric axial depth float64[H,W].

    NaN depth is unknown. The v1 capture contract supplies a rectified grid with
    RGB registered to it; equal image dimensions alone do not prove registration.
    Acquisition time and provenance come from scene.json. No ground truth is read.
    """

    rgb: NDArray[np.uint8]
    depth_m: NDArray[np.float64]
    intrinsics: PinholeIntrinsics
    base_from_optical: RigidTransform
    stamp_ns: int
    clock_domain: str
    source_provenance: str
    rectified: bool = True
    rgb_registered_to_depth_grid: bool = True

    @property
    def pixel_frame(self) -> str:
        """Coordinate frame of the rectified depth grid."""
        return self.intrinsics.frame_id


@dataclass(frozen=True)
class SceneSample:
    """Evaluator-owned input, ground truth and complete scene.json metadata."""

    input: SceneInput
    ground_truth: PocketObservation
    scene: dict


def _require_fields(
    obj: dict, names: set[str], label: str, *, allow_extra: bool = False
) -> None:
    if not isinstance(obj, dict):
        raise ValueError(f"{label} must be a JSON object")
    missing = names - obj.keys()
    if missing:
        raise ValueError(f"{label} is missing fields: {sorted(missing)}")
    if not allow_extra and obj.keys() - names:
        raise ValueError(f"{label} contains unknown fields")


def _nonnegative_int(value: int, name: str) -> int:
    if (
        isinstance(value, (bool, np.bool_))
        or not isinstance(value, Integral)
        or value < 0
    ):
        raise ValueError(f"{name} must be a nonnegative integer, excluding bool")
    return int(value)


def intrinsics_from_camera_info(obj: dict) -> PinholeIntrinsics:
    """Read the exact rectified monocular grid; reject unsupported CameraInfo.

    D must be zero, R identity, K canonical (no skew), P=[K|0], binning 0/1,
    and ROI empty. Intrinsic values are preserved without half-pixel adjustment.
    """
    _require_fields(
        obj,
        {
            "frame_id",
            "stamp_ns",
            "width",
            "height",
            "distortion_model",
            "d",
            "k",
            "r",
            "p",
            "binning_x",
            "binning_y",
            "roi",
        },
        "camera_info",
    )
    _nonnegative_int(obj["stamp_ns"], "camera_info.stamp_ns")
    arrays = {}
    for name, shape in (("k", (9,)), ("r", (9,)), ("p", (12,))):
        array = _real_array(obj[name], name)
        if array.shape != shape or not np.isfinite(array).all():
            raise ValueError(f"CameraInfo {name} must be finite with shape {shape}")
        arrays[name] = array
    d = _real_array(obj["d"], "d")
    if d.ndim != 1 or not np.all(d == 0):
        raise ValueError("CameraInfo D must contain only zero distortion coefficients")
    k, r, p = (
        arrays["k"].reshape(3, 3),
        arrays["r"].reshape(3, 3),
        arrays["p"].reshape(3, 4),
    )
    canonical_k = np.array([[k[0, 0], 0, k[0, 2]], [0, k[1, 1], k[1, 2]], [0, 0, 1]])
    if not np.array_equal(k, canonical_k):
        raise ValueError("CameraInfo K must be canonical with zero skew")
    if not np.array_equal(r, np.eye(3)):
        raise ValueError("CameraInfo R must be identity")
    if not np.array_equal(p[:, :3], k) or not np.all(p[:, 3] == 0):
        raise ValueError("CameraInfo P must equal [K|0] (no stereo baseline)")
    for name in ("binning_x", "binning_y"):
        if _nonnegative_int(obj[name], name) not in (0, 1):
            raise ValueError("CameraInfo binning must be 0 or 1")
    roi = obj["roi"]
    _require_fields(
        roi, {"x_offset", "y_offset", "height", "width", "do_rectify"}, "roi"
    )
    for name in ("x_offset", "y_offset", "height", "width"):
        if _nonnegative_int(roi[name], f"roi.{name}") != 0:
            raise ValueError("CameraInfo ROI must be empty")
    if roi["do_rectify"] is not False:
        raise ValueError("CameraInfo roi.do_rectify must be false")
    return PinholeIntrinsics(
        width=obj["width"],
        height=obj["height"],
        fx=float(k[0, 0]),
        fy=float(k[1, 1]),
        cx=float(k[0, 2]),
        cy=float(k[1, 2]),
        frame_id=obj["frame_id"],
    )


def transform_from_tf_json(obj: dict) -> RigidTransform:
    """Read received /tf_static: camera_optical_frame -> base_link, translation in m."""
    _require_fields(
        obj,
        {"target_frame", "source_frame", "translation_m", "quaternion_xyzw", "origin"},
        "tf",
    )
    if (
        obj["target_frame"] != OBSERVATION_FRAME
        or obj["source_frame"] != "camera_optical_frame"
    ):
        raise ValueError("TF must map camera_optical_frame to base_link")
    if obj["origin"] != "received_tf_static":
        raise ValueError("TF origin must be received_tf_static")
    return RigidTransform(
        source_frame=obj["source_frame"],
        target_frame=obj["target_frame"],
        rotation=rotation_matrix_from_quaternion_xyzw(obj["quaternion_xyzw"]),
        translation_m=obj["translation_m"],
    )


def decode_depth_mm(raw: NDArray, meta: dict) -> NDArray[np.float64]:
    """Decode v1 uint16[H,W] millimetre axial depth into float64 metres, 0 -> NaN.

    Metadata must specify optical_axis_z, mm, exact scale 0.001 and integer
    sentinel 0. Input is not modified; other formats raise ValueError.
    """
    _require_fields(
        meta, {"unit", "meters_per_unit", "unknown_value", "kind"}, "depth_meta"
    )
    if meta["unit"] != "mm" or meta["kind"] != "optical_axis_z":
        raise ValueError("Depth v1 requires millimetres and optical_axis_z")
    scale = _finite_scalar(meta["meters_per_unit"], "meters_per_unit")
    if scale != 0.001:
        raise ValueError("Depth v1 meters_per_unit must be exactly 0.001")
    unknown = _nonnegative_int(meta["unknown_value"], "unknown_value")
    if unknown != 0:
        raise ValueError("Depth v1 unknown_value must be integer zero")
    if not isinstance(raw, np.ndarray) or raw.ndim != 2 or raw.dtype != np.uint16:
        raise ValueError("Depth v1 raw must be a two-dimensional uint16 array")
    depth_m = raw.astype(np.float64) * scale
    depth_m[raw == unknown] = np.nan
    return depth_m


def _read_json(path: Path) -> dict:
    obj = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(obj, dict):
        raise ValueError(f"{path.name} must contain a JSON object")
    return obj


def _read_png(path: Path, *, depth: bool) -> NDArray:
    try:
        from PIL import Image
    except ImportError as exc:
        raise ImportError(
            "Scene loading requires pip install 'forklift-core[dataset]'"
        ) from exc

    try:
        with Image.open(path) as image:
            if image.format != "PNG":
                raise ValueError(f"{path.name} must be PNG, regardless of extension")
            # Pillow exposes even 16-bit RGB PNG as RGB uint8. Check the original
            # IHDR bit depth before trusting the decoded mode or dtype.
            with path.open("rb") as stream:
                header = stream.read(26)
            if len(header) != 26 or header[8:16] != b"\x00\x00\x00\rIHDR":
                raise ValueError("PNG must start with a complete IHDR chunk")
            expected_format = (16, 0) if depth else (8, 2)
            if (header[24], header[25]) != expected_format:
                raise ValueError("PNG must store 16-bit grayscale depth or 8-bit RGB")
            if not depth:
                if image.mode != "RGB":
                    raise ValueError("rgb.png must be 8-bit RGB")
                return np.array(image)
            if image.mode in {"I;16", "I;16L", "I;16B", "I;16N"}:
                return np.asarray(image).astype(np.uint16)
            if image.mode == "I":
                # Older Pillow versions expose 16-bit PNG samples as signed int32.
                raw = np.asarray(image)
                if np.any(raw < 0) or np.any(raw > 65535):
                    raise ValueError("Depth PNG samples must lie in [0, 65535]")
                return raw.astype(np.uint16)
            raise ValueError("depth_mm.png must be a 16-bit single-channel PNG")
    except FileNotFoundError:
        raise
    except OSError as exc:
        raise ValueError(f"Cannot decode PNG: {path.name}") from exc


def _load_input_and_scene(scene_dir: Path) -> tuple[SceneInput, dict]:
    scene = _read_json(scene_dir / "scene.json")
    _require_fields(
        scene,
        {
            "scene_id",
            "catalogue_version",
            "category",
            "split",
            "stamp_ns",
            "clock_domain",
            "source_provenance",
        },
        "scene",
        allow_extra=True,
    )
    stamp_ns = _nonnegative_int(scene["stamp_ns"], "scene.stamp_ns")
    for name, allowed in (
        ("clock_domain", CLOCK_DOMAINS),
        ("source_provenance", PROVENANCES),
    ):
        if not isinstance(scene[name], str) or scene[name] not in allowed:
            raise ValueError(f"Unsupported scene {name}: {scene[name]!r}")
    camera_info = _read_json(scene_dir / "camera_info.json")
    intrinsics = intrinsics_from_camera_info(camera_info)
    base_from_optical = transform_from_tf_json(_read_json(scene_dir / "tf.json"))
    if camera_info["stamp_ns"] != stamp_ns:
        raise ValueError("CameraInfo and scene acquisition stamps must match")
    if intrinsics.frame_id != base_from_optical.source_frame:
        raise ValueError("CameraInfo frame must match TF source_frame")
    rgb = _read_png(scene_dir / "rgb.png", depth=False)
    raw = _read_png(scene_dir / "depth_mm.png", depth=True)
    depth_m = decode_depth_mm(raw, _read_json(scene_dir / "depth_meta.json"))
    shape = (intrinsics.height, intrinsics.width)
    if rgb.shape != (*shape, 3) or rgb.dtype != np.uint8 or depth_m.shape != shape:
        raise ValueError("RGB and depth shapes must match CameraInfo width/height")
    return SceneInput(
        rgb=rgb,
        depth_m=depth_m,
        intrinsics=intrinsics,
        base_from_optical=base_from_optical,
        stamp_ns=stamp_ns,
        clock_domain=scene["clock_domain"],
        source_provenance=scene["source_provenance"],
    ), scene


def load_scene_input(scene_dir: Path) -> SceneInput:
    """Load only recognizer input; never read ground_truth.json or expose scene labels."""
    given, _ = _load_input_and_scene(Path(scene_dir))
    return given


def load_scene_sample(scene_dir: Path) -> SceneSample:
    """Load evaluator input and truth, requiring identical acquisition stamp and clock."""
    scene_dir = Path(scene_dir)
    given, scene = _load_input_and_scene(scene_dir)
    ground_truth = pocket_observation_from_json(
        _read_json(scene_dir / "ground_truth.json")
    )
    if (
        ground_truth.stamp_ns != given.stamp_ns
        or ground_truth.clock_domain != given.clock_domain
    ):
        raise ValueError(
            "Ground truth and scene must share acquisition stamp and clock_domain"
        )
    return SceneSample(input=given, ground_truth=ground_truth, scene=scene)
