"""ROS-independent encoding and synchronization for the v1 scene producer."""

import hashlib
import json
from copy import deepcopy
from numbers import Integral
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

OPTICAL_FRAME = "camera_optical_frame"


def _integer(value: Any, name: str) -> int:
    if (
        isinstance(value, (bool, np.bool_))
        or not isinstance(value, Integral)
        or value < 0
    ):
        raise ValueError(f"{name} must be a nonnegative integer")
    return int(value)


def stamp_to_ns(stamp: Any) -> int:
    """Convert the acquisition header stamp using integer arithmetic only."""
    sec = _integer(stamp.sec, "sec")
    nanosec = _integer(stamp.nanosec, "nanosec")
    if nanosec >= 1_000_000_000:
        raise ValueError("nanosec must be below 1_000_000_000")
    return sec * 1_000_000_000 + nanosec


def depth_to_millimetre_png_array(depth_m: NDArray) -> NDArray[np.uint16]:
    """Convert float32/64[H,W] axial metres to uint16 mm; 0 means unknown.

    Nonfinite/nonpositive samples become 0. Finite positive values above 65.535 m
    fail before rounding; float32 samples are compared in float64, without
    rounding the upper bound to the input precision. Input is not modified.
    """
    if (
        not isinstance(depth_m, np.ndarray)
        or depth_m.ndim != 2
        or depth_m.dtype.kind != "f"
        or depth_m.dtype.itemsize not in (4, 8)
    ):
        raise ValueError("depth_m must be a two-dimensional float32/64 array")
    values = depth_m.astype(np.float64)
    known = np.isfinite(values) & (values > 0)
    if np.any(values[known] > 65.535):
        raise ValueError("finite positive depth exceeds 65.535 m")
    result = np.zeros(values.shape, dtype=np.uint16)
    result[known] = np.rint(values[known] * 1000).astype(np.uint16)
    return result


def _optical_header(msg: Any) -> int:
    if msg.header.frame_id != OPTICAL_FRAME:
        raise ValueError(f"image/CameraInfo frame must be {OPTICAL_FRAME}")
    return stamp_to_ns(msg.header.stamp)


def _dimensions(msg: Any) -> tuple[int, int]:
    height = _integer(msg.height, "height")
    width = _integer(msg.width, "width")
    if not height or not width:
        raise ValueError("image dimensions must be positive")
    return height, width


def image_to_array(msg_like: Any, encoding: str) -> NDArray:
    """Decode packed rgb8 uint8[H,W,3] or little-endian 32FC1 float32[H,W].

    Only the registered optical grid is accepted; stride, byte count, encoding
    and endianness must exactly match. Returns an owned copy of the pixels.
    """
    if encoding not in ("rgb8", "32FC1") or msg_like.encoding != encoding:
        raise ValueError("image encoding mismatch; expected rgb8 or 32FC1")
    _optical_header(msg_like)
    height, width = _dimensions(msg_like)
    if msg_like.is_bigendian != 0:
        raise ValueError("big-endian images are not supported")
    pixel_bytes = 3 if encoding == "rgb8" else 4
    if msg_like.step != width * pixel_bytes:
        raise ValueError("image step must be packed width times pixel bytes")
    data = bytes(msg_like.data)
    if len(data) != height * width * pixel_bytes:
        raise ValueError("image data length does not match dimensions and step")
    shape = (height, width, 3) if encoding == "rgb8" else (height, width)
    return (
        np.frombuffer(data, dtype=np.uint8 if encoding == "rgb8" else "<f4")
        .reshape(shape)
        .copy()
    )


def camera_info_to_json(msg_like: Any) -> dict:
    """Preserve received CameraInfo fields and row-major matrices as strict JSON."""
    stamp_ns = _optical_header(msg_like)
    height, width = _dimensions(msg_like)
    result = {
        "frame_id": msg_like.header.frame_id,
        "stamp_ns": stamp_ns,
        "width": width,
        "height": height,
        "distortion_model": msg_like.distortion_model,
        **{
            name: [float(v) for v in getattr(msg_like, name)]
            for name in ("d", "k", "r", "p")
        },
        "binning_x": int(msg_like.binning_x),
        "binning_y": int(msg_like.binning_y),
        "roi": {
            **{
                name: int(getattr(msg_like.roi, name))
                for name in ("x_offset", "y_offset", "height", "width")
            },
            "do_rectify": bool(msg_like.roi.do_rectify),
        },
    }
    json.dumps(result, allow_nan=False)
    return result


def tf_static_to_json(transform_like: Any) -> dict:
    """Serialize the received base_link <- optical transform, independent of stamp."""
    if (
        transform_like.header.frame_id != "base_link"
        or transform_like.child_frame_id != OPTICAL_FRAME
    ):
        raise ValueError("TF frame pair must be base_link <- camera_optical_frame")
    translation = transform_like.transform.translation
    rotation = transform_like.transform.rotation
    result = {
        "target_frame": transform_like.header.frame_id,
        "source_frame": transform_like.child_frame_id,
        "translation_m": [float(getattr(translation, axis)) for axis in "xyz"],
        "quaternion_xyzw": [float(getattr(rotation, axis)) for axis in "xyzw"],
        "origin": "received_tf_static",
    }
    json.dumps(result, allow_nan=False)
    if abs(np.linalg.norm(result["quaternion_xyzw"]) - 1) > 1e-6:
        raise ValueError("TF quaternion must have unit norm")
    return result


def select_synchronized_set(buffers: dict, warmup_ns: int) -> int | None:
    """Select the earliest shared header stamp >= warmup with nonblack RGB.

    Buffers map rgb/depth/info to stamp-keyed arrays/JSON. No approximate-time
    matching is permitted, and all-unknown depth is a valid captured observation.
    """
    shared = buffers["rgb"].keys() & buffers["depth"].keys() & buffers["info"].keys()
    for stamp_ns in sorted(shared):
        if stamp_ns >= warmup_ns and np.any(buffers["rgb"][stamp_ns]):
            return stamp_ns
    return None


def build_scene_json(
    entry: dict,
    stamp_ns: int,
    image_id: str,
    source_sha: str,
    run_id: str,
    wall_times: dict,
) -> dict:
    """Copy scene metadata and attach capture provenance without inventing entry fields."""
    result = deepcopy(
        {
            name: entry[name]
            for name in (
                "scene_id",
                "catalogue_version",
                "category",
                "split",
                "camera",
                "visibility",
            )
        }
    )
    result.update(
        stamp_ns=_integer(stamp_ns, "stamp_ns"),
        clock_domain="ros_sim",
        source_provenance="synthetic",
        image_id=image_id,
        source_snapshot_sha256=source_sha,
        run_id=run_id,
        wall_times_s=deepcopy(wall_times),
    )
    return result


def ground_truth_for_capture(entry: dict, stamp_ns: int) -> dict:
    """Copy complete catalogue truth, replacing only acquisition time and clock."""
    result = deepcopy(entry["ground_truth"])
    result.update(stamp_ns=_integer(stamp_ns, "stamp_ns"), clock_domain="ros_sim")
    return result


def write_json(path: Path, value: dict) -> None:
    """Publish strict JSON by replacement so progress readers never see partial JSON."""
    text = json.dumps(value, indent=2, allow_nan=False) + "\n"
    temporary = path.with_suffix(".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def write_scene_files(
    scene_dir: Path,
    *,
    rgb: NDArray,
    depth_m: NDArray,
    camera_info: dict,
    tf: dict,
    ground_truth: dict,
    scene: dict,
) -> dict[str, str]:
    """Write the seven v1 files and a 0-5 m preview; return their SHA-256 hashes.

    RGB is uint8[H,W,3]; depth is float32/64[H,W] axial metres. Unknown depth is
    black in the preview. All JSON writes are atomic; storage errors propagate.
    """
    from PIL import Image

    raw = depth_to_millimetre_png_array(depth_m)
    shape = (camera_info["height"], camera_info["width"])
    if (
        not isinstance(rgb, np.ndarray)
        or rgb.dtype != np.uint8
        or rgb.shape != (*shape, 3)
        or raw.shape != shape
    ):
        raise ValueError("RGB/depth shape must match CameraInfo dimensions")
    if (
        camera_info["stamp_ns"] != scene["stamp_ns"]
        or ground_truth["stamp_ns"] != scene["stamp_ns"]
    ):
        raise ValueError("CameraInfo, scene and ground truth stamps must match")
    scene_dir = Path(scene_dir)
    scene_dir.mkdir(parents=True, exist_ok=True)
    preview = np.rint(np.minimum(raw.astype(np.float64), 5000) * 255 / 5000).astype(
        np.uint8
    )
    images = {"rgb.png": rgb, "depth_mm.png": raw, "depth_preview.png": preview}
    for name, values in images.items():
        Image.fromarray(values).save(scene_dir / name)
    objects = {
        "depth_meta.json": {
            "unit": "mm",
            "meters_per_unit": 0.001,
            "unknown_value": 0,
            "kind": "optical_axis_z",
        },
        "camera_info.json": camera_info,
        "tf.json": tf,
        "ground_truth.json": ground_truth,
        "scene.json": scene,
    }
    for name, value in objects.items():
        write_json(scene_dir / name, value)
    return {
        name: hashlib.sha256((scene_dir / name).read_bytes()).hexdigest()
        for name in (*images, *objects)
    }


class CaptureState:
    """Bounded received-message state, independent of ROS and wall-clock scheduling."""

    def __init__(self, warmup_ns: int):
        self.warmup_ns = _integer(warmup_ns, "warmup_ns")
        self.buffers = {kind: {} for kind in ("rgb", "depth", "info")}
        self.counts = dict.fromkeys((*self.buffers, "clock", "tf_static"), 0)
        self.tf = None

    def accept(self, kind: str, msg: Any) -> None:
        """Validate a message and retain at most the latest 50 stamps per image stream."""
        if kind == "clock":
            stamp_to_ns(msg.clock)
        elif kind == "tf_static":
            for transform in msg.transforms:
                if (
                    transform.header.frame_id == "base_link"
                    and transform.child_frame_id == OPTICAL_FRAME
                ):
                    self.tf = tf_static_to_json(transform)
        elif kind in self.buffers:
            value = (
                camera_info_to_json(msg)
                if kind == "info"
                else image_to_array(msg, "rgb8" if kind == "rgb" else "32FC1")
            )
            buffer = self.buffers[kind]
            buffer[stamp_to_ns(msg.header.stamp)] = value
            while len(buffer) > 50:
                del buffer[min(buffer)]
        else:
            raise ValueError(f"unknown capture stream: {kind}")
        self.counts[kind] += 1

    def selected_stamp(self) -> int | None:
        """Reevaluate selection on every event, including late static TF or clock."""
        if not self.counts["clock"] or self.tf is None:
            return None
        return select_synchronized_set(self.buffers, self.warmup_ns)
