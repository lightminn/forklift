"""The RGB-D rig in Isaac: cameras on base_link, frozen captures, freshness checks.

Plan: docs/plans/2026-10-07-visual-slam-and-fusion.md, D1. The geometry comes
from forklift_core.sensors.camera_rig (config/isaac_depth_rig.yaml). A capture
renders ``frozen_renders`` times with physics stopped, so the pixels settle on
the physics state of that step even though the render pipeline trails physics
(docs/validation/2026-09-20-capture-freshness.md); physics time and the truck
pose must not change while it does. A freshness check casts PhysX rays through
a pixel grid from the current pose and from earlier poses: the rendered depth
must agree best with the current one. Only create_cameras, capture and
cast_depth touch Isaac; the rest runs (and is tested) on CPU.
"""

from __future__ import annotations

import hashlib
import json
import queue
import threading
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from forklift_core.sensors.camera_rig import (
    CameraRig,
    RigCamera,
    pixel_grid,
    rig_from_config,
)

# Physics steps (1/120 s) between the pose a frame is checked against and now.
FRESHNESS_LAGS_STEPS = (0, 1, 3, 12)
DEFAULT_CLIPPING: dict[str, list[float]] = {}
FRESHNESS_GRID = (16, 12)
# A rendered frame may differ from the collision geometry by this much (median).
LAG0_LIMIT_M = 0.005
# Pixels whose cast depth moved more than this between two poses tell them apart.
MOVED_M = 0.005
MIN_MOVED = 5


def load_rig(path: Path) -> tuple[CameraRig, dict, str]:
    import yaml

    text = Path(path).read_text()
    config = yaml.safe_load(text)
    return rig_from_config(config), config, hashlib.sha256(text.encode()).hexdigest()


def _rotation_wxyz(quaternion_wxyz) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion_wxyz, dtype=float) / np.linalg.norm(
        quaternion_wxyz
    )
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def quaternion_wxyz(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Unit quaternion (w, x, y, z), w >= 0, of a rotation matrix."""
    m = np.asarray(rotation, dtype=float)
    trace = float(np.trace(m))
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = (
            s / 4,
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
        )
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = (
            (m[2, 1] - m[1, 2]) / s,
            s / 4,
            (m[0, 1] + m[1, 0]) / s,
            (m[0, 2] + m[2, 0]) / s,
        )
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = (
            (m[0, 2] - m[2, 0]) / s,
            (m[0, 1] + m[1, 0]) / s,
            s / 4,
            (m[1, 2] + m[2, 1]) / s,
        )
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = (
            (m[1, 0] - m[0, 1]) / s,
            (m[0, 2] + m[2, 0]) / s,
            (m[1, 2] + m[2, 1]) / s,
            s / 4,
        )
    q = np.asarray(q) / np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return tuple(float(v) for v in q)


def world_rays(
    camera: RigCamera, base_position, base_quaternion_wxyz, pixels_uv: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """World origin (3,) and unit directions (N, 3) of pixel rays at a base pose."""
    world_from_base = _rotation_wxyz(base_quaternion_wxyz)
    origin = (
        np.asarray(base_position, float) + world_from_base @ camera.translation_base_m
    )
    return origin, camera.ray_directions_base(pixels_uv) @ world_from_base.T


def axial_from_range(
    camera: RigCamera, pixels_uv: np.ndarray, ranges_m: np.ndarray
) -> np.ndarray:
    """z-depth of each pixel ray's hit from its range along the unit ray."""
    k = camera.intrinsics
    uv = np.asarray(pixels_uv, float)
    norm = np.sqrt(
        ((uv[:, 0] - k.cx) / k.fx) ** 2 + ((uv[:, 1] - k.cy) / k.fy) ** 2 + 1.0
    )
    return np.asarray(ranges_m, float) / norm


def sample_depth(depth_m: np.ndarray, pixels_uv: np.ndarray) -> np.ndarray:
    """Rendered depth at integer-index pixel centres (the grid is integer-rounded)."""
    uv = np.rint(np.asarray(pixels_uv, float)).astype(int)
    return np.asarray(depth_m, float)[uv[:, 1], uv[:, 0]]


def freshness_residuals(
    rendered_m: np.ndarray, cast_by_lag_m: dict[int, np.ndarray], far_m: float
) -> dict:
    """Does the rendered depth belong to this physics step, not an earlier one?

    ``cast_by_lag_m[0]`` is ray-cast depth from the current pose, other keys
    from poses that many physics steps earlier. Lag 0 must agree within
    ``LAG0_LIMIT_M`` (median over the pixels both see), and, for every lag,
    over the pixels whose cast depth differs from lag 0 by more than
    ``MOVED_M`` (where an older frame would show), the rendered depth must sit
    closer to lag 0 than to that lag. A lag with fewer than ``MIN_MOVED``
    such pixels cannot tell the poses apart (standing still, or a floor
    seen while driving parallel to it) and is not judged. A floor is a plane,
    so a camera translating over it sees the same floor depth: that is why the
    plain median cannot separate the lags.
    """
    current = cast_by_lag_m.get(0)
    out = {"pixels": int(len(rendered_m)), "lags_steps": {}}
    if current is None:
        out.update({"fresh": False, "judged": False})
        return out
    seen = (
        np.isfinite(rendered_m)
        & np.isfinite(current)
        & (current < far_m)
        & (rendered_m < far_m)
    )
    lag0 = (
        float(np.median(np.abs(rendered_m[seen] - current[seen])))
        if seen.any()
        else None
    )
    out["lag0_common"] = int(seen.sum())
    out["lag0_median_abs_m"] = lag0
    fresh = lag0 is not None and lag0 <= LAG0_LIMIT_M
    judged = False
    for lag, cast in cast_by_lag_m.items():
        if lag == 0:
            continue
        both = seen & np.isfinite(cast) & (cast < far_m)
        moved = both & (np.abs(cast - current) > MOVED_M)
        n = int(moved.sum())
        record = {"moved": n}
        if n >= MIN_MOVED:
            to_now = float(np.median(np.abs(rendered_m[moved] - current[moved])))
            to_lag = float(np.median(np.abs(rendered_m[moved] - cast[moved])))
            record.update({"median_to_now_m": to_now, "median_to_lag_m": to_lag})
            judged = True
            fresh = fresh and to_now < to_lag
        out["lags_steps"][str(lag)] = record
    out.update({"fresh": bool(fresh), "judged": judged})
    return out


class PoseHistory:
    """The truck pose at the last ``depth`` physics steps, newest last."""

    def __init__(self, depth: int = max(FRESHNESS_LAGS_STEPS) + 1):
        self._poses = deque(maxlen=depth)

    def push(self, position, quaternion_wxyz) -> None:
        self._poses.append(
            (np.array(position, float), np.array(quaternion_wxyz, float))
        )

    def lagged(self, steps: int):
        if steps >= len(self._poses):
            return None
        return self._poses[-1 - steps]


class FrameWriter:
    """Writes rig frames on a background thread: RGB JPEG (q95), depth uint16 PNG.

    Depth is stored noise-free in millimetres (0 = no data), the record's clean
    measurement; noise is added at replay or send time. close() waits for the
    queue and re-raises the first write error.
    """

    def __init__(self, root: Path, names, *, workers: int = 3, quality: int = 95):
        self.root = Path(root)
        for name in names:
            (self.root / name).mkdir(parents=True, exist_ok=False)
        self.quality = quality
        self._queue: queue.Queue = queue.Queue(maxsize=64)
        self._error: BaseException | None = None
        self._threads = [
            threading.Thread(target=self._work, daemon=True) for _ in range(workers)
        ]
        for thread in self._threads:
            thread.start()

    def _work(self) -> None:
        from PIL import Image

        while True:
            item = self._queue.get()
            if item is None:
                self._queue.task_done()
                return
            name, index, rgb, depth_mm = item
            try:
                base = self.root / name / f"{index:06d}"
                Image.fromarray(rgb).save(
                    base.with_suffix(".jpg"), quality=self.quality
                )
                Image.fromarray(depth_mm).save(base.with_suffix(".png"))
            except BaseException as exc:  # surfaced by close()
                self._error = self._error or exc
            finally:
                self._queue.task_done()

    def put(self, name: str, index: int, rgb: np.ndarray, depth_mm: np.ndarray) -> None:
        if self._error is not None:
            raise RuntimeError(f"rig frame write failed: {self._error}")
        self._queue.put(
            (name, index, np.ascontiguousarray(rgb), np.ascontiguousarray(depth_mm))
        )

    def close(self) -> None:
        for _ in self._threads:
            self._queue.put(None)
        for thread in self._threads:
            thread.join(timeout=600)
        if self._error is not None:
            raise RuntimeError(f"rig frame write failed: {self._error}")


def clean_depth_mm(depth_m: np.ndarray, near_m: float, far_m: float) -> np.ndarray:
    """Noise-free uint16 mm; outside [near, far] or non-finite is 0."""
    depth = np.asarray(depth_m, dtype=float)
    valid = np.isfinite(depth) & (depth >= near_m) & (depth <= far_m)
    return np.where(valid, np.round(np.where(valid, depth, 0.0) * 1000.0), 0).astype(
        np.uint16
    )


def rig_record(rig: CameraRig, config: dict, sha256: str, sdk_intrinsics: dict) -> dict:
    """What meta.json keeps about the rig: config, hash, K and mounts per camera."""
    return {
        "config": config,
        "config_sha256": sha256,
        "source": "synthetic Isaac Sim cameras, not a measured D435i",
        "depth_kind": "optical_axis_z registered to the colour grid",
        "stored": "rgb JPEG q95, depth uint16 mm PNG (0 = no data), noise-free",
        "cameras": {
            camera.name: {
                "frame_id": camera.frame_id,
                "K_integer_index": [
                    [camera.intrinsics.fx, 0.0, camera.intrinsics.cx],
                    [0.0, camera.intrinsics.fy, camera.intrinsics.cy],
                    [0.0, 0.0, 1.0],
                ],
                "rotation_base_from_optical": np.asarray(
                    camera.rotation_base_from_optical
                ).tolist(),
                "translation_base_m": np.asarray(camera.translation_base_m).tolist(),
                "sdk_intrinsics": sdk_intrinsics.get(camera.name),
            }
            for camera in rig.cameras
        },
    }


# --- Isaac -------------------------------------------------------------------


def create_cameras(
    Camera, rig: CameraRig, parent: str = "/World/Forklift/base_link"
) -> dict:
    """One Isaac Camera per rig camera, child of base_link, ROS optical axes."""
    cameras = {}
    for spec in rig.cameras:
        k = spec.intrinsics
        camera = Camera(
            prim_path=f"{parent}/Rig_{spec.name}",
            frequency=-1,
            resolution=(k.width, k.height),
        )
        camera.set_local_pose(
            translation=np.asarray(spec.translation_base_m, float),
            orientation=np.asarray(quaternion_wxyz(spec.rotation_base_from_optical)),
            camera_axes="ros",
        )
        camera.set_projection_mode("perspective")
        camera.set_lens_distortion_model("pinhole")
        camera.set_focal_length(1.0)
        camera.set_horizontal_aperture(k.width / k.fx, maintain_square_pixels=True)
        # The USD default, kept as evidence of what the earlier records used.
        DEFAULT_CLIPPING[spec.name] = [float(v) for v in camera.get_clipping_range()]
        camera.set_clipping_range(*rig.clipping_range_m)
        cameras[spec.name] = camera
    return cameras


def verify_intrinsics(read_isaac_intrinsics, cameras: dict, rig: CameraRig) -> dict:
    """Read each SDK K back; its integer-index form must equal the rig's K."""
    records = {}
    for spec in rig.cameras:
        calibration = read_isaac_intrinsics(cameras[spec.name])
        got, want = calibration.integer_index, spec.intrinsics
        if (got.width, got.height) != (want.width, want.height):
            raise RuntimeError(f"{spec.name}: resolution {got.width}x{got.height}")
        if abs(got.fx / want.fx - 1) > 1e-6 or abs(got.fy / want.fy - 1) > 1e-6:
            raise RuntimeError(f"{spec.name}: focal {got.fx}, {got.fy} vs {want.fx}")
        if abs(got.cx - want.cx) > 1e-6 or abs(got.cy - want.cy) > 1e-6:
            raise RuntimeError(f"{spec.name}: principal point {got.cx}, {got.cy}")
        clipping = [float(v) for v in cameras[spec.name].get_clipping_range()]
        if not np.allclose(clipping, rig.clipping_range_m, rtol=1e-6, atol=0):
            raise RuntimeError(f"{spec.name}: clipping range reads back {clipping}")
        records[spec.name] = {
            **calibration.to_record(),
            "clipping_range_m": clipping,
            "default_clipping_range_m": DEFAULT_CLIPPING.get(spec.name),
        }
    return records


def capture(world, robot, cameras: dict, rig: CameraRig) -> dict:
    """Render ``frozen_renders`` times without physics, then read every camera.

    Returns name -> (rgb (H, W, 3) uint8, depth (H, W) float32 m, inf = none).
    Raises when physics time or the truck pose moves during the capture.
    """
    stamp = world.current_time
    base, q = (np.array(v, copy=True) for v in robot.get_world_pose())

    def frozen() -> None:
        if world.current_time != stamp:
            raise RuntimeError("Physics advanced during a rig capture")
        after, after_q = robot.get_world_pose()
        if not (
            np.allclose(after, base, atol=1e-9, rtol=0)
            and np.allclose(after_q, q, atol=1e-9, rtol=0)
        ):
            raise RuntimeError("Truck pose changed during a rig capture")

    for _ in range(rig.frozen_renders):
        world.render()
        frozen()
    frames = {}
    for spec in rig.cameras:
        k = spec.intrinsics
        rgba = cameras[spec.name].get_rgba()
        depth = cameras[spec.name].get_depth()
        if (
            rgba is None
            or depth is None
            or np.shape(rgba)[:2] != (k.height, k.width)
            or np.shape(depth)[:2] != (k.height, k.width)
        ):
            raise RuntimeError(f"rig camera {spec.name} not ready")
        frames[spec.name] = (
            np.ascontiguousarray(np.asarray(rgba)[:, :, :3], dtype=np.uint8),
            np.asarray(depth, dtype=np.float32).reshape(k.height, k.width).copy(),
        )
    frozen()
    return frames


def freshness_check(
    cast_scan, rig: CameraRig, frames: dict, history: PoseHistory
) -> dict:
    """Ray-cast depth at the grid from lagged poses against the rendered depth."""
    result = {}
    far = rig.depth_range_m[1]
    for spec in rig.cameras:
        # Whole pixels, so the ray and the sampled depth are the same pixel.
        uv = np.rint(pixel_grid(spec.intrinsics, *FRESHNESS_GRID))
        rendered = sample_depth(frames[spec.name][1], uv)
        cast_by_lag, self_hits = {}, 0
        for lag in FRESHNESS_LAGS_STEPS:
            pose = history.lagged(lag)
            if pose is None:
                continue
            origin, directions = world_rays(spec, pose[0], pose[1], uv)
            distances, hits, own = cast_scan(origin, directions, far * 1.5)
            if lag == 0:
                self_hits = int(own)
            cast_by_lag[lag] = np.where(
                hits, axial_from_range(spec, uv, distances), np.inf
            )
        record = freshness_residuals(rendered, cast_by_lag, far)
        record["self_hits"] = self_hits
        result[spec.name] = record
    return result


def summarise_freshness(checks: list[dict]) -> dict:
    """Counts over every check: judged (some lag separable), fresh, failures."""
    judged = fresh_judged = lag0_ok = total = self_hits = 0
    medians, failures = [], []
    for check in checks:
        for name, record in check["cameras"].items():
            total += 1
            self_hits += record["self_hits"]
            if record.get("lag0_median_abs_m") is not None:
                medians.append(record["lag0_median_abs_m"])
                lag0_ok += record["lag0_median_abs_m"] <= LAG0_LIMIT_M
            if record["judged"]:
                judged += 1
                fresh_judged += bool(record["fresh"])
            if not record["fresh"] and (
                record["judged"]
                or record.get("lag0_median_abs_m") is None
                or record["lag0_median_abs_m"] > LAG0_LIMIT_M
            ):
                failures.append({"index": check["index"], "camera": name})
    return {
        "camera_checks": total,
        "camera_checks_lag0_within_limit": lag0_ok,
        "camera_checks_judged": judged,
        "camera_checks_judged_fresh": fresh_judged,
        "failures": failures,
        "self_hits": self_hits,
        "lag0_median_abs_m_median": float(np.median(medians)) if medians else None,
        "lag0_median_abs_m_max": float(np.max(medians)) if medians else None,
    }


@dataclass
class ChaseView:
    """Third-person camera behind and above the truck, for the video only."""

    back_m: float = 3.6
    up_m: float = 2.4
    look_ahead_m: float = 1.2
    look_up_m: float = 0.4

    def eye_and_target(
        self, base_position, yaw_rad: float
    ) -> tuple[np.ndarray, np.ndarray]:
        forward = np.array([np.cos(yaw_rad), np.sin(yaw_rad), 0.0])
        base = np.asarray(base_position, float)
        eye = base - self.back_m * forward + np.array([0.0, 0.0, self.up_m])
        target = (
            base + self.look_ahead_m * forward + np.array([0.0, 0.0, self.look_up_m])
        )
        return eye, target


def write_frames_index(path: Path, frames: list[dict], extra: dict) -> None:
    path.write_text(json.dumps({**extra, "frames": frames}, indent=1) + "\n")


__all__ = [
    "ChaseView",
    "FRESHNESS_LAGS_STEPS",
    "FrameWriter",
    "PoseHistory",
    "axial_from_range",
    "capture",
    "clean_depth_mm",
    "create_cameras",
    "freshness_check",
    "freshness_residuals",
    "load_rig",
    "quaternion_wxyz",
    "rig_record",
    "sample_depth",
    "summarise_freshness",
    "verify_intrinsics",
    "world_rays",
    "write_frames_index",
]
