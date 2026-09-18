"""SDK-free conversion of synthetic camera data into perception input.

The caller owns camera creation, optical-axis configuration and same-time
robot pose acquisition. This module neither steps Isaac nor imports its SDK.
"""

import importlib.util
from collections.abc import Callable
from dataclasses import dataclass
from math import atan2, cos, pi, sin
from numbers import Integral
from pathlib import Path

import numpy as np

from forklift_core._validation import _finite_scalar, _real_array
from forklift_core.geometry import RigidTransform
from forklift_core.perception.pocket_observation import PocketObservation
from forklift_core.perception.scene_dataset import SceneInput
from forklift_core.planning.pallet_mission import PalletSite

# tools is outside the installed src package. Load the repository's canonical
# rig by path, like the existing Isaac geometry unit tests, without sys.path edits.
_RIG_SPEC = importlib.util.spec_from_file_location(
    "_isaac_perception_scene_rig",
    Path(__file__).resolve().parents[2] / "tools/scene_rig.py",
)
_RIG = importlib.util.module_from_spec(_RIG_SPEC)
_RIG_SPEC.loader.exec_module(_RIG)


def estimate_world_pallet_site(
    base_xy_m: tuple[float, float],
    insertion_yaw_rad: float,
    base_position_m: tuple[float, float],
    base_yaw_rad: float,
) -> PalletSite:
    """Transform the estimated base_link centre using the capture-time world pose."""
    x, y = base_xy_m
    c, s = cos(base_yaw_rad), sin(base_yaw_rad)
    yaw = base_yaw_rad + insertion_yaw_rad
    yaw = atan2(sin(yaw), cos(yaw))
    if yaw == -pi:
        yaw = pi
    return PalletSite(
        base_position_m[0] + c * x - s * y,
        base_position_m[1] + s * x + c * y,
        yaw,
    )


def xyzw_to_wxyz(
    q: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Reorder quaternion components; do not change the rotation or camera axes."""
    x, y, z, w = q
    return w, x, y, z


def default_base_from_optical() -> RigidTransform:
    """Return the canonical synthetic baseline_0p50 mount from scene_rig."""
    return _RIG.Camera().base_from_optical()


@dataclass(frozen=True)
class FrameDiagnostics:
    """Disjoint raw-depth counts, with positive finite extrema in metres."""

    nan_count: int
    posinf_count: int
    neginf_count: int
    zero_count: int
    negative_count: int
    finite_positive_count: int
    finite_positive_min: float | None
    finite_positive_max: float | None


def normalize_depth(raw_depth: np.ndarray) -> tuple[np.ndarray, FrameDiagnostics]:
    """Copy axial metre depths to float64; record sentinels before normalization."""
    depth = _real_array(raw_depth, "raw_depth")
    if not depth.size:
        raise ValueError("raw_depth must not be empty")
    finite = np.isfinite(depth)
    positive = finite & (depth > 0)
    values = depth[positive]
    diagnostics = FrameDiagnostics(
        nan_count=int(np.count_nonzero(np.isnan(depth))),
        posinf_count=int(np.count_nonzero(np.isposinf(depth))),
        neginf_count=int(np.count_nonzero(np.isneginf(depth))),
        zero_count=int(np.count_nonzero(depth == 0)),
        negative_count=int(np.count_nonzero(finite & (depth < 0))),
        finite_positive_count=int(values.size),
        finite_positive_min=float(values.min()) if values.size else None,
        finite_positive_max=float(values.max()) if values.size else None,
    )
    depth[~finite | (depth <= 0)] = np.nan
    return depth, diagnostics


class CaptureFailure(Exception):
    """Bounded capture failure; reason describes the last rejected attempt."""

    def __init__(self, reason: str):
        self.reason = reason
        super().__init__(reason)


def capture_scene_input(
    camera,
    base_from_optical: RigidTransform,
    stamp_ns: int | None = None,
    *,
    max_attempts: int = 200,
    std_threshold: float = 2.0,
    step_fn: Callable[[], None] | None = None,
    frame_id_fn: Callable[[], object] | None = None,
    stamp_ns_fn: Callable[[], int] | None = None,
) -> tuple[SceneInput, FrameDiagnostics, int]:
    """Read a ready synthetic frame, calling step_fn before every attempt.

    frame_id_fn is sampled each attempt and compared with the previous ID using
    !=; the first attempt has no freshness comparison. Without it, both raw
    buffers must change, which cannot distinguish identical static renders.
    IDs must support scalar inequality and remain stable after being returned.
    stamp_ns_fn is called on the accepted attempt and takes precedence over
    stamp_ns. A fixed stamp_ns remains sufficient for stationary observation
    waypoint captures where the observation time does not change.
    The caller must synchronize the stamp and robot pose with the accepted
    capture, and configure registered, undistorted RGB and axial-z metre depth.
    Return the scene, raw-depth diagnostics and 1-based accepted attempt count.
    An all-unobserved depth frame is returned with finite_positive_count == 0;
    the caller decides how to handle the resulting perception failure.
    """
    if (
        isinstance(max_attempts, (bool, np.bool_))
        or not isinstance(max_attempts, Integral)
        or max_attempts <= 0
    ):
        raise ValueError("max_attempts must be a positive integer")
    if stamp_ns_fn is None and (
        isinstance(stamp_ns, (bool, np.bool_))
        or not isinstance(stamp_ns, Integral)
        or stamp_ns < 0
    ):
        raise ValueError("stamp_ns must be a nonnegative integer")
    threshold = _finite_scalar(std_threshold, "std_threshold")
    if threshold < 0:
        raise ValueError("std_threshold must be nonnegative")
    intrinsics = _RIG.intrinsics()
    if (
        base_from_optical.source_frame != intrinsics.frame_id
        or base_from_optical.target_frame != "base_link"
    ):
        raise ValueError("Expected camera_optical_frame to base_link transform")
    # Forced scene_rig calibration, without overrides. Checking actual Isaac K,
    # distortion and camera_axes belongs to the later simulator validation stage.
    shape = (intrinsics.height, intrinsics.width)
    previous = None
    previous_frame_id = None
    reason = "timeout"
    for attempt in range(max_attempts):
        if step_fn is not None:
            step_fn()
        # Own both buffers: Isaac/fakes may mutate the same arrays on the next step.
        rgba = np.array(camera.get_rgba(), copy=True)
        raw_depth = np.array(
            camera.get_current_frame().get("distance_to_image_plane"), copy=True
        )
        # TODO: run_transport.py must verify the actual Isaac frame number/time
        # field in camera.get_current_frame() and supply frame_id_fn from it.
        frame_id = frame_id_fn() if frame_id_fn is not None else None
        fresh = True
        if attempt:
            if frame_id_fn is not None:
                fresh = frame_id != previous_frame_id
            else:
                # Last resort for cameras without IDs; identical static renders
                # can be falsely rejected even though they are fresh frames.
                fresh = not (
                    np.array_equal(rgba, previous[0], equal_nan=True)
                    or np.array_equal(raw_depth, previous[1], equal_nan=True)
                )
        rgba_ready = (
            rgba.shape == (*shape, 4)
            and rgba.dtype == np.uint8
            and float(np.std(rgba[:, :, :3])) > threshold
        )
        depth_ready = (
            raw_depth.shape == shape
            and raw_depth.dtype.kind in "uif"
        )
        if not rgba_ready:
            reason = "rgba_not_ready"
        elif not depth_ready:
            reason = "depth_not_ready"
        elif not fresh:
            reason = "stale_frame"
        else:
            accepted_stamp = stamp_ns_fn() if stamp_ns_fn is not None else stamp_ns
            if (
                isinstance(accepted_stamp, (bool, np.bool_))
                or not isinstance(accepted_stamp, Integral)
                or accepted_stamp < 0
            ):
                raise ValueError("stamp_ns must be a nonnegative integer")
            depth, diagnostics = normalize_depth(raw_depth)
            scene = SceneInput(
                rgb=rgba[:, :, :3].copy(),
                depth_m=depth,
                intrinsics=intrinsics,
                base_from_optical=base_from_optical,
                stamp_ns=int(accepted_stamp),
                clock_domain="synthetic",
                source_provenance="synthetic",
            )
            return scene, diagnostics, attempt + 1
        previous = rgba, raw_depth
        previous_frame_id = frame_id
    raise CaptureFailure(reason)


def estimate_pallet_center_m(
    observation: PocketObservation, pallet_depth_m: float
) -> tuple[float, float]:
    """Shift a valid base-frame pocket front midpoint half a depth inward."""
    if observation.status != "valid":
        raise ValueError("A valid pocket observation is required")
    depth = _finite_scalar(pallet_depth_m, "pallet_depth_m")
    if depth <= 0:
        raise ValueError("pallet_depth_m must be positive")
    left, right = observation.left.center_m, observation.right.center_m
    yaw = observation.insertion_yaw_rad
    return (
        (left[0] + right[0]) / 2 + depth / 2 * cos(yaw),
        (left[1] + right[1]) / 2 + depth / 2 * sin(yaw),
    )
