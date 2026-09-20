"""SDK-free conversion of synthetic camera data into perception input.

The caller owns camera creation, optical-axis configuration and same-time
robot pose acquisition. This module neither steps Isaac nor imports its SDK.
"""

import importlib.util
from collections.abc import Callable
from copy import deepcopy
from dataclasses import dataclass, field
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

    def __init__(self, reason: str, diagnostics=None):
        self.reason = reason
        self.diagnostics = diagnostics
        super().__init__(reason)


@dataclass
class CaptureDiagnostics:
    """Capture metadata, separate from the unchanged raw-depth statistics.

    Times are simulation seconds; poses are world position metres and wxyz.
    On failure these describe the last attempt, never an accepted observation.
    """

    sensor_frame_id: object = None
    acquisition_time_s: float | None = None
    physics_time_before_s: float | None = None
    physics_time_after_s: float | None = None
    pose_before: tuple | None = None
    pose_after: tuple | None = None
    capture_start_time_s: float | None = None
    render_steps: int = 0
    acquisition_since_capture_start_s: float | None = None
    accepted_pose: tuple | None = None
    pose_reference: str = "stationary_capture_end"
    attempts: int = 0
    rejection_counts: dict = field(default_factory=dict)


@dataclass
class CaptureState:
    """Caller-owned, single-camera/session state. Never share across cameras.

    Retain this object across observation waypoints. A camera reset requires a
    new session. Arrays and IDs are owned copies, including mutable SDK IDs.
    """

    previous: tuple | None = None
    previous_frame_id: object = None
    diagnostics: CaptureDiagnostics = field(default_factory=CaptureDiagnostics)


class SensorCapture:
    """Strict Isaac capture boundary, SDK-free for fake-camera verification.

    Camera 5.1 exposes rendering_frame (fabric-time dict) and rendering_time
    (simulation seconds) in get_current_frame(). Read pixels via direct getters.
    Single-threaded stepping is required. Metadata brackets both getters; the
    pose/time bracket spans capture start through the latest render step. Flush
    delayed frames until acquisition is at or after capture start plus the
    configured render latency. Compare each
    post-step pose to the fixed start pose; the accepted end pose represents the
    acquisition under this stationary-capture contract, not exact
    pose interpolation or proof against motion that returns to its start.
    """

    def __init__(
        self,
        camera,
        mount,
        *,
        step_fn,
        physics_time_fn,
        pose_fn,
        position_tolerance_m=0.001,
        angle_tolerance_rad=0.001,
        render_latency_s=0.066667,
    ):
        self.camera, self.mount = camera, mount
        self.step_fn, self.physics_time_fn, self.pose_fn = (
            step_fn,
            physics_time_fn,
            pose_fn,
        )
        self.position_tolerance_m = _finite_scalar(
            position_tolerance_m, "position_tolerance_m"
        )
        self.angle_tolerance_rad = _finite_scalar(
            angle_tolerance_rad, "angle_tolerance_rad"
        )
        if min(self.position_tolerance_m, self.angle_tolerance_rad) < 0:
            raise ValueError("pose tolerances must be nonnegative")
        # A conservative flush margin, not a derived pixel latency. The probe
        # that produced 0.066667 s ran at rendering_dt=1/60 (ws1 Isaac 5.1,
        # 2026-09-20), where the metadata time trailed physics by four render
        # frames for 24 steps after warm-up. run_transport steps the world at
        # rendering_dt=1/120, where the same run's diagnostics show the metadata
        # trailing by about 0.0167 s - so this default overshoots that config
        # and forces extra renders rather than fewer.
        #
        # What the margin buys is unproven in general: measuring how far the
        # metadata time trails physics does NOT establish how far the pixels
        # trail the metadata, and only the latter makes "metadata past
        # capture_start + L" imply "pixels past capture_start". It held on the
        # run it was validated against (seed 0, four waypoints, estimates 0.8
        # and 2.8 mm against ground truth). Recalibrate per render cadence, and
        # treat a passing capture as evidence only for the configuration it ran
        # in.
        self.render_latency_s = _finite_scalar(render_latency_s, "render_latency_s")
        if self.render_latency_s < 0:
            raise ValueError("render_latency_s must be nonnegative")
        self.state = CaptureState()

    def _metadata(self):
        try:
            frame = self.camera.get_current_frame()
            identifier = deepcopy(frame["rendering_frame"])
            stamp = float(frame["rendering_time"])
            if isinstance(identifier, dict):
                numerator = identifier["referenceTimeNumerator"]
                denominator = identifier["referenceTimeDenominator"]
                if (
                    not np.isfinite(numerator)
                    or not np.isfinite(denominator)
                    or denominator <= 0
                ):
                    raise ValueError("invalid fabric time")
                identifier = {
                    "referenceTimeNumerator": int(numerator),
                    "referenceTimeDenominator": int(denominator),
                }
            elif isinstance(identifier, Integral) and not isinstance(identifier, bool):
                identifier = int(identifier)
            else:
                raise ValueError("unsupported sensor identifier")
            if not np.isfinite(stamp):
                raise ValueError("invalid acquisition time")
            return identifier, stamp
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            raise CaptureFailure("sensor_metadata_unavailable") from exc

    def _pose(self):
        position, quaternion = self.pose_fn()
        position, quaternion = np.asarray(position), np.asarray(quaternion)
        if (
            position.shape != (3,)
            or quaternion.shape != (4,)
            or not np.isfinite(position).all()
            or not np.isfinite(quaternion).all()
            or abs(np.linalg.norm(quaternion) - 1) > 1e-5
        ):
            raise CaptureFailure("invalid_capture_pose")
        return tuple(map(float, position)), tuple(map(float, quaternion))

    def capture(self, *, max_attempts=200):
        metadata = None

        def step():
            nonlocal metadata
            diag = self.state.diagnostics
            if diag.capture_start_time_s is None:
                diag.pose_before = self._pose()
                diag.capture_start_time_s = float(self.physics_time_fn())
            diag.physics_time_before_s = float(self.physics_time_fn())
            self.step_fn()
            diag.render_steps += 1
            diag.pose_after = self._pose()
            diag.physics_time_after_s = float(self.physics_time_fn())
            p0, q0 = map(np.asarray, diag.pose_before)
            p1, q1 = map(np.asarray, diag.pose_after)
            angle = 2 * np.arccos(np.clip(abs(np.dot(q0, q1)), 0, 1))
            if (
                np.linalg.norm(p1 - p0) > self.position_tolerance_m
                or angle > self.angle_tolerance_rad
            ):
                raise CaptureFailure("pose_changed")
            metadata = self._metadata()

        def frame_id():
            diag = self.state.diagnostics
            after = self._metadata()
            diag.sensor_frame_id, diag.acquisition_time_s = after
            diag.acquisition_since_capture_start_s = (
                after[1] - diag.capture_start_time_s
            )
            if metadata != after:
                raise CaptureFailure("sensor_frame_changed_during_read")
            identifier = after[0]
            zero_id = (
                identifier["referenceTimeNumerator"] == 0
                if isinstance(identifier, dict)
                else identifier == 0
            )
            if zero_id or after[1] == 0:
                raise CaptureFailure("sensor_frame_not_ready")
            start_s, after_s = diag.capture_start_time_s, diag.physics_time_after_s
            # Pixels may lag the reported acquisition reference by this margin.
            # Keep the upper bound: even after retries, a timestamp ahead of
            # the current physics clock indicates inconsistent sensor metadata.
            if not (
                np.isfinite(start_s)
                and np.isfinite(after_s)
                and start_s + self.render_latency_s <= after[1] <= after_s
            ):
                raise CaptureFailure("acquisition_outside_capture")
            return after[0]

        try:
            result = capture_scene_input(
                self.camera,
                self.mount,
                max_attempts=max_attempts,
                state=self.state,
                step_fn=step,
                frame_id_fn=frame_id,
                stamp_ns_fn=lambda: int(metadata[1] * 1e9),
            )
            self.state.diagnostics.accepted_pose = self.state.diagnostics.pose_after
            return result
        except CaptureFailure as exc:
            if exc.diagnostics is None:
                diag = self.state.diagnostics
                diag.rejection_counts[exc.reason] = (
                    diag.rejection_counts.get(exc.reason, 0) + 1
                )
                exc.diagnostics = deepcopy(diag)
            raise


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
    state: CaptureState | None = None,
) -> tuple[SceneInput, FrameDiagnostics, int]:
    """Read a ready synthetic frame, calling step_fn before every attempt.

    Retain state across calls to compare the first attempt against the last
    accepted frame. Without state this is a one-shot conversion only. The first
    ever frame has no predecessor; all later attempts compare IDs, or both raw
    arrays when IDs are unavailable. Array fallback deliberately favors false
    rejection of identical static renders over reuse, also across calls.
    The generic callbacks must describe the same acquisition. Use SensorCapture
    for the Isaac capture-start freshness/pose contract and required metadata.
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
    if state is None:
        state = CaptureState()
    state.diagnostics = CaptureDiagnostics()
    previous = state.previous
    previous_frame_id = state.previous_frame_id
    reason = "timeout"
    for attempt in range(max_attempts):
        state.diagnostics.attempts = attempt + 1
        try:
            if step_fn is not None:
                step_fn()
            # Own both buffers: Isaac/fakes may mutate the same arrays on the next step.
            rgba = np.array(camera.get_rgba(), copy=True)
            raw_depth = np.array(camera.get_depth(), copy=True)
            if raw_depth.ndim and raw_depth.shape[-1] == 1:
                raw_depth = np.squeeze(raw_depth, axis=-1)
            frame_id = frame_id_fn() if frame_id_fn is not None else None
        except CaptureFailure as exc:
            # Only transient sensor readiness/timing failures may consume retries.
            # Metadata corruption, torn reads and motion still fail immediately.
            if exc.reason not in {
                "sensor_frame_not_ready",
                "acquisition_outside_capture",
            }:
                raise
            reason = exc.reason
            counts = state.diagnostics.rejection_counts
            counts[reason] = counts.get(reason, 0) + 1
            continue
        fresh = True
        if previous is not None:
            if frame_id is not None:
                fresh = frame_id != previous_frame_id and (
                    state.previous is None or frame_id != state.previous_frame_id
                )
            else:
                # Last resort for cameras without IDs; identical static renders
                # can be falsely rejected even though they are fresh frames.
                fresh = all(
                    not (
                        np.array_equal(rgba, old[0], equal_nan=True)
                        or np.array_equal(raw_depth, old[1], equal_nan=True)
                    )
                    for old in (previous, state.previous)
                    if old is not None
                )
        rgba_ready = (
            rgba.shape == (*shape, 4)
            and rgba.dtype == np.uint8
            and float(np.std(rgba[:, :, :3])) > threshold
        )
        depth_ready = raw_depth.shape == shape and raw_depth.dtype.kind in "uif"
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
            state.previous = rgba, raw_depth
            state.previous_frame_id = deepcopy(frame_id)
            state.diagnostics.sensor_frame_id = deepcopy(frame_id)
            return scene, diagnostics, attempt + 1
        counts = state.diagnostics.rejection_counts
        counts[reason] = counts.get(reason, 0) + 1
        previous = rgba, raw_depth
        previous_frame_id = deepcopy(frame_id)
    raise CaptureFailure(reason, deepcopy(state.diagnostics))


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
