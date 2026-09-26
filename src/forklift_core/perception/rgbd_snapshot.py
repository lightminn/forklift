"""Copy an explicitly calibrated synthetic RGB-D capture into detector input.

No camera SDK, renderer, detector, registration, or unit conversion runs here.
The caller supplies one acquisition's registered RGB and axial metric depth.
"""

from numbers import Integral

import numpy as np
from numpy.typing import NDArray

from forklift_core._validation import _frame_id, _real_array
from forklift_core.geometry import RigidTransform
from forklift_core.perception.pocket_observation import CLOCK_DOMAINS, OBSERVATION_FRAME
from forklift_core.perception.scene_dataset import SceneInput
from forklift_core.sensors.rgbd import PinholeIntrinsics


def scene_input_from_rgbd_snapshot(
    *,
    rgb: NDArray[np.uint8],
    depth_m: NDArray[np.floating],
    intrinsics: PinholeIntrinsics,
    base_from_optical: RigidTransform,
    pixel_frame: str,
    stamp_ns: int,
    clock_domain: str,
    source_provenance: str,
    rectified: bool,
    rgb_registered_to_depth_grid: bool,
    depth_kind: str,
    depth_unit: str,
) -> SceneInput:
    """Return independent RGB/depth buffers for one synthetic acquisition.

    ``rgb`` must be uint8 (H,W,3) RGB; ``depth_m`` must be floating (H,W)
    optical-axis Z in metres. Supported floating types have at most 64 bits,
    and conversion to float64 preserves their finite values without millimetre
    quantization. Nonpositive, infinite and NaN depths become NaN (unknown).

    Both rectification/registration arguments must explicitly be Python True.
    Require depth_kind="optical_axis_z", depth_unit="m", and
    source_provenance="synthetic". Equal image dimensions do not establish RGB
    registration or rectify lens distortion; this function does neither.

    The pixel/calibration/transform source frames must agree, and the transform
    target must be base_link. ``stamp_ns`` is the shared acquisition time in the
    declared clock domain, not receipt time. The caller must associate both
    buffers and calibration with that acquisition before invoking this function.
    Calibration objects already own their validated data; image buffers are
    copied because a renderer or SDK may reuse its storage on the next frame.
    """
    if not isinstance(intrinsics, PinholeIntrinsics):
        raise ValueError("intrinsics must be validated PinholeIntrinsics")
    if not isinstance(base_from_optical, RigidTransform):
        raise ValueError("base_from_optical must be a validated RigidTransform")
    if rectified is not True or rgb_registered_to_depth_grid is not True:
        raise ValueError("snapshot requires explicitly rectified and registered RGB-D")
    if depth_kind != "optical_axis_z" or depth_unit != "m":
        raise ValueError("snapshot requires optical_axis_z depth in metres")
    if source_provenance != "synthetic":
        raise ValueError("this snapshot adapter requires synthetic provenance")
    _frame_id(pixel_frame)
    if not (
        pixel_frame == intrinsics.frame_id == base_from_optical.source_frame
        and base_from_optical.target_frame == OBSERVATION_FRAME
    ):
        raise ValueError(
            "pixel/calibration/transform frames must agree and target base_link"
        )
    if (
        isinstance(stamp_ns, (bool, np.bool_))
        or not isinstance(stamp_ns, Integral)
        or stamp_ns < 0
    ):
        raise ValueError("stamp_ns must be a nonnegative acquisition integer")
    if not isinstance(clock_domain, str) or clock_domain not in CLOCK_DOMAINS:
        raise ValueError("unsupported acquisition clock_domain")
    shape = (intrinsics.height, intrinsics.width)
    if (
        not isinstance(rgb, np.ndarray)
        or rgb.dtype != np.uint8
        or rgb.shape != (*shape, 3)
    ):
        raise ValueError("rgb must be uint8 (H,W,3) matching calibration")
    if (
        not isinstance(depth_m, np.ndarray)
        or depth_m.dtype.kind != "f"
        or depth_m.dtype.itemsize > 8
        or depth_m.shape != shape
    ):
        raise ValueError(
            "depth_m must be floating (H,W), at most 64 bits, matching calibration"
        )
    depth = _real_array(depth_m, "depth_m")
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    return SceneInput(
        rgb=rgb.copy(),
        depth_m=depth,
        intrinsics=intrinsics,
        base_from_optical=base_from_optical,
        stamp_ns=int(stamp_ns),
        clock_domain=clock_domain,
        source_provenance=source_provenance,
        rectified=rectified,
        rgb_registered_to_depth_grid=rgb_registered_to_depth_grid,
    )
