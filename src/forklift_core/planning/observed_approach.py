"""Derive a planar approach target exclusively from acquired pocket geometry.

The transform must describe base_link at the observation's exact acquisition
time. A current robot pose cannot replace it after the robot has moved. This
bridge supplies a target, not obstacle clearance, insertion authorization, or
evidence of successful perception beyond the observation's own validity.
"""

from dataclasses import dataclass
from math import acos, atan2, cos, pi, sin
from numbers import Integral

import numpy as np

from forklift_core._validation import _finite_scalar, _frame_id
from forklift_core.geometry import FramePoints, RigidTransform
from forklift_core.perception.pallet_prior import PalletPrior
from forklift_core.perception.pocket_observation import CLOCK_DOMAINS, PocketObservation

from .geometry import Pose2D
from .pallet_mission import PalletSite, SyntheticMissionGeometry, site_poses


@dataclass(frozen=True)
class ObservedApproachTarget:
    """World-frame estimated pallet centre and rear-axle approach targets.

    status is valid, no_pallet, or invalid. Every geometry field is None on
    rejection. front_midpoint_world_m is the transformed observed pocket-front
    midpoint (x, y, z) in metres, not a pallet centre inferred from scene truth.
    Sigma fields are not propagated: this is a point target, not a covariance
    estimate or an assertion that an unreported uncertainty is zero.
    """

    success: bool
    status: str
    reason: str | None
    pallet_site: PalletSite | None = None
    approach_rear: Pose2D | None = None
    prealign_rear: Pose2D | None = None
    front_midpoint_world_m: tuple[float, float, float] | None = None


def _nonnegative_nanoseconds(value: int, name: str) -> None:
    if (
        isinstance(value, (bool, np.bool_))
        or not isinstance(value, Integral)
        or value < 0
    ):
        raise ValueError(f"{name} must be a nonnegative integer, excluding bool")


def observed_approach_goal(
    observation: PocketObservation,
    prior: PalletPrior,
    world_from_base: RigidTransform,
    *,
    transform_stamp_ns: int,
    transform_clock_domain: str,
    now_ns: int,
    now_clock_domain: str,
    max_age_ns: int = 250_000_000,
    max_tilt_rad: float = 0.05,
    geometry: SyntheticMissionGeometry | None = None,
    world_frame_id: str = "world",
) -> ObservedApproachTarget:
    """Convert a fresh base-frame observation into a world-frame approach goal.

    Acquisition, transform, and now must use the same declared clock domain;
    transform_stamp_ns must exactly equal observation.stamp_ns. Age equal to
    max_age_ns is accepted. Clock/timestamp mismatch, future/stale observation,
    synthetic_ground_truth provenance, and excessive tilt return invalid with
    a reason code and no targets. no_pallet/invalid observations preserve their
    status and detector reason. Malformed configuration or frames raise
    ValueError.

    Apply the complete 3D rigid transform to both the pocket midpoint and the
    insertion axis. Move half the explicitly supplied prior depth along that
    axis to estimate the pallet centre, then project into world XY. Robot
    approach/prealignment offsets come from the explicit or default named
    synthetic geometry (1.69/2.49 m behind that centre).

    The horizontal-pallet contract permits at most max_tilt_rad between the
    transformed base up axis and world up. Small accepted tilt still affects
    the complete 3D transform; large tilt is never silently flattened.
    """
    for name, value in (
        ("transform_stamp_ns", transform_stamp_ns),
        ("now_ns", now_ns),
        ("max_age_ns", max_age_ns),
    ):
        _nonnegative_nanoseconds(value, name)
    for name, value in (
        ("transform_clock_domain", transform_clock_domain),
        ("now_clock_domain", now_clock_domain),
    ):
        if not isinstance(value, str) or value not in CLOCK_DOMAINS:
            raise ValueError(f"unsupported {name}: {value!r}")
    max_tilt_rad = _finite_scalar(max_tilt_rad, "max_tilt_rad")
    if not 0 <= max_tilt_rad < pi / 2:
        raise ValueError("max_tilt_rad must lie in [0, pi/2)")
    depth_m = _finite_scalar(prior.overall_depth_m, "prior.overall_depth_m")
    if depth_m <= 0:
        raise ValueError("prior.overall_depth_m must be positive")
    _frame_id(world_frame_id)
    if world_from_base.source_frame != observation.frame_id:
        raise ValueError(
            "world_from_base source frame must match observation base frame"
        )
    if world_from_base.target_frame != world_frame_id:
        raise ValueError("world_from_base target frame must match world_frame_id")

    if observation.source_provenance == "synthetic_ground_truth":
        return ObservedApproachTarget(
            False, "invalid", "synthetic_ground_truth_observation"
        )
    if observation.status != "valid":
        return ObservedApproachTarget(False, observation.status, observation.reason)
    if not observation.clock_domain == transform_clock_domain == now_clock_domain:
        return ObservedApproachTarget(False, "invalid", "clock_domain_mismatch")
    if transform_stamp_ns != observation.stamp_ns:
        return ObservedApproachTarget(False, "invalid", "transform_timestamp_mismatch")
    if now_ns < observation.stamp_ns:
        return ObservedApproachTarget(False, "invalid", "future_observation")
    if now_ns - observation.stamp_ns > max_age_ns:
        return ObservedApproachTarget(False, "invalid", "stale_observation")

    tilt_rad = acos(float(np.clip(world_from_base.rotation[2, 2], -1, 1)))
    if tilt_rad > max_tilt_rad + 1e-12:
        return ObservedApproachTarget(False, "invalid", "nonplanar_transform")

    midpoint_base_m = 0.5 * np.asarray(observation.left.center_m) + 0.5 * np.asarray(
        observation.right.center_m
    )
    midpoint_world_m = world_from_base.apply(
        FramePoints(observation.frame_id, [midpoint_base_m])
    ).xyz_m[0]
    insertion_axis_world = world_from_base.rotation @ np.array(
        [
            cos(observation.insertion_yaw_rad),
            sin(observation.insertion_yaw_rad),
            0.0,
        ]
    )
    centre_world_m = midpoint_world_m + depth_m / 2 * insertion_axis_world
    insertion_yaw_world_rad = atan2(insertion_axis_world[1], insertion_axis_world[0])
    pallet_site = PalletSite(
        float(centre_world_m[0]), float(centre_world_m[1]), insertion_yaw_world_rad
    )
    poses = site_poses(pallet_site, geometry)
    return ObservedApproachTarget(
        True,
        "valid",
        None,
        pallet_site,
        poses["approach"],
        poses["prealign"],
        tuple(float(value) for value in midpoint_world_m),
    )
