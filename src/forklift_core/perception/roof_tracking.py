"""Current-depth continuation of a previously identified empty slatted pallet.

The visible roof rear edge measures range; longitudinal board/gap edges measure
heading and lateral phase. Hidden pocket positions are inferred from the known
rigid model. This is neither a general pallet detector nor an observation of
hidden pocket clearance. The caller must qualify it against direct front-pocket
observations before using continuation for motion.
"""

import math
from dataclasses import dataclass, fields

import numpy as np
from numpy.typing import ArrayLike, NDArray

from forklift_core._validation import _finite_scalar, _real_array
from forklift_core.perception.pallet_geometry import PalletGeometry
from forklift_core.perception.pocket_observation import (
    Pocket,
    PocketObservation,
    yaw_difference_rad,
)
from forklift_core.perception.rgbd_snapshot import scene_input_from_rgbd_snapshot
from forklift_core.perception.scene_dataset import SceneInput


@dataclass(frozen=True)
class RoofTrackingParams:
    """Evidence thresholds for near-horizontal, near-forward synthetic captures.

    Defaults match the 640x480 synthetic camera's tested insertion envelope.
    These are explicit model/measurement assumptions, not real camera calibration.
    Counts are pixels, distances metres, and orientations radians.
    """

    roof_height_tolerance_m: float = 0.004
    roof_plane_tolerance_m: float = 0.0015
    min_edge_height_drop_m: float = 0.002
    roi_margin_m: float = 0.10
    rear_association_m: float = 0.06
    lateral_association_m: float = 0.03
    yaw_association_rad: float = 0.08
    max_abs_yaw_rad: float = 0.25
    min_roof_points: int = 500
    min_rear_points: int = 80
    min_rear_span_m: float = 0.25
    rear_outlier_floor_m: float = 0.003
    max_rear_excess_rms_m: float = 0.003
    end_exclusion_m: float = 0.025
    min_groove_points: int = 100
    min_points_per_edge: int = 5
    min_edge_length_m: float = 0.035
    min_longitudinal_variance_m2: float = 0.02
    groove_line_tolerance_m: float = 0.0025
    pattern_tolerance_m: float = 0.003
    min_pattern_inlier_fraction: float = 0.85
    min_distinct_edges: int = 4
    min_interior_edges: int = 2
    uncertainty_floor_m: float = 0.002
    yaw_uncertainty_floor_rad: float = 0.001

    def __post_init__(self) -> None:
        counts = {
            "min_roof_points",
            "min_rear_points",
            "min_groove_points",
            "min_points_per_edge",
            "min_distinct_edges",
            "min_interior_edges",
        }
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name in counts:
                if type(value) is not int or value < 1:
                    raise ValueError(f"{field.name} must be a positive Python integer")
            elif _finite_scalar(value, field.name) <= 0:
                raise ValueError(f"{field.name} must be positive")
        if self.min_distinct_edges < 4 or self.min_interior_edges < 2:
            raise ValueError(
                "At least four edges including two interior edges required"
            )
        if self.min_interior_edges > self.min_distinct_edges:
            raise ValueError("Interior minimum cannot exceed total edge minimum")
        if self.min_points_per_edge < 3 or self.min_rear_points < 3:
            raise ValueError("Line fitting requires at least three measured points")
        if self.min_pattern_inlier_fraction > 1:
            raise ValueError("min_pattern_inlier_fraction cannot exceed one")
        if self.max_abs_yaw_rad >= math.pi / 4:
            raise ValueError("Roof continuation is limited to near-forward views")


@dataclass(frozen=True)
class RoofTrackingDiagnostics:
    """Measured evidence counts and residuals; absent estimates remain None."""

    reason: str
    roof_points: int = 0
    rear_points: int = 0
    groove_points: int = 0
    distinct_groove_edges: int = 0
    interior_groove_edges: int = 0
    measured_roof_height_m: float | None = None
    rear_rms_m: float | None = None
    rear_excess_rms_m: float | None = None
    groove_rms_m: float | None = None
    rear_half_pixel_m: float | None = None
    pattern_inlier_fraction: float | None = None
    front_center_m: tuple[float, float, float] | None = None
    yaw_rad: float | None = None


@dataclass(frozen=True)
class RoofTrackingResult:
    observation: PocketObservation
    diagnostics: RoofTrackingDiagnostics


def _result(
    scene: SceneInput,
    reason: str,
    diagnostics: dict,
    *,
    pockets=None,
    yaw=None,
    position_sigma=None,
    yaw_sigma=None,
) -> RoofTrackingResult:
    return RoofTrackingResult(
        PocketObservation(
            stamp_ns=scene.stamp_ns,
            clock_domain=scene.clock_domain,
            frame_id="base_link",
            source_provenance=scene.source_provenance,
            status="valid" if pockets is not None else "invalid",
            left=pockets[0] if pockets is not None else None,
            right=pockets[1] if pockets is not None else None,
            insertion_yaw_rad=yaw,
            position_sigma_m=position_sigma,
            yaw_sigma_rad=yaw_sigma,
            reason=None if pockets is not None else reason,
        ),
        RoofTrackingDiagnostics(reason=reason, **diagnostics),
    )


def _rays(scene: SceneInput, u: NDArray, v: NDArray) -> NDArray:
    k = scene.intrinsics
    optical = np.stack(((u - k.cx) / k.fx, (v - k.cy) / k.fy, np.ones_like(u)), axis=-1)
    return optical @ scene.base_from_optical.rotation.T


def _plane_points(scene: SceneInput, u: NDArray, v: NDArray, height: float) -> NDArray:
    directions = _rays(scene, u, v)
    origin = scene.base_from_optical.translation_m
    distance = np.full(u.shape, np.nan)
    np.divide(
        height - origin[2],
        directions[..., 2],
        out=distance,
        where=np.abs(directions[..., 2]) > 1e-9,
    )
    distance[distance <= 0] = np.nan
    return origin + distance[..., None] * directions


def _slope(x: NDArray, y: NDArray) -> float | None:
    dx, dy = x - np.mean(x), y - np.mean(y)
    denominator = float(dx @ dx)
    return float(dx @ dy / denominator) if denominator > 1e-12 else None


_DEFAULT_PARAMS = RoofTrackingParams()


def track_roof(
    scene: SceneInput,
    geometry: PalletGeometry,
    expected_front_center_m: ArrayLike,
    expected_yaw_rad: float,
    params: RoofTrackingParams = _DEFAULT_PARAMS,
) -> RoofTrackingResult:
    """Infer current base-frame pockets from measured roof edges, with no memory.

    ``expected_front_center_m`` is a finite XY or XYZ front-opening midpoint in
    base_link metres. XY assumes the pallet bottom at z=0; XYZ gives the expected
    opening-centre height. This prior restricts ROI and board-edge association;
    output range, lateral phase, yaw and height all require current depth.

    RGB/depth follow SceneInput's registered/rectified, optical-axis-Z contract.
    This first implementation accepts only synthetic captures; ground-truth
    provenance and malformed contracts raise ValueError. Missing evidence returns
    an invalid observation carrying this acquisition's time. It does not fall
    back to the expected pose. Only near-forward horizontal empty pallets are
    supported; roof texture alone cannot establish hidden pocket clearance.
    """
    if not isinstance(scene, SceneInput):
        raise ValueError("scene must be SceneInput")
    if not isinstance(geometry, PalletGeometry) or not isinstance(
        params, RoofTrackingParams
    ):
        raise ValueError("geometry and params must be validated typed objects")
    # Reuse the public capture contract instead of a second, divergent validator.
    scene = scene_input_from_rgbd_snapshot(
        rgb=scene.rgb,
        depth_m=scene.depth_m,
        intrinsics=scene.intrinsics,
        base_from_optical=scene.base_from_optical,
        pixel_frame=getattr(scene.intrinsics, "frame_id", ""),
        stamp_ns=scene.stamp_ns,
        clock_domain=scene.clock_domain,
        source_provenance=scene.source_provenance,
        rectified=scene.rectified,
        rgb_registered_to_depth_grid=scene.rgb_registered_to_depth_grid,
        depth_kind="optical_axis_z",
        depth_unit="m",
    )
    reference = _real_array(expected_front_center_m, "expected_front_center_m")
    if reference.shape not in ((2,), (3,)) or not np.isfinite(reference).all():
        raise ValueError("expected_front_center_m must be finite XY or XYZ")
    expected_yaw = _finite_scalar(expected_yaw_rad, "expected_yaw_rad")
    diagnostic = {}
    if abs(expected_yaw) > params.max_abs_yaw_rad:
        return _result(scene, "unsupported_view_orientation", diagnostic)
    if geometry.top_board_pitch_m <= geometry.top_board_width_m:
        return _result(scene, "model_has_no_roof_gaps", diagnostic)
    if geometry.overall_depth_m <= 2 * params.end_exclusion_m:
        return _result(scene, "insufficient_model_depth", diagnostic)
    expected_height = geometry.overall_height_m
    if reference.shape == (3,):
        expected_height += reference[2] - geometry.opening_centre_height_m
    if scene.base_from_optical.translation_m[2] <= expected_height:
        return _result(scene, "camera_not_above_roof", diagnostic)

    expected_n = np.array([math.cos(expected_yaw), math.sin(expected_yaw)])
    expected_t = np.array([-expected_n[1], expected_n[0]])
    vv, uu = np.indices(scene.depth_m.shape)
    xyz = scene.base_from_optical.translation_m + scene.depth_m[..., None] * _rays(
        scene, uu, vv
    )
    good = np.isfinite(scene.depth_m)
    offset = xyz[..., :2] - reference[:2]
    longitudinal, transverse = offset @ expected_n, offset @ expected_t
    roi = (
        (longitudinal > -params.roi_margin_m)
        & (longitudinal < geometry.overall_depth_m + params.roi_margin_m)
        & (np.abs(transverse) < geometry.overall_width_m / 2 + params.roi_margin_m)
    )
    roof = (
        good
        & roi
        & (np.abs(xyz[..., 2] - expected_height) < params.roof_height_tolerance_m)
    )
    diagnostic["roof_points"] = int(np.sum(roof))
    if diagnostic["roof_points"] < params.min_roof_points:
        return _result(scene, "insufficient_roof", diagnostic)
    height = float(np.median(xyz[..., 2][roof]))
    diagnostic["measured_roof_height_m"] = height
    roof = good & roi & (np.abs(xyz[..., 2] - height) < params.roof_plane_tolerance_m)
    diagnostic["roof_points"] = int(np.sum(roof))
    if diagnostic["roof_points"] < params.min_roof_points:
        return _result(scene, "insufficient_roof_plane", diagnostic)
    low = good & (xyz[..., 2] < height - params.min_edge_height_drop_m)

    # An image-border or unknown neighbor is not a measured rear boundary.
    v, u = np.argmax(roof, axis=0), np.arange(roof.shape[1])
    keep = roof.any(axis=0) & (v > 1) & (v < roof.shape[0] - 3)
    keep &= low[np.maximum(v - 1, 0), u]
    rear_u, rear_v = u[keep], v[keep] - 0.5
    rear_points = _plane_points(scene, rear_u, rear_v, height)
    expected_rear_axis = reference[:2] @ expected_n + geometry.overall_depth_m
    selected = np.isfinite(rear_points).all(axis=1) & (
        np.abs(rear_points[:, :2] @ expected_n - expected_rear_axis)
        < params.rear_association_m
    )
    rear_points, rear_u, rear_v = (
        rear_points[selected],
        rear_u[selected],
        rear_v[selected],
    )
    for _ in range(4):
        if len(rear_points) < params.min_rear_points:
            return _result(scene, "insufficient_rear_edge", diagnostic)
        a, b = rear_points[:, :2] @ expected_n, rear_points[:, :2] @ expected_t
        if np.ptp(b) < params.min_rear_span_m:
            return _result(scene, "insufficient_rear_span", diagnostic)
        slope = _slope(b, a)
        if slope is None:
            return _result(scene, "degenerate_rear_edge", diagnostic)
        residual = a - (slope * b + np.mean(a) - slope * np.mean(b))
        selected = np.abs(residual) < max(
            params.rear_outlier_floor_m, 3 * float(np.median(np.abs(residual)))
        )
        rear_points, rear_u, rear_v = (
            rear_points[selected],
            rear_u[selected],
            rear_v[selected],
        )
    if len(rear_points) < params.min_rear_points:
        return _result(scene, "insufficient_rear_inliers", diagnostic)
    a, b = rear_points[:, :2] @ expected_n, rear_points[:, :2] @ expected_t
    slope = _slope(b, a)
    if slope is None or np.ptp(b) < params.min_rear_span_m:
        return _result(scene, "insufficient_rear_span", diagnostic)
    yaw = expected_yaw - math.atan(slope)
    if abs(yaw_difference_rad(yaw, expected_yaw)) > params.yaw_association_rad:
        return _result(scene, "rear_orientation_mismatch", diagnostic)
    n, t = (
        np.array([math.cos(yaw), math.sin(yaw)]),
        np.array([-math.sin(yaw), math.cos(yaw)]),
    )
    rear = float(np.median(rear_points[:, :2] @ n))
    diagnostic["rear_points"] = len(rear_points)

    edge = (roof[:, :-1] & low[:, 1:]) | (low[:, :-1] & roof[:, 1:])
    v, u = np.nonzero(edge)
    grooves = _plane_points(scene, u + 0.5, v, height)
    axis = grooves[:, :2] @ n
    selected = (
        np.isfinite(grooves).all(axis=1)
        & (axis < rear - params.end_exclusion_m)
        & (axis > rear - geometry.overall_depth_m + params.end_exclusion_m)
    )
    # Increasing image u points toward decreasing pallet y for the allowed view.
    signs = np.where(roof[v, u], -1, 1)[selected]
    grooves = grooves[selected]
    if len(grooves) < params.min_groove_points:
        return _result(scene, "insufficient_known_groove_edges", diagnostic)
    board_edges = np.array(
        [
            center + sign * geometry.top_board_width_m / 2
            for center in geometry.top_board_centres_y_m()
            for sign in (-1, 1)
        ]
    )
    edge_sides = np.tile([-1, 1], geometry.top_board_count)
    expected_phase = reference[:2] @ t
    distances = np.abs(
        grooves[:, :2] @ t[:, None] - expected_phase - board_edges[None, :]
    )
    distances = np.where(signs[:, None] == edge_sides[None, :], distances, np.inf)
    identities = np.argmin(distances, axis=1)

    # Fit common longitudinal direction after removing each edge's own intercept.
    # Exclusion is monotonic: rejected samples must not re-enter as zero residuals.
    active = np.ones(len(grooves), dtype=bool)
    longitudinal, transverse = grooves[:, :2] @ expected_n, grooves[:, :2] @ expected_t
    for _ in range(3):
        dx, dy = np.zeros(len(grooves)), np.zeros(len(grooves))
        for identity in np.unique(identities[active]):
            group = (identities == identity) & active
            if (
                np.sum(group) < params.min_points_per_edge
                or np.ptp(longitudinal[group]) < params.min_edge_length_m
            ):
                active[group] = False
                continue
            dx[group] = longitudinal[group] - np.mean(longitudinal[group])
            dy[group] = transverse[group] - np.mean(transverse[group])
        variance = float(dx[active] @ dx[active])
        if variance < params.min_longitudinal_variance_m2:
            return _result(scene, "insufficient_groove_length", diagnostic)
        slope = float(dx[active] @ dy[active] / variance)
        active &= np.abs(dy - slope * dx) < params.groove_line_tolerance_m
    yaw = expected_yaw + math.atan(slope)
    if abs(yaw_difference_rad(yaw, expected_yaw)) > params.yaw_association_rad:
        return _result(scene, "groove_orientation_mismatch", diagnostic)
    n, t = (
        np.array([math.cos(yaw), math.sin(yaw)]),
        np.array([-math.sin(yaw), math.cos(yaw)]),
    )
    rear = float(np.median(rear_points[:, :2] @ n))
    rear_rms = float(np.sqrt(np.mean((rear_points[:, :2] @ n - rear) ** 2)))
    diagnostic["rear_rms_m"] = rear_rms
    # A straight physical edge becomes a staircase in the image. Each measured
    # boundary brackets its true location between adjacent pixel-centre rays.
    # Only residual beyond that measured interval is evidence of non-straightness.
    lower = _plane_points(scene, rear_u, rear_v - 0.5, height)
    upper = _plane_points(scene, rear_u, rear_v + 0.5, height)
    if not (np.isfinite(lower).all() and np.isfinite(upper).all()):
        return _result(scene, "unbounded_edge_projection", diagnostic)
    lower_axis, upper_axis = lower[:, :2] @ n, upper[:, :2] @ n
    edge_min, edge_max = (
        np.minimum(lower_axis, upper_axis),
        np.maximum(lower_axis, upper_axis),
    )
    # Minimize squared distance to all measured pixel intervals. A median of
    # interval midpoints is biased toward the more populated staircase row.
    lo, hi = float(np.min(edge_min)), float(np.max(edge_max))
    for _ in range(32):
        rear = (lo + hi) / 2
        derivative = float(np.sum(rear - np.clip(rear, edge_min, edge_max)))
        if abs(derivative) < 1e-12:
            break
        if derivative > 0:
            hi = rear
        else:
            lo = rear
    rear_rms = float(np.sqrt(np.mean((rear_points[:, :2] @ n - rear) ** 2)))
    diagnostic["rear_rms_m"] = rear_rms
    excess = np.maximum(np.maximum(edge_min - rear, rear - edge_max), 0)
    excess_rms = float(np.sqrt(np.mean(excess**2)))
    half_pixel = float(np.median(edge_max - edge_min) / 2)
    diagnostic.update(rear_excess_rms_m=excess_rms, rear_half_pixel_m=half_pixel)
    if excess_rms > params.max_rear_excess_rms_m:
        return _result(scene, "rear_not_straight", diagnostic)
    phases = grooves[:, :2] @ t - board_edges[identities]
    if not np.any(active):
        return _result(scene, "insufficient_groove_inliers", diagnostic)
    phase = float(np.median(phases[active]))
    residual = np.abs(phases - phase)
    inliers = active & (residual < params.pattern_tolerance_m)
    fraction = float(np.mean(inliers))
    distinct = np.unique(identities[inliers])
    interior = distinct[(distinct > 0) & (distinct < len(board_edges) - 1)]
    diagnostic.update(
        groove_points=int(np.sum(inliers)),
        distinct_groove_edges=len(distinct),
        interior_groove_edges=len(interior),
        pattern_inlier_fraction=fraction,
    )
    if (
        fraction < params.min_pattern_inlier_fraction
        or len(distinct) < params.min_distinct_edges
        or len(interior) < params.min_interior_edges
        or np.sum(inliers) < params.min_groove_points
    ):
        return _result(scene, "inconsistent_groove_pattern", diagnostic)
    if abs(phase - reference[:2] @ t) > params.lateral_association_m:
        return _result(scene, "groove_phase_outside_association", diagnostic)

    front_xy = (rear - geometry.overall_depth_m) * n + phase * t
    front_z = height - geometry.overall_height_m + geometry.opening_centre_height_m
    diagnostic.update(front_center_m=(*map(float, front_xy), front_z), yaw_rad=yaw)
    groove_rms = float(np.sqrt(np.mean(residual[inliers] ** 2)))
    # Pixel-edge quantization is systematic, so do not divide by sample count.
    diagnostic["groove_rms_m"] = groove_rms
    sigma = max(params.uncertainty_floor_m, half_pixel, rear_rms, groove_rms)
    yaw_sigma = max(
        params.yaw_uncertainty_floor_rad, groove_rms / params.min_edge_length_m
    )
    pockets = tuple(
        Pocket(
            (
                *map(float, front_xy + sign * geometry.opening_centre_offset_m * t),
                front_z,
            ),
            geometry.opening_width_m,
            geometry.block_height_m,
        )
        for sign in (1, -1)
    )
    return _result(
        scene,
        "measured_roof_continuation",
        diagnostic,
        pockets=pockets,
        yaw=yaw,
        position_sigma=sigma,
        yaw_sigma=yaw_sigma,
    )
