"""Depth-only geometric baseline for a known, horizontal pallet shape.

The workspace assumes the synthetic floor is base z=0. Pocket centre height
and opening height come from the explicit prior, not a depth measurement.
Raw depth rays, including unknown samples, decide opening visibility. This
module owns neither catalogue labels nor ground truth; RGB is not inspected.
"""

import math
import time
import traceback
from dataclasses import dataclass, fields
from numbers import Integral

import numpy as np
from numpy.typing import NDArray

from forklift_core._validation import _finite_scalar
from forklift_core.perception.pallet_prior import PalletPrior
from forklift_core.perception.pocket_observation import (
    Pocket,
    PocketObservation,
    yaw_difference_rad,
)
from forklift_core.perception.scene_dataset import SceneInput
from forklift_core.sensors.rgbd import deproject_depth_pixels


@dataclass(frozen=True)
class DetectorParams:
    """Finite algorithm parameters, separate from explicitly supplied shape."""

    cell_m: float = 0.01
    plane_inlier_m: float = 0.02
    band_margin_m: float = 0.01
    # Dev-tuning starting value, not derived from plane residuals. For EPAL 6,
    # the unchanged floor filter leaves less than 2 mm of downward tolerance.
    deck_evidence_tol_m: float = 0.006
    ransac_iterations: int = 200
    min_plane_points: int = 300
    min_band_points: int = 100
    # Per-opening upper-deck threshold. Separate from min_band_points because
    # the two answer different questions -- how much evidence a support column
    # must show, against how much deck must sit over one opening -- and the
    # measurement that would set them apart needs a real sensor. Zero means
    # "follow min_band_points", which is the frozen behaviour.
    upper_band_points: int = 0
    max_plane_candidates: int = 3
    range_min_m: float = 0.8
    range_max_m: float = 5.0
    floor_z_m: float = 0.02
    front_margin_m: float = 0.05
    occluded_front_frac: float = 0.5
    open_behind_frac: float = 0.3
    # Dev-tuned 2026-09-13: correct front planes reach 12.5 mm p95 under
    # oblique views, so 10 mm rejected them without suppressing any
    # false pattern. See docs/validation/2026-09-13-pocket-detector-m2.md.
    max_plane_residual_m: float = 0.015
    width_mismatch_frac: float = 0.20
    seed: int = 20260913

    def __post_init__(self) -> None:
        integers = {
            "ransac_iterations",
            "min_plane_points",
            "min_band_points",
            "upper_band_points",
            "max_plane_candidates",
            "seed",
        }
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name in integers:
                minimum = 0 if field.name in {"seed", "upper_band_points"} else 1
                if (
                    isinstance(value, (bool, np.bool_))
                    or not isinstance(value, Integral)
                    or value < minimum
                ):
                    raise ValueError(f"{field.name} must be an integer >= {minimum}")
            else:
                _finite_scalar(value, field.name)
        for name in (
            "cell_m",
            "plane_inlier_m",
            "deck_evidence_tol_m",
            "front_margin_m",
            "max_plane_residual_m",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.band_margin_m < 0 or self.floor_z_m < 0:
            raise ValueError("band_margin_m and floor_z_m must be nonnegative")
        if not 0 < self.range_min_m < self.range_max_m:
            raise ValueError("range must satisfy 0 < min < max")
        for name in ("occluded_front_frac", "open_behind_frac"):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError(f"{name} must lie in (0, 1]")
        if not 0 < self.width_mismatch_frac < 1:
            raise ValueError("width_mismatch_frac must lie in (0, 1)")


_DEFAULT_PARAMS = DetectorParams()


@dataclass(frozen=True)
class OpeningRayCounts:
    """Disjoint ray counts; near-plane and unknown returns stay in total."""

    front: int
    behind: int
    near_plane: int
    unknown: int
    total: int

    @property
    def front_fraction(self) -> float:
        return self.front / self.total if self.total else 0.0

    @property
    def behind_fraction(self) -> float:
        return self.behind / self.total if self.total else 0.0


@dataclass(frozen=True)
class DetectionDiagnostics:
    """Fit and sampling diagnostics, without an unvalidated uncertainty bound."""

    plane_residual_p95_m: float | None
    plane_inlier_count: int
    candidate_plane_count: int
    rejected_plane_reasons: tuple[str, ...]
    opening_rays: dict[str, OpeningRayCounts]
    opening_width_raw_m: tuple[float, float] | None
    boundary_resolution_m: float
    centre_height_is_prior: bool
    elapsed_s: float
    seed: int
    # Evidence of the selected pattern, per term. Recorded even when the
    # observation is invalid, because the reason a shape was refused is the
    # count that fell short, and no table could show it before.
    selected_support_min: int | None
    selected_lower: int | None
    selected_upper_left: int | None
    selected_upper_right: int | None
    exception_traceback: str | None = None


@dataclass(frozen=True)
class DetectionResult:
    observation: PocketObservation
    diagnostics: DetectionDiagnostics


@dataclass(frozen=True)
class _Plane:
    point: NDArray[np.float64]
    normal: NDArray[np.float64]
    points: NDArray[np.float64]
    residual_p95_m: float

    @property
    def left_axis(self):
        return np.array((self.normal[1], -self.normal[0], 0.0))


@dataclass(frozen=True)
class _Pattern:
    # Lateral bounds in increasing order: right opening first, then left.
    gaps: tuple[tuple[float, float], ...]
    support_count: int
    deck_count: int
    # Evidence kept per term rather than summed. support_count is a total and
    # cannot stand in for the weakest column, and a caller that recovers `lower`
    # by subtracting `upper` from `deck_count` is reading a coincidence.
    support_min: int
    lower: int
    upper_left: int
    upper_right: int
    # Both openings must show the deck above them. A pattern that fails this is
    # not a pallet seen badly; it is a shape that has no upper deck over one of
    # its gaps, and it can never be reported valid.
    upper_ok: bool


def _base_points(scene):
    """Return one base point and one base ray per pixel, in image row order."""
    intrinsics = scene.intrinsics
    v, u = np.indices(scene.depth_m.shape)
    pixels = np.column_stack((u.ravel(), v.ravel()))
    optical = deproject_depth_pixels(
        scene.depth_m,
        pixels,
        intrinsics,
        meters_per_unit=1.0,
        pixel_frame=scene.pixel_frame,
        rectified=scene.rectified,
    )
    points = scene.base_from_optical.apply(optical).xyz_m
    rays_optical = np.column_stack(
        (
            (pixels[:, 0] - intrinsics.cx) / intrinsics.fx,
            (pixels[:, 1] - intrinsics.cy) / intrinsics.fy,
            np.ones(len(pixels)),
        )
    )
    rays = rays_optical @ scene.base_from_optical.rotation.T
    return points, rays


def _filter_workspace(points, camera, prior, params):
    horizontal_range = np.hypot(points[:, 0] - camera[0], points[:, 1] - camera[1])
    mask = (
        np.isfinite(points).all(axis=1)
        & (points[:, 2] > params.floor_z_m)
        & (points[:, 2] <= prior.height_m + 0.10)
        & (horizontal_range >= params.range_min_m)
        & (horizontal_range <= params.range_max_m)
    )
    return points[mask]


def _refit_vertical(points, camera):
    """Least-squares XY line fit, extruded vertically; n_z is exactly zero."""
    centre = points.mean(axis=0)
    delta = points[:, :2] - centre[:2]
    _, vectors = np.linalg.eigh(delta.T @ delta)
    normal = np.array((*vectors[:, 0], 0.0))
    if np.dot(normal, camera - centre) < 0:
        normal = -normal
    residual = np.abs((points - centre) @ normal)
    return _Plane(centre, normal, points, float(np.percentile(residual, 95)))


def _vertical_plane_candidates(points, camera, params):
    """Extract up to the configured number of RANSAC vertical-plane candidates."""
    rng = np.random.default_rng(params.seed)
    remaining = points
    planes = []
    for _ in range(params.max_plane_candidates):
        if len(remaining) < max(3, params.min_plane_points):
            break
        best_mask, best_count = None, 0
        for _ in range(params.ransac_iterations):
            sample = remaining[rng.choice(len(remaining), 3, replace=False)]
            normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-12:
                continue
            normal /= norm
            if abs(normal[2]) >= 0.1:
                continue
            residual = np.abs((remaining - sample[0]) @ normal)
            mask = residual <= params.plane_inlier_m
            count = int(mask.sum())
            if count > best_count:
                best_mask, best_count = mask, count
        if best_count < params.min_plane_points:
            break
        plane = _refit_vertical(remaining[best_mask], camera)
        # The grid and residual must use inliers of the vertical model, not
        # the slightly tilted hypothesis that RANSAC used to find that model.
        # Recover shared evidence consumed by earlier candidate planes.
        residual = np.abs((points - plane.point) @ plane.normal)
        full_mask = residual <= params.plane_inlier_m
        if np.count_nonzero(full_mask) < params.min_plane_points:
            break
        plane = _Plane(
            plane.point,
            plane.normal,
            points[full_mask],
            float(np.percentile(residual[full_mask], 95)),
        )
        planes.append(plane)
        remaining_residual = np.abs((remaining - plane.point) @ plane.normal)
        remaining = remaining[remaining_residual > params.plane_inlier_m]
    return planes


def _column_grid(points, prior, params):
    """Input columns are (normal coordinate, lateral metres, base z metres)."""
    mask = (points[:, 2] >= prior.deck_bottom_m + params.band_margin_m) & (
        points[:, 2] <= prior.height_m - prior.deck_top_m - params.band_margin_m
    )
    lateral = points[mask, 1]
    if not len(lateral):
        return np.zeros(0, dtype=int), 0
    # A tiny index epsilon keeps exact metre-grid boundaries stable in float64.
    indices = np.floor(lateral / params.cell_m + 1e-9).astype(int)
    origin = int(indices.min())
    return np.bincount(indices - origin), origin


def occupied_columns(
    points: NDArray[np.float64], prior: PalletPrior, params: DetectorParams
) -> NDArray[np.bool_]:
    """Return occupied columns between observed extrema for (n, lateral, z) points.

    Points are float64[N,3] in metres. Columns use the guarded opening-height
    band; outside space is never padded into the search window.
    """
    counts, _ = _column_grid(points, prior, params)
    return counts > 0


def _gap_runs(columns):
    indices = np.flatnonzero(columns)
    if not len(indices):
        return []
    start, stop = indices[0], indices[-1] + 1
    gaps = []
    first = None
    for index in range(start, stop):
        if not columns[index] and first is None:
            first = index
        elif columns[index] and first is not None:
            gaps.append((first, index))
            first = None
    return gaps


def count_interior_gaps(columns: NDArray[np.bool_]) -> int:
    """Count only empty runs bounded by occupied columns on both sides."""
    return len(_gap_runs(columns))


def _opening_candidates(plane, prior, params, workspace):
    lateral = plane.points @ plane.left_axis
    local = np.column_stack((np.zeros(len(lateral)), lateral, plane.points[:, 2]))
    counts, origin = _column_grid(local, prior, params)
    gaps = _gap_runs(counts > 0)
    workspace_lateral = workspace @ plane.left_axis
    # The normal points towards the camera, so depth behind the plane is negative
    # signed normal distance. No front-side allowance: even a 0.5 mm strip ahead
    # of the face must not count as the lower deck.
    depth = -(workspace - plane.point) @ plane.normal
    lower_band = (
        (np.abs(workspace[:, 2] - prior.deck_bottom_m) <= params.deck_evidence_tol_m)
        & (depth >= 0)
        & (depth <= prior.overall_depth_m + params.plane_inlier_m)
    )
    patterns = []
    for first, second in zip(gaps, gaps[1:], strict=False):
        # The occupied spacer includes up to two boundary cells.
        spacer = (second[0] - first[1]) * params.cell_m
        if not (
            prior.centre_spacer_min_m - 2 * params.cell_m
            <= spacer
            <= prior.centre_spacer_max_m + 2 * params.cell_m
        ):
            continue
        supports = (
            counts[: first[0]].sum(),
            counts[first[1] : second[0]].sum(),
            counts[second[1] :].sum(),
        )
        left_edge = (origin + first[0]) * params.cell_m
        right_edge = (origin + second[1]) * params.cell_m
        over_openings = (lateral >= left_edge) & (lateral <= right_edge)
        lower = np.count_nonzero(
            lower_band
            & (workspace_lateral >= left_edge)
            & (workspace_lateral <= right_edge)
        )
        upper = np.count_nonzero(
            over_openings & (local[:, 2] >= prior.height_m - prior.deck_top_m)
        )
        if min(*supports, lower, upper) < params.min_band_points:
            continue
        bounds = tuple(
            ((origin + start) * params.cell_m, (origin + stop) * params.cell_m)
            for start, stop in (first, second)
        )
        # Per-opening upper-deck evidence, from the plane's own inliers inside a
        # band no thicker than the deck itself. The combined count above cannot
        # distinguish a pallet from a shape with a deck over one gap and nothing
        # over the other, because one strong opening carries the sum.
        deck_band = (local[:, 2] >= prior.height_m - prior.deck_top_m) & (
            local[:, 2] <= prior.height_m + params.plane_inlier_m
        )
        per_opening = []
        for start, stop in (first, second):
            low = (origin + start) * params.cell_m
            high = (origin + stop) * params.cell_m
            per_opening.append(
                int(
                    np.count_nonzero(
                        deck_band & (lateral >= low) & (lateral <= high)
                    )
                )
            )
        # bounds are (right, left); report the pair in the same order.
        upper_right, upper_left = per_opening
        upper_threshold = params.upper_band_points or params.min_band_points
        upper_ok = min(per_opening) >= upper_threshold
        patterns.append(
            _Pattern(
                bounds,
                int(sum(supports)),
                int(lower + upper_left + upper_right),
                int(min(supports)),
                int(lower),
                upper_left,
                upper_right,
                upper_ok,
            )
        )
    return patterns


def _classify_opening_rays(scene, rays, plane, gap, prior, params):
    """Use unfiltered original depth, counting every ray through the rectangle."""
    camera = scene.base_from_optical.translation_m
    denominator = rays @ plane.normal
    distances = np.full(len(rays), np.nan)
    np.divide(
        (plane.point - camera) @ plane.normal,
        denominator,
        out=distances,
        where=np.abs(denominator) > 1e-12,
    )
    intersections = camera + distances[:, None] * rays
    lateral = intersections @ plane.left_axis
    mask = (
        (distances > 0)
        & (lateral >= gap[0])
        & (lateral <= gap[1])
        & (intersections[:, 2] >= prior.deck_bottom_m + params.band_margin_m)
        & (
            intersections[:, 2]
            <= prior.height_m - prior.deck_top_m - params.band_margin_m
        )
    )
    depth = scene.depth_m.ravel()[mask]
    unknown = ~np.isfinite(depth) | (depth <= 0)
    signed = (depth - distances[mask]) * denominator[mask]
    front = ~unknown & (signed > params.front_margin_m)
    behind = ~unknown & (signed < -params.front_margin_m)
    total = int(mask.sum())
    return OpeningRayCounts(
        int(front.sum()),
        int(behind.sum()),
        int((~(front | behind | unknown)).sum()),
        int(unknown.sum()),
        total,
    )


def _build_observation(scene, prior, params, plane, pattern, opening_rays):
    # Reconstruct lengths from integer cell counts: subtracting translated
    # bounds can turn an exact 0.30 m grid width into 0.30000000000000004.
    raw_widths = [
        round((high - low) / params.cell_m) * params.cell_m
        for low, high in reversed(pattern.gaps)
    ]
    if (
        any(
            not prior.opening_width_min_m - 2 * params.cell_m
            <= width
            <= prior.opening_width_max_m
            for width in raw_widths
        )
        or abs(raw_widths[0] - raw_widths[1]) / max(raw_widths)
        >= params.width_mismatch_frac
    ):
        return _status_observation(scene, "invalid", "opening_width_mismatch")
    for side in ("left", "right"):
        counts = opening_rays[side]
        if counts.front_fraction > params.occluded_front_frac:
            return _status_observation(scene, "invalid", f"pocket_occluded:{side}")
        if counts.behind_fraction < params.open_behind_frac:
            return _status_observation(scene, "invalid", f"pocket_ambiguous:{side}")
    pockets = []
    # Half-cell correction belongs to reported geometry, after raw validation.
    widths = [width + params.cell_m for width in raw_widths]
    for gap, width in zip(reversed(pattern.gaps), widths, strict=True):
        centre = (
            plane.point
            + (sum(gap) / 2 - plane.point @ plane.left_axis) * plane.left_axis
        )
        centre[2] = prior.opening_centre_height_m
        pockets.append(Pocket(tuple(centre), width, prior.opening_height_m))
    yaw = yaw_difference_rad(
        math.atan2(plane.normal[1], plane.normal[0]) + math.pi, 0.0
    )
    return PocketObservation(
        scene.stamp_ns,
        scene.clock_domain,
        "base_link",
        scene.source_provenance,
        "valid",
        *pockets,
        yaw,
        None,
        None,
        None,
    )


def _upper_deck_reason(pattern, params):
    """Name which upper-deck failure this is, keeping the side when there is one."""
    counts = {"right": pattern.upper_right, "left": pattern.upper_left}
    threshold = params.upper_band_points or params.min_band_points
    if max(counts.values()) < threshold:
        return "no_upper_deck"
    side = min(counts, key=counts.__getitem__)
    return f"upper_deck_occluded:{side}"


def _status_observation(scene, status, reason):
    return PocketObservation(
        scene.stamp_ns,
        scene.clock_domain,
        "base_link",
        scene.source_provenance,
        status,
        None,
        None,
        None,
        None,
        None,
        reason,
    )


def detect_pockets(
    scene_input: SceneInput,
    prior: PalletPrior,
    params: DetectorParams = _DEFAULT_PARAMS,
) -> DetectionResult:
    """Estimate pockets from depth and an explicit prior, with no evaluation labels.

    Configuration errors raise ValueError. Missing or insufficient observations
    return no_pallet/invalid. Both sigmas are unknown (None) in this baseline.
    """
    if 2 * params.band_margin_m >= prior.opening_height_m:
        raise ValueError("band_margin_m must leave a nonempty opening height band")
    start = time.perf_counter()
    planes, rejected, candidates = [], {}, []
    selected_plane, selected_pattern, selected_rays = None, None, {}
    exception_traceback = None
    try:
        points, rays = _base_points(scene_input)
        camera = scene_input.base_from_optical.translation_m
        if np.isfinite(points).all(axis=1).sum() < params.min_plane_points:
            observation = _status_observation(
                scene_input, "invalid", "insufficient_points"
            )
        else:
            workspace = _filter_workspace(points, camera, prior, params)
            planes = _vertical_plane_candidates(workspace, camera, params)
            for index, plane in enumerate(planes):
                if plane.residual_p95_m > params.max_plane_residual_m:
                    rejected[index] = "vertical_refit_residual"
                    continue
                patterns = _opening_candidates(plane, prior, params, workspace)
                if not patterns:
                    rejected[index] = "no_opening_pattern"
                for pattern in patterns:
                    opening_rays = {
                        side: _classify_opening_rays(
                            scene_input, rays, plane, gap, prior, params
                        )
                        for side, gap in zip(
                            ("right", "left"), pattern.gaps, strict=True
                        )
                    }
                    score = pattern.support_count + pattern.deck_count
                    score *= 1 + sum(
                        count.behind_fraction for count in opening_rays.values()
                    )
                    distance = float(np.linalg.norm((plane.point - camera)[:2]))
                    candidates.append(
                        (score, -distance, index, plane, pattern, opening_rays)
                    )
            if candidates:
                (
                    _,
                    _,
                    selected_index,
                    selected_plane,
                    selected_pattern,
                    selected_rays,
                ) = max(candidates, key=lambda candidate: candidate[:2])
                for index in range(len(planes)):
                    if index != selected_index and index not in rejected:
                        rejected[index] = "lower_pattern_score"
                if not selected_pattern.upper_ok:
                    # A pallet has deck over both openings. Something that does
                    # not is a real object in view, not an absent one, so the
                    # status is invalid rather than no_pallet: the contract
                    # reserves no_pallet for nothing being there.
                    #
                    # Separate absence from obstruction. Deck over neither
                    # opening is a different shape; deck over one and not the
                    # other is this pallet with something in the way, and the
                    # side is worth keeping -- collapsing both to one reason
                    # discards which pocket the caller cannot trust.
                    observation = _status_observation(
                        scene_input, "invalid", _upper_deck_reason(selected_pattern, params)
                    )
                else:
                    observation = _build_observation(
                        scene_input,
                        prior,
                        params,
                        selected_plane,
                        selected_pattern,
                        selected_rays,
                    )
            else:
                observation = _status_observation(
                    scene_input,
                    "no_pallet",
                    "no_front_plane" if not planes else "no_opening_pattern",
                )
    except Exception as exc:
        exception_traceback = traceback.format_exc()
        observation = _status_observation(
            scene_input, "invalid", f"exception:{type(exc).__name__}"
        )
    diagnostics = DetectionDiagnostics(
        selected_plane.residual_p95_m if selected_plane else None,
        len(selected_plane.points) if selected_plane else 0,
        len(planes),
        tuple(rejected[index] for index in sorted(rejected)),
        selected_rays,
        tuple(high - low for low, high in reversed(selected_pattern.gaps))
        if selected_pattern
        else None,
        params.cell_m,
        True,
        time.perf_counter() - start,
        params.seed,
        selected_pattern.support_min if selected_pattern else None,
        selected_pattern.lower if selected_pattern else None,
        selected_pattern.upper_left if selected_pattern else None,
        selected_pattern.upper_right if selected_pattern else None,
        exception_traceback,
    )
    return DetectionResult(observation, diagnostics)
