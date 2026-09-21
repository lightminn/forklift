"""SDK-free numerical measurements for G1, plan v11 (2026-09-21).

Integer pixel indices are centres, exactly as pocket_detector._base_points.
RGB corners retain subpixel coordinates. Repeated renders are never pooled as
independent calibration points. All plane comparisons use the base_link origin.
"""

from dataclasses import asdict
from math import ceil

import numpy as np

from forklift_core.geometry import rotation_matrix_from_quaternion_xyzw
from forklift_core.sensors.rgbd import PinholeIntrinsics

DISTANCES_M = (0.8, 1.0, 2.0, 3.0, 4.0, 5.0)
DISTANCE_EDGES_M = (0.8, 0.9, 1.5, 2.5, 3.5, 4.5, 5.0)
HEIGHTS_M = (0.0, 0.022, 0.144)
SCREEN_CENTRES_UV = tuple(
    (u, v) for v in (80.0, 240.0, 400.0) for u in (100.0, 320.0, 540.0)
)
BOARD_CORNERS = (9, 7)
CORNER_SPACING_PX = 16.0
REPEATS = 3
GRID_STRIDE_PX = 2
EDGE_EROSION_PX = 2
# Empirical principal-point envelope from the 2026-09-21 G1 run, section 5.4.
# It establishes only that each axis stays below a whole pixel; the erosion
# margin below is +1 px for any envelope in (0, 1], so this number does not
# size that margin and a smaller measurement would not shrink it. This is a
# selection uncertainty budget, NOT a new acceptance limit for 2a.
HEIGHT_PRINCIPAL_POINT_BOUND_PX = 0.1442
GRID_SAMPLES_PER_BIN = 20
MIN_PLANE_SAMPLES = 6
# Predeclared floor for the coverage rule (USER decision, 2026-09-21; not a
# judgment this code made). A distance/screen bin whose baseline holds fewer
# than this many grid samples cannot carry a statistic, which is the same
# judgment MIN_PLANE_SAMPLES already encodes for board planes. Losing such a
# bin to the selection margin is therefore recorded with its count instead of
# failing the gate: one pixel at the image border must not fail a run. It is
# currently the same number as MIN_PLANE_SAMPLES but its own constant, so the
# two can diverge. A bin AT or above this floor that the margin empties still
# fails coverage, exactly as before.
MIN_COVERAGE_BASELINE_SAMPLES = 6
# Measurement region of a height panel: forward and lateral half extents as a
# fraction of the anchor horizontal range. Unchanged since plan v11; the
# viewing-direction margin below is added outside it and is never measured.
HEIGHT_PANEL_HALF_EXTENTS = (0.20, 0.16)
# Only the far central board is rotated, and only so its endpoint column has a
# measurable plane. Everything else is front-parallel on purpose: with dZ/du=0
# the depth gates are structurally insensitive to RGB corner error
# (docs/validation/2026-09-21-g1-calibration-run.md, section 13.1).
FAR_CENTRAL_TILT_DEG = 10.0

# Plan v11: one source for both recorded criteria and executable judgments.
# The two point references are diagnostic only, never acceptance gates.
G1_LIMITS = {
    "focal_relative": 0.0012,
    "principal_point_px": 0.1,
    "intrinsics_standard_error_px": 0.1,
    "holdout_euclidean_p95_px": 0.5,
    "mount_translation_m": 0.001,
    "mount_rotation_rad": 0.00022,
    "depth_bias_m": 0.001,
    "depth_centered_p95_m": 0.002,
    "plane_normal_rad": 0.00022,
    "plane_offset_m": 0.001,
    "height_bias_m": 0.0005,
    "height_absolute_p95_m": 0.001,
    "point_reference_p95_m": 0.001,
    "point_reference_max_m": 0.002,
    "usd_getter_matrix_agreement_atol": 1e-6,
}


def height_selection_margin() -> dict:
    """Record the predeclared selection-only margin and its empirical scope."""
    # u=fx*X/Z+cx, v=fy*Y/Z+cy: with focal lengths fixed, delta(c) is
    # exactly the image-boundary translation. A subpixel shift can flip an
    # integer boundary row/column, so the square erosion kernel needs one
    # whole extra Chebyshev pixel above the evaluation's unchanged 2 px.
    # ceil() therefore yields +1 for ANY envelope in (0, 1] px: the cited
    # observations size nothing here, they only establish that the envelope is
    # below 1. An envelope above 1 px would need a wider margin, and nothing
    # measured so far says it would get one. This is not a bound on focal
    # error, arbitrary rasterization, or future fits, nor a diagnosis of the
    # RGB fitting bias. Never tune it from depth.
    extra = ceil(HEIGHT_PRINCIPAL_POINT_BOUND_PX)
    return {
        "principal_point_bound_px": HEIGHT_PRINCIPAL_POINT_BOUND_PX,
        "additional_selection_erosion_px": extra,
        "evaluation_erosion_px": EDGE_EROSION_PX,
        "selection_erosion_px": EDGE_EROSION_PX + extra,
        "derivation": "delta(u,v)=delta(cx,cy) at fixed f; ceil() of the envelope, which is +1 square-kernel pixel for any envelope in (0, 1] px; the measured envelope establishes only that it is below 1, it does not scale this margin",
        "source": "docs/validation/2026-09-21-g1-calibration-run.md section 5.4",
        "scope": "observed principal-point envelope only; RGB bias cause unresolved",
    }


def protocol() -> dict:
    """Return the predeclared sampling policy, persisted before any rendering."""
    return {
        # v11-3 supersedes v11-1, the version every executed run recorded up to
        # section 11 of the run record. It adds the warm-up capture that no fit
        # uses and the height panels' viewing-direction margin, so the sampling
        # policy differs from v11-1 and results must not be pooled across them.
        "version": "G1-v11-3",
        "distance_anchors_m": DISTANCES_M,
        "distance_edges_m": DISTANCE_EDGES_M,
        "distance_definition": "hypot(sample_base.xy - camera_base.xy); not optical Z",
        "interval_convention": "left closed, right open; final interval includes 5.0",
        "screen_bins": "3 x 3 equal image rectangles; u then v; integer index centres",
        "board_centres_uv": SCREEN_CENTRES_UV,
        "board_corners": BOARD_CORNERS,
        "corner_spacing_px": CORNER_SPACING_PX,
        "far_central_board_optical_y_rotation_deg": FAR_CENTRAL_TILT_DEG,
        "corners_per_board_per_repeat": 63,
        "repeats": REPEATS,
        "holdout": "(corner_row + corner_column) % 4 == 0, fixed before rendering",
        "repeat_policy": "separate fit per distance anchor and repeat; never pool repeated views",
        "grid_stride_px": GRID_STRIDE_PX,
        "edge_erosion_px": EDGE_EROSION_PX,
        "height_selection_margin": height_selection_margin(),
        "grid_selection": "before depth capture: erode the rendered panel ROI by evaluation erosion plus selection-only margin, confine the result to the measurement region, take stride-grid centres, select min(20, eligible count) per bin by linspace over row-major indices; record baseline/current counts and lost bins; a bin whose baseline holds at least minimum_coverage_baseline_samples grid samples and that the margin empties fails coverage, while a baseline below that floor is recorded as BELOW_BASELINE_FLOOR with its count and does not fail; every selected sample must survive the semantic and finite-depth checks; no replacement or residual rejection",
        "grid_samples_per_observed_bin_cap": GRID_SAMPLES_PER_BIN,
        "minimum_plane_corners_per_observed_bin": MIN_PLANE_SAMPLES,
        "minimum_coverage_baseline_samples": MIN_COVERAGE_BASELINE_SAMPLES,
        "corner_depth": "bilinear axial depth, all four neighbours in eroded same-label ROI",
        "height_depth": "native integer depth grid; nominal K and nominal mount; no RGB rounding",
        "height_judgment": "bias and absolute p95 per distance interval; screen-cell statistics are diagnostics; every preselected sample is required",
        "height_surfaces_base_m": HEIGHTS_M,
        "height_panel_size_m": "measurement region forward 0.40*r, lateral 0.32*r, surface at h, no thickness; the rendered panel adds a viewing-direction margin near and far, sized as the metres that move each measurement edge by the selection erosion in rows, so the eroded selection still reaches the extreme distance bins; the margin is rendered but never selected, measured or counted as coverage",
        "empty_bin_status": "UNOBSERVED, never PASS; expected-visible missing data fails coverage",
        "guarantee": "Only tested layouts and populated predeclared bins: plane normal/offset and height statistics. No all-point 3D error guarantee.",
        "limits": dict(G1_LIMITS),
    }


def transform_points(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Transform N x 3 metre points using a column-convention homogeneous matrix."""
    points = np.asarray(points, dtype=float)
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def mount_matrix(mount) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = mount.rotation, mount.translation_m
    return matrix


def rigid_matrix(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=float)
    if (
        matrix.shape != (4, 4)
        or not np.isfinite(matrix).all()
        or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7, rtol=0)
        or not np.allclose(
            matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6, rtol=0
        )
        or abs(np.linalg.det(matrix[:3, :3]) - 1) > 1e-6
    ):
        raise ValueError("Expected a finite rigid transform in metres (no scale/shear)")
    return matrix


def read_mount(
    camera,
    world_from_base_row: np.ndarray,
    world_from_camera_row: np.ndarray,
    nominal: np.ndarray,
) -> tuple:
    """G1③: always read ROS optical axes, cross-check transposed USD transforms.

    Inputs must be read at the same paused render time, in metre stage units.
    USD camera local axes are converted with diag(1,-1,-1,1). Agreement of these
    two scene readbacks is not an independent render calibration.
    """
    position, wxyz = camera.get_local_pose(camera_axes="ros")
    w, x, y, z = np.asarray(wxyz, dtype=float)
    actual = np.eye(4)
    actual[:3, :3] = rotation_matrix_from_quaternion_xyzw([x, y, z, w])
    actual[:3, 3] = position
    wb, wc = (
        rigid_matrix(np.asarray(world_from_base_row).T),
        rigid_matrix(np.asarray(world_from_camera_row).T),
    )
    crosscheck = np.linalg.inv(wb) @ wc @ np.diag([1, -1, -1, 1])
    rigid_matrix(actual)
    rigid_matrix(nominal)
    if not np.allclose(
        actual, crosscheck, atol=G1_LIMITS["usd_getter_matrix_agreement_atol"], rtol=0
    ):
        raise ValueError("Camera getter and USD transform paths disagree")
    delta = actual[:3, :3] @ nominal[:3, :3].T
    angle = float(np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1)))
    distance = float(np.linalg.norm(actual[:3, 3] - nominal[:3, 3]))
    return actual, {
        "primary_path": 'Camera.get_local_pose(camera_axes="ros")',
        "crosscheck_path": "inv(ComputeLocalToWorldTransform(base).T) @ ComputeLocalToWorldTransform(camera).T @ diag(1,-1,-1,1)",
        "base_from_optical": actual.tolist(),
        "usd_base_from_optical": crosscheck.tolist(),
        "comparison_frame": "base_link",
        "position_error_m": distance,
        "rotation_error_rad": angle,
        "status": "PASS"
        if distance <= G1_LIMITS["mount_translation_m"]
        and angle <= G1_LIMITS["mount_rotation_rad"]
        else "FAIL",
    }


def project(points_optical: np.ndarray, calibration: PinholeIntrinsics) -> np.ndarray:
    points = np.asarray(points_optical, dtype=float)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or not np.isfinite(points).all()
        or (points[:, 2] <= 0).any()
    ):
        raise ValueError("Projection requires finite forward optical points")
    return points[:, :2] / points[:, 2, None] * [calibration.fx, calibration.fy] + [
        calibration.cx,
        calibration.cy,
    ]


def backproject(
    uv: np.ndarray,
    depth_m: np.ndarray,
    calibration: PinholeIntrinsics,
    base_from_optical: np.ndarray,
) -> np.ndarray:
    """Preserve fractional RGB coordinates; grid callers supply integer centres."""
    xy = (np.asarray(uv) - [calibration.cx, calibration.cy]) / [
        calibration.fx,
        calibration.fy,
    ]
    optical = np.column_stack((xy, np.ones(len(xy)))) * np.asarray(depth_m)[:, None]
    return transform_points(base_from_optical, optical)


def fit_intrinsics(
    points_optical: np.ndarray,
    uv: np.ndarray,
    holdout: np.ndarray,
    nominal: PinholeIntrinsics,
) -> dict:
    """OLS K and Cov = residual_variance * inv(A.T A), N-2 degrees of freedom.

    Holdout points never enter either fit or covariance. Report 2D Euclidean
    held-out residuals, not either axis and not residuals of a fitted centroid.
    """
    points, uv, holdout = (
        np.asarray(points_optical),
        np.asarray(uv),
        np.asarray(holdout, dtype=bool),
    )
    if (
        len(points) != len(uv)
        or holdout.shape != (len(points),)
        or (~holdout).sum() < 3
        or holdout.sum() < 1
        or not np.isfinite(points).all()
        or not np.isfinite(uv).all()
        or (points[:, 2] <= 0).any()
    ):
        raise ValueError("Insufficient or invalid independent fit/holdout points")
    ratios = points[:, :2] / points[:, 2, None]
    estimated, errors, covariance = {}, {}, {}
    predicted = np.empty_like(uv, dtype=float)
    for axis, focal, centre in [(0, "fx", "cx"), (1, "fy", "cy")]:
        a = np.column_stack((ratios[~holdout, axis], np.ones((~holdout).sum())))
        if np.linalg.matrix_rank(a) < 2:
            raise ValueError("Calibration design matrix is rank deficient")
        beta = np.linalg.lstsq(a, uv[~holdout, axis], rcond=None)[0]
        residual = uv[~holdout, axis] - a @ beta
        cov = (residual @ residual) / (len(a) - 2) * np.linalg.inv(a.T @ a)
        estimated[focal], estimated[centre] = map(float, beta)
        errors[focal], errors[centre] = map(float, np.sqrt(np.diag(cov)))
        covariance[focal + "_" + centre] = cov.tolist()
        predicted[:, axis] = ratios[:, axis] * beta[0] + beta[1]
    residuals = uv - predicted
    p95 = float(np.percentile(np.linalg.norm(residuals[holdout], axis=1), 95))
    within = all(
        abs(estimated[k] / getattr(nominal, k) - 1) <= G1_LIMITS["focal_relative"]
        for k in ("fx", "fy")
    )
    within &= all(
        abs(estimated[k] - getattr(nominal, k)) <= G1_LIMITS["principal_point_px"]
        for k in ("cx", "cy")
    )
    within &= all(
        value <= G1_LIMITS["intrinsics_standard_error_px"] for value in errors.values()
    )
    return {
        "estimated": estimated,
        "standard_errors_px": errors,
        "covariance_px2": covariance,
        "fit_count": int((~holdout).sum()),
        "holdout_count": int(holdout.sum()),
        "heldout_p95_px": p95,
        "residuals_uv_px": residuals.tolist(),
        "holdout": holdout.tolist(),
        "gate_2a": "PASS" if within else "FAIL",
        "gate_2b": "PASS" if p95 <= G1_LIMITS["holdout_euclidean_p95_px"] else "FAIL",
    }


def semantic_mask(payload: dict, label: str) -> np.ndarray:
    """Resolve IDs only from idToLabels, accepting string or list class values."""
    labels = payload["info"]["idToLabels"]
    ids = []
    for key, value in labels.items():
        classes = value.get("class", [])
        if label == classes or (
            isinstance(classes, (list, tuple)) and label in classes
        ):
            ids.append(int(key))
    if not ids:
        raise ValueError(f"Semantic label missing: {label}")
    array = np.asarray(payload["data"])
    if array.ndim == 3 and array.shape[-1] == 1:
        array = array[..., 0]
    if array.ndim != 2 or array.dtype.kind not in "ui":
        raise ValueError("Semantic segmentation must be an uncolorized integer grid")
    return np.isin(array, ids)


def sample_depth_bilinear(
    depth: np.ndarray, uv: np.ndarray, interior: np.ndarray
) -> np.ndarray:
    """Sample all four valid, same-surface neighbours; never round a RGB corner."""
    depth, uv = np.asarray(depth), np.asarray(uv)
    result = np.full(len(uv), np.nan)
    for i, (u, v) in enumerate(uv):
        if not np.isfinite([u, v]).all():
            continue
        x, y = int(np.floor(u)), int(np.floor(v))
        if x < 0 or y < 0 or x + 1 >= depth.shape[1] or y + 1 >= depth.shape[0]:
            continue
        values = depth[y : y + 2, x : x + 2]
        if (
            not interior[y : y + 2, x : x + 2].all()
            or not np.isfinite(values).all()
            or (values <= 0).any()
        ):
            continue
        dx, dy = u - x, v - y
        result[i] = np.array([1 - dy, dy]) @ values @ np.array([1 - dx, dx])
    return result


def fit_plane(points: np.ndarray) -> tuple[np.ndarray, float]:
    """Unconstrained 3D TLS plane n.p=d, unit n; origin is the caller's base_link."""
    points = np.asarray(points, dtype=float)
    if len(points) < 3 or not np.isfinite(points).all():
        raise ValueError("Insufficient finite plane points")
    centre = points.mean(axis=0)
    _, singular, vh = np.linalg.svd(points - centre, full_matrices=False)
    if singular[1] <= 1e-10:
        raise ValueError("Plane points are collinear")
    normal = vh[-1]
    return normal, float(normal @ centre)


def plane_metrics(reconstructed_base: np.ndarray, truth_base: np.ndarray) -> dict:
    n, d = fit_plane(reconstructed_base)
    truth_n, truth_d = fit_plane(truth_base)
    if n @ truth_n < 0:
        n, d = -n, -d
    angle = float(
        np.arctan2(np.linalg.norm(np.cross(n, truth_n)), np.clip(n @ truth_n, -1, 1))
    )
    offset_error = float(abs(d - truth_d))
    return {
        "frame": "base_link",
        "normal": n.tolist(),
        "offset_m": d,
        "truth_normal": truth_n.tolist(),
        "truth_offset_m": truth_d,
        "count": len(reconstructed_base),
        "normal_angle_rad": angle,
        "offset_error_m": offset_error,
        "status": "PASS"
        if angle <= G1_LIMITS["plane_normal_rad"]
        and offset_error <= G1_LIMITS["plane_offset_m"]
        else "FAIL",
    }


def sample_bins(
    points_base: np.ndarray,
    uv: np.ndarray,
    camera_base: np.ndarray,
    calibration: PinholeIntrinsics,
) -> np.ndarray:
    """Assign actual sample XY distances, never plate-centre distance or axial Z."""
    distance = np.linalg.norm(
        np.asarray(points_base)[:, :2] - np.asarray(camera_base)[:2], axis=1
    )
    bins = np.searchsorted(DISTANCE_EDGES_M, distance, side="right") - 1
    bins[distance == DISTANCE_EDGES_M[-1]] = len(DISTANCE_EDGES_M) - 2
    bins[
        (distance < DISTANCE_EDGES_M[0])
        | (distance > DISTANCE_EDGES_M[-1])
        | ~np.isfinite(distance)
    ] = -1
    screen = np.floor(
        np.asarray(uv) / [calibration.width / 3, calibration.height / 3]
    ).astype(int)
    return np.column_stack((bins, screen))


def error_metrics(errors_m, *, kind: str) -> dict:
    values = np.asarray(errors_m, dtype=float)
    if not len(values):
        return {"count": 0, "status": "UNOBSERVED"}
    if not np.isfinite(values).all():
        return {
            "count": len(values),
            "status": "FAIL",
            "reason": "nonfinite measurement",
        }
    bias = float(values.mean())
    p95 = float(np.percentile(np.abs(values), 95))
    spread = float(np.percentile(np.abs(values - bias), 95))
    if kind == "depth":
        passed = (
            abs(bias) <= G1_LIMITS["depth_bias_m"]
            and spread <= G1_LIMITS["depth_centered_p95_m"]
        )
    elif kind == "height":
        passed = (
            abs(bias) <= G1_LIMITS["height_bias_m"]
            and p95 <= G1_LIMITS["height_absolute_p95_m"]
        )
    elif kind == "point":
        passed = True
    else:
        raise ValueError("Unknown error metric kind")
    return {
        "count": len(values),
        "bias_m": bias,
        "absolute_p95_m": p95,
        "centered_p95_m": spread,
        "max_abs_m": float(np.abs(values).max()),
        "status": "RECORD_ONLY" if kind == "point" else "PASS" if passed else "FAIL",
    }


def height_visibility(
    distance_m: float,
    u_px: float,
    height_m: float,
    calibration: PinholeIntrinsics,
    mount,
) -> dict:
    """Intersect a fixed image column with horizontal range and base surface h.

    Solve the column-plane intersection analytically at the nominated height.
    This uses actual mount/K for geometric visibility, not reconstructed depth.
    """
    matrix = mount_matrix(mount)
    a = matrix[:3, 0] - (u_px - calibration.cx) / calibration.fx * matrix[:3, 2]
    dz = height_m - matrix[2, 3]
    length = np.linalg.norm(a[:2])
    projection = -a[2] * dz / length
    if abs(projection) > distance_m:
        return {"visible": False, "v_px": None, "point_base_m": None}
    along = a[:2] / length
    across = np.array([-along[1], along[0]])
    span = np.sqrt(max(0.0, distance_m**2 - projection**2))
    candidates = [
        matrix[:3, 3] + np.r_[projection * along + sign * span * across, dz]
        for sign in [-1, 1]
    ]
    optical = transform_points(np.linalg.inv(matrix), np.array(candidates))
    index = int(np.argmax(optical[:, 2]))
    if optical[index, 2] <= 0:
        return {
            "visible": False,
            "v_px": None,
            "point_base_m": candidates[index].tolist(),
        }
    uv = project(optical[index : index + 1], calibration)[0]
    return {
        "visible": bool(
            0 <= uv[0] < calibration.width and 0 <= uv[1] < calibration.height
        ),
        "u_px": float(uv[0]),
        "v_px": float(uv[1]),
        "point_base_m": candidates[index].tolist(),
    }


def row_shift_metres(
    optical_point: np.ndarray,
    optical_axis: np.ndarray,
    rows_px: float,
    calibration: PinholeIntrinsics,
) -> float:
    """Metres along `optical_axis` that move a point's image row by `rows_px`.

    With v = fy*Y/Z + cy and q(m) = q0 + m*a the row shift is

        dv(m) = fy*m*(a_y*Z - a_z*Y) / (Z*(Z + m*a_z)),

    which is linear in m once the target shift is fixed:

        m = s*rows*Z^2 / (fy*(a_y*Z - a_z*Y) - s*rows*Z*a_z),  s = sign(dv).

    Both signs are solved and the smallest positive root is returned, so the
    caller gets the first place along that axis where the row has moved far
    enough. This is geometry only: no depth, render or tolerance enters.
    """
    y, z = float(optical_point[1]), float(optical_point[2])
    axis_y, axis_z = float(optical_axis[1]), float(optical_axis[2])
    gradient = calibration.fy * (axis_y * z - axis_z * y)
    roots = []
    for sign in (1.0, -1.0):
        denominator = gradient - sign * rows_px * z * axis_z
        if abs(denominator) < 1e-12:
            continue
        metres = sign * rows_px * z * z / denominator
        if np.isfinite(metres) and metres > 0:
            roots.append(float(metres))
    if not roots:
        raise ValueError("No forward extension reaches the requested row shift")
    return min(roots)


def height_panel_geometry(
    centre_base_m,
    distance_m: float,
    height_m: float,
    calibration: PinholeIntrinsics,
    actual_mount: np.ndarray,
) -> dict:
    """Split a height panel into its measurement region and a viewing margin.

    The measurement region is the plan v11 rectangle, unchanged. Around it the
    rendered panel is extended along the viewing axis only, by the metres that
    move its near and far edges `selection_erosion_px + 1` rows further out.
    The square selection kernel then bites into that margin instead of into the
    measurement region, so a distance bin that ends one stride-grid row short
    of the panel edge keeps its rows. Lateral extent, surface height, truth,
    the 2 px evaluation erosion and every acceptance limit are untouched.

    The extra row is integer rounding, not slack: a panel edge at continuous
    row p yields integer rows from ceil(p), so a measurement edge at row m is
    reached after eroding e pixels only when ceil(m) - ceil(p) >= e. A shift of
    exactly e leaves that difference at e-1 whenever the fractional parts fall
    the wrong way, and one more row closes it for every phase.
    """
    centre = np.asarray(centre_base_m, dtype=float)
    forward, lateral = (half * distance_m for half in HEIGHT_PANEL_HALF_EXTENTS)
    measurement = np.array(
        [
            [centre[0] - forward, centre[1] - lateral, height_m],
            [centre[0] + forward, centre[1] - lateral, height_m],
            [centre[0] + forward, centre[1] + lateral, height_m],
            [centre[0] - forward, centre[1] + lateral, height_m],
        ]
    )
    # A constant: the selection erosion plus one row of integer rounding. It
    # is not a share of the measured principal-point envelope, and no measured
    # number moves it. Only the metres below scale with the geometry.
    rows = height_selection_margin()["selection_erosion_px"] + 1
    optical_from_base = np.linalg.inv(rigid_matrix(actual_mount))
    optical = transform_points(optical_from_base, measurement)
    axis = optical_from_base[:3, :3] @ np.array([1.0, 0.0, 0.0])
    near = max(row_shift_metres(optical[i], -axis, rows, calibration) for i in (0, 3))
    far = max(row_shift_metres(optical[i], axis, rows, calibration) for i in (1, 2))
    panel = measurement.copy()
    panel[[0, 3], 0] -= near
    panel[[1, 2], 0] += far
    # Verify the derivation against the projection it was derived from, so a
    # mount or intrinsics that breaks the closed form fails here, not silently.
    shifts = np.abs(
        project(transform_points(optical_from_base, panel), calibration)[:, 1]
        - project(optical, calibration)[:, 1]
    )
    if shifts.min() < rows - 1e-9:
        raise ValueError("Height panel margin does not clear the selection erosion")
    return {
        "measurement_vertices_base_m": measurement.tolist(),
        "panel_vertices_base_m": panel.tolist(),
        "forward_half_extent_m": float(forward),
        "lateral_half_extent_m": float(lateral),
        "near_margin_m": float(near),
        "far_margin_m": float(far),
        "margin_rows_px": rows,
        "measured_row_shift_px": [float(value) for value in shifts],
        "margin_axis_base": "panel forward axis, base +x",
        "derivation": "rows = selection_erosion_px + 1, a constant for integer row rounding and not a function of any measured number; m = rows*Z^2 / (fy*(a_y*Z - a_z*Y) -+ rows*Z*a_z) per measurement edge",
        "scope": "rendered margin only; never selected, measured or counted as coverage",
    }


def surface_height_base(
    world_vertices: np.ndarray, world_from_base: np.ndarray
) -> float:
    """Read render surface (not centre) height after composed parent transforms."""
    vertices = transform_points(
        np.linalg.inv(rigid_matrix(world_from_base)), world_vertices
    )
    if not np.isfinite(vertices).all() or np.ptp(vertices[:, 2]) > 1e-7:
        raise ValueError("Render surface is not horizontal in base_link")
    return float(vertices[:, 2].mean())


def attach_annotators(rep, camera, *, annotators: dict | None = None) -> dict:
    """Bind channels, retaining partial ownership for the caller's finally block."""
    product = camera.get_render_product_path()
    if annotators is None:
        annotators = {}
    for key, name, kwargs in (
        ("rgb", "rgb", {}),
        ("seg", "semantic_segmentation", {"init_params": {"colorize": False}}),
        ("z", "distance_to_image_plane", {}),
    ):
        annotators[key] = rep.AnnotatorRegistry.get_annotator(name, **kwargs)
        annotators[key].attach([product])
    rep.orchestrator.set_capture_on_play(False)
    return annotators


def detach_annotators(annotators: dict) -> dict:
    """Release owned channels once; record teardown failures without masking data.

    Render-product changes can already have detached a channel. Query the public
    Replicator is_attached property and detach its own bindings, not the camera's
    possibly replaced product. Consume ownership so later cleanup cannot repeat.
    """
    report = {}
    for name in list(annotators):
        annotator = annotators.pop(name)
        try:
            if not annotator.is_attached:
                report[name] = {"status": "already_detached"}
                continue
            annotator.detach()
            report[name] = {"status": "detached"}
        except Exception as exc:
            # A connection can disappear between the readback and detach.
            already_detached = "not attached to any render products" in str(exc)
            report[name] = {
                "status": "already_detached" if already_detached else "error",
                "error": f"{type(exc).__name__}: {exc}",
            }
    return report


def capture_static(rep, annotators: dict) -> dict:
    """Own the annotator buffers from one explicitly paused zero-delta capture."""
    from copy import deepcopy

    rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=True)
    return {
        name: deepcopy(annotator.get_data()) for name, annotator in annotators.items()
    }


def semantic_diagnostics(payload, label: str, shape: tuple[int, int]) -> dict:
    """Describe raw ID pixels independently of label-map availability.

    Never reinterpret colorized bytes as IDs or infer identity from RGB. Shape
    and dtype describe the original buffer, including a possible singleton axis.
    """
    from copy import deepcopy

    report = {
        "target_label": label,
        "ready": False,
        "status": "missing_buffer",
        "data_shape": None,
        "data_dtype": None,
        "id_pixel_counts": {},
        "label_pixel_counts": {},
        "id_to_labels": {},
        "target_ids": [],
        "target_pixel_count": 0,
        "target_bbox_xyxy": None,
    }
    if not isinstance(payload, dict) or payload.get("data") is None:
        return report
    array = np.asarray(payload["data"])
    report.update(data_shape=list(array.shape), data_dtype=str(array.dtype))
    info = payload.get("info")
    labels = info.get("idToLabels", {}) if isinstance(info, dict) else {}
    report["id_to_labels"] = deepcopy(labels)
    if not array.size:
        report["status"] = "empty_buffer"
        return report
    if array.ndim == 3 and array.shape[-1] == 1:
        array = array[..., 0]
    if array.ndim != 2 or array.dtype.kind not in "ui":
        report["status"] = "invalid_format"
        return report
    ids, counts = np.unique(array, return_counts=True)
    report["id_pixel_counts"] = {
        str(i): int(n) for i, n in zip(ids, counts, strict=True)
    }
    if array.shape != shape:
        report["status"] = "shape_mismatch"
        return report
    if not isinstance(labels, dict):
        report["status"] = "invalid_label_map"
        return report
    label_ids = {}
    try:
        for key, value in labels.items():
            classes = value.get("class", [])
            if isinstance(classes, str):
                classes = [classes]
            for name in classes:
                label_ids.setdefault(name, set()).add(int(key))
    except (AttributeError, TypeError, ValueError):
        report["status"] = "invalid_label_map"
        return report
    report["label_pixel_counts"] = {
        name: sum(report["id_pixel_counts"].get(str(i), 0) for i in keys)
        for name, keys in label_ids.items()
    }
    report["target_ids"] = sorted(label_ids.get(label, []))
    report["unmapped_ids"] = [
        str(i) for i in ids if str(i) not in {str(k) for k in labels}
    ]
    mask = np.isin(array, report["target_ids"])
    report["target_pixel_count"] = int(mask.sum())
    if mask.any():
        rows, cols = np.nonzero(mask)
        report.update(
            ready=True,
            status="ready",
            target_bbox_xyxy=[
                int(cols.min()),
                int(rows.min()),
                int(cols.max()),
                int(rows.max()),
            ],
        )
    elif (
        np.all(array == 0)
        or report["label_pixel_counts"].get("BACKGROUND") == array.size
    ):
        report["status"] = "all_background"
    elif not report["target_ids"]:
        report["status"] = "target_label_missing"
    else:
        report["status"] = "target_mask_empty"
    return report


def bright_pixel_mask(rgb: np.ndarray) -> tuple[np.ndarray, float]:
    """Keep the original marker's mean-RGB, max(240, p99) criterion."""
    brightness = np.asarray(rgb)[..., :3].astype(float).mean(axis=2)
    threshold = max(240.0, float(np.percentile(brightness, 99)))
    return brightness >= threshold, threshold


def render_diagnostics(rgb: np.ndarray, depth: np.ndarray) -> dict:
    """Summarize RGB-D independently of semantics; ignore RGBA's alpha.

    Black RGB plus wholly nonfinite depth is an empty rendered view, not proof
    of a stopped renderer: an isolated, fully clipped target looks the same.
    No quality thresholds or missing-depth replacements are introduced here.
    """
    rgb, depth = np.asarray(rgb), np.asarray(depth)
    report = {
        "status": "invalid_render_buffers",
        "rgb": {"shape": list(rgb.shape), "dtype": str(rgb.dtype)},
        "depth": {"shape": list(depth.shape), "dtype": str(depth.dtype)},
    }
    rgb_valid = (
        rgb.ndim == 3 and rgb.shape[-1] in (3, 4) and rgb.size and rgb.dtype == np.uint8
    )
    if rgb_valid:
        channels = rgb[..., :3]
        report["rgb"].update(
            mean=float(channels.mean()),
            std=float(channels.std()),
            max=int(channels.max()),
            nonzero_count=int(np.count_nonzero(channels)),
        )
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    depth_valid = depth.ndim == 2 and depth.size and depth.dtype.kind in "fiu"
    if depth_valid:
        finite = depth[np.isfinite(depth)]
        report["depth"].update(
            finite_count=int(finite.size),
            finite_fraction=float(finite.size / depth.size),
            min_m=float(finite.min()) if finite.size else None,
            max_m=float(finite.max()) if finite.size else None,
        )
    if rgb_valid and depth_valid and rgb.shape[:2] == depth.shape:
        report["status"] = (
            "empty_render_frame"
            if report["rgb"]["nonzero_count"] == 0 and not finite.size
            else "nonempty"
        )
    return report


def target_clipping_diagnostics(
    vertices_world: np.ndarray, world_from_camera_usd: np.ndarray, clipping_range
) -> dict:
    """Compare composed geometry's optical Z with the camera's readback range."""
    near, far = map(float, clipping_range)
    optical = transform_points(
        np.linalg.inv(world_from_camera_usd @ np.diag([1, -1, -1, 1])), vertices_world
    )
    z = optical[:, 2]
    if not len(z) or not np.isfinite(z).all() or not 0 < near < far < np.inf:
        raise ValueError("Invalid target geometry or camera clipping range")
    return {
        "clipping_range_m": [near, far],
        "optical_z_min_m": float(z.min()),
        "optical_z_max_m": float(z.max()),
        "vertex_count": len(z),
        "vertices_in_depth_range": int(((z >= near) & (z <= far)).sum()),
        "all_before_near": bool(z.min() > 0 and z.max() < near),
        "all_beyond_far": bool(z.min() > far),
    }


def lower_near_clip_for_target(camera, target: dict) -> dict:
    """Correct a wholly near-clipped forward target; preserve far clip.

    This is a renderer view-volume setting, not a G1 measurement tolerance.
    Call only after persisting the original capture and geometry readback.
    """
    before = list(map(float, camera.get_clipping_range()))
    result = {"applied": False, "before_m": before, "after_m": before}
    if not target["all_before_near"]:
        return result
    if before != target["clipping_range_m"]:
        raise ValueError("Camera clipping readback changed since geometry check")
    near = target["optical_z_min_m"] / 2
    camera.set_clipping_range(near_distance=near)
    after = list(map(float, camera.get_clipping_range()))
    if not np.isclose(after[0], near, rtol=1e-6, atol=0) or after[1] != before[1]:
        raise ValueError("Camera clipping readback disagrees with near-only change")
    result.update(
        applied=True,
        after_m=after,
        rule="near = half the minimum positive target optical Z; far unchanged",
    )
    return result


def compare_annotator_streams(
    rep, existing: dict, fresh: dict, *, max_steps: int = 8, pipeline_state
) -> tuple[dict, dict]:
    """Diagnostic only: compare old/new bindings on the SAME paused steps.

    Both bindings can share renderer nodes. Recovery of both does not establish
    that reattachment was necessary, and no probe frame is a G1 measurement.
    """
    from copy import deepcopy

    attempts = []
    for step in range(1, max_steps + 1):
        before = deepcopy(pipeline_state())
        # One orchestrator step, then own both sets of channel buffers.
        old = capture_static(rep, existing)
        new = {name: deepcopy(a.get_data()) for name, a in fresh.items()}
        attempts.append(
            {
                "step": step,
                "pipeline_before": before,
                "pipeline_after": deepcopy(pipeline_state()),
                "existing": render_diagnostics(old["rgb"], old["z"]),
                "fresh": render_diagnostics(new["rgb"], new["z"]),
            }
        )
    old_status, new_status = (
        attempts[-1][key]["status"] for key in ("existing", "fresh")
    )
    return {
        "attempts": attempts,
        "observation": (
            "existing_empty_fresh_nonempty"
            if old_status == "empty_render_frame" and new_status == "nonempty"
            else "both_nonempty"
            if old_status == new_status == "nonempty"
            else "both_empty"
            if old_status == new_status == "empty_render_frame"
            else "inconclusive"
        ),
        "measurement_eligible": False,
    }, {"existing": old, "fresh": new}


def capture_semantic_static(
    rep, annotators: dict, *, target_label: str, max_steps: int = 8, pipeline_state=None
) -> dict:
    """Probe bounded paused steps; return last raw frame even when not ready.

    Every attempt owns ALL channels from the same step. Diagnostics are evidence
    of observed readiness only, not proof of a renderer-latency root cause or
    of pixel freshness after moving a target with the same semantic label.
    The caller must persist the result before require_capture_semantics().
    """
    from copy import deepcopy

    if isinstance(max_steps, bool) or not isinstance(max_steps, int) or max_steps < 1:
        raise ValueError("max_steps must be a positive integer")
    attempts = []
    first_ready_step = None
    for step in range(1, max_steps + 1):
        before = deepcopy(pipeline_state()) if pipeline_state is not None else None
        frame = capture_static(rep, annotators)
        rgb = np.asarray(frame["rgb"])
        segmentation = semantic_diagnostics(frame["seg"], target_label, rgb.shape[:2])
        record = {
            "step": step,
            "segmentation": segmentation,
            "render": render_diagnostics(rgb, frame["z"]),
        }
        if pipeline_state is not None:
            record.update(
                pipeline_before=before, pipeline_after=deepcopy(pipeline_state())
            )
        if (
            rgb.ndim == 3
            and rgb.shape[-1] in (3, 4)
            and rgb.size
            and rgb.dtype == np.uint8
        ):
            bright, threshold = bright_pixel_mask(rgb)
            record.update(bright_count=int(bright.sum()), bright_threshold=threshold)
            if segmentation["ready"]:
                mask = semantic_mask(frame["seg"], target_label)
                record["bright_inside_target_count"] = int((bright & mask).sum())
                depth = np.asarray(frame["z"])
                if depth.ndim == 3 and depth.shape[-1] == 1:
                    depth = depth[..., 0]
                if depth.shape == mask.shape and depth.dtype.kind in "fiu":
                    record["target_depth_finite_fraction"] = float(
                        np.isfinite(depth[mask]).mean()
                    )
        else:
            record.update(
                bright_count=None, bright_threshold=None, rgb_error="invalid_rgb"
            )
        attempts.append(record)
        if segmentation["ready"]:
            first_ready_step = step
            break
    frame["capture_diagnostics"] = {
        "target_label": target_label,
        "max_steps": max_steps,
        "ready": first_ready_step is not None,
        "first_ready_step": first_ready_step,
        "attempts": attempts,
        "empty_render_steps": [
            a["step"] for a in attempts if a["render"]["status"] == "empty_render_frame"
        ],
        "failure_reason": (
            attempts[-1]["render"]["status"]
            if attempts[-1]["render"]["status"] != "nonempty"
            else "segmentation_" + segmentation["status"]
            if first_ready_step is None
            else None
        ),
    }
    return frame


def require_capture_semantics(frame: dict) -> None:
    """Report RGB-D failure before downstream semantic/brightness symptoms."""
    report = frame["capture_diagnostics"]
    last = report["attempts"][-1]
    missing = []
    if last["render"]["status"] != "nonempty":
        missing.append(last["render"]["status"])
    elif not report["ready"]:
        missing.append("segmentation_" + last["segmentation"]["status"])
    if (
        report["target_label"] == "axis_marker"
        and last["render"]["status"] != "empty_render_frame"
    ):
        if last["bright_count"] is None:
            missing.append("bright_pixels_unavailable_invalid_rgb")
        elif last["bright_count"] == 0:
            missing.append("no_bright_pixels")
    if missing:
        raise ValueError(
            f"{report['target_label']} identification failed: {'; '.join(missing)} "
            f"(steps={len(report['attempts'])}/{report['max_steps']}, "
            f"target_pixels={last['segmentation']['target_pixel_count']}, "
            f"bright_pixels={last['bright_count']})"
        )


def checkerboard_corners(rgb: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Use semantics only as ROI; measure and canonically order RGB SB corners."""
    import cv2

    rows, cols = np.nonzero(mask)
    if not len(rows):
        raise ValueError("Checkerboard corners unavailable: empty semantic ROI")
    # No centroid as a proxy for corner coordinates and no integer rounding.
    gray = cv2.cvtColor(np.asarray(rgb)[..., :3], cv2.COLOR_RGB2GRAY)
    gray[~mask] = 255
    x0, x1 = max(0, int(cols.min()) - 4), min(gray.shape[1], int(cols.max()) + 5)
    y0, y1 = max(0, int(rows.min()) - 4), min(gray.shape[0], int(rows.max()) + 5)
    found, corners = cv2.findChessboardCornersSB(
        gray[y0:y1, x0:x1],
        BOARD_CORNERS,
        flags=cv2.CALIB_CB_ACCURACY,
    )
    if not found:
        raise ValueError("Checkerboard corners not detected")
    corners = corners.reshape(BOARD_CORNERS[1], BOARD_CORNERS[0], 2).astype(float)
    corners += [x0, y0]
    # Boards are authored with increasing row down and column right in the
    # nominal optical plane; G1③ separately rejects a wrong camera-axis setting.
    if corners[0, :, 1].mean() > corners[-1, :, 1].mean():
        corners = corners[::-1]
    if corners[:, 0, 0].mean() > corners[:, -1, 0].mean():
        corners = corners[:, ::-1]
    return corners.reshape(-1, 2)


def board_statistics(
    truth_base: np.ndarray,
    reconstructed_base: np.ndarray,
    uv: np.ndarray,
    depth_errors_m: np.ndarray,
    camera_base: np.ndarray,
    calibration: PinholeIntrinsics,
) -> dict:
    """Score actual corner subsets in fixed range/screen bins; ⑦b is diagnostic."""
    bins = sample_bins(truth_base, uv, camera_base, calibration)
    point_errors = np.linalg.norm(reconstructed_base - truth_base, axis=1)
    groups, depth_status, plane_status = [], [], []
    for distance in range(6):
        for u in range(3):
            for v in range(3):
                selected = (bins == [distance, u, v]).all(axis=1)
                n = int(selected.sum())
                depth = error_metrics(
                    np.asarray(depth_errors_m)[selected], kind="depth"
                )
                point = error_metrics(point_errors[selected], kind="point")
                plane = {"count": n, "status": "UNOBSERVED"}
                if n:
                    if n < MIN_PLANE_SAMPLES:
                        plane["status"] = "INSUFFICIENT"
                    else:
                        try:
                            plane = plane_metrics(
                                reconstructed_base[selected], truth_base[selected]
                            )
                        except ValueError as exc:
                            plane = {"count": n, "status": "FAIL", "reason": str(exc)}
                    depth_status.append(depth["status"])
                    plane_status.append(plane["status"])
                groups.append(
                    {
                        "bin": [distance, u, v],
                        "depth": depth,
                        "plane": plane,
                        "point": point,
                    }
                )
    return {
        "groups": groups,
        "out_of_range_count": int((bins[:, 0] == -1).sum()),
        "point": error_metrics(point_errors, kind="point"),
        "gate_4": "PASS"
        if depth_status and all(s == "PASS" for s in depth_status)
        else "FAIL",
        "gate_7a": "PASS"
        if plane_status and all(s == "PASS" for s in plane_status)
        else "FAIL",
    }


def default_tilt_deg(
    distance_m: float, centre_uv: tuple, calibration: PinholeIntrinsics
) -> float:
    """The authored rotation rule of the judging layout, as a readable value."""
    return (
        FAR_CENTRAL_TILT_DEG
        if distance_m == 5.0 and centre_uv[0] == calibration.cx
        else 0.0
    )


def checkerboard_layout(
    distance_m: float,
    centre_uv: tuple,
    calibration: PinholeIntrinsics,
    nominal_mount: np.ndarray,
    *,
    tilt_deg: float | None = None,
) -> dict:
    """Author full-screen 10x8 square boards; centre distance is horizontal XY.

    These are placement instructions only. Read the composed mesh vertices back
    at capture time to obtain the actual corner truth, including authored scale.

    tilt_deg is a diagnostic override for layout comparisons only. The judging
    run never passes it, so the authored rule below stays the measured layout.
    """
    ray = np.r_[
        (np.asarray(centre_uv) - [calibration.cx, calibration.cy])
        / [calibration.fx, calibration.fy],
        1.0,
    ]
    z = distance_m / np.linalg.norm((nominal_mount[:3, :3] @ ray)[:2])
    spacing = CORNER_SPACING_PX * z / calibration.fx
    centre = ray * z
    # With an untilted central board at r=5, every off-axis column has
    # hypot(x,y)>5. Only one collinear column remains in range. Rotate the
    # board about its unchanged centre so the endpoint has a measurable plane.
    angle = np.deg2rad(
        default_tilt_deg(distance_m, centre_uv, calibration)
        if tilt_deg is None
        else tilt_deg
    )
    c, s = np.cos(angle), np.sin(angle)
    board_rotation = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    vertices = np.array(
        [
            centre + board_rotation @ [(j - 5) * spacing, (i - 4) * spacing, 0]
            for i in range(9)
            for j in range(11)
        ]
    )
    faces = [
        [i * 11 + j, i * 11 + j + 1, (i + 1) * 11 + j + 1, (i + 1) * 11 + j]
        for i in range(8)
        for j in range(10)
    ]
    corners = [i * 11 + j for i in range(1, 8) for j in range(1, 10)]
    return {
        "vertices_base_m": transform_points(nominal_mount, vertices),
        "faces": np.array(faces),
        "corner_indices": np.array(corners),
        "white_faces": [i * 10 + j for i in range(8) for j in range(10) if (i + j) % 2],
        "black_faces": [
            i * 10 + j for i in range(8) for j in range(10) if not (i + j) % 2
        ],
        "border_base_m": transform_points(
            nominal_mount,
            np.array(
                [
                    centre + board_rotation @ [-6 * spacing, -5 * spacing, 1e-5],
                    centre + board_rotation @ [6 * spacing, -5 * spacing, 1e-5],
                    centre + board_rotation @ [6 * spacing, 5 * spacing, 1e-5],
                    centre + board_rotation @ [-6 * spacing, 5 * spacing, 1e-5],
                ]
            ),
        ),
    }


def erode_mask(mask: np.ndarray, *, radius: int | None = None) -> np.ndarray:
    """Erode with a square kernel; evaluation retains the default 2 px radius."""
    import cv2

    radius = EDGE_EROSION_PX if radius is None else radius
    return cv2.erode(
        np.asarray(mask, dtype=np.uint8),
        np.ones((2 * radius + 1, 2 * radius + 1), np.uint8),
        borderType=cv2.BORDER_CONSTANT,
        borderValue=0,
    ).astype(bool)


def within_rectangle(
    points_base: np.ndarray, axial_m: np.ndarray, vertices_base: np.ndarray
) -> np.ndarray:
    """Forward rays whose plane intersection lies inside a horizontal rectangle."""
    low, high = vertices_base.min(axis=0), vertices_base.max(axis=0)
    return (
        np.isfinite(axial_m)
        & (axial_m > 0)
        & (points_base[:, 0] >= low[0])
        & (points_base[:, 0] <= high[0])
        & (points_base[:, 1] >= low[1])
        & (points_base[:, 1] <= high[1])
    )


def height_grid_selection(
    vertices_base: np.ndarray,
    rendered_calibration: PinholeIntrinsics,
    actual_mount: np.ndarray,
    measurement_vertices_base: np.ndarray | None = None,
) -> dict:
    """Preselect up to 20 native grid samples per geometrically visible bin.

    No depth or semantic measurement enters selection. Bin coordinates use the
    readback plane, actual transform and independent rendered K. Height truth is
    solely the composed surface z, never a fitted reconstructed point cloud.

    `vertices_base` is the rendered panel, `measurement_vertices_base` the part
    of it that is measured. Both erosions run on the rendered panel, because
    that is the silhouette the renderer produces, and the result is confined to
    the measurement region. A viewing-direction margin therefore stops the
    selection kernel emptying an extreme distance bin without moving the
    measurement region, changing the truth or relaxing the coverage rule: a bin
    whose baseline reaches MIN_COVERAGE_BASELINE_SAMPLES and that the margin
    empties still fails. Defaulting the measurement region to the whole panel
    keeps the marginless behaviour.
    """
    k = rendered_calibration
    vertices = np.asarray(vertices_base)
    if np.ptp(vertices[:, 2]) > 1e-7:
        raise ValueError("Height panel must be horizontal in base_link")
    region = (
        vertices
        if measurement_vertices_base is None
        else np.asarray(measurement_vertices_base, dtype=float)
    )
    if np.ptp(region[:, 2]) > 1e-7 or abs(region[:, 2].mean() - vertices[0, 2]) > 1e-6:
        raise ValueError("Measurement region must be horizontal at the panel surface")
    # 1e-6 m is USD float32 point storage, not a measurement tolerance: the
    # readback panel and the requested region share their lateral edges exactly
    # in float64 and differ only by single-precision rounding of the vertices.
    if (region.min(axis=0)[:2] < vertices.min(axis=0)[:2] - 1e-6).any() or (
        region.max(axis=0)[:2] > vertices.max(axis=0)[:2] + 1e-6
    ).any():
        raise ValueError("Measurement region must lie inside the rendered panel")
    v, u = np.indices((k.height, k.width))
    uv = np.column_stack((u.ravel(), v.ravel()))
    rays = (
        np.column_stack(((uv - [k.cx, k.cy]) / [k.fx, k.fy], np.ones(len(uv))))
        @ actual_mount[:3, :3].T
    )
    axial = np.full(len(rays), np.nan)
    np.divide(
        vertices[0, 2] - actual_mount[2, 3],
        rays[:, 2],
        out=axial,
        where=np.abs(rays[:, 2]) > 1e-12,
    )
    # Finite dummy positions outside the mask avoid NaN-to-int warnings in bins.
    safe_axial = np.where(np.isfinite(axial), axial, 0.0)
    truth = rays * safe_axial[:, None] + actual_mount[:3, 3]
    inside = within_rectangle(truth, axial, vertices).reshape(k.height, k.width)
    measured = within_rectangle(truth, axial, region).reshape(k.height, k.width)
    margin = height_selection_margin()
    baseline_mask = (erode_mask(inside) & measured).ravel()
    geometric_mask = (
        erode_mask(inside, radius=margin["selection_erosion_px"]) & measured
    ).ravel()
    grid = ((uv[:, 0] % GRID_STRIDE_PX) == 0) & ((uv[:, 1] % GRID_STRIDE_PX) == 0)
    bins = sample_bins(truth, uv, actual_mount[:3, 3], k)
    indices, groups = [], []
    for distance in range(6):
        for col in range(3):
            for row in range(3):
                bin_grid = grid & (bins == [distance, col, row]).all(axis=1)
                baseline_count = int(np.count_nonzero(baseline_mask & bin_grid))
                candidate = np.flatnonzero(geometric_mask & bin_grid)
                n = min(len(candidate), GRID_SAMPLES_PER_BIN)
                selected = (
                    candidate[np.linspace(0, len(candidate) - 1, n, dtype=int)]
                    if n
                    else np.array([], dtype=int)
                )
                indices.extend(selected.tolist())
                groups.append(
                    {
                        "bin": [distance, col, row],
                        "baseline_geometric_count": baseline_count,
                        "baseline_planned_count": min(
                            baseline_count, GRID_SAMPLES_PER_BIN
                        ),
                        "geometric_count": len(candidate),
                        "planned_count": n,
                        "required_count": n,
                        # A bin the margin empties fails only if its baseline
                        # could have carried a statistic. Below the floor it is
                        # reclassified, never deleted: the bin and its baseline
                        # count stay on record under their own status.
                        "coverage_status": "COMPLETE"
                        if n
                        else "FAIL"
                        if baseline_count >= MIN_COVERAGE_BASELINE_SAMPLES
                        else "BELOW_BASELINE_FLOOR"
                        if baseline_count
                        else "UNOBSERVED",
                    }
                )
    indices = np.asarray(indices, dtype=int)
    baseline_samples = sum(g["baseline_planned_count"] for g in groups)
    lost_bins = [g["bin"] for g in groups if g["coverage_status"] == "FAIL"]
    return {
        "uv": uv[indices],
        "truth_base_m": truth[indices],
        "bins": bins[indices],
        "groups": groups,
        "selection_K": asdict(k),
        "margin": margin,
        "panel_region": {
            "measurement_vertices_base_m": np.asarray(region).tolist(),
            "panel_vertices_base_m": vertices.tolist(),
            "selection_confined_to_measurement_region": measurement_vertices_base
            is not None,
        },
        "coverage": {
            "status": "FAIL"
            if lost_bins
            else "COMPLETE"
            if len(indices)
            else "UNOBSERVED",
            "baseline_sample_count": baseline_samples,
            "sample_count": len(indices),
            "removed_sample_count": baseline_samples - len(indices),
            "lost_bins": lost_bins,
        },
        "location_source": "composed surface, actual USD mount, independently rendered fitted K",
    }


def height_grid_statistics(
    selection: dict,
    depth: np.ndarray,
    interior_mask: np.ndarray,
    nominal: PinholeIntrinsics,
    nominal_mount: np.ndarray,
    truth_height_m: float,
) -> dict:
    """Apply the detection grid convention to a selection fixed before capture."""
    uv = selection["uv"]
    u, v = uv.T
    samples = np.asarray(depth)[v, u]
    valid = np.isfinite(samples) & (samples > 0) & interior_mask[v, u]
    restored = backproject(uv, np.where(valid, samples, np.nan), nominal, nominal_mount)
    errors = restored[:, 2] - truth_height_m
    errors[~valid] = np.nan
    groups = []
    sampling_complete = []
    for group in selection["groups"]:
        selected = (selection["bins"] == group["bin"]).all(axis=1)
        selected_errors = errors[selected]
        measured = error_metrics(
            selected_errors[np.isfinite(selected_errors)], kind="height"
        )
        measured.update(group)
        measured["missing_count"] = int((~np.isfinite(selected_errors)).sum())
        if group["required_count"]:
            complete = (
                group["planned_count"] == group["required_count"]
                and measured["missing_count"] == 0
            )
            sampling_complete.append(complete)
            measured["sampling_status"] = "COMPLETE" if complete else "FAIL"
        # Row ⑦a′ gates by distance, unlike row ④ which gates every screen
        # cell. Keep these cell statistics for diagnosis without adding a
        # stricter numerical gate than the plan specifies.
        measured["role"] = "screen_diagnostic"
        groups.append(measured)
    per_distance = [
        dict(
            distance_bin=i,
            **error_metrics(
                errors[(selection["bins"][:, 0] == i) & valid], kind="height"
            ),
        )
        for i in range(6)
    ]
    lost_bins = [g["bin"] for g in groups if g.get("coverage_status") == "FAIL"]
    passed = bool(sampling_complete) and all(sampling_complete) and not lost_bins
    passed &= all(g["status"] in ("PASS", "UNOBSERVED") for g in per_distance)
    return {
        "status": ("PASS" if passed else "FAIL")
        if len(uv) or lost_bins
        else "UNOBSERVED",
        "coverage_status": "FAIL"
        if lost_bins
        else "COMPLETE"
        if len(uv)
        else "UNOBSERVED",
        "lost_bins": lost_bins,
        "sample_count": len(uv),
        "missing_count": int((~valid).sum()),
        "surface_height_base_m": truth_height_m,
        "groups": groups,
        "per_distance": per_distance,
    }


def marker_centroids(
    rgb: np.ndarray,
    mask: np.ndarray,
    expected_uv: np.ndarray,
    optical_z_m: float,
    calibration: PinholeIntrinsics,
) -> dict:
    """Compare the old global-brightness method against actual semantic identity.

    A sphere silhouette centroid is not its projected 3D centre. This comparison
    diagnoses the method; it is never a subpixel intrinsics measurement.
    """
    brightness = np.asarray(rgb)[..., :3].astype(float).mean(axis=2)
    bright, threshold = bright_pixel_mask(rgb)
    rows, cols = np.nonzero(mask)
    bright_rows, bright_cols = np.nonzero(bright)
    missing = []
    if not len(rows):
        missing.append("segmentation_target_mask_empty")
    if not len(bright_rows):
        missing.append("no_bright_pixels")
    if missing:
        raise ValueError("Marker identification failed: " + "; ".join(missing))
    semantic = np.array([cols.mean(), rows.mean()])
    global_bright = np.array([bright_cols.mean(), bright_rows.mean()])
    delta = global_bright - semantic
    outside = not (
        cols.min() <= global_bright[0] <= cols.max()
        and rows.min() <= global_bright[1] <= rows.max()
    )
    return {
        "semantic_centroid_uv": semantic.tolist(),
        "bright_centroid_uv": global_bright.tolist(),
        "projected_sphere_centre_uv": np.asarray(expected_uv).tolist(),
        "centroid_difference_px": delta.tolist(),
        "centroid_difference_m": (
            delta * optical_z_m / [calibration.fx, calibration.fy]
        ).tolist(),
        "semantic_minus_projected_centre_px": (semantic - expected_uv).tolist(),
        "bright_minus_projected_centre_px": (global_bright - expected_uv).tolist(),
        "semantic_count": len(rows),
        "bright_count": len(bright_rows),
        "bright_threshold": threshold,
        "bright_outside_marker_count": int((bright & ~mask).sum()),
        "bright_centroid_outside_marker_bbox": outside,
        "old_method_isolated": bool(np.std(brightness) > 2 and bright.mean() <= 0.05),
        "sphere_silhouette_is_not_projected_centre": True,
    }
