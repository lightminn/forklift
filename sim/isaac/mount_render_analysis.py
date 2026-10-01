"""SDK-free rules of the carriage-camera render check.

Plan: docs/plans/2026-10-01-carriage-camera-render-check.md. Everything that
decides a number in that run lives here so it can be tested without Isaac:
the face-gap grid, the region masks built from three renders, the pixel rule
for fork-tip visibility, the frame-freshness check against the depth the
current pose should produce, the depth preparation the detector sees, the
per-region depth-difference statistics and the rule that keeps a distance
boundary from being claimed across an unconfirmed cell.

Depth here is optical-axis z in metres. A pixel is valid only when finite and
positive; the scene's back wall takes every ray, so a miss is a defect.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

INVALID, OTHER, CARRIAGE, PALLET = -1, 0, 1, 2
REGION_NAMES = {OTHER: "other", CARRIAGE: "carriage", PALLET: "pallet"}
MASK_TOLERANCE_M = 0.002  # nearer by more than this when the part is shown
TIP_DEPTH_TOLERANCE_M = 0.005
FRESHNESS_TOLERANCE_M = 0.003
TRANSITION_MIN_CHANGE_M = 0.006  # > 2 x FRESHNESS_TOLERANCE_M
EDGE_CURVATURE_INV_M = 1e-4  # 1/m; second difference of inverse depth off a plane
REFERENCE_PIXEL_LIMIT = 64
MIN_RANGE_M = 0.175
QUANTIZE_STEP_M = 0.001


def face_gap_grid(start_m: float = -0.20, stop_m: float = 0.70, step_m: float = 0.01):
    """The pre-registered grid, both ends included (91 cells by default)."""
    count = int(round((stop_m - start_m) / step_m)) + 1
    return [round(start_m + step_m * k, 6) for k in range(count)]


def valid_depth(depth: np.ndarray) -> np.ndarray:
    depth = np.asarray(depth, dtype=float)
    with np.errstate(invalid="ignore"):
        return np.isfinite(depth) & (depth > 0)


def region_labels(
    full: np.ndarray,
    no_carriage: np.ndarray,
    no_pallet: np.ndarray,
    *,
    tolerance_m: float = MASK_TOLERANCE_M,
) -> np.ndarray:
    """Per-pixel region from three raw renders of one pose.

    Carriage where the full render is nearer than the carriage-hidden one by
    more than ``tolerance_m``; pallet likewise against the pallet-hidden one;
    everything else is floor or wall. A pixel invalid in any of the three
    renders is INVALID -- a mask needs both sides of its comparison.
    """
    full = np.asarray(full, dtype=float)
    ok = valid_depth(full) & valid_depth(no_carriage) & valid_depth(no_pallet)
    labels = np.full(full.shape, OTHER, dtype=np.int8)
    with np.errstate(invalid="ignore"):
        pallet = full < np.asarray(no_pallet, dtype=float) - tolerance_m
        carriage = full < np.asarray(no_carriage, dtype=float) - tolerance_m
    labels[pallet] = PALLET
    # A carriage pixel in front of the pallet does not change when the pallet hides.
    labels[carriage] = CARRIAGE
    labels[~ok] = INVALID
    return labels


def boundary_mask(labels: np.ndarray) -> np.ndarray:
    """Pixels with a 4-neighbour of a different label (INVALID counts as one)."""
    labels = np.asarray(labels)
    edge = np.zeros(labels.shape, dtype=bool)
    vertical = labels[1:, :] != labels[:-1, :]
    horizontal = labels[:, 1:] != labels[:, :-1]
    edge[1:, :] |= vertical
    edge[:-1, :] |= vertical
    edge[:, 1:] |= horizontal
    edge[:, :-1] |= horizontal
    return edge


def depth_edges(
    depth: np.ndarray, *, curvature_inv_m: float = EDGE_CURVATURE_INV_M
) -> np.ndarray:
    """Pixels off a locally planar surface: silhouettes and creases.

    On a plane, inverse optical depth is linear in pixel coordinates, so its
    second difference along a row or a column is zero however steeply the
    depth itself grows (a far floor gains centimetres per pixel). A pixel is
    an edge when that second difference exceeds ``curvature_inv_m`` along
    either axis, when it lies on the image border, or when it or a neighbour
    is invalid.
    """
    depth = np.asarray(depth, dtype=float)
    valid = valid_depth(depth)
    inverse = np.where(valid, 1.0 / np.where(valid, depth, 1.0), np.nan)
    edge = np.ones(depth.shape, dtype=bool)
    with np.errstate(invalid="ignore"):
        rows = np.abs(inverse[2:, :] - 2 * inverse[1:-1, :] + inverse[:-2, :])
        cols = np.abs(inverse[:, 2:] - 2 * inverse[:, 1:-1] + inverse[:, :-2])
    calm = np.zeros(depth.shape, dtype=bool)
    calm[1:-1, 1:-1] = (rows[:, 1:-1] <= curvature_inv_m) & (
        cols[1:-1, :] <= curvature_inv_m
    )
    edge &= ~calm
    return edge


def project_nearest(points_optical: np.ndarray, intrinsics) -> np.ndarray:
    """Nearest pixel (row, col) of optical points, integer-index centres; -1 if behind."""
    points = np.asarray(points_optical, dtype=float).reshape(-1, 3)
    out = np.full((len(points), 2), -1, dtype=int)
    front = points[:, 2] > 0
    u = intrinsics.fx * points[front, 0] / points[front, 2] + intrinsics.cx
    v = intrinsics.fy * points[front, 1] / points[front, 2] + intrinsics.cy
    out[front, 0] = np.floor(v + 0.5).astype(int)
    out[front, 1] = np.floor(u + 0.5).astype(int)
    return out


def pixel_tip_visibility(
    depth_full: np.ndarray,
    labels: np.ndarray,
    boundary: np.ndarray,
    samples_base: np.ndarray,
    base_from_optical,
    intrinsics,
    *,
    min_range_m: float = MIN_RANGE_M,
    tolerance_m: float = TIP_DEPTH_TOLERANCE_M,
) -> dict:
    """The plan's pixel rule for one blade's tip samples.

    A sample is seen when its nearest pixel lies in the image, is a carriage
    pixel, and that pixel's full-render depth is within ``tolerance_m`` of the
    sample's optical depth, which is at least ``min_range_m``. Samples landing
    on a boundary pixel are counted separately and stay in the denominator;
    samples landing on an INVALID pixel make the result unconfirmed.
    """
    rotation = np.asarray(base_from_optical.rotation, dtype=float)
    origin = np.asarray(base_from_optical.translation_m, dtype=float)
    samples = np.asarray(samples_base, dtype=float)
    optical = (samples - origin) @ rotation
    pixels = project_nearest(optical, intrinsics)
    height, width = labels.shape
    seen = on_boundary = on_invalid = 0
    for point, (row, col) in zip(optical, pixels, strict=True):
        if not (0 <= row < height and 0 <= col < width) or point[2] < min_range_m:
            continue
        if labels[row, col] == INVALID:
            on_invalid += 1
            continue
        if boundary[row, col]:
            on_boundary += 1
        if (
            labels[row, col] == CARRIAGE
            and abs(float(depth_full[row, col]) - point[2]) <= tolerance_m
        ):
            seen += 1
    total = len(samples)
    return {
        "seen": seen,
        "total": total,
        "fraction": seen / total,
        "on_boundary": on_boundary,
        "on_invalid": on_invalid,
        "confirmed": on_invalid == 0,
    }


def _spread(indices: np.ndarray, limit: int) -> np.ndarray:
    if len(indices) <= limit:
        return indices
    keep = np.linspace(0, len(indices) - 1, limit).round().astype(int)
    return indices[keep]


def reference_pixels(
    expected_now: np.ndarray,
    expected_before: np.ndarray | None,
    *,
    min_change_m: float = TRANSITION_MIN_CHANGE_M,
    limit: int = REFERENCE_PIXEL_LIMIT,
) -> dict[str, np.ndarray]:
    """Pixels a fresh frame must reproduce, chosen from the expected depth alone.

    ``changed``: away from any depth edge in either expectation, and expected
    to move by more than ``min_change_m`` from the previous render -- these
    are what a stale frame cannot pass. ``static``: away from edges and not
    expected to move -- these check the renderer agrees at all. Both are
    evenly spread over the qualifying pixels, at most ``limit`` each.
    ``expected_changes`` counts every pixel, edge or not, whose expected depth
    changes at all (or changes validity) -- zero means the expected image is
    identical, which is a separate question from which pixels can be checked.
    """
    now = np.asarray(expected_now, dtype=float)
    calm = ~depth_edges(now)
    if expected_before is None:
        changed = np.zeros(now.shape, dtype=bool)
        static = calm
        moved = 0
    else:
        before = np.asarray(expected_before, dtype=float)
        calm &= ~depth_edges(before)
        with np.errstate(invalid="ignore"):
            delta = np.abs(now - before)
        changed = calm & (delta > min_change_m)
        static = calm & (delta <= 1e-9)
        moved = int(
            np.count_nonzero(
                (valid_depth(now) != valid_depth(before)) | (delta > 1e-9)
            )
        )
    return {
        "changed": _spread(np.argwhere(changed), limit),
        "static": _spread(np.argwhere(static), limit),
        "expected_changes": moved,
    }


def freshness(
    depth: np.ndarray,
    expected: np.ndarray,
    pixels: dict[str, np.ndarray],
    *,
    needs_change: bool,
    tolerance_m: float = FRESHNESS_TOLERANCE_M,
) -> dict:
    """Whether a render matches the depth its current pose should produce.

    Fails when any reference pixel is invalid or off by more than
    ``tolerance_m``. A transition whose expected image changes but has no
    changed pixel to check is ``unverifiable``, not passed. One whose expected
    image does not change at all (a part hidden behind another) is checked by
    its static pixels alone: a stale frame would equal the expected one, so
    nothing that depends on the render can differ.
    """
    report = {
        "ok": True,
        "unverifiable": False,
        "expected_changes": pixels.get("expected_changes"),
    }
    for kind in ("changed", "static"):
        chosen = pixels.get(kind, np.zeros((0, 2), dtype=int))
        if len(chosen) == 0:
            report[kind] = {"count": 0, "max_error_m": None}
            continue
        rows, cols = chosen[:, 0], chosen[:, 1]
        got = np.asarray(depth, dtype=float)[rows, cols]
        want = np.asarray(expected, dtype=float)[rows, cols]
        bad = ~valid_depth(got)
        error = np.where(bad, np.inf, np.abs(got - want))
        report[kind] = {
            "count": int(len(chosen)),
            "max_error_m": float(np.max(error)),
            "over_tolerance": int(np.count_nonzero(error > tolerance_m)),
        }
        if np.any(error > tolerance_m):
            report["ok"] = False
    if (
        needs_change
        and len(pixels.get("changed", ())) == 0
        and pixels.get("expected_changes", 1) > 0
    ):
        report["unverifiable"] = True
    if len(pixels.get("static", ())) == 0 and len(pixels.get("changed", ())) == 0:
        report["unverifiable"] = True
    return report


def detector_depth(
    raw: np.ndarray, *, quantize: bool, min_range_m: float = MIN_RANGE_M
) -> np.ndarray:
    """The CPU rig's order: invalid to NaN, round to 1 mm, then the min-range mask."""
    depth = np.where(valid_depth(raw), np.asarray(raw, dtype=float), np.nan)
    if quantize:
        depth = np.round(depth / QUANTIZE_STEP_M) * QUANTIZE_STEP_M
    with np.errstate(invalid="ignore"):
        return np.where(depth < min_range_m, np.nan, depth)


def difference_stats(
    isaac: np.ndarray, cpu: np.ndarray, labels: np.ndarray, boundary: np.ndarray
) -> dict:
    """Isaac - CPU depth per region, over pixels valid in all three Isaac renders
    and in the CPU render, with silhouette pixels excluded and included."""
    diff = np.asarray(isaac, dtype=float) - np.asarray(cpu, dtype=float)
    base = (labels != INVALID) & valid_depth(isaac) & valid_depth(cpu)
    out = {}
    for region, name in REGION_NAMES.items():
        for scope, mask in (
            ("interior", base & (labels == region) & ~boundary),
            ("all", base & (labels == region)),
        ):
            values = diff[mask]
            if values.size == 0:
                out[f"{name}_{scope}"] = {"pixels": 0}
                continue
            magnitude = np.abs(values)
            out[f"{name}_{scope}"] = {
                "pixels": int(values.size),
                "median_m": float(np.median(values)),
                "median_abs_m": float(np.median(magnitude)),
                "p95_abs_m": float(np.percentile(magnitude, 95)),
                "max_abs_m": float(np.max(magnitude)),
            }
    return out


def nearest_claim(
    cells: Sequence[tuple[float, bool | None]],
    *,
    grid: Sequence[float] | None = None,
    excluded: Sequence[float] = (),
) -> dict:
    """The nearest face gap where the predicate holds, and whether it may be claimed.

    ``cells`` are (face_gap, value) with None for an unconfirmed cell. The
    nearest True cell is ``claimed`` only when every nearer cell is
    confirmed; otherwise ``unconfirmed_up_to`` names the farthest unconfirmed
    gap nearer than it, and the boundary reads "unconfirmed up to there".
    With ``grid``, a grid gap absent from ``cells`` and not in ``excluded``
    is unconfirmed, so a partial run cannot claim a boundary.
    """
    cells = list(cells)
    if grid is not None:
        present = {round(g, 6) for g, _ in cells} | {round(g, 6) for g in excluded}
        cells += [(g, None) for g in grid if round(g, 6) not in present]
    unconfirmed = None
    for gap, value in sorted(cells, key=lambda c: c[0]):
        if value is None:
            unconfirmed = gap
        elif value:
            return {
                "first_true": gap,
                "claimed": gap if unconfirmed is None else None,
                "unconfirmed_up_to": unconfirmed,
            }
    return {"first_true": None, "claimed": None, "unconfirmed_up_to": unconfirmed}


def runs(cells: Sequence[tuple[float, bool | None]]) -> str:
    """Compact cell states nearest first: '+' holds, '.' does not, '?' unconfirmed."""
    symbols = {True: "+", False: ".", None: "?"}
    return "".join(symbols[value] for _, value in sorted(cells, key=lambda c: c[0]))


def pallet_pose_from_matrix(base_from_pallet: np.ndarray) -> tuple[float, float, float]:
    """(x, y, yaw) of a pallet transform that must be upright on the floor."""
    matrix = np.asarray(base_from_pallet, dtype=float)
    rotation, translation = matrix[:3, :3], matrix[:3, 3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError("pallet transform is not rigid")
    if abs(rotation[2, 2] - 1.0) > 1e-9 or abs(translation[2]) > 1e-6:
        raise ValueError("pallet is not upright on the floor")
    return (
        float(translation[0]),
        float(translation[1]),
        math.atan2(rotation[1, 0], rotation[0, 0]),
    )


def boxes_match(
    got: Sequence[tuple[Sequence[float], Sequence[float]]],
    want: Sequence[tuple[Sequence[float], Sequence[float]]],
    *,
    tolerance_m: float = 1e-5,
) -> bool:
    """Whether two sets of (centre, size) boxes are the same, order ignored."""
    if len(got) != len(want):
        return False

    def key(box):
        return tuple(round(v, 4) for v in (*box[0], *box[1]))

    for a, b in zip(sorted(got, key=key), sorted(want, key=key), strict=True):
        if not (
            np.allclose(a[0], b[0], atol=tolerance_m)
            and np.allclose(a[1], b[1], atol=tolerance_m)
        ):
            return False
    return True
