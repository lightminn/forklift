"""Rigid 2D scan-to-reference matching for destination docking (plan v3.8).

Point-to-line ICP: each live point is pulled onto the local line through its
nearest reference points (normals from the reference neighbourhood), solved
for (x, y, yaw) by Gauss-Newton on the linearised residuals. The result says
where the live scan's frame sits in the reference frame, and whether the match
can be used: enough inliers, a small median residual, and an information
matrix that fixes all three directions (a single wall leaves the along-wall
direction free and is reported degenerate). numpy only; the KD search is a
brute-force nearest neighbour, which is fast enough for two ~1,600-beam scans.

A run that never takes a small step can still be settled (2026-10-13): each
point uses the normal of its nearest reference sample, so a point equidistant
from two samples with different normals can switch sample every iteration and
leave the pose trading between two states for ever. When the last four poses
repeat with period two to rounding and each pair used the same correspondences,
the run is settled at the midpoint of the two states if they lie within
max_cycle_half_m / max_cycle_half_rad of it, and refused as wide_cycle if not.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, sin

import numpy as np


@dataclass(frozen=True)
class MatchResult:
    accepted: bool
    reason: str
    pose: tuple  # live frame in the reference frame (x, y, yaw)
    inlier_fraction: float
    median_residual_m: float
    min_information: float
    iterations: int
    cycle: tuple | None = None  # a period-two cycle seen: (half distance m, half turn rad, iteration)


def transform_points(points, pose, *, inverse: bool = False) -> np.ndarray:
    """Points expressed in a frame at `pose` -> parent frame (or back)."""
    p = np.asarray(points, dtype=float)
    x, y, yaw = (float(v) for v in pose)
    c, s = cos(yaw), sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    if inverse:
        return (p - [x, y]) @ rotation
    return p @ rotation.T + [x, y]


CYCLE_TOLERANCE = 1e-9  # m per component and rad: a repeat to rounding, not an approach


def _same_pose(a, b, tolerance: float) -> bool:
    turn = atan2(sin(a[2] - b[2]), cos(a[2] - b[2]))
    return abs(a[0] - b[0]) <= tolerance and abs(a[1] - b[1]) <= tolerance and abs(turn) <= tolerance


def _two_cycle(poses, pairs, tolerance: float = CYCLE_TOLERANCE) -> bool:
    """The last four iterations repeat with period two: poses k / k-2 and k-1 / k-3 within
    tolerance, and each pair used the same correspondences (nearest indices and in-range
    mask). The same pose and correspondences give the same next step, so the cycle goes on."""
    if len(poses) < 4 or len(pairs) < 4:
        return False
    return all(
        _same_pose(poses[-1 - i], poses[-3 - i], tolerance)
        and all(np.array_equal(a, b) for a, b in zip(pairs[-1 - i], pairs[-3 - i]))
        for i in (0, 1)
    )


def _midpoint(a, b) -> np.ndarray:
    """The pose halfway between two poses, the yaw across the seam."""
    yaw = b[2] + atan2(sin(a[2] - b[2]), cos(a[2] - b[2])) / 2
    return np.array([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, atan2(sin(yaw), cos(yaw))])


def _nearest(reference: np.ndarray, query: np.ndarray, chunk: int = 512):
    index = np.empty(len(query), dtype=int)
    distance = np.empty(len(query))
    for start in range(0, len(query), chunk):
        block = query[start : start + chunk]
        d2 = ((block[:, None, :] - reference[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        index[start : start + chunk] = j
        distance[start : start + chunk] = np.sqrt(d2[np.arange(len(block)), j])
    return index, distance


def _normals(reference: np.ndarray, k: int = 6) -> np.ndarray:
    normals = np.empty_like(reference)
    for start in range(0, len(reference), 512):
        block = reference[start : start + 512]
        d2 = ((block[:, None, :] - reference[None, :, :]) ** 2).sum(-1)
        nearest = np.argsort(d2, axis=1)[:, :k]
        for row, ids in enumerate(nearest):
            q = reference[ids] - reference[ids].mean(0)
            _, vectors = np.linalg.eigh(q.T @ q)
            normals[start + row] = vectors[:, 0]
    return normals


def match_scans(
    reference,
    live,
    initial=(0.0, 0.0, 0.0),
    *,
    max_iterations: int = 40,
    max_correspondence_m: float = 0.3,
    inlier_m: float = 0.05,
    min_inlier_fraction: float = 0.6,
    max_median_residual_m: float = 0.03,
    min_information: float = 50.0,
    max_cycle_half_m: float = 1e-3,
    max_cycle_half_rad: float = 1e-4,
) -> MatchResult:
    """Pose of the live scan's frame in the reference frame.

    min_information is the smallest eigenvalue of the point-to-line normal
    matrix sum(J^T J) with J = [n_x, n_y, n x p] per inlier (yaw scaled per
    radian at the point's lever arm): below it some direction is unobserved.
    max_cycle_half_m / max_cycle_half_rad bound a settled period-two cycle (module
    docstring): half the distance and half the turn between its two states.
    """
    if max_iterations < 1:
        raise ValueError("max_iterations must be at least 1")
    reference = np.asarray(reference, dtype=float).reshape(-1, 2)
    live = np.asarray(live, dtype=float).reshape(-1, 2)
    pose = np.array(initial, dtype=float)
    if len(reference) < 10 or len(live) < 10:
        # Nothing to match against (Codex v3.8 impl P2): refuse, do not crash.
        return MatchResult(False, "too_few_points", tuple(pose), 0.0, float("inf"), 0.0, 0)
    normals = _normals(reference)
    used = 0
    converged = False
    cycle = None
    poses, pairs = [], []
    for used in range(1, max_iterations + 1):
        moved = transform_points(live, pose)
        j, d = _nearest(reference, moved)
        keep = d < max_correspondence_m
        if keep.sum() < 10:
            return MatchResult(False, "no_overlap", tuple(pose), 0.0, float("inf"), 0.0, used)
        p, q, n = moved[keep], reference[j[keep]], normals[j[keep]]
        residual = ((p - q) * n).sum(1)
        lever = p - pose[:2]
        jac = np.column_stack((n[:, 0], n[:, 1], n[:, 0] * -lever[:, 1] + n[:, 1] * lever[:, 0]))
        step, *_ = np.linalg.lstsq(jac, -residual, rcond=None)
        pose[:2] += step[:2]
        pose[2] = atan2(sin(pose[2] + step[2]), cos(pose[2] + step[2]))
        if np.abs(step[:2]).max() < 1e-5 and abs(step[2]) < 1e-6:
            converged = True
            break
        poses.append(pose.copy())
        pairs.append((j, keep))
        if _two_cycle(poses, pairs):
            a, b = poses[-1], poses[-2]
            cycle = (hypot(a[0] - b[0], a[1] - b[1]) / 2, abs(atan2(sin(a[2] - b[2]), cos(a[2] - b[2]))) / 2, used)
            if cycle[0] <= max_cycle_half_m and cycle[1] <= max_cycle_half_rad:
                pose = _midpoint(a, b)
                converged = True
            break
    moved = transform_points(live, pose)
    j, d = _nearest(reference, moved)
    inliers = d < inlier_m
    fraction = float(inliers.mean())
    point_line = np.abs(((moved - reference[j]) * normals[j]).sum(1))
    median = float(np.median(point_line[inliers])) if inliers.any() else float("inf")
    n = normals[j[inliers]]
    lever = moved[inliers] - pose[:2]
    jac = np.column_stack((n[:, 0], n[:, 1], n[:, 0] * -lever[:, 1] + n[:, 1] * lever[:, 0]))
    information = float(np.linalg.eigvalsh(jac.T @ jac)[0]) if inliers.sum() >= 3 else 0.0
    if not converged:
        reason = "not_converged" if cycle is None else "wide_cycle"
    elif fraction < min_inlier_fraction:
        reason = "few_inliers"
    elif median > max_median_residual_m:
        reason = "large_residual"
    elif information < min_information:
        reason = "degenerate"
    else:
        reason = "ok"
    return MatchResult(
        reason == "ok", reason, tuple(float(v) for v in pose), fraction, median, information, used,
        None if cycle is None else (float(cycle[0]), float(cycle[1]), int(cycle[2])),
    )




# ---- sim-only weak-direction acceptance (docs/plans/2026-10-22-open-zone-docking.md D2) ----
#
# A delivery match refused only because one direction is weakly observed (a single far wall with a few sideways faces)
# is taken when these hold at the agreed pose. They are local checks, not proofs: faces moved together after teaching
# and a repeated pattern farther away than the range check are taken wrongly (the plan's documented risk). Not for the
# real vehicle.

PRECISION_SIGMA_M = 0.005
PRECISION_SIGMA_RAD = 0.0005
PRECISION_CARRIER_SHARE = 0.2
PRECISION_GROUP_LINK_M = 1.0
PRECISION_GROUP_MIN_POINTS = 3
PRECISION_FACET_LINK_M = 0.10
PRECISION_FACET_COS = cos(0.5236)  # 30 deg
PRECISION_DROP_MIN_INFORMATION = 5.0
PRECISION_DROP_SHIFT_M = 0.01
PRECISION_DROP_SHIFT_RAD = 0.0005
PRECISION_RANGE_M = 0.20
PRECISION_RANGE_STEP_M = 0.05
PRECISION_AGREE_M = 0.01
PRECISION_AGREE_RAD = 0.005
PRECISION_BOUND_M = 0.20


@dataclass(frozen=True)
class PrecisionCheck:
    accepted: bool
    reason: str  # "ok" or why not (sd, one_group, group_shift, not_unique, bound, refine_<reason>)
    pose: tuple  # the re-solved pose (live frame in the reference frame)
    record: dict


def _information_rows(reference, normals, live, pose, inlier_m):
    moved = transform_points(live, pose)
    j, d = _nearest(reference, moved)
    inl = d < inlier_m
    n, p = normals[j[inl]], moved[inl]
    residual = ((p - reference[j[inl]]) * n).sum(1)
    lever = p - np.asarray(pose[:2])
    # u = (dx, dy, dyaw): the yaw column is per radian at each point's lever arm, i.e. a 1 m lever about the origin
    jac = np.column_stack((n[:, 0], n[:, 1], n[:, 0] * -lever[:, 1] + n[:, 1] * lever[:, 0]))
    return jac, residual, inl, j, p


def _sigma(jac, residual):
    scale = 1.4826 * float(np.median(np.abs(residual - np.median(residual))))
    h = jac.T @ jac
    try:
        cov = scale * scale * np.linalg.inv(h)
    except np.linalg.LinAlgError:
        return scale, (float("inf"),) * 3
    return scale, tuple(float(v) for v in np.sqrt(np.clip(np.diag(cov), 0.0, None)))


def _link_groups(points, link):
    label = -np.ones(len(points), int)
    current = 0
    for i in range(len(points)):
        if label[i] >= 0:
            continue
        label[i] = current
        stack = [i]
        while stack:
            k = stack.pop()
            near = np.where((label < 0) & (np.hypot(*(points - points[k]).T) < link))[0]
            label[near] = current
            stack.extend(near.tolist())
        current += 1
    return label


def _facets(reference, normals, link, min_cos):
    """Reference faces: points linked within `link` whose normals agree to `min_cos` (a wall's side face is not
    the wall)."""
    label = -np.ones(len(reference), int)
    current = 0
    for i in range(len(reference)):
        if label[i] >= 0:
            continue
        label[i] = current
        stack = [i]
        while stack:
            k = stack.pop()
            near = np.where((label < 0) & (np.hypot(*(reference - reference[k]).T) < link)
                            & (np.abs(normals @ normals[k]) >= min_cos))[0]
            label[near] = current
            stack.extend(near.tolist())
        current += 1
    return label


def _close_pose(a, b, metres, radians):
    turn = atan2(sin(a[2] - b[2]), cos(a[2] - b[2]))
    return hypot(a[0] - b[0], a[1] - b[1]) <= metres and abs(turn) <= radians


def precision_check(reference, live, pose, prior, *, inlier_m: float = 0.05, max_correspondence_m: float = 0.3,
                    min_inlier_fraction: float = 0.6, max_median_residual_m: float = 0.03) -> PrecisionCheck:
    """The D2 checks at `pose` (an agreed start pose), `prior` = the initial laser pose of the match (the plan's)."""
    reference = np.asarray(reference, dtype=float).reshape(-1, 2)
    live = np.asarray(live, dtype=float).reshape(-1, 2)
    record: dict = {}
    gates = dict(inlier_m=inlier_m, min_inlier_fraction=min_inlier_fraction,
                 max_median_residual_m=max_median_residual_m, min_information=0.0)
    # (1) the final solve on the inlier set the gates use; its gates judged again
    refined = match_scans(reference, live, initial=pose, max_correspondence_m=inlier_m, **gates)
    record["refined_reason"] = refined.reason
    if refined.reason != "ok":
        return PrecisionCheck(False, f"refine_{refined.reason}", tuple(pose), record)
    p1 = refined.pose
    normals = _normals(reference)
    jac, residual, inl, nearest, points = _information_rows(reference, normals, live, p1, inlier_m)
    # (2), (3) the weak direction and the conditional precision
    h = jac.T @ jac
    w, v = np.linalg.eigh(h)
    weak = v[:, 0]
    scale, sigma = _sigma(jac, residual)
    _, sigma_thin = _sigma(jac[::4], residual[::4])
    record.update(min_information=float(w[0]), weak=[float(c) for c in weak], scale_m=scale, sigma_m=list(sigma),
                  sigma_decimated_m=list(sigma_thin))
    if sigma[0] > PRECISION_SIGMA_M or sigma[1] > PRECISION_SIGMA_M or sigma[2] > PRECISION_SIGMA_RAD:
        return PrecisionCheck(False, "sd", p1, record)
    # (4) the weak direction carried by two groups at least; dropping each group's reference faces keeps the pose
    share = (jac @ weak) ** 2
    carriers = np.where(share >= PRECISION_CARRIER_SHARE * share.max())[0]
    labels = _link_groups(points[carriers], PRECISION_GROUP_LINK_M)
    groups = [carriers[labels == k] for k in range(labels.max() + 1) if (labels == k).sum() >= PRECISION_GROUP_MIN_POINTS]
    record["groups"] = len(groups)
    record["group_sizes"] = [int(len(g)) for g in groups]
    if len(groups) < 2:
        return PrecisionCheck(False, "one_group", p1, record)
    facets = _facets(reference, normals, PRECISION_FACET_LINK_M, PRECISION_FACET_COS)
    live_nearest, live_nearest_d = _nearest(reference, transform_points(live, p1))
    drops = []
    for g in groups:
        faces = np.unique(facets[nearest[g]])
        keep_ref = ~np.isin(facets, faces)
        keep_live = ~(np.isin(facets[live_nearest], faces) & (live_nearest_d < max_correspondence_m))
        again = match_scans(reference[keep_ref], live[keep_live], initial=p1, **gates)
        moved = hypot(again.pose[0] - p1[0], again.pose[1] - p1[1])
        turn = abs(atan2(sin(again.pose[2] - p1[2]), cos(again.pose[2] - p1[2])))
        drops.append({"faces": int(len(faces)), "reason": again.reason, "min_information": again.min_information,
                      "shift_m": moved, "turn_rad": turn})
        if (again.reason != "ok" or again.min_information < PRECISION_DROP_MIN_INFORMATION
                or moved > PRECISION_DROP_SHIFT_M or turn > PRECISION_DROP_SHIFT_RAD):
            record["drops"] = drops
            return PrecisionCheck(False, "group_shift", p1, record)
    record["drops"] = drops
    # (5) every start along the weak direction within the range ends at the same pose
    agree = True
    ends = []
    k = int(round(PRECISION_RANGE_M / PRECISION_RANGE_STEP_M))
    for step in range(-k, k + 1):
        start = (p1[0] + step * PRECISION_RANGE_STEP_M * weak[0], p1[1] + step * PRECISION_RANGE_STEP_M * weak[1],
                 p1[2] + step * PRECISION_RANGE_STEP_M * weak[2])
        end = match_scans(reference, live, initial=start, max_correspondence_m=max_correspondence_m, **gates)
        ends.append([float(c) for c in end.pose])
        if not _close_pose(end.pose, p1, PRECISION_AGREE_M, PRECISION_AGREE_RAD):
            agree = False
    record["range_agree"] = agree
    record["range_ends"] = ends
    if not agree:
        return PrecisionCheck(False, "not_unique", p1, record)
    # (6) this round's correction along the weak direction
    du = (p1[0] - prior[0], p1[1] - prior[1], atan2(sin(p1[2] - prior[2]), cos(p1[2] - prior[2])))
    along = abs(float(np.dot(weak, du)))
    record["weak_correction_m"] = along
    if along > PRECISION_BOUND_M:
        return PrecisionCheck(False, "bound", p1, record)
    return PrecisionCheck(True, "ok", p1, record)


__all__ = ["MatchResult", "PrecisionCheck", "match_scans", "precision_check", "transform_points"]
