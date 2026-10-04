"""Rigid 2D scan-to-reference matching for destination docking (plan v3.8).

Point-to-line ICP: each live point is pulled onto the local line through its
nearest reference points (normals from the reference neighbourhood), solved
for (x, y, yaw) by Gauss-Newton on the linearised residuals. The result says
where the live scan's frame sits in the reference frame, and whether the match
can be used: enough inliers, a small median residual, and an information
matrix that fixes all three directions (a single wall leaves the along-wall
direction free and is reported degenerate). numpy only; the KD search is a
brute-force nearest neighbour, which is fast enough for two ~1,600-beam scans.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, sin

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


def transform_points(points, pose, *, inverse: bool = False) -> np.ndarray:
    """Points expressed in a frame at `pose` -> parent frame (or back)."""
    p = np.asarray(points, dtype=float)
    x, y, yaw = (float(v) for v in pose)
    c, s = cos(yaw), sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    if inverse:
        return (p - [x, y]) @ rotation
    return p @ rotation.T + [x, y]


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
) -> MatchResult:
    """Pose of the live scan's frame in the reference frame.

    min_information is the smallest eigenvalue of the point-to-line normal
    matrix sum(J^T J) with J = [n_x, n_y, n x p] per inlier (yaw scaled per
    radian at the point's lever arm): below it some direction is unobserved.
    """
    reference = np.asarray(reference, dtype=float)
    live = np.asarray(live, dtype=float)
    normals = _normals(reference)
    pose = np.array(initial, dtype=float)
    used = 0
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
    if fraction < min_inlier_fraction:
        reason = "few_inliers"
    elif median > max_median_residual_m:
        reason = "large_residual"
    elif information < min_information:
        reason = "degenerate"
    else:
        reason = "ok"
    return MatchResult(reason == "ok", reason, tuple(float(v) for v in pose), fraction, median, information, used)


__all__ = ["MatchResult", "match_scans", "transform_points"]
