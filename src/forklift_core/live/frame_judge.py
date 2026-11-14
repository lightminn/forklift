"""Judging saved live-console frames for the overlay lines, and the overlay's offline
pixel shift between two camera poses (docs/plans/2026-11-14-path-display-perception-leak.md
4, S4).

Pure NumPy, no image library: the caller decodes the frames. Nothing here decides whether
an arm passes; the S4 registration sets those thresholds, and these functions only count.

- ``colour_mask``: the pixels within a per-channel absolute tolerance of one colour.
- ``label_components``: 4-connected components of a mask (run-length labelling).
- ``frame_summary`` / ``arm_summary``: the counts the judge reports, per frame and per arm.
- ``pixel_shift``: how far a world point's pixel moves between two camera poses, for the
  misalignment the overlay gets when the frame's pixels trail its render time.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence

import numpy as np

NEAR_M = 0.05  # as sim/isaac/path_overlay.NEAR_M: no point this close or behind is drawn


def colour_mask(rgb: np.ndarray, colour: Sequence[int], tol: int) -> np.ndarray:
    """Pixels of ``rgb`` within ``tol`` of ``colour`` in every channel.

    ``rgb``: (H, W, 3) or (H, W, 4) array of 0-255 values (the fourth channel is ignored).
    ``colour``: three 0-255 values. ``tol``: the largest absolute difference allowed per
    channel, >= 0. Returns an (H, W) bool array.
    """
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] not in (3, 4):
        raise ValueError(f"rgb must be (H, W, 3|4), not {rgb.shape}")
    if tol < 0:
        raise ValueError(f"tol must be >= 0, not {tol}")
    want = np.asarray(colour, dtype=np.int16).reshape(-1)
    if want.shape != (3,):
        raise ValueError(f"colour must have three channels, not {colour!r}")
    diff = np.abs(rgb[:, :, :3].astype(np.int16) - want)
    return np.all(diff <= int(tol), axis=2)


def _runs(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Horizontal runs of True in raster order: row, start column, end column (exclusive)."""
    h, w = mask.shape
    padded = np.zeros((h, w + 2), dtype=np.int8)
    padded[:, 1:-1] = mask
    step = np.diff(padded, axis=1)
    rows, starts = np.nonzero(step == 1)
    _, ends = np.nonzero(step == -1)  # row-major, so the n-th end closes the n-th start
    return rows, starts, ends


def _run_edges(rows, starts, ends, width: int) -> tuple[np.ndarray, np.ndarray]:
    """Pairs of runs in consecutive rows that share a column (4-connectivity)."""
    stride = width + 1  # an end may equal ``width``; the key stays below the next row's
    start_key = rows * stride + starts
    end_key = rows * stride + ends
    below = (rows + 1) * stride
    # the runs below run i that overlap it: end > start_i and start < end_i, contiguous
    lo = np.searchsorted(end_key, below + starts, side="right")
    hi = np.searchsorted(start_key, below + ends, side="left")
    counts = np.maximum(hi - lo, 0)
    total = int(counts.sum())
    if total == 0:
        return np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.int64)
    a = np.repeat(np.arange(len(rows)), counts)
    first = np.repeat(np.cumsum(counts) - counts, counts)
    b = np.repeat(lo, counts) + (np.arange(total) - first)
    return a, b


def _union(n: int, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Each node's component root, the smallest node index in its component."""
    lab = np.arange(n)
    if a.size == 0:
        return lab
    while True:
        before = lab.copy()
        la, lb = lab[a], lab[b]
        low = np.minimum(la, lb)
        # hook both roots and both ends to the smaller label; labels only decrease and
        # never leave the component, so this ends with one label per component
        np.minimum.at(lab, la, low)
        np.minimum.at(lab, lb, low)
        np.minimum.at(lab, a, low)
        np.minimum.at(lab, b, low)
        while True:
            jumped = lab[lab]
            if np.array_equal(jumped, lab):
                break
            lab = jumped
        if np.array_equal(lab, before):
            return lab


def label_components(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """4-connected components of a 2-D bool mask.

    Returns ``(labels, sizes)``: ``labels`` is an (H, W) int32 array, 0 off the mask and
    1..n on it, numbered in raster order of each component's first pixel; ``sizes`` is an
    (n,) int64 array of pixel counts, ``sizes[k - 1]`` for label k. Diagonal neighbours are
    not connected.
    """
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2:
        raise ValueError(f"mask must be 2-D, not {mask.shape}")
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    rows, starts, ends = _runs(mask)
    if rows.size == 0:
        return labels, np.zeros(0, dtype=np.int64)
    a, b = _run_edges(rows, starts, ends, w)
    roots = _union(len(rows), a, b)
    _, component = np.unique(roots, return_inverse=True)  # roots ascend in raster order
    component = component.reshape(-1)
    lengths = ends - starts
    sizes = np.bincount(component, weights=lengths).astype(np.int64)
    run_of_pixel = np.repeat(np.arange(len(rows)), lengths)
    first = np.repeat(np.cumsum(lengths) - lengths, lengths)
    flat = (rows * w + starts)[run_of_pixel] + (np.arange(int(lengths.sum())) - first)
    labels.reshape(-1)[flat] = component[run_of_pixel] + 1
    return labels, sizes


def frame_summary(rgb: np.ndarray, colours: Mapping[str, Sequence[int]], tol: int,
                  min_component: int) -> dict[str, dict[str, int]]:
    """Per colour name: ``pixels`` (pixels within ``tol``), ``components`` (4-connected
    components of at least ``min_component`` pixels) and ``max_component`` (the largest
    component, whatever its size; 0 without one)."""
    out = {}
    for name, colour in colours.items():
        mask = colour_mask(rgb, colour, tol)
        _, sizes = label_components(mask)
        out[name] = {"pixels": int(mask.sum()),
                     "components": int((sizes >= min_component).sum()),
                     "max_component": int(sizes.max()) if sizes.size else 0}
    return out


def arm_summary(frames: Iterable[Mapping[str, Mapping[str, int]] | None],
                names: Sequence[str]) -> dict:
    """One arm's counts over its frames.

    ``frames``: per-frame ``frame_summary`` results, None for a frame that could not be
    read. Per colour: ``frames_with_component`` (frames with at least one component of the
    minimum size), ``fraction`` (of the readable frames; None without one),
    ``frames_with_pixels`` and ``max_component`` (over every frame).
    """
    frames = list(frames)
    judged = [f for f in frames if f is not None]
    colours = {}
    for name in names:
        rows = [f[name] for f in judged]
        hits = sum(1 for r in rows if r["components"] > 0)
        colours[name] = {
            "frames_with_component": hits,
            "fraction": hits / len(judged) if judged else None,
            "frames_with_pixels": sum(1 for r in rows if r["pixels"] > 0),
            "max_component": max((r["max_component"] for r in rows), default=0),
        }
    return {"frames": len(frames), "judged": len(judged),
            "unreadable": len(frames) - len(judged), "colours": colours}


def _project(points: np.ndarray, world_from_camera: np.ndarray, k: np.ndarray):
    r, t = world_from_camera[:3, :3], world_from_camera[:3, 3]
    cam = (points - t) @ r  # R^T (p - t), row form
    with np.errstate(divide="ignore", invalid="ignore"):
        homog = cam @ k.T
        uv = homog[:, :2] / homog[:, 2:3]
    return cam[:, 2], uv


def pixel_shift(points_world: np.ndarray, cam_pose_a: np.ndarray, cam_pose_b: np.ndarray,
                K: np.ndarray, near_m: float = NEAR_M) -> dict:
    """Pixel displacement of world points between two poses of one pinhole camera.

    ``points_world``: (N, 3) metres in the world frame. ``cam_pose_a``/``cam_pose_b``:
    4x4 world-from-camera transforms with ROS optical axes (x right, y down, z forward).
    ``K``: 3x3 intrinsic matrix in pixels. A point is dropped when its depth along z is
    ``<= near_m`` in either pose (behind or at the camera, as the overlay clips it).

    Returns ``kept`` (N,) bool; for the M kept points ``uv_a`` and ``uv_b`` (M, 2) pixels
    and ``shift_px`` (M,) their distance; ``max_px`` and ``p95_px`` (linear-interpolated
    95th percentile) as floats, None when no point is kept.
    """
    points = np.asarray(points_world, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points_world must be (N, 3), not {points.shape}")
    poses = [np.asarray(p, dtype=float) for p in (cam_pose_a, cam_pose_b)]
    if any(p.shape != (4, 4) for p in poses):
        raise ValueError("camera poses must be 4x4 world-from-camera transforms")
    k = np.asarray(K, dtype=float)
    if k.shape != (3, 3):
        raise ValueError(f"K must be 3x3, not {k.shape}")
    za, uv_a = _project(points, poses[0], k)
    zb, uv_b = _project(points, poses[1], k)
    kept = (za > near_m) & (zb > near_m)
    uv_a, uv_b = uv_a[kept], uv_b[kept]
    shift = np.linalg.norm(uv_b - uv_a, axis=1)
    return {"kept": kept, "uv_a": uv_a, "uv_b": uv_b, "shift_px": shift,
            "max_px": float(shift.max()) if shift.size else None,
            "p95_px": float(np.percentile(shift, 95)) if shift.size else None}


__all__ = ["NEAR_M", "arm_summary", "colour_mask", "frame_summary", "label_components",
           "pixel_shift"]
