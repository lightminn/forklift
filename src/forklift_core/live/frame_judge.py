"""Judging saved live-console frames for the overlay lines, and the overlay's offline
pixel shift between two camera poses (docs/plans/2026-11-14-path-display-perception-leak.md
4, S4).

Pure NumPy, no image library: the caller decodes the frames. Nothing here decides whether
an arm passes; the S4 registration sets those thresholds, and these functions only count.

- ``colour_mask``: the pixels within a per-channel absolute tolerance of one colour.
- ``label_components``: 4-connected components of a mask (run-length labelling).
- ``frame_summary`` / ``arm_summary``: the counts the judge reports, per frame and per arm.
- ``pixel_shift``: how far a world point's pixel moves between two camera poses, for the
  misalignment the overlay gets when the frame's pixels trail its render time;
  ``pixel_shift_in_view`` keeps only the points on the picture.
- ``PoseTrack`` (``interpolate_pose``, ``motion_at``) / ``interpolate_value`` / ``camera_pose``: a recorded
  pose track read at any time (linear position, spherical rotation) and the camera on it.
- ``displayed_at``: which display events stand at a time (the latest per name).
- ``shift_summary``: max, 95th percentile and median of per-frame shifts.
"""

from __future__ import annotations

import math
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


def pixel_shift_in_view(points_world: np.ndarray, cam_pose_a: np.ndarray, cam_pose_b: np.ndarray,
                        K: np.ndarray, width: int, height: int, near_m: float = NEAR_M) -> dict:
    """``pixel_shift`` restricted to the points that land inside a ``width`` x ``height``
    image in either pose (0 <= u < width, 0 <= v < height): a point off the picture in both
    is not seen, whatever it moves. ``kept`` is over all N points, as in ``pixel_shift``."""
    out = pixel_shift(points_world, cam_pose_a, cam_pose_b, K, near_m)

    def inside(uv):
        return (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)

    seen = inside(out["uv_a"]) | inside(out["uv_b"])
    kept = out["kept"].copy()
    kept[np.flatnonzero(kept)[~seen]] = False
    shift = out["shift_px"][seen]
    return {"kept": kept, "uv_a": out["uv_a"][seen], "uv_b": out["uv_b"][seen], "shift_px": shift,
            "max_px": float(shift.max()) if shift.size else None,
            "p95_px": float(np.percentile(shift, 95)) if shift.size else None}


# ------------------------------------------------------------------ pose tracks


def matrix_from_wxyz(q: Sequence[float]) -> np.ndarray:
    """3x3 rotation of a quaternion (w, x, y, z); normalised first."""
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def slerp_wxyz(q0: Sequence[float], q1: Sequence[float], a: float) -> np.ndarray:
    """Spherical interpolation from ``q0`` (a = 0) to ``q1`` (a = 1) the short way round."""
    q0 = np.asarray(q0, dtype=float) / np.linalg.norm(q0)
    q1 = np.asarray(q1, dtype=float) / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:  # nearly the same: linear, renormalised
        q = q0 + a * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(min(1.0, d))
    return (math.sin((1 - a) * theta) * q0 + math.sin(a * theta) * q1) / math.sin(theta)


def _check_stamps(stamps) -> np.ndarray:
    stamps = np.asarray(stamps, dtype=float).reshape(-1)
    if stamps.size > 1 and np.any(np.diff(stamps) < 0):
        raise ValueError("stamps must not decrease")
    return stamps


def _bracket(stamps: np.ndarray, t: float):
    """(i, j, a): ``t`` lies at fraction ``a`` from stamps[i] to stamps[j]; None outside."""
    if stamps.size == 0 or not math.isfinite(t) or t < stamps[0] or t > stamps[-1]:
        return None
    i = int(np.searchsorted(stamps, t, side="right")) - 1
    j = min(i + 1, stamps.size - 1)
    span = stamps[j] - stamps[i]
    return i, j, (0.0 if span <= 0 else float((t - stamps[i]) / span))


class PoseTrack:
    """A pose track checked once and read at any time.

    ``stamps``: (N,) non-decreasing seconds. ``poses``: (N, 7) rows ``x, y, z, qw, qx, qy,
    qz`` (the runner's slam_log ``base_pose_world``)."""

    def __init__(self, stamps, poses) -> None:
        self.stamps = _check_stamps(stamps)
        self.poses = np.asarray(poses, dtype=float)
        if self.poses.ndim != 2 or self.poses.shape != (self.stamps.size, 7):
            raise ValueError(f"poses must be (N, 7) with N = len(stamps), not {self.poses.shape}")

    def at(self, t: float) -> np.ndarray | None:
        """The 4x4 world-from-body transform at ``t``: position linear, rotation spherical
        between the two stamps around ``t``; None outside the track (no extrapolation)."""
        where = _bracket(self.stamps, float(t))
        if where is None:
            return None
        i, j, a = where
        m = np.eye(4)
        m[:3, 3] = (1 - a) * self.poses[i, :3] + a * self.poses[j, :3]
        m[:3, :3] = matrix_from_wxyz(slerp_wxyz(self.poses[i, 3:], self.poses[j, 3:], a))
        return m

    def motion(self, t: float, window_s: float) -> dict | None:
        """Mean motion over ``[t - window_s/2, t + window_s/2]`` cut to the track:
        ``yaw_rate_radps`` (left positive) and ``speed_mps`` (planar, unsigned). None when
        less than half the window is on the track."""
        if not window_s > 0:
            raise ValueError("window_s must be > 0")
        if self.stamps.size == 0:
            return None
        t0, t1 = max(t - window_s / 2, self.stamps[0]), min(t + window_s / 2, self.stamps[-1])
        if t1 - t0 < window_s / 2:
            return None
        a, b = self.at(t0), self.at(t1)
        d = yaw_of(b) - yaw_of(a)
        return {"yaw_rate_radps": math.atan2(math.sin(d), math.cos(d)) / (t1 - t0),
                "speed_mps": float(np.hypot(*(b[:2, 3] - a[:2, 3]))) / (t1 - t0)}


def interpolate_pose(stamps, poses, t: float) -> np.ndarray | None:
    """``PoseTrack(stamps, poses).at(t)``."""
    return PoseTrack(stamps, poses).at(t)


def motion_at(stamps, poses, t: float, window_s: float) -> dict | None:
    """``PoseTrack(stamps, poses).motion(t, window_s)``."""
    return PoseTrack(stamps, poses).motion(t, window_s)


def interpolate_value(stamps, values, t: float) -> float | None:
    """A scalar track read linearly at ``t``; None outside it."""
    stamps = _check_stamps(stamps)
    values = np.asarray(values, dtype=float).reshape(-1)
    if values.size != stamps.size:
        raise ValueError("values and stamps differ in length")
    where = _bracket(stamps, float(t))
    if where is None:
        return None
    i, j, a = where
    return float((1 - a) * values[i] + a * values[j])


def camera_pose(world_from_base: np.ndarray, base_from_camera: np.ndarray, lift_m: float = 0.0) -> np.ndarray:
    """world <- camera of a camera mounted at ``base_from_camera`` (its pose at lift 0) on a
    carriage raised ``lift_m`` along the base's z axis (the fork_lift joint's axis); 0 for a
    camera on the base itself."""
    lift = np.eye(4)
    lift[2, 3] = float(lift_m)
    return np.asarray(world_from_base, dtype=float) @ lift @ np.asarray(base_from_camera, dtype=float)


def yaw_of(world_from_body: np.ndarray) -> float:
    """Heading of the body's x axis in the world xy plane."""
    r = np.asarray(world_from_body, dtype=float)
    return math.atan2(r[1, 0], r[0, 0])


def displayed_at(events: Iterable[Mapping], t: float) -> dict[str, Mapping]:
    """The display standing at ``t``: per ``name``, the latest event with ``time_s <= t``
    (a later event of one name replaces its earlier line and nothing removes one, as
    sim/isaac/path_overlay's items). Of two events of one name at one time, the later in
    order stands."""
    out: dict[str, Mapping] = {}
    for event in sorted(events, key=lambda e: float(e["time_s"])):  # stable: equal times keep order
        if float(event["time_s"]) > t:
            break
        out[str(event["name"])] = event
    return out


def shift_summary(values: Iterable[float | None]) -> dict:
    """``frames`` (every value), ``measured`` (the non-None ones) and their ``max_px``,
    ``p95_px`` (linear-interpolated) and ``median_px``; None without a measured one."""
    values = list(values)
    got = np.asarray([v for v in values if v is not None], dtype=float)
    return {"frames": len(values), "measured": int(got.size),
            "max_px": float(got.max()) if got.size else None,
            "p95_px": float(np.percentile(got, 95)) if got.size else None,
            "median_px": float(np.median(got)) if got.size else None}


__all__ = ["NEAR_M", "PoseTrack", "arm_summary", "camera_pose", "colour_mask", "displayed_at", "frame_summary",
           "interpolate_pose", "interpolate_value", "label_components", "matrix_from_wxyz", "motion_at",
           "pixel_shift", "pixel_shift_in_view", "shift_summary", "slerp_wxyz", "yaw_of"]
