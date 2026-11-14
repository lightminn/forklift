"""S4 live frames: colour masks, 4-connected components, frame and arm summaries, and the
offline pixel shift between two camera poses
(docs/plans/2026-11-14-path-display-perception-leak.md 4, S4)."""

import math
from collections import deque

import numpy as np
import pytest

from forklift_core.live.frame_judge import (
    arm_summary,
    colour_mask,
    frame_summary,
    label_components,
    pixel_shift,
)

# Test colours only; the tool reads the runner's colours from the source.
LINE = (250, 160, 20)
OTHER = (20, 110, 250)
GREY = 90


def grey_frame(shape=(60, 80)):
    return np.full((*shape, 3), GREY, dtype=int)


def jpeg_like(base, seed=0, amplitude=25):
    """``base`` with uniform noise of +-amplitude per channel and pixel (JPEG-like)."""
    rng = np.random.default_rng(seed)
    noise = rng.integers(-amplitude, amplitude + 1, size=base.shape)
    return np.clip(base + noise, 0, 255).astype(np.uint8)


def bfs_components(mask):
    """Reference 4-connected component sizes in raster order of first pixel."""
    seen = np.zeros_like(mask, dtype=bool)
    sizes = []
    h, w = mask.shape
    for r in range(h):
        for c in range(w):
            if not mask[r, c] or seen[r, c]:
                continue
            q, n = deque([(r, c)]), 0
            seen[r, c] = True
            while q:
                y, x = q.popleft()
                n += 1
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < h and 0 <= xx < w and mask[yy, xx] and not seen[yy, xx]:
                        seen[yy, xx] = True
                        q.append((yy, xx))
            sizes.append(n)
    return sizes


def test_colour_mask_takes_jpeg_like_noise_within_the_tolerance():
    base = grey_frame()
    region = (slice(20, 26), slice(5, 75))
    base[region] = LINE
    rgb = jpeg_like(base)
    want = np.zeros(rgb.shape[:2], dtype=bool)
    want[region] = True
    assert np.array_equal(colour_mask(rgb, LINE, 40), want)
    assert colour_mask(rgb, LINE, 5).sum() < want.sum() // 2  # too tight for the noise
    assert colour_mask(rgb, OTHER, 40).sum() == 0


def test_colour_mask_is_per_channel_absolute():
    rgb = np.zeros((1, 4, 3), dtype=np.uint8)
    rgb[0, 0] = LINE
    rgb[0, 1] = (LINE[0] - 40, LINE[1] + 40, LINE[2])  # on the edge in two channels
    rgb[0, 2] = (LINE[0], LINE[1], LINE[2] + 41)  # one channel over
    rgb[0, 3] = (LINE[0] - 41, LINE[1], LINE[2])
    assert colour_mask(rgb, LINE, 40).tolist() == [[True, True, False, False]]


def test_colour_mask_refuses_a_non_rgb_frame_or_a_negative_tolerance():
    with pytest.raises(ValueError):
        colour_mask(np.zeros((4, 4), dtype=np.uint8), LINE, 40)
    with pytest.raises(ValueError):
        colour_mask(np.zeros((4, 4, 3), dtype=np.uint8), LINE, -1)


def test_components_are_4_connected():
    u_shape = np.array([[1, 0, 1], [1, 0, 1], [1, 1, 1]], dtype=bool)
    labels, sizes = label_components(u_shape)
    assert sizes.tolist() == [7]
    assert np.array_equal(labels > 0, u_shape)

    diagonal = np.array([[1, 0], [0, 1]], dtype=bool)  # corners only touch: two components
    labels, sizes = label_components(diagonal)
    assert sizes.tolist() == [1, 1]
    assert labels.tolist() == [[1, 0], [0, 2]]


def test_components_join_branches_that_meet_further_down():
    # two arms that meet only in the last row, the right arm starting first
    mask = np.array([
        [0, 0, 0, 1],
        [1, 0, 0, 1],
        [1, 0, 0, 1],
        [1, 1, 1, 1],
    ], dtype=bool)
    labels, sizes = label_components(mask)
    assert sizes.tolist() == [9]
    assert set(np.unique(labels[mask]).tolist()) == {1}


def test_components_match_a_breadth_first_reference():
    rng = np.random.default_rng(3)
    for density in (0.2, 0.45, 0.6):
        mask = rng.random((40, 57)) < density
        labels, sizes = label_components(mask)
        assert sizes.tolist() == bfs_components(mask)
        assert labels.max() == len(sizes)
        counts = np.bincount(labels[mask], minlength=len(sizes) + 1)[1:]
        assert np.array_equal(counts, sizes)


def test_an_empty_mask_has_no_component():
    labels, sizes = label_components(np.zeros((5, 6), dtype=bool))
    assert sizes.size == 0 and not labels.any()


def test_frame_summary_counts_pixels_and_large_components_per_colour():
    base = grey_frame()
    base[10:15, 0:20] = LINE  # a 100 px line
    base[40:42, 50:52] = LINE  # a 4 px speck
    rgb = jpeg_like(base, amplitude=15)
    s = frame_summary(rgb, {"line": LINE, "other": OTHER}, tol=40, min_component=50)
    assert s["line"] == {"pixels": 104, "components": 1, "max_component": 100}
    assert s["other"] == {"pixels": 0, "components": 0, "max_component": 0}
    small = frame_summary(rgb, {"line": LINE}, tol=40, min_component=3)
    assert small["line"]["components"] == 2


def test_arm_summary_counts_frames_with_a_component():
    hit = {"line": {"pixels": 120, "components": 1, "max_component": 120}}
    big = {"line": {"pixels": 400, "components": 2, "max_component": 300}}
    miss = {"line": {"pixels": 10, "components": 0, "max_component": 10}}
    s = arm_summary([hit, miss, big, None, miss], ["line"])
    assert s["frames"] == 5 and s["unreadable"] == 1 and s["judged"] == 4
    line = s["colours"]["line"]
    assert line["frames_with_component"] == 2
    assert line["fraction"] == pytest.approx(0.5)
    assert line["max_component"] == 300
    empty = arm_summary([], ["line"])
    assert empty["colours"]["line"]["fraction"] is None


K = np.array([[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]])


def pose(rotation=None, translation=(0.0, 0.0, 0.0)):
    m = np.eye(4)
    if rotation is not None:
        m[:3, :3] = rotation
    m[:3, 3] = translation
    return m


def test_pixel_shift_of_a_sideways_step_by_hand():
    # camera A at the origin, optical axes = world axes; B moved 0.2 m to its right
    points = np.array([[1.0, 0.5, 2.0], [0.0, 0.0, 4.0]])
    out = pixel_shift(points, pose(), pose(translation=(0.2, 0.0, 0.0)), K)
    # (1, 0.5, 2): A sees u = 50 + 100*1/2 = 100, v = 40 + 100*0.5/2 = 65; B sees x = 0.8
    assert out["uv_a"][0] == pytest.approx([100.0, 65.0])
    assert out["uv_b"][0] == pytest.approx([90.0, 65.0])
    # (0, 0, 4): u 50 -> 50 - 100*0.2/4 = 45
    assert out["shift_px"] == pytest.approx([10.0, 5.0])
    assert out["max_px"] == pytest.approx(10.0)
    assert out["p95_px"] == pytest.approx(np.percentile([10.0, 5.0], 95))
    assert out["kept"].tolist() == [True, True]


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# A camera 0.5 m up looking along world +x with ROS optical axes: x right = world -y,
# y down = world -z, z forward = world +x (columns: the optical axes in the world).
LEVEL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def test_a_pure_yaw_shifts_every_depth_on_a_ray_alike():
    yaw = math.radians(3.0)
    a = pose(LEVEL, (0.0, 0.0, 0.5))
    b = pose(rot_z(yaw) @ LEVEL, (0.0, 0.0, 0.5))  # turned left about the world vertical
    ahead = np.array([[d, 0.0, 0.5] for d in (1.0, 3.0, 10.0)])
    out = pixel_shift(ahead, a, b, K)
    # the point ahead moves right by fx tan(yaw), whatever its depth; v does not change
    assert out["uv_a"][:, 0] == pytest.approx([50.0] * 3)
    assert out["uv_b"][:, 0] == pytest.approx([50.0 + 100.0 * math.tan(yaw)] * 3)
    assert out["uv_b"][:, 1] == pytest.approx(out["uv_a"][:, 1])
    assert out["shift_px"] == pytest.approx([100.0 * math.tan(yaw)] * 3)
    # off axis: a point 20 deg to the left (u < cx) keeps its bearing less the yaw
    bearing = math.radians(20.0)
    side = np.array([[2.0, 2.0 * math.tan(bearing), 0.5]])
    out = pixel_shift(side, a, b, K)
    assert out["uv_a"][0, 0] == pytest.approx(50.0 - 100.0 * math.tan(bearing))
    assert out["uv_b"][0, 0] == pytest.approx(50.0 - 100.0 * math.tan(bearing - yaw))


def test_points_behind_or_at_the_camera_are_dropped():
    a = pose(LEVEL, (-1.0, 0.0, 0.5))
    b = pose(LEVEL, (0.0, 0.0, 0.5))  # one metre further forward
    points = np.array([
        [-2.0, 0.0, 0.0],  # behind both
        [0.04, 0.0, 0.5],  # ahead of A, 0.04 m ahead of B: dropped (z <= 0.05)
        [0.05, 0.2, 0.5],  # exactly 0.05 m ahead of B: dropped
        [3.0, 0.2, 0.0],  # ahead of both
    ])
    out = pixel_shift(points, a, b, K)
    assert out["kept"].tolist() == [False, False, False, True]
    assert out["shift_px"].shape == (1,) and out["uv_a"].shape == (1, 2)
    none = pixel_shift(points[:1], a, b, K)
    assert none["shift_px"].size == 0 and none["max_px"] is None and none["p95_px"] is None


def test_pixel_shift_refuses_bad_shapes():
    with pytest.raises(ValueError):
        pixel_shift(np.zeros((2, 2)), pose(), pose(), K)
    with pytest.raises(ValueError):
        pixel_shift(np.zeros((2, 3)), np.eye(3), pose(), K)
    with pytest.raises(ValueError):
        pixel_shift(np.zeros((2, 3)), pose(), pose(), np.eye(4))
