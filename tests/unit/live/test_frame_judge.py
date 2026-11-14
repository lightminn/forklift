"""S4 live frames: colour masks, 4-connected components, frame and arm summaries, and the
offline pixel shift between two camera poses
(docs/plans/2026-11-14-path-display-perception-leak.md 4, S4)."""

import math
from collections import deque

import numpy as np
import pytest

from forklift_core.live.frame_judge import (
    arm_summary,
    camera_pose,
    colour_mask,
    densify_polyline,
    displayed_at,
    frame_summary,
    interpolate_pose,
    interpolate_value,
    label_components,
    matrix_from_wxyz,
    motion_at,
    pixel_shift,
    pixel_shift_in_view,
    shift_summary,
    slerp_wxyz,
    yaw_of,
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


def q_yaw(a):
    """(w, x, y, z) of a turn ``a`` about the world vertical."""
    return [math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)]


def test_matrix_and_slerp_of_a_yaw():
    assert matrix_from_wxyz(q_yaw(0.4)) == pytest.approx(rot_z(0.4))
    assert matrix_from_wxyz([2.0, 0.0, 0.0, 0.0]) == pytest.approx(np.eye(3))  # normalised first
    for a in (0.0, 0.25, 0.5, 1.0):
        assert matrix_from_wxyz(slerp_wxyz(q_yaw(0.2), q_yaw(1.0), a)) == pytest.approx(rot_z(0.2 + 0.8 * a))
    # the short way round: from +170 deg to -170 deg passes 180, not 0
    mid = matrix_from_wxyz(slerp_wxyz(q_yaw(math.radians(170)), q_yaw(math.radians(-170)), 0.5))
    assert abs(yaw_of(pose(mid))) == pytest.approx(math.pi)


def track(stamps, xs, yaws, z=0.0):
    poses = np.array([[x, 0.0, z, *q_yaw(a)] for x, a in zip(xs, yaws, strict=True)])
    return poses, np.asarray(stamps, dtype=float)


def test_interpolate_pose_is_linear_in_position_and_spherical_in_rotation():
    poses, stamps = track([0.0, 1.0, 2.0], [0.0, 1.0, 3.0], [0.0, 0.5, 0.5])
    m = interpolate_pose(stamps, poses, 0.25)
    assert m[:3, 3] == pytest.approx([0.25, 0.0, 0.0])
    assert m[:3, :3] == pytest.approx(rot_z(0.125))
    m = interpolate_pose(stamps, poses, 1.5)
    assert m[:3, 3] == pytest.approx([2.0, 0.0, 0.0])
    assert yaw_of(m) == pytest.approx(0.5)
    assert interpolate_pose(stamps, poses, 2.0)[:3, 3] == pytest.approx([3.0, 0.0, 0.0])  # the last stamp
    assert interpolate_pose(stamps, poses, 0.0)[:3, 3] == pytest.approx([0.0, 0.0, 0.0])
    assert interpolate_pose(stamps, poses, -0.01) is None
    assert interpolate_pose(stamps, poses, 2.01) is None


def test_interpolate_pose_takes_a_repeated_stamp_and_refuses_bad_input():
    poses, stamps = track([0.0, 1.0, 1.0, 2.0], [0.0, 1.0, 1.0, 2.0], [0.0] * 4)
    assert interpolate_pose(stamps, poses, 1.0)[0, 3] == pytest.approx(1.0)
    with pytest.raises(ValueError):
        interpolate_pose([1.0, 0.0], poses[:2], 0.5)  # decreasing stamps
    with pytest.raises(ValueError):
        interpolate_pose(stamps, poses[:, :6], 0.5)


def test_interpolate_value():
    assert interpolate_value([0.0, 0.1, 0.2], [0.0, 0.1, 0.3], 0.15) == pytest.approx(0.2)
    assert interpolate_value([0.0, 0.1], [0.0, 0.1], 0.2) is None
    with pytest.raises(ValueError):
        interpolate_value([0.0, 0.1], [0.0], 0.05)


def test_camera_pose_adds_the_lift_along_the_base_vertical():
    base = pose(rot_z(math.pi / 2), (1.0, 2.0, 0.0))  # facing world +y
    mount = pose(LEVEL, (0.3, 0.0, 0.4))  # 0.3 m ahead, 0.4 m up, looking forward
    cam = camera_pose(base, mount, 0.1)
    assert cam[:3, 3] == pytest.approx([1.0, 2.3, 0.5])
    assert cam[:3, 2] == pytest.approx([0.0, 1.0, 0.0])  # optical z along the truck's heading
    assert camera_pose(base, mount)[:3, 3] == pytest.approx([1.0, 2.3, 0.4])


def test_motion_at_a_turn_and_a_straight_drive():
    stamps = np.arange(0, 241) / 120
    turn = np.array([[0.0, 0.0, 0.0, *q_yaw(0.3 * t)] for t in stamps])
    m = motion_at(stamps, turn, 1.0, 0.1)
    assert m["yaw_rate_radps"] == pytest.approx(0.3)
    assert m["speed_mps"] == pytest.approx(0.0, abs=1e-9)
    drive = np.array([[0.5 * t, 0.0, 0.0, *q_yaw(0.0)] for t in stamps])
    m = motion_at(stamps, drive, 1.0, 0.1)
    assert m["yaw_rate_radps"] == pytest.approx(0.0)
    assert m["speed_mps"] == pytest.approx(0.5)
    assert motion_at(stamps, drive, 2.0, 0.1)["speed_mps"] == pytest.approx(0.5)  # half the window on the track
    assert motion_at(stamps, drive, 2.04, 0.1) is None  # less than half
    with pytest.raises(ValueError):
        motion_at(stamps, drive, 1.0, 0.0)


def test_displayed_at_keeps_the_latest_line_per_name():
    events = [
        {"time_s": 1.0, "name": "Approach", "v": 1},
        {"time_s": 1.0, "name": "Transport", "v": 2},
        {"time_s": 5.0, "name": "Approach", "v": 3},
        {"time_s": 5.0, "name": "Approach", "v": 4},  # same time: the later one stands
        {"time_s": 3.0, "name": "Observe", "v": 5},  # out of order in the list
    ]

    def standing(t):
        return {name: e["v"] for name, e in displayed_at(events, t).items()}

    assert standing(0.5) == {}
    assert standing(1.0) == {"Approach": 1, "Transport": 2}
    assert standing(4.0) == {"Approach": 1, "Transport": 2, "Observe": 5}
    assert standing(9.0)["Approach"] == 4
    assert list(displayed_at([{"time_s": -math.inf, "name": "transport"}], 0.0)) == ["transport"]


def test_pixel_shift_in_view_drops_points_off_the_picture():
    a = pose(LEVEL, (0.0, 0.0, 0.5))
    b = pose(rot_z(math.radians(3.0)) @ LEVEL, (0.0, 0.0, 0.5))
    points = np.array([
        [2.0, 0.0, 0.5],  # centre: u = 50
        [1.0, 3.0, 0.5],  # far left: u = 50 - 100*3 < 0 in both
        [-1.0, 0.0, 0.5],  # behind
    ])
    out = pixel_shift_in_view(points, a, b, K, 100, 80)
    assert out["kept"].tolist() == [True, False, False]
    assert out["shift_px"] == pytest.approx([100.0 * math.tan(math.radians(3.0))])
    # a point that is on the picture in pose b only is kept
    edge = np.array([[2.0, 2.0 * math.tan(math.radians(26.8)), 0.5]])
    plain = pixel_shift(edge, a, b, K)
    assert plain["uv_a"][0, 0] < 0 < plain["uv_b"][0, 0]
    assert pixel_shift_in_view(edge, a, b, K, 100, 80)["kept"].tolist() == [True]
    empty = pixel_shift_in_view(points[1:], a, b, K, 100, 80)
    assert empty["max_px"] is None
    assert empty["shift_px"].size == 0


def test_displayed_at_applies_clears_and_keys():
    ring = {"name": "destination"}
    events = [
        {"time_s": 0.0, "name": "Approach", "v": 1},
        {"time_s": 0.0, **ring, "key": "destination:a", "v": 2},
        {"time_s": 0.0, **ring, "key": "destination:b", "v": 3},  # two rings of one name stand together
        {"time_s": 1.0, "name": "Approach", "clear": True},
        {"time_s": 2.0, **ring, "key": "destination:a", "clear": True},  # one ring by its key
        {"time_s": 3.0, "name": "Approach", "v": 4},
        {"time_s": 4.0, **ring, "clear": True},  # no key: every ring
    ]

    def standing(t):
        return {k: e["v"] for k, e in displayed_at(events, t).items()}

    assert standing(0.5) == {"Approach": 1, "destination:a": 2, "destination:b": 3}
    assert standing(1.5) == {"destination:a": 2, "destination:b": 3}
    assert standing(2.5) == {"destination:b": 3}
    assert standing(3.5) == {"destination:b": 3, "Approach": 4}
    assert standing(5.0) == {"Approach": 4}


def test_densify_polyline_keeps_vertices_and_spacing():
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.1, 0.0]])
    out = densify_polyline(pts, 0.3)
    # 1 m in ceil(1/0.3) = 4 pieces, 0.1 m in 1, both ends kept
    assert out[:, 0].tolist() == pytest.approx([0.0, 0.25, 0.5, 0.75, 1.0, 1.0])
    assert out[-1] == pytest.approx(pts[-1]) and out[0] == pytest.approx(pts[0])
    assert np.linalg.norm(np.diff(out, axis=0), axis=1).max() <= 0.3 + 1e-12
    for v in pts:
        assert np.min(np.linalg.norm(out - v, axis=1)) == pytest.approx(0.0, abs=1e-12)
    square = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]])
    ring = densify_polyline(square, 0.5, closed=True)
    assert len(ring) == 8  # four sides of two pieces, the closing end not repeated
    assert ring[-1] == pytest.approx([0.0, 0.5, 0.0])  # the closing side is sampled
    assert densify_polyline(pts[:1], 0.1).tolist() == pts[:1].tolist()
    with pytest.raises(ValueError):
        densify_polyline(pts, 0.0)
    with pytest.raises(ValueError):
        densify_polyline(pts[:, :2], 0.1)


def test_a_segment_crossing_the_picture_is_seen_only_once_densified():
    a = pose(LEVEL, (0.0, 0.0, 0.5))
    b = pose(rot_z(math.radians(3.0)) @ LEVEL, (0.0, 0.0, 0.5))
    across = np.array([[2.0, 30.0, 0.0], [2.0, -30.0, 0.0]])  # 2 m ahead, ends far off both sides
    assert pixel_shift_in_view(across, a, b, K, 100, 80)["max_px"] is None
    out = pixel_shift_in_view(densify_polyline(across, 0.02), a, b, K, 100, 80)
    assert out["shift_px"].size > 0 and out["max_px"] is not None


def test_shift_summary():
    s = shift_summary([1.0, None, 3.0, 2.0])
    assert s["frames"] == 4 and s["measured"] == 3
    assert s["max_px"] == 3.0 and s["median_px"] == 2.0
    assert s["p95_px"] == pytest.approx(np.percentile([1.0, 3.0, 2.0], 95))
    assert shift_summary([None]) == {"frames": 1, "measured": 0, "max_px": None, "p95_px": None,
                                     "median_px": None}
