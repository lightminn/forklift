import math

import numpy as np
import pytest

from forklift_core.localization.map_relocalization import (
    LossDetector,
    OccupancyMap,
    distance_field,
    hit_fraction,
    read_map,
    relocalize,
    scan_points,
    stack_scans,
)

RES = 0.05
# No box has another box at its half-turn image (x, y) -> (12 - x, 10 - y).
ASYMMETRIC = [(1.0, 1.0, 2.0, 4.0), (9.0, 1.0, 11.0, 2.0), (5.0, 8.0, 5.4, 8.4)]


def _room(width_m, height_m, boxes=()):
    """Walls one cell thick around a width x height room, plus filled boxes."""
    cols, rows = round(width_m / RES), round(height_m / RES)
    occupied = np.zeros((rows, cols), dtype=bool)
    occupied[[0, -1], :] = True
    occupied[:, [0, -1]] = True
    for x0, y0, x1, y1 in boxes:
        occupied[
            round(y0 / RES) : round(y1 / RES), round(x0 / RES) : round(x1 / RES)
        ] = True
    return OccupancyMap(occupied, ~occupied, RES, (0.0, 0.0))


def _scan(grid, pose, beams=180, max_range=12.0):
    """Ray-march every beam until it enters an occupied cell."""
    angles = pose[2] + np.linspace(-math.pi, math.pi, beams, endpoint=False)
    steps = np.arange(0.0, max_range, RES / 4)
    xy = (
        pose[:2]
        + steps[None, :, None]
        * np.stack((np.cos(angles), np.sin(angles)), 1)[:, None, :]
    )
    row, col, inside = grid.cells(xy)
    hit = inside & grid.occupied[row, col]
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    ranges = np.where(first >= 0, steps[first], np.inf)
    return scan_points(ranges, -math.pi, 2 * math.pi / beams, 0.1, max_range)


def test_distance_field_matches_brute_force():
    rng = np.random.default_rng(3)
    occupied = rng.random((20, 25)) < 0.05
    field = distance_field(occupied, RES, 0.4)
    cells = np.argwhere(occupied)
    for r in range(20):
        for c in range(25):
            exact = np.hypot(*(cells - (r, c)).T).min() * RES
            assert field[r, c] == pytest.approx(min(exact, 0.4))


def test_map_round_trips_through_the_recorder_pgm(tmp_path):
    grid = _room(3.0, 2.0, boxes=[(1.0, 0.5, 1.5, 1.0)])
    unknown = np.zeros_like(grid.occupied)
    unknown[5:8, 20:30] = True
    image = np.where(grid.occupied, 0, np.where(unknown, 205, 254)).astype(np.uint8)
    rows, cols = image.shape
    (tmp_path / "map.pgm").write_bytes(
        f"P5\n{cols} {rows}\n255\n".encode() + np.flipud(image).tobytes()
    )
    (tmp_path / "map.yaml").write_text(
        "image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [-1.5, 2.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    loaded = read_map(tmp_path / "map.yaml")
    assert np.array_equal(loaded.occupied, grid.occupied)
    assert np.array_equal(loaded.free, ~grid.occupied & ~unknown)
    assert loaded.origin_xy_m == (-1.5, 2.0)


def test_scan_points_drop_non_finite_and_apply_the_mount():
    ranges = np.array([1.0, np.inf, 2.0, -np.inf, 0.05, 20.0])
    angles_step = math.pi / 2
    points = scan_points(
        ranges, 0.0, angles_step, 0.1, 12.0, mount_xy_yaw=(-0.12, 0.0, 0.0)
    )
    assert np.allclose(points, [[0.88, 0.0], [-2.12, 0.0]])


def test_stacked_scans_put_one_world_point_in_one_place():
    target = np.array([4.0, 1.0])
    poses = np.array([[0.0, 0.0, 0.0], [1.0, 0.5, 0.4]])
    seen = []
    for x, y, yaw in poses:
        c, s = math.cos(yaw), math.sin(yaw)
        d = target - (x, y)
        seen.append([[c * d[0] + s * d[1], -s * d[0] + c * d[1]]])
    stacked = stack_scans(seen, poses)
    assert np.allclose(stacked[0], stacked[1])


def test_relocalization_finds_the_pose_in_an_asymmetric_room():
    grid = _room(
        12.0,
        10.0,
        boxes=ASYMMETRIC,
    )
    truth = np.array([6.2, 3.3, 0.7])
    result = relocalize(grid, _scan(grid, truth), coarse_yaw_step_rad=math.radians(4))
    assert result.accepted, result.reason
    assert math.hypot(result.pose[0] - truth[0], result.pose[1] - truth[1]) < 0.1
    assert abs(result.pose[2] - truth[2]) < 0.05


def test_relocalization_refuses_a_room_that_looks_the_same_turned_around():
    grid = _room(10.0, 6.0)
    truth = np.array([3.0, 2.0, 0.3])
    result = relocalize(grid, _scan(grid, truth), coarse_yaw_step_rad=math.radians(4))
    assert not result.accepted
    assert result.reason.startswith("ambiguous")
    # The rival is the half-turn image of the truth.
    candidates = [result.pose, result.rival_pose]
    mirror = (10.0 - truth[0], 6.0 - truth[1])
    assert any(
        math.hypot(p[0] - mirror[0], p[1] - mirror[1]) < 0.15 for p in candidates
    )


def test_hit_fraction_separates_the_true_pose_from_a_wrong_one():
    grid = _room(12.0, 10.0, boxes=ASYMMETRIC)
    field = distance_field(grid.occupied, RES, 1.0)
    truth = np.array([6.2, 3.3, 0.7])
    points = _scan(grid, truth)
    assert hit_fraction(grid, field, points, truth, 0.1) > 0.95
    assert hit_fraction(grid, field, points, truth + (0.5, -0.4, 0.2), 0.1) < 0.5


def test_loss_is_declared_after_consecutive_low_scans_only():
    detector = LossDetector(threshold=0.5, consecutive=3)
    states = [detector.update(f) for f in (0.9, 0.3, 0.3, 0.9, 0.3, 0.3, 0.3)]
    assert states == [False] * 6 + [True]
