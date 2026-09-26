import numpy as np
import pytest

from forklift_core.sensors import lidar


def scan(ranges, **overrides):
    options = dict(
        angle_min_rad=0,
        angle_increment_rad=np.pi / 2,
        range_min_m=0.1,
        range_max_m=10,
        frame_id="laser",
    )
    options.update(overrides)
    return lidar.scan_to_points(ranges, **options)


def test_scan_angle_origin_and_rotation_follow_laser_frame_convention():
    points = scan([1, 2, 3, 4])
    assert points.frame_id == "laser"
    np.testing.assert_allclose(
        points.xyz_m, [[1, 0, 0], [0, 2, 0], [-3, 0, 0], [0, -4, 0]], atol=1e-14
    )


def test_negative_angle_increment_is_preserved():
    points = scan([1, 2], angle_min_rad=np.pi / 2, angle_increment_rad=-np.pi / 2)
    np.testing.assert_allclose(points.xyz_m, [[0, 1, 0], [2, 0, 0]], atol=1e-14)


def test_invalid_beams_remain_unknown_and_range_boundaries_are_inclusive():
    points = scan([np.nan, np.inf, -np.inf, 0, -1, 0.09, 10.01, 0.1, 10])
    np.testing.assert_array_equal(points.valid, [False] * 7 + [True, True])
    assert np.isnan(points.xyz_m[:7]).all()


@pytest.mark.parametrize(
    "override",
    [
        dict(angle_min_rad=np.nan),
        dict(angle_increment_rad=0),
        dict(angle_increment_rad=np.inf),
        dict(range_min_m=-1),
        dict(range_max_m=np.inf),
        dict(range_min_m=11),
        dict(range_max_m=0),
        dict(frame_id=""),
        dict(angle_min_rad=1e308, angle_increment_rad=1e308),
    ],
)
def test_malformed_scan_metadata_is_rejected(override):
    with pytest.raises(ValueError):
        scan([1, 2], **override)


@pytest.mark.parametrize("ranges", [[[1, 2]], [1 + 2j], ["1", "2"]])
def test_scan_requires_a_one_dimensional_real_numeric_range_array(ranges):
    with pytest.raises(ValueError):
        scan(ranges)


def test_empty_scan_is_not_an_obstacle_free_observation():
    result = scan([])
    assert result.xyz_m.shape == (0, 3)
    assert result.valid.size == 0


def test_full_circle_pattern_spaces_beams_evenly_from_minus_pi():
    pattern = lidar.PlanarScanPattern(beam_count=1600, range_min_m=0.2, range_max_m=12)
    angles = pattern.beam_angles_rad()
    assert angles.shape == (1600,)
    assert angles[0] == -np.pi
    assert pattern.angle_increment_rad == pytest.approx(np.deg2rad(0.225))
    assert angles[-1] == pytest.approx(np.pi - pattern.angle_increment_rad)


def test_ray_distances_follow_rep117_for_misses_and_close_hits():
    pattern = lidar.PlanarScanPattern(beam_count=4, range_min_m=0.2, range_max_m=12)
    ranges = pattern.ranges_from_hits(
        np.array([5.0, np.nan, 0.1, 12.0]), np.array([True, False, True, True])
    )
    # +inf: nothing within range; -inf: an object too close to measure.
    np.testing.assert_array_equal(ranges, [5.0, np.inf, -np.inf, 12.0])
    points = lidar.scan_to_points(
        ranges,
        angle_min_rad=-np.pi,
        angle_increment_rad=pattern.angle_increment_rad,
        range_min_m=pattern.range_min_m,
        range_max_m=pattern.range_max_m,
        frame_id="laser",
    )
    np.testing.assert_array_equal(points.valid, [True, False, False, True])


@pytest.mark.parametrize(
    "kwargs",
    [
        dict(beam_count=0, range_min_m=0.2, range_max_m=12),
        dict(beam_count=True, range_min_m=0.2, range_max_m=12),
        dict(beam_count=360, range_min_m=-0.1, range_max_m=12),
        dict(beam_count=360, range_min_m=5, range_max_m=5),
        dict(beam_count=360, range_min_m=0.2, range_max_m=np.inf),
    ],
)
def test_invalid_scan_pattern_raises(kwargs):
    with pytest.raises(ValueError):
        lidar.PlanarScanPattern(**kwargs)


def test_ranges_from_hits_rejects_mismatched_or_impossible_input():
    pattern = lidar.PlanarScanPattern(beam_count=2, range_min_m=0.2, range_max_m=12)
    with pytest.raises(ValueError):
        pattern.ranges_from_hits(np.array([1.0]), np.array([True]))
    with pytest.raises(ValueError):
        pattern.ranges_from_hits(np.array([13.0, 1.0]), np.array([True, True]))
