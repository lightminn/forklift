from importlib import import_module

import numpy as np
import pytest


def scan(ranges, **overrides):
    lidar = import_module("forklift_core.lidar")
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
