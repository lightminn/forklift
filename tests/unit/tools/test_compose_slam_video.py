"""Frame geometry of the three-panel SLAM video, without video files."""

import math

import numpy as np
import pytest

from tools.compose_slam_video import (
    fit_floor_affine,
    latest_index,
    map_from_world,
    occupancy_colours,
    sample_occupancy,
    world_from_map,
)


def test_floor_affine_recovers_a_nadir_camera_mapping():
    # 20 px per metre, north up: u grows with x, v shrinks with y.
    world = np.array([[0, 0], [10, 0], [10, 8], [0, 8], [5, 4]], float)
    pixels = np.column_stack((100 + 20 * world[:, 0], 600 - 20 * world[:, 1]))
    affine = fit_floor_affine(world, pixels)
    np.testing.assert_allclose(affine @ [2.5, 1.0, 1.0], [150.0, 580.0])


def test_latest_index_is_the_last_sample_not_after_the_time():
    stamps = np.array([0.0, 0.1, 0.2, 0.3])
    assert latest_index(stamps, -0.01) == -1
    assert latest_index(stamps, 0.0) == 0
    assert latest_index(stamps, 0.25) == 2
    assert latest_index(stamps, 9.0) == 3


def test_world_and_map_frames_round_trip_through_the_start_pose():
    start = np.array([3.0, -2.0, math.pi / 2])
    # One metre ahead in the map frame is one metre north of the start.
    np.testing.assert_allclose(
        world_from_map(np.array([[1.0, 0.0, 0.0]]), start), [[3.0, -1.0, math.pi / 2]]
    )
    points = np.array([[4.0, 1.0], [3.0, -2.0]])
    back = world_from_map(
        np.column_stack((map_from_world(points, start), np.zeros(2))), start
    )
    np.testing.assert_allclose(back[:, :2], points, atol=1e-12)


def test_occupancy_sampling_uses_row_zero_at_the_origin_and_marks_outside():
    grid = np.array([[0, 100], [-1, 50]], dtype=np.int8)  # row 0 is y = origin
    xm = np.array([0.02, 0.07, 0.02, 0.5])
    ym = np.array([0.02, 0.02, 0.07, 0.5])
    values = sample_occupancy(
        grid, origin_xy=(0.0, 0.0), resolution_m=0.05, xm=xm, ym=ym
    )
    assert values.tolist() == [0, 100, -1, -2]


def test_occupancy_colours_separate_unknown_free_occupied_and_outside():
    colours = occupancy_colours(np.array([-2, -1, 0, 100]))
    outside, unknown, free, occupied = colours.astype(int)
    assert free.sum() > unknown.sum() > occupied.sum()
    assert np.array_equal(outside, unknown)
    with pytest.raises(ValueError):
        occupancy_colours(np.array([101]))


def test_phase_at_follows_the_recorded_transitions():
    from tools.compose_slam_video import PHASE_NAMES, phase_at

    transitions = [
        {"from": "observe", "to": "approach", "time_s": 17.3},
        {"from": "approach", "to": "insert", "time_s": 27.5},
    ]
    assert phase_at(transitions, 0.0) == "observe"
    assert phase_at(transitions, 17.3) == "approach"
    assert phase_at(transitions, 99.0) == "insert"
    assert phase_at([], 5.0) is None
    assert {"observe", "insert", "transport", "return_home", "complete"} <= set(
        PHASE_NAMES
    )
