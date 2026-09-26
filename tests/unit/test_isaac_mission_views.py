"""CPU tests for optional Isaac recording geometry and presentation mapping."""

import math

import numpy as np
import pytest

from sim.isaac.mission_views import (
    chase_pose,
    depth_colormap,
    parse_extra_views,
    phase_label,
    project_world_opening,
)


def test_extra_views_validate_dependencies_and_names():
    assert parse_extra_views("quarter,chase,perception", True, True) == (
        "quarter",
        "chase",
        "perception",
    )
    assert parse_extra_views("", False, False) == ()
    with pytest.raises(ValueError, match="--video"):
        parse_extra_views("chase", False, False)
    with pytest.raises(ValueError, match="--use-perception"):
        parse_extra_views("perception", True, False)
    with pytest.raises(ValueError, match="Unknown"):
        parse_extra_views("other", True, True)


def test_chase_pose_wraps_yaw_and_offsets_behind_base():
    eye, target, yaw = chase_pose([1, 2, 0], -math.pi + 0.1, math.pi - 0.1, 0.5)
    assert abs(abs(yaw) - math.pi) < 0.2
    assert eye[2] == pytest.approx(1.6)
    assert target[2] == pytest.approx(0.3)
    assert np.linalg.norm(eye[:2] - [1, 2]) > 2.4


def test_depth_colormap_uses_fixed_range_and_black_invalid():
    colors = depth_colormap(np.array([[0.279, 0.28, 0.3, 4.0, np.nan, 0.0]]))
    assert colors.shape == (1, 6, 3)
    assert colors.dtype == np.uint8
    assert tuple(colors[0, 0]) == (0, 0, 0)
    assert tuple(colors[0, 4]) == (0, 0, 0)
    assert tuple(colors[0, 5]) == (0, 0, 0)
    assert tuple(colors[0, 1]) != (0, 0, 0)
    assert not np.array_equal(colors[0, 2], colors[0, 3])


def test_observe_plan_window_and_overlay_use_logged_acceptance_time():
    from tools.deck.isaac_mission_views import observe_plan_selection

    frames = [
        {"phase": phase, "simulation_time_s": time}
        for phase, time in (
            ("observe", 0.0),
            ("observe", 0.05),
            ("approach", 0.1),
            ("approach", 3.1),
            ("approach", 3.15),
        )
    ]
    selected, overlay_start, fallback = observe_plan_selection(
        frames, {"acceptance_simulation_time_s": 0.05}
    )
    assert selected == [0, 1, 2, 3]
    assert overlay_start == 0.05
    assert fallback is False
    selected, overlay_start, fallback = observe_plan_selection(frames, {})
    assert selected == [0, 1, 2, 3]
    assert overlay_start == 0.1
    assert fallback is True


def test_phase_labels_cover_all_mission_phases():
    assert [
        phase_label(name)
        for name in (
            "observe",
            "approach",
            "insert",
            "lift",
            "extract",
            "transport",
            "lower",
            "withdraw",
            "settle",
            "complete",
        )
    ] == [
        "관측",
        "접근",
        "삽입",
        "들기",
        "후진",
        "운반",
        "하역",
        "후진",
        "하역",
        "하역",
    ]


def test_world_opening_projection_moves_with_base_and_skips_behind_camera():
    mount = {
        "translation_m": [0, 0, 0],
        "rotation": [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
    }
    intrinsics = {
        "width": 640,
        "height": 480,
        "fx": 400,
        "fy": 400,
        "cx": 320,
        "cy": 240,
        "frame_id": "camera_optical_frame",
    }
    corners = [[2, -0.1, 0.1], [2, 0.1, 0.1], [2, 0.1, -0.1], [2, -0.1, -0.1]]
    front = project_world_opening(corners, [0, 0, 0], [1, 0, 0, 0], mount, intrinsics)
    assert front is not None and len(front) == 4
    assert (
        project_world_opening(corners, [3, 0, 0], [1, 0, 0, 0], mount, intrinsics)
        is None
    )
