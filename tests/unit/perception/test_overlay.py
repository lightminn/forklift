import math

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.overlay import (
    ESTIMATE_COLOUR,
    TRUTH_COLOUR,
    draw_scene_overlay,
    opening_corners_m,
    project_point,
)
from forklift_core.perception.pocket_observation import Pocket, PocketObservation
from forklift_core.sensors.rgbd import PinholeIntrinsics

INTRINSICS = PinholeIntrinsics(
    width=640,
    height=480,
    fx=465.741156,
    fy=465.741156,
    cx=320.0,
    cy=240.0,
    frame_id="camera_optical_frame",
)
BASE_FROM_OPTICAL = RigidTransform(
    "camera_optical_frame",
    "base_link",
    rotation_matrix_from_quaternion_xyzw([-0.5, 0.5, -0.5, 0.5]),
    [0.75, 0.0, 0.5],
)


def test_a_point_on_the_optical_axis_lands_on_the_principal_point():
    # base (2.75, 0, 0.5) is 2.0 m straight ahead of the camera at its own height
    assert project_point(
        (2.75, 0.0, 0.5), INTRINSICS, BASE_FROM_OPTICAL
    ) == pytest.approx((320.0, 240.0), abs=1e-6)


def test_a_point_left_and_below_moves_left_and_down_in_the_image():
    u, v = project_point((2.75, 0.2, 0.15), INTRINSICS, BASE_FROM_OPTICAL)
    assert u < 320.0 and v > 240.0
    # optical x = -0.2, y = 0.35, z = 2.0  ->  u = -0.2*fx/2 + 320, v = 0.35*fx/2 + 240
    assert (u, v) == pytest.approx(
        (320.0 - 0.1 * 465.741156, 240.0 + 0.175 * 465.741156), abs=1e-6
    )


def test_points_behind_the_camera_do_not_project():
    assert project_point((0.0, 0.0, 0.5), INTRINSICS, BASE_FROM_OPTICAL) is None


def test_a_point_on_the_optical_plane_does_not_project():
    # base x = 0.75 is exactly the camera plane: optical z = 0, not merely small
    assert project_point((0.75, 0.0, 0.9), INTRINSICS, BASE_FROM_OPTICAL) is None


def test_a_point_off_the_sensor_still_projects_so_partial_rectangles_clip():
    # 3 m to the left at 2 m depth: in front of the camera but outside the image
    uv = project_point((2.75, 3.0, 0.5), INTRINSICS, BASE_FROM_OPTICAL)
    assert uv is not None
    assert uv[0] == pytest.approx(320.0 - 1.5 * 465.741156, abs=1e-6)


@pytest.mark.parametrize("yaw", [0.0, 0.4, -0.4, math.pi / 2, -math.pi / 2, math.pi])
def test_opening_corners_are_top_left_top_right_bottom_right_bottom_left(yaw):
    pocket = Pocket((2.2, 0.17, 0.15), 0.24, 0.20)
    corners = opening_corners_m(pocket, yaw)
    assert len(corners) == 4
    centre = np.array(pocket.center_m)
    left_axis = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    along = [float(np.dot(np.array(c) - centre, left_axis)) for c in corners]
    up = [float(c[2] - centre[2]) for c in corners]
    # each corner is pinned: a rotated or mirrored order such as [TR, TL, BL, BR]
    # keeps the same width and height but draws the rectangle as a bow tie
    assert along == pytest.approx([0.12, -0.12, -0.12, 0.12], abs=1e-9)
    assert up == pytest.approx([0.10, 0.10, -0.10, -0.10], abs=1e-9)
    # the opening rectangle carries no depth along the insertion axis
    axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    depth = [float(np.dot(np.array(c) - centre, axis)) for c in corners]
    assert depth == pytest.approx([0.0, 0.0, 0.0, 0.0], abs=1e-9)


def test_the_overlay_marks_truth_and_estimate_in_different_colours():
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    truth = PocketObservation(
        stamp_ns=1,
        clock_domain="ros_sim",
        frame_id="base_link",
        source_provenance="synthetic_ground_truth",
        status="valid",
        left=Pocket((2.2, 0.17, 0.15), 0.24, 0.20),
        right=Pocket((2.2, -0.17, 0.15), 0.24, 0.20),
        insertion_yaw_rad=0.0,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason=None,
    )
    estimate = PocketObservation(
        stamp_ns=1,
        clock_domain="ros_sim",
        frame_id="base_link",
        source_provenance="synthetic",
        status="valid",
        # 0.5 m short: the two outlines must not overlap for the colour check
        left=Pocket((1.7, 0.17, 0.15), 0.24, 0.20),
        right=Pocket((1.7, -0.17, 0.15), 0.24, 0.20),
        insertion_yaw_rad=0.0,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason=None,
    )
    out = draw_scene_overlay(
        rgb,
        truth=truth,
        estimate=estimate,
        intrinsics=INTRINSICS,
        base_from_optical=BASE_FROM_OPTICAL,
        caption="s001 wrong_pose",
    )
    assert out.shape == rgb.shape and out.dtype == np.uint8
    assert rgb.max() == 0  # the input is not modified

    colours = {tuple(c) for c in out.reshape(-1, 3).tolist()}
    assert TRUTH_COLOUR != ESTIMATE_COLOUR
    assert TRUTH_COLOUR in colours and ESTIMATE_COLOUR in colours

    def drawn_bbox(colour):
        rows, cols = np.nonzero(np.all(out == np.array(colour, dtype=np.uint8), axis=2))
        return cols.min(), cols.max(), rows.min(), rows.max()

    def projected_bbox(observation):
        points = opening_corners_m(observation.left, 0.0) + opening_corners_m(
            observation.right, 0.0
        )
        uvs = [project_point(c, INTRINSICS, BASE_FROM_OPTICAL) for c in points]
        assert all(uv is not None for uv in uvs)
        us = [uv[0] for uv in uvs]
        vs = [uv[1] for uv in uvs]
        return min(us), max(us), min(vs), max(vs)

    # the green pixels trace the ground-truth openings and the magenta pixels the
    # estimate, so a single colour used for both (or for the caption) fails here
    for colour, observation in ((TRUTH_COLOUR, truth), (ESTIMATE_COLOUR, estimate)):
        assert drawn_bbox(colour) == pytest.approx(projected_bbox(observation), abs=2)


def test_the_overlay_still_renders_when_the_estimate_has_no_pockets():
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    truth = PocketObservation(
        stamp_ns=1,
        clock_domain="ros_sim",
        frame_id="base_link",
        source_provenance="synthetic_ground_truth",
        status="no_pallet",
        left=None,
        right=None,
        insertion_yaw_rad=None,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason="no target pallet in scene",
    )
    out = draw_scene_overlay(
        rgb,
        truth=truth,
        estimate=truth,
        intrinsics=INTRINSICS,
        base_from_optical=BASE_FROM_OPTICAL,
        caption="s012 true_negative",
    )
    assert out.shape == rgb.shape
