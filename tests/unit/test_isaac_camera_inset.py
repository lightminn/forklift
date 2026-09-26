"""The video's robot-camera inset: geometry, labels and composition, on CPU."""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.pocket_observation import Pocket, PocketObservation
from forklift_core.planning.pallet_mission import PalletSite
from forklift_core.sensors.rgbd import PinholeIntrinsics

SCRIPT = Path(__file__).resolve().parents[2] / "sim/isaac/camera_inset.py"
spec = importlib.util.spec_from_file_location("camera_inset_under_test", SCRIPT)
inset = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = inset
spec.loader.exec_module(inset)

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
OBSERVATION = PocketObservation(
    stamp_ns=1,
    clock_domain="synthetic",
    frame_id="base_link",
    source_provenance="synthetic",
    status="valid",
    left=Pocket((3.0, 0.2, 0.06), 0.23, 0.08),
    right=Pocket((3.0, -0.2, 0.06), 0.23, 0.08),
    insertion_yaw_rad=0.0,
    position_sigma_m=None,
    yaw_sigma_rad=None,
    reason=None,
)
ORIGIN = (0.0, 0.0, 0.0)


def estimate(pallet_yaw_rad=0.0):
    return inset.InsetEstimate(
        observation=OBSERVATION,
        capture_pose=ORIGIN,
        pallet_site=PalletSite(3.3, 0.0, pallet_yaw_rad),
        intrinsics=INTRINSICS,
        base_from_optical=BASE_FROM_OPTICAL,
        attempt_number=2,
    )


def yellow_pixels(image):
    target = np.array(inset.POCKET_COLOUR)
    return int(np.sum(np.all(np.abs(image.astype(int) - target) < 40, axis=-1)))


def test_transfer_is_identity_when_the_robot_has_not_moved():
    assert inset.transfer_point((3.0, 0.2, 0.1), ORIGIN, ORIGIN) == pytest.approx(
        (3.0, 0.2, 0.1)
    )


def test_driving_forward_brings_the_pallet_closer_by_the_same_distance():
    moved = inset.transfer_point((3.0, 0.2, 0.1), ORIGIN, (1.0, 0.0, 0.0))
    assert moved == pytest.approx((2.0, 0.2, 0.1))


def test_turning_left_puts_a_point_that_was_ahead_on_the_right():
    moved = inset.transfer_point((3.0, 0.0, 0.0), ORIGIN, (0.0, 0.0, math.pi / 2))
    assert moved == pytest.approx((0.0, -3.0, 0.0), abs=1e-12)


@pytest.mark.parametrize(
    "robot,pallet,expected",
    [
        (0.0, 0.2, 0.2),  # pallet axis left of heading -> turn left (positive)
        (0.0, -0.19, -0.19),  # seed-0 style skew -> turn right
        (math.radians(-170), math.radians(170), math.radians(-20)),  # wraps
    ],
)
def test_required_turn_is_signed_and_wrapped(robot, pallet, expected):
    assert inset.required_turn_rad(robot, pallet) == pytest.approx(expected)


def test_a_pallet_straight_ahead_projects_symmetrically_about_the_centre():
    outlines = inset.projected_openings(estimate(), ORIGIN)
    assert len(outlines) == 2
    centres = [np.mean([u for u, _ in outline]) for outline in outlines]
    assert np.mean(centres) == pytest.approx(INTRINSICS.cx, abs=1e-6)


def test_stepping_left_shifts_the_carried_estimate_right_in_the_image():
    before = inset.projected_openings(estimate(), ORIGIN)
    after = inset.projected_openings(estimate(), (0.0, 0.3, 0.0))
    shift = np.mean([u for o in after for u, _ in o]) - np.mean(
        [u for o in before for u, _ in o]
    )
    assert shift > 20


def test_the_estimate_is_not_projected_once_it_is_behind_the_camera():
    assert inset.projected_openings(estimate(), (3.5, 0.0, 0.0)) == []


def test_distance_reads_from_the_fork_tip_and_goes_negative_when_inserted():
    assert inset.pallet_front_distance_m(estimate(), ORIGIN) - 0.95 == pytest.approx(
        2.05
    )
    assert inset.pallet_front_distance_m(estimate(), (2.4, 0.0, 0.0)) - 0.95 < 0


def render(phase, est, camera=None):
    if camera is None:
        camera = np.full((480, 640, 3), 90, np.uint8)
    return inset.render_inset(
        camera,
        phase=phase,
        current_pose=ORIGIN,
        estimate=est,
        status="팔레트 탐색 중",
        fork_tip_x_m=0.95,
    )


def test_inset_is_the_camera_picture_with_a_panel_below_it():
    width, height = inset.INSET_SIZE
    assert render("approach", estimate()).shape == (
        height + inset.PANEL_HEIGHT,
        width,
        3,
    )


def test_pockets_are_drawn_while_the_pallet_is_on_the_floor_only():
    height = inset.INSET_SIZE[1]
    picture = slice(30, height)  # below the header, above the panel
    assert yellow_pixels(render("approach", estimate())[picture]) > 0
    # Once lifted the pallet rides on the forks; the floor estimate is stale.
    assert yellow_pixels(render("lift", estimate())[picture]) == 0


def test_nothing_is_drawn_before_a_detection():
    height = inset.INSET_SIZE[1]
    assert yellow_pixels(render("observe", None)[30:height]) == 0


def test_a_missing_camera_frame_still_renders_a_panel():
    width, height = inset.INSET_SIZE
    image = inset.render_inset(
        None,
        phase="observe",
        current_pose=ORIGIN,
        estimate=None,
        status="팔레트 탐색 중",
        fork_tip_x_m=0.95,
    )
    assert image.shape == (height + inset.PANEL_HEIGHT, width, 3)


def test_the_turn_gauge_marker_sits_on_the_side_of_the_turn():
    width, height = inset.INSET_SIZE
    panel = slice(height, None)
    left_turn = render("approach", estimate(pallet_yaw_rad=0.3))[panel]
    right_turn = render("approach", estimate(pallet_yaw_rad=-0.3))[panel]
    # Only the marker is amber in the panel (the aligned marker would be green).
    left_cols = np.nonzero(
        np.all(np.abs(left_turn.astype(int) - inset.POCKET_COLOUR) < 40, axis=-1)
    )[1]
    right_cols = np.nonzero(
        np.all(np.abs(right_turn.astype(int) - inset.POCKET_COLOUR) < 40, axis=-1)
    )[1]
    assert left_cols.mean() < width / 2 < right_cols.mean()


def test_compose_places_the_inset_top_left_without_touching_the_input():
    overview = np.zeros((720, 1280, 3), np.uint8)
    tile = np.full((50, 60, 3), 200, np.uint8)
    frame = inset.compose_frame(overview, tile)
    assert overview.max() == 0
    top = left = inset.INSET_MARGIN
    assert (frame[top : top + 50, left : left + 60] == 200).all()
    assert frame[:, 400:].max() == 0  # the working area to the right is untouched
    assert frame.shape == (720, 1280, 3)


def test_the_rejection_reason_goes_to_the_panel_not_the_header():
    width, height = inset.INSET_SIZE
    with_detail = inset.render_inset(
        None,
        phase="observe",
        current_pose=ORIGIN,
        estimate=None,
        status="인식 실패 #1 · 재관측 이동",
        detail="사유: no_opening_pattern",
        fork_tip_x_m=0.95,
    )
    without = inset.render_inset(
        None,
        phase="observe",
        current_pose=ORIGIN,
        estimate=None,
        status="인식 실패 #1 · 재관측 이동",
        fork_tip_x_m=0.95,
    )
    assert (with_detail[:height] == without[:height]).all()
    assert (with_detail[height:] != without[height:]).any()
