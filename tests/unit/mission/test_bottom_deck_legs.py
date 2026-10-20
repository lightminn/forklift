"""The bottom-deck side stands (plan 2026-10-19 task 4): where the truck stands to see each evidence face."""

import math

import numpy as np
import pytest

from forklift_core.mission import bottom_deck_legs as L
from forklift_core.perception.bottom_deck_evidence import BottomDeckTarget, stand_angle_deg

MOUNT_M, REAR_M = 0.619, 0.37


def _target(model_yaw):
    return BottomDeckTarget((2.0, -1.0), model_yaw, spec=None)  # spec is not read by stand_angle_deg


@pytest.mark.parametrize("insertion_yaw", [0.0, 0.7, -2.5])
def test_x_hypothesis_stands_see_both_sides_at_35_deg_from_1_35_m_on_the_truck_side(insertion_yaw):
    model_yaw = L.model_yaw(insertion_yaw, "x")
    assert model_yaw == pytest.approx(insertion_yaw)
    e = np.array([math.cos(insertion_yaw), math.sin(insertion_yaw)])
    for face in L.FACES["x"]:
        s = L.stand((2.0, -1.0), insertion_yaw, "x", face, mount_x_m=MOUNT_M, rear_axle_offset_m=REAR_M)
        cam = np.array(s.camera_xy)
        assert np.linalg.norm(cam - [2.0, -1.0]) == pytest.approx(1.35)
        assert stand_angle_deg(cam, _target(model_yaw), face) == pytest.approx(35.0)
        assert (cam - [2.0, -1.0]) @ e < 0  # tilted toward the entry face, where the truck comes from
        heading = np.array([math.cos(s.rear[2]), math.sin(s.rear[2])])
        to_centre = (np.array([2.0, -1.0]) - cam) / 1.35
        assert heading @ to_centre == pytest.approx(1.0)  # looking at the centre
        base = np.array(s.rear[:2]) + REAR_M * heading
        assert np.linalg.norm(base + MOUNT_M * heading - cam) == pytest.approx(0.0, abs=1e-12)


def test_y_hypothesis_stands_face_the_back_then_the_entry_face_head_on():
    insertion_yaw = 0.4
    model_yaw = L.model_yaw(insertion_yaw, "y")
    assert model_yaw == pytest.approx(insertion_yaw - math.pi / 2)
    e = np.array([math.cos(insertion_yaw), math.sin(insertion_yaw)])
    back, entry = L.FACES["y"]
    cams = {}
    for face in (back, entry):
        s = L.stand((2.0, -1.0), insertion_yaw, "y", face, mount_x_m=MOUNT_M, rear_axle_offset_m=REAR_M)
        cams[face] = np.array(s.camera_xy)
        assert stand_angle_deg(cams[face], _target(model_yaw), face) == pytest.approx(0.0, abs=1e-6)
    assert (cams[back] - [2.0, -1.0]) @ e == pytest.approx(1.35)    # beyond the pallet
    assert (cams[entry] - [2.0, -1.0]) @ e == pytest.approx(-1.35)  # where the truck enters


def test_x_stands_go_nearer_first_and_y_back_first():
    order = L.ordered_faces("x", (2.0, -1.0), 0.0, rear_xy=(0.0, -3.0))
    near = min(L.FACES["x"], key=lambda f: np.linalg.norm(np.array(L.stand(
        (2.0, -1.0), 0.0, "x", f, mount_x_m=MOUNT_M, rear_axle_offset_m=REAR_M).rear[:2]) - [0.0, -3.0]))
    assert order[0] == near and sorted(order) == [-1, 1]
    assert L.ordered_faces("y", (2.0, -1.0), 0.0, rear_xy=(9.0, 9.0)) == L.FACES["y"]  # back first, always


def test_the_retreat_point_is_behind_the_stand_along_its_heading():
    assert L.retreat_pose((1.0, 2.0, math.pi / 2), 0.3) == pytest.approx((1.0, 1.7, math.pi / 2))
