import math

import pytest

from forklift_core.mission import adoption_zone as az

ZONE = az.AdoptionZone(2.6, 3.6, -0.2, 1.5, "scene_layout")  # 1.0 x 1.7 m, circumscribed radius 0.986 m


@pytest.mark.parametrize("xy, inside", [
    ((3.1, 0.65), True),           # centre
    ((3.6, 1.5), True),            # on the rectangle's corner
    ((3.85, 0.65), True),          # on the grown edge (margin 0.25)
    ((3.86, 0.65), False),         # just past it
    ((2.34, 0.65), False),
    ((3.1, -0.46), False),
])
def test_the_rectangle_grown_by_the_margin(xy, inside):
    r = az.check(xy, ZONE, 0.25, stage="observe")
    assert r["inside"] is inside
    assert r["stage"] == "observe" and r["zone"] == [2.6, 3.6, -0.2, 1.5] and r["margin_m"] == 0.25


def test_inside_the_circumscribed_circle_is_not_enough():
    # A corner point of the circle: 0.95 m from the centre (inside 0.986) but 0.32 m past x_max -- refused.
    xy = (3.1 + 0.95 * math.cos(0.0), 0.65)
    assert math.hypot(xy[0] - 3.1, xy[1] - 0.65) < math.hypot(0.5, 0.85)
    assert not az.check(xy, ZONE, 0.25, stage="observe")["inside"]


def test_the_signed_distance_is_recorded():
    assert az.check((3.1, 0.65), ZONE, 0.25, stage="observe")["edge_m"] == pytest.approx(-0.5)
    assert az.check((3.9, 0.65), ZONE, 0.25, stage="observe")["edge_m"] == pytest.approx(0.3)


@pytest.mark.parametrize("args", [(3.0, 2.0, 0.0, 1.0), (0.0, 1.0, 0.0, float("nan")), (0.0, float("inf"), 0.0, 1.0)])
def test_a_broken_rectangle_is_an_error(args):
    with pytest.raises(ValueError):
        az.AdoptionZone(*args, "scene_layout")


@pytest.mark.parametrize("xy, margin", [((float("nan"), 0.0), 0.25), ((3.1, 0.65), -0.1), ((3.1, 0.65), float("nan"))])
def test_a_broken_estimate_or_margin_is_an_error(xy, margin):
    with pytest.raises(ValueError):
        az.check(xy, ZONE, margin, stage="observe")


def test_the_mission_rectangle_at_the_first_place_is_the_zone_itself():
    assert az.mission_zone(at_first=True, first_zone=ZONE, put_down_xy=(9.0, 9.0)) is ZONE


def test_a_re_pick_takes_a_square_about_the_put_down_centre():
    z = az.mission_zone(at_first=False, first_zone=ZONE, put_down_xy=(3.0, -8.946))
    assert (z.x_min_m, z.x_max_m, z.y_min_m, z.y_max_m) == pytest.approx((2.9, 3.1, -9.046, -8.846))
    assert z.source == "put_down"


def test_an_unknown_centre_has_no_rectangle():
    assert az.mission_zone(at_first=False, first_zone=ZONE, put_down_xy=None) is None


def test_the_put_down_centre_is_the_verified_destination_not_the_pose_a_later_observation_rewrote():
    from forklift_core.mission.targets import Pallet, Pose

    destinations = {"D-2": Pose(3.0, -8.946, 0.0)}
    delivered = Pallet("P-1", "epal6", Pose(3.07, -8.90, 0.0), state="delivered", basis="recognised", at="D-2")
    assert az.put_down_centre(delivered, destinations) == (3.0, -8.946)
    # moved by other means (at cleared), not delivered, or a destination that no longer exists: unknown
    assert az.put_down_centre(Pallet("P-1", "epal6", Pose(3.0, -8.9, 0.0), state="delivered", at=None),
                              destinations) is None
    assert az.put_down_centre(Pallet("P-1", "epal6", Pose(1.0, 0.0, 0.0), at="D-2"), destinations) is None
    assert az.put_down_centre(Pallet("P-1", "epal6", None, state="delivered", at="D-9"), destinations) is None


def test_the_reasons():
    assert az.OUTSIDE == "adoption_outside_zone" and az.UNDEFINED == "adoption_zone_undefined"
