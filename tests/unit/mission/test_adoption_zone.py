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


def book():
    from forklift_core.mission.targets import Pose, TargetsBook

    tb = TargetsBook()
    tb.add_shape("epal6", 0.8, 0.6)
    tb.add_pallet("P-1", "epal6", Pose(3.1, 0.65, 0.0), at="D-0")
    tb.add_destination("D-2", Pose(3.0, -8.946, 0.0))
    return tb, Pose


def test_a_pallet_never_put_down_keeps_its_first_zone_whatever_its_pose_became():
    # Codex implementation round 1 P2: an observation that moves the pose 2 cm must not lose the first zone.
    tb, Pose = book()
    tb.update_pose("P-1", Pose(3.11, 0.67, 0.0), basis="recognised", uncertainty_m=0.05)
    assert az.mission_zone(tb.pallets["P-1"], ZONE) is ZONE


def test_a_put_down_pallet_takes_a_square_about_where_it_was_put_down():
    tb, Pose = book()
    tb.carried("P-1")
    tb.put_down("P-1", Pose(3.0, -8.946, 0.0), uncertainty_m=0.05)
    tb.update_pose("P-1", Pose(3.07, -8.90, 0.0))  # a re-pick's observation rewrites the pose, not the snapshot
    tb.update_destination("D-2", Pose(4.0, -8.946, 0.0))  # nor does a later estimate of the destination
    z = az.mission_zone(tb.pallets["P-1"], ZONE)
    assert (z.x_min_m, z.x_max_m, z.y_min_m, z.y_max_m) == pytest.approx((2.9, 3.1, -9.046, -8.846))
    assert z.source == "put_down"


def test_a_recovery_put_back_counts_as_a_put_down():
    tb, Pose = book()
    tb.carried("P-1")
    tb.put_down("P-1", Pose(5.0, 1.0, 0.0), state="available", uncertainty_m=0.05)
    assert az.mission_zone(tb.pallets["P-1"], ZONE).source == "put_down"


def test_the_put_down_snapshot_moves_with_a_change_of_frame():
    tb, Pose = book()
    tb.carried("P-1")
    tb.put_down("P-1", Pose(3.0, -8.946, 0.0), uncertainty_m=0.05)
    tb.set_correction_version(1, lambda p: Pose(p.x_m + 1.0, p.y_m, p.yaw_rad))
    assert tb.pallets["P-1"].put_down_pose == Pose(4.0, -8.946, 0.0)


def test_without_a_first_zone_a_pallet_never_put_down_has_no_rectangle():
    tb, _ = book()
    assert az.mission_zone(tb.pallets["P-1"], None) is None


def test_the_default_margin_follows_the_localization_path():
    # plan section 8a, lead 2026-10-28: localizing on a survey bundle 0.10 m, mapping (and reported) 0.30 m
    assert az.default_margin(localized=True) == pytest.approx(0.10)
    assert az.default_margin(localized=False) == pytest.approx(0.30)


def test_the_reasons():
    assert az.OUTSIDE == "adoption_outside_zone" and az.UNDEFINED == "adoption_zone_undefined"
