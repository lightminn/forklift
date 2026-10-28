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


# --- docs/plans/2026-10-28-far-target-binding.md F1: the far capture's expected site from the adoption rectangle ---

def test_the_far_site_is_the_circle_around_the_rectangle_plus_margin():
    zone = az.AdoptionZone(2.6, 3.6, -0.2, 1.5, "session_zone", yaw_centre_rad=0.0, yaw_half_rad=0.2094)
    site = az.far_site(zone, 0.10, base_xy=(0.0, 0.0), base_yaw=0.0)
    assert (site.x_m, site.y_m) == pytest.approx((3.1, 0.65))
    assert site.radius_m == pytest.approx(math.hypot(0.5 + 0.10, 0.85 + 0.10))
    assert site.axis_tolerance_rad == pytest.approx(0.2094 + math.radians(5))


def test_the_corner_cases_codex_raised_lie_inside_the_far_site():
    # a 0.5 x 0.5 zone at m 0.10: (0.349, 0.349) from the centre passes the adoption check, so it must lie in the site
    zone = az.AdoptionZone(-0.25, 0.25, -0.25, 0.25, "session_zone")
    site = az.far_site(zone, 0.10, base_xy=(0.0, 0.0), base_yaw=0.0)
    assert site.radius_m == pytest.approx(0.495, abs=1e-3) and math.hypot(0.349, 0.349) <= site.radius_m
    assert az.check((0.349, 0.349), zone, 0.10, stage="far")["inside"]
    repick = az.AdoptionZone(-0.1, 0.1, -0.1, 0.1, "put_down", yaw_centre_rad=0.0, yaw_half_rad=az.REPICK_YAW_HALF_RAD)
    assert az.far_site(repick, 0.10, base_xy=(0.0, 0.0), base_yaw=0.0).radius_m == pytest.approx(0.283, abs=1e-3)


def test_the_far_site_is_placed_in_the_capture_base():
    zone = az.AdoptionZone(4.9, 5.1, 1.9, 2.1, "put_down", yaw_centre_rad=math.pi / 2, yaw_half_rad=0.26)
    site = az.far_site(zone, 0.10, base_xy=(5.0, 0.0), base_yaw=math.pi / 2)  # the base faces +y, 2 m short
    assert (site.x_m, site.y_m, site.yaw_rad) == pytest.approx((2.0, 0.0, 0.0), abs=1e-9)


def test_the_relative_yaw_wraps_at_pi():
    zone = az.AdoptionZone(-0.1, 0.1, -0.1, 0.1, "put_down", yaw_centre_rad=math.pi - 0.01, yaw_half_rad=0.26)
    site = az.far_site(zone, 0.10, base_xy=(-2.0, 0.0), base_yaw=-math.pi + 0.01)
    assert abs(site.yaw_rad) == pytest.approx(0.02, abs=1e-9)


def test_a_zone_without_a_yaw_range_takes_the_default_axis_tolerance():
    zone = az.AdoptionZone(-0.25, 0.25, -0.25, 0.25, "bay")
    assert az.far_site(zone, 0.10, base_xy=(-3.0, 0.0), base_yaw=0.0).axis_tolerance_rad == pytest.approx(
        az.FAR_AXIS_DEFAULT_RAD)


def test_a_put_down_square_carries_the_put_down_heading():
    tb, Pose = book()
    tb.carried("P-1")
    tb.put_down("P-1", Pose(3.0, -8.946, 0.3), uncertainty_m=0.05)
    z = az.mission_zone(tb.pallets["P-1"], ZONE)
    assert z.yaw_centre_rad == pytest.approx(0.3) and z.yaw_half_rad == pytest.approx(az.REPICK_YAW_HALF_RAD)


@pytest.mark.parametrize("half", [-0.1, float("nan")])
def test_a_broken_yaw_range_is_an_error(half):
    with pytest.raises(ValueError):
        az.AdoptionZone(0.0, 1.0, 0.0, 1.0, "bay", yaw_half_rad=half)


def test_the_reasons():
    assert az.OUTSIDE == "adoption_outside_zone" and az.UNDEFINED == "adoption_zone_undefined"
