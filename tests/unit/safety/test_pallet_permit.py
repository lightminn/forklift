"""The teleop permit sees the pallets (persistent console plan D10, 4th ed.): each target pallet is a keep-out zone
(its outline grown by its error margin and the bundle's envelope); an arc is allowed up to the first sample whose
outline meets a zone, judged by area (separating axes), the gap between samples covered by a pad; inside a zone
every arc is 0."""

import math

import pytest

from forklift_core.control.drive_permission import StoppingModel
from forklift_core.planning.geometry import Footprint
from forklift_core.safety.pallet_permit import PalletPermit, Zone, first_contact_m, obb_overlap

STOP = StoppingModel(latency_s=0.15, decel_mps2=1.5, margin_m=0.05)
BODY = Footprint(front_m=0.95, rear_m=0.17, half_width_m=0.36)  # one part at the rear axle


def zone_ahead(gap_m, *, length=1.2, width=0.8, grow=0.0):
    """A zone whose near face is gap_m ahead of the body's front at the origin, facing it."""
    return Zone("P-1", 0.95 + gap_m + length / 2, 0.0, length + 2 * grow, width + 2 * grow, 0.0)


def test_two_boxes_overlap_by_separating_axes():
    import numpy as np

    c = np.array([[0.0, 0.0], [2.0, 0.0], [0.0, 0.0]])
    yaw = np.array([0.0, 0.0, math.pi / 4])
    out = obb_overlap(c, (0.5, 0.5), yaw, (0.0, 0.0), (0.5, 0.5), 0.0)
    assert out.tolist() == [True, False, True]


def test_a_straight_arc_stops_before_the_zone_and_a_turn_away_is_free():
    s = first_contact_m((0.0, 0.0, 0.0), 0.0, 1, 2.0, 0.02, BODY, [zone_ahead(0.60)])
    assert 0.55 <= s <= 0.60
    assert first_contact_m((0.0, 0.0, 0.0), 0.0, -1, 2.0, 0.02, BODY, [zone_ahead(0.60)]) == math.inf


def test_a_body_that_could_hold_the_whole_pallet_still_meets_it():
    """Codex D10 round 2 P2-3: the body (1.12 x 0.72 m) holds a T11 (0.66 m) square whole; area, not boundary."""
    inside = Zone("T", 0.40, 0.0, 0.30, 0.30, 0.0)
    assert first_contact_m((0.0, 0.0, 0.0), 0.0, 1, 1.0, 0.02, BODY, [inside]) == 0.0


def test_the_pad_covers_the_move_between_samples():
    """Codex D10 impl round 1 P3: a turning front corner meets a point only between two samples; no sample's box
    holds it, so only the pad stops the arc before it."""
    import numpy as np

    from forklift_core.control.drive_permission import arc_poses
    from forklift_core.safety.pallet_permit import _meets, _part_boxes, first_contact

    kappa, step = 0.45, 0.02
    poses, s = arc_poses((0.0, 0.0, 0.0), kappa, 1, 1.0, step)
    mid = arc_poses((0.0, 0.0, 0.0), kappa, 1, (s[10] + s[11]) / 2, 1.0)[0][-1]
    x, y, t = mid  # the outer (right) front corner at the mid pose
    corner = (x + 0.95 * math.cos(t) + 0.36 * math.sin(t), y + 0.95 * math.sin(t) - 0.36 * math.cos(t))
    dot = Zone("T", corner[0], corner[1], 0.0002, 0.0002, 0.0)
    assert not _meets(_part_boxes(poses, BODY), dot, 0.0).any()  # no sample meets it
    assert _meets(_part_boxes(np.array([mid]), BODY), dot, 0.0)[0]  # the motion does
    s_stop, zone = first_contact((0.0, 0.0, 0.0), kappa, 1, 1.0, step, BODY, [dot])
    assert zone == "T" and s_stop <= s[10]


def test_inside_a_zone_every_arc_is_zero():
    permit = PalletPermit(STOP, step_m=0.02)
    zones = [zone_ahead(-0.05)]  # the body's front already 5 cm into it
    for direction in (1, -1):
        speed, reason = permit.limit(zones, ("rev", 1), (0.0, 0.0, 0.0), [-0.05, 0.0, 0.05], direction, 0.5,
                                     BODY, key=("snap", (0.0, 0.0, 0.0)))
        assert (speed, reason) == (0.0, "pallet_near")


def test_the_cache_keeps_the_directions_apart():
    """Codex D10 round 1 P1: a reverse result was reused for the forward arcs at a direction change."""
    permit = PalletPermit(STOP, step_m=0.02)
    zones = [zone_ahead(0.06)]
    rev = permit.limit(zones, ("rev", 1), (0.0, 0.0, 0.0), [0.0], -1, 0.5, BODY, key=("snap", (0, 0, 0)))
    fwd = permit.limit(zones, ("rev", 1), (0.0, 0.0, 0.0), [0.0], 1, 0.5, BODY, key=("snap", (0, 0, 0)))
    assert rev == (math.inf, "ok") and fwd[0] == 0.0 and fwd[1] == "pallet:P-1"


def test_a_correction_or_targets_change_installs_the_middle_arcs_again():
    """Codex D10 round 2 P1-2 / impl round 1 P3: the correction moved the pose with no driving and no revision change
    (or the targets changed with no pose change); the middle arcs' installed distances must be made again, from the
    pose now (the end arcs are judged from it every call anyway)."""
    from forklift_core.safety.pallet_permit import first_contact

    curv = [-0.45, 0.0, 0.45]
    ahead = [Zone("P-9", 1.6, 0.0, 0.3, 0.3, 0.0)]
    permit = PalletPermit(STOP, step_m=0.02)
    length = permit._length_m(0.5)

    def middle():
        return permit._cache[1]["arcs"][1][1]

    permit.limit(ahead, ("rev", 1), (0.0, 0.0, 0.0), curv, 1, 0.5, BODY, key=("snap", (-0.12, 0.0, 0.0)))
    assert middle() == first_contact((0.0, 0.0, 0.0), 0.0, 1, length, 0.02, BODY, ahead)[0]
    permit.limit(ahead, ("rev", 1), (0.3, 0.0, 0.0), curv, 1, 0.5, BODY, key=("snap", (0.12, 0.0, 0.0)))
    assert middle() == first_contact((0.3, 0.0, 0.0), 0.0, 1, length, 0.02, BODY, ahead)[0]  # the new correction
    closer = [Zone("P-9", 1.45, 0.0, 0.3, 0.3, 0.0)]
    permit.limit(closer, ("rev", 2), (0.3, 0.0, 0.0), curv, 1, 0.5, BODY, key=("snap", (0.12, 0.0, 0.0)))
    assert middle() == first_contact((0.3, 0.0, 0.0), 0.0, 1, length, 0.02, BODY, closer)[0]  # the new revision


def test_the_middle_arcs_use_the_installed_distance_less_the_driven():
    permit = PalletPermit(STOP, step_m=0.02)
    zones = [Zone("P-9", 1.5, 0.0, 0.2, 0.2, 0.0)]  # straight ahead: only the middle arc meets it
    curv = [-0.3, 0.0, 0.3]
    first = permit.limit(zones, ("rev", 1), (0.0, 0.0, 0.0), curv, 1, 0.5, BODY, key=("snap", (0, 0, 0)))
    permit.advance(0.2)
    second = permit.limit(zones, ("rev", 1), (0.2, 0.0, 0.0), curv, 1, 0.5, BODY, key=("snap", (0, 0, 0)))
    assert second[0] < first[0] and second[1] == first[1] == "pallet:P-9"


def test_the_zones_follow_the_targets_basis():
    from forklift_core.mission.targets import Pose, TargetsBook
    from forklift_core.planning.geometry import Rectangle
    from forklift_core.safety.pallet_permit import ENVELOPE_M, PRIOR_POSE_MARGIN_M, RECOGNISED_MARGIN_M, zones_from

    tb = TargetsBook()
    tb.add_pallet("P-1", "epal6", Pose(3.2, 1.0, 0.0), depth_m=1.2, width_m=0.8)
    zone = Rectangle(3.1, 0.65, 2.0 + 1.44, 2.7 + 1.44, 0.0)
    (z,) = zones_from(tb, zone)  # prior: the pickup zone, not the registered (scene) place
    assert (z.x_m, z.y_m) == (3.1, 0.65)
    assert z.length_m == pytest.approx(zone.length_m + 2 * (PRIOR_POSE_MARGIN_M + ENVELOPE_M))
    tb.update_pose("P-1", Pose(3.18, 1.03, 0.02), basis="recognised", uncertainty_m=RECOGNISED_MARGIN_M)
    (z,) = zones_from(tb, zone)
    assert (z.x_m, z.y_m, z.yaw_rad) == (3.18, 1.03, 0.02)
    assert (z.length_m, z.width_m) == pytest.approx((1.2 + 2 * (RECOGNISED_MARGIN_M + ENVELOPE_M),
                                                     0.8 + 2 * (RECOGNISED_MARGIN_M + ENVELOPE_M)))
    tb.carried("P-1")
    assert zones_from(tb, zone) == []  # on the forks: in the loaded outline


def test_the_preparations_pallet_obstacle_follows_the_basis():
    """Plan D4b item 1: the next preparation's pallet obstacle (one function for live_prepare and its start check):
    prior -> the pickup zone itself; recognised or delivered -> the outline grown by its uncertainty."""
    from forklift_core.mission.targets import Pose, TargetsBook
    from forklift_core.planning.geometry import Rectangle
    from forklift_core.safety.pallet_permit import RECOGNISED_MARGIN_M, preparation_obstacle

    tb = TargetsBook()
    tb.add_pallet("P-1", "epal6", Pose(3.2, 1.0, 0.0), depth_m=1.2, width_m=0.8)
    zone = Rectangle(3.1, 0.65, 3.44, 4.14, 0.0)
    assert preparation_obstacle(tb.pallets["P-1"], zone) == zone
    tb.update_pose("P-1", Pose(3.18, 1.03, 0.02), basis="recognised", uncertainty_m=0.15)
    r = preparation_obstacle(tb.pallets["P-1"], zone)
    assert (r.x_m, r.y_m, r.yaw_rad) == (3.18, 1.03, 0.02) and (r.length_m, r.width_m) == pytest.approx((1.5, 1.1))
    tb.carried("P-1")
    tb.put_down("P-1", Pose(-5.0, -2.0, 1.57))  # delivered, no uncertainty recorded: the recognised margin
    r = preparation_obstacle(tb.pallets["P-1"], zone, pose=Pose(-5.1, -2.0, 1.57))  # a carried pose replaces it
    assert (r.x_m, r.length_m) == pytest.approx((-5.1, 1.2 + 2 * RECOGNISED_MARGIN_M))
    tb.carried("P-1")
    assert preparation_obstacle(tb.pallets["P-1"], zone) is None
