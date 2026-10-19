"""Persistent console plan D3 (3rd edition): the submit refusals already_there and destination_occupied, judged on
the targets book -- a verified place, not a distance, and D10's outlines with an upper bound for unknown sizes."""

import pytest

from forklift_core.mission.targets import Pose, TargetsBook

EPAL = (0.60, 0.80)
T11 = (0.66, 0.66)


def scene(**sizes):
    b = TargetsBook()
    b.add_shape("epal6", *EPAL)
    b.add_shape("t11_06", *T11)
    b.add_pallet("P-1", "epal6", Pose(1.0, 0.0, 0.0), at="D-0", **sizes)
    b.add_destination("D-0", Pose(1.0, 0.0, 0.0))
    b.add_destination("D-1", Pose(9.0, 0.0, 0.0))
    return b


def test_the_first_place_is_verified_d0_so_sending_it_there_is_already_there():
    b = scene()
    assert b.check_submit("P-1", "D-0", b.revision) == "already_there"
    assert b.check_submit("P-1", "D-1", b.revision) is None


def test_lifting_forgets_the_place_and_only_a_passed_delivery_sets_a_new_one():
    b = scene()
    b.carried("P-1")
    assert b.pallets["P-1"].at is None
    b.put_down("P-1", Pose(9.0, 0.0, 0.0))  # the destination's estimate, copied: no proof it got there
    assert b.check_submit("P-1", "D-1", b.revision) is None  # a failed delivery may be retried
    rev = b.revision
    assert b.verify_at("P-1", "D-1") and b.revision == rev + 1
    assert not b.verify_at("P-1", "D-1") and b.revision == rev + 1  # the same place again: no change
    assert b.check_submit("P-1", "D-1", b.revision) == "already_there"
    assert b.check_submit("P-1", "D-0", b.revision) is None


def test_a_correction_keeps_the_verified_place():
    b = scene()
    b.set_correction_version(2, reexpress=lambda p: Pose(p.x_m + 0.3, p.y_m, p.yaw_rad))
    assert b.pallets["P-1"].at == "D-0"


def test_another_pallet_in_the_destination_occupies_it():
    b = scene()
    b.add_pallet("P-2", "epal6", Pose(9.1, 0.0, 0.0), depth_m=EPAL[0], width_m=EPAL[1])
    b.put_down("P-2", Pose(9.1, 0.0, 0.0), uncertainty_m=0.0)
    assert b.check_submit("P-1", "D-1", b.revision) == "destination_occupied"


def test_a_pallet_clear_of_the_destination_does_not_occupy_it():
    b = scene()
    b.add_pallet("P-2", "epal6", Pose(9.0, 2.0, 0.0), depth_m=EPAL[0], width_m=EPAL[1])
    b.put_down("P-2", Pose(9.0, 2.0, 0.0), uncertainty_m=0.0)
    assert b.check_submit("P-1", "D-1", b.revision) is None


def test_the_other_pallets_uncertainty_grows_its_outline():
    b = scene()
    b.add_pallet("P-2", "epal6", Pose(9.0, 0.85, 0.0), depth_m=EPAL[0], width_m=EPAL[1])
    b.put_down("P-2", Pose(9.0, 0.85, 0.0), uncertainty_m=0.0)
    assert b.check_submit("P-1", "D-1", b.revision) is None  # 0.05 m apart
    b.put_down("P-2", Pose(9.0, 0.85, 0.0), uncertainty_m=0.21)
    assert b.check_submit("P-1", "D-1", b.revision) == "destination_occupied"


def test_a_rotated_pallet_is_judged_by_its_rotated_outline():
    b = scene()
    # 0.75 m off sideways: turned 90 deg its half depth 0.30 reaches across (0.40 + 0.30 < 0.75), aligned 0.40 does
    b.add_pallet("P-2", "epal6", Pose(9.0, 0.75, 1.5707963), depth_m=EPAL[0], width_m=EPAL[1])
    b.put_down("P-2", Pose(9.0, 0.75, 1.5707963), uncertainty_m=0.0)
    assert b.check_submit("P-1", "D-1", b.revision) is None  # 0.40 + 0.30 = 0.70 < 0.75
    b.put_down("P-2", Pose(9.0, 0.75, 0.0), uncertainty_m=0.0)
    assert b.check_submit("P-1", "D-1", b.revision) == "destination_occupied"  # 0.40 + 0.40 = 0.80 > 0.75


def test_an_unknown_size_takes_the_largest_allowed_shape():
    """Codex D3 round 1 P2-2: the EPAL's own size let a T11 overlap pass (0.62 m apart: 2 cm gap vs 1 cm overlap)."""
    b = scene(depth_m=EPAL[0], width_m=EPAL[1])
    b.add_pallet("P-2", "t11_06", Pose(9.62, 0.0, 0.0))  # no size registered
    b.put_down("P-2", Pose(9.62, 0.0, 0.0), uncertainty_m=0.0)
    assert b.check_submit("P-1", "D-1", b.revision) == "destination_occupied"


def test_a_pallet_without_a_size_needs_an_allowed_shape():
    """Codex D3 round 2 P2: the upper bound needs a non-empty shape set covering the kind."""
    b = scene()
    with pytest.raises(ValueError):
        b.add_pallet("P-9", "mystery", Pose(0.0, 5.0, 0.0))
    empty = TargetsBook()
    empty.add_pallet("P-1", "epal6", Pose(1.0, 0.0, 0.0))
    empty.add_destination("D-1", Pose(9.0, 0.0, 0.0))
    empty.add_pallet("P-2", "epal6", Pose(5.0, 5.0, 0.0))
    assert empty.check_submit("P-1", "D-1", empty.revision) == "invalid"  # no shape set: no occupancy verdict


def test_a_carried_pallet_does_not_occupy_anything():
    b = scene()
    b.add_pallet("P-2", "epal6", Pose(9.0, 0.0, 0.0), depth_m=EPAL[0], width_m=EPAL[1])
    b.carried("P-2")
    assert b.check_submit("P-1", "D-1", b.revision) is None


def test_the_order_is_invalid_then_already_there_then_occupied():
    b = scene()
    b.add_pallet("P-2", "epal6", Pose(1.0, 0.0, 0.0), depth_m=EPAL[0], width_m=EPAL[1])
    assert b.check_submit("P-1", "D-0", b.revision) == "already_there"  # before the occupancy by P-2
    assert b.check_submit("P-1", "D-0", b.revision - 1) == "stale_targets"
