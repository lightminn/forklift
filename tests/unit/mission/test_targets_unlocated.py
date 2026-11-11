"""Sim-to-real 4d-2b-2: a pallet whose place is unknown after a field recovery -- kept as an obstacle, never a pick,
until an observation places it again or the operator says it is gone (docs/design/2026-11-11-p05-truth-audit.md 4-4)."""

import pytest

from forklift_core.mission.targets import Pose, TargetsBook
from forklift_core.safety.pallet_permit import session_pallet_props


def book():
    b = TargetsBook(shapes={"epal": (1.2, 0.8)})
    b.add_destination("D1", Pose(5.0, 0.0, 0.0))
    b.add_destination("D2", Pose(8.0, 0.0, 0.0))
    b.add_pallet("P1", "epal", Pose(1.0, 0.0, 0.0))
    b.put_down("P1", Pose(5.0, 0.0, 0.0))
    b.verify_at("P1", "D1")
    return b


def props(b):
    return [p.asset.uri for p in session_pallet_props(b, exclude_id=None, nominal={}, shape=(1.2, 0.8, 0.144),
                                                      prior_grow_m=0.1)]


def test_an_unlocated_pallet_stays_an_obstacle_and_is_never_a_pick():
    b = book()
    rev = b.revision
    assert b.unlocate("P1") and b.revision > rev
    p = b.pallets["P1"]
    assert p.state == "unlocated" and p.at is None and p.put_down_pose is None and p.pose == Pose(5.0, 0.0, 0.0)
    assert props(b) == ["session:pallet:P1"]  # where it was last known: still kept out of
    assert b.check_submit("P1", "D2", b.revision) == "invalid"
    assert b.message()["pallets"][0]["state"] == "unlocated"


def test_the_operator_clearing_it_frees_the_place():
    b = book()
    b.unlocate("P1")
    assert b.clear_unlocated("P1")
    assert b.pallets["P1"].pose is None and b.pallets["P1"].state == "unlocated" and props(b) == []
    assert b.check_submit("P1", "D2", b.revision) == "invalid"  # still not a pick: nobody knows where it is
    with pytest.raises(ValueError):
        book().clear_unlocated("P1")  # only an unlocated pallet


def test_an_observation_places_it_again():
    b = book()
    b.unlocate("P1")
    b.clear_unlocated("P1")
    assert b.update_pose("P1", Pose(3.0, 1.0, 0.1), basis="recognised", uncertainty_m=0.05)
    p = b.pallets["P1"]
    assert p.state == "available" and p.pose == Pose(3.0, 1.0, 0.1) and props(b) == ["session:pallet:P1"]
    assert b.check_submit("P1", "D2", b.revision) is None


def test_a_prior_pose_does_not_place_an_unlocated_pallet():
    b = book()
    b.unlocate("P1")
    b.update_pose("P1", Pose(4.0, 0.0, 0.0))  # a frame's re-expression, no observation
    assert b.pallets["P1"].state == "unlocated"


def test_a_carried_pallets_last_pose_moves_with_the_frame():
    from forklift_core.localization.slam_pose import compose

    b = book()
    b.pallets["P1"] = b.pallets["P1"].__class__(**{**b.pallets["P1"].__dict__, "state": "available", "at": None})
    b.carried("P1")
    assert b.pallets["P1"].last_pose == Pose(5.0, 0.0, 0.0)

    def shift(p):
        x, y, yaw = compose((1.0, 0.0, 0.0), (p.x_m, p.y_m, p.yaw_rad))
        return Pose(x, y, yaw)

    b.set_correction_version(1, reexpress=shift)  # a frame change: x + 1
    assert b.pallets["P1"].last_pose == Pose(6.0, 0.0, 0.0)
    assert b.move_last_pose("P1", shift) and b.pallets["P1"].last_pose == Pose(7.0, 0.0, 0.0)  # a hold's release
    b.unlocate("P1")
    assert b.pallets["P1"].pose == Pose(7.0, 0.0, 0.0)  # kept out where it was last known, in this frame
    assert b.move_last_pose("P1", shift) is False  # only while carried
