"""CT-2 next pick (docs/plans/2026-10-24-ct2-next-pick.md D2, V1)."""

import math
import time

import pytest

from forklift_core.mission.next_pick import PickSnapshot, recommend
from forklift_core.mission.targets import Pose, TargetsBook
from forklift_core.planning import Pose2D
from forklift_core.planning.geometry import Bounds, Rectangle
from forklift_core.planning.pallet_mission import (
    AssetSpec, PalletSite, PlacedProp, SyntheticMissionGeometry, TransportScenario, site_poses)

G = SyntheticMissionGeometry(alignment_straight_m=2.1, delivery_straight_m=1.5)
HOME = (-3.0, 0.0, 0.0)
BOUNDS = Bounds(-6.0, 6.0, -12.0, 6.0)
SHAPE = (0.6, 0.8, 0.144)


def book(pallets, destinations, *, recognised=True):
    tb = TargetsBook()
    tb.add_shape("epal6", 0.6, 0.8)
    for pid, (x, y, yaw), *at in pallets:
        tb.add_pallet(pid, "epal6", Pose(x, y, yaw), depth_m=0.6, width_m=0.8, at=at[0] if at else None)
        if recognised:
            tb.update_pose(pid, Pose(x, y, yaw), basis="recognised", uncertainty_m=0.05)
    for did, (x, y, yaw) in destinations:
        tb.add_destination(did, Pose(x, y, yaw))
    return tb


def snap(tb, *, props=(), truck=HOME, home=HOME):
    sc = TransportScenario(seed=0, start_rear=Pose2D(*HOME), pickup=PalletSite(0, 0, 0),
                           destination=PalletSite(0, 0, 0), props=tuple(props), bounds=BOUNDS)
    return PickSnapshot(tb, truck, home, sc, G, 0.10, {}, SHAPE, 0.20)


def deliver(tb, pid, did):
    d = tb.destinations[did]
    tb.carried(pid)
    tb.put_down(pid, Pose(d.x_m, d.y_m, d.yaw_rad), uncertainty_m=0.21)
    tb.verify_at(pid, did)


# a line: pallets one behind the other along +x, slots one behind the other along +x (closer than a delivery straight)
LINE_P = [("P-1", (0.8, 0.0, 0.0)), ("P-2", (1.86, 0.0, 0.0)), ("P-3", (2.92, 0.0, 0.0))]
LINE_D = [("D-1", (3.0, -9.0, 0.0)), ("D-2", (1.94, -9.0, 0.0)), ("D-3", (0.88, -9.0, 0.0))]


def test_a_line_takes_the_front_pallet_to_the_deepest_slot():
    r = recommend(snap(book(LINE_P, LINE_D)))
    assert r["reason"] == "ok" and r["missions"] == 3
    assert r["pick"] == {"pallet_id": "P-1", "destination_id": "D-1"}
    assert r["plan"] == [["P-1", "D-1"], ["P-2", "D-2"], ["P-3", "D-3"]]
    assert "pickup:P-1" in r["blocked"]["P-2"]


def test_the_shallow_slot_first_would_lose_missions():
    # P-1 already put down at the shallow D-3: the deeper slots' delivery straights pass it
    tb = book(LINE_P, LINE_D)
    deliver(tb, "P-1", "D-3")
    r = recommend(snap(tb))
    assert r["missions"] == 0 and r["reason"] == "all_blocked"
    assert "delivery:P-1" in r["blocked"]["D-1"] and "delivery:P-1" in r["blocked"]["D-2"]


def test_a_delivered_pallet_keeps_blocking_the_slot_behind_it():
    # Codex round 1 P2-2: P-1 at D-2 -- D-1's delivery straight passes it, so D-1 is never offered
    tb = book(LINE_P, LINE_D)
    deliver(tb, "P-1", "D-2")
    r = recommend(snap(tb))
    assert r["pick"] == {"pallet_id": "P-2", "destination_id": "D-3"} and r["missions"] == 1


def test_a_slot_held_by_a_pallet_opens_once_that_pallet_moves():
    # Codex round 1 P2-3 / round 2 P2-1: Q stands at D-1 (its verified place); moving Q first frees D-1 for P
    slots = [("D-1", (3.0, -4.0, 0.0)), ("D-2", (3.0, -6.0, 0.0))]
    tb = book([("P", (0.8, 2.0, 0.0)), ("Q", (3.0, -4.0, 0.0), "D-1")], slots)
    r = recommend(snap(tb))
    assert r["missions"] == 2
    assert r["plan"][0] == ["Q", "D-2"] and r["plan"][1] == ["P", "D-1"]


def test_only_the_first_mission_starts_from_the_truck():
    # Codex round 1 P2-4: the estimate starts the first mission at the truck, every later one at home
    tb = book([("P-1", (0.8, 0.0, 0.0))], [("D-1", (3.0, -9.0, 0.0))])
    truck = (-1.0, 1.0, 0.0)
    r = recommend(snap(tb, truck=truck))
    s, d = site_poses(PalletSite(0.8, 0.0, 0.0), G), site_poses(PalletSite(3.0, -9.0, 0.0), G)

    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    want = (dist(truck, (s["prealign"].x_m, s["prealign"].y_m))
            + dist((s["extracted"].x_m, s["extracted"].y_m), (d["predelivery"].x_m, d["predelivery"].y_m))
            + dist((d["withdrawn"].x_m, d["withdrawn"].y_m), HOME))
    assert r["estimate_total_m"] == pytest.approx(round(want * 1000) / 1000, abs=1e-9)
    assert r["home_assumed"] is False


def test_no_home_takes_the_truck_and_says_so():
    tb = book([("P-1", (0.8, 0.0, 0.0))], [("D-1", (3.0, -9.0, 0.0))])
    assert recommend(snap(tb, home=None))["home_assumed"] is True


def test_equal_choices_go_by_the_whole_sequence_in_order():
    # two mirror pallets, two mirror slots, the truck on the axis: every assignment costs the same
    tb = book([("P-b", (0.8, 1.5, 0.0)), ("P-a", (0.8, -1.5, 0.0))],
              [("D-b", (3.0, -6.0, 0.0)), ("D-a", (3.0, -8.0, 0.0))])
    r = recommend(snap(tb, truck=(-3.0, 0.0, 0.0), home=(-3.0, 0.0, 0.0)))
    assert r["missions"] == 2
    assert recommend(snap(tb, truck=(-3.0, 0.0, 0.0), home=(-3.0, 0.0, 0.0)))["plan"] == r["plan"]  # deterministic


def test_more_pallets_than_slots_ends_on_no_free_destination():
    tb = book([("P-1", (0.8, 2.0, 0.0)), ("P-2", (0.8, -2.0, 0.0))], [("D-1", (3.0, -9.0, 0.0))])
    r = recommend(snap(tb))
    assert r["reason"] == "ok" and r["missions"] == 1
    deliver(tb, r["pick"]["pallet_id"], "D-1")
    assert recommend(snap(tb))["reason"] == "no_free_destination"


def test_none_remaining_only_when_no_available_pallet_is_left():
    tb = book([("P-1", (0.8, 0.0, 0.0))], [("D-1", (3.0, -9.0, 0.0))])
    deliver(tb, "P-1", "D-1")
    assert recommend(snap(tb))["reason"] == "none_remaining"


def test_a_pallet_without_a_pose_is_not_done():
    # Codex round 2 P2-3: an available pallet with no pose stops the queue (all_blocked), it does not end it
    tb = book([("P-1", (0.8, 0.0, 0.0))], [("D-1", (3.0, -9.0, 0.0))])
    tb.pallets["P-1"] = tb.pallets["P-1"].__class__(**{**tb.pallets["P-1"].__dict__, "pose": None})
    r = recommend(snap(tb))
    assert r["reason"] == "all_blocked" and r["blocked"]["P-1"] == ["no_pose"]


def test_a_static_prop_on_the_pickup_straight_blocks_that_pallet():
    prop = PlacedProp(AssetSpec("scene:post", 0.2, 0.2, 1.3), Rectangle(-1.2, 0.0, 0.2, 0.2, 0.0))
    tb = book([("P-1", (0.8, 0.0, 0.0)), ("P-2", (0.8, 2.5, 0.0))], [("D-1", (3.0, -9.0, 0.0))])
    r = recommend(snap(tb, props=[prop]))
    assert "props" in r["blocked"]["P-1"]
    assert r["pick"]["pallet_id"] == "P-2"


def test_the_occupancy_the_submit_check_refuses_is_never_offered():
    # every pick is one check_submit accepts
    tb = book(LINE_P, LINE_D)
    while True:
        r = recommend(snap(tb))
        if r["pick"] is None:
            break
        pid, did = r["pick"]["pallet_id"], r["pick"]["destination_id"]
        assert tb.check_submit(pid, did, tb.revision) is None
        deliver(tb, pid, did)


def test_five_pallets_three_slots_in_well_under_a_tick():
    pallets = [("P-1", (3.2, 1.85, 0.0)), ("P-2", (2.13, 1.8, 0.0)), ("P-3", (1.9, -0.14, -0.14)),
               ("P-4", (0.63, 1.28, 0.17)), ("P-5", (1.02, -1.51, -0.10))]
    slots = [("D-1", (3.4, -7.18, 0.0)), ("D-2", (3.4, -8.47, 0.0)), ("D-3", (3.4, -9.76, 0.0))]
    tb = book(pallets, slots)
    started = time.perf_counter()
    r = recommend(snap(tb, truck=(-2.1, 0.0, 0.0), home=(-2.1, 0.0, 0.0)))
    assert time.perf_counter() - started < 0.5
    assert r["missions"] == 3
