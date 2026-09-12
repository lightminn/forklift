"""Check the planned docking trajectory without creating a renderer."""

from pathlib import Path

import pytest

from tools import preview_docking

pytestmark = pytest.mark.simulation
REPO_ROOT = Path(__file__).resolve().parents[2]
FORKLIFT = REPO_ROOT / "sim/models/dls08_provisional/forklift.xml"
PALLET = REPO_ROOT / "sim/models/epal6_pallet/pallet.xml"


def test_every_frame_keeps_the_whole_truck_clear_of_the_pallet():
    frames = preview_docking.plan_trajectory(FORKLIFT, PALLET, frames=96)
    assert len(frames) == 96
    assert min(f.clearance_m for f in frames) > 0.0


def test_the_insertion_stops_short_of_the_carriage_touching_the_pallet():
    frames = preview_docking.plan_trajectory(FORKLIFT, PALLET, frames=96)
    inserted = [f for f in frames if f.phase == "insert"]
    assert inserted[-1].penetration_m == pytest.approx(0.360, abs=0.002)
    # the tightest pair anywhere is the fork underside over the lower deck,
    # 28 mm against 22 mm, which is not what limits the depth
    assert inserted[-1].clearance_m == pytest.approx(0.006, abs=0.001)
    assert inserted[-1].clearance_pair[1] == "deck_bottom"
    # what limits the depth is the carriage cross member, which runs out at 0.406 m
    assert inserted[-1].insertion_margin_m == pytest.approx(0.046, abs=0.003)


def test_the_lift_stops_below_the_top_deck_without_moving_the_pallet():
    frames = preview_docking.plan_trajectory(FORKLIFT, PALLET, frames=96)
    assert frames[-1].clearance_m == pytest.approx(0.008, abs=0.001)
    assert frames[-1].clearance_pair[1] == "deck_top"


def test_asking_for_a_deeper_insertion_than_the_truck_allows_is_refused():
    with pytest.raises(ValueError):
        preview_docking.plan_trajectory(FORKLIFT, PALLET, frames=24, insertion_m=0.42)


def test_the_phases_run_in_order_and_each_one_is_used():
    frames = preview_docking.plan_trajectory(FORKLIFT, PALLET, frames=96)
    seen = [f.phase for f in frames]
    assert seen == sorted(seen, key=["approach", "insert", "lift", "settle"].index)
    assert set(seen) == {"approach", "insert", "lift", "settle"}


def test_the_forks_only_rise_after_they_are_inside():
    frames = preview_docking.plan_trajectory(FORKLIFT, PALLET, frames=96)
    assert all(f.lift_m == 0.0 for f in frames if f.phase in ("approach", "insert"))
    assert frames[-1].lift_m == pytest.approx(0.040)
