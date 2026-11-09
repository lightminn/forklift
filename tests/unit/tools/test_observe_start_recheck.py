"""tools/observe_start_recheck.py: the observe-replan start check rebuilt from records (results 2026-11-10 4)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import observe_start_recheck as O  # noqa: E402

from forklift_core.planning.geometry import Bounds, Footprint, Rectangle  # noqa: E402

FP = Footprint(1.29, 0.17, 0.36)
BIG = Bounds(-20.0, 20.0, -20.0, 20.0)


def test_a_rectangle_ahead_blocks_at_the_clearance_and_backing_off_frees_it():
    # the footprint's front edge at x = 1.29; a box whose near face is 0.05 m ahead of it
    box = ("prior:P-5", Rectangle(1.29 + 0.05 + 0.5, 0.0, 1.0, 1.0, 0.0))
    out = O.recheck([box], FP, BIG, (0.0, 0.0, 0.0), 0.10)
    assert out["free_at_zero"] and not out["free_at_clearance"]
    assert out["blockers"] == ["prior:P-5"]
    assert out["max_free_clearance_m"] == pytest.approx(0.05, abs=1e-3)
    assert out["advice_back"]["backoff_m"] == pytest.approx(0.05, abs=0.011)
    assert out["advice_forward"]["backoff_m"] is None


def test_the_bounds_are_named_when_they_block():
    out = O.recheck([], FP, Bounds(-0.2, 1.33, -1.0, 1.0), (0.0, 0.0, 0.0), 0.10)
    assert out["blockers"] == ["bounds"] and not out["free_at_clearance"]


def test_a_start_inside_an_obstacle_is_not_free_at_zero():
    out = O.recheck([("box", Rectangle(0.5, 0.0, 1.0, 1.0, 0.0))], FP, BIG, (0.0, 0.0, 0.0), 0.10)
    assert not out["free_at_zero"] and out["max_free_clearance_m"] is None


def test_the_search_props_leave_out_the_slot_stage_and_grow_only_tall_props():
    scenario = {"props": [
        {"asset": {"uri": "riibotics:slot_marker", "height_m": 1.3}, "rectangle": {"x_m": 1, "y_m": 2, "length_m": 0.9,
                                                                                  "width_m": 0.03, "yaw_rad": 0}},
        {"asset": {"uri": "session:pallet:P-4", "height_m": 0.144}, "rectangle": {"x_m": 0, "y_m": 0, "length_m": 1,
                                                                                 "width_m": 1.2, "yaw_rad": 0}},
        {"asset": {"uri": "rack@slot", "height_m": 2.0}, "rectangle": {"x_m": 5, "y_m": 5, "length_m": 1, "width_m": 1}}]}
    props = dict(O.props_of(scenario, keepout_m=0.15))
    assert set(props) == {"riibotics:slot_marker", "session:pallet:P-4"}
    assert props["riibotics:slot_marker"].length_m == pytest.approx(1.2)  # 1.3 m tall: grown 0.15 each side
    assert props["session:pallet:P-4"].length_m == pytest.approx(1.0)  # under the plane: as drawn
