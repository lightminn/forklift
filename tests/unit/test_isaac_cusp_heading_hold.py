"""Records of the observe cusp heading hold (EPAL 6 fork spacing plan, fix A)."""

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "cusp_heading_hold", ROOT / "sim/isaac/cusp_heading_hold.py"
)
CH = importlib.util.module_from_spec(SPEC)
sys.modules["cusp_heading_hold"] = CH
SPEC.loader.exec_module(CH)


def end(status="braking", failure=None):
    return {
        "status": status,
        "failure": failure,
        "position_error_m": 0.01,
        "yaw_error_rad": 0.06,
    }


def test_a_release_is_watched_to_the_next_leg_end_with_its_motion_and_largest_offsets():
    state = {}
    holds = CH.CuspHeadingHolds(state)
    holds.held(1.0, [7, 0], {"yaw_error_rad": 0.062}, end())
    assert (
        holds.held(1.1, [7, 0], {"yaw_error_rad": 0.0}, end())["yaw_error_rad"] == 0.062
    )  # the same hold
    holds.decide(1.5, "released", {"decided_yaw_error_rad": 0.061}, end())
    assert (
        state["cusp_heading_hold"] is None and state["cusp_heading_watch"]["leg"] == 1
    )
    holds.tick(1.6, 7, 1, end("tracking"), False, 0.0, (0.004, -0.02))
    holds.tick(1.7, 7, 1, end("tracking"), False, -0.05, (-0.009, 0.01))
    holds.tick(3.0, 7, 2, end("braking"), False, 0.0, None)
    leg = state["cusp_heading_holds"][0]["next_leg"]
    assert leg["end"] == "accepted" and leg["time_s"] == 3.0 and leg["moving_s"] == 1.7
    assert leg["max_abs_lateral_m"] == 0.009 and leg["max_abs_heading_rad"] == 0.02
    assert state["cusp_heading_watch"] is None


def test_a_released_leg_held_again_keeps_its_own_record_before_the_next_hold_opens():
    # Codex fix A implementation review 1 P2: the second release overwrote the first watch.
    state = {}
    holds = CH.CuspHeadingHolds(state)
    holds.held(1.0, [7, 0], {}, end())
    holds.decide(1.5, "released", {}, end())
    holds.tick(1.6, 7, 1, end("tracking"), False, -0.05, (0.003, 0.01))
    holds.held(4.0, [7, 1], {}, end())  # the released leg's end, held again
    first = state["cusp_heading_holds"][0]["next_leg"]
    assert (
        first["end"] == "cusp_pending"
        and first["time_s"] == 4.0
        and first["max_abs_lateral_m"] == 0.003
    )
    holds.decide(4.5, "released", {}, end())
    holds.tick(6.0, 7, 2, end("arrived"), False, 0.0, None)
    assert [h["decision"] for h in state["cusp_heading_holds"]] == [
        "released",
        "released",
    ]
    assert state["cusp_heading_holds"][1]["next_leg"]["end"] == "accepted"
    assert state["cusp_heading_holds"][0]["next_leg"]["end"] == "cusp_pending"


@pytest.mark.parametrize(
    "tracker_id, leg, status, pending, how",
    [
        (8, 1, "tracking", False, "replaced"),
        (7, 1, "failed", False, "failed"),
        (7, 1, "braking", True, "cusp_pending"),
    ],
)
def test_a_watch_ends_when_its_leg_is_replaced_fails_or_holds_again(
    tracker_id, leg, status, pending, how
):
    state = {}
    holds = CH.CuspHeadingHolds(state)
    holds.held(1.0, [7, 0], {}, end())
    holds.decide(1.5, "released", {}, end())
    holds.tick(
        2.0,
        tracker_id,
        leg,
        end(status, "endpoint_heading" if status == "failed" else None),
        pending,
        0.0,
    )
    assert (
        state["cusp_heading_holds"][0]["next_leg"]["end"] == how
        and state["cusp_heading_watch"] is None
    )


def test_a_hold_not_released_is_recorded_without_a_watch_and_bad_inputs_are_refused():
    state = {}
    holds = CH.CuspHeadingHolds(state)
    holds.held(1.0, [7, 0], {}, end())
    holds.decide(1.5, "judged_out", {}, end())
    assert (
        state["cusp_heading_holds"][0]["decision"] == "judged_out"
        and state["cusp_heading_watch"] is None
    )
    assert "next_leg" not in state["cusp_heading_holds"][0]
    with pytest.raises(ValueError):
        holds.decide(2.0, "released", {}, end())  # nothing open
    holds.held(3.0, [7, 0], {}, end())
    with pytest.raises(ValueError):
        holds.decide(3.5, "maybe", {}, end())


def test_the_closing_tick_counts_and_a_new_tracker_held_at_once_is_a_replacement():
    # Codex fix A implementation review 2 P3.
    state = {}
    holds = CH.CuspHeadingHolds(state)
    holds.held(1.0, [7, 0], {}, end())
    holds.decide(1.5, "released", {}, end())
    holds.tick(1.6, 7, 1, end("tracking"), False, -0.05, (0.003, 0.01))
    holds.tick(1.7, 7, 1, end("failed", "cross_track"), False, 0.0, (0.04, 0.09))
    leg = state["cusp_heading_holds"][0]["next_leg"]
    assert (
        leg["end"] == "failed"
        and leg["max_abs_lateral_m"] == 0.04
        and leg["max_abs_heading_rad"] == 0.09
    )
    holds.held(2.0, [7, 1], {}, end())
    holds.decide(2.5, "released", {}, end())
    holds.held(
        3.0, [8, 0], {}, end()
    )  # another observe tracker, held before any tick saw it
    assert state["cusp_heading_holds"][1]["next_leg"]["end"] == "replaced"


def test_a_leg_released_and_ended_on_the_same_tick_is_accepted_with_its_first_command():
    # Codex fix A implementation review 3 P2: the runner ticks the new watch on the release tick.
    state = {}
    holds = CH.CuspHeadingHolds(state)
    holds.held(1.0, [7, 0], {}, end())
    holds.decide(1.0, "released", {}, end())
    holds.tick(1.0, 7, 1, end("arrived"), False, -0.0025, (0.001, 0.049))
    leg = state["cusp_heading_holds"][0]["next_leg"]
    assert (
        leg["end"] == "accepted"
        and leg["max_abs_heading_rad"] == 0.049
        and state["cusp_heading_watch"] is None
    )
    holds.held(2.0, [9, 0], {}, end())
    holds.decide(2.0, "released", {}, end())
    holds.tick(2.0, 9, 1, end("tracking"), False, -0.0025, (0.0, 0.0))
    assert state["cusp_heading_watch"]["moving_s"] == 2.0
