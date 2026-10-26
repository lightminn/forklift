"""Re-approach hold plan (docs/plans/2026-10-25-reapproach-slam-hold.md): H1 holds the SLAM correction through a D7
re-approach, from its install to the retry match; E1 exits release, re-plan and hold again once, then leave the hold
off for the docking."""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_core_docking_helpers import World, leg  # noqa: E402


class FakeSlam:
    """SlamPoseTracker's surface the core reads: the mode, the two corrections, hold / release and the hold travel."""

    def __init__(self, mode="holding"):
        self.mode = mode
        self.received = ((0.0, 0.0, 0.0), 1.0, 7)
        self.applied = ((0.0, 0.0, 0.0), 1.0, 7)
        self.hold_travel_m = 0.0
        self.calls = []

    def hold(self, *, odom_from_base):
        assert self.mode == "tracking"
        self.calls.append("hold")
        self.mode, self.hold_travel_m = "holding", 0.0

    def release(self, *, odom_from_base):
        assert self.mode == "holding"
        self.calls.append("release")
        self.mode, self.applied = "tracking", self.received
        return 0.02, 0.01


def world(tmp_path, hold="on", plans=2, **docking):
    w = World(tmp_path, docking={"previous_goal": [9.0, 0.0, 0.0], "goal_frame": "held", **docking},
              plans=[(leg(1.1, 6.0 + k), {"status": "success"}) for k in range(plans)])
    w.slam["tracker"] = FakeSlam()
    w.hooks.slam_odom_base = lambda: (1.0, 2.0, 0.0)
    w.core.options.d7_reapproach_hold = hold
    return w


def test_off_leaves_the_reapproach_tracking(tmp_path):
    w = world(tmp_path, hold="off")
    w.core.start_docking_retry(40.0, "b", "docking_unaligned")
    assert w.slam["tracker"].calls == ["release"] and w.slam["tracker"].mode == "tracking"
    assert "reapproach_hold" not in w.slam["docking"]
    assert not [h for h in w.slam["holds"] if h.get("event") == "hold_reapproach"]


def test_on_holds_from_the_reapproach_install(tmp_path):
    w = world(tmp_path)
    w.core.start_docking_retry(40.0, "b", "docking_unaligned")
    assert w.slam["tracker"].calls == ["release", "hold"] and w.slam["tracker"].mode == "holding"
    assert w.slam["docking"]["reapproach_hold"] == {"since_s": 40.0, "length_m": 6.0}
    assert w.slam["holds"][-1]["event"] == "hold_reapproach"


def test_a_spent_docking_or_a_tracker_not_tracking_is_not_held(tmp_path):
    w = world(tmp_path, reapproach_hold_spent=True)
    w.core.start_docking_retry(40.0, "b", "docking_unaligned")
    assert w.slam["tracker"].calls == ["release"]
    w = world(tmp_path)
    w.slam["tracker"].mode = "warming_up"
    w.core.hold_reapproach(40.0, 3.0)
    assert w.slam["tracker"].calls == [] and "reapproach_hold" not in w.slam["docking"]


def held(tmp_path, **kw):
    w = world(tmp_path, **kw)
    w.core.start_docking_retry(40.0, "b", "docking_unaligned")
    return w


def test_no_exit_below_every_bound(tmp_path):
    w = held(tmp_path)
    tr = w.slam["tracker"]
    tr.received = ((0.11, 0.0, 0.07), 2.0, 9)  # within 0.12 m / 0.08 rad of the applied one
    tr.hold_travel_m = 7.9  # max(2 x 6, 6 + 2) = 12 m
    assert w.core.reapproach_hold_exit_due(79.0) is None  # 39 s of work time


@pytest.mark.parametrize("change, reason", [
    (lambda w: setattr(w.slam["tracker"], "received", ((0.0, 0.13, 0.0), 2.0, 9)), "correction_gap"),
    (lambda w: setattr(w.slam["tracker"], "received", ((0.0, 0.0, 0.09), 2.0, 9)), "correction_gap"),
    (lambda w: setattr(w.slam["tracker"], "hold_travel_m", 12.1), "hold_travel"),
])
def test_each_bound_is_an_exit(tmp_path, change, reason):
    w = held(tmp_path)
    change(w)
    assert w.core.reapproach_hold_exit_due(50.0) == reason


def test_the_time_bound_counts_work_time(tmp_path):
    w = held(tmp_path)
    assert w.core.reapproach_hold_exit_due(80.1) == "hold_time"


def test_the_short_leg_travel_bound_is_length_plus_two(tmp_path):
    w = held(tmp_path)
    w.slam["docking"]["reapproach_hold"]["length_m"] = 0.7
    w.slam["tracker"].hold_travel_m = 2.69
    assert w.core.reapproach_hold_exit_due(50.0) is None
    w.slam["tracker"].hold_travel_m = 2.71
    assert w.core.reapproach_hold_exit_due(50.0) == "hold_travel"


def test_no_watch_outside_a_held_reapproach(tmp_path):
    w = held(tmp_path)
    w.slam["tracker"].hold_travel_m = 99.0
    w.slam["docking"]["reapproach"] = False  # the straight began (the docking's own hold)
    assert w.core.reapproach_hold_exit_due(50.0) is None
    w = world(tmp_path, hold="off")
    w.core.start_docking_retry(40.0, "b", "docking_unaligned")
    w.slam["tracker"].hold_travel_m = 99.0
    assert w.core.reapproach_hold_exit_due(50.0) is None


def test_the_first_exit_replans_and_holds_again_uncounted(tmp_path):
    w = held(tmp_path, plans=2)
    counted = [e for e in w.state["docking_retries"] if e.get("counted")]
    w.core.reapproach_hold_exit(50.0, "correction_gap")
    tr = w.slam["tracker"]
    assert tr.calls == ["release", "hold", "release", "hold"] and tr.mode == "holding"
    ev = w.state["docking_retries"][-1]
    assert (ev["trigger"], ev["counted"], ev["exit"], ev["exit_number"]) == ("h", False, "correction_gap", 1)
    assert [e for e in w.state["docking_retries"] if e.get("counted")] == counted  # the budget is untouched
    assert w.core.paths["transport"].length_m == 7.0 and w.slam["docking"]["reapproach_hold"]["since_s"] == 50.0


def test_a_second_exit_leaves_the_hold_off_for_the_docking(tmp_path):
    w = held(tmp_path, plans=2)
    w.core.reapproach_hold_exit(50.0, "correction_gap")
    w.core.reapproach_hold_exit(60.0, "hold_time")
    tr = w.slam["tracker"]
    assert tr.mode == "tracking" and tr.calls[-1] == "release"
    assert w.slam["docking"]["reapproach_hold_spent"] is True and "reapproach_hold" not in w.slam["docking"]
    assert w.state["docking_retries"][-1]["spent"] is True
    w.core.hold_reapproach(70.0, 3.0)  # a later re-approach of this docking (a d retry, an obstacle replan)
    assert tr.mode == "tracking"


def test_an_exit_with_no_plan_ends_as_a_retry_with_no_plan(tmp_path):
    w = held(tmp_path, plans=1)
    w.hooks.plan_docking_reapproach = lambda *a, **k: (None, {"refused": "box", "status": "success"})
    with pytest.raises(RuntimeError, match="docking_retry_no_plan:box:success"):
        w.core.reapproach_hold_exit(50.0, "hold_travel")


def test_a_d_retry_inside_a_held_reapproach_releases_and_holds_the_next_one(tmp_path):
    w = held(tmp_path, plans=2)
    w.core.start_docking_retry(55.0, "d", "transport_recovery_after_docking")
    assert w.slam["tracker"].calls == ["release", "hold", "release", "hold"]
    assert w.slam["docking"]["reapproach_hold"]["since_s"] == 55.0
