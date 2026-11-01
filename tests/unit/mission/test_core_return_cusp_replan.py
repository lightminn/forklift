"""docs/plans/2026-10-31-return-cusp-replan.md D1/D2: a heading failure at a return_home cusp is braked to a stand,
then the return is planned again from there (the goal-stall recovery's call: no turnaround, no start backoff, no tight
retry), at most ``return_cusp_replans`` times per return; off (0) it ends the run as before."""

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

import test_core_cusp_observe as co
from forklift_core.planning import PlanResult

MISS = co.Tracking(status="failed", failure="endpoint_heading", at_cusp=True, speed_mps=0.0095)


def from_here():
    """A replanned return starting where the test's truck stands (co.REAR)."""
    return PlanResult(True, "success", np.array([[1.0, 2.0, 0.0], [2.0, 2.0, 0.0], [3.0, 2.0, 0.0]]),
                      np.ones(3, np.int8), np.zeros(3), 2.0, 0)


def world(*, budget=2, slam=True, slam_mode="tracking", plans=(), return_replans=0, **kw):
    w = co.World("return_home", slam=slam, slam_mode=slam_mode, plans=plans, **kw)
    w.core.options.return_cusp_replans = budget
    w.core.options.return_to_pose = "HOME"
    if return_replans:
        w.state["return_cusp_replans"] = [{}] * return_replans
    plans_ = list(plans)

    def plan_return_leg(*a, **k):
        w.calls.append(("plan_return_leg", a, k))
        assert plans_, "an unexpected planning call"
        return plans_.pop(0)
    w.hooks.plan_return_leg = plan_return_leg
    if slam:
        w.slam["return_scenario"] = "SC_R"
        w.slam["holds"] = []
        w.slam["tracker"].applied = (np.array([0.2, -0.1, 0.05]),)
    return w


def stand(w, ticks=12, tracking=MISS):
    out = None
    for _ in range(ticks):
        out = w.tick(tracking)
    return out


def test_a_return_cusp_miss_brakes_at_zero_until_it_stands_then_replans_the_return():
    w = world(plans=[from_here()])
    for _ in range(11):
        out = w.tick(MISS)
        assert out.tracking.status == "braking" and out.tracking.speed_mps == 0.0  # zero, not the failed 0.0095
    assert "plan_return_leg" not in w.names() and "return_cusp_replans" not in w.state
    out = w.tick(MISS)
    (_, args, kwargs), = [c for c in w.calls if c[0] == "plan_return_leg"]
    assert args[0] == ("world", "SC_R") and (args[1].x_m, args[1].y_m) == (1.0, 2.0) and args[2] == "HOME"
    assert args[3] == co.Config() and kwargs == {"geometry": "G", "travel_config": None}
    assert ("grid_kwargs", (None,)) in [(c[0], c[1]) for c in w.calls]
    rec, = w.state["return_cusp_replans"]
    assert rec["status"] == "success" and rec["path"] == {"status": "success"} and rec["released"] is None
    assert rec["replaced_path"] == {"status": "success"} and rec["search_attempts"] == []
    assert w.core.trackers["return_home"] is not w.tracker and w.ms.phase_started == 50.0
    assert w.obstacle["progress"] == {} and w.core._return_cusp_wait is None and w.ms.cusp_stop_ticks == 0
    assert rec["plan_start_pose"] == [1.0, 2.0, 0.0]
    assert w.calls[[c[0] for c in w.calls].index("show_path")][1][1:] == ("Return", (0.55, 0.2, 0.85))
    assert out.tracking.status == "tracking"  # the new leg's first update


def test_no_turnaround_no_start_backoff_and_no_second_call_when_the_plan_fails():
    failed = PlanResult(False, "invalid_start", np.empty((0, 3)), np.empty(0, np.int8), np.empty(0), 0.0, 0)
    w = world(plans=[failed])
    w.core.options.return_start_backoff_m = 0.3  # F2' on: still not for a cusp
    w.core.return_turnaround_kwargs = lambda **k: pytest.fail("a cusp replan is no turnaround")
    w.core._return_backoff_kwargs = lambda *a: pytest.fail("a cusp replan has no start backoff")
    with pytest.raises(RuntimeError, match="return_cusp_replan_failed: invalid_start"):
        stand(w)
    assert len([c for c in w.calls if c[0] == "plan_return_leg"]) == 1
    rec, = w.state["return_cusp_replans"]
    assert rec["status"] == "invalid_start" and rec["path"] is None
    assert "write_paths" not in w.names() and w.core.trackers["return_home"] is w.tracker


def test_the_budget_is_the_returns_own_and_spent_it_ends_as_before():
    w = world(return_replans=2)
    w.state["cusp_replans"] = [{}, {}]
    out = w.tick(MISS)
    assert out.tracking == MISS and getattr(w.core, "_return_cusp_wait", None) is None  # no budget left: it stands
    w = world(plans=[co.plan()])
    w.state["cusp_replans"] = [{}, {}]  # the transport's two spent
    stand(w)
    assert len(w.state["return_cusp_replans"]) == 1


def test_off_it_is_as_before_no_call_no_key():
    w = world(budget=0)
    assert w.tick(MISS).tracking == MISS and "return_cusp_replans" not in w.state
    assert not hasattr(w.core, "_return_cusp_wait") and w.ms.cusp_stop_ticks == 0  # off: nothing written
    w = co.World("return_home")  # options without the attribute (a record from before the option)
    assert w.tick(MISS).tracking == MISS and "return_cusp_replans" not in w.state


@pytest.mark.parametrize("miss", [
    replace(MISS, at_cusp=False), replace(MISS, off_path=True), replace(MISS, failure="endpoint_position"),
    replace(MISS, status="braking"), replace(MISS, status="arrived"),
])
def test_only_a_heading_failure_at_a_cusp_on_the_path_is_replanned(miss):
    w = world(plans=[co.plan()])
    stand(w, 13, miss)
    assert "plan_return_leg" not in w.names() and "return_cusp_replans" not in w.state


def test_a_held_slam_is_released_and_the_grid_reprojected_before_the_plan_reads_the_pose():
    w = world(slam_mode="holding", plans=[co.plan()])
    refreshed = []
    w.obstacle.update(applied=(0.0, 0.0, 0.0), version=4,
                      layer=SimpleNamespace(refresh=lambda *a, **k: refreshed.append((a, k))))
    order = []
    w.core.release_slam = lambda odom: order.append("release") or (0.12, 0.03)
    w.hooks.slam_odom_base = lambda: "ODOM"
    w.hooks.now_s = lambda: 49.9
    w.hooks.slam_rear = lambda now: order.append("slam_rear") or np.array([3.0, 4.0, 0.1])
    grid = w.hooks.grid_kwargs
    w.hooks.grid_kwargs = lambda *a: order.append(("grid", tuple(w.runner_rear))) or grid(*a)
    stand(w)
    assert order[:3] == ["release", "slam_rear", ("grid", (3.0, 4.0, 0.1))]  # set_rear before grid_kwargs
    assert w.slam["holds"][-1]["event"] == "release_before_return_cusp_replan"
    assert w.slam["holds"][-1]["jump_m"] == 0.12 and w.obstacle["version"] == 5 and refreshed
    assert w.obstacle["applied"] == pytest.approx((0.2, -0.1, 0.05))
    (_, args, _), = [c for c in w.calls if c[0] == "plan_return_leg"]
    assert (args[1].x_m, args[1].y_m, args[1].yaw_rad) == pytest.approx((3.0, 4.0, 0.1))
    assert w.state["return_cusp_replans"][-1]["released"] == {"jump_m": 0.12, "jump_rad": 0.03}
    rec = w.state["return_cusp_replans"][-1]
    assert rec["rear_pose"] == [1.0, 2.0, 0.0] and rec["plan_start_pose"] == pytest.approx([3.0, 4.0, 0.1])


def test_without_slam_the_scenario_is_the_missions_and_the_stand_is_the_true_one():
    w = world(slam=False, plans=[co.plan()])
    stand(w)
    (_, args, _), = [c for c in w.calls if c[0] == "plan_return_leg"]
    assert args[0] == ("world", "SC") and w.state["return_cusp_replans"][-1]["released"] is None
    w = world(slam=False, velocity=(0.2, 0.0))
    for _ in range(13):
        assert w.tick(MISS).tracking.status == "braking"  # rolling: never stands, never plans
    assert w.core._return_cusp_wait["ticks"] == 0


def test_a_held_cusp_judged_out_is_braked_in_the_same_tick_not_ended():
    w = world(plans=[co.plan()])
    w.tracker.update = lambda rear, speed, dt: w.calls.append(("update", (), {})) or MISS  # latched once failed
    w.tracker.remaining = 1.0  # outside the leg-end window: the hold decides "window" and latches the miss
    out = w.tick(co.Tracking(status="tracking", cusp_pending=True))
    assert "fail_pending" in w.names() and out.tracking.status == "braking" and out.tracking.speed_mps == 0.0


def test_the_runner_options_default_to_every_earlier_run_and_the_contract_knows_them():
    """D1/D2: --return-cusp-replans 0 (off) and --transport-cusp-replans 2 by default, read with getattr (a recorded
    run's arguments predate them), registered in the near-field contract with the same values."""
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(root / "sim/isaac"))
    sys.path.insert(0, str(root / "tests/unit/tools"))
    import near_tracking
    from test_riibotics_isaac_scenes import _runner_args

    args = _runner_args([])
    assert (args.return_cusp_replans, args.transport_cusp_replans) == (0, 2)
    args = _runner_args(["--return-cusp-replans", "2", "--transport-cusp-replans", "3"])
    assert (args.return_cusp_replans, args.transport_cusp_replans) == (2, 3)
    with pytest.raises(SystemExit):
        _runner_args(["--return-cusp-replans", "-1"])
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["return_cusp_replans"] == 0
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["transport_cusp_replans"] == 2
    source = (root / "sim/isaac/run_transport.py").read_text()
    assert 'max_cusp_replans = int(getattr(args, "transport_cusp_replans", 2))' in source
    assert 'return_cusp_replans=int(getattr(args, "return_cusp_replans", 0))' in source


def test_only_the_return_cusp_replan_writes_its_budget():
    """D1: the budget is the length of state["return_cusp_replans"], written by this rule alone -- an obstacle, scene
    or release replan that later replaces the path (even in the same tick) spends none of it."""
    import inspect

    from forklift_core.mission import core

    source = inspect.getsource(core)
    assert source.count('"return_cusp_replans"') == 3  # the budget read, the append, and the option read
    assert source.count('state.setdefault("return_cusp_replans", []).append(record)') == 1


def test_a_path_replaced_or_a_move_while_waiting_starts_the_stand_count_again():
    """Codex implementation review P2-2: the stand count belongs to one failure on one tracker -- 11 ticks standing,
    then another rule replacing the path (or the truck moving on), and a later failure counts from zero again."""
    w = world(plans=[co.plan()])
    stand(w, 11)
    assert w.core._return_cusp_wait["ticks"] == 11
    w.core.trackers["return_home"] = co.HoldTracker(w.calls)  # an obstacle replan / resume installed another path
    w.tick(MISS)
    assert w.core._return_cusp_wait["ticks"] == 1 and "plan_return_leg" not in w.names()
    stand(w, 5)
    w.tick(co.Tracking(status="tracking", speed_mps=0.2))  # following again: no failure to wait on
    assert w.core._return_cusp_wait is None
    w.tick(MISS)
    assert w.core._return_cusp_wait["ticks"] == 1 and "plan_return_leg" not in w.names()
    assert w.ms.cusp_stop_ticks == 0  # the transport's count untouched


def test_the_budget_and_its_records_are_the_missions():
    """Codex implementation review P2-1: a session's next mission starts with the whole budget -- the record is a
    mission key, absent at capture, absent after each restore."""
    from forklift_core.mission.baseline import MISSION_KEYS

    assert "return_cusp_replans" in MISSION_KEYS["state"]


def test_a_restored_baseline_gives_the_next_mission_its_whole_budget():
    """Two missions in a session: the first spends both replans, the restore hands their records over, and the next
    mission's return cusp failure is planned again."""
    from forklift_core.mission.baseline import MissionBaseline

    w = world(plans=[from_here(), from_here(), from_here()])
    w.ms.last_tracking = None  # as at a mission's start (the test's stand-in is no plain data)
    base = MissionBaseline.capture(w.core)
    for _ in range(2):
        w.core.trackers["return_home"] = w.tracker  # back on a path that misses its cusp
        stand(w)
    assert len(w.state["return_cusp_replans"]) == 2
    records = base.restore(w.core)
    assert len(records["state"]["return_cusp_replans"]) == 2 and "return_cusp_replans" not in w.state
    w.core.trackers["return_home"] = w.tracker
    stand(w)
    assert len(w.state["return_cusp_replans"]) == 1
