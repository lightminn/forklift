"""M-a1 slice 3c: stop, replan, resume (plan D4) moved from the runner (behaviour, state, port requests and their
order, effects up to a refusal -- the plan's M-a1 criterion)."""

from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning import PlanResult

REAR = np.array([1.0, 2.0, 0.0])


def plan(status="success", x0=1.0, length=1.0, direction=1):
    if status not in ("success", "escape", "backoff"):
        return PlanResult(False, status, np.empty((0, 3)), np.empty(0, np.int8), np.empty(0), 0.0, 0)
    poses = np.array([[x0, 2.0, 0.0], [x0 + direction * length, 2.0, 0.0]])
    return PlanResult(True, status, poses, np.full(2, direction, np.int8), np.zeros(2), length, 0)


@dataclass(frozen=True)
class Config:
    clearance_m: float = 0.3


@dataclass
class Cand:
    direction: int
    length_m: float
    sweep_free: bool

    def record(self):
        return {"d": self.direction, "l": self.length_m}


class World:
    """The runner around the core: the planners answer from queues, every port call is logged."""

    def __init__(self, phase, *, plans=(), slam=None, recognised=False, same=False, escape=None, candidates=(),
                 sweep_free=True, waypoints=(), mission=None, remaining=3.0):
        self.calls = []
        self.plans = list(plans)
        log = self.log

        def answer(name):
            def call(*args, **kwargs):
                log(name, *args, **kwargs)
                assert self.plans, f"an unexpected planning call: {name}"  # the fake port expects its calls
                return self.plans.pop(0)
            return call

        def require(ok, reason):
            log("require", bool(ok), reason)
            if not ok:
                raise RuntimeError(reason)

        clock = iter(range(1000, 2000))
        self.runner_rear = REAR  # run()'s rear, which the runner's closures read
        rect_checker = SimpleNamespace(free=lambda pose, pad: sweep_free)
        self.hooks = SimpleNamespace(
            require=require, wall=lambda: float(next(clock)),
            same_path=lambda new, cur, rem: same, path_stats=lambda p: {"length_m": float(p.length_m)},
            path_record=lambda p: {"status": p.status},
            write_paths=lambda: log("write_paths"), save_npz=lambda name, **a: log("save_npz", name),
            realign_active=lambda: False, realign_install=lambda reason: log("realign_install", reason),
            release_and_carry=lambda t: log("release_and_carry", t) or {"released": True},
            release_after_backoff=lambda t, event: log("release_after_backoff", t, event) or np.array([1.5, 2.0, 0.0]),
            slam_rear=lambda t: np.array([1.2, 2.0, 0.0]),
            grid_world=lambda sc: ("world", sc),
            set_rear=lambda value: setattr(self, "runner_rear", value),
            grid_kwargs=lambda *a, **k: log("grid_kwargs", *a, own_pose=tuple(self.runner_rear)) or {"occupancy": SimpleNamespace(
                occupied=np.zeros((1, 1)), origin_x_m=0.0, origin_y_m=0.0, resolution_m=0.05)},
            plan_observation_leg=answer("plan_observation_leg"), plan_return_leg=answer("plan_return_leg"),
            transport_replan=answer("transport_replan"),
            plan_transport=lambda *a, **k: log("plan_transport", **k) or mission,
            final_straight_prefix=lambda approach, length: log("final_straight_prefix") or (
                plan() if mission is not None and mission.success else None),
            find_escape=lambda t, rear, d, kappa: log("find_escape", d) or (escape, list(candidates)),
            escape_leg=lambda phase: (None, None, rect_checker),
            apply_tracker_profile=lambda: log("apply_tracker_profile"), arm_docking=lambda: log("arm_docking"),
        )
        config = TrackerConfig()
        tracker = SimpleNamespace(config=config, leg_ahead=lambda: (None, 1), remaining_to_goal_m=lambda: remaining)
        self.trackers = {name: tracker for name in ("observe", "approach", "insert", "extract", "transport",
                                                    "withdraw", "return_home")}
        self.paths = {name: plan() for name in self.trackers}
        layer = SimpleNamespace(permission=SimpleNamespace(path_check=SimpleNamespace(blocked="occupied"),
                                                           last_estop=SimpleNamespace(blocked=None)))
        self.obstacle = {"layer": layer, "blocked_ticks": 120, "replans": []}
        if recognised:
            self.obstacle["pickup_estimate"] = "EST"
        self.pocket = {"check": None, "planning_pickup": "PICKUP" if recognised else None}
        self.state = {"transitions": [], "paths": {}, "observation_candidates": [], "near_capture": {}}
        self.ms = MissionState(phase=phase, phase_started=1.0)
        scenario = SimpleNamespace(destination="DEST")
        options = SimpleNamespace(
            planner_config=Config(), travel_config=None, scenario=scenario,
            return_to_pose="HOME", pickup_bounds="BOUNDS", mission_stages=["approach", "insert"],
            observation_waypoints=list(waypoints), backoff_m=0.30, backoff_speed_mps=0.15, backoff_per_leg=2,
            escape_per_leg=2)
        self.core = MissionCore(self.ms, state=self.state, obstacle=self.obstacle, slam=slam, pocket=self.pocket,
                                paths=self.paths, trackers=self.trackers, geometry=SimpleNamespace(
                                    alignment_straight_m=1.0), settings={}, pallet_geometry=None,
                                hooks=self.hooks, options=options)
        # Docking/withdrawal helpers are MissionCore methods since slice 6b; this test's stand-ins go on the instance (6c: the release and escape helpers too).
        for name_ in ['dock_at_delivery_straight', 'arm_docking', 'release_and_carry', 'transport_replan', 'start_docking_retry', 'release_and_stand', 'release_after_backoff', 'slam_release', 'escape_leg', 'find_escape', 'hold_count', 'near_tick', 'near_redraw', 'near_capture_redraw', 'near_protection_prediction', 'near_install', 'start_pickup_retry', 'control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
            if hasattr(self.hooks, name_):
                setattr(self.core, name_, getattr(self.hooks, name_))

    def log(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def names(self):
        return [c[0] for c in self.calls if c[0] not in ("require", "grid_kwargs")]

    def replan(self, t=50.0, requested=0.3):
        return self.core.blocked_replan(t, REAR, 0.1, requested)


def replace_config(config, **kwargs):
    from dataclasses import replace
    return replace(config, **kwargs)


def test_a_new_path_is_installed_with_the_leg_config_and_restarts_the_phase_and_the_watch():
    w = World("observe", plans=[plan(x0=1.0, length=2.0)])
    w.obstacle["progress"] = {"phase": "observe"}
    out = w.replan()
    assert w.names() == ["plan_observation_leg", "write_paths", "realign_install"]
    _, args, kwargs = [c for c in w.calls if c[0] == "plan_observation_leg"][0]
    assert args[0] == ("world", w.core.options.scenario) and (args[1].x_m, args[1].y_m) == (2.0, 2.0)  # the leg's end
    assert kwargs["start_rear"].x_m == 1.0 and kwargs["pickup_bounds"] == "BOUNDS" and kwargs["extended"] is False
    assert w.paths["observe"].length_m == 2.0 and w.state["paths"] == {"observe": {"status": "success"}}
    assert w.trackers["observe"].config == TrackerConfig() and w.ms.phase_started == 50.0
    assert w.obstacle["progress"] == {} and w.obstacle["blocked_ticks"] == 0 and w.obstacle["deadline"] is None
    rec = w.obstacle["replans"][-1]
    assert rec["status"] == "success" and rec["clearance_m"] == 0.3 and rec["length_m"] == 2.0
    assert rec["stop"] == {"path": "occupied", "estop": None, "kappa_inv_m": 0.1, "read": "recovery"}
    assert rec["planning_wall_s"] == 2.0  # the replan's start, its deadline read, the record's read
    assert out.requested_speed == 0.0 and out.rear is REAR
    assert w.calls[-1] == ("realign_install", ("obstacle_replan",), {})


def test_the_deadline_is_20_s_and_halved_for_the_leg_after_recognition():
    seen = []
    w = World("transport", plans=[plan()])
    w.core.transport_replan = lambda *a: seen.append(dict(w.obstacle)["deadline"]) or plan()
    w.replan()
    assert seen == [1021.0]  # the start read at 1000, the deadline read at 1001 + 20 s
    w = World("observe", plans=[plan()], recognised=True)
    w.hooks.plan_observation_leg = lambda *a, **k: seen.append(w.obstacle["deadline"]) or plan()
    w.replan()
    assert seen[-1] == 1010.0  # after recognition: the replan's start + 10 s, the rest for the mission replan


def test_transport_replans_through_the_dispatch_and_arms_the_docking_unless_reapproaching():
    slam = {"docking": {"reapproach": False}, "tracker": SimpleNamespace(mode="tracking"), "control": [[0, 0, 0, 0]]}
    w = World("transport", plans=[plan()], slam=slam)
    w.replan()
    assert w.names() == ["transport_replan", "write_paths", "arm_docking"]
    slam["docking"]["reapproach"] = True
    w = World("transport", plans=[plan()], slam=slam)
    out = w.replan()
    assert w.names() == ["release_and_carry", "transport_replan", "write_paths"]  # released, never armed
    np.testing.assert_array_equal(out.rear, [1.2, 2.0, 0.0])
    assert slam["control"][-1] == [0, 1.2, 2.0, 0.0]


def test_a_reapproach_replan_asks_for_the_hold_again_whether_the_path_changed_or_not():
    """Re-approach hold plan H1 (Codex round 1 P2-1): the replan released the hold before planning and installs the
    tracker itself, so it asks for the re-approach hold again -- on a new path and on the same path kept."""
    for same in (False, True):
        slam = {"docking": {"reapproach": True}, "tracker": SimpleNamespace(mode="tracking"), "control": [[0, 0, 0, 0]]}
        w = World("transport", plans=[plan(length=2.5)], slam=slam, same=same)
        w.core.hold_reapproach = lambda t, length_m: w.log("hold_reapproach", length_m)
        w.replan()
        assert w.names()[-1] == "hold_reapproach"
        assert [c[1] for c in w.calls if c[0] == "hold_reapproach"] == [(1.0,)] if same else [(2.5,)]
    slam = {"docking": {"reapproach": False}, "tracker": SimpleNamespace(mode="tracking"), "control": [[0, 0, 0, 0]]}
    w = World("transport", plans=[plan()], slam=slam)
    w.core.hold_reapproach = lambda t, length_m: w.log("hold_reapproach", length_m)
    w.replan()
    assert "hold_reapproach" not in w.names()  # a transport leg is armed for docking, not held


def test_the_return_replans_home_on_the_return_scenario():
    back = SimpleNamespace(destination="D2")
    slam = {"return_scenario": back, "docking": {}, "tracker": SimpleNamespace(mode="tracking")}
    w = World("return_home", plans=[plan()], slam=slam)
    w.replan()
    _, args, kwargs = [c for c in w.calls if c[0] == "plan_return_leg"][0]
    assert args[0] == ("world", back) and args[2:] == ("HOME", w.core.options.planner_config)
    assert [c[1] for c in w.calls if c[0] == "grid_kwargs"] == [("return", "D2")]


def test_a_held_realignment_refuses_the_recovery():
    slam = {"docking": {}, "tracker": SimpleNamespace(mode="holding")}
    w = World("observe", slam=slam)
    w.hooks.realign_active = w.core.realign_active = lambda: True
    with pytest.raises(RuntimeError, match="near_capture_realign_held_recovery"):
        w.replan()
    assert w.obstacle["blocked_ticks"] == 120 and w.obstacle["replans"] == []


def test_an_invalid_start_tries_an_escape_straight_first_and_drives_it_on_the_backoff_machinery():
    escape = SimpleNamespace(direction=-1, length_m=0.2, record=lambda: {"escape": 1})
    cands = [Cand(-1, 0.30, False), Cand(1, 0.2, True)]
    w = World("transport", plans=[plan("invalid_start")], escape=escape, candidates=cands)
    w.obstacle["replan_wait"] = {"transport": True}
    out = w.replan()
    assert w.names() == ["transport_replan", "find_escape"]
    rec = w.obstacle["replans"][-1]
    assert rec["status"] == "escape" and rec["backoff"] is True and rec["failed"] is False
    assert rec["escape"] == {"escape": 1} and rec["clearance_m"] is None
    assert rec["escape_candidates"] == [{"d": -1, "l": 0.30}, {"d": 1, "l": 0.2}]
    assert w.obstacle["escapes"] == {"transport": 1} and w.obstacle["replan_wait"] == {}
    assert w.obstacle["backoff"] == {"phase": "transport", "config": TrackerConfig(), "time_s": 50.0, "leg_dir": 1,
                                     "escape": True}
    np.testing.assert_allclose(w.paths["transport"].poses[[0, -1]], [[1.0, 2.0, 0.0], [0.8, 2.0, 0.0]])
    cfg = w.trackers["transport"].config
    assert (cfg.cruise_speed_mps, cfg.position_tolerance_m, cfg.yaw_tolerance_rad) == (0.15, 0.01, 0.02)
    assert w.ms.phase_started == 50.0 and out.requested_speed == 0.0


def test_without_an_escape_the_start_replans_at_zero_clearance():
    w = World("transport", plans=[plan("invalid_start"), plan()])
    w.replan()
    assert w.names() == ["transport_replan", "find_escape", "transport_replan", "write_paths"]
    tight = [c for c in w.calls if c[0] == "transport_replan"][1][1]
    assert tight[1].clearance_m == 0.0 and tight[2] is None
    assert w.obstacle["tight_replans"] == 1 and w.obstacle["replans"][-1]["clearance_m"] == 0.0
    w = World("transport", plans=[plan("invalid_start"), plan()], slam={"docking": {"accepted_any": True}})
    w.replan()
    assert "find_escape" not in w.names()  # no escape once the delivery docking accepted a match


def test_a_failed_replan_waits_backs_off_against_the_leg_and_refuses_on_the_fifth_failure():
    w = World("transport", plans=[plan("no_path")] * 2 * 5)
    out = w.replan()
    assert w.names() == ["transport_replan", "transport_replan", "save_npz"]
    rec = w.obstacle["replans"][-1]
    assert rec["failed"] is True and rec["backoff"] is True and w.obstacle["backoffs"] == {"transport": 1}
    assert w.obstacle["replan_wait"] == {}  # driving the backoff, not waiting
    back = w.paths["transport"]
    assert back.status == "backoff" and back.length_m == 0.30 and (back.directions == -1).all()
    np.testing.assert_allclose(back.poses[-1], [0.7, 2.0, 0.0])
    assert w.trackers["transport"].config.cruise_speed_mps == 0.15 and out.requested_speed == 0.0
    w.replan()
    assert w.obstacle["backoffs"] == {"transport": 2}
    w.replan()  # the backoff budget is spent: stand and wait for a plan
    assert w.obstacle["backoffs"] == {"transport": 2} and w.obstacle["replan_wait"] == {"transport": True}
    w.replan()
    with pytest.raises(RuntimeError, match="obstacle_replan_failed in transport: no_path"):
        w.replan()


def test_the_return_backoff_is_skipped_when_it_would_sweep_the_delivered_pallet():
    w = World("return_home", plans=[plan("no_path"), plan("no_path")], sweep_free=False)
    w.replan()
    rec = w.obstacle["replans"][-1]
    assert rec.get("backoff_skipped") is True and "backoff" not in rec and w.obstacle["replan_wait"] == {
        "return_home": True}
    w = World("transport", plans=[plan("invalid_start"), plan("no_path")], candidates=[Cand(-1, 0.30, False)])
    w.replan()  # the escape's checker refused the same straight: no backoff either
    assert w.obstacle["replans"][-1].get("backoff_skipped") is True


def test_before_recognition_a_blocked_observe_leg_tries_the_remaining_candidates_from_here():
    w = World("observe", plans=[plan("no_path"), plan("no_path"), plan("no_path"), plan(length=4.0)],
              waypoints=[(0, 0, 0), (5.0, 1.0, 0.0), (6.0, 1.0, 0.0), (7.0, 1.0, 0.0)])
    w.ms.next_candidate_index = 1
    w.replan()
    assert w.ms.next_candidate_index == 3  # stopped at the first that plans
    assert [c["candidate_index"] for c in w.state["observation_candidates"]] == [1, 2]
    assert [c["success"] for c in w.state["observation_candidates"]] == [False, True]
    assert w.state["observation_waypoint_selected"] == [6.0, 1.0, 0.0]
    assert w.paths["observe"].length_m == 4.0 and w.obstacle["replans"][-1]["clearance_m"] == 0.0


def test_after_recognition_the_whole_mission_is_planned_again_for_a_new_near_waypoint():
    mission = SimpleNamespace(success=True, status="success", approach=plan(), insert=plan(length=0.5))
    w = World("observe", plans=[plan("no_path"), plan("no_path")], recognised=True, mission=mission)
    w.replan()
    assert w.names()[:4] == ["plan_observation_leg", "plan_observation_leg", "save_npz", "plan_transport"]
    assert w.paths["insert"].length_m == 0.5 and w.state["paths"]["insert"] == {"status": "success"}
    assert "apply_tracker_profile" in w.names()
    assert w.obstacle["mission_replans"][-1]["near_found"] is True
    assert w.state["near_capture"]["waypoint"] == [2.0, 2.0, 0.0]
    assert w.obstacle["replans"][-1]["clearance_m"] == 0.3


def test_the_same_path_again_is_a_wait_and_never_counts():
    w = World("observe", plans=[plan()] * 6, same=True)
    for k in range(6):
        out = w.replan(t=50.0 + k)
    assert all(r.get("same_path") for r in w.obstacle["replans"]) and out.requested_speed == 0.0
    assert w.ms.phase_started == 1.0 and "write_paths" not in w.names()


def test_four_new_paths_in_30_s_or_eleven_on_a_leg_end_the_run():
    w = World("observe", plans=[plan()] * 4)
    for k in range(3):
        w.replan(t=50.0 + k)
    with pytest.raises(RuntimeError, match="obstacle_blocked in observe: 4 replans in 30 s"):
        w.replan(t=53.0)
    w = World("observe", plans=[plan()] * 11)
    for k in range(10):
        w.replan(t=50.0 + 31.0 * k)
    with pytest.raises(RuntimeError, match="obstacle_replan_limit in observe: 11"):
        w.replan(t=50.0 + 310.0)


def test_a_backoff_end_releases_first_and_its_config_returns_with_the_new_path():
    w = World("transport", plans=[plan()])
    leg_config = replace_config(TrackerConfig(), cruise_speed_mps=0.4)
    w.obstacle["backoff"] = {"phase": "transport", "config": leg_config, "leg_dir": 1,
                             "stop_cause": {"path": "unknown", "estop": "occupied"}}
    out = w.replan()
    assert w.names()[0] == "release_after_backoff"
    assert [c[1] for c in w.calls if c[0] == "release_after_backoff"] == [(50.0, "release_in_backoff_recovery")]
    np.testing.assert_array_equal(out.rear, [1.5, 2.0, 0.0])
    _, args, _ = [c for c in w.calls if c[0] == "transport_replan"][0]
    assert args[0].x_m == 1.5  # planned from the released pose
    assert w.obstacle["replans"][-1]["stop"] == {"path": "unknown", "estop": "occupied", "kappa_inv_m": 0.1,
                                                 "read": "backoff_end"}
    assert "backoff" not in w.obstacle and w.trackers["transport"].config == leg_config


def test_a_candidate_found_after_a_failure_without_a_tight_retry_counts_as_planned_tight():
    w = World("observe", plans=[plan("timeout"), plan(length=4.0)], waypoints=[(5.0, 1.0, 0.0)])
    w.replan()
    assert "tight_replans" not in w.obstacle  # a timeout is not retried at zero clearance
    assert w.obstacle["replans"][-1]["clearance_m"] == 0.0


def test_the_mission_replan_falls_back_to_zero_clearance_on_an_invalid_start_or_goal():
    tight = SimpleNamespace(success=True, status="success", approach=plan(), insert=plan())
    w = World("observe", plans=[plan("no_path"), plan("no_path")], recognised=True, mission=tight)
    first = SimpleNamespace(success=False, status="approach:invalid_goal", approach=None)
    answers = iter([first, tight])
    w.hooks.plan_transport = lambda *a, **k: w.log("plan_transport", *a, **k) or next(answers)
    w.replan()
    plans = [c for c in w.calls if c[0] == "plan_transport"]
    assert [p[1][1] for p in plans] == [Config(), Config(clearance_m=0.0)]
    assert w.obstacle["replans"][-1]["clearance_m"] == 0.0


def test_after_a_release_the_runners_closures_see_the_released_pose():
    """Codex slice-3 P2: grid_kwargs reads run()'s rear for its own pose; a release inside the core must reach it
    before the replan's grid is prepared."""
    w = World("transport", plans=[plan()])
    w.obstacle["backoff"] = {"phase": "transport", "config": TrackerConfig(), "leg_dir": 1}
    w.replan()
    grids = [c for c in w.calls if c[0] == "grid_kwargs"]
    assert grids == [] or all(c[2]["own_pose"] == (1.5, 2.0, 0.0) for c in grids)
    np.testing.assert_array_equal(w.runner_rear, [1.5, 2.0, 0.0])
    slam = {"docking": {"reapproach": True}, "tracker": SimpleNamespace(mode="tracking"), "control": [[0, 0, 0, 0]]}
    w = World("observe", plans=[plan()], slam=slam)
    w.ms.phase = "transport"
    w.trackers["transport"] = w.trackers["observe"]
    w.plans = [plan()]
    w.core.transport_replan = lambda *a: w.hooks.grid_kwargs(None) and plan()
    w.replan()
    (grid,) = [c for c in w.calls if c[0] == "grid_kwargs"]
    assert grid[2]["own_pose"] == (1.2, 2.0, 0.0)  # the re-approach release's pose


# M-a2 (docs/plans/2026-10-16-m-a2-source-tests-to-behaviour.md): the obstacle fallback's candidate records.
def test_every_obstacle_fallback_candidate_record_keeps_its_start_status_and_searches():
    w = World("observe", plans=[plan("no_path"), plan("no_path"), plan("invalid_goal"), plan(length=4.0)],
              waypoints=[(0, 0, 0), (5.0, 1.0, 0.0), (6.0, 1.0, 0.0), (7.0, 1.0, 0.0)])
    w.ms.next_candidate_index = 1
    w.replan()
    records = w.state["observation_candidates"]
    assert records and all({"search_attempts", "start_rear", "status"} <= set(r) for r in records)
    assert [r["status"] for r in records] == ["invalid_goal", "success"]


def test_same_path_waits_between_new_paths_never_reach_the_replan_limits():
    """M-a2: a same-path wait is neither a 30 s replan nor a leg replan -- two new paths, five waits, one more new
    path inside 30 s is three, under the limit."""
    w = World("observe", plans=[plan()] * 8)
    answers = iter([False, False, True, True, True, True, True, False])
    w.hooks.same_path = lambda new, cur, rem: next(answers)
    for k in range(8):
        try:
            w.replan(t=50.0 + k)
        except RuntimeError as exc:  # a limit reached: a same-path wait was counted
            raise AssertionError(f"replan {k} refused: {exc}") from exc
    replans = w.obstacle["replans"]
    assert [bool(r.get("same_path")) for r in replans] == [False, False, True, True, True, True, True, False]


@pytest.mark.parametrize("kind", ["failed", "escape"])
@pytest.mark.parametrize("window", ["30 s", "leg"])
def test_failed_and_escape_replans_never_count_towards_the_replan_limits(kind, window):
    """M-a2 (Codex review P2): only new paths count -- a failed replan and an escape are neither 30 s replans
    nor leg replans; each kind and each count on its own."""
    w = World("observe", plans=[plan()])
    entry = {"phase": "observe", "time_s": 49.0 if window == "30 s" else 1.0,
             kind: True if kind == "failed" else {"e": 1}}
    w.obstacle["replans"] = [dict(entry) for _ in range(4 if window == "30 s" else 11)]
    try:
        w.replan(t=50.0)
    except RuntimeError as exc:
        raise AssertionError(f"a {kind} replan was counted ({window}): {exc}") from exc
    assert w.paths["observe"].status == "success" and w.names()[-1] == "realign_install"


def test_the_same_path_is_judged_before_the_replan_limits():
    """M-a2 (Codex review P2): three new paths inside 30 s, then the same path again -- a wait, not a fourth
    replan that would end the run."""
    w = World("observe", plans=[plan()] * 4)
    answers = iter([False, False, False, True])
    w.hooks.same_path = lambda new, cur, rem: next(answers)
    for k in range(4):
        try:
            w.replan(t=50.0 + k)
        except RuntimeError as exc:
            raise AssertionError(f"replan {k} refused: {exc}") from exc
    assert w.obstacle["replans"][-1]["same_path"] is True


@pytest.mark.parametrize("used, tried", [({"transport": 2}, False), ({"transport": 1}, True),
                                         ({"return_home": 2}, True)])
def test_the_escape_budget_is_two_per_leg(used, tried):
    """M-a2 (Codex review P2): a leg that has used its two escapes replans at zero clearance instead; another
    leg's escapes do not count."""
    w = World("transport", plans=[plan("invalid_start"), plan()], escape=None)
    w.obstacle["escapes"] = dict(used)
    w.replan()
    assert ("find_escape" in w.names()) is tried


@pytest.mark.parametrize("docking", [{"reapproach": True}, {"accepted_any": True}])
def test_no_escape_on_the_transport_after_a_docking_match_or_during_a_reapproach(docking):
    """M-a2 (Codex review P2): an invalid start on the transport with a docking re-approach under way or after an
    accepted match replans without looking for an escape."""
    w = World("transport", plans=[plan("invalid_start"), plan()], escape=None,
              slam={"docking": dict(docking), "tracker": SimpleNamespace(mode="tracking"), "control": [[0.0] * 4]})
    try:
        w.replan()
    except Exception:  # what follows the refusal of an escape is not this test's subject
        pass
    assert "transport_replan" in w.names()  # the escape rule was reached
    assert "find_escape" not in w.names() and w.obstacle.get("escapes", {}).get("transport", 0) == 0


# M-a2 core sweep (tools/ma2_core_sweep.py --all): conditions the kept source tests read that no behaviour test held.
def test_the_return_replan_retries_at_zero_clearance():
    back = SimpleNamespace(destination="D2")
    slam = {"return_scenario": back, "docking": {}, "tracker": SimpleNamespace(mode="tracking")}
    w = World("return_home", plans=[plan("no_path"), plan()], slam=slam)
    w.core.options.travel_config = Config(clearance_m=0.2)
    w.replan()
    calls = [c for c in w.calls if c[0] == "plan_return_leg"]
    assert [c[1][3] for c in calls] == [Config(), Config(clearance_m=0.0)]
    assert calls[1][2]["travel_config"] == Config(clearance_m=0.0)
    assert w.obstacle["replans"][-1]["clearance_m"] == 0.0


@pytest.mark.parametrize("phase, travel, expected", [("observe", 0.2, 0.3), ("transport", 0.2, 0.2), ("return_home", 0.2, 0.2)])
def test_the_recorded_clearance_is_the_travel_configs_on_the_travel_legs_only(phase, travel, expected):
    w = World(phase, plans=[plan()])
    w.core.options.travel_config = Config(clearance_m=travel)
    w.replan()
    assert w.obstacle["replans"][-1]["clearance_m"] == expected


def test_an_observe_leg_neither_escapes_nor_backs_off():
    w = World("observe", plans=[plan("invalid_start"), plan("no_path"), plan("no_path"), plan("no_path")],
              escape=SimpleNamespace(direction=-1, length_m=0.2, record=lambda: {}))
    for _ in range(2):
        w.replan()
    assert "find_escape" not in w.names() and w.paths["observe"].status not in ("backoff", "escape")
    assert w.obstacle.get("backoffs", {}).get("observe", 0) == 0


def test_when_every_remaining_candidate_fails_the_search_ends_with_the_leg_failed():
    w = World("observe", plans=[plan("no_path"), plan("no_path"), plan("no_path"), plan("no_path")],
              waypoints=[(0, 0, 0), (5.0, 1.0, 0.0), (6.0, 1.0, 0.0)])
    w.ms.next_candidate_index = 1
    try:
        w.replan()
    except Exception as exc:  # the leg's own refusal is fine; running past the list is not
        assert isinstance(exc, RuntimeError), repr(exc)
    assert w.ms.next_candidate_index == 3
    assert [c["candidate_index"] for c in w.state["observation_candidates"]] == [1, 2]


def test_the_return_leg_escapes_an_invalid_start_too():
    """M-a2 (Codex review round 3): both travel legs look for an escape."""
    escape = SimpleNamespace(direction=-1, length_m=0.2, record=lambda: {"escape": 1})
    w = World("return_home", plans=[plan("invalid_start")], escape=escape, candidates=[Cand(-1, 0.30, False)],
              slam={"docking": {}, "tracker": SimpleNamespace(mode="tracking"), "return_scenario": SimpleNamespace(
                  destination="D2")})
    w.replan()
    assert "find_escape" in w.names() and w.obstacle["escapes"] == {"return_home": 1}


def test_a_refused_eleventh_leg_replan_leaves_the_no_progress_watch_alone():
    """M-a2 sweep (order): the leg limit refuses before the watch restarts on a path that is not installed."""
    w = World("observe", plans=[plan()] * 11)
    for k in range(10):
        w.replan(t=50.0 + 31.0 * k)
    w.obstacle["progress"] = {"phase": "observe", "best_m": 2.0, "since_s": 300.0}
    with pytest.raises(RuntimeError, match="obstacle_replan_limit in observe: 11"):
        w.replan(t=50.0 + 310.0)
    assert w.obstacle["progress"] == {"phase": "observe", "best_m": 2.0, "since_s": 300.0}


def test_the_return_leg_backs_off_too():
    """M-a2 sweep (narrowing): both travel legs back off a failed replan."""
    back = SimpleNamespace(destination="D2")
    slam = {"return_scenario": back, "docking": {}, "tracker": SimpleNamespace(mode="tracking")}
    w = World("return_home", plans=[plan("no_path")] * 2, slam=slam)
    w.replan()
    assert w.paths["return_home"].status == "backoff" and w.obstacle["backoffs"] == {"return_home": 1}


def test_the_fifth_failed_replan_refuses_before_it_would_wait():
    """M-a2 sweep (order, Codex round 4): four failures, a success, a fifth failure -- the refusal leaves no wait."""
    w = World("observe", plans=[plan("no_path")] * 8 + [plan()] + [plan("no_path")] * 2)
    for k in range(4):
        w.replan(t=50.0 + k)
    w.replan(t=55.0)  # a new path: the wait ends
    assert w.obstacle.get("replan_wait", {}) == {}
    with pytest.raises(RuntimeError, match="obstacle_replan_failed in observe"):
        w.replan(t=56.0)
    assert w.obstacle.get("replan_wait", {}) == {}


