"""M-a1 slice 6b: the delivery docking's helpers, moved from run() closures into MissionCore methods
(behaviour, state, port requests and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

import json
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning import PlanResult, Pose2D


def leg(x0=0.0, length=2.0, direction=1):
    poses = np.array([[x0, 0.0, 0.0], [x0 + length * direction, 0.0, 0.0]])
    return PlanResult(True, "success", poses, np.full(2, direction, np.int8), np.zeros(2), length, 3)


@dataclass(frozen=True)
class Config:
    clearance_m: float = 0.3


@dataclass(frozen=True)
class Dry:
    status: str
    position_error_m: float = 0.01
    yaw_error_rad: float = 0.0
    trajectory: tuple = (1,)


@dataclass(frozen=True)
class Scenario:
    destination: Pose2D


class World:
    def __init__(self, tmp_path, *, docking=None, mode="holding", obstacle=None, plans=(), dry_ok=True, retries=()):
        self.calls = []
        log_ = self.log
        clock = iter(np.arange(100.0, 200.0, 0.5))
        poses = {"rear": np.array([1.0, 2.0, 0.0])}

        def release(**k):
            log_("release", **k)
            poses["rear"] = np.array([1.1, 2.0, 0.01])
            return 0.1, 0.01

        def require(ok, reason):
            log_("require", bool(ok), reason)
            if not ok:
                raise RuntimeError(reason)

        plans_ = list(plans)
        self.slam = {"tracker": SimpleNamespace(mode=mode, release=release, applied=(np.array([0.1, 0.0, 0.01]),)),
                     "docking": dict(docking or {}), "holds": [], "transport_scenario": Scenario(Pose2D(9.0, 0.0, 0.0))}
        self.hooks = SimpleNamespace(
            require=require, path_record=lambda p: {"length_m": p.length_m}, record_json=lambda v, indent=None: "{}",
            final_straight_prefix=lambda path, keep: log_("prefix", keep) or leg(path.poses[-1][0] - keep, keep),
            time_module=lambda: SimpleNamespace(monotonic=lambda: float(next(clock))),
            bicycle_rollout=lambda *a: log_("rollout", a[3]) or Dry("arrived" if dry_ok else "failed"),
            grid_kwargs=lambda *a, **k: log_("grid_kwargs", *a, **k) or {"deadline": None},
            grid_world=lambda sc: ("world", sc),
            plan_docking_reapproach=lambda *a, **k: log_("plan_docking_reapproach", *a, **k) or plans_.pop(0),
            plan_transport_leg=lambda *a, **k: log_("plan_transport_leg", *a, **k) or leg(),
            add_path_display=lambda stage, path, label, color: log_("show", label), stage=lambda: "STAGE",
            slam_odom_base=lambda: "ODOM", slam_rear=lambda t: poses["rear"].copy(), loaded=lambda: True,
            world=lambda: SimpleNamespace(current_time=57.0),
            # The runner's Sink: the core writes no file itself (it has no output directory).
            write_paths=lambda: (tmp_path / "paths.json").write_text(json.dumps(self.state["paths"])),
        )
        retry = SimpleNamespace(
            carry_on_release=lambda docking, before, after, withdraw_poses, destination: log_("carry", before, after) or {
                "moved": True, "withdraw_poses": np.array([[0.0, 0.5, 0.0], [0.0, 0.2, 0.0]]),
                "destination": (9.1, 0.0, 0.01), "frame_change": [0.1, 0.0, 0.01]},
            line_start=lambda goal, keep: (goal[0] - keep, goal[1], goal[2]),
        )
        self.paths = {"transport": leg(0.0, 9.0), "withdraw": leg(9.0, 0.4, -1)}
        tracker = SimpleNamespace(config=TrackerConfig(cruise_speed_mps=0.5))
        self.trackers = {"transport": tracker, "withdraw": SimpleNamespace(config=TrackerConfig())}
        self.ms = MissionState(phase="transport", phase_started=0.0)
        self.state = {"transitions": [], "paths": {}, "docking_retries": list(retries)}
        self.core = MissionCore(
            self.ms, state=self.state, obstacle=obstacle, slam=self.slam, pocket={"check": None}, paths=self.paths,
            trackers=self.trackers, geometry=SimpleNamespace(delivery_straight_m=1.5), settings={},
            pallet_geometry=None, hooks=self.hooks,
            options=SimpleNamespace(DOCKING_RETRY=retry, scenario="SC", planner_config=Config(),
                                    initial_time=50.0))

    def log(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def names(self):
        return [c[0] for c in self.calls if c[0] not in ("require",)]


def test_arm_docking_drives_the_transport_only_to_its_delivery_straight(tmp_path):
    w = World(tmp_path, docking={"status": "pending"})
    w.core.arm_docking()
    d = w.slam["docking"]
    assert d["status"] == "armed" and d["delivery_rear"] == [9.0, 0.0, 0.0] and d["predelivery_rear"] == [9.0, 0.0, 0.0]
    assert w.calls[0] == ("prefix", (1.5,), {})
    cfg = w.core.trackers["transport"].config
    assert (cfg.position_tolerance_m, cfg.yaw_tolerance_rad, cfg.cruise_speed_mps) == (0.03, 0.05, 0.5)
    assert w.slam["transport_config"].cruise_speed_mps == 0.5
    w = World(tmp_path, docking={"status": "done"})
    w.core.arm_docking()
    assert w.calls == []  # only a pending or armed docking is armed
    w = World(tmp_path, docking={"status": "pending", "keep_m": 0.4})
    w.hooks.final_straight_prefix = lambda path, keep: None
    w.core.arm_docking()
    assert w.slam["docking"]["status"] == "no_final_straight"


def test_the_withdraw_moves_with_the_docked_goal(tmp_path):
    w = World(tmp_path)
    w.core.move_withdraw((0.0, 0.1, 0.0))
    np.testing.assert_allclose(w.core.paths["withdraw"].poses, [[9.0, 0.1, 0.0], [8.6, 0.1, 0.0]])
    assert w.state["paths"]["withdraw"] == {"length_m": 0.4}
    w.paths.pop("withdraw")
    w.core.move_withdraw((0.0, 0.1, 0.0))  # nothing to move
    assert w.core.docking_stop_config().position_tolerance_m == 0.03


def test_a_reapproach_plan_is_accepted_only_when_its_dry_run_stops_at_the_line_start(tmp_path):
    w = World(tmp_path, plans=[(leg(1.0, 3.0), {"status": "success"})])
    path, record = w.core.reapproach_plan((1.0, 2.0, 0.0), (7.5, 0.0, 0.0))
    assert path is not None and record["planning_wall_s"] == 0.5 and record["dry_run"]["status"] == "arrived"
    _, args, kwargs = [c for c in w.calls if c[0] == "plan_docking_reapproach"][0]
    assert args[0] == ("world", w.slam["transport_scenario"]) and args[3] == Config() and kwargs["keep_m"] == 1.5
    assert kwargs["deadline"].seconds == 20.0
    assert [c for c in w.calls if c[0] == "grid_kwargs"][0][2] == {"own": (1.0, 2.0, 0.0)}
    w = World(tmp_path, plans=[(leg(1.0, 3.0), {"status": "success"})], dry_ok=False)
    path, record = w.core.reapproach_plan((1.0, 2.0, 0.0), (7.5, 0.0, 0.0))
    assert path is None and record["refused"] == "dry_run"


def test_a_release_carries_a_held_goal_the_withdraw_and_the_destination_and_reprojects_the_grid(tmp_path):
    refreshed = []
    obstacle = {"applied": (0, 0, 0), "version": 1,
                "layer": SimpleNamespace(refresh=lambda *a, **k: refreshed.append((a, k)))}
    w = World(tmp_path, obstacle=obstacle)
    out = w.core.release_and_carry(30.0)
    assert out["released"] is True and out["goal_moved"] is True and out["rear"] == [1.1, 2.0, 0.01]
    assert w.slam["transport_scenario"].destination == Pose2D(9.1, 0.0, 0.01)
    np.testing.assert_allclose(w.core.paths["withdraw"].poses[-1], [0.0, 0.2, 0.0])
    assert w.slam["holds"][-1]["event"] == "release_docking_retry" and obstacle["version"] == 2
    assert refreshed[0][1]["current_pose"] == (1.1, 2.0, 0.01) and refreshed[0][1]["loaded"] is True
    w = World(tmp_path, mode="tracking")
    assert w.core.release_and_carry(30.0) == {"released": False}


def test_a_docking_retry_releases_plans_back_to_the_line_and_installs_the_reapproach(tmp_path):
    w = World(tmp_path, docking={"previous_goal": [9.0, 0.0, 0.0], "goal_frame": "held"},
              plans=[(leg(1.1, 6.0), {"status": "success"})], obstacle={"progress": {"x": 1}, "replan_wait": {
                  "transport": True}, "backoff": {"phase": "transport"}, "applied": (0, 0, 0), "version": 1,
                  "layer": SimpleNamespace(refresh=lambda *a, **k: None)})
    w.core.start_docking_retry(40.0, "c", "transport_recovery_after_docking")
    (event,) = w.state["docking_retries"]
    assert event["trigger"] == "c" and event["counted"] is True and event["line_start"] == [7.5, 0.0, 0.0]
    assert event["release"]["released"] is True and event["start"] == [1.1, 2.0, 0.01]
    d = w.slam["docking"]
    assert (d["status"], d["round"], d["reapproach"], d["retry_seed_goal"]) == ("armed", 1, True, [9.0, 0.0, 0.0])
    assert w.core.paths["transport"].length_m == 6.0 and w.ms.phase_started == 40.0
    assert "backoff" not in w.core.obstacle and w.core.obstacle["replan_wait"] == {} and w.core.obstacle["progress"] == {}
    assert json.loads((tmp_path / "paths.json").read_text()) == {  # the paths file is rewritten through the Sink
        "transport": {"length_m": 6.0}, "withdraw": {"length_m": 0.4}}
    assert w.state["paths_replaced"][0]["by"] == "reapproach"


def test_a_third_counted_retry_or_no_plan_ends_the_run_and_a_first_match_refused_uses_the_prior(tmp_path):
    w = World(tmp_path, retries=[{"counted": True}, {"counted": True}])
    with pytest.raises(RuntimeError, match="transport_recovery_after_docking"):
        w.core.start_docking_retry(40.0, "c", "transport_recovery_after_docking")
    w = World(tmp_path, docking={"delivery_rear_prior": [8.8, 0.1, 0.0]}, plans=[(None, {"refused": "box"})])
    with pytest.raises(RuntimeError, match="docking_retry_no_plan:box:None"):
        w.core.start_docking_retry(40.0, "a", "docking_unaligned")
    assert w.slam["docking"]["previous_goal"] == [8.8, 0.1, 0.0] and w.slam["docking"]["goal_frame"] == "map"


def test_an_obstacle_replan_during_a_reapproach_replans_the_reapproach(tmp_path):
    w = World(tmp_path, docking={"reapproach": True, "previous_goal": [9.0, 0.0, 0.0]},
              plans=[(leg(1.0, 6.0), {"status": "success"}), (None, {"refused": "no_path", "status": "no_path"}),
                     (None, {"refused": "box", "expansions": 7})])
    start = Pose2D(1.0, 2.0, 0.0)
    assert w.core.transport_replan(start, Config(), None).length_m == 6.0
    assert w.state["docking_retries"][-1] == {"trigger": "obstacle_replan", "time_s": 7.0, "counted": False,
                                              "plan": w.state["docking_retries"][-1]["plan"]}
    assert w.core.transport_replan(start, Config(), None).status == "no_path"
    failed = w.core.transport_replan(start, Config(), None)
    assert (failed.success, failed.status, failed.expanded_nodes) == (False, "reapproach_box", 7)
    w = World(tmp_path)
    assert w.core.transport_replan(start, Config(), "TRAVEL").length_m == 2.0
    _, args, kwargs = [c for c in w.calls if c[0] == "plan_transport_leg"][0]
    assert args[0] == ("world", w.slam["transport_scenario"]) and kwargs["travel_config"] == "TRAVEL"


@dataclass(frozen=True)
class Place:
    destination: Pose2D
    bounds: tuple = (0.0, 0.0, 20.0, 10.0)


def docking_world(tmp_path, monkeypatch, *, accepted=True, goal=(9.05, 0.02, 0.01), docking=None, straight=True,
                  dry_status="arrived", swept=True, stop_ok=True, save_error=None, d7=True):
    """A delivery match at the docking line: the scan matcher, the straight, its dry run and
    the round-2 stop check are stand-ins; the method itself is the moved one."""
    w = World(tmp_path, docking={"delivery_rear_prior": [9.0, 0.0, 0.0], "delivery_rear": [9.0, 0.0, 0.0],
                                 **(docking or {})})
    log_ = w.log
    w.slam.update(last_scan_odom_base=(8.0, 0.0, 0.0), last_sent_ranges=np.array([1.0, np.inf, 2.0]),
                  reference_points=np.zeros((2, 2)), transport_config=TrackerConfig(position_tolerance_m=0.03,
                                                                                    yaw_tolerance_rad=0.05))
    w.slam["tracker"].applied = ((0.0, 0.0, 0.0),)
    w.slam["transport_scenario"] = Place(Pose2D(9.0, 0.0, 0.0))

    def dock(reference, live, **k):
        log_("dock", len(live), **k)
        return SimpleNamespace(accepted=accepted, reason="ok" if accepted else "not_converged", correction=[0.0],
                               starts=[(accepted, "ok", (0.0, 0.0, 0.0))], start_details=[{"iterations": 3}],
                               goal_estimate=goal if accepted else None, relative_rear=(-1.0, 0.0, 0.0))

    def save_npz(name, **arrays):
        log_("save_npz", name)
        if save_error:
            raise save_error

    def straight_from_pose(start, line_start, goal_, **k):
        log_("straight", (line_start.x_m, line_start.y_m, line_start.yaw_rad), (goal_.x_m, goal_.y_m, goal_.yaw_rad))
        return (leg(8.0, goal_.x_m - 8.0) if straight else None), {"lateral_m": 0.02}

    hooks = w.hooks
    hooks.dock, hooks.save_npz, hooks.straight_from_pose = dock, save_npz, straight_from_pose
    hooks.beam_angles = lambda: np.array([-0.5, 0.0, 0.5])
    hooks.laser_mount = lambda: SimpleNamespace(xyz_m=(0.3, 0.0, 0.2), yaw_rad=0.0)
    hooks.truth_obstacles, hooks.rear = (lambda: ()), (lambda: np.array([8.0, 0.0, 0.0]))
    hooks.truth_rear = lambda: (8.0, 0.01, 0.0)
    hooks.bicycle_rollout = lambda *a: log_("rollout") or Dry(dry_status, trajectory=((8.0, 0.0, 0.0), (9.0, 0.0, 0.0)))
    hooks.write_paths = lambda: log_("write_paths")
    w.core.options.DOCKING_RETRY.round_two_stop_check = (
        lambda straight_, rear, cfg, keep, clear: log_("stop_check", keep) or (stop_ok, {"status": "arrived"}))
    opts = w.core.options
    opts.rear_axle_offset_m, opts.d7_docking, opts.grid_planning, opts.ROUND2_KEEP_M = 0.0, d7, False, 0.6
    opts.scenario = Place(Pose2D(9.0, 0.0, 0.0))
    w.core.geometry = SimpleNamespace(delivery_straight_m=1.5, loaded_footprint="FP")
    monkeypatch.setattr("forklift_core.mission.core.collision_free_pose", lambda *a: swept)
    w.core.start_docking_retry = lambda t, trigger, reason: log_("retry", trigger, reason.split(":")[0])
    return w


def test_an_accepted_first_match_moves_the_goal_and_withdraw_installs_the_straight_and_arms_round_two(
        tmp_path, monkeypatch):
    w = docking_world(tmp_path, monkeypatch)
    withdraw_before = w.core.paths["withdraw"].poses.copy()
    w.core.dock_at_delivery_straight(12.0)
    assert w.names() == ["dock", "save_npz", "straight", "rollout", "stop_check", "write_paths", "show", "prefix"]
    assert w.calls[0][2]["delivery_rear"] == (9.0, 0.0, 0.0) and w.calls[0][1] == (2,)  # the finite beams only
    assert w.calls[1][1] == ("delivery_match_1.npz",)
    assert w.calls[2][1][0] == pytest.approx((9.05 - 1.5 * np.cos(0.01), 0.02 - 1.5 * np.sin(0.01), 0.01))  # line start 1.5 m back
    d = w.slam["docking"]
    assert d["accepted_any"] is True and d["previous_goal"] == [9.05, 0.02, 0.01] and d["goal_frame"] == "held"
    assert (d["status"], d["round"], d["keep_m"]) == ("armed", 2, 0.6) and d["rounds"][0]["accepted"] is True
    assert d["match_seed"] == "prior" and d["dry_run"]["swept_clear"] is True and "to_world" in d
    # Moved exactly once by the step since the last goal (the SLAM goal), before the straight (Codex 6c P3).
    from forklift_core.localization.slam_pose import compose, invert

    step = compose((9.05, 0.02, 0.01), invert((9.0, 0.0, 0.0)))
    np.testing.assert_allclose(w.core.paths["withdraw"].poses, [compose(step, tuple(p)) for p in withdraw_before],
                               atol=1e-12)
    assert w.core.paths["transport"].length_m == pytest.approx(1.05) and w.ms.phase_started == 12.0
    assert w.state["paths_replaced"][-1]["by"] == "delivery_straight" and w.state["delivery_docking"] is d
    dest = w.slam["transport_scenario"].destination
    assert (dest.x_m, dest.y_m) == pytest.approx((9.05, 0.02)) and dest.yaw_rad == pytest.approx(0.01)
    assert w.core.trackers["transport"].config.position_tolerance_m == 0.03  # the round-2 prefix's stop


def test_a_refused_second_match_keeps_the_first_rounds_goal(tmp_path, monkeypatch):
    first = [9.05, 0.02, 0.01]
    w = docking_world(tmp_path, monkeypatch, accepted=False,
                      docking={"round": 2, "keep_m": 0.6, "previous_goal": first, "accepted_any": True,
                               "to_world": [0.0, 0.0, 0.0]})
    withdraw_before = w.core.paths["withdraw"].poses.copy()
    w.core.dock_at_delivery_straight(20.0)
    d = w.slam["docking"]
    assert d["status"] == "done" and d["previous_goal"] == first and d["rounds"][-1]["accepted"] is False
    assert d["goal_error_vs_truth"] is None and "stop_check" not in w.names()  # no round 3
    assert w.calls[2][1][1] == tuple(first)  # the straight to the kept goal
    np.testing.assert_allclose(w.core.paths["withdraw"].poses, withdraw_before, atol=1e-12)  # no step since the last round


@pytest.mark.parametrize("refusal", ["straight", "dry_run", "swept", "round_two_stop"])
def test_an_unaligned_straight_retries_from_the_committed_goal_with_its_partial_record(tmp_path, monkeypatch, refusal):
    w = docking_world(tmp_path, monkeypatch, straight=refusal != "straight",
                      dry_status="failed" if refusal == "dry_run" else "arrived", swept=refusal != "swept",
                      stop_ok=refusal != "round_two_stop")
    transport_before = w.core.paths["transport"]
    w.core.dock_at_delivery_straight(12.0)
    assert w.names()[-1] == "retry" and w.calls[-1][1] == ("b", "docking_unaligned")
    d = w.slam["docking"]
    assert d["previous_goal"] == [9.05, 0.02, 0.01] and d["goal_frame"] == "held"  # committed before the check
    assert d["offsets_to_new_line"] == {"lateral_m": 0.02} and len(d["rounds"]) == 1
    assert w.slam["transport_scenario"].destination.x_m == pytest.approx(9.05)  # the corrected drop is kept
    assert w.core.paths["transport"] is transport_before and "write_paths" not in w.names()
    if refusal == "swept":
        assert d["dry_run"]["swept_clear"] is False


def test_without_d7_an_unaligned_straight_ends_the_run(tmp_path, monkeypatch):
    w = docking_world(tmp_path, monkeypatch, straight=False, d7=False)
    with pytest.raises(RuntimeError, match="docking_unaligned"):
        w.core.dock_at_delivery_straight(12.0)
    assert "previous_goal" not in w.slam["docking"] and "match_seed" not in w.slam["docking"]


def test_the_first_match_after_a_retry_must_be_accepted(tmp_path, monkeypatch):
    w = docking_world(tmp_path, monkeypatch, accepted=False,
                      docking={"await_retry_match": True, "retry_seed_goal": [9.02, 0.0, 0.0]})
    with pytest.raises(RuntimeError, match="docking_retry_match_refused:not_converged"):
        w.core.dock_at_delivery_straight(30.0)
    d = w.slam["docking"]
    assert w.calls[0][2]["delivery_rear"] == (9.02, 0.0, 0.0) and d["match_seed"] == "latest_goal"
    assert "retry_seed_goal" not in d and d["rounds"][-1]["retry_match"] is True


def test_a_failed_match_input_write_is_noted_and_the_match_goes_on(tmp_path, monkeypatch):
    w = docking_world(tmp_path, monkeypatch, save_error=OSError("disk full"))
    w.core.dock_at_delivery_straight(12.0)
    assert w.slam["docking"]["match_input_errors"] == ["OSError: disk full"]
    assert w.slam["docking"]["status"] == "armed"
