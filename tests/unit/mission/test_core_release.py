"""M-a1 slice 6c: the SLAM releases and the obstacle escape, moved from run() closures into MissionCore
methods (behaviour, state, port requests and their order, effects up to a refusal -- the plan's M-a1
criterion; Codex 6c review P2)."""

import json
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.localization.slam_pose import compose, invert
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning import PlanResult, Pose2D


@dataclass(frozen=True)
class Scenario:
    destination: Pose2D


@dataclass(frozen=True)
class PlannerCfg:
    clearance_m: float = 0.3


def leg(x0=0.0, length=2.0, success=True, status="success"):
    poses = np.array([[x0, 0.0, 0.0], [x0 + length, 0.0, 0.0]])
    return PlanResult(success, status, poses, np.ones(2, np.int8), np.zeros(2), length, 3)


class Tracker:
    """The SLAM tracker: a release moves the control estimate by ``jump``."""

    def __init__(self, world, jump=(0.1, 0.0, 0.0), mode="holding"):
        self.world, self.jump, self.mode = world, jump, mode
        self.applied = (np.array([0.0, 0.0, 0.0]),)

    def release(self, odom_from_base):
        self.world.calls.append(("release", odom_from_base))
        self.world.rear = np.array(compose(self.jump, tuple(self.world.rear)))
        self.applied = (np.array(self.jump),)
        self.mode = "tracking"
        return float(np.hypot(*self.jump[:2])), float(self.jump[2])


class World:
    def __init__(self, *, phase="transport", obstacle=True, jump=(0.1, 0.0, 0.0), mode="holding", docking=None,
                 plans=(), grid_planning=True):
        self.calls, self.rear = [], np.array([2.0, 1.0, 0.0])
        log = self.log
        plans_ = list(plans)
        clock = iter(np.arange(0.0, 100.0, 0.25))
        self.refreshed = []
        self.obstacle = None if not obstacle else {
            "applied": (0.0, 0.0, 0.0), "version": 4, "pickup_estimate": Pose2D(5.0, 0.0, 0.0),
            "layer": SimpleNamespace(refresh=lambda *a, **k: self.refreshed.append((a, k)))}
        self.slam = {"tracker": Tracker(self, jump, mode), "holds": [], "docking": dict(docking or {}),
                     "control": [[0.0, 2.0, 1.0, 0.0]], "pending_release": phase}

        def require(ok, reason):
            log("require", bool(ok), reason)
            if not ok:
                raise RuntimeError(reason)

        self.hooks = SimpleNamespace(
            require=require, loaded=lambda: False, slam_odom_base=lambda: "ODOM",
            slam_rear=lambda t: self.rear.copy(), world=lambda: SimpleNamespace(current_time=60.0),
            path_record=lambda p: {"length_m": p.length_m}, time_module=lambda: SimpleNamespace(
                monotonic=lambda: float(next(clock))),
            add_path_display=lambda stage, path, label, color: log("show", label), stage=lambda: "STAGE",
            grid_kwargs=lambda *a, **k: log("grid_kwargs", *a, **k) or {"occupancy": "GRID"},
            grid_world=lambda sc: ("world", sc), write_paths=lambda: log("write_paths"),
            plan_transport_leg=lambda *a, **k: log("plan_transport_leg", *a, **k) or plans_.pop(0),
            plan_return_leg=lambda *a, **k: log("plan_return_leg", *a, **k) or plans_.pop(0),
            final_straight_prefix=lambda path, keep: None,
        )
        paths = {"transport": leg(), "return_home": leg(5.0), "observe": leg()}
        trackers = {n: SimpleNamespace(config=TrackerConfig(cruise_speed_mps=0.3)) for n in paths}
        self.ms = MissionState(phase=phase, phase_started=0.0)
        self.state = {"transitions": [], "paths": {}}
        self.scenario = Scenario(Pose2D(9.0, 0.0, 0.0))
        self.core = MissionCore(
            self.ms, state=self.state, obstacle=self.obstacle, slam=self.slam,
            pocket={"check": None, "planning_pickup": Pose2D(5.1, 0.0, 0.0)}, paths=paths, trackers=trackers,
            geometry="GEO", settings={}, pallet_geometry=None, hooks=self.hooks, near={},
            options=SimpleNamespace(initial_time=50.0, grid_planning=grid_planning, planner_config=PlannerCfg(),
                                    return_to_pose="HOME", scenario=self.scenario, travel_config=None,
                                    near_tracking=SimpleNamespace(RETRY_STAND_M=0.05)))
        self.core.arm_docking = lambda: log("arm_docking")

    def log(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def names(self):
        return [c[0] for c in self.calls if c[0] != "require"]


def test_release_and_stand_carries_the_estimates_reprojects_and_installs_the_two_pose_stand():
    w = World(phase="approach")
    record = w.core.release_and_stand(12.0, "release_pickup_retry")
    jump = (0.1, 0.0, 0.0)
    assert record["jump_m"] == pytest.approx(0.1) and record["rear_before"] == [2.0, 1.0, 0.0]
    assert record["rear_after"] == pytest.approx([2.1, 1.0, 0.0]) and record["applied_before"] == [0.0, 0.0, 0.0]
    assert record["frame_change"] == pytest.approx(list(jump))
    assert record["reprojected"] == {"version": 5, "applied": [0.1, 0.0, 0.0], "reprojections": 1}
    (args, kwargs), = w.refreshed
    assert args == (12.0, (0.1, 0.0, 0.0), 5) and kwargs["current_pose"] == pytest.approx((2.1, 1.0, 0.0))
    # The pickup estimate and the planning target ride along with the frame change.
    assert w.obstacle["pickup_estimate"].x_m == pytest.approx(5.1)
    assert w.core.pocket["planning_pickup"].x_m == pytest.approx(5.2)
    stand = w.core.paths["observe"]
    np.testing.assert_allclose(stand.poses, [[2.05, 1.0, 0.0], [2.1, 1.0, 0.0]])
    assert w.core.trackers["observe"].config.cruise_speed_mps == 0.3 and w.core.near["retry_stand"] is True
    assert w.slam["holds"] == [{"phase": "approach", "time_s": 12.0, "event": "release_pickup_retry",
                                "jump_m": pytest.approx(0.1), "jump_rad": 0.0}]
    w = World(phase="approach", obstacle=False)
    assert w.core.release_and_stand(12.0, "release_near_realign")["reprojected"] is None


def test_a_backoff_end_releases_a_hold_reprojects_and_rewrites_the_control_row():
    w = World(phase="observe")
    rear = w.core.release_after_backoff(20.0, "release_after_backoff")
    np.testing.assert_allclose(rear, [2.1, 1.0, 0.0])
    assert w.slam["control"][-1] == pytest.approx([0.0, 2.1, 1.0, 0.0])
    assert w.slam["holds"][-1]["event"] == "release_after_backoff" and w.obstacle["version"] == 5
    np.testing.assert_allclose(w.refreshed[0][1]["path_ahead"], [[2.1, 1.0, 0.0]])
    # Not without a hold, nor while the delivery docking is accepted or re-approaching.
    assert World(mode="tracking").core.release_after_backoff(20.0, "e") is None
    for docking in ({"accepted_any": True}, {"reapproach": True}):
        w = World(phase="transport", docking=docking)
        assert w.core.release_after_backoff(20.0, "e") is None and w.calls == []
    w = World(phase="return_home", docking={"accepted_any": True})  # the return is not guarded
    assert w.core.release_after_backoff(20.0, "e") is not None


def test_a_small_jump_on_the_transport_only_arms_the_docking():
    w = World(phase="transport", jump=(0.01, 0.0, 0.005))
    w.core.slam_release(30.0)
    assert w.names() == ["release", "arm_docking"] and w.slam["pending_release"] is None
    assert w.slam["holds"][-1]["replanned"] is False


def test_a_large_jump_replans_the_transport_with_the_released_correction_then_arms_the_docking():
    w = World(phase="transport", plans=[leg(2.1, 3.0)])
    w.core.slam_release(30.0)
    assert w.names() == ["release", "grid_kwargs", "plan_transport_leg", "write_paths", "show", "arm_docking"]
    _, args, kwargs = w.calls[1]
    assert args == (None,) and kwargs["applied"] == pytest.approx((0.1, 0.0, 0.0))
    assert np.allclose(kwargs["own"], [2.1, 1.0, 0.0])
    event = w.slam["holds"][-1]
    assert event["replanned"] is True and event["replaced_path"] == {"length_m": 2.0} and event["planning_wall_s"] == 0.25
    assert w.core.paths["transport"].length_m == 3.0 and w.state["paths"]["transport"] == {"length_m": 3.0}
    assert w.ms.phase_started == 30.0


def test_after_a_docked_drop_even_no_jump_replans_the_return_and_carries_the_pallet():
    w = World(phase="return_home", jump=(0.0, 0.0, 0.0), docking={"accepted_any": True}, plans=[leg(2.0, 4.0)])
    w.slam["transport_scenario"] = w.scenario
    w.core.slam_release(40.0)
    assert "plan_return_leg" in w.names() and "arm_docking" not in w.names()
    event = w.slam["holds"][-1]
    assert event["pallet_frame_change"] == pytest.approx([0.0, 0.0, 0.0])
    assert w.slam["return_scenario"].destination == Pose2D(9.0, 0.0, 0.0)
    w = World(phase="return_home", jump=(0.05, 0.0, 0.0), docking={"accepted_any": True}, plans=[leg(2.0, 4.0)])
    w.slam["transport_scenario"] = w.scenario
    w.core.slam_release(40.0)
    dest = w.slam["return_scenario"].destination
    expected = compose(compose((2.05, 1.0, 0.0), invert((2.0, 1.0, 0.0))), (9.0, 0.0, 0.0))
    assert (dest.x_m, dest.y_m, dest.yaw_rad) == pytest.approx(expected)
    (_, args, _), = [c for c in w.calls if c[0] == "plan_return_leg"]
    assert args[0] == ("world", w.slam["return_scenario"]) and args[2] == "HOME"


def test_an_invalid_start_is_retried_tight_and_a_failed_replan_ends_the_run_with_its_record():
    w = World(phase="transport", plans=[leg(success=False, status="invalid_start"), leg(2.1, 3.0)])
    w.core.slam_release(30.0)
    plans = [c for c in w.calls if c[0] == "plan_transport_leg"]
    assert len(plans) == 2 and plans[1][1][2].clearance_m == 0.0 and w.slam["holds"][-1]["tight_retry"] is True
    w = World(phase="transport", plans=[leg(success=False, status="no_path")])
    with pytest.raises(RuntimeError, match="slam_release_replan_failed: no_path"):
        w.core.slam_release(30.0)
    event = w.slam["holds"][-1]
    assert event["replan_status"] == "no_path" and event["replanned"] is True  # recorded before the refusal
    assert w.core.paths["transport"].length_m == 2.0 and "write_paths" not in w.names()  # nothing installed
    w = World(phase="transport", plans=[leg(success=False, status="invalid_start")], grid_planning=False)
    with pytest.raises(RuntimeError, match="invalid_start"):
        w.core.slam_release(30.0)  # no tight retry without the grid


def test_the_escape_leg_takes_each_legs_rectangles_from_its_own_scenario(monkeypatch):
    seen = []
    monkeypatch.setattr("forklift_core.planning.pallet_mission.leg_obstacles",
                        lambda kind, world, geometry: seen.append((kind, world)) or ("RECTS", "FP", "BOUNDS"))
    monkeypatch.setattr("forklift_core.planning.grid_collision.FootprintCollisionChecker",
                        lambda r, f, b: ("CHECKER", r))
    w = World()
    w.slam["transport_scenario"] = "TRANSPORT_SC"
    w.slam["return_scenario"] = SimpleNamespace(destination="DROP")
    assert w.core.escape_leg("transport") == ("transport", (None, None), ("CHECKER", "RECTS"), ("RECTS", "FP", "BOUNDS"))
    kind, args, _, _ = w.core.escape_leg("return_home")
    assert (kind, args) == ("return", ("return", "DROP"))
    assert seen == [("transport", ("world", "TRANSPORT_SC")), ("return", ("world", w.slam["return_scenario"]))]


def test_the_escape_checks_its_candidates_with_the_permission_and_dumps_the_first_six_decisions(monkeypatch):
    import forklift_core.control.escape as esc

    asked = {}

    def evaluate(pose, *, sweep_free, path_blocked, arc_blocked, end_clearance, **kw):
        asked.update(kw, pose=pose, path=path_blocked(["P"], -1, 0.02, 0.1), arc=arc_blocked(0.3, -1),
                     end=end_clearance([(1.0, 1.0, 0.0), (1.1, 1.0, 0.0)]))
        return [SimpleNamespace(record=lambda: {"d": -1})]

    monkeypatch.setattr(esc, "evaluate", evaluate)
    monkeypatch.setattr(esc, "choose", lambda cands, d: cands[0] if d == 1 else None)
    monkeypatch.setattr(esc, "clearance_m", lambda free, p: 0.1)
    monkeypatch.setattr("forklift_core.planning.grid_collision.make_checker", lambda *a: SimpleNamespace(free=None))
    w = World(phase="transport")
    w.core.escape_leg = lambda phase: ("transport", (None, None), "RC", ("RECTS", "FP", "BOUNDS"))
    checks = []
    layer = SimpleNamespace(
        permission=SimpleNamespace(config=SimpleNamespace(stopping=SimpleNamespace(distance_m=lambda v: 0.2),
                                                          step_m=0.05)),
        error_table=SimpleNamespace(at=lambda age: (0.0, 0.004)), grid_config=SimpleNamespace(occupied_max_age_s=1.0),
        check_candidate=lambda poses, **k: checks.append(("path", k)) or SimpleNamespace(blocked="occupied"),
        check_arc_candidate=lambda **k: checks.append(("arc", k)) or SimpleNamespace(blocked=None),
        snapshot=SimpleNamespace(state=0, free_stamp=0, origin_x_m=0.0, origin_y_m=0.0, resolution_m=0.05))
    w.obstacle["layer"] = layer
    grid = SimpleNamespace(occupied=np.zeros((2, 2), bool), origin_x_m=0.0, origin_y_m=0.0, resolution_m=0.05)
    w.hooks.grid_kwargs = lambda *a, **k: w.log("grid_kwargs", *a, **k) or {"occupancy": grid}
    w.core.options.backoff_speed_mps = 0.1
    dumps = []
    w.hooks.save_npz = lambda name, **arrays: dumps.append((name, arrays))
    choice, candidates = w.core.find_escape(25.0, (2.0, 1.0, 0.0), 1, 0.2)
    assert choice is candidates[0]
    assert asked["stop_length_m"] == pytest.approx(0.25) and asked["psi_rad"] == 0.004 and asked["kappa_now"] == 0.2
    assert asked["planner_clearance_m"] == 0.3  # the planner's clearance (no travel config)
    assert asked["path"] == "occupied" and asked["arc"] is None and asked["end"] == (True, 0.1)
    assert checks[0] == ("path", {"current_pose": (2.0, 1.0, 0.0), "direction": -1, "loaded": False,
                                  "extra_margin_m": 0.02, "extra_ramp_m": 0.1})
    assert checks[1] == ("arc", {"current_pose": (2.0, 1.0, 0.0), "curvature_inv_m": 0.3, "direction": -1,
                                 "loaded": False, "cap_mps": 0.1})
    grids = [c for c in w.calls if c[0] == "grid_kwargs"]
    assert grids[0][2] == {"label": "escape"} and grids[1][2]["label"] == "escape_end"
    assert grids[1][2]["own"] == [(1.0, 1.0, 0.0), (1.1, 1.0, 0.0)] == grids[1][2]["band_pose"]
    (name, arrays), = dumps
    assert name == "obstacle_recovery_1.npz" and arrays["phase"] == "transport" and arrays["leg_dir"] == 1
    assert json.loads(arrays["chosen"]) == {"d": -1} and w.obstacle["recovery_dumps"] == 1
    w.obstacle["recovery_dumps"] = 6
    w.core.find_escape(26.0, (2.0, 1.0, 0.0), -1, 0.2)
    assert len(dumps) == 1  # six dumps at most


def test_a_failed_escape_dump_propagates_after_the_dump_count_moved():
    """The recovery dump is not guarded (unlike the delivery match's): a Sink error ends the run, with
    the count already taken -- as before the move."""
    import forklift_core.control.escape as esc

    w = World()
    w.core.escape_leg = lambda phase: ("transport", (None, None), "RC", ("RECTS", "FP", "BOUNDS"))
    grid = SimpleNamespace(occupied=np.zeros((2, 2), bool), origin_x_m=0.0, origin_y_m=0.0, resolution_m=0.05)
    w.hooks.grid_kwargs = lambda *a, **k: {"occupancy": grid}
    w.core.options.backoff_speed_mps = 0.1
    w.obstacle["layer"] = SimpleNamespace(
        permission=SimpleNamespace(config=SimpleNamespace(stopping=SimpleNamespace(distance_m=lambda v: 0.2),
                                                          step_m=0.05)),
        error_table=SimpleNamespace(at=lambda age: (0.0, 0.004)), grid_config=SimpleNamespace(occupied_max_age_s=1.0),
        snapshot=SimpleNamespace(state=0, free_stamp=0, origin_x_m=0.0, origin_y_m=0.0, resolution_m=0.05))

    def save_npz(name, **arrays):
        raise OSError("disk full")

    w.hooks.save_npz = save_npz
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(esc, "evaluate", lambda *a, **k: [])
        mp.setattr(esc, "choose", lambda c, d: None)
        with pytest.raises(OSError, match="disk full"):
            w.core.find_escape(25.0, (2.0, 1.0, 0.0), 1, 0.0)
    assert w.obstacle["recovery_dumps"] == 1


def test_the_return_replan_after_a_release_is_retried_tight_on_the_carried_destination():
    """M-a2 core sweep: the return leg's invalid start is retried at zero clearance, on the return scenario."""
    w = World(phase="return_home", jump=(0.05, 0.0, 0.0), docking={"accepted_any": True},
              plans=[leg(success=False, status="invalid_start"), leg(2.0, 4.0)])
    w.slam["transport_scenario"] = w.scenario
    w.core.slam_release(40.0)
    plans = [c for c in w.calls if c[0] == "plan_return_leg"]
    assert len(plans) == 2 and plans[1][1][3].clearance_m == 0.0 and w.slam["holds"][-1]["tight_retry"] is True
    assert plans[1][1][0] == ("world", w.slam["return_scenario"]) and plans[1][1][2] == "HOME"
