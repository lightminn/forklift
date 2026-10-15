"""M-a1 slice 3a: the live travel leg and the replan wait moved from the runner (behaviour, state, port requests
and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState


@dataclass(frozen=True)
class Config:
    clearance_m: float = 0.3


@dataclass
class Plan:
    status: str
    poses: np.ndarray = field(default_factory=lambda: np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
    directions: np.ndarray = field(default_factory=lambda: np.array([1, 1]))
    curvatures_inv_m: np.ndarray = field(default_factory=lambda: np.zeros(2))
    turnaround: object = None

    @property
    def success(self):
        return self.status == "success"


def make(phase="transport", *, statuses=("success",), slam=None, grid_planning=True, turnaround=False,
         travel_config=Config(), obstacle=None):
    calls = []
    plans = iter(Plan(s) for s in statuses)

    def planner(name):
        def plan(*args, **kwargs):
            calls.append((name, args, kwargs))
            return next(plans)
        return plan

    def require(ok, reason):
        calls.append(("require", (bool(ok), reason), {}))
        if not ok:
            raise RuntimeError(reason)

    hooks = SimpleNamespace(
        echo=lambda *a: None, record_json=str, snapshot=lambda *a: None, require=require,
        wall=lambda: calls.append(("wall", (), {})) or 100.0,
        grid_world=lambda sc: ("world", sc), grid_kwargs=lambda *a, **k: calls.append(("grid_kwargs", a, k)) or {"grid": a},
        return_turnaround_kwargs=lambda: {"turn": 1},
        plan_transport_leg=planner("transport"), plan_return_leg=planner("return"),
        path_record=lambda path: {"status": path.status}, write_paths=lambda: calls.append(("write_paths", (), {})),
        arm_docking=lambda: calls.append(("arm_docking", (), {})),
    )
    tracker = SimpleNamespace(config=TrackerConfig(), leg_ahead=lambda: ("LEG", 1))
    if obstacle is None:
        layer = SimpleNamespace(refresh=lambda *a, **k: calls.append(("refresh", a, k)),
                                permission=SimpleNamespace(path_check=None))
        obstacle = {"layer": layer, "applied": (0.0, 0.0, 0.0), "version": 3}
    ms = MissionState(phase=phase, phase_started=1.0)
    state = {"transitions": [], "paths": {}}
    scenario = SimpleNamespace(destination="DEST")
    options = SimpleNamespace(grid_planning=grid_planning, return_turnaround=turnaround, planner_config=Config(),
                              travel_config=travel_config, scenario=scenario, return_to_pose="HOME")
    trackers = {phase: tracker} if phase in ("transport", "return_home") else {}
    core = MissionCore(ms, state=state, obstacle=obstacle, slam=slam, pocket={"check": None}, paths={},
                       trackers=trackers, geometry="G", settings={}, pallet_geometry=None, hooks=hooks,
                       options=options)
    # Docking/withdrawal helpers are MissionCore methods since slice 6b; this test's stand-ins go on the instance.
    for name_ in ['dock_at_delivery_straight', 'arm_docking', 'release_and_carry', 'transport_replan', 'start_docking_retry']:
        if hasattr(hooks, name_):
            setattr(core, name_, getattr(hooks, name_))
    return core, ms, state, obstacle, calls, scenario


REAR = np.array([1.0, 2.0, 0.5])


def test_a_live_transport_leg_replaces_the_follower_restarts_the_phase_and_stands_this_tick():
    core, ms, state, obstacle, calls, scenario = make("transport", slam=None)
    old = core.trackers["transport"]
    assert core.live_leg(20.0, REAR, True) is True
    names = [c[0] for c in calls]
    assert names == ["wall", "grid_kwargs", "transport", "write_paths"]
    _, args, kwargs = calls[2]
    assert args[0] == ("world", scenario) and (args[1].x_m, args[1].y_m, args[1].yaw_rad) == (1.0, 2.0, 0.5)
    assert kwargs == {"geometry": "G", "travel_config": Config(), "grid": (None,)}
    assert obstacle["deadline"] is None and obstacle["live_replanned"] == {"transport": True}
    assert obstacle["live_plans"] == [{"phase": "transport", "time_s": 20.0, "status": "success", "turnaround": None}]
    assert core.paths["transport"].status == "success" and state["paths"] == {"transport": {"status": "success"}}
    assert core.trackers["transport"] is not old and core.trackers["transport"].config is old.config
    assert ms.phase_started == 20.0
    assert core.live_leg(21.0, REAR, True) is False  # planned once: no second plan
    assert names == [c[0] for c in calls]


def test_an_invalid_start_retries_at_zero_clearance_and_transport_under_slam_arms_the_docking():
    slam = {"transport_scenario": "SC_T", "pending_release": None}
    core, _, _, _, calls, _ = make("transport", statuses=("invalid_start", "success"), slam=slam)
    core.live_leg(20.0, REAR, True)
    plans = [c for c in calls if c[0] == "transport"]
    assert [p[1][0] for p in plans] == [("world", "SC_T")] * 2
    assert plans[0][1][2] == Config() and plans[1][1][2] == Config(clearance_m=0.0)
    assert plans[1][2]["travel_config"] == Config(clearance_m=0.0)
    assert [c[0] for c in calls][-2:] == ["write_paths", "arm_docking"]
    core, _, _, _, calls, _ = make("transport", statuses=("no_path", "success"), slam=None, travel_config=None)
    core.live_leg(20.0, REAR, True)
    assert [c[2]["travel_config"] for c in calls if c[0] == "transport"] == [None, None]


def test_a_failed_live_plan_retries_after_a_second_and_refuses_on_the_fifth():
    core, ms, _, obstacle, calls, _ = make("transport", statuses=("no_path", "no_path") * 5)
    assert core.live_leg(20.0, REAR, True) is True
    assert obstacle["live_replanned"] == {"transport": False} and obstacle["live_retry_after_s"] == 21.0
    assert core.live_leg(20.5, REAR, True) is True  # waiting to retry: stand, no plan
    assert sum(1 for c in calls if c[0] == "transport") == 2
    for k in range(3):
        core.live_leg(21.0 + k, REAR, True)
    with pytest.raises(RuntimeError, match="obstacle_live_plan_failed in transport: no_path"):
        core.live_leg(24.0, REAR, True)
    assert len(obstacle["live_plans"]) == 5 and obstacle["live_replanned"] == {"transport": True}
    assert ms.phase_started == 1.0


def test_the_return_leg_plans_home_on_the_return_scenario_with_the_turnaround():
    slam = {"return_scenario": SimpleNamespace(destination="D2"), "transport_scenario": "SC_T",
            "pending_release": None, "tracker": SimpleNamespace(applied=None)}
    core, _, _, obstacle, calls, _ = make("return_home", slam=slam)
    core.live_leg(30.0, REAR, False)
    (_, args, kwargs), = [c for c in calls if c[0] == "return"]
    assert args[0] == ("world", slam["return_scenario"]) and args[2:] == ("HOME", Config())
    assert kwargs == {"geometry": "G", "travel_config": Config(), "grid": ("return", "D2"), "turn": 1}
    assert "arm_docking" not in [c[0] for c in calls]


def test_the_turnaround_waits_for_the_release_and_reprojects_a_changed_correction_first():
    tracker_ = SimpleNamespace(applied=None)
    slam = {"pending_release": "return_home", "tracker": tracker_}
    core, _, _, obstacle, calls, _ = make("return_home", slam=slam, turnaround=True)
    assert core.live_leg(30.0, REAR, False) is True  # not planned yet: stands waiting for the plan
    assert calls == []
    slam["pending_release"] = None
    tracker_.applied = (np.array([0.1, 0.0, 0.01]),)
    core.live_leg(31.0, REAR, False)
    assert [c[0] for c in calls][:3] == ["refresh", "wall", "grid_kwargs"]
    _, args, kwargs = calls[0]
    assert args == (31.0, (0.1, 0.0, 0.01), 4) and kwargs == {
        "current_pose": (1.0, 2.0, 0.5), "path_ahead": "LEG", "loaded": False, "direction": 1}
    assert (obstacle["applied"], obstacle["reprojections"], obstacle["return_reprojected_at_s"]) == (
        (0.1, 0.0, 0.01), 1, 31.0)


def test_no_live_plan_outside_travel_or_without_grid_planning():
    for phase, grid in (("approach", True), ("transport", False)):
        core, _, _, obstacle, calls, _ = make(phase, grid_planning=grid)
        assert core.live_leg(20.0, REAR, True) is False and calls == []


def test_a_waiting_replan_resumes_once_the_path_ahead_verifies_and_stands_until_then():
    layer = SimpleNamespace(permission=SimpleNamespace(path_check=SimpleNamespace(blocked="occupied")))
    obstacle = {"layer": layer, "replan_wait": {"approach": True}}
    core, _, _, _, _, _ = make("approach", obstacle=obstacle)
    assert core.live_leg(20.0, REAR, False) is True and obstacle["replan_wait"] == {"approach": True}
    layer.permission.path_check = None  # nothing verified yet
    assert core.live_leg(20.1, REAR, False) is True
    layer.permission.path_check = SimpleNamespace(blocked=None)
    obstacle["backoff"] = {"phase": "approach"}  # still backing off: not the existing path
    assert core.live_leg(20.2, REAR, False) is True
    obstacle["backoff"] = {"phase": "observe"}
    assert core.live_leg(20.3, REAR, False) is False and obstacle["replan_wait"] == {}
