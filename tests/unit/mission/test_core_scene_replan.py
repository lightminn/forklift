"""The scene-mode replan device inside the core (docs/plans/2026-10-23-scene-replan.md 2.1-2.4): real trackers and
the real leg planner on an open floor, the core's ports, the cusp latch's end turned into a request, the follower's
early return, the worker's drops, the mission end."""

from dataclasses import dataclass
import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control import RearAxlePathTracker, ackermann_command
from forklift_core.control.path_tracking import AckermannGeometry
from forklift_core.mission.core import MissionCore
from forklift_core.mission.failures import MISSION, classify
from forklift_core.mission.state import MissionState
from forklift_core.planning import Bounds, Pose2D
from forklift_core.planning.pallet_mission import (
    PalletSite,
    SyntheticMissionGeometry,
    TransportScenario,
    make_transport_planner_config,
    plan_return_leg,
    plan_transport_leg,
)

DT = 1 / 120
DRIVE = AckermannGeometry(wheelbase_m=0.66, track_m=0.53, wheel_radius_m=0.125, max_steering_rad=0.2618,
                          max_wheel_rate_rad_s=8.0)


@dataclass(frozen=True)
class Tracking:
    status: str = "tracking"
    speed_mps: float = 0.0


def _scene():
    return TransportScenario(0, Pose2D(-3.0, 0.0, 0.0), PalletSite(0.0, 0.0, 0.0), PalletSite(6.0, 0.0, 0.0), (),
                             Bounds(-6.0, 10.0, -5.0, 5.0))


def make(*, scene=True, jobs=None, held=None):
    geometry = SyntheticMissionGeometry()
    config = make_transport_planner_config(curvature_limit_inv_m=0.29, clearance_m=0.10)
    scenario = _scene()
    calls = []
    held = held if held is not None else {"kappa": 0.0}

    def require(ok, reason):
        calls.append(("require", bool(ok), reason))
        if not ok:
            raise RuntimeError(reason)

    rear = {"pose": np.array([2.0, 0.3, 0.05])}
    hooks = SimpleNamespace(
        require=require, grid_world=lambda sc: sc, path_record=lambda p: {"status": p.status, "n": len(p.poses)},
        write_paths=lambda: calls.append(("write_paths",)), set_rear=lambda r: calls.append(("set_rear", tuple(r))),
        held_kappa=lambda: held["kappa"], loaded=lambda: True, rear=lambda: rear["pose"],
        steering_actual=lambda: ackermann_command(0.0, held["kappa"], DRIVE).steering_rad,
        truth_rear=lambda: rear["pose"] + 0.01, dump_tracking=lambda *a: calls.append(("dump",) + a),
        plan_transport_leg=lambda *a, **k: (calls.append(("plan_transport_leg",)), plan_transport_leg(*a, **k))[1],
        plan_return_leg=lambda *a, **k: (calls.append(("plan_return_leg",)), plan_return_leg(*a, **k))[1],
        wall=lambda: 0.0,
    )
    start = Pose2D(0.5, 0.0, 0.0)
    first = plan_transport_leg(scenario, start, config, geometry=geometry)
    assert first.success
    tracker = RearAxlePathTracker(first.poses, first.directions, first.curvatures_inv_m, config_tracker())
    ms = MissionState(phase="transport", phase_started=0.0)
    state = {"paths": {"transport": {"status": "success"}}, "transitions": []}
    options = SimpleNamespace(pickup_prior=scene, scenario=scenario, planner_config=config, travel_config=None,
                              return_to_pose=scenario.start_rear, drive_geometry=DRIVE,
                              backoff_speed_mps=0.1, dwells=None, safety_stops=None,
                              cusp_heading_hold_phases=("observe", "return_home"))
    core = MissionCore(ms, state=state, obstacle=None, slam=None, pocket={"check": None},
                       paths={"transport": first}, trackers={"transport": tracker}, geometry=geometry,
                       settings={"drive_acceleration_mps2": 0.3}, pallet_geometry=None, hooks=hooks, options=options)
    core.jobs = jobs
    return core, ms, state, calls, rear, held


def config_tracker():
    from forklift_core.control import TrackerConfig

    return TrackerConfig(cruise_speed_mps=0.3, max_curvature_inv_m=0.36)


def _drive(core, rear, held, *, n=200, t0=10.0):
    """Ticks of the device; the wheels take a presteer target at once (held kappa follows it)."""
    outs = []
    for k in range(n):
        out = core.scene_replan_tick(t0 + k * DT, rear["pose"], 0.0, DT)
        outs.append(out)
        if out.owned and out.output == "presteer":
            held["kappa"] = out.curvature
        if not out.owned:
            break
    return outs


def test_a_raised_request_replans_the_leg_from_the_stop_and_hands_back_batch():
    core, ms, state, calls, rear, held = make(held={"kappa": 0.3})  # the wheels turned the other way at the stall
    old_tracker = core.trackers["transport"]
    assert core.scene_replan_raise(10.0, "cusp_pose", rear["pose"], deviation={"position_m": 0.075}) is None
    assert core.scene_raised and state["scene_replans"][0]["trigger"] == "cusp_pose"
    outs = _drive(core, rear, held)
    assert outs[-1].owned is False and any(o.output == "presteer" for o in outs)
    assert core.trackers["transport"] is not old_tracker
    assert ("plan_transport_leg",) in calls and ("write_paths",) in calls
    rec = state["scene_replans"][-1]
    assert rec["outcome"] == "installed" and rec["presteer"] == "aligned" and rec["candidate"]["status"] == "tracking"
    assert rec["budget_left"] == {"total": 3, "cusp_pose": 2}
    assert state["paths_replaced"][-1]["by"] == "scene_replan"
    assert rec["truth_rear"] == pytest.approx(list(rear["pose"] + 0.01))


def test_the_device_does_nothing_outside_a_scene_and_a_spent_budget_ends_the_mission():
    core, ms, state, *_ = make(scene=False)
    assert not core.scene_replan_allowed()
    core, ms, state, calls, rear, held = make()
    for k in range(3):
        assert core.scene_replan_raise(10.0 + k, "cusp_pose", rear["pose"]) is None
        _drive(core, rear, held, t0=10.0 + k)
    reason = core.scene_replan_raise(20.0, "cusp_pose", rear["pose"])
    assert reason == "scene_replan_exhausted:cusp_pose" and classify(reason) == MISSION
    core.end_reset = MissionCore.end_reset.__get__(core)  # the mission's end resets the budget
    core.scene_replan.reset()
    assert core.scene_replan_raise(30.0, "cusp_pose", rear["pose"]) is None


def test_a_cusp_stall_ended_on_its_pose_is_a_request_in_a_scene_and_ends_elsewhere():
    for scene in (True, False):
        core, ms, state, calls, rear, held = make(scene=scene)
        latch = SimpleNamespace(record={"position_error_m": 0.0746, "yaw_error_rad": -0.0524})
        monitor = SimpleNamespace(latch=latch, tick=lambda *a, **k: SimpleNamespace(
            tracking=Tracking(), hold=True, end="cusp_position_stall", force_replan=False))
        _bind_cusp(core, monitor)
        tracking = Tracking()
        if scene:
            out = core.cusp_tick(10.0, rear["pose"], tracking, tracking, False)
            assert out[1] is True and core.scene_raised
            assert state["scene_replans"][-1]["deviation"]["position_m"] == pytest.approx(0.0746)
        else:
            with pytest.raises(RuntimeError, match="cusp_position_stall"):
                core.cusp_tick(10.0, rear["pose"], tracking, tracking, False)


def _bind_cusp(core, monitor):
    from dataclasses import replace as dc_replace  # noqa: F401

    core.hooks.cusp_monitor = lambda: monitor
    core.hooks.cusp_standing = lambda: True
    core.hooks.signed_speed = lambda: 0.0
    for name in ("cusp_alive", "cusp_end_recovered", "cusp_excluded", "cusp_leg_checker", "cusp_ownership",
                 "cusp_scan_id", "cusp_sensors", "cusp_stopped"):
        setattr(core, name, lambda *a, **k: None)
    core.options.dt = DT
    core.options.grid_planning = False


def test_an_unconfirmed_stall_still_ends():
    core, ms, state, calls, rear, held = make()
    monitor = SimpleNamespace(latch=None, tick=lambda *a, **k: SimpleNamespace(
        tracking=Tracking(), hold=True, end="cusp_position_stall_unconfirmed", force_replan=False))
    _bind_cusp(core, monitor)
    t_ = Tracking()
    with pytest.raises(RuntimeError, match="cusp_position_stall_unconfirmed"):
        core.cusp_tick(10.0, rear["pose"], t_, t_, False)


class Jobs:
    def __init__(self):
        self.submitted = []

    def submit(self, name, payload):
        self.submitted.append((name, payload))
        return len(self.submitted)

    def discard(self, job):
        pass


def test_live_the_worker_plans_a_drop_is_asked_again_outside_the_shared_count():
    core, ms, state, calls, rear, held = make(jobs=Jobs())
    state["async_plans"] = []
    core.commands.begin_wait = lambda *a, **k: None
    core.commands.end_wait = lambda *a, **k: None
    core.scene_replan_raise(10.0, "cusp_pose", rear["pose"])
    core.scene_replan_tick(10.0, rear["pose"], 0.0, DT)
    assert core.jobs.submitted[-1][0] == "leg" and core.pending.site == "scene_replan"
    payload = core.jobs.submitted[-1][1]
    assert payload["retry_on"] == () and "deadline" in payload["kwargs"]  # no tight retry, its own deadline
    core._drop(core.pending, 10.1, "correction", count=True)
    assert state.get("async_plan_drops", {}).get("scene_replan") is None
    core.scene_replan_tick(10.1, rear["pose"], 0.0, DT)  # the drop read: back to wait for the jobs
    core.scene_replan_tick(10.2, rear["pose"], 0.0, DT)  # asked again
    assert len(core.jobs.submitted) == 2 and state["scene_replans"][-1]["drops"] == 1
    sc, start, cfg = payload["args"]
    result = plan_transport_leg(sc, start, cfg, geometry=core.geometry)
    core._scene_finish(core.pending, {"result": result}, 10.3, rear["pose"])
    core.pending = None
    outs = _drive(core, rear, held, t0=10.3)
    assert outs[-1].owned is False and state["scene_replans"][-1]["outcome"] == "installed"


def test_the_follower_returns_at_once_on_its_own_tick_after_a_raise():
    core, ms, state, calls, rear, held = make()

    def raising_cusp(t, rear_, raw, tracking, stand):
        core.scene_replan_raise(t, "cusp_pose", rear_)
        return tracking, True

    core.cusp_tick = raising_cusp
    core.hooks.cusp_holds = lambda: SimpleNamespace(sync=lambda *a: None, tick=lambda *a: None)
    core.hooks.slam_odom_base = lambda: None
    core.hooks.slam_error = lambda: None
    core.cusp_heading_end = lambda tr: {}
    core.cusp_heading_offset = lambda *a: None
    out = core.follow(1.0, rear["pose"], 0.0, DT)
    assert out.scene_raised is True and out.cusp_hold is True
    out2 = SimpleNamespace(**vars(out))
    assert out2.tracking is not None


def test_the_correction_contribution_is_kept_per_gear_leg():
    core, ms, state, calls, rear, held = make()
    core.scene_correction_note(1.0, 0.02, 0.01)
    core.scene_correction_note(1.1, 0.064, 0.05, 0.01)
    core.scene_correction_note(1.2, 0.03, 0.02)
    c = core.scene_correction
    assert c["net_max_m"] == pytest.approx(0.064) and c["net_at_s"] == 1.1 and c["reply_max_m"] == pytest.approx(0.05)
    core.trackers["transport"] = RearAxlePathTracker(np.array([[0, 0, 0], [1, 0, 0]], float), np.array([1, 1]),
                                                     np.zeros(2), config_tracker())
    core.scene_correction_note(2.0, 0.001, 0.001)
    assert core.scene_correction["net_max_m"] == pytest.approx(0.001)  # a new gear leg starts again
    core.scene_replan_raise(2.1, "cusp_pose", rear["pose"])
    jump = state["scene_replans"][-1]["correction_jump"]["slam"]
    assert jump["net_max_m"] == pytest.approx(0.001) and "key" not in jump


# --- B: the first far observation's stall (plan 2.3) -----------------------------------------------------------------
def _observe(core, ms, state, *, waypoints, next_index=1, scene=True):
    from forklift_core.planning.pallet_mission import plan_observation_leg

    core.options.pickup_prior = scene
    core.options.observation_waypoints = waypoints
    core.options.observe_bounds = None
    core.options.pickup_bounds = None
    core.hooks.plan_observation_leg = plan_observation_leg
    core.hooks.grid_kwargs = lambda *a, **k: {}
    core.realign_active = lambda: False
    ms.phase = "observe"
    ms.next_candidate_index = next_index
    core.trackers["observe"] = core.trackers["transport"]
    state.update(observation_candidates=[], observe_replans=[], observe_replan_leg={"serial": 3, "start": 0})


def test_a_far_observation_stall_takes_the_next_candidate_with_its_own_budget():
    core, ms, state, calls, rear, held = make()
    _observe(core, ms, state, waypoints=[(-2.0, 1.0, 0.0), (-1.5, -1.5, 0.3), (-2.5, 2.0, 0.0)])
    assert core.scene_far_observe()
    old = core.trackers["observe"]
    took = core.scene_observe_candidate(5.0, np.array([-2.2, 0.2, 0.0]))
    assert took.installed and core.trackers["observe"] is not old and ms.next_candidate_index == 2
    assert state["observation_waypoint_selected"] == [-1.5, -1.5, 0.3]
    assert state["observe_replan_leg"] == {"serial": 4, "start": 0}
    rec = state["scene_replans"][-1]
    assert rec["trigger"] == "observe_stall" and rec["outcome"] == "installed"
    ms.next_candidate_index = 3
    none = core.scene_observe_candidate(6.0, np.array([-2.2, 0.2, 0.0]))  # none left: the pose back, nothing tried
    assert not none.installed and not none.tried and none.rear.tolist() == [-2.2, 0.2, 0.0]


def test_the_far_observation_rule_is_a_scene_s_before_recognition_only():
    core, ms, state, calls, rear, held = make()
    _observe(core, ms, state, waypoints=[(-2.0, 1.0, 0.0)], scene=False)
    assert not core.scene_far_observe()
    core.options.pickup_prior = True
    assert core.scene_far_observe()
    core.pocket["planning_pickup"] = "recognised"
    assert not core.scene_far_observe()
    core.pocket["planning_pickup"] = None
    core.near_active = lambda: True
    assert not core.scene_far_observe()


# --- the escape prefix on a real obstacle layer (plan 2.1 step 5) ----------------------------------------------------
def _real_layer():
    import importlib.util
    import json
    import sys
    from pathlib import Path

    from forklift_core.perception.obstacle_grid import AgeErrorTable
    from forklift_core.planning.geometry import Footprint

    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location("obstacle_layer", root / "sim/isaac/obstacle_layer.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["obstacle_layer"] = module
    spec.loader.exec_module(module)
    config = module.load_layer_config(root / "config/obstacle_layer.yaml")
    age = json.loads((root / "config/obstacle_odometry_age.json").read_text())
    table = AgeErrorTable(tuple(age["ages_s"]), tuple(age["cumulative_position_m"]), tuple(age["cumulative_yaw_rad"]))
    layer = module.ObstacleLayer(config, hall=Bounds(-6, 10, -5, 5), error_table=table,
                                 unloaded=Footprint(1.29, 0.17, 0.36), loaded=Footprint(1.53, 0.17, 0.40),
                                 body_front_m=0.884, rear_axle_x_in_base_m=-0.34, noise_seed=1)
    return layer, config


def test_an_escape_prefix_is_found_checked_installed_under_the_escape_ownership_and_cleared():
    core, ms, state, calls, rear, held = make()
    layer, config = _real_layer()
    pose = (2.0, 0.3, 0.05)
    n = len(layer.beam_angles)
    open_ = {s_.name: (np.full(n, 4.0), np.ones(n, bool), np.zeros(n, bool)) for s_ in config["sensors"]}
    for k in range(3):
        layer.add_scans(0.1 * k, open_, odom_rear=pose, loaded=True)
    path = np.array([pose, (2.5, 0.3, 0.05)])
    layer.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=path, loaded=True)
    core.obstacle = {"layer": layer, "applied": (0.0, 0.0, 0.0), "version": 0}
    ports = core._scene_ports(0.2, np.array(pose), 0.0, DT)
    chosen = ports.escape_plan((), None)
    assert chosen is not None and state["scene_escape_candidates"][-1]["permit"]["ok"]
    poses, direction, leg_dir = chosen
    config0 = core.trackers["transport"].config
    ports.escape_install(chosen)
    assert core.obstacle["backoff"]["scene"] and core.obstacle["backoff"]["phase"] == "transport"
    assert core.permission_acting()[1] is False  # never the delivery straight: the permission acts on it
    assert core.trackers["transport"].config.cruise_speed_mps == core.options.backoff_speed_mps
    assert ports.escape_state() == "running"
    ports.escape_clear()
    assert "backoff" not in core.obstacle
    # the normal path installed after it gets the leg's own settings back (Codex plan round 4 P3)
    result = plan_transport_leg(_scene(), Pose2D(*pose), core.options.planner_config, geometry=core.geometry)
    ports.install(result)
    assert core.trackers["transport"].config == config0



def test_the_correction_contribution_composes_the_transforms_without_reading_slam_rear():
    """Codex impl round 1 P2-1: slam_rear checks freshness and may refuse before the reply that makes it fresh is
    in; the contribution is the same odometry pose under the correction before and after."""
    core, ms, state, calls, rear, held = make()

    class Tracker:
        applied = ((0.0, 0.0, 0.0), 1.0)
        received = None
        mode = "tracking"

        def receive(self, scan_id, stamp_s, status, map_from_odom):
            self.applied = (tuple(map_from_odom), stamp_s)

    core.slam = {"tracker": Tracker(), "records": []}
    core.hooks.slam_odom_base = lambda: (2.0, 0.0, 0.0)
    core.hooks.slam_rear = lambda t: (_ for _ in ()).throw(RuntimeError("localization_stale"))
    core.hold_count = lambda: 0
    reply = SimpleNamespace(scan_id=1, stamp_s=1.2, status="ok", map_from_odom=(0.0, 0.064, 0.0), slam_wall_s=0.0)
    core.slam_replies(1.26, [(reply, {}, 0.0, 0.0)], loaded=False)
    assert core.scene_correction["net_max_m"] == pytest.approx(0.064)


def test_the_batch_leg_plan_does_not_pass_the_exit_prefix_twice():
    """Codex impl round 1 P2-6: the batch hook binds the exit prefix itself; only the worker's payload carries it."""
    core, ms, state, calls, rear, held = make()
    seen = {}
    core.hooks.exit_prefix = "PREFIX"

    def hook(*a, **k):
        seen.update(k)
        return plan_transport_leg(*a, **k)

    core.hooks.plan_transport_leg = hook
    core.scene_replan_raise(10.0, "cusp_pose", rear["pose"])
    _drive(core, rear, held)
    assert "exit_prefix" not in seen
    core2, *_rest = make(jobs=Jobs())
    core2.state["async_plans"] = []
    core2.commands.begin_wait = core2.commands.end_wait = lambda *a, **k: None
    core2.hooks.exit_prefix = "PREFIX"
    core2.scene_replan_raise(10.0, "cusp_pose", rear["pose"])
    core2.scene_replan_tick(10.0, rear["pose"], 0.0, DT)
    assert core2.jobs.submitted[-1][1]["kwargs"]["exit_prefix"] == "PREFIX"


def test_an_attempt_after_failed_candidates_records_the_candidate_its_waypoint_came_from():
    """Codex impl round 2 P3: failed later candidates advance next_candidate_index; the attempt names the waypoint's."""
    core, ms, state, calls, rear, held = make()
    _observe(core, ms, state, waypoints=[(-2.0, 1.0, 0.0), (-1.5, -1.5, 0.3), (-2.5, 2.0, 0.0)])
    state["observation_waypoint_selected"] = [-2.0, 1.0, 0.0]
    ms.next_candidate_index = 3  # candidates 1 and 2 tried and failed, the stall fell back to candidate 0
    assert core._selected_candidate_index() == 0
    state["observation_waypoint_selected"] = [-1.5, -1.5, 0.3]
    assert core._selected_candidate_index() == 1
    state["observation_waypoint_selected"] = [9.0, 9.0, 0.0]  # a waypoint not from the list: the old rule
    assert core._selected_candidate_index() == 2


def test_the_correction_yaw_contribution_wraps_at_pi():
    """Codex impl round 2 P3: pi - 0.01 -> -pi + 0.01 is a 0.02 rad correction, not -2 pi + 0.02."""
    core, ms, state, calls, rear, held = make()

    class Tracker:
        applied = ((0.0, 0.0, math.pi - 0.01), 1.0)
        received = None
        mode = "tracking"

        def receive(self, scan_id, stamp_s, status, map_from_odom):
            self.applied = (tuple(map_from_odom), stamp_s)

    core.slam = {"tracker": Tracker(), "records": []}
    core.hooks.slam_odom_base = lambda: (0.0, 0.0, 0.0)
    core.hold_count = lambda: 0
    seen = {}
    core.scene_correction_note = lambda t, net, reply, yaw=0.0: seen.update(yaw=yaw)
    reply = SimpleNamespace(scan_id=1, stamp_s=1.2, status="ok", map_from_odom=(0.0, 0.0, -math.pi + 0.01),
                            slam_wall_s=0.0)
    core.slam_replies(1.26, [(reply, {}, 0.0, 0.0)], loaded=False)
    assert seen["yaw"] == pytest.approx(0.02)


def test_a_correction_before_the_phase_has_a_tracker_starts_its_own_record():
    """ws1 sr_a22ec45: a reply at 0.4 s, before the phase's tracker exists, found the empty record under key None."""
    core, ms, state, calls, rear, held = make()
    ms.phase = "init"
    core.scene_correction_note(0.4, 0.01, 0.01)
    assert core.scene_correction["key"] is None and core.scene_correction["net_max_m"] == pytest.approx(0.01)
    core.scene_correction_note(0.5, 0.005, 0.02)
    assert core.scene_correction["net_max_m"] == pytest.approx(0.01)
    assert core.scene_correction["reply_max_m"] == pytest.approx(0.02)
