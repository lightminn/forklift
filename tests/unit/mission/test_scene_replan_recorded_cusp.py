"""A on a recorded case (docs/plans/2026-10-23-scene-replan.md 2.2): the baseline's transport cusp stall (CT-1 #107,
line_m seed 1, mission 3 -- 74.6 mm / -52 mrad at the end of the reverse loaded leg, ended `cusp_position_stall`)
replayed by injection on CPU, end to end through the real pieces: the recorded scene, path, tracker and planner
settings; the real tracker followed kinematically to that leg's end; the recorded stall pose injected as the control
pose; the real cusp latch refusing it on its pose bounds; the device planning the leg again from the stop in the
recorded scene, checking the candidate on a real obstacle layer fed with scans cast in that scene, steering the
wheels, handing the tick back; the follower then driving the new leg past the old stall to its next end.

The fixture tests/fixtures/scene_replan_line_m_s1_m3_cusp.json is cut from the run's records (its "source")."""

import importlib.util
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control import RearAxlePathTracker, TrackerConfig, ackermann_command
from forklift_core.control import cusp_stall as CUSP_STALL
from forklift_core.control.path_tracking import AckermannGeometry
from forklift_core.localization.scan_docking import ray_scan
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.perception.obstacle_grid import AgeErrorTable
from forklift_core.planning.geometry import Footprint
from forklift_core.planning.hybrid_astar import PlannerConfig
from forklift_core.planning.pallet_mission import SyntheticMissionGeometry, plan_return_leg, plan_transport_leg

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = ROOT / "tests/fixtures/scene_replan_line_m_s1_m3_cusp.json"
DT = 1 / 120
DRIVE = AckermannGeometry(wheelbase_m=0.66, track_m=0.53, wheel_radius_m=0.125, max_steering_rad=0.2618,
                          max_wheel_rate_rad_s=8.0)
REAR_X_IN_BASE_M = -0.34


def _sim_module(name):
    module = sys.modules.get(name)  # another test's module, kept: its classes must stay the pickled ones
    if module is None:
        spec = importlib.util.spec_from_file_location(name, ROOT / f"sim/isaac/{name}.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return module


def _fixture():
    f = json.loads(FIXTURE.read_text())
    scenario = _sim_module("scene_file").scenario_from_record(f["mission_scenario"])
    geometry = SyntheticMissionGeometry(unloaded_footprint=Footprint(**f["unloaded_footprint"]), **f["geometry"])
    assert (geometry.loaded_footprint.front_m, geometry.loaded_footprint.half_width_m) == pytest.approx(
        (f["loaded_footprint"]["front_m"], f["loaded_footprint"]["half_width_m"]))  # the run's geometry rebuilt
    path = f["transport_path"]
    return f, scenario, geometry, PlannerConfig(**f["planner_config"]), TrackerConfig(**f["tracker_config"]), (
        np.asarray(path["poses"], float), np.asarray(path["directions"], int), np.asarray(path["curvatures_inv_m"], float))


def _layer(scenario):
    """A real obstacle layer fed with scans cast in the recorded scene: every prop a box as tall as recorded (the
    1.05 m plane sees the 1.3 m slot markers, not the 0.144 m pallets), the hall bounds as walls."""
    module = _sim_module("obstacle_layer")
    config = module.load_layer_config(ROOT / "config/obstacle_layer.yaml")
    age = json.loads((ROOT / "config/obstacle_odometry_age.json").read_text())
    table = AgeErrorTable(tuple(age["ages_s"]), tuple(age["cumulative_position_m"]), tuple(age["cumulative_yaw_rad"]))
    layer = module.ObstacleLayer(config, hall=scenario.bounds, error_table=table,
                                 unloaded=Footprint(1.29, 0.17, 0.36), loaded=Footprint(1.56, 0.17, 0.40),
                                 body_front_m=0.884, rear_axle_x_in_base_m=REAR_X_IN_BASE_M, noise_seed=1)
    b = scenario.bounds
    walls = [((b.x_min_m, b.y_min_m), (b.x_max_m, b.y_min_m)), ((b.x_max_m, b.y_min_m), (b.x_max_m, b.y_max_m)),
             ((b.x_max_m, b.y_max_m), (b.x_min_m, b.y_max_m)), ((b.x_min_m, b.y_max_m), (b.x_min_m, b.y_min_m))]
    boxes = []
    for prop in scenario.props:
        r = prop.rectangle
        c, s = math.cos(r.yaw_rad), math.sin(r.yaw_rad)
        corners = [(r.x_m + c * dx - s * dy, r.y_m + s * dx + c * dy)
                   for dx, dy in ((r.length_m / 2, r.width_m / 2), (-r.length_m / 2, r.width_m / 2),
                                  (-r.length_m / 2, -r.width_m / 2), (r.length_m / 2, -r.width_m / 2))]
        boxes.append((prop.asset.height_m, [(corners[k], corners[(k + 1) % 4]) for k in range(4)]))

    def scan(t, rear, loaded=True):
        raw = {}
        for sensor in layer.sensors:
            segments = walls + [seg for height, sides in boxes if height > sensor.xyz_m[2] for seg in sides]
            dx = sensor.xyz_m[0] - REAR_X_IN_BASE_M
            c, s = math.cos(rear[2]), math.sin(rear[2])
            laser = (rear[0] + c * dx - s * sensor.xyz_m[1], rear[1] + s * dx + c * sensor.xyz_m[1],
                     rear[2] + sensor.yaw_rad)
            ranges = ray_scan(segments, laser, layer.beam_angles, float(config["range_max_m"]))
            hits = np.isfinite(ranges)
            raw[sensor.name] = (np.where(hits, ranges, np.nan), hits, np.zeros(len(ranges), bool))
        layer.add_scans(t, raw, odom_rear=tuple(rear), loaded=loaded)
    return layer, scan


def _core(f, scenario, geometry, planner, tracker_config, path):
    poses, directions, curvatures = path
    held = {"kappa": 0.0}
    calls = []

    def require(ok, reason):
        calls.append(("require", bool(ok), reason))
        if not ok:
            raise RuntimeError(reason)

    hooks = SimpleNamespace(
        require=require, grid_world=lambda sc: sc, path_record=lambda p: {"status": p.status, "n": len(p.poses)},
        write_paths=lambda: None, set_rear=lambda r: None, held_kappa=lambda: held["kappa"], loaded=lambda: True,
        steering_actual=lambda: ackermann_command(0.0, held["kappa"], DRIVE).steering_rad,
        truth_rear=lambda: None, dump_tracking=lambda *a: None, wall=lambda: 0.0,
        plan_transport_leg=lambda *a, **k: (calls.append(("plan_transport_leg",)), plan_transport_leg(*a, **k))[1],
        plan_return_leg=plan_return_leg, signed_speed=lambda: 0.0, cusp_standing=lambda: True)
    tracker = RearAxlePathTracker(poses, directions, curvatures, tracker_config)
    ms = MissionState(phase="transport", phase_started=0.0)
    state = {"paths": {"transport": {"status": "success"}}, "transitions": [], "cusp_stalls": []}
    options = SimpleNamespace(pickup_prior=True, scenario=scenario, planner_config=planner, travel_config=None,
                              return_to_pose=scenario.start_rear, drive_geometry=DRIVE, backoff_speed_mps=0.1,
                              dwells=None, safety_stops=None, cusp_heading_hold_phases=("observe", "return_home"),
                              dt=DT, grid_planning=False)
    core = MissionCore(ms, state=state, obstacle=None, slam=None, pocket={"check": None},
                       paths={"transport": SimpleNamespace(poses=poses, directions=directions,
                                                           curvatures_inv_m=curvatures, status="success")},
                       trackers={"transport": tracker}, geometry=geometry,
                       settings={"drive_acceleration_mps2": 0.3}, pallet_geometry=None, hooks=hooks, options=options)
    core.jobs = None
    monitor = CUSP_STALL.CuspStallMonitor(state["cusp_stalls"])
    hooks.cusp_monitor = lambda: monitor
    core.cusp_alive = lambda now: True
    core.cusp_excluded = lambda phase: None
    core.cusp_stopped = lambda: True
    core.cusp_ownership = lambda now, rear, command: (True, {"time_s": now})
    core.cusp_scan_id = lambda now: int(now * 10)  # a scan every 0.1 s
    core.cusp_end_recovered = lambda is_goal: False
    core.cusp_leg_checker = lambda: None
    core.cusp_sensors = lambda *a: (True, False, None)
    return core, ms, state, held, monitor, calls


def _step(pose, cmd):
    x, y, yaw = pose
    v, k = float(cmd.speed_mps), float(cmd.curvature_inv_m)
    return np.array([x + v * math.cos(yaw) * DT, y + v * math.sin(yaw) * DT, yaw + v * k * DT])


def test_the_recorded_transport_cusp_stall_is_replanned_from_the_stop_and_the_new_leg_passes_it():
    f, scenario, geometry, planner, tracker_config, path = _fixture()
    core, ms, state, held, monitor, calls = _core(f, scenario, geometry, planner, tracker_config, path)
    tracker = core.trackers["transport"]
    stall_pose = np.asarray(f["stall"]["rear"], float)
    # 1. The real tracker followed kinematically from the path's start to the recorded leg's end window.
    pose, t = path[0][0].copy(), 0.0
    for _ in range(int(120 / DT)):
        cmd = tracker.update(pose, 0.0, DT)
        assert cmd.status in ("tracking", "braking"), cmd
        if tracker.leg_index == f["stall"]["leg_index"] and tracker.leg_remaining_m() <= 0.05:
            break
        pose, t = _step(pose, cmd), t + DT
    else:
        pytest.fail("the recorded leg's end was not reached")
    # 2. Injected: the control pose jumps to the recorded stall pose (the SLAM correction of the run) and the truck
    #    stands there; the real latch takes the stall and refuses it on its pose bounds after the 5 s wait.
    raised = None
    for k in range(int(12 / DT)):
        now = t + k * DT
        raw = tracker.update(stall_pose, 0.0, DT)
        follow_, hold_ = core.cusp_tick(now, stall_pose, raw, raw, False)
        if core.scene_raised:
            raised = now
            break
    assert raised is not None, state["cusp_stalls"]
    latch_record = state["cusp_stalls"][-1]
    assert latch_record["disposition"] == "ended" and latch_record["position_error_m"] == pytest.approx(
        f["stall"]["position_error_m"], abs=0.01)
    request = state["scene_replans"][-1]
    assert request["trigger"] == "cusp_pose" and request["phase"] == "transport"
    assert request["deviation"]["position_m"] == pytest.approx(latch_record["position_error_m"])
    # 3. The device: the leg planned again from the stop in the recorded scene, the candidate checked on a real
    #    layer fed with scans cast in that scene, the wheels steered, the tick handed back.
    layer, scan = _layer(scenario)
    core.obstacle = {"layer": layer, "applied": (0.0, 0.0, 0.0), "version": 0}
    old_tracker = core.trackers["transport"]
    out = None
    for k in range(int(30 / DT)):
        now = raised + k * DT
        if k % 12 == 0:
            scan(now, stall_pose)
            layer.refresh(now, (0.0, 0.0, 0.0), 0, current_pose=tuple(stall_pose),
                          path_ahead=core.trackers["transport"].leg_ahead()[0], loaded=True)
        out = core.scene_replan_tick(now, stall_pose, 0.0, DT)
        if out.owned and out.output == "presteer":
            held["kappa"] = out.curvature
        if not out.owned:
            break
    record = state["scene_replans"][-1]
    assert out is not None and not out.owned, record
    assert record["outcome"] == "installed", record
    assert record["candidate"]["status"] == "tracking" and record["candidate"]["ok"], record["candidate"]
    assert record["preview"]["ok"] and record["presteer"] in ("aligned", "timeout")
    assert ("plan_transport_leg",) in calls
    new_tracker = core.trackers["transport"]
    assert new_tracker is not old_tracker
    # 4. The follower drives the new leg from the stop: past the old stall to the new leg's own end, never failed.
    pose, t = stall_pose.copy(), now
    first_leg = new_tracker.leg_index
    for _ in range(int(120 / DT)):
        cmd = new_tracker.update(pose, 0.0, DT)
        assert cmd.status != "failed", cmd
        if new_tracker.leg_index != first_leg or cmd.status == "arrived":
            break
        pose, t = _step(pose, cmd), t + DT
    else:
        pytest.fail("the new leg did not reach its end")
    assert np.hypot(*(pose[:2] - stall_pose[:2])) > 0.2  # it moved on from the stall
