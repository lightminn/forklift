"""The real truck's run assembly (docs/design/2026-11-20-real-hil-6a.md 10): one module for the HIL run (6a-2b) and,
with its sources swapped, the real-input run (PR 7).

``prepare_run`` loads and checks the configuration with the dry assembly's own loaders (below, shared with
``tools/real_autonomy.py --dry-assemble``), plans the reference once, makes the console channels, the session epoch
and the common time origin, and starts the plan worker on a thread that a stop flag cancels. ``assemble_run`` builds
the host's port, the permit, the arbiter on the prepared channels, the real obstacle layer, the delay bound, the
production evidence, the watches and the loop on the given ``RunSources`` -- or refuses with ``AssemblyRefused``.
``RealRun`` runs the loop at 50 Hz on the wall clock, closes it (DISARM only, its confirmation within 1 s) and keeps
the record (JSONL by a writer thread; a tick never waits on it). No device is opened here: the runner owns the pty,
the model and the console server."""

from __future__ import annotations

import hashlib
import importlib.util
import io
import itertools
import json
import math
import queue
import secrets
import sys
import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from forklift_core.live import real_assembly as asm
from forklift_core.live import real_autonomy as ra
from forklift_core.live.real_assembly import AssemblyRefused
from forklift_core.live.real_channels import RealChannels, RealLink
from forklift_core.live.real_loop import (BYPASSABLE, ENCODER_CUT, RealLoop, RealSession, motion_allowed)
from forklift_core.mcu_link.build_policy import Client
from forklift_core.mcu_link.messages import FaultFlag
from forklift_core.plant.mcu_host import HostConfig

REPO = Path(__file__).resolve().parents[3]

# the executor's host link (the rig's, bring-up's and the MCU link tool's values): STATE fresh for 50 ms by receipt
# (the HIL rule, design 10.5), DRIVE cap 150 ms
RUN_HOST_CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=16)
TICK_NS = 20_000_000
# the STATE delay a measurement interval may have and still place scans (design 10.10): its own value, not the host's
# freshness (which ages a STATE from its receipt, not its measurement)
RUN_STATE_DELAY_S = 0.05
HALL_MARGIN_M = 1.0
CLOSING_CONFIRM_S = 1.0  # the closing DISARM's confirmation (design 10.3)
# each source's work per call (design 10.5): scans and SLAM replies beyond these are dropped oldest first (counted)
SCANS_PER_CALL = 4
SLAM_REPLIES_PER_CALL = 8
RECORD_QUEUE = 8192
PLAN_READY_TIMEOUT_S = 30.0
EXIT_OK, EXIT_REFUSED, EXIT_RECORD = 0, 2, 3


class SourceFatal(RuntimeError):
    """A source adapter's unrecoverable error (design 10.1): any source's ends the run; an IMU's other errors are a
    missing batch the IMU watch judges (5b)."""


class RunCancelled(RuntimeError):
    """The stop flag came while preparing."""


# --- the loaders the dry assembly uses too (design 10: one set) ------------------------------------------------------
def _sha(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _insertion_geometry():
    name = "insertion_geometry"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, REPO / "sim/isaac/insertion_geometry.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _urdf_source(urdf):
    """A path as given, or the URDF's bytes read once (the operating run: hashed and parsed from the same bytes, Codex
    U2b round 1 P2-2) as a fresh stream for each parse."""
    return io.BytesIO(urdf) if isinstance(urdf, (bytes, bytearray)) else urdf


def mission_geometry(urdf, depth_m: float, width_m: float, reserve_m: float = 0.016):
    from forklift_core.planning.geometry import Footprint
    from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

    ig = _insertion_geometry()
    axle_to_tip, _ = ig.read_chassis_reference_m(_urdf_source(urdf))
    return SyntheticMissionGeometry(unloaded_footprint=Footprint(axle_to_tip, 0.17, 0.36), pallet_depth_m=depth_m,
                                    pallet_width_m=width_m, axle_to_fork_tip_m=axle_to_tip,
                                    carriage_limit_m=ig.read_carriage_limit_m(_urdf_source(urdf)),
                                    insertion_reserve_m=reserve_m)


def chassis_reference(urdf) -> tuple:
    """(axle to fork tip, rear axle offset behind base_link, carriage limit) in metres from the chassis model (a path,
    or its bytes)."""
    ig = _insertion_geometry()
    axle_to_tip, rear_x = ig.read_chassis_reference_m(_urdf_source(urdf))
    return float(axle_to_tip), abs(float(rear_x)), float(ig.read_carriage_limit_m(_urdf_source(urdf)))


def map_occupancy(grid):
    """A surveyed map's grid (rows = y) as the planner's occupancy (cells x by y); a rotated origin is refused."""
    from forklift_core.planning.grid_collision import OccupancyGrid

    if abs(float(grid.origin[2])) > 1e-9:
        raise ValueError("map_origin_rotated")
    return OccupancyGrid(float(grid.origin[0]), float(grid.origin[1]), float(grid.resolution_m),
                         np.asarray(grid.occupied()).T)


def read_settings(path) -> dict:
    import yaml

    try:
        data = yaml.safe_load(Path(path).read_text())
        for key in ("lift_target_m", "lift_rate_mps", "drive_acceleration_mps2", "withdraw_speed_mps", "speeds",
                    "mission_stages"):
            if key not in data:
                raise KeyError(f"setting_missing:{key}")
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as err:
        raise AssemblyRefused([f"settings_unreadable:{err}"]) from err
    return data


def read_vehicle(path):
    from forklift_core.plant.kinematics import RealKinematics
    from forklift_core.plant.real_vehicle import load_real_vehicle

    try:
        vehicle = load_real_vehicle(path)
        return vehicle, RealKinematics(vehicle)
    except Exception as err:  # noqa: BLE001 -- any malformed vehicle file is a structural refusal
        raise AssemblyRefused([f"vehicle_unreadable:{type(err).__name__}:{err}"]) from err


def read_map(bundle, cfg) -> tuple:
    """(map id, occupancy) of a surveyed bundle verified by its manifest against the S10-A map id."""
    from forklift_core.localization.slam_map import MapBundleError, load_bundle

    try:
        b = load_bundle(bundle, expected_map_id=cfg.map_id)
        return b.map_id, map_occupancy(b.grid)
    except (MapBundleError, ValueError) as err:
        raise AssemblyRefused([f"map:{err}"]) from err


def planning_settings(data: dict, kin) -> tuple:
    """(settings, planner config, curvature cap): every cap is the steering's own range."""
    from forklift_core.planning import pallet_mission as pm

    low, high = kin.cfg.steering.curvature_range
    cap = min(-float(low), float(high))
    settings = {k: data[k] for k in ("lift_target_m", "lift_rate_mps", "drive_acceleration_mps2",
                                     "withdraw_speed_mps")}
    settings.update(steering_command_rate_rad_s=kin.steering_rate_rad_s, tracker_curvature_inv_m=cap)
    return settings, pm.make_transport_planner_config(curvature_limit_inv_m=cap), cap


def core_options(settings: dict, rear_offset_m: float) -> dict:
    """The core's options beyond ``real_options`` that the control tick reads (the dry and the run assembly alike).
    S10-A has no cusp, observation or dwell (those stay at their off values); the command ramp and the mid-path
    stop's deceleration are autonomy.yaml's ``drive_acceleration_mps2`` (provisional, as that file); the observation
    arrival bounds are the Isaac runner's (unused while observation is off); the rear-axle offset is the chassis
    model's."""
    accel = float(settings["drive_acceleration_mps2"])
    return {"cusp_heading_hold_rad": 0.05, "cusp_heading_hold_phases": ("return_home",), "tracker_profile": None,
            "command_accel_mps2": accel, "mid_still_decel_mps2": accel, "max_cusp_replans": 0,
            "cusp_stop_needed": False, "cusp_stop_yaw_rate_radps": 0.05, "cusp_heading_hold_position_m": 0.03,
            "observe_replans_per_leg": 0, "drive_geometry": None, "rear_axle_offset_m": float(rear_offset_m),
            "initial_time": 0.0, "dwell_d_end_m": 0.0, "observe_arrival_m": 0.10, "observe_arrival_yaw_rad": 0.10}


def check_plan(cfg, planner, geometry, occupancy):
    """The reference plan to ``check_pallet_pose``; the pallet itself is the pickup obstacle, never occupancy (10.6)."""
    from forklift_core.planning import Pose2D
    from forklift_core.planning import pallet_mission as pm
    from forklift_core.planning.geometry import Rectangle

    check = pm.PalletSite(*cfg.check_pallet_pose)
    return pm.plan_transport(replace(asm.scenario_of(cfg), props=()), planner, geometry=geometry,
                             target_pickup=check, start_rear=Pose2D(*cfg.start_rear),
                             return_to=Pose2D(*cfg.wait_pose), occupancy=occupancy,
                             pickup_obstacle=Rectangle(check.x_m, check.y_m, geometry.pallet_depth_m,
                                                       geometry.pallet_width_m, check.yaw_rad))


# --- the HIL scene (design 10.4-10.6) --------------------------------------------------------------------------------
def read_hil_scene(path) -> list:
    """The fixed obstacles of an S10-A file's ``hil:`` part: (x, y, half length, half width, yaw) in the map frame;
    a file without it is no HIL scene (refused for a HIL run)."""
    import yaml

    d = yaml.safe_load(Path(path).read_text()) or {}
    hil = d.get("hil")
    if not isinstance(hil, dict):
        raise AssemblyRefused(["hil_scene_missing"])
    out = []
    for box in hil.get("obstacles") or ():
        v = [float(x) for x in box]
        if len(v) == 4:
            v.append(0.0)
        if len(v) != 5 or not all(math.isfinite(x) for x in v) or v[2] <= 0 or v[3] <= 0:
            raise AssemblyRefused([f"hil_obstacle_invalid:{box}"])
        out.append(tuple(v))
    return out


def _box_corners(box) -> list:
    x, y, hl, hw, yaw = box
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * u - s * v, y + s * u + c * v) for u, v in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def scene_occupancy(cfg, obstacles, resolution_m: float = 0.1):
    """The HIL map's occupancy over the S10-A bounds: the fixed obstacles only (the target pallet is the planner's
    pickup obstacle -- in the occupancy it would block the insertion; 10.6)."""
    from forklift_core.planning.grid_collision import OccupancyGrid

    x0, x1, y0, y1 = (float(v) for v in cfg.bounds)
    nx, ny = int(math.ceil((x1 - x0) / resolution_m)), int(math.ceil((y1 - y0) / resolution_m))
    xs = x0 + (np.arange(nx) + 0.5) * resolution_m
    ys = y0 + (np.arange(ny) + 0.5) * resolution_m
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    occ = np.zeros((nx, ny), bool)
    pad = resolution_m * math.sqrt(2) / 2  # a cell the box touches is occupied
    for x, y, hl, hw, yaw in obstacles:
        c, s = math.cos(yaw), math.sin(yaw)
        u = (gx - x) * c + (gy - y) * s
        v = -(gx - x) * s + (gy - y) * c
        occ |= (np.abs(u) <= hl + pad) & (np.abs(v) <= hw + pad)
    return OccupancyGrid(x0, y0, resolution_m, occ)


def pallet_box(cfg) -> tuple:
    x, y, yaw = (float(v) for v in cfg.check_pallet_pose)
    return (x, y, float(cfg.pallet_depth_m) / 2, float(cfg.pallet_width_m) / 2, yaw)


def run_welcome(epoch: int) -> dict:
    return {"t": "welcome", "api": "console/1", "vehicle_id": "real-forklift", "session_epoch": int(epoch),
            "backend": "real", "capabilities": {
                "control": True, "mission": True, "teleop_direct": False, "teleop_video": False, "lift": False,
                "recovery_actions": [], "sim_reset": False,
                "feedback": {"rear_axle_speed": True, "steering": True, "lift_height": True, "load_state": False},
                "media": {}, "provisional_limits": True}}


# --- preparation (design 10.1, 10.5-10.7) ----------------------------------------------------------------------------
@dataclass
class RunPaths:
    s10a: Path
    settings: Path = REPO / "config/real/autonomy.yaml"
    vehicle: Path = REPO / "config/real/vehicle.yaml"
    watch: Path = REPO / "config/real/safety_watch.yaml"
    layer: Path = REPO / "config/real/obstacle_layer_real.yaml"
    budget: Path = REPO / "config/real/obstacle_layer.yaml"
    map_bundle: Path | None = None
    hil: bool = True
    # PR 7 7r (replay_assembly): an observe record replayed behind the HIL run, the LiDAR config the layer and SLAM
    # take, the IMU source config, and whether a synthetic record may be replayed (its output then not for judgement)
    replay: Path | None = None
    lidar_config: Path = REPO / "config/real/lidar.yaml"
    imu_config: Path = REPO / "config/real/imu.yaml"
    allow_synthetic: bool = False
    # the operating unlock (docs/plans/2026-11-26-real-operating-assembly.md 2.4): S10's human-confirmed record
    preconditions: Path = REPO / "config/real/s10_preconditions.yaml"
    # the operating run's site bundle (plan 2.8, unlock U2b): the site record it is found by, the registry and the
    # launcher's SLAM_BUNDLE_ROOT (the bundle is the row's folder under it; none is given by hand)
    site_record: Path | None = None
    registry: Path = REPO / "config/slam_map_bundles.yaml"
    bundle_root: Path | None = None


@dataclass
class RunPrep:
    paths: RunPaths
    cfg: ra.S10AConfig
    data: dict
    vehicle: object
    kin: object
    settings: dict
    planner: object
    cap: float
    geometry: object
    rear_offset_m: float
    body_front_m: float
    map_id: str
    occupancy: object
    obstacles: list  # the HIL scene's fixed obstacles (empty on a surveyed map)
    watches: object
    layer_cfg: dict
    budget_s: float
    overdue_end_s: float
    channels: RealChannels
    epoch: int
    welcome: dict
    start_ns: int
    hashes: dict
    check_plan: str
    kernels: dict = field(default_factory=dict)
    jobs: object = None
    replay: object = None  # replay_assembly.ReplayPlan (7r), None for every other run
    site: object = None  # real_site.SiteBundle of the operating run (U2b), None for every other run
    lidar: object = None  # the operating run's LidarConfig, loaded once with the layer file (U2a; None on HIL/replay)
    # U3c (plan 2.6): the pallet confirmation's lift target, test pallet and mass, from the record the assembly judged
    operator_load: dict | None = None
    # U3e-1 (plan 6.1): the operating preconditions' list as the assembly judged it (None: not judged -- HIL, replay)
    judgement: list | None = None


class _Handover:
    """The plan worker's start on its own thread, handed over atomically (10.7): ``pending -> done | cancelled`` under
    one lock; the side that comes second closes the worker."""

    def __init__(self, factory) -> None:
        self._lock = threading.Lock()
        self.state, self.worker, self.error = "pending", None, None
        self.done = threading.Event()
        self._thread = threading.Thread(target=self._run, args=(factory,), name="plan-worker-start", daemon=True)
        self._thread.start()

    def _run(self, factory) -> None:
        try:
            worker = factory()
        except Exception as err:  # noqa: BLE001 -- told to the waiting side
            with self._lock:
                self.error = err
            self.done.set()
            return
        with self._lock:
            if self.state == "pending":
                self.state, self.worker = "done", worker
                worker = None
        if worker is not None:
            worker.close()  # cancelled before it was ready: this side came second
        self.done.set()

    def cancel(self) -> None:
        with self._lock:
            if self.state == "pending":
                self.state = "cancelled"
                return
            worker, self.worker, self.state = self.worker, None, "cancelled"
        if worker is not None:
            worker.close()


def _sha_or_none(path) -> str | None:
    try:
        return _sha(path)
    except OSError:
        return None


def _hashed_files(paths) -> tuple:
    return (("s10a", paths.s10a), ("settings", paths.settings), ("vehicle", paths.vehicle), ("watch", paths.watch),
            ("layer", paths.layer), ("budget", paths.budget), ("lidar", paths.lidar_config),
            ("imu", paths.imu_config))


def prepare_run(paths: RunPaths, *, clock=time.monotonic_ns, stop: threading.Event | None = None,
                jobs_factory=None, epoch: int | None = None) -> RunPrep:
    """Everything before the model starts (10.3 ②). ``AssemblyRefused`` with the structural reasons, ``RunCancelled``
    once ``stop`` is set (the plan worker closed by whichever side knows last)."""
    import yaml

    from forklift_core.plant.real_vehicle import chassis_model_path
    from forklift_core.sensors.lidar_cfg import lidar_config_from

    stop = stop or threading.Event()
    real_lidar = None
    # the files' hashes before anything is loaded; compared again at the end (a file changed while it was loaded is
    # refused: the run and the operating preconditions judge the same bytes -- Codex unlock U1 review P1-2)
    hashes_at_start = {name: _sha_or_none(p) for name, p in _hashed_files(paths)}
    try:
        _params = chassis_model_path(paths.vehicle)
        hashes_at_start.update(chassis_params=_sha_or_none(_params),
                               chassis_urdf=_sha_or_none(_params.parent / "forklift.urdf"))
    except (OSError, ValueError, TypeError, yaml.YAMLError):
        pass  # the vehicle loader below refuses it with its own reason (vehicle_unreadable)
    try:
        cfg = ra.load_s10a(paths.s10a)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as err:
        raise AssemblyRefused([f"s10a_unreadable:{err}"]) from err
    data = read_settings(paths.settings)
    vehicle, kin = read_vehicle(paths.vehicle)
    watches = asm.build_watches(data, paths.watch, vehicle.wheel_radius_m)
    settings, planner, cap = planning_settings(data, kin)
    obstacles: list = []
    replay = None
    site = None
    if paths.replay is None and not paths.hil:
        # the operating run (plan 2.8, unlock U2b): the bundle is the registry row's, found before the map is loaded
        from forklift_core.live import real_site as rs_

        if paths.map_bundle is not None:
            raise AssemblyRefused([f"{rs_.PREFIX}:map_bundle_by_hand"])
        try:
            site = rs_.resolve_site(site_record=paths.site_record, registry=paths.registry,
                                    bundle_root=paths.bundle_root, encoder_model=rs_.REAL_ENCODER_MODEL)
        except rs_.SiteRefused as err:
            raise AssemblyRefused([err.reason]) from err
    if paths.replay is not None:
        # 7r plan 4: the record and its bundle stand for the HIL scene (no hil: part, no zero start); the configs and
        # the record are checked before anything starts
        from forklift_core.live import replay_assembly as rpa

        if not paths.hil:
            raise AssemblyRefused(["replay_needs_hil"])
        replay = rpa.replay_inputs(paths, watches)
        map_id, occupancy = read_map(rpa.replay_bundle(replay, paths), cfg)
    elif paths.hil:
        obstacles = read_hil_scene(paths.s10a)  # the synthetic LiDAR casts them whatever map plans (round 1 P2)
    if replay is not None:
        pass  # the map read above, from the record's bundle (or --map-bundle)
    elif site is not None:
        try:
            map_id, grid = rs_.read_site_map(site, cfg.map_id)  # every file hashed and parsed from one read
        except rs_.SiteRefused as err:
            raise AssemblyRefused([err.reason]) from err
        try:
            occupancy = map_occupancy(grid)
        except ValueError as err:  # e.g. a rotated origin: a structured refusal, never an escape (round 2 P3)
            raise AssemblyRefused([f"{rs_.PREFIX}:bundle_grid:{err}"]) from err
    elif paths.map_bundle is not None:
        map_id, occupancy = read_map(paths.map_bundle, cfg)
    elif paths.hil:
        if not cfg.map_id:
            raise AssemblyRefused(["map_id_unset"])
        map_id = cfg.map_id
        occupancy = scene_occupancy(cfg, obstacles)
    else:
        raise AssemblyRefused(["map_bundle_required"])
    refused = ra.s10a_refusal(cfg, map_id)
    if cfg.check_pallet_pose is None:
        refused.append("check_pallet_pose_unset")
    if paths.hil and replay is None and any(abs(float(v)) > 1e-12 for v in cfg.start_rear):
        refused.append("hil_start_not_zero")  # the odometry origin is the synthetic SLAM's identity (10.5)
    if refused:
        raise AssemblyRefused(refused)
    urdf = chassis_model_path(paths.vehicle).parent / "forklift.urdf"
    urdf_src = urdf
    if site is not None:
        # the operating run: the chassis geometry parsed from the very bytes whose hash C6 compares (round 1 P2-2)
        try:
            urdf_src = Path(urdf).read_bytes()
        except OSError as err:
            raise AssemblyRefused([f"config_unreadable:{type(err).__name__}:{err}"]) from err
        if hashlib.sha256(urdf_src).hexdigest() != hashes_at_start.get("chassis_urdf"):
            raise AssemblyRefused(["files_changed_while_preparing:chassis_urdf"])
    try:
        geometry = mission_geometry(urdf_src, cfg.pallet_depth_m, cfg.pallet_width_m)
        axle_to_tip, rear_offset, carriage = chassis_reference(urdf_src)
        def bound_yaml(path, name):
            # the operating run: parsed from the very bytes whose hash the assembly's judgement binds -- a file
            # replaced and put back while preparing cannot leave its middle version here (Codex U3e-2 P2-1)
            data = Path(path).read_bytes()
            if site is not None and hashlib.sha256(data).hexdigest() != hashes_at_start.get(name):
                raise AssemblyRefused([f"files_changed_while_preparing:{name}"])
            return yaml.safe_load(data.decode("utf-8"))

        layer_cfg = bound_yaml(paths.layer, "layer")
        if replay is not None:
            layer_cfg = rpa.replay_layer_cfg(layer_cfg, replay.lidar_cfg)  # one LiDAR source (7r plan 4)
        elif not paths.hil:
            # the operating run: one LiDAR calibration object for the layer and the scan source (U2a, Codex P2-5)
            # parsed from the very bytes hashed: those must be the bytes hashed at the start (Codex U2a round 2 P2-2)
            lidar_bytes = Path(paths.lidar_config).read_bytes()
            if hashlib.sha256(lidar_bytes).hexdigest() != hashes_at_start.get("lidar"):
                raise AssemblyRefused(["files_changed_while_preparing:lidar"])
            real_lidar = lidar_config_from(yaml.safe_load(lidar_bytes.decode("utf-8")))
            layer_cfg = real_layer_cfg(layer_cfg, real_lidar)
        budget = bound_yaml(paths.budget, "budget")
        budget_s = float(budget["latency_budget"]["seconds"])
        overdue_end_s = float(budget["overdue_end"]["seconds"])
        if site is not None and not (math.isfinite(budget_s) and budget_s > 0.0
                                     and math.isfinite(overdue_end_s) and overdue_end_s > 0.0):
            # a .nan or non-positive limit would never end a sustained overdue (Codex U3e-2 P2-3)
            raise AssemblyRefused(["precondition:C13:budget:values_invalid"])
    except AssemblyRefused:
        raise
    except Exception as err:  # noqa: BLE001 -- a malformed model or layer file is structural
        raise AssemblyRefused([f"config_unreadable:{type(err).__name__}:{err}"]) from err
    if replay is not None:
        cfg = rpa.replay_start(replay, cfg, map_id, rear_offset)  # the record's start for the loop, plan and hello
    if site is not None:
        # C6 (plan 2.8): the report against the files this run reads, before anything starts
        from forklift_core.live import real_site as rs_
        from forklift_core.live.real_loop import START_TOL_M, START_TOL_RAD

        refused = rs_.check_site(site, encoder_model=rs_.REAL_ENCODER_MODEL, lidar_sha256=hashes_at_start.get("lidar"),
                                 chassis_sha256=hashes_at_start.get("chassis_urdf"),
                                 calibration_id=getattr(vehicle, "calibration_id", None), map_id=map_id,
                                 start_rear=cfg.start_rear, rear_offset_m=rear_offset, tol_m=START_TOL_M,
                                 tol_rad=START_TOL_RAD)
        if refused:
            raise AssemblyRefused(refused)
    # a wall-clock run (real-time plan D2-D3): the fastest obstacle and permission kernels, compiled before the loop
    from forklift_core import kernels

    kernel_mode = kernels.prefer_fast()
    kernel_warm_s = kernels.warm_up()
    plan = check_plan(cfg, planner, geometry, occupancy)
    if not plan.success:
        raise AssemblyRefused([f"check_plan:{plan.status}"])
    hashes = {name: _sha_or_none(p) for name, p in _hashed_files(paths)}
    odo = layer_cfg.get("odometry_age") if isinstance(layer_cfg, dict) else None
    hashes["odometry_age"] = _sha_or_none(REPO / odo) if odo else None
    hashes["chassis_params"] = _sha_or_none(chassis_model_path(paths.vehicle))
    hashes["chassis_urdf"] = _sha_or_none(urdf)
    moved = [name for name, sha in hashes.items() if name in hashes_at_start and hashes_at_start.get(name) != sha]
    if moved:
        raise AssemblyRefused([f"files_changed_while_preparing:{name}" for name in moved])
    if site is not None:
        hashes.update(site.hashes)  # the site record, the registry and the report as judged (U2b)
    if replay is not None:
        hashes.update(replay_record=replay.record.sha256,
                      **{f"{k}_config": v["sha256"] for k, v in replay.configs.items() if v is not None})
    epoch = secrets.randbelow(0x7FFFFFFE) + 1 if epoch is None else int(epoch)
    prep = RunPrep(paths=paths, cfg=cfg, data=data, vehicle=vehicle, kin=kin, settings=settings, planner=planner,
                   cap=cap, geometry=geometry, rear_offset_m=rear_offset, body_front_m=axle_to_tip - carriage,
                   map_id=map_id, occupancy=occupancy, obstacles=obstacles, watches=watches, layer_cfg=layer_cfg,
                   budget_s=budget_s, overdue_end_s=overdue_end_s, channels=RealChannels(), epoch=epoch,
                   welcome=run_welcome(epoch), start_ns=int(clock()), hashes=hashes, check_plan=plan.status,
                   kernels={"mode": kernel_mode, "warm_up_s": round(float(kernel_warm_s), 3)}, replay=replay,
                   lidar=real_lidar, site=site)
    if jobs_factory is None:
        from forklift_core.planning.plan_worker import PlanWorker

        def jobs_factory():
            return PlanWorker(ready_timeout_s=PLAN_READY_TIMEOUT_S)

    hand = _Handover(jobs_factory)
    while not hand.done.wait(0.05):
        if stop.is_set():
            hand.cancel()
            raise RunCancelled("preparing")
    if stop.is_set():
        hand.cancel()
        raise RunCancelled("preparing")
    if hand.error is not None:
        raise AssemblyRefused([f"plan_worker:{hand.error}"])
    prep.jobs = hand.worker
    return prep


# --- the sources (design 10.1) ---------------------------------------------------------------------------------------
@dataclass
class RunSources:
    """The run's sources, contract only: ``client`` (HIL here; REAL refused until PR 7's preconditions), the host made
    with that client and the transport it has read the first STATE through, ``scans`` (``scans(now_ns)``,
    ``scan_source_events(now_ns)``, ``sensor``), ``slam`` (the 5c-2 epoch contract + ``map_id``), ``imu``
    (``samples(upto_ns, limit)``) and ``depth`` (``frames(now_ns)`` with a pocket check builder -- None: D5 only by the
    ``d5_pocket`` bypass)."""

    client: Client
    host: object
    transport: object
    scans: object
    slam: object
    imu: object = None
    depth: object = None
    counts: dict = field(default_factory=lambda: {"scans_dropped": 0, "slam_dropped": 0})
    on_assembled: list = field(default_factory=list)  # callbacks given the loop once it exists (late-bound poses)
    # PR 7 design 6.1, 1 (7c-1b-2b): the SLAM path's branch -- called with every scan the feed handed the layer, after
    # the batch cut (the scan the layer takes is the scan SLAM takes); None: the source takes its scans itself
    slam_feed: object = None
    closers: list = field(default_factory=list)  # called once when the run's workers close (sockets the sources hold)
    # the operating run (unlock U2a): whether the IMU qualified (C7, plan 2.7 -- None: not judged, a HIL run)
    imu_qualified: bool | None = None
    replay: object = None  # replay_assembly.ReplayControl (7r): the run's EOF, drain and read-failure step


def make_hil_sources(prep: RunPrep, host, transport, *, imu_seed: int = 0, slam_socket: str | None = None) -> RunSources:
    """The HIL run's synthetic sources (design 10.2) on a transport whose HIL check holds (else ``SyntheticRefused``):
    the LiDAR casts the scene's obstacles and the target pallet from the loop's odometry pose with the layer's mount,
    once the session check has started; the SLAM keeps the epoch contract on those acquisitions; a still IMU.

    ``slam_socket`` (PR 7 7c-1b-2b): the real SLAM source on a real-mode bridge listening at that Unix socket instead
    of the synthetic SLAM -- fed the scans the layer takes (``slam_feed``), the session's start at the registered
    start pose, the loop's odometry history, continuity generation and every history entry (``odom_listeners``)."""
    from forklift_core.live import synthetic_sources as ss
    from forklift_core.localization.start_reloc import base_from_rear

    box: dict = {}
    boxes = list(prep.obstacles) + [pallet_box(prep.cfg)]
    scan = ss.SyntheticScan(transport, Client.HIL, prep.start_ns, odom_pose=lambda: box["loop"].odom_pose,
                            boxes=boxes, mount=lidar_mount(prep),
                            active=lambda: "loop" in box and bool(box["loop"].permit.session_started))
    # the IMU only where the watches take one (imu: none builds no IMU watch, and the loop refuses a source without it)
    imu = ss.SyntheticImu(transport, Client.HIL, seed=imu_seed) if prep.watches.imu_policy == "required" else None
    if slam_socket is None:
        slam = ss.SyntheticSlam(transport, Client.HIL, scan, map_id=prep.map_id,
                                odom_base=lambda: base_from_rear(box["loop"].odom_pose, prep.rear_offset_m))
        sources = RunSources(client=Client.HIL, host=host, transport=transport, scans=scan, slam=slam, imu=imu)
    else:
        sources = _real_slam_sources(prep, host, transport, scan, imu, box, slam_socket)
    sources.on_assembled.append(lambda loop: box.__setitem__("loop", loop))
    return sources


def make_run_sources(prep: RunPrep, host, transport, *, slam_socket: str | None = None) -> RunSources:
    """The HIL run's sources: the replay's (7r) when the run replays a record, else the synthetic ones."""
    if prep.replay is not None:
        from forklift_core.live.replay_assembly import make_replay_sources

        if slam_socket is None:
            raise AssemblyRefused(["replay_needs_slam_bridge"])
        return make_replay_sources(prep, host, transport, slam_socket=slam_socket)
    return make_hil_sources(prep, host, transport, slam_socket=slam_socket)


def make_real_sources(prep: RunPrep, host, transport, *, scan_sock, slam_socket: str, imu_device=None,
                      clock=time.monotonic_ns, connect_timeout_s: float = 5.0) -> RunSources:
    """The operating REAL run's sources (docs/plans/2026-11-26-real-operating-assembly.md 2.2, unlock U2a). This call
    owns ``scan_sock`` (the scan container's socket, bound by the caller): it is closed on any failure here and by the
    run's closers afterwards.

    - the real scan source on ``scan_sock`` with the run's one ``LidarConfig`` (``prep.lidar``, the one the layer was
      configured from);
    - the real SLAM source in operating mode on the bridge at ``slam_socket``; its scans reach it only through
      ``slam_feed`` -- the run's branch after the layer accepted the scan (the scan the layer takes is the scan SLAM
      takes, with its acquisition interval);
    - with ``imu_device``, the D435i motion module on an operating IMU source, qualified (C7, plan 2.7) only when its
      config is measured, the watch's level reference is measured, and the profile opened says a rate that fits the
      watch. An unqualified IMU leaves ``imu`` None and ``imu_qualified`` False (the runtime readiness judges it, U3);
      a device started and found unfit is stopped again (a failed stop stays on the closers to be tried again).

    No depth source (7e-2b on hold)."""
    from forklift_core.live.real_sources import RealScanSource, start_real_imu

    closers: list = [lambda: _close_quietly(scan_sock)]
    try:
        if prep.paths.hil or prep.replay is not None:
            raise AssemblyRefused(["real_sources_on_hil"])
        if prep.lidar is None:
            raise AssemblyRefused(["real_sources_without_lidar_config"])
        box: dict = {}
        sources = _real_slam_operating(prep, host, transport, box, slam_socket, connect_timeout_s)
        closers += sources.closers
        scan = RealScanSource(scan_sock, prep.lidar, prep.start_ns, slam=sources.slam, clock=clock)
        imu, qualified = None, False
        if imu_device is not None:
            qualified, imu = _real_imu(prep, imu_device, closers, start_real_imu)
    except BaseException:
        _run_closers(closers)
        raise
    sources.scans, sources.imu, sources.imu_qualified = scan, imu, qualified
    sources.slam_feed = scan.offer_for
    sources.closers = closers
    sources.on_assembled.append(lambda loop: box.__setitem__("loop", loop))
    return sources


def _close_quietly(sock) -> None:
    try:
        sock.close()
    except (OSError, AttributeError):
        pass


def _run_closers(closers) -> None:
    for fn in reversed(closers):
        try:
            fn()
        except Exception:  # noqa: BLE001 -- every part is closed whatever one does
            pass


def _watch_imu_on_hashed_bytes(path, want_sha, watch_cfg):
    """The IMU watch's gap limit when C7's watch side holds on the bytes prep hashed (``want_sha``), else None: the
    level reference (``safety_watch.yaml`` ``imu`` provenance) measured, and the running watch's config (``watch_cfg``,
    built by ``build_watches`` from its own read) equal to the one these bytes give -- the watch that runs is the
    watch that was judged (Codex U2a round 3 P2)."""
    import yaml

    from forklift_core.live.real_preconditions import _provenances  # noqa: PLC2701 -- the one walker
    from forklift_core.safety.real_watch import imu_config_from

    try:
        raw = Path(path).read_bytes()
        if want_sha is None or hashlib.sha256(raw).hexdigest() != want_sha:
            return None
        data = yaml.safe_load(raw.decode("utf-8"))
        judged = imu_config_from(data)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
        return None
    level = [prov for key, prov in _provenances(data if isinstance(data, dict) else {}) if key == "imu"]
    if not (level and all(isinstance(prov, dict) and prov.get("measured") is True for prov in level)):
        return None
    if watch_cfg is None or judged != watch_cfg:
        return None
    return float(judged.gap_max_s)


def _real_imu(prep, device, closers, start_real_imu) -> tuple:
    """(qualified, source or None) -- module text of ``make_real_sources``."""
    import yaml

    from forklift_core.sensors.imu_source import imu_source_config_from

    hashes = getattr(prep, "hashes", None) or {}
    want = hashes.get("imu")
    try:
        raw = Path(prep.paths.imu_config).read_bytes()  # read once: hashed and parsed from the same bytes
    except OSError:
        return False, None
    if want is None or hashlib.sha256(raw).hexdigest() != want:
        return False, None  # the IMU config judged only on the bytes prep hashed (Codex U2a round 2 P2-2)
    cfg = imu_source_config_from(yaml.safe_load(raw.decode("utf-8")))
    if not cfg.measured or cfg.delay_bound_s is None or prep.watches.imu is None:
        return False, None
    gap = _watch_imu_on_hashed_bytes(prep.paths.watch, hashes.get("watch"), prep.watches.imu.cfg)
    if gap is None:
        return False, None
    stopped = {"done": False}

    def stop():
        if not stopped["done"]:
            device.stop()  # raises on a failed stop: the closer stays and a later close tries again
            stopped["done"] = True

    closers.append(stop)
    source, fits, _profile = start_real_imu(cfg, device, gap)
    if not fits:
        try:
            stop()
        except Exception:  # noqa: BLE001 -- left on the closers for the next try
            pass
        return False, None
    return True, source


def bridge_laser_xyz_yaw(lidar, rear_offset_m: float) -> tuple:
    """base_link <- laser (x, y, z, yaw) from the run's ``LidarConfig`` mount (rear-axle frame) and the chassis's rear
    axle offset: the value the operating hello carries and the SLAM bridge's ``laser_xyz_yaw`` is launched with
    (plan 2.3, unlock U2b; ``tools/real_bridge_laser.py`` prints it for the launcher)."""
    from forklift_core.localization.slam_real_scan import base_from_laser

    return base_from_laser(lidar.mount, rear_offset_m)


def _real_slam_operating(prep: RunPrep, host, transport, box: dict, path: str,
                         connect_timeout_s: float = 5.0) -> RunSources:
    """The real SLAM source for the operating run: as the HIL's (``_real_slam_sources``) but the REAL client, no HIL
    check, and no layer branch (the scan source offers its own scans)."""
    import socket as socket_

    from forklift_core.live.real_slam_source import RealSlamLink, RealSlamSource
    from forklift_core.localization.start_reloc import base_from_rear

    sock = socket_.socket(socket_.AF_UNIX, socket_.SOCK_STREAM)
    try:
        # the start's remaining time when the operating run gives it (U3e-1 plan 6.1 step 9), never more than 5 s
        sock.settimeout(max(0.001, min(5.0, float(connect_timeout_s))))
        sock.connect(str(path))
    except OSError as err:
        sock.close()
        raise AssemblyRefused([f"slam_bridge:{err}"]) from err

    def boot():
        st = host.state
        return None if st is None else int(st.boot_id)

    try:
        slam = RealSlamSource(
            RealSlamLink(sock), mode="operating", map_id=prep.map_id, start_ns=prep.start_ns,
            start_pose=base_from_rear(tuple(float(v) for v in prep.cfg.start_rear), prep.rear_offset_m),
            odom_history=lambda: box["loop"].odom_history,
            odom_base=lambda: base_from_rear(box["loop"].odom_pose, prep.rear_offset_m),
            odom_gen=lambda: box["loop"].odom_gen, boot=boot, rear_offset_m=prep.rear_offset_m,
            session_epoch=lambda: prep.epoch, hil_gate=None,
            base_from_laser=bridge_laser_xyz_yaw(prep.lidar, prep.rear_offset_m))
    except BaseException:
        sock.close()
        raise
    slam.client = Client.REAL
    sources = RunSources(client=Client.REAL, host=host, transport=transport, scans=None, slam=slam, imu=None)
    sources.on_assembled.append(lambda loop: loop.odom_listeners.append(slam.odom_step))
    sources.closers.append(sock.close)
    return sources


def _real_slam_sources(prep: RunPrep, host, transport, scan, imu, box: dict, path: str) -> RunSources:
    """The real SLAM source on the bridge at ``path`` (7c-1b-2b): connected here (a refused connection refuses the
    assembly), its scans the layer's own (the synthetic cast is exact: its first beam is the interval)."""
    import socket as socket_

    sock = socket_.socket(socket_.AF_UNIX, socket_.SOCK_STREAM)
    try:
        sock.settimeout(5.0)
        sock.connect(str(path))
    except OSError as err:
        sock.close()
        raise AssemblyRefused([f"slam_bridge:{err}"]) from err

    def boot():
        st = host.state
        return None if st is None else int(st.boot_id)

    try:
        return _real_slam_on(sock, prep, host, transport, scan, imu, box, boot)
    except BaseException:
        sock.close()  # never left to the collector (Codex 7c-1b-2b round 1 P2-2)
        raise


def _real_slam_on(sock, prep, host, transport, scan, imu, box, boot) -> RunSources:
    from forklift_core.live.real_slam_source import RealSlamLink, RealSlamSource
    from forklift_core.localization.slam_real_scan import from_layer
    from forklift_core.localization.start_reloc import base_from_rear

    # operating, named here (7a-3 plan 3, Codex round 3 P2-1): the HIL's synthetic cast is exact -- its first beam
    # is its interval -- so its scans start the session and carry evidence as before
    slam = RealSlamSource(
        RealSlamLink(sock), mode="operating", map_id=prep.map_id, start_ns=prep.start_ns,
        start_pose=base_from_rear(tuple(float(v) for v in prep.cfg.start_rear), prep.rear_offset_m),
        odom_history=lambda: box["loop"].odom_history,
        odom_base=lambda: base_from_rear(box["loop"].odom_pose, prep.rear_offset_m),
        odom_gen=lambda: box["loop"].odom_gen, boot=boot, rear_offset_m=prep.rear_offset_m,
        session_epoch=lambda: prep.epoch,
        # 7r plan 3: the HIL transport's own check, judged at every entry point (a HIL run's source only)
        hil_gate=lambda: (getattr(transport, "hil_plant", False) is True, getattr(transport, "hil_boot", None),
                          getattr(transport, "hil_losses", 0)))
    slam.client = Client.HIL
    sources = RunSources(client=Client.HIL, host=host, transport=transport, scans=scan, slam=slam, imu=imu,
                         slam_feed=lambda layer_scan: slam.offer(from_layer(layer_scan, prep.start_ns)))
    sources.on_assembled.append(lambda loop: loop.odom_listeners.append(slam.odom_step))
    sources.closers.append(sock.close)
    return sources


class _NullHolds:
    """S10-A's phases never hold at a cusp (the hold phases are return_home's; S10-A stops at the lift)."""

    def sync(self, *a, **k):
        pass

    def tick(self, *a, **k):
        pass

    def held(self, *a, **k):
        return {}

    def decide(self, *a, **k):
        pass

    def close_watch(self, *a, **k):
        pass


class _SlamGuard:
    """The SLAM source behind the loop: any exception marks the loop's ``fatal_source`` (and still raises, so that tick
    is an emergency), at most SLAM_REPLIES_PER_CALL replies a call (the oldest dropped, counted)."""

    def __init__(self, src, mark, counts) -> None:
        self._src, self._mark, self._counts = src, mark, counts
        self.map_id = getattr(src, "map_id", None)
        # PR 7 design 6.1, 2: the real source's acceptance checks and seed end states, behind the same guard (only
        # when the source has them: the loop asks for them by presence)
        for name in ("can_confirm", "can_seed", "seed_state"):
            if callable(getattr(src, name, None)):
                setattr(self, name, lambda *a, _n=name: self._call(_n, *a))

    def _call(self, name, *a):
        try:
            return getattr(self._src, name)(*a)
        except Exception as exc:
            self._mark(f"slam:{name}:{type(exc).__name__}:{exc}")
            raise

    def relocalize(self, epoch):
        return self._call("relocalize", epoch)

    def seed(self, epoch, base_pose, stamp_s):
        return self._call("seed", epoch, base_pose, stamp_s)

    def request_processed(self, epoch, attempt, not_before):
        return self._call("request_processed", epoch, attempt, not_before)

    def replies(self, now_ns):
        try:
            out = list(getattr(self._src, "replies")(now_ns))  # the iteration is the source's too (round 1 P3)
        except Exception as exc:
            self._mark(f"slam:replies:{type(exc).__name__}:{exc}")
            raise
        if len(out) > SLAM_REPLIES_PER_CALL:
            self._counts["slam_dropped"] += len(out) - SLAM_REPLIES_PER_CALL
            out = out[-SLAM_REPLIES_PER_CALL:]
        return out


class _ImuGuard:
    """The IMU behind the loop: a ``SourceFatal`` marks the run fatal; any other error is the 5b missing batch. A
    source with ``generation()`` (PR 7 design 5) keeps it behind the same guard."""

    def __init__(self, src, mark) -> None:
        self._src, self._mark = src, mark
        if callable(getattr(src, "generation", None)):
            self.generation = self._generation

    @property
    def last_read_capped(self) -> bool:
        try:
            return getattr(self._src, "last_read_capped", False) is True
        except SourceFatal as exc:  # marked as any other read's; the frame waits (round 2 P2)
            self._mark(f"imu:{exc}")
            return True

    def _generation(self):
        try:
            return self._src.generation()
        except SourceFatal as exc:
            self._mark(f"imu:{exc}")
            raise

    def samples(self, upto_ns, limit):
        try:
            return list(self._src.samples(upto_ns, limit))  # its iteration too (round 2 P1-2)
        except SourceFatal as exc:
            self._mark(f"imu:{exc}")
            raise


class _DepthGuard:
    def __init__(self, src, mark) -> None:
        self._src, self._mark = src, mark

    def frames(self, now_ns):
        try:
            return list(self._src.frames(now_ns))  # its iteration too (round 2 P1-2)
        except Exception as exc:
            self._mark(f"depth:{type(exc).__name__}:{exc}")
            raise


def assembly_refusal(prep: RunPrep, sources: RunSources) -> list:
    """Every structural reason the run is not assembled (design 10.1); empty: it may."""
    out = []
    client = sources.client
    if client is Client.REAL:
        # the operating unlock (plan 2.4): S10's preconditions instead of PR 7's unconditional refusal (the lock that
        # refused whatever the record said was removed by U3e-2)
        judged = real_preconditions(prep, sources.host, sources.transport)
        prep.judgement = list(judged)  # the list the assembly judged on; the run's grant takes this one (U3e-1)
        out += judged
    elif client is not Client.HIL:
        out.append(f"client_unsupported:{client}")
    host_client = getattr(sources.host, "client", None)
    if host_client is not client:
        out.append(f"host_client_mismatch:{host_client}")
    for name in ("scans", "slam", "imu"):
        src_client = getattr(getattr(sources, name), "client", client)
        if getattr(sources, name) is not None and src_client is not client:
            out.append(f"source_client_mismatch:{name}")
    tr = sources.transport
    if getattr(tr, "fake_mcu", False) is True:
        out.append("transport_fake_mcu")
    if client is Client.HIL and getattr(tr, "hil_plant", False) is not True:
        out.append("hil_check_not_held")
    if getattr(sources.slam, "map_id", None) != prep.map_id:
        out.append(f"slam_map_mismatch:{getattr(sources.slam, 'map_id', None)}")
    names = {str(s.get("name")) for s in prep.layer_cfg.get("sensors") or ()}
    sensor = getattr(sources.scans, "sensor", None)
    if sensor not in names:
        out.append(f"scan_sensor_unknown:{sensor}")
    if prep.watches.imu_policy == "required" and sources.imu is None and client is not Client.REAL:
        out.append("imu_required")  # the operating run's unqualified IMU is a readiness refusal instead (plan 2.7)
    if sources.depth is None and "d5_pocket" not in BYPASSABLE:
        out.append("no_depth_no_bypass")
    if prep.jobs is None:
        out.append("plan_worker_missing")
    elif not isinstance(getattr(type(prep.jobs), "alive", None), property) and not hasattr(prep.jobs, "alive"):
        out.append("plan_worker_no_liveness")
    if sources.depth is not None and (not callable(getattr(sources.depth, "pocket_frame", None))
                                      or not callable(getattr(sources.depth, "pocket_check", None))):
        out.append("depth_without_pocket_hooks")
    return out


def run_time_setting(data: dict, key: str, default: float) -> float:
    """A run-time margin from autonomy.yaml (U3b-1): a finite positive number, else the run is refused -- a NaN would
    leave every comparison open (Codex U3b-1 round 1 P2-4)."""
    v = data.get(key, default) if isinstance(data, dict) else default
    try:
        ok = not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(float(v)) and v > 0
    except OverflowError:
        ok = False  # an integer too large for a float (round 2 P3)
    if not ok:
        raise AssemblyRefused([f"settings_unreadable:{key}"])
    return float(v)


def _case_a_started(prep):
    """The operating run's adoption check (plan 2.4, U3b-1): the adopted plan's approach and insert go forward only
    and stay within the approved corridor -- S10-A's taped start to the insert end on the approved placement -- else
    the adoption raises and the loop ends the mission before any ARM."""
    from forklift_core.live import real_case_a as rca

    def started(t, inputs, plan, ready):
        cfg = prep.cfg
        if cfg.check_pallet_pose is None:
            raise asm.RealRequireFailed("case_a:placement_unset")
        seg = rca.corridor(cfg.start_rear, cfg.check_pallet_pose, prep.geometry.inserted_offset_m)
        why = rca.path_refusal(plan, seg)
        if why is not None:
            raise asm.RealRequireFailed(why)

    return started


def board_facts(host, transport) -> dict:
    """The first STATE's facts for the operating preconditions: the opened fd's USB product string and serial, the
    first STATE read on that fd (the transport's own; else the host's latest), the host's build refusal for it, and
    whether any bad build was ever seen on the fd."""
    state = getattr(transport, "first_state", None) or getattr(host, "state", None)
    refusal = None
    if state is not None:
        try:
            refusal = host.build_refusal(state)
        except Exception as err:  # noqa: BLE001 -- recorded as the refusal
            refusal = f"build_refusal:{type(err).__name__}"
    return {"usb_product": getattr(transport, "usb_product", None), "usb_serial": getattr(transport, "usb_serial", None),
            "state_seen": state is not None, "drive_refusal": refusal,
            "boot_id": None if state is None else int(state.boot_id),
            "bad_build_seen": bool(getattr(transport, "bad_build_seen", False))}


def real_preconditions(prep, host, transport) -> list:
    """``real_preconditions.operating_preconditions`` on this run's paths and the board's facts (plan 2.4)."""
    import yaml

    from forklift_core.live import real_preconditions as rp

    p = prep.paths
    try:
        facts = rp.gather(record_path=p.preconditions, lidar_path=p.lidar_config, imu_path=p.imu_config,
                          settings_path=p.settings, s10a_path=p.s10a, layer_path=p.layer, budget_path=p.budget,
                          vehicle_path=p.vehicle, watch_path=p.watch, board=board_facts(host, transport))
    except rp.GATHER_ERRORS as err:
        return [f"precondition:facts_unreadable:{type(err).__name__}:{err}"]
    # the corridor end the run will drive is the one prep built from the chassis it loaded, not a second reading
    # (Codex unlock U1 review round 3 P1)
    facts.inserted_offset_m = float(prep.geometry.inserted_offset_m)
    # C12 compares the record with the calibration the run loaded, not a second reading (Codex U1 round 4 P1)
    if facts.vehicle_calibration_id != prep.vehicle.calibration_id:
        changed_calib = True
    else:
        changed_calib = False
    facts.vehicle_calibration_id = prep.vehicle.calibration_id
    out = rp.operating_preconditions(facts)
    prep.operator_load = rp.operator_load(facts)  # the same record bytes as the judgement (plan 2.9)
    if changed_calib:
        out.append("precondition:files_changed:vehicle_calibration")
    # the judgement's bytes are the ones the run loaded (Codex unlock U1 review P1-2)
    out += [f"precondition:files_changed:{name}" for name, sha in facts.file_hashes.items()
            if name in prep.hashes and prep.hashes[name] != sha]
    out += [f"precondition:files_changed:{name}" for name in ("chassis_params", "chassis_urdf")
            if prep.hashes.get(name) is None or facts.file_hashes.get(name) != prep.hashes.get(name)]
    if facts.stop_files.get("odometry_age") != prep.hashes.get("odometry_age"):
        out.append("precondition:files_changed:odometry_age")
    return out


def real_layer_cfg(layer_cfg: dict, lidar) -> dict:
    """The operating run's layer file with the LiDAR mount, the scan period and whether the mount is measured from the
    run's one ``LidarConfig`` (docs/plans/2026-11-26-real-operating-assembly.md 2.3: the layer's projection, stop delay
    and odometry allowance and the SLAM's scans use one calibration -- the replay's rule, ``replay_layer_cfg``)."""
    from forklift_core.live.replay_assembly import replay_layer_cfg

    out = replay_layer_cfg(layer_cfg, lidar)
    out["sensors"][0]["measured"] = bool(lidar.mount_measured)
    return out


def _layer_sensors(layer_cfg: dict) -> dict:
    out = {}
    for s in layer_cfg.get("sensors") or ():
        xyz = [float(v) for v in s["xyz"]]
        out[str(s["name"])] = ((xyz[0], xyz[1], math.radians(float(s.get("yaw_deg", 0.0)))), True,
                               {"measured": s.get("measured") is True})
    return out


def lidar_mount(prep: RunPrep, sensor: str = "lidar") -> tuple:
    """The scan's mount in the rear-axle frame as the layer places it (the synthetic scan casts from the same)."""
    return _layer_sensors(prep.layer_cfg)[sensor][0]


def measurement_view(host):
    """The production measurement evidence from the newest STATE (design 10.1): steering and lift valid and in range,
    the wheel counts' faults clear."""
    s = host.state
    if s is None:
        return ra.MeasurementView(False, False, False, False)
    f = FaultFlag(int(s.fault_flags))
    steer = not (f & (FaultFlag.STEER_SENSOR_INVALID | FaultFlag.STEER_OUT_OF_RANGE)) and math.isfinite(
        float(s.steering_rad))
    lift = not (f & FaultFlag.LIFT_SENSOR_INVALID) and math.isfinite(float(s.lift_m))
    origin = lift and not (f & FaultFlag.LIFT_OUT_OF_RANGE)
    return ra.MeasurementView(bool(steer), bool(lift), bool(origin), not (f & ENCODER_CUT))


def assemble_run(prep: RunPrep, sources: RunSources, *, events=None, clock=time.monotonic_ns,
                 layer_worker=None) -> "RealRun":
    """The loop on the given sources (design 10.1), or ``AssemblyRefused``. ``layer_worker``: the layer's worker
    (default a ``ThreadWorker``; the fake-clock tests pass a synchronous one -- the only thing they replace)."""
    from forklift_core.live.real_layer import RealLayer, ThreadWorker, real_layer_configs
    from forklift_core.planning import Pose2D
    from forklift_core.live.real_teleop import DelayBound, EncoderSpeedParams, real_auto_refusal
    from forklift_core.localization.slam_pose import SlamPoseTracker
    from forklift_core.localization.start_reloc import base_from_rear
    from forklift_core.perception.obstacle_grid import AgeErrorTable
    from forklift_core.planning.geometry import Footprint
    from forklift_core.plant.port import RealPlantPort
    from forklift_core.plant.real_vehicle import RealOdometry
    from forklift_core.control.cusp_stall import CuspStallMonitor
    from forklift_core.safety.arbiter import Arbiter, ArbiterConfig

    refused = assembly_refusal(prep, sources)
    if refused:
        raise AssemblyRefused(refused)
    cfg, kin, vehicle, host = prep.cfg, prep.kin, prep.vehicle, sources.host
    box: dict = {}

    def mark(why: str) -> None:
        loop_ = box.get("loop")
        if loop_ is not None and loop_.fatal_source is None:
            loop_.fatal_source = why

    port = RealPlantPort(host, kin, lift_motion_counts=True)
    odo = RealOdometry(vehicle, max_gap_s=0.2)
    tracker = SlamPoseTracker(max_age_s=ra.SLAM_MAX_AGE_S, hold_limit_m=ra.HOLD_MAX_M, bootstrap_scan_id=0)
    # the real SLAM source starts by relocalizing (PR 7 design 6.1, 6, 7c-2c): no identity
    # correction -- the session opens a generation the operator confirms (or declares) on the bridge's first instance
    from forklift_core.live.real_slam_source import RealSlamSource

    # by kind, not by the epoch contract: the synthetic SLAM has ``relocalize`` too (Codex 7c-2c round 1 P2)
    real_slam = isinstance(sources.slam, RealSlamSource)
    if not real_slam:
        # the synthetic SLAM only (HIL, refused otherwise): the start is the odometry origin and its correction the
        # identity (10.5), so tracking starts on it
        tracker.receive(0, -1.0, "processed", (0.0, 0.0, 0.0))

    def odom_base():
        return base_from_rear(box["loop"].odom_pose, prep.rear_offset_m)

    def slam_rear(t):
        from forklift_core.localization.start_reloc import rear_from_base

        base = tracker.map_from_base(t, odom_base(), closed=box["loop"].outputs_closed)
        return np.asarray(rear_from_base(base, prep.rear_offset_m), dtype=float)

    def steering():
        q = box["loop"]._measured_steering()  # noqa: SLF001
        return (0.0 if q is None else q,)

    tick = SimpleNamespace(slam_rear=slam_rear, speed=lambda: box["loop"].speed, steering=steering, kinematics=kin,
                           loaded=lambda: False, cusp_holds=_NullHolds(), cusp_monitor=CuspStallMonitor([]),
                           standing=lambda: bool(box["loop"].core.slam.get("stop_now")),
                           now_s=lambda: (int(clock()) - prep.start_ns) / 1e9)
    # the real layer from the configuration files (10.9: the budget, the overdue end, the scan period, the state
    # delay and the sensor timeout read here, not the rig's constants)
    lc, b = prep.layer_cfg, prep.layer_cfg["bridge"]
    data = (REPO / lc["odometry_age"]).read_bytes()  # one read: parsed and hashed from the same bytes
    if prep.hashes.get("odometry_age") is not None and hashlib.sha256(data).hexdigest() != prep.hashes["odometry_age"]:
        raise AssemblyRefused(["files_changed:odometry_age"])  # the table judged is the table the layer takes
    age = json.loads(data.decode("utf-8"))
    table = AgeErrorTable(tuple(age["ages_s"]), tuple(age["cumulative_position_m"]), tuple(age["cumulative_yaw_rad"]))
    kappa = max(abs(k) for k in vehicle.steering.curvatures_inv_m)
    # the layer's grid over the S10-A field and a metre round it (the odometry frame is the map frame at the HIL start,
    # 10.5); the teleop bridge's +-hall_half_m square (800 x 800 cells) is far larger than the field it drives in
    x0, x1, y0, y1 = _odom_bounds(cfg)
    hall = SimpleNamespace(x_min_m=x0 - HALL_MARGIN_M, x_max_m=x1 + HALL_MARGIN_M, y_min_m=y0 - HALL_MARGIN_M,
                           y_max_m=y1 + HALL_MARGIN_M)
    # the STATE delay a measurement interval may have and still place scans (design 10.10): the layer file's 0.03 s is
    # the teleop bridge's measured bound; the 50 Hz loop reads once per 20 ms over a 10 ms STATE period, so its
    # round-trip bound reaches 30-40 ms and 0.03 s would leave every other STATE out and no scan placed
    scan_period_s, state_delay_s = float(b["scan_period_s"]), RUN_STATE_DELAY_S
    grid_cfg, perm_cfg = real_layer_configs(lc, error_table=table, budget_s=prep.budget_s,
                                            scan_period_s=scan_period_s, state_delay_s=state_delay_s,
                                            v_scan_mps=1.5 * float(cfg.max_speed_mps), kappa_max_inv_m=kappa,
                                            hall=hall)
    hull = prep.geometry.unloaded_footprint
    body = Footprint(prep.body_front_m, hull.rear_m, hull.half_width_m)
    worker = ThreadWorker() if layer_worker is None else layer_worker
    try:
        layer = RealLayer(grid_cfg, perm_cfg, sensors=_layer_sensors(lc), unloaded=hull,
                          loaded=prep.geometry.loaded_footprint, body=body,
                          placer=lambda a, b_: box["loop"].place_scan(a, b_), budget_s=prep.budget_s,
                          overdue_end_s=prep.overdue_end_s, scan_period_s=scan_period_s,
                          scan_max_age_s=float(lc["permission"]["sensor_timeout_s"]), worker=worker,
                          odom_now=lambda: box["loop"].odom_travel)
        hooks = asm.RealHooks(lift_m=lambda: float("nan") if host.state is None else float(host.state.lift_m),
                              odom_base=odom_base, ready=lambda t, inputs: None,
                              started=_case_a_started(prep) if sources.client is Client.REAL else (lambda *a: None),
                              # U3b-2: the operating run's bypass region must be the one its D5 confirmation named
                              confirmed_region=(lambda: box["loop"].confirmed_region())
                              if sources.client is Client.REAL and sources.depth is None else None,
                              record=lambda name, obj: _emit(events, {"t": "record", "name": str(name)}),
                              geometry=prep.geometry,
                              # the depth source's frame and check builder (PR 7; HIL has none: D5 only by the bypass)
                              pocket_frame=None if sources.depth is None else (
                                  lambda: _frame_seen(box, sources.depth)),
                              pocket_check=None if sources.depth is None else (
                                  lambda *a: _register_check(box, sources.depth, a)),
                              operator=lambda r: box["loop"].operator_check(r), tick=tick, layer=layer,
                              occupancy=lambda: box["loop"].planning_occupancy())
        core = asm.assemble(
            cfg=cfg, loaded_map_id=prep.map_id, geometry=prep.geometry, pallet_geometry=None, kinematics=kin,
            settings=prep.settings, planner_config=prep.planner, travel_config=prep.planner,
            speeds=prep.data["speeds"], mission_stages=prep.data["mission_stages"],
            curvatures={"preview": prep.cap}, layer=layer, applied=(0.0, 0.0, 0.0), slam_tracker=tracker,
            jobs=prep.jobs, hooks=hooks, extra_options=core_options(prep.settings, prep.rear_offset_m))
        core.obstacle.update(ticks={}, min_allowed={}, slowed={}, reasons={})
        tokens = itertools.count(1)
        token_prefix = secrets.token_hex(4)
        arbiter = Arbiter(prep.epoch, ArbiterConfig(tick_s=0.02, auto_lease_loss="stop", external_stop=True,
                                                    disarm_ignores_control_gen=True, real_requests=True),
                          lambda: f"{token_prefix}-{next(tokens)}", mission=core.commands)
        link = RealLink(arbiter, prep.channels, tick_period_s=0.05, state_period_s=0.1)
        permit = ra.AutoPermit(host, port, outstanding=port.outstanding,
                               stop_since=lambda n: box["loop"].rearm_stand(n))
        arbiter.rearm_refusal, arbiter.on_rearmed = permit.arbiter_callbacks()  # seconds -> ns (10.1)
        params = EncoderSpeedParams(radius_m=vehicle.wheel_radius_m, counts_per_rev=vehicle.counts_per_wheel_rev,
                                    track_m=vehicle.track_m, kappa_max=prep.cap, axle_accel_mps2=0.5, max_age_s=0.1,
                                    dt_min_s=0.02, dt_max_s=0.2)

        def t_of(n):
            return (int(n) - prep.start_ns) / 1e9

        def snapshot_view(n):
            key, applied_v = layer.installed_key(), int(core.obstacle["version"])
            newest = layer.installed_newest_s()
            applied_c = tuple(float(v) for v in core.obstacle["applied"])
            wait_ = layer.watchdog(t_of(n))
            # the correction the installed result was made with, compared with the one applied now (round 1 P2)
            return ra.SnapshotView(None if key is None else layer.installed_correction(), applied_c,
                                   None if newest is None else int(prep.start_ns + newest * 1e9),
                                   layer.installing(), None if key is None else key.correction_gen, applied_v,
                                   wait_ if wait_ == "layer_overdue" else None)

        def worker_ok(n):
            return bool(prep.jobs.alive) and worker.alive

        def link_reason(n):
            if not host.health(n).fresh:
                return "stale"
            why = host.build_refusal()
            return None if why is None else f"build:{why}"

        operating = sources.client is Client.REAL  # the operating REAL run (make_real_sources), never HIL or replay
        # C7 (plan 2.7): qualified only with its source started on a measured config and a fitting profile, and the
        # watch required; autonomy.yaml's imu: none is unqualified there (a setting never removes the watch)
        imu_unqualified = operating and not (sources.imu is not None and sources.imu_qualified is True
                                             and prep.watches.imu_policy == "required")
        evidence = ra.Evidence(
            worker_ok=worker_ok, baseline_ok=lambda n: core.baseline is not None,
            layer_ok=lambda n: layer.ok(t_of(n)), link_reason=link_reason,
            measurement=lambda n: measurement_view(host),
            odometry_continuous=lambda n: box["loop"].odom_continuous,
            # the loop replaces pose_reply_ns with its own epoch's freshness (an epoch-contract source, 5c-2 design 6, 4)
            # the hold's owner is the mission it began in (core.hold_slam) against the running mission -- both mission
            # ids in the generation fields (4b: "a hold must be this mission's"; a pause or a wait moves the control
            # generation within one mission; round 1 P2)
            slam=lambda n: ra.SlamView(tracker.mode, None, float(getattr(tracker, "hold_travel_m", 0.0)),
                                       getattr(core, "hold_mission_id", None) if tracker.mode == "holding" else None),
            snapshot=snapshot_view,
            operator_start_ok=lambda n: box["loop"].operator_start.ok(
                None if arbiter.lease is None else arbiter.lease.lease_id),
            core_refusal=lambda n: real_auto_refusal(core.safety_state(None)),
            mission_gen=lambda n: core.commands.mission_id,
            imu_unqualified=(lambda n: True) if imu_unqualified else None)
        scans = sources.scans
        counts = sources.counts

        def scan_events(now_ns):
            for kind, sensor in scans.scan_source_events(now_ns):
                if kind == "lost":
                    layer.source_lost(sensor)
                elif kind == "connected":
                    layer.source_connected(sensor)
                elif kind == "boundary" and operating:
                    box["loop"].scan_boundary(now_ns, sensor)  # U3a: the evidence boundary's latch
                else:
                    raise ValueError(f"scan source event {kind!r}")

        def scan_feed(now_ns):
            scan_events(now_ns)
            got = list(scans.scans(now_ns))
            if len(got) > SCANS_PER_CALL:
                counts["scans_dropped"] += len(got) - SCANS_PER_CALL
                got = got[-SCANS_PER_CALL:]
            loop_ = box["loop"]
            for scan in got:
                refused = layer.add_scan(scan, loop_._t(now_ns), initial_correction=tuple(core.obstacle["applied"]),  # noqa: SLF001
                                         initial_version=int(core.obstacle["version"]))
                if refused is not None:
                    counts[f"layer_refused_{refused}"] = counts.get(f"layer_refused_{refused}", 0) + 1
                    continue  # the scan the layer takes is the scan SLAM takes: a refused one goes to neither
                if sources.slam_feed is not None:
                    try:
                        sources.slam_feed(scan)  # the SLAM path's branch: the same scan (PR 7 design 6.1, 1)
                    except Exception as exc:
                        mark(f"slam:offer:{type(exc).__name__}:{exc}")
                        raise
            fatal = getattr(scans, "fatal", None)
            if fatal is not None:
                mark(f"scan:{fatal}")  # 7r: the replay's socket_error / replay_mismatch ends the run (plan 3)

        loop = RealLoop(core=core, session=RealSession(core, link), permit=permit, port=port, host=host, link=link,
                        channels=prep.channels, transport=sources.transport, params=params, odometry=odo,
                        kinematics=kin, max_speed_mps=float(cfg.max_speed_mps), start_ns=prep.start_ns,
                        slam_source=_SlamGuard(sources.slam, mark, counts), evidence=evidence,
                        # the plan's return leg ends at the S10-A wait pose (S10 stops at the lift; the planner plans the
                        # whole mission, as the dry assembly's reference plan)
                        base_scenario=core.options.scenario, wait_pose=Pose2D(*(float(v) for v in cfg.wait_pose)),
                        depth_source=None if sources.depth is None else _DepthGuard(sources.depth, mark),
                        clock=clock, real_layer=layer, delay_bound=DelayBound(), state_delay_s=state_delay_s,
                        pallet_size=(float(cfg.pallet_depth_m), float(cfg.pallet_width_m)), kappa_max_inv_m=kappa,
                        operator_start=asm.OperatorStart(prep.epoch), start_rear=tuple(cfg.start_rear),
                        rear_offset_m=prep.rear_offset_m, watches=prep.watches,
                        imu_source=None if sources.imu is None or imu_unqualified else _ImuGuard(sources.imu, mark),
                        scan_feed=scan_feed, events=events, operating=operating,
                        scan_events=scan_events if operating else None, imu_unqualified=imu_unqualified,
                        d5_region=hooks.bypass_region)
        loop.scan_refusal = lambda: getattr(scans, "last_refusal", None)
        if operating:
            # U3b-1 (plan 2.2, 2.4): the approved placement and the run-time margins (the run sets its deadline)
            loop.check_pallet_pose = None if cfg.check_pallet_pose is None else tuple(cfg.check_pallet_pose)
            for key in ("mission_time_budget_s", "insert_time_min_s"):
                setattr(loop, key, run_time_setting(prep.data, key, getattr(loop, key)))
            # U3c (plan 2.9): an assembled operating run passed every assembly precondition; the confirmation's load
            from forklift_core.live.real_preconditions import ASSEMBLY_CHECKS

            loop.preconditions_passed = ASSEMBLY_CHECKS
            loop.operator_load = prep.operator_load
        box["loop"] = loop
        arbiter.stop_evidence = lambda now: loop.physical_stop(now)
        if real_slam:
            # 6.1, 6: the session's start generation, before the first tick -- nothing is submitted before the
            # operator's confirmation (or a declaration) closes it
            core.require_relocalization(0.0, "session_start")
        for fn in sources.on_assembled:
            fn(loop)
        return RealRun(prep, sources, loop, layer=layer, layer_worker=worker, clock=clock)
    except BaseException:
        getattr(worker, "close", lambda: None)()  # the layer thread never outlives a refused or failed assembly
        raise


def _odom_bounds(cfg) -> tuple:
    """The S10-A field (map frame) in the odometry frame, whose origin is the start's rear axle: the bounds themselves
    at the HIL's zero start, else the box round the field's corners carried by the start's inverse (a replay starts
    where its record did, 7r-2)."""
    from forklift_core.localization.slam_pose import compose, invert

    x0, x1, y0, y1 = (float(v) for v in cfg.bounds)
    start = tuple(float(v) for v in cfg.start_rear)
    if not any(start):
        return x0, x1, y0, y1
    inv = invert(start)
    pts = [compose(inv, (x, y, 0.0))[:2] for x in (x0, x1) for y in (y0, y1)]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), max(xs), min(ys), max(ys)


def _frame_seen(box, depth):
    """The depth source's frame handed to the core (``known_pocket_frame``), kept: the check is built from this very
    frame -- the one the core has just judged fresh -- never a second read (round 3 P1)."""
    frame = depth.pocket_frame()
    box["frame"] = frame
    return frame


def _register_check(box, depth, args):
    """Design 10.5 P1-4: the depth source's check built from the frame the core judged (its acquisition in the still
    span, its mission generation, phase and correction), registered in the core before RealHooks installs its region
    as the layer's exemption; a frame not of this moment, or another frame than the judged one, registers nothing."""
    loop = box["loop"]
    core = loop.core
    frame = box.get("frame")
    stamp = args[3] if len(args) > 3 else None
    if frame is None or stamp is None or float(getattr(frame, "stamp_s", float("nan"))) != float(stamp) \
            or getattr(frame, "mission_gen", None) != core.commands.mission_gen \
            or getattr(frame, "phase", None) != core.ms.phase \
            or tuple(getattr(frame, "correction", ()) or ()) != tuple(float(v) for v in core.obstacle["applied"]):
        return None
    check = depth.pocket_check(*args, frame=frame)  # the judged frame itself (the depth contract's keyword)
    if check is not None and getattr(check, "valid", False):
        core.pocket["check"] = check  # registered before the exemption is set (round 2 P1-1)
    return check


def _emit(events, event: dict) -> None:
    if events is not None:
        try:
            events(event)
        except Exception:  # noqa: BLE001 -- the record never holds the tick
            pass


# --- the record (design 10.3) ----------------------------------------------------------------------------------------
def exclusive_record_opener(path, mode: str = "w"):
    """The operating run's record file (plan 6.1, Codex U3e-1 P1-3): created anew (``O_EXCL``, an existing file or a
    link refused, ``O_NOFOLLOW``) and only kept when the opened fd is a regular file -- the record writer never writes
    to a device or an alias of one, and never cuts an earlier run's record."""
    import os
    import stat as stat_

    if mode != "w":
        raise ValueError("the record is written only")
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o644)
    try:
        if not stat_.S_ISREG(os.fstat(fd).st_mode):
            raise OSError(f"{path}: not a regular file")
        return os.fdopen(fd, "w")
    except BaseException:
        os.close(fd)
        raise


class RunRecord:
    """JSONL by a writer thread from a bounded queue: ``put`` never waits (a full queue or a failed writer leaves
    ``incomplete``); ``close(end)`` drains up to the end event and says whether every row was written."""

    def __init__(self, path: Path | None, *, maxsize: int = RECORD_QUEUE, opener=open) -> None:
        """``path`` None keeps nothing (and is never complete); an unopenable file raises before the thread starts."""
        self.path = None if path is None else Path(path)
        self._q: queue.Queue = queue.Queue(maxsize=maxsize)
        self.incomplete: str | None = None
        self.failed: str | None = None
        self.rows = 0
        self.end_written = False  # the end row reached the file (the writer's own word, round 1 P2)
        self.closed = False  # the writer closed the file (round 2 P2-8)
        self._finish = threading.Event()  # close() was called: an empty queue ends the writer
        self._f = None if self.path is None else opener(self.path, "w")
        self._thread = threading.Thread(target=self._run, name="real-run-record", daemon=True)
        self._thread.start()

    def put(self, row: dict) -> None:
        if self.failed is not None:
            self.incomplete = self.incomplete or f"writer_failed:{self.failed}"
            return
        try:
            self._q.put_nowait(row)
        except queue.Full:
            self.incomplete = self.incomplete or "queue_full"

    def _run(self) -> None:
        while True:
            try:
                row = self._q.get(timeout=0.05)
            except queue.Empty:
                if self._finish.is_set():
                    break  # the end row never made it into the queue: stop anyway (round 3 P3)
                continue
            if self.failed is None:
                self._write_row(row)
            if row.get("t") == "end":
                break  # the end row is the last item (no queue slot for a separate marker)
        if self._f is not None:  # the writer owns the file to its close (round 2 P2-8)
            try:
                self._f.close()
            except Exception as err:  # noqa: BLE001
                self.failed = self.failed or f"close:{type(err).__name__}:{err}"
        self.closed = True

    def _write_row(self, row: dict) -> None:
        try:
            if self._f is not None:
                self._f.write(json.dumps(row, default=_json_default) + "\n")
                self._f.flush()
            self.rows += 1
            if row.get("t") == "end":
                self.end_written = True
        except Exception as err:  # noqa: BLE001 -- told by ``failed`` and the exit code
            self.failed = f"{type(err).__name__}:{err}"
            self.incomplete = self.incomplete or f"writer_failed:{self.failed}"

    def close(self, end: dict, timeout_s: float = 5.0) -> bool:
        """The end row (carrying ``record_incomplete`` when it is known), then the writer's end, within one deadline:
        True when every row reached the file and the writer closed it. A writer stuck in a write is not waited for
        past ``timeout_s`` (it owns the file; the record is then incomplete)."""
        deadline = time.monotonic() + timeout_s
        end = dict(end, record_incomplete=self.incomplete)
        try:
            self._q.put(end, timeout=max(0.0, deadline - time.monotonic()))  # read, it ends the writer itself
        except queue.Full:
            self.incomplete = self.incomplete or "queue_full_at_end"
            self._finish.set()  # no end row: the writer stops on an empty queue (round 4 P2-6)
        self._thread.join(max(0.0, deadline - time.monotonic()))
        return (not self._thread.is_alive() and self.closed and self.failed is None and self.incomplete is None
                and self.end_written and self._f is not None)  # no file: nothing kept, never complete


def _json_default(v):
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (np.floating,)):
        return float(v)
    if isinstance(v, (set, frozenset, tuple)):
        return list(v)
    return repr(v)


def tick_row(rec, now_ns: int) -> dict:
    return {"t": "tick", "ns": int(now_ns), "core_t": rec.t, "interval_s": rec.start_interval_s, "branch": rec.branch,
            "reason": rec.reason, "seqs": list(rec.seqs), "phase": rec.phase, "permit": rec.permit,
            "elapsed_s": rec.elapsed_s, "notes": list(rec.notes), "stop_decided_ns": getattr(rec, "stop_decided_ns", None)}


# --- the run (design 10.3) -------------------------------------------------------------------------------------------
class RealRun:
    """The assembled loop with its wall-clock schedule and its end."""

    def __init__(self, prep: RunPrep, sources: RunSources, loop: RealLoop, *, layer, layer_worker,
                 clock=time.monotonic_ns) -> None:
        self.prep, self.sources, self.loop = prep, sources, loop
        self.layer, self.layer_worker = layer, layer_worker
        self.clock = clock
        self.end_reason: str | None = None
        self.ticks = 0
        # U3e-1: the operating judgement the assembly passed on (empty on an assembled operating run), for the grant
        self.judgement = None if prep.judgement is None else list(prep.judgement)

    def start_session(self, now_ns: int) -> None:
        """The permit's callbacks bound and its session check started (two DISARM exchanges; 10.3 ⑧)."""
        self.loop.start_session(now_ns)

    def stop_reason(self, now_ns: int, deadline_ns: int, stop, model_exit=None, record=None) -> str | None:
        """Why the run ends now, or None (10.3): the stop flag, the deadline, the device loss, the model's exit, a
        fatal source or the record's writer failing."""
        tr, loop = self.sources.transport, self.loop
        if stop is not None and stop.is_set():
            return "signal"
        if getattr(tr, "lost", None) is not None:
            return str(tr.lost["reason"] if isinstance(tr.lost, dict) else tr.lost)
        if model_exit is not None:
            code = model_exit()
            if code is not None:
                peer = getattr(tr, "peer_exited", None)
                if peer is not None:
                    peer()
                return f"model_exit:{code}"
        replay = getattr(self.sources, "replay", None)
        why = None if replay is None or loop.fatal_source is not None else replay.step(now_ns)
        if loop.fatal_source is not None:
            return f"source_fatal:{loop.fatal_source}"
        if record is not None and record.failed is not None:
            return f"record_failed:{record.failed}"
        if why is not None:
            return why  # replay_eof: the normal closing (7r plan 2)
        if now_ns >= deadline_ns:
            return "deadline"
        return None

    def run(self, duration_s: float, *, stop=None, model_exit=None, record=None, sleep=None, before_tick=None) -> str:
        """Ticks at 50 Hz until a stop reason: each sleep to the next boundary of ``first + k * 20 ms`` (missed
        boundaries skipped), ``cycle`` given the clock read just after waking (design 1)."""
        sleep = sleep or (lambda s: (stop.wait(s) if stop is not None else time.sleep(s)))
        first = int(self.clock())
        deadline = first + int(float(duration_s) * 1e9)
        # U3b-1 (plan 2.2): the one deadline the submit, the insert and every write read, and the stop flag
        self.loop.run_deadline_ns = deadline
        self.loop.run_stopping = (lambda: stop.is_set()) if stop is not None else None
        nxt = first
        self._permit_seen = getattr(self, "_permit_seen", 0)
        while True:
            now = int(self.clock())
            why = self.stop_reason(now, deadline, stop, model_exit, record)
            if why is not None:
                self.end_reason = why
                return why
            if before_tick is not None:
                before_tick(now)  # the tests' injections (tools/real_autonomy.py)
            rec = self.loop.cycle(now)
            self.ticks += 1
            replay = getattr(self.sources, "replay", None)
            if replay is not None:
                replay.after_tick(now)  # 7r: the IMU the loop did not read in this tick, drained after it
            if record is not None:
                row = tick_row(rec, now)
                loop = self.loop
                rear = getattr(loop, "_rear_now", None)
                row.update(odom=[round(float(v), 5) for v in loop.odom_pose],
                           rear=None if rear is None else [round(float(v), 5) for v in rear],
                           # the layer watchdog's stored first overdue evaluation (core s; the scan judge's 1 s start)
                           overdue_since=getattr(self.layer, "_overdue_since", None))
                record.put(row)
                self.permit_events(record)
            nxt += TICK_NS
            after = int(self.clock())
            if after >= nxt:
                nxt += ((after - nxt) // TICK_NS + 1) * TICK_NS  # missed boundaries skipped
            wait_s = (min(nxt, deadline) - int(self.clock())) / 1e9  # the last sleep ends at the deadline (round 1 P3)
            if wait_s > 0:
                sleep(wait_s)

    def permit_events(self, record) -> None:
        """The permit's new events (a DISARM's reason, the S10 stop's confirmation with its seq) into the record."""
        events = getattr(self.loop.permit, "events", None)
        if events is None or record is None:
            return
        seen = getattr(self, "_permit_seen", 0)
        for ev in list(events)[seen:]:
            record.put({"t": "permit", "ns": int(ev.t_ns), "event": ev.event, "reason": ev.reason,
                        "detail": dict(ev.detail or {})})
        self._permit_seen = len(events)

    def close(self, *, record=None, sleep=None, confirm_s: float = CLOSING_CONFIRM_S) -> str:
        """The closing (10.3): DISARM only, ticks on until a closing DISARM is confirmed or ``confirm_s`` passed --
        ``confirmed`` or ``disarm_unconfirmed`` (``link_lost`` when the device is gone)."""
        loop, tr = self.loop, self.sources.transport
        if getattr(tr, "lost", None) is not None:
            return "link_lost"
        sleep = sleep or time.sleep
        now = int(self.clock())
        loop.begin_closing(now, self.end_reason or "run_end")
        end = now + int(confirm_s * 1e9)
        nxt = now
        while loop.closing_confirmed_ns is None:
            now = int(self.clock())
            if now >= end or getattr(tr, "lost", None) is not None:
                break
            rec = loop.cycle(now)
            if record is not None:
                record.put(tick_row(rec, now))
                self.permit_events(record)
            nxt += TICK_NS
            wait_s = (min(nxt, end) - int(self.clock())) / 1e9
            if wait_s > 0:
                sleep(wait_s)
        # confirmed only within the second from the closing's start (a tick begun before the end that confirms after
        # it is late; round 1 P2)
        started = loop.closing_started_ns if loop.closing_started_ns is not None else now
        if loop.closing_confirmed_ns is not None and loop.closing_confirmed_ns <= started + int(confirm_s * 1e9):
            return "confirmed"
        return "link_lost" if getattr(tr, "lost", None) is not None else "disarm_unconfirmed"

    def close_workers(self, timeout_s: float = 2.0) -> list:
        """The layer worker and the plan worker closed (the plan worker last); what did not end in time."""
        late = []
        for fn in self.sources.closers:
            try:
                fn()
            except Exception as err:  # noqa: BLE001
                late.append(f"source_closer:{err}")
        try:
            self.layer_worker.close()
            th = getattr(self.layer_worker, "_thread", None)
            if th is not None:
                th.join(timeout_s)
                if th.is_alive():
                    late.append("layer_worker")
        except Exception as err:  # noqa: BLE001
            late.append(f"layer_worker:{err}")
        jobs = self.prep.jobs
        if jobs is not None:
            try:
                jobs.close()
            except Exception as err:  # noqa: BLE001
                late.append(f"plan_worker:{err}")
        return late


def header_row(prep: RunPrep, *, synthetic: bool, uninspected=(), model_args=(), extra=None) -> dict:
    return {"t": "head", "synthetic": bool(synthetic), "uninspected": list(uninspected), "start_ns": prep.start_ns,
            "hashes": dict(prep.hashes), "map_id": prep.map_id, "model_args": [str(a) for a in model_args],
            "session_epoch": prep.epoch, "check_plan": prep.check_plan, "kernels": dict(prep.kernels),
            "state_rule": "host_receipt",
            "state_max_age_s": RUN_HOST_CFG.state_max_age_s,
            "scope": "S10-A commands, STATEs, odometry and stop transitions (no position accuracy, pocket contact, "
                     "load, real sensor delay or USB timing)", **(extra or {})}


__all__ = ["EXIT_OK", "EXIT_RECORD", "EXIT_REFUSED", "RUN_HOST_CFG", "RealRun", "RunCancelled", "RunPaths", "RunPrep",
           "RunRecord", "RunSources", "SourceFatal", "assemble_run", "assembly_refusal", "chassis_reference",
           "check_plan", "core_options", "header_row", "lidar_mount", "make_hil_sources", "make_run_sources",
           "map_occupancy", "measurement_view", "mission_geometry",
           "motion_allowed", "pallet_box", "planning_settings", "prepare_run", "read_hil_scene", "read_map",
           "read_settings", "read_vehicle", "run_welcome", "scene_occupancy", "tick_row"]
