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


def mission_geometry(urdf: Path, depth_m: float, width_m: float, reserve_m: float = 0.016):
    from forklift_core.planning.geometry import Footprint
    from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

    ig = _insertion_geometry()
    axle_to_tip, _ = ig.read_chassis_reference_m(urdf)
    return SyntheticMissionGeometry(unloaded_footprint=Footprint(axle_to_tip, 0.17, 0.36), pallet_depth_m=depth_m,
                                    pallet_width_m=width_m, axle_to_fork_tip_m=axle_to_tip,
                                    carriage_limit_m=ig.read_carriage_limit_m(urdf), insertion_reserve_m=reserve_m)


def chassis_reference(urdf: Path) -> tuple:
    """(axle to fork tip, rear axle offset behind base_link, carriage limit) in metres from the chassis model."""
    ig = _insertion_geometry()
    axle_to_tip, rear_x = ig.read_chassis_reference_m(urdf)
    return float(axle_to_tip), abs(float(rear_x)), float(ig.read_carriage_limit_m(urdf))


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


def prepare_run(paths: RunPaths, *, clock=time.monotonic_ns, stop: threading.Event | None = None,
                jobs_factory=None, epoch: int | None = None) -> RunPrep:
    """Everything before the model starts (10.3 ②). ``AssemblyRefused`` with the structural reasons, ``RunCancelled``
    once ``stop`` is set (the plan worker closed by whichever side knows last)."""
    import yaml

    from forklift_core.plant.real_vehicle import chassis_model_path

    stop = stop or threading.Event()
    try:
        cfg = ra.load_s10a(paths.s10a)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as err:
        raise AssemblyRefused([f"s10a_unreadable:{err}"]) from err
    data = read_settings(paths.settings)
    vehicle, kin = read_vehicle(paths.vehicle)
    watches = asm.build_watches(data, paths.watch, vehicle.wheel_radius_m)
    settings, planner, cap = planning_settings(data, kin)
    obstacles: list = []
    if paths.hil:
        obstacles = read_hil_scene(paths.s10a)  # the synthetic LiDAR casts them whatever map plans (round 1 P2)
    if paths.map_bundle is not None:
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
    if paths.hil and any(abs(float(v)) > 1e-12 for v in cfg.start_rear):
        refused.append("hil_start_not_zero")  # the odometry origin is the synthetic SLAM's identity (10.5)
    if refused:
        raise AssemblyRefused(refused)
    urdf = chassis_model_path(paths.vehicle).parent / "forklift.urdf"
    try:
        geometry = mission_geometry(urdf, cfg.pallet_depth_m, cfg.pallet_width_m)
        axle_to_tip, rear_offset, carriage = chassis_reference(urdf)
        layer_cfg = yaml.safe_load(Path(paths.layer).read_text())
        budget = yaml.safe_load(Path(paths.budget).read_text())
        budget_s = float(budget["latency_budget"]["seconds"])
        overdue_end_s = float(budget["overdue_end"]["seconds"])
    except AssemblyRefused:
        raise
    except Exception as err:  # noqa: BLE001 -- a malformed model or layer file is structural
        raise AssemblyRefused([f"config_unreadable:{type(err).__name__}:{err}"]) from err
    # a wall-clock run (real-time plan D2-D3): the fastest obstacle and permission kernels, compiled before the loop
    from forklift_core import kernels

    kernel_mode = kernels.prefer_fast()
    kernel_warm_s = kernels.warm_up()
    plan = check_plan(cfg, planner, geometry, occupancy)
    if not plan.success:
        raise AssemblyRefused([f"check_plan:{plan.status}"])
    hashes = {name: _sha(p) for name, p in (("s10a", paths.s10a), ("settings", paths.settings),
                                             ("vehicle", paths.vehicle), ("watch", paths.watch),
                                             ("layer", paths.layer), ("budget", paths.budget))}
    epoch = secrets.randbelow(0x7FFFFFFE) + 1 if epoch is None else int(epoch)
    prep = RunPrep(paths=paths, cfg=cfg, data=data, vehicle=vehicle, kin=kin, settings=settings, planner=planner,
                   cap=cap, geometry=geometry, rear_offset_m=rear_offset, body_front_m=axle_to_tip - carriage,
                   map_id=map_id, occupancy=occupancy, obstacles=obstacles, watches=watches, layer_cfg=layer_cfg,
                   budget_s=budget_s, overdue_end_s=overdue_end_s, channels=RealChannels(), epoch=epoch,
                   welcome=run_welcome(epoch), start_ns=int(clock()), hashes=hashes, check_plan=plan.status,
                   kernels={"mode": kernel_mode, "warm_up_s": round(float(kernel_warm_s), 3)})
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


def make_hil_sources(prep: RunPrep, host, transport, *, imu_seed: int = 0) -> RunSources:
    """The HIL run's synthetic sources (design 10.2) on a transport whose HIL check holds (else ``SyntheticRefused``):
    the LiDAR casts the scene's obstacles and the target pallet from the loop's odometry pose with the layer's mount,
    once the session check has started; the SLAM keeps the epoch contract on those acquisitions; a still IMU."""
    from forklift_core.live import synthetic_sources as ss
    from forklift_core.localization.start_reloc import base_from_rear

    box: dict = {}
    boxes = list(prep.obstacles) + [pallet_box(prep.cfg)]
    scan = ss.SyntheticScan(transport, Client.HIL, prep.start_ns, odom_pose=lambda: box["loop"].odom_pose,
                            boxes=boxes, mount=lidar_mount(prep),
                            active=lambda: "loop" in box and bool(box["loop"].permit.session_started))
    slam = ss.SyntheticSlam(transport, Client.HIL, scan, map_id=prep.map_id,
                            odom_base=lambda: base_from_rear(box["loop"].odom_pose, prep.rear_offset_m))
    # the IMU only where the watches take one (imu: none builds no IMU watch, and the loop refuses a source without it)
    imu = ss.SyntheticImu(transport, Client.HIL, seed=imu_seed) if prep.watches.imu_policy == "required" else None
    sources = RunSources(client=Client.HIL, host=host, transport=transport, scans=scan, slam=slam, imu=imu)
    sources.on_assembled.append(lambda loop: box.__setitem__("loop", loop))
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
        out.append("client_real:pr7_preconditions")  # 10.5: LiDAR interval, SLAM processed, generations, budget
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
    if prep.watches.imu_policy == "required" and sources.imu is None:
        out.append("imu_required")
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


def _layer_sensors(layer_cfg: dict) -> dict:
    out = {}
    for s in layer_cfg.get("sensors") or ():
        xyz = [float(v) for v in s["xyz"]]
        out[str(s["name"])] = ((xyz[0], xyz[1], math.radians(float(s.get("yaw_deg", 0.0)))), True,
                               {"measured": False})
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
    # HIL only (refused otherwise): the start is the odometry origin and the synthetic SLAM's correction the identity
    # (10.5), so tracking starts on it; PR 7 starts by relocalizing instead
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
    age = json.loads((REPO / lc["odometry_age"]).read_text())
    table = AgeErrorTable(tuple(age["ages_s"]), tuple(age["cumulative_position_m"]), tuple(age["cumulative_yaw_rad"]))
    kappa = max(abs(k) for k in vehicle.steering.curvatures_inv_m)
    # the layer's grid over the S10-A field and a metre round it (the odometry frame is the map frame at the HIL start,
    # 10.5); the teleop bridge's +-hall_half_m square (800 x 800 cells) is far larger than the field it drives in
    x0, x1, y0, y1 = (float(v) for v in cfg.bounds)
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
                              odom_base=odom_base, ready=lambda t, inputs: None, started=lambda *a: None,
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
            mission_gen=lambda n: core.commands.mission_id)
        scans = sources.scans
        counts = sources.counts

        def scan_feed(now_ns):
            for kind, sensor in scans.scan_source_events(now_ns):
                (layer.source_lost if kind == "lost" else layer.source_connected)(sensor)
            got = list(scans.scans(now_ns))
            if len(got) > SCANS_PER_CALL:
                counts["scans_dropped"] += len(got) - SCANS_PER_CALL
                got = got[-SCANS_PER_CALL:]
            loop_ = box["loop"]
            for scan in got:
                layer.add_scan(scan, loop_._t(now_ns), initial_correction=tuple(core.obstacle["applied"]),  # noqa: SLF001
                               initial_version=int(core.obstacle["version"]))

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
                        imu_source=None if sources.imu is None else _ImuGuard(sources.imu, mark),
                        scan_feed=scan_feed, events=events)
        box["loop"] = loop
        arbiter.stop_evidence = lambda now: loop.physical_stop(now)
        for fn in sources.on_assembled:
            fn(loop)
        return RealRun(prep, sources, loop, layer=layer, layer_worker=worker, clock=clock)
    except BaseException:
        getattr(worker, "close", lambda: None)()  # the layer thread never outlives a refused or failed assembly
        raise


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
        if loop.fatal_source is not None:
            return f"source_fatal:{loop.fatal_source}"
        if record is not None and record.failed is not None:
            return f"record_failed:{record.failed}"
        if now_ns >= deadline_ns:
            return "deadline"
        return None

    def run(self, duration_s: float, *, stop=None, model_exit=None, record=None, sleep=None, before_tick=None) -> str:
        """Ticks at 50 Hz until a stop reason: each sleep to the next boundary of ``first + k * 20 ms`` (missed
        boundaries skipped), ``cycle`` given the clock read just after waking (design 1)."""
        sleep = sleep or (lambda s: (stop.wait(s) if stop is not None else time.sleep(s)))
        first = int(self.clock())
        deadline = first + int(float(duration_s) * 1e9)
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
           "check_plan", "core_options", "header_row", "lidar_mount", "make_hil_sources", "map_occupancy", "measurement_view", "mission_geometry",
           "motion_allowed", "pallet_box", "planning_settings", "prepare_run", "read_hil_scene", "read_map",
           "read_settings", "read_vehicle", "run_welcome", "scene_occupancy", "tick_row"]
