"""The real truck's mission core assembly (docs/design/2026-11-19-real-assembly.md, 4b sections 2, 4, 5).

``assemble`` builds the ``MissionCore`` behind ``fail_closed_hooks`` with the real options, the boot inputs whose
values and meaning the tests pin, the S10-A targets (the pallet registered unlocated: only the operator's map pose
makes it a pick), the worker and the baseline -- or refuses with the structural reasons. ``RealHooks`` implements the
hooks the preparation and the adoption read (the table ``ADOPTION_HOOKS``); the tick path's hooks join it in 4c.
No device is opened here.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, replace

import numpy as np
from types import SimpleNamespace

from forklift_core.live import real_autonomy as ra
from forklift_core.mission.baseline import MissionBaseline
from forklift_core.mission.core import MissionCore
from forklift_core.mission.hook_contract import fail_closed_hooks
from forklift_core.mission.state import MissionState
from forklift_core.mission.targets import Pose as TargetPose
from forklift_core.planning import Pose2D
from forklift_core.planning.geometry import Bounds
from forklift_core.planning.pallet_mission import PalletSite, TransportScenario

# The hooks the known-pallet preparation and its adoption read (design 4; the test records them through
# fail_closed_hooks and compares). The tick path's (follow, command, permission, lift, end) join in 4c.
ADOPTION_HOOKS = frozenset({
    "grid_kwargs", "grid_world", "live_mission_ready", "live_mission_start", "wall", "require", "lift_position",
    "path_record", "write_paths", "show_path", "echo", "record_json", "snapshot", "slam_odom_base",
})


# The first-frame stand and the S10 end at the lift read these (design 4); the follow / command / permission tick's
# table is 4c's, with its loop.
STAND_AND_END_HOOKS = frozenset({"known_pocket_frame", "build_pocket_check", "require", "operator_check",
                                 "lift_position", "build_bypass_pocket_check"})

# The control tick's hooks on S10-A's phases (docs/design/2026-11-19-real-loop.md 6; read every tick, by name or
# called, in follow / docking_and_stall / cusp_and_observe_recovery / command / permission / assemble_command).
TICK_HOOKS = frozenset({
    "require", "rear", "set_rear", "slam_rear", "slam_odom_base", "cusp_holds", "cusp_monitor", "cusp_standing",
    "cusp_heading_motion", "loaded", "signed_speed", "held_kappa", "grid_world", "grid_kwargs", "plan_transport_leg",
    "plan_return_leg", "plan_observation_leg", "safety_speed", "steering_position", "dump_tracking", "path_stats",
    "path_record", "wall", "lift_position", "save_npz", "known_pocket_frame", "build_pocket_check", "operator_check",
    "echo", "record_json", "snapshot", "write_paths", "show_path", "add_path_display", "now_s",
    "build_bypass_pocket_check",
})

# the pocket check's exemption bands (sim/isaac/pocket_check.py PocketConfig band_along_m / band_across_m): the D5
# bypass check's region is the pallet with the same bands (real safety wiring design 7.2)
BYPASS_BAND_ALONG_M = 0.60
BYPASS_BAND_ACROSS_M = 0.20


@dataclass
class BypassPocketCheck:
    """The operator's D5 bypass (real safety wiring design 7.2): in the pocket check's place, its region from the
    submitted pallet pose and the check's bands, its answer always clear -- nothing inside the region is seen; the
    output cap and the layer outside the region are what remain."""

    centre: tuple
    length_m: float
    width_m: float
    axis_yaw: float
    valid: bool = True
    bypass: bool = True
    invalid_reason: str | None = None
    last_new_s: float = -math.inf

    def region_control(self):
        return (float(self.centre[0]), float(self.centre[1]), float(self.length_m), float(self.width_m),
                float(self.axis_yaw))

    def limit(self, *a, **k):
        return math.inf, "ok"

    def add_frame(self, *a, **k):
        return {"new": False}

    def summary(self):
        return {"bypass": True, "region": list(self.region_control())}

def bypass_region(pose, depth_m: float, width_m: float) -> list:
    """The D5 bypass region of a registered pallet pose (x, y, yaw): [x, y, length, width, yaw] -- the pallet grown by
    the bypass bands, about its own axis (case A: the approach axis is the pallet's, plan 2.4)."""
    x, y, yaw = (float(v) for v in pose)
    return [x, y, float(depth_m) + 2 * BYPASS_BAND_ALONG_M, float(width_m) + 2 * BYPASS_BAND_ACROSS_M, yaw]


REGION_YAW_TOL_RAD = math.radians(5.0)


def bypass_region_matches(applied, confirmed) -> bool:
    """The region applied (``region_control()``: centre, length, width, yaw) is the confirmed one: the same centre
    and size (they come from the same registration) and an axis within the case-A tolerance."""
    a, c = [float(v) for v in applied], [float(v) for v in confirmed]
    if len(a) != 5 or len(c) != 5:
        return False
    same = all(abs(a[i] - c[i]) <= 1e-6 for i in range(4))
    return same and abs(math.remainder(a[4] - c[4], math.tau)) <= REGION_YAW_TOL_RAD


# Optional hooks those paths try and the real truck leaves absent (the core falls back to its default).
ADOPTION_OPTIONAL_ABSENT = frozenset({"exit_prefix", "far_recognition_mission_end"})


class RealRequireFailed(RuntimeError):
    """A core ``require`` that failed: the control loop ends the mission (4c), never a silent continue."""


class RealHooks:
    """The real truck's hooks for the preparation and the adoption (design 4). Sources are injected: ``lift_m()``
    the port's measured lift, ``odom_base()`` the SLAM link's odometry base, ``ready(t, inputs)`` the adoption's last
    check (fresh obstacle evidence -- raises to fail the preparation), ``started(t, inputs, plan, ready)`` the
    executor's own start, ``record(name, obj)`` the run folder."""

    def __init__(self, *, lift_m, odom_base, ready, started, record, geometry, occupancy=None,
                 pocket_frame=None, pocket_check=None, operator=None, tick=None, layer=None,
                 confirmed_region=None) -> None:
        self._confirmed_region = confirmed_region  # () -> the region this mission's D5 confirmation named (U3b-2)
        self._lift_m, self._odom_base, self._ready, self._started = lift_m, odom_base, ready, started
        self._record, self._occupancy, self._geometry = record, occupancy, geometry
        # PR 7 supplies the depth frame and the check (until then: no frame -- 4a's wait ends the mission); the
        # console supplies the operator's answers (PR 5)
        self._pocket_frame, self._pocket_check, self._operator = pocket_frame, pocket_check, operator
        # the control tick's sources (docs/design/2026-11-19-real-loop.md): slam_rear(t), speed() -> SafetySpeed,
        # steering() -> (q,), kinematics, loaded(), cusp_holds / cusp_monitor objects, now_s(); None: not assembled
        self._tick = tick
        self._rear = None
        # 5a (real safety wiring design 1.3): the real obstacle layer the D5 check's region is installed on as its
        # exemption, as the Isaac hook does; None: as before (the check only)
        self._layer = layer

    # --- the control tick (4c) -------------------------------------------------------------------------------------
    def _src(self, name):
        if self._tick is None or getattr(self._tick, name, None) is None:
            raise RealRequireFailed(f"tick_source_missing:{name}")
        return getattr(self._tick, name)

    def rear(self):
        return self._rear

    def set_rear(self, rear):
        self._rear = np.asarray(rear, dtype=float)

    def slam_rear(self, t):
        return np.asarray(self._src("slam_rear")(t), dtype=float)

    def safety_speed(self):
        return self._src("speed")()

    def signed_speed(self):
        return float(self._src("speed")().signed_mps)

    def steering_position(self):
        return tuple(self._src("steering")())

    def held_kappa(self):
        return float(self._src("kinematics").held_curvature(tuple(self._src("steering")())) or 0.0)

    def loaded(self):
        return bool(self._src("loaded")())

    def cusp_holds(self):
        return self._src("cusp_holds")

    def cusp_monitor(self):
        return self._src("cusp_monitor")

    def cusp_standing(self):
        return bool(self._src("standing")())

    def cusp_heading_motion(self, signed_speed_now):
        return {"control_speed_mps": float(signed_speed_now), "measured_speed_mps": self.signed_speed(),
                "truth_rear": None, "steering_command_rad": None, "steering_actual_rad": list(self.steering_position())}

    def plan_transport_leg(self, *a, **k):
        from forklift_core.planning import pallet_mission as pm

        return pm.plan_transport_leg(*a, **k)

    def plan_return_leg(self, *a, **k):
        from forklift_core.planning import pallet_mission as pm

        return pm.plan_return_leg(*a, **k)

    def plan_observation_leg(self, *a, **k):
        from forklift_core.planning import pallet_mission as pm

        return pm.plan_observation_leg(*a, **k)

    def dump_tracking(self, reason, tracking):
        self._record("tracking_dump", {"reason": reason})

    def path_stats(self, path):
        return {"length_m": float(getattr(path, "length_m", 0.0) or 0.0)}

    def save_npz(self, name, **arrays):
        self._record("npz", name)

    def add_path_display(self, *a):
        pass  # display only

    def now_s(self):
        return float(self._src("now_s")())

    def require(self, ok, reason):
        if not ok:
            raise RealRequireFailed(str(reason))

    def wall(self):
        return time.monotonic()

    def lift_position(self):
        return float(self._lift_m())

    def slam_odom_base(self):
        return self._odom_base()

    def grid_world(self, scenario):
        return replace(scenario, props=())  # a grid plan sees no prior props (the plan audit table)

    def grid_kwargs(self, kind=None, target=None, *args, **kwargs) -> dict:
        """Side-effect free at a preparation (design 4; Codex 4a P3): the layer's occupancy as it is and, for a
        mission plan, the target pallet as the approach's obstacle -- the operator's registered pose, not the
        scenario's reference one (Codex 4b P2) -- never a write to the core's pickup estimate."""
        from forklift_core.planning.geometry import Rectangle

        occupancy = None if self._occupancy is None else self._occupancy()
        out = {} if occupancy is None else {"occupancy": occupancy}
        if kind == "mission" and target is not None:
            g = self._geometry
            out["pickup_obstacle"] = Rectangle(float(target.x_m), float(target.y_m), g.pallet_depth_m,
                                               g.pallet_width_m, float(target.yaw_rad))
            if occupancy is not None:
                # the docking straights see the grid without the target pallet's own cells (the insert goes into
                # it, Codex 4b P2) -- the approach keeps the shared grid and the obstacle
                out["docking_occupancy"] = docking_occupancy(occupancy, target, g)
        return out

    def known_pocket_frame(self):
        return None if self._pocket_frame is None else self._pocket_frame()

    def build_pocket_check(self, axis_yaw, rear, base, stamp_s=None, rear_at_stamp=None):
        if self._pocket_check is None:
            raise RealRequireFailed("pocket_check_unavailable")  # PR 7: the D435i depth path
        check = self._pocket_check(axis_yaw, rear, base, stamp_s, rear_at_stamp)
        if self._layer is not None:
            # the Isaac hook's rule: only a valid check gates, and its region is the layer's exemption
            if check is None or not getattr(check, "valid", False):
                raise RealRequireFailed(f"pocket_check_refused:{getattr(check, 'invalid_reason', 'no_check')}")
            self._layer.exempt = check.region_control()

    def bypass_region(self, pose) -> list:
        """The bypass region of a pallet pose on this assembly's geometry -- the one source the D5 confirmation's
        region and the check built for the mission both use (U3b-2)."""
        g = self._geometry
        return bypass_region(pose, float(g.pallet_depth_m), float(g.pallet_width_m))

    def build_bypass_pocket_check(self, axis_yaw, pickup):
        """5d (design 7.2): the bypass check for the submitted pickup -- built only, no side effect (the core
        registers it, then sets the exemption). On the operating run (``confirmed_region``) the region applied must be
        the one the operator confirmed on screen for this mission, from the same registration (plan 2.6, U3b-2)."""
        x, y, length, width, _ = self.bypass_region((pickup.x_m, pickup.y_m, axis_yaw))
        check = BypassPocketCheck((x, y), length, width, float(axis_yaw))
        if self._confirmed_region is not None:
            confirmed = self._confirmed_region()
            if confirmed is None:
                raise RealRequireFailed("d5_confirmation_missing")
            if not bypass_region_matches(check.region_control(), confirmed):
                raise RealRequireFailed("d5_region_mismatch")
            # the region exempted is the confirmed rectangle itself, not one turned within the axis tolerance
            # (Codex U3b-2 round 1 P3-1: a turned rectangle frees cells outside what the eyes cleared)
            cx, cy, clen, cwid, cyaw = (float(v) for v in confirmed)
            check = BypassPocketCheck((cx, cy), clen, cwid, cyaw)
        return check

    def operator_check(self, request):
        return None if self._operator is None else self._operator(request)

    def live_mission_ready(self, t, inputs):
        return self._ready(t, inputs)

    def live_mission_start(self, t, inputs, plan, ready):
        self._started(t, inputs, plan, ready)

    def path_record(self, path):
        return {"length_m": float(getattr(path, "length_m", 0.0)), "poses": len(getattr(path, "poses", ()))}

    def write_paths(self):
        pass  # the paths go into the record with the state (4c's record)

    def record_json(self, obj):
        return json.dumps(obj, default=str)

    def echo(self, line):
        self._record("echo", line)

    def show_path(self, *a):
        pass  # display only

    def snapshot(self, phase):
        pass  # display only


# Only cells whose centre lies inside the registered footprint are taken as the pallet's own: a cell outside it cannot
# be told from a separate obstacle (Codex 4b round 4), so it stays -- a pallet placed off its registered pose blocks
# the docking plan instead (the safe failure: the operator measures and registers it again).
DOCKING_CLEAR_MARGIN_M = 0.0


def docking_occupancy(occupancy, target, geometry, margin_m: float = DOCKING_CLEAR_MARGIN_M):
    """``occupancy`` without the target pallet's own cells -- its footprint at the registered pose grown by
    ``margin_m`` -- so the insert can enter it; anything else in front of the face stays (Codex 4b round 3: Isaac's
    whole 0.425 m docking band also cleared a separate obstacle there)."""
    import math

    import numpy as np

    from forklift_core.planning.grid_collision import OccupancyGrid

    occ = np.array(occupancy.occupied, dtype=bool, copy=True)
    nx, ny = occ.shape
    gi, gj = np.meshgrid(np.arange(nx), np.arange(ny), indexing="ij")
    dx = occupancy.origin_x_m + (gi + 0.5) * occupancy.resolution_m - float(target.x_m)
    dy = occupancy.origin_y_m + (gj + 0.5) * occupancy.resolution_m - float(target.y_m)
    c, s = math.cos(float(target.yaw_rad)), math.sin(float(target.yaw_rad))
    own = (np.abs(dx * c + dy * s) <= geometry.pallet_depth_m / 2 + margin_m) & (
        np.abs(-dx * s + dy * c) <= geometry.pallet_width_m / 2 + margin_m)
    occ[own] = False
    return OccupancyGrid(occupancy.origin_x_m, occupancy.origin_y_m, occupancy.resolution_m, occ,
                         getattr(occupancy, "version", 0))


class AssemblyRefused(RuntimeError):
    def __init__(self, reasons):
        super().__init__("; ".join(reasons))
        self.reasons = list(reasons)


@dataclass(frozen=True)
class Watches:
    """The real autonomy's watches (safety wiring design 4): the current and wheel SafetyWatch, the IMU watch (None
    with ``imu: none``) and the policy the loop enforces (``required``: no loop without the IMU watch and source)."""

    safety: object
    imu: object
    imu_policy: str


def build_watches(autonomy: dict, watch_path, wheel_radius_m) -> Watches:
    """The one assembly of the watches for the dry and the live assembly (design 4): ``config/real/safety_watch.yaml``
    (``SafetyWatch`` and its ``imu:`` part) and ``autonomy.yaml``'s ``imu: required | none``; any fault refuses."""
    import yaml

    from forklift_core.safety.real_watch import ImuWatch, SafetyWatch, load_imu_config, load_watch_config

    try:
        safety = SafetyWatch(load_watch_config(watch_path), wheel_radius_m=tuple(float(v) for v in wheel_radius_m))
    except (OSError, ValueError, KeyError, TypeError, AttributeError, yaml.YAMLError) as err:
        raise AssemblyRefused([f"watch_config:{type(err).__name__}:{err}"]) from err
    policy = autonomy.get("imu") if isinstance(autonomy, dict) else None
    if policy not in ("required", "none"):
        raise AssemblyRefused([f"imu_policy:{policy!r}"])
    imu = None
    if policy == "required":
        try:
            imu = ImuWatch(load_imu_config(watch_path))
        except (OSError, ValueError, KeyError, TypeError, AttributeError, yaml.YAMLError) as err:
            msg = str(err)
            raise AssemblyRefused([msg if msg.startswith("imu_config:") else f"imu_config:{msg}"]) from err
    return Watches(safety, imu, policy)


def scenario_of(cfg: ra.S10AConfig) -> TransportScenario:
    x0, x1, y0, y1 = (float(v) for v in cfg.bounds)
    pickup = PalletSite(*(float(v) for v in cfg.check_pallet_pose))
    return TransportScenario(seed=0, start_rear=Pose2D(*(float(v) for v in cfg.start_rear)), pickup=pickup,
                             destination=PalletSite(*(float(v) for v in cfg.destination)), props=(),
                             bounds=Bounds(x0, x1, y0, y1))


def assemble(*, cfg: ra.S10AConfig, loaded_map_id, geometry, pallet_geometry, kinematics, settings: dict,
             planner_config, travel_config, speeds: dict, mission_stages, curvatures: dict, layer, applied,
             slam_tracker, jobs, hooks: RealHooks, extra_options: dict | None = None) -> MissionCore:
    """The assembled core (design 4b 4-5), or ``AssemblyRefused`` with every structural reason."""
    refusals = ra.s10a_refusal(cfg, loaded_map_id)
    if cfg.check_pallet_pose is None:
        refusals.append("check_pallet_pose_unset")
    for key in ("drive_acceleration_mps2", "lift_rate_mps", "lift_target_m", "steering_command_rate_rad_s",
                "withdraw_speed_mps", "tracker_curvature_inv_m"):
        if key not in settings:
            refusals.append(f"setting_missing:{key}")
    if refusals:
        raise AssemblyRefused(refusals)
    scenario = scenario_of(cfg)
    options = ra.real_options(kinematics=kinematics, planner_config=planner_config, travel_config=travel_config,
                              scenario=scenario, speeds=speeds, mission_stages=mission_stages,
                              **(extra_options or {}))
    refusals = ra.real_option_refusal(options, tracker_curvature_inv_m=settings["tracker_curvature_inv_m"],
                                      preview_curvature_inv_m=curvatures.get("preview"),
                                      max_speed_mps=cfg.max_speed_mps)
    if refusals:
        raise AssemblyRefused(refusals)
    state = {"transitions": [], "planning_wall_s": 0.0, "paths": {}}
    pocket = {"check": None, "frames_read": 0}
    obstacle = {"layer": layer, "applied": tuple(float(v) for v in applied), "version": 0, "ticks": {},
                "min_allowed": {}, "slowed": {}, "reasons": {}}
    slam = {"tracker": slam_tracker, "records": [], "docking": {}, "stop_now": False, "holds": [], "control": []}
    core = MissionCore(MissionState(phase="idle", phase_started=0.0), state=state, obstacle=obstacle, slam=slam,
                       pocket=pocket, paths={}, trackers={}, geometry=geometry, settings=dict(settings),
                       pallet_geometry=pallet_geometry, hooks=fail_closed_hooks(hooks), near=None, options=options)
    tb = core.targets
    tb.add_shape(cfg.pallet_kind, cfg.pallet_depth_m, cfg.pallet_width_m)
    tb.add_destination(cfg.destination_id, TargetPose(*(float(v) for v in cfg.destination)))
    # registered unlocated with no pose: only the operator's map pose (operator_input) makes it a pick (design 2)
    tb.add_pallet(cfg.pallet_id, cfg.pallet_kind, TargetPose(*(float(v) for v in cfg.check_pallet_pose)))
    tb.unlocate(cfg.pallet_id)
    tb.clear_unlocated(cfg.pallet_id)
    core.jobs = jobs
    core.baseline = MissionBaseline.capture(core)
    return core


@dataclass(frozen=True)
class FrozenSubmit:
    """What an approved S10 submit fixes (design 2), taken from the accepted mission itself: its id, pallet,
    destination and revision, the pallet's registered pose and the destination's pose, used for the request and every
    re-request."""

    mission_id: str
    pallet_id: str
    destination_id: str
    pose: tuple
    destination: tuple
    targets_revision: int


def freeze_submit(core: MissionCore) -> FrozenSubmit:
    """At the approval (the commands' accepted mission): refused when the list moved since that submit's revision --
    the pose it was approved on is no longer the one registered."""
    cmds, tb = core.commands, core.targets
    mission = cmds.mission
    if mission is None or cmds.status not in ("accepted", "preparing"):
        raise ValueError("no_accepted_mission")
    if tb.revision != mission.targets_revision:
        raise ValueError("targets_changed")
    why = s10_submit_refusal(core, mission.pallet_id)
    if why is not None:
        raise ValueError(why)
    p, d = tb.pallets[mission.pallet_id].pose, tb.destinations[mission.destination_id]
    return FrozenSubmit(cmds.mission_id, mission.pallet_id, mission.destination_id,
                        (float(p.x_m), float(p.y_m), float(p.yaw_rad)),
                        (float(d.x_m), float(d.y_m), float(d.yaw_rad)), int(mission.targets_revision))


def known_request(core: MissionCore, t: float, rear, frozen: FrozenSubmit, *, base_scenario, wait_pose,
                  pickup_bounds=None, pickup_zone=None):
    """The preparation's request (and re-request) on the frozen submit: the mission's inputs built from it (the
    planning destination is the one the submit approved); a frozen submit that is not the accepted mission's (another
    mission, pallet, destination or revision) ends the preparation instead."""
    cmds = core.commands
    mission = cmds.mission
    if mission is None or (cmds.mission_id, mission.pallet_id, mission.destination_id, mission.targets_revision) != (
            frozen.mission_id, frozen.pallet_id, frozen.destination_id, frozen.targets_revision):
        if cmds.status in ("accepted", "preparing"):
            cmds.terminate("failed", t, "prepare:known:submit_mismatch")
        return "submit_mismatch"
    scenario = replace(base_scenario, destination=PalletSite(*frozen.destination))
    inputs = {"scenario": scenario, "wait_pose": wait_pose, "pickup_bounds": pickup_bounds, "pickup_zone": pickup_zone}
    return core.known_pallet_request(t, rear, inputs, frozen.pose, targets_revision=frozen.targets_revision)


class OperatorStart:
    """The operator's physical check before a submit (design 2, 13): empty forks, out of any pocket, the lift down.
    Bound to the session, the lease that gave it and the next submit only; any motion, a lease change, a link loss or
    a field recovery voids it."""

    def __init__(self, session_epoch: int) -> None:
        self.session_epoch = int(session_epoch)
        self._given: tuple | None = None
        self.voided: str | None = None
        # the operating run's D5 confirmation (plan 2.6, unlock U3b-2): what the operator saw and confirmed on screen
        # -- the pallet, its registration revision, the relocalization generation and the whole bypass region --
        # and, once a submit used it, that binding with its mission id (the record and the region's check)
        self.binding: dict | None = None
        self.used: dict | None = None

    def confirm(self, session_epoch: int, lease_id: str, binding: dict | None = None) -> bool:
        if int(session_epoch) != self.session_epoch:
            return False
        self._given, self.voided = (int(session_epoch), str(lease_id)), None
        self.binding = None if binding is None else dict(binding)
        return True

    def void(self, reason: str) -> None:
        if self._given is not None:
            self._given, self.voided, self.binding = None, reason, None

    def ok(self, lease_id: str | None) -> bool:
        return self._given is not None and lease_id is not None and self._given[1] == str(lease_id)

    def ok_for(self, lease_id: str | None, key: tuple) -> bool:
        """Given by this lease for exactly (pallet id, registration revision, relocalization generation)."""
        b = self.binding
        return self.ok(lease_id) and b is not None and (
            b.get("pallet_id"), b.get("targets_revision"), b.get("reloc_gen")) == tuple(key)

    def take(self, lease_id: str | None, key: tuple | None = None, mission_id=None) -> bool:
        """A submit uses it up (one submit only); with ``key`` only a confirmation of that registration does."""
        if not self.ok(lease_id) or (key is not None and not self.ok_for(lease_id, key)):
            return False
        self.used = None if self.binding is None else {**self.binding, "mission_id": mission_id}
        self._given, self.voided, self.binding = None, "used", None
        return True


def register_pallet(core: MissionCore, pallet_id: str, pose) -> int:
    """The operator's map pose for the next submit (design 2): the pallet becomes a pick; returns the revision the
    submit names."""
    core.targets.update_pose(pallet_id, TargetPose(*(float(v) for v in pose)), basis="operator_input",
                             uncertainty_m=None)
    return core.targets.revision


def s10_submit_refusal(core: MissionCore, pallet_id: str) -> str | None:
    """A submit names a pallet whose pose the operator registered (design 2); None: it may."""
    pallet = core.targets.pallets.get(pallet_id)
    if pallet is None or pallet.basis != "operator_input" or pallet.pose is None:
        return "pallet_not_registered"
    return None


def assembly_record(core: MissionCore, cfg: ra.S10AConfig, curvatures: dict) -> dict:
    """The dry assembly's record (design 9): the options' values, the hook table, the settings and the caps."""
    opts = vars(core.options)
    kin = opts["kinematics"]
    low, high = kin.cfg.steering.curvature_range
    return {"options": {k: (v if isinstance(v, (bool, int, float, str, type(None), tuple, list, dict)) else repr(v))
                        for k, v in sorted(opts.items())},
            "adoption_hooks": sorted(ADOPTION_HOOKS), "settings": dict(core.settings),
            "curvature_caps": {"planner": opts["planner_config"].curvature_limit_inv_m,
                               "travel": opts["travel_config"].curvature_limit_inv_m,
                               "tracker": core.settings.get("tracker_curvature_inv_m"), **dict(curvatures)},
            "steering_curvature_range": [low, high],
            "steering_rate_rad_s": kin.steering_rate_rad_s, "s10a": {k: v for k, v in vars(cfg).items()}}


__all__ = ["TICK_HOOKS", "DOCKING_CLEAR_MARGIN_M", "docking_occupancy", "ADOPTION_HOOKS", "ADOPTION_OPTIONAL_ABSENT", "FrozenSubmit", "OperatorStart", "STAND_AND_END_HOOKS",
           "freeze_submit", "known_request", "AssemblyRefused", "RealHooks", "RealRequireFailed", "assemble", "assembly_record",
           "register_pallet", "s10_submit_refusal", "scenario_of"]
