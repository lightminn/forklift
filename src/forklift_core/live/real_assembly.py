"""The real truck's mission core assembly (docs/design/2026-11-19-real-assembly.md, 4b sections 2, 4, 5).

``assemble`` builds the ``MissionCore`` behind ``fail_closed_hooks`` with the real options, the boot inputs whose
values and meaning the tests pin, the S10-A targets (the pallet registered unlocated: only the operator's map pose
makes it a pick), the worker and the baseline -- or refuses with the structural reasons. ``RealHooks`` implements the
hooks the preparation and the adoption read (the table ``ADOPTION_HOOKS``); the tick path's hooks join it in 4c.
No device is opened here.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, replace
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
                                 "lift_position"})

# Optional hooks those paths try and the real truck leaves absent (the core falls back to its default).
ADOPTION_OPTIONAL_ABSENT = frozenset({"exit_prefix"})


class RealRequireFailed(RuntimeError):
    """A core ``require`` that failed: the control loop ends the mission (4c), never a silent continue."""


class RealHooks:
    """The real truck's hooks for the preparation and the adoption (design 4). Sources are injected: ``lift_m()``
    the port's measured lift, ``odom_base()`` the SLAM link's odometry base, ``ready(t, inputs)`` the adoption's last
    check (fresh obstacle evidence -- raises to fail the preparation), ``started(t, inputs, plan, ready)`` the
    executor's own start, ``record(name, obj)`` the run folder."""

    def __init__(self, *, lift_m, odom_base, ready, started, record, geometry, occupancy=None,
                 pocket_frame=None, pocket_check=None, operator=None) -> None:
        self._lift_m, self._odom_base, self._ready, self._started = lift_m, odom_base, ready, started
        self._record, self._occupancy, self._geometry = record, occupancy, geometry
        # PR 7 supplies the depth frame and the check (until then: no frame -- 4a's wait ends the mission); the
        # console supplies the operator's answers (PR 5)
        self._pocket_frame, self._pocket_check, self._operator = pocket_frame, pocket_check, operator

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
        self._pocket_check(axis_yaw, rear, base, stamp_s, rear_at_stamp)

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


DOCKING_CLEAR_MARGIN_M = 0.10  # the registered pose's tape-measure error the pallet's own cells are taken within


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

    def confirm(self, session_epoch: int, lease_id: str) -> bool:
        if int(session_epoch) != self.session_epoch:
            return False
        self._given, self.voided = (int(session_epoch), str(lease_id)), None
        return True

    def void(self, reason: str) -> None:
        if self._given is not None:
            self._given, self.voided = None, reason

    def ok(self, lease_id: str | None) -> bool:
        return self._given is not None and lease_id is not None and self._given[1] == str(lease_id)

    def take(self, lease_id: str | None) -> bool:
        """A submit uses it up (one submit only)."""
        if not self.ok(lease_id):
            return False
        self._given, self.voided = None, "used"
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


__all__ = ["DOCKING_CLEAR_MARGIN_M", "docking_occupancy", "ADOPTION_HOOKS", "ADOPTION_OPTIONAL_ABSENT", "FrozenSubmit", "OperatorStart", "STAND_AND_END_HOOKS",
           "freeze_submit", "known_request", "AssemblyRefused", "RealHooks", "RealRequireFailed", "assemble", "assembly_record",
           "register_pallet", "s10_submit_refusal", "scenario_of"]
