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
from dataclasses import replace
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


# Optional hooks those paths try and the real truck leaves absent (the core falls back to its default).
ADOPTION_OPTIONAL_ABSENT = frozenset({"exit_prefix"})


class RealRequireFailed(RuntimeError):
    """A core ``require`` that failed: the control loop ends the mission (4c), never a silent continue."""


class RealHooks:
    """The real truck's hooks for the preparation and the adoption (design 4). Sources are injected: ``lift_m()``
    the port's measured lift, ``odom_base()`` the SLAM link's odometry base, ``ready(t, inputs)`` the adoption's last
    check (fresh obstacle evidence -- raises to fail the preparation), ``started(t, inputs, plan, ready)`` the
    executor's own start, ``record(name, obj)`` the run folder."""

    def __init__(self, *, lift_m, odom_base, ready, started, record, occupancy=None) -> None:
        self._lift_m, self._odom_base, self._ready, self._started = lift_m, odom_base, ready, started
        self._record, self._occupancy = record, occupancy

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
        """Side-effect free at a preparation (design 4; Codex 4a P3): the layer's occupancy as it is, never a write
        to the core's pickup estimate (the adoption sets it)."""
        return {} if self._occupancy is None else {"occupancy": self._occupancy()}

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
    refusals = ra.real_option_refusal(options, curvatures=curvatures, max_speed_mps=cfg.max_speed_mps)
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
    return {"options": {k: (v if isinstance(v, (bool, int, float, str, type(None))) else type(v).__name__)
                        for k, v in sorted(opts.items())},
            "adoption_hooks": sorted(ADOPTION_HOOKS), "settings": dict(core.settings),
            "curvature_caps": dict(curvatures), "steering_curvature_range": [low, high],
            "steering_rate_rad_s": kin.steering_rate_rad_s, "s10a": {k: v for k, v in vars(cfg).items()}}


__all__ = ["ADOPTION_HOOKS", "ADOPTION_OPTIONAL_ABSENT", "AssemblyRefused", "RealHooks", "RealRequireFailed", "assemble", "assembly_record",
           "register_pallet", "s10_submit_refusal", "scenario_of"]
