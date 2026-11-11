"""The mission core's hook contract (sim-to-real 4d-2a, docs/design/2026-11-11-p05-truth-audit.md 4-2).

Every hook name the core reads is registered here with its class. The real entry point wraps its hook object with
``fail_closed_hooks``: an audited name passes, an evaluation name answers None without reading the real object, and
everything else -- truth, a human judgement still made from truth, a test tool, a simulation-only object, or any name
not registered at all -- raises ``TruthAccessViolation``. A test scans the core's source so a new hook cannot come in
without being classified.
"""

from __future__ import annotations

from enum import Enum

from forklift_core.safety.truth_access import TruthAccessViolation


class HookClass(str, Enum):
    CONTROL = "control"  # measurement, planning, command: the real truck's own implementation
    RECORD = "record"  # records and displays: the real truck's files and console
    CLOCK = "clock"  # time: the real truck's clocks (world.current_time is replaced in 4e-1)
    EVAL = "eval"  # evaluation against truth: None on the real truck ("unjudgeable")
    HUMAN = "human"  # a judgement made from truth today; an operator check in 4d-2b
    FALLBACK = "fallback"  # the SLAM-off truth path: never assembled on the real truck
    TEST = "test"  # test tools: absent on the real truck
    SIM_ONLY = "sim_only"  # simulation objects: not read on the real truck


_C, _R, _K, _E, _H, _F, _T, _S = (HookClass.CONTROL, HookClass.RECORD, HookClass.CLOCK, HookClass.EVAL,
                                  HookClass.HUMAN, HookClass.FALLBACK, HookClass.TEST, HookClass.SIM_ONLY)

# name -> (class, optional). Optional hooks are read with getattr(hooks, name, default): only those may be absent.
# Mixed hooks (a measurement record with truth fields) are CONTROL whose real implementation fills the truth fields
# with None in the same shape: cusp_heading_motion (the core spreads it into a record) and the far_recognition*
# adapters (4e-2 builds a truth-free one).
HOOKS: dict[str, tuple[HookClass, bool]] = {  # noqa: RUF012
    # control: pose, motion, actuators
    "require": (_C, False), "rear": (_C, False), "set_rear": (_C, False), "slam_rear": (_C, False),
    "slam_odom_base": (_C, False), "lift_position": (_C, False), "steering_position": (_C, False),
    "steering_command": (_C, False), "steering_actual": (_C, False), "safety_speed": (_C, True),
    "signed_speed": (_C, False), "held_kappa": (_C, False), "loaded": (_C, False), "loaded_cargo_ok": (_C, False),
    # control: planning
    "bicycle_rollout": (_C, False), "plan_observation_leg": (_C, False), "plan_transport_leg": (_C, False),
    "plan_return_leg": (_C, False), "plan_transport": (_C, False), "plan_docking_reapproach": (_C, False),
    "final_straight_prefix": (_C, False), "straight_from_pose": (_C, False), "same_path": (_C, False),
    "grid_kwargs": (_C, False), "grid_world": (_C, False), "correction_grid": (_C, False),
    "redraw_docking": (_C, False), "redraw_docking_traced": (_C, False), "dock": (_C, False),
    "exit_prefix": (_C, True), "session_neighbour_rects": (_C, False), "unaligned_delivery_refusal": (_C, True),
    "prepare_start_ok": (_C, True), "planning_target": (_C, False),
    # control: sensing and recognition
    "beam_angles": (_C, False), "laser_mount": (_C, False), "fork_blades": (_C, False),
    "near_fork_geometry": (_C, False), "near_tracker_config": (_C, False), "near_tracker_create": (_C, False),
    "tracking_frame": (_C, False), "detect_pockets": (_C, False), "detect_pockets_expected": (_C, False),
    "build_pocket_check": (_C, False), "perception_capture": (_C, False), "trace_ground": (_C, True),
    "far_recognition": (_C, True), "far_recognition_cancel": (_C, True), "far_recognition_mission_end": (_C, True),
    "far_recognition_poll": (_C, False), "far_recognition_submit": (_C, True), "far_recognition_zone": (_C, True),
    # control: cusp and live
    "cusp_holds": (_C, False), "cusp_monitor": (_C, False), "cusp_standing": (_C, False),
    "cusp_heading_motion": (_C, False), "live_mission_ready": (_C, False), "live_mission_start": (_C, False),
    "live_abort": (_C, True),
    "operator_check": (_C, False),  # the operator's answer to a confirmation request (4d-2b-1; the console in 4e-2)
    # records and displays
    "path_record": (_R, False), "write_paths": (_R, False), "record_json": (_R, False), "save_npz": (_R, False),
    "save_capture": (_R, False), "dump_tracking": (_R, False), "path_stats": (_R, False), "echo": (_R, False),
    "show_path": (_R, False), "add_path_display": (_R, False), "set_inset": (_R, False),
    "set_inset_estimate": (_R, False), "set_inset_status": (_R, False), "snapshot": (_R, False),
    # clocks
    "now_s": (_K, False), "wall": (_K, False), "time_module": (_K, False), "world": (_K, False),
    # evaluation
    "truth_rear": (_E, True), "truth_relative_yaw": (_E, False), "slam_error": (_E, False),
    "insertion_measured": (_E, False), "record_capture_evaluation": (_E, False), "adoption_truth": (_E, True),
    "truth_obstacles": (_E, False),  # conditional control: the real truck plans on the grid (core refuses otherwise)
    # human (4d-2b), fallback, tests, simulation objects
    "pallet_z": (_H, False),
    "truth_speed": (_F, True), "truth_velocity": (_F, False), "truth_yaw_rate": (_F, False),
    "test_recovery_blocked": (_T, True), "test_return_correction_drops": (_T, True),
    "stage": (_S, False), "export_scene": (_S, False),
}

_PASS = (HookClass.CONTROL, HookClass.RECORD, HookClass.CLOCK)
_FORBIDDEN = (HookClass.HUMAN, HookClass.FALLBACK, HookClass.SIM_ONLY)


def _unjudgeable(*_args, **_kwargs):
    return None


class FailClosedHooks:
    """The real truck's hooks under the contract (4d-2a). Reads: an audited control, record or clock name from the
    wrapped object (an absent optional one is absent here too; an absent required one is a violation); an evaluation
    name answers None and never reads the wrapped object; a test tool is absent; anything else raises
    ``TruthAccessViolation``. Writes go to the wrapped object, registered names only."""

    __slots__ = ("_base",)

    def __init__(self, base):
        object.__setattr__(self, "_base", base)

    def __getattr__(self, name: str):
        spec = HOOKS.get(name)
        if spec is None:
            raise TruthAccessViolation(f"unregistered hook {name!r}")
        cls, optional = spec
        if cls is HookClass.EVAL:
            return _unjudgeable
        if cls is HookClass.TEST:
            raise AttributeError(name)  # absent: the core's getattr(..., None) takes its default
        if cls in _FORBIDDEN:
            raise TruthAccessViolation(f"{cls.value} hook {name!r}")
        base = object.__getattribute__(self, "_base")
        try:
            return getattr(base, name)
        except AttributeError:
            if optional:
                raise
            raise TruthAccessViolation(f"required hook {name!r} missing") from None

    def __setattr__(self, name: str, value) -> None:
        spec = HOOKS.get(name)
        if spec is None or spec[0] not in _PASS:
            raise TruthAccessViolation(f"hook {name!r} cannot be set on the real truck")
        setattr(object.__getattribute__(self, "_base"), name, value)


def fail_closed_hooks(base) -> FailClosedHooks:
    return FailClosedHooks(base)
