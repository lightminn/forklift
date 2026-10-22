"""The mission core (M-a, docs/plans/2026-10-15-m-a-mission-core.md), moved out of the Isaac runner one
slice at a time.

Slice 1: the phase transition and the known-pallet bookkeeping it carries (``transition`` /
``known_transition``, until 2026-10-15 closures in ``sim/isaac/run_transport.py``). The core shares the
runner's dictionaries (``state``, ``obstacle``, ``slam``, ``pocket``) and its ``paths`` / ``trackers``, and
reaches what has not moved yet through ``hooks``:

- ``echo(text)``, ``record_json(value)``: the runner's console line;
- ``snapshot(label)``: the video still at a transition (recording only);
- ``require(condition, reason)``: the runner's refusal (a RuntimeError);
- ``control_correction()``, ``known_pallet(rect, r_fix_m, event)``: obstacle-layer helpers;
- ``slam_odom_base()``, ``slam_error()``: the odometry pose for a hold, and the hold's error record
  (estimate minus truth -- recorded, not decided on);
- ``fork_blades()``: the fork blades from the chassis model, read when a withdrawal is certified.

Slice 2: the follower's time limit, the arrival transitions other than a perception observe, the lift target's
ramp (``MissionState.lift_command``) and the timed transitions (``tracking_limit`` / ``arrive`` /
``lift_and_timers``, until 2026-10-15 inline in the runner's loop). ``near`` is the runner's near-tracking
record (None without it); ``options`` carries ``near_tracking_stop_at``, ``d7_docking``, the ``dwells`` and
``safety_stops`` schedules and ``home_rear`` (the scenario's start). More hooks:

- ``insertion_measured()``, ``truth_relative_yaw()``, ``pallet_z()``, ``lift_position()``, ``truth_rear()``:
  simulation truth (the plan's ``SimTruth`` list) -- the loop's samples of this tick, read when called;
- ``dock_at_delivery_straight(t)``, ``slam_rear(t)``: the D7 docking at the delivery straight and the control
  pose it may have moved.

Slice 3a: the executed travel leg planned on the live grid when transport or return starts, and the stand while
a blocked-path replan waits (``live_leg``). Its planners and grid are ports (``plan_transport_leg``,
``plan_return_leg``, ``grid_world``, ``grid_kwargs``, ``return_turnaround_kwargs`` -- the recorded ones when the
boundary is recorded), the planning deadline's wall clock is ``wall()`` (the core never reads it itself, plan
D1), the path record and ``paths.json`` are ``path_record(path)`` / ``write_paths()``, and ``arm_docking()`` arms
the delivery docking. ``options`` adds ``grid_planning``, ``return_turnaround``, ``planner_config``,
``travel_config``, ``scenario`` and ``return_to_pose``.

Slice 3b: one tick of the obstacle permission while a path is followed (``permission``): re-projection after a
correction change, the permission's limit and its diagnostics dump (``save_npz(name, **arrays)``), the docking
straight and the pocket check's answer, the path-blocked count, the N9 silence stop, the speed cap and the
blocked/no-progress watch. Truth: ``truth_speed()``, ``truth_rear()``, ``truth_velocity()`` (the standing test
without SLAM), ``lift_position()`` (the pocket check's lift). ``options`` adds ``obstacle_act``, ``pocket_check``
and ``mid_still_decel_mps2``.

Slice 3c: stop, replan, resume once the truck has stood blocked or a replan is forced (``blocked_replan``),
statement for statement with the runner's names bound to hooks: the releases (``release_and_carry``,
``release_after_backoff``, ``slam_rear``), the planners (``plan_observation_leg``, ``plan_transport``,
``plan_return_leg``, ``transport_replan``, ``final_straight_prefix``), the recovery helpers (``find_escape``,
``escape_leg``, ``realign_active``, ``realign_install``, ``apply_tracker_profile``) and the runner's pure
``same_path`` / ``path_stats``. ``MissionState.next_candidate_index`` is the observation candidate cursor the
replan advances. ``options`` adds ``pickup_bounds``, ``mission_stages``, ``observation_waypoints`` and the
backoff/escape constants. The truth evaluation of the stopping volume stays in the runner (plan D3).

The bodies are the runner's, statement for statement: moving them changes no decision.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, replace
from types import SimpleNamespace

import numpy as np

from forklift_core.control import RearAxlePathTracker, TrackerConfig, ackermann_command
from forklift_core.control import cusp_stall as CUSP_STALL
from forklift_core.mission.baseline import start_digest
from forklift_core.mission.cargo import APPROACH_GAP_M, CargoState
from forklift_core.safety.pallet_permit import RECOGNISED_MARGIN_M
from forklift_core.mission.commands import MissionCommands
from forklift_core.control.speed_chain import creep, slew_cap_stop
from forklift_core.mission.targets import Pose as TargetPose
from forklift_core.mission.targets import TargetsBook
from forklift_core.mission.state import MissionState
from forklift_core.mission.scene_replan import SceneReplan
from forklift_core.mission import bottom_deck_legs
from forklift_core.perception import bottom_deck_evidence, bottom_deck_permit
from forklift_core.perception.bottom_deck_evidence import BottomDeckTarget
from forklift_core.perception.pocket_detector import DetectorParams, ExpectedSite
from forklift_core.localization.scan_docking import laser_points
from forklift_core.localization.slam_pose import compose, invert
from forklift_core.planning import PlanResult, Rectangle, collision_free_pose
from forklift_core.planning.pallet_mission import SearchBudget
from forklift_core.planning import Pose2D as PlanningPose

__all__ = ["MissionCore"]


def rollout_callback_spec(config, start) -> dict:
    """The request descriptor of a dry-run callback (``Trace.callback_spec``, M-a1 replay plan 3.4): the follower
    settings and the start pose it rolls a straight out with. redraw_docking's closures and
    redraw_docking_traced's dry configs describe themselves the same way."""
    return {"kind": "bicycle_rollout", "config": asdict(config), "start": [float(v) for v in np.ravel(start)]}


CLEAR_OUT_BUDGET_M = 1.0  # plan D4b: the most a cancelled truck backs out to where the next preparation can start
CLEAR_OUT_MARGIN_M = 0.08  # its end: clear with the arrival and tracking tolerances (0.03 + 0.008 + 0.02 rad x 1.66 m)
CLEAR_OUT_STEP_M = 0.05  # a final check that fails at the end backs out this much more (within the budget)
RECOVERY_PHASE_LIMIT_S = 20.0
MANUAL_GROUND_HOLD_S = 1.0  # plan D14: the manual ground verdict holds this long before the reverse opens
ALIGN_HEADING_RAD = 0.02  # plan D14: the straight reverse's heading tolerance to the pocket axis  # plan D4: each recovery phase's work-clock limit
RECOVERY_LOWER_MIN_S = 1.0  # the lowered verdict is read after this much of recover_lower (the lift ramps down first)


def slam_may_drive(slam: dict) -> bool:
    """SLAM lets the truck move: the tracker past warming up and, with the gyro heading, a standstill bias found
    (gyro-heading plan G2 (A) D2: no drive before the first bias, on every path)."""
    gyro = slam.get("gyro")
    return slam["tracker"].may_drive() and (gyro is None or gyro.get("bias") is not None)


@dataclass
class RecoveryPath:
    """A recovery's retrace (plan D4): rear-axle poses, all reversing."""

    poses: np.ndarray
    directions: np.ndarray
    curvatures_inv_m: np.ndarray
    length_m: float
    status: str = "recovery"
    success: bool = True
    expanded_nodes: int = 0  # the fields path_record reads (Codex 6b impl round 1 P2-7)


_HELD = object()  # a hold basis entry that no estimate is: every estimate counts as found under the hold


def apply_tracker_profile_to(trackers: dict, profile) -> None:
    """The ``--tracker-profile`` followers (M-a1 slice 6e: a module function, because the runner applies it
    to the first followers before the core exists and the core to the ones it installs later)."""
    # The 2026-09-21 G2 baseline had no cusp or overshoot tolerances and
    # stopped observation at 8 mm (git show 6f9fb82:sim/isaac/run_transport.py).
    if profile != "20260921":
        return
    for name, tracker in trackers.items():
        tracker.config = replace(
            tracker.config,
            cusp_position_tolerance_m=None,
            cusp_yaw_tolerance_rad=None,
            cusp_brake_window_m=None,
            overshoot_tolerance_m=None,
            position_tolerance_m=0.008,
            # Nor the cusp heading hold (fix A / A'): with no cusp tolerance it would hold
            # between 0.03 and 0.08 rad (Codex A' design review 1).
            cusp_heading_pending_rad=None,
        )
        if name == "observe":
            # It also judged observation stops at 0.03 rad (P3 is newer).
            tracker.config = replace(tracker.config, yaw_tolerance_rad=0.03)


def record_tracker_configs_in(state: dict, trackers: dict) -> None:
    """Every tracker's full settings as built (G2 rerun plan, 2026-10-01) -- state the near-field contract
    reads back (M-a1 slice 6e: a module function for the same reason)."""
    state.setdefault("tracker_configs", {}).update(
        {name: asdict(tracker.config) for name, tracker in trackers.items()}
    )


# Plan 2026-10-22 task A: the near capture's candidates must lie this close to the planned pickup. The far->near
# estimate shift over every ws1 record is at most 0.121 m (yaw 0.033 rad) and aisle_out's nearest straddling pattern
# sat 0.353 m from the pallet: 0.20 m keeps the pallet and leaves that pattern out at the largest shift seen.
NEAR_EXPECTED_RADIUS_M = 0.20
NEAR_EXPECTED_AXIS_RAD = math.radians(15.0)
# Plan 2026-10-22 task C (user decision 2026-10-22): the check-off near lower waiver needs the far detector's own pallet
# within this of the adopted far estimate, and the near candidate within this of the expected site.
NEAR_WAIVER_RADIUS_M = 0.15
NEAR_WAIVER_AXIS_RAD = math.radians(15.0)
# M-a3c (R11): a pause that moved the truck more than this from where it stopped (the docking straight was checked
# from there) -- a stop that holds stays well inside it (the stop detector's 12 mm/s mean over 0.2 s).
PAUSE_MOVED_M = 0.01
PAUSE_MOVED_RAD = 0.01
# M-a3 (E3): a stop not seen standing this long after it began is reported as not_stopped.
NOT_STOPPED_S = 10.0
# R7: a lift or lower resumes only with the lift near the target held since the stop. Provisional: a loaded lift
# sags about 2.8 mm below its target in Isaac (ma3_p_tr); real sensor noise is set at the vehicle readiness gate.
LIFT_RESUME_TOL_M = 0.01


class MissionCore:
    def __init__(self, ms: MissionState, *, state: dict, obstacle: dict | None, slam: dict | None, pocket: dict,
                 paths: dict, trackers: dict, geometry, settings: dict, pallet_geometry, hooks: SimpleNamespace,
                 near: dict | None = None, options: SimpleNamespace | None = None):
        self.ms = ms
        self.state = state
        self.obstacle = obstacle
        self.slam = slam
        self.pocket = pocket
        self.paths = paths
        self.trackers = trackers
        self.geometry = geometry
        self.settings = settings
        self.pallet_geometry = pallet_geometry
        self.hooks = hooks
        self.near = near
        self.options = options
        # M-a3: the mission lifecycle and its commands (docs/plans/2026-10-16-m-a3-mission-commands.md E1) --
        # outside every container the replay compares; a run without commands only ever submits once.
        self.targets = TargetsBook()  # M-a4: the pallets and destinations a submit names (plan D7)
        self.slam_releases = 0  # every SLAM release the core made (the targets list's correction version)
        self.commands = MissionCommands(self.targets)
        self.last_permission: tuple | None = None  # (allowed, why) of the latest permission or watch (E8)
        # Real-time plan D8 (docs/plans/2026-10-17-realtime-sim.md): a real-time run hands the long plans to a
        # worker (the runner sets ``jobs``); a batch run leaves it None and plans in the tick, as before.
        self.jobs = None
        self.pending: SimpleNamespace | None = None  # the plan being waited for
        self.release_replan_needed: dict | None = None  # a discarded release replan to ask again when running
        self.observe_failed_needed: dict | None = None  # a discarded re-observation plan (live) to ask again
        self.capture_live = None  # D9c: the capture running over control ticks
        self.observation_live = None  # D9c: a finished capture's observation, handled on the next tick
        self.far_leg_recaptures = 0  # a live far stop captured again after its leg ended (plan 2026-10-22 T1 (A) 3)
        self.capture_acquired_s: float | None = None  # D9c: its acquisition time, for the plan it asks
        self.prepare_attempts = 0  # persistent console plan D2: the current submit's preparation requests
        self.baseline = None  # persistent console plan D2: the mission-owned state before a live session's first mission
        self.record_start_digest = False  # use case U21 on Isaac: digest the mission-owned state at each start
        self.start_digest = None  # this mission's start digest (taken by the live end into the mission's records)
        self.end_release_due: dict | None = None  # plan D1 step 4: an end release waiting for the truck to stand
        self.hold_basis: dict | None = None  # the estimates a SLAM hold began with (those stay map estimates)
        self.fresh_depth_wait: tuple | None = None  # (since, acquisition stamp): an async near build's first frame
        # Persistent console plan D5a: the fork relation and the load, the session's (a batch run updates it too,
        # but only a live run reads it -- core.baseline is set there alone).
        self.cargo = CargoState()
        self._recovery_install_gen = None  # plan D4: the commands generation the retrace was installed under
        self.recent_odom: list = []  # plan D4b: the last 3 m of odometry rear poses (live), every 0.02 m
        self.recent_steps: list = []  # whether the step arriving at each of them went forward
        self.recent_forward = True  # the direction of the last motion
        self.clear_out_used_m = 0.0  # the back-out budget spent
        # Plan D14: the event hold tolerance a live runner gives (a callable: loaded or not, as of now); None in a
        # batch run, whose first pause keeps the measured latch the replay gate records.
        self.lift_hold_tolerance = None
        # Plan D11: the return turnaround's correction drops, per phase -- a mission's (the baseline restores it at each
        # mission's start; a pause or a resume keeps it, Codex impl round 1 P2), outside the shared live_leg drop
        # limit; at RETURN_CORRECTION_FALLBACK that return is planned in the tick as before.
        self.return_correction_drops: dict = {}
        # Plan D5 6c: a loaded mission's start wait (the mission's: the baseline clears it), and the loaded lift's
        # (sag, tolerance) a live runner gives -- the measured lift must be at the carry target less the sag, within
        # the tolerance, before the truck drives.
        self.loaded_wait: dict | None = None
        self.loaded_lift_band: tuple | None = None
        # The scene-mode replan device (docs/plans/2026-10-23-scene-replan.md): one request at a time ahead of the
        # follower; its records are the mission's (state["scene_replans"], made at the first request -- a run without
        # one keeps its state as it was), its budget reset at each mission's end.
        self.scene_replan = SceneReplan(None)
        self.scene_mail = None  # the device's plan answer: ("ok", result) | ("ok_fail", status) | ("dropped", why)
        self.scene_raised = False  # a request raised in this tick's follower: the rest of the tick stands
        self.scene_escape: dict | None = None  # the escape prefix driven for a request
        self.scene_correction: dict = {}  # the largest SLAM correction contribution on the current gear leg

    def known_transition(self, from_phase: str, next_phase: str, t: float) -> None:
        """Known pallets across the mission (plan v10 D5)."""
        from forklift_core.perception.known_obstacles import fork_pocket_gaps, invert_pose
        from forklift_core.perception.obstacle_grid import compose as grid_compose

        obstacle, state, hooks, geometry = self.obstacle, self.state, self.hooks, self.geometry
        layer_ = obstacle["layer"]
        rear_ = obstacle.get("control_rear")
        if next_phase == "lift":
            # The pallet is on the forks now (the loaded outline covers it). Keep
            # where perception put it relative to the truck: at the drop that is
            # the delivered pallet's pose (fork geometry, no new observation).
            est_ = obstacle.get("pickup_estimate")
            if est_ is not None and rear_ is not None:
                obstacle["pallet_rel"] = grid_compose(invert_pose(rear_), (est_.x_m, est_.y_m, est_.yaw_rad))
            layer_.known = []
            state.setdefault("known_pallets", []).append({"event": "lifted", "time_s": t})
        if next_phase == "withdraw" and layer_.known_config.get("withdraw_mode", "certified") != "certified":
            # Interim (user, 2026-10-08): the gate's b_w 12.9 mm + e_w 8.4 mm + the checker's
            # 18.5 mm side width exceed the 35.6 mm smallest blade-to-pocket gap measured (all at
            # 0.29 m fork spacing; re-measured before "certified" returns), so the certificate
            # cannot hold. The withdrawal runs as before v10 (no permission,
            # the planned straight); the delivered pallet joins the permission where the
            # withdrawal stops, re-fixed at b_w + e_w_full (the measured release error and
            # four-corner drift to the stop).
            if obstacle.get("pallet_rel") is not None and rear_ is not None:
                obstacle["delivered_pending"] = (grid_compose(rear_, obstacle["pallet_rel"]), self.control_correction(),
                                                 float(obstacle.get("last_stamp", t)))
            state.setdefault("known_pallets", []).append({"event": "withdraw_uncertified", "time_s": t})
        elif next_phase == "withdraw":
            # No recognised pallet or control pose: nothing to certify against -- refuse
            # rather than back out uncertified (Codex stage-1 P1-5).
            hooks.require(obstacle.get("pallet_rel") is not None and rear_ is not None,
                          "withdraw_not_certified:no_pallet_estimate")
            rel_ = obstacle["pallet_rel"]
            kp_ = layer_.known_config
            r_fix_ = kp_.get("b_w_m") if kp_.get("b_w_m") is not None else kp_.get("r_fix_pickup_m", 0.05)
            self.known_pallet(grid_compose(rear_, rel_), r_fix_, "delivered")
            blades_ = hooks.fork_blades()
            pg_ = self.pallet_geometry
            centre_ = sum(abs(y0 + y1) / 2 for _, _, y0, y1 in blades_) / len(blades_)
            half_ = sum((y1 - y0) / 2 for _, _, y0, y1 in blades_) / len(blades_)
            depth_ = geometry.axle_to_fork_tip_m + geometry.pallet_depth_m / 2 - geometry.inserted_offset_m
            gaps_ = fork_pocket_gaps(
                float(rel_[1]), float(rel_[2]), blade_centre_m=centre_, blade_half_width_m=half_,
                blade_length_m=depth_, pocket_inner_m=pg_.block_widths_m[1] / 2,
                pocket_outer_m=pg_.overall_width_m / 2 - pg_.block_widths_m[0],
            )
            until_ = depth_ + 0.10 + layer_.permission.config.stopping.distance_m(self.settings["withdraw_speed_mps"])
            # The certified straight starts where the truck stands, back along its own
            # heading: measured from the planned line, the insertion's few-mm end error
            # would already break the 6 mm tracking bound (plan v10 D5 (4)).
            paths, trackers = self.paths, self.trackers
            length_ = float(paths["withdraw"].length_m)
            count_ = max(2, int(math.ceil(length_ / 0.02)) + 1)
            s_ = np.linspace(0.0, length_, count_)
            straight_ = np.column_stack((rear_[0] - s_ * math.cos(rear_[2]), rear_[1] - s_ * math.sin(rear_[2]),
                                         np.full(count_, rear_[2])))
            paths["withdraw"] = replace(paths["withdraw"], poses=straight_,
                                        directions=np.full(count_, -1, dtype=np.int8),
                                        curvatures_inv_m=np.zeros(count_))
            trackers["withdraw"] = RearAxlePathTracker(
                straight_, paths["withdraw"].directions, paths["withdraw"].curvatures_inv_m, trackers["withdraw"].config
            )
            obstacle["withdraw_line"] = (tuple(rear_), length_)
            ok_ = layer_.certify_withdrawal(paths["withdraw"].poses, layer_.parts_shape, until_, gaps_)
            state["withdraw_certificate"] = {
                "time_s": t, "gaps_m": [float(g) for g in gaps_], "b_w_m": kp_.get("b_w_m"), "e_w_m": kp_.get("e_w_m"),
                "corridor_until_m": until_, "certified": ok_,
            }
            # Uncertified: the truck does not back out of the pallet (plan v10 D5 (3)).
            hooks.require(ok_, f"withdraw_not_certified:{state['withdraw_certificate']}")
        if from_phase == "withdraw":
            layer_.end_withdrawal(now_s=float(obstacle.get("last_stamp", t)))
            pending_ = obstacle.pop("delivered_pending", None)
            if pending_ is not None:
                from forklift_core.perception.known_obstacles import KnownRect

                kp_ = layer_.known_config
                (dx_, dy_, dyaw_), corr_, released_s_ = pending_
                if kp_.get("b_w_m") is not None and kp_.get("e_w_full_m") is not None:
                    # Re-fixed where the withdrawal stops: release error + measured four-corner drift.
                    full_ = float(kp_["b_w_m"]) + float(kp_["e_w_full_m"])
                    stamp_ = float(obstacle.get("last_stamp", t))
                else:
                    # Either bound missing (Codex stage-1 3rd P1): no re-fix -- the pallet keeps
                    # ageing from the release, with the release error alone.
                    full_ = float(kp_["b_w_m"]) if kp_.get("b_w_m") is not None else float(kp_.get("r_fix_pickup_m", 0.05))
                    stamp_ = released_s_
                layer_.known = [KnownRect((dx_, dy_, geometry.pallet_depth_m, geometry.pallet_width_m, dyaw_),
                                          fix_stamp_s=stamp_, r_fix_m=full_, rho_m=None, frame_correction=corr_)]
                state.setdefault("known_pallets", []).append(
                    {"event": "delivered_after_withdraw", "time_s": stamp_, "rect": [dx_, dy_, dyaw_], "r_fix_m": full_})

    def transition(self, next_phase: str, t: float) -> None:
        ms, state, obstacle, pocket, slam, hooks = self.ms, self.state, self.obstacle, self.pocket, self.slam, self.hooks
        state["transitions"].append({"from": ms.phase, "to": next_phase, "time_s": t})
        mission_ = self.commands.mission
        self.cargo.on_transition(ms.phase, next_phase, None if mission_ is None else mission_.pallet_id)
        if obstacle is not None and obstacle["layer"].known_enabled:
            self.known_transition(ms.phase, next_phase, t)
        hooks.echo("TRANSITION " + hooks.record_json(state["transitions"][-1]))
        hooks.snapshot(ms.phase)
        if next_phase == "insert" and pocket["check"] is not None:
            # D5: one frame taken standing at the approach end before the forks
            # go in. From there the strip before the face is in view; driving in,
            # the frames erode by the speed and, close up, the strip leaves the
            # camera's view (L3c v22 seed 2: four voxels 6.5 cm before the face).
            pocket["await_still_frame"] = True
        if ms.phase == "insert" and pocket["check"] is not None:
            # The insertion is over: the depth check stops answering and the
            # pallet region is no longer waived in the grid.
            obstacle["layer"].exempt = None
            obstacle["layer"].depth_support = None
            state.setdefault("pocket_check", {}).update(pocket["check"].summary(), frames_read=pocket["frames_read"])
            pocket["check"] = None
        if (
            next_phase == "extract" and obstacle is not None and obstacle["layer"].memory is not None
            and obstacle.get("pickup_estimate") is not None
        ):
            # The pallet is the truck's now: its remembered hits around the
            # recognised estimate go (D2/D3 delta; Codex review P1 -- they kept
            # the loaded start blocked). By the estimate, never the truth.
            est_ = obstacle["pickup_estimate"]
            mem_ = obstacle["layer"].memory
            # Endpoints only, within the estimate's error (0.05 m) and the range
            # noise bound (noise_cut_m): every endpoint the pallet can have goes
            # (Codex re-review 4 P2: a -0.059 m noisy one stayed and blocked the
            # loaded start); a box beyond the 0.12 m placement margin loses only
            # its endpoints with noise below -0.01 m and keeps the rest (Codex
            # review P1).
            grow_ = 0.05 + obstacle["layer"].noise_cut_m
            gone_ = mem_.retract_endpoints(est_.x_m, est_.y_m, self.geometry.pallet_depth_m,
                                           self.geometry.pallet_width_m, est_.yaw_rad, grow_)
            obstacle.setdefault("memory_events", []).append({"time_s": t, "event": "retract_pickup", "cells": gone_,
                                                              "grow_m": grow_})
        ms.phase, ms.phase_started = next_phase, t
        state["phase"] = ms.phase
        if slam is None:
            return
        # Docking phases freeze the applied correction (plan v3.1); the next
        # travel leg waits for a stop and the release below.
        # approach: the pallet estimate was made with the correction applied at
        # the accepted capture (this same instant -- the truck stands still for
        # the capture, so no keyframe lands in between). Keep that correction
        # until extract: the docking then runs on odometry relative to what was
        # seen, and a later SLAM correction cannot slide the truck against the
        # pallet estimate (plan v3.5; S2 seed 0 on v3.4 hit a block that way).
        if ms.phase in ("approach", "insert", "lower") and slam["tracker"].mode == "tracking":
            self.hold_slam(t, held_frame_pallet=True)  # the capture's estimate was made with this correction
            slam["holds"].append(
                {"phase": ms.phase, "time_s": t, "event": "hold", "error": hooks.slam_error()}
            )
        elif ms.phase in ("transport", "settle"):
            slam["pending_release"] = ms.phase
            slam["release_wait_from"] = t

    def _remaining_m(self, phase):
        """The follower's distance left in ``phase`` for the published state; None without a follower there."""
        remaining = getattr(self.trackers.get(phase) if phase is not None else None, "remaining_to_goal_m", None)
        return None if remaining is None else float(remaining())

    def tracking_limit(self, t: float) -> float:
        """The follower's time limit in the current phase: three times the path time at the tracker's own speed
        caps (length / cruise when there are none), plus 10 s; standing by design is not slowness."""
        ms, options = self.ms, self.options
        limit = max(30.0, 3 * self.trackers[ms.phase].nominal_duration_s() + 10)
        if options.dwells is not None and ms.phase == "approach":
            limit += options.dwells.held_s(t)  # plan D8b: standing by design is not slowness
        if options.safety_stops is not None and ms.phase == "approach":
            limit += options.safety_stops.held_s(t)  # plan D8 S4a-1 ⓪: the scheduled stops
        return limit

    def arrive(self, t: float, tracking) -> SimpleNamespace:
        """The follower arrived in a phase other than a perception observe (whose capture is still the
        runner's). ``stop`` ends the mission loop; ``rear`` is the control pose re-read when a D7 docking
        retry released the correction this tick (the runner's ``rear`` takes it), else None."""
        ms, state, slam, hooks, geometry, near = self.ms, self.state, self.slam, self.hooks, self.geometry, self.near
        transition = self.transition
        rear = None
        if ms.phase == "recover_withdraw":
            # plan D4: backed out; the live mission ends cancelled through the next start's check (D4b)
            self.finish_cancel(t, self.cargo.last_rear)
            return SimpleNamespace(stop=False, rear=rear)
        if ms.phase == "clear_out":
            self.clear_out_arrived(t, self.cargo.last_rear)  # plan D4b: the final check, or 0.05 m more
            return SimpleNamespace(stop=False, rear=rear)
        if ms.phase == "approach":
            if near is not None and self.options.near_tracking_stop_at == "insert":
                # Plan D8 S4a-1: approach only -- end where the insertion would start.
                state["near_tracking"]["outcome"] = "stopped_at_insert"
                state["near_tracking"]["arrival"] = {
                    "time_s": t,
                    "position_error_m": tracking.position_error_m,
                    "yaw_error_rad": tracking.yaw_error_rad,
                }
                transition("near_tracking_stopped", t)
                return SimpleNamespace(stop=True, rear=rear)
            if getattr(self.options, "bottom_deck_check", "off") == "on":
                # Plan 2026-10-19 task 3: the side evidence permits this insertion only when it read present and
                # was taken of the pallet the near capture now targets (the planned pickup: where the forks go).
                pickup_ = self.pocket["planning_pickup"]
                deck_ = state.setdefault("bottom_deck", {})
                shape_ = self.pallet_geometry.geometry_version
                at_ = {"target": [pickup_.x_m, pickup_.y_m, pickup_.yaw_rad], "shape": shape_}
                record_ = deck_.get("record")  # kept as JSON (BottomDeckRecord.to_record)
                refusal_ = bottom_deck_permit.insert_refusal(
                    None if record_ is None else bottom_deck_permit.BottomDeckRecord.from_record(record_),
                    (pickup_.x_m, pickup_.y_m), pickup_.yaw_rad, shape_)
                if refusal_ is not None:
                    deck_["refused"] = {"reason": refusal_, "time_s": t, **at_}
                    state["mission_result"] = "bottom_deck_refused"
                    transition("bottom_deck_refused", t)
                    return SimpleNamespace(stop=True, rear=rear)
                deck_["permitted"] = {"time_s": t, **at_}
            transition("insert", t)
        elif ms.phase == "insert":
            state["insertion_error"] = {
                "position_m": tracking.position_error_m,
                "yaw_rad": tracking.yaw_error_rad,
            }
            # What the forks actually reach, not the planned target.
            state["insertion_measured"] = hooks.insertion_measured()
            if near is not None:
                # Plan D8 S4a-2 ⑤: why the insertion ended; insertion_error keeps its
                # meaning (the follower against the path end).
                arrival_ = near.get("arrival") or {"reason": "path", "time_s": t}
                state["near_tracking"]["insert_arrival"] = {
                    **arrival_,
                    "last_tick": near["ticks"][-1] if near["ticks"] else None,
                    "insertion_measured": state["insertion_measured"],
                }
            # Insertion-axis yaw against the true pallet (fork axis
            # along the pallet x axis either way round).
            relative_yaw = hooks.truth_relative_yaw()
            state["insertion_truth_yaw_rad"] = float(
                math.atan2(math.sin(2 * relative_yaw), math.cos(2 * relative_yaw)) / 2
            )
            if slam is not None:
                state["slam_error_insert_end"] = hooks.slam_error()
            transition("lift", t)
        elif ms.phase == "extract":
            transition("transport", t)
        elif ms.phase == "transport":
            if slam is not None and slam["docking"]["status"] == "armed":
                self.dock_at_delivery_straight(t)
                if self.options.d7_docking:
                    # A (b) retry may have released the correction: this
                    # tick's pose and control record follow (Codex D7c impl P2-3).
                    rear = hooks.slam_rear(t)
                    hooks.set_rear(rear)  # the runner's closures (grid_kwargs, ...) read its rear (Codex slice-3 P2)
                    slam["control"][-1][1:4] = rear.tolist()
            else:
                transition("lower", t)
        elif ms.phase == "withdraw":
            transition("settle", t)
        elif ms.phase == "return_home":
            transition("home_settle", t)
        return SimpleNamespace(stop=False, rear=rear)

    def lift_and_timers(self, t: float, dt: float) -> bool:
        """The lift target's ramp and the timed transitions (lift, lower, settle, home settle). True ends the
        mission loop."""
        ms, state, settings, hooks, transition = self.ms, self.state, self.settings, self.hooks, self.transition
        desired_lift = (
            settings["lift_target_m"]
            if ms.phase in ["lift", "extract", "transport"]
            else 0.0
        )
        if self.baseline is None or self.commands.adopted_id is not None:
            # Plan D14: live, with no mission adopted, the core does not own the lift -- the ended mission's phase
            # ramped a lift lowered by hand back up (the manual lift and the hold own it then).
            ms.lift_command += float(
                np.clip(
                    desired_lift - ms.lift_command,
                    -settings["lift_rate_mps"] * dt,
                    settings["lift_rate_mps"] * dt,
                )
            )
        if ms.phase == "lift" and self.commands.work_elapsed(ms.phase_started, t) > 4.5:
            hooks.require(hooks.pallet_z() > 0.06, "Pallet was not lifted")
            transition("extract", t)
        if ms.phase == "lower" and self.commands.work_elapsed(ms.phase_started, t) > 4.5:
            hooks.require(
                abs(hooks.pallet_z()) < 0.008, "Pallet did not settle onto destination floor"
            )
            hooks.require(
                hooks.lift_position() < 0.012,
                "Fork did not lower",
            )
            transition("withdraw", t)
        if ms.phase in ("recover_lower", "recover_withdraw"):
            self.recovery_timers(t)
        if ms.phase == "settle" and self.commands.work_elapsed(ms.phase_started, t) > 1.0:
            if "return_home" in self.trackers:
                transition("return_home", t)
            else:
                transition("complete", t)
                return True
        if ms.phase == "home_settle" and self.commands.work_elapsed(ms.phase_started, t) > 1.0:
            truth_rear, home = hooks.truth_rear(), self.options.home_rear
            state["return_home_error"] = {
                "position_m": float(
                    np.hypot(
                        truth_rear[0] - home.x_m,
                        truth_rear[1] - home.y_m,
                    )
                ),
                "yaw_rad": float(
                    math.atan2(
                        math.sin(truth_rear[2] - home.yaw_rad),
                        math.cos(truth_rear[2] - home.yaw_rad),
                    )
                ),
            }
            transition("complete", t)
            return True
        return False

    def live_leg(self, t: float, rear, loaded) -> bool:
        """The executed travel leg planned on the live grid when transport or return starts (retried each
        second, five tries), and the stand while a blocked-path replan waits. True: stand this tick (the
        runner's requested speed becomes 0)."""
        ms, state, obstacle, slam, hooks, options = self.ms, self.state, self.obstacle, self.slam, self.hooks, self.options
        paths, trackers, geometry = self.paths, self.trackers, self.geometry
        planner_config, travel_config, scenario = options.planner_config, options.travel_config, options.scenario
        grid_world, grid_kwargs, return_turnaround_kwargs = hooks.grid_world, hooks.grid_kwargs, self.return_turnaround_kwargs
        stand = False
        if (
            options.grid_planning
            and ms.phase in ("transport", "return_home")
            and ms.phase in trackers
            and not obstacle.setdefault("live_replanned", {}).get(ms.phase)
            and t >= obstacle.get("live_retry_after_s", 0.0)
            # --return-turnaround: the first return plan waits for the SLAM release (settle may hand
            # over to return_home with it still pending), so the turnaround is weighed once, on the
            # released pose (Codex implementation review 2026-10-12, 2nd).
            and not (options.return_turnaround and ms.phase == "return_home" and slam is not None
                     and slam.get("pending_release") is not None)
        ):
            # The mission plan's travel legs saw the pickup pallet's band
            # cleared for the carried part; the executed leg is planned now,
            # on the live grid, where the carried pallet is the truck's own.
            obstacle["live_replanned"][ms.phase] = True
            if options.return_turnaround and ms.phase == "return_home" and slam is not None:
                # Standing, the obstacle block may not have re-projected the grid after the release:
                # do it here as it would, so the plan's grid, the arc check's snapshot and the start
                # pose share one correction.
                applied_now = (tuple(float(v) for v in slam["tracker"].applied[0])
                               if slam["tracker"].applied is not None else (0.0, 0.0, 0.0))
                if applied_now != obstacle["applied"]:
                    obstacle["version"] += 1
                    obstacle["applied"] = applied_now
                    obstacle["reprojections"] = obstacle.get("reprojections", 0) + 1
                    obstacle["return_reprojected_at_s"] = t
                    leg_now, leg_dir_now = trackers[ms.phase].leg_ahead()
                    obstacle["layer"].refresh(
                        t, applied_now, obstacle["version"], current_pose=tuple(float(v) for v in rear),
                        path_ahead=leg_now, loaded=loaded, direction=leg_dir_now,
                    )
            start = PlanningPose(float(rear[0]), float(rear[1]), float(rear[2]))
            obstacle["deadline"] = hooks.wall() + 20.0
            if self.jobs is not None and (self.pending is not None or (slam is not None
                                                                       and slam.get("pending_release") is not None)):
                # Real-time plan D8b: one plan waited for at a time; a pending SLAM release goes first (its own
                # replan follows), and the leg is planned after it, on the released pose.
                obstacle["live_replanned"][ms.phase] = False
                obstacle["deadline"] = None
                return True
            worker_ = self.jobs is not None and not self._return_in_tick()
            if worker_ and slam is not None and not slam["stop_now"]:
                # Real-time plan D8b: a leg asked while the truck still rolls out of the last phase is dropped as
                # moving when it comes back, and each drop counts (Isaac V8-4 d8c_6: two drops in 0.25 s at the
                # transport's start). It stands until the stop detector holds, then asks.
                obstacle["live_replanned"][ms.phase] = False
                obstacle["deadline"] = None
                return True
            if worker_:
                # Real-time plan D8b: the worker plans (the tight retry with it, under the same deadline); the leg
                # stands until the plan is taken. Plan D11: a return turnaround goes too, with its arc check frozen.
                if ms.phase == "transport":
                    sc_ = slam.get("transport_scenario", scenario) if slam is not None else scenario
                    payload = self._leg_payload("transport", grid_world(sc_), start, grid_kwargs(None),
                                                retry_on=("invalid_start", "no_path"))
                else:
                    sc_ = scenario if slam is None else slam.get("return_scenario", slam.get("transport_scenario", scenario))
                    payload = self._leg_payload("return", grid_world(sc_), start,
                                                {**grid_kwargs("return", sc_.destination),
                                                 **return_turnaround_kwargs(frozen=True)},
                                                retry_on=("invalid_start", "no_path"))
                obstacle["deadline"] = None
                self._request("live_leg", "leg", payload, t, rear, self._applied_now(),
                              ctx={"turnaround": bool(options.return_turnaround and ms.phase == "return_home")})
                return True
            if ms.phase == "transport":
                sc_ = slam.get("transport_scenario", scenario) if slam is not None else scenario
                live = hooks.plan_transport_leg(grid_world(sc_), start, planner_config, geometry=geometry,
                                                travel_config=travel_config, **grid_kwargs(None))
                if live.status in ("invalid_start", "no_path"):
                    live = hooks.plan_transport_leg(grid_world(sc_), start, replace(planner_config, clearance_m=0.0),
                                                    geometry=geometry,
                                                    travel_config=replace(travel_config, clearance_m=0.0) if travel_config is not None else None,
                                                    **grid_kwargs(None))
            else:
                sc_ = scenario if slam is None else slam.get("return_scenario", slam.get("transport_scenario", scenario))
                live = hooks.plan_return_leg(grid_world(sc_), start, options.return_to_pose, planner_config,
                                             geometry=geometry, travel_config=travel_config,
                                             **grid_kwargs("return", sc_.destination), **return_turnaround_kwargs())
                if live.status in ("invalid_start", "no_path"):
                    live = hooks.plan_return_leg(grid_world(sc_), start, options.return_to_pose,
                                                 replace(planner_config, clearance_m=0.0), geometry=geometry,
                                                 travel_config=replace(travel_config, clearance_m=0.0) if travel_config is not None else None,
                                                 **grid_kwargs("return", sc_.destination), **return_turnaround_kwargs())
            obstacle["deadline"] = None
            obstacle.setdefault("live_plans", []).append(
                {"phase": ms.phase, "time_s": t, "status": live.status,
                 "turnaround": getattr(live, "turnaround", None)})
            if not live.success:
                # Standing, the grid keeps updating (the cells the carried
                # pallet stood on clear once seen again): retry each second,
                # five times, before giving up (L3c v41 seed 4: no_path at the
                # first transport plan, which v38 planned through).
                tries = sum(1 for lp in obstacle["live_plans"] if lp["phase"] == ms.phase)
                hooks.require(tries < 5, f"obstacle_live_plan_failed in {ms.phase}: {live.status}")
                obstacle["live_replanned"][ms.phase] = False
                obstacle["live_retry_after_s"] = t + 1.0
            else:
                paths[ms.phase] = live
                state["paths"][ms.phase] = hooks.path_record(live)
                hooks.write_paths()
                trackers[ms.phase] = RearAxlePathTracker(live.poses, live.directions, live.curvatures_inv_m, trackers[ms.phase].config)
                if ms.phase == "transport" and slam is not None:
                    self.arm_docking()
                ms.phase_started = t
            stand = True
        elif (
            options.grid_planning and ms.phase in ("transport", "return_home") and ms.phase in trackers
            and not obstacle["live_replanned"].get(ms.phase)
        ):
            stand = True  # waiting to retry the live plan: stand
        if obstacle is not None and obstacle.get("replan_wait", {}).get(ms.phase):
            check_ = obstacle["layer"].permission.path_check
            if (
                obstacle.get("backoff", {}).get("phase") != ms.phase
                and check_ is not None and check_.blocked is None
            ):
                # The path ahead verified again -- no OCCUPIED and no UNKNOWN
                # cell (a zero blocked count is not that, Codex checkpoint 8
                # P2): the existing path resumes.
                obstacle["replan_wait"].pop(ms.phase, None)
            else:
                stand = True  # a blocked-path replan failed: stand until one succeeds
        return stand

    # --- persistent console plan D4: the recovery of a cancel that stood with the forks in a pocket ------------------
    def recovery_failed(self, t: float, why: str) -> None:
        self.commands.terminate("recovery_required", t, f"recovery:{why}")

    # --- plan D4b: a cancel ends where the next preparation can start ---------------------------------------------
    def observe_motion(self, odom_rear) -> None:
        """Each live tick's end: the last 3 m of odometry rear poses (never the control pose: a correction jump is not
        a move), every 0.02 m, and whether the last of those steps went forward along the heading (plan D4b)."""
        odom_rear = tuple(float(v) for v in odom_rear)
        forward = True
        if self.recent_odom:
            last = self.recent_odom[-1]
            dx, dy = odom_rear[0] - last[0], odom_rear[1] - last[1]
            if math.hypot(dx, dy) < 0.02:
                return
            forward = self.recent_forward = dx * math.cos(odom_rear[2]) + dy * math.sin(odom_rear[2]) >= 0.0
        self.recent_odom.append(odom_rear)
        self.recent_steps.append(forward)
        del self.recent_odom[:-150], self.recent_steps[:-150]

    def end_gate(self, t: float, rear, extra_m: float = 0.0) -> bool:
        """Whether the next preparation's start check passes here with the inputs it would have after the end release
        (the runner's prepare_start_ok); no check (a batch run): passes."""
        check = getattr(self.hooks, "prepare_start_ok", None)
        if check is None or self.baseline is None:
            return True
        return bool(check(tuple(float(v) for v in rear), extra_m=extra_m, preview=True, t=t))

    def finish_cancel(self, t: float, rear) -> bool:
        """A live cancel (or a recovery) ends through the gate: there, cancelled (True, the end goes on); inside the
        next preparation's clearance, a back-out first (False)."""
        cmds = self.commands
        if self.end_gate(t, rear):
            if cmds.status == "cancelling":
                cmds.cancelled(t)
            else:
                self.recovery_done(t)
            return True
        if cmds.status == "cancelling":
            cmds.begin_recovery(t)
        self.clear_out_used_m = 0.0
        self.start_clear_out(t, rear)
        return False

    def _odom_to_control(self, pose):
        slam = self.slam
        if slam is None or slam["tracker"].applied is None:
            return tuple(pose)
        return tuple(float(v) for v in compose(tuple(float(v) for v in slam["tracker"].applied[0]), tuple(pose)))

    def start_clear_out(self, t: float, rear, *, extend_only: bool = False) -> None:
        """The back-out (plan D4b): held; after forward motion the recent odometry trajectory retraced, after reverse
        motion (or to extend) straight back along the heading; it ends at the first sample the next preparation's
        start check passes with CLEAR_OUT_MARGIN_M; none within the budget: recovery_required."""
        slam = self.slam
        if slam is not None and slam["tracker"].mode != "holding":
            self.hold_slam(t, held_frame_pallet=False)  # the back-out moves in one frame
        rear = tuple(float(v) for v in rear)
        budget = CLEAR_OUT_BUDGET_M - self.clear_out_used_m
        if budget < 0.02:
            self.recovery_failed(t, "clear_out:budget")
            return
        if extend_only:
            count = max(2, int(round(CLEAR_OUT_STEP_M / 0.02)) + 1)
            poses = [(rear[0] - s * math.cos(rear[2]), rear[1] - s * math.sin(rear[2]), rear[2])
                     for s in np.linspace(0.0, min(CLEAR_OUT_STEP_M, budget), count)]
        else:
            poses = [rear]
            if self.recent_forward and len(self.recent_odom) >= 2:
                # Only the last forward run (Codex D4b impl round 1 P1-2: a gear change in the trace put forward
                # segments into a reverse path): back to where that run began.
                first = len(self.recent_odom) - 1
                while first > 0 and self.recent_steps[first]:
                    first -= 1
                poses += [self._odom_to_control(p) for p in reversed(self.recent_odom[first:-1])]
            # then straight back along the heading there, for what is left of the budget
            end = poses[-1]
            run = float(np.sum(np.hypot(np.diff(np.asarray(poses)[:, 0]), np.diff(np.asarray(poses)[:, 1])))) \
                if len(poses) > 1 else 0.0
            poses += [(end[0] - s * math.cos(end[2]), end[1] - s * math.sin(end[2]), end[2])
                      for s in np.arange(0.02, budget - run + 1e-9, 0.02)]
            out, length = [poses[0]], 0.0
            for pose in poses[1:]:
                step = math.hypot(pose[0] - out[-1][0], pose[1] - out[-1][1])
                if step < 1e-6:
                    continue
                if length + step > budget + 1e-9:
                    break
                length += step
                out.append(pose)
                if self.end_gate(t, pose, CLEAR_OUT_MARGIN_M):
                    break
            else:
                out = None
            if out is None or len(out) < 2 or not self.end_gate(t, out[-1], CLEAR_OUT_MARGIN_M):
                self.recovery_failed(t, "clear_out:no_start_within_budget")
                return
            poses = out
        if self._install_retreat(t, rear, poses, "clear_out"):
            self.clear_out_used_m += self.paths["clear_out"].length_m
            if self.ms.phase != "clear_out":
                self.transition("clear_out", t)

    @staticmethod
    def _rest_from(poses, cum, done, rear):
        """The rest of a path from the truck's projection on it, in the path's order (Codex D4b impl round 4): the
        segments that end beyond the follower's progress; the nearest one's foot point starts the rest, so it never
        goes back to a sample already passed."""
        here = np.asarray(rear, dtype=float)[:2]
        best = None
        for k in range(len(poses) - 1):
            if cum[k + 1] < done - 1e-9:
                continue  # wholly behind the progress
            a, b = poses[k, :2], poses[k + 1, :2]
            ab = b - a
            span = float(ab @ ab)
            lo = 0.0 if cum[k] >= done else min(1.0, (done - cum[k]) / max(cum[k + 1] - cum[k], 1e-12))
            u = lo if span < 1e-18 else min(1.0, max(lo, float((here - a) @ ab) / span))
            foot = a + u * ab
            dist = float(np.hypot(*(here - foot)))
            if best is None or dist < best[0] - 1e-12:
                best = (dist, k, u, foot)
        if best is None:
            return poses[-1:]
        _, k, u, foot = best
        yaw = float(poses[k, 2] + u * math.atan2(math.sin(poses[k + 1, 2] - poses[k, 2]),
                                                 math.cos(poses[k + 1, 2] - poses[k, 2])))
        rest = poses[k + 1:]
        if u < 1.0 - 1e-9:
            rest = np.vstack((np.asarray([[foot[0], foot[1], yaw]]), rest))
        return rest

    def clear_out_arrived(self, t: float, rear) -> None:
        if self.end_gate(t, rear):
            self.recovery_done(t)
        else:
            self.start_clear_out(t, rear, extend_only=True)

    def restore_pallet_target(self) -> None:
        """The recovered pallet back in the targets (on the floor where the mission's estimate holds it: the pickup's,
        or the destination's once placed there) -- at the ground verdict, so the end's start check sees it."""
        mission = self.commands.mission
        tb = self.targets
        ctx = self.cargo.recovery_context
        if ctx is not None and ctx.get("pose") is not None:
            # Plan D14 (Codex design round 1 P2-7): the mission is gone; the context kept where the pallet is.
            pid = ctx["pallet_id"]
            if pid in tb.pallets and tb.pallets[pid].state == "carried":
                tb.put_down(pid, TargetPose(*(float(v) for v in ctx["pose"])),
                            **({} if ctx["placed"] else {"state": "available"}), uncertainty_m=RECOGNISED_MARGIN_M)
            return
        if mission is None or mission.pallet_id not in tb.pallets or tb.pallets[mission.pallet_id].state != "carried":
            return
        if self.cargo.placed is not None:
            scenario_ = self.options.scenario
            if self.slam is not None:
                scenario_ = self.slam.get("return_scenario", self.slam.get("transport_scenario", scenario_))
            site_ = scenario_.destination
            tb.put_down(mission.pallet_id, TargetPose(float(site_.x_m), float(site_.y_m), float(site_.yaw_rad)),
                        uncertainty_m=RECOGNISED_MARGIN_M)
        else:
            est_ = (self.obstacle or {}).get("pickup_estimate") or (self.pocket or {}).get("planning_pickup")
            if est_ is not None:
                tb.put_down(mission.pallet_id, TargetPose(float(est_.x_m), float(est_.y_m), float(est_.yaw_rad)),
                            state="available", uncertainty_m=RECOGNISED_MARGIN_M)

    # --- plan D14: the manual recovery after a live mission ended with the forks in a pocket --------------------
    def capture_recovery_context(self) -> None:
        """Before the mission and its state go (end_reset): what a manual recovery needs -- the pallet's ID and
        estimate, where the destination's pallet stands, the SLAM hold's basis, and whether it is on the floor."""
        cargo, mission = self.cargo, self.commands.mission
        if self.baseline is None or cargo.relation not in ("constrained", "unknown") or mission is None:
            return
        slam = self.slam
        holding = slam is not None and slam["tracker"].mode == "holding"
        pallet_held, destination_held = self._held_estimates() if holding else (False, False)
        if cargo.placed is not None:
            site_ = self._destination_estimate()
        else:
            site_ = (self.obstacle or {}).get("pickup_estimate") or (self.pocket or {}).get("planning_pickup")
        pose = None if site_ is None else (float(site_.x_m), float(site_.y_m), float(site_.yaw_rad))
        cargo.recovery_context = {
            "pallet_id": mission.pallet_id, "destination_id": mission.destination_id, "pose": pose,
            "placed": cargo.placed is not None, "pallet_held": bool(pallet_held),
            "destination_held": bool(destination_held), "stage": "in_pocket",
            "grounded": cargo.load == "none", "ground_since": None}

    def recovery_report(self) -> dict | None:
        """The stage the teleop policy reads (plan D14 table): grounded, the evidence, aligned to the pocket axis."""
        cargo, ctx = self.cargo, self.cargo.recovery_context
        if ctx is None:
            return None
        aligned = False
        if cargo.last_rear is not None:
            offset = cargo.alignment(cargo.last_rear)
            aligned = offset is not None and abs(offset[0]) <= cargo.band(cargo.last_rear) + 1e-9 \
                and abs(offset[1]) <= ALIGN_HEADING_RAD + 1e-12
        return {"stage": ctx["stage"], "grounded": bool(ctx["grounded"]), "evidence": bool(cargo.trace_valid),
                "aligned": bool(aligned), "pallet_id": ctx["pallet_id"]}

    def manual_recovery_tick(self, t: float, rear, *, standing: bool) -> bool:
        """Each live tick with no mission adopted: the manual ground verdict (the pallet off the forks onto the
        floor, held 1 s: the evidence moves to the floor and the pallet back into the targets), then the free
        verdict by the automatic recovery's criteria (the release deferred since the end, the cargo carried with
        it), then the manual clear (ended by the runner's permit check). True: a release moved the control pose."""
        cargo, ctx, hooks = self.cargo, self.cargo.recovery_context, self.hooks
        if ctx is None or ctx["stage"] != "in_pocket" or cargo.relation != "constrained":
            return False
        rear = tuple(float(v) for v in rear)
        lowered = standing and hooks.lift_position() < 0.012 and abs(hooks.pallet_z()) < 0.008
        log = self.state.setdefault("manual_recovery", [])
        if not ctx["grounded"]:
            if not lowered:
                ctx["ground_since"] = None
                return False
            ctx["ground_since"] = t if ctx["ground_since"] is None else ctx["ground_since"]
            if t - ctx["ground_since"] < MANUAL_GROUND_HOLD_S:
                return False
            ctx["grounded"] = True
            getattr(hooks, "trace_ground", lambda: None)()  # the evidence's reference moves to the floor
            self.restore_pallet_target()
            log.append({"time_s": t, "event": "grounded", "depth_m": cargo.depth(rear)})
            return False
        out = cargo.depth(rear) < -(APPROACH_GAP_M + cargo.band(rear))
        if not (lowered and out and cargo.trace_valid):
            return False
        cargo.recovered()
        ctx["stage"] = "manual_clear"
        self.restore_pallet_target()
        log.append({"time_s": t, "event": "free", "depth_m": cargo.depth(rear)})
        slam = self.slam
        if slam is not None and slam["tracker"].mode == "holding":
            # The end release deferred since the mission ended constrained (D1 step 4), with the basis kept then.
            self.end_release_due = {"pallet_id": ctx["pallet_id"], "destination_id": ctx["destination_id"],
                                    "pallet_held": ctx["pallet_held"], "destination_held": ctx["destination_held"]}
            self.end_release(t, deferred=True)
            return True
        return False

    def manual_clear_done(self, t: float) -> None:
        """The runner's permit allows the reverse without the recovered pallet's exemption: an ordinary teleop."""
        ctx = self.cargo.recovery_context
        if ctx is None or ctx["stage"] != "manual_clear":
            return
        self.state.setdefault("manual_recovery", []).append({"time_s": t, "event": "clear"})
        self.cargo.recovery_context = None

    def recovery_done(self, t: float) -> None:
        """Backed out: the mission ends cancelled; the pallet is back on the floor in the targets list (Codex 6b impl
        P1-1; already there since the ground verdict)."""
        self.commands.recovered(t)
        self.restore_pallet_target()
        self.cargo.recovered()

    def recovery_timers(self, t: float) -> None:
        """recover_lower's verdict (the lift down and the pallet on the floor -- always read, however low the lift
        started) and both phases' time limit."""
        ms, hooks = self.ms, self.hooks
        if self.commands.status != "recovering":
            return
        elapsed = self.commands.work_elapsed(ms.phase_started, t)
        if elapsed > RECOVERY_PHASE_LIMIT_S:
            self.recovery_failed(t, f"{ms.phase}_timeout")
            return
        if ms.phase == "recover_lower" and elapsed >= RECOVERY_LOWER_MIN_S:
            if hooks.lift_position() < 0.012 and abs(hooks.pallet_z()) < 0.008:
                if not self.cargo.trace_valid:
                    self.recovery_failed(t, "trace_invalid")
                    return
                self.restore_pallet_target()  # plan D4b: the end's start check sees it (Codex 6b-2 round 2 P2-2)
                self.transition("recover_withdraw", t)
                self.install_recover_withdraw(t, self.cargo.last_rear)

    def install_recover_withdraw(self, t: float, rear) -> None:
        """The retrace from the depth now (plan D4): judged whole by the permission before it moves; none left: the
        end, through the next preparation's start check (plan D4b)."""
        if rear is None:
            self.recovery_failed(t, "no_pose")
            return
        poses = self.cargo.recovery_path(rear, 0.02)
        if len(poses) < 2:
            self.finish_cancel(t, rear)
            return
        if self._install_retreat(t, rear, poses, "recover_withdraw"):
            self.state.setdefault("recoveries", []).append({"time_s": t, "from_depth_m": self.cargo.depth(rear),
                                                           "length_m": self.paths["recover_withdraw"].length_m})

    def _install_retreat(self, t: float, rear, poses, phase: str) -> bool:
        """A reversing path for a recovery phase: judged whole by the permission, then installed with the withdraw's
        tracker settings. False: refused (recovery_required)."""
        poses = np.asarray(poses, dtype=float)
        count = len(poses)
        steps = np.hypot(np.diff(poses[:, 0]), np.diff(poses[:, 1]))
        if "withdraw" in self.trackers:  # the withdraw's own speed and tolerances (Codex 6b impl P1-2)
            config = self.trackers["withdraw"].config
        elif self.trackers:  # another phase's, at the withdraw's speed and docking tolerance (the runner's withdraw)
            config = replace(next(iter(self.trackers.values())).config,
                             cruise_speed_mps=self.settings["withdraw_speed_mps"], position_tolerance_m=0.008)
        else:
            self.recovery_failed(t, "no_tracker_config")
            return False
        # The planner's convention (turn = signed distance x curvature; entry i is the segment arriving at i): a
        # retraced arc keeps its curvature as the follower's feed-forward, within the follower's limit (odometry
        # jitter over a 0.02 m step is not a sharper turn).
        curvatures = np.zeros(count)
        if count > 1:
            turns = np.arctan2(np.sin(np.diff(poses[:, 2])), np.cos(np.diff(poses[:, 2])))
            curvatures[1:] = np.clip(np.where(steps > 1e-9, turns / -np.maximum(steps, 1e-9), 0.0),
                                     -config.max_curvature_inv_m, config.max_curvature_inv_m)
            curvatures[0] = curvatures[1]
        path = RecoveryPath(poses, np.full(count, -1, dtype=np.int8), curvatures, float(np.sum(steps)))
        obstacle = self.obstacle
        if getattr(self.hooks, "test_recovery_blocked", False) == "withdraw" and phase == "recover_withdraw":
            # Plan D14 tests only (U20 c): the retrace refused with its evidence valid -- the manual recovery's case.
            self.recovery_failed(t, "blocked:test")
            return False
        if obstacle is not None:
            check = obstacle["layer"].check_candidate(path.poses, current_pose=tuple(float(v) for v in rear),
                                                      direction=-1, loaded=False, lookahead_m=path.length_m)
            if check.blocked is not None or check.verified_m < path.length_m - 1e-6:
                self.recovery_failed(t, f"blocked:{check.blocked}")
                return False
        self.paths[phase] = path
        self.trackers[phase] = RearAxlePathTracker(path.poses, path.directions, path.curvatures_inv_m, config)
        self.state.setdefault("paths", {})[phase] = self.hooks.path_record(path)  # the record and the map
        self._recovery_install_gen = self.commands.mission_gen
        return True

    def exit_prefix_m(self) -> float:
        """The exit prefix of the transport path being driven (pallet_mission.ExitPrefix, docs/plans/
        2026-10-22-riibotics-trailer-exit.md D1): the straight reverse its planner put first, worked out again from the
        tracker's start pose and the run's ExitPrefix (hooks.exit_prefix) and held only where the path does start with
        that straight -- so a split (arm_docking) keeps it, a replan gets its own, the docked straight none. 0 without an
        exit prefix. Cached per tracker."""
        exit_ = getattr(self.hooks, "exit_prefix", None)
        tracker = self.trackers.get("transport")
        if exit_ is None or tracker is None:
            return 0.0
        cached = getattr(self, "_exit_prefix_cache", None)
        if cached is not None and cached[0] is tracker:
            return cached[1]
        from forklift_core.planning.pallet_mission import exit_prefix_length_m

        start = PlanningPose(*(float(v) for v in tracker._poses[0]))
        delta = exit_prefix_length_m(start, self.geometry.loaded_footprint, exit_) or 0.0
        value = delta if delta > 0 and tracker.leading_reverse_straight_m() >= delta - 1e-6 else 0.0
        self._exit_prefix_cache = (tracker, value)
        return value

    def permission_acting(self) -> tuple[bool, bool]:
        """Whether the permission acts on this phase's command, and whether the truck is on a docking straight
        (the permission's own reading, shared by the permission tick and its watch while paused)."""
        ms, obstacle, slam, trackers, geometry = self.ms, self.obstacle, self.slam, self.trackers, self.geometry
        docking_straight = (ms.phase == "approach" and trackers[ms.phase].remaining_to_goal_m() <= geometry.alignment_straight_m) or (
            ms.phase == "transport" and trackers[ms.phase].remaining_to_goal_m() <= geometry.delivery_straight_m
        )
        if obstacle.get("backoff", {}).get("phase") == ms.phase:
            # A backoff is never the delivery straight, whatever its
            # remaining length: the permission acts on it (Codex
            # checkpoint 8 P1).
            docking_straight = False
        if slam is not None and slam["docking"].get("reapproach"):
            # Nor is a D7c re-approach (Codex D7c review P1-2): the
            # permission, and the safety evaluation, act on all of it.
            docking_straight = False
        if docking_straight and ms.phase == "transport" and (prefix_m := self.exit_prefix_m()) > 0 \
                and trackers["transport"].progress_m() < prefix_m:
            # Nor is the transport's exit prefix (docs/plans/2026-10-22-riibotics-trailer-exit.md D1, Codex 2nd P1): on
            # a short transport the delivery straight's window (after arm_docking's split) can reach back into it.
            docking_straight = False
        acting = (
            ms.phase in ("observe", "approach", "transport", "return_home", "recover_withdraw", "clear_out")
            or (ms.phase == "withdraw" and obstacle["layer"].known_enabled
                and obstacle["layer"].known_config.get("withdraw_mode", "certified") == "certified")  # D5 corridor
        ) and not docking_straight
        return acting, docking_straight

    def permission(self, t: float, rear, kappa: float, requested_speed: float, loaded: bool, cusp_handed_) -> SimpleNamespace:
        """One tick of the obstacle permission while the truck follows a path (or a stall is handed over): the
        grid re-projected after a correction change, the permission's limit and its diagnostics dump, the
        docking straight and the pocket check's answer inside the pallet region, the path-blocked count, the
        N9 silence stop, the speed cap and the blocked/no-progress watch. Returns ``allowed``, ``why``,
        ``acting``, ``pocket_phase``, ``obstacle_hold`` and the capped ``requested_speed`` (the runner's
        replan reads the first four)."""
        ms, state, obstacle, slam, pocket, hooks, options = (self.ms, self.state, self.obstacle, self.slam, self.pocket,
                                                            self.hooks, self.options)
        trackers, geometry = self.trackers, self.geometry
        truth_speed, truth_rear = hooks.truth_speed(), hooks.truth_rear()
        obstacle_hold = False
        if slam is not None and slam["tracker"].applied is not None:
            applied_now = tuple(float(v) for v in slam["tracker"].applied[0])
        else:
            applied_now = (0.0, 0.0, 0.0)
        if applied_now != obstacle["applied"]:
            # A correction change since the snapshot (a SLAM release): the
            # grid is re-projected now, never mixed with the new pose
            # (Codex L0b P1).
            self._reproject(t, applied_now, rear, loaded)
        if requested_speed != 0.0:
            direction = 1 if requested_speed > 0 else -1
        elif cusp_handed_ is not None:
            # The stalled leg's direction (the next leg's for an occupied prefix), not the sign of
            # a standing truck's noise -- the e-stop arc must not flip side every tick.
            direction = cusp_handed_.hand_direction
        else:
            direction = 1 if truth_speed > 0 else -1
        allowed, why = obstacle["layer"].limit(
            t, current_pose=tuple(float(v) for v in rear), curvature_inv_m=kappa,
            direction=direction, loaded=loaded, cap_mps=max(abs(requested_speed), abs(truth_speed)),
        )
        dumped_phases = obstacle.setdefault("dumped_phases", set())
        if allowed == 0.0 and why in ("unknown", "occupied") and t > 5.0 and (
            obstacle.get("dumps", 0) < 4 or ms.phase not in dumped_phases
        ):
            dumped_phases.add(ms.phase)
            # Diagnostics: the snapshot and the check inputs of the first unknown stops.
            obstacle["dumps"] = obstacle.get("dumps", 0) + 1
            snap_ = obstacle["layer"].snapshot
            hooks.save_npz(
                f"obstacle_dump_{obstacle['dumps']}.npz",
                state=snap_.state, free_stamp=snap_.free_stamp,
                origin=[snap_.origin_x_m, snap_.origin_y_m], res=snap_.resolution_m,
                t=t, rear=np.asarray(rear, dtype=float), truth_rear=np.asarray(truth_rear, dtype=float),
                kappa=kappa, direction=direction, loaded=loaded,
                requested=requested_speed, truth_speed=truth_speed, phase=ms.phase, why=why,
                leg=(trackers[ms.phase].leg_ahead()[0] if ms.phase in trackers else np.zeros((0, 3))),
                correction=np.asarray(obstacle["applied"], dtype=float), version=obstacle["version"],
                stamp=snap_.stamp_s,
                blocked_cells=np.asarray(
                    getattr(getattr(obstacle["layer"].permission, "last_estop", None), "blocked_cells", ()),
                    dtype=np.int64,
                ).reshape(-1, 2),
                path_blocked_cells=np.asarray(
                    getattr(obstacle["layer"].permission.path_check, "blocked_cells", ()),
                    dtype=np.int64,
                ).reshape(-1, 2),
                **{
                    f"raw_{name}_{part}": np.asarray(arr, dtype=float)
                    for name, values in obstacle.get("last_raw", ({}, None, None))[0].items()
                    for part, arr in zip(("d", "hit", "own", "limit"), values)
                },
                raw_base=obstacle.get("last_raw", ({}, np.zeros(3), np.zeros(4)))[1],
                raw_q=obstacle.get("last_raw", ({}, np.zeros(3), np.zeros(4)))[2],
            )
        ticks = obstacle["ticks"]
        ticks[ms.phase] = ticks.get(ms.phase, 0) + 1
        low = obstacle["min_allowed"]
        low[ms.phase] = min(low.get(ms.phase, float("inf")), allowed)
        acting, docking_straight = self.permission_acting()
        pocket_phase = options.pocket_check and (ms.phase == "insert" or (ms.phase == "approach" and docking_straight))
        if pocket_phase:
            # D5: the approach straight and the insertion run under the
            # permission too; inside the pallet region the depth check
            # answers (its region is waived in the grid only while set).
            acting = True
            if pocket["check"] is None:
                allowed, why = 0.0, "pocket_missing"
            elif pocket.get("await_still_frame"):
                allowed, why = 0.0, "pocket_settle"
            elif (
                ms.phase == "approach" and not pocket.get("mid_still_done")
                and trackers[ms.phase].remaining_to_goal_m() <= 0.45
            ):
                # D5: one standing frame with the camera about 1.2 m from
                # the face. Closer, the band's sides before the face
                # leave the view; farther, the depth noise is wider than
                # their 5 cm to the face; passing at speed, the frame's
                # pose uncertainty is (L3c v27 seed 2: 27 voxels never seen).
                pocket["mid_still_done"] = True
                pocket["await_still_frame"] = True
                allowed, why = 0.0, "pocket_settle"
            else:
                lp, wp = pocket["check"].limit(
                    t, tuple(float(v) for v in rear), kappa, direction,
                    max(abs(requested_speed), abs(truth_speed)),
                    float(hooks.lift_position()),
                )
                if lp < allowed:
                    allowed, why = lp, wp
                if ms.phase == "approach" and not pocket.get("mid_still_done"):
                    # The standing frame is a planned stop: brake into it at
                    # MID_STILL_DECEL_MPS2, not as a step (L3c v35 seed 4: a step
                    # from 0.53 m/s locked an odometry wheel, the held pose fell
                    # 1.85 cm behind and the face read as an obstacle).
                    cap = max(0.05, math.sqrt(2.0 * options.mid_still_decel_mps2
                                              * max(0.0, trackers[ms.phase].remaining_to_goal_m() - 0.45)))
                    if cap < allowed:
                        allowed, why = cap, "pocket_settle_brake"
                if wp != "ok" and pocket.get("last_reason") != wp:
                    # Kept now: a stop that ends the run must leave its evidence.
                    pocket["last_reason"] = wp
                    state.setdefault("pocket_check", {}).update(
                        pocket["check"].summary(), frames_read=pocket["frames_read"], last_reason=wp,
                        last_reason_s=t, last_reason_rear=[float(v) for v in rear],
                    )
            # A docking segment does not replan (plan D4): held at 0 for
            # 5 s, the check's reason ends the run.
            if allowed == 0.0:
                pocket.setdefault("zero_since", t)
                hooks.require(self.commands.work_elapsed(pocket["zero_since"], t) < 5.0, f"pocket_stop in {ms.phase}: {why}")
            else:
                pocket.pop("zero_since", None)
        # Plan D4: an occupied cell on the path ahead stops the truck where
        # it is, so the replan starts with room to manoeuvre -- creeping up
        # to the obstacle left a 2 m-radius truck boxed in (L3b v24-v27).
        # 0.3 s of a blocked path first: a passing mark does not stop it.
        check_now = obstacle["layer"].permission.path_check
        path_blocked = (
            options.grid_planning and acting and ms.phase in ("observe", "transport", "return_home")
            and check_now is not None and check_now.blocked == "occupied"
        )
        obstacle["path_blocked_ticks"] = obstacle.get("path_blocked_ticks", 0) + 1 if path_blocked else 0
        # The permission's own limit, applied to the wheels as a step
        # (the stop it was measured with); the path-blocked stop below
        # is a planned one and brakes on the slew.
        obstacle["permission_cap"] = (t, float(allowed)) if (options.obstacle_act and acting) else None
        if obstacle["permission_cap"] is not None and allowed == 0.0 and why.startswith("sensor_silent"):
            # N9 deadlines: the permission's zero reaches the wheels as a
            # step through the final cap whatever the tracker asks (Codex
            # checkpoint 11): the first such tick, and the stop it ends in.
            trace = obstacle.setdefault("silence_trace", {})
            trace.setdefault("first_zero_s", t)
            trace.setdefault("reason", why)
            trace.setdefault("speed_at_zero_mps", abs(truth_speed))
            trace.setdefault("rear_at_zero", [float(v) for v in truth_rear])
            # The stop is the start of the final standing: a slow instant followed by
            # motion is not it (Codex stage-1 3rd P2), so moving again clears it.
            if abs(truth_speed) < 0.01:
                if "stopped_s" not in trace:
                    trace["stopped_s"] = t
                    trace["rear_stopped"] = [float(v) for v in truth_rear]
            elif "stopped_s" in trace:
                trace.setdefault("interrupted_stops_s", []).append(trace.pop("stopped_s"))
                trace.pop("rear_stopped", None)
            # The N9 outcome: stopped and standing 1 s (Codex stage-1 P1-1, 2nd P2-4) ends the run.
            hooks.require(not ("stopped_s" in trace and t - trace["stopped_s"] >= 1.0), "sensor_silence_stopped")
        if obstacle["path_blocked_ticks"] >= 36 and allowed > 0.0:
            allowed, why = 0.0, "occupied"
        self.last_permission = (float(allowed), why)  # for the safety state's reason (E8)
        if abs(requested_speed) > allowed + 1e-9:
            obstacle["slowed"][ms.phase] = obstacle["slowed"].get(ms.phase, 0) + 1
            key = f"{ms.phase}:{why}"
            obstacle["reasons"][key] = obstacle["reasons"].get(key, 0) + 1
            if options.obstacle_act and acting:
                requested_speed = math.copysign(allowed, requested_speed)
                # Zero allowed: the emergency stop the check assumed --
                # wheels to zero, steering held (Codex L0b P1).
                obstacle_hold = allowed == 0.0
        # Stop, replan, resume (plan D4): blocked by an obstacle and standing.
        stopped_now = slam["stop_now"] if slam is not None else float(np.linalg.norm(hooks.truth_velocity()[:2])) < 0.01
        # Plan cusp position stall v7: an occupied prefix of the next leg (the current leg's check
        # ends at the cusp and cannot see it) counts as a blocked path while it stays.
        cusp_prefix_blocked_ = cusp_handed_ is not None and cusp_handed_.prefix_occupied_now
        if (
            options.grid_planning
            and acting
            and ms.phase in ("observe", "transport", "return_home")
            and ((allowed == 0.0 and why == "occupied") or cusp_prefix_blocked_)
            and stopped_now
        ):
            obstacle["blocked_ticks"] = obstacle.get("blocked_ticks", 0) + 1
        else:
            obstacle["blocked_ticks"] = 0
        progress_left = trackers[ms.phase].remaining_to_goal_m()
        watch = obstacle.setdefault("progress", {})
        if watch.get("phase") != ms.phase or not acting or progress_left < watch["best_m"] - 1.0:
            watch.update(phase=ms.phase, best_m=progress_left, since_s=t)
        hooks.require(
            not (options.grid_planning and acting and self.commands.work_elapsed(watch["since_s"], t) > 60.0),
            f"obstacle_no_progress in {ms.phase}",
        )
        return SimpleNamespace(allowed=allowed, why=why, acting=acting, pocket_phase=pocket_phase,
                               obstacle_hold=obstacle_hold, requested_speed=requested_speed)

    def blocked_replan(self, t: float, rear, kappa: float, requested_speed: float) -> SimpleNamespace:
        """Stop, replan, resume (plan D4) once the truck has stood blocked for a second or a replan is forced:
        the release a backoff or a D7c re-approach needs, the replan of the leg (an escape straight first when
        the start is inside the clearance, then clearance 0, other observation candidates before recognition,
        the whole mission again after it), and what is installed -- an escape, a backoff, a wait, the same
        path or the new path. Returns the control pose (moved by a release) and the requested speed."""
        if self.bottom_deck_leg_active():
            # Plan 2026-10-19 task 4: a side leg is replanned to its own stand only, released first; its failure
            # ends the face -- never the far candidates or the mission replan below (Codex design round 1 P2).
            self.obstacle["blocked_ticks"] = 0
            out_ = self.bottom_deck_recover(t, rear, "blocked")
            return SimpleNamespace(rear=out_.rear, requested_speed=0.0)
        ms, state, obstacle, slam, pocket, hooks, options = (self.ms, self.state, self.obstacle, self.slam, self.pocket,
                                                            self.hooks, self.options)
        paths, trackers, geometry = self.paths, self.trackers, self.geometry
        planner_config, travel_config, scenario = options.planner_config, options.travel_config, options.scenario
        return_to_pose, pickup_bounds, mission_stages = options.return_to_pose, options.pickup_bounds, options.mission_stages
        observe_bounds_ = self.observe_bounds()
        BACKOFF_M, BACKOFF_SPEED_MPS, BACKOFF_PER_LEG, ESCAPE_PER_LEG = (
            options.backoff_m, options.backoff_speed_mps, options.backoff_per_leg, options.escape_per_leg)
        require, wall, same_path, path_stats, path_record = (hooks.require, hooks.wall, hooks.same_path, hooks.path_stats,
                                                             hooks.path_record)
        realign_active, realign_install, release_and_carry, release_after_backoff, slam_rear = (
            self.realign_active, self.realign_install, self.release_and_carry, self.release_after_backoff,
            hooks.slam_rear)
        grid_world, grid_kwargs, plan_observation_leg, plan_return_leg, plan_transport = (
            hooks.grid_world, hooks.grid_kwargs, hooks.plan_observation_leg, hooks.plan_return_leg, hooks.plan_transport)
        transport_replan, find_escape, escape_leg, final_straight_prefix = (
            self.transport_replan, self.find_escape, self.escape_leg, hooks.final_straight_prefix)
        apply_tracker_profile, arm_docking = self.apply_tracker_profile, self.arm_docking
        # Plan D8 S4c-3 stage 2: as above, an obstacle recovery held in the realignment.
        require(not (ms.phase == "observe" and realign_active() and slam is not None
                     and slam["tracker"].mode == "holding"), "near_capture_realign_held_recovery")
        obstacle["blocked_ticks"] = 0
        perm_ = obstacle["layer"].permission
        # Plan v4 D4: which check stopped the truck, at what steering -- read before a release
        # (a backoff's end released already and kept what it read before).
        pre_ = (obstacle["backoff"].pop("stop_cause", None)
                if obstacle.get("backoff", {}).get("phase") == ms.phase else None)
        stop_cause_ = {
            **(pre_ or {"path": getattr(perm_.path_check, "blocked", None),
                        "estop": getattr(getattr(perm_, "last_estop", None), "blocked", None)}),
            "kappa_inv_m": float(kappa),
            "read": "backoff_end" if pre_ is not None else "recovery",
        }
        if ms.phase == "transport" and slam is not None and slam["docking"].get("reapproach"):
            # Plan D7c: a re-approach path is replaced released, never as
            # a long held manoeuvre (Codex D7c re-review P2-5).
            released_ = release_and_carry(t)
            if released_["released"]:
                rear = slam_rear(t)
                hooks.set_rear(rear)  # the runner's closures (grid_kwargs, ...) read its rear (Codex slice-3 P2)
                slam["control"][-1][1:4] = rear.tolist()
        elif obstacle.get("backoff", {}).get("phase") == ms.phase:
            released_b = release_after_backoff(t, "release_in_backoff_recovery")  # plan v4 E3
            if released_b is not None:
                rear = released_b
                hooks.set_rear(rear)  # the runner's closures (grid_kwargs, ...) read its rear (Codex slice-3 P2)
        start = PlanningPose(float(rear[0]), float(rear[1]), float(rear[2]))
        replan_start = wall()
        tight = replace(planner_config, clearance_m=0.0)
        tight_travel = replace(travel_config, clearance_m=0.0) if travel_config is not None else None
        # One shared 20 s budget for the whole replan (plan D3):
        # retries and other candidates included.
        obstacle["deadline"] = wall() + 20.0
        if ms.phase == "observe" and obstacle.get("pickup_estimate") is not None:
            # After recognition half the shared budget goes to this leg,
            # the rest to the mission replan below if it fails
            # (Codex checkpoint P2: one 20 s budget for the whole replan).
            obstacle["deadline"] = replan_start + 10.0
        if ms.phase == "observe":
            target = PlanningPose(*(float(v) for v in paths["observe"].poses[-1]))
            replanned = plan_observation_leg(
                grid_world(scenario), target, planner_config, geometry=geometry,
                start_rear=start, pickup_bounds=observe_bounds_, extended=False,
                **grid_kwargs("observe"),
            )
        elif ms.phase == "transport":
            replanned = transport_replan(start, planner_config, travel_config)
        else:
            back = scenario if slam is None else slam.get("return_scenario", slam.get("transport_scenario", scenario))
            replanned = plan_return_leg(
                grid_world(back), start, return_to_pose, planner_config, geometry=geometry,
                travel_config=travel_config, **grid_kwargs("return", back.destination),
            )
        escape_, escape_cands_, escape_skips_back_ = None, None, False
        leg_dir_e = obstacle.get("backoff", {}).get("leg_dir")
        if leg_dir_e is None and ms.phase in trackers:
            leg_dir_e = trackers[ms.phase].leg_ahead()[1] or 1
        if (
            replanned.status == "invalid_start" and ms.phase in ("transport", "return_home")
            and obstacle.setdefault("escapes", {}).get(ms.phase, 0) < ESCAPE_PER_LEG
            and not (ms.phase == "transport" and slam is not None
                     and (slam["docking"].get("reapproach") or slam["docking"].get("accepted_any")))
        ):
            # Obstacle recovery v4 E1/E2: the start is inside the planning clearance -- look
            # for a straight escape before the clearance-0 replan (smoke 2382).
            escape_, escape_cands_ = find_escape(t, rear, leg_dir_e, kappa)
            old_d_ = -1 if leg_dir_e >= 0 else 1
            escape_skips_back_ = any(
                c.direction == old_d_ and abs(c.length_m - BACKOFF_M) < 1e-9 and not c.sweep_free
                for c in escape_cands_
            )
            if escape_ is not None:
                from forklift_core.control import escape as esc_

                poses_e = esc_.straight(rear, escape_.direction, escape_.length_m)
                replanned = PlanResult(
                    True, "escape", poses_e, np.full(len(poses_e), escape_.direction, dtype=np.int8),
                    np.zeros(len(poses_e)), escape_.length_m, 0,
                )
        # Plan v4 D4: the clearance of the plan finally adopted.
        clearance_used_ = float(
            (travel_config if ms.phase in ("transport", "return_home") and travel_config is not None
             else planner_config).clearance_m)
        if replanned.status in ("invalid_start", "no_path"):
            # The truck stopped closer to the obstacle than the planning
            # clearance (the stop guards the volume ahead, not the
            # sides): replan once with no clearance. The grid's swelling
            # already holds the placement error and the permission
            # still guards every tick.
            if ms.phase == "observe":
                replanned = plan_observation_leg(
                    grid_world(scenario), target, tight, geometry=geometry,
                    start_rear=start, pickup_bounds=observe_bounds_, extended=False,
                    **grid_kwargs("observe"),
                )
            elif ms.phase == "transport":
                replanned = transport_replan(start, tight, tight_travel)
            else:
                replanned = plan_return_leg(
                    grid_world(back), start, return_to_pose, tight, geometry=geometry,
                    travel_config=tight_travel, **grid_kwargs("return", back.destination),
                )
            obstacle.setdefault("tight_replans", 0)
            obstacle["tight_replans"] += 1
            clearance_used_ = 0.0
        if not replanned.success:
            # Diagnostics: the inputs of the failed replan, for a CPU replay.
            failed_grid = grid_kwargs("observe" if ms.phase == "observe" else None)
            hooks.save_npz(
                f"obstacle_replan_fail_{len(obstacle['replans'])}.npz",
                occupied=failed_grid["occupancy"].occupied,
                origin=[failed_grid["occupancy"].origin_x_m, failed_grid["occupancy"].origin_y_m],
                res=failed_grid["occupancy"].resolution_m,
                start=[start.x_m, start.y_m, start.yaw_rad],
                target=[target.x_m, target.y_m, target.yaw_rad] if ms.phase == "observe" else [np.nan] * 3,
                phase=ms.phase, status=replanned.status,
                pickup_obstacle=(
                    [failed_grid["pickup_obstacle"].x_m, failed_grid["pickup_obstacle"].y_m,
                     failed_grid["pickup_obstacle"].length_m, failed_grid["pickup_obstacle"].width_m,
                     failed_grid["pickup_obstacle"].yaw_rad] if "pickup_obstacle" in failed_grid else []
                ),
            )
        if not replanned.success and ms.phase == "observe" and obstacle.get("pickup_estimate") is None:
            # Before recognition only: the way to this observation point
            # is blocked, so try the remaining observation candidates
            # from here, as an unreachable candidate is handled at the
            # first plan. After recognition the leg goes to the near
            # capture and another observation point would be wrong.
            while ms.next_candidate_index < len(options.observation_waypoints):
                candidate_index = ms.next_candidate_index
                ms.next_candidate_index += 1
                coordinates = options.observation_waypoints[candidate_index]
                candidate_plan = plan_observation_leg(
                    grid_world(scenario), PlanningPose(*(float(v) for v in coordinates)),
                    tight, geometry=geometry, start_rear=start,
                    pickup_bounds=observe_bounds_, extended=False, **grid_kwargs("observe"),
                )
                state["observation_candidates"].append(
                    {"candidate_index": candidate_index, "pose": list(coordinates),
                     "start_rear": [float(v) for v in rear], "success": candidate_plan.success,
                     "status": candidate_plan.status, "after": "obstacle_block",
                     "analytic_expansion_interval": candidate_plan.analytic_expansion_interval,
                     "search_attempts": [list(entry) for entry in candidate_plan.search_attempts]}
                )
                if candidate_plan.success:
                    replanned = candidate_plan
                    clearance_used_ = 0.0  # the candidates are planned tight
                    state["observation_waypoint_selected"] = list(coordinates)
                    break
        if (
            not replanned.success and ms.phase == "observe" and obstacle.get("pickup_estimate") is not None
            and pocket.get("planning_pickup") is not None
        ):
            # After recognition: the near-capture waypoint came from a
            # mission plan made on an earlier grid and may now sit
            # against an obstacle that has grown in view (L3b v24: valid
            # only at zero clearance, no path in 120 000 expansions).
            # The pallet, not the waypoint, is the goal: plan the
            # mission again from here, and its final straight gives the
            # new waypoint. The rest of the shared budget.
            obstacle["deadline"] = replan_start + 20.0
            planning_trace = []
            mission_again = plan_transport(
                grid_world(scenario), planner_config, geometry=geometry,
                target_pickup=pocket["planning_pickup"], start_rear=start,
                return_to=return_to_pose, pickup_bounds=pickup_bounds, travel_config=travel_config,
                trace=planning_trace, **grid_kwargs("mission", pocket["planning_pickup"]),
            )
            mission_clearance_ = float(planner_config.clearance_m)
            if "invalid_start" in mission_again.status or "invalid_goal" in mission_again.status:
                mission_clearance_ = 0.0
                # The waypoint next to an obstacle that has grown in
                # view is valid only without the planning clearance
                # (L3b v26); the grid's swelling holds the placement error.
                mission_again = plan_transport(
                    grid_world(scenario), tight, geometry=geometry,
                    target_pickup=pocket["planning_pickup"], start_rear=start,
                    return_to=return_to_pose, pickup_bounds=pickup_bounds, travel_config=tight_travel,
                    trace=planning_trace, **grid_kwargs("mission", pocket["planning_pickup"]),
                )
            near_again = (
                final_straight_prefix(mission_again.approach, geometry.alignment_straight_m)
                if mission_again.success else None
            )
            obstacle.setdefault("mission_replans", []).append(
                {"time_s": t, "status": mission_again.status, "near_found": near_again is not None,
                 "trace": planning_trace}
            )
            if near_again is not None:
                for name in mission_stages:
                    paths[name] = getattr(mission_again, name)
                    trackers[name] = RearAxlePathTracker(
                        paths[name].poses, paths[name].directions, paths[name].curvatures_inv_m,
                        trackers[name].config,
                    )
                    state["paths"][name] = path_record(paths[name])
                apply_tracker_profile()
                replanned = near_again
                clearance_used_ = mission_clearance_
                state["near_capture"]["waypoint"] = near_again.poses[-1].tolist()
                state["observation_waypoint_selected"] = near_again.poses[-1].tolist()
        obstacle["deadline"] = None
        obstacle["replans"].append(
            {
                "phase": ms.phase,
                "time_s": t,
                "rear_pose": [float(v) for v in rear],
                "status": replanned.status,
                "planning_wall_s": wall() - replan_start,
                **path_stats(replanned),
                # Plan v4 D4: the clearance the installed plan had, what stopped the truck.
                "clearance_m": clearance_used_,
                "stop": stop_cause_,
            }
        )
        if escape_cands_ is not None:
            obstacle["replans"][-1]["escape_candidates"] = [c.record() for c in escape_cands_]
        if escape_ is not None:
            # Plan v4 E2: driven on the backoff machinery with its own budget and its end
            # tolerance; not a failure, outside the 30 s and per-leg counts.
            obstacle["escapes"][ms.phase] = obstacle["escapes"].get(ms.phase, 0) + 1
            obstacle["replans"][-1].update(
                backoff=True, failed=False, escape=escape_.record(), clearance_m=None,
            )
            config0 = obstacle.get("backoff", {}).get("config", trackers[ms.phase].config)
            obstacle["backoff"] = {"phase": ms.phase, "config": config0, "time_s": t, "leg_dir": leg_dir_e,
                                   "escape": True}
            obstacle.setdefault("replan_wait", {}).pop(ms.phase, None)
            paths[ms.phase] = replanned
            trackers[ms.phase] = RearAxlePathTracker(
                replanned.poses, replanned.directions, replanned.curvatures_inv_m,
                replace(config0, cruise_speed_mps=BACKOFF_SPEED_MPS,
                        position_tolerance_m=esc_.END_POSITION_TOL_M,
                        yaw_tolerance_rad=esc_.END_YAW_TOL_RAD,
                        overshoot_tolerance_m=esc_.END_POSITION_TOL_M),
            )
            ms.phase_started = t  # the escape's own tracking timeout
            requested_speed = 0.0
        elif not replanned.success:
            # Standing, the grid keeps updating: a failed replan is
            # retried when the blocked count comes round again (1 s),
            # five failures per leg at most; the no-progress watch
            # bounds the wait too (L3c v45 seed 5: no_path on the way
            # home after the delivery).
            obstacle["replans"][-1]["failed"] = True
            failures = sum(1 for r in obstacle["replans"] if r["phase"] == ms.phase and r.get("failed"))
            require(failures < 5, f"obstacle_replan_failed in {ms.phase}: {replanned.status}")
            # Held until a replan succeeds (D4: stop, replan, resume
            # only on a plan -- Codex checkpoint 7 P2).
            obstacle.setdefault("replan_wait", {})[ms.phase] = True
            requested_speed = 0.0
            backs = obstacle.setdefault("backoffs", {})
            back_ok_ = not escape_skips_back_  # plan v4: E2 refused this straight by its checker
            if back_ok_ and ms.phase == "return_home":
                # Plan v4: the return's backoff must clear the delivered pallet the single
                # LiDAR cannot see (the leg's rectangles and bounds).
                from forklift_core.control import escape as esc_

                d_b = -1 if (leg_dir_e if leg_dir_e is not None else 1) >= 0 else 1
                back_ok_ = esc_.rectangles_sweep_free(
                    escape_leg(ms.phase)[2], esc_.straight(rear, d_b, BACKOFF_M)
                )
            if not back_ok_:
                obstacle["replans"][-1]["backoff_skipped"] = True
            if (ms.phase in ("transport", "return_home") and backs.get(ms.phase, 0) < BACKOFF_PER_LEG
                    and back_ok_):
                # D4 delta: stood with the outline already on the
                # blocking cells, no forward plan starts (L3c v46 seed
                # 5). Back off straight, against the leg's direction,
                # under the permission every tick (the stop volume of
                # the reverse is checked; unobserved space stops it),
                # then replan from there.
                backs[ms.phase] = backs.get(ms.phase, 0) + 1
                # The blocked leg's direction, kept across repeated
                # backoffs (a second one must not undo the first,
                # Codex checkpoint 8 P2).
                leg_dir = obstacle.get("backoff", {}).get("leg_dir")
                if leg_dir is None:
                    leg_dir = trackers[ms.phase].leg_ahead()[1] or 1
                d = -1 if leg_dir >= 0 else 1
                n_ = max(1, int(math.ceil(BACKOFF_M / 0.04)))
                fr = np.linspace(0.0, 1.0, n_ + 1)
                yaw0 = float(rear[2])
                back = PlanResult(
                    True, "backoff",
                    np.column_stack((float(rear[0]) + fr * d * BACKOFF_M * math.cos(yaw0),
                                     float(rear[1]) + fr * d * BACKOFF_M * math.sin(yaw0),
                                     np.full(n_ + 1, yaw0))),
                    np.full(n_ + 1, d, dtype=np.int8), np.zeros(n_ + 1), BACKOFF_M, 0,
                )
                # The leg's own tracker config survives repeated backoffs.
                config0 = obstacle.get("backoff", {}).get("config", trackers[ms.phase].config)
                obstacle["backoff"] = {"phase": ms.phase, "config": config0, "time_s": t, "leg_dir": leg_dir}
                obstacle["replans"][-1]["backoff"] = True
                obstacle["replan_wait"].pop(ms.phase, None)
                paths[ms.phase] = back
                trackers[ms.phase] = RearAxlePathTracker(
                    back.poses, back.directions, back.curvatures_inv_m,
                    replace(config0, cruise_speed_mps=BACKOFF_SPEED_MPS),
                )
                ms.phase_started = t  # the backoff's own tracking timeout
                if ms.phase == "observe":
                    realign_install("backoff")  # Plan D8 S4c-3 stage 2
        elif same_path(replanned, paths[ms.phase], trackers[ms.phase].remaining_to_goal_m()):
            # D4: the same path again is a wait, not a retry -- it
            # neither restarts the tracker nor the no-progress watch
            # and does not count against the retry limits (Codex P2).
            obstacle["replans"][-1]["same_path"] = True
            obstacle.setdefault("replan_wait", {}).pop(ms.phase, None)
            requested_speed = 0.0
        else:
            # D4: a replan may rightly be longer (a detour), so the
            # no-progress watch restarts on the new path; the
            # oscillation it also cut is bounded by a per-leg replan
            # count (L3c v34 seed 4: two good detours tripped the 60 s watch).
            # Checked once the path is known to differ: a same-path wait
            # never counts (Codex checkpoint 6 P2).
            recent = [r for r in obstacle["replans"]
                      if r["phase"] == ms.phase and self.commands.work_elapsed(r["time_s"], t) < 30.0
                      and not r.get("same_path") and not r.get("failed") and not r.get("escape")]
            require(len(recent) <= 3, f"obstacle_blocked in {ms.phase}: {len(recent)} replans in 30 s")
            leg_replans = sum(1 for r in obstacle["replans"] if r["phase"] == ms.phase and not r.get("same_path")
                              and not r.get("failed") and not r.get("escape"))
            require(leg_replans <= 10, f"obstacle_replan_limit in {ms.phase}: {leg_replans}")
            obstacle.setdefault("progress", {}).clear()
            obstacle.setdefault("replan_wait", {}).pop(ms.phase, None)
            paths[ms.phase] = replanned
            state.setdefault("paths", {})[ms.phase] = path_record(replanned)
            hooks.write_paths()
            config_ = trackers[ms.phase].config
            if obstacle.get("backoff", {}).get("phase") == ms.phase:
                config_ = obstacle.pop("backoff")["config"]
            trackers[ms.phase] = RearAxlePathTracker(
                replanned.poses, replanned.directions, replanned.curvatures_inv_m, config_
            )
            if ms.phase == "observe":
                realign_install("obstacle_replan")  # Plan D8 S4c-3 stage 2
            if ms.phase == "transport" and slam is not None and not slam["docking"].get("reapproach"):
                arm_docking()
            ms.phase_started = t
            requested_speed = 0.0
        return SimpleNamespace(rear=rear, requested_speed=requested_speed)

    def follow(self, t: float, rear, signed_speed: float, dt: float) -> SimpleNamespace:
        """The follower's tick: the time limit, the tracker update and the rules that rewrite its result
        before anything else reads it -- the cusp heading holds' records, a re-observation stand, an
        escape's end, the cusp position stall (``cusp_tick``) and the SLAM hold near a final goal. Returns
        the rewritten ``tracking`` and ``cusp_hold``."""
        ms, state, obstacle, slam, hooks, options, near = (self.ms, self.state, self.obstacle, self.slam, self.hooks,
                                                          self.options, self.near)
        trackers, settings, core = self.trackers, self.settings, self
        require, dump_tracking, cusp_heading_end, cusp_heading_offset, cusp_tick = (
            hooks.require, hooks.dump_tracking, self.cusp_heading_end, self.cusp_heading_offset, self.cusp_tick)
        slam_odom_base, slam_error = hooks.slam_odom_base, hooks.slam_error
        cusp_holds, CUSP_HEADING_HOLD_PHASES = hooks.cusp_holds(), options.cusp_heading_hold_phases
        self.scene_raised = False
        limit = core.tracking_limit(t)  # M-a1 slice 2
        if self.commands.work_elapsed(ms.phase_started, t) >= limit:
            dump_tracking("timeout", ms.last_tracking)
        require(self.commands.work_elapsed(ms.phase_started, t) < limit, f"Tracking timeout in {ms.phase}")
        tracking = trackers[ms.phase].update(rear, signed_speed, dt)
        raw_tracking_ = tracking  # the tracker's own result, before any rule below rewrites it
        # Fix A / A' records (cusp_heading_hold): every tracker tick, on the tracker's own
        # result before any rule below rewrites it or replaces the tracker (Codex A' design
        # review 2 P2: a return goal failure was taken by the stall replan first and the
        # watch read "replaced"), a hold or watch of another tracker or phase ends; then the
        # released leg's watch -- when it first moves, its largest lateral and heading offset
        # from the path, and how it ends (ahead of the hold step, so a leg held again at its
        # end counts that tick -- Codex fix A review 2 P3).
        cusp_holds.sync(t, ms.phase, id(trackers[ms.phase]), trackers[ms.phase].leg_index, cusp_heading_end(tracking))
        if ms.phase in CUSP_HEADING_HOLD_PHASES and state["cusp_heading_watch"] is not None:
            cusp_holds.tick(
                t, ms.phase, id(trackers[ms.phase]), trackers[ms.phase].leg_index, cusp_heading_end(tracking),
                tracking.cusp_pending, tracking.speed_mps,
                cusp_heading_offset(trackers[ms.phase], tracking.progress_m, rear)
                if id(trackers[ms.phase]) == state["cusp_heading_watch"]["tracker"] else None)
        retry_stand_ = near is not None and near.get("retry_stand") and ms.phase == "observe"
        if retry_stand_:
            # Plan D8 S4c-2: standing to re-observe -- no follower command, no hold and no
            # observe recovery: the capture follows at once.
            tracking = replace(tracking, status="arrived", speed_mps=0.0)
        escape_now_ = (
            obstacle is not None and obstacle.get("backoff", {}).get("phase") == ms.phase
            and obstacle["backoff"].get("escape")
        )
        if escape_now_:
            # Obstacle recovery v4 E3: an escape's end -- arrived, an endpoint failure, standing
            # at its end, a 120-tick stand on the way -- is the backoff end, never a docking
            # stop, a stall replan or a tracking failure; it stays so until a replan replaces it.
            back_e = obstacle["backoff"]
            standing_e = slam["stop_now"] if slam is not None else abs(signed_speed) < 0.01
            back_e["stand_ticks"] = back_e.get("stand_ticks", 0) + 1 if standing_e else 0
            if (
                "end_s" in back_e or tracking.status == "arrived"
                or (standing_e and (tracking.status == "failed" or trackers[ms.phase].remaining_to_goal_m() <= 1e-6))
                or back_e["stand_ticks"] >= 120
            ):
                tracking = replace(tracking, status="arrived", speed_mps=0.0)
            elif tracking.status == "failed":
                # An endpoint failure while still rolling: brake and take the end standing.
                tracking = replace(tracking, status="braking", speed_mps=0.0)
        # Plan cusp position stall v7: a leg standing at its cusp end outside the cusp tolerance
        # is held at zero, then started from the measured pose or handed on (K2/K3).
        tracking, cusp_hold = cusp_tick(t, rear, raw_tracking_, tracking, retry_stand_)
        ms.last_tracking = tracking
        if self.scene_raised:
            # Scene-mode replan plan 2.1 (Codex round 1 P2-1): the request's tick stands -- no SLAM hold below, no
            # stall, recovery, command or arrival after this in the runner; the device takes the next tick.
            return SimpleNamespace(tracking=tracking, cusp_hold=True, scene_raised=True)
        if (
            slam is not None
            and not retry_stand_
            and ms.phase in ("observe", "approach", "transport", "return_home")
            and slam["tracker"].mode == "tracking"
            and trackers[ms.phase].remaining_to_goal_m()
            <= 0.5 + signed_speed**2 / (2 * settings["drive_acceleration_mps2"])
        ):
            # The window grows with the braking distance (Codex v3.5 P2:
            # a correction at 0.507 m and 0.55 m/s left no room to stop).
            # A final goal is judged at 8 mm:
            # a SLAM correction landing in the last half metre moves
            # the estimate by centimetres and leaves the truck stopped
            # outside the tolerance (S2 seed 1 on v3.4, transport).
            # Freeze it there (plan v3.5; approach already holds from
            # the capture). Released at settle as before.
            self.hold_slam(t, held_frame_pallet=False)
            slam["holds"].append(
                {"phase": ms.phase, "time_s": t, "event": "hold", "error": slam_error()}
            )
        return SimpleNamespace(tracking=tracking, cusp_hold=cusp_hold, scene_raised=False)

    def docking_and_stall(self, t: float, rear, tracking, signed_speed: float, dt: float, loaded: bool) -> SimpleNamespace:
        """After the follower's rules: the D7c docking stop at the delivery straight (a), the final goal
        missed in heading under SLAM, and the stall at a path end -- a docking retry (c)/(d) when D7 owns
        the transport, else a SLAM stall replan of the leg (twice at most). Returns ``tracking``, ``rear``,
        ``d7_docking_`` and ``d7_owned`` (the cusp rule after it reads the last two)."""
        ms, state, obstacle, slam, hooks, options = self.ms, self.state, self.obstacle, self.slam, self.hooks, self.options
        paths, trackers, geometry = self.paths, self.trackers, self.geometry
        planner_config, travel_config, scenario = options.planner_config, options.travel_config, options.scenario
        return_to_pose = options.return_to_pose
        require, wall, path_stats, path_record = hooks.require, hooks.wall, hooks.path_stats, hooks.path_record
        dock_at_delivery_straight, start_docking_retry, slam_rear, slam_odom_base, arm_docking = (
            self.dock_at_delivery_straight, self.start_docking_retry, hooks.slam_rear, hooks.slam_odom_base,
            self.arm_docking)
        grid_world, grid_kwargs, plan_transport_leg, plan_return_leg = (
            hooks.grid_world, hooks.grid_kwargs, hooks.plan_transport_leg, hooks.plan_return_leg)
        # The final goal judged out of heading tolerance under SLAM:
        # brake to a stop and replan from there rather than end the
        # mission (S3 seed 3 failed its return at -0.054 rad vs 0.03).
        d7_docking_ = slam["docking"] if (options.d7_docking and slam is not None) else {}
        # Plan D7c: a docked straight or a re-approach is recovered by a
        # docking retry, judged before the stall/cusp budgets (Codex D7c P2).
        d7_owned = ms.phase == "transport" and bool(
            d7_docking_.get("accepted_any") or d7_docking_.get("reapproach")
        )
        dock_stop = (
            ms.phase == "transport"
            and d7_docking_.get("status") == "armed"
            and slam["tracker"].mode == "holding"
            and not tracking.at_cusp
            and not tracking.off_path
            and (
                (tracking.status == "failed" and tracking.failure == "endpoint_heading")
                or (tracking.status == "tracking" and trackers[ms.phase].remaining_to_goal_m() <= 1e-6)
            )
        )
        if dock_stop:
            # Plan D7c (a): stopped at the docking stop outside its tolerance
            # (0.03 m, 0.05 rad) -- dock here, as an arrival would, rather
            # than a stall replan (l8_measured seed 1 + N1: 37.5 mm lateral).
            tracking = replace(tracking, status="braking", speed_mps=0.0)
            ms.dock_stop_ticks = ms.dock_stop_ticks + 1 if slam["stop_now"] else 0
            if ms.dock_stop_ticks >= 120:
                ms.dock_stop_ticks = 0
                state.setdefault("docking_retries", []).append(
                    {"trigger": "a", "time_s": t, "counted": False, "round": d7_docking_.get("round", 1),
                     "reapproach": bool(d7_docking_.get("reapproach")),
                     "position_error_m": tracking.position_error_m, "yaw_error_rad": tracking.yaw_error_rad}
                )
                dock_at_delivery_straight(t)
                if not (self.jobs is not None and self.job_pending()):  # D9b: else the match is asked; it stands
                    rear = slam_rear(t)  # a retry may have released the correction
                    hooks.set_rear(rear)  # the runner's closures (grid_kwargs, ...) read its rear (Codex slice-3 P2)
                    slam["control"][-1][1:4] = rear.tolist()
                    tracking = trackers[ms.phase].update(rear, signed_speed, dt)
                    ms.last_tracking = tracking
        else:
            ms.dock_stop_ticks = 0
        goal_heading_miss = (
            slam is not None
            and not dock_stop
            and ms.phase in ("transport", "return_home")
            and tracking.status == "failed"
            and tracking.failure == "endpoint_heading"
            and not tracking.at_cusp
            and not tracking.off_path
            and (len(state["stall_replans"]) < 2 or d7_owned)
        )
        if goal_heading_miss:
            # Judged inside the 8 mm brake window, still creeping
            # (S3 rerun: 6.6 mm left at -0.008 m/s): command zero and
            # wait for the stop detector.
            tracking = replace(tracking, status="braking", speed_mps=0.0)
        stalled = (
            slam is not None
            and ms.phase in ("transport", "return_home")
            and tracking.speed_mps == 0.0
            and slam["stop_now"]
            and (
                goal_heading_miss
                or (
                    tracking.status == "tracking"
                    and trackers[ms.phase].remaining_to_goal_m() <= 1e-6
                )
                # Plan D7c (d): a re-approach stopped at a cusp short of its
                # tolerance (status tracking, no failure -- Codex D7c impl P2-5).
                or (
                    bool(d7_docking_.get("reapproach"))
                    and tracking.status == "tracking"
                    and tracking.at_cusp
                )
            )
        )
        if stalled and goal_heading_miss:
            ms.slam_stall_ticks = max(ms.slam_stall_ticks, 119)
        ms.slam_stall_ticks = ms.slam_stall_ticks + 1 if stalled else 0
        if ms.slam_stall_ticks >= 120 and d7_owned:
            # Plan D7c (c)/(d): after a docked straight or during a
            # re-approach, a retry instead of transport_recovery_after_docking.
            ms.slam_stall_ticks = 0
            start_docking_retry(
                t, "d" if d7_docking_.get("reapproach") else "c", "transport_recovery_after_docking"
            )
            rear = slam_rear(t)
            hooks.set_rear(rear)  # the runner's closures (grid_kwargs, ...) read its rear (Codex slice-3 P2)
            slam["control"][-1][1:4] = rear.tolist()
            tracking = trackers[ms.phase].update(rear, signed_speed, dt)
            ms.last_tracking = tracking
        elif ms.slam_stall_ticks >= 120 and len(state["stall_replans"]) < 2:
            # Stopped at the end of the path but outside the goal
            # tolerance (an estimate shift before the hold): plan the
            # leg again from here, in the held frame (Codex v3.5 P2).
            ms.slam_stall_ticks = 0
            start = PlanningPose(float(rear[0]), float(rear[1]), float(rear[2]))
            replan_start = wall()
            # After a docking match the transport is held relative to the
            # station: a Hybrid A* recovery there would drive a held
            # detour in a frame mixed with world obstacles (Codex 5de6c3a
            # P2) -- end the run instead.
            require(
                not (ms.phase == "transport" and slam["docking"].get("accepted_any")),
                "transport_recovery_after_docking",
            )
            if slam["tracker"].mode == "holding":
                # No match is held against (the docking fell back): a
                # recovery planned now may be a long manoeuvre -- a yaw
                # fix at a 2 m radius is a full loop -- and must not run
                # on dead reckoning (L3c v22 seed 1: 15 m held). The truck
                # stands still: let SLAM back in first, as at a capture.
                jump_m, jump_rad = self.release_slam(slam_odom_base())
                slam["holds"].append(
                    {"phase": ms.phase, "time_s": t, "event": "release_before_stall_replan",
                     "jump_m": jump_m, "jump_rad": jump_rad}
                )
                rear_now = slam_rear(hooks.now_s())
                if obstacle is not None and slam["tracker"].applied is not None:
                    # The new correction re-projects the grid before any
                    # plan reads it, and this tick goes on from the new
                    # pose (Codex checkpoint P1: no mixed frames).
                    obstacle["applied"] = tuple(float(v) for v in slam["tracker"].applied[0])
                    obstacle["version"] += 1
                    obstacle["reprojections"] = obstacle.get("reprojections", 0) + 1
                    obstacle["layer"].refresh(
                        t, obstacle["applied"], obstacle["version"],
                        current_pose=tuple(float(v) for v in rear_now),
                        path_ahead=np.array([rear_now]), loaded=loaded,
                    )
                rear = np.asarray(rear_now, dtype=float)
                hooks.set_rear(rear)  # the runner's closures (grid_kwargs, ...) read its rear (Codex slice-3 P2)
                start = PlanningPose(float(rear_now[0]), float(rear_now[1]), float(rear_now[2]))
            if ms.phase == "transport":
                # After docking, the corrected drop (v3.8, Codex P1).
                replanned = plan_transport_leg(
                    grid_world(slam.get("transport_scenario", scenario)), start, planner_config,
                    geometry=geometry, travel_config=travel_config,
                    **grid_kwargs(None),
                )
            else:
                replanned = plan_return_leg(
                    grid_world(slam.get("return_scenario", scenario)), start, return_to_pose, planner_config,
                    geometry=geometry, travel_config=travel_config,
                    **grid_kwargs(None),
                )
            state["stall_replans"].append(
                {
                    "phase": ms.phase,
                    "time_s": t,
                    "rear_pose": rear.tolist(),
                    "position_error_m": tracking.position_error_m,
                    "yaw_error_rad": tracking.yaw_error_rad,
                    "status": replanned.status,
                    "planning_wall_s": wall() - replan_start,
                    **path_stats(replanned),
                }
            )
            require(replanned.success, f"stall_replan_failed:{replanned.status}")
            paths[ms.phase] = replanned
            state.setdefault("paths", {})[ms.phase] = path_record(replanned)
            hooks.write_paths()
            hooks.show_path(
                replanned,
                "Transport" if ms.phase == "transport" else "Return",
                (1.0, 0.65, 0.04) if ms.phase == "transport" else (0.55, 0.2, 0.85),
            )
            stall_config = trackers[ms.phase].config
            if obstacle is not None and obstacle.get("backoff", {}).get("phase") == ms.phase:
                # A replan accepted here ends a backoff too (Codex checkpoint 8 P2).
                stall_config = obstacle.pop("backoff")["config"]
                obstacle.setdefault("replan_wait", {}).pop(ms.phase, None)
            trackers[ms.phase] = RearAxlePathTracker(
                replanned.poses,
                replanned.directions,
                replanned.curvatures_inv_m,
                stall_config,
            )
            if ms.phase == "transport":
                arm_docking()  # keep the stop before the delivery straight
            if obstacle is not None:
                # The no-progress watch compares remaining lengths on one path:
                # a replaced path starts its own baseline next tick, after
                # arm_docking, as obstacle replans already do (plan D7d -- a
                # stall replan's 23 m loop was judged against the old path's
                # near-zero remainder, l8_measured seed 1 + N1).
                obstacle.setdefault("progress", {}).clear()
            ms.phase_started = t
            tracking = trackers[ms.phase].update(rear, signed_speed, dt)
            ms.last_tracking = tracking
        return SimpleNamespace(tracking=tracking, rear=rear, d7_docking_=d7_docking_, d7_owned=d7_owned)

    def cusp_and_observe_recovery(self, t: float, rear, tracking, signed_speed: float, dt: float, d7_docking_,
                                  d7_owned: bool, step: int) -> SimpleNamespace:
        """The recoveries at a cusp and on the observe leg: the transport's cusp heading miss (a docking
        retry under D7, else a cusp replan once stopped), the observe/return cusp heading hold (fix A / A':
        judge the next leg from here and release, or latch the failure), and the observe leg's heading miss
        or cusp stall under SLAM (an observe replan, twice per leg). Returns ``tracking`` and ``rear``."""
        ms, state, obstacle, slam, hooks, options = self.ms, self.state, self.obstacle, self.slam, self.hooks, self.options
        paths, trackers, geometry = self.paths, self.trackers, self.geometry
        planner_config, travel_config, scenario = options.planner_config, options.travel_config, options.scenario
        pickup_bounds = options.pickup_bounds
        max_cusp_replans, cusp_stop_needed, cusp_stop_yaw_rate_radps = (
            options.max_cusp_replans, options.cusp_stop_needed, options.cusp_stop_yaw_rate_radps)
        CUSP_HEADING_HOLD_PHASES, CUSP_HEADING_HOLD_POSITION_M, CUSP_HEADING_HOLD_RAD, OBSERVE_REPLANS_PER_LEG = (
            options.cusp_heading_hold_phases, options.cusp_heading_hold_position_m, options.cusp_heading_hold_rad,
            options.observe_replans_per_leg)
        require, wall, path_record, start_docking_retry, slam_rear, arm_docking = (
            hooks.require, hooks.wall, hooks.path_record, self.start_docking_retry, hooks.slam_rear, self.arm_docking)
        grid_world, grid_kwargs, plan_transport_leg, plan_observation_leg = (
            hooks.grid_world, hooks.grid_kwargs, hooks.plan_transport_leg, hooks.plan_observation_leg)
        realign_active, realign_install = self.realign_active, self.realign_install
        cusp_holds, cusp_monitor = hooks.cusp_holds(), hooks.cusp_monitor()
        cusp_heading_motion, cusp_heading_end, cusp_heading_offset = (
            hooks.cusp_heading_motion, self.cusp_heading_end, self.cusp_heading_offset)
        cusp_stopped, cusp_end_recovered, cusp_leg_checker, cusp_sensors = (
            self.cusp_stopped, self.cusp_end_recovered, self.cusp_leg_checker, self.cusp_sensors)
        if (
            tracking.status == "failed"
            and ms.phase == "transport"
            and tracking.failure == "endpoint_heading"
            and tracking.at_cusp
            and not tracking.off_path
            and (len(state["cusp_replans"]) < max_cusp_replans or d7_owned)
        ):
            stopped = (
                slam["stop_now"]
                if slam is not None
                else (
                    tracking.speed_mps == 0.0
                    and float(np.linalg.norm(hooks.truth_velocity()[:2]))
                    <= trackers[ms.phase].config.stop_speed_mps
                    and abs(float(hooks.truth_yaw_rate()))
                    <= cusp_stop_yaw_rate_radps
                )
            )
            ms.cusp_stop_ticks = ms.cusp_stop_ticks + 1 if stopped else 0
            if ms.cusp_stop_ticks < cusp_stop_needed:
                # The failed tracker already commands zero; let it stop.
                tracking = replace(tracking, status="braking")
            elif d7_owned:
                # Plan D7c (c)/(d): a cusp after a docked straight or inside a
                # re-approach is a docking retry, not a cusp replan.
                ms.cusp_stop_ticks = 0
                start_docking_retry(
                    t, "d" if d7_docking_.get("reapproach") else "c", "transport_recovery_after_docking"
                )
                rear = slam_rear(t)
                hooks.set_rear(rear)  # the runner's closures read its rear (Codex slice-3 P2)
                slam["control"][-1][1:4] = rear.tolist()
                tracking = trackers[ms.phase].update(rear, signed_speed, dt)
                ms.last_tracking = tracking
            else:
                ms.cusp_stop_ticks = 0
                require(
                    not (slam is not None and slam["docking"].get("accepted_any")),
                    "transport_recovery_after_docking",
                )
                replan_start = wall()
                replanned = plan_transport_leg(
                    grid_world(slam.get("transport_scenario", scenario)
                    if slam is not None
                    else scenario),
                    PlanningPose(float(rear[0]), float(rear[1]), float(rear[2])),
                    planner_config,
                    geometry=geometry,
                    travel_config=travel_config,
                    **grid_kwargs(None),
                )
                replan_wall_s = wall() - replan_start
                state["planning_wall_s"] = (
                    state.get("planning_wall_s") or 0.0
                ) + replan_wall_s
                state["cusp_replans"].append(
                    {
                        "time_s": t,
                        "loop_step": step,
                        "rear_pose": rear.tolist(),
                        "position_error_m": tracking.position_error_m,
                        "yaw_error_rad": tracking.yaw_error_rad,
                        "status": replanned.status,
                        "planning_wall_s": replan_wall_s,
                        "search_attempts": [
                            list(entry) for entry in replanned.search_attempts
                        ],
                        "path": path_record(replanned)
                        if replanned.success
                        else None,
                        "replaced_path": path_record(paths[ms.phase]),
                    }
                )
                if replanned.success:
                    paths[ms.phase] = replanned
                    # The active plan everywhere: record, file, display.
                    state.setdefault("paths", {})[ms.phase] = path_record(replanned)
                    hooks.write_paths()
                    hooks.show_path(replanned, "Transport", (1.0, 0.65, 0.04))
                    trackers[ms.phase] = RearAxlePathTracker(
                        replanned.poses,
                        replanned.directions,
                        replanned.curvatures_inv_m,
                        trackers[ms.phase].config,
                    )
                    if slam is not None:
                        arm_docking()
                    if obstacle is not None:
                        obstacle.setdefault("progress", {}).clear()  # plan D7d, as above
                    ms.phase_started = t
                    tracking = trackers[ms.phase].update(rear, signed_speed, dt)
                    ms.last_tracking = tracking
        # EPAL 6 fork spacing plan, fix A (observe) and A' (return): a cusp held for a
        # heading miss of 0.05-0.08 rad (the tracker's cusp_pending). Standing, re-check the
        # window, judge the next leg from here with this cusp's limits only, and release it;
        # anything else latches the tracker's failure for the existing path (the observe
        # replan below; a failed return ends the run as before).
        hold_tracker = trackers[ms.phase]
        if ms.phase in CUSP_HEADING_HOLD_PHASES and tracking.cusp_pending:
            hold = cusp_holds.held(
                t, [ms.phase, id(hold_tracker), hold_tracker.leg_index],
                {"rear": rear.tolist(), "position_error_m": tracking.position_error_m,
                 "yaw_error_rad": tracking.yaw_error_rad, **cusp_heading_motion(signed_speed)},
                cusp_heading_end(tracking))
            decision = None
            if slam is not None and slam["tracker"].mode == "holding" and realign_active():
                decision = "realign_held"  # no recovery in a held realignment (S4c-3)
            elif cusp_stopped():
                if not (tracking.position_error_m <= CUSP_HEADING_HOLD_POSITION_M
                        and hold_tracker.leg_remaining_m() <= hold_tracker.leg_end_window_m()):
                    decision = "window"
                else:
                    from forklift_core.control import escape as esc

                    judged = CUSP_STALL.judge(
                        hold_tracker, rear, tracking, {}, end_recovered=cusp_end_recovered,
                        sweep_free=lambda trajectory_: esc.rectangles_sweep_free(cusp_leg_checker(), trajectory_),
                        sensors=(lambda trajectory_, direction_, t_=t, rear_=rear: cusp_sensors(
                            t_, rear_, trajectory_, direction_)) if obstacle is not None else None,
                        bounds=(CUSP_HEADING_HOLD_POSITION_M, CUSP_HEADING_HOLD_RAD),
                    )
                    hold["judgement"] = judged.record
                    if not (judged.pose_ok and judged.sensor_ok and not judged.prefix_occupied):
                        decision = "judged_out"
                    elif hold_tracker.release_cusp(True):
                        decision = "released"
                    else:
                        decision = "release_refused"
            if decision is not None:
                cusp_holds.decide(
                    t, decision,
                    {"decided_rear": rear.tolist(), "decided_position_error_m": tracking.position_error_m,
                     "decided_yaw_error_rad": tracking.yaw_error_rad,
                     "decided": cusp_heading_motion(signed_speed)},
                    cusp_heading_end(tracking))
                if decision != "released":
                    hold_tracker.fail_pending()
                tracking = hold_tracker.update(rear, signed_speed, dt)
                ms.last_tracking = tracking
                if decision == "released":
                    # The new watch sees this tick's update too: the next leg's first command,
                    # or its end reached at once (Codex fix A implementation review 3 P2).
                    cusp_holds.tick(
                        t, ms.phase, id(hold_tracker), hold_tracker.leg_index, cusp_heading_end(tracking),
                        tracking.cusp_pending, tracking.speed_mps,
                        cusp_heading_offset(hold_tracker, tracking.progress_m, rear))
        # Under SLAM, an observe leg judged out of heading at a cusp or
        # its end (S2 seed 1: -0.078 rad at a cusp of the near leg, three
        # versions running): brake, then plan the leg again from the
        # stop to the same target, at most twice per run.
        # Also a stop at a cusp outside its tolerance (S3 seed 1: 44.5 mm,
        # command 0, status tracking until the timeout) once it has
        # stood for a second.
        # Plan cusp position stall v7: not while the stall handling holds an observe stall (its
        # sensor wait or hand-off) -- only once it has passed the stall to this rule.
        observe_stalled = (
            slam is not None
            and ms.phase == "observe"
            and tracking.status == "tracking"
            and tracking.at_cusp
            and tracking.speed_mps == 0.0
            and slam["stop_now"]
            and not cusp_monitor.holds_observe()
        )
        ms.observe_stall_ticks = ms.observe_stall_ticks + 1 if observe_stalled else 0
        if (
            slam is not None and ms.phase == "observe" and slam["tracker"].mode == "holding"
            and realign_active()
            and (
                (tracking.status == "failed" and tracking.failure == "endpoint_heading"
                 and not tracking.off_path)
                or ms.observe_stall_ticks >= 120
            )
        ):
            # Plan D8 S4c-3 stage 2: no recovery runs held in the realignment (the
            # S4c-2 failure's replan did) -- whatever its budget.
            require(False, "near_capture_realign_held_recovery")
        deck_miss_ = (
            # Plan 2026-10-19 task 4 (Codex implementation round 1 P2): without SLAM the observe replan below never
            # runs; a side leg's heading miss still ends its own way instead of failing the tracker.
            slam is None and self.bottom_deck_leg_active()
            and tracking.status == "failed" and tracking.failure == "endpoint_heading" and not tracking.off_path
        )
        if deck_miss_:
            tracking = replace(tracking, status="braking", speed_mps=0.0)
            # Stopped first (as the transport's cusp recovery without SLAM; Codex implementation round 2 P2): the
            # command stays 0 until the truck stands, then the face ends or the stand is planned again.
            stopped_ = (float(np.linalg.norm(hooks.truth_velocity()[:2])) <= trackers["observe"].config.stop_speed_mps
                        and abs(float(hooks.truth_yaw_rate())) <= cusp_stop_yaw_rate_radps)
            if stopped_:
                out_ = self.bottom_deck_recover(t, rear, "observe_miss")
                rear = out_.rear
                if out_.installed and ms.phase == "observe":
                    tracking = trackers["observe"].update(rear, signed_speed, dt)  # the new leg's, not the old one
                    ms.last_tracking = tracking
        observe_stall_ = (
            slam is not None
            and ms.phase == "observe"
            and (
                (
                    tracking.status == "failed"
                    and tracking.failure == "endpoint_heading"
                    and not tracking.off_path
                )
                or ms.observe_stall_ticks >= 120
            )
        )
        if observe_stall_ and slam["stop_now"] and self.scene_far_observe():
            # Scene-mode replan plan 2.3 (B): the first far observation's stall tries the remaining observation
            # candidates before the same waypoint is planned again.
            took_ = self.scene_observe_candidate(t, rear)
            rear = took_.rear  # released or not, the pose every later plan of this tick starts from
            if took_.installed:
                ms.observe_stall_ticks = 0
                tracking = trackers["observe"].update(rear, signed_speed, dt)
                ms.last_tracking = tracking
                return SimpleNamespace(tracking=tracking, rear=rear)
            if took_.tried:  # one candidate planned this tick (live keeps its tick short): the next on the next
                return SimpleNamespace(tracking=replace(tracking, status="braking", speed_mps=0.0), rear=rear)
        observe_miss = (
            observe_stall_
            and len(state["observe_replans"]) - state["observe_replan_leg"]["start"]
            < OBSERVE_REPLANS_PER_LEG
        )
        if observe_stall_ and not observe_miss and slam["stop_now"] and self.scene_far_observe():
            # B: no candidate left and the waypoint's own budget spent -- end it, rather than stand to the timeout.
            self.state.setdefault("scene_replans", []).append(
                {"time_s": float(t), "trigger": "observe_stall", "phase": "observe", "outcome": "exhausted",
                 "rear": [float(v) for v in rear], "candidate_index": ms.next_candidate_index})
            require(False, "observe:stall_exhausted")
        if observe_miss:
            ms.observe_stall_ticks = 0
            tracking = replace(tracking, status="braking", speed_mps=0.0)
            if slam["stop_now"] and self.bottom_deck_leg_active():
                # Plan 2026-10-19 task 4: released before the replan; a retreat makes no detour (design rounds 4-5).
                out_ = self.bottom_deck_recover(t, rear, "observe_miss")
                rear = out_.rear
                if out_.installed and ms.phase == "observe":
                    tracking = trackers["observe"].update(rear, signed_speed, dt)  # the new leg's, not the old one
                    ms.last_tracking = tracking
            elif slam["stop_now"]:
                target = PlanningPose(*(float(v) for v in paths["observe"].poses[-1]))
                replan_start = wall()
                replanned = plan_observation_leg(
                    grid_world(scenario),
                    target,
                    planner_config,
                    geometry=geometry,
                    start_rear=PlanningPose(float(rear[0]), float(rear[1]), float(rear[2])),
                    pickup_bounds=self.observe_bounds(),
                    extended=False,
                    **grid_kwargs("observe"),
                )
                leg_ = state["observe_replan_leg"]
                state["observe_replans"].append(
                    {
                        "time_s": t,
                        "rear_pose": rear.tolist(),
                        "yaw_error_rad": ms.last_tracking.yaw_error_rad,
                        "status": replanned.status,
                        "planning_wall_s": wall() - replan_start,
                        "leg_serial": leg_["serial"],
                        "budget_left": OBSERVE_REPLANS_PER_LEG
                        - (len(state["observe_replans"]) + 1 - leg_["start"]),
                    }
                )
                require(replanned.success, f"observe_replan_failed:{replanned.status}")
                paths["observe"] = replanned
                state.setdefault("paths", {})["observe"] = path_record(replanned)
                hooks.write_paths()
                hooks.show_path(replanned, "Observe", (0.2, 0.8, 0.8))
                trackers["observe"] = RearAxlePathTracker(
                    replanned.poses,
                    replanned.directions,
                    replanned.curvatures_inv_m,
                    trackers["observe"].config,
                )
                ms.phase_started = t
                realign_install("observe_replan")  # Plan D8 S4c-3 stage 2
                tracking = trackers["observe"].update(rear, signed_speed, dt)
                ms.last_tracking = tracking
        return SimpleNamespace(tracking=tracking, rear=rear)

    def command(self, t: float, rear, rear_stamp: float, tracking, signed_speed: float, dwell_hold: bool,
                dwell_standing: bool, safety_hold: bool) -> SimpleNamespace:
        """The follower's command and what stops it before the arrival: a failed tracker ends the run, SLAM
        warming holds still, the D8b dwells and the scheduled safety stops on the approach straight, the
        near-field stop (and its carriage-corner insertion end), a loose observe arrival standing within
        its tolerance, and a backoff's end (stand, release, force the replan). Returns ``requested_speed``,
        ``curvature``, ``tracking``, ``rear`` and the three hold flags."""
        ms, state, obstacle, slam, hooks, options, near = (self.ms, self.state, self.obstacle, self.slam, self.hooks,
                                                          self.options, self.near)
        trackers = self.trackers
        dwells, safety_stops, dwell_d_end_m = options.dwells, options.safety_stops, options.dwell_d_end_m
        OBSERVE_ARRIVAL_M, OBSERVE_ARRIVAL_YAW_RAD = options.observe_arrival_m, options.observe_arrival_yaw_rad
        require, dump_tracking, near_tick, release_after_backoff = (
            hooks.require, hooks.dump_tracking, self.near_tick, self.release_after_backoff)
        if tracking.status == "failed":
            dump_tracking("failed", tracking)
        require(
            tracking.status != "failed",
            f"Tracking failed in {ms.phase}: pos={tracking.position_error_m:.4f},yaw={tracking.yaw_error_rad:.4f}",
        )
        requested_speed, curvature = (
            tracking.speed_mps,
            tracking.curvature_inv_m,
        )
        if slam is not None and not slam_may_drive(slam):
            requested_speed = 0.0  # warming up (or no gyro bias yet): hold still
        if (
            dwells is not None and ms.phase == "approach"
            and state.get("near_capture", {}).get("status") == "done"
        ):
            # Plan D8b calibration dwells (recording runs only): stand at the
            # straight's start, at each camera-face gap and after arriving.
            hold_, released_ = dwells.update(
                t, d_est_m=trackers["approach"].remaining_to_goal_m() + dwell_d_end_m,
                still=bool(slam["stop_now"]), arrived=tracking.status == "arrived",
            )
            if hold_:
                requested_speed = 0.0
                # Steering held too: turning at standstill shifted the body
                # 20 um per 0.1 s through a whole stand (matrix 1883 seed 3,
                # 0.18-0.25 rad of steering in every start dwell).
                dwell_hold = True
                # The wheels lock only once the truck has stopped: locked at the
                # trigger, while still braking, they sprang the body back ~0.8 mm
                # (diagnostic 1984).
                dwell_standing = dwells.active is not None and dwells.active["still_since_s"] is not None
                if tracking.status == "arrived":
                    tracking = replace(tracking, status="tracking", speed_mps=0.0)
            elif released_:
                # Accelerate from the stop, not from the slew's held cruise.
                trackers["approach"].restart_speed_slew(0.0)
                requested_speed = 0.0
        if (
            safety_stops is not None and ms.phase == "approach"
            and state.get("near_capture", {}).get("status") == "done"
        ):
            # Plan D8 S4a-1 ⓪: the near-field tracker's safety stop, scheduled --
            # zero command, steering held (the dwell path), no wheel lock.
            hold_, released_ = safety_stops.update(
                t, d_est_m=trackers["approach"].remaining_to_goal_m() + dwell_d_end_m,
                still=bool(slam["stop_now"]),
            )
            if hold_:
                requested_speed = 0.0
                dwell_hold = True
                # A safety stop is a step to zero at the wheels, as the permission's
                # cap and the e-stop are (the stop the stopping model measured), not
                # the slewed braking of a dwell (Codex S4a-1 ⓪ review P1).
                safety_hold = True
                if tracking.status == "arrived":
                    tracking = replace(tracking, status="tracking", speed_mps=0.0)
            elif released_:
                trackers["approach"].restart_speed_slew(0.0)
                requested_speed = 0.0
        if near is not None and near["tracker"] is not None and ms.phase in ("approach", "insert"):
            # Plan D8 S4a ④: the near-field stop, before this tick's arrival --
            # zero command, steering held (the safety stop the drive terms were
            # measured with), arrival deferred; a restart only from standing.
            hold_, released_, arrive_ = near_tick(t, rear_stamp, rear, requested_speed, ms.phase, curvature,
                                                  tracking.status == "arrived")
            if arrive_:
                # Plan D8 S4a-2 ③: the carriage-corner insertion end, standing: the
                # insertion's own arrival processing (insert -> lift) follows.
                requested_speed = 0.0
                dwell_hold = True
                safety_hold = True
                tracking = replace(tracking, status="arrived", speed_mps=0.0)
            elif hold_:
                requested_speed = 0.0
                dwell_hold = True
                safety_hold = True
                if tracking.status == "arrived":
                    tracking = replace(tracking, status="tracking", speed_mps=0.0)
            elif released_:
                trackers[ms.phase].restart_speed_slew(0.0)
                requested_speed = 0.0
        if (
            ms.phase == "observe" and tracking.status == "tracking"
            and trackers[ms.phase].remaining_to_goal_m() <= 1e-9 and abs(tracking.speed_mps) < 1e-12
            and (slam["stop_now"] if slam is not None else abs(signed_speed) < 0.01)
            and tracking.position_error_m <= OBSERVE_ARRIVAL_M
            and abs(tracking.yaw_error_rad) <= OBSERVE_ARRIVAL_YAW_RAD
        ):
            # The tracker stops at a path end it missed sideways and never
            # arrives (L3c v38 seed 3: 6.1 cm off an arc's end). An
            # observation waypoint is a viewpoint, not a docking pose:
            # perception uses the pose the truck stands at. Standing 1 s
            # within OBSERVE_ARRIVAL_M is arrival there.
            loose = state.setdefault("observe_loose_arrival", {})
            loose.setdefault("since_s", t)
            if self.commands.work_elapsed(loose["since_s"], t) >= 1.0:
                state.setdefault("observe_loose_arrivals", []).append(
                    {"time_s": t, "position_error_m": tracking.position_error_m,
                     "yaw_error_rad": tracking.yaw_error_rad}
                )
                state.pop("observe_loose_arrival", None)
                tracking = replace(tracking, status="arrived")
        else:
            state.pop("observe_loose_arrival", None)
        if (
            tracking.status == "arrived" and obstacle is not None
            and obstacle.get("backoff", {}).get("phase") == ms.phase
        ):
            # The end of a backoff is not the leg's arrival: stand and
            # replan the leg from here, each second until a replan is
            # accepted (D4 delta).
            back_ = obstacle["backoff"]
            if "end_s" not in back_:
                back_["end_s"] = t
                obstacle.setdefault("backoff_records", []).append(
                    {"phase": ms.phase, "start_s": back_["time_s"], "end_s": t,
                     "rear": [float(v) for v in rear], "escape": bool(back_.get("escape"))}
                )
            # Plan v4 E3: standing first -- then the hold released and the replan forced.
            standing_b = slam["stop_now"] if slam is not None else abs(signed_speed) < 0.01
            if standing_b and not back_.get("released"):
                back_["released"] = True
                # The stop's cause, read before the release refreshes the permission, for the
                # forced replan's record (Codex implementation review 2 P2).
                perm_b = obstacle["layer"].permission
                back_["stop_cause"] = {
                    "path": getattr(perm_b.path_check, "blocked", None),
                    "estop": getattr(getattr(perm_b, "last_estop", None), "blocked", None),
                }
                released_b = release_after_backoff(t, "release_after_backoff")
                if released_b is not None:
                    rear = released_b
                    hooks.set_rear(rear)  # the runner's closures read its rear (Codex slice-3 P2)
            if standing_b and self.commands.work_elapsed(back_.get("forced_s", -1e9), t) >= 1.0:
                back_["forced_s"] = t
                obstacle["force_replan"] = ms.phase
            tracking = replace(tracking, status="tracking", speed_mps=0.0)
            requested_speed = 0.0
        return SimpleNamespace(requested_speed=requested_speed, curvature=curvature, tracking=tracking, rear=rear,
                               dwell_hold=dwell_hold, dwell_standing=dwell_standing, safety_hold=safety_hold)

    def observe_failed(self, t: float, rear, attempt: dict, attempt_number: int, observation) -> bool:
        """An observation that did not detect the pallet: a near capture fails outright (no fallback to the
        far estimate); otherwise the remaining observation candidates are planned from here and the first
        reachable one installed as the observe leg (all exhausted ends the run, or the repeat-capture
        diagnostic). True ends the mission loop (the repeat diagnostic's target not reached)."""
        ms, state, hooks, options = self.ms, self.state, self.hooks, self.options
        paths, trackers, geometry, settings, transition = self.paths, self.trackers, self.geometry, self.settings, self.transition
        planner_config, scenario, pickup_bounds, speeds = (options.planner_config, options.scenario, options.pickup_bounds,
                                                           options.speeds)
        CUSP_HEADING_HOLD_RAD, Pose2D = options.cusp_heading_hold_rad, PlanningPose
        require, wall, grid_world, grid_kwargs, plan_observation_leg = (
            hooks.require, hooks.wall, hooks.grid_world, hooks.grid_kwargs, hooks.plan_observation_leg)
        if self.bottom_deck_returning():
            return self._bottom_deck_reobserve_failed(t, rear, attempt, attempt_number, observation)
        far_ = attempt.get("recognition") or {}
        decided_ = str((far_.get("decided") or {}).get("hypothesis") or "")
        if far_.get("status") == "not_pickable" and "fork_spacing_adjust" in (
                list(far_.get("reasons") or []) + [far_.get("reason")]) and decided_ and not decided_.startswith(
                "parametric"):
            # A known shape whose fixed openings do not fit the forks at this spacing (T11 at 0.3725 m, 2026-10-21):
            # another viewpoint changes neither, so the mission ends by name instead of trying the next observation
            # points until the obstacle watch stops it. A measured (parametric) fit may change with a better view and
            # looks again as before (Codex T11 review P2).
            require(False, "not_pickable:fork_spacing_adjust")
        apply_tracker_profile, realign_install, record_tracker_configs = (
            self.apply_tracker_profile, self.realign_install, self.record_tracker_configs)
        attempt["retry_reason"] = (
            f"perception_{observation.status}:{observation.reason}"
        )
        if state.get("near_capture", {}).get("status") == "pending":
            # No silent fallback to the far estimate (v3.6).
            state["near_capture"]["status"] = "failed"
            require(
                False,
                f"near_capture_failed:{attempt['retry_reason']}",
            )
        hooks.set_inset(f"인식 실패 #{attempt_number} · 재관측 이동", f"사유: {observation.reason}")
        if self.jobs is not None and getattr(hooks, "live_abort", None) is not None and not options.repeat_captures:
            # A live run (plan 2026-10-22-live-far-recognition task 10): the remaining candidates are the planning
            # worker's -- the truck stands at the observation pose meanwhile and the ticks go on (D8b).
            self._observe_failed_request(t, rear, {"first": ms.next_candidate_index,
                                                   "retry_reason": attempt["retry_reason"]})
            return False
        planning_start = wall()
        observe_plan = None
        while ms.next_candidate_index < len(
            options.observation_waypoints
        ):
            candidate_index = ms.next_candidate_index
            ms.next_candidate_index += 1
            coordinates = options.observation_waypoints[
                candidate_index
            ]
            waypoint = Pose2D(*coordinates)
            candidate_plan = plan_observation_leg(
                grid_world(scenario),
                waypoint,
                planner_config,
                geometry=geometry,
                start_rear=Pose2D(rear[0], rear[1], rear[2]),
                pickup_bounds=self.observe_bounds(),
                # Re-observation keeps the earlier ladder.
                extended=False,
                **grid_kwargs("observe"),
            )
            state["observation_candidates"].append(
                {
                    "candidate_index": candidate_index,
                    "pose": list(coordinates),
                    "start_rear": rear.tolist(),
                    "success": candidate_plan.success,
                    "status": candidate_plan.status,
                    "analytic_expansion_interval": (
                        candidate_plan.analytic_expansion_interval
                    ),
                    "search_attempts": [
                        list(entry)
                        for entry in candidate_plan.search_attempts
                    ],
                }
            )
            if candidate_plan.success:
                observe_plan = candidate_plan
                state["observation_waypoint_selected"] = list(
                    coordinates
                )
                break
        state["planning_wall_s"] += (
            wall() - planning_start
        )
        if observe_plan is None and options.repeat_captures:
            state["repeat_capture"] = {
                "status": "repeat_target_not_reached",
                "attempts_made": attempt_number,
                "reason": "candidates_exhausted",
            }
            transition("repeat_target_not_reached", t)
            return True
        if observe_plan is None:
            state["planning_status"] = "all_candidates_exhausted"
            reasons = ";".join(
                candidate["status"]
                for candidate in state["observation_candidates"]
            )
            require(
                False,
                f"observe:all_candidates_exhausted:{attempt['retry_reason']}:{reasons}",
            )
        state["planning_status"] = observe_plan.status
        paths["observe"] = observe_plan
        trackers["observe"] = self.observe_tracker(observe_plan)
        ms.phase_started = t
        apply_tracker_profile()
        realign_install("observe_candidate")  # Plan D8 S4c-3 stage 2
        record_tracker_configs()
        return False

    def _observe_failed_request(self, t: float, rear, ctx: dict) -> None:
        """The candidates left after ``ctx["first"]`` asked of the worker from where the truck stands, on the near
        ladder only (as the synchronous re-observation), within 20 s each."""
        options, hooks = self.options, self.hooks
        waypoints = [tuple(float(v) for v in w) for w in options.observation_waypoints[ctx["first"]:]]
        grid = dict(hooks.grid_kwargs("observe"))
        grid.pop("deadline", None)
        world = hooks.grid_world(replace(options.scenario,
                                         start_rear=PlanningPose(float(rear[0]), float(rear[1]), float(rear[2]))))
        payload = {"world": world, "waypoints": waypoints, "config": options.planner_config,
                   "geometry": self.geometry, "pickup_bounds": self.observe_bounds(), "grid_kwargs": grid,
                   "ladders": (False,), "deadline": time.monotonic() + max(1, len(waypoints)) * 20.0}
        self._request("observe_failed", "observe_ladder", payload, t, rear, self._applied_now(), dict(ctx))

    def _observe_failed_finish(self, p, value, t: float, rear) -> None:
        """The worker's answer for the remaining candidates: the first reachable one installed as the observe leg
        (as the synchronous loop does), else the mission ends as it did (all candidates exhausted)."""
        ms, state, options = self.ms, self.state, self.options
        first = p.ctx["first"]
        for c in value["candidates"]:
            state["observation_candidates"].append(
                {"candidate_index": first + int(c["candidate_index"]), "pose": c["pose"], "start_rear": c["start_rear"],
                 "success": c["success"], "status": c["status"],
                 "analytic_expansion_interval": c.get("analytic_expansion_interval"),
                 "search_attempts": c.get("search_attempts", []), "extended_ladder": c.get("extended_ladder", False)})
        state["planning_wall_s"] += float(value.get("wall_s", 0.0))
        if value["result"] is None:
            ms.next_candidate_index = len(options.observation_waypoints)
            state["planning_status"] = "all_candidates_exhausted"
            reasons = ";".join(c["status"] for c in state["observation_candidates"])
            if value.get("timed_out"):
                reasons += ";timeout"
            self.hooks.require(False, f"observe:all_candidates_exhausted:{p.ctx['retry_reason']}:{reasons}")
        ms.next_candidate_index = first + int(value["selected"]) + 1
        state["observation_waypoint_selected"] = list(state["observation_candidates"][-1]["pose"])
        state["planning_status"] = value["result"].status
        self.paths["observe"] = value["result"]
        self.trackers["observe"] = self.observe_tracker(value["result"])
        ms.phase_started = t
        self.apply_tracker_profile()
        self.realign_install("observe_candidate")
        self.record_tracker_configs()

    def observe_tracker(self, plan):
        """The observation leg's follower, as the setup makes it (shared by the next candidate and a live
        mission's adoption, persistent console plan D2)."""
        speeds, settings, CUSP_HEADING_HOLD_RAD = self.options.speeds, self.settings, self.options.cusp_heading_hold_rad
        return RearAxlePathTracker(
            plan.poses,
            plan.directions,
            plan.curvatures_inv_m,
            TrackerConfig(
                cruise_speed_mps=speeds["observe"],
                max_curvature_inv_m=settings[
                    "tracker_curvature_inv_m"
                ],
                max_acceleration_mps2=settings[
                    "drive_acceleration_mps2"
                ],
                lookahead_m=0.28,
                position_tolerance_m=0.03,
                # Judged like a cusp in heading (P3, above).
                yaw_tolerance_rad=0.05,
                # Gear-change cusps are not goals (2026-09-26): the next leg
                # starts from the measured pose. Final goals keep the rules above.
                cusp_position_tolerance_m=0.03,
                cusp_yaw_tolerance_rad=0.05,
                # Brake and judge the cusp at 8 mm, still accepting 30 mm / 50 mrad
                # (docs/plans/2026-10-02-planner-tracker-robustness.md, P2).
                cusp_brake_window_m=0.008,
                # As the first observe tracker (fix A; Codex fix A implementation
                # review 1 P2: the candidate path dropped it).
                cusp_heading_pending_rad=CUSP_HEADING_HOLD_RAD,
                # Optional path speed caps; absent from the settings = off.
                max_lateral_acceleration_mps2=settings.get(
                    "max_lateral_acceleration_mps2"
                ),
                max_reverse_speed_mps=settings.get(
                    "max_reverse_speed_mps"
                ),
                overshoot_tolerance_m=0.03,
                stop_speed_mps=0.012,
                max_cross_track_error_m=0.35,
            ),
        )

    def observe_succeeded(self, t: float, rear, base, yaw: float, attempt: dict, attempt_number: int, observation,
                          scene_input, capture_diagnostics, step: int) -> SimpleNamespace:
        """A valid observation: the repeat diagnostic's early end, the near-field injection (S4c-1), the pallet
        site from the observation and the capture base (the runner's mixed sample, passed as it stands), the
        near stand and near-capture records, the evaluation records (the runner's, at the same point), the
        planning target, the known pallet, the mission plan and its followers, and the pending near capture
        of the first far observation under SLAM. Returns ``stop`` (the repeat diagnostic ends the loop),
        ``observation`` (biased under a mismatch injection) and ``near_leg``."""
        state, obstacle, pocket, hooks, options, near = (self.state, self.obstacle, self.pocket, self.hooks,
                                                         self.options, self.near)
        geometry, transition = self.geometry, self.transition
        planner_config, travel_config, scenario, pickup_bounds = (options.planner_config, options.travel_config,
                                                                  options.scenario, options.pickup_bounds)
        return_to_pose = options.return_to_pose
        adapter, near_tracking, Pose2D = options.adapter, options.near_tracking, PlanningPose
        wall, known_pallet = hooks.wall, self.known_pallet
        grid_world, grid_kwargs, plan_transport = hooks.grid_world, hooks.grid_kwargs, hooks.plan_transport
        if options.repeat_captures and options.repeat_at_attempt is not None:
            # G2r took another path to a valid detection
            # before the requested attempt: no batch, no plan.
            state["repeat_capture"] = {
                "status": "repeat_target_not_reached",
                "attempts_made": attempt_number,
                "reason": "valid_detection_before_target",
            }
            transition("repeat_target_not_reached", t)
            return SimpleNamespace(stop=True, observation=observation, near_leg=None)
        if options.extra_views:
            attempt["acceptance_simulation_time_s"] = (
                hooks.now_s()
            )
        state["perception"] = attempt.copy()
        # Plan D8 S4c-1: a capture before the first tracker is biased once --
        # correction: its target only; mismatch: its observation (so its target
        # and the tracker's start observation too).
        inject_ = (near is not None and options.near_inject is not None
                   and not near.get("injection_closed"))
        raw_observation = observation
        if inject_ and options.near_inject == "mismatch":
            observation = near_tracking.biased_observation(
                observation, geometry.pallet_depth_m,
                options.near_inject_lateral_m, options.near_inject_yaw_rad)
        base_xy = adapter.estimate_pallet_center_m(
            observation, geometry.pallet_depth_m
        )
        target_pickup = adapter.estimate_world_pallet_site(
            base_xy,
            observation.insertion_yaw_rad,
            (base[0], base[1]),
            yaw,
        )
        if inject_:
            raw_target = adapter.estimate_world_pallet_site(
                adapter.estimate_pallet_center_m(raw_observation, geometry.pallet_depth_m),
                raw_observation.insertion_yaw_rad, (base[0], base[1]), yaw)
            if options.near_inject == "correction":
                target_pickup = near_tracking.biased_site(
                    target_pickup, options.near_inject_lateral_m, options.near_inject_yaw_rad)
            state["near_tracking"]["injection"]["captures"].append({
                "attempt": attempt_number,
                "phase": "near" if state.get("near_capture", {}).get("status") == "pending" else "far",
                "time_s": t,
                "capture_base": [float(base[0]), float(base[1]), float(yaw)],
                "raw_target": [raw_target.x_m, raw_target.y_m, raw_target.yaw_rad],
                "target": [target_pickup.x_m, target_pickup.y_m, target_pickup.yaw_rad],
                "raw_observation": asdict(raw_observation),
                "observation": asdict(observation),
            })
        stand_key_ = next(
            (k_ for k_ in ("retry", "realign")
             if near is not None
             and (state["near_tracking"].get(k_) or {}).get("capture_attempt") == attempt_number),
            None)
        retry_capture_ = stand_key_ is not None
        if retry_capture_:
            # Plan D8 S4c-2 / S4c-3: the re-observation as placed and its target.
            state["near_tracking"][stand_key_].update(
                capture_base=[float(base[0]), float(base[1]), float(yaw)],
                observation=asdict(observation),
                target=[target_pickup.x_m, target_pickup.y_m, target_pickup.yaw_rad])
        if near is not None and state.get("near_capture", {}).get("status") == "pending":
            # Plan D8 S4a ①: the near capture's observation, the control
            # base it was placed with (this map estimate) and its time.
            near["start"] = {
                "observation": observation,
                "pose": (float(base[0]), float(base[1]), float(yaw)),
                "aligned_s": float(attempt["t_before_capture_s"]),
                # The capture's own acquisition stamp: tau's reference
                # before the first frame (Codex S4a-1 review P3).
                "capture_stamp_s": scene_input.stamp_ns * 1e-9 - options.initial_time,
                "attempt": attempt_number,
            }
        start_rear_pose = Pose2D(rear[0], rear[1], rear[2])
        if self.bottom_deck_returning():
            # The re-observation after the side legs (plan 2026-10-19, cycle fix): the same pallet as the one judged
            # -- checked before its estimate becomes the planned pickup, the known pallet or a mission plan. The
            # record is not rebound to it (Codex: a shape and 0.15 m do not prove the same pallet by themselves).
            out_ = self._bottom_deck_reobserved(t, rear, attempt, target_pickup)
            if out_ is not None:
                return SimpleNamespace(stop=True, observation=observation, near_leg=None)
        elif self.bottom_deck_on() and state.get("near_capture", {}).get("status") == "pending":
            # Plan 2026-10-21 task C (Codex plan round 1 P1): the near estimate against the record before it becomes
            # the planned pickup, the known pallet or a plan -- the insert gate alone came after the approach.
            out_ = self._bottom_deck_near_check(t, rear, target_pickup)
            if out_ is not None:
                return SimpleNamespace(stop=True, observation=observation, near_leg=None)
        hooks.set_inset_estimate(observation, (float(base[0]), float(base[1]), yaw), target_pickup, scene_input, attempt_number)
        state["perception"]["perception_pickup_estimate_m"] = {
            "x_m": target_pickup.x_m,
            "y_m": target_pickup.y_m,
            "yaw_rad": target_pickup.yaw_rad,
        }
        # Evaluation (truth): the runner's, at this same point (Codex 4b design: a plain call, explicit inputs).
        hooks.record_capture_evaluation(target_pickup, attempt, attempt_number, capture_diagnostics, step, t)
        planning_pickup, state["planning_target_source"] = hooks.planning_target(target_pickup)
        pocket["planning_pickup"] = planning_pickup
        if inject_:
            state["near_tracking"]["injection"]["captures"][-1]["planned_target"] = [
                planning_pickup.x_m, planning_pickup.y_m, planning_pickup.yaw_rad]
        if retry_capture_:
            state["near_tracking"][stand_key_]["planned_target"] = [
                planning_pickup.x_m, planning_pickup.y_m, planning_pickup.yaw_rad]
        if obstacle is not None and obstacle["layer"].known_enabled:
            # Each detection is a new relative fix: the age restarts
            # even for the same value (Codex stage-1 P2-8).
            known_pallet((planning_pickup.x_m, planning_pickup.y_m, planning_pickup.yaw_rad),
                         obstacle["layer"].known_config.get("r_fix_pickup_m", 0.05),
                         f"recognised:attempt{len(state.get('observation_attempts', []))}")
        if self.bottom_deck_on() and "bottom_deck" not in state:
            # Plan 2026-10-19 task 4: the side legs before the mission plan and any near capture.
            return self._bottom_deck_begin(t, rear, base, yaw, attempt, attempt_number, observation, planning_pickup)
        planning_start = wall()
        planning_trace = []
        if self.jobs is not None:
            # Real-time plan D8b: the mission plan is the worker's; the truck stands at the observation pose until
            # it is taken, and the rest of this observation (its followers, the near leg or the near capture's
            # straight) is finished then, with this capture's acquisition kept apart (_observe_finish).
            kwargs_ = dict(geometry=geometry, target_pickup=planning_pickup, start_rear=start_rear_pose,
                           return_to=return_to_pose, pickup_bounds=pickup_bounds, travel_config=travel_config,
                           trace=planning_trace, **grid_kwargs("mission", planning_pickup),
                           **self._exit_prefix_kwargs())
            self._request("observe", "plan_transport", {"args": (grid_world(scenario), planner_config),
                                                        "kwargs": kwargs_},
                          t, rear, self._applied_now(),
                          ctx={"base": tuple(float(v) for v in base), "yaw": float(yaw), "observation": observation,
                               "attempt_number": attempt_number, "planning_start": planning_start,
                               # the world time after the capture: what the tick would have stamped its depth with
                               "acquired_s": float(hooks.now_s()) if self.capture_acquired_s is None
                               else self.capture_acquired_s})
            self.capture_acquired_s = None
            return SimpleNamespace(stop=False, observation=observation, near_leg=None, pending=True)
        plans = plan_transport(
            grid_world(scenario),
            planner_config,
            geometry=geometry,
            target_pickup=planning_pickup,
            start_rear=start_rear_pose,
            return_to=return_to_pose,
            pickup_bounds=pickup_bounds,
            travel_config=travel_config,
            trace=planning_trace,
            **grid_kwargs("mission", planning_pickup),
        )
        near_leg = self._observe_install(plans, planning_trace, planning_start, attempt_number)
        return SimpleNamespace(stop=False, observation=observation, near_leg=near_leg)

    def tracker_config(self, name: str) -> TrackerConfig:
        """The follower configuration of a mission leg (observe excluded: it keeps its own), as the mission plan's
        adoption builds it (persistent console plan D5 6c: the loaded start builds its legs the same way)."""
        settings, speeds = self.settings, self.options.speeds
        CUSP_HEADING_HOLD_RAD, CUSP_HEADING_HOLD_PHASES = (self.options.cusp_heading_hold_rad,
                                                           self.options.cusp_heading_hold_phases)
        return TrackerConfig(
            cruise_speed_mps=speeds[name],
            max_curvature_inv_m=settings[
                "tracker_curvature_inv_m"
            ],
            max_acceleration_mps2=settings[
                "drive_acceleration_mps2"
            ],
            lookahead_m=0.28,
            # Same rule as the trackers built
            # without perception (observe is
            # excluded below and keeps its own
            # tracker; the return is 0.03 rad).
            position_tolerance_m=(
                0.03 if name == "observe" else 0.008
            ),
            yaw_tolerance_rad=(
                0.05
                if name == "observe"
                else 0.03
                if name == "return_home"
                else 0.02
            ),
            # Gear-change cusps are not goals (2026-09-26): the next leg
            # starts from the measured pose. Final goals keep the rules above.
            cusp_position_tolerance_m=0.03,
            cusp_yaw_tolerance_rad=0.05,
            # Brake and judge the cusp at 8 mm, still accepting 30 mm / 50 mrad
            # (docs/plans/2026-10-02-planner-tracker-robustness.md, P2).
            cusp_brake_window_m=0.008,
            # The return holds a cusp heading miss of 0.05-0.08 rad for
            # judgement (EPAL 6 fork spacing plan, A').
            cusp_heading_pending_rad=CUSP_HEADING_HOLD_RAD
            if name in CUSP_HEADING_HOLD_PHASES
            else None,
            # Optional path speed caps; absent from the settings = off.
            max_lateral_acceleration_mps2=settings.get(
                "max_lateral_acceleration_mps2"
            ),
            max_reverse_speed_mps=settings.get(
                "max_reverse_speed_mps"
            ),
            # A stop just past the goal is fine except deeper into
            # the pallet: insertion keeps the round 8 mm.
            overshoot_tolerance_m=None
            if name == "insert"
            else 0.03,
            stop_speed_mps=0.012,
            max_cross_track_error_m=0.35,
        )

    def _observe_install(self, plans, planning_trace: list, planning_start: float, attempt_number: int):
        """After the mission plan of a valid observation: its records, the followers, the paths shown and saved,
        and (the first far observation under SLAM) the near leg to the near capture point."""
        state, slam, hooks, options = self.state, self.slam, self.hooks, self.options
        paths, trackers, geometry = self.paths, self.trackers, self.geometry
        mission_stages = options.mission_stages
        require, wall, path_record, record_json = hooks.require, hooks.wall, hooks.path_record, hooks.record_json
        final_straight_prefix = hooks.final_straight_prefix
        apply_tracker_profile, record_tracker_configs = self.apply_tracker_profile, self.record_tracker_configs
        state.setdefault("planning_traces", []).append(
            planning_trace
        )
        state["planning_wall_s"] += (
            wall() - planning_start
        )
        state["planning_status"] = plans.status
        # MissionPlan.status already contains the failing phase.
        require(plans.success, plans.status)
        paths.update(
            {name: getattr(plans, name) for name in mission_stages}
        )
        trackers.update(
            {name: RearAxlePathTracker(path.poses, path.directions, path.curvatures_inv_m, self.tracker_config(name))
             for name, path in paths.items() if name != "observe"}
        )
        apply_tracker_profile()
        record_tracker_configs()
        state["paths"] = {
            name: path_record(path) for name, path in paths.items()
        }
        hooks.write_paths()
        hooks.show_path(paths["approach"], "Approach", (0.05, 0.45, 1.0))
        hooks.show_path(paths["transport"], "Transport", (1.0, 0.65, 0.04))
        hooks.export_scene()
        hooks.echo("PLANNED " + record_json(
                {
                    k: {
                        "length_m": v.length_m,
                        "expansions": v.expanded_nodes,
                    }
                    for k, v in paths.items()
                }
            ))
        near_leg = None
        if slam is not None and "near_capture" not in state:
            # Plan v3.6: see the pallet again where the final
            # straight begins, so only that straight (and the
            # insert/extract) runs on held odometry. Curved
            # approaches drifted 0.02-0.03 rad held (S2 v3.5).
            near_leg = final_straight_prefix(
                paths["approach"], geometry.alignment_straight_m
            )
            # Every approach ends in the 0.8 m alignment straight;
            # not finding it is a failure, not a skip (Codex v3.6 P1).
            require(near_leg is not None, "near_capture_no_final_straight")
            state["near_capture"] = {
                "status": "pending",
                "far_attempt": attempt_number,
                "far_estimate_m": state["perception"][
                    "perception_pickup_estimate_m"
                ],
                "far_error": state["perception"]["perception_error"],
            }
        return near_leg

    def _observe_finish(self, p, value, t: float, rear) -> None:
        """The mission plan taken: install it, then the near leg or the near capture's straight -- the
        observation placed by its capture (base, yaw, the acquisition time and pose for the first depth frame),
        the straight drawn and judged from where the truck stands now (Codex real-time D8 round 3 P1)."""
        ctx = p.ctx
        near_leg = self._observe_install(value["result"], value["trace"], ctx["planning_start"], ctx["attempt_number"])
        built_before = (self.state.get("pocket_check") or {}).get("built_s")
        self.near_leg_or_recapture(t, rear, np.asarray(ctx["base"], dtype=float), ctx["yaw"], ctx["observation"],
                                   ctx["attempt_number"], near_leg, acquired=(ctx["acquired_s"], p.rear))
        built_now = (self.state.get("pocket_check") or {}).get("built_s")
        if built_now is not None and built_now != built_before:
            self.fresh_depth_wait = (float(t), float(ctx["acquired_s"]))  # a fresh pocket frame must follow (D8b)
        self._refresh_for_new_leg(t, rear)  # the new leg's path check before the first move

    # --- plan 2026-10-19 task 4: the bottom-deck side legs (design rounds 1-5, Codex converged) ---
    def bottom_deck_on(self) -> bool:
        return getattr(self.options, "bottom_deck_check", "off") == "on"

    def bottom_deck_leg_active(self) -> bool:
        """A bottom-deck leg (a stand, a retake's retreat, or the return to the far observation point) is the observe
        leg now: its replans, recoveries and checks use the side stage's bounds."""
        deck_ = self.state.get("bottom_deck")
        return bool(deck_ and deck_.get("leg") is not None and self.ms.phase == "observe")

    def bottom_deck_side_leg(self) -> bool:
        """A stand or a retreat: its arrival is the side capture's and the far recognition takes no driving frames
        on it. The return leg is a far observation leg (Codex return design round 1 P2 3)."""
        return self.bottom_deck_leg_active() and self.state["bottom_deck"]["leg"]["kind"] in ("stand", "retreat")

    def bottom_deck_returning(self) -> bool:
        deck_ = self.state.get("bottom_deck")
        return bool(deck_ and deck_.get("status") == "returning")

    def _bottom_deck_target(self, attempt: dict, pickup):
        """The judged target from the far observation: the pickup estimate's centre and insertion yaw, and -- when
        the far recognition read it -- the used candidate's hypothesis, axis, support and id (Codex design round
        1 P2). The detector path has none of them: the run's shape, the x axis and the floor. Returns the target
        dict or a refusal reason."""
        shape_run = self.pallet_geometry.geometry_version
        far_ = attempt.get("recognition")
        if far_ is not None:
            used_ = far_.get("used") or {}
            if any(used_.get(k) is None for k in ("candidate_id", "hypothesis", "axis", "support_z_m")):
                return "far_record_incomplete"
            shape, axis = str(used_["hypothesis"]).split(":")[0], used_["axis"]
            support, candidate, track = float(used_["support_z_m"]), used_["candidate_id"], far_.get("selected_track")
        else:
            shape, axis, support, candidate, track = shape_run, "x", 0.0, None, None
        if shape != shape_run:
            return "shape_mismatch"
        if axis not in bottom_deck_legs.FACES:
            return "far_record_incomplete"
        return {"centre_xy": [float(pickup.x_m), float(pickup.y_m)], "insertion_yaw_rad": float(pickup.yaw_rad),
                "model_yaw_rad": bottom_deck_legs.model_yaw(float(pickup.yaw_rad), axis), "shape": shape,
                "axis": axis, "support_z_m": support, "candidate_id": candidate,
                "track_id": None if track is None else str(track)}

    def _bottom_deck_stand(self, face: int):
        tg_ = self.state["bottom_deck"]["target"]
        return bottom_deck_legs.stand(tg_["centre_xy"], tg_["insertion_yaw_rad"], tg_["axis"], face,
                                      mount_x_m=self.options.camera_mount_x_m,
                                      rear_axle_offset_m=self.options.rear_axle_offset_m)

    def _bottom_deck_plan(self, rear, face: int, goal=None):
        """A side leg to the face's stand (or to ``goal``) from ``rear``: at the planning clearance, then at none
        (as an observe replan). The planner checks the whole footprint against the map and the grid -- a stand
        or a retreat end that collides is refused (invalid_goal), so is a path through an obstacle."""
        options, hooks, geometry = self.options, self.hooks, self.geometry
        grid_world, grid_kwargs, plan_observation_leg = hooks.grid_world, hooks.grid_kwargs, hooks.plan_observation_leg
        planner_config, scenario = options.planner_config, options.scenario
        side_ = getattr(self, "bottom_deck_bounds", None)  # the side stage's bounds (also before a leg is active)
        observe_bounds_ = side_ if side_ is not None else self.observe_bounds()
        target = PlanningPose(*(self._bottom_deck_stand(face).rear if goal is None else goal))
        start = PlanningPose(float(rear[0]), float(rear[1]), float(rear[2]))
        plan_ = plan_observation_leg(grid_world(scenario), target, planner_config, geometry=geometry,
                                     start_rear=start, pickup_bounds=observe_bounds_, extended=False,
                                     **grid_kwargs("observe"))
        if not plan_.success:
            tight = replace(planner_config, clearance_m=0.0)
            plan_ = plan_observation_leg(grid_world(scenario), target, tight, geometry=geometry,
                                         start_rear=start, pickup_bounds=observe_bounds_, extended=False,
                                         **grid_kwargs("observe"))
        return plan_

    def _bottom_deck_install(self, t: float, path, kind: str, face: int, captures: int) -> None:
        """Install a side leg as the observe leg: its own replan budget and no-progress watch (Codex design
        round 2 P2), the path records."""
        ms, state, obstacle, hooks, paths, trackers = (self.ms, self.state, self.obstacle, self.hooks, self.paths,
                                                       self.trackers)
        paths["observe"] = path
        trackers["observe"] = RearAxlePathTracker(path.poses, path.directions, path.curvatures_inv_m,
                                                  trackers["observe"].config)
        state.setdefault("paths", {})["observe"] = hooks.path_record(path)
        hooks.write_paths()
        state["observe_replan_leg"] = {"serial": state["observe_replan_leg"]["serial"] + 1,
                                       "start": len(state["observe_replans"])}
        if obstacle is not None:
            obstacle.setdefault("progress", {}).clear()
        state["observation_waypoint_selected"] = np.asarray(path.poses[-1]).tolist()
        ms.phase_started = t
        state["bottom_deck"]["leg"] = {"kind": kind, "face": None if face is None else int(face),
                                       "captures": int(captures), "time_s": t}

    def _bottom_deck_release_if_held(self, t: float, event: str):
        """Release a SLAM hold before a side plan or a side capture's judgement reads the pose (Codex design
        rounds 1-4): the grid re-projected, the runner's rear set. The targets are not moved: the judged target, the
        pickup estimate and the planned pickup came from the far observation, made after its own release -- map
        targets the held odometry never carried (Codex return design rounds 1-2; release_and_stand's targets were
        made inside the hold and do move). Returns the released rear, or None when nothing was held."""
        slam = self.slam
        if slam is None or slam["tracker"].mode != "holding":
            return None
        released_ = self._release_reproject(t, event, carry=False)
        self.state["bottom_deck"].setdefault("releases", []).append(
            {"time_s": t, "event": event, "frame_change": [float(v) for v in released_["frame_change"]],
             "jump_m": released_["jump_m"], "after_attempts": len(self.state["bottom_deck"]["attempts"]),
             "carried": False})
        rear_ = np.asarray(released_["after"], dtype=float)
        self.hooks.set_rear(rear_)
        return rear_

    def _bottom_deck_refuse(self, t: float, reason: str, rear, stage: str = "side") -> SimpleNamespace:
        """End the mission at the side stage, the return or the re-observation (Codex design round 1: no reason to
        drive to the insert gate)."""
        state = self.state
        deck_ = state["bottom_deck"]
        deck_["status"] = "failed" if deck_.get("status") in ("pending", "returning") else deck_.get("status")
        deck_["leg"] = None
        deck_["refused"] = {"reason": reason, "time_s": t, "stage": stage}
        state["mission_result"] = "bottom_deck_refused"
        self.transition("bottom_deck_refused", t)
        return SimpleNamespace(stop=True, rear=rear, installed=False)

    def _bottom_deck_begin(self, t: float, rear, base, yaw: float, attempt: dict, attempt_number: int, observation,
                           pickup) -> SimpleNamespace:
        """The first valid far observation with the check on: no mission plan yet, the side legs first (all of
        them before any near capture)."""
        state, obstacle = self.state, self.obstacle
        deck_ = state["bottom_deck"] = {"status": "pending", "leg": None, "faces": [], "attempts": [], "record": None}
        # The first valid far observation's stand: the side legs return to it and observe again from there, so the
        # near leg starts where it always did (the near capture's held yaw drifted after a turn from a side stand).
        waypoint_ = attempt.get("waypoint") or state.get("observation_waypoint_selected")
        self.bottom_deck_far = SimpleNamespace(observation=observation, attempt_number=attempt_number,
                                               base=np.asarray(base, dtype=float), yaw=float(yaw),
                                               waypoint=None if waypoint_ is None else
                                               tuple(float(v) for v in waypoint_))
        target_ = self._bottom_deck_target(attempt, pickup)
        if isinstance(target_, str):
            deck_["target"] = None
            out_ = self._bottom_deck_refuse(t, "bottom_deck_target_mismatch" if target_ == "shape_mismatch"
                                            else "bottom_deck_unobserved", rear)
            deck_["refused"]["detail"] = target_
            return SimpleNamespace(stop=True, observation=observation, near_leg=None, handled=True)
        deck_["target"] = target_
        if obstacle is not None:
            # What grid_kwargs("mission") would have set (Codex design round 1 P2): the side plans see the
            # recognised pallet, not the pickup zone, and a failure is not handed to the far candidates.
            obstacle["pickup_estimate"] = pickup
        order_ = bottom_deck_legs.ordered_faces(target_["axis"], target_["centre_xy"], target_["insertion_yaw_rad"],
                                                rear_xy=(float(rear[0]), float(rear[1])),
                                                mount_x_m=self.options.camera_mount_x_m,
                                                rear_axle_offset_m=self.options.rear_axle_offset_m)
        stands_ = {f: self._bottom_deck_stand(f) for f in order_}
        far_wp_ = self.bottom_deck_far.waypoint
        if far_wp_ is None:
            out_ = self._bottom_deck_refuse(t, "bottom_deck_unobserved", rear)
            deck_["refused"]["detail"] = "no_far_waypoint"
            return SimpleNamespace(stop=True, observation=observation, near_leg=None, handled=True)
        # The side stage's own bounds (Codex A'): the pickup zone confines the pickup legs for the search's sake, not
        # as a wall, and a side stand of a pallet near its edge lies outside it. Every side plan, retreat, recovery,
        # the return and the cusp check use these. The mission is planned after the return, from the far
        # observation point in the zone, so no stand need lie in it (Codex return design round 2 P2 2).
        zone_, fp_ = self.observe_bounds(), self.geometry.unloaded_footprint  # the batch zone; live: the hall
        margin_ = min(float(self.options.planner_config.clearance_m), float(self.geometry.approach_gap_m) / 2)
        # Where the mission after the re-observation starts from: its near_bounds, the pickup bounds or else the
        # scenario's (Codex retry review round 6 P2), with the approach's own clearance.
        mission_zone_ = self.options.pickup_bounds
        self.bottom_deck_zone = (self.options.scenario.bounds if mission_zone_ is None else mission_zone_, fp_, margin_)
        self.bottom_deck_bounds = zone_ if zone_ is None else bottom_deck_legs.side_bounds(
            zone_, self.options.scenario.bounds,
            [stands_[f].rear for f in order_] + [bottom_deck_legs.retreat_pose(stands_[f].rear) for f in order_]
            + [far_wp_], fp_, margin_)
        deck_["order"] = list(order_)
        deck_["stands"] = {str(f): {"rear": list(stands_[f].rear), "camera_xy": list(stands_[f].camera_xy)}
                           for f in order_}
        deck_["return_waypoint"] = list(far_wp_)
        deck_["bounds"] = None if self.bottom_deck_bounds is None else asdict(self.bottom_deck_bounds)
        # The legs in their order reachable before the first side leg starts (task 4: else no side leg at all):
        # from here to the first stand, from the first stand to the second, and from it back to the far stand.
        plans_ = {order_[0]: self._bottom_deck_plan(rear, order_[0]),
                  order_[1]: self._bottom_deck_plan(stands_[order_[0]].rear, order_[1]),
                  "return": self._bottom_deck_plan(stands_[order_[1]].rear, None, goal=far_wp_)}
        deck_["reachable"] = {str(f): p.status for f, p in plans_.items()}
        if not all(p.success for p in plans_.values()):
            out_ = self._bottom_deck_refuse(t, "bottom_deck_unobserved", rear)
            deck_["refused"]["detail"] = "stand_unreachable"
            return SimpleNamespace(stop=True, observation=observation, near_leg=None, handled=True)
        self._bottom_deck_install(t, plans_[order_[0]], "stand", order_[0], 0)
        self.hooks.set_inset_status("옆면 증거 위치로 이동")
        return SimpleNamespace(stop=False, observation=observation, near_leg=None, handled=True)

    def _bottom_deck_scene_target(self, rear_est):
        """The judged target in the capture's base frame, by the control estimate (never the truth)."""
        tg_ = self.state["bottom_deck"]["target"]
        off_ = abs(self.options.rear_axle_offset_m)
        base_ = (float(rear_est[0]) + off_ * math.cos(float(rear_est[2])),
                 float(rear_est[1]) + off_ * math.sin(float(rear_est[2])), float(rear_est[2]))
        cx_, cy_, my_ = compose(invert(base_), (tg_["centre_xy"][0], tg_["centre_xy"][1], tg_["model_yaw_rad"]))
        return BottomDeckTarget((float(cx_), float(cy_)), float(my_), self.options.bottom_deck_spec,
                                float(tg_["support_z_m"])), base_

    def bottom_deck_capture(self, t: float, rear, step: int) -> SimpleNamespace:
        """The side leg's arrival. A retreat's end: release, then the leg back to the same stand (a retake
        arrives afresh). A stand: one standing frame judged on its face, saved; a decision or the third capture
        ends the face, anything else retreats for a retake. Returns ``stop``, ``rear``."""
        state, hooks, options = self.state, self.hooks, self.options
        deck_ = state["bottom_deck"]
        leg_ = deck_["leg"]
        face = leg_["face"]
        if leg_["kind"] == "retreat":
            rear_ = self._bottom_deck_release_if_held(t, "bottom_deck_retreat_end")
            rear = rear if rear_ is None else rear_
            plan_ = self._bottom_deck_plan(rear, face)
            if not plan_.success:
                return self._bottom_deck_face_end(t, rear, "unobserved", f"no_path:{plan_.status}")
            self._bottom_deck_install(t, plan_, "stand", face, leg_["captures"])
            return SimpleNamespace(stop=False, rear=rear, installed=True)
        number_ = len(deck_["attempts"]) + 1
        rec_ = {"face": face, "capture": leg_["captures"] + 1, "time_s": t, "step": step,
                "rear_arrival": [float(v) for v in rear]}
        try:
            scene_, _, capture_attempts_ = hooks.perception_capture().capture(
                max_attempts=options.perception_max_attempts)
        except options.adapter.CaptureFailure as exc:
            rec_.update(verdict="unobserved", reason=f"capture_failed:{exc.reason}")
            scene_ = None
        # Released before the judgement (Codex return design round 2): the map target goes into the capture's base
        # frame by the released map pose, not by the held odometry.
        rear_ = self._bottom_deck_release_if_held(t, "release_at_bottom_deck_capture")
        rear = rear if rear_ is None else rear_
        if scene_ is not None:
            rear_est_ = hooks.slam_rear(hooks.now_s()) if self.slam is not None else rear
            target_b_, base_est_ = self._bottom_deck_scene_target(rear_est_)
            axis_ = deck_["target"]["axis"]
            arrived_ = bottom_deck_evidence.arrival_ok(scene_, target_b_, face=face, axis=axis_)
            cam_ = bottom_deck_evidence.camera_xyz(scene_)[:2]
            rec_.update(capture_attempts=capture_attempts_, base_estimate=list(base_est_),
                        rear_estimate=[float(v) for v in rear_est_],
                        theta_e_deg=bottom_deck_evidence.stand_angle_deg(cam_, target_b_, face),
                        range_m=float(np.linalg.norm(cam_ - np.asarray(target_b_.centre_xy))),
                        truth_rear=[float(v) for v in hooks.truth_rear()])
            if not arrived_:
                rec_.update(verdict="unobserved", reason="arrival")
            else:
                ev_ = bottom_deck_evidence.evaluate_side(scene_, target_b_, face=face,
                                                         params=options.bottom_deck_params)
                rec_.update(verdict=ev_.verdict, reason=ev_.reason, gaps=[g.state for g in ev_.gaps])
            rec_["npz"] = f"bottom_deck/side_{number_:02d}.npz"
            hooks.save_npz(rec_["npz"], depth_m=np.asarray(scene_.depth_m, dtype=float),
                           intrinsics=np.asarray([scene_.intrinsics.width, scene_.intrinsics.height,
                                                  scene_.intrinsics.fx, scene_.intrinsics.fy,
                                                  scene_.intrinsics.cx, scene_.intrinsics.cy], dtype=float),
                           intrinsics_frame=scene_.intrinsics.frame_id,
                           mount_translation=np.asarray(scene_.base_from_optical.translation_m, dtype=float),
                           mount_rotation=np.asarray(scene_.base_from_optical.rotation, dtype=float),
                           target_base=np.asarray([*target_b_.centre_xy, target_b_.model_yaw_rad,
                                                   target_b_.support_z_m]),
                           target_map=np.asarray([*deck_["target"]["centre_xy"], deck_["target"]["model_yaw_rad"]]),
                           rear_estimate=np.asarray(rec_["rear_estimate"], dtype=float),
                           truth_rear=np.asarray(rec_["truth_rear"], dtype=float),
                           face=face, axis=axis_, shape=deck_["target"]["shape"],
                           params=json.dumps(asdict(options.bottom_deck_params)), verdict=rec_["verdict"],
                           reason=rec_["reason"], stamp_ns=int(scene_.stamp_ns))
        deck_["attempts"].append(rec_)
        leg_["captures"] += 1
        if rec_["verdict"] in ("present", "absent") or leg_["captures"] >= bottom_deck_legs.CAPTURES:
            return self._bottom_deck_face_end(t, rear, rec_["verdict"], rec_["reason"])
        # A retake arrives afresh: back off to a point behind the stand, planned (Codex implementation round 1 P1:
        # the planner checks the footprint along it), then the leg back to the stand. No detour: a retreat the
        # planner can only make long ends the face.
        back_ = bottom_deck_legs.retreat_pose(self._bottom_deck_stand(face).rear)
        plan_ = self._bottom_deck_plan(rear, face, goal=back_)
        if not plan_.success or plan_.length_m > bottom_deck_legs.RETREAT_MAX_M:
            return self._bottom_deck_face_end(t, rear, "unobserved",
                                              "retreat_detour" if plan_.success else f"retreat_blocked:{plan_.status}")
        self._bottom_deck_install(t, plan_, "retreat", face, leg_["captures"])
        return SimpleNamespace(stop=False, rear=rear, installed=True)

    def _bottom_deck_face_end(self, t: float, rear, verdict: str, reason: str) -> SimpleNamespace:
        """One face concluded: the next face's leg, or the two-face policy -- present plans the mission from here
        (the near capture is set up only now), anything else ends the mission (Codex design round 5)."""
        deck_ = self.state["bottom_deck"]
        leg_ = deck_["leg"]
        deck_["faces"].append({"face": leg_["face"], "verdict": verdict, "reason": reason,
                               "captures": leg_["captures"]})
        deck_["leg"] = None
        rest_ = [f for f in deck_["order"] if f not in [x["face"] for x in deck_["faces"]]]
        if verdict != "absent" and rest_:
            rear_ = self._bottom_deck_release_if_held(t, "bottom_deck_next_face")
            rear = rear if rear_ is None else rear_
            plan_ = self._bottom_deck_plan(rear, rest_[0])
            if plan_.success:
                self._bottom_deck_install(t, plan_, "stand", rest_[0], 0)
                return SimpleNamespace(stop=False, rear=rear, installed=True)
            deck_["faces"].append({"face": rest_[0], "verdict": "unobserved", "reason": f"no_path:{plan_.status}",
                                   "captures": 0})
        return self._bottom_deck_conclude(t, rear)

    def _bottom_deck_conclude(self, t: float, rear) -> SimpleNamespace:
        state, pocket, hooks, options = self.state, self.pocket, self.hooks, self.options
        deck_ = state["bottom_deck"]
        # Released before the record is made: the record's centre is the one the planned pickup is carried to
        # (a release after it would move the pickup away from the record -- found with Codex round 2).
        rear_ = self._bottom_deck_release_if_held(t, "bottom_deck_done")
        rear = rear if rear_ is None else rear_
        verdicts_ = [f["verdict"] for f in deck_["faces"]]
        faces_seen_ = {f["face"] for f in deck_["faces"] if f["verdict"] == "present"}
        verdict = ("absent" if "absent" in verdicts_ else
                   "present" if len(faces_seen_) == 2 and set(verdicts_) == {"present"} else "unobserved")
        tg_ = deck_["target"]
        applied_ = None if self.slam is None else self.slam["tracker"].applied
        deck_["record"] = bottom_deck_permit.BottomDeckRecord(
            verdict=verdict, faces=tuple(deck_["faces"]), centre_xy=tuple(tg_["centre_xy"]),
            model_yaw_rad=tg_["model_yaw_rad"], shape=tg_["shape"], axis=tg_["axis"],
            support_z_m=tg_["support_z_m"], candidate_id=tg_["candidate_id"], track_id=tg_["track_id"],
            captured_s=tuple(a["time_s"] for a in deck_["attempts"]),
            correction_s=None if applied_ is None else float(applied_[1]),
            params_sha256=bottom_deck_permit.params_digest(options.bottom_deck_params)).to_record()
        deck_["status"] = "judged"
        if verdict != "present":
            return self._bottom_deck_refuse(t, "no_bottom_deck" if verdict == "absent" else "bottom_deck_unobserved",
                                            rear)
        # Two faces present: back to the far observation point, observe again there and go on as without the side
        # legs (the near leg then starts where it always did). The record stays the map target judged.
        self.bottom_deck_return_goal = self.bottom_deck_far.waypoint  # a look-again retry moves it on
        plan_ = self._bottom_deck_plan(rear, None, goal=self.bottom_deck_return_goal)
        if not plan_.success:
            out_ = self._bottom_deck_refuse(t, "bottom_deck_return_failed", rear, stage="return")
            deck_["refused"]["detail"] = plan_.status
            return out_
        deck_["status"] = "returning"
        zone_hook_ = getattr(self.hooks, "far_recognition_zone", None)
        if zone_hook_ is not None:
            # The re-observation looks for the pallet judged, not for any pallet in the pickup zone: another object
            # metres away must not leave it undecided (Isaac dc44a32: a 0.07-score proposal 2.6 m off read as an
            # uncertain candidate in the zone's 1 m radius). The gate's 0.15 m still binds what is adopted.
            zone_hook_(tuple(deck_["record"]["centre_xy"]), bottom_deck_legs.REOBSERVE_RADIUS_M)
            deck_["reobservation_zone"] = {"centre_xy": list(deck_["record"]["centre_xy"]),
                                           "radius_m": bottom_deck_legs.REOBSERVE_RADIUS_M}
        self._bottom_deck_install(t, plan_, "return", None, 0)
        self.hooks.set_inset_status("원거리 관측 위치로 복귀")
        return SimpleNamespace(stop=False, rear=rear, installed=True)

    def _bottom_deck_reobserve_failed(self, t: float, rear, attempt: dict, attempt_number: int, observation) -> bool:
        """A re-observation that did not see the pallet judged. A decision that asks to look again (an uncertain or an
        unresolved object in reach of the record's zone -- Isaac ecf4be5 s1) takes the next fixed observation point,
        as the far observation's own failure does (observe_failed's ladder), as a return leg: planned in the side
        stage's bounds, its recoveries aimed at that point, the record and the zone kept. Any other failure, the
        bound or no plannable point refuses. True ends the mission loop (Codex retry design round 1)."""
        deck_ = self.state["bottom_deck"]
        far_ = attempt.get("recognition") or {}
        reason_ = getattr(observation, "reason", None)
        tries_ = deck_.setdefault("reobservation_attempts", [])
        retries_ = sum(1 for a in tries_ if a["outcome"] == "retry")
        tries_.append({"time_s": t, "attempt": attempt_number, "far_leg": far_.get("leg"), "status": far_.get("status"),
                       "reason": far_.get("reason", reason_), "outcome": "refused"})

        def refuse(detail):
            self._bottom_deck_refuse(t, "bottom_deck_reobservation_failed", rear, stage="reobserve")
            deck_["refused"]["detail"] = detail
            return True

        if (far_.get("status") != "reobserve" or far_.get("reason") not in bottom_deck_legs.REOBSERVE_RETRY_REASONS
                or self.state.get("near_capture", {}).get("status") == "pending"
                or retries_ >= bottom_deck_legs.REOBSERVE_RETRIES):
            return refuse(reason_)
        waypoints_ = self.options.observation_waypoints
        while self.ms.next_candidate_index < len(waypoints_):
            index_ = self.ms.next_candidate_index
            self.ms.next_candidate_index += 1  # the far observation's ladder: a point tried is not tried again
            goal_ = tuple(float(v) for v in waypoints_[index_])
            zone_, fp_, margin_ = self.bottom_deck_zone
            if not bottom_deck_legs.inside(zone_, goal_, fp_, margin_):
                # Reachable in the side bounds, but the mission planned there after the re-observation starts in the
                # pickup bounds (approach:invalid_start; Codex retry review round 5 P2): not a re-observation point.
                plan_ = SimpleNamespace(success=False, status="outside_mission_zone", search_attempts=())
            else:
                plan_ = self._bottom_deck_plan(rear, None, goal=goal_)
            self.state.setdefault("observation_candidates", []).append(
                {"candidate_index": index_, "pose": list(goal_), "start_rear": np.asarray(rear).tolist(),
                 "success": plan_.success, "status": plan_.status, "bottom_deck_reobservation": True,
                 "search_attempts": [list(entry) for entry in plan_.search_attempts]})
            if plan_.success:
                tries_[-1].update(outcome="retry", candidate_index=index_, goal=list(goal_))
                self.bottom_deck_return_goal = goal_
                self._bottom_deck_install(t, plan_, "return", None, 0)  # a fresh replan budget and progress watch
                self.hooks.set_inset_status("다음 관측 위치에서 재관측")
                return False
        tries_[-1]["outcome"] = "exhausted"
        return refuse("candidates_exhausted")

    def _bottom_deck_reobserved(self, t: float, rear, attempt: dict, target_pickup):
        """The re-observation's estimate against the record: the same shape (the far recognition's hypothesis when it
        read one, else the run's), the centre within 0.15 m and the entry axis within 15 deg -- the insert gate's own
        test. Refused (None otherwise) before the estimate is adopted; the record is never moved."""
        deck_ = self.state["bottom_deck"]
        far_ = attempt.get("recognition")
        used_ = (far_ or {}).get("used") or {}
        shape_ = (str(used_["hypothesis"]).split(":")[0] if used_.get("hypothesis")
                  else self.pallet_geometry.geometry_version)
        refusal_ = bottom_deck_permit.insert_refusal(
            bottom_deck_permit.BottomDeckRecord.from_record(deck_["record"]),
            (float(target_pickup.x_m), float(target_pickup.y_m)), float(target_pickup.yaw_rad), shape_)
        deck_["reobservation"] = {"time_s": t, "target": [float(target_pickup.x_m), float(target_pickup.y_m),
                                                          float(target_pickup.yaw_rad)],
                                  "shape": shape_, "refusal": refusal_,
                                  "used_error": (far_ or {}).get("used_error"),
                                  "single_error": (far_ or {}).get("single_error")}
        deck_.setdefault("reobservation_attempts", []).append(
            {"time_s": t, "far_leg": (far_ or {}).get("leg"), "status": (far_ or {}).get("status"),
             "outcome": "mismatch" if refusal_ is not None else "accepted"})
        if refusal_ is not None:
            return self._bottom_deck_refuse(t, "bottom_deck_target_mismatch", rear, stage="reobserve")
        deck_["status"] = "done"
        if self.obstacle is not None:
            # The near leg planned next is a new observe leg: the return's no-progress watch must not carry over
            # (Codex cycle implementation round 1 P2 -- the return's small best_m tripped a normal near leg at 60 s).
            self.obstacle.setdefault("progress", {}).clear()
        return None

    def _near_expected_on(self) -> bool:
        """Plan 2026-10-22 task A: a near capture (pending) under --near-capture-expected on."""
        return bool(getattr(self.options, "near_capture_expected", "off") == "on"
                    and self.state.get("near_capture", {}).get("status") == "pending")

    def _near_waiver_on(self) -> bool:
        """Plan 2026-10-22 task C: --near-capture-lower waive_far_confirmed (needs --near-capture-expected on)."""
        return getattr(self.options, "near_capture_lower", "gate") == "waive_far_confirmed"

    def _expected_record(self, site: ExpectedSite, candidates_) -> dict:
        pickup_ = self.pocket["planning_pickup"]
        return {
            "site": [site.x_m, site.y_m, site.yaw_rad], "radius_m": site.radius_m,
            "axis_tolerance_rad": site.axis_tolerance_rad,
            "planning_pickup": [float(pickup_.x_m), float(pickup_.y_m), float(pickup_.yaw_rad)],
            "candidates": [asdict(c_) for c_ in candidates_],
        }

    def _far_waiver_refusal(self) -> tuple[str | None, dict]:
        """Condition 1 of the user's decision: the far capture this near capture follows passed the full detector at
        its own thresholds, and the pallet that detector saw is the adopted far estimate. (refusal or None, record)."""
        near_ = self.state["near_capture"]
        number_ = near_.get("far_attempt")
        record_ = {"far_attempt": number_}
        attempts_ = self.state.get("observation_attempts") or []
        if not isinstance(number_, int) or not 1 <= number_ <= len(attempts_):
            return "far_attempt_missing", record_
        far_ = attempts_[number_ - 1]
        detector_ = far_.get("detector_reference", far_.get("pocket_observation")) or {}
        record_["far_detector_status"] = detector_.get("status")
        if detector_.get("status") != "valid":
            return f"far_detector:{detector_.get('status')}", record_
        if far_.get("lower_evidence", True) is not True:
            return "far_lower_waived", record_
        if far_.get("expected") is not None:
            return "far_restricted", record_
        site_ = far_.get("detector_site")
        if site_ is None:
            return "far_detector_site_missing", record_
        estimate_ = near_["far_estimate_m"]
        link_m_ = math.hypot(float(site_[0]) - float(estimate_["x_m"]), float(site_[1]) - float(estimate_["y_m"]))
        link_rad_ = abs(math.atan2(math.sin(float(site_[2]) - float(estimate_["yaw_rad"])),
                                   math.cos(float(site_[2]) - float(estimate_["yaw_rad"]))))
        record_.update(far_detector_site=[float(v) for v in site_],
                       far_estimate=[float(estimate_[k]) for k in ("x_m", "y_m", "yaw_rad")],
                       far_link_m=link_m_, far_link_rad=link_rad_)
        if link_m_ > NEAR_WAIVER_RADIUS_M or link_rad_ > NEAR_WAIVER_AXIS_RAD:
            return "far_link", record_
        return None, record_

    def _near_lower_waiver(self, attempt: dict, detection, site: ExpectedSite, detector_input, prior, params):
        """Plan 2026-10-22 task C (user decision 2026-10-22, conditional): a near capture with the check off that found
        no valid pocket inside its expected site is detected once more with the lower band waived -- only after a far
        capture that passed in full (condition 1), only among candidates within 0.15 m and 15 deg of the expected site
        (condition 2), and recorded each time with its conditions (condition 3). The first result stands otherwise."""
        first_ = detection.observation
        refusal_, record_ = self._far_waiver_refusal()
        record_["first"] = {"status": first_.status, "reason": first_.reason}
        if refusal_ is None:
            narrow_ = ExpectedSite(site.x_m, site.y_m, site.yaw_rad, NEAR_WAIVER_RADIUS_M, NEAR_WAIVER_AXIS_RAD)
            second_, candidates_ = self.hooks.detect_pockets_expected(detector_input, prior, params, narrow_,
                                                                      lower_evidence=False)
            record_["second"] = self._expected_record(narrow_, candidates_)
            if second_.observation.status == "valid":
                record_["applied"] = True
                record_["first"]["expected"] = attempt["expected"]
                attempt["expected"] = record_["second"]
                attempt["lower_evidence"] = False
                attempt["lower_waiver"] = record_
                return second_
            refusal_ = f"second:{second_.observation.status}:{second_.observation.reason}"
        record_.update(applied=False, why=refusal_)
        attempt["lower_waiver"] = record_
        return detection

    def _near_expected_site(self, base, yaw: float) -> ExpectedSite:
        """The planned pickup in base_link at the capture's control base (x, y, yaw)."""
        pickup_ = self.pocket["planning_pickup"]
        dx, dy = float(pickup_.x_m) - float(base[0]), float(pickup_.y_m) - float(base[1])
        c_, s_ = math.cos(yaw), math.sin(yaw)
        rel_ = math.atan2(math.sin(float(pickup_.yaw_rad) - yaw), math.cos(float(pickup_.yaw_rad) - yaw))
        return ExpectedSite(c_ * dx + s_ * dy, -s_ * dx + c_ * dy, rel_, NEAR_EXPECTED_RADIUS_M,
                            NEAR_EXPECTED_AXIS_RAD)

    def _bottom_deck_lower_waived(self) -> bool:
        """The near capture after a present bottom-deck record and its checked re-observation (plan 2026-10-21 task C)."""
        deck_ = self.state.get("bottom_deck") or {}
        return bool(self.bottom_deck_on() and self.state.get("near_capture", {}).get("status") == "pending"
                    and deck_.get("status") == "done" and (deck_.get("record") or {}).get("verdict") == "present")

    def _bottom_deck_near_check(self, t: float, rear, target_pickup):
        """The near capture's estimate against the record, before adoption (plan 2026-10-21 task C): the insert gate's
        own test (the run's shape -- the near detector reads no hypothesis). Refused (None otherwise); recorded."""
        deck_ = self.state["bottom_deck"]
        shape_ = self.pallet_geometry.geometry_version
        refusal_ = None
        if deck_.get("record") is not None:
            refusal_ = bottom_deck_permit.insert_refusal(
                bottom_deck_permit.BottomDeckRecord.from_record(deck_["record"]),
                (float(target_pickup.x_m), float(target_pickup.y_m)), float(target_pickup.yaw_rad), shape_)
        deck_["near_check"] = {"time_s": t, "target": [float(target_pickup.x_m), float(target_pickup.y_m),
                                                       float(target_pickup.yaw_rad)],
                               "shape": shape_, "refusal": refusal_}
        if refusal_ is not None:
            return self._bottom_deck_refuse(t, "bottom_deck_target_mismatch", rear, stage="near")
        return None

    def bottom_deck_recover(self, t: float, rear, why: str) -> SimpleNamespace:
        """A recovery that would replace a side leg's path (blocked, a heading miss or a stall): released first
        (Codex design rounds 3-5). A retreat makes no detour -- the face's retakes end there; a stand is planned
        again to the same stand, and a failure ends the face. Returns ``stop``, ``rear``, ``installed``."""
        deck_ = self.state["bottom_deck"]
        leg_ = deck_["leg"]
        rear_ = self._bottom_deck_release_if_held(t, f"bottom_deck_{why}")
        rear = rear if rear_ is None else rear_
        deck_.setdefault("recoveries", []).append({"time_s": t, "why": why, "kind": leg_["kind"],
                                                   "face": leg_["face"]})
        if leg_["kind"] == "return":
            deck_["recoveries"][-1]["goal"] = list(self.bottom_deck_return_goal)
        if leg_["kind"] == "retreat":
            return self._bottom_deck_face_end(t, rear, "unobserved", f"retreat_{why}")
        if leg_["kind"] == "return":
            # Back to the far observation point (or a retry's next point): a detour is allowed (it is a far observation
            # leg), bounded apart for each point.
            goal_ = list(self.bottom_deck_return_goal)
            if (sum(1 for r in deck_["recoveries"] if r["kind"] == "return" and r.get("goal") == goal_)
                    > bottom_deck_legs.RETURN_RECOVERIES):
                return self._bottom_deck_refuse(t, "bottom_deck_return_failed", rear, stage="return")
            plan_ = self._bottom_deck_plan(rear, None, goal=self.bottom_deck_return_goal)
            if not plan_.success:
                return self._bottom_deck_refuse(t, "bottom_deck_return_failed", rear, stage="return")
            if self.hooks.same_path(plan_, self.paths["observe"], self.trackers["observe"].remaining_to_goal_m()):
                deck_["recoveries"][-1]["same_path"] = True
                return SimpleNamespace(stop=False, rear=rear, installed=False)
            self._bottom_deck_install(t, plan_, "return", None, 0)
            return SimpleNamespace(stop=False, rear=rear, installed=True)
        if sum(1 for r in deck_["recoveries"] if r["face"] == leg_["face"]) > bottom_deck_legs.RECOVERIES_PER_FACE:
            # Every install restarts the leg's replan budget, so the face carries its own bound.
            return self._bottom_deck_face_end(t, rear, "unobserved", f"{why}:recovery_limit")
        plan_ = self._bottom_deck_plan(rear, leg_["face"])
        if not plan_.success:
            return self._bottom_deck_face_end(t, rear, "unobserved", f"{why}:{plan_.status}")
        if self.hooks.same_path(plan_, self.paths["observe"], self.trackers["observe"].remaining_to_goal_m()):
            # The same path again is a wait (as blocked_replan's D4): it does not restart the no-progress watch.
            deck_["recoveries"][-1]["same_path"] = True
            return SimpleNamespace(stop=False, rear=rear, installed=False)
        self._bottom_deck_install(t, plan_, "stand", leg_["face"], leg_["captures"])
        return SimpleNamespace(stop=False, rear=rear, installed=True)

    def _sweep_clear(self, trajectory, rectangles) -> bool:
        """The empty outline swept along ``trajectory`` (rear-axle poses) meets none of ``rectangles``."""
        from forklift_core.planning.geometry import Bounds, collision_free_path

        return collision_free_path(np.asarray(trajectory, dtype=float), list(rectangles),
                                   self.geometry.unloaded_footprint, Bounds(-1e6, 1e6, -1e6, 1e6))

    def near_leg_or_recapture(self, t: float, rear, base, yaw: float, observation, attempt_number: int, near_leg,
                              acquired=None) -> None:
        """After the mission plan: the first far observation under SLAM drives the near leg to the near capture
        point; the near capture itself re-draws the approach straight from where the truck stands (box,
        dry run, the S4c-3 re-draw acceptance, the once-per-pickup realignment), installs it with the pocket
        check and the near-field tracker, and moves to the approach."""
        ms, state, slam, hooks, options, near = self.ms, self.state, self.slam, self.hooks, self.options, self.near
        paths, trackers, geometry, transition = self.paths, self.trackers, self.geometry, self.transition
        speeds, near_tracking, Pose2D = options.speeds, options.near_tracking, PlanningPose
        require, path_record, final_straight_prefix, realign_active, realign_install = (
            hooks.require, hooks.path_record, hooks.final_straight_prefix, self.realign_active, self.realign_install)
        straight_from_pose, bicycle_rollout, near_capture_redraw, start_near_realign = (
            hooks.straight_from_pose, hooks.bicycle_rollout, self.near_capture_redraw, self.start_near_realign)
        record_tracker_configs, build_pocket_check, build_near_tracker = (
            self.record_tracker_configs, hooks.build_pocket_check, self.build_near_tracker)
        if near_leg is not None:
            # Fix (1): a near leg opens its own observe replan budget.
            state["observe_replan_leg"] = {
                "serial": state["observe_replan_leg"]["serial"] + 1,
                "start": len(state["observe_replans"]),
            }
            paths["observe"] = near_leg
            trackers["observe"] = RearAxlePathTracker(
                near_leg.poses,
                near_leg.directions,
                near_leg.curvatures_inv_m,
                trackers["observe"].config,
            )
            state["observation_waypoint_selected"] = (
                near_leg.poses[-1].tolist()
            )
            state["near_capture"]["waypoint"] = near_leg.poses[-1].tolist()
            ms.phase_started = t
            hooks.set_inset_status("근접 재관측 위치로 이동")
            realign_install("near_leg")  # Plan D8 S4c-3 stage 2
        else:
            near_realign_ = False  # Plan D8 S4c-3: stage 2 began at this capture
            if state.get("near_capture", {}).get("status") == "pending":
                # The new estimate moves the straight by a few
                # cm; re-draw it from where the truck stands
                # instead of a Hybrid A* manoeuvre that would
                # run held (Codex v3.6 P2).
                new_prefix = final_straight_prefix(
                    paths["approach"], geometry.alignment_straight_m
                )
                require(
                    new_prefix is not None,
                    "near_capture_no_final_straight",
                )
                line_start = Pose2D(*new_prefix.poses[-1])
                line_end = Pose2D(*paths["approach"].poses[-1])
                straight, offsets = straight_from_pose(
                    Pose2D(float(rear[0]), float(rear[1]), float(rear[2])),
                    line_start,
                    line_end,
                    # The box stays at 5 cm / 0.05 rad: the dry run
                    # checks arrival only, not the swept footprint,
                    # and a wider box admitted a collision (Codex v3.7 P1).
                    # A smoke may narrow the heading (plan D8 S4c-3).
                    max_lateral_m=0.05,
                    max_yaw_rad=options.near_capture_box_yaw_rad,
                    min_length_m=0.3,
                )
                state["near_capture"]["offsets_to_new_line"] = offsets
                state["near_capture"]["box"] = {
                    "lateral_m": 0.05, "yaw_rad": options.near_capture_box_yaw_rad, "length_m": 0.3}
                # The box above only bounds the projection; whether
                # the approach tracker converges within this straight
                # is checked by a dry run with margin (v3.7, Codex
                # v3.6 re-review P2).
                acceptance_config = trackers["approach"].config
                if options.approach_straight_speed_mps is not None:
                    # Plan D8b: the baseline cruise, also when an earlier
                    # near capture left a custom-cruise tracker here.
                    acceptance_config = replace(acceptance_config,
                                                cruise_speed_mps=speeds["approach"])
                refusal = f"near_capture_misaligned:{offsets}"
                if straight is not None:
                    dry = bicycle_rollout(
                        straight.poses,
                        straight.directions,
                        straight.curvatures_inv_m,
                        acceptance_config,
                        rear,
                    )
                    state["near_capture"]["dry_run"] = asdict(dry)
                    if not (
                        dry.status == "arrived"
                        and dry.position_error_m <= 0.006
                        and abs(dry.yaw_error_rad) <= 0.015
                    ):
                        refusal = f"near_capture_misaligned:dry_run:{asdict(dry)}"
                        straight = None
                    elif getattr(options, "near_capture_neighbour_sweep", False):
                        # Plan 2026-10-22-rigid-session-pallets 2.6: a session's default straight is driven past the
                        # other pallets -- its dry run's empty outline swept against them (as planned: nominal + D1,
                        # or the estimate + its uncertainty); a meeting is refused to the redraw, which checks its own.
                        neighbours_ = list(hooks.session_neighbour_rects())
                        clear_ = (not neighbours_ or not len(dry.trajectory) or self._sweep_clear(
                            dry.trajectory, neighbours_))
                        state["near_capture"]["neighbour_sweep"] = {"neighbours": len(neighbours_), "clear": clear_}
                        if not clear_:
                            refusal = "near_capture_misaligned:neighbour_sweep"
                            straight = None
                redraw_entry = None
                if straight is None and self._near_capture_redraw_allowed():
                    # Plan D8 S4c-3 stage 1: the S4b re-draw acceptance at the near
                    # capture -- the heading box 0.12 rad with both dry runs' ends,
                    # their swept footprints and the correction points checked.
                    driven_config = acceptance_config
                    if options.approach_straight_speed_mps is not None:
                        driven_config = replace(acceptance_config,
                                                cruise_speed_mps=options.approach_straight_speed_mps)
                    redraw_plans, redraw_entry = near_capture_redraw(
                        rear, (float(base[0]), float(base[1]), float(yaw)), observation,
                        attempt_number, acceptance_config, driven_config, t)
                    if redraw_plans is not None:
                        straight = redraw_plans["approach"]
                        for name_ in ("insert", "extract"):
                            paths[name_] = redraw_plans[name_]
                            state["paths"][name_] = path_record(redraw_plans[name_])
                            trackers[name_] = RearAxlePathTracker(
                                redraw_plans[name_].poses, redraw_plans[name_].directions,
                                redraw_plans[name_].curvatures_inv_m, trackers[name_].config)
                    state["near_capture"].setdefault("recoveries", []).append(redraw_entry)
                if straight is not None and realign_active() and (
                        straight.length_m < near_tracking.REALIGN_MIN_LEFT_M):
                    # Plan D8 S4c-3 stage 2: the re-capture keeps at least 1.5 m of
                    # straight -- the approach starts before the near-field section.
                    refusal = f"near_capture_realign_short:{straight.length_m:.3f}"
                    straight = None
                if (straight is None and near is not None
                        and state["near_tracking"].get("realigns", 0) == 0):
                    # Plan D8 S4c-3 stage 2: once per pickup, instead of ending.
                    start_near_realign(t, attempt_number)
                    near_realign_ = True
                if not near_realign_:
                    require(straight is not None, refusal)
                    approach_config = acceptance_config
                    if options.approach_straight_speed_mps is not None:
                        # Plan D8b: only the driven cruise changes. The acceptance
                        # above stays the baseline's: at a slow cruise the tracker
                        # arrives where 1.5 x remaining meets the 0.012 m/s stop
                        # speed, ~8 mm short, past the 6 mm acceptance (smoke run
                        # seed 1 at 0.30: 7.76 mm). That dry run is kept as a record.
                        approach_config = replace(approach_config,
                                                  cruise_speed_mps=options.approach_straight_speed_mps)
                        state["near_capture"]["dry_run_driven"] = asdict(bicycle_rollout(
                            straight.poses, straight.directions, straight.curvatures_inv_m,
                            approach_config, rear,
                        ))
                    paths["approach"] = straight
                    state["paths"]["approach"] = path_record(straight)
                    hooks.write_paths()
                    hooks.show_path(straight, "Approach", (0.05, 0.45, 1.0))
                    trackers["approach"] = RearAxlePathTracker(
                        straight.poses,
                        straight.directions,
                        straight.curvatures_inv_m,
                        approach_config,
                    )
                    if options.approach_straight_speed_mps is not None:
                        record_tracker_configs()  # the straight's own cruise
                    if redraw_entry is not None:
                        # Plan D8 S4c-3: what was installed, by content and by the
                        # followers' own copies.
                        redraw_entry["installed"] = {
                            name_: {
                                "sha256": near_tracking.path_digest(
                                    paths[name_].poses, paths[name_].directions,
                                    paths[name_].curvatures_inv_m),
                                "tracker_sha256": near_tracking.path_digest(
                                    *trackers[name_].path_arrays()),
                                "poses": np.asarray(paths[name_].poses).tolist(),
                                "directions": np.asarray(paths[name_].directions).tolist(),
                                "curvatures_inv_m": np.asarray(paths[name_].curvatures_inv_m).tolist(),
                            }
                            for name_ in ("approach", "insert", "extract")
                        }
                    if options.pocket_check:
                        if acquired is None:
                            build_pocket_check(float(straight.poses[-1][2]), rear, base)
                        else:  # real-time plan D8b: the capture's depth at its own time and pose
                            build_pocket_check(float(straight.poses[-1][2]), rear, base, acquired[0], acquired[1])
                    state["near_capture"].update(
                        status="done",
                        near_attempt=attempt_number,
                        near_estimate_m=state["perception"][
                            "perception_pickup_estimate_m"
                        ],
                        near_error=state["perception"]["perception_error"],
                    )
                    if realign_active():
                        # Plan D8 S4c-3 stage 2: the re-capture accepted ends the
                        # realignment.
                        state["near_tracking"]["realign"]["end"] = {
                            "reason": "recaptured", "attempt": attempt_number, "time_s": t,
                            "length_left_m": float(straight.length_m),
                            # the observe follower the truck came by (the last install)
                            "observe_tracker_sha256": near_tracking.path_digest(
                                *trackers["observe"].path_arrays())}
            if not near_realign_:
                transition("approach", t)
                if near is not None and state.get("near_capture", {}).get("status") == "done":
                    build_near_tracker(t)  # in the hold the transition began

    def _reproject(self, t: float, applied_now, rear, loaded) -> None:
        """The grid re-projected with a new correction (the permission tick's statements)."""
        ms, obstacle, trackers = self.ms, self.obstacle, self.trackers
        old = tuple(float(v) for v in (obstacle.get("applied") or (0.0, 0.0, 0.0)))
        if self.cargo.recovery_context is not None and tuple(float(v) for v in applied_now) != old:
            # Plan D14 (Codex design round 4 P2): every applied correction change carries the cargo's map poses.
            self.cargo.reexpress(compose(tuple(float(v) for v in applied_now), invert(old)))
        obstacle["version"] += 1
        obstacle["applied"] = applied_now
        obstacle["reprojections"] = obstacle.get("reprojections", 0) + 1
        if ms.phase in trackers:
            leg_now, leg_dir_now = trackers[ms.phase].leg_ahead()
        else:  # a phase without a follower (lift, lower, settle): the truck's own place
            leg_now, leg_dir_now = np.array([tuple(float(v) for v in rear)]), 0
        obstacle["layer"].refresh(
            t, applied_now, obstacle["version"], current_pose=tuple(float(v) for v in rear),
            path_ahead=leg_now, loaded=loaded, direction=leg_dir_now,
        )

    def slam_replies(self, t: float, replies, loaded) -> None:
        """Real-time plan D9a: the SLAM replies in since the last tick, applied at this tick's start -- before any
        mission work, plan or permission reads the pose -- in order, each recorded with its scan's own record and
        what it changed; a changed applied correction re-projects the grid here, so this tick's pose and grid
        share it (Codex D9 1st P2-4)."""
        slam, obstacle, hooks = self.slam, self.obstacle, self.hooks
        tracker = slam["tracker"]
        before = tracker.applied
        # Scene-mode replan plan 2.4: the correction's contribution -- the same odometry under the correction before
        # and after -- per reply and over the tick (a scene run only; the others compute nothing new).
        # (the transforms composed with the same odometry pose -- not slam_rear, whose freshness check may refuse
        # before the reply that makes it fresh is in, Codex impl round 1 P2-1)
        watch_ = bool(getattr(self.options, "pickup_prior", False)) and replies and before is not None
        odom_ = tuple(float(v) for v in hooks.slam_odom_base()) if watch_ else None
        rear0_ = prev_ = (tuple(float(v) for v in compose(tuple(before[0]), odom_)) if watch_ else None)
        reply_max_ = 0.0
        for reply, meta, sent_mono, received_mono in replies:
            tracker.receive(reply.scan_id, reply.stamp_s, reply.status, reply.map_from_odom)
            if watch_ and tracker.applied is not None:
                now_ = tuple(float(v) for v in compose(tuple(tracker.applied[0]), odom_))
                reply_max_ = max(reply_max_, math.hypot(now_[0] - prev_[0], now_[1] - prev_[1]))
                prev_ = now_
            slam["records"].append({
                **meta, "link": "async", "status": reply.status, "map_from_odom": list(reply.map_from_odom),
                "received_after": list(tracker.received[0]) if tracker.received is not None else None,
                "applied_after": list(tracker.applied[0]) if tracker.applied is not None else None,
                "mode": tracker.mode, "hold": self.hold_count(), "sent_mono": sent_mono,
                "received_mono": received_mono, "applied_tick_s": float(t), "slam_wall_s": reply.slam_wall_s})
        if watch_:
            self.scene_correction_note(t, math.hypot(prev_[0] - rear0_[0], prev_[1] - rear0_[1]), reply_max_,
                                       math.remainder(prev_[2] - rear0_[2], math.tau))
        if obstacle is None or tracker.applied is None or tracker.applied is before:
            return
        applied_now = tuple(float(v) for v in tracker.applied[0])
        if applied_now != obstacle["applied"]:
            self._reproject(t, applied_now, hooks.slam_rear(t), loaded)

    def _selected_candidate_index(self) -> int:
        """The candidate the selected waypoint came from: the last one tried before the next, unless a scene observe
        stall tried later ones that failed and fell back to the waypoint (Codex impl round 2 P3)."""
        ms, selected = self.ms, list(self.state["observation_waypoint_selected"])
        for index_ in range(min(ms.next_candidate_index, len(self.options.observation_waypoints or ())) - 1, -1, -1):
            if list(self.options.observation_waypoints[index_]) == selected:
                return index_
        return ms.next_candidate_index - 1

    def _new_attempt(self, t: float, rear, step: int):
        """The observation attempt's record, appended (capture_and_detect's first statements)."""
        ms, state = self.ms, self.state
        attempt_number = len(state["observation_attempts"]) + 1
        attempt = {
            "candidate_index": self._selected_candidate_index(),
            "waypoint": list(state["observation_waypoint_selected"]),
            "arrived_pose": rear.tolist(),
            "pocket_observation": None,
            "frame_diagnostics": None,
            "detection_diagnostics": None,
            "capture_attempts": None,
            # Main-loop step and the t the next phase starts from:
            # together they fix the remaining time budget.
            "loop_step": step,
            "t_before_capture_s": t,
        }
        state["observation_attempts"].append(attempt)
        return attempt, attempt_number

    def _capture_failed(self, attempt: dict, exc) -> SimpleNamespace:
        """A capture failure: a STOP inside it pauses (``aborted``), anything else ends the run."""
        hooks = self.hooks
        attempt["capture_failure"] = exc.reason
        attempt["capture_diagnostics"] = asdict(exc.diagnostics)
        attempt["capture_attempts"] = exc.diagnostics.attempts
        live_abort = getattr(hooks, "live_abort", None)
        if live_abort is not None and live_abort():
            # A STOP inside a failing capture (the truck stopped mid-capture) pauses, it does not end the run.
            attempt["aborted"] = "live_stop"
            return SimpleNamespace(aborted=True)
        if self.bottom_deck_returning():
            # The re-observation could not be captured: refused by name, and the runner ends the loop (Codex return
            # design round 1 P2 4 -- not a bare require).
            self._bottom_deck_refuse(float(hooks.now_s()), "bottom_deck_reobservation_failed",
                                     None, stage="reobserve")
            self.state["bottom_deck"]["refused"]["detail"] = f"capture_failed:{exc.reason}"
            self.state["bottom_deck"].setdefault("reobservation_attempts", []).append(
                {"time_s": float(hooks.now_s()), "outcome": "capture_failed", "reason": exc.reason})
            return SimpleNamespace(aborted=False, refused=True)
        hooks.require(False, f"perception_capture_failed:{exc.reason}")

    # --- real-time plan D9c: the capture over control ticks (live runs with the planning worker only) ----------------
    def capture_begin(self, t: float, rear, step: int):
        """Open the capture at the observe arrival and run it to its first render step (the guard, the start pose
        and time taken now). None: it waits for the render steps; else the failure's result (``aborted``)."""
        attempt, attempt_number = self._new_attempt(t, rear, step)
        session = self.hooks.perception_capture().session(max_attempts=self.options.perception_max_attempts)
        self.capture_live = SimpleNamespace(session=session, attempt=attempt, attempt_number=attempt_number, raw=None)
        try:
            session.start()
        except self.options.adapter.CaptureFailure as exc:
            self.capture_live = None
            return self._capture_failed(attempt, exc)
        return None

    def capture_stepped(self, stopped: bool):
        """Right after a render step, before the next tick's start (the step's own STOP is still known): a STOP
        or an expiry at the step ends the capture as the blocking one did (``aborted``); else the capture reads
        on, and once it accepted a frame ``capture_live.raw`` holds it for the next tick (``capture_and_detect``
        with ``raw``)."""
        c = self.capture_live
        if stopped:
            c.session.close()
            c.attempt["aborted"] = "live_stop"
            self.capture_live = None
            return SimpleNamespace(aborted=True)
        try:
            if c.session.stepped():
                c.raw = c.session.result
        except self.options.adapter.CaptureFailure as exc:
            self.capture_live = None
            return self._capture_failed(c.attempt, exc)
        return None

    def capture_abandon(self) -> None:
        """A pause or a cancel while the capture runs: it is dropped; the arrival opens a new one when running."""
        if self.capture_live is not None:
            self.capture_live.session.close()
            self.capture_live.attempt["aborted"] = "paused"
            self.capture_live = None

    FAR_LEG_RECAPTURES = 1  # a live far stop whose leg ended before its decision: captured again once (T1 (A) 3)

    def observation_freeze(self, t: float, captured) -> None:
        """D9c: the observation is handled on the next tick; what it rests on is fixed now (Codex D9 1st P1). A
        live far stop's decision is still being worked on: the work clocks stand until it is taken or dropped."""
        far = getattr(captured, "far_pending", None)
        self.observation_live = SimpleNamespace(captured=captured, applied=self._applied_now(),
                                                gen=self.commands.mission_gen, phase=self.ms.phase,
                                                acquired_s=float(self.hooks.now_s()), t=float(t), far=far,
                                                far_wait=False)
        if far is not None and not (self.commands.waits and self.commands.waits[-1][1] is None):
            self.commands.begin_wait(t)
            self.observation_live.far_wait = True

    def _far_wait_end(self, o, t: float) -> None:
        if getattr(o, "far_wait", False):
            self.commands.end_wait(t)
            o.far_wait = False

    def observation_drop(self, t: float, why: str) -> None:
        """A pause, a cancel or a mission end drops the frozen observation; a far decision still pending is
        dropped with it (``recognition_discarded``) -- the arrival captures again when running."""
        o, self.observation_live = self.observation_live, None
        if o is None:
            return
        self._far_wait_end(o, t)
        if getattr(o, "far", None) is not None:
            cancel = getattr(self.hooks, "far_recognition_cancel", None)
            if cancel is not None:
                cancel(why)
            self.state.setdefault("recognition_discarded", []).append(
                {"time_s": float(t), "captured_s": o.t, "why": why})

    def _far_take(self, t: float, o):
        """A frozen far stop's decision: None while it is worked on (the observation stays frozen), else the
        captured namespace with the decided observation -- or with a ``no_pallet`` one once its leg ended twice
        (observe failed: the next candidate). A leg that ended once drops the observation and captures again."""
        got = self.hooks.far_recognition_poll(o.far)  # may raise recognition_timeout / recognition_error:
        if got.state == "waiting":
            return None
        self.observation_live = None
        self._far_wait_end(o, t)
        captured = o.captured
        if got.state == "done":
            self.far_leg_recaptures = 0
            captured.observation = got.observation
            self._far_adopt(captured.attempt, got.record)
            captured.attempt["pocket_observation"] = asdict(got.observation)
            return captured
        self.state.setdefault("recognition_discarded", []).append(
            {"time_s": float(t), "captured_s": o.t, "why": got.why})
        if got.why != "leg":
            return False  # a STOP or cancel cleared it: nothing to take, the pause handles the rest
        self.far_leg_recaptures = getattr(self, "far_leg_recaptures", 0) + 1
        if self.far_leg_recaptures <= self.FAR_LEG_RECAPTURES:
            return False  # captured again where the truck stands, in the new leg
        self.far_leg_recaptures = 0
        captured.observation = got.observation
        captured.attempt["observation_source"] = "none"
        captured.attempt["recognition"] = {"status": "invalid", "reason": got.observation.reason, "source": None}
        captured.attempt["pocket_observation"] = asdict(got.observation)
        return captured

    def observation_take(self, t: float):
        """The fixed observation if nothing it rests on changed (the correction, the run, the phase); else it is
        dropped (counted, 3 at most per run) and None -- the arrival captures again. A live far stop's decision
        is waited for first (None, the observation kept frozen)."""
        o = self.observation_live
        if getattr(o, "far", None) is not None:
            taken = self._far_take(t, o)
            if taken is None or taken is False:
                return None
        self.observation_live = None
        why = ("mission" if self.commands.status != "running" or self.commands.mission_gen != o.gen
               else "phase" if self.ms.phase != o.phase
               else "correction" if self._applied_now() != o.applied else None)
        if why is None:
            self.capture_acquired_s = o.acquired_s
            return o.captured
        drops = self.state.setdefault("observation_drops", [])
        drops.append({"time_s": float(t), "captured_s": o.t, "why": why})
        if getattr(o, "far", None) is not None:  # a far decision taken, then dropped with its observation
            self.state.setdefault("recognition_discarded", []).append(
                {"time_s": float(t), "captured_s": o.t, "why": why})
        self.hooks.require(len(drops) <= self.ASYNC_MAX_DISCARDS, f"observation_discarded:{why}")
        return None

    def capture_and_detect(self, t: float, rear, step: int, raw=None) -> SimpleNamespace:
        """The observe leg's arrival with perception: the attempt record, the capture (a port that steps the
        physics until the truck stands, refused on a capture failure), its diagnostic dumps (the runner's), the
        accepted pose as the capture base -- under SLAM the hold let go (except before the near capture) and
        the control pose taken from the estimate at this instant, kept as the runner's mixed sample (q the
        accepted truth, base/yaw/forward/rear the estimate's) -- the detector input and the detection, and the
        re-observation stand's check. Returns everything the runner reads after it."""
        ms, state, slam, pocket, hooks, options, near = (self.ms, self.state, self.slam, self.pocket, self.hooks,
                                                         self.options, self.near)
        adapter, G2, yaw_and_tilt = options.adapter, options.G2, options.yaw_and_tilt
        perception_capture = hooks.perception_capture()
        require, slam_odom_base, slam_rear, slam_error, detect_pockets = (
            hooks.require, hooks.slam_odom_base, hooks.slam_rear, hooks.slam_error, hooks.detect_pockets)
        rear_stamp = None  # set only under SLAM, as the runner's was
        if self.bottom_deck_returning() and (state["bottom_deck"].get("leg") or {}).get("kind") == "return":
            state["bottom_deck"]["leg"] = None  # arrived back: this is the re-observation (still "returning")
        if raw is None:
            attempt, attempt_number = self._new_attempt(t, rear, step)
            try:
                scene_input, frame_diagnostics, capture_attempts = (
                    perception_capture.capture(
                        max_attempts=options.perception_max_attempts,
                    )
                )
            except adapter.CaptureFailure as exc:
                return self._capture_failed(attempt, exc)
        else:  # D9c: the capture ran over the ticks before this one (capture_begin, capture_stepped)
            attempt, attempt_number, (scene_input, frame_diagnostics, capture_attempts) = raw
        live_abort = getattr(hooks, "live_abort", None)  # M-c: set only by the live runner
        if live_abort is not None and live_abort():
            # A STOP or an expiry at a physics step inside the capture (M-c plan C2): nothing after the raw capture
            # runs -- no SLAM release, no detection, no re-observation state; the next tick pauses the mission.
            attempt["aborted"] = "live_stop"
            return SimpleNamespace(aborted=True)
        attempt["depth_sha256"] = G2.depth_sha256(scene_input.depth_m)
        pocket["capture"] = scene_input
        # Diagnostic dump for offline root-cause analysis (a Sink, the runner's statements).
        hooks.save_capture(attempt_number, scene_input)
        # Use the accepted end pose as the stationary acquisition
        # representative; no timestamp interpolation is implied.
        capture_diagnostics = perception_capture.state.diagnostics
        attempt["capture_diagnostics"] = asdict(capture_diagnostics)
        attempt["accepted_pose"] = [
            np.asarray(v, dtype=float).tolist()
            for v in capture_diagnostics.accepted_pose
        ]
        base, q = map(np.asarray, capture_diagnostics.accepted_pose)
        yaw, _ = yaw_and_tilt(q)
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        rear = np.array(
            [
                base[0] - abs(options.rear_axle_offset_m) * forward[0],
                base[1] - abs(options.rear_axle_offset_m) * forward[1],
                yaw,
            ]
        )
        if slam is not None:
            now_s = hooks.now_s()
            near_pending = (
                state.get("near_capture", {}).get("status") == "pending"
            )
            if slam["tracker"].mode == "holding" and not near_pending:
                # An observe leg ends held (below). Anything but the
                # near capture is followed by more driving, so let
                # SLAM back in now, while the truck stands still
                # for the capture (plan v3.6).
                jump_m, jump_rad = self.release_slam(slam_odom_base())
                slam["holds"].append(
                    {
                        "phase": ms.phase,
                        "time_s": now_s,
                        "event": "release_at_capture",
                        "jump_m": jump_m,
                        "jump_rad": jump_rad,
                    }
                )
                if self.bottom_deck_on():
                    # Plan 2026-10-19 (Codex return design round 2 P2 1): the plan made right after this capture --
                    # the side legs from the first far one, the mission from the re-observation -- reads the grid at
                    # the released correction, not the held one. Off runs keep what they always did.
                    slam["holds"][-1]["reprojected"] = self._reproject_after_release(
                        now_s, slam_rear(now_s), hooks.loaded())
            # The accepted ground-truth pose only validated the
            # capture; control takes the estimate at this instant.
            truth_rear_capture = [float(v) for v in rear]
            rear = slam_rear(now_s)
            rear_stamp = now_s
            yaw = float(rear[2])
            forward = np.array([math.cos(yaw), math.sin(yaw)])
            base = np.array(
                [
                    rear[0] + abs(options.rear_axle_offset_m) * forward[0],
                    rear[1] + abs(options.rear_axle_offset_m) * forward[1],
                    float(base[2]),
                ]
            )
            attempt["control_pose_source"] = "slam_estimate"
            attempt["slam_error"] = slam_error()
            # Plan D8 S4c-2 (x) v3: the control sample of this instant (the
            # control record has none -- the capture ends between loop ticks).
            attempt["capture_control"] = {
                "time_s": now_s,
                "control_rear": [float(v) for v in rear],
                "truth_rear": truth_rear_capture,
            }
        attempt.update(
            {
                "frame_diagnostics": asdict(frame_diagnostics),
                "capture_attempts": capture_attempts,
            }
        )
        prior = options.pallet_prior_loaded
        params = DetectorParams.derived_for(prior)
        state["detector_params"] = asdict(params)
        # B1c: the detector sees millimetre depth when asked; the
        # saved depth above stays raw (G3 applies the same rounding).
        detector_input = adapter.detector_input(
            scene_input, options.depth_quantize_mm
        )
        attempt["base_from_optical"] = {
            "translation_m": np.asarray(
                scene_input.base_from_optical.translation_m
            ).tolist(),
            "rotation": np.asarray(
                scene_input.base_from_optical.rotation
            ).tolist(),
        }
        attempt["depth_quantize_mm"] = options.depth_quantize_mm
        # The far recognition (--recognition far, docs/plans/2026-10-18-isaac-recognition-far-capture.md): only the
        # far observation -- not the near capture, not a re-observation stand. The detector still runs first, in
        # the same call order, and stays as the reference; the mission reads the new recognition's observation.
        far_hook = getattr(hooks, "far_recognition", None)
        far_capture = (far_hook is not None and state.get("near_capture", {}).get("status") != "pending"
                       and not (near is not None and near.get("retry_stand")))
        waive_kw = {}
        if self._bottom_deck_lower_waived():
            # Plan 2026-10-21 task C: the side faces proved the bottom deck of the pallet this near capture looks at
            # (record present, the re-observation checked against it); the near detector's lower band -- a
            # sub-millimetre sign test on the face's own bottom board, no discrimination against deckless shapes --
            # no longer gates it. The count is still recorded; the estimate is checked against the record before
            # adoption (_bottom_deck_near_check).
            attempt["lower_evidence"] = False
            waive_kw = {"lower_evidence": False}
        if self._near_expected_on():
            # Plan 2026-10-22 task A: the near capture looks for the pallet the mission is approaching -- the
            # planned pickup in the control base this capture was placed with -- not for the strongest pattern in
            # view (aisle_out picked the neighbour 0.94 m away). Continuity of the far choice, not identity.
            site = self._near_expected_site(base, yaw)
            detection, candidates_ = hooks.detect_pockets_expected(detector_input, prior, params, site, **waive_kw)
            attempt["expected"] = self._expected_record(site, candidates_)
            if not waive_kw and detection.observation.status != "valid" and self._near_waiver_on():
                detection = self._near_lower_waiver(attempt, detection, site, detector_input, prior, params)
        else:
            detection = detect_pockets(detector_input, prior, params, **waive_kw)
            if self._near_waiver_on() and detection.observation.status == "valid":
                # Plan 2026-10-22 task C condition 1: the pallet this capture's own detector saw, in the control
                # frame of this capture -- a far capture's adopted estimate may be the learned recognition's.
                depth_ = float(options.pallet_prior_loaded.overall_depth_m)
                site_ = options.adapter.estimate_world_pallet_site(
                    options.adapter.estimate_pallet_center_m(detection.observation, depth_),
                    detection.observation.insertion_yaw_rad, (float(base[0]), float(base[1])), float(yaw))
                attempt["detector_site"] = [float(site_.x_m), float(site_.y_m), float(site_.yaw_rad)]
        observation = detection.observation
        far_pending = None
        if far_capture:
            reference = observation
            attempt["detector_reference"] = asdict(reference)
            far_submit = getattr(hooks, "far_recognition_submit", None)
            if far_submit is not None:
                # A live run (plan 2026-10-22-live-far-recognition T1 (A)): the decision's work goes to the far
                # worker; the observation is frozen with it (observation_freeze) and taken once it is decided
                # (observation_take). The detector's stays as the reference only, never adopted.
                far_pending = far_submit(scene_input, base, yaw, reference)
                attempt["observation_source"] = "pending"
            else:
                observation, far_record = far_hook(scene_input, base, yaw, reference)
                self._far_adopt(attempt, far_record)
        if near is not None and near.get("retry_stand"):
            # Plan D8 S4c-2 / S4c-3: the re-observation must see the pallet; no
            # candidate moves on old estimates.
            near["retry_stand"] = False
            stand_key_ = near.get("stand_record", "retry")
            state["near_tracking"][stand_key_].update(
                capture_attempt=attempt_number, capture_status=observation.status)
            if observation.status != "valid":
                reason_ = ("near_field_retry_no_detection" if stand_key_ == "retry"
                           else "near_capture_realign_no_detection")
                state["near_tracking"]["outcome"] = reason_
                require(False, reason_)
        attempt.update(
            {
                "pocket_observation": asdict(observation),
                "frame_diagnostics": asdict(frame_diagnostics),
                "detection_diagnostics": asdict(detection.diagnostics),
                "capture_attempts": capture_attempts,
            }
        )
        return SimpleNamespace(attempt=attempt, attempt_number=attempt_number, scene_input=scene_input,
                               frame_diagnostics=frame_diagnostics, capture_attempts=capture_attempts,
                               capture_diagnostics=capture_diagnostics, base=base, q=q, yaw=yaw, forward=forward,
                               rear=rear, rear_stamp=rear_stamp, prior=prior, params=params, detection=detection,
                               observation=observation, far_pending=far_pending)

    @staticmethod
    def _far_adopt(attempt: dict, far_record: dict) -> None:
        """The far recognition's decision on the attempt's record."""
        attempt["observation_source"] = far_record.get("source") or "none"
        attempt["recognition"] = {k: far_record.get(k) for k in (
            "status", "reasons", "reason", "source", "observations", "selected_track", "frames", "drive_frames",
            "used_error", "single_error", "detector_error", "lag_fit", "leg",
            "used", "decided")}  # the candidate read: its id, hypothesis, axis and support are the side evidence's target

    # --- M-a3: the paused tick and the resume (docs/plans/2026-10-16-m-a3-mission-commands.md E3-E6) ---
    def permission_watch(self, t: float, rear, kappa: float, loaded: bool) -> SimpleNamespace:
        """The permission while paused (E3): the grid re-projected after a correction change, the layer's limit
        with its emergency stop and sensor silence, and the cap the command assembly steps to -- every tick,
        whatever the speed. No work counter (path blocked, blocked and standing), no progress watch, no N9 end:
        a silent sensor is reported and keeps the resume from finishing."""
        ms, obstacle, slam, trackers, hooks, options = (self.ms, self.obstacle, self.slam, self.trackers, self.hooks,
                                                        self.options)
        truth_speed = hooks.truth_speed()
        applied_now = (tuple(float(v) for v in slam["tracker"].applied[0])
                       if slam is not None and slam["tracker"].applied is not None else (0.0, 0.0, 0.0))
        if applied_now != obstacle["applied"]:
            obstacle["version"] += 1
            obstacle["applied"] = applied_now
            obstacle["reprojections"] = obstacle.get("reprojections", 0) + 1
            if ms.phase in trackers:
                leg_now, leg_dir_now = trackers[ms.phase].leg_ahead()
            else:  # no follower in this phase: the pose itself, as the physics-tick scan does
                leg_now, leg_dir_now = np.array([tuple(float(v) for v in rear)]), 0
            obstacle["layer"].refresh(t, applied_now, obstacle["version"], current_pose=tuple(float(v) for v in rear),
                                      path_ahead=leg_now, loaded=loaded, direction=leg_dir_now)
        allowed, why = obstacle["layer"].limit(
            t, current_pose=tuple(float(v) for v in rear), curvature_inv_m=kappa,
            direction=1 if truth_speed > 0 else -1, loaded=loaded, cap_mps=abs(truth_speed),
        )
        self.last_permission = (float(allowed), why)  # for the safety state's reason (E8)
        acting = False
        if ms.phase in trackers:
            acting, docking_straight = self.permission_acting()
            # D5: the approach straight and the insertion act too (as in the permission tick) -- their pocket
            # work does not run while paused, the layer's own cap does.
            acting = acting or bool(options.pocket_check and (
                ms.phase == "insert" or (ms.phase == "approach" and docking_straight)))
        obstacle["permission_cap"] = (t, float(allowed)) if (options.obstacle_act and acting) else None
        return SimpleNamespace(allowed=allowed, why=why, acting=acting,
                               obstacle_hold=bool(options.obstacle_act and acting and allowed == 0.0))

    def paused_tick(self, t: float, rear, kappa: float, loaded: bool) -> SimpleNamespace:
        """One control tick while the mission is paused, resuming or cancelling (E3, E5, E6): nothing of the
        mission's own work runs (no follower, no arrival, no lift ramp, no replan); the permission watches, a
        lift on its way is held where it stood at the stop's first tick, and the stop is watched. A cancel ends once standing; a resume
        steps through its checks. Returns ``requested_speed`` (0), ``obstacle_hold``, ``stop`` (the run ends) and
        ``rear`` (a replan inside the resume may move it)."""
        ms, slam, obstacle, hooks, cmds = self.ms, self.slam, self.obstacle, self.hooks, self.commands
        standing = bool(slam["stop_now"]) if slam is not None else abs(float(hooks.truth_speed())) < 0.01
        if cmds.pause_hold_count is None:  # M-a3c: the stop's hold, as it began (R3, R4n, R11, R9/R14)
            cmds.pause_hold_count = self.hold_count()
            cmds.pause_holding = slam is not None and slam["tracker"].mode == "holding"
            cmds.pause_release_pending = slam is not None and slam.get("pending_release") is not None
            if ms.phase in ("lift", "lower", "recover_lower"):
                # E3/R7: a lift on its way stops where it is -- the target is set to the measured lift once, at the
                # stop's first tick, and held. Synchronising every tick ratcheted a loaded lift down by its sag
                # (Isaac ma3_p_tr: 2.8 mm a tick, the pallet dropped 0.27 s into a transport pause). A lift that is
                # not being moved (transport and the rest) keeps its target.
                measured_ = float(hooks.lift_position())
                tolerance_ = self.lift_hold_tolerance() if self.lift_hold_tolerance is not None else None
                if self.baseline is not None and tolerance_ is not None:
                    # Live (plan D14, main 2026-10-19; Codex impl round 5 P1): as the gate's latch -- the target
                    # within the hold tolerance of the measurement, so a loaded sag is not taken for a new target.
                    ms.lift_command = float(min(measured_ + tolerance_, max(measured_ - tolerance_,
                                                                              float(ms.lift_command))))
                else:
                    ms.lift_command = measured_
        if standing and cmds.stop_seen_s is None:
            cmds.stop_seen_s = t
            cmds.stop_pose = tuple(float(v) for v in rear)  # R11 measures a move from where it stood, not braking
        # E3: not standing 10 s after the stop began -- reported for the arbiter (the run goes on).
        cmds.not_stopped = (cmds.status != "held" and cmds.stop_seen_s is None and cmds.paused_since is not None
                            and t - cmds.paused_since >= NOT_STOPPED_S)  # held: the operator drives (M-b B6)
        obstacle_hold = False
        if obstacle is not None:
            obstacle_hold = self.permission_watch(t, rear, kappa, loaded).obstacle_hold
        stop = False
        applied_zero = abs(float((self.state.get("applied_command") or (0.0,))[0])) < 1e-12
        if cmds.status == "cancelling" and standing and applied_zero:
            # Cancelled once standing and the wheels already commanded to zero (the slew may still be stepping
            # down while the stop detector reads standing): this tick's output is zero too. Live (plan D4): the forks
            # still in a pocket back out first; without the evidence to, the mission needs a recovery by hand.
            decision = (self.cargo.cancel_decision(rear, in_withdraw=ms.phase == "withdraw")
                        if self.baseline is not None else "cancelled")
            if decision == "recover" and getattr(self.hooks, "test_recovery_blocked", False) == "lower":
                # Plan D14 tests only (U20 c, lifted): the recovery refused before its lowering, the pallet left on
                # the forks for the manual lowering and ground verdict.
                cmds.begin_recovery(t)
                self.recovery_failed(t, "blocked:test_lower")
                stop = True
            elif decision == "recover":
                cmds.begin_recovery(t)
                self.transition("recover_lower", t)
            elif decision == "recovery_required":
                cmds.terminate("recovery_required", t, "recovery:" + ("trace_invalid" if not self.cargo.trace_valid
                                                                    else self.cargo.relation))
                stop = True
            elif self.baseline is not None and self.cargo.relation == "free":
                # plan D4b: empty forks end through the next preparation's start check; a loaded cancel keeps its
                # cargo where it stands (plan D5a: cargo_held -- Codex D4b impl round 1 P1-1)
                stop = self.finish_cancel(t, rear)
            else:
                cmds.cancelled(t)
                stop = True
        elif cmds.status == "resuming":
            rear = self.resume_step(t, rear, loaded, standing)
        return SimpleNamespace(requested_speed=0.0, obstacle_hold=obstacle_hold, stop=stop, rear=rear)

    def resume_step(self, t: float, rear, loaded: bool, standing: bool):
        """The resume's checks this tick (E5), by the resume table's rows (M-a0.5 part 2, section 8): standing
        first; a stop that began held keeps its hold (mode and episode) unless a pending release was due; a held
        realignment does not resume (R3x); a docking straight moved off since the stop retries the docking or does
        not resume (R11); a phase without a follower (lift, lower, settle) and a re-observation stand (R2a) resume
        as they are; otherwise the follower must rejoin (a copy updated once from here must not fail) and, where
        the permission acts, the path ahead must be clear and the sensors alive -- a free travel leg (R1, R2b, R10,
        R12) is replanned from here by its obstacle recovery, a held one (R3, R4n) or one without a recovery (R8,
        R13) does not resume; with the D5 check its own answer must allow the move. ``no_resume`` puts the mission
        back to paused (a new resume is needed); a wait stays resuming. Returns the control pose."""
        import copy

        ms, obstacle, trackers, cmds, options = self.ms, self.obstacle, self.trackers, self.commands, self.options
        slam, near, pocket = self.slam, self.near, self.pocket
        cmds.resume_check = check = {"time_s": t, "step": "stop_confirmed"}
        if not standing:
            return rear

        def refuse(reason):
            check["reason"] = reason
            check["refused"] = True  # kept with the check: later commands must not hide that this resume failed
            cmds.refuse_resume(t, reason)
            return rear

        def accept_hold_state():
            # A release the resume's own recovery made (a docking retry, a re-approach or backoff replan) is the
            # expected state from here on: the hold checks take its mode and episode as their reference.
            cmds.pause_holding = slam is not None and slam["tracker"].mode == "holding"
            cmds.pause_hold_count = self.hold_count()

        check["step"] = "state_verified"
        if self.near_active():
            # M-b B5: a stop inside the near-field contract (an arbiter STOP; a pause is refused there) is not resumed
            # -- pausing near tracking (R4-R6) is not built; what follows is a recovery action.
            return refuse("no_resume:near_tracking")
        if ms.phase in ("lift", "lower") and abs(float(self.hooks.lift_position()) - float(ms.lift_command)) > LIFT_RESUME_TOL_M:
            return refuse("state_verified:lift_mismatch")  # R7: the lift is not where the stop held it
        holding_now = slam is not None and slam["tracker"].mode == "holding"
        if ms.phase == "clear_out" and slam is not None and not holding_now:
            self.recovery_failed(t, "clear_out:hold_released")  # plan D4b: the back-out moves in its hold only
            return rear
        if cmds.pause_holding and not cmds.pause_release_pending:
            if not holding_now:
                return refuse("state_verified:hold_state_mismatch")
            if self.hold_count() != cmds.pause_hold_count:
                return refuse("state_verified:hold_episode_changed")
        if ms.phase == "observe" and holding_now and self.realign_active():
            return refuse("no_resume:realign_held")  # R3x: no recovery runs held in a realignment
        docking_ = slam["docking"] if slam is not None else {}
        recovering_ = obstacle is not None and obstacle.get("backoff", {}).get("phase") == ms.phase
        straight_row = (ms.phase == "transport" and holding_now and not docking_.get("reapproach")
                        and docking_.get("status") in ("armed", "done", "fallback") and not recovering_)
        if straight_row and cmds.stop_pose is not None:
            # R11: the docking straight was dry-run and swept from where the match ran; moved since the stop,
            # it is not that straight -- a D7c docking retry from here (its budget first), or no resume.
            dx_, dy_ = float(rear[0]) - cmds.stop_pose[0], float(rear[1]) - cmds.stop_pose[1]
            dyaw_ = math.atan2(math.sin(float(rear[2]) - cmds.stop_pose[2]), math.cos(float(rear[2]) - cmds.stop_pose[2]))
            if math.hypot(dx_, dy_) > PAUSE_MOVED_M or abs(dyaw_) > PAUSE_MOVED_RAD:
                if not options.d7_docking:
                    return refuse("no_resume:docking_moved")
                if sum(1 for r_ in self.state.get("docking_retries", []) if r_.get("counted")) >= 2:
                    return refuse("no_resume:docking_retry_budget")
                self.start_docking_retry(t, "resume", "docking_moved_while_paused")
                rear = self.hooks.slam_rear(t)  # the retry released the correction
                cmds.stop_pose = tuple(float(v) for v in rear)  # the re-approach starts here
                accept_hold_state()
                check["reason"] = "docking_retry"
                return rear
        retry_stand = near is not None and near.get("retry_stand") and ms.phase == "observe"
        if ms.phase == "clear_out" and self._recovery_install_gen != cmds.mission_gen:
            # Plan D4b: the back-out goes on from its path, the progress the follower made along it and the budget
            # (the hold kept -- checked before the general hold rows).
            path_ = self.paths.get("clear_out")
            remaining = None
            if path_ is not None and "clear_out" in trackers:
                poses_ = np.asarray(path_.poses, dtype=float)
                cum_ = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(poses_[:, 0]), np.diff(poses_[:, 1])))))
                done_ = float(path_.length_m) - float(trackers["clear_out"].remaining_to_goal_m())
                remaining = self._rest_from(poses_, cum_, done_, rear)
            if remaining is None or len(remaining) < 2:
                self.clear_out_arrived(t, rear)
            else:
                # The progress made plus the rest is the planned path; the way back onto it from where the truck
                # stands now is part of the path the permission checks (Codex D4b impl round 2 P2-2), and only what it
                # adds beyond the rest the follower had left is new travel for the budget (round 3: a stop between
                # samples is still on the path).
                here_ = tuple(float(v) for v in rear)
                join_ = float(math.hypot(remaining[0][0] - here_[0], remaining[0][1] - here_[1]))
                if join_ > 1e-3:
                    remaining = np.vstack((np.asarray([here_]), remaining))
                new_ = float(np.sum(np.hypot(np.diff(remaining[:, 0]), np.diff(remaining[:, 1]))))
                added_ = max(0.0, new_ - float(trackers["clear_out"].remaining_to_goal_m()))
                if self.clear_out_used_m + added_ > CLEAR_OUT_BUDGET_M + 1e-9:
                    self.recovery_failed(t, "clear_out:budget")
                    return rear
                if self._install_retreat(t, rear, remaining, "clear_out"):
                    self.clear_out_used_m += added_
            if cmds.status != "resuming":
                return rear
        if ms.phase in ("recover_lower", "recover_withdraw") and not self.cargo.trace_valid:
            self.recovery_failed(t, "trace_invalid")  # the evidence went while paused (Codex 6b impl P1-3)
            return rear
        if ms.phase == "recover_withdraw" and not (self.hooks.lift_position() < 0.012
                                                   and abs(self.hooks.pallet_z()) < 0.008):
            self.transition("recover_lower", t)  # not on the floor any more: lower again first
            cmds.resumed(t)
            return rear
        if ms.phase == "recover_withdraw" and self._recovery_install_gen != cmds.mission_gen:
            # Plan D4: the retrace made again from the depth now (the trace's placement does not move), once per
            # resume -- a resume that waits does not install it every tick (Codex 6b impl round 2 P2); it may end
            # here (already out) or refuse (blocked).
            self.install_recover_withdraw(t, rear)
            if cmds.status != "resuming":
                return rear
        if ms.phase in trackers and not retry_stand:
            check["step"] = "path_rechecked"
            probe = copy.deepcopy(trackers[ms.phase])
            rejoined = probe.update(rear, 0.0, options.dt).status != "failed"
            acting, docking_straight = self.permission_acting() if obstacle is not None else (False, False)
            pocket_row = bool(options.pocket_check and (
                ms.phase == "insert" or (ms.phase == "approach" and docking_straight)))
            held_observe = ms.phase == "observe" and holding_now  # R3
            free_leg = ms.phase in ("observe", "transport", "return_home") and not held_observe and not straight_row
            blocked, silent, verified_m = None, False, None
            if obstacle is not None and acting and not straight_row:
                silent = not self.cusp_alive(t)
                leg_, dir_ = trackers[ms.phase].leg_ahead()
                check_ = obstacle["layer"].check_candidate(leg_, current_pose=tuple(float(v) for v in rear),
                                                           direction=dir_, loaded=loaded)
                blocked, verified_m = check_.blocked, check_.verified_m
            check.update(rejoined=rejoined, blocked=blocked, silent=silent, free_leg=free_leg)
            if silent:
                check["reason"] = "path_blocked:sensor_silent"
                return rear
            if not rejoined or blocked == "occupied":
                if not free_leg or obstacle is None:
                    return refuse("no_resume:" + ("rejoin_failed" if not rejoined else "path_blocked:occupied"))
                if t < cmds.resume_retry_s:
                    return rear  # a replan without a path waits a second (E5 (c))
                before_path = self.paths.get(ms.phase)
                out = self.blocked_replan(t, rear, 0.0, 0.0)
                rear = out.rear
                accept_hold_state()  # a release the recovery made (re-approach, backoff) is expected from here
                installed = self.paths.get(ms.phase)
                if installed is before_path:
                    cmds.resume_retry_s = t + 1.0
                    check["reason"] = "replan_waiting"
                    return rear
                if installed.status not in ("backoff", "escape"):
                    check["reason"] = "replanned"  # (a) a new path: rejoined and checked again next tick
                    return rear
                check["reason"] = "recovery_installed"  # (b) the backoff machinery drives it from here
            elif blocked is not None and obstacle["layer"].permission.config.stopping.speed_for(verified_m) <= 0.0:
                # Unknown or an edge so close that the permission would allow nothing: wait for evidence. Farther
                # out it resumes as a running leg drives -- up to the verified distance, the permission slowing it
                # (Isaac ma3b_p_tr: a carried pallet hides the floor ahead, standing never sees it, and a resume
                # waiting for the whole leg to be known waited 300 s).
                check["reason"] = f"path_blocked:{blocked}"
                return rear
            elif pocket_row:
                # R4n with D5: the depth check's own answer for the move from here.
                if pocket["check"] is None or pocket.get("await_still_frame"):
                    check["reason"] = "path_blocked:pocket_settle"
                    return rear
                lp_, wp_ = pocket["check"].limit(t, tuple(float(v) for v in rear), 0.0, 1,
                                                 trackers[ms.phase].config.cruise_speed_mps,
                                                 float(self.hooks.lift_position()))
                if lp_ <= 0.0:
                    check["reason"] = f"path_blocked:pocket:{wp_}"
                    return rear
        length = cmds.resumed(t)
        check["step"] = "running"
        if obstacle is not None and "live_retry_after_s" in obstacle:
            obstacle["live_retry_after_s"] += length  # E4: a deadline-type anchor moves by the pause
        monitor = self.hooks.cusp_monitor() if hasattr(self.hooks, "cusp_monitor") else None
        if monitor is not None:
            monitor.shift_time(length)
        self.scene_replan.shift_time(length)
        if ms.phase in trackers:
            trackers[ms.phase].restart_speed_slew(0.0)  # from standing, not from the held cruise
        return rear

    def near_active(self) -> bool:
        """The near-field contract applies this tick: a near tracker exists and the phase is one ``near_tick`` runs in
        (approach, insert)."""
        near = self.near
        return near is not None and near.get("tracker") is not None and self.ms.phase in ("approach", "insert")

    def release_slam(self, odom_from_base):
        """The one place the core releases a SLAM hold: applies the received correction and counts the frame change
        the targets list follows (M-a4). Returns the tracker's (position, yaw) jump."""
        jump = self.slam["tracker"].release(odom_from_base=odom_from_base)
        self.slam_releases += 1
        return jump

    def refresh_targets(self) -> None:
        """M-a4 (plan D7): the targets list follows the mission each control tick, from what the core holds (no
        port call). The correction version counts the SLAM releases (every ``tracker.release`` the core makes, whatever event
        records it) -- the frame changes the core re-expresses its
        estimates across (while tracking, the correction moves with every scan and the map poses do not jump).
        The destination follows the core's current estimate of it (a docking match, a release's re-expression);
        the pallet takes its recognised estimate once seen, is carried from the lift to the lowering (no pose) and
        is put down where that estimate stands, which it then follows."""
        tb, ms, slam, pocket = self.targets, self.ms, self.slam, self.pocket
        mission = self.commands.mission
        if mission is None or mission.pallet_id not in tb.pallets or mission.destination_id not in tb.destinations:
            return
        if self.commands.adopted_id != self.commands.mission_id:
            # Not adopted (preparing, or a preparation that ended): what the core holds is the last mission's
            # (persistent console plan D2; Codex stage 2 design P1 and round 2 P1).
            return
        pid, did = mission.pallet_id, mission.destination_id
        tb.set_correction_version(self.slam_releases)
        scenario_ = self.options.scenario
        if slam is not None:
            scenario_ = slam.get("return_scenario", slam.get("transport_scenario", scenario_))
        site_ = scenario_.destination
        site_pose = TargetPose(float(site_.x_m), float(site_.y_m), float(site_.yaw_rad))
        tb.update_destination(did, site_pose)
        if ms.phase in ("recover_lower", "recover_withdraw", "clear_out"):
            return  # plan D4/D4b: the recovery puts the pallet back itself; no branch below applies
        if ms.phase in ("lift", "extract", "transport", "lower"):
            tb.carried(pid)
        elif ms.phase in ("withdraw", "settle", "return_home", "home_settle", "complete"):
            if tb.pallets[pid].state == "carried":
                tb.put_down(pid, site_pose, uncertainty_m=RECOGNISED_MARGIN_M)  # plan D10
            elif tb.pallets[pid].state == "delivered":
                tb.update_pose(pid, site_pose)
        elif pocket.get("planning_pickup") is not None:
            site_ = pocket["planning_pickup"]
            tb.update_pose(pid, TargetPose(float(site_.x_m), float(site_.y_m), float(site_.yaw_rad)),
                           basis="recognised", uncertainty_m=RECOGNISED_MARGIN_M)  # the observation placed it (D10)

    def safety_state(self, t: float | None) -> dict:
        """The mission's safety state for the arbiter (plan E8, M-a0.5 section 4), from what the core already holds
        this tick; its only port read is the SLAM mode (served from the record in a replay, which runs it too).
        ``basis`` says what a value rests on. ``t`` None: before the first control tick (no work time yet)."""
        ms, state, slam, obstacle, near, cmds = self.ms, self.state, self.slam, self.obstacle, self.near, self.commands
        phase_ = ms.phase
        applied_ = state.get("applied_command")
        hold_ = "none"
        if slam is not None and slam["tracker"].mode == "holding":
            hold_ = ("docking" if phase_ == "transport" else phase_ if phase_ in ("approach", "insert") else "held")
        out = {
            "t": t,
            "mission": {"id": cmds.mission_id, "status": cmds.status, "generation": cmds.mission_gen,
                        "pallet_id": None if cmds.mission is None else cmds.mission.pallet_id,
                        "destination_id": None if cmds.mission is None else cmds.mission.destination_id,
                        "phase": phase_,
                        "phase_elapsed_s": (cmds.work_elapsed(ms.phase_started, t)
                                            if phase_ is not None and t is not None else None),
                        # the follower's distance left in this phase (None without one): U7's cancel at a leg's end
                        "remaining_m": self._remaining_m(phase_),
                        "resume": dict(cmds.resume_check) if cmds.resume_check else None,
                        "stop_seen_s": cmds.stop_seen_s, "not_stopped": cmds.not_stopped},
            "load": {"loaded": phase_ in ("lift", "extract", "transport", "lower"), "basis": "phase",
                     "lift_target_m": float(ms.lift_command)},
            "fork_in_pocket": {"value": phase_ in ("insert", "lift", "extract", "lower", "withdraw"), "basis": "phase"},
            "near": None if near is None else {
                "tracker_built": near.get("tracker") is not None,
                # M-b B7: the near-field contract applies now (the condition the core runs near_tick under);
                # tracker_built stays true after a normal near-field end.
                "active": self.near_active(),
                "stop_reason": getattr(near.get("stop_state"), "reason", None),
                "terminal": bool(getattr(near.get("stop_state"), "terminal", False)),
                "protect": near.get("protect") is not None},
            "frame": {"correction_version": None if obstacle is None else obstacle.get("version"), "hold": hold_,
                      "slam": slam is not None},
            "command": {"applied_speed_mps": None if applied_ is None else float(applied_[0]),
                        "steering_rad": None if applied_ is None else [float(v) for v in applied_[1]]},
            "blocked": {"cap": None if obstacle is None or obstacle.get("permission_cap") is None
                        else float(obstacle["permission_cap"][1]),
                        "reason": None if self.last_permission is None else self.last_permission[1]},
        }
        if self.baseline is not None:
            # A live session (persistent console plan D5a): what the forks are in is the session's cargo state, kept
            # across a mission's end; a batch run's record stays as it was (the replay compares it).
            fields_ = self.cargo.safety_fields()
            out["load"] = {**fields_["load"], "lift_target_m": float(ms.lift_command)}
            out["fork_in_pocket"] = fields_["fork_in_pocket"]
        if cmds.adopted_id is None and self.baseline is not None:
            # A live mission ended (the batch run's mission is always the adopted one): the forks in a pocket refuse
            # a submit and teleop until the recovery; the load held refuses a submit until the cargo submit (6c).
            # The key only when it holds, so a batch run's safety record is as it was.
            refusal_ = self.cargo.submit_refusal()
            if refusal_ is not None:
                out[refusal_] = True
            report_ = self.recovery_report()
            if report_ is not None:
                out["recovery"] = report_  # plan D14: the recovery stage the teleop policy reads
        return out

    def release_gate(self, t: float, rear) -> SimpleNamespace:
        """At each control tick's start: a pending SLAM release waits for the truck to stand (10 s at most), then
        releases and re-reads the control pose; SLAM warming up holds the follower. Returns ``releasing``,
        ``warming`` and ``rear``."""
        slam, hooks = self.slam, self.hooks
        require, slam_release, slam_rear = hooks.require, self.slam_release, hooks.slam_rear
        if (self.commands.status == "recovering" and self.ms.phase in ("recover_lower", "recover_withdraw")
                and not self.cargo.trace_valid):
            # Plan D4: the evidence went after the retrace was installed -- ended at the tick's start, before the
            # follower and the command run (Codex 6b impl round 2 P1).
            self.recovery_failed(t, "trace_invalid")
        if self.end_release_due is not None and slam is not None:
            # A live mission's end release, waiting for the truck to stand (plan D1 step 4).
            if slam["tracker"].mode != "holding" or not self.cargo.release_allowed():
                # gone already, or the forks' state no longer allows it (a pallet check failed since the reservation:
                # unknown keeps the hold, plan D5a, Codex impl round 1 P2)
                self.end_release_due = None
            elif slam["stop_now"]:
                self.end_release(t, deferred=True)
                rear = slam_rear(t)
        releasing = slam is not None and slam.get("pending_release") is not None
        if releasing:
            waited = self.commands.work_elapsed(slam["release_wait_from"], t)
            require(waited < 10.0, f"slam_release_no_stop after {waited:.1f} s")
            if slam["stop_now"]:
                slam_release(t)
                releasing = False
                rear = slam_rear(t)  # the released estimate, this very tick
                hooks.set_rear(rear)  # the runner's closures read its rear (Codex slice-3 P2)
                slam["control"][-1][1:4] = rear.tolist()
        warming = slam is not None and not slam_may_drive(slam)
        return SimpleNamespace(releasing=releasing, warming=warming, rear=rear)

    def assemble_command(self, t: float, dt: float, requested_speed: float, curvature: float, kappa: float,
                         steering_command, estop_holding: bool, obstacle_hold: bool, dwell_hold: bool,
                         cusp_hold: bool, safety_hold: bool) -> SimpleNamespace:
        """The tick's command (boundary design section 1): a hold zeroes the wheels with the steering held, the
        Ackermann command, the creep while the steering catches up, the wheel-speed slew with the permission's
        own cap and the e-stop/safety step to zero, the steering rate limit (``steering_command`` is the
        runner's array, changed in place) and the applied command record. Returns ``drive``,
        ``steering_command``, ``actual_steering`` (the measured steering it compared) and the held
        ``requested_speed``."""
        state, obstacle, settings, hooks, options, near = (self.state, self.obstacle, self.settings, self.hooks,
                                                           self.options, self.near)
        drive_geometry, COMMAND_ACCEL_MPS2 = options.drive_geometry, options.command_accel_mps2
        if estop_holding or obstacle_hold or dwell_hold or cusp_hold:
            # Zero every wheel target at once, steering held (plan D4; D8b dwells; a cusp stall).
            requested_speed = 0.0
        # A dwell's braking keeps the wheels on the curvature the held steering holds,
        # not the tracker's, so the wheel speeds agree with the steering (Codex D8b
        # confirmation review P2) -- here and in the applied command below.
        wheel_curvature = kappa if (dwell_hold or cusp_hold) else curvature
        drive = ackermann_command(requested_speed, wheel_curvature, drive_geometry)
        target_steering = (
            steering_command.copy() if (estop_holding or obstacle_hold or dwell_hold or cusp_hold)
            else np.asarray(drive.steering_rad)
        )
        actual_steering = hooks.steering_position()
        # Creep while steering catches up; log the measured physical response.
        steering_error = float(np.max(np.abs(target_steering - actual_steering)))
        wheel_speed = creep(requested_speed, steering_error)
        if obstacle is not None and options.obstacle_act:
            # A step wheel target locked an odometry wheel (L3c v35/v36 seed 4:
            # the held pose lost 1.85 and 2.80 cm). The final wheel speed is
            # slewed -- braking at the stopping model's deceleration, pulling
            # away at COMMAND_ACCEL_MPS2 -- and the permission's own limit (a
            # sensor falling silent included) and the e-stop probe still cut
            # it as a step, the stop their distances were measured with
            # (Codex checkpoint 6 P1). The steering creep is slewed too: as a
            # step from 0.6 m/s it skidded a rear wheel and the held pose
            # fell 3 cm behind (L3c v43 seed 1, pocket_obstacle).
            # The deceleration is read only on a braking tick and the cap after the slew, as before the
            # extraction: both are recorded reads and a replay compares their order.
            wheel_speed = slew_cap_stop(state.get("command_speed", 0.0), wheel_speed,
                                        decel_mps2=lambda: obstacle["layer"].permission.config.stopping.decel_mps2,
                                        accel_mps2=COMMAND_ACCEL_MPS2, dt=dt, cap_mps=None)
            cap = obstacle.get("permission_cap")
            if cap is not None and cap[0] == t and abs(wheel_speed) > cap[1]:
                wheel_speed = math.copysign(cap[1], wheel_speed)
            if estop_holding or safety_hold:
                wheel_speed = 0.0
            state["command_speed"] = wheel_speed
        drive = ackermann_command(wheel_speed, wheel_curvature, drive_geometry)
        rate = settings["steering_command_rate_rad_s"] * dt
        steering_command += np.clip(target_steering - steering_command, -rate, rate)
        # Plan D8 S4a-1 ⓪: what the wheels and steering run this tick, for the recorder --
        # the speed after the Ackermann wheel-rate limit (Codex review P3).
        state["applied_command"] = (float(drive.speed_mps), [float(v) for v in steering_command])
        if near is not None and near["tracker"] is not None:
            near["commands"].add(t + dt, drive.speed_mps)  # the interval [t, t + dt]
        return SimpleNamespace(drive=drive, steering_command=steering_command, actual_steering=actual_steering,
                               requested_speed=requested_speed)

    # --- M-a1 slice 6a: closures moved from run(), statement for statement ---
    def cusp_excluded(self, phase_) -> str | None:
        """K2: where another rule owns a cusp stop -- D7 transport (docking accepted or re-approaching,
        whatever --d7-docking says), a held S4c-3 realignment, the legacy tracker profile."""
        options = self.options
        realign_active = self.realign_active
        slam = self.slam
        if options.tracker_profile == "20260921":
            return "tracker_profile"
        if phase_ == "transport" and slam is not None and (
            slam["docking"].get("accepted_any") or slam["docking"].get("reapproach")
        ):
            return "d7_transport"
        if phase_ == "observe" and slam is not None and slam["tracker"].mode == "holding" and realign_active():
            return "realign_held"
        return None

    def cusp_stopped(self) -> bool:
        """K1's stopped: standing with a zero command applied at the wheels on the last tick."""
        cusp_standing = self.hooks.cusp_standing
        state = self.state
        return cusp_standing() and state.get("applied_command", (None, None))[0] == 0.0

    def cusp_heading_end(self, tracking_now) -> dict:
        """The tracking result a hold record keeps (fix A)."""
        return {"status": tracking_now.status, "failure": tracking_now.failure,
                "position_error_m": tracking_now.position_error_m, "yaw_error_rad": tracking_now.yaw_error_rad}

    def cusp_heading_offset(self, tracker_, progress_m, pose) -> tuple[float, float]:
        """Signed lateral and heading offset of pose from tracker_'s path at progress_m."""
        path_ = tracker_._poses
        i_ = int(np.clip(np.searchsorted(tracker_._distance, progress_m, side="right") - 1, 0, len(path_) - 2))
        f_ = float(np.clip((progress_m - tracker_._distance[i_]) / tracker_._lengths[i_], 0.0, 1.0))
        point_ = path_[i_, :2] + f_ * (path_[i_ + 1, :2] - path_[i_, :2])
        y0_, y1_ = float(path_[i_, 2]), float(path_[i_ + 1, 2])
        yaw_ = y0_ + f_ * math.atan2(math.sin(y1_ - y0_), math.cos(y1_ - y0_))
        d_ = np.asarray(pose[:2], dtype=float) - point_
        return (float(math.cos(yaw_) * d_[1] - math.sin(yaw_) * d_[0]),
                math.atan2(math.sin(float(pose[2]) - yaw_), math.cos(float(pose[2]) - yaw_)))

    def cusp_alive(self, now_s) -> bool:
        """Every obstacle sensor within its timeout -- checked every tick: a silent one sends no scan."""
        obstacle = self.obstacle
        if obstacle is None:
            return True
        layer_ = obstacle["layer"]
        timeout_ = layer_.permission.config.sensor_timeout_s
        return all(now_s - last_ <= timeout_ for last_ in layer_.last_scan_s.values())

    def cusp_scan_id(self, now_s):
        """Which scan a standing decision belongs to: the obstacle layer's last stamp (0.1 s without one)."""
        obstacle = self.obstacle
        if obstacle is not None:
            return obstacle.get("last_stamp")
        return math.floor(now_s * 10.0 + 1e-9)

    def cusp_ownership(self, now_s, rear_now, command_mps) -> tuple[bool, dict | None]:
        """K2: whether the obstacle side leaves this stop alone, judged on the layer's latest snapshot
        now -- the current leg's path ahead and the e-stop arc at the held curvature in the leg's
        direction (the permission's own cap, max(|command|, |measured|)) unblocked and every sensor
        alive. Without a layer the planner obstacles alone decide, so the stall handling owns it."""
        held_kappa = self.hooks.held_kappa
        loaded = self.hooks.loaded()
        ms = self.ms
        obstacle = self.obstacle
        signed_speed = self.hooks.signed_speed()
        trackers = self.trackers
        if obstacle is None:
            return True, None
        layer_ = obstacle["layer"]
        if layer_.snapshot is None:
            return False, {"time_s": now_s, "why": "no_snapshot"}
        leg_, dir_ = trackers[ms.phase].leg_ahead()
        pose_ = tuple(float(v) for v in rear_now)
        kappa_ = held_kappa()
        path_ = layer_.check_candidate(leg_, current_pose=pose_, direction=dir_, loaded=loaded)
        arc_ = layer_.check_arc_candidate(
            current_pose=pose_, curvature_inv_m=kappa_, direction=dir_, loaded=loaded,
            cap_mps=max(abs(float(command_mps)), abs(float(signed_speed))),
        )
        timeout_ = layer_.permission.config.sensor_timeout_s
        silent_ = sorted(name_ for name_, last_ in layer_.last_scan_s.items() if now_s - last_ > timeout_)
        record_ = {
            "time_s": now_s, "path": path_.blocked, "path_verified_m": path_.verified_m, "arc": arc_.blocked,
            "arc_verified_m": arc_.verified_m, "kappa_inv_m": kappa_, "silent": silent_,
        }
        return path_.blocked is None and arc_.blocked is None and not silent_, record_

    def cusp_leg_checker(self):
        """K3: the leg's planner rectangles and bounds -- transport and return as their replans check
        them (leg_obstacles), observe as plan_observation_leg does (the pickup obstacle, the pickup and
        scene bounds)."""
        escape_leg = self.escape_leg
        geometry = self.geometry
        grid_planning = self.options.grid_planning
        grid_world = self.hooks.grid_world
        ms = self.ms
        obstacle = self.obstacle
        pickup_zone = self.options.pickup_zone
        scenario = self.options.scenario
        from forklift_core.planning.grid_collision import FootprintCollisionChecker as GridChecker
        from forklift_core.planning.pallet_mission import leg_obstacles

        if ms.phase in ("transport", "return_home"):
            return escape_leg(ms.phase)[2]
        pickup_ = None
        # A grid plan, and a --layout scene run on its prior map (options.pickup_prior; absent in older records),
        # never plan on the pickup pallet's true pose: the zone before recognition, the estimate after it.
        if grid_planning or getattr(self.options, "pickup_prior", False):
            known_ = obstacle.get("pickup_estimate")
            pickup_ = pickup_zone if known_ is None else Rectangle(
                known_.x_m, known_.y_m, geometry.pallet_depth_m, geometry.pallet_width_m, known_.yaw_rad
            )
        rects_, footprint_, bounds_ = leg_obstacles(
            "observe", grid_world(scenario), geometry, pickup_obstacle=pickup_, pickup_bounds=self.observe_bounds()
        )
        return GridChecker(rects_, footprint_, bounds_)

    def cusp_sensors(self, now_s, rear_now, trajectory, direction):
        """K3's sensor checks on the next leg's prefix: no unknown or edge cell within the stopping
        distance (at the leg's cruise, plus a step), no occupied cell anywhere in it (walking past
        unknown), the FREE evidence it relied on within the permission's age limit, sensors alive."""
        loaded = self.hooks.loaded()
        ms = self.ms
        obstacle = self.obstacle
        trackers = self.trackers
        layer_ = obstacle["layer"]
        pconf_ = layer_.permission.config
        pose_ = tuple(float(v) for v in rear_now)
        plain_ = layer_.check_candidate(trajectory, current_pose=pose_, direction=direction, loaded=loaded)
        passes_ = layer_.check_candidate(trajectory, current_pose=pose_, direction=direction, loaded=loaded,
                                         unknown_passes=True)
        stop_need_ = pconf_.stopping.distance_m(trackers[ms.phase].config.cruise_speed_mps) + pconf_.step_m
        unknown_near_ = plain_.blocked in ("unknown", "edge") and plain_.verified_m < stop_need_
        # Every FREE cell either walk relied on, past the first unknown too (Codex implementation review P1).
        age_ = now_s - min(plain_.oldest_free_s, passes_.oldest_free_s)
        silent_ = sorted(n_ for n_, last_ in layer_.last_scan_s.items() if now_s - last_ > pconf_.sensor_timeout_s)
        occupied_ = passes_.blocked == "occupied"
        ok_ = not unknown_near_ and not occupied_ and age_ <= pconf_.evidence_max_age_s and not silent_
        return ok_, occupied_, {
            "plain": plain_.blocked, "plain_verified_m": plain_.verified_m, "stop_need_m": stop_need_,
            "passes": passes_.blocked, "first_unknown_m": passes_.first_unknown_m,
            "evidence_age_s": age_ if math.isfinite(age_) else None, "silent": silent_,
        }

    def cusp_end_recovered(self, goal: bool) -> bool:
        """A short next leg's end missed (in heading, or a stall at the final goal -- cusp design
        amendment) that an existing rule recovers within its budget: a transport cusp by the cusp
        replan, a transport or return goal by the stall replan (SLAM only); a return cusp and the
        observe leg are refused."""
        max_cusp_replans = self.options.max_cusp_replans
        ms = self.ms
        slam = self.slam
        state = self.state
        if ms.phase == "transport" and not goal:
            return len(state["cusp_replans"]) < max_cusp_replans
        if ms.phase in ("transport", "return_home") and goal and slam is not None:
            return len(state["stall_replans"]) < 2
        return False

    def cusp_tick(self, now_s, rear_now, raw_, tracking_, retry_stand) -> tuple:
        """K2/K3 for this tick: (tracking, hold the steering); a hand-off forces the obstacle replan,
        an unrecoverable stall ends the run."""
        require = self.hooks.require  # a module-level function of the runner
        cusp_alive = self.cusp_alive
        cusp_end_recovered = self.cusp_end_recovered
        cusp_excluded = self.cusp_excluded
        cusp_leg_checker = self.cusp_leg_checker
        cusp_monitor = self.hooks.cusp_monitor()
        cusp_ownership = self.cusp_ownership
        cusp_scan_id = self.cusp_scan_id
        cusp_sensors = self.cusp_sensors
        cusp_standing = self.hooks.cusp_standing
        cusp_stopped = self.cusp_stopped
        dt = self.options.dt
        dump_tracking = self.hooks.dump_tracking
        grid_planning = self.options.grid_planning
        loaded = self.hooks.loaded()
        ms = self.ms
        obstacle = self.obstacle
        signed_speed = self.hooks.signed_speed()
        trackers = self.trackers
        from forklift_core.control import escape as esc

        tracker_ = trackers[ms.phase]
        pose_ = tuple(float(v) for v in rear_now)

        def judge_now(latch_):
            return CUSP_STALL.judge(
                tracker_, rear_now, raw_, latch_.cache, end_recovered=cusp_end_recovered,
                sweep_free=lambda trajectory_: esc.rectangles_sweep_free(cusp_leg_checker(), trajectory_),
                sensors=(lambda trajectory_, direction_: cusp_sensors(now_s, rear_now, trajectory_, direction_))
                if obstacle is not None else None,
            )

        def replan_pending():
            return obstacle is not None and (
                obstacle.get("force_replan") == ms.phase or bool(obstacle.get("replan_wait", {}).get(ms.phase))
            )

        def prefix_occupied(latch_):
            return obstacle["layer"].check_candidate(
                latch_.prefix_trajectory, current_pose=pose_, direction=latch_.hand_direction, loaded=loaded,
                unknown_passes=True,
            ).blocked == "occupied"

        result_ = cusp_monitor.tick(
            now_s, ms.phase, tracker_, raw_, tracking_, rear=rear_now, excluded=cusp_excluded(ms.phase),
            retry_stand=bool(retry_stand), standing=cusp_standing, stopped=cusp_stopped,
            alive=lambda: cusp_alive(now_s),
            ownership=lambda command_: cusp_ownership(now_s, rear_now, command_), judge_now=judge_now,
            scan_id=lambda: cusp_scan_id(now_s), replan_pending=replan_pending, prefix_occupied=prefix_occupied,
            release_update=lambda: tracker_.update(rear_now, signed_speed, dt),
            watch_since_s=(obstacle.get("progress", {}).get("since_s") if obstacle is not None else None),
        )
        if result_.force_replan and obstacle is not None:
            if grid_planning:
                obstacle["force_replan"] = ms.phase
            elif self.scene_replan_allowed():
                # Scene-mode replan plan 2.2: the hand-off goes to the device (no grid replan in a scene layout).
                why_ = self.scene_replan_raise(now_s, "cusp_handed", rear_now, detail={"latch": "handed"})
                require(why_ is None, why_ or "")
                return replace(result_.tracking, speed_mps=0.0), True
            elif cusp_monitor.latch is not None:
                # The obstacle replan exists only with grid planning (Codex implementation review P1).
                cusp_monitor.latch.record["events"].append({"time_s": now_s, "event": "force_skipped_no_grid"})
        if result_.end is not None:
            if result_.end == "cusp_position_stall" and self.scene_replan_allowed():
                # Scene-mode replan plan 2.2 (the stall recovery design v7 revised): a stall refused on its pose bounds
                # after its stop was confirmed is planned again from where the truck stands, not ended.
                rec_ = cusp_monitor.latch.record if cusp_monitor.latch is not None else {}
                why_ = self.scene_replan_raise(now_s, "cusp_pose", rear_now, deviation={
                    "position_m": rec_.get("position_error_m"), "yaw_rad": rec_.get("yaw_error_rad"),
                    "far": bool((rec_.get("judged") or {}).get("far")) if isinstance(rec_.get("judged"), dict) else None})
                require(why_ is None, why_ or "")
                return replace(result_.tracking, speed_mps=0.0), True
            dump_tracking(result_.end, result_.tracking)
            require(False, result_.end)
        return result_.tracking, result_.hold

    # --- M-a1 slice 6b: closures moved from run(), statement for statement ---
    def set_withdraw(self, poses) -> None:
        paths = self.paths
        path_record = self.hooks.path_record  # a runner module-level name
        state = self.state
        trackers = self.trackers
        paths["withdraw"] = replace(paths["withdraw"], poses=np.asarray(poses, dtype=float))
        state["paths"]["withdraw"] = path_record(paths["withdraw"])
        trackers["withdraw"] = RearAxlePathTracker(
            paths["withdraw"].poses,
            paths["withdraw"].directions,
            paths["withdraw"].curvatures_inv_m,
            trackers["withdraw"].config,
        )

    def move_withdraw(self, frame_change) -> None:
        """The withdraw follows the docked goal: move it by the goal's change."""
        paths = self.paths
        set_withdraw = self.set_withdraw
        if "withdraw" in paths:
            set_withdraw(np.array([compose(tuple(frame_change), tuple(pose)) for pose in paths["withdraw"].poses]))

    def docking_stop_config(self):
        """A stop to look, judged like arm_docking's (3 cm, 0.05 rad)."""
        slam = self.slam
        trackers = self.trackers
        base_ = slam.get("transport_config", trackers["transport"].config)
        return replace(base_, position_tolerance_m=0.03, yaw_tolerance_rad=0.05)

    def unaligned_delivery_check(self) -> None:
        """The delivery straight about to be driven with no accepted docking (docs/plans/2026-10-22-slot-anchored-
        neighbours.md 2.3): the runner's hook re-checks it as it was checked before slot anchors -- None, or the
        anchored pallets whose estimates block it, and the mission ends before the straight. No hook: nothing."""
        refusal = getattr(self.hooks, "unaligned_delivery_refusal", None)
        if refusal is None:
            return
        anchored = refusal()
        self.hooks.require(anchored is None, f"session_anchor_unaligned:{anchored}")

    def arm_docking(self) -> None:
        """Drive transport only to where its delivery straight begins (v3.8)."""
        final_straight_prefix = self.hooks.final_straight_prefix
        geometry = self.geometry
        paths = self.paths
        slam = self.slam
        trackers = self.trackers
        docking = slam["docking"]
        if docking["status"] not in ("pending", "armed"):
            return
        full = paths["transport"]
        # The current round's stop: 1.5 m, then 0.4 m (Codex 5de6c3a P2).
        prefix = final_straight_prefix(full, docking.get("keep_m", geometry.delivery_straight_m))
        if prefix is None:
            docking["status"] = "no_final_straight"
            self.unaligned_delivery_check()  # the straight is driven without a docking (Codex CT-1 PR 4 plan r4 P2)
            return
        docking.update(
            status="armed",
            delivery_rear=[float(v) for v in full.poses[-1]],
            predelivery_rear=[float(v) for v in prefix.poses[-1]],
        )
        # A stop to look, not a final goal: judged like an observe stop (3 cm,
        # 0.05 rad); the docked straight that follows keeps the 8 mm goal.
        slam.setdefault("transport_config", trackers["transport"].config)
        trackers["transport"] = RearAxlePathTracker(
            prefix.poses,
            prefix.directions,
            prefix.curvatures_inv_m,
            replace(
                slam["transport_config"],
                position_tolerance_m=0.03,
                yaw_tolerance_rad=0.05,
            ),
        )

    def reapproach_plan(self, start_rear, line, config=None):
        """Plan D7c ③: the re-approach to the docking line's start, accepted only
        inside its box and length, and when the transport tracker's dry run stops
        at the line start within the stop's tolerance."""
        time = self.hooks.time_module()  # the runner's time (the recorded Clock proxy when recording)
        bicycle_rollout = self.hooks.bicycle_rollout
        docking_stop_config = self.docking_stop_config
        geometry = self.geometry
        grid_kwargs = self.hooks.grid_kwargs
        grid_world = self.hooks.grid_world
        plan_docking_reapproach = self.hooks.plan_docking_reapproach
        planner_config = self.options.planner_config
        scenario = self.options.scenario
        slam = self.slam
        planned_at = time.monotonic()
        # The grid around the start pose (after a release: the released one, Codex
        # D7c impl P1-2), and an obstacle replan's shared budget when one runs (P1-1).
        grid_ = grid_kwargs(None, own=tuple(float(v) for v in start_rear))
        deadline_ = grid_.get("deadline")
        path, record = plan_docking_reapproach(
            grid_world(slam.get("transport_scenario", scenario)),
            PlanningPose(*(float(v) for v in start_rear)),
            PlanningPose(*(float(v) for v in line)),
            config if config is not None else planner_config,
            geometry=geometry,
            keep_m=geometry.delivery_straight_m,
            deadline=deadline_ if deadline_ is not None else SearchBudget(20.0),
            **({"occupancy": grid_["occupancy"]} if "occupancy" in grid_ else {}),
        )
        record["planning_wall_s"] = time.monotonic() - planned_at
        if path is not None:
            dry = bicycle_rollout(
                path.poses, path.directions, path.curvatures_inv_m, docking_stop_config(), np.asarray(start_rear)
            )
            record["dry_run"] = {k: v for k, v in asdict(dry).items() if k != "trajectory"}
            if not (dry.status == "arrived" and dry.position_error_m <= 0.03 and abs(dry.yaw_error_rad) <= 0.05):
                record["refused"] = "dry_run"
                path = None
        return path, record

    def install_reapproach(self, t: float, path, goal) -> None:
        """Plan D7c ④: the re-approach replaces the transport path; the docking waits
        at its end (no arm_docking: a re-approach may end on a reverse curve)."""
        path_record = self.hooks.path_record  # a runner module-level name
        add_path_display = self.hooks.add_path_display
        docking_stop_config = self.docking_stop_config
        geometry = self.geometry
        ms = self.ms
        obstacle = self.obstacle
        paths = self.paths
        slam = self.slam
        stage = self.hooks.stage()
        state = self.state
        trackers = self.trackers
        state.setdefault("paths_replaced", []).append(
            {"leg": "transport", "time_s": t, "by": "reapproach", "path": state["paths"].get("transport")})
        paths["transport"] = path
        state["paths"]["transport"] = path_record(path)
        self.hooks.write_paths()
        add_path_display(stage, path, "Transport", (1.0, 0.65, 0.04))
        trackers["transport"] = RearAxlePathTracker(
            path.poses, path.directions, path.curvatures_inv_m, docking_stop_config()
        )
        slam["docking"].update(
            status="armed", round=1, keep_m=geometry.delivery_straight_m, reapproach=True,
            await_retry_match=True, retry_seed_goal=[float(v) for v in goal],
        )
        if obstacle is not None:
            if obstacle.get("backoff", {}).get("phase") == "transport":
                obstacle.pop("backoff")
            obstacle.setdefault("replan_wait", {}).pop("transport", None)
            obstacle.setdefault("progress", {}).clear()
        ms.phase_started = t

    def release_and_carry(self, t: float) -> dict:
        """Plan D7c: release a held correction; a goal still in the held frame moves
        with everything held by T = after o before^-1, once (docking_retry.py)."""
        DOCKING_RETRY = self.options.DOCKING_RETRY  # a runner module-level name
        loaded = self.hooks.loaded()
        ms = self.ms
        obstacle = self.obstacle
        paths = self.paths
        scenario = self.options.scenario
        set_withdraw = self.set_withdraw
        slam = self.slam
        slam_odom_base = self.hooks.slam_odom_base
        slam_rear = self.hooks.slam_rear
        docking = slam["docking"]
        if slam["tracker"].mode != "holding":
            return {"released": False}
        before = slam_rear(t)
        jump_m, jump_rad = self.release_slam(slam_odom_base())
        after = slam_rear(t)
        site = slam.get("transport_scenario", scenario).destination
        carry = DOCKING_RETRY.carry_on_release(
            docking,
            tuple(float(v) for v in before),
            tuple(float(v) for v in after),
            withdraw_poses=paths["withdraw"].poses if "withdraw" in paths else None,
            destination=(site.x_m, site.y_m, site.yaw_rad),
        )
        if carry["moved"]:
            if "withdraw" in paths:
                set_withdraw(carry["withdraw_poses"])
            sc_ = slam.get("transport_scenario", scenario)
            x_, y_, yaw_ = carry["destination"]
            slam["transport_scenario"] = replace(
                sc_, destination=replace(sc_.destination, x_m=x_, y_m=y_, yaw_rad=yaw_)
            )
        event = {"phase": ms.phase, "time_s": t, "event": "release_docking_retry", "jump_m": jump_m,
                 "jump_rad": jump_rad, "frame_change": carry["frame_change"], "goal_moved": carry["moved"]}
        slam["holds"].append(event)
        if obstacle is not None and slam["tracker"].applied is not None:
            # The grid is re-projected with the released correction before any
            # plan reads it, as for a stall replan's release (Codex checkpoint P1).
            obstacle["applied"] = tuple(float(v) for v in slam["tracker"].applied[0])
            obstacle["version"] += 1
            obstacle["reprojections"] = obstacle.get("reprojections", 0) + 1
            obstacle["layer"].refresh(
                t, obstacle["applied"], obstacle["version"],
                current_pose=tuple(float(v) for v in after),
                path_ahead=np.array([after]), loaded=loaded,
            )
        return {"released": True, **{k: event[k] for k in ("jump_m", "jump_rad", "frame_change", "goal_moved")},
                "rear": [float(v) for v in after]}

    def start_docking_retry(self, t: float, trigger: str, failure: str) -> None:
        """Plan D7c (delivery): release, plan back to the docking line's start, and
        match again on arrival. At most two per delivery; the third ends the run with
        the failure the retry replaced."""
        DOCKING_RETRY = self.options.DOCKING_RETRY  # a runner module-level name
        require = self.hooks.require  # a runner module-level name
        geometry = self.geometry
        install_reapproach = self.install_reapproach
        reapproach_plan = self.reapproach_plan
        release_and_carry = self.release_and_carry
        slam = self.slam
        slam_rear = self.hooks.slam_rear
        state = self.state
        retries = state.setdefault("docking_retries", [])
        require(sum(1 for r in retries if r.get("counted")) < 2, failure)
        event = {"trigger": trigger, "time_s": t, "counted": True, "replaces": failure}
        retries.append(event)
        if "previous_goal" not in slam["docking"]:
            # No match committed a goal (a refused first match): the prior, a map pose
            # that a release does not move (Codex D7c impl P2-4).
            slam["docking"]["previous_goal"] = list(slam["docking"]["delivery_rear_prior"])
            slam["docking"]["goal_frame"] = "map"
        event["release"] = release_and_carry(t)
        goal = tuple(slam["docking"]["previous_goal"])
        line = DOCKING_RETRY.line_start(goal, geometry.delivery_straight_m)
        start_now = slam_rear(t)
        path, record = reapproach_plan(start_now, line)
        event.update(goal=list(goal), goal_frame=slam["docking"].get("goal_frame"), line_start=list(line),
                     start=[float(v) for v in start_now], plan=record)
        require(path is not None, f"docking_retry_no_plan:{record.get('refused')}:{record.get('status')}")
        install_reapproach(t, path, goal)

    def transport_replan(self, start, config, travel):
        """The transport leg -- or, during a D7c re-approach, the re-approach to the
        docking line's start (an obstacle replan must not undo it: Codex D7c P2)."""
        DOCKING_RETRY = self.options.DOCKING_RETRY  # a runner module-level name
        geometry = self.geometry
        grid_kwargs = self.hooks.grid_kwargs
        grid_world = self.hooks.grid_world
        initial_time = self.options.initial_time
        plan_transport_leg = self.hooks.plan_transport_leg
        reapproach_plan = self.reapproach_plan
        scenario = self.options.scenario
        slam = self.slam
        state = self.state
        world = self.hooks.world()
        if slam is not None and slam["docking"].get("reapproach"):
            line = DOCKING_RETRY.line_start(slam["docking"]["previous_goal"], geometry.delivery_straight_m)
            path, record = reapproach_plan((start.x_m, start.y_m, start.yaw_rad), line, config)
            state.setdefault("docking_retries", []).append(
                {"trigger": "obstacle_replan", "time_s": world.current_time - initial_time, "counted": False,
                 "plan": record}
            )
            if path is not None:
                return path
            # A search failure keeps its own status (invalid_start / no_path drive the
            # zero-clearance retry -- Codex D7c impl P2-6); an operating refusal is named.
            status_ = record.get("status") if record.get("refused") == "no_path" else f"reapproach_{record.get('refused')}"
            return PlanResult(False, status_, np.empty((0, 3)), np.empty(0, dtype=np.int8), np.empty(0), 0.0,
                              int(record.get("expansions", 0)))
        return plan_transport_leg(
            grid_world(slam.get("transport_scenario", scenario) if slam is not None else scenario),
            start, config, geometry=geometry, travel_config=travel, **grid_kwargs(None),
        )

    PRECISION_CUMULATIVE_M = 0.20

    @classmethod
    def _precision_cumulative(cls, result, delivery):
        """D2's cumulative bound (plan 2026-10-22 open-zone docking): a weak-direction acceptance must not move the
        delivery goal more than 0.20 m from the first SLAM goal over all rounds -- a D7c re-approach seeds from the
        latest goal, so each round's own bound does not limit the sum. Measured as the goal positions' distance (a
        frame change of the whole path would mix in the yaw about the origin, Codex v4 P2)."""
        if not getattr(result, "accepted", False) or getattr(result, "reason", None) != "ok:precision":
            return result
        moved = math.hypot(result.goal_estimate[0] - delivery[0], result.goal_estimate[1] - delivery[1])
        record = dict(result.precision or {}, cumulative_m=moved)
        if moved <= cls.PRECISION_CUMULATIVE_M:
            return replace(result, precision=record)
        return replace(result, accepted=False, reason="degenerate:cumulative", goal_estimate=None, precision=record)

    def dock_at_delivery_straight(self, t: float, answer=None) -> None:
        """Match the live scan to the delivery reference and re-place the goal.

        Real-time plan D9b: in a real-time run (``self.jobs``) the first call stops at the match and asks the
        worker for it; the truck stands; once the answer passes the D8 checks this is called again with
        ``answer`` = (the asking call's inputs, the match result, its wall time) and goes on from the match with
        those inputs -- the estimate, the scan and the truth it was matched on -- and the pose, grid and obstacles
        of now. Nothing of the docking's state changes before the answer, so a dropped match leaves it armed.

        Codex v3.8: goal_est = A o R^-1 with R = X o Q o X^-1; the whole rest
        of the drop (delivery straight, withdraw, any transport replan) is
        moved by the same correction E = goal_est o D^-1; an accepted match
        whose straight cannot be redrawn is re-aligned by a collision-checked
        plan, not dropped; a refused match falls back to the SLAM goal.
        """
        DOCKING_RETRY = self.options.DOCKING_RETRY  # a runner module-level name
        ROUND2_KEEP_M = self.options.ROUND2_KEEP_M  # a runner module-level name
        path_record = self.hooks.path_record  # a runner module-level name
        require = self.hooks.require  # a runner module-level name
        time = self.hooks.time_module()  # the runner's time (the recorded Clock proxy when recording)
        options = self.options
        add_path_display = self.hooks.add_path_display
        beam_angles = self.hooks.beam_angles()  # bound only under SLAM in run()
        bicycle_rollout = self.hooks.bicycle_rollout
        dock = self.hooks.dock
        docking_stop_config = self.docking_stop_config
        final_straight_prefix = self.hooks.final_straight_prefix
        geometry = self.geometry
        grid_kwargs = self.hooks.grid_kwargs
        grid_planning = self.options.grid_planning
        laser_mount = self.hooks.laser_mount()  # bound only under SLAM in run()
        move_withdraw = self.move_withdraw
        ms = self.ms
        obstacles = self.hooks.truth_obstacles()
        paths = self.paths
        rear = self.hooks.rear()  # the runner's, read when called
        scenario = self.options.scenario
        slam = self.slam
        stage = self.hooks.stage()
        start_docking_retry = self.start_docking_retry
        state = self.state
        straight_from_pose = self.hooks.straight_from_pose
        trackers = self.trackers
        truth_rear = self.hooks.truth_rear()  # the runner's, read when called
        docking = slam["docking"]
        state["delivery_docking"] = docking  # linked first: kept if a step fails
        delivery = tuple(docking["delivery_rear_prior"])
        path_end = docking["delivery_rear"]
        docking["path_end_vs_prior_m"] = float(math.hypot(path_end[0] - delivery[0], path_end[1] - delivery[1]))
        # A: the estimate at the instant of the scan being matched (Codex P3).
        # Re-expressed with the correction applied now, so a release between
        # the scan and this match cannot mix frames (Codex 5de6c3a P2).
        bx, by, byaw = compose(slam["tracker"].applied[0], slam["last_scan_odom_base"])
        estimate = (
            bx - abs(options.rear_axle_offset_m) * math.cos(byaw),
            by - abs(options.rear_axle_offset_m) * math.sin(byaw),
            byaw,
        )
        offset = abs(options.rear_axle_offset_m)
        rear_from_laser = (
            offset + float(laser_mount.xyz_m[0]),
            float(laser_mount.xyz_m[1]),
            float(laser_mount.yaw_rad),
        )
        live = laser_points(slam["last_sent_ranges"], beam_angles)
        # The station the reference scan was taken at (the prior differs only with
        # --destination-prior-error-m). After a D7c retry the first match is seeded
        # from the latest goal, not the prior (Codex D7c re-review P2-3).
        reference = tuple(docking.get("delivery_rear_reference", delivery))
        seeded = docking.get("retry_seed_goal") is not None
        seed = tuple(docking["retry_seed_goal"]) if seeded else delivery
        # Plan 2026-10-22 open-zone docking D2 (sim only, off by default): the weak-direction acceptance, passed only
        # when it is on so the recorded calls of every other run keep their shape.
        acceptance_ = getattr(options, "docking_acceptance", "information")
        acceptance_kw = {} if acceptance_ == "information" else {"acceptance": acceptance_}
        if answer is None and self.jobs is not None:
            self._request("dock", "dock", {"args": (slam["reference_points"], live), "kwargs": dict(
                rear_from_laser=rear_from_laser, estimate_rear=estimate, delivery_rear=seed, **acceptance_kw)},
                t, rear, self._applied_now(), ctx={"estimate": estimate, "live": live, "rear_from_laser": rear_from_laser,
                                                   "reference": reference, "seed": seed, "seeded": seeded,
                                                   "truth_rear": tuple(float(v) for v in truth_rear)})
            return
        if answer is not None:
            asked, result, answer_wall_s = answer
            estimate, live, rear_from_laser = asked["estimate"], asked["live"], asked["rear_from_laser"]
            reference, seed, seeded, truth_rear = asked["reference"], asked["seed"], asked["seeded"], asked["truth_rear"]
        else:
            match_start = time.monotonic()
            result = dock(
                slam["reference_points"],
                live,
                rear_from_laser=rear_from_laser,
                estimate_rear=estimate,
                delivery_rear=seed,
                **acceptance_kw,
            )
        result = self._precision_cumulative(result, delivery)
        # Evaluation only: where the delivery pose really is in the estimate frame.
        expected = compose(estimate, compose(invert(tuple(truth_rear)), reference))
        docking.update(
            match_wall_s=time.monotonic() - match_start if answer is None else answer_wall_s,
            live_beams=int(len(live)),
            accepted=result.accepted,
            reason=result.reason,
            correction=result.correction,
            starts=[[ok, why, list(q)] for ok, why, q in result.starts],
            start_details=[dict(d) for d in result.start_details],  # iterations, cycle, gates (2026-10-13)
            goal_error_vs_truth=None
            if result.goal_estimate is None
            else [
                float(result.goal_estimate[0] - expected[0]),
                float(result.goal_estimate[1] - expected[1]),
                float(math.atan2(math.sin(result.goal_estimate[2] - expected[2]), math.cos(result.goal_estimate[2] - expected[2]))),
            ],
            slam_goal_error_vs_truth=[
                float(delivery[0] - expected[0]),
                float(delivery[1] - expected[1]),
            ],
        )
        # The match inputs, so a refused match can be replayed offline (2026-10-13:
        # every not_converged refusal so far had all starts within 0.11 mm). Record
        # only: a failed write is noted and the match goes on (Codex P2).
        try:
            self.hooks.save_npz(
                f"delivery_match_{len(docking.get('rounds', [])) + 1}.npz",
                reference=slam["reference_points"], live=live, rear_from_laser=rear_from_laser,
                estimate=estimate, seed=seed, reference_rear=reference, truth_rear=tuple(truth_rear),
            )
        except OSError as exc:
            docking.setdefault("match_input_errors", []).append(f"{type(exc).__name__}: {exc}")
        if options.d7_docking:
            docking["match_seed"] = "latest_goal" if seeded else "prior"
        retry_match = docking.pop("await_retry_match", False)
        round_number = docking.get("round", 1)
        keep_m = docking.get("keep_m", geometry.delivery_straight_m)
        previous_goal = tuple(docking.get("previous_goal", delivery))
        # A refused match keeps the best goal so far (the SLAM goal in round 1).
        goal = result.goal_estimate if result.accepted else previous_goal
        shift = compose(goal, invert(delivery))  # E: planned -> corrected (total)
        step_shift = compose(goal, invert(previous_goal))  # since the last round
        docking.setdefault("rounds", []).append(
            {
                "round": round_number,
                "keep_m": keep_m,
                "accepted": result.accepted,
                "reason": result.reason,
                "goal_error_vs_truth": docking["goal_error_vs_truth"],
                "estimate_vs_truth_at_match": [
                    float(estimate[0] - truth_rear[0]),
                    float(estimate[1] - truth_rear[1]),
                ],
                # Each round's own starts: the top-level ones are the last match's.
                "starts": docking["starts"],
                "start_details": docking["start_details"],
                # the D2 checks of this round (only with --docking-acceptance precision)
                **({"precision": result.precision} if getattr(result, "precision", None) is not None else {}),
            }
        )
        if options.d7_docking:
            # Plan D7c records: when, how long, and the errors in the docking line's
            # frame (along, lateral, yaw) rather than world dx/dy.
            entry_ = docking["rounds"][-1]
            entry_.update(time_s=t, match_wall_s=docking["match_wall_s"], retry_match=retry_match,
                          estimate_error_truck_frame=list(compose(invert(tuple(truth_rear)), estimate)))
            if result.goal_estimate is not None:
                entry_["goal_error_line_frame"] = list(compose(invert(expected), tuple(result.goal_estimate)))
        if retry_match:
            # Plan D7c: the first match after a retry must be accepted -- no
            # falling back to the previous goal (Codex D7c review P1-4).
            docking.pop("retry_seed_goal", None)
            require(result.accepted, f"docking_retry_match_refused:{result.reason}")
        if result.accepted:
            docking["accepted_any"] = True
        # A refused second round keeps the first round's valid correction
        # (Codex 5de6c3a P2): still "done", and its world map W is reused.
        docking["status"] = "done" if docking.get("accepted_any") else "fallback"
        if not docking.get("accepted_any"):
            self.unaligned_delivery_check()
        committed = False
        if options.d7_docking and result.accepted:
            # Plan D7c: commit the matched goal before the straight check, so a
            # refused straight retries from it (Codex D7c review P1-3). The goal is
            # in the held frame the match ran in.
            docking["to_world"] = list(compose(compose(reference, result.relative_rear), invert(estimate)))
            move_withdraw(step_shift)
            docking["previous_goal"] = list(goal)
            docking["goal_frame"] = "held"
            committed = True
        line_start = compose(goal, (-keep_m, 0.0, 0.0))
        # Delivery corrections of ~10 cm are the point of docking (seed 0: 92 mm),
        # so the box is wider than the insertion one; what makes it safe is the
        # dry run arriving AND its swept loaded footprint staying clear.
        # D9b: an asynchronous match is placed with the estimate it was matched on, but the straight starts where
        # the truck stands when it is taken (Codex D9 impl P2).
        straight, offsets = straight_from_pose(
            PlanningPose(*(estimate if answer is None else tuple(float(v) for v in rear))),
            PlanningPose(*line_start),
            PlanningPose(*goal),
            max_lateral_m=0.12,
            max_yaw_rad=0.08,
            min_length_m=0.3,
        )
        docking["offsets_to_new_line"] = offsets
        path = None
        if straight is not None:
            config = slam.get("transport_config", trackers["transport"].config)
            dry = bicycle_rollout(
                straight.poses, straight.directions, straight.curvatures_inv_m, config, rear
            )
            # The obstacles are in world; the dry run is in the estimate frame.
            # After an accepted match the truck's world pose is D o R, so
            # W = (D o R) o A^-1 carries estimate poses into world (Codex v3.8
            # impl P1). Without a match the estimate is the best world guess.
            if result.accepted:
                to_world = compose(compose(reference, result.relative_rear), invert(estimate))
                docking["to_world"] = list(to_world)  # the held frame keeps it valid
            elif docking.get("to_world") is not None:
                to_world = tuple(docking["to_world"])
            else:
                to_world = (0.0, 0.0, 0.0)
            if grid_planning:
                # The LiDAR grid, not the ground truth (plan audit table): the
                # dry run is in the estimate frame, which the grid shares.
                from forklift_core.planning.grid_collision import GridFootprintChecker

                swept_checker = GridFootprintChecker(
                    grid_kwargs("docking")["occupancy"], geometry.loaded_footprint, scenario.bounds
                )

                def pose_clear(pose):
                    return swept_checker.free(tuple(pose))
            else:

                def pose_clear(pose):
                    return collision_free_pose(
                        np.asarray(compose(to_world, tuple(pose))),
                        obstacles,
                        geometry.loaded_footprint,
                        scenario.bounds,
                    )
            swept_clear = all(pose_clear(pose) for pose in dry.trajectory)
            docking["dry_run"] = {
                **{k: v for k, v in asdict(dry).items() if k != "trajectory"},
                "trajectory_samples": len(dry.trajectory),
                "swept_clear": swept_clear,
            }
            # The tracker stops anywhere inside its own tolerance (Codex v3.8 P2:
            # a clean 0.7 m straight stops at 7.8 mm); require yaw margin only.
            if (
                dry.status == "arrived"
                and dry.position_error_m <= config.position_tolerance_m
                and abs(dry.yaw_error_rad) <= 0.75 * config.yaw_tolerance_rad
                and swept_clear
            ):
                path = straight
                if options.d7_docking and round_number == 1 and result.accepted:
                    # Plan D7c: what is driven after an accepted first round is the
                    # straight cut at the round-2 stop, judged at the stop's tolerance
                    # -- dry-run that, not only the whole straight (Codex D7 Isaac review
                    # P1: seed 1, whole straight 0.0145 rad, the 0.90 m stop path 0.086).
                    stop_ok, docking["prefix_dry_run"] = DOCKING_RETRY.round_two_stop_check(
                        straight, rear, docking_stop_config(), ROUND2_KEEP_M, clear=pose_clear
                    )
                    if not stop_ok:
                        path = None
        corrected = replace(
            scenario,
            destination=replace(
                scenario.destination,
                **dict(
                    zip(
                        ("x_m", "y_m", "yaw_rad"),
                        compose(shift, (scenario.destination.x_m, scenario.destination.y_m, scenario.destination.yaw_rad)),
                    )
                ),
            ),
        )
        slam["transport_scenario"] = corrected
        # No long held detour: a Hybrid A* re-alignment for a 9 cm correction
        # drove 13.3 m forward-only on held odometry and drifted 0.5 m (S2 v3.8
        # seed 0). A straight that cannot be accepted ends the run instead.
        if path is None and options.d7_docking:
            # Plan D7c (b): back to the line's start and match again.
            start_docking_retry(t, "b", f"docking_unaligned:{offsets}:{docking.get('dry_run')}")
            return
        require(path is not None, f"docking_unaligned:{offsets}:{docking.get('dry_run')}")
        docking.pop("reapproach", None)  # plan D7c: a straight ends the re-approach
        # The path being replaced, kept (paths.json holds only the latest): the transport search
        # path's cusps are judged from it (fork spacing plan, V5 cusp arrivals).
        state.setdefault("paths_replaced", []).append(
            {"leg": "transport", "time_s": t, "by": "delivery_straight", "path": state["paths"].get("transport")})
        paths["transport"] = path
        state["paths"]["transport"] = path_record(path)
        trackers["transport"] = RearAxlePathTracker(
            path.poses, path.directions, path.curvatures_inv_m, slam.get("transport_config", trackers["transport"].config)
        )
        if not committed:
            move_withdraw(step_shift)
        self.hooks.write_paths()
        add_path_display(stage, path, "Transport", (1.0, 0.65, 0.04))
        ms.phase_started = t
        docking["previous_goal"] = list(goal)
        if round_number == 1 and result.accepted:
            # Second round: loaded wheels slip (~5 % over the 1.5 m straight,
            # S2 v3.8c seed 3: 8 cm short), so stop again 0.6 m out and match
            # once more; the last 0.6 m then carries only that slip (~3 cm).
            # 0.4 m left the tracker too little straight to settle its yaw: a
            # 1.1 cm / 6 mrad start failed the 0.015 rad dry-run margin (L3c v15
            # seed 3: 0.016); 0.6 m settles it to 0.0045 (offline dry run, Codex).
            final_keep = ROUND2_KEEP_M
            prefix = final_straight_prefix(path, final_keep)
            if prefix is not None:
                docking.update(status="armed", round=2, keep_m=final_keep)
                trackers["transport"] = RearAxlePathTracker(
                    prefix.poses,
                    prefix.directions,
                    prefix.curvatures_inv_m,
                    replace(slam["transport_config"], position_tolerance_m=0.03, yaw_tolerance_rad=0.05),
                )

    # --- M-a1 slice 6c: closures moved from run(), statement for statement ---
    def release_and_stand(self, now_s: float, hold_event: str) -> dict:
        """Plan D8 S4c-2 / S4c-3 (shared): release the SLAM hold and re-project the grid,
        carry the pickup estimate and planning target into the released frame, and install
        the two-pose stand in observe -- its follower result, end hold and recoveries are
        bypassed until the re-observation (near["retry_stand"]). Returns the release record."""
        near_tracking = self.options.near_tracking  # a runner module-level name
        near, paths, trackers = self.near, self.paths, self.trackers
        from forklift_core.planning.hybrid_astar import PlanResult as _PlanResult

        released_ = self._release_reproject(now_s, hold_event)
        jump_m, jump_rad, frame_change, before_, after_ = (released_["jump_m"], released_["jump_rad"],
                                                           released_["frame_change"], released_["before"],
                                                           released_["after"])
        applied_before_, reprojected_ = released_["applied_before"], released_["reprojected"]
        heading_ = np.array([math.cos(float(after_[2])), math.sin(float(after_[2]))])
        stand_ = np.array([[*(np.asarray(after_[:2], dtype=float) - near_tracking.RETRY_STAND_M * heading_),
                            float(after_[2])], [float(after_[0]), float(after_[1]), float(after_[2])]])
        paths["observe"] = _PlanResult(True, "success", stand_, np.ones(2, dtype=np.int8), np.zeros(2),
                                       near_tracking.RETRY_STAND_M, 0)
        trackers["observe"] = RearAxlePathTracker(stand_, np.ones(2), np.zeros(2), trackers["observe"].config)
        near["retry_stand"] = True
        return {"jump_m": jump_m, "jump_rad": jump_rad, "frame_change": list(frame_change),
                "rear_before": [float(v) for v in before_], "rear_after": [float(v) for v in after_],
                "applied_before": applied_before_, "reprojected": reprojected_}

    def _reproject_after_release(self, now_s: float, after_, loaded) -> dict | None:
        """The grid re-projected with the tracker's released correction (None without a grid or a correction)."""
        obstacle, slam = self.obstacle, self.slam
        if obstacle is None or slam["tracker"].applied is None:
            return None
        obstacle["applied"] = tuple(float(v) for v in slam["tracker"].applied[0])
        obstacle["version"] += 1
        obstacle["reprojections"] = obstacle.get("reprojections", 0) + 1
        obstacle["layer"].refresh(now_s, obstacle["applied"], obstacle["version"],
                                  current_pose=tuple(float(v) for v in after_), path_ahead=np.array([after_]),
                                  loaded=loaded)
        return {"version": obstacle["version"], "applied": list(obstacle["applied"]),
                "reprojections": obstacle["reprojections"]}

    def _release_reproject(self, now_s: float, hold_event: str, carry: bool = True) -> dict:
        """Release the SLAM hold, re-project the grid with the released correction before any plan reads it, and
        carry the pickup estimate and the planning target into the released frame (release_and_stand's first
        half; the bottom-deck side legs release the same way, plan 2026-10-19 task 4). Returns what it did."""
        loaded = self.hooks.loaded()
        ms, obstacle, pocket, slam = self.ms, self.obstacle, self.pocket, self.slam
        slam_odom_base, slam_rear = self.hooks.slam_odom_base, self.hooks.slam_rear
        from forklift_core.localization.slam_pose import compose as pose_compose
        from forklift_core.localization.slam_pose import invert as pose_invert

        before_ = slam_rear(now_s)
        applied_before_ = [float(v) for v in slam["tracker"].applied[0]]
        jump_m, jump_rad = self.release_slam(slam_odom_base())
        after_ = slam_rear(now_s)
        frame_change = pose_compose(tuple(float(v) for v in after_), pose_invert(tuple(float(v) for v in before_)))
        slam["holds"].append({"phase": ms.phase, "time_s": now_s, "event": hold_event, "jump_m": jump_m,
                              "jump_rad": jump_rad})
        # As a D7c release: the grid re-projected with the released correction before any plan reads it.
        reprojected_ = self._reproject_after_release(now_s, after_, loaded)

        def carried(site_):
            x_, y_, yaw_ = pose_compose(frame_change, (site_.x_m, site_.y_m, site_.yaw_rad))
            return replace(site_, x_m=x_, y_m=y_, yaw_rad=yaw_)

        # The estimates made inside the hold move with it (release_and_stand's); map targets made before it do not
        # (carry False -- the bottom-deck side stage's).
        if carry and obstacle is not None and obstacle.get("pickup_estimate") is not None:
            obstacle["pickup_estimate"] = carried(obstacle["pickup_estimate"])
        if carry and pocket.get("planning_pickup") is not None:
            pocket["planning_pickup"] = carried(pocket["planning_pickup"])
        return {"jump_m": jump_m, "jump_rad": jump_rad, "frame_change": frame_change, "before": before_,
                "after": after_, "applied_before": applied_before_, "reprojected": reprojected_}

    def release_after_backoff(self, now_s, event):
        """Obstacle recovery v4 E3: a SLAM hold -- whenever it began -- released at a backoff's or
        escape's end, or in a recovery during one, the way the stall replan does: the grid
        re-projected before any plan reads it; returns the released rear. Not while the delivery
        docking is accepted or re-approaching."""
        initial_time = self.options.initial_time
        loaded = self.hooks.loaded()
        ms = self.ms
        obstacle = self.obstacle
        slam = self.slam
        slam_odom_base = self.hooks.slam_odom_base
        slam_rear = self.hooks.slam_rear
        world = self.hooks.world()  # the runner's, read when called
        if slam is None or slam["tracker"].mode != "holding":
            return None
        if ms.phase == "transport" and (slam["docking"].get("accepted_any") or slam["docking"].get("reapproach")):
            return None
        jump_m, jump_rad = self.release_slam(slam_odom_base())
        slam["holds"].append({"phase": ms.phase, "time_s": now_s, "event": event, "jump_m": jump_m, "jump_rad": jump_rad})
        rear_now = slam_rear(world.current_time - initial_time)
        if obstacle is not None and slam["tracker"].applied is not None:
            obstacle["applied"] = tuple(float(v) for v in slam["tracker"].applied[0])
            obstacle["version"] += 1
            obstacle["reprojections"] = obstacle.get("reprojections", 0) + 1
            obstacle["layer"].refresh(
                now_s, obstacle["applied"], obstacle["version"],
                current_pose=tuple(float(v) for v in rear_now), path_ahead=np.array([rear_now]), loaded=loaded,
            )
        slam["control"][-1][1:4] = np.asarray(rear_now, dtype=float).tolist()
        return np.asarray(rear_now, dtype=float)

    def slam_release(self, t: float) -> None:
        """Apply the received correction while stopped; replan the next leg
        from the new estimate if it moved more than 2 cm or 0.02 rad."""
        path_record = self.hooks.path_record  # a runner module-level name
        require = self.hooks.require  # a runner module-level name
        time = self.hooks.time_module()  # the runner's time (the recorded Clock proxy when recording)
        add_path_display = self.hooks.add_path_display
        core = self
        geometry = self.geometry
        grid_kwargs = self.hooks.grid_kwargs
        grid_planning = self.options.grid_planning
        grid_world = self.hooks.grid_world
        ms = self.ms
        paths = self.paths
        plan_return_leg = self.hooks.plan_return_leg
        plan_transport_leg = self.hooks.plan_transport_leg
        planner_config = self.options.planner_config
        return_to_pose = self.options.return_to_pose
        scenario = self.options.scenario
        slam = self.slam
        slam_odom_base = self.hooks.slam_odom_base
        slam_rear = self.hooks.slam_rear
        stage = self.hooks.stage()  # the runner's, read when called
        state = self.state
        trackers = self.trackers
        travel_config = self.options.travel_config
        leg = "transport" if slam["pending_release"] == "transport" else "return_home"
        slam["pending_release"] = None
        before = slam_rear(t)
        jump_m, jump_rad = self.release_slam(slam_odom_base())
        after = slam_rear(t)
        event = {
            "phase": ms.phase,
            "time_s": t,
            "event": "release",
            "jump_m": jump_m,
            "jump_rad": jump_rad,
            "rear_before": before.tolist(),
            "rear_after": after.tolist(),
            "replanned": False,
        }
        slam["holds"].append(event)
        # After an accepted docking match the withdraw (and so the return's
        # start) has moved: the return is replanned whatever the jump (Codex
        # 8428113 P2: a 0 jump left the old return start 366 mm away).
        docked_return = leg == "return_home" and slam["docking"].get("accepted_any")
        if leg not in trackers or (
            jump_m <= 0.02 and abs(jump_rad) <= 0.02 and not docked_return
        ):
            if leg == "transport":
                core.arm_docking()
            return
        start = PlanningPose(float(after[0]), float(after[1]), float(after[2]))
        released_ = (tuple(float(v) for v in slam["tracker"].applied[0])
                     if slam["tracker"].applied is not None else None)
        replan_start = time.monotonic()
        if self.jobs is not None:
            # Real-time plan D8b: the release itself is this tick's; the leg's replan is the worker's, and the leg
            # stands until it is taken (its delivered pallet carried into the released frame here, as below).
            return_scenario = None
            if leg != "transport":
                return_scenario = slam.get("transport_scenario", scenario)
                if slam["docking"].get("accepted_any"):
                    frame_change = compose(tuple(after), invert(tuple(before)))
                    site = return_scenario.destination
                    moved_site = compose(frame_change, (site.x_m, site.y_m, site.yaw_rad))
                    return_scenario = replace(
                        return_scenario,
                        destination=replace(site, x_m=moved_site[0], y_m=moved_site[1], yaw_rad=moved_site[2]),
                    )
                    event["pallet_frame_change"] = list(frame_change)
                    slam["return_scenario"] = return_scenario
            self._release_request(t, {"leg": leg, "event": event, "after": tuple(float(v) for v in after),
                                      "return_scenario": return_scenario, "redo": False})
            return
        if leg == "transport":
            replanned = plan_transport_leg(
                grid_world(scenario), start, planner_config, geometry=geometry, travel_config=travel_config,
                **grid_kwargs(None, own=after, applied=released_),
            )
        else:
            # The delivered pallet sits where docking put it -- right in the
            # held frame. Carry it into the released frame with the same
            # change of frame as the truck, T = after o before^-1, so the
            # truck-pallet relation survives the release (S3 seed 1 on a351056:
            # invalid_start against the nominal pallet after a docked drop).
            return_scenario = slam.get("transport_scenario", scenario)
            if slam["docking"].get("accepted_any"):
                frame_change = compose(tuple(after), invert(tuple(before)))
                site = return_scenario.destination
                moved_site = compose(frame_change, (site.x_m, site.y_m, site.yaw_rad))
                return_scenario = replace(
                    return_scenario,
                    destination=replace(
                        site, x_m=moved_site[0], y_m=moved_site[1], yaw_rad=moved_site[2]
                    ),
                )
                event["pallet_frame_change"] = list(frame_change)
                slam["return_scenario"] = return_scenario  # later return recoveries too
            replanned = plan_return_leg(
                grid_world(return_scenario),
                start,
                return_to_pose,
                planner_config,
                geometry=geometry,
                travel_config=travel_config,
                **grid_kwargs(None, own=after, applied=released_),
            )
        if replanned.status == "invalid_start" and grid_planning:
            # The released pose may sit inside the planning clearance of a grid
            # mark (L3b seed 3: transport after the extract); as for the obstacle
            # replans, once more with no clearance -- the grid's swelling holds
            # the placement error and the permission guards every tick.
            tight_cfg = replace(planner_config, clearance_m=0.0)
            tight_travel_cfg = replace(travel_config, clearance_m=0.0) if travel_config is not None else None
            if leg == "transport":
                replanned = plan_transport_leg(
                    grid_world(scenario), start, tight_cfg, geometry=geometry, travel_config=tight_travel_cfg,
                    **grid_kwargs(None, own=after, applied=released_),
                )
            else:
                replanned = plan_return_leg(
                    grid_world(return_scenario), start, return_to_pose, tight_cfg, geometry=geometry,
                    travel_config=tight_travel_cfg, **grid_kwargs(None, own=after, applied=released_),
                )
            event["tight_retry"] = True
        event.update(
            replaced_path=path_record(paths[leg]),
            replanned=True,
            replan_status=replanned.status,
            planning_wall_s=time.monotonic() - replan_start,
        )
        require(replanned.success, f"slam_release_replan_failed: {replanned.status}")
        paths[leg] = replanned
        state.setdefault("paths", {})[leg] = path_record(replanned)
        self.hooks.write_paths()
        trackers[leg] = RearAxlePathTracker(
            replanned.poses,
            replanned.directions,
            replanned.curvatures_inv_m,
            trackers[leg].config,
        )
        add_path_display(
            stage,
            replanned,
            "Transport" if leg == "transport" else "Return",
            (1.0, 0.65, 0.04) if leg == "transport" else (0.55, 0.2, 0.85),
        )
        if leg == ms.phase:
            ms.phase_started = t
        if leg == "transport":
            core.arm_docking()

    def escape_leg(self, phase_):
        """Obstacle recovery v4: (kind, the leg's grid_kwargs args, its rectangle checker, outline)
        for transport or return_home -- the rectangles, outline and bounds its replan checks."""
        geometry = self.geometry
        grid_world = self.hooks.grid_world
        scenario = self.options.scenario
        slam = self.slam
        from forklift_core.planning.grid_collision import FootprintCollisionChecker
        from forklift_core.planning.pallet_mission import leg_obstacles

        if phase_ == "transport":
            sc_ = slam.get("transport_scenario", scenario) if slam is not None else scenario
            kind_, args_ = "transport", (None, None)
        else:
            sc_ = scenario if slam is None else slam.get("return_scenario", slam.get("transport_scenario", scenario))
            kind_, args_ = "return", ("return", sc_.destination)
        rects_, footprint_, bounds_ = leg_obstacles(kind_, grid_world(sc_), geometry)
        return kind_, args_, FootprintCollisionChecker(rects_, footprint_, bounds_), (rects_, footprint_, bounds_)

    def find_escape(self, now_s, rear_now, leg_dir_, kappa_now):
        """Obstacle recovery v4 E2: the straight escape for a stop inside the planning clearance --
        (the chosen candidate or None, every candidate). The candidates' checks: the leg's planner
        checker over the sweep (the permission's own-cell rule on the grid), the permission's path
        check to the length plus the stop length (and with the extra margin ramping in), the stop arcs
        at the held steering, and the end region on the grid rebuilt for the end pose."""
        BACKOFF_SPEED_MPS = self.options.backoff_speed_mps  # a runner module-level name
        escape_leg = self.escape_leg
        grid_kwargs = self.hooks.grid_kwargs
        loaded = self.hooks.loaded()
        ms = self.ms
        obstacle = self.obstacle
        planner_config = self.options.planner_config
        travel_config = self.options.travel_config
        from forklift_core.control import escape as esc
        from forklift_core.planning.grid_collision import make_checker

        _, args_, rect_checker_, (rects_, footprint_, bounds_) = escape_leg(ms.phase)
        pose_ = tuple(float(v) for v in rear_now)
        occ_now_ = grid_kwargs(*args_, label="escape")["occupancy"]
        layer_ = obstacle["layer"]
        loaded_now_ = bool(loaded)
        pconf_ = layer_.permission.config
        stop_len_ = pconf_.stopping.distance_m(BACKOFF_SPEED_MPS) + pconf_.step_m
        psi_ = float(layer_.error_table.at(layer_.grid_config.occupied_max_age_s)[1])
        clearance_ = float((travel_config or planner_config).clearance_m)

        def end_clearance(region):
            # One grid per candidate for its whole end region: the remembered cells under the truck
            # now come back once it has left them, and only the cells under every region pose stay
            # out (the return band likewise) -- each pose's own grid is contained in it.
            occ_end_ = grid_kwargs(*args_, own=list(region), band_pose=list(region), label="escape_end")["occupancy"]
            checker_ = make_checker(rects_, footprint_, bounds_, occ_end_)
            values_ = [esc.clearance_m(checker_.free, p_) for p_ in region]
            return min(values_) >= 0.0, min(values_)

        candidates_ = esc.evaluate(
            pose_,
            sweep_free=lambda poses, d: esc.rectangles_sweep_free(rect_checker_, poses)
            and esc.grid_sweep_blocked(occ_now_, footprint_, pose_, d, poses) is None,
            path_blocked=lambda poses, d, extra, ramp: layer_.check_candidate(
                poses, current_pose=pose_, direction=d, loaded=loaded_now_, extra_margin_m=extra, extra_ramp_m=ramp
            ).blocked,
            arc_blocked=lambda k, d: layer_.check_arc_candidate(
                current_pose=pose_, curvature_inv_m=k, direction=d, loaded=loaded_now_, cap_mps=BACKOFF_SPEED_MPS
            ).blocked,
            end_clearance=end_clearance,
            kappa_now=float(kappa_now), stop_length_m=stop_len_, planner_clearance_m=clearance_, psi_rad=psi_,
            cell_m=float(occ_now_.resolution_m),
        )
        choice_ = esc.choose(candidates_, int(leg_dir_))
        dumps_ = obstacle.setdefault("recovery_dumps", 0)
        if dumps_ < 6:
            # Plan v4 D4: the inputs of the decision, for a CPU replay.
            obstacle["recovery_dumps"] = dumps_ + 1
            snap_ = layer_.snapshot
            self.hooks.save_npz(
                f"obstacle_recovery_{dumps_ + 1}.npz",
                state=snap_.state, free_stamp=snap_.free_stamp, origin=[snap_.origin_x_m, snap_.origin_y_m],
                res=snap_.resolution_m, planner_occupied=occ_now_.occupied,
                planner_origin=[occ_now_.origin_x_m, occ_now_.origin_y_m], planner_res=occ_now_.resolution_m,
                t=now_s, rear=np.asarray(pose_), kappa=float(kappa_now), leg_dir=int(leg_dir_), phase=ms.phase,
                loaded=loaded_now_, candidates=json.dumps([c.record() for c in candidates_]),
                chosen=json.dumps(None if choice_ is None else choice_.record()),
            )
        return choice_, candidates_

    # --- M-a1 slice 6d: closures moved from run(), statement for statement ---
    # --- real-time plan D8: waiting for a plan without stopping the control loop -----------------------------------
    RETURN_CORRECTION_FALLBACK = 3  # plan D11: correction drops of a return turnaround before the tick plan
    ASYNC_DRIFT_M = 0.02
    ASYNC_DRIFT_RAD = math.radians(1.0)
    ASYNC_MAX_DISCARDS = 3
    ASYNC_FRESH_DEPTH_S = 2.0
    ASYNC_STAND_CONFIRM_S = 2.0  # how long a finished plan waits for the stop detector before it is dropped

    def _leg_payload(self, kind: str, scenario_, start, grid_kw: dict, *, retry_on) -> dict:
        """A ``leg`` job: the leg planner's call and its tight retry, as the tick would make them."""
        options = self.options
        cfg, travel = options.planner_config, options.travel_config
        tight_cfg = replace(cfg, clearance_m=0.0)
        tight_travel = replace(travel, clearance_m=0.0) if travel is not None else None
        exit_kw = {}
        if kind == "transport":
            args, tight_args = (scenario_, start, cfg), (scenario_, start, tight_cfg)
            exit_kw = self._exit_prefix_kwargs()
        else:
            args = (scenario_, start, options.return_to_pose, cfg)
            tight_args = (scenario_, start, options.return_to_pose, tight_cfg)
        return {"kind": kind, "args": args,
                "kwargs": dict(geometry=self.geometry, travel_config=travel, **grid_kw, **exit_kw),
                "retry_on": tuple(retry_on), "tight_args": tight_args,
                "tight_kwargs": dict(geometry=self.geometry, travel_config=tight_travel, **grid_kw, **exit_kw)}

    def _exit_prefix_kwargs(self) -> dict:
        """The worker's transport planners get the run's exit prefix as the hooks' do (the runner binds it there;
        docs/plans/2026-10-22-riibotics-trailer-exit.md D1); none when the run has none."""
        exit_ = getattr(self.hooks, "exit_prefix", None)
        return {} if exit_ is None else {"exit_prefix": exit_}

    def _refresh_for_new_leg(self, t: float, rear) -> None:
        """Right after a plan is taken: the permission's path check on the new leg before the first move (D8b)."""
        obstacle, ms, trackers = self.obstacle, self.ms, self.trackers
        if obstacle is None or ms.phase not in trackers:
            return
        leg_now, leg_dir_now = trackers[ms.phase].leg_ahead()
        obstacle["layer"].refresh(
            t, obstacle["applied"], obstacle["version"], current_pose=tuple(float(v) for v in rear),
            path_ahead=leg_now, loaded=ms.phase in ("lift", "extract", "transport", "lower"), direction=leg_dir_now)

    def _release_request(self, t: float, need: dict) -> None:
        """The release replan of ``need["leg"]``: from the released pose the first time, from where the truck
        stands now when asked again after a drop; its grid and the start share the correction applied now."""
        hooks, options = self.hooks, self.options
        leg = need["leg"]
        after = tuple(float(v) for v in (hooks.slam_rear(t) if need["redo"] else need["after"]))
        start = PlanningPose(*after)
        released_ = self._applied_now()
        grid_kw = hooks.grid_kwargs(None, own=after, applied=released_)
        if leg == "transport":
            payload = self._leg_payload("transport", hooks.grid_world(options.scenario), start, grid_kw,
                                        retry_on=("invalid_start",) if options.grid_planning else ())
        else:
            payload = self._leg_payload("return", hooks.grid_world(need["return_scenario"]), start, grid_kw,
                                        retry_on=("invalid_start",) if options.grid_planning else ())
        self._request("release", "leg", payload, t, after, released_, ctx={"need": dict(need, redo=True)})

    def _release_finish(self, p, value, t: float, rear) -> None:
        ms, state, paths, trackers, hooks = self.ms, self.state, self.paths, self.trackers, self.hooks
        need = p.ctx["need"]
        leg, event = need["leg"], need["event"]
        replanned = value["result"]
        if value.get("tight"):
            event["tight_retry"] = True
        event.update(replaced_path=hooks.path_record(paths[leg]), replanned=True, replan_status=replanned.status,
                     planning_wall_s=hooks.wall() - p.wall, async_taken_s=float(t))
        hooks.require(replanned.success, f"slam_release_replan_failed: {replanned.status}")
        paths[leg] = replanned
        state.setdefault("paths", {})[leg] = hooks.path_record(replanned)
        hooks.write_paths()
        trackers[leg] = RearAxlePathTracker(replanned.poses, replanned.directions, replanned.curvatures_inv_m,
                                            trackers[leg].config)
        hooks.add_path_display(hooks.stage(), replanned, "Transport" if leg == "transport" else "Return",
                               (1.0, 0.65, 0.04) if leg == "transport" else (0.55, 0.2, 0.85))
        if leg == ms.phase:
            ms.phase_started = p.t
        if leg == "transport":
            self.arm_docking()
        self._refresh_for_new_leg(t, rear)

    def _live_leg_finish(self, p, value, t: float, rear) -> None:
        ms, obstacle, state, paths, trackers, hooks, slam = (self.ms, self.obstacle, self.state, self.paths,
                                                             self.trackers, self.hooks, self.slam)
        live = value["result"]
        obstacle.setdefault("live_plans", []).append(
            {"phase": ms.phase, "time_s": p.t, "status": live.status, "turnaround": getattr(live, "turnaround", None),
             "async": True, "taken_s": float(t)})
        if not live.success:
            tries = sum(1 for lp in obstacle["live_plans"] if lp["phase"] == ms.phase)
            hooks.require(tries < 5, f"obstacle_live_plan_failed in {ms.phase}: {live.status}")
            obstacle["live_replanned"][ms.phase] = False
            obstacle["live_retry_after_s"] = t + 1.0  # from now: the wait is not a retry interval (Codex D8 P2-6)
            return
        paths[ms.phase] = live
        state["paths"][ms.phase] = hooks.path_record(live)
        hooks.write_paths()
        trackers[ms.phase] = RearAxlePathTracker(live.poses, live.directions, live.curvatures_inv_m,
                                                 trackers[ms.phase].config)
        if ms.phase == "transport" and slam is not None:
            self.arm_docking()
        ms.phase_started = p.t  # with the wait out of the work clock, as the tick that asked would have set it
        self._refresh_for_new_leg(t, rear)

    def _dock_finish(self, p, value, t: float, rear) -> None:
        """D9b: the match taken -- the rest of the docking from it, then this tick's pose as the asking call sites
        read it after the docking (a retry may have released the correction)."""
        hooks, slam = self.hooks, self.slam
        self.state["async_plans"][-1]["dock_reason"] = getattr(value["result"], "reason", None)
        self.dock_at_delivery_straight(t, answer=(p.ctx, value["result"], value.get("wall_s", 0.0)))
        rear_ = hooks.slam_rear(t)
        hooks.set_rear(rear_)
        slam["control"][-1][1:4] = rear_.tolist()

    def job_pending(self) -> bool:
        return self.pending is not None

    def _applied_now(self):
        slam = self.slam
        if slam is None or slam["tracker"].applied is None:
            return None
        return tuple(float(v) for v in slam["tracker"].applied[0])

    def _request(self, site: str, name: str, payload: dict, t: float, rear, applied, ctx: dict) -> None:
        """Ask the worker for a plan; the work clocks stand still until it is taken or dropped (D8c)."""
        slam = self.slam
        if self.pending is not None:  # one plan waited for at a time (Codex D8 implementation P1)
            self._drop(self.pending, t, "superseded", count=False)
        job = self.jobs.submit(name, payload)
        self.commands.begin_wait(t)
        self.pending = SimpleNamespace(
            site=site, job=job, t=float(t), rear=tuple(float(v) for v in rear), applied=applied,
            mode=slam["tracker"].mode if slam is not None else None, hold=self.hold_count(),
            gen=self.commands.mission_gen, phase=self.ms.phase, ctx=ctx, wall=self.hooks.wall(), moving_since=None,
            answer=None, answered_s=None)
        self.state.setdefault("async_plans", []).append(
            {"site": site, "job": job, "requested_s": float(t), "phase": self.ms.phase,
             "asked": self._plan_context(rear, applied)})

    def _plan_context(self, rear, applied) -> dict:
        """What a plan was asked on and taken on, for the run's record (Codex real-time D8 round 10: the async
        runs could not be told apart from the synchronous ones without it): the control and true rear poses, the
        applied correction, the SLAM mode and hold count."""
        slam = self.slam
        return {"rear": [float(v) for v in rear], "truth_rear": [float(v) for v in self.hooks.truth_rear()],
                "applied": None if applied is None else [float(v) for v in applied],
                "mode": slam["tracker"].mode if slam is not None else None, "hold": self.hold_count()}

    def _job_invalid(self, p, t: float, rear) -> str | None:
        """Why the plan p may not be taken now (None: it may), judged on the current state (D8b). A preparation
        (persistent console plan D2) is judged on its own status and on the targets list it was asked on."""
        cmds, slam = self.commands, self.slam
        if p.site == "prepare":
            if cmds.status != "preparing" or cmds.mission_gen != p.gen:
                return "mission"
            if self.targets.revision != p.ctx["targets_revision"]:
                return "targets"
            if p.ctx.get("loaded") and not self.hooks.loaded_cargo_ok():
                return "cargo"  # plan D5 6c item 4a: the carried pallet is no longer what the submit delivers
        elif cmds.status != "running" or cmds.mission_gen != p.gen:
            return "mission"
        elif self.ms.phase != p.phase:
            return "phase"
        if self._applied_now() != p.applied:
            return "correction"
        if (p.site == "live_leg" and (p.ctx or {}).get("turnaround")
                and getattr(self.hooks, "test_return_correction_drops", 0) > 0):
            self.hooks.test_return_correction_drops -= 1  # plan D11 tests only: the fallback forced
            return "correction"
        if slam is not None and (slam["tracker"].mode != p.mode or self.hold_count() != p.hold):
            return "slam_mode"
        if slam is not None and not slam["stop_now"]:
            return "moving"
        dx, dy = float(rear[0]) - p.rear[0], float(rear[1]) - p.rear[1]
        dyaw = math.atan2(math.sin(float(rear[2]) - p.rear[2]), math.cos(float(rear[2]) - p.rear[2]))
        if math.hypot(dx, dy) > self.ASYNC_DRIFT_M or abs(dyaw) > self.ASYNC_DRIFT_RAD:
            return "drift"
        return None

    def job_hold(self, t: float, rear, truth_speed: float, paused: bool) -> SimpleNamespace:
        """At each control tick's start in a real-time run (D8b): whether this tick stands for a plan. A finished
        plan is checked and taken (or dropped) here, once; a dropped release replan is asked again when the
        mission runs; the first fresh depth frame after an asynchronous near build is watched."""
        hooks, state = self.hooks, self.state
        p = self.pending
        if p is not None and paused:
            # A pause or a cancel: the plan belongs to the mission as it was; drop it, ask again when running.
            self._drop(p, t, "paused", count=False)
            return SimpleNamespace(hold=False)
        if p is None:
            if self.fresh_depth_wait is not None:
                since, stamp = self.fresh_depth_wait
                check = self.pocket.get("check")
                last_new = getattr(check, "last_new_s", None) if check is not None else None
                cfg = getattr(check, "config", None)
                lag, max_age = getattr(cfg, "frame_lag_s", 0.0), getattr(cfg, "frame_max_age_s", 0.2)
                if last_new is not None and last_new > stamp and t - (last_new - lag) <= max_age:
                    self.fresh_depth_wait = None  # a new pocket frame the check itself takes as fresh
                else:
                    hooks.require(t - since <= self.ASYNC_FRESH_DEPTH_S, "async_near_no_fresh_depth")
            if self.release_replan_needed is not None and not paused and self.commands.status == "running":
                need, self.release_replan_needed = self.release_replan_needed, None
                self._release_request(t, need)
                return SimpleNamespace(hold=True)
            if self.observe_failed_needed is not None and not paused and self.commands.status == "running":
                ctx, self.observe_failed_needed = self.observe_failed_needed, None
                self._observe_failed_request(t, rear, ctx)
                return SimpleNamespace(hold=True)
            return SimpleNamespace(hold=False)
        if abs(float(truth_speed)) > 0.05:
            p.moving_since = t if p.moving_since is None else p.moving_since
            hooks.require(t - p.moving_since <= 1.0, "planning_wait_moving")
        else:
            p.moving_since = None
        out = p.answer if p.answer is not None else self.jobs.poll(p.job)
        if out is None:
            return SimpleNamespace(hold=True)
        why = self._job_invalid(p, t, rear)
        if why == "moving" and (p.answered_s is None or t - p.answered_s <= self.ASYNC_STAND_CONFIRM_S):
            # The stop detector confirms a stand some ticks after the wheels stop (Isaac d8e_p: a plan asked as the
            # truck came to rest came back 0.18 s later, 0.3 s before the confirmation, and was dropped). The
            # answer is kept and judged again each tick; a real motion still ends the run (planning_wait_moving),
            # and a stand never confirmed drops it after ASYNC_STAND_CONFIRM_S.
            if p.answer is None:
                p.answer, p.answered_s = out, float(t)
            return SimpleNamespace(hold=True)
        ok, value = out
        if not ok and getattr(self.jobs, "_dead", None) is not None:
            # No worker any more (its restart failed): not the plan's failure, the session's (Codex D6 P2-4).
            hooks.require(False, f"plan_worker_dead: {self.jobs._dead}")
        self.pending = None
        self.commands.end_wait(t)
        record = state["async_plans"][-1]
        record.update(finished_s=float(t), wall_s=hooks.wall() - p.wall, ok=bool(ok), dropped=why,
                      taken=self._plan_context(rear, self._applied_now()))
        if p.site == "prepare":
            self._prepare_answer(p, why, ok, value, t)
            return SimpleNamespace(hold=True)
        if why is not None:
            self._drop(p, t, why, count=True)
            return SimpleNamespace(hold=True)
        if not ok and p.site == "scene_replan":
            self.scene_replan.abort("async_plan_failed")  # its record closed (Codex impl round 1 P3)
        hooks.require(ok, f"async_plan_failed:{str(value).splitlines()[0] if value else ''}")
        {"observe": self._observe_finish, "release": self._release_finish,
         "live_leg": self._live_leg_finish, "dock": self._dock_finish,
         "observe_failed": self._observe_failed_finish, "scene_replan": self._scene_finish}[p.site](p, value, t, rear)
        return SimpleNamespace(hold=True)

    def _drop(self, p, t: float, why: str, *, count: bool) -> None:
        if self.pending is p:
            self.pending = None
            self.commands.end_wait(t)
            self.jobs.discard(p.job)
            self.state["async_plans"][-1].update(finished_s=float(t), dropped=why)
        if (count and why == "correction" and p.site == "live_leg" and (p.ctx or {}).get("turnaround")):
            # Plan D11: a return turnaround's correction drop has its own count (the shared one ended the mission
            # before the fallback could run -- Codex design round 4 P2); the other reasons count as before.
            key = p.phase
            self.return_correction_drops[key] = self.return_correction_drops.get(key, 0) + 1
            self.state.setdefault("return_correction_drops", []).append(
                {"time_s": float(t), "phase": p.phase, "count": self.return_correction_drops[key]})
            count = False
        if p.site == "scene_replan":
            # Scene-mode replan plan 2.1 (Codex round 2 P2-3): the device asks again and bounds its own drops (3 per
            # request, 4 requests); outside the shared per-site count.
            self.scene_mail = ("dropped", why)
            count = False
        if count:
            drops = self.state.setdefault("async_plan_drops", {})
            drops[p.site] = drops.get(p.site, 0) + 1
            self.hooks.require(drops[p.site] <= self.ASYNC_MAX_DISCARDS, f"async_plan_discarded:{p.site}:{why}")
        if p.site == "release":
            self.release_replan_needed = p.ctx["need"]
        elif p.site == "live_leg":
            self.obstacle.setdefault("live_replanned", {})[p.phase] = False
        elif p.site == "observe_failed":
            self.observe_failed_needed = p.ctx  # asked again from where the truck stands when the mission runs
        # observe: nothing to undo -- the truck stands at the observation pose and captures again

    # --- a live mission's preparation (docs/plans/2026-10-18-persistent-live-console.md D2) ------------------------
    PREPARE_ATTEMPTS = 2  # a preparation whose inputs changed is asked again once

    def prepare_request(self, t: float, rear, inputs: dict, payload: dict, job: str = "observe_ladder") -> None:
        """Ask the worker for a submitted mission's first leg (``observe_ladder``) from where the truck stands -- or,
        a loaded submit (plan D5 6c), its whole delivery (``loaded_delivery``). ``inputs`` (the mission's scenario,
        waiting pose, candidates, bounds, prior zone) are fixed here and used as they are at the adoption; nothing of
        the running state changes before it (plan principle P4)."""
        cmds = self.commands
        if cmds.status == "accepted":
            cmds.begin_prepare()
            self.prepare_attempts = 0
        self.prepare_attempts += 1
        self._request("prepare", job, payload, t, rear, self._applied_now(),
                      {"inputs": inputs, "targets_revision": self.targets.revision,
                       "loaded": job == "loaded_delivery"})

    def _prepare_answer(self, p, why: str | None, ok: bool, value, t: float) -> None:
        """A preparation's answer: dropped and asked again once when what it rests on changed (not counted as a
        plan drop), failed when it planned nothing, else adopted."""
        cmds = self.commands
        loaded = bool(p.ctx.get("loaded"))
        if why == "cargo":
            cmds.terminate("failed", t, "prepare:loaded:cargo_changed")
            return
        if why is not None:
            if why != "mission" and self.prepare_attempts >= self.PREPARE_ATTEMPTS:
                cmds.terminate("failed", t, f"prepare:{why}")
            return  # the runner asks again on the next tick (preparing, nothing pending)
        if not ok:
            cmds.terminate("failed", t, f"prepare:error:{str(value).splitlines()[0] if value else ''}")
        elif loaded and not value["result"].success:
            cmds.terminate("failed", t, f"prepare:loaded:{value['result'].status}")  # its stage and status (item 3)
        elif value["result"] is None:
            cmds.terminate("failed", t, "prepare:timeout" if value["timed_out"] else
                           "prepare:all_candidates_failed:" + ";".join(c["status"] for c in value["candidates"]),
                           advice=value.get("advice"))  # plan D13 item 6: the worker's, on this job's inputs
        else:
            try:  # what can fail is made before anything changes (Codex stage 2 impl P2-6): the docking reference
                ready = self.hooks.live_mission_ready(t, p.ctx["inputs"])
            except Exception as exc:  # noqa: BLE001 -- the preparation fails, the session goes on (P4)
                cmds.terminate("failed", t, f"prepare:reference:{exc}")
                return
            if loaded:
                self.start_loaded_mission(t, value["result"], p.ctx["inputs"], ready)
            else:
                self.start_mission(t, value["result"], p.ctx["inputs"], value, ready)

    # --- the scene-mode replan device (docs/plans/2026-10-23-scene-replan.md) ---------------------------------------
    SCENE_PHASES = ("transport", "return_home")

    def scene_replan_allowed(self) -> bool:
        """A request may be raised: a scene layout, a travel leg, none of the rules that keep their own recoveries
        (plan 2.1: a docking accepted or re-approaching, near, the bottom-deck legs, a backoff or escape)."""
        ms, slam, obstacle = self.ms, self.slam, self.obstacle
        if not getattr(self.options, "pickup_prior", False) or ms.phase not in self.SCENE_PHASES:
            return False
        if slam is not None and (slam["docking"].get("accepted_any") or slam["docking"].get("reapproach")):
            return False
        if obstacle is not None and obstacle.get("backoff", {}).get("phase") == ms.phase:
            return False
        if self.near_active() or self.bottom_deck_leg_active():
            return False
        return True

    def scene_far_observe(self) -> bool:
        """Plan 2.3 (B): the first far observation in a scene layout -- before recognition, and none of the near
        capture, its realignment, the bottom-deck legs or a backoff."""
        ms, obstacle = self.ms, self.obstacle
        if not getattr(self.options, "pickup_prior", False) or ms.phase != "observe":
            return False
        if obstacle is not None and (obstacle.get("pickup_estimate") is not None
                                     or obstacle.get("backoff", {}).get("phase") == "observe"):
            return False
        if self.pocket.get("planning_pickup") is not None:
            return False
        return not (self.near_active() or self.realign_active() or self.bottom_deck_leg_active())

    def scene_observe_candidate(self, t: float, rear):
        """Plan 2.3 (B): from the stop -- the hold released first, whatever follows (the candidates are map poses,
        and the same waypoint's replan after them starts from the released pose too, Codex impl round 1 P2-3) -- the
        next observation candidate, one per call (the tick stays short live as the waypoint's own replan does,
        P2-2); installed with its own replan budget when it plans. Returns (installed, tried, rear)."""
        ms, state, slam, hooks, options = self.ms, self.state, self.slam, self.hooks, self.options
        paths, trackers, geometry = self.paths, self.trackers, self.geometry
        rear_ = np.asarray(rear, dtype=float)
        if slam is not None and slam["tracker"].mode == "holding":
            out_ = self.release_and_carry(t)
            if out_.get("released"):
                rear_ = np.asarray(out_["rear"], dtype=float)
                hooks.set_rear(rear_)
                if slam.get("control"):
                    slam["control"][-1][1:4] = rear_.tolist()
        if ms.next_candidate_index >= len(options.observation_waypoints or ()):
            return SimpleNamespace(installed=False, tried=False, rear=rear_)
        start_ = PlanningPose(float(rear_[0]), float(rear_[1]), float(rear_[2]))
        if ms.next_candidate_index < len(options.observation_waypoints):
            index_ = ms.next_candidate_index
            ms.next_candidate_index += 1
            coordinates_ = options.observation_waypoints[index_]
            plan_ = hooks.plan_observation_leg(
                hooks.grid_world(options.scenario), PlanningPose(*(float(v) for v in coordinates_)),
                options.planner_config, geometry=geometry, start_rear=start_, pickup_bounds=self.observe_bounds(),
                extended=False, **hooks.grid_kwargs("observe"))
            state["observation_candidates"].append(
                {"candidate_index": index_, "pose": list(coordinates_), "start_rear": [float(v) for v in rear_],
                 "success": plan_.success, "status": plan_.status, "after": "scene_observe_stall"})
            state.setdefault("scene_replans", []).append(
                {"time_s": float(t), "trigger": "observe_stall", "phase": "observe", "candidate_index": index_,
                 "rear": [float(v) for v in rear_], "status": plan_.status,
                 "outcome": "installed" if plan_.success else "failed"})
            if plan_.success:
                state["observation_waypoint_selected"] = list(coordinates_)
                state["observe_replan_leg"] = {"serial": state["observe_replan_leg"]["serial"] + 1,
                                               "start": len(state["observe_replans"])}
                paths["observe"] = plan_
                state.setdefault("paths", {})["observe"] = hooks.path_record(plan_)
                hooks.write_paths()
                trackers["observe"] = RearAxlePathTracker(plan_.poses, plan_.directions, plan_.curvatures_inv_m,
                                                          trackers["observe"].config)
                ms.phase_started = t
                return SimpleNamespace(installed=True, tried=True, rear=rear_)
        return SimpleNamespace(installed=False, tried=True, rear=rear_)

    def scene_correction_note(self, t: float, net_m: float, reply_max_m: float, net_yaw_rad: float = 0.0) -> None:
        """Plan 2.4: the largest correction contribution since the current gear leg began (its tracker and leg)."""
        ms, trackers = self.ms, self.trackers
        key_ = (id(trackers[ms.phase]), trackers[ms.phase].leg_index) if ms.phase in trackers else None
        c_ = self.scene_correction
        if c_.get("key") != key_:
            c_.clear()
            c_.update(key=key_, since_s=float(t), net_max_m=0.0, net_at_s=None, reply_max_m=0.0, net_yaw_rad=0.0)
        if net_m > c_["net_max_m"]:
            c_.update(net_max_m=float(net_m), net_at_s=float(t), net_yaw_rad=float(net_yaw_rad))
        c_["reply_max_m"] = max(c_["reply_max_m"], float(reply_max_m))

    def _scene_control_jump(self) -> dict | None:
        """Plan 2.4 (simulation): the largest one-tick change of control minus truth since the current gear leg began."""
        slam = self.slam
        since_ = self.scene_correction.get("since_s")
        if slam is None or since_ is None or not slam.get("control"):
            return None
        best_, at_, prev_ = 0.0, None, None
        for row in slam["control"]:
            if row[0] < since_ or len(row) < 7:
                continue
            d_ = (row[1] - row[4], row[2] - row[5])
            if prev_ is not None:
                step_ = math.hypot(d_[0] - prev_[0], d_[1] - prev_[1])
                if step_ > best_:
                    best_, at_ = step_, float(row[0])
            prev_ = d_
        return {"step_max_m": best_, "at_s": at_}

    def scene_replan_raise(self, t: float, trigger: str, rear, *, deviation=None, extra_obstacles_fn=None,
                           clearance_m=None, detail=None) -> str | None:
        """Raise the device's request (plan 2.1): None when taken or noted, the end reason when the budget is spent.
        The tick that raises it stands (``scene_raised``)."""
        ms, trackers, hooks = self.ms, self.trackers, self.hooks
        self.scene_replan.records = self.state.setdefault("scene_replans", [])
        tracker_ = trackers.get(ms.phase)
        truth_ = getattr(hooks, "truth_rear", None)
        correction_ = {k: v for k, v in self.scene_correction.items() if k != "key"} or None
        jump_ = self._scene_control_jump()
        why_ = self.scene_replan.request(
            t, trigger, phase=ms.phase, tracker_serial=None if tracker_ is None else id(tracker_) % 10**6,
            leg=None if tracker_ is None else tracker_.leg_index, rear=rear,
            truth_rear=None if truth_ is None else truth_(),
            deviation=deviation, correction_jump={"slam": correction_, "control_minus_truth": jump_},
            extra_obstacles_fn=extra_obstacles_fn, clearance_m=clearance_m, detail=detail)
        if why_ is None:
            self.scene_raised = True
        return why_

    def scene_replan_request(self, t: float, trigger: str, *, rear=None, extra_obstacles_fn=None, clearance_m=None,
                             detail=None) -> str | None:
        """The device's entry for a trigger outside the follower (plan 2.1, the permit-stop trigger of the
        permission-clearance session): the same as a follower's raise; ``rear`` defaults to the runner's."""
        rear_ = rear if rear is not None else self.hooks.rear()
        return self.scene_replan_raise(t, trigger, rear_, extra_obstacles_fn=extra_obstacles_fn,
                                       clearance_m=clearance_m, detail=detail)

    def _scene_finish(self, p, value, t: float, rear) -> None:
        result = value["result"]
        self.scene_mail = ("ok", result) if result.success else ("ok_fail", result.status)

    def _scene_preview(self, tracker, t: float, rear, dt: float) -> dict:
        """Plan 2.1 steps 6 and 9: a tracker copy's first command from the stop pose -- its status branched first; a
        failure-free braking (a cusp switch) previewed one tick further -- and the permission's candidate check on
        the leg it starts, at that command's curvature and at the wheels' measured one."""
        obstacle, hooks = self.obstacle, self.hooks
        pose_ = np.asarray(rear, dtype=float)
        pv = tracker.update(pose_, 0.0, dt)
        for _ in range(2):
            if pv.status == "failed":
                return {"status": "failed", "ok": False, "why": f"failed:{pv.failure}"}
            if getattr(pv, "off_path", False):
                return {"status": "off_path", "ok": False, "why": "off_path"}
            if pv.status == "arrived":
                return {"status": "arrived", "ok": True, "why": "arrived"}
            if getattr(pv, "cusp_pending", False):
                return {"status": "cusp_pending", "ok": True, "why": "cusp_pending"}
            if pv.status != "braking":
                break
            pv = tracker.update(pose_, 0.0, dt)  # the switch to the next gear leg: the leg actually started
        else:
            return {"status": "failed", "ok": False, "why": "braking_twice"}
        leg_, direction_ = tracker.leg_ahead()
        kappa_ = float(pv.curvature_inv_m)
        wheelbase_ = float(self.options.drive_geometry.wheelbase_m)
        out = {"status": "tracking", "kappa": kappa_, "steer": kappa_, "direction": int(direction_),
               "wheelbase_m": wheelbase_}
        if obstacle is None:
            out.update(ok=True, why="no_layer")
            return out
        permit_ = obstacle["layer"].candidate_permit(
            t, leg_, current_pose=tuple(float(v) for v in pose_), direction=int(direction_), loaded=hooks.loaded(),
            curvature_inv_m=kappa_, measured_curvature_inv_m=float(hooks.held_kappa()),
            cap_mps=float(tracker.config.cruise_speed_mps))
        out.update(ok=bool(permit_["ok"]), why=permit_["why"], permit=permit_)
        return out

    def _scene_ports(self, t: float, rear, signed_speed: float, dt: float) -> SimpleNamespace:
        """The device's world this tick (plan 2.1)."""
        import copy

        from forklift_core.control import escape as esc
        from forklift_core.planning.grid_collision import FootprintCollisionChecker
        from forklift_core.planning.pallet_mission import AssetSpec, PlacedProp

        ms, slam, obstacle, hooks, options = self.ms, self.slam, self.obstacle, self.hooks, self.options
        paths, trackers, state, geometry = self.paths, self.trackers, self.state, self.geometry
        core = self
        box = SimpleNamespace(rear=np.asarray(rear, dtype=float))

        def stopped():
            return bool(slam["stop_now"]) if slam is not None else abs(float(signed_speed)) < 0.01

        def busy():
            pending_ = core.pending is not None and core.pending.site != "scene_replan"
            return pending_ or (slam is not None and slam.get("pending_release") is not None)

        def release():
            if slam is None or slam["tracker"].mode != "holding":
                return
            out_ = core.release_and_carry(t)
            if out_.get("released"):
                box.rear = np.asarray(out_["rear"], dtype=float)
                hooks.set_rear(box.rear)
                if slam.get("control"):
                    slam["control"][-1][1:4] = box.rear.tolist()

        def scenario_now(extra):
            scenario = options.scenario
            if ms.phase == "transport":
                sc_ = slam.get("transport_scenario", scenario) if slam is not None else scenario
            else:
                sc_ = scenario if slam is None else slam.get("return_scenario", slam.get("transport_scenario", scenario))
            if extra:
                sc_ = replace(sc_, props=tuple(sc_.props) + tuple(
                    PlacedProp(AssetSpec("scene_replan:extra", float(r.length_m), float(r.width_m), 1.0), r)
                    for r in extra))
            return sc_

        def submit(extra, clearance_m):
            cfg_, travel_ = options.planner_config, options.travel_config
            if clearance_m is not None:
                cfg_ = replace(cfg_, clearance_m=float(clearance_m))
                travel_ = replace(travel_, clearance_m=float(clearance_m)) if travel_ is not None else None
            sc_ = hooks.grid_world(scenario_now(extra))
            start_ = PlanningPose(float(box.rear[0]), float(box.rear[1]), float(box.rear[2]))
            if ms.phase == "transport":
                kind_, args_, extra_kw_ = "transport", (sc_, start_, cfg_), core._exit_prefix_kwargs()
            else:
                kind_, args_, extra_kw_ = "return", (sc_, start_, options.return_to_pose, cfg_), {}
            kwargs_ = dict(geometry=geometry, travel_config=travel_)
            if core.jobs is not None:
                worker_kw_ = {**kwargs_, **extra_kw_}  # the worker's planner is unbound: the exit prefix goes along
                payload_ = {"kind": kind_, "args": args_, "kwargs": {**worker_kw_, "deadline": time.monotonic() + 20.0},
                            "retry_on": (), "tight_args": args_, "tight_kwargs": worker_kw_}
                core._request("scene_replan", "leg", payload_, t, box.rear, core._applied_now(), ctx={})
                return
            planner_ = hooks.plan_transport_leg if kind_ == "transport" else hooks.plan_return_leg
            result_ = planner_(*args_, **kwargs_)
            core.scene_mail = ("ok", result_) if result_.success else ("ok_fail", result_.status)

        def poll():
            mail_, core.scene_mail = core.scene_mail, None
            return mail_

        def candidate(result):
            tracker_ = RearAxlePathTracker(result.poses, result.directions, result.curvatures_inv_m,
                                           trackers[ms.phase].config)
            return core._scene_preview(tracker_, t, box.rear, dt)

        def install(result):
            paths[ms.phase] = result
            state["paths"][ms.phase] = hooks.path_record(result)
            hooks.write_paths()
            config_ = (core.scene_escape or {}).get("config", trackers[ms.phase].config)
            core.scene_escape = None  # consumed: a later install keeps its own leg's settings (Codex impl round 1 P2-4)
            trackers[ms.phase] = RearAxlePathTracker(result.poses, result.directions, result.curvatures_inv_m, config_)
            state.setdefault("paths_replaced", []).append(
                {"leg": ms.phase, "time_s": float(t), "by": "scene_replan", "path": state["paths"].get(ms.phase)})
            if ms.phase == "transport" and slam is not None:
                core.arm_docking()
            ms.phase_started = t
            core._refresh_for_new_leg(t, box.rear)

        def escape_plan(extra, clearance_m):
            if obstacle is None:
                return None
            _, _, _, (rects_, footprint_, bounds_) = core.escape_leg(ms.phase)
            checker_ = FootprintCollisionChecker(list(rects_) + list(extra), footprint_, bounds_)
            layer_ = obstacle["layer"]
            loaded_ = bool(hooks.loaded())
            pose_ = tuple(float(v) for v in box.rear)
            pconf_ = layer_.permission.config
            speed_ = options.backoff_speed_mps
            stop_len_ = pconf_.stopping.distance_m(speed_) + pconf_.step_m
            psi_ = float(layer_.error_table.at(layer_.grid_config.occupied_max_age_s)[1])
            clear_ = float(clearance_m) if clearance_m is not None else float(
                (options.travel_config or options.planner_config).clearance_m)
            cell_ = float(layer_.snapshot.resolution_m) if layer_.snapshot is not None else 0.05
            kappa_now_ = float(hooks.held_kappa())
            candidates_ = esc.evaluate(
                pose_,
                sweep_free=lambda poses_, d_: esc.rectangles_sweep_free(checker_, poses_),
                path_blocked=lambda poses_, d_, x_, r_: layer_.check_candidate(
                    poses_, current_pose=pose_, direction=d_, loaded=loaded_, extra_margin_m=x_, extra_ramp_m=r_).blocked,
                arc_blocked=lambda k_, d_: layer_.check_arc_candidate(
                    current_pose=pose_, curvature_inv_m=k_, direction=d_, loaded=loaded_, cap_mps=speed_).blocked,
                end_clearance=lambda region_: (lambda v_: (min(v_) >= 0.0, min(v_)))(
                    [esc.clearance_m(checker_.free, p_) for p_ in region_]),
                kappa_now=kappa_now_, stop_length_m=stop_len_, planner_clearance_m=clear_, psi_rad=psi_, cell_m=cell_)
            leg_dir_ = trackers[ms.phase].leg_ahead()[1]
            choice_ = esc.choose(candidates_, int(leg_dir_))
            state.setdefault("scene_escape_candidates", []).append(
                {"time_s": float(t), "candidates": [c_.record() for c_ in candidates_],
                 "chosen": None if choice_ is None else choice_.record()})
            if choice_ is None:
                return None
            poses_ = esc.straight(pose_, choice_.direction, choice_.length_m)
            permit_ = layer_.candidate_permit(t, poses_, current_pose=pose_, direction=int(choice_.direction),
                                              loaded=loaded_, curvature_inv_m=0.0,
                                              measured_curvature_inv_m=kappa_now_, cap_mps=speed_)
            state["scene_escape_candidates"][-1]["permit"] = permit_
            if not permit_["ok"]:
                return None  # the prefix itself must start (Codex counterexample: an end clear, a start held at 0)
            return poses_, int(choice_.direction), int(leg_dir_)

        def escape_install(chosen):
            poses_, direction_, leg_dir_ = chosen
            config0_ = trackers[ms.phase].config
            n_ = len(poses_)
            trackers[ms.phase] = RearAxlePathTracker(
                poses_, np.full(n_, direction_, dtype=np.int8), np.zeros(n_),
                replace(config0_, cruise_speed_mps=options.backoff_speed_mps,
                        position_tolerance_m=esc.END_POSITION_TOL_M, yaw_tolerance_rad=esc.END_YAW_TOL_RAD,
                        overshoot_tolerance_m=esc.END_POSITION_TOL_M))
            # The existing escape's ownership, so the permission acts on it every tick (Codex round 3 P2-2).
            obstacle["backoff"] = {"phase": ms.phase, "config": config0_, "time_s": float(t), "leg_dir": leg_dir_,
                                   "escape": True, "scene": True}
            core.scene_escape = {"status": "running", "config": config0_, "zero_since": None}
            ms.phase_started = t
            core._refresh_for_new_leg(t, box.rear)

        def escape_state():
            return (core.scene_escape or {}).get("status", "done")

        def escape_clear():
            if obstacle is not None and obstacle.get("backoff", {}).get("scene"):
                obstacle.pop("backoff", None)  # the ownership goes; install restores the tracker settings
            if core.scene_escape is not None:
                core.scene_escape["status"] = "cleared"

        def wheel_targets(kappa_):
            return ackermann_command(0.0, float(kappa_), options.drive_geometry).steering_rad

        def steering_error(target_kappa):
            # each front wheel against its own Ackermann angle (Codex impl round 1 P3: the mean held curvature
            # hides +0.08 / -0.08 rad)
            actual_ = [float(v) for v in hooks.steering_actual()]
            return max(abs(a_ - b_) for a_, b_ in zip(actual_, wheel_targets(target_kappa)))

        def steer_gap(a_, b_):
            return max(abs(x_ - y_) for x_, y_ in zip(wheel_targets(a_), wheel_targets(b_)))

        def preview():
            return core._scene_preview(copy.deepcopy(trackers[ms.phase]), t, box.rear, dt)

        return SimpleNamespace(stopped=stopped, busy=busy, release=release, submit=submit, poll=poll,
                               candidate=candidate, install=install, escape_plan=escape_plan,
                               escape_install=escape_install, escape_state=escape_state, escape_clear=escape_clear,
                               steering_error=steering_error, steer_gap=steer_gap, preview=preview, box=box)

    def scene_replan_tick(self, t: float, rear, signed_speed: float, dt: float) -> SimpleNamespace:
        """The device's tick ahead of the follower (plan 2.1): owned or not, what the wheels and steering get, the
        control pose after a release; a request that ends raises its reason."""
        ms, trackers, slam = self.ms, self.trackers, self.slam
        self.scene_raised = False
        if not self.scene_replan.active():
            return SimpleNamespace(owned=False, rear=rear)
        ports = self._scene_ports(t, rear, signed_speed, dt)
        out_ = self.scene_replan.step(t, ports)
        self.hooks.require(out_.end is None, out_.end or "")
        result = SimpleNamespace(owned=out_.owned, output=out_.output, rear=ports.box.rear, requested_speed=0.0,
                                 curvature=0.0)
        if out_.output == "presteer":
            result.curvature = float(out_.steer)  # the target is the first command's curvature
        elif out_.output == "escape" and self.scene_escape is not None:
            tracker_ = trackers[ms.phase]
            tracking = tracker_.update(ports.box.rear, signed_speed, dt)
            ms.last_tracking = tracking
            standing_ = ports.stopped()
            if tracking.status == "arrived" or (
                    standing_ and (tracking.status == "failed" or tracker_.remaining_to_goal_m() <= 1e-6)):
                self.scene_escape["status"] = "done"
            else:
                allowed_ = self.last_permission[0] if self.last_permission is not None else None
                if allowed_ is not None and allowed_ <= 0.0 and standing_:
                    self.scene_escape["zero_since"] = self.scene_escape.get("zero_since") or float(t)
                    if t - self.scene_escape["zero_since"] >= 1.0:
                        self.scene_escape["status"] = "stopped"
                else:
                    self.scene_escape["zero_since"] = None
                result.requested_speed, result.curvature = tracking.speed_mps, tracking.curvature_inv_m
        return result

    def end_reset(self, t: float) -> dict:
        """A live mission's end (D1 steps 3 and 8): the pocket check closed as the insertion's end closes it, then
        the mission-owned state back to the baseline. Returns the mission's records (for its own record)."""
        pocket, obstacle, state, slam = self.pocket, self.obstacle, self.state, self.slam
        self.capture_recovery_context()  # plan D14: before the mission-owned state goes
        if slam is not None:
            # The mission's release reservation goes with it (Codex D6 P2-3: its 10 s limit failed every tick after);
            # a hold left with the forks free is released for the end (D1 step 4), now or once standing.
            slam["pending_release"] = slam["release_wait_from"] = None
            if slam["tracker"].mode == "holding" and self.cargo.release_allowed():  # free or carrying (plan D5a)
                if slam["stop_now"]:
                    self.end_release(t, deferred=False)
                else:
                    mission = self.commands.mission
                    pallet_held, destination_held = self._held_estimates()
                    self.end_release_due = {"pallet_id": None if mission is None else mission.pallet_id,
                                            "destination_id": None if mission is None else mission.destination_id,
                                            "pallet_held": pallet_held, "destination_held": destination_held}
        # The list takes the mission's last estimates (a docking match, a release's re-expression) now, and the
        # ended mission moves it no more: the scenarios it read from are dropped below (Codex stage 2 round 2 P2).
        self.refresh_targets()
        self.commands.adopted_id = None
        if pocket.get("check") is not None:
            if obstacle is not None:
                obstacle["layer"].exempt = None
                obstacle["layer"].depth_support = None
            state.setdefault("pocket_check", {}).update(pocket["check"].summary(), frames_read=pocket["frames_read"])
            pocket["check"] = None
        began = time.perf_counter()
        records = self.baseline.restore(self) if self.baseline is not None else {}
        self.restore_wall_s = time.perf_counter() - began  # plan U29: holds the records' handover (top-level copies)
        self.scene_replan.reset()  # a mission's budget (never a tracker's or a pause's)
        self.scene_mail, self.scene_raised, self.scene_escape, self.scene_correction = None, False, None, {}
        return records

    def hold_slam(self, t: float, *, held_frame_pallet: bool) -> None:
        """Hold the SLAM correction, keeping which estimates the hold began with: those were made under earlier
        corrections and stay map estimates at an end release; what the hold finds itself -- the docking match's
        destination, a capture's pallet -- is in its frame. ``held_frame_pallet``: the pallet estimate in hand was
        made with this very correction (the approach's hold begins at the accepted capture)."""
        slam, obstacle, pocket = self.slam, self.obstacle, self.pocket
        slam["tracker"].hold(odom_from_base=self.hooks.slam_odom_base())
        self.hold_basis = {
            "pallet": (_HELD, _HELD) if held_frame_pallet else (
                pocket.get("planning_pickup"), None if obstacle is None else obstacle.get("pickup_estimate")),
            "destination": self._destination_estimate()}

    def _destination_estimate(self):
        """The destination the core steers to now (a docking match's, a release's re-expression, or the prior)."""
        slam = self.slam
        scenario_ = slam.get("return_scenario", slam.get("transport_scenario", getattr(self.options, "scenario", None)))
        return getattr(scenario_, "destination", None)

    def _held_estimates(self) -> tuple[bool, bool]:
        """Whether the pallet estimate and the destination in hand were found under the current hold."""
        obstacle, pocket = self.obstacle, self.pocket
        destination = self._destination_estimate()
        # Without a recorded basis: the pallet estimate in hand is the hold's, the destination a map goal.
        basis = self.hold_basis or {"pallet": (_HELD, _HELD), "destination": destination}
        planning, estimate = pocket.get("planning_pickup"), None if obstacle is None else obstacle.get("pickup_estimate")
        pallet = ((planning is not None and planning is not basis["pallet"][0])
                  or (estimate is not None and estimate is not basis["pallet"][1]))
        return pallet, destination is not None and destination is not basis["destination"]

    def end_release(self, t: float, *, deferred: bool) -> None:
        """The end release of a SLAM hold (plan D1 step 4): the received correction applied, the hold-frame pallet
        estimate carried by the frame change once -- the destinations are map goals and stay -- the grid
        re-projected and the release recorded. No plan, no path, no re-observation (U28). Deferred (the truck was
        moving at the end), the ended mission's pallet target is carried instead, its correction version raised."""
        slam, hooks, obstacle, pocket = self.slam, self.hooks, self.obstacle, self.pocket
        before = hooks.slam_rear(t)
        jump_m, jump_rad = self.release_slam(hooks.slam_odom_base())
        after = hooks.slam_rear(t)
        frame_change = compose(tuple(float(v) for v in after), invert(tuple(float(v) for v in before)))

        def carried(site):
            x_, y_, yaw_ = compose(frame_change, (float(site.x_m), float(site.y_m), float(site.yaw_rad)))
            return replace(site, x_m=x_, y_m=y_, yaw_rad=yaw_)

        if deferred:  # the ended mission's own targets, in the list (refresh_targets no longer runs for it)
            due, tb = self.end_release_due or {}, self.targets
            pid, did = due.get("pallet_id"), due.get("destination_id")
            if due.get("destination_held") and did in tb.destinations:
                # the docking hold: the docked destination and the pallet delivered there
                old = tb.destinations[did]
                tb.update_destination(did, carried(old))
                if pid in tb.pallets and tb.pallets[pid].state == "delivered" and tb.pallets[pid].pose == old:
                    tb.update_pose(pid, carried(old))
            elif due.get("pallet_held") and pid in tb.pallets and tb.pallets[pid].pose is not None:
                tb.update_pose(pid, carried(tb.pallets[pid].pose))  # the near capture's hold: its pallet
            tb.set_correction_version(self.slam_releases)
        else:
            # Only what this hold found is in its frame (Codex D6 rounds 4-5): the docked destination, carried as
            # slam_release carries it (the delivered pallet follows it in the list), or a capture's pallet.
            pallet_held, destination_held = self._held_estimates()
            if destination_held:
                scenario_ = slam.get("return_scenario", slam.get("transport_scenario", self.options.scenario))
                slam["return_scenario"] = replace(scenario_, destination=carried(scenario_.destination))
            if pallet_held:
                if obstacle is not None and obstacle.get("pickup_estimate") is not None:
                    obstacle["pickup_estimate"] = carried(obstacle["pickup_estimate"])
                if pocket.get("planning_pickup") is not None:
                    pocket["planning_pickup"] = carried(pocket["planning_pickup"])
            mission, tb = self.commands.mission, self.targets
            if self.ms.phase in ("recover_lower", "recover_withdraw", "clear_out") and mission is not None:
                # Plan D4b item 4: the recovery put the pallet back from the hold's estimate and the list no longer
                # follows the core there -- carried with what it was put back from (the truck's gap to it kept).
                entry = tb.pallets.get(mission.pallet_id)
                held = destination_held if entry is not None and entry.state == "delivered" else pallet_held
                if held and entry is not None and entry.pose is not None:
                    tb.update_pose(mission.pallet_id, carried(entry.pose))
        if obstacle is not None and slam["tracker"].applied is not None:
            self._reproject(t, tuple(float(v) for v in slam["tracker"].applied[0]), after, False)
        slam["holds"].append({"phase": self.ms.phase, "time_s": float(t), "event": "release", "reason": "mission_end",
                              "deferred": deferred, "jump_m": jump_m, "jump_rad": jump_rad,
                              "frame_change": [float(v) for v in frame_change]})
        hooks.set_rear(after)
        if slam.get("control"):
            slam["control"][-1][1:4] = [float(v) for v in after]
        self.end_release_due = None

    def start_mission(self, t: float, plan, inputs: dict, ladder: dict, ready=None) -> None:
        """The adoption (D2 start processing): the mission-owned state back to the session's baseline (D0), the
        mission's inputs into the options, the observation leg and its records, the runner's own start
        (``hooks.live_mission_start``: its locals, trackers, prior zone, docking reference), then running."""
        ms, state, options = self.ms, self.state, self.options
        # The last mission's end restored the baseline (end_reset); what came since -- this mission's preparation --
        # stays with this mission; the rest (the follower's counters, the near and SLAM run keys) starts clean.
        for owner, kept in self.baseline.restore(self).items():
            getattr(self, owner).update(kept)
        options.scenario = inputs["scenario"]
        options.return_to_pose = options.home_rear = inputs["wait_pose"]
        options.observation_waypoints[:] = inputs["waypoints"]
        options.pickup_bounds, options.observe_bounds = inputs["pickup_bounds"], None
        options.pickup_zone = inputs["pickup_zone"]
        ms.next_candidate_index = ladder["selected"] + 1
        ms.lift_command = float(self.hooks.lift_position())
        state["observation_candidates"] = list(ladder["candidates"])
        state["observation_waypoint_selected"] = list(ladder["candidates"][-1]["pose"])
        state["planning_status"] = plan.status
        self.paths.clear()
        self.paths["observe"] = plan
        getattr(self.trackers, "close", lambda: None)()  # the last mission's leg switches into the record first
        self.trackers.clear()
        self.trackers["observe"] = self.observe_tracker(plan)
        self.apply_tracker_profile()
        self.record_tracker_configs()
        self.hooks.live_mission_start(t, inputs, plan, ready)
        state["transitions"].append({"from": ms.phase, "to": "observe", "time_s": t,
                                     "mission_id": self.commands.mission_id})
        ms.phase, ms.phase_started = "observe", t
        state["phase"] = ms.phase
        self.commands.adopt()
        if self.record_start_digest:
            self.start_digest = start_digest(self)

    def start_loaded_mission(self, t: float, plan, inputs: dict, ready=None) -> None:
        """A loaded submit's adoption (plan D5 6c item 4): the baseline back, the mission's inputs into the options,
        the delivery's legs (transport, withdraw, return) and their followers built as the mission plan's are, the
        runner's own start, then transport entered directly -- not through ``transition()``: a transport entry books
        a SLAM release, and the cancel that left the pallet on the forks already released (Codex 6c P2-1). The cargo
        stays as it is (the session's). The docking is armed on the planned leg, and the truck stands in its start
        wait (``loaded_start_wait``) until the lift, the first live leg and the docking are ready."""
        ms, state, options, paths, trackers = self.ms, self.state, self.options, self.paths, self.trackers
        for owner, kept in self.baseline.restore(self).items():
            getattr(self, owner).update(kept)
        options.scenario = inputs["scenario"]
        options.return_to_pose = options.home_rear = inputs["wait_pose"]
        options.observation_waypoints[:] = inputs["waypoints"]
        options.pickup_bounds, options.observe_bounds = inputs["pickup_bounds"], None
        options.pickup_zone = inputs["pickup_zone"]
        ms.lift_command = float(self.hooks.lift_position())  # the ramp starts from the measured lift (item 5)
        state["planning_status"] = plan.status
        paths.clear()
        paths.update({name: getattr(plan, name) for name in ("transport", "withdraw", "return_home")
                      if getattr(plan, name) is not None})
        getattr(trackers, "close", lambda: None)()  # the last mission's leg switches into the record first
        trackers.clear()
        trackers.update({name: RearAxlePathTracker(path.poses, path.directions, path.curvatures_inv_m,
                                                   self.tracker_config(name)) for name, path in paths.items()})
        self.apply_tracker_profile()
        self.record_tracker_configs()
        state["paths"] = {name: self.hooks.path_record(path) for name, path in paths.items()}
        self.hooks.write_paths()
        self.hooks.live_mission_start(t, inputs, plan, ready)
        state["transitions"].append({"from": ms.phase, "to": "transport", "time_s": t,
                                     "mission_id": self.commands.mission_id, "loaded": True})
        ms.phase, ms.phase_started = "transport", t
        state["phase"] = ms.phase
        if self.slam is not None:
            self.arm_docking()
        self.loaded_wait = {"since": float(t)}
        state.setdefault("loaded_lift_wait", []).append({"start_s": float(t)})
        self.commands.adopt()
        if self.record_start_digest:
            self.start_digest = start_digest(self)

    LOADED_START_WAIT_S = 60.0

    def lift_ramps_in_plan_wait(self) -> bool:
        """Whether the lift ramp runs on a tick standing for a plan (plan D5 6c item 5): in a loaded start wait, so the
        lift reaches the carry height while the first live leg is planned (Codex 6c impl round 1 P2)."""
        return self.loaded_wait is not None

    def loaded_start_wait(self, t: float) -> bool:
        """A loaded mission's start wait (plan D5 6c item 5), each control tick: True while the truck stands for
        it -- the runner then skips the follower, the arrival and the cusp and stall counts, and commands no wheel
        speed; the lift ramp, the live leg and the permission watch run. It ends when the measured lift is at the
        carry target less the loaded sag (within the loaded tolerance), the first live leg is taken and the docking
        armed on it; the cargo is checked again then (item 4a), and the tracking clock starts there. A live leg with
        no delivery straight, or 60 s of waiting outside the operator's pauses (a plan's wait counts), fails the
        mission. Outside a running mission it only stands."""
        wait, cmds = self.loaded_wait, self.commands
        if wait is None:
            return False
        if cmds.status != "running":
            return True
        slam, obstacle = self.slam, self.obstacle
        docking = (slam or {}).get("docking") or {}
        if docking.get("status") == "no_final_straight":
            cmds.terminate("failed", t, "prepare:loaded:no_final_straight")
            return True
        sag, tolerance = self.loaded_lift_band if self.loaded_lift_band is not None else (0.0, 0.01)
        left = []
        if abs(float(self.hooks.lift_position()) - (self.settings["lift_target_m"] - sag)) > tolerance:
            left.append("lift")
        taken = (bool((obstacle or {}).get("live_replanned", {}).get("transport"))
                 and (self.pending is None or self.pending.site != "live_leg"))
        if not taken:
            left.append("leg")
        if slam is not None and docking.get("status") != "armed":
            left.append("docking")
        if not left:
            if not self.hooks.loaded_cargo_ok():
                cmds.terminate("failed", t, "prepare:loaded:cargo_changed")
                return True
            self.loaded_wait = None
            self.ms.phase_started = t
            self.state["loaded_lift_wait"][-1]["end_s"] = float(t)
            return False
        paused = sum(max(0.0, min(t, end if end is not None else t) - max(wait["since"], start))
                     for start, end in cmds.pauses)
        if t - wait["since"] - paused > self.LOADED_START_WAIT_S:
            cmds.terminate("failed", t, "prepare:loaded:start_wait:" + ",".join(left))
        return True

    def observe_bounds(self):
        """The bounds of the observation travel (persistent console plan D2): a live run plans it in the hall
        (``options.observe_bounds`` None) wherever the operator left the truck; a batch run's options have none, so
        its observation keeps the pickup bounds. On a bottom-deck side leg, the side stage's own bounds (plan 2026-10-19
        task 5): every check of that leg -- its replans and the cusp check -- sees the bounds it was planned in."""
        side_ = getattr(self, "bottom_deck_bounds", None)
        if side_ is not None and self.bottom_deck_leg_active():
            return side_
        options = self.options
        return options.observe_bounds if hasattr(options, "observe_bounds") else options.pickup_bounds

    def abandon_jobs(self, t: float) -> None:
        """A live mission's end (docs/plans/2026-10-18-persistent-live-console.md D1 step 2): the plan waited for,
        the capture and the observation belong to the mission that ended. Each is dropped without ``_drop``'s
        undo for a resume (a release replan asked again, a live leg planned again) and without counting it."""
        p = self.pending
        if p is not None:
            self.jobs.discard(p.job)
            self.commands.end_wait(t)
            self.state["async_plans"][-1].update(finished_s=float(t), dropped="abandoned")
            self.pending = None
        self.release_replan_needed = None
        self.observe_failed_needed = None
        self.fresh_depth_wait = None
        self.capture_abandon()
        self.observation_drop(t, "mission_end")
        self.far_leg_recaptures = 0
        far_end = getattr(self.hooks, "far_recognition_mission_end", None)
        if far_end is not None:  # plan 2026-10-22-live-far-recognition task 7: the mission's far summary, every end
            far_end(t)
        self.capture_acquired_s = None

    def hold_count(self) -> int:
        """SLAM hold episodes so far: the near-field tracker's frame_version."""
        slam = self.slam
        return sum(1 for h in slam["holds"] if h.get("event") == "hold") if slam is not None else 0

    def near_tick(self, now_s: float, rear_stamp_: float, rear_now, requested_mps: float, phase_: str,
                  curvature_cmd: float, arrived_: bool = False) -> tuple[bool, bool, bool]:
        """Plan D8 S4a ④⑤ and S4a-2: one approach/insert tick of the near-field stop, decided
        before the tick's arrival. Returns (hold, released, arrive): hold -- command zero,
        steering held, arrival deferred; released -- a stop just ended standing (restart the
        slew); arrive -- the carriage-corner insertion end, standing with nothing waiting:
        the caller turns it into the insertion's arrival.

        Reasons by priority (near_tracking.decide_stop): terminal (failed, mismatch,
        unbounded, stuck -- latched, ending the run once the stop detector sees the truck
        stand) > the insertion end (latched) > waiting (lost, budget, gap_wait,
        carriage_uncertain, an open correction point -- released only standing, once none
        is left). S4b: a correction point re-draws the docking straights once standing."""
        near_tracking = self.options.near_tracking  # a runner module-level name
        require = self.hooks.require  # a runner module-level name
        options = self.options
        hold_count = self.hold_count
        near = self.near
        near_install = self.near_install
        near_protection_prediction = self.near_protection_prediction
        near_redraw = self.near_redraw
        slam = self.slam
        start_pickup_retry = self.start_pickup_retry
        state = self.state
        steering_command = self.hooks.steering_command()
        tracker_ = near["tracker"]
        revision_used = near["path_revision"]  # this tick's follower update ran before
        require(slam["tracker"].mode == "holding" and hold_count() == near["episode"],
                "near_field_frame_released")  # S4a: the near-capture hold lasts through insert
        episode = near["episode"]
        near["history"].add(rear_stamp_, rear_now, episode)
        base_now = near["history"].base_at(rear_stamp_, episode)
        bounds_, terms_ = near["bounds"], near["terms"]
        latest_ = tracker_.latest
        widths_ = (latest_.observation.left.width_m, latest_.observation.right.width_m)
        if not tracker_.armed:
            face_x = near_tracking.face_min_x_m(latest_.held, widths_, base_now)
            if near_tracking.section_entered(face_x, near["fork_tip_x_m"]):
                tracker_.arm(now_s)
                near["events"].append({"time_s": now_s, "event": "armed", "face_x_m": face_x,
                                       "lost": tracker_.status(now_s).lost})
        status_ = tracker_.status(now_s)
        pixel_s = latest_.stamp_s - bounds_.max_latency_s
        age = now_s - pixel_s
        path = near["history"].path_length_since(pixel_s, episode)
        tau = near_tracking.result_wait_s(now_s, near["last_stamp"], bounds_.render_period_s, bounds_.read_delay_s)
        vbar = near["commands"].vbar(now_s, terms_.delta_v_mps, pending_mps=requested_mps)
        budget = near_tracking.travel_budget(path, age, tau, vbar, terms_)
        terminal = ("lost" if status_.failed else "mismatch" if near.get("mismatch")
                    else "unbounded" if near.get("unbounded") else None)
        waits = []
        if tracker_.armed:
            if status_.lost:
                waits.append("lost_wait")
            if not budget.ok:
                waits.append(f"budget:{budget.reason}")
        # Plan D8 S4a-2: the blade-wall gaps and the carriage corners, from the protection's
        # start (within W of the estimated face, or the insertion) on.
        face = near_tracking.carried_face(latest_.held, widths_, base_now)
        tip_x, rear_x = near["fork_tip_x_m"], -abs(options.rear_axle_offset_m)
        face_distance = near_tracking.face_distance_m(face, tip_x)
        if (options.near_drop_frames_from_m is not None and near.get("drop_latch") is None
                and face_distance <= options.near_drop_frames_from_m):
            # Plan D8 S4c-1: from here no frame reaches the tracker (feed_near_frame).
            near["drop_latch"] = {"time_s": now_s, "face_distance_m": face_distance, "latest_stamp_s": latest_.stamp_s}
            near["events"].append({"time_s": now_s, "event": "drop_latch", **near["drop_latch"]})
        wb_, ht_ = options.drive_geometry.wheelbase_m, options.drive_geometry.track_m / 2
        # Each front wheel's curvature, measured and as the held steering command (a stop
        # keeps steering_command -- Codex S4a-2 code review P2), and this tick's request.
        kappa_b = abs(curvature_cmd)
        for steer_ in (self.hooks.steering_position(), steering_command):  # the measured steering, the held command
            kappa_b = max(kappa_b, abs(math.tan(steer_[0]) / (wb_ + math.tan(steer_[0]) * ht_)),
                          abs(math.tan(steer_[1]) / (wb_ - math.tan(steer_[1]) * ht_)))
        s_term = terms_.at(terms_.delta_s_m, near_tracking.STOP_LATENCY_S)
        sweep = near_tracking.stop_distance_m(vbar) + (s_term if s_term is not None else math.inf)
        odometry_ = bounds_.odometry(age)
        eps_yaw = latest_.bound["yaw_rad"]
        protection = {}
        insert_end_now = False
        if near["protect"] is None and (phase_ == "insert" or face_distance <= near_tracking.PROTECT_W_M):
            rho_face = max(math.hypot(face[s_][w_][0] - rear_x, face[s_][w_][1])
                           for s_ in ("left", "right") for w_ in ("outer", "inner"))
            along_e = (latest_.bound["along_m"] + odometry_[0] + rho_face * odometry_[1]
                       if odometry_ is not None else math.inf)
            k_tip = near_tracking.corner_sweep_factor((tip_x, near["blade_y_max_m"]), face, eps_yaw, kappa_b, rear_x)
            near["protect"] = {"time_s": now_s, "phase": phase_, "face_distance_m": face_distance,
                               "along_error_m": along_e, "tip_sweep_m": k_tip * sweep, "kappa_b": kappa_b}
            near["events"].append({"time_s": now_s, "event": "protect", **near["protect"]})
            if odometry_ is None or not math.isfinite(sweep):
                terminal = terminal or "unbounded"  # no bound: not a measured shortfall
            elif face_distance - along_e <= k_tip * sweep:
                terminal = terminal or "stuck"
        if near["protect"] is not None:
            if odometry_ is None or not math.isfinite(sweep):
                terminal = terminal or "unbounded"
            else:
                gaps_ = near_tracking.blade_gaps(face, near["blades"], (0.0, near["carriage_limit_m"]), rear_x)
                lat_sweep = near_tracking.lateral_sweep_m(kappa_b, sweep, tip_x - rear_x, near["blade_y_max_m"])
                margins_ = [p_["gap_m"] - near_tracking.gap_threshold_m(p_, latest_.bound, odometry_, gaps_["rho_m"],
                                                                     face, lat_sweep) for p_ in gaps_["points"]]
                worst_ = int(np.argmin(margins_))
                fresh = age <= bounds_.max_latency_s + bounds_.read_delay_s + near_tracking.TICK_S + 1e-9
                stale_now = set()  # waits raised on an observation that is not fresh
                if margins_[worst_] < 0:
                    if fresh:
                        terminal = terminal or "stuck"  # confirmed on a new observation
                    else:
                        stale_now.add("gap_wait")
                ghat = near_tracking.carriage_gaps(face, near["corners"])
                ks = [near_tracking.corner_sweep_factor(c_, face, eps_yaw, kappa_b, rear_x) for c_ in near["corners"]]
                sigmas = [near_tracking.carriage_uncertainty_m(latest_.bound, odometry_, c_, face, rear_x, vbar)
                          for c_ in near["corners"]]
                inside = near_tracking.entered(face, tip_x)
                certain = max(sigmas) < near_tracking.INSERT_RESERVE_M
                if phase_ == "insert" and min(g_ - k_ * sweep for g_, k_ in zip(ghat, ks, strict=True)) <= (
                        near_tracking.INSERT_RESERVE_M):
                    if fresh and certain:
                        insert_end_now = True  # latched only on a fresh, certain observation
                    else:
                        stale_now.add("insert_end_unconfirmed")  # stand; the next result decides
                if inside and not certain:
                    stale_now.add("carriage_uncertain")
                # A wait raised on old evidence holds until a new accepted observation, which
                # then decides it afresh (Codex S4a-2 code review P2).
                held_waits = {r_: st_ for r_, st_ in near["stale_waits"].items() if st_ >= latest_.stamp_s}
                held_waits.update({r_: latest_.stamp_s for r_ in stale_now})
                near["stale_waits"] = held_waits
                waits += sorted(held_waits)
                protection = {
                    "inside": inside, "fresh": fresh, "min_margin_m": margins_[worst_],
                    "worst": {k_: v_ for k_, v_ in gaps_["points"][worst_].items()}, "rho_m": gaps_["rho_m"],
                    "lateral_sweep_m": lat_sweep, "corner_gaps_m": ghat, "corner_sweep_k": ks,
                    "corner_sigma_m": sigmas, "face_distance_m": face_distance,
                }
        standing = bool(slam["stop_now"])
        # Plan D8 S4b: the correction points (approach, before entry), a waiting reason while
        # open; decided standing on an observation whose pixel time is after t_s.
        corr_prev = near["correction"]
        if near["stop_state"].terminal:
            # A latched terminal stop only stands and records until it ends the run: no correction
            # is decided, recorded or installed meanwhile (Codex S4c-1 review P2 -- the terminal
            # hold would otherwise re-decide a refused correction).
            corr, corr_act = corr_prev, {"wait": None, "decide": False, "refuse": None, "event": None}
        else:
            corr, corr_act = near_tracking.correction_update(
                corr_prev, now_s, phase=phase_, entered=near_tracking.entered(face, tip_x),
                face_distance_m=face_distance, standing=standing, pixel_s=pixel_s, source=latest_.source,
                frame_version=latest_.frame_version, episode=episode,
            )
        if corr_act["event"] == "trigger":
            near["corrections"].append({"point_m": corr.point_m, "episode": episode, "trigger_s": now_s,
                                        "face_distance_m": face_distance,
                                        "standing_s": [], "moved_s": [], "accepted": None})
            near["events"].append({"time_s": now_s, "event": "correction_trigger", "point_m": corr.point_m,
                                   "face_distance_m": face_distance})
        corr_record = near["corrections"][-1] if near["corrections"] else None
        if corr.standing_s is not None and corr.standing_s != corr_prev.standing_s and corr_act["event"] != "closed":
            corr_record["standing_s"].append(now_s)
        if corr_prev.standing_s is not None and corr.standing_s is None and corr_prev.installed_s is None:
            corr_record["moved_s"].append(now_s)  # the stop detector saw motion again: t_s restarts
        if corr_act["refuse"] is not None:
            corr_record.update(accepted=False, decision_s=now_s, refused=corr_act["refuse"])
            terminal = terminal or "replan_rejected"
        elif corr_act["decide"] and terminal is None:
            plans_, record_, target_ = near_redraw(latest_, rear_now)
            if plans_ is not None and abs(corr.point_m - near_tracking.PREDICT_POINT_M) < 1e-9:
                # Plan D8 S4 near field, protection prediction 3판: the last correction screens the
                # protection's first fresh margin; short with a retry left, it is refused into the retry.
                record_["protection_prediction"] = pred_ = near_protection_prediction(
                    plans_["approach"], latest_, widths_, rear_now, face_distance)
                if pred_["disposition"] == "refused":
                    record_["refused"] = "protection_predicted"
                    plans_ = None
            corr_record.update(
                decision_s=now_s, observation={"stamp_s": latest_.stamp_s, "pixel_s": pixel_s, "source": latest_.source,
                                               "frame_version": latest_.frame_version, "aligned_s": latest_.aligned_s},
                redraw=record_, accepted=plans_ is not None,
            )
            if plans_ is None:
                terminal = "replan_rejected"
            else:
                corr_record["revision"] = near_install(now_s, plans_, target_, corr.point_m, rear_now)["revision"]
                corr = near_tracking.correction_installed(corr, now_s)
        if corr_act["event"] == "closed" and corr_record is not None:
            corr_record["closed_s"] = now_s
        near["correction"] = corr
        if corr_act["wait"] is not None:
            waits.append(corr_act["wait"])
        if phase_ == "approach" and arrived_ and near_tracking.corrections_missed(corr) and not near["stop_state"].terminal:
            terminal = terminal or "correction_missed"
        previous = near["stop_state"]
        near["stop_state"], action = near_tracking.decide_stop(
            previous, now_s, terminal=terminal, insert_end=insert_end_now, waits=tuple(waits), standing=standing
        )
        current = near["stop_state"]
        if action["changed"] and current.reason is not None:
            near["events"].append({"time_s": now_s, "event": "stop", "reason": current.reason})
            if previous.reason is None:  # a new stop: what the stopping model was given (judgement (vii))
                near["stop_events"].append({
                    "start_s": now_s, "episode": episode, "reason": current.reason, "vbar_mps": vbar,
                    "d_stop_m": near_tracking.stop_distance_m(vbar), "sweep_m": sweep, "kappa_b": kappa_b,
                    "corner_sweep_k": protection.get("corner_sweep_k"), "lateral_sweep_m": protection.get("lateral_sweep_m"),
                    "standing_s": None,
                })
        if ((current.reason is not None or action["released"]) and standing and near["stop_events"]
                and near["stop_events"][-1]["standing_s"] is None):
            near["stop_events"][-1]["standing_s"] = now_s  # also when this tick releases it
        if action["released"]:
            near["events"].append({"time_s": now_s, "event": "release", "reason": previous.reason,
                                   "stopped_s": now_s - previous.since_s})
            last_corr = near["corrections"][-1] if near["corrections"] else None
            if last_corr is not None and last_corr.get("revision") is not None and "release_s" not in last_corr:
                last_corr["release_s"] = now_s  # the stop the correction held ends here
        if action["arrive"]:
            near["arrival"] = {"reason": "insert_end", "time_s": now_s, **protection}
            near["events"].append({"time_s": now_s, "event": "insert_end_arrival"})
        near["ticks"].append({
            "t": now_s, "rear_stamp_s": rear_stamp_, "base": list(base_now), "armed": tracker_.armed,
            "age_s": age, "lost": status_.lost, "failed": status_.failed, "pixel_s": pixel_s,
            "path_m": path, "len_term_m": budget.len_term_m, "d_m": budget.d_m, "tau_s": tau,
            "horizon_s": budget.horizon_s, "vbar_mps": vbar, "s_term_m": budget.s_term_m,
            "travel_m": budget.travel_m, "total_m": budget.total_m, "budget_reason": budget.reason,
            "terminal": terminal, "waits": list(waits), "insert_end": current.insert_end, "reason": current.reason,
            "standing": standing, "hold": action["hold"], "released": action["released"], "arrive": action["arrive"],
            "mode": tracker_.mode, "latest_stamp_s": latest_.stamp_s, "last_received_stamp_s": near["last_stamp"],
            "kappa_b": kappa_b, "sweep_m": sweep if math.isfinite(sweep) else None,
            "protect": near["protect"] is not None, **({"protection": protection} if protection else {}),
            "path_revision": revision_used,
            "episode": episode,
            "face_distance_m": face_distance,
            "correction": None if corr.point_m is None else {
                "point_m": corr.point_m, "standing_s": corr.standing_s, "installed_s": corr.installed_s},
        })
        if action["end"]:
            # A terminal stop ends the run only after the stop detector has seen the truck stand
            # for TERMINAL_HOLD_S: its 12 mm/s mean stands while the truck still creeps, and the
            # record must show the standstill (judgement (vii); S4c-1 mismatch smoke).
            since_ = near.setdefault("terminal_standing_since", now_s)
            if now_s - since_ < near_tracking.TERMINAL_HOLD_S - 1e-9:
                return action["hold"], action["released"], action["arrive"]
        else:
            near.pop("terminal_standing_since", None)
        if action["end"] and near_tracking.retry_allowed(
                current.reason, near.get("retries", 0), near["protect"] is not None, face_distance):
            # Plan D8 S4c-2: a mismatch or a refused correction before the protection, far
            # enough to see the pallet again -- release, re-observe here, plan again.
            start_pickup_retry(now_s, current.reason, face_distance, episode)
            return True, False, False
        if action["end"]:
            near["events"].append({"time_s": now_s, "event": "terminal_standing", "reason": current.reason})
            state["near_tracking"]["outcome"] = f"near_field_{current.reason}"
            require(False, f"near_field_{current.reason}")
        return action["hold"], action["released"], action["arrive"]

    def near_redraw(self, latest_, rear_now):
        """Plan D8 S4b: the docking straights re-drawn from the tracker's latest accepted
        observation, placed with its own pose (the near capture's path to a target), from
        where the truck stands: redraw_docking with the near capture's acceptance dry run
        (baseline cruise) and one at the driven cruise. Returns (plans or None, record,
        target)."""
        G2 = self.options.G2  # a runner module-level name
        near_tracking = self.options.near_tracking  # a runner module-level name
        options = self.options
        adapter = self.options.adapter
        bicycle_rollout = self.hooks.bicycle_rollout
        correction_grid = self.hooks.correction_grid
        geometry = self.geometry
        grid_world = self.hooks.grid_world
        near = self.near
        pickup_bounds = self.options.pickup_bounds
        planner_config = self.options.planner_config
        pocket = self.pocket
        redraw_docking = self.hooks.redraw_docking
        scenario = self.options.scenario
        speeds = self.options.speeds
        trackers = self.trackers
        obs_, pose_ = latest_.observation, latest_.pose
        base_xy_ = adapter.estimate_pallet_center_m(obs_, geometry.pallet_depth_m)
        site_ = adapter.estimate_world_pallet_site(base_xy_, obs_.insertion_yaw_rad, (pose_[0], pose_[1]), pose_[2])
        target_, _ = G2.planning_target(options.planning_target, site_, scenario.pickup)
        driven_ = trackers["approach"].config
        acceptance_ = driven_
        if options.approach_straight_speed_mps is not None:
            acceptance_ = replace(driven_, cruise_speed_mps=speeds["approach"])  # as the near capture
        start_ = np.asarray(rear_now, dtype=float)

        def dry(config_):
            run_ = lambda st_: bicycle_rollout(st_.poses, st_.directions, st_.curvatures_inv_m, config_, start_)  # noqa: E731
            run_.callback_spec = rollout_callback_spec(config_, start_)
            return run_

        plans_, record_ = redraw_docking(
            grid_world(scenario), PlanningPose(*(float(v) for v in rear_now)), target_, planner_config,
            geometry=geometry, pickup_bounds=pickup_bounds, dry_runs=(dry(acceptance_), dry(driven_)),
            # The heading box covers the follower's own S-curve after the 1.0 m correction (up to
            # 110.3 mrad from a start inside the box); the dry runs' end and swept checks guard
            # it (plan D8 S4b revision, S4c-1 smoke). The near capture keeps 0.05 rad.
            max_yaw_rad=near_tracking.CORRECTION_MAX_YAW_RAD,
            **correction_grid(near["first_target"], target_, rear_now),
        )
        previous_ = pocket["planning_pickup"]
        record_.update(
            site=[site_.x_m, site_.y_m, site_.yaw_rad],
            previous_target=[previous_.x_m, previous_.y_m, previous_.yaw_rad],
            dry_run_cruise_mps=[acceptance_.cruise_speed_mps, driven_.cruise_speed_mps],
        )
        return plans_, record_, target_

    def _near_capture_redraw_allowed(self) -> bool:
        """Stage 1 at a refused near capture: `on` with the near-field tracking (plan D8 S4c-3), `always` also
        without it (plan 2026-10-21 task B), `off` never."""
        mode_ = getattr(self.options, "near_capture_redraw", "on")
        return mode_ == "always" or (mode_ == "on" and self.near is not None)

    def near_capture_redraw(self, rear_now, capture_base, observation_, attempt_, acceptance_, driven_, now_s):
        """Plan D8 S4c-3 stage 1: the near capture's docking straights re-drawn as at an S4b
        correction point, before any tracker exists -- redraw_docking for the planning target
        this capture made (its correction injection included), the heading box 0.12 rad, the
        baseline- and driven-cruise dry runs, on the grid with only this target's band cleared,
        and both dry runs' first poses within 1.0 and 0.6 m of the face (estimated fork tip to
        face along the new line) inside the S4b correction box. Returns (plans or None, the
        recovery entry)."""
        near_tracking = self.options.near_tracking  # a runner module-level name
        options = self.options
        adapter = self.options.adapter
        correction_grid = self.hooks.correction_grid
        geometry = self.geometry
        grid_world = self.hooks.grid_world
        pickup_bounds = self.options.pickup_bounds
        planner_config = self.options.planner_config
        pocket = self.pocket
        redraw_docking_traced = self.hooks.redraw_docking_traced
        scenario = self.options.scenario
        from forklift_core.perception.near_field_tracking import held_pockets
        from forklift_core.planning.pallet_mission import site_poses

        target_ = pocket["planning_pickup"]
        base_xy_ = adapter.estimate_pallet_center_m(observation_, geometry.pallet_depth_m)
        raw_ = adapter.estimate_world_pallet_site(base_xy_, observation_.insertion_yaw_rad,
                                                 (capture_base[0], capture_base[1]), capture_base[2])
        tracking_ = self.near is not None
        # Plan 2026-10-22 task D: the untracked re-draw may take a wider lateral box (random_1 stood 6.7 / 9.8 cm off
        # the new line); the default leaves the keyword out, so the recorded port call is unchanged.
        lateral_ = float(getattr(options, "near_capture_redraw_lateral_m", 0.05))
        lateral_kw_ = {"max_lateral_m": lateral_} if (not tracking_ and lateral_ != 0.05) else {}
        # The dry runs run inside the Planner port, so their trajectories come back in its response:
        # a replay that does not run the callbacks still checks the same correction points (M-a1 6d).
        plans_, record_, trajectories_ = redraw_docking_traced(
            grid_world(scenario), PlanningPose(*(float(v) for v in rear_now)), target_, planner_config,
            start_rear=rear_now, dry_configs=(acceptance_, driven_), geometry=geometry, pickup_bounds=pickup_bounds,
            max_yaw_rad=near_tracking.CORRECTION_MAX_YAW_RAD, **lateral_kw_,
            **correction_grid(target_, target_, rear_now),
        )
        sites_ = site_poses(target_, geometry)
        points_, tip_x_ = [], None
        if tracking_:
            # The correction trigger's own distance: this observation placed at the capture base
            # (the tracker's anchor), its four wall points seen from the base at each rolled-out
            # pose, less the fork tip x (Codex S4c-3 implementation review P2: measured along the
            # line it checked later poses than the run would).
            held_ = held_pockets(observation_, tuple(float(v) for v in capture_base))
            widths_ = (observation_.left.width_m, observation_.right.width_m)
            tip_x_ = self.hooks.tracking_frame().fork_tip_x_m  # read from the chassis URDF by the runner
            face_distance_ = near_tracking.rollout_face_distance(held_, widths_, tip_x_, options.rear_axle_offset_m)
            if plans_ is not None:
                points_, inside_ = near_tracking.correction_points_check(
                    list(trajectories_), sites_["prealign"], face_distance_)
                if not inside_:
                    record_["refused"] = "correction_point"
                    plans_ = None
        # Without the near-field tracking (--near-capture-redraw always, plan 2026-10-21 task B) no correction fires
        # on the way in, so there are no correction points to check; redraw_docking's own checks stand.
        entry = {
            "stage": "redraw", "attempt": attempt_, "time_s": now_s,
            "capture_base": [float(v) for v in capture_base], "rear": [float(v) for v in rear_now],
            "observation": asdict(observation_),
            "raw_target": [raw_.x_m, raw_.y_m, raw_.yaw_rad],
            "target": [target_.x_m, target_.y_m, target_.yaw_rad],
            "line": [[float(v) for v in (p_.x_m, p_.y_m, p_.yaw_rad)] for p_ in (sites_["prealign"], sites_["approach"])],
            "dry_run_cruise_mps": [acceptance_.cruise_speed_mps, driven_.cruise_speed_mps],
            # The dry runs' full follower settings: the judge rolls them out again.
            "dry_run_configs": [asdict(acceptance_), asdict(driven_)],
            "record": record_, "accepted": plans_ is not None,
        }
        if tracking_:
            entry.update({"fork_tip_x_m": tip_x_, "correction_points": points_})
        else:
            # Only the untracked re-draw is marked: a tracking run's entry keeps its recorded shape (Codex task B
            # implementation review 1 P2 -- the boundary replay compares the final state exactly).
            entry["tracking"] = False
            if lateral_kw_:
                entry["max_lateral_m"] = lateral_
        return plans_, entry

    def near_protection_prediction(self, path_, latest_, widths_, rear_now, face_distance_) -> dict:
        """Plan D8 S4 near field, protection prediction 3판: near_tracking.protection_prediction for the
        re-drawn approach from where the truck stands, its disposition (the retry permission the same
        predicate the terminal stop uses, the lost injection from the arguments) and every input the
        judgement (xii) recomputes it from."""
        near_tracking = self.options.near_tracking  # a runner module-level name
        record_json = self.hooks.record_json  # a runner module-level name
        options = self.options
        near = self.near
        state = self.state
        trackers = self.trackers
        config_ = trackers["approach"].config
        arrays_ = (np.asarray(path_.poses, dtype=float), np.asarray(path_.directions),
                   np.asarray(path_.curvatures_inv_m, dtype=float))
        pred_ = near_tracking.protection_prediction(
            arrays_, config_, rear_now, latest_.held, widths_, rear_axle_offset_m=options.rear_axle_offset_m,
            fork_tip_x_m=near["fork_tip_x_m"], camera_xy=near["tracker"].config.camera_xy_m,
            blades=near["blades"], carriage_limit_m=near["carriage_limit_m"], blade_y_max_m=near["blade_y_max_m"],
            bounds=near["bounds"], terms=near["terms"],
        )
        retry_ = near_tracking.retry_allowed("replan_rejected", near.get("retries", 0), near["protect"] is not None,
                                             face_distance_)
        lost_ = options.near_drop_frames_from_m is not None
        pred_.update(
            retry_allowed=bool(retry_), lost_injection=lost_,
            disposition=near_tracking.prediction_disposition(pred_["margin_phys_m"], lost_injection=lost_,
                                                             retry_allowed=retry_),
            inputs={
                "rear": [float(v) for v in rear_now], "face_distance_m": float(face_distance_),
                "retries": int(near.get("retries", 0)), "protected": near["protect"] is not None,
                "approach": {"sha256": near_tracking.path_digest(*arrays_), "poses": arrays_[0].tolist(),
                             "directions": arrays_[1].tolist(), "curvatures_inv_m": arrays_[2].tolist()},
                "tracker_config": asdict(config_), "held": json.loads(record_json(latest_.held)),
                "widths_m": [float(v) for v in widths_], "observation_stamp_s": float(latest_.stamp_s),
                "camera_xy_m": [float(v) for v in near["tracker"].config.camera_xy_m],
                "carriage_limit_m": float(near["carriage_limit_m"]), "fork_tip_x_m": float(near["fork_tip_x_m"]),
                "bounds_sha256": state["near_tracking"]["bounds_sha256"],
                "drive_terms_sha256": state["near_tracking"]["drive_terms_sha256"],
                "constants": {"loss_per_rad": near_tracking.PREDICT_LOSS_PER_RAD, "loss_m": near_tracking.PREDICT_LOSS_M,
                              "kappa_pad": near_tracking.PREDICT_KAPPA_PAD, "protect_w_m": near_tracking.PROTECT_W_M},
            },
        )
        return pred_

    def near_install(self, now_s: float, plans_: dict, target_, point_m: float, rear_now) -> dict:
        """Plan D8 S4b: install an accepted correction at once -- the approach, insertion and
        extraction paths and their followers (each keeps its config), the pickup target the
        lift and the extraction read, the phase's start time (the timeout is measured on
        the new, shorter path), the obstacle permission's path, and a path revision with
        the paths and the followers' own copies by content."""
        near_tracking = self.options.near_tracking  # a runner module-level name
        path_record = self.hooks.path_record  # a runner module-level name
        add_path_display = self.hooks.add_path_display
        known_pallet = self.known_pallet
        loaded = self.hooks.loaded()
        ms = self.ms
        near = self.near
        obstacle = self.obstacle
        paths = self.paths
        pocket = self.pocket
        stage = self.hooks.stage()  # the runner's, read when called
        state = self.state
        trackers = self.trackers
        revision_ = near["path_revision"] + 1
        previous_ = pocket["planning_pickup"]
        entry_ = {"revision": revision_, "episode": near["episode"], "time_s": now_s, "point_m": point_m,
                  "target": [target_.x_m, target_.y_m, target_.yaw_rad],
                  "previous_target": [previous_.x_m, previous_.y_m, previous_.yaw_rad], "paths": {}, "trackers": {}}
        for name_ in ("approach", "insert", "extract"):
            path_ = plans_[name_]
            paths[name_] = path_
            trackers[name_] = RearAxlePathTracker(path_.poses, path_.directions, path_.curvatures_inv_m,
                                                  trackers[name_].config)
            state["paths"][name_] = path_record(path_)
            entry_["paths"][name_] = {
                "sha256": near_tracking.path_digest(path_.poses, path_.directions, path_.curvatures_inv_m),
                "poses": path_.poses.tolist(), "directions": path_.directions.tolist(),
                "curvatures_inv_m": path_.curvatures_inv_m.tolist(),
            }
            entry_["trackers"][name_] = near_tracking.path_digest(*trackers[name_].path_arrays())
        self.hooks.write_paths()
        add_path_display(stage, plans_["approach"], "Approach", (0.05, 0.45, 1.0))
        pocket["planning_pickup"] = target_
        if obstacle is not None:
            obstacle["pickup_estimate"] = target_
            if obstacle["layer"].known_enabled:
                known_pallet((target_.x_m, target_.y_m, target_.yaw_rad),
                             obstacle["layer"].known_config.get("r_fix_pickup_m", 0.05), f"correction:{point_m}")
            obstacle.setdefault("progress", {}).clear()
            leg_, dir_ = trackers["approach"].leg_ahead()
            obstacle["layer"].refresh(now_s, obstacle["applied"], obstacle["version"],
                                      current_pose=tuple(float(v) for v in rear_now), path_ahead=leg_, loaded=loaded,
                                      direction=dir_)
        ms.phase_started = now_s
        near["path_revision"] = revision_
        near["path_revisions"].append(entry_)
        near["events"].append({"time_s": now_s, "event": "path_revision", "revision": revision_, "point_m": point_m})
        return entry_

    def start_pickup_retry(self, now_s: float, reason: str, face_distance_m: float, episode_: int) -> None:
        """Plan D8 S4c-2: end the episode, release the SLAM hold and re-project the grid,
        carry the old pickup estimate and target into the released frame, forget the near
        capture (the next valid capture is a far one), and stand in observe to re-observe
        here -- the observe leg's hold and its follower are bypassed until that capture."""
        near_tracking = self.options.near_tracking  # a runner module-level name
        core = self
        near = self.near
        state = self.state
        transition = self.transition
        retry_ = {"time_s": now_s, "reason": reason, "episode": episode_, "face_distance_m": face_distance_m,
                  "min_face_distance_m": min((r["face_distance_m"] for r in near["ticks"]
                                              if r.get("episode") == episode_), default=None)}
        near["events"].append({"time_s": now_s, "event": "retry_start", "reason": reason, "episode": episode_})
        near["retries"] = near.get("retries", 0) + 1
        near["tracker"] = None  # frames stop reaching it and near_tick stops
        near["mismatch"] = near["unbounded"] = False
        near["stale_waits"], near["protect"], near["arrival"] = {}, None, None
        near["stop_state"] = near_tracking.StopState()
        near.pop("terminal_standing_since", None)
        state.setdefault("near_capture_history", []).append(state.pop("near_capture"))
        retry_.update(core.release_and_stand(now_s, "release_pickup_retry"))
        state["near_tracking"]["retry"] = retry_
        near["stand_record"] = "retry"
        transition("observe", now_s)

    # --- M-a1 slice 6e: closures moved from run(), statement for statement ---
    def control_correction(self):
        """The correction control applies now (the one an estimate made now is expressed in)."""
        slam = self.slam
        try:
            slam_ref_ = slam
        except NameError:
            slam_ref_ = None
        if slam_ref_ is not None and slam_ref_["tracker"].applied is not None:
            return tuple(float(v) for v in slam_ref_["tracker"].applied[0])
        return (0.0, 0.0, 0.0)

    def known_pallet(self, rect_xyyaw, r_fix_m: float, event: str) -> None:
        """Replace the known pallet with a fresh fix (plan v10 D5: the age restarts).

        Stored with the correction control applies at this instant -- the one the
        estimate is in -- not the layer's last-scan copy, which lags a capture's
        release (Codex stage-1 2nd P1-1: 0.30 m shown 0.30 m off).
        """
        control_correction = self.control_correction
        geometry = self.geometry
        obstacle = self.obstacle
        state = self.state
        from forklift_core.perception.known_obstacles import KnownRect

        stamp_ = float(obstacle.get("last_stamp", 0.0))
        applied_ = control_correction()
        obstacle["layer"].known = [KnownRect(
            (float(rect_xyyaw[0]), float(rect_xyyaw[1]), geometry.pallet_depth_m, geometry.pallet_width_m,
             float(rect_xyyaw[2])),
            fix_stamp_s=stamp_, r_fix_m=float(r_fix_m), rho_m=None, frame_correction=tuple(applied_))]
        state.setdefault("known_pallets", []).append(
            {"event": event, "time_s": stamp_, "rect": [float(v) for v in rect_xyyaw], "r_fix_m": float(r_fix_m)})

    def _return_in_tick(self) -> bool:
        """Plan D11: this return turnaround is planned in the tick -- its worker answers were dropped for a changed
        correction RETURN_CORRECTION_FALLBACK times in this mission's return (the way it ran before D11)."""
        options, ms = self.options, self.ms
        if not (options.return_turnaround and ms.phase == "return_home"):
            return False
        return self.return_correction_drops.get(ms.phase, 0) >= self.RETURN_CORRECTION_FALLBACK

    def return_turnaround_kwargs(self, frozen: bool = False) -> dict:
        """--return-turnaround: the plan_return_leg options for the return path the truck sets off
        with -- the first live plan on the grid, which runs after the SLAM release and the obstacle
        block's re-projection, so the arc check's snapshot and the plan's grid share one correction
        (the release replan itself is replaced by it and passes none; Codex implementation review
        2026-10-12 P2). Later return replans pass none. Each turnaround arc is checked on the drive
        permission's snapshot before it is compared: the planner's grid takes UNKNOWN cells as free,
        the permission does not enter them (Codex design review 2026-10-12)."""
        options = self.options
        grid_planning = self.options.grid_planning
        obstacle = self.obstacle
        slam = self.slam
        if not options.return_turnaround:
            return {}
        try:  # as grid_kwargs: the first plans run before the SLAM link exists
            slam_link_ = slam
        except NameError:
            slam_link_ = None
        if slam_link_ is not None and slam_link_.get("pending_release") is not None:
            return {}  # not before the release: the snapshot is re-projected after it

        def arc_check(poses):
            if not grid_planning:
                return None
            layer_ = obstacle["layer"]
            if getattr(layer_, "snapshot", None) is None:
                return "no_snapshot"
            # The start part verified as the permission's own walk, no known occupied cell on the whole
            # arc (ObstacleLayer.check_turnaround_arc; Codex implementation reviews 2026-10-12).
            return layer_.check_turnaround_arc(poses, current_pose=tuple(float(v) for v in poses[0]))

        # What the check runs with, without asking the layer anything (M-a1 replay plan 3.4): the grid switch and
        # the obstacle snapshot's version the arcs are checked on.
        arc_check.callback_spec = {"kind": "turnaround_arc", "grid_planning": bool(grid_planning),
                                   "obstacle_version": obstacle.get("version") if obstacle is not None else None}
        if frozen:
            # Plan D11: for the worker -- the permission and its snapshot copied now (the plan's grid is fixed now
            # too); no grid planning: no check, as the closure's None.
            checker = (obstacle["layer"].turnaround_arc_check(dict(arc_check.callback_spec)) if grid_planning
                       else None)
            return {"forward_turnaround": True, "turnaround_check": checker}

        return {"forward_turnaround": True, "turnaround_check": arc_check}

    def realign_active(self) -> bool:
        """Plan D8 S4c-3 stage 2: a realignment started and its re-capture not yet accepted."""
        state = self.state
        rec_ = (state.get("near_tracking") or {}).get("realign")
        return rec_ is not None and rec_.get("end") is None

    def realign_install(self, reason: str) -> None:
        """Plan D8 S4c-3 stage 2: every observe path installed during the realignment, by
        content and by its follower's copy, with the SLAM mode then; a path longer than
        REALIGN_MAX_LENGTH_M ends the run (no recovery reaches past it)."""
        near_tracking = self.options.near_tracking  # a runner module-level name
        require = self.hooks.require  # a runner module-level name
        initial_time = self.options.initial_time
        obstacle = self.obstacle
        paths = self.paths
        realign_active = self.realign_active
        slam = self.slam
        state = self.state
        trackers = self.trackers
        world = self.hooks.world()  # the runner's, read when called
        if not realign_active():
            return
        path_ = paths["observe"]
        poses_ = np.asarray(path_.poses, dtype=float)
        length_ = float(np.sum(np.hypot(*np.diff(poses_[:, :2], axis=0).T))) if len(poses_) > 1 else 0.0
        installs_ = state["near_tracking"]["realign"]["installs"]
        installs_.append({
            "n": len(installs_), "time_s": world.current_time - initial_time, "reason": reason,
            "slam_mode": slam["tracker"].mode, "length_m": length_, "poses": poses_.tolist(),
            "directions": np.asarray(path_.directions).tolist(),
            "curvatures_inv_m": np.asarray(path_.curvatures_inv_m).tolist(),
            "sha256": near_tracking.path_digest(path_.poses, path_.directions, path_.curvatures_inv_m),
            "tracker_sha256": near_tracking.path_digest(*trackers["observe"].path_arrays()),
        })
        if obstacle is not None:
            obstacle.setdefault("progress", {}).clear()  # the no-progress watch restarts on it
        require(length_ <= near_tracking.REALIGN_MAX_LENGTH_M + 1e-9, f"near_capture_realign_too_long:{length_:.3f}")

    def start_near_realign(self, now_s: float, attempt_: int) -> None:
        """Plan D8 S4c-3 stage 2: a near capture refused by its acceptance and by stage 1 --
        once per pickup -- releases SLAM, re-projects the grid, carries the estimates, keeps
        the refused capture apart (near_capture_refused, not near_capture_history) and stands
        to re-observe here; the next valid capture is a far one and the normal flow follows.
        No tracker exists yet: no retry event, no retry count, no episode closed."""
        record_json = self.hooks.record_json  # a runner module-level name
        core = self
        near = self.near
        realign_install = self.realign_install
        state = self.state
        transition = self.transition
        state["near_tracking"]["realigns"] = state["near_tracking"].get("realigns", 0) + 1
        state["near_capture"]["refused_attempt"] = attempt_
        state.setdefault("near_capture_refused", []).append(json.loads(record_json(state.pop("near_capture"))))
        record_ = {"time_s": now_s, "refused_attempt": attempt_, "installs": [], "end": None}
        state["near_tracking"]["realign"] = record_
        record_.update(core.release_and_stand(now_s, "release_near_realign"))
        near["stand_record"] = "realign"
        realign_install("stand")
        transition("observe", now_s)

    def apply_tracker_profile(self) -> None:
        apply_tracker_profile_to(self.trackers, self.options.tracker_profile)

    def record_tracker_configs(self) -> None:
        record_tracker_configs_in(self.state, self.trackers)

    def build_near_tracker(self, now_s: float) -> None:
        """Plan D8 S4a ①: the D8c 3판 tracker from the near capture, in the SLAM hold that
        follows it -- the capture's observation, the control base it was placed with and
        t_before_capture_s. M-a1 slice 6e: the mission side (the hold and contract checks, the
        episode, the injection latch, the tracker's state and records); the runner builds the
        service -- NearFieldConfig and the detectors (near_tracker_config, reading the URDF),
        the tracker and its recording proxy (near_tracker_create) and the fork geometry
        (near_fork_geometry) -- in the order the closure did."""
        near_tracking = self.options.near_tracking  # a runner module-level name
        record_json = self.hooks.record_json  # a runner module-level name
        require = self.hooks.require  # a runner module-level name
        near, slam, state, pocket, trackers = self.near, self.slam, self.state, self.pocket, self.trackers
        start = near["start"]
        require(start is not None, "near_field_without_capture")
        require(slam is not None and slam["tracker"].mode == "holding", "near_field_without_hold")
        service = self.hooks.near_tracker_config()
        config, frame_geo = service.config, service.frame_geo
        # The followers as the measurement runs drove them (Codex S4a-1 2nd review P2):
        # their configs exist only once the mission is planned, so they are checked here,
        # standing at the near capture, before any near-field driving.
        tracker_problems = near_tracking.tracker_config_problems(
            near["terms"].condition.get("tracker_configs"), json.loads(record_json(state.get("tracker_configs")))
        )
        state["near_tracking"]["tracker_config_problems"] = tracker_problems
        require(not tracker_problems, "near_field_contract:" + "; ".join(tracker_problems))
        episode = self.hold_count()
        near["injection_closed"] = True  # Plan D8 S4c-1: no capture after this one is biased
        near["tracker"] = self.hooks.near_tracker_create(service, start, episode)
        blades_, corners_ = self.hooks.near_fork_geometry()
        near.update(episode=episode, fork_tip_x_m=frame_geo.fork_tip_x_m,
                    last_stamp=start["capture_stamp_s"], last_fed_stamp=None,
                    # Plan D8 S4a-2: the blades, the carriage front corners and the depth
                    # past which the carriage meets the face first.
                    blades=blades_, blade_y_max_m=max(max(abs(b_[2]), abs(b_[3])) for b_ in blades_),
                    corners=corners_,
                    carriage_limit_m=frame_geo.carriage_limit_m,
                    protect=None, stop_state=near_tracking.StopState(), arrival=None,
                    stale_waits={},
                    # Plan D8 S4b: the correction points, from the mission the near capture planned.
                    correction=near_tracking.CorrectionState(), first_target=pocket["planning_pickup"],
                    # Plan D8 S4c-2: an episode's execution latches start clean; its records
                    # (stop records, events, corrections, revisions, tick rows) accumulate.
                    mismatch=False, unbounded=False, path_revision=0)
        near.setdefault("stop_events", [])
        near.pop("terminal_standing_since", None)
        near["history"].clear()
        state["near_tracking"]["start"] = {
            "built_s": now_s,
            "attempt": start["attempt"],
            "pose": list(start["pose"]),
            "aligned_s": start["aligned_s"],
            "capture_stamp_s": start["capture_stamp_s"],
            "episode": episode,
            "gate_front_x_m": config.handoff_start_front_x_m,
            "fork_tip_x_m": frame_geo.fork_tip_x_m,
            "observation": asdict(start["observation"]),
            # Plan D8 S4c-3: the paths the tracker starts on (path revision 0), by the followers'
            # own copies.
            "paths0": {name_: near_tracking.path_digest(*trackers[name_].path_arrays())
                       for name_ in ("approach", "insert", "extract")},
        }
        state["near_tracking"].setdefault("starts", []).append(state["near_tracking"]["start"])
        near["events"].append({"time_s": now_s, "event": "tracker_built", "episode": episode})
