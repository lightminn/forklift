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

import math
from dataclasses import asdict, replace
from types import SimpleNamespace

import numpy as np

from forklift_core.control import RearAxlePathTracker, TrackerConfig
from forklift_core.control import cusp_stall as CUSP_STALL
from forklift_core.mission.state import MissionState
from forklift_core.perception.pocket_detector import DetectorParams
from forklift_core.planning import PlanResult
from forklift_core.planning import Pose2D as PlanningPose

__all__ = ["MissionCore"]


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
                obstacle["delivered_pending"] = (grid_compose(rear_, obstacle["pallet_rel"]), hooks.control_correction(),
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
            hooks.known_pallet(grid_compose(rear_, rel_), r_fix_, "delivered")
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
            slam["tracker"].hold(odom_from_base=hooks.slam_odom_base())
            slam["holds"].append(
                {"phase": ms.phase, "time_s": t, "event": "hold", "error": hooks.slam_error()}
            )
        elif ms.phase in ("transport", "settle"):
            slam["pending_release"] = ms.phase
            slam["release_wait_from"] = t

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
                hooks.dock_at_delivery_straight(t)
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
        ms.lift_command += float(
            np.clip(
                desired_lift - ms.lift_command,
                -settings["lift_rate_mps"] * dt,
                settings["lift_rate_mps"] * dt,
            )
        )
        if ms.phase == "lift" and t - ms.phase_started > 4.5:
            hooks.require(hooks.pallet_z() > 0.06, "Pallet was not lifted")
            transition("extract", t)
        if ms.phase == "lower" and t - ms.phase_started > 4.5:
            hooks.require(
                abs(hooks.pallet_z()) < 0.008, "Pallet did not settle onto destination floor"
            )
            hooks.require(
                hooks.lift_position() < 0.012,
                "Fork did not lower",
            )
            transition("withdraw", t)
        if ms.phase == "settle" and t - ms.phase_started > 1.0:
            if "return_home" in self.trackers:
                transition("return_home", t)
            else:
                transition("complete", t)
                return True
        if ms.phase == "home_settle" and t - ms.phase_started > 1.0:
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
        grid_world, grid_kwargs, return_turnaround_kwargs = hooks.grid_world, hooks.grid_kwargs, hooks.return_turnaround_kwargs
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
                    hooks.arm_docking()
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
            obstacle["version"] += 1
            obstacle["applied"] = applied_now
            obstacle["reprojections"] = obstacle.get("reprojections", 0) + 1
            leg_now, leg_dir_now = trackers[ms.phase].leg_ahead()
            obstacle["layer"].refresh(
                t, applied_now, obstacle["version"], current_pose=tuple(float(v) for v in rear),
                path_ahead=leg_now, loaded=loaded, direction=leg_dir_now,
            )
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
        acting = (
            ms.phase in ("observe", "approach", "transport", "return_home")
            or (ms.phase == "withdraw" and obstacle["layer"].known_enabled
                and obstacle["layer"].known_config.get("withdraw_mode", "certified") == "certified")  # D5 corridor
        ) and not docking_straight
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
                hooks.require(t - pocket["zero_since"] < 5.0, f"pocket_stop in {ms.phase}: {why}")
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
            not (options.grid_planning and acting and t - watch["since_s"] > 60.0),
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
        ms, state, obstacle, slam, pocket, hooks, options = (self.ms, self.state, self.obstacle, self.slam, self.pocket,
                                                            self.hooks, self.options)
        paths, trackers, geometry = self.paths, self.trackers, self.geometry
        planner_config, travel_config, scenario = options.planner_config, options.travel_config, options.scenario
        return_to_pose, pickup_bounds, mission_stages = options.return_to_pose, options.pickup_bounds, options.mission_stages
        BACKOFF_M, BACKOFF_SPEED_MPS, BACKOFF_PER_LEG, ESCAPE_PER_LEG = (
            options.backoff_m, options.backoff_speed_mps, options.backoff_per_leg, options.escape_per_leg)
        require, wall, same_path, path_stats, path_record = (hooks.require, hooks.wall, hooks.same_path, hooks.path_stats,
                                                             hooks.path_record)
        realign_active, realign_install, release_and_carry, release_after_backoff, slam_rear = (
            hooks.realign_active, hooks.realign_install, hooks.release_and_carry, hooks.release_after_backoff,
            hooks.slam_rear)
        grid_world, grid_kwargs, plan_observation_leg, plan_return_leg, plan_transport = (
            hooks.grid_world, hooks.grid_kwargs, hooks.plan_observation_leg, hooks.plan_return_leg, hooks.plan_transport)
        transport_replan, find_escape, escape_leg, final_straight_prefix = (
            hooks.transport_replan, hooks.find_escape, hooks.escape_leg, hooks.final_straight_prefix)
        apply_tracker_profile, arm_docking = hooks.apply_tracker_profile, hooks.arm_docking
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
                start_rear=start, pickup_bounds=pickup_bounds, extended=False,
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
                    start_rear=start, pickup_bounds=pickup_bounds, extended=False,
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
                    pickup_bounds=pickup_bounds, extended=False, **grid_kwargs("observe"),
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
                      if r["phase"] == ms.phase and t - r["time_s"] < 30.0
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
            hooks.require, hooks.dump_tracking, hooks.cusp_heading_end, hooks.cusp_heading_offset, hooks.cusp_tick)
        slam_odom_base, slam_error = hooks.slam_odom_base, hooks.slam_error
        cusp_holds, CUSP_HEADING_HOLD_PHASES = hooks.cusp_holds(), options.cusp_heading_hold_phases
        limit = core.tracking_limit(t)  # M-a1 slice 2
        if t - ms.phase_started >= limit:
            dump_tracking("timeout", ms.last_tracking)
        require(t - ms.phase_started < limit, f"Tracking timeout in {ms.phase}")
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
            slam["tracker"].hold(odom_from_base=slam_odom_base())
            slam["holds"].append(
                {"phase": ms.phase, "time_s": t, "event": "hold", "error": slam_error()}
            )
        return SimpleNamespace(tracking=tracking, cusp_hold=cusp_hold)

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
            hooks.dock_at_delivery_straight, hooks.start_docking_retry, hooks.slam_rear, hooks.slam_odom_base,
            hooks.arm_docking)
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
                jump_m, jump_rad = slam["tracker"].release(odom_from_base=slam_odom_base())
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
            hooks.require, hooks.wall, hooks.path_record, hooks.start_docking_retry, hooks.slam_rear, hooks.arm_docking)
        grid_world, grid_kwargs, plan_transport_leg, plan_observation_leg = (
            hooks.grid_world, hooks.grid_kwargs, hooks.plan_transport_leg, hooks.plan_observation_leg)
        realign_active, realign_install = hooks.realign_active, hooks.realign_install
        cusp_holds, cusp_monitor = hooks.cusp_holds(), hooks.cusp_monitor()
        cusp_heading_motion, cusp_heading_end, cusp_heading_offset = (
            hooks.cusp_heading_motion, hooks.cusp_heading_end, hooks.cusp_heading_offset)
        cusp_stopped, cusp_end_recovered, cusp_leg_checker, cusp_sensors = (
            hooks.cusp_stopped, hooks.cusp_end_recovered, hooks.cusp_leg_checker, hooks.cusp_sensors)
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
        observe_miss = (
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
            and len(state["observe_replans"]) - state["observe_replan_leg"]["start"]
            < OBSERVE_REPLANS_PER_LEG
        )
        if observe_miss:
            ms.observe_stall_ticks = 0
            tracking = replace(tracking, status="braking", speed_mps=0.0)
            if slam["stop_now"]:
                target = PlanningPose(*(float(v) for v in paths["observe"].poses[-1]))
                replan_start = wall()
                replanned = plan_observation_leg(
                    grid_world(scenario),
                    target,
                    planner_config,
                    geometry=geometry,
                    start_rear=PlanningPose(float(rear[0]), float(rear[1]), float(rear[2])),
                    pickup_bounds=pickup_bounds,
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
            hooks.require, hooks.dump_tracking, hooks.near_tick, hooks.release_after_backoff)
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
        if slam is not None and not slam["tracker"].may_drive():
            requested_speed = 0.0  # warming up: hold still
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
            if t - loose["since_s"] >= 1.0:
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
            if standing_b and t - back_.get("forced_s", -1e9) >= 1.0:
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
        apply_tracker_profile, realign_install, record_tracker_configs = (
            hooks.apply_tracker_profile, hooks.realign_install, hooks.record_tracker_configs)
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
                pickup_bounds=pickup_bounds,
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
        trackers["observe"] = RearAxlePathTracker(
            observe_plan.poses,
            observe_plan.directions,
            observe_plan.curvatures_inv_m,
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
        ms.phase_started = t
        apply_tracker_profile()
        realign_install("observe_candidate")  # Plan D8 S4c-3 stage 2
        record_tracker_configs()
        return False

    def observe_succeeded(self, t: float, rear, base, yaw: float, attempt: dict, attempt_number: int, observation,
                          scene_input, capture_diagnostics, step: int) -> SimpleNamespace:
        """A valid observation: the repeat diagnostic's early end, the near-field injection (S4c-1), the pallet
        site from the observation and the capture base (the runner's mixed sample, passed as it stands), the
        near stand and near-capture records, the evaluation records (the runner's, at the same point), the
        planning target, the known pallet, the mission plan and its followers, and the pending near capture
        of the first far observation under SLAM. Returns ``stop`` (the repeat diagnostic ends the loop),
        ``observation`` (biased under a mismatch injection) and ``near_leg``."""
        ms, state, obstacle, slam, pocket, hooks, options, near = (self.ms, self.state, self.obstacle, self.slam,
                                                                   self.pocket, self.hooks, self.options, self.near)
        paths, trackers, geometry, settings, transition = self.paths, self.trackers, self.geometry, self.settings, self.transition
        planner_config, travel_config, scenario, pickup_bounds = (options.planner_config, options.travel_config,
                                                                  options.scenario, options.pickup_bounds)
        return_to_pose, mission_stages, speeds = options.return_to_pose, options.mission_stages, options.speeds
        CUSP_HEADING_HOLD_RAD, CUSP_HEADING_HOLD_PHASES = options.cusp_heading_hold_rad, options.cusp_heading_hold_phases
        adapter, near_tracking, Pose2D = options.adapter, options.near_tracking, PlanningPose
        require, wall, path_record, record_json, known_pallet = (hooks.require, hooks.wall, hooks.path_record,
                                                                 hooks.record_json, hooks.known_pallet)
        grid_world, grid_kwargs, plan_transport, final_straight_prefix = (
            hooks.grid_world, hooks.grid_kwargs, hooks.plan_transport, hooks.final_straight_prefix)
        apply_tracker_profile, record_tracker_configs = hooks.apply_tracker_profile, hooks.record_tracker_configs
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
        planning_start = wall()
        planning_trace = []
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
            {
                name: RearAxlePathTracker(
                    path.poses,
                    path.directions,
                    path.curvatures_inv_m,
                    TrackerConfig(
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
                    ),
                )
                for name, path in paths.items()
                if name != "observe"
            }
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
        return SimpleNamespace(stop=False, observation=observation, near_leg=near_leg)

    def near_leg_or_recapture(self, t: float, rear, base, yaw: float, observation, attempt_number: int, near_leg) -> None:
        """After the mission plan: the first far observation under SLAM drives the near leg to the near capture
        point; the near capture itself re-draws the approach straight from where the truck stands (box,
        dry run, the S4c-3 re-draw acceptance, the once-per-pickup realignment), installs it with the pocket
        check and the near-field tracker, and moves to the approach."""
        ms, state, slam, hooks, options, near = self.ms, self.state, self.slam, self.hooks, self.options, self.near
        paths, trackers, geometry, transition = self.paths, self.trackers, self.geometry, self.transition
        speeds, near_tracking, Pose2D = options.speeds, options.near_tracking, PlanningPose
        require, path_record, final_straight_prefix, realign_active, realign_install = (
            hooks.require, hooks.path_record, hooks.final_straight_prefix, hooks.realign_active, hooks.realign_install)
        straight_from_pose, bicycle_rollout, near_capture_redraw, start_near_realign = (
            hooks.straight_from_pose, hooks.bicycle_rollout, hooks.near_capture_redraw, hooks.start_near_realign)
        record_tracker_configs, build_pocket_check, build_near_tracker = (
            hooks.record_tracker_configs, hooks.build_pocket_check, hooks.build_near_tracker)
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
                redraw_entry = None
                if straight is None and near is not None and options.near_capture_redraw == "on":
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
                        build_pocket_check(float(straight.poses[-1][2]), rear, base)
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

    def capture_and_detect(self, t: float, rear, step: int) -> SimpleNamespace:
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
        attempt_number = len(state["observation_attempts"]) + 1
        attempt = {
            "candidate_index": ms.next_candidate_index - 1,
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
        try:
            scene_input, frame_diagnostics, capture_attempts = (
                perception_capture.capture(
                    max_attempts=options.perception_max_attempts,
                )
            )
        except adapter.CaptureFailure as exc:
            attempt["capture_failure"] = exc.reason
            attempt["capture_diagnostics"] = asdict(exc.diagnostics)
            attempt["capture_attempts"] = exc.diagnostics.attempts
            require(False, f"perception_capture_failed:{exc.reason}")
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
                jump_m, jump_rad = slam["tracker"].release(
                    odom_from_base=slam_odom_base()
                )
                slam["holds"].append(
                    {
                        "phase": ms.phase,
                        "time_s": now_s,
                        "event": "release_at_capture",
                        "jump_m": jump_m,
                        "jump_rad": jump_rad,
                    }
                )
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
        detection = detect_pockets(detector_input, prior, params)
        observation = detection.observation
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
                               observation=observation)
