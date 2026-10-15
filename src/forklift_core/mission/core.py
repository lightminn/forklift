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

The bodies are the runner's, statement for statement: moving them changes no decision.
"""

from __future__ import annotations

import math
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from forklift_core.control import RearAxlePathTracker
from forklift_core.mission.state import MissionState

__all__ = ["MissionCore"]


class MissionCore:
    def __init__(self, ms: MissionState, *, state: dict, obstacle: dict | None, slam: dict | None, pocket: dict,
                 paths: dict, trackers: dict, geometry, settings: dict, pallet_geometry, hooks: SimpleNamespace):
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
