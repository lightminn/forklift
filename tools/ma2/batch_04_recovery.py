"""M-a2 batch 4: grid kwargs, retraction, projection, path records, progress baseline, D7c, obstacle recovery v4
(escape), cusp position stall v7 wiring, near protection prediction (test_isaac_transport_planning_config.py)."""

PC = "tests/unit/test_isaac_transport_planning_config.py::"
M = "tests/unit/mission/"
BR, REL, FOL, CMD = M + "test_core_blocked_replan.py::", M + "test_core_release.py::", M + "test_core_follow.py::", M + "test_core_command.py::"
DS, NF, TR, DH = M + "test_core_docking_stall.py::", M + "test_core_near_field.py::", M + "test_core_transition.py::", M + "test_core_docking_helpers.py::"
CR, CO = M + "test_core_cusp_rules.py::", M + "test_core_cusp_observe.py::"
CORE = "src/forklift_core/mission/core.py"
WIRING = ("walks the runner and the core together: the runner's own sites, constants and cross-file order stay "
          "guarded by this test as written; the core's behaviour is guarded by the D/N properties beside it")
RUNNER = "the code it guards stays in the runner (grid_kwargs, the scenario's truth destination, argument checks)"


def kept(test, statement, reason=WIRING):
    return {"id": f"B4:R:{test}", "class": "R", "checks": [PC + test + "::*"], "statement": statement,
            "tests": [PC + test], "reason": reason}


def prop(pid, test, statement, guards, find, replace, count=1, cls="D"):
    return {"id": f"B4:{pid}", "class": cls, "checks": [PC + test + "::*"], "statement": statement, "tests": guards,
            "mutation": {"file": CORE, "find": find, "replace": replace, "count": count}}


PROPERTIES = [
    kept("test_grid_kwargs_never_reads_slam_before_it_exists", "grid_kwargs never reads slam before it exists", RUNNER),
    kept("test_slam_ref_lives_only_inside_grid_kwargs", "slam_ref lives only inside grid_kwargs", RUNNER),
    prop("pickup-retraction", "test_the_pickup_retraction_covers_the_estimate_and_the_range_noise",
         "the pickup retraction grows the estimate by 0.05 m plus the layer's range-noise cut",
         [TR + "test_extract_retracts_the_pickup_from_the_memory_by_the_estimate"],
         'grow_ = 0.05 + obstacle["layer"].noise_cut_m', 'grow_ = 0.05'),
    kept("test_plans_project_with_the_grids_correction_and_a_release_passes_its_own",
         "plans project with the grid's correction; a release passes its own"),
    prop("release-passes-applied", "test_plans_project_with_the_grids_correction_and_a_release_passes_its_own",
         "a release's replans pass the released correction to the grid",
         [REL + "test_a_large_jump_replans_the_transport_with_the_released_correction_then_arms_the_docking"],
         "grid_kwargs(None, own=after, applied=released_)", "grid_kwargs(None, own=after)", count=4),
    prop("replan-records-path", "test_replans_record_their_path_before_the_mission_plan_exists",
         "an observe replan records its path even before the mission plan exists",
         [CO + "test_an_observe_replan_records_its_path_before_any_mission_plan_exists"],
         'state.setdefault("paths", {})["observe"] = path_record(replanned)', 'pass', cls="N"),
    kept("test_every_path_replacement_restarts_the_no_progress_baseline",
         "every path replacement restarts the no-progress baseline (runner and core sites)"),
    prop("replan-restarts-progress", "test_every_path_replacement_restarts_the_no_progress_baseline",
         "a blocked replan's new path restarts the no-progress watch",
         [BR + "test_a_new_path_is_installed_with_the_leg_config_and_restarts_the_phase_and_the_watch"],
         'require(leg_replans <= 10, f"obstacle_replan_limit in {ms.phase}: {leg_replans}")\n            obstacle.setdefault("progress", {}).clear()',
         'require(leg_replans <= 10, f"obstacle_replan_limit in {ms.phase}: {leg_replans}")\n            pass'),
    prop("d7-inert-docking", "test_d7c_docking_branches_are_inert_without_the_flag",
         "without --d7-docking an unaligned straight ends the run (no retry branch)",
         [DH + "test_without_d7_an_unaligned_straight_ends_the_run"],
         "        if path is None and options.d7_docking:\n", "        if path is None:\n"),
    prop("d7-inert-match", "test_d7c_docking_branches_are_inert_without_the_flag",
         "without --d7-docking an accepted match records no D7 seed or committed goal",
         [DH + "test_without_d7_an_unaligned_straight_ends_the_run"],
         "        if options.d7_docking and result.accepted:\n", "        if result.accepted:\n"),
    prop("d7-reapproach-keeps-docking", "test_d7c_reapproach_is_under_the_permission_and_kept_by_obstacle_replans",
         "an obstacle replan during a re-approach replans the re-approach and does not arm the docking",
         [BR + "test_transport_replans_through_the_dispatch_and_arms_the_docking_unless_reapproaching"],
         'and not slam["docking"].get("reapproach"):\n                arm_docking()', ':\n                arm_docking()'),
    kept("test_d7c_reapproach_is_under_the_permission_and_kept_by_obstacle_replans",
         "the re-approach is under the permission and kept by obstacle replans"),
    kept("test_a_destination_prior_error_moves_planning_only", "a destination prior error moves planning only", RUNNER),
    kept("test_d7c_docking_needs_slam", "--d7-docking needs --slam-feedback", RUNNER),
    prop("escape-on-invalid-start", "test_obstacle_recovery_v4_escapes_before_the_clearance_zero_replan",
         "an invalid start on a travel leg tries an escape straight before the zero-clearance replan",
         [BR + "test_an_invalid_start_tries_an_escape_straight_first_and_drives_it_on_the_backoff_machinery"],
         'replanned.status == "invalid_start" and ms.phase in ("transport", "return_home")',
         'replanned.status == "invalid_start" and ms.phase in ()'),
    prop("escape-budget", "test_obstacle_recovery_v4_escapes_before_the_clearance_zero_replan",
         "escapes are budgeted per leg",
         [BR + "test_an_invalid_start_tries_an_escape_straight_first_and_drives_it_on_the_backoff_machinery"],
         'obstacle.setdefault("escapes", {}).get(ms.phase, 0) < ESCAPE_PER_LEG', 'obstacle.setdefault("escapes", {}).get(ms.phase, 0) < 0'),
    prop("escape-on-backoff", "test_obstacle_recovery_v4_escapes_before_the_clearance_zero_replan",
         "an escape drives on the backoff machinery, not as a failure, with its record",
         [BR + "test_an_invalid_start_tries_an_escape_straight_first_and_drives_it_on_the_backoff_machinery"],
         "backoff=True, failed=False, escape=escape_.record()", "backoff=True, failed=True, escape=escape_.record()"),
    prop("escape-skips-backoff", "test_obstacle_recovery_v4_escapes_before_the_clearance_zero_replan",
         "the old backoff is skipped where the escape's checker refused its straight",
         [BR + "test_the_return_backoff_is_skipped_when_it_would_sweep_the_delivered_pallet"],
         "back_ok_ = not escape_skips_back_", "back_ok_ = True"),
    kept("test_obstacle_recovery_v4_escapes_before_the_clearance_zero_replan",
         "the escape constants and the replan's order (ESCAPE_PER_LEG in the runner)"),
    prop("escape-end", "test_obstacle_recovery_v4_an_escape_end_bypasses_docking_stall_and_failure",
         "an escape's end is a backoff end: standing 120 ticks after a failure ends it",
         [FOL + "test_an_escape_end_is_the_backoff_end"], 'back_e["stand_ticks"] >= 120', 'back_e["stand_ticks"] >= 1200'),
    kept("test_obstacle_recovery_v4_an_escape_end_bypasses_docking_stall_and_failure",
         "an escape end bypasses the docking stop, the stall and the failure (order in the follow step)"),
    prop("backoff-end-release", "test_obstacle_recovery_v4_releases_a_hold_at_a_backoff_end_and_in_its_recovery",
         "a backoff end releases a hold once standing",
         [CMD + "test_a_backoffs_end_stands_releases_once_and_forces_a_replan_each_second"],
         'release_after_backoff(t, "release_after_backoff")', 'None'),
    prop("backoff-end-force-replan", "test_obstacle_recovery_v4_releases_a_hold_at_a_backoff_end_and_in_its_recovery",
         "a backoff end forces a replan each second while standing",
         [CMD + "test_a_backoffs_end_stands_releases_once_and_forces_a_replan_each_second"],
         'if standing_b and t - back_.get("forced_s", -1e9) >= 1.0:', 'if standing_b and t - back_.get("forced_s", -1e9) >= 5.0:'),
    prop("tight-candidates-record", "test_obstacle_recovery_v4_releases_a_hold_at_a_backoff_end_and_in_its_recovery",
         "a replan through the tight candidates records clearance 0",
         [BR + "test_before_recognition_a_blocked_observe_leg_tries_the_remaining_candidates_from_here"],
         "clearance_used_ = 0.0  # the candidates are planned tight", "clearance_used_ = 0.3  # the candidates are planned tight"),
    kept("test_obstacle_recovery_v4_releases_a_hold_at_a_backoff_end_and_in_its_recovery",
         "the backoff-end release, the recovery's release and records, and grid_kwargs' band pose (runner)"),
    kept("test_cusp_position_stall_v7_is_wired_into_the_runner", "the cusp position stall v7 wiring (runner and core)"),
    prop("stall-replans-two", "test_cusp_position_stall_v7_is_wired_into_the_runner",
         "at most two stall replans", [DS + "test_two_stall_replans_at_most_and_none_after_a_docking_match"],
         'len(state["stall_replans"]) < 2', 'len(state["stall_replans"]) < 3', count=3),
    prop("observe-hold-blocks", "test_cusp_position_stall_v7_is_wired_into_the_runner",
         "a held observe cusp is not taken by the observe recovery",
         [CO + "test_an_observe_cusp_stall_counts_120_standing_ticks_and_the_budget_is_two_per_leg"],
         "and not cusp_monitor.holds_observe()", "and True"),
    prop("near-protection-refusal", "test_near_protection_prediction_screens_only_the_last_correction",
         "a predicted protection shortfall at the last correction point refuses the redraw",
         [NF + "test_the_last_correction_point_screens_the_protection_and_a_predicted_shortfall_is_refused"],
         'if pred_["disposition"] == "refused":', 'if False:'),
    prop("near-protection-last-point", "test_near_protection_prediction_screens_only_the_last_correction",
         "only the last correction point is screened",
         [NF + "test_a_correction_before_the_last_point_is_not_screened_by_the_protection_prediction"],
         "if plans_ is not None and abs(corr.point_m - near_tracking.PREDICT_POINT_M) < 1e-9:", "if plans_ is not None:", cls="N"),
    kept("test_near_protection_prediction_screens_only_the_last_correction",
         "the protection prediction's rules are recorded (runner) and its disposition decided in order"),
]
