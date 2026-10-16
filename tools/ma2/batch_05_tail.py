"""M-a2 batch 5: the rest of test_isaac_transport_planning_config.py -- the return turnaround, the cusp heading hold,
the per-leg observe replan budget (core), and the runner's probe, plant, encoder, odometry and tracker-book tests."""

PC = "tests/unit/test_isaac_transport_planning_config.py::"
M = "tests/unit/mission/"
LL, CO, NC = M + "test_core_live_leg.py::", M + "test_core_cusp_observe.py::", M + "test_core_near_capture.py::"
CORE = "src/forklift_core/mission/core.py"
WIRING = ("walks the runner and the core together: the runner's own sites, constants and cross-file order stay "
          "guarded by this test as written; the core's behaviour is guarded by the D/N properties beside it")
RUNNER = ("the code it guards stays in the runner (the steering probe, the plant readback and settings, the encoder "
          "and odometry wiring, the tracker book, the leg-switch stamping, the boundary record's close)")


def kept(test, statement, reason=WIRING):
    return {"id": f"B5:R:{test}", "class": "R", "checks": [PC + test + "::*"], "statement": statement,
            "tests": [PC + test], "reason": reason}


def prop(pid, test, statement, guards, find, replace, count=1, cls="D"):
    return {"id": f"B5:{pid}", "class": cls, "checks": [PC + test + "::*"], "statement": statement, "tests": guards,
            "mutation": {"file": CORE, "find": find, "replace": replace, "count": count}}


PROPERTIES = [
    kept("test_the_return_turnaround_runs_on_the_first_live_return_plan_only_after_the_release",
         "the return turnaround runs on the first live return plan, only after the release"),
    prop("turnaround-after-release", "test_the_return_turnaround_runs_on_the_first_live_return_plan_only_after_the_release",
         "the live return plan waits for a pending release before it plans with the turnaround",
         [LL + "test_the_turnaround_waits_for_the_release_and_reprojects_a_changed_correction_first"],
         '                     and slam.get("pending_release") is not None)\n        ):',
         '                     and False)\n        ):'),
    kept("test_only_the_observe_and_return_trackers_hold_a_cusp_heading_miss_for_judgement",
         "only the observe and return followers hold a cusp heading miss for judgement (constants in the runner)"),
    prop("heading-hold-release", "test_only_the_observe_and_return_trackers_hold_a_cusp_heading_miss_for_judgement",
         "a judged-good held cusp heading miss is released on the follower",
         [CO + "test_a_held_cusp_heading_miss_is_judged_from_here_and_released_or_latched"],
         "hold_tracker.release_cusp(True)", "hold_tracker.fail_pending()"),
    prop("heading-hold-realign", "test_only_the_observe_and_return_trackers_hold_a_cusp_heading_miss_for_judgement",
         "a held cusp heading miss during a held realignment is decided as realign_held",
         [CO + "test_a_held_cusp_heading_miss_in_a_held_realignment_is_decided_realign_held"],
         'decision = "realign_held"', 'decision = "released"', cls="N"),
    kept("test_the_observe_replan_budget_is_per_leg_and_each_near_leg_opens_its_own",
         "the observe replan budget is per leg and each near leg opens its own (constant in the runner)"),
    prop("observe-budget-per-leg", "test_the_observe_replan_budget_is_per_leg_and_each_near_leg_opens_its_own",
         "the observe replan budget counts the replans since the leg's start",
         [CO + "test_the_observe_replan_budget_counts_only_the_replans_since_the_legs_start"],
         'len(state["observe_replans"]) - state["observe_replan_leg"]["start"]', 'len(state["observe_replans"])', cls="N"),
    prop("near-leg-opens-budget", "test_the_observe_replan_budget_is_per_leg_and_each_near_leg_opens_its_own",
         "a near leg opens its own observe replan budget",
         [NC + "test_the_first_far_observation_drives_the_near_leg_with_its_own_replan_budget"],
         '                "start": len(state["observe_replans"]),', '                "start": 0,'),
]
for test, statement in [
    ("test_the_steering_probe_runs_after_the_spawn_checks_and_ends_the_run", "the steering probe runs after the spawn checks"),
    ("test_the_probe_segments_cover_each_curvature_standing_forward_and_reverse", "the probe segments"),
    ("test_the_long_probe_set_holds_each_run_long_enough_and_comes_back", "the long probe set"),
    ("test_the_probe_window_metrics_read_curvature_residual_and_rolling_ratio", "the probe window metrics"),
    ("test_the_plant_backend_readback_uses_the_isaac_5_1_body_getters", "the plant readback getters"),
    ("test_the_cause_separation_settings_are_probe_only_and_recorded", "the cause-separation settings"),
    ("test_the_reverse_first_long_set_swaps_each_pair_and_keeps_the_segments", "the reverse-first long set"),
    ("test_the_encoder_model_feeds_the_odometry_increments_and_is_part_of_the_contract", "the encoder model wiring"),
    ("test_the_pocket_check_is_refused_with_a_team_spec_encoder_model", "the pocket check refused with a team-spec encoder"),
    ("test_the_tracker_book_keeps_every_switch_with_its_owner_and_version", "the tracker book"),
    ("test_the_runner_stamps_every_leg_switch_and_the_plant_record_carries_friction", "leg-switch stamping, plant friction"),
    ("test_the_odometry_wheel_radius_is_the_odometrys_own_and_part_of_the_contract", "the odometry wheel radius"),
    ("test_the_obstacle_layers_age_table_must_match_the_runs_odometry", "the age table must match the odometry"),
    ("test_the_boundary_record_closes_an_exit_tick_with_its_branches_and_the_propagating_failure", "the record's exit tick"),
]:
    PROPERTIES.append(kept(test, statement, RUNNER))
