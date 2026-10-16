"""M-a2 batch 8: properties the Codex implementation review (rounds 1-2) found held only by an R test or by a
mutation that did not break them -- each with a behaviour test that observes it and a mutation that breaks exactly
it."""

PC = "tests/unit/test_isaac_transport_planning_config.py::"
CLI = "tests/integration/test_isaac_transport_cli.py::"
M = "tests/unit/mission/"
BR, CO, NF = M + "test_core_blocked_replan.py::", M + "test_core_cusp_observe.py::", M + "test_core_near_field.py::"
CORE = "src/forklift_core/mission/core.py"
SAME = "test_same_path_waits_do_not_count_as_retries"
COUNT = BR + "test_failed_and_escape_replans_never_count_towards_the_replan_limits"
W30 = 'and not r.get("same_path") and not r.get("failed") and not r.get("escape")]'
LEG = 'and not r.get("failed") and not r.get("escape"))'


def prop(pid, checks, statement, guards, find, replace):
    return {"id": f"B8:{pid}", "class": "N", "checks": checks, "statement": statement, "tests": guards,
            "mutation": {"file": CORE, "find": find, "replace": replace, "count": 1}}


PROPERTIES = [
    prop("30s-skips-failed", [PC + SAME + "::*"], "a failed replan is not a 30 s replan",
         [COUNT + "[30 s-failed]"], W30, 'and not r.get("same_path") and not r.get("escape")]'),
    prop("30s-skips-escape", [PC + SAME + "::*"], "an escape is not a 30 s replan",
         [COUNT + "[30 s-escape]"], W30, 'and not r.get("same_path") and not r.get("failed")]'),
    prop("leg-skips-failed", [PC + SAME + "::*"], "a failed replan is not a leg replan",
         [COUNT + "[leg-failed]"], LEG, 'and not r.get("escape"))'),
    prop("leg-skips-escape", [PC + SAME + "::*"], "an escape is not a leg replan",
         [COUNT + "[leg-escape]"], LEG, 'and not r.get("failed"))'),
    prop("cusp-replan-transport-only", [PC + "test_only_a_transport_heading_failure_at_a_cusp_is_replanned::*"],
         "only the transport replans a heading failure at a cusp",
         [CO + "test_the_cusp_heading_replan_is_the_transports_only"],
         '            tracking.status == "failed"\n            and ms.phase == "transport"\n            and tracking.failure == "endpoint_heading"',
         '            tracking.status == "failed"\n            and ms.phase in ("transport", "return_home")\n            and tracking.failure == "endpoint_heading"'),
    prop("cusp-replan-keeps-config", [PC + "test_the_replan_waits_for_a_stop_and_keeps_the_tracker_rules::*"],
         "the cusp replan follows the new path with the leg's own tracker config",
         [CO + "test_the_cusp_replan_keeps_the_legs_tracker_config"],
         '                    hooks.show_path(replanned, "Transport", (1.0, 0.65, 0.04))\n'
         "                    trackers[ms.phase] = RearAxlePathTracker(\n"
         "                        replanned.poses,\n                        replanned.directions,\n"
         "                        replanned.curvatures_inv_m,\n                        trackers[ms.phase].config,",
         '                    hooks.show_path(replanned, "Transport", (1.0, 0.65, 0.04))\n'
         "                    trackers[ms.phase] = RearAxlePathTracker(\n"
         "                        replanned.poses,\n                        replanned.directions,\n"
         "                        replanned.curvatures_inv_m,\n                        TrackerConfig(),"),
    prop("no-escape-in-reapproach", [PC + "test_obstacle_recovery_v4_escapes_before_the_clearance_zero_replan::*"],
         "no escape on the transport during a docking re-approach",
         [BR + "test_no_escape_on_the_transport_after_a_docking_match_or_during_a_reapproach[docking0]"],
         '(slam["docking"].get("reapproach") or slam["docking"].get("accepted_any"))',
         '(slam["docking"].get("accepted_any"))'),
    prop("no-escape-after-match", [PC + "test_obstacle_recovery_v4_escapes_before_the_clearance_zero_replan::*"],
         "no escape on the transport after an accepted docking match",
         [BR + "test_no_escape_on_the_transport_after_a_docking_match_or_during_a_reapproach[docking1]"],
         '(slam["docking"].get("reapproach") or slam["docking"].get("accepted_any"))',
         '(slam["docking"].get("reapproach"))'),
    prop("retry-observe-after-release",
         [CLI + "test_s4c2_a_terminal_mismatch_or_refusal_retries_once_from_a_fresh_far_capture::*"],
         "a pickup retry moves to observe only after the release and stand",
         [NF + "test_a_pickup_retry_moves_to_observe_only_after_the_release_and_stand"],
         '        retry_.update(core.release_and_stand(now_s, "release_pickup_retry"))\n'
         '        state["near_tracking"]["retry"] = retry_\n        near["stand_record"] = "retry"\n'
         '        transition("observe", now_s)',
         '        transition("observe", now_s)\n'
         '        retry_.update(core.release_and_stand(now_s, "release_pickup_retry"))\n'
         '        state["near_tracking"]["retry"] = retry_\n        near["stand_record"] = "retry"'),
]

# The core sweep (tools/ma2_core_sweep.py on 91f77aa, ws1 job 3131): 188 core places the kept source tests read,
# 180 killed by the behaviour tests; the eight survivors below each got a behaviour test.
CR, CA, NC = M + "test_core_cusp_rules.py::", M + "test_core_command.py::", M + "test_core_near_capture.py::"
CUSP_WIRED = [PC + "test_cusp_position_stall_v7_is_wired_into_the_runner::*"]
PROPERTIES += [
    prop("observe-replan-installs-path", [PC + "test_the_observe_replan_budget_is_per_leg_and_each_near_leg_opens_its_own::*"],
         "an observe replan installs its path",
         [CO + "test_an_observe_heading_miss_brakes_and_once_stopped_replans_the_leg_to_the_same_target"],
         '                paths["observe"] = replanned\n                state.setdefault("paths", {})["observe"]',
         '                pass\n                state.setdefault("paths", {})["observe"]'),
    prop("near-tick-phases",
         [PC + "test_new_obstacle_rules_get_the_leg_direction_the_pallet_face_and_can_silence_the_depth_camera::*"],
         "the near tick runs only in the approach and the insertion",
         [CA + "test_the_near_tick_runs_only_in_the_approach_and_the_insertion[transport]",
          CA + "test_the_near_tick_runs_only_in_the_approach_and_the_insertion[observe]"],
         'if near is not None and near["tracker"] is not None and ms.phase in ("approach", "insert"):',
         'if near is not None and near["tracker"] is not None and True:'),
    prop("near-leg-follower", [PC + "test_only_the_observe_and_return_trackers_hold_a_cusp_heading_miss_for_judgement::*"],
         "the near leg is followed by a new observe follower on its path",
         [NC + "test_the_first_far_observation_drives_the_near_leg_with_its_own_replan_budget"],
         "            trackers[\"observe\"] = RearAxlePathTracker(\n                near_leg.poses,\n"
         "                near_leg.directions,\n                near_leg.curvatures_inv_m,\n"
         "                trackers[\"observe\"].config,\n            )\n",
         "            pass\n"),
    prop("cusp-sensor-unknown-only", CUSP_WIRED, "a short verified walk refuses only at unknown or edge",
         [CR + "test_the_sensor_check_counts_only_unknown_or_edge_as_short_evidence_and_ages_the_free_cells"],
         'unknown_near_ = plain_.blocked in ("unknown", "edge") and plain_.verified_m < stop_need_',
         "unknown_near_ = True and plain_.verified_m < stop_need_"),
    prop("cusp-sensor-evidence-age", CUSP_WIRED, "FREE evidence past the permission's age limit refuses",
         [CR + "test_the_sensor_check_counts_only_unknown_or_edge_as_short_evidence_and_ages_the_free_cells"],
         "and age_ <= pconf_.evidence_max_age_s and", "and True and"),
    prop("cusp-return-cusp-not-recovered", CUSP_WIRED, "a return cusp is never recovered by the stall replan",
         [CR + "test_a_return_cusp_is_never_recovered_by_the_stall_replan"],
         'if ms.phase in ("transport", "return_home") and goal and slam is not None:',
         'if ms.phase in ("transport", "return_home") and True and slam is not None:'),
    prop("cusp-replan-pending-this-leg", CUSP_WIRED, "the cusp monitor sees a pending replan of this leg only",
         [CR + "test_the_cusp_monitor_sees_a_pending_replan_of_this_leg_only[obstacle_state1-False]"],
         'obstacle.get("force_replan") == ms.phase or', 'obstacle.get("force_replan") is not None or'),
    prop("cusp-replan-pending-wait", CUSP_WIRED, "a waiting replan of this leg is a pending replan",
         [CR + "test_the_cusp_monitor_sees_a_pending_replan_of_this_leg_only[obstacle_state2-True]"],
         'or bool(obstacle.get("replan_wait", {}).get(ms.phase))', "or False"),
]

# Codex implementation review round 3 (b0ea955): a strengthened condition, a boundary value and an order -- kinds
# the first sweep did not generate (tools/ma2_core_sweep.py --operators adds them).
REL = M + "test_core_release.py::"
PROPERTIES += [
    prop("return-leg-escapes", [PC + "test_obstacle_recovery_v4_escapes_before_the_clearance_zero_replan::*"],
         "the return leg looks for an escape too", [BR + "test_the_return_leg_escapes_an_invalid_start_too"],
         'replanned.status == "invalid_start" and ms.phase in ("transport", "return_home")',
         'replanned.status == "invalid_start" and ms.phase == "transport"'),
    prop("small-jump-bound-m", [CLI + "test_slam_release_replans_both_travel_legs_and_gates_on_a_stop::*"],
         "a 20 mm jump is still small", [REL + "test_a_jump_of_exactly_20_mm_or_0_02_rad_is_still_small[jump0]"],
         "jump_m <= 0.02 and abs(jump_rad) <= 0.02 and not docked_return",
         "jump_m < 0.02 and abs(jump_rad) <= 0.02 and not docked_return"),
    prop("small-jump-bound-rad", [CLI + "test_slam_release_replans_both_travel_legs_and_gates_on_a_stop::*"],
         "a 0.02 rad jump is still small", [REL + "test_a_jump_of_exactly_20_mm_or_0_02_rad_is_still_small[jump1]"],
         "jump_m <= 0.02 and abs(jump_rad) <= 0.02 and not docked_return",
         "jump_m <= 0.02 and abs(jump_rad) < 0.02 and not docked_return"),
    prop("release-reprojects-before-carrying",
         [CLI + "test_s4c2_a_terminal_mismatch_or_refusal_retries_once_from_a_fresh_far_capture::*"],
         "the release re-projects the grid before the estimates ride along",
         [REL + "test_a_release_whose_reprojection_fails_has_not_carried_the_estimates_yet"],
         "        reprojected_ = None\n        if obstacle is not None and slam[\"tracker\"].applied is not None:\n",
         "        reprojected_ = None\n"
         "        if obstacle is not None and obstacle.get(\"pickup_estimate\") is not None:\n"
         "            obstacle[\"pickup_estimate\"] = replace(obstacle[\"pickup_estimate\"],"
         " x_m=obstacle[\"pickup_estimate\"].x_m + frame_change[0])  # carried before the refresh\n"
         "        if obstacle is not None and slam[\"tracker\"].applied is not None:\n"),
]
