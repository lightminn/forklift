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
