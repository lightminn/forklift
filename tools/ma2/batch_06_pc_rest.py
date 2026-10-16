"""M-a2 batch 6: the planning-config tests left (runner setup: runtime viewpoints, path_record; the 2026-09-21
profile: runner order R, the core helper D)."""

PC = "tests/unit/test_isaac_transport_planning_config.py::"
RS = "tests/unit/mission/test_core_realign_and_setup.py::"
CORE = "src/forklift_core/mission/core.py"
RUNNER = "the code it guards stays in the runner (setup of the runtime viewpoints, path_record)"


def kept(test, statement, reason=RUNNER):
    return {"id": f"B6:R:{test}", "class": "R", "checks": [PC + test + "::*"], "statement": statement,
            "tests": [PC + test], "reason": reason}


PROPERTIES = [
    kept("test_path_record_keeps_the_interval_through_the_straight_tail", "path_record keeps the analytic interval"),
    kept("test_runtime_viewpoints_extend_the_default_list_before_any_candidate_is_planned",
         "runtime viewpoints extend the default list once, before any candidate is planned"),
    kept("test_runtime_viewpoints_only_follow_the_default_bay_list", "runtime viewpoints only follow the default bay list"),
    kept("test_runtime_viewpoints_see_the_pallet_only_as_an_unlabelled_rectangle",
         "runtime viewpoints see the pallet only as an unlabelled rectangle"),
    kept("test_runtime_viewpoints_need_both_the_default_list_and_the_bay", "runtime viewpoints need the default list and the bay"),
    kept("test_the_20260921_profile_restores_the_old_observation_heading",
         "the 2026-09-21 profile is applied before the configs are recorded and the core is built (runner order)",
         reason="the profile's application order in the runner's setup (Codex: a cross-file order a core test cannot see)"),
    {"id": "B6:profile-observe-heading", "class": "D",
     "checks": [PC + "test_the_20260921_profile_restores_the_old_observation_heading::*"],
     "statement": "the 2026-09-21 profile restores the old observation heading tolerance (0.03 rad)",
     "tests": [RS + "test_the_tracker_profile_and_its_record_are_shared_by_the_runner_and_the_core"],
     "mutation": {"file": CORE, "find": "tracker.config = replace(tracker.config, yaw_tolerance_rad=0.03)",
                  "replace": "tracker.config = replace(tracker.config, yaw_tolerance_rad=0.05)"}},
]
