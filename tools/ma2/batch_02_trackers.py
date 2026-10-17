"""M-a2 batch 2: planning config and follower rules (test_isaac_transport_planning_config.py, first group).

Tests that walk both the runner and the core stay as R (they still guard the runner's own sites and the cross-file
wiring); the core's sites get behaviour tests (N) with mutations."""

PC = "tests/unit/test_isaac_transport_planning_config.py::"
OS = "tests/unit/mission/test_core_observe_succeeded.py::"
OF = "tests/unit/mission/test_core_observe_failed.py::"
BR = "tests/unit/mission/test_core_blocked_replan.py::"
CORE = "src/forklift_core/mission/core.py"
WIRING = ("walks the runner and the core together: the runner's own construction sites and the cross-file wiring "
          "stay guarded by this test as written; the core's sites are behaviour-tested (the N properties beside it)")


def kept(test, statement):
    return {"id": f"B2:R:{test}", "class": "R", "checks": [PC + test + "::*"], "statement": statement,
            "tests": [PC + test], "reason": WIRING}


PROPERTIES = [
    kept("test_all_runner_planning_calls_use_the_recorded_config",
         "the runner builds the planner config once, records it, and every planning call (runner and core) uses it"),
    {"id": "B2:mission-plan-config", "class": "N",
     "checks": [PC + "test_all_runner_planning_calls_use_the_recorded_config::*"],
     "statement": "the core's mission plan after a detection uses the run's planner config object (Codex M-a2 review P2)",
     "tests": ["tests/unit/mission/test_core_observe_succeeded.py::test_the_mission_is_planned_with_the_recorded_planner_config_itself"],
     "mutation": {"file": "src/forklift_core/mission/core.py",
                  "find": "        plans = plan_transport(\n            grid_world(scenario),\n            planner_config,\n",
                  "replace": "        plans = plan_transport(\n            grid_world(scenario),\n"
                             "            replace(planner_config, clearance_m=0.0),\n"}},
    kept("test_trackers_rebuilt_after_detection_use_the_same_tolerance_rules",
         "followers built per stage take 0.03/0.05 (observe), 0.008/0.03 (return) and 0.008/0.02 otherwise"),
    kept("test_the_re_observation_tracker_uses_the_repositioning_tolerances",
         "the re-observation follower uses the repositioning tolerances"),
    kept("test_every_runner_tracker_releases_gear_change_cusps_the_same_way",
         "every follower releases gear-change cusps at 0.03 m / 0.05 rad and brakes at 8 mm"),
    kept("test_every_runner_tracker_accepts_small_overshoot_except_insertion",
         "every follower accepts a 0.03 m overshoot except insertion"),
    kept("test_every_runner_tracker_reads_the_same_optional_speed_caps",
         "every follower reads the optional speed caps from the settings"),
    kept("test_every_observation_candidate_record_keeps_its_search_attempts",
         "every observation candidate record keeps its search attempts, start and status"),
    {"id": "B2:N:detection-follower-tolerances", "class": "N",
     "checks": [PC + "test_trackers_rebuilt_after_detection_use_the_same_tolerance_rules::*"],
     "statement": "the followers built after a detection: 0.008 m everywhere, 0.03 rad on the return, 0.02 rad otherwise",
     "tests": [OS + "test_every_follower_built_after_a_detection_takes_its_stages_tolerances_cusp_and_speed_rules"],
     "mutation": {"file": CORE, "find": '0.03 if name == "observe" else 0.008', "replace": '0.03 if name == "observe" else 0.009'}},
    {"id": "B2:N:detection-follower-return-yaw", "class": "N",
     "checks": [PC + "test_trackers_rebuilt_after_detection_use_the_same_tolerance_rules::*"],
     "statement": "the return follower built after a detection takes 0.03 rad",
     "tests": [OS + "test_every_follower_built_after_a_detection_takes_its_stages_tolerances_cusp_and_speed_rules"],
     "mutation": {"file": CORE, "find": '                            else 0.03\n                            if name == "return_home"',
                  "replace": '                            else 0.02\n                            if name == "return_home"'}},
    {"id": "B2:N:reobserve-follower-position", "class": "N",
     "checks": [PC + "test_the_re_observation_tracker_uses_the_repositioning_tolerances::*"],
     "statement": "the re-observation follower repositions to 0.03 m",
     "tests": [OF + "test_the_re_observation_follower_repositions_and_releases_cusps_like_every_follower"],
     "mutation": {"file": CORE, "find": "position_tolerance_m=0.03,\n                # Judged like a cusp in heading",
                  "replace": "position_tolerance_m=0.008,\n                # Judged like a cusp in heading"}},
    {"id": "B2:N:cusp-release-rule", "class": "N",
     "checks": [PC + "test_every_runner_tracker_releases_gear_change_cusps_the_same_way::*"],
     "statement": "the core's followers brake and judge gear-change cusps at 8 mm",
     "tests": [OS + "test_every_follower_built_after_a_detection_takes_its_stages_tolerances_cusp_and_speed_rules",
               OF + "test_the_re_observation_follower_repositions_and_releases_cusps_like_every_follower"],
     "mutation": {"file": CORE, "find": "cusp_brake_window_m=0.008,", "replace": "cusp_brake_window_m=0.02,", "count": 2}},
    {"id": "B2:N:overshoot-except-insert", "class": "N",
     "checks": [PC + "test_every_runner_tracker_accepts_small_overshoot_except_insertion::*"],
     "statement": "the followers built after a detection accept a 0.03 m overshoot except insertion",
     "tests": [OS + "test_every_follower_built_after_a_detection_takes_its_stages_tolerances_cusp_and_speed_rules"],
     "mutation": {"file": CORE, "find": "else 0.03,\n                        stop_speed_mps",
                  "replace": "else 0.05,\n                        stop_speed_mps"}},
    {"id": "B2:N:speed-caps-from-settings", "class": "N",
     "checks": [PC + "test_every_runner_tracker_reads_the_same_optional_speed_caps::*"],
     "statement": "the core's followers read the optional speed caps from the settings (absent = off)",
     "tests": [OS + "test_every_follower_built_after_a_detection_takes_its_stages_tolerances_cusp_and_speed_rules"],
     "mutation": {"file": CORE, "find": "max_reverse_speed_mps=settings.get(", "replace": "max_reverse_speed_mps=0.1 or settings.get(",
                  "count": 2}},
    {"id": "B2:N:hold-phases-pending", "class": "N",
     "checks": [PC + "test_trackers_rebuilt_after_detection_use_the_same_tolerance_rules::*"],
     "statement": "only the hold phases' followers hold a cusp heading miss for judgement",
     "tests": [OS + "test_every_follower_built_after_a_detection_takes_its_stages_tolerances_cusp_and_speed_rules"],
     "mutation": {"file": CORE, "find": "CUSP_HEADING_HOLD_RAD\n                        if name in CUSP_HEADING_HOLD_PHASES",
                  "replace": "CUSP_HEADING_HOLD_RAD\n                        if True"}},
    {"id": "B2:N:reobserve-candidate-record", "class": "N",
     "checks": [PC + "test_every_observation_candidate_record_keeps_its_search_attempts::*"],
     "statement": "a re-observation candidate record keeps its start, status and searches",
     "tests": [OF + "test_every_re_observation_candidate_record_keeps_its_start_status_and_searches"],
     "mutation": {"file": CORE, "find": '                    "start_rear": rear.tolist(),\n', "replace": ""}},
    {"id": "B2:N:fallback-candidate-record", "class": "N",
     "checks": [PC + "test_every_observation_candidate_record_keeps_its_search_attempts::*"],
     "statement": "an obstacle-fallback candidate record keeps its start, status and searches",
     "tests": [BR + "test_every_obstacle_fallback_candidate_record_keeps_its_start_status_and_searches"],
     "mutation": {"file": CORE, "find": '"start_rear": [float(v) for v in rear], "success": candidate_plan.success,',
                  "replace": '"success": candidate_plan.success,'}},
]
