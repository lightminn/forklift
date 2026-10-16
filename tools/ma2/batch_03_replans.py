"""M-a2 batch 3: cusp replan, observation ladder, cameras, same-path waits, live/blocked replans, backoff,
new-obstacle rules (test_isaac_transport_planning_config.py, second group)."""

PC = "tests/unit/test_isaac_transport_planning_config.py::"
M = "tests/unit/mission/"
CO = M + "test_core_cusp_observe.py::"
BR = M + "test_core_blocked_replan.py::"
LL = M + "test_core_live_leg.py::"
CA = M + "test_core_command_assembly.py::"
CAP = M + "test_core_capture.py::"
PERM = M + "test_core_permission.py::"
OF = M + "test_core_observe_failed.py::"
CORE = "src/forklift_core/mission/core.py"
WIRING = ("walks the runner and the core together: the runner's own sites, constants and cross-file order stay "
          "guarded by this test as written; the core's behaviour is guarded by the D/N properties beside it")
RUNNER = "the code it guards stays in the runner (camera mounts, new-obstacle rules, the truth evaluator, same_path)"


def kept(test, statement, reason=WIRING):
    return {"id": f"B3:R:{test}", "class": "R", "checks": [PC + test + "::*"], "statement": statement,
            "tests": [PC + test], "reason": reason}


def core(pid, test, statement, guards, find, replace, count=1, checks=None, cls="D"):
    return {"id": f"B3:{pid}", "class": cls, "checks": checks or [PC + test + "::*"], "statement": statement,
            "tests": guards, "mutation": {"file": CORE, "find": find, "replace": replace, "count": count}}


PROPERTIES = [
    kept("test_only_a_transport_heading_failure_at_a_cusp_is_replanned",
         "only a transport heading failure at a cusp, on the path, within budget (or D7-owned), is replanned"),
    core("cusp-only-at-cusp", "test_only_a_transport_heading_failure_at_a_cusp_is_replanned",
         "a transport heading failure away from a cusp is not replanned",
         [CO + "test_only_a_transport_heading_failure_at_a_cusp_on_the_path_is_replanned"],
         'and tracking.failure == "endpoint_heading"\n            and tracking.at_cusp\n            and not tracking.off_path\n            and (len(state["cusp_replans"])',
         'and tracking.failure == "endpoint_heading"\n            and not tracking.off_path\n            and (len(state["cusp_replans"])', cls="N"),
    kept("test_the_replan_waits_for_a_stop_and_keeps_the_tracker_rules",
         "the cusp replan brakes until stopped for 12 ticks, then plans the transport leg with the leg's tracker rules"),
    core("cusp-waits-12", "test_the_replan_waits_for_a_stop_and_keeps_the_tracker_rules",
         "the cusp replan waits 12 standing ticks before planning",
         [CO + "test_a_transport_cusp_miss_brakes_until_stopped_for_12_ticks_then_replans_the_leg"],
         "cusp_stop_ticks < cusp_stop_needed", "cusp_stop_ticks < 1"),
    core("cusp-brakes-while-moving", "test_the_replan_waits_for_a_stop_and_keeps_the_tracker_rules",
         "while not stopped the cusp miss keeps braking",
         [CO + "test_a_transport_cusp_miss_brakes_until_stopped_for_12_ticks_then_replans_the_leg"],
         'replace(tracking, status="braking")', 'replace(tracking, status="failed")'),
    kept("test_the_extended_ladder_is_a_second_pass_for_the_first_observation_only",
         "the extended ladder is a second pass of the first observation only; re-observation stays on the earlier ladder"),
    core("reobserve-earlier-ladder", "test_the_extended_ladder_is_a_second_pass_for_the_first_observation_only",
         "the re-observation plans its candidates on the earlier ladder",
         [OF + "test_the_first_reachable_candidate_is_installed_as_the_observe_leg"],
         "                extended=False,\n                **grid_kwargs(\"observe\"),\n            )\n            state[\"observation_candidates\"].append(",
         "                extended=True,\n                **grid_kwargs(\"observe\"),\n            )\n            state[\"observation_candidates\"].append("),
    kept("test_the_legacy_mount_keeps_the_recorded_camera_exactly", "the legacy mount keeps the recorded camera", RUNNER),
    kept("test_both_cameras_use_the_same_parent_and_orientation", "the cameras share the mount parent and orientation", RUNNER),
    kept("test_the_carriage_mount_is_guarded_and_checked", "the carriage mount is lift-guarded and checked", RUNNER),
    kept("test_the_detector_sees_rounded_depth_only_when_asked_and_attempts_record_the_mount",
         "both detection paths round the depth only when asked; attempts record the mount and the quantisation"),
    core("detector-quantize", "test_the_detector_sees_rounded_depth_only_when_asked_and_attempts_record_the_mount",
         "the observe arrival's detection sees the depth rounded by the run's setting",
         [CAP + "test_the_rear_axle_sits_behind_the_base_along_any_heading_and_the_detector_sees_rounded_depth"],
         "scene_input, options.depth_quantize_mm", "scene_input, 0"),
    core("attempt-records-quantize", "test_the_detector_sees_rounded_depth_only_when_asked_and_attempts_record_the_mount",
         "an observation attempt records the depth quantisation",
         [CAP + "test_the_capture_is_recorded_dumped_and_its_accepted_pose_is_the_capture_base"],
         'attempt["depth_quantize_mm"] = options.depth_quantize_mm', 'attempt["depth_quantize_mm"] = 0'),
    kept("test_same_path_is_the_remaining_part_again", "same_path is the remaining part of the current path again", RUNNER),
    core("same-path-not-counted", "test_same_path_waits_do_not_count_as_retries",
         "a same-path wait never counts towards the replan limits", [BR + "test_same_path_waits_between_new_paths_never_reach_the_replan_limits"],
         'and not r.get("same_path")', "and True", count=2, cls="N"),
    core("failed-escape-not-counted", "test_same_path_waits_do_not_count_as_retries",
         "a failed replan and an escape never count towards the replan limits (Codex M-a2 review P2)",
         [BR + "test_failed_and_escape_replans_never_count_towards_the_replan_limits"],
         'and not r.get("failed") and not r.get("escape")', "", count=2, cls="N"),
    core("same-path-before-limit", "test_the_retry_limit_is_checked_after_the_same_path_test",
         "the replan limits are checked only after the same-path test (a same-path wait never ends the run)",
         [BR + "test_the_same_path_is_judged_before_the_replan_limits"],
         "        elif same_path(replanned, paths[ms.phase], trackers[ms.phase].remaining_to_goal_m()):",
         '        elif len([r for r in obstacle["replans"] if r["phase"] == ms.phase and t - r["time_s"] < 30.0'
         ' and not r.get("same_path") and not r.get("failed") and not r.get("escape")]) <= 3'
         " and same_path(replanned, paths[ms.phase], trackers[ms.phase].remaining_to_goal_m()):", cls="N"),
    kept("test_wheel_target_brakes_at_the_stopping_model_deceleration",
         "the wheel target brakes at the stopping model's deceleration, the creep is slewed, the cap and stops come after"),
    core("wheel-brake-decel", "test_wheel_target_brakes_at_the_stopping_model_deceleration",
         "the slewed wheel target slows at the stopping model's deceleration",
         [CA + "test_the_wheel_speed_is_slewed_capped_by_the_permission_and_stepped_to_zero_by_a_stop"],
         'rate = (obstacle["layer"].permission.config.stopping.decel_mps2', "rate = (COMMAND_ACCEL_MPS2"),
    core("loose-observe-arrival", "test_loose_arrival_only_at_observation_waypoints",
         "an observation waypoint accepts a loose arrival when standing at the path end",
         [M + "test_core_command.py::test_an_observe_leg_standing_off_its_end_within_tolerance_arrives_after_a_second"],
         "and tracking.position_error_m <= OBSERVE_ARRIVAL_M", "and tracking.position_error_m <= -1.0"),
    core("loose-arrival-observe-only", "test_loose_arrival_only_at_observation_waypoints",
         "only an observation leg arrives loosely (Codex M-a2 review P2)",
         [M + "test_core_command.py::test_the_loose_arrival_is_only_for_an_observation_leg"],
         'ms.phase == "observe" and tracking.status == "tracking"', 'tracking.status == "tracking"', cls="N"),
    core("live-plan-retries", "test_a_failed_live_plan_stands_and_retries",
         "a failed live plan stands, retries after a second and refuses on the fifth",
         [LL + "test_a_failed_live_plan_retries_after_a_second_and_refuses_on_the_fifth"], "tries < 5", "tries < 6"),
    core("live-plan-retry-second", "test_a_failed_live_plan_stands_and_retries",
         "a failed live plan retries after one second", [LL + "test_a_failed_live_plan_retries_after_a_second_and_refuses_on_the_fifth"],
         'live_retry_after_s"] = t + 1.0', 'live_retry_after_s"] = t + 3.0'),
    kept("test_a_failed_blocked_replan_stands_and_retries",
         "a failed blocked replan stands and retries; the live leg runs before the command assembly"),
    core("blocked-replan-fifth", "test_a_failed_blocked_replan_stands_and_retries",
         "a failed blocked replan refuses on the fifth failure",
         [BR + "test_a_failed_replan_waits_backs_off_against_the_leg_and_refuses_on_the_fifth_failure"], "failures < 5", "failures < 6"),
    kept("test_a_failed_replan_backs_off_a_bounded_distance_then_replans",
         "a failed replan backs off 0.30 m at 0.15 m/s, twice per leg, against the leg, then replans"),
    core("backoff-against-leg", "test_a_failed_replan_backs_off_a_bounded_distance_then_replans",
         "the backoff drives against the leg's direction",
         [BR + "test_a_failed_replan_waits_backs_off_against_the_leg_and_refuses_on_the_fifth_failure"],
         "d = -1 if leg_dir >= 0 else 1", "d = 1 if leg_dir >= 0 else -1"),
    core("backoff-budget", "test_a_failed_replan_backs_off_a_bounded_distance_then_replans",
         "the backoff budget is per leg", [BR + "test_a_failed_replan_waits_backs_off_against_the_leg_and_refuses_on_the_fifth_failure"],
         "backs.get(ms.phase, 0) < BACKOFF_PER_LEG", "backs.get(ms.phase, 0) < 99"),
    kept("test_new_obstacle_rules_get_the_leg_direction_the_pallet_face_and_can_silence_the_depth_camera",
         "the new-obstacle rules get the leg direction and the pallet face and can silence the depth camera", RUNNER),
    kept("test_pocket_bars_are_judged_against_the_block_columns_not_the_outline",
         "pocket bars are judged against the block columns, not the outline", RUNNER),
    kept("test_a_backoff_offers_no_leg_to_the_rules_and_silence_is_traced",
         "a backoff offers no leg to the rules and the sensor silence is traced"),
    core("silence-traced", "test_a_backoff_offers_no_leg_to_the_rules_and_silence_is_traced",
         "the sensor silence trace records when the limit first went to zero",
         [PERM + "test_sensor_silence_ends_the_run_after_one_second_standing"],
         'trace.setdefault("first_zero_s", t)', 'trace.setdefault("first_zero_s", -1.0)'),
    kept("test_pocket_bars_meet_the_pallet_solids_exactly", "pocket bars meet the pallet solids exactly", RUNNER),
    kept("test_the_n3_straight_follows_the_leg_direction", "the N3 straight follows the leg direction", RUNNER),
]
