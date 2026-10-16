# M-a2 — 실행기 소스 문자열 시험을 코어 동작 시험으로 (2026-10-16, 1판)

상위 계획: `docs/plans/2026-10-15-m-a-mission-core.md` 의 M-a2 행 — "남은 실행기 소스 문자열 시험을 코어 동작 시험으로 옮김(지키는 성질 목록 유지)", 합격 "성질 목록 대조표, 빠진 성질 없음". M-a1 은 2026-10-16 범위를 제한한 판정으로 닫혔다(`docs/plans/2026-10-16-m-a1-replay.md`).

## 1. 대상

소스를 읽거나 소스 조각 문자열을 대조하는 시험 118 개를 기계적으로 분류했다(`ast.unparse` 로 시험 본문을 읽어, `in` 비교의 문자열 상수가 지금 어느 파일에 있는지 셈 — 분류 스크립트는 1 판 작성 때 scratchpad 에서 돌렸고 결과 표는 아래 4 절).

- **C (24)**: 문자열이 전부 `src/forklift_core/mission/core.py` 에 있다 — 이동된 미션 로직을 문자열로 지키는 시험.
- **C? (28)**: 코어와 실행기에 섞여 있거나, 코어까지 AST 로 훑는다 — 코어 쪽 성질과 실행기 연결 성질이 한 시험에 있다.
- **R (33)**: 실행기에만 있다 — Isaac·설정·탐침·플랜트·인자 연결. M-a2 범위 밖(실행기에 남는 코드의 연결 시험이므로 소스·AST 시험으로 둔다).
- **- (33)**: 문자열 대조 없이 소스를 다른 용도로 읽거나(예: drive 분할 시험) 상수만 쓴다 — 범위 밖.

M-a2 대상은 C·C? 52 개다.

## 2. 방법

1. **성질 추출**: 대상 시험마다 그 시험이 지키는 성질을 문장으로 나눈다(한 시험이 여러 성질을 지키면 여럿). 각 성질에 원 시험 이름과 어떤 문자열·AST 조건이 그것을 지켰는지를 남긴다.
2. **기존 동작 시험 대조**: M-a1 조각들이 만든 코어 동작 시험(`tests/unit/mission/test_core_*.py`, 188 개)이 이미 그 성질을 행동으로 검사하면 그 시험 이름을 적는다(D: 이미 덮임). 문자열이 같다는 것만으로 덮였다고 하지 않는다 — 그 시험이 성질을 깨뜨리는 변경에서 실패해야 한다.
3. **새 동작 시험**: 덮이지 않은 코어 성질은 `MissionCore`(또는 코어 모듈 함수)를 가짜 포트·훅으로 돌려 결과(포트 호출과 그 인자, 기록, 전이, 명령)로 검사하는 시험을 쓴다. 문자열·AST 대조는 쓰지 않는다.
4. **실행기 연결 성질**: C? 시험의 실행기 쪽 성질(인자 → 코어 옵션 연결, 실행기에 남은 상수)은 실행기 연결 시험으로 남긴다 — 소스 시험이어도 된다(코어 로직이 아니므로).
5. **대조표와 검사기**: 성질마다 (원 시험, 성질, 새 위치: D 기존 시험 / N 새 동작 시험 / R 실행기 연결 시험) 를 표로 둔다(`tests/ma2_property_map.json`). 검사 시험 하나가 ① 대상 52 개 원 시험이 모두 표에 있고, ② 표의 모든 성질에 새 위치가 있으며, ③ 그 위치의 시험이 실제로 존재하는지 확인한다. 원 시험은 표에서 모든 성질이 옮겨진 뒤에만 지운다(또는 실행기 연결 부분만 남긴다).
6. **동작 보존**: 시험만 옮기는 것이 원칙이다. 동작 시험을 쓰려고 코어에 작은 추출(예: 추종기 설정을 이름으로 돌려주는 함수)이 필요하면 호출 순서·요청이 같도록만 하고, 마지막에 M-a1 의 재생 29 건(`docs/validation/2026-10-16-m-a1-replay-manifest.json`)을 새 코드로 다시 재생해 통과를 확인한다(CPU 잡, 기록은 그대로).

## 3. 합격

- 대상 52 개 원 시험의 성질이 표에 모두 있고 각 성질이 D/N/R 중 하나로 옮겨졌다(검사 시험 통과).
- 새 동작 시험이 성질을 깨는 변경에서 실패한다 — 대표 성질마다 한 번 코어를 고의로 바꿔 실패를 확인한다(돌연변이 확인; 표에 결과를 남긴다).
- ws1 전체 시험 통과(알려진 ws1 전용 실패 10 건 외).
- M-a1 재생 29 건 재통과(코어를 손댄 경우).

## 4. 분류 표(1 판, 자동 분류)

분류 C·C? 는 문자열 위치로 정했으므로 성질 추출 때 바뀔 수 있다(예: 코어 문자열이지만 실행기 연결을 지키는 경우).

**`test_bay_planning_sweep.py`**

| 시험 | 분류 | 문자열 위치(코어/실행기/둘 다/없음) |
|---|---|---|
| `test_the_factory_mode_plans_the_runners_mission_on_the_v10_scene` | R | 0/1/0/0 |
| `test_the_runner_reads_the_shared_observation_candidates` | R | 0/1/0/0 |

**`test_core_near_field.py`**

| 시험 | 분류 | 문자열 위치(코어/실행기/둘 다/없음) |
|---|---|---|
| `test_a_correction_point_opens_waits_for_a_standing_observation_and_installs_the_redraw` | C | 1/0/0/0 |
| `test_a_correction_from_the_roof_detector_or_another_hold_is_refused_into_a_terminal_stop` | C | 1/0/0/0 |
| `test_a_pickup_retry_closes_the_episode_releases_and_stands_in_observe` | C | 1/0/1/0 |
| `test_the_runners_traced_redraw_returns_its_dry_runs_trajectories_in_call_order_and_is_one_planner_call` | R | 0/1/0/0 · AST |

**`test_isaac_pocket_stop.py`**

| 시험 | 분류 | 문자열 위치(코어/실행기/둘 다/없음) |
|---|---|---|
| `test_the_runner_hands_the_models_target_to_the_controller` | - | 0/0/0/2 |

**`test_isaac_transport_cli.py`**

| 시험 | 분류 | 문자열 위치(코어/실행기/둘 다/없음) |
|---|---|---|
| `test_isaac_transport_help_does_not_require_simulator_sdk` | R | 0/2/0/0 |
| `test_return_home_is_opt_in_and_documented_in_usage` | - | 0/0/0/2 |
| `test_isaac_transport_rejects_unsupported_camera_rate_before_startup` | R | 0/1/0/1 |
| `test_extra_views_require_video_and_perception_before_startup` | - | 0/0/0/0 |
| `test_g2_rerun_options_validate_before_startup` | - | 0/0/0/0 |
| `test_quarter_options_validate_before_simulator_startup` | R | 0/1/0/1 |
| `test_isaac_record_encoder_preserves_numpy_numbers_and_arrays` | - | 0/0/0/0 |
| `test_camera_inset_without_perception_is_rejected_before_startup` | R | 0/1/0/0 |
| `test_camera_inset_keeps_the_perception_prior_loaded` | - | 0/0/0/1 |
| `test_t11_configuration_reaches_sdk_startup` | R | 0/3/0/1 |
| `test_epal_yaml_rejects_t11_envelope_before_startup` | R | 0/1/0/1 |
| `test_noncentred_pallet_is_rejected_before_startup` | R | 0/1/0/1 |
| `test_t11_named_boxes_reject_90_degree_swap_before_startup` | - | 0/0/0/2 |
| `test_slam_feedback_options_validate_before_startup` | - | 0/0/0/1 |
| `test_every_physics_step_after_start_goes_through_step_world` | R | 0/2/0/2 · AST |
| `test_slam_release_replans_both_travel_legs_and_gates_on_a_stop` | C? | 3/1/2/0 · AST |
| `test_slam_runs_see_the_pallet_again_before_the_final_straight` | C | 3/0/0/0 |
| `test_the_delivery_match_inputs_are_recorded_without_stopping_the_match` | C | 3/0/0/0 |
| `test_slam_runs_dock_on_a_scan_before_the_delivery_straight` | C? | 2/1/0/0 |
| `test_d8b_recording_options_validate_before_startup` | - | 0/0/0/1 |
| `test_d8b_every_read_is_recorded_before_the_d5_check_can_drop_it` | C? | 3/6/0/0 · AST |
| `test_d8b_custom_cruise_drives_the_straight_but_not_the_acceptance_dry_run` | C | 1/0/0/0 |
| `test_d8b_dwells_hold_the_steering_as_well_as_the_wheels` | C | 6/0/0/1 |
| `test_d8b_dwells_lock_the_wheels_and_give_their_gains_back` | C? | 1/5/0/0 |
| `test_s4_safety_stops_take_the_dwell_path_without_the_wheel_lock_and_record_the_command` | C | 3/0/1/1 · AST |
| `test_s4_near_tracking_options_validate_before_startup` | - | 0/0/0/0 |
| `test_s4_near_tracking_runs_only_behind_the_flag_and_before_the_arrival` | C? | 5/2/3/8 · AST |
| `test_s4c1_injections_bias_only_the_captures_before_the_tracker_and_drop_before_it` | C? | 2/1/0/2 · AST |
| `test_s4c2_a_terminal_mismatch_or_refusal_retries_once_from_a_fresh_far_capture` | C | 8/0/0/5 · AST |
| `test_s4b_correction_points_are_decided_before_the_stop_and_install_at_once` | C? | 3/4/0/6 · AST |
| `test_s4c3_stage_one_redraws_a_refused_near_capture_before_ending` | C | 4/0/0/0 · AST |
| `test_s4c3_stage_two_realigns_once_on_the_shared_release_and_records_every_install` | C | 3/0/0/4 · AST |

**`test_isaac_transport_planning_config.py`**

| 시험 | 분류 | 문자열 위치(코어/실행기/둘 다/없음) |
|---|---|---|
| `test_all_runner_planning_calls_use_the_recorded_config` | C | 1/0/0/0 · AST |
| `test_trackers_rebuilt_after_detection_use_the_same_tolerance_rules` | C? | 0/0/0/0 · AST |
| `test_the_re_observation_tracker_uses_the_repositioning_tolerances` | C? | 0/0/0/0 · AST |
| `test_every_runner_tracker_releases_gear_change_cusps_the_same_way` | C? | 0/0/0/0 · AST |
| `test_every_runner_tracker_accepts_small_overshoot_except_insertion` | - | 0/0/0/0 · AST |
| `test_every_runner_tracker_reads_the_same_optional_speed_caps` | - | 0/0/0/0 · AST |
| `test_path_record_keeps_the_interval_through_the_straight_tail` | - | 0/0/0/0 · AST |
| `test_every_observation_candidate_record_keeps_its_search_attempts` | C? | 0/0/1/0 · AST |
| `test_the_20260921_profile_restores_the_old_observation_heading` | - | 0/0/0/0 |
| `test_runtime_viewpoints_extend_the_default_list_before_any_candidate_is_planned` | C? | 1/2/0/0 |
| `test_runtime_viewpoints_only_follow_the_default_bay_list` | R | 0/1/0/1 · AST |
| `test_runtime_viewpoints_see_the_pallet_only_as_an_unlabelled_rectangle` | - | 0/0/0/0 · AST |
| `test_runtime_viewpoints_need_both_the_default_list_and_the_bay` | - | 0/0/0/0 · AST |
| `test_the_replan_waits_for_a_stop_and_keeps_the_tracker_rules` | C? | 5/2/3/5 |
| `test_the_extended_ladder_is_a_second_pass_for_the_first_observation_only` | C? | 0/1/0/0 · AST |
| `test_the_legacy_mount_keeps_the_recorded_camera_exactly` | R | 0/4/0/2 |
| `test_both_cameras_use_the_same_parent_and_orientation` | R | 0/3/0/1 |
| `test_the_carriage_mount_is_guarded_and_checked` | R | 0/4/0/0 |
| `test_the_detector_sees_rounded_depth_only_when_asked_and_attempts_record_the_mount` | C? | 4/1/0/0 · AST |
| `test_same_path_waits_do_not_count_as_retries` | C | 3/0/1/0 |
| `test_wheel_target_brakes_at_the_stopping_model_deceleration` | C? | 2/1/1/3 |
| `test_the_retry_limit_is_checked_after_the_same_path_test` | - | 0/0/0/0 |
| `test_loose_arrival_only_at_observation_waypoints` | C? | 0/0/3/0 |
| `test_a_failed_live_plan_stands_and_retries` | C | 3/0/0/0 |
| `test_a_failed_blocked_replan_stands_and_retries` | C | 4/0/0/0 |
| `test_a_failed_replan_backs_off_a_bounded_distance_then_replans` | C? | 9/5/1/0 |
| `test_new_obstacle_rules_get_the_leg_direction_the_pallet_face_and_can_silence_the_depth_camera` | C? | 0/3/2/0 |
| `test_pocket_bars_are_judged_against_the_block_columns_not_the_outline` | R | 0/5/0/0 |
| `test_a_backoff_offers_no_leg_to_the_rules_and_silence_is_traced` | C? | 2/2/0/0 |
| `test_pocket_bars_meet_the_pallet_solids_exactly` | - | 0/0/0/2 |
| `test_the_n3_straight_follows_the_leg_direction` | R | 0/2/0/0 |
| `test_grid_kwargs_never_reads_slam_before_it_exists` | - | 0/0/0/0 · AST |
| `test_slam_ref_lives_only_inside_grid_kwargs` | - | 0/0/0/0 · AST |
| `test_the_pickup_retraction_covers_the_estimate_and_the_range_noise` | C | 2/0/0/0 |
| `test_plans_project_with_the_grids_correction_and_a_release_passes_its_own` | C | 1/0/2/0 · AST |
| `test_replans_record_their_path_before_the_mission_plan_exists` | C | 1/0/0/2 |
| `test_every_path_replacement_restarts_the_no_progress_baseline` | C | 4/0/0/0 |
| `test_d7c_docking_branches_are_inert_without_the_flag` | C | 4/0/0/0 |
| `test_d7c_reapproach_is_under_the_permission_and_kept_by_obstacle_replans` | C | 2/0/0/0 |
| `test_a_destination_prior_error_moves_planning_only` | R | 0/3/0/0 |
| `test_d7c_docking_needs_slam` | R | 0/1/0/0 |
| `test_obstacle_recovery_v4_escapes_before_the_clearance_zero_replan` | C? | 11/1/0/0 |
| `test_obstacle_recovery_v4_an_escape_end_bypasses_docking_stall_and_failure` | C | 6/0/0/0 |
| `test_obstacle_recovery_v4_releases_a_hold_at_a_backoff_end_and_in_its_recovery` | C? | 14/2/0/0 |
| `test_cusp_position_stall_v7_is_wired_into_the_runner` | C? | 31/5/2/0 |
| `test_near_protection_prediction_screens_only_the_last_correction` | C? | 7/1/0/0 |
| `test_the_return_turnaround_runs_on_the_first_live_return_plan_only_after_the_release` | C | 2/0/1/0 |
| `test_only_the_observe_and_return_trackers_hold_a_cusp_heading_miss_for_judgement` | C? | 10/4/0/0 |
| `test_the_observe_replan_budget_is_per_leg_and_each_near_leg_opens_its_own` | C? | 1/2/0/2 |
| `test_the_steering_probe_runs_after_the_spawn_checks_and_ends_the_run` | C? | 0/4/1/0 |
| `test_the_probe_segments_cover_each_curvature_standing_forward_and_reverse` | - | 0/0/0/0 |
| `test_the_plant_backend_readback_uses_the_isaac_5_1_body_getters` | - | 0/0/0/2 |
| `test_the_cause_separation_settings_are_probe_only_and_recorded` | R | 0/1/0/4 |
| `test_the_encoder_model_feeds_the_odometry_increments_and_is_part_of_the_contract` | R | 0/8/0/0 |
| `test_the_pocket_check_is_refused_with_a_team_spec_encoder_model` | R | 0/2/0/1 |
| `test_the_runner_stamps_every_leg_switch_and_the_plant_record_carries_friction` | R | 0/3/0/2 |
| `test_the_odometry_wheel_radius_is_the_odometrys_own_and_part_of_the_contract` | R | 0/9/0/0 |
| `test_the_obstacle_layers_age_table_must_match_the_runs_odometry` | R | 0/2/0/2 |
| `test_the_boundary_record_closes_an_exit_tick_with_its_branches_and_the_propagating_failure` | R | 0/1/0/0 |

**`test_isaac_truth_audit.py`**

| 시험 | 분류 | 문자열 위치(코어/실행기/둘 다/없음) |
|---|---|---|
| `test_every_plan_sees_the_grid_world_without_props` | R | 0/1/0/0 · AST |
| `test_the_planner_pickup_obstacle_is_the_zone_prior_or_the_estimate` | C? | 0/1/1/0 · AST |
| `test_the_delivery_docking_sweep_uses_the_grid_when_grid_planning` | C | 1/0/0/0 |
| `test_runtime_viewpoints_cannot_join_grid_planning` | R | 0/1/0/0 |
| `test_grid_planning_plans_on_perception_only` | R | 0/1/0/0 |

**`test_replay_setup.py`**

| 시험 | 분류 | 문자열 위치(코어/실행기/둘 다/없음) |
|---|---|---|
| `test_drive_inputs_are_its_prologue_names` | - | 0/0/0/0 |
| `test_an_absent_container_service_stays_none_in_the_snapshot` | R | 0/1/0/0 |

**`test_runner_drive_split.py`**

| 시험 | 분류 | 문자열 위치(코어/실행기/둘 다/없음) |
|---|---|---|
| `test_the_runner_module_imports_with_isaac_refused` | - | 0/0/0/0 |
| `test_drive_imports_nothing_from_isaac_and_run_hands_over_at_the_initial_plans` | - | 0/0/0/0 · AST |
| `test_drive_reads_no_isaac_helper_names` | - | 0/0/0/0 · AST |
| `test_the_end_state_projection_covers_the_containers_by_logical_service_and_hides_the_output` | R | 0/1/0/0 |
| `test_the_stdout_observer_records_printed_lines_as_sink_echo_and_still_prints` | - | 0/0/0/0 |
| `test_a_print_whose_end_changed_is_a_different_sink_payload` | - | 0/0/0/0 |
| `test_main_closes_the_record_with_what_left_run_and_the_harness_does_the_same` | R | 0/2/0/2 |
| `test_the_projection_encodes_cpu_state_by_value_refuses_unknown_objects_and_hides_both_output_roots` | - | 0/0/0/0 |
| `test_main_writes_failure_txt_outside_the_record_and_capture_images_are_sinks` | R | 0/1/0/0 |
| `test_the_projection_handles_the_slam_container_and_refuses_a_socket` | - | 0/0/0/0 |
| `test_service_owned_slam_state_is_compared_by_id_and_native_objects_refuse` | - | 0/0/0/0 |
| `test_a_c_backed_generator_is_projected_by_its_state_and_foreign_objects_refuse` | - | 0/0/0/0 |
| `test_object_identities_in_the_containers_do_not_reach_the_projection` | - | 0/0/0/0 |

## 5. 순서

주제별 묶음으로 진행하고 묶음마다 커밋한다: ① 계획·추종기 설정(계획 설정 기록, 추종기 허용치, cusp 해제 규칙), ② 관찰(후보·사다리·런타임 시점·검출 입력), ③ 장애물 복구(재계획·후진·탈출·대기), ④ 근접·D7 도킹, ⑤ 기록(경로 기록, 추종기 장부, 참값 감사), ⑥ CLI 통합 시험의 코어 쪽 성질.

## 6. 자체 검토

(검토 뒤 채운다.)
