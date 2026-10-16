# M-a2 — 실행기 소스 문자열 시험을 코어 동작 시험으로 (2026-10-16, 2판)

상위 계획: `docs/plans/2026-10-15-m-a-mission-core.md` 의 M-a2 행 — "남은 실행기 소스 문자열 시험을 코어 동작 시험으로 옮김(지키는 성질 목록 유지)", 합격 "성질 목록 대조표, 빠진 성질 없음". M-a1 은 2026-10-16 범위를 제한한 판정으로 닫혔다(`docs/plans/2026-10-16-m-a1-replay.md`).

**2판에서 바뀐 것(Codex 적대 검토 P2 4·P3 1, agy 독립 검토 — 7 절)**: 대상을 문자열 위치가 아니라 **소스를 읽는 모든 시험의 모든 assert** 로 동결한다; 성질마다 변이 증거(기존 시험이든 새 시험이든)를 남긴다; 구조 성질(S)·실행기 연결·순서 성질(R)을 따로 둔다; 보존 게이트를 production 코드 수정 전체로 넓힌다; 알려진 실패는 nodeid 로 고정한다. 규모는 1판 추정(시험 52 개, 2–3 일)보다 크다 — assert 693 개를 성질로 묶고 성질마다 변이를 돌린다.

## 1. 대상 — assert 단위 동결

`tools/ma2_assertions.py` 가 `tests/` 에서 실행기(`sim/isaac/run_transport.py`)나 코어(`src/forklift_core/mission/core.py`) 소스를 읽는 시험을 찾는다 — 시험 본문이 소스 경로를 직접 쓰거나, 그 경로로 만든 모듈 상수(`SCRIPT`, `CORE`, 파싱한 `TREE` …)나 그것을 쓰는 모듈 보조 함수(`_calls()` 같은, 전이적으로)를 쓰면 대상이다(1판의 문자열 위치 분류가 놓친 `index()`/`count()`/부정 조건·보조 함수 경유 시험 — Codex P2-1). 기준 revision `cda3ee2` 에서 13 개 파일, 시험 142 개, **assert 693 개**. 목록과 그 digest 를 `docs/validation/2026-10-16-m-a2-assertions.json` 으로 동결한다(재생성하면 같은 결과가 나와야 한다 — agy A1). 소스 경로를 자료로만 쓰는 시험(예: 비교 도구가 실행기 파일명을 문자열로 다루는 경우)도 목록에 들어오며, 대조표에서 X 로 사유를 적는다.

## 2. 방법

1. **성질 ID**: assert 하나하나를 성질 ID 에 대응시킨다(여러 assert 가 한 성질, 한 assert 가 여러 성질일 수 있다). 주석으로 흡수된 기대식(예: `02e4a1b` 에서 주석이 된 `pocket['planning_pickup'] = target_`)처럼 원래 assert 였던 성질은 그 시험의 git 이력에서 찾아 같은 방식으로 등록한다(Codex P2-2).
2. **분류**: 성질마다 하나 —
   - **D** 기존 동작 시험이 지킨다; **N** 새 동작 시험(`MissionCore` 나 코어 모듈 함수를 가짜 포트·훅으로 돌려 포트 호출과 인자, 기록, 전이, 명령을 본다 — 문자열·AST 대조 없음);
   - **R** 실행기 쪽 연결·순서·초기화(인자 → 코어 옵션, 실행기에 남은 상수, "프로필 적용 뒤 설정 기록" 같은 파일 간 호출 순서, 실행기 `drive()` 에 남은 초기 후보 선택·실패 처리) — 실행기 시험으로 남긴다(소스·AST 여도 된다; 실행기에 남은 미션 로직이면 그 성질을 R 로 적고 사유를 남긴다 — Codex P2-1);
   - **S** 구조 성질(모든 호출 지점의 정적 금지 — 예: 계획 호출에 참값 인자가 없다, 코어가 무엇을 import 하지 않는다) — AST 검사를 유지하고 사유를 적는다(agy 3);
   - **X** 더 이상 성질이 아니다(옮겨져 없어진 문자열, 자료 용도) — 사유.
3. **변이 증거(D·N 전부, R·S 는 기존 시험 그대로면 생략)**: 성질마다 원 assert ID, 적용 조건·입력, 정확한 pytest nodeid(매개변수 포함), 시험 안 assert 위치, 그 성질을 깨는 구체적 변이(대상 파일·바꾼 내용), 정상 통과와 변이 실패 결과(변이에서 **그 성질의 assert** 가 실패해야 한다 — import·구문 오류는 증거가 아니다), 검증한 위반 유형을 남긴다. 변이는 `tools/ma2_mutate.py` 가 작업 사본에 적용해 지정 시험만 돌리고 결과를 대조표에 기록한다(Codex P2-3, agy A3).
4. **대조표**: `tests/ma2_property_map.json` — 성질(ID, 문장, 분류, 원 assert ID 들, 새 위치 nodeid, 변이 증거). 검사 시험이 ① 동결 목록의 모든 assert ID 가 어떤 성질에 대응되고, ② 중복·미해결이 없고, ③ D/N 에 변이 증거가 있고, ④ 적힌 nodeid 가 존재함을 확인한다. **검사기 자신의 시험**: 대조표에서 항목 하나를 빼거나 증거를 지우면 검사기가 실패하는지 본다(agy 12). 의미상 완전성(성질 문장이 assert 를 다 담는가)은 리뷰(Codex)로 판정한다.
5. **원 시험 정리**: 성질이 모두 옮겨진 원 시험만 지운다; C? 처럼 섞인 시험은 R·S 부분만 남기고, 남긴 시험을 단독으로 돌려 통과를 확인한다(agy 7).
6. **동작 보존 게이트(production 코드를 바꿀 때 — 코어·실행기·공유 보조 함수 모두, Codex P2-4)**: 시험만 옮기는 것이 원칙이다. 시험을 쓰려고 production 코드를 추출·변경하면 ① 그 변경의 모든 호출자·분기·옵션·예외 경로를 목록으로 적고 각각을 동작 시험이 덮는지 확인(공유 객체 동일성, 조회 시점, 호출 순서, 예외 직전 부분 상태 포함; 새 보조 함수만 시험하지 않는다 — 호출자가 그것을 쓰는지도), ② ws1 전체 시험, ③ M-a1 재생 29 건(`docs/validation/2026-10-16-m-a1-replay-manifest.json`) 재통과 — 기록 밖 경로(탈출·인증 인출 등)는 ①이 맡는다.

## 3. 합격

- 동결된 assert 693 개가 모두 성질에 대응되고, 모든 성질이 D/N/R/S/X 중 하나이며 D/N 은 변이 증거가 있다(검사 시험 통과, 검사기 자체 시험 통과).
- ws1 전체 시험: 기준 revision `cda3ee2` 의 알려진 실패(nodeid 목록 — 착수 때 ws1 에서 다시 확인해 `docs/validation/2026-10-16-m-a2-known-failures.txt` 로 고정, 실패 양상 포함)와 **같은 집합**이고 새 실패가 없다(Codex P3, agy 10).
- `core.py` 줄·분기 커버리지가 줄지 않는다(줄면 사유) — 원 소스 시험은 커버리지를 만들지 않으므로 주로 새 시험이 늘린다(agy A7).
- production 코드를 바꿨다면 2 절 6 의 게이트 통과.
- 최종 증거: 실행 revision·변경분, 명령·잡 ID, 수집/실행/skip 수, 성질별 변이 결과, 재생 결과.

## 4. 1 판 분류 표(참고 — 2 판은 1 절의 assert 동결 목록이 정본)

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

- **① 직접 재계산**: 1 판 분류(118·24·28·33·33)와 코어 동작 시험 188 개를 스크립트를 다시 돌려 같은 결과로 확인했다. 2 판의 동결 목록(13 파일·142 시험·693 assert, 기준 `cda3ee2`)은 `tools/ma2_assertions.py` 로 냈고, Codex 가 1 판에서 빠졌다고 짚은 세 시험(`test_the_retry_limit_is_checked_after_the_same_path_test`, `test_every_plan_sees_the_grid_world_without_props`, `test_no_plan_takes_a_ground_truth_argument`)이 모두 들어 있음을 확인했다.
- **② Codex 적대 검토(1 판, 새 스레드)**: P1 없음, P2 4·P3 1 — 문자열 위치 분류의 누락과 오분류(보조 함수 경유·`index()`·실행 결과를 보는 시험), 검사기가 성질 누락을 못 잡음(assert 동결과 이력의 주석 흡수 기대식), D 의 검출력 요구와 대표 변이만의 불일치(실행기 순서를 뒤집는 변이에 기존 코어 시험이 통과한 반례), 보존 게이트가 재생 29 건뿐(호출자·분기·예외 경로 감사와 production 전체로 확대), 알려진 실패는 nodeid 로. 모두 반영(1–3 절).
- **③ agy 독립 검토(Claude Opus 4.6, 1 판만 보여 줌)**: 적히지 않은 전제 12 와 합격 조건 7. 반영 — 분류 재현성(동결 도구·digest), 구조 성질 S, D 도 변이로 증명, 섞인 시험의 남은 부분 단독 통과, 검사기 자체 시험, 알려진 실패 명시, 커버리지 감소 보고. 반영하지 않음 — 'core.py 동결'(agy 8: 다른 작업이 같은 파일을 바꾸면 대조가 낡는다)은 기준 revision 을 적고 착수 중 다른 변경이 들어오면 동결 목록을 다시 내는 것으로 대신한다.
