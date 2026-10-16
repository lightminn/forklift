"""M-a2 batch 0: tests that read the runner/core files but guard no moved mission logic (X), and runner tests that
stay with the runner (R): behaviour tests of runner functions, the runner's argument checks, the replay
infrastructure, the planning-grid preparation that stays in the runner's drive(), and the tools that must match a
runner constant."""

T = "tests/unit/"
I = "tests/integration/"


def whole(file, test, cls, reason, statement):
    return {"id": f"B0:{test}", "statement": statement, "class": cls, "checks": [f"{file}::{test}::*"],
            "tests": [] if cls == "X" else [f"{file}::{test}"], "reason": reason}


X_DATA = "the runner's file name is a key or hash input here (provenance comparison); no runner or core property"
PROPERTIES = []
for test in ("test_runs_that_differ_only_in_wall_time_paths_snapshot_and_nondeterministic_reads_compare_equal",
             "test_a_decision_difference_is_reported_with_its_path", "test_arrays_compare_bit_for_bit_and_missing_files_count"):
    PROPERTIES.append(whole(T + "tools/test_mission_ab_compare.py", test, "X", X_DATA, "A/B comparison tool behaviour"))
for test in ("test_the_full_matrix_has_no_problems", "test_only_the_manifest_files_may_differ_and_only_at_their_reviewed_hashes",
             "test_arguments_and_settings_must_match_the_d8b_run_of_the_same_seed_and_speed",
             "test_files_named_by_arguments_are_compared_by_content", "test_a_required_input_missing_on_either_or_both_sides_is_refused",
             "test_files_a_config_names_as_values_are_followed_and_required", "test_paths_mentioned_only_in_comments_are_not_inputs",
             "test_recorded_lidar_factory_and_scene_must_match_the_d8b_run", "test_the_near_field_files_are_not_compared_with_the_d8b_run",
             "test_a_run_outside_the_plan_matrix_makes_the_result_diagnostic", "test_a_stop_with_a_command_left_on_or_no_restart_is_refused"):
    PROPERTIES.append(whole(T + "tools/test_s4_drive_terms.py", test, "X", X_DATA, "S4 drive-terms judge behaviour"))
for test in ("test_a_run_record_passes_its_gates", "test_a_tampered_depth_fails_the_hash_gate",
             "test_an_unverified_truth_chain_makes_the_configuration_undecidable",
             "test_a_recorded_mount_that_is_not_the_named_one_fails_the_mount_gate"):
    PROPERTIES.append(whole(T + "tools/test_diagnose_detection.py", test, "X", X_DATA, "detection diagnosis gates"))
PROPERTIES.append(whole(T + "tools/test_d8b_observations.py",
                        "test_every_unobserved_read_resets_the_handoff_and_both_chains_lose_the_same_reads", "X",
                        "inspect.getsource of a tool module (observe_run), not of the runner or the core",
                        "the D8b observation tool's handoff reset"))

R_RUNNER_FN = "a behaviour test of a function that stays in the runner (loaded from the runner module); no moved core logic"
for test in ("test_getter_accepts_matching_intrinsics_and_records_readback_path",
             "test_g1a_compares_and_records_raw_and_integer_index_conventions",
             "test_render_product_mismatch_is_refused_even_when_camera_cache_matches",
             "test_render_product_readback_missing_fails_closed", "test_getter_refuses_out_of_tolerance_and_retains_failure",
             "test_getter_refuses_either_resolution_mismatch", "test_getter_accepts_values_just_inside_limits",
             "test_source_hashes_cover_transitive_core_and_preserve_repository_paths",
             "test_getter_limits_are_inclusive_at_exact_authored_bounds", "test_getter_v11_boundary_in_both_directions"):
    PROPERTIES.append(whole(T + "test_isaac_camera_contract.py", test, "R", R_RUNNER_FN, "the runner's camera contract"))
for test in ("test_control_failure_brakes_and_measures_stop_before_reraising",
             "test_zero_setpoint_without_physical_stopping_is_recorded_as_failure"):
    PROPERTIES.append(whole(T + "test_isaac_approach_stop.py", test, "R", R_RUNNER_FN, "the runner's approach stop"))
R_TOOL = "a tool's list must match the runner's default observation list (a runner constant); the check stays as written"
PROPERTIES += [
    whole(T + "tools/test_observation_viewpoint_eval.py", "test_the_fixed_list_matches_the_runner_default", "R", R_TOOL,
          "the viewpoint tool's fixed list is the runner default"),
    whole(T + "tools/test_observation_candidate_design.py", "test_the_runner_appends_the_chosen_candidates_after_the_original_five",
          "R", R_TOOL, "the runner appends the chosen candidates after the original five"),
    whole(T + "tools/test_bay_planning_sweep.py", "test_the_runner_reads_the_shared_observation_candidates", "R", R_TOOL,
          "the runner reads the shared observation candidates"),
]
R_REPLAY = "the replay infrastructure of the runner (drive split, projection, observers): runner code, tested as such"
for test in ("test_the_runner_module_imports_with_isaac_refused",
             "test_drive_imports_nothing_from_isaac_and_run_hands_over_at_the_initial_plans",
             "test_drive_reads_no_isaac_helper_names",
             "test_the_end_state_projection_covers_the_containers_by_logical_service_and_hides_the_output",
             "test_the_stdout_observer_records_printed_lines_as_sink_echo_and_still_prints",
             "test_a_print_whose_end_changed_is_a_different_sink_payload",
             "test_main_closes_the_record_with_what_left_run_and_the_harness_does_the_same",
             "test_the_projection_encodes_cpu_state_by_value_refuses_unknown_objects_and_hides_both_output_roots",
             "test_main_writes_failure_txt_outside_the_record_and_capture_images_are_sinks",
             "test_the_projection_handles_the_slam_container_and_refuses_a_socket",
             "test_service_owned_slam_state_is_compared_by_id_and_native_objects_refuse",
             "test_a_c_backed_generator_is_projected_by_its_state_and_foreign_objects_refuse",
             "test_object_identities_in_the_containers_do_not_reach_the_projection"):
    PROPERTIES.append(whole(T + "test_runner_drive_split.py", test, "R", R_REPLAY, "runner replay infrastructure"))
PROPERTIES.append(whole(T + "test_replay_setup.py", "test_drive_inputs_are_its_prologue_names", "R", R_REPLAY,
                        "the setup snapshot names are drive's prologue names"))
R_GRID = ("the planning-grid preparation (grid_kwargs) stays in the runner's drive() as part of the Planner port; "
          "these tests already run it as behaviour through a harness")
for test in ("test_grid_kwargs_runs_before_the_drive_loop_and_with_a_slam_map",
             "test_grid_kwargs_takes_an_end_region_for_its_own_cells_and_the_return_band",
             "test_a_map_swapped_after_it_was_opened_is_judged_by_the_file_read"):
    PROPERTIES.append(whole(T + "test_isaac_obstacle_layer.py", test, "R", R_GRID, "the runner's planning-grid preparation"))
R_CLI = "the runner's command-line validation before the simulator starts (runner argument checks, run as behaviour)"
for test in ("test_isaac_transport_help_does_not_require_simulator_sdk", "test_return_home_is_opt_in_and_documented_in_usage",
             "test_isaac_transport_rejects_unsupported_camera_rate_before_startup",
             "test_extra_views_require_video_and_perception_before_startup", "test_g2_rerun_options_validate_before_startup",
             "test_quarter_options_validate_before_simulator_startup", "test_isaac_record_encoder_preserves_numpy_numbers_and_arrays",
             "test_camera_inset_without_perception_is_rejected_before_startup", "test_camera_inset_keeps_the_perception_prior_loaded",
             "test_t11_configuration_reaches_sdk_startup", "test_epal_yaml_rejects_t11_envelope_before_startup",
             "test_noncentred_pallet_is_rejected_before_startup", "test_t11_named_boxes_reject_90_degree_swap_before_startup",
             "test_slam_feedback_options_validate_before_startup", "test_d8b_recording_options_validate_before_startup",
             "test_s4_near_tracking_options_validate_before_startup"):
    PROPERTIES.append(whole(I + "test_isaac_transport_cli.py", test, "R", R_CLI, "the runner's argument validation"))
