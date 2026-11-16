# Applied values: this batch (DRY_RUN of pfb_2bea23e5, 2026-11-15) against the comparators' recorded arguments.
# Made by applied_table.py on ws1; the live DRY_RUN rows were taken before the live sessions were renamed LD_/LO_ (same arguments).

## riib (R / S)

### random_1

| 항목 | R DRY_RUN | S DRY_RUN | lcr_B0 기록 | k1c_K1 기록 | k1r_K1 기록 |
|---|---|---|---|---|---|
| deterministic | ARGS_OK | — | True | True | True |
| near_end_stand | on | — | on | on | on |
| near_observe_arrival | 0.05,0.05 | — | 0.05,0.05 | 0.05,0.05 | 0.05,0.05 |
| layer_latency_budget | config/layer_latency_budget.yaml | — | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml |
| return_cusp_replans | 2 | — | 2 | 2 | 2 |
| return_start_backoff | boxed | — | boxed | boxed | boxed |
| exit_prefix_exempt | on | — | on | on | on |
| low_clearance_guard | on | — | on | on | on |
| restart_confirm_scans | 3 | — | 3 | 3 | 3 |
| d7_reapproach_hold | on | — | on | on | on |
| docking_degenerate_continue | off | — | — | — | — |
| lidar_keepout_m | 0 | — | — | — | — |
| encoder_model | team_spec_quantized | — | team_spec_quantized | team_spec_quantized | team_spec_quantized |
| path_display | (러너 기본) | — | — | — | overlay |
| far_capture_expected | (러너 기본) | — | off | off | off |
| near_capture_lower | waive | — | waive | waive | waive |
| near_capture_redraw | always | — | always | always | always |
| near_capture_redraw_lateral_m | 0.10 | — | 0.1 | 0.1 | 0.1 |
| recognition_proposals | learned | — | oracle | oracle | oracle |
| recognition_pixel_lag_steps | 24 | — | 0 | 0 | 0 |
| live_factor | (러너 기본) | — | 1.0 | 1.0 | 1.0 |

R riib_defaults.txt: `MODE=slam NEAR=on MARKERS=on RACK=on KEEPOUT=0 NOISE=const START_BAND=off START_BAND_M=0.03 ESCAPE_PRESTEER=off ESCAPE_START=off LOW_CLEARANCE=on RESTART_CONFIRM=3 D7_HOLD=on EXIT_EXEMPT=on RETURN_START=boxed ENCODER_MODEL=team_spec_quantized GYRO=off LAYER_BUDGET=config/layer_latency_budget.yaml SCENE_FILE=/home/projects/forklift/test_runs/riibotics/scenes_p1c_81a007b1/random_1_m.json / NEAR_END_STAND=on NEAR_END_STAND_FROM=launcher / NEAR_OBSERVE_ARRIVAL=0.05,0.05 NEAR_OBSERVE_ARRIVAL_FROM=launcher / RETURN_CUSP_REPLANS=2 RETURN_CUSP_REPLANS_FROM=launcher / DOCKING_DEGENERATE_CONTINUE=off DOCKING_DEGENERATE_CONTINUE_FROM=launcher / VIEWS=none / VIDEO=off / SLAM=mapping (riib_scene.sbatch maps online; no Riibotics demo scene has an accepted survey bundle)`

### line_2

| 항목 | R DRY_RUN | S DRY_RUN | lcr_B0 기록 | k1c_K1 기록 | k1r_K1 기록 |
|---|---|---|---|---|---|
| deterministic | ARGS_OK | ARGS_OK | True | True | True |
| near_end_stand | on | on | on | on | on |
| near_observe_arrival | 0.05,0.05 | 0.05,0.05 | 0.05,0.05 | 0.05,0.05 | 0.05,0.05 |
| layer_latency_budget | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml |
| return_cusp_replans | 2 | 2 | 2 | 2 | 2 |
| return_start_backoff | boxed | boxed | boxed | boxed | boxed |
| exit_prefix_exempt | on | on | on | on | on |
| low_clearance_guard | on | on | on | on | on |
| restart_confirm_scans | 3 | 3 | 3 | 3 | 3 |
| d7_reapproach_hold | on | on | on | on | on |
| docking_degenerate_continue | on | on | on | on | on |
| lidar_keepout_m | 0 | 0 | — | — | — |
| encoder_model | team_spec_quantized | team_spec_quantized | team_spec_quantized | team_spec_quantized | team_spec_quantized |
| path_display | (러너 기본) | (러너 기본) | — | — | overlay |
| far_capture_expected | (러너 기본) | (러너 기본) | off | off | off |
| near_capture_lower | waive | waive | waive | waive | waive |
| near_capture_redraw | always | always | always | always | always |
| near_capture_redraw_lateral_m | 0.10 | 0.10 | 0.1 | 0.1 | 0.1 |
| recognition_proposals | learned | learned | oracle | oracle | oracle |
| recognition_pixel_lag_steps | 24 | 24 | 0 | 0 | 0 |
| live_factor | (러너 기본) | (러너 기본) | 1.0 | 1.0 | 1.0 |

R riib_defaults.txt: `MODE=slam NEAR=on MARKERS=on RACK=on KEEPOUT=0 NOISE=const START_BAND=off START_BAND_M=0.03 ESCAPE_PRESTEER=off ESCAPE_START=off LOW_CLEARANCE=on RESTART_CONFIRM=3 D7_HOLD=on EXIT_EXEMPT=on RETURN_START=boxed ENCODER_MODEL=team_spec_quantized GYRO=off LAYER_BUDGET=config/layer_latency_budget.yaml SCENE_FILE=/home/projects/forklift/test_runs/riibotics/scenes_p1c_81a007b1/line_2_m.json / NEAR_END_STAND=on NEAR_END_STAND_FROM=launcher / NEAR_OBSERVE_ARRIVAL=0.05,0.05 NEAR_OBSERVE_ARRIVAL_FROM=launcher / RETURN_CUSP_REPLANS=2 RETURN_CUSP_REPLANS_FROM=launcher / DOCKING_DEGENERATE_CONTINUE=on DOCKING_DEGENERATE_CONTINUE_FROM=launcher / VIEWS=none / VIDEO=off / SLAM=mapping (riib_scene.sbatch maps online; no Riibotics demo scene has an accepted survey bundle)`

### aisle_rack

| 항목 | R DRY_RUN | S DRY_RUN | lcr_B0 기록 | k1c_K1 기록 | k1r_K1 기록 |
|---|---|---|---|---|---|
| deterministic | ARGS_OK | ARGS_OK | True | True | True |
| near_end_stand | on | on | on | on | on |
| near_observe_arrival | 0.05,0.05 | 0.05,0.05 | 0.05,0.05 | 0.05,0.05 | 0.05,0.05 |
| layer_latency_budget | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml |
| return_cusp_replans | 2 | 2 | 2 | 2 | 2 |
| return_start_backoff | boxed | boxed | boxed | boxed | boxed |
| exit_prefix_exempt | on | on | on | on | on |
| low_clearance_guard | on | on | on | on | on |
| restart_confirm_scans | 3 | 3 | 3 | 3 | 3 |
| d7_reapproach_hold | on | on | on | on | on |
| docking_degenerate_continue | off | off | — | — | — |
| lidar_keepout_m | 0.15 | 0.15 | 0.15 | 0.15 | 0.15 |
| encoder_model | team_spec_quantized | team_spec_quantized | team_spec_quantized | team_spec_quantized | team_spec_quantized |
| path_display | (러너 기본) | (러너 기본) | — | — | overlay |
| far_capture_expected | (러너 기본) | (러너 기본) | off | off | off |
| near_capture_lower | waive | waive | waive | waive | waive |
| near_capture_redraw | always | always | always | always | always |
| near_capture_redraw_lateral_m | 0.10 | 0.10 | 0.1 | 0.1 | 0.1 |
| recognition_proposals | learned | learned | oracle | oracle | oracle |
| recognition_pixel_lag_steps | 24 | 24 | 0 | 0 | 0 |
| live_factor | (러너 기본) | (러너 기본) | 1.0 | 1.0 | 1.0 |

R riib_defaults.txt: `MODE=slam NEAR=on MARKERS=on RACK=on KEEPOUT=0.15 NOISE=const START_BAND=off START_BAND_M=0.03 ESCAPE_PRESTEER=off ESCAPE_START=off LOW_CLEARANCE=on RESTART_CONFIRM=3 D7_HOLD=on EXIT_EXEMPT=on RETURN_START=boxed ENCODER_MODEL=team_spec_quantized GYRO=off LAYER_BUDGET=config/layer_latency_budget.yaml SCENE_FILE=/home/projects/forklift/test_runs/riibotics/scenes_p1c_81a007b1/aisle_rack.json / NEAR_END_STAND=on NEAR_END_STAND_FROM=launcher / NEAR_OBSERVE_ARRIVAL=0.05,0.05 NEAR_OBSERVE_ARRIVAL_FROM=launcher / RETURN_CUSP_REPLANS=2 RETURN_CUSP_REPLANS_FROM=launcher / DOCKING_DEGENERATE_CONTINUE=off DOCKING_DEGENERATE_CONTINUE_FROM=launcher / VIEWS=none / VIDEO=off / SLAM=mapping (riib_scene.sbatch maps online; no Riibotics demo scene has an accepted survey bundle)`

## live (L)

### line_m

| 항목 | L-D DRY_RUN | L-O DRY_RUN | cts D 기록 | cts O6 기록 |
|---|---|---|---|---|
| deterministic | True | (러너 기본) | True | False |
| near_end_stand | on | on | on | on |
| near_observe_arrival | 0.05,0.05 | 0.05,0.05 | — | — |
| layer_latency_budget | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml | — | — |
| return_cusp_replans | (러너 기본) | (러너 기본) | 2 | 2 |
| return_start_backoff | boxed | boxed | boxed | boxed |
| exit_prefix_exempt | (러너 기본) | (러너 기본) | on | on |
| low_clearance_guard | (러너 기본) | (러너 기본) | on | on |
| restart_confirm_scans | (러너 기본) | (러너 기본) | 3 | 3 |
| d7_reapproach_hold | on | on | on | on |
| encoder_model | team_spec_quantized | team_spec_quantized | team_spec_quantized | team_spec_quantized |
| far_capture_expected | on | on | on | on |
| near_capture_lower | waive_far_confirmed | waive_far_confirmed | waive_far_confirmed | waive_far_confirmed |
| near_capture_redraw | always | always | always | always |
| near_capture_redraw_lateral_m | 0.10 | 0.10 | 0.1 | 0.1 |
| recognition_proposals | learned | learned | oracle | learned |
| recognition_pixel_lag_steps | 12 | 12 | 0 | 12 |
| live_factor | 0.4 | 0.4 | 0.4 | 0.4 |
| slam_map_bundle | /home/projects/forklift/artifacts/slam_bundles/line_m_team_spec_quantized_0ff9e3bdff13 | /home/projects/forklift/artifacts/slam_bundles/line_m_team_spec_quantized_0ff9e3bdff13 | /home/projects/forklift/artifacts/slam_bundles/line_m_team_spec_quantized_0ff9e3bdff13 | /home/projects/forklift/artifacts/slam_bundles/line_m_team_spec_quantized_0ff9e3bdff13 |

L-D RIIB_DEFAULTS: `RIIB_DEFAULTS return_start_backoff=boxed near_end_stand=on near_observe_arrival=0.05,0.05`

### random_m

| 항목 | L-D DRY_RUN | L-O DRY_RUN | cts D 기록 | cts O6 기록 |
|---|---|---|---|---|
| deterministic | True | (러너 기본) | True | False |
| near_end_stand | on | on | on | on |
| near_observe_arrival | 0.05,0.05 | 0.05,0.05 | — | — |
| layer_latency_budget | config/layer_latency_budget.yaml | config/layer_latency_budget.yaml | — | — |
| return_cusp_replans | (러너 기본) | (러너 기본) | 2 | 2 |
| return_start_backoff | boxed | boxed | boxed | boxed |
| exit_prefix_exempt | (러너 기본) | (러너 기본) | on | on |
| low_clearance_guard | (러너 기본) | (러너 기본) | on | on |
| restart_confirm_scans | (러너 기본) | (러너 기본) | 3 | 3 |
| d7_reapproach_hold | on | on | on | on |
| encoder_model | team_spec_quantized | team_spec_quantized | team_spec_quantized | team_spec_quantized |
| far_capture_expected | on | on | on | on |
| near_capture_lower | waive_far_confirmed | waive_far_confirmed | waive_far_confirmed | waive_far_confirmed |
| near_capture_redraw | always | always | always | always |
| near_capture_redraw_lateral_m | 0.10 | 0.10 | 0.1 | 0.1 |
| recognition_proposals | learned | learned | oracle | learned |
| recognition_pixel_lag_steps | 12 | 12 | 0 | 12 |
| live_factor | 0.4 | 0.4 | 0.4 | 0.4 |
| slam_map_bundle | /home/projects/forklift/artifacts/slam_bundles/random_m_team_spec_quantized_0facac3eea66 | /home/projects/forklift/artifacts/slam_bundles/random_m_team_spec_quantized_0facac3eea66 | /home/projects/forklift/artifacts/slam_bundles/random_m_team_spec_quantized_0facac3eea66 | /home/projects/forklift/artifacts/slam_bundles/random_m_team_spec_quantized_0facac3eea66 |

L-D RIIB_DEFAULTS: `RIIB_DEFAULTS return_start_backoff=boxed near_end_stand=on near_observe_arrival=0.05,0.05`

## 비교 대상끼리 기록 인자 차이(장면 random_1 / line_m s0)

- lcr_B0 vs k1r_K1: path_display=None→'overlay', slam_feedback='/home/projects/forklift/test_runs/lcr/cmp_a2b91d2d/lcr_B0_random_1_s0/sock/slam.sock'→'/home/projects/forklift/test_runs/k1r/k1r_7744f285/k1r_K1_random_1_s0/sock/slam.sock', slam_map_dir='/home/projects/forklift/test_runs/lcr/cmp_a2b91d2d/lcr_B0_random_1_s0/bridge'→'/home/projects/forklift/test_runs/k1r/k1r_7744f285/k1r_K1_random_1_s0/bridge'
- k1c_K1 vs k1r_K1: path_display=None→'overlay', slam_feedback='/home/projects/forklift/test_runs/k1/cmp_c7701296/k1c_K1_random_1_s0/sock/slam.sock'→'/home/projects/forklift/test_runs/k1r/k1r_7744f285/k1r_K1_random_1_s0/sock/slam.sock', slam_map_dir='/home/projects/forklift/test_runs/k1/cmp_c7701296/k1c_K1_random_1_s0/bridge'→'/home/projects/forklift/test_runs/k1r/k1r_7744f285/k1r_K1_random_1_s0/bridge'
- cts_D vs cts_O6 (line_m): deterministic=True→False, live_port=9205→9207, live_port_file='/home/projects/forklift/test_runs/ct_survey/cts_dfa9aaf2/D_line_m_s0/live/live_port.json'→'/home/projects/forklift/test_runs/ct_survey/cts_dfa9aaf2/O6_line_m_s0/live/live_port.json', recognition_pixel_lag_steps=0→12, recognition_proposals='oracle'→'learned', slam_feedback='/home/projects/forklift/test_runs/ct_survey/cts_dfa9aaf2/D_line_m_s0/live/epoch_1/sock/slam.sock'→'/home/projects/forklift/test_runs/ct_survey/cts_dfa9aaf2/O6_line_m_s0/live/epoch_1/sock/slam.sock', slam_map_dir='/home/projects/forklift/test_runs/ct_survey/cts_dfa9aaf2/D_line_m_s0/live/epoch_1/bridge'→'/home/projects/forklift/test_runs/ct_survey/cts_dfa9aaf2/O6_line_m_s0/live/epoch_1/bridge'
