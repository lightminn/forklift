# 실시간 실행기의 상태 소유권 표 (D0, 3판)

[상시 운용 콘솔 계획](../plans/2026-10-18-persistent-live-console.md) D0 의 산출물이다. 실시간 실행에서 미션이 끝나고(D1) 다음 미션이 시작될 때(D2), 각 상태를 어떻게 처리할지 정한다.

- 기준 코드: `feat/realtime-d9` 8452120. main 9f0d215(D9 머지, PR #52)와 다시 대조했다. 그사이 바뀐 것은 GC 동결(1e53cbe)뿐이고, 그 키는 7절·8절에 넣었다.
- 줄 번호 표기: R = `sim/isaac/run_transport.py`, C = `src/forklift_core/mission/core.py`.
- 목록은 기계적 추출(각 dict 의 키 접근, `MissionCore` 속성, `ms` 속성)에 조사·Codex 지적을 더한 것이다.

## 처리 기호

| 기호 | 뜻 |
|---|---|
| **S** | 세션 소유. 유지한다. |
| **S/조각** | 세션 소유인 감사 기록. 계획 D8 의 조각 저장으로 디스크에 넘긴다. **닫힌 항목만** 불변 조각이 된다. 아직 갱신 중인 항목은 소유자가 쥐고 있다. 미션 경계와 무관하다. |
| **M0** | 미션 소유. 미션 시작 때 시작값으로 되돌린다(같은 객체를 제자리에서). |
| **M기록** | 미션 소유 기록. 종료 때 미션 기록(계획 D1 7단계)으로 넘기고 비운다. |
| **API** | 소유자의 정리 API 로 닫은 뒤 초기화한다. 내부를 직접 지우지 않는다. |
| **C** | 화물(cargo, 계획 D5) 소유. 미션이 아니라 화물 상태를 따른다. |
| **N** | 미션 시작 때 새 입력으로 다시 만든다(계획 D2 시작 처리). |

## 1. `MissionCore` 속성 (C)

| 속성 | 처리 | 근거·주의 |
|---|---|---|
| `commands`, `targets` | S | 중재기가 `commands` 객체를 쥔다(live_runtime 57). 대상 revision 은 단조 증가해야 한다. |
| `jobs` | S | 작업자. 미션 종료 때 `abandon_jobs` 로 작업만 버린다. 작업자가 죽어 있으면 교체하거나 세션 실패로 간다(U23). |
| `pending` | API | `abandon_jobs`(신규). `_drop` 은 release 재예약 부작용이 있어 그대로 쓰지 않는다(C4363). |
| `release_replan_needed` | M0 | `abandon_jobs` 뒤 **`None`**(형식 `dict | None`, C158·4445). `False` 를 넣으면 다음 `job_hold` 에서 `need["leg"]` 가 형 오류다. |
| `fresh_depth_wait` | M0 | `None`. |
| `capture_live`, `observation_live`, `capture_acquired_s` | API | `capture_abandon()` 뒤 `None`. 메인 D9 의 안내를 따른다. |
| `last_permission` | M0 | 이전 미션의 허가 결과가 다음 판단에 섞이지 않게 한다. |
| `slam_releases` | S | 보정 회계. |
| `ms`, `state`, `obstacle`, `slam`, `pocket`, `near`, `paths`, `trackers` | (아래 절) | 객체는 유지하고 내용만 처리한다. 다시 묶으면 루프 클로저와 갈라진다. |
| `options` | N | 필드 단위로 바꾼다: `scenario`, `return_to_pose`=대기 자리, `home_rear`=대기 자리, `observation_waypoints` 목록의 내용, `pickup_bounds`, **`pickup_zone`**(R4591; 커스프 검사기가 읽는다, C3244). 지역 `pickup_zone` 과 함께 바꾼다. |

## 2. `ms` (MissionState)

| 속성 | 처리 | 근거 |
|---|---|---|
| `phase`, `phase_started` | N | `observe` 또는 적재 제출이면 `transport`. `phase_started`=채택 시각. |
| `observe_stall_ticks`, `slam_stall_ticks`, `dock_stop_ticks`, `cusp_stop_ticks`, `next_candidate_index` | M0 | 0. |
| `last_tracking` | M0 | `None`. |
| `lift_command` | C | 화물이 없으면 지금 승강 높이(바닥 근처). 적재 제출이면 `cargo.lift_hold_m`. 0 으로 덮지 않는다. |

## 3. `obstacle`

| 키 | 처리 | 근거 |
|---|---|---|
| `layer` | S, 일부 API | 층 자체는 유지한다. 아래 '층 속성' 참조. |
| `version`, `applied` | S | 격자가 어느 보정으로 투영됐는지. 틱 시작 재투영의 기준이다(메인 D9). |
| `control_rear`, `last_stamp`, `last_raw`, `ticks`, `min_allowed`, `slowed`, `reasons`, `unpermitted`, `events`, `event_objects`, `run_wall_start` | S | 센서·허가 실행 상태와 집계. |
| `scans`, `plans`, `reprojections`, `silence_trace` | S/조각 | 틱마다 늘어난다. |
| `plan_cache` | S | 격자 버전 키 캐시(R3593·3651). 버전이 바뀌면 스스로 무효가 된다. |
| `pickup_estimate` | M0 | 이전 팔레트 인식값. 남으면 관찰 계획과 `blocked_replan` 이 옛 팔레트를 쓴다(C858·947·975). |
| `pallet_rel` | C | 들기 때 화물 상대 자세(C178). 화물 소유로 옮긴다. |
| `delivered_pending`, `withdraw_line` | M0 | 철수 인증 상태. 철수 회랑을 닫은 뒤 지운다. |
| `live_replanned`, `live_retry_after_s`, `live_plans` | M0 | 운반·복귀 실시간 재계획 래치와 단계별 시도 수(C460·540·548). 남으면 다음 미션에서 실시간 재계획이 다시 돌지 않는다. |
| `replans`, `replan_wait`, `escapes`, `backoffs`, `backoff`, `tight_replans`, `mission_replans`, `progress`, `blocked_ticks`, `path_blocked_ticks`, `force_replan`, `deadline`, `permission_cap` | M0 (기록은 M기록) | 단계별 예산과 감시. 단계 이름 키라서 미션이 바뀌어도 누적된다(C1027–1164·4342). |
| `dumps`, `dumped_phases`, `recovery_dumps`, `backoff_records`, `return_reprojected_at_s` | M기록 | 진단 덤프 회계. |

**층 속성(`obstacle["layer"]`)**

| 속성 | 처리 | 근거 |
|---|---|---|
| `exempt`, `depth_support` | API | 포켓 검사기의 면제와 bound method(R3278). 포켓 검사 닫기와 같은 경로로 해제한다(C268–276). |
| `corridor` | API | `end_withdrawal`(C234). |
| `known` | N, 조건부 | **`known_enabled` 일 때만** 미션 시작 때 지금 팔레트 자리의 사전 영역 하나로 둔다(R2895 와 같은 조건). 꺼져 있으면 비워 둔다. `refresh()` 는 목록이 비어 있지 않으면 적용하기 때문이다(obstacle_layer 300). 진행 중에는 인식(C4893)·들기(C176)·철수 뒤 배달(C250)이 기존대로 바꾼다. |
| `memory` | S | 관측된 장애물 기억(근거·보정·시간 포함). 미션 경계에서 일괄 삭제하지 않는다. 들기 뒤 픽업 기억 철회(C291)는 미션 진행이 한다. **주의**: `planner_grid()` 는 `known` 을 읽지 않는다(obstacle_layer 404). 그래서 배달된 팔레트 자리의 기억은 다음 미션의 계획 격자에 그대로 점유로 남는다. 고른 픽업의 처리는 도킹 띠(R3707)가 한다. 다음 미션에서 이 팔레트를 집는 계획이 이 기억 때문에 막히지 않는지는 U3 가 확인해야 할 열린 항목이다. |
| 허가 내부와 실시간 캐시 | API | `permission.snapshot`·`path_check`·`_driven_since_m`, 실시간 블록의 `live_state.snapshot`·`key`, `live_perm._cache`(BundlePermit 설치 원호). 미션 종료와 계획 D10 의 대상·적재 변경 때 무효화한다. 사후조건: 새 경로의 `refresh`/`update` 가 옛 값 없이 다시 계산된다. |
| `grid.scans`, `last_scan_s`, 난수, 그림자 `memory`·`correction`·`started` | S | 센서 기억. `shadow.started` 를 되돌리면 지금 자리에 '시작 때 빈 띠' 전제가 다시 들어간다. |

## 4. `slam`

| 키 | 처리 | 근거 |
|---|---|---|
| `link`, `async_link`, `module`, `noise`, `odometry`, `odom_*`, `scan_id`, `tracker`, `tracker_speed`, `stop`, `stop_now`, `stale`, `silence_dead_reckoning_s`, `last_command`, `last_scan_*`, `last_sent_ranges` | S | 링크·주행 기록계·정지 감지·마지막 센서 값. 메인 D9 가 유지하라고 한 것을 포함한다. |
| `holds` | S | hold 일화 기록이자 `hold_count()` 의 근거(C4368). 지우지 않는다. |
| `records`, `control` | S/조각 | 스캔·틱마다 늘어난다(메인 의견). |
| `pending_release`, `release_wait_from` | API | 종료용 해제(계획 D1 4단계)가 처리한 뒤 지운다. hold 를 유지하는 경우(포크 관계 `constrained`)는 남겨 두고 복구가 처리한다. |
| `docking` | N | `{"status": "pending", …}`. 배달 뒤 `done`/`fallback` 으로 남으면 `arm_docking` 이 일찍 돌아간다(C3308). |
| `reference_points` 와 도킹 참조값 | N | 새 목적지에서 참조 스캔을 다시 뜬다(R5475–5508). |
| `transport_scenario`, `return_scenario`, `transport_config` | M0(삭제) | `options.scenario` 보다 먼저 읽힌다(C513·873·1403…). 읽는 곳은 대부분 `.get` 이다. 직접 읽기 둘(C3428·3932)은 같은 미션의 도킹 1회차 `setdefault`(C3422) 뒤에만 실행되므로, 미션 시작 때 지워도 안전하다(구현 때 시험으로 고정). |

## 5. `near` (근접 추적)

| 키 | 처리 | 근거 |
|---|---|---|
| `bounds`, `bounds_data`, `terms`, `blades`, `blade_y_max_m`, `carriage_limit_m`, `corners`, `fork_tip_x_m` | S | 설정·기하. |
| `episode` | S | 일화 번호. 단조 증가로 둔다. 프레임 기록이 이 번호로 나뉜다(R3405·3438). |
| `tracker`, `start`, `stop_state`, `correction`, `path_revision`, `first_target`, `retries`, `retry_stand`, `mismatch`, `unbounded`, `protect`, `arrival`, `drop_latch`, `stale_waits`, `last_stamp`, `last_fed_stamp`, `injection_closed`, `stand_record`, `terminal_standing_since` | M0 | 남은 `tracker` 는 다음 접근에서 `near_active()` 를 참으로 만든다(C2883). |
| `history`, `commands` | S(짧은 이력) | 명령 창·이력. 링 버퍼로 묶는다(계획 D8). |
| `events`, `frames`, `ticks`, `path_revisions` | M0 (기록은 M기록) | 기록. 추적 중에는 제어 틱마다 `ticks` 가 하나씩 붙어 분당 약 6 MB 가 된다. 재시도 판정이 지금 일화의 `ticks` 를 모두 읽으므로(C5229–5231) 마지막 항목만 남기는 스풀에는 넣지 않는다. 미션 종료 때 미션 기록으로 넘기고 비운다(계획 D8 구현 설계, Codex D8 설계 검토 P2-3, 2026-10-19). |
| `corrections`, `stop_events` | M0 (기록은 M기록), **열린 항목 보유** | 마지막 항목을 뒤에서 계속 고친다(C4657 보정의 standing·판정·설치, C4706 정지 시각). 그 항목을 쥐는 `correction`·`stop_state` 도 M0 이라 같은 복원에서 놓으므로, 넘긴 뒤에는 아무도 고치지 않는다. |

## 6. `pocket`

| 키 | 처리 | 근거 |
|---|---|---|
| `check` | API | 포켓 검사 닫기(층 `exempt`·`depth_support` 해제 포함). |
| `capture`, `history`, `planning_pickup`, `await_still_frame`, `mid_still_done`, `zero_since`, `control_stamped`, `still_frame_s`, `still_frames`, `last_reason` | M0 | `planning_pickup` 이 남으면 `refresh_targets` 가 팔레트를 옛 계획 자리로 옮긴다(C2923). |
| `frames_read`, `frames_bad_time`, `frames_without_time`, `silenced_frames` | M기록 | 프레임 회계. |

## 7. `state`

**세션(S)**:
- 설정·장면 기록: `arguments`, `geometry`, `scenario`(첫 장면), `factory`, `factory_items`, `props`, `prism_colliders`, `camera`, `camera_inset`, `extra_views`, `perception_mount`, `perception_camera_intrinsics`, `pocket_camera`, `plant`, `plant_backend`, `plant_tyre_contact`, `physics_scene`, `planner_config`, `travel_planner_config`, `mission_stages`, `insertion_reserve_m`, `approach_clearance_m`, `pocket_clearance_margin_m`, `lateral_clearance_min_m`, `lidar_synthetic`, `encoder_model`, `source_sha256`, `scene_*`, `pallet_geometry_*`, `asset_origin_offsets_m`, `hidden_overhead_prims`, `low_obstacles_removed`, `new_obstacles`, `destination_marker`, `destination_prior`, `initial_pallet_m`, `setup_snapshot`, `velocities_*`, `steering_probe`, `render_mode`, `overview_render`, `overview_floor_points`, `isaac_audio_args`, `kernels`, `estop_probes`, `slam_feedback`, `slam_async`, `shared_lidar_scan`, `obstacle_layer`, `async_plan_worker`, `gc_frozen_objects`(첫 제어 틱 전 한 번)
- 지금 명령: `applied_command`, `command_speed`, `preempt_seen_s`
- **`lateral_clearance_min_m` 는 설정이 아니다.** 실제 최소 간극을 누적한다(R5877). 세션 최소(S)와 미션 최소(M0, 미션 기록)로 나눈다.
- 설정성 기록: `runtime_viewpoints`(N, 미션 시작 때 다시 만듦), `tracker_configs`(S)
- 영상 프레임 수: `frames`

**세션 기록(S/조각)**:
- 틱마다 늘어나는 것: `samples`, `tyre_contact_rows`, `leg_switches`, `tracker_versions`
- `transitions`: 미션 종료 때 그 미션 몫을 미션 기록으로도 넘긴다.

**미션(M0, 기록은 M기록)**:
- 경로·계획: `paths`, `paths_replaced`, `planning_status`, `planning_target_source`, `planning_traces`, `planning_wall_s`
- 관찰: `observation_attempts`, `observation_candidates`, `observation_waypoint_selected`, `observation_drops`, `observe_replans` 와 **`observe_replan_leg` 를 함께**(C1688), `observe_loose_arrival(s)`
- 인식·근접: `perception`, `near_capture`, `near_capture_history`, `near_capture_refused`
  - `near_tracking` 은 실행 필드만 M0: `realigns`, `realign`, `retry`, `outcome`, `start`, `starts`, `arrival`, `insert_arrival`, `injection.captures`. 설정 필드(R2785)는 S.
- 재계획·감시 카운터: `cusp_replans`, `stall_replans`, `docking_retries`, `cusp_heading_watch`, `cusp_stalls`, `cusp_heading_hold`
- 장애물 기억 사건: `obstacle.memory_events` 는 S/조각
- 비동기 작업 기록: `async_plans`, `async_plan_drops`
- 단계 결과: `delivery_docking`, `insertion_error`, `insertion_measured`, `insertion_truth_yaw_rad`, `withdraw_certificate`, `withdraw_tracking`, `lift_tilt_peak`, `slam_error_insert_end`, `return_home_error`, `tracking_failure`, `handoff`
- 포켓·기타: `pocket_check`, `forbidden_pocket_contacts`, `pocket_geometry_checks`, `repeat_capture`, `phase`
- 상태 기록 `known_pallets`(C180·192·255): 층 `known` 과 같이 N.

**결과(실시간에서 의미가 바뀜)**:
- `success`, `failure_reason`, `mission_result`, `tilt_abort_state`: 실시간에서는 미션마다 결과 기록부(계획 D1 9단계)에 남긴다. 세션 끝의 `success` 는 세션 종료 사유로 바꾼다(계획 D9).

## 8. `drive()` 지역 상태와 클로저

| 이름 | 처리 | 근거 |
|---|---|---|
| `paths`, `trackers`(TrackerBook) | N | `paths` 는 비우고 새로 넣는다. TrackerBook 은 **미션 종료의 기록 확정 전에** 옛 추적기를 `_drain`(R1659) 하고, 시작 때 키를 지운다. `clear` 를 추가하되 drain 을 거치게 한다. `_versions` 항목은 교체 때 `replaced_s` 가 바뀌므로(R1642) 열린 항목 규칙을 따른다. |
| `scenario`, `truth_destination`, `initial_pallet`, `pickup_obstacle`, `pickup_zone`, `pickup_bounds`, `return_to_pose` | N | 계획 D2 시작 처리. `truth_destination` 은 고른 목적지의 참값이다. |
| `wheel_lock` | API | 저장된 gain 으로 정상 unlock(R6331 부근). |
| `steering_command` | S | 배열을 유지하고 값은 지금 측정 조향으로 맞춘다. |
| `cusp_monitor`, `cusp_holds` | API → M0 | 커스프 감시·hold(R4670·4691). 내부 `latch`·`_watch`·`_forced`·`_serials`·`_count`, 열린 hold·watch, 기록 sink 가 있다. **미션 종료의 기록 확정 전에** 열린 hold·watch 를 닫는다. `cusp_heading_hold.py` 149 는 기존 기록 인덱스로 종료 결과를 쓴다. 그 뒤 새 메서드 `reset()` 으로 비운다. 기록(`state.cusp_stalls`, `cusp_heading_hold`·`holds`)은 M기록. |
| `inset_estimate`·`inset_status`·`inset_detail`, `overlay` | M0 | 화면 삽입 정보. |
| `slam_log`, `frame_audit`, `video_frames` | S/조각 | |
| 시간 원점(`initial_time`)·stepper 카운터 | S | |
| `live_last_rear_`, `live_first_replan_` | 삭제 | 임시 처리. 계획 D2 가 대체한다. |
| `perception_capture`(SensorCapture), 카메라·영상 카메라 | S | 서비스. `CaptureState.previous`·`previous_frame_id` 는 S 다. 이전 프레임 거부를 보존해야 한다. |
| `LiveRuntime` 의 `frame_id`·`_last_render_time`·`step_start`·프레임 원장 | S | 픽셀 시각 대응을 보존해야 한다. |
| `LiveRuntime.arbiter.requests`(요청 원장) | S, **상한 계약 필요** | 기본 4,096건이고 회수 경로가 없다(arbiter 43, generations 18). 상시 세션에서는 다 차면 새 요청이 모두 `busy` 가 된다(arbiter 229). 결과를 지운 뒤에는 옛 요청과 처음 온 요청을 구별할 수 없다. 예: 응답이 유실된 `lease_request` 가 회수 뒤 재전송되면 새 lease 를 받는다(arbiter 247·270). 그래서 계획 D7 의 **요청 토큰(다시 쓰지 않는 번호)·토큰별 단조 순번·워터마크**가 필요하다(중재기·프로토콜·브라우저 변경). |
| 작업자 `_discarded` | S, 회수 필요 | `discard()` 가 쌓기만 한다(plan_worker 149). 응답이 더 올 수 없는 ID(작업자 재시작·응답 수신)는 회수한다. `_next` 는 유지한다. |
| `book_clock`, `tracker_book` | S | |
| `gc_seen_`, GC 콜백 `live_gc_timer`, `gc.freeze()` 상태 | S | 1e53cbe. 콜백은 세션에 한 번 붙이고, 목록은 틱 시작에 비운다. 동결은 첫 제어 틱 전 한 번이다. 미션이 남긴 객체는 동결 밖이라 세션이 길수록 2세대 수집이 걷는 양이 는다. 미션 종료 뒤 다시 동결할지는 계획 D8(자원 상한)에서 측정으로 정한다. |

## 9. 미션 종료 때의 순서 (계획 D1 과 맞춤)

1. 정지와 물리 상태 고정
2. 비동기 작업 폐기(`abandon_jobs` → `capture_abandon`)
3. 보호 종료(포켓 검사·면제·`depth_support`·철수 회랑·허가 캐시)
4. 종료용 SLAM 해제(포크 관계로 분기)
5. `wheel_lock` unlock
6. **열린 구간과 감시 닫기**(상태는 바꾸지 않는다)
   - 새 `commands.close_open(t)`: 열린 pause·wait 를 닫는다. `pauses[-1]` 을 고치므로 기록 이전보다 먼저다. terminal 전환은 하지 않는다. 지금의 `finish()` 는 닫기와 terminal 전환을 함께 하고, 이미 terminal 이면 뒤 호출을 무시하므로 여기서 쓰지 않는다.
   - TrackerBook 의 마지막 drain
   - 열린 커스프 hold·watch 닫기
   - 열린 보정·정지 사건 닫기
7. 결과 판정(배달 참값). 판정 결과가 최종 outcome 이다.
8. **terminal 전환**: 새 `commands.terminate(outcome, t, reason)`. outcome 은 `done`·`failed`·`cancelled`·`recovery_required` 중 하나다(지금 `finish("cancelled")` 는 `ValueError`). 같은 outcome 이 commands·결과 기록부·발행 결과에 쓰인다.
9. 최종 기록 확정(닫힌 항목과 남은 조각, 결과·manifest 상한)
10. 미션 소유 상태 초기화(이 표)
11. 결과 공개(결과 기록부, `last_result`, `missions` 메시지)

시험: 배달 판정이 실패한 `complete` 에서 commands·결과 기록부·발행 결과가 모두 `failed` 로 일치한다.

## 10. 검증

- **U21**: 두 번째 미션 채택 직후의 M0·N 항목을 첫 미션 채택 직후와 비교한다. 위치·자세처럼 입력에 따라 다른 값은 입력과 함께 비교한다. 표에 없는 키가 나타나거나 값이 남으면 실패.
- **추출 시험**: 이 표에 있는 키 목록과 코드에서 기계적으로 뽑은 키 목록이 같은지 비교한다. 새 키가 생기면 표를 고칠 때까지 실패한다.
- **API 시험**: 각 API 정리 뒤 해제 대상(층 면제, bound method, gain, hold, 작업)이 남지 않는지 확인한다.
</content>
</invoke>
