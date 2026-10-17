# M-a0 목록: 시계·벽시계·선변경 연산·세계 읽기·기록 순서 (2026-10-15)

[M-a 계획](2026-10-15-m-a-mission-core.md) D2·D3·D5 의 입력. 줄 번호는 `aab7066` 의 `sim/isaac/run_transport.py`(RT) 기준이고, 다른 파일은 이름을 붙인다. 목록은 서브에이전트가 만들고 표본 13 개(RT 5948·5964·6057·6241·7756·7769·7821·8213·2262·3190·2917, `slam_pose.py` 283–285, `cusp_stall.py` 524)를 원본과 대조했다. **분류는 M-a0.5 의 제안이며 검토를 받아 확정한다.**

시계: **t** = 루프 시각(`world.current_time - initial_time`, 5735 에서 반복마다 한 번), **phys** = 물리 틱 시각(`step_world` 5528, 캡처 중에도 흐른다), **render** = 카메라 `rendering_time`, **wall** = `time.monotonic()`/`time.time()`, **calls** = 루프 반복·물리 틱·메서드 호출 횟수. 루프 예산 `range(int(120 * max_sim_seconds))`(5734)은 **루프 반복 수**다 — 캡처는 반복 없이 t 를 진행시키므로 sim 시간 예산이 아니다.

## 1. 시계·카운터와 일시정지 분류

분류: **W** 작업 시간(일시정지에 멈춤), **F** 신선도·감시(계속 흐름), **WT** 작업 틱 카운터(정체·막힘을 세므로 일시정지 중 서 있으면 잘못 발동 — 멈춤), **M** 측정(물리 틱마다 계속), **B** 횟수 예산(시간 무관), **R** 기록·표시만.

| # | 위치 | 무엇 | 한도 | 시계 | 분류 |
|---|---|---|---|---|---|
| 1 | RT 5734 | 미션 예산 | `120 × max_sim_seconds` 반복 | calls | B(시험은 일시정지만큼 늘린다) |
| 2 | RT 5964–5966 | 추종 시간 초과 | `max(30, 3×nominal+10)` + dwell·safety stop 보류 | t | W (release 대기 중 건너뛰지만 `phase_started` 는 안 바뀜) |
| 3 | RT 4288 외 14 곳 | `phase_started` 재설정 | — | t | W |
| 4 | RT 5948–5949 | SLAM release 전 정지 대기 | 10 s | t | W |
| 5 | RT 7576/7579 | lift·lower 고정 시간 | 4.5 s | t | W |
| 6 | RT 7588/7594 | settle·home_settle | 1 s | t | W |
| 7 | RT 6558–6559 | observe 느슨한 도착 | 1 s | t | W |
| 8 | RT 6057–6058 | D7 도킹 정지 | 120 반복 | calls | WT |
| 9 | RT 6108–6121 | SLAM 정체 | 120 반복(목표 heading 놓치면 119 로 미리 채움) | calls | WT |
| 10 | RT 6241–6242 | cusp 재계획 정지 확인 | 12 반복 | calls | WT |
| 11 | RT 6390–6412 | observe cusp 정체 | 120 반복 | calls | WT |
| 12 | RT 5998/6002 | 탈출 끝(서 있기) | 120 반복 | calls | WT |
| 13 | RT 6596–6597 | 후퇴 끝 강제 재계획 간격 | 1 s | t | W |
| 14 | RT 7756–7757 | 포켓 정지 0 유지 → 종료 | 5 s | t | W |
| 15 | RT 7769/7794 | 경로 막힘 0.3 s → 허가 0 | 36 반복 | calls | WT |
| 16 | RT 7817/7828 | 정지·재계획 트리거 | 120 반복 | calls | WT |
| 17 | RT 7821–7826 | 진행 없음 감시 | 60 s | t | W |
| 18 | RT 7779–7793 | N9 센서 침묵 정지 | 1 s, 서 있음은 `truth_speed` | t | F |
| 19 | RT 8147–8149 | 재계획 빈도 창 | 30 s 에 3 회 | t | W |
| 20 | RT 8213/8273 | live 계획 재시도 | 1 s | t | W |
| 21 | RT 8381, `estop_probe.py:61·96–98` | 비상정지 탐침 지연 | 설정값, 0.25 s | t | W(시험 주입) |
| 22 | RT 2917–2918 | 근접 종료 정지 유지 | 0.5 s(`near_tracking.py:25`) | phys | F |
| 23 | RT 5653·4933 | 진단 덤프 제한 | 횟수 | t | R |
| 24 | RT 2308 | 오버레이 격자 표본 | 0.5 s | t | R |
| 25–26 | RT 8499–8510 | 포켓 자세 이력 120 개, `step % 12` 렌더 주기 | — | calls | M |
| 27 | RT 5524–5581 | LiDAR·영상·타이어 기록 주기 | `scan_every` 등 | phys | M |
| 28 | RT 6080·6228·6414·4834·7888·8093·8075·8152·8271·7428·2922·6900 | 재계획·재시도·탈출·후퇴·후보 횟수 | 2–10 회 | — | B |
| 29 | `slam_pose.py:283–305` | `StopDetector` 창·명령 유지·주행 | 0.1/0.2/0.1 s → 12/24/12 호출 | phys calls | M(명령 입력 `last_command` 는 루프 반복마다만 갱신, RT 8489) |
| 30 | `slam_pose.py:245–247` | 보정 나이 → `LocalizationStale` | 0.25 s(RT 3923) | phys/t | F |
| 31 | `slam_pose.py:250–261` | hold 이동 거리 | 15 m | 거리 | F |
| 32 | `slam_pose.py:209–211` | warming 해제 | scan id ≥ 4 | 스캔 수 | M |
| 33 | `path_tracking.py:214–220` | 명목 소요(추종 시간 초과의 입력) | — | — | W |
| 34 | `cusp_stall.py:35·420·448` | 진행 정체 | 120 tick() | calls | WT |
| 35 | `cusp_stall.py:42·359·524` | 제동 미확인 종료·센서 인계 | 5 s | t | F(정지 감시) |
| 36 | `cusp_stall.py:47·560` | 점유 스캔 | 3 스캔 | 스캔 | F |
| 37 | `near_field_tracking.py:193–194·607–627` | 관측 소실·재획득 → 실패 고정 | 0.3 s / 3 s | render vs phys | F(일시정지 중에도 계속 — 계획 D5) |
| 38 | `near_field_tracking.py:192·195·568` | 불일치·인계 프레임 수 | 3 | 프레임 | F |
| 39–41 | `near_tracking.py:21–22·28·140–153` | 후방 이력, 명령 창, 결과 대기 | 1 s, 0.3 s | t/render | F |
| 42 | RT 2781·2804 | 깊이 프레임 신선도·대기 | 지연 합 | render | F |
| 43–44 | `near_tracking.py:523–570·647–681` | 정지 상태·보정 설치 시각 | — | phys/render | F |
| 45 | `obstacle_layer.py:258·272`, `drive_permission.py:482` | 센서 침묵 | 0.25 s | phys | F |
| 46–48 | `drive_permission.py:488`, `obstacle_grid.py:330–367`, `obstacle_layer.py:198–211` | FREE 증거·점유 나이 | 0.2/0.3 s | phys | F |
| 49 | `known_obstacles.py:65·123` | 알려진 팔레트 성장 | 나이 비례 | phys | F |
| 50 | `pocket_check.py:63·251·275` | 깊이 프레임 나이 | 0.2 s | render vs t | F |
| 51 | `pocket_frame_recorder.py:26·253–307` | dwell 2 s·safety stop 0.5 s | — | t | W |
| 52 | `new_obstacles.py:152` | 예정 장애물 주입 | 설정값 | t | W(시험 주입) |
| 53 | `perception_adapter.py:400–417·453·535` | 캡처 루프 | 최대 시도 수, 렌더 지연 0.0667 s | phys/render | M |
| 54 | `cusp_heading_hold.py:67·83·135` | 기록 시각 | — | t | R |

**정리.** W 와 WT 는 일시정지에 멈춘다 — 특히 WT(정체·막힘 카운터)는 일시정지로 서 있는 동안 세면 재계획·실패가 잘못 발동한다. F 는 계속 흐르므로 오래 멈춘 뒤에는 재관측·재캡처가 필요할 수 있다(설계 4.2). 18 번(N9)의 '서 있음' 판정과 2·4 번처럼 기존에 release 대기를 어떻게 세는지가 얽힌 항목은 M-a0.5 에서 다시 본다.

## 2. 결정을 바꾸는 벽시계 읽기

기록만 하는 `*_wall_s`(RT 3638·3734·3754·3768·4137·4342·4398·4488·4499·4779·4794·5443·5456·5490·6127·6183·6262–6276·6422·6440·6898·6942·7184·7201·7856·8037, `pocket_detector.py:742·846` 의 `elapsed_s`)는 정규화 비교에서 뺀다. 결정을 바꾸는 것은 다음이다.

| 위치 | 무엇 | 영향 |
|---|---|---|
| RT 2262 → 3190 | `run_wall_start` 와 `latest_map.npz` mtime 비교 | 실행 전 지도 거부 |
| RT 3178–3209, 브리지 `isaac_slam_bridge.py:157–167·318` | 계획이 읽는 지도 파일(비동기 교체) | 지도 내용 — 결정성 실험에서 관측됨 |
| RT 7861·7866·7989·8241 | `obstacle["deadline"] = monotonic + 20` (인식 뒤 10/20 분할) | 계획 마감 |
| RT 4791 | D7 재접근 `SearchBudget(20.0)` | 마감 → `docking_retry_no_plan` |
| `pallet_mission.py:123·177–180` | 마감 뒤 성공도 timeout 으로 | 계획 결과 |
| `pallet_mission.py:1101·1243–1315` | 복귀 회전 후보 18 s 한도 | 고른 복귀 경로 |
| `hybrid_astar.py:603` | 256 확장마다 마감 검사 | 탐색 결과 |
| `slam_link.py:215`(RT 3902) | SLAM 응답 시간 초과 | 실행 실패 |
| 브리지 `isaac_slam_bridge.py:237·246–259` | warmup 1 s·응답 60 s 대기 | processed/skipped/failed → 보정 모드 |

## 3. 포트 호출 전에 미션 상태를 바꾸는 연산

비동기 포트(M-c 이후)에서 늦은 결과를 버리려면 이 연산들을 준비 → 포트 → 확정으로 나눠야 한다(계획 D5, M-c 전 별건). 4·7·13 은 중단이 없어도 실패 경로에서 반쯤 바뀐 상태를 남긴다.

| # | 연산 | 먼저 바꾸는 것 | 그 뒤 호출 |
|---|---|---|---|
| 1 | `slam_release`(4309) | `pending_release=None`, `tracker.release`, `holds`, 도킹 복귀면 `return_scenario` | `plan_transport_leg`/`plan_return_leg`(4344/4367), `require`(4400) |
| 2 | 루프의 release 지점(5946–5954) | `slam_release`, `rear`·`slam["control"][-1]` 덮어쓰기 | 추종 계속 |
| 3 | `release_and_carry`(4733) | release, `set_withdraw`, 목적지 이동, `holds`, 격자 재투영 | 호출자의 계획(4846, 7875) |
| 4 | `dock_at_delivery_straight`(4451) | `delivery_docking` 연결, 정합 뒤 `rounds`·`status`·`to_world`·`move_withdraw`·`goal_frame="held"`·`transport_scenario` | `bicycle_rollout`·`round_two_stop_check`(4599/4653), `start_docking_retry`(4676); 4678 거부 때 인출 경로는 이미 옮겨짐 |
| 5 | `start_docking_retry`(4829) | `retries` 추가, `previous_goal`·`goal_frame`, `release_and_carry` | `reapproach_plan`(4846), `require`(4849) |
| 6 | 재접근 중 `transport_replan`(4852) | 호출자가 이미 release(7847) | `reapproach_plan` 뒤 기록 |
| 7 | `release_after_backoff`(4948) | release, `holds`, 격자 갱신, `slam["control"]` 덮어쓰기; 후퇴 끝 호출자는 `released`·`stop_cause` 를 먼저 쓰고 `force_replan` 은 다음 반복 | 다음 반복의 재계획 |
| 8 | 정체 재계획(6121) | 카운터 0, release, `holds`, 격자 버전·재투영, `rear` | 계획(6164/6170), `require`(6187) |
| 9 | cusp 재계획(6257) | 카운터 0 | `plan_transport_leg`(6263) |
| 10 | observe 재계획(6417) | 카운터 0, 추종 제동 | `plan_observation_leg`(6423), `require`(6446) |
| 11 | 장애물 재계획(7828) | `force_replan` pop, 카운터, `stop_cause` pop, release, 마감, 후보 번호 증가, 경로·추종기 설치 | 후보·임무 계획, 실패·후퇴 분기 |
| 12 | live 계획(8208) | `live_replanned`, 격자 재투영, 마감 | `plan_transport_leg`/`plan_return_leg`(8244/8253) |
| 13 | observe 도착 캡처(6601) | `observation_attempts` 추가 → 캡처(안에서 물리·SLAM·오도메트리·장애물 층·StopDetector 진행) → release, `retry_stand`, 실패 상태; 유효 검출이면 `perception`·`near["start"]`·`planning_pickup`·`known_pallet` | `plan_transport`(7186), `require`(7206); `grid_kwargs("mission")` 도 `pickup_estimate` 를 쓴다(3277) |
| 14 | 근접 캡처 재작도(7409–7419) | insert·extract 경로 설치 | `realign_short` 거부 → `start_near_realign`(재작도 경로가 남는다) |
| 15 | `start_near_realign`(3620) | `realigns`, `near_capture` 거부 표시 | `release_and_stand`, `transition("observe")` |
| 16 | `start_pickup_retry`(3572) | 재시도 수, 추적기 제거, 보호 재설정, 기록 이동 | `release_and_stand`, `transition` |
| 17 | `release_and_stand`(3524) | release, 재투영, 추정 이월, 서기 경로 설치 | — |
| 18 | `near_tick`(2692) | 이력, 추적기 `arm`·실패 래치, 보호 상태, 보정 기록 | `near_redraw`→`redraw_docking`, `near_install` |
| 19 | `near_install`(3481) | 계획 뒤 확정(경로·추종기·추정·known pallet·진행 감시·층·`phase_started`) | — (확정형) |
| 20 | `transition()`(4248) | `transitions` 기록, withdraw 면 경로 재작성·`certify_withdrawal`(층 설정 변경) → `require`(4225), 포켓 정리, 기억 철회, `phase`, hold/`pending_release` | 호출자의 `build_near_tracker`(실패하면 hold 가 열린 채, 2566) |
| 21 | `cusp_tick`/`CuspStallMonitor` | 래치 기록·`restart_speed_slew(0)`, heading hold 기록 | 판정·`release_cusp`/`fail_pending` |

## 4. 결정 안의 세계 읽기

- **참값이 결정에 쓰임(항상)**: 팔레트 z 로 lift·lower 종료(7577·7581), `truth_speed`(7614 → 7621 재계획 조건·7647 방향·7650/7732 상한·7785 N9), 캡처 정지 판정의 `pose_fn=robot.get_world_pose`(2168, `perception_adapter.py:418–425`), 비상정지 탐침(8378–8392), 도킹 기준(taught station)을 참 목적지에 광선 투사로 미리 고정(5704–5707), `scenario.pickup`(3645–3653 런타임 시점, 7166·3352 `planning_target`), 격자 계획이 없을 때 도킹의 참 장애물 검사(4626–4632), 인식이 없을 때 시나리오 집기점으로 계획(3757).
- **참값이 결정에 쓰임(SLAM 없을 때만)**: 제어 자세(5741), 속도(5753), cusp 정지(5007–5008·6235–6237), 정지 판정(7806), 캡처 수락 자세(6668–6677).
- **실측 피드백(실차에도 있음)**: 조향 관절(4973 `held_kappa`, 2750, 8405), 바퀴 관절(8445 dwell 잠금, 5538–5541 엔코더), 승강 관절(7584 내리기 확인, 2145 캡처 승강 가드, 2660·2986·7733 카메라 장착·포켓).
- **센서 시뮬레이션**: LiDAR 광선 원점·초기 스캔(5538, 3019), 카메라 프레임·깊이(2425·2638·2959·2943), 물리 시각(5528).
- **기록·평가·표시만**: 4497–4560 도킹의 `truth_rear`, 3971, 3996, 5788·5801·5916·5934·7524·7541·8387–8388·8525, 7123, 7153–7163, 5406–5427·5499·5504·5531·5314–5394·4150, 7598·8535–8564, 2187–2188, 8346–8377(장애물 주입).

## 5. 기록에 들어가는 `id()`·순서

- `id()` 가 기록에 들어간다: cusp heading hold 키(5976·5979·5982·6326·6369 → `cusp_heading_hold.py:45–49·91–99`, `result.json`), 경로 변경 감지(2292–2293 → `plan_history.json`). 기록기는 추종기 일련번호(`TrackerBook` serial)·경로 번호로 바꾼다. 같은 객체가 GC 뒤 id 를 재사용하면 지금도 기록이 합쳐질 수 있다.
- 순서: `record_json` 은 `sort_keys` 없음(166) — 삽입 순서이고, 같은 순서로 사건이 나면 결정적이다. `stale_now` 집합(2782)은 `sorted` 로 쓰므로 안정. `obstacle["ticks"/"reasons"]`(7686–7689·7797–7799)는 첫 발생 순서. 소스 해시는 정렬(1444–1445).
