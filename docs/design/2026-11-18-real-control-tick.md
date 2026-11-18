# 실차 제어 틱 계약 — 4e-1b (나)안 (설계)

작성 2026-11-18 · 4판(Codex 3 회차 반영) · 상위 계획 [실차 자율 실행 경로](../plans/2026-11-18-real-autonomy-entry.md) PR 1·4 · 앞선 설계 [4e-1](2026-11-11-p04e1-control-tick.md) 2절(4e-1b), [4a'](2026-11-11-p04a-kinematics-and-command.md)

## 0. 결정

- 총괄(2026-11-18)이 (나)를 채택했다: **Isaac 루프(`sim/isaac/run_transport.py` 10201–11555 행)는 바꾸지 않는다.** 실차 루프(`tools/real_autonomy.py`, PR 4)가 같은 `MissionCore` 호출을 같은 순서로 부른다.
- Codex 착수 전 상의(새 스레드)도 (나)를 권했다. 다만 완료 기준은 호출 순서 일치보다 **분기별 상태와 최종 출력**이어야 한다고 했다(P1 3). 그래서 이 PR 1 은 그 계약을 정하는 문서이고, 두 루프의 AST 대조와 가짜 포트 시험은 실차 루프가 생기는 PR 4 에 둔다.
- 총괄 결정 둘을 여기에 함께 적는다.
  - 실차 AUTO 에서 lease(콘솔 연결)를 잃으면 정지 래치, 자동 재개 없음. 운전자가 다시 무장해야 재개한다. Isaac 콘솔 동작은 바꾸지 않는다(실차 진입점 한정).
  - 실차 지도: A2M12 도착 → S6 텔레옵 → S7 LiDAR 장착 보정 뒤, 감독 텔레옵(7b 스캔 허가, 저속)으로 조사 주행. 시험장은 사용자가 학교에서 정하고 바닥 테이프로 시작 위치를 표시한다(줄자 cm·도). 시작 자세는 그 테이프 위치를 알려진 시작으로 쓴다(재위치는 나중). 그 전엔 지도 없는 실차 AUTO 없음.

## 1. 실차 한 틱 (순서)

Isaac 루프의 같은 자리를 괄호에 적는다. 실차에 없는 것은 3절.

1. **머리**: 링크 수신(`receive`) → 새 STATE 마다 한 번 `RealOdometry`·`SafetySpeed`(표본 시각 기준, 중복·공백 처리) → `health` → **AUTO 허가 상태** 갱신(lease, 과속, 주기 초과, `SafetyWatch`, 엔코더 유효성) → 콘솔 명령 반영(Isaac `live.tick_start()` 10255).
2. **SLAM·층·준비**: `core.slam_replies`(10342)로 제어 자세 → 장애물 층의 설치된 스냅숏(비동기, 계획 PR 5) → 미션 준비 요청(`live_prepare*()` → `prepare_request()`, 10372) — 이 순서가 `release_gate`·`job_hold` 앞이다.
3. **제어**(10628–11131): `release_gate` → `paused_`·`plan_wait_`(`job_hold`, 실측 `SafetySpeed`) → 관측 받기(실차 원천은 PR 7; PR 4 는 가짜 관측) → `loaded_wait_`(`loaded_start_wait`) → `scene_replan_tick`(틱 소유) → `follow`(`scene_raised_`) → `observe_backoff_tick` / `docking_and_stall` → `cusp_and_observe_recovery` → `command` → 도착 처리(`arrive`) → `job_pending` → `lift_and_timers` → `cusp_handed_` → `permission` / `permission_watch` / `blocked_replan` → `paused_tick` / 선회 정지 / `plan_wait_` / `live_leg` → `loaded_wait_` 정지.
4. **출력 조립**: `low_clearance_tick`(11242) → `assemble_command`(11245) → `observe_motion`(오도메트리, 11275) → `safety_state` → 최종 중재(Isaac `live.tick_end()` 11301 — preempt·STOP 이면 후보를 0 으로).
5. **제출**: STOP·AUTO 허가를 **다시 읽고**(2절 ①) `RealPlantPort` 로 한 프레임 → 기록.
6. **다음 틱으로 넘김**: 실제로 제출되고 MCU 가 확인한 출력만 `command_speed`·`applied_command`·`slam["applied_speed"]` 에 넣는다(Isaac 11327 의 중재 결과 반영과 같은 자리). STOP·포트 거부·미확인 후보는 '적용된 출력'으로 넘기지 않는다 — 다음 틱의 감속·취소·정체 판정(core 2298 등)이 옛 후보를 적용된 것으로 읽지 않게. 그러나 **확인되지 않았어도 아직 작동할 수 있는 DRIVE**(송신됐고 만료 전)는 방향 허가(core 6550 `command_speed`)와 취소·정지 완료 판정(core 5810 `applied_command`)에 포함한다 — 확인 출력이 0 이고 전진 DRIVE 의 확인 STATE 가 늦는 동안 MCU 는 이미 전진할 수 있다. 그 영향이 해소될 때(확인되거나 만료)까지 방향 전환·정지 완료를 보류한다. 확인 출력은 틱 머리에서 갱신한다. **큐에 남아 송신될 수 있는 프레임과 부분 송신된 프레임**(첫 바이트 뒤 `EAGAIN` — 다음 틱에 마저 나가 MCU 에서 실행될 수 있음, `real_mcu_link` 154)도 따로 추적해 같은 규칙에 넣는다. 정지 완료에는 옛 구동 명령의 영향 해소와 신선한 정지 측정이 둘 다 필요하다.

## 2. 계약 (Codex P1·P2)

① **STOP 의 기준점은 최종 제출이다.** 머리에서만 STOP 을 읽으면 중재가 끝난 뒤 온 STOP 을 놓친다. 제출 직전에 다시 읽고, STOP·허가 없음이면 옛 후보를 내지 않는다(아직 첫 바이트를 쓰지 않은 후보 포함). 송신이 시작된 프레임은 취소되지 않는다 — 그 프레임의 만료가 상한이다. 출력 직전 예외는 정지 경로로 간다.
   - **어느 STOP 에 무엇을**: 운용자 DISARM·lease 상실·링크 상실·측정 무효·제출 거절은 응답을 기다리지 않고 즉시 DISARM(상위 계획 PR 4 와 같음). 신선하고 유효한 조향·승강 측정과 MCU 허가가 있는 **제어 정지**만 아래 순서로 승강을 유지하고, 그 유지 DRIVE 자체가 거절되면 확인 시한을 기다리지 않고 DISARM 한다.
   - **제어 정지의 처리는 송신자 한 곳에서 이 순서로**: STOP 래치 → 옛 DRIVE 폐기(세대 변경) → 측정 위치를 담은 새 정지 DRIVE(바퀴 0, 승강 `target` = 측정 위치) 제출. 지금 포트는 `new_control_generation()` 이 방금 큐에 넣은 유지 DRIVE 까지 버리고, 먼저 세대를 바꾸면 `hold_lift()` 에 필요한 `_intent` 가 지워진다. 그래서 PR 4 에서 포트에 '세대를 바꾸고 측정 위치로 정지 프레임을 내는' 한 호출을 둔다(의도 캐시를 거치지 않음).

② **포트 결과에 seq 가 있다.** 지금 `PortResult` 에는 확인할 seq 가 없고, 호스트의 `nonzero`(움직일 수 있는 명령)는 바퀴만 본다. PR 4 에서 `PortResult.seq` 를 더하고, 바퀴 0 인 승강 명령과 `keep`(진행 중인 상승 목표를 유지하면서 MCU 만료를 갱신)도 움직일 수 있는 명령으로 센다. 큐 제출(`queued`)과 MCU 수락(그 seq 를 알린 STATE)을 구분한다. 승강 유지가 필요한 제어 정지는 그 seq 를 알린 STATE 를 짧은 시한(잠정 100 ms) 안에 못 받으면 DISARM 이다(계획 PR 4).

③ **분기 계약** — 각 상태에서 건너뛰는 일과 계속하는 일. 겹칠 때의 우선순위는 Isaac 그대로다: 최종 분기는 `paused_tick` → 장면 선조향 → `plan_wait_` → `live_leg` 의 `elif` 이고(`live_leg` 는 앞 분기가 잡지 않을 때만), 그 뒤 `loaded_wait_` 가 속도·곡률을 0 으로 덮고 조향을 유지한다.

| 상태 | 건너뜀 | 계속함 |
|---|---|---|
| `paused_`(정지·재개·취소, 미션 없음) | 관측 채택(진행 중 포획은 버림), 추종·도착·승강 램프 | `paused_tick`(안에서 `permission_watch`, 승강 목표, 취소·재개), 층·SLAM·링크 |
| `plan_wait_`(계획 요청·계산 중·채택한 틱, 준비 중 미션, 진행 중 포획) | 추종·도착 | `permission_watch`, 승강 램프(`lift_ramps_in_plan_wait` 일 때), 조향 유지 |
| `loaded_wait_`(적재 출발 대기) | 추종·도착 | 승강 램프와 허용된 준비 작업, `permission_watch`, 조향 유지(최종 분기 뒤 속도·곡률 0 덮기) |
| `scene_raised_`(추종이 장면 재계획을 올림) | 그 틱의 backoff·stall·recovery·`command`·도착 | `plan_wait_` 로 처리 |
| `scene_.owned`(장면 재계획이 틱을 가짐) | 일반 추종 경로 | 그 출력(presteer·escape·대기); escape 출력은 뒤의 허가·정지 분기를 그대로 받음 |
| `cusp_handed_`(선회점 넘김) | — | 활성 추종 단계에서 pause·abort·두 wait 가 없을 때만 0 속도로 `permission` 을 열어 장애물 쪽 재계획으로 넘김(`blocked_replan` 은 자기 조건) |
| 실차 조기 중단(별도 규칙 — Isaac `live_abort_` 는 뒤의 모든 작업을 막지 않음) | 관측 채택·전이·승강 램프 | 정지 출력, 층·SLAM·링크 |

`assemble_command` 의 0 요청은 감속을 거칠 수 있다. 즉시 차단은 최종 AUTO 허가(①)가 맡는다.

④ **S10 종료는 `lift → extract` 전이 전이다.** 지금 `carrying/confirmed` 는 `lift → extract` 에서 설정되므로, 전이를 생략하면 화물 상태를 따로 갱신해야 한다. 지금은 실차 분기도 들림 확인에서 `lift_and_timers()` 안의 `transition("extract")` 를 부르고, 전이는 화물 관계와 장애물 기억을 바꾼다. 그래서 반환 뒤 바퀴만 0 으로 하는 것으로는 부족하다. PR 4 에서 `MissionCore` 에 실차 전용 S10 종료(옵션, 기본 꺼짐 — Isaac 무변경)를 두어 들림 확인 때 전이 대신 '들고 멈춤' 종료 상태로 간다. 화물 상태에 반영하고, 바퀴 0 을 유지하며, 후속 AUTO·텔레옵은 현장 복구와 재위치 확인까지 잠근다. 코어 변경이므로 재생 게이트 29/29 와 전체 pytest 를 거친다. 하역·복귀 완주 시험은 이 옵션을 끈 별도 구성이다.

⑤ **관측의 채택 조건**(Codex P2). 실차 카메라가 프레임을 주는 것과 그 결과로 삽입을 허가하는 것은 별개다. 미션 세대·단계·SLAM 보정이 바뀐 뒤 온 결과는 버린다(지금 코어의 관측 경로). `observation_freeze()` 는 호출 시점의 태그를 찍으므로, 실차 비동기 결과는 **요청·취득 시의 태그**를 함께 넘기고 도착 뒤에 처음 태그를 찍지 않는다. 틱 끝의 `cargo.observe()`·`observe_motion()`(맵 보정이 섞이지 않은 오도메트리)은 실차 루프에도 둔다. PR 4 는 가짜 관측으로 이 경계를 시험한다.

## 3. 실차에 없는 것

참값 평가 검사(10380–10627: 기울기·발자국·미끄럼·접촉), Isaac 새 장애물(11132–11224), estop 시험 probe(11225 의 `estop`), stand_hold·dwell 잠금(11376–11427), 렌더·물리 스텝·추적 카메라·pocket_frames·포획 진행(11444–11528 의 Isaac 부분), 시험 주입(`live_test_*`, `correction_inject`, cusp·observe 종료 주입, near_end_detour), `manual_recovery_tick()`(`pallet_z()` 를 읽는 Isaac 복구 — 실차는 4d-2b 현장 복구 경로). 실차 루프는 이것들을 조립하지 않는다(훅은 `fail_closed_hooks`).

**실차에도 남는 것**: D5 포켓 검사의 입력(취득 시각에 맞춘 깊이·자세·승강, 새 정지 프레임 판정 — 없으면 `await_still_frame` 이 풀리지 않음; 실 입력은 PR 7), 하역·복귀 구성의 후퇴 통로 감시 `withdraw_tracking`(추정 `rear` 로 매 틱, 10359).

## 4. PR 4 의 시험 (이 계약을 고정)

- **AST 부분 순서**(평탄한 목록이 아님): ① 명령 반영·SLAM 응답·제어 자세 → 미션 작업, ② `scene_replan_tick` 이 틱을 가지면 추종 없음, `follow` 가 `scene_raised_` 면 stall·도착 없음, ③ `permission`/`permission_watch`/`paused_tick` 의 분기와 감시 횟수, ④ 층 감시·명령 조립 → 최종 중재 → 포트 제출. `core.commands.apply()` 처럼 한 단계 더 들어간 호출과 어댑터 경계도 포함한다. Isaac 루프와 실차 루프에 같은 검사를 돌려, 어느 쪽이든 순서를 바꾸면 실패한다.
- **가짜 훅 값 시험**: 같은 core 호출 안에서 `set_rear()` 다음 `grid_kwargs()` 가 새 자세를 읽음, 반환된 `rear`·hold 플래그가 다음 호출에 전달됨.
- **미확인 DRIVE 시험**: 확인된 0 → 전진 DRIVE 송신(완전 송신, 그리고 첫 바이트 뒤 `EAGAIN` 의 부분 송신 두 경우) → 확인 지연 중 반대 방향 요청 또는 취소 → 방향 전환·정지 완료가 보류됨.
- **순서 시험**: SLAM 응답 → 제어 자세 → `prepare_request()` → `release_gate`·`job_hold`.
- **가짜 포트 출력 시험**: 정상 틱의 최종 `command()` 1 회; 중재와 제출 사이 STOP → 옛 후보 제출 0 회; 큐에 들어간 뒤 STOP → 폐기; 출력 직전 예외 → 정지 경로; 세대·만료·MCU 확인까지 검증; STOP·거부 뒤 다음 틱이 옛 후보를 적용된 출력으로 읽지 않음; 바퀴 0·승강 상승 중 STOP — **성공 경로**(실제 호스트 큐를 거쳐 정지 프레임의 정확한 seq 가 STATE 로 확인됨)와 확인 STATE 유실(→ 시한 뒤 DISARM) 둘 다, 그리고 유지 DRIVE 가 거절되면 시한을 기다리지 않고 DISARM.
- **분기 시험**: ③ 표의 각 행과 겹침 — `plan_wait_` + 선조향, `loaded_wait_` + 선조향, `cusp_handed_` + wait.
- **S10 종료 시험**: 들림 확인에서 멈춤, `extract` 전이 없음(그 장애물 기억 삭제도 없음), 화물이 `carrying/confirmed`, 확인 틱과 다음 틱 모두 최종 바퀴 출력 0, AUTO·텔레옵이 현장 복구와 새 세대 재위치 확인 전까지 거부.
- **관측 태그 시험**: 요청 뒤 세대·단계·보정이 각각 바뀐 뒤 도착한 결과는 버려지고, 일치하는 결과는 채택된다.
- **lease 상실**: AUTO 중 lease 를 잃으면 정지 래치, 재무장 전 재개 없음.

## 5. 자체 검토

- Codex 착수 전 상의(새 스레드, 2026-11-18): (나) 권고에 동의. P1 3(STOP 기준점은 최종 제출 — `PortResult` seq·호스트 nonzero 의 승강; 분기별 건너뜀·계속 계약; S10 종료는 전이 전), P2 2(AST 는 부분 순서·분기 소속 + 가짜 훅 값 시험; 관측 채택 조건·틱 끝 기록), P3 1(break 13·core 직접 호출 52 곳·43 종). 모두 2·4 절에 반영.
- 이 문서의 Codex 검토 1 회차: P1 1(STOP 이 승강 유지 프레임까지 지움 — 송신자 한 곳의 순서와 포트의 '세대 변경 + 측정 위치 정지 프레임' 한 호출, `keep` 도 outstanding, 성공 경로 시험), P2 5(분기 겹침 우선순위·`scene_raised_`·`cusp_handed_` 조건·조기 중단 규칙, 준비 요청 순서와 다음 틱으로 넘길 적용 출력, 실차에 남는 D5 입력·`withdraw_tracking`·`manual_recovery_tick` 은 Isaac, 관측 태그는 요청·취득 시, S10 종료 상태의 화물·출력·잠금 확인). 모두 1–4 절에 반영.
- 2 회차: P1 1(확인 출력만으로는 현재 움직일 수 있는 방향을 모름 — 송신됐고 만료 전인 미확인 DRIVE 를 방향 허가·취소 판정에 포함, 해소까지 보류, 시험), P2 1(승강 유지 경로는 제어 정지에만, 나머지는 즉시 DISARM, 유지 DRIVE 거절 시 즉시 DISARM — 상위 계획과 맞춤), P3 1(`manual_recovery_tick` 을 1 절 목록에서 뺌). 반영.
- 3 회차: P1 1(부분 송신·큐에 남은 프레임도 미확인 DRIVE 처럼 — 추적·보류·시험, 정지 완료의 두 조건), P3 1(상위 계획 PR 4 문구 — lease 상실·유효한 조향 측정). 반영.
