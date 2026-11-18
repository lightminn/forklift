# 실차 조립 · AUTO 허가 상태 — 4b (설계)

작성 2026-11-19 · 2판(Codex 1 회차 반영) · 상위 [실차 자율 계획](../plans/2026-11-18-real-autonomy-entry.md) PR 4 · [제어 틱 계약](2026-11-18-real-control-tick.md) · 앞 단계 [4a 알려진 팔레트 시작](2026-11-19-known-pallet-start.md)(#598)

## 범위와 책임 나누기

| 단계 | 하는 일 | 장치 |
|---|---|---|
| **4b** (이 설계) | 조립(옵션·훅·부팅 입력·S10-A 설정)과 조립 진단, AUTO 허가 상태의 전이(가짜 입력), 준비 판정이 읽을 증거의 공급 인터페이스, 중재기의 실차 lease 정책 설정, 4a 첫 프레임의 정지 판정 교체 | 열지 않음 |
| 4c | 50 Hz 루프·가짜 포트(실제 `McuLinkHost` + 가짜 MCU)의 틱·STOP·제어 정지·outstanding 추적, S10 종료 수거와 `end_reset()` 뒤 잠금 | 시험용 전송기만 |
| PR 5 | 안전 배선(비동기 층, 감시 배선, 실제 콘솔 입력, 단일 실행기) | — |
| PR 7 | 실센서(SLAM 링크, D435i 깊이) | 실기 |

- 실 MCU 로 AUTO 의 ARM·확인용 0 DRIVE·구동을 내보내는 일은 PR 5 와 S10 선행 조건 뒤에만 연다(상위 계획). 4b 실행기는 장치를 열지 않는다. 4c 는 운영 장치에서 AUTO ARM 과 DRIVE 가 모두 불가능함을 시험한다.
- Isaac: 공유 코드 변경은 둘이다. 중재기 설정 하나(기본은 지금과 같음)와 4a 첫 프레임의 정지 판정(알려진 시작에서만)이다. 둘 다 게이트와 전체 pytest 로 확인하고, Isaac 기본값의 lease 상실 동작은 중재기 단위 시험으로 고정한다(게이트는 live lease 상실을 지나지 않음).

## 1. 자리

- `src/forklift_core/live/real_autonomy.py`(새 모듈): 조립, 옵션 검사, S10-A 설정, 준비 판정, AUTO 허가 상태. 순수 Python 이다.
- `tools/real_autonomy.py`(새 진입점): 4b 에서는 `--dry-assemble` 만 한다. 조립 기록을 쓰고 끝내며, 종료 코드는 다음과 같다.
  - 0: 조립됨(운용 준비 미충족, 예컨대 STATE 없음은 기록만).
  - 2: 구조 오류(옵션 거부·훅 누락·설정 오류).
- 콘솔 서버(`console/`, wheel 밖)는 4c·PR 5 에서 이 진입점이 붙인다.

## 2. S10-A 설정 파일 `config/real/s10a.yaml`

- 지도 기준(조사 지도 묶음 경로), 테이프 시작 자세, 시험장 경계(`pickup_bounds`·`pickup_zone`).
- 팔레트 형상·ID, 계획용 고정 목적지(코어의 `plan_transport` 와 설치는 `transport` 를 포함한 경로를 요구한다; 실제 주행은 S10 에서 승강으로 끝난다), `scenario`·`wait_pose`.
- 팔레트 자세는 설정이 아니라 운용자가 미션마다 입력한다(4a, `operator_input`).
- 조립은 이 설정으로 `TargetsBook` 에 팔레트·목적지를 등록한다. 그래서 첫 제출이 기존 명령 경로(팔레트 ID·목적지 ID·대상 revision 승인)를 지난다.
- 첫 제출 전에 운용자가 빈 포크·포켓 밖·승강 내림을 확인한다(코어의 초기 `free/none` 은 물리 측정이 아님). 그 확인이 없으면 준비 판정이 거부한다.

## 3. 옵션 — 실차에서 명시, 빠지면 거부

- `real_options(cfg)` 는 S10-A 에 필요한 값만 명시한다.
  - 필수: `dt = 1/50`, `real_vehicle = True`, `kinematics`(`RealKinematics`, 조향률은 `vehicle.yaml` 의 `steering_control.steering_rate_rad_s`), `grid_planning`·`obstacle_act`·`pocket_check`·`s10_stop_at_lift = True`.
  - 계획: `mission_stages`(Isaac 과 같은 단계), `observation_waypoints = []`(가변 목록), 계획·추종 설정.
  - 나머지 기능 옵션은 꺼짐으로 명시한다: `near_tracking = None`, `bottom_deck_check = "off"`, 장면 재계획·탈출 선조향·D7 재접근 등. 코어가 `getattr` 로 읽는 이름 전체를 조립 기록에 값과 함께 남긴다.
- `real_option_refusal(options, wiring) -> list[str]`: 필수 값 누락·불일치, 근접 추적·밑판 검사 켬이면 거부한다. 곡률은 **실제로 planner·tracker·preview 에 넘긴 값**이 같고 `RealKinematics` 의 `min(−κmin, κmax)` 이하여야 한다.

## 4. 훅 — `fail_closed_hooks(RealHooks(...))`, 경로 표로 고정

- 준비 → 채택 → 접근 → 첫 프레임 대기·삽입 → 승강 → 종료 경로에서 코어가 실제로 읽는 훅 이름과 반환형을 표로 고정한다(`tests/unit/live/test_real_autonomy_hooks.py` 의 `S10A_HOOKS`). 시험은 가짜 원천으로 이 경로를 돌려 읽힌 이름이 표와 같고, 각 반환형·실패 사유가 맞는지 본다.
  - cusp 기능을 꺼도 접근은 `cusp_holds()` 를 부르고 `dump_tracking` 을 읽는다. `docking_and_stall()`·`cusp_and_observe_recovery()` 도 분기 전에 planner·cusp 훅을 읽는다. 표에 넣는다.
- 원칙:
  - `require` 는 실패를 제어 루프로 전달한다(예외). `rear`·`set_rear` 는 같은 틱 안에서 같은 자세를 공유한다.
  - 표시 훅(`show_path`·`add_path_display`·`set_inset*`·`snapshot`·`echo`)은 명시적 무동작이다. 기록 훅(`path_record`·`write_paths`·`record_json`·`save_npz`·`dump_tracking`)은 실행 폴더에 쓴다.
  - 평가 훅은 `fail_closed_hooks` 가 None 으로 답하고, 참값·시뮬 전용 훅은 TruthAccessViolation 이다.
  - `grid_kwargs` 는 준비 때 부작용이 없다(`obstacle["pickup_estimate"]` 는 채택 때 코어가 넣음).
  - `known_pocket_frame` 은 PR 7 전까지 None 이다. 4a 규칙대로 5 s 뒤 미션이 끝난다.
  - `operator_check` 는 `(serial, verdict)` 를 돌려준다(4b 시험은 가짜, 4c·PR 5 는 콘솔).
- 시계: 코어 시각은 실차 단조 시계에서 세션 시작을 뺀 값이다. `wall` 은 벽시계, `world`·`time_module` 은 구현하지 않는다(경로에서 읽히면 표가 잡음).

## 5. 부팅 입력 (값과 뜻을 시험; Isaac 과 dict 키 전체 비교 안 함)

- `MissionState(phase="idle")`, `near=None`, `paths = {}`, `trackers = {}`(코어가 채움).
- `state = {"transitions": [], "planning_wall_s": 0.0, "paths": {}}`, `pocket = {"check": None, "frames_read": 0}`.
- `obstacle`: 실차 층(알려진 팔레트 꺼짐), `applied` = 지금 적용 보정, `version: 0`, 집계 dict 들.
- `slam`: `tracker`(SLAM 링크 추적기), `records: []`, `docking: {}`, `stop_now: False`, `holds: []`, `control: []`.
- `settings`: 코어가 직접 읽는 다섯 키(`drive_acceleration_mps2`·`lift_rate_mps`·`lift_target_m`(운반 높이)·`steering_command_rate_rad_s`·`withdraw_speed_mps`).
- `Arbiter(mission=core.commands, config=… auto_lease_loss="stop")`, `core.jobs = PlanWorker(...)`, 마지막으로 `core.baseline = MissionBaseline.capture(core)`.

## 6. 준비 판정과 증거 공급

- `auto_ready_refusal(now, stage) -> str | None` 을 두 단계로 판정한다.
  - **제출·채택(`stage="submit"`)**: SLAM `tracking` 이고 응답이 신선, 재위치·현장 복구 잠금 없음(`real_auto_refusal`), 운용자 시작 확인(2절).
  - **무장·주행(`stage="drive"`)**: 이 미션의 정상 `holding` 을 허용한다(`transition("approach")` 가 hold 를 건다). 대신 다음을 모두 본다.
    - 최신 유효 SLAM 응답의 취득 시각이 나이 상한 안.
    - hold 이동 한도 안.
    - 오도메트리 연속.
    - **적용 중인** 보정과 설치된 층 스냅숏의 보정이 같음.
    - 스캔 신선도는 설치 결과의 최신 스캔 취득 시각으로 판정(스냅숏 `stamp_s` 는 재투영 시각이라 쓰지 않음).
  - 보정이 바뀐 뒤에는 새 결과가 설치될 때까지 출력을 닫는다.
- 공통: 작업자 준비(`PlanWorker._dead` 없음 → 공개 `worker_ok()` 를 조립 쪽에서 감쌈), 층 있음, 기준선 캡처됨, 링크 `health` 정상·빌드·시계율 검증(텔레옵의 `_arm_blocker` 와 같은 검사를 공용 함수로), 측정 유효(조향·승강, 계수 결함 비트).
- 증거 공급 인터페이스(4b 는 인터페이스와 가짜, 4c 가 새 STATE·호스트 이벤트에 연결):
  - `odometry_continuous()`: `RealOdometry.update()` 의 부팅 변경·시간 공백. 단절이면 이전 위치·정지 증거를 버린다.
  - `stop_confirmed()`: 새 고유 표본 기준 정지 확인.
  - `scan_fresh()`, `slam_fresh()`.
- 준비가 안 되면 미션 제출은 그 사유로 거부되고(`prepare` 전), 구동 출력도 없다.

## 7. AUTO 허가 상태

- 상태: `OFF → ARMING → READY → DRIVING`. 래치는 `LATCHED` 다.
- **AUTO 요청 = 새 사용자 `mission_submit` 한 건**(S10-A, 일회성). 콘솔의 `arm`·`mode_request(AUTO)` 는 쓰지 않는다(지금 TELEOP 전용·거부).
- ACK 뒤 계획 중에는 무장하지 않는다. 무장 순서:
  1. `job_hold()` 가 성공 답을 채택해 `adopted_id == mission_id`·`running`·`approach` 가 확정되면, 지금 lease 와 세대를 다시 검사한다.
  2. 제어 세대 변경과 큐 폐기(`port.new_control_generation()`) 뒤 `host.request_arm()` 을 **한 번** 부른다 → `ARMING`.
  3. `ARMING` 동안 루프는 코어 틱을 돌리되 최종 출력은 0 이다(허가 정지처럼 추종기는 움직임이 없음). 무장 시한은 호스트 `arm_timeout_s` 0.5 s 이고 추종 시한(≥ 30 s)에 비해 작다.
  4. 호스트가 확인용 0 DRIVE 를 직접 보내고 `ZERO_SEEN` 으로 `ready` → `READY`. 이때 `health`·현재 권한 세대를 같이 확인한다.
  5. 그 뒤 **새 후보**부터 제출하고, 첫 DRIVE 가 큐에 들어가면 `DRIVING`.
  - `Blocked`, 시간 초과, 무장 중 권한 철회는 래치 사유다.
- **래치**: 다음 사유가 오면 실차 허가 상태가 직접 받아 이전 요청·후보를 버리고 `LATCHED` 로 간다.
  - STOP, 운용자 DISARM, lease 상실, 세션 종료, 링크 상실.
  - 측정 무효, 포트 `refused`/`blocked`, 감시 trip(과속·주기 초과·SafetyWatch), 준비 판정(drive) 실패, `real_auto_refusal` 사유.
  - 긴급 정지는 즉시 DISARM 을 큐에 넣고, 첫 바이트를 아직 쓰지 않은 후보를 제거한다.
  - S10 종료처럼 승강을 유지하는 정지는 제어 정지다(4c: 래치 → 세대 변경·큐 폐기 → 측정 위치 정지 DRIVE → 그 seq 를 같은 부팅·무장 세대의 STATE 로 100 ms 안에 확인, 아니면 DISARM).
- **재무장**: 지금 lease 의 운용자 재무장 요청이 래치만 푼다. S10-A 는 이전 미션을 재개하지 않는다. 정지 처리 뒤 새 제출을 받고, 포켓에 들어간 뒤였다면 현장 복구가 먼저다(4d-2b).
- 래치를 푸는 조건은 셋 다다: 신선한 `DISARMED`, 옛 명령 해소, 그 뒤 고유 표본으로 확인한 정지. **outstanding 을 모르면 거부한다**(4b). 바퀴 0 인 승강·`keep` 을 포함한 큐·부분 송신·송신 완료 명령의 추적은 4c 에서 한다(지금 `older_nonzero_may_act()` 는 바퀴만 봄). 입력이 돌아오는 것만으로는 재개하지 않는다.
- 감시는 이 상태를 본다: `armed = state in (ARMING, READY, DRIVING)`(중재기는 AUTO 진입 때 `armed` 를 내린다).
- **중재기 설정 `ArbiterConfig.auto_lease_loss = "continue" | "stop"`**
  - 기본 `continue`: 지금 그대로(세대 증가, AUTO 출력, 미션 유지).
  - 실차 `stop`: AUTO 에서 lease 를 잃으면 `STOPPED`(`PAUSED` 가 아님 — `rearm` 은 `STOPPED` 에서만 받는다).
  - 공유 `preempt_latch` 는 정지 확인으로 자동으로 풀리므로, 실차 재무장 강제는 실차 허가 상태의 `LATCHED` 가 맡는다.

## 8. 4a 첫 프레임의 정지 판정 (코어, 알려진 시작에서만)

- 지금 `_known_first_frame` 은 `SafetySpeed.standing(0.01)` 을 쓴다. 실차 엔코더 모델에서는 가속도 오차 항만 0.024 m/s 라 서지 못한다.
- 알려진 시작에서는 `slam["stop_now"]`(표본 기준 정지 검출기; 4c 가 새 표본마다 갱신)로 판정한다. 알려진 시작은 SLAM 을 요구하므로 늘 있다. Isaac 의 인식 시작 경로는 그대로다.
- 정지 뒤 출발은 남은 운동 방향을 판정할 수 있을 때만이다. 지금 `direction_unsure` 가 막는 것을 4c 에서 실제 엔코더 모델로 시험하고, 실차 삽입은 PR 5 의 방향 판정과 PR 7 의 깊이 입력 검증에 묶는다.
- 첫 프레임 뒤 중간·삽입 전 정지 프레임(`await_still_frame`)의 공급은 실차 프레임 원천(PR 7)이다. 취득 시각의 자세·승강·미션·단계·보정에 묶어 검사기가 새 정지 프레임을 받아들였을 때만 푼다. 4c 는 가짜 깊이로 이 경계를 본다.

## 9. 기록

조립 기록 `assembly.json`:
- 옵션 감사(값 전부), 훅 표와 구현 여부, 설정 파일과 해시.
- planner·tracker·preview 의 곡률 상한과 `RealKinematics` 범위, 조향률 출처.
- 준비 판정 결과.

## 10. 시험 (가짜 원천, Isaac 없음)

- 옵션: 필수 값 각각 빠짐·틀림, 근접 추적·밑판 검사 켬, 곡률 불일치·상한 초과 → 거부. `--dry-assemble` 종료 코드(구조 오류 2, 미준비 0).
- 훅 표: S10-A 경로를 가짜 원천으로 돌려 읽힌 이름 = 표, 반환형·실패 사유. 참값 훅 → TruthAccessViolation.
- 부팅·설정: 대상 등록, 첫 제출이 명령 경로를 통과, 운용자 시작 확인 없으면 거부, 기준선 캡처 뒤 첫 채택에서 미션 키가 깨끗함.
- 준비 판정: submit/drive 각 조건을 하나씩 깨면 그 사유. hold 중 drive 판정은 받고, 적용 보정과 스냅숏 보정이 다르면 닫힘.
- AUTO 허가(실제 `McuLinkHost` + 가짜 MCU 로 ARM → 0 DRIVE → `ZERO_SEEN`):
  - 계획 중 ARM 없음, 채택 뒤 한 번, ready 전 DRIVE 없음.
  - 무장 중 Blocked·시한 초과·권한 철회 → 래치.
  - 각 래치 사유 → 즉시 DISARM·후보 제거.
  - outstanding 모름이면 재무장 거부, 입력 복구만으로 재개 없음.
- 중재기: `stop` 이면 AUTO lease 상실 → `STOPPED`. 기본 `continue` 는 세대 증가·AUTO 출력·미션 유지(고정 시험).
- 4a 첫 프레임: 알려진 시작은 `stop_now` 로 판정(SafetySpeed 경계가 넓어도 받음), 인식 시작은 그대로. 게이트 29/29.

## 11. 4c 로 넘기는 것

- 50 Hz 루프: 주기 초과 두 조건(연속 N 회·단일 장기 초과, 지금 텔레옵 `loop_slow` 와 다름)과 단계별 기록, STATE 를 새 표본마다 한 번 `RealOdometry` 에.
- 최종 제출 직전 STOP 재확인, outstanding 추적(바퀴 0 승강·`keep`·부분 송신 포함), 제어 정지 계약과 `PortResult.seq`.
- `take_s10_stop()` 소비 → S10 종료를 배달 판정 없이 한 번 수거 → `end_reset()` 까지 실행해 다음 제출과 텔레옵이 닫히는지 확인. 종료 뒤 타이머 건너뛰기.
- 가짜 포트 S10-A 운반 높이 정지(상위 계획 표의 '하역·복귀 완주' 문구는 S10-A 정지로 맞춤), 운영 장치에서 AUTO ARM·DRIVE 불가 시험.

## 12. 자체 검토

- Codex 1 회차(1판): P1 6, P2 4, P3 2. 모두 반영했다.
  - P1: 제출 ACK ≠ 채택 → 채택 뒤 한 번 무장. 주행 중 tracking 요구 → hold 허용 + 신선도 판정. 공유 래치로 재무장 강제 불가 → 실차 상태가 래치, lease 상실은 STOPPED. 바퀴 0 승강 outstanding → 4b 는 모름이면 거부, 4c 추적. 실차 엔코더로 첫 프레임 정지 불가 → `stop_now`. 실 MCU 경계 → 4b 장치 안 엶, 4c 운영 장치 불가 시험.
  - P2: 증거 공급 인터페이스, 훅 경로 표, S10-A 설정·대상 등록·시작 확인, 단계별 책임.
  - P3: 조향률 키, dry-assemble 종료 코드, `loop_slow` 차이·상위 계획 문구.
