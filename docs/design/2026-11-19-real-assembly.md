# 실차 조립 · AUTO 허가 상태 — 4b (설계)

작성 2026-11-19 · 6판(Codex 5 회차 수렴 + 새 스레드 전제 검토 반영) · 상위 [실차 자율 계획](../plans/2026-11-18-real-autonomy-entry.md) PR 4 · [제어 틱 계약](2026-11-18-real-control-tick.md) · 앞 단계 [4a 알려진 팔레트 시작](2026-11-19-known-pallet-start.md)(#598)

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
- 팔레트 자세는 설정이 아니라 운용자가 미션마다 입력한다(4a, `operator_input`). 순서는 **등록 뒤 제출**이다.
  - 지금 lease 의 운용자가 그 팔레트의 지도 자세를 등록한다(`update_pose(basis="operator_input")`). 그러면 대상 revision 이 오르고, 그 revision 으로 제출한다.
  - 제출이 승인되는 순간의 `operator_input` 자세와 revision(`Submit.targets_revision`)을 그 미션에 고정해 `known_pallet_request` 에 넘긴다. 처음 요청과 재요청 모두 그 고정 revision 을 작업 문맥에 쓰고, 작업을 넣기 전에도 지금 값과 비교해 다르면 `prepare:known:targets_changed` 로 끝낸다. 준비 중 등록이 바뀌면(`_job_invalid` 의 `targets`) 알려진 시작은 다시 묻지 않고 준비를 끝낸다(`prepare:known:targets_changed`, 코어의 작은 변경). 새 자세는 새 제출로 받는다.
  - 그 팔레트에 `operator_input` 자세가 없으면(ID 만 등록, 또는 다른 출처) 제출을 거부한다.
  - 처음 등록과 현장 복구 뒤 재등록(`unlocated → available`)은 같은 흐름이다.
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
- `settings`: 코어가 직접 읽는 키 — `drive_acceleration_mps2`·`lift_rate_mps`·`lift_target_m`(운반 높이)·`steering_command_rate_rad_s`·`withdraw_speed_mps`, 그리고 추종기의 `tracker_curvature_inv_m`.
- `options.speeds[단계]` 와 접근 직선 속도는 `s10a.yaml` 의 승인된 저속 상한 이하여야 한다. 상한 `max_speed_mps` 는 후륜축 속도이고, 유한한 양수가 아니면 조립을 거부한다. 4c 는 최종 후보의 후륜축 속도를 그 상한으로 자른 뒤, 그 속도와 최종 조향 곡률로 좌우 바퀴 목표(rad/s, `RealKinematics`)를 함께 계산한다.
- 시험은 `--dry-assemble` 과 같은 조립 결과로 첫 채택까지 간다(시험이 따로 설정을 보충하지 않음).
- `Arbiter(mission=core.commands, config=… auto_lease_loss="stop")`, `core.jobs = PlanWorker(...)`, 마지막으로 `core.baseline = MissionBaseline.capture(core)`.

## 6. 준비 판정과 증거 공급

- **위치·장애물 증거 — 모든 단계 공통**:
  - 연속 오도메트리.
  - 신선한 설치 스냅숏: 적용 중인 보정의 값·세대에 묶이고, 스캔 신선도는 설치 결과의 최신 스캔 취득 시각으로 판정(스냅숏 `stamp_s` 는 재투영 시각이라 쓰지 않음).
  - 최신 유효 SLAM 응답의 취득 시각이 나이 상한 안.
  - 제출 때, 그리고 채택 직전(`live_mission_ready()` 가 검사하고 실패하면 `prepare:reference:…` 로 준비 실패)에도 요구한다.
  - 판정 순서: hold 이동 거리를 갱신하는 `map_from_base()` 가 그 틱에 돈 뒤에 판정한다(4c 루프 순서).
  - 상태를 둘로 나눈다. **새 스냅숏 설치 대기**(보정이 바뀐 직후)는 출력만 닫고 래치하지 않는다. **센서 만료**(나이 상한 초과)는 래치 사유다.
- **SLAM 모드**: 제출·채택에는 `tracking`. 무장·주행에는 이 미션의 정상 `holding` 도 받는다(`transition("approach")` 가 hold 를 건다; hold 이동 한도 안).
- **잠금**: 재위치·현장 복구 잠금이 없어야 한다(`real_auto_refusal`). 운용자 시작 확인(2절)도 필요하다.
- **MCU 검사는 단계별로 나눈다**(`RealPlantPort.health()` 는 `DISARMED`·`zero_pending` 을 사유로 내고 텔레옵 `_arm_blocker()` 는 `ARMED` 를 거부하므로, 그대로 공용으로 쓰지 않는다).
  - 공통: STATE 신선도·빌드·시계율 검증.
  - ARM 직전: `DISARMED`, 신선한 정지, 옛 명령 해소.
  - ARMING: 그 ARM 과 확인용 0 의 진행.
  - 주행: 같은 부팅·권한 세대의 `ARMED`·`ready`.
  - 무장 전 시계율·지연 추정에 쓰는 DISARM 응답 경로(텔레옵의 `_ping`)는 4c 배선에 둔다.
- 그 밖: 작업자 준비(`PlanWorker._dead` 없음 → 조립 쪽 `worker_ok()`), 층 있음, 기준선 캡처됨, 측정 유효(조향·승강, 계수 결함 비트).
- 증거 공급 인터페이스(4b 는 인터페이스와 가짜, 4c 가 새 STATE·호스트 이벤트에 연결):
  - `odometry_continuous()`: `RealOdometry.update()` 의 부팅 변경·시간 공백. 단절이면 이전 위치·정지 증거를 버리고 `stop_now` 를 즉시 무효로 한다.
  - `stop_confirmed()` 와 그 정지가 시작된 표본 시각: 새 고유 표본 기준.
  - `scan_fresh()`, `slam_fresh()`.
- 준비가 안 되면 미션 제출은 그 사유로 거부되고(`prepare` 전), 구동 출력도 없다.

## 7. AUTO 허가 상태

- 상태: `OFF → ARMING → READY → DRIVING`, 그리고 `STOP_PENDING`(제어 정지의 확인 대기)·`DISARMING`·`LATCHED`.
- **AUTO 요청 = 새 사용자 `mission_submit` 한 건**(S10-A, 일회성). 콘솔의 `arm`·`mode_request(AUTO)` 는 쓰지 않는다(지금 TELEOP 전용·거부).
- **무장 대기는 별도 분기**다. 채택한 틱부터 `ZERO_SEEN` 확인까지 루프는 링크·SLAM·층·감시만 갱신한다. 추종·도착·승강 램프·포켓 채택은 부르지 않는다. 작업 시계는 기존 계획 대기 시계(`commands.begin_wait`/`end_wait`)로 멈춘다.
- 무장 순서:
  1. `job_hold()` 가 성공 답을 채택해 `adopted_id == mission_id`·`running`·`approach` 가 되면, 지금 lease·세대를 다시 검사하고 `begin_wait`.
  2. 제어 세대 변경과 큐 폐기 뒤 ARM 직전 검사(6절)를 통과하면 `host.request_arm()` 을 **한 번** → `ARMING`.
  3. 호스트가 확인용 0 DRIVE 한 건을 직접 보내고, `ZERO_SEEN` 으로 `ready` 가 되면 주행 검사(같은 부팅·권한 세대) → `READY`, `end_wait`.
  4. 첫 허가 틱은 확인된 바퀴 0 과 신선한 실측 조향·승강에서 후보를 만든다. 첫 미션 DRIVE 가 큐에 들어가면 `DRIVING`.
  - `Blocked`, 무장 시한 초과(호스트 `arm_timeout_s` 0.5 s), 무장 중 권한 철회는 래치 사유다.
- **래치(`LATCHED`)**: 다음 사유가 오면 실차 허가 상태가 직접 받는다.
  - STOP, 운용자 DISARM, lease 상실, 세션 종료, 링크 상실.
  - 측정 무효, 센서 만료, 포트 `refused`/`blocked`, 감시 trip(과속·주기 초과·SafetyWatch), `real_auto_refusal` 사유.
  - **처리 순서**(어느 사유든 같은 공통 처리):
    1. 첫 바이트를 아직 쓰지 않은 후보와 이전 요청을 버린다.
    2. DISARM 을 큐에 넣고 그 틱의 송신을 시도한다.
    3. 중재기를 `stop()` 으로 `STOPPED` 로 보낸다. 같은 틱에 STOP·lease 상실로 이미 `STOPPED` 면 세대를 다시 올리지 않는다.
    4. 달리던 미션을 끝내 수거한다.
  - 미션 수거: 기존 취소 판단(`cargo.cancel_decision`)으로 `cancelled` 이거나, 포켓에 들어간 뒤면 `recovery_required` → 현장 복구. 그래서 재개할 미션이 남지 않는다.
  - 수거는 SLAM 이 무효여도 종료 기록과 복구 잠금을 남긴다. hold 해제는 유효한 새 위치 증거가 생길 때까지 미룬다(4c: 코어의 기존 미룬 해제 `end_release_due` 경로를 쓴다). 처음 수거와 이후의 해제 시도(`release_gate`) 양쪽에서 정지와 함께 SLAM 신선도를 확인한다. 실차 루프는 SLAM 이 유효하지 않으면 해제 시도를 부르지 않는다. 만료된 SLAM 을 `slam_rear` 로 다시 읽지 않는다. 여러 틱 동안 만료된 채여도 종료 기록은 한 번이고 해제만 기다린다.
  - 수거 중 예외가 나도 1–3 은 이미 끝나 있다.
- **S10 제어 정지(유한한 절차: `STOP_PENDING` → `DISARMING` → `LATCHED`)**: `take_s10_stop()` 이 오면 그 틱에 다음을 한다.
  1. 래치 → 세대 변경·큐 폐기 → 측정 위치 정지 DRIVE(바퀴 0, 승강 측정 위치).
  2. 그 seq 를 같은 부팅·무장 세대의 STATE 로 100 ms 안에 확인하면 바로 DISARM → `DISARMING`, 신선한 `DISARMED` 가 오면 `LATCHED`.
  3. 확인 실패·거부·새 긴급 정지·감시 trip 은 즉시 DISARM 이다.
  - 펌웨어는 마지막 DRIVE 가 만료되면 `STOPPING → DISARMED` 로 가며 승강 위치 제어를 끝낸다. 그래서 무장한 채 승강을 유지하는 상태는 두지 않는다. DISARM 뒤 운반 높이의 포크는 **무전원 하중 유지 조건**(상위 계획 S10 선행 조건 — 확인 전에는 빈 포크만)에 맡긴다.
  - 확인 대기 동안 미션 출력은 닫히고 안전 감시는 켠 채다.
  - 정상 `end_reset()` 이 게시하는 `recovery_required`(현장 복구)는 후속 AUTO·텔레옵만 막는다. 이미 큐에 넣은 그 정지 DRIVE 를 폐기하지 않는다(시험: 실제 만료를 반영한 MCU 모형으로).
  - 현장 복구 완료(`field_recovery_done(disarmed=True)`)는 신선한 `DISARMED` 뒤에만 받는다.
- **재무장**:
  - 지금 lease 의 운용자 `rearm` 이 래치만 푼다. 실차 래치는 중재기가 지금 lease·제어 세대를 통과한 새 `rearm` 으로 **실제로 `STOPPED → PAUSED` 를 확정하는 곳**에서 함께 풀린다(중재기에 `on_rearmed` 콜백, 기본 None). 저장된 ACK 의 재전송·중복 요청은 풀지 않는다(새 STOP 뒤 옛 `rearm` 재전송에도 두 래치 유지 — 시험). 새 미션은 새 제출로 시작한다.
  - 중재기에 `rearm_refusal` 콜백(기본 None = Isaac 그대로)을 넘긴다. 실차 조건을 **ACK 와 모드 변경 전에** 확인하고, 실패하면 래치와 모드를 모두 유지한다.
  - 실차 조건(셋 다): 신선한 `DISARMED`, 옛 명령 해소, 그 뒤 고유 표본으로 확인한 정지. outstanding 을 모르면 거부한다(4b). 바퀴 0 인 승강·`keep`·부분 송신·송신 완료의 추적은 4c 에서 한다(지금 `older_nonzero_may_act()` 는 바퀴만 봄).
  - 입력이 돌아오는 것만으로는 재개하지 않는다.
- 감시는 이 상태를 본다: `armed = state in (ARMING, READY, DRIVING, STOP_PENDING, DISARMING)`(중재기는 AUTO 진입 때 `armed` 를 내린다).
- **중재기 설정 `ArbiterConfig.auto_lease_loss = "continue" | "stop"`**
  - 기본 `continue`: 지금 그대로(세대 증가, AUTO 출력, 미션 유지).
  - 실차 `stop`: AUTO 에서 lease 를 잃으면 `stop()` 으로 `STOPPED`(제어 세대는 한 번만 바뀜, `rearm` 은 `STOPPED` 에서만 받음).
  - 제출 ACK 와 채택 사이의 lease 상실도 시험한다.
  - 공유 `preempt_latch` 는 정지 확인으로 자동으로 풀리므로, 실차 재무장 강제는 실차 허가 상태의 `LATCHED` 와 `rearm_refusal` 이 맡는다.

## 8. 4a 첫 프레임의 정지 판정 (코어, 알려진 시작에서만)

- 지금 `_known_first_frame` 은 `SafetySpeed.standing(0.01)` 을 쓴다. 실차 엔코더 모델에서는 가속도 오차 항만 0.024 m/s 라 서지 못한다.
- 알려진 시작에서는 `slam["stop_now"]`(표본 기준 정지 검출기; 4c 가 새 표본마다 갱신하고 표본 공백·연속성 단절에서 즉시 무효)로 판정한다. 알려진 시작은 SLAM 을 요구하므로 늘 있다. Isaac 의 인식 시작 경로는 그대로다.
- **첫 프레임은 정지 구간에서 취득한 것만** 받는다. 정지 구간은 신선한 정지 측정과 이전 구동 영향 해소(포트에 남은 비영 DRIVE 없음 — `StopDetector` 는 명령과 속도만 읽어 스스로 모른다)가 함께 성립하는 동안이다. 4c 루프가 `slam["stop_now"]` 를 그 둘의 곱으로 넣는다. 새 구동이나 표본 공백이 구간을 끊는다.
- 대기 중에도 MCU 의 DRIVE 만료로 무장이 풀리지 않도록 루프는 **대기 프레임**을 낸다: 바퀴 0, 조향은 측정값 유지, 승강은 `target` = 측정 위치(이전 이동 목표를 이어 가는 `keep` 이 아님). 대기 프레임은 '이전 구동 영향'으로 세지 않는다. 이전 이동 목표를 잇는 `keep`·비영 바퀴는 센다. 4c 는 실제 만료를 적용한 가짜 MCU 로, 대기 중 무장이 유지되며 첫 프레임이 채택되는 경우와 이전 이동 명령이 남아 채택이 막히는 경우를 모두 시험한다.
- 구간이 시작된 시각을 `pocket["first_frame_still_since"]`(미션 키 — baseline 에 추가, 다음 미션에 남지 않음)에 두고, 서 있지 않으면 지운다. 프레임의 `stamp_s` 가 그 시각 이후여야 하고, 프레임에 붙은 미션 세대·단계·적용 보정이 지금과 같아야 한다(`known_pocket_frame` 반환에 `mission_gen`·`phase`·`correction` 추가). 제동 중 찍혀 늦게 온 프레임은 거부된다.
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
  - 계획 중 ARM 없음, 채택 뒤 한 번. ready 전 **미션** DRIVE 없음(호스트 확인용 0 DRIVE 한 건은 `ZERO_SEEN` 전에 나감). 무장 대기 중 추종·승강·작업 시계 정지, 첫 허가 틱 후보는 바퀴 0·실측 조향에서.
  - S10 제어 정지: ACK 대기 중 감시 켜짐, ACK 뒤 DISARM → `DISARMED` → `LATCHED`, 그 뒤 DRIVE 없음, `DISARMED` 확인 전 현장 복구 완료 거부, 시한 초과 → DISARM, 복구 잠금 게시가 큐의 정지 DRIVE 를 폐기하지 않음.
  - 긴급 래치 순서: SLAM 만료로 래치될 때 DISARM 이 큐에 들어가고 **가짜 전송기의 송신 호출이 수거보다 먼저**다(수거가 예외를 내도 그 순서). 수거와 이후 해제 시도가 만료 SLAM 을 다시 읽지 않고, 여러 틱 만료여도 종료 기록은 한 번이다.
  - 등록 revision: 제출 ACK 뒤 첫 요청 전·보정 변경으로 버린 뒤 재요청 전에 등록이 바뀌면 `targets_changed` 로 끝남.
  - 재무장: 새 STOP 뒤 옛 `rearm` 재전송·중복에도 래치 유지. 모든 래치 경로에서 중재기 `STOPPED`(세대는 한 번).
  - 등록: 준비 중 자세 변경 → 준비 종료(재질문 없음), `operator_input` 자세 없는 제출 거부.
  - 래치 때 달리던 미션 종료·수거(포켓 진입 뒤면 현장 복구), 재무장 조건 실패면 ACK·모드 변경 없음.
  - 무장 중 Blocked·시한 초과·권한 철회 → 래치.
  - 각 래치 사유 → 즉시 DISARM·후보 제거.
  - outstanding 모름이면 재무장 거부, 입력 복구만으로 재개 없음.
- 중재기: `stop` 이면 AUTO lease 상실 → `STOPPED`. 기본 `continue` 는 세대 증가·AUTO 출력·미션 유지(고정 시험).
- 4a 첫 프레임: 알려진 시작은 `stop_now` 로 판정(SafetySpeed 경계가 넓어도 받음), 정지 전 취득·다른 세대·다른 보정의 프레임 거부, 인식 시작은 그대로. 게이트 29/29.
- 운영 깊이 원천은 PR 7 전까지 None 이다. 4c 완주 시험은 가짜 깊이와 받아들일 수 있는 방향 증거를 주입한다(엔코더 `direction_unsure` 차단 시험과 따로).

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
- Codex 2 회차(2판): P1 4, P2 4, P3 1. 반영했다.
  - P1: 무장 대기는 별도 분기(계획 대기 시계로 작업 시간 정지, 첫 후보는 바퀴 0 에서). MCU 검사는 단계별. 첫 프레임은 정지 구간 취득·같은 세대·보정만. S10 제어 정지는 `STOP_PENDING`/`HOLDING` 에서 감시 유지.
  - P2: 재무장은 `rearm_refusal` 로 ACK 전에 막고 래치 때 미션 종료·수거, lease 상실은 `stop()` 한 번. 제출·채택에도 신선한 장애물 증거(설치 대기와 만료 구분). `tracker_curvature_inv_m`·`speeds`·저속 상한. 등록 뒤 제출.
  - P3: 시험 문구(미션 DRIVE·확인용 0 DRIVE, 가짜 깊이).
- Codex 3 회차(3판): P1 2, P2 3, P3 1. 반영했다.
  - P1: 무장한 승강 유지 상태 없음 — S10 은 정지 DRIVE 확인 → DISARM 의 유한 절차이고 승강은 무전원 유지 조건. 래치는 후보 폐기 → DISARM 송신 → 중재기 STOPPED → 수거 순서, 무효 SLAM 수거는 해제 보류.
  - P2: 모든 래치에서 중재기 STOPPED, 래치는 rearm ACK 확정 때만 풀림. 제출 자세·revision 고정, 준비 중 변경은 종료. 정지 구간은 이전 구동 해소와 함께, `first_frame_still_since` 는 미션 키.
  - P3: 저속 상한의 단위(후륜축 m/s)와 좌우 바퀴 목표 계산 위치.
- Codex 4 회차(4판): 새 P1 없음, P2 4, P3 1. 반영했다.
  - P2: 미룬 해제도 SLAM 신선도를 확인. 고정 revision 을 모든 작업 요청에. 래치 해제는 중재기의 실제 `STOPPED → PAUSED` 전이에(`on_rearmed`). 첫 프레임 대기 프레임(바퀴 0·조향 유지·승강 `target` = 측정)과 이전 이동 `keep` 을 구분.
  - P3: 송신 호출 순서 시험.
- Codex 5 회차(5판): 새 P1/P2 없음(수렴).

## 13. 적히지 않은 전제 (Codex 새 스레드, 12 절을 보지 않음) — 4b 에 넣는 것

- **세션 시작은 늘 정지 쪽부터**: 실행기는 시작하면 먼저 DISARM 을 내고, 신선한 `DISARMED` 를 기다린다. 그다음 DISARM 응답으로 시계율·지연을 확인하고, 정지 표본을 모은다. 그 뒤 테이프 위치에서 SLAM·층이 준비되면 운용자 확인 → 등록 → 제출 순서다. 차를 움직이지 않고 모은다. 이전 실행기가 무장한 채 끝났어도 첫 조치는 DISARM 이다.
- **새 세션·재시작은 현장 복구를 대신하지 않는다**: 코어의 초기 `free/none` 은 측정이 아니다. 매 세션 첫 제출 전에 운용자의 물리 확인(빈 포크·포켓 밖·승강 내림)이 필요하다(2절).
- **사람 확인의 유효 범위**: 확인은 세션·lease 보유자·다음 제출 한 건에 묶인다. 차가 움직이면(텔레옵·AUTO), lease 가 바뀌면, 링크를 잃으면, 현장 복구가 열리면 무효다.
- **모든 미션 끝은 MCU 권한을 끝낸다**: 취소·일시정지·첫 프레임 5 s 만료·실패·S10 어느 것이든 `ARMING`/`READY`/`DRIVING` 을 래치 처리(7절 순서)로 빠져나간다. S10-A 에서 일시정지는 정지로 다룬다(래치·수거, 재개 없음).
- **MCU 결함과 재무장은 따로다**: DISARM 은 `FAULT`/`ESTOP` 를 지우지 않는다. 결함 해제는 현장 원인 해결 뒤 운용자의 별도 동작이다. `rearm_refusal` 은 같은 부팅의 신선한 `DISARMED`(결함 없음)·옛 명령 해소·그 뒤 정지를 확인한다.
- **복구 뒤 새 위치는 구동 없이**: 재위치 확인은 테이프 재배치와 정지 스캔으로 한다(배선은 PR 5/7). 텔레옵 이동을 요구하지 않는다 — 두 잠금이 서로 막지 않게.
- **시각은 한 축**: 정지·스캔·SLAM·깊이의 취득 시각은 모두 호스트 단조 시계로 옮겨 비교한다(MCU 시각은 링크의 시계 대응, ROS·카메라 stamp 는 PR 7 변환). 수신 시각이나 멈추는 작업 시계를 대신 쓰지 않는다. 4b 의 증거 인터페이스는 이 축의 시각을 돌려준다.
- **출력 닫힘과 MCU 만료**: 보정 뒤 스냅숏 대기처럼 짧게 출력을 닫을 때는 대기 프레임(8절: 바퀴 0·조향 유지·승강 `target` = 측정)을 새 세대로 내서 무장을 유지한다. 옛 비영 목표는 그 프레임으로 바로 대체된다. 증거 만료는 래치다.
- **승강 측정의 원점**: 준비 판정은 승강 측정이 유한할 뿐 아니라 원점·범위가 유효하다는 증거를 요구한다(`lift_valid()` 인터페이스; 센서 A/B 결정 뒤 실구현, 가짜는 그 증거를 준다).
- **SLAM 신선도 ≠ 위치 신뢰**: 위치 증거의 신선도는 새 위치를 계산한 응답만 센다(`skipped` 는 세지 않음). 첫 위치·복구 뒤 새 위치의 입증은 재위치 확인이 따로 한다.
- **지도 식별**: `s10a.yaml` 은 조사 지도 묶음의 식별자를 묶는다. 불러온 지도와 다르면 조립을 거부한다. 시작 자세 허용 오차는 현장에서 확인한다.
- **계획용 목적지가 실제로 계획되는가**: `--dry-assemble` 은 설정의 기준 팔레트 자세(`check_pallet_pose`)로 테이프 시작에서 `plan_transport` 를 한 번 돌려, 목적지·대기 자세가 막혀 있지 않은지 본다.
- **가짜는 원천만 바꾼다**: 허가·준비·래치 코드는 진짜를 쓴다. Jetson 50 Hz 여유, USB 단절, 실제 제동 거리와 포크 거동은 HIL·S10 실측으로 남긴다.
- 4b 밖(기록만): 상시 현장 감독·주 전원 차단 담당(S10 조건), 적재 판정의 시야(PR 5/7), 단일 송신자와 실제 콘솔 STOP 배선(PR 5), 2D LiDAR 사각의 시험장 통제(PR 5/7·S10), D435i 포켓 깊이(PR 7), 확인 프레임 수락 ≠ 물리 정지(4c·HIL).

## 14. 구현에서 정한 것 (4b)

- 모듈: `live/real_autonomy.py`(`AutoPermit`, `real_options`·`real_option_refusal`, `S10AConfig`·`s10a_refusal`, `Evidence`·`SlamView`·`SnapshotView`·`MeasurementView`·`auto_ready_refusal`·`latches`), `live/real_assembly.py`(`assemble`, `RealHooks`, `FrozenSubmit`·`freeze_submit`·`known_request`, `OperatorStart`, `register_pallet`·`s10_submit_refusal`), `tools/real_autonomy.py --dry-assemble`, `config/real/s10a.yaml`(측정 전 값은 null — 조립 거부)·`config/real/autonomy.yaml`(잠정 설정).
- 허가 증거:
  - `outstanding(now)` 은 `("settled", t) / ("pending", None) / ("unknown", None)` 을 돌려준다. 4c 가 송신 경로 전체를 추적하기 전까지 `unknown` 은 재무장을 거부한다.
  - `stop_since(now)` 은 `(t, boot_id)` 를 돌려준다. 옛 명령 해소 뒤, 같은 부팅이어야 한다.
  - 정지 프레임의 쓰기 시각은 `stop_written(seq, t)` 로 받는다.
  - 콜백 `transmit`·`arbiter_stop`·`collect` 는 `bind` 로 한 번만 묶는다.
- 래치: 허가를 먼저 닫고, 네 단계가 앞 단계 예외와 무관하게 모두 돈다(첫 예외는 뒤에 다시 냄). 반복 래치는 DISARM 만 다시 하고, 수거는 한 번이다.
- 준비 판정 순서: 래치 사유 전부 → `wait:snapshot_installing`. `SLAM_MAX_AGE_S` 0.5, `HOLD_MAX_M` 1.5, `SCAN_MAX_AGE_S` 0.35 는 잠정값이다.
- 훅 표:
  - 준비·채택은 `ADOPTION_HOOKS`, 첫 프레임 대기·S10 끝은 `STAND_AND_END_HOOKS` 다. `exit_prefix` 는 선택이고 실차에는 없다.
  - 추종·명령·허가 틱의 표는 4c 의 루프와 함께 고정한다.
  - `grid_kwargs("mission", target)` 은 등록 자세를 접근 장애물(`pickup_obstacle`)로 준다.
- `--dry-assemble`:
  - 지도 묶음(`--map-bundle`)은 `load_bundle(expected_map_id=s10a.map_id)` 로 확인하고, 그 격자(회전 원점은 거부)로 기준 계획을 푼다.
  - 설정 파일이 깨졌거나 키가 빠지면 종료 2 다.
  - 기록: 파일 해시, 실제 곡률 상한(planner·travel·tracker·preview), 옵션 값.
- 남긴 것(Codex P3): `FORKLIFT_LIVE_START_DIGEST=1` 의 U21 기록에 `pocket.first_frame_still_since: "absent"` 가 한 줄 더 생긴다(4a 의 `first_frame_since` 와 같은 종류). 진단 기록이라 그대로 둔다.
- 세션 확인(Codex 4b 5 회차): 첫 DISARM 의 seq 를 정확히 단 `DISARMED` 를 받은 뒤, 1024–64512 떨어진 seq 로 두 번째 DISARM 을 쓰고 같은 부팅에서 그것도 정확히 답해야 `session_started` 다(텔레옵 다리의 세션 확인과 같은 생각). 실행기의 호스트는 무작위 seq 로 시작한다(4c 배선).
- 도킹 격자: 등록 외곽 안의 셀만 지운다(여유 0). 자세가 어긋난 팔레트는 계획 실패로 끝난다(안전한 실패 — 다시 재서 등록).
- **PR 5 요구(Codex 4b 5 회차 P2, 4b 범위 밖으로 둠)**: 실차 층(`ObstacleGrid`)은 반사를 격자 셀로 팽창하므로, 정확히 등록한 팔레트의 전면 반사도 외곽 밖 셀(예: 전면 2.70 m 앞 2.675 m 셀)로 남아 삽입 계획을 막는다. 실차 층 배선에서 등록 팔레트로 확인된 반사를 팽창 **전에** 분리하고 나머지 점유는 유지해야 한다. 회귀 시험은 `ObstacleGrid → plan_transport` 경로로 한다. 4b 에는 격자를 만드는 쪽이 없어(장치·층 없음) 함수와 수동 격자 시험만 둔다.
