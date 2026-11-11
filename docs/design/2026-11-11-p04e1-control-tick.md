# P0-4 4e-1 — 공통 조립과 제어 틱 분리, 50 Hz 상수 목록 (설계 초안, 2026-11-11)

계획 `docs/plans/2026-11-11-sim-to-real-bringup.md` 2.2 의 '공통 조립 경로'와 '벽시계 루프'의 앞 단계다. 실차 진입점(4e-2)은 `drive()` 를 쓰지 않고, 여기서 떼어 낸 공통 조립과 제어 틱만 다시 쓴다. Isaac 은 120 Hz 1:1 그대로다(총괄 조건). 4e-1 은 동작을 바꾸지 않는 이동이고, 재생 게이트 29/29 와 전체 pytest 로 확인한다.

## 1. 지금 구조 (조사 2026-11-11, `sim/isaac/run_transport.py`)

- `drive(S, isaac)` 는 5252 행부터 파일 끝까지 이어진다. 설정이 약 4,500 줄이고, 루프가 9809–11126 행, 마지막 기록이 11180 행부터다.
- 루프는 솎아내지 않는다. 한 바퀴가 제어 틱 하나와 물리 스텝 하나(`stepper["fn"]`)다. 예외는 포획이다: batch 포획은 틱 안에서 `cells.stepper` 로 여러 스텝을 진행하고, live D9c 포획은 틱에 걸쳐 진행한다(그래서 층 감시는 틱 머리의 `t` 가 아니라 `layer_clock_box_` 로 지금 시각을 다시 읽는다, 4880–4882). 순서는 다음과 같다.
  1. 머리(시계·명령·live tick)
  2. Isaac 참값 읽기(`rear`·`velocity`·`signed_speed`) → SLAM 응답·층 갱신
  3. 참값 평가 검사(기울기·발자국·미끄럼)
  4. **제어**(`core.*`: release_gate → job/관측 → 추종 블록 → lift_and_timers → permission/permission_watch → paused_tick/live_leg)
  5. 층 감시 → 새 장애물(Isaac) → estop → low_clearance → `assemble_command`
  6. live 출력
  7. 구동(`plant`)
  8. 추적 카메라
  9. 물리 스텝
- 결합 위험: MissionCore 훅(7114–7203)과 `drive()` 안 클로저가 루프가 다시 묶는 지역 이름을 잡는다.
  - 직접 λ 에서 11 개, 전달된 함수의 간접 의존까지 14 개다(Codex 대조): `rear`, `truth_rear`, `base`, `q`, `ppos`, `pq`, `velocity`, `loaded`, `signed_speed`, `steering_command`, `truth_speed`, `forward`(`cusp_heading_motion`), `t`·`step`(`steering_position_`, `dump_tracking`).
  - 훅 밖에도 `grid_kwargs()`·`correction_grid()` 가 `rear`·`loaded` 를, `read_pocket_frame()` 이 `requested_speed` 를 읽는다.
  - **쓰기도 루프 밖에 있다**: `set_rear()`(7001)는 `nonlocal rear` 로 core 호출 도중 즉시 바꾸고, core 는 같은 호출 안에서 `grid_kwargs()` 를 부른다(core 2528 → 2535).
  - `grid_kwargs()` 는 초기 계획에서 아직 없는 `rear`·`loaded` 를 `except NameError` 로 다룬다(5968–5977).
- 제약:
  - `tests/unit/test_runner_drive_split.py` 가 `drive` 서두와 Isaac 이름을 고정한다.
  - 약 39 개 시험 파일이 `run_transport.py` 원문을 `index`·`in` 으로 고정한다. 들여쓰기를 포함하는 것도 있다(`for step in` 줄, `stepper["fn"](...)` 줄).
  - #442 의 다섯 자리는 제자리에 둔다(총괄): 층 설정 6289, `loaded_wait_` 뒤 감시 10702, `layer_clock_box_` 5393, `initial_time` 뒤 시계 연결 6822, 마지막 기록 11352.

## 2. 나누기 (제안)

한 번에 옮기면 원문 고정 시험과 클로저 결합이 동시에 깨진다. 셋으로 나눈다.

### 4e-1a — 틱 변수 상자와 50 Hz 상수 목록 (이 PR)
- **목표는 이중 갱신이 아니라 접근 경로를 먼저 검증하는 것**이다(Codex P1-1).
- 4e-1b 가 옮길 구간이 다시 묶는 이름만 대상으로 한다: `rear`, `forward`, `truth_speed`, `steering_command`, `requested_speed`. 나머지(`base`·`q`·`t`·`step` …)는 그대로 둔다. 그 이름들은 머리와 Isaac 읽기 구간에서 묶이고, 그 구간은 `drive()` 에 남는다.
- **상자 `tick_vars`**: 아직 묶이지 않은 이름을 읽으면 지금처럼 `NameError` 를 낸다(`grid_kwargs` 의 초기 계획 동작 보존).
  - 쓰기: `drive()` 안의 모든 대입(루프 안 + 루프 전 `steering_command = np.zeros(2)` 초기화) 바로 뒤에 `tick_vars.sync(...)` 를 둔다. `set_rear()` 는 지역 이름과 상자를 함께 바꾼다(core 호출 도중에도 일치).
  - 읽기: `drive()` 안에서 루프 밖에 정의된 모든 클로저·λ(훅, `grid_kwargs`, `correction_grid`, `cusp_heading_motion`, `read_pocket_frame` 등)는 이 다섯 이름을 상자에서 읽는다. 루프 본문 자체는 지역 이름을 그대로 쓴다.
  - 값은 지금과 같다. 모든 쓰기 직후 상자가 지역과 같고, 읽는 쪽은 그 시각의 값을 본다.
- **AST 시험**:
  - 루프 안에서 다섯 이름에 대입하는 모든 문장 뒤에 상자 갱신이 있다.
  - `set_rear` 가 상자를 갱신한다.
  - 루프 밖에 정의된 `drive()` 의 클로저·λ 는 다섯 이름을 직접 읽지 않는다(setter 의 쓰기와 그 뒤 동기화 문장은 제외).
- **동작 시험**: core 호출 도중 `set_rear` 다음에 부른 `grid_kwargs` 가 새 자세를 본다. 초기 계획의 `NameError` 경로가 그대로다.
- **50 Hz 상수 목록**(3절)은 코드를 바꾸지 않는다.

### 4e-1b — 제어 틱 함수들 (다음 PR, 착수 전 이 절을 보강해 다시 상의)
- 단일 함수가 아니다(Codex P1-2). #442 의 `loaded_wait_` 뒤 감시, 새 장애물(Isaac), estop **시험** probe 가 제어와 `assemble_command` 사이에 그 순서로 있다. 그래서 여러 호출 경계로 나눈다: **제어 전반 → (drive: `loaded_wait_` 처리·층 감시 → 새 장애물 → estop probe) → low_clearance·명령 조립**.
- 참값·Isaac 입력은 출처를 모두 적고 콜백이나 측정 계약 뒤에 둔다:
  - 정지 호의 참 장애물 평가(10629–10671)
  - cusp·observe 종료 오프셋 주입(10303–10344, `pallet.current`·`ppos`·`initial_pallet`)
  - 반복 포획 실험(10482–10529)
  - 제어가 쓰는 참속도(10241·10590, `velocity`·`forward` → 실차는 안전 속도 측정)
  - estop 시험 probe(10801–10815)는 `drive()` 에 남는다.
  - 기존 읽기를 미리 묶거나 `plant.sample()` 하나로 합치는 것은 호출 횟수 변경이라 이 이동에서 하지 않는다.
- **반환 계약**: 바깥 루프를 끝내는 `break` 가 11 곳이다(`observe_failed` 뒤, side 거부, 포획 종료, 도착, batch 미션 완료 …). 출력 전에 끝나는 것과, `cancel_stop_` 처럼 마지막 정지 출력을 적용한 뒤 끝나는 것을 구분해 돌려준다. 밖에서 쓰는 결과(`drive`, `actual_steering`, `steer_before_`, `tracking`, `rear_stamp`, `requested_speed`, `curvature`, hold 플래그 …)를 목록으로 정해 반환한다.
- 원문 고정 시험은 옮긴 문자열을 새 위치에서 찾도록 고친다.

### 4e-1c — 공통 조립 (그다음 PR)
- `MissionCore` 생성, 장애물 층·SLAM 링크·콘솔 서버 조립을 Isaac 없이 하는 함수로 뗀다. 훅 객체는 호출자가 넘긴다. Isaac 드라이버 묶기(`IsaacPlant.command`, stand_hold·dwell 잠금)도 여기서 한다.
- 완료 조건: 재생 게이트 29/29 는 `run()` 초기 조립을 지나지 않는다. 그래서 **Isaac 실제 live smoke(단독, ws1 Isaac 순서 뒤)** 와 **Isaac 없는 가짜 포트 기동 시험**을 함께 둔다(계획 2.2).

## 3. 120 Hz 에 묶인 상수 (4e-2 에서 다시 정할 것)

원칙:
- 이미 초 단위인 정책은 120→50 비례 환산 대상이 아니다.
- 틱 수로 1 s 를 세는 것은 시간 정의로 바꾼다. 다만 `round(1/dt)` 는 명목 주파수 환산일 뿐이다. overrun 을 따라잡지 않는 실차에서는 유효·연속 표본과 실제 경과 시간을 함께 본다.
- Isaac 은 `dt = 1/120` 이므로 값이 그대로다.

| 영역 | 자리 | 지금 | 뜻 | 4e-2 |
|---|---|---|---|---|
| core 정체 창 | core 2246 `stand_ticks`, 2286 `GOAL_CREEP_WINDOW_TICKS`, 2409 `dock_stop_ticks`, 2474·2486·2588 `slam_stall_ticks`(`% 120` 포함), 2821·2854 `observe_stall_ticks`, run 10618 `blocked_ticks`, `control/cusp_stall.py:35` | 120 틱 | 약 1 s 정체 | 시간 정의 + 실제 경과 시간 |
| 틱 간격 | core 5822·6880 `options.dt`, run 6474 `dt` | 1/120 | 틱 간격 | 진입점이 넘김 |
| 근접 추적 | `near_tracking.py` 20 `TICK_S`, 22 `MAX_ROW_GAP_S`, 24 `STOP_LATENCY_S = 0.15 + TICK_S`, 165 `DriveTerms.tick_s`(측정표의 시간축), 229–232 검사, 295 명령 이력 여유, 462 문턱을 지나는 한 틱 이동, 739·759 예측 적분·관측 나이 | 1/120 | 기록 간격이자 안전 계산 | `DriveTerms` 표는 120 Hz 시간축이라 간격만 바꾸면 인덱스의 뜻이 바뀐다 — 그 간격으로 조회 / 보수 변환 / 실차 재측정 중 정한다. 검사 완화로 해결되지 않는다 |
| 중재기 | `safety/arbiter.py:63` `tick_s`, run 3400–3423·3511·3620·3698·4668, 3401–3414 catch-up 차감·`live_factor`·카메라 나이 보정 | 1/120 | 정지 판정, TELEOP 구동·승강 | 실차 틱. 시뮬 배속·catch-up 정책은 실차에 없음 |
| 정지·오도메트리 | run 6610 `StopDetector(tick_s=1/120)`(0.2 s 명령 유지·0.1 s 창·0.1 s 연속), 80·6716·8086 `LIVE_STAND_STEPS = 24`, 8077 yaw 차분 `× 120`, 11045 pocket 이력 120 행 | 틱·1/120 | 0.2 s 유지, 측정 시각 차, 이력 시간 범위 | StopDetector 가 실제로 어떤 주기 표본으로 갱신되는지 확인 뒤 정함. yaw 는 측정 시각 차, 이력은 시간 범위 |
| 층 작업 설치 | run 4820 `(layer_worker_k - 1)/120`, `layer_worker.py:133` `scan_tick + k`·barrier 대기·`install_tick`, `obstacle_layer.worker_config():121` `/120` | 틱 | 지연값과 설치 스케줄 | 지연값 환산과 '설치 스케줄·최대 0.5 s 대기 제거'는 다른 일. 요청·답·설치 시각을 구분 |
| SLAM | run 3274–3275 1 s·20 개, 6606·6648 0.25 s, 8976 배속 가산 | 초·개수 | 큐 용량, 송신 후 deadline, 취득 후 관측 나이 | 셋은 서로 다른 제한. 늦게 온 응답을 받는 `AsyncSlamLink` 정책도 실차용으로 |
| 계획 미리보기 | `control/rollout.py:63,121` `dt_s` | 1/120 | 계획 적분 간격 | 실차 틱과 무관(정밀도). 그대로 둘 수 있음 |
| 자이로 | `localization/gyro.py:104` | 1/120 | 합성 `GyroSampler` 의 첫 잡음 표본 간격 | 실차 상수 아님. 실제 `heading_step()` 의 dt 는 IMU 표본 간격(MCU STATE 간격 근거 없음) |
| Isaac 센서·영상 | run 6508 `scan_every`, 6473 `fps_divisor`, `% 12`·`% 240`, 11030 | 틱 | Isaac 센서 생성 주기 | Isaac 전용 |
| 콘솔·live | run 8449·8453·8540 `LiveRuntime(dt=1/120)`, `live_runtime.py:693` `% 12` 스캔·`% 120` 지도 발행, 727·735 영상 시각 연결 | 틱 | 센서 취득·콘솔 발행·로그 주기(섞여 있음) | 셋을 나눠 다시 정함 |
| 문구 | `failures.py:45` "requires 120Hz physics" | — | Isaac 설정 검사 | Isaac 전용 |

## 4. 시험
- 4e-1a:
  - 훅이 상자를 읽는지 확인한다(훅 이름별로 루프 이름 대신 `tick_vars` 를 쓰는지 원문으로).
  - 루프 안 대입 뒤 상자 갱신을 AST 로 고정한다.
  - 재생 게이트 29/29, 전체 pytest.
- 4e-1b·c 는 각 PR 에서 정한다.

## 5. 자체 검토
- Codex 설계 상의 1회차(새 스레드): P1 2·P2 4.
  - `set_rear` 의 루프 밖 쓰기와 누락 이름 → 4e-1a 범위를 다섯 이름으로 좁히고 setter 포함.
  - 단일 추출 경계 → 4e-1b 를 여러 경계로.
  - 참값·Isaac 입력 출처, break 반환 계약, 상수 표 보강, 4e-1c 완료 조건.
- 2회차: P2 2. 루프 전 `steering_command` 초기화 동기화, 자유 변수 시험을 '직접 읽기 금지'로. 반영 뒤 4e-1a 구현 승인.
- 총괄 결정(2026-11-11): a·b·c 세 PR 로 나눔. 4e-1a 뒤 'S1–S6 최소 bring-up 경로'를 먼저 하고 4e-1b·c 는 그 뒤. 4e-1c 의 Isaac smoke 는 ws1 Isaac 순서 ④.
