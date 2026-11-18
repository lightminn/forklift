# 실차 조립 · AUTO 허가 상태 — 4b (설계)

작성 2026-11-19 · 1판 · 상위 [실차 자율 계획](../plans/2026-11-18-real-autonomy-entry.md) PR 4 · [제어 틱 계약](2026-11-18-real-control-tick.md) · 앞 단계 [4a 알려진 팔레트 시작](2026-11-19-known-pallet-start.md)(#598)

## 범위

PR 4 를 셋으로 쌓는다(계획 표 4 행의 '커밋 단위' 순서). 4a 는 코어 쪽(#598)이다. **4b** 는 실차 조립과 AUTO 허가 상태이며, 50 Hz 루프는 아직 없다. 4c 는 50 Hz 루프·포트·STOP 계약과 그 시험이다.

4b 는 Isaac 을 건드리지 않는다. 새 모듈 둘과 중재기의 실차 전용 설정 하나(기본은 지금과 같음)만 더한다. 재생 게이트를 돌리고, 전체 pytest 도 돌린다.

## 1. 자리

- `src/forklift_core/live/real_autonomy.py`(새 모듈): 조립 함수, 옵션 검사, 준비 판정, AUTO 허가 상태. 순수 Python 이고 가짜 포트로 시험한다(텔레옵의 `real_teleop.py` 와 같은 자리).
- `tools/real_autonomy.py`(새 진입점): 4b 에서는 인자 파싱과 조립까지 하고, `--dry-assemble` 로 조립 결과(옵션 감사·준비 판정)를 출력한 뒤 끝낸다. 장치 열기와 루프는 4c 에서 한다. 콘솔 서버(`console/`)는 wheel 밖이라 이 진입점이 붙인다.
- MCU 는 실행기 하나만 쥔다. 텔레옵 다리와 같은 장치를 동시에 열지 않는다(장치 잠금 파일은 4c 에서).

## 2. 옵션 — 실차에서 명시, 빠지면 거부

- `real_options(...)` 는 S10-A 에 필요한 값만 명시한 `SimpleNamespace` 를 만든다. 나머지 기능 옵션은 모두 꺼짐으로 명시한다(코어의 `getattr(..., 기본값)` 에 기대지 않음).
  - 필수: `dt = 1/50`, `real_vehicle = True`, `kinematics`(`RealKinematics`), `grid_planning = True`, `obstacle_act = True`, `pocket_check = True`, `s10_stop_at_lift = True`.
  - 꺼짐: `near_tracking = None`, `bottom_deck_check = "off"`, 장면 재계획·탈출 선조향·D7 재접근 등.
  - 계획·추종 설정은 `config/real/` 의 실차 파일에서 읽는다. 속도는 낮은 값(계획의 저속 원칙)을 쓴다.
- `real_option_refusal(options) -> list[str]` 가 조립 직전에 검사한다.
  - 필수 값이 빠졌거나 다르면 거부한다(`dt` 가 1/50 이 아님, `real_vehicle` 이 아님, `obstacle_act` 꺼짐 등).
  - 근접 추적이나 밑판 검사가 켜져 있으면 거부한다.
  - 곡률: 추종·계획·미리보기 곡률 상한이 모두 같아야 하고, 그 값이 `RealKinematics` 의 비대칭 범위에서 `min(−κmin, κmax)` 이하여야 한다.
  - 하나라도 걸리면 조립하지 않는다.
- 설정: `lift_target_m` 은 운반 높이이고 S10 은 거기까지만 든다. `steering_command_rate_rad_s` 등은 실차 설정 파일의 값을 쓴다.

## 3. 훅 — `fail_closed_hooks(RealHooks(...))`

- `RealHooks` 는 S10-A 경로가 읽는 CONTROL·RECORD·CLOCK 훅만 구현한다. 나머지 이름은 정의하지 않으므로, `fail_closed_hooks` 가 TruthAccessViolation 을 내거나(필수) 없는 것으로 다룬다(선택).
- 4b 시험 하나가 실제 접근 경로를 가짜 원천으로 돌려서, 읽힌 훅 이름이 모두 구현됐는지 확인한다.
- 원천은 주입한다(가짜 또는 실물):
  - 위치·보정: SLAM 원천(`slam_rear`, `slam_odom_base`).
  - 측정: 포트 표본(`lift_position`, `steering_position`·`steering_actual`, `safety_speed` = 엔코더 SafetySpeed).
  - 계획: 기존 계획 함수를 그대로 쓰고, 작업자는 `PlanWorker`.
  - 포켓 프레임: `known_pocket_frame` 은 PR 7 전까지 없음(None) → 4a 의 대기 규칙대로 미션이 끝난다.
  - 운용자 확인: 콘솔(`operator_check`).
- `grid_kwargs` 는 준비 때 부작용이 없다(Codex 4a P3). `obstacle["pickup_estimate"]` 를 바꾸지 않는다. 그 값은 채택 때 코어가 넣는다.
- 시계: 코어 시각은 실차 단조 시계에서 세션 시작을 뺀 값이다. `world`·`time_module` 은 없다(필수 CLOCK 이면 실차 구현).

## 4. 부팅 최소 입력 (값과 뜻을 시험; Isaac 과 dict 키 전체 비교는 하지 않음)

- `state = {"transitions": [], "planning_wall_s": 0.0, ...}`, `pocket = {"check": None, "frames_read": 0}`.
- `obstacle`:
  - `layer`: 실차 층 설정 — 7b 의 `obstacle_layer_real.yaml`, 알려진 팔레트 꺼짐.
  - `applied`: 지금 보정.
  - `version: 0`, `ticks`·`min_allowed`·`slowed`·`reasons` 는 빈 dict.
- `slam`: `tracker`(실차 SLAM 링크 추적기), `records: []`, `docking: {}`, `stop_now`, `holds: []`, `control: []`.
- `MissionBaseline.capture(core)` 는 위 구성을 마친 뒤 한 번 한다(Isaac 의 `:9928` 자리).

## 5. 서버 기동 ≠ AUTO 준비

- 서버(콘솔)는 조립 직후 뜬다. 상태 보기, 텔레옵 금지 표시, 운용자 확인 입력을 받는다.
- `auto_ready_refusal(now) -> str | None` 이 준비 판정이다. 준비가 안 되면 미션 제출은 그 사유로 거부되고(`prepare` 전) 구동 출력도 없다.
- 조건(앞에서부터 처음 걸린 것을 사유로):
  1. 필수 구성: 작업자 살아 있음, 층 있음, 기준선 캡처됨.
  2. 링크 신선: MCU STATE 가 `state_max_age_s` 안.
  3. SLAM `tracking` 이고 응답이 나이 상한 안. 재위치 세대 잠금이 없어야 한다(`real_auto_refusal`).
  4. 장애물 증거: 설치된 층 스냅숏이 나이 상한 안이고 지금 보정 세대와 같다(비동기 갱신 자체는 PR 5 이고, 4b 는 판정만).
  5. 측정 유효: 조향·승강·바퀴 계수 연속.
  6. 현장 복구 없음.

## 6. AUTO 허가 상태 (하나만 두고 최종 DRIVE 와 감시가 모두 본다)

- 상태: `OFF → REQUESTED → ARMING → READY → DRIVING`. `LATCHED` 는 운용자 재무장이 필요하다.
- 무장 순서(송신자 하나):
  1. 운용자 AUTO 요청 → `REQUESTED`.
  2. 미션 승인(채택)이 나면 MCU ARM 을 요청한다(`host.request_arm`, 호스트의 확인용 0 DRIVE 포함) → `ARMING`.
  3. 호스트 `ready` → `READY`.
  4. 첫 미션 DRIVE 가 큐에 들어가면 → `DRIVING`.
- 준비 판정(5절)이 계속 서야 한다.
- `LATCHED` 로 가는 사유는 다음과 같다. DISARM 은 응답을 기다리지 않고 즉시 한다.
  - STOP, 운용자 DISARM, lease 상실(실차 정책: 정지 — 총괄 결정 2026-11-19), 링크 상실.
  - 측정 무효, 포트 거부(`refused`/`blocked`), 감시 trip(과속·주기 초과·SafetyWatch).
  - 준비 판정 실패, `real_auto_refusal` 사유.
- `LATCHED` 에서 다시 나가는 길은 운용자 재무장 하나다. 이전 세대의 비영 DRIVE 가 더 작동할 수 없고(`older_nonzero_may_act` 거짓) 신선한 측정으로 정지가 확인된 뒤에만 받는다. 입력이 돌아오는 것만으로는 재개하지 않는다.
- 감시는 이 상태를 본다. 지금 감시는 중재기 `armed` 를 보는데, 중재기는 AUTO 진입 때 `armed` 를 내린다. 그래서 실차 AUTO 의 감시 입력은 `armed = state in (ARMING, READY, DRIVING)` 이다.
- 중재기: `ArbiterConfig.auto_lease_loss = "continue" | "stop"` 를 더한다. 기본 `continue` 는 지금 Isaac 그대로이고, 실차는 `stop` 이다. `stop` 이면 `_lose_lease` 가 AUTO 에서도 `preempt_latch` 를 세우고 PAUSED 로 간다. 이 설정 하나가 Isaac 과 공유하는 코드의 변경이고, 기본값은 동작을 바꾸지 않는다(게이트).
- 4b 는 상태 전이와 무장 순서를 가짜 호스트(`tests/unit/plant/test_port.py` 의 Bench 모양)로 시험한다. 4c 가 루프에서 최종 DRIVE 를 이 상태에 묶는다.

## 7. 기록

조립 기록 `assembly.json` 에 다음을 남긴다. 4c 에서는 매 틱 최종 목표·제출 결과·만료·세대·측정 STATE 를 구분해 남긴다.
- 옵션 감사(필수·꺼짐 목록과 값), 훅 구현 목록.
- 설정 파일과 해시, 곡률 상한.
- 준비 판정의 처음 실패 사유와 시각.

## 8. 시험 (가짜 원천, Isaac 없음)

- 옵션: 필수 값 각각 빠짐·틀림 → 거부. 근접 추적·밑판 검사 켬 → 거부. 곡률 불일치·비대칭 상한 초과 → 거부.
- 훅: S10-A 경로(알려진 시작 준비 → 채택 → 접근 몇 틱 → 첫 프레임 대기)를 가짜 원천으로 돌려 TruthAccessViolation 이 없음을 본다. 참값 훅을 부르면 TruthAccessViolation.
- 부팅 입력의 값·뜻, 기준선 캡처 뒤 첫 채택에서 미션 키가 깨끗함.
- 준비 판정: 각 조건을 하나씩 깨면 그 사유로 제출이 거부되고 출력이 없음. 모두 서면 받는다.
- AUTO 허가: 무장 순서(채택 전 ARM 없음, ready 전 DRIVE 없음), 각 래치 사유 → 즉시 DISARM·`LATCHED`, 재무장 조건(옛 비영 DRIVE 가능·정지 미확인이면 거부), 입력 복구만으로 재개 없음.
- 중재기 `auto_lease_loss`: `stop` 이면 AUTO 에서 lease 를 잃었을 때 래치·PAUSED, 기본은 지금과 같음.
- `tools/real_autonomy.py --dry-assemble`: 조립 기록 출력, 거부 사유가 있으면 0 이 아닌 종료.

## 9. 4c 로 넘기는 것

- 50 Hz 벽시계 루프(주기 초과 두 조건, 단계별 기록), STATE 를 새 표본마다 한 번 `RealOdometry` 에 넣기.
- 표본 시각 기준 `StopDetector`, 최종 제출 직전 STOP 재확인, 제어 정지(래치 → 옛 DRIVE 폐기 → 측정 위치 정지 DRIVE → 100 ms 확인).
- `take_s10_stop()` 소비, 종료 뒤 타이머 건너뛰기(Codex 4a P3), `PortResult.seq`.
- 가짜 포트 완주·S10 제한 운용 시험.

## 10. 자체 검토

(검토 뒤 채움)
