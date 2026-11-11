# S6 콘솔 텔레옵 다리 — 콘솔 → 중재기 → 엔코더 속도 상한 → `McuLinkHost` (설계 2판, 2026-11-12)

bring-up 묶음의 PR ⑥ 이다(총괄 순서: #452 계약 → #453 도구 ① → #455 도구 ② → 이 PR → ⑦ A2M12 스캔). 계획 `docs/plans/2026-11-11-sim-to-real-bringup.md` S6(콘솔 텔레옵: 직진·좌우·후진, 정지 응답, 콘솔 lease·STOP·재무장)의 소프트웨어 경로를 **Isaac LiveRuntime 밖에서** 닫는다. 2판은 Codex 설계 상의(P1 3·P2 7·P3 1)를 반영했다(11절).

**4e-2 와의 경계**(Codex 권고): **지금 구동을 허가하는 데 필요한 안전 계약**은 이 PR 에서 닫는다 — 무장 상태기계, 출력 마감, 거부 시 정지, 엔코더 속도의 유효성·방향·상한, 실차 적재·복구 차단. 미션 코어·SLAM·공통 조립·추가 실차 감시(IMU·전류 등)는 4e-2 에 남는다. 4e-2 는 이 다리의 부품(무장 대응·엔코더 속도·출력 게이트)을 그대로 쓴다.

## 1. 재사용하는 것과 새로 만드는 것

재사용(바꾸지 않음): `console/live/server.py`(`LiveServer`), `live/link.py`(`Channels`·`Link`), `safety/state.py`, `safety/arbiter.py`, `safety/teleop_policy.py`, `safety/teleop_drive.py`, `plant/kinematics.py`(`RealKinematics`), `plant/port.py`(`RealPlantPort`), `plant/mcu_host.py`(`McuLinkHost`, `Client.REAL`).

새로:
- `src/forklift_core/live/real_teleop.py`(입출력 없음): 엔코더 속도(2절), 상한(3절), 무장 상태기계(4절), 한 주기의 판단(5절), 출력 게이트(6절), 안전 상태(7절), 허가 감싸기(8절).
- `tools/real_mcu_link.py`: `ReconnectingLink` 에 수신만(`receive`)·송신만(`transmit`) 나눈 두 단계(5절, 기존 `service` 는 그대로).
- `tools/teleop_bridge.py`: 입출력 어댑터(장치·콘솔 서버·50 Hz 벽시계 루프·시작 확인).

## 2. 엔코더 속도 (`SafetySpeed`, 참값 없음)

`STATE`(신선할 때만)의 바퀴 각속도 `ω_L, ω_R`(MCU 가 엔코더 카운트로 계산)와 `vehicle.yaml` 의 바퀴 반지름 `r_L, r_R`(지금 차체 모델 공칭값, S8 실효 반경 실측 대기 — 그래서 상한은 '공칭 반지름 환산'이다):
- `v_L = r_L·ω_L`, `v_R = r_R·ω_R`, `signed = (v_L + v_R)/2`, `yaw_rate = (v_R − v_L)/track`.
- **경계** `bound = max(|v_L|, |v_R|) + a_bound·(age + window) + q`:
  - 바깥 바퀴 속도로 잡아 회전 중에도 차체 어느 바퀴도 넘지 않게 한다(평균만 보면 반대로 도는 두 바퀴가 0 으로 보인다, Codex P2-8).
  - `age` 는 `STATE` 의 호스트 수신 나이(≤ 50 ms, 그보다 늙으면 무효), `window` 는 MCU 속도 창(빌드 상수, 설정값), `a_bound` 는 가속 상한(텔레옵 가속 `accel_mps2` 0.8 와 같은 잠정값 — 측정 뒤 S5 에서 정함), `q` 는 한 카운트의 속도 양자화(`2π·r / (CPR·window)`).
  - 이 식과 값의 출처는 `vehicle.yaml` `teleop_speed_limit` 아래에 provenance 와 함께 둔다(설정 파서 확장, Codex P3).
- **가능 방향** `directions`: `signed` 가 `+bound` 쪽으로 0 을 넘을 수 있으면 `+1`, `−bound` 쪽이면 `−1`(경계가 0 을 걸치면 둘 다). 정지 판정 문턱을 쓰지 않는다.
- **무효**(`valid=False`): 신선하지 않음, 각속도가 유한하지 않음, 카운트 연속성·stuck·polarity 비트, 부팅이 바뀐 뒤 첫 표본, 조향 측정이 지도 밖(`PlantSample.valid["steering"]`). 무효면 `Feedback.rear_axle_speed_mps = None` 이라 중재기가 `no_feedback` 걸쇠로 무장을 풀고(4절) 정지 확인 전엔 다시 못 연다.

## 3. 상한 0.2 m/s — 명령 상한·안전 상한·고장 문턱 (Codex P2-9)

- **안전 상한** `V = teleop_speed_limit_mps`(0.2, 총괄 결정): 엔코더로 잰 속도의 **경계**가 이것을 넘으면 안 된다. 허용 오차를 덧붙이지 않는다.
- **고장 문턱**: 무장한 텔레옵 중 `bound > V` 이면 그 주기에 0 을 보내고 중재기를 `STOP`(`overspeed`). 재개는 콘솔 재무장(정지 확인 포함).
- **명령 상한**: 정상 주행이 고장 문턱에 닿지 않게 `V − a_bound·(age_max + window) − q`(잠정값으로 약 0.15 m/s)로 `TeleopLimits` 의 직진·후진·영상·적재 상한을 깎는다(`real_teleop_limits`). 이 값은 계산해 기록하고 시험이 고정한다.
- 참값 경로(`truth_access`)를 부르지 않음을 시험이 고정한다.

## 4. 무장 상태기계 (중재기 `arm_gen` ↔ MCU 무장, Codex P2-5·P1-3)

중재기의 `armed` 는 콘솔 arm 일 뿐이다. 다리가 MCU 무장과 짝짓는다. 상태: `idle`(MCU 무장 없음), `arming(g)`(중재기 `arm_gen = g` 를 위한 MCU 무장 진행), `ready(g)`, `disarming`(DISARM 확인 대기), `held`(거부·철회 뒤 재무장 전).
- **무장**: 중재기가 TELEOP·`armed` 이고 `arm_gen = g` 인데 MCU 가 `g` 에 묶여 있지 않으면:
  - MCU 가 다른 세대로 무장돼 있거나 무장 중이면 먼저 `DISARM` 하고 `DISARMED` 를 확인한 뒤(같은 주기의 disarm → arm 도 이렇게 걸린다),
  - 옛 비영 DRIVE 가 아직 작용할 수 있으면(`port.older_nonzero_may_act`) 기다린 뒤,
  - `host.request_arm` 을 **한 번** 부른다. 거부·시간 초과면 중재기를 `STOP`(`mcu_arm_refused:<이유>`).
  - 자동 `ARM`·`CLEAR_FAULTS` 는 없다. 중재기의 새 arm(운용자 요청)마다 한 번이다.
- **준비 전**: `ready(g)` 가 아니면 비영 출력은 없다. 중재기에는 적용 속도 0 을 알린다.
- **해제**: 중재기가 `armed` 를 잃거나 `arm_gen` 이 바뀌거나 `control_gen` 이 바뀌면 그 주기의 송신 전에 대기 DRIVE 를 버리고(`port.new_control_generation`) `DISARM` 을 넣는다. 단 MCU 가 스스로 `STOPPING`(만료)이면 그 제어 감속을 `DISARM` 으로 끊지 않고 `DISARMED` 를 기다린다(최대 `stop_grace` + 여유). 운용자 STOP·lease 상실·피드백 상실은 바로 `DISARM` 한다(계약 3.4: `DISARM` 은 즉시 채널별 정지 상태).
- **MCU 쪽 철회**: `ready(g)` 에서 호스트 권한이 끊기면(MCU `STOPPING`·`FAULT`·`ESTOP`·부팅 변경·링크 끊김·빌드 거부) 중재기를 `STOP`(`mcu_<이유>`). 다시 움직이려면 콘솔 재무장(중재기의 정지 확인 포함) → 새 `arm_gen` → 새 MCU 무장. 재연결만으로 다시 무장하지 않는다.
- **빌드**: `Client.REAL` 이라 `outputs_locked`·`estop_unwired`·`commission_build`·`hil_plant` 보드에는 무장하지 않는다(계약 3.3). S6 선행 조건(bit6·bit7 해제)이 코드로 걸린다.

## 5. 한 주기 (50 Hz, 벽시계, 따라잡기 없음) — 순서가 안전 조건이다 (Codex P1-2)

1. **수신만**: `link.receive(host, now)` — 읽기 → `on_bytes` → `host.tick`. 송신하지 않는다.
2. **판단**: 엔코더 속도(2절) → `Feedback` → `link.gate` → `link.begin`(STOP·요청·텔레옵) → 안전 상태(7절) → `link.end(..., drive)` → 상한 감시(3절) → 무장 상태기계(4절) → 출력 게이트(6절).
3. **송신만**: `link.transmit(host, now)` — 2 에서 버린 것은 나가지 않는다. 이미 쓰기 시작한 프레임은 끝까지 쓰고 `older_nonzero_may_act` 로 추적한다(부분 송신은 취소가 아니다, 4a' D4).
- 주기 초과는 세고 기록한다. 늦은 주기를 몰아서 돌리지 않는다. `ArbiterConfig(tick_s=0.02)`, 나머지는 기본값(Isaac 배속 보정 없음).

## 6. 출력 게이트

- **만료**(Codex P2-4): 비영 출력의 절대 만료 = `min(결정 시각 + 100 ms, 중재기 텔레옵 마감, lease 마감)`. 콘솔 명령이 갱신되지 않으면 MCU 쪽 마감도 그 원래 마감을 넘지 않는다. 0 출력도 같은 식이다.
- **조향**(Codex P2-7): 조향 목표가 텔레옵 곡률을 따르는 것은 `ready(g)`·정책 `drive_allowed`·데드맨 눌림·latch 없음일 때뿐이다. 그 밖(정책 거부·데드맨 놓음·준비 전·preempt)에서는 **유지 조향** `q_hold` 를 보낸다. `q_hold` 는 새 MCU 무장마다 신선한 측정 `q` 로 다시 잡고, 유지 구간에 들어갈 때 그 순간의 측정 `q` 로 다시 잡는다. 곡률 → `q` 변환이 실패하거나(`DriveRefusal`) 측정 `q` 가 지도 밖이면 비영을 내지 않고 `STOP`(`steering_invalid`).
- **승강**(Codex P2-10): 늘 `lift_hold`(MCU 가 무장 때 잡은 목표 유지)다. 승강 텔레옵은 열지 않는다. 일반 펌웨어의 `ARM` 은 조향·승강 센서·전류 측정이 모두 유효해야 하므로, **승강 센서가 유효한 것**이 S6 선행 조건이다(계획 S6 에 적음). 다리는 센서 유효성을 꾸미지 않고 시운전 빌드로 우회하지 않는다.
- **거부**(Codex P1-3): `port.command` 가 `blocked`·`refused` 를 내면(큐 포화·세대·만료·`DriveRefusal`) 정지 의도를 건다 — 대기 DRIVE 를 버리고 `DISARM`, 중재기 `STOP`(`port_<이유>`). 재개는 4절 무장 조건(옛 비영 명령의 영향 가능성 소멸 + 중재기의 정지 확인 + 새 arm)으로만.
- 기록은 셋을 나눈다: 중재기가 계산한 목표(`Output`), 포트에 낸 결과(`PortResult`), 측정(`SafetySpeed`).

## 7. 안전 상태 (미션 코어 없이, Codex P2-6)

이 다리에는 미션 코어가 없다(`Arbiter(mission=None)` — AUTO·미션 요청은 `unsupported`). 정책이 읽는 안전 상태를 다리가 만든다.
- 기본은 **알 수 없음**: `load`·`fork_in_pocket` 이 `None` → 정책이 `state_unknown` 으로 거부.
- 독립 S6 운용의 시작 확인은 네 가지를 **따로** 받는다(`--yes` 로 대신 못 함): `--empty-forks`(무적재), `--forks-clear`(포켓 밖), `--lift-low`(승강이 바닥~운반 높이 사이, 계획 5절), `--no-open-recovery`(미해결 현장 복구 없음). 넷이 다 있어야 빈 포크·포켓 밖·SLAM 없음(`frame.slam = False`)·미션 없음 상태를 쓴다. 하나라도 빠지면 알 수 없음이다.
- **다리 자체의 실차 규칙**: 안전 상태 제공자가 무엇을 주든(4e-2 에서 미션 코어가 줄 때도) `load.loaded` 가 참이면 `teleop_refused = "real_loaded"` 를 덮어쓴다(실차 적재 텔레옵 금지, 감사 4-3·4-4). `recovery_required`·현장 복구(`fork_in_pocket` 참)는 정책이 거부한다.

## 8. 허가 — 스캔 없으면 0, 엔코더 방향까지 검사 (Codex P1-1)

- 다리의 허가 `permit(curvatures, direction, check_speed) -> (limit, reason)` 는 바닥 허가(장애물 층)를 감싼다:
  - 요청 방향을 `check = max(check_speed, bound)` 로 검사한다.
  - 2절 `directions` 가운데 요청과 다른 방향(잔류·반대 운동)도 `bound` 로 검사하고, 하나라도 막히면 `limit = 0`(`unasked_<dir>:<이유>`). 4d-1 의 실차 규칙(측정 방향을 경계 속도로, 각자 검사, 실패는 정지)과 같다.
  - 엔코더 속도가 무효면 `limit = 0`.
- **CLI 의 바닥 허가는 '스캔 소스 없음 → 0 (`no_scan`)'** 다. 그래서 ⑦ 전에는 실차 바퀴가 움직이지 않는다(계획 S6 선행 조건: 실제 스캔·LiDAR 장착 보정). 무장·정지·재무장·0 DRIVE·lease 는 지금도 닫힌다.
- 시험은 합성 격자 위의 진짜 `DrivePermission` + `arc_permit` 바닥 허가를 주입한다. 주입 경로는 CLI 에서 닿지 않는다. ⑦ 이 스캔 소스 → 장애물 층 → 바닥 허가를 채운다.

## 9. 시험

- 단위(`tests/unit/live/test_real_teleop.py`):
  - 엔코더 속도: 부호·평균·yaw·바깥 바퀴 경계·나이·양자화, 무효 조건 각각, 경계가 0 을 걸칠 때 두 방향.
  - 상한: 명령 상한 계산, 경계 > 0.2 → 0 + STOP, 0.21 m/s 측정이 통과하지 않음.
  - 허가 감싸기: Codex 반례(후진 −0.005·후방 허가 0·전진 요청 → 0), 경계 검사 속도, 무효 → 0.
  - 무장: 운용자 arm → `request_arm` 한 번, 지연된 ARMED, 같은 주기 disarm → arm(DISARM 확인 뒤 다시 ARM), 무장 대기 중 STOP·부팅 변경, MCU 철회 → 중재기 STOP, 재연결 뒤 자동 재무장 없음, 옛 비영 명령이 작용할 수 있으면 재무장 대기, 빌드 거부.
  - 출력 게이트: 만료가 텔레옵·lease 마감을 넘지 않음(재송신 포함), 유지 조향(데드맨 놓음·정책 거부·준비 전), 큐 포화로 0 도 거부 → DISARM + STOP, 무효 조향 → STOP.
  - 주기 순서: STOP 이 채널에 있는 주기에 옛 비영 DRIVE 가 나가지 않음, 마감이 지난 주기에도.
  - 안전 상태: 기본 알 수 없음, 확인 넷 중 하나라도 빠지면 거부, `real_loaded` 덮어쓰기, `recovery_required`·현장 복구 → 비영 DRIVE 없음.
  - **양성**: SLAM 무효 + 나머지 유효 → 텔레옵이 비영 DRIVE 를 낸다(따로 둔다). AUTO 요청은 `unsupported`.
- 종단(`tests/integration/test_teleop_bridge.py`, MCU 모형 pty + `Channels` 로 콘솔 메시지): lease → TELEOP → arm → MCU ARMED·ready → 비영 → 모형 바퀴가 돌고 경계 ≤ 0.2; STOP → MCU `DISARMED`; lease 만료 → 정지; 송신만 끊김 → MCU 만료 `STOPPING` → 다리는 `DISARM` 으로 끊지 않고 기다림 → 중재기 STOP; 수신만 끊김 → 호스트 stale → 중재기 STOP; 출력 잠금 빌드 → 무장 거부; 바닥 허가 없음(CLI 기본) → 무장돼도 비영 없음; 루프가 마감 직전에 멈춤 → MCU 가 원래 마감 안에 섬.
- 실차 S6 는 같은 다리로 하되 ⑦ 뒤에 한다.

## 10. 범위 밖

실제 스캔·장애물 층(⑦), SLAM·미션 코어·공통 조립(4e-2), 카메라 영상(`video` 모드는 영상 신선도가 없어 중재기가 거부 — `direct` 만), 승강 텔레옵, 상한·경계 값의 확정(S5), IMU·전류 감시(4e-2).

## 11. 자체 검토

Codex 설계 상의 1회(P1 3·P2 7·P3 1). 모두 반영했다.

| 지적 | 처리 |
|---|---|
| P1-1 `SafetySpeed` 상한·가능 방향이 허가에서 사라짐(후진 −0.005·후방 0·전진 요청 → +0.016 반례) | 허가 감싸기: 요청 방향은 경계 속도로, 다른 가능 방향도 검사, 무효면 0(8절) |
| P1-2 주기 첫 `service()` 가 STOP·만료 전에 옛 DRIVE 송신 | 수신만 → 판단 → 송신만(5절) |
| P1-3 포트 거부를 0 으로 바꾸는 것으로 정지가 안 됨 | 정지 의도 + DISARM + 중재기 STOP, 재개는 무장 조건으로만(6절) |
| P2-4 만료가 콘솔 마감을 연장 | `min(결정+100 ms, 텔레옵 마감, lease 마감)`(6절) |
| P2-5 `armed` 불리언 비교로 무장 전이 누락 | `arm_gen` 상태기계, 같은 주기 disarm → arm, 피드백 상실 경로(4절) |
| P2-6 빈 포크 확인 하나로 세 상태 확정 | 확인 넷 분리, 다리 자체의 `real_loaded` 덮어쓰기(7절) |
| P2-7 속도 0 ≠ 조향 정지 | 유지 조향 `q_hold`, 새 무장마다 측정으로 초기화(6절) |
| P2-8 yaw rate·조향 유효성 계약 없음 | 바퀴 차로 yaw, 바깥 바퀴 경계, 조향 무효 → 무효·STOP(2·6절) |
| P2-9 상한 구분 없음(0.21 통과) | 안전 상한 0.2 = 경계 문턱, 명령 상한은 그 아래로 계산(3절) |
| P2-10 승강 센서 의존 | 승강 센서 유효가 S6 선행 조건, 꾸미지 않음(6절) |
| P3 설정 파서 확장, 양성 시험 분리 | 2절, 9절 |
