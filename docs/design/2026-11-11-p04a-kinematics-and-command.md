# P0-4 4a': 차량 운동학 주입과 명령 확정 (설계, 2026-11-11)

- 상위 계획: [`docs/plans/2026-11-11-sim-to-real-bringup.md`](../plans/2026-11-11-sim-to-real-bringup.md) 2.2, P0-4.
- 앞 단계: 4a(#436)는 Isaac 구동 호출을 `IsaacPlant` 뒤로 옮겼다. 시그니처는 임시다.
- 총괄 결정(2026-11-11):
  1. 4a' 는 MissionCore 를 고치므로 재생 게이트 runs29 가 **29/29 완전 동치**여야 머지한다.
  2. 기본 운동학은 지금의 `ackermann_command` 그대로이고, Isaac 기본 경로는 거부를 내지 않는다.
  3. 거부를 정지·재계획으로 잇는 일과 단일 조향 상태의 rate 제한은 **실차 운동학에만** 들어간다. 실차 한계는 `config/real/vehicle.yaml` 에서 읽는다.
  4. 포트 명령이 P0-7 `McuLinkHost.drive` 와 맞는지를 시험으로 보인다.
- 이 문서가 정하지 않는 것:
  - 측정 어댑터(관절·STATE → 훅)는 4b 다.
  - 센서는 4c, 참값 분리는 4d, 50 Hz 루프는 4e 다.

## 1. 왜 바꾸는가

지금은 운동학이 MissionCore 안에 박혀 있다(`core.py` 6113·6152 의 `ackermann_command`, 6155 의 앞바퀴 두 각 rate 제한, `wheel_targets`, 러너 live TELEOP 분기의 `live_ackermann`). 그리고 코어는 포트 호출 **전에** `applied_command`·근접 명령 이력을 쓴다.

실차 포트가 뒤에서 명령을 다시 포화하거나 거부하면, 코어 기록이 실제와 어긋난다. 이 어긋남은 실차에서 진단할 수 없다(총괄 결정 이유). 그래서 운동학을 주입 가능하게 하고, 실차에서 명령이 어떤 상태인지를 기록에서 구분한다.

## 2. 결정

### D1. 운동학 인터페이스 `forklift_core/plant/kinematics.py`

`VehicleKinematics` 가 맡는 연산은 아래와 같다. 각 연산의 Isaac 기본(`AckermannKinematics`)은 지금 코드를 같은 산술·같은 순서로 부른다.

| 연산 | 뜻 | Isaac 기본 | 실차(`RealKinematics`, K6) |
|---|---|---|---|
| `steering_dim` | 조향 상태 벡터 길이 | 2(앞 두 바퀴 각) | 1(기구각 q) |
| `command(speed, κ)` | 목표 구동 | `ackermann_command(speed, κ, geometry)`. 곡률을 자르고 거부는 없다 | `rear_drive_command`. 맵 밖이면 `Refusal` |
| `steering_targets(κ)` | κ 에 해당하는 조향 벡터 | `ackermann_command(0, κ).steering_rad`(지금 `wheel_targets`) | `(q(κ),)`. 맵 밖이면 `None` |
| `slew_steering(cmd, target, dt)` | 조향 명령의 rate 제한(제자리 변경) | 원소별 `np.clip(target − cmd, ±rate·dt)`, rate = `settings["steering_command_rate_rad_s"]` | q 하나를 `vehicle.yaml` 의 q 속도 × dt 로 |
| `steering_error(target, actual)` | creep 비교용 오차 | 원소별 max\|Δ\| | \|Δq\| |
| `creep_threshold_rad`, `presteer_threshold_rad` | 오차 문턱 | 지금 상수: `control/speed_chain.py` `CREEP_STEERING_ERROR_RAD` 0.05, `mission/scene_replan.py` `presteer_tolerance_rad` 0.02 | `vehicle.yaml`(q 단위, 출처 포함) |
| `held_curvature(steer)` | 측정 조향의 대표 곡률 | 지금처럼 두 바퀴 역산의 **평균** | 맵 κ(q). 무효·맵 밖이면 `None` |
| `curvature_bound(steer)` | 보호용 곡률 상한 | 지금처럼 바퀴별 \|κ\| 의 **최댓값** | \|κ(q)\|. 무효면 `None` |
| `finalize(drive, steering_cmd)` | slew 뒤 최종 구동 | 그대로 돌려줌(지금 동작) | 제한 뒤 q 의 κ 로 전자 차동 바퀴 속도를 다시 계산. 속도는 유지하고 바퀴 상한 안으로 |

- `None` 을 0 곡률로 바꾸지 않는다. 받은 쪽이 판정 불가로 처리한다.
- 정지 유지·제동 때(`dwell_hold`·`cusp_hold`·hold)는 지금처럼 **유지 중인 조향의 곡률**로 바퀴를 돌린다. 실차에서는 q → κ 로 구해서 맵 안의 값이다. 불가능 곡률을 속도 0 으로 다시 부르지 않는다(K6 는 속도 0 에서도 맵 밖 곡률을 거부한다).

### D2. MissionCore 변경

- 운동학 객체는 MissionCore 생성 때 만든다. `options.kinematics` 가 없으면 `AckermannKinematics(options.drive_geometry, settings)` 다. `args`·기록 state·`S` 서비스에는 넣지 않아서, 옛 setup 스냅숏의 재생에 새 이름이 생기지 않는다. 실차 진입점(4e-2)만 `RealKinematics` 를 넘긴다.
- 바꾸는 호출은 6113·6152 `command`, 6155 `slew_steering`, creep 의 `steering_error`·문턱, `wheel_targets`→`steering_targets`, held κ·near protection 의 κ 계산, presteer 비교다.
- **쓰기 시점은 Isaac 경로에서 그대로 둔다.**
  - `applied_command`·근접 명령 이력은 assemble 끝에서 쓴다.
  - TELEOP/preempt 는 선택 뒤 덮어쓴다.
  - `slam["applied_speed"]` 는 바퀴 액션 뒤에 쓴다.
  - `safety_state()` 의 읽는 순서도 지금 그대로다.
  - 세 쓰기를 한 시점으로 합치지 않는다(Codex 4a' P2-7: 합치면 단순 추출이 아니다). 공통 기록 함수로 추출하더라도 호출 위치는 지금 자리다.
- **차원 독립화**: `steering_command`·측정 벡터 길이를 `steering_dim` 으로 검사한다. 길이가 맞지 않으면 예외로 끝낸다(NumPy broadcast·`zip` 이 조용히 넘기지 않게). near protection 의 `[1]` 접근 같은 2차원 가정은 운동학 연산으로 바꾼다. 1차원 경로는 가짜 훅으로 시험한다. 실제 측정 연결은 4b 다.
- 러너 live TELEOP 분기(`live_ackermann` + 수동 slew + `applied_command`)도 같은 운동학 객체를 쓴다. Isaac 기본에서는 산술이 같다.

### D3. 거부의 처리(실차 운동학에서만 생긴다)

- 거부를 받은 틱은 다음과 같이 처리한다.
  - 정지 의도를 래치한다: `requested_speed = 0`, 조향은 지금 명령을 유지, 바퀴는 유지 조향의 κ 로 제동.
  - 거부 사건을 남긴다(요청 κ, 맵 범위, phase).
- 재계획은 경우를 나눈다.
  - AUTO 의 운반·복귀(지금 `scene_replan_allowed()` 가 허용하는 phase): `scene_replan_request(..., "kinematic_refusal")` 를 **한 번만** 요청한다. 이미 대기 중이거나 같은 계획에서 다시 거부되면 더 요청하지 않는다(중복·무한 반복 방지).
  - 그 밖의 phase(접근·삽입·하역): 정지를 유지하고 미션을 중단 대기로 둔다. 운용자 복구 몫이다.
  - TELEOP: 운용자에게 거부를 돌려주고 자동 재계획은 시작하지 않는다.
- 추종기·재계획·후보 미리보기가 같은 곡률 한계를 쓰도록 실차 진입점에서 맵 범위를 넘긴다(4e-2). 그렇지 않으면 같은 불가능 명령을 다시 만든다.

### D4. 포트 명령 계약 `forklift_core/plant/port.py`

- `PlantCommand`:
  - `decision_ns`, `expires_ns` — 실차는 호스트 단조 시각. Isaac 은 이 필드를 쓰지 않는다. 두 시간 영역을 섞지 않는다.
  - `control_gen` — **러너의 제어 세대**다. 콘솔 lease·미션 명령이 바뀔 때 오른다. MCU `arm_gen` 과 다르다.
  - `drive`(finalize 뒤), `steering_cmd`, `lift_m`.
  - `lift_mode`: `"target"`(lift_m 을 새 목표로) | `"keep"`(MCU 의 지금 목표 유지 = 펌웨어 `lift_hold`). `hold_lift(position)` 은 `"target"` 이다(Codex 4a' P2: `lift_hold` 플래그는 새 위치를 무시한다).
- `PortResult(kind, reason)` 는 셋으로 나눈다.
  - `queued`: 큐에 들어갔다. 물리 적용은 아니다.
  - `refused`: 운동학 거부.
  - `blocked`: 링크·허가·만료·큐·세대 이유로 내보내지 못했다.
- **출력 소유자는 하나다.** 실차 포트는 바퀴·조향·승강의 최신 의도를 들고, `zero_wheels()`·`hold_lift()` 를 같은 DRIVE 프레임으로 합성한다. 송신 큐와 프레임 송신은 지금처럼 `McuLinkHost`/`TtyLink` 가 맡는다.
- **세대 무효화**:
  - `control_gen` 이 바뀌면 포트가 `McuLinkHost` 에 새 메서드 `drop_drives(reason)` 를 불러 대기 중인 DRIVE 를 버린다.
  - 옛 세대의 명령 제출은 `blocked("stale_generation")` 이다.
  - 재무장(MCU 허가 새로)은 최신 의도 캐시를 비운다.
  - 재전송·부분 합성은 원래 `expires_ns` 를 넘지 않는다.
- **blocked 를 0 으로 기록하지 않는다**(Codex 4a' P1-1).
  - 이전 DRIVE 가 큐나 MCU 에서 아직 유효할 수 있다.
  - 실차 포트는 `outstanding_until_ns`(큐·송신된 DRIVE 가운데 가장 늦은 원래 만료)를 노출한다.
  - 코어는 blocked 를 받으면 정지 의도를 래치하고 미션 진행·재계획을 멈춘다. `outstanding_until_ns` 가 지나거나 측정(4b)이 정지를 보일 때까지는 이전 명령이 살아 있을 수 있다고 보수적으로 다룬다.
- **실차 기록은 셋을 구분한다**: 계산·중재 뒤 목표(`applied_command`, 지금 뜻 그대로), 제출 결과와 만료·세대(`submitted`), 측정(4b). Isaac 기록에는 새 키를 더하지 않는다(게이트 동치).
- `IsaacPlant` 는 이 계약의 `command(cmd)` 를 받아 지금의 세 호출(바퀴 + Isaac 전용 잠금 q, 조향, 승강)을 내고 `queued` 를 돌려준다. 다만 4a' 에서 drive() 의 틱 명령을 `command()` 로 묶으면 `slam["applied_speed"]` 쓰기가 바퀴와 조향 액션 사이에 있어 순서가 바뀐다. 그래서 4a' 의 drive() 는 지금 호출을 그대로 두고, `IsaacPlant.command` 는 계약 시험용으로만 둔다(드라이버 묶기는 4e-1).

### D5. 합격 시험

1. 재생 게이트 runs29 29/29(SETUP_INPUTS=recorded). Isaac 기본 경로에서 거부·새 기록 키 없음.
2. `AckermannKinematics` 의 각 연산이 지금 코드와 바이트 단위로 같은 값을 낸다(무작위 κ·속도·조향 표본).
3. `RealKinematics`:
   - 맵 경계 안팎.
   - slew 뒤 finalize 의 바퀴·q·곡률 일치.
   - 무효 q 는 `None`.
   - held 평균과 보호 최댓값의 구분(Isaac `(+0.08, −0.08)` → 평균 0, 상한 ≈ 0.1177 1/m).
4. 코어 1차원 경로: 가짜 훅과 `RealKinematics` 로 `assemble_command` 를 돌려 shape 불일치가 예외로 끝나는지, 거부 래치·한 번만 요청·지원하지 않는 phase 정지 유지를 본다.
5. 포트 + `McuLinkHost`(가짜 STATE):
   - DRIVE 프레임의 바퀴·q·valid·`lift_hold` 가 기대값인지.
   - 비영 DRIVE 로 큐를 채운 뒤 zero 제출이 blocked 일 때 `outstanding_until_ns` 가 남는지.
   - 세대 변경 → 대기 DRIVE 폐기.
   - 재무장 뒤 옛 명령 재제출은 blocked.
   - `lift_mode` 두 값.

## 3. 위험

- **게이트 동치**: 운동학 래퍼가 산술 순서를 바꾸면(예: 배열 복사, dtype) 기록된 값이 달라진다. D5-2 의 바이트 비교로 먼저 막는다.
- **차원 독립화의 범위**: 코어 전반의 2차원 가정을 빠짐없이 찾는 것이 어렵다. AST 검색으로 `steering_command[`·`steering_position()[`·`[1]` 패턴을 목록으로 만들어 PR 에 붙인다.
- **문턱의 단위**: q 단위 문턱(creep·presteer)과 q 속도는 실측 전이라 `vehicle.yaml` 에 '실측 대기' 꼬리표를 단다.

## 4. 자체 검토

**① 원자료·코드 대조(Claude)**: `(+0.08, −0.08)` 의 평균 0·상한 0.11768 1/m 를 `wheel_odometry` 식으로 다시 계산해 맞음. creep 0.05·presteer 0.02 의 정의 위치를 확인해 표에 적음. `scene_replan_allowed`(core.py 7966)·`scene_replan_request`(8098) 위치 확인.

(② Codex 적대 검토, ③ 앞의 둘을 보지 않은 agy 의 '적히지 않은 전제' 는 결과를 받은 뒤 채운다.)
