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

- `None` 을 0 곡률로 바꾸지 않는다. 받은 쪽이 판정 불가로 처리한다. 정지에는 곡률이 필요 없으므로, 유지 조향의 곡률이 `None` 이면(무효·맵 밖 q) 두 바퀴를 **곡률 없이 0** 으로 두고 조향 명령은 유지한다. 이렇게 해서 제동 명령을 만드는 경로가 예외로 끊기지 않는다(agy ③).
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
- **TELEOP 속도 체인의 문턱**: `safety/teleop_drive.py` 의 `TeleopDrive.step()` 이 공통 `creep()`(고정 0.05 rad)를 부르고, `reverse_straight` 는 `recovery_steer_tol_rad`(0.01, 앞바퀴 각 단위)를 쓴다. 운동학의 `creep_threshold_rad` 와 q 단위 직진 문턱을 `TeleopDrive` 생성 때 주입하는 경로를 둔다. Isaac 기본은 지금 값 그대로이고, 실차는 `vehicle.yaml` 에서 읽는다(Codex 4a' 문서 P2-4). AUTO 와 TELEOP 이 같은 실차 문턱을 쓰는지 시험한다.

### D3. 거부의 처리(실차 운동학에서만 생긴다)

거부 상태기를 코어에 둔다. Isaac 기본 운동학은 거부를 내지 않으므로 이 상태기는 Isaac 경로에서 들어가지 않는다.

**4a' 에서 만든 범위(구현 리뷰 뒤, Codex 4a' 구현 P2-4)**:
- 1(거부 래치)과 6·7·8 의 '정지 유지 + 운용자' 쪽만 만든다. 거부가 생기면 정지를 래치하고 미션을 **일시정지**한다(`commands.queue(Pause("kinematic_refusal"))`). 운용자가 재개(다시 시도)하거나 취소한다.
- 래치는 새 계획(phase·추종기 serial 변경), 운용자 재개(`resuming`), 채택 미션 없음에서 풀린다.
- 2–5(정지 확인 → 자동 재계획 → presteer → 검증 → 재개)는 **뒤로 미룬다**. 자동 재계획은 scene replan 의 정지 확인을 포트 해제 조건 (a)·(b) 와 묶어야 해서 그 변경과 함께 들어간다(4e-2 실차 진입점에서 해제 조건이 실제로 연결될 때).
- 거부만으로 미션이 자동으로 다시 움직이는 일은 없다. 정지와 운용자 판단으로 끝난다(총괄 조건 '거부를 정지나 재계획으로'의 정지 쪽).

1. **거부(REFUSED)**: 아래 중 하나가 생기면 들어간다.
   - `command` 가 `Refusal` 을 낸다.
   - `steering_targets` 가 `None` 이다.
   - 측정 곡률이 `None` 이다(이유 `steering_unknown`).
   들어가면 정지 의도를 래치한다: `requested_speed = 0`, 조향 명령 유지, 바퀴는 유지 조향의 κ 로 제동하되 그 κ 가 `None` 이면 곡률 없이 0. 사건(요청 κ, 맵 범위, phase, 계획 식별자)을 남긴다.
2. **정지 확인**: D4 의 해제 두 조건(옛 비영 명령이 더 활성화될 수 없음 + 신선한 측정으로 정지)이 모두 참일 때까지 기다린다. 측정 연결은 4b 이므로, 4a' 는 가짜 훅으로 시험한다.
3. **재계획 요청**: AUTO 의 운반·복귀에서 `scene_replan_allowed()` 가 참이면 `scene_replan_request(..., "kinematic_refusal")` 를 계획 식별자(추종기 book serial)당 **한 번** 요청한다.
4. **presteer**: 재계획이 받아들여지면 조향 유지 래치를 풀고, 바퀴 0 으로 새 경로 시작 조향까지 돌린다(지금 scene_replan 의 presteer 와 검증을 그대로 쓴다).
5. **재개**: 새 계획이 검증되면 거부 상태를 지우고 주행을 재개한다.
6. **실패 처리**: 재계획 실패·예산 소진, `scene_replan_allowed()` 거짓, 지원하지 않는 phase(접근·삽입·하역), 새 계획에서 다시 거부(같은 식별자) 가운데 하나면 정지를 유지하고 미션 상태를 '운용자 복구 대기'로 둔다. 운용자가 취소·재시도를 정한다. 가용성보다 정지를 택한 결정이다(agy ③ 의 영구 정체 지적은 의도된 결과로 기록한다).
7. **TELEOP**: 거부를 운용자에게 돌려주고(그 틱 정지), 자동 재계획은 시작하지 않는다.
8. **초기화**: 미션 종료·취소, 새 미션 채택 때 거부 상태와 요청 기록을 지운다.
- 추종기·재계획·후보 미리보기가 같은 곡률 한계를 쓰도록 실차 진입점에서 맵 범위를 넘긴다(4e-2). 그렇지 않으면 같은 불가능 명령을 다시 만든다.

### D4. 포트 명령 계약 `forklift_core/plant/port.py`

- `PlantCommand`:
  - `decision_ns`, `expires_ns` — 실차는 호스트 단조 시각. Isaac 은 이 필드를 쓰지 않는다. 두 시간 영역을 섞지 않는다.
  - `control_gen` — **러너의 제어 세대**다. 콘솔 lease·미션 명령이 바뀔 때 오른다. MCU `arm_gen` 과 다르다.
  - `auth_epoch` — 명령을 만들 때 포트가 찍는 **허가 epoch** 다. `McuLinkHost` 의 허가가 시작되거나 끝날 때마다 오른다(지금 `_auth_gen`, 공개 속성 `auth_epoch` 로 노출). 제출 때 현재 epoch 와 다르면 `blocked("stale_authorization")` 이다. 그래서 같은 lease·미션·`control_gen` 을 유지한 채 재무장해도, 그 전에 만든 명령은 나가지 않는다(Codex 4a' 문서 P1-1).
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
  - 재무장(MCU 허가 새로)은 최신 의도 캐시를 비운다. epoch 가 바뀌므로 호출자가 들고 있던 옛 `PlantCommand` 도 막힌다.
  - **부분 송신 프레임**: `drop_drives` 는 큐에 남은 DRIVE 만 버린다. `TtyLink` 가 이미 쓰기 시작한 프레임은 끝까지 보내고 **outstanding 으로 추적**한다. 처음에는 COBS 구분자 `0x00` 으로 끊는 안을 검토했으나 버렸다. 마지막 구분자만 남은 경우 `wire[:-1] + 0x00` 은 원래 프레임과 같아서 MCU 가 그대로 받는다(Codex 4a' 문서 재검토 P1, 41바이트 DRIVE 의 분할 위치 전부를 디코더로 확인). 그러니 끊기는 취소를 보장하지 못한다. 첫 바이트 전에 EAGAIN 이 난 프레임은 지금처럼 큐로 되돌려(P0-7 `unsent`) 폐기할 수 있다. 끝까지 나간 프레임, 커널·USB 버퍼의 바이트는 회수할 수 없다. 정지는 MCU 마감(원래 만료 이하, 상한 cap)과 DISARM 이 보장하고, 해제 조건 (a) 가 그 프레임의 만료까지 기다린다(Codex 4a' 문서 P2-3, agy ③).
  - 재전송·부분 합성은 원래 `expires_ns` 를 넘지 않는다.
- **blocked 를 0 으로 기록하지 않는다**(Codex 4a' P1-1).
  - 이전 DRIVE 가 큐나 MCU 에서 아직 유효할 수 있다.
  - 실차 포트는 `outstanding_until_ns`(큐·송신된 DRIVE 가운데 가장 늦은 원래 만료)를 노출한다.
  - 코어는 blocked 를 받으면 정지 의도를 래치하고 미션 진행·재계획을 멈춘다.
  - **해제는 두 조건이 모두 참이어야 한다**(Codex 4a' 문서 P1-2). 만료는 영향 가능성을 관리하는 시각이지 정지 판정값이 아니다.
    (a) 옛 비영 명령이 더 활성화될 수 없다: 큐와 쓰는 중인 프레임에 비영 DRIVE 가 없고, 마지막으로 나간 비영 DRIVE 는 `outstanding_until_ns` 가 지났거나 MCU STATE 가 그 명령이 끝났다고 보인다(`command_fresh` 거짓 또는 STOPPING/DISARMED).
    (b) 신선한 측정으로 정지를 확인한다(4b).
    만료로 MCU 허가가 철회됐다면 운용자 재무장이 따로 필요하다.
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
4. 코어 1차원 경로: 가짜 훅과 `RealKinematics` 로 shape 불일치가 예외로 끝나는지 본다. 거부 상태기 전체(거부 → 정지 확인 → 재계획 한 번 → presteer → 검증 → 재개)와 실패 경로(재계획 실패·지원하지 않는 phase·같은 계획 재거부 → 운용자 대기)를 확인한다. `None` 조향의 곡률 없는 0 제동, TELEOP 거부, AUTO·TELEOP 이 같은 실차 문턱을 쓰는지도 본다.
5. 포트 + `McuLinkHost`(가짜 STATE):
   - DRIVE 프레임의 바퀴·q·valid·`lift_hold` 가 기대값인지.
   - 비영 DRIVE 로 큐를 채운 뒤 zero 제출이 blocked 일 때 `outstanding_until_ns` 가 남는지.
   - 세대 변경 → 대기 DRIVE 폐기.
   - 재무장 뒤 옛 명령 재제출은 blocked — **같은 lease·미션·`control_gen` 을 유지한 재무장** 반례 포함.
   - 부분 송신: 모든 분할 위치(0…len−1 바이트를 쓴 뒤 `drop_drives`)에서, 0 바이트면 큐로 돌아가 폐기되고 1 바이트 이상이면 프레임이 끝까지 나가 outstanding 에 남는다. 해제 조건 (a) 가 그 만료까지 거짓이다. 다음 정상 프레임까지 재동기화되는지도 본다.
   - 해제 조건: 측정 정지만 있고 큐에 비영 DRIVE 가 남은 경우, 만료만 있고 정지 측정이 없는 경우 둘 다 해제하지 않는다.
   - `lift_mode` 두 값.

## 3. 위험

- **게이트 동치**: 운동학 래퍼가 산술 순서를 바꾸면(예: 배열 복사, dtype) 기록된 값이 달라진다. D5-2 의 바이트 비교로 먼저 막는다.
- **차원 독립화의 범위**: 코어 전반의 2차원 가정을 빠짐없이 찾는 것이 어렵다. AST 검색으로 `steering_command[`·`steering_position()[`·`[1]` 패턴을 목록으로 만들어 PR 에 붙인다.
- **문턱의 단위**: q 단위 문턱(creep·presteer)과 q 속도는 실측 전이라 `vehicle.yaml` 에 '실측 대기' 꼬리표를 단다.
- **게이트가 덮는 범위**(agy ③): runs29 는 Isaac 경로만 재생한다. 실차 분기(1차원 조향, 거부 상태기, blocked, epoch, 부분 송신, `lift_mode`)는 게이트가 한 줄도 실행하지 않는다. 그쪽은 D5-3·4·5 의 시험이 덮고, 뒤로 가짜 포트 정상 미션(4e-2)과 HIL(P0-9)이 덮는다. 게이트 29/29 를 실차 경로의 안전 근거로 쓰지 않는다.
- **승강 높이에 따른 한계**(agy ③): 포크가 높을수록 무게중심이 올라가므로 곡률·조향 속도·감속 한계를 줄여야 할 수 있다. 이 설계의 실차 한계는 정적(`vehicle.yaml`)이다. 승강 높이별 한계 축소는 후속 결정 항목으로 총괄에 올린다. 그 전까지 실차 단계(계획 5절)는 빈 포크·저속·낮은 승강에서만 진행한다.
- **조향 맵의 이력·하중 의존**(agy ③): q → κ 를 정적 단조 맵으로 둔다. 링크 유격·히스테리시스·하중에 따른 차이는 S8 실측으로 크기를 보고, 크면 맵을 방향별로 나누거나 오차 여유를 문턱에 넣는다.
- **호스트–MCU 시계**(agy ③ 1번): 시계 동기화를 전제하지 않는다. MCU 마감은 echo 한 STATE 의 MCU 시각 기준이고(mcu-link 계약 2절, P0-7), 호스트 쪽 만료는 호스트 단조 시각만 쓴다. 전송 지연은 마감을 늦추지 않고 앞당기기만 한다.

## 4. 자체 검토

**① 원자료·코드 대조(Claude)**: `(+0.08, −0.08)` 의 평균 0·상한 0.11768 1/m 를 `wheel_odometry` 식으로 다시 계산해 맞음. creep 0.05·presteer 0.02 의 정의 위치를 확인해 표에 적음. `scene_replan_allowed`(core.py 7966)·`scene_replan_request`(8098) 위치 확인.

**구현 리뷰(Codex 4a' 구현 P1 3·P2 3)**: 쓴 비영 DRIVE 전부 추적(해제 (a)), 측정 조향 무효 → 거부, 합성 프레임이 원래 만료를 넘지 않음, 조향을 알 수 없는 정지는 마지막 의도의 q 로 0 프레임 또는 DISARM, presteer 비교와 문턱을 운동학으로, 거부 상태기는 정지·운용자 일시정지까지만(자동 재계획은 뒤로), 소스 계약 시험 갱신, `TtyLink` 실제 바이트 부분 송신 시험.

**② Codex 적대 검토(설계 상의 P1 3·P2 4 반영 뒤 문서 검토 P1 2·P2 3)**
- 재무장 뒤 옛 명령 → `auth_epoch`.
- 해제 조건 → (a) 옛 비영 명령 비활성 + (b) 측정 정지.
- 부분 송신 → 처음 안(`0x00` 으로 끊기)은 재검토 P1 반례(구분자만 남으면 원래 프레임 완성)로 버렸고, 쓰기 시작한 프레임은 끝까지 보내 outstanding 으로 추적한다.
- TELEOP 문턱 주입.
- 거부 상태기 전체 전이와 실패·초기화.

**③ agy(Gemini 3.8 Flash, ①② 를 보지 않음) '적히지 않은 전제' 7개**
- 반영: 무효 조향의 곡률 없는 0 제동, 게이트 범위의 한계 명시, 부분 송신·버퍼 회수 불가, 승강 높이별 한계(후속 결정·저속 단계 제한), 맵 이력(S8).
- 이미 다룬 것: 시계 동기화(echo 기반이라 전제하지 않음).
- 의도된 결과로 기록한 것: 재계획 한 번 뒤 영구 정체(가용성보다 정지).
