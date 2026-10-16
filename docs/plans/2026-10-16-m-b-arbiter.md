# M-b 중재기 (`forklift_core/safety/`) 계획

작성: 2026-10-16 · 상태: **3판 — Codex 2회차까지 반영, 재검토 중** · 근거: [운용 콘솔 설계](../design/2026-10-13-operator-console.md) 3절·4.1–4.5·5.1–5.4, [콘솔 계약 초안](../interfaces/2026-10-13-operator-console-contract-draft.md) 1–6·9절, [M-a 계획](2026-10-15-m-a-mission-core.md) D6, [M-a3 계획](2026-10-16-m-a3-mission-commands.md)

## 1. 범위

**목표.** 설계 3절 표의 중재기 책임 — 조종 권한 lease, 세대·신선도 검증, 모드 전이, 무장·deadman, 명령 출처 선택, 출처와 무관한 이동 금지 — 를 Isaac·ROS 없이 도는 순수 모듈로 만든다. M-c(Isaac 실시간 실행기)와 C2(게이트웨이)가 이 모듈을 그대로 쓴다.

- 포함
  - 세대 4종(`session_epoch`·`lease_id`·`control_gen`·`arm_gen`)과 신뢰 요청 처리 순서(STOP 예외, 결과 저장, 저장 상한이면 `busy`)
  - lease 발급·만료·재연결 유예
  - 모드 `PAUSED`·`AUTO`·`TELEOP`·`STOPPED` 와 정지 종류, `rearm`
  - 회신 토큰(tick) 발급·보관, 텔레옵 명령 검증과 절대 만료, 영상 신선도
  - 무장·deadman·입력 상태
  - 매 제어 주기의 주행·승강 허가(설계 4.5)와 TELEOP 명령 사슬(비율 → 상한 → 곡률 → slew → 허가 상한 → 선점)
  - TELEOP 허가 입력: 명령 곡률 원호를 `DrivePermission.update()` 로 설치한 뒤 `allowed_speed` 를 읽는 함수
  - 미션 쪽 보류 `held`(M-a3 에서 M-b 로 미룬 `hold_on`/`hold_off`)
  - 계약 6절 상태 메시지 중 중재기 몫을 만드는 함수
- 제외
  - ROS 2 노드·msg·게이트웨이(C2), Isaac 실시간 실행기와 TELEOP 출력의 실제 구동 연결(M-c)
  - 복구 동작 `withdraw_straight`: 이번에는 `capabilities.recovery_actions = []` 로 두고 `recovery` 요청은 `reject(unsupported)` 로 답한다. 동작 생성기는 M-c 와 함께 만든다
  - 수동 승강 명령: 승강 허가(`lift_allowed`)는 매 주기 판정하지만, 텔레옵의 `lift_dir` 를 실제 승강으로 옮기는 것은 설정 `teleop_lift`(기본 `false`)가 켜졌을 때만 한다. 설계 4.5 가 "수동 승강을 지원할지는 따로 정한다"고 남겨 둔 결정이다(6절 열린 질문)
  - 잠정 수치(`command_max_age_s` 300 ms, 전진 0.5·후진 0.25 m/s)의 확정. H3 실측 뒤에 정한다

**완료 조건.** 2절의 결정을 시험으로 고정하고, 기존 기준 기록 29건의 재생과 ws1 전체 시험이 바뀌지 않는다(M-b 는 배치 실행기의 결정 경로를 바꾸지 않는다).

## 2. 결정

**B1 모듈과 순수성.** `src/forklift_core/safety/` 에 다음을 둔다. numpy 는 허가 함수에서만 쓴다.

| 파일 | 내용 |
|---|---|
| `generations.py` | 세대 값과 신뢰 요청 결과 저장소(`RequestBook`) |
| `ledger.py` | 회신 토큰 송출 기록(`TickLedger`), 카메라 프레임 기록(`FrameLedger`) |
| `arbiter.py` | `Arbiter`: 연결·lease·요청 처리·모드·무장·텔레옵 검증·매 주기 `step()` |
| `teleop_policy.py` | 설계 4.5 표: 미션 안전 상태 → `drive_allowed`·`lift_allowed`·`teleop_allowed`·사유 |
| `teleop_drive.py` | TELEOP 명령 사슬과 원호 허가 질의 |

시계와 난수원은 주입한다(계약 9절 1). 모든 공개 메서드가 `now_mono` 를 인자로 받고, 토큰·`lease_id` 는 생성자에 넘긴 `token_source: Callable[[], str]` 에서 받는다. 실행기는 `time.monotonic()` 과 `os.urandom(16).hex()` 을 넘기고, 시험은 고정 값을 넘긴다. 모듈은 스스로 시계를 읽지 않는다.

**B2 세대.**

- `session_epoch`: 생성자 인자. `Arbiter` 하나가 세션 하나다. 다르면 `reject(stale_epoch)`.
- `lease_id`: 권한을 줄 때마다(유예로 돌려줄 때 포함) 새로 받는다.
- `control_gen`: 계약 1절의 경계(STOP·안전 선점, 명령 만료, lease 만료, 텔레옵 중 끊김 정지, 일시정지, 재무장 완료)에 더해 **모든 모드 전이와 반복 STOP** 에서 1 오른다. 계약 1절 목록보다 넓다(6절 계약 델타 1). 근거: 모드 전이마다 무장이 풀리므로(설계 5.1) 전이 전에 보낸 무장·작업 요청이 전이 뒤에 실행되면 안 된다. AUTO 중 lease 만료는 계약대로 이 값과 권한만 바꾸고 모드·정지 종류·phase 는 그대로 둔다.
- `arm_gen`: `arm` 수락마다 1 오른다.

**B3 한 틱의 순서와 신뢰 요청 처리(계약 1절).** 중재기는 스레드 안전하지 않다. 실행기의 제어 스레드 하나가 매 주기 다음을 이 순서로 부른다. 게이트웨이가 받은 메시지는 큐에 쌓였다가 ① 에서만 처리된다. 그래서 주기 도중에 들어온 STOP 은 다음 주기 머리에서 효과를 낸다(최대 한 주기 지연; 마지막 방어선은 구동 어댑터의 자체 만료, 설계 3절).

1. `arbiter.handle(...)` — 쌓인 메시지를 도착 순서대로. 신뢰 요청은 아래 0–2 순서.
2. `arbiter.mission_context(now)` — 이번 주기에 미션에 넘길 정지 종류(B5)와, ① 에서 받아 둔 미션 요청 가운데 **지금도 세대·모드가 유효한 것**만 `core.commands` 에 넣는다. ① 과 ② 사이에 경계(STOP·모드 전이)가 있었으면 그 요청은 `reject(stale_control)` 로 확정한다.
3. `core.commands.apply(t, Context(...))` 와 코어의 제어 틱.
4. `arbiter.resolve(records)` — ③ 의 생명주기 기록으로 미션 요청의 `ack`/`reject(사유)` 를 확정·저장하고, 수락된 것에 따라 모드를 바꾼다(B5). 미션 요청은 같은 주기 안에서 확정되므로 경계를 넘어 남는 보류 요청이나 예약된 모드 전이가 없다.
5. `arbiter.step(...)` — 출력(B11).

신뢰 요청 처리 순서:

0. STOP: 연결이 `control: true` 면 세대·저장소와 무관하게 정지를 실행하고 `ack` 한다. 저장하지 않는다. 이미 `STOPPED` 여도 무장 해제와 `control_gen` +1 을 한다 — 첫 STOP 뒤 만든 `rearm` 이 두 번째 STOP 뒤에 실행되면 안 된다.
1. 저장소 조회: 키는 `(client_id, request_id)`(`session_epoch` 는 `Arbiter` 하나에 하나라 키에 넣지 않는다). 있으면 저장한 결과를 그대로 돌려준다. 미션 요청이 아직 ④ 전이면 답하지 않는다(같은 주기 안에 확정된다).
2. 처음이면 저장 상한(개수, 기본 4096)을 먼저 본다. 닿았으면 `reject(busy)` 를 **저장하지 않고** 돌려준다. 아니면 세대·권한 검사 → 실행 → 결과 저장.

**B4 lease.**

- 요청: 보유자가 없고 유예 예약도 없으면(또는 예약이 같은 `client_id` 면) 새 `lease_id` 와 `lease_timeout_s` 를 준다. 예약이 다른 클라이언트 것이면 `reject(lease_reserved)`, 다른 보유자가 있으면 `reject(no_lease)`.
- 갱신 기준 시각: 보유자가 보낸 유효한 회신(`echo` 또는 텔레옵 명령의 `echo`)이 가리킨 tick 의 송출 시각. 발급 직후에는 발급 시각. `now > 기준 + lease_timeout_s` 면 만료.
- 만료와 끊김: 권한 해제, `control_gen` +1, 같은 `client_id` 에 `grace_s` 동안 예약. 모드별로:
  - AUTO: 그대로(사용자 결정, 설계 2절).
  - TELEOP: 안전 선점 래치, 무장 해제, 모드 `PAUSED`(설계 4.1 "수동 조종 끝"), 미션 `hold_off`(B6).
  - PAUSED·STOPPED: 권한만 푼다. `STOPPED` 는 `rearm` 말고는 어떤 경로로도 바뀌지 않는다.
- `lease_release`: 예약을 만들지 않는다. 나머지는 위와 같다.

**B5 모드, 정지 종류, 미션에 넘기는 정지.** 중재기는 매 주기 두 값을 낸다.

| 상태 | 구동 출력 정지 종류 | 미션 `Context.stop_kind` |
|---|---|---|
| `STOPPED`, 또는 선점 래치(만료·lease 상실·끊김·피드백 상실, 정지 확인 전) | `preempt` | `preempt` |
| `PAUSED` | `controlled` | `controlled` |
| `TELEOP` | `none`(조종자 명령이 움직인다. 주행 금지는 B8 의 최종 상한 0) | `controlled`(미션은 `held`) |
| `AUTO` | `none` | `none` |

- **불변식: 미션은 AUTO 에서만 돈다.** `PAUSED`·`TELEOP`·`STOPPED` 에서는 미션에 정지를 넘기므로 생명주기가 `running`·`resuming` 이면 그 주기에 `paused` 로 넘어간다(M-a3d). 승강 램프·단계 전이·추종이 함께 멈춘다(설계 4.2). 미션의 `pause` 수락 여부에 기대지 않는다.
- 재개·제출은 미션에 `none` 을 넘겨야 받는다(생명주기는 정지 중 `resume`·`submit` 을 `stopping` 으로 거절한다). 그래서 ② 는 **미션 상태가 그 요청을 받을 수 있을 때만** `none` 을 넘긴다: `mission_resume` 은 생명주기가 `paused` 일 때, `mission_submit` 은 `idle`·종료 상태일 때. 그 밖에는 `controlled` 를 넘기고(그러면 `running` 인 미션은 그 주기에 멈춘다) 요청은 생명주기의 거절 사유로 확정된다. `none` 을 넘긴 주기에도 미션이 움직이기 시작하는 것은 수락된 요청 하나 때문뿐이다.
- **AUTO 유지 조건은 틱이 끝난 뒤의 미션 상태다.** ④·⑤ 에서 미션 상태가 `accepted`·`running`·`resuming`·`cancelling` 이면 AUTO, `paused`(재개 절차 거절 포함 — 수락 기록 뒤 같은 틱의 `resume_step` 이 `no_resume` 을 낼 수 있다)·`idle`·종료 상태면 `PAUSED` 로 돌아간다(`control_gen` +1, 사유는 미션의 마지막 기록). 그래서 중재기 AUTO 와 미션 `paused` 가 같이 남지 않는다.
- **근접 추적 중 일시정지.** 근접 추적 중(`near.active`, B7) 미션의 일시정지·재개는 아직 구현되지 않았다(M-a3 제외 범위 R4–R6). 그래서 그동안 `mode_request to: PAUSED`·`mission_pause` 는 `reject(near_tracking_active)` 로 거절하고 AUTO 를 유지한다(설계 4.1 "어느 단계에서나 받는다"의 예외, 6절 계약 델타). STOP 은 언제나 받는다: 미션은 `preempt` 로 일시정지되고, 근접 추적 중에 멈춘 미션의 재개는 미션이 `no_resume:near_tracking` 으로 거절한다(M-b2 에서 코어에 더함). 그 뒤는 복구 동작 몫이다.

전이(괄호 안은 요청자). **`PAUSED` 에서 나가는 전이(AUTO·TELEOP)는 모두 정지 확인(B9)을 요구한다** — 아니면 `reject(not_stopped)`(설계 4.2 "실측 정지가 끝나기 전에는 모드를 넘기거나 재개하지 않는다"). AUTO → TELEOP 직행은 없다: 먼저 `PAUSED`.

- `STOP`(control 연결 누구나): 어느 모드에서나 `STOPPED`, 무장 해제, `control_gen` +1(B3 0).
- `rearm`(보유자): `STOPPED` 에서만. 바퀴 속도 피드백이 없으면 `reject(no_feedback:rear_axle_speed)`, 정지 확인 전이면 `reject(not_stopped)`. 수락하면 `PAUSED`.
- `mode_request to: PAUSED`(보유자): `STOPPED` 면 `reject(stopped_locked)`. AUTO 면(근접 추적 중이 아니면) `PAUSED`. TELEOP 이면 무장 해제, `PAUSED`, 미션 `hold_off`.
- `mode_request to: TELEOP`(보유자): `STOPPED` 면 `stopped_locked`, AUTO 면 `reject(invalid)`(먼저 PAUSED), 정지 확인 전이면 `not_stopped`, `teleop_allowed`(B7)가 거짓이면 `reject(state_forbids_teleop)`. 수락하면 미션 `hold_on`.
- `mission_submit`(보유자): `PAUSED` 에서만, 정지 확인 뒤. 수락되면 AUTO.
- `mission_pause`: AUTO 에서. `PAUSED` 로(위 `to: PAUSED` 와 같다).
- `mission_resume`: `PAUSED` 에서, 정지 확인 뒤. 수락되면 AUTO(미션은 재개 절차 동안 0 을 낸다).
- `mission_cancel`: `PAUSED`·AUTO 에서. 미션이 끝나면(done·failed·cancelled) AUTO 는 `PAUSED` 로.
- **정지 실패.** `PAUSED` 나 선점 래치가 `not_stopped_s`(10 s, 미션 E3 과 같은 값) 안에 정지 확인을 얻지 못하면 `STOPPED`(사유 `not_stopped`)로 넘긴다 — 고장 판정이다(설계 4.1). 코어의 `not_stopped` 도 같은 판정이므로 중재기는 자기 판정기로 본다.
- 재접속·통신 복구는 전이를 일으키지 않는다(설계 4.1).
- `TELEOP` 에서 나가는 모든 경로(요청·lease 상실·끊김·STOP·정지 실패)는 미션 `hold_off` 를 부른다.

**B6 미션 보류 `held`.** `MissionCommands` 에 `hold_on(t)`·`hold_off(t)` 를 더한다.

- `hold_on`: 상태가 `accepted`·`running`·`paused`·`resuming` 이면 `held`. 열린 정지 구간이 없으면 연다(작업 시간 정지). `mission_gen` +1. `resuming` 의 재개 절차는 버린다.
- `hold_off`: `held` → `paused`. 정지 구간은 열린 채로 두고, 다시 움직이려면 `mission_resume` 이 필요하다. 재개 절차(R1–R15)가 차체가 옮겨진 것을 확인한다.
- 코어는 `held` 를 `paused` 와 같이 처리한다.
- `held` 중 `pause`·`resume` 은 `held` 로 거절, `cancel` 은 받는다.

**B7 수동 조종 허용 상태(설계 4.5).** `teleop_policy(safety, feedback)` 를 매 주기 부른다. 입력은 코어 `safety_state()` 와 피드백이다. 코어의 안전 상태에 두 필드를 더한다(M-b2):

- `near.active`: 근접 계약이 **지금** 적용되는가 — 근접 추적기가 있고 단계가 `approach`·`insert`(코어가 `near_tick` 을 부르는 조건과 같다). 기존 `tracker_built` 는 정상 종료 뒤에도 참으로 남아 근접 구간 판정에 쓸 수 없다. 배치 실행기가 미션에 넘기는 `Context.near_tracker` 도 이 값으로 바꾼다.
- `frame.slam`: SLAM 이 있는가. 없으면 `frame.hold` 가 언제나 `none` 이라 근접 구간을 알 수 없다.

| 조건(위에서부터) | 주행 | 승강 | 사유 |
|---|---|---|---|
| 안전 상태 없음, `load`·`fork_in_pocket` 을 모름 | 거부 | 거부 | `state_unknown` |
| 바퀴 속도 피드백 없음·낡음(B9) | 거부 | 거부 | `no_feedback:rear_axle_speed` |
| `fork_in_pocket` 참 | 거부 | 거부 | `fork_in_pocket` |
| `near.active`, 또는 `frame.hold` 가 `approach`·`insert` | 거부 | 거부 | `near_section` |
| `frame.slam` 거짓이고 단계가 `approach` | 거부 | 거부 | `near_section_unknown` |
| 적재(`load.loaded`) | 허용(적재 상한·적재 footprint) | 거부 | — |
| 빈 차체 | 허용 | 허용(정지 확인 중일 때만) | — |

- `teleop_allowed` = 주행 허용. 매 주기 판정한다. TELEOP 중 주행이 거부로 바뀌면 그 주기 출력은 **최종 허가 상한 0 으로 계단식으로 자른다**(B8 4단계) — 설계 4.2 의 이동 금지 판정은 안전 선점이다. 바퀴 피드백 상실은 선점 래치도 건다.
- `basis` 가 `phase` 인 값은 단계 추정이다. 실차 적재 센서가 생기면 바꾼다.
- 들기·내리기 단계는 `fork_in_pocket` 이 참이라 이미 거부된다. 수동 승강(`teleop_lift` 를 켰을 때)은 주행과 배타적이다.

**B8 TELEOP 명령 사슬(설계 4.4 적용 순서).** 매 제어 주기:

1. 비율 → 물리량: `speed = speed_ratio × 상한`(모드 `direct`·`video`, 전진·후진, 적재별 설정). 목표 곡률 `κ* = curvature_ratio × teleop_curvature_inv_m`.
2. 곡률 변화율 제한: 출력 곡률 `κ` 는 `κ*` 를 향해 주기당 `teleop_curvature_rate × dt` 만 움직인다(조향 전환 구간을 덮는 운용 제한, 설계 4.4 보장 범위 — 아래 원호 질의와 함께).
3. deadman 이 거짓이거나 무장이 아니면 목표 속도 0.
4. 속도 사슬 — 코어 `assemble_command` 의 속도 계산과 **같은 함수**(M-b4 에서 코어에서 뽑아 둘이 함께 쓴다; 재생 29건으로 같음을 확인): 조향 따라잡기(목표 조향과 실측 조향 차 > 0.05 rad 이면 목표 속도 ×¼) → slew(감속은 정지 모델 `decel_mps2`, 가속 `command_accel_mps2` 0.8 m/s², 부호가 바뀌면 먼저 0) → 최종 허가 상한(원호 질의의 상한, `drive_allowed` 가 거짓이면 0; 넘으면 계단식으로 자름) → 선점이면 0. 시작값은 직전 **적용** 속도다. 따라잡기를 slew 뒤에 두면 감속 한계를 넘는 계단이 생긴다(0.5 m/s 에서 0.125 대 0.4875).

원호 질의: 중재기 전용 `DrivePermission` 인스턴스(미션의 허가 상태를 건드리지 않는다)에 같은 snapshot 을 쓴다.

- 조향 전환 덮기: 실측 조향 곡률 `κ_m` 부터 출력 곡률 `κ` 까지를 0.05 m⁻¹ 이하 간격으로 나눈 곡률마다 원호를 `update()` 로 설치하고 `allowed_speed`(정지 호 곡률도 같은 값)를 읽어 **가장 작은 상한**을 쓴다. 조향이 그 사이를 단조로 움직이는 동안의 궤적은 이 원호들의 다발 안에 든다는 근사이며, 간격 사이의 틈은 허가 설정의 envelope 여유가 덮는다고 본다(검증 대상, 6절).
- 방향: 실측 속도 크기가 정지 문턱보다 크면 실측 이동 방향, 아니면 요청 방향(요청 0 이면 직전 적용 방향, 그것도 0 이면 전진).
- 원호 길이: `lookahead_m + stopping.distance_m(검사 속도) + step_m` — 허가 검사 길이보다 길어 `reached_end` 가 서지 않는다(수동 조종에는 목표점이 없다).
- 검사 속도: `max(|실측 속도|, |직전 적용 명령|, |이번 요청 속도|)` — 계약 9절 2 의 "남은 속도와 직전 적용 명령을 모두 덮는 값"에 코어가 허가를 부를 때 쓰는 이번 요청을 더한 것. 예: 직전 0.5·실측 0.4·요청 0 이면 0.5(정지 거리 0.208 m), 직전 값을 빼면 0.4(0.163 m)로 줄어든다.
- 센서 침묵과 증거 나이: 장애물 층이 미션에 넘기는 것과 같은 `sensors_last_s` 를 넘긴다. 격자가 낡으면 허가가 0 을 낸다.
- footprint: 적재면 `footprints(loaded=True)`, 아니면 빈 차체.

**출력 경계.** 중재기의 TELEOP 출력은 사슬을 마친 바퀴 속도와 곡률이다. 실행기는 이것을 Ackermann 바퀴 목표와 조향 속도 제한에만 넣는다(따라잡기·slew 는 이미 사슬 안에서 했다). 그 결과(`applied`)는 다음 주기 피드백으로 중재기에 돌아간다. 조향이 늦게 따라오는 구간은 위 원호 다발이 덮는다.

**B9 정지 확인과 피드백.** 중재기가 자기 `StopDetector`(`forklift_core/localization/slam_pose.py`, 코어 재개 1단계와 같은 판정기, 기본값)를 가진다. 입력은 직전 적용 속도, 바퀴 속도 피드백, 요 각속도다. 피드백은 측정 시각(`vehicle_mono`)을 함께 받고, `feedback_max_age_s` 보다 낡았으면 없는 것으로 본다. 피드백이 없으면 정지 확인은 언제나 거짓이다 — 그런 차량은 `rearm`·재개·TELEOP 진입을 받지 않는다(설계 4.3).

**B10 회신 토큰과 텔레옵 검증(계약 4절).**

- `tick(now)`: `state_seq` +1, `token_source` 의 토큰을 `(state_seq, token) -> sent_mono` 로 기록. 기록은 시간으로만 지운다: `now − sent_mono > max(command_max_age_s, lease_timeout_s)` 인 것.
- 텔레옵 명령 검증 순서와 버릴 때의 사유(응답 없음, 상태의 `last_teleop_reject`): `stale_epoch` → `stale_lease` → `stale_control` → `stale_arm` → `stale_seq`(같은 `arm_gen` 안에서 같거나 작은 `seq`) → `bad_echo` → `expired` → `stale_video` → `not_armed`(TELEOP 이 아니거나 무장이 아님).
- 만료 시각: `sent_mono(echo) + command_max_age_s`. 영상 모드면 `acquired_mono(frame_id) + video_max_age_s − camera_latency_bound_s` 와의 최솟값.
- `step()` 머리에서 현재 명령의 만료를 검사한다. 지났으면 선점 래치, 무장 해제, `control_gen` +1, 사유 `expired`(또는 `stale_video`). 무장 뒤 아직 명령이 없으면 만료 시각이 없고 출력은 0 이다 — 첫 명령도 회신 만료를 거치므로 오래 묵은 명령이 될 수 없다.
- `input_state`(가림·포커스·패드 분리)는 무장만 푼다.
- 프레임 기록: 같은 `frame_id` 의 재등록은 처음 시각을 지킨다. `frame_id` 는 세션 안에서 증가만 하므로(계약 1절) 지금까지 본 가장 큰 번호 이하인데 기록에 없는 번호(창 밖으로 지운 것)는 등록하지 않는다 — 지운 촬영을 다시 등록해 나이를 0 으로 되돌릴 수 없다.

**B11 `step()` 의 입출력.**

- 입력: `now_mono`, 피드백(바퀴 속도와 측정 시각, 요 각속도, 조향, 승강, 직전 `applied`), 코어 `safety_state()`, 코어 후보(코어 `assemble_command` 가 낸 바퀴 속도·곡률), 허가 질의(snapshot·자세·footprint·`sensors_last_s`).
- 출력: `mode`, `stop_kind`, `source`(`auto`·`teleop`·`core_stop`), 바퀴 속도·곡률, `permit`(상한·사유), `drive_allowed`·`lift_allowed`·`teleop_allowed`·사유, 세대, `armed`, lease 요약.
- 순서: ① 만료·lease 만료 → ② 정지 확인 갱신, 래치 해제(정지 확인되면)·정지 실패 판정 → ③ 정책(B7) → ④ 출처: AUTO 면 코어 후보 그대로. `PAUSED` 면 코어 후보(코어가 일시정지 틱에서 감속시키는 값)를 쓰되 크기가 직전 적용 속도를 넘지 못하게 자른다. TELEOP 이면 B8. → ⑤ `preempt` 면 0.
- **감속의 소유자는 하나다.** 출처가 코어가 아닌 주기(TELEOP, 선점 0)에 M-c 실행기는 최종 적용 값을 코어가 읽는 명령 상태 셋에 되쓴다: `command_speed`(코어 slew 시작값), `applied_command`(정지·cusp 판정), `slam["last_command"]`(SLAM 정지 판정기의 명령 입력 — 이것이 0 으로 남으면 TELEOP 중 움직이는 차를 '정지'로 판정해 release·재개가 잘못 일어난다). 그래서 TELEOP 에서 `PAUSED` 로 넘어가면 코어가 실제 속도에서 감속한다. AUTO·PAUSED 에서는 출력이 코어 후보와 같으므로 되쓸 것이 없고, 배치 실행기(중재기 없음)는 바뀌지 않는다.
- AUTO 에서 코어 후보를 다시 slew·허가하지 않는다. 코어가 이미 같은 사슬을 적용했고, 다시 거르면 배치 실행과 달라진다.

**B12 상태 메시지.** `state_message(...)` 가 계약 6절의 중재기 몫(`mode`·`stop_kind`·`control_gen`·`armed`·`arm_gen`·`lease`·`command`·`applied`·`permit`·`limits`·`drive_allowed`·`lift_allowed`·`mission`·`teleop_allowed`·`teleop_forbidden_reason`)을 만든다. 잴 수 없는 값은 `null`. `pose`·`freshness`·`pacing` 은 실행기가 넘긴다.

**B13 경계 밖(밝혀 둘 전제).** 중재기 출력이 구동기에 늦게 닿거나 프로세스가 멈추는 경우는 구동 어댑터의 자체 시계 만료(실차 MCU, Isaac 은 M-c 실행기)가 막는다(설계 3절). 한 세션에 중재기가 하나라는 것은 실행기가 보장한다(M-c). 정지 모델은 빈 차체 기준 하나뿐이다 — 적재 정지 거리·경사는 실차 TELEOP 을 켜기 전 검증 대상이다(설계 4.5).

## 3. 수치

모두 잠정값이며 설정으로 받는다(설계 5.4).

| 이름 | 잠정값 | 근거 |
|---|---|---|
| `command_max_age_s` | 0.30 | 설계 5.4 목업 |
| tick 주기 | ≤ 0.075 s | `command_max_age_s / 4` |
| `lease_timeout_s` | 1.0 | ≥ `command_max_age_s` |
| `grace_s` | 10 | 정책값 |
| `video_max_age_s` | 0.5 | 잠정 |
| `camera_latency_bound_s` | 0.15 | 잠정. M-c 에서 잰다 |
| 텔레옵 상한 전진·후진 | 영상 0.5·0.25 m/s, 육안은 그 이하로 시작 | 설계 5.3·5.4 |
| 적재 상한 | 0.30 m/s 이하 | `transport_speed_mps` |
| `teleop_curvature_inv_m` | 0.36 | 실측 모델 추종기(모델 한계 0.3665) |
| `teleop_curvature_rate` | 0.5 m⁻¹/s | 잠정 운용 제한 |
| 정지 모델 | 0.15 s, 1.5 m/s², 0.05 m | `config/obstacle_layer.yaml` |
| `feedback_max_age_s` | 0.1 | 잠정 |
| `not_stopped_s` | 10 | 미션 E3 |
| 결과 저장 상한 | 4096 | 정책값 |

재계산(검토 ①): 0.5 m/s 에서 정지 거리 = 0.5·0.15 + 0.25/3 + 0.05 = 0.208 m, 0.25 m/s 0.108 m, 0.30 m/s 0.125 m. 검사 속도가 0 이면(정지 중, 요청 0) 정지 호 길이는 여유 0.05 + 표본 간격 0.02 = 0.07 m 이고 그 상한은 `speed_for(0.07)` = 0.108 m/s 다. 요청이 있으면 검사 속도에 들어가 정지 호가 그만큼 길어진다. 설계 4.4 의 slew 예시(0.5 → 0.4975 m/s)는 감속 0.3 m/s² 기준이고, 코어가 실제로 쓰는 감속 1.5 m/s² 로는 0.4875 m/s 다 — 어느 쪽이든 새 상한 0.1 을 넘으므로 slew 뒤 허가 상한을 두는 순서의 근거는 같다.

## 4. 조각

| 조각 | 내용 | 시험 |
|---|---|---|
| M-b1 | `generations.py`·`ledger.py`, 연결·lease·신뢰 요청 처리 순서·저장 상한·STOP | 단위 |
| M-b2 | 모드·정지 종류·`rearm`·정지 확인·정지 실패, 한 틱 순서(②·④), `MissionCommands.hold_on/off`, 코어의 `held`·`near.active`·`frame.slam`·근접 중 정지의 `no_resume`, 배치 실행기 `Context.near_tracker` 정정 | 단위 + 코어 시험, 재생 29건·ws1 전체 시험 |
| M-b3 | 텔레옵 검증·만료·영상 신선도·무장·입력 상태 | 단위 |
| M-b4 | 코어 `assemble_command` 의 속도 사슬을 순수 함수로 뽑기(코어와 중재기가 공유), `teleop_policy`·`teleop_drive`(사슬, 원호 다발 허가) | 단위(합성 격자), 재생 29건 |
| M-b5 | `state_message`, 가짜 차량 루프 시나리오(만료 정지, lease 유예, STOP→rearm, TELEOP 진입 거부, 적재 운반 텔레옵, PAUSED 감속, 정지 실패) | 통합(CPU) |

각 조각마다 Codex 구현 리뷰를 수렴까지 받는다.

## 5. 검증

- 단위 시험은 시계·토큰을 주입해 시각을 고정한다(만료 경계: 만료 시각과 같으면 유효, 넘으면 만료).
- 재생 29건: 바뀌면 안 된다.
- 가짜 차량 루프: 1차 지연으로 속도를 따르는 점 질량 차체와 합성 격자. 허가가 막는 장애물 앞에서 TELEOP 이 정지 호 안에서 멈추는지, 조향을 꺾는 동안에도 멈추는지 본다.

## 6. 열린 질문과 계약 델타

계약 델타(합의 문서에 반영):

1. `control_gen` 은 모든 모드 전이와 반복 STOP 에서도 오른다.
2. 거부 사유 `not_stopped`(정지 확인 전의 `rearm`·재개·제출·TELEOP 진입), `near_tracking_active`(근접 추적 중 일시정지).
3. AUTO → TELEOP 직행은 없다(먼저 `PAUSED`).
4. 미션 요청의 응답은 같은 제어 주기의 생명주기 판정 뒤에 나간다.
5. `recovery` 는 이번에 `unsupported`.

검증 대상: 조향 전환 덮기의 원호 다발 근사(간격 0.05 m⁻¹ 과 envelope 여유)가 실제 조향 응답에서 충분한지 — M-c 의 Isaac 실행과 H3 에서 잰다.

사용자 결정이 필요할 수 있는 것: 수동 승강 지원(`teleop_lift`, 기본 끔).

## 7. 자체 검토 기록


- **① 재계산(Claude).** 3절 끝. 정지 확인은 코어와 같은 `StopDetector` 로 바꿨다. 설계 slew 예시의 감속값 차이를 적었다.
- **③ agy 독립 검토(Claude Opus 4.6 Thinking, 결론 미공개).** 반영: `handle`·`step` 직렬 실행(B3), 피드백 신선도(B9), TELEOP 중 피드백 상실 → 주행 거부·선점(B7), STOP 이 미확정 AUTO 전이를 남기지 않음(B3 ②·④), TELEOP 진입은 정지 확인 뒤(B5), 허가의 센서 침묵·증거 나이 입력(B8), 구동 지연·프로세스 정지·다중 인스턴스·적재 제동은 경계 밖 전제로 명시(B13). 반영 안 함: 무장 뒤 첫 명령 제한 시간 — 첫 명령도 회신 만료(0.3 s)를 거치므로 묵은 명령이 될 수 없다.
- **② Codex 적대적 검토 1회차(P1 7·P2 4·P3 2, 모두 반영).** PAUSED 정지가 근접 추적 중 미션 `pause` 거절에 막힘 → 미션에 `controlled` 를 넘기는 불변식 + 근접 중 PAUSED 거절(B5). 주행 금지가 slew 전 목표만 0 → 최종 상한 0(B7·B8). 반복 STOP 이 세대를 안 올림 → 올림(B3 0, 초안 코드 수정). 보류 미션 요청·AUTO 예약이 경계를 넘음 → 한 틱 안 확정(B3). 정지 확인이 이양·제출에 없고 정지 실패 처리 없음 → B5. 조향 전환 구간 의무 누락 → 곡률 변화율 제한 + 원호 다발(B8), 출력 경계(B8). `FrameLedger` 가 지운 번호를 다시 받음 → 거부(B10, 초안 코드 수정). `tracker_built` 가 근접 종료 뒤에도 참 → `near.active`(B7). TELEOP 을 떠나는 경로에 `hold_off` 누락 → 모든 경로(B5). PAUSED 감속 경로 없음 → 코어 후보 + 적용 값 되쓰기(B11). SLAM 유무를 알 수 없음 → `frame.slam`(B7). P3: 육안·영상 상한 방향, 검사 속도 규칙 정정(3절·B8).
- **② Codex 2회차(P1 4·P2 2·P3 1, 모두 반영).** 제출·재개 주기의 `none` 예외가 아직 `running` 인 미션을 풀어 줌(같은 배치의 PAUSED 요청 뒤 제출) → 생명주기 상태가 그 요청을 받을 수 있을 때만 `none`(B5). TELEOP 구동 출력까지 `controlled` 로 적음 → 구동 `none`·미션 `controlled` 로 나눔(B5 표). 되쓰기 대상에 SLAM 정지 판정기의 `last_command` 누락 → 추가, M-c 경계 한정(B11). 검사 속도에서 직전 적용 명령 누락 → 셋의 최댓값(B8). 수락 기록만 보고 AUTO 확정 → 틱 뒤 미션 상태로 판정, 이후 재개 실패도 PAUSED 복귀(B5). 조향 따라잡기를 slew 뒤로 옮김 → 코어와 같은 속도 사슬 함수를 공유(B8). P3: 첫 상한 0.108 의 조건(3절). 반례 두 건(반복 STOP, 지운 프레임 재등록)은 닫힘 확인.
