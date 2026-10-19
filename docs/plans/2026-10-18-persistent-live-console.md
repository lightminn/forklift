# 상시 운용 콘솔 — Isaac 실시간 실행기를 계속 띄워 두는 지휘 콘솔로 (계획, 4판)

작성 2026-10-18. 브랜치 `fix/live-submit-clock-video`(작업 트리 `forklift-wt-console`, `origin/main` 1a26426 기반). 관련 문서:
- [운용 콘솔 설계](../design/2026-10-13-operator-console.md)
- [console/1 계약 초안](../interfaces/2026-10-13-operator-console-contract-draft.md)
- [C3 검증](../validation/2026-10-17-c3-console-isaac.md)
- [실시간화 계획](2026-10-17-realtime-sim.md) — D8·D9 는 메인 세션 담당

판마다 바뀐 것은 6절 끝에 있다. 요약하면 이렇다:
- Codex 적대적 검토의 P1 5개와 P2 6개를 반영했다.
- agy 독립 검토와 메인 세션의 D9 의견을 반영했다.
- 미션 종료 절차, 준비 뒤 채택, 실패 분류, 장면 손상 래치, 복구 상태 기계, 적재(cargo) 상태, 감독 루프의 epoch 소유를 새로 정했다.

## 0. 왜 하는가

사용자 지시(2026-10-18, 시연 중):

> 유지하는 방향으로 가고, 이건 기본적으로 한번 쓰고 버리는 물건이 아니라 계속 띄워 두고 통제 지휘하는 콘솔인 걸 생각하셈. codex 검토 받고 가능한 모든 유즈케이스에 대해서 테스트 해봐. 변경해야 할 사항들은 지금까지 내가 지적했던 것들 포함해서 계획서로 쓰고 교차검토한 다음에 구현해.

지금의 `run_transport.py --live` 는 배치 실행기에 콘솔을 붙인 것이다. 프로세스는 다음 경우에 끝난다:
- 미션 하나를 끝낼 때: `complete` 에서 루프를 빠져나온다(R6075).
- 미션 안의 `require` 하나가 실패할 때: 예외가 그대로 `main()` 까지 올라간다.
- 실행 시간 상한에 닿을 때: 루프 상한이 `--max-sim-seconds` 다(R5602).

콘솔은 그때마다 '통신 끊김'을 보인다. 이 계획은 **실행(세션)과 미션을 떼어 놓는다.** 세션은 운용자가 끝낼 때까지 살아 있고, 미션은 그 안에서 몇 번이든 지시·완료·실패·취소된다.

범위 밖(바꾸지 않는 것):
- **배치 실행**(`--live` 없음)의 동작. 재생 게이트(기준 기록 29건 완전 재생)는 바뀐 커밋마다 29/29 이어야 한다. 단, 이 게이트는 배치 회귀만 증명한다. 실시간 쪽 증거는 4절의 시험이 따로 낸다.
- 실차 게이트웨이(C2b)와 물리 정지 수단(C4).

## 1. 지금까지의 지적과 처리 상태

| # | 사용자 지적(2026-10-18 시연) | 원인 | 처리 |
|---|---|---|---|
| 1 | 자동 작업을 내리자 바로 죽음 | 관찰 단계 시간을 sim 0 부터 셈 | 완료 `8b11e37` |
| 2 | 카메라 아래 1/3 이 검음 | 영상이 인식 카메라(near 1.0 m) 그대로 | 완료 `8b11e37`·`7207b87`. 배속 하락은 D12 |
| 3 | 맞춤 보기 안 됨, 지도 강제 복귀, 경로가 안 보임 | 격자 범위가 바뀔 때마다 다시 맞춤; 실행기가 `plan` 을 안 보냄 | 완료 `3818be2` |
| 4 | 왜 후진을 선호하나 | 복귀 목표 방향이 출발 자세 그대로; 첫 해석 경로를 비교 없이 채택; 전진 선호 옵션 꺼짐 | D11 |
| 5 | 운반 한 사이클 끝나면 잡이 죽음 | `complete` 에서 루프 종료 | D1 |
| 6 | 주변에 아무것도 없는데 `drive_forbidden` | 배속이 페이싱 기준 근처에서 오르내림; 화면은 허가 코드만 보임 | 사유 표시 완료 `ed3497f`. 배속은 D12 |
| 7 | 수동 조종 뒤 자동으로 바꾸면 팔레트까지 가야 함 | 첫 구간이 출발 자세에서 미리 계획됨 | 임시 처리 `7610ed5`·`2166aa3`. D2 가 대체 |
| 8 | 취소하자 죽음 | 취소가 루프를 끝냄 | 완료 `4f6fb5b`. 취소 뒤 상태 정리는 D1·D4 |
| 9 | 팔레트에 박자 죽음 | 수동 조종 허가가 팔레트를 못 봄; 평가용 겹침 검사가 실행을 끝냄 | D10·D6 |

조사 중 발견한 내 회귀: `7610ed5` 가 철수 추적 인증 검사(계획 v10 D5 (4))를 재계획 분기 안으로 넣어 꺼 놓았다. `cb4a79a` 에서 되돌렸고, AST 시험으로 고정했다.

## 2. 원칙

- **P1 세션과 미션 분리.** 미션의 끝은 세션을 끝내지 않는다. 세션이 끝나는 경우는 다섯 가지다:
  - 운용자의 세션 종료
  - 장면 초기화
  - 잡 시간 한도
  - 세션 실패(D6 분류표의 세션 실패 사유)
  - 감독 루프의 재시작 상한
- **P2 실패는 결과로 남긴다.** 미션 일 중의 복구 가능한 실패는 그 미션의 결과(`failed` + 사유)로 남긴다. 지게차는 세워 두고 세션은 계속한다. 결과는 다음 제출 전까지, 그리고 재접속한 페이지에서도 조회된다.
- **P3 다음 미션은 지금 상태에서.** 새 미션은 지게차의 지금 자세와 팔레트의 지금 자리에서 계획한다. 이전 미션의 경로·카운터·허가·도킹 상태를 물려받지 않는다.
- **P4 준비가 성공해야 바꾼다.** 새 미션 준비가 실패하면 세션 상태는 준비 전과 같다.
- **P5 해제는 API 로.** 작업자, 캡처 세션, SLAM hold, 관절 gain, 검사기 콜백은 그 소유자의 정리 API 로 닫는다. 내부 dict 를 지우는 것으로 대신하지 않는다.
- **P6 배치 불변.** 모든 변경은 `--live` 에서만 동작한다. 공통 코드를 함수로 뽑으면 재생 게이트로 확인한다.

## 3. 설계

### D0 상태 소유권 표

구현의 첫 산출물은 `docs/design/2026-10-18-live-mission-state.md` 다. 표의 각 행은 상태 하나다. 열은 다음과 같다:
- 소유: 세션 / 미션 / 화물 / 장면
- 위치: 파일·줄
- 미션 종료 처리: 유지 / API 정리 / 제자리 초기화 / 시작값으로
- 미션 시작 처리
- 근거

입력은 세 가지다:
- 이번 조사의 1차 목록(시나리오, 관찰 후보, 경로·추적기, `ms`, `near`, `pocket`, `slam` 도킹, 장애물 실행 상태와 단계별 예산, 상태 기록, 커스프 감시)
- Codex 가 보탠 목록: `layer.exempt`·`depth_support`·철수 회랑·허가 캐시, SLAM 실제 hold·`release_wait_from`·hold 일화, `observe_replan_leg`, 관찰·근접 기록, `path_blocked_ticks`·`silence_trace`·`permission_cap`·`last_permission`, `wheel_lock`·승강 목표·조향 배열, D9 캡처·관찰 상태
- 메인 세션이 알려 준 유지 항목: `obstacle.applied`·`version`, `slam.tracker`·`async_link`·`scan_id`, `state["slam_async"]`

다음은 **세션 소유이고 되돌리지 않는다**:
- `core.commands`: 중재기가 객체를 직접 쥐고 있다.
- 대상 목록의 revision
- `mission_gen`
- 작업자 job 번호
- SLAM 보정 버전과 hold 일화 번호: `holds` 는 `hold_count()` 의 근거다.
- 장애물 기억(memory)과 알려진 사각형: 관측 근거·보정·시간을 가지므로 행마다 처리를 정한다.

표에 없는 상태가 미션 사이에서 값이 바뀐 채 남으면 결함으로 본다. 시험(U21)이 두 번째 미션 시작 직후의 미션 소유 상태를 첫 미션 시작 직후와 비교한다.

### D1 미션 종료 절차 (실시간)

`complete`, 복구 가능한 실패, 취소 확정은 모두 같은 절차 `end_mission(t, outcome, reason)` 을 거친다. 순서는 다음과 같다.

1. **정지와 물리 상태 고정.** 바퀴 명령 0, 승강 유지 목표를 확정하고, 이 틱의 적용 명령을 기록한다. 이어서 **물리 상태(D5 의 포켓 점유·적재 상태)와 복구 근거(포크 축, 삽입 길이, 팔레트 추정)를 스냅숏으로 고정**한다. 아래 단계는 이 스냅숏으로 분기한다. 포켓 검사기를 닫았다고 포켓 밖으로 보지 않는다.
2. **비동기 작업 폐기.**
   - 새 `core.abandon_jobs(t)` 를 만든다. 대기 계획을 버리고, 열린 작업 시계 대기를 닫는다.
   - `_drop` 의 release 재계획 예약 부작용(`release_replan_needed=True`, C4363)을 되돌린다.
   - 이어서 `capture_abandon()`, `observation_live=None`, `capture_acquired_s=None`.
   - 메인 D9 머지 뒤 실제 API 에 맞춘다.
3. **보호 종료.**
   - 포켓 검사가 열려 있으면 `insert` 이탈 전이와 같은 닫기를 부른다. `layer.exempt`·`depth_support` 해제와 허가 캐시 무효화가 여기서 일어난다.
   - 철수 회랑(`end_withdrawal`)을 닫는다.
4. **SLAM hold.**
   - tracker 가 holding 이고 지게차가 서 있고 **포크 관계**(D5)가 `free` 또는 `carrying` 이면 hold 를 푼다.
   - 이 **종료용 해제**는 보정 적용·필요한 재표현·재투영·회계만 한다. 계획 요청, 경로·추적기 설치, 재관찰 예약은 하지 않는다(사후조건, 시험으로 고정).
   - 기존 `release_and_stand`(관찰 경로 설치·`retry_stand` 설정)와 `slam_release`(다음 구간 재계획 요청)는 이 조건을 어기므로 그대로 부르지 않는다. 공통 부분을 떼어 내는 구현은 D0·API 검토에서 정한다.
   - 포크 관계가 `constrained`·`unknown` 이면 hold 를 유지하고 D4 의 복구로 간다.
5. **관절.** `wheel_lock` 이 걸려 있으면 저장된 gain 으로 정상 unlock 한다.
5a. **열린 구간·감시 닫기.** 기록보다 먼저 하고, 상태는 바꾸지 않는다(D0 9절).
   - 새 `commands.close_open(t)`(열린 pause·wait 를 닫는다. 지금의 `finish()` 는 terminal 전환까지 해서 쓰지 않는다)
   - TrackerBook 마지막 drain
   - 열린 커스프 hold·watch 닫기
   - 열린 근접 보정·정지 사건 닫기
6. **결과 판정.**
   - `complete` 이면 지금 루프 밖에 있는 배달 판정(R6438–6474: 목적지 0.08 m, 접지, 포크 이탈)을 미션별 함수로 뽑아 여기서 부른다.
   - 기준은 그 미션의 `truth_destination` 이다.
   - 통과하면 `done`, 아니면 `failed` 다. 사유는 판정 메시지다.
   - 배치는 같은 함수를 지금 자리에서 부른다(순수 이동).
7. **기록.**
   - 그 미션의 기록(result·paths·transitions·async_plans·observation_drops·관찰 시도)을 `missions/<id>/` 에 쓴다.
   - **소유권은 미션 진행 중에 조금씩 넘긴다(D8 조각).** 감사 기록은 쌓이는 동안 크기·시간 단위(예: 1 MB 또는 10 s)로 불변 조각이 된다. 조각 하나를 확정하는 제어 스레드 비용은 그 조각 크기로 묶인다. 쓰기 스레드는 미션 ID 가 붙은 불변 바이트 조각만 받는다. 얕은 복사로 넘기지 않는다.
   - 종료 틱에서는 남은 조각 하나와 크기가 묶인 결과·manifest(조각 목록, 판정, 사유)만 직렬화한다. 상한: 결과·manifest 64 KB, 종료 틱의 기록 비용 20 ms(벽시계). 넘으면 나머지는 다음 틱들로 나누어 넘긴다.
   - 쓰기 큐 상한은 바이트로 둔다(예: 64 MB). 가득 차면 제어 스레드는 기다리지 않고, 버린 조각을 결과 기록부에 누락으로 표시한다. 쓰기 실패도 결과 기록부에 남긴다.
   - 세션 종료 때 큐를 비울 때까지 기다린다(상한 시간). 남은 것은 누락으로 기록한다.
8. **미션 소유 상태 초기화.** D0 표대로.
9. **결과 확정.**
   - 6단계 판정 직후, 기록 전에 `commands.terminate(outcome, t, reason)` 으로 terminal 이 된다(outcome 은 `done`·`failed`·`cancelled`·`recovery_required`). 여기서는 공개만 한다. 판정이 실패한 `complete` 는 commands·기록부·발행 결과가 모두 `failed` 다.
   - **결과 기록부**(세션 소유)에 `{mission_id, seq, outcome, reason, started_s, ended_s, pallet_id, destination_id}` 를 더한다.
   - 상태 메시지의 `mission.last_result`, 그리고 targets 처럼 저장소에 두는 `missions` 메시지로 내보낸다. 재접속한 페이지도 받는다.
   - `apply()` 가 다음 틱에 슬롯을 `idle` 로 비워도(C111) 결과는 남는다.

세션 종료 때는 진행 중 미션이 있으면 `end_mission(…, "failed", "session_end")` 을 먼저 부른다.

### D2 미션 준비와 채택 (실시간)

실시간 실행에서 모든 제출(첫 제출 포함)은 **준비 → 채택** 을 거친다. 배치는 지금 설정 경로 그대로다.

**받는 조건**
- 슬롯이 `idle`
- 정지 확인
- 장면 손상 없음(D6)
- 위치 추정 tracking
- 대상 검사(D3)
- 적재 상태면 적재 제출(D5)로 간다.

**준비**
- 받아들인 제출은 `preparing` 이 된다. 계약에 상태 하나를 추가한다.
- 작업자에 `mission_start` 작업을 보낸다. 메인 D8 작업자에 작업 종류를 하나 추가하는 것으로, 메인과 합의가 필요하다.
- 입력:
  - 시작 자세: 이 틱의 SLAM 응답을 적용하고 격자를 재투영한 **뒤**의 제어 자세(D9 순서)
  - 팔레트 사전값(자세와 방향)
  - 목적지
  - 장면 소품과 경계
- 출력: 관찰 후보와 관찰 경로.
- 계획 중에도 루프는 돌고 STOP·heartbeat·상태 발행이 계속된다.

**관찰 후보 일반화**
- 지금의 후보 생성은 절대 좌표 격자를 돌고 팔레트 방향을 0 근처로 본다(`observation_viewpoints.py` 140·187).
- 이를 **팔레트 좌표계에서 만들고 팔레트 자세로 변환**하는 함수로 바꾼다.
- 첫 미션의 원래 자리에서는 지금 후보와 같은 결과가 나오는지 CPU 시험으로 확인한다. 다르면 차이를 기록하고, 배치는 옛 함수를 쓴다.

**이동 경계와 사전 영역 분리**
- 관찰 경로는 홀 경계에서 계획한다. 시작점이 픽업 구역 밖이어도 된다(U5).
- 팔레트 주변 사전 영역(`pickup_zone`)과 픽업 구역 제한(`pickup_bounds`)은 팔레트 자리를 기준으로 다시 둔다.
- `pickup_bounds` 는 접근·삽입 단계(plan_transport near_bounds)에만 쓴다.

**채택**
- 다음이 모두 준비 시점과 같으면 D0 표의 '미션 시작 처리'를 실행하고 `running` 이 된다:
  - 미션 세대
  - 정지 상태
  - 보정 버전
  - 자세(2 cm, 1°)
  - 대상 revision
- 하나라도 바뀌었으면 한 번 다시 준비한다.
- 준비 실패, 다시 실패, 취소, STOP 이면 결과 기록부에 `failed: prepare:<사유>` 또는 `cancelled` 를 남긴다. 세션 상태는 바꾸지 않는다(P4).

**시작 처리에서 하는 일**
- 미션 시나리오: 소품과 경계는 같고, `start_rear`·`pickup`·`destination` 을 바꾼다.
- 참값(시뮬 전용):
  - `truth_destination` = **고른 목적지의 고정된 참값**. 평가(D1 6단계)와 도킹 참조 스캔을 가르치는 위치(R5479)에 쓰인다.
  - `initial_pallet`·`pickup_obstacle` = 지금 집을 팔레트의 참값 자리.
  - 목적지의 세 값 대응: 계획용 사전값 = 대상 목록의 목적지 자세, 도킹 참조 = 그 목적지에서 다시 뜬 참조 스캔, 평가 참값 = `truth_destination`.
- 경로·추적기:
  - `paths` 를 비우고 관찰 경로를 넣는다.
  - `TrackerBook` 의 옛 추적기는 drain 하고 키를 지운다.
- 미션 상태:
  - `ms` 초기화: 단계 observe, `phase_started`=t, 카운터 0.
  - `observe_replans` 와 `observe_replan_leg` 를 함께 초기화한다.
- 장애물 층의 알려진 팔레트: 사전 영역으로 둔다.
- `options.return_to_pose`: 7절의 복귀 정책을 따른다.

`7610ed5`·`2166aa3` 의 임시 처리(첫 구간 재계획, `not_at_start`)는 D2 가 대체하므로 지운다.

### D3 대상 목록

- **배달된 팔레트.**
  - 내려놓은 자리에서 다시 `available` 이 된다.
  - 자리는 지금처럼 목적지 위치(도킹 결과)를 사전값으로 쓰고, 인식이 실제 자세를 다시 찾는다.
  - 화면용 '배달됨' 표시는 별도 필드로 둔다.
  - 판정은 매 미션의 참값 배달 오차(D1 6단계)로 한다. 사전값을 실제 위치처럼 쓰지 않는다.
- **목적지.** 시나리오 목적지 `D-1` 과 원래 픽업 자리 `D-0` 을 둔다.
- **제출 거절 사유.**
  - `already_there`: 팔레트가 이미 그 목적지에 있음
  - `destination_occupied`: 목적지를 다른 팔레트가 차지함
  - `loaded`: 적재 중에 다른 팔레트를 지정함
  - `scene_damaged`: 장면 손상
  - `localization_lost`: 위치 추정 tracking 아님
- **단순 이동 미션**(agy 지적)은 이번 범위에서 뺀다. 미션 종류별로 제출 입력·준비 출력·장애물 구성(`plan_return_leg` 은 배달 팔레트를 장애물로 넣는다)·완료 판정을 따로 정해야 해서 별도 계획으로 한다. 그동안 빈 차 이동은 수동 조종으로 한다.

### D4 취소와 복구

지금 `cancelling` 은 미션 일을 금지하고, 정지가 확인되면 곧바로 `cancelled` 가 된다(C2741). 그래서 기존 단계로 정리 동작을 실행할 수 없다. 취소 의도와 물리 복구를 나눈다. 취소가 확정되면 물리 상태에 따라 아래 표를 따른다.

| 물리 상태 | 판단 근거 | 복구 |
|---|---|---|
| 포켓 밖, 빈 포크 | 포켓 점유 `outside`, 적재 `none` | 없음. `cancelled` |
| 부분 삽입(포켓 안, 안 들림) | 실제 포크 자세와 팔레트 추정 사이 축·삽입 길이 | 그 축을 따라 삽입 길이 + 여유만큼 직선 후진. 허가가 후방을 본다. 성공하면 `cancelled` |
| 들리는 중·들린 상태(픽업 자리) | 승강 높이, 화물 | 같은 자리에 승강 ramp 로 내림 → 접지 확인 → 부분 삽입 복구 |
| 추출 중 | 포크 관계 | `carrying` 이면 운반과 같음. 아직 `constrained`(팔레트가 지지대·접지에 걸림)면 정지, `recovery_required` |
| 운반(적재) | 포크 관계 `carrying` | 그 자리에 정지, 화물 유지. SLAM hold 해제(D1 4단계). 결과 `cancelled(cargo)`. 다음 제출은 적재 제출(D5) |
| 내리는 중(목적지) | 승강·접지 | 내림을 끝내고 접지 확인 → 후진 복구 |
| 철수 중 | 포켓 점유로 분기(단계 이름으로 분기하지 않음) | `inside` 이면 부분 삽입 복구, `outside` 이면 `cancelled` |
| 정착·복귀 | 포켓 점유 `outside` | 정지. `cancelled` |
| 판단 불가 | 포켓 점유·적재 `unknown` | 정지, `recovery_required` |

- 복구는 새 단계 `recover_lower` 와 `recover_withdraw` 로 실행한다. 이 동안 미션 슬롯은 `recovering` 이다.
- 복구 경로는 기존 `withdraw` 경로를 재사용하지 않는다. 지금의 포크 축과 측정한 삽입 길이로 새로 만든다. 인증 철수(C213)의 `pallet_rel` 전제는 부분 삽입에서 성립하지 않는다.
- 다음 중 하나면 결과 `recovery_required` 가 된다:
  - 허가 거절
  - 시간 초과
  - 접지 실패
  - SLAM hold 를 풀 수 없음
- `recovery_required` 에서는 자동 제출을 받지 않는다. 운용자의 수동 복구만 받는다(D14, 정책은 중재기 쪽 합의).
- 복구 중 취소나 STOP 은 정지로만 처리한다. 다시 시작하면 같은 표에서 다시 판단한다.
- `refresh_targets` 는 복구 중 팔레트를 목적지에 내려놓은 것으로 기록하면 안 된다(Codex 반례). 복구로 내린 팔레트의 자리는 그 자리의 추정값이다.

### D5 적재(cargo) 상태와 적재 제출

- **물리 상태를 단계와 검사기 수명에서 떼어 놓는다.** 기존 `insert → lift` 전이는 포크가 아직 팔레트 안인데 포켓 검사기를 닫는다(C268·C413). 그래서 검사기 상태로 포켓 점유를 판단하지 않는다.
  - **포켓 점유** `inside / outside / unknown`: 포크 끝·캐리지 외곽과 팔레트 추정(인식값 또는 알려진 사각형)의 기하로 매 틱 계산한다. 추정이 없거나 불확실성이 크면 `unknown`.
  - **적재** `none / acquiring / confirmed / releasing / unknown`:
    - 들기 시작에서 `acquiring`
    - 들기 판정 통과에서 `confirmed`
    - 내리기 시작에서 `releasing`
    - 접지와 포켓 이탈(`outside`)을 확인하면 `none`
  - 세션 소유의 `cargo = {pallet_id, pallet_rel, lift_hold_m, evidence, state}`.
  - `acquiring`·`releasing`·`unknown` 에서는 보수적인 외곽(빈 차 외곽과 적재 외곽의 합집합)을 쓴다.
  - **포크 관계**(포켓 점유와 적재를 함께 본 값) `free / constrained / carrying / unknown`:
    - `free`: 포켓 점유 `outside`.
    - `carrying`: 점유 `inside`, 적재 `confirmed`, 팔레트가 접지에서 떨어져 있고(승강 높이와 팔레트 추정 높이) 주변 구조물과 겹치지 않음. 화물이 차와 함께 움직이는 상태다.
    - `constrained`: 점유 `inside` 이고 `carrying` 이 아님. 접지된 팔레트나 지지대에 걸린 삽입이다.
    - `unknown`: 그 밖.
  - 종료용 hold 해제(D1 4단계)와 중재기 허가는 이 포크 관계로 판단한다. 포켓 점유 하나만으로 판단하지 않는다.
    - `carrying`: 적재 수동 조종(적재 외곽)
    - `constrained`: 복구 조종만(D14)
  - 실시간에서는 중재기의 `loaded`·`fork_in_pocket` 과 외곽 선택이 `ms.phase` 대신 이 상태와 포켓 상태를 읽는다(C2947·R5408).
- **적재 제출.**
  - 일반 D2 초기화를 거치지 않는다.
  - 준비 작업이 운반·도킹·내리기·철수·복귀 경로와 추적기를 모두 만든다(정상 계획의 후반부와 같은 구성).
  - 채택하면 `transport` 단계부터 시작한다.
  - 제출 검사와 화면은 `carried` 팔레트를 고를 수 있게 바꾼다(targets.py 107, ui.js 55).

### D6 실패 처리

**분류표.** `require` 사유마다 셋 중 하나로 정한다:
- 미션 실패(복구 가능)
- 세션 실패
- 장면 손상

분류표는 코드에 둔다. AST 시험이 run_transport·core·live 의 모든 `require` 사유 문자열이 분류표에 있는지 확인한다. 기본값은 세션 실패다. 분류되지 않은 새 사유는 세션을 끝내고 시험이 실패한다. 다음은 세션 실패다:
- 모델·보정·설정 계약 위반: `pocket_check_without_capture`, `near_field_contract` 등
- 비유한 상태
- 메인 D9 의 `SessionFailure`: `slam_link_failed`, `slam_prewarm_timeout`. 53eba4c 에 들어갔다.
- `LocalizationStale`

**잡는 범위는 두 곳만이다.**
1. 틱의 미션 작업 구간: follower 부터 명령 조립까지.
2. 물리 스텝 뒤의 캡처 콜백: 메인 D9 의 `capture_stepped`.

두 곳 모두 예외를 결과로 바꾸고, 공통 꼬리를 정확히 한 번 실행한다. 공통 꼬리:
- 이 틱 명령을 0 으로 확정한다. 승강은 유지한다. 승강 목표가 이미 바뀌었으면 현재 높이로 되돌린다.
- 요청 응답(`tick_end`)
- 상태 발행
- 물리 스텝

스텝 수행 여부를 플래그로 추적해 두 번 진행하지 않는다. 결과는 D1 의 `end_mission(…, "failed", 사유)` 로 넘긴다. 루프 전체를 감싸는 방식은 쓰지 않는다. 경계는 메인 D9 머지 뒤 실제 코드에서 정한다.

**장면 손상은 래치한다.**
- 실시간에서 참값 검사(조사 §4 표: 겹침, 기울어짐, 팔레트 떨어짐·밀림·미끄러짐, 포크 접촉)는 처음 한 번 걸리면 `scene.damaged = {reason, t}` 로 래치한다. 진행 중 미션은 실패로 끝낸다.
- 그 뒤로는 같은 검사로 예외를 내지 않는다. 기록만 하고 세션 서비스는 계속된다.
- 손상 상태에서는 다음과 같다:
  - 자동 제출 거절(`scene_damaged`)
  - 수동 조종은 허용
  - 장면 초기화 버튼 표시
- 배치는 지금처럼 끝난다.

### D7 세션 수명

- 실시간에서는 `--max-sim-seconds` 를 루프 상한으로 쓰지 않는다. 잡 시간 한도가 상한이다.
- 세션 종료 요청 `session_end`: lease 보유자이고 정지가 확인된 상태여야 한다.
- 고정 포트 `--live-port`. 빠른 재바인드는 Isaac Python 에서 따로 확인한다(V 항목).
- **요청 원장 상한.** 중재기 요청 원장(4,096건, 회수 없음)은 상시 세션에서 결국 다 차서, 새 요청이 모두 `busy` 가 된다. 결과를 지우면 옛 요청을 구별할 수 없으므로 계약을 바꾼다:
  - **요청 토큰**: `hello` 때 서버가 다시 쓰지 않는 단조 번호의 토큰을 준다. 브라우저는 요청마다 이 토큰과 토큰별 **단조 순번 `seq`** 를 붙인다.
  - 서버는 살아 있는 토큰마다 결과를 지운 최고 순번(워터마크)을 둔다. `seq ≤ 워터마크` 인 요청은 결과 없이 `stale` 로 거절하고, 다시 실행하지 않는다.
  - 원장은 재전송 창(예: 60 s)을 넘은 결과를 순번 순으로 지우고 워터마크를 올린다.
  - lease 가 없고 일정 시간(예: 10 분) 조용한 토큰은 표에서 지운다. **모르는 토큰**(지워졌거나 위조)의 요청은 모두 `stale` 이다. 브라우저는 `stale` 을 받으면 `hello` 로 새 토큰을 받는다. 토큰 번호를 다시 쓰지 않으므로, 지운 토큰의 옛 요청은 결코 새 요청으로 보이지 않는다. 메모리는 살아 있는 토큰 수로 묶인다.
  - lease 재획득(새로고침 뒤)은 지금처럼 `client_id` 로 하고, 요청 중복 판정은 토큰으로 한다.
  - 시험:
    - 회수 뒤 재전송한 `lease_request` 가 새 lease 를 받지 않는다.
    - 지운 토큰의 재전송은 `stale` 이다.
    - 회수가 계속돼도 원장 크기 상한을 지킨다.
  - 중재기·프로토콜·브라우저 변경(메인 합의).
- 작업자 폐기 ID 목록은 응답이 더 올 수 없는 ID(응답 수신·작업자 재시작)를 회수한다.

### D8 장시간 실행의 자원

- **두 종류로 나눈다.**
  - 제어에 필요한 짧은 이력: 고정 크기 링 버퍼.
  - 감사 기록: 시간·크기 단위 조각으로 디스크에 쓰고 메모리에서 비운다.
  - 유휴 중에도 같은 규칙이다(U1).
- **대상.**
  - `state["samples"]`, `slam["control"]`, `slam["records"]`
  - `slam_log` 의 관절·자세·스캔 배열
  - `obstacle["scans"]`·`plans`, `state["transitions"]`
  - 근접 기록, 겹침 격자
  - LiveRuntime 의 카메라 지연 목록
  - `commands.pauses`·`waits`: 미션 종료 때 미션 기록으로 옮긴다. `work_elapsed` 정렬 비용도 함께 줄어든다.
- **SLAM 브리지.** 지도 배열과 기록을 모두 메모리에 쥔다(`isaac_slam_bridge.py` 150·274). 같은 규칙을 적용한다. ROS 패키지 변경이다.
- **상한**(초안, 측정 뒤 확정): 실행기 RSS 증가 < 200 MB/시간, VRAM 증가 0(±100 MB), 세션 디스크 < 5 GB/8시간, `live_log` 회전 200 MB × 3.
- 기록 쓰기는 별도 스레드다.
- **GC 동결.** main 1e53cbe 는 첫 제어 틱 전에 `gc.freeze()` 를 한 번 한다. 상시 세션에서는 미션마다 남는 객체가 동결 밖에 쌓인다. 미션 종료(D1) 뒤 다시 동결할지는 U1·U2 장시간 시험에서 2세대 수집 시간을 재고 정한다.

### D9 감독 루프와 장면 초기화

잡 스크립트 `deploy/console/isaac_live.sbatch` 가 epoch 마다 다음을 소유한다:
- 출력 디렉터리 `epoch_<n>/`. 지금 `--output` 은 `exist_ok=False` 다(R6733).
- SLAM 브리지 컨테이너. 브리지는 연결을 한 번만 받으므로(`accept`, 225) epoch 마다 새로 띄운다.
- 실행기 하나.
- 포트 파일 상태.

종료 코드로 다음 행동을 정한다:

| 종료 | 감독 루프의 행동 |
|---|---|
| 0 세션 종료 | 끝 |
| 3 장면 초기화 | 즉시 새 epoch |
| 그 밖(세션 실패, 충돌) | backoff 후 새 epoch |

같은 원인으로 연속 3회 실패하면 멈춘다. 지금은 성공 여부(`state["success"]`)가 종료 코드를 정한다(R6876). 실시간에서는 세션 종료 사유로 바꾼다.

새 epoch 의 규칙:
- SLAM 예열을 마친 뒤에야 포트 파일을 `running` 으로 쓴다.
- 옛 lease·arm 은 복원하지 않는다(C3 의 epoch 처리).
- 강제 종료로 포트 파일이 `running` 으로 남은 경우, 감독 루프가 `crashed` 로 고친다.

장면 초기화 `sim_reset` 은 시뮬 전용 기능으로, welcome `capabilities.sim_reset` 에 싣는다. 실행기를 정상 종료(코드 3)하면 감독 루프가 새 장면으로 다시 띄운다. 재기동 시간은 측정해서 화면에 '초기화 중(예상 n 초)'으로 보인다.

### D10 수동 조종 허가가 팔레트를 본다

- **팔레트를 장애물로 더한다.** 수동 조종 원호 허가에 대상 목록 팔레트의 사각형을 장애물로 넣는다.
  - 사전값은 사전 불확실성만큼 키운다.
  - 인식값은 인식 오차만큼 키우고, 보정 버전을 따라 재투영한다.
  - 대상 목록의 `Pose` 에 근거 종류(prior/recognised)와 불확실성을 붙인다(계약 확장).
- **실은 팔레트는 적재 외곽에 들어간다.**
- **캐시를 무효화한다.** 허가 캐시 키에 대상 revision 과 적재 외곽을 더한다. BundlePermit 의 설치된 원호 검사도 이때 무효화한다.
- **실차.** 같은 규칙이 필요하다. 중재기 변경이므로 사용자 확인(메인 의견)과 메인 합의가 필요하다.

### D11 복귀 전진 선호

- 지금은 `return_turnaround` 이면 비동기 작업자를 우회해 루프 안에서 동기로 최대 20 s 계획한다(C493–535).
- 먼저 turnaround 계획을 작업자로 옮기고, 그 뒤 실시간 기본값으로 켠다.
- 판정(U17): 적용 가능한 장면(집이 뒤에 있는 배치)에서 복귀 경로의 후진 길이가 기존 대비 줄고, 복귀가 성공해야 한다.

### D12 실시간 배속

영상 카메라 렌더를 줄이는 방법 세 가지를 같은 스냅숏·같은 부하에서 각각 재고, 하나를 고른다:
- 격번 렌더(5 Hz sim)
- 낮은 해상도
- 구독자가 없으면 렌더 끄기

시험 쪽 조건(agy 지적): 장시간 시험(U16)은 영상을 구독한 채로 잰다. 그렇지 않으면 렌더가 꺼져 누수를 못 본다.

배속은 다음과 같이 정한다:
- 기본 배속은 측정값이 페이싱 기준(선언의 90 %)보다 10 % 이상 위에 있게 정한다.
- 화면에 실측 배속과 기준을 늘 보인다.

### D13 화면

- 미션 결과 기록부(지시·결과·사유·시각)와 실패 사유의 한국어 설명
- 준비 중·복구 중·`recovery_required` 표시
- 장면 손상 배너와 장면 초기화·세션 종료 버튼(확인 단계 포함)
- 목적지 `D-0`, 팔레트의 배달됨/사용 가능/적재 표시


### D14 수동 승강과 복구용 조종

- 중재기가 `lift_dir` 을 실제로 구동한다(상한·속도 제한).
- 지금 정책은 포켓 안에서 주행과 승강을, 적재 중에는 승강을 금지한다(`teleop_policy.py` 33). 그래서 **복구 전용 규칙**이 따로 필요하다:
  - 포켓 안이고 안 들렸을 때: 포크 축을 따른 직선 후진만, 저속.
  - 들린 상태에서 서 있을 때: 내리기만.
- 중재기와 계약 변경이므로 사용자 확인과 메인 합의가 필요하다.

### D11 구현 설계 (8단계, 2026-10-19, D14 fb07e29 뒤; 5판 — Codex 1–4회차·agy 반영)

**지금.** `--return-turnaround` 이면 복귀 첫 계획(전진 방향 전환 후보 비교)이 작업자를 우회해 제어 루프 안에서 동기로 돈다. 우회 조건은 core `live_leg` 의 **두 곳**(작업자 정지 대기 분기, 작업자 요청 분기)이다. 바깥의 `pending_release` 대기 조건은 배치에도 필요하므로 그대로 둔다(Codex 1회차 P2-3: 셋 다 지우면 해제 대기 중에 방향 전환 없는 첫 복귀 계획을 소비한다). 우회 이유는 `return_turnaround_kwargs()` 의 `arc_check` 가 장애물 층을 호출 때 읽는 클로저라 피클할 수 없기 때문이다. 그동안 최대 20 s 루프가 멈춘다.

**설계.**
1. **순수 함수.** `ObstacleLayer.check_turnaround_arc` 의 본문(허가의 `check_path` 두 번)을 `forklift_core` 의 `turnaround_arc_reason(permission, snapshot, footprint, own, poses, *, current_pose)` 로 옮기고 층 메서드는 그것을 부른다(동작 불변). Codex 1회차 확인: `check_path` 는 넘긴 스냅숏과 `permission.config` 만 쓴다(`path_check`·`_driven_since_m`·내부 스냅숏을 바꿔도 결과 같음).
2. **고정 검사기.** `TurnaroundArcCheck`: 요청 시점의 허가·스냅숏 깊은 사본, 발자국 둘, `callback_spec`, `__call__(poses)`. 비용은 사본+피클 약 5 ms·1 MB(240×240 창, 복귀 계획 한 번에 한 번 — agy 3). 층의 `turnaround_arc_check()` 가 만든다.
3. **코어.** `return_turnaround_kwargs(frozen=False)`: frozen 이면 검사기. `live_leg` 의 작업자 분기 두 곳에서 return_turnaround 예외를 없애고 복귀 요청은 frozen 인자로. 해제 뒤 재투영 블록은 요청 전에 그대로. 배치 경로(작업자 없음)는 그대로(클로저, 재생 게이트).
4. **보정이 바뀐 답(Codex 1회차 P2-1; 2·3회차).** 작업자 답은 지금처럼 요청 때 보정과 **정확히 같아야** 채택한다. 경로를 보정 변화만큼 옮기는 것(2판)은 지도 좌표의 목표·계획 기억·SLAM 지도와 어긋나고(2회차), 경로 전체를 다시 검증하는 것(3판)은 계획기의 구간 검사(구간 중점의 이동량만큼 팽창)·tight 재시도의 여유와 같아야 하고 약 80 ms 가 들어(40 m·668 표본, 한 틱 8.3 ms) 한 틱에 들지 않는다(3회차). 실측으로는 이 경우가 드물다: 콘솔 Isaac 실행 42 개 미션 기록에서 `live_leg` 작업자 답 19 개가 모두 채택됐고, 보정 때문에 버려진 답은 관찰 계획 63 개 중 1 개였다(서 있는 동안 slam_toolbox 는 스캔을 건너뛰고 건너뛴 답은 보정을 바꾸지 않는다, `slam_pose.py` 191). (19 개는 지금 동기로 도는 turnaround 복귀를 포함하지 않아 간접 근거다 — 4회차 P3; 정확히는 브리지의 `KeyframeGate` 가 서 있는 동안 스캔 전송을 건너뛰고, 이미 보낸 스캔의 `processed` 답은 정지 뒤에도 보정을 바꿀 수 있다.) 그래서:
   - 복귀 turnaround 요청이 `correction` 으로 버려진 횟수는 **복귀 단계 전용 집계**로 센다(그 미션의 그 복귀 단계, 단계·미션이 바뀌면 0). 이 사유의 폐기는 지금의 `live_leg` 공통 폐기 한도(합계 네 번째에 실패, 4회차 P2)에 **넣지 않는다**. 다른 사유(미션·단계·모드·움직임·표류)는 지금처럼 공통 한도에 넣는다.
   - 그 집계가 **세 번**이 되면 그 복귀 단계는 끝날 때까지 지금의 동기 계획(제어 루프 안, 클로저 검사)으로 한다 — 그 사유로는 실패하지 않던 지금 동작이다. 동기 계획이 실패하면(`invalid_start`·`no_path`) 지금의 동기 재시도(1 s 마다, 다섯 번)를 따르고, 다시 작업자로 가지 않는다.
   - 폐기·대체 횟수를 기록한다. 대체 경로는 자연 발생을 기다리지 않고 시험 플래그 `--live-test-return-correction-drops N`(복귀 turnaround 답 처음 N 개를 `correction` 으로 버림)으로 강제해 잰다: 대체 진입, 대체 계획 동안의 제어 틱 간격, 그 동안의 STOP·취소 처리 지연.
5. **작업자의 커널(Codex 1회차 P2-2).** 라이브 본체는 `prefer_fast()`·`warm_up()` 을 부르지만 작업자는 새 인터프리터라 `reference` 커널로 돈다(후보 비교의 벽시계 마감에서 `skipped_time`·`late` 가 갈린다). 작업자를 띄울 때 환경에 본체의 커널(`FORKLIFT_KERNELS=<kernels.mode()>`)을 넘기고, 작업자는 `ready` 전에 `warm_up()` 한다(재시작도 같은 환경). 작업자 커널을 상태에 기록한다.
6. **라이브 기본값.** `--live` 이면 `--return-turnaround` 를 기본으로 켠다(`--no-return-turnaround` 로 끔). 배치 기본값은 그대로.
7. **agy 독립 검토(적히지 않은 전제) 처리.** 대기 중 차가 움직이면: 기존 `moving`·`drift` 검사가 버린다. 오래된 장애물: 실행 중 허가가 매 틱 다시 본다. 대기 중 취소·STOP·단계 전이: 기존 `mission`·`phase`·`paused` 폐기. 결정성: 배치 경로 불변(재생 게이트), 라이브는 원래 벽시계 의존. 후보 기각 뒤의 기본 후진: 같은 계획 호출 안의 비교라 따로 기다리지 않는다.

**시험.**
- CPU: 함수와 층 메서드가 같은 결과(실제 층, 막힘·짧음·점유·통과), 검사기 피클 왕복 뒤 같은 결과, 요청 뒤 층이 바뀌어도 검사기 불변; `live_leg` 이 복귀 turnaround 를 작업자로 보냄(payload 에 검사기·`forward_turnaround`), 해제 대기 중에는 요청하지 않음(배치·라이브 둘 다), 정지 대기; 보정이 바뀐 답은 지금처럼 폐기·재요청; 복귀 turnaround 의 `correction` 폐기는 전용 집계로 세고 공통 한도에 안 들어감(운반 폐기 1 + 복귀 폐기 3 에서도 실패하지 않음), 세 번이면 그 단계는 동기 계획, 동기 실패 뒤는 동기 재시도(작업자로 돌아가지 않음), 단계·미션이 바뀌면 집계 0, 다른 사유(미션·단계·모드·움직임)는 지금처럼 폐기, STOP·취소 뒤 늦은 답 폐기, tight 재시도; 작업자 환경의 커널 전달과 예열; 연속 미션에서 기본값.
- Isaac U17: 세 구성 — D11 이전(fb07e29) + `--return-turnaround`(동기), D11 + 켬(작업자), D11 + 끔. 그리고 D11 + 켬 + `--live-test-return-correction-drops 3`(대체 진입, 대체 동안 틱 간격, STOP 응답). 복귀 경로의 후진 길이, 복귀 성공, 복귀 계획 동안 제어 틱 간격(최대·p99).

### D14 구현 설계 (7단계, 2026-10-19, D4b f5dd4e8 뒤; 5판 — Codex 1–4회차·agy 반영)

**메인 합의(2026-10-19, forklift-87).** 방향 동의, 조건:
1. 승강 출력은 자율과 같은 꼴이다: 목표 위치(`ms.lift_command` 의 뜻) + 속도 상한 램프 + [0, 상한] 클립. 속도형 변환은 실차 어댑터가 맡는다(실차 접점 하나).
2. 텔레옵 승강 속도 기본 0.02 m/s, 설정값은 자율 램프 `lift_rate_mps`(0.06) 이하인지 검사한다. 위 한계는 차체 모형의 승강 한계에서 읽고, 없으면 0.20 m 로 두고 그렇게 기록한다.
3. 데드맨: `lift_dir` 은 메시지가 계속 와야 움직인다. 끊김·lease 만료·STOP 에서는 목표를 그 순간의 **측정** 승강 위치로 고정한다. 10-17 후속 "실행기 정지 중 lease 만료"와 같은 경로로 시험한다.
4. 복구 후진은 인증 없는 빼기다(자율 withdraw 는 `certify_withdrawal` 없이 빼지 않는다). (a) `recovery_required` 에서만, (b) 기록에 `uncertified_teleop_withdraw` 로 남기고, (c) 금지 접촉을 따로 집계한다. 0.10 m/s 이하. 트럭 뒤 장애물 허가(DrivePermission)는 그대로 적용한다.
5. 곡률 0 명령만으로는 부족하다(조향이 0 으로 돌아오는 동안 호를 그린다): **측정 조향각 ≈ 0**(허용치 설정값)일 때만 속도 > 0.
6. "안 들림" = 측정 승강 ≤ `LIFT_RESUME_TOL_M`(0.01 m). 그 위면 후진 거절, 내리기만(포크만 들린 중간 높이도 내리기부터).
7. free 복귀 판정은 D4 자동 복구와 같은 기준. 적재 중 승강 거절은 유지.

**모형.** URDF `fork_lift` 관절 상한 0.28 m(`dls08_measured/parameters.yaml` 의 `lift_travel_m` 과 같음). 러너가 관절 한계를 읽어 넘긴다. 못 읽으면 0.20 m 로 두고 `teleop_lift_limit_source: "default"` 를 기록한다.

**설계(5판).**

1. **승강 목표는 하나, 소유자는 셋.** 목표는 지금처럼 `ms.lift_command`(액추에이터가 따르는 값) 하나다. 소유자:
   - `core`: 채택된 미션이 running·resuming·recovering 이고 모드가 AUTO. 코어 램프(`lift_and_timers`)가 움직인다.
   - `teleop`: 모드 TELEOP. 중재기의 `TeleopLift` 가 움직인다.
   - `hold`: 그 밖(유휴, PAUSED, STOPPED, 텔레옵 없는 held). 아무도 움직이지 않는다. 지금은 유휴에서도 끝난 미션의 단계로 코어 램프가 돌아 손으로 내린 포크를 다시 들 수 있다 — 라이브에서 채택된 미션이 없으면 코어 램프는 목표를 바꾸지 않는다.
   - **인계**: TELEOP 에 들어갈 때 `TeleopLift` 는 그 틱의 `ms.lift_command` 에서 시작한다(측정이 아니라 — 실린 하중이 처진 만큼 내려가지 않게). 나갈 때는 마지막 목표가 `ms.lift_command` 로 남고 다음 소유자가 그 값에서 시작한다. 미션 채택은 지금처럼 측정에서 시작한다(core 5070).
   - **측정 위치 고정은 사건마다 한 번**(Codex 1회차 P1-1, 2회차 P2-5): **움직이던 수동 승강이 이 틱에 움직일 수 없게 된 첫 틱**에 목표 := 그 틱의 측정 승강, 이후 유지. 이유를 가리지 않는다 — 데드맨 해제, 명령 기한, lease 만료, STOP·preempt, `lift_dir` 0, PAUSED 모드 요청, disarm, 정책이 그 방향을 막음(배속 차단, 주행 시작으로 서 있지 않음 등), TELEOP 이탈. "움직이던" = **허용된 수동 승강 명령의 활성 구간** 안이다: TELEOP·무장·데드맨·기한 안·허용 방향의 `lift_dir ≠ 0` 인 틱이 이어지는 동안(목표가 상·하한에 클립되어 바뀌지 않은 틱도 포함 — Codex 3회차 P2-2: 목표 0.28·측정 0.26 에서 데드맨을 놓으면 20 mm 가 남는다). 그 구간이 끝나는 첫 틱에 고정한다. 매 틱 측정으로 맞추지 않는다(코어 3118 의 처짐 회귀와 같은 이유). 움직이지 않던 승강(적재 중 등)은 목표를 그대로 둔다.
   - **Isaac 정지 게이트**(live_runtime 348)는 액추에이터를 측정값에 래치한다. 그 틱의 제어가 `ms.lift_command` 와 `TeleopLift` 목표를 같은 래치값으로 맞춘다(Codex P2-4: 게이트가 풀린 뒤 옛 목표로 튀지 않게).
   - **측정 없음**(None·NaN·무한): 승강 이동 거절, 목표 유지. `lift_target_m=None`(출력) 은 "중재기가 승강을 갖지 않음"만 뜻한다.
   - 속도 `TeleopLimits.lift_rate_mps = 0.02`(양수·유한, 시작 때 설정의 `lift_rate_mps` 이하 검사), 이동 = `lift_dir × rate × dt`, [0, 상한] 클립. 0.3 s 가 지난 텔레옵 명령은 지금처럼 받지 않으므로(명령 기한) 늦게 몰려 온 패킷이 승강을 움직이지 않는다.
2. **측정값을 중재기에.** `Feedback` 에 `lift_m`·`steering_rad`(좌우 측정 조향 중 큰 절댓값)를 더한다. 유한하지 않으면 None.
3. **정책(teleop_policy).** `Policy` 에 `lift_dirs: frozenset`·`drive_mode` 를 더한다. `lift_allowed = bool(lift_dirs)`(계약 호환). `teleop_allowed = drive_allowed or lift_allowed`(Codex P2-5: 내리기만 되는 상태도 TELEOP 에 들어갈 수 있어야 한다). 배속(pacing) 차단은 주행·승강 둘 다 막는다. 판정 순서: 상태 모름 → 피드백 없음 → 근접 구간 → 복구 규칙 → 포켓 안 거절 → carrying(세션 relation, `load.loaded` 아님 — P3) → free. **근접 구간**은 실행 중인 근접 계약만이다: `near.active`, 또는 채택된 미션이 ACTIVE 이고 `frame.hold ∈ {approach, insert}`(Codex 2회차 P2-3: insert 에서 끝난 미션은 baseline 복원 뒤에도 `frame.hold="insert"` 가 남아 지금 판정으로는 복구가 막힌다). 끝난 미션이 남긴 hold 는 복구 맥락이 있으면 복구 규칙으로 간다.

   | 상태 | 주행 | 승강 |
   |---|---|---|
   | free, 서 있음 | normal | {−1, +1} |
   | free, 움직임 | normal | 없음 |
   | carrying | normal(적재 외곽) | 없음 |
   | 복구 A: `recovery_required` · constrained · **증거 유효** · 접지 확인됨 · 측정 승강 ≤ 0.01 m · 정렬 안(아래 4) | reverse_straight | 없음 |
   | 복구 B: `recovery_required` · constrained/unknown · 측정 승강 > 0.01 m · 서 있음 | 거절 | {−1} |
   | 복구 C: 복구 A 의 조건 중 정렬·접지가 빠짐, 또는 unknown · 승강 ≤ 0.01 m | 거절 | 없음 |
   | 복구 D(빠져나옴 뒤): `manual_clear` · 증거 유효 | reverse_straight(팔레트 구역 면제) | 없음 |
   | 그 밖의 포켓 안 / 측정 없음 | 거절 | 없음 |

4. **reverse_straight**(Codex P1-2, agy 1·7):
   - 곡률 목표 0, 전진 요청 0, 후진 상한 `recovery_reverse_mps = 0.10`.
   - **최종 출력에 거는 조건**: 측정 조향 `|steering| > recovery_steer_tol_rad`(0.01 rad), 또는 정렬 이탈이면 그 틱 출력 0(`slew_cap_stop(..., stop=True)`) — 속도 목표만 0 으로 두면 slew 가 −0.0875 m/s 를 남긴다.
   - **증거**: 시작과 매 틱 `trace_valid` 가 참이어야 한다(Codex 2회차 P2-4). 움직이는 동안 무효가 되면 그 틱 출력 0, 정책은 복구 C.
   - **정렬**: 포켓 축은 삽입 궤적의 앵커(cargo 의 앵커 자세)다. 차의 방향과 앵커 방향의 차 ≤ 0.02 rad, 앵커 축에서의 횡오차 ≤ D4 의 띠(0.02 + 0.05·|순변위| m). 시작 전과 움직이는 동안 매 틱 본다. 넘으면 출력 0, 정책은 복구 C(손으로 더 할 수 없음 — 기록에 사유).
   - 트럭 뒤 장애물 허가(BundlePermit, LiDAR 격자)는 그대로. D10 팔레트 허가는 **복구 대상 팔레트의 구역만** 면제하고 다른 팔레트 구역은 유지한다(Codex P2-6: 구역 안에서는 방향과 상관없이 0).
5. **수동 접지와 증거 기준 전환**(Codex P1-3): 팔레트가 포크 위에 있던 복구(들기·운반·내리기 실패)는 손으로 내린 뒤 **수동 접지 판정**을 지나야 후진이 열린다: 서 있음, 측정 승강 < 0.012 m, (시뮬레이션) `|pallet_z| < 0.008`, 1 s 유지. 그 순간 `TraceEvidence` 를 바닥 기준으로 바꾼다(새 `ground(pallet)` — 자동 복구의 `recover_withdraw` 진입과 같은 전환). 그 뒤 팔레트가 끌려 움직이면 바닥 기준 검사가 증거를 무효로 한다. `ground()` 는 이미 무효인 증거를 되살리지 않는다(무효는 다음 insert 까지 유지 — 지금과 같다). 삽입 단계에서 끝난 복구(팔레트가 바닥에서 들린 적 없음)는 삽입 시작의 바닥 기준이 이미 있다.
6. **free 판정과 manual_clear**(조건 7, Codex P2-6): 틱 끝 `cargo.observe` 가 텔레옵 중에도 깊이를 본다. 깊이 < −(간극 + 띠), 측정 승강 < 0.012 m, 증거 유효(바닥 기준), 서 있음 → relation free·load none. 이어서 **`manual_clear`** 상태: 포크는 빠졌지만 차가 아직 그 팔레트의 D10 구역 안이라 일반 조종으로는 모든 원호가 0 이다. reverse_straight(그 팔레트 구역 면제)만 허용하고, **면제를 뺀 실제 허가가 그 틱 실제 주행 체인의 후진 묶음에 0 보다 큰 속도를 줄 때** normal 로 바꾼다. 묶음은 `TeleopDrive` 가 쓰는 것 그대로(측정 곡률·출력 곡률·설정 여유로 만든 `bundle(...)`), 허가도 같은 `permit` 호출에서 면제만 뺀 것이다(Codex 2회차 P2-1: `inside()` 만으로는 경계 1 mm 밖에서 다시 갇힌다; 3회차 P2-1: 고정 곡률 [−0.05, 0, +0.05] 은 측정 곡률 0.014·여유 0.04 의 실제 묶음 [−0.05, 0, +0.05, +0.10] 보다 작다). 다음 미션의 준비가 시작될 자리인지는 D4b 의 출발 검사가 제출 때 다시 본다(떨어지면 `failed: prepare:*`, 세션 유지 — 손으로 더 물러난 뒤 다시 지시).
7. **복구 맥락 보존**(Codex P2-7): 미션이 relation constrained 로 끝날 때(`recovery_required`, 또는 constrained 인 취소) 코어가 `cargo.recovery_context = {pallet_id, destination_id, 팔레트 추정 자세, placed, pallet_held, destination_held}` 를 남긴다(미션·미션 소유 상태가 지워지기 전). `restore_pallet_target` 은 미션이 없으면 이 맥락을 쓴다. 그때 보류했던 SLAM 종료 해제는 **free 판정의 틱**(서 있음이 조건이다)에 이 맥락의 `*_held` 로 한다(`end_release(deferred=True)` 의 목록 이동 분기). **상대 기하 보존 규칙**: 복구 맥락이 있는 동안(`recovery_required` 로 끝난 뒤 manual_clear 가 끝날 때까지) 코어가 적용 보정을 바꿀 때마다 — 이 지연 해제, 그 뒤 tracking 의 매 SLAM 응답(`slam_replies` 의 재투영 자리) — 그 변화 `C_new ∘ C_old⁻¹` 로 cargo 의 지도 좌표 자세(앵커·삽입 궤적·마지막 자세)를 옮긴다(새 `cargo.reexpress(change)`; Codex 4회차 P2: 해제 직후 한 번만 옮기면 다음 응답의 횡 보정 0.04 m 가 정지한 차를 띠 0.03 m 밖으로 보낸다). 차의 실제 움직임은 오도메트리로 들어오므로 물리적 정렬 이탈은 그대로 검출된다. 그러면 manual_clear 의 정렬 검사와 끝 판정이 모두 **그때의 프레임**에서 이뤄진다(Codex 2회차 P2-2: 옛 앵커로 보면 정지한 차가 횡 0.0975 m 이탈; 3회차 P2-3: manual_clear 끝에서 해제하면 지도 기준 대상은 그대로이고 차만 옮겨져, 해제 전 통과한 허가가 해제 뒤 0 이 된다). 목적지 팔레트는 `delivered` 상태와 목적지와 같은 복원 자세로 둔다(지연 해제 분기의 조건). 맥락은 manual_clear 가 끝날 때 지운다.
8. **기록·접촉**(조건 4, P3): 복구 후진 출력이 0 이 아닌 구간마다 `state["uncertified_teleop_withdraw"]` 에 {시작·끝, 거리, 시작·끝 깊이, 최소 횡간극, 끝난 사유(멈춤·모드 변경·STOP·세션 끝·정렬 이탈)}. 금지 접촉·차체 접촉·횡간극 검사는 그 구간과, 출력이 0 이 된 뒤 **서 있을 때까지** 이어진다. 접촉은 `teleop_withdraw_contacts` 로 따로 세고, 나면 cargo unknown(이후 내리기만).
9. **계약·화면.** 상태의 teleop 쪽에 `lift_dirs`·`drive_mode`(`normal`/`reverse_straight`)·`lift_m`(측정)·`lift_limit_m`·복구 단계(`ground_wait`/`reverse`/`manual_clear`/`blocked:<사유>`) 를 싣는다. 화면은 복구 후진일 때 "복구 후진(곧게, 인증 없음)"과 막힌 이유를 보이고, 막힌 승강 버튼을 끈다.
10. **실차 한계(agy 2–4).** 접지 판정의 `pallet_z`, 금지 접촉, `TraceEvidence` 는 시뮬레이션 참값이다. 실차에는 아직 같은 수단이 없어 **수동 복구의 free 판정은 시뮬레이션에서만** 된다. 실차에서는 이 경로를 열지 않는다(free 판정 근거 없음 → 복구 C). 실차 판정 수단은 메인과 따로 정한다. 승강 체인 처짐(측정 0 인데 포크가 걸림)도 실차 검증 대상이다.

**시험(5판).**
- CPU:
  - 정책 표 전 행과 판정 순서, `teleop_allowed` 가 내리기만 되는 상태에서 참, 배속 차단.
  - `TeleopLift`: 이동·클립·사건마다 한 번 측정 고정(이후 처짐을 따라가지 않음 — 적재 중 TELEOP 유지, 승강 입력 없음), 인계(TELEOP 진입·이탈·게이트 해제), 측정 없음, 설정 검사(양수·유한·≤ 0.06).
  - reverse_straight: 전진 0, 곡률 0, **후진 중** 조향이 허용치를 넘으면 그 틱 출력 0, 정렬 이탈·**움직이는 중 증거 무효**에서 출력 0, 0.10 상한, 팔레트 구역 면제가 그 팔레트에만.
  - 승강 고정 사건 각각(PAUSED·disarm·배속 차단·주행 시작 포함)에서 움직이던 승강만 한 번 고정.
  - 근접 판정: 끝난 insert 미션의 남은 hold 는 복구 맥락이 있으면 복구 규칙, 실행 중인 근접 계약은 지금처럼 거절.
  - manual_clear 끝: 구역 경계 바로 밖(허가 아직 0)에서는 유지, 허가가 생긴 틱에 normal, 다음 틱에도 후진 허가 > 0. 허용치 안의 비영 측정 조향(곡률 0.014)과 바꾼 곡률 여유에서도 실제 묶음으로 판정.
  - free 틱의 해제: 지연 해제 + `cargo.reexpress` 뒤 정지한 차의 정렬 오차가 0, held 플래그별(대상 이동/유지) 허가가 해제 뒤 프레임에서 계산됨.
  - 그 뒤 SLAM 응답으로 보정을 두 번 이상 바꿔도 정지·후진 중 깊이·띠·상대 정렬이 보존되고, 실제 횡 이동(오도메트리)은 이탈로 검출됨. 맥락이 없으면 재표현하지 않음.
  - 승강 활성 구간: 상한 클립 중 데드맨 해제·PAUSED·정책 차단에서 측정으로 한 번 고정.
  - 코어 유휴 램프 정지, 수동 접지 판정과 증거 전환(전환 뒤 함께 끌린 팔레트 → 무효), free 판정(조건 하나씩 빠질 때 유지), manual_clear 의 끝, 복구 맥락으로 대상 복원·지연 해제, 기록 구간이 각 사유로 닫힘, 접촉 집계가 멈출 때까지.
- Isaac U20:
  - (a) 유휴에서 손으로 올리고 내리기(측정이 목표를 따라감, 상한 0.28 m).
  - (b) 실행기 정지(SIGSTOP) 중 lease 만료 뒤 재개 — 승강 목표가 측정에 한 번 고정되고 튀지 않음.
  - (c) 증거가 유효한 채 자동 복구가 막힌 경우(새 시험 플래그: 복구 경로 허가 거절 주입) → `recovery_required` → (들려 있으면) 내리기 → 접지 → 곧은 후진 → free → manual_clear → 다음 미션 `done`.
  - (d) 증거 무효(기존 무효화) → 후진 거절 확인.

**D14 구현(2026-10-19, 6cbee37 … fb07e29, Codex 구현 리뷰 6회차 수렴).**
- 메인 합의 뒤 고친 규칙(메인 세션과 Codex, U20 근거): 사건 때 승강 목표는 늘 clip(목표, 측정 ± a_eff) 하나다. a_eff 는 빈 포크 0.02 m, 적재(cargo carrying 또는 acquiring·confirmed·releasing·unknown)면 max(0.02, 적재 처짐 + 0.01). 처짐은 엔진 기록(`plant_backend`)으로 계산한다: 승강이 싣는 몸체(이름에 fork: 3.6 kg)를 `fork_lift` 강성 4000 N/m 로 나눈다. 빈 처짐 8.8 mm(관측 8.5 mm), 적재(설정·현장표·엔진 팔레트 질량 중 최대 + 현장표 짐) 33.4 mm → a_eff 43.4 mm. 실차 값은 구동기 실측 뒤에 정한다.
  - 처음 규칙(목표 := 측정)은 사건마다 처짐만큼 내려앉았다(U20: 9–13 mm).
  - "움직이던 목표만" 규칙(①)은 상한에서 80 mm 뒤처진 승강을 놓아도 계속 밀었다(Codex 반례).
- 같은 규칙을 수동 승강의 활성 구간 끝, TELEOP 이탈, 정지 게이트 래치, 미션 종료, 라이브 첫 일시정지(lift·lower·recover_lower)에 쓴다. 배치의 첫 일시정지는 측정값 그대로(재생 게이트).
- 시험 플래그 `--live-test-recovery-blocked [withdraw|lower]`: 자동 복구를 되짚기에서, 또는 내리기 전에 거절한다.

**U20 결과(2026-10-19, ws1, 배속 0.4).**
- (a) 수동 승강(pc7d_9c57bc4): 올리고 내리기 정상, 상한 0.28 m(URDF). 놓을 때·끊김 때 내려앉지 않는다(+2·+1.7 mm: 앞선 램프가 a 안에서 멈춤).
- (b) 실행기 SIGSTOP 8 s(main 00ee85f SLAM 링크 수정 병합 뒤): 세션 유지, `long_poll_gaps` 한 쌍, 승강 23.8 mm 그대로, 재진입·내리기 정상. 수정 전에는 `slam_link_failed` 로 세션이 끝났다.
- (c) 수동 복구:
  - 삽입 취소 + 되짚기 거절(pc7b): 곧은 후진 → free → manual_clear → normal → 다음 미션 `done`.
  - 들기 취소 + 내리기 전 거절(pc7e): 내리기만(47.5 → 9.9 mm) → 접지 판정 → 곧은 후진(57 구간, 금지 접촉 0, 최소 횡간극 64 mm) → free → manual_clear → normal. 다음 준비는 차가 4 cm 모자라 `invalid_start`(설계대로 거절·세션 유지). 프로브의 뒤로 물러나기가 회당 약 2 cm 라 재실행 중.
  - 배속 기준(pacing)이 깨지는 동안에는 수동 조종·승강이 거절된다. ws1 부하가 높을 때(부하 평균 35–40) 승강 시험이 판정 불가였다.
- 재생 게이트: main 00ee85f 의 `far_runtime` 미결합(PR #53, 메인에 보고) 때문에 29/29 실패. 그 한 줄만 고친 진단 사본으로는 29/29 통과.

### 구현 메모 — 메인 세션 의견(D8·D9 와 닿는 곳, 2026-10-18)

- **`close_open`**: D8 의 대기 `waits` 는 `begin_wait` 가 연 마지막 항목이 `[t, None]` 인 꼴이다. `end_wait` 와 같은 방식으로 닫아야 `work_elapsed` 가 맞는다. `begin_wait` 는 열린 대기가 있으면 예외를 낸다. 새 미션에서는 `pauses`·`waits` 를 비워(미션 기록으로 옮긴 뒤) 이전 미션 대기가 작업 시계에 섞이지 않게 한다.
- **`abandon_jobs`**: `_drop` 의 부작용은 사이트마다 다르다. release 는 `release_replan_needed` 를 다시 세우고, live_leg 는 `obstacle["live_replanned"][phase]=False`, dock·observe 는 부작용이 없다. 그래서 직접 처리한다. 순서:
  1. `jobs.discard(p.job)`
  2. `commands.end_wait(t)`
  3. `async_plans[-1]` 에 `finished_s`·`dropped:"abandoned"` 를 기록한다.
  4. `pending`·`release_replan_needed` 를 `None` 으로 둔다.
  5. 함께 비운다: `fresh_depth_wait`, `capture_abandon()`, `observation_live`, `capture_acquired_s`.

  `obstacle["live_retry_after_s"]` 는 미션 시작 초기화가 지운다.
- **작업자 `_discarded` 회수는 시간으로 하지 않는다.** 회수 지점은 둘이다:
  - 그 ID 의 답을 받아 버릴 때
  - `_restart_process` 가 again 목록을 만들 때(`_sent` 에 없는 버린 ID)

  대기열에 있던 작업이나 `_sent` 의 첫 번째가 아니던 작업은 늦게 오는 답을 막으려고 그때까지 남는다.
- **turnaround 를 작업자로 옮기기**: `return_turnaround_kwargs()` 의 `arc_check` 클로저가 장애물 층을 그때그때 읽어서 pickle 할 수 없다. 요청 때 고정한 격자 스냅숏을 받는 순수 함수로 바꿔야 한다. 이 변경은 Codex 교차검증을 따로 받는다(D11 선행 조건).
- **종료용 SLAM 해제**:
  - 재투영은 D9 의 `MissionCore._reproject(t, applied_now, rear, loaded)` 를 쓴다. 추종기가 없는 단계는 자기 자세만 `path_ahead` 로 쓴다.
  - 회계는 `slam["holds"]` 와 `obstacle["reprojections"]` 다.
  - D9a 비동기에서는 해제 직후에도 대기열에 남은 답이 다음 틱 시작에 들어와 보정이 한 번 더 바뀔 수 있다. 미션 준비의 채택 검사(보정 버전)가 이것을 잡는다.

### 합의 — 준비 작업 `observe_ladder` (메인 세션, 2026-10-18, D9 머지 9f0d215 뒤)

D2 의 `mission_start` 는 작업자 작업 `observe_ladder` 로 한다. 이름 그대로, 콘솔 브랜치에서 `plan_worker.py` 에 더한다. 조건:
1. payload 는 `waypoints=[(x, y, yaw)…]` 만 받는다. 두 바퀴(모두 `extended=False` → 하나도 안 되면 모두 `extended=True`, T2·seed 4020)는 작업자가 만든다. 셋업과 같은 순서인지 단위 시험으로 확인한다.
2. 배치 셋업 경로는 이 작업으로 바꾸지 않는다. 재생 게이트가 `plan_observation_leg` 를 호출 단위로 기록하기 때문이다. 작업자 루프는 의도된 중복이다.
3. 후보 요약은 `state["observation_candidates"]` 와 같은 키다(`candidate_index`, `pose`, `success`, `status`, `analytic_expansion_interval`, `search_attempts`, `start_rear`, `extended_ladder`). 실패해도 전부 돌려준다(`all_candidates_failed` 사유는 이 status 들의 `;` 연결이다). 출력에 `wall_s` 를 넣는다.
4. 채택 때 코어에 둘 것: `ms.next_candidate_index = selected + 1`(extended 바퀴에서도 waypoint 인덱스 + 1), `state["observation_waypoint_selected"]`, `observation_candidates`, `planning_status`. core 953·1905 의 재관측 폴백이 이 커서를 읽는다. 전부 실패면 `planning_status="all_candidates_failed"`.
5. 시간: payload 에 같은 호스트 monotonic 의 전체 deadline 을 넣는다. 작업자는 후보 사이에서 멈추고, 각 호출에도 그 deadline 을 넘긴다.
6. `grid_kwargs("observe")` 는 요청 틱에 평가한다(D9 의 마지막 응답 스캔 id 를 읽고, submit 때 피클로 고정된다). 세션 중간이면 own 이 지금 rear 라 트럭 밑 칸이 빠지는데, 그게 맞다.
7. 시험: `plan_observation_leg` 를 monkeypatch 하고 `_run` 을 직접 부른다(순서, 첫 성공에서 멈춤, 전부 실패 요약, deadline 멈춤).

§5-2 의 나머지 계약 추가는 메인이 PR 때 본다.

### D2 구현 설계 (2단계, 2026-10-18, 1단계 cc90aca·0c4a08d 수렴 뒤; 3판 — Codex 설계 검토 1·2회차 반영)

**명령 상태.**
- `preparing` 을 `ACTIVE` 와 중재기 `AUTO_STATUSES` 에 더한다. `hold_on` 대상에도 넣는다. 흐름은 `accepted → preparing → running` 이다(Codex P2-3: AUTO_STATUSES 가 따로 있어 빠지면 AUTO 에 못 간다).
- 실행기는 상태가 `accepted`·`preparing` 이면 `pending` 이 있든 없든 그 틱을 계획 대기로 세운다(`plan_wait_`). `paused_` 는 거짓으로 둔다. paused 로 두면 `job_hold` 가 준비 작업을 버린다.
- 취소는 `accepted` 와 같이 곧바로 `cancelled` 다.
- 중재기의 정지 종류가 `none` 이 아니면 준비를 끝낸다. `apply()` 안에서 `terminate` 한다.
  - `preempt` 는 `failed: prepare:stopped` 다.
  - `controlled` 는 `failed: prepare:paused` 다. PAUSED 요청이 여기 해당한다. AUTO→TELEOP 직행은 거절되므로 TELEOP 로 가려면 PAUSED 를 먼저 거친다.
- `hold_on` 은 `failed: prepare:teleop` 다.
- 끝난 미션은 `tick_start` 가 `end_mission` 으로 거둔다. 1단계의 '실행 전 취소 수거'를 FINAL 전체로 넓혀, 결과 기록·작업 폐기까지 한다(Codex P2-3).
- 거두는 자리는 둘이다. **`apply()` 전**에는 앞 틱의 `job_hold` 에서 끝난 준비(시간 초과, 후보 전부 실패, 두 번째 불일치)를 거둔다. `apply()` 가 FINAL 을 `idle` 로 지우기 전이어야 한다(Codex 2회차 P2). **`apply()` 뒤**에는 그 apply 안의 STOP·취소를 거둔다.
- 결과의 끝 시각은 종료 로그의 시각이다. 결과는 미션마다 정확히 한 번 남는다.

**제출 조건(임시, D4·D5 전까지).** 실시간 `Context.clear_for_submit` 은 지난 미션이 끝난 `ms.phase` 로 정한다(Codex P1-2: 부분 삽입 중 취소 뒤 재제출이 받아들여졌다).
- `insert`·`lift`·`extract`·`transport`·`lower`·`withdraw` 이면 거짓이다. 이때 제출은 `recovery_required` 로 거절된다.
- 근거는 `safety_state` 의 `fork_in_pocket`·`load` 와 같은 '단계 기준'이다.
- D4·D5 가 포켓 점유·포크 관계로 바꾼다.

**대상 목록.** `refresh_targets` 는 지금 미션이 **채택된 미션**일 때만 대상을 갱신한다(Codex P1-1: 준비 한 틱에 새 목적지 자리가 이전 시나리오 목적지로 덮였다).
- 판단은 상태가 아니라 `commands.adopted_id == commands.mission_id` 로 한다. 준비가 실패·취소로 끝난 틱에는 상태가 FINAL 이라도 그 미션은 채택된 적이 없다(Codex 2회차 P1).
- `adopted_id` 는 `start()`(배치)와 `adopt()`(실시간)가 정한다.
- 시험은 종료 틱까지 대상이 바뀌지 않는지 본다.

**요청.**
- 자리: 지금의 첫 구간 재계획 자리(장애물 `control_rear` 와 철수 감시 뒤)다.
- 조건: 상태가 `accepted`·`preparing` 이고 `pending` 이 없을 때. 최대 2회다(`core.prepare_attempts`).
- 호출: `core.prepare_request(t, rear, inputs)` → `_request("prepare", "observe_ladder", …)`. 요청하면 `preparing` 이 된다.
- payload:
  - `world = grid_world(미션 시나리오)`
  - `waypoints = relocate_waypoints(후보 원본, 기준, 대상 팔레트 사전 자세)`. 후보 원본은 실시간 시작 때 복사해 둔 **세션 소유의 불변 목록**이다(Codex P2-5: `options.observation_waypoints` 는 `args` 목록과 같은 객체라 두 번째 변환이 옮긴 후보를 다시 옮겼다).
  - `pickup_bounds` = 관찰 경계(아래)
  - `grid_kwargs("observe")` 를 요청 틱에 평가하되, `pickup_obstacle` 은 **새 팔레트의 사전 영역**으로 바꿔 넣는다(Codex P2-4: 지역 `pickup_zone`·`pickup_estimate` 는 옛 미션 것이고, 준비 중에 지역을 바꾸면 P4 를 어긴다).
  - `deadline` = 요청 시각 + 2N × 20 s
- 미션 입력은 요청 때 한 번 만들어 `pending.ctx` 에 둔다. 채택은 그 입력을 그대로 쓴다.
  - 시나리오: 세션 소품·경계, `start_rear`=지금 `rear`, `pickup`=팔레트 사전 자세, `destination`=목적지 사전 자세
  - 사전 영역, 경계, 참값 목적지, 옮긴 후보

**사전 영역과 경계.**
- 팔레트가 세션 시작의 사전 자리(`scenario.pickup`)에 있으면:
  - 사전 영역 = 세션의 `pickup_zone`
  - 접근·삽입 경계 = 세션의 `pickup_bounds`
- 그 밖(배달된 팔레트):
  - 사전 영역 = 사전 자세에 놓은, 팔레트보다 사방 0.3 m 큰 사각형
  - 접근·삽입 경계 = `None`(홀)
- **관찰 이동의 경계는 실시간에서 항상 홀(`None`)이다.** 대상은 준비·관찰 재계획(C927)·관찰 커스프 검사(C3255)다. 코어 옵션에 `observe_bounds` 를 두고, 배치는 지금처럼 `pickup_bounds` 와 같게 둔다(Codex P2-6: 준비만 홀이면 그 뒤 관찰 재계획과 커스프 검사가 같은 자리를 경계 밖으로 본다).

**채택 검사(`_job_invalid` 의 prepare 갈래).**
- 미션 세대, 상태 `preparing`, 보정(`applied`), SLAM 모드·hold 수, 정지, 자세(2 cm, 1°), 대상 revision 이 요청 때와 같아야 한다.
- 다르면 버리고(셈하지 않음) 다음 틱에 실행기가 새 입력으로 한 번 더 요청한다. 두 번째도 다르면 `failed: prepare:<사유>` 다.
- 결과가 `None` 이면 다음 중 하나다:
  - `timed_out`: `failed: prepare:timeout`
  - 그 밖: `failed: prepare:all_candidates_failed:<status;…>`
- 실패해도 세션 상태는 바꾸지 않는다(P4).

**채택 = 미션 시작 처리.**
- 코어 쪽 `core.start_mission(t, plan, inputs, ladder)`: D0 1–6절의 M0·N 항목이다. 메인 합의 4 의 커서·기록과 `options` 필드(`scenario`, `return_to_pose`, `home_rear`, `observation_waypoints` 의 내용, `pickup_bounds`, `observe_bounds`, `pickup_zone`)를 포함한다.
- 실행기 쪽 훅 `live_mission_start(inputs)`:
  - `drive()` 지역을 `nonlocal` 로 다시 묶는다: `scenario`, `truth_destination`, `initial_pallet`, `pickup_obstacle`, `pickup_zone`, `pickup_bounds`, `return_to_pose`.
  - 추적기 drain·비움 뒤 관찰 추적기를 설치한다.
  - 층 `known` 은 `known_enabled` 일 때 새 사전 영역으로 둔다. `KnownRect` 는 값으로 쥐므로 새로 만든다.
  - 허가 캐시를 무효화한다.
  - 도킹 참조 스캔은 새 목적지의 참값에서 다시 뜬다. 실시간에서는 **cast 한 자세(`delivery_rear_reference`)와 `reference_base_z_m` 를 언제나 함께 저장**한다(Codex P2-8: 반복 미션에서는 사전값이 참값과 갈라지는데, 사전값이 참조 자세 자리를 대신해 좌표 변환에 들어갔다). 계획용 사전값은 대상 목록의 목적지 자세다. cast 는 시작 처리 전에 끝내 실패가 상태를 바꾸지 않게 하고, 틱 비용을 잰다(P3).
- 목적지의 참값:
  - `D-1` 은 세션 시나리오의 참 목적지다.
  - `D-0` 은 세션 시작 때 팔레트의 참 자리다. **실시간에서만** 등록한다(P3: 배치 targets revision 이 바뀐다).
- 대기 자리: `return_to_pose`=`home_rear`=세션 시작의 `scenario.start_rear`(또는 `--live-wait-pose`). 복귀 목표 방향은 완화하지 않는다.

**실시간 시작.**
- 셋업의 동기 관찰 사다리를 실시간에서는 돌리지 않는다. 경로 없이 `idle` 로 시작한다(Codex P2-7: 다 실패하면 콘솔이 뜨기 전에 예외가 났다). 배치는 그대로다.
- 실시간은 비동기 계획 작업자를 요구한다. `FORKLIFT_LIVE_ASYNC_PLAN=0` 이면 시작을 거부한다.

**화면(최소).** `ui.js` 의 진행 중 목록(취소 버튼)과 `wizard.js` 의 제출 막기(`blockReason`)에 `preparing` 을 넣는다(P3, Codex 2회차 P3). 표시 개선은 D13 에서 한다.

**지우는 것.**
- `live_first_replan_`, `live_last_rear_`, `Context.at_start`·`not_at_start`
- `blocked_replan(hall=…)`
- 1단계의 '완료 뒤 제출 거절'
- 그 소스 시험들

**시험.**
- CPU:
  - 명령 상태 전이(준비 중 취소·preempt·controlled·hold_on·채택·재요청·실패, 같은 apply 의 종료 수거)
  - 중재기가 `preparing` 에서 AUTO 를 유지함
  - `refresh_targets` 가 준비 중 대상을 바꾸지 않음
  - 포켓 단계에서 끝난 미션 뒤 제출이 `recovery_required` 로 거절됨
  - `relocate_waypoints`: 항등·회전, 두 번 옮겨도 원본 기준
  - prepare 갈래 `_job_invalid`
  - `start_mission` 뒤 미션 소유 상태가 첫 미션 시작 직후와 같음(U21 의 CPU 판)
  - 관찰 경계가 배치에서 `pickup_bounds` 와 같음
- 소스 시험: 실시간 시작이 셋업 사다리를 건너뜀, 지운 임시 처리가 남지 않음.
- Isaac: U2·U3·U4·U5.

### D6 구현 설계 (3단계, 2026-10-18, 2단계 9803b9c 수렴 뒤; 5판 — Codex 설계 검토 1–4회차 반영)

**분류표.** `forklift_core/mission/failures.py` 의 `classify(reason) -> "mission" | "scene" | "session"`. 사유 접두로 고르고(가장 긴 접두), 표에 없으면 `session` 이다.
- **scene**(참값 검사): `Excessive body tilt`, `Actual truck/load footprint overlap`, `Measured pallet footprint overlap`, `Measured pallet solid overlap`, `Pallet dropped during transport`, `Pallet slipped off forks`, `Pallet displaced before pickup`, `Forbidden fork/pallet contact`.
- **session**: 계약·설정 위반(`near_field_contract`, `near_field_without_capture`, `near_field_without_hold`, `near_field_frame_released`, `pocket_check_without_capture`), 캡처 계약(`perception_capture_failed:invalid_capture_pose`, `perception_capture_failed:sensor_metadata_unavailable`; Codex P1-2), 작업자 고장(`plan_worker_dead`; Codex P2-4), 위치 추정·링크(`localization_stale`, `slam_link_failed`, `slam_prewarm_timeout`), 센서(`sensor_silence_stopped`, `Camera did not produce RGB`, `Perception camera did not produce depth`), `Nonfinite body state`, 셋업 사유 전부.
- **mission**: 나머지 미션 일 사유. 추적(`Tracking timeout`, `Tracking failed`), 장애물(`obstacle_`), 인식·근접 캡처(`perception_capture_failed`, `observation_discarded`, `near_capture_`, `near_field_`(위 세 계약 사유 제외)), 관찰(`observe:`, `observe_replan_failed`), 도킹(`docking_`), 삽입 거절(`pocket_check_refused`; 관측 yaw 에 따라 정상적으로도 거절된다, Codex P2-5), SLAM 해제 재계획(`slam_release_`), 작업자(`async_plan_failed`, `async_plan_discarded`, `async_near_no_fresh_depth`, `planning_wait_moving`), 단계(`Pallet was not lifted`, `Pallet did not settle onto destination floor`, `Fork did not lower`, `withdraw_not_certified`, `withdraw_tracking_exceeded`, `pocket_stop`, `transport_recovery_after_docking`, `stall_replan_failed`), 계획(`Mission planning failed`, 미션 계획 상태 `approach:`·`insert:`·`extract:`·`transport:`·`withdraw:`·`return_home:`), 커스프(`cusp_position_stall`).
- 미션 실패가 포켓 단계에서 나면 다음 제출은 2단계의 `recovery_required` 로 막힌다. 그래서 미션으로 분류해도 포크가 포켓 안인 채 새 미션이 시작되지 않는다.
- AST 시험: `run_transport.py`·`core.py` 의 모든 `require` 사유(상수와 f-문자열 접두)가 표에서 기본값이 아닌 분류를 받고, **시험 안의 알려진 사유 목록**과 같다. 새 사유는 넓은 접두(`near_field_` 등)에 걸려도 목록에 없으면 시험이 실패한다(Codex 1회차). 캡처 어댑터의 `CaptureFailure` 사유도 목록으로 고정한다. 동적 사유 호출 자리(`plans.status`, `failure`, `result_.end`, `reason_`, `refusal`, 판정 사유)는 시험 안의 목록에 자리와 분류를 적어 고정한다.

**잡는 자리 — 계획 D6 의 '두 곳' 대신 틱 본문 한 곳.**
- 미션 일은 틱 안에 흩어져 있다: `job_hold`, 준비 요청, 캡처, 추종·도착, `lift_and_timers`, `live_leg`, `paused_tick`·재개, 물리 스텝 뒤 `capture_stepped`. 그래서 `for step` 본문 전체를 `try` 로 감싼다(들여쓰기만 바뀌는 기계적 변경이다. `git diff -w` 로 확인하고 재생 게이트로 배치 불변을 본다).
- `except RuntimeError` 는 **실시간이고 `SessionFailure` 가 아니며 분류가 `mission` 또는 `scene` 일 때만** 처리한다. 그 밖은 다시 올린다(배치는 언제나 다시 올린다).
- 공통 꼬리를 정확히 한 번 실행한다. 본문이 지나간 자리를 틱 플래그(`tick_out_`, `stepped_`)로 남긴다.
  1. `live.end_mission(t, "failed", 사유)` 를 부른다. 정지 훅이 바퀴를 0 으로 둔다. 승강은 지금 높이로 유지한다.
  2. 틱 응답이 아직이면 `live.tick_end(t, applied_commands_, safety, (0, 0), None)` 를 하고, 바퀴 0·조향 유지·승강 유지를 적용하고, `live.publish` 를 한다.
  3. 물리 스텝이 아직이면 `stepper["fn"](False)` 를 한다.
- 잡은 예외는 `state["live_failures"]` 에 `{t, phase, reason, class, traceback 마지막 줄}` 로 남긴다.

**비유한 상태를 먼저.** `Nonfinite body state` 검사에 쿼터니언(차체·팔레트)을 넣고, 참값 검사보다 먼저 둔다(Codex P1-2: NaN 쿼터니언이 `Excessive body tilt` 장면 손상이 됐다).

**작업자 고장.** 작업자가 다시 시작하지 못해 모든 작업이 즉시 실패하는 상태(`_dead`)면 `job_hold`·준비 응답이 `plan_worker_dead`(session)로 끝낸다. 경로 없음과 고장을 섞지 않는다(Codex P2-4).

**종료 때의 정지·승강.** `end_mission` 의 정지 훅이 바퀴 0 과 함께 **승강을 지금 높이로 고정**한다(`ms.lift_command` 와 관절 목표). 틱 응답 여부와 무관하게 언제나 한다(Codex P2-6: 물리 스텝 뒤 `capture_stepped` 실패에서 승강이 빠졌다). 공통 꼬리의 안전 상태는 종료 뒤에 다시 계산한다. 물리 스텝 완료 표시는 `stepper` 가 정상 반환한 직후, `capture_stepped` 전에 둔다(기존 `stepped_` 변수와 이름을 나눈다).

**종료용 SLAM 해제(D1 4단계, Codex P2-3).**
- `end_reset` 이 미션의 release 예약(`pending_release`, `release_wait_from`)을 지운다. 남기면 다음 틱 `release_gate` 의 10 s 시한이 매 틱 같은 실패를 낸다.
- tracker 가 holding 이고 포크가 비어 있으면(아래 단계 기준) 종료용 해제를 한다. 비어 있다는 것은 종료 단계가 포켓·적재 단계가 아니라는 뜻이다.
- 서 있으면 그 틱에 하고, 아니면 설 때까지 미룬다(`release_gate` 가 미션 없이도 처리한다).
- 해제는 다음만 한다:
  1. `release_slam`
  2. **hold 좌표계의 추정만** 좌표계 변화 `after ∘ before⁻¹` 로 한 번 옮긴다(Codex 3회차 P2). 지도 사전값은 옮기지 않는다(`docking_retry.py` 계약). hold 좌표계의 추정은 둘이다.
     - 근접 캡처 hold(관찰·접근에서 끝남): 팔레트 추정(`obstacle["pickup_estimate"]`, `pocket["planning_pickup"]`). 기존 `release_and_stand` 와 같다.
     - 도킹 hold(도킹이 받아들여진 뒤 `settle` 등에서 끝남; 포켓·적재 단계에서 끝나면 해제하지 않는다): 도킹으로 보정한 목적지. 기존 `slam_release` 의 `return_scenario` 재표현과 같다(Codex 4회차 P2). 배달된 팔레트는 이 목적지를 따른다(`refresh_targets`).
     - 즉시 해제: 종료 처리의 대상 확정(`refresh_targets`) **전에** 하여, 옮긴 추정이 그대로 목록에 들어간다.
     - 지연 해제: 종료 뒤에는 `refresh_targets` 가 돌지 않는다(Codex 2회차 P2). 그래서 목록에서 직접 옮긴다. 근접 캡처 hold 면 그 미션 팔레트를, 도킹 hold 면 그 목적지와 거기 배달된 팔레트를 옮긴다. 보정 버전만 올린다(재표현 없음).
  3. 재투영(`_reproject`)
  4. 회계(`slam["holds"]` 에 `reason: "mission_end"`)
- 계획·경로·재관찰 예약은 없다(U28 사후조건). 즉시 해제와 지연 해제 모두 CPU 시험으로 확인한다.
- 포크가 포켓·적재 단계면 hold 를 유지하고 제출은 `recovery_required` 다(6단계 D4).

**종료 뒤 수동 조종(Codex P1-1, D5 전까지의 임시 조건).**
- 안전 상태의 적재·포켓 판단은 아직 단계 기준이다. 그런데 종료는 `ms.phase` 를 그대로 둔다.
- 운반 중 팔레트가 떨어져 끝나면 '적재'로 남아, 떨어진 팔레트가 차체의 자기 반사로 장애물에서 빠진다.
- 그래서 **포켓·적재 단계에서 끝난 미션 뒤에는 수동 조종도 거절**한다. `safety_state` 의 `recovery_required` 를 중재기 허가가 읽고, 거절 사유는 `recovery_required` 다.
- 사용자 결정 4('손상 뒤 수동 조종 허용')는 6단계(D5 포크 관계) 전까지 **포켓·적재 단계 밖에서 끝난 경우에만** 성립한다. 보고에 밝힌다.

**미션 없는 동안의 2D 겹침(Codex P2-7).** 실시간에서 진행 중 미션이 없으면 팔레트 사각형을 2D 겹침 검사(`checked_obstacles`)에서 뺀다.
- 수동으로 포크를 포켓에 넣으면 2D 사각형은 겹쳐도 실제로는 닿지 않는다.
- 실제 접촉은 포크·팔레트 3D 접촉 검사(`Forbidden fork/pallet contact`)가 본다. 이 검사는 지금 insert·withdraw 에서만 돈다. 실시간에서 미션이 없고 **지난 미션이 포켓·적재 단계 밖에서 끝났을 때**(= 수동 조종이 허용될 때)는 단계와 무관하게 돌린다(Codex 2회차 P2). 수동 조종으로 포크가 블록에 닿으면 장면 손상으로 래치된다. 포켓·적재 단계에서 끝났으면 돌리지 않는다. 이 검사는 정상 지지 접촉도 잡고(`insertion_geometry.py` 계약), 그때는 수동 조종이 거절돼 차가 움직이지 않는다(Codex 3회차 P2).
- 소품과의 2D 겹침은 그대로 장면 손상이다.

**장면 손상 래치.**
- 참값 검사 자리(위 scene 사유 8개)를 `scene_require(cond, reason)` 로 바꾼다.
- 배치는 지금의 `require` 와 같다.
- 실시간에서는 처음 거짓일 때 `scene = {"damaged": {reason, t}}` 를 래치하고 예외를 낸다. 그 예외는 위 처리로 진행 중 미션을 `failed` 로 끝낸다.
- 래치 뒤에는 같은 검사가 기록만 하고 예외를 내지 않는다. 같은 조건이 틱마다 거짓이라 틱 본문이 매번 끊기기 때문이다.
- 래치 뒤의 처리:
  - 제출은 `scene_damaged` 로 거절한다(`Context.scene_damaged` 를 추가한다).
  - 수동 조종은 허용한다(사용자 결정 4).
  - 상태 메시지에 `scene` 을 싣는다(계약 추가).
  - 장면 초기화는 4단계(D9)에서 한다.

### D7 요청 원장 구현 설계 (4단계 c, 2026-10-18; 2판 — Codex 설계 검토 1회차 반영)

**왜.** 지금의 `RequestBook` 은 `(client_id, request_id)` 의 결과를 4,096건까지 쥐고 지우지 않는다. 상시 세션에서는 다 차면 모든 새 요청이 `busy` 다. 결과를 지우면 옛 요청의 재전송을 처음 온 요청과 구별할 수 없어 다시 실행된다(예: 회수 뒤 재전송한 `lease_request` 가 새 lease 를 받는다).

**계약(교체 — 계약 1절의 request-ID 중복 판정과 'epoch 끝까지 보존'을 대신한다).**
- `welcome` 에 `request_token`(정수)을 싣는다. 연결마다 새로 주는 단조 번호이고, 다시 쓰지 않는다.
- STOP 이 아닌 신뢰 요청은 `request_token` 과 `request_seq`(그 토큰 안의 단조 정수, 수동 조종의 `seq` 와 별개)를 싣는다. 재전송은 같은 메시지다. `request_id` 는 그대로 회신의 짝을 맞추는 데 쓴다.
- 새 거절 사유 `stale`: 이 연결의 토큰이 아님(다른 클라이언트·지난 연결·위조), 또는 `request_seq ≤ 워터마크`(이미 지운 결과). 다시 실행하지 않는다.
- STOP 은 토큰이 없거나 낡아도 저장 없이 언제나 실행한다(검증기·채널 포함).

**원장(`RequestLedger`, `generations.py`; 실차 쪽과 가짜 차량이 같은 클래스를 쓴다).**
- 토큰은 발급한 연결 `(client_id, conn_id)` 에 묶는다(Codex P2-1). 판정은 원장 조회 전에 한다. 이 연결의 현재 토큰이 아니면 `stale`. 지금의 '이전 conn 차단'도 유지한다.
- 토큰마다 `{client_id, conn_id, results: seq→(확정 시각 또는 PENDING, 결과), watermark}` 를 둔다.
- **실행 전 받아들이기**(Codex P3-6): 저장된 결과가 있으면 그것(PENDING 이면 회신 보류), `seq ≤ watermark` 면 `stale`, 토큰당 결과 상한(256)이나 전체 결과 상한(4,096)을 넘으면 `busy`(저장하지 않음), 아니면 실행한다.
- **PENDING**(Codex P2-5): 미션 요청은 처음 받은 원장 키 `(token, seq)` 를 쥐고, 확정 때 그 키에 쓴다. 재접속 뒤에도 같다. 확정은 상한과 무관하게 허용한다.
- **회수**(틱마다 한 번, 상한으로 묶인 비용): 토큰마다 `seq` 순으로, 확정 뒤 재전송 창(60 s)을 넘은 결과를 지우고 워터마크를 올린다. PENDING 에서 멈춘다.
- **토큰 수 상한**(Codex P2-2): 살아 있는 토큰은 64개까지다.
  - 같은 클라이언트가 다시 연결하면 그 클라이언트의 옛 토큰은 퇴역한다. 새 요청은 받지 않고, 남은 결과가 회수되면 지운다.
  - 상한에서 새 연결이 오면 퇴역한 토큰 중 PENDING 없는 가장 오래된 것을 지운다. 그래도 없으면 그 연결을 받지 않는다(`busy` 로 닫음).
  - 보호되는 토큰은 lease 보유자의 현재 연결 토큰 하나다.
- lease 재획득(새로고침 뒤)은 지금처럼 `client_id` 로 한다. 요청 중복 판정만 토큰으로 한다.

**연결별 welcome**(Codex P2-4).
- 서버 스레드가 잠금 있는 카운터로 토큰을 발급한다.
- 연결 이벤트(`channels.connect(conn_id, client_id, control, token)`)에 실어 제어 스레드로 넘긴다. 같은 순서 채널의 요청보다 먼저 처리되므로, 요청이 오기 전에 원장에 등록된다.
- welcome 은 공용 사전의 사본에 토큰을 더해 그 연결로 보낸다(공용 사전은 바꾸지 않는다).

**클라이언트**(Codex P2-3).
- 웹(`protocol.js`)과 프로브는 welcome 의 토큰을 쥔다. 요청마다 `request_seq` 를 하나씩 올리고, 재전송은 같은 메시지다.
- `stale` 을 받은 요청이 **지난 토큰**의 것이면 그 요청만 끝낸다(재접속하지 않음).
- **현재 토큰**의 것이면 연결을 다시 맺는다.
- 재접속 때 남은 요청을 새 토큰·세대로 다시 내지 않는다. 지금의 명시적인 lease 재획득 예외만 둔다.

**시험.**
- 원장 단위: 실행 전 받아들이기, 같은 seq 재전송, 회수와 워터마크, PENDING 보존과 확정 키, 토큰 수 상한과 퇴역, 남의 토큰.
- 중재기: 회수 뒤 재전송한 `lease_request` 가 새 lease 를 받지 않음, 위조·지난 토큰은 `stale`, 토큰 없는 STOP·옛 토큰 STOP·원장 포화 중 STOP 이 실행됨.
- 실시간 경로: 두 연결의 토큰이 다르고 welcome 직후 요청이 받아들여짐.
- 웹: 지난 토큰의 `stale` 은 재접속하지 않음.
- 가짜 차량: 같은 원장 의미(공통 시험).

**구현에서 남긴 것(2026-10-18).** 가짜 차량(`console/mock_vehicle`)은 아직 자기 요청 저장소를 쓰고 토큰을 주지 않는다. 클라이언트는 그때 토큰 없이 보내므로 예전 규칙대로 동작한다(Codex 4c 구현 검토 P3: 실시간 수정의 선행 조건은 아니지만, 가짜 차량 기반 시험을 원장 검증으로 세지 않는다). 공유 `RequestLedger` 로 옮기는 일은 화면 단계(D13) 전에 한다.

### D8 구현 설계 (5단계, 2026-10-19, 4단계 80fc9ff 뒤; 3판 — Codex 설계 검토 1·2회차 반영)

**측정(U1, c795917, ws1 GPU 0).** 영상을 구독한 채 30분 대기한 뒤 미션 하나를 수행했다. 실행기 RSS 는 5분 6,455 MB → 30분 6,585 MB → 미션 뒤 40분 6,699 MB 였다. 대기 중 증가는 분당 약 5 MB(시간당 약 300 MB)로, 상한 초안 200 MB/시간을 넘는다. VRAM 은 3,280–3,344 MiB 로 평평했다. 종료 때 쓴 기록(시뮬 914 s)은 다음과 같다.

| 기록 | 항목 수 | 디스크 | 증가 단위 |
|---|---|---|---|
| `slam_log` 관절 5목록(팀 엔코더면 8) | 109,655 | npz 35 MB 의 일부 | 물리 스텝(120 Hz) |
| `slam_log` 스캔 3목록(거리 1,600개) | 9,138 | 같은 npz 의 대부분 | 스캔(10 Hz) |
| `slam["control"]` 7열 | 109,606 | 6.1 MB | 스텝 |
| `slam["records"]` | 9,138 | 7.6 MB | 스캔 |
| `obstacle["scans"]` | 9,138 | 1.9 MB | 스캔 |
| `state["samples"]` | 9,134 | result.json 의 6.8 MB | 0.1 s |
| `live_log.jsonl` | — | 41 MB(파일) | 메시지·사건 |

메모리 위 객체 크기로 셈한 합(관절 약 1.5 MB/분 + 스캔 약 1.5 MB/분 + 제어 약 0.8 MB/분 + 나머지 약 0.6 MB/분)이 측정한 5 MB/분과 맞는다. 그 밖에 커지는 것: 근접 추적의 `near["ticks"]` 는 추적 중 제어 틱마다 하나씩 붙어 분당 약 6 MB 다(Codex P2-3). `--live --video` 에서는 `frame_audit`·`video_frames`(로봇 카메라)·겹침 격자 `overlay["grids"]`(0.5 s 마다)가 유휴 중에도 쌓인다(Codex P2-4). `transitions`·`obstacle["plans"]`·`commands.log` 는 미션마다 수 KB 로, U16 에서 잰다.

**읽는 쪽 조사.** 실행 중에 위 목록을 읽는 곳은 모두 마지막 항목뿐이다.
- `slam_log["scan_ranges_m"][-1]`·`["scan_stamps_s"][-1]`: 도킹 스캔 포트(R5338–5351)
- `slam["control"][-1][1:4]`: 코어가 보정 자세로 고쳐 쓴다(core.py 9곳, R6195). 틱 시작의 `collect_end → end_mission → end_reset → end_release` 는 그 틱의 행이 붙기 전에 **직전 틱의 행**을 고칠 수 있다(Codex 확인). `keep=1` 이 그 행을 남기므로 안전하다. 더 오래된 행을 고치는 경로는 없다.
- `obstacle["plans"][-1]`: 덧붙인 직후 같은 호출 안에서 고친다(R3790·3826)
- 전체 목록은 실행이 끝날 때만 읽는다(R6915–7021: 파일 쓰기·요약).
- 경계 기록(`DictWatch` 의 append_only)은 라이브에서 금지라(R902) 스풀과 겹치지 않는다.
- 이 목록들은 모두 D0 의 세션 키다. 미션 종료의 기준값 복원·미션 기록(RECORD_OWNERS) 과 겹치지 않는다(`test_baseline` 이 MISSION_KEYS 와 SESSION_KEYS 의 분리를 확인한다).

**설계.**
1. **스풀 등록(라이브만).** 루프 직전, 목록이 다 만들어진 뒤 `RecordSpool(args.output / "spool", interval_s=10)` 에 등록한다. 모두 `keep=1` 이다.
   - 묶음 `slam_log.joint`: 관절 목록(팀 엔코더 목록 포함). 같은 스텝에 하나씩 붙으므로 한 길이로 자른다.
   - 묶음 `slam_log.scan`: `scan_stamps_s`·`scan_ranges_m`·`laser_pose_world`.
   - 목록: `slam_log.silenced`, `slam.control`, `slam.records`, `obstacle.scans`, `state.samples`, `state.tyre_contact_rows`(켜져 있을 때).
   - 영상 기록(`--video`)이 켜져 있으면 `frame_audit`, `overlay.grids`, `overlay.plan_history` 도 등록한다. `video_frames`(`[-1]` 을 R4918 에서 고친다)는 `--robot-camera` 일 때만 등록한다. 그 기록과 메타는 로봇 카메라가 있을 때만 생기기 때문이다(Codex 2회차 P2-4: 없이 등록하면 종료가 `perception_calibration` 을 읽다 예외를 낸다).
   - `spool.tick(time.monotonic())` 는 틱 시작(어떤 append 보다 먼저)에 부른다. 자르는 비용은 10초분의 슬라이스와 `del` 이다. 스풀은 자르기마다 걸린 벽시계의 최댓값을 manifest 에 적는다.
   - 배치(비라이브)는 등록하지 않는다. 동작·출력이 지금과 같다(재생 게이트 29/29 로 확인).
2. **스풀이 지키는 것**(Codex P2-1·P2-2·P3).
   - `close()` 는 꼬리까지 모두 잘라 마지막 조각으로 쓴다. 그래서 디스크만으로 전체가 되살아난다. 닫을 때 묶음의 목록 길이가 다르면 manifest 에 `misaligned` 로 적는다.
   - 쓰기 스레드가 꺼낸 조각은 다 쓸 때까지 큐 상한에 계속 센다(메모리에 남아 있으므로). 닫기 제한 시간이 지나도 큐에 남았거나 쓰는 중인 항목은 `unsettled_items` 로 센다. 그 뒤에 쓰기가 끝나도 manifest 는 바꾸지 않는다.
   - manifest 의 `complete` 는 누락·미확정·길이 어긋남이 모두 없을 때만 참이다.
   - 조각은 `<n>.<kind>.tmp` 에 쓰고 이름을 바꾼다. 읽는 쪽은 `.tmp` 를 읽지 않는다.
   - 큐 상한은 **아직 쓰지 못한 자르기 회차 수**(기본 6회 = 기록 60초분)다. 넘으면 그 회차 전체를 버리고 센다. 닫기의 마지막 회차는 언제나 받는다. 바이트 추정은 쓰지 않는다. 마지막 항목 × 개수는 크기가 들쭉날쭉한 격자 기록을 과소 계산하고(Codex 2회차 P2-2), 항목마다 재면 10초분에 약 4 ms 가 든다(실측). 회차 상한은 메모리에 남는 밀린 기록을 시간으로 묶는다. 한 회차 분량은 생산 속도로 정해진다(U1 실측 분당 약 5 MB → 60초분 약 5 MB). 쓰기 오류는 총수와 마지막 10개만 둔다.
   - `close()` 는 한 번만 일한다. 다시 부르면 첫 manifest 를 돌려준다(Codex 2회차 P3-1).
3. **종료 때.** `finally` 의 기록 쓰기 앞에서 `spool.close()` 한다. 라이브에서는 스풀에 넣은 목록의 종료 파일(`slam_records.json`·`slam_control.npy`·`obstacle_scans.json`·`slam_log.npz`·`video_frames.json` 의 프레임·`obstacle_grid_frames.npz`)을 메모리에서 쓰지 않는다. 내용은 `spool/` 에 있고, `result.json` 의 `"spool"` 에 manifest(조각·항목·누락·미확정·`complete`)를 싣는다.
   - 요약(`slam_summary` 의 수·RMSE, `obstacle_layer` 의 스캔당 벽시계)은 라이브에서 스풀 조각을 하나씩 읽어 누적해 셈한다(`iter_chunks`, 한 번에 한 조각). `complete` 가 거짓이면 요약에 `incomplete: true` 를 단다.
   - `meta.json`(재생 형식)은 그대로 쓰고 `"spool": {"joint": "slam_log.joint", "scan": "slam_log.scan", "silenced": "slam_log.silenced", "root": "spool"}` 를 적는다. `tyre_contacts.npy` 도 라이브에서는 쓰지 않는다(빈 배열이 되므로, Codex 2회차 P3-2).
   - **읽는 쪽**(Codex 2회차 P2-3). `forklift_core.live.spool` 에 실행 뒤 읽기 함수를 둔다.
     - `load_slam_log(run)`: `slam_log.npz` 가 있으면 그것, 없으면 `meta.json` 이 가리키는 조각을 잇는다. manifest 가 `complete` 가 아니면 거부한다(`allow_incomplete` 로만 읽음).
     - `load_list(root, name)`: 스풀에 넣은 목록 전체. 같은 완전성 검사를 한다.
     - `read_live_log(run)`: 넘긴 로그를 오래된 것부터 이어 읽는다. LiveRuntime 이 넘길 때마다 `live_log.index.json`(넘긴 횟수·보존 개수)을 쓴다. 횟수가 보존 개수를 넘으면(가장 오래된 부분이 지워짐) 거부한다.
     - 복사본에서 조각이나 로그 파일이 빠졌으면 거부한다. 읽은 항목 수를 manifest 의 항목 수와, 있어야 할 로그 파일을 index 와 대조한다(Codex 3회차 P3-1).
     - 라이브 실행을 읽는 `tools/bundle_coverage.py` 는 세 함수를 쓴다(SLAM 기록, `state.samples` 의 명령 곡률, TELEOP 구간). 같은 자료를 배치 형식과 스풀 형식으로 준 두 실행의 결과가 같음을 시험으로 확인한다.
     - 나머지 `slam_log.npz` 를 읽는 도구(`compose_slam_video`·`p0a_measure`·`gate_measure`·`plant_gates`·`curvature_residual` 등)는 배치 실행용이다. 라이브 실행을 주면 파일이 없다는 오류로 멈춘다. 조용히 다른 입력을 쓰지는 않는다.
4. **근접 기록은 미션 소유**(Codex 1회차 P2-3). `near` 의 `frames`·`ticks`·`events`·`corrections`·`path_revisions`·`stop_events` 를 D0 의 M0(기록은 M기록)으로 옮긴다. 미션 종료의 복원이 미션 기록(`missions/<id>/records.json`)으로 넘기고 비운다. 재시도 판정이 지금 일화의 `ticks` 를 모두 읽으므로(C5229–5231) 스풀에는 넣지 않는다. 한 미션의 근접 기록은 그 미션의 추적 시간에 묶인다(60 s 에 약 6 MB). 라이브 종료의 `near_tracking` 기록은 마지막 미션 이후 것만 담는다.
   - **미션 안에서도 묶는다**(Codex 2회차 P2-1). 근접 추적 중 STOP 은 미션을 끝내지 않고 `paused` 로 둔다. 그동안에도 프레임은 10 Hz 로 읽혀 `near["frames"]` 에 쌓인다. 라이브에서는 미션당 12,000개(10 Hz 로 20분)까지 두고 넘친 수를 `frames_overflow`(미션 키)로 센다. `ticks` 는 일시정지 중 붙지 않는다(`paused_tick` 은 근접 틱을 돌지 않음). 배치는 지금처럼 모두 둔다.
5. **LiveRuntime.**
   - 카메라 지연 두 목록은 측정된 프레임 36,000개 창(`LatencyWindow`)이다. 벽시계 1시간이 아니라 측정 개수다(Codex P3). 종료 요약에는 전체 수·전체 최댓값과 창 안의 중앙값·p99 를 싣고, 창이 개수임을 적는다.
   - `arbiter.events` 는 LiveRuntime 이 `live_log` 에 쓴 뒤 비운다. 중재기를 직접 시험하는 시험은 이 비우기를 거치지 않는다.
   - `live_log.jsonl` 은 200 MB 마다 `live_log.jsonl.1`(가장 새것) … `.3` 으로 넘기고 더 오래된 것은 지운다. 제어 스레드와 미션 기록 쓰기 스레드가 함께 쓰므로 쓰기·넘기기·닫기를 잠금 하나로 묶는다(Codex P3). 넘기기는 200 MB 넘는 실행에서만 일어나므로, 짧은 시험 실행을 읽는 도구(`live_console_probe` 등)는 그대로다.
6. **GC 재동결.** live 타이머의 `gc.gen2` 를 U1 재측정에서 보고 정한다. 2세대 수집이 평균 0.8 ms(틱 예산 8.3 ms 의 10 %)·최대 20 ms 안이면 하지 않는다.
7. **SLAM 브리지.** 컨테이너 안 별도 프로세스라 실행기 RSS 에 들지 않는다. U1 재측정에서 브리지 RSS 도 잰다. 늘면 ROS 패키지 변경으로 따로 다룬다(이 단계 밖).

**합격 기준(Codex 1회차 P2-5·2회차 P3-3 반영).** 틱 p99 비교는 10초마다 한 번인 자르기를 가리지 못하므로 쓰지 않는다. 자르기 비용은 직접 재고, 쓰기 스레드의 GIL 경쟁과 로그 넘기기는 틱 간격 전체로 본다. 틱 간격은 `live_log` 의 틱마다 기록(`mono`)에서 구한다.
- U1 재측정(영상 구독, 30분 대기 뒤 미션):
  - 실행기 RSS 의 5분 뒤 → 30분 증가 < 100 MB, 5–30분 선형 기울기 < 200 MB/시간
  - manifest `complete: true`, 오류 0
  - 자르기 최대 벽시계(`cut_wall_max_s`) < 5 ms(로컬 실측 0.2–0.4 ms)
  - 틱 간격: 스풀 없는 U1(c795917)과 비교해, 간격 p99.9 증가 < 2 ms, 최대 간격 증가 < 10 ms. 페이싱 보고의 실시간 비율 차이 < 2 %, `stalls` 가 늘지 않음
  - 미션 `done`, 배달 참값 판정 통과
- U16 짧은 판(미션 3회 반복, 영상 기록 켬): 위 기준에 더해 첫 미션 뒤와 셋째 미션 뒤의 RSS 차이 < 100 MB(미션당 남는 것이 50 MB 아래). 미션마다 남는 것과 영상 경로를 본다.
- U30(새, STOP 뒤 오래 대기): 근접 추적 중 STOP → **시뮬 시간 25분** 대기(상한 12,000개는 시뮬 10 Hz 로 20분이다. 배속 0.4 에서 벽시계 약 63분) → 취소 → 새 미션. 대기 동안 RSS 증가 < 100 MB, 그 미션 기록의 `frames_overflow` > 0(상한이 동작, Codex 3회차 P3-2), 새 미션 `done`.
- 디스크: 세션 디스크 < 5 GB/8시간(U1 의 시간당 증가로 외삽해 확인).

**시험(먼저 쓴다).**
- 스풀 단위: 잘라 쓰기, 묶음 한 길이, 밀린 회차 상한(크기가 들쭉날쭉해도 회차 전체가 함께 감), 쓰기 실패 계수와 오류 상한, 닫기 때 꼬리까지 기록, 닫기 재호출, 쓰는 중 제한 시간 초과의 `unsettled_items`·`complete: false`, `.tmp` 무시, 길이 어긋남, 자르기 비용 기록, `iter_chunks`·`load_group`·`load_list`·`load_slam_log`(불완전 거부)·`read_live_log`(지워진 부분 거부).
- 근접 프레임 상한: 상한까지 두고 넘친 수를 셈, 배치는 모두 둠, 러너의 다섯 곳이 모두 상한 함수를 거침.
- 기준값: 근접 기록이 미션 기록으로 넘어가고 비워짐, 근접 추적이 없는 실행에서는 `near` 기록 없음.
- 러너 원천 시험: 라이브만 등록, 틱 시작 호출, 종료 순서(close 가 기록 쓰기 앞), 라이브 종료 파일 생략, 배치 경로 불변.
- 요약 누적: 조각으로 셈한 `slam_summary` 가 같은 자료를 한 번에 셈한 값과 같음(CPU, 합성 자료).
- LiveRuntime: 지연 창, `arbiter.events` 를 한 번만 기록하고 비움, 로그 넘기기와 `live_log.index.json`, 다른 스레드가 쓰는 중의 넘기기.
- `bundle_coverage`: 배치 형식과 스풀 형식으로 같은 자료를 준 두 실행의 결과가 같음(예전 읽기 코드로 되돌리면 실패함을 확인).
- Isaac: U1 재측정, 이어 U16 의 짧은 판(미션 3회 반복, 영상 기록 켬), U30.

**U1 재측정 결과(2026-10-19, d57e0da 스냅숏, ws1 GPU 0, 배속 0.4, 30분 대기 뒤 미션 P-1→D-1, 직접 실행).** 미션 `done`, 종료 코드 0.

| 기준 | 스풀 없음(c795917) | 스풀(d57e0da) | 판정 |
|---|---|---|---|
| 실행기 RSS 5→30분 증가 < 100 MB | 130.5 MB | 7.2 MB | 통과 |
| 5–30분 기울기 < 200 MB/시간 | 325 MB/시간 | 16.7 MB/시간 | 통과 |
| manifest `complete`, 오류 0 | — | 참, 0(자르기 244회, 누락·미확정 0) | 통과 |
| 자르기 최대 < 5 ms | — | 0.12 ms | 통과 |
| 틱 간격 p99.9 증가 < 2 ms | 65.7 ms | 68.1 ms(+2.45) | 근소 초과(아래) |
| 최대 간격 증가 < 10 ms | 17.7 s | 10.2 s | 통과 |
| 실시간 비율 차이 < 2 %, stalls | 0.3756, 1 | 0.3739, 1 | 통과 |

- p99.9 차이는 스풀 때문이 아니다. 조각 파일의 수정 시각으로 자르기 244회의 시점을 되살려, 자르기 직후 0.6 s 안의 틱 7,133개와 나머지 101,549개를 비교했다. p99.9 67.4 대 68.2 ms, 50 ms 초과 7.14 % 대 7.15 % 로 같다. 차이는 모든 틱에 고르게 있어, 두 실행 사이의 ws1 부하 차이로 본다(이번 실행 시작 때 부하 평균 12.7).
- 디스크는 40분에 148 MB(스풀 없음 105 MB)다. 시간당 약 220 MB, 8시간 약 1.8 GB 로 5 GB 상한 안이다.
- 같은 실행에서 SLAM 브리지 컨테이너는 분당 약 10.6 MB 늘었다(D8b 의 원인). 이 스냅숏에는 D8b 수정이 없다.

### D8b SLAM 브리지의 자원 (5단계 b, 2026-10-19, d57e0da 뒤; 4판 — Codex 1–3회차 반영)

**측정(U1 재측정 초반, d57e0da 스냅숏).** 브리지 컨테이너 메모리가 대기 중 분당 약 10.6 MB 늘었다(시간당 약 640 MB). 컨테이너 한도가 4 GB 라 약 6시간이면 OOM 이다. 프로세스별로 재면 `isaac_slam_bridge` 만 30초마다 5.3 MB 늘고 slam_toolbox 는 그대로다. 원인은 브리지가 `/map` 을 받을 때마다 격자를 통째로 `map_arrays` 에 쌓는 것이다(`isaac_slam_bridge.py` 150). slam_toolbox 는 지도를 1초마다 발행하고(`map_update_interval: 1.0`), 이 장면의 격자는 약 176 KB 다. 176 KB × 60장/분 ≈ 10.6 MB/분으로 측정과 맞는다. 정지 중에도 같은 지도가 매번 다시 쌓인다. 응답 기록 `records` 도 스캔마다(10 Hz) 붙는다.

**읽는 쪽.** 실행 중에 읽는 것은 `latest_map.npz`(계획기, 지도마다 원자적으로 다시 씀)뿐이다. `maps.json`·`maps.npz`·`bridge_records.json` 은 종료 때 쓴다. 배치 영상 도구(`compose_online_slam_video.py`)는 `maps.json`·`maps.npz` 와 실행기의 `slam_records.json` 을 읽는다(`bridge_records.json` 은 읽지 않는다). `bridge_status.json` 은 실행 판정이 읽는다.

**설계.** 브리지에 ROS 인자 `persistent`(기본 거짓)를 둔다. 거짓이면 지금과 같다. 참이면 다음과 같다.
1. 직전과 같은 지도(격자 바이트·크기·해상도·원점 위치와 방향·`frame_id` 가 같음)는 다시 쌓지 않는다. 반복 구간을 센다. 정지 중에는 늘지 않는다. `latest_map.npz` 는 지금처럼 지도마다 다시 쓴다.
2. 바뀐 지도는 메모리에 두지 않는다. 받는 즉시 `maps/map_NNNNNN.npz`(압축, `.tmp` 에 쓰고 이름 바꿈)에 쓰고 메타는 `maps.jsonl` 에 한 줄 붙인다. 메모리에는 비교용 직전 격자 하나만 둔다.
3. **번호·시각 계약**(Codex 1회차 P3). `index` 는 지금처럼 **수신 순번**이다. 반복도 순번을 하나 쓴다(`latest_map.npz` 의 `index` 가 달라야 실행기가 다시 설치한다, R3682). 저장된 지도의 줄은 그 지도의 **처음** `after_scan_id`·`stamp_ns`(메시지 시각)·`received_wall_s`(수신 시각)를 갖고, 반복이 덮어쓰지 않는다(영상 도구는 `after_scan_id < last_scan_id` 로 지도가 있었는지 판정한다). 반복 구간은 다음 바뀐 지도가 올 때와 종료 때 `{"repeat_of": i, "repeats": n, "last_index", "last_stamp_ns", "last_after_scan_id", "last_received_wall_s"}` 한 줄로 확정한다. 종료 전에 프로세스가 죽으면 열린 반복 구간 한 줄은 잃는다(지도 자체는 이미 디스크에 있음).
4. 응답 기록은 `records.jsonl` 에 한 줄씩 붙인다. 메모리에는 판정에 필요한 실패 수와 "stray 가 기록된 응답이 있었는지"만 둔다. 마지막 응답 뒤의 stray 는 지금처럼 별도 줄을 남기고 실패로 센다.
5. **응답 경로를 막지 않는다**(Codex 1회차 P2). 기본 콜백 그룹은 상호 배제라, 지도를 저장(압축)하는 동안 응답이 기다리는 `/pose` 콜백이 멈춘다. `/map` 구독을 자기 그룹(`map_group`)으로 뗀다. 응답 기록은 `sendall` 뒤에 넘긴다(`finally` 로, 보내기가 실패해도 기록은 남음). serve 스레드는 기록을 넘긴 직후 다음 스캔을 읽으므로, 파일 쓰기는 자기 쓰기 스레드가 한다(대기 줄 10,000개 상한. 큐 초과·쓰기 오류는 `records_dropped`, 종료 기한에 남았거나 쓰는 중이던 줄은 `records_unsettled` 로 `bridge_status.json` 에 셈, Codex 2·4회차). 판정용 실패 수·stray 여부는 넘길 때 바로 고친다. 실행기의 응답 제한은 1 s 다.
6. 종료(`save`)는 먼저 `bridge_status.json` 을 `{"pending": true, "ok": false}` 로 쓰고, 기록을 다 쓰고(쓰기 스레드를 기한 30 s 까지 기다림) 열린 반복 구간을 확정한 뒤 최종 판정으로 바꾼다. 두 번 모두 `.tmp` 에 쓰고 이름을 바꾼다. 기한에 쓰는 중이던 줄과 남은 줄은 `records_unsettled` 로 세고, 그 뒤에 쓰기가 끝나도 바꾸지 않는다(Codex 3회차 P2 2건). 상시 모드의 상태에는 `records_dropped`·`records_unsettled` 가 붙는다.
   - 라이브 작업의 정지 스크립트(`isaac_live.sbatch` 의 `stop.sh`)는 브리지 컨테이너가 스스로 끝나기를 최대 45 s 기다린 뒤 지운다. 실행기가 소켓을 닫으면 브리지는 저장하고 끝난다. 지금은 실행기가 끝나자마자 지워, 저장 중인 기록이 잘릴 수 있었다. `maps.json`·`maps.npz`·`bridge_records.json` 은 쓰지 않는다. 배치 도구에 상시 세션 출력을 주면 파일이 없다는 오류로 멈춘다.
7. 켜는 곳: `tools/slam_bridge_container.sh` 의 넷째 인자 `persistent`. 라이브 실행을 띄우는 `deploy/console/isaac_live.sbatch` 와 ws1 시험용 `live.sbatch` 가 넘긴다.

**디스크.** 지도 보관량은 정지 중 0, 주행 중 바뀐 지도당 수십 KB 다. 응답 기록은 정지 중에도 스캔마다 한 줄씩 는다(10 Hz × 약 300 B ≈ 시간당 11 MB). `latest_map.npz` 는 덮어쓰므로 늘지 않는다. U1 재측정에서 둘을 재서 세션 디스크 상한(8시간 5 GB)과 대조한다. `stray` 목록은 지우지 않지만, 쌓이면 다음 응답이 실패로 끝나므로 정상 대기 중에는 늘지 않는다.

**slam_toolbox 자체.** 정지 중에는 늘지 않았다. 주행하면 자세 그래프가 거리만큼 커지는 것은 slam_toolbox 의 성질이다. U16 에서 미션당 증가를 재고, 상한을 넘으면 따로 다룬다.

**시험.** ROS 를 가짜 모듈로 바꾼 지금의 규칙 시험 방식으로 한다.
- persistent 끔: 지도 두 장이 `maps` 와 `map_arrays` 에 쌓이고 save 가 `maps.npz` 를 쓴다(지금과 같음).
- persistent 켬: 같은 지도 100장은 메모리·파일이 늘지 않고 반복 구간만 센다. 바뀐 지도는 파일로 가고 메모리에 남지 않는다. 응답 기록은 `records.jsonl` 로 간다. save 는 상태 파일을 쓰고 열린 반복 구간을 `maps.jsonl` 에 확정할 뿐, `maps.json`·`maps.npz`·`bridge_records.json` 은 쓰지 않는다(Codex 2회차 P3).
- 반복 구간이 처음 지도의 `after_scan_id`·`stamp_ns` 를 지키고, `frame_id` 가 다르면 새 지도다.
- `/map` 구독이 자기 콜백 그룹을 쓰고, 응답 기록이 `sendall` 뒤에 있다. 쓰기가 멈춘 디스크에서도 기록 넘기기가 기다리지 않고, 상한을 넘은 줄은 버리고 센다.
- 컨테이너 스크립트가 넷째 인자로 `-p persistent:=true` 를 넘기고, 두 sbatch 가 그것을 넘긴다.
- Isaac: U1 재측정에서 브리지 메모리 증가 < 20 MB/시간(정지 중).

**D8b 확인(2026-10-19, 5611c13 스냅숏, ws1, 배속 0.4, 15분 대기 뒤 미션 P-1→D-1, 직접 실행).** 미션 `done`, 종료 코드 0. 브리지 컨테이너 메모리는 대기 15분 동안 113.1 → 113.8 MB 로 평평했다(수정 전 분당 약 10.6 MB). 미션 중에는 132 MB 로 늘었다(+19 MB, 주행 중 지도·자세 그래프). 바뀐 지도 147장이 `maps/` 에 저장됐고 브리지 출력은 모두 3.2 MB 다. `bridge_status.json` 은 `ok: true`, `records_dropped`·`records_unsettled` 0. 미션당 브리지 증가는 U16 에서 잰다.

### D5a 물리 상태 구현 설계 (6단계 a, 2026-10-19, 5611c13 뒤; 4판 — Codex 1–3회차 반영)

6단계를 나눈다.
- **6a**(이 절): 포크 관계와 적재를 세션 상태로 두고, 미션이 끝난 뒤의 판단(제출·수동 조종·SLAM 해제·화면)을 그 값으로 한다.
- **D10**(허가가 팔레트를 봄): 6a 바로 다음에 한다(순서 변경, 아래 '남는 위험').
- **6b**: D4 복구 단계(`recover_lower`·`recover_withdraw`)와 그 깊이 추정.
- **6c**: D5 적재 제출.

**지금.** 포켓 안 여부와 적재는 단계 이름으로만 정한다.
- `safety_state` 의 `load.loaded` 는 `phase in (lift, extract, transport, lower)`, `fork_in_pocket` 은 `phase in (insert, lift, extract, lower, withdraw)` 다(C3067·3069).
- 미션이 끝난 뒤 제출은 실행기의 `clear_for_submit`(R5912, 단계와 SLAM hold)으로 거절하고, `safety_state` 는 `recovery_required` 를 붙인다(C3086). 수동 조종 정책은 `fork_in_pocket` 이 참이면 거절한다(`teleop_policy.py` 37). 그래서 운반 중 취소한 차는 적재 상태로 움직일 수 없다(사용자 결정 '적재 텔레옵 허용'과 어긋남).
- 수동 조종 허가의 외곽은 단계 이름을 직접 읽는다(R5491).
- 종료용 SLAM 해제는 `phase not in POCKET_PHASES` 로 정한다(C4587). 운반 중 취소하면 hold 가 남아, 적재 수동 조종 중 보정이 멈추고 누적 이동 15 m 에서 `LocalizationStale` 로 끝난다(`slam_pose.py` 258).

**왜 기하로 증명하지 않는가(Codex 1·2회차).** 라이브 실행에는 근접 추적이 없고(U1 미션 기록의 근접 틱 0건), 팔레트 추정과 제어 자세의 지도 좌표 비교는 오차가 시간으로 커진다(오차 표 0.5 s 에 59 mm, 알려진 팔레트 계층이 꺼진 이유). 삽입 시작에서 포크 축을 따라 잰 상대 깊이는 실측 오차가 작지만(삽입 42구간 최대 17.7 mm, 추출 41구간 24.8 mm, 철수 41구간 15.2 mm), 마지막 관측과 삽입 시작 사이의 이동, 철수 때 앞 오차의 계승, SLAM 보정 점프가 반례를 만든다(2회차 P1-2, P2-3·4·5). 그래서 6a 는 포크 관계를 **증명하지 않고**, 미션이 포크를 어디에 두었는지(단계 전이)와 들기·내림 판정을 근거로 보수적으로 둔다. 근거는 출력에 `basis: "phase+cargo"` 로 적는다. 상대 깊이는 6b 복구의 후진 거리에서 상한으로 쓴다.

**상태(세션 소유 `core.cargo`, 라이브만 읽음).**
- `pallet_id`: 지금 포크와 관계있는 팔레트. 채택한 미션의 팔레트이고, 미션이 끝나도 남는다.
- `relation`: `free` / `constrained` / `carrying` / `unknown`. 세션 시작은 `free` 다(포크는 비어 있고 팔레트 밖이라는 장면 계약).
- `load`: `none` / `acquiring` / `confirmed` / `releasing` / `unknown`.
- 전이(`transition()` 안, 배치도 같은 갱신을 하지만 배치 출력은 읽지 않음):

| 전이 | relation | load |
|---|---|---|
| `→ insert` | constrained | none |
| `→ lift` | constrained | acquiring |
| `lift → extract`(들기 판정 통과) | carrying | confirmed |
| `→ lower` | constrained | releasing |
| `lower → withdraw`(내림 판정 통과) | constrained | none |
| `withdraw → settle`(철수 경로 끝) | free | none |

  - 미션이 그 밖의 단계(관찰·접근·정착·복귀)에서 끝나면 관계는 바뀌지 않는다. 그 단계에서는 포크가 팔레트 밖이다(접근은 앞면 앞 간격에서 끝남).
- **무효화**(2회차 P2-6, 3회차 P2 2건). 팔레트 관련 검사가 실패를 **관측하는 그 자리에서** `load = unknown`, `relation = unknown` 으로 바꾼다. 미션이 있는지, 이미 끝났는지, 첫 장면 손상 래치가 이미 섰는지와 상관없다(래치가 선 뒤 `scene_require` 는 계수만 하고, 끝난 미션의 `end_mission` 은 바로 돌아오므로, 미션 종료에 묶으면 적재 수동 조종 중 낙하를 놓친다).
  - 검사마다 원인을 표시한다(`scene_require(..., cargo=True)`). 메시지와 배치 동작은 지금과 같다.
    - 팔레트 관련: `Pallet dropped during transport`, `Pallet slipped off forks`, `Pallet displaced before pickup`, `Forbidden fork/pallet contact`, `Measured pallet footprint/solid overlap`, 팔레트 기울기(지금은 차체 기울기와 한 검사 `Excessive body tilt` 안에 있다, R6016 — 둘로 나누고 메시지는 그대로 둔다), 외곽 겹침 중 대상 팔레트(`pickup_obstacle`·측정 팔레트 사각형)와의 겹침(지금은 다른 장애물과 한 검사 `Actual truck/load footprint overlap` 안에 있다, R6022·6055 — 같은 방식으로 나눈다).
    - 차체만: 차체 기울기, 다른 장애물·홀 경계와의 외곽 겹침, 비유한 자세. 이것들은 포크 관계를 바꾸지 않는다. 그래서 차체 손상 뒤 수동 조종(사용자 결정)이 그대로 된다.
  - 낙하·미끄러짐 검사는 지금 `ms.phase in (extract, transport)` 일 때만 돈다(R6116). 라이브에서는 `relation == "carrying"` 이면 미션이 끝난 뒤에도 돈다(적재 수동 조종 중).

**쓰는 곳(라이브만, 배치 출력 불변).**
1. **제출 게이트**. `tick_start` 의 `clear_for_submit`(참·거짓)을 거절 사유로 바꾼다. `constrained`·`unknown` 이면 `recovery_required`, `carrying` 이면 `cargo_held`(6c 전까지), 그 밖은 통과다. SLAM hold 중 거절은 그대로 둔다. 판정은 직전 틱 끝의 `core.cargo` 로 한다(명령 적용이 이번 틱의 제어보다 먼저이므로).
2. **수동 조종 정책**. `teleop_policy` 가 `fork_in_pocket.relation` 을 읽는다. `free` 는 빈 차 외곽, `carrying` 은 적재 외곽으로 허용한다. `constrained`·`unknown` 은 `fork_in_pocket` 으로 거절한다(복구 조종은 D14·6b). 허가의 외곽 선택(R5491)도 라이브에서는 `relation == "carrying"` 또는 `load in (acquiring, releasing)` 이면 적재 외곽이다.
3. **`safety_state`**: 라이브에서는 `load = {"loaded": relation == "carrying" 또는 load in (acquiring, releasing, unknown), "basis": "phase+cargo", "pallet_id", "state"}`, `fork_in_pocket = {"value": relation != "free", "basis": "phase+cargo", "relation"}`. `recovery_required` 는 1번과 같은 조건이다. 배치는 지금 값 그대로다(재생 게이트가 비교함).
4. **종료용 SLAM 해제**. D1 대로 `relation in (free, carrying)` 이면 해제한다. `constrained`·`unknown` 이면 해제하지 않는다.
5. **상태 발행의 팔레트 ID**. `safety/state.py` 의 적재 표시는 미션 대신 `load.pallet_id` 를 읽는다.

**남는 위험(2회차 P1-1).** `free` 인 차를 수동 조종으로 팔레트 포켓에 넣으면 `free` 가 그대로 남는다. 알려진 팔레트 계층이 꺼져 있고 접촉 검사도 100 mm 삽입에서 0건이라 지금은 막는 것이 없다. 이 위험은 6a 가 만든 것이 아니라 지금도 있다(접근 단계에서 끝난 미션 뒤의 수동 조종). D10(수동 조종 허가가 대상 목록의 팔레트를 외곽 + 고정 여유로 봄)이 막는다. 그래서 D10 을 6a 바로 다음, 6b 보다 먼저 한다. D10 전까지 라이브 수동 조종에서 팔레트 쪽 접근은 운용자 책임이다(지금과 같음).

**시험.**
- 전이 표: 정상 단계 순서의 관계·적재, 각 단계에서 미션이 끝났을 때 남는 값.
- 무효화: 팔레트 관련 실패는 `unknown`, 차체 겹침은 관계 유지.
- 제출·수동 조종: 접근 중 종료 → 제출·조종 허용, 삽입·들기·내림·철수 중 종료 → `recovery_required`·조종 거절, 운반 중 종료 → `cargo_held`·적재 외곽 조종 허용, 그리고 종료용 해제가 일어남.
- 배치 불변: 배치 `safety_state` 출력이 지금과 같음(재생 게이트 29/29).
- Isaac: 접근·삽입·운반 단계 취소 각 1회.

### D10 구현 설계 (2026-10-19, 6a c500c19 뒤 · 6b 앞; 4판 — Codex 1–3회차 반영)

**왜 지금.** 6a 의 포크 관계는 미션이 포크를 둔 자리로 정한다. 그래서 `free` 인 차를 수동 조종으로 팔레트 포켓에 넣으면 `free` 가 그대로 남는다(6a '남는 위험'). LiDAR 평면(1.05 m)은 0.144 m 팔레트를 보지 못하고, 알려진 팔레트 계층은 꺼져 있으며(`config/obstacle_layer_single.yaml` 79–87), 포크 접촉 검사도 100 mm 삽입에서 0건이다. `pickup_obstacle` 의 2D 검사(R6025)는 참값 판정이고 미션 없는 라이브에서는 빠진다. 그래서 지금 수동 조종 허가는 팔레트 앞에서 멈추지 않는다.

**대상과 사각형.**
- 대상: 대상 목록의 팔레트 중 `carried` 가 아닌 것. 실은 팔레트는 적재 외곽에 들어간다(6a 의 외곽 선택).
- `prior`(아직 인식 전): 대상 목록의 자리는 시나리오 참값이라(R4675, 시뮬레이션의 등록) 쓰지 않는다. 픽업 구역 사각형(`pickup_zone`, 배치 분포를 팔레트 반대각선만큼 키운 것, R2938)을 쓴다. 대기 자리의 포크 끝에서 구역까지는 3.15 m 라 시작부터 갇히지 않는다(Codex 1회차 확인). 그 구역에도 차량 자세 오차 여유 0.182 m(위치 0.130 m + 방위 2 × 1.66 × sin(0.031/2) = 0.0515 m, 아래와 같은 실측, 올림 — Codex 4회차 P3)를 더한다. 구역은 팔레트 배치의 불확실성만 덮고, 차량이 자기 자리를 틀리게 아는 것은 덮지 않는다(Codex 3회차 P1 반례: 추정 뒤축이 0.13 m 옆이면 구역 밖으로 보이던 차체가 직진 0.028 m 에서 팔레트와 겹친다).
- `recognised`·`delivered`: 팔레트 외곽(깊이 × 폭)을 여유 `m = 0.21 m` 만큼 키운다. 근거(2026-10-19 실측, U1 재측정 두 실행의 표본 최댓값, 상한 아님): 지도 좌표 인식 오차 0.022·0.026 m(`perception_pickup_estimate_m` 대 참값), 지금 제어 자세의 위치 오차 최대 0.123·0.130 m(스풀 `slam.control`), 방위 오차 최대 0.031 rad 이 외곽의 가장 먼 점을 옮기는 거리 2 × 1.66 m × sin(0.031/2) = 0.051 m(적재 외곽 T11 의 먼 모서리 1.66 m, EPAL 1.61 m, `pallet_mission.py` 303 — Codex 2회차 P2-4). 합 0.207 m. 내려놓은 자리는 도킹 판정의 목적지 추정이고 배달 오차가 0.018 m(U2) 라 같은 여유를 쓴다. 지금의 오차 표(시간으로만 늘어남)는 SLAM 이 보정하는 지도 좌표에는 맞지 않아 쓰지 않는다. 값은 설정으로 두고, U11·U16 에서 다시 잰다.
- 번들의 원호 사이 틈은 기존 허가처럼 `envelope_m`(0.01 m, `docs/plans/2026-10-17-teleop-bundle-margin.md` 46)으로 덮는다. 사각형을 그만큼 더 키운다(오차 여유와 따로 센다).
- 이렇게 키운 사각형이 **출입 금지 구역**이다. 실제 팔레트는 오차가 여유 안이면 그 안 어딘가에 있다. 구역 안의 어느 자리가 안전한지는 이 근거로 알 수 없다(Codex 2회차 P1-1).

**판정(Codex 1·2회차).**
- 외곽은 기존 허가와 같다: `layer.footprints(loaded, direction)` 의 부품(전진은 hull, 후진은 차체 + 날; 적재는 적재 외곽, `obstacle_layer.py` 218).
- 원호마다 지금 자세부터 걷는 거리 + 정지 거리 + 한 칸까지 0.02 m 간격으로 자세를 놓는다. 각 자세에서 외곽 부품(방향 있는 사각형) **면 전체**와 구역 사각형의 교차를 분리축 정리로 정확히 판정한다(경계점 표본이 아니다 — 2회차 P2-3: 차체 1.114 × 0.72 m 는 T11 0.66 m 사각형을 통째로 품을 수 있다).
- 샘플 사이의 이동은 구역을 더 키워 덮는다. 한 칸에서 외곽의 어느 점이 움직이는 거리의 상한은 `step + |κ| × step × R_max`(R_max = 뒤축에서 외곽의 가장 먼 점, 적재 T11 1.66 m)이다. κ 는 원호마다 실제 검사하는 곡률이다. `bundle()` 은 곡률 여유와 바깥 반올림을 더해 0.45 까지 검사하므로(`teleop_drive.py` 66), 그때 0.02 m 칸의 pad 는 0.035 m 다(Codex 3회차 P3). 기존 허가의 구간 pad(`drive_permission.py` 386)와 같은 생각이다.
- 처음 교차하는 거리 `s` 가 막히는 거리다. 허용 속도는 `stopping.speed_for(s)`(자기 여유 0.05 m 포함, `drive_permission.py` 50), 사유는 `pallet:<id>`. 교차가 없으면 제한이 없다.
- **이미 구역 안이면**(지금 자세에서 교차) 모든 원호가 0 이고 사유는 `pallet_near` 다. 구역 안에서 어느 쪽으로 나가면 안전한지는 이 근거로 판정할 수 없다(2회차 P1-1 반례: 실제 뒤축이 0.13 m 옆이면 직진 후진 0.04 m 에서 차체가 팔레트와 겹친다). 빠져나오는 일은 6b 의 복구(미션이 들어온 직선을 되짚어 나감 — 그 길은 방금 차가 지나간 자리다)가 맡는다.
  - 그래서 6b 전까지는 접근 끝에서 취소한 차(포크 끝이 앞면 0.10 m 앞)를 수동으로 뺄 수 없다. 안전 쪽을 고른 결과다. 그동안은 장면 초기화로 되돌린다.

**캐시(Codex 1회차 P1·P2-5, 2회차 P1-2).** `BundlePermit` 과 같은 방식이다.
- 방향마다 따로 둔다.
- 키는 (방향, 원호 끝 인덱스, 원호 수, 적재 외곽, 대상 목록 revision, 격자 스냅숏 키, 적용된 SLAM 보정)이다. 추적 중의 보정은 응답마다 바뀌고 대상 revision 은 해제 때만 오르므로(`slam_pose.py` 206, C3015), 보정을 키에 따로 넣는다. 스냅숏 키가 스캔마다 바뀌므로 설치는 대개 스캔마다(10 Hz) 일어난다.
- 설치할 때 원호마다 막히는 거리를 구하고, 그 뒤에는 달린 거리를 빼서 쓴다. `refresh_m`(0.25 m)을 달리면 다시 설치한다.
- **매 틱 양 끝 원호는 지금 자세에서 다시 판정한다**(기존 허가와 같음, `teleop_drive.py` 183).

**비용.** Codex 의 로컬 참고 측정(`shape_meets` 직접 순회, 팔레트 하나, 1.03 m)은 원호 3개 1.8 ms, 17개 9.3 ms(중앙값)였다. 분리축 판정을 numpy 로 자세 전체에 한 번에 한다. 설치(스캔마다)와 매 틱 양 끝 판정의 비용을 ws1 에서 방향 전환·최대 번들·대상 변경·영상 구독 조건으로 재고, 틱 간격·페이싱이 D8 기준(U1 대비 p99.9 +2 ms 안)을 지키는지 본다.

**계약.** 대상 목록의 팔레트에 `basis`(`prior`·`recognised`·`delivered`)와 `uncertainty_m` 을 더한다(계약 3절 확장, 화면은 표시만). `prior` 의 표시 자리는 지금처럼 등록 자리이지만, 허가는 구역을 쓴다는 것을 계약에 적는다.
- 근거는 이렇게 바뀐다: 등록 = `prior`. `refresh_targets` 가 `planning_pickup`(관찰의 인식값)으로 자리를 고치면 `recognised`. `put_down` 이면 `delivered`. 보정 재표현은 근거를 바꾸지 않는다.

**실차.** 같은 규칙이 필요하다(중재기 쪽 허가). 이 변경은 Isaac 실행기의 라이브 허가에만 넣고, 실차 중재기 적용은 PR 에서 메인 세션과 합의한다(사용자 결정 3 '변경 허용').

**시험.**
- 기하: 직진 원호가 팔레트 앞에서 멈추는 거리, 비켜 가는 원호는 제한 없음, 차체가 팔레트 사각형을 통째로 품는 자세도 교차로 판정(2회차 P2-3), 샘플 사이 pad, 구역 안이면 모든 원호 0(2회차 P1-1 반례 자세), `carried` 제외, 근거별 사각형.
- 캐시: 방향 전환 틱에 전진·후진이 따로 계산됨(1회차 P1 반례: 후진 결과가 전진에 재사용되어 0.2375 m/s), revision·보정 변경에 다시 설치(2회차 P1-2 반례: 보정 −0.12 → +0.12 m, 이동 0, revision 불변에서 캐시 0.60 m/s 대 재계산 0), 매 틱 양 끝 재판정, 달린 거리 차감.
- 대상 목록: 근거 전이(등록 → 인식 → 운반 → 내려놓음), 메시지에 `basis`·`uncertainty_m`.
- Isaac U11: 인식된 팔레트로 수동 직진 → 팔레트 앞에서 멈춤. 이 검사를 끄면 같은 시험이 실패함(포크가 포켓에 들어감). 같은 실행에서 틱 간격·설치 비용을 잰다.

**U11 결과(2026-10-19, D10 구현 스냅숏, ws1 GPU 0·1 동시, 배속 0.4, 대기 자리에서 수동 조종 0.3 m/s · 곡률 0.065 · 벽시계 120 s, 직접 실행).**

| 실행 | 마지막 뒤축 x | 포크 끝 x | 사전 구역 앞면 x(키운 뒤) | 정지 사유 `pallet:P-1` | 장면 손상 | 틱 p99.9 |
|---|---|---|---|---|---|---|
| D10 켬 | 0.526 m | 1.82 m | 1.91 m | 251회(발행 상태) | 없음 | 67.4 ms |
| D10 끔(`--live-test-no-pallet-permit`) | 1.404 m | 2.69 m | 1.91 m | 0회 | 없음 | 68.5 ms |

- 켬은 구역 앞 0.09 m 에서 멈췄고 끔은 그 자리를 지나 구역 안으로 0.78 m 더 들어갔다. 켬의 정지는 D10 이 만든 것이다. 비용은 틱 간격으로 드러나지 않는다(멈춤 0, 실시간 비율 0.368 대 0.376).
- 위 실행에서는 끔에서도 팔레트와 닿지 않았다. 프로브가 속도·곡률을 한도에 대한 **비율**(`speed_ratio`·`curvature_ratio`)로 보내는데 m/s·1/m 값을 넣어, 실제로는 0.15 m/s·0.023 1/m 로 움직여 팔레트 옆(y ≈ 0.1, 팔레트 y 0.64–1.44)을 지나갔다(시험 설정의 잘못, 시스템 문제 아님).
- **비율 0.6·0.18(0.3 m/s·0.065 1/m)로 다시 돌린 결과.** 켬: 뒤축 x 0.523 에서 멈춤(포크 끝 약 1.81 m, 구역 앞면 1.91 m), 장면 손상 없음. 끔: 시뮬 30.1 s 에 캐리지 가로대가 팔레트 블록·스트링거에 닿아 `Forbidden fork/pallet contact in observe` 장면 손상(포크 관계 `unknown`). **U11 통과**: 켜면 팔레트 앞에서 멈추고, 끄면 같은 조종이 팔레트에 닿는다.

### D4 복구 구현 설계 (6단계 b, 2026-10-19, D10 682d239 뒤; 6판 — Codex 1–5회차 반영)

**지금.** 취소는 정지가 확인되면 곧바로 `cancelled` 다(`paused_tick`, C2857). 포크가 팔레트에 걸린 채(6a 의 `constrained`) 끝나면 제출·수동 조종이 모두 거절되고(`recovery_required`), D10 은 키운 구역 안에서 모든 원호를 막는다. 차를 꺼낼 길이 장면 초기화뿐이다.

**삽입 궤적과 깊이(Codex 1회차 P1-1·P1-2, 2회차 P1·P2-2).** 라이브에는 근접 추적이 없어 팔레트와의 상대 깊이를 직접 잴 근거가 없다. 대신 포크가 들어온 궤적을 기록한다.
- 삽입 진입(`approach → insert`)에서 `anchor = (그 순간 제어 자세, 방향 h)` 를 코어의 세션 상태(`core.cargo`)에 둔다. 접근 끝은 포크 끝이 앞면 앞 0.10 m 인 자리다(계획 좌표, 6a 의 Codex 확인). 그래서 그 자리의 깊이는 −0.10 m 다.
- **깊이는 순변위로 잰다**: `depth = −0.10 + (지금 뒤축 − anchor 뒤축)·h`. 이동량 합은 쓰지 않는다. 정지 중에도 오도메트리 잡음이 적분되어, 60 s 서 있으면 합은 0.60 m 가 되지만 순변위는 0.006 m 다(Codex 반례).
- 삽입 중 제어 자세를 0.02 m 마다 지도 좌표로 기록한다(`insert_trace`, 각 점의 깊이와 함께). 첫 점은 anchor(깊이 −0.10 m)다. 들기 판정 때 깊이를 고정한다(`depth_fixed`).
- **궤적의 지도 배치는 한 번만 정한다**(Codex 3회차 P2-1). 픽업에서는 기록한 그대로다. 목적지에서는 내리기 진입 순간에 `T = (그 순간 자세) ∘ (삽입 끝 자세)⁻¹` 로 한 번 옮겨 둔다(팔레트가 포크에 같은 상대 자세로 실려 왔다는 전제, 아래). 그 뒤 철수·일시정지·재개가 있어도 배치는 바뀌지 않는다. 복구는 이 배치된 궤적에서 **지금 깊이에 해당하는 점부터** 거꾸로 자른다. 지금 자세에 궤적을 다시 맞추지 않는다(그렇게 하면 이미 철수한 거리가 두 번 들어간다).
- **궤적 근거의 확인**(Codex 3회차 P2-2, 4회차 P2-1, 5회차 P2 2건). 시뮬레이션은 팔레트가 어디 있느냐에 따라 기준을 나눠 매 틱 참값으로 본다.
  - 팔레트가 바닥에 있는 동안(삽입 중, 내림 판정 뒤 철수 중): 팔레트의 지도 자세가 기준 값(삽입 시작 때, 내림 판정 때)에서 벗어나지 않는지.
  - 팔레트가 포크에 실린 동안(삽입 끝 `insert → lift` 부터 내림 판정까지 — 들기·추출·운반·내리기): 포크에 대한 팔레트의 상대 자세가 **삽입 끝 때 값**에서 벗어나지 않는지(궤적의 변환과 같은 기준 시점이다. 들기 판정 때로 잡으면 들기 중의 회전이 기준에 흡수된다).
  - `recover_lower` 의 접지 판정 때는 복구에 들어갈 때 팔레트가 실려 있었으면 상대 자세를, 바닥에 있었으면 지도 자세를 본다.
  - 범위: 종방향 0.02 m, 횡방향 0.01 m, 방위 0.01 rad. 하나라도 넘으면 궤적 재사용을 무효로 한다(`trace_valid = false`). 이것은 실패가 아니다. **복구가 움직이기 전에** `recovery_required` 로 끝난다(픽업·목적지 모두). 금지 접촉 검사까지 가지 않는다.
  - 기존 미끄러짐 검사(종 ±0.08 m·횡 ±0.04 m, 추출·운반만, 방위 없음)는 실패 판정으로 그대로 둔다. 방위 0.01 rad 는 팔레트 깊이 1.2 m 에서 끝이 12 mm 움직이는 값으로, 철수 횡간극 최소 27.9 mm 안이다.
  - 실차에는 이 상대 자세를 잴 센서가 없다. 실차의 복구는 그 근거가 생길 때까지 열려 있다.
- 목적지에서는 내리기 진입(`→ lower`)에서 `anchor` 를 다시 두고(그 순간 자세와 방향), 깊이는 `depth = depth_fixed − (anchor 뒤축 − 지금 뒤축)·h` 다(철수는 순변위로 뺀다).
- 띠 `u = 0.02 + 0.05 × |anchor 이후 순변위|`(6a 의 실측: 삽입 42구간 17.7 mm/0.43 m, 철수 41구간 15.2 mm/0.55 m).

**언제 복구하는가.** 라이브에서 취소가 정지로 확인된 틱에 포크 관계를 본다.
- `free`·`carrying`: 지금처럼 `cancelled`(적재는 6a 의 `cargo_held`).
- `constrained` 이고 철수 중이며 `depth < −(0.10 + u)`(포크 끝이 앞면 앞으로 이미 빠져나옴): 관계 `free`, `cancelled`(Codex 1회차 P3: 철수 후반의 취소).
- 그 밖의 `constrained`: 복구로 들어간다.
- `unknown`: 복구하지 않는다. 결과 `recovery_required`(근거가 없으므로 운용자 복구, D14).

**복구 단계.**
1. `recover_lower`(복구는 언제나 여기서 시작한다, Codex 1회차 P2-6): 승강 목표 0, 지금의 들기·내리기와 같은 속도로 내린다. 끝나는 조건은 승강 < 0.012 m 이고 팔레트 접지 판정(지금의 내림 판정과 같은 참값, `abs(pallet_z) < 0.008`, C426)을 통과하는 것이다. 승강이 처음부터 낮아도 접지 판정은 한다. 통과하면 적재 `none`.
2. `recover_withdraw`: **배치해 둔 삽입 궤적을 되짚는 경로**다. 배치한 궤적의 시작(깊이 −0.10 m)에서 그 방향으로 여유 구간(깊이 −(0.10 + u_max)까지, `u_max` 는 지금 쓸 수 있는 가장 큰 띠)을 먼저 붙인 뒤, 그 연장된 궤적을 **지금 깊이로 자르고** 끝점 깊이 `−(0.10 + u)` 까지 거꾸로 따라간다(Codex 4회차 P2-2: 철수가 −0.10 m 를 지났지만 이탈 판정 전이면 기록 점이 없다). 지금 깊이가 끝점과 같거나 지났으면 움직이지 않고 끝낸다(추종기는 같은 자리의 연속 점을 거절한다, `path_tracking.py` 164). `trace_valid` 가 거짓이면(픽업·목적지 모두) 들어가지 않고 `recovery_required` 다. 첫 기록점(0.02 m) 전에 취소되면 궤적은 anchor 한 점이다. 그때는 anchor 방향의 직선을 쓴다(그 사이의 방향 변화는 기록 간격 안이다). 끝점은 깊이 `−(0.10 + u)` 다(앵커 자리가 아니라 깊이로 정한다 — Codex 2회차 P1: 목적지 앵커에서 끝점을 앵커 뒤로 두면 0.21 m 가 남는다). 철수 속도(`withdraw_speed_mps`)로, 지금의 철수와 같은 추종기로 따라간다.
   - 횡 근거: 지금의 철수도 포켓 안의 추종 후진이다. 이전 실행 413회의 철수 구간에서 포크 날과 포켓 사이 최소 횡간극은 27.9 mm, 삽입 466회는 28.0 mm 였다(참값 `lateral_clearance_min_m`, ws1 기록 전수, 2026-10-19). 같은 추종기가 기록 궤적을 되짚으므로 같은 범위로 본다. Isaac 시험에서 복구 구간의 횡간극을 같은 방식으로 잰다.
   - 지금 방향의 직선이나 축 직선은 쓰지 않는다(Codex 반례: 삽입 중 횡오차를 고치며 방향이 0.015–0.034 rad 돌아, 그 직선으로 되짚으면 2.6–4.3 mm 어긋나고 가까운 벽에서는 간극 검사가 실패한다).
3. 끝에 서면 포크 관계 `free`, 미션은 `cancelled` 로 끝난다(라이브 미션만 끝내고 세션 루프는 그대로 — 도착 처리가 루프를 끝내지 않는다, Codex 1회차 P2-5).

**허가와 검사(Codex 1회차 P1-3·P2-7).**
- `permission_acting()` 에 `recover_withdraw` 를 넣는다(지금은 단계 열거, C594). 경로를 허가에 설치하고, 주행·일시정지 감시·바퀴 제한이 다른 주행 단계와 같게 걸린다. 허가는 LiDAR 격자로 뒤쪽을 본다. 알려진 팔레트 계층이 꺼져 있어 팔레트와 포크의 간극은 허가가 보지 않는다. 그것은 삽입 축 되짚기의 근거와 아래 접촉 검사가 맡는다.
- 시작 전에 허가가 경로 전체를 판정해 막히면 실패다.
- 금지 접촉 검사와 횡간극 기록(R6166, 지금은 `insert`·`withdraw`·미션 없는 free)을 `recover_withdraw` 에 건다. `recover_lower` 에는 걸지 않는다. 그 단계는 바퀴가 0 이라 수평으로 움직이지 않고, 하중을 받친 포크는 정상 지지 접촉을 금지 접촉으로 읽힌다(검사기 계약, `insertion_geometry.py` 3 — Codex 2회차 P2-3: 승강 0.100 m 의 정확한 지지에서도 포크·스트링거 4쌍). 접지를 확인한 뒤 `recover_withdraw` 에서 전체 검사를 건다. 시험은 이 검사가 실제로 돌았고 금지 접촉이 0건이었음을 본다.

**명령 상태(Codex 1회차 P2-4·P2-5).**
- `MissionCommands` 에 복구 표시 `recovery`(일시정지·재개를 지나도 남음)를 둔다. 복구에 들어가면 취소가 연 정지 구간을 닫고(작업 시계가 다시 흐름), 상태는 `recovering`(ACTIVE)이다.
- 복구 중 취소는 받지 않는다(`recovering`). 일시정지·STOP 은 지금의 일시정지와 같다(바퀴 0, 승강 유지, 작업 시계 멈춤). 재개는 `recovering` 으로 돌아간다(`running` 이 아님). 재개하면 같은 표에서 다시 판단한다. 승강·접지가 아직이면 `recover_lower`, 아니면 남은 깊이로 `recover_withdraw` 경로를 다시 만든다.
- `recovery_required` 를 FINAL 에 더한다. `terminate("recovery_required", t, reason)` 가 가능해지고, 결과 기록부는 그대로 받는다. 이 상태를 읽는 곳을 모두 맞춘다: 중재기(AUTO 상태·종료 판정), `live_runtime` 의 종료 수집, 제출의 FINAL 거절, 화면 문구.
- 실패(허가 거절, 시간 초과 — 복구 단계마다 작업 시계 20 s, 접지 실패)는 `recovery_required` 로 끝난다. 포크 관계는 `constrained` 로 남는다.

**SLAM.** 복구는 삽입·내리기 때의 hold 안에서 움직인다(보정 점프 없음). 끝나 `free` 가 되면 6a 의 종료 해제가 일어난다.

**포크 관계 전이(6a 표에 더함).** `recover_lower → recover_withdraw`: constrained·none. `recover_withdraw` 끝: free·none. 철수 후반 취소의 이탈: free·none.

**시험.**
- CPU:
  - 취소 정지 틱의 분기(free·carrying → cancelled, 철수 후반 이탈 → cancelled, constrained → recovering, unknown → recovery_required)
  - 순변위 깊이(정지 중 잡음이 합에는 쌓이고 순변위에는 안 쌓임), 목적지 anchor, 띠
  - 되짚기 경로: 궤적의 배치(픽업은 기록 그대로, 목적지는 내리기 진입 때 한 번), 지금 깊이로 자른 시작점(철수 중·일시정지 재개 뒤에도 일치), 깊이 기준 끝점, 첫 기록점 전 취소·마지막 기록점 사이 취소, 허가가 그 경로에 걸림(막히면 recovery_required)
  - 궤적 근거 확인: 실린 동안의 기준이 삽입 끝 때 값, 들기 판정 전에 상대 자세가 바뀌는 경우 잡음, 바닥에 있는 동안은 지도 자세(정상 철수는 무효로 보지 않음), `recover_lower` 접지 때 실려 있었는지에 따른 기준, 범위를 넘으면 `trace_valid` 거짓 → 픽업·목적지 모두 움직이기 전에 recovery_required
  - 여유 구간 안의 깊이(−0.10 m 를 지났지만 이탈 전)에서 잘림, 끝점과 같은 깊이에서 움직이지 않고 끝남
  - 접지 판정을 늘 함(승강이 낮아도)
  - 단계별 20 s 시간 초과
  - 복구 중 취소 거절, 일시정지·재개가 `recovering` 으로 돌아오고 남은 깊이로 다시 만듦, 작업 시계
  - `recovery_required` 종료의 수집·결과 기록·제출 거절
  - 배치 불변(배치는 복구하지 않음, 재생 게이트)
- Isaac:
  - U7: 삽입 중간, `approach → insert` 바로 뒤 틱, 들기 중, 내리기 중, 철수 초반 취소
  - U26: 부분 상승·접지 지연
  - U27: 철수 초반(복구)·후반(이탈 → cancelled), 철수 중 60 s 일시정지 뒤 취소
  - 복구 중 반복 STOP·재개, 목적지 도킹 재시도 뒤 철수 중 취소
  - 판정: 접촉 검사가 돌았고 접촉 0건, 복구 뒤 포크 끝이 앞면에서 떨어짐(참값), 결과 1회 기록, 종료 해제, 다음 미션 `done`

**U7 1차 결과(2026-10-19, b3fbd6b, ws1 GPU 0·1, 배속 0.4, 직접 실행).** 첫 미션 P-1→D-1 을 단계별로 취소했다.

| 취소 | 지나간 상태·단계 | 결과 | 복구 시작 깊이 / 경로 | 복구 중 횡간극 최소 | 금지 접촉 |
|---|---|---|---|---|---|
| 삽입 3 s | running → cancelling → recovering; insert → recover_lower → recover_withdraw | cancelled | 0.045 / 0.172 m | 76.6 mm | 0 |
| 들기 2 s | 같음(lift →) | cancelled | 0.315 / 0.435 m | 83.4 mm | 0 |
| 내리기 2 s | running → recovering; lower → … | cancelled | 0.321 / 0.441 m | 71.4 mm | 0 |
| 철수 1 s | withdraw → … | cancelled | 0.229 / 0.353 m | 66.7 mm | 0 |
| 철수 5.5 s | cancelling → cancelled(이미 빠져나옴, 복구 없음) | cancelled | — | — | 0 |

- 복구 자체는 다섯 경우 모두 설계대로 끝났다.
- **다음 미션은 다섯 경우 모두 준비에서 실패했다**(`prepare:all_candidates_failed:invalid_start`). 아래 6b-2.

### D4b 취소 뒤 다음 준비가 시작될 자리 (6단계 b-2, 2026-10-19, b3fbd6b 뒤; 4판 — Codex 1–3회차 반영)

**원인.** 다음 미션의 준비(관찰 경로)는 그 미션 팔레트를 장애물로 넣는다(`live_prepare`). 팔레트가 처음 자리면 픽업 구역 전체, 아니면 팔레트를 사방 0.30 m 키운 사각형이다. 계획기는 출발 자세의 외곽(포크를 포함한 전체 hull)을 그 장애물에 계획 여유 0.10 m(`planning_clearance_m`)를 더해 본다(`hybrid_astar.py` 556). 복구는 앞면 앞 약 0.12–0.14 m 에서 끝나고, 접근 끝 취소는 0.10 m, 복귀 시작은 0.22 m 다. 그래서 다음 준비가 시작되지 않는다. D10 의 수동 조종 허가도 그 자리에서는 모든 원호를 막는다.

**판정 기준은 단계 이름이나 고정 거리가 아니라 다음 준비의 출발 검사 자체다**(Codex 1회차 P2-2·P2-3). 그 검사를 코어에 두고(`prepare_start_ok(rear)`), 준비 단계가 쓰는 것과 같은 외곽·장애물·여유로 판정한다.

**설계.**
1. **준비의 팔레트 장애물을 근거로 고른다.** 지금은 "등록 자리와 같은가"로 구역과 0.30 m 사각형을 고른다. 대상 목록의 근거(D10)로 바꾼다: `prior` 는 픽업 구역, `recognised`·`delivered` 는 팔레트 외곽을 `uncertainty_m`(0.21 m)만큼 키운 사각형. `live_prepare` 와 `prepare_start_ok` 가 같은 함수로 이 장애물을 만든다.
2. **라이브에서 미션이 끝날 자리를 출발 검사로 확인한다.** 취소가 정지로 확정된 틱(포크 관계 `free` 이든 철수 후반의 이탈이든), 복구(`recover_withdraw`)가 끝난 틱에 지금 자세로 `prepare_start_ok` 를 본다.
   - 검사는 **해제 뒤의 입력**으로 한다(아래). 처음 검사든 물러나기 뒤든 같다. 모든 `cancelled` 확정이 이 관문을 지난다(Codex 3회차 P2-1: hold 안에서 통과해도 SLAM 지도가 들어가면 떨어질 수 있다).
   - 통과하면 끝난다(`cancelled`).
   - 떨어지면 **물러나기**로 간다. 삽입 복구(`recover_withdraw`)와 다른 단계 `clear_out` 이다(Codex 3회차 P2-3: 같은 단계로 두면 재개가 삽입 깊이 규칙으로 끝난다). 상태는 `recovering`, 승강·접지 판정은 없다:
     - 마지막 움직임이 전진이었으면(관찰·접근 중), 방금 지나온 실제 궤적을 거꾸로 되짚는다. 코어가 라이브에서 **오도메트리 자세**(보정 전, `slam["odom_rear"]`)를 0.02 m 마다 최근 3 m 까지 기록한다(`recent_trace`). 제어 자세로 기록하지 않는다: 추적 중의 SLAM 보정 점프가 움직이지 않은 이동으로 섞인다(Codex 2회차 P2-1). 물러나기를 시작할 때 SLAM 을 hold 로 두고(이미 hold 면 그대로, 아니면 `hold_slam`), 그 hold 의 보정으로 궤적을 한 번 제어 좌표에 옮긴다. 물러나는 동안 보정이 고정이라 그 경로가 곧 오도메트리 궤적의 역순이다. 장면은 그대로이고, 차가 방금 차지했던 자리다.
     - 마지막 움직임이 후진이었으면(철수·복귀 시작·복구 끝), 지금 방향으로 곧게 더 물러난다. 그 길은 허가가 LiDAR 격자로 본다.
     - 되짚는 것은 **마지막 연속 전진 구간**까지다(구현 리뷰 1회차 P1-2: 기어가 바뀐 궤적 전체를 후진 경로로 만들면 전진 구간이 후진으로 표시된다). 그 구간으로 모자라면 구간 시작 자세의 방향 반대로 곧게 잇는다. 이 직선은 지나온 궤적이 아니라 새 경로이고, 허가가 LiDAR 격자로 본다(구현 리뷰 2회차 P3).
     - 경로는 표본마다 `prepare_start_ok` 를 보고, **추종 허용 오차를 더한 여유로** 처음 통과하는 표본에서 끝낸다(Codex 1회차 P2-2). 더하는 여유는 접근 도착 허용 0.03 m + 복구 추종 위치 0.008 m + 방위 0.02 rad × 외곽 먼 점 1.66 m(0.033 m) = 0.071 m → 0.08 m.
     - 최대 1.0 m. 그 안에 통과하는 표본이 없거나 허가가 막으면 `recovery_required`.
   - **해제 뒤의 입력**(Codex 2회차 P2-3, 3회차 P2-2): 해제하면 제어 자세는 그때의 최신 보정으로 옮겨지고, 준비의 격자에는 SLAM 지도가 들어간다(holding 중에는 빠짐, R3690). 검사는 (최신 보정 ∘ 지금 오도메트리) 자세, SLAM 지도를 넣은 격자, 그리고 **실제 종료 해제와 같은 판정으로 옮긴 팔레트 대상**으로 한다. 그 판정은 지금의 `_held_estimates`(그 hold 가 찾은 추정만 옮기고, hold 전의 지도 추정은 그대로) 하나를 둘이 같이 쓴다. 물러나기를 위해 새로 시작한 hold 에서는 팔레트가 hold 전 지도 추정이라 옮기지 않는다. 여유 없이 본다.
   - 물러나기 끝에서 떨어지면 같은 방향으로 0.05 m 씩 더 물러나며 다시 본다(전체 1.0 m 안). 끝내 떨어지면 `recovery_required`.
   - **물러나기의 일시정지·재개**: 재개는 저장한 물러나기 경로(그 hold 의 제어 좌표), 지금까지의 진행량, 남은 1.0 m 예산을 그대로 쓴다. 지금 자세를 그 경로에 투영한 점부터 남은 부분을 따라간다. hold 는 유지한다. 재개 때 hold 가 풀려 있으면(외부 해제) `recovery_required` 다.
3. **복구한 팔레트는 접지 판정 때 대상 목록에 다시 올린다**(Codex 2회차 P2-2). 지금은 `recovery_done` 이 미션을 끝내면서 올려, 복구 끝의 검사가 볼 팔레트가 없다(들기·내리기 취소는 `carried`·자세 없음). `recover_lower` 의 접지 판정을 통과하는 순간 `put_down` 한다(픽업: 그 미션의 픽업 추정, `available` / 목적지: 도킹 판정의 목적지 추정, `delivered`). 복구 중 `refresh_targets` 는 지금처럼 건드리지 않는다.
4. **종료 해제가 그 팔레트 대상을 함께 옮긴다**(Codex 1회차 P2-1). 그 자리는 hold 안의 추정(픽업은 접근 끝 캡처, 목적지는 도킹 판정)이다. 종료 해제(D1 4단계)는 그 미션의 팔레트를 hold 가 찾은 추정으로 보고 목적지와 같이 새 프레임으로 옮긴다. 차와 팔레트의 상대 간격이 해제 뒤에도 같다. 2의 마지막 검사가 그 결과(같은 보정으로 옮긴 대상)를 미리 본다.
5. 운반 중 취소(적재)는 그대로다(6a 의 `cargo_held`).

**시험.**
- CPU: 근거별 준비 장애물(`live_prepare` 와 같은 함수), `prepare_start_ok` 가 계획기의 출발 검사와 같은 판정(같은 외곽·장애물·여유), 취소·복구 끝에서 통과·불통과 분기(첫 검사도 해제 뒤 입력), hold 안에서 통과·해제 뒤 실패, 새 hold 의 지도 팔레트를 옮기지 않음과 기존 hold 의 팔레트를 옮김(실제 해제와 같은 판정), `clear_out` 의 일시정지·재개가 저장 경로·진행량·예산으로 이어짐(방향이 바뀌는 경로에서 진행 순서 보존), `clear_out` 에 허가·금지 접촉 검사가 걸리고 대상 갱신이 멈춤(단계 이름을 열거하는 곳마다 — Codex 4회차 P3), 전진 끝(오도메트리 궤적 되짚기 — 보정 점프가 궤적에 들어가지 않음)·후진 끝(곧은 연장), 물러나기 시작의 hold, 여유 0.08 m 로 끝점 고르기, 1.0 m 안에 없으면 recovery_required, 접지 판정 때의 `put_down`, 해제 뒤 입력(최신 보정·SLAM 지도 격자)의 마지막 검사와 0.05 m 연장, 종료 해제가 팔레트 대상을 옮김(상대 간격 유지), 배치 불변.
- Isaac U7 재실행: 다섯 경우 + 접근 끝 취소 + 복귀 시작 취소에서 다음 미션 `done`.

**구현(2026-10-19).**
- 코어 `finish_cancel` 은 포크가 빈(`free`) 취소 확정과 복구 끝(`recover_withdraw` 도착·남은 경로 없음)에서만 관문을 본다. 적재 취소는 그대로 끝나고 `cargo_held` 를 유지한다(구현 리뷰 1회차 P1-1).
- 관문은 러너 훅 `prepare_start_ok(pose, extra_m, preview, t)` 이다. hold 중이면 `frame = received ∘ applied⁻¹` 로 자세를 옮긴다. 팔레트는 `_held_estimates` 판정대로 옮긴다. 격자는 러너의 `grid_kwargs` scratch 모드로 만든다: 장애물 층 사본에서 해제의 재투영(`refresh(t, received, version+1)`)을 먼저 하고 준비와 같은 호출을 한다. 공유 층은 바뀌지 않는다. 판정은 `leg_obstacles("observe")` + `make_checker.free(…, clearance + extra)` 이다.
- 물러나기 예산은 처음 경로 길이를 쓴다. 연장은 그 길이를 더한다. 재개는 추종기 진행량 뒤의 남은 경로에 지금 자세를 붙이고, 남은 경로보다 늘어난 거리만 더한다(구현 리뷰 2·3회차).
- 계약: 라이브 미션 상태에 `remaining_m` 을 더했다. 프로브 `--cancel-at PHASE@METRES` 로 접근 끝 취소를 건다.
- 시험(CPU): `test_core_recovery`(관문·물러나기·궤적 되짚기·곡률·재개 진행량·합류 예산·hold 해제·적재 취소), `test_core_async_plan`(복구 뒤 종료 해제의 목록 팔레트 이동), `test_pallet_permit`(근거별 준비 장애물), `test_isaac_obstacle_layer`(러너 훅을 AST 로 꺼내 실제 장애물 층에서 미리보기와 실제 해제 뒤 준비 검사를 비교하는 동등성 시험, Codex 반례 회귀).

**U7 2차 결과(2026-10-19, f5dd4e8, ws1 GPU 0·1, 배속 0.4, 직접 실행).** 첫 미션을 취소하고 다음 미션을 끝까지 수행했다.

| 취소 | 첫 미션 단계 | 복구 시작 깊이 / 경로 | 물러나기 | 복구 중 횡간극 최소 | 금지 접촉 | 다음 미션 |
|---|---|---|---|---|---|---|
| 삽입 3 s | insert → recover_lower → recover_withdraw → clear_out | 0.058 / 0.186 m | 0.30 m | 82.0 mm | 0 | P-1→D-1 `done` |
| 들기 2 s | lift → … → clear_out | 0.316 / 0.436 m | 0.30 m | 77.8 mm | 0 | P-1→D-1 `done` |
| 내리기 2 s(3회) | lower → … → clear_out | 0.312 / 0.432 m | 0.30 m | 82.4–83.6 mm | 0 | P-1→D-0: `done` 1회, `failed: near_capture_failed:…:no_opening_pattern` 2회 |
| 철수 1 s | withdraw → … → clear_out | 0.224 / 0.349 m | 0.28 m | 75.6 mm | 0 | P-1→D-0 `done` |
| 철수 5.5 s | withdraw → clear_out(이미 빠져나옴) | — | 0.18 m | — | 0 | P-1→D-0 `done` |
| 접근 끝(0.3 m 남음) | approach, 관문 통과 | — | 없음 | — | 0 | P-1→D-1 `done` |
| 복귀 시작 1 s | return_home, 관문 통과 | — | 없음 | — | 0 | P-1→D-0 `done` |

- 9회 모두 다음 미션의 준비가 시작됐다(1차의 `invalid_start` 없음). D4b 의 목표는 충족했다.
- 내리기 경우의 실패 2회는 다음 미션의 **근접 캡처**에서 났다. 먼 캡처는 세 번 모두 성공했다(참값 오차 8–9 cm). 근접 캡처 지점은 먼 추정으로 옮긴 관찰 지점(팔레트에서 약 3.7 m)이다. 실패한 두 번은 그 지점이 참값 중심에서 옆으로 0.60 m 떨어졌고, 성공한 세 번(내리기 1·철수 2)은 0.51–0.57 m 였다. 먼 추정의 오차가 관찰 지점을 옆으로 밀어 포켓 입구가 덜 보이는 것으로 본다. 취소 뒤 처리와는 무관한 **인식·관찰 지점의 견고성 문제**이고, 이 계획의 범위 밖이라 메인 세션에 넘긴다(미해결).

## 4. 유즈케이스와 시험

시험은 세 층이다:
- CPU 단위 시험: ws1 pytest
- Isaac 스크립트 시험: `tools/live_console_probe.py` 에 시나리오를 추가하고 ws1 에서 돌린다. 판정 스크립트가 통과를 정한다.
- 브라우저 확인: Playwright

판정은 결과 코드만 보지 않는다. 미션마다 참값 배달 오차·접지·포크 이탈을 보고, 실패 뒤에는 **새 미션을 끝까지 수행**해야 통과다.

| U | 유즈케이스 | 통과 조건 | 시험 |
|---|---|---|---|
| U1 | 띄워 두고 30분(벽시계) 대기 후 지시 | 대기 중 sim 이 계속 진행. 미션 성공. 대기 동안 메모리 상한 안(D8) | Isaac `idle_then_submit` |
| U2 | 미션 완료 뒤 세션 유지 | 결과 `done`, 배달 참값 판정 통과, 결과 기록부·재접속 페이지에 남음 | Isaac `cycle` |
| U3 | 연속 미션 P-1→D-1, P-1→D-0, P-1→D-1 | 세 번 모두 참값 판정 통과 | Isaac `cycle` ×3 |
| U4 | 수동으로 픽업 구역 안 다른 곳에 둔 뒤 지시 | 지금 자세에서 계획, 성공 | Isaac `teleop_then_submit` |
| U5 | 차만 홀로 옮김(팔레트는 원래 자리) | 홀 경로로 관찰, 성공 | 같은 시나리오, 다른 자리 |
| U6 | 미션 중 일시정지 → 수동 조종 → 재개 | 재개 성공, 또는 사유와 함께 거절 → 취소 → 재지시 성공 | Isaac `intervene` |
| U7 | 단계별 취소: 관찰·접근·부분 삽입·들기·추출·운반·내리기·철수·정착·복귀·home_settle | 표 D4 의 복구 후 기대 결과. 다음 지시 성공 | Isaac `cancel_each` (단계마다 1회) |
| U8 | 적재 중 취소 → 다른 목적지로 적재 제출. 목적지 접근에서 SLAM hold 가 걸린 뒤 취소하는 경우 포함 | hold 해제, 적재 제출 받아들임, 운반·내리기·철수·복귀 성공 | `cancel_each` 운반 경우 2회 |
| U9 | STOP → 재무장 → 재개 | C3 기준 | C3 시나리오 재실행 |
| U10a | 준비 실패(관찰 경로 불가 자리에서 지시, 또는 `mission_start` 실패 주입) | `failed: prepare:*`, 상태 불변(U21 비교), 다음 지시 성공 | CPU + Isaac |
| U10b | 실행 중 실패(실패 주입: 인식 실패, 추적 실패; 배달 목적지 계획 불가) | `failed` + 사유, 세션 유지, 다음 지시 성공 | CPU 주입 + Isaac 주입 플래그 |
| U11 | LiDAR 에 안 보이는 팔레트로 수동 직진 | 허가가 팔레트 앞에서 멈춤. D10 을 끄면 같은 시험이 실패함 | Isaac `pallet_teleop`(새 시험) |
| U12 | 장면 손상(통제된 주입: 시험 플래그로 팔레트를 넘어뜨림) | 래치, 자동 거절, 수동 허용, 장면 초기화 → 새 epoch 정상 | Isaac `reset` |
| U13 | 끊김·재접속·새로고침·두 브라우저 | C3 기준 | C3 시나리오 재실행 |
| U14 | 세션 종료 | 정상 종료, 감독 루프 끝 | Isaac `end` |
| U15 | 실행기 강제 종료 | 감독 루프가 같은 포트로 재기동, 포트 파일 `crashed`→새 epoch, 재접속 | Isaac `kill` |
| U16 | 2시간 연속(영상 구독, 미션 반복) | D8 상한 안, 페이싱 정상 비율 ≥ 95 % | Isaac `soak` |
| U17 | 복귀 방향 | D11 판정 | `cycle` 기록 |
| U18 | 지도·경로·사유·결과 기록부 표시 | 화면 항목 | Playwright |
| U19 | 제출 거절 사유 | D3 의 각 사유가 거절되고 한국어 사유가 보임 | CPU + Playwright |
| U20 | 수동 승강·복구 조종 | D14 규칙 안에서만 움직임 | Isaac `lift` |
| U21 | 두 번째 미션 시작 직후 미션 소유 상태 | D0 표의 '시작값'과 일치(첫 미션 시작 직후와 비교) | CPU + Isaac 덤프 |
| U22 | 경쟁: 준비 중·채택 순간·캡처 완료 직후·계획 대기 중의 취소/STOP | 유령 미션 없음, 상태 불변 또는 정상 취소 | Isaac `race` |
| U23 | 작업자 사망 | 진행 미션 `failed: worker_dead`, 작업자 교체 후 다음 지시 성공 | CPU + Isaac 주입 |
| U24 | 재시작 직후 옛 요청 재전송 | 옛 epoch 요청 거절 | Isaac `kill` 후 |
| U26 | 들기 직후(검사기 닫힘, 적재 확인 전) 취소 | 포켓 점유 `inside` 로 판단해 복구, 결과 기록 | Isaac `cancel_each` |
| U27 | 철수 초반·후반 취소 | 포켓 점유로 분기(초반 복구, 후반 `cancelled`) | Isaac `cancel_each` |
| U28 | 종료용 SLAM 해제 | 해제 뒤 대기 작업·경로 설치·재관찰 예약 0건 | CPU |
| U29 | 기록 소유권과 비용 | 두 미션 연속 뒤 각 미션 기록이 자기 자료만 담음. 큐 가득 참·쓰기 실패가 결과 기록부에 표시. 최대 조각·가득 찬 큐에서 종료 틱의 기록 비용 ≤ 20 ms, 틱 간격·STOP 응답이 평소 분포 안 | CPU + Isaac |
| U30 | 근접 추적 중 STOP 뒤 시뮬 25분 대기 → 취소 → 새 미션 | 대기 중 RSS 증가 < 100 MB, 미션 기록의 `frames_overflow` > 0, 새 미션 `done`(D8) | Isaac `stop_wait` |

배치 회귀: 재생 게이트 29/29, ws1 전체 pytest(알려진 실패 목록 외 0).

## 5. 순서

1. 이 계획의 교차검토가 수렴할 때까지 고친다. 사용자 확인 항목(7절)을 받는다.
2. 메인과 고정 SHA 로 경계를 합의한다:
   - D9 의 예외 경계와 정리 API(`abandon_jobs`)
   - 작업자 `mission_start` 작업
   - 계약 추가: `preparing`·`recovering`·`last_result`·`missions`·`scene`·`session_end`·`sim_reset`·대상 근거
3. D0 상태 소유권 표를 쓰고 Codex 로 검토한다.
4. 메인 D9 머지 뒤 이 브랜치를 main 위로 올린다.
5. 구현한다. 시험을 먼저 쓴다. 순서:
   1. 결과 기록부·`end_mission`(배달 판정 함수 이동 포함)
   2. 준비·채택(D2·D3)
   3. 실패 분류·두 경계·장면 손상 래치(D6)
   4. 감독 루프·세션 수명(D7·D9)
   5. 자원(D8)
   6. 복구·화물(D4·D5)
   7. 팔레트 허가·수동 승강(D10·D14)
   8. turnaround 비동기·배속(D11·D12)
   9. 화면(D13)
6. 단계마다 Codex 구현 검토를 수렴까지 받는다.
7. Isaac 유즈케이스를 돌린다. 재생 게이트와 전체 pytest 를 돌린다.
8. PR 을 올린다.

## 6. 교차검토 기록

**① 원자료 재확인(Claude)**
- 확인한 것:
  - `complete` 에서 루프 종료(R6075)
  - `check_submit` 이 `available` 아닌 팔레트 거절(targets.py 114)
  - 집기 전 팔레트 밀림 검사가 `initial_pallet` 기준(R5841)
  - `live_replanned` 래치(C464·475)
  - 루프 상한 `max_sim_seconds`(R5602)
  - `_drop` 의 release 재예약(C4363)
  - 브리지 `accept` 1회(225)
  - `--output` `exist_ok=False`(R6733)
  - `depth_support` 가 포켓 검사기의 bound method(R3278)
  - `observe_replan_leg` 시작 오프셋 예산(C1688)
  - 배달 팔레트 자리 = 목적지 위치(`refresh_targets` C2914–2920)
- 반박 없음.

**② Codex 적대적 검토(1회차)**
판정은 "구현 전 수정 필요". 지적과 처리:

| 등급 | 지적 | 처리 |
|---|---|---|
| P1 | 루프 전체 catch 는 틱 부분 실행·두 번 스텝·`tick_end` 누락 | D6 두 경계 + 공통 꼬리 |
| P1 | 참값 실패 반복 → 살아 있지만 조작 불가; 모든 `require` 를 미션 실패로 바꾸면 안 됨 | 장면 손상 래치 + 분류표(기본 세션 실패) |
| P1 | D2 에 허가·SLAM hold·gain·콜백 해제 누락, `_drop` 부작용, `observe_replan_leg` | D0 표, D1 2–5단계, P5 |
| P1 | 취소 정리를 기존 단계로 못 함 | D4 복구 상태 기계 |
| P1 | 적재가 단계로 결정됨, 운반 재계획은 한 구간뿐 | D5 cargo 상태, 적재 제출 |
| P2 | 관찰 bounds 가 시작점에도 걸림, 후보가 절대 좌표·방향 0, 준비 실패의 원자성 | D2 일반화·경계 분리·준비/채택 |
| P2 | 완료 검증 생략, terminal 결과 소실 | D1 6·9단계, 결과 기록부 |
| P2 | 유휴·긴 미션 메모리, 브리지 누락 | D8 |
| P2 | 감독 루프의 출력 디렉터리·브리지 수명·포트 파일 | D9 |
| P2 | turnaround 동기 20 s, 준비 동기 | D11 작업자 먼저, D2 작업자 |
| P2 | 팔레트 허가 캐시·근거, U11 이 기능을 시험하지 않음 | D10, U11 새 시험 |
| P3 | U16·U17 통과 조건 없음 | D8 상한, D11 판정 |

추가 시험(경쟁, 작업자 사망, 옛 요청, 단계 취소 확대)은 U7·U21–U24 에 넣었다.

**③ 독립 검토(agy, Gemini 3.8 Flash High, 1판만 봄)**
지적과 처리:
- SLAM 드리프트 누적: D3 판정을 참값으로, 제출 조건에 tracking 확인. 재위치 추정은 범위 밖으로 기록.
- 복구 후진의 안전 전제: D4 허가·실패 시 `recovery_required`.
- 내려놓은 팔레트 자리 가정: D3 인식으로 다시 찾음, 참값 판정.
- 엔진 내부 누수: U16 VRAM 측정, D9 재기동.
- 단순 이동 미션 누락: 3판에서 범위 제외(별도 계획).
- 영상 지연: 기존 `stale_video` 해제와 픽셀 지연 상한이 있음(C3·M-c). 반영 없음.
- 준비 중 경쟁: U22.
- D4 가 D14 에 의존한다는 지적: 오독. 미션 자신의 승강 ramp 로 내린다. 반영 없음.
- 순서(D10 을 수동 개입 시험 전에): 5절 순서상 수동 개입 시험은 구현이 끝난 뒤.
- 키 삭제와 제자리 초기화의 충돌: D0 표에서 키별로 정함.
- U1 대기가 sim 시간으로 진행되는지: U1 조건에 명시.
- 실패 뒤 재지시 미검증: 모든 실패 유즈케이스에 '다음 지시 성공'.
- U16 영상 구독: D12.
- 재생 게이트 착시: 0절에 명시.

**메인 세션 의견(D9)**: 정리 순서, 유지 항목, `slam["records"]`, `SessionFailure`(53eba4c)를 D1·D0·D8·D6 에 반영했다.

**1판 → 2판**: 위 표대로 D1·D2·D4·D5·D6·D8·D9·D10·D11 을 다시 썼다. D0 과 U21–U25 를 새로 넣었다.

**Codex 2회차(2판 검토)**: 판정 "미수렴, P1 1·P2 4". 1판 지적들은 계획 수준에서 대응됐다고 확인.

| 등급 | 지적 | 처리(3판) |
|---|---|---|
| P1 | 포켓 점유·부분 적재를 검사기 수명·"들기 성공"으로 판단 → `insert→lift` 사이 취소에서 포크가 안인데 밖으로 봄; 철수 초반 취소 충돌 | D5 물리 상태(포켓 점유 3값, 적재 5값), D1 1단계 스냅숏, D4 표를 물리 상태로 분기, U26·U27 |
| P2 | `truth_destination` 을 픽업 자리로 덮어씀(평가·도킹 참조 오류) | D2 참값 분리와 목적지 세 값 대응 |
| P2 | 종료 중 SLAM 해제 함수가 새 작업·경로를 만듦 | D1 4단계 사후조건, U28 |
| P2 | 비동기 기록과 제자리 초기화의 소유권 | D1 7단계 직렬화 이전·큐 상한·실패 표시, U29 |
| P2 | 단순 이동 미션이 공통 규칙과 안 맞음 | 이번 범위에서 제외(별도 계획) |
| P3 | U10a 준비 실패 시험 방법 | U10a/U10b 재정의 |

**2판 → 3판**: 위 표대로 D1·D2·D3·D4·D5 와 U10·U26–U29 를 고쳤다.

**Codex 3회차(3판 검토)**: 판정 "미수렴, 새 P1 없음, 새 P2 2". 2회차 지적은 계획 수준에서 해결됐다고 확인.

| 등급 | 지적 | 처리(4판) |
|---|---|---|
| P2 | 포켓 점유 `inside` 가 적재 운반 중에도 참 → 운반 중 hold 뒤 취소하면 해제도 적재 제출도 막힘 | D5 포크 관계 4값(`free/constrained/carrying/unknown`), D1 4단계와 D4 표를 포크 관계로, U8 에 hold 뒤 취소 추가 |
| P2 | 종료 틱 직렬화가 제어 루프를 무제한 점유 | D1 7단계: 진행 중 조각 확정, 종료 틱은 결과·manifest 64 KB·20 ms 상한, 큐 상한을 바이트로, U29 에 틱 간격·STOP 응답 |

**3판 → 4판**: 위 표대로 D1·D4·D5 와 U8·U29 를 고쳤다.

**Codex 4회차(4판 검토)**: "새 P1/P2 없음, 계획 단계에서 수렴". 3회차 P2 두 건은 계획 수준에서 해소됐다. 남은 것은 합의대로 D0/API 와 정확한 catch 경계의 별도 검토다. 그때 실제 코드에서 확인할 것:
- 물리 상태 판정의 불확실성 처리
- 다음 틱으로 넘긴 기록의 초기화 이후 소유권
- 공통 꼬리가 정확히 한 번 실행되는지

**D0 상태 소유권 표 검토(Codex, 새 스레드)**: `docs/design/2026-10-18-live-mission-state.md`, 기준 8452120.

| 회차 | 판정 | 지적과 처리 |
|---|---|---|
| 1회차 | P2 6 · P3 3 | `release_replan_needed` 는 `None`(dict|None) / 기록 전 열린 구간·감시 닫기(9절 순서) / 조각은 닫힌 항목만(`near.corrections`·`stop_events`·`_versions`) / `options.pickup_zone` / `known` 은 `known_enabled` 조건 / 요청 원장 포화 → D7 / P3: 누락 항목·`lateral_clearance_min_m` 분리·`near_tracking` 필드·memory 근거 정정·작업자 폐기 ID 회수 |
| 2회차 | P2 2 | `finish()` 가 판정 전에 terminal 로 확정 → `close_open` 과 `terminate` 분리 / 원장을 지운 뒤 옛 요청을 구별할 근거 없음 → 다시 쓰지 않는 요청 토큰·토큰별 순번·워터마크 |
| 3회차 | 새 P1/P2 없음 | 계획 수준에서 수렴. 구현 뒤 API·원장 시험과 U3·U21·U29 로 확인할 것 |

### 구현 검토 기록 (Codex, 단계별 수렴)

| 단계 | 커밋 | 회차 | 지적과 처리 |
|---|---|---|---|
| 1 결과 기록부·`end_mission` | cc90aca → 0c4a08d | 3 | P2 첫 구간 재계획 실패가 결과를 남기지 않음 → `end_mission`. P3 확정 취소의 종료 로그·종료 사유 표시 → `note_end`, 상태 메시지 우선순위. P3 같은 apply 의 취소+제출 → 끝난 틱의 제출 거절. 2회차에 함께 본 `observe_ladder` P2(deadline 중복) → 한 번만 전달. 3회차 새 P1/P2 없음 |
| 2 설계 | 5d6c666 → 6bd8657 | 3 | 1회차 P1 2(준비 중 대상 갱신, 포켓 안 취소 뒤 재제출)·P2 6·P3 3, 2회차 P1(끝난 틱의 대상 갱신)·P2(job_hold 실패 수거)·P3(wizard.js). 3회차 새 P1/P2 없음 |
| 2 구현 | 03a656f·6aa0026·a923f87 → 94aa7b2 → 9803b9c | 3 | 1회차 P1 2(접근 중 취소가 포켓 검사기·면제·SLAM hold 를 넘김; 배달된 팔레트 재제출 불가)·P2 4(사전 영역 보정 중복, 커스프 감시기 목록 끊김, 기록 소유 ID, 도킹 참조 cast 순서)·P3(세션 끝의 사유). 2회차 P2(종료 틱의 대상 되돌림) → 종료 때 대상 확정·`adopted_id` 해제, 화면 `selectable`. 3회차 새 P1/P2 없음 |
| 3 설계 | 3a6c603 → 348555d | 6 | 1회차 P1 2(적재 단계 종료 뒤 수동 조종이 떨어진 팔레트를 차체로 봄 → D5 전까지 수동 조종 거절; NaN 쿼터니언·캡처 계약 사유 → 세션)·P2 5(종료용 SLAM 해제와 예약 정리, 작업자 고장, 삽입 거절, 승강 고정, 미션 없을 때의 2D 겹침), 2회차 P2 2(미션 없을 때 3D 접촉 검사, 대상 재표현), 3회차 P2 2(정상 지지 접촉, 지도 목표 이동), 4회차 P2(도킹 목적지), 5회차 P2 2(hold 동안 얻은 추정만 옮김) → 구현 검토로 넘김 |
| 3 구현 | 7042d19·6095336·b14facd → 1f7cbcf → 706a533 → e147b1e | 4 | 1회차 P2 3(비유한 검사 위치, 복귀 단계 팔레트 사각형, 차체·캐리지 검사), 2회차 P2(바퀴), 3회차 P2(조향 바퀴를 실측 조향각으로). 4회차 새 P1/P2 없음 |
| 4a 세션 수명 | 5699642 → 364fd66 이전 | 2 | P2 2(정리 실패가 0/3 에 가려짐, 종료 승인 뒤 같은 틱의 주행·제출). 2회차 새 P1/P2 없음 |
| 4b 감독 루프 | 4c04a52·a5fe1d9 → 364fd66 | 3 | 1회차 P2 5(TERM 전달, 실제 충돌이 종료 파일에 가려짐, 시작 실패, 같은 원인 규칙, sbatch 종료 상태)·P3(경로 이스케이프), 2회차 P2(대기 중 TERM)·P3(정리 실패 기록). 3회차 새 P1/P2 없음. Isaac U15(kill -9 → crashed → epoch 2)·U14(session_end → ended) 통과 |
| 4c 설계 | 09a83d1 → 2판 | 2 | P2 5(토큰 소유, 전체 상한, 옛 토큰 stale 의 재접속 연쇄, 연결별 welcome, PENDING 키)·P3 2. 2회차 새 P1/P2 없음 |
| 4c 구현 | 823c98c → f8a4416 | 2 | P2 2(닫힌 연결 토큰 회수, 재기동 뒤 토큰 재사용)·P3 2(등록 거절의 연결 거절, 가짜 차량 — 후속). 2회차 새 P1/P2 없음 |
| 5 설계·구현(D8) | 80fc9ff → 3판 | 3 | 1회차 P2 5(종료 꼬리 유실, 쓰는 중 시간 초과의 계수, 근접 기록 미회수, 영상 기록 경로, 합격 기준)·P3 3. 2회차 P2 4(STOP 중 근접 프레임, 바이트 추정 과소, 읽는 쪽 전환 일부, 로봇 카메라 없는 영상 종료 예외)·P3 3. 3회차 새 P1/P2 없음, P3 2(복사본 누락 감지, U30 대기 시간) 반영 |
| 5b 브리지(D8b) | d57e0da → 4판 | 4 | 1회차 P2(지도 콜백이 /pose 를 막음, 기록이 sendall 앞)·P3 2(번호·시각 계약, 소비자·디스크 설명). 2회차 P2(같은 스레드의 동기 기록이 다음 응답을 막음)·P3. 3회차 P2 2(쓰는 중 줄 누락 집계, 기록 전 상태 공개·컨테이너 즉시 제거). 4회차 새 P1/P2 없음, P3(문구) 반영 |
| 6a 설계(D5a) | 5611c13 → 4판 | 4 | 1회차 P1(base 좌표의 날을 뒤축에 놓음)·P2 7(여유 근거, `pallet_rel` 없음, 정책·제출·해제·재표현·낙하 무효화)·P3. 2회차 P1 2(free 고정, 철수 재앵커링)·P2 4 → 기하 증명을 버리고 전이 기반 세션 상태로. 3회차 P2 2(무효화를 관측 지점에, 원인 분리). 4회차 새 P1/P2 없음 |
| 6a 구현 | 4판 → 커밋 | 2 | 1회차 P2(지연 해제 직전 재검증)·P3(거절 키 라이브 한정). 2회차 새 P1/P2/P3 없음. ws1 전체 pytest 4549 통과(알려진 10 실패), 재생 게이트 29/29 |
| D10 설계 | c500c19 → 4판 | 4 | 1회차 P1(캐시 키에 방향 없음)·P2 4(여유 근거, 겹친 뒤의 판정식, 외곽·번들 여유, 비용). 2회차 P1 2(키운 사각형 안의 이탈은 안전 근거 없음, 보정 변경 무효화)·P2 2(경계 표본은 포함을 못 봄, 적재 외곽 지렛팔). 3회차 P1(사전 구역에 차량 자세 오차 없음)·P3. 4회차 새 P1/P2 없음 |
| D10 구현 | 4판 → 커밋 | 2 | 1회차 P3 2(패드·캐시 시험이 변이를 못 잡음). 2회차 새 P1/P2 없음. 재생 게이트 29/29, ws1 전체 pytest 4561 통과(알려진 10 실패), U11 켬/끔 |
| 6b 설계(D4) | 682d239 → 6판 | 6 | 1회차 P1 3(이동량 합, 지금 방향 직선, 새 단계에 허가 없음)·P2 4·P3. 2회차 P1(목적지 끝점)·P2 2(축 직선, 내리기 지지 접촉). 3회차 P2 2(궤적 재배치 이중 반영, 상대 방위 무검사)·P3. 4회차 P2 2(기준 시점, 기록 밖 구간). 5회차 P2 2(정상 철수가 이탈로 판정, 픽업 무효 허용). 6회차 새 P1/P2 없음 |
| 6b 구현 | 6판 → 커밋 | 3 | 1회차 P1 3(대상 목록 미반영, 관찰 추종기 설정, 재개 재검증 없음)·P2 5·P3 3. 2회차 P1(설치 뒤 근거 무효에도 후진)·P2(재개 대기 중 반복 설치)·P3. 3회차 새 P1/P2 없음, P3(준비 단계 취소 버튼) 반영. 재생 게이트 29/29 |

## 7. 사용자 결정 (2026-10-18)

| 질문 | 결정 | 계획에 반영 |
|---|---|---|
| 복귀 정책 | **대기 자리로** | 미션이 끝나면 고정된 대기 자리로 간다. 기본은 세션 시작 자세(시나리오 `start_rear`)이고, `--live-wait-pose x,y,yaw` 로 바꿀 수 있다. `options.return_to_pose`·`home_rear` 를 미션마다 이 값으로 둔다(D2). |
| 복귀 목표 방향을 풀어 줄지 | **금지** | 목표 방향은 그대로다. 후진을 줄이는 것은 D11(전진 선호 turnaround) 몫이다. |
| 계약·중재기 변경 | **허용** | 계약 추가(D1·D2·D6·D7·D9·D10)와 중재기 변경(D10·D14)을 진행한다. 메인 세션과 기술 합의는 그대로 거친다. |
| 장면 손상 뒤 수동 조종 | **허용** | D6 그대로. |
| 단계 우선순위 변경 | **없음** | 5절 순서 그대로. |
