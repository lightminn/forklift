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

## 7. 사용자 결정 (2026-10-18)

| 질문 | 결정 | 계획에 반영 |
|---|---|---|
| 복귀 정책 | **대기 자리로** | 미션이 끝나면 고정된 대기 자리로 간다. 기본은 세션 시작 자세(시나리오 `start_rear`)이고, `--live-wait-pose x,y,yaw` 로 바꿀 수 있다. `options.return_to_pose`·`home_rear` 를 미션마다 이 값으로 둔다(D2). |
| 복귀 목표 방향을 풀어 줄지 | **금지** | 목표 방향은 그대로다. 후진을 줄이는 것은 D11(전진 선호 turnaround) 몫이다. |
| 계약·중재기 변경 | **허용** | 계약 추가(D1·D2·D6·D7·D9·D10)와 중재기 변경(D10·D14)을 진행한다. 메인 세션과 기술 합의는 그대로 거친다. |
| 장면 손상 뒤 수동 조종 | **허용** | D6 그대로. |
| 단계 우선순위 변경 | **없음** | 5절 순서 그대로. |
