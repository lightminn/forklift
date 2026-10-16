# M-a3 미션 명령 구현 (계획 1판, 2026-10-16)

[M-a 계획](2026-10-15-m-a-mission-core.md) D5 와 [M-a0.5 경계 설계](2026-10-15-m-a05-boundary.md) 1 부 2–3 절·2 부 8–9 절(재개 표 R1–R15, 실패 사유)을 코드로 옮긴다. 입력은 [M-a0 목록](2026-10-15-m-a0-inventory.md) 1 절(시계 분류 W·WT·F·M·B·R). 기준 revision 은 `99727c1`(M-a2 끝)이고 줄 번호는 그 판의 `src/forklift_core/mission/core.py`(이하 C)와 `sim/isaac/run_transport.py`(이하 RT)다.

## 1. 범위

- 포함: `mission_submit`·`mission_pause`·`mission_resume`·`mission_cancel` 과 작업 생명주기, 정지 종류 `none`·`controlled`·`preempt` 의 물리 틱 적용, 일시정지 틱(출력 0·조향 유지·승강 동기화·근접 보호 계속), 작업 시간(W) 정책, 재개 절차(R1–R15), 취소 종료, 배치 실행기의 명령 일정 입력(`--mission-commands`), 미션 안전 상태 출력(D6, M-b 입력).
- 제외: TELEOP 보류 `held`·`hold_on/off`(M-b 와 함께), 비동기 포트의 늦은 결과 폐기와 준비·확정 분리(M-c 전 별건, 계획 D5), 대상·목적지 목록과 revision(M-a4), 근접 bounds 를 일시정지 실행에 적용하는 조건 확장(계획 6 절 열린 질문 — 그때까지 근접 추적과 명령 일정은 함께 쓰지 않는다).
- **기본 경로 불변**: 명령 일정이 없으면(배치 실행기의 자동 `submit` 하나뿐) 모든 출력·기록이 지금과 같아야 한다. 증거는 M-a1 재생 29 건 재통과와 ws1 전체 시험(알려진 실패 10 건과 같은 집합), 그리고 명령 없는 폐루프 A/B(같은 seed, `tools/mission_ab_compare.py`).

## 2. 결정

**E1 명령 경계(순수 모듈).** `forklift_core/mission/commands.py` 에 명령(`Submit(pallet_id, destination_id, targets_revision)`, `Pause`, `Resume`, `Cancel(mission_id)`)과 생명주기 상태기계를 둔다: `idle → accepted → running ⇄ paused → resuming → running`, 종료 `done`·`failed`·`cancelled` → `idle`. 활성 임무는 하나, `mission_id` 는 코어가 발급(세션 안 단조 증가 문자열). 거절: 활성 중 제출 `mission_active`, 빈 차체·근접 구간 밖이 아닐 때 제출 `recovery_required`, `preempt`·`controlled` 동안 `resume`·`submit` 은 `stopping`. 같은 틱 여러 명령은 `cancel > pause > resume·submit`(M-a0.5 3 절). `mission_gen`(정수)은 정지 경계·취소·재개 완료마다 1 증가하고 안전 상태(E8)에만 싣는다 — 재생 비교 대상 컨테이너에 넣지 않는다. 같은 틱에 종료 전이와 `submit` 이 겹치면 종료가 먼저 확정되고(`idle`), 그 뒤 큐의 `submit` 을 판정한다. 같은 틱 명령은 도착 순서가 아니라 우선순위로 하나씩 적용하고, 앞선 명령으로 무의미해진 명령은 거절 사유와 함께 기록한다(예: `cancel` 뒤 `resume` → `not_running`). 이 모듈은 Isaac·numpy 없이 시험한다.

**E2 적용 시점.** 명령 일정 파일의 시각은 시뮬레이션 시각 `t`(루프 시각, 벽시계 아님)이고, 명령은 `t` 가 그 시각 이상이 된 첫 제어 틱에 큐에 들어간다. 명령 큐는 제어 틱 시작(RT 의 `tick_begin` 직후, release gate 전)에 비운다. 그 틱 이전에 끝난 동기 연산(캡처·계획·release + 재계획)의 결과는 유효하다(계획 D5). 기록기의 틱 머리 `commands` 와 `stop_kind`(지금 항상 `[]`·`"none"`)에 실제 값을 싣는다. 명령 일정이 없으면 자동 `submit` 은 틱 머리에 싣지 않는다(실행기 시작 때 루프 밖에서 한 번) — 그래서 기준 기록과 틱 머리가 같다. 재생 하네스는 틱 머리를 비교하므로(`test_the_tick_heads_commands_are_compared`) 이 동일성이 재생 게이트로 확인된다.

**E3 일시정지 틱.** `paused`·`resuming` 동안 실행기는 미션 하위 단계(`follow`·`docking_and_stall`·`cusp_and_observe_recovery`·`command`·도착·`lift_and_timers`·허가 뒤 재계획·`live_leg`)를 부르지 않고 `core.paused_tick(t, rear, ...)` 를 부른다. 결과:
- 요청 속도 0, 조향은 그때의 `steering_command` 를 유지(dwell 경로와 같은 `dwell_hold` — 바퀴는 유지 조향의 곡률로 제동, C 2584).
- 명령 조립(`assemble_command`)은 그대로 돈다 — slew·허가 상한·비상정지가 지금처럼 걸리고, `applied_command`·`command_speed` 가 실제 출력과 같게 남는다.
- 승강: 승강 목표를 매 틱 **실측 승강 위치로 동기화**(목표 동결이 아니다, 계획 D5). 실측은 시뮬레이터 관절 위치라 잡음이 없다; 실차의 승강 센서 잡음은 R7 허용치를 정할 때(M-b·실차 준비 게이트) 다룬다.
- 근접 보호: approach·insert 에서 근접 추적기가 있으면 `near_tick` 을 요청 속도 0 으로 계속 부른다(관측 전달·`status()`·보호 판정; M-a0.5 2 부 7 절 2 — 이 두 단계 밖에서는 프레임을 받지 않는다). 다만 근접 추적과 명령 일정은 1 절 제외 때문에 이번 판에서 함께 켜지지 않으므로, 이 경로는 CPU 시험으로만 확인한다.
- WT 카운터(M-a0 1 절 8–12·15·16·34)는 하위 단계를 부르지 않으므로 늘지 않는다. 추종기 `update` 도 부르지 않는다(진행 탐색·slew 동결, M-a0.5 2 절).
- 근접 보호의 정지(근접 추적 켬)와 허가·비상정지의 정지는 지금처럼 **미션 안의 정지**(hold·속도 상한)이고 명령 상태기계를 거치지 않는다 — 생명주기는 `running` 그대로다. 일시정지는 조종자·중재기의 명령에서만 생긴다.
- F 항목(SLAM 보정 나이, 정지 감지, 격자 FREE 증거, 센서 침묵, 근접 관측 나이)은 `step_world` 와 서비스 안에서 지금처럼 흐른다.

**E4 작업 시간 = 구간 길이 − 그 구간 안의 일시정지.** M-a0.5 2 절은 `work_t = tick_t − 누적 일시정지` 를 W 항목에 쓰자고 했다. 그대로 하면 W 의 기준 시각(`ms.phase_started`, `release_wait_from`, `since_s`, `forced_s`, 재계획 기록의 `time_s`)을 `work_t` 로 저장해야 하고, 그 값들은 기록에도 들어가므로 기록의 시각 의미가 섞인다. 대신 **기준 시각은 지금처럼 `t` 로 저장하고, '시작 시각' 형 W 비교만 `self.work_elapsed(since, now)` = `(now − since) − (since..now 안의 일시정지 합)` 으로 한다.** 일시정지 구간이 하나도 없으면 `work_elapsed` 는 뺄셈 없이 `now − since` 를 그대로 돌려준다(빈 목록 합의 형·길이 0 구간에 기대지 않는다) — 그래서 기본 경로의 W 판정이 지금과 비트 단위로 같다. **'마감 시각' 형**(`live_retry_after_s = t + 1.0` 을 저장하고 `t >= after` 로 비교, C 431·491)은 같은 식으로 바꾸면 `(after − 1.0)` 의 반올림 때문에 경계에서 비트 단위로 같지 않을 수 있으므로, 재개 완료 때 그 마감에 이번 일시정지 길이를 더한다(일시정지가 없으면 더하지 않는다; 여러 번 일시정지하면 매번 그 길이만큼 더해져 누적된다 — 시험으로 고정). 일시정지 구간 목록은 명령 상태 객체(E1)가 가지고 `MissionState`·`state`·`obstacle` 같은 재생 비교 대상 딕셔너리에는 넣지 않는다 — 기준 기록의 끝 상태 투영(`final_projection`, RT 1764 — `PROJECTION_CONTAINERS` 의 키별 digest 와 `MissionState` 필드 값)에 새 키·필드가 생기지 않게. 바꿀 W 비교(C, `99727c1`, ①에서 다시 셈):

| W 항목(M-a0 #) | 위치 |
|---|---|
| 추종 시간 초과(2) | C 1120·1122 |
| SLAM release 전 정지 대기 10 s(4) | C 2555 |
| lift·lower 4.5 s(5) | C 379·382 |
| settle·home_settle 1 s(6) | C 391·397 |
| observe 느슨한 도착 1 s(7) | C 1769–1770 |
| 후퇴 끝 강제 재계획 1 s(13) | C 1808 |
| 진행 없음 60 s(17) | C 733 |
| 재계획 빈도 창 30 s(19) | C 1082 |
| live 계획 재시도 1 s(20) | C 431·491 (마감 형 — 재개 때 이동) |
| 포켓 정지 0 유지 5 s(14) | C 665–666 |

M-a0 의 RT 줄 번호는 M-a1 이동 전 판이라 위 표는 이번 판에서 다시 세어 닫는다(①검토 항목). W 이지만 실행기에 남은 것 — 비상정지 탐침 지연(21), 예정 장애물 주입(52), dwell·safety stop 일정(51) — 은 시험 주입·기록용이라, 명령 일정과 함께 켜면 실행 시작 전에 거부한다(`--record-pocket-frames`, `--measure-safety-stops-m`, `--near-tracking`, `--estop-probe*` 와 `--mission-commands` 는 함께 쓰지 않음). 예정 장애물 주입은 세계 사건이라 W 가 아니라 그대로 둔다(일시정지 중에도 나타날 수 있다 — 그것이 재개 경로 재검사의 시험이 된다).

**E5 재개 절차.** `resume` 은 `resuming` 으로 들어가 매 틱 단계를 하나씩 확인한다: `stop_confirmed`(정지 감지 `stop_now` — SLAM 없으면 참값 속도 < 0.01 m/s, 지금 판정과 같은 기준; 10 s 안에 안 되면 `not_stopped_timeout`) → `state_verified` → `frame_reexpressed` → `path_rechecked`. 단계 결과와 실패 사유는 M-a0.5 9 절 이름을 쓴다. 행별 동작은 M-a0.5 8 절 표를 따르되, 이번 판에서 구현하는 범위:

| 행 | 이번 판 |
|---|---|
| R1·R2b·R10 (자유 추종) | 재결합 검사(현재 자세가 추종기의 남은 경로에서 그 추종기 설정의 `max_cross_track_error_m` 안 — 관찰·근접 추종기는 0.35 m, C 1943·2151, 나머지는 기본 0.65 m, `path_tracking.py:37`) → 안 되면 그 단계의 기존 재계획 흐름으로 현재 자세에서 재계획(관찰: 관찰 다리 재계획, 운반·복귀: 장애물 재계획 경로); 재투영은 보정이 바뀌었을 때만 기존 `refresh` 로. 허가 재검사는 재개 뒤 첫 틱의 허가 단계가 그대로 한다(막히면 기존 장애물 대응 — 횟수 한도 그대로). |
| R2a (`retry_stand`) | 서서 다시 캡처(오래 멈췄으면 증거가 낡았으므로) |
| R3 (observe hold 중) | hold episode 가 같으면 근접 캡처를 처음부터; 다르면 `hold_episode_changed` |
| R3x (hold 중 재정렬) | `no_resume` |
| R4·R4n (approach·insert) | hold episode 확인 → 추종 계속(근접 추적 켬은 1 절 제외로 이번 판에 실행 경로 없음 — CPU 시험만) |
| R5·R6 | 근접 추적 켬 전용 — 이번 판은 CPU 시험으로 결과 이름(`pickup_retry`, `recovery_only`)만 고정 |
| R7 (lift·lower) | 승강 실측이 동기화 목표 근방(허용치는 ①에서 승강 slew·실측 잡음으로 정함) → 실측에서 이어서; 4.5 s 는 E4 로 자동 연장 |
| R8 (extract) | 같은 후진 직선 계속 |
| R9·R14 (release 대기) | 기존 흐름(release 는 정지 확인 때) |
| R11 (도킹) | 일시정지 중 이동(일시정지 때 자세 대비 정지 판정 허용치 초과)이 없으면 직선 계속; 있으면 D7 켬일 때 `start_docking_retry`, D7 끔이면 `no_resume` |
| R12 (재접근) | 재결합 검사 → 재접근 계획 |
| R13 (withdraw) | 같은 축으로 계속 |
| R15 (home_settle) | 남은 시간 계속 |

`no_resume`·`recovery_only` 는 `paused` 에 머물고 사유를 안전 상태에 낸다(조종자가 취소하거나 복구를 고른다). 재개 완료 때 추종기 속도 slew 를 실측 속도에서 다시 시작한다(`restart_speed_slew`). 표에 없는 조합은 `no_resume`.

**E6 취소.** `cancel` 은 일시정지 틱과 같은 출력으로 정지를 확인한 뒤 `cancelled` 로 끝낸다(적재·삽입 상태는 안전 상태에 그대로). 배치 실행기는 루프를 끝내고 결과에 `cancelled` 를 별도로 남긴다 — 지금 `complete` 외 종료를 실패로 보던 종료부를 바꾼다(실패 사유 문자열은 `mission_cancelled`).

**E7 정지 종류(물리 틱).** 중재기 대신 명령 일정이 시간 구간별 정지 종류를 준다. `step_world` 의 `world.step()` 직전에: `preempt` 면 바퀴 속도 목표 0·조향 유지를 그 물리 틱에 바로 적용, `controlled` 면 정지 모델 감속(`permission.config.stopping.decel_mps2`)으로 낮춘다. 적용한 값으로 `applied_command`·`command_speed`·정지 감지의 `last_command` 를 맞춘다(M-a0.5 3 절). 정지 종류가 `none` 이 아닌 동안 미션은 `paused` 와 같은 틱을 돈다(재개는 정지 종류가 `none` 으로 돌아온 뒤 `resume` 으로만 — `stopping` 거절). 캡처처럼 코어 호출 안에서 물리가 도는 동안에도 `preempt` 가 걸리는지 CPU 시험(가짜 물리)과 Isaac 시험으로 본다. 시작 뒤 모든 물리 틱이 `step_world` 를 지난다는 것은 기존 시험(`test_every_physics_step_after_start_goes_through_step_world`, R 로 유지)이 지킨다 — 정지 종류 적용을 `step_world` 한 곳에 두면 빠지는 물리 틱이 없다. `controlled` 감속 중에는 조향을 유지하므로 차체가 지금 곡률의 호를 따라 선다(추종 경로의 곡률과 같다); 일시정지 뒤 경로 이탈은 재개의 재결합 검사(E5)가 맡는다.

**E8 안전 상태 출력.** M-a0.5 4 절 형식을 매 제어 틱 코어가 만들고 기록 `Sink` 로 낸다(`mission`·`load`·`fork_in_pocket`·`near`·`frame`·`command`·`blocked`; `basis` 에 근거). 명령 일정이 없을 때도 내지만 기존 산출물에 섞지 않고 별도 파일(`mission_safety.jsonl`)로 쓴다 — 기본 경로 A/B 는 그 파일을 비교 대상에서 뺀다(새 산출물). 재생 기록에는 `Sink` 효과로 들어가므로 M-a1 기록(이 출력이 없던 기준)과의 재생 비교에서 빠지는 방식을 정한다(①에서 하네스의 Sink 관찰 목록으로 확인).

## 3. 조각과 판정

| 조각 | 내용 | 판정 |
|---|---|---|
| M-a3a | E1 명령·생명주기 모듈, E2 큐 적용, 실행기 `--mission-commands` 일정 파일(시각 → 명령·정지 종류), 근접 계약 `POST_MEASUREMENT_ARGUMENTS` 에 `None` 등록, 함께 못 쓰는 인자 거부, 배치 자동 `submit` | 모듈 CPU 시험(우선순위·거절·mission_id·세대), 인자 검증 시험, 기본 경로 불변 |
| M-a3b | E3 일시정지 틱, E4 작업 시간, E6 취소 | 가짜 포트 CPU 시험: 단계 묶음(관찰·approach·lift·transport·withdraw·settle)마다 일시정지 → 출력 0·조향 유지·승강 동기화·WT 정지·W 연장; 동기 연산(캡처·계획) 중 들어온 정지가 다음 틱에 반영; 취소 종료 |
| M-a3c | E5 재개 절차(R1–R15 의 이번 판 범위) | 행마다 CPU 시험(재결합 실패 → 재계획, 도킹 중 이동 → 재시도/`no_resume`, hold episode 변경, 승강 불일치) |
| M-a3d | E7 정지 종류 물리 틱 적용 | CPU(가짜 `step_world`) + Isaac 시험 |
| M-a3e | E8 안전 상태 | 형식 시험, 단계별 값 시험 |
| 통합 | Isaac(ws1 Slurm): 관찰·운반·들기 중 일시정지/재개, 운반 중 취소, 운반 중 preempt(근접 추적 끔) | 각 실행이 끝까지 가거나(일시정지 뒤 complete) 정의된 결과(`cancelled`)로 끝남; 기본 경로 재생 29 건·ws1 전체 시험·명령 없는 A/B 동일 |

각 조각은 별도 커밋이고, production 을 바꾸므로 조각마다 M-a1 재생 29 건(`docs/validation/2026-10-16-m-a1-replay-manifest.json`)과 ws1 전체 시험을 다시 통과시킨다(M-a2 계획 2.6 의 게이트와 같은 이유). Codex 리뷰는 조각마다 수렴까지.

## 4. 위험

- **기본 경로의 미세한 차이**: E4 는 일시정지가 없을 때 `0.0` 을 빼는 것이라 같다고 보지만, `work_elapsed` 를 부르는 순서·예외 경로가 바뀌면 다르다. 재생이 잡는다.
- **재생 하네스의 새 출력**: E8·E2 가 기록 형식(틱 머리 `commands`·`stop_kind`, 새 Sink 효과)을 바꾼다. 기준 기록(형식 2)은 그 값이 `[]`·`"none"` 이고 새 Sink 효과가 없으므로, 하네스가 새 효과를 '기준에 없는 효과' 로 거절하지 않게 범위를 정해야 한다(M-a3e 착수 때).
- **규모**: 계획 4 절의 M-a3 2–3 일보다 크다(재개 표 15 행). 행별 Isaac 검증은 하지 않고 CPU 시험으로 덮고, Isaac 은 통합 4 실행으로 한정한다.

## 5. 열린 질문

- 일시정지 중 예정 장애물이 경로에 나타나면 재개 뒤 첫 허가 틱이 막고 기존 장애물 대응으로 간다 — 재개 절차의 `path_rechecked` 를 허가 질의로 따로 둘지(실패 사유 `path_blocked`), 기존 흐름에 맡길지. 1판은 맡긴다.
- 승강 동기화 허용치(R7)와 '일시정지 중 이동' 허용치(R11)의 값.

## 6. 자체 검토

- **① 직접 재계산(원본 `99727c1`)**: W 비교 위치를 core.py 에서 다시 셌다 — 추종 시간 초과 1120·1122, release 대기 2555, lift·lower 379·382, settle 391·397, 느슨한 도착 1769–1770, 후퇴 강제 재계획 1808, 진행 없음 733, 재계획 창 1082, 포켓 정지 0 유지 665–666, live 재시도 431·491. 고친 것 두 가지: live 재시도는 '시작 시각' 이 아니라 '마감 시각' 형이라 같은 식으로 바꾸면 경계에서 비트 단위로 같지 않을 수 있다(재개 때 마감을 미는 방식으로 바꿈); 재결합 한도는 M-a0.5 의 '0.35 m' 가 아니라 추종기별 `max_cross_track_error_m`(관찰·근접 0.35, 나머지 기본 0.65 — C 1943·2151, `path_tracking.py:37`). 재생 끝 상태 투영(`final_projection`, RT 1764)이 `PROJECTION_CONTAINERS` 의 키와 `MissionState` 필드를 비교하는 것을 확인해, 명령 상태를 그 밖에 두기로 했다.
- **③ agy 독립 검토(Claude Opus 4.6 (Thinking), 계획 본문만 보여 줌)**: 적히지 않은 전제 13 건. 반영 — 같은 틱 종료와 `submit`(E1), 같은 틱 명령 충돌 쌍의 처리와 거절 기록(E1), 빈 일시정지 목록에서 뺄셈 자체를 하지 않음(E4), 여러 번 일시정지의 마감 누적(E4), 정지 종류 적용이 모든 물리 틱을 덮는 근거(E7), `controlled` 감속 중 조향 유지의 의미(E7), 틱 머리 기본값과 재생 비교(E2), `mission_gen` 을 재생 비교 컨테이너 밖에(E1), 일정 시각 = 시뮬레이션 시각(E2), 근접 보호 정지는 명령 경로가 아님(E3), 승강 실측 잡음(E3). 반영하지 않음 — 경사면 일시정지 Isaac 시험(장면에 경사가 없고, 정지 확인 실패는 `not_stopped_timeout` 으로 정의된 결과다), 일시정지 중 0.65 m 넘게 밀리는 Isaac 시험(외력 주입 수단이 없다; 재결합 실패 → 재계획은 CPU 시험으로 덮는다).
- **② Codex 적대 검토**: (진행 예정)
