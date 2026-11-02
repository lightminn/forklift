# 확인 전 위치 조각 ② 구현 계획 (2026-11-02, 2 판)

설계 정본은 `docs/plans/2026-11-02-unconfirmed-position.md`(8 판, PR #250)다. 이 문서는 그 1.4·1.5 를 러너·중재기·콘솔에 **어디에 어떻게** 배선할지만 정한다. 조각 ①(PR #251)의 `forklift_core.localization.twin_check` 를 쓴다. 2 판은 1 판의 세 검토(7 절)를 반영한다. 설계를 좁힌 곳은 0 절 끝에 따로 적는다.

## 0. 원칙

- **러너 기본은 끔**(`--pose-check off`). 켜야만 새 경로가 돈다.
  - 끔에서는 기록도 기존과 같다. 새 인자는 끔 기본값일 때 `recorded_arguments` 에서 뺀다.
  - 옛 setup 스냅숏을 복원하는 재생에서는 새 인자가 없으면 끔으로 묶는다.
  - 재생 게이트 runs29 29/29 와 끔 실행의 기록 비교를 각 러너 PR 에서 확인한다.
  - 운영 런처 기본 전환은 ③ 검증 뒤에 따로 한다.
- 순수 코어(시험 먼저)와 배선을 나눈다. 판정 규칙은 코어에 두고, 러너는 입력을 모아 넘기고 결과를 적용·기록한다.
- **좌표 기준**: 상태기와 `twin_check` 의 모든 자세 계산은 base_link 와 base_link 오도메트리(`slam_odom_base()`)로 한다. 보정 C 는 map←odom 이다. 뒤 차축은 허가에 넘길 때만 우곱 오프셋으로 바꾼다. 뒤 차축 자세로 C 를 만들거나 `moved` 에 뒤 차축 오도메트리를 넣지 않는다.
- **개발 값은 켬의 전제다.** `--pose-check on` 은 δp(d)·δθ(d)(오도메트리 한계), E_STILL, E_SLAM 을 모두 받아야 시작한다. 없으면 `reloc_config_error` 로 시작하지 않는다. 이 값들은 ③ 개발에서 **끔 실행**(곧은 후진 탐침, 정지 구간, 확정 직후)으로 잰다. 그래서 E_STILL 없는 감시나 δ 없는 후진 같은 반쪽 상태가 없다.
- **설계를 좁힌 곳**(총괄 결정 범위 안의 규칙, 보고 대상):
  - 확인 뒤 다시 정합은 미션이 돌지 않을 때(AUTO 가 아니고 진행 중 작업이 없을 때)만 시작한다. 중재기의 AUTO 출력은 정책을 보지 않으므로(`arbiter.py` `step` 의 AUTO 분기), 미션을 멈춰 세우는 배선 대신 시작 조건을 좁힌다. 미션 중에는 Δ_int 0.20 m·0.05 rad 회수 문만 지킨다. 설계가 근접 구간에서 이미 그렇게 한 것을 미션 전체로 넓힌 셈이다.
  - 이 좁힘으로 다시 정합이 덜 일어나므로, 바른 세션의 회수 수가 쓸모 판정(설계 4 절)에서 하한을 넘는지 ③ 에서 본다.

## 1. 조각 나눔

| PR | 내용 | 재생 게이트 |
|---|---|---|
| ②a | 순수 코어: 상태기(`PoseCheck`), 합집합 구역, 누름 상한, 정책·중재기의 거절과 끝 잠금, 실패 분류 | 필요(중재기·정책은 러너가 매 틱 부른다) |
| ②b | 러너 배선: 출발 검사, 상태 적용, 다시 정합 작업자, SLAM 일치·감시, 회수, 기록, 인자·기록 호환, 감독자 | 필요 |
| ②c | 콘솔: 띠·잠김 표시·가설 구역 그리기·거절 문구, 계약 문서 | 필요(발행은 러너) |

## 2. ②a 순수 코어

### 2.1 `twin_check` 추가

- `last_leaves(grid, points, cfg, index, r_resid, *, clock, deadline, cancel) -> (status, leaves)`.
  - 그 스캔 묶음에 대한 한 번의 전역 격자 검색이다. 최고점의 R_RESID 이상인 억제 전 잎(base 자세)과 그 검색의 status 를 돌려준다.
  - 마감·취소를 `_Budget` 으로 끝까지 넘긴다. 설정의 `deadline_s` 만으로는 마감이 걸리지 않기 때문이다(Codex 재현).
  - 비용 도구의 `pool_and_leaves` 는 이것을 부르도록 바꾼다. 동결 정합기는 건드리지 않는다.
- `start_check(grid, points, b, pool, cfg, *, e_min, index, cons, clock, deadline, cancel) -> StartCheck`.
  - 순서는 `last_leaves` → `synthetic_twins(b)` → `hypotheses` → `residual_ambiguity(hyp_status=...)`. 한 마감을 모두가 나눠 쓴다.
  - `StartCheck` 는 `Residual` 과, 불가능으로 빠진 것을 뺀 가설 목록(겨룸·계산 실패 포함)을 싣는다.
- `union_zones(zones, hyps_now, c_base, d, bound) -> list[Zone]`: 설계 1.4 의 c∘h⁻¹∘z 합집합.
  - `hyps_now` 는 오도메트리로 옮긴 가설 자세 h_t 와 b_t 이고, `c_base` 는 같은 틱의 적용 자세다. 둘 다 base_link 다.
  - 각 구역의 꼭짓점 p 를 c∘h_t⁻¹∘p 로 옮긴다.
  - 그 구역을 (0.30 + δp(d) + 2d·sin 0.05) + 2ρ_max·sin((0.10 + δθ(d))/2) 만큼 바깥으로 키운다. ρ_max 는 옮긴 꼭짓점과 c 의 base_link 사이 거리의 최댓값이다.
    - 영역(±0.30 m, ±0.10 rad)은 base_link 둘레라서 ρ 도 base_link 에서 잰다.
    - 키움은 ρ 에 단조라 꼭짓점의 최댓값이 구역 전체를 덮는다.
  - 입력 `zones` 는 `zones_from` 의 결과 그대로다. 그 안의 기존 여유(사전 자세 0.182 m, 인식 0.21 m, 외곽 0.01 m)는 팔레트 자세 자체의 불확실성이다. 가설의 불확실성과는 다른 것이라 빼지 않는다. 이 보수적 겹침이 뒤 막힘을 얼마나 늘리는지는 ③ 에서 잰다.
- `rear_clear_for_check(...)`: 설계 3 절의 출발 가능 판정이다.
  - 실제 텔레옵 허가 경로(러너의 `live_permit`, 곧은 후진의 곡률 묶음 `TeleopDrive` 가 넘기는 그대로, 그 허가의 검사 길이 그대로)로 잰다.
  - 구역은 d = D_MIN + 정지 거리(0.568 m)에서 키운 합집합이다. d 가 자라며 구역이 커지는 양을 미리 넣는다.
  - 통과는 "D_MIN 에 이를 때까지 허가 속도가 0 이 되지 않음"이다. 자세는 그 틱의 적용 자세를 인자로 받는다(`live_state["rear"]` 를 읽지 않는다).

### 2.2 상태기 `PoseCheck` (새 모듈 `src/forklift_core/localization/pose_check.py`)

순수하다. 입력은 그 틱의 시각, base 오도메트리, 적용 보정 C, 정지 확인, deadman, 미션 활동, 근접 구간이다. 매 틱 상태와 할 일을 돌려준다.

- **상태**: `confirmed` | `unconfirmed` | `checking` | `ended:<reason>`.
  - `checking` 은 다시 정합을 계산하는 동안이고, 움직이지 않는다.
  - `ended` 는 되돌릴 수 없다.
- **출발**: `PoseCheck.from_start(check, b_refined, o0, source)`.
  - 남은 모호성이 없으면 `confirmed` 후보가 되고, 아래 SLAM 일치를 기다린다. 그 전에는 `unconfirmed` 와 같이 잠긴다.
  - 남은 모호성이 있으면 `unconfirmed`.
  - 알려진 시작은 상태기를 만들지 않는다(설계 1.2, 범위 밖).
- **누름**(`unconfirmed`):
  - `ready`: 정지 확인, deadman 놓음, 남은 예산 있음. 이 상태도 정책에서는 "후진 가능"이다. TELEOP 진입·arm 이 막히지 않게 하려는 것이다(Codex P2-4).
  - `active`: deadman 이 눌린 첫 틱에 열린다. 그때의 오도메트리 O_s 를 잡는다. 이동 s 는 **순변위** |O_t − O_s| 다. 서 있는 동안 바퀴 잡음이 쌓이는 경로 합이 아니다(agy 9).
  - 남은 거리 = min(D_STROKE − s, D_TOTAL − 앞선 누름들의 순변위 합 − s). 이 값이 정지 거리 이하가 되면 속도 0 을 요구하고 `closed` 로 간다(Codex P2-3).
  - `closed` 에서 정지 확인과 deadman 놓음이 오면 `ready`(예산이 남았을 때) 또는 `spent` 로 간다.
- **다시 정합 요청**(확인 전): 다음을 모두 채우면 `want_rematch` 를 돌려준다. 러너가 작업자를 띄우면 `checking` 이다.
  - 출발점에서의 순변위 d ≥ D_MIN.
  - 정지 확인, deadman 놓음.
  - 이번 정지에서 아직 하지 않음.
- **다시 정합 결과**는 `rematch_decision` 으로 판정한다.
  - `confirm` → SLAM 일치로.
  - `twin_preferred` / `out_of_set` → `ended:pose_twin_preferred` / `ended:pose_out_of_set`.
  - `unconfirmed`·`too_short` → `unconfirmed`(누름 `ready`).
  - 결과를 버리는 경우: 근거 묶음의 마지막 스캔 뒤 오도메트리가 0.002 m 또는 0.002 rad 넘게 움직였거나 epoch 가 바뀌었을 때. 확인 전 다시 정합은 control_gen 변화로 버리지 않는다. deadman 을 놓고 패킷이 끊겨 생긴 선점이 정상 결과를 매번 지우는 일을 막으려는 것이다(agy 2). 움직임은 오도메트리 검사가 막는다.
- **SLAM 일치**(모든 승격): 같은 틱의 적용 자세 c = C_now∘O_now 와 근거 자세 r_now 를 비교한다.
  - r_now 는 근거 자세를 근거 시각에서 지금까지 오도메트리로 옮긴 것이다. 근거 시각은 출발 확인이면 확정 스캔 묶음의 마지막 스캔, 다시 정합이면 그 묶음의 마지막 스캔이다.
  - E_SLAM 안이면 `confirmed`. 그 틱에 **C_int = r_scan ∘ O_scan⁻¹** 를 잡는다. 근거 자세와 그 시각의 오도메트리를 같이 쓰고, 전파한 자세와 과거 오도메트리를 섞지 않는다(Codex P2-5).
  - E_SLAM 밖이면 `ended:pose_slam_disagree`.
  - 출발 확인의 이 검사는 SLAM 이 처음 tracking 에 든 틱에 한다. 지금의 `bootstrap_check` 자리이고, 그 0.10 m 검사는 그대로 둔다.
- **확인 뒤 감시**: 매 틱 Δ_int = C_now∘O_t 와 C_int∘O_t 의 차(base_link 위치, 방향)를 잰다.
  - Δ_int 가 0.20 m 또는 0.05 rad 를 넘으면 `ended:pose_revoked:int`.
  - 정지 확인이 시작될 때마다 C_still 을 잡는다. 서 있는 동안 Δ_still 이 E_STILL 을 넘으면 `ended:pose_revoked:still`.
  - **확인 뒤 다시 정합**은 다음을 모두 채우면 `want_rematch` 다(0 절의 좁힘).
    - 정지 확인, 근접 구간 밖, 미션 비활동.
    - Δ_int > 0.10 m.
  - 결과 계약은 설계 1.5 그대로다.
    - 확정이고, 보수 판정을 통과하고, 최신 c 와 g(같은 기준·같은 시각)가 E_SLAM 안이면 C_int 를 새로 잡는다.
    - 확정·통과인데 E_SLAM 밖이면 `ended:pose_revoked:rematch`.
    - 그 밖은 증거가 아니라서 C_int 를 그대로 둔다.
    - 결과는 epoch·control_gen·스캔 시각에 묶는다(설계 1.5).
- 판정과 근거(자세, 보정, Δ, d, 누름 상태)는 상태기의 사건 목록에 남는다.

### 2.3 정책·중재기

- **`teleop_policy(..., pose=None)`**: `pose` 는 `{"status", "drive_ready"}` 이다. `None`(끔)이면 지금과 같다.
  - `unconfirmed`: 다른 거절을 먼저 본다. 그다음 `drive_ready`(누름이 `ready`·`active`, 출발 가능 판정 통과)이면 `Policy(True, False, None, drive_mode="reverse_straight")` 다. 아니면 거절한다.
    - 거절 이유: `pose_unconfirmed:stroke`(누름 `closed`), `pose_unconfirmed:total`(`spent`).
    - 포크는 두 방향 모두 막는다.
  - `checking` → `pose_checking`, `ended:*` → `pose_ended`. 둘 다 운전·포크 모두 거절한다.
  - `confirmed` 는 지금과 같다.
- **곧은 후진**: 기존 `reverse_straight` 를 그대로 쓴다(0.10 m/s = V_REV_U, 조향 허용오차 0.01 rad 밖이면 0). 목표 곡률이 0 이므로 조향은 0 으로 모인다(기존 복구와 같은 경로).
- **누름 상한**: 남은 거리가 정지 거리 이하인 틱은 상태기가 `closed` 로 가므로 정책이 거절하고, 중재기가 `drive_allowed=False` 로 같은 틱에 속도 0 을 낸다(Codex 재현: 같은 틱 0).
- **중재기 거절**(`_run`): `pose_status != "confirmed"` 이면 다음을 거절한다(`RELOC_REFUSED` 와 같은 자리).
  - 대상: `mission_submit`·`mission_resume`·`pick_request`·`recovery`·`sim_reset`, AUTO 로의 `mode_request`.
  - 이유: `pose_unconfirmed`, 또는 `ended` 이면 `pose_ended`.
- **끝 잠금**: `arbiter.end_pose(now, reason)`.
  - `stop(now, reason)` 와 `_boundary` 의 효과를 낸다.
  - `pose_ended` 잠금을 건다. 잠금 동안은 stop 과 상태 조회 말고 모두(`rearm`·`arm`·`mode_request`·`session_end`·`session_reset` 포함) `pose_ended` 로 거절한다.
  - 운전자의 세션 끝 요청이 위치 실패를 정상 끝(종료 코드 0)으로 바꾸지 않게 하려는 것이다(agy 6).
- 진행 중인 들기 램프:
  - `unconfirmed` 에서는 `lift_dirs` 가 비므로 기존 `TeleopLift` 가 멈춘다. 출발 직후라 미션 들기는 없다.
  - `ended` 에서는 `stop` 이 TELEOP 를 떠나게 하고, 러너가 들기를 유지한다.
  - 둘 다 시험으로 고정한다.

### 2.4 실패 분류

- `pose_rear_blocked`·`pose_twin_preferred`·`pose_out_of_set`·`pose_slam_disagree`·`pose_revoked` 를 `failures.PREFIXES` 에 SESSION 으로 등록한다(`strict` 계약 시험 포함).
- 감독자 `STOP_AT_ONCE` 에 `pose_` 접두를 넣는다. 같은 출발점에서 다시 띄우면 같은 결과가 되기 때문이다. 운전자가 차를 옮긴 뒤 다시 시작한다(agy 10).

## 3. ②b 러너 배선

### 3.1 인자·기록

- 인자 넷을 둔다.
  - `--pose-check off|on`(기본 off).
  - `--pose-check-odom-bound JSON`: δp(d) = a + b·d, δθ(d) = c + e·d.
  - `--pose-check-e-still JSON`, `--pose-check-e-slam JSON`.
- `on` 은 `--start-relocalize`, SLAM 지도 묶음, 세 값을 모두 요구한다.
- `near_tracking.POST_MEASUREMENT_ARGUMENTS` 에 끔 기본값을 등록하고, `recorded_arguments` 에서 끔 기본값을 뺀다. 옛 setup 복원은 끔으로 묶는다(Codex P2-6).
- 시험은 인자 등록 시험, 끔 실행의 기록 키 비교, 옛 setup 재생이다.

### 3.2 출발 검사 (`reloc_phase`)

- `chosen` 뒤, `write_reloc_pose` 전에 작업자 스레드에서 `start_check` 를 돌린다. 차는 서서 스텝만 진행한다.
- 입력:
  - b: 추정의 base. 정밀화 자세는 `start_check` 가 국소 정합으로 다시 구한다.
  - 점: **고른 세대**의 확정 스캔 묶음이다(Codex P2-9).
    - 무인 확정이면 확정한 시도의 묶음.
    - 운전자 고름이면 그 세대 마지막 시도의 묶음.
    - 운전자 놓기면 그 놓기의 묶음.
    - 시도·놓기 기록에 세대 번호를 넣고, 각 묶음의 점과 마지막 스캔 시각의 오도메트리를 메모리에 둔다.
  - 풀: 고른 세대의 시도 후보 전부(중단된 시도 포함, base 자세).
  - 마감: 검사 시작에서 30 s. 하나의 마감을 잎 검색·국소 정합·영역 검색이 나눠 쓴다. 넘으면 `search:timeout` 으로 `unconfirmed` 이고, 세션을 끝내지 않는다.
- 결과(요약과 가설 목록)를 `out["pose_check"]` 에 싣는다.
- 감독자의 시작 대기(`isaac_live.sbatch` 운전자 540 s·전체 600 s)는 켬일 때 30 s 늘린다. 런처가 `POSE_CHECK=on` 에서 같이 넘긴다(Codex P2-7).

### 3.3 라이브 루프

- **시작**: SLAM 이 처음 tracking 에 든 틱(`bootstrap_check` 자리)에 다음을 한다.
  - `PoseCheck.from_start`.
  - `unconfirmed` 이면 `rear_clear_for_check`. 막히면 `pose_rear_blocked`.
- **틱 안의 순서**(Codex P1-1, agy 1). 기존 순서는 `tick_start`(요청·미션 명령 적용) → SLAM 응답 적용 → 계획·추종 → `tick_end` 다. 상태기는 세 곳에서 본다.
  1. `tick_start` 앞: 지난 틱 끝의 상태를 중재기(`arbiter.pose_status`)와 `Context` 에 넣는다. `Context.localization_ok` 는 `confirmed` 일 때만 참이다. `resume` 도 이 검사를 거치게 `MissionCommands._resume` 에 `localization_lost` 거절을 더한다(Codex 재현: 지금은 `_submit` 에만 있다).
  2. SLAM 응답 적용 바로 뒤, 미션 작업·계획 결과 채택 앞: 새 보정으로 감시(Δ_int·Δ_still)와 SLAM 일치를 판정한다. 여기서 `ended` 가 나오면 그 틱의 미션 작업·계획 채택을 건너뛰고, `arbiter.end_pose` 를 부르고, 바퀴 0 과 들기 유지로 간다.
  3. `tick_end` 앞(최종 출력 앞): 같은 틱의 보류 해제(`manual_recovery_tick` 의 자세 변경)까지 반영한 적용 자세로 2 를 한 번 더 한다.
- **허가**(`live_permit`): 상태가 `confirmed` 가 아니면 팔레트 구역을 `union_zones` 로 바꾼다.
  - 캐시 열쇠는 (현재 h_t 전체의 판본, 적용 보정, d 의 0.05 m 칸)이다.
  - 구역은 그 칸의 위 끝(d + 0.05 m)에서 키운다. 칸 안에서 d 가 자라는 동안에도 구역이 작지 않다(agy 5). 칸 단위 캐시라 매 틱 다시 계산하지 않는다.
- **다시 정합 작업자**: 별도 프로세스 하나를 라이브 시작 때 띄운다.
  - 지도 묶음 경로로 격자·`MapIndex`·`ConservativeIndex` 를 그 안에서 만든다.
  - 계산은 GIL 밖에서 돌아 제어 틱과 SLAM 폴링을 밀지 않는다(agy 7). 죽으면 다시 띄우고, 다시 못 띄우면 `plan_worker_dead` 와 같은 SESSION 실패다.
  - 요청은 한 세대다(설계 1.5): 시도 3 번, 30 s 마감, `StartRelocalizer` 의 결정, 무인.
    - 시도마다 정지 확인 동안 SLAM 링크로 보낸 새 스캔 5 개(`scan_capture.SCANS`)를 모은다.
    - 그 5 개 동안 오도메트리 이동이 0.002 m·0.002 rad 이하인 묶음만 쓴다. 출발 캡처의 정지 검사와 같은 한계이고, 넘으면 그 묶음을 버리고 다시 모은다(Codex P2-5).
    - 각 묶음의 마지막 스캔 시각과 그때의 오도메트리를 함께 넘긴다.
  - 세대가 무인 확정으로 끝나면 그 g 와 묶음으로 `rematch_decision`(또는 확인 뒤 계약)을 작업자 안에서 돌린다. 아니면 마지막 이유로 `unconfirmed` 다.
  - 마감은 벽시계 30 s 다. 시뮬레이션 배속이 느리면 스캔을 모으는 데 쓰는 시간도 마감에 든다. 이 시간은 ③ 에서 잰다.
- **끝**(`ended:*`):
  - 그 틱에 `arbiter.end_pose`, 바퀴 0, 들기 유지.
  - 정지 확인(또는 2 s 뒤)에 `SessionFailure(<reason>)` 를 올린다. 이 검사는 루프 맨 위의 `session_request` 확인보다 먼저 한다(agy 6).

### 3.4 기록

- `state["pose_check"]` 에 출발 검사, 사건 목록, 다시 정합마다의 입력·결과·시간, 끝 사유를 남긴다.
- ③ 판정용 기록:
  - 매 제어 틱의 상태와, 전진·미션 권한이 열렸는지를 남긴다.
  - 적용 자세와 참값의 오차(base_link)를 틱마다 계산해 F1 4.0 분류별 개수와 최댓값을 센다. 원자료는 10 Hz 로 남긴다.
  - 확인 틱과 첫 권한 틱은 따로 표시한다.
  - **모든 C_int 갱신마다** 근거 자세의 참값 대비 오차와 분류를 따로 남긴다(설계 4 절, Codex P3).

## 4. ②c 콘솔

- 발행: `msg["pose_check"] = {"status", "reason", "stroke": {"state", "left_m", "total_left_m"}, "zones": [...]}`. 끔이면 필드가 없다.
- 지도 그림: 합집합 구역 c∘h⁻¹∘z 는 "가설 h 가 참일 때 팔레트가 차에 대해 있을 자리"를 지금의 추정 c 둘레에 놓은 것이다. 지도 위의 실제 팔레트 자리가 아니다. 그래서 따로 `가설 구역` 으로 이름 붙이고, 차 기준으로 읽도록 범례를 단다(agy 10).
- 띠: "확인 전 위치 · 전진·포크 잠김 · 뒤쪽 낮은 물체는 보이지 않습니다 · 차 뒤를 직접 확인하세요". 끝 사유별 문구는 설계 1.4·1.5 의 것이다.
- `REJECT` 에 `pose_unconfirmed`·`pose_unconfirmed:stroke`·`pose_unconfirmed:total`·`pose_checking`·`pose_ended` 를 넣는다.
- 계약 문서 `docs/interfaces/2026-10-13-operator-console-contract-draft.md` 5·6 절을 고친다.

## 5. 시험 (먼저)

- ②a 상태기 — 전이 표 전체:
  - 출발: 확인·확인 전, SLAM 일치 통과·실패.
  - 누름: ready·active·closed·spent. 부분 누름 넷으로 D_TOTAL 을 넘지 않음. 서 있는 동안의 잡음이 s 를 늘리지 않음.
  - 다시 정합: 다섯 결과. 움직임·epoch 로 버림. control_gen 만 바뀌면 확인 전은 유지, 확인 뒤는 버림.
  - 감시: Δ_int·Δ_still 회수, 확인 뒤 다시 정합 세 갈래, 미션 중에는 시작하지 않음.
  - C_int 식: 0.3 m 움직인 뒤 잡아도 Δ_int 가 0.
- ②a 합집합 구역:
  - 가설 하나에서 c∘h⁻¹∘z 가 손으로 계산한 자리와 같음.
  - 키움 양(설계의 0.65 m 예, Codex 독립 계산 0.649854 m).
  - c 가 바뀌어도 차 기준 구역이 그대로.
  - base_link 와 뒤 차축을 섞으면 회전 때 어긋나는 반례가 실패함.
- ②a 정책·중재기: 확인 전 거절 목록, ready 에서 TELEOP 진입·arm, 곧은 후진만, 포크 막힘, 끝 잠금(rearm·session_end 거절), resume 의 `localization_lost`.
- ②b:
  - 끔의 재생 게이트, 끔 실행의 기록 비교, 옛 setup 재생.
  - 켬의 러너 단위 시험(가짜 엔진·가짜 SLAM): 출발 검사 → 확인 전 → 누름 → 다시 정합 → 확인/끝. SLAM 응답 뒤 틱 안 회수가 그 틱의 출력을 0 으로 함.
  - 작업자 프로세스의 죽음·재시작.
- 전체 묶음은 ws1 에서, 머지 전 최종 트리로.

## 6. 남는 것 (③ 으로)

- δp(d)·δθ(d)·E_STILL·E_SLAM 의 개발 측정(끔 실행)과 동결, Isaac 탐침, 판정(설계 4 절).
- 작업자 프로세스의 제어 틱 영향(배속·`pacer`), 다시 정합의 벽시계 시간, 0 절 좁힘이 바른 세션 회수 수에 주는 영향.
- 운영 런처의 기본 전환.

## 7. 자체 검토

### 1 판 검토

- **① 직접 재계산**(원자료):
  - 정지 거리 0.1×0.15 + 0.1²/(2×1.5) + 0.05 = 0.0683 m(`config/obstacle_layer_single_team_spec.yaml` latency 0.15, decel 1.5, margin 0.05).
  - V_REV_U = `TeleopLimits.recovery_reverse_mps` 0.10, 조향 허용오차 0.01 rad(`teleop_drive.py` 62·63).
  - 스캔 묶음 5 개(`scan_capture.SCANS`), 재위치 마감 30 s·시도 3 번(`RelocConfig`). 정합기 설정은 `max_work` 하나뿐(`match_config.json`).
  - 이 재계산에서 1 판의 다시 정합이 "한 번의 `relocalize`"로 적혀 설계의 "한 세대(시도 3 번)"와 다른 것을 찾아 고쳤다.
- **② Codex 적대적 검토**(새 스레드): P1 2, P2 7, P3 3. 처리는 다음과 같다.
  - P1-1 틱 순서: 감시를 SLAM 적용 뒤와 최종 출력 앞에서도 본다. resume 에도 `localization_lost` 를 둔다(3.3).
  - P1-2 AUTO 는 정책을 보지 않는다: 소스로 확인했다(`arbiter.py` `step`). 확인 뒤 다시 정합을 미션 비활동 때로 좁혔다(0 절).
  - P2-3 D_TOTAL 을 누름 제동에 넣었다.
  - P2-4 누름 `ready` 를 정책상 후진 가능으로 둬 TELEOP 진입 순환을 풀었다.
  - P2-5 묶음 안 정지 검사(0.002 m·0.002 rad)를 두고, C_int 를 r_scan∘O_scan⁻¹ 로 고정했다.
  - P2-6 기록 호환: 끔 기본값을 기록에서 빼고, 옛 setup 은 끔으로 묶는다.
  - P2-7 마감을 잎 검색까지 넘기고, 감독자 대기를 늘린다.
  - P2-8 E_STILL 없는 감시 끔: 설계 변경이 맞다. 켬의 전제로 바꿨다(0 절).
  - P2-9 시도 기록에 세대를 넣고 고른 세대만 쓴다.
  - P3 셋도 반영했다: 출발 판정의 자세는 인자로 받고, 캐시 열쇠는 h_t 판본으로 하고, C_int 근거 자세를 분류해 기록한다.
- **③ agy(Gemini 4 Argon) 독립 검토**(① ② 를 보여주지 않음, "적히지 않은 전제"): 10 건. 처리는 다음과 같다.
  - 1 틱 순서: Codex P1-1 과 같다.
  - 2 deadman 을 놓은 뒤 선점이 control_gen 을 올린다: 확인 전 다시 정합은 control_gen 으로 버리지 않는다. 움직임은 오도메트리로 막는다.
  - 3 base_link 와 뒤 차축의 비가환: 좌표 기준을 0 절에 고정했다.
  - 4 출발 판정이 실제 허가와 다르다: 실제 허가 경로와 d 키움으로 잰다.
  - 5 구역 여유가 겹친다: 다른 불확실성이라 유지하고 ③ 에서 비용을 잰다. 캐시는 0.05 m 칸의 위 끝으로 정했다.
  - 6 끝 대기 동안 `ended` 분기가 빠지고 세션 끝이 정상 끝으로 둔갑한다: 끝 잠금을 두고, 루프 맨 위 순서를 바꿨다.
  - 7 GIL·벽시계: 작업자를 별도 프로세스로 바꿨다. 배속 영향은 ③ 에서 잰다.
  - 8 조향이 처음에 어긋난 경우: 목표 곡률 0 으로 모이는 기존 복구 경로와 같다. 모이지 않으면 기존 `recovery:steering` 이유로 움직이지 않는다. 남은 위험으로 둔다.
  - 9 경로 합이 s 를 늘린다: 순변위로 바꿨다.
  - 10 가설 구역 그림이 헷갈린다, 감독자가 같은 출발에서 되풀이한다: 범례를 달고 `STOP_AT_ONCE` 에 넣었다.
  - `D_TOTAL = 2.0 m` 라고 적은 곳은 agy 의 오기다. 설계값은 3.0 m 다.
