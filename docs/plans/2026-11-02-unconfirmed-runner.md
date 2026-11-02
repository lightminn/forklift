# 확인 전 위치 조각 ② 구현 계획 (2026-11-02, 1 판)

설계 정본은 `docs/plans/2026-11-02-unconfirmed-position.md`(8 판, PR #250)다. 이 문서는 그 1.4·1.5 를 러너·중재기·콘솔에 **어디에 어떻게** 배선할지만 정한다. 설계의 값과 규칙을 바꾸지 않는다. 조각 ①(PR #251)의 `forklift_core.localization.twin_check` 를 쓴다.

## 0. 원칙

- **러너 기본은 끔**(`--pose-check off`). 켜야만 새 경로가 돈다. 끔에서는 기존 동작과 바이트 단위로 같다. 재생 게이트 runs29 29/29 를 각 러너 PR 에서 확인한다. 운영 런처 기본 전환은 ③ 검증 뒤에 따로 한다.
- 순수 코어(시험 먼저)와 배선을 나눈다. 판정 규칙은 모두 코어에 두고, 러너는 입력을 모아 넘기고 결과를 적용·기록만 한다.
- 개발에서 재는 값(δp(d)·δθ(d), E_STILL, E_SLAM)은 인자로 받는다. **δ 가 없으면 확인 전 후진은 0**(설계 1.4)이다. E_STILL 이 없으면 정지 기준 감시는 꺼지고 기록에 그렇게 남는다. E_SLAM 은 설계의 잠정값(0.05 m, 0.025 rad)을 기본으로 둔다.

## 1. 조각 나눔

| PR | 내용 | 러너 변경 | 재생 게이트 |
|---|---|---|---|
| ②a | 순수 코어: 상태기(`PoseCheck`), 합집합 구역, 누름 상한, 정책·중재기의 거절, 실패 분류 | 없음 | 필요(중재기·정책은 러너가 매 틱 부른다) |
| ②b | 러너 배선: 출발 검사, 상태 적용, 다시 정합 작업자, SLAM 일치·감시, 회수, 기록, 인자 등록 | 있음 | 필요 |
| ②c | 콘솔: 띠·잠김 표시·합집합 구역 그리기·거절 문구, 계약 문서 | 발행 필드만 | 필요(발행은 러너) |

## 2. ②a 순수 코어

### 2.1 `twin_check` 추가 (`src/forklift_core/localization/twin_check.py`)

- `last_leaves(grid, points, cfg, index, r_resid) -> (status, leaves)`: 그 스캔 묶음에 대한 한 번의 전역 격자 검색에서 최고점의 R_RESID 이상인 억제 전 잎(base 자세)과 그 검색의 status. 지금 비용 도구의 `pool_and_leaves` 를 옮긴다(도구는 이것을 부른다). 동결 정합기(`global_match`·`relocalize`)는 건드리지 않는다. 그래서 잎은 확정한 시도와 같은 스캔 묶음에 대한 **다시 검색**이다(같은 입력·같은 설정이므로 같은 잎이다).
- `start_check(grid, points, b, pool, cfg, *, e_min, index, cons, clock, deadline) -> Residual`: `last_leaves` + `synthetic_twins(b)` + `hypotheses` + `residual_ambiguity(hyp_status=...)`. 출발 검사의 한 진입점이다. 반환한 `Residual` 에 가설 목록 `hyps` 를 함께 싣는다(합집합 구역과 다시 정합이 쓴다).
- `union_zones(zones, hyps_now, c, d, bound) -> list[Zone]`: 설계 1.4 의 c∘h⁻¹∘z 합집합.
  - `zones`: 지도 좌표의 팔레트 구역(`pallet_permit.zones_from` 의 결과).
  - `hyps_now`: 오도메트리로 옮긴 가설 자세 h_t 전부와 b_t(불가능으로 뺀 것 말고, 겨룸·계산 실패 포함). 자세는 base_link, 허가는 뒤 차축 자세로 계산하므로 같은 기준점으로 바꿔 쓴다.
  - 각 구역 꼭짓점 p 를 c∘h_t⁻¹∘p 로 옮긴 뒤, 그 구역을 (0.30 + δp(d) + 2d·sin 0.05) + 2ρ_max·sin((0.10 + δθ(d))/2) 만큼 바깥으로 키운다. ρ_max 는 옮긴 꼭짓점과 차(c) 사이 거리의 최댓값이다(키움이 ρ 에 단조라 꼭짓점 최댓값으로 전체를 덮는다).
  - `bound` 가 없으면(δ 미정) `None` 을 돌려주고, 부르는 쪽은 후진 0 으로 둔다.
- `rear_clear_for_check(...)`: 출발 직후 한 번, D_MIN + 정지 거리(0.068 m)만큼 곧게 물러나는 동안의 발자국이 합집합 구역·장애물층 허가에 걸리는지. 걸리면 `pose_rear_blocked` 로 세션을 끝낸다(총괄 결정 8). 장애물층 쪽은 러너의 실제 허가 함수(곧은 후진, 곡률 0)로, 구역 쪽은 `first_contact` 로 잰다.

### 2.2 상태기 `PoseCheck` (새 모듈 `src/forklift_core/localization/pose_check.py`)

순수. 시계·오도메트리·SLAM 보정을 입력으로 받고, 매 틱 상태와 할 일을 돌려준다.

- 상태: `confirmed` | `unconfirmed` | `checking`(다시 정합 계산 중, 움직이지 않음) | `ended:<reason>`.
- 출발: `PoseCheck.from_start(residual, b, o0, source)`. 남은 모호성이 없으면 `confirmed` 후보(아래 SLAM 일치를 기다림), 있으면 `unconfirmed`. 알려진 시작(재위치 없음)은 `confirmed` 이고 감시도 하지 않는다(설계 1.2 끝, 범위 밖).
- **누름**(확인 전): 누름 하나는 deadman 이 눌린 첫 틱에 시작하고, 오도메트리 이동 거리를 센다. 남은 거리 D_STROKE − s 가 정지 거리 이하가 되면 속도 0 을 요구하고 누름을 닫는다. 다음 누름은 정지 확인 + deadman 놓음 뒤에만 열린다. 누적 D_TOTAL(3.0 m)에 이르면 더 열지 않는다(`pose_unconfirmed:total`).
- **다시 정합 요청**: `unconfirmed` 이고 출발점에서의 순변위 d ≥ D_MIN, 정지 확인, deadman 놓음, 그 정지에서 아직 안 했으면 `want_rematch` 를 돌려준다. 러너가 작업자를 띄우면 `checking` 이다(이동 0). 결과는 `rematch_decision` 으로 판정한다.
  - `confirm` → SLAM 일치 검사(아래)로.
  - `twin_preferred` / `out_of_set` → `ended:pose_twin_preferred` / `ended:pose_out_of_set`.
  - `unconfirmed`·`too_short` → `unconfirmed` 로 돌아가 다음 누름을 연다.
  - 계산 중 차가 움직였거나(정지 확인이 풀림) epoch·control_gen 이 바뀌었으면 결과를 버린다.
- **SLAM 일치**(모든 승격): 같은 틱의 적용 보정 C 로 만든 뒤 차축 자세와, 근거 자세(출발 확인이면 b 의 정밀화 자세, 다시 정합이면 g 를 스캔 시각에서 지금까지 오도메트리로 옮긴 것)의 뒤 차축 자세를 비교한다. E_SLAM 안이면 `confirmed`, 그 틱에 C_int = 근거 자세 ∘ O(근거 시각)⁻¹ 를 잡는다. 밖이면 `ended:pose_slam_disagree`.
  - 출발 때 바로 확인인 경우, 이 검사는 SLAM 이 처음 tracking 에 든 틱(지금 `bootstrap_check` 자리)에서 한다. 그 전에는 `unconfirmed` 와 같은 잠김이다(권한이 아직 열리지 않는다).
- **확인 뒤 감시**: 매 틱 Δ_int = |C_now∘O_t − C_int∘O_t|(뒤 차축 위치, 방향). 0.20 m 또는 0.05 rad 를 넘으면 `ended:pose_revoked:int`. 정지할 때마다 C_still 을 잡고, 서 있는 동안 Δ_still > E_STILL 이면 `ended:pose_revoked:still`(E_STILL 이 없으면 이 검사는 끔, 기록).
  - 정지 확인, 근접 구간 밖, Δ_int > 0.10 m 이면 `want_rematch`(확인 뒤 다시 정합). 결과 계약은 설계 1.5 그대로다: 확정이고 보수 판정 통과이고 최신 c−g 가 E_SLAM 안이면 C_int 를 새로 잡는다. 확정·통과인데 E_SLAM 밖이면 `ended:pose_revoked:rematch`. 그 밖은 증거가 아니다(C_int 그대로).
- 모든 판정과 근거(자세·보정·Δ·d·누름)는 상태기의 사건 목록에 남는다(러너가 기록에 싣는다).

### 2.3 정책·중재기 (`teleop_policy.py`, `arbiter.py`, `teleop_drive.py`)

- `teleop_policy(..., pose=None)`: `pose` 는 `{"status", "reverse_ok", "stroke_open"}`. `None`(기능 끔)이면 지금과 같다.
  - `unconfirmed`: 다른 거절(상태 모름·근접 구간·복구 등)을 먼저 본 뒤, `reverse_ok` 와 `stroke_open` 이 모두 참일 때만 `Policy(True, False, None, drive_mode="reverse_straight")`, 아니면 거절. 거절 이유는 `pose_unconfirmed`(δ 미정·뒤 막힘), `pose_unconfirmed:stroke`(누름 닫힘), `pose_unconfirmed:total`. 포크는 두 방향 모두 막는다.
  - `checking`: `pose_checking` 으로 거절.
  - `confirmed`: 지금과 같다.
- `reverse_straight` 의 속도 상한은 이미 `recovery_reverse_mps` 0.10 m/s(V_REV_U 와 같다), 조향 허용오차 밖이면 움직이지 않는다(`recovery:steering`). 확인 전에는 같은 값을 쓰되, 이유 문자열은 `pose:steering` 으로 구분한다.
- 누름 상한의 속도 요구는 중재기가 `TeleopDrive.step` 에 넘기는 기존 `drive_allowed`/허가 경로로 적용한다(남은 거리가 정지 거리 이하면 drive_allowed 를 거짓으로).
- 중재기 `_run`: `self.pose_status in ("unconfirmed", "checking")` 이면 `mission_submit`·`mission_resume`·`pick_request`·`recovery`·`sim_reset` 과 AUTO 로의 `mode_request` 를 `pose_unconfirmed` 로 거절한다(`RELOC_REFUSED` 와 같은 자리). 대기 중인 submit·resume 은 회수 때의 경계(control_gen 증가)로 `stale_control` 이 된다(기존 `mission_context`).
- 중재기 `revoke(now, reason)`: `stop(now, reason)` + `_boundary` 와 같은 효과. 러너가 상태기의 `ended:*` 를 받은 틱에 부른다.
- 이미 받은 들기 명령·진행 중인 들기 램프는 상태가 `unconfirmed` 로 있는 동안 `lift_dirs` 가 비므로 기존 `TeleopLift` 가 멈춘다. 상태가 바뀌는 틱(`confirmed` → 끝)에는 `revoke` 의 정지가 멈춘다. 이것을 시험으로 고정한다.

### 2.4 실패 분류 (`mission/failures.py`)

`pose_rear_blocked`, `pose_twin_preferred`, `pose_out_of_set`, `pose_slam_disagree`, `pose_revoked` 를 SESSION 으로 명시한다(이미 모르는 이유는 SESSION 이지만 명시한다).

## 3. ②b 러너 배선 (`sim/isaac/run_transport.py`, `sim/isaac/reloc_phase.py`)

- **인자**: `--pose-check off|on`(기본 off), `--pose-check-odom-bound JSON`(δp(d)=a+b·d, δθ(d)=c+e·d 의 계수, 기본 없음), `--pose-check-e-still JSON`(기본 없음), `--pose-check-e-slam JSON`(기본 잠정값). `on` 은 `--start-relocalize` 와 SLAM 지도 묶음을 요구한다. `near_tracking.POST_MEASUREMENT_ARGUMENTS` 에 `pose_check: "off"` 와 나머지의 기본값을 등록한다(메모리 규칙: 러너 새 인자는 근접 계약에 등록).
- **출발 검사**(`reloc_phase`): `chosen` 뒤, `write_reloc_pose` 전에 작업자 스레드에서 `start_check` 를 돌린다(차는 서서 스텝만 진행). 입력:
  - b: 추정(`relocalizer.estimate.base`).
  - 점: 확정에 쓰인 스캔 묶음. 무인 확정이면 그 시도의 묶음, 운전자 고름이면 마지막 시도의 묶음, 운전자 놓기(F1)면 그 놓기의 묶음.
  - 풀: 세대의 모든 시도 후보(`out["attempts"][*].candidates`, base 자세). 운전자 놓기도 같은 세대의 시도 후보를 쓴다.
  - 마감: 30 s(정합기 마감과 같은 값). 넘으면 `search:timeout` 으로 `unconfirmed`.
  - 결과(`Residual` 요약과 가설 목록)를 `out["pose_check"]` 에 싣는다. 세션을 끝내지 않는다.
- **라이브 루프**:
  - 시작(SLAM 이 tracking 에 든 첫 틱): `PoseCheck.from_start`, 그리고 `rear_clear_for_check`. 막히면 `pose_rear_blocked`.
  - 매 틱 `tick_start` 전에 상태기를 진행하고, `arbiter.pose_status` 와 정책 입력(`reverse_ok`, `stroke_open`)을 넣는다. `Context.localization_ok` 는 `confirmed` 일 때만 참이다(그 밖에는 미션 처리가 `localization_lost` 로 거절한다 — 중재기 거절과 겹치는 마지막 문).
  - 허가(`live_permit`): `unconfirmed`/`checking` 이면 팔레트 구역을 `union_zones` 로 바꾼다. 허가 캐시 열쇠에 (가설 판본, 적용 보정, d) 를 넣는다(설계 1.4 끝 줄).
  - 다시 정합: `want_rematch` 이면 동결 시작 재위치기를 **한 세대** 돌린다(설계 1.5: 시도 3 번·30 s 마감, `StartRelocalizer` 의 결정 그대로, 무인 — 운전자 고름은 없다). 시도마다 정지 확인 동안 SLAM 링크로 보낸 새 스캔 5 개(`scan_capture.SCANS`)를 `reloc_phase._points` 와 같은 방식으로 쌓아 작업자 스레드에서 `relocalize` 를 부른다. competitor·periodic 이면 다음 시도는 새 5 개를 기다린다. 세대가 무인 확정(`estimate.source == "matcher"`)으로 끝나면 그 자세 g 와 마지막 묶음으로 `rematch_decision`(또는 확인 뒤 계약)을 돌리고, 아니면 그 세대의 마지막 이유로 `unconfirmed`. 결과를 epoch·control_gen·스캔 시각과 함께 상태기에 넘긴다.
  - `ended:*`: 그 틱에 `arbiter.revoke`, 바퀴 0, 들기 유지. 정지 확인(또는 2 s)이 되면 `SessionFailure(<reason>)` 를 올린다(기존 `sensor_silence_stopped` 의 순서와 같다). 콘솔 문구는 설계 1.4·1.5 의 것.
- **기록**: `state["pose_check"]` 에 출발 검사, 사건 목록, 다시 정합마다의 입력·결과·시간, 회수. ③ 판정용으로 매 제어 틱의 (상태, 전진·미션 권한이 열렸는지, 적용 뒤 차축 자세, 참값 뒤 차축 자세) 를 압축해 남긴다: 틱마다 오차를 계산해 F1 4.0 분류의 개수와 최댓값을 세고, 원자료는 10 Hz 로 남긴다. 확인 틱과 첫 권한 틱은 따로 표시한다.

## 4. ②c 콘솔

- 발행(`LiveRuntime.publish`): `msg["pose_check"] = {"status", "reason", "stroke": {"left_m", "total_left_m"}, "zones": [합집합 구역 꼭짓점, 지도 좌표]}`. 끔이면 필드가 없다.
- 화면: `unconfirmed` 동안 띠 "확인 전 위치 · 전진·포크 잠김 · 뒤쪽 낮은 물체는 보이지 않습니다 · 차 뒤를 직접 확인하세요", 지도에 합집합 구역. 세션 끝 사유별 문구(뒤 막힘, 다른 곳으로 보임, 회수).
- `profile.js` 의 `REJECT` 에 `pose_unconfirmed`·`pose_unconfirmed:stroke`·`pose_unconfirmed:total`·`pose_checking`·`pose:steering`.
- 계약 문서 `docs/interfaces/2026-10-13-operator-console-contract-draft.md` 5·6 절.

## 5. 시험 (먼저)

- ②a: 상태기의 전이 표 전체(출발 확인·확인 전, 누름 열림·닫힘·D_TOTAL, 다시 정합 다섯 결과, 계산 중 움직임·control_gen 변화로 버림, SLAM 일치 통과·실패, Δ_int·Δ_still 회수, 확인 뒤 다시 정합 세 갈래). 합집합 구역: 가설 하나에서 c∘h⁻¹∘z 가 손으로 계산한 자리와 같음, 키움 양, c 가 바뀌어도 차 기준 구역이 그대로(설계 1.4 의 상쇄), δ 없음 → None. 정책·중재기: 확인 전 거절 목록, 곧은 후진만, 포크 막힘, 들기 램프 멈춤.
- ②b: 끔의 바이트 동일(재생 게이트), 켬의 러너 단위 시험(가짜 엔진·가짜 SLAM 으로 출발 검사 → 확인 전 → 누름 → 다시 정합 → 확인/끝), 인자 등록 시험.
- 전체 묶음은 ws1 에서, 머지 전 최종 트리로.

## 6. 남는 것 (③ 으로)

- δp(d)·δθ(d)·E_STILL·E_SLAM 의 개발 측정과 동결, Isaac 탐침, 판정(설계 4 절). ② 는 이 값들을 인자로 받기만 한다.
- 운영 런처의 기본 전환.

## 7. 자체 검토

(검토 뒤 채운다.)
