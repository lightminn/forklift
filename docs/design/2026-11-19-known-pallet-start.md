# 알려진 팔레트로 시작 + S10 들고 멈춤 — 4a (설계)

작성 2026-11-19 · 상위 [실차 자율 계획](../plans/2026-11-18-real-autonomy-entry.md) PR 4(S10 사례 A 범위, 총괄 승인 2026-11-19) · [제어 틱 계약](2026-11-18-real-control-tick.md) 2절 ④

## 범위

S10 사례 A: 테이프로 표시한 알려진 시작 → 접근 → 삽입 → 운반 높이까지 들고 멈춤. 팔레트 자세는 운용자가 지도 좌표로 입력한다(줄자 시험장 배치, 인식 아님). 코어에 두 가지를 더한다. 둘 다 새 메서드·옵션이고 기본은 쓰지 않으므로 Isaac 경로는 그대로다(재생 게이트 29/29, 전체 pytest).

## 1. `start_known_pallet_mission(t, rear, pickup, inputs, ready=None)`

- 지금 미션은 늘 `observe`(관측 다리 → 포획 → 자세 → 계획)로 시작한다. 관측 없이 시작하는 진입이 없다.
- 머리는 `start_mission` 과 같다: baseline 복원, 입력을 옵션에(`scenario`·`wait_pose`·`pickup_bounds`·`pickup_zone`, `adoption_zone` 없음, 관측 waypoint 없음), `ms.lift_command` = 측정 승강.
- 팔레트 자세: `pickup`(Pose2D)을 그대로 계획 목표로 쓴다. `hooks.planning_target`(Isaac 의 oracle 선택)은 부르지 않는다. 기록: `state["planning_target_source"] = "operator_input"`, `state["perception"]["perception_pickup_estimate_m"]` 에 같은 값과 `source: operator_input, not recognition`. `pocket["planning_pickup"] = pickup`. 알려진 팔레트 층이 켜져 있으면 `known_pallet(...)` 를 같은 규칙으로.
- 계획: 50 Hz 루프를 막지 않도록 **작업자(D8b) 경로**로 — `observe_succeeded` 가 쓰는 것과 같은 `plan_transport` 요청(같은 인자: `start_rear` = 지금 자세, `target_pickup` = `pickup`, 격자 인자 등)을 `ctx["kind"] = "known_pallet"` 로 낸다. 계획을 기다리는 동안 차는 선다(`plan_wait_`).
- 채택(`_known_finish`): `_observe_install(plans, …)` 로 다리·추종기 설치 → `transition("approach")` → `pocket_check` 가 켜져 있으면 지금 자세에서 `build_pocket_check` (깊이 입력은 PR 7; 가짜 포트 시험은 합성 훅) → `_refresh_for_new_leg`.
- 거부: 근접 추적 켜짐, 밑판 검사 켜짐, SLAM 없음·무효, 작업자 없음, 실차가 아닌 실행(Isaac 경로에서 부를 일 없음) — 각각 이유를 남기고 시작하지 않는다.
- 시작 전 phase 는 `idle` 그대로이고, 채택 때 `transition` 이 `idle → approach` 를 남긴다.

## 2. S10 들고 멈춤 — 옵션 `s10_stop_at_lift`(기본 꺼짐)

- 지금 실차 분기는 운용자의 들림 확인(`_operator_gate(..., "lifted")`)에서 `transition("extract")` 를 부르고, 그 전이가 화물 관계(`lift → extract` 에서 `carrying/confirmed`)와 장애물 기억 삭제를 함께 한다.
- 옵션이 켜지면 확인에서 전이 대신 `_s10_end(t)`:
  - 화물을 `carrying/confirmed` 로(새 `CargoState.confirm_lifted()` — 전이표와 같은 값).
  - `state["s10_end"]` 에 시각·승강 측정·팔레트 자세 출처를 남긴다.
  - `commands.terminate("recovery_required", t, "s10_lifted")` — 후속 AUTO·텔레옵은 4d-2b 현장 복구와 새 세대 재위치 확인까지 잠긴다. 그 틱과 다음 틱의 바퀴 출력은 0(정지 분기).
  - `extract` 의 장애물 기억 삭제는 하지 않는다.
- 거절·시한 초과는 지금 규칙 그대로(`Pallet was not lifted`, `Lift did not reach its position`).

## 3. 시험 (가짜 훅, Isaac 없음)

- 시작: 운용자 자세로 계획 요청이 나가고, 채택 뒤 `approach` 로 들어가며 다리가 설치됨; 기록에 `operator_input`; `planning_target` 훅은 불리지 않음; 거부 조건 각각.
- 계획 대기 중 정지(`plan_wait_`), 채택 전 취소.
- S10: 들림 확인에서 `extract` 전이 없음, 화물 `carrying/confirmed`, `recovery_required` 로 끝남, 장애물 기억 그대로, 옵션이 꺼지면 지금과 같음.
- 재생 게이트 29/29(옵션 꺼짐 = 기록과 같은 호출), 전체 pytest.

## 4. 자체 검토

(Codex 검토 뒤 채움)
