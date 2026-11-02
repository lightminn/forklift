# 근접 재관측 다리 끝 정지의 안전망 — C2 (계획 1 판, 2026-11-02)

상위 계획: `docs/plans/2026-11-02-near-leg-end-stall.md`(PR #260)의 C2. D2′(PR #261) 결과로 C1 은 보류됐다. 총괄 승인(2026-11-02)을 받았다.

총괄 조건:
- 별도 계획 → 구현 전 중단점 → Codex 재검토.
- 상위 계획 4 판의 회귀 조건(K3 판정·`cusp_end_recovered` 불변)을 따른다.
- 러너 옵션 뒤에 두고, 러너 기본은 끔이다.
- 재생 게이트는 `SETUP_INPUTS=recorded` 로 29/29 다.
- 런처 기본 승격은 총괄이 결정한다. 승격 조건은 이 계획에 미리 적는다.

## 0. 받는 것과 받지 않는 것

- 받는 것: 근접 재관측 다리의 최종 목표 정지. 조건은 투영 잔량 0, 추종 `tracking`, cusp 아님, 명령 0, 그리고 관측 완화 도착(0.10 m · 0.10 rad) 밖이다. S1 의 off s2·on s5·on s9 가 이 꼴이다. 지금은 받는 규칙이 없어 30 s 시간 초과로 끝난다.
- 받지 않는 것:
  - 근접 다리 cusp 정체와 그 예산 소진(on s2 류, 후속).
  - 첫 원거리 관측(기존 O1).
  - SLAM 고정 중 재정렬(`realign_active()`, 기존 `near_capture_realign_held_recovery` 가 받음).
  - 밑판 옆면 다리, backoff(관측 시작·장애물).

## 1. 규칙 (O1 의 의미를 그대로 근접 다리로)

- **적용 조건 `near_reobserve_leg()`** — 아래를 모두 만족해야 한다. 기존 `near_active()` 는 `approach`·`insert` 에서만 참이라 쓰지 않는다(Codex).
  - SLAM 이 있고 단계가 `observe` 다.
  - `state["near_capture"]["status"] == "pending"` 이고 `"waypoint"` 가 있다. 즉 근접 다리가 설치됐다(`near_leg_or_recapture`).
  - `realign_active()`·`bottom_deck_leg_active()` 가 거짓이고, `observe_backoff` 가 없으며, 장애물 backoff 단계가 `observe` 가 아니다.
  - `scene_far_observe()` 와는 서로 배타다(O1 은 그대로 둔다).
- **정지 판정**은 O1 과 같다.
  - 추종 `tracking`, `at_cusp` 아님, 명령 속도 0, `remaining_to_goal_m() ≤ 1e-9`, `cusp_monitor.holds_observe()` 아님.
  - 이 조건이 이어지는 동안 명령 0 지속 시간을 `commands.work_elapsed` 로 잰다. `OBSERVE_END_STAND_S`(3 s)를 넘기고, 그 틱에 `stop_now` 이면서 완화 도착 범위 밖이면 판정한다. 범위 안이면 완화 도착이 자기 1 s 를 마치게 둔다.
- **처분**은 O1 과 같다.
  - 근접 다리 일련번호(`observe_replan_leg.serial`)당 한 번 시도한다. 설치한 다리에서 다시 무장하지 않는다.
  - 다리 예산(`observe_replans_per_leg`, 2)이 남아 있을 때만 같은 목표(`paths["observe"].poses[-1]` = 근접 waypoint)로 다시 계획한다. 호출은 O1·`observe_miss` 와 같은 `plan_observation_leg(..., extended=False, grid_kwargs("observe"))` 다.
  - 성공하면 설치한다. `observe_replans` 에 `trigger: "near_end_stand"` 로 넣어 예산 하나를 쓰고, `phase_started` 를 갱신하고, `realign_install` 을 부른다(재정렬 중이 아니면 아무 일도 하지 않음).
  - 실패하거나 예산이 없으면 기록만 하고 기다린다. 즉시 끝내지 않는다. 그 뒤 완화 도착이나 시간 초과가 지금처럼 받는다.
- **기록:** `observe_end_stands` 에 O1 과 같은 꼴(시각, 다리 일련번호, `zero_command_s`, 위치·방위 오차, 자세, `outcome`)로 남기고, `"kind": "near"` 를 더한다. O1 기록에는 `"kind"` 가 없으니 읽는 쪽은 없음을 `"far"` 로 읽는다.
- **SLAM 고정과 좌표:** 근접 다리 끝 정지는 SLAM 고정 상태에서 일어난다. 이 점은 O1 과 같다.
  - 최종 목표 고정 규칙(`core.py` `follow` 끝, 남은 거리 ≤ 0.5 m + 제동거리)이 `observe` 단계에도 건다. S1 세 실패는 다리 시작 t+10.7–11.3 s 에 `holding` 으로 바뀌어 끝 정지 내내 고정이었다(`slam_records` mode).
  - 그 전 다리 구간은 `tracking` 이었다.
  - C2 는 고정을 유지한 채 같은 목표로 재계획한다. O1·`observe_miss` 의 같은 목표 재계획과 같다. 시작 자세(`rear`, 고정된 제어 좌표)와 목표(근접 waypoint, 다리 설치 때의 제어 좌표)는 고정이 보정을 얼린 같은 좌표계에 있다. 고정 해제는 지금처럼 settle 에서 한다.
  - 판정하지 않는 것은 재정렬 중 고정(`realign_active()` 이면서 `holding`)뿐이다. 이것은 기존 `near_capture_realign_held_recovery` 가 받는다.

## 2. 옵션과 결합

- 러너 인자 `--near-end-stand on|off`(기본 `off`). `off` 면 코드 경로가 지금과 바이트 단위로 같은 결정을 낸다.
- 근접 추적 계약 등록: 새 인자는 `sim/isaac/near_tracking.py` 의 `RUN_SPECIFIC_ARGUMENTS` 에 넣는다(측정 뒤 인자라면 `POST_MEASUREMENT_ARGUMENTS`). 시험 `tests/integration/test_isaac_transport_cli.py` 가 등록을 확인한다. 그래야 `--near-tracking` 실행이 거부되지 않는다(메모리: 러너 인자 → 근접 계약).
- 기록 형식: 경계 기록 인자 재구성(`rebuild_cli`)과 재생에서 새 인자의 기본값이 옛 기록에 `off` 로 들어가게 한다.
- C1 과의 결합: C1 은 보류다. 나중에 C1 이 놓기를 거부해 재계획이 예산을 쓰면, C2 는 남은 예산에서만 시도한다. 예산 소진은 상위 계획 6 절 후속(on s2 류)에서 다룬다.

## 3. 회귀 조건

- **R1 — C2 는 K3 를 대신하지 않는다.**
  - `cusp_end_recovered` 와 `prefix_converged` 를 바꾸지 않는다(코드 diff 로 확인).
  - 같은 `judge` 입력에서 K3 판정은 옵션 켬·끔이 같다. C2 가 개입하기 전까지의 공통 이력만 비교한다(상위 계획 4 판).
- **R2 — 끔이면 지금과 같다.**
  - 재생 게이트: `SETUP_INPUTS=recorded`, runs29 29/29(옵션 끔). 기존 기록은 근접 다리 끝 정체를 포함하지 않는다.
  - 기존 O1 단위 시험이 그대로 통과한다.
- **R3 — 적용 조건 밖에서는 발동하지 않는다.** 다음 각각에서 기록이 생기지 않는다.
  - 원거리 관측
  - 재정렬 중
  - 밑판 옆면
  - backoff
  - 재정렬 중 고정(최종 목표 고정은 적용 대상이다)
  - cusp 정지
  - 완화 범위 안 정지
  - 명령이 있는 틱
- **R4 — 실패·예산 없음은 기다린다.** 재계획 실패나 예산 없음 뒤에도 차는 서 있다. 완화 도착이나 시간 초과가 지금처럼 받는다(O1 의 CPU 반례 꼴을 근접 다리로 옮긴 시험).
- **R5 — 다리당 한 번.** 설치한 다리에서 다시 정지해도 C2 는 다시 시도하지 않는다. 그 다리의 다른 회복(`observe_miss`)은 남은 예산 안에서 그대로 동작한다.

## 4. 시험

- 단위(코어):
  - S1 세 실패의 정지 상태를 재현하는 고정 상태(근접 다리 설치, 최종 목표 SLAM 고정, 잔량 0, 0.053–0.067 m · 0.11–0.17 rad, `stop_now`)에서 3 s 뒤 재계획을 설치한다. 고정은 유지된다.
  - R3 각 경우, R4·R5, O1 과의 배타, 옵션 끔.
- 재생 게이트: 옵션 끔 29/29.
- Isaac 검증 배치는 등록 문서로 수치를 결과 전에 고정한다.
  - 모집단: random_1 20 seed × {C2 켬, 끔}. S1c 와 같은 스냅숏 계열·새 질량이다.
  - 재현을 늘리려면 근접 다리 끝 정체를 일으키는 주입이 필요할 수 있다(S1c 에서 빈도가 낮을 때). 주입은 등록 전에 정한다.

## 5. 승격 조건 (런처 기본 켬 — 총괄 결정의 입력, 결과 전에 고정)

검증 배치에서 모두 만족해야 한다.
1. **틀린 확정 0:** C2 가 설치한 다리를 거쳐 근접 캡처를 받아들인 실행에서, 그 뒤 `near_capture_misaligned`·금지 접촉·팔레트 변위 실패가 0 이다. C2 가 받아들일 수 없는 자세를 근접 캡처로 넘기지 않았다는 뜻이다.
2. **불필요 발동 0:** C2 기록마다 판정 틱이 완화 범위 밖이고 `stop_now` 이며 명령 0 지속이 3 s 이상이다(규칙의 구조 검사). 그리고 같은 seed 의 끔 팔이 같은 근접 다리 끝 정지에서 완화 도착으로 끝난 경우를 "불필요 의심"으로 세며, 그 수가 0 이다. Isaac 비결정성 때문에 짝은 같은 근접 다리 끝 정지가 양 팔에 다 있을 때만 맞춘다.
3. **무회귀:** 켬 팔의 완주가 끔 팔보다 나쁘지 않다. 판정은 실패 1 건 허용 이내이고, 근접 다리 밖 실패 유형의 새 출현은 0 이다.
4. **효과:** 켬 팔에서 근접 다리 끝 정체로 인한 시간 초과가 0 이다. 발동이 0 건이면 효과는 판정 불가로 두고, 승격하지 않는다(주입 배치로 다시 본다).

## 6. 구현 전 중단점

이 계획이 Codex 재검토에서 수렴한 뒤 구현한다. 구현 diff 도 Codex 검토와 재생 게이트를 거친다.

## 7. 자체 검토

- 수치 출처:
  - S1 세 실패의 정지 오차(`result.json` `tracking_failure`)
  - O1 의 `OBSERVE_END_STAND_S` 3 s·완화 범위 0.10/0.10(`core.py`)
  - 근접 다리 설치 경로(`near_leg_or_recapture`, `final_straight_prefix`)
  - SLAM 고정 시점(S1 `slam_records` mode, 다리 시작 t+10.7–11.3 s 에 `holding`)과 최종 목표 고정 규칙(`core.py` `follow`)
- 독립 검토 3 회 의무는 상위 계획(#260)에서 거쳤다. 이 문서는 그 C2 갈래의 구현 계약이라 Codex 재검토로 수렴을 본다.
