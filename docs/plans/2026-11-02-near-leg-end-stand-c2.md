# 근접 재관측 다리 끝 정지의 안전망 — C2 (계획 2 판, 2026-11-02)

상위 계획: `docs/plans/2026-11-02-near-leg-end-stall.md`(PR #260)의 C2. D2′(PR #261) 결과로 C1 은 보류됐다. 총괄 승인(2026-11-02)을 받았다.

총괄 조건:
- 별도 계획 → 구현 전 중단점 → Codex 재검토.
- 상위 계획 4 판의 회귀 조건(K3 판정·`cusp_end_recovered` 불변)을 따른다.
- 러너 옵션 뒤에 두고, 러너 기본은 끔이다.
- 재생 게이트는 `SETUP_INPUTS=recorded` 로 29/29 다.
- 런처 기본 승격은 총괄이 결정한다. 승격 조건은 이 계획에 미리 적는다.

## 0. 받는 것과 받지 않는 것

- 받는 것: 근접 재관측 다리의 최종 목표 정지. 조건은 투영 잔량 0, 추종 `tracking`, cusp 아님, 명령 0, 그리고 관측 완화 도착(0.10 m · 0.10 rad) 밖이다. S1 의 off s2·on s5·on s9 가 이 꼴이다. 지금은 받는 규칙이 없어 30 s 시간 초과로 끝난다.
- 받지 않는 것(후속 포함):
  - O1 의 같은 무진행 감시 빈틈(설치 뒤 재시작 없음).
  - 근접 다리 cusp 정체와 그 예산 소진(on s2 류, 후속).
  - 첫 원거리 관측(기존 O1).
  - SLAM 고정 중 재정렬(`realign_active()`, 기존 `near_capture_realign_held_recovery` 가 받음).
  - 밑판 옆면 다리, backoff(관측 시작·장애물).

## 1. 규칙 (O1 의 의미를 그대로 근접 다리로)

- **적용 조건 `near_reobserve_leg()`** — 아래를 모두 만족해야 한다. 기존 `near_active()` 는 `approach`·`insert` 에서만 참이라 쓰지 않는다(Codex).
  - SLAM 이 있고 단계가 `observe` 다.
  - `state["near_capture"]["status"] == "pending"` 이고 `"waypoint"` 가 있다. 즉 근접 다리가 설치됐다(`near_leg_or_recapture`).
  - `realign_active()`·`bottom_deck_leg_active()` 가 거짓이고, `observe_backoff` 가 없으며, 장애물 backoff 단계가 `observe` 가 아니다.
  - `not scene_far_observe()` 를 조건에 명시한다. 정상 흐름에서는 인식 뒤 `pocket["planning_pickup"]` 가 설정돼 있어 서로 배타지만, 그 불변식에 기대지 않는다. 둘이 동시에 참인 상태가 없음을 시험한다(Codex 5 차). O1 은 그대로 둔다.
- **정지 판정**은 O1 과 같다.
  - 추종 `tracking`, `at_cusp` 아님, 명령 속도 0, `remaining_to_goal_m() ≤ 1e-9`, `cusp_monitor.holds_observe()` 아님.
  - 이 조건이 이어지는 동안 명령 0 지속 시간을 `commands.work_elapsed` 로 잰다. `OBSERVE_END_STAND_S`(3 s)를 넘기고, 그 틱에 `stop_now` 이면서 완화 도착 범위 밖이면 판정한다. 범위 안이면 완화 도착이 자기 1 s 를 마치게 둔다.
- **처분**은 O1 과 같다.
  - 근접 다리 일련번호(`observe_replan_leg.serial`)당 한 번 시도한다. 설치한 다리에서 다시 무장하지 않는다.
  - 다리 예산(`observe_replans_per_leg`, 2)이 남아 있을 때만 같은 목표(`paths["observe"].poses[-1]` = 근접 waypoint)로 다시 계획한다. 호출은 O1·`observe_miss` 와 같은 `plan_observation_leg(..., extended=False, grid_kwargs("observe"))` 다.
  - 성공하면 설치한다. `observe_replans` 에 `trigger: "near_end_stand"` 로 넣어 예산 하나를 쓰고, `phase_started` 를 갱신하고, `realign_install` 을 부른다(재정렬 중이 아니면 아무 일도 하지 않음).
  - 설치에 성공하면 장애물 층의 60 s 무진행 감시(`obstacle["progress"]`, `core.py` permission)를 새 경로 기준으로 다시 시작한다. 그대로 두면 잔량 최솟값 0 이 남아, 새 경로를 진행해도 갱신되지 않는다. Codex CPU 반례로는 설치 7.01 s 뒤 `obstacle_no_progress in observe` 로 끝났다. 실패·예산 없음에서는 감시를 그대로 둔다. O1 에도 같은 빈틈이 있지만 이번 범위가 아니어서 후속에 둔다.
  - 실패하거나 예산이 없으면 기록만 하고 기다린다. 즉시 끝내지 않는다. 그 뒤 완화 도착이나 시간 초과가 지금처럼 받는다.
- **기록:** `observe_end_stands` 에 O1 과 같은 꼴(시각, 다리 일련번호, `zero_command_s`, 위치·방위 오차, 자세, `outcome`)로 남기고, `"kind": "near"` 를 더한다. O1 기록에는 `"kind"` 가 없으니 읽는 쪽은 없음을 `"far"` 로 읽는다.
- **SLAM 고정과 좌표:** 근접 다리 끝 정지는 SLAM 고정 상태에서 일어난다. 이 점은 O1 과 같다.
  - 최종 목표 고정 규칙(`core.py` `follow` 끝, 남은 거리 ≤ 0.5 m + 제동거리)이 `observe` 단계에도 건다. S1 세 실패는 다리 시작 t+10.7–11.3 s 에 `holding` 으로 바뀌어 끝 정지 내내 고정이었다(`slam_records` mode).
  - 그 전 다리 구간은 `tracking` 이었다.
  - C2 는 고정을 유지한 채 같은 목표로 재계획한다. O1·`observe_miss` 의 같은 목표 재계획과 같다. 시작 자세(`rear`, 고정된 제어 좌표)와 목표(근접 waypoint, 다리 설치 때의 제어 좌표)는 고정이 보정을 얼린 같은 좌표계에 있다. 고정 해제는 지금처럼 settle 에서 한다.
  - 판정하지 않는 것은 재정렬 중 고정(`realign_active()` 이면서 `holding`)뿐이다. 이것은 기존 `near_capture_realign_held_recovery` 가 받는다.
  - waypoint 는 고정 map 목표로 둔다(O1·`observe_miss` 와 같음). 같은 map 프레임이라는 것이 팔레트와의 상대 정렬까지 보장하지는 않는다. 그것은 재캡처의 정렬·굴림 검사가 맡는다.
  - 고정 해제는 settle 뿐 아니라 transport 진입에서도 예약된다. 그것도 지금과 같다.

## 2. 옵션과 결합

- 러너 인자 `--near-end-stand off|shadow|on`(기본 `off`).
  - `off` 면 지금과 같은 결정을 낸다.
  - `shadow` 는 적용 조건과 정지 판정을 계산해 "발동했을 것"을 `observe_end_stands` 에 `outcome: "shadow"` 로 기록만 한다. 재계획·설치·감시 재시작은 하지 않는다. 같은 실행 안에서 그 시점 뒤가 완화 도착으로 끝났는지, 시간 초과로 끝났는지를 본다(불필요 발동 측정, Codex 5 차).
  - `on` 은 1 절대로 동작한다.
- 근접 추적 계약 등록: 새 인자는 `sim/isaac/near_tracking.py` 의 `RUN_SPECIFIC_ARGUMENTS` 에 넣는다. `POST_MEASUREMENT_ARGUMENTS` 에 기본 `off` 만 넣으면 옛 측정 기록에 대해 `on` 이 조건 불일치가 되므로 쓰지 않는다(Codex P3). 시험 `tests/integration/test_isaac_transport_cli.py` 가 등록을 확인한다. 그래야 `--near-tracking` 실행이 거부되지 않는다(메모리: 러너 인자 → 근접 계약).
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
  - R3 각 경우, R4·R5, O1 과의 배타, 옵션 끔, `shadow` 가 행동을 바꾸지 않음.
  - 설치 시험은 그 뒤의 `permission()` 까지 지나, 무진행 감시가 다시 시작됐는지 확인한다(실패·예산 없음에서는 보존).
  - 고정 상태에서 받은 보정과 적용 보정이 다른 경우(`received ≠ applied`)에 시작 자세와 계획 격자가 같은 적용 보정을 쓰는지 본다.
  - C2 설치부터 실제 재캡처 호출 흐름(`near_leg_or_recapture` → 새 직선의 box·dry-run·redraw/realign 검사)까지 이어 본다. 채택과 거부 양쪽을 다룬다. 거부되면 `done` 이나 근접 tracker 생성으로 넘어가지 않아야 한다.
  - 두 미션 연속 실행: 두 번째 미션에서 C2 상태(시도 기록, 다리 일련번호)가 초기화돼 다시 동작한다. 관련 키는 `baseline` 소유 목록에 있다.
- 재생 게이트: 옵션 끔 29/29.
- Isaac 검증 배치는 등록 문서로 수치를 결과 전에 고정한다.
  - 모집단: random_1 20 seed × {`on`, `shadow`}. S1c 와 같은 스냅숏 계열·새 질량이다. 끔 팔 대신 `shadow` 를 써서 같은 정지가 기다렸으면 어떻게 끝났을지를 그 실행 안에서 본다.
  - 근접 다리 식별: 미션 번호, 근접 다리 일련번호, 실제 tracker 경로(`tracking_failure.path`나 그 다리의 설치 기록)로 한다. `paths.json`·`result.paths.observe` 의 끝은 원거리 목표라 쓰지 않는다(Codex 5 차).
  - 재현을 늘리려면 근접 다리 끝 정체를 일으키는 주입이 필요할 수 있다(S1c 에서 빈도가 낮을 때). 주입은 등록 전에 정하고, 주입 배치는 자연 발생 배치와 따로 집계한다.

## 5. 승격 조건 (런처 기본 켬 — 총괄 결정의 입력, 결과 전에 고정)

용어: 사건마다 단계를 나눈다. 판정(기록) → 시도(예산 있음) → 설치(재계획 성공) → 캡처 채택(`near_capture` `done`) → 다음 다리 성공(approach 완료) → 미션 완주.

`on` 팔에서 모두 만족해야 한다.
1. **회복:** 설치 사건 중 미션 완주에 이른 것이 최소 2 건이고, 설치 사건의 절반 이상이다. 설치가 2 건 미만이면 판정 불가로 두고 승격하지 않는다(주입 배치로 다시 본다). 허용하는 실패 전이는 다음 둘뿐이다.
   - 설치 → 캡처 거부(`near_capture_misaligned` 등 채택 전 거부, 또는 `near_capture_failed`)
   - 설치 → 근접 다리 재정지 → 시간 초과
   이 둘은 "미회복"으로 센다. 그 밖의 전이는 아래 2 의 대상이다.
2. **틀린 확정·안전 0:** 설치 사건에서 C2 주행 중(캡처 채택 전 포함)과 그 뒤에 금지 접촉·팔레트 변위·장면 손상 실패가 0 이다. 캡처를 채택한 뒤 그 캡처 자세 때문에 approach·insert 가 실패한 것(`near_capture` 채택 뒤 삽입 정렬 실패 유형)도 0 이다.
3. **무회귀:** 미션 완주가 `shadow` 팔보다 나쁘지 않다(실패 1 건 허용 이내). 근접 다리 밖의 새 실패 유형 출현은 0 이다.
4. **불필요 발동 0:** `shadow` 팔에서 "발동했을 것" 기록 뒤 같은 정지가 완화 도착으로 끝난 사건이 0 이다. 대응이 잡힌 `shadow` 기록 수와 대응을 못 잡은 수를 따로 보고한다. 유효한 `shadow` 기록이 0 이면 이 조건은 미판정이고, 승격하지 않는다.

## 6. 구현 전 중단점

이 계획이 Codex 재검토에서 수렴한 뒤 구현한다. 구현 diff 도 Codex 검토와 재생 게이트를 거친다.

## 7. 자체 검토

- Codex C2 계획 검토 1 차(`tmp/rx/codex_c2a.out`, 6a60f0fc): P1 없음, P2 세 건. 반영한 것은 설치 뒤 무진행 감시 재시작, 승격 조건을 단계 전이·최소 회복·미판정으로 다시 쓴 것, `shadow` 모드로 불필요 발동을 같은 실행 안에서 재는 것이다. P3 인자 등록도 확정했다. predicate 배타 명시, `received ≠ applied` 고정 시험, 재캡처 흐름의 채택·거부 시험, 두 미션 연속 시험을 더했다.

- 수치 출처:
  - S1 세 실패의 정지 오차(`result.json` `tracking_failure`)
  - O1 의 `OBSERVE_END_STAND_S` 3 s·완화 범위 0.10/0.10(`core.py`)
  - 근접 다리 설치 경로(`near_leg_or_recapture`, `final_straight_prefix`)
  - SLAM 고정 시점(S1 `slam_records` mode, 다리 시작 t+10.7–11.3 s 에 `holding`)과 최종 목표 고정 규칙(`core.py` `follow`)
- 독립 검토 3 회 의무는 상위 계획(#260)에서 거쳤다. 이 문서는 그 C2 갈래의 구현 계약이라 Codex 재검토로 수렴을 본다.
