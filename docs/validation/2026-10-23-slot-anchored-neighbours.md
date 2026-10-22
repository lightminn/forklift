# 하역 직선의 이웃을 칸 기준으로 — 검증 (2026-10-23)

계획: `docs/plans/2026-10-22-slot-anchored-neighbours.md`(5판). 상위 계획: `docs/plans/2026-10-22-continuous-transport-ct1.md` PR ④. 브랜치 `feat/ct1-queue-run`.

## 원인 재현 (CPU)

- 실패 두 실행(`a770031` random_m seed 0·1)의 하역된 P-4·P-5 등록 자세를 기록에서 복원했다.
  - 복원 식: 해제 프레임 변화 ∘ (도킹 목표 후축 ⊕ 1.26 m). `34b877b` r0 의 기록된 `mission_scenario` 와 소수 넷째 자리까지 같다.
  - 복원된 P-5 는 D-3 쪽으로 0.1629 m·0.1651 m 에 있다.
  - D-3 하역 직선의 결과: 그대로 두면 `straight_collision`. P-5 를 빼거나 칸에 두면 `success`. r0(0.0708 m)는 `success`.
- 이 재현은 시험 `tests/unit/planning/test_slot_stage_props.py::test_random_m_third_missions_delivery_straight_with_and_without_the_anchor` 로 고정했다.
- 진단용 기록 `mission_scenario`(세션 미션이 계획한 장면)를 미션 기록에 더했다(`34b877b`).

## Isaac live 세션 (ws1, `shard:2`)

- 설정: 학습 원거리 인식·SLAM·D7 도킹, PR #88 근접 옵션, live 배율 목표 0.4. 장면은 표지 세션(`scenes_ct1_a770031/{random,line}_m_session.json`)이다.
- main #105(다리 끝 최소 속도)를 병합한 위에서 돌렸다.
  - random_m 은 `9ddc5d5`(잡 8910·8911)다.
  - line_m 은 `74ed6a1`(잡 9140·9141)이다. `9ddc5d5` 에 더한 것은 자기 팔레트 앵커 제외·시험·콘솔 문구다.
- 실행 폴더는 ws1 `/home/projects/forklift/test_runs/live/ct1_anc_<sha>_<장면>_s<seed>` 다. 요약은 `ct1_report.py` 로 했다.

| 장면·seed | P → D | 결과 | 하역 오차 |
|---|---|---|---|
| random_m s0 | P-4 → D-1 · P-5 → D-2 · P-3 → D-3 | **done · done · done** | 1.2 · 3.6 · 1.6 cm |
| random_m s1 | 같은 순서 | done · **실패** `Tracking timeout in observe` · done | 3.2 · — · 0.9 cm |
| line_m s0 | P-1 → D-1 · P-2 → D-2 · P-3 → D-3 | done · done · **실패** `Tracking timeout in return_home`(하역 뒤) | 1.6 · 2.6 · 1.6 cm |
| line_m s1 | 같은 순서 | done · done · **실패** `cusp_position_stall`(운반 중) | 1.8 · 2.5 · — |

- **앵커**(`session_slot_anchor_log`):
  - random_m s0: P-4 → D-1·P-5 → D-2 둘 다 `anchored`. 세 번째 미션(P-3 → D-3)이 앵커 장면으로 계획돼 하역했다. PR ③ 에서 두 번 다 막혔던 미션이다.
  - random_m s1: P-4 `anchored`, P-5 `docking_not_accepted`. P-5 는 관측 단계에서 끝나 하역 도킹이 없었다.
  - line_m s0·s1: P-1 → D-1·P-2 → D-2 `anchored`. 세 번째 미션은 둘 다 앵커 장면의 계획 검사를 통과했다. s0 은 하역했고, s1 은 하역 도킹 전 운반 중에 끝났다(아래 분류).
- **도킹 거부 되돌림**(`session_anchor_unaligned`): 네 실행 모두 호출되지 않았다. 앵커로 계획한 미션은 8 개다. 그중 하역 도킹에 도달한 6 개는 모두 수용됐고, 2 개(random_m s1 m2, line_m s1 m3)는 도킹 전에 끝났다(Codex 6차 P3).
- **밀림 0.** 손대지 않은 팔레트의 쉬는 자세 대비 최대 이동은 0.041 mm·0.00019 rad 이하다. 장면 손상·금지 접촉은 0 이다.
  - PR ③ 검증 문서의 "2 mm 이하"는 요약 스크립트 단위(mm)를 m 로 읽은 오기다. 실제로는 0.0023 mm 이하였다.
- **이웃 실제 최소 간격**: random_m s0 은 P-5 0.436 m(운반, P-3 미션), P-4 0.445 m. s1 은 P-5 0.460 m 다. line_m 은 s0·s1 모두 P-2 0.34 m(추출, P-1 미션)다. PR ③ 의 0.353 m 와 같은 자리다.
- **다리 끝 크리프**: #105 병합 전 같은 장면에서는 3 건(속도 0.0121 m/s·곡률 0.36 고정)이 나왔다. 병합 뒤 두 실행에서는 없다.
  - s1 의 실패는 다른 양상이다(cusp 정체). 다리 끝 담당 세션 답: #105 하한 소관이 아니라 cusp 정체 래치(`cusp_stall.py`) 소관이다.
    - cusp 래치는 세 번 다 잡았다(224.5·241.6·251.2 s, 마지막은 위치 오차 0.163 m). 매번 관측 규칙(`observe_rule`)으로 넘겼다.
    - 관측 재계획은 두 번(230.8·242.6 s)으로 예산을 다 썼다. 세 번째 인계 뒤에는 재계획이 막혀(core 2166) 명령 0 으로 서 있다가 `Tracking timeout in observe` 로 끝났다(Codex 6차 P2 — 처음 적은 "래치 기록 없음"은 잘린 출력을 읽은 오기다).
    - 기록: `ct1_anc_9ddc5d5_random_s1/live/epoch_1/run/missions/m2/records.json`(`cusp_stalls`·`observe_replans`·`tracking_failure`).
- 실시간 배율: random_m s0 0.349(시뮬 588.5 s / 벽 1686.6 s), s1 0.345(494.8 s / 1436.1 s). line_m s0 0.357(573.7 / 1605.8 s), s1 0.352(459.9 / 1306.3 s).

### 실패 분류 — 칸 앵커와 무관한 제어 단계

| 실행 | 단계 | 양상 | 소관 |
|---|---|---|---|
| random_m s1 m2 | 관측 | cusp 래치 세 번 → 관측 규칙 인계. 재계획 예산 2 회를 다 쓴 뒤 세 번째 정체(오차 0.163 m)에서 시간 초과 | 관측 다리의 cusp 정체·재계획 예산 |
| line_m s0 m3 | 복귀 | 하역 1.6 cm 뒤 해제 재계획 경로를 잘 따라감. 경로 61 번 자세(−0.17, −8.07)에서 명령 0·`at_cusp` 로 85 s 서 있다가 시간 초과. SLAM 위치 오차 최대 0.28 m(튐 없음) | 복귀 다리 정지(원인 미확인) |
| line_m s1 m3 | 운반 | 운반 다리 끝에서 위치 오차 0.075 m 로 cusp 정체 래치 → `cusp_position_stall` | cusp 정체 래치 |

- 세 실패 모두 계획 단계를 통과한 뒤 주행·cusp 단계에서 났다. 칸 앵커는 하역 직선·후퇴의 계획 검사만 바꾼다.
- Codex 6차가 기록의 입력으로 앵커를 빼고 다시 계획해 비교했다. line_m s1 m3 의 운반 경로 263 자세와 line_m s0 m3 의 해제 뒤 복귀 경로 315 자세가 기록과 정확히 같았다(방향·곡률 포함). 그래서 앵커가 경로를 바꿔 이 실패를 만들었다는 근거는 없다. 실행 전체의 인과까지 같다는 증명은 아니다.
- 기록: 각 실행 폴더 `run/missions/m{2,3}/records.json`(`cusp_stalls`·`observe_replans`·`tracking_failure`), `run.log` SAMPLE.

## 시험

- 계획기(`tests/unit/planning/test_slot_stage_props.py`):
  - 표시 없는 장면은 단계별 목록이 같다.
  - 세 계획기에서 하역 직선을 `@slot` 만 막으면 거부, `@search` 만 막으면 통과. 후퇴도 같다.
  - 단계별로 받는 장애물 목록, 작업자 세 분기의 pickle 왕복, random_m 세 번째 미션 재현.
- 대상 목록의 장애물(`tests/unit/safety/test_pallet_permit.py`): 앵커된 하역 팔레트는 두 장애물이고, 하역되지 않은 팔레트의 앵커는 무시한다.
- 판단 함수(`tests/unit/test_session_pallets.py`):
  - `decide_anchor` 의 열 가지 거부 사유와 수용.
  - `anchor_poses` 는 처음 가는 칸·옮겨진 목표·회전에서 맞다.
- 러너(`tests/unit/test_session_runner.py`, AST):
  - 증거 수집 → 결과 뒤 확정, 정지 중 흔적 무효 → 해제, 거부 사례.
  - 자기 팔레트 제외.
  - payload 전 확정·채택 때 재확정(`session_anchor_changed`).
  - 도킹 거부 되돌림 판단과 core 의 두 호출 자리·분류(MISSION).
- 소스 가드: `test_failures`(새 사유), 콘솔 사유 문구(`test_results_reasons`).
- ws1 전체(`9ddc5d5`): 6 011 통과. 실패 12 는 알려진 ws1 전용 10 과 콘솔 사유 문구 2 다. 문구 2 는 `74ed6a1` 에서 고쳤다.

## 회귀

- M-a1 재생 게이트 29/29 `finish ok`(`9ddc5d5`, ws1 잡 8909).

## 검토

- 계획:
  - ① 직접 재계산.
  - ② Codex 적대 검토 1–4 차. 차례로 P2 6·2·1·1 을 받아들였다.
  - ③ agy(Gemini 3.8 Flash) 독립 검토 — 받아들인 것과 아닌 것은 계획 7 절에 적었다.
- 구현(5판 + `920b055`·`9ddc5d5`): Codex "새 P1/P2 없음". P3 둘(자기 앵커 제외·후퇴 역검사)은 `43375f7` 에서 반영했다.
- 총괄 요구는 계획 1.3·2.2·2.3 에 넣었다: 하역 오차 가정의 정량화, 판정 실패·불확실 팔레트 규칙, 프레임 섞임 명시, 실차 지도 프레임에서도 성립하는지.
