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
| line_m s0 | P-1 → D-1 · P-2 → D-2 · P-3 → D-3 | (작성 중) | |
| line_m s1 | 같은 순서 | (작성 중) | |

- **앵커**(`session_slot_anchor_log`):
  - random_m s0: P-4 → D-1·P-5 → D-2 둘 다 `anchored`. 세 번째 미션(P-3 → D-3)이 앵커 장면으로 계획돼 하역했다. PR ③ 에서 두 번 다 막혔던 미션이다.
  - random_m s1: P-4 `anchored`, P-5 `docking_not_accepted`. P-5 는 관측 단계에서 끝나 하역 도킹이 없었다.
- **도킹 거부 되돌림**(`session_anchor_unaligned`): 두 실행 모두 호출되지 않았다. 앵커로 계획한 미션은 모두 하역 도킹이 수용됐다.
- **밀림 0.** 손대지 않은 팔레트의 쉬는 자세 대비 최대 이동은 0.041 mm·0.00019 rad 이하다. 장면 손상·금지 접촉은 0 이다.
  - PR ③ 검증 문서의 "2 mm 이하"는 요약 스크립트 단위(mm)를 m 로 읽은 오기다. 실제로는 0.002 mm 이하였다.
- **이웃 실제 최소 간격**: random_m s0 은 P-5 0.436 m(운반, P-3 미션), P-4 0.445 m. s1 은 P-5 0.460 m 다.
- **다리 끝 크리프**: #105 병합 전 같은 장면에서는 3 건(속도 0.0121 m/s·곡률 0.36 고정)이 나왔다. 병합 뒤 두 실행에서는 없다.
  - s1 의 실패는 다른 양상이다. 관측 다리 cusp 에서 위치 오차 0.163 m·명령 0 으로 22 s 서 있다가 시간 초과로 끝났다(cusp 정체). 다리 끝 담당 세션에 넘겼다.
- 실시간 배율: random_m s0 0.349(시뮬 588.5 s / 벽 1686.6 s), s1 0.345(494.8 s / 1436.1 s).

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
