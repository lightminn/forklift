# 평면 후보 예산 5 → 6 (G4-B) — 회귀 기록

계획: [G4 계획](../plans/2026-10-02-g4-observation-candidates.md) · 사용자 결정 2026-10-02 · 근거: [G3 기록](2026-10-02-g3-detection-diagnosis.md)
(seed 1 = 계열 B 예산 소진).

## 효과

G2 재실행의 저장 캡처 10 시도를 예산 6 으로 다시 검출했다(`tools/diagnose_detection.py` 의 장면 재구성): **seed 1 시도 2 만**
`no_pallet` → `valid`(좌 3.1 mm·우 5.6 mm, yaw 0.06 mrad). 나머지 9 시도는 상태·포켓 좌표·yaw 가 같다. seed 1 시도 1 은 여전히
`no_pallet`(6–12 모두). 임무 단위 효과는 G2′ 에서 잰다.

## closeout G4 공통 회귀

| 항목 | 결과 |
|---|---|
| 저장 평가 7 실행 330 관측(`python -m tools.compare_plane_budget --before 5 --after 6 artifacts/*_pocket_eval_*`) | **변화 0**. 141 관측이 실제로 6 번째 평면을 추출했다(공허하지 않음). 고유 장면은 200, 장면·설정 조합 300. 오늘의 검출기에 각 실행의 기록 파라미터를 넣고 예산만 바꾼 A/B 이며, 저장 당시 코드의 재생은 아니다(v1 세 실행의 prior 해시는 보관값과 다르다) |
| `scene_rig` 36 자세(x 2.0/2.5/3.0/3.5 × y −0.4/0/0.4 × yaw −0.3/0/0.3) × `pallet`·`blocks`·`grounded`·`shelf` × EPAL 6·T11(유도 파라미터) | 288 장면 **변화 0**. EPAL: 팔레트 29, `grounded` 29, `shelf` 29, `blocks` 0 /36. T11: 27·25·26·0 |
| 실패하는 회귀 시험 | `test_default_budget_reaches_a_pallet_behind_five_larger_planes`: 팔레트 전면이 6 번째 평면인 리그 장면, 예산 5 `no_pallet` → 6 `valid`(변경 전 실패 확인) |
| 전체 pytest | 1,899 통과, 실패 2 = `test_detector_v1_replay.py` 의 **기존 실패**(변경 전 HEAD 에서도 같은 2 건: `median_plane_offset` 추가 뒤 누락 필드 검사가 깨짐). Codex 가 사전검사 이후를 따로 돌려 예산 3 의 100 관측이 허용 델타 안임을 확인했다. 가드 복구는 별건이며, 이 결과를 "전체 통과" 로 적지 않는다 |

## 알려진 한계 — 위양성 증가 사례 1 건 (CPU)

Codex 반례: 위 회귀 시험의 장면에서 팔레트를 `grounded` 음성으로 바꾸면 예산 5 `no_pallet` → 6 `valid`. 예산 5 가 아예 보지 않던 음성을
6 이 검사하게 되고, 검출기의 기존 약점(`grounded` 를 36 자세 중 29 에서 받아들임 — 두 예산 모두)이 그대로 위양성이 된다. 예산이 약점을
만든 것이 아니라 혼잡한 장면에서 그 약점에 닿게 한 것이다. 시험으로 고정했다(`test_known_limit_budget_six_also_reaches_a_grounded_lookalike`).
**렌더 음성 모집단에 대한 영향은 평가하지 않았다.**

## 이 기록이 말하지 않는 것

위양성 비증가, G2′ 임무 결과, 반복 캡처 안정성, 렌더 음성, 실물 성능.
