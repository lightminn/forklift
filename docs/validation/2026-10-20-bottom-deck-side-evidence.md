# 밑판 옆면 증거 — CPU 평가와 Isaac 미션 (2026-10-20)

계획: `docs/plans/2026-10-19-bottom-deck-side-evidence.md`(작업 1–5, 각 단계 Codex 수렴). 브랜치 `feat/bottom-deck-evidence`. 판정 고정본은 `979dcd3`(판정 해시 `c9c45eb1…`, 평가기·장면 도구 해시 `12a4cd9e…`)이다.

사용자 결정(2026-10-19–20):
- "옆면 증거 확인 추가 (권장)", "양쪽 옆면 모두 확인" — 두 면이 모두 `present` 여야 집는다.
- "판정에 면 찾기 추가 (권장)" — Isaac 에서 드러난 옆면 지점 자세 오차를 판정이 흡수한다.
- "eval2 까지 끝내고 정리·PR (권장)" — 옆면 다리 뒤 SLAM 연결은 다음 작업.

## 요약

- **문제:** 진입면만 보면 팔레트와 grounded(밑판 없이 블록이 바닥까지 내려간 모양)가 같아 보인다. 단계 3 학습 제안기의 음성 오검출 13/65 가 모두 grounded 였다(`docs/validation/2026-10-19-recognition-stage3-learned-proposals.md`). 밑판은 모형 x 를 따라 블록 열 아래를 지나므로, 옆면(모형 ±y)의 블록 행 사이 틈에서만 보인다.
- **판정(CPU):** 옆면 한 장에서 블록 행을 찾고 틈마다 밑판 높이의 띠를 판·열림·가림으로 가른다. 모든 틈이 판이고 표지·윗면 검사를 지나면 `present`, 모든 틈이 열리면 `absent`, 그 밖은 `unobserved` 다. 두 면 정책은 하나라도 `absent` 면 거부, 둘 다 `present` 일 때만 허가다.
- **CPU 평가(이 시뮬 모집단에 대한 조건부 값, 실물 오허가율이 아님):**
  - v1(옛 판정 `ce0f714`, 법선 오차 ±30 mm): eval 1 회 12 조합 모두 합격.
  - v2(면 찾기 판정 `979dcd3`, 법선 오차 ±100 mm + 촬영마다 자세 오차): dev2·eval2 모두 12 조합 합격. eval2 에서 음성 `present` 0/299(상한 1.00 %), 진짜 `absent` 0/459, 진짜 `present` 하한 EPAL 6 x 95.22–96.08 %, T11 98.42–99.00 %.
- **Isaac(979dcd3, learned 원거리 인식, 공장 홀, seed 0–2):** 옆면 판정이 동작했다 — 진짜 EPAL 6 의 6 면 모두 첫 촬영 `present`, grounded 3/3 이 첫 면 `absent` 로 **삽입 전에 거부**. 같은 seed 의 검사 끔 대조는 grounded 를 3/3 집어 사이클을 마쳤다.
- **남은 문제:** 진짜 팔레트 3 건은 옆면 판정(`present`) 뒤 사이클을 마치지 못했다. 근거리 촬영 정렬(`near_capture_misaligned`, 측면 6.0·13.5 cm) 또는 삽입 게이트의 대상 대조(`bottom_deck_target_mismatch`, 중심 차 15.17 cm — 문턱 0.15 m 를 1.7 mm 넘음)에서 멈췄다. 옆면 다리로 더 움직이는 동안의 SLAM 추정 변화와 좌표 연결을 의심하지만(옆면 지점의 제어 추정이 truth 와 3–8 cm, s1 근거리 추정이 truth 와 13 cm), SLAM·좌표 변환·근거리 인식 오차의 몫은 분리하지 않았다. 다음 작업이다. `--bottom-deck-check` 기본값은 `off` 라 기존 동작은 바뀌지 않는다.

## 작업 1–4 — 판정·평가기·게이트·미션

| 작업 | 내용 | 커밋 |
|---|---|---|
| 1 | 옆면 판정 `bottom_deck_evidence.evaluate_side`, 옆면 장면 조리법 | `748f9a9` |
| 2 | 두 면 CPU 평가기·dev 조정·v1 eval | `ce0f714` |
| 3 | 기록 `BottomDeckRecord`·삽입 게이트(`approach → insert`) | `06fe859` |
| 4 | 미션의 옆면 다리·재촬영·복구 가로채기·재계산 도구 | `7f83aa8` |
| 5 준비 | 옆면 단계 경계(구역 밖 지점), grounded 물체 선언 | `fd16759`, `85cc2fe` |
| 5 | 면 찾기·v2 모형·근거리 절단 | `979dcd3` |

- **게이트:** 기록이 `present` 이고, 근거리 목표(계획 픽업)가 같은 형상·중심 0.15 m 안·진입축 15° 안일 때만 삽입한다. 사유는 `no_bottom_deck` / `bottom_deck_unobserved` / `bottom_deck_target_mismatch` 이다.
- **미션:** 첫 유효 원거리 관측 뒤 미션 계획 대신 옆면 다리를 먼저 간다. x 가설은 두 옆면을 θₑ 35°(카메라–중심 1.35 m)에서, T11 y 는 뒤 → 진입면을 정면에서 본다. 면마다 최대 3 번 찍고, 다시 찍을 때는 지점 뒤 0.3 m 로 계획된 후퇴를 거쳐 새로 도착한다. 첫 면이 `absent` 면 거기서 끝내고, `unobserved` 여도 둘째 면은 본다. 두 면 결론이 `present` 가 아니면 그 자리에서 끝낸다.
- **재계산:** `tools/recompute_bottom_deck.py <run>` 이 저장한 옆면 프레임으로 촬영·면 결론·두 면 정책·기록(형상·축·파라미터 해시·대상 중심과 yaw)을 다시 만들어 비교한다. Isaac 기록에서 정확히 재현됐다.

## CPU 평가

합격 기준(eval 한 번, 형상 × 가설 × 평활화 × K 12 조합마다): 음성 `present` 단측 95 % Clopper–Pearson 상한 ≤ 1 %(0/299), 진짜 `present` 하한 ≥ 90 %, 진짜 `absent` 상한 ≤ 1 %, 가림판·쌓임 `present` 0, 두 면이 실제 반대 면.

- 음성 299 는 grounded 단독이 아니라 음성 8 종(grounded, shelf, deckless, 이웃·낮은 판·턱 1·2·4 붙은 grounded)의 혼합이고, 순수 grounded 는 조합마다 38 장면이다. 네 틈 턱이 판별 불가 높이인 장면(운용 가정 밖)은 분모에서 빼 따로 기록했다.
- 같은 장면을 네 설정(평활화 × K)에서 다시 판정하므로, 행 수(예: eval2 9,524)는 독립 표본 수가 아니다.

| 판·대역 | 음성 present | 진짜 absent | 진짜 present (EPAL 6 x / T11 x / T11 y) | 결과 |
|---|---|---|---|---|
| v1 eval(`ce0f714`, 700000) | 0/299 | 0/459 | 294–295 / 292–296 / 298–299 | 12/12 합격 |
| v2 dev2(`979dcd3`, 400000) | 0/299 | 0/459 | 290–295 / 299 / 299 | 12/12 합격 |
| v2 eval2(`979dcd3`, 900000, 한 번) | 0/299 | 0/459 | 291–293 / 298–299 / 298–299 | 12/12 합격 |

- v2 오차 모형: 후보 오차는 면을 따라 ±100 mm, 법선 ±100 mm, yaw ±5°(장면 공통), 촬영마다 자세 오차 ±20 mm·±20 mm·±1°, 도착 오차 ±0.1 m·±5°, 카메라 기울기 ±1°·높이 ±10 mm, 팀 잡음판.
- 기록(합격 기준 아님, eval2): 가림판·쌓임 0/32 `present`, 판 한 장 32/32, 낮은 판 + 진짜 0–2/32, 네 틈 턱 + 진짜 4–15/32, 손상 + 턱 10–17/32 `present`(아래 한계), shelf 38/38 `unobserved`, 운용 가정 밖 grounded 0/1–5 `present`.
- v1 dev 기록: 법선 오차 30·45·60 mm 에서 T11 y 진짜 `present` 126–128 / 100–102 / 1–3 (/128), 오허가 0 — v2 의 면 찾기는 이 한계를 넓히려고 넣었다. 치수 공차 ±2 mm 는 진짜 60–64/64, 오허가 0.

## Isaac 미션

구성: M-c 미션 배치 실행(`--recognition far --recognition-proposals learned`, `--bottom-deck-check on`, `--slam-feedback`, 공장 홀 `factory_south_hall`), EPAL 6 또는 grounded(`--pallet-article grounded`, `sim/models/epal6_grounded/pallet.urdf`).

| 실행 | 첫 면 | 둘째 면 | 결과 |
|---|---|---|---|
| 진짜 s0 | +1 present (35.1°, 1.36 m) | −1 present | 근거리 촬영 `near_capture_misaligned`(측면 6.0 cm) |
| 진짜 s1 | +1 present | −1 present | 근거리 뒤 게이트 `bottom_deck_target_mismatch`(15 cm) |
| 진짜 s2 | −1 present | +1 present | 근거리 촬영 `near_capture_misaligned`(측면 13.5 cm) |
| grounded s0–2 | absent | — | `no_bottom_deck` 거부, 삽입 없음 |
| grounded, 검사 끔 s0–2 | — | — | 집고 사이클 완료(3/3) |

- 옆면 단계 시간(시뮬, 각 `result.json` 의 원거리 촬영 시각·옆면 촬영 시각): 원거리 촬영 뒤 첫 옆면 촬영까지 35–41 s, 두 면 84–91 s. grounded 는 첫 촬영 직후 거부된다. 측면 6.0·13.5 cm 는 각 `run.log` 의 `near_capture_misaligned` 값이다.
- 재계산: `tools/recompute_bottom_deck.py` 가 진짜 s0(`present`)·grounded s1(`absent`) 기록을 촬영·면·정책·기록까지 정확히 재현했다(문제 0).
- 처음 두 차례 실행에서 고친 것:
  - 관측 구역 경계(탐색 효율용, 벽이 아님) 밖에 +y 지점이 있어 시작 전 거부됐다 → 옆면 단계 전용 경계(지점·후보점 footprint 를 덮는 사각형, 홀로 자름).
  - 인식 카메라의 근거리 절단이 Isaac 기본 1.0 m 라 1.35 m 지점에서 팔레트 앞부분이 잘렸다 → 검사가 켜진 실행만 0.28 m(D435i 최대 해상도 기준 최소 거리, 포켓 카메라와 같은 값).
  - 옆면 촬영 순간의 자세 오차가 법선 4–9 cm 였다(평가 가정 ±30 mm) → 판정의 면 찾기와 v2 평가.

## 이 시험이 보이지 않는 것

- **사이클 완료:** 진짜 팔레트로 옆면 검사를 켠 사이클은 아직 한 번도 끝나지 않았다(위의 원인 미분리 문제 — SLAM·좌표 연결 의심).
- **실물:** 상한은 시뮬 모집단에 대한 조건부 값이다. 실물 grounded 모양·D435i 실측 잡음판 위 옆면 촬영은 하지 않았다.
- **운용 가정:** 네 틈 모두에 밑판 높이 물체를 둔 grounded, 밑판이 일부 빠진 손상 팔레트 + 그 틈의 턱은 두 면을 속일 수 있다(사전 명시 한계).
- **도달성·비용:** 양쪽 옆면에 차가 설 자리가 있어야 한다. 밀집 보관에서는 `unobserved` 로 집지 않는다.
- **재생:** `--bottom-deck-check on` 기록은 boundary replay 가 이름 붙은 오류로 거부한다(원거리 인식 경로가 원래 재생되지 않음).

## 검토 기록

- 작업 2 Codex 1–5차, 작업 3 1–2차, 작업 4 설계 1–5차·구현 1–3차, 작업 5 경계(A′) 1–2차, 면 찾기 설계 1차·구현 1–4차 — 모두 "새 P1/P2 없음" 으로 수렴. 지적과 처리는 계획 §7 에 있다.
- SLAM 연결(옆면 뒤 원거리 지점 복귀, 기록 다시 묶기)은 Codex 상의에서 "복귀 방향 동의, 다시 묶기 반대"로 갈렸고 사용자 결정으로 다음 작업이다.

## 시험

- ws1 전체 단위 + 러너 CLI 시험(`979dcd3`): 4,599 통과, 13 건너뜀, 실패 8 은 ws1 전용으로 알려진 것(mujoco 없음 6, 미커밋 요건 2).

## 재현

- CPU 평가: `tools/evaluate_bottom_deck.py run --band {dev,eval,dev2,eval2} --n 299 --records 32 [--shard i --of 4] --out rows.jsonl` → `summary rows*.jsonl --json summary.json`. 대역은 오차 모형만 고르고 판정은 체크아웃한 코드의 것이다 — v1 결과는 `ce0f714`, v2 결과는 `979dcd3` 에서 재현한다. v2 는 UBAI cpu2(16 CPU × 4 조각, `~/forklift-test-env`), v1 은 ws1 Slurm.
- Isaac: ws1 `/home/projects/forklift/test_runs/bd_isaac/bd.sbatch`(recog.sbatch 기반, `PALLET_URDF`), 스냅숏 `bd_979dcd3`, 결과 `c_real_on_s{0,1,2}`·`c_grounded_on_s{0,1,2}`·`b_grounded_off_s{0,1,2}`.
- 재계산: `tools/recompute_bottom_deck.py <run>/run`.
