# 계획기 폴백·전환점 제동 창 — CPU 검증 (실행 기록)

상위: [계획](../plans/2026-10-02-planner-tracker-robustness.md) 검증 1. 코드: P2 `f0392e4`, P1 `f1d369f`. 로컬 CPU, conda base Python 3.11.7,
`OMP_NUM_THREADS=1`. 폴백 끔은 같은 코드에서 `pallet_mission.FALLBACK_ANALYTIC_INTERVALS = ()` 로 만든다. 스크립트·원자료는 세션
스크래치(추적 안 함)에 있었고, 수치는 아래에 옮겼다.

## 시험

`tests/unit/control`·`tests/unit/planning`·`tests/unit/test_isaac_transport_planning_config.py` 318 통과, 진단 도구 시험 22 통과(커밋 뒤).
전체 `tests --ignore=tests/simulation` 에서는 `tests/integration/test_detector_v1_replay.py` 2 개가 실패하는데, 로컬에만 있는 v1 산출물
(`artifacts/20260912T17*`)이 있을 때만 돌고 **HEAD `81d755a` 에서도 같은 이유로 실패한다**(`median_plane_offset` 이 허용 목록에 없음,
`091722c` 이후). 이번 변경과 무관하다.

## bay 200 seed (복귀 포함, 명목 출발·목표)

| | 폴백 끔 | 폴백 켬 |
|---|---|---|
| 계획 성공 | 126 / 200 | **151 / 200** |
| 성공 → 실패 | — | 0 |
| 양쪽 성공 경로 동일(poses·directions·curvatures 전 단계 해시) | — | 126 / 126 |
| CPU 시간, 양쪽 성공 seed 평균 | 2.00 s | 1.96 s |
| CPU 시간, 구제된 25 seed | — | 평균 49.4 s, 최대 119.5 s |
| CPU 시간, 끝내 실패한 49 seed | 평균 9.7 s | 평균 61.6 s, 최대 179.4 s |

- 구제된 seed 의 탐색 간격: 8 이 48 탐색, 4 가 12, 2 가 5, 1 이 10. 구제된 경로의 전 단계 전환점 합은 평균 10.6, 최대 24(양쪽 성공 seed 는 평균
  5.1, 최대 29).
- 끝내 실패: `return_home:no_path` 25, `approach:no_path` 14, `return_home:expansion_limit` 6, `approach:expansion_limit` 3, `transport:no_path` 1.
- 처음 잰 값(16 병렬, 벽시계)은 평균 9.2 → 39.1 s 였는데, 재시도가 없는 seed 에서도 수십 배 차가 나 경합이 섞였다(Codex 지적). 위 표는
  10 병렬 `process_time` 이다.

## 섭동표 (G2′·G5 의 인계 36 개 × 목표 섭동 7 개, 간격 8)

섭동: 없음, x ±2 mm, y ±2 mm, yaw ±1 mrad. **227 / 252 → 247 / 252**, 성공 → 실패 0. 폴백 끔의 실패는 `approach:expansion_limit` 18·
`transport:no_path` 7, 켬의 실패는 **1025 의 `transport:no_path` 5 개뿐**이다. CPU 시간 평균 2.92 → 9.08 s, 최대 24.3 → 160.9 s.

## factory 60 seed (`tools/factory_planning_sweep.py`, 0–19 + 9/27 무작위 40)

6 조각 × 2 설정 병렬(12 프로세스, 코어 18), 시간은 도구의 벽시계.

| | 폴백 끔 | 폴백 켬 |
|---|---|---|
| 임무 계획 성공 | 39 / 60 | **44 / 60** |
| 성공 → 실패 | — | 0 |
| 양쪽 성공 39 개의 단계별 길이·확장 수 | — | 39 / 39 같음 |
| SLAM survey 성공 | 60 / 60 | 60 / 60 (폴백 밖, 영향 없음) |
| 개발 20 / 무작위 40 | 13 / 26 | 13 / 31 |
| 임무 계획 시간, 양쪽 성공 평균 | 26.0 s | 22.6 s |
| 임무 계획 시간, 끝내 실패한 16 seed | 평균 87.5 s | **평균 892.8 s, 최대 1,245 s** |

- 구제 5 개: 1179(`approach:expansion_limit`), 1188·1951·2354·5940(`return_home:expansion_limit`). 1951 은 944 s 가 걸렸다.
- 끝내 실패(켬): `return_home:expansion_limit` 13, `approach:expansion_limit` 2, `transport:no_path` 1.

## 판단

- 폴백은 간격 8 로 성공하는 탐색의 결과를 바꾸지 않는다(bay 126, factory 39, 단위 시험). 이득은 bay +25, 섭동표 +20, factory +5 다.
- 비용은 **끝내 실패하는 탐색**에 붙는다. 실패 탐색을 네 번 돌리므로 bay 는 약 6 배, factory 는 약 10 배가 되고, factory 의 넓은 홀에서는 실패
  seed 하나가 20 분에 이른다. Isaac 에서는 계획 중 시뮬레이션 시간이 흐르지 않아 단계 한도를 직접 소모하지 않지만, 작업의 벽시계와 Slurm 한도는
  늘어난다. factory Isaac 실행을 다시 돌릴 때는 작업 시간을 이에 맞춰 잡는다.
- `search_attempts` 는 시도별 (간격, 상태, 확장 수)만 남기고 시간은 넣지 않는다 — 시간을 넣으면 같은 입력의 계획 결과가 같지 않게 된다(기존
  비교 시험 5 개가 그 이유로 깨졌다).

## 이 기록이 말하지 않는 것

- Isaac 에서의 효과 — [개발 집합 기록](2026-10-02-planner-tracker-dev-run.md).
- 실패 탐색의 비용을 줄이는 방법(간격별 확장 한도 축소 등), 경로 품질(구제 경로의 전환점이 많다)의 개선.
