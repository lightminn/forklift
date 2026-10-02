# 세밀 격자 폴백·관측 끝점 yaw — Isaac 개발 집합 69 seed (실행 기록)

상위: [계획](../plans/2026-10-02-second-eval-failure-fixes.md) 검증 3. 비교 대상: [개발 집합](2026-10-02-planner-tracker-dev-run.md)(0–8·1000–1029, 37/39),
[두 번째 동결 평가](2026-10-02-second-frozen-evaluation.md)(2000–2029, 26/30). 변경: P4 세밀 격자 폴백(`81af1a5`), P3 관측 끝점 yaw 0.05(`98228e2`),
20260921 프로필 수정(`d7136fc`). P5(팔레트 기준 관측 후보)는 사용자 결정으로 뺐다.

## 사전 고정 (결과를 보기 전, 2026-10-02)

- 코드: 커밋 `d7136fc`(브랜치 `feat/fine-lattice-observe-yaw`), ws1 snapshot `snapshots/fix_d7136fc`(`git archive d7136fc`).
  `find src sim tools config -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum` = `309d6644…`. `run_transport.py` `01aa6542…`,
  `pallet_mission.py` `98237525…`, `path_tracking.py` `7d6a42a0…`(이전과 같음), `pocket_detector.py` `ce7ea377…`(같음),
  `config/isaac_transport.yaml` `51f05655…`(같음).
- 인자: 이전 두 실행과 같다(`--obstacles 4 --max-sim-seconds 150 --use-perception --planning-target perception`, 잠정 차체, EPAL 6, 관측 후보 8 개,
  평면 예산 6, `bay`, 복귀 없음).
- seed: **0–8, 1000–1029, 2000–2029 (69 개)**, 순차, 한 Slurm 작업. 2000–2029 는 두 번째 동결 평가에 한 번 쓴 seed 이며 이번에는 개발용이다(26/30
  기록은 바꾸지 않는다).
- 재실행: `result.json` 없음·Slurm OOM/NODE_FAIL/CANCELLED·`phase == "startup"` 이면 같은 snapshot 으로 1 회(첫 시도 폴더는 `seed_*_crash_1`). 그 외
  실패는 결과.
- 판정(개발 집합이므로 기록):
  - **해결 대상 5 개**: 2007(접근 격자), 1025(운반 격자), 2025(관측 구간 격자), 2023(검출 기각 뒤 관측 구간 격자), 2018(관측 끝점 yaw).
  - **비회귀 분모 63 개**: 직전 완주 seed 전부(개발 37 + 평가 26). 깨진 seed 는 원인 단계를 적고, 검출 판정 변화로 깨진 것도 변경의 간접 회귀로 센다.
  - 1028 은 P5 없이는 해결 대상이 아니다. 기대 최대치는 68/69 다.
  - 모든 관측 시도를 G3 도구로 분류하고, 폴백이 쓰인 탐색(실행기 `planning_traces`·후보 기록의 `search_attempts`)의 격자·간격을 센다.
- 세 번째 동결 평가는 이 기록의 범위가 아니다(사용자 결정).

## 결과 (2026-10-02)

ws1 Slurm 작업 684, 48 분 11 초, 69 회 모두 `result.json` 있음, `phase == "startup"` 0, 재실행 0, 렌더 모드 69 회 모두 RaytracedLighting,
`run_transport.py` 해시 69 회 모두 `01aa6542…`. 원본 `artifacts/20261002_fix_dev/`(로컬·ws1).

**완주 65 / 69.** 해결 대상 5 개 중 3 개 해소, 비회귀 분모 63 개 중 1 개 회귀.

| seed | 직전 | 이번 | 실행 기록 |
|---|---|---|---|
| 2007 | 접근 `expansion_limit` | **완주** | `approach_search` 기본 격자 `expansion_limit` → 세밀 격자 성공(P4). 삽입 오차 7.72 mm |
| 1025 | 운반 `no_path` | **완주** | `transport_search` 기본 `no_path` → 세밀 성공(P4). 7.73 mm |
| 2025 | 관측 후보 모두 계획 실패 | **완주** | 후보 5 관측 구간 기본 `no_path` → 세밀 성공(P4). 7.69 mm |
| 2018 | 관측 구간 추종 실패 | 실패(검출) | 관측 끝점 판정은 통과해 후보 5 에 도착(P3), 그 자리의 검출이 `invalid:opening_width_mismatch`. 나머지 후보는 `invalid_goal` |
| 2023 | 검출 기각 뒤 도달성 | 실패(검출) | 후보 4 관측 구간이 세밀 격자로 계획돼 도착(P4), 그 자리의 검출도 `no_pallet:no_opening_pattern` |
| 1028 | 관측 후보 모두 `invalid_goal` | 실패(같음) | P5 를 빼서 해결 대상이 아니다 |
| **1006** | 완주 | **실패 — 회귀** | `approach:no_path`. 아래 |

**1006 회귀.** 후보 1 로 가는 관측 구간이 직전에는 간격 2 경로였는데 이번엔 세밀 격자 경로가 되어(P4) 도착 yaw 가 −0.016 → −0.048 rad 로 바뀌었다(P3 의 0.05
안이라 도착으로 판정). 그 자세에서 접근 탐색은 모든 재시도가 **확장 1 회**에서 `no_path` 였다. 오프라인 재생: 출발 자세는 접근 여유 0.05 m 에서 비어 있지만,
`_sample_arc` 가 휩쓸린 영역을 감싸려고 더하는 여유(직선 +0.03, 곡선 +0.04 m) 안에 소품 3 이 들어와 여섯 원소가 모두 거부된다. yaw 를 −0.016 이나 0 으로만
바꿔도 0.1 s 에 계획되고, 충돌 단계 0.01 m 면 이 자세 그대로 0.3 s 에 계획된다. → 계획 개정 P6(`d73de25`).

**남은 실패의 성격.** 2018·2023 은 이번 변경으로 관측점에 닿은 뒤 검출에서 끝났다 — 계획·추종이 아니라 검출기 문제로 옮겨 갔다. 1028 은 관측 후보 위치 문제다.

**계획 시간.** `planning_wall_s` 최대 81.5 s(2016), 계획 중 시뮬레이션 시간은 흐르지 않는다.
