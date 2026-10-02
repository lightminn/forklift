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

## 결과

(실행 뒤 기록)
