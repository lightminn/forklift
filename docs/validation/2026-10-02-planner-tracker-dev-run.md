# 계획기 폴백·전환점 제동 창 — Isaac 개발 집합 39 seed (실행 기록)

상위: [계획](../plans/2026-10-02-planner-tracker-robustness.md) 검증 2·3, 비교 대상 [G2′](2026-10-02-g2-prime.md)·[G5](2026-10-02-g5-frozen-evaluation.md).
변경: P2 전환점 제동 창 0.008 m(`f0392e4`), P1 탐색 폴백 4→2→1(`f1d369f`). 그 밖의 구성은 G5 동결 표와 같다.

## 사전 고정 (결과를 보기 전, 2026-10-02)

- 코드: 커밋 `f1d369f`(브랜치 `feat/planner-tracker-robustness`), ws1 snapshot `snapshots/robust_f1d369f`(`git archive f1d369f`).
  `find src sim tools config -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum` = `81853192…`(같은 방법으로 G5 의
  `g2prime_4c7780a` 는 `4524350a…` — G5 기록의 `b8ba0fdf…` 와 방법이 달라 값이 다르다). `run_transport.py` `4d3fe127…`,
  `pallet_mission.py` `e821bac8…`, `path_tracking.py` `7d6a42a0…`, `pocket_detector.py` `ce7ea377…`(G5 와 같음),
  `config/isaac_transport.yaml` `51f05655…`(G5 와 같음).
- 인자: G5 와 같다(`--obstacles 4 --max-sim-seconds 150 --use-perception --planning-target perception`, 잠정 차체, EPAL 6, 관측 후보 8 개,
  평면 예산 6). 계획기·추종 설정은 G5 동결 표에 P1·P2 만 더한 것이다.
- seed: **0–8 과 1000–1029, 39 개**, 순차, 한 Slurm 작업. 1000–1029 는 G5 로 이미 한 번 평가에 쓴 seed 이며, 이번에는 **개발용**이다
  (G5 의 22/30 기록은 바꾸지 않는다).
- 재실행: `result.json` 없음·Slurm OOM/NODE_FAIL/CANCELLED·`phase == "startup"` 이면 같은 snapshot 으로 1 회(첫 시도 폴더는 `seed_*_crash_1` 로 옮긴다). 그 외 실패는 결과.
- 판정(개발 집합이므로 합격/불합격이 아니라 기록):
  - **해결 대상 5 개**: G2′ seed 1(접근 계획), G5 1020·1029(접근 `expansion_limit`), 1011·1019(운반 전환점 yaw). 각각 해소 여부.
  - **비회귀**: 이전 완주 seed 30 개(G2′ 0·2–8, G5 22 개)가 그대로 완주하는지. 깨진 seed 는 원인 단계를 적는다.
  - **해결 대상이 아닌 실패** 1002(검출)·1028(관측점 도달 불가)·1025(운반 `no_path`)·1006(관측 구간 끝점 heading)은 실패해도 이
    주기의 결함으로 세지 않는다. 따라서 기대 최대치는 35/39 다.
  - 모든 관측 시도를 G3 도구(`tools.diagnose_detection`)로 분류한다. 폴백이 쓰인 탐색은 `planning_traces`·`search_attempts` 로 센다.
- 조건부 새 동결 평가: 완주 **36/39 이상**일 때만 seed 2000–2029 를 별도 사전 등록 뒤 한 번 돌린다(3순위의 **두 번째** 동결 평가임을
  공개). 미사용 확인(2026-10-02): 로컬 `artifacts/` 와 ws1 `artifacts/` 의 `result.json` 에 seed 2000–2029 가 없고, 문서에는 이 계획의
  예약 문구만 있다. 35/39 이하이면 새 평가 없이 이 기록으로 보고한다.

## 결과

(실행 뒤 기록)
