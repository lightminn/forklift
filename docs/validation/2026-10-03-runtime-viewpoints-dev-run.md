# 실행 중 관측점 — Isaac 개발 집합 99 seed (실행 기록)

상위: [계획](../plans/2026-10-03-runtime-observation-viewpoints.md). 비교 기준: [평면 예산 8 개발 실행](2026-10-02-plane-budget-eight-dev-run.md)
(작업 688, 67/69)과 [세 번째 동결 평가](2026-10-03-third-frozen-evaluation.md)(작업 689, 25/30). 두 기준은 같은 `budget8_eeffac1` 코드다.

## 사전 고정 (결과를 보기 전, 2026-10-03)

- 코드: 커밋 `edfefb3`(브랜치 `feat/runtime-viewpoints`), ws1 snapshot `snapshots/viewpoints_edfefb3`, 파일 목록 해시 `26c00de9…`.
  `run_transport.py` `dbb795b9…`, `observation_viewpoints.py` `c323dae8…`, `pallet_mission.py` `01de36d8…`(기준과 같음), `config/isaac_transport.yaml`
  `51f05655…`(같음), `pocket_detector.py` `7bd2a28c…`(기준 `a0b6fe7b…` 와 주석 2 줄만 다름). 바뀐 동작은 기본 관측 목록 뒤의 실행 중 후보뿐이다.
- 인자는 기준과 같다. seed 0–8, 1000–1029, 2000–2029, 3000–3029(99 개, 모두 이미 사용한 seed). 출력 `artifacts/20261003_viewpoints_dev/`.
- 재실행: `result.json` 없음·Slurm OOM/NODE_FAIL/CANCELLED·`phase == "startup"` 이면 같은 snapshot 으로 1 회. 그 외 실패는 결과.
- 판정(계획 확인 2·3):
  - 회귀: 기준에서 고정 후보 안에서 끝난 seed(실행 중 후보를 계획하지 않은 seed)의 성패가 바뀌면 회귀로 센다. 기준에서 센서 프레임 같은 변경 무관 사유로
    갈린 seed 는 사유를 적는다.
  - 해결 대상 3012·3015·3020 중 2 개 이상 완주 + 회귀 0 → 채택. 가능성 2018·1028 은 주장하지 않는다. 3003·3008(운반 단계)은 대상이 아니다. 기대 최대 97/99.
  - 실행 중 후보에 들어간 seed 마다 `planning_wall_s`, 관측 이동 거리, 최종 시뮬 시간을 적고, 시간 한도로 끝난 것은 해결로 세지 않는다.
- 모든 관측 시도를 G3 도구로 분류한다(같은 커밋의 git worktree 에서).
