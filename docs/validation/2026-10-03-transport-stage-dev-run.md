# 운반 단계 수정 — Isaac 개발 집합 129 seed (실행 기록)

상위: [계획](../plans/2026-10-03-transport-stage-fixes.md). 비교 기준: [실행 중 관측점 개발 실행](2026-10-03-runtime-viewpoints-dev-run.md)(작업 695,
97/99)과 [네 번째 동결 평가](2026-10-03-fourth-frozen-evaluation.md)(작업 711, 27/30) — 둘 다 `viewpoints_edfefb3`.

## 사전 고정 (결과를 보기 전, 2026-10-03)

- 코드: 커밋 `1cf9ac2`(브랜치 `feat/transport-stage`), ws1 snapshot `snapshots/transport_1cf9ac2`, 파일 목록 해시 `f439921f…`. `run_transport.py`
  `53f6ef00…`, `pallet_mission.py` `7f337736…`, `path_tracking.py` `0de86f81…`, `observation_viewpoints.py` `c323dae8…`(기준과 같음),
  `pocket_detector.py` `7bd2a28c…`(같음), `config/isaac_transport.yaml` `51f05655…`(같음). 바뀐 동작은 운반 cusp 재계획(T1)과 확장 사다리(T2)뿐이다.
- 인자는 기준과 같다. seed 0–8, 1000–1029, 2000–2029, 3000–3029, 4000–4029(129 개, 모두 이미 사용). 출력 `artifacts/20261003_transport_dev/`.
- 재실행: `result.json` 없음·Slurm OOM/NODE_FAIL/CANCELLED·`phase == "startup"` 이면 같은 snapshot 으로 1 회. 그 외 실패는 결과.
- 판정(계획 확인 3, 그대로 옮김):
  - 회귀: 기준 완주 124 개 중 실패로 바뀐 것이 0. 시간 한도로 끝난 것도 실패다.
  - 해결 대상 3003·4007·4013·4020 중 2 개 이상 완주하면 채택. 4007 은 시간 한도에 걸릴 가능성이 크다. 3008 은 대상이 아니다. 기대 최대 128/129, 현실적 기대
    127/129.
  - 대상마다 해결 경로(cusp 재계획·확장 사다리·관측점 변경)를, cusp 재계획이 일어난 seed 마다 횟수·오차·새 경로 길이·전환 수를, `planning_wall_s` 최대값을
    적는다.
- 모든 관측 시도를 G3 도구로 분류한다(같은 커밋의 git worktree 에서).
