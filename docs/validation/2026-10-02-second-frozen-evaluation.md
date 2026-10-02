# 두 번째 동결 평가 — 미사용 seed 2000–2029 (실행 기록)

상위: [계획](../plans/2026-10-02-planner-tracker-robustness.md) 검증 3, [개발 집합 기록](2026-10-02-planner-tracker-dev-run.md),
[CPU 기록](2026-10-02-planner-fallback-cpu.md). 첫 동결 평가: [G5](2026-10-02-g5-frozen-evaluation.md)(22/30, 불합격).

## 공개 (결과를 보기 전)

- 이것은 3순위(인식 ↔ 운반 연결)의 **두 번째 동결 평가**다. 첫 평가 G5 는 22/30 으로 불합격했고, 그 30 seed(1000–1029)는 이후 개발에 썼다.
  G5 기록은 바꾸지 않는다.
- 진입은 사전 등록한 기준(개발 39 seed 완주 36 이상)을 37/39 로 넘어서 이뤄졌다. 그러나 계획은 기대 최대치를 35/39 로 보았고, 문턱을 넘긴 2 건
  (seed 1·1002)은 P2 가 관측 정지 자세를 0.07–10 mm 바꾼 뒤 **검출 판정이 우연히 바뀐 결과**다. 개발 집합 수치는 성능 근거가 아니며, 이 평가가
  편향 없는 시험이다.
- 계획의 추정(남은 실패 유형 비율 4/30 이 유지되면 28/30 합격 확률 약 0.25 이하)을 바꿀 근거는 없다. 1025·1028 같은 구조적 실패가 남아 있다.

## 동결

| 항목 | 값 |
|---|---|
| 코드 | 커밋 `f1d369f`, ws1 snapshot `snapshots/robust_f1d369f`(개발 집합과 같은 snapshot; 해시는 [개발 집합 기록](2026-10-02-planner-tracker-dev-run.md) 사전 고정 절) |
| 설정 | G5 동결 표와 같고, 바뀐 것은 P2 `cusp_brake_window_m = 0.008`(모든 단계 추종기)과 P1 탐색 폴백(간격 8 실패 시 4 → 2 → 1, `expansion_limit`·`no_path` 만) 둘뿐 |
| 인자 | 개발 집합과 같다: `--obstacles 4 --max-sim-seconds 150 --use-perception --planning-target perception`, 잠정 차체 `dls08_provisional`, EPAL 6, 관측 후보 8 개, 평면 예산 6, `config/isaac_transport.yaml` `51f05655…` |
| 장면·렌더 | `base_scene_warehouse_forklift/`, RaytracedLighting, ws1 RTX 5070 Ti, Isaac Sim 5.1 |

## 규칙

- **seed 2000–2029, 한 번만**, 순차, 한 Slurm 작업.
- 미사용 확인(2026-10-02 17:05 KST):
  - 로컬 `artifacts/` 와 ws1 `artifacts/`·`snapshots/` 에 `seed_20[0-2][0-9]` 디렉터리가 없다.
  - JSON·로그·sbatch·셸·문서·jsonl 에 `--seed(s) 20xx`·`"seed": 20xx` 가 없다(로컬 `artifacts docs tools sim config`, ws1 `artifacts`).
  - 문서·스크립트에 나오는 seed 범위는 0–999·0–199·200–399·1000–1029·100–129·0–24·`range(20, 10000)` 표본 40 개(2000–2029 를 포함하지 않음)다.
  - 9/1 이후 ws1 Slurm 작업 이름 201 개에도 이 범위를 쓴 작업이 없다.
- 재실행: `result.json` 없음·Slurm OOM/NODE_FAIL/CANCELLED·`phase == "startup"` 이면 같은 snapshot 으로 1 회(첫 시도 폴더는
  `seed_*_crash_1`). 그 외 실패는 결과.
- 판정: **분모 30**, 계획·캡처·추종 실패와 계열 A 도 실패. **28/30 이상(Clopper–Pearson 단측 95 % 하한 80.47 %)이 합격.**
- 정답 진단(G3 도구)은 30 개 결과를 이 문서에 확정한 뒤에만 연다. 미달이면 3순위는 미완료로 남고, 이 결과로 구성을 고쳐 같은 seed 나 새 seed 로
  다시 평가하지 않는다(다음 평가는 별도 결정).

## 결과 (정답 진단 전에 확정, 2026-10-02)

ws1 Slurm 작업 679, 23 분 42 초, 30 회 모두 `result.json` 있음, `phase == "startup"` 0, 재실행 0, 렌더 모드 30 회 모두 RaytracedLighting,
`run_transport.py` 해시 30 회 모두 `4d3fe127…`(snapshot 과 같음). 원본 `artifacts/20261002_second_eval/`(로컬·ws1).

**완주 26 / 30 — 합격선 28/30 미달.** Clopper–Pearson 단측 95 % 하한 72.04 %(합격 기준 80 %).

| 실패 단계(실행 기록의 사유 그대로) | seed | 사유 |
|---|---|---|
| 유효 검출 뒤 임무 계획 | 2007 | `approach:expansion_limit` — 간격 8·4·2·1 모두 `expansion_limit`(각 30,000) |
| 추종 | 2018 | 관측 구간 추종 실패(pos 0.0299 m, yaw −0.0364 rad) |
| 검출 | 2023 | 관측 후보 소진, 첫 후보 `no_pallet:no_opening_pattern` 뒤 나머지 후보 계획 실패(`invalid_goal` 6, 후보 4 `expansion_limit`) |
| 관측점 도달 | 2025 | 관측 후보 8 개 모두 계획 실패(`invalid_goal` 5, 후보 5–7 간격 1 까지 `no_path`) |

## 판정

**두 번째 동결 평가도 불합격. 3순위(인식 ↔ 운반 연결)는 미완료로 남는다.** 사전 등록대로 이 결과로 구성을 고쳐 다시 평가하지 않는다.
첫 평가 G5(22/30)와 함께 보면, 이번 변경 뒤 실패 유형은 계획(접근 탐색 소진)·추종(관측 구간)·검출·관측점 도달로 G5 와 같은 네 갈래다.
