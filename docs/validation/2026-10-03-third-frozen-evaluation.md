# 세 번째 동결 평가 — 미사용 seed 3000–3029 (실행 기록)

상위: [평면 예산 8 개발 집합 기록](2026-10-02-plane-budget-eight-dev-run.md)(67/69), [세밀 격자 개발 기록](2026-10-02-fine-lattice-dev-run.md),
[두 번째 동결 평가](2026-10-02-second-frozen-evaluation.md)(26/30, 불합격), 첫 동결 평가 [G5](2026-10-02-g5-frozen-evaluation.md)(22/30, 불합격).

## 공개 (결과를 보기 전)

- 3순위(인식 ↔ 운반 연결)의 **세 번째 동결 평가**다. 사용자 결정(2026-10-03). 앞의 두 평가 seed(1000–1029, 2000–2029)는 이후 개발에 썼고,
  그 기록은 바꾸지 않는다.
- 개발 집합 67/69 는 성능 근거가 아니다. 69 seed 모두 진단·수정에 썼고, 검출은 관측 정지 자세의 mm 이하 변화에도 판정이 바뀐다.
- 개발 집합에 남은 실패 유형은 시점(2018), 고정 관측 후보 전부 막힘(1028), 관측 도착 여유 위험, 예산 8 의 혼잡 `grounded` 위양성(이 평가는
  양성 장면만이라 재지 않는다)이다. 앞의 두 평가에서 30 seed 당 관측점 도달성 실패가 1 개씩(1028, 2025) 나왔다.

## 동결

| 항목 | 값 |
|---|---|
| 코드 | ws1 snapshot `snapshots/budget8_eeffac1`(개발 집합 기록과 같은 snapshot). main `346c6f7` 과의 소스 차이는 `pocket_detector.py` 주석 2 줄뿐 |
| 해시 | `pocket_detector.py` `a0b6fe7b…`, `run_transport.py` `01aa6542…`, `pallet_mission.py` `01de36d8…`, `config/isaac_transport.yaml` `51f05655…` |
| 설정 | 개발 집합 기록과 같다: 평면 예산 8, 세밀 격자 폴백·간격 4·2·1·갇힌 출발 충돌 간격 0.01 m, 관측 도착 yaw 0.05 rad, cusp 제동 창 0.008 |
| 인자 | `--obstacles 4 --max-sim-seconds 150 --use-perception --planning-target perception`, 잠정 차체 `dls08_provisional`, EPAL 6, 관측 후보 8 개 |
| 장면·렌더 | `base_scene_warehouse_forklift/`, RaytracedLighting, ws1 RTX 5070 Ti, Isaac Sim 5.1 |

## 규칙

- **seed 3000–3029, 한 번만**, 순차, 한 Slurm 작업. 출력 `artifacts/20261003_third_eval/`.
- 미사용 확인(2026-10-03):
  - 로컬 `artifacts/` 와 ws1 `artifacts/`·`snapshots/` 에 `seed_30[0-2][0-9]` 디렉터리가 없다.
  - 로컬 `artifacts docs tools sim config deploy`, ws1 `artifacts` 의 JSON·로그·sbatch·셸·문서에 `--seed(s) 30xx`·`"seed": 30xx`·`seq 3000` 이 없다.
  - 문서에 나오는 seed 범위(0–999·0–199·200–399·1000–1029·2000–2029·100–129·0–24)와 `random.Random(20260927).sample(range(20, 10000), 40)`
    표본(3000–3029 에 드는 값 없음, 가장 가까운 값 3051)이 이 범위와 겹치지 않는다.
  - 9/1 이후 ws1 Slurm 작업 이름에 이 범위를 쓴 작업이 없다.
- 재실행: `result.json` 없음·Slurm OOM/NODE_FAIL/CANCELLED·`phase == "startup"` 이면 같은 snapshot 으로 1 회(첫 시도 폴더는
  `seed_*_crash_1`). 그 외 실패는 결과.
- 판정: **분모 30**, 계획·캡처·추종 실패와 계열 A 도 실패. **28/30 이상(Clopper–Pearson 단측 95 % 하한 80.47 %)이 합격** — 앞의 두 평가와 같은 기준.
- 정답 진단(G3 도구)은 30 개 결과를 이 문서에 확정한 뒤에만 연다. 미달이면 3순위는 미완료로 남고, 이 결과로 구성을 고쳐 같은 seed 로
  다시 평가하지 않는다(다음 평가는 별도 결정).
