# 네 번째 동결 평가 — 미사용 seed 4000–4029 (실행 기록)

상위: [실행 중 관측점 개발 기록](2026-10-03-runtime-viewpoints-dev-run.md)(97/99), [계획](../plans/2026-10-03-runtime-observation-viewpoints.md).
앞선 동결 평가: [G5](2026-10-02-g5-frozen-evaluation.md) 22/30, [두 번째](2026-10-02-second-frozen-evaluation.md) 26/30,
[세 번째](2026-10-03-third-frozen-evaluation.md) 25/30(모두 불합격).

## 공개 (결과를 보기 전)

- 3순위(인식 ↔ 운반 연결)의 **네 번째 동결 평가**다. 사용자 결정(2026-10-03, "돌려"). 앞의 세 평가 seed 는 이후 개발에 썼고 그 기록은 바꾸지 않는다.
- 개발 집합 97/99 는 성능 근거가 아니다. 99 seed 모두 진단·수정에 썼고, 앞의 세 번 모두 개발 집합 수치가 미사용 seed 에서 재현되지 않았다.
- 남은 알려진 실패 유형: 운반 단계(3003 추종, 3008 운반 목적지까지 격자 경로 없음), 검출 기각의 반복(실행 중 후보에서도 D 가 잦음), 실행 중 후보를 쓴
  실행의 시간 여유(113–132 s, 예산 150 s). 세 번째 평가에서 운반 단계 실패는 30 seed 중 2 개였다.

## 동결

| 항목 | 값 |
|---|---|
| 코드 | ws1 snapshot `snapshots/viewpoints_edfefb3`(개발 기록과 같은 snapshot, 파일 목록 해시 `26c00de9…`). main `6f95e64` 과 소스·시험·설정 차이 없음(문서만 다름) |
| 해시 | `run_transport.py` `dbb795b9…`, `observation_viewpoints.py` `c323dae8…`, `pallet_mission.py` `01de36d8…`, `pocket_detector.py` `7bd2a28c…`, `config/isaac_transport.yaml` `51f05655…` |
| 설정 | 개발 기록과 같다: 평면 예산 8, 세밀 격자·간격 4·2·1·갇힌 출발 폴백, 관측 도착 yaw 0.05 rad, cusp 제동 창 0.008, 고정 관측 후보 8 + 실행 중 후보 ≤ 6 |
| 인자 | `--obstacles 4 --max-sim-seconds 150 --use-perception --planning-target perception`, 잠정 차체 `dls08_provisional`, EPAL 6, bay |
| 장면·렌더 | `base_scene_warehouse_forklift/`, RaytracedLighting, ws1 RTX 5070 Ti, Isaac Sim 5.1 |

## 규칙

- **seed 4000–4029, 한 번만**, 순차, 한 Slurm 작업. 출력 `artifacts/20261003_fourth_eval/`.
- 미사용 확인(2026-10-03):
  - 로컬 `artifacts/` 와 ws1 `artifacts/`·`snapshots/` 에 `seed_40[0-2][0-9]` 디렉터리가 없다.
  - 로컬 `artifacts docs tools sim config deploy`, ws1 `artifacts` 의 JSON·로그·sbatch·셸·문서에 `--seed(s) 40xx`·`"seed": 40xx`·`seq 4000` 이 없다.
  - 문서에 나오는 4000–4029 는 이 평가를 위한 예약 문구뿐이다(계획 문서, `tools/observation_viewpoint_eval.py` 가 이 범위를 거부한다).
    `random.Random(20260927).sample(range(20, 10000), 40)` 표본에도 이 범위의 값이 없다.
  - 9/1 이후 ws1 Slurm 작업 이름에 이 범위를 쓴 작업이 없다.
- 재실행: `result.json` 없음·Slurm OOM/NODE_FAIL/CANCELLED·`phase == "startup"` 이면 같은 snapshot 으로 1 회(첫 시도 폴더는 `seed_*_crash_1`).
  그 외 실패는 결과.
- 판정: **분모 30**, 계획·캡처·추종·시간 한도 실패와 계열 A 도 실패. **28/30 이상(Clopper–Pearson 단측 95 % 하한 80.47 %)이 합격** — 앞의 세 평가와 같은 기준.
- 정답 진단(G3 도구)은 30 개 결과를 이 문서에 확정한 뒤에만 연다. 미달이면 3순위는 미완료로 남고, 이 결과로 구성을 고쳐 같은 seed 로 다시 평가하지 않는다.

## 결과 (정답 진단 전에 확정, 2026-10-03)

ws1 Slurm 작업 711, 23 분 5 초, 30 회 모두 `result.json` 있음, `phase == "startup"` 0, 재실행 0, 렌더 모드 30 회 모두 RaytracedLighting,
`run_transport.py` 해시 30 회 모두 `dbb795b9…`. snapshot 파일 목록 해시는 `__pycache__` 를 빼면 `26c00de9…` 로 사전 고정과 같다(개발 실행이 만든 바이트코드
27 개만 늘었다). 원본 `artifacts/20261003_fourth_eval/`(로컬·ws1).

**완주 27 / 30 — 합격선 28/30 미달(1 개 부족).** Clopper–Pearson 단측 95 % 하한 76.14 %(합격 기준 80 %). 시간 한도로 끝난 실행 0, 완주의 최대 시뮬 시간
142.7 s(4002), 계획 시간 최대 70.7 s(4000).

| 실패 단계(실행 기록의 사유 그대로) | seed | 사유 |
|---|---|---|
| 운반 추종 | 4007 | `Tracking failed in transport: pos=0.0091,yaw=-0.0577`. 후보 0 `no_pallet` → 후보 3 유효 뒤 접근·삽입·들어올림·인출 완료 |
| 유효 검출 뒤 운반 계획 | 4013 | 후보 4 에서 유효 뒤 `transport:no_path` — 기본 격자·간격 4·2·1 은 17,801 확장에서 탐색 공간 소진, 세밀 격자는 `expansion_limit` |
| 관측점 도달 | 4020 | 관측 후보 14 개(고정 8 + 실행 중 6) 모두 계획 실패: 고정 0–4 `invalid_goal`, 나머지 9 개는 폴백 사다리 전부 `no_path`(기본 격자 27 확장, 세밀 격자 64 확장) |

실행 중 후보를 쓴 seed 는 4009 하나이고 완주했다(고정 후보 0·4·6 `no_pallet` 뒤 후보 8 유효).

## 판정

**네 번째 동결 평가도 불합격. 3순위(인식 ↔ 운반 연결)는 미완료로 남는다.** 사전 등록대로 이 결과로 구성을 고쳐 같은 seed 로 다시 평가하지 않는다.
네 평가는 22/30 → 26/30 → 25/30 → 27/30 이다.
