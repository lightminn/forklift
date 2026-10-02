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

## 결과 (정답 진단 전에 확정, 2026-10-03)

ws1 Slurm 작업 689, 23 분 4 초, 30 회 모두 `result.json` 있음, `phase == "startup"` 0, 재실행 0, 렌더 모드 30 회 모두 RaytracedLighting,
`run_transport.py` 해시 30 회 모두 `01aa6542…`(snapshot 과 같음). 원본 `artifacts/20261003_third_eval/`(로컬·ws1).

**완주 25 / 30 — 합격선 28/30 미달.** Clopper–Pearson 단측 95 % 하한 68.10 %(합격 기준 80 %). 완주 25 개의 삽입 오차는 7.64–7.73 mm.

| 실패 단계(실행 기록의 사유 그대로) | seed | 사유 |
|---|---|---|
| 운반 추종 | 3003 | `Tracking failed in transport: pos=0.0167,yaw=0.0576`. 관측 후보 0 은 간격 1 까지 `expansion_limit`, 후보 1 에서 유효 검출 뒤 접근·삽입·들어올림·인출은 완료 |
| 유효 검출 뒤 운반 계획 | 3008 | 후보 3 에서 유효(후보 1 `invalid:no_upper_deck`, 후보 2 `no_pallet:no_opening_pattern`) 뒤 `transport:no_path` — 기본 격자·간격 4·2·1 은 10,394 확장에서 탐색 공간 소진(`no_path`), 세밀 격자는 `expansion_limit` |
| 검출 → 관측점 도달 | 3012 | 도달 가능한 관측 후보가 후보 4 하나(나머지 7 개 `invalid_goal`), 그 관측이 `no_pallet:no_opening_pattern` |
| 검출 → 관측점 도달 | 3015 | 후보 0 관측이 `invalid:opening_width_mismatch`, 나머지 7 후보 모두 `invalid_goal` |
| 검출 → 관측점 도달 | 3020 | 후보 0 관측이 `no_pallet:no_opening_pattern`, 나머지 7 후보 모두 `invalid_goal` |

## 판정

**세 번째 동결 평가도 불합격. 3순위(인식 ↔ 운반 연결)는 미완료로 남는다.** 사전 등록대로 이 결과로 구성을 고쳐 같은 seed 로 다시 평가하지 않는다.
세 평가는 22/30 → 26/30 → 25/30 이다. 개발 집합 67/69 와의 차이는 개발 집합이 진단·수정에 쓴 seed 라는 점과 맞는다.

## 정답 진단 (결과 확정 뒤, G3 도구)

`python -m tools.diagnose_detection artifacts/20261003_third_eval/perception/seed_*` 를 snapshot 과 같은 커밋 `eeffac1` 의 git worktree 에서
실행했다(main 의 `pocket_detector.py` 는 주석 차이로 출처 게이트를 통과하지 못한다). 결과 `artifacts/20261003_third_eval/diagnosis/`.
관측 시도 40 개, 게이트 40/40 통과, 정답 사슬 일치 최고 1.0.

**시도 단위:** OK 27, C 6(개구 4·증거 2), D(하부 증거) 4, A 2(가림 1·시야 밖 1), B(예산 소진) 1. 기각 13 시도 가운데 3007·3008·3011·3023·3024·3027·3028·3029 의
10 시도는 다음 관측점에서 회복했고, 회복하지 못한 것은 3012·3015·3020 이다.

**임무 단위(실패 5):**

- **3003** — 검출 문제 아님. 후보 1 에서 OK 뒤 접근·삽입·들어올림·인출을 마치고 운반 구간 추종에서 실패(pos 16.7 mm, yaw 57.6 mrad — yaw 가 경계를 넘음).
- **3008** — 후보 1 B(예산 8 도 소진), 후보 2 A(팔레트가 시야 밖) 뒤 후보 3 에서 OK. 그 뒤 운반 탐색이 기본 격자에서 10,394 확장으로 탐색 공간을
  다 써 `no_path`(간격 4·2·1 도 같음, 세밀 격자는 `expansion_limit`). 검출은 회복했고, 운반 목적지까지의 격자 경로가 없었다.
- **3012** — 도달 가능한 관측 후보가 후보 4 하나였고, 그 시점에서 팔레트가 다른 물체에 완전히 가렸다(A, 보이는 화소 0·가린 화소 2,833).
  계열 A 지만 규칙대로 실패로 센다.
- **3015** — 후보 0 이 C(개구 판정)로 기각, 나머지 7 후보 `invalid_goal`.
- **3020** — 후보 0 이 D(하부 증거 부족)로 기각, 나머지 7 후보 `invalid_goal`.

최종 종료 사유로 묶으면 **검출 기각 뒤 관측점 도달 불가 3**(3012·3015·3020), 운반 계획 1(3008), 운반 추종 1(3003)이다. 앞의 두 평가와 같이
관측점 도달성(고정 관측 후보가 막힘)이 가장 큰 실패 유형이고, 이번에는 첫 관측이 기각된 seed 에서 재관측할 곳이 없는 모양으로 나왔다. 개발 집합에서
고친 접근 탐색 소진과 관측 구간 추종 실패는 나오지 않았다. 운반 구간 실패 2 개(3003 추종, 3008 계획)는 앞선 개발 집합에서 남은 유형이 아니었다.

## 이 기록이 말하지 않는 것

- 실패 5 개의 원인 수정 방향(관측 후보 도달성, 운반 구간 계획·추종, 검출 기각)과 우선순위 — 별도 결정.
- 위양성(렌더 음성 모집단), 실측 차체·T11·실물 카메라.
- 이 30 seed 결과로 구성을 다시 고치지 않는다(동결 평가는 1 회).
