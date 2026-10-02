# G5 — 동결 구성의 미사용 30 seed 평가 (실행 기록)

상위: [closeout](../plans/2026-09-21-perception-detection-closeout.md) G2′·G5·§4, [G4 계획](../plans/2026-10-02-g4-observation-candidates.md),
[G2′ 기록](2026-10-02-g2-prime.md).

## 사용자 결정 (2026-10-02) — 범위 재결정 ㉰

G2′ 는 8/9 였다. 남은 seed 1 은 예산 6 으로 검출은 유효(4.32 mm)했지만 그 자리에서의 접근 계획이 실패했다(목표 수 mm 차이로 Hybrid A*
성패가 뒤집히는 계획기 민감성; 확장 한도를 늘려도 `no_path`). 사용자는 **이 실패를 "인식 뒤 계획기 실패" 로 기록하고 현재 구성을 동결해
G5 를 진행**하기로 했다. closeout §4 의 개발 집합 조건(9/9 또는 계열 A 예외)은 **충족되지 않은 채로** 진행하는 것이며, 그 사실을 여기와
closeout 에 남긴다. G5 의 분모(30, 계획·캡처·추종 실패 포함)와 합격선(28/30)은 바꾸지 않는다. 계획기 민감성은 별도 과제다.

## 동결 (결과를 보기 전)

| 항목 | 값 |
|---|---|
| 코드 | 커밋 `4c7780a`, ws1 snapshot `snapshots/g2prime_4c7780a`(src·sim·tools·config 파일 sha256 목록의 sha256 `b8ba0fdf…`). `run_transport.py` `6af4a20d…`, `pocket_detector.py` `ce7ea377…` |
| 설정 | `config/isaac_transport.yaml` `51f05655…`(접근 0.60 m/s 등) |
| `PlannerConfig` | curvature 0.5, xy 0.2 m, yaw 10°, primitive 0.25 m, collision step 0.06 m, max_expansions 30,000, analytic 8, heuristic 1.8, reverse 1.3, gear 1.0 m, steering 0.15, steering change 0.15 m, clearance 0.10 m, obstacle heuristic 없음 |
| 단계별 clearance | 관측·이동 0.10 m, 접근 0.05 m(`approach_clearance_m`) |
| 추종 | 사용자 결정 2026-10-02 의 현재 규칙: 관측 30 mm/30 mrad, 접근·추출·운반·이탈 8 mm/20 mrad, 삽입 8 mm/20 mrad 지나침 허용 없음, 전환점 30 mm/50 mrad, 그 외 지나침 30 mm. 속도: 관측·접근 0.60, 삽입 0.055, 추출 0.18, 운반 0.30, 이탈 0.12 m/s |
| 시간 한도 | `--max-sim-seconds 150`(주 루프 횟수), 단계 한도 max(30, 3·nominal + 10) s |
| 관측 후보 | 8 개: (−0.10, 0.90, 0) (−1.20, 0.30, 0) (−0.10, −0.60, 0) (−1.50, −0.60, 0) (−2.00, −0.30, 0) (0.00, 2.10, −0.25) (0.40, 1.20, 0) (−0.60, 1.80, −0.25) |
| `DetectorParams` | `derived_for(EPAL 6 prior)` + 기본값: max_plane_candidates **6**, range 0.8–5.0 m, min_plane_points 46, ransac 200, seed 20260913, median_plane_offset False … (G2′ seed 0 기록의 `detector_params` 전체) |
| 장면·자산 | `base_scene_warehouse_forklift/` (`scene.usda` `23488b6e…`, `forklift.usd` `453918c4…`, configuration 4 레이어), 장애물 4, 카탈로그 BarelPlastic_A_01·CardBoxA_02·CratePlastic_D_01(Isaac 5.1 Simple_Warehouse Props) |
| 렌더·GPU | RaytracedLighting(기본), ws1 RTX 5070 Ti, 드라이버 580.178.04, Isaac Sim 5.1 |
| 설계 도구 | `tools/observation_candidate_design.py`(커밋 `4c7780a`), 설계 결과 `artifacts/20261002_g4_candidate_design/design_200_400.json`, 기록 입력 G2 재실행 seed 0 `result.json` |
| 차체·팔레트 | 잠정 차체 `dls08_provisional`, EPAL 6 |

## G5 규칙 (결과를 보기 전)

- **seed 1000–1029, 한 번만.** 미사용 확인: 로컬·ws1 산출물의 `result.json` 에 seed 1000–1029 가 없고, 저장소 문서·코드에 이 범위를 쓴 실행이
  없다(9/17 계획 작업 1026 은 seed 0–999, 10/1 R1 은 0–199, 설계는 200–399).
- 인자: G2′ 와 같다(동결 표). 순차, 한 Slurm 작업.
- 재실행: `result.json` 없음·Slurm OOM/NODE_FAIL/CANCELLED·`phase == "startup"` 이면 같은 snapshot 으로 1 회. 그 외 실패는 결과.
- 판정: **분모 30**, 계획·캡처·추종 실패와 계열 A 도 실패. **28/30 이상(Clopper–Pearson 단측 95 % 하한 80.47 %)이 합격.**
- 정답 진단(G3 도구)은 30 개 결과를 이 문서에 확정한 뒤에만 연다. 미달이면 3순위는 미완료로 남고 새 seed 로 다시 시도하지 않는다.

## 결과 (정답 진단 전에 확정, 2026-10-02)

ws1 Slurm 작업 677, 18 분 10 초, 30 회 모두 `result.json` 있음, `phase == "startup"` 0, 재실행 0, 렌더 모드 30 회 모두 RaytracedLighting.

**완주 22 / 30 — 합격선 28/30 미달.** Clopper–Pearson 단측 95 % 하한 57.01 %(합격 기준 80 %). (처음 기록에 21/30·53.49 % 로 잘못 세었다가
Codex 재집계로 바로잡았다 — 판정은 같다.)

| 실패 단계(실행 기록의 사유 그대로) | seed | 사유 |
|---|---|---|
| 검출 | 1002 | 관측 후보 소진, `no_pallet:no_opening_pattern` |
| 관측점 도달 | 1028 | 관측 후보 8 개 모두 `invalid_goal` |
| 유효 검출 뒤 임무 계획 | 1020, 1029 | `approach:expansion_limit` |
| 유효 검출 뒤 임무 계획 | 1025 | `transport:no_path` |
| 추종 | 1006 | 관측 구간 추종 실패(pos 0.0294 m, yaw 0.0432 rad) |
| 추종 | 1011, 1019 | 운반 구간 추종 실패(pos 0.0299·0.0297 m, yaw 0.0539·0.0570 rad) |

- **유효 검출에 이른 seed 는 27 / 30** 이다(검출 전에 끝난 것: 1002 검출 실패, 1006 관측 이동 중 추종 실패, 1028 관측점 도달 불가).
  그 27 seed 의 인식 위치 오차(명목 대비)는 0.13–5.53 mm 다. 실패 8 개 중 6 개(1011·1019·1020·1025·1029 와 1006)는 검출이 아니라 계획·추종
  단계에서 끝났다(1006 은 첫 관측이 기각된 뒤 다음 관측점으로 가다 추종 실패).
- 결과 표는 실행 기록의 `failure_reason`·`planning_status` 를 옮긴 것이며, 정답을 쓴 원인 분류는 아래 진단 절에서만 한다.

## 판정

**G5 불합격. 3순위(인식 ↔ 운반 연결)는 미완료로 남는다.** closeout 규칙대로 새 seed 로 다시 시도하지 않는다.

## 정답 진단 (결과 확정 뒤, G3 도구)

`python -m tools.diagnose_detection artifacts/20261002_g5_frozen/perception/seed_*` (결과 `artifacts/20261002_g5_frozen/diagnosis/`).
관측 시도 41 개, 게이트·복제 루프·선택 재현·비변형 41/41, 실수 최대 차 8.9e-16, 정답 사슬 일치 최고 1.0.

**시도 단위:** OK 27, D(하부 증거) 4, C 6(상부 3·개구 2·선택 1), B(예산 소진) 3, A(가림) 1.

| seed·시도(후보) | 계열 | 전면 보임 / 화각 안 | 메모 |
|---|---|---|---|
| 1001·1 (1) | C 상부 | 1,617 / 1,617 | `upper_deck_occluded:left`; 다음 후보에서 OK, 완주 |
| **1002·1 (1)** | D 하부 | 1,861 / 1,861 | `lower` 부족 |
| **1002·2 (3)** | C 선택 | 1,584 / 1,585 | 전면 패턴이 최종 검증까지 유효였는데 다른 후보에 점수로 밀림 |
| **1002·3 (4)** | B 예산 소진 | 1,157 / 1,175 | → seed 1002 는 세 관측점에서 세 다른 계열로 실패 |
| 1004·1 (0) | C 개구 | 5,066 / 5,066 | 다음 후보에서 OK, 완주 |
| 1005·1/2/3 | B·A·B | 2,162·0/520·1,277 | 덧붙인 후보 5 에서 OK, 완주 |
| **1006·1 (0)** | C 상부 | 5,627 / 5,627 | 기각 뒤 다음 후보로 가다 관측 구간 추종 실패 |
| 1011·1 (0) | D 하부 | 5,886 / 5,886 | 다음 후보에서 OK, 운반 추종 실패 |
| 1017·1/2 | D·C 개구 | 1,388·789 | 후보 3 에서 OK, 완주 |
| 1023·1 (0) | C 상부 | 5,292 / 5,292 | 다음 후보에서 OK, 완주 |
| 1026·1 (0) | D 하부 | 6,836 / 6,836 | 다음 후보에서 OK, 완주 |

**임무 단위로 다시 묶으면** 실패 8 개 중 최종 종료 사유가 검출 후보 소진인 것은 **1002 하나**(D·C 선택·B)이고, 그 밖에 검출 기각 뒤 재관측 이동에서 추종이 실패한
1006 이다. 나머지는 관측점 도달 불가 1(1028), 유효 검출 뒤 계획기 3(1020·1029 접근 `expansion_limit`, 1025 운반 `no_path`),
추종 2(1011·1019 운반, yaw 0.054·0.057 rad 로 전환점 허용치 0.05 rad 초과)다. **최종 종료 사유가 검출 실패인 seed 는 1 개**다(관측 기각 자체는 9 개 seed 의 14 시도에서
있었고, 대부분 다음 관측점에서 회복했다). 1011·1019 의 실패 지점은 전환점이다(덤프의 `leg_end_is_cusp`).
G5 불합격의 대부분은 계획기와 추종기에서 왔다. 계열 A(가림) 예외에 해당하는 seed 는 없다(1005 는 A 시도가 있었지만 완주).

## 이 기록이 말하지 않는 것

- 계획기 민감성(1020·1029·1025 와 G2′ seed 1)과 운반 추종 yaw 초과(1011·1019)의 원인 — 별도 과제.
- 위양성, 실측 차체·T11·실물 카메라.
- 이 30 seed 결과로 구성을 다시 고치지 않는다(동결 평가는 1 회).
