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

## 결과

(실행 후 기록)
