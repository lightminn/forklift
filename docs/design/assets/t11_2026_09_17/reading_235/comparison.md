# 두 순폭 해석의 차이 — 축척 0.6

**어느 쪽도 채택하지 않았다. 표준 원문 확인 필요.** 두 값은 CLI로 선택한 가설이며 표준 회귀 상수가 아니다.
단위 mm; 차이는 235 해석 − 350 해석. 부품 수·상판 배치는 입력 YAML 그대로이며
블록과 하부 판재 폭, 블록 깊이 및 이에 연동된 스트링거 길이와 중심 위치가 바뀐다.
각 ID의 뜻과 모든 원자료는 같은 실행의 dimensions.md / verification.json에서 확인한다.

| 항목 | 235 해석 | 350 해석 | 차이 | 출처 / 유도식 |
| --- | --- | --- | --- | --- |
| 전체 폭 y | 660 | 660 | 0 | W: config/pallet_geometry_t11_06.yaml:22 (overall_width_m) × (0.6/0.6) [--scale/--source-scale] |
| 전체 깊이 x | 660 | 660 | 0 | D: config/pallet_geometry_t11_06.yaml:23 (overall_depth_m) × (0.6/0.6) [--scale/--source-scale] |
| 전체 높이 z | 90 | 90 | 0 | H: config/pallet_geometry_t11_06.yaml:24 (overall_height_m) × (0.6/0.6) [--scale/--source-scale] |
| 하부 판재 두께 | 15 | 15 | 0 | DB: config/pallet_geometry_t11_06.yaml:27 (deck_bottom_m) × (0.6/0.6) [--scale/--source-scale] |
| 블록 높이 / 로더 개구 높이 | 45 | 45 | 0 | BH: config/pallet_geometry_t11_06.yaml:28 (block_height_m) × (0.6/0.6) [--scale/--source-scale] |
| 스트링거 두께 | 15 | 15 | 0 | ST: config/pallet_geometry_t11_06.yaml:29 (stringer_m) × (0.6/0.6) [--scale/--source-scale] |
| 상부 판재 두께 | 15 | 15 | 0 | TT: config/pallet_geometry_t11_06.yaml:30 (top_board_thickness_m) × (0.6/0.6) [--scale/--source-scale] |
| 상부 덱 합계 (스트링거+상판) | 30 | 30 | 0 | DT: ST + TT |
| 각 블록 폭 y = 깊이 x | 126 | 80 | 46 | B: (W − 2×O)/3; 같은 폭의 지지대 3개 가정; docs/decisions/0002-test-pallet-and-geometry-generality.md:195; docs/decisions/0002-test-pallet-and-geometry-generality.md:196 |
| 하부 판재 각각의 폭 | 126 | 80 | 46 | BW: B와 동일; src/forklift_core/perception/pallet_geometry.py:55 |
| 두 개구 각각의 순폭 (조건부) | 141 | 210 | -69 | 235: CLI --opening-reading 235 mm × --scale 0.6; docs/decisions/0002-test-pallet-and-geometry-generality.md:195; docs/decisions/0002-test-pallet-and-geometry-generality.md:196; 350: CLI --opening-reading 350 mm × --scale 0.6; docs/decisions/0002-test-pallet-and-geometry-generality.md:195; docs/decisions/0002-test-pallet-and-geometry-generality.md:196 |
| 포켓 중심 y 절댓값 | 133.5 | 145 | -11.5 | P: (B + O)/2 |
| 포켓 중심 간격 | 267 | 290 | -23 | PS: 2×P |
| 블록 중심 z = 포켓 중심 z | 37.5 | 37.5 | 0 | BZ: DB + BH/2 |
| 상부 판재 폭 | 82.5 | 82.5 | 0 | TW: config/pallet_geometry_t11_06.yaml:45 (top_board_width_m) × (0.6/0.6) [--scale/--source-scale] |
| 상부 판재 중심 피치 | 96.25 | 96.25 | 0 | TP: (W − TW)/(TC − 1) |
| 상부 판재 사이 슬롯 | 13.75 | 13.75 | 0 | TG: TP − TW |
| 정면 통로: 바닥부터 천장까지 | 60 | 60 | 0 | FH: DB+BH; 정면 개구 아래 판재 없음; tools/build_pallet_model.py:34 |
| 측면 개구 순높이 | 45 | 45 | 0 | SH: BH; 측면 아래에는 하부 판재가 가로지름; tools/build_pallet_model.py:34 |
| 블록 / 스트링거 x 중심 | -267, 0, 267 | -290, 0, 290 | 23, 0, -23 | [−(D−B)/2, 0, +(D−B)/2]; tools/build_pallet_model.py:34 |
| 블록 / 하부 판재 y 중심 | -267, 0, 267 | -290, 0, 290 | 23, 0, -23 | [−(W−B)/2, 0, +(W−B)/2]; tools/build_pallet_model.py:34 |
| 상부 판재 y 중심 | -288.75, -192.5, -96.25, 0, 96.25, 192.5, 288.75 | -288.75, -192.5, -96.25, 0, 96.25, 192.5, 288.75 | 0, 0, 0, 0, 0, 0, 0 | −(W−TW)/2 + i×TP; i=0…TC−1; tools/build_pallet_model.py:34 |
| 포크 안쪽 여유 | 54.5 | 77.5 | -23 | src/forklift_core/perception/pallet_geometry.py:195; 각 dimensions.md의 포크 계산식 |
| 포크 바깥쪽 여유 | 31.5 | 77.5 | -46 | src/forklift_core/perception/pallet_geometry.py:195; 각 dimensions.md의 포크 계산식 |
| 포크 천장 여유 | 8 | 8 | 0 | src/forklift_core/perception/pallet_geometry.py:195; 각 dimensions.md의 포크 계산식 |
| 깊이 비례 삽입 후보 | 396 | 396 | 0 | 사용자 지정 min(D×0.6, 406−46) mm; docs/decisions/0003-target-selection-and-blind-zone-insertion.md:139; 406 = 950−544; 46 = 406−360 |
| 삽입 목표 | 360 | 360 | 0 | 사용자 지정 min(D×0.6, 406−46) mm; docs/decisions/0003-target-selection-and-blind-zone-insertion.md:139; 406 = 950−544; 46 = 406−360 |
| 캐리지까지 잔여 | 46 | 46 | 0 | 사용자 지정 min(D×0.6, 406−46) mm; docs/decisions/0003-target-selection-and-blind-zone-insertion.md:139; 406 = 950−544; 46 = 406−360 |

포크 fits: 235=True, 350=True
(기존 check_fork_fit, 정렬된 잠정 블레이드 기하만).
기존 prior 개구 폭 범위 안: 235=False, 350=True.
기존 prior 중앙 지지대 범위 안: 235=False, 350=True.
출처: config/pallet_prior_t11_06.yaml:9–11 (opening_width_range); config/pallet_prior_t11_06.yaml:12–14 (centre_spacer_range).
