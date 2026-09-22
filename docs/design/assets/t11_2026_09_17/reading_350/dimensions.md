# T11 ×0.6 — 350 mm 순폭 해석 (미채택)

**어느 쪽도 채택하지 않았다. 표준 원문 확인 필요.** 아래 값은 기존 YAML의 설계 선택값과
CLI로 지정한 순폭 해석의 조건부 유도값이다. 표준 인증치·실측치·출력 허용공차가 아니다.
도면: [drawing.svg](drawing.svg), [차이표](comparison.md), [동일 스키마의 해석 사본](geometry.yaml).

단위는 mm. 계산은 SI, 표시만 mm로 변환(m × 1000). 좌표 원점은 바닥 외형 중심,
x는 삽입 방향, y는 전면 폭, z는 위다. 전면 x=−D/2에서 +x로 진입한다.
근거: config/pallet_geometry_t11_06.yaml:19. SVG는 mm 치수값을 읽으며 인쇄 배율로 치수를 재지 않는다.

입력 축척 0.6 → 출력 축척 0.6.
입력 YAML 자체가 이미 축소되어 있으므로 기존 길이에는 --scale/--source-scale만 곱한다.
삽입구 CLI 값은 원형 후보치이므로 --scale을 한 번 곱한다.
기본 축척의 근거: docs/decisions/0002-test-pallet-and-geometry-generality.md:41.

입력 출처: `config/pallet_geometry_t11_06.yaml` (SHA-256 `4f4fc0386f067d34d055657780c620485de1f94808d12a7db03109a2c0c57287`).
기존 YAML 주석에는 원형 높이 **144 mm**가 있지만 실제 필드 90/0.6은 **150 mm**다
(config/pallet_geometry_t11_06.yaml:4; config/pallet_geometry_t11_06.yaml:24 (overall_height_m);
docs/decisions/0002-test-pallet-and-geometry-generality.md:194). 이 불일치를 해결하지 않았으며 실제 필드를 사용한다.
판재 두께·상판 개수·폭은 기존 설계 선택값이다. 모서리 형상·체결부·분할 접합부·재료·공차는 미정이다.

## 치수와 출처

| ID | 항목 | 값 (mm, 개수는 별도 표기) | 출처 / 유도식 |
| --- | --- | --- | --- |
| W | 전체 폭 y | 660 | config/pallet_geometry_t11_06.yaml:22 (overall_width_m) × (0.6/0.6) [--scale/--source-scale] |
| D | 전체 깊이 x | 660 | config/pallet_geometry_t11_06.yaml:23 (overall_depth_m) × (0.6/0.6) [--scale/--source-scale] |
| H | 전체 높이 z | 90 | config/pallet_geometry_t11_06.yaml:24 (overall_height_m) × (0.6/0.6) [--scale/--source-scale] |
| DB | 하부 판재 두께 | 15 | config/pallet_geometry_t11_06.yaml:27 (deck_bottom_m) × (0.6/0.6) [--scale/--source-scale] |
| BH | 블록 높이 / 로더 개구 높이 | 45 | config/pallet_geometry_t11_06.yaml:28 (block_height_m) × (0.6/0.6) [--scale/--source-scale] |
| ST | 스트링거 두께 | 15 | config/pallet_geometry_t11_06.yaml:29 (stringer_m) × (0.6/0.6) [--scale/--source-scale] |
| TT | 상부 판재 두께 | 15 | config/pallet_geometry_t11_06.yaml:30 (top_board_thickness_m) × (0.6/0.6) [--scale/--source-scale] |
| TW | 상부 판재 폭 | 82.5 | config/pallet_geometry_t11_06.yaml:45 (top_board_width_m) × (0.6/0.6) [--scale/--source-scale] |
| DT | 상부 덱 합계 (스트링거+상판) | 30 | ST + TT |
| O | 두 개구 각각의 순폭 (조건부) | 210 | CLI --opening-reading 350 mm × --scale 0.6; docs/decisions/0002-test-pallet-and-geometry-generality.md:195; docs/decisions/0002-test-pallet-and-geometry-generality.md:196 |
| B | 각 블록 폭 y = 깊이 x | 80 | (W − 2×O)/3; 같은 폭의 지지대 3개 가정; docs/decisions/0002-test-pallet-and-geometry-generality.md:195; docs/decisions/0002-test-pallet-and-geometry-generality.md:196 |
| BW | 하부 판재 각각의 폭 | 80 | B와 동일; src/forklift_core/perception/pallet_geometry.py:55 |
| BC | 블록 수 | 3 × 3 = 9 | config/pallet_geometry_t11_06.yaml:34 (block_widths_m); tools/build_pallet_model.py:34 |
| SC | 스트링거 수 | 3 | tools/build_pallet_model.py:34; 각 블록 x행에 하나 |
| NC | 하부 판재 수 | 3 | tools/build_pallet_model.py:34; 각 블록 y열에 하나 |
| TC | 상부 판재 수 (설계 선택값) | 7 | config/pallet_geometry_t11_06.yaml:44 (top_board_count) |
| TP | 상부 판재 중심 피치 | 96.25 | (W − TW)/(TC − 1) |
| TG | 상부 판재 사이 슬롯 | 13.75 | TP − TW |
| BZ | 블록 중심 z = 포켓 중심 z | 37.5 | DB + BH/2 |
| P | 포켓 중심 y 절댓값 | 145 | (B + O)/2 |
| PS | 포켓 중심 간격 | 290 | 2×P |
| PX | 전면 포켓 중심 x | -330 | −D/2; 삽입은 전면 −x에서 +x 방향 |
| OZ | 로더 개구 z 대역 | 15–60 | [DB, DB+BH]; src/forklift_core/perception/pallet_geometry.py:89 |
| FH | 정면 통로: 바닥부터 천장까지 | 60 | DB+BH; 정면 개구 아래 판재 없음; tools/build_pallet_model.py:34 |
| SH | 측면 개구 순높이 | 45 | BH; 측면 아래에는 하부 판재가 가로지름; tools/build_pallet_model.py:34 |
| BX | 블록 / 스트링거 x 중심 | -290, 0, 290 | [−(D−B)/2, 0, +(D−B)/2]; tools/build_pallet_model.py:34 |
| BY | 블록 / 하부 판재 y 중심 | -290, 0, 290 | [−(W−B)/2, 0, +(W−B)/2]; tools/build_pallet_model.py:34 |
| TY | 상부 판재 y 중심 | -288.75, -192.5, -96.25, 0, 96.25, 192.5, 288.75 | −(W−TW)/2 + i×TP; i=0…TC−1; tools/build_pallet_model.py:34 |

로더 개구 대역은 z=15–60 mm이고 높이는 BH다.
실제 정면 통로는 z=0–60 mm이며 아래 판재가 없다. 측면에는 아래 판재가 있으므로
개구 높이는 SH다. 정사각 외형과 같은 블록 배치만으로 덱까지 회전 대칭인 것은 아니다.
포켓 좌표는 (PX, ±P, BZ); 이는 로더 기준점이며 바닥까지 열린 통로의 면적 중심과 구별한다.

## 부품별 명목 치수와 배치

각 행은 조립 박스 하나다. size는 (x 길이, y 폭, z 높이), centre는 (x, y, z).
모든 위치와 모서리는 centre ± size/2로 재현한다. 판재의 방향과 아래 열린 통로를 바꾸지 않는다.

| 부품 | size (mm) | centre (mm) | 출처 / 유도식 |
| --- | --- | --- | --- |
| bottom_board_0 | 660, 80, 15 | 0, -290, 7.5 | size=(D,BW,DB); centre=(0,BY,DB/2); tools/build_pallet_model.py:34 |
| bottom_board_1 | 660, 80, 15 | 0, 0, 7.5 | size=(D,BW,DB); centre=(0,BY,DB/2); tools/build_pallet_model.py:34 |
| bottom_board_2 | 660, 80, 15 | 0, 290, 7.5 | size=(D,BW,DB); centre=(0,BY,DB/2); tools/build_pallet_model.py:34 |
| block_x0_y0 | 80, 80, 45 | -290, -290, 37.5 | size=(B,B,BH); centre=(BX,BY,BZ); tools/build_pallet_model.py:34 |
| block_x0_y1 | 80, 80, 45 | -290, 0, 37.5 | size=(B,B,BH); centre=(BX,BY,BZ); tools/build_pallet_model.py:34 |
| block_x0_y2 | 80, 80, 45 | -290, 290, 37.5 | size=(B,B,BH); centre=(BX,BY,BZ); tools/build_pallet_model.py:34 |
| block_x1_y0 | 80, 80, 45 | 0, -290, 37.5 | size=(B,B,BH); centre=(BX,BY,BZ); tools/build_pallet_model.py:34 |
| block_x1_y1 | 80, 80, 45 | 0, 0, 37.5 | size=(B,B,BH); centre=(BX,BY,BZ); tools/build_pallet_model.py:34 |
| block_x1_y2 | 80, 80, 45 | 0, 290, 37.5 | size=(B,B,BH); centre=(BX,BY,BZ); tools/build_pallet_model.py:34 |
| block_x2_y0 | 80, 80, 45 | 290, -290, 37.5 | size=(B,B,BH); centre=(BX,BY,BZ); tools/build_pallet_model.py:34 |
| block_x2_y1 | 80, 80, 45 | 290, 0, 37.5 | size=(B,B,BH); centre=(BX,BY,BZ); tools/build_pallet_model.py:34 |
| block_x2_y2 | 80, 80, 45 | 290, 290, 37.5 | size=(B,B,BH); centre=(BX,BY,BZ); tools/build_pallet_model.py:34 |
| stringer_0 | 80, 660, 15 | -290, 0, 67.5 | size=(B,W,ST); centre=(BX,0,DB+BH+ST/2); tools/build_pallet_model.py:34 |
| stringer_1 | 80, 660, 15 | 0, 0, 67.5 | size=(B,W,ST); centre=(BX,0,DB+BH+ST/2); tools/build_pallet_model.py:34 |
| stringer_2 | 80, 660, 15 | 290, 0, 67.5 | size=(B,W,ST); centre=(BX,0,DB+BH+ST/2); tools/build_pallet_model.py:34 |
| top_board_0 | 660, 82.5, 15 | 0, -288.75, 82.5 | size=(D,TW,TT); centre=(0,TY,H−TT/2); tools/build_pallet_model.py:34 |
| top_board_1 | 660, 82.5, 15 | 0, -192.5, 82.5 | size=(D,TW,TT); centre=(0,TY,H−TT/2); tools/build_pallet_model.py:34 |
| top_board_2 | 660, 82.5, 15 | 0, -96.25, 82.5 | size=(D,TW,TT); centre=(0,TY,H−TT/2); tools/build_pallet_model.py:34 |
| top_board_3 | 660, 82.5, 15 | 0, 0, 82.5 | size=(D,TW,TT); centre=(0,TY,H−TT/2); tools/build_pallet_model.py:34 |
| top_board_4 | 660, 82.5, 15 | 0, 96.25, 82.5 | size=(D,TW,TT); centre=(0,TY,H−TT/2); tools/build_pallet_model.py:34 |
| top_board_5 | 660, 82.5, 15 | 0, 192.5, 82.5 | size=(D,TW,TT); centre=(0,TY,H−TT/2); tools/build_pallet_model.py:34 |
| top_board_6 | 660, 82.5, 15 | 0, 288.75, 82.5 | size=(D,TW,TT); centre=(0,TY,H−TT/2); tools/build_pallet_model.py:34 |

## 포크 적합성 (정렬된 블레이드의 정적 기하)

| 입력 | 값 (mm) | 출처 |
| --- | --- | --- |
| fork_spacing_m | 290 | sim/models/dls08_provisional/parameters.yaml:24 (dimensions.fork_spacing_m) |
| fork_width_m | 55 | sim/models/dls08_provisional/parameters.yaml:25 (dimensions.fork_width_m) |
| fork_thickness_m | 24 | sim/models/dls08_provisional/parameters.yaml:26 (dimensions.fork_thickness_m) |
| fork_centre_height_m | 40 | sim/models/dls08_provisional/parameters.yaml:27 (dimensions.fork_center_height_m) |
| fork_length_m | 420 | rear_extent_x_m + overall_length_m − fork_root_x_m; sim/models/dls08_provisional/parameters.yaml:13 (dimensions.rear_extent_x_m); sim/models/dls08_provisional/parameters.yaml:8 (catalogue.overall_length_m); sim/models/dls08_provisional/parameters.yaml:23 (dimensions.fork_root_x_m) |
| lift_travel_m | 280 | sim/models/dls08_provisional/parameters.yaml:30 (dimensions.lift_travel_m) |

포크 치수는 사진 비율 추정이며 실측이 아니다: sim/models/dls08_provisional/parameters.yaml:5 (evidence.dimensions).
`check_fork_fit` 호출 결과 fits=True, lateral_ok=True, vertical_ok=True.
안쪽 여유 77.5 mm, 바깥쪽 여유 77.5 mm,
천장 여유 8 mm, 필요 승강 0 mm.
근거: src/forklift_core/perception/pallet_geometry.py:195; 안쪽=(포크 중심간격−포크 폭−B)/2,
바깥쪽=B/2+O−(포크 중심간격+포크 폭)/2, 천장=DB+BH−(포크 중심높이+승강+포크 두께/2).
함수의 기본 floor_clearance=4 mm는 로더 개구 하단 기준의 계산 조건이며
출력 공차가 아니다. 블레이드 도달률 0.6364=포크 길이/D는 캐리지 간섭을 검사하지 않는다.

## 기존 prior 범위 대조 (검출 실행 결과 아님)

| 항목 | 해석 명목값 (mm) | 기존 범위 (mm) | 범위 안 | 범위 밖 거리 (mm) | 출처 |
| --- | --- | --- | --- | --- | --- |
| opening_width | 210 | [190, 230] | True | 0 | config/pallet_prior_t11_06.yaml:9–11 (opening_width_range) |
| centre_spacer | 80 | [65, 95] | True | 0 | config/pallet_prior_t11_06.yaml:12–14 (centre_spacer_range) |

위 표는 고정된 기존 prior와 직접 비교한다. prior를 바꾸거나 실센서 검출을 실행한 결과가 아니다.
실제 검출기에는 격자 경계 보정도 있다: 중앙 지지대는 ±2×cell, 개구 하한은 −2×cell
(src/forklift_core/perception/pocket_detector.py:415;
src/forklift_core/perception/pocket_detector.py:526).
인쇄 후 측정치로 prior를 재생성해야 한다(docs/decisions/0002-test-pallet-and-geometry-generality.md:62).

## 삽입 깊이

D×0.6=396 mm,
캐리지 한계−여유=406−46=360 mm.
목표=min(두 값)=360 mm,
캐리지까지 남는 거리=46 mm.
출처: 사용자 지정 min(D×0.6, 406−46) mm; docs/decisions/0003-target-selection-and-blind-zone-insertion.md:139; 406 = 950−544; 46 = 406−360. 삽입 비율과 팔레트 축척은 별개다.
이는 ADR의 잠정 차체·정렬 자세 규칙 적용이며 궤적·접촉·변형·하중·실물 승강 검증이 아니다.

## 채택·출력 전에 확인

- 표준 번호·판본·도면을 특정하고 그 도면의 235 / 350 mm가 재는 대상을 확인한다.
  개구 순폭인지 블록 간 중심거리인지 확정하기 전에는 어느 쪽도 회귀 정본으로 사용하지 않는다.
  출처: docs/decisions/0002-test-pallet-and-geometry-generality.md:200.
- 원형 높이와 판재/블록 세부 치수는 표준 원문 확인 필요. 현재 판재 선택값을 표준치로 표시하지 않는다.
- 출력 공차: 장비·재료별 수축, 휨, 분할 접합 누적오차를 측정한다. 수치 공차는 아직 정하지 않았다.
  외형, 모든 부품의 size/centre, 개구 최소 폭·천장, 포켓 간격을 조립 후 검수한다.
- 적층 방향: 상판의 휨과 포크 접촉, 블록 접합부의 전단·층간 분리를 고려해 시험편으로 결정한다.
  이 도면에는 적층 방향·레이어 높이·인필·벽 수·접합 상세를 임의 지정하지 않았다.
- 하중 방향: 포크의 위쪽 지지력, 화물의 아래쪽 하중, 삽입 시 수평 접촉력을 나누어 확인한다.
  허용 하중·안전계수·내구 횟수는 미정이며 정적 기하 적합 판정으로 보증하지 않는다.
- 인쇄 전 실제 포크 두께·중심 높이·간격·승강 행정을 측정한다:
  docs/decisions/0002-test-pallet-and-geometry-generality.md:58.
