# 세션 장면 형식·공통 팔레트 장애물·truth 외곽 대조 — 연속 이송 CT-1 PR ② (1판)

브랜치 `feat/ct1-session-scene`(origin/main `cbe2d8b` = PR #97·#98 머지 뒤). 상위 계획 `docs/plans/2026-10-22-continuous-transport-ct1.md` 3절 PR ②. 사용자 결정 D1 = (가): 아직 인식하지 않은 이웃 팔레트는 공칭 외곽 + 0.25 m 다. 이 값은 경로 존재 상한일 뿐이고, 실차 SLAM 오차를 실측한 뒤 다시 정하는 임시값이다.

## 1. 사실 (소스)

- 장면 파일 v1(`sim/isaac/scene_file.py`, `riibotics_isaac_scene_v1`)은 미션 하나를 담는다. 장면 도구(`tools/riibotics_isaac_scenes.py`)는 k 번째 미션마다 장면을 만든다(`_sequence_scene`).
  - 이웃 팔레트는 `scenario.props` 의 소품으로 들어간다. 위치는 도면 자리 그대로이고, 기울기는 truth 다.
  - 미션 팔레트만 `PICKUP_ERRORS` 오차를 가진다. 공칭 yaw 는 0 이다(`scene_record`, 기울기는 truth 에만 있다).
- 순서:
  - 일자: 적재 [앞 333, 가운데 215, 뒤 95 px] → 하역 [가장 깊은 87, 205, 323 px], (0,0)(1,1)(2,2).
  - 랜덤: 적재 5(index 2·3·4 의 기울기 −8°/10°/−6°), 하역 3, (3,0)(4,1)(2,2).
- 오차(hall, dx, dy m, dyaw °):
  - 일자: line_1 (0.05, −0.06, 3), line_2 (−0.04, 0.05, −2), line_3 (0.03, 0.04, 2).
  - 랜덤: random_1 (0.06, −0.08, 0), random_2 (−0.07, 0.05, 0), random_3 (0.05, 0.07, 0).
- targets: 러너가 `scenario.pickup` 을 P-1 로 등록하고(5396행), live 는 D-0 도 등록한다. 이웃은 targets 에 없다.
- 계획 장애물:
  - 관측 다리는 소품 + `pickup_obstacle`(사전이면 픽업 구역, 인식·하역이면 추정 + 불확실도 — `preparation_obstacle`)이다.
  - 운반은 소품, 복귀는 소품 + 하역 사각형이다(`leg_obstacles` 957행).
  - live 준비(`live_prepare`)와 시작 판정(`prepare_start_ok`)은 `live_session_["scenario"]` 의 소품을 그대로 쓴다.
  - `prepare_start_ok` 는 홀드가 그 팔레트를 찾았을 때 받은 보정으로 미션 팔레트만 옮긴다(SLAM 해제 뒤 예상).
- teleop 허가 `zones_from(targets, pickup_zone)`: 사전 팔레트는 하나의 픽업 구역 + 0.182 + 0.01 m, 인식·하역 팔레트는 외곽 + 불확실도(0.21) + 0.01 m 다(`pallet_permit.py`).
- Isaac 에서 들 수 있는 팔레트는 `/World/Pallet` 하나다. 장면의 다른 팔레트는 정적 복사본이다(PR ③ 이 바꾼다).

## 2. 범위

이 PR 은 형식·함수·CPU 기하 검증까지 한다. v2 세션을 Isaac 에서 돌리는 것은 PR ③ 이다(이웃을 강체로 만들고 미션마다 손잡이를 바꿔야 한다). 러너에 넣는 부분은 v1 을 한 팔레트 세션으로 바꿔 같은 경로를 지나게 하고, 결과가 지금과 같음을 시험한다.

1. **세션 장면 형식 v2**(`riibotics_isaac_session_v1`, `sim/isaac/scene_file.py`).
   - 필드:
     - `name`, `pallet`(세션 형상 하나), `start_rear`, `bounds`
     - `static_props`: 팔레트가 아닌 계획 소품(벽·블록·랙)
     - `pallets`: [{`pallet_id`, `nominal`, `truth`, `pickup_zone`, `observation_waypoints`}]
     - `destinations`: [{`destination_id`, `x_m`, `y_m`, `yaw_rad`}] — '하역 칸 표지' 세션과 맞춘 형식(PalletSite 필드 + id)
     - `sequence`: [[pallet_id, destination_id], ...] — 도면의 순서이고 CPU 검증의 기본 순서다. 선택 규칙은 CT-2.
     - `visuals`: 팔레트를 뺀 것, `views`
   - 로더 `load_session`:
     - 팔레트별 형상 필드가 있으면 세션 형상과 같아야 한다(섞임 거부).
     - id 중복을 거부한다.
     - 순서는 있는 id 만 써야 하고, 한 팔레트를 두 번 고르면 거부한다.
   - `session_from_scene(SceneFile)`: v1 을 한 팔레트 세션으로 바꾼다.
     - P-1 = 장면의 픽업(nominal = `pickup_nominal` 이 있으면 그것, 없으면 `scenario.pickup`), D-1 = 장면 목적지.
     - static_props 는 장면 소품 그대로다(이웃 팔레트 소품 포함 — v1 의 이웃은 소품이다).
2. **공칭·truth 규칙**(장면 도구 `--session line|random`).
   - 공칭 = 도면 자리 + 도면 기울기를 hall 로 돌린 것(yaw 포함).
   - truth = 공칭 + 그 팔레트의 오차.
     - 일자: P-1/2/3 = line_1/2/3 의 오차.
     - 랜덤: P-4/P-5/P-3 = random_1/2/3 의 오차. 한 번도 옮기지 않는 P-1·P-2 의 오차는 (0.04, −0.03, 1°)·(−0.03, 0.04, −1°) 로 정한다(성분 최대 0.08 m 안).
   - 팔레트 id: 적재 index 순서로 P-1..P-n, 하역 칸은 하역 index 순서로 D-1..D-m.
   - 픽업 구역: 공칭 ± 0.25 m, yaw 12°(지금 규칙).
   - 관측 후보: 공칭에서 만든다. 공칭 yaw 에 기울기가 있으므로 후보도 그만큼 돈다 — v1 과 다른 점이다(v1 은 yaw 0 공칭에서 만들었다).
   - `visuals`: 팔레트 그림은 각 팔레트의 truth 자리에 그린다(PR ③ 이 강체로 바꿀 때 쓰는 값).
3. **공통 함수** `session_pallet_props(targets, *, exclude_id, nominal, shape, prior_grow_m, moved=None)`(`src/forklift_core/safety/pallet_permit.py` 옆, 순수):
   - 이웃 팔레트를 계획 소품(PlacedProp, 높이 = 팔레트 높이)으로 돌려준다.
     - 사전(basis prior): 공칭 외곽 + `prior_grow_m`(D1 0.25).
     - 인식·하역: 추정 외곽 + 불확실도(없으면 0.21 m — `RECOGNISED_MARGIN_M`, 지금 `preparation_obstacle` 과 같은 값).
     - 운반 중·자세 없음: 뺀다.
     - `exclude_id`(미션 팔레트)도 뺀다. 미션 팔레트는 지금처럼 `pickup_obstacle` 이다.
   - `moved`: 프레임 변경 함수(SLAM 해제 뒤 예상). 홀드가 찾은 팔레트만 옮긴다 — `prepare_start_ok` 의 미션 팔레트 규칙과 같다.
   - 쓰는 곳 넷이 모두 이 함수를 부른다:
     1. live 준비의 장면(`live_prepare` → `scenario_.props` = static + 이웃)
     2. `prepare_start_ok`
     3. SLAM 해제 뒤 예상(같은 함수에 `moved`)
     4. 계획 작업자 입력(준비 장면을 그대로 피클)
   - 채택 뒤의 운반·복귀 계획은 `options.scenario`(= 준비 장면)를 읽으므로 같은 소품을 본다.
4. **teleop 구역**: `zones_from(targets, zones)` 는 `zones` 를 `{pallet_id: Rectangle}` 으로 받는다(사전 팔레트별 구역). v1 은 `{"P-1": pickup_zone}` 이다. teleop 여유(0.182 + 0.01 m)와 계획기 D1(0.25 m)은 다른 계약이라 따로 둔다. 수동 복구의 면제(`recovery_context`)는 id 로 이미 고른다.
5. **truth 외곽 대조**(`tools/session_plan_check.py`, CPU):
   - 세션의 미션 열을 순서대로 계획만 한다. 미션 k:
     - targets 를 지금까지의 결과로 둔다: 앞 미션 팔레트는 하역 칸에 delivered, 추정 = 칸 자세, 불확실도 0.21.
     - 관측 → 접근·삽입·추출·운반·복귀를 Isaac 미션 설정(장면 도구의 STRAIGHTS·RESERVE·KAPPA·CLEARANCE)으로 계획한다. 관측의 미션 팔레트는 구역, 그 뒤는 truth 를 추정으로 쓴다(지금 `plan_scene` 과 같음).
     - 소품 = static + `session_pallet_props`(D1 0.25).
   - 대조:
     - 계획된 모든 다리의 자세마다 차체 외곽(관측·접근·삽입·복귀는 빈 외곽, 추출·운반은 실은 외곽 — `leg_obstacles` 의 footprint)과 이웃 팔레트 truth 외곽의 최소 거리를 잰다.
     - 사전 이웃은 truth 외곽, 하역된 팔레트는 칸 자세 외곽이다. 운반 중인 팔레트는 뺀다. 삽입·들기 중 미션 팔레트 자신은 대상이 아니다.
     - 미션별·다리별 최소 거리와 그 이웃 id 를 기록한다. 0 이하가 하나라도 있으면 그 미션은 `truth_contact` 이고, D1 을 다시 올린다.
   - 실행: UBAI(CPU 전용), 일자 3·랜덤 3.
6. **시험**:
   - 로더: 형식, 섞임·중복·순서 거부, v1 → 세션 변환.
   - 도구: 공칭에 기울기, truth = 공칭 + 오차, 세션의 미션 k 를 v1 장면 k 와 비교(같은 미션 팔레트·목적지·관측 순서, 이웃 소품의 차이는 기울기·오차뿐).
   - 공통 함수: 기저별 크기, 운반 중·미션 팔레트 제외, `moved` 는 찾은 팔레트만.
   - live 연결: v1 장면에서 준비 장면의 소품·시작 판정·작업자 입력이 지금과 같다(이웃 없음 → 소품 그대로).
   - `zones_from` 의 팔레트별 구역.
   - 대조 도구: 작은 합성 장면에서 거리 계산(겹침 0 이하, 떨어진 거리 정확).
   - 리플레이 게이트 29 건(배치 경로는 바뀌지 않는다).

## 3. 위험·범위 밖

- 관측 후보가 기울기만큼 돈다(랜덤 P-4 10°). CPU 계획으로 확인하고, Isaac 은 PR ④ 다.
- 하역된 팔레트의 truth 는 칸 자세로 둔다. 실제 하역 오차(Isaac 0.02–0.10 m)는 PR ④ 의 Isaac 에서 이웃 최소 거리로 기록한다.
- D1 0.25 m 는 경로 존재 상한이다. 대조 결과가 안전 여유를 증명하지는 않는다(SLAM 오차 0.14–0.16 m 는 대조에 없다).
- v2 의 배치 실행과 경계 기록은 거부한다.
