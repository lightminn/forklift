# 세션 장면 형식·공통 팔레트 장애물·계획 포락선 대조 — 연속 이송 CT-1 PR ② (4판)

브랜치 `feat/ct1-session-scene`(origin/main `cbe2d8b` = PR #97·#98 머지 뒤). 상위 계획 `docs/plans/2026-10-22-continuous-transport-ct1.md` 3절 PR ②. 사용자 결정 D1 = (가): 아직 인식하지 않은 이웃 팔레트는 공칭 외곽 + 0.25 m 다. 이 값은 경로 존재 상한일 뿐이고, 실차 SLAM 오차를 실측한 뒤 다시 정하는 임시값이다.

## 1. 사실 (소스)

- **장면 파일 v1**(`sim/isaac/scene_file.py`, `riibotics_isaac_scene_v1`)은 미션 하나를 담는다.
  - JSON 에는 `pickup_nominal` 이 있다. 그런데 `SceneFile`·`load_scene` 이 그것을 버린다(38·80행).
  - 장면 도구(`tools/riibotics_isaac_scenes.py`)는 k 번째 미션마다 장면을 따로 만든다(`_sequence_scene`). 이웃 팔레트는 `scenario.props` 의 소품이고, 위치는 도면 자리·기울기 그대로다(오차 없음).
  - 미션 팔레트만 `PICKUP_ERRORS` 오차를 가지고, 공칭 yaw 는 0 이다(기울기는 truth 에만 있다).
- **순서**:
  - 일자: 적재 [앞 333, 가운데 215, 뒤 95 px] → 하역 [가장 깊은 87, 205, 323 px], (0,0)(1,1)(2,2).
  - 랜덤: 적재 5(index 2·3·4 가 −8°/10°/−6°), 하역 3, (3,0)(4,1)(2,2).
- **오차**(hall dx, dy m, dyaw °):
  - 일자: line_1 (0.05, −0.06, 3), line_2 (−0.04, 0.05, −2), line_3 (0.03, 0.04, 2).
  - 랜덤: random_1 (0.06, −0.08, 0), random_2 (−0.07, 0.05, 0), random_3 (0.05, 0.07, 0).
- **러너의 단일 팔레트 기준**:
  - targets 에는 truth 인 `scenario.pickup` 이 P-1 로 등록된다(5396행).
  - `live_session_` 은 `template/reference/zone/bounds` 를 하나씩만 가진다(6779행). `reference` 도 truth(`scenario.pickup`)다.
  - `live_prepare` 는 후보를 그 reference 에서 미션 팔레트 자세로 옮기고, 처음 자리인지는 자세가 정확히 같은지로 판정한다(6858행).
  - 이 기준을 nominal 로 바꾸면 random_1 을 D-1 에서 다시 집을 때 첫 후보가 0.680 m·10° 달라진다(Codex 두 검토 모두 재현).
- **구역의 세 표현**:
  - 파일의 `pickup_zone`(`PickupZone`)은 팔레트 중심의 배치 범위(공칭 ± 0.25 m)와 절대 yaw 상한(12°, 중심 0 — `observation_viewpoints.py` 140행)이다.
  - 러너는 여기에 팔레트 반대각선 0.5 m 를 더해 1.5 × 1.5 m 충돌 사각형을 만든다(3468행).
  - teleop(`zones_from`)은 그 사각형에 0.182 + 0.01 m 를 더 키운다.
- **targets 등록**: 팔레트와 목적지가 id 집합 `_issued` 를 함께 쓴다(61행). 인식·하역 팔레트의 teleop 구역은 팔레트 자체의 치수가 있어야 생긴다(`zones_from` 48행 — 치수 없이 등록하면 구역이 `[]` 다).
- **계획·실행이 읽는 장애물**:
  - 관측 다리는 소품 + `pickup_obstacle`, 운반은 소품, 복귀는 소품 + 하역 사각형이다(`leg_obstacles` 957행).
  - live 의 준비·시작 판정은 `live_session_["scenario"]` 의 소품이고, 채택 뒤에는 `options.scenario` 다.
  - 하역 도킹 보정의 dry-run 은 setup 때 소품 목록(`run_transport.py` 4646행 `obstacles` → `truth_obstacles`, core 5067·5257행)을 세션 내내 쓴다.
  - 근접 재관측의 기본 직선은 위치·각도 상자와 도착 오차만 본다(core 3499·3534행).
- **SLAM 해제 뒤 예상**: `prepare_start_ok` 는 홀드가 찾은 추정만 옮긴다(`_held_estimates`, core 6061·6087행 — 그 미션의 팔레트·목적지).
- **계획 단계**: `STAGES` = approach·insert·extract·transport·withdraw·return_home(+ 관측)이다. `collision_free_path` 는 표본 사이 운동을 이동량 패딩으로 덮는다(`geometry.py` 200행).
- Isaac 에서 들 수 있는 팔레트는 `/World/Pallet` 하나다. 장면의 다른 팔레트는 visual 마다 만든 정적 collider 다(`add_scene_pallets`).

## 2. 범위

- 이 PR 은 형식, 등록·준비 입력 계약, 공통 함수, CPU 대조까지 한다.
- 러너는 v1·bay 의 P-1 준비 입력을 지금 값 그대로 팔레트별 구조(2.4)에 넣는다.
- 세션 어댑터(`session_from_scene`·`register_session`)를 러너에 연결하는 것은 v2 실행과 함께 PR ③ 이 한다.
  - 지금 연결하면 실행 값은 같으면서 리플레이 기록의 setup 이름만 건드린다.
  - 이 PR 은 어댑터가 v1 열 장면 모두에서 러너와 같은 등록·준비 입력(기준 자세·후보·구역 사각형)을 냄을 시험으로 고정한다.
- v2 세션을 Isaac 에서 돌리는 것은 PR ③ 이다(이웃 강체, 미션별 손잡이, 아래 4절의 인계 항목).

### 2.1 세션 장면 형식 v2

형식 이름은 `riibotics_isaac_session_v1`, 모듈은 `sim/isaac/scene_file.py` 다.

- 필드:
  - `name`, `pallet`(세션 형상 하나), `start_rear`, `bounds`
  - `static_props`: 팔레트가 아닌 계획 소품
  - `pallets`: [{`pallet_id`, `nominal`, `truth`, `zone`, `observation_waypoints`}]
    - `zone`: {`x_min_m`..`y_max_m`(중심 배치 범위), `yaw_centre_rad`, `yaw_half_rad`}
  - `destinations`: [{`destination_id`, `x_m`, `y_m`, `yaw_rad`}] — '하역 칸 표지' 세션과 맞춘 형식
  - `sequence`: [[pallet_id, destination_id], ...]
  - `visuals`: 팔레트가 아닌 것만, `views`
- **v2 팔레트의 초기 배치 정본은 `pallets[].truth` 하나다.** Isaac 의 팔레트 그림·강체는 그것에서 만든다(PR ③).
- 로더 `load_session`:
  - 형상 섞임 거부
  - 팔레트·목적지 id 를 합친 집합에서 중복 거부
  - 순서는 있는 id 만 쓰고, 한 팔레트를 두 번 고르지 않고, 한 목적지를 두 번 쓰지 않는다.
- `SceneFile` 에 `pickup_nominal`(있으면)을 남긴다.
- **v1 변환 `session_from_scene(SceneFile)`** — v1 동작을 그대로 둔다:
  - P-1 은 등록 자세 = 후보 이동 기준 = `scenario.pickup`(지금의 truth 등록 그대로)이다. truth = `scenario.pickup`, 후보 = 파일 후보, 구역 = 파일 구역(`yaw_centre_rad` 0)이다.
  - nominal 은 `pickup_nominal` 이다(기록만 — v1 소비자는 읽지 않는다).
  - D-1 = 장면 목적지, live 는 D-0 = P-1 등록 자세(지금과 같음)다.
  - static_props 는 장면 소품 그대로다(v1 의 이웃 팔레트 소품을 포함한다 — 호환 예외).
  - 팔레트별로 `registered`(targets 등록·후보 이동 기준)를 둔다. v1 은 `scenario.pickup`, v2 는 `nominal` 이다.

### 2.2 공칭·truth 규칙(장면 도구 `--session line|random`)

- 공칭 = 도면 자리 + 도면 기울기를 hall 로 돌린 것이다.
- truth = 공칭 + 그 팔레트의 오차.
  - 일자: P-1/2/3 = line_1/2/3.
  - 랜덤: P-4/P-5/P-3 = random_1/2/3. 옮기지 않는 P-1·P-2 는 (0.04, −0.03, 1°)·(−0.03, 0.04, −1°).
- id: 적재 index 순서로 P-1..P-n, 하역 index 순서로 D-1..D-m. 순서: 일자 P-1:D-1, P-2:D-2, P-3:D-3 / 랜덤 P-4:D-1, P-5:D-2, P-3:D-3.
- 구역: 공칭 ± 0.25 m, `yaw_centre_rad` = 공칭 yaw, `yaw_half_rad` = 12°.
  - 공칭 yaw 10° 에서 hall (+0.25, +0.25) m 는 팔레트 축 방향 0.2896 m 다. 공칭 외곽 + 0.25 m(D1)는 구역 전체를 덮지 않는다(상위 계획 D1 의 조건 그대로).
- 관측 후보는 공칭(기울기 포함)에서 만든다. v1 보다 random_1 첫 후보가 약 0.593 m 움직인다(CPU 계획으로 확인, Isaac 은 PR ④).

### 2.3 targets 등록 계약 `register_session(targets, session, *, kind, depth_m, width_m, live)`

- 모든 팔레트를 `registered` 자세, 실제 치수(형상 파일), 러너의 형상 키 `kind`(`Path(args.pallet_geometry).stem`)로 등록한다. 목적지 D-k 도 등록한다.
- live 는 v1 에서만 D-0 을 더한다(v2 의 처음 자리는 CT-2).
- v1 변환에 대해 지금 러너와 같은 호출을 낸다(같은 id·자세·치수·`at`·revision).
- 치수 없는 등록은 하지 않는다(teleop 구역이 사라진다).

### 2.4 팔레트별 준비 입력

`live_session_` 을 팔레트 id → {`template`, `registered`, `zone_rect`, `bounds`} 로 바꾼다.

- `live_prepare`·`prepare_start_ok` 는 미션 `pallet_id` 의 것을 쓴다.
- 처음 자리 판정은 그 팔레트의 `registered` 와 정확히 같은지로 한다(지금과 같은 규칙).
- 후보 이동 기준도 그 팔레트의 `registered` 다.
- v1 은 P-1 하나라 지금과 같다.
- 원거리 인식 구역(PR ①의 `far_zone`)과 truth 목적지도 팔레트·목적지 id 로 찾는다.

### 2.5 구역 변환(한 곳)

- `zone_rect(zone, depth, width)` = 중심 범위 + 반대각선 → 충돌 사각형(지금 3468행의 식, yaw 0)이다.
- teleop 은 `zones_from(targets, {pallet_id: zone_rect})` 로 받는다. 여유(0.182 + 0.01)는 `zones_from` 이 그대로 더한다.
- 계획기 D1(0.25)과 teleop 여유는 별도 계약이다.

### 2.6 공통 함수 `session_pallet_props(targets, *, exclude_id, nominal, shape, prior_grow_m, moved=None, moved_ids=())`

순수 함수이고 위치는 `pallet_permit.py` 옆이다.

- 이웃 팔레트를 계획 소품(PlacedProp, 높이 = 팔레트 높이)으로 돌려준다.
  - 사전: 그 팔레트 `nominal` 외곽 + `prior_grow_m`(D1 0.25).
  - 인식·하역: targets 자세 외곽 + 불확실도(없으면 0.21).
  - 운반 중·자세 없음과 `exclude_id` 는 뺀다.
- `moved`·`moved_ids`: 프레임 변경은 `moved_ids` 에 든 id 만 받는다.
  - 그 집합은 코어가 홀드에서 찾은 추정의 id 다(`_held_estimates` 를 id 로 돌려주도록 넓힌다).
  - 실제 해제와 미리보기가 같은 판단을 쓴다. `basis` 로 고르지 않는다.
- 쓰는 곳:
  1. live 준비 장면(`scenario_.props` = static + 이웃)
  2. `prepare_start_ok`
  3. 해제 뒤 예상(`moved`)
  4. 작업자 입력(준비 장면 피클)
- 채택 뒤 운반·복귀는 `options.scenario` 로 같은 소품을 본다.
- live 하역 도킹 보정의 dry-run 은 러너의 `obstacles` 목록에 세션 이웃 소품을 합친 것을 본다.
  - `obstacles` 목록에는 실행 중 생기고 지워지는 장애물이 들어 있다(`run_transport.py` 7851·7868행, `--new-obstacles`). 그래서 목록을 바꿔 끼우지 않는다.
  - 소유는 둘로 나뉜다. 세션 이웃은 준비 장면이 갖고, 생성 장애물은 러너가 갖는다.
  - P-1 하나뿐(v1)이면 이웃이 없어 지금 목록 그대로다.
- 소품은 준비 때 고정한다. 미션 중 다른 팔레트는 바뀌지 않는다 — 동시 미션 거부, 준비 채택 전 targets revision 검사. 미션 중 이웃 갱신이 생기면 재계획 계약이 따로 필요하다.

### 2.7 계획 포락선 대조(이상적 추정·하역) `tools/session_plan_check.py`

CPU 에서 돈다.

- **범위**: 파일 순서대로, 매번 `start_rear` 에서 출발해 그리로 돌아온다. 앞 미션은 이상적으로 하역됐다고 본다. 목적지는 순서상 비어 있다.
  - 미션 k 는 앞 미션들이 계획·대조를 통과했을 때만 계획한다. 아니면 `prior_failed` 다.
  - 결과 이름은 "지정 순서·완전 복귀·이상적 하역의 미션 열" 이다.
- **미션 k 의 상태**: 앞 팔레트는 칸 자세에 delivered(불확실도 0.21), 나머지는 prior 다.
  - 관측의 미션 팔레트는 구역 충돌 사각형이고, 그 뒤 계획의 추정은 truth 다(지금 `plan_scene` 과 같음).
  - 소품 = static + `session_pallet_props`(D1 0.25).
- **단계표**:

| 단계 | 외곽 | 미션 팔레트 | 이웃 |
|---|---|---|---|
| 관측 | 빈 외곽 | truth 외곽 포함 | truth |
| 접근 | 빈 외곽 | truth 외곽 포함(면 앞 0.10 m 에서 멈춤 — 계획기도 검사한다, `pallet_mission.py` 298·1676·1691행) | truth |
| 삽입 | 빈 외곽 | 제외 | truth |
| 추출 | 실은 외곽 | 실림(제외) | truth |
| 운반 | 실은 외곽 | 실림 | truth |
| 후퇴(withdraw) | 빈 외곽 | 제외(포크가 포켓 안) | truth |
| 복귀 | 빈 외곽 | 칸 자세 외곽 포함 | truth |

  - 이웃 truth: 사전은 `truth`, 하역된 것은 칸 자세다.
  - 외곽은 계획기 `Footprint` 다(차체 + 포크 포락선 — 실제 접촉 판정보다 보수적).
- **거리**:
  - 다리 자세마다 외곽과 각 truth 외곽의 최소 거리를 재고, 이웃 id 와 함께 기록한다(진단값).
  - 판정: 연속 구간을 쓴다.
    - 연이은 두 자세 사이는 첫 자세의 방향에 접하고 둘째 자세를 지나는 원호(방향이 같으면 직선) 위에서 외곽 점의 이동이 h 이하가 되게 나누어 잰다(Codex 구현 1차: 현 위 직선 보간은 반원 반례를 놓쳤다).
    - 도는 쪽은 계획기의 주행 방향(`directions`, 둘째 자세에 도착하는 값)으로 정한다. 끝 yaw 의 정규화 부호(±π)로 정하지 않는다(Codex 구현 2차). 방향 없이 반원처럼 모호한 구간은 거부한다.
    - 하한 = 표본 최소 − h/2 다. 원호 위 표본이라 처짐 항은 없다. h 는 처음 5 mm, 재분할하면 0.5 mm 다.
    - 표본 최소 ≤ 0 이면 `contact_observed` 다.
    - 하한 ≤ 0 이지만 표본 최소 > 0 이면 그 구간을 0.5 mm 로 다시 나눈다. 그래도 하한 ≤ 0 이면 `clearance_unproven` 이다(접촉이 확인된 것이 아니다).
  - `contact_observed`·`clearance_unproven` 은 원인부터 분류한다(사전 이웃 → D1 재상정, 하역 팔레트·단계 문제 → D1 과 무관).
- **계획 설정**: 지금 `plan_scene` 과 같은 geometry·planner 를 쓴다.
  - `mission_geometry(URDF, pallet, RESERVE_M 0.016, alignment 2.1·delivery 1.5·withdrawal 0.55)`
  - `make_transport_planner_config(κ 0.29, clearance 0.10, max_expansions 30000, D7_PLANNER_OPTIONS)`
  - 결과에 실제 적용값을 남긴다.
- 실행: UBAI(CPU 전용), 일자 3·랜덤 3.

### 2.8 시험

- 로더: 형식, 섞임·중복(팔레트+목적지)·순서 거부, `pickup_nominal` 보존.
- v1 변환: P-1 의 `registered`·후보·구역이 지금과 같다. **하역 뒤 다시 집기의 live 입력**(후보·구역·처음 자리 판정)도 지금 코드와 같다.
- 도구: 공칭 기울기, truth = 공칭 + 오차, 팔레트 visual 없음, 세션 미션 k 와 v1 장면 k 의 관계(미션 팔레트·목적지 같음, 후보 차이는 기울기뿐).
- 등록: v1 변환이 지금과 같은 targets(id·자세·치수·`at`·revision)를 낸다. v2 는 모두 치수가 있다.
- 공통 함수:
  - 기저별 크기, 운반 중·미션 팔레트 제외.
  - `moved_ids` 밖 이웃은 안 움직인다.
  - 치수 없는 인식 팔레트는 등록 단계에서 막힌다.
- `zones_from` 팔레트별 구역, `zone_rect` 가 지금 식과 같다.
- 대조 도구:
  - 합성:
    - 겹침·정확한 거리.
    - 표본 사이 접촉(두 끝 0.01 m, 중간 접촉)이 잡힌다.
    - 1 mm 평행 직선은 `clearance_unproven` 이 아니라 0.5 mm 재분할로 통과한다.
    - 접근 끝을 팔레트 안으로 1 cm 옮긴 변형이 잡힌다.
  - 단계표의 포함·제외, `prior_failed`.
- live 연결(v1):
  - 준비 장면 소품·시작 판정·작업자 입력이 지금과 같다.
  - 도킹 dry-run 목록이 생성·제거 장애물을 포함해 지금과 같다.
- 리플레이 게이트 29 건.

## 3. 위험·범위 밖

- 관측 후보가 기울기만큼 돈다. CPU 계획으로 확인하고, Isaac 은 PR ④ 다.
- 하역 truth 는 칸 자세다. 실제 하역 오차(Isaac 0.02–0.10 m)는 PR ④ 의 Isaac 에서 이웃 최소 거리로 기록한다.
- D1 0.25 m 는 경로 존재 상한이다. 대조는 이상적 추정의 계획 포락선이고, SLAM 오차(0.14–0.16 m)·실제 접촉은 대조에 없다.
- v2 의 배치 실행과 경계 기록은 거부한다.

## 4. PR ③ 인계(완료 조건)

- 이웃 강체·미션별 손잡이(상위 계획 PR ③).
- 러너의 등록·준비 입력을 세션 어댑터(`register_session`, 세션 팔레트의 `registered`·후보·`zone_rect`)로 만든다. v1 은 이 PR 의 동등성 시험이 지킨다.
- 센서 자기 필터 경로(`/World/Pallet` 고정, 5717행)를 실제로 싣고 있는 팔레트 id 로 한다.
- 근접 재관측의 기본 직선 채택에 이웃 소품 sweep 검사를 둔다(Codex 1차 반례: 2.1 m 직선, 시작 오프셋 0.049 m·0.049 rad 에서 공칭 + 0.25 침범).
- 도킹 보정 dry-run 이 v2 세션 소품을 보는지 시험한다(이 PR 에서 목록 출처를 바꾼다).
- v2 팔레트 초기 배치는 `pallets[].truth` 에서 만들고, v1 정적 이웃은 호환 예외로 둔다.

## 5. 자체 검토 — 직접 재계산

- 장면 도구의 자리·기울기·순서·오차(위 1절)를 `tools/riibotics_isaac_scenes.py` 에서 다시 읽었다. 랜덤 index 2·3·4 = (343, 213, −8°)·(185, 356, 10°)·(495, 312, −6°).
- 여유 상수 `RECOGNISED_MARGIN_M` 0.21, `PRIOR_POSE_MARGIN_M` 0.182, `ENVELOPE_M` 0.01(`pallet_permit.py`), `ZONE_GROW_M` 0.25, `ZONE_YAW_RAD` 12°.
- 반대각선 hypot(0.6, 0.8)/2 = 0.5 m → 0.5 + 2·0.5 = 1.5 m 사각형.
- 10° 축 방향: 0.25·cos 10° + 0.25·sin 10° = 0.2462 + 0.0434 = 0.2896 m(Codex 독립 검토 값과 같다).

## 6. 검토 기록

### 3판 → 4판(구현 검토)

- **Codex 구현 검토(1차): P1 없음, P2 3·P3 2 — 받아들였다.**
  - 연속 하한이 현을 썼다 → 원호 표본(정·역방향 시험).
  - 해제 미리보기 회귀 시험의 하네스가 옛 `live_session_` 구조였다 → 새 구조·`session_scenario_` 추출.
  - 어댑터 미연결 → 범위를 고쳐 PR ③ 으로 넘기고, v1 여섯 장면의 동등성 시험을 더했다.
  - P3: `centre_zone` 이 0 이 아닌 yaw 중심을 거부한다.
- **Codex 구현 검토(2차): P1 없음, P2 1·P3 1 — 받아들였다.** 원호의 도는 쪽을 주행 방향으로 정한다(끝 yaw −π 반례). v1 동등성 시험을 열 장면 모두로 넓혔다(등록·준비 입력). 단계표 시험(실은 외곽, withdraw 이웃, 복귀의 자기 하역 팔레트, 접촉 뒤 `prior_failed`)을 더했다.

### 3판(수렴)

- **③ Codex 재검토(3차): 새 P1/P2 없음.** P3(재분할 뒤 하한의 감산량 h/2)를 반영했다. 도킹 합집합 hook 의 원래 목록 유지·추가·제거 반영과 접근 단계 1 cm 침범 검출을 Codex 가 따로 실행해 확인했다.

### 2판 → 3판

- **③ Codex 재검토(2차, 독립 스레드에서 이어감): P1 없음, P2 2·P3 2 — 받아들였다.**
  - 접근 단계에서 미션 팔레트를 포함한다.
  - 도킹 dry-run 은 생성 장애물 목록 + 세션 이웃으로 한다(교체하지 않는다).
  - P3: `contact_observed`·`clearance_unproven` 구분, |κ|·호 길이 명시, 계획 설정 명시와 기록.
- 이전 지적(registered·구역 변환·등록·팔레트별 입력·`moved_ids`·`prior_failed`·v2 truth 정본)은 계획 수준에서 해소됐다고 Codex 가 확인했다. 근접 직선 sweep 의 PR ③ 이관도 그전까지 v2 실행을 거부한다는 범위에서 받아들였다.

### 1판 → 2판

- **② Codex 적대적 검토(1차): P1 없음, P2 4·P3 1 — 모두 받아들였다.**
  - v1 의 후보 기준·등록이 truth 다 → `registered` 분리.
  - 도킹 dry-run 이 공통 소품을 우회한다 → live 출처를 미션 소품으로, v2 시험은 PR ③.
  - 근접 기본 직선 sweep → PR ③ 인계.
  - 표본 사이 접촉·withdraw 누락 → 연속 구간 하한, 단계표.
  - P3: 팔레트별 준비 입력 → 2.4절(이 PR 에서 계약·연결).
- **③ 독립 검토(Codex 새 스레드, ② 를 보지 않음): P1 없음, P2 8·P3 2 — 모두 받아들였다.**
  - `pickup_nominal` 이 버려짐.
  - 구역의 세 표현과 yaw 기준 → 2.5절·`yaw_centre_rad`.
  - 등록 계약(치수·형상 키·id 유일성) → 2.3절.
  - 팔레트별 준비 입력 → 2.4절.
  - `moved` 의 id 집합 → `moved_ids`.
  - 단계표·연속 판정·자기 팔레트 규칙.
  - CPU 상태 전이(`prior_failed`)·결과 범위 이름.
  - v2 truth 정본 하나.
  - P3: 준비 소품 고정의 전제, 이름·D1 범위.
- agy 는 사용량 조회 실패(exit 2)로 부르지 않았다. 그래서 ③ 을 Codex 새 스레드로 채웠다.
