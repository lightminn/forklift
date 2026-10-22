# 다중 강체 팔레트·원자적 미션 전환 — 연속 이송 CT-1 PR ③ (1판)

브랜치 `feat/ct1-rigid-pallets`(origin/main `11d7015` = PR #102 뒤). 상위 계획 `docs/plans/2026-10-22-continuous-transport-ct1.md` 3절 PR ③. PR ② 인계: `docs/plans/2026-10-22-session-scene-pallet-props.md` 4절.

## 1. 사실 (소스 조사 — 러너의 단일 팔레트 가정 전수)

- **생성.**
  - `scene.create_pallet`(scene.py 354–445)은 `output/pallet.usd` 를 만들어 고정 경로 `/World/Pallet` 에 참조한다. 강체가 1 개라고 단언하고, 장면 객체 이름 "pallet" 으로 `SingleRigidPrim` 을 등록한다.
  - `add_scene_pallets`(257–281)는 같은 usd 를 `/World/ScenePallets/Pallet_i` 에 정적 복사본으로 둔다(강체·질량 API 제거).
  - `configure_drives` 는 `/World/Forklift`·`/World/Pallet` 두 뿌리에만 다음을 한다(546–637).
    - 충돌 인스턴스 펼침
    - 접촉 오프셋·재질
    - 상자 수 단언
- **손잡이 하나(`pallet`).** 읽기·쓰기는 다음 자리들이다. RT = `run_transport.py`.
  - 정착 검사 RT 3207
  - 인식 평가 5106
  - 원거리 정지 truth: 배치 5182, live 5196
  - 영상 기록 5690
  - D8b truth 5788
  - 원거리 단계 기록 5968
  - live 실은 질량 6251
  - live 하역 판정 6325
  - `live_mission_start` 6004
  - **틱 앞 truth 표본 `ppos, pq` 7153**
  - 밀기 시험 7163
  - 배치 하역 판정 8193
- **`ppos`/`pq` 소비자.**
  - 들기 기울기, 실제 외곽 겹침, 측정 팔레트 겹침, 떨어짐·미끄러짐·밀림, 금지 접촉, 측면 여유, 새 장애물의 팔레트 면, 흔적 증거, `cargo_rel`, 수동 복구 접지, 0.1 s 표본.
  - 코어 hook `insertion_measured`·`truth_relative_yaw`·`pallet_z`(RT 5265–5268)도 이 틱 표본을 쓴다.
- **미션 채택은 틱 중간이다.** `job_hold`(RT 7469) → `_prepare_answer` → `start_mission` → `live_mission_start`(RT 6993–7031) 순서로 진행된다.
  - truth 검사(RT 7240–7457)가 이미 그 틱의 `ppos` 로 끝난 뒤다. 그 뒤의 hook·`cargo_rel` 은 틱 앞 표본을 쓴다.
- **경로 문자열.**
  - LiDAR 자기 필터는 부분 문자열 비교다(`planar_lidar.py` 165). 실렸을 때 `"/World/Pallet"` 를 빼므로, `/World/Pallet_1`·`/World/Pallets/...` 도 같이 빠진다.
  - 의미 분할 라벨은 `/World/Pallet` → `inst_0`, 장면 팔레트 → `inst_1..` 이다(RT 2972–2986). 원거리 인식의 truth 인스턴스는 0 에 고정돼 있다(`far_recognition_runtime.py` 312–318).
- **한 번 계산해 다시 쓰는 값.**
  - `truth_destination`(RT 2704, 하나), 하역 고리(D-1 만, 2697).
  - `initial_pallet`·`pickup_obstacle`: live 는 미션마다 다시 묶는다(7004–7005).
  - truth 장애물 목록 `obstacles = scenario.props 의 사각형`(RT 4654): 미션마다 다시 묶지 않는다.
  - `live_session_["truth"]` = {D-1, D-0}.
  - 원거리 setup 구역.
  - `PALLET_BOXES` 와 `insertion_geometry` 는 형상 하나다.
- **기록·재생.**
  - `replay_setup.SERVICES = {"pallet": "SimTruth"}` — 로컬 이름 `pallet` 이 SimTruth 포트다.
  - `drive()` 머리말의 `if "x" in _bound` 줄이 재생 입력 목록이다.
  - live 는 경계 기록을 거부한다(RT 1127).
- **근접 기본 직선**(core 3495–3540): 위치·각도 상자와 dry run 의 도착만 검사한다. 쓸고 간 외곽은 검사하지 않는다. `RolloutResult.trajectory` 에 매 스텝 자세가 있다.
- **PR ② 의 것.**
  - `scene_file.load_session`·`register_session`·`session_from_scene`·`zone_rect`
  - `pallet_permit.session_pallet_props`
  - 러너 `live_session_["pallets"]`·`session_scenario_`·`session_extra_`
  - 계획 포락선 대조 도구

## 2. 범위

**세션 모드**: `--scene-file` 이 `riibotics_isaac_session_v1` 이면 켜진다.

- 세션 모드는 `--live`·`--layout scene` 에서만 받는다.
- 배치·경계 기록·`--recognition far` 없는 조합의 허용 여부는 2.6.
- v1 장면·bay·배치의 경로는 그대로다(리플레이 게이트 29 건, 같은 port 호출).

### 2.1 모든 팔레트를 강체로

- usd 는 한 번만 가져온다(`create_pallet` 를 import 와 배치로 나눈다).
- 세션 팔레트 k 는 `/World/SessionPallets/{id}`(`P-1` 포함 — 세션 모드에서는 `/World/Pallet` 을 만들지 않는다)에 둔다. 장면 객체 이름은 `pallet_{id}` 이고, 자리는 `pallets[].truth` 다.
- 정적 복사본(`add_scene_pallets`)은 v1 에만 남긴다.
- `configure_drives` 의 뿌리: 세션 모드는 모든 세션 팔레트 뿌리에 재질·오프셋·상자 수 단언을 한다.
- 정착: 모든 팔레트를 5 mm·z 5 mm 로 검사한다(지금 규칙을 팔레트마다).

### 2.2 손잡이 대리 객체 `PalletSwitch`(순수, `sim/isaac/session_pallets.py`)

- `{id: handle}` 과 현재 id 를 가진다. `get_world_pose`·`set_world_pose`·`get_mass`·`get_linear_velocity`·`get_angular_velocity` 를 지금 손잡이에 넘긴다.
- 세션 모드에서 러너의 `pallet` 은 이 객체다. v1 은 손잡이 그대로다.
- 전환은 `switch(id)` 하나다. 바꾼 id 와 시각을 기록한다.

### 2.3 채택 틱의 원자적 전환

`live_mission_start`(채택 hook) 안, 다른 일보다 먼저 한 곳에서 한꺼번에 바꾼다.

1. `pallet.switch(mission.pallet_id)`
2. 그 틱의 truth 표본 `ppos`·`pq`·`pallet_yaw`·`pallet_tilt` 를 새 손잡이로 다시 읽는다(`drive` 의 nonlocal). 그 뒤의 hook(`pallet_z`·`insertion_measured`·`truth_relative_yaw`)·`cargo_rel`·0.1 s 표본이 새 팔레트를 본다.
3. `initial_pallet`·`pickup_obstacle`(지금처럼 다시 읽음)
4. `truth_destination` = 세션 목적지 truth(= 칸 자세 — 세션 목적지는 칸 그 자체)
5. 하역 고리: 미션 목적지로 옮긴다.
6. 원거리 인식 truth 인스턴스 = 그 팔레트의 `inst_k`
7. truth 장애물 목록 `obstacles`:
   - static 소품, 실행 중 생성된 장애물, 다른 세션 팔레트의 지금 truth 외곽(채택 순간 읽음)을 합친다.
   - 생성 장애물의 소유는 그대로 러너다(합치기만 한다).
8. `TraceEvidence` 의 기준 초기화 — 새 팔레트의 바닥 자세를 기준으로 한다.

- 준비 실패·취소·거절에는 채택이 없으므로 아무것도 바꾸지 않는다.
- 미션 끝 뒤 cargo·복구가 남아도 다음 채택 전까지 같은 팔레트를 가리킨다.

### 2.4 지금 팔레트가 아닌 팔레트의 밀림 감시

- 쉬는 자세: 세션 시작 정착 뒤, 그리고 그 팔레트의 미션이 하역 판정을 받은 뒤(그 미션의 끝)에 확정한다.
- 12 틱마다 읽는다. 1 cm 또는 0.01 rad 를 넘으면 `scene_require` 의 장면 손상(`Session pallet displaced: {id}`) → 그 미션 종료, 이후 제출 거부(지금의 장면 손상 규칙).
- 지금 팔레트는 감시하지 않는다(지금의 떨어짐·미끄러짐·밀림 검사가 맡는다).

### 2.5 세션 어댑터 연결(PR ② 인계)

- 등록: `register_session(core.targets, session, kind, depth, width, live=True)` — 모든 팔레트·D-1..D-m.
- 준비 입력: `live_session_["pallets"]` 를 세션 팔레트마다 {`template` = 후보, `reference` = `registered`, `zone` = `zone_rect(zone)`, `bounds`} 로 만든다. `nominal` 은 세션 공칭이다.
  - `bounds`: `None` — `--layout scene` 은 지금도 `pickup_bounds` 가 없다(RT 2579, 공장 배치만 2607).
- `live_session_["truth"]` = 세션 목적지(D-k) 칸 자세(이상적 칸 — 하역 판정의 기준).
- `session_extra_` 는 팔레트가 둘 이상이면 켜진다(PR ② 코드 그대로).
- v1 은 지금 코드 그대로다(PR ② 동등성 시험이 지킨다).

### 2.6 근접 기본 직선의 이웃 sweep(PR ② 인계, Codex 반례)

- 세션 모드에서 기본 직선의 dry run 궤적(`trajectory`)의 빈 외곽을 쓸고 간 범위가 세션 이웃 소품(`session_extra_` 의 사각형)과 겹치면 `near_capture_misaligned:neighbour_sweep` 로 거절한다 → 재작도 경로로 간다(재작도는 자기 소품 검사를 한다).
- v1 은 끈다(옵션 플래그 `near_capture_neighbour_sweep`, 세션 모드만 켬 — 리플레이 불변).

### 2.7 센서 자기 필터

- 실린 팔레트만 자기로 뺀다. 세션 모드의 접두는 `"/World/SessionPallets/{carried_id}/"`(뒤 슬래시 포함 — 다른 id 와 섞이지 않음)다.
- 어느 팔레트가 실렸는지는 `PalletSwitch` 의 지금 id 와 단계(지금 규칙)로 정한다.
- v1 은 지금 문자열 그대로다.

### 2.8 의미 분할 라벨

- 세션 팔레트 `P-k` → `inst_{k-1}`(P-1 = inst_0).
- 원거리 런타임의 truth 인스턴스를 `FarRuntime.truth_instance` 로 둔다(기본 0). 채택 때 바꾼다.

### 2.9 조합

- 세션 모드는 `--live` 필수다. 배치·`--record-boundary`·`--bottom-deck-check on`·`--near-tracking` 은 거부한다.
- `--recognition far` 는 권장이고 필수는 아니다(검출기만으로도 돈다).
- 새 인자는 없다(장면 파일 형식으로 판단한다). 근접 계약(`POST_MEASUREMENT_ARGUMENTS`)에 바뀌는 것이 없음을 시험한다.

## 3. 시험 (먼저)

- `PalletSwitch`: 위임, 전환 기록, 모르는 id 거부.
- 원자 전환(CPU 가짜 손잡이 둘, 서로 다른 자세):
  - 채택 틱에 모든 truth 소비자가 같은 id 를 본다. 대상은 `ppos`·hook 셋·`initial_pallet`·`pickup_obstacle`·`truth_destination`·원거리 truth 인스턴스·`obstacles` 의 이웃·흔적 증거 기준이다.
  - 준비 실패·취소에서는 이전 id 가 유지된다.
  - 끝난 뒤 cargo·복구가 남으면 다음 채택 전까지 유지된다.
  - 러너 함수는 AST 로 뽑아 실행한다 — 기존 시작 판정 시험과 같은 방식.
- 밀림 감시: 1 cm·0.01 rad 경계, 쉬는 자세 확정 시점(하역 판정 뒤), 지금 팔레트 제외.
- 근접 기본 직선 이웃 sweep: Codex 반례(2.1 m 직선, 시작 0.049 m·0.049 rad, 이웃 공칭 (1.2, 1.09, 0) + 0.25)가 거절되고, 옵션을 끄면 지금과 같다.
- 자기 필터 접두: 실린 id 만, 다른 세션 팔레트는 남는다.
- parser: 세션 모드 조합 거부·허용.
- 리플레이 게이트 29 건, ws1 전체.

## 4. 검증 (Isaac, ws1)

- line 세션 live(학습 원거리 인식·SLAM, `shard:2`): 탐침 cycle P-1:D-1 → P-2:D-2 → P-3:D-3.
  - 미션별 결과와 하역 오차를 기록한다.
  - 이웃 팔레트와의 실제 최소 거리를 기록한다(CT-1 D1 조건 — truth 외곽 대 차체 외곽, 틱마다 최소).
  - 밀림 0, 금지 접촉 0.
  - 전환 기록(채택 틱 id)도 남긴다.
- random 세션 live: P-4:D-1 → P-5:D-2 → P-3:D-3, 같은 기록.
- 결과는 기록만 하고, 실패는 원인별로 분류한다(다른 세션 소관은 넘긴다).
- 실시간 배율(강체 팔레트 3·5 개의 물리 비용)을 기록한다.

## 5. 위험·범위 밖

- 강체가 늘면 물리 비용이 는다. 배율만 기록한다.
- 근접 캡처의 예상 위치 선택은 원거리 선택의 연속성일 뿐 P-k 식별이 아니다. 이웃이 가까우면 다른 팔레트를 볼 수 있다. Isaac 에서 관찰한다.
- 하역된 팔레트를 다시 집는 미션(재이송)은 등록·후보가 지원하지만, Isaac 확인은 PR ④ 다.
- 하역 정밀도·다리 끝 크리프는 다른 세션 소관이다.

## 6. 자체 검토 — 직접 재계산

- 조사 결과의 행 번호는 Explore 조사 뒤 다음을 직접 다시 읽었다.
  - 정착 검사(RT 3207–3214: 평면 5 mm·z 5 mm)
  - `pickup_bounds`(RT 2579·2607: 장면 배치는 None)
  - 근접 기본 직선(core 3495–3540)
  - `RolloutResult.trajectory`(rollout.py 28–31)
  - LiveSession 구조(RT 6793–6810)
- 자기 필터 부분 문자열 문제: `"/World/Pallet" in "/World/Pallets/P-2/..."` 는 참이다. 그래서 세션 팔레트를 `/World/SessionPallets/` 아래에 두고 뒤 슬래시 접두를 쓴다. v1 의 `/World/Pallet` 은 `/World/SessionPallets/...` 와 겹치지 않는다(부분 문자열이 아니다).

