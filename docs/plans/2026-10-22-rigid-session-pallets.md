# 다중 강체 팔레트·원자적 미션 전환 — 연속 이송 CT-1 PR ③ (3판)

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

- 세션 모드는 `--live`·`--layout scene`·`--near-capture-redraw always` 에서만 받는다(2.6).
- 배치·`--record-boundary`·`--bottom-deck-check on`·`--near-tracking` 은 거부한다.
- v1 장면·bay·배치의 경로는 그대로다(리플레이 게이트 29 건, 같은 port 호출).

### 2.1 모든 팔레트를 강체로

- **이름**: 논리 id(`P-1`)와 prim 이름을 나눈다.
  - `pallet_prim(id)` = `/World/SessionPallets/` + id 의 `-` 를 `_` 로 바꾼 이름(`P_1`). USD 는 `-` 를 받지 않는다(Codex 1차 P1, `Sdf.Path` 재현).
  - 물리 뿌리·의미 분할 라벨·LiDAR 접두가 모두 이 함수 하나를 쓴다. 로더는 id 가 `[A-Za-z][A-Za-z0-9_-]*` 이고 바꾼 이름끼리 겹치지 않는지 검사한다.
- usd 는 한 번만 가져온다(`create_pallet` 를 import 와 배치로 나눈다). 각 팔레트 자리는 `pallets[].truth` 이고, 장면 객체 이름은 `pallet_{prim 이름}` 이다.
- 세션 모드에서는 `/World/Pallet` 을 만들지 않는다.
- 정적 복사본(`add_scene_pallets`)은 v1 에만 남긴다.
- `configure_drives` 의 뿌리: 세션 모드는 모든 세션 팔레트 뿌리에 재질·오프셋·상자 수 단언을 한다.
- 정착: 모든 팔레트를 5 mm·z 5 mm 로 검사한다(지금 규칙, 팔레트마다).
- 물리 설정은 `/World/Pallet` 과 같다(잠 문턱 등은 바꾸지 않는다). 다른 팔레트의 표류는 2.4 감시가 기록한다.

### 2.2 손잡이 대리 객체 `PalletSwitch`(순수, `sim/isaac/session_pallets.py`)

- `{id: handle}` 과 현재 id 를 가진다. `get_world_pose`·`set_world_pose`·`get_mass`·`get_linear_velocity`·`get_angular_velocity` 를 지금 손잡이에 넘긴다.
- `handle(id)` 는 특정 팔레트를 직접 읽게 한다.
- 세션 모드에서 러너의 `pallet` 은 이 객체다. v1 은 손잡이 그대로다.
- 전환 `switch(id)` 는 바꾼 id 와 틱을 기록한다.

### 2.3 전환은 채택 다음 틱의 머리에서 (분열 없음)

- 채택 hook(`live_mission_start`)이 하는 일:
  - 새 팔레트 id 를 `pending_switch` 로 둔다.
  - 미션 값을 정한다. 이때 새 팔레트는 `pallet.handle(id)` 로 직접 읽는다.
    - `initial_pallet`·`pickup_obstacle`
    - `truth_destination` = 세션 목적지 칸 자세
    - 하역 고리를 미션 목적지로 옮긴다.
- 그 틱의 남은 일은 끝까지 옛 팔레트를 본다. 채택 틱은 `job_hold` 가 `hold=True` 라 주행하지 않는다(core 5948).
- 전환을 기다리는 동안(`pallet.pending` 이 있음)은 원거리 인식의 단계 기록·주행 프레임 제출을 하지 않는다. `far_observing` 이 거짓을 낸다(Codex 2차 P2: 채택 틱의 물리 스텝이 옛 팔레트를 새 미션의 `leg_steps` 에 넣었다).
- **다음 틱 머리**(truth 표본 `ppos, pq` 를 읽기 직전, RT 7153)에서 `pending_switch` 가 있으면 한꺼번에 바꾼다.
  1. `pallet.switch(id)`
  2. 나가는 팔레트의 지금 자세를 그 팔레트의 쉬는 자세로 확정한다(2.4).
  3. 원거리 인식 truth 인스턴스 = 새 팔레트의 `inst_k`
  4. truth 장애물 목록의 다른 팔레트 외곽을 갱신한다(2.4).
  5. `TraceEvidence`: 다른 id 로 바뀔 때만 새로 만든다. 새 팔레트는 바닥에 있으므로 바닥 기준 = 그 자세다. 같은 id(실은 채 재제출)면 지금 증거를 그대로 둔다(Codex 1차 P2, agy). 단 세션 장면에서 실은 채 재제출 자체는 이번 PR 범위 밖이다 — `live_loaded_pallet` 이 `grid_planning` 을 요구하는데 `--layout scene` 에는 그것이 없다(구현 리뷰 1차 P2, 2.9). 이 규칙은 같은 id 의 빈 포크 재제출에 적용된다.
  - 그 뒤 그 틱의 모든 소비자(검사·hook·`cargo_rel`·0.1 s 표본·영상 기록)가 새 팔레트를 본다.
- 운반 중에는 다른 팔레트의 제출이 거절된다(`cargo_held`, `commands.py` 79). 그래서 전환 때 나가는 팔레트는 늘 바닥에 있다. 그래도 바닥이 아니면(cargo 가 그 id 를 싣고 있음) 전환을 세션 오류로 거부한다(계약 위반).
- 준비 실패·취소·거절에는 채택이 없으므로 아무것도 바꾸지 않는다.
- 미션 끝 뒤 cargo·복구가 남아도 다음 채택 전까지 같은 팔레트를 가리킨다.

### 2.4 지금 팔레트가 아닌 팔레트: 쉬는 자세·밀림 감시·truth 장애물

- **쉬는 자세**: 세션 시작 정착 때 모든 팔레트에, 그리고 전환 때 나가는 팔레트에 확정한다. 하역 판정·수동 접지·취소 여부와 무관하게 전환 순간의 자세다(Codex 1차 P2, agy P1).
- **감시**: 매 틱 지금 팔레트가 아니고 cargo 가 싣지 않은 팔레트를 읽는다(세션 팔레트 3–5 개).
  - 3D 위치 1 cm 또는 전체 회전각(기울기 포함) 0.01 rad 를 넘으면 `scene_require` 의 장면 손상 `Session pallet displaced: {id}` → 그 미션 종료, 이후 제출 거부(지금의 장면 손상 규칙).
  - 이 접두를 `failures.py`(SCENE)·`results.js` 에 등록하고, 세션이 살아 다음 상태(제출 거부)로 가는지 시험한다(Codex 1차 P2).
- **표류 기록**: 감시가 읽은 최대 변위를 팔레트마다 기록한다. Isaac 에서 물리 표류만으로 문턱을 넘는지 본다(agy P2).
- **truth 장애물 목록**(`obstacles`): static 소품 + 생성 장애물(러너 소유 그대로) + 다른 팔레트의 **지금 자세** 외곽이다.
  - 다른 팔레트는 감시와 같은 틱 읽기로 갱신한다. 쉬는 자세는 변위 판정에만 쓴다(Codex 2차 P2: 1 cm 허용 안에서 다가온 이웃을 고정 외곽은 놓친다).
  - idle 의 지금 팔레트는 2D 외곽 검사에 넣지 않는다. 포크가 포켓에 남은 정상 상태를 손상으로 판정하기 때문이다(Codex 2차 P2 재현). 지금의 3D 포크·차체 접촉 검사(RT 7400)가 맡는다. agy 의 idle 지적은 이것으로 답한다.

### 2.5 세션 어댑터 연결(PR ② 인계)

- 등록: `register_session(core.targets, session, kind, depth, width, live=True)` — 모든 팔레트·D-1..D-m.
- 준비 입력 `live_session_["pallets"][id]`:
  - `template` = 후보, `reference` = `registered`, `zone` = `zone_rect(zone)`, `bounds` = `None`(`--layout scene` 은 지금도 없다, RT 2579).
  - `far_zone` = 그 팔레트 구역 중심 + 중심 범위의 반대각선(지금 setup 구역 규칙, RT 3165–3171). 처음 자리에서 쓴다. 인식·하역 뒤는 추정 기준(PR ① 규칙)이다. P-2 가 P-1 구역을 쓰지 않는다(Codex 1차 P2).
- `nominal` = 세션 공칭.
- `live_session_["truth"]` = 세션 목적지(D-k) 칸 자세.
- teleop 허가·수동 정리: 세션 모드는 `zones_from(targets, {id: zone_rect})` 로 모든 사전 팔레트의 자기 구역을 넘긴다. 실은 채 재제출(구역 없음)에서도 그 지도는 그대로다(Codex 1차 P2).
- `session_extra_` 는 팔레트가 둘 이상이면 켜진다(PR ② 코드 그대로).
- v1 은 지금 코드 그대로다(PR ② 동등성 시험이 지킨다).

### 2.6 근접 기본 직선의 이웃 sweep(PR ② 인계)

- 세션 모드에서 검사한다. 기본 직선의 dry run 궤적(`RolloutResult.trajectory`)을 빈 외곽으로 쓸고 간 범위가 세션 이웃 소품(`session_extra_` 의 사각형)과 겹치면 `near_capture_misaligned:neighbour_sweep` 로 거절한다.
- 세션 모드는 `--near-capture-redraw always` 를 요구한다(parser). 그래서 거절은 재작도로 간다. 재작도는 자기 궤적·소품 검사를 한다(Codex 1차 P2).
- 옵션 `near_capture_neighbour_sweep` 은 세션 모드만 켠다(v1 리플레이 불변).

### 2.7 센서 자기 필터(agy P1)

- 실린 판단 = 지금 규칙(단계 lift·extract·transport·lower) 또는 cargo 가 싣고 있음(teleop·idle 운반 포함).
- 뺄 팔레트 = cargo 가 싣고 있으면 그 id, 아니면 실린 단계의 지금 팔레트다.
- 접두는 `pallet_prim(id) + "/"` 다(다른 id 와 섞이지 않음).
- v1 은 지금 문자열·규칙 그대로다.

### 2.8 의미 분할 라벨·기록

- 세션 모드 라벨: 팔레트 `pallet_prim(P-k)` → `inst_{k-1}`, `/World/Scene` → floor. `/World/Pallet` 은 요구하지 않는다(agy P2: 지금은 없으면 시작 단언이 실패한다).
- 원거리 런타임의 truth 인스턴스를 `FarRuntime.truth_instance` 로 둔다(기본 0). 전환 때 바꾼다.
- 영상 프레임 기록·0.1 s 표본에 세션 모드는 `pallet_id` 를 함께 남긴다(agy P3). D8b truth 기록(`--record-pocket-frames`)은 팔레트 id 를 담지 않으므로 세션 장면에서는 그 인자를 거부한다(구현 리뷰 1차 P3).
- 이웃 팔레트와의 실제 최소 간격: 틱마다 계획 외곽(실음/빈 포크)을 truth 후축 자세에 두고, 다른 팔레트의 지금 외곽과의 간격을 재 이웃별 최솟값을 `state["session_neighbour_gap"]` 에 남긴다(구현 리뷰 1차 P2, CT-1 D1).

### 2.9 남는 것

- 운영자의 미션 순서가 다른 팔레트에 막히면 준비가 실패한다(`prepare:all_candidates_failed`). 순서 규칙은 CT-2 다(agy P2).
- 경계 기록·리플레이는 세션 모드를 지원하지 않는다(거부). 회귀는 CPU 시험과 Isaac 기록으로 본다(agy P3).
- 세션 장면의 실은 채 재제출은 지원하지 않는다(`live_loaded_pallet` 이 `grid_planning` 필요). 이번 수용 범위(각 미션을 빈 포크로 시작하는 line·random 연속 세션)는 막지 않는다(구현 리뷰 2차 확인).

## 3. 시험 (먼저)

- `pallet_prim`: `-` 치환, 겹침 거부, USD 경로 유효성(로컬 pxr 가 있으면).
- `PalletSwitch`: 위임, `handle(id)`, 전환 기록, 모르는 id 거부.
- 전환(러너 함수를 AST 로 뽑아 실행 — 기존 시작 판정 시험과 같은 방식. 가짜 손잡이 둘, 서로 다른 자세):
  - 채택 틱의 남은 일은 옛 팔레트를 보고, 원거리 단계 기록·프레임 제출이 없다.
  - 다음 틱은 모든 소비자가 새 id 를 본다. 대상은 `ppos`·hook 셋·`cargo_rel`·원거리 truth 인스턴스·`obstacles` 의 다른 팔레트·흔적 증거다.
  - 채택 때 정한 `initial_pallet`·`pickup_obstacle`·`truth_destination` 은 새 팔레트의 값이다.
  - 같은 id 재제출이면 흔적 증거를 보존한다(운반 중 15 mm 미끄러짐이 계속 잡힘). 세션 장면에서 실은 채 재제출은 범위 밖(2.9).
  - 준비 실패·취소는 이전 id 를 유지한다.
  - 끝난 뒤 cargo·복구가 남으면 다음 채택 전까지 유지된다.
  - 나가는 팔레트를 cargo 가 싣고 있으면 거부한다.
- truth 장애물: 다른 팔레트가 쉬는 자세에서 9 mm 다가와 지금 외곽이 4 mm 겹치면 겹침 검사가 잡는다(Codex 2차 반례).
- idle 의 포크가 포켓에 남은 지금 팔레트는 2D 외곽 검사로 손상이 되지 않는다.
- 쉬는 자세·감시:
  - 하역 뒤 복귀 중 취소 → 다음 채택: 감시가 정상 팔레트를 밀림으로 잡지 않는다.
  - 1 cm·0.01 rad 경계, 지금 팔레트·실린 팔레트 제외.
  - `Session pallet displaced` 는 SCENE 분류이고 세션이 산다.
- 근접 기본 직선 이웃 sweep: Codex 반례(2.1 m 직선, 시작 0.049 m·0.049 rad, 이웃 공칭 (1.2, 1.09, 0) + 0.25)가 거절되고 재작도로 들어간다. 옵션을 끄면 지금과 같다.
- 자기 필터: 단계 밖 cargo 운반에서도 실린 id 만 빠지고, 다른 세션 팔레트는 남는다.
- teleop 구역: 팔레트마다 자기 구역, 실은 채 재제출에서도 유지.
- 원거리 구역: P-2 의 처음 자리 구역이 P-2 중심이다.
- 라벨: 세션 모드에 `/World/Pallet` 없이 시작한다.
- parser: 세션 모드 조합 거부·허용(`--near-capture-redraw always` 필수).
- 리플레이 게이트 29 건, ws1 전체.

## 4. 검증 (Isaac, ws1)

- line 세션 live(학습 원거리 인식·SLAM, `shard:2`): 탐침 cycle P-1:D-1 → P-2:D-2 → P-3:D-3.
  - 미션별 결과와 하역 오차를 기록한다.
  - 이웃 팔레트와의 실제 최소 거리를 기록한다(CT-1 D1 조건 — truth 외곽 대 차체 외곽, 틱마다 최소).
  - 밀림 0, 금지 접촉 0.
  - 전환 기록에 요청 틱(채택)과 실제 전환 틱(다음 틱 머리)을 따로 남긴다.
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

## 7. 검토 기록

### 구현 리뷰(Codex, 87a4c6e → 7f644f2)

- 1차: P2 4·P3 1.
  - 도킹 dry run 이 world 궤적을 계획 좌표 이웃 외곽과 비교했다 → truth 외곽으로 바꿨다.
  - 실은 채 재제출은 세션에서 불가 → 범위 제외로 적었다(2.3·2.9).
  - 첫 미션의 원거리 truth 인스턴스가 0 에 남았다 → 첫 미션 팔레트의 인스턴스로 바꿨다.
  - 이웃 실제 최소 간격 기록이 없었다 → 기록을 추가했다(2.8).
  - P3: 영상 기록 `pallet_id` 를 추가하고, `--record-pocket-frames` 를 거부한다.
- 2차: 새 P1/P2 없음(수렴). P3: 계획에 남은 실은 채 재제출·D8b 약속을 고쳤다(이 판).

### 3판 수정(Codex 3차)

- P2: 감시는 x·y·yaw 만 봐서 0.2 rad 기울어진 이웃(평면 8 mm·높이 79 mm)을 놓쳤다 → 3D 위치·전체 회전각.
- P3: id 끝 개행이 정규식을 지났다 → 전체 일치.
- P3: 상위 계획 위험 절의 옛 쉬는 자세 문구를 고쳤다.

### 2판 → 3판

- **② Codex 재검토(2차): 새 P1 없음, P2 3·P3 1 — 받아들였다.**
  - 전환 대기 중 원거리 기록·프레임 금지.
  - idle 2D 외곽 검사 철회(3D 접촉 검사 유지).
  - truth 장애물은 다른 팔레트의 지금 자세.
  - P3: 상위 계획 PR ③ 문구를 다음 틱 전환·전환 순간 쉬는 자세로 맞춤, 요청·전환 틱 구분 기록.
- 1판의 P1·P2 6 은 계획 수준에서 해소됐다고 Codex 가 확인했다.

### 1판 → 2판

- **② Codex 적대적 검토(1차, 새 스레드): P1 1·P2 6 — 모두 받아들였다.**
  - P1: USD 가 `P-1` 의 `-` 를 받지 않는다 → `pallet_prim`.
  - P2:
    - teleop 구역이 하나 → 팔레트별 지도.
    - 원거리 탐색 구역이 첫 팔레트에 남는다 → 팔레트별 `far_zone`.
    - 흔적 증거 초기화가 실은 채 재제출을 깬다 → 다른 id 일 때만.
    - 하역 뒤 취소면 쉬는 자세가 낡는다 → 전환 순간 확정.
    - 새 오류의 분류 → SCENE 등록.
    - 기본 설정에서 재작도로 안 간다 → `always` 필수.
- **③ 독립 검토(agy, Gemini 3.8 Flash Medium; ② 를 보지 않음): P1 3·P2 4·P3 2.**
  - 받아들임:
    - 실린 채 끝난 팔레트의 쉬는 자세.
    - 틱 분열 → 다음 틱 머리 전환.
    - 자기 필터가 단계만 봄 → cargo 포함.
    - idle 의 지금 팔레트 겹침 검사.
    - `/World/Pallet` 라벨 단언.
    - 순서는 CT-2.
    - 기록에 `pallet_id`.
    - 리플레이 제외 명시.
  - 받아들이지 않음: 물리 잠 문턱 변경. `/World/Pallet` 과 같은 물리를 두고 표류를 기록해 판단한다.

