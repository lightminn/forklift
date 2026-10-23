# 장면 모드 재계획 장치와 다리 끝 cusp 정체 복구 — 계획 (2026-10-23, 6판)

사용자 결정(2026-10-23, 총괄 세션 경유): 다리 끝 정체 복구 A·B 를 진행한다.

- 정지 복구 설계 7판(`docs/plans/2026-10-04-lidar-obstacle-map.md` "cusp 위치 정체 복구")의 "범위 밖이면 종료"를 개정한다.
- 조건:
  1. 복구가 발동할 때마다 미션·다리·자세 이탈·직전 SLAM 보정 튐 크기를 기록하고, 실행별 집계를 결과 문서에 넣는다.
  2. 근본 원인 분리(보정 유무·조향/구동 응답 재생)는 이 PR 과 별개로 보고한다. 다리 끝 근처 보정 보류(C)가 필요해지면 그때 따로 올린다.
- 통합: "v-g2 조사 잡 재개" 세션의 P6(`docs/plans/2026-10-23-planner-permission-clearance.md`, 장면 모드 허가 정지 재계획·탈출)과 A 는 **하나의 장면 모드 재계획 장치**로 만든다.
  - 공통 장치는 이 세션이 만든다: 정지 자세에서 다리 재계획, 미션 유한 예산, 계획 충돌·출발 허가, 예산 소진 시 명시 종료.
  - 허가 정지 트리거는 그 세션이 얹는다.

브랜치 `feat/scene-replan`(origin/main `140a686`).

## 1. 사실

### 1.1 실패 네 건 (CT-1 PR ④-1 Isaac, `docs/validation/2026-10-23-slot-anchored-neighbours.md`)

원자료는 ws1 `/home/projects/forklift/test_runs/live/` 의 두 실행이다.

- `ct1_anc_74ed6a1_line_s1`(L): `run/missions/m3/records.json`, `run/spool/{slam.control,slam.records,state.samples}`.
- `ct1_anc_9ddc5d5_random_s1`(R): `run/missions/m2/records.json`, `run/spool/{slam.control,slam.records,state.samples}`.

| 사건 | 단계·다리 | 래치 시각 | 위치/방향 이탈 | `far` | 처리 |
|---|---|---:|---:|---|---|
| L 운반 | transport, tracker 6, leg 1 | 454.817 s | 74.6 mm / −52.4 mrad | 아님 | 51 회 판단 뒤 `ended` → `cusp_position_stall` |
| R 관측 1 | observe, tracker 1, leg 0 | 224.542 s | 49.4 mm / +62.5 mrad | 아님 | 5 s 뒤 `observe_rule` → 관측 재계획 1 |
| R 관측 2 | observe, tracker 2, leg 0 | 241.567 s | 94.9 mm / −27.5 mrad | 예 | 즉시 `observe_rule` → 관측 재계획 2(예산 끝) |
| R 관측 3 | observe, tracker 3, leg 0 | 251.183 s | 162.8 mm / −43.1 mrad | 예 | `observe_rule` → 재계획 막힘(core 2166) → 명령 0 → 272.6 s `Tracking timeout in observe` |

- 공통 조건: 다리 끝(`leg_remaining_m` 0)에서 정지가 확인됐다. 최대 곡률 호 끝이고(래치 기록 κ 0.345–0.373, 조향 포화), 자세 조건(`pose_bounds`: 위치 0.06 m·방향 0.05 rad, `far` 는 그보다 0.03 더)으로 거부됐다.
- 래치는 판단까지 갔다. 문제는 그 다음 처리다.
  - 운반·복귀는 `ended` 다(설계 7판).
  - 관측은 `observe_rule` 로 같은 관측점을 다시 계획한다. 예산(`OBSERVE_REPLANS_PER_LEG` = 2)이 끝나면 아무 분기도 없이 시간 초과까지 선다.
- 관측의 느슨한 도착(core 2318–2340, `OBSERVE_ARRIVAL_M`)은 마지막 끝(`remaining_to_goal` 0)에서만 적용된다. `position_error_m` 은 지금 cusp 까지의 오차라, R 관측 1 은 cusp 까지 49 mm 지만 최종 관측점까지는 1.832 m 였다(Codex).
- 장면 모드(`--layout scene`)에서는 래치의 장애물 인계(`force_replan`)도 격자 계획 조건이라 건너뛴다(`force_skipped_no_grid`, core 4786–4791).
- 정지·재계획·진척 없음 조기 실패도 모두 `grid_planning` 조건이다(core 1201–1203·1251–1264 — 허가 정지 사례와 같은 뿌리, P6).

### 1.2 원인 (분리 중 — 이 PR 과 별개 보고)

- L 운반의 경과:
  - 448.9 s 에 SLAM 보정이 한 tick 에 제어 자세를 옆으로 옮겼다(제어 횡오차 −16 → +19 mm, 제어−truth 차 변화 64 mm).
  - 그 뒤 cusp 까지 약 1.2 m·2.5 s 동안 횡오차가 줄지 않고 +19 → +74 mm 로 커졌다(truth +32 → +95 mm, 후진·적재, 경로 곡률 0.29).
  - 정체 다리 구간(14–208)으로 제한해 잰 값이다. 스크립트: ws1 `test_runs/live/ct1_lateral_trace.py`.
- R 관측:
  - 두 번째 재계획 직후 243.375 s 에 43.6 mm 보정 튐이 있다.
  - 세 정체는 모두 각 경로의 첫 다리 끝이고, 다리 길이는 1.121·3.021·1.657 m 다.
- 마지막 관측 경로를 이상적 자전거 모델로 재생하면 첫 cusp 오차가 2.5 mm 다(Codex). 기록된 보정을 넣어도 도착했다. 즉 경로 기하나 보정 하나만으로는 163 mm 가 재현되지 않는다. 조향 응답(변화율 1 rad/s, 큰 각도 평균 부족 0.016–0.018 rad)·구동 제한(조향 오차 구동 제한, core 4535)·보정의 상호작용을 더 분리해야 한다.
- 그래서 이 계획은 원인을 고치는 것이 아니라 **실패를 유한한 복구로 바꾸는 것**이다. 원인 분리는 2.6 에 따로 둔다.

### 1.3 쓸 수 있는 부품

- **live 다리 재계획**(격자 계획용):
  - `_leg_payload(kind, world, start, grid_kw, retry_on)` — 작업자 `leg` 작업, tight 재시도와 운반 출구 접두 포함(core 5751–5773).
  - `_request("live_leg", …)` — 한 번에 한 작업(5867–).
  - `_live_leg_finish` — 경로 설치·운반 도킹 정지점 재무장·새 다리 허가 갱신(5825–5846).
- **운반 cusp 재계획**: 끝점 방향 실패 때 동기 `plan_transport_leg`, `cusp_replans` 기록·예산 `max_cusp_replans`(core 1959–2030). D7 수용 뒤면 `transport_recovery_after_docking` 로 끝난다.
- **관측 재계획**: `plan_observation_leg`(같은 최종 관측점, core 2155–2215).
- **허가**: 새 다리 허가 갱신(`_refresh_for_new_leg`), 경로 검사 `check_path`·`check_candidate`(장애물 층).

## 2. 설계

### 2.1 공통 장치 — 장면 모드 재계획 (A·P6 공통, 이 세션)

**인터페이스**("v-g2" 세션과 맞춤, 2판에서 추가 장애물을 콜백으로 바꿈):

- `core.scene_replan_request(t, trigger, *, extra_obstacles_fn=None, clearance_m=None, detail=None)`.
  - `trigger`: `"cusp_pose"`(A) · `"cusp_handed"`(래치의 장애물 인계, 장면 모드) · `"permit_stop"`(P6, 그 세션).
  - `extra_obstacles_fn()`: **계획하는 그 순간**(해제·재투영 뒤)에 불러 계획 프레임 Rectangle 들을 받는다. P6 은 그때의 허가 스냅숏으로 지금 경로 앞을 다시 검사해, 막힌 칸(`path_blocked_cells`, 없으면 `blocked_cells`)을 이웃끼리 합친 사각형 목록으로 돌려준다(없으면 빈 목록, Codex 1차 P2-4: 해제 전 스냅숏에서 만든 사각형은 해제 뒤 프레임이 다르다).
    - 콜백이 예외를 던지면 빈 장애물로 계획하지 않는다. 그 발동을 `scene_replan_failed:<trigger>:obstacles_error` 로 끝낸다("v-g2" 세션 요청).
  - `clearance_m`: 이번 재계획의 계획 여유 덮어쓰기(None 이면 평소 값).
  - `detail`: 기록용.
- 요청은 **플래그만 세운다**(`self.scene_replan` 에 요청 한 건). 그 tick 은 0 명령·조향 유지로 끝난다.
  - 이미 요청이 있으면 새 요청은 무시하고 기록만 한다(한 번에 한 건).
  - 요청 자리(래치 끝, 래치 인계, P6 트리거)는 그 tick 의 나머지 추종·정체·도착 처리를 건너뛴다(Codex 1차 P2-1: `cusp_tick` 다음의 `follow()` 가 같은 tick 에 hold 를 다시 걸어 요청을 무효로 만든다).

**적용 조건**:

- 장면 모드(core 에 `scene` 여부를 명시적으로 넘긴다 — `grid_planning` 꺼짐과 같지 않다, Codex 1차 P2-8).
- live·배치 모두. 단계 `transport`·`return_home`.
- 제외(지금 규칙 유지): D7 도킹 수용 뒤·재접근 중(`transport_recovery_after_docking`·도킹 재시도), near·bottom-deck·held realign.

**tick 안 자리와 명령 소유**(Codex 2차 P2-1):

- 순서는 `release_gate` → `job_hold` → **장면 재계획 상태 기계** → (상태 기계가 그 tick 을 소유하지 않을 때만) 일반 `follow`.
- 하단의 허가·명령 조립·live 중재는 그대로 돈다.
- 요청이 있는 동안 기존 `live_leg` 요청은 끼어들지 않는다.
- 상태별 출력:

| 상태 | 바퀴 | 조향 | 추종 |
|---|---|---|---|
| 대기(`wait_stop`·`wait_jobs`·`plan`·`check`) | 제동 0 | 지금 값 유지(hold) | 생략 |
| `presteer` | 0 | 목표 곡률로 변경(hold 아님 — `plan_wait_`/`dwell_hold` 를 쓰지 않는다) | 생략 |
| `escape` | 탈출 접두 전용 추종기 | 추종기 명령 | 그 추종기만, 매 tick 허가 |

- 탈출 접두의 끝(도착·정지)은 장치가 소비한다. 일반 도착 처리로 넘기지 않는다.
- 탈출 중에는 기존 `backoff/escape` 소유 표시를 세워 매 tick 허가가 실제로 적용되게 한다(Codex 3차 P2-2).
  - `permission_acting()`(core 1043)은 운반 단계에서 남은 거리가 짧으면 도킹 직선으로 보고 허가를 끈다.
  - 0.3 m 탈출 추종기만 넣으면 `(acting=False, docking_straight=True)` 였다. 기존 escape 설치(core 1525)처럼 표시를 세우면 `(True, False)` 다.
  - 허가가 탈출 경로·방향을 읽어 최종 명령에 상한을 건다. 허용이 0 이 되면(센서 침묵·새 장애물) 그 자리에서 서고, 그 요청은 `scene_replan_failed:<trigger>:escape_stopped` 로 끝난다.

**진행**: 요청이 있는 동안 상태 기계 `scene_replan_step(t, rear)` 를 돈다.

1. `wait_stop`: 정지 확인(`stop_now`, 없으면 truth 속도)까지 기다린다.
2. `wait_jobs`: 다른 계획 작업·보류 중인 SLAM 해제가 있으면 끝날 때까지 기다린다(기존 `live_leg` 의 D8b 가드와 같은 조건, core 602–617).
3. `release`: hold 중이면 해제·재투영(`release_and_carry`)하고, 그 tick 의 제어 자세를 해제 뒤 자세로 갱신한다(`set_rear`·`slam["control"]`, Codex 1차 P2-1).
4. `plan`: 그 단계의 다리를 계획한다.
   - transport → `plan_transport_leg`(transport_scenario, 출구 접두 포함).
   - return_home → `plan_return_leg`(return_scenario).
   - 장애물 = 계획기가 장면에서 만드는 장애물 + `extra_obstacles_fn()`. 장치는 장애물 집합을 따로 만들지 않는다(P1c 자동 적용).
   - **tight 재시도는 하지 않는다**. 기존 `_leg_payload` 의 여유 0 재시도는 `invalid_start` 를 지우고 `clearance_m` 덮어쓰기를 취소한다(Codex 1차 P2-6). 장치 전용 payload 를 쓴다.
   - live 는 작업자에 보낸다(새 site `scene_replan`, 요청 id, 20 s 기한). 배치는 동기다.
   - 응답이 버려지면 같은 요청으로 돌아간다(예산을 다시 세지 않는다, Codex 1차 P2-2).
     - `moving` 이면 `wait_stop` 으로 돌아간다. 제출마다 정지·보류 해제·작업 점유를 다시 확인한다(D8b 가드, core 610, Codex 2차 P2-3).
     - `superseded`·`correction` 이면 `wait_jobs` 로 돌아간다.
     - 일시 정지·STOP 뒤 재개도 같은 요청을 이어 간다.
   - **유한 상한**(Codex 2차 P2-3):
     - 한 요청의 버림·재제출은 3 회까지다. 넘으면 `scene_replan_failed:<trigger>:dropped` 다.
     - 이 site 는 기존 `_drop` 의 site 누적 상한(3 회, core 6005)에서 뺀다. 대신 이 요청별 상한과 요청 수 예산(4)이 묶는다(최대 12 회).
     - 한 요청 전체(정지 대기부터 재개까지)는 **시뮬 경과 시간** 60 s 다. 계획 대기도 포함한다. `commands.work_elapsed()` 는 계획 대기를 빼므로 쓰지 않는다(Codex 3차 P3). 일시 정지 동안은 멈춘다(재개 때 그 길이만큼 미룬다 — 래치의 `shift_time` 과 같은 방식).
     - 넘으면 `scene_replan_failed:<trigger>:timeout` 다. 정지 대기에도 기한이 생긴다. 요청 중에는 일반 추종 시간 초과가 돌지 않기 때문이다.
5. `escape`: 결과가 `invalid_start` 면 탈출 접두를 만든다.
   - 기존 `escape.evaluate/straight/choose` 를 장면 장애물 + 추가 장애물 + 이번 여유로 부른다. 격자 래퍼 `find_escape` 는 장면 모드의 `grid_kwargs` 에 점유 격자가 없어 쓸 수 없다(Codex 1차 P2-5).
   - 접두는 6 의 허가 검사를 첫 틱부터 받는다.
   - 접두를 주행하고 정지하면 같은 요청의 `plan` 으로 돌아간다. 탈출은 요청당 1 회다.
   - 탈출 뒤 정상 경로를 설치할 때 탈출 소유 표시(`backoff/escape`)를 지우고 원래 tracker 설정(속도·허용치)을 복원한다(기존 장애물 재계획이 core 1629 에서 하는 것과 같게, Codex 4차 P3). 표시가 남으면 새 경로가 탈출 종료 처리(core 1680)를 받고 탈출 설정을 물려받는다.
   - 접두가 없거나 허가에 떨어지면 `scene_replan_failed:<trigger>:invalid_start` 로 끝낸다.
6. `check`: 설치 전 검사.
   - **먼저 후보 tracker 의 상태를 분기한다**(Codex 3차 P2-1: 실패한 tracker 의 명령도 곡률 0·속도 0 이고, 그때도 `allowed_speed` 는 양수일 수 있다).
     - `failed`(끝점 방향 등)·이탈이면 같은 요청의 `plan` 으로 돌아간다(버림 상한에 센다).
     - `arrived` 면 이 다리는 이미 끝난 것이니 설치 뒤 일반 도착 처리로 넘긴다.
     - `cusp_pending`(cusp 방향 보류)이면 그 규칙에 맡기고 출발로 치지 않는다.
     - 실패 없는 `braking`(정상 cusp 전환 tick — 첫 다리가 이미 끝나 다음 다리로 넘어가는 tick, path_tracking 508·585)이면, 복사본을 한 tick 더 미리보아 **실제로 출발할 다리**(다음 기어 다리)와 그 첫 명령을 얻는다(Codex 4차 P2). 그 다리·명령으로 아래 검사·presteer 를 한다. 실제 tracker 의 전환은 재개 뒤 일반 follow 가 같은 순서로 한다.
     - `tracking` 이거나 위처럼 다음 다리를 얻었을 때만 아래 허가 검사로 간다.
   - 허가 검사(장치 안에서 구현, 기준은 그쪽 P6):
   - 그 tick 의 실제 격자 스냅숏을 쓴다.
   - **후보 전용 허가 문맥**에서 검사한다. 기존 허가 객체의 `path_check`·`_driven_since_m` 를 읽으면 옛 경로 상태가 섞인다(Codex 1차 P2-3: 새 경로 `verified_m` 3.0·`blocked` None 인데 허용 0 인 반례).
     - 만드는 법(Codex 2차 P3): 새 `DrivePermission(layer.permission.config)` 를 만든다.
     - 거기에 `layer.footprints(loaded, direction)` 와 **가공된 `layer.snapshot`**(shadow·known·면제 처리가 된 것, obstacle_layer 296)을 넣는다.
     - 후보 첫 다리로 `update` 한 뒤 `allowed_speed(t, layer.last_scan_s, …)` 를 부른다.
     - 원본 허가의 `allowed_speed` 는 `last_estop` 을 바꾸므로 부르지 않는다. raw 격자로 스냅숏을 새로 만들지 않는다.
   - 대상은 후보 tracker 의 **첫 기어 다리**다(`leg_ahead`). 검증 거리가 `min(lookahead, 다리 길이)` 에 닿고 `blocked` 가 None 이어야 한다.
   - 출발 허가: 후보 tracker 의 첫 명령(곡률·방향·속도 상한)으로 `check_arc`·`allowed_speed` > 0 이어야 한다.
   - 지금 바퀴의 **측정 조향 곡률**(runtime 이 쓰는 held curvature, run_transport 8057)로 정지 호 검사도 통과해야 한다(포화 조향에서 반대로 재출발하는 경우 포함).
   - 기록: `verified_m`·`blocked`·첫 틱 허용 속도·두 곡률·이유.
7. `install`: `_live_leg_finish` 의 설치 부분(경로·tracker·운반 도킹 정지점 재무장·새 다리 허가 갱신)을 쓴다. 실패 처리(`live_replanned` 재시도 예약)는 쓰지 않는다 — 장면 모드에는 그 재시도 경로가 없다(Codex 1차 P2-2).
8. `presteer`: 설치 뒤 바퀴 0 으로 첫 명령의 조향각까지 조향을 맞춘다(agy 지적 — 조향 변화율 1 rad/s 로 출발 초기 오차가 생긴다).
   - 완료: 두 앞바퀴 측정 조향이 명령과 0.02 rad 안.
   - 만료: 1.0 s. 만료는 완료와 구분해 기록하고(`presteer: aligned|timeout`, 남은 조향 오차), 다음 단계로 간다.
9. `resume_check`: 출발 직전 다시 검사한다(Codex 2차 P2-2 — 해제 뒤에는 tracking 모드라 새 보정이 바로 적용되고, 정지 중 자세가 6 cm 옮겨지면 직선 후보의 첫 곡률이 0 → −0.36 으로 바뀐 반례).
   - 정지 확인.
   - 지금 제어 자세에서 **설치된 tracker 의 첫 명령**을 다시 계산한다. 계산은 tracker 의 복사본으로 하는 미리보기다. 설치된 tracker 의 진척도·명령 속도·도착·기어 다리 상태를 바꾸지 않는다(Codex 3차 P3).
   - 미리보기의 tracker 상태도 6 처럼 먼저 분기한다(`failed`·이탈이면 재계획, 실패 없는 `braking` 이면 한 tick 더 미리보아 출발할 다리).
   - 최신 스냅숏으로 6 의 허가 검사를 다시 한다.
   - 첫 명령 조향이 presteer 목표와 0.02 rad 넘게 다르면 `presteer` 로 돌아간다(요청당 2 회까지). 허가가 떨어지면 같은 요청의 `plan` 으로 돌아간다(버림 상한에 센다).
   - 통과하면 요청을 닫고 일반 추종을 재개한다.

**종료**(모두 MISSION 분류):

- 예산이 소진되면 `scene_replan_exhausted:<trigger>`.
- 계획 실패면 `scene_replan_failed:<trigger>:<status>`.
- 허가 거부면 `scene_replan_refused:<trigger>:<why>`.
- 시간 초과까지 서 있지 않는다.

**기록**: 요청마다 `state["scene_replans"]` 한 줄(미션 기록 키 `MISSION_KEYS`). 넣는 것:

- 미션 id·단계·tracker 일련번호·다리·트리거.
- 정지 자세(제어·truth). cusp 이면 위치/방향 이탈.
- **직전 SLAM 보정 기여**(2.4).
- 각 단계 시각·계획 결과·탈출 여부·허가 검사·설치 여부·남은 예산.

### 2.2 A — 운반·복귀의 cusp 자세 거부

- 래치가 `ended`(`cusp_position_stall`)를 내는 자리(core 4796–4798)를 바꾼다. 장면 모드이고 단계가 transport·return_home 이면, 끝내는 대신 `scene_replan_request(t, "cusp_pose", detail={이탈})` 을 세운다.
  - 대상은 정지 확인 뒤 자세 조건으로 거부된 정체다(`far` 든 아니든, Codex: L 운반은 `far` 가 아니었다).
  - `cusp_position_stall_unconfirmed`(정지 미확인)는 지금처럼 끝낸다.
  - 요청 뒤 래치는 닫는다(키는 새 tracker 로 바뀐다).
- 래치의 장애물 인계(`force_replan`)도 장면 모드에서 건너뛰지 않는다. `scene_replan_request(t, "cusp_handed")` 로 보낸다.
- 격자 계획 실행(factory 등)은 바뀌지 않는다.
- 설계 7판 개정 문장(그 설계 문서에 덧붙인다): "자세 범위 밖이면 종료 → 장면 모드의 운반·복귀에서는 공통 재계획 장치(유한 예산)로 다리를 다시 계획하고, 예산 소진·실패·허가 거부면 명시 종료한다. 격자 계획 실행은 그대로다."

### 2.3 B — 최초 원거리 관측의 정체 (장면 모드만)

- 대상: 장면 모드에서 최초 원거리 관측 이동(인식 전, `pickup_estimate is None`) 중의 정체.
  - 래치의 `observe_rule` 인계와 기존 `observe_miss`(끝점 방향 실패·진척 없음)가 대상이다.
  - near capture·재정렬·bottom-deck 측면 관측·backoff/escape 는 제외한다.
  - 격자 실행에서는 지금 규칙 그대로다(scene 조건을 명시, Codex 1차 P2-8).
- 순서(6판): **같은 관측점 재계획(지금 규칙, 예산 2)을 먼저** 쓰고, 그 예산이 끝나 시간 초과까지 서게 될 때만 남은 후보를 본다.
  - 5판은 후보를 먼저 봤다. Isaac(`sr_0f88b91`)에서 random_m 두 seed 의 첫 미션이 첫 정체에서 후보를 바꾼 뒤 실패했다. 기준선(#107)은 같은 정체를 같은 관측점 재계획으로 받고 완주했다.
  - 이 근거는 "기존 복구 기회를 보존한다" 까지다. 기준선도 후보 2 에서 인식해 완주했으므로 낮은 순위 후보가 나쁘다는 증거는 아니다(Codex Isaac 상의).
  - 후보 호출 전체(hold 해제 포함)를 예산 소진 뒤로 둔다. 예산이 남은 동안에는 지금과 똑같이 동작한다.
  - 1422 분기는 같은 목표 계획이 실패해야만 열리므로 호출만으로는 안 된다(Codex 1차 P2-7). 그래서 후보 선택을 따로 둔다.
  1. 같은 관측점 재계획 예산이 끝난 정체에서: 정지 → hold 면 해제·재투영(인식 전 후보는 지도 좌표라 옮기지 않는다).
  2. **남은 관측 후보**가 있으면 `next_candidate_index` 의 다음 후보로 관측 다리를 계획한다. tick 안에서 동기로 계획하되 **한 tick 에 한 후보**만 계획한다. 이는 기존 같은 관측점 재계획(core 2196, live 에서도 동기)과 같은 방식이다. 실패하면 다음 후보는 다음 tick 에 계획한다(Codex 구현 1차 P2-2 — 여러 후보를 한 tick 에 돌면 제어 tick 이 그만큼 막힌다).
     - 성공하면 설치하고 `observation_waypoint_selected`·`observe_replan_leg`(새 serial·start)를 갱신한다.
     - 실패하면 다음 후보로 간다.
     - 새 후보가 설치되면 그 다리의 같은 관측점 재계획 예산 2 가 새로 열린다(`observe_replan_leg` 새 serial·start, 지금 규칙).
  3. 후보가 없거나 다 실패하면 **명시 종료** `observe:stall_exhausted`(MISSION). 지금처럼 명령 0 으로 시간 초과까지 서지 않는다(기존 `observe_miss` 의 예산 조건 밖에 둔다).
  - 같은 관측점 계획이 `invalid_start` 로 실패하면 지금처럼 바로 끝난다. 예산 안의 `endpoint_heading` 처리도 그대로다. 이 개정은 기존 예산이 끝난 뒤의 복구만 더한다.
- "선 자리에서 관측"(즉석 관측점)은 이번에 넣지 않는다(시야·거리·가림 판정이 별도 관측점 선택이라 범위가 커진다).
- 기록은 같은 `scene_replans` 에 trigger `observe_stall` 로 남기되, 예산은 2.1 과 따로 센다(2.4).

### 2.4 예산과 기록 정의

- **공통 장치(2.1)**: 요청 한 건이 수락될 때 1 을 센다. 미션 합산 4 회, 트리거별 3 회다.
  - 같은 요청의 작업자 재제출·탈출 뒤 재계획·presteer 는 세지 않는다.
  - 같은 자리 반복도 센다(묶으면 무한히 이어질 수 있다).
  - 새 tracker·단계 시작·일시 정지(`mission_gen`)에 초기화하지 않는다. 미션 끝(`end_reset`)에만 초기화한다. 저장 자리는 core 속성이고 `CORE_FIELDS` 초기화에 넣는다(Codex 1차 P3).
- **B**: 후보 전환은 남은 후보 수만큼, 같은 관측점 재계획은 지금 예산 2 다. 2.1 예산과 따로다.
- **기존 카운터**(`cusp_replans`·`stall_replans`·`live_plans`)와는 독립이다. 4/3 은 미션 전체 재계획 상한이 아니라 이 장치의 상한이다.
- **SLAM 보정 기여**(Codex 1차 P3): 같은 오도메트리 자세에 적용 전·후 보정을 합성한 차이를 쓴다(`SlamPoseTracker.release` 와 같은 정의, slam_pose.py 261).
  - 지금 tracker(기어 다리) 시작 이후 tick 별 순변화의 최대와 그 시각을 남긴다. 그 tick 안 개별 응답의 최대도 따로 남긴다.
  - sim 에서는 제어−truth 차의 한 tick 변화도 함께 남긴다(실차에는 없다).

### 2.5 하지 않는 것

- 정지 판정 문턱·cusp 허용치·#105 다리 끝 하한·래치 판단 규칙(K2/K3)은 바꾸지 않는다.
- 허가 띠·계획 여유(P1c)·잡음 측정(P3)은 그 세션 소관이다(별도 PR). permit_stop 트리거(허가 상한 0 + `occupied` 가 T 초, 막힌 칸 사각형, 여유 + 0.10)도 그 세션이 이 장치 위에 얹는다(이 PR 머지 뒤).
- 러너 인자를 새로 두지 않는다(장면 모드면 켜진다). 혹시 두게 되면 근접 추적 계약(`near_tracking.py` RUN_SPECIFIC/POST_MEASUREMENT)에 등록하고, 재생 코덱에서 기본값을 기록에서 빼는 처리(`replay_codec` 의 exit_prefix 방식)를 한다.
- 다리 끝 근처 보정 보류(C)는 원인 분리 결과에 따라 따로 올린다.

### 2.6 원인 분리 (별개 보고)

- 같은 기록 경로를 재생해 두 효과를 따로 본다. 하나는 보정 튐을 넣고 뺀 것, 다른 하나는 기록된 조향/구동 응답(명령 대비 실측 지연·변화율·구동 제한)을 넣고 뺀 것이다.
- L 운반·R 관측 3 에서 마지막 다리의 이탈이 어느 쪽으로 재현되는지 보고, 총괄 세션에 보고한다.

## 3. 시험 (먼저)

- 공통 장치(core, 가짜 계획기·허가):
  - 요청 tick 은 추종·정체·도착을 건너뛴다. 같은 tick 의 hold 가 요청을 무효로 만들지 않는다(실제 `follow()` 로).
  - 해제 뒤 제어 자세가 갱신되고, `extra_obstacles_fn` 은 해제 뒤에 불린다.
  - 작업자 응답이 버려지면(superseded·moving·correction) 같은 요청을 다시 보내고 예산은 그대로다. 일시 정지 뒤 이어진다.
  - tight 재시도 없음. `invalid_start` → 탈출 접두(장면 장애물·추가·여유로) → 주행·정지 → 같은 요청 재계획.
  - 허가 검사는 후보 전용 문맥이다(옛 경로 상태를 읽지 않음). 첫 기어 다리, 측정 조향 정지 호를 본다.
  - presteer 는 조향이 맞거나 1 s 뒤 끝난다.
  - 허가 검사: 검증 거리 부족·blocked·첫 틱 허용 0 이면 설치하지 않는다(사유별). 접두도 첫 틱부터 검사한다.
  - 후보 전용 허가 문맥은 **실제 객체**로 시험한다(가짜 허가만으로 하지 않음): 원본 허가 객체·스냅숏이 불변이고, 원본 허용 0 인 자리에서 후보가 허용 > 0 이 되는 경우(Codex 2차 P3).
  - 상태별 출력: 대기는 조향 hold, presteer 는 hold 없이 조향 변경, escape 는 전용 추종. 요청 중 `live_leg` 이 끼어들지 않는다.
  - `resume_check`: 정지 중 자세가 옮겨져 첫 명령이 바뀌면 presteer 로, 허가가 떨어지면 재계획으로 간다.
  - 후보·미리보기 tracker 의 상태 분기: `failed`·이탈 → 재계획, `arrived` → 도착 처리, `cusp_pending` → 그 규칙. Codex 반례(첫 다리 6 cm, 자세 x +5.6 cm·yaw +0.09 rad → `failed/endpoint_heading`)가 출발로 통과하지 않는다.
  - 미리보기는 설치된 tracker 를 바꾸지 않는다(불변성).
  - 정상 cusp 전환 tick(첫 다리 6 cm, x +5.6 cm·yaw 0 → 실패 없는 `braking`): 한 tick 더 미리보아 다음 다리로 검사·presteer 하고, 재개 뒤 일반 follow 가 다음 다리로 넘어간다(요청 기한까지 기다리지 않는다).
  - 탈출 완료 → 같은 요청의 정상 경로 설치 → 소유 표시 제거·설정 복원 → 일반 follow.
  - 탈출 중 허가: 소유 표시로 `permission_acting` 이 참이고, 허용이 0 이 되면 서서 `escape_stopped` 로 끝난다.
  - 요청 전체 60 s 는 계획 대기를 포함한 시뮬 시간이고, 일시 정지만큼 미뤄진다.
  - 상한: 요청별 버림 3 회, 요청 전체 60 s, `moving` 버림은 정지 대기로 간다. `_drop` 의 site 누적 상한에서 빠진다.
  - `clearance_m` 덮어쓰기가 작업자 payload·동기 호출 둘 다에 간다. `extra_obstacles` 가 장면 장애물에 더해진다.
  - 예산: 합산 4·트리거별 3, 같은 자리 반복도 센다.
  - 장면 모드 transport·return_home 에서 요청 → 정지 대기 → 계획 → 허가 통과 → 설치(운반이면 도킹 재무장)·기록 한 줄.
  - `invalid_start` → 탈출 접두 → 재계획.
  - 실패·허가 거부·예산 소진 → 각 사유로 MISSION 종료.
  - 예산은 새 tracker·단계 시작에 초기화되지 않는다.
  - 격자 계획·D7 수용 뒤·제외 단계에서는 요청이 아무것도 하지 않는다(지금 규칙 그대로).
  - live 작업자 경로(작업 하나만, 응답 때 설치)와 배치 동기 경로.
- A:
  - 래치가 `ended` 를 내는 경우(정지 확인 뒤 자세 거부, `far` 아님·`far` 둘 다)에 장면 모드 운반·복귀면 종료 대신 요청한다.
  - 미확인 정지는 그대로 종료한다. 격자 계획은 그대로 종료한다.
  - 장면 모드 `force_replan` 인계 → 요청.
- B:
  - 장면 모드 인식 전 관측 정체 → 같은 관측점 재계획(예산 2) → 다른 후보(hold 해제, 관측 선택·serial 갱신) → `observe:stall_exhausted`.
  - 격자 실행·near·bottom-deck·재정렬은 바뀌지 않는다.
- 기록: `scene_replans` 줄의 필드, 보정 튐 계산(가짜 제어/truth 열).
- 분류: 새 사유 넷이 MISSION 이다. 콘솔 사유 문구.
- 재생 게이트 29/29(격자 계획 기록이라 동작이 바뀌지 않아야 한다 — 기존 동작 보존용이고 새 장치의 검증은 아니다), ws1 전체.
- 새 장치의 CPU 통합 시험(Codex 1차 P2-8): 장면 모드 v1 live·배치에서 return_home, hold 해제, 탈출 완료, correction 버림, 미션 2 회 연속(예산 초기화)을 본다.

## 4. 검증 (Isaac, ws1)

- line_m·random_m 세션 live × seed 0·1(필요하면 2), 앵커(PR #107) 위에서.
- 기록할 것:
  - 미션별 결과.
  - `scene_replans` 발동 수·트리거·이탈·직전 보정 튐·결과(실행별 집계 — 사용자 조건 1).
  - 연속 완주 수.
- 결과 문서에 집계 표를 넣는다. 원인 분리(2.6)는 별도 보고로 둔다.

## 5. 위험·범위 밖

- 재계획한 다리도 같은 보정 튐에 다시 빗나갈 수 있다. 예산 N 이 그것을 유한하게 묶는다. 반복되면 원인(C)의 근거가 된다.
- 정지 자세가 허가 띠 안이면 재계획 출발이 `invalid_start` 다. 탈출 접두로 한 번 빼고, 그래도 안 되면 끝낸다(P6 위험과 같음).
- 실차: 보정 튐 기록은 truth 없이 보정 적용량만 남는다.

## 6. 자체 검토 — 직접 재계산

- 1.1 표의 시각·이탈·`far` 는 원자료 `cusp_stalls` 에서 다시 읽었다(Codex 재확인과 같음).
- 1.2 의 횡오차는 `slam.control` 을 정체 다리 구간으로 제한해 다시 계산했다.

## 7. 검토 기록

### 6판 — Isaac 결과로 B 순서 개정 (Codex 상의)

- 근거: 위 2.3. Codex 는 재정렬에 동의했다(새 P1/P2 없음). 다음 세 가지를 바로잡았다.
  - 기준선은 후보 2 에서 인식했다. 그래서 근거는 복구 기회 보존까지다.
  - 추가 hold 해제는 두 실패의 공통 원인이 아니다.
  - 호출 전체를 소진 뒤로 옮겨야 한다(후보가 없어도 해제가 먼저 일어나기 때문).
- random_m s0 m3 의 최종 목표 정체(위치 0.1013 m, loose 도착 0.10 m 를 1.25 mm 넘음, cusp 아님)는 B 와 별개 부류다. 다만 m1 실패로 P-4 가 제자리에 남아 장면이 달랐으니 B 와 무관하다고 단정하지 않는다.

### 구현 리뷰 2차(Codex) — 수렴

- 1차 지적의 수정(`a22ec45`)을 확인했다. 새 P1/P2 는 없다.
- P3 두 건(진단 기록)을 고쳤다. 첫째, 후보가 실패해 원래 관측점으로 돌아간 뒤에도 관측 시도 기록이 그 관측점이 나온 후보 번호를 남긴다. 둘째, 보정 기여의 방향 차이를 ±π 에서 감싼다.

### 구현 리뷰 1차(Codex) — 계획 문구 변경 하나

- B 의 후보 계획: live 작업자 대신 tick 안 동기, 한 tick 한 후보(2.3). 기존 관측 재계획과 같은 방식이고, 여러 후보를 한 tick 에 계획하던 구현을 묶었다.
- 그 밖의 지적(보정 기여를 변환 합성으로, B 의 해제 먼저, 탈출 설정 소비, 제출마다 정지 재확인, 배치 출구 접두 중복, 바퀴별 presteer, 작업자 예외 기록)은 구현 수정이다.

### 5판 수렴

- **② Codex 재검토(5차)**: 새 P1/P2 없음(수렴). P3(후보 허가의 검증 거리 비교가 `step_m` 만큼 느슨함)는 구현에서 수치 오차(1e-6 m)만 허용하도록 고쳤다.

### 4판 → 5판

- **② Codex 재검토(4차)**: 새 P1 없음, P2 하나·P3 하나 — 받아들였다.
  - P2 정상 cusp 전환의 `braking` 이 분기에서 빠짐 → 한 tick 더 미리보아 출발할 다리로 검사, 실제 전환은 일반 follow.
  - P3 탈출 뒤 소유 표시 제거·설정 복원 → 명시하고 시험 추가.

### 3판 → 4판

- **② Codex 재검토(3차)**: 새 P1 없음, P2 둘·P3 둘 — 받아들였다.
  - P2-1 `resume_check` 가 실패한 tracker 를 통과시킴 → 후보·미리보기 tracker 상태를 먼저 분기.
  - P2-2 탈출의 매 tick 허가가 운반 도킹 예외로 꺼짐 → 기존 escape 소유 표시를 세우고, 허용 0 이면 `escape_stopped`.
  - P3 요청 60 s 의 시계 → 계획 대기를 포함한 시뮬 경과 시간, 일시 정지만큼 미룸.
  - P3 첫 명령 재계산이 tracker 를 바꿈 → 복사본 미리보기와 불변성 시험.

### 2판 → 3판

- **② Codex 재검토(2차)**: 새 P1 없음, P2 셋·P3 둘 — 모두 받아들였다.
  - P2-1 요청 중 명령 소유가 escape·presteer 와 모순 → tick 순서와 상태별 출력 표, `live_leg` 배제.
  - P2-2 presteer 뒤 출발 조건 변화 → `resume_check`(정지·첫 명령 재계산·허가 재검사).
  - P2-3 재제출·유한 종료 → `moving` 은 정지 대기로, 요청별 버림 3 회, 요청 전체 60 s, `_drop` site 상한에서 제외.
  - P3 후보 허가 문맥 만드는 법·실제 객체 시험, presteer 완료/만료 구분과 허용 0.02 rad.

### 1판 → 2판

- **① 직접 재계산**: 1.1 의 래치 한계(위치 2×0.03 = 0.06 m·방향 0.05 rad, `far` +0.03 m·+0.03 rad, `cusp_stall.py` 66–89)와 네 건의 판정을 다시 대조했다.
  - L 운반 74.6 mm 는 한계 밖·`far` 아님이다.
  - R 관측 1 은 49 mm(위치 안)·62.5 mrad(방향 밖)다.
  - `OBSERVE_REPLANS_PER_LEG` = 2, `OBSERVE_ARRIVAL_M` 0.10(run_transport 49·63).
- **② Codex 적대 검토(1차)**: 새 P1 없음, P2 여덟·P3 둘 — 모두 받아들였다.
  - P2-1 요청 tick 의 hold 무효화 → 요청은 플래그, tick 앞 상태 기계, 해제 뒤 자세 갱신.
  - P2-2 작업자 폐기·재요청·실패 처리 → 요청 id·버림 재제출·장면 전용 실패 처리.
  - P2-3 허가 검사가 옛 경로 상태를 읽음 → 후보 전용 문맥·첫 기어 다리·측정 조향 정지 호.
  - P2-4 해제 전 추가 장애물의 프레임 → 콜백으로 해제 뒤 생성(인터페이스 변경, "v-g2" 에 알림).
  - P2-5 격자 탈출 래퍼 사용 불가 → `escape.*` 직접, 주행 뒤 같은 요청 이어 가기, 요청당 1 회.
  - P2-6 tight 재시도가 탈출·여유를 깨뜨림 → 재시도 없음, 장치 전용 payload, 20 s 기한.
  - P2-7 B 를 1422 분기 호출로 못 함 → 후보 선택·hold·예산을 분리, 소진 종료를 예산 조건 밖에.
  - P2-8 scene ≠ grid 꺼짐, 검증 범위 → 명시 scene 조건, CPU 통합 시험 추가.
  - P3 예산 세는 단위·초기화 자리, 보정 기여 정의 → 2.4.
- **③ 다른 관점의 독립 검토(agy, Gemini 3.8 Flash — `agy-quota-ok --pick`, ② 결론 비공개)**:
  - 받아들임: 재계획 출발 자세의 안정(정지·해제·보정 대기 뒤 계획 — 2.1 의 1–3), 정지 상태 조향 맞추기(presteer), 반복·예산 소진·결함 가림 위험(5 절·기록으로 보이게).
  - 받아들이지 않음:
    - 중간 cusp 허용치 완화(fly-by) — 기존 cusp 쓸림 검증을 벗어난다(Codex 진단). 래치의 해제(범위 안 수렴) 경로는 이미 있다.
    - 경로 생성에 조향 변화율 반영(클로소이드) — 계획기 전체 변경이라 범위 밖, 원인 분리 뒤 판단.
    - 관측 느슨한 도착 확장 — cusp 까지 오차와 관측점까지 거리가 달라 단순 확장 불가(Codex).
  - 보류: 다리 끝 근처 보정 보류(C) — 사용자 결정대로 원인 분리 뒤 따로.
