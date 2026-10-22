# 장면 모드 재계획 장치와 다리 끝 cusp 정체 복구 — 계획 (2026-10-23, 1판)

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

- 진입점 `core.scene_replan_request(t, trigger, *, extra_obstacles=(), clearance_m=None, detail=None)`. 인터페이스는 "v-g2" 세션과 맞췄다(2026-10-23 회신).
  - `trigger`: `"cusp_pose"`(A) · `"cusp_handed"`(래치의 장애물 인계, 장면 모드) · `"permit_stop"`(P6, 그 세션).
  - `extra_obstacles`: 계획 프레임 Rectangle 들. P6 은 막힌 칸들을 칸 정사각형으로 만들고 이웃끼리 합쳐 넘긴다. 격자 스냅숏을 계획기에 넘기지는 않는다.
  - `clearance_m`: 이번 재계획의 계획 여유 덮어쓰기(None 이면 평소 값). P6 은 그 자리에서만 평소 + 0.10 m 로 키운다(최종 값은 그쪽 계획서 v4).
  - `detail`: 기록용.
- 적용 조건: 격자 계획 꺼짐(`--layout scene`), live·배치 모두, 단계 `transport`·`return_home`.
  - 제외(지금 규칙 유지):
    - D7 도킹 수용 뒤·재접근 중이면 지금처럼 `transport_recovery_after_docking`/도킹 재시도로 간다.
    - near·bottom-deck·held realign·backoff/escape 진행 중은 그 규칙을 따른다.
- 순서:
  1. 정지 확인(`stop_now`, 없으면 truth 속도)까지 0 명령. 래치가 이미 정지를 인정했으면 바로 간다.
  2. hold 중이면 기존 해제(`release_and_carry`)로 풀고 재투영한다.
  3. 정지 자세에서 그 단계의 다리를 계획한다.
     - transport → `plan_transport_leg`(transport_scenario, 출구 접두 포함).
     - return_home → `plan_return_leg`(return_scenario).
     - 장애물 = 계획기가 장면에서 만드는 장애물 + `extra_obstacles`. 장치는 장애물 집합을 따로 만들지 않는다. 그쪽 P1c(탐색 장애물 키움)가 계획기에 들어오면 자동으로 같은 규칙을 탄다.
     - live 는 작업자(`leg` 작업, 새 site `scene_replan`), 배치는 동기다.
  4. `invalid_start` 면 탈출 접두(직진 후진, 기존 escape 예산 단위)를 한 번 넣고 그 끝에서 다시 계획한다. 접두 자체도 5 의 허가 검사를 첫 틱부터 통과해야 한다(그쪽 Codex 반례: 끝점 이격만 맞춘 접두가 출발 틱에 허가 0).
  5. 설치 전 허가 검사(장치 안에서 구현, 기준은 그쪽 P6):
     - 그 틱의 실제 격자 스냅숏(지금 허가가 든 `GridSnapshot`)을 쓴다.
     - 새 경로(접두가 있으면 접두부터)로 `check_path` 를 돌린다. 검증 거리가 `min(lookahead, 경로 길이)` 에 닿고 `blocked` 가 None 이어야 한다.
     - 추종기 첫 명령(곡률·방향·속도 상한)으로 `check_arc`·`allowed_speed` 가 0 보다 커야 한다.
     - 둘 다 통과한 계획만 `_live_leg_finish` 와 같은 경로로 설치한다(운반이면 `arm_docking`). 계획기 자신의 충돌 검사는 계획 단계에서 이미 한다.
     - 기록: `verified_m`·`blocked`·첫 틱 허용 속도·이유.
- **예산**: 미션 전체 합산 4 회, 트리거별 상한 3 회(2.4). 새 경로·tracker·단계 시작에 초기화하지 않는다. 같은 자리 반복(직전 발동과 정지 자세가 0.10 m 안이고 같은 트리거)도 묶지 않고 그대로 센다.
  - 소진되면 `scene_replan_exhausted:<trigger>` 로 끝낸다.
  - 재계획·탈출이 실패하면 `scene_replan_failed:<trigger>:<status>` 로 끝낸다.
  - 허가 검사에 떨어지면 `scene_replan_refused:<trigger>:<why>` 로 끝낸다.
  - 셋 다 MISSION 분류다. 시간 초과까지 서 있지 않는다.
- **기록**: 발동마다 `state["scene_replans"]` 에 한 줄을 남긴다(미션 기록 키 — baseline `MISSION_KEYS`). 넣는 것:
  - 미션 id·단계·tracker·다리·트리거.
  - 정지 자세(제어·truth).
  - cusp 이면 위치/방향 이탈.
  - **직전 SLAM 보정 튐**: 마지막 다리 시작 이후 가장 큰 한 tick 제어−truth 변화와 그 시각. live 는 보정 적용 기록의 이동량을 쓰고, truth 가 없는 실차에서는 보정 적용량만 쓴다.
  - 계획 결과·tight·탈출 여부·허가 검사·설치 여부·남은 예산.

### 2.2 A — 운반·복귀의 cusp 자세 거부

- 래치(`cusp_stall`)가 `ended`(`cusp_position_stall`)를 내는 자리(core 4796–4798)를 바꾼다. 장면 모드이고 단계가 transport·return_home 이면, 끝내는 대신 `scene_replan_request(t, "cusp_pose", detail={이탈})` 을 부른다.
  - 대상은 `far` 만이 아니라 **정지 확인 뒤 자세 조건으로 거부된 정체**다(Codex: L 운반은 `far` 가 아니었다).
  - `cusp_position_stall_unconfirmed`(정지 미확인)는 지금처럼 끝낸다.
- 래치의 장애물 인계(`force_replan`)도 장면 모드에서 건너뛰지 않는다. `scene_replan_request(t, "cusp_handed")` 로 보낸다.
- 격자 계획 실행(factory 등)은 바뀌지 않는다. 그쪽은 지금의 장애물 재계획·종료 규칙 그대로다.
- 설계 7판 개정 문장: "자세 범위 밖이면 종료 → 장면 모드의 운반·복귀에서는 공통 재계획 장치(예산 유한)로 다리를 다시 계획하고, 예산 소진·실패면 명시 종료한다. 격자 계획 실행은 그대로다."

### 2.3 B — 최초 원거리 관측의 정체

- 대상: 최초 원거리 관측 이동(인식 전, `pickup_estimate is None`) 중의 cusp 자세 거부·관측 재계획 예산 소진.
  - near capture·재정렬·bottom-deck 측면 관측·backoff/escape 는 제외한다(Codex: 그쪽은 자기 복구 규칙이 있다).
- 순서(같은 관측점 재계획보다 먼저):
  1. **다른 관측 후보**로 넘긴다 — 인식 전 후보 전환(core 1422 의 규칙)과 같은 경로, 남은 후보가 있을 때.
  2. 후보가 없으면 **같은 관측점 재계획**(지금 규칙, 예산 2).
  3. 그것도 끝나면 **명시 종료** `observe:stall_exhausted`(MISSION). 지금처럼 명령 0 으로 시간 초과까지 서지 않는다.
- "선 자리에서 관측"(즉석 관측점)은 이번에 넣지 않는다. 시야·거리·가림 판정이 별도 관측점 선택이라 범위가 커진다(Codex). 후보 전환·종료로 시간 초과를 없애는 것까지만 한다.
- 관측 단계의 재계획은 2.1 장치를 쓰지 않는다(관측 다리 계획기는 따로다). 예산·기록 형식은 2.1 과 맞춘다(`scene_replans` 에 trigger `observe_stall` 로 함께 남긴다).

### 2.4 예산

- 미션 전체 합산 4 회, 트리거별 상한 3 회("v-g2" 세션 제안 — 한 트리거가 다 쓰지 않게). 관측(B)은 지금 예산(같은 관측점 2)을 그대로 쓰고 2.1 예산과 별도다.
- 근거: 이번 실패는 미션당 1 건(운반)과 3 건(관측)이었다. 같은 다리를 세 번 넘게 다시 계획해도 안 되면 원인이 다른 데 있다(예: 보정 튐 반복).
- 같은 자리 반복도 센다. 반복은 소진으로 끝나야 한다(묶으면 무한히 이어질 수 있다).

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
  - 허가 검사: 검증 거리 부족·blocked·첫 틱 허용 0 이면 설치하지 않는다(사유별). 접두도 첫 틱부터 검사한다.
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
  - 인식 전 관측 정체 → 다른 후보 → 같은 관측점 재계획 → `observe:stall_exhausted`.
  - near·bottom-deck·재정렬은 바뀌지 않는다.
- 기록: `scene_replans` 줄의 필드, 보정 튐 계산(가짜 제어/truth 열).
- 분류: 새 사유 넷이 MISSION 이다. 콘솔 사유 문구.
- 재생 게이트 29/29(격자 계획 기록이라 동작이 바뀌지 않아야 한다), ws1 전체.

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

(검토 뒤 채운다.)
