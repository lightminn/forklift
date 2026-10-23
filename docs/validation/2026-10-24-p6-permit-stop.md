# P6 허가 정지 재계획 트리거 — 검증 결과 (2026-10-24)

계획: `docs/plans/2026-10-23-planner-permission-clearance.md` 3 장 P6. 사용자 결정 2026-10-23: P6 을 장면 모드 재계획 장치(PR #112, `docs/plans/2026-10-23-scene-replan.md`)에 트리거로 얹는다.
구현: branch `feat/p6-permit-stop`(15e0d271). 장치 쪽 계약 변경은 장치 작성 세션(sw-연속이송-재관측)과 합의했다.

## 무엇을 바꿨나

- **트리거**(`MissionCore.permit_stop_tick`, `core.permission()` 끝에서 부름):
  - 조건: 장면 레이아웃의 운반·복귀 다리, 허가가 실제로 적용 중(acting, obstacle_act), 허가 상한 0·이유 `occupied`, 장치가 다른 요청을 처리하고 있지 않음, 장치의 제외 조건(`scene_replan_allowed`)을 모두 만족.
  - 이 조건이 작업 시간 3 s 이어지면 `scene_replan_request(t, "permit_stop", …)` 를 부른다. 감시는 틱 간격이 0.25 s 를 넘거나(장치 소유·일시정지) 추적기·단계가 바뀌면 다시 시작한다.
  - 한 다리·추적기당 한 번만 발동한다. 예산(트리거당 3, 미션 합산 4)이 다 되면 그 미션에서 감시를 멈출 뿐, 미션을 끝내지 않는다.
- **추가 장애물**(`permit_stop.current_blocked_rectangles`):
  - 장치가 계획하는 순간(SLAM hold 해제·재투영 뒤)에 그때의 허가 스냅숏 위에서 현재 다리 앞과 정지 원호를 다시 검사한다.
  - 막힌 OCCUPIED 칸을 0.5 m 안의 8 연결 점유 칸까지 키워, 칸 사각형(행 단위 병합)으로 넘긴다.
  - 저장된 검사는 쓰지 않는다. hold 해제는 차 자세만 다시 검사해서, 저장된 칸이 이전 스냅숏의 것이기 때문이다(Codex 1 회차 P2).
- **여유**: 그 다리의 계획 여유에 0.10 m 를 더한다.
- **스위치** `--permit-stop-replan on|off`:
  - 새 실행은 on 이다.
  - 이 인자가 없는 옛 기록은 off 로 재생된다(`getattr` 기본값).
  - 근접 계약값은 on 이다. bay 레이아웃에서는 트리거가 발동하지 않아 어느 값이든 같기 때문이다.
- **장치 계약 변경**(`SceneReplanConfig.standing_safe_triggers`, 기본 `("permit_stop",)`):
  - 이 목록에 든 트리거의 요청이 설치·탈출 없이 끝나면, 원래 추적기가 hold 해제 뒤 자세에서도 따라갈 수 있을 때(`resumable()` 포트, Codex 3 회차 P2)만 미션을 끝내지 않고 틱을 돌려준다.
  - 해당하는 끝: 계획 실패·탈출 없는 invalid_start·설치 전 refused·timeout·drop 초과·콜백 예외.
  - 기록에는 `mission_end: false` 와 `reason` 이 남고, 미션 합산 예산은 되돌린다.
  - 설치·탈출 뒤의 실패와, 다른 트리거(cusp_pose·cusp_handed)의 끝은 지금처럼 미션을 끝낸다.
  - 모든 기록 줄에 `mission_end` 가 붙는다.

## 왜 장치 계약을 바꿨나 (1 차 Isaac)

같은 시간대에 켬/끔을 섞어 돌렸다(스냅숏 p6_24f83a14, seed 0–2, 이때는 실패가 미션을 끝내던 판).

| 장면 | 켬 | 끔 |
|---|---|---|
| trailer(`--exit-clearance-m 0.30`) | 1/3 — s0·s2 가 운반 정지 3 s 에 발동 → `scene_replan_failed:permit_stop:invalid_start` 로 종료 | 3/3 — 같은 seed 가 허가 정지(누적 1 378·620 틱)를 서서 기다려 완주 |
| aisle_rack | 0/3(invalid_start 빠른 종료 2, 근접 캡처 실패 1) | 0/3(운반 시간 초과 3) |
| line_2_m | 3/3 | — |

트리거가 저절로 풀렸을 정지를 미션 종료로 바꿨다. 그래서 "서 있는 것보다 나빠지지 않게" 계약을 바꿨다.

## 결과 (최종, p6_15e0d271)

- **재생 게이트** runs29: 29/29 exit 0. 옛 기록은 스위치 off 로 재생된다.
- **ws1 전체 pytest**: 6426 passed · 10 failed · 21 skipped. 실패 10 건은 알려진 ws1 전용 목록이다.
- **Isaac**(같은 배치에 켬/끔을 섞음, KEEPOUT 0, 장면 `scenes_p1c_81a007b1`):

| 장면 | seed | 켬 | 끔 |
|---|---|---|---|
| trailer(`--exit-clearance-m 0.30`) | 0–4 | 3/5 | 3/5 |
| aisle_rack | 0–2 | 0/3 — s0·s2 는 운반을 지나 하역 뒤 복귀에서 시간 초과, s1 은 운반 시간 초과 | 0/3 — 모두 운반 시간 초과 |
| line_2_m | 0–2 | 3/3 | — |

- **trailer**: 켬과 끔이 같다.
  - 켬에서 발동한 2 건(s1·s3)은 모두 설치 전 `invalid_start` 로 넘겨졌다(`mission_end: false`). 그 뒤 끔과 같이 서 있다가 시간 초과로 끝났다.
  - 실패 seed 는 켬과 끔이 서로 다르다(켬 s1·s3, 끔 s0·s1). 실행 사이 비결정성 때문이다.
- **aisle_rack s0**: P6 가 실제로 회복했다.
  - 운반에서 invalid_start → 곧은 후진 탈출 완료 → 재계획 설치가 두 번 이어졌다(`installed`, escape `done`).
  - 그 뒤 운반·하역을 지나 복귀에서 예산이 소진(`exhausted`)됐다. 이때 미션은 끝나지 않고 서 있다가 복귀 시간 초과로 끝났다.
- **aisle_rack s2**: 운반에서 두 번, 복귀에서 한 번 넘겨졌다. 서 있는 동안 표시가 걷혀 운반을 지났다.
- 끔은 rack 세 seed 가 모두 운반에서 멈췄고, 켬은 두 seed 가 하역까지 갔다. 완주 수는 둘 다 0 이다.

## 한계

- **정지 자리에서 탈출이 막힌다.**
  - 실기록 덤프 6 건(트레일러 운반 정지 3, rack 운반 정지 3)을 CPU 에서 재현했다. 정지 자세는 추가 장애물에서 0.008–0.03 m, 장면 소품에서 0.19–0.23 m 다. 그래서 여유 0.10·0.20 모두 `invalid_start` 다.
  - 곧은 후진 0.3–0.8 m 는 6 건 중 5 건이 실시간 허가에서 0 m 다. 정지 외곽에서는 막힌 칸과 안 겹치지만, 첫 후진 구간 중간점의 쓸림 여유 0.01 m 가 그 칸에 닿는다.
  - 탈출 계획의 사각형 검사(`escape.py` 205 행)도 출발 자세를 포함해 막힌다.
  - 이 자리들에서 P6 는 회복하지 못하고, 넘겨준 뒤 서 있는다. 끔과 같은 결과다.
- **넘겨준 뒤에도 시간은 돈다.** 일반 추종 시간 초과(3 × 경로 시간 + 10 s)는 넘겨준 뒤 서 있는 동안 그대로 흐른다. 끔 실행과 같은 조건이다.
- **표본이 작고 실행이 비결정적이다.** 같은 코드·seed 도 t ≈ 8 s 부터 갈라진다. 위 비율은 같은 시간대에 섞은 seed 로만 읽는다.

## 남은 결정 (사용자)

막힌 자리의 탈출을 열려면 계획 쪽과 허가 쪽 완화가 함께 필요하다(장치 작성 세션과 범위 합의).
- **계획 쪽**: 출발 외곽에 이미 걸치거나 닿은 **추가 장애물 사각형만**, 출발 검사와 탈출 쓸림 검사에서 "멀어지는 동안 통과"로 친다. 장면 소품에는 지금 규칙을 그대로 둔다.
- **허가 쪽**: 쓸림 여유(0.01 m)가 처음 닿는 칸이 출발 외곽에서 그 여유 안에 있을 때만, 그리고 진행하는 동안 외곽과 그 칸의 거리가 줄지 않을 때만 통과로 친다. 앞쪽의 새 칸과 정지 원호 검사(`allowed_speed`)는 바꾸지 않는다.
- 둘 다 안전 규칙 변경이라 사용자 결정 사항이다. 결정 전까지 P6 는 위 결과대로 "나빠지지 않는 트리거"로 둔다.

## 검증

- Codex: 구현 4 회차에서 수렴했다(15e0d271, 새 P1/P2 없음). 반영한 지적은 1 회차 P2(저장된 막힌 칸)·P3(감시 시계), 3 회차 P2(hold 해제 뒤 복귀 가능성)다.
- 시험:
  - `tests/unit/mission/test_permit_stop.py`
  - `tests/unit/mission/test_scene_replan.py`(standing-safe 경계 7 건 추가)
  - `tests/unit/tools/test_permit_stop_runner.py`
  - 장치 작성 세션의 시험(`test_scene_replan.py`·`test_core_scene_replan.py`·`test_scene_replan_recorded_cusp.py`) 그대로 통과. 관련 62 passed.
- ws1 경로:
  - `test_runs/riibotics/p6_{on,off}_*`(1 차), `p6b_{on,off}_*`(최종)
  - 재생 게이트 `test_runs/replay_gate/p6_15e0d271`
  - 스냅숏 `snapshots/p6_15e0d271`
