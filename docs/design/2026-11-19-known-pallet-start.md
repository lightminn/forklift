# 알려진 팔레트로 시작 + S10 들고 멈춤 — 4a (설계)

작성 2026-11-19 · 2판(Codex 1 회차 반영) · 상위 [실차 자율 계획](../plans/2026-11-18-real-autonomy-entry.md) PR 4(S10 사례 A 범위, 총괄 승인 2026-11-19) · [제어 틱 계약](2026-11-18-real-control-tick.md) 2절 ④

## 범위

S10 사례 A: 테이프로 표시한 알려진 시작 → 접근 → 삽입 → 운반 높이까지 들고 멈춤. 팔레트 자세는 운용자가 지도 좌표로 입력한다(줄자 시험장 배치, 인식 아님). 코어 변경은 새 메서드·옵션과 한 번의 '그대로 옮기기'(1절 설치 공유)뿐이고, 기본은 쓰지 않으므로 Isaac 경로는 그대로다(재생 게이트 29/29, 전체 pytest).

## 1. 알려진 팔레트로 시작

- 지금 미션은 늘 `observe`(관측 다리 → 포획 → 자세 → 계획)로 시작한다. 관측 없이 시작하는 진입이 없다.
- **계획은 기존 준비(prepare) 경로로**: 제출 → `prepare_request(job="known_pallet_plan")` 가 `plan_transport` 작업을 내고(`observe_succeeded` 와 같은 인자: `start_rear` = 지금 자세, `target_pickup` = 운용자 자세, 격자 인자 등), 유효한 답에서만 채택한다. 작업자 오류·계획 실패·보정 변경·대기 중 STOP·취소·옛 세대의 늦은 답은 지금 준비 경로의 처리 그대로(채택하지 않음). 그동안 차는 선다.
- **채택** `start_known_pallet_mission(t, plan, inputs)`(= `start_mission` 의 짝): baseline 복원, 입력을 옵션에(`scenario`·`wait_pose`·`pickup_bounds`·`pickup_zone`, `adoption_zone` 없음, 관측 waypoint 없음), `ms.lift_command` = 측정 승강, 다리 설치, `commands.adopt()`, `idle → approach` 전이(러너가 `ms.phase = "idle"` 로 초기화해 둠).
- **설치 공유**: `_observe_install` 의 앞부분(경로·추종기 설치, 추종기 설정 기록, 경로 기록·저장·표시)을 `_install_mission_plan(plans, trace, start)` 로 **그대로 옮기고** `_observe_install` 이 그것을 부른다(게이트로 확인). 알려진 시작은 이것만 부르고, 관측 뒤처리(최종 직선 분할, `perception_error`, `near_leg_or_recapture`, `observe` 추종기·포획 기록)는 부르지 않는다.
- **팔레트 자세**: 운용자 입력은 **지도 고정 자세**이고 출처는 `operator_input` 이다.
  - `hooks.planning_target`(Isaac 의 oracle 선택)은 부르지 않는다.
  - 기록: `state["planning_target_source"] = "operator_input"`, `state["perception"]["perception_pickup_estimate_m"]` 에 같은 값과 `source: operator_input, not recognition`. `pocket["planning_pickup"]` 도 같은 값.
  - SLAM hold 의 분류·`known_pallet()` 사각형·적재 때 남기는 `last_pose`·`refresh_targets()` 의 대상 기록은 '그 보정에서 촬영한 추정'으로 다루지 않는다: 지도 고정으로 두고(보정이 바뀌어도 움직이지 않음), 대상 기록의 출처는 `recognised` 가 아니라 `operator_input`.
- **거부**: 근접 추적 켜짐, 밑판 검사 켜짐, SLAM 없음·무효, 작업자 없음 — 이유를 남기고 시작하지 않는다.

## 2. 포켓 검사(D5)의 첫 깊이 프레임

- 지금 `build_pocket_check()` 는 관측 포획(`pocket["capture"]`)의 깊이·내외부 보정·취득 시각·자세를 쓴다. 알려진 시작에는 그 포획이 없다.
- 계약: 알려진 시작은 접근 끝에 서서 **새 정지 깊이 프레임**(훅 `known_pocket_frame()` → 깊이, 내부 보정, 외부 변환, 취득 시각, 자세)을 받아 그것으로 포켓 검사를 만든다. 프레임이 없거나 오래됐으면 삽입에 들어가지 않는다. 새 프레임이 온 뒤에만 허가.
- 실차의 그 훅은 PR 7(D435i 실입력)이다. 4a 는 가짜 훅으로 시험한다. 포켓 검사를 끈 구성은 이 PR 의 실차 경로에 두지 않는다(총괄: D5 끔 우회는 만들지 않음). 끈 구성은 가짜 포트 시험에서만.

## 3. S10 들고 멈춤 — 옵션 `s10_stop_at_lift`(기본 꺼짐)

- 지금 실차 분기는 운용자의 들림 확인(`_operator_gate(..., "lifted")`)에서 `transition("extract")` 를 부르고, 그 전이가 화물 관계(`lift → extract` 에서 `carrying/confirmed`)와 장애물 기억 삭제를 함께 한다. `lift_and_timers()` 가 `True` 를 돌려주면 라이브 러너는 배달 판정으로 간다.
- 옵션이 켜지면 확인 틱에서 전이 대신 `_s10_end(t)`:
  - 화물을 `carrying/confirmed` 로(새 `CargoState.confirm_lifted()` — 전이표와 같은 값). `extract` 의 장애물 기억 삭제는 하지 않는다.
  - `state["s10_end"]`(미션 소유 키 — 종료 기록으로 넘어가고 다음 미션에서 지워짐)에 시각·승강 측정·팔레트 자세 출처.
  - **그 틱에서 제어 정지**로 처리한다: 코어가 `s10_stop` 신호를 내고, 실차 루프가 그 틱의 최종 제출 전에 제어 정지(제어 틱 계약 2절 ①: 래치 → 옛 DRIVE 폐기 → 바퀴 0·측정 승강 위치의 정지 DRIVE)를 한다. `terminate()` 만으로는 그 틱의 바퀴 0 이 보장되지 않는다(현장 복구의 즉시 0 은 `end_reset()` 뒤에야 열림).
  - 그다음 `commands.terminate("recovery_required", t, "s10_lifted")` 를 **배달 판정 없이 한 번** 수거한다(`lift_and_timers()` 는 `False`).
  - 후속 AUTO·텔레옵은 4d-2b 현장 복구와 새 세대 재위치 확인까지 잠긴다 — 아래 4절.
- 거절·시한 초과는 지금 규칙 그대로(`Pallet was not lifted`, `Lift did not reach its position`).
- 콘솔: 결과 사유에 "운반 높이 도달 · 현장 복구 필요"(대기열 정지는 그대로), 현장 복구 안내는 실차 복구 상태를 따른다.

## 4. 재위치 확인 전 잠금 (기존 빈틈)

- `field_recovery_done()` 은 복구를 마치고 새 재위치 세대를 두며, 그때부터 AUTO 제출은 `relocalize_required` 로 거부된다. 그러나 지금 `enforce_real()` 과 실차 텔레옵 정책은 그 키를 읽지 않아, 정상 피드백·정지 조건에서 주행과 승강이 다시 열린다(Codex 가 메모리에서 재현).
- 4a 는 코어가 `relocalize_required`(현재 세대의 확인 전)를 노출하고, 실차 텔레옵 정책(`real_teleop`)과 실차 AUTO 허가가 그것을 읽어 **주행·승강을 모두 거부**하게 한다. 옛 세대의 확인 응답은 풀지 못한다.

## 5. 시험 (가짜 훅, Isaac 없음)

- 시작: 준비 경로로 계획 요청 → 유효한 답에서 채택 → `approach` 로, 다리 설치; 기록에 `operator_input`; `planning_target` 훅·포획 호출 0 회; SLAM 을 켠 채 관측 키와 `observe` 추종기 없이 채택; 거부 조건 각각.
- 준비 실패들: 작업자 오류, 계획 실패, 보정 변경, 대기 중 STOP·취소, 옛 세대의 늦은 답 — 채택 없음.
- 지도 고정: 채택 뒤 보정이 바뀌어도 대상 자세·출처가 그대로, `refresh_targets()` 출처가 `operator_input`.
- 포켓 검사 켬: 깊이 프레임 없음·오래됨 → 삽입 진입 없음, 새 프레임 뒤 허가.
- S10: 직전 비영 출력이 있는 상태에서 확인 틱과 다음 틱의 최종 바퀴 0·옛 DRIVE 폐기·측정 승강 위치 정지 프레임(가짜 포트, 4c 루프와 함께 — 4a 는 코어의 `s10_stop` 신호까지), `extract` 전이 없음, 화물 `carrying/confirmed`, 장애물 기억 그대로, `recovery_required/s10_lifted` 한 번·배달 판정 없음, `s10_end` 가 미션 소유 키, 옵션이 꺼지면 지금과 같음.
- 재위치 잠금: 복구 전·복구 직후·옛 세대 응답·현재 세대 확인에서 AUTO 와 텔레옵(주행·승강) 모두.
- 재생 게이트 29/29(옵션 꺼짐 = 기록과 같은 호출, `_install_mission_plan` 옮기기 포함), 전체 pytest.

## 6. 자체 검토

- Codex 1 회차: P1 2(S10 확인 틱의 바퀴 0 을 `terminate` 가 보장하지 않음 → 그 틱 제어 정지 + 배달 판정 없이 한 번 수거; 재위치 확인 전 텔레옵이 열림 → 4절), P2 4(설치와 관측 뒤처리 분리 → `_install_mission_plan`; 준비 경로로 계획·실패 처리; 지도 고정·출처; 포켓 검사의 첫 깊이 프레임 계약), P3 1(콘솔 표시·`s10_end` 소유). 반영.
