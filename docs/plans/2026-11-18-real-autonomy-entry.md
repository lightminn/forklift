# 실차 자율 실행 경로 — PR 나누기와 순서 (짧은 계획)

작성 2026-11-18 · 총괄 배정(사용자 "모두 해") · 상위 계획 [sim-to-real bring-up](2026-11-11-sim-to-real-bringup.md) P0-4 ⑤·P0-9·S10, 설계 [4a'](../design/2026-11-11-p04a-kinematics-and-command.md)(4e-2 필수 항목), [4e-1](../design/2026-11-11-p04e1-control-tick.md)

## 지금 있는 것 / 없는 것

- 있음: `MissionCore`(공통 미션 로직), `RealPlantPort`(`src/forklift_core/plant/port.py`: 명령·만료·`zero_wheels`·`hold_lift`·`sample`·`health`), `RealKinematics`·`RealOdometry`, `McuLinkHost`·`ReconnectingLink`, `fail_closed_hooks`(4d-2a), SafetySpeed·실차 방향 규칙(4d-1), 현장 복구 상태(4d-2b), 텔레옵 다리(#459)와 스캔 허가(#559), 실차 안전 감시(#557), 비상정지 면제(#584·#585), 4e-1a `tick_vars`.
- 없음: 실차 자율 진입점. 제어 틱은 아직 `sim/isaac/run_transport.py` 의 `drive()`(5588 행부터) 루프(10201 행) 안에 있고, 루프를 끝내는 `break` 가 11 곳이다. 공통 조립(MissionCore·층·SLAM 링크·콘솔)도 `drive()` 안이다.

## 원칙 (바꾸지 않음)

- Isaac 경로는 동작 보존: 호출 순서·횟수가 바뀌면 멈추고 보고. 러너·미션 코어·층을 건드리는 PR 은 재생 게이트 29/29 + ws1 전체 pytest, 머지 직전 main 병합 뒤 최종 트리.
- 50 Hz 벽시계 루프는 실차 진입점에만(따라잡지 않음). Isaac 은 120 Hz 1:1.
- 실차 진입점은 참값 경로를 조립하지 않고(P0-5 감사표), 훅은 `fail_closed_hooks` 로 감싼다.
- SLAM 이 없거나 무효면 정지, 자동 재개 없음. 사람 감시·저속·빈 포크, 승강은 운반 높이까지. V1.0 비상정지 면제 조건(#580·#584·#585) 유지.

## PR 순서

| # | PR | 내용 | 검증 |
|---|---|---|---|
| 1 | 4e-1b 제어 틱 함수 | `drive()` 루프의 제어 구간을 함수 몇 개로 뗀다(설계 4e-1 2절: 제어 전반 → 층 감시·새 장애물·estop probe(drive 에 남음) → low_clearance·명령 조립). `break` 11 곳의 반환 계약(출력 전 종료 / 정지 출력 뒤 종료)과 밖에서 쓰는 결과 목록을 정한다. **착수 전 반환 계약을 Codex 와 다시 상의**. 참값·Isaac 입력은 콜백 뒤로. | 게이트 29/29, 전체 pytest, 원문 고정 시험 갱신 |
| 2 | 틱 수 상수 → 시간 정의 | 설계 4e-1 3절 표의 120 틱 창(정체·dock·slam·observe·blocked·cusp_stall 등)을 `dt` 로 계산한 시간 정의로. Isaac 은 `dt = 1/120` 이라 값이 같다. 근접 추적 `DriveTerms`(120 Hz 시간축 표)는 건드리지 않고 실차에서는 끈다(아래 질문 2). | 게이트 29/29, 전체 pytest |
| 3 | 4e-1c 공통 조립 | MissionCore·장애물 층·SLAM 링크·콘솔 서버를 Isaac 없이 조립하는 함수. 훅은 호출자가 넘김. Isaac 드라이버 묶기(`IsaacPlant.command`, stand_hold·dwell). | 게이트, 전체 pytest, **Isaac live smoke 단독**, Isaac 없는 가짜 포트 기동 시험 |
| 4 | 4e-2a 실차 진입점 골격 | `tools/real_autonomy.py`: 50 Hz 벽시계 루프, `McuLinkHost`+`RealPlantPort` 로 DRIVE, STATE 마다 `RealOdometry`, health → 중재기 STOP, `fail_closed_hooks`, 래치 위 AUTO 게이트, 확인 넷(다리와 같음), 빈 포크·운반 높이 상한. SLAM 링크가 없거나 무효면 미션 시작 거부·진행 중 정지(자동 재개 없음). | 가짜 포트 단위·통합 시험(정상 미션 시작, SLAM 무효 → 정지, health 끊김 → STOP) |
| 5 | 4e-2b 안전 배선 | 무장 전 장애물 층 존재·`obstacle_act=True` 강제(실행 중 해제 금지), 실제 스캔을 층에(7b 경로, unknown 은 지우지 않음), `config/real/safety_watch.yaml` 감시, 콘솔 입력(lease·STOP·재무장·미션·운용자 확인), 재위치 배선(`field_recovery_done` → 재위치 세대), 포켓 구간 재개 조건(각 방향 답함, 그 전까지 삽입은 감독·저속·빈 포크). | 단위 시험 + 4 의 통합 시험 확장 |
| 6 | P0-9 HIL 루프 | 실행기 → MCU 모형(`firmware/tools/mcu_model.c`, pty) → STATE → 오도메트리가 닫힘. 링크 끊김·STATE 지연·스캔 끊김·SLAM 끊김 주입 → 입력 나이 상한 안에 정지. 스캔·SLAM 은 합성 원천(실 장치에서는 거부, 다리의 `--synthetic-permit` 과 같은 방식). | realtime 표시 통합 시험, ws1 |

- 1·2·3 은 Isaac 쪽 이동이라 차례로(서로 같은 파일). 4 는 3 의 공통 조립을 쓰고, 5·6 은 4 뒤.
- 11-19 Jetson `jetson_prep.sh`, 11-20 보드 확인일 지원이 오면 그것이 먼저다.
- 뒤로 미루는 것(하드웨어·실측 뒤): 4c-2(Isaac 센서 추출), 실차 SLAM 링크(ROS 컨테이너의 slam_toolbox, P0-3·P0-5a), 근접 추적의 50 Hz 표, S10 점검표.

## 총괄에게 물을 것

1. **실차 SLAM**: S10 은 시험장 지도가 먼저 필요하다(지도 먼저·위치 추정 기본, 2026-11-14 결정). 실차 지도 만들기(조사 주행)는 텔레옵으로 하는가? 4–6 은 합성 SLAM 원천으로 시험하고, 실 SLAM 링크는 A2M12·Jetson 뒤 별도 PR 로 둔다.
2. **근접 추적**: `DriveTerms` 표가 120 Hz 시간축이라, 실차 첫 자율(S10 사례 A, 정렬된 직진 삽입)은 근접 추적을 끈 채로 한다고 제안한다. 켜려면 50 Hz 재측정이 필요하다.

## 자체 검토

(검토 뒤 채움)
