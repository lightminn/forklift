# CT-1 PR ④ — 콘솔 대기열 정책으로 연속 실행 (2026-10-24)

계획: `docs/plans/2026-10-22-continuous-transport-ct1.md` 3 절 PR ④. 대기열 정책: `docs/plans/2026-10-22-console-mission-queue.md` D3–D6(`console/web/js/queue.js`). 브랜치 `feat/ct1-pr4-queue-run`.

## 바꾼 것

- `tools/mission_queue.py`: `console/web/js/queue.js` 의 `MissionQueue`·`newItem`·`resolveItem` 을 줄 단위로 옮겼다. 멈춤 문구도 콘솔과 같다.
  - 무응답 2 s, 피드백 상실 0.5 s(다음 상태가 회복이라고 해도 한 번 세면 멈춤), 결과 뒤 1 s 안정 대기를 그대로 옮겼다. STOPPED·TELEOP·안전 선점, 우리 것이 아니거나 일시정지·취소 중인 미션, lease 상실, 새 세션에서도 멈춘다.
  - 답이 불확실한 거부(`busy`·`stale`)와 `forget` 처리, 250 ms 타이머, 다음 제출 조건(PAUSED·`stopped_confirmed`·피드백 유효·결과 뒤 1 s)도 같다.
- 탐침 `tools/live_console_probe.py` 에 `--scenario queue` 를 더했다.
  - `--cycle` 의 다리를 대기열에 넣고 한 번 시작한다. 250 ms 마다 대기열이 내라고 할 때만 제출한다.
  - 상태·결과·연결 끊김은 콘솔의 queuewire 와 같은 순서(상태 → lease → 결과)로 대기열에 넣는다.
  - lease 는 protocol.js 와 같다. 상태의 보유자가 이 클라이언트가 아니면 잃는다.
  - 답이 없는 요청은 같은 메시지를 0.5 s 마다 다시 보낸다. 보내기 전마다 받은 답을 먼저 확인하고, `forget` 이 오면 재전송을 끝낸다.
  - 앞 회차의 `cycle --stop-on-anomaly` 는 대기열 정책을 근사한 것이라 거뒀다. 그 회차(`q_b9833dc`)의 결과는 이 문서의 증거로 쓰지 않는다.
- 탐침의 `request_until_standing` 은 `session_end` 에 한해 `stale_control` 도 다시 묻는다(아래 한계 참고).
- 시험
  - `tests/unit/tools/test_mission_queue_port.py`: Node 로 queue.js 를 직접 돌려 같은 입력의 궤적을 비교한다(13 경우 + 단독 1). Node 가 없으면 건너뛴다.
  - `tests/unit/tools/test_probe_queue_scenario.py`: 실제 `Probe._read`·`request_resent`·`queue_scenario` 를 가짜 웹소켓 차량과 가짜 시계로 돌린다(12 경우: 모두 완료, 실패, 거부, 미션 사이 TELEOP, 회복된 피드백 상실, lease 보유자 바뀜, 늦은 답 재전송, 답 대기 중 STOP, ack → STOP → done 순서, 재전송 경계의 답, 세대가 바뀐 `session_end` 재시도, `rearm` 은 재시도 안 함).
- ws1 실행 스크립트 `test_runs/live/ct1_queue.sbatch`(저장소 밖)다. `SCENARIO=queue`, `CYCLE=` 로 순서를 준다. 탐침이 끝나면 120 s 기다린 뒤 실행기를 끝낸다.

## Isaac 실행

- 스냅숏 `q2_0136fc5`(이 브랜치 0136fc5)다. 차량 쪽 코드는 main 4d762c6(#112 장면 재계획 장치, #113 포함)과 같다. #116(P6 허가 정지)·#117(최종 목표 크리프 정체)은 들어 있지 않다.
- 설정은 #107·#112 와 같다. 학습 원거리 인식, SLAM, D7 도킹, PR #88 근접 옵션, live 배율 목표 0.4.
- 장면은 `scenes_ct1_a770031/{line,random}_m_session.json` 이다. 순서는 일자 P-1:D-1 → P-2:D-2 → P-3:D-3, 랜덤 P-4:D-1 → P-5:D-2 → P-3:D-3 이다.
- seed 0–4 × 두 장면 = 10 실행이다. 같은 시간대에 돌도록 seed 마다 두 장면을 번갈아 냈다(ws1 잡 10190–10199, 동시 4 개).
- 실행 폴더는 ws1 `test_runs/live/q2_0136fc5_<장면>_s<seed>` 다. 요약 스크립트는 `q_report.py`(대기열)·`q3_detail.py`(미션 기록)다.
  - 실행기가 끝나며 쓰는 실행 단위 `result.json` 은 3 실행에서 없다(line_m s1·s4, random_m s0). 세 실행 모두 탐침의 `session_end` 요청이 `stale_control` 로 거절됐다(`close_code` 없음). 탐침은 그 뒤 약 120 s 상태를 더 받다가 끝났고, 실행 스크립트가 다시 120 s 기다린 뒤 실행기를 SIGTERM 으로 끝냈다. 정확한 SIGTERM 시각은 확인하지 않았다.
  - 미션 결과·금지 접촉·삽입 기록은 미션마다 남는 `missions/m*/` 에서 읽었다. 누락된 세 실행도 마지막 미션 파일이 온전하고 탐침의 결과와 맞는다(Codex 확인). 그래서 미션 집계는 잘리지 않았다.

| 장면 | seed | 미션 결과 | 대기열 멈춤 | 하역 오차 |
|---|---|---|---|---|
| line_m | 0 | done · done · done | 완료 | 1.6 · 0.5 · 1.3 cm |
| line_m | 1 | done · done · done | 완료 | 2.7 · 1.7 · 1.6 cm |
| line_m | 2 | done · `transport_recovery_after_docking` | 실패 | 3.0 cm |
| line_m | 3 | done · `Excessive body tilt in lift` | 실패 | 1.6 cm |
| line_m | 4 | done · done · done | 완료 | 2.5 · 0.4 · 1.5 cm |
| random_m | 0 | `observe:stall_exhausted` | 실패 | — |
| random_m | 1 | done · done · done | 완료 | 2.9 · 1.0 · 3.1 cm |
| random_m | 2 | done · done · done | 완료 | 2.3 · 2.2 · 0.7 cm |
| random_m | 3 | `observe:stall_exhausted` | 실패 | — |
| random_m | 4 | (제출 없음) | `상태 수신 끊김` | — |

- **연속 완주(3/3) 세션: 10 실행 중 5**(line_m s0·s1·s4, random_m s1·s2). 미션을 낸 9 실행으로 보면 9 중 5 다.
- 제출한 미션 21 개 중 완료 17 이다(line_m 11/13, random_m 6/8).
- 완료 미션의 하역 오차는 0.4–3.1 cm 다.
- 금지 포켓 접촉은 21 미션 모두 0 이다.
- 실행 단위 기록이 있는 7 실행에서 대상이 아닌 팔레트의 최대 밀림은 약 0.01 mm · 0.00037 rad 다(장면 손상 문턱 1 cm · 0.01 rad).
- 실시간 배율(시뮬 / 벽 시간)은 미션을 내고 실행 단위 기록도 있는 6 실행에서 0.31–0.36 이다. random_m s4 는 시작 직후 끝나 0.02 다.
- 대기열 정책: 실패 4 실행은 모두 실패 결과 뒤 다음 미션을 내지 않았다. 미션 중 피드백 상실·STOPPED·선점·lease 상실로 멈춘 일은 없다. 답이 불확실한 항목도 없다.
  - 결과 뒤 다음 제출까지 12 회의 대기는 1.001–1.504 s 였다. 제출 직전 상태는 모두 PAUSED 였고, 활성 미션은 없었다(Codex 확인).
  - 탐침이 남긴 상태에는 `feedback_ok`·`stopped_confirmed`·lease 가 없다. 이 조건들은 실행 기록으로 따로 감사하지 않았다. 시험(`test_probe_queue_scenario.py`)으로만 확인했다.

### random_m s4 — 시작 직후 무응답 멈춤

- 대기열이 시작한 직후 상태 간격이 2.42 s 벌어졌다(벽 시계 기준, 시뮬 0.50 → 0.51 s). 대기열의 무응답 2 s 문턱을 넘어 `상태 수신 끊김` 으로 멈췄다. 미션은 하나도 내지 않았다.
- 그 틱의 렌더 한 번이 2.40 s 걸렸다(`step.render`). 왜 그만큼 걸렸는지는 기록으로 가리지 못했다.
- queue.js 와 같은 동작이다. 콘솔도 같은 상황에서 멈춘다. 다만 사람이 시작 버튼을 누르는 시점은 차량이 상태를 안정되게 보낸 뒤일 가능성이 크다. 탐침은 첫 상태와 lease 직후에 시작했다.
- 미션 중 2 s 넘는 무응답은 나머지 9 실행 어디에도 없었다.

## 실패 분류 (원인 확정 아님)

| 실행·미션 | 단계 | 양상 | 관련 |
|---|---|---|---|
| random_m s0 m1, s3 m1 | 첫 관측(P-4) | 후보 0–4 의 관측 5 회가 모두 팔레트를 찾지 못했다(`perception_no_pallet:unresolved_in_radius`·`uncertain_in_radius`). 마지막 후보 5 로 가는 길에 cusp 에서 정체했다. 남은 후보가 없어 끝났다(`scene_replans` 는 `candidate_index` 6 의 `exhausted` 하나). 그 사이 관측 재계획 2 회가 섞여 있다 | 원거리 인식(P-4). cusp 정체는 마지막 후보에서만 결정적이었다 |
| line_m s2 m2 | 운반(도킹 뒤) | D7 도킹 재시도 2 회 뒤 `transport_recovery_after_docking` | D7 도킹 뒤 복구 |
| line_m s3 m2 | 들기(P-2) | 포크가 293–296 mm 들어가 팔레트 중심을 4–7 mm 못 넘었다. 들기 중 팔레트 기울기 0.151 rad. 삽입 끝 SLAM 오차는 진행 방향 +4.25 cm 였다 | 삽입 깊이 규칙(ADR 0004 D3) |

- 삽입 깊이(19 미션): 이 한 건만 중심을 넘지 못했다. 나머지 18 건은 중심을 5.8–30.5 mm 넘었다. 목표 330 mm 에 중심이 300 mm 라, 설계 여유 30 mm 중 10 mm 안쪽으로 남은 미션이 3 건 더 있다(5.8–9.9 mm).
- 앞 회차(`q_b9833dc`, 같은 차량 코드, 대기열 근사)에서도 같은 부류가 나왔다. line_m s3 들기 기울기, line_m s2 도킹 뒤 복구, random_m 첫 관측 실패다. Isaac 은 같은 seed 에서도 결정적이지 않아, seed 와 부류의 관계는 이 표본으로 말할 수 없다.
- 이번 회차에는 남은 거리 0 최종 목표 정지(관측·복귀)가 나오지 않았다. 앞 회차의 두 건은 이 세션이 따로 맡는다.

## 한계

- 한 버전만 돌렸다. 비교 실험이 아니다. Isaac 은 같은 코드·seed 에서도 결정적이지 않아, 이 완주율은 이 시간대의 표본이다.
- 탐침의 대기열은 queue.js 의 이식판이다. 같은 입력에서 같은 궤적인지는 Node 비교 시험으로 확인했다. 브라우저·화면 동작(버튼, 표시)은 이번 범위 밖이다.
- 실행 단위 기록이 없는 3 실행은 장면 손상(밀림)과 배율을 보지 못했다. 원인은 탐침의 `session_end` 가 `stale_control` 로 거절된 것이다. 기록이 보여 주는 것은 그 거절까지다. 마지막 정지에서 제어 세대가 바뀌고 탐침이 그 세대를 담은 상태보다 먼저 요청을 보냈다고 추정하지만, 세 실행의 상태 기록에는 `control_gen` 이 남지 않아 순서를 확인하지 못했다. `request_until_standing` 은 `not_stopped`·`no_feedback:rear_axle_speed` 만 다시 물었다. 이 PR 에서 `session_end` 에 한해 `stale_control` 도 다시 묻게 고쳤다. `rearm`·`mission_resume` 의 `stale_control` 은 그대로 호출자에게 돌려준다. 앞선 STOP 뒤에 준비한 요청이 다음 STOP 뒤에 통과하면 안 되기 때문이다. 시험 둘(가짜 차량에서 추정 순서를 재현해 `session_end` 가 끝나는 것, `rearm` 은 한 번만 묻고 거절을 돌려주는 것)이 고치기 전·범위를 넓힌 코드에서 각각 실패함을 확인했다. Isaac 으로 다시 돌리지는 않았다. 이 재시도로 탐침의 `--end-session` 은 STOP 뒤에도 종료 의도를 유지한다(STOPPED·비무장은 그대로이고, 종료만 요청한다). 브라우저의 종료 확인은 `control_gen` 이 바뀌면 취소되는 별개 동작(`results.js` ConfirmGate)이라, 종료 버튼도 같다는 근거로 쓰지 않는다.

## 다음

- 8 주차 목표 판정은 총괄 세션이 이 표로 한다.
- 이 결과로 올릴 후보:
  1. random_m P-4 의 원거리 인식. 두 실행 모두 다섯 후보에서 팔레트를 찾지 못했다. 앞 회차에도 P-4 첫 관측 실패가 세 번 있었다(그중 하나는 모든 후보 `perception_no_pallet`).
  2. 삽입이 중심을 넘지 못한 들기 기울기. 삽입 끝 진행 방향 SLAM 오차와 함께 본다.
  3. D7 도킹 뒤 복구.
  4. 시작 직후 실행기 멈춤과 대기열 무응답 문턱. 탐침의 시작 시점을 사람처럼 늦출지 정한다.

## 검토

- Codex 적대적 검토(원자료 재계산). 1 차: P2 2 건(실행 단위 기록 개수·배율 범위, random_m P-4 실패의 후보 전환 순서)을 반영했다. 2 차: P2 1 건(`stale_control` 재시도가 `rearm`·`mission_resume` 에도 걸림)을 `session_end` 한정으로 좁혀 반영했다. 3 차: 새 P1/P2 없음(수렴).
- Codex 가 원자료로 다시 맞춘 것: 5/10 세션 완주, 17/21 미션 완료, 금지 접촉 0, 하역 오차 0.4–3.1 cm, 삽입 19 건의 중심 여유, 실패 뒤 추가 제출 없음, 결과 뒤 제출 대기 1.001–1.504 s.
