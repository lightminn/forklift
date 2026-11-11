# 지연 내성 시험 A(결정 배치) 결과 (2026-11-11)

등록: `docs/plans/2026-11-11-latency-tolerance.md`(7판, #430; 구현 개정 #431). 판정 도구: `2026-11-11-latency/lat_analyze.py`. 실행별 요약: `2026-11-11-latency/lat_a_summary.json`(원본 판정 출력 `lat_an3.json` 은 ws1 `test_runs/lat/a_8362b5d3/`, 판정 도구 8750960e).

## 실행

- 스냅숏 `lat_8362b5d3` = main 8362b5d3(#431 머지). riib_scene `--deterministic --lidar-cast loop`, VIDEO off, `RETURN_START=boxed`, `NEAR_END_STAND=on`, G3 커널 변수 없음, 흔들림 seed = 장면 seed.
- 묶음 잡 17886·17887(riib 3 슬롯 × GPU 2, `--retries 0`, `--run-timeout-s 4200`), 09:37–11:30, 80 실행.
- 실행 하나의 벽시계는 148–1,558 s(중앙 398 s)였다. 계획 5 절의 추정(0.5–1 h)보다 짧았다.
- 다른 사용자 잡과 겹친 시간이 있었지만 A 의 판정은 시뮬 시간 기준이라 무효 조건이 아니다(4.7).

## 유효성

- 80 실행 모두 유효했다. 묶음 시도 1 회·종료, 필요 파일 있음, `deterministic.broken` 없음. 재제출은 없다.
- 등가 대조(2.3, `deterministic_compare --ignore-injection`, run 폴더):
  - S0 = B0: line_2·random_1 seed 0(본 배치), aisle_rack seed 0(구현 PR 스모크 `smoke_84f18175`) 모두 EQUAL.
  - L0 = B0: line_2·random_1 seed 0 모두 EQUAL.
- 결정성 반복: B0 와 반복은 두 장면 모두 EQUAL 이다. C6 와 반복은 아래 예외를 적용해 EQUAL 이다.

### 결과 뒤 예외 1: 결정성 비교에서 층 작업자 측정 필드 제외(총괄 승인 2026-11-11)

- **예외 전 판정 상태:** 등록대로 비교하면 B0 반복은 EQUAL 이었다. C6 반복은 두 장면 모두 DIFFERENT 였는데, 차이는 `layer_worker.json` 과 `result.json` 의 `latency_injection.layer.pending_at_end` 안의 측정 필드뿐이었다. 등록 4.7 에 따라 두 장면의 지연 효과 판정을 멈추고 총괄에 올렸다.
- **무엇이 왜 달랐나:** 층 작업자가 작업마다 남기는 CPU·벽시계 측정값이다. 1e 단계 1 에서 넣은 기록으로, 실행마다 당연히 다르다. 목록: `compute_s`, `compute_cpu_s`, `run_ns`, `run_delay_ns`, `cpu`, `submit_t`, `done_t`, `install_start_t`, `install_s`, `wait_s`, `tick_t`. 이 목록을 그 두 곳에서만 빼면 C6 반복도 두 장면 모두 EQUAL 이다. 목록 밖 필드의 차이는 없었다.
- **판정 경로가 읽지 않는다는 근거(조건 2):**
  - grep 범위: `sim/isaac/run_transport.py`, `layer_worker.py`, `obstacle_layer.py`, `latency_injection.py`, `src/forklift_core/mission/core.py`, `src/forklift_core/perception/`, `src/forklift_core/control/`. 키를 문자열과 속성 형태로 모두 찾았다.
  - 이 필드들은 `layer_worker.py` 의 기록 자리에서만 쓰인다(`_run` 224–231, `submit` 144, `install_now` 의 `info.update`, live 의 `tick_start` 174).
  - `wait_s` 의 다른 일치는 `cusp_stall.py` 의 설정값(`stall.wait_s`)과 live prep 시험 인자로, 이름만 같고 이 기록이 아니다.
  - 설치 콜백(`run_transport.py` 4833–4847)이 읽는 키는 `record`·`stamp_s`·`scan_tick`·`install_tick`·`reason`·`correction`·`version`·`writes` 뿐이다.
  - 결정 모드의 층 지연은 물리 틱으로만 정해진다(`due_tick = scan_tick + k`, `step_world` 시작의 `install_due(tick)`). delay 모드는 계산을 `submit` 안에서 끝내므로, `install_now` 의 벽시계 대기(`_done.wait(timeout_s)`)는 바로 반환한다. 벽시계 값은 동작에 들어가지 않는다.
- **적용:** `lat_analyze.py --determinism-without-layer-timing`. 필드 이름을 명시한 목록이고, 와일드카드나 파일째 제외는 쓰지 않는다.

### 결과 뒤 변경 2: 판정 도구의 결함 규칙 수정(두 가지)

- (a) 기준선이 실패한 seed 에서 지연 팔이 임무를 마치면 분류가 '비교 없는 허용'(`tolerated_no_operating_comparison`)이 된다. 수정 전 도구는 이것을 결함으로 올렸다. 임무를 마친 실행은 결함이 아니라서 뺐다.
- (b) 우선순위를 등록 4.4 문구대로 맞췄다(Codex 결과 대조 P2).
  - 50 ms 이하 칸(S3·S6·U3·U6·C6): 관측 종료 2, 실패 3, 운영 저하 3.5.
  - 그 뒤 100 ms·흔들림 팔(S12·U12·D12·J): 관측 종료 4, 실패 5.
  - 100 ms·흔들림 팔의 운영 저하는 등록상 결함 목록이 아니라 보고만 한다.
  - 수정 전 도구는 관측 종료를 팔과 관계없이 2 로, 100 ms 팔의 운영 저하를 4.5 결함으로 올렸다.

| 실행 | 수정 전 | 수정 후 |
|---|---|---|
| `lat_S12_aisle_rack_s1`(기준선 S0 seed 1 실패, S12 완주) | 결함 4 | 결함 아님(a) |
| random_1 D12 s0·s2, random_1 S12 s1(운영 저하) | 결함 4.5 | 보고만(b) |
| 100 ms·흔들림 팔의 관측 종료 16 건 | 결함 2 | 결함 4(b) |

- 결함 여부가 바뀐 것은 위 넷이고, 나머지는 우선순위만 바뀌었다.

## 결과

안전 위반(4.1)은 80 실행 모두 0 이다. 포켓 금지 접촉 0, 허가 없는 진입 0, 낮은 물체 물리 겹침·정지 호 위반 0 이고, 물리 최근접 최소는 0.129 m 다.

칸 분류(4.4), seed 0 / 1 / 2:

| 장면 | 팔 | 분류 |
|---|---|---|
| line_2 | S3 | 허용 / 허용 / 허용 |
| line_2 | S6 | 운영 저하 / 허용 / 허용 |
| line_2 | S12 | 허용 / 허용 / 허용 |
| line_2 | U3·U6·U12 | 모두 관측 종료 |
| line_2 | D12 | 허용 / 허용 / 허용 |
| line_2 | C6·J | 모두 관측 종료 |
| random_1 | S3 | 허용 / 허용 / 허용 |
| random_1 | S6 | 운영 저하 / 안전 실패 / 허용 |
| random_1 | S12 | 관측 종료 / 운영 저하 / 관측 종료 |
| random_1 | U3·U6·U12 | 모두 관측 종료 |
| random_1 | D12 | 운영 저하 / 관측 종료 / 운영 저하 |
| random_1 | C6·J | 모두 관측 종료 |
| aisle_rack | S12 | 관측 종료\* / 허용(비교 없음)\* / 허용 |
| aisle_rack | U12·C6 | 관측 종료\* / 관측 종료\* / 관측 종료 |

\* 기준선 S0 이 실패한 seed(0, 1)다. 등록 3.1 의 예외에 따라 임무 결과는 지연 결함으로 세지 않고 안전만 읽는다.

- 운영 저하 다섯 건은 모두 하역 오차가 허용 폭(기준선 + 1 cm)을 넘은 경우다. 임무 시간과 도킹 라운드는 폭 안이었다.
  - line_2 S6 s0: 3.35 cm(한도 2.39). random_1 S6 s0: 1.77(1.45). random_1 S12 s1: 6.19(1.32). random_1 D12 s0: 3.30(1.45). random_1 D12 s2: 2.04(1.50).
- 실패 사유:
  - 비알림 층(U3·U6·U12)과 C6·J: 모두 추적 시간 초과(observe·return_home·transport)다.
    - 증거 나이 감속(`evidence_stale`)이 실행마다 375–9,214 제어 틱 기록됐다. random_1 U3 375–429, line_2 U12 5,566–9,214 다.
    - 같은 장면·seed 의 B0·S0 와 알림 층 D12 는 0 틱이다.
    - 끝날 때 차가 움직이고 있어(예: U6 line_2 s0 −0.15 m/s) 정지가 확인되지 않았다.
  - SLAM S12 random_1: s0 `near_capture_misaligned`, s2 운반 중 추적 실패(pos 3.33 m).
  - S6 random_1 s1: `observe_replan_failed:invalid_start`. 끝까지 6.9 s 서 있었다(안전 정지 확인).
  - D12 random_1 s1: `near_capture_misaligned`.
- 노출(4.5)은 모든 지연 팔에서 충족했다.
  - SLAM 팔(지연 팔만): 이동 중 holding 밖 지연 답 233–1,900.
  - 층 팔: 이동 중 제출 작업 207–2,508, 예정 설치 비율 0.95–1.00.

## 결론(등록 결론 규칙)

- **위험(P1):** 없다.
- **50 ms 이하 칸의 관측 종료(우선순위 2):** 19 건.
  - U3·U6 × line_2·random_1 × 3 seed = 12.
  - C6 × line_2·random_1 × 3 = 6, aisle_rack C6 s2 = 1.
- **50 ms 이하 칸의 실패(3):** random_1 S6 s1(안전 실패).
- **50 ms 이하 칸의 운영 저하(3.5):** line_2 S6 s0, random_1 S6 s0.
- **100 ms·흔들림 팔의 관측 종료(4):** 16 건.
  - U12 × line_2·random_1 × 3 = 6, aisle_rack U12 s2 = 1.
  - J × line_2·random_1 × 3 = 6.
  - random_1 S12 s0·s2 = 2, random_1 D12 s1 = 1.
- **보고만(100 ms 팔의 운영 저하):** random_1 D12 s0·s2, random_1 S12 s1.
- **반드시 허용이어야 하는 칸(S3·S6·U3·U6·C6):**
  - S3 은 두 장면 모두 6/6 허용이다.
  - S6 은 운영 저하 2 와 안전 실패 1 이 있다.
  - U3·U6·C6 은 전부 관측 종료다.
  - 결함으로 진단할 대상이다.
- **D12 / U12(4.4):** 여섯 장면·seed 모두 '정지 안전성 미확인'이다. U12 가 관측 종료여서 "지연을 알면 견디고 모르면 선다"로는 쓸 수 없다.
  - line_2 에서 D12 는 3/3 허용이고 U12 는 3/3 관측 종료다. 관측된 것은 알린 쪽은 증거 나이 감속 0 틱으로 완주하고, 알리지 않은 쪽은 감속이 쌓인 채 추적 시간 초과로 끝났다는 것이다. 감속이 시간 초과의 원인이라는 것은 잠정 해석이다(원인 진단 안 함).
- **해석(판정 밖):**
  - SLAM 답 지연은 25 ms 까지 두 장면 모두 견뎠다. 50·100 ms 에서 random_1 의 근접 캡처·운반·관측 재계획과 하역 오차가 흔들렸다.
  - 층 설치 지연은 정지 모델과 사용 나이에 알리지 않으면 25 ms 부터 임무가 실패했다.
  - 증거 나이 감속이 비알림 팔에만 나왔다는 점은 1e-0 과 같은 기전(사용 나이가 설치 지연을 덮지 못함)과 맞는다. 다만 원인 진단은 하지 않았으므로 잠정 해석이다.
  - 계획 4.4 가 예로 든 고칠 방향은 실제 설치 지연을 재서 사용 나이에 반영하는 것(측정 기반 알림)이다. 결정은 총괄 몫이다.

## aisle_rack: 기준선보다 앞선 단계의 다른 실패(등록 3.1, 따로 기록)

- 기준선이 실패한 seed 에서 지연 팔이 기준선보다 앞선 단계에서 다른 사유로 실패한 경우다. 지연 결함으로 세지는 않는다(3.1).

| seed | 기준선 S0 실패 | 지연 팔 실패 |
|---|---|---|
| 0 | return_home, `scene_replan_failed:cusp_handed:no_path` | C6·U12: observe 추적 시간 초과 / S12: transport 추적 실패 |
| 1 | transport 추적 실패 | C6·U12: observe 추적 시간 초과 |

## aisle_rack 기준선 실패(지연 0, 따로 기록)

- S0(지연 0) seed 0–2 가운데 2 건이 실패했다.
  - **seed 0:** 집 복귀(return_home) 중 `scene_replan_failed:cusp_handed:no_path`. 시뮬 약 286 s, 후축 자세 (2.23, −11.21, −0.57 rad). 끝날 때 0.5 s 넘게 서 있었다(안전 정지 확인). 구현 PR 스모크의 B0·S0 seed 0 도 같은 실패였다(결정적 재현).
  - **seed 1:** 운반(transport) 중 추적 실패(pos 0.011 m, yaw 0.058 rad). 운반 시작 81.9 s, 종료 시뮬 약 295 s, 자세 (0.58, −10.75, −0.28). 정지는 확인되지 않았다.
  - **seed 2:** 완주(343 s).
- 총괄 결정(2026-11-11): 지금은 진단을 배정하지 않는다. 지연 판정은 seed 별 기준선 대비로만 읽는다.

## 계획과 다른 점

- 4.1 의 '공칭 지연을 더한 보수적 민감도'는 낮은 물체 보고서로만 계산했다. 참값 평가기는 오프라인으로 다시 돌릴 수 없어서다. 보고 전용이라 판정에 쓰지 않는다(총괄 수용 2026-11-11).
- 결과 뒤 예외 1, 결과 뒤 변경 2(위).

## 한계

- 계획 7 절 그대로다. 특히 다음이 있다.
  - 결정 배치의 틱 지연은 live 의 벽시계 지연을 대신한다.
  - 두 장면 + aisle 최악 수준, seed 셋이다.
  - holding 단계에서는 SLAM 지연이 제어 자세에 바로 닿지 않는다.
  - 결정 모드 온라인 매핑이라 정적 층이 계획에 들어가지 않는다.
- B(live 계획 지연)는 아직 내지 않았다. ws1 의 다른 사용자 잡이 끝난 뒤 단독으로 낸다.
