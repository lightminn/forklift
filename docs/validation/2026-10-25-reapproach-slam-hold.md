# D7 재접근 동안 SLAM 보정 보류 검증 (H1 · E1)

계획: [재접근 보정 보류 2판](../plans/2026-10-25-reapproach-slam-hold.md). 브랜치 `feat/d7-slam-steps`.

## 결론

- **V5 확인 배치(계획 2a, 실행 전 등록)는 합격이다.** 이 처방의 마지막 판정이다.
  - 60 쌍, on 우위 26 · off 우위 6 · 동점 28. 단측 정확 부호 검정 p = 0.00027 이다.
  - 보조 기준은 모두 통과했다. 합계 점수는 off 121 → on 155 다. 전체 실패는 39 → 25, D7 뒤 실패는 9 → 3 이다. 거절은 0 / 0, on 팔 금지 접촉 0, 확인할 수 없는 접촉 0 이다.
- 그래서 장면 런처의 기본을 **켬**으로 바꾼다(`D7_HOLD=on`). 대상은 `deploy/console/isaac_live.sbatch`(CT-1·콘솔), `deploy/riibotics/riib_scene.sbatch`, 그리고 콘솔 runner 를 글자 그대로 따르는 `deploy/console/slam_survey.sbatch` 다. 러너(`run_transport.py`)의 기본은 off 그대로다.
- 아래 V4 는 앞선 판정이다. V4a 불합격 뒤 지표와 표본 크기를 새로 등록해 V5 를 돌렸다. 같은 자료로 다시 판정하지 않았다.

### V4 (앞선 판정)

- 사전 등록한 V4a 판정은 **불합격**이다. 쌍 검정이 p = 0.125 로 0.05 를 넘었다.
  - 나머지 기준(완료 미션 비열화, `transport_recovery_after_docking` 비증가, 거절 비증가, 쓰지 못한 쌍의 쏠림)은 모두 통과했다.
- 차가 0 이 아닌 쌍은 3 개뿐이었다(s3 · s6 · s8). 3 개 모두 보류 쪽이 나았고, 나쁜 쌍은 0 이다.
  - 관측된 0 아닌 쌍 3 개로 나올 수 있는 가장 작은 p 는 1/8 = 0.125 다.
  - 그렇다고 설계가 원래 통과할 수 없었던 것은 아니다. 같은 9 쌍에서 보류 우위 쌍이 5 개였다면 p = 0.03125 였다(Codex V4 판정 검토 P3).
  - 그래서 불합격을 검정력 부족만으로 설명하지 않는다. 효능이 입증됐다고 읽지도 않는다.
- 사전 기준 밖의 기술 통계(판정에 쓰지 않음)
  - 세 비교를 합친 재접근 시도는 main 42/48(정체 6), 보류 41/41 이다.
  - 이 합에는 세션이 끊긴 마지막 미션의 기록도 들어 있다. 세션이 `slam_link_failed` 로 끝난 실행 6 개(off 4, on 2)는 마지막 미션 폴더가 없어, 실행 단위 `result.json` 에서 읽었다(Codex V4 판정 검토 P2).
  - D7 재시도 매칭 거절로 끝난 미션은 main 3, 보류 0 이다.
  - 정체가 한 번이라도 있었던 세션은 main 5/20(random_m s3 · s6 · s8, 관절각 s1 · s3), 보류 0/20 이다.
- V4 때의 사전 규칙대로 그때는 런처 기본값을 off 로 두었다. 코드(`--d7-reapproach-hold`, 기본 off)는 넣었다.

## V1 · V3

- V1 단위 시험: `tests/unit/mission/test_core_reapproach_hold.py`(14), `test_core_docking_stall.py` +3, `test_core_blocked_replan.py` +1, `tests/unit/tools/test_leg_failures_runner.py` +1, `tests/unit/tools/test_leg_failures_pairs.py` +1.
- ws1 전체: 6,783 통과, 실패 10 은 알려진 ws1 전용(코드 `b27b0914`).
- 재생 게이트 runs29: 29/29(스냅숏 `lf_b27b0914`, 기본 off). 그 뒤 고친 E1 의 R2 이월은 off 에서 실행되지 않는 경로다.
- Codex: 계획 2 회차, 구현 2 회차에서 "새 P1/P2 없음"으로 수렴했다.

## V4 Isaac

- 팔: 같은 스냅숏 `lf_5d280e80` 에서 `--d7-reapproach-hold` off / on 만 다르다.
  - 런처 `ct1_queue.sbatch`, `SCENARIO=queue`, 고정 손 순서, `QUEUE_EXTRA` 없음.
  - 오도메트리: random_m · line_m 은 reported(런처 기본 전환 PR #137 전 스냅숏), tq_random_m 은 `ENCODER_MODEL=team_spec_quantized`.
- 실행 방식: 모두 live 4 세션 묶음이다(같은 목록, 같은 claim-dir, `--pace-floor 0.335 --pace-gap-p99-s 0.12`).
  - 11205(29 실행, 물러남으로 동시 1–2 세션까지 줄었다)와 11206(총괄이 22:00 에 취소, 7 실행)이 대부분을 돌았다.
  - 11206 취소로 끊긴 random_m s3 쌍과 남은 실행 11 개는 재제출 11365(정리 단계 `rh_redo_prestep.py`, `--pace-warmup-s 540`)가 돌렸다.
  - 끊긴 출력은 `*.killed11206` 으로 옮겼고 판정에서 뺐다.
- 보류 작동: 수정 팔 18 실행에서 `hold_reapproach` 41 회, main 팔 0 회다. 출구 `h` 는 두 팔 모두 0 회다.

### V4a (random_m seed 0–9, reported)

세션의 재접근 [완료, 시도, 거절]. 완료는 시도 창 안의 도킹 round 다(`tools/leg_failures_pairs.py --arms off on`).

| seed | off | on | 차 |
|---|---|---|---|
| 0 | 3/3 | 3/3 | 0 |
| 1 | 2/2 | 2/2 | 0 |
| 2 | 2/2 | 2/2 | 0 |
| 3 | 3/4 | 3/3 | +0.25 |
| 4 | 3/3 | 2/2 | 0 |
| 5 | 2/2 | 3/3 | 0 |
| 6 | 3/5 | 2/2 | +0.40 |
| 7 | 3/3 | 3/3 | 0 |
| 8 | 2/3 | 3/3 | +0.33 |
| 9 | 0/0 | 0/0 | 쓰지 않음 |

| 기준(사전 등록) | 결과 | 판정 |
|---|---|---|
| 1. 쌍 검정 p < 0.05, 나쁜 쌍 ≤ 2 | 쌍 9, p 0.125, 나쁜 쌍 0 | 불합격 |
| 2. 완료 미션 합 ≥ off, `transport_recovery_after_docking` 합 ≤ off | 23 대 21, 0 대 0(세션 중단 off 1, on 2) | 통과 |
| 3. 거절 합 비증가 | 0 대 0 | 통과 |
| 4. 시도 0 쌍 쏠림(차 3 이상이면 판정 불가) | s9 하나, 두 팔 모두 0 | 통과 |

### V4b (line_m seed 0–4, 기능)

- 완료 미션 off 12, on 11. 기준(2 건 넘게 적지 않음) 통과. 세션 중단 off 2, on 0.
- 재접근: off 6/6, on 6/6.

### V4c (random_m seed 0–4, 관절각, 기능)

- `transport_recovery_after_docking`: off 0, on 0. 기준(늘지 않음) 통과.
- 완료 미션 off 12, on 13. 재접근 off 13/15, on 12/12. 세션 중단 off 1, on 0.

### 공통

- 금지 접촉: 두 팔 모두 0.
- 실패한 미션의 사유(D7 과 관계있는 것만 판정에 쓴다)

  | 사유 | off | on |
  |---|---|---|
  | `docking_retry_match_refused:correction_out_of_bounds`(D7 재접근 뒤 매칭 거절) | 3(random_m s3 · s7, tq s0) | 0 |
  | `return_home:invalid_start` | 1 | 3 |
  | `observe_replan_failed:invalid_start` | 2 | 1 |
  | 그 밖(복귀 추종, 들기 기울기, `return_home:no_path`, `session_anchor_unaligned`) | 3 | 2 |
  | 세션 중단 `slam_link_failed`(마지막 미션) | 4 | 2 |

  - `return_home:invalid_start` 의 on 3 건(random_m s4 m3 · s6 m3, 관절각 s2 m2)은 모두 관측 단계에서 미션 전체 경로를 처음 계획하다 났다(`pallet_mission.py` 의 복귀 경로 계획).
  - 보류는 정상적으로 풀렸다. 예를 들어 on s4 m2 는 H1 시작 324.8 s → settle release 356.7 s → 복귀 끝 보류 → mission_end release 388.9 s 였고, m3 실패는 419.3 s 다. D7 보류가 남은 채 복귀를 달린 사례는 아니다.
  - 다만 release 는 자세와 하역 팔레트 좌표를 바꾸고, SLAM 추적기·지도는 세션 내내 남는다. 그러니 "앞 미션의 보류 → 주행·보정·하역 위치의 변화 → 다음 미션 계획"이라는 경로는 열려 있다. **보류 구간 밖에서 났지만 인과는 확정하지 못했다**(Codex V4 판정 검토 P2).

## V5 Isaac 확인 배치

등록: 계획 2a(7판과 수집기 수정, Codex 12 차 "새 P1/P2 없음", 총괄 승인). 판정은 120 실행이 모두 끝난 뒤 등록한 명령으로 한 번 했다.

### 실행 환경

- 코드: 스냅숏 `lf_030c105e`(이 브랜치 030c105e, main 878753e6 병합). 두 팔이 같은 스냅숏을 쓰고 `--d7-reapproach-hold` off / on 만 다르다.
- 장면 random_m, seed 10–69. 런처 `ct1_queue.sbatch`, `SCENARIO=queue`, 고정 손 순서 P-4:D-1,P-5:D-2,P-3:D-3, `QUEUE_EXTRA` 없음. 오도메트리는 런처 기본 관절각 `team_spec_quantized` 다.
- 목록은 seed 순이다. seed 마다 두 팔의 앞뒤를 `default_rng(20261026)` 으로 정했다(off 먼저 29 / 60).
- 묶음: 2 슬롯 8 잡(11474–11481, shard 2 · CPU 4 · 13G · 4:45). 한 목록과 한 claim-dir 를 함께 썼다.
  - 묶음 runner 는 시작 전에 8 잡 모두 main 1393ea8c(PR #150) 판으로 바꿨다. 바뀐 것은 틱 정지 120 s, 멈춤 진단, runner 존재 조건이고 각 실행의 코드·옵션은 그대로다.
  - 슬롯 시간 사용 92 %, 꼬리 0.4 % 였다(ws1-잡-효율화 측정).
- 실행: 120 개가 각각 한 번씩 끝났다. 기반 실패·OOM·시간 부족으로 남은 실행은 0 이다.
- 세션 중단은 등록 규칙대로 다시 돌리지 않고 그때까지의 기록으로 셌다(끊긴 미션은 0 점).

  | 중단 | off | on |
  |---|---|---|
  | `slam_link_failed` | 3(s37 · s52 · s55) | 1(s53) |
  | 멈춤 감시 종료(기록 없는 미션 1, off 최선 경계로 셈) | 1(s25) | 0 |

  - 세션당 CPU 2 인 묶음에서 세션과 SLAM 브리지가 물리 코어 하나를 같이 써서 생긴 것으로 본다(ws1-잡-효율화). 같은 계열을 CPU 2 단독 array task 로 돌렸을 때는 3/3 이 이 실패였다. 두 팔은 같은 잡에 섞여 있었다.
- 디스크: 끝난 실행의 큰 출력(recognition_missions · spool · live_log · perception_capture · scene.usda)은 `gdrive:forklift-archive/ws1-v5-030c105e/` 로 옮기고 ws1 에서 지웠다. 판정 입력(probe.json · 실행 result.json · missions/)은 남겼다.

### 판정

명령: `python3 tools/leg_failures_pairs.py --v5 --collect /home/projects/forklift/test_runs/live v5_030c105e random_m --arms off on --first-seed 10 --seeds 60`(스냅숏의 도구). 출력은 ws1 `test_runs/live/v5_bundle/v5_judgement.json` 이다.

| 기준 | off(최선 경계) | on(최악 경계) | 판정 |
|---|---|---|---|
| 일차: 세션 점수 쌍 차 단측 정확 부호 검정 | — | 26 승 · 6 패 · 28 동점 | p = 0.00027 < 0.05, 통과 |
| 합계 점수 on ≥ off | 121 | 155 | 통과 |
| 1. 전체 실패 on ≤ off + 6 | 39 | 25 | 통과 |
| 1b. D7 뒤 실패 on ≤ off | 9 | 3 | 통과 |
| 2. on 팔 금지 접촉 0 · 확인 불가 0 | — | 0 · 0 | 통과 |
| 3. 재접근 계획 거절 on ≤ off | 0 | 0 | 통과 |
| 기록 불일치 세션 | 0 | 0 | 판정 가능 |

- 쌍 점수 차(on − off) 분포: −3 1, −2 1, −1 4, 0 28, +1 14, +2 7, +3 5.
- 알 수 없는 미션은 off 1 개(s25, 멈춤)뿐이었다. off 최선 경계로 깨끗한 완료로 셌다.

### 판정에 쓰지 않은 기술 통계

- 계수 재접근 시도: off 130 / 160 도착(정체 30), on 118 / 118 도착이다. 거절은 두 팔 모두 0 이다.
- on 팔에서 보류(`hold_reapproach`)는 53 실행에서 118 번 작동했다. off 팔은 0 번이다. 보류 출구(h)는 0 번이다. 보류 기록은 실행 `result.json` 에 있다. 미션 `records.json` 에는 없다.
- 미완료 미션 사유:

  | 사유 | off | on |
  |---|---|---|
  | `transport_recovery_after_docking` | 7 | 0 |
  | `docking_retry_match_refused` | 2 | 3 |
  | `observe_replan_failed` | 3 | 6 |
  | `scene_replan_refused` | 3 | 0 |
  | `session_anchor_unaligned` | 2 | 3 |
  | `slam_link_failed`(세션 중단) | 3 | 1 |
  | 그 밖(`return_home`, 기록 없음) | 2 | 0 |

- 민감도(판정 아님): off 의 `slam_link_failed` 세 쌍(모두 on 우위)을 빼도 23 승 · 6 패로 p ≈ 0.001 이다. 합격은 이 중단들에 기대지 않는다.
- on 의 `observe_replan_failed` 6 건은 관측 단계 계획 실패로 D7 앞에서 났다. 보류는 D7 재접근에서만 걸린다. 다만 V4 와 같이 앞 미션의 보류가 세션 상태를 거쳐 다음 미션 계획에 닿는 경로는 열려 있어, 인과를 확정하지는 않는다. 이 실패도 점수 0 과 보조 기준 1 에 이미 들어 있다.

## 남는 일

- 실차: 실차 오도메트리 표류(재접근 길이 2–3 m, cusp 1–2 회, 적재 상태)를 실측한다. 그 값으로 보류 상한과 출구 문턱(0.12 m · 0.08 rad, max(2L, L + 2 m), 40 s)을 다시 정한다([실차 이식 검토](../design/real-vehicle-port-review.md)).
- 리보틱스 장면은 V5 에서 따로 검증하지 않았다. 등록문대로 같은 기본을 넣었고, 보류가 걸리는 조건(SLAM 추적, D7 재접근)은 두 런처에서 같다. 그다음 리보틱스 배치에서 `D7_HOLD` 를 기록에 남기고 새 실패 유형이 없는지 본다.
- 출구 E1 은 V4 · V5 를 통틀어 한 번도 걸리지 않았다. 출구 경로는 단위 시험으로만 검증돼 있다.
