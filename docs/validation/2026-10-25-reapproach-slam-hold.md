# D7 재접근 동안 SLAM 보정 보류 검증 (H1 · E1)

계획: [재접근 보정 보류 2판](../plans/2026-10-25-reapproach-slam-hold.md). 브랜치 `feat/d7-slam-steps`.

## 결론

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
- 사전 규칙대로 런처 기본값은 **off 그대로** 둔다. 코드(`--d7-reapproach-hold`, 기본 off)는 넣는다. (다음 단계는 아래 "남는 일")

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

## 남는 일

(Codex 판정 검토 뒤 채운다)
