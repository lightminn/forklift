# 하역 뒤 복귀 출발 무효 — 복귀 경로 앞의 곧은 물러남, 검증 결과 (2026-10-27)

계획: `docs/plans/2026-10-27-return-start-withdraw-extension.md`(3 판, Codex 4 회차 수렴, agy 독립 검토). 지시: 총괄(2026-10-27) — `return_home:invalid_start` 4 미션의 원인 분리, F2 조건부 승인(근본 원인 확인·좁은 장면 여유·기본값은 Isaac 뒤). 정책(사용자 2026-10-24): 복구 로직·런처 기본값은 사용자 결정 없이 정하고 문서·보고로 남긴다.
구현: branch `fix/return-withdraw-extension`. Isaac 비교에 쓴 코드는 `b35d7f53`(스냅숏 `rsb_b35d7f53`, PR #167 이전 main 위)이다. 그 뒤로 바뀐 것은 main 병합(rebase)뿐이다.

## 결론

- **기능만 넣고 기본은 끈다.** 러너 `--return-start-backoff` 는 off 다. 런처(CT-1 `isaac_live.sbatch`·`ct1_queue.sbatch`, 리보틱스 `riib_scene.sbatch`)에 기본값을 넣지 않았다. 켜려면 CT-1 래퍼는 `MORE`, 리보틱스는 `EXTRA_ARGS` 에 `--return-start-backoff on` 을 준다.
  - 효과의 근거는 CPU 다(C2·C3). 기록 4 사례가 모두 풀리고, 이웃 추정 오프셋 스윕에서 출발 무효 126/126 을 풀며 유효 출발 195/195 의 경로를 바꾸지 않았다.
  - Isaac 비교(live 20·riib 32 실행)에서 노출은 0 이다. 두 팔 모두 복귀 출발 무효가 한 번도 나지 않았다. 그래서 계획서 5 장 C5 의 기본값 조건(채택·후진 완료·복귀 완료 사례)은 채워지지 않았고, 이 비교는 해가 없음을 보는 확인으로만 읽는다.
  - 켬 팔의 실패는 모두 이 기능 밖에서 났다(아래 실패 목록).
- **근본 원인은 저장 경로의 결함이 아니다**(계획서 2 장). 하역 줄에서 차의 위치 추정이 truth 에서 치우친 만큼 이웃 추정이 밀린다. 콘솔의 S3 값으로 보면 SLAM 근본 해결 A(조사 지도 위 위치 추정)에서는 이 치우침이 임계값보다 훨씬 작다(아래). live 기본이 A 로 바뀌었으므로(PR #167) 전환 뒤 첫 배치에서 치우침과 발동 빈도를 다시 잰다.
- **리보틱스 trailer 회귀 신호를 따로 보고한다.** 두 팔 모두 0/5 다(rcf 는 3/5·4/5). 이 기능과 무관하고, 지금 런처 기본값의 회귀 후보다(아래 절).

## 무엇을 넣었나

계획서 3 장(3 판) 그대로다.
- **R1 스위치**: `--return-start-backoff on|off`(기본 off, 근접 계약 off). 끄면 계획 함수에 새 인자가 가지 않는다. `recorded_arguments` 에 `return_start_backoff: off` 가 남고, `start_digest` 에 `state.return_start_backoffs` 항목이 생긴다.
- **R2 거는 곳**: 물러남 끝에서 출발하는 복귀만이다.
  - ① 미션 계획(`plan_transport`·`plan_loaded_delivery`). 러너의 첫 미션 계획, core 의 관측 계획(동기·작업자), 막힘 재계획과 그 tight 재시도, 적재 준비 작업이다.
  - ② 도킹 해제 재계획(`slam_release` 동기, `_release_request` 작업자)
  - ③ 격자 모드의 첫 `live_leg` 복귀(동기·작업자)
  - 일반·tight 호출 모두에 넘긴다. 정지·출발 패드·cusp 재계획에는 넘기지 않는다.
- **R3 거리**: 1 cm 간격, 상한 0.30 m. 다음 두 조건을 모두 만족하는 첫 거리가 조언이다.
  - (a) 끝 자세가 복귀의 출발 검사(탐색 소품·KEEPOUT·자기 팔레트·계획 영역·격자, travel 여유)를 통과한다.
  - (b) 출발에서 그 끝까지의 직선이 칸 단계 소품에 대해 planner 여유를 지킨다.

  L = min(조언 + 0.03, 끊기지 않고 이어지는 최대, 상한)이다. 조언 + 0.01 에 못 미치면 걸지 않는다.
- **R4 경로**: 후진 직선 끝에서 기존 복귀(턴어라운드 포함)를 계획해 잇는다. `PlanResult.start_backoff` 에 기록한다. 실패 결과에도 남고, 쓰지 않을 때는 `record_omit_default` 로 기록에서 뺀다. 미션 계획 trace 의 `return_home` 항목에도 남는다.
- **R5 실행**:
  - 해제·live_leg 재계획의 (b) 는 `@slot` 만 T = 지금 목적지 ∘ 준비 때 목적지⁻¹ 로 옮긴다(`moved_slot_props`).
  - 저상 감시는 후진을 붙인 복귀 경로를 그대로 따르는 추종기에서만 [0, L) 을 칸 단계로 본다(`LegClearance.slot_to_m`). P6 탈출처럼 추종기만 바꾸면 적용되지 않는다.
- **R6 기록**: `state["return_start_backoffs"]` 에 시도마다 `{where, advice_m, length_m, status, adopted}` 를 남긴다.
  - 작업자가 tight 결과로 바꾼 첫 시도는 `first_backoff` 로 돌아와 `adopted: false` 로 남는다. 적재 작업은 `return_backoffs` 로 돌려준다.
  - 미션 기준값에 등록한다.

## CPU 확인

- **C2 기록 4 사례**(구현으로 다시 계산, `plan_return_leg` 와 턴어라운드 켬도 같음):

| 실행 · 미션 | 끔 | 켬 | 조언 m | L m | 복귀 길이 m |
|---|---|---|---:|---:|---:|
| rh off_random_m_s8 m3 | invalid_start | 성공 | 0.17 | 0.20 | 14.40 |
| rh on_random_m_s4 m3 | invalid_start | 성공 | 0.21 | 0.24 | 14.42 |
| rh on_random_m_s6 m3 | invalid_start | 성공 | 0.19 | 0.22 | 14.56 |
| rh on_tq_random_m_s2 m2 | invalid_start | 성공 | 0.08 | 0.11 | 13.17 |

- **C3 이웃 추정 오프셋 스윕**(`tools/return_backoff_sweep.py`): 기록 장면 두 개(s8 m3 의 P-5, tq s2 m2 의 P-4)에서 이웃 `@search` 를 칸에서 dy 0–0.35(0.01 간격) × dx {−0.10, 0, +0.05} × dyaw {0, ±0.015} 만큼 옮겼다. 장면마다 324 칸이다.
  - 두 장면 모두 결과가 같다. 끔에서 출발 무효인 126 칸을 켬이 모두 푼다. 출발이 유효한 195 칸은 켬과 끔의 경로가 비트 단위로 같다.
  - 막히기 시작하는 dy 는 0.21(dyaw +0.015)·0.22(0)·0.23(−0.015) m 다. L 은 dx 에 따라 0.07(+0.05)·0.12–0.13(0)·0.22–0.23(−0.10) m 이고, dy 에는 따르지 않는다.
  - 나머지 3 칸(dy 0.22, dyaw −0.015, dx 셋 모두)은 출발은 유효한데 탐색이 `no_path` 다. 이 기능은 출발 무효에만 걸리므로 켬도 끔과 같다.
- **C1 실행 시험**:
  - 기록 장면에서 후진 뒤 같은 후진 기어로 곡선을 이어 가는 복귀를 운동학 추종기로 끝까지 따라갔다. 처음 L + 0.30 m 동안 칸 소품과의 간격은 0.317 m 다.
  - 후진 직후 전진으로 바뀌는 합성 cusp 경로도 도착한다. 기록 장면에서는 후진 끝에서 전진할 길이 없어 cusp 가 나오지 않는다.
- **좁은 장면 여유(총괄 요청 2)**: 계획서 4 장 표. 22 장면·30 하역 칸 모두 칸 뒤쪽이 0.60 m 넘게 비어 있어 상한 0.30 에 닿지 않는다(벽 포함, 여유 0.10).
- 단위 시험:
  - `tests/unit/planning/test_return_start_backoff.py` 16
  - `tests/unit/mission/test_core_return_backoff.py` 9
  - `tests/unit/tools/test_return_backoff_runner.py` 2
  - `test_low_clearance.py`·`test_low_clearance_core.py` 의 slot_to 시험 3
  - 소스 시험(`test_isaac_transport_planning_config.py`) 이름공간 갱신
- 재생 게이트 runs29 29/29(`replay_gate/rsb_b35d7f53`, 끔).
- ws1 전체 pytest(`b35d7f53`): 7144 passed, 알려진 ws1 전용 실패 10 건만.

## Isaac 비교 (rsb)

조건: 스냅숏 `rsb_b35d7f53`(PR #167 이전 → live 두 팔 모두 mapping, MAP_BUNDLE 없음). 두 팔은 런처 기본값을 그대로 쓴다(team-spec 오도메트리, D7_HOLD on, 리보틱스 RESTART_CONFIRM 3·LOW_CLEARANCE on). 팔 차이는 `--return-start-backoff on` 하나이고, 두 팔이 같은 seed 를 번갈아 돈다. 묶음은 PR #161 크기(live 3 슬롯 shard 4, riib 3 슬롯 shard 4)로 냈고 의존성은 걸지 않았다.

### CT random_m 대기열(live, 11710·11711)

P-4:D-1, P-5:D-2, P-3:D-3, seed 0–9. rh 에서 이 실패가 난 장면이다.

| 팔 | 미션 완료 | 유효 짝 9 seed | 후진 시도 / 채택 |
|---|---|---|---|
| 끔 | 30/30 | 27/27 | — |
| 켬 | 27/28 | 26/27 | 0 / 0 |

- 세션 중단: 켬 s5 의 셋째 미션 운반 중 `slam_link_failed: scan 4427: no reply within 1.00 s`. 10 시간에 1 건꼴인 알려진 중단이라 짝 비교에서 s5 를 뺐다.
- 켬 실패: s3 m2(P-5→D-2) 운반 중 P6 `scene_replan_failed:permit_stop:straight_collision`. 복귀가 아니고 후진 기록도 없다. 그 순간 SLAM 위치가 truth 와 0.23 m 차이였다.

### 리보틱스 장면(riib, 11712·11713, `scenes_p1c_81a007b1`)

| 장면 · seed | 끔 완주 | 켬 완주 | 후진 시도 |
|---|---|---|---|
| aisle_rack 0–7 | 2/8 | 3/8 | 0 |
| line_2_m 0–2 | 3/3 | 3/3 | 0 |
| trailer 0–4 | 0/5 | 0/5 | 0 |

- aisle_rack 의 실패:
  - 끔: 운반 시간 초과 3, P6 `no_path` 1, 복귀 추종 실패 2
  - 켬: 운반 시간 초과 3, 근접 캡처 `opening_width_mismatch` 1, s0 의 `slam_release_replan_failed: no_path` 1
  - 켬 s0 는 해제 재계획이 출발 무효가 아니라 `no_path` 여서 후진이 걸리지 않았다(`start_backoff` 없음). 이때 계획기는 끔과 같은 경로를 탄다.
  - 나머지 유형은 rcf 에도 있다.
- 감시의 `slot_to` 구간이 생긴 추종기: 0(노출 0 과 같다).

### 리보틱스 trailer 회귀 신호 (이 기능과 무관, 총괄 보고)

- 두 팔 10 실행이 모두 같은 양상이다.
  - 꺼내기 직후 트레일러 안(후축 0.56, 0.12)에서 허가가 `occupied` 로 0 을 낸다.
  - P6 재계획은 `invalid_start` 로 실패하고, 운반 69 s 만에 시간 초과로 끝난다(운반 occupied 제한 틱 약 8 270).
- 재출발 확인은 무장 1 번·막은 틱 0 이라 원인이 아니다.
- rcf(2026-10-26, 그때 기본)는 3/5·4/5 였다. 그 뒤 riib 런처 기본에서 바뀐 것은 team-spec 오도메트리와 그 장애물층 설정(#137), D7 재접근 보류(#155)다.
- 진단: trailer 0–4 를 `ENCODER_MODEL=reported` 로 5 실행 더 돌렸다(`rsbt_b35d7f53`, 11732). 결과: (진행 중)

## SLAM 근본 해결 A 와의 관계 (콘솔 S3 2 차 최종값, 계획 5d `eb7610aa`)

하역 도킹 첫 회의 위치 추정 − truth 다.

| 지점 | A(조사 지도 위 위치 추정) | mapping |
|---|---|---|
| random_m 하역 줄 | \|e\| 최대 0.045 m(s3 30 회), 0.044 m(s4 6 회). 미션이 거듭되어도 커지지 않음 | s3 0.086·0.112 m, s4 0.073 → 0.565 m(미션마다 커짐, 주로 x) |
| line_m | 0.050·0.051 m | 0.218·0.186 m |

- 이 기능이 걸리는 경계는 이웃 추정이 칸 쪽으로(y) 약 0.21 m 밀릴 때다. A 의 오차는 그보다 한 자릿수 작다. mapping 의 큰 오차는 주로 x 방향이라 이 경계와 축이 다르지만, rh 처럼 y 로 0.2 m 넘게 밀린 사례도 있었다.
- live 기본이 A 로 바뀌었으므로(PR #167, 2026-10-27) 전환 뒤 첫 random_m 배치에서 하역 줄 치우침과 이 기능의 시도 빈도를 다시 잰다(총괄 요청).

## 한계와 남은 일

- Isaac 에서 후진이 실제로 설치·주행된 적이 없다. 감시의 `slot_to`, 해제 재계획의 T, 작업자 기록 통로는 단위 시험과 CPU 로만 확인했다.
- 출발은 유효한데 첫 움직임이 막히는 `no_path`(C3 의 3 칸, 켬 aisle s0 의 해제 재계획)는 이 기능의 범위 밖이다.
- 같은 칸을 한 세션에서 다시 쓰는 경우는 범위 밖이다(계획서 6 장).

## 실행 위치

- ws1 `test_runs/live/rsbl_b35d7f53`(요약 `rsb_sum.py`, `sum_live.json`), `test_runs/riibotics/rsbr_b35d7f53`(`sum_riib.json`), `test_runs/riibotics/rsbt_b35d7f53`(trailer 진단), 스냅숏 `snapshots/rsb_b35d7f53`, 재생 게이트 `replay_gate/rsb_b35d7f53`.
- 기록 4 사례의 원자료: `gdrive:forklift-archive/ws1-cleanup-20261027-d/` `d27_rh_01`·`d27_rh_02`. 시험 고정 자료 `tests/fixtures/return_start_backoff_rh_s8_m3.json`.

## 자체 검토

- ① 재계산: C2·C3·4 장 여유는 구현 코드로 다시 계산했다. Isaac 표는 실행 기록(`missions/mK`, `run/result.json`)에서 요약 스크립트로 셌다.
- Codex 문서 검토: (진행 예정)
