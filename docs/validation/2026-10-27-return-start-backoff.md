# 하역 뒤 복귀 출발 무효 — 복귀 경로 앞의 곧은 물러남, 검증 결과 (2026-10-27)

계획: `docs/plans/2026-10-27-return-start-withdraw-extension.md`(3 판, Codex 4 회차 수렴, agy 독립 검토). 지시: 총괄(2026-10-27) — `return_home:invalid_start` 4 미션의 원인 분리, F2 조건부 승인(근본 원인 확인·좁은 장면 여유·기본값은 Isaac 뒤). 정책(사용자 2026-10-24): 복구 로직·런처 기본값은 사용자 결정 없이 정하고 문서·보고로 남긴다.
구현: branch `fix/return-withdraw-extension`. Isaac 비교에 쓴 코드는 `b35d7f53`(스냅숏 `rsb_b35d7f53`, PR #167 이전 main 위)이다. 그 뒤로 바뀐 것은 main 위로의 rebase 와, 러너의 첫 미션 계획이 시도를 상태에 남기는 기록 수정(배치 경로, Codex 문서 검토 P2)이다.

## 결론

- **기능만 넣고 기본은 끈다.** 러너 `--return-start-backoff` 는 off 다. 런처(CT-1 `isaac_live.sbatch`·`ct1_queue.sbatch`, 리보틱스 `riib_scene.sbatch`)에 기본값을 넣지 않았다. 켜려면 CT-1 래퍼는 `MORE`, 리보틱스는 `EXTRA_ARGS` 에 `--return-start-backoff on` 을 준다.
  - 효과의 근거는 CPU 다(C2·C3). 기록 4 사례가 모두 풀리고, 이웃 추정 오프셋 스윕에서 출발 무효 126/126 을 풀며 유효 출발 195/195 의 경로를 바꾸지 않았다.
  - Isaac 비교(live 20·riib 32 실행)에서 노출은 0 이다.
    - 켬 팔 26 실행의 기록 인자는 모두 `return_start_backoff: on` 이다.
    - 실행 폴더의 모든 JSON·JSONL 어디에도 후진 기록 키(`start_backoff`·`return_start_backoffs`·`first_backoff`·`return_backoffs`)가 없다. 이 키는 시도가 있어야만 써진다.
    - 그래서 계획서 5 장 C5 의 기본값 조건(채택·후진 완료·복귀 완료 사례)은 채워지지 않았다. 이 비교는 회귀를 점검한 자료일 뿐이고, 해가 없음을 통과했다는 판정은 아니다.
  - 켬 팔의 실패에는 후진 기록이 없고, 관측된 단계도 복귀 출발이 아니다(아래 실패 목록). 다만 이 기능은 미션 계획 때부터 호출되므로, 그것만으로 영향이 완전히 배제되지는 않는다.
- **근본 원인은 저장 경로의 결함이 아니다**(계획서 2 장). 하역 줄에서 차의 위치 추정이 truth 에서 치우친 만큼 이웃 추정이 밀린다. 콘솔의 S3 값으로 보면 SLAM 근본 해결 A(조사 지도 위 위치 추정)에서는 이 치우침이 임계값보다 훨씬 작다(아래). live 기본이 A 로 바뀌었으므로(PR #167) 전환 뒤 첫 배치에서 치우침과 발동 빈도를 다시 잰다.
- **리보틱스 trailer 회귀 신호를 따로 보고한다.** 두 팔 모두 0/5 다(rcf 는 3/5·4/5). 두 팔이 같은 지점에서 같은 기전으로 막혔고 후진 기록이 없어, 지금 런처 기본값의 회귀 후보로 본다(아래 절).

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
- **R6 기록**: `state["return_start_backoffs"]` 에 시도마다 `{where, advice_m, length_m, status, adopted}` 를 남긴다. 러너의 첫 미션 계획도 그 trace 에서 남긴다(Codex 문서 검토 P2 로 추가).
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
  - 막히기 시작하는 dy 는 0.21(dyaw +0.015)·0.22(0)·0.23(−0.015) m 다. L 은 dx 에 따라 0.07–0.08(+0.05)·0.12–0.13(0)·0.22–0.23(−0.10) m 이고, dy 에는 따르지 않는다.
  - 나머지 3 칸(dy 0.22, dyaw −0.015, dx 셋 모두)은 출발은 유효한데 탐색이 `no_path` 다. 이 기능은 출발 무효에만 걸리므로 켬도 끔과 같다.
- **C1 실행 시험**:
  - 기록 장면에서 후진 뒤 같은 후진 기어로 곡선을 이어 가는 복귀를 운동학 추종기로 끝까지 따라갔다. 처음 L + 0.30 m 동안 칸 소품과의 간격은 0.317 m 다.
  - 후진 직후 전진으로 바뀌는 합성 cusp 경로도 도착한다. 기록 장면에서는 후진 끝에서 전진할 길이 없어 cusp 가 나오지 않는다.
- **좁은 장면 여유(총괄 요청 2)**: 계획서 4 장 표. 22 장면·30 하역 칸 모두 검사한 0.60 m 까지 막힘이 없어 상한 0.30 에 닿지 않는다(벽 포함, 여유 0.10).
- 단위 시험:
  - `tests/unit/planning/test_return_start_backoff.py` 16
  - `tests/unit/mission/test_core_return_backoff.py` 9
  - `tests/unit/tools/test_return_backoff_runner.py` 3
  - `test_low_clearance.py`·`test_low_clearance_core.py` 의 slot_to 시험 3
  - 소스 시험(`test_isaac_transport_planning_config.py`) 이름공간 갱신
- 재생 게이트 runs29 29/29(`replay_gate/rsb_b35d7f53`, 끔).
- ws1 전체 pytest(`b35d7f53`): 7144 passed, 알려진 ws1 전용 실패 10 건만. main 위로 rebase 한 `75b4fd17`: 7259 passed, 같은 10 건만. 재생 게이트는 `rsb_75b4fd17`·최종 코드 `rsb_109a2f31` 도 29/29.

## Isaac 비교 (rsb)

조건: 스냅숏 `rsb_b35d7f53`(PR #167 이전 → live 두 팔 모두 mapping, MAP_BUNDLE 없음). 두 팔은 런처 기본값을 그대로 쓴다(team-spec 오도메트리, D7_HOLD on, 리보틱스 RESTART_CONFIRM 3·LOW_CLEARANCE on). 팔 차이는 `--return-start-backoff on` 하나이고, 두 팔이 같은 seed 를 번갈아 돈다. 묶음은 PR #161 크기(live 3 슬롯 shard 4, riib 3 슬롯 shard 4)로 냈고 의존성은 걸지 않았다.

### CT random_m 대기열(live, 11710·11711)

P-4:D-1, P-5:D-2, P-3:D-3, seed 0–9. rh 에서 이 실패가 난 장면이다.

| 팔 | 기록된 미션 완료 | 유효 짝 9 seed(s5 제외, 예정 27 미션) | 후진 시도 / 채택 |
|---|---|---|---|
| 끔 | 30/30 | 27/27 | — |
| 켬 | 27/28 | 25/27(기록 26) | 0 / 0 |

- 세션 중단: 켬 s5 의 셋째 미션 운반 중 `slam_link_failed: scan 4427: no reply within 1.00 s`. 총괄 규칙대로 SLAM 링크 중단은 기능과 따로 세고, 짝 비교에서 s5 를 뺐다.
- 켬 s3 은 둘째 미션이 실패한 뒤 대기열이 멈춰 셋째 미션 기록이 없다(예정 27 중 기록 26).
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

### 리보틱스 trailer 회귀 신호 (두 팔이 같음, 총괄 보고)

- 두 팔 10 실행이 모두 같은 양상이다(실행마다 `run/result.json` 에서 확인).
  - 꺼내기 직후 트레일러 안(후축 0.56, 0.12–0.13)에서 허가가 `occupied` 로 0 을 낸다.
  - 운반 단계의 P6 재계획은 `invalid_start` 로 실패하고, 운반 시작 66–69 s 뒤 시간 초과로 끝난다(운반 occupied 제한 틱 7 951–8 274).
- 재출발 확인은 무장 1 번·막은 틱 0 이라 원인이 아니다.
- rcf(2026-10-26, 그때 기본)는 3/5·4/5 였다. 그 뒤 riib 런처 기본에서 바뀐 것은 team-spec 오도메트리와 그 장애물층 설정(#137), D7 재접근 보류(#155)다.
- 진단: trailer 0–4 를 `ENCODER_MODEL=reported` 로 5 실행 더 돌렸다(`rsbt_b35d7f53`, 11732).
  - 1/5 완주했다(s0). s1–s4 는 같은 기전이다(운반 occupied 7 939–8 236 틱, P6 `invalid_start`).
  - 그 P6 의 탈출 후보는 전진 0.3·0.6 m 가 쓸림은 통과하지만 경사 경로가 `occupied` 라 자격이 없었다.
  - 그래서 team-spec 장애물층 설정은 주원인이 아니다.

#### 다음 진단과 판정 규칙 (2026-10-27, 결과 전에 적음 — 총괄 지시)

두 배치를 나란히 낸다. 모두 trailer seed 0–4, `scenes_p1c_81a007b1`, 리보틱스 3 슬롯 묶음, 의존성 없음이다.
- ① **rcf 코드**: `fdf0e789`(rcf 의 `rc_fdf0e789` 와 같은 커밋)를 그 시점 런처 기본으로 돈다(reported 오도메트리, RESTART_CONFIRM 0). rcf 끔 팔(3/5)과 같은 조건이다.
- ② **지금 코드, D7_HOLD 만 끔**: `rsb_b35d7f53` 에 `D7_HOLD=off`, 나머지는 지금 런처 기본이다. 지금 기본(0/10)과 비교한다.

판정(완주 수 / 5). 표본이 작아 가운데 값은 판정하지 않는다.
- ① ≥ 3: rcf 이후 코드 회귀로 본다 → ② 결과와 함께 이분으로 간다.
- ① ≤ 1: 코드가 아니라 실행 환경 쪽으로 본다(공유 GPU·묶음 부하·장면 자산 등) → 환경 비교로 간다.
- ① = 2: 판정하지 않고 seed 5–9 를 더 돈다.
- ② ≥ 3: D7_HOLD 가 관여한 것으로 본다.
- ② ≤ 1: D7_HOLD 는 아니다.
- ② = 2: 판정하지 않고 seed 5–9 를 더 돈다.
- ① ≥ 3 이고 ② ≤ 1 이면 #151 전후 스냅숏으로 이분할지 총괄이 정한다.
- 기전 확인: 실패 실행마다 운반 occupied 틱과 P6 상태를 같이 적는다. 완주했더라도 같은 정지가 있었는지(운반 occupied 틱이 수천 단위인지) 본다.

## SLAM 근본 해결 A 와의 관계 (콘솔 S3 2 차 최종값, 계획 5d `eb7610aa`)

하역 도킹 첫 회의 위치 추정 − truth 다.

| 지점 | A(조사 지도 위 위치 추정) | mapping |
|---|---|---|
| random_m 하역 줄 | \|e\| 최대 0.045 m(s3 30 회), 0.044 m(s4 6 회). 미션이 거듭되어도 커지지 않음 | s3 0.086·0.112 m, s4 0.073 → 0.565 m(미션마다 커짐, 주로 x) |
| line_m | 0.050·0.051 m | 0.218·0.186 m |

- 이 기능이 걸리는 경계는 이웃 추정이 칸 쪽으로(y) 약 0.21 m 밀릴 때다. A 의 하역 도킹 위치 오차 최대 0.044–0.045 m 는 그 약 1/4.7 이다.
  - 두 지표는 같지 않다. S3 값은 도킹 첫 회의 차 위치 오차이고, 경계는 다음 미션에 저장된 이웃 추정의 y 오프셋이다. rh 에서는 둘이 −0.04 – +0.015 m 안에서 같았다(계획서 2 장).
  - S3 은 `D7_HOLD=off`, 이번 rsb 는 on 이다.
  - mapping 의 큰 오차는 주로 x 방향이라 이 경계와 축이 다르지만, rh 처럼 y 로 0.2 m 넘게 밀린 사례도 있었다.
- live 기본이 A 로 바뀌었으므로(PR #167, 2026-10-27) 전환 뒤 첫 random_m 배치에서 하역 줄 치우침과 이 기능의 시도 빈도를 다시 잰다(총괄 요청).

## 한계와 남은 일

- Isaac 에서 후진이 실제로 설치·주행된 적이 없다. 감시의 `slot_to`, 해제 재계획의 T, 작업자 기록 통로는 단위 시험과 CPU 로만 확인했다.
- 요약 스크립트의 `adopted_done` 은 채택 기록이 있는 미션의 완료 수다. 채택한 경로가 나중에 바뀔 수 있으므로, 후진 구간 자체를 끝까지 탔는지는 세지 못한다. 노출이 생겨 기본값을 판단할 때는 그 미션의 추종기 진행·경로 교체 기록으로 따로 세야 한다.
- 출발은 유효한데 첫 움직임이 막히는 `no_path`(C3 의 3 칸, 켬 aisle s0 의 해제 재계획)는 이 기능의 범위 밖이다.
- 같은 칸을 한 세션에서 다시 쓰는 경우는 범위 밖이다(계획서 6 장).

## 실행 위치

- ws1 `test_runs/live/rsbl_b35d7f53`(요약 `rsb_sum.py`, `sum_live.json`), `test_runs/riibotics/rsbr_b35d7f53`(`sum_riib.json`), `test_runs/riibotics/rsbt_b35d7f53`(trailer 진단), 스냅숏 `snapshots/rsb_b35d7f53`, 재생 게이트 `replay_gate/rsb_b35d7f53`.
- 기록 4 사례의 원자료: `gdrive:forklift-archive/ws1-cleanup-20261027-d/` `d27_rh_01`·`d27_rh_02`. 시험 고정 자료 `tests/fixtures/return_start_backoff_rh_s8_m3.json`.

## 자체 검토

- ① 재계산: C2·C3·4 장 여유는 구현 코드로 다시 계산했다. Isaac 표는 실행 기록(`missions/mK`, `run/result.json`)에서 요약 스크립트로 셌다.
- Codex 문서 검토(1 회차): P1 없음. 런처 off 유지 판단은 타당하다는 판정이다.
  - P2: 켬 유효 짝 26/27 → 25/27(기록 26)로 고쳤다.
  - P2: 집계 스크립트가 미션 기록만 세서 노출 0 을 확정할 수 없음 → 실행 폴더 전수 키 검사와 켬 인자 확인을 더했다.
  - P2: 러너의 첫 미션 계획이 시도를 상태에 남기지 않음 → 코드를 고치고 시험을 넣었다.
  - P3 반영: 실패와 기능의 관계를 관측 단계·기록으로 좁혔다. trailer 10 실행의 기전을 실행마다 확인했다. S3 비교를 약 4.7 배·지표 차이·D7_HOLD 차이로 고쳤다. L 0.07–0.08, "0.60 m 까지"로 고쳤다.
  - C2·C3·C1 간격·좁은 장면 수치는 독립 재현됐다. ws1 pytest 수치는 기존 로그와 대조됐다.
- Codex 문서 검토(2 회차): 새 P1/P2 없음(수렴). P3 반영: 이력에 기록 수정을 적음, 러너 시험 수 3, 전수 검사가 읽지 못한 파일을 따로 보고하게 함(이번 52 실행에서 0 개), 실패·trailer 의 단정 표현을 좁힘. 후진 구간 완료 집계는 한계 절에 적었다.
