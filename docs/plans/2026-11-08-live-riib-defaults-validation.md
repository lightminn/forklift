# 라이브 런처 riib 기본 보강 — 조사 결과와 검증 사전 등록 (2026-11-08, 2 판)

연속 이송 현황 조사(#330, `docs/validation/2026-11-06-ct-status-survey.md` 요약 1)의 첫 발견에 대한 조사와, 총괄 결정(2026-11-08)에 따른 최소 검증의 사전 등록이다. 결과를 보기 전에 고정한다. 런처 기본값 승격 구현은 이 검증 결과를 본 총괄 결정 뒤에 따로 한다.

## 0. 결정과 범위 (총괄 2026-11-08)

- **다르게 유지:** ① `--exit-prefix-exempt`, ③ `--restart-confirm-scans`(2026-10-26 결정과 그 재검토 계기 유지), ⑥ `--return-cusp-replans`.
- **검증 뒤 넣음:** ② `--low-clearance-guard on`, ④ `--return-start-backoff boxed`, ⑤ `--near-end-stand on`.
- 범위: 라이브 런처의 `SCENE_FILE` 모드, 등록부의 `line_m`·`random_m` 세션 장면, team_spec, 등록 지도 위치 추정, CT 인식 설정(아래 3.1). 모든 `SCENE_FILE` 운용으로 넓히지 않는다. 승격 구현도 이 조건(장면 해시·encoder·map ID·인식 설정)에 맞을 때만 기본을 바꾼다.
- factory(hall) 모드(`SCENE_FILE` 없음, `--grid-planning`)는 여섯 모두 지금대로 둔다.
- **배속(총괄 결정):** 시뮬-물성 세션의 실시간 기준선을 본 뒤, GPU 1 노드에서 배속 0.4(라이브 런처 기본)를 유지하는 동시 수가 있으면 0.4 로, 없으면 0.3 으로 돌린다. 0.3 으로 돌리면 승격 범위를 배속 0.3 으로 한정한다.
- **④ 의 예외 승격 조건(총괄 결정 (나)):** 라이브는 `invalid_start` 발동으로 후진 설치·후진·복귀 경로를 확인한다. 막힌 시작(`boxed`) 판정 분기는 riib 배치 1 건과 CPU 시험(2.2)을 근거로 받는다. **막힌 시작의 라이브 발동은 관측되지 않았다. 첫 라이브 발동은 개별 조사한다.** CPU 시험이 조건을 채우지 못하면 ④ 는 `on`(`invalid_start` 만) 승격 후보로 내린다.
- 이 검증은 기능이 라이브로 옮겨 가는지와 결합 회귀를 보는 최소 확인이다. 드문 실패율의 무해성을 증명하지 않는다(B 팔 세션 수로는 사건 0 건이라도 단측 95 % 상한이 크다).

## 1. 항목별 조사 결과 (main f7035877)

라이브 런처(`deploy/console/isaac_live.sbatch`)는 여섯을 넘기지 않아 러너 기본(off·0)으로 돈다. 여섯 모두 라이브가 공유하는 `MissionCore` 생성자로 전달된다(run_transport 6875–6950). restart-confirm 만 line_m·random_m(CT-1)에서 검증됐고, 나머지는 riib 계열 장면(aisle·aisle_rack·trailer·line_2(_m)·random_1)과 온라인 매핑에서 검증됐다. 근접 재관측 `lower` 는 검증 시점의 riib 런처 값이다(2026-10-27 22eb0f66 전은 `waive_far_confirmed`, 뒤는 `waive`). 라이브는 등록 지도 위치 추정이고 `lower` 는 CT 래퍼의 `waive_far_confirmed` 다.

| 항목 | riib 값과 출처 | 라이브 | 라이브에서의 영향 | 일부러 다르게 둔 근거 | 결정 |
|---|---|---|---|---|---|
| ① exit-prefix-exempt | on(slam), #237, `2026-10-31-riib-default-batch2.md` 규칙 ① 통과 | off | 라이브는 `--exit-clearance-m` 을 주지 않아 운반 출구 접두가 없다(run_transport 4679). 켜도 latch 가 `no_prefix` 로 닫혀 작동하지 않는다(core 1245). factory 모드는 러너가 거부한다(run_transport 1559) | 기록 없음(누락) | 다르게 유지. 출구 여유·keepout 을 쓰는 라이브 운용이 생기면 따로 검증 |
| ② low-clearance-guard | on, #133, `2026-10-25-low-clearance-guard.md`(겹침 0·완주 감소 없음, Isaac 에서 한 번도 걸리지 않음) | off | 속도 상한만 건다. 상한은 0 이 될 수 있고 자체 복구가 없다. 틱 비용은 최적화 뒤 0.09 ms(가까운 소품 없음)·1.7–2.9 ms(소품 옆), 최적화 뒤 Isaac 배율은 재측정하지 않았다. 등록된 낮은 소품(라이브는 다른 세션 팔레트, 높이 0.144 m)만 추정 자세로 본다. factory 는 소품이 1.15 m 프리즘이라 작동하지 않는다 | 기록 없음 | 검증 뒤 |
| ③ restart-confirm-scans | 3, #148, `2026-10-26-restart-confirmation.md` | 0 | — | **있음**: "CT-1·콘솔 런처 기본은 끔"(CT 세션과 합의, CT-1 켬 10 실행 노출 0). 그 사례가 다시 모이면 재결정 | 다르게 유지 |
| ④ return-start-backoff | boxed(slam), #263, `2026-11-02-return-start-backoff-boxed-batch.md` | off | 복귀가 `invalid_start`·막힌 시작이면 먼저 직선 후진한다. 적재 재제출(`live_prepare_loaded`)에도 전달된다. grid-planning 복귀 경로는 미검증 | 기록 없음 | 검증 뒤 |
| ⑤ near-end-stand | on(slam), #273, `2026-11-03-near-end-stand-c2-injection-results.md` | off | 근접 재관측 다리 끝 재계획을 제어 루프에서 동기로 부른다(core 3213, 라이브 계획 작업자를 쓰지 않음) | 기록 없음 | 검증 뒤 |
| ⑥ return-cusp-replans | 2(slam), #282, `2026-11-04-return-cusp-replan-reregistration-results.md` | 0 | `return_home` 의 failed·endpoint_heading·at_cusp·not off_path 만 받는다. CT 조사에서 이 대상은 확인되지 않았다(집 최종 정지·`cusp_pose` 재계획은 다른 규칙). 계획을 동기로 부르고(core 6341) riib 계획 시간이 중앙값 1.32 s·최대 9.85 s 다 | 기록 없음 | 다르게 유지. 라이브 대상이 확인되면 재검토 |

## 2. 발화 fixture (CPU, 러너·미션 코어 변경 없음)

세 fixture 모두 기존 러너 인자다. 시험 전용이며 기본값으로 쓰지 않는다. 결과는 자연 발생률이 아니다.

### 2.1 ② 저상 감시: `--low-clearance-margin-m 0.30 --low-clearance-deviation-m 0.015`

- 감시 기준은 `min(margin, max(0, c − deviation))` 다(`low_clearance.guard_cap`). margin 을 키우고 deviation 을 줄이면, 낮은 소품 0.30 m 안에서 계획보다 1.5 cm 넘게 가까워질 때 건다.
- 오프라인 확인(직선 빈 차 다리 옆 팔레트 높이 소품, 0.3 m/s): 기본값은 계획 여유 0.10 m 에서 5 cm 가까워져야 걸고, 0.20·0.30 m 에서는 5 cm 로도 걸지 않는다. fixture 는 계획대로면 걸지 않고, 2 cm 이상 가까워지면 0.10·0.20·0.30 m 모두 상한 0 으로 세운다.
- 라이브 장면의 낮은 소품은 다른 세션 팔레트다(정적 소품은 1.3 m 칸 표지라 보지 않는다). 이웃 팔레트 옆을 지날 때 걸릴 것으로 보지만 보장은 아니다.

### 2.2 ④ 복귀 출발 후진: `--withdrawal-m 0.40`

- 라이브 런처의 `--withdrawal-m 0.55` 뒤에 `EXTRA_ARGS` 로 와서 이긴다.
- 오프라인 확인(032 진단 캡처의 라이브 형상·설정, 두 장면의 D-1): 물러남 끝이 여유 0 에서는 비고 계획 여유 0.10 m 에서 막힌다(`invalid_start`). `return_start_backoff` 의 조언은 0.03–0.04 m, 후진 길이 0.06–0.07 m 다(riib 자연 발생 조언 0.035–0.04 m). 0.45 m 이상에서는 막히지 않는다.
- 이 fixture 는 물러남 길이와 복귀 출발점을 바꾼다(`site_poses` 의 `withdrawn` 만, 목적지·삽입 목표는 그대로). 짧아진 물러남이 팔레트 변위·포크 이탈·SLAM 상태에 주는 영향은 따로 적는다. 후진 기록은 사전계획의 채택과 실제 물러남 뒤의 설치·주행을 나눈다. 명목 자세의 CPU 확인은 실제 하역 오차와 보정 해제 뒤에도 같은 발동이 난다는 보장이 아니다.
- **막힌 시작(`boxed`) 발동은 이 형상으로 만들지 못했다**(0.45 m 에서 첫걸음이 열려 있다). 이 fixture 는 `invalid_start` 발동만 다룬다.
- **막힌 시작의 CPU 시험(0 절 (나)의 조건):** `tests/unit/mission/test_core_return_boxed_install.py`. riib 기록의 막힌 시작 해제 재계획(`rc_G_aisle_rack_s1`, 모든 시도가 노드 1 의 `no_path`)을 `MissionCore._release_request` 가 묻고(복귀 후진 인자에 boxed 키), 실제 계획 작업자(`plan_worker._run("leg")`)가 계획하고, `MissionCore.job_hold` → `_release_finish` 가 받는다. 경로와 새 `RearAxlePathTracker` 의 첫 다리가 `length_m` 만큼의 직선 후진(방향 −1)이고, `return_start_backoffs` 에 채택된 `boxed` 로 남는다. boxed 없이(`on`)는 같은 재계획이 `slam_release_replan_failed` 로 끝난다. 라이브의 해제 재계획이 쓰는 작업자 경로다.

### 2.3 ⑤ C2: `--inject-near-end-detour` (`--near-end-stand shadow|on`)

- 실행당 한 번, 근접 재관측 다리의 마지막 후진 다리(0.80 m 이상)에서 무장한다(`near_end_detour.decide`). 마지막 다리가 전진이거나 0.80 m 보다 짧으면 `not_armed` 다. line_m·random_m 에서 무장하는지는 실행 전에 알 수 없다.
- 무장하지 않으면 ⑤ 는 fixture 재현 불가로 승격 보류다.

### 2.4 재현물

`docs/validation/assets/live_riib/fixture_offline.py` 와 출력 `fixture_offline.txt`. 입력: 진단 실행 032 full 의 `prepare_input_1.pkl`(sha256 787865e0…, 보관본 `ws1-cleanup-20261108-a/a_uc.tar.zst`), 세션 장면 `line_m_session.json`(83e27b4c…)·`random_m_session.json`(8b8fe554…). 2.1·2.2 의 수치는 이 출력이다.

## 3. 검증 배치 (시뮬-물성 실시간 기준선 측정이 끝난 뒤 제출)

### 3.1 공통

- 스냅숏 = 제출 시점 main 하나(모든 세션 같음). ws1 래퍼 `test_runs/live/ct1_queue.sbatch` 의 CT 인식 설정(원거리 학습 제안기·`--near-capture-lower waive_far_confirmed`·`--near-capture-expected on`·`--near-capture-redraw always`·redraw lateral 0.10)을 그대로 쓴다. 탐침 옵션을 넘기는 복사본을 이 배치용으로 만든다(래퍼는 ws1 스크립트이고 러너는 바꾸지 않는다).
- 등록부 장면·지도, `ENCODER_MODEL` team_spec, `D7_HOLD` on, 배속 0.3, `MAX_EPOCHS=1`, 묶음 자동 재시도 0.
- 노드: 지금 VM 때문에 Slurm 은 GPU 1·CPU 8·30 GB 다. 예약은 `tools/isaac_bundle.py sizes live`(기본 3 세션·shard 4·CPU 8·19 GiB)로 하되, 동시 수와 배속은 실시간 기준선으로 고르고 통합 A/B 안에서는 고정한다. 그때의 노드 조건(GPU·CPU·메모리, 다른 잡)을 결과에 적는다.
- 스텝 간격: 모든 세션에 `FORKLIFT_LIVE_TICK_TRACE=1` 을 같이 준다. 물리 스텝 간격은 틱 추적의 `S` 간격으로, 상태 수신 간격과 따로 잰다. 누락 행이 있는 구간은 판정에서 빼고 수를 적는다. 측정 창은 첫 임무 제출부터 마지막 임무 종료까지다.
- 제출 전에 실행 폴더·스냅숏 이름을 총괄에게 준다(디스크 세션 전달은 총괄). 시뮬-물성 세션에 제출 시각을 알린다.
- 팔 A = 라이브 현재 기본. 팔 B = A + ② + ④ + ⑤(기본값, fixture 없음). ①·③·⑥ 은 두 팔 모두 꺼 둔다.

### 3.2 세션 (최초 18)

임무 순서는 CT 조사와 같다: line_m `P-1:D-1,P-2:D-2,P-3:D-3`, random_m `P-4:D-1,P-5:D-2,P-3:D-3`. seed 는 표에 적은 것 하나다.

| 묶음 | 장면 × 설정 | seed | 세션 | 탐침 |
|---|---|---|---|---|
| C2 | line_m·random_m × `--near-end-stand shadow`·`on`, 둘 다 `--inject-near-end-detour`, ②·④ 는 꺼 둠 | 0 | 4 | queue, 세 임무 |
| 감시 표적 | line_m·random_m × 팔 B + 2.1 fixture | 0 | 2 | 세 임무, 감시 정지 회복 절차(아래) |
| 후진 표적 | line_m·random_m × 팔 B + 2.2 fixture | 0 | 2 | queue, 세 임무 |
| 통합 A/B | line_m·random_m × A·B | 0·1 | 8 | queue, 세 임무, 팔 순서를 섞어 같은 동시 수·기록 부하 |
| 생명주기 1 | line_m × 팔 B | 0 | 1 | `main`: 캡처 중 STOP·재무장·TELEOP·재개 |
| 생명주기 2 | random_m × 팔 B | 0 | 1 | `cycle --cycle P-4:D-1,P-4:D-1,P-5:D-2 --cancel-at transport:5`: 운반 중 취소 → 적재 유지 → 같은 팔레트 재제출 거부(`cargo_held`) 확인 → 다음 임무 |
| 생명주기 3 | random_m × 팔 B | 0 | 1 | queue `P-4:D-1`, 환경 `START_RELOC=on RELOC_UNATTENDED=1`: 묶음의 알려진 출발(`BUNDLE_TOKEN`) 대신 시작 재위치 → 무인 확정 → 탐침이 `running` 을 기다려 제출(random_m 은 등록부에서 `start_relocalizable`). 모호해 `awaiting_pick` 이면 고르지 않고 기록대로 끝난 것으로 센다 |

- 합계 최초 19. 확장은 통합 A/B 의 짝이 기반 실패(Isaac 기동 실패·`startup_silent`·`startup_viewport`·탐침 전 래퍼 종료)로 깨졌을 때 같은 짝을 다시 내는 것뿐이다. 상한 25. 노출이 적다는 이유로 늘리지 않는다.
- SCENE_FILE 모드는 grid-planning 이 아니어서 적재 재제출(`live_prepare_loaded`)에 가지 않는다(`live_loaded_pallet` 는 grid_planning 일 때만). 생명주기 2 는 그 거부를 확인한다. 적재 재제출의 양성 확인은 이 검증의 범위 밖이다.
- **감시 정지 회복 절차**(별도 tools PR 로 탐침에 넣는다, 결과 전 고정): 임무가 도는 중 바퀴 속도 0·위치 변화 1 cm 미만이 20 s 이어지고 계획 대기·도킹 정지가 아니면 정지로 본다 → 그 임무를 취소 → TELEOP 으로 허가된 직선 후진 한 번(최대 0.3 m) → 정지 → 다음 임무 제출. TELEOP 이 거부되거나 후진이 허가되지 않으면 보류로 기록하고 세션을 끝낸다. 정지 원인은 실행 기록의 감시 상한 시각과 맞춰 사후에 귀속한다. TELEOP 은 AUTO 후보와 다른 경로라 감시가 수동 이동까지 보호한다고 보지 않는다.
- 각 세션의 전체 인자 목록(래퍼 환경·`EXTRA_ARGS`·러너 `--check-args` 출력)을 제출 전에 결과 폴더에 남긴다.

### 3.3 판정 (결과 전 고정)

**공통.**

- 팔 B 와 fixture 세션의 안전값이 모두 0: 금지 접촉, 낮은 소품 겹침, `unpermitted_dir_entries`, 적재 안전 사건.
- 판정에 필요한 기록이 없으면 그 항목은 판정 불가이고 승격하지 않는다. 기능 기록 위치: ④ `missions/<id>/records.json` 의 `state.return_start_backoffs`, ⑤ 같은 곳의 `observe_end_stands`·`near_end_injections`, ② 는 임무 기록에 없으므로 epoch 실행 기록의 누계·시각.
- 팔 B 에서만 난 실패는 기능 발화와 관계없이 모두 조사한다. 원인을 좁히지 못하면 그 항목은 미해결로 남고 승격하지 않는다.
- 페이스(통합 A/B, 팔별): 중앙값 실시간 배율, 최대 스텝 간격, stall 수(간격 > 0.25 s, pacer 정의), 최장 연속 `pacing.ok=false` 를 적는다. **합격 규칙(총괄 결정):** ⑤ 의 동기 재계획 간격(아래 상한 안)을 따로 집계해 빼고, B 의 stall 수 또는 최장 `pacing.ok=false` 구간이 A 보다 나쁘면 보류다. B 의 중앙값 배율 ≥ A 의 0.95 는 보조 예산이다. 이 규칙은 기존 실시간 목표(stall 0.25 s)의 충족을 뜻하지 않는다.
- 최소 유효 발화: ② 감시 표적에서 상한 1 회 이상, ④ 후진 표적에서 설치 → 실제 후진 → 복귀 완료 1 회 이상, ⑤ C2 on 에서 `installed` → 근접 캡처 → 삽입 1 회 이상. 못 미치면 그 항목은 재현 불가로 보류다.

**② 감시.** fixture 세션에서 실제 상한(요청보다 낮은 값) 발생을 빈 차·적재로 나눠 세고, 상한 0 정지와 그 뒤 수동 회복·다음 임무를 확인한다. 상한이 한 번도 나오지 않으면 fixture 재현 불가로 보류한다. 통합 A/B 와 생명주기에서 감시가 건 경우는 모두 열거한다.

**④ 후진.** fixture 세션에서 `invalid_start` 발동 → 후진 설치 → 실제 후진 → 복귀 완료가 이어진 경우를 센다(설정 적용·시도·설치·실제 주행을 나눠 적는다). 적재 재제출 경로(생명주기)에서 인자가 전달·기록되는지 확인한다. 막힌 시작 발동은 2.2 대로 총괄 판단이다.

**⑤ C2.** `installed` 가 설치 검증을 채운 사례이고, 유효 노출은 그 뒤 근접 캡처 → 삽입까지 이어진 경우다. `not_armed`·`no_budget`·shadow 판정도 시도·판정 표에 남기고 `no_budget` 은 미회복으로 센다(기존 C2 판정과 같다).

- 동기 재계획의 제어 스텝 간격: 재계획을 시작한 물리 스텝부터 다음 물리 스텝까지의 벽시계 시간(틱 추적).
- C2 재계획 중 STOP: STOP 은 틱 시작에서 모아 처리한다(`live_runtime.tick_start` → `link.begin`). 그래서 재계획 중 보낸 STOP 의 처리 지연 상한은 그 스텝 간격이다. 이 배치는 재계획 중 STOP 을 보내지 않고 간격으로 판정한다.
- 상한(정책, 사전 고정, 기존 실시간 기준이 아님): 재계획 스텝 간격 2.0 s. 넘으면 ⑤ 는 비동기 처리 검토가 먼저이고 승격하지 않는다.
- 일반 STOP(생명주기 1): 탐침의 STOP 송신 → 그 요청의 확인(응답) 지연, 확인 뒤 첫 상태에서 정지 래치와 바퀴 명령 0, 이동 중 STOP 의 감속·정지 시간을 나눠 적는다. 보고용이다.

**승격 권고.** 항목마다 그 기능 확인이 유효하고, 통합 A/B·생명주기의 공통 기준을 통과하고, 미해결 실패가 없을 때만 승격을 권고한다. 그 밖은 보류이고 사유를 적는다. 결과는 팔별로 적고, 이전 라이브 기록(CT 조사 O0·O6 등)과 한 분모로 합치지 않는다.

## 4. 공유

⑤ 의 재계획 중 제어 간격·STOP 지연과 ② 의 틱 비용 측정값은 시뮬-물성 세션의 실시간 작업과 관련 있으므로 결과를 그쪽에도 보낸다.

## 5. 자체 검토

- 조사·권고 Codex 2 라운드(2026-11-08): 1 라운드 P2 5·P3 2 로 초안을 바꿨다.
  - ① 은 라이브에 출구 접두가 없어 작동하지 않으므로 검증 뒤가 아니라 유지.
  - CT 래퍼 설정과 라이브 런처 기본을 구분하고 범위를 두 장면·등록 지도·CT 인식으로 한정.
  - ⑤·⑥ 의 동기 계획 위험, ⑥ 은 라이브 대상 미확인이라 유지.
  - 기록 위치(② 는 임무 기록에 없음).
  - 24 세션은 무해성 증명이 아님.
  - 페이스 경계는 보조 예산.
  - riib 의 EXTRA_ARGS 처리는 항목마다 다름.
  2 라운드 "새 P1/P2 없음", 사전 등록 문구 셋(C2 미설치도 표에 남김, 승격 조건은 장면·encoder·지도·인식 범위, STOP 지연 정의·상한 사전 고정)을 1 판에 반영했다.
- 1 판 Codex 검토(P2 5·P3 6) → 2 판:
  - 적재 재제출은 SCENE_FILE 에서 닿지 않음 → 거부 확인으로 바꿈(총괄 승인).
  - queue 의 `--manual-recovery` 는 감시 정지를 회복하지 않음 → 회복 절차를 사전 고정하고 별도 tools PR.
  - C2 STOP 정의가 처리를 검증하지 못함 → 틱 시작 처리라 스텝 간격이 상한임을 근거로 간격 판정, 일반 STOP 은 세 지연으로 나눔.
  - boxed 처분 미고정 → 총괄 결정 (나)와 CPU 시험(MissionCore 요청 → 작업자 → 설치)으로 고정.
  - 배속·페이스 규칙 → 총괄 결정으로 고정.
  - P3: 재현물 추가, `lower` 서술 정정, seed·임무 목록·최소 성공 고정, 시작 재위치는 무인 확정 뒤 대기열 제출로 명시, `sizes` 는 예약용이고 동시 수는 실시간 기준선으로, 스텝 간격은 틱 추적 `S` 간격.
