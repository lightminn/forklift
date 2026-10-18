# Isaac 미션 연결 — 원거리 촬영을 새 인식·추적이 맡는 한 사이클

> **실행 방식:** 이 세션에서 직접 구현, Codex 검토를 수렴까지 받고 진행한다.

**Goal:** 새 인식(`instance_fit` → `PocketCandidate` → `select_target`)과 원거리 추적·융합(`candidate_tracking`, 2B-3h)을 실제 Isaac 미션 실행기(`sim/isaac/run_transport.py`)에 넣고, 미션 한 사이클(관측 → 계획 → 근거리 촬영 → 도킹·삽입 → 들기 → 운반 → 내리기 → 복귀)이 끝까지 도는지 확인한다.

사용자 결정(2026-10-18):
- "실제 우리 아이작심 파이프라인에 넣어서 한 사이클 모두 완료되는지 테스트해."
- 연결 방식은 **"새 인식이 원거리를 맡음"**: 원거리 촬영 검출을 새 인식 + 대상 선택으로 바꾸고, 접근 주행 중 프레임을 추적기에 넣어 융합 추정을 계획 목표로 쓴다. 근거리 촬영 뒤는 기존 경로 그대로다.

**브랜치:** `feat/isaac-recognition` = `origin/main`(미션·실시간 실행기) + `origin/feat/p5-p0a`(인식 2A–2B-3h) 병합(충돌 없음).

## 지금 구조 (조사 결과)

- **원거리 촬영:** 관측 지점 도착 → `MissionCore.capture_and_detect`(core.py)가 `SensorCapture` 정지 프레임으로 `detect_pockets(detector_input, prior, params)` → `PocketObservation`.
  - 성공(`observe_succeeded`): `estimate_pallet_center_m` → `PalletSite` → 계획 목표.
  - 실패(`observe_failed`): 다음 관측 지점.
- **같은 함수를 쓰는 다른 촬영:**
  - 근거리 촬영: `near_capture.status == "pending"`.
  - 근접 복구 재관측: `start_pickup_retry`·`start_near_realign` 이 `near_capture` 를 지우고 `retry_stand` 를 세움(core.py:4994, 5124). 다시 못 보면 즉시 실패.
- **M-c 미션 구성**(ws1 `live.sbatch`, 사이클 성공 실적):
  - 쓰는 것: `--use-perception --pallet-prior epal6 --planning-target perception --perception-mount carriage_low_measured --depth-quantize-mm 1 --record-slam --return-home --obstacle-layer … --obstacle-act --grid-planning --slam-feedback … --d7-planner --d7-docking`, 공장 배치, EPAL 6.
  - **`--near-tracking`·`--pocket-check` 는 쓰지 않는다.** 근접 계약(`near_tracking.condition_problems`)은 측정 실행과 다른 인자를 모두 거부하므로, 그 둘과 새 인식을 함께 쓰는 실행은 이 범위 밖이다.
- 새 인식의 제안은 렌더러 정답 인스턴스 이미지에서 만든 `oracle` 뿐이다. Isaac 렌더 도구가 프림 라벨(`add_labels`)과 `decode_semantic` 으로 이를 만든다. `decode_semantic` 은 **라벨 붙은 바닥 화소가 있어야** 준비 완료다(recognition_render_core.py:125).

## 바꾸는 것 (2판)

1. **인스턴스 마스크(`--recognition far` 일 때만):**
   - 장면의 팔레트 프림(`/World/Pallet`)과 바닥 프림에 렌더 도구와 같은 계약(`scene_label(base, token)`, 바닥 라벨 포함)으로 라벨을 붙인다.
   - 인식 카메라에 의미 분할을 더한다.
   - 촬영(정지·주행 중 모두)은 깊이와 **같은 프레임**의 분할을 `decode_semantic` 으로 읽어 `instance_ids` 를 낸다. 준비 안 됨이면 그 시도는 받지 않는다(깊이와 같은 재시도 규칙).
   - 제안은 `ids == k` 마스크, `source = "oracle"`(2B 평가와 같은 수준, 실제 검출기 아님).
2. **적용 범위(촬영 종류별):**

   | 촬영 | 경로 |
   |---|---|
   | 원거리 관측(`near_capture` 없음 또는 pending 아님, `retry_stand` 아님) | **새 인식** |
   | 근거리 촬영(pending) | 기존 `detect_pockets` |
   | 근접 복구 재관측(`retry_stand`) | 기존 `detect_pockets` |

   `--recognition far` 는 `--near-tracking`·`--pocket-check` 와 함께 쓰면 시작 전에 거부한다(근접 계약의 측정 조건 밖).
3. **원거리 판정:** 새 경로에서도 지금처럼 `detect_pockets` 를 **먼저 같은 순서로 호출**한다(경계 재생의 호출 순서와 기존 기록 재생 유지). 그 결과는 `attempt["detector_reference"]` 로만 남기고, 미션이 쓰는 `observation` 만 새 인식의 것으로 바꾼다.
   - **단일 프레임 판정**은 정지 프레임의 `fit_scene` 전체 결과(모든 후보 + `failures` 를 `unresolved` 로)에 `select_target` 을 적용한다. 2B 평가의 선택 의미 — 모호 후보는 `reobserve` — 를 그대로 유지한다.
   - **지시**는 관측 배치의 적재 구역 사전(`BAY_PICKUP_ZONE`, 참값 아님 — 실행기의 시야점 계산과 같은 정보)의 중심, 반경은 구역 반대각선 + 팔레트 반대각선이다. 선택 시각의 **제어 자세(SLAM 추정, map←base)** 로 base 좌표로 바꿔 넘긴다.
   - **대상 유지 규칙(설계 §8, 자동 대체 없음):** 대상 선택 상태는 트랙 만료와 따로, **관측 다리 하나** 동안 유지한다. 관측 다리마다(다음 관측 지점의 재관측 포함) 추적기와 선택을 새로 시작한다 — 재관측이 곧 새 판정이다.
     - 다리 안에서 선택이 이미 있으면, 정지 프레임 판정이 `selected` 이고 그 후보를 정지 프레임에서 **실제로 받은**(`FrameReport.accepted`) 트랙이 **그 선택 트랙**일 때만 그 트랙의 융합 후보를 쓴다(`observation_source = "fused"`).
     - 다른 트랙이 받았거나, 아무도 받지 않았거나, 선택 트랙이 지워졌으면 `no_pallet`(`reason = "target_lost"` / `"target_inconsistent"`)이다. 다른 후보로 바꾸지 않는다.
     - 다리 안에서 선택이 한 번도 없었으면 정지 프레임 판정이 처음 선택한다: 그 후보를 받은 보이는 트랙이 있으면 그 트랙을 선택하고 그 출력을 쓴다(`"fused"`, 관측 수 기록). 받은 트랙이 그림자이거나 없으면 단일 프레임 후보를 쓴다(`"single"`).
   - 그 밖(`reobserve`·`none`·`not_pickable`·`ask_operator`)은 `status = "no_pallet"` 관측 → 기존 `observe_failed`.
4. **주행 중 추적:** 관측 다리(`observe` 단계)를 달리는 동안 0.2 s(24 물리 스텝)마다 인식 카메라를 렌더하고, 깊이·분할로 `fit_scene` 을 돌려 `CandidateTracker.update` 에 넣는다.
   - **선택 초기화(정답 없이):** 프레임마다 그 프레임의 단일 프레임 판정(위 3 과 같은 규칙)이 `selected` 이고 그 후보를 받은 트랙이 보이면, 그 다리에서 **아직 한 번도 선택이 없었을 때만** `tracker.select(track_id)` 한다 — 2B 평가의 "처음 대상을 가리킨 보이는 트랙"을 정답 대신 판정으로 정한 것. 한 번 선택하면 그 다리에서 다시 고르지 않는다. 선택 트랙이 지워져도 다른 트랙으로 바꾸지 않고, 정지 판정이 `target_lost` 가 된다(위 3).
   - **프레임 계약:**
     - 한 프레임 = 같은 렌더의 깊이·분할, 그 취득 시각(실행기의 `stamp_now` 원점, `initial_time` 을 뺀 sim s — 정지 촬영 stamp 도 같은 원점으로 변환), 그 취득 시각의 **odom←base**(SLAM 보정 없는 주행 기록, epoch 0).
     - 보정표 `config/recognition_track_bounds.yaml`(known [1, 4) m, parametric [1, 3) m — 밖은 `untracked`, 기록).
   - **배치 픽셀 지연:** 연기 시험에서 렌더 깊이의 팔레트 앞면 거리 d(t)를 `d_truth(t − τ) + b` 로 맞춘다(τ 는 0–48 스텝 정수, b 는 상수).
     - 출발 가속·정지 감속 구간이 있어 τ 와 b 가 갈린다.
     - **채택 조건:** 속도 변화 구간이 있고, 잔차 p95 ≤ 10 mm 이며, 두 구간(가속·감속)에서 같은 τ 가 나와야 한다.
     - 채택되지 않으면 주행 중 프레임을 쓰지 않고 정지 프레임만 쓴다(융합 판정 J2 는 불합격으로 기록).
5. **변환** `candidate_to_observation`:
   - 지원 형상은 알려진 형상(`epal6`·`t11_06`)만이다. 진입축 깊이 = 하류의 고정 깊이(`pallet_depth_m`)인지 검사하고, 아니면 거부한다.
   - 포켓은 `Pocket(center_m = PocketEstimate.centre_m, width_m, height_m)` 이다(앞면 개구 중심, insertion-left 순서 — instance_fit.py:541).
   - `position_sigma_m = yaw_sigma_rad = None` — 추적 예산은 p95 경험 예산이라 1σ 가 아니다. 예산은 기록에만 둔다.
   - `source_provenance = "synthetic"`.
6. **기록·재계산:**
   - 프레임마다 `recognition/frame_XXXX.npz` 를 남긴다: 깊이·분할 id·`base_from_optical`·내부 파라미터·취득 stamp·odom·제어 자세.
   - `result.json` 의 `recognition` 블록: 프레임별 후보·판정·트랙 보고, 선택 이력, 최종 판정(`observation_source`, 선택 트랙의 관측 수·예산), 원거리 추정의 참값 오차, 같은 프레임의 단일 프레임 후보 오차, `detector_reference` 오차, 인식 1 회 시간, `untracked` 수.
   - 재계산 도구 `tools/recompute_far_recognition.py`: 기록만으로 같은 판정·관측을 다시 만든다(비트 동일 확인).
   - 기존 `--recognition off` 기록의 재생·검출 재생은 바뀌지 않는다. `diagnose_detection` 은 `observation_source` 가 있는 시도에서 `detector_reference` 와 비교한다.
7. 기본값은 지금 동작(`--recognition off`). 기존 실행·재생·시험은 그대로.

## 판정 (한 사이클)

ws1 Slurm, M-c 미션 구성(위)에서 `--live` 만 빼고 `--recognition far`, 배치 실행, SLAM 다리 컨테이너, seed 3 개.

**J1 — 새 인식이 원거리를 맡은 사이클(seed 마다).** 다음을 모두 만족한다.
- `success: true`(phase `complete`, 배달 오차 < 0.08 m, 팔레트 접지, 포크 이탈).
- 계획에 넘어간 원거리 관측의 출처가 새 인식(`observation_source` 가 `fused` 또는 `single`)이고, 판정 `selected`.
- 원거리 추정 오차가 2B-3h 평가(`evaluation_i`)와 맞는다(오프라인 평가와의 일치 기준).
  - 오차는 평가기와 **같은 함수**(`evaluate_track_sequences.errors` — 좌우 포켓 중 최악의 가로·삽입축, yaw)로 잰다.
  - 거리 구간은 평가기의 `_distance`·`BINS`([1.3, 3.5) m, 0.5 m 구간)를 쓰되, `camera_x` 는 기본값(CPU 리그 0.75 m)이 아니라 **취득 프레임의 `base_from_optical` 평행이동 x**(M-c `carriage_low_measured` 0.619 m)를 명시해 넘긴다 — 구간과 범위 밖 여부 모두 이 값으로 정한다(기본값이면 131 mm 짧게 계산됨, Codex).
  - 기준값은 그 구간의 V1·V2 p95 중 큰 값이다 — 관측 다리는 어느 한 접근 기하와 같지 않다. 단일이면 `latest` 열, 융합이면 `output` 열.
  - 정지 거리가 [1.3, 3.5) 밖이면 이 조건은 "평가 구간 밖"으로 기록하고 J1 을 **판정 불가**로 둔다(통과로 세지 않음).

**J2 — 융합 연결(seed 마다, J1 과 별도).** 다음을 모두 만족한다.
- 픽셀 지연이 채택됐다.
- 선택 트랙이 서로 다른 취득의 주행 프레임 판독을 2 개 이상 받았다.
- 계획에 넘어간 관측이 `fused` 이고, 그 융합에 쓰인 관측 수 ≥ 2.

**대조:** 같은 snapshot·seed 로 `--recognition off` 한 번씩(사이클 성공·시간·배달 오차·원거리 오차).

**불합격 처리:** 원인(마스크·인식·선택·변환·추적·지연·기존 단계)을 기록하고 고친 뒤 같은 seed 로 다시 돈다(통합 시험이라 판정 집합 개념 없음). 3 seed 는 통합이 도는지 보는 수이지 통계 근거가 아니다.

## 작업

1. `src/forklift_core/perception/recognition_bridge.py`(새, SDK 없음):
   - 함수·클래스:
     - `proposals_from_ids`;
     - `instruction_from_zone(zone, pallet_spec, map_from_base)`;
     - `decide(fit, instruction)`(단일 프레임 판정);
     - `candidate_to_observation`;
     - `FarRecognition(bounds)`(프레임 투입·선택 초기화·정지 판정·기록).
   - 시험(먼저 실패 확인):
     - 포켓 중심·순서;
     - 지원 형상·깊이 검사;
     - 변환 → `estimate_pallet_center_m` 이 후보 팔레트 중심과 일치;
     - 지시 좌표 변환(Codex 0.932 m 반례);
     - 모호 후보 옆의 정상 후보는 `reobserve`(Codex 반례);
     - 정지 프레임에서 받지 않은 트랙의 0.2 s 전 후보는 쓰지 않음(Codex 반례);
     - 그림자가 받은 판독 → 단일;
     - 선택 초기화는 판정으로만, 다리 안 재선택 없음, 선택 트랙이 보이는데 다른 트랙이 받으면 `target_inconsistent`(Codex 2판 반례), 선택 트랙 삭제 → `target_lost`, 다리마다 새 시작;
     - J1 오차가 평가기 `errors` 와 같은 값(최악 포켓 — Codex 2판 반례 14.7 mm);
     - 프레임 없음 → 단일;
     - 지연 추정의 τ·b 분리(일정 속도만 있으면 채택 안 함).
2. `sim/isaac/perception_adapter.py`: 같은 프레임의 분할 읽기, 준비 판정, stamp 원점 변환(가짜 카메라 시험).
3. `core.py`·`run_transport.py`:
   - `--recognition`, 라벨·분할, 주행 중 프레임 훅, 원거리 교체(참조 검출 먼저), 거부 조합, 기록;
   - `near_tracking.POST_MEASUREMENT_ARGUMENTS` 에 `recognition: "off"` 등록(근접 실행에서는 off 만 허용 — 위 2 의 거부와 같은 뜻);
   - 기존 미션·재생 시험 그대로 통과.
4. `tools/recompute_far_recognition.py` + 시험.
5. ws1 실행: 연기 시험(1 seed, 지연 측정·라벨·분할 준비 확인) → J1·J2 판정(3 seed + 대조) → 기록 `docs/validation/2026-10-18-isaac-recognition-far-capture.md`.

## 위험

- **제안이 정답 분할이다:** 실제 검출기 없이 Isaac 정답 인스턴스 분할을 제안으로 쓴다. 2B 평가와 같은 한계이고, "새 인식이 미션을 끝까지 이끈다"까지만 보인다. 오검출·가림·조명은 이 시험에 없다(agy).
- **근거리가 원거리 오차를 가린다(agy):** 근거리 촬영이 기존 검출기로 다시 잡으므로 사이클 성공이 원거리 정확도를 입증하지 않는다. 그래서 J1 에 원거리 오차 기준을 두고, 근거리 보정량(원거리 추정 → 근거리 추정 차이)을 기록한다.
- **융합 효과(agy):** 같은 정지 프레임의 단일 프레임 후보 오차를 함께 기록해 융합의 기여를 본다. 3 seed 로는 결론 내지 않는다.
- **렌더 깊이:** Isaac 깊이는 팀 실측 잡음 모형과 다르다(잡음 없음에 가깝다). 보정표는 CPU 리그 + 잡음 + V1/V2 접근 기하에서 나왔으므로, 관측 다리 기하에서 그대로 맞는다고 단정하지 않는다(`untracked`·보류·예산 포함을 기록).
- **처리 시간:** 0.2 s 마다 렌더 + 인식은 배치 실행을 느리게 한다. 실시간(`--live`) 연결은 범위 밖이다.
- **사전 형상:** 근거리·근접 경로가 `--pallet-prior` 를 요구하므로 비규격 팔레트 사이클은 이 범위로 안 된다.

## 검토 기록

### 1판 (2026-10-18) → 2판

- **자체 재계산:** 미션 경로(`capture_and_detect`·`observe_succeeded`·근거리/복구 촬영), M-c 실행 인자(ws1 `live.sbatch` — 근접 추적·포켓 검사 없음), 적재 구역 사전(`BAY_PICKUP_ZONE` x 2.6–3.6, y −0.2–1.5), 지연 상수(`live_runtime.camera_pixel_lag_s`, 11 스텝은 live 측정)를 원자료에서 확인.
- **Codex**(P1 2 · P2 7 · P3 1):
  1. 팔레트 라벨만으로는 `decode_semantic` 이 바닥 없음으로 준비 실패 → 바닥 라벨 포함.
  2. `--recognition far` 는 근접 계약이 거부 → 근접 추적·포켓 검사 없는 M-c 구성으로 시험, 함께 쓰면 시작 전 거부.
  3. `not pending` 이 복구 재관측까지 바꾸고, 0.2 s 전 후보가 재관측 성공을 대신함 → 촬영 종류별 적용표, 정지 프레임에서 실제로 받은 트랙만.
  4. 지시와 후보의 좌표계 불일치(0.932 m)·`scenario.pickup` 은 참값 → 적재 구역 사전 + 제어 자세 변환.
  5. 트랙만 넘기면 모호성 보호가 사라짐 → 정지 프레임 전체 결과로 판정하고 받은 트랙에 연결.
  6. 마지막에만 선택하면 2B-3h 의 선택 트랙 규칙이 작동하지 않음 → 프레임마다 판정으로 선택 초기화, 재선택 없음.
  7. 거리 잔차만으로는 지연과 편향이 갈리지 않음, stamp 원점 불일치 → τ·b 동시 추정과 채택 조건, 프레임 계약.
  8. 재생 복원 불가 → 원자료 프레임 기록·재계산 도구, 참조 검출을 같은 순서로 먼저 호출.
  9. 단일 프레임 대체만으로도 통과하는 판정 → J1(단일 허용)·J2(융합 연결) 분리, 원거리 오차 기준.
  10. p95 예산을 1σ 로 넣지 말 것·변환 지원 형상 검사.
- **agy 독립 검토**(Gemini 3.8 Flash High, 도구 없음, 1·2 결론 비공개): 정답 분할의 한계, 주행 기록 표류·흔들림, 지연 일정성, 원거리 깊이 품질, 반경 안 다른 팔레트, 근거리가 원거리 오차를 가림, 융합 효과 미입증, 3 seed → 위험 절과 J1·J2.

### 2판 → 3판 (2026-10-18)

- **Codex**(P2 2): ① 정지 프레임 판정의 후보를 받은 트랙이 유지 중인 선택 트랙과 달라도 그 트랙을 출력(반례: 선택 0 이 보이는데 출력 1), 삭제 뒤 자동 재초기화가 설계 §8 의 "자동 대체 없음"과 충돌 → 대상 선택을 관측 다리 단위로 유지, 다른 트랙·삭제는 `target_inconsistent`·`target_lost` 로 재관측, 다리마다 새 시작. ② J1 의 중점 오차와 평가기의 최악 포켓 오차가 다름(14.7 mm 반례), 표·범위 미고정 → 평가기 함수·구간, V1·V2 p95 중 큰 값, 범위 밖은 판정 불가.

### 3판 → 4판 (2026-10-18)

- **Codex**(P2 1): J1 거리 구간이 평가기 기본 카메라 위치(0.75 m)를 쓰면 M-c 카메라(0.619 m) 대비 131 mm 짧게 계산돼 범위·구간이 틀어짐(3.55 → 3.419 m) → 취득 프레임의 외부 파라미터 x 를 명시.

### 4판 재검토 (2026-10-18)

- **Codex**: 수렴 — 4판 범위에서 새 P1/P2/P3 없음.
