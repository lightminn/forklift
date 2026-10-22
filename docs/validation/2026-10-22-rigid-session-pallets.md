# 세션 강체 팔레트·미션 전환 — 검증 (2026-10-22)

계획: `docs/plans/2026-10-22-rigid-session-pallets.md`(3판 + 구현 리뷰 반영). 상위 계획: `docs/plans/2026-10-22-continuous-transport-ct1.md` PR ③. 브랜치 `feat/ct1-rigid-pallets`.

## Isaac live 세션 (ws1, `shard:2`)

- 설정: 학습 원거리 인식(YOLO11s, pixel lag 12)·SLAM·D7 도킹, PR #88 의 근접 재관측 옵션, live 배율 목표 0.4, seed 0.
  - 탐침 `tools/live_console_probe.py --scenario cycle` 이 콘솔로 세 미션을 차례로 낸다(각 미션은 빈 포크로 시작).
- 장면: 장면 도구 `--session line|random`(표지 없음)과 `--session … --slot-markers`(`<kind>_m_session.json`, PR #99 의 칸 뒤 표지 판을 정적 장애물·벽으로).
  - 첫 실행은 표지 없는 장면으로 `87a4c6e`(잡 8725·8726)에서 돌렸다.
  - 표지 장면은 `a770031`(main #103 병합 뒤, 잡 8751·8752, random_m seed 1 은 잡 8783)에서 돌렸다.
  - 이웃 간격 기록은 `a770031` 에만 있다.
  - 실행 폴더(ws1 `/home/projects/forklift/test_runs/live/`): `ct1_sline_87a4c6e`·`ct1_srand_87a4c6e`·`ct1_slinem_a770031`·`ct1_srandm_a770031`·`ct1_srandm_s1_a770031`.
  - 요약 스크립트: 같은 폴더의 `ct1_report.py <run.log> <장면>`.
- 하역 오차는 미션 종료 시점 마지막 표본의 팔레트 중심과 칸 중심 사이 거리다.

| 장면 | P → D | 결과 | 하역 오차 |
|---|---|---|---|
| line (표지 없음) | P-1 → D-1 · P-2 → D-2 · P-3 → D-3 | done · done · done | 5.6 · 6.5 · 2.5 cm |
| random (표지 없음) | P-4 → D-1 · P-5 → D-2 · P-3 → D-3 | 셋 다 `Pallet missed the green destination center` | 10.6 · 12.6 · 8.9 cm(모두 +y 측방) |
| line_m | P-1 → D-1 · P-2 → D-2 · P-3 → D-3 | done · done · done | 3.2 · 1.7 · 1.2 cm |
| random_m | P-4 → D-1 · P-5 → D-2 · P-3 → D-3 | done · done · **실패** `transport:straight_collision`(준비 단계) | 1.5 · 1.0 cm · — |
| random_m seed 1 | 같은 순서 | done · **실패** `Tracking timeout in return_home`(하역 뒤) · **실패** `transport:straight_collision` | 4.2 · 2.1 cm · — |

- **전환.** 미션이 바뀔 때마다 `session_switches` 에 전환 틱과 앞뒤 팔레트가 하나씩 남았다(line_m · random_m · random_m seed 1 각 2 건). 요청 틱(`session_switch_requests`)과 실제 전환 틱(다음 틱 머리)은 따로 기록됐다.
  - 첫 미션 팔레트(P-1·P-4)는 시작부터 지금 팔레트라 전환이 없다.
- **밀림 0.** 손대지 않은 팔레트의 쉬는 자세 대비 최대 이동은 line_m 0.0005 mm 미만·0.00012 rad, random_m 두 실행 0.0023 mm 이하·0.00026 rad 이다(한계 1 cm·0.01 rad; 2026-10-23 단위 정정 — 요약 스크립트가 mm 로 찍은 값을 m 로 읽었다). `Session pallet displaced` 0 건, 금지 접촉 0 건.
- **이웃 실제 최소 간격**(CT-1 D1 기록). 틱마다 계획 외곽(실음·빈 포크)을 truth 후축에 두고, 다른 팔레트의 지금 외곽과 잰 값의 이웃별 최솟값이다.

  | 장면 | 이웃 | 최소 간격 | 그 순간(단계, 미션 팔레트) |
  |---|---|---|---|
  | line_m | P-2 | 0.353 m | 추출, P-1 |
  | | P-3 | 0.414 m | 운반, P-2 |
  | | P-1(하역됨) | 0.466 m | 내림, P-2 |
  | random_m | P-4(하역됨) | 0.493 m | 운반, P-5 |
  | | P-5 | 0.529 m | 운반, P-4 |
  | | P-3 | 0.756 m | 추출, P-5 |
  | | P-2 · P-1 | 0.840 · 1.991 m | 추출, P-4 |
  | random_m seed 1 | P-4(하역됨) | 0.518 m | 운반, P-5 |
  | | P-3 · P-5 | 0.768 · 0.780 m | 추출, P-5 · 운반, P-4 |

  - CPU 계획 포락선 대조(PR ②, 이상적 추정·하역)의 line 최소 0.343 m(추출, P-2)와 같은 자리·같은 크기다.
  - 이 값은 D1(인식 전 이웃 = 공칭 + 0.25 m)의 경로 존재 한계를 실측으로 대신하지 않는다. 한 seed 의 기록일 뿐이다.
- **실시간 배율.** 강체 팔레트 3 개(line_m)에서 0.354(시뮬 509.6 s / 벽 1438 s), 5 개(random_m)에서 0.344(414 s / 1204 s)였다. 목표는 0.4 다.
- **VRAM.** 최대 3.9 GB 로 `shard:2` 안이다.

### 실패 분류

- **random 표지 없음 — 하역 정밀도(PR #99 소관).** 열린 칸에서 단일 벽 정합이 거부되는 알려진 경우다. 같은 장면의 표지 사본에서는 1.0–1.5 cm 로 들어갔다.
- **random_m 세 번째 미션 — 하역 진입 직선의 계획 충돌.**
  - 준비의 계획 단계가 `approach`·`insert`·`extract`·`transport_search` 까지 성공하고, D-3 로 들어가는 1.5 m 하역 직선에서 `straight_collision` 이 났다.
  - 이 검사는 정적 장애물만 본다. `--layout scene` 은 격자 계획을 쓰지 않으므로 LiDAR·SLAM 지도 격자는 입력이 아니다(Codex 구현 리뷰 3차).
  - 이상적 입력으로는 재현되지 않는다. 오프라인 확인은 이렇다.
    - 장애물: 표지 판 4 장, 인식 전 P-1·P-2(공칭 + 0.25 m), 하역된 P-4·P-5(칸 자세 + 0.21 m).
    - 계획 여유 0.10 m, 적재 외곽(앞 1.56 m·반폭 0.40 m)으로 그 직선을 검사하면 통과한다.
    - 하역된 팔레트를 옆으로 ±0.15 m 옮겨도 통과한다.
    - 그러나 P-5 를 D-3 쪽으로 0.15 m 옮긴 상태에서 yaw ±4° 를 더하면 막힌다(Codex 재계산). yaw 만 ±4° 이거나 반대쪽으로 0.15 m 옮기면 통과한다.
  - 하역된 팔레트는 다음 미션의 장애물로 들어간다. 그 자세는 하역 때 쓰인 보정된 목적지(`return_scenario.destination`, core 4401)의 x·y·yaw 를 그대로 쓴다.
  - 이번 실행에서 그 값은 기록에 남지 않았다(대상 목록은 저장되지 않는다).
  - 표지 없는 같은 순서(잡 8726)에서는 이 미션이 계획됐다. 표지 장면에서는 seed 0·1 두 번 다 같은 단계에서 막혔다.
  - **분류: 정적 충돌, 충돌 대상 미확인.** 후보는 하역 팔레트의 이어받은 자세(세션 소관)와 표지 판 배치의 여유다. 실패 당시의 장면·대상 목록을 기록해 다시 돌리는 것은 다음 PR(대기열 그대로의 실행)에서 한다.
  - 원자료(ws1 `/home/projects/forklift/test_runs/live/`):
    - `ct1_srandm_a770031/live/epoch_1/run/missions/m3/records.json`(`state.planning_traces`·`async_plans`)과 `run.log`.
    - seed 1 은 `ct1_srandm_s1_a770031/…`.
    - 장면은 `test_runs/riibotics/scenes_ct1_a770031/random_m_session.json`.
- **random_m seed 1 두 번째 미션 — 다리 끝 교착(다리 끝 크리프 소관).** 하역은 2.1 cm 로 끝났다. 그 뒤 복귀 다리 끝(진행 13.196 m)에서 37 s 넘게 속도 0.012 m/s·곡률 0.36 고정, 위치 오차 0.032 m 로 버티다 시간 초과로 끝났다.

## 시험

- `tests/unit/test_session_pallets.py`:
  - `pallet_prim` 의 `-` 치환·겹침 거부·USD 경로.
  - `PalletSwitch` 의 위임·`handle`·전환 기록·운반 중 거부.
  - `RestWatch` 의 3D 위치·전체 회전각(기울어진 이웃).
- `tests/unit/test_session_runner.py`(러너 함수를 AST 로 뽑아 실행):
  - 다음 틱 머리 전환과 그 기록, 같은 팔레트 재제출 때 흔적 증거 보존.
  - 쉬는 자세 감시와 지금 외곽(id 순서 포함), 운반 중 팔레트 제외.
  - LiDAR 자기 필터 접두, 전환 대기 중 원거리 기록 금지, 장면 손상 분류.
  - 파서(세션 = live + 재작도 always, `--record-pocket-frames` 거부).
  - 근접 직선 sweep(Codex 반례).
  - 외곽 간격 기하(45° 이웃·겹침·최솟값 유지), 러너 배선.
- `tests/unit/tools/test_session_scene.py`: 표지 세션 = 표지 v1 장면의 판(정적 장애물 = 벽), 원본 불변.
- 러너 소스 가드 갱신: `test_live_records`·`test_live_runtime`·`test_failures`(세션 분기와 두 사유).
- ws1 전체(`9cd7a59` 병합 뒤 `a770031`): 5 954 통과, 실패 10 은 알려진 ws1 전용(`test_preview_docking` 6·`test_diagnose_detection` 2·`test_evaluate_cli` 1·`test_remote_model_jobs` 1).

## 회귀

- M-a1 재생 게이트 29/29 `finish ok`(`87a4c6e`, `a770031`; ws1 잡 8753).

## 검토

- 계획: Codex 적대 검토 3 회(1 → 3판), agy(Gemini 3.8 Flash) 독립 검토, 직접 재계산. 기록은 계획 7 절.
- 구현: Codex 1차 P2 4·P3 1 → 반영(`7f644f2`). 2차에서 "새 P1/P2 없음"으로 수렴했고, P3(계획의 범위 문구)는 반영했다(`78c15bd`).
- 3차(표지 세션·소스 가드·이 문서): 표지 코드는 새 P1/P2 없음. P2 는 random_m 실패를 지도 격자로 돌린 분류였다. 장면 모드에 격자 계획이 없어 반증됐고, "정적 충돌, 대상 미확인"으로 고쳤다(`bd6b85c`).
- 4차: 새 P1/P2 없음(수렴). random 연속 운반의 완주와 이 충돌의 원인 규명은 PR ④ 의 미완료 항목으로 남긴다.
- 범위 밖으로 둔 것: 세션 장면의 실은 채 재제출(`live_loaded_pallet` 이 `grid_planning` 필요), D8b truth 기록(세션에서 거부).
