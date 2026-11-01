# 팔레트 질량을 시험 출력물 예상 무게로 (2026-11-01, 1 판)

- 작성: 2026-11-01, 시뮬-물성 세션. 기준 origin/main `f594dc1a`.
- 지시: 사용자(2026-11-01, 총괄 경유) — Isaac 팔레트 질량을 완구 카탈로그 적재 한계 10 kg 대신 우리 시험 출력물 예상 무게로 바꾼다. 운영 기본을 바꾸기 전에 10 kg 과 새 값을 비교하는 작은 Isaac 배치로 성공률·미끄럼 판정·삽입 결과가 깨지지 않는지 본다.
- 결정 범위: 형상별 값과 적용 방식은 임시값·런처 기본값이라 문서화 후 진행한다(변경 승인 정책). 실제 무게는 팀 실측 요청 후보다(§6).

## 1. 값

| 형상(`geometry_version`) | 질량 | 근거 |
|---|---|---|
| `t11_06` (T11 × 0.6, 660 × 660 × 90) | **2.49 kg** | ADR 0002 §4 의 PLA 5 % 인필·벽 0.8 mm 추정값 |
| `t11_06_d600` (얕은 T11 시험체) | **2.49 kg** | 위와 같은 값을 쓴다(총괄 지시 "T11 × 0.6 류") |
| `epal6` (800 × 600 × 144, 실물 크기) | **4.93 kg** | 같은 인필·벽 조건으로 체적에서 계산(아래) |

모두 **추정값이고 실측이 아니다.** 시뮬은 이 질량을 팔레트 상자들에 부피 비례로 나눈다(`sim/isaac/scene.py` `import_pallet_usd`). 출력물은 벽이 무거운 껍질 구조라 실제 관성은 다르다. 관성은 여전히 합성값이다.

### 계산 방법

ADR 0002 는 2.49 kg 의 계산식을 남기지 않았다. ADR 의 5 %·10 % 두 값(2.49·3.20 kg)과 실체적 12.898 L 을 거꾸로 풀면, **부재마다 따로 인쇄해 모든 면에 0.8 mm 껍질을 두고 나머지를 인필로 채운 무게**와 맞는다(PLA 밀도 1.24 g/cm³, `tools/export_pallet_stl.py:35`). 내부 약 11.5 L, 껍질 약 1.43 L(면적 약 1.79 m² × 0.8 mm)이다. 이 방법을 이 판의 형상 파일(`tools/recognition_scenes.known_spec` → `spec_boxes`)에 그대로 쓰면:

| 형상 | 실체적 | 부재 표면적 | 5 % | 10 % |
|---|---|---|---|---|
| `epal6` | 33.156 L | 3.050 m² | **4.93 kg** | 6.83 kg |
| `t11_06` | 13.061 L | 1.930 m² | 2.63 kg | 3.34 kg |
| `t11_06_d600` | 12.326 L | 1.814 m² | 2.47 kg | 3.15 kg |

질량 = 1.24 g/cm³ × (표면적 × 0.8 mm + 0.05 × (실체적 − 표면적 × 0.8 mm)). Codex 가 같은 수를 독립으로 다시 냈다(4.930 / 2.628 / 2.473 kg).

- T11 × 0.6 은 지금 형상 파일로 계산하면 2.63 kg 이다. ADR 의 12.898 L 은 이전 상판 배치의 체적이라 차이(약 5 %)가 난다. 총괄 지시대로 ADR 값 2.49 kg 을 그대로 쓰고, 이 차이는 추정 불확실성 안으로 본다. ADR 의 계산 방법을 확정한 것은 아니다.
- EPAL 6 에 ADR 값과의 비율(2.49 / 2.63)을 곱해 4.67 kg 으로 맞추지 않는다. 계산식이 남지 않은 값에서 옮겨 쓸 보정 계수가 나오지 않는다(Codex P3).
- 인쇄 방식이 값을 바꾼다. 같은 방법을 내보내기 도구의 기본 256 mm 타일 분할에 쓰면 EPAL 6 는 4.09 kg, 베드보다 긴 부재를 자른 부재 분할이면 5.07 kg 이다. 4.93 kg 은 부재를 자르지 않은 이상화다.
- 밑판 없는 변형(`*_grounded`, `--pallet-article grounded`)은 같은 형상 파일을 쓰므로 그 형상 값을 그대로 받는다. 상자가 적은 만큼 상자당 질량이 커진다. 인쇄할 시험체가 아니어서 따로 값을 두지 않는다.
- 동봉 완구 팔레트는 아직 형상 파일이 없다. 생기면 표에 값을 더해야 그 형상으로 실행할 수 있다(아래 §2 의 거부).

## 2. 적용 방식

- `config/isaac_transport_measured.yaml`·`_measured_tyre.yaml` 의 `pallet_mass_kg` 를 **형상별 표**로 바꾼다: `{epal6: 4.93, t11_06: 2.49, t11_06_d600: 2.49}`. 운영 런처 셋(`isaac_live`·`riib_scene`·`slam_survey`)과 T11 실행이 모두 이 한 설정 파일을 쓰기 때문에 형상별로 나눠야 한다.
- 잠정 모델 설정 `isaac_transport.yaml`·`_fast.yaml` 은 10.0 그대로 둔다. 잠정 모델 진입점(`run_perception_approach`·`run_pocket_insertion`)과 옛 기준선 재현용이고, 그 파일들을 내용으로 묶은 기록이 깨지지 않게 하려는 것이다.
- 형상은 **로드한 기하의 `geometry_version`** 으로 고른다. 파일 이름이 아니다(Codex P2-1: T11 내용을 `pallet_geometry_epal6.yaml` 이름으로 두면 파일 이름 규칙은 4.93 kg 을 고른다).
- `sim/isaac/pallet_mass.py` 의 `resolve_pallet_mass` 가 Isaac 없이 표를 그 실행의 숫자로 바꾼다. 세 진입점이 모두 쓴다(Codex P2-2).
  - `run_transport.py` 는 `validate_before_isaac` 에서 바꾼다. 표에 그 형상이 없으면 Isaac 전에 거부하고, `--check-args` 사전 검사도 같은 이유로 거부한다.
  - `run_pocket_insertion.py` 는 자기 EPAL 6 기하의 버전으로 바꾼다.
  - `run_perception_approach.py` 는 기하를 읽지 않으므로 숫자 설정만 받는다. 표를 주면 이유를 말하고 거부한다.
- 바꾼 숫자가 `settings_synthetic` 에 들어가므로 설정을 읽는 쪽은 계속 숫자 하나를 본다. 표와 고른 형상은 결과의 `pallet_mass` 기록(`kg`·`shape`·`source`·`settings_value`)에 남는다. 숫자 설정은 읽은 그대로 기록된다(옛 실행과 같은 기록).
- `config/site_load_defaults.yaml`(현장 기본 적재 선언)도 같은 형상별 값으로 바꾼다. 주석의 "시뮬 설정과 같다"는 실측 차체 설정의 표와 같다는 뜻으로 고쳤다.
- 적재 승강 유지 오차(`loaded_lift_mass_kg`)는 설정·현장 표·엔진 질량 중 최댓값을 쓴다. 셋 다 새 값이 되므로 EPAL 6 정적 처짐 추정이 28.45 mm 에서 16.02 mm, T11 은 10.03 mm 로 준다(Codex 계산, 실측 URDF 포크 질량 기준). 실제 유지 동작은 §4 배치로 본다.

## 3. 내용 해시로 묶인 계약에 생기는 일

| 묶인 곳 | 이 변경 뒤 |
|---|---|
| `config/near_field_drive_terms.json`, `_riib_line_1_m.json` 의 조건(`argument settings` = 옛 `isaac_transport_measured.yaml`, riib 쪽은 `config/site_load_defaults.yaml` 도) | 새 설정으로 `--near-tracking` 을 켜면 `condition_problems` 가 거부한다. 10 kg 에서 잰 측정이므로 맞는 동작이다. 다시 내보내지 않는다. 운영 런처 기본은 근접 추적을 켜지 않는다(riib `NEAR=on` 은 근접 재관측이다). |
| 근접 추적 재측정을 이어 갈 세션(`docs/plans/2026-10-27-insertion-lateral-drift.md` (B)) | 설정 파일 사본만으로는 riib 조건을 못 지킨다 — 실행기가 현장 표를 자기 체크아웃의 고정 경로에서 읽는다(Codex P2-3). 두 파일이 다 10 kg 인 **옛 스냅숏에서** 돌리거나, 새 질량 조건으로 다시 잰다. |
| `tools/reloc_judge.py` 지문 | 동결할 때만 쓴다. 새 동결은 새 지문을 받는다. 러너가 바뀔 때마다 생기는 일과 같다. |
| SLAM 지도 묶음(`slam_map.SHARED_INPUTS`) | 설정을 대조하지 않는다. 영향 없음. |
| 재생 게이트 runs29 | 재생 도구가 기록 당시 설정 파일과 바이트가 같은지 먼저 본다(`boundary_replay.build_inputs`). 바뀐 `isaac_transport_measured.yaml` 그대로는 29 건 모두 `setup input files differ` 로 거부된다(`pm_275031ea`, 잡 13847) — 물성이 바뀐 실행은 정의상 재생할 수 없다. 그래서 게이트는 **세 설정 파일만 main 바이트로 되돌린 스냅숏(비교 배치 A 팔과 같은 것)** 에서 돌려 코드 변경만 본다. 해석 함수와 새 물성은 단위 시험·사전 검사 시험과 §4 배치로 본다. |

`config/isaac_slam_lidar.yaml` 주석처럼 위 파일을 주석 때문에 고치지 않는 것과 같은 이유로, 이 변경은 질량이 실제로 바뀌는 곳에서만 해시를 바꾼다.

## 4. 비교 배치 등록 (결과 전에 정함)

### 팔

- A: 10 kg. 이 브랜치에서 `isaac_transport_measured.yaml`·`_measured_tyre.yaml`·`site_load_defaults.yaml` 세 파일만 main 의 바이트로 되돌린 스냅숏.
- B: 새 값. 이 브랜치 그대로의 스냅숏.
- 두 팔은 코드가 같고 세 설정 파일만 다르다. 같은 묶음 방식(`tools/isaac_bundle.py`, 두 팔을 한 목록에 seed 별로 번갈아), `VIEWS=none`, #181 사전 검사를 지킨다. 결과 기록의 `pallet_mass.kg` 로 팔을 확인하고, 10 kg 기준선과 섞지 않는다.

### 실행 목록 (40)

| 묶음 | 내용 | 팔당 | 합 |
|---|---|---|---|
| riib | `riib_scene.sbatch` 런처 기본, 장면 line_2·random_1·aisle_rack·trailer × seed 0–2 | 12 | 24 |
| 사이클 EPAL 6 | 공장 홀 운반 사이클(원거리 인식·SLAM 피드백·복귀, 2026-10-22 얕은 T11 기록의 `bd.sbatch` 구성), seed 0–2 | 3 | 6 |
| 사이클 T11 | 같은 구성에 `t11_06_d600` 시험체·0.29 m 간격 장면(`base_scene_0p29`)과 그 URDF, seed 0–2 | 3 | 6 |
| live 적재 정지 | `isaac_live.sbatch` line_m 세션 + 콘솔 탐침 `--scenario cycle --cycle P-1:D-1 --stop-wait-at transport:5 --stop-wait-sim-s 30`(U30: 적재 운반 중 STOP, 30 s 서 있기, 재무장, 취소), seed 0–1 | 2 | 4 |

T11 사이클과 live 적재 정지는 Codex P3 를 반영했다. 운영 런처는 EPAL 6 만 고르므로 riib 만으로는 T11 값을 한 번도 쓰지 않고, 끊김 없는 자율 사이클은 적재 유지 오차가 쓰이는 STOP·재무장 판정을 지나지 않는다. **live 적재 정지는 적재 상태의 재제출 출발 판정(`loaded_wait`, 적재 승강 띠)은 지나지 않는다** — 이 배치는 그것을 보이지 않는다(Codex 2 차 P3).

### 측정 원천 (결과 전 고정)

배치 실행은 `run/result.json`, live 는 `probe.json` 과 live 기록을 읽는다.

| 측정 | 원천 |
|---|---|
| 완주 | `success` 참 그리고 `phase == "complete"` |
| 끝 사유 | `failure_reason` |
| 안전 실패 | `failure_reason` 이 다음 중 하나로 시작: `Actual truck/load footprint overlap`, `Measured pallet footprint overlap`, `Forbidden fork/pallet contact`, `Excessive body tilt`, `Pallet slipped off forks`, `Pallet dropped during transport`. 그리고 `forbidden_pocket_contacts` 가 비어 있지 않은 실행 |
| 미끄럼 실패 | `slip_watch.failure` 가 있는 실행 |
| 들기 도달 | `lift_tilt_peak` 가 있는 실행 |
| 포크 도달 깊이 | `insertion_measured.left/right.insertion_m`, `beyond_centre_m` |
| 삽입 중 최소 가로 틈 | `lateral_clearance_min_m.insert` 의 좌우 최솟값 |
| 들기 기울기 | `lift_tilt_peak.pallet_tilt_rad` |
| 삽입으로 밀린 거리 | `lift_tilt_peak.pallet_position_m` 과 `initial_pallet_m` 의 수평 거리(들기 기울기 최대 순간의 팔레트 위치, 삽입 끝 자세의 대리값) |
| 포크 위 상대 이동 | `slip_watch.outline_motion_max_m`, `pocket_gap_min_m` |

`collision_counts` 는 실행 중 충돌 수가 아니라 장면의 충돌체 수라 쓰지 않는다(Codex 2 차 P2-2). 기록이 없는 측정은 깨끗한 관측으로 세지 않고 결측으로 센다.

### 유효성

- 인프라 무효는 riib 배치 2 등록(`docs/plans/2026-10-31-riib-default-batch2.md` §4)의 허용 목록만 쓴다. 나머지 끝은 그 팔의 결과다.
- 깨진 짝(같은 묶음·장면·seed 의 두 팔 중 하나 이상 무효)은 두 팔을 함께 한 번 다시 돈다. 그래도 깨지면 결측이다.
- 안전 실패는 무효로 분류된 실행도 관측된 그대로 센다.
- **최소 유효 짝:** riib 12 짝 중 10, 사이클은 형상마다 3 짝 모두, live 2 짝 모두. 모자라면 판정 불가.
- **단계 도달(기준선이 시험할 거리를 만들었는가):** A 팔이 riib 에서 8 실행 이상, 사이클 형상마다 2 실행 이상 들기에 도달해야 한다. 모자라면 판정 불가. 미끄럼·삽입 측정은 두 팔 모두 들기에 도달한 짝만 비교한다.
- **판정 불가면 운영 기본을 바꾸지 않는다.** 원인과 함께 총괄에 올린다.

### 판정

- **완주:** riib 유효 짝 합 B ≥ A − 1. 사이클은 형상마다 B ≥ A − 1.
- **안전:** B 의 안전 실패 0. A 에 안전 실패가 있었으면 B 는 그 수를 넘지 않고, 그 짝을 따로 적는다.
- **미끄럼:** 미끄럼 실패 B ≤ A.
- **새 끝:** B 에만 나온 `failure_reason` 은 원인을 댄다. 질량과 무관함을 보이지 못하면 불합격이다.
- **삽입·들기:** 포크 도달 깊이, 최소 가로 틈, 들기 기울기, 밀린 거리, 포크 위 상대 이동을 팔별로 적는다. 문턱이 없는 서술 비교이고 수치 비열등 판정이 아니다(Codex 2 차 P3).
- **live 적재 정지:** `probe.json` 의 첫 임무 `stop_wait` 에서 두 팔 모두
  - `phase_reached` 참(운반 단계 도달), `stop_reply.t == "ack"`, `end_reason == "sim_time"`, `end_sim_s − start_sim_s ≥ 30`, `mode_at_end == "STOPPED"`,
  - `rearm` 성공 뒤 PAUSED 관측(`paused_after_rearm` 대기가 시간 초과 없이 끝남),
  - `after_cancel_statuses` 가 `cancelled` 또는 `recovery_required` 로 끝남,
  - 그 세션 live 기록에 장면 손상(`scene_damaged`)이 없음.
  - A·B 의 끝 상태가 다르면 원인을 댄다. 세션 `success` 는 실패한 임무가 있어도 참일 수 있어 쓰지 않는다(Codex 2 차 P2-4).
- 위를 모두 넘으면 운영 기본을 바꾸는 이 PR 을 머지한다. 하나라도 깨지면 머지하지 않고, 원인과 함께 총괄에 올린다. 더 가벼운 팔레트가 더 사실에 가까운 물성일 수 있으므로, 깨진 것을 "새 값이 틀렸다"로 읽지 않고 무엇이 그 질량에 기대고 있었는지를 먼저 찾는다.
- 결과를 본 뒤 판정식을 고치거나 실행을 무효로 돌리지 않는다.

## 5. 이 변경이 보이지 않는 것

- 실제 출력물 무게. 추정값이다.
- 출력물의 관성·무게중심. 부피 비례 분배의 합성값이다.
- 접촉 재질. 지금도 하나의 재질(μs 0.8·μd 0.7)을 모든 충돌체에 쓴다. 재질 분리는 2 단계 계획에서 다룬다.
- 차체 모델의 질량·승강 한계 대조(총괄 감사 (가)). 장면 일치 검사가 질량·관성·승강 이동 한계를 보지 않는 빈틈은 별도 PR 로 다룬다.

## 6. 팀 실측 요청 후보 (총괄 전달)

- 출력물 실제 무게(형상별).
- 포크 위 팔레트 미끄럼 각도(μs = tan θ), 포크 끝 처짐, 앞뒤 바퀴 하중(무게중심) — 2 단계 재질 계획과 같이 쓴다.

## 7. 자체 검토

- 원자료 재계산: 형상 파일에서 실체적·부재 표면적을 다시 뽑아 위 표를 냈다. ADR 의 5 %·10 % 두 값에서 거꾸로 푼 껍질·내부 체적이 부재별 껍질 모형과 맞는 것을 확인했다.
- Codex 설계 검토 1 차: P1 없음, P2 3 건 — ① 파일 이름 대신 `geometry_version` ② `run_pocket_insertion` 이 설정을 따로 읽어 표를 넘기면 `scene.py` 질량 분배에서 `TypeError` ③ 근접 추적 10 kg 유지에는 설정 사본만으로 부족(현장 표 고정 경로). 셋 다 소스로 확인해 반영했다(§2, §3). P3 2 건(EPAL 값의 보정 계수 없음, 배치에 T11·live 적재 정지 포함)도 반영했다.
- Codex 2 차(구현·등록): P1 없음, P2 4 건 — ① 해석 결과(`args.pallet_mass_record`)가 기록 인자로 새어 숫자 설정에서도 근접 추적 인자 계약이 깨짐: 기록 인자에서 뺐다(시험 추가) ② `collision_counts` 는 충돌체 수: 안전 실패를 `failure_reason` 문자열로 정의 ③ 최소 유효 짝·단계 도달 없이 0 ≥ 0 − 1 로 통과 가능: 최소 짝·A 팔 들기 도달·판정 불가 규칙 추가 ④ `final_pallet_m` 은 배달 뒤 값, `insertion_error` 는 경로 끝 오차, live 세션 `success` 는 임무 실패를 숨김: 측정 원천 표와 live `probe.json` 조건으로 고정. P3(재제출 출발 판정 미검증, 삽입 비교는 서술)도 반영했다.
