# 밑판 옆면 증거 — 앞에서 똑같은 grounded 를 집지 않기 (1판)

> **실행 방식:** 이 세션에서 직접 구현하고, Codex 검토를 수렴까지 받으며 진행한다. 이 계획은 구현 전에 독립 검토 3회(직접 재계산, Codex 적대적 검토, agy 독립 검토)를 거친다.

**Goal:** 단계 3 학습 제안기의 dev 음성 오검출 13/65 는 전부 **grounded** 음성(ADR 0004 D6: 밑판 없이 블록 기둥이 바닥까지 내려간 구조)이다. 진입면에서 보면 EPAL 6 와 grounded 는 윤곽·깊이가 같아 원거리·정면으로는 가를 수 없다. 이 계획은 **옆면이 보이는 근거리 시점에서 밑판 띠를 확인한 뒤에만 집기를 허가**하는 증거 판정과 미션 단계를 만든다.

사용자 결정(2026-10-19): grounded 처리 방식으로 **"옆면 증거 확인 추가"** — "EPAL 형상 후보는 집기 전에 옆면이 보이는 각도(근거리 재관측)에서 밑판 띠를 확인한 뒤에만 확정한다."

**Spec:** 설계 `docs/design/2026-10-15-pallet-recognition-generality.md` §3.4(음성 오검출), §7.5(식별), §9.1(근거리 재관측이 원거리 보류 항목을 푼다), ADR 0004 D6(grounded 음성).

## 1. 사실 (원자료에서 다시 계산)

**왜 정면으로는 안 되는가.** 두 알려진 형상 모두 밑판이 **블록 기둥 바로 아래에만** 있고 포크 개구는 바닥까지 열려 있다(`config/pallet_geometry_epal6.yaml` "The fork openings are between the block columns and are open to the floor", `bottom_board_widths_m` = `block_widths_m`).
- 진입면에서는 기둥 하나가 "밑판 + 블록"인지 "바닥까지 내려간 블록"인지 윤곽·깊이로 구분되지 않는다.
- 렌더는 인스턴스마다 한 색이라 판과 블록 사이 이음매도 없다.
- grounded 구조는 `tools/measure_pocket_evidence.py:80–92` 정의를 쓴다(밑판 제거, 블록을 바닥까지 늘림).

**옆면에서는 갈린다.** 바깥 기둥 옆면에서, 블록 행 사이 틈의 아래쪽 띠(0 – 밑판 두께)는 다음과 같다.
- 진짜: 밑판이 전체 깊이로 이어져 그 띠가 **옆면 평면 위에 채워져** 있다.
- grounded: 그 띠가 **비어** 있어, 광선이 평면 바로 뒤 바닥이나 다음 블록 앞면에 닿는다.

| 형상 | 블록 행 사이 틈 | 밑판 두께 | 밑판 띠 화소(fx 388.8: 1.0 / 1.25 / 1.5 m) | 1.25 m·40° 에서 틈 폭 화소 |
|---|---|---|---|---|
| EPAL 6 | 82.5 mm | 22 mm | 8.6 / 6.8 / 5.7 | 16.5 |
| T11 × 0.6 | 210 mm | 15 mm | 5.8 / 4.7 / 3.9 | 42.0 |

(fx 388.8 은 팀 잡음판 `config/depth_noise_d435i_team.yaml` 의 `source_fx`.)

**실현 가능성 시험(일회용, 2026-10-19, ws1).**
- 조건: CPU 리그(`instance_rig`, 카메라 높이 0.27 m — 미션 `carriage_low_measured` 와 같은 높이), EPAL 6 진짜 vs grounded, 팀 잡음 `frame_640`, 자세 오차 위치 10 mm·yaw 1°(정규분포 σ), seed 6.
- 판정 방법:
  - 옆면 평면은 **관측된 블록 옆면 화소(z 0.03–0.09 m, 예상 평면 ±40 mm)로 다시 맞춘다**(자기 기준). 맞춤 자세만으로 띠 위치를 정하면 자세 오차 10 mm·1° 에서 구분이 무너졌다.
  - 시험 화소는 다시 맞춘 평면에서 틈 안쪽(끝에서 10 mm 안) × 띠 높이 10–20 mm 와 만나는 광선이다.
  - 증거는 그중 측정점이 **평면 위(±15 mm)·틈 안쪽(끝에서 12 mm 안)·띠 높이(측정 z ≥ 6 mm, 광선 교점 높이와 ±8 mm)** 에 있는 비율이다. 높이 조건은 바닥에 닿은 광선을, 틈 안쪽 조건은 다음 블록 앞면에 닿은 광선을 뺀다.

| 시점(거리, 진입면 기준 회전) | 진짜 증거(최소–최대) | grounded 증거 | 판정 |
|---|---|---|---|
| 1.0 m, 40° | 0.83–0.94 | 0.00–0.06 | 갈림 |
| 1.5 m, 40° | 0.65–0.96 | 0.00–0.33 | 갈림 |
| 1.5 m, 30° | 0.68–0.83 | 0.03–0.29 | 갈림 |
| 1.0 m, 30° | 0.73–0.85 | 0.05–0.73 | 겹침 |
| 20° | 0.17–0.93 | 0.00–0.77 | 겹침 |
| 15° 이하 | — | — | 시험 화소 ≤ 5 |

- 잡음·자세 오차가 없으면 30–40° 에서 진짜 ≥ 0.83, grounded 0.00 이다.
- **결론:** 진입면에서 약 40° 돌아간 시점, 1.0–1.5 m 에서 가를 수 있다. T11 은 시험하지 않았다(띠 4–6 px 로 더 얇다).

## 2. 범위

- 들어간다:
  - 증거 판정 모듈(코어, SDK 없음)
  - CPU 평가 집합과 문턱 보정·판정
  - 집기 보류 항목 `bottom_deck`
  - 미션의 옆면 확인 지점
  - Isaac 검증(진짜 EPAL 6 사이클, grounded 거부)
- 빠진다:
  - 비규격(S7) 후보: 밑판 없는 비규격 팔레트가 정상일 수 있어 이 증거를 요구하지 않는다(기존 `PARAMETRIC_PENDING` 그대로).
  - 손상(S5)으로 바깥 밑판이 빠진 팔레트: 증거 없음 → 집지 않음(안전 쪽, 기록만).
  - 실물 센서·실물 팔레트.
  - 실시간(`--live`).

## 3. 작업

### 작업 1 — 증거 판정 모듈

`src/forklift_core/perception/bottom_deck_evidence.py`(새, SDK 없음).

- `BottomDeckParams`(frozen dataclass):
  - `band_rel=(0.45, 0.9)`: 띠 높이 = 밑판 두께 × 범위. EPAL 10–20 mm, T11 6.75–13.5 mm.
  - `floor_rel=0.3`: 측정 z ≥ 밑판 두께 × 0.3(EPAL 6.6 mm, T11 4.5 mm).
  - `z_tol_m=0.008`.
  - `plane_tol_m=0.015`.
  - `gap_margin_m=0.012`.
  - `refit_band_m=0.04`, `refit_z=(0.03, 0.09)`.
  - `min_refit_points=30`, `min_valid=40`, `threshold`(작업 2 에서 dev 로 정함).
- `side_evidence(scene, candidate_pose, spec, params) -> BottomDeckEvidence`
  - 입력 `candidate_pose` 는 포켓 후보의 팔레트 중심·삽입 yaw 다.
  - `spec` 의 상자들(`spec_boxes`)에서 바깥 기둥의 블록 행 구간을 읽어 틈 구간을 만든다. EPAL·T11 상수를 하드코딩하지 않는다.
  - 카메라를 향한 옆면(±y 중 법선이 카메라 쪽)을 고른다. 블록 옆면 화소로 평면을 다시 맞추고, 시험 화소·증거 비율을 낸다.
- 결과 `BottomDeckEvidence(verdict, fraction, n_test, n_valid, view_angle_deg, range_m, side)`.
  - `verdict`: `present`(n_valid ≥ min_valid 이고 fraction ≥ threshold) / `absent`(n_valid ≥ min_valid 이고 fraction < threshold) / `unobserved`(그 밖: 평면 재맞춤 실패, 시험 화소 부족, 회전각 부족).
- 시험(먼저 실패 확인):
  - 잡음 없는 진짜 EPAL 6·T11 → `present`, grounded → `absent`(30–40°, 1.0–1.5 m).
  - 정면(0°) → `unobserved`.
  - 자세 오차 10 mm·1° 에서 판정 유지.
  - 블록 옆면이 안 보이면(가림) → `unobserved`.
  - 틈 구간이 `spec` 에서 오는지(가짜 spec 의 다른 행 배치).
  - 결정적.

### 작업 2 — CPU 평가 집합과 문턱

`tools/bottom_deck_scenes.py` 로 CPU 리그 장면을 만든다.
- 구성: 형상 EPAL 6·T11 × 구조 {pallet, grounded} × 거리 [0.9, 1.6] m × 회전 [25°, 50°](양쪽 옆면) × 팀 잡음 `frame_640` × 자세 오차(근거리 맞춤 오차 분포).
- 자세 오차 분포의 근거: 단일 프레임 자세 p95 는 1.5 m 안에서 20 mm·2° 안이다(팀 잡음판 기록 ⑳). 그래서 σ 를 위치 10 mm·yaw 1° 로 두고, 민감도로 20 mm·2° 도 낸다.
- seed 대역은 새로 정한다. dev 와 eval 은 대역이 다르다.
- 판정 대상은 **시점 범위 안**(작업 4 의 옆면 지점이 보장하는 범위: 1.0–1.5 m, 35–45°) 표본이다. 범위 밖 표본은 `unobserved` 비율만 기록한다.
- **문턱:** dev 에서 고른다. grounded `present` 를 0 으로 두는 가장 낮은 문턱 중, 진짜 `present` 비율이 가장 높은 값이다.
- **합격 기준(eval 한 번, 형상별):**
  - grounded 를 `present` 로 보는 비율의 단측 95 % 상한 ≤ 2 % — 안전 쪽, 표본 수는 이 상한에서 정한다(0 실패면 n ≥ 149).
  - 진짜를 `present` 로 보는 비율 ≥ 90 % — 놓치면 재관측을 부르는 쪽.
- 기록: `docs/validation/` 에 표(형상 × 거리 × 회전), 자세 오차 민감도, 잡음 끔/켬.

### 작업 3 — 집기 보류 항목 `bottom_deck`

- `instance_fit.FAR_PENDING` 에 알려진 형상용 `bottom_deck` 을 더한다(`PARAMETRIC_PENDING` 은 그대로).
  - 원거리 후보는 지금도 집기를 허가하지 않으므로(`instance_fit.py:32`) 원거리 판정은 바뀌지 않는다.
  - 기존 시험이 `FAR_PENDING` 내용을 고정하고 있으면 그 시험을 함께 고친다.
- 새 판정 `pick_gate(candidate, evidence) -> (allowed, reason)` 은 근거리 쪽에서 쓴다.
  - `present` 만 `bottom_deck` 을 푼다.
  - `absent` 는 사유 `no_bottom_deck` 으로 집지 않는다.
  - `unobserved` 는 사유 `bottom_deck_unobserved` 로 집지 않는다(콘솔 확인).
  - 사유 코드는 `fork_fit.REASON_CODES` 에 더한다.

### 작업 4 — 미션의 옆면 확인 지점

- **위치:** 원거리 관측이 `selected` 이고 대상이 알려진 형상일 때, 근거리 촬영 전에 옆면 지점을 하나 끼운다.
  - 지점은 팔레트 중심에서 거리 1.25 m, 진입면 법선에서 ±40° 방향이고, 카메라가 팔레트 중심을 향한다.
  - 두 쪽 중 충돌 없고(`collision_free_pose`, 지도 사각형) 경로가 있는 쪽을 고른다. 둘 다면 현재 위치에서 가까운 쪽이다.
- **촬영:** 정지 촬영(기존 `capture` 경로, 근거리와 같은 깊이 처리)이다. 원거리 융합 후보의 자세를 그 지점의 base 로 옮겨 `side_evidence` 를 돌린다. 기록은 `state["bottom_deck"]` 이다.
- **판정:**
  - `present` → 기존 근거리 정렬로 진행한다.
  - `absent` → 집지 않고 미션을 `no_bottom_deck` 으로 끝낸다(콘솔).
  - `unobserved` → 다른 쪽 지점을 한 번 시도하고, 그래도 아니면 `bottom_deck_unobserved` 로 끝낸다.
- **인자:** `--bottom-deck-check {off,on}`(기본 off — 기존 측정 실행을 그대로 재현). `near_tracking` 의 `POST_MEASUREMENT_ARGUMENTS` 가 아니라 **`RUN_SPECIFIC`** 에 둔다 — 경로와 시간을 바꾸므로 측정 조건과 다르다. 근접 추적·포켓 검사와 함께 쓰는 실행은 근접 계약 밖이라 거부한다.
- **재현 자료:** 옆면 촬영 프레임(깊이·외부 파라미터·후보 자세)을 저장한다. 재계산 도구가 같은 판정을 다시 만든다.

### 작업 5 — Isaac 검증

- **진짜 EPAL 6:** 기존 미션 구성 + `--recognition far --recognition-proposals learned`(GPU) + `--bottom-deck-check on`, seed 0–2.
  - 판정: 사이클 완료, 옆면 판정 `present`, 추가 시간 기록.
- **grounded:** 공장 배치에서 팔레트 자리에 grounded 구조(밑판 제거, 블록 바닥까지)를 놓는 장면 변형이다(`--pallet-urdf` 대신 grounded URDF 를 생성).
  - 판정: 원거리에서 고르더라도 옆면 판정 `absent` 로 집지 않고 끝남. seed 0–2.
- **대조:** 같은 seed 의 `--bottom-deck-check off`(grounded 를 집으러 들어가는 것을 기록).
- 기록: `docs/validation/2026-10-2x-bottom-deck-side-evidence.md`.

## 4. 위험

- **T11 띠가 얇다**(4–6 px). 작업 2 에서 형상별로 따로 판정하고, T11 이 미달이면 T11 은 `unobserved` 쪽으로 보수 처리한다(집지 않음).
- **옆면 지점이 막힌 공간**(사례 D: 뒤가 막힘): 두 쪽 다 막히면 집지 않는다. 운용 비용이 생긴다.
- **실물 차이:** 렌더는 판·블록이 같은 색, 모서리가 날카롭다. 실물은 판 이음매·마모·짐 걸침으로 띠가 가려질 수 있다. 이 계획은 렌더·CPU 리그에서만 보인다.
- **가정한 자세 오차**가 실제 근거리 맞춤 오차보다 작을 수 있다. 민감도(20 mm·2°)로 본다.
- **ADR 0004 D6 와의 관계:** grounded 를 계속 음성으로 둔다(사용자 결정). 다리형 플라스틱 팔레트처럼 밑판 없는 실물은 이 판정으로 집지 않는다.

## 5. 자체 검토 기록

- 직접 재계산(1회차): 틈 폭·띠 두께·화소 수를 `config/pallet_geometry_{epal6,t11_06}.yaml` 에서 다시 계산했다(위 표). 실현 가능성 표는 ws1 일회용 스크립트의 출력 그대로다. 카메라 높이 0.27 m 는 `perception_adapter.py:77` 과 `instance_rig.PERCEPTION_CAMERA` 에서 확인했다. 기울기 0.10 rad 는 시험에 넣지 않았다.
- Codex 적대적 검토: (대기)
- agy 독립 검토: (대기)
