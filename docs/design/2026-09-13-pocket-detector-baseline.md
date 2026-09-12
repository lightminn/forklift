# 팔레트·포켓 위치 추정 기준선과 평가기 (M2) 설계

작성일: 2026-09-13. 상태: **v2 승인(2026-09-13).** Codex 교차검증을 반영한 뒤 사용자가 §8의 결정 4개를 권장안대로 승인했다(가림은 `invalid`, 형상 prior는 파일에서 명시 주입·기본값 없음, σ는 v1에서 `None`, 구현은 §10의 2단계). 로드맵 M2([로드맵](../plans/2026-09-11-development-roadmap.md) §3 M2·§6)와 2026-09-13 사용자 확인(기하 기반 기준선 먼저, 초기 목표 p95 위치 ≤ 20 mm·yaw ≤ 2°·양성 검출률 ≥ 95 %, dev로만 튜닝·eval은 최종 보고, 첫 시연물은 포켓 경계·좌표 영상 + 장면별 오차표)을 구현 가능한 모듈로 구체화한다. 실물 센서·추적(M3)·주행은 범위가 아니다.

**이 기준선의 성격:** 합성 카탈로그의 팔레트 형상을 **사전에 아는** 기하 기준선이다. 형상 prior는 명시적으로 주입하며 실물 입력에 자동으로 적용하지 않는다.

## 1. 목표와 완료 조건

- **인식기:** `SceneInput`(RGB·depth·내부 보정·base←optical 변환·시각)과 명시적 형상 prior만 받아 `PocketObservation`(`base_link`)을 돌려주는 순수 Python 함수. 팔레트가 없거나 관측이 불충분하면 예외가 아니라 `no_pallet`/`invalid` 상태와 사유를 낸다.
- **평가기:** split을 골라 인식기를 돌리고 [관측 계약](../interfaces/pocket-observation.md)의 평가 규약으로 지표 JSON·장면별 CSV·정답과 추정을 겹쳐 그린 영상을 만든다.
- **완료 조건(M2):** 고정 eval 30장면에서 위치·yaw 오차 분포, 검출률·오검출, 결측률·처리 지연, 실패 영상이 기록되고, dev 70장면으로 튜닝한 기준선이 초기 목표에 도달하는지를 **있는 그대로** 보고한다. 목표 미달도 완료 조건 위반이 아니라 측정 결과다.
- **보장하지 않는 것:** 실물 D435i 잡음·정합, 실제 팔레트 치수, 삽입 여유. 합성 세트의 지표는 알고리즘 논리와 데이터 파이프라인의 검증이지 실물 성능이 아니다.

## 2. 데이터 특성 조사 (2026-09-13, catalogue v1 100장면)

전체 세트를 읽어 측정한 **데이터 특성 조사**다. 여기서 얻은 값으로 문턱을 정하므로, 이후 eval 결과를 보고 문턱을 다시 바꾸면 그 eval은 "미사용 최종 평가"가 아니다(§5의 dev/eval 규칙).

| 관측 | 값 | 설계에 주는 의미 |
|---|---|---|
| 전면 판 점의 평면 오차 | 중앙값 0.1 mm, p95 1.9 mm, 최대 3.1 mm(240점) | 전면을 평면으로 적합할 수 있다 |
| 개구부 사각형(경계 2 cm·높이 ±8 cm 안쪽) 안의 점 | 100 %가 평면에서 5 cm 이상 벗어남 | 개구부는 평면 위 "점 없음" 구간. **경계를 제외했을 때만 성립**(아래 덱 반례) |
| 개구부 중심 depth − 판 depth | 최소 −0.63 m(가림), 중앙값 +0.31 m | 개구부 중심 픽셀 하나를 포켓 위치로 쓰면 안 된다 |
| 전면 판 대역 점 수 | 장면당 2 446–24 552 | 최대 거리(3.1 m)에서도 수천 점 |
| 투영된 개구부 폭 | 26.1–128.4 px(중앙값 47.6, n=160) | 개구부 경계는 픽셀 몇 개 수준. base 미터 격자가 픽셀 격자보다 안정적 |
| 유효 점 중 바닥 비율 | 중앙값 88.9 % | 바닥 제거가 첫 단계 |
| 카메라→판 거리, \|yaw\| | 1.10–3.09 m, ≤ 0.52 rad | 근접(< 1.1 m)은 M3 범위 |

**반례 1 — 덱 윗면 점(측정):** z 대역을 [덱 두께, 높이 − 덱 두께] = [0.05, 0.25]로 잡으면 **양성 60장면 전부에서 두 개구 구간을 얻지 못한다**. 카메라가 내려다보므로 아래 덱의 **윗면**이 보이고, 그 점들이 z ≈ 0.0501–0.0514로 대역 하단에 들어와 개구부 열을 채운다(s003 기준 왼쪽 개구부 안에 268점). 대역을 [0.06, 0.24]로 1 cm씩 좁히면 **60/60 성공**한다. 따라서 z 대역에 경계 여유(`band_margin_m`, 기본 0.01)를 둔다.

**반례 2 — 탐색 창(측정):** 빈 구간 탐색을 팔레트 폭보다 넓은 창에서 하면 팔레트 **바깥**의 빈 공간이 개구부 크기의 가짜 구간 두 개로 잡힌다(60장면 전부 재현). 탐색은 그 대역의 최좌·최우 **점유 열 사이**로 제한한다.

**반례 3 — 점유 지도는 가림과 열림을 구별하지 못한다(측정):** 개구부 사각형을 지나는 광선을 분류하면 두 경우가 뚜렷이 갈린다.

| 개구부 | 표본 | 평면 **앞** 반환 비율 | 평면 **뒤** 반환 비율 |
|---|---:|---|---|
| 양성 장면 | 120 | p50 0.00, 최대 0.00 | p50 0.94, 최소 0.67 |
| 가림 장면의 안 가려진 쪽 | 20 | p50 0.00, 최대 0.00 | p50 0.95, 최소 0.76 |
| 가림 장면의 가려진 쪽 | 20 | p50 0.79, 최대 1.00 | p50 0.17, 최소 0.00 |

평면 점유 지도에서는 두 경우 모두 "빈 열"이라 구별되지 않는다. **개구부 판정은 광선 단위로 해야 한다**(§4.2-5).

**타당성 상한(버린 탐침):** 정답 평면을 안다고 가정하고 1 cm 격자 빈 구간만으로 포켓 중심을 구하면 오차가 양성 p50 4.7 · p95 6.5 mm, 가림 p50 5.6 · p95 8.2 mm였다. 실제 인식기는 평면 적합 오차가 더해지므로 이는 상한이다. 목표 20 mm가 **원리적으로** 가능함을 보일 뿐 달성을 뜻하지 않는다.

## 3. 대안 비교

| 접근 | 판단 |
|---|---|
| **A. depth 기하: base 점군 → 바닥 제거 → 수직 평면 후보 → 평면 위 점유 격자로 개구부 후보 → 광선 단위 검증 → 포켓 중심·yaw·폭** | **채택.** 합성 색과 무관하고 실물에서도 depth 기하가 우선 정보. 실패 이유를 상태로 표현하기 쉽다 |
| B. RGB 색·에지 기반 | 기각. 합성 단색은 실물과 무관. RGB는 시각화에만 |
| C. 학습 기반 | 보류. 기하 기준선의 실패 사례가 모인 뒤 판단 |

## 4. 인식기 — `src/forklift_core/perception/pocket_detector.py`

### 4.1 입력·출력·설정

- 입력: `detect_pockets(scene_input: SceneInput, prior: PalletPrior, params: DetectorParams) -> PocketObservation`. 정답·카탈로그·장면 범주는 받지 않는다(평가기만 `SceneSample`을 소유).
- 출력: `frame_id "base_link"`, `stamp_ns`·`clock_domain`·`source_provenance`는 입력 그대로.
- **`PalletPrior`(형상, 명시 주입):** `height_m 0.30`, `deck_m 0.05`, `opening_height_m 0.20`, `opening_width_range [0.18, 0.30]`, `centre_spacer_range [0.08, 0.12]`, `overall_width_m 0.8`. `config/pallet_prior_v1.yaml`에서만 읽고 **dataclass 기본값을 두지 않는다**(실물 입력에 합성 치수가 조용히 적용되는 것을 막는다). 파일에 `source_provenance: synthetic`, `catalogue_version: v1`을 적고 실행 결과에 파일 해시를 남긴다. `height_m == 2*deck_m + opening_height_m` 관계와 미지 키를 검증한다. **출력 포켓 중심의 z = deck_m + opening_height_m/2 는 측정값이 아니라 prior 적용 결과**임을 문서와 결과 JSON에 표시한다.
- **`DetectorParams`(알고리즘, 형상과 분리):** `cell_m 0.01`, `plane_inlier_m 0.02`, `band_margin_m 0.01`, `ransac_iterations`, `min_plane_points`, `max_plane_candidates 3`, `range_m [0.8, 5.0]`(**카메라 원점에서 점까지의 수평 거리**), `floor_z_m 0.02`, `occluded_front_frac 0.5`, `open_behind_frac 0.3`, `front_margin_m 0.05`, `seed`. dev 70장면으로만 조정하고 값을 검증 기록에 남긴다.

### 4.2 알고리즘

1. **역투영:** `deproject_depth_pixels(depth_m, 전 픽셀, intrinsics, meters_per_unit=1.0, pixel_frame=input.pixel_frame, rectified=True)` → `base_from_optical.apply` → base 점군. 픽셀 인덱스와의 대응을 유지한다(5단계에서 필요). NaN 행은 "미관측"으로 남긴다.
2. **바닥·범위 필터:** z ≤ `floor_z_m` 제거, z > `height_m + 0.10` 제거, 카메라 수평 거리 ∈ `range_m`. (바닥 z=0 가정은 현재 합성 장착 조건에 한정하며 문서에 명시한다.)
3. **수직 평면 후보:** RANSAC으로 `|n_z| < 0.1` 평면을 최대 `max_plane_candidates`개 뽑는다. 각 후보를 **정확히 수직(n_z = 0)으로 재적합**하고 잔차 p95를 기록한다(기울어진 법선에 (ℓ, z) 격자를 붙이면 수직 개구부 계약과 어긋난다). 바깥 법선은 카메라 위치 c에 대해 `n_out·(c − p₀) > 0`인 방향으로 고정한다.
4. **개구부 후보(점유 격자):** 평면 inlier를 (ℓ, z) 격자(`cell_m`)에 넣고, z ∈ [`deck_m + band_margin_m`, `height_m − deck_m − band_margin_m`] 대역에서 열별 점유를 본다(반례 1). 그 대역의 **최좌·최우 점유 열 사이**에서만 빈 구간을 찾는다(반례 2). 구간 폭이 `opening_width_range`(격자 이산화로 최대 두 셀 작게 나오므로 허용치에 반영) 안이고, 두 구간 사이 점유 폭이 `centre_spacer_range`이며, 바깥에 지지대 점유가 있고, 위·아래 덱 대역에 점유가 있어야 "덱–지지대–개구–지지대–개구–지지대–덱" 패턴이다. 보고 폭은 구간 경계에 반 셀씩 더해 보정한다.
5. **광선 단위 개구부 검증(반례 3):** 각 개구부 후보 사각형을 지나는 픽셀 광선을 평면과 교차시켜, 관측점이 평면보다 `front_margin_m` 이상 **앞**이면 `front`, 그만큼 **뒤**면 `behind`, NaN이면 `unknown`으로 센다. `front` 비율 > `occluded_front_frac`이면 그 개구부는 **가림**, `behind` 비율 ≥ `open_behind_frac`이면 **열림**, 둘 다 아니면 **불확실**이다. 전역 "앞쪽 점 개수"는 쓰지 않는다(무관한 상자가 섞인다).
6. **평면 채택:** 패턴 점수(지지대·덱 점유량, 광선 검증 결과)로 후보 순위를 매기되, 지지대·덱의 **최소 관측량 문턱**을 넘어야 채택한다. 점수만으로 자동 채택하지 않는다. 후보가 여럿이면 최고 점수, 동점이면 가까운 것을 고르고 나머지는 결과에 기록한다.
7. **출력:** 포켓 중심 = 개구 구간의 ℓ 중앙, z = `deck_m + opening_height_m/2`(prior 적용), 평면 위. 폭 = 보정 구간 폭, 높이 = prior. 방향은 `a = −n_out`, `ψ = wrap(atan2(n_out_y, n_out_x) + π)`를 (−π, π]로. **3단계에서 이미 바깥 법선으로 고정했으므로 여기서 다시 부호를 뒤집지 않는다**(이중 반전은 삽입 방향을 180° 뒤집는다). 좌우는 관측 계약의 ℓ 내적 규칙.
8. **상태 판정(순서):** 부분 구조 후보 생성 → 치수·가시성 검사 → 상태 결정.
   - 유효 점이 `min_plane_points` 미만이거나 전부 NaN → `invalid`(`insufficient_points`).
   - 수직 평면 후보 없음 → `no_pallet`(`no_front_plane`).
   - 어느 후보에서도 부분 패턴조차 없음 → `no_pallet`(`no_opening_pattern`).
   - 두 개구부 모두 **열림** → `valid`.
   - 한쪽이 **가림** 또는 **불확실** → `invalid`(`pocket_occluded:<side>` / `pocket_ambiguous:<side>`). 대칭 추정으로 채우지 않는다.
   - 개구 폭이 prior 범위 밖이거나 두 폭이 20 % 이상 다름 → `invalid`(`opening_width_mismatch`).
9. **불확실성:** v1은 `position_sigma_m`·`yaw_sigma_rad`를 **`None`(미상)** 으로 둔다. 계약상 σ는 1σ 상한이며, 현재 오차 모델(평면 적합 편향·경계 추정·비스듬한 투영·prior 오차)이 검증되지 않았다. 대신 평면 잔차 p95, inlier 수, 개구 경계 분해능, 광선 분류 비율을 **평가 진단**으로 결과 JSON에 남기고, 오차 모델을 정의·검증한 뒤 σ를 채운다.

### 4.3 성능·결정론

순수 numpy. 목표 처리 시간은 노트북에서 장면당 < 0.5 s(측정해 보고). RANSAC 난수는 `numpy.random.default_rng(params.seed)`로 고정하고 seed를 결과에 남긴다.

## 5. 평가기 — `src/forklift_core/perception/evaluation.py` + `tools/evaluate_pocket_detector.py`

**판정표**(범주별 전체 장면 수를 분모로 유지):

| 장면 범주 | 추정 | 결과 |
|---|---|---|
| positive / occluded (각각) | `valid`, 문턱 이내 | `true_positive` |
| positive / occluded (각각) | `valid`, 문턱 초과 | `wrong_pose` (검출 실패로 세고 별도 기록) |
| positive / occluded (각각) | `no_pallet` | `false_negative` |
| positive / occluded (각각) | `invalid` | `invalid` |
| 음성 각 범주 | `valid` | `false_positive` |
| 음성 각 범주 | `no_pallet` | `true_negative` |
| 음성 각 범주 | `invalid` | `invalid` |

- **검출 대응 허용 문턱**(정확도 목표와 별개, dev에서 정하고 고정): 위치 오차 ≤ 0.20 m, `abs(yaw_difference_rad)` ≤ 0.35 rad. 삽입 허용 오차가 아니다.
- yaw 판정·분위수는 **절댓값**을 쓴다(부호 있는 차이를 `≤ 0.35`로 검사하면 −1 rad도 통과한다).
- **주 오차 분포는 모든 `valid` 출력**(즉 `true_positive` + `wrong_pose`)에서 계산한다. `true_positive`만의 분포는 보조 지표로 병기한다. `invalid`·`no_pallet`에는 가짜 오차를 넣지 않고 개수만 센다.
- 표본이 0개인 분위수는 `null`이며 목표 통과로 처리하지 않는다.
- `summarize`는 범주별 개수·검출률(양성·가림 분모 분리)·오검출률·invalid 수·위치/yaw 오차 p50/p95/max·처리 시간·목표 도달 여부를 낸다. **eval 양성은 18장면이므로 ≥ 95 %는 18/18을 뜻한다**(17/18 = 94.4 %).
- CLI `tools/evaluate_pocket_detector.py --dataset … --split dev|eval --prior … --params … --output artifacts/<UTC>_pocket_eval_<split>_NN/ [--video]`: `metrics.json`, `scenes.csv`, `overlay/sNNN.png`(RGB 위 정답·추정 사각형과 상태), `--video`면 PNG를 5 fps MP4로 묶는다(**독립 장면 모음이며 연속 관측이 아님을 영상에 표시**). 결과에 dataset manifest 해시·prior 해시·params·seed·revision을 기록한다.
- **dev/eval 규칙:** 파라미터·문턱 조정은 `--split dev`에서만 한다. eval은 확정 후 한 번 실행해 기록한다. §2의 데이터 특성 조사로 문턱을 정한 사실도 함께 적는다.

## 6. 데이터 흐름

```text
data/synthetic_scenes/catalogue_v1 (Git 밖)   config/pallet_prior_v1.yaml
  → load_scene_sample → sample.input ─┬→ detect_pockets(input, prior, params) → PocketObservation
       └─ sample.ground_truth ────────┴→ evaluate_scene → SceneResult → summarize
                                          → metrics.json / scenes.csv / overlay PNG / MP4
```

## 7. 오류 처리와 경계

- 입력 형식 위반은 로더가 거부한다. 인식기는 정상 입력에서 예외를 내지 않고 상태로 답한다. 내부 오류는 `invalid` + 사유로 바꾼다.
- 평가기는 인식기 예외를 `invalid`(`exception:<type>`)로 집계하고 traceback을 남긴다.
- prior 파일의 미지 키·범위 위반·치수 관계 위반은 `ValueError`.
- **지원 범위 밖:** 근접(< 1.1 m), 기울어진 팔레트, 다중 팔레트. 기울어진 평면은 3단계의 수직 재적합 잔차 검사에서 걸러지고, 팔레트가 둘 이상 보이면 최고 점수 하나만 내며 나머지를 결과에 기록한다. 이들은 "성공"이 아니라 **정의된 결과**를 내는지 시험한다.

## 8. 사용자 결정 필요 항목

1. **가려진 포켓:** 광선 검증으로 가림이 확인되면 `invalid`(`pocket_occluded`)로 낸다(권장). 가림 장면 20개의 검출률은 낮게 나오는 것이 정상이며 M3 추적으로 보완한다. 대안: 대칭 추정으로 `valid`를 내되 σ를 키운다(v1에서는 하지 않음 — σ 자체를 `None`으로 두므로 불가).
2. **형상 prior:** 합성 상수를 `config/pallet_prior_v1.yaml`로 명시 주입(권장, 기본값 없음). 대안: prior 없이 전부 추정(더 취약).
3. **σ:** v1은 `None`(미상)으로 두고 진단값만 기록(권장, Codex 지적 반영). 대안: 검증되지 않은 오차 예산을 넣는다.
4. **depth 전용:** RGB는 시각화에만(권장).

## 9. 시험 계획

- **단위(광선 추적 합성 입력):** 알려진 평면·직사각형 개구·바닥을 광선 교차로 렌더해 `SceneInput`을 직접 만든다(yaw가 있으면 평면 depth는 픽셀마다 달라야 한다). 로더의 8×6·640×480 fixture는 **파일·좌표 계약 시험**이므로 인식기 성공 시험에 재사용하지 않는다.
- **필수 반례:** 덱 경계 잔여점, 개구 내부의 소수 전면점, 두 구간이 남아 있는 가림(= 광선 검증이 잡아야 함), 가림 없는 NaN 패치(= `unknown`이 많아 불확실), 전부 NaN, 경쟁 평면(s009형: cube 4 180점 > 팔레트 측면 3 067점 > 전면 2 696점), 거리·격자 위상 변화, 법선 ±부호, yaw ±π 경계, 폭 문턱 바로 안팎. **"개구부 = NaN"을 무조건 `valid`로 기대하지 않는다**(unknown 계약).
- **평가기:** 판정표 7행 전부, `abs` yaw 사용, 모든 `valid` 기준 분포와 TP 전용 분포의 차이, 표본 0 → `null`, CSV/JSON 스키마.
- **작은 통합:** 같은 합성 장면을 v1 PNG·JSON으로 임시 저장 → 로더 → 인식기 → 평가기(mm 반올림·unknown=0 포함). 데이터 세트가 없어도 항상 실행한다.
- **실데이터 CLI:** 데이터 세트가 있을 때만 dev 3장면으로 끝까지 실행(없으면 skip 사유 보고). overlay는 **알려진 코너의 투영 좌표 시험**과 대표 결과 직접 확인을 구분한다(Pillow가 열린다는 것만으로는 정확도 증명이 아니다).

## 10. 구현 단계 (계획 2개)

1. **핵심 계산:** 평가 판정표·집계 시험 → 인식기(광선 추적 fixture) → 작은 파일 통합시험.
2. **실행·시연:** 얇은 CLI → metrics/CSV/overlay/MP4 → dev 튜닝 → 고정 eval 1회 → 검증 기록.

**빼는 것:** 범용 렌더러·플러그인 구조, 자동 파라미터 탐색, 학습/RGB 검출기, 대칭 복원, 다중 대상 연결, 검증되지 않은 σ. YAML·Pillow·ffmpeg는 실행 경계에서만 의존하고 순수 평가 계산에 끌어들이지 않는다.
