# Isaac 입력의 반 픽셀 좌표 규약 변환

## 범위와 증거

기준은 HEAD `2671053` + 기존 transport primitive 미커밋 변경이다.
이번 수정은 Isaac 입력 어댑터와 G1 비교·기록 경로에만 적용한다.
Isaac/Kit·원격 실행은 하지 않았다. 아래 저장 캡처 분석 수치는 사용자가 이번
구현 지시에 확정 근거로 제공한 이전 교차검증 결과이며, 이번에 재측정한 값이 아니다.
로컬 CPU 시험 통과는 새 Isaac 렌더 또는 G1 전체 통과를 뜻하지 않는다.

| 기존 depth 독립 계산 | 결과 |
|---|---|
| 수평 높이판의 유효 depth 6,246개로 cy 역산 | 239.499941 px, σ 0.0000685 px |
| 기울어진 원거리 교정판 9캡처로 cx 역산 | 319.5017 px |

| 저장 캡처 역투영 주점 cx / cy | 평균 높이 오차 | 거리별 게이트 |
|---|---|---|
| 320 / 240 | +2.9683 mm | 0/285 PASS |
| 319.5 / 239.5 | +0.000248 mm | 285/285 PASS |
| 319.45 / 239.43 | −0.4153 mm | 174/285 PASS, 과보정 |

적용하는 변환은 **cx와 cy 각각 −0.5 px**다. RGB 적합에서 보인 추가 약 0.05 px는
depth 보정에 더하지 않는다. fx·fy·해상도·장착 변환·depth 값·정수 픽셀 인덱스는
이 좌표 변환으로 바꾸지 않는다.

## 구현과 실행 기록

- `sim/isaac/perception_adapter.py`: `read_isaac_intrinsics()`가 실제
  `Camera.get_intrinsics_matrix()`와 `get_resolution()`을 읽는다. 기존에는
  `scene_rig.intrinsics()`를 검출기 입력에 직접 넣었다.
- `IsaacIntrinsics.raw_sdk`가 원본을 보존하고, `integer_index`는 그 원본에서
  정규화 K를 만든다. `normalize_isaac_intrinsics()`에 반환된 쌍을 다시 넣어도
  같은 쌍을 반환한다. 이미 정규화된 `integer_index` 멤버를 raw SDK 값이라고
  다시 전달하는 것은 API 계약 밖이다. getter 배열은 수정하지 않는다.
- `SceneInput.intrinsics`에는 `integer_index`만 넣는다. 공통
  `pocket_detector._base_points`, `deproject_depth_pixels`, CPU
  `tools/scene_rig.intrinsics()`는 수정하지 않는다.
- `run_transport.py`의 G1a는 **raw getter ↔ raw SDK nominal**, **정규화 getter ↔
  integer-index nominal**을 각각 비교한다. 기존 focal 상대 허용치 0.0012와
  주점 허용치 0.1 px, 양방향의 inclusive 경계, render product 해상도 대조를 유지한다.
- `verify_perception_camera.py`는 G1a에 raw nominal을 전달하고, 영상 배열에 대한
  투영·역투영·높이 및 RGB 적합 비교에는 정규화 nominal을 사용한다. RGB 잔차를
  추가 보정하거나 ②a·⑦a′의 기준을 완화하지 않는다. 교정판의 기존 배치는 raw
  nominal을 계속 사용하여 이번 좌표 규약 변환 때문에 표적 geometry가 바뀌지 않게 한다.

현재 nominal의 두 K는 다음과 같다. 실 실행에서는 getter 값을 읽어 기록한다.

```text
raw_sdk, coordinate_convention = isaac_sdk_half_integer_centers
[[465.741156,   0,          320.0],
 [  0,        465.741156,   240.0],
 [  0,          0,           1.0]]

integer_index, coordinate_convention = integer_index_centers
[[465.741156,   0,          319.5],
 [  0,        465.741156,   239.5],
 [  0,          0,           1.0]]
```

G1 실행 기록의 `perception_camera_intrinsics.raw_sdk`와 `.integer_index`에는
각각 `matrix`, `coordinate_convention`, `nominal`, `errors`, `status`를 남긴다.
종전의 모호한 최상위 `matrix`·`nominal`·`errors`는 이 두 규약별 항목으로 옮겼다.
`normalization`에는 중심 성분에서 0.5를 빼는 식을 기록한다. getter 자체가
잘못되어 읽지 못한 경우에는 nominal과 실패 이유만 남으며 실제 K를 꾸며내지 않는다.

Transport의 매 관측 `capture_diagnostics.intrinsics`에도 실제 raw/정규화 K의
`matrix`, `coordinate_convention`, 해상도, frame 및 변환식을 함께 남긴다.
G1b에는 `measurement_intrinsics_coordinate_convention = integer_index_centers`도 남긴다.

## 닫히지 않은 항목

1. **②a — RGB 적합의 추가 잔차:** 319.5/239.5와 비교해도 **3/18 FAIL**이다.
   최대 cx 잔차는 **−0.1326 px**이며 원인은 미특정이다. 반 픽셀 규약 변환은
   이 잔차의 원인을 밝히거나 게이트를 닫지 않는다.
2. **⑦a′ — 사전 선정 표본의 관측 누락:** 반 픽셀 보정 후에도
   **38 PASS · 11 FAIL · 5 UNOBSERVED**다. 계획 표본 6,393개 중 **147개가
   원래부터 누락**이며 유효 depth는 6,246개다. 「중심이 보인다」는 조건은
   「모든 사전 선정 표본이 관측된다」를 보장하지 않는다. 누락을 제외해
   통과로 집계하지 않으며 이번 수정으로 해결됐다고 간주하지 않는다.
3. **Gazebo v1 — 별도 반 픽셀 항목:** 기존 s095의 cx 역산 중앙값은 319.495이며,
   바닥 높이 오차는 cy=240에서 +1.107 mm, cy=239.5에서 −0.032 mm였다.
   [Gazebo 기준선 설명](../../sim/gazebo/README.md)에도 반 픽셀 차이가 기록돼 있다.
   그러나 `scene_dataset.intrinsics_from_camera_info()`는 주점 보존이 계약이다.
   이를 바꾸면 s009의 왼쪽 개구 폭이 0.27 → 0.26 m로 달라진다.
   **v1 데이터·로더·저장 관측·재생 가드는 그대로 보존한다.** 이 문제는 별도 후속 항목이다.

## 로컬 검증

개발 패키지가 이 checkout을 가리키는 editable 설치임을 확인한 뒤 conda base Python으로
실행했다. 단위 시험의 카메라와 G1a USD readback은 가짜 입력이며 Isaac을 실행하지 않는다.

- RED: 어댑터·G1a 집중 시험 **6 failed, 117 passed**. 주점 미변환,
  변환 API 부재, 규약별 기록 부재를 확인했다.
- GREEN: 같은 집중 시험 **123 passed**. 정확한 0.5 변환, 비중앙 주점 보존,
  반복 어댑터·캡처의 이중 보정 방지, 원본 getter 불변, 정수 픽셀의 역투영과
  두 규약별 G1a 비교·기록을 확인했다.
- 전체 시험: **1,140 passed in 188.44s**, 실패·skip 없음. 사용자 제공 기준선
  1,135개에서 5개를 추가했다.
- 수정하지 않은 v1 재생 가드: **2 passed in 26.64s**. 저장된 dev 70개와 eval
  30개 관측의 기존 허용 delta를 그대로 대조했다.
- G1 측정·결과·render pipeline CPU 시험: **126 passed in 4.07s**.
- 변경한 Python 5개 파일의 Ruff lint/format check 및 `git diff --check` 통과.
  작업 전 diff와 대조해 기존 primitive 변경의 모든 hunk를 보존했으며 공통
  검출기·RGB-D 수학·CPU 리그·v1 로더·재생 가드·Gazebo 경로의 diff가 없음을 확인했다.

```sh
python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error
python -m pytest tests/integration/test_detector_v1_replay.py -q -p no:cacheprovider -W error
```

Isaac 재실행과 ws1 검증은 사용자가 수행한다. `omni`는 건드리지 않았고 커밋하지 않았다.
