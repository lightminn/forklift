# 장면 데이터 세트 계약 v1

합성 장면의 PNG·JSON을 [`scene_dataset.py`](../../src/forklift_core/perception/scene_dataset.py)로 읽어 기존 `PinholeIntrinsics`·`RigidTransform`과 [포켓 관측 타입](pocket-observation.md)을 구성한다. 설계 근거는 [승인 설계 §7·§9](../design/2026-09-11-pocket-observation-and-scene-set.md)다. 이 로더는 파일 입력 계약을 검증하며, 캡처 실행·카탈로그 생성·실물 보정을 수행하지 않는다.

## 설치와 소유 규칙

PNG 로딩에는 Pillow ≥ 10이 필요하다. `python -m pip install 'forklift-core[dataset]'`로 선택 의존성을 설치한다. 저장소 개발 설치 `python -m pip install -e '.[dev]'`에도 Pillow가 포함된다. Pillow는 PNG 로딩 함수 안에서 지연 import하며, 없으면 위 `dataset` 설치 명령을 안내하는 `ImportError`를 발생시킨다. ROS·Gazebo·센서 SDK는 필요하지 않다.

| 타입·함수 | 소유자와 내용 |
|---|---|
| `SceneInput` / `load_scene_input(scene_dir)` | 인식기가 받는 유일한 입력. RGB, depth, 내·외부 보정, 촬영 시각·clock domain·출처. 정답과 장면 분류 메타데이터는 노출하지 않음 |
| `SceneSample` / `load_scene_sample(scene_dir)` | 평가기만 소유. `input: SceneInput`, `ground_truth: PocketObservation`, `scene: dict` |

`load_scene_input`은 `ground_truth.json`을 읽지 않는다. 따라서 정답 파일이 없거나 잘못돼도 입력 파일이 유효하면 인식기 입력을 읽을 수 있다. `SceneSample`이나 그 `scene`·정답을 인식기에 넘기지 않는다. 평가기는 `sample.input`만 전달한다.

`SceneInput` 필드는 다음과 같다. 로더가 v1 입력을 검사하고 구성하며, 단순히 `SceneInput` 생성자를 호출하는 것이 파일 검증을 대신하지는 않는다.

| 필드 | 형식·의미 |
|---|---|
| `rgb` | `uint8[H,W,3]`, RGB 순서 |
| `depth_m` | `float64[H,W]`, m 단위 광축 z, NaN = unknown |
| `intrinsics` | 정확히 이 격자의 `PinholeIntrinsics` |
| `base_from_optical` | `camera_optical_frame` → `base_link`의 `RigidTransform` |
| `stamp_ns` | `scene.json`의 촬영·측정 시각, bool을 제외한 정수 ≥ 0 |
| `clock_domain` | `scene.json`의 `CLOCK_DOMAINS` 값 |
| `source_provenance` | `scene.json`의 `PROVENANCES` 값 |
| `rectified` | `True`, 왜곡 보정된 격자 |
| `rgb_registered_to_depth_grid` | `True`, RGB가 depth 격자에 정합되어 있다는 v1 캡처 계약 |
| `pixel_frame` | 속성값 `intrinsics.frame_id` |

두 영상의 크기가 같다는 사실만으로 정합을 증명할 수 없다. 이 v1 형식의 생산자는 왜곡 보정·RGB/depth 정합을 보장해야 한다. 생성자나 로더는 영상 내용을 분석해서 정합 정확도를 측정하지 않는다.

## 장면 디렉터리와 파일 7개

```text
scenes/s001/
├── rgb.png
├── depth_mm.png
├── depth_meta.json
├── camera_info.json
├── tf.json
├── ground_truth.json
└── scene.json
```

| 파일 | 형식·검증 |
|---|---|
| `rgb.png` | 8-bit RGB, shape (H, W, 3). 회색·팔레트·RGBA 모드를 자동 RGB 변환하지 않고 거부 |
| `depth_mm.png` | 16-bit 단일 채널, shape (H, W). raw 값은 mm, 0은 unknown |
| `depth_meta.json` | 아래 네 필드로 광축 깊이·단위·변환 계수·unknown 값 명시 |
| `camera_info.json` | 수신 CameraInfo의 frame·stamp·해상도·D/K/R/P·binning·ROI. 아래 좁은 계약 적용 |
| `tf.json` | 실제 수신 `/tf_static`의 frame·이동·quaternion·출처 |
| `ground_truth.json` | `PocketObservation`의 모든 필드. 평가 표본 로딩 시에만 읽음 |
| `scene.json` | 장면 식별·카탈로그 버전·분류·split·stamp·clock·출처. 추가 키도 보존 |

두 영상은 `Image.open` 결과의 `image.format == "PNG"`를 확인한다. 확장자만 PNG인 TIFF 등은 거부한다. PNG IHDR의 원본 비트 깊이·색상 유형도 검사해 8-bit RGB와 16-bit grayscale만 받는다. Pillow가 16-bit RGB를 uint8로 축소해 읽어도 통과시키지 않는다. depth의 `I;16` 계열 모드는 uint16으로 읽는다. 구버전 Pillow가 16-bit PNG를 `I`(int32)로 열면 **0–65535 범위를 먼저 검사한 뒤** uint16으로 변환한다. 그 외 모드는 거부한다. RGB·depth shape가 CameraInfo width/height와 다르면 `ValueError`다.

잘못된 형식·수치·frame·시각·필수 메타데이터 누락은 `ValueError`로 거부한다. CameraInfo·ROI·TF·depth metadata는 아래에 명시한 키만 허용하며 미지 키를 거부한다. 필수 파일 누락은 `FileNotFoundError`도 허용한다. `SceneSample.scene`은 `scene.json`의 추가 필드까지 보존한다.

## Depth 저장과 복원

생산자는 depth를 `round(depth_m × 1000)`으로 최근접 반올림해 저장한다. NaN·±Inf·0 이하 값은 unknown인 0으로 기록한다. 유한한 양수 깊이가 **65.535 m를 넘으면 캡처 실패로 거부**하며 잘라내거나 uint16 overflow로 변환하지 않는다. 시각화용 8-bit `depth_preview.png`는 별도 산출물이며 로더의 depth 입력이 아니다.

`depth_meta.json`은 다음 고정 형식이다.

```json
{
  "unit": "mm",
  "meters_per_unit": 0.001,
  "unknown_value": 0,
  "kind": "optical_axis_z"
}
```

`decode_depth_mm(raw, meta)`는 2차원 uint16 배열만 받는다. `kind`는 `optical_axis_z`, `unit`은 `mm`, `meters_per_unit`은 **정확히 0.001**, `unknown_value`는 bool을 제외한 정수 0이어야 한다. NaN·Inf scale, 다른 sentinel, 다른 dtype·shape도 `ValueError`다. 입력 배열을 수정하지 않고 raw 0을 NaN으로, 나머지를 raw × 0.001의 float64 m 값으로 만든다. 광축 z는 카메라 원점에서의 유클리드 광선 길이와 다르다.

**복원된 `depth_m`를 역투영할 때는 `meters_per_unit=1.0`을 쓴다.** 다시 0.001을 곱하면 깊이가 1/1000이 된다.

```python
from pathlib import Path

import numpy as np

from forklift_core.perception.scene_dataset import load_scene_input
from forklift_core.sensors.rgbd import deproject_depth_pixels

given = load_scene_input(Path("data/synthetic_scenes/catalogue_v1/scenes/s001"))
optical = deproject_depth_pixels(
    given.depth_m,
    np.array([[6, 1]], dtype=np.int64),
    given.intrinsics,
    meters_per_unit=1.0,
    pixel_frame=given.pixel_frame,
    rectified=given.rectified,
)
base = given.base_from_optical.apply(optical)
```

픽셀 (6, 1)은 사용법 예시다. [로더 시험](../../tests/unit/perception/test_scene_dataset.py)의 8×6 합성 fixture는 fx=fy=4, cx=4, cy=3, 해당 픽셀 깊이 2 m를 사용한다. 독립 기대값은 optical (1, −1, 2) m, 주어진 합성 장착 변환 후 base (2.2, −1, 1.5) m다. 이 값은 실물 카메라 보정값이 아니다. fixture PNG·JSON은 시험 코드가 임시 디렉터리에 생성하며 저장소에 바이너리 fixture를 추가하지 않는다.

## CameraInfo의 좁은 계약

필수 키는 `frame_id`, `stamp_ns`, `width`, `height`, `distortion_model`, `d`, `k`, `r`, `p`, `binning_x`, `binning_y`, `roi`다. 행렬 배열은 행 우선 순서이며 K와 R은 길이 9, P는 길이 12다. 수신 값을 그대로 보존하고 반 픽셀 보정을 하지 않는다.

- D는 1차원이며 모든 계수가 0이어야 한다. 빈 배열도 허용한다.
- R = I여야 한다.
- K는 `[fx, 0, cx; 0, fy, cy; 0, 0, 1]` 형태이고 fx·fy는 유한 양수, cx·cy는 유한 값이어야 한다. skew는 0이다.
- P의 왼쪽 3×3은 K와 같고 마지막 열은 모두 0이어야 한다. 단안이며 stereo baseline은 없다.
- `binning_x`, `binning_y`는 정수 0 또는 1이다.
- ROI의 `x_offset`, `y_offset`, `height`, `width`는 모두 정수 0, `do_rectify`는 false다.
- width·height는 양의 정수이며 RGB와 depth 모두의 해상도와 일치해야 한다.

벗어나는 CameraInfo에서 K만 조용히 추출하지 않는다. `intrinsics_from_camera_info(obj)`는 위 조건을 검사한 뒤 `PinholeIntrinsics(width, height, fx=k[0], fy=k[4], cx=k[2], cy=k[5], frame_id)`를 반환한다. `distortion_model`은 수신 메타데이터로 남고, 로더가 다른 왜곡 모델을 보정하는 기능은 없다.

## TF와 정답 시각

`tf.json`의 형식은 다음과 같다. 숫자는 시험용 합성 장착 예시다.

```json
{
  "target_frame": "base_link",
  "source_frame": "camera_optical_frame",
  "translation_m": [0.2, 0.0, 0.5],
  "quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
  "origin": "received_tf_static"
}
```

생산자는 설정값 사본 대신 **실제 수신한 `/tf_static`** 값을 저장한다. `transform_from_tf_json`은 위 frame 두 개와 `origin == "received_tf_static"`를 검사한다. 이동은 유한한 m 단위 3개 값이다. quaternion은 xyzw 순서의 유한 값 4개, 노름 1±1e-6이며 허용오차 내에서는 정규화 후 회전행렬로 변환한다. q와 −q는 같은 회전이다. 로더의 출처 문자열 검사는 실제 수신 과정 자체의 재검증을 대신하지 않는다.

`scene.json`의 필수 키와 예시는 다음과 같다. 카탈로그 항목·이미지 ID·소스 snapshot 해시·카메라 설정 같은 추가 메타데이터도 그대로 저장한다.

```json
{
  "scene_id": "s001",
  "catalogue_version": "test",
  "category": "positive",
  "split": "dev",
  "stamp_ns": 5000000000,
  "clock_domain": "ros_sim",
  "source_provenance": "synthetic"
}
```

`load_scene_input`에서도 다음 교차 검증을 수행한다.

- `camera_info.stamp_ns == scene.stamp_ns`. 두 시각은 모두 bool을 제외한 0 이상 정수다.
- `intrinsics.frame_id == base_from_optical.source_frame`.
- `scene.clock_domain ∈ CLOCK_DOMAINS`, `scene.source_provenance ∈ PROVENANCES`. 허용값은 [관측 계약 필드 표](pocket-observation.md#필드)를 따른다.

캡처 생산자는 RGB·depth·CameraInfo가 **같은 header stamp**인 한 세트를 저장한다. 카탈로그 정답은 시각이 없는 기하이므로 `stamp_ns=0`, `clock_domain="synthetic"`를 사용한다. 캡처 시 정답 사본에 해당 프레임의 stamp와 `clock_domain="ros_sim"`을 붙이고 `source_provenance="synthetic_ground_truth"`는 유지한다. `load_scene_sample`은 `ground_truth.stamp_ns == scene.stamp_ns`와 clock domain 일치를 추가로 요구한다. 팔레트가 없으면 정답은 `no_pallet` 상태이며, 가림은 정답 기하를 바꾸지 않고 장면의 가시성 메타데이터로 기록한다.

배치 최상위 `manifest.json`에는 파일별 SHA-256과 성공·실패 기록을 두지만, 이 단일 장면 로더는 manifest 해시·배치 완전성을 검사하지 않는다. 카탈로그·batch 완비 검증은 아래 병합 도구가 담당하며 실물 정확도 검증은 별도다.

## 캡처 생산자와 batch/세트 manifest

[`scene_files.py`](../../ros2/src/forklift_ros/forklift_ros/scene_files.py)가 ROS 없이
PNG·JSON을 인코딩하고, `scene_capture` 노드가 실제 수신 세트를 선택한다.
[`capture_scenes.py`](../../sim/gazebo/capture_scenes.py)는 카탈로그 범위를 실행하고
`scene_capture/manifest.json`을 장면마다 갱신한다. 구독·warmup·deadline 순서는
[캡처 절차](../../sim/gazebo/README.md#장면-캡처)를 따른다.

생산자는 RGB `rgb8`·depth `32FC1`의 little-endian packed payload, step·길이·frame을
검사한다. 촬영 시각은 header의 정수 `sec × 1_000_000_000 + nanosec`다.
RGB·depth·CameraInfo에 같은 stamp를 요구하고, static TF의 stamp 0에는 영상과의
시각 일치를 요구하지 않는다. `scene.json`은 기존 필수 키에 `camera`, `image_id`,
`source_snapshot_sha256`, `run_id`, `visibility`, `wall_times_s`를 추가한다.
`clock_domain`은 `ros_sim`, 입력 출처는 `synthetic`이다. GT는 카탈로그 전체 사본에서
`stamp_ns`·`clock_domain`만 치환하며 `synthetic_ground_truth`를 유지한다.

| batch manifest 필드 | 의미 |
|---|---|
| `catalogue_version`, `catalogue_sha256`, `camera` | 카탈로그 버전·입력 YAML 실제 바이트 SHA-256·합성 카메라 설정 |
| `image_id`, `source_snapshot_sha256`, `run_id` | 원격 wrapper가 전달한 immutable 이미지·검증된 snapshot digest·실행 ID |
| `requested_scenes` | 요청한 모든 장면 ID |
| `scenes[id].passed`, `.error`, `.files`, `.wall_times_s` | 성공 여부·실패 사유·파일 8개 SHA-256·누적 monotonic 도달 시각 |
| `failed_count` | 기록된 실패 장면 수. 요청·결과 집합 일치와 모든 `passed`도 확인해야 batch 성공 |

[`merge_scene_batches.py`](../../tools/merge_scene_batches.py)는 회수 디렉터리 안의
`scene_capture/manifest.json`과 `scene_capture/scenes/sNNN/`을 받아 새 세트를 만든다.
기존 출력 디렉터리는 거부하며 검증에 실패한 batch의 일부 장면을 회수하지 않는다.
실패 batch 전체를 새 run ID로 재제출하고 원래 batch는 입력에서 제외한다.

병합 시 모든 batch의 카탈로그 버전·실제 파일 해시·camera·image ID·snapshot digest가
일치해야 한다. `failed_count == 0`, 모든 장면 `passed`, 요청 ID·결과 키·디렉터리명·
`scene.json.scene_id` 일치를 검사하고, 요청 ID 합집합이 중복·누락 없이 카탈로그와
같아야 한다. 장면 category·split·버전·camera·image·snapshot·run ID를 카탈로그와
batch에 대조한다. GT는 허용한 시각 치환 외에는 카탈로그와 동일해야 하고, 양성·가림은
`valid`, 음성 **세** 범주(`negative_no_pallet`·`negative_lookalike`·`negative_block_row`,
`merge_scene_batches.py:41-46`)는 `no_pallet`이어야 한다. 이는 **정답의 상태** 제약이며,
검출기의 기대 출력과는 별개다 — 음성 GT 가 `no_pallet` 인 것과 그 장면에서 검출기가
`invalid` 를 내는 것이 기대값인 것은 양립한다. 각 장면 파일 해시 재계산과
`load_scene_sample`도 모두 성공해야 한다.

세트 출력은 `manifest.json`과 `scenes/sNNN/`이며 장면마다 필수 7개 파일과 preview
1개를 복사한다. 복사본의 해시를 다시 확인하며 로그·world는 원본 batch에 보존한다.
세트 manifest는 공통 카탈로그·camera·image·snapshot 메타데이터,
`scenes[id]: {batch_run_id, files}`, `category_counts`, `split_counts`를 담는다.
실행 명령은 [개발 환경의 원격 절](../development.md#장면-batch-제출과-병합-scenes)을 따른다.

단일 장면 로딩 성공, 세트 완비·해시 검증, 실제 Gazebo 관측, 실물 보정·성능은 각각
다른 증거다. 구현과 호스트 시험만으로 실제 100장면 캡처가 완료됐다고 판단하지 않는다.
