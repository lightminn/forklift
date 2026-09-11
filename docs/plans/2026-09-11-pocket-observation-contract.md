# M1-b 1단계: 포켓 관측 계약과 장면 데이터 세트 로더 구현 계획

> **실행자:** 구현은 Codex에 위임한다(개인 전역 규칙). 각 Task는 실패 시험 → 실패 확인 → 구현 → 통과 확인 순서로 진행하고 체크박스로 추적한다. Claude는 계획·검증·커밋·문서 확인을 맡는다. Codex 샌드박스는 `.git`·`/home/light/anaconda3`·네트워크·Docker에 쓰지 못하므로 새 파일은 `mkdir`/편집으로 만들고 커밋·stage는 하지 않는다. 시험은 `/home/light/anaconda3/bin/python`(conda base, editable 설치됨, Pillow 12.2 있음)으로 실행한다.

**목표:** `base_link` 기준 포켓 관측 타입 `PocketObservation`과 파일 기반 합성 장면을 기존 코어 타입으로 읽는 로더를 만들고 계약 문서를 쓴다. 인식기 입력(`SceneInput`)과 평가 표본(`SceneSample`)을 분리해 정답 누출을 막는다.

**아키텍처:** `src/forklift_core/perception/` 패키지에 관측 타입(`pocket_observation.py`)과 로더(`scene_dataset.py`)를 둔다. 로더는 PNG/JSON을 읽어 `PinholeIntrinsics`·`RigidTransform`·`PocketObservation`을 구성한다. quaternion→회전행렬 변환은 `geometry.py`에 추가한다. ROS·Gazebo 의존 없음. Pillow는 `dataset` extra로 격리하고 로더 안에서 지연 import한다.

**기술 스택:** Python ≥ 3.10, NumPy ≥ 1.23, Pillow ≥ 10(선택 extra), pytest ≥ 7, Ruff.

**Spec:** `docs/design/2026-09-11-pocket-observation-and-scene-set.md` §4(계약), §7(저장 형식), §9(로더), §10(시험).

## 전역 제약

- 기준 revision `main` `743c008`. 브랜치 `feat/pocket-observation-contract`. 완료 후 `--ff-only`로 `main`에 올린다.
- `frame_id`는 `"base_link"`만 허용한다. optical 관측은 먼저 base로 변환한 뒤 타입을 만든다. `transform_observation` 헬퍼는 만들지 않는다.
- 방향 정본은 `insertion_yaw_rad` 하나. 포켓별 축·법선 필드는 없다. 수평 팔레트·수직 직사각형 개구부 가정을 docstring과 문서에 명시한다.
- 좌우 판정은 ℓ=(−sin ψ, cos ψ, 0)와의 내적으로 한다. 단순 y 비교 금지.
- σ `None`은 미상이며 0으로 취급하지 않는다. 정답은 σ=0.0.
- NaN 금지 JSON. 잘못된 입력은 `ValueError`, 정상 소실은 `status`.
- 로더 출력 depth는 m 단위 `depth_m`이며 이후 `deproject_depth_pixels(..., meters_per_unit=1.0)`로 호출한다. 문서와 시험에 명시한다.
- CameraInfo 좁은 계약: D 전부 0, R=I, P[:, :3]=K, skew 0, binning 0 또는 1, ROI 0/`do_rectify` false, width/height가 영상과 일치. 벗어나면 `ValueError`. 값은 그대로 보존(반 픽셀 보정 없음).
- 시험 fixture는 파일이 아니라 시험 코드가 `tmp_path`에 생성한다(snapshot 허용 목록은 `tests/`의 `.py`만 전송).
- 기존 146개 시험 동작 보존, `ruff check .`·`ruff format --check .` 통과. 기능 외 리팩터링 금지.

## 파일 구조

| 파일 | 책임 |
|---|---|
| `src/forklift_core/geometry.py` | 추가: `rotation_matrix_from_quaternion_xyzw(q) -> NDArray` (단위 quaternion 검증, 부호 규약 xyzw) |
| `src/forklift_core/perception/__init__.py` | docstring만 |
| `src/forklift_core/perception/pocket_observation.py` | `Pocket`, `PocketObservation`, `CLOCK_DOMAINS`, `STATUSES`, `PROVENANCES`, `yaw_difference_rad(a, b)`, `pocket_observation_from_json(obj)`, `PocketObservation.to_json()` |
| `src/forklift_core/perception/scene_dataset.py` | `SceneInput`, `SceneSample`, `load_scene_input(dir)`, `load_scene_sample(dir)`, `intrinsics_from_camera_info(obj) -> PinholeIntrinsics`, `transform_from_tf_json(obj) -> RigidTransform`, `decode_depth_mm(raw, meta) -> NDArray` |
| `tests/unit/test_geometry.py` | 추가: quaternion 변환 시험 |
| `tests/unit/perception/test_pocket_observation.py` | 계약 시험 |
| `tests/unit/perception/test_scene_dataset.py` | 로더 시험(생성 fixture) |
| `pyproject.toml` | `dataset = ["Pillow>=10"]`, `dev`에 `Pillow>=10` 추가 |
| `docs/interfaces/pocket-observation.md` | 필드·단위·좌표계·검증·평가 규약·예시 JSON |
| `docs/interfaces/scene-dataset.md` | 장면 디렉터리 파일 형식·CameraInfo 계약·depth 인코딩·tf.json·정답 시각 |
| `README.md` | "현재 구현" 목록에 perception 두 모듈 추가, 설치에 `.[dev]`가 Pillow를 포함함을 명시 |

---

### Task 0: 기준 상태

- [ ] 브랜치 `feat/pocket-observation-contract`가 `main` `743c008` 위에 계획 커밋 1개(`4f981c6`)를 갖고 있고 `git status --short`가 비어 있음(Claude가 확인 후 Codex에 위임).
- [ ] 기준: `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error` → 146 passed, 1 deselected.

### Task 1: quaternion → 회전행렬 (`geometry.py`)

**Interfaces:** `rotation_matrix_from_quaternion_xyzw(q: ArrayLike) -> NDArray[np.float64]`. 입력은 길이 4 `[x, y, z, w]`, 유한, 노름이 1±1e-6이어야 하며 아니면 `ValueError`. 허용오차 안의 입력은 **정규화한 뒤** 행렬로 바꾼다(정규화 없이 표준식을 쓰면 노름 오차 5e-7도 `RigidTransform`의 1e-6 직교 검사에 걸린다). q와 −q는 같은 회전이다. 반환 3×3은 `RigidTransform`의 `rotation` 검증을 통과한다.

- [ ] **Step 1 (RED):** `tests/unit/test_geometry.py`에 추가.

```python
def test_quaternion_xyzw_to_rotation_matches_optical_to_base_convention():
    # Existing synthetic mounting: optical +z -> base +x, +x -> -y, +y -> -z.
    rotation = g.rotation_matrix_from_quaternion_xyzw([-0.5, 0.5, -0.5, 0.5])
    np.testing.assert_allclose(
        rotation, [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-12
    )
    identity = g.rotation_matrix_from_quaternion_xyzw([0, 0, 0, 1])
    np.testing.assert_allclose(identity, np.eye(3), atol=1e-12)


@pytest.mark.parametrize(
    "quaternion",
    [[0, 0, 0, 0.5], [0, 0, 0, 2], [1, 0, 0], [0, 0, np.nan, 1], [0, 0, 0, np.inf]],
)
def test_non_unit_or_malformed_quaternions_are_rejected(quaternion):
    with pytest.raises(ValueError):
        g.rotation_matrix_from_quaternion_xyzw(quaternion)
```

- [ ] **Step 2:** `python -m pytest tests/unit/test_geometry.py -q -p no:cacheprovider -W error` → `AttributeError` 실패 확인.
- [ ] **Step 3:** 구현(표준 xyzw → 행렬식). `_real_array`로 읽고 shape (4,)·유한·`abs(norm − 1) ≤ 1e-6` 검사 후 `q / norm`으로 정규화해 표준식을 적용한다. 시험에 `[1.0000005, 0, 0, 0]`(허용오차 안 → 통과하며 결과가 `RigidTransform` 검증을 통과)과 q/−q 동일성(`[-0.5,0.5,-0.5,0.5]` vs `[0.5,-0.5,0.5,-0.5]`)을 추가한다.
- [ ] **Step 4 (GREEN):** Step 2 명령 통과.

### Task 2: 관측 계약 (`pocket_observation.py`)

**Interfaces:**

```python
CLOCK_DOMAINS = frozenset({"ros_sim", "ros_system", "device", "synthetic"})
STATUSES = frozenset({"valid", "no_pallet", "invalid"})
PROVENANCES = frozenset({"synthetic", "replay", "live", "synthetic_ground_truth"})
OBSERVATION_FRAME = "base_link"


@dataclass(frozen=True)
class Pocket:
    center_m: tuple[float, float, float]
    width_m: float
    height_m: float


@dataclass(frozen=True)
class PocketObservation:
    stamp_ns: int
    clock_domain: str
    frame_id: str
    source_provenance: str
    status: str
    left: Pocket | None
    right: Pocket | None
    insertion_yaw_rad: float | None
    position_sigma_m: float | None
    yaw_sigma_rad: float | None
    reason: str | None

    def to_json(self) -> dict: ...


def pocket_observation_from_json(obj: dict) -> PocketObservation: ...
def yaw_difference_rad(
    estimate_rad: float, reference_rad: float
) -> float: ...  # (−π, π]
```

검증 규칙(`__post_init__`, 모두 `ValueError`):
1. `stamp_ns`: `int`(bool 제외) ≥ 0. `clock_domain ∈ CLOCK_DOMAINS`, `status ∈ STATUSES`, `source_provenance ∈ PROVENANCES`, `frame_id == OBSERVATION_FRAME`.
2. `Pocket`: `center_m`는 유한 float 3개 튜플로 정규화, `width_m`·`height_m`는 유한 양수.
3. `status == "valid"`: `left`·`right`·`insertion_yaw_rad` 필수, `reason`은 `None`. yaw 유한이며 (−π, π] 범위. σ는 `None` 또는 유한 ≥ 0.
4. `status != "valid"`: `left`·`right`·`insertion_yaw_rad`·σ 모두 `None`, `reason`은 비어 있지 않은 문자열.
5. 좌우·비중첩: ℓ=(−sin ψ, cos ψ, 0)에 대해 `ℓ·(left.center − right.center) > (left.width_m + right.width_m)/2`(양수이면서 개구부가 겹치지 않음). 두 중심 거리 ∈ (0.05, 2.0) m. 기본 예시는 0.35 > 0.25로 통과한다.
8. 수치 정규화: `width_m`·`height_m`·yaw·σ는 `_finite_scalar`의 반환값(Python float)을 저장하고, `stamp_ns`는 `numbers.Integral`(bool·`np.bool_` 제외)을 받아 `int`로 저장한다. 따라서 numpy 스칼라 입력도 JSON 직렬화가 가능해야 한다.
6. `to_json()`: 필드명 그대로, 튜플은 list, `None`은 null, `Pocket`은 중첩 dict. NaN/Inf가 있으면 검증에서 이미 거부됨. `pocket_observation_from_json`: 알 수 없는 키·누락 키는 `ValueError`(조용히 무시하지 않음).
7. `yaw_difference_rad`: `(estimate − reference + π) mod 2π − π`를 (−π, π]로 조정(정확히 −π는 +π로).

- [ ] **Step 1 (RED):** `tests/unit/perception/test_pocket_observation.py` 작성(발췌; 나머지 규칙도 같은 방식으로 1규칙 1시험).

```python
import json
import math

import numpy as np
import pytest

from forklift_core.perception import pocket_observation as po


def valid_observation(**overrides):
    fields = dict(
        stamp_ns=1_700_000_000_000_000_000,
        clock_domain="ros_sim",
        frame_id="base_link",
        source_provenance="synthetic",
        status="valid",
        left=po.Pocket((1.7, 0.175, 0.15), 0.25, 0.20),
        right=po.Pocket((1.7, -0.175, 0.15), 0.25, 0.20),
        insertion_yaw_rad=0.0,
        position_sigma_m=0.01,
        yaw_sigma_rad=0.02,
        reason=None,
    )
    fields.update(overrides)
    return po.PocketObservation(**fields)


def test_valid_observation_round_trips_through_json():
    observation = valid_observation()
    encoded = observation.to_json()
    assert encoded["left"] == {
        "center_m": [1.7, 0.175, 0.15],
        "width_m": 0.25,
        "height_m": 0.2,
    }
    assert po.pocket_observation_from_json(encoded) == observation
    wire = json.loads(json.dumps(encoded, allow_nan=False))
    assert po.pocket_observation_from_json(wire) == observation


def test_numpy_scalars_are_normalised_to_python_types():
    observation = valid_observation(
        stamp_ns=np.int64(7),
        left=po.Pocket((np.float32(1.7), 0.175, 0.15), np.float32(0.25), 0.2),
        insertion_yaw_rad=np.float64(0.0),
        position_sigma_m=np.float32(0.01),
    )
    assert type(observation.stamp_ns) is int
    assert type(observation.left.width_m) is float
    json.dumps(observation.to_json(), allow_nan=False)
    with pytest.raises(ValueError):
        valid_observation(stamp_ns=np.bool_(True))


def test_non_valid_observation_round_trips_through_json():
    lost = valid_observation(
        status="invalid",
        left=None,
        right=None,
        insertion_yaw_rad=None,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason="both pockets occluded",
    )
    assert (
        po.pocket_observation_from_json(json.loads(json.dumps(lost.to_json()))) == lost
    )


def test_left_and_right_are_defined_by_the_pallet_left_axis_not_by_frame_y():
    # yaw = +0.5 rad: left pocket has larger dot product with l = (-sin, cos, 0).
    left = po.Pocket((2.1552, 0.2054, 0.15), 0.24, 0.20)
    right = po.Pocket((2.3182, -0.0930, 0.15), 0.24, 0.20)
    valid_observation(left=left, right=right, insertion_yaw_rad=0.5)
    with pytest.raises(ValueError):
        valid_observation(left=right, right=left, insertion_yaw_rad=0.5)


def test_half_turn_yaw_swaps_which_pocket_has_larger_frame_y():
    # psi = pi: the pallet faces -x, so the LEFT pocket has the SMALLER frame y.
    # A naive y comparison passes the wrong pair and rejects the right one.
    left = po.Pocket((1.7, -0.175, 0.15), 0.25, 0.2)
    right = po.Pocket((1.7, 0.175, 0.15), 0.25, 0.2)
    valid_observation(left=left, right=right, insertion_yaw_rad=math.pi)
    with pytest.raises(ValueError):
        valid_observation(left=right, right=left, insertion_yaw_rad=math.pi)


def test_overlapping_openings_are_rejected():
    with pytest.raises(ValueError):
        valid_observation(
            left=po.Pocket((1.7, 0.12, 0.15), 0.30, 0.2),
            right=po.Pocket((1.7, -0.12, 0.15), 0.30, 0.2),
        )  # separation 0.24 <= (0.30 + 0.30) / 2


def test_swapped_pockets_at_zero_yaw_are_rejected():
    with pytest.raises(ValueError):
        valid_observation(
            left=po.Pocket((1.7, -0.175, 0.15), 0.25, 0.2),
            right=po.Pocket((1.7, 0.175, 0.15), 0.25, 0.2),
        )


def test_non_valid_status_requires_reason_and_forbids_geometry():
    lost = valid_observation(
        status="no_pallet",
        left=None,
        right=None,
        insertion_yaw_rad=None,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason="no target pallet in scene",
    )
    assert lost.to_json()["left"] is None
    with pytest.raises(ValueError):
        valid_observation(status="invalid", reason="occluded")  # geometry still present
    with pytest.raises(ValueError):
        valid_observation(
            status="no_pallet",
            left=None,
            right=None,
            insertion_yaw_rad=None,
            position_sigma_m=None,
            yaw_sigma_rad=None,
            reason=None,
        )


def test_sigma_none_is_allowed_but_negative_or_nan_is_not():
    assert valid_observation(position_sigma_m=None).position_sigma_m is None
    for bad in (-0.001, math.nan, math.inf):
        with pytest.raises(ValueError):
            valid_observation(position_sigma_m=bad)


@pytest.mark.parametrize(
    "field,value",
    [
        ("frame_id", "camera_optical_frame"),
        ("clock_domain", "wall"),
        ("status", "lost"),
        ("source_provenance", "guess"),
        ("stamp_ns", -1),
        ("stamp_ns", True),
        ("stamp_ns", 1.5),
        ("insertion_yaw_rad", 4.0),
        ("insertion_yaw_rad", math.nan),
    ],
)
def test_enumerations_stamp_and_yaw_range_are_validated(field, value):
    with pytest.raises(ValueError):
        valid_observation(**{field: value})


def test_pocket_dimensions_and_center_must_be_finite_positive():
    with pytest.raises(ValueError):
        po.Pocket((1.7, 0.175, 0.15), 0.0, 0.2)
    with pytest.raises(ValueError):
        po.Pocket((1.7, math.nan, 0.15), 0.25, 0.2)


def test_pocket_separation_bounds():
    with pytest.raises(ValueError):
        valid_observation(
            left=po.Pocket((1.7, 0.01, 0.15), 0.25, 0.2),
            right=po.Pocket((1.7, -0.01, 0.15), 0.25, 0.2),
        )
    with pytest.raises(ValueError):
        valid_observation(
            left=po.Pocket((1.7, 1.5, 0.15), 0.25, 0.2),
            right=po.Pocket((1.7, -1.5, 0.15), 0.25, 0.2),
        )


def test_from_json_rejects_unknown_and_missing_keys():
    encoded = valid_observation().to_json()
    with pytest.raises(ValueError):
        po.pocket_observation_from_json({**encoded, "extra": 1})
    del encoded["reason"]
    with pytest.raises(ValueError):
        po.pocket_observation_from_json(encoded)


@pytest.mark.parametrize(
    "estimate,reference,expected",
    [
        (0.1, -0.1, 0.2),
        (math.pi - 0.1, -math.pi + 0.1, -0.2),
        (-math.pi + 0.1, math.pi - 0.1, 0.2),
        (math.pi, 0.0, math.pi),
        (0.0, math.pi, math.pi),
    ],
)
def test_yaw_difference_wraps_to_half_open_interval(estimate, reference, expected):
    assert po.yaw_difference_rad(estimate, reference) == pytest.approx(expected)
```

- [ ] **Step 2:** 실행 → `ModuleNotFoundError: forklift_core.perception` 실패 확인.
- [ ] **Step 3:** 구현. `Pocket.__post_init__`에서 `_real_array`로 center 검증 후 float 튜플로 저장(`object.__setattr__`). `PocketObservation.__post_init__`에 규칙 1–5. 좌우 내적은 `math`로 계산.
- [ ] **Step 4 (GREEN):** 통과. `ruff check src tests`, `ruff format --check src tests` 통과.

### Task 3: 장면 데이터 세트 로더 (`scene_dataset.py`) + `dataset` extra

**Interfaces:**

```python
@dataclass(frozen=True)
class SceneInput:  # 인식기가 받는 유일한 입력
    rgb: NDArray[np.uint8]  # (H, W, 3)
    depth_m: NDArray[np.float64]  # (H, W), NaN = unknown, 광축 z
    intrinsics: PinholeIntrinsics
    base_from_optical: RigidTransform  # source camera_optical_frame → target base_link
    stamp_ns: int
    clock_domain: str
    source_provenance: str
    rectified: bool = True
    rgb_registered_to_depth_grid: bool = True

    @property
    def pixel_frame(self) -> str:
        return self.intrinsics.frame_id


@dataclass(frozen=True)
class SceneSample:  # 평가기만 소유
    input: SceneInput
    ground_truth: PocketObservation
    scene: dict  # scene.json 원문(scene_id, category, split 등)


def intrinsics_from_camera_info(obj: dict) -> PinholeIntrinsics: ...
def transform_from_tf_json(obj: dict) -> RigidTransform: ...
def decode_depth_mm(raw: NDArray, meta: dict) -> NDArray[np.float64]: ...
def load_scene_input(scene_dir: Path) -> SceneInput: ...
def load_scene_sample(scene_dir: Path) -> SceneSample: ...
```

파일 계약(`docs/interfaces/scene-dataset.md`에 그대로 적는다):
- `rgb.png`: 8-bit RGB, shape (H, W, 3).
- `depth_mm.png`: 16-bit 단일 채널(Pillow mode `I;16`). `depth_meta.json`: `{"unit": "mm", "meters_per_unit": 0.001, "unknown_value": 0, "kind": "optical_axis_z"}`. `decode_depth_mm`은 v1 고정 형식만 받는다: `kind == "optical_axis_z"`, `unit == "mm"`, `meters_per_unit == 0.001`(정확히), `unknown_value == 0`(bool 제외 정수), `raw`는 2차원 `uint16`. 그 외(NaN/Inf scale, 다른 sentinel, 다른 dtype·shape)는 `ValueError`. 검사 후 `raw == unknown_value`를 NaN, 나머지를 `raw * meters_per_unit`(float64)로 만든다.
- `camera_info.json`: `{"frame_id", "stamp_ns", "width", "height", "distortion_model", "d": [..], "k": [9], "r": [9], "p": [12], "binning_x", "binning_y", "roi": {"x_offset","y_offset","height","width","do_rectify"}}`. `intrinsics_from_camera_info`는 전역 제약의 좁은 계약을 검사하고 `PinholeIntrinsics(width, height, fx=k[0], fy=k[4], cx=k[2], cy=k[5], frame_id)`를 반환한다.
- `tf.json`: `{"target_frame": "base_link", "source_frame": "camera_optical_frame", "translation_m": [3], "quaternion_xyzw": [4], "origin": "received_tf_static"}`. `transform_from_tf_json`은 frame 이름 두 개와 `origin`을 검사한다.
- `ground_truth.json`: `PocketObservation` JSON. 로더는 `ground_truth.stamp_ns == scene["stamp_ns"]`와 `clock_domain` 일치를 요구한다.
- 교차 검증(`load_scene_input`에서도 수행): `camera_info.stamp_ns == scene.stamp_ns`, `intrinsics.frame_id == base_from_optical.source_frame`, `scene.clock_domain ∈ CLOCK_DOMAINS`, `scene.source_provenance ∈ PROVENANCES`. `SceneInput.stamp_ns`·`clock_domain`·`source_provenance`는 `scene.json`에서 온다.
- `Image.open` 결과는 `format == "PNG"`여야 한다(확장자만으로 형식을 믿지 않는다).
- `scene.json`: 필수 키 `scene_id`, `catalogue_version`, `category`, `split`, `stamp_ns`, `clock_domain`, `source_provenance`. 나머지는 그대로 보존.
- 영상 shape와 `camera_info` width/height 불일치, `rgb.png`가 RGB가 아님, `depth_mm.png`가 16-bit가 아님, 파일 누락 → `ValueError`(파일 누락은 `FileNotFoundError`도 허용하되 시험은 예외 발생만 확인).

- [ ] **Step 1 (RED):** `tests/unit/perception/test_scene_dataset.py` 작성. fixture는 함수로 생성한다.

```python
import json
import math
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from forklift_core.perception import scene_dataset as sd
from forklift_core.sensors.rgbd import deproject_depth_pixels

WIDTH, HEIGHT = 8, 6
K = [4.0, 0.0, 4.0, 0.0, 4.0, 3.0, 0.0, 0.0, 1.0]  # fx=fy=4, cx=4, cy=3


def write_scene(root: Path, **overrides) -> Path:
    scene = root / "s001"
    scene.mkdir(parents=True, exist_ok=False)  # distinct root per call
    rgb = np.full((HEIGHT, WIDTH, 3), 90, dtype=np.uint8)
    Image.fromarray(rgb).save(scene / "rgb.png")
    depth = np.full((HEIGHT, WIDTH), 2000, dtype=np.uint16)
    depth[1, 6] = 2000
    depth[2, 2] = 1500
    depth[4, 0] = 0  # unknown
    Image.fromarray(depth).save(scene / "depth_mm.png")
    files = {
        "depth_meta.json": {
            "unit": "mm",
            "meters_per_unit": 0.001,
            "unknown_value": 0,
            "kind": "optical_axis_z",
        },
        "camera_info.json": {
            "frame_id": "camera_optical_frame",
            "stamp_ns": 5_000_000_000,
            "width": WIDTH,
            "height": HEIGHT,
            "distortion_model": "plumb_bob",
            "d": [0.0] * 5,
            "k": K,
            "r": [1, 0, 0, 0, 1, 0, 0, 0, 1],
            "p": [4.0, 0.0, 4.0, 0.0, 0.0, 4.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            "binning_x": 0,
            "binning_y": 0,
            "roi": {
                "x_offset": 0,
                "y_offset": 0,
                "height": 0,
                "width": 0,
                "do_rectify": False,
            },
        },
        "tf.json": {
            "target_frame": "base_link",
            "source_frame": "camera_optical_frame",
            "translation_m": [0.2, 0.0, 0.5],
            "quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
            "origin": "received_tf_static",
        },
        "ground_truth.json": {
            "stamp_ns": 5_000_000_000,
            "clock_domain": "ros_sim",
            "frame_id": "base_link",
            "source_provenance": "synthetic_ground_truth",
            "status": "valid",
            "left": {"center_m": [1.7, 0.175, 0.15], "width_m": 0.25, "height_m": 0.2},
            "right": {
                "center_m": [1.7, -0.175, 0.15],
                "width_m": 0.25,
                "height_m": 0.2,
            },
            "insertion_yaw_rad": 0.0,
            "position_sigma_m": 0.0,
            "yaw_sigma_rad": 0.0,
            "reason": None,
        },
        "scene.json": {
            "scene_id": "s001",
            "catalogue_version": "test",
            "category": "positive",
            "split": "dev",
            "stamp_ns": 5_000_000_000,
            "clock_domain": "ros_sim",
            "source_provenance": "synthetic",
        },
    }
    for name, payload in overrides.items():
        files[name] = payload
    for name, payload in files.items():
        (scene / name).write_text(json.dumps(payload))
    return scene


def test_loader_yields_metric_depth_and_core_types(tmp_path):
    sample = sd.load_scene_sample(write_scene(tmp_path))
    given = sample.input
    assert given.rgb.shape == (HEIGHT, WIDTH, 3) and given.rgb.dtype == np.uint8
    assert given.depth_m.shape == (HEIGHT, WIDTH)
    assert given.depth_m[1, 6] == pytest.approx(2.0) and math.isnan(given.depth_m[4, 0])
    assert (
        given.intrinsics.fx == 4
        and given.intrinsics.cx == 4
        and given.intrinsics.cy == 3
    )
    assert given.pixel_frame == "camera_optical_frame" and given.rectified is True
    assert given.base_from_optical.source_frame == "camera_optical_frame"
    assert sample.ground_truth.left.center_m == (1.7, 0.175, 0.15)
    assert sample.scene["scene_id"] == "s001"


def test_depth_is_already_metric_so_deprojection_uses_unit_scale(tmp_path):
    given = sd.load_scene_input(write_scene(tmp_path))
    optical = deproject_depth_pixels(
        given.depth_m,
        np.array([[6, 1], [0, 4]]),
        given.intrinsics,
        meters_per_unit=1.0,
        pixel_frame=given.pixel_frame,
        rectified=True,
    )
    base = given.base_from_optical.apply(optical)
    # pixel (6,1) at 2.0 m: optical (1.0, -1.0, 2.0) -> base (2.2, -1.0, 1.5)
    np.testing.assert_allclose(base.xyz_m[0], [2.2, -1.0, 1.5], atol=1e-12)
    assert not base.valid[1]  # unknown pixel stays unknown


def test_scene_input_does_not_carry_ground_truth(tmp_path):
    given = sd.load_scene_input(write_scene(tmp_path))
    assert not hasattr(given, "ground_truth") and not hasattr(given, "scene")


@pytest.mark.parametrize(
    "patch",
    [
        {"d": [0.1, 0, 0, 0, 0]},
        {"r": [1, 0, 0, 0, 0.99, 0, 0, 0, 1]},
        {
            "p": [4.5, 0.0, 4.0, 0.0, 0.0, 4.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        },  # P[:, :3] != K
        {
            "p": [4.0, 0.0, 4.0, 1.0, 0.0, 4.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        },  # P[:, 3] != 0
        {"k": [4.0, 0.5, 4.0, 0.0, 4.0, 3.0, 0.0, 0.0, 1.0]},
        {"binning_x": 2},
        {
            "roi": {
                "x_offset": 1,
                "y_offset": 0,
                "height": 0,
                "width": 0,
                "do_rectify": False,
            }
        },
        {"width": WIDTH + 1},
    ],
)
def test_camera_info_outside_the_narrow_contract_is_rejected(tmp_path, patch):
    base = json.loads(
        json.dumps(
            {
                "frame_id": "camera_optical_frame",
                "stamp_ns": 5_000_000_000,
                "width": WIDTH,
                "height": HEIGHT,
                "distortion_model": "plumb_bob",
                "d": [0.0] * 5,
                "k": K,
                "r": [1, 0, 0, 0, 1, 0, 0, 0, 1],
                "p": [4.0, 0.0, 4.0, 0.0, 0.0, 4.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                "binning_x": 0,
                "binning_y": 0,
                "roi": {
                    "x_offset": 0,
                    "y_offset": 0,
                    "height": 0,
                    "width": 0,
                    "do_rectify": False,
                },
            }
        )
    )
    base.update(patch)
    with pytest.raises(ValueError):
        sd.load_scene_input(write_scene(tmp_path, **{"camera_info.json": base}))


def test_principal_point_is_preserved_without_half_pixel_adjustment():
    info = {
        "frame_id": "f",
        "stamp_ns": 0,
        "width": 320,
        "height": 240,
        "distortion_model": "plumb_bob",
        "d": [0.0] * 5,
        "k": [160.0, 0, 160.0, 0, 160.0, 120.0, 0, 0, 1],
        "r": [1, 0, 0, 0, 1, 0, 0, 0, 1],
        "p": [160.0, 0, 160.0, 0, 0, 160.0, 120.0, 0, 0, 0, 1, 0],
        "binning_x": 0,
        "binning_y": 0,
        "roi": {
            "x_offset": 0,
            "y_offset": 0,
            "height": 0,
            "width": 0,
            "do_rectify": False,
        },
    }
    intrinsics = sd.intrinsics_from_camera_info(info)
    assert (intrinsics.cx, intrinsics.cy) == (160.0, 120.0)


def test_depth_meta_kind_unit_and_unknown_value_are_validated():
    raw = np.array([[0, 1500]], dtype=np.uint16)
    good = {
        "unit": "mm",
        "meters_per_unit": 0.001,
        "unknown_value": 0,
        "kind": "optical_axis_z",
    }
    decoded = sd.decode_depth_mm(raw, good)
    assert math.isnan(decoded[0, 0]) and decoded[0, 1] == pytest.approx(1.5)
    for bad in (
        {**good, "kind": "ray_length"},
        {**good, "unit": "m"},
        {**good, "meters_per_unit": 0},
        {**good, "unknown_value": 0.5},
    ):
        with pytest.raises(ValueError):
            sd.decode_depth_mm(raw, bad)


def test_tf_json_frames_and_origin_are_checked(tmp_path):
    for patch in (
        {"target_frame": "odom"},
        {"source_frame": "camera_link"},
        {"origin": "config_copy"},
    ):
        tf = {
            "target_frame": "base_link",
            "source_frame": "camera_optical_frame",
            "translation_m": [0.2, 0.0, 0.5],
            "quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
            "origin": "received_tf_static",
            **patch,
        }
        with pytest.raises(ValueError):
            sd.load_scene_input(
                write_scene(tmp_path / patch[list(patch)[0]], **{"tf.json": tf})
            )


def test_ground_truth_stamp_must_match_scene_stamp(tmp_path):
    gt = json.loads((write_scene(tmp_path / "a") / "ground_truth.json").read_text())
    gt["stamp_ns"] = 1
    with pytest.raises(ValueError):
        sd.load_scene_sample(write_scene(tmp_path / "b", **{"ground_truth.json": gt}))


def test_eight_bit_depth_png_is_rejected(tmp_path):
    scene = write_scene(tmp_path)
    Image.fromarray(np.zeros((HEIGHT, WIDTH), dtype=np.uint8)).save(
        scene / "depth_mm.png"
    )
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)
```

(`write_scene`은 `scene.mkdir(parents=True, exist_ok=False)`이므로 서로 다른 root를 넘기면 부모가 없어도 되고, 같은 root를 두 번 쓰면 실패한다. 추가 시험: `camera_info.stamp_ns`가 scene과 다르면 거부, `camera_info.frame_id`가 tf의 source_frame과 다르면 거부, GT `clock_domain`이 scene과 다르면 거부, 비valid GT(`status: no_pallet`)도 로더가 읽어 `SceneSample.ground_truth.status == "no_pallet"`, depth PNG dtype이 uint16이 아니거나 shape이 camera_info와 다르면 거부.)

- [ ] **Step 2:** 실행 → `ModuleNotFoundError`/`AttributeError` 실패 확인.
- [ ] **Step 3:** `pyproject.toml`에 `dataset = ["Pillow>=10"]` 추가, `dev`를 `["pytest>=7", "ruff", "build", "pip-tools", "Pillow>=10"]`로. 로더는 `from PIL import Image`를 함수 안에서 import하고 실패 시 `ImportError` 메시지에 `pip install 'forklift-core[dataset]'`를 안내한다.
- [ ] **Step 4:** 구현. Pillow 12.2에서 `Image.fromarray(array, mode=...)`는 DeprecationWarning을 내고 시험은 `-W error`이므로 `mode` 인자를 쓰지 않는다(uint8 (H,W,3)→RGB, uint16 (H,W)→`I;16`으로 자동 추론되며 다시 열면 mode `I;16`·dtype uint16임을 2026-09-11에 확인). PNG는 `Image.open`으로 열고 `mode`가 `"RGB"`/`"I;16"`(depth는 `"I;16"` 계열)인지 확인한 뒤 `np.asarray`. 16-bit PNG를 Pillow가 `"I"`(int32)로 열면 값 범위 0–65535를 검사한 뒤 uint16으로 바꾸고 그 외 모드는 거부한다(이 환경 Pillow 12.2에서 실제 mode를 확인해 시험에 맞춘다).
- [ ] **Step 5 (GREEN):** 통과. `python -m pytest tests -q -p no:cacheprovider -W error` 전체 통과.

### Task 4: 계약 문서와 README

- [ ] `docs/interfaces/pocket-observation.md`: 목적, 가정(수평 팔레트·수직 개구부·공통 삽입축), 필드 표(단위·frame·허용값), 좌우 정의(ℓ 내적), 방향 정본(`insertion_yaw_rad`, 바깥 법선 −a), 검증 규칙 7개, σ 의미(`None`=미상), 평가 규약(포켓별 위치 오차·wrap yaw 오차·장면 집계·분모 분리), 예시 JSON(valid 1개, no_pallet 1개), "M5 여유 계산에는 불충분" 경계.
- [ ] `docs/interfaces/scene-dataset.md`: 디렉터리 파일 7개 형식(Task 3 계약 그대로), CameraInfo 좁은 계약, depth 인코딩(반올림·unknown·상한 65.535 m), tf.json 출처, 정답 시각 연결, `meters_per_unit=1.0` 사용 주의, `SceneInput` vs `SceneSample` 소유 규칙.
- [ ] `README.md` "현재 구현" 목록에 `perception/pocket_observation.py`, `perception/scene_dataset.py` 두 줄 추가. 로컬 실행 절에 `.[dev]`가 Pillow를 포함한다는 한 줄.
- [ ] 링크 검사 0개, `diff <(tail -n +4 AGENTS.md) <(tail -n +4 CLAUDE.md)` 빈 출력(이번 단계는 두 파일을 바꾸지 않는다).

### Task 5: 검증·커밋 (Claude)

- [ ] 호스트: 전체 회귀(`-m 'not rendering'`) 통과 개수 기록, `ruff check .`, `ruff format --check .`.
- [ ] wheel 메타데이터 확인: `python -m build --wheel` 후 METADATA에 `Provides-Extra: dataset`과 `Requires-Dist: Pillow>=10; extra == "dataset"`이 있는지 확인(원격은 `--system-site-packages`·`--no-deps` 설치라 extra 선언을 검증하지 않는다).
- [ ] 원격 model-cpu 1회(새 snapshot)로 새 시험이 원격 venv에서도 통과함을 확인(Pillow는 원격 base env 12.2.0에 있음). `job_result.json`의 `pytest.tests` 기록.
- [ ] 커밋 `feat(perception): add pocket observation contract and scene dataset loader`(코드·시험·pyproject·interfaces 문서·README 한 번에), 검증 결과는 `docs/validation/2026-09-11-pocket-observation-contract.md`에 짧게 남겨 `docs(validation): …`로 커밋. `main`에 ff-merge, push, 원격 repo 갱신.

## 자체 검토

- **범위:** 설계 §4의 필드·검증·평가 규약 → Task 2; §7 저장 형식·§9 로더·CameraInfo 계약·이중 스케일 방지·fixture 생성 → Task 3; 문서 → Task 4; 패키징(Pillow extra) → Task 3 Step 3. quaternion 변환은 tf.json 로딩에 필요 → Task 1.
- **자리표시자 없음.** 시험 코드는 실제 기대값(2.2, −1.0, 1.5 등)을 포함한다.
- **이름 일관성:** `Pocket`, `PocketObservation`, `pocket_observation_from_json`, `yaw_difference_rad`, `SceneInput`, `SceneSample`, `load_scene_input`, `load_scene_sample`, `intrinsics_from_camera_info`, `transform_from_tf_json`, `decode_depth_mm`, `rotation_matrix_from_quaternion_xyzw`가 Interfaces·시험·파일 표에서 동일하다.
- **미포함(의도):** optical 관측 변환 헬퍼, 추적 상태 타입(M3), `iter_split`, 카탈로그·SDF·캡처(2·3단계).
