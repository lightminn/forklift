# M2 1단계: 포켓 인식기와 평가 계산 구현 계획

> **실행자:** 구현은 Codex에 위임한다. 각 Task는 실패 시험 → 실패 확인 → 구현 → 통과 확인 순서로 진행하고 체크박스로 추적한다. Codex 샌드박스는 `.git`·`/home/light/anaconda3`·네트워크·Docker에 쓰지 못하므로 커밋·stage는 하지 않는다. 시험은 `/home/light/anaconda3/bin/python`으로 실행한다(conda base, editable 설치됨, NumPy 2.4.6 · Pillow 12.2 · PyYAML 6.0.3 · pytest 9.1).
>
> 상태: **v2 — 2026-09-13 Codex 검토 반영.**

**목표:** `SceneInput`과 명시적 형상 prior만 받아 `PocketObservation`을 내는 기하 인식기와, 정답 대비 판정·집계를 계산하는 평가 모듈을 만든다. CLI·overlay·MP4·실데이터 실행은 **모두 2단계**다.

**아키텍처:** `src/forklift_core/perception/`에 세 모듈을 둔다. `pallet_prior.py`는 형상 prior를 YAML에서 검증해 읽고(기본값 없음), `pocket_detector.py`는 depth 점군 → 바닥 제거 → 수직 평면 재적합 → 평면 위 점유 격자 → 광선 단위 개구부 검증 → 상태 판정을 수행하며, `evaluation.py`는 정답과 추정을 판정표에 따라 분류하고 집계한다. ROS·Gazebo 의존 없음.

**기술 스택:** Python ≥ 3.10, NumPy ≥ 1.23, PyYAML ≥ 6, pytest ≥ 7, Ruff.

**Spec:** `docs/design/2026-09-13-pocket-detector-baseline.md`(사용자 승인 v2). 설계 §4 인식기, §5 평가기 판정표, §7 경계, §9 시험 계획.

## 전역 제약

- **기준 코드 revision `2d295fe`**(= 이 계획 커밋의 부모). 구현 착수 시 HEAD는 이 계획 커밋이며 브랜치는 `feat/pocket-detector`다.
- 인식기는 `SceneInput`과 prior·params만 받는다. `SceneSample`·`ground_truth`·`scene` dict를 넘기지 않는다(정답 누출 금지).
- `PalletPrior`는 **dataclass 기본값을 두지 않는다.** 반드시 `load_pallet_prior(path)`로 읽는다. `DetectorParams`는 알고리즘 값이므로 기본값을 갖되 `__post_init__`에서 검증한다.
- σ는 v1에서 `position_sigma_m = None`, `yaw_sigma_rad = None`.
- 방향: `a = −n_out`, `psi = wrap(atan2(n_out[1], n_out[0]) + pi)`를 (−π, π]로. 바깥 법선은 카메라 쪽(`n_out·(c − p0) > 0`). **부호를 두 번 뒤집지 않는다.**
- z 대역은 `[deck_m + band_margin_m, height_m − deck_m − band_margin_m]`. 빈 구간 탐색은 그 대역의 **최좌·최우 점유 열 사이**로 제한한다.
- 개구부 열림/가림 판정은 **광선 단위**로 하고, 비율의 분모는 **후보 사각형을 지나는 전체 광선 수**로 고정한다(평면 ±`front_margin_m` 안의 유한 반환은 `near_plane` 버킷으로 따로 세고 분모에 남긴다).
- 부분 구조 후보를 버리지 않는다. 폭이 범위 밖이어도 후보로 남겨 `opening_width_mismatch` 상태에 도달할 수 있어야 한다.
- yaw 비교는 `abs(yaw_difference_rad(...))`. 위치 오차는 **유클리드 거리**(`math.dist`)다.
- 정상 입력에서 인식기는 예외를 내지 않는다(상태로 답한다). 잘못된 설정은 `ValueError`.
- 기존 시험 571개 행동 보존, `ruff check .`·`ruff format --check .` 통과(저장소는 `B` 규칙을 켜므로 **기본 인자에 함수 호출을 두지 않는다** — B008). 기능 외 리팩터링 금지.

**합성 카메라(시험 fixture 정본, 데이터 세트와 동일):** `width 640`, `height 480`, `fx = fy = 465.741156`, `cx = 320.0`, `cy = 240.0`, base 기준 translation `(0.75, 0.0, 0.5)`, optical quaternion xyzw `(-0.5, 0.5, -0.5, 0.5)`. 이 장착에서 base↔optical 관계는 `(Xopt, Yopt, Zopt) = (−Ybase, 0.5 − Zbase, Xbase − 0.75)`이다.

**팔레트 상자 5개(설계 §5.1 기하, 팔레트 좌표계에서 정의한 뒤 yaw 회전·평행이동):** 원점 C는 바닥 위 footprint 중심(z=0), 삽입축 `a = (cos ψ, sin ψ, 0)`, 왼쪽축 `ℓ = (−sin ψ, cos ψ, 0)`. 깊이 0.6(a 방향)·폭 0.8(ℓ 방향)·높이 0.30, 덱 두께 0.05. 전면은 `C − 0.3·a`를 지나는 평면. 아래 덱 z∈[0, 0.05], 위 덱 z∈[0.25, 0.30], 중앙 지지대 폭 0.10(ℓ 중앙), 바깥 지지대 폭 `s_o = (0.8 − 2w − 0.1)/2`가 ℓ = ±(0.4 − s_o/2)에, 지지대 높이 z∈[0.05, 0.25]. 개구 중심 = `C − 0.3·a ± d·ℓ + 0.15·u`, `d = 0.05 + w/2`. **두 개구 중심 사이 거리는 `2d = 0.10 + w`**(w=0.20이면 0.30, w=0.24면 0.34).

## 파일 구조

| 파일 | 책임 |
|---|---|
| `config/pallet_prior_v1.yaml` | 합성 카탈로그 팔레트 형상(출처·버전 표기) |
| `pyproject.toml` | `dev` extra에 `PyYAML>=6` 추가 |
| `src/forklift_core/perception/pallet_prior.py` | `PalletPrior`, `load_pallet_prior(path)` |
| `src/forklift_core/perception/pocket_detector.py` | `DetectorParams`, `OpeningRayCounts`, `DetectionDiagnostics`, `DetectionResult`, `detect_pockets(...)`, 내부 단계 함수 |
| `src/forklift_core/perception/evaluation.py` | `Outcome`, `SceneResult`, `evaluate_scene(...)`, `summarize(...)` |
| `tests/fixtures/synthetic_scene.py` | 광선 추적으로 `SceneInput`을 만드는 시험 헬퍼 |
| `tests/conftest.py` | 위 헬퍼를 경로 로드해 `pallet_scene` fixture로 노출(모든 하위 시험에서 사용) |
| `tests/unit/perception/test_pallet_prior.py` | prior 로더 검증 |
| `tests/unit/perception/test_pocket_detector.py` | 인식기 단위·반례 시험 |
| `tests/unit/perception/test_evaluation.py` | 판정표·집계 시험 |
| `tests/integration/test_detector_pipeline.py` | 합성 장면 → v1 파일 저장 → 로더 → 인식기 → 평가기 |

---

### Task 0: 기준 상태

- [ ] `git rev-parse HEAD`가 이 계획 커밋이고 그 부모가 `2d295fe`이며 `git status --short`가 비어 있음을 확인한다.
- [ ] 기준 회귀: `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error` → `571 passed, 1 deselected`.

### Task 1: 형상 prior 로더

**Files:** Create `config/pallet_prior_v1.yaml`, `src/forklift_core/perception/pallet_prior.py`, `tests/unit/perception/test_pallet_prior.py`. Modify `pyproject.toml`(`dev` extra에 `PyYAML>=6`).

**Interfaces — Produces:**

```python
@dataclass(frozen=True)
class PalletPrior:
    height_m: float
    deck_m: float
    opening_height_m: float
    opening_width_min_m: float
    opening_width_max_m: float
    centre_spacer_min_m: float
    centre_spacer_max_m: float
    overall_width_m: float
    source_provenance: str  # v1에서는 "synthetic"
    catalogue_version: str  # "v1"

    @property
    def opening_centre_height_m(self) -> float: ...  # deck_m + opening_height_m / 2


def load_pallet_prior(path: Path) -> PalletPrior: ...
```

검증(모두 `ValueError`): 최상위 키 집합이 정확히 `{height_m, deck_m, opening_height_m, opening_width_range, centre_spacer_range, overall_width_m, source_provenance, catalogue_version}`, 치수는 유한 양수, 두 range는 길이 2이고 `min < max`, `height_m == 2*deck_m + opening_height_m`(1e-9), `overall_width_m > 2*opening_width_max_m + centre_spacer_max_m`, `source_provenance in PROVENANCES`.

- [ ] **Step 1: 시험 작성** — `tests/unit/perception/test_pallet_prior.py`

```python
import dataclasses
from pathlib import Path

import pytest
import yaml

from forklift_core.perception.pallet_prior import PalletPrior, load_pallet_prior

REPO_PRIOR = Path(__file__).resolve().parents[3] / "config" / "pallet_prior_v1.yaml"
GOOD = {
    "height_m": 0.30,
    "deck_m": 0.05,
    "opening_height_m": 0.20,
    "opening_width_range": [0.18, 0.30],
    "centre_spacer_range": [0.08, 0.12],
    "overall_width_m": 0.8,
    "source_provenance": "synthetic",
    "catalogue_version": "v1",
}


def write(tmp_path, **overrides):
    data = {**GOOD, **overrides}
    for key in [k for k, v in overrides.items() if v is None]:
        del data[key]
    path = tmp_path / "prior.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


def test_repository_prior_loads_and_reports_the_opening_centre_height():
    prior = load_pallet_prior(REPO_PRIOR)
    assert (prior.height_m, prior.deck_m, prior.opening_height_m) == (0.30, 0.05, 0.20)
    assert prior.opening_centre_height_m == pytest.approx(0.15)
    assert prior.source_provenance == "synthetic" and prior.catalogue_version == "v1"


def test_the_prior_dataclass_has_no_defaults_so_synthetic_sizes_cannot_leak_in():
    for field in dataclasses.fields(PalletPrior):
        assert field.default is dataclasses.MISSING
        assert field.default_factory is dataclasses.MISSING


@pytest.mark.parametrize(
    "overrides",
    [
        {"extra_key": 1},
        {"height_m": None},
        {"height_m": 0.31},
        {"deck_m": -0.05},
        {"opening_height_m": 0.0},
        {"opening_width_range": [0.30, 0.18]},
        {"opening_width_range": [0.18]},
        {"centre_spacer_range": [0.08, float("inf")]},
        {"overall_width_m": 0.5},
        {"source_provenance": "guess"},
    ],
)
def test_malformed_prior_files_are_rejected(tmp_path, overrides):
    with pytest.raises(ValueError):
        load_pallet_prior(write(tmp_path, **overrides))
```

- [ ] **Step 2: 실패 확인** — `python -m pytest tests/unit/perception/test_pallet_prior.py -q -p no:cacheprovider -W error` → `ModuleNotFoundError`.
- [ ] **Step 3: `config/pallet_prior_v1.yaml` 작성**

```yaml
# Pallet shape of the synthetic catalogue v1. These are NOT measured real-pallet
# dimensions; a real pallet needs its own file measured after delivery.
source_provenance: synthetic
catalogue_version: v1
height_m: 0.30
deck_m: 0.05
opening_height_m: 0.20
opening_width_range: [0.18, 0.30]
centre_spacer_range: [0.08, 0.12]
overall_width_m: 0.8
```

- [ ] **Step 4: 로더 구현과 `pyproject.toml` 수정** — `dev` extra를 `["pytest>=7", "ruff", "build", "pip-tools", "Pillow>=10", "PyYAML>=6"]`로 바꾼다(새 단위시험이 `yaml`을 쓴다).
- [ ] **Step 5: 통과 확인** — Step 2 통과, `ruff check src tests config` / `format --check` 통과.

### Task 2: 광선 추적 시험 장면 헬퍼와 공유 fixture

**Files:** Create `tests/fixtures/synthetic_scene.py`, `tests/conftest.py`. Extend `tests/unit/perception/test_pocket_detector.py`(자체 점검 시험).

로더 fixture(8×6·640×480, 일정 깊이)는 **파일 계약 시험**이므로 인식기 시험 입력으로 쓰지 않는다. 이 헬퍼가 인식기 시험 입력의 유일한 출처다.

**Interfaces — Produces (`tests/fixtures/synthetic_scene.py`):**

```python
CAMERA = {
    "width": 640,
    "height": 480,
    "fx": 465.741156,
    "fy": 465.741156,
    "cx": 320.0,
    "cy": 240.0,
    "translation_m": (0.75, 0.0, 0.5),
    "quaternion_xyzw": (-0.5, 0.5, -0.5, 0.5),
}
BACK_WALL_X_M = 6.0


def make_pallet_scene(
    *,
    centre_xy_m: tuple[float, float] = (2.5, 0.0),
    yaw_rad: float = 0.0,
    opening_width_m: float = 0.24,
    pallet: bool = True,
    lookalike: bool = False,  # solid 0.6 x 0.8 x 0.30 box, no openings
    occluder: dict | None = None,  # {"side": "left"|"right", "gap_m": 0.4,
    #  "width_frac": 0.4, "depth_m": 0.10, "height_m": 0.80}
    openings_unknown: bool = False,  # rays through the openings return NaN
    unknown_patch: tuple[int, int, int, int]
    | None = None,  # (u0, v0, u1, v1) forced NaN
    extra_box: dict | None = None,  # {"centre_xy_m": (x, y), "size_m": (dx, dy, dz)}
    stamp_ns: int = 2_000_000_000,
) -> tuple[SceneInput, dict]:
    """Ray-trace a synthetic pallet scene.

    Returns the recognizer input and the exact ground-truth geometry:
    {"left_centre_m", "right_centre_m", "yaw_rad", "opening_width_m",
     "front_plane_point_m", "outward_normal"}.
    """


def opening_ray_fractions(scene: SceneInput, truth: dict, side: str) -> dict:
    """Independent check used by tests: {"front", "behind", "near_plane", "unknown", "total"}
    over the rays that pass through that opening rectangle."""
```

장면 구성: 바닥 z=0 무한 평면, 뒤쪽 벽 x=`BACK_WALL_X_M`, 팔레트 상자 5개(전역 제약의 기하), 선택적 유사물·가림 상자·추가 상자. 각 픽셀 광선과 모든 표면의 교차 중 가장 가까운 것을 optical z(depth)로 기록하고, 교차 없음·`openings_unknown`·`unknown_patch`는 NaN. depth는 float64 m, RGB는 표면별 고정 색(uint8).

- [ ] **Step 1: 자체 점검 시험 작성** — `tests/unit/perception/test_pocket_detector.py` 맨 앞에 둔다.

```python
def test_the_scene_helper_puts_the_openings_and_the_front_face_where_it_says(
    pallet_scene,
):
    scene, truth = pallet_scene(
        centre_xy_m=(2.5, 0.2), yaw_rad=0.4, opening_width_m=0.24
    )
    assert scene.depth_m.shape == (480, 640) and scene.rgb.shape == (480, 640, 3)
    assert truth["left_centre_m"][2] == pytest.approx(0.15)
    spacing = math.dist(truth["left_centre_m"], truth["right_centre_m"])
    assert spacing == pytest.approx(0.10 + 0.24)  # 2d = 0.10 + w
    # a ray through an opening centre must return something well BEHIND the front plane;
    # which surface it is (deck top or back wall) depends on the geometry, so check the depth
    front_depth = depth_at(scene, truth["front_plane_point_m"])
    opening_depth = depth_at(scene, truth["left_centre_m"])
    assert opening_depth > front_depth + 0.2
    # a ray through the centre spacer must stop ON the front plane
    spacer = midpoint(truth["left_centre_m"], truth["right_centre_m"])
    assert depth_at(scene, spacer) == pytest.approx(front_depth, abs=0.02)


def test_the_scene_helper_reports_ray_fractions_that_separate_open_from_occluded(
    pallet_scene,
):
    clear, truth = pallet_scene()
    assert opening_ray_fractions(clear, truth, "left")["behind"] > 0.5
    assert opening_ray_fractions(clear, truth, "left")["front"] == 0.0
    blocked, truth_b = pallet_scene(
        occluder={
            "side": "left",
            "gap_m": 0.4,
            "width_frac": 0.4,
            "depth_m": 0.10,
            "height_m": 0.80,
        }
    )
    left = opening_ray_fractions(blocked, truth_b, "left")
    right = opening_ray_fractions(blocked, truth_b, "right")
    assert left["front"] > 0.5  # the detector threshold occluded_front_frac
    assert right["behind"] > 0.5  # the other pocket stays open
```

(`depth_at(scene, point_base)`와 `midpoint`는 같은 시험 파일의 헬퍼: base 점을 optical로 옮겨 `u = x*fx/z + cx`, `v = y*fy/z + cy`로 반올림한 픽셀의 `depth_m` 값을 돌려준다.)

- [ ] **Step 2: 실패 확인** → fixture 없음.
- [ ] **Step 3: 구현.** `tests/conftest.py`는 `importlib.util.spec_from_file_location`로 `tests/fixtures/synthetic_scene.py`를 읽어(저장소가 ROS 시험에서 쓰는 방식과 같다) `pallet_scene`·`opening_ray_fractions` fixture를 노출한다. `sys.path`를 수정하지 않는다.
- [ ] **Step 4: 통과 확인.** `python -m pytest tests/integration -q -p no:cacheprovider -W error`를 단독 실행해 conftest 경로가 통합시험에서도 동작함을 확인한다.

### Task 3: 인식기

**Files:** Create `src/forklift_core/perception/pocket_detector.py`. Extend `tests/unit/perception/test_pocket_detector.py`.

**Interfaces — Produces:**

```python
@dataclass(frozen=True)
class DetectorParams:
    cell_m: float = 0.01
    plane_inlier_m: float = 0.02
    band_margin_m: float = 0.01
    ransac_iterations: int = 200
    min_plane_points: int = 300
    min_band_points: int = 100  # supports/decks must be observed this much
    max_plane_candidates: int = 3
    range_min_m: float = 0.8
    range_max_m: float = 5.0
    floor_z_m: float = 0.02
    front_margin_m: float = 0.05
    occluded_front_frac: float = 0.5
    open_behind_frac: float = 0.3
    max_plane_residual_m: float = 0.01  # p95 residual of the vertical refit
    width_mismatch_frac: float = 0.20
    seed: int = 20260913


_DEFAULT_PARAMS = (
    DetectorParams()
)  # module constant; never a call in a default argument (B008)


@dataclass(frozen=True)
class OpeningRayCounts:
    front: int
    behind: int
    near_plane: int
    unknown: int
    total: int  # == front + behind + near_plane + unknown

    @property
    def front_fraction(self) -> float: ...  # front / total, 0.0 when total == 0
    @property
    def behind_fraction(self) -> float: ...


@dataclass(frozen=True)
class DetectionDiagnostics:
    plane_residual_p95_m: float | None
    plane_inlier_count: int
    candidate_plane_count: int
    rejected_plane_reasons: tuple[str, ...]  # one entry per rejected candidate
    opening_rays: dict[
        str, OpeningRayCounts
    ]  # {"left": ..., "right": ...} when candidates exist
    opening_width_raw_m: tuple[float, float] | None  # before the half-cell correction
    boundary_resolution_m: float  # cell_m
    centre_height_is_prior: bool  # always True in v1
    elapsed_s: float
    seed: int


@dataclass(frozen=True)
class DetectionResult:
    observation: PocketObservation
    diagnostics: DetectionDiagnostics


def detect_pockets(
    scene_input: SceneInput,
    prior: PalletPrior,
    params: DetectorParams = _DEFAULT_PARAMS,
) -> DetectionResult: ...
```

`DetectorParams.__post_init__`(모두 `ValueError`): 모든 float가 유한, `cell_m > 0`, `plane_inlier_m > 0`, `band_margin_m >= 0`이고 `2*band_margin_m < opening_height` 관계는 `detect_pockets`에서 prior와 함께 검사, `0 < range_min_m < range_max_m`, `floor_z_m >= 0`, `front_margin_m > 0`, `0 < occluded_front_frac <= 1`, `0 < open_behind_frac <= 1`, `0 < width_mismatch_frac < 1`, `ransac_iterations >= 1`, `min_plane_points >= 1`, `max_plane_candidates >= 1`.

- [ ] **Step 1: 시험 작성**(Task 2의 자체 점검 시험 뒤에 이어 쓴다)

```python
def detect(scene, **overrides):
    params = (
        dataclasses.replace(_DEFAULT_PARAMS, **overrides)
        if overrides
        else _DEFAULT_PARAMS
    )
    return detect_pockets(scene, load_pallet_prior(REPO_PRIOR), params)


def assert_pockets_match(obs, truth, tol_m=0.02):
    assert math.dist(obs.left.center_m, truth["left_centre_m"]) <= tol_m
    assert math.dist(obs.right.center_m, truth["right_centre_m"]) <= tol_m


def test_a_straight_pallet_is_detected_within_two_centimetres(pallet_scene):
    scene, truth = pallet_scene(
        centre_xy_m=(2.5, 0.0), yaw_rad=0.0, opening_width_m=0.24
    )
    obs = detect(scene).observation
    assert obs.status == "valid" and obs.frame_id == "base_link"
    assert_pockets_match(obs, truth)
    assert abs(yaw_difference_rad(obs.insertion_yaw_rad, 0.0)) < math.radians(2)
    assert obs.position_sigma_m is None and obs.yaw_sigma_rad is None
    assert obs.stamp_ns == scene.stamp_ns and obs.clock_domain == scene.clock_domain
    assert obs.source_provenance == scene.source_provenance


@pytest.mark.parametrize("yaw", [-0.4, -0.2, 0.2, 0.4])
def test_rotated_pallets_keep_the_insertion_axis_sign(pallet_scene, yaw):
    scene, truth = pallet_scene(centre_xy_m=(2.6, 0.1), yaw_rad=yaw)
    obs = detect(scene).observation
    assert obs.status == "valid"
    # a double sign flip would land on yaw + pi; the wrapped difference catches it
    assert abs(yaw_difference_rad(obs.insertion_yaw_rad, yaw)) < math.radians(2)
    left_axis = (-math.sin(yaw), math.cos(yaw), 0.0)
    delta = np.array(obs.left.center_m) - np.array(obs.right.center_m)
    assert float(np.dot(delta, left_axis)) > 0


@pytest.mark.parametrize(
    "centre_x,width", [(2.2, 0.20), (2.5, 0.24), (3.0, 0.28), (2.37, 0.22)]
)
def test_accuracy_holds_across_distance_and_grid_phase(pallet_scene, centre_x, width):
    scene, truth = pallet_scene(centre_xy_m=(centre_x, 0.0), opening_width_m=width)
    obs = detect(scene).observation
    assert obs.status == "valid"
    assert_pockets_match(obs, truth)
    spacing = math.dist(obs.left.center_m, obs.right.center_m)
    assert spacing == pytest.approx(0.10 + width, abs=0.03)


def test_the_z_band_margin_keeps_deck_edge_points_out_of_the_opening_columns():
    # Measured on catalogue v1: a naive band [deck, height-deck] fails all 60 positive
    # scenes because deck-top returns sit at z ~ 0.0501-0.0514. Reproducing that through
    # ray tracing depends on a single pixel row landing in a ~1 px window, so this is a
    # direct unit test of the column builder with an explicit point cloud instead.
    prior = load_pallet_prior(REPO_PRIOR)
    deck_edge = np.array([[0.0, y, 0.0505] for y in np.linspace(-0.35, 0.35, 71)])
    spacer = np.array(
        [[0.0, y, z] for y in (-0.0, 0.30, -0.30) for z in np.linspace(0.07, 0.23, 17)]
    )
    points = np.vstack([deck_edge, spacer])
    naive = occupied_columns(
        points, prior, dataclasses.replace(_DEFAULT_PARAMS, band_margin_m=0.0)
    )
    guarded = occupied_columns(points, prior, _DEFAULT_PARAMS)
    assert count_interior_gaps(naive) == 0  # deck edge fills every column
    assert count_interior_gaps(guarded) == 2  # the two openings reappear


def test_space_outside_the_pallet_is_not_mistaken_for_openings(pallet_scene):
    scene, truth = pallet_scene(centre_xy_m=(2.2, 0.0), opening_width_m=0.20)
    obs = detect(scene).observation
    assert obs.status == "valid"
    assert_pockets_match(obs, truth)
    # fake outer gaps would put the centres about 0.6 m apart instead of 0.30 m
    assert math.dist(obs.left.center_m, obs.right.center_m) == pytest.approx(
        0.30, abs=0.03
    )


def test_an_occluded_pocket_is_invalid_not_a_silent_guess(pallet_scene):
    scene, truth = pallet_scene(
        occluder={
            "side": "left",
            "gap_m": 0.4,
            "width_frac": 0.4,
            "depth_m": 0.10,
            "height_m": 0.80,
        }
    )
    assert (
        opening_ray_fractions(scene, truth, "left")["front"] > 0.5
    )  # fixture precondition
    obs = detect(scene).observation
    assert obs.status == "invalid" and obs.reason.startswith("pocket_occluded:left")
    assert obs.left is None and obs.right is None and obs.insertion_yaw_rad is None
    assert obs.position_sigma_m is None


def test_unknown_returns_in_the_openings_are_not_treated_as_open(pallet_scene):
    scene, _ = pallet_scene(openings_unknown=True)
    obs = detect(scene).observation
    assert obs.status == "invalid" and "ambiguous" in obs.reason


def test_a_partial_unknown_patch_does_not_invent_an_opening(pallet_scene):
    scene, truth = pallet_scene(
        centre_xy_m=(2.5, 0.0), unknown_patch=(280, 200, 320, 260)
    )
    obs = detect(scene).observation
    assert obs.status in ("valid", "invalid")
    if obs.status == "valid":
        assert_pockets_match(obs, truth, tol_m=0.03)
    else:
        assert "ambiguous" in obs.reason or "occluded" in obs.reason


def test_a_few_stray_points_inside_an_opening_do_not_destroy_the_detection(
    pallet_scene,
):
    scene, truth = pallet_scene(
        extra_box={"centre_xy_m": (2.19, 0.17), "size_m": (0.02, 0.02, 0.02)}
    )
    obs = detect(scene).observation
    assert obs.status == "valid"
    assert_pockets_match(obs, truth, tol_m=0.03)


def test_scenes_without_a_target_pallet_report_no_pallet(pallet_scene):
    for kwargs in ({"pallet": False}, {"pallet": False, "lookalike": True}):
        scene, _ = pallet_scene(**kwargs)
        obs = detect(scene).observation
        assert obs.status == "no_pallet" and obs.reason


def test_all_unknown_depth_is_invalid_not_no_pallet(pallet_scene):
    scene, _ = pallet_scene()
    blank = dataclasses.replace(scene, depth_m=np.full_like(scene.depth_m, np.nan))
    obs = detect(blank).observation
    assert obs.status == "invalid" and obs.reason == "insufficient_points"


def test_a_larger_competing_box_face_does_not_win_over_the_pallet(pallet_scene):
    scene, truth = pallet_scene(
        centre_xy_m=(3.0, 0.0),
        extra_box={"centre_xy_m": (2.0, 1.1), "size_m": (0.5, 1.4, 1.2)},
    )
    obs = detect(scene).observation
    assert obs.status == "valid"
    assert_pockets_match(obs, truth, tol_m=0.03)


def test_openings_outside_the_prior_width_range_are_reported_as_a_mismatch(
    pallet_scene,
):
    scene, _ = pallet_scene(opening_width_m=0.34)  # prior allows 0.18-0.30
    obs = detect(scene).observation
    assert obs.status == "invalid" and obs.reason == "opening_width_mismatch"


def test_a_pallet_closer_than_the_search_range_gives_a_defined_result(pallet_scene):
    scene, _ = pallet_scene(
        centre_xy_m=(1.4, 0.0)
    )  # front face ~0.35 m from the camera
    obs = detect(scene).observation
    assert obs.status in ("no_pallet", "invalid") and obs.reason


def test_diagnostics_report_plane_quality_ray_counts_and_the_prior_flag(pallet_scene):
    scene, _ = pallet_scene()
    diag = detect(scene).diagnostics
    assert diag.plane_inlier_count >= 300
    assert diag.plane_residual_p95_m is not None and diag.plane_residual_p95_m < 0.01
    left = diag.opening_rays["left"]
    assert left.total == left.front + left.behind + left.near_plane + left.unknown
    assert left.behind_fraction > 0.3
    assert diag.centre_height_is_prior is True
    assert diag.boundary_resolution_m == pytest.approx(0.01)
    assert diag.seed == 20260913 and diag.elapsed_s > 0


def test_detection_is_deterministic_for_a_fixed_seed(pallet_scene):
    scene, _ = pallet_scene(centre_xy_m=(2.7, -0.3), yaw_rad=0.25)
    assert detect(scene).observation.to_json() == detect(scene).observation.to_json()


@pytest.mark.parametrize(
    "overrides",
    [
        {"cell_m": 0.0},
        {"range_min_m": 5.0, "range_max_m": 1.0},
        {"occluded_front_frac": 1.5},
        {"ransac_iterations": 0},
        {"plane_inlier_m": float("nan")},
    ],
)
def test_invalid_detector_params_are_rejected(overrides):
    with pytest.raises(ValueError):
        dataclasses.replace(_DEFAULT_PARAMS, **overrides)
```

- [ ] **Step 2: 실패 확인** — `python -m pytest tests/unit/perception/test_pocket_detector.py -q -p no:cacheprovider -W error`.
- [ ] **Step 3: 구현** — 설계 §4.2의 1–9단계. 내부 함수를 책임별로 나눈다: `_base_points`(픽셀 대응 유지), `_filter_workspace`, `_vertical_plane_candidates`, `_refit_vertical`, `occupied_columns`(시험이 직접 부르므로 공개), `count_interior_gaps`(공개), `_opening_candidates`(부분 후보도 반환), `_classify_opening_rays`(바닥 제거 **전**의 원본 depth 사용, 분모는 후보 사각형 광선 전체), `_build_observation`. RANSAC은 `np.random.default_rng(params.seed)`.
- [ ] **Step 4: 통과 확인** + Ruff.

### Task 4: 평가 계산

**Files:** Create `src/forklift_core/perception/evaluation.py`, `tests/unit/perception/test_evaluation.py`

**Interfaces — Produces:**

```python
POSITIVE_CATEGORIES = ("positive", "occluded")
NEGATIVE_CATEGORIES = ("negative_no_pallet", "negative_lookalike")
POSITION_TOLERANCE_M = (
    0.20  # detection-correspondence threshold, not an accuracy target
)
YAW_TOLERANCE_RAD = 0.35
TARGET_POSITION_P95_M = 0.020
TARGET_YAW_P95_RAD = 0.0349  # 2 degrees
TARGET_DETECTION_RATE = 0.95


class Outcome(str, Enum):
    TRUE_POSITIVE = "true_positive"
    WRONG_POSE = "wrong_pose"
    FALSE_NEGATIVE = "false_negative"
    FALSE_POSITIVE = "false_positive"
    TRUE_NEGATIVE = "true_negative"
    INVALID = "invalid"


@dataclass(frozen=True)
class SceneResult:
    scene_id: str
    category: str
    split: str
    truth_status: str
    estimate_status: str
    outcome: Outcome
    left_error_m: float | None
    right_error_m: float | None
    position_error_m: (
        float | None
    )  # max(left, right); None unless both truth and estimate are valid
    yaw_error_rad: float | None  # absolute wrapped difference
    reason: str | None
    elapsed_s: float | None


def evaluate_scene(
    sample,
    observation,
    *,
    elapsed_s=None,
    position_tolerance_m=POSITION_TOLERANCE_M,
    yaw_tolerance_rad=YAW_TOLERANCE_RAD,
) -> SceneResult: ...


def summarize(results: Sequence[SceneResult]) -> dict: ...
```

**반환 계약(고정):**

```python
{
  "scene_count": int,
  "splits": ["dev"],                       # sorted unique splits present
  "counts": {category: {outcome_value: int}},          # every observed category; all six outcomes as keys, 0 when absent
  "detection_rate": {"positive": float|None, "occluded": float|None},
      # TP / (all scenes of that category, including wrong_pose, false_negative and invalid)
  "false_positive_rate": {"negative_no_pallet": float|None, "negative_lookalike": float|None},
      # FP / (all scenes of that category, including invalid)
  "position_error_m": {
      "all_valid": stats, "true_positive": stats,               # over positive+occluded
      "positive": stats, "occluded": stats},                    # per category, all valid outputs
  "yaw_error_rad": {same four keys},
  "elapsed_s": stats,                                           # over results with a finite elapsed_s
  "targets": {
      "position_p95_m": 0.020, "yaw_p95_rad": 0.0349, "detection_rate": 0.95,
      "met": {"position_p95_positive": bool, "yaw_p95_positive": bool,
              "detection_rate_positive": bool}},
}
# stats = {"p50": float|None, "p95": float|None, "max": float|None, "n": int}
```

규칙: `stats`의 분위수는 `numpy.percentile(..., method="linear")`, `n == 0`이면 세 값 모두 `None`. 비율의 분모가 0이면 `None`. `targets.met`의 세 항목은 **`positive` 범주**를 대상으로 하고, 대응하는 표본이 없으면 `False`(통과로 치지 않는다). 음성 장면의 `false_positive`는 정답 포켓이 없으므로 위치·yaw 오차가 `None`이다. `elapsed_s`가 `None`이거나 유한하지 않으면 시간 집계에서 제외한다(음수는 `ValueError`). 여러 split이 섞여 들어오면 `splits`에 모두 싣고 집계는 합산한다(호출자가 split을 골라 넘기는 것이 기본이다).

- [ ] **Step 1: 시험 작성**

```python
def test_an_accurate_estimate_on_a_positive_scene_is_a_true_positive():
    result = evaluate_scene(sample("positive"), shifted(OBS, 0.01), elapsed_s=0.2)
    assert result.outcome is Outcome.TRUE_POSITIVE
    assert result.position_error_m == pytest.approx(0.01, abs=1e-9)
    assert result.yaw_error_rad == pytest.approx(0.0, abs=1e-9)
    assert result.elapsed_s == 0.2


def test_the_scene_position_error_is_the_worse_of_the_two_pockets():
    result = evaluate_scene(sample("positive"), shifted(OBS, 0.01, right_extra=0.03))
    assert result.left_error_m == pytest.approx(0.01)
    assert result.right_error_m == pytest.approx(0.04)
    assert result.position_error_m == pytest.approx(0.04)


def test_a_valid_but_far_estimate_is_wrong_pose_not_a_true_positive():
    result = evaluate_scene(sample("positive"), shifted(OBS, 0.5))
    assert result.outcome is Outcome.WRONG_POSE and result.position_error_m > 0.2


def test_an_error_exactly_at_the_threshold_still_counts_as_a_true_positive():
    result = evaluate_scene(sample("positive"), shifted(OBS, 0.20))
    assert result.outcome is Outcome.TRUE_POSITIVE


@pytest.mark.parametrize("yaw", [-1.0, 1.0])
def test_yaw_is_compared_by_absolute_wrapped_difference(yaw):
    result = evaluate_scene(sample("positive"), rotated(OBS, yaw))
    assert result.outcome is Outcome.WRONG_POSE
    assert result.yaw_error_rad == pytest.approx(1.0, abs=1e-9)


def test_yaw_differences_across_pi_wrap_to_the_short_way_round():
    result = evaluate_scene(
        sample("positive", truth_yaw=math.pi - 0.05), rotated(OBS, -math.pi + 0.05)
    )
    assert result.yaw_error_rad == pytest.approx(0.10, abs=1e-9)
    assert result.outcome is Outcome.TRUE_POSITIVE


@pytest.mark.parametrize(
    "category,estimate,expected",
    [
        ("positive", "valid_close", Outcome.TRUE_POSITIVE),
        ("positive", "valid_far", Outcome.WRONG_POSE),
        ("positive", "no_pallet", Outcome.FALSE_NEGATIVE),
        ("positive", "invalid", Outcome.INVALID),
        ("occluded", "valid_close", Outcome.TRUE_POSITIVE),
        ("occluded", "valid_far", Outcome.WRONG_POSE),
        ("occluded", "no_pallet", Outcome.FALSE_NEGATIVE),
        ("occluded", "invalid", Outcome.INVALID),
        ("negative_no_pallet", "valid_close", Outcome.FALSE_POSITIVE),
        ("negative_no_pallet", "no_pallet", Outcome.TRUE_NEGATIVE),
        ("negative_no_pallet", "invalid", Outcome.INVALID),
        ("negative_lookalike", "valid_close", Outcome.FALSE_POSITIVE),
        ("negative_lookalike", "no_pallet", Outcome.TRUE_NEGATIVE),
        ("negative_lookalike", "invalid", Outcome.INVALID),
    ],
)
def test_the_full_judgment_table(category, estimate, expected):
    assert evaluate_scene(sample(category), estimate_for(estimate)).outcome is expected


def test_a_false_positive_on_a_negative_scene_has_no_pose_error():
    result = evaluate_scene(sample("negative_lookalike"), OBS)
    assert result.outcome is Outcome.FALSE_POSITIVE
    assert result.position_error_m is None and result.yaw_error_rad is None


def test_detection_rate_uses_the_whole_category_as_denominator():
    results = [
        evaluate_scene(sample("positive", scene_id=f"s{i:03d}"), shifted(OBS, 0.01))
        for i in range(17)
    ]
    results.append(
        evaluate_scene(sample("positive", scene_id="s018"), estimate_for("invalid"))
    )
    summary = summarize(results)
    assert summary["detection_rate"]["positive"] == pytest.approx(17 / 18)
    assert summary["targets"]["met"]["detection_rate_positive"] is False
    assert summary["counts"]["positive"]["invalid"] == 1
    assert summary["counts"]["positive"]["false_negative"] == 0


def test_false_positive_rate_counts_invalid_negatives_in_the_denominator():
    results = [
        evaluate_scene(sample("negative_no_pallet", scene_id="s001"), OBS),
        evaluate_scene(
            sample("negative_no_pallet", scene_id="s002"), estimate_for("invalid")
        ),
        evaluate_scene(
            sample("negative_no_pallet", scene_id="s003"), estimate_for("no_pallet")
        ),
    ]
    assert summarize(results)["false_positive_rate"][
        "negative_no_pallet"
    ] == pytest.approx(1 / 3)


def test_error_distribution_covers_every_valid_output_not_only_successes():
    close = evaluate_scene(sample("positive", scene_id="s001"), shifted(OBS, 0.01))
    far = evaluate_scene(sample("positive", scene_id="s002"), shifted(OBS, 0.5))
    summary = summarize([close, far])
    assert summary["position_error_m"]["all_valid"]["n"] == 2
    assert summary["position_error_m"]["all_valid"]["max"] == pytest.approx(0.5)
    assert summary["position_error_m"]["true_positive"]["n"] == 1
    assert summary["position_error_m"]["true_positive"]["max"] == pytest.approx(0.01)
    assert summary["position_error_m"]["positive"]["n"] == 2


def test_empty_samples_give_null_quantiles_and_never_meet_a_target():
    summary = summarize([evaluate_scene(sample("positive"), estimate_for("no_pallet"))])
    stats = summary["position_error_m"]["all_valid"]
    assert stats["n"] == 0 and stats["p50"] is None and stats["p95"] is None
    assert summary["targets"]["met"]["position_p95_positive"] is False
    assert summary["targets"]["met"]["yaw_p95_positive"] is False


def test_summarize_of_no_results_is_defined():
    summary = summarize([])
    assert summary["scene_count"] == 0 and summary["counts"] == {}
    assert summary["detection_rate"]["positive"] is None
    assert all(value is False for value in summary["targets"]["met"].values())


def test_elapsed_times_without_a_measurement_are_excluded():
    a = evaluate_scene(
        sample("positive", scene_id="s001"), shifted(OBS, 0.01), elapsed_s=0.4
    )
    b = evaluate_scene(sample("positive", scene_id="s002"), shifted(OBS, 0.01))
    assert summarize([a, b])["elapsed_s"]["n"] == 1
```

(`sample(category, *, truth_status=…, truth_yaw=0.0, split="dev", scene_id="s001")`, `estimate_for(kind)`, `shifted(obs, dx, *, right_extra=0.0)`, `rotated(obs, yaw)`, `OBS`는 같은 시험 파일의 헬퍼·상수이며 Step 3에서 함께 구현한다. `sample`은 `SceneSample(input=…, ground_truth=…, scene={"scene_id":…, "category":…, "split":…})`를 만들고, 인식기를 부르지 않으므로 `input`은 최소 크기 더미 `SceneInput`이면 된다.)

- [ ] **Step 2: 실패 확인.** **Step 3: 구현.** **Step 4: 통과 확인** + Ruff.

### Task 5: 파일 왕복 통합시험

**Files:** Create `tests/integration/test_detector_pipeline.py`

- [ ] **Step 1: 시험 작성**

```python
def test_a_synthetic_scene_survives_the_v1_file_format_and_is_still_detected(
    tmp_path, pallet_scene
):
    scene, truth = pallet_scene(
        centre_xy_m=(2.5, 0.1), yaw_rad=0.2, unknown_patch=(10, 10, 40, 40)
    )
    scene_dir = write_v1_scene(tmp_path / "s001", scene, truth)
    sample = load_scene_sample(scene_dir)
    # the unknown patch must survive millimetre encoding as NaN, not as a 0 mm reading
    assert np.isnan(sample.input.depth_m[20, 20])
    finite = np.isfinite(scene.depth_m) & np.isfinite(sample.input.depth_m)
    assert np.abs(sample.input.depth_m[finite] - scene.depth_m[finite]).max() <= 0.0005
    assert np.isnan(sample.input.depth_m).sum() == np.isnan(scene.depth_m).sum()

    result = detect_pockets(sample.input, load_pallet_prior(REPO_PRIOR))
    outcome = evaluate_scene(
        sample, result.observation, elapsed_s=result.diagnostics.elapsed_s
    )
    assert outcome.outcome is Outcome.TRUE_POSITIVE
    assert outcome.position_error_m < 0.03


def test_depth_beyond_the_encodable_range_is_refused_by_the_writer(
    tmp_path, pallet_scene
):
    scene, truth = pallet_scene()
    too_far = dataclasses.replace(
        scene, depth_m=np.where(np.isfinite(scene.depth_m), 70.0, scene.depth_m)
    )
    with pytest.raises(ValueError):
        write_v1_scene(tmp_path / "s002", too_far, truth)
```

`write_v1_scene(scene_dir, scene_input, truth)`는 같은 파일의 헬퍼로 `docs/interfaces/scene-dataset.md` v1 형식을 정확히 만든다:

- `depth_mm.png`: `round(depth_m × 1000)` uint16. NaN·±Inf·≤ 0 → 0. **유한 양수가 65.535 m를 넘으면 `ValueError`**(잘라내지 않는다).
- `rgb.png`: 8-bit RGB.
- `depth_meta.json`: `{"unit": "mm", "meters_per_unit": 0.001, "unknown_value": 0, "kind": "optical_axis_z"}`.
- `camera_info.json`: 정확히 12키(`frame_id, stamp_ns, width, height, distortion_model, d, k, r, p, binning_x, binning_y, roi`). `d`는 0 다섯 개, `k` 길이 9, `r`은 단위행렬 9, `p`는 `[K|0]` 12, `binning_x = binning_y = 0`, `roi`는 정확히 5키이며 네 정수 **모두 0**·`do_rectify: False`(영상 크기를 ROI에 넣으면 로더가 거부한다).
- `tf.json`: 정확히 5키 `{target_frame: "base_link", source_frame: "camera_optical_frame", translation_m, quaternion_xyzw, origin: "received_tf_static"}`. 시험용 구성이며 실제 TF 수신 증거가 아님을 주석에 적는다.
- `ground_truth.json`: `truth`로 `Pocket`·`PocketObservation`을 만들어 `to_json()`(`stamp_ns`·`clock_domain`은 입력과 동일, `source_provenance: "synthetic_ground_truth"`, σ는 `None`, `reason: None`).
- `scene.json`: 필수 7키 `{scene_id, catalogue_version, category, split, stamp_ns, clock_domain, source_provenance}`.

- [ ] **Step 2: 실패 확인.** **Step 3: 구현.** **Step 4: 통과 확인**(`python -m pytest tests/integration -q …` 단독 실행 포함).

### Task 6: 검증·커밋 (Claude)

- [ ] 전체 회귀(`-m 'not rendering'`) 통과 개수, `ruff check .`, `ruff format --check .`.
- [ ] 커밋 `feat(perception): add the geometric pocket detector and evaluation`, `main` ff-merge, push, 원격 repo 갱신.
- [ ] 실데이터 실행(dev/eval)은 **2단계 계획**이다. 이 단계에서는 하지 않는다.

## 자체 검토

- **Spec 대조:** §4.1 입력·prior·params → Task 1·3; §4.2 1–9단계 → Task 3(단계별 내부 함수와 반례 시험); §4.3 결정론 → `test_detection_is_deterministic_for_a_fixed_seed`; §5 판정표 14조합 → `test_the_full_judgment_table`; 분모·분포·null → Task 4의 여섯 시험; §9 광선 추적 fixture·필수 반례 → Task 2·3; 작은 통합 → Task 5. CLI·overlay·MP4·dev 튜닝·eval은 2단계(의도).
- **Codex 검토 반영:** Task 0 HEAD 조건, 640×480·fx/fy/cx/cy 명시, 헬퍼 자체 점검의 "벽까지 도달" 기대 제거, 덱 반례를 `occupied_columns` 단위시험으로 전환(광선 추적으로는 픽셀 한 행 차이에 좌우됨을 직접 확인), 위치 허용오차를 유클리드 거리로, 중심 간격 `0.10 + w` 정정, 가림 fixture의 상자 기하 명시와 사전 조건 확인, B008 회피용 `_DEFAULT_PARAMS`, `DetectorParams` 검증, 평가 반환 계약 확정, 판정표 14조합·문턱 경계·±π·FP 오차 `None`·빈 입력, `write_v1_scene`의 키 집합·GT 직렬화·65.535 m 거부·unknown 왕복, 공유 fixture의 conftest 경로, 함수 길이 제한 삭제, 부분 후보 유지, 광선 분모 고정, 진단 필드 추가, PyYAML을 `dev` extra에, dev 10장면 실행을 2단계로 이동.
- **자리표시자 없음.** 모든 시험이 실제 기대값을 갖는다. 헬퍼(`depth_at`, `midpoint`, `sample`, `estimate_for`, `shifted`, `rotated`, `write_v1_scene`)는 해당 시험 파일에서 Step 3에 함께 구현한다.
- **이름 일관성:** `PalletPrior`·`load_pallet_prior`·`DetectorParams`·`_DEFAULT_PARAMS`·`OpeningRayCounts`·`DetectionDiagnostics`·`DetectionResult`·`detect_pockets`·`occupied_columns`·`count_interior_gaps`·`Outcome`·`SceneResult`·`evaluate_scene`·`summarize`·`make_pallet_scene`·`opening_ray_fractions`가 Interfaces·시험·파일 표에서 동일하다.
