# M1-b 2단계: 장면 카탈로그 생성기와 장면 월드 생성 구현 계획

> **실행자:** 구현은 Codex에 위임한다. 각 Task는 실패 시험 → 실패 확인 → 구현 → 통과 확인 순서로 진행하고 체크박스로 추적한다. Codex 샌드박스 제약(`.git`·anaconda3·네트워크·Docker 쓰기 불가)은 1단계 계획과 같다. 시험은 `/home/light/anaconda3/bin/python`으로 실행한다.

**목표:** 결정론적 생성기로 독립 합성 장면 100개의 카탈로그(`catalogue_v1.yaml`, 정답·가시성 포함)를 만들어 저장소에 고정하고, 카탈로그 항목 하나를 Gazebo SDF·bridge·TF 설정으로 바꾸는 장면 월드 생성기를 만든다. 기존 reference experiment 생성기는 출력 바이트 동일을 유지한 채 공용 SDF 헬퍼를 공유한다.

**아키텍처:** `sim/gazebo/sdf_parts.py`(SDF 헬퍼, ROS·코어 의존 없음) ← `build_sensor_world.py`(기존) / `build_scene_world.py`(신규). `tools/generate_scene_catalogue.py`는 호스트에서 실행하며 `forklift_core.perception.pocket_observation`으로 정답 JSON을 만든다. SDF 생성기는 컨테이너에서 실행되므로 `forklift_core`를 import하지 않고 카탈로그 YAML만 읽는다.

**기술 스택:** Python ≥ 3.10, NumPy, PyYAML, `xml.etree`, pytest, Ruff. Gazebo 실행은 3단계.

**Spec:** `docs/design/2026-09-11-pocket-observation-and-scene-set.md` §5(카탈로그·기하), §6(월드 생성), §10(시험). 1단계 결과(`PocketObservation`, `to_json`)를 사용한다.

## 전역 제약

- 기준: 1단계가 `main`에 병합된 revision. 브랜치 `feat/scene-catalogue`.
- 팔레트 기하는 설계 §5.1을 그대로 쓴다: C=(x, y, 0), a=(cos ψ, sin ψ, 0), ℓ=(−sin ψ, cos ψ, 0), u=(0,0,1). 외형 0.6(a)×0.8(ℓ)×0.30, 덱 0.05, 개구 높이 0.20 고정, 중앙 지지대 0.10, 바깥 지지대 s_o=(0.8−2w−0.1)/2, 개구 폭 w∈[0.20, 0.28], 개구 중심 `C − 0.3a ± dℓ + 0.15u`, d=0.05+w/2.
- 카메라(설계 승인): 640×480, `horizontal_fov` 1.204 rad, 5 Hz, base 기준 (0.75, 0, 0.5), optical quaternion xyzw (−0.5, 0.5, −0.5, 0.5). 핀홀 fx=fy=(640/2)/tan(0.602)=**465.741**, cx=320, cy=240(생성기의 시야 계산용; 실제 CameraInfo는 캡처 시 Gazebo가 준다).
- 카탈로그는 생성 후 **파일로 고정**하고 시험은 파일을 읽는다. 재생성은 `catalogue_v2.yaml`처럼 새 파일로만.
- 기존 `build_sensor_world.py`의 출력(`sensor_world.sdf`, `bridge.yaml`, `transforms.yaml`)은 리팩터링 전후 SHA-256이 같아야 한다(아래 값). `scene_config.yaml` 사본과 고정 config 거부도 유지.
- 새 경로에 LiDAR·`/scan`은 넣지 않는다. 가림은 정답을 바꾸지 않는다. 음성 두 범주는 `pallet: null`.
- 시험 기대값은 생성 결과를 복사하지 않고 독립 계산한 문자 그대로의 값을 쓴다.
- 기존 회귀·Ruff 통과. 기능 외 리팩터링 금지.

**회귀 기준 해시(2026-09-11, 현재 `build_sensor_world.py`, `scene_config.yaml` 기준; Claude가 conda base로 `--output <scratch>` 실행 후 `sha256sum`으로 계산):**

| 파일 | SHA-256 |
|---|---|
| `sensor_world.sdf` | `75dda4e7ab0c04e2924d3d5da1abc1c44282329e01ae6eeb5f4315beea1d0adc` |
| `bridge.yaml` | `3b83e534d363dc05a54bac95ab90938fdf6e5ad7d13b75198d09273760329413` |
| `transforms.yaml` | `018b2669d9b12aa9f10db5b5e231b7db55be2c63d365561cdfbedb1514714e00` |

## 파일 구조

| 파일 | 책임 |
|---|---|
| `sim/gazebo/sdf_parts.py` | `element`, `rotation_rpy`, `origin_matrix`, `pose_text`, `add_urdf_visuals`, `box(link, name, center, size, color, yaw=0.0)`, `add_world_skeleton(root, name, ambient, background, light_direction, light_diffuse)`, `add_rgbd_camera(model, translation_m, width, height, horizontal_fov_rad, rate_hz)`, `add_gpu_lidar(model, translation_m, samples, rate_hz)`, `write_bridge(path, topics)`, `write_transforms(path, transforms)` |
| `sim/gazebo/build_sensor_world.py` | 기존 동작 유지, 헬퍼는 `sdf_parts`에서 import(출력 바이트 동일) |
| `sim/gazebo/build_scene_world.py` | `load_catalogue(path) -> dict`, `scene_entry(catalogue, scene_id) -> dict`, `generate_scene(catalogue_path, scene_id, output) -> dict` → `scene_world.sdf`, `bridge.yaml`, `transforms.yaml`, `scene.yaml` |
| `tools/generate_scene_catalogue.py` | `pallet_ground_truth(x_m, y_m, yaw_rad, opening_width_m) -> PocketObservation`, `opening_corners(x_m, y_m, yaw_rad, opening_width_m) -> dict[str, list[tuple]]`, `project_to_image(point_base, camera) -> tuple[u, v, z]`, `openings_in_view(...) -> dict`, `sample_catalogue(seed, count) -> dict`, CLI `--seed --count --output` |
| `sim/gazebo/scenes/catalogue_v1.yaml` | 고정 산출물(seed 20260911, count 100) |
| `tests/simulation/test_gazebo_sensor_world.py` | 추가: 회귀 해시 시험 |
| `tests/simulation/test_scene_catalogue.py` | 정답 독립 기대값·시야·분포·결정론·거부 |
| `tests/simulation/test_build_scene_world.py` | 장면 SDF 구조 시험 |
| `sim/gazebo/README.md` | 새 경로 절 추가(카탈로그·생성기·회귀 기준) |

---

### Task 0: 기준 상태

- [ ] `main`이 1단계 병합 revision이고 `git status --short` 비어 있음. Claude가 `git switch -c feat/scene-catalogue` 후 위임.
- [ ] 기준 회귀 통과 개수 기록.

### Task 1: SDF 헬퍼 분리와 회귀 고정

**Files:** `sim/gazebo/sdf_parts.py`(신규), `sim/gazebo/build_sensor_world.py`(수정), `tests/simulation/test_gazebo_sensor_world.py`(추가)

- [ ] **Step 1 (RED, 회귀 고정):** 기존 시험 파일에 추가한다. 이 시험은 리팩터링 전에도 통과해야 하며(기준 해시가 현재 출력), 리팩터링 후에도 통과해야 한다.

```python
import hashlib

EXPECTED_SHA256 = {
    "sensor_world.sdf": "75dda4e7ab0c04e2924d3d5da1abc1c44282329e01ae6eeb5f4315beea1d0adc",
    "bridge.yaml": "3b83e534d363dc05a54bac95ab90938fdf6e5ad7d13b75198d09273760329413",
    "transforms.yaml": "018b2669d9b12aa9f10db5b5e231b7db55be2c63d365561cdfbedb1514714e00",
}


def test_reference_experiment_outputs_are_byte_identical_to_the_pinned_baseline(
    builder, tmp_path
):
    builder.generate(ROOT / "sim/gazebo/scene_config.yaml", tmp_path)
    observed = {
        name: hashlib.sha256((tmp_path / name).read_bytes()).hexdigest()
        for name in EXPECTED_SHA256
    }
    assert observed == EXPECTED_SHA256
```

- [ ] **Step 2:** 실행해 **통과**함을 먼저 확인한다(기준이 현재 출력이므로). 실패하면 해시를 다시 계산하지 말고 Claude에게 보고한다.
- [ ] **Step 3:** `sdf_parts.py`를 만들고 `build_sensor_world.py`의 함수 6개를 옮긴다. `box`는 `yaw=0` **정수 0** 기본 인자를 추가하고 pose 문자열은 `" ".join(map(str, [*center, 0, 0, yaw]))`로 유지한다(`str(0.0)`은 `"0.0"`이라 기본값을 `0.0`으로 두면 기존 `"... 0 0 0"`과 달라져 회귀 해시가 깨진다 — 2026-09-11 Codex 검토 지적). 장면 생성기가 yaw를 넘길 때만 float가 들어간다. `add_world_skeleton`, `add_rgbd_camera`, `add_gpu_lidar`, `write_bridge`, `write_transforms`는 기존 `generate()` 본문을 그대로 잘라 만든다(요소 순서·문자열 포맷 변경 금지). `build_sensor_world.py`는 `from sdf_parts import ...`가 아니라 **직접 실행과 `spec_from_file_location` 양쪽에서 동작**하도록 `sys.path`를 건드리지 않고 같은 디렉터리 모듈을 `spec = importlib.util.spec_from_file_location("forklift_sdf_parts", Path(__file__).with_name("sdf_parts.py"))` → `module_from_spec` → `exec_module` 순서로 읽는 작은 `_load_parts()`를 둔다(인자 하나짜리 호출은 None을 돌려준다). `ET.indent(root)`는 기존처럼 **전체 트리 조립 후 write 직전**에 한 번만 호출한다(skeleton 단계로 옮기면 공백이 달라져 해시가 깨진다)(컨테이너에서 `python3 sim/gazebo/build_sensor_world.py`로 실행되므로 패키지 import를 가정할 수 없다).
- [ ] **Step 4 (GREEN):** 회귀 해시 시험과 기존 두 시험 통과. `python3 sim/gazebo/build_sensor_world.py --output <tmp>`를 직접 실행해도 같은 해시.

### Task 2: 카탈로그 생성기 (`tools/generate_scene_catalogue.py`)

**Interfaces:**

```python
CAMERA = {"width": 640, "height": 480, "horizontal_fov_rad": 1.204, "rate_hz": 5,
          "translation_m": [0.75, 0.0, 0.5], "optical_quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5]}
PALLET = {"depth_m": 0.6, "width_m": 0.8, "height_m": 0.30, "deck_m": 0.05,
          "center_spacer_m": 0.10, "opening_height_m": 0.20}
RANGES = {"x_m": (2.0, 4.0), "y_m": (-1.0, 1.0), "yaw_rad": (-0.52, 0.52),
          "opening_width_m": (0.20, 0.28), "occluder_gap_m": (0.3, 0.6),
          "occluded_fraction": (0.2, 0.6)}
COUNTS = {"positive": 60, "occluded": 20, "negative_no_pallet": 10, "negative_lookalike": 10}
LIGHTING = [("sun_front", (-1.0, -0.5, -1.0)), ("sun_left", (-0.5, 1.0, -1.0)), ("sun_top", (0.0, 0.0, -1.0))]
DIFFUSE = (0.5, 0.9)
SURFACES = [(".35 .4 .4 1", ".15 .2 .25 1"), (".6 .6 .55 1", ".8 .85 .9 1"),
            (".25 .25 .3 1", ".3 .3 .3 1"), (".5 .45 .35 1", ".55 .65 .75 1")]
DISTRACTORS = [  # name, center_m (base), size_m, color
    ("crate_a", (1.5, 1.6, 0.2), (0.4, 0.4, 0.4), ".5 .5 .5 1"),
    ("crate_b", (4.5, -1.8, 0.3), (0.6, 0.6, 0.6), ".2 .3 .7 1"),
    ("post", (3.0, 2.2, 0.5), (0.3, 0.3, 1.0), ".8 .7 .2 1"),
    ("wall_block", (5.5, 0.0, 0.4), (1.0, 0.5, 0.8), ".7 .2 .2 1"),
    ("bin", (2.2, -2.0, 0.15), (0.5, 0.3, 0.3), ".2 .6 .3 1"),
    ("cube", (4.0, 1.9, 0.25), (0.5, 0.5, 0.5), ".9 .9 .9 1"),
]

def pallet_ground_truth(x_m, y_m, yaw_rad, opening_width_m) -> PocketObservation
def opening_corners(x_m, y_m, yaw_rad, opening_width_m) -> dict  # {"left": [4×(x,y,z)], "right": [...]}
def project_to_image(point_base, camera=CAMERA) -> tuple[float, float, float]  # (u, v, z_optical)
def openings_in_view(x_m, y_m, yaw_rad, opening_width_m, camera=CAMERA, margin_px=4, min_z_m=0.3) -> dict
    # {"left_in_view": bool, "right_in_view": bool, "corners_px": {"left": [[u,v],...], "right": [...]}}
def sample_catalogue(seed: int, count: int = 100) -> dict
```

카탈로그 문서 구조:

```yaml
format_version: 1
catalogue_version: v1
seed: 20260911
generator: tools/generate_scene_catalogue.py
source_provenance: synthetic
camera: {…CAMERA…}
pallet: {…PALLET…}
ranges: {…RANGES…}
scenes:
  - scene_id: s001
    split: dev
    category: positive
    pallet: {x_m: …, y_m: …, yaw_rad: …, opening_width_m: …}   # 음성은 null
    lookalike: null | {x_m, y_m, yaw_rad}
    occluder: null | {side: left|right, gap_m, fraction, center_m: [x,y,z], size_m: [d,w,h], color}
    distractors: [crate_a, …]
    lighting: {name: sun_front, direction: [..], diffuse: 0.9}
    surfaces: {floor: "...", background: "..."}
    visibility: {left_in_view: true, right_in_view: true, occluded_side: null|left|right, occluded_fraction_image: null|float, corners_px: {...}}
    ground_truth: {…PocketObservation.to_json()…}
```

생성 규칙(`random.Random(seed)` 하나만 사용, 호출 순서 고정):
1. 범주 라벨 목록을 `COUNTS`대로 만든 뒤 `rng.shuffle`. split은 범주별로 인덱스를 `rng.shuffle`해 앞 70 %를 `dev`(positive 42/18, occluded 14/6, 음성 각 7/3).
2. 장면마다: 양성·가림은 `x,y,yaw,w`를 `rng.uniform`으로 뽑고 `openings_in_view`가 두 개구부 모두 `True`일 때까지 최대 1000회 거부 샘플링(실패 시 `RuntimeError`). 유사물은 `x,y,yaw`만. 무팔레트는 없음.
3. 가림: `side = rng.choice(["left","right"])`, `gap`∈[0.3, 0.6]·`fraction`∈[0.2, 0.6]을 뽑는다. 상자 중심은 **대상 개구부 중심에서 카메라 원점(0.75, 0, 0.5)을 향하는 시선 위**의 점: `center_xy = opening_xy + gap · unit(camera_xy − opening_xy)`, z 중심 0.40, size = (0.10, fraction·w, 0.80), yaw는 팔레트 yaw. 팔레트 축 방향(−a) 앞에 두면 카메라가 비스듬히 볼 때 시선 밖·화면 밖에 놓이는 반례(x=3, y=1, yaw 0, gap 0.6, fraction 0.2 → 상자 u −99…−56 px)가 있어 시선 위 배치로 바꿨다(Codex 2차 검토). 채택 조건(아니면 가림만 재샘플, 팔레트 유지): 상자 중심 x ≥ 1.05; 상자 8모서리 투영 u구간과 대상 개구부 u구간의 겹침 비율 `occluded_fraction_image`(겹침 폭 / 개구부 u폭) ≥ fraction; 반대쪽 개구부 u구간과의 겹침 ≤ 그 폭의 10 %. `occluded_fraction_image`를 `visibility`에 기록한다. `occluded_fraction_nominal`은 물리 폭 비율이며 영상 비율은 원근 때문에 그 이상이다. color ".45 .3 .5 1".
4. distractor: `rng.randint(0, 2)`개를 프리셋에서 `rng.sample`로 뽑되 팔레트/유사물 중심과 XY 거리 < 1.0 m인 프리셋은 제외.
5. lighting: 프리셋 `rng.choice`, diffuse `rng.choice(DIFFUSE)`. surfaces: `rng.choice`.
6. `ground_truth`: 양성·가림은 `pallet_ground_truth(...).to_json()`(stamp_ns 0, clock_domain synthetic, provenance synthetic_ground_truth, σ 0.0). 음성은 `status: no_pallet`, `reason: "no target pallet in scene"`.
7. `visibility`는 2의 결과에 `occluded_side`와 `occluded_fraction_image`(가림 장면만, 아니면 null)를 더한다. 픽셀 좌표는 소수 둘째 자리로 반올림해 저장.
8. **기본형 정규화:** `sample_catalogue`가 반환하는 dict는 YAML 왕복 후에도 같아야 하므로 tuple·numpy 타입 없이 list·float·int·str·bool·None만 담는다(`RANGES`·`CAMERA` 등 상수도 list로 변환해 넣는다). 시험 `regenerated == data`가 이를 검증한다.
9. **반올림 순서:** 샘플링한 팔레트·가림 파라미터(x, y, yaw, w, gap, fraction)를 먼저 `round(v, 4)`로 반올림하고, 정답·가시성·가림 상자 중심은 **반올림된 파라미터로 계산**한 뒤 `round(v, 6)`으로 저장한다. 저장 직전에 일괄 반올림하면 저장된 파라미터로 정답을 재계산할 때 1e-4를 넘는 차이가 생긴다(Codex 검토 반례). YAML은 `yaml.safe_dump(..., sort_keys=False, allow_unicode=True)`.

- [ ] **Step 1 (RED):** `tests/simulation/test_scene_catalogue.py`.

```python
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
CATALOGUE = ROOT / "sim/gazebo/scenes/catalogue_v1.yaml"


@pytest.fixture
def generator():
    path = ROOT / "tools/generate_scene_catalogue.py"
    assert path.exists(), "implementation is missing"
    spec = importlib.util.spec_from_file_location("scene_catalogue", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    "x,y,yaw,w,left,right",
    [  # independently computed from the §5.1 formula, not copied from the generator
        (3.0, 0.4, 0.0, 0.20, (2.7, 0.55, 0.15), (2.7, 0.25, 0.15)),
        (2.5, 0.2, 0.5, 0.24, (2.1552, 0.2054, 0.15), (2.3182, -0.0930, 0.15)),
        (3.5, -0.6, -0.3, 0.28, (3.2695, -0.3298, 0.15), (3.1573, -0.6929, 0.15)),
    ],
)
def test_ground_truth_matches_hand_computed_opening_centres(
    generator, x, y, yaw, w, left, right
):
    truth = generator.pallet_ground_truth(x, y, yaw, w)
    assert truth.status == "valid" and truth.frame_id == "base_link"
    assert truth.insertion_yaw_rad == pytest.approx(yaw)
    assert truth.left.center_m == pytest.approx(left, abs=5e-5)
    assert truth.right.center_m == pytest.approx(right, abs=5e-5)
    assert truth.left.width_m == w and truth.left.height_m == 0.20
    assert (
        truth.position_sigma_m == 0.0
        and truth.source_provenance == "synthetic_ground_truth"
    )


def test_projection_of_a_known_opening_corner(generator):
    # scene (3.0, 0.4, 0, 0.20): left opening top-outer corner at base (2.7, 0.65, 0.25)
    u, v, z = generator.project_to_image((2.7, 0.65, 0.25))
    assert (u, v, z) == pytest.approx((164.8, 299.7, 1.95), abs=0.1)
    view = generator.openings_in_view(3.0, 0.4, 0.0, 0.20)
    assert view["left_in_view"] and view["right_in_view"]
    assert generator.openings_in_view(1.2, 0.0, 0.0, 0.25)["left_in_view"] is False


def test_fixed_catalogue_has_the_approved_composition():
    data = yaml.safe_load(CATALOGUE.read_text())
    scenes = data["scenes"]
    assert data["format_version"] == 1 and data["catalogue_version"] == "v1"
    assert [s["scene_id"] for s in scenes] == [f"s{i:03d}" for i in range(1, 101)]
    by_category = {}
    for s in scenes:
        by_category.setdefault(s["category"], []).append(s)
    assert {k: len(v) for k, v in by_category.items()} == {
        "positive": 60,
        "occluded": 20,
        "negative_no_pallet": 10,
        "negative_lookalike": 10,
    }
    assert sum(s["split"] == "dev" for s in scenes) == 70
    assert sum(s["split"] == "dev" for s in by_category["positive"]) == 42
    for s in scenes:
        if s["category"] in ("positive", "occluded"):
            p = s["pallet"]
            assert 2.0 <= p["x_m"] <= 4.0 and -1.0 <= p["y_m"] <= 1.0
            assert (
                -0.52 <= p["yaw_rad"] <= 0.52 and 0.20 <= p["opening_width_m"] <= 0.28
            )
            assert s["visibility"]["left_in_view"] and s["visibility"]["right_in_view"]
            assert s["ground_truth"]["status"] == "valid"
        else:
            assert s["pallet"] is None and s["ground_truth"]["status"] == "no_pallet"
        if s["category"] == "occluded":
            occ = s["occluder"]
            assert occ["side"] in ("left", "right") and occ["center_m"][0] >= 1.05
            assert 0.2 <= occ["fraction"] <= 0.6
            assert s["visibility"]["occluded_side"] == occ["side"]
            assert s["visibility"]["occluded_fraction_image"] >= occ["fraction"]
            assert occ["size_m"][2] == pytest.approx(0.80)
        else:
            assert s["occluder"] is None
        assert s["category"] != "negative_lookalike" or s["lookalike"] is not None


def test_occlusion_does_not_alter_ground_truth(generator):
    # same pallet, with and without occluder, must give identical ground truth
    data = yaml.safe_load(CATALOGUE.read_text())
    occluded = next(s for s in data["scenes"] if s["category"] == "occluded")
    p = occluded["pallet"]
    truth = generator.pallet_ground_truth(
        p["x_m"], p["y_m"], p["yaw_rad"], p["opening_width_m"]
    )
    stored = occluded["ground_truth"]
    assert stored["left"]["center_m"] == pytest.approx(truth.left.center_m, abs=1e-6)


def test_generation_is_deterministic_and_reproduces_the_fixed_file(generator):
    data = yaml.safe_load(CATALOGUE.read_text())
    regenerated = generator.sample_catalogue(seed=data["seed"], count=100)
    assert regenerated == data


def test_catalogue_ground_truth_loads_as_pocket_observation():
    from forklift_core.perception.pocket_observation import pocket_observation_from_json

    data = yaml.safe_load(CATALOGUE.read_text())
    for s in data["scenes"]:
        pocket_observation_from_json(s["ground_truth"])  # must not raise
```

- [ ] **Step 2:** 실행 → 파일 없음/AttributeError 실패 확인.
- [ ] **Step 3:** 생성기 구현. `project_to_image`는 R=`rotation_matrix_from_quaternion_xyzw(CAMERA quaternion)`(1단계 함수)로 base→optical을 `R.T @ (p − t)`로 계산하고 `u = fx·x/z + cx`, `v = fy·y/z + cy`. `openings_in_view`는 8모서리 모두 `z ≥ min_z_m`, `margin ≤ u ≤ W−margin`, `margin ≤ v ≤ H−margin`.
- [ ] **Step 4:** `python tools/generate_scene_catalogue.py --seed 20260911 --count 100 --output sim/gazebo/scenes/catalogue_v1.yaml`로 파일을 생성한다. 두 번 실행해 바이트 동일 확인. 파일 크기(KB)를 보고.
- [ ] **Step 5 (GREEN):** 시험 통과. `ruff check tools tests`, `format --check` 통과.

### Task 3: 장면 월드 생성기 (`sim/gazebo/build_scene_world.py`)

**Interfaces:** `generate_scene(catalogue_path: Path, scene_id: str, output: Path) -> dict`. 출력 `scene_world.sdf`, `bridge.yaml`(`/camera/image`, `/camera/depth_image`, `/camera/camera_info`, `/clock`), `transforms.yaml`(카메라 변환 1개), `scene.yaml`(카탈로그 항목 + `catalogue_version` + `camera`). 반환 `{"scene_id", "category", "visual_count", "source_provenance": "synthetic"}`. CLI `--catalogue --scene --output`.

SDF 구성: `add_world_skeleton(...)`(조명 방향·diffuse·표면색은 항목에서), 바닥은 표면 floor 색, 지게차 visual(기존과 같은 `add_urdf_visuals`), 그다음:
- `synthetic_pallet` 모델: `<pose>x y 0 0 0 yaw</pose>`, 링크 안 로컬 좌표로 덱 2개·중앙 지지대(0,0,0.15; 0.6×0.10×0.20)·바깥 지지대 2개(0, ±(0.4−s_o/2), 0.15; 0.6×s_o×0.20). 5개 상자.
- `lookalike` 모델: `<pose>x y 0 0 0 yaw</pose>`, 상자 1개 (0,0,0.15; 0.6×0.8×0.30).
- `occluder` 모델(정적): 항목의 center/size, `<pose>… 0 0 yaw</pose>`(팔레트와 같은 yaw).
- `distractor_<name>` 모델들.
- `synthetic_sensor_rig`: 카메라만(640×480·1.204·5 Hz·translation (0.75,0,0.5)).

- [ ] **Step 1 (RED):** `tests/simulation/test_build_scene_world.py`.

```python
import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
CATALOGUE = ROOT / "sim/gazebo/scenes/catalogue_v1.yaml"


@pytest.fixture
def builder():
    path = ROOT / "sim/gazebo/build_scene_world.py"
    assert path.exists(), "implementation is missing"
    spec = importlib.util.spec_from_file_location("scene_world", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def scene_of(category):
    data = yaml.safe_load(CATALOGUE.read_text())
    return next(s for s in data["scenes"] if s["category"] == category)


def test_positive_scene_places_a_rotated_pallet_with_two_openings(builder, tmp_path):
    scene = scene_of("positive")
    builder.generate_scene(CATALOGUE, scene["scene_id"], tmp_path)
    world = ET.parse(tmp_path / "scene_world.sdf").getroot()
    pallet = world.find(".//model[@name='synthetic_pallet']")
    pose = [float(v) for v in pallet.findtext("pose").split()]
    assert pose[:2] == pytest.approx([scene["pallet"]["x_m"], scene["pallet"]["y_m"]])
    assert pose[5] == pytest.approx(scene["pallet"]["yaw_rad"])
    assert len(pallet.findall(".//collision")) == 5
    w = scene["pallet"]["opening_width_m"]
    outer = [c for c in pallet.findall(".//collision") if "outer" in c.get("name")]
    assert len(outer) == 2
    s_o = (0.8 - 2 * w - 0.1) / 2
    for collision in outer:
        size = [float(v) for v in collision.findtext("geometry/box/size").split()]
        assert size == pytest.approx([0.6, s_o, 0.2], abs=1e-9)
    # Reconstruct both opening centres from the spacer poses (model-local frame)
    # and compare with the catalogue ground truth via the model pose.
    psi = pose[5]
    a = np.array([math.cos(psi), math.sin(psi), 0.0])
    left_axis = np.array([-math.sin(psi), math.cos(psi), 0.0])
    origin = np.array([pose[0], pose[1], 0.0])
    outer_y = sorted(float(c.findtext("pose").split()[1]) for c in outer)
    for side, sign, edge in (
        ("left", 1, outer_y[1] - s_o / 2),
        ("right", -1, outer_y[0] + s_o / 2),
    ):
        local_y = (edge + sign * 0.05) / 2  # between outer spacer and centre spacer
        expected = origin - 0.3 * a + local_y * left_axis + np.array([0.0, 0.0, 0.15])
        assert list(expected) == pytest.approx(
            scene["ground_truth"][side]["center_m"], abs=1e-6
        )
    camera = world.find(".//sensor[@type='rgbd_camera']")
    assert camera.findtext("camera/image/width") == "640"
    assert camera.findtext("camera/horizontal_fov") == "1.204"
    assert world.find(".//sensor[@type='gpu_lidar']") is None
    assert world.find(".//model[@name='occluder']") is None
    bridge = yaml.safe_load((tmp_path / "bridge.yaml").read_text())
    assert {e["ros_topic_name"] for e in bridge} == {
        "/camera/image",
        "/camera/depth_image",
        "/camera/camera_info",
        "/clock",
    }
    transforms = yaml.safe_load((tmp_path / "transforms.yaml").read_text())[
        "transforms"
    ]
    assert [t["child"] for t in transforms] == ["camera_optical_frame"]
    assert (
        yaml.safe_load((tmp_path / "scene.yaml").read_text())["scene_id"]
        == scene["scene_id"]
    )


def test_occluded_scene_adds_an_occluder_in_front_of_the_named_pocket(
    builder, tmp_path
):
    scene = scene_of("occluded")
    builder.generate_scene(CATALOGUE, scene["scene_id"], tmp_path)
    world = ET.parse(tmp_path / "scene_world.sdf").getroot()
    occluder = world.find(".//model[@name='occluder']")
    pose = [float(v) for v in occluder.findtext("pose").split()]
    assert pose[:3] == pytest.approx(scene["occluder"]["center_m"], abs=1e-4)
    assert pose[5] == pytest.approx(scene["pallet"]["yaw_rad"])


def test_negative_scenes_have_no_pallet_openings(builder, tmp_path):
    for category, model in (
        ("negative_no_pallet", None),
        ("negative_lookalike", "lookalike"),
    ):
        out = tmp_path / category
        builder.generate_scene(CATALOGUE, scene_of(category)["scene_id"], out)
        world = ET.parse(out / "scene_world.sdf").getroot()
        assert world.find(".//model[@name='synthetic_pallet']") is None
        if model:
            assert (
                len(world.find(f".//model[@name='{model}']").findall(".//collision"))
                == 1
            )


def test_lighting_and_surfaces_follow_the_catalogue(builder, tmp_path):
    scene = scene_of("positive")
    builder.generate_scene(CATALOGUE, scene["scene_id"], tmp_path)
    world = ET.parse(tmp_path / "scene_world.sdf").getroot()
    light = world.find(".//light[@name='sun']")
    d = scene["lighting"]["diffuse"]
    assert light.findtext("diffuse") == f"{d} {d} {d} 1"
    assert world.find(".//scene/background").text == scene["surfaces"]["background"]


def test_unknown_scene_id_is_rejected(builder, tmp_path):
    with pytest.raises(ValueError):
        builder.generate_scene(CATALOGUE, "s999", tmp_path)
```

- [ ] **Step 2:** 실행 → 실패 확인. **Step 3:** 구현(Task 1의 `sdf_parts` 사용, `_load_parts()` 방식 동일). 바깥 지지대 collision 이름에 `outer`를 포함한다(`spacer_outer_0`, `spacer_outer_1`), 중앙은 `spacer_center`. **Step 4 (GREEN):** 통과.

### Task 4: 문서

- [ ] `sim/gazebo/README.md`에 "합성 평가 장면 세트(카탈로그)" 절: 카탈로그 위치·생성 명령·고정 정책, 장면 월드 생성 명령, 회귀 해시가 reference experiment를 보호한다는 설명, 캡처·원격은 3단계에서 추가된다는 경계.
- [ ] 링크 검사 0개.

### Task 5: 검증·커밋 (Claude)

- [ ] 전체 회귀·Ruff. 카탈로그 파일 크기와 범주·split 표를 검증 기록에 남긴다.
- [ ] 커밋 `feat(sim): add scene catalogue generator and per-scene world builder`(`sdf_parts` 분리 포함), 검증 기록 커밋, `main` ff-merge, push, 원격 repo 갱신.

## 자체 검토

- 설계 §5.1 기하·§5.2 항목·거부 샘플링·정답 시각 의미(`stamp_ns 0`, synthetic) → Task 2. §6 월드 생성·카메라 설정·LiDAR 제외·회귀 보호 → Task 1·3. §10 시험(yaw 0/+/−, 음성, 시야 밖, 가림, 결정론) → Task 2·3.
- 자리표시자 없음: 해시·기대 좌표·투영값은 2026-09-11 계산값이다.
- 이름 일관성: `sdf_parts`, `generate_scene`, `pallet_ground_truth`, `opening_corners`, `project_to_image`, `openings_in_view`, `sample_catalogue`, `catalogue_v1.yaml`, 모델 이름 `synthetic_pallet`/`lookalike`/`occluder`/`distractor_<name>`/`synthetic_sensor_rig`.
- 미포함(의도): LiDAR·scan, 캡처 노드·runner·`scenes` mode(3단계), 로더의 카탈로그 접근(1단계 `SceneSample.scene`이 담당).
