# EPAL 6 팔레트 모델과 지게차 도킹 미리보기 구현 계획

> **실행자:** Task 1–4는 Codex에 위임한다. 각 Task는 실패 시험 → 실패 확인 → 구현 → 통과 확인 순서로 진행한다. **커밋·stage는 하지 않는다.** Task 5(영상 확인·검증 기록)는 Claude가 한다.

**목표:** 팔레트 형상을 설정 파일 한 곳으로 모으고 거기에 실물 EPAL 6 규격을 넣는다. 그 파일에서 시뮬레이션 팔레트 모델과 인식기 prior를 모두 생성해, 실물 입고 후에는 **치수 숫자만 고치면** 되게 만든다. 마지막으로 지게차와 팔레트를 한 장면에 놓고 포크가 개구부에 들어가는 과정을 영상으로 낸다.

**왜 지금 하는가:** 현재 팔레트 치수는 `sim/gazebo/build_scene_world.py`에 리터럴로 박혀 있고 `config/pallet_prior_v1.yaml`에 같은 값이 또 적혀 있다. 두 곳이 어긋나면 조용히 틀린 결과가 나온다. 또 현재 형상(높이 0.30 m, 개구 높이 0.20 m)은 실물 규격이 아니다.

**기술 스택:** Python ≥ 3.10, NumPy, PyYAML, MuJoCo 3.10, Pillow, ffmpeg. 시험은 `/home/light/anaconda3/bin/python -m pytest`로 돌린다.

## 전역 제약

- 기준: `main` `6a9453e`. 트리 깨끗.
- **기존 시험 685개의 행동을 보존한다.** `ruff check .`·`ruff format --check .` 통과. Ruff `B` 규칙이 켜져 있어 기본 인자에 함수 호출을 두지 않는다.
- **측정값과 규격값과 추정값을 구분해 적는다.** EPAL 6 치수는 공개 규격이고, 지게차 포크 치수는 상품 사진 비율 추정(`dimensions: image_proportion_estimates_not_measured`)이다. 이 구분을 설정 파일과 문서에 남긴다.
- 도킹 영상은 **관절 값을 직접 지정한 운동학 미리보기**다. 접촉·마찰·적재 안정성을 검증한 것이 아니며 그 표시를 영상과 보고서에 넣는다.
- 데이터 세트 재생성과 M2 재평가는 **이 계획에 포함하지 않는다.** 형상이 바뀌면 기존 100장면과 `pallet_prior_v1`은 그대로 v1 기록으로 남고, 재생성은 별도 단계다.

## 확정 수치 (EPAL 6 반 팔레트, 공개 규격)

외형 800(y, 접근 면 폭) × 600(x, 삽입 깊이) × 144(z) mm. 블록 9개, 각 145(y) × 100(x) × 78(z) mm.

| 항목 | 값 | 유도 |
|---|---|---|
| 개구 폭 | 182.5 mm | (800 − 3×145) / 2 |
| 개구 중심 \|y\| | 163.75 mm | 145/2 + 182.5/2 |
| 개구 중심 간격 | 327.5 mm | 2 × 163.75 |
| 개구 z 대역 | 0.022 – 0.100 m | 아래 덱 22, 블록 78 |
| 개구 중심 높이 | 61.0 mm | 22 + 78/2 |
| 위 덱 두께 | 44 mm | 144 − 22 − 78 |
| 블록 y 중심 | −0.3275, 0, +0.3275 | |
| 블록 x 중심 | −0.25, 0, +0.25 | |
| 교차 개구(600 면) | 150 mm | (600 − 3×100) / 2 |

포크(추정) 간격 0.29 m, 폭 0.055, 두께 0.024, 내림 중심 높이 0.04, 길이 0.42, 승강 행정 0.28. 한 포크는 y 0.1175–0.1725를 차지하고 개구는 y 0.0725–0.2550이므로 **양옆 45.0 mm·82.5 mm 여유로 들어간다.** 세로로는 포크가 z 0.028–0.052, 개구가 0.022–0.100이라 **승강 없이 들어간다.**

**단순화(문서에 명시):** 개별 덱 보드 사이 틈은 모델링하지 않는다. 아래 덱을 800×600×22 한 장, 위 덱(스트링거+상판)을 800×600×44 한 장으로 둔다. 외형·개구 위치·전면 평면은 보존되고 상판 표면의 틈만 사라진다.

## 파일 구조

| 파일 | 책임 |
|---|---|
| `config/pallet_geometry_epal6.yaml` | **유일한 정본.** 외형·블록·덱 치수와 출처 분류 |
| `src/forklift_core/perception/pallet_geometry.py` | 형상 파일 로더, 유도값(개구 폭·중심·대역) 계산, `PalletPrior` 생성 |
| `config/pallet_prior_epal6.yaml` | 형상 파일에서 **생성한** 인식기 prior |
| `tools/build_pallet_prior.py` | 형상 → prior YAML 생성 CLI |
| `tools/build_pallet_model.py` | 형상 → MJCF/URDF 팔레트 모델 생성 CLI |
| `sim/models/epal6_pallet/` | 생성된 `pallet.xml`·`pallet.urdf`·`model_manifest.json` |
| `tools/preview_docking.py` | 지게차+팔레트 장면, 접근·삽입 운동학 애니메이션, 간극 보고서, MP4 |
| `sim/models/docking_scene.xml` | 지게차와 팔레트를 함께 놓은 MuJoCo 장면 |
| `tests/unit/perception/test_pallet_geometry.py` | 형상 유도값·포크 적합 검사 |
| `tests/integration/test_pallet_model_build.py` | 생성 모델의 기하 일치 |

---

### Task 1: 형상 정본과 로더

**Files:** Create `config/pallet_geometry_epal6.yaml`, `src/forklift_core/perception/pallet_geometry.py`, `tests/unit/perception/test_pallet_geometry.py`

**Interfaces — Produces:**

```python
@dataclass(frozen=True)
class PalletGeometry:
    """Physical pallet shape. Loaded from YAML only; no dataclass defaults."""

    source_provenance: str  # "epal6_published_standard"
    geometry_version: str  # "epal6"
    overall_width_m: float  # 0.800, across the approach face (y)
    overall_depth_m: float  # 0.600, along the insertion axis (x)
    overall_height_m: float  # 0.144
    block_width_m: float  # 0.145 (y)
    block_depth_m: float  # 0.100 (x)
    block_height_m: float  # 0.078
    deck_bottom_m: float  # 0.022
    block_count_across: int  # 3
    block_count_deep: int  # 3

    @property
    def deck_top_m(self) -> float: ...  # height - deck_bottom - block_height
    @property
    def opening_width_m(self) -> float: ...  # (width - 3*block_width) / 2
    @property
    def opening_centre_offset_m(self) -> float: ...
    @property
    def opening_centre_spacing_m(self) -> float: ...
    @property
    def opening_centre_height_m(self) -> float: ...
    @property
    def opening_z_band_m(self) -> tuple[float, float]: ...
    def block_centres_y_m(self) -> list[float]: ...
    def block_centres_x_m(self) -> list[float]: ...


def load_pallet_geometry(path: Path) -> PalletGeometry: ...


@dataclass(frozen=True)
class ForkFit:
    """Whether a given fork set can enter this pallet, with the margins."""

    lateral_ok: bool
    vertical_ok: bool
    lateral_inner_margin_m: float
    lateral_outer_margin_m: float
    lift_required_m: float
    reach_fraction: float

    @property
    def fits(self) -> bool: ...


def check_fork_fit(
    geometry: PalletGeometry,
    *,
    fork_spacing_m: float,
    fork_width_m: float,
    fork_thickness_m: float,
    fork_centre_height_m: float,
    fork_length_m: float,
    lift_travel_m: float,
    floor_clearance_m: float = 0.004,
) -> ForkFit: ...
```

형상 파일은 미지 키를 `ValueError`로 거부하고, `overall_height_m == deck_bottom_m + block_height_m + deck_top_m` 관계와 `3*block_width_m < overall_width_m`를 검증한다. `deck_top_m`이 음수면 거부한다.

- [ ] **Step 1: 시험 작성** — `tests/unit/perception/test_pallet_geometry.py`

```python
import math
from pathlib import Path

import pytest

from forklift_core.perception.pallet_geometry import (
    check_fork_fit,
    load_pallet_geometry,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
EPAL6 = REPO_ROOT / "config" / "pallet_geometry_epal6.yaml"

# dls08_provisional, estimated from product images and not measured
FORKS = {
    "fork_spacing_m": 0.29,
    "fork_width_m": 0.055,
    "fork_thickness_m": 0.024,
    "fork_centre_height_m": 0.04,
    "fork_length_m": 0.42,
    "lift_travel_m": 0.28,
}


def test_the_published_epal6_envelope_is_what_the_file_says():
    g = load_pallet_geometry(EPAL6)
    assert (g.overall_width_m, g.overall_depth_m, g.overall_height_m) == (
        0.800,
        0.600,
        0.144,
    )
    assert (g.block_width_m, g.block_depth_m, g.block_height_m) == (0.145, 0.100, 0.078)
    assert g.source_provenance == "epal6_published_standard"


def test_the_opening_geometry_is_derived_not_restated():
    g = load_pallet_geometry(EPAL6)
    assert g.opening_width_m == pytest.approx(0.1825)
    assert g.opening_centre_offset_m == pytest.approx(0.16375)
    assert g.opening_centre_spacing_m == pytest.approx(0.3275)
    assert g.deck_top_m == pytest.approx(0.044)
    assert g.opening_z_band_m == pytest.approx((0.022, 0.100))
    assert g.opening_centre_height_m == pytest.approx(0.061)


def test_block_centres_are_symmetric_and_span_the_envelope():
    g = load_pallet_geometry(EPAL6)
    assert g.block_centres_y_m() == pytest.approx([-0.3275, 0.0, 0.3275])
    assert g.block_centres_x_m() == pytest.approx([-0.25, 0.0, 0.25])
    # the outer blocks end exactly at the envelope
    assert g.block_centres_y_m()[-1] + g.block_width_m / 2 == pytest.approx(
        g.overall_width_m / 2
    )


def test_the_estimated_dls08_forks_enter_this_pallet_without_lifting():
    fit = check_fork_fit(load_pallet_geometry(EPAL6), **FORKS)
    assert fit.fits
    assert fit.lateral_inner_margin_m == pytest.approx(0.045)
    assert fit.lateral_outer_margin_m == pytest.approx(0.0825)
    assert fit.lift_required_m == pytest.approx(0.0)
    assert fit.reach_fraction == pytest.approx(0.42 / 0.600)


def test_forks_wider_than_the_openings_are_reported_as_blocked():
    g = load_pallet_geometry(EPAL6)
    fit = check_fork_fit(g, **{**FORKS, "fork_spacing_m": 0.70})
    assert not fit.fits and not fit.lateral_ok


def test_forks_thicker_than_the_opening_cannot_be_lifted_into_it():
    g = load_pallet_geometry(EPAL6)
    fit = check_fork_fit(g, **{**FORKS, "fork_thickness_m": 0.20})
    assert not fit.fits and not fit.vertical_ok


def test_a_pallet_whose_parts_do_not_add_up_is_refused(tmp_path):
    text = EPAL6.read_text().replace(
        "overall_height_m: 0.144", "overall_height_m: 0.30"
    )
    bad = tmp_path / "bad.yaml"
    bad.write_text(text)
    with pytest.raises(ValueError):
        load_pallet_geometry(bad)


def test_unknown_keys_are_refused(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(EPAL6.read_text() + "\nnot_a_field: 1\n")
    with pytest.raises(ValueError):
        load_pallet_geometry(bad)
```

- [ ] **Step 2: 실패 확인** → `ModuleNotFoundError: forklift_core.perception.pallet_geometry`.
- [ ] **Step 3: 구현.** **Step 4: 통과 확인** + Ruff.

### Task 2: 형상에서 prior 생성

**Files:** Create `tools/build_pallet_prior.py`, `config/pallet_prior_epal6.yaml`; Modify `src/forklift_core/perception/pallet_prior.py`, `src/forklift_core/perception/pocket_detector.py`, `tests/unit/perception/test_pallet_prior.py`

현재 `PalletPrior`는 덱이 위아래 같다고 보고 `height_m == 2*deck_m + opening_height_m`을 검증한다. EPAL 6은 아래 22 mm·위 44 mm로 **비대칭**이라 이 계약이 성립하지 않는다.

**Interfaces — Produces:** `PalletPrior`에서 `deck_m`을 **`deck_bottom_m`과 `deck_top_m` 두 필드로 나눈다.** 검증식은 `height_m == deck_bottom_m + opening_height_m + deck_top_m`. `opening_centre_height_m`은 `deck_bottom_m + opening_height_m/2`로 유도한다(기존 동작과 동일한 의미). 인식기의 z 대역은 `_column_grid`에서 `deck_bottom_m + band_margin` ~ `height_m - deck_top_m - band_margin`으로 바꾼다.

기존 `config/pallet_prior_v1.yaml`은 **삭제하지 않는다.** v1 데이터 세트의 기록이므로 `deck_m: 0.05`를 `deck_bottom_m: 0.05`·`deck_top_m: 0.05`로 옮겨 적고 같은 동작을 유지한다. v1 회귀 시험이 그대로 통과해야 한다.

`tools/build_pallet_prior.py`는 형상 YAML을 읽어 prior YAML을 표준출력 또는 `--output`으로 낸다. 개구 폭·중앙 지지대 범위는 형상값에 **대칭 허용오차**를 붙인다: 개구 폭 `[w - 0.02, w + 0.02]`, 중앙 지지대 `[b - 0.015, b + 0.015]`. 허용오차 값은 CLI 인자로 바꿀 수 있게 하고 기본값을 prior 파일에 기록한다.

- [ ] **Step 1: 시험 작성** — `tests/unit/perception/test_pallet_prior.py`에 추가

```python
def test_the_committed_epal6_prior_matches_the_geometry_file(tmp_path):
    # the prior is generated, so drift between the two files must fail here
    import importlib.util

    path = REPO_ROOT / "tools" / "build_pallet_prior.py"
    spec = importlib.util.spec_from_file_location("build_pallet_prior", path)
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    out = tmp_path / "prior.yaml"
    assert (
        builder.main(
            [
                "--geometry",
                str(REPO_ROOT / "config" / "pallet_geometry_epal6.yaml"),
                "--output",
                str(out),
            ]
        )
        == 0
    )
    committed = REPO_ROOT / "config" / "pallet_prior_epal6.yaml"
    assert yaml.safe_load(out.read_text()) == yaml.safe_load(committed.read_text())


def test_the_epal6_prior_carries_the_asymmetric_decks():
    prior = load_pallet_prior(REPO_ROOT / "config" / "pallet_prior_epal6.yaml")
    assert prior.deck_bottom_m == pytest.approx(0.022)
    assert prior.deck_top_m == pytest.approx(0.044)
    assert prior.opening_height_m == pytest.approx(0.078)
    assert prior.opening_centre_height_m == pytest.approx(0.061)


def test_a_prior_whose_decks_and_opening_do_not_reach_the_height_is_refused(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(
        "source_provenance: synthetic\ncatalogue_version: test\n"
        "height_m: 0.144\ndeck_bottom_m: 0.022\ndeck_top_m: 0.044\n"
        "opening_height_m: 0.100\nopening_width_range: [0.16, 0.20]\n"
        "centre_spacer_range: [0.13, 0.16]\noverall_width_m: 0.8\n"
    )
    with pytest.raises(ValueError):
        load_pallet_prior(bad)
```

- [ ] **Step 2: 실패 확인.** **Step 3: 구현.** **Step 4: 통과 확인** + Ruff. **v1 prior를 쓰는 기존 시험이 전부 통과해야 한다.**

### Task 3: 팔레트 시뮬레이션 모델 생성

**Files:** Create `tools/build_pallet_model.py`, `sim/models/epal6_pallet/`(생성물), `tests/integration/test_pallet_model_build.py`

형상 YAML에서 MJCF(`pallet.xml`)와 URDF(`pallet.urdf`)를 생성한다. 구성은 아래 덱 1개, 블록 9개, 위 덱 1개로 총 11개 box다. 좌표 원점은 **바닥 위 footprint 중심**(z=0)으로 `build_scene_world.py`의 팔레트 원점 규약과 같게 둔다. `model_manifest.json`에 입력 형상 파일의 SHA-256, 생성 시각, 단순화 항목을 적는다.

- [ ] **Step 1: 시험 작성** — 생성한 MJCF를 MuJoCo로 읽어 기하를 되재는 시험.

```python
def test_the_generated_pallet_has_eleven_boxes_in_the_declared_places(tmp_path):
    out = tmp_path / "epal6"
    assert (
        build_pallet_model.main(["--geometry", str(EPAL6), "--output", str(out)]) == 0
    )
    model = mujoco.MjModel.from_xml_path(str(out / "pallet.xml"))
    sizes = {}
    for index in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        sizes[name] = (model.geom_size[index].copy(), model.geom_pos[index].copy())
    assert len(sizes) == 11
    # MuJoCo box size is the half extent
    half, pos = sizes["deck_bottom"]
    assert half == pytest.approx([0.300, 0.400, 0.011])
    assert pos == pytest.approx([0.0, 0.0, 0.011])
    half, pos = sizes["block_x1_y2"]
    assert half == pytest.approx([0.050, 0.0725, 0.039])
    assert pos == pytest.approx([0.0, 0.3275, 0.061])


def test_the_fork_openings_of_the_generated_model_are_actually_empty(tmp_path):
    # sample points along each fork's swept volume and assert none is inside a box
    ...


def test_the_manifest_records_the_geometry_hash(tmp_path): ...
```

- [ ] **Step 2: 실패 확인.** **Step 3: 구현.** **Step 4: 통과 확인** + Ruff.

### Task 4: 도킹 장면과 영상

**Files:** Create `sim/models/docking_scene.xml`, `tools/preview_docking.py`

지게차 `forklift.xml`과 생성된 `pallet.xml`을 한 장면에 놓는다. 팔레트는 지게차 앞 약 2.0 m에 두고 `mocap` 또는 고정 body로 배치한다. `tools/preview_docking.py`는 **관절과 base 자세를 프레임마다 직접 지정하는 운동학 애니메이션**을 만든다. 물리 적분과 접촉을 쓰지 않는다.

구간은 네 개다. ① 접근: 2.0 m에서 개구 앞 0.10 m까지 직진 ② 정렬: 남은 측면 오차와 yaw를 0으로 ③ 삽입: 포크가 깊이의 70 %까지 전진 ④ 들어올림: 0.04 m 상승.

프레임마다 **양쪽 포크와 팔레트 사이 최소 간극**을 계산해 `clearance.json`에 남기고, 한 프레임이라도 간극이 음수면 **CLI가 nonzero로 끝난다**. 영상 자막에 구간 이름과 그 프레임의 최소 간극을 넣고, 마지막 줄에 `Kinematic preview; contact and payload not simulated`를 고정으로 넣는다.

산출물은 `overview.png`, `docking.mp4`(24 fps), `clearance.json`, `run.json`(형상·모델 해시, MuJoCo 버전, 백엔드, 렌더러 이름)이다.

- [ ] **Step 1: 시험 작성** — `tests/unit/test_preview_docking.py`. 렌더링 없이 궤적 계산만 검사한다(렌더링 시험은 `rendering` 마커).

```python
def test_the_trajectory_keeps_the_forks_clear_of_the_pallet_at_every_frame():
    frames = preview_docking.plan_trajectory(GEOMETRY, FORKS, frames=96)
    assert len(frames) == 96
    assert min(f.clearance_m for f in frames) > 0.0


def test_the_insertion_phase_actually_reaches_into_the_pallet():
    frames = preview_docking.plan_trajectory(GEOMETRY, FORKS, frames=96)
    inserted = [f for f in frames if f.phase == "insert"]
    assert inserted[-1].penetration_m == pytest.approx(0.42, abs=0.01)


def test_a_fork_set_that_cannot_fit_is_refused_before_rendering():
    with pytest.raises(ValueError):
        preview_docking.plan_trajectory(
            GEOMETRY, {**FORKS, "fork_spacing_m": 0.70}, frames=24
        )
```

- [ ] **Step 2: 실패 확인.** **Step 3: 구현.** **Step 4: 통과 확인** + Ruff. 렌더링은 Claude가 실행한다.

### Task 5: 영상 확인과 기록 (Claude)

- [ ] `tools/preview_docking.py`를 EGL 백엔드로 실행해 MP4를 만든다.
- [ ] `clearance.json`의 최소 간극과 시험의 기대값을 대조한다.
- [ ] **영상을 직접 보고** 포크가 개구부에 들어가는지 확인한다. 보지 않은 영상을 완료로 표시하지 않는다.
- [ ] 검증 기록 작성, 사용자에게 영상 전달, 커밋.
