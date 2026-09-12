# M2 2단계: 평가 CLI·시연물·dev 튜닝·eval 실행 구현 계획

> **실행자:** Task 0(기준 확인)은 Claude가 하고, Task 1–4(도구 구현·문서)는 Codex에 위임한다. 각 Task는 실패 시험 → 실패 확인 → 구현 → 통과 확인 순서로 진행하고 체크박스로 추적한다. Codex 샌드박스는 `.git`·`/home/light/anaconda3`·네트워크·Docker에 쓰지 못하므로 커밋·stage는 하지 않는다. **Task 5–7(실데이터 실행·튜닝·기록)은 Claude가 수행한다.** 시험은 `/home/light/anaconda3/bin/python`으로 실행한다(conda base, editable 설치됨, NumPy 2.4.6 · Pillow 12.2 · PyYAML 6.0.3 · pytest 9.1, `/usr/bin/ffmpeg` n9.0.1).

> 상태: **v2 — 2026-09-13 Codex 검토 반영.**

**목표:** 1단계의 인식기·평가 계산을 데이터 세트에 적용하는 CLI를 만들고, dev 70장면으로 파라미터를 조정한 뒤 eval 30장면을 **한 번** 실행해 지표·장면별 표·시연 영상과 검증 기록을 남긴다. 로드맵 M2의 완료 조건을 닫는다.

**아키텍처:** `src/forklift_core/perception/overlay.py`가 정답·추정 사각형을 RGB에 그리는 순수 함수를 제공하고(Pillow는 이 모듈 안에서만 지연 import), `tools/evaluate_pocket_detector.py`가 데이터 세트를 순회하며 인식기 → 평가기 → 산출물 저장을 잇는 얇은 CLI다. MP4는 저장된 PNG를 ffmpeg로 묶는 별도 단계이며 실패해도 지표 산출을 막지 않는다.

**기술 스택:** Python ≥ 3.10, NumPy, Pillow ≥ 10, PyYAML ≥ 6, ffmpeg(선택), pytest, Ruff.

**Spec:** `docs/design/2026-09-13-pocket-detector-baseline.md`(사용자 승인 v2) §5 평가기·CLI, §10 구현 단계 2.

## 전역 제약

- 기준: `main` `6738e39`. 브랜치 `feat/pocket-evaluation`.
- **dev/eval 규칙(승인된 결정):** 파라미터·문턱 조정은 `--split dev`에서만 한다. `--split eval`은 파라미터를 확정한 **뒤 한 번** 실행하고 그 결과를 최종 보고로 쓴다. eval 결과를 보고 파라미터를 되돌리면 그 eval은 최종 보고가 아니며 검증 기록에 그 사실을 적는다.
- **`--split`은 `dev` 또는 `eval`만 받는다(`all` 없음).** `summarize`는 여러 split을 넘기면 **하나로 합산**하며 `splits`는 이름 목록일 뿐이다(2026-09-13 확인: dev TP 1 + eval FN 1 → 양성 검출률 0.5). 혼합 집계가 최종 지표로 오해될 여지를 만들지 않는다.
- **eval 직전 동결 게이트(Task 5.5):** 구현·시험·prior·params를 커밋해 고정 revision으로 만든 뒤 eval을 실행한다. `git_dirty` 불리언만으로는 당시 코드를 재현할 수 없다.
- CLI는 인식기에 `SceneInput`만 넘긴다. `SceneSample`은 평가기에서만 쓴다(정답 누출 금지).
- 결과 디렉터리는 `artifacts/<UTC시각>_pocket_eval_<split>_NN/`이며 **기존 디렉터리를 덮어쓰지 않는다**(`mkdir(parents=True, exist_ok=False)`).
- 목표 수치(위치 p95 20 mm, yaw p95 2°, 양성 검출률 95 %)는 `evaluation.py`의 상수를 그대로 쓴다. CLI에서 재정의하지 않는다.
- 지표·집계는 **모두 `evaluation.summarize`가 계산한다.** CLI가 따로 평균·비율을 계산하지 않는다.
- overlay는 정답과 추정을 **다른 색**으로 그리고 상태·오차를 글자로 넣는다. MP4에는 독립 장면 모음이며 연속 관측이 아니라는 표시를 넣는다.
- 기존 시험 650개 행동 보존, `ruff check .`·`ruff format --check .` 통과. Ruff `B` 규칙이 켜져 있으므로 기본 인자에 함수 호출을 두지 않는다.
- 데이터 세트가 없으면 CLI 시험은 skip 사유를 남기고, 합성 장면으로 만든 임시 데이터 세트 시험은 **항상** 실행한다.

## 파일 구조

| 파일 | 책임 |
|---|---|
| `src/forklift_core/perception/pocket_detector.py` | 내부 예외의 traceback을 진단에 실어 보낸다(Task 1) |
| `src/forklift_core/perception/overlay.py` | `project_point`, `opening_corners_m`, `draw_scene_overlay`, 색 상수 |
| `tools/evaluate_pocket_detector.py` | CLI: 데이터 세트 순회 → 인식기 → 평가기 → `metrics.json`·`scenes.csv`·`overlay/`·`run.json`, 선택적 MP4 |
| `tests/unit/perception/test_overlay.py` | 투영·코너·그리기 시험 |
| `tests/integration/test_evaluate_cli.py` | 임시 데이터 세트로 CLI 끝까지 실행, split 격리·CSV 스키마·traceback |
| `tests/fixtures/scene_writer.py` | 1단계 `write_v1_scene`을 옮겨 category·split·GT를 인자로 받게 한 장면 작성기 |
| `docs/validation/2026-09-1x-pocket-detector-m2.md` | dev 튜닝과 eval 1회 결과(Task 7) |

---

### Task 0: 기준 상태

- [ ] `git rev-parse HEAD`가 이 계획 커밋이고 그 부모가 `6738e39`이며 `git status --short`가 비어 있음을 확인한다.
- [ ] 기준 회귀: `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error` → `650 passed, 1 deselected`.

### Task 1: 인식기 내부 예외의 traceback 보존

**Files:** Modify `src/forklift_core/perception/pocket_detector.py`, `tests/unit/perception/test_pocket_detector.py`

인식기는 내부 예외를 잡아 `invalid`(`exception:<Type>`)로 바꾸고 **traceback을 버린다**(2026-09-13 확인). CLI 바깥의 `try/except`로는 복구할 수 없으므로 진단에 실어 보낸다.

**Interfaces — Produces:** `DetectionDiagnostics`에 `exception_traceback: str | None = None` 필드를 **마지막에** 추가한다(기존 위치 인자 호출을 깨지 않는다). 정상 경로에서는 `None`, `except` 분기에서는 `traceback.format_exc()`.

- [ ] **Step 1: 시험 작성** — `tests/unit/perception/test_pocket_detector.py`에 추가.

```python
def test_an_internal_failure_becomes_invalid_and_keeps_its_traceback(
    pallet_scene, monkeypatch
):
    scene, _ = pallet_scene()

    def boom(*args, **kwargs):
        raise RuntimeError("synthetic internal failure")

    monkeypatch.setattr(pocket_detector, "_vertical_plane_candidates", boom)
    result = detect(scene)
    assert result.observation.status == "invalid"
    assert result.observation.reason == "exception:RuntimeError"
    assert "synthetic internal failure" in result.diagnostics.exception_traceback
    assert "Traceback" in result.diagnostics.exception_traceback


def test_a_successful_detection_carries_no_traceback(pallet_scene):
    scene, _ = pallet_scene()
    result = detect(scene)
    assert result.observation.status == "valid"
    assert result.diagnostics.exception_traceback is None
```

- [ ] **Step 2: 실패 확인** → `AttributeError: 'DetectionDiagnostics' object has no attribute 'exception_traceback'`.
- [ ] **Step 3: 구현.** **Step 4: 통과 확인** + Ruff.

### Task 2: overlay 그리기

**Files:** Create `src/forklift_core/perception/overlay.py`, `tests/unit/perception/test_overlay.py`

**Interfaces — Produces:**

```python
TRUTH_COLOUR = (0, 255, 0)  # ground truth openings
ESTIMATE_COLOUR = (255, 0, 255)  # detector estimate
CAPTION_COLOUR = (255, 255, 255)


def project_point(
    point_base, intrinsics: PinholeIntrinsics, base_from_optical: RigidTransform
) -> tuple[float, float] | None:
    """Pixel (u, v) of a base-frame point.

    None when the point is at or behind the optical plane; coordinates outside
    the image are returned unchanged so partial rectangles clip when drawn.
    """


def opening_corners_m(
    pocket: Pocket, insertion_yaw_rad: float
) -> list[tuple[float, float, float]]:
    """Four base-frame corners of the opening rectangle, ordered
    top-left, top-right, bottom-right, bottom-left as seen along the insertion axis."""


def draw_scene_overlay(
    rgb: NDArray[np.uint8],
    *,
    truth: PocketObservation,
    estimate: PocketObservation,
    intrinsics: PinholeIntrinsics,
    base_from_optical: RigidTransform,
    caption: str,
) -> NDArray[np.uint8]:
    """RGB copy with the ground-truth openings in green, the estimate in magenta and
    the caption drawn at the top left. Openings that do not project are skipped."""
```

`opening_corners_m`: 왼쪽축 `l = (−sin ψ, cos ψ, 0)`, 위축 `u = (0, 0, 1)`에 대해 네 코너를 **이 순서 그대로** 돌려준다 — `c + w/2·l + h/2·u`(top-left), `c − w/2·l + h/2·u`(top-right), `c − w/2·l − h/2·u`(bottom-right), `c + w/2·l − h/2·u`(bottom-left). 폭·높이만 맞고 순서가 틀리면 사각형이 나비 모양으로 그려지므로 시험이 코너를 하나씩 고정한다.

`project_point`는 **광학 z ≤ 0일 때만** `None`이다(카메라 평면 위 또는 뒤). 화면 밖 좌표는 그대로 돌려준다 — 부분적으로 보이는 사각형은 그리기 단계에서 잘려야 하기 때문이다.

`draw_scene_overlay`는 정답을 `TRUTH_COLOUR`, 추정을 `ESTIMATE_COLOUR`, 글자를 `CAPTION_COLOUR`로 그린다. 선 굵기 2 px, `ImageDraw`의 `line`은 안티에일리어싱을 하지 않으므로 시험이 색을 정확히 비교할 수 있다. Pillow는 `draw_scene_overlay` 안에서만 import하고, 없으면 `ImportError`에 `pip install 'forklift-core[dataset]'`를 안내한다.

- [ ] **Step 1: 시험 작성** — `tests/unit/perception/test_overlay.py`

```python
import math

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.overlay import (
    ESTIMATE_COLOUR,
    TRUTH_COLOUR,
    draw_scene_overlay,
    opening_corners_m,
    project_point,
)
from forklift_core.perception.pocket_observation import Pocket, PocketObservation
from forklift_core.sensors.rgbd import PinholeIntrinsics

INTRINSICS = PinholeIntrinsics(
    width=640,
    height=480,
    fx=465.741156,
    fy=465.741156,
    cx=320.0,
    cy=240.0,
    frame_id="camera_optical_frame",
)
BASE_FROM_OPTICAL = RigidTransform(
    "camera_optical_frame",
    "base_link",
    rotation_matrix_from_quaternion_xyzw([-0.5, 0.5, -0.5, 0.5]),
    [0.75, 0.0, 0.5],
)


def test_a_point_on_the_optical_axis_lands_on_the_principal_point():
    # base (2.75, 0, 0.5) is 2.0 m straight ahead of the camera at its own height
    assert project_point(
        (2.75, 0.0, 0.5), INTRINSICS, BASE_FROM_OPTICAL
    ) == pytest.approx((320.0, 240.0), abs=1e-6)


def test_a_point_left_and_below_moves_left_and_down_in_the_image():
    u, v = project_point((2.75, 0.2, 0.15), INTRINSICS, BASE_FROM_OPTICAL)
    assert u < 320.0 and v > 240.0
    # optical x = -0.2, y = 0.35, z = 2.0  ->  u = -0.2*fx/2 + 320, v = 0.35*fx/2 + 240
    assert (u, v) == pytest.approx(
        (320.0 - 0.1 * 465.741156, 240.0 + 0.175 * 465.741156), abs=1e-6
    )


def test_points_behind_the_camera_do_not_project():
    assert project_point((0.0, 0.0, 0.5), INTRINSICS, BASE_FROM_OPTICAL) is None


def test_a_point_on_the_optical_plane_does_not_project():
    # base x = 0.75 is exactly the camera plane: optical z = 0, not merely small
    assert project_point((0.75, 0.0, 0.9), INTRINSICS, BASE_FROM_OPTICAL) is None


def test_a_point_off_the_sensor_still_projects_so_partial_rectangles_clip():
    # 3 m to the left at 2 m depth: in front of the camera but outside the image
    uv = project_point((2.75, 3.0, 0.5), INTRINSICS, BASE_FROM_OPTICAL)
    assert uv is not None
    assert uv[0] == pytest.approx(320.0 - 1.5 * 465.741156, abs=1e-6)


@pytest.mark.parametrize("yaw", [0.0, 0.4, -0.4, math.pi / 2, -math.pi / 2, math.pi])
def test_opening_corners_are_top_left_top_right_bottom_right_bottom_left(yaw):
    pocket = Pocket((2.2, 0.17, 0.15), 0.24, 0.20)
    corners = opening_corners_m(pocket, yaw)
    assert len(corners) == 4
    centre = np.array(pocket.center_m)
    left_axis = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    along = [float(np.dot(np.array(c) - centre, left_axis)) for c in corners]
    up = [float(c[2] - centre[2]) for c in corners]
    # each corner is pinned: a rotated or mirrored order such as [TR, TL, BL, BR]
    # keeps the same width and height but draws the rectangle as a bow tie
    assert along == pytest.approx([0.12, -0.12, -0.12, 0.12], abs=1e-9)
    assert up == pytest.approx([0.10, 0.10, -0.10, -0.10], abs=1e-9)
    # the opening rectangle carries no depth along the insertion axis
    axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    depth = [float(np.dot(np.array(c) - centre, axis)) for c in corners]
    assert depth == pytest.approx([0.0, 0.0, 0.0, 0.0], abs=1e-9)


def test_the_overlay_marks_truth_and_estimate_in_different_colours():
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    truth = PocketObservation(
        stamp_ns=1,
        clock_domain="ros_sim",
        frame_id="base_link",
        source_provenance="synthetic_ground_truth",
        status="valid",
        left=Pocket((2.2, 0.17, 0.15), 0.24, 0.20),
        right=Pocket((2.2, -0.17, 0.15), 0.24, 0.20),
        insertion_yaw_rad=0.0,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason=None,
    )
    estimate = PocketObservation(
        stamp_ns=1,
        clock_domain="ros_sim",
        frame_id="base_link",
        source_provenance="synthetic",
        status="valid",
        # 0.5 m short: the two outlines must not overlap for the colour check
        left=Pocket((1.7, 0.17, 0.15), 0.24, 0.20),
        right=Pocket((1.7, -0.17, 0.15), 0.24, 0.20),
        insertion_yaw_rad=0.0,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason=None,
    )
    out = draw_scene_overlay(
        rgb,
        truth=truth,
        estimate=estimate,
        intrinsics=INTRINSICS,
        base_from_optical=BASE_FROM_OPTICAL,
        caption="s001 wrong_pose",
    )
    assert out.shape == rgb.shape and out.dtype == np.uint8
    assert rgb.max() == 0  # the input is not modified

    colours = {tuple(c) for c in out.reshape(-1, 3).tolist()}
    assert TRUTH_COLOUR != ESTIMATE_COLOUR
    assert TRUTH_COLOUR in colours and ESTIMATE_COLOUR in colours

    def drawn_bbox(colour):
        rows, cols = np.nonzero(np.all(out == np.array(colour, dtype=np.uint8), axis=2))
        return cols.min(), cols.max(), rows.min(), rows.max()

    def projected_bbox(observation):
        points = opening_corners_m(observation.left, 0.0) + opening_corners_m(
            observation.right, 0.0
        )
        uvs = [project_point(c, INTRINSICS, BASE_FROM_OPTICAL) for c in points]
        assert all(uv is not None for uv in uvs)
        us = [uv[0] for uv in uvs]
        vs = [uv[1] for uv in uvs]
        return min(us), max(us), min(vs), max(vs)

    # the green pixels trace the ground-truth openings and the magenta pixels the
    # estimate, so a single colour used for both (or for the caption) fails here
    for colour, observation in ((TRUTH_COLOUR, truth), (ESTIMATE_COLOUR, estimate)):
        assert drawn_bbox(colour) == pytest.approx(projected_bbox(observation), abs=2)


def test_the_overlay_still_renders_when_the_estimate_has_no_pockets():
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    truth = PocketObservation(
        stamp_ns=1,
        clock_domain="ros_sim",
        frame_id="base_link",
        source_provenance="synthetic_ground_truth",
        status="no_pallet",
        left=None,
        right=None,
        insertion_yaw_rad=None,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason="no target pallet in scene",
    )
    out = draw_scene_overlay(
        rgb,
        truth=truth,
        estimate=truth,
        intrinsics=INTRINSICS,
        base_from_optical=BASE_FROM_OPTICAL,
        caption="s012 true_negative",
    )
    assert out.shape == rgb.shape
```

- [ ] **Step 2: 실패 확인** — `python -m pytest tests/unit/perception/test_overlay.py -q -p no:cacheprovider -W error` → `ModuleNotFoundError: forklift_core.perception.overlay`.
- [ ] **Step 3: 구현.** **Step 4: 통과 확인** + Ruff.

### Task 3: 평가 CLI

**Files:** Create `tools/evaluate_pocket_detector.py`, `tests/integration/test_evaluate_cli.py`, `tests/fixtures/scene_writer.py`; Modify `tests/integration/test_detector_pipeline.py`(옮긴 helper를 쓰도록)

**CLI:**

```text
python tools/evaluate_pocket_detector.py
    --dataset data/synthetic_scenes/catalogue_v1
    --split dev|eval
    --prior config/pallet_prior_v1.yaml
    --output artifacts/<UTC>_pocket_eval_<split>_NN
    [--params <yaml>] [--scenes s003,s006] [--video] [--no-overlay]
```

`--scenes`는 **선택한 split 안에 있는 ID만** 허용한다. 빈 선택·없는 ID·다른 split의 ID는 모두 `ValueError`다 — 조용히 건너뛰면 장면 수가 달라진 것을 아무도 눈치채지 못한다.

산출물:

| 파일 | 내용 |
|---|---|
| `metrics.json` | `summarize(results)` 결과 그대로 + `"run"` 블록 |
| `scenes.csv` | 장면당 한 행, 18열: `scene_id,category,split,truth_status,estimate_status,outcome,left_error_m,right_error_m,position_error_m,yaw_error_rad,reason,elapsed_s,plane_residual_p95_m,plane_inlier_count,left_front_frac,left_behind_frac,right_front_frac,right_behind_frac`. `outcome`은 `.value`. 광선 비율은 `diagnostics.opening_rays.get(side)`로 읽고 **없으면 빈칸**(JSON은 `null`)이다 — `opening_rays`는 점 부족·후보 없음·예외에서 `{}`가 될 수 있으므로 없는 진단을 0 %로 만들지 않는다. `front_fraction`·`behind_fraction`은 property라 `asdict`에 없으니 명시적으로 계산해 넣는다 |
| `observations/sNNN.json` | 장면별 `PocketObservation.to_json()` + `diagnostics` |
| `overlay/sNNN.png` | 정답·추정 겹친 RGB(`--no-overlay`면 생략) |
| `overlay.mp4` | `--video`일 때만, PNG를 5 fps로 묶음 |
| `run.json` | 재현 정보(아래) |

`run.json`과 `metrics.json["run"]`(같은 내용): `dataset_dir`, `dataset_manifest_sha256`, `split`, `scene_ids`(실제 평가한 ID 목록), `scene_count`, `prior_path`, `prior_sha256`, `params`(기본값까지 적용한 `DetectorParams`의 asdict, seed 포함), `params_path`(있으면), `position_tolerance_m`, `yaw_tolerance_rad`, `git_revision`, `git_dirty`, `started_at_utc`, `ended_at_utc`, `python_version`, `numpy_version`. `--params`는 `DetectorParams` 필드와 같은 키만 허용하고 미지 키는 `ValueError`.

동작 규칙: 장면은 `scene_id` 오름차순. 인식기는 내부 예외를 이미 `invalid`(`exception:<Type>`)로 바꾸므로 CLI의 `try/except`로는 잡히지 않는다(2026-09-13 확인). 따라서 **traceback은 인식기가 진단에 실어 보내고**(Task 1), CLI는 그것을 `observations/sNNN.json`에 옮겨 적는다. CLI 자신이 던지는 예외(설정·입출력)는 별도로 잡아 같은 형식으로 남긴다.

**종료 코드:** 설정 오류(미지 파라미터 키·prior 불량), 입력 오류(데이터 세트·split·장면 ID), 출력 디렉터리 중복, 필수 산출물 저장 실패는 **nonzero**. 목표 미달과 MP4 실패(ffmpeg 없음·오류, `run.json["video_error"]`에 기록)는 **정상 종료**다(측정 결과이지 도구 실패가 아니다).

- [ ] **Step 1: 시험 작성** — `tests/integration/test_evaluate_cli.py`

```python
import csv
import importlib.util
import json
from pathlib import Path

import pytest

from forklift_core.perception import pocket_detector

REPO_ROOT = Path(__file__).resolve().parents[2]
REPO_PRIOR = REPO_ROOT / "config" / "pallet_prior_v1.yaml"
REAL_DATASET = REPO_ROOT / "data" / "synthetic_scenes" / "catalogue_v1"


def _load_cli():
    """tools/ is not an installed package, so load the CLI by path."""
    path = REPO_ROOT / "tools" / "evaluate_pocket_detector.py"
    spec = importlib.util.spec_from_file_location("evaluate_pocket_detector", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


cli = _load_cli()


def run(dataset, out, *extra):
    return cli.main(
        [
            "--dataset",
            str(dataset),
            "--prior",
            str(REPO_PRIOR),
            "--output",
            str(out),
            *extra,
        ]
    )


def test_the_cli_writes_metrics_scenes_and_overlays_for_a_small_dataset(
    tmp_path, pallet_scene
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)  # helper in this file
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev") == 0
    metrics = json.loads((out / "metrics.json").read_text())
    assert metrics["scene_count"] == 3
    assert metrics["run"]["split"] == "dev" and metrics["run"]["prior_sha256"]
    assert metrics["run"]["scene_ids"] == ["s001", "s002", "s003"]
    assert metrics["counts"]["positive"]["true_positive"] == 2
    assert metrics["counts"]["negative_no_pallet"]["true_negative"] == 1
    rows = list(csv.DictReader((out / "scenes.csv").read_text().splitlines()))
    assert [r["scene_id"] for r in rows] == ["s001", "s002", "s003"]
    assert {r["outcome"] for r in rows} == {"true_positive", "true_negative"}
    assert float(rows[0]["position_error_m"]) < 0.03
    assert (out / "overlay" / "s001.png").exists()
    assert (
        json.loads((out / "observations" / "s001.json").read_text())["status"]
        == "valid"
    )


def test_the_dev_run_never_hands_an_eval_scene_to_the_recognizer(
    tmp_path, pallet_scene, monkeypatch
):
    # the tiny data set holds s001-s003 in dev and s004 in eval
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    real = cli.detect_pockets
    calls = {"n": 0}

    def counting(*args, **kwargs):
        calls["n"] += 1
        return real(*args, **kwargs)

    monkeypatch.setattr(cli, "detect_pockets", counting)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--no-overlay") == 0
    assert calls["n"] == 3
    rows = list(csv.DictReader((out / "scenes.csv").read_text().splitlines()))
    assert {r["split"] for r in rows} == {"dev"}
    assert not (out / "observations" / "s004.json").exists()


def test_the_eval_split_evaluates_only_its_own_scene(tmp_path, pallet_scene):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "eval", "--no-overlay") == 0
    metrics = json.loads((out / "metrics.json").read_text())
    assert metrics["scene_count"] == 1
    assert metrics["run"]["scene_ids"] == ["s004"]


@pytest.mark.parametrize(
    "selection",
    ["", "s999", "s004"],
    ids=["empty", "unknown id", "id from the eval split"],
)
def test_bad_scene_selections_are_rejected(tmp_path, pallet_scene, selection):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    with pytest.raises(ValueError):
        run(dataset, tmp_path / "run", "--split", "dev", "--scenes", selection)


def test_the_scene_table_has_the_declared_columns(tmp_path, pallet_scene):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--no-overlay") == 0
    lines = (out / "scenes.csv").read_text().splitlines()
    assert lines[0].split(",") == [
        "scene_id",
        "category",
        "split",
        "truth_status",
        "estimate_status",
        "outcome",
        "left_error_m",
        "right_error_m",
        "position_error_m",
        "yaw_error_rad",
        "reason",
        "elapsed_s",
        "plane_residual_p95_m",
        "plane_inlier_count",
        "left_front_frac",
        "left_behind_frac",
        "right_front_frac",
        "right_behind_frac",
    ]
    rows = {r["scene_id"]: r for r in csv.DictReader(lines)}
    # the no-pallet scene produces no opening rays: blank, never 0.0
    assert rows["s003"]["left_front_frac"] == ""
    assert rows["s003"]["right_behind_frac"] == ""
    assert 0.0 <= float(rows["s001"]["left_front_frac"]) <= 1.0


def test_the_cli_refuses_to_overwrite_an_existing_output_directory(
    tmp_path, pallet_scene
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    out = tmp_path / "run"
    out.mkdir()
    with pytest.raises(FileExistsError):
        run(dataset, out, "--split", "dev")


def test_unknown_parameter_keys_are_rejected(tmp_path, pallet_scene):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    params = tmp_path / "p.yaml"
    params.write_text("cell_m: 0.01\nnot_a_parameter: 1\n")
    with pytest.raises(ValueError):
        run(
            dataset,
            tmp_path / "run",
            "--split",
            "dev",
            "--params",
            str(params),
        )


def test_a_detector_exception_becomes_an_invalid_scene_and_does_not_stop_the_run(
    tmp_path, pallet_scene, monkeypatch
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    calls = {"n": 0}

    def boom(*args, **kwargs):
        calls["n"] += 1
        raise RuntimeError("synthetic failure")

    monkeypatch.setattr(cli, "detect_pockets", boom)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--no-overlay") == 0
    assert calls["n"] == 3
    metrics = json.loads((out / "metrics.json").read_text())
    assert sum(c.get("invalid", 0) for c in metrics["counts"].values()) == 3
    saved = json.loads((out / "observations" / "s001.json").read_text())
    assert saved["reason"] == "exception:RuntimeError"
    assert "synthetic failure" in saved["diagnostics"]["exception_traceback"]


def test_a_failure_inside_the_detector_is_saved_with_its_own_traceback(
    tmp_path, pallet_scene, monkeypatch
):
    # patched inside the recognizer, so the CLI never sees the exception: this
    # exercises Task 1 end to end instead of the CLI's own guard
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)

    def boom(*args, **kwargs):
        raise RuntimeError("synthetic internal failure")

    monkeypatch.setattr(pocket_detector, "_vertical_plane_candidates", boom)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--no-overlay") == 0
    saved = json.loads((out / "observations" / "s001.json").read_text())
    assert saved["reason"] == "exception:RuntimeError"
    trace = saved["diagnostics"]["exception_traceback"]
    assert "synthetic internal failure" in trace
    assert "_vertical_plane_candidates" in trace


def test_missing_optional_video_tool_does_not_fail_the_run(
    tmp_path, pallet_scene, monkeypatch
):
    dataset = build_tiny_dataset(tmp_path / "ds", pallet_scene)
    monkeypatch.setattr(cli.shutil, "which", lambda name: None)
    out = tmp_path / "run"
    assert run(dataset, out, "--split", "dev", "--video") == 0
    assert json.loads((out / "run.json").read_text())["video_error"]
    assert not (out / "overlay.mp4").exists()


@pytest.mark.skipif(
    not REAL_DATASET.is_dir(), reason="synthetic scene data set v1 is not present"
)
def test_three_real_dev_scenes_run_end_to_end(tmp_path):
    out = tmp_path / "run"
    assert run(REAL_DATASET, out, "--split", "dev", "--scenes", "s003,s006,s010") == 0
    metrics = json.loads((out / "metrics.json").read_text())
    assert metrics["scene_count"] == 3
    assert metrics["run"]["dataset_manifest_sha256"]
    assert (out / "overlay" / "s003.png").exists()
```

**CLI 모듈 구조(시험의 monkeypatch 전제):** 모듈 전역에 `import shutil`과 `from forklift_core.perception.pocket_detector import detect_pockets`를 두고 실행 시 그 전역 이름으로 `detect_pockets(...)`·`shutil.which("ffmpeg")`를 호출한다. 함수 안 import나 `pocket_detector.detect_pockets(...)` 형태는 쓰지 않는다. 시험은 `tests/integration/test_merge_scene_batches.py`처럼 `importlib.util.spec_from_file_location`으로 CLI를 한 번 로드해 그 모듈 객체를 patch·호출한다(`tools/`는 설치 패키지가 아니다).

**장면별 예외 처리:** CLI는 장면 한 개의 인식·평가를 `try/except`로 감싸고, 잡히면 `status: invalid`·`reason: exception:<Type>` 관측을 만들어 다음 장면을 계속 처리한다. 인식기가 스스로 잡은 경우에는 `diagnostics`가 `DetectionDiagnostics`의 asdict이고 `exception_traceback`에 인식기의 traceback이 들어 있다. CLI가 직접 잡은 경우에는 `DetectionDiagnostics`를 만들 수 없으므로 `diagnostics`를 `{"exception_traceback": traceback.format_exc()}` 한 키짜리 사전으로 쓴다. 두 경우 모두 `observations/sNNN.json`의 `diagnostics.exception_traceback`을 읽으면 되고, 소비자는 나머지 키가 없을 수 있다고 본다.

`build_tiny_dataset(root, pallet_scene)`는 **세트 구조**(`root/manifest.json` + `root/scenes/sNNN/`)로 v1 장면 **4개**를 쓴다: `s001`·`s002` 양성(`category: positive`, GT `valid`), `s003` 무팔레트(`category: negative_no_pallet`, **GT는 `status: no_pallet`·기하와 yaw·σ 모두 `None`·비어 있지 않은 `reason`**) — 여기까지 `split: dev` — 그리고 `s004` 양성이지만 **`split: eval`**. 네 번째 장면은 split 격리 시험의 대조군이며, 이것이 없으면 dev 실행이 실제로 걸러내는지 증명할 수 없다. 1단계의 `write_v1_scene`은 GT를 항상 `valid`·category `positive`·split `dev`로 고정하므로 **그대로 재사용하면 잘못된 음성 fixture가 된다**. Task 3 Step 3에서 `write_v1_scene`을 `tests/fixtures/scene_writer.py`로 옮겨 category·split·GT를 인자로 받게 하고 1단계 통합시험도 그 helper를 쓰도록 바꾼다(`load_scene_sample`은 장면 디렉터리만 받고 manifest를 읽지 않는다. manifest는 CLI의 해시 기록용이다).

- [ ] **Step 2: 실패 확인.** **Step 3: 구현.** **Step 4: 통과 확인** + Ruff. `python tools/evaluate_pocket_detector.py --help` 실행.

### Task 4: 문서

- [ ] `docs/development.md`에 "포켓 인식 평가 실행" 절: CLI 명령 예시(dev·eval), 산출물 목록, **dev에서만 튜닝하고 eval은 한 번**이라는 규칙, 데이터 세트가 Git 밖이라는 점.
- [ ] `README.md` "현재 구현" 목록에 `perception/pocket_detector.py`·`evaluation.py`·`overlay.py` 한 줄 추가.
- [ ] 링크 검사 0개, `diff <(tail -n +4 AGENTS.md) <(tail -n +4 CLAUDE.md)` 빈 출력(이 단계는 두 파일을 바꾸지 않는다).

### Task 5: dev 실행과 튜닝 (Claude)

- [ ] 기본 파라미터로 `--split dev` 1회 실행. `metrics.json`·`scenes.csv`를 읽고 실패 장면(특히 **s009**: 경쟁 평면으로 `no_opening_pattern`)의 `overlay/`·`observations/`를 직접 본다.
- [ ] 실패 원인별로 파라미터 후보를 정하고 **dev에서만** 재실행한다. 변경한 값과 그 근거, 각 실행의 지표를 표로 기록한다. 시험 허용오차나 `evaluation.py`의 목표 상수는 바꾸지 않는다.
- [ ] 파라미터를 고치면 `--params` YAML로 저장하고 `config/detector_params_v1.yaml`로 커밋한다(바꾸지 않았다면 기본값을 쓴다고 기록).
- [ ] dev 실행은 여러 번 할 수 있다. 모든 실행의 run ID와 지표를 검증 기록에 남긴다.

### Task 5.5: 동결 게이트 (Claude)

- [ ] 최종 파라미터 YAML을 CLI로 **다시 읽어 dev 1회** 실행하고 그 실행을 선택 근거로 기록한다.
- [ ] 구현·시험·prior·params를 커밋해 고정 revision을 만든다. 전체 회귀와 Ruff 통과를 확인한다(`git_dirty`가 `false`인 상태에서 eval을 돌린다).
- [ ] 최종 eval 명령에 같은 `--params`를 명시한다. 기본값을 택했더라도 유효 파라미터 전체를 `run.json`에 동결 기록한다.
- [ ] eval split이 **30장면 전체·양성 18개**이고 부분 선택이 없음을 확인한다.

### Task 6: eval 실행 (Claude, 1회)

- [ ] 파라미터 확정 후 `--split eval --video` **한 번** 실행. eval 양성은 18장면이므로 검출률 95 %는 18/18을 뜻한다.
- [ ] `metrics.json`의 목표 도달 여부를 **있는 그대로** 기록한다. 미달이어도 파라미터를 되돌리지 않는다(되돌리면 그 eval은 최종 보고가 아니며 그 사실을 적는다).
- [ ] overlay PNG 몇 장과 MP4를 직접 확인한다(양성·가림·음성 각 1장 이상). MP4가 실패했으면 저장된 PNG로 다시 묶는다(인식기를 재실행하지 않는다). **영상을 눈으로 확인하기 전에는 시연물 완료로 표시하지 않는다.**

### Task 7: 검증 기록과 커밋 (Claude)

- [ ] `docs/validation/2026-09-1x-pocket-detector-m2.md`: 구현 범위, dev 실행 표(run ID·파라미터·지표), eval 1회 결과(범주별 개수·검출률·오차 p50/p95/max·처리 시간·목표 도달 여부), 실패 장면 분석, eval을 돌린 고정 revision, 경계(합성 세트·prior가 카탈로그 형상을 안다는 점·σ 미상·실물 미검증). **문턱은 전체 100장면의 사전 특성 조사(설계 §2)로 정했으므로 eval을 '전혀 사용하지 않은 holdout'이라고 쓰지 않는다.**
- [ ] 체크포인트(`docs/validation/2026-09-11-development-checkpoint.md`)에 M2 행 추가, 로드맵 §8의 M2 항목 체크.
- [ ] 커밋 → `main` ff-merge → push → 원격 repo 갱신. 산출물 디렉터리는 Git 밖(`artifacts/`)이며 run ID와 해시만 기록한다.

## 자체 검토

- **Spec 대조:** §5의 CLI 산출물(`metrics.json`·`scenes.csv`·overlay·MP4·재현 정보) → Task 3; overlay 투영·코너·색 시험 → Task 2; 내부 예외의 traceback 보존 → Task 1; dev/eval 규칙 → 전역 제약·Task 4·5·5.5; 목표 도달 여부를 있는 그대로 보고 → Task 6·7. 지표 계산은 1단계 `summarize`가 전담하므로 이 계획에서 다시 정의하지 않는다.
- **자리표시자 없음.** 시험은 실제 기대값을 갖는다(투영 좌표는 손으로 계산). `build_tiny_dataset`은 Task 3 Step 3에서 함께 구현한다.
- **이름 일관성:** `project_point`·`opening_corners_m`·`draw_scene_overlay`·`TRUTH_COLOUR`·`ESTIMATE_COLOUR`·`evaluate_pocket_detector.main`(시험에서는 `cli.main`)·`build_tiny_dataset`이 Interfaces·시험·파일 표에서 동일하다. 1단계 이름(`detect_pockets`·`DetectionResult`·`evaluate_scene`·`summarize`·`load_pallet_prior`·`DetectorParams`)을 그대로 쓴다.
- **미포함(의도):** 자동 파라미터 탐색, 학습 검출기, 실물 데이터, 추적(M3), σ 추정.
