# 팔레트 인식 일반화 단계 1A — 장면·정답·평가기 (CPU) 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **이 계획의 실행 방식(사용자 결정 2026-10-15):** 이 세션에서 직접 구현(executing-plans), **각 작업 묶음(아래 "검토 지점")마다 Codex 검토를 받고 지적이 수렴한 뒤 다음 묶음으로 간다.**

**Goal:** 설계 §5·§3.3–3.4 의 장면 생성·정답·데이터 계약 판 2·통계·다중 인스턴스 평가기를 CPU 에서 끝까지 돌게 만들고, 지금 검출기의 기준선을 CPU 렌더 장면에서 기록한다.

**Architecture:** 코어(`src/forklift_core/perception/`)에 순수 numpy 모듈 — 일반화된 팔레트 형상(`pallet_model`), 손상 연산(`pallet_damage`), 포크 끼움 정답(`fork_fit`), 깊이 잡음(`depth_noise`), 통계(`recognition_stats`), 데이터 계약 판 2(`recognition_dataset`), 다중 인스턴스 평가(`evaluation_multi`) — 를 두고, `tools/` 에 CPU 인스턴스 렌더러(`instance_rig`, 기존 `scene_rig` 의 광선 교차 재사용), 장면 조리법(`recognition_scenes`), 정답 계산(`recognition_truth`), 생성·평가 CLI 를 둔다. Isaac 렌더(단계 1B)와 기하 코어(단계 2)는 이 계획이 만든 타입 위에 별도 계획으로 올린다.

**Tech Stack:** Python ≥ 3.10, numpy, Pillow(선택 의존성 `dataset`), PyYAML(설정), pytest. scipy 는 코어에서 쓰지 않는다(코어 의존성은 numpy 뿐 — `pyproject.toml:10`).

**Spec:** `docs/design/2026-10-15-pallet-recognition-generality.md` (6판 + §13 결정, 커밋 `4800e72`)

## Global Constraints

- 코어 의존성은 `numpy>=1.23` 뿐이다. PNG 는 `PIL` 을 함수 안에서 지연 import 한다(`scene_dataset.py:_read_png` 와 같은 방식).
- `tools/scene_rig.py` 는 **수정하지 않는다** — Isaac 실행 기록이 그 파일의 sha256 을 담고 있다(`tests/fixtures/near_protection_prediction/near_field_bounds_0p29.json:131`). 새 렌더러는 `tools/instance_rig.py` 에서 그 함수를 import 해 쓴다.
- 기존 `pallet_boxes` 와 `PalletGeometry` 의 동작을 바꾸지 않는다. 일반화 형상은 알려진 형상에서 **같은 22 상자(이름·크기·중심)** 를 내야 한다(회귀 시험).
- 형상 YAML: `config/pallet_geometry_epal6.yaml`, `config/pallet_geometry_t11_06.yaml`.
- 포크 실측: 날 폭 0.055 m, 두께 0.025 m, 길이 0.36 m, 승강 0.28 m. 최저 날 중심 0.040 m(모델값, 미실측). 캐리지 한계 0.346 m, 예비량 0.016 m → 삽입 목표 min(깊이 × 0.6, 0.330 m).
- 시험 포크 간격(설계 §13): EPAL 6 **0.3725 m**, T11 × 0.6 **0.290 m**.
- 끼움·위험 판정 상수(설계 §3.1·§3.4·§9.3): M_req 잠정 **0.066 m**, 추종 종점 허용 **0.0338 m**(8 + 25.8 mm), 삽입 끝 투영 길이 **0.33 m**, 종방향 지지 여유 **0.036 m**, 횡방향 지지 여유 **0.020 m**, 세로 여유 **0.005 m**, 적재 한도 **10 kg**.
- 사유 코드 다섯 개(설계 §3.4): `fork_spacing_adjust`, `lift_alignment_needed`, `support_unverified`, `payload_limit`, `near_tracking_unavailable`.
- 통계: 단측 95 % Clopper–Pearson 상한. 분할 eval = ⌈0.3 × 총⌉(ADR 0004 D6). 시험 단위는 장면.
- 장면 수(설계 §3.4): 알려진 형상 S1–S6 각 61 × 2 형상, S7 194, N 94. 형상별 eval 지시 장면 ≥ 59(애매하지 않은 것 ≥ 29, 애매한 것 ≥ 15).
- TF 출처(설계 §13): 스키마 판 2 의 `tf.json` `origin` 은 `received_tf_static` · `isaac_render_pose` · `rig_render_pose` 중 하나. CPU 렌더(이 계획)는 `rig_render_pose` 이고 **기구 회귀용이지 렌더 기반 판정 모집단이 아니다**(CLAUDE.md, ADR 0004 D5 — CPU 결과는 렌더 기반 분모에 넣지 않는다).
- 카메라: `config/isaac_perception_camera.yaml` — 위치 (0.75, 0, 0.27) m, 쿼터니언 xyzw (−0.5, 0.5, −0.5, 0.5), 640 × 480, 초점 465.741156 px.
- pytest 는 ws1 에서 돌린다: `FORKLIFT_REPO=/home/light/school/2-2/embedded-design/forklift-wt-perception ~/bin/forklift-ws1-pytest <인자>`. 이 래퍼는 `git ls-files` 만 보내므로 **새 파일은 먼저 `git add`** 한다. 로컬은 1분 안에 끝나는 한 파일 확인만.
- 커밋은 파일 단위 `git add`, 메시지는 이 저장소의 `<영역>: <내용>` 형식(최근 커밋 참조) + `Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>`.

## Review Focus

1. **맞닿은 팔레트(간격 0)** — 인스턴스 렌더가 두 팔레트를 서로 다른 ID 로 가르고, 정답 노출 비율이 각자의 단독 렌더 대비로 계산돼야 한다(합쳐진 면적을 한 팔레트 것으로 세면 노출 > 1). → 작업 4·7 의 시험.
2. **정사각 T11 의 네 진입면과 앞뒤 면** — 같은 팔레트의 다른 면으로 낸 후보가 그 면의 정답과 대응되고, 카메라를 등진 면의 정답과는 대응되지 않아야 한다. → 작업 7·10 의 시험.
3. **정답상 끼움 불가인 팔레트를 올바르게 멈춘 경우** — 위험 오위치로 세면 안 되고, 집기를 허용하면 T_i 가 음수여도 위험으로 세야 한다. → 작업 10 의 시험.
4. **분모가 0 인 모집단·칸**(eval 에 지시 장면이 없는 칸 등) — 상한은 `null`, 합격 판정은 거짓(통과로 처리하지 않음). → 작업 9·10 의 시험.
5. **학습·평가 시드 대역 겹침** — 생성기가 겹치는 시드를 만들면 즉시 실패해야 한다. → 작업 8 의 시험.

## 파일 구조

| 파일 | 책임 |
|---|---|
| `src/forklift_core/perception/pallet_model.py` (신규) | `PalletSpec`(블록·스트링거 열 N 개, 행, 상판 형식, 진입면) · 알려진 형상 변환 · 상자 생성 · 면 배치 · 포켓 쌍 선택 |
| `src/forklift_core/perception/pallet_damage.py` (신규) | 손상 연산(설계 §5.2) 4종과 적용 |
| `src/forklift_core/perception/fork_fit.py` (신규) | 포크 설정·적재 선언·끼움 판정(사유 코드), 위험 문턱 T_i |
| `src/forklift_core/perception/depth_noise.py` (신규) | D435i 류 잡음 모델(설계 §5.3) |
| `src/forklift_core/perception/recognition_stats.py` (신규) | Clopper–Pearson 상한, 허용 실패 수, 필요 장면 수 |
| `src/forklift_core/perception/recognition_dataset.py` (신규) | 스키마 판 2 쓰기·읽기, 입력/정답 분리 |
| `src/forklift_core/perception/evaluation_multi.py` (신규) | `CandidateView`·`TargetDecision`, 장면 판정, 모집단 집계 |
| `tools/instance_rig.py` (신규) | 인스턴스 ID 를 내는 CPU 광선 렌더 |
| `tools/recognition_scenes.py` (신규) | 사례 조리법 S1–S7·N, 지시, 분할·시드 대역 |
| `tools/recognition_truth.py` (신규) | 진입면별 포켓 정답, 노출 비율, 끼움 정답 |
| `tools/generate_recognition_scenes.py` (신규) | 조리법 → CPU 렌더 → 판 2 장면 디렉터리 |
| `tools/evaluate_recognition.py` (신규) | 기준선(지금 검출기)·oracle 후보 → 장면 판정 → `metrics.json` |
| `docs/decisions/0004-simulation-engine-and-insertion-depth.md` (수정) | D5 개정: 판 2 TF 출처 |
| 시험 `tests/unit/perception/test_<모듈>.py`, `tests/unit/test_<도구>.py` | 각 모듈 |

## 검토 지점

- **검토 A (작업 1–3 뒤):** 형상·손상·끼움 — 수치 규칙이 설계 §3.1·§9.3 과 같은가.
- **검토 B (작업 4–6 뒤):** 렌더·잡음·통계.
- **검토 C (작업 7–9 뒤):** 정답·조리법·데이터 계약.
- **검토 D (작업 10–12 뒤):** 평가기·CLI·기준선 실행 결과.

각 검토는 `node ~/.claude/plugins/cache/openai-codex/codex/1.0.6/scripts/codex-companion.mjs adversarial-review`(브랜치 변경분, 초점 문장 포함) 또는 `task`(질문 파일 stdin)로 하고, 지적이 "새 P1/P2 없음"이 될 때까지 같은 스레드(`--resume-last`)로 반복한다. 대기는 `run_in_background` 로 건다.

---

### Task 1: 일반화 팔레트 형상 `pallet_model`

**Files:**
- Create: `src/forklift_core/perception/pallet_model.py`
- Test: `tests/unit/perception/test_pallet_model.py`

**Interfaces:**
- Consumes: `PalletGeometry`, `PalletBox`, `pallet_boxes`, `load_pallet_geometry` (`pallet_geometry.py`)
- Produces:
  - `PalletSpec` (frozen dataclass): `name: str`, `known: bool`, `overall_width_m`, `overall_depth_m`, `deck_bottom_m`, `opening_height_m`, `stringer_m`, `top_thickness_m`, `column_centres_y_m: tuple[float, ...]`, `column_widths_m: tuple[float, ...]`, `row_centres_x_m: tuple[float, ...]`, `row_depth_m: float`, `top_style: str` (`"boards"`|`"flat"`), `top_board_count: int`, `top_board_width_m: float`, `entry_axes: tuple[str, ...]` (`("x",)` 또는 `("x", "y")`); 속성 `overall_height_m`, `opening_ceiling_m`.
  - `spec_from_geometry(geometry: PalletGeometry, *, entry_axes: tuple[str, ...]) -> PalletSpec`
  - `spec_boxes(spec: PalletSpec) -> list[PalletBox]`
  - `FaceLayout` (frozen): `axis: str`, `width_m: float`, `depth_m: float`, `member_intervals_m: tuple[tuple[float, float], ...]`, `opening_intervals_m: tuple[tuple[float, float], ...]`
  - `face_layout(spec: PalletSpec, axis: str) -> FaceLayout` — 진입면에서 본 배치. 좌표는 면의 **왼쪽 축**(그 면으로 들어갈 때 왼쪽이 +) 기준.
  - `choose_pocket_pair(layout: FaceLayout, fork_spacing_m: float) -> tuple[int, int]` — 개구 쌍 인덱스(좌, 우 아님 — `opening_intervals_m` 의 오름차순 인덱스 두 개).
  - `KNOWN_ENTRY_AXES = {"epal6": ("x",), "t11_06": ("x", "y")}`

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_pallet_model.py
"""Generalised pallet shape: the known shapes must reproduce the 22-box model."""

from pathlib import Path

import pytest

from forklift_core.perception.pallet_geometry import load_pallet_geometry, pallet_boxes
from forklift_core.perception.pallet_model import (
    KNOWN_ENTRY_AXES,
    PalletSpec,
    choose_pocket_pair,
    face_layout,
    spec_boxes,
    spec_from_geometry,
)

ROOT = Path(__file__).resolve().parents[3]


def _known(name):
    geometry = load_pallet_geometry(ROOT / f"config/pallet_geometry_{name}.yaml")
    return geometry, spec_from_geometry(geometry, entry_axes=KNOWN_ENTRY_AXES[name])


@pytest.mark.parametrize("name", ["epal6", "t11_06"])
def test_known_shape_reproduces_the_22_box_model(name):
    geometry, spec = _known(name)
    expected = {b.name: (b.size_m, b.centre_m) for b in pallet_boxes(geometry)}
    produced = {b.name: (b.size_m, b.centre_m) for b in spec_boxes(spec)}
    assert produced.keys() == expected.keys()
    for key, (size, centre) in expected.items():
        assert produced[key][0] == pytest.approx(size, abs=1e-12)
        assert produced[key][1] == pytest.approx(centre, abs=1e-12)


def test_epal6_x_face_openings_match_the_geometry():
    geometry, spec = _known("epal6")
    layout = face_layout(spec, "x")
    widths = [high - low for low, high in layout.opening_intervals_m]
    assert widths == pytest.approx([geometry.opening_width_m] * 2, abs=1e-12)
    assert layout.width_m == pytest.approx(0.800)
    assert layout.depth_m == pytest.approx(0.600)


def test_t11_y_face_uses_the_block_rows():
    _, spec = _known("t11_06")
    layout = face_layout(spec, "y")
    assert len(layout.member_intervals_m) == 3
    assert [high - low for low, high in layout.member_intervals_m] == pytest.approx(
        [0.080] * 3
    )


def test_y_face_is_not_an_entry_face_for_epal6():
    _, spec = _known("epal6")
    with pytest.raises(ValueError, match="not an entry axis"):
        face_layout(spec, "y")


def test_four_column_face_picks_the_pair_closest_to_fork_spacing():
    spec = PalletSpec(
        name="four_column",
        known=False,
        overall_width_m=1.0,
        overall_depth_m=0.6,
        deck_bottom_m=0.02,
        opening_height_m=0.08,
        stringer_m=0.02,
        top_thickness_m=0.02,
        column_centres_y_m=(-0.45, -0.15, 0.15, 0.45),
        column_widths_m=(0.10, 0.10, 0.10, 0.10),
        row_centres_x_m=(-0.25, 0.0, 0.25),
        row_depth_m=0.10,
        top_style="boards",
        top_board_count=7,
        top_board_width_m=0.10,
        entry_axes=("x",),
    )
    layout = face_layout(spec, "x")
    # Openings centred at -0.30, 0.0, 0.30 (left axis); 0.60 apart for (0, 2).
    assert choose_pocket_pair(layout, 0.29) in {(0, 1), (1, 2)}
    assert choose_pocket_pair(layout, 0.60) == (0, 2)


def test_overlapping_columns_are_rejected():
    with pytest.raises(ValueError, match="overlap"):
        PalletSpec(
            name="bad",
            known=False,
            overall_width_m=0.6,
            overall_depth_m=0.6,
            deck_bottom_m=0.02,
            opening_height_m=0.08,
            stringer_m=0.02,
            top_thickness_m=0.02,
            column_centres_y_m=(-0.25, -0.20, 0.25),
            column_widths_m=(0.10, 0.10, 0.10),
            row_centres_x_m=(-0.25, 0.0, 0.25),
            row_depth_m=0.10,
            top_style="boards",
            top_board_count=5,
            top_board_width_m=0.10,
            entry_axes=("x",),
        )
```

- [ ] **Step 2: 실패 확인**

Run (로컬, 한 파일 1분 이내): `python -m pytest tests/unit/perception/test_pallet_model.py -q -p no:cacheprovider`
Expected: FAIL — `ModuleNotFoundError: forklift_core.perception.pallet_model`

- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/pallet_model.py
"""Generalised box-model pallet: N block/stringer columns, any entry axes.

Pallet frame: x along the depth (forks travel +x through an x-entry face),
y across the width, z up from the support surface (floor or the pallet below).
The x-entry face is at x = -depth/2. The fork openings are open to the support
surface: bottom boards run under the columns only, as in ``pallet_boxes``.
"""

from dataclasses import dataclass, fields

from forklift_core._validation import _finite_scalar
from forklift_core.perception.pallet_geometry import PalletBox, PalletGeometry

KNOWN_ENTRY_AXES = {"epal6": ("x",), "t11_06": ("x", "y")}
TOP_STYLES = ("boards", "flat")
_EPS = 1e-9


def _intervals(centres, widths):
    return tuple(
        (c - w / 2, c + w / 2) for c, w in zip(centres, widths, strict=True)
    )


def _check_members(centres, widths, span, label):
    if len(centres) != len(widths) or len(centres) < 3:
        raise ValueError(f"{label} needs at least three members with widths")
    for index, width in enumerate(widths):
        if _finite_scalar(width, f"{label} width[{index}]") <= 0:
            raise ValueError(f"{label} widths must be positive")
    intervals = _intervals(centres, widths)
    if any(b[0] < a[1] - _EPS for a, b in zip(intervals, intervals[1:])):
        raise ValueError(f"{label} members overlap or are out of order")
    if intervals[0][0] < -span / 2 - _EPS or intervals[-1][1] > span / 2 + _EPS:
        raise ValueError(f"{label} members leave the pallet outline")
    if abs(intervals[0][0] + span / 2) > 1e-6 or abs(intervals[-1][1] - span / 2) > 1e-6:
        raise ValueError(f"{label} outer members must touch the edges")


@dataclass(frozen=True)
class PalletSpec:
    name: str
    known: bool
    overall_width_m: float
    overall_depth_m: float
    deck_bottom_m: float
    opening_height_m: float
    stringer_m: float
    top_thickness_m: float
    column_centres_y_m: tuple[float, ...]
    column_widths_m: tuple[float, ...]
    row_centres_x_m: tuple[float, ...]
    row_depth_m: float
    top_style: str
    top_board_count: int
    top_board_width_m: float
    entry_axes: tuple[str, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("name must be nonempty text")
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name.endswith("_m") and not isinstance(value, tuple):
                if _finite_scalar(value, field.name) <= 0:
                    raise ValueError(f"{field.name} must be positive")
        object.__setattr__(self, "column_centres_y_m", tuple(map(float, self.column_centres_y_m)))
        object.__setattr__(self, "column_widths_m", tuple(map(float, self.column_widths_m)))
        object.__setattr__(self, "row_centres_x_m", tuple(map(float, self.row_centres_x_m)))
        _check_members(self.column_centres_y_m, self.column_widths_m, self.overall_width_m, "columns")
        _check_members(
            self.row_centres_x_m,
            (self.row_depth_m,) * len(self.row_centres_x_m),
            self.overall_depth_m,
            "rows",
        )
        if self.top_style not in TOP_STYLES:
            raise ValueError(f"top_style must be one of {TOP_STYLES}")
        if self.top_style == "boards":
            if type(self.top_board_count) is not int or self.top_board_count < 2:
                raise ValueError("top_board_count must be at least two boards")
            if self.top_board_count * self.top_board_width_m > self.overall_width_m + _EPS:
                raise ValueError("Top boards must fit across the width")
        if not self.entry_axes or set(self.entry_axes) - {"x", "y"}:
            raise ValueError("entry_axes must name x and/or y")

    @property
    def overall_height_m(self) -> float:
        return self.deck_bottom_m + self.opening_height_m + self.stringer_m + self.top_thickness_m

    @property
    def opening_ceiling_m(self) -> float:
        """Height of the opening's ceiling above the support surface."""
        return self.deck_bottom_m + self.opening_height_m


def spec_from_geometry(geometry: PalletGeometry, *, entry_axes: tuple[str, ...]) -> PalletSpec:
    g = geometry
    return PalletSpec(
        name=g.geometry_version,
        known=True,
        overall_width_m=g.overall_width_m,
        overall_depth_m=g.overall_depth_m,
        deck_bottom_m=g.deck_bottom_m,
        opening_height_m=g.block_height_m,
        stringer_m=g.stringer_m,
        top_thickness_m=g.top_board_thickness_m,
        column_centres_y_m=tuple(g.block_centres_y_m()),
        column_widths_m=tuple(g.block_widths_m),
        row_centres_x_m=tuple(g.block_centres_x_m()),
        row_depth_m=g.block_depth_m,
        top_style="boards",
        top_board_count=g.top_board_count,
        top_board_width_m=g.top_board_width_m,
        entry_axes=entry_axes,
    )


def _top_board_centres_y(spec: PalletSpec) -> list[float]:
    pitch = (spec.overall_width_m - spec.top_board_width_m) / (spec.top_board_count - 1)
    start = -(spec.overall_width_m - spec.top_board_width_m) / 2
    return [start + i * pitch for i in range(spec.top_board_count)]


def spec_boxes(spec: PalletSpec) -> list[PalletBox]:
    """Same names and order as ``pallet_boxes`` for three-column known shapes."""
    s = spec
    boxes: list[PalletBox] = []
    columns = list(zip(s.column_centres_y_m, s.column_widths_m, strict=True))
    for index, (y, width) in enumerate(columns):
        boxes.append(
            PalletBox(f"bottom_board_{index}", (s.overall_depth_m, width, s.deck_bottom_m), (0.0, y, s.deck_bottom_m / 2))
        )
    block_z = s.deck_bottom_m + s.opening_height_m / 2
    for xi, x in enumerate(s.row_centres_x_m):
        for yi, (y, width) in enumerate(columns):
            boxes.append(
                PalletBox(f"block_x{xi}_y{yi}", (s.row_depth_m, width, s.opening_height_m), (x, y, block_z))
            )
    stringer_z = s.deck_bottom_m + s.opening_height_m + s.stringer_m / 2
    for xi, x in enumerate(s.row_centres_x_m):
        boxes.append(
            PalletBox(f"stringer_{xi}", (s.row_depth_m, s.overall_width_m, s.stringer_m), (x, 0.0, stringer_z))
        )
    top_z = s.overall_height_m - s.top_thickness_m / 2
    if s.top_style == "flat":
        boxes.append(PalletBox("top_deck", (s.overall_depth_m, s.overall_width_m, s.top_thickness_m), (0.0, 0.0, top_z)))
    else:
        for index, y in enumerate(_top_board_centres_y(s)):
            boxes.append(
                PalletBox(f"top_board_{index}", (s.overall_depth_m, s.top_board_width_m, s.top_thickness_m), (0.0, y, top_z))
            )
    return boxes


@dataclass(frozen=True)
class FaceLayout:
    """Members and openings across one entry face, on that face's left axis."""

    axis: str
    width_m: float
    depth_m: float
    member_intervals_m: tuple[tuple[float, float], ...]
    opening_intervals_m: tuple[tuple[float, float], ...]


def face_layout(spec: PalletSpec, axis: str) -> FaceLayout:
    """Entering an x face, left is +y; entering a y face (from -y), left is -x.

    The opposite face of the same axis is the mirror image; callers place it by
    rotating the pallet yaw by pi, which keeps this layout valid.
    """
    if axis not in spec.entry_axes:
        raise ValueError(f"{axis!r} is not an entry axis of {spec.name}")
    if axis == "x":
        members = _intervals(spec.column_centres_y_m, spec.column_widths_m)
        width, depth = spec.overall_width_m, spec.overall_depth_m
    else:
        rows = _intervals(spec.row_centres_x_m, (spec.row_depth_m,) * len(spec.row_centres_x_m))
        members = tuple(sorted((-high, -low) for low, high in rows))
        width, depth = spec.overall_depth_m, spec.overall_width_m
    openings = tuple((a[1], b[0]) for a, b in zip(members, members[1:]))
    return FaceLayout(axis, width, depth, members, openings)


def choose_pocket_pair(layout: FaceLayout, fork_spacing_m: float) -> tuple[int, int]:
    """Opening pair whose centre spacing is closest to the fork spacing.

    Ties go to the pair nearest the face centre, then the lower index.
    """
    centres = [(low + high) / 2 for low, high in layout.opening_intervals_m]
    if len(centres) < 2:
        raise ValueError("A face needs two openings")
    best = None
    for i in range(len(centres)):
        for j in range(i + 1, len(centres)):
            key = (
                round(abs((centres[j] - centres[i]) - fork_spacing_m), 9),
                round(abs((centres[i] + centres[j]) / 2), 9),
                i,
            )
            if best is None or key < best[0]:
                best = (key, (i, j))
    return best[1]
```

- [ ] **Step 4: 통과 확인**

Run: `python -m pytest tests/unit/perception/test_pallet_model.py -q -p no:cacheprovider`
Expected: 6 passed

- [ ] **Step 5: 커밋**

```bash
git add src/forklift_core/perception/pallet_model.py tests/unit/perception/test_pallet_model.py
git commit -m "perception: generalised pallet shape (N columns, entry axes, flat or board top) that reproduces the 22-box model for the known shapes"
```

### Task 2: 손상 연산 `pallet_damage`

**Files:**
- Create: `src/forklift_core/perception/pallet_damage.py`
- Test: `tests/unit/perception/test_pallet_damage.py`

**Interfaces:**
- Consumes: `PalletBox`, `spec_boxes`, `PalletSpec`
- Produces:
  - `DamageOp` (frozen): `kind: str` ∈ `DAMAGE_KINDS = ("top_board_missing", "top_board_broken", "block_corner_chipped", "block_split")`, `target: str`(상자 이름), `params: tuple[tuple[str, float], ...]`
  - `apply_damage(boxes: list[PalletBox], ops: Sequence[DamageOp]) -> list[PalletBox]`
  - `removed_volume_fraction(before: list[PalletBox], after: list[PalletBox]) -> float`

연산 정의(설계 §5.2 범위를 생성기와 이 모듈이 함께 강제한다):
- `top_board_missing`: 대상 `top_board_<i>` 제거.
- `top_board_broken`: 대상 판을 x 방향 `fraction`(0.2–0.6) 지점에서 잘라 `keep`(0 = 앞쪽 유지, 1 = 뒤쪽 유지) 조각만 남긴다. 판은 x 방향으로 길다(`pallet_geometry.py:328–334`).
- `block_corner_chipped`: 대상 블록의 진입면(−x) 쪽 모서리를 `depth`·`width`(각 0.010–0.040 m)만큼 깎는다. `side` 0 = −y 모서리, 1 = +y 모서리. 블록은 (뒤 부분 전체 폭) + (앞 부분 깎인 폭) 두 상자로 바뀐다. 깎인 부피는 블록의 30 % 이하.
- `block_split`: 대상 블록을 y 방향 가운데에서 `gap`(0.005–0.015 m) 틈으로 둘로 가른다.

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_pallet_damage.py
from pathlib import Path

import pytest

from forklift_core.perception.pallet_damage import (
    DamageOp,
    apply_damage,
    removed_volume_fraction,
)
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_model import spec_boxes, spec_from_geometry

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def epal_boxes():
    g = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    return spec_boxes(spec_from_geometry(g, entry_axes=("x",)))


def _by_name(boxes):
    return {b.name: b for b in boxes}


def test_missing_board_removes_exactly_that_board(epal_boxes):
    out = apply_damage(epal_boxes, [DamageOp("top_board_missing", "top_board_3", ())])
    assert "top_board_3" not in _by_name(out)
    assert len(out) == len(epal_boxes) - 1


def test_broken_board_keeps_the_front_piece_shorter(epal_boxes):
    op = DamageOp("top_board_broken", "top_board_0", (("fraction", 0.4), ("keep", 0.0)))
    piece = _by_name(apply_damage(epal_boxes, [op]))["top_board_0"]
    assert piece.size_m[0] == pytest.approx(0.6 * 0.4)
    # Front of the board stays at x = -0.30 (the entry face).
    assert piece.centre_m[0] - piece.size_m[0] / 2 == pytest.approx(-0.30)


def test_chipped_corner_leaves_a_notch_on_the_entry_face(epal_boxes):
    op = DamageOp(
        "block_corner_chipped",
        "block_x0_y0",
        (("depth", 0.03), ("width", 0.02), ("side", 1.0)),
    )
    out = apply_damage(epal_boxes, [op])
    parts = [b for b in out if b.name.startswith("block_x0_y0")]
    assert len(parts) == 2
    front = min(parts, key=lambda b: b.centre_m[0])
    assert front.size_m[1] == pytest.approx(0.100 - 0.02)
    assert 0 < removed_volume_fraction(epal_boxes, out) < 0.01


def test_split_block_leaves_a_gap(epal_boxes):
    op = DamageOp("block_split", "block_x0_y1", (("gap", 0.01),))
    parts = [b for b in apply_damage(epal_boxes, [op]) if b.name.startswith("block_x0_y1")]
    assert len(parts) == 2
    widths = sorted(p.size_m[1] for p in parts)
    assert sum(widths) == pytest.approx(0.145 - 0.01)


@pytest.mark.parametrize(
    "op",
    [
        DamageOp("top_board_broken", "top_board_0", (("fraction", 0.9), ("keep", 0.0))),
        DamageOp("block_corner_chipped", "block_x0_y0", (("depth", 0.08), ("width", 0.08), ("side", 0.0))),
        DamageOp("block_split", "block_x0_y0", (("gap", 0.05),)),
        DamageOp("top_board_missing", "block_x0_y0", ()),
    ],
)
def test_out_of_range_or_wrong_target_is_rejected(epal_boxes, op):
    with pytest.raises(ValueError):
        apply_damage(epal_boxes, [op])
```

- [ ] **Step 2: 실패 확인** — Run: `python -m pytest tests/unit/perception/test_pallet_damage.py -q -p no:cacheprovider` · Expected: `ModuleNotFoundError`

- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/pallet_damage.py
"""Damage operators on a pallet box list (design §5.2). Render, physics and mass
all come from the same output list, so a damaged pallet is one object."""

from collections.abc import Sequence
from dataclasses import dataclass

from forklift_core.perception.pallet_geometry import PalletBox

DAMAGE_KINDS = (
    "top_board_missing",
    "top_board_broken",
    "block_corner_chipped",
    "block_split",
)
_RANGES = {
    ("top_board_broken", "fraction"): (0.2, 0.6),
    ("block_corner_chipped", "depth"): (0.010, 0.040),
    ("block_corner_chipped", "width"): (0.010, 0.040),
    ("block_split", "gap"): (0.005, 0.015),
}


@dataclass(frozen=True)
class DamageOp:
    kind: str
    target: str
    params: tuple[tuple[str, float], ...]

    def __post_init__(self) -> None:
        if self.kind not in DAMAGE_KINDS:
            raise ValueError(f"Unknown damage kind {self.kind!r}")
        prefix = "top_board_" if self.kind.startswith("top_board") else "block_"
        if not self.target.startswith(prefix):
            raise ValueError(f"{self.kind} must target a {prefix}* box")
        for name, value in self.params:
            bounds = _RANGES.get((self.kind, name))
            if bounds and not bounds[0] <= value <= bounds[1]:
                raise ValueError(f"{self.kind}.{name}={value} outside {bounds}")

    def get(self, name: str) -> float:
        for key, value in self.params:
            if key == name:
                return float(value)
        raise ValueError(f"{self.kind} needs parameter {name!r}")


def _volume(box: PalletBox) -> float:
    x, y, z = box.size_m
    return x * y * z


def _apply_one(box: PalletBox, op: DamageOp) -> list[PalletBox]:
    (sx, sy, sz), (cx, cy, cz) = box.size_m, box.centre_m
    if op.kind == "top_board_missing":
        return []
    if op.kind == "top_board_broken":
        length = sx * op.get("fraction")
        keep_front = op.get("keep") == 0.0
        x0 = cx - sx / 2 if keep_front else cx + sx / 2 - length
        return [PalletBox(box.name, (length, sy, sz), (x0 + length / 2, cy, cz))]
    if op.kind == "block_corner_chipped":
        depth, width = op.get("depth"), op.get("width")
        if depth * width * sz > 0.3 * _volume(box):
            raise ValueError("A chipped corner may remove at most 30 % of the block")
        sign = 1.0 if op.get("side") == 1.0 else -1.0
        front_x0 = cx - sx / 2
        rear = PalletBox(f"{box.name}_rear", (sx - depth, sy, sz), (front_x0 + depth + (sx - depth) / 2, cy, cz))
        front = PalletBox(
            f"{box.name}_front", (depth, sy - width, sz), (front_x0 + depth / 2, cy - sign * width / 2, cz)
        )
        return [rear, front]
    gap = op.get("gap")
    half = (sy - gap) / 2
    return [
        PalletBox(f"{box.name}_a", (sx, half, sz), (cx, cy - (gap + half) / 2, cz)),
        PalletBox(f"{box.name}_b", (sx, half, sz), (cx, cy + (gap + half) / 2, cz)),
    ]


def apply_damage(boxes: list[PalletBox], ops: Sequence[DamageOp]) -> list[PalletBox]:
    out = list(boxes)
    for op in ops:
        names = [b.name for b in out]
        if op.target not in names:
            raise ValueError(f"No box named {op.target!r} (already damaged?)")
        index = names.index(op.target)
        out[index : index + 1] = _apply_one(out[index], op)
    return out


def removed_volume_fraction(before: list[PalletBox], after: list[PalletBox]) -> float:
    total = sum(_volume(b) for b in before)
    return (total - sum(_volume(b) for b in after)) / total
```

- [ ] **Step 4: 통과 확인** — Expected: 8 passed
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/pallet_damage.py tests/unit/perception/test_pallet_damage.py` · `git commit -m "perception: pallet damage operators (missing/broken top board, chipped corner, split block) on one box list"`

### Task 3: 포크 끼움 정답 `fork_fit`

**Files:**
- Create: `src/forklift_core/perception/fork_fit.py`
- Test: `tests/unit/perception/test_fork_fit.py`

**Interfaces:**
- Consumes: `PalletSpec`, `face_layout`, `choose_pocket_pair`, `target_insertion_depth_m` (`pallet_geometry.py:257`)
- Produces:
  - 상수 `M_REQ_PROVISIONAL_M = 0.066`, `TRACKING_ALLOWANCE_M = 0.0338`, `TIP_LEVER_M = 0.33`, `SUPPORT_LONGITUDINAL_MARGIN_M = 0.036`, `SUPPORT_LATERAL_MARGIN_M = 0.020`, `VERTICAL_MARGIN_M = 0.005`, `PAYLOAD_LIMIT_KG = 10.0`, `REASON_CODES`
  - `ForkSetup` (frozen): `spacing_m`, `blade_width_m=0.055`, `blade_thickness_m=0.025`, `lowest_centre_m=0.040`, `carriage_limit_m=0.346`, `reserve_m=0.016`; 속성 `blade_top_m`, `blade_bottom_m`
  - `TEST_FORK_SPACING_M = {"epal6": 0.3725, "t11_06": 0.290}`
  - `LoadDeclaration` (frozen): `pallet_mass_kg`, `load_mass_kg`, `com_x_max_m`(진입면에서), `com_y_max_m`(팔레트 중심선에서)
  - `FitResult` (frozen): `pickable: bool`, `reasons: tuple[str, ...]`, `pair: tuple[int, int]`, `min_lateral_margin_m: float`, `fork_centre_offset_m: float`, `insertion_target_m: float`
  - `fork_fit(spec, axis, setup, load: LoadDeclaration | None, *, m_req_m=M_REQ_PROVISIONAL_M) -> FitResult`
  - `danger_threshold_m(fit: FitResult) -> float` = `fit.min_lateral_margin_m - TRACKING_ALLOWANCE_M`

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_fork_fit.py
from pathlib import Path

import pytest

from forklift_core.perception.fork_fit import (
    TEST_FORK_SPACING_M,
    ForkSetup,
    LoadDeclaration,
    danger_threshold_m,
    fork_fit,
)
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_model import (
    KNOWN_ENTRY_AXES,
    PalletSpec,
    spec_from_geometry,
)

ROOT = Path(__file__).resolve().parents[3]


def _known(name):
    g = load_pallet_geometry(ROOT / f"config/pallet_geometry_{name}.yaml")
    return spec_from_geometry(g, entry_axes=KNOWN_ENTRY_AXES[name])


def _nonstandard(**overrides):
    base = dict(
        name="ns",
        known=False,
        overall_width_m=0.82,
        overall_depth_m=0.55,
        deck_bottom_m=0.022,
        opening_height_m=0.078,
        stringer_m=0.022,
        top_thickness_m=0.022,
        column_centres_y_m=(-0.36, 0.0, 0.36),
        column_widths_m=(0.10, 0.145, 0.10),
        row_centres_x_m=(-0.2025, 0.0, 0.2025),
        row_depth_m=0.145,
        top_style="boards",
        top_board_count=7,
        top_board_width_m=0.10,
        entry_axes=("x",),
    )
    base.update(overrides)
    return PalletSpec(**base)


CENTRED = LoadDeclaration(2.0, 6.0, com_x_max_m=0.275, com_y_max_m=0.0)


@pytest.mark.parametrize(("name", "margin"), [("epal6", 0.08625), ("t11_06", 0.0775)])
def test_known_shapes_at_their_test_spacing(name, margin):
    fit = fork_fit(_known(name), "x", ForkSetup(TEST_FORK_SPACING_M[name]), None)
    assert fit.pickable and fit.reasons == ()
    assert fit.min_lateral_margin_m == pytest.approx(margin, abs=1e-9)


def test_t11_at_the_epal_spacing_needs_adjustment():
    fit = fork_fit(_known("t11_06"), "x", ForkSetup(0.3725), None)
    assert fit.min_lateral_margin_m == pytest.approx(0.03625, abs=1e-9)
    assert fit.reasons == ("fork_spacing_adjust",)


def test_danger_threshold_is_margin_minus_tracking_allowance():
    fit = fork_fit(_known("t11_06"), "x", ForkSetup(0.290), None)
    assert danger_threshold_m(fit) == pytest.approx(0.0775 - 0.0338)


def test_nonstandard_needs_a_load_declaration():
    fit = fork_fit(_nonstandard(), "x", ForkSetup(0.3725), None)
    assert "support_unverified" in fit.reasons


def test_nonstandard_centred_load_passes_at_0_55_depth():
    fit = fork_fit(_nonstandard(), "x", ForkSetup(0.3725), CENTRED)
    assert fit.pickable, fit.reasons


def test_offset_centre_of_mass_fails_lateral_support():
    load = LoadDeclaration(2.0, 6.0, com_x_max_m=0.275, com_y_max_m=0.24)
    fit = fork_fit(_nonstandard(), "x", ForkSetup(0.3725), load)
    assert "support_unverified" in fit.reasons


def test_deep_pallet_fails_longitudinal_support():
    deep = _nonstandard(overall_depth_m=0.60, row_centres_x_m=(-0.2275, 0.0, 0.2275))
    load = LoadDeclaration(2.0, 6.0, com_x_max_m=0.314, com_y_max_m=0.0)
    assert "support_unverified" in fork_fit(deep, "x", ForkSetup(0.3725), load).reasons


def test_flat_top_cannot_be_near_tracked():
    flat = _nonstandard(top_style="flat")
    assert "near_tracking_unavailable" in fork_fit(flat, "x", ForkSetup(0.3725), CENTRED).reasons


def test_low_ceiling_needs_lift_alignment():
    low = _nonstandard(deck_bottom_m=0.010, opening_height_m=0.045)
    # Ceiling 0.055 < blade top 0.0525 + 0.005.
    assert "lift_alignment_needed" in fork_fit(low, "x", ForkSetup(0.3725), CENTRED).reasons


def test_payload_limit():
    heavy = LoadDeclaration(4.0, 7.0, com_x_max_m=0.275, com_y_max_m=0.0)
    assert "payload_limit" in fork_fit(_nonstandard(), "x", ForkSetup(0.3725), heavy).reasons
```

- [ ] **Step 2: 실패 확인** — `ModuleNotFoundError`

- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/fork_fit.py
"""Fork-fit truth and decision with reason codes (design §3.1, §9.2, §9.3).

Margins are measured from the real blade edges at the configured spacing, with
the fork centre on the chosen opening pair's midpoint. The support check is a
reach / footprint precheck against an instructed centre of mass, not a support
polygon or contact verification (ADR 0004 D3).
"""

from dataclasses import dataclass

from forklift_core.perception.pallet_geometry import target_insertion_depth_m
from forklift_core.perception.pallet_model import PalletSpec, choose_pocket_pair, face_layout

M_REQ_PROVISIONAL_M = 0.066
TRACKING_ALLOWANCE_M = 0.0338
TIP_LEVER_M = 0.33
SUPPORT_LONGITUDINAL_MARGIN_M = 0.036
SUPPORT_LATERAL_MARGIN_M = 0.020
VERTICAL_MARGIN_M = 0.005
PAYLOAD_LIMIT_KG = 10.0
REASON_CODES = (
    "fork_spacing_adjust",
    "lift_alignment_needed",
    "support_unverified",
    "payload_limit",
    "near_tracking_unavailable",
)
TEST_FORK_SPACING_M = {"epal6": 0.3725, "t11_06": 0.290}


@dataclass(frozen=True)
class ForkSetup:
    spacing_m: float
    blade_width_m: float = 0.055
    blade_thickness_m: float = 0.025
    lowest_centre_m: float = 0.040
    carriage_limit_m: float = 0.346
    reserve_m: float = 0.016

    @property
    def blade_top_m(self) -> float:
        return self.lowest_centre_m + self.blade_thickness_m / 2

    @property
    def blade_bottom_m(self) -> float:
        return self.lowest_centre_m - self.blade_thickness_m / 2


@dataclass(frozen=True)
class LoadDeclaration:
    pallet_mass_kg: float
    load_mass_kg: float
    com_x_max_m: float
    com_y_max_m: float


@dataclass(frozen=True)
class FitResult:
    pickable: bool
    reasons: tuple[str, ...]
    pair: tuple[int, int]
    min_lateral_margin_m: float
    fork_centre_offset_m: float
    insertion_target_m: float


def fork_fit(
    spec: PalletSpec,
    axis: str,
    setup: ForkSetup,
    load: LoadDeclaration | None,
    *,
    m_req_m: float = M_REQ_PROVISIONAL_M,
) -> FitResult:
    layout = face_layout(spec, axis)
    pair = choose_pocket_pair(layout, setup.spacing_m)
    (l0, h0), (l1, h1) = (layout.opening_intervals_m[i] for i in pair)
    centre = ((l0 + h0) / 2 + (l1 + h1) / 2) / 2
    half = setup.spacing_m / 2
    w = setup.blade_width_m / 2
    blades = ((centre - half - w, centre - half + w), (centre + half - w, centre + half + w))
    margins = (blades[0][0] - l0, h0 - blades[0][1], blades[1][0] - l1, h1 - blades[1][1])
    min_margin = min(margins)
    target = target_insertion_depth_m(layout.depth_m, setup.carriage_limit_m, setup.reserve_m)
    reasons = []
    if min_margin < m_req_m:
        reasons.append("fork_spacing_adjust")
    if spec.opening_ceiling_m < setup.blade_top_m + VERTICAL_MARGIN_M:
        reasons.append("lift_alignment_needed")
    if not spec.known:
        if (
            load is None
            or target < load.com_x_max_m + SUPPORT_LONGITUDINAL_MARGIN_M
            or abs(centre) + load.com_y_max_m > half - SUPPORT_LATERAL_MARGIN_M
        ):
            reasons.append("support_unverified")
        if load is not None and load.pallet_mass_kg + load.load_mass_kg > PAYLOAD_LIMIT_KG:
            reasons.append("payload_limit")
        if spec.top_style != "boards":
            reasons.append("near_tracking_unavailable")
    elif load is not None and load.pallet_mass_kg + load.load_mass_kg > PAYLOAD_LIMIT_KG:
        reasons.append("payload_limit")
    ordered = tuple(code for code in REASON_CODES if code in reasons)
    return FitResult(not ordered, ordered, pair, min_margin, centre, target)


def danger_threshold_m(fit: FitResult) -> float:
    """T_i (design §3.4): the blade-tip lateral displacement that reaches contact."""
    return fit.min_lateral_margin_m - TRACKING_ALLOWANCE_M
```

- [ ] **Step 4: 통과 확인** — Expected: 11 passed
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/fork_fit.py tests/unit/perception/test_fork_fit.py` · `git commit -m "perception: fork-fit truth with real blade margins, support precheck against an instructed centre of mass and five reason codes"`

**검토 A:** 작업 1–3 커밋 뒤 Codex `adversarial-review` — 초점: "pallet_model 이 알려진 형상에서 pallet_boxes 와 같은가, fork_fit 의 여유·지지·세로 규칙이 설계 §3.1·§9.2·§9.3 수치와 같은가, 손상 범위가 §5.2 와 같은가". 수렴까지 반복.

### Task 4: 인스턴스 렌더 `instance_rig`

**Files:**
- Create: `tools/instance_rig.py`
- Test: `tests/unit/test_instance_rig.py`

**Interfaces:**
- Consumes: `tools.scene_rig` 의 `Box`, `Camera`, `_rays`, `_box_depth`, `_yaw_matrix`, `_rig_intrinsics`, `BACK_WALL_X_M`; `SceneInput`
- Produces:
  - `PERCEPTION_CAMERA = scene_rig.Camera((0.75, 0.0, 0.27), 0.0)`
  - `RenderedScene` (frozen): `scene: SceneInput`(깨끗한 depth, 양자화 없음), `instance_ids: NDArray[np.int32]`(−1 = 배경·바닥·벽, −2 = 방해물, k ≥ 0 = 인스턴스 k)
  - `render_instances(instances: Sequence[Sequence[Box]], *, distractors: Sequence[Box] = (), camera=PERCEPTION_CAMERA, intrinsics=None, floor=True, back_wall=True, stamp_ns=2_000_000_000) -> RenderedScene`
  - `solo_pixel_count(instance: Sequence[Box], *, camera=PERCEPTION_CAMERA, intrinsics=None) -> int`

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/test_instance_rig.py
import numpy as np

from tools import instance_rig, scene_rig


def _cube(x, y, size=0.3):
    return [scene_rig.Box((x, y, size / 2), (size, size, size))]


def test_two_touching_boxes_get_their_own_ids():
    left, right = _cube(2.0, 0.15), _cube(2.0, -0.15)
    out = instance_rig.render_instances([left, right])
    ids = set(np.unique(out.instance_ids))
    assert {0, 1} <= ids
    # Left (+y) appears left in the image (smaller u).
    u0 = np.argwhere(out.instance_ids == 0)[:, 1].mean()
    u1 = np.argwhere(out.instance_ids == 1)[:, 1].mean()
    assert u0 < u1


def test_depth_matches_scene_rig_without_quantisation():
    boxes = _cube(2.0, 0.0)
    ours = instance_rig.render_instances([boxes]).scene.depth_m
    theirs = scene_rig.render(boxes, camera=instance_rig.PERCEPTION_CAMERA, quantize=False).depth_m
    assert np.array_equal(np.isnan(ours), np.isnan(theirs))
    assert np.allclose(ours[~np.isnan(ours)], theirs[~np.isnan(theirs)])


def test_occluded_instance_loses_pixels_against_its_solo_render():
    front, back = _cube(1.6, 0.0), _cube(2.4, 0.0)
    out = instance_rig.render_instances([front, back])
    visible = int((out.instance_ids == 1).sum())
    assert visible < instance_rig.solo_pixel_count(back)


def test_distractors_are_marked_minus_two():
    out = instance_rig.render_instances([], distractors=_cube(2.0, 0.0))
    assert (out.instance_ids == -2).any()
```

- [ ] **Step 2: 실패 확인** — Run: `python -m pytest tests/unit/test_instance_rig.py -q -p no:cacheprovider` · Expected: `ImportError`

- [ ] **Step 3: 구현**

```python
# tools/instance_rig.py
"""CPU first-hit renderer that also reports which instance each pixel hit.

Reuses tools.scene_rig's slab intersection unchanged (that file's hash is part
of recorded Isaac runs, so it is not edited). Depth is clean and unquantised;
forklift_core.perception.depth_noise adds the sensor model afterwards.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from forklift_core.perception.scene_dataset import SceneInput
from tools import scene_rig

PERCEPTION_CAMERA = scene_rig.Camera((0.75, 0.0, 0.27), 0.0)
BACKGROUND, DISTRACTOR = -1, -2


@dataclass(frozen=True)
class RenderedScene:
    scene: SceneInput
    instance_ids: NDArray[np.int32]


def _hit(origin, rays, box: scene_rig.Box):
    return scene_rig._box_depth(
        origin, rays, np.asarray(box.centre_m, float), box.size_m, scene_rig._yaw_matrix(box.yaw_rad)
    )


def render_instances(
    instances: Sequence[Sequence[scene_rig.Box]],
    *,
    distractors: Sequence[scene_rig.Box] = (),
    camera: scene_rig.Camera = PERCEPTION_CAMERA,
    intrinsics=None,
    floor: bool = True,
    back_wall: bool = True,
    stamp_ns: int = 2_000_000_000,
) -> RenderedScene:
    spec = scene_rig._rig_intrinsics() if intrinsics is None else intrinsics
    transform = camera.base_from_optical()
    origin = np.asarray(transform.translation_m, dtype=float)
    rays = scene_rig._rays(spec, transform.rotation)
    depth = np.full((spec.height, spec.width), np.inf)
    ids = np.full(depth.shape, BACKGROUND, dtype=np.int32)
    with np.errstate(divide="ignore", invalid="ignore"):
        if floor:
            t = -origin[2] / rays[..., 2]
            depth = np.fmin(depth, np.where((rays[..., 2] < 0) & (t > 0), t, np.inf))
        if back_wall:
            t = (scene_rig.BACK_WALL_X_M - origin[0]) / rays[..., 0]
            depth = np.fmin(depth, np.where((rays[..., 0] > 0) & (t > 0), t, np.inf))
    labelled = [(DISTRACTOR, b) for b in distractors]
    labelled += [(k, b) for k, boxes in enumerate(instances) for b in boxes]
    for label, box in labelled:
        hit = _hit(origin, rays, box)
        closer = hit < depth
        depth = np.where(closer, hit, depth)
        ids = np.where(closer, label, ids)
    depth = np.where(np.isfinite(depth), depth, np.nan)
    scene = SceneInput(
        rgb=np.zeros((spec.height, spec.width, 3), dtype=np.uint8),
        depth_m=depth,
        intrinsics=spec,
        base_from_optical=transform,
        stamp_ns=stamp_ns,
        clock_domain="ros_sim",
        source_provenance="synthetic",
    )
    return RenderedScene(scene, ids)


def solo_pixel_count(
    instance: Sequence[scene_rig.Box], *, camera: scene_rig.Camera = PERCEPTION_CAMERA, intrinsics=None
) -> int:
    out = render_instances([instance], camera=camera, intrinsics=intrinsics, floor=False, back_wall=False)
    return int((out.instance_ids == 0).sum())
```

- [ ] **Step 4: 통과 확인** — Expected: 4 passed
- [ ] **Step 5: 커밋** — `git add tools/instance_rig.py tests/unit/test_instance_rig.py` · `git commit -m "tools: CPU instance renderer on scene_rig's slab intersection (scene_rig untouched), with solo pixel counts for exposure"`

### Task 5: 깊이 잡음 모델 `depth_noise`

**Files:**
- Create: `src/forklift_core/perception/depth_noise.py`
- Test: `tests/unit/perception/test_depth_noise.py`

**Interfaces:**
- Consumes: `SceneInput`
- Produces:
  - `NoiseModel` (frozen): `axial_k: float = 0.005`(σ = k·z², ADR 없는 모델값 — `scene_rig.py` 의 `DEFAULT_NOISE_K` 설명 참조), `edge_jump_m: float = 0.05`, `flying_fraction: float = 0.5`, `registration_shift_px: tuple[int, int] = (2, 0)`, `dropout_fraction: float = 0.01`, `quantize_step_m: float = 0.001`, `seed: int = 0`; `to_json() -> dict`
  - `apply_noise(scene: SceneInput, labels: NDArray[np.int32], model: NoiseModel) -> tuple[SceneInput, NDArray[np.int32]]` — depth 에 축 잡음·경계 flying pixel·무효 화소·1 mm 양자화를 넣고, **RGB 와 라벨 영상을 depth 격자에 대해 `registration_shift_px` 만큼 민다**(D435i 의 RGB–depth 정렬 오차 모사). 입력은 바꾸지 않는다.

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_depth_noise.py
import numpy as np

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from tools import instance_rig, scene_rig


def _scene():
    boxes = [scene_rig.Box((2.0, 0.0, 0.15), (0.3, 0.6, 0.3))]
    return instance_rig.render_instances([boxes])


def test_noise_is_deterministic_and_leaves_input_alone():
    out = _scene()
    before = out.scene.depth_m.copy()
    a, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=3))
    b, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=3))
    assert np.array_equal(a.depth_m, b.depth_m, equal_nan=True)
    assert np.array_equal(out.scene.depth_m, before, equal_nan=True)


def test_axial_noise_grows_with_range():
    out = _scene()
    noisy, _ = apply_noise(
        out.scene, out.instance_ids,
        NoiseModel(edge_jump_m=1e9, dropout_fraction=0.0, quantize_step_m=1e-9, registration_shift_px=(0, 0)),
    )
    diff = noisy.depth_m - out.scene.depth_m
    near = np.abs(out.scene.depth_m - 2.0) < 0.2
    assert 0.5 * 0.005 * 4 < np.nanstd(diff[near]) < 2.0 * 0.005 * 4


def test_labels_shift_with_registration():
    out = _scene()
    _, labels = apply_noise(out.scene, out.instance_ids, NoiseModel(registration_shift_px=(3, 0)))
    cols = np.argwhere(out.instance_ids == 0)[:, 1]
    shifted = np.argwhere(labels == 0)[:, 1]
    assert shifted.min() == cols.min() + 3


def test_dropout_makes_some_pixels_unknown():
    out = _scene()
    noisy, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(dropout_fraction=0.05))
    assert np.isnan(noisy.depth_m).sum() > np.isnan(out.scene.depth_m).sum()
```

- [ ] **Step 2: 실패 확인** — `ModuleNotFoundError`

- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/depth_noise.py
"""A datasheet-level D435i-like depth model for rendered scenes (design §5.3).

Every parameter is a model value, not a measurement: axial sigma k*z^2, mixed
("flying") pixels across depth edges, random dropouts, 1 mm rounding, and a
fixed RGB/label shift against the depth grid. Gates are judged on the noisy
variant because clean renders make p95 optimistic.
"""

from dataclasses import asdict, dataclass, replace

import numpy as np
from numpy.typing import NDArray

from forklift_core.perception.scene_dataset import SceneInput


@dataclass(frozen=True)
class NoiseModel:
    axial_k: float = 0.005
    edge_jump_m: float = 0.05
    flying_fraction: float = 0.5
    registration_shift_px: tuple[int, int] = (2, 0)
    dropout_fraction: float = 0.01
    quantize_step_m: float = 0.001
    seed: int = 0

    def to_json(self) -> dict:
        obj = asdict(self)
        obj["registration_shift_px"] = list(self.registration_shift_px)
        return obj


def _shift(image, du, dv, fill):
    out = np.full_like(image, fill)
    h, w = image.shape[:2]
    src_u = slice(max(0, -du), min(w, w - du))
    dst_u = slice(max(0, du), min(w, w + du))
    src_v = slice(max(0, -dv), min(h, h - dv))
    dst_v = slice(max(0, dv), min(h, h + dv))
    out[dst_v, dst_u] = image[src_v, src_u]
    return out


def apply_noise(
    scene: SceneInput, labels: NDArray[np.int32], model: NoiseModel
) -> tuple[SceneInput, NDArray[np.int32]]:
    rng = np.random.default_rng(model.seed)
    depth = scene.depth_m.astype(np.float64, copy=True)
    finite = np.isfinite(depth)
    sigma = model.axial_k * np.square(np.where(finite, depth, 0.0))
    depth = np.where(finite, depth + rng.normal(0.0, 1.0, depth.shape) * sigma, depth)
    # Flying pixels: on a depth jump, the nearer pixel's value is mixed with the
    # farther neighbour's, as stereo matching does across occlusion edges.
    right = np.roll(depth, -1, axis=1)
    jump = finite & np.isfinite(right) & (np.abs(right - depth) > model.edge_jump_m)
    jump[:, -1] = False
    mix = jump & (rng.random(depth.shape) < model.flying_fraction)
    t = rng.random(depth.shape)
    depth = np.where(mix, depth + t * (right - depth), depth)
    depth = np.where(rng.random(depth.shape) < model.dropout_fraction, np.nan, depth)
    if model.quantize_step_m > 0:
        depth = np.round(depth / model.quantize_step_m) * model.quantize_step_m
    du, dv = model.registration_shift_px
    rgb = _shift(scene.rgb, du, dv, 0)
    shifted = _shift(labels, du, dv, -1)
    return replace(scene, depth_m=depth, rgb=rgb), shifted
```

- [ ] **Step 4: 통과 확인** — Expected: 4 passed
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/depth_noise.py tests/unit/perception/test_depth_noise.py` · `git commit -m "perception: datasheet-level depth noise model (axial k*z^2, flying pixels, dropouts, 1 mm rounding, RGB/label registration shift)"`

### Task 6: 통계 `recognition_stats`

**Files:**
- Create: `src/forklift_core/perception/recognition_stats.py`
- Test: `tests/unit/perception/test_recognition_stats.py`

**Interfaces:**
- Produces:
  - `cp_upper(failures: int, n: int, confidence: float = 0.95) -> float | None` — n = 0 이면 None. 이항 누적분포를 정확한 정수 합으로 계산하고 이분법으로 푼다(scipy 없음).
  - `allowed_failures(n: int, bound: float) -> int | None` — 상한 ≤ bound 인 최대 실패 수, 하나도 없으면 None(n = 0 포함).
  - `min_scenes(failures: int, bound: float) -> int`
  - `eval_count(total: int) -> int` = ⌈0.3 × total⌉

- [ ] **Step 1: 실패하는 시험 작성** — 값은 설계 §3.3 표(scipy 로 2026-10-15 계산, Codex 재계산 일치)

```python
# tests/unit/perception/test_recognition_stats.py
import pytest

from forklift_core.perception.recognition_stats import (
    allowed_failures,
    cp_upper,
    eval_count,
    min_scenes,
)


@pytest.mark.parametrize(
    ("bound", "k", "n"),
    [(0.05, 0, 59), (0.05, 1, 93), (0.05, 2, 124), (0.10, 0, 29), (0.10, 1, 46),
     (0.10, 2, 61), (0.15, 0, 19), (0.15, 1, 30), (0.20, 0, 14), (0.20, 1, 22)],
)
def test_min_scenes_matches_the_design_table(bound, k, n):
    assert min_scenes(k, bound) == n


def test_known_values():
    assert cp_upper(0, 59) == pytest.approx(0.0495, abs=1e-4)
    assert cp_upper(1, 114) == pytest.approx(0.04094, abs=1e-4)
    assert cp_upper(5, 100) == pytest.approx(0.10225, abs=1e-4)
    assert cp_upper(1, 20) == pytest.approx(0.21611, abs=1e-4)


def test_allowed_failures_at_114():
    assert allowed_failures(114, 0.05) == 1
    assert allowed_failures(114, 0.10) == 5


def test_empty_population_has_no_bound():
    assert cp_upper(0, 0) is None
    assert allowed_failures(0, 0.05) is None
    assert allowed_failures(10, 0.05) is None  # even 0/10 exceeds 5 %


def test_eval_split_convention():
    assert [eval_count(n) for n in (44, 61, 94, 194)] == [14, 19, 29, 59]
```

- [ ] **Step 2: 실패 확인** — `ModuleNotFoundError`

- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/recognition_stats.py
"""One-sided Clopper-Pearson bounds without scipy (design §3.3)."""

import math


def _binom_cdf(k: int, n: int, p: float) -> float:
    if p <= 0.0:
        return 1.0
    if p >= 1.0:
        return 0.0 if k < n else 1.0
    log_p, log_q = math.log(p), math.log1p(-p)
    return min(1.0, sum(math.exp(math.log(math.comb(n, i)) + i * log_p + (n - i) * log_q) for i in range(k + 1)))


def cp_upper(failures: int, n: int, confidence: float = 0.95) -> float | None:
    if n == 0:
        return None
    if not 0 <= failures <= n:
        raise ValueError("failures must lie in [0, n]")
    if failures == n:
        return 1.0
    alpha = 1.0 - confidence
    low, high = 0.0, 1.0
    for _ in range(200):
        mid = (low + high) / 2
        if _binom_cdf(failures, n, mid) > alpha:
            low = mid
        else:
            high = mid
    return high


def allowed_failures(n: int, bound: float) -> int | None:
    best = None
    for k in range(n + 1):
        upper = cp_upper(k, n)
        if upper is None or upper > bound:
            break
        best = k
    return best


def min_scenes(failures: int, bound: float) -> int:
    n = failures + 1
    while cp_upper(failures, n) > bound:
        n += 1
    return n


def eval_count(total: int) -> int:
    return math.ceil(0.3 * total - 1e-12)
```

- [ ] **Step 4: 통과 확인** — Expected: 17 passed
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/recognition_stats.py tests/unit/perception/test_recognition_stats.py` · `git commit -m "perception: Clopper-Pearson bounds, allowed failures and scene counts without scipy"`

**검토 B:** 작업 4–6 뒤 Codex — 초점: "인스턴스 렌더가 scene_rig 와 같은 depth 를 내는가, 잡음 모델의 순서·이동이 맞는가, 통계가 설계 표와 같은가". 수렴까지 반복.

### Task 7: 정답 계산 `recognition_truth`

**Files:**
- Create: `tools/recognition_truth.py`
- Test: `tests/unit/test_recognition_truth.py`

**Interfaces:**
- Consumes: `PalletSpec`, `face_layout`, `choose_pocket_pair`, `fork_fit`, `ForkSetup`, `LoadDeclaration`, `danger_threshold_m`, `instance_rig.render_instances`, `instance_rig.solo_pixel_count`, `scene_rig.Box`, `scene_rig.place`
- Produces:
  - `Placement` (frozen): `x_m`, `y_m`, `yaw_rad`, `support_z_m`
  - `placed_boxes(boxes: list[PalletBox], placement: Placement) -> list[scene_rig.Box]`
  - `FaceTruth` (frozen): `axis: str`, `sign: int`(+1 = 기본 면, −1 = 반대 면), `insertion_yaw_rad: float`, `left_m: tuple[float, float, float]`, `right_m: tuple[...]`, `pocket_width_m: tuple[float, float]`(좌, 우), `pocket_height_m: float`, `accessible: bool`
  - `InstanceTruth` (frozen): `instance_id: int`, `spec_name: str`, `known: bool`, `layer: int`, `support_z_m: float`, `faces: tuple[FaceTruth, ...]`, `exposure: float`, `eligible: bool`, `fit: dict`(`FitResult` 의 JSON), `danger_threshold_m: float`, `damage: list[dict]`
  - `face_truths(spec, placement, setup) -> tuple[FaceTruth, ...]`
  - `instance_truth(...)`, 상수 `EXPOSURE_MIN = 0.5`

진입면별 포켓: 면 축 `x` 의 기본 면은 x = −깊이/2(삽입 yaw = 배치 yaw), 반대 면은 yaw + π. 축 `y` 의 기본 면은 y = −폭/2 쪽에서 +y 로 들어가는 면(삽입 yaw = 배치 yaw + π/2), 반대는 + π. 각 면에서 `choose_pocket_pair` 로 쌍을 고르고, 좌 = 그 면의 왼쪽 축에서 큰 쪽. 접근 가능 = 면의 바깥 법선이 카메라 쪽(법선 · (카메라 − 면 중심) > 0).

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/test_recognition_truth.py
import math
from pathlib import Path

import pytest

from forklift_core.perception.fork_fit import TEST_FORK_SPACING_M, ForkSetup
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_model import KNOWN_ENTRY_AXES, spec_from_geometry
from tools import recognition_truth as rt
from tools import scene_rig

ROOT = Path(__file__).resolve().parents[2]


def _spec(name):
    g = load_pallet_geometry(ROOT / f"config/pallet_geometry_{name}.yaml")
    return g, spec_from_geometry(g, entry_axes=KNOWN_ENTRY_AXES[name])


def test_x_face_pockets_match_scene_rig_true_pockets():
    g, spec = _spec("epal6")
    placement = rt.Placement(2.5, 0.2, 0.1, 0.0)
    faces = rt.face_truths(spec, placement, ForkSetup(TEST_FORK_SPACING_M["epal6"]))
    front = next(f for f in faces if f.axis == "x" and f.sign == 1)
    truth = scene_rig.true_pockets(g, x_m=2.5, y_m=0.2, yaw_rad=0.1)
    assert front.left_m == pytest.approx(tuple(truth[0]), abs=1e-9)
    assert front.right_m == pytest.approx(tuple(truth[1]), abs=1e-9)
    assert front.accessible
    back = next(f for f in faces if f.axis == "x" and f.sign == -1)
    assert not back.accessible
    assert back.insertion_yaw_rad == pytest.approx(0.1 + math.pi - 2 * math.pi)


def test_t11_has_four_faces_and_two_face_the_camera_at_45_degrees():
    _, spec = _spec("t11_06")
    faces = rt.face_truths(spec, rt.Placement(2.5, 0.0, math.pi / 4, 0.0), ForkSetup(0.290))
    assert len(faces) == 4
    assert sum(f.accessible for f in faces) == 2


def test_stacked_layer_raises_the_pockets():
    _, spec = _spec("epal6")
    faces = rt.face_truths(spec, rt.Placement(2.5, 0.0, 0.0, 0.144), ForkSetup(0.3725))
    assert faces[0].left_m[2] == pytest.approx(0.144 + 0.022 + 0.078 / 2)


def test_touching_pallets_each_keep_their_own_exposure():
    g, spec = _spec("epal6")
    a = rt.Placement(2.5, 0.4, 0.0, 0.0)
    b = rt.Placement(2.5, -0.4, 0.0, 0.0)
    truths = rt.scene_truths(
        [(spec, a, (), None), (spec, b, (), None)], ForkSetup(0.3725)
    )
    assert all(0.9 < t.exposure <= 1.0 for t in truths)
    assert all(t.eligible for t in truths)
```

- [ ] **Step 2: 실패 확인** — `ImportError`

- [ ] **Step 3: 구현**

```python
# tools/recognition_truth.py
"""Ground truth for recognition scenes: per-face pockets, exposure, fit truth."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import asdict, dataclass

import numpy as np

from forklift_core.perception.fork_fit import (
    ForkSetup,
    LoadDeclaration,
    danger_threshold_m,
    fork_fit,
)
from forklift_core.perception.pallet_damage import DamageOp, apply_damage
from forklift_core.perception.pallet_geometry import PalletBox
from forklift_core.perception.pallet_model import (
    PalletSpec,
    choose_pocket_pair,
    face_layout,
    spec_boxes,
)
from tools import instance_rig, scene_rig

EXPOSURE_MIN = 0.5


def _wrap(angle: float) -> float:
    wrapped = (angle + math.pi) % (2 * math.pi) - math.pi
    return math.pi if wrapped == -math.pi else wrapped


@dataclass(frozen=True)
class Placement:
    x_m: float
    y_m: float
    yaw_rad: float
    support_z_m: float


def placed_boxes(boxes: Sequence[PalletBox], placement: Placement) -> list[scene_rig.Box]:
    lifted = [
        scene_rig.Box((b.centre_m[0], b.centre_m[1], b.centre_m[2] + placement.support_z_m), tuple(b.size_m))
        for b in boxes
    ]
    return scene_rig.place(lifted, x_m=placement.x_m, y_m=placement.y_m, yaw_rad=placement.yaw_rad)


@dataclass(frozen=True)
class FaceTruth:
    axis: str
    sign: int
    insertion_yaw_rad: float
    left_m: tuple[float, float, float]
    right_m: tuple[float, float, float]
    pocket_width_m: tuple[float, float]
    pocket_height_m: float
    accessible: bool


def face_truths(spec: PalletSpec, placement: Placement, setup: ForkSetup) -> tuple[FaceTruth, ...]:
    camera = np.asarray(instance_rig.PERCEPTION_CAMERA.xyz_m, dtype=float)
    out = []
    for axis in spec.entry_axes:
        layout = face_layout(spec, axis)
        i, j = choose_pocket_pair(layout, setup.spacing_m)
        lows = [layout.opening_intervals_m[k] for k in (i, j)]
        centres = [(lo + hi) / 2 for lo, hi in lows]
        widths = [hi - lo for lo, hi in lows]
        base = placement.yaw_rad + (0.0 if axis == "x" else math.pi / 2)
        for sign in (1, -1):
            yaw = _wrap(base + (0.0 if sign == 1 else math.pi))
            a = np.array([math.cos(yaw), math.sin(yaw), 0.0])  # insertion axis
            left_axis = np.array([-a[1], a[0], 0.0])
            face_centre = np.array([placement.x_m, placement.y_m, 0.0]) - a * layout.depth_m / 2
            z = placement.support_z_m + spec.deck_bottom_m + spec.opening_height_m / 2
            # The opposite face sees the layout mirrored, so its left is the
            # nominal right: mirror the lateral coordinates.
            lat = centres if sign == 1 else [-c for c in centres]
            wid = widths if sign == 1 else list(widths)
            pockets = sorted(zip(lat, wid), key=lambda item: item[0])
            (r_lat, r_w), (l_lat, l_w) = pockets
            left = face_centre + left_axis * l_lat
            right = face_centre + left_axis * r_lat
            outward = -a
            accessible = float(outward @ (camera - face_centre)) > 0
            out.append(
                FaceTruth(
                    axis, sign, yaw,
                    (float(left[0]), float(left[1]), z),
                    (float(right[0]), float(right[1]), z),
                    (l_w, r_w), spec.opening_height_m, accessible,
                )
            )
    return tuple(out)


@dataclass(frozen=True)
class InstanceTruth:
    instance_id: int
    spec_name: str
    known: bool
    layer: int
    support_z_m: float
    faces: tuple[FaceTruth, ...]
    exposure: float
    eligible: bool
    fit: dict
    danger_threshold_m: float
    damage: list[dict]

    def to_json(self) -> dict:
        obj = asdict(self)
        obj["faces"] = [asdict(f) for f in self.faces]
        return obj


def instance_boxes(spec: PalletSpec, damage: Sequence[DamageOp]) -> list[PalletBox]:
    return apply_damage(spec_boxes(spec), damage)


def scene_truths(
    instances: Sequence[tuple[PalletSpec, Placement, Sequence[DamageOp], LoadDeclaration | None]],
    setup: ForkSetup,
    *,
    distractors: Sequence[scene_rig.Box] = (),
    layers: Sequence[int] | None = None,
) -> list[InstanceTruth]:
    placed = [placed_boxes(instance_boxes(s, d), p) for s, p, d, _ in instances]
    rendered = instance_rig.render_instances(placed, distractors=distractors)
    out = []
    for k, (spec, placement, damage, load) in enumerate(instances):
        solo = instance_rig.solo_pixel_count(placed[k])
        visible = int((rendered.instance_ids == k).sum())
        exposure = visible / solo if solo else 0.0
        faces = face_truths(spec, placement, setup)
        fit = fork_fit(spec, faces[0].axis, setup, load)
        out.append(
            InstanceTruth(
                instance_id=k,
                spec_name=spec.name,
                known=spec.known,
                layer=0 if layers is None else layers[k],
                support_z_m=placement.support_z_m,
                faces=faces,
                exposure=exposure,
                eligible=exposure >= EXPOSURE_MIN and any(f.accessible for f in faces),
                fit={"pickable": fit.pickable, "reasons": list(fit.reasons),
                     "min_lateral_margin_m": fit.min_lateral_margin_m},
                danger_threshold_m=danger_threshold_m(fit),
                damage=[{"kind": d.kind, "target": d.target, "params": [list(p) for p in d.params]} for d in damage],
            )
        )
    return out
```

> 주: 끼움 정답은 첫 진입 축(`faces[0].axis`, 즉 `"x"`)으로 계산한다. T11 의 y 면은 정사각이라 같은 결과다. 비정사각 형상에 y 진입이 있으면 면별 끼움이 필요하다 — S7 생성기는 비규격에 `("x",)` 만 준다(작업 8).

- [ ] **Step 4: 통과 확인** — Run: `python -m pytest tests/unit/test_recognition_truth.py -q -p no:cacheprovider` · Expected: 4 passed
- [ ] **Step 5: 커밋** — `git add tools/recognition_truth.py tests/unit/test_recognition_truth.py` · `git commit -m "tools: recognition ground truth -- per-face pockets with accessibility, exposure against solo renders, fit truth and danger threshold"`

### Task 8: 장면 조리법 `recognition_scenes`

**Files:**
- Create: `tools/recognition_scenes.py`
- Test: `tests/unit/test_recognition_scenes.py`

**Interfaces:**
- Consumes: 작업 1–3·7, `tools.measure_pocket_evidence.structure`, `scene_rig.boxes_interpenetrate`, `recognition_stats.eval_count`
- Produces:
  - 상수 `CASES = ("S1", "S2", "S3", "S4", "S5", "S6", "S7", "N")`, `SEED_BANDS = {"eval_pool": 1_000_000, "train": 5_000_000}`(대역 폭 1,000,000)
  - `InstanceRecipe` (frozen): `spec: PalletSpec`, `placement: Placement`, `damage: tuple[DamageOp, ...]`, `load: LoadDeclaration | None`, `layer: int`
  - `Instruction` (frozen): `kind: str` ∈ (`"none"`, `"unambiguous"`, `"ambiguous"`), `target: int | None`, `x_m`, `y_m`, `radius_m`, `layer: int | None`, `load: LoadDeclaration | None`
  - `SceneRecipe` (frozen): `scene_id: str`, `case: str`, `shape_key: str`(`"epal6"`·`"t11_06"`·`"nonstandard"`·`"negative"`), `split: str`(`"dev"`·`"eval"`·`"train"`), `seed: int`, `fork_spacing_m: float`, `instances: tuple[InstanceRecipe, ...]`, `distractors: tuple[scene_rig.Box, ...]`, `instruction: Instruction`
  - `generate(case: str, shape_key: str, count: int, *, band: str, first_seed: int) -> list[SceneRecipe]` — 분할: `band == "train"` 이면 전부 `"train"`; 아니면 앞에서부터 `eval_count(count)` 개를 시드 순서 무관하게 고정 순열로 `"eval"`, 나머지 `"dev"`.
  - `assert_disjoint(*recipe_lists)` — 시드가 겹치면 `ValueError`.
  - `catalogue() -> list[SceneRecipe]` — 설계 §3.4 의 평가 장면 전부(S1–S6 × 2 형상 × 61, S7 194, N 94) = 1,020.

사례별 규칙은 설계 §5.1 그대로이고, 지시 배정은 **그 장면이 속한 분할(eval/dev/train) 안에서의 순위 r 로 결정적**이다(무작위나 전체 인덱스로 하면 eval 하한을 보장할 수 없다 — eval 은 순열로 뽑으므로):
- 알려진 형상: 분할 안 순위 r 의 지시 종류 = S2·S3·S4 는 `r % 5 ∈ {0, 1}` → 애매함, `{2, 3}` → 애매하지 않음, `4` → 없음; S1·S6·S7 은 `r % 4 ∈ {0, 1, 2}` → 애매하지 않음, `3` → 없음. eval 19 장면이면 S2·S3·S4 각각 애매함 8·애매하지 않음 7, S1·S6 각각 애매하지 않음 15 → 형상별 eval 애매함 ≥ 24, 애매하지 않음 ≥ 51, 지시 ≥ 75(S5 는 기반 규칙에 따라 더해진다)(애매함은 여러 개가 있어야 성립). S5 는 기반 배치가 S2·S3 이면 S2·S3 규칙.
- 애매한 지시: 대상과 다른 인스턴스가 모두 반경 안에 들어가게 반경 = 두 중심 거리 + 0.1 m.
- 애매하지 않은 지시: 반경 = min(0.3, 다른 인스턴스까지 거리의 절반).

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/test_recognition_scenes.py
import itertools

import pytest

from forklift_core.perception.recognition_stats import eval_count
from tools import recognition_scenes as rs
from tools import scene_rig


def test_catalogue_counts_match_the_design():
    recipes = rs.catalogue()
    assert len(recipes) == 1020
    for shape in ("epal6", "t11_06"):
        mine = [r for r in recipes if r.shape_key == shape]
        assert len(mine) == 6 * 61
        ev = [r for r in mine if r.split == "eval"]
        assert len(ev) == 6 * eval_count(61) == 114
        kinds = [r.instruction.kind for r in ev]
        assert kinds.count("unambiguous") + kinds.count("ambiguous") >= 59
        assert kinds.count("unambiguous") >= 29
        assert kinds.count("ambiguous") >= 15
    assert sum(r.case == "S7" and r.split == "eval" for r in recipes) == 59
    assert sum(r.case == "N" and r.split == "eval" for r in recipes) == 29


def test_generation_is_deterministic():
    a = rs.generate("S2", "epal6", 5, band="eval_pool", first_seed=1_000_000)
    b = rs.generate("S2", "epal6", 5, band="eval_pool", first_seed=1_000_000)
    assert a == b


def test_train_and_eval_bands_are_disjoint_and_checked():
    ev = rs.generate("S1", "epal6", 3, band="eval_pool", first_seed=1_000_000)
    tr = rs.generate("S1", "epal6", 3, band="train", first_seed=5_000_000)
    rs.assert_disjoint(ev, tr)
    with pytest.raises(ValueError, match="overlap"):
        rs.assert_disjoint(ev, rs.generate("S1", "epal6", 3, band="train", first_seed=1_000_000))


def test_first_seed_must_lie_in_its_band():
    with pytest.raises(ValueError, match="band"):
        rs.generate("S1", "epal6", 3, band="train", first_seed=1_000_000)


@pytest.mark.parametrize("case", ["S2", "S3", "S4", "S6"])
def test_objects_do_not_interpenetrate(case):
    for recipe in rs.generate(case, "t11_06", 8, band="eval_pool", first_seed=1_000_000):
        boxes = [rs.recipe_boxes(recipe, k) for k in range(len(recipe.instances))]
        groups = boxes + [list(recipe.distractors)]
        for a, b in itertools.combinations(groups, 2):
            assert not any(scene_rig.boxes_interpenetrate(x, y) for x in a for y in b)


def test_ambiguous_instruction_covers_two_instances():
    recipes = rs.generate("S2", "epal6", 10, band="eval_pool", first_seed=1_000_000)
    amb = [r for r in recipes if r.instruction.kind == "ambiguous"]
    assert amb
    for r in amb:
        inside = [
            k for k, inst in enumerate(r.instances)
            if (inst.placement.x_m - r.instruction.x_m) ** 2 + (inst.placement.y_m - r.instruction.y_m) ** 2
            <= r.instruction.radius_m ** 2
        ]
        assert len(inside) >= 2


def test_s7_mixes_pickable_and_unpickable():
    from tools.recognition_truth import scene_truths
    from forklift_core.perception.fork_fit import ForkSetup

    recipes = rs.generate("S7", "nonstandard", 30, band="eval_pool", first_seed=1_000_000)
    verdicts = set()
    for r in recipes:
        truths = scene_truths(
            [(i.spec, i.placement, i.damage, r.instruction.load) for i in r.instances],
            ForkSetup(r.fork_spacing_m),
        )
        verdicts.add(truths[0].fit["pickable"])
    assert verdicts == {True, False}
```

- [ ] **Step 2: 실패 확인** — `ImportError`

- [ ] **Step 3: 구현**

```python
# tools/recognition_scenes.py
"""Deterministic scene recipes for the recognition generality evaluation (design §5.1)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

from forklift_core.perception.fork_fit import TEST_FORK_SPACING_M, LoadDeclaration
from forklift_core.perception.pallet_damage import DamageOp
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_model import KNOWN_ENTRY_AXES, PalletSpec, spec_from_geometry
from forklift_core.perception.recognition_stats import eval_count
from tools import measure_pocket_evidence, scene_rig
from tools.recognition_truth import Placement, instance_boxes, placed_boxes

ROOT = Path(__file__).resolve().parents[1]
CASES = ("S1", "S2", "S3", "S4", "S5", "S6", "S7", "N")
SEED_BANDS = {"eval_pool": 1_000_000, "train": 5_000_000}
BAND_WIDTH = 1_000_000
MAX_TRIES = 200


@lru_cache(maxsize=None)
def known_spec(shape_key: str) -> PalletSpec:
    g = load_pallet_geometry(ROOT / f"config/pallet_geometry_{shape_key}.yaml")
    return spec_from_geometry(g, entry_axes=KNOWN_ENTRY_AXES[shape_key])


@dataclass(frozen=True)
class InstanceRecipe:
    spec: PalletSpec
    placement: Placement
    damage: tuple[DamageOp, ...]
    layer: int


@dataclass(frozen=True)
class Instruction:
    kind: str
    target: int | None
    x_m: float
    y_m: float
    radius_m: float
    layer: int | None
    load: LoadDeclaration | None


@dataclass(frozen=True)
class SceneRecipe:
    scene_id: str
    case: str
    shape_key: str
    split: str
    seed: int
    fork_spacing_m: float
    instances: tuple[InstanceRecipe, ...]
    distractors: tuple[scene_rig.Box, ...]
    instruction: Instruction


def recipe_boxes(recipe: SceneRecipe, k: int) -> list[scene_rig.Box]:
    inst = recipe.instances[k]
    return placed_boxes(instance_boxes(inst.spec, inst.damage), inst.placement)


def _base_pose(rng):
    """Observation distribution: 1.1-4 m ahead of the camera, +-1 m lateral, +-30 deg."""
    distance = rng.uniform(1.1, 4.0)
    return 0.75 + distance, rng.uniform(-1.0, 1.0), rng.uniform(-math.radians(30), math.radians(30))


def _clear(groups, candidate):
    return not any(
        scene_rig.boxes_interpenetrate(a, b) for g in groups for a in g for b in candidate
    )


def _damage_ops(rng, spec: PalletSpec) -> tuple[DamageOp, ...]:
    kind = rng.choice(["top_board_missing", "top_board_broken", "block_corner_chipped", "block_split"])
    if kind == "top_board_missing":
        picks = rng.choice(spec.top_board_count, size=int(rng.integers(1, 3)), replace=False)
        return tuple(DamageOp(kind, f"top_board_{int(i)}", ()) for i in sorted(picks))
    if kind == "top_board_broken":
        i = int(rng.integers(spec.top_board_count))
        return (DamageOp(kind, f"top_board_{i}", (("fraction", float(rng.uniform(0.2, 0.6))), ("keep", float(rng.integers(2))))),)
    col = int(rng.integers(len(spec.column_widths_m)))
    if kind == "block_corner_chipped":
        return (DamageOp(kind, f"block_x0_y{col}", (("depth", float(rng.uniform(0.01, 0.04))),
                                                     ("width", float(rng.uniform(0.01, 0.04))),
                                                     ("side", float(rng.integers(2))))),)
    return (DamageOp(kind, f"block_x0_y{col}", (("gap", float(rng.uniform(0.005, 0.015))),)),)


def _nonstandard_spec(rng, index: int) -> PalletSpec:
    width = float(rng.uniform(0.60, 1.20))
    depth = float(rng.uniform(0.50, 1.20))
    columns = int(rng.choice([3, 4]))
    widths = tuple(float(rng.uniform(0.06, 0.20)) for _ in range(columns))
    free = width - sum(widths)
    gaps = rng.dirichlet(np.ones(columns - 1)) * free
    edges = [-width / 2]
    centres = []
    for k in range(columns):
        centres.append(edges[-1] + widths[k] / 2)
        if k < columns - 1:
            edges.append(edges[-1] + widths[k] + gaps[k])
    row_depth = float(min(rng.uniform(0.08, 0.15), depth / 4))
    rows = (-(depth - row_depth) / 2, 0.0, (depth - row_depth) / 2)
    flat = rng.random() < 0.2
    board_w = float(rng.uniform(0.08, 0.14))
    count = max(2, min(9, int(width / (board_w * 1.4))))
    return PalletSpec(
        name=f"nonstandard_{index}",
        known=False,
        overall_width_m=width,
        overall_depth_m=depth,
        deck_bottom_m=float(rng.uniform(0.010, 0.025)),
        opening_height_m=float(rng.uniform(0.045, 0.12)),
        stringer_m=float(rng.uniform(0.012, 0.025)),
        top_thickness_m=float(rng.uniform(0.012, 0.025)),
        column_centres_y_m=tuple(centres),
        column_widths_m=widths,
        row_centres_x_m=rows,
        row_depth_m=row_depth,
        top_style="flat" if flat else "boards",
        top_board_count=count,
        top_board_width_m=board_w,
        entry_axes=("x",),
    )


def _load(rng, spec: PalletSpec, declare: bool) -> LoadDeclaration | None:
    if not declare:
        return None
    centred = rng.random() < 0.7
    com_y = 0.0 if centred else float(rng.uniform(0.0, 0.3))
    return LoadDeclaration(
        pallet_mass_kg=float(rng.uniform(1.0, 4.0)),
        load_mass_kg=float(rng.uniform(0.0, 8.0)),
        com_x_max_m=spec.overall_depth_m / 2 + (0.0 if centred else float(rng.uniform(0.0, 0.2))),
        com_y_max_m=com_y,
    )


def _instruction(rng, kind, instances, target, load):
    if kind == "none" or target is None:
        return Instruction("none", None, 0.0, 0.0, 0.0, None, load)
    t = instances[target].placement
    others = [
        math.hypot(i.placement.x_m - t.x_m, i.placement.y_m - t.y_m)
        for k, i in enumerate(instances) if k != target
    ]
    if kind == "ambiguous":
        radius = min(others) + 0.1
    else:
        radius = 0.3 if not others else min(0.3, min(others) / 2)
    return Instruction(kind, target, t.x_m, t.y_m, radius, instances[target].layer, load)


def _instruction_kind(case_for_rule: str, index: int) -> str:
    if case_for_rule in ("S2", "S3", "S4"):
        return {0: "ambiguous", 1: "ambiguous", 2: "unambiguous", 3: "unambiguous", 4: "none"}[index % 5]
    if case_for_rule in ("S1", "S6", "S7"):
        return "none" if index % 4 == 3 else "unambiguous"
    return "none"


def _scene(case, shape_key, seed, index):
    rng = np.random.default_rng(seed)
    spacing = TEST_FORK_SPACING_M.get(shape_key, float(rng.choice([0.3725, 0.290])))
    rule = case
    distractors: list[scene_rig.Box] = []
    if case == "S5":
        rule = ("S1", "S2", "S3")[index % 3]
    for _ in range(MAX_TRIES):
        instances, groups = [], []
        if case == "N":
            g = load_pallet_geometry(ROOT / f"config/pallet_geometry_{('epal6', 't11_06')[index % 2]}.yaml")
            kind = ("grounded", "shelf", "blocks")[index % 3]
            x, y, yaw = _base_pose(rng)
            distractors = scene_rig.place(measure_pocket_evidence.structure(g, kind), x_m=x, y_m=y, yaw_rad=yaw)
            return instances, distractors, spacing, rule
        spec = _nonstandard_spec(rng, index) if case == "S7" else known_spec(shape_key)
        x, y, yaw = _base_pose(rng)
        count = {"S2": int(rng.integers(2, 4)), "S3": int(rng.integers(2, 4)), "S4": int(rng.integers(2, 4))}.get(rule, 1)
        ok = True
        for k in range(count):
            if rule == "S2":
                width = spec.overall_width_m
                gap = float(rng.uniform(0.0, 0.30))
                lateral = (k - (count - 1) / 2) * (width + gap)
                stagger = float(rng.uniform(-0.15, 0.15)) if rng.random() < 0.5 else 0.0
                # Side by side along the pallet's left axis (-sin yaw, cos yaw).
                p = Placement(
                    x + stagger * math.cos(yaw) - lateral * math.sin(yaw),
                    y + stagger * math.sin(yaw) + lateral * math.cos(yaw),
                    yaw + float(rng.uniform(-0.087, 0.087)),
                    0.0,
                )
                layer = 0
            elif rule == "S3":
                behind = sum(float(rng.uniform(0.05, 1.0)) + spec.overall_depth_m for _ in range(k))
                p = Placement(x + behind * math.cos(yaw) + float(rng.uniform(-0.2, 0.2)) * (k > 0),
                              y + behind * math.sin(yaw), yaw, 0.0)
                layer = 0
            elif rule == "S4":
                p = Placement(x + float(rng.uniform(-0.03, 0.03)) * (k > 0), y + float(rng.uniform(-0.03, 0.03)) * (k > 0),
                              yaw + float(rng.uniform(-0.052, 0.052)) * (k > 0), k * spec.overall_height_m)
                layer = k
            else:
                p, layer = Placement(x, y, yaw, 0.0), 0
            damage = _damage_ops(rng, spec) if case == "S5" and k == 0 else ()
            inst = InstanceRecipe(spec, p, damage, layer)
            boxes = placed_boxes(instance_boxes(spec, damage), p)
            if rule != "S4" and not _clear(groups, boxes):
                ok = False
                break
            instances.append(inst)
            groups.append(boxes)
        if not ok:
            continue
        if rule == "S4" and rng.random() < 0.5:
            top = instances[-1].placement
            size = (float(rng.uniform(0.4, 0.6)), float(rng.uniform(0.4, 0.6)), float(rng.uniform(0.2, 0.4)))
            z = top.support_z_m + spec.overall_height_m + size[2] / 2
            distractors = [scene_rig.Box((top.x_m, top.y_m, z), size, top.yaw_rad)]
        if case == "S6":
            distractors = []
            for _ in range(int(rng.integers(2, 6))):
                kind = rng.choice(["rack", "crate", "box", "wall"])
                dx, dy = float(rng.uniform(0.3, 1.5)) * rng.choice([-1, 1]), float(rng.uniform(0.6, 1.6)) * rng.choice([-1, 1])
                if kind == "wall":
                    cand = [scene_rig.Box((x + spec.overall_depth_m / 2 + float(rng.uniform(0.05, 0.6)), y, 0.6), (0.05, 2.0, 1.2), yaw)]
                elif kind == "rack":
                    g = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
                    cand = scene_rig.place(measure_pocket_evidence.structure(g, "shelf"), x_m=x + dx, y_m=y + dy, yaw_rad=yaw)
                else:
                    s = (0.45, 0.66, 0.19) if kind == "crate" else (0.5, 0.5, 0.25)
                    cand = [scene_rig.Box((x + dx, y + dy, s[2] / 2), s, float(rng.uniform(-math.pi, math.pi)))]
                if _clear(groups + [distractors], cand):
                    distractors.extend(cand)
            if rng.random() < 0.3:
                size = (spec.overall_depth_m, spec.overall_width_m, float(rng.uniform(0.2, 0.5)))
                distractors.append(scene_rig.Box((x, y, spec.overall_height_m + size[2] / 2), size, yaw))
        return instances, distractors, spacing, rule
    raise RuntimeError(f"{case} seed {seed}: no non-overlapping placement in {MAX_TRIES} tries")


def generate(case: str, shape_key: str, count: int, *, band: str, first_seed: int) -> list[SceneRecipe]:
    if case not in CASES:
        raise ValueError(f"unknown case {case!r}")
    start = SEED_BANDS[band]
    if not start <= first_seed < start + BAND_WIDTH or first_seed + count > start + BAND_WIDTH:
        raise ValueError(f"seeds {first_seed}+{count} leave the {band} band")
    order = np.random.default_rng(first_seed).permutation(count)
    eval_set = set(order[: eval_count(count)].tolist()) if band != "train" else set()
    ranks, seen = {}, {"eval": 0, "dev": 0, "train": 0}
    for i in range(count):
        split = "train" if band == "train" else ("eval" if i in eval_set else "dev")
        ranks[i] = (split, seen[split])
        seen[split] += 1
    recipes = []
    for i in range(count):
        seed = first_seed + i
        instances, distractors, spacing, rule = _scene(case, shape_key, seed, i)
        split, rank = ranks[i]
        kind = _instruction_kind(rule, rank)
        rng = np.random.default_rng(seed + 7)
        if kind == "ambiguous" and len(instances) < 2:
            kind = "unambiguous"
        target = None if kind == "none" or not instances else int(rng.integers(len(instances)))
        load = _load(rng, instances[target].spec, rng.random() < 0.85) if target is not None and case == "S7" else None
        if case == "S7" and target is None and instances:
            load = _load(rng, instances[0].spec, rng.random() < 0.85)
        instruction = _instruction(rng, kind, instances, target, load)
        recipes.append(
            SceneRecipe(
                scene_id=f"{case.lower()}_{shape_key}_{seed}",
                case=case,
                shape_key=shape_key,
                split=split,
                seed=seed,
                fork_spacing_m=spacing,
                instances=tuple(instances),
                distractors=tuple(distractors),
                instruction=instruction,
            )
        )
    return recipes


def assert_disjoint(*lists) -> None:
    seen: dict[int, str] = {}
    for recipes in lists:
        for r in recipes:
            if r.seed in seen:
                raise ValueError(f"seed {r.seed} overlaps between {seen[r.seed]} and {r.scene_id}")
            seen[r.seed] = r.scene_id


def catalogue() -> list[SceneRecipe]:
    out, seed = [], SEED_BANDS["eval_pool"]
    for case in ("S1", "S2", "S3", "S4", "S5", "S6"):
        for shape in ("epal6", "t11_06"):
            out += generate(case, shape, 61, band="eval_pool", first_seed=seed)
            seed += 61
    out += generate("S7", "nonstandard", 194, band="eval_pool", first_seed=seed)
    seed += 194
    out += generate("N", "negative", 94, band="eval_pool", first_seed=seed)
    assert_disjoint(out)
    return out
```

- [ ] **Step 4: 통과 확인** — Run(ws1, 장면 1,020개 생성이 들어가 몇 분 걸릴 수 있음): `git add tools/recognition_scenes.py tests/unit/test_recognition_scenes.py && FORKLIFT_REPO=/home/light/school/2-2/embedded-design/forklift-wt-perception ~/bin/forklift-ws1-pytest tests/unit/test_recognition_scenes.py -q -p no:cacheprovider -W error` · Expected: 9 passed (`run_in_background` 로 대기)
- [ ] **Step 5: 커밋** — `git commit -m "tools: deterministic recognition scene recipes (S1-S7, negatives) with index-assigned instructions, eval split and disjoint seed bands; catalogue = 1,020 scenes"`

### Task 9: 데이터 계약 판 2 `recognition_dataset` + ADR 0004 D5 개정

**Files:**
- Create: `src/forklift_core/perception/recognition_dataset.py`
- Modify: `docs/decisions/0004-simulation-engine-and-insertion-depth.md` (D5 절 끝에 개정 문단 추가)
- Test: `tests/unit/perception/test_recognition_dataset.py`

**Interfaces:**
- Consumes: `SceneInput`, `intrinsics_from_camera_info`, `decode_depth_mm`, `_read_png`, `_read_json`, `_require_fields`(`scene_dataset.py`), `rotation_matrix_from_quaternion_xyzw`
- Produces:
  - `TF_ORIGINS = ("received_tf_static", "isaac_render_pose", "rig_render_pose")`, `RENDER_SOURCES = ("rig", "isaac")`
  - `RecognitionSample` (frozen): `input: SceneInput`, `origin: str`, `meta: dict`(scene.json), `truth: dict`(truth.json), `instruction: dict`, `instance_ids: NDArray[np.int32]`
  - `write_recognition_scene(scene_dir, scene: SceneInput, instance_ids, *, origin: str, meta: dict, truth: dict, instruction: dict) -> None` — 파일: `rgb.png`, `depth_mm.png`, `depth_meta.json`, `camera_info.json`, `tf.json`, `instances.png`(uint16, 0 = 없음, k + 1 = 인스턴스 k, 65535 = 방해물), `scene.json`(`schema_version: 2`), `truth.json`, `instruction.json`
  - `load_recognition_input(scene_dir) -> SceneInput` — **truth·instruction·instances 를 읽지 않는다**
  - `load_recognition_sample(scene_dir) -> RecognitionSample`

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_recognition_dataset.py
import json

import numpy as np
import pytest

from forklift_core.perception import recognition_dataset as rd
from tools import instance_rig, scene_rig


def _write(tmp_path, origin="rig_render_pose"):
    out = instance_rig.render_instances([[scene_rig.Box((2.0, 0.0, 0.15), (0.3, 0.6, 0.3))]])
    rd.write_recognition_scene(
        tmp_path, out.scene, out.instance_ids, origin=origin,
        meta={"scene_id": "s", "case": "S1", "shape_key": "epal6", "split": "dev", "render_source": "rig"},
        truth={"instances": []}, instruction={"kind": "none"},
    )
    return out


def test_round_trip_keeps_depth_to_a_millimetre_and_ids(tmp_path):
    out = _write(tmp_path)
    sample = rd.load_recognition_sample(tmp_path)
    finite = np.isfinite(out.scene.depth_m)
    assert np.nanmax(np.abs(sample.input.depth_m[finite] - out.scene.depth_m[finite])) <= 0.0005 + 1e-12
    assert np.array_equal(sample.instance_ids, out.instance_ids)
    assert sample.origin == "rig_render_pose"
    assert json.loads((tmp_path / "scene.json").read_text())["schema_version"] == 2


def test_input_loader_does_not_touch_truth(tmp_path):
    _write(tmp_path)
    (tmp_path / "truth.json").unlink()
    (tmp_path / "instances.png").unlink()
    (tmp_path / "instruction.json").unlink()
    rd.load_recognition_input(tmp_path)  # must not raise


def test_unknown_origin_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="origin"):
        _write(tmp_path, origin="computed")
```

- [ ] **Step 2: 실패 확인** — `ImportError`

- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/recognition_dataset.py
"""Scene dataset schema version 2: multi-instance truth beside the v1 input files.

The input files (rgb, depth, camera_info, tf) keep the v1 encodings. tf.json may
name one of TF_ORIGINS; a rendered pose is named as rendered, never as a
received /tf_static (ADR 0004 D5 amendment, 2026-10-15). Truth, instruction and
the instance image are evaluator-only and are never read by the input loader.
"""

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.scene_dataset import (
    SceneInput,
    _read_json,
    _read_png,
    _require_fields,
    decode_depth_mm,
    intrinsics_from_camera_info,
)

SCHEMA_VERSION = 2
TF_ORIGINS = ("received_tf_static", "isaac_render_pose", "rig_render_pose")
RENDER_SOURCES = ("rig", "isaac")
_DISTRACTOR_CODE = 65535


@dataclass(frozen=True)
class RecognitionSample:
    input: SceneInput
    origin: str
    meta: dict
    truth: dict
    instruction: dict
    instance_ids: NDArray[np.int32]


def _quaternion_xyzw(rotation):
    m = np.asarray(rotation, dtype=float)
    w = np.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    return [float(x), float(y), float(z), float(w)]


def _write_png(path: Path, array, mode: str) -> None:
    from PIL import Image

    Image.fromarray(array, mode=mode).save(path, format="PNG")


def write_recognition_scene(scene_dir, scene: SceneInput, instance_ids, *, origin, meta, truth, instruction) -> None:
    if origin not in TF_ORIGINS:
        raise ValueError(f"tf origin must be one of {TF_ORIGINS}")
    if meta.get("render_source") not in RENDER_SOURCES:
        raise ValueError(f"meta.render_source must be one of {RENDER_SOURCES}")
    scene_dir = Path(scene_dir)
    scene_dir.mkdir(parents=True, exist_ok=True)
    depth = scene.depth_m
    ok = np.isfinite(depth) & (depth > 0)
    if np.any(depth[ok] > 65.535):
        raise ValueError("Depth exceeds the uint16 millimetre range")
    raw = np.zeros(depth.shape, dtype=np.uint16)
    raw[ok] = np.rint(depth[ok] * 1000).astype(np.uint16)
    ids = np.asarray(instance_ids)
    codes = np.where(ids >= 0, ids + 1, np.where(ids == -2, _DISTRACTOR_CODE, 0)).astype(np.uint16)
    i = scene.intrinsics
    k = [i.fx, 0.0, i.cx, 0.0, i.fy, i.cy, 0.0, 0.0, 1.0]
    files = {
        "depth_meta.json": {"unit": "mm", "meters_per_unit": 0.001, "unknown_value": 0, "kind": "optical_axis_z"},
        "camera_info.json": {
            "frame_id": i.frame_id, "stamp_ns": scene.stamp_ns, "width": i.width, "height": i.height,
            "distortion_model": "plumb_bob", "d": [0.0] * 5, "k": k, "r": np.eye(3).ravel().tolist(),
            "p": np.column_stack((np.array(k).reshape(3, 3), np.zeros(3))).ravel().tolist(),
            "binning_x": 0, "binning_y": 0,
            "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False},
        },
        "tf.json": {
            "target_frame": scene.base_from_optical.target_frame,
            "source_frame": scene.base_from_optical.source_frame,
            "translation_m": [float(v) for v in scene.base_from_optical.translation_m],
            "quaternion_xyzw": _quaternion_xyzw(scene.base_from_optical.rotation),
            "origin": origin,
        },
        "scene.json": {
            **meta, "schema_version": SCHEMA_VERSION, "stamp_ns": scene.stamp_ns,
            "clock_domain": scene.clock_domain, "source_provenance": scene.source_provenance,
        },
        "truth.json": truth,
        "instruction.json": instruction,
    }
    for name, obj in files.items():
        (scene_dir / name).write_text(json.dumps(obj, indent=1, sort_keys=True), encoding="utf-8")
    _write_png(scene_dir / "rgb.png", np.asarray(scene.rgb, dtype=np.uint8), "RGB")
    _write_png(scene_dir / "depth_mm.png", raw, "I;16")
    _write_png(scene_dir / "instances.png", codes, "I;16")


def _tf(obj: dict) -> tuple[RigidTransform, str]:
    _require_fields(obj, {"target_frame", "source_frame", "translation_m", "quaternion_xyzw", "origin"}, "tf")
    if obj["origin"] not in TF_ORIGINS:
        raise ValueError(f"tf origin must be one of {TF_ORIGINS}")
    if obj["target_frame"] != "base_link" or obj["source_frame"] != "camera_optical_frame":
        raise ValueError("TF must map camera_optical_frame to base_link")
    return RigidTransform(
        source_frame=obj["source_frame"], target_frame=obj["target_frame"],
        rotation=rotation_matrix_from_quaternion_xyzw(obj["quaternion_xyzw"]),
        translation_m=obj["translation_m"],
    ), obj["origin"]


def _load(scene_dir: Path):
    meta = _read_json(scene_dir / "scene.json")
    if meta.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("scene.json schema_version must be 2")
    intrinsics = intrinsics_from_camera_info(_read_json(scene_dir / "camera_info.json"))
    transform, origin = _tf(_read_json(scene_dir / "tf.json"))
    rgb = _read_png(scene_dir / "rgb.png", depth=False)
    depth = decode_depth_mm(_read_png(scene_dir / "depth_mm.png", depth=True), _read_json(scene_dir / "depth_meta.json"))
    scene = SceneInput(
        rgb=rgb, depth_m=depth, intrinsics=intrinsics, base_from_optical=transform,
        stamp_ns=int(meta["stamp_ns"]), clock_domain=meta["clock_domain"],
        source_provenance=meta["source_provenance"],
    )
    return scene, origin, meta


def load_recognition_input(scene_dir) -> SceneInput:
    scene, _, _ = _load(Path(scene_dir))
    return scene


def load_recognition_sample(scene_dir) -> RecognitionSample:
    scene_dir = Path(scene_dir)
    scene, origin, meta = _load(scene_dir)
    codes = _read_png(scene_dir / "instances.png", depth=True).astype(np.int64)
    ids = np.where(codes == _DISTRACTOR_CODE, -2, codes - 1).astype(np.int32)
    return RecognitionSample(
        scene, origin, meta, _read_json(scene_dir / "truth.json"),
        _read_json(scene_dir / "instruction.json"), ids,
    )
```

- [ ] **Step 4: ADR 0004 D5 개정 문단 추가** — D5 절(`### D5 —` 로 시작하는 절)의 마지막 문단 뒤에 다음을 넣는다:

```markdown
**개정 (2026-10-15, 사용자 결정 — 팔레트 인식 일반화 설계 §13).** 데이터 계약을 **명시적으로 확장**한다: 스키마 판 2(`forklift_core.perception.recognition_dataset`)의 `tf.json` `origin` 은 `received_tf_static` · `isaac_render_pose` · `rig_render_pose` 중 하나다. 렌더 자세에서 계산한 변환은 그 출처 이름으로 적고, `received_tf_static` 이라는 이름을 붙이지 않는다(위 금지는 그대로). `isaac_render_pose` 장면은 Isaac 모집단으로만, `rig_render_pose` 장면은 CPU 기구 회귀로만 쓰며 렌더 기반 판정 분모에 넣지 않는다. 판 1 로더(`scene_dataset.py`)는 바꾸지 않는다.
```

- [ ] **Step 5: 통과 확인** — Expected: 3 passed
- [ ] **Step 6: 커밋** — `git add src/forklift_core/perception/recognition_dataset.py tests/unit/perception/test_recognition_dataset.py docs/decisions/0004-simulation-engine-and-insertion-depth.md` · `git commit -m "perception: scene dataset schema v2 (multi-instance truth, instance image, explicit rendered-pose TF origins); ADR 0004 D5 amendment"`

**검토 C:** 작업 7–9 뒤 Codex — 초점: "정답(진입면·좌우·접근 가능·노출·끼움)이 설계와 코드 규약(`true_pockets`, `PocketObservation` 의 왼쪽 축)에 맞는가, 조리법이 설계 §5.1 범위·지시 하한을 보장하는가, 판 2 가 입력과 정답을 분리하는가". 수렴까지 반복.

### Task 10: 다중 인스턴스 평가기 `evaluation_multi`

**Files:**
- Create: `src/forklift_core/perception/evaluation_multi.py`
- Test: `tests/unit/perception/test_evaluation_multi.py`

**Interfaces:**
- Consumes: `recognition_stats`, `fork_fit.TIP_LEVER_M`, `pocket_observation.yaw_difference_rad`, `RecognitionSample`(truth/instruction dict 형식은 작업 7·8·9)
- Produces:
  - `CandidateView` (frozen): `candidate_id: str`, `insertion_yaw_rad: float`, `left_m: tuple[float, float, float]`, `right_m: tuple[...]`, `fitted: bool`, `identified: bool`, `pickable: bool | None`, `reasons: tuple[str, ...]`, `layer_abs: int | None`
  - `TargetDecision` (frozen): `status: str` ∈ (`"selected"`, `"ask_operator"`, `"reobserve"`, `"none"`, `"not_pickable"`), `candidate_id: str | None`, `reasons: tuple[str, ...]`
  - `SceneVerdict` (frozen): `scene_id`, `case`, `shape_key`, `split`, `population`(`"known:epal6"`·`"known:t11_06"`·`"nonstandard"`·`"negative"`), `miss: bool | None`, `dangerous: bool`, `extra: bool`, `negative_fp: bool | None`, `target_error: bool | None`, `asked_unambiguous: bool | None`, `fit_error: bool | None`, `errors: tuple[tuple[float, float, float, float], ...]`(가로, 삽입축, yaw, z 의 절댓값), `elapsed_s: float | None`
  - `judge_scene(meta: dict, truth: dict, instruction: dict, candidates: Sequence[CandidateView], decision: TargetDecision, *, elapsed_s=None) -> SceneVerdict`
  - `summarize(verdicts: Sequence[SceneVerdict], *, split: str = "eval") -> dict` — 모집단별 실패 수·n·상한·합격, 오차 p95, 칸별 기록. 상수 `GATES`.

대응 규칙: 식별된 적합 후보(`fitted and identified`)와 정답 인스턴스의 **모든 진입면**(접근 가능 여부와 무관 — 카메라를 등진 면과 대응되면 그 면 오차로 계산되어 큰 yaw 오차가 남는다) 사이 비용 = 그 면과의 max(좌, 우 포켓 3차원 거리), yaw 차 > 0.35 rad 또는 거리 > 0.20 m 이면 대응 불가. 일대일 최소 비용 대응(후보 ≤ 8, 인스턴스 ≤ 4 라 전수 탐색). 오차 분해: 정답 면의 삽입축 a, 왼쪽 축 n 에 대해 두 포켓 차이의 평균 벡터 d → 가로 |d·n|, 삽입축 |d·a|, 높이 |d_z|, yaw |Δψ|.

판정(설계 §3.4):
- `miss`(양성 모집단): `eligible` 인스턴스 중 대응되지 않은 것이 있으면 참.
- `dangerous`: `pickable is True` 인 식별된 적합 후보가 (a) 대응된 인스턴스에 대해 가로 + 0.33·tan(yaw) > `danger_threshold_m` 이거나 삽입축 > 0.040 m, 또는 (b) 대응된 인스턴스의 정답 `fit.pickable` 이 거짓이면 참.
- `extra`: 대응되지 않은 식별된 적합 후보가 있으면 참(중복 포함).
- `negative_fp`(음성): 식별된 적합 후보가 하나라도 있으면 참.
- `target_error`(지시 장면): 애매한 지시인데 `selected`; 애매하지 않은 지시인데 선택된 후보가 대상 인스턴스와 대응되지 않음(층 포함); 대상이 `eligible` 인데 `none`/`not_pickable`(정답 끼움 가능 시).
- `asked_unambiguous`(애매하지 않은 지시 장면): 결정이 `ask_operator`.
- `fit_error`(S7): 대상(지시가 없으면 인스턴스 0)이 대응된 후보의 `pickable` 이 정답 `fit.pickable` 과 다르면 참; 후보에 `pickable` 이 없으면 None.

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_evaluation_multi.py
import math

import pytest

from forklift_core.perception.evaluation_multi import (
    CandidateView,
    TargetDecision,
    judge_scene,
    summarize,
)


def _face(left, right, yaw=0.0, accessible=True, axis="x", sign=1):
    return {"axis": axis, "sign": sign, "insertion_yaw_rad": yaw, "left_m": left, "right_m": right,
            "pocket_width_m": [0.2, 0.2], "pocket_height_m": 0.078, "accessible": accessible}


def _inst(k, y, *, eligible=True, pickable=True, thr=0.0437):
    return {"instance_id": k, "spec_name": "epal6", "known": True, "layer": 0, "support_z_m": 0.0,
            "faces": [_face([2.0, y + 0.186, 0.061], [2.0, y - 0.186, 0.061]),
                      _face([2.6, y - 0.186, 0.061], [2.6, y + 0.186, 0.061], yaw=math.pi, accessible=False, sign=-1)],
            "exposure": 1.0, "eligible": eligible, "fit": {"pickable": pickable, "reasons": []},
            "danger_threshold_m": thr, "damage": []}


META = {"scene_id": "s", "case": "S2", "shape_key": "epal6", "split": "eval"}


def _cand(cid, y, dy=0.0, yaw=0.0, pickable=True):
    return CandidateView(cid, yaw, (2.0, y + 0.186 + dy, 0.061), (2.0, y - 0.186 + dy, 0.061),
                         True, True, pickable, (), 0)


NONE = {"kind": "none", "target": None}


def test_perfect_two_pallets_pass():
    truth = {"instances": [_inst(0, 0.5), _inst(1, -0.5)]}
    v = judge_scene(META, truth, NONE, [_cand("a", 0.5), _cand("b", -0.5)], TargetDecision("selected", "a", ()))
    assert (v.miss, v.dangerous, v.extra) == (False, False, False)


def test_duplicate_candidate_is_extra():
    truth = {"instances": [_inst(0, 0.5)]}
    v = judge_scene(META, truth, NONE, [_cand("a", 0.5), _cand("b", 0.5, dy=0.01)], TargetDecision("none", None, ()))
    assert v.extra


def test_lateral_plus_yaw_projection_is_dangerous():
    truth = {"instances": [_inst(0, 0.0, thr=0.0437)]}
    # 35 mm lateral + 0.33 tan(3.9 deg) = 57.5 mm > 43.7 mm.
    c = _cand("a", 0.0, dy=0.035, yaw=math.radians(3.9))
    v = judge_scene(META, truth, NONE, [c], TargetDecision("selected", "a", ()))
    assert v.dangerous


def test_correct_stop_on_a_nonfitting_pallet_is_not_dangerous():
    truth = {"instances": [_inst(0, 0.0, pickable=False, thr=-0.0475)]}
    c = _cand("a", 0.0, pickable=False)
    v = judge_scene({**META, "case": "S7", "shape_key": "nonstandard"}, truth, NONE, [c],
                    TargetDecision("not_pickable", "a", ("fork_spacing_adjust",)))
    assert not v.dangerous and v.fit_error is False


def test_allowing_a_pick_on_a_nonfitting_pallet_is_dangerous():
    truth = {"instances": [_inst(0, 0.0, pickable=False, thr=-0.0475)]}
    v = judge_scene({**META, "case": "S7", "shape_key": "nonstandard"}, truth, NONE, [_cand("a", 0.0)],
                    TargetDecision("selected", "a", ()))
    assert v.dangerous and v.fit_error


def test_back_face_match_leaves_a_large_yaw_error():
    truth = {"instances": [_inst(0, 0.0)]}
    c = CandidateView("a", math.pi, (2.6, -0.186, 0.061), (2.6, 0.186, 0.061), True, True, True, (), 0)
    v = judge_scene(META, truth, NONE, [c], TargetDecision("none", None, ()))
    assert not v.miss  # matched to the back face geometry
    assert v.errors[0][2] == pytest.approx(0.0, abs=1e-9)  # yaw error against that face


def test_ambiguous_instruction_must_not_auto_select():
    truth = {"instances": [_inst(0, 0.2), _inst(1, -0.2)]}
    inst = {"kind": "ambiguous", "target": 0, "layer": 0}
    v = judge_scene(META, truth, inst, [_cand("a", 0.2), _cand("b", -0.2)], TargetDecision("selected", "a", ()))
    assert v.target_error
    v = judge_scene(META, truth, inst, [_cand("a", 0.2), _cand("b", -0.2)], TargetDecision("ask_operator", None, ()))
    assert v.target_error is False


def test_asking_on_an_unambiguous_instruction_is_counted():
    truth = {"instances": [_inst(0, 0.5), _inst(1, -0.5)]}
    inst = {"kind": "unambiguous", "target": 1, "layer": 0}
    v = judge_scene(META, truth, inst, [_cand("a", 0.5), _cand("b", -0.5)], TargetDecision("ask_operator", None, ()))
    assert v.asked_unambiguous and v.target_error is False


def test_empty_population_never_passes():
    summary = summarize([])
    assert summary["gates"] and all(not gate["pass"] for gate in summary["gates"].values())


def test_known_populations_have_no_fit_gate():
    assert not any(k.startswith("known") and k.endswith(":fit_error") for k in summarize([])["gates"])
```

- [ ] **Step 2: 실패 확인** — `ImportError`

- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/evaluation_multi.py
"""Scene-level judgment for multi-instance recognition (design §3.3-3.4).

Units are scenes: any failure inside a scene fails that scene. Gates are judged
on pooled populations with one-sided 95 % Clopper-Pearson bounds; per-cell
records are reported only. An empty population has no bound and never passes.
"""

import itertools
import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from forklift_core.perception.fork_fit import TIP_LEVER_M
from forklift_core.perception.pocket_observation import yaw_difference_rad
from forklift_core.perception.recognition_stats import allowed_failures, cp_upper

MATCH_DISTANCE_M = 0.20
MATCH_YAW_RAD = 0.35
AXIAL_DANGER_M = 0.040
GATES = {
    "miss": 0.05,
    "dangerous": 0.05,
    "extra": 0.10,
    "negative_fp": 0.10,
    "target_error": 0.05,
    "asked_unambiguous": 0.10,
    "fit_error": 0.05,
}
POSE_P95 = {"lateral": 0.020, "axial": 0.020, "yaw": math.radians(2.0), "z": 0.005}
APPLICABLE = {
    "known:epal6": ("miss", "dangerous", "extra", "target_error", "asked_unambiguous"),
    "known:t11_06": ("miss", "dangerous", "extra", "target_error", "asked_unambiguous"),
    "nonstandard": ("miss", "dangerous", "extra", "target_error", "asked_unambiguous", "fit_error"),
    "negative": ("negative_fp",),
}


@dataclass(frozen=True)
class CandidateView:
    candidate_id: str
    insertion_yaw_rad: float
    left_m: tuple[float, float, float]
    right_m: tuple[float, float, float]
    fitted: bool
    identified: bool
    pickable: bool | None
    reasons: tuple[str, ...]
    layer_abs: int | None

    @property
    def counted(self) -> bool:
        return self.fitted and self.identified


@dataclass(frozen=True)
class TargetDecision:
    status: str
    candidate_id: str | None
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class SceneVerdict:
    scene_id: str
    case: str
    shape_key: str
    split: str
    population: str
    miss: bool | None
    dangerous: bool
    extra: bool
    negative_fp: bool | None
    target_error: bool | None
    asked_unambiguous: bool | None
    fit_error: bool | None
    errors: tuple[tuple[float, float, float, float], ...]
    elapsed_s: float | None


def population_of(meta: dict) -> str:
    if meta["case"] == "N":
        return "negative"
    if meta["case"] == "S7":
        return "nonstandard"
    return f"known:{meta['shape_key']}"


def _face_cost(c: CandidateView, face: dict):
    dyaw = abs(yaw_difference_rad(c.insertion_yaw_rad, face["insertion_yaw_rad"]))
    dl = np.subtract(c.left_m, face["left_m"])
    dr = np.subtract(c.right_m, face["right_m"])
    dist = max(float(np.linalg.norm(dl)), float(np.linalg.norm(dr)))
    if dyaw > MATCH_YAW_RAD or dist > MATCH_DISTANCE_M:
        return None
    a = np.array([math.cos(face["insertion_yaw_rad"]), math.sin(face["insertion_yaw_rad"]), 0.0])
    n = np.array([-a[1], a[0], 0.0])
    d = (dl + dr) / 2
    return dist, (abs(float(d @ n)), abs(float(d @ a)), dyaw, abs(float(d[2])))


def _match(cands: Sequence[CandidateView], instances: Sequence[dict]):
    """Exhaustive one-to-one minimum-cost matching (sizes are tiny)."""
    costs = {}
    for ci, c in enumerate(cands):
        for ii, inst in enumerate(instances):
            best = None
            for face in inst["faces"]:
                fc = _face_cost(c, face)
                if fc and (best is None or fc[0] < best[0]):
                    best = fc
            if best:
                costs[ci, ii] = best
    best_pairs, best_total = {}, None
    ci_all = list(range(len(cands)))
    for size in range(min(len(cands), len(instances)), -1, -1):
        for chosen in itertools.combinations(ci_all, size):
            for perm in itertools.permutations(range(len(instances)), size):
                if all((c, i) in costs for c, i in zip(chosen, perm)):
                    total = sum(costs[c, i][0] for c, i in zip(chosen, perm))
                    if best_total is None or total < best_total:
                        best_total, best_pairs = total, dict(zip(chosen, perm))
        if best_total is not None:
            break
    return best_pairs, costs


def judge_scene(meta, truth, instruction, candidates, decision, *, elapsed_s=None) -> SceneVerdict:
    population = population_of(meta)
    instances = truth.get("instances", [])
    counted = [c for c in candidates if c.counted]
    pairs, costs = _match(counted, instances)
    matched_inst = {i: c for c, i in pairs.items()}
    errors = tuple(costs[c, i][1] for c, i in pairs.items())
    extra = len(pairs) < len(counted)
    dangerous = False
    for c, i in pairs.items():
        cand, inst = counted[c], instances[i]
        if cand.pickable is not True:
            continue
        lat, axial, dyaw, _ = costs[c, i][1]
        if (
            not inst["fit"]["pickable"]
            or lat + TIP_LEVER_M * math.tan(dyaw) > inst["danger_threshold_m"]
            or axial > AXIAL_DANGER_M
        ):
            dangerous = True
    negative = population == "negative"
    miss = None if negative else any(inst["eligible"] and k not in matched_inst for k, inst in enumerate(instances))
    negative_fp = bool(counted) if negative else None
    target_error = asked = None
    kind = instruction.get("kind", "none")
    by_id = {c.candidate_id: k for k, c in enumerate(counted)}
    if kind in ("ambiguous", "unambiguous") and not negative:
        target = instruction["target"]
        if kind == "ambiguous":
            target_error = decision.status == "selected"
        else:
            asked = decision.status == "ask_operator"
            tgt = instances[target]
            if decision.status == "selected":
                k = by_id.get(decision.candidate_id)
                target_error = k is None or pairs.get(k) != target or (
                    counted[k].layer_abs is not None and counted[k].layer_abs != instruction.get("layer")
                )
            elif decision.status in ("none", "not_pickable"):
                target_error = bool(tgt["eligible"] and tgt["fit"]["pickable"])
            else:
                target_error = False
    fit_error = None
    if population == "nonstandard" and instances:
        target = instruction.get("target")
        k_inst = 0 if target is None else target
        if k_inst in matched_inst and counted[matched_inst[k_inst]].pickable is not None:
            fit_error = counted[matched_inst[k_inst]].pickable != instances[k_inst]["fit"]["pickable"]
    return SceneVerdict(
        meta["scene_id"], meta["case"], meta["shape_key"], meta["split"], population,
        miss, dangerous, extra, negative_fp, target_error, asked, fit_error, errors, elapsed_s,
    )


def _gate(values, bound):
    values = [v for v in values if v is not None]
    n, k = len(values), sum(bool(v) for v in values)
    upper = cp_upper(k, n)
    allowed = allowed_failures(n, bound) if n else None
    return {"n": n, "failures": k, "upper": upper, "bound": bound,
            "allowed_failures": allowed, "pass": upper is not None and upper <= bound}


def summarize(verdicts: Sequence[SceneVerdict], *, split: str = "eval") -> dict:
    chosen = [v for v in verdicts if v.split == split]
    populations = ("known:epal6", "known:t11_06", "nonstandard", "negative")
    gates, pose, cells = {}, {}, {}
    for pop in populations:
        mine = [v for v in chosen if v.population == pop]
        for name in APPLICABLE[pop]:
            # An applicable gate with no scenes reports n = 0 and fails.
            gates[f"{pop}:{name}"] = _gate([getattr(v, name) for v in mine], GATES[name])
        if pop.startswith("known") or pop == "nonstandard":
            gates[f"{pop}:dangerous_observed_zero"] = {
                "n": len(mine), "failures": sum(v.dangerous for v in mine), "bound": 0.0,
                "pass": bool(mine) and not any(v.dangerous for v in mine),
            }
            errs = np.array([e for v in mine for e in v.errors]) if any(v.errors for v in mine) else np.zeros((0, 4))
            pose[pop] = {
                axis: (None if not len(errs) else float(np.percentile(errs[:, j], 95)))
                for j, axis in enumerate(("lateral", "axial", "yaw", "z"))
            }
        for case in sorted({v.case for v in mine}):
            cell = [v for v in mine if v.case == case]
            cells[f"{pop}:{case}"] = {name: _gate([getattr(v, name) for v in cell], GATES[name]) for name in APPLICABLE[pop]}
    return {"split": split, "scenes": len(chosen), "gates": gates, "pose_p95": pose, "cells": cells,
            "pose_targets": POSE_P95}
```

- [ ] **Step 4: 통과 확인** — Expected: 10 passed
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/evaluation_multi.py tests/unit/perception/test_evaluation_multi.py` · `git commit -m "perception: multi-instance scene judgment (miss, dangerous with blade-tip projection, extra, target error, ask rate, fit error) and pooled Clopper-Pearson gates"`

### Task 11: 생성·평가 CLI

**Files:**
- Create: `tools/generate_recognition_scenes.py`, `tools/evaluate_recognition.py`
- Test: `tests/integration/test_recognition_cli.py`

**Interfaces:**
- Consumes: 작업 4–10, `detect_pockets`, `DetectorParams.derived_for`, `load_pallet_prior`
- Produces:
  - `generate_recognition_scenes.py --out DIR [--cases S1,...] [--limit N] [--noise default|none]` — `catalogue()` 의 조리법을 CPU 렌더 → 정답 → (잡음) → 판 2 로 쓴다. `origin="rig_render_pose"`, `render_source="rig"`. 각 장면 `scene.json` 에 `noise` 모델 JSON 과 `recipe`(시드·사례·지시 종류)를 남긴다.
  - `evaluate_recognition.py --dataset DIR --split dev|eval --detector baseline|oracle --output DIR` — `baseline` 은 지금 검출기(`detect_pockets`, 형상별 prior 와 유도 파라미터; 비규격·음성 장면은 EPAL 6 prior)를 후보 하나로 감싼다(`identified=True`, `fitted = status == "valid"`, `pickable = fitted`, 결정 = `selected` 또는 `none`). `oracle` 은 정답을 그대로 후보로 낸다(평가기 자기 점검 — 모든 관문 통과가 기대값). 출력: `metrics.json`(`summarize` + 데이터 해시·git 리비전·검출기·인자), `verdicts.csv`.

- [ ] **Step 1: 실패하는 통합 시험 작성**

```python
# tests/integration/test_recognition_cli.py
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _run(*args):
    subprocess.run([sys.executable, *args], cwd=ROOT, check=True)


def test_oracle_passes_its_own_scenes_and_baseline_runs(tmp_path):
    data = tmp_path / "data"
    _run("tools/generate_recognition_scenes.py", "--out", str(data), "--cases", "S1,S2", "--limit", "4", "--noise", "none")
    assert len(list(data.glob("*/scene.json"))) == 4 * 2 * 2
    oracle = tmp_path / "oracle"
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "oracle", "--output", str(oracle))
    metrics = json.loads((oracle / "metrics.json").read_text())
    for name, gate in metrics["gates"].items():
        if gate["n"] and name.endswith((":miss", ":extra", ":dangerous_observed_zero")):
            assert gate["failures"] == 0, name
    base = tmp_path / "base"
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "baseline", "--output", str(base))
    assert (base / "verdicts.csv").exists()
```

- [ ] **Step 2: 실패 확인** — 스크립트 없음
- [ ] **Step 3: 구현** — `generate_recognition_scenes.py`:

```python
#!/usr/bin/env python3
"""Render the recognition catalogue on the CPU rig into schema-v2 scene dirs."""

import argparse
import sys
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from forklift_core.perception.fork_fit import ForkSetup
from forklift_core.perception.recognition_dataset import write_recognition_scene
from tools import instance_rig, recognition_scenes as rs
from tools.recognition_truth import scene_truths


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--cases", default=",".join(rs.CASES))
    p.add_argument("--limit", type=int, default=None, help="per case and shape, first N recipes")
    p.add_argument("--noise", choices=("default", "none"), default="default")
    a = p.parse_args(argv)
    cases = set(a.cases.split(","))
    counts = {}
    for recipe in rs.catalogue():
        if recipe.case not in cases:
            continue
        key = (recipe.case, recipe.shape_key)
        counts[key] = counts.get(key, 0) + 1
        if a.limit is not None and counts[key] > a.limit:
            continue
        insts = [(i.spec, i.placement, i.damage, recipe.instruction.load) for i in recipe.instances]
        truths = scene_truths(insts, ForkSetup(recipe.fork_spacing_m), distractors=recipe.distractors,
                              layers=[i.layer for i in recipe.instances])
        placed = [rs.recipe_boxes(recipe, k) for k in range(len(recipe.instances))]
        rendered = instance_rig.render_instances(placed, distractors=recipe.distractors)
        scene, ids = rendered.scene, rendered.instance_ids
        noise = None
        if a.noise == "default":
            model = NoiseModel(seed=recipe.seed)
            scene, ids = apply_noise(scene, ids, model)
            noise = model.to_json()
        instr = recipe.instruction
        write_recognition_scene(
            a.out / recipe.scene_id, scene, ids, origin="rig_render_pose",
            meta={"scene_id": recipe.scene_id, "case": recipe.case, "shape_key": recipe.shape_key,
                  "split": recipe.split, "seed": recipe.seed, "render_source": "rig", "noise": noise,
                  "fork_spacing_m": recipe.fork_spacing_m},
            truth={"instances": [t.to_json() for t in truths]},
            instruction={"kind": instr.kind, "target": instr.target, "x_m": instr.x_m, "y_m": instr.y_m,
                         "radius_m": instr.radius_m, "layer": instr.layer,
                         "load": None if instr.load is None else asdict(instr.load)},
        )


if __name__ == "__main__":
    main()
```

`evaluate_recognition.py`:

```python
#!/usr/bin/env python3
"""Judge a schema-v2 dataset with the current detector (baseline) or the truth (oracle)."""

import argparse
import csv
import hashlib
import json
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from forklift_core.perception.evaluation_multi import CandidateView, TargetDecision, judge_scene, summarize
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.recognition_dataset import load_recognition_sample

ROOT = Path(__file__).resolve().parents[1]
PRIORS = {"epal6": "config/pallet_prior_epal6.yaml", "t11_06": "config/pallet_prior_t11_06.yaml"}


def _baseline(sample):
    shape = sample.meta["shape_key"] if sample.meta["shape_key"] in PRIORS else "epal6"
    prior = load_pallet_prior(ROOT / PRIORS[shape])
    params = DetectorParams.derived_for(prior)
    obs = detect_pockets(sample.input, prior, params).observation
    if obs.status != "valid":
        return [], TargetDecision("none", None, (obs.reason or "",))
    c = CandidateView("baseline", obs.insertion_yaw_rad, obs.left.center_m, obs.right.center_m, True, True, True, (), None)
    return [c], TargetDecision("selected", "baseline", ())


def _oracle(sample):
    cands = []
    for inst in sample.truth["instances"]:
        face = next((f for f in inst["faces"] if f["accessible"]), inst["faces"][0])
        cands.append(CandidateView(f"o{inst['instance_id']}", face["insertion_yaw_rad"], tuple(face["left_m"]),
                                   tuple(face["right_m"]), True, True, inst["fit"]["pickable"],
                                   tuple(inst["fit"]["reasons"]), inst["layer"]))
    kind, target = sample.instruction["kind"], sample.instruction.get("target")
    if kind == "ambiguous":
        return cands, TargetDecision("ask_operator", None, ())
    if kind == "unambiguous":
        return cands, TargetDecision("selected", f"o{target}", ())
    return cands, TargetDecision("selected" if cands else "none", cands[0].candidate_id if cands else None, ())


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--split", choices=("dev", "eval"), required=True)
    p.add_argument("--detector", choices=("baseline", "oracle"), required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args(argv)
    run = {"baseline": _baseline, "oracle": _oracle}[a.detector]
    verdicts, digest = [], hashlib.sha256()
    for scene_json in sorted(a.dataset.glob("*/scene.json")):
        sample = load_recognition_sample(scene_json.parent)
        if sample.meta["split"] != a.split:
            continue
        digest.update(scene_json.read_bytes())
        start = time.perf_counter()
        cands, decision = run(sample)
        verdicts.append(judge_scene(sample.meta, sample.truth, sample.instruction, cands, decision,
                                    elapsed_s=time.perf_counter() - start))
    a.output.mkdir(parents=True, exist_ok=True)
    revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    metrics = summarize(verdicts, split=a.split)
    metrics.update({"detector": a.detector, "dataset_sha256": digest.hexdigest(), "revision": revision,
                    "render_note": "rig_render_pose scenes are a CPU mechanism regression, not a render-based population"})
    (a.output / "metrics.json").write_text(json.dumps(metrics, indent=1, default=float))
    with (a.output / "verdicts.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(asdict(verdicts[0]).keys()) if verdicts else ["scene_id"])
        w.writeheader()
        for v in verdicts:
            w.writerow(asdict(v))


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 통과 확인** — `git add tools/generate_recognition_scenes.py tools/evaluate_recognition.py tests/integration/test_recognition_cli.py && FORKLIFT_REPO=... ~/bin/forklift-ws1-pytest tests/integration/test_recognition_cli.py -q -p no:cacheprovider -W error` (백그라운드) · Expected: 1 passed
- [ ] **Step 5: 커밋** — `git commit -m "tools: recognition scene generation (CPU rig, schema v2) and evaluation CLI with baseline and oracle candidates"`

### Task 12: CPU 기준선 실행과 검증 기록 (UBAI)

CPU 전용 잡이므로 전역 지침대로 **UBAI(`gate1_External`) Slurm** 에서 돌린다. 저장소의 `tools/hpc_sweep.sh jobs.tsv [파티션] [코어]` 가 동기화 → 제출 → 대기 → 회수(`artifacts/hpc/<run-id>/out/<이름>.txt`)를 한 번에 한다(원격 venv `~/forklift-venv`, 원격 사본 `~/forklift`). SSH 왕복이 2분 넘으니 묶어서 보낸다.

**Files:**
- Create: `deploy/hpc/recognition_stage1a_env.tsv`, `deploy/hpc/recognition_stage1a_generate.tsv`, `deploy/hpc/recognition_stage1a_evaluate.tsv`, `docs/validation/<실행일>-recognition-stage1a-cpu-baseline.md`

- [ ] **Step 1: 원격 환경 확인 묶음** — `deploy/hpc/recognition_stage1a_env.tsv` 한 줄로 Pillow 를 확인하고 없으면 설치한다(생성 줄들과 동시에 설치하지 않도록 **별도 sweep 으로 먼저** 1코어로 돌린다):

```
env	python -c "import PIL, numpy, yaml; print(PIL.__version__, numpy.__version__, yaml.__version__)" || ($HOME/forklift-venv/bin/pip install 'Pillow>=10' && python -c "import PIL; print(PIL.__version__)")
```

Run(worktree 에서, `run_in_background`): `tools/hpc_sweep.sh deploy/hpc/recognition_stage1a_env.tsv cpu1 1` — 출력 `env.txt` 에 세 버전이 찍혀야 한다.

- [ ] **Step 2: 생성 묶음** — 사례·형상마다 한 줄(병렬). 같은 `out/data` 아래 장면 디렉터리가 겹치지 않는다(장면 ID 에 사례·형상·시드):

```
gen_s1	python tools/generate_recognition_scenes.py --out hpc_data/stage1a --cases S1
gen_s2	python tools/generate_recognition_scenes.py --out hpc_data/stage1a --cases S2
gen_s3	python tools/generate_recognition_scenes.py --out hpc_data/stage1a --cases S3
gen_s4	python tools/generate_recognition_scenes.py --out hpc_data/stage1a --cases S4
gen_s5	python tools/generate_recognition_scenes.py --out hpc_data/stage1a --cases S5
gen_s6	python tools/generate_recognition_scenes.py --out hpc_data/stage1a --cases S6
gen_s7	python tools/generate_recognition_scenes.py --out hpc_data/stage1a --cases S7
gen_n	python tools/generate_recognition_scenes.py --out hpc_data/stage1a --cases N
```

`--exclude='data/'` 때문에 출력은 `hpc_data/` 에 둔다(동기화 대상에서 빠지지 않는 이름이지만 원격에서만 생긴다). Run: `tools/hpc_sweep.sh deploy/hpc/recognition_stage1a_generate.tsv cpu1 8` (백그라운드).

- [ ] **Step 3: 평가 묶음** — dev 만(**eval 은 돌리지 않는다** — 기하 코어 확정 전 eval 을 읽으면 상한이 무효, 설계 §5.3):

```
oracle_dev	python tools/evaluate_recognition.py --dataset hpc_data/stage1a --split dev --detector oracle --output hpc_data/stage1a_oracle_dev && cat hpc_data/stage1a_oracle_dev/metrics.json
baseline_dev	python tools/evaluate_recognition.py --dataset hpc_data/stage1a --split dev --detector baseline --output hpc_data/stage1a_baseline_dev && cat hpc_data/stage1a_baseline_dev/metrics.json && cat hpc_data/stage1a_baseline_dev/verdicts.csv
```

Run: `tools/hpc_sweep.sh deploy/hpc/recognition_stage1a_evaluate.tsv cpu1 2` (백그라운드).

- [ ] **Step 4: 결과 확인** — oracle: 모든 양성 모집단의 `miss`·`extra`·`dangerous_observed_zero` 실패 0(평가기 자기 점검; 하나라도 실패면 평가기나 정답의 결함이므로 고친 뒤 다시). baseline: 사례별 칸(`cells`)의 놓침·추가 후보·위험·음성 오검출을 표로 읽는다. 기대(설계 §1 진단): S2(맞닿음·부분 가림)·S4(위층)·S5(상판 결손)에서 놓침, S6·N 에서 추가 후보·음성 오검출. 기대와 다른 칸은 반례로 따로 적는다.
- [ ] **Step 5: 검증 기록 작성** — `docs/validation/<실행일>-recognition-stage1a-cpu-baseline.md`: 실행 조건(리비전, `dataset_sha256`, 잡음 모델, 장면 수, UBAI run-id·job ID, numpy·Pillow 버전), oracle 자기 점검, baseline 사례별 표(dev 만), 해석 경계("`rig_render_pose` CPU 기구 회귀이며 렌더 기반 모집단이 아니다 — Isaac 판정은 단계 1B").
- [ ] **Step 6: 커밋·푸시** — `git add deploy/hpc/recognition_stage1a_env.tsv deploy/hpc/recognition_stage1a_generate.tsv deploy/hpc/recognition_stage1a_evaluate.tsv docs/validation/<파일>` · `git commit -m "validation: recognition stage 1A CPU baseline on dev at UBAI (oracle self-check, current detector by case)"` · `git push`

**검토 D:** 작업 10–12 뒤 Codex — 초점: "판정 규칙이 설계 §3.4 와 같은가(특히 위험·추가 후보·지시·끼움), 빈 모집단 처리, CLI 가 정답을 입력 경로로 흘리지 않는가, 기준선 결과 해석이 데이터로 지지되는가". 수렴까지 반복.

---

## 이 계획 다음 (별도 계획)

- **단계 1B — Isaac 렌더:** `sim/isaac/render_recognition_scenes.py`(Replicator `instance_segmentation`, 팔레트 prim 묶기, `isaac_render_pose`), 렌더 전용 외형 메쉬, 잡음 판, Isaac dev 기준선. 이 계획의 조리법·정답·판 2·평가기를 그대로 쓴다.
- **단계 2 — 기하 코어:** `instance_proposals`(oracle·기하), `instance_fit`, `pocket_candidate`, `target_selection`, `candidate_tracking`. 평가는 이 계획의 `CandidateView`·`TargetDecision` 으로 변환해서 한다.
- **단계 3 — 학습 제안기(YOLO), 단계 4 — 근거리·실행기·콘솔, 단계 5 — eval 1회, 단계 6 — 임무.**

## 자체 검토 (writing-plans 체크리스트)

1. **설계 대응:** §5.1 사례 → 작업 8; §5.2 손상 → 작업 2; §5.3 렌더·잡음·TF·정답·분할·누출 방지 → 작업 4·5·7·9(Isaac 부분은 단계 1B); §3.3 통계 → 작업 6; §3.4 지표 → 작업 10; §9.3 끼움 정답 → 작업 3; §13 결정(간격·TF 출처) → Global Constraints·작업 3·9. §3.4 의 지연(`처리 지연`) 지표는 `elapsed_s` 로 기록만 한다. 임무 지표는 단계 6.
2. **자리표시 점검:** 작성 중 남았던 S2 배치 중복 식과 S7 시험의 적재 인자를 바로잡았다. TBD·"적절히" 류 문구 없음.
3. **타입 일관성:** `CandidateView`·`TargetDecision`(작업 10) ↔ CLI(작업 11) 일치. `scene_truths` 인자 순서 (spec, placement, damage, load) ↔ 작업 8·11 일치. `FitResult.reasons` 튜플 ↔ truth JSON 리스트.
4. **Review Focus:** 다섯 항목 각각 작업 4·7·10·6·8 에 시험을 넣었다.
5. **검토 ① (원자료 재계산, 작성 직후):** 끼움 시험값(EPAL 6 86.25 mm, T11 0.290 m 77.5 mm, 0.3725 m 에서 36.25 mm), 비규격 시험 형상의 개구·여유(86.25 mm)·지지(0.275 + 0.036 = 0.311 ≤ 0.330, 0.314 + 0.036 = 0.350 > 0.330)·천장(0.055 < 0.0575)·적재(11 > 10 kg), 적층 포켓 높이(0.144 + 0.022 + 0.039), 장면 수(1,020, eval 114/59/29)를 손으로 다시 계산해 일치를 확인했다. 그 과정에서 세 가지를 고쳤다 — 지시 종류를 전체 인덱스로 배정하면 순열로 뽑은 eval 에서 하한이 보장되지 않아 분할 안 순위로 바꿈, 적용되지 않는 관문(알려진 형상의 끼움 판정)이 실패로 집계되던 것을 모집단별 적용 목록으로 바꿈, CPU 전용 기준선을 ws1 대신 UBAI 로(전역 지침).
