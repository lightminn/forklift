# 팔레트 인식 일반화 단계 1A — 장면·정답·평가기 (CPU) 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **이 계획의 실행 방식(사용자 결정 2026-10-15):** 이 세션에서 직접 구현(executing-plans), **각 작업 묶음(아래 "검토 지점")마다 Codex 검토를 받고 지적이 수렴한 뒤 다음 묶음으로 간다.**
> **판:** 2판 — 1판의 검토 ① 원자료 재계산, ② Codex 적대적 검토(P1 2 · P2 16 · P3 4), ③ 결론을 보지 않은 독립 검토(계획 코드를 실제로 실행, P1 4 · P2 8)를 반영. 처리 기록은 맨 끝 "자체 검토".

**Goal:** 설계 §5·§3.3–3.4 의 장면 생성·정답·데이터 계약 판 2·통계·다중 인스턴스 평가기를 CPU 에서 끝까지 돌게 만들고, 지금 검출기의 기준선을 CPU 렌더 장면(dev)에서 기록한다.

**Architecture:** 코어(`src/forklift_core/perception/`)에 순수 numpy 모듈 — 일반화된 팔레트 형상(`pallet_model`), 손상 연산(`pallet_damage`), 포크 끼움 정답(`fork_fit`), 깊이 잡음(`depth_noise`), 통계(`recognition_stats`), 데이터 계약 판 2(`recognition_dataset`), 다중 인스턴스 평가(`evaluation_multi`) — 를 두고, `tools/` 에 CPU 인스턴스 렌더러(`instance_rig`, 기존 `scene_rig` 의 광선 교차 재사용), 정답 계산(`recognition_truth`), 장면 조리법(`recognition_scenes`), 생성·평가 CLI 를 둔다. Isaac 렌더(단계 1B)와 기하 코어(단계 2)는 이 계획이 만든 타입 위에 별도 계획으로 올린다.

**Tech Stack:** Python ≥ 3.10, numpy, Pillow(선택 의존성 `dataset`, **10 이상 13 미만** — 13 에서 `fromarray(mode=)` 가 제거된다), PyYAML(설정), pytest. scipy 는 코어에서 쓰지 않는다(코어 의존성은 numpy 뿐 — `pyproject.toml:10`).

**Spec:** `docs/design/2026-10-15-pallet-recognition-generality.md` (6판 + §13 결정 + §9.3 해석 보충 — 보충은 작업 3 에서 커밋)

## Global Constraints

- 코어 의존성은 `numpy>=1.23` 뿐이다. PNG 는 함수 안에서 `PIL` 을 지연 import 하고, `Image.fromarray` 에 `mode=` 를 넘기지 않는다(uint16 2차원 → `I;16`, uint8 H×W×3 → `RGB` 로 자동 결정).
- `tools/scene_rig.py` 는 **수정하지 않는다** — Isaac 실행 기록이 그 파일의 sha256 을 담고 있다(`tests/fixtures/near_protection_prediction/near_field_bounds_0p29.json:131`). 새 렌더러는 `tools/instance_rig.py` 에서 그 함수를 import 해 쓴다.
- 기존 `pallet_boxes`·`PalletGeometry`·`check_fork_fit`·`scene_dataset` 판 1 의 동작을 바꾸지 않는다. 일반화 형상은 알려진 형상에서 **같은 22 상자(이름·크기·중심)** 를 내야 한다.
- 팔레트 좌표: x = 깊이 방향(x 진입면은 x = −깊이/2, 포크는 +x 로 들어간다), y = 폭 방향, z = 받침면(바닥 또는 아래 팔레트 윗면)에서 위. 진입면의 **왼쪽 축**은 삽입축 a 에 대해 (−a_y, a_x) 이고 `PocketObservation` 의 left 와 같다(`pocket_observation.py:44`, `scene_rig.true_pockets` 의 첫째 = +y = 왼쪽).
- 포크 실측: 날 폭 0.055 m, 두께 0.025 m, 길이 0.36 m, 승강 0.28 m. 최저 날 중심 0.040 m(모델값, 미실측). 캐리지 한계 0.346 m, 예비량 0.016 m → 삽입 목표 min(깊이 × 0.6, 0.330 m).
- 시험 포크 간격(설계 §13): EPAL 6 **0.3725 m**, T11 × 0.6 **0.290 m**.
- 끼움·위험 판정 상수(설계 §3.1·§3.4·§9.3): M_req 잠정 **0.066 m**, 추종 종점 허용 **0.0338 m**(8 + 25.8 mm), 삽입 끝 투영 길이 **0.33 m**, 삽입축 위험 **0.040 m**, 종방향 지지 여유 **0.036 m**, 횡방향 지지 여유 **0.020 m**, 세로 여유 **0.005 m**, 적재 한도 **10 kg**.
- 사유 코드 다섯 개(설계 §3.4): `fork_spacing_adjust`, `lift_alignment_needed`, `support_unverified`, `payload_limit`, `near_tracking_unavailable`.
- **알려진 형상의 적재(설계 §9.3 해석 보충, 작업 3 에서 설계에 기록):** 알려진 형상은 작업 지시에 질량이 없으면 **빈 팔레트로 가정**하고 적재 사유를 내지 않으며, 결과에 그 가정을 표시한다(`payload_assumed_empty`). 지시가 질량을 주면 그 값으로 검사한다. 비규격은 질량·질량중심이 지시에 없으면 `support_unverified` 다. 사용자 결정 ③(지시 없으면 가장 가까운 팔레트를 자율로)이 알려진 형상에서 성립하려면 이 가정이 필요하다.
- 통계: 단측 95 % Clopper–Pearson 상한. 분할 eval = ⌈0.3 × 총⌉(ADR 0004 D6). **시험 단위는 장면**이다.
- 장면 수(설계 §3.4): 알려진 형상 S1–S6 각 61 × 2 형상 = 732, S7 194(eval 59), N 94(eval 29) = 1,020. 형상별 eval 지시 장면 ≥ 59(애매하지 않은 것 ≥ 29, 애매한 것 ≥ 15).
- TF 출처(설계 §13): 판 2 `tf.json` `origin` ∈ {`received_tf_static`, `isaac_render_pose`, `rig_render_pose`}, 그리고 `rig_render_pose` ↔ `render_source = "rig"`, `isaac_render_pose` ↔ `"isaac"` 로 묶인다. CPU 렌더(이 계획)는 **기구 회귀용이지 렌더 기반 판정 모집단이 아니다**(ADR 0004 D5). 정식 판정은 `isaac` + 잡음 판에서만 한다(설계 §5.3).
- 카메라: `config/isaac_perception_camera.yaml` — 위치 (0.75, 0, 0.27) m, 쿼터니언 xyzw (−0.5, 0.5, −0.5, 0.5), 640 × 480, 초점 465.741156 px.
- 시드: 장면 시드 하나에서 `np.random.SeedSequence(seed).spawn(3)` 으로 배치·지시·잡음 스트림을 나눈다. 분할 순열은 `SeedSequence([first_seed, 0x5E1F])`.
- pytest 는 ws1: `FORKLIFT_REPO=/home/light/school/2-2/embedded-design/forklift-wt-perception ~/bin/forklift-ws1-pytest <인자>`(`-W error` 포함). 래퍼는 `git ls-files` 만 보내므로 **새 파일은 먼저 `git add`**. 로컬은 1분 안에 끝나는 한 파일 확인만.
- 커밋: 파일 단위 `git add`, 메시지 `<영역>: <내용>` + `Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>`.

## Review Focus

1. **맞닿은 팔레트(간격 0)** — 생성기가 eval 에 맞닿은 장면을 실제로 넣고(분할 안 순위 r % 4 == 0), 인스턴스 렌더가 두 팔레트를 다른 ID 로 가르며, 진입면 노출이 각자 계산돼야 한다. → 작업 4·7·8 의 시험.
2. **카메라를 등진 면** — 그 면의 정답 포켓을 그대로 낸 후보는 대응되지 않아야 한다(놓침 + 추가 후보). → 작업 10 의 시험.
3. **정답상 집을 수 없는 팔레트** — 올바르게 멈추면 위험이 아니고, 집기를 허용하면 위험이다. 집을 수 있는 비규격이 eval 에 실제로 있어야 한다. → 작업 8·10 의 시험.
4. **분모가 0 인 관문** — 상한 `null`, 합격 거짓. 해당 없는 관문은 만들지 않는다. → 작업 6·10 의 시험.
5. **정답의 입력 누출** — 입력 로더는 정답·인스턴스 영상·지시 정답(종류·대상)을 읽지 않고, 기준선은 장면의 형상 라벨로 prior 를 고르지 않는다. → 작업 9·11 의 시험.

## 파일 구조

| 파일 | 책임 |
|---|---|
| `src/forklift_core/perception/pallet_model.py` (신규) | `PalletSpec`·`FaceLayout`(면별 차선 바닥·천장·포켓 띠) · 알려진 형상 변환 · 상자 생성 · 날 여유 기준 포켓 쌍 선택 |
| `src/forklift_core/perception/pallet_damage.py` (신규) | 손상 연산(설계 §5.2) |
| `src/forklift_core/perception/fork_fit.py` (신규) | 포크 설정·적재 선언·면별 끼움 판정(사유 코드)·위험 문턱·층 도달 |
| `src/forklift_core/perception/depth_noise.py` (신규) | 깊이 잡음 모델 |
| `src/forklift_core/perception/recognition_stats.py` (신규) | Clopper–Pearson 상한 |
| `src/forklift_core/perception/recognition_dataset.py` (신규) | 판 2 쓰기·읽기, 입력/정답 분리, 기존 입력 검증 유지 |
| `src/forklift_core/perception/evaluation_multi.py` (신규) | `CandidateView`·`TargetDecision`, 장면 판정, 모집단 관문 |
| `tools/instance_rig.py` (신규) | 인스턴스 ID·적중점을 내는 CPU 광선 렌더 |
| `tools/recognition_truth.py` (신규) | 면별 포켓·노출·끼움 정답 |
| `tools/recognition_scenes.py` (신규) | 사례 조리법 S1–S7·N, 층화 S7, 지시, 분할·시드 |
| `tools/generate_recognition_scenes.py`, `tools/evaluate_recognition.py` (신규) | 생성·평가 CLI |
| `docs/decisions/0004-…md`, `docs/design/2026-10-15-…md` (수정) | D5 개정(판 2 TF 출처), §9.3 해석 보충 |

## 검토 지점

- **검토 A (작업 1–3 뒤):** 형상·손상·끼움.
- **검토 B (작업 4–6 뒤):** 렌더·잡음·통계.
- **검토 C (작업 7–9 뒤):** 정답·조리법·데이터 계약.
- **검토 D (작업 10–12 뒤):** 평가기·CLI·기준선 실행 결과.

각 검토는 Codex `adversarial-review`(브랜치 변경분, 초점 문장) 또는 `task`(질문 파일 stdin)로 하고 "새 P1/P2 없음"까지 같은 스레드(`--resume-last`)로 반복한다. 대기는 `run_in_background`.

---

### Task 1: 일반화 팔레트 형상 `pallet_model`

**Files:**
- Create: `src/forklift_core/perception/pallet_model.py`
- Test: `tests/unit/perception/test_pallet_model.py`

**Interfaces:**
- Consumes: `PalletGeometry`, `PalletBox` (`pallet_geometry.py`)
- Produces:
  - `PalletSpec` (frozen): `name`, `known: bool`, `overall_width_m`, `overall_depth_m`, `deck_bottom_m`, `opening_height_m`, `stringer_m`, `top_thickness_m`, `column_centres_y_m: tuple`, `column_widths_m: tuple`, `row_centres_x_m: tuple`, `row_depth_m`, `top_style` (`"boards"`|`"flat"`), `top_board_count: int`, `top_board_width_m`, `entry_axes: tuple[str, ...]`; 속성 `overall_height_m`; `to_json() -> dict`
  - `spec_from_geometry(geometry, *, entry_axes) -> PalletSpec`, `spec_boxes(spec) -> list[PalletBox]`
  - `FaceLayout` (frozen): `axis`, `width_m`, `depth_m`, `member_intervals_m`, `opening_intervals_m`, `pocket_band_m: tuple[float, float]`(포켓 정답 높이 띠, 받침면 기준), `lane_floor_m`(차선 아래 가장 높은 장애물 윗면), `lane_ceiling_m`(차선 위 가장 낮은 장애물 아랫면)
  - `face_layout(spec, axis) -> FaceLayout`
  - `pair_margin(layout, pair, spacing_m, blade_width_m) -> tuple[float, float]` → (최소 날 여유, 포크 중심의 왼쪽 축 좌표)
  - `choose_pocket_pair(layout, spacing_m, blade_width_m) -> tuple[int, int]` — **최소 날 여유가 가장 큰 쌍**(동률이면 면 중심에 가까운 쌍, 그다음 낮은 인덱스)
  - `KNOWN_ENTRY_AXES = {"epal6": ("x",), "t11_06": ("x", "y")}`

면별 높이 규칙(박스 모델 그대로): x 면 — 하판은 블록 열 아래에만 있어 차선 바닥은 받침면(0), 천장은 스트링거 아랫면(`deck_bottom + opening`), 포켓 띠는 블록 띠(`deck_bottom`, `deck_bottom + opening`) — `true_pockets` 와 같은 관례. y 면 — 포크가 x 방향 하판을 가로지르므로 바닥은 `deck_bottom`, 스트링거는 행 위에만 있어 천장은 상판 아랫면(`deck_bottom + opening + stringer`), 포켓 띠는 그 사이.

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
    pair_margin,
    spec_boxes,
    spec_from_geometry,
)

ROOT = Path(__file__).resolve().parents[3]


def _known(name):
    geometry = load_pallet_geometry(ROOT / f"config/pallet_geometry_{name}.yaml")
    return geometry, spec_from_geometry(geometry, entry_axes=KNOWN_ENTRY_AXES[name])


def _spec(**overrides):
    base = dict(
        name="t", known=False, overall_width_m=1.2, overall_depth_m=0.6,
        deck_bottom_m=0.02, opening_height_m=0.08, stringer_m=0.02, top_thickness_m=0.02,
        column_centres_y_m=(-0.57, -0.055, 0.055, 0.57), column_widths_m=(0.06,) * 4,
        row_centres_x_m=(-0.25, 0.0, 0.25), row_depth_m=0.10, top_style="boards",
        top_board_count=7, top_board_width_m=0.10, entry_axes=("x",),
    )
    base.update(overrides)
    return PalletSpec(**base)


@pytest.mark.parametrize("name", ["epal6", "t11_06"])
def test_known_shape_reproduces_the_22_box_model(name):
    geometry, spec = _known(name)
    expected = {b.name: (b.size_m, b.centre_m) for b in pallet_boxes(geometry)}
    produced = {b.name: (b.size_m, b.centre_m) for b in spec_boxes(spec)}
    assert produced.keys() == expected.keys()
    for key, (size, centre) in expected.items():
        assert produced[key][0] == pytest.approx(size, abs=1e-12)
        assert produced[key][1] == pytest.approx(centre, abs=1e-12)


def test_epal6_x_face():
    geometry, spec = _known("epal6")
    layout = face_layout(spec, "x")
    assert [hi - lo for lo, hi in layout.opening_intervals_m] == pytest.approx([geometry.opening_width_m] * 2)
    assert (layout.width_m, layout.depth_m) == pytest.approx((0.800, 0.600))
    assert layout.lane_floor_m == 0.0
    assert layout.lane_ceiling_m == pytest.approx(0.100)
    assert layout.pocket_band_m == pytest.approx((0.022, 0.100))


def test_t11_y_face_lane_runs_from_bottom_boards_to_top_boards():
    _, spec = _known("t11_06")
    layout = face_layout(spec, "y")
    assert [hi - lo for lo, hi in layout.member_intervals_m] == pytest.approx([0.080] * 3)
    assert layout.lane_floor_m == pytest.approx(0.015)
    assert layout.lane_ceiling_m == pytest.approx(0.075)
    assert layout.pocket_band_m == pytest.approx((0.015, 0.075))


def test_y_face_is_not_an_entry_face_for_epal6():
    _, spec = _known("epal6")
    with pytest.raises(ValueError, match="not an entry axis"):
        face_layout(spec, "y")


def test_pair_choice_maximises_blade_margin_not_centre_spacing():
    # Openings (-0.54,-0.085), (-0.025,0.025), (0.085,0.54). At 0.3725 m the
    # spacing-closest pair (0,1) has a 50 mm middle opening (blade 55 mm);
    # the pair (0,2) fits with 73.75 mm (Codex counterexample).
    layout = face_layout(_spec(), "x")
    pair = choose_pocket_pair(layout, 0.3725, 0.055)
    assert pair == (0, 2)
    assert pair_margin(layout, pair, 0.3725, 0.055)[0] == pytest.approx(0.07375)


def test_overlapping_columns_are_rejected():
    with pytest.raises(ValueError, match="overlap"):
        _spec(overall_width_m=0.6, column_centres_y_m=(-0.25, -0.20, 0.25), column_widths_m=(0.10,) * 3)
```

- [ ] **Step 2: 실패 확인** — Run: `python -m pytest tests/unit/perception/test_pallet_model.py -q -p no:cacheprovider` · Expected: `ModuleNotFoundError: forklift_core.perception.pallet_model`

- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/pallet_model.py
"""Generalised box-model pallet: N block/stringer columns, any entry axes.

Pallet frame: x along the depth (forks travel +x through an x-entry face at
x = -depth/2), y across the width, z up from the support surface. Bottom boards
run under the columns only, as in ``pallet_boxes``, so x-face fork lanes are
open to the support surface.
"""

from dataclasses import asdict, dataclass, fields

from forklift_core._validation import _finite_scalar
from forklift_core.perception.pallet_geometry import PalletBox, PalletGeometry

KNOWN_ENTRY_AXES = {"epal6": ("x",), "t11_06": ("x", "y")}
TOP_STYLES = ("boards", "flat")
_EPS = 1e-9


def _intervals(centres, widths):
    return tuple((c - w / 2, c + w / 2) for c, w in zip(centres, widths, strict=True))


def _check_members(centres, widths, span, label):
    if len(centres) != len(widths) or len(centres) < 3:
        raise ValueError(f"{label} needs at least three members with widths")
    for index, width in enumerate(widths):
        if _finite_scalar(width, f"{label} width[{index}]") <= 0:
            raise ValueError(f"{label} widths must be positive")
    intervals = _intervals(centres, widths)
    if any(b[0] < a[1] - _EPS for a, b in zip(intervals, intervals[1:])):
        raise ValueError(f"{label} members overlap or are out of order")
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
    column_centres_y_m: tuple
    column_widths_m: tuple
    row_centres_x_m: tuple
    row_depth_m: float
    top_style: str
    top_board_count: int
    top_board_width_m: float
    entry_axes: tuple

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("name must be nonempty text")
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name.endswith("_m") and not isinstance(value, (tuple, list)):
                if _finite_scalar(value, field.name) <= 0:
                    raise ValueError(f"{field.name} must be positive")
        for name in ("column_centres_y_m", "column_widths_m", "row_centres_x_m", "entry_axes"):
            value = tuple(getattr(self, name))
            object.__setattr__(self, name, value if name == "entry_axes" else tuple(map(float, value)))
        _check_members(self.column_centres_y_m, self.column_widths_m, self.overall_width_m, "columns")
        _check_members(self.row_centres_x_m, (self.row_depth_m,) * len(self.row_centres_x_m),
                       self.overall_depth_m, "rows")
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

    def to_json(self) -> dict:
        obj = asdict(self)
        for name in ("column_centres_y_m", "column_widths_m", "row_centres_x_m", "entry_axes"):
            obj[name] = list(obj[name])
        return obj


def spec_from_geometry(geometry: PalletGeometry, *, entry_axes) -> PalletSpec:
    g = geometry
    return PalletSpec(
        name=g.geometry_version, known=True,
        overall_width_m=g.overall_width_m, overall_depth_m=g.overall_depth_m,
        deck_bottom_m=g.deck_bottom_m, opening_height_m=g.block_height_m,
        stringer_m=g.stringer_m, top_thickness_m=g.top_board_thickness_m,
        column_centres_y_m=tuple(g.block_centres_y_m()), column_widths_m=tuple(g.block_widths_m),
        row_centres_x_m=tuple(g.block_centres_x_m()), row_depth_m=g.block_depth_m,
        top_style="boards", top_board_count=g.top_board_count, top_board_width_m=g.top_board_width_m,
        entry_axes=tuple(entry_axes),
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
        boxes.append(PalletBox(f"bottom_board_{index}", (s.overall_depth_m, width, s.deck_bottom_m),
                               (0.0, y, s.deck_bottom_m / 2)))
    block_z = s.deck_bottom_m + s.opening_height_m / 2
    for xi, x in enumerate(s.row_centres_x_m):
        for yi, (y, width) in enumerate(columns):
            boxes.append(PalletBox(f"block_x{xi}_y{yi}", (s.row_depth_m, width, s.opening_height_m), (x, y, block_z)))
    stringer_z = s.deck_bottom_m + s.opening_height_m + s.stringer_m / 2
    for xi, x in enumerate(s.row_centres_x_m):
        boxes.append(PalletBox(f"stringer_{xi}", (s.row_depth_m, s.overall_width_m, s.stringer_m), (x, 0.0, stringer_z)))
    top_z = s.overall_height_m - s.top_thickness_m / 2
    if s.top_style == "flat":
        boxes.append(PalletBox("top_deck", (s.overall_depth_m, s.overall_width_m, s.top_thickness_m), (0.0, 0.0, top_z)))
    else:
        for index, y in enumerate(_top_board_centres_y(s)):
            boxes.append(PalletBox(f"top_board_{index}", (s.overall_depth_m, s.top_board_width_m, s.top_thickness_m),
                                   (0.0, y, top_z)))
    return boxes


@dataclass(frozen=True)
class FaceLayout:
    axis: str
    width_m: float
    depth_m: float
    member_intervals_m: tuple
    opening_intervals_m: tuple
    pocket_band_m: tuple
    lane_floor_m: float
    lane_ceiling_m: float


def face_layout(spec: PalletSpec, axis: str) -> FaceLayout:
    """Entering an x face (+x travel), left is +y. Entering a y face (+y travel),
    left is -x. The opposite face is reached by turning the pallet yaw by pi."""
    if axis not in spec.entry_axes:
        raise ValueError(f"{axis!r} is not an entry axis of {spec.name}")
    if axis == "x":
        members = _intervals(spec.column_centres_y_m, spec.column_widths_m)
        width, depth = spec.overall_width_m, spec.overall_depth_m
        band = (spec.deck_bottom_m, spec.deck_bottom_m + spec.opening_height_m)
        floor, ceiling = 0.0, band[1]
    else:
        rows = _intervals(spec.row_centres_x_m, (spec.row_depth_m,) * len(spec.row_centres_x_m))
        members = tuple(sorted((-hi, -lo) for lo, hi in rows))
        width, depth = spec.overall_depth_m, spec.overall_width_m
        band = (spec.deck_bottom_m, spec.deck_bottom_m + spec.opening_height_m + spec.stringer_m)
        floor, ceiling = band
    openings = tuple((a[1], b[0]) for a, b in zip(members, members[1:]))
    return FaceLayout(axis, width, depth, members, openings, band, floor, ceiling)


def pair_margin(layout: FaceLayout, pair, spacing_m: float, blade_width_m: float) -> tuple[float, float]:
    (l0, h0), (l1, h1) = (layout.opening_intervals_m[i] for i in pair)
    centre = ((l0 + h0) / 2 + (l1 + h1) / 2) / 2
    half, w = spacing_m / 2, blade_width_m / 2
    blades = ((centre - half - w, centre - half + w), (centre + half - w, centre + half + w))
    margins = (blades[0][0] - l0, h0 - blades[0][1], blades[1][0] - l1, h1 - blades[1][1])
    return min(margins), centre


def choose_pocket_pair(layout: FaceLayout, spacing_m: float, blade_width_m: float) -> tuple[int, int]:
    n = len(layout.opening_intervals_m)
    if n < 2:
        raise ValueError("A face needs two openings")
    best = None
    for i in range(n):
        for j in range(i + 1, n):
            margin, centre = pair_margin(layout, (i, j), spacing_m, blade_width_m)
            key = (-round(margin, 9), round(abs(centre), 9), i, j)
            if best is None or key < best[0]:
                best = (key, (i, j))
    return best[1]
```

- [ ] **Step 4: 통과 확인** — Expected: 7 passed
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/pallet_model.py tests/unit/perception/test_pallet_model.py` · `git commit -m "perception: generalised pallet shape with per-face lane floor/ceiling and blade-margin pocket pair choice; reproduces the 22-box model"`

### Task 2: 손상 연산 `pallet_damage`

**Files:**
- Create: `src/forklift_core/perception/pallet_damage.py`
- Test: `tests/unit/perception/test_pallet_damage.py`

**Interfaces:**
- Consumes: `PalletBox`
- Produces: `DAMAGE_KINDS`, `DamageOp`(frozen: `kind`, `target`, `params: tuple[tuple[str, float], ...]`; `get(name)`; `to_json()`), `apply_damage(boxes, ops) -> list[PalletBox]`, `removed_volume_fraction(before, after) -> float`

연산(설계 §5.2): `top_board_missing`(판 제거) · `top_board_broken`(판을 앞에서 `fraction` 지점에서 잘라 `keep` 0 = 앞 조각 길이 f·L, 1 = 뒤 조각 길이 (1 − f)·L) · `block_corner_chipped`(진입면 쪽 모서리를 `depth`·`width` 10–40 mm, 부피 30 % 이하) · `block_split`(y 가운데 `gap` 5–15 mm).

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_pallet_damage.py
from pathlib import Path

import pytest

from forklift_core.perception.pallet_damage import DamageOp, apply_damage, removed_volume_fraction
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
    assert "top_board_3" not in _by_name(out) and len(out) == len(epal_boxes) - 1


@pytest.mark.parametrize(("keep", "length", "front_x"), [(0.0, 0.24, -0.30), (1.0, 0.36, -0.06)])
def test_broken_board_cuts_at_the_fraction_from_the_front(epal_boxes, keep, length, front_x):
    op = DamageOp("top_board_broken", "top_board_0", (("fraction", 0.4), ("keep", keep)))
    piece = _by_name(apply_damage(epal_boxes, [op]))["top_board_0"]
    assert piece.size_m[0] == pytest.approx(length)
    assert piece.centre_m[0] - piece.size_m[0] / 2 == pytest.approx(front_x)


def test_chipped_corner_leaves_a_notch_on_the_entry_face(epal_boxes):
    op = DamageOp("block_corner_chipped", "block_x0_y0", (("depth", 0.03), ("width", 0.02), ("side", 1.0)))
    out = apply_damage(epal_boxes, [op])
    parts = [b for b in out if b.name.startswith("block_x0_y0")]
    front = min(parts, key=lambda b: b.centre_m[0])
    assert len(parts) == 2 and front.size_m[1] == pytest.approx(0.100 - 0.02)
    assert 0 < removed_volume_fraction(epal_boxes, out) < 0.01


def test_split_block_leaves_a_gap(epal_boxes):
    parts = [b for b in apply_damage(epal_boxes, [DamageOp("block_split", "block_x0_y1", (("gap", 0.01),))])
             if b.name.startswith("block_x0_y1")]
    assert len(parts) == 2 and sum(p.size_m[1] for p in parts) == pytest.approx(0.145 - 0.01)


@pytest.mark.parametrize(
    ("kind", "target", "params"),
    [
        ("top_board_broken", "top_board_0", (("fraction", 0.9), ("keep", 0.0))),
        ("block_corner_chipped", "block_x0_y0", (("depth", 0.08), ("width", 0.08), ("side", 0.0))),
        ("block_split", "block_x0_y0", (("gap", 0.05),)),
        ("top_board_missing", "block_x0_y0", ()),
    ],
)
def test_out_of_range_or_wrong_target_is_rejected(kind, target, params):
    with pytest.raises(ValueError):
        DamageOp(kind, target, params)


def test_damaging_a_missing_box_is_rejected(epal_boxes):
    once = apply_damage(epal_boxes, [DamageOp("top_board_missing", "top_board_1", ())])
    with pytest.raises(ValueError, match="No box"):
        apply_damage(once, [DamageOp("top_board_missing", "top_board_1", ())])
```

- [ ] **Step 2: 실패 확인** — `ModuleNotFoundError`
- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/pallet_damage.py
"""Damage operators on a pallet box list (design §5.2). Render, physics and mass
all come from the same output list, so a damaged pallet is one object."""

from collections.abc import Sequence
from dataclasses import dataclass

from forklift_core.perception.pallet_geometry import PalletBox

DAMAGE_KINDS = ("top_board_missing", "top_board_broken", "block_corner_chipped", "block_split")
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
    params: tuple

    def __post_init__(self) -> None:
        if self.kind not in DAMAGE_KINDS:
            raise ValueError(f"Unknown damage kind {self.kind!r}")
        prefix = "top_board_" if self.kind.startswith("top_board") else "block_"
        if not self.target.startswith(prefix):
            raise ValueError(f"{self.kind} must target a {prefix}* box")
        object.__setattr__(self, "params", tuple((str(k), float(v)) for k, v in self.params))
        for name, value in self.params:
            bounds = _RANGES.get((self.kind, name))
            if bounds and not bounds[0] <= value <= bounds[1]:
                raise ValueError(f"{self.kind}.{name}={value} outside {bounds}")

    def get(self, name: str) -> float:
        for key, value in self.params:
            if key == name:
                return value
        raise ValueError(f"{self.kind} needs parameter {name!r}")

    def to_json(self) -> dict:
        return {"kind": self.kind, "target": self.target, "params": [list(p) for p in self.params]}


def _volume(box: PalletBox) -> float:
    x, y, z = box.size_m
    return x * y * z


def _apply_one(box: PalletBox, op: DamageOp) -> list[PalletBox]:
    (sx, sy, sz), (cx, cy, cz) = box.size_m, box.centre_m
    front = cx - sx / 2
    if op.kind == "top_board_missing":
        return []
    if op.kind == "top_board_broken":
        cut = front + sx * op.get("fraction")
        if op.get("keep") == 0.0:
            return [PalletBox(box.name, (cut - front, sy, sz), ((front + cut) / 2, cy, cz))]
        back = cx + sx / 2
        return [PalletBox(box.name, (back - cut, sy, sz), ((cut + back) / 2, cy, cz))]
    if op.kind == "block_corner_chipped":
        depth, width = op.get("depth"), op.get("width")
        if depth * width * sz > 0.3 * _volume(box):
            raise ValueError("A chipped corner may remove at most 30 % of the block")
        sign = 1.0 if op.get("side") == 1.0 else -1.0
        rear = PalletBox(f"{box.name}_rear", (sx - depth, sy, sz), (front + depth + (sx - depth) / 2, cy, cz))
        notch = PalletBox(f"{box.name}_front", (depth, sy - width, sz), (front + depth / 2, cy - sign * width / 2, cz))
        return [rear, notch]
    gap = op.get("gap")
    half = (sy - gap) / 2
    return [
        PalletBox(f"{box.name}_a", (sx, half, sz), (cx, cy - (gap + half) / 2, cz)),
        PalletBox(f"{box.name}_b", (sx, half, sz), (cx, cy + (gap + half) / 2, cz)),
    ]


def apply_damage(boxes: Sequence[PalletBox], ops: Sequence[DamageOp]) -> list[PalletBox]:
    out = list(boxes)
    for op in ops:
        names = [b.name for b in out]
        if op.target not in names:
            raise ValueError(f"No box named {op.target!r} (already damaged?)")
        index = names.index(op.target)
        out[index : index + 1] = _apply_one(out[index], op)
    return out


def removed_volume_fraction(before: Sequence[PalletBox], after: Sequence[PalletBox]) -> float:
    total = sum(_volume(b) for b in before)
    return (total - sum(_volume(b) for b in after)) / total
```

- [ ] **Step 4: 통과 확인** — Expected: 10 passed
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/pallet_damage.py tests/unit/perception/test_pallet_damage.py` · `git commit -m "perception: pallet damage operators (missing/broken top board cut from the front, chipped corner, split block)"`

### Task 3: 포크 끼움 정답 `fork_fit` + 설계 §9.3 해석 보충

**Files:**
- Create: `src/forklift_core/perception/fork_fit.py`
- Modify: `docs/design/2026-10-15-pallet-recognition-generality.md` (§9.3 끝에 해석 보충 한 문단)
- Test: `tests/unit/perception/test_fork_fit.py`

**Interfaces:**
- Consumes: `PalletSpec`, `face_layout`, `choose_pocket_pair`, `pair_margin`, `target_insertion_depth_m` (`pallet_geometry.py:257`)
- Produces: 상수(Global Constraints), `TEST_FORK_SPACING_M`, `ForkSetup`(`spacing_m`, `blade_width_m=0.055`, `blade_thickness_m=0.025`, `lowest_centre_m=0.040`, `carriage_limit_m=0.346`, `reserve_m=0.016`, `lift_travel_m=0.28`; 속성 `blade_top_m`, `blade_bottom_m`), `LoadDeclaration`(`pallet_mass_kg`, `load_mass_kg`, `com_x_max_m`, `com_y_max_m`; `to_json()`), `FitResult`(`pickable`, `reasons`, `axis`, `pair`, `min_lateral_margin_m`, `fork_centre_offset_m`, `insertion_target_m`, `payload_assumed_empty`; `to_json()`), `fork_fit(spec, axis, setup, load, *, support_z_m=0.0, m_req_m=M_REQ_PROVISIONAL_M) -> FitResult`, `danger_threshold_m(fit) -> float`, `lane_reachable(spec, axis, setup, *, support_z_m) -> bool`

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_fork_fit.py
from pathlib import Path

import pytest

from forklift_core.perception.fork_fit import (
    TEST_FORK_SPACING_M, ForkSetup, LoadDeclaration, danger_threshold_m, fork_fit, lane_reachable,
)
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_model import KNOWN_ENTRY_AXES, PalletSpec, spec_from_geometry

ROOT = Path(__file__).resolve().parents[3]


def _known(name):
    return spec_from_geometry(load_pallet_geometry(ROOT / f"config/pallet_geometry_{name}.yaml"),
                              entry_axes=KNOWN_ENTRY_AXES[name])


def _ns(**overrides):
    base = dict(
        name="ns", known=False, overall_width_m=0.82, overall_depth_m=0.55,
        deck_bottom_m=0.022, opening_height_m=0.078, stringer_m=0.022, top_thickness_m=0.022,
        column_centres_y_m=(-0.36, 0.0, 0.36), column_widths_m=(0.10, 0.145, 0.10),
        row_centres_x_m=(-0.2025, 0.0, 0.2025), row_depth_m=0.145, top_style="boards",
        top_board_count=7, top_board_width_m=0.10, entry_axes=("x",),
    )
    base.update(overrides)
    return PalletSpec(**base)


CENTRED = LoadDeclaration(2.0, 6.0, com_x_max_m=0.275, com_y_max_m=0.0)


@pytest.mark.parametrize(("name", "margin"), [("epal6", 0.08625), ("t11_06", 0.0775)])
def test_known_shapes_at_their_test_spacing(name, margin):
    fit = fork_fit(_known(name), "x", ForkSetup(TEST_FORK_SPACING_M[name]), None)
    assert fit.pickable and fit.reasons == () and fit.payload_assumed_empty
    assert fit.min_lateral_margin_m == pytest.approx(margin, abs=1e-9)


def test_t11_at_the_epal_spacing_needs_adjustment():
    fit = fork_fit(_known("t11_06"), "x", ForkSetup(0.3725), None)
    assert fit.min_lateral_margin_m == pytest.approx(0.03625, abs=1e-9)
    assert fit.reasons == ("fork_spacing_adjust",)


def test_known_shape_with_a_declared_heavy_load_hits_the_payload_limit():
    heavy = LoadDeclaration(9.0, 3.0, com_x_max_m=0.3, com_y_max_m=0.0)
    assert fork_fit(_known("epal6"), "x", ForkSetup(0.3725), heavy).reasons == ("payload_limit",)


def test_danger_threshold_is_margin_minus_tracking_allowance():
    assert danger_threshold_m(fork_fit(_known("t11_06"), "x", ForkSetup(0.290), None)) == pytest.approx(0.0437)


def test_nonstandard_rules():
    s = ForkSetup(0.3725)
    assert fork_fit(_ns(), "x", s, None).reasons == ("support_unverified",)
    assert fork_fit(_ns(), "x", s, CENTRED).pickable
    assert "support_unverified" in fork_fit(_ns(), "x", s, LoadDeclaration(2.0, 6.0, 0.275, 0.24)).reasons
    deep = _ns(overall_depth_m=0.60, row_centres_x_m=(-0.2275, 0.0, 0.2275))
    assert "support_unverified" in fork_fit(deep, "x", s, LoadDeclaration(2.0, 6.0, 0.314, 0.0)).reasons
    assert "near_tracking_unavailable" in fork_fit(_ns(top_style="flat"), "x", s, CENTRED).reasons
    low = _ns(deck_bottom_m=0.010, opening_height_m=0.045)  # ceiling 0.055 < 0.0525 + 0.005
    assert "lift_alignment_needed" in fork_fit(low, "x", s, CENTRED).reasons
    assert "payload_limit" in fork_fit(_ns(), "x", s, LoadDeclaration(4.0, 7.0, 0.275, 0.0)).reasons


def test_upper_layer_needs_lift_alignment_but_is_reachable():
    spec = _known("epal6")
    fit = fork_fit(spec, "x", ForkSetup(0.3725), None, support_z_m=0.288)
    assert "lift_alignment_needed" in fit.reasons
    # Lane 0.288-0.388 m; blades lifted 0.28 m sit at 0.3075-0.3325 m.
    assert lane_reachable(spec, "x", ForkSetup(0.3725), support_z_m=0.288)
    assert not lane_reachable(spec, "x", ForkSetup(0.3725), support_z_m=0.432)
```

- [ ] **Step 2: 실패 확인** — `ModuleNotFoundError`
- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/fork_fit.py
"""Fork-fit truth and decision with reason codes (design §3.1, §9.2, §9.3).

Margins are measured from the real blade edges at the configured spacing, with
the fork centre on the chosen opening pair's midpoint. The support check is a
reach/footprint precheck against an instructed centre of mass, not a support
polygon or contact verification (ADR 0004 D3).
"""

from dataclasses import asdict, dataclass

from forklift_core.perception.pallet_geometry import target_insertion_depth_m
from forklift_core.perception.pallet_model import PalletSpec, choose_pocket_pair, face_layout, pair_margin

M_REQ_PROVISIONAL_M = 0.066
TRACKING_ALLOWANCE_M = 0.0338
TIP_LEVER_M = 0.33
AXIAL_DANGER_M = 0.040
SUPPORT_LONGITUDINAL_MARGIN_M = 0.036
SUPPORT_LATERAL_MARGIN_M = 0.020
VERTICAL_MARGIN_M = 0.005
PAYLOAD_LIMIT_KG = 10.0
REASON_CODES = ("fork_spacing_adjust", "lift_alignment_needed", "support_unverified",
                "payload_limit", "near_tracking_unavailable")
TEST_FORK_SPACING_M = {"epal6": 0.3725, "t11_06": 0.290}


@dataclass(frozen=True)
class ForkSetup:
    spacing_m: float
    blade_width_m: float = 0.055
    blade_thickness_m: float = 0.025
    lowest_centre_m: float = 0.040
    carriage_limit_m: float = 0.346
    reserve_m: float = 0.016
    lift_travel_m: float = 0.28

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

    def to_json(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class FitResult:
    pickable: bool
    reasons: tuple
    axis: str
    pair: tuple
    min_lateral_margin_m: float
    fork_centre_offset_m: float
    insertion_target_m: float
    payload_assumed_empty: bool

    def to_json(self) -> dict:
        obj = asdict(self)
        obj["reasons"], obj["pair"] = list(self.reasons), list(self.pair)
        return obj


def _lane(spec, axis, support_z_m):
    layout = face_layout(spec, axis)
    return layout, support_z_m + layout.lane_floor_m, support_z_m + layout.lane_ceiling_m


def lane_reachable(spec: PalletSpec, axis: str, setup: ForkSetup, *, support_z_m: float) -> bool:
    """Some lift in [0, travel] puts both blades inside the lane with margins."""
    _, floor, ceiling = _lane(spec, axis, support_z_m)
    lift = max(0.0, floor + VERTICAL_MARGIN_M - setup.blade_bottom_m)
    return lift <= setup.lift_travel_m + 1e-12 and setup.blade_top_m + lift + VERTICAL_MARGIN_M <= ceiling + 1e-12


def fork_fit(spec, axis, setup, load, *, support_z_m: float = 0.0, m_req_m: float = M_REQ_PROVISIONAL_M) -> FitResult:
    layout, floor, ceiling = _lane(spec, axis, support_z_m)
    pair = choose_pocket_pair(layout, setup.spacing_m, setup.blade_width_m)
    margin, centre = pair_margin(layout, pair, setup.spacing_m, setup.blade_width_m)
    target = target_insertion_depth_m(layout.depth_m, setup.carriage_limit_m, setup.reserve_m)
    reasons = set()
    if margin < m_req_m:
        reasons.add("fork_spacing_adjust")
    if (floor > setup.blade_bottom_m - VERTICAL_MARGIN_M + 1e-12
            or ceiling < setup.blade_top_m + VERTICAL_MARGIN_M - 1e-12):
        reasons.add("lift_alignment_needed")
    if load is not None and load.pallet_mass_kg + load.load_mass_kg > PAYLOAD_LIMIT_KG:
        reasons.add("payload_limit")
    assumed_empty = spec.known and load is None
    if not spec.known:
        if (load is None or target < load.com_x_max_m + SUPPORT_LONGITUDINAL_MARGIN_M
                or abs(centre) + load.com_y_max_m > setup.spacing_m / 2 - SUPPORT_LATERAL_MARGIN_M):
            reasons.add("support_unverified")
        if spec.top_style != "boards":
            reasons.add("near_tracking_unavailable")
    ordered = tuple(code for code in REASON_CODES if code in reasons)
    return FitResult(not ordered, ordered, axis, pair, margin, centre, target, assumed_empty)


def danger_threshold_m(fit: FitResult) -> float:
    """T_i (design §3.4): the blade-tip lateral displacement that reaches contact."""
    return fit.min_lateral_margin_m - TRACKING_ALLOWANCE_M
```

- [ ] **Step 4: 설계 §9.3 보충** — §9.3 문단 끝에 덧붙인다:

```markdown
**해석 보충 (2026-10-15, 단계 1A 계획 검토):** ⑤ 의 "작업 지시의 값, 없으면 콘솔 확인"은 **비규격**에 적용한다(비규격은 질량·질량중심이 없으면 ③ 에서 이미 `support_unverified` 다). **알려진 형상은 지시에 질량이 없으면 빈 팔레트로 가정**하고 적재 사유를 내지 않되 결과에 그 가정(`payload_assumed_empty`)을 남긴다 — 그렇지 않으면 사용자 결정 ③(지시 없으면 가장 가까운 팔레트를 자율로)이 알려진 형상에서도 성립하지 않는다. 지시가 질량을 주면 알려진 형상도 그 값으로 검사한다.
```

- [ ] **Step 5: 통과 확인** — Expected: 8 passed
- [ ] **Step 6: 커밋** — `git add src/forklift_core/perception/fork_fit.py tests/unit/perception/test_fork_fit.py docs/design/2026-10-15-pallet-recognition-generality.md` · `git commit -m "perception: fork-fit truth with real blade margins, per-face lanes and layer reach, support precheck, five reason codes; design 9.3 clarification for known-shape payload"`

**검토 A:** 작업 1–3 뒤 Codex — 초점: "알려진 형상 22 상자 일치, 면별 차선·포켓 띠, 날 여유 기준 쌍 선택, fork_fit 의 여유·세로·지지·적재 규칙이 설계 §3.1·§9.2·§9.3(보충 포함)과 같은가, 손상 범위·절단 위치". 수렴까지 반복.

### Task 4: 인스턴스 렌더 `instance_rig`

**Files:**
- Create: `tools/instance_rig.py`
- Test: `tests/unit/test_instance_rig.py`

**Interfaces:**
- Consumes: `tools.scene_rig`(`Box`, `Camera`, `_rays`, `_box_depth`, `_yaw_matrix`, `_rig_intrinsics`, `BACK_WALL_X_M`), `SceneInput`
- Produces: `PERCEPTION_CAMERA`, `BACKGROUND = -1`, `DISTRACTOR = -2`, `RenderedScene`(`scene: SceneInput`(깨끗한 depth, 양자화 없음), `instance_ids: int32[H,W]`, `points_base: float64[H,W,3]`(적중점, 없으면 NaN)), `render_instances(instances, *, distractors=(), camera=PERCEPTION_CAMERA, intrinsics=None, floor=True, back_wall=True, stamp_ns=2_000_000_000) -> RenderedScene`

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/test_instance_rig.py
import numpy as np

from tools import instance_rig, scene_rig


def _cube(x, y, size=0.3):
    return [scene_rig.Box((x, y, size / 2), (size, size, size))]


def test_two_touching_boxes_get_their_own_ids_and_order():
    out = instance_rig.render_instances([_cube(2.0, 0.15), _cube(2.0, -0.15)])
    assert {0, 1} <= set(np.unique(out.instance_ids))
    u0 = np.argwhere(out.instance_ids == 0)[:, 1].mean()
    u1 = np.argwhere(out.instance_ids == 1)[:, 1].mean()
    assert u0 < u1  # +y (left) appears left in the image


def test_depth_matches_scene_rig_without_quantisation():
    boxes = _cube(2.0, 0.0)
    ours = instance_rig.render_instances([boxes]).scene.depth_m
    theirs = scene_rig.render(boxes, camera=instance_rig.PERCEPTION_CAMERA, quantize=False).depth_m
    assert np.array_equal(np.isnan(ours), np.isnan(theirs))
    assert np.allclose(ours[~np.isnan(ours)], theirs[~np.isnan(theirs)])


def test_hit_points_lie_on_the_box_face():
    out = instance_rig.render_instances([_cube(2.0, 0.0)])
    pts = out.points_base[out.instance_ids == 0]
    assert np.allclose(pts[:, 0].min(), 2.0 - 0.15, atol=1e-9)


def test_distractors_are_marked_minus_two():
    assert (instance_rig.render_instances([], distractors=_cube(2.0, 0.0)).instance_ids == -2).any()
```

- [ ] **Step 2: 실패 확인** — `ImportError`
- [ ] **Step 3: 구현**

```python
# tools/instance_rig.py
"""CPU first-hit renderer that also reports the instance and base-frame point each
pixel hit. Reuses tools.scene_rig's slab intersection unchanged (that file's hash
is part of recorded Isaac runs). Depth is clean and unquantised;
forklift_core.perception.depth_noise adds the sensor model afterwards."""

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
    points_base: NDArray[np.float64]


def render_instances(instances: Sequence[Sequence[scene_rig.Box]], *, distractors=(), camera=PERCEPTION_CAMERA,
                     intrinsics=None, floor=True, back_wall=True, stamp_ns=2_000_000_000) -> RenderedScene:
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
    labelled = [(DISTRACTOR, b) for b in distractors] + [(k, b) for k, boxes in enumerate(instances) for b in boxes]
    for label, box in labelled:
        hit = scene_rig._box_depth(origin, rays, np.asarray(box.centre_m, float), box.size_m,
                                   scene_rig._yaw_matrix(box.yaw_rad))
        closer = hit < depth
        depth = np.where(closer, hit, depth)
        ids = np.where(closer, label, ids)
    depth = np.where(np.isfinite(depth), depth, np.nan)
    points = origin + depth[..., None] * rays
    scene = SceneInput(rgb=np.zeros((spec.height, spec.width, 3), dtype=np.uint8), depth_m=depth,
                       intrinsics=spec, base_from_optical=transform, stamp_ns=stamp_ns,
                       clock_domain="ros_sim", source_provenance="synthetic")
    return RenderedScene(scene, ids, points)
```

- [ ] **Step 4: 통과 확인** — Expected: 7 passed
- [ ] **Step 5: 커밋** — `git add tools/instance_rig.py tests/unit/test_instance_rig.py` · `git commit -m "tools: CPU instance renderer on scene_rig's slab intersection (scene_rig untouched) with per-pixel instance ids and hit points"`

### Task 5: 깊이 잡음 모델 `depth_noise`

**Files:**
- Create: `src/forklift_core/perception/depth_noise.py`
- Test: `tests/unit/perception/test_depth_noise.py`

**Interfaces:**
- Produces: `NoiseModel`(`axial_k=0.005`, `edge_jump_m=0.05`, `flying_fraction=0.5`, `registration_shift_px=(2, 0)`, `dropout_fraction=0.01`, `quantize_step_m=0.001`, `seed=0`; `to_json()`), `apply_noise(scene, labels, model) -> (SceneInput, labels)` — 축 잡음 σ = k·z², **깊이 경계에서 더 먼 쪽 화소를 가까운 쪽으로 섞는 flying pixel**, 무효 화소, 1 mm 양자화, RGB·라벨을 depth 격자에 대해 (du, dv) 이동. 입력 불변. 모든 값은 데이터시트 수준의 모델값(실측 아님).

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_depth_noise.py
import numpy as np

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from tools import instance_rig, scene_rig

QUIET = dict(edge_jump_m=1e9, dropout_fraction=0.0, quantize_step_m=0.0, registration_shift_px=(0, 0))


def _wall(x):
    return instance_rig.render_instances([[scene_rig.Box((x, 0.0, 0.5), (0.02, 4.0, 1.0))]], floor=False, back_wall=False)


def test_noise_is_deterministic_and_leaves_input_alone():
    out = _wall(2.0)
    before = out.scene.depth_m.copy()
    a, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=3))
    b, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=3))
    assert np.array_equal(a.depth_m, b.depth_m, equal_nan=True)
    assert np.array_equal(out.scene.depth_m, before, equal_nan=True)


def test_axial_sigma_scales_with_range_squared():
    stds = []
    for x in (1.5, 3.0):
        out = _wall(x)
        noisy, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(**QUIET))
        stds.append(np.nanstd(noisy.depth_m - out.scene.depth_m))
    expected = ((3.0 - 0.76) / (1.5 - 0.76)) ** 2  # optical depth of the wall front at its centre
    assert 0.8 * expected < stds[1] / stds[0] < 1.2 * expected


def test_labels_shift_with_registration():
    out = instance_rig.render_instances([[scene_rig.Box((2.0, 0.0, 0.15), (0.3, 0.6, 0.3))]])
    _, labels = apply_noise(out.scene, out.instance_ids, NoiseModel(registration_shift_px=(3, 0)))
    assert np.argwhere(labels == 0)[:, 1].min() == np.argwhere(out.instance_ids == 0)[:, 1].min() + 3


def test_flying_pixels_move_the_far_side_towards_the_near_side():
    out = instance_rig.render_instances([[scene_rig.Box((2.0, 0.0, 0.15), (0.3, 0.6, 0.3))]], floor=False)
    noisy, _ = apply_noise(out.scene, out.instance_ids,
                           NoiseModel(axial_k=0.0, dropout_fraction=0.0, quantize_step_m=0.0,
                                      registration_shift_px=(0, 0), flying_fraction=1.0))
    changed = np.isfinite(out.scene.depth_m) & (np.abs(noisy.depth_m - out.scene.depth_m) > 1e-9)
    assert changed.any()
    assert np.all(noisy.depth_m[changed] < out.scene.depth_m[changed])  # far pixels pulled nearer
```

- [ ] **Step 2: 실패 확인** — `ModuleNotFoundError`
- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/depth_noise.py
"""A datasheet-level D435i-like depth model for rendered scenes (design §5.3).

Every parameter is a model value, not a measurement: axial sigma k*z^2, mixed
("flying") pixels on the far side of depth edges, random dropouts, rounding, and
a fixed RGB/label shift against the depth grid. Gates are judged on noisy
renders because clean renders make p95 optimistic.
"""

from dataclasses import asdict, dataclass, replace

import numpy as np

from forklift_core.perception.scene_dataset import SceneInput


@dataclass(frozen=True)
class NoiseModel:
    axial_k: float = 0.005
    edge_jump_m: float = 0.05
    flying_fraction: float = 0.5
    registration_shift_px: tuple = (2, 0)
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
    out[max(0, dv):min(h, h + dv), max(0, du):min(w, w + du)] = \
        image[max(0, -dv):min(h, h - dv), max(0, -du):min(w, w - du)]
    return out


def apply_noise(scene: SceneInput, labels, model: NoiseModel):
    rng = np.random.default_rng(model.seed)
    clean = scene.depth_m
    finite = np.isfinite(clean)
    sigma = model.axial_k * np.square(np.where(finite, clean, 0.0))
    depth = np.where(finite, clean + rng.normal(0.0, 1.0, clean.shape) * sigma, clean)
    mix = np.zeros(clean.shape, bool)
    target = np.full(clean.shape, np.nan)
    for axis in (0, 1):
        for step in (1, -1):
            neighbour = np.roll(clean, step, axis=axis)
            edge = finite & np.isfinite(neighbour) & (clean - neighbour > model.edge_jump_m)
            if axis == 0:
                edge[0 if step == 1 else -1, :] = False
            else:
                edge[:, 0 if step == 1 else -1] = False
            fresh = edge & ~mix
            target = np.where(fresh, neighbour, target)
            mix |= edge
    mix &= rng.random(clean.shape) < model.flying_fraction
    t = rng.random(clean.shape)
    depth = np.where(mix, depth + t * (target - depth), depth)
    depth = np.where(rng.random(clean.shape) < model.dropout_fraction, np.nan, depth)
    if model.quantize_step_m > 0:
        depth = np.round(depth / model.quantize_step_m) * model.quantize_step_m
    du, dv = model.registration_shift_px
    return replace(scene, depth_m=depth, rgb=_shift(scene.rgb, du, dv, 0)), _shift(np.asarray(labels), du, dv, -1)
```

- [ ] **Step 4: 통과 확인** — Expected: 4 passed
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/depth_noise.py tests/unit/perception/test_depth_noise.py` · `git commit -m "perception: datasheet-level depth noise model (k*z^2, far-side flying pixels, dropouts, rounding, registration shift)"`

### Task 6: 통계 `recognition_stats`

**Files:**
- Create: `src/forklift_core/perception/recognition_stats.py`
- Test: `tests/unit/perception/test_recognition_stats.py`

**Interfaces:** `cp_upper(failures, n, confidence=0.95) -> float | None`(n = 0 → None), `allowed_failures(n, bound) -> int | None`, `min_scenes(failures, bound) -> int`, `eval_count(total) -> int`

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_recognition_stats.py
import pytest

from forklift_core.perception.recognition_stats import allowed_failures, cp_upper, eval_count, min_scenes


@pytest.mark.parametrize(("bound", "k", "n"), [(0.05, 0, 59), (0.05, 1, 93), (0.05, 2, 124), (0.10, 0, 29),
                                               (0.10, 1, 46), (0.10, 2, 61), (0.15, 0, 19), (0.15, 1, 30),
                                               (0.20, 0, 14), (0.20, 1, 22)])
def test_min_scenes_matches_the_design_table(bound, k, n):
    assert min_scenes(k, bound) == n


def test_known_values():
    assert cp_upper(0, 59) == pytest.approx(0.0495, abs=1e-4)
    assert cp_upper(1, 114) == pytest.approx(0.04094, abs=1e-4)
    assert cp_upper(5, 100) == pytest.approx(0.10225, abs=1e-4)
    assert cp_upper(1, 20) == pytest.approx(0.21611, abs=1e-4)


def test_allowed_failures():
    assert allowed_failures(114, 0.05) == 1 and allowed_failures(114, 0.10) == 5
    assert allowed_failures(0, 0.05) is None and allowed_failures(10, 0.05) is None


def test_empty_population_has_no_bound_and_eval_split_convention():
    assert cp_upper(0, 0) is None
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
    lp, lq = math.log(p), math.log1p(-p)
    return min(1.0, sum(math.exp(math.log(math.comb(n, i)) + i * lp + (n - i) * lq) for i in range(k + 1)))


def cp_upper(failures: int, n: int, confidence: float = 0.95) -> float | None:
    if n == 0:
        return None
    if not 0 <= failures <= n:
        raise ValueError("failures must lie in [0, n]")
    if failures == n:
        return 1.0
    alpha, low, high = 1.0 - confidence, 0.0, 1.0
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

- [ ] **Step 4: 통과 확인** — Expected: 13 passed
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/recognition_stats.py tests/unit/perception/test_recognition_stats.py` · `git commit -m "perception: Clopper-Pearson bounds, allowed failures and scene counts without scipy"`

**검토 B:** 작업 4–6 뒤 Codex — 초점: "인스턴스 렌더·적중점, 잡음의 거리 의존·flying pixel 방향·이동, 통계 수치". 수렴까지 반복.

### Task 7: 정답 계산 `recognition_truth`

**Files:**
- Create: `tools/recognition_truth.py`
- Test: `tests/unit/test_recognition_truth.py`

**Interfaces:**
- Consumes: 작업 1–4
- Produces:
  - `Placement`(`x_m`, `y_m`, `yaw_rad`, `support_z_m`), `placed_boxes(boxes, placement) -> list[scene_rig.Box]`, `instance_boxes(spec, damage) -> list[PalletBox]`
  - `FaceTruth`(`axis`, `sign`(+1 기본 면 / −1 반대 면), `insertion_yaw_rad`, `left_m`, `right_m`, `pocket_width_m: (좌, 우)`, `pocket_height_m`, `accessible`, `exposure`, `fit: dict`, `danger_threshold_m`)
  - `InstanceTruth`(`instance_id`, `spec: dict`(`PalletSpec.to_json()`), `known`, `layer`, `support_z_m`, `support_visible`, `lane_reachable`, `faces`, `eligible`, `damage: list[dict]`; `to_json()`)
  - `scene_truths(instances, setup, *, distractors=(), layers=None) -> tuple[list[InstanceTruth], RenderedScene]` — `instances` 원소는 `(spec, placement, damage, load)`. **전체 렌더 한 번 + 인스턴스별 단독 렌더**, 반환한 렌더를 생성기가 그대로 쓴다.
  - `EXPOSURE_MIN = 0.5`, `FACE_SLAB_M = 0.02`

면 노출(설계 §3.4 "진입면 노출"): 적중점을 그 인스턴스의 팔레트 좌표로 옮겨, 면 평면(x 면은 x = ∓깊이/2, y 면은 y = ∓폭/2)에서 `FACE_SLAB_M` 이내인 그 인스턴스 화소를 센다. 노출 = 장면에서의 개수 / 단독 렌더에서의 개수. `eligible` = 접근 가능한 면 중 노출 ≥ 0.5 인 것이 있음. `support_visible` = 1 층(바닥)이거나 바로 아래 인스턴스 화소가 장면에 하나라도 있음. 끼움 정답은 **면마다** `fork_fit(spec, axis, setup, load, support_z_m=…)`.

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/test_recognition_truth.py
import math
from pathlib import Path

import pytest

from forklift_core.perception.fork_fit import ForkSetup
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_model import KNOWN_ENTRY_AXES, spec_from_geometry
from tools import recognition_truth as rt
from tools import scene_rig

ROOT = Path(__file__).resolve().parents[2]


def _spec(name):
    g = load_pallet_geometry(ROOT / f"config/pallet_geometry_{name}.yaml")
    return g, spec_from_geometry(g, entry_axes=KNOWN_ENTRY_AXES[name])


def _truth(items, spacing=0.3725, **kw):
    return rt.scene_truths(items, ForkSetup(spacing), **kw)[0]


def test_x_face_pockets_match_scene_rig_true_pockets():
    g, spec = _spec("epal6")
    t = _truth([(spec, rt.Placement(2.5, 0.2, 0.1, 0.0), (), None)])[0]
    front = next(f for f in t.faces if f.axis == "x" and f.sign == 1)
    truth = scene_rig.true_pockets(g, x_m=2.5, y_m=0.2, yaw_rad=0.1)
    assert front.left_m == pytest.approx(tuple(truth[0]), abs=1e-9)
    assert front.right_m == pytest.approx(tuple(truth[1]), abs=1e-9)
    back = next(f for f in t.faces if f.axis == "x" and f.sign == -1)
    assert front.accessible and not back.accessible
    assert back.insertion_yaw_rad == pytest.approx(0.1 - math.pi)


def test_t11_faces_at_45_degrees_and_y_face_height():
    _, spec = _spec("t11_06")
    t = _truth([(spec, rt.Placement(2.5, 0.0, math.pi / 4, 0.0), (), None)], spacing=0.290)[0]
    assert len(t.faces) == 4 and sum(f.accessible for f in t.faces) == 2
    y_face = next(f for f in t.faces if f.axis == "y")
    assert y_face.left_m[2] == pytest.approx(0.045) and y_face.pocket_height_m == pytest.approx(0.060)


def test_stacked_layer_raises_the_pockets_and_needs_lift():
    _, spec = _spec("epal6")
    lower = (spec, rt.Placement(2.5, 0.0, 0.0, 0.0), (), None)
    upper = (spec, rt.Placement(2.5, 0.0, 0.0, 0.144), (), None)
    _, up_t = _truth([lower, upper], layers=[0, 1])
    front = up_t.faces[0]
    assert front.left_m[2] == pytest.approx(0.144 + 0.022 + 0.039)
    assert front.fit["reasons"] == ["lift_alignment_needed"] and up_t.lane_reachable
    assert up_t.support_visible


def test_touching_pallets_keep_their_own_face_exposure():
    _, spec = _spec("epal6")
    a, b = _truth([(spec, rt.Placement(2.5, 0.4, 0.0, 0.0), (), None),
                   (spec, rt.Placement(2.5, -0.4, 0.0, 0.0), (), None)])
    for t in (a, b):
        front = next(f for f in t.faces if f.accessible)
        assert 0.95 < front.exposure <= 1.0 and t.eligible


def test_a_low_screen_in_front_cuts_face_exposure_below_half():
    _, spec = _spec("epal6")
    screen = [scene_rig.Box((2.5 - 0.3 - 0.05, 0.0, 0.06), (0.01, 1.0, 0.12))]
    t = _truth([(spec, rt.Placement(2.5, 0.0, 0.0, 0.0), (), None)], distractors=screen)[0]
    front = next(f for f in t.faces if f.accessible)
    assert front.exposure < 0.5 and not t.eligible
```

- [ ] **Step 2: 실패 확인** — `ImportError`
- [ ] **Step 3: 구현**

```python
# tools/recognition_truth.py
"""Ground truth for recognition scenes: per-face pockets, face exposure, fit truth."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import asdict, dataclass

import numpy as np

from forklift_core.perception.fork_fit import ForkSetup, danger_threshold_m, fork_fit, lane_reachable
from forklift_core.perception.pallet_damage import apply_damage
from forklift_core.perception.pallet_model import PalletSpec, face_layout, spec_boxes
from tools import instance_rig, scene_rig

EXPOSURE_MIN = 0.5
FACE_SLAB_M = 0.02


def _wrap(angle: float) -> float:
    wrapped = (angle + math.pi) % (2 * math.pi) - math.pi
    return math.pi if wrapped == -math.pi else wrapped


@dataclass(frozen=True)
class Placement:
    x_m: float
    y_m: float
    yaw_rad: float
    support_z_m: float


def instance_boxes(spec: PalletSpec, damage):
    return apply_damage(spec_boxes(spec), damage)


def placed_boxes(boxes, placement: Placement) -> list[scene_rig.Box]:
    lifted = [scene_rig.Box((b.centre_m[0], b.centre_m[1], b.centre_m[2] + placement.support_z_m), tuple(b.size_m))
              for b in boxes]
    return scene_rig.place(lifted, x_m=placement.x_m, y_m=placement.y_m, yaw_rad=placement.yaw_rad)


@dataclass(frozen=True)
class FaceTruth:
    axis: str
    sign: int
    insertion_yaw_rad: float
    left_m: tuple
    right_m: tuple
    pocket_width_m: tuple
    pocket_height_m: float
    accessible: bool
    exposure: float
    fit: dict
    danger_threshold_m: float


@dataclass(frozen=True)
class InstanceTruth:
    instance_id: int
    spec: dict
    known: bool
    layer: int
    support_z_m: float
    support_visible: bool
    lane_reachable: bool
    faces: tuple
    eligible: bool
    damage: list

    def to_json(self) -> dict:
        obj = asdict(self)
        obj["faces"] = [asdict(f) for f in self.faces]
        return obj


def _local(points, placement: Placement):
    c, s = math.cos(placement.yaw_rad), math.sin(placement.yaw_rad)
    d = points - np.array([placement.x_m, placement.y_m, placement.support_z_m])
    return np.stack((c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1], d[..., 2]), axis=-1)


def _face_mask(local, spec: PalletSpec, axis: str, sign: int):
    if axis == "x":
        plane = -spec.overall_depth_m / 2 if sign == 1 else spec.overall_depth_m / 2
        return np.abs(local[..., 0] - plane) <= FACE_SLAB_M
    plane = -spec.overall_width_m / 2 if sign == 1 else spec.overall_width_m / 2
    return np.abs(local[..., 1] - plane) <= FACE_SLAB_M


def _faces(spec, placement, setup, load, scene_local, scene_ids, k, solo_local, solo_ids):
    camera = np.asarray(instance_rig.PERCEPTION_CAMERA.xyz_m, dtype=float)
    out = []
    for axis in spec.entry_axes:
        layout = face_layout(spec, axis)
        fit = fork_fit(spec, axis, setup, load, support_z_m=placement.support_z_m)
        intervals = [layout.opening_intervals_m[i] for i in fit.pair]
        z0, z1 = layout.pocket_band_m
        z = placement.support_z_m + (z0 + z1) / 2
        base = placement.yaw_rad + (0.0 if axis == "x" else math.pi / 2)
        for sign in (1, -1):
            yaw = _wrap(base + (0.0 if sign == 1 else math.pi))
            a = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            left_axis = np.array([-a[1], a[0], 0.0])
            face_centre = np.array([placement.x_m, placement.y_m, 0.0]) - a * layout.depth_m / 2
            # The opposite face sees the layout mirrored: its left axis is the nominal right.
            (r_lat, r_w), (l_lat, l_w) = sorted((sign * (lo + hi) / 2, hi - lo) for lo, hi in intervals)
            left, right = face_centre + left_axis * l_lat, face_centre + left_axis * r_lat
            seen = int((_face_mask(scene_local, spec, axis, sign) & (scene_ids == k)).sum())
            solo = int((_face_mask(solo_local, spec, axis, sign) & (solo_ids == 0)).sum())
            out.append(FaceTruth(
                axis, sign, yaw, (float(left[0]), float(left[1]), z), (float(right[0]), float(right[1]), z),
                (l_w, r_w), z1 - z0, float(-a @ (camera - face_centre)) > 0,
                seen / solo if solo else 0.0, fit.to_json(), danger_threshold_m(fit),
            ))
    return tuple(out)


def scene_truths(instances, setup: ForkSetup, *, distractors: Sequence[scene_rig.Box] = (), layers=None):
    placed = [placed_boxes(instance_boxes(s, d), p) for s, p, d, _ in instances]
    rendered = instance_rig.render_instances(placed, distractors=distractors)
    layers = [0] * len(instances) if layers is None else list(layers)
    truths = []
    for k, (spec, placement, damage, load) in enumerate(instances):
        solo = instance_rig.render_instances([placed[k]], floor=False, back_wall=False)
        faces = _faces(spec, placement, setup, load, _local(rendered.points_base, placement), rendered.instance_ids,
                       k, _local(solo.points_base, placement), solo.instance_ids)
        below = [m for m in range(len(instances)) if layers[m] == layers[k] - 1]
        support_visible = layers[k] == 0 or any((rendered.instance_ids == m).any() for m in below)
        first = next((f for f in faces if f.accessible), faces[0])
        truths.append(InstanceTruth(
            k, spec.to_json(), spec.known, layers[k], placement.support_z_m, support_visible,
            lane_reachable(spec, first.axis, setup, support_z_m=placement.support_z_m), faces,
            any(f.accessible and f.exposure >= EXPOSURE_MIN for f in faces), [d.to_json() for d in damage],
        ))
    return truths, rendered
```

- [ ] **Step 4: 통과 확인** — Expected: 5 passed
- [ ] **Step 5: 커밋** — `git add tools/recognition_truth.py tests/unit/test_recognition_truth.py` · `git commit -m "tools: recognition ground truth -- per-face pockets and fit, entry-face exposure against solo renders, support visibility and layer reach"`

### Task 8: 장면 조리법 `recognition_scenes`

**Files:**
- Create: `tools/recognition_scenes.py`
- Test: `tests/unit/test_recognition_scenes.py`

**Interfaces:**
- Consumes: 작업 1–3·7, `tools.measure_pocket_evidence.structure`, `scene_rig.boxes_interpenetrate`, `eval_count`
- Produces: `CASES`, `SEED_BANDS = {"eval_pool": 1_000_000, "train": 5_000_000}`, `BAND_WIDTH = 1_000_000`, `S7_VIOLATIONS`(사유 다섯), `InstanceRecipe`(`spec`, `placement`, `damage`, `layer`), `Instruction`(`kind`(평가용 정답), `target`(평가용 정답), `x_m`, `y_m`, `radius_m`, `layer`, `load`), `SceneRecipe`(`scene_id`, `case`, `shape_key`, `split`, `rank`, `seed`, `fork_spacing_m`, `touching: bool`, `instances`, `distractors`, `instruction`), `recipe_boxes(recipe, k)`, `generate(case, shape_key, count, *, band, first_seed)`, `assert_disjoint(*lists)`, `catalogue()`, `catalogue_digest(recipes) -> str`

규칙(분할 안 순위 r 기준, 결정적):
- **지시 종류**: S2·S3·S4 → r % 5 ∈ {0, 1} 애매함, {2, 3} 애매하지 않음, 4 없음. S1·S6 → r % 4 == 3 없음, 나머지 애매하지 않음. S5 → 기반 규칙(r % 3 → S1·S2·S3)을 따른다. **S7 → 항상 애매하지 않음**(적재 선언이 지시로만 들어오므로). N → 없음.
- **지시 내용**: S2·S3 의 애매함 = 반경이 대상과 가장 가까운 이웃까지 + 0.1 m, 층 없음. 애매하지 않음 = 반경 min(0.3, 이웃 거리/2), 층 없음. **S4 의 애매하지 않음 = 반경 0.3 m + 대상 절대 층, 애매함 = 반경 0.3 m + 층 없음**(같은 수평 위치의 층이 모두 반경 안).
- **S2 맞닿음**: r % 4 == 0 이면 같은 yaw·간격 0·엇갈림 없음(`touching=True`). 나머지는 간격 0.01–0.30 m, yaw ±5°, 엇갈림 ±0.15 m(삽입축 방향). 줄지어 놓는 축 = 팔레트의 왼쪽 축.
- **S3**: 뒤 팔레트를 삽입축 방향으로 (간격 0.05–1.0 m + 깊이)만큼, 가로는 왼쪽 축 방향 ±0.2 m.
- **S7 층화**: r % 2 == 0 → 집을 수 있는 계열, 아니면 위반 계열 `S7_VIOLATIONS[(r // 2) % 5]`. 집을 수 있는 계열은 시험 간격 s ∈ {0.3725, 0.290} 에 포켓 중심을 ±s/2 에 두고 개구 폭 w ∈ [0.195, min(0.28, s − 0.06)](여유 (w − 0.055)/2 ≥ 0.07 ≥ M_req), 가운데 기둥 c = s − w, 바깥 기둥 o ∈ [0.06, 0.20], 전체 폭이 0.60 m 미만이면 o 를 늘린다; 깊이 0.50–0.58 m, 판 상판, 천장 ≥ 0.07 m(받침 0.010–0.025 + 개구 0.06–0.12), 중앙 하중(x_c = 깊이/2, y_c = 0), 질량 합 ≤ 10 kg. r % 4 == 0 이면 s = 0.3725, w ∈ [0.195, 0.2025] 로 가운데를 두 기둥 + 0.05 m 좁은 개구로 나눈 **4 기둥** 형상(쌍 선택 시험). 위반 계열은 집을 수 있는 계열에서 하나만 바꾼다 — `fork_spacing_adjust`: w ∈ [0.10, 0.18]; `lift_alignment_needed`: 받침 0.010 · 개구 0.040–0.045; `support_unverified`: 깊이 0.62–1.20 m; `payload_limit`: 질량 합 10.5–14 kg; `near_tracking_unavailable`: 평평한 상판.
- **N**: `structure(g, kind)` 의 grounded·shelf·blocks + 플라스틱 크레이트 더미(상자 2–4개).
- 겹침 검사는 S4 의 층끼리를 빼고 모든 물체 쌍에 한다. 실패하면 같은 배치 스트림으로 다시 뽑는다(최대 200회).

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/test_recognition_scenes.py
import itertools

import pytest

from forklift_core.perception.fork_fit import ForkSetup
from forklift_core.perception.recognition_stats import eval_count
from tools import recognition_scenes as rs
from tools import scene_rig
from tools.recognition_truth import scene_truths


@pytest.fixture(scope="module")
def catalogue():
    return rs.catalogue()


def test_catalogue_counts_and_instruction_quotas(catalogue):
    assert len(catalogue) == 1020
    for shape in ("epal6", "t11_06"):
        ev = [r for r in catalogue if r.shape_key == shape and r.split == "eval"]
        assert len(ev) == 6 * eval_count(61) == 114
        kinds = [r.instruction.kind for r in ev]
        assert kinds.count("unambiguous") >= 29 and kinds.count("ambiguous") >= 15
        assert kinds.count("unambiguous") + kinds.count("ambiguous") >= 59
        assert sum(r.touching for r in ev if r.case == "S2") >= 4
    assert sum(r.case == "S7" and r.split == "eval" for r in catalogue) == 59
    assert sum(r.case == "N" and r.split == "eval" for r in catalogue) == 29


def test_generation_is_deterministic():
    a = rs.generate("S2", "epal6", 5, band="eval_pool", first_seed=1_000_000)
    b = rs.generate("S2", "epal6", 5, band="eval_pool", first_seed=1_000_000)
    assert a == b and rs.catalogue_digest(a) == rs.catalogue_digest(b)


def test_seed_overlap_is_detected():
    a = rs.generate("S1", "epal6", 3, band="eval_pool", first_seed=1_000_000)
    rs.assert_disjoint(a, rs.generate("S1", "epal6", 3, band="train", first_seed=5_000_000))
    with pytest.raises(ValueError, match="overlap"):
        rs.assert_disjoint(a, rs.generate("S1", "epal6", 3, band="eval_pool", first_seed=1_000_001))


def test_first_seed_must_lie_in_its_band():
    with pytest.raises(ValueError, match="band"):
        rs.generate("S1", "epal6", 3, band="train", first_seed=1_000_000)


@pytest.mark.parametrize("case", ["S2", "S3", "S6", "S7"])
def test_objects_do_not_interpenetrate(case):
    shape = "nonstandard" if case == "S7" else "t11_06"
    for recipe in rs.generate(case, shape, 8, band="eval_pool", first_seed=1_000_000):
        groups = [rs.recipe_boxes(recipe, k) for k in range(len(recipe.instances))] + [list(recipe.distractors)]
        for a, b in itertools.combinations(groups, 2):
            assert not any(scene_rig.boxes_interpenetrate(x, y) for x in a for y in b)


def test_s4_instructions_carry_the_layer_only_when_unambiguous():
    for r in rs.generate("S4", "epal6", 10, band="eval_pool", first_seed=1_000_000):
        if r.instruction.kind == "unambiguous":
            assert r.instruction.layer == r.instances[r.instruction.target].layer
        if r.instruction.kind == "ambiguous":
            assert r.instruction.layer is None and r.instruction.radius_m == pytest.approx(0.3)


def test_s7_eval_is_stratified(catalogue):
    verdicts = []
    for r in (r for r in catalogue if r.case == "S7" and r.split == "eval"):
        truths, _ = scene_truths([(i.spec, i.placement, i.damage, r.instruction.load) for i in r.instances],
                                 ForkSetup(r.fork_spacing_m))
        verdicts.append(tuple(truths[0].faces[0].fit["reasons"]))
    assert sum(v == () for v in verdicts) >= 25
    for code in rs.S7_VIOLATIONS:
        assert sum(code in v for v in verdicts) >= 4, code
```

- [ ] **Step 2: 실패 확인** — `ImportError`
- [ ] **Step 3: 구현**

```python
# tools/recognition_scenes.py
"""Deterministic scene recipes for the recognition generality evaluation (design §5.1)."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass
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
S7_VIOLATIONS = ("fork_spacing_adjust", "lift_alignment_needed", "support_unverified",
                 "payload_limit", "near_tracking_unavailable")


@lru_cache(maxsize=None)
def _geometry(shape_key: str):
    return load_pallet_geometry(ROOT / f"config/pallet_geometry_{shape_key}.yaml")


@lru_cache(maxsize=None)
def known_spec(shape_key: str) -> PalletSpec:
    return spec_from_geometry(_geometry(shape_key), entry_axes=KNOWN_ENTRY_AXES[shape_key])


@dataclass(frozen=True)
class InstanceRecipe:
    spec: PalletSpec
    placement: Placement
    damage: tuple
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
    rank: int
    seed: int
    fork_spacing_m: float
    touching: bool
    instances: tuple
    distractors: tuple
    instruction: Instruction


def recipe_boxes(recipe: SceneRecipe, k: int) -> list[scene_rig.Box]:
    inst = recipe.instances[k]
    return placed_boxes(instance_boxes(inst.spec, inst.damage), inst.placement)


def _clear(groups, candidate):
    return not any(scene_rig.boxes_interpenetrate(a, b) for g in groups for a in g for b in candidate)


def _base_pose(rng):
    return 0.75 + rng.uniform(1.1, 4.0), rng.uniform(-1.0, 1.0), rng.uniform(-math.radians(30), math.radians(30))


def _axes(yaw):
    a = np.array([math.cos(yaw), math.sin(yaw)])
    return a, np.array([-a[1], a[0]])


def _damage_ops(rng, spec) -> tuple:
    kind = str(rng.choice(["top_board_missing", "top_board_broken", "block_corner_chipped", "block_split"]))
    if kind == "top_board_missing":
        picks = sorted(int(i) for i in rng.choice(spec.top_board_count, size=int(rng.integers(1, 3)), replace=False))
        return tuple(DamageOp(kind, f"top_board_{i}", ()) for i in picks)
    if kind == "top_board_broken":
        return (DamageOp(kind, f"top_board_{int(rng.integers(spec.top_board_count))}",
                         (("fraction", float(rng.uniform(0.2, 0.6))), ("keep", float(rng.integers(2))))),)
    col = int(rng.integers(len(spec.column_widths_m)))
    if kind == "block_corner_chipped":
        return (DamageOp(kind, f"block_x0_y{col}", (("depth", float(rng.uniform(0.01, 0.04))),
                                                     ("width", float(rng.uniform(0.01, 0.04))),
                                                     ("side", float(rng.integers(2))))),)
    return (DamageOp(kind, f"block_x0_y{col}", (("gap", float(rng.uniform(0.005, 0.015))),)),)


def _s7_spec(rng, rank: int, index: int):
    """Returns (spec, spacing, load). Even ranks pickable, odd ranks violate one reason."""
    violation = None if rank % 2 == 0 else S7_VIOLATIONS[(rank // 2) % 5]
    four = violation is None and rank % 4 == 0
    s = 0.3725 if four else float(rng.choice([0.3725, 0.290]))
    if violation == "fork_spacing_adjust":
        w = float(rng.uniform(0.10, 0.18))
    elif four:
        w = float(rng.uniform(0.195, 0.2025))
    else:
        w = float(rng.uniform(0.195, min(0.28, s - 0.06)))
    c = s - w
    o = float(rng.uniform(0.06, 0.20))
    width = 2 * o + 2 * w + c
    if width < 0.60:
        o += (0.60 - width) / 2
        width = 0.60
    if four:
        inner = (c - 0.05) / 2
        widths = (o, inner, inner, o)
        left = -width / 2
        centres = (left + o / 2, left + o + w + inner / 2, left + o + w + inner + 0.05 + inner / 2, width / 2 - o / 2)
    else:
        widths = (o, c, o)
        centres = (-(c / 2 + w + o / 2), 0.0, c / 2 + w + o / 2)
    depth = float(rng.uniform(0.62, 1.20)) if violation == "support_unverified" else float(rng.uniform(0.50, 0.58))
    row = float(min(rng.uniform(0.08, 0.15), depth / 4))
    lift_case = violation == "lift_alignment_needed"
    deck = 0.010 if lift_case else float(rng.uniform(0.010, 0.025))
    opening = float(rng.uniform(0.040, 0.045)) if lift_case else float(rng.uniform(0.06, 0.12))
    board = float(rng.uniform(0.08, 0.14))
    spec = PalletSpec(
        name=f"nonstandard_{index}", known=False, overall_width_m=width, overall_depth_m=depth,
        deck_bottom_m=deck, opening_height_m=opening, stringer_m=float(rng.uniform(0.012, 0.025)),
        top_thickness_m=float(rng.uniform(0.012, 0.025)), column_centres_y_m=centres, column_widths_m=widths,
        row_centres_x_m=(-(depth - row) / 2, 0.0, (depth - row) / 2), row_depth_m=row,
        top_style="flat" if violation == "near_tracking_unavailable" else "boards",
        top_board_count=max(2, min(9, int(width / (board * 1.4)))), top_board_width_m=board, entry_axes=("x",),
    )
    if violation == "payload_limit":
        pallet, load_mass = float(rng.uniform(3.0, 5.0)), float(rng.uniform(7.5, 9.0))
    else:
        pallet, load_mass = float(rng.uniform(1.0, 4.0)), float(rng.uniform(0.0, 6.0))
    return spec, s, LoadDeclaration(pallet, load_mass, depth / 2, 0.0)


def _instruction_kind(rule: str, rank: int) -> str:
    if rule in ("S2", "S3", "S4"):
        return ("ambiguous", "ambiguous", "unambiguous", "unambiguous", "none")[rank % 5]
    if rule in ("S1", "S6"):
        return "none" if rank % 4 == 3 else "unambiguous"
    if rule == "S7":
        return "unambiguous"
    return "none"


def _place_known(rule, rng, spec, touching):
    x, y, yaw = _base_pose(rng)
    a, n = _axes(yaw)
    count = int(rng.integers(2, 4)) if rule in ("S2", "S3", "S4") else 1
    out = []
    for k in range(count):
        if rule == "S2":
            gap = 0.0 if touching else float(rng.uniform(0.01, 0.30))
            lateral = (k - (count - 1) / 2) * (spec.overall_width_m + gap)
            stagger = 0.0 if touching or rng.random() < 0.5 else float(rng.uniform(-0.15, 0.15))
            pos = np.array([x, y]) + a * stagger + n * lateral
            jitter = 0.0 if touching else float(rng.uniform(-0.087, 0.087))
            out.append((Placement(float(pos[0]), float(pos[1]), yaw + jitter, 0.0), 0))
        elif rule == "S3":
            behind = sum(float(rng.uniform(0.05, 1.0)) + spec.overall_depth_m for _ in range(k))
            side = float(rng.uniform(-0.2, 0.2)) if k else 0.0
            pos = np.array([x, y]) + a * behind + n * side
            out.append((Placement(float(pos[0]), float(pos[1]), yaw, 0.0), 0))
        elif rule == "S4":
            off = rng.uniform(-0.03, 0.03, size=2) if k else np.zeros(2)
            dyaw = float(rng.uniform(-0.052, 0.052)) if k else 0.0
            out.append((Placement(x + float(off[0]), y + float(off[1]), yaw + dyaw, k * spec.overall_height_m), k))
        else:
            out.append((Placement(x, y, yaw, 0.0), 0))
    return out


def _distractors_s6(rng, spec, base):
    x, y, yaw = base
    found = []
    for _ in range(int(rng.integers(2, 6))):
        kind = str(rng.choice(["rack", "crate", "box", "wall"]))
        dx = float(rng.uniform(0.3, 1.5)) * float(rng.choice([-1, 1]))
        dy = float(rng.uniform(0.6, 1.6)) * float(rng.choice([-1, 1]))
        if kind == "wall":
            a, _ = _axes(yaw)
            pos = np.array([x, y]) + a * (spec.overall_depth_m / 2 + float(rng.uniform(0.05, 0.6)) + 0.025)
            cand = [scene_rig.Box((float(pos[0]), float(pos[1]), 0.6), (0.05, 2.0, 1.2), yaw)]
        elif kind == "rack":
            cand = scene_rig.place(measure_pocket_evidence.structure(_geometry("epal6"), "shelf"),
                                   x_m=x + dx, y_m=y + dy, yaw_rad=yaw)
        else:
            size = (0.45, 0.66, 0.19) if kind == "crate" else (0.5, 0.5, 0.25)
            cand = [scene_rig.Box((x + dx, y + dy, size[2] / 2), size, float(rng.uniform(-math.pi, math.pi)))]
        found.append(cand)
    if rng.random() < 0.3:
        size = (spec.overall_depth_m, spec.overall_width_m, float(rng.uniform(0.2, 0.5)))
        found.append([scene_rig.Box((x, y, spec.overall_height_m + size[2] / 2), size, yaw)])
    return found


def _scene(case, shape_key, seed, rank):
    place_seq, instr_seq, _ = np.random.SeedSequence(seed).spawn(3)
    rng = np.random.default_rng(place_seq)
    rule = ("S1", "S2", "S3")[rank % 3] if case == "S5" else case
    touching = rule == "S2" and rank % 4 == 0
    for _ in range(MAX_TRIES):
        if case == "N":
            x, y, yaw = _base_pose(rng)
            kind = ("grounded", "shelf", "blocks", "crates")[rank % 4]
            if kind == "crates":
                boxes = [scene_rig.Box((x, y, 0.095 + 0.19 * i), (0.45, 0.66, 0.19), yaw)
                         for i in range(int(rng.integers(2, 5)))]
            else:
                boxes = scene_rig.place(measure_pocket_evidence.structure(_geometry(("epal6", "t11_06")[rank % 2]), kind),
                                        x_m=x, y_m=y, yaw_rad=yaw)
            return [], boxes, 0.3725, rule, touching, None, instr_seq
        load = None
        if case == "S7":
            spec, spacing, load = _s7_spec(rng, rank, seed)
            x, y, yaw = _base_pose(rng)
            places = [(Placement(x, y, yaw, 0.0), 0)]
        else:
            spec, spacing = known_spec(shape_key), TEST_FORK_SPACING_M[shape_key]
            places = _place_known(rule, rng, spec, touching)
        instances, groups, ok = [], [], True
        for k, (p, layer) in enumerate(places):
            damage = _damage_ops(rng, spec) if case == "S5" and k == 0 else ()
            boxes = placed_boxes(instance_boxes(spec, damage), p)
            if rule != "S4" and not _clear(groups, boxes):
                ok = False
                break
            instances.append(InstanceRecipe(spec, p, damage, layer))
            groups.append(boxes)
        if not ok:
            continue
        distractors = []
        if rule == "S4" and rng.random() < 0.5:
            top = instances[-1].placement
            size = tuple(float(v) for v in (rng.uniform(0.4, 0.6), rng.uniform(0.4, 0.6), rng.uniform(0.2, 0.4)))
            distractors = [scene_rig.Box((top.x_m, top.y_m, top.support_z_m + spec.overall_height_m + size[2] / 2),
                                         size, top.yaw_rad)]
        if case == "S6":
            p0 = instances[0].placement
            for cand in _distractors_s6(rng, spec, (p0.x_m, p0.y_m, p0.yaw_rad)):
                if _clear(groups + [distractors], cand):
                    distractors.extend(cand)
        return instances, distractors, spacing, rule, touching, load, instr_seq
    raise RuntimeError(f"{case} seed {seed}: no non-overlapping placement in {MAX_TRIES} tries")


def _instruction(rule, kind, instances, load, rng):
    if kind == "none" or not instances:
        return Instruction("none", None, 0.0, 0.0, 0.0, None, load)
    target = int(rng.integers(len(instances)))
    t = instances[target].placement
    others = [math.hypot(i.placement.x_m - t.x_m, i.placement.y_m - t.y_m) for k, i in enumerate(instances) if k != target]
    if rule == "S4":
        layer = instances[target].layer if kind == "unambiguous" else None
        return Instruction(kind, target, t.x_m, t.y_m, 0.3, layer, load)
    if kind == "ambiguous":
        return Instruction(kind, target, t.x_m, t.y_m, min(others) + 0.1, None, load)
    radius = 0.3 if not others else min(0.3, min(others) / 2)
    return Instruction(kind, target, t.x_m, t.y_m, radius, None, load)


def generate(case: str, shape_key: str, count: int, *, band: str, first_seed: int) -> list[SceneRecipe]:
    if case not in CASES:
        raise ValueError(f"unknown case {case!r}")
    start = SEED_BANDS[band]
    if first_seed < start or first_seed + count > start + BAND_WIDTH:
        raise ValueError(f"seeds {first_seed}+{count} leave the {band} band")
    order = np.random.default_rng(np.random.SeedSequence([first_seed, 0x5E1F])).permutation(count)
    eval_set = set(order[: eval_count(count)].tolist()) if band != "train" else set()
    seen, recipes = {"eval": 0, "dev": 0, "train": 0}, []
    for i in range(count):
        split = "train" if band == "train" else ("eval" if i in eval_set else "dev")
        rank = seen[split]
        seen[split] += 1
        seed = first_seed + i
        instances, distractors, spacing, rule, touching, load, instr_seq = _scene(case, shape_key, seed, rank)
        kind = _instruction_kind(rule, rank)
        if kind == "ambiguous" and len(instances) < 2:
            kind = "unambiguous"
        instruction = _instruction(rule, kind, instances, load, np.random.default_rng(instr_seq))
        recipes.append(SceneRecipe(f"{case.lower()}_{shape_key}_{seed}", case, shape_key, split, rank, seed,
                                   spacing, touching, tuple(instances), tuple(distractors), instruction))
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


def catalogue_digest(recipes) -> str:
    """sha256 over the recipes at 12 significant digits, to compare machines."""
    def norm(value):
        if isinstance(value, float):
            return float(f"{value:.12g}")
        if isinstance(value, (list, tuple)):
            return [norm(v) for v in value]
        if isinstance(value, dict):
            return {k: norm(v) for k, v in value.items()}
        return value
    payload = json.dumps([norm(asdict(r)) for r in recipes], sort_keys=True, default=str)
    return hashlib.sha256(payload.encode()).hexdigest()
```

- [ ] **Step 4: 통과 확인** — `git add tools/recognition_scenes.py tests/unit/test_recognition_scenes.py`, Run(ws1, 백그라운드): `FORKLIFT_REPO=/home/light/school/2-2/embedded-design/forklift-wt-perception ~/bin/forklift-ws1-pytest tests/unit/test_recognition_scenes.py -q -p no:cacheprovider -W error` · Expected: 10 passed. S7 층화 시험이 실패하면 사유별 개수를 출력해 생성 규칙을 고친다(시험 문턱은 바꾸지 않는다).
- [ ] **Step 5: 커밋** — `git commit -m "tools: deterministic recognition scene recipes -- rank-assigned instructions and touching rows, S4 layer instructions, stratified S7 (pickable vs one violation each), split seed streams, catalogue digest; 1,020 scenes"`

### Task 9: 데이터 계약 판 2 `recognition_dataset` + ADR 0004 D5 개정

**Files:**
- Create: `src/forklift_core/perception/recognition_dataset.py`
- Modify: `docs/decisions/0004-simulation-engine-and-insertion-depth.md` (D5 절 끝)
- Test: `tests/unit/perception/test_recognition_dataset.py`

**Interfaces:**
- Consumes: `SceneInput`, `_read_json`, `_read_png`, `_require_fields`, `decode_depth_mm`, `intrinsics_from_camera_info`(`scene_dataset.py`), `rotation_matrix_from_quaternion_xyzw`
- Produces: `SCHEMA_VERSION = 2`, `TF_ORIGINS`, `ORIGIN_SOURCE = {"rig_render_pose": "rig", "isaac_render_pose": "isaac"}`, `RecognitionInput`(`scene`, `instruction: dict`), `RecognitionSample`(`input`, `origin`, `meta`, `truth`, `instruction`, `instance_ids`), `write_recognition_scene(scene_dir, scene, instance_ids, *, origin, meta, truth, instruction)`, `read_meta(scene_dir) -> dict`(scene.json 만), `load_recognition_input(scene_dir) -> RecognitionInput`, `load_recognition_sample(scene_dir) -> RecognitionSample`, `scene_digest(scene_dir) -> str`(장면 디렉터리의 모든 파일 sha256)

파일: `rgb.png`, `depth_mm.png`, `depth_meta.json`, `camera_info.json`, `tf.json`, `instances.png`(uint16: 0 없음, k+1 인스턴스, 65535 방해물), `scene.json`(`schema_version: 2`, 메타), `instruction.json`(**시스템 입력만** — `present`, `x_m`, `y_m`, `radius_m`, `layer`, `load`), `truth.json`(인스턴스 정답 + `instruction_truth: {kind, target}`). 입력 로더는 `instruction.json` 까지 읽고 `truth.json`·`instances.png` 는 읽지 않는다. 로더는 판 1 과 같은 검증(CameraInfo·장면 시각 일치, CameraInfo 프레임 = TF source, RGB·depth 크기 = CameraInfo; 표본 로더는 마스크 크기도)을 하고, `origin` 과 `meta.render_source` 의 짝(`ORIGIN_SOURCE`)을 강제한다.

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_recognition_dataset.py
import json
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform
from forklift_core.perception import recognition_dataset as rd
from tools import instance_rig, scene_rig

META = {"scene_id": "s", "case": "S1", "shape_key": "epal6", "split": "dev", "render_source": "rig", "noise": None}


def _write(tmp_path, origin="rig_render_pose", meta=META, scene=None):
    out = instance_rig.render_instances([[scene_rig.Box((2.0, 0.0, 0.15), (0.3, 0.6, 0.3))]])
    rd.write_recognition_scene(tmp_path, scene or out.scene, out.instance_ids, origin=origin, meta=meta,
                               truth={"instances": [], "instruction_truth": {"kind": "none", "target": None}},
                               instruction={"present": False})
    return out


def test_round_trip(tmp_path):
    out = _write(tmp_path)
    sample = rd.load_recognition_sample(tmp_path)
    finite = np.isfinite(out.scene.depth_m)
    assert np.nanmax(np.abs(sample.input.scene.depth_m[finite] - out.scene.depth_m[finite])) <= 0.0005 + 1e-12
    assert np.array_equal(sample.instance_ids, out.instance_ids) and sample.origin == "rig_render_pose"
    assert json.loads((tmp_path / "scene.json").read_text())["schema_version"] == 2


def test_input_loader_reads_no_truth(tmp_path):
    _write(tmp_path)
    (tmp_path / "truth.json").unlink()
    (tmp_path / "instances.png").unlink()
    assert rd.load_recognition_input(tmp_path).instruction == {"present": False}


def test_origin_and_render_source_must_agree(tmp_path):
    with pytest.raises(ValueError, match="origin"):
        _write(tmp_path, origin="computed")
    with pytest.raises(ValueError, match="render_source"):
        _write(tmp_path / "b", meta={**META, "render_source": "isaac"})


def test_mismatched_camera_info_frame_is_rejected(tmp_path):
    _write(tmp_path)
    info = json.loads((tmp_path / "camera_info.json").read_text())
    info["frame_id"] = "other"
    (tmp_path / "camera_info.json").write_text(json.dumps(info))
    with pytest.raises(ValueError, match="frame"):
        rd.load_recognition_input(tmp_path)


@pytest.mark.parametrize("axis", [(1, -1, 0), (0, 0, 1), (1, 0, 0)])
def test_tf_rotation_round_trips_at_180_degrees(tmp_path, axis):
    out = instance_rig.render_instances([])
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    rot = 2 * np.outer(k, k) - np.eye(3)  # 180 degrees about k
    tf = RigidTransform("camera_optical_frame", "base_link", rot, out.scene.base_from_optical.translation_m)
    _write(tmp_path, scene=replace(out.scene, base_from_optical=tf))
    assert np.allclose(rd.load_recognition_input(tmp_path).scene.base_from_optical.rotation, rot, atol=1e-9)


def test_digest_covers_every_file(tmp_path):
    _write(tmp_path)
    before = rd.scene_digest(tmp_path)
    (tmp_path / "truth.json").write_text(json.dumps({"instances": [1]}))
    assert rd.scene_digest(tmp_path) != before
```

- [ ] **Step 2: 실패 확인** — `ImportError`
- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/recognition_dataset.py
"""Scene dataset schema version 2: multi-instance truth beside the v1 input files.

Input files keep the v1 encodings and checks. tf.json names one of TF_ORIGINS
and a rendered pose is named as rendered, never as a received /tf_static
(ADR 0004 D5 amendment, 2026-10-15). instruction.json holds the system input
only; whether an instruction is ambiguous and which instance it means are
evaluator truth in truth.json. The input loader never opens truth.json or the
instance image.
"""

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.scene_dataset import (
    SceneInput, _read_json, _read_png, _require_fields, decode_depth_mm, intrinsics_from_camera_info,
)

SCHEMA_VERSION = 2
TF_ORIGINS = ("received_tf_static", "isaac_render_pose", "rig_render_pose")
ORIGIN_SOURCE = {"rig_render_pose": "rig", "isaac_render_pose": "isaac"}
_DISTRACTOR = 65535


@dataclass(frozen=True)
class RecognitionInput:
    scene: SceneInput
    instruction: dict


@dataclass(frozen=True)
class RecognitionSample:
    input: RecognitionInput
    origin: str
    meta: dict
    truth: dict
    instruction: dict
    instance_ids: np.ndarray


def _quaternion_xyzw(rotation) -> list[float]:
    """Shepperd's method: branch on the largest of w, x, y, z."""
    m = np.asarray(rotation, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    i = int(np.argmax((trace, m[0, 0], m[1, 1], m[2, 2])))
    if i == 0:
        s = 2.0 * np.sqrt(1.0 + trace)
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4)
    elif i == 1:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = (s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif i == 2:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s)
    q = np.asarray(q) / np.linalg.norm(q)
    return [float(v) for v in q]


def _png(path: Path, array) -> None:
    from PIL import Image

    Image.fromarray(np.ascontiguousarray(array)).save(path, format="PNG")


def _check_meta(origin: str, meta: dict) -> None:
    if origin not in TF_ORIGINS:
        raise ValueError(f"tf origin must be one of {TF_ORIGINS}")
    expected = ORIGIN_SOURCE.get(origin)
    if expected is None or meta.get("render_source") != expected:
        raise ValueError(f"render_source must be {expected!r} for origin {origin!r}")


def write_recognition_scene(scene_dir, scene: SceneInput, instance_ids, *, origin, meta, truth, instruction) -> None:
    _check_meta(origin, meta)
    scene_dir = Path(scene_dir)
    scene_dir.mkdir(parents=True, exist_ok=True)
    depth = scene.depth_m
    ok = np.isfinite(depth) & (depth > 0)
    if np.any(depth[ok] > 65.535):
        raise ValueError("Depth exceeds the uint16 millimetre range")
    raw = np.zeros(depth.shape, dtype=np.uint16)
    raw[ok] = np.rint(depth[ok] * 1000).astype(np.uint16)
    ids = np.asarray(instance_ids)
    codes = np.where(ids >= 0, ids + 1, np.where(ids == -2, _DISTRACTOR, 0)).astype(np.uint16)
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
            "target_frame": scene.base_from_optical.target_frame, "source_frame": scene.base_from_optical.source_frame,
            "translation_m": [float(v) for v in scene.base_from_optical.translation_m],
            "quaternion_xyzw": _quaternion_xyzw(scene.base_from_optical.rotation), "origin": origin,
        },
        "scene.json": {**meta, "schema_version": SCHEMA_VERSION, "stamp_ns": scene.stamp_ns,
                       "clock_domain": scene.clock_domain, "source_provenance": scene.source_provenance},
        "truth.json": truth,
        "instruction.json": instruction,
    }
    for name, obj in files.items():
        (scene_dir / name).write_text(json.dumps(obj, indent=1, sort_keys=True), encoding="utf-8")
    _png(scene_dir / "rgb.png", np.asarray(scene.rgb, dtype=np.uint8))
    _png(scene_dir / "depth_mm.png", raw)
    _png(scene_dir / "instances.png", codes)


def read_meta(scene_dir) -> dict:
    meta = _read_json(Path(scene_dir) / "scene.json")
    if meta.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("scene.json schema_version must be 2")
    return meta


def _load_input(scene_dir: Path):
    meta = read_meta(scene_dir)
    info = _read_json(scene_dir / "camera_info.json")
    intrinsics = intrinsics_from_camera_info(info)
    tf = _read_json(scene_dir / "tf.json")
    _require_fields(tf, {"target_frame", "source_frame", "translation_m", "quaternion_xyzw", "origin"}, "tf")
    _check_meta(tf["origin"], meta)
    if tf["target_frame"] != "base_link" or tf["source_frame"] != "camera_optical_frame":
        raise ValueError("TF must map camera_optical_frame to base_link")
    if intrinsics.frame_id != tf["source_frame"]:
        raise ValueError("CameraInfo frame must match TF source_frame")
    if info["stamp_ns"] != meta["stamp_ns"]:
        raise ValueError("CameraInfo and scene acquisition stamps must match")
    transform = RigidTransform(source_frame=tf["source_frame"], target_frame=tf["target_frame"],
                               rotation=rotation_matrix_from_quaternion_xyzw(tf["quaternion_xyzw"]),
                               translation_m=tf["translation_m"])
    rgb = _read_png(scene_dir / "rgb.png", depth=False)
    depth = decode_depth_mm(_read_png(scene_dir / "depth_mm.png", depth=True), _read_json(scene_dir / "depth_meta.json"))
    shape = (intrinsics.height, intrinsics.width)
    if rgb.shape != (*shape, 3) or depth.shape != shape:
        raise ValueError("RGB and depth shapes must match CameraInfo width/height")
    scene = SceneInput(rgb=rgb, depth_m=depth, intrinsics=intrinsics, base_from_optical=transform,
                       stamp_ns=int(meta["stamp_ns"]), clock_domain=meta["clock_domain"],
                       source_provenance=meta["source_provenance"])
    return RecognitionInput(scene, _read_json(scene_dir / "instruction.json")), tf["origin"], meta


def load_recognition_input(scene_dir) -> RecognitionInput:
    given, _, _ = _load_input(Path(scene_dir))
    return given


def load_recognition_sample(scene_dir) -> RecognitionSample:
    scene_dir = Path(scene_dir)
    given, origin, meta = _load_input(scene_dir)
    codes = _read_png(scene_dir / "instances.png", depth=True).astype(np.int64)
    if codes.shape != given.scene.depth_m.shape:
        raise ValueError("instances.png must match the depth grid")
    ids = np.where(codes == _DISTRACTOR, -2, codes - 1).astype(np.int32)
    return RecognitionSample(given, origin, meta, _read_json(scene_dir / "truth.json"), given.instruction, ids)


def scene_digest(scene_dir) -> str:
    digest = hashlib.sha256()
    for path in sorted(Path(scene_dir).iterdir()):
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()
```

- [ ] **Step 4: ADR 0004 D5 개정 문단** — D5 절 끝에 추가:

```markdown
**개정 (2026-10-15, 사용자 결정 — 팔레트 인식 일반화 설계 §13).** 데이터 계약을 **명시적으로 확장**한다: 스키마 판 2(`forklift_core.perception.recognition_dataset`)의 `tf.json` `origin` 은 `received_tf_static` · `isaac_render_pose` · `rig_render_pose` 중 하나이고, 렌더 출처는 `meta.render_source` 와 짝이 맞아야 한다. 렌더 자세에서 계산한 변환은 그 출처 이름으로 적고 `received_tf_static` 이라는 이름을 붙이지 않는다(위 금지는 그대로). `isaac_render_pose` 장면은 Isaac 모집단으로만, `rig_render_pose` 장면은 CPU 기구 회귀로만 쓰며 렌더 기반 판정 분모에 넣지 않는다. 판 1 로더(`scene_dataset.py`)는 바꾸지 않는다.
```

- [ ] **Step 5: 통과 확인** — Expected: 8 passed(Pillow 12 에서 `-W error` 로도)
- [ ] **Step 6: 커밋** — `git add src/forklift_core/perception/recognition_dataset.py tests/unit/perception/test_recognition_dataset.py docs/decisions/0004-simulation-engine-and-insertion-depth.md` · `git commit -m "perception: scene dataset schema v2 (instruction input vs truth, v1 input checks kept, origin/render_source pairing, robust quaternion, full-file digest); ADR 0004 D5 amendment"`

**검토 C:** 작업 7–9 뒤 Codex — 초점: "면별 정답·노출·끼움, 조리법 규칙(지시·맞닿음·S4·S7 층화)과 하한, 판 2 의 입력/정답 분리와 검증". 수렴까지 반복.

### Task 10: 다중 인스턴스 평가기 `evaluation_multi`

**Files:**
- Create: `src/forklift_core/perception/evaluation_multi.py`
- Test: `tests/unit/perception/test_evaluation_multi.py`

**Interfaces:**
- Consumes: `recognition_stats`, `fork_fit.TIP_LEVER_M`, `fork_fit.AXIAL_DANGER_M`, `yaw_difference_rad`
- Produces: `CandidateView`(유한성 검사; `candidate_id`, `insertion_yaw_rad`, `left_m`, `right_m`, `fitted`, `identified`, `pickable: bool | None`, `reasons`, `layer_abs: int | None`; 속성 `counted`), `TargetDecision`(`status` ∈ `DECISIONS = ("selected", "ask_operator", "reobserve", "none", "not_pickable")`, `candidate_id`, `reasons`), `SceneVerdict`(…, `render_source`, `noisy`, `miss`, `dangerous`, `extra`, `negative_fp`, `target_error`, `deferred_unambiguous`, `fit_error`, `errors: tuple[(가로, 삽입축, yaw, z), ...]`, `z_judged: bool`, `elapsed_s`), `judge_scene(meta, truth, instruction_input, candidates, decision, *, elapsed_s=None) -> SceneVerdict`, `summarize(verdicts, *, split, render_source) -> dict`, `GATES`, `APPLICABLE`, `POSE_P95`

규칙(설계 §3.4):
- **대응:** 식별된 적합 후보와 정답 인스턴스의 **접근 가능한 면만**. 비용 = max(좌, 우 포켓 3차원 거리), 대응 조건 거리 ≤ 0.20 m 이고 |yaw 차| ≤ 0.35 rad. 일대일 최대 개수·최소 비용(전수 탐색, 후보 ≤ 8). 오차 = 그 면 축으로 분해한 (가로, 삽입축, yaw, 높이).
- **위험:** `pickable is True` 인 **모든** 식별된 적합 후보. 대응된 후보는 그 면의 정답으로, 대응되지 않은 후보(중복 포함)는 접근 가능한 면 중 비용이 가장 낮은 면으로 판정하고 그런 면이 없으면 위험. 위험 = 정답 면 `fit.pickable` 거짓, 또는 가로 + 0.33·tan|yaw| > 그 면 `danger_threshold_m`, 또는 삽입축 > 0.040 m.
- **놓침:** 양성 모집단에서 `eligible` 인스턴스 중 대응되지 않은 것이 있으면 참.
- **추가 후보:** 대응되지 않은 식별된 적합 후보가 있으면 참. **음성 오검출:** 음성 모집단에서 식별된 적합 후보가 하나라도 있으면 참.
- **지시**(`truth["instruction_truth"]` 기준): 애매함 → `selected` 이면 오류. 애매하지 않음 → `ask_operator`·`reobserve` 는 `deferred_unambiguous`(요청률에 포함, 오류 아님); `selected` 는 대상 인스턴스와 대응되지 않거나, 입력 지시에 층이 있는데 후보 `layer_abs` 가 그 층이 아니면(**None 포함**) 오류; `none` 은 대상이 `eligible` 이면 오류; `not_pickable` 은 대상의 접근 가능한 면 정답이 집을 수 있음이면 오류.
- **끼움 오류**(비규격): 대상(지시 대상) 인스턴스에 대응된 후보의 `pickable` 이 그 면 정답과 다르면 참. 대응 후보가 없거나 `pickable` 이 None 이면 None.
- **높이 판정:** S4(알려진 형상)·S7 장면의 오차만 높이 관문에 들어간다(`z_judged`).
- **관문:** `APPLICABLE` 모집단별(알려진 형상: miss·dangerous·extra·target_error·deferred_unambiguous; 비규격: + fit_error; 음성: negative_fp) + 위험 관측 0 + 자세 p95(가로·삽입축·yaw, 높이는 `z_judged` 오차만). 표본이 없으면 합격 거짓. `summarize` 는 `render_source` 로 거르고 `official = render_source == "isaac" and 모든 장면 noisy` 를 기록한다.

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_evaluation_multi.py
import math

import pytest

from forklift_core.perception.evaluation_multi import CandidateView, TargetDecision, judge_scene, summarize


def _face(left, right, yaw=0.0, accessible=True, sign=1, pickable=True, thr=0.0437):
    return {"axis": "x", "sign": sign, "insertion_yaw_rad": yaw, "left_m": left, "right_m": right,
            "pocket_width_m": [0.2, 0.2], "pocket_height_m": 0.078, "accessible": accessible, "exposure": 1.0,
            "fit": {"pickable": pickable, "reasons": [] if pickable else ["fork_spacing_adjust"]},
            "danger_threshold_m": thr}


def _inst(k, y, *, eligible=True, pickable=True, thr=0.0437, layer=0):
    return {"instance_id": k, "known": True, "layer": layer, "support_z_m": 0.0, "eligible": eligible,
            "faces": [_face([2.0, y + 0.186, 0.061], [2.0, y - 0.186, 0.061], pickable=pickable, thr=thr),
                      _face([2.6, y - 0.186, 0.061], [2.6, y + 0.186, 0.061], yaw=math.pi, accessible=False, sign=-1)]}


META = {"scene_id": "s", "case": "S2", "shape_key": "epal6", "split": "eval", "render_source": "rig", "noise": {"k": 1}}
NONE_IN = {"present": False}


def _truth(insts, kind="none", target=None):
    return {"instances": insts, "instruction_truth": {"kind": kind, "target": target}}


def _cand(cid, y, dy=0.0, yaw=0.0, pickable=True, layer=0, dx=0.0):
    return CandidateView(cid, yaw, (2.0 + dx, y + 0.186 + dy, 0.061), (2.0 + dx, y - 0.186 + dy, 0.061),
                         True, True, pickable, (), layer)


def test_perfect_two_pallets_pass():
    v = judge_scene(META, _truth([_inst(0, 0.5), _inst(1, -0.5)]), NONE_IN,
                    [_cand("a", 0.5), _cand("b", -0.5)], TargetDecision("selected", "a", ()))
    assert (v.miss, v.dangerous, v.extra) == (False, False, False)


def test_back_face_candidate_is_not_matched():
    c = CandidateView("a", math.pi, (2.6, -0.186, 0.061), (2.6, 0.186, 0.061), True, True, False, (), 0)
    v = judge_scene(META, _truth([_inst(0, 0.0)]), NONE_IN, [c], TargetDecision("none", None, ()))
    assert v.miss and v.extra


def test_unmatched_pickable_duplicate_far_along_the_axis_is_dangerous():
    v = judge_scene(META, _truth([_inst(0, 0.0)]), NONE_IN, [_cand("a", 0.0), _cand("b", 0.0, dx=0.25)],
                    TargetDecision("selected", "b", ()))
    assert v.extra and v.dangerous


def test_lateral_plus_yaw_projection_is_dangerous():
    c = _cand("a", 0.0, dy=0.035, yaw=math.radians(3.9))  # 35 + 0.33 tan 3.9 deg = 57.5 mm > 43.7
    assert judge_scene(META, _truth([_inst(0, 0.0)]), NONE_IN, [c], TargetDecision("selected", "a", ())).dangerous


def test_nonfitting_pallet_correct_stop_vs_allowed_pick():
    meta = {**META, "case": "S7", "shape_key": "nonstandard"}
    truth = _truth([_inst(0, 0.0, pickable=False, thr=-0.0475)], "unambiguous", 0)
    instr = {"present": True, "x_m": 2.3, "y_m": 0.0, "radius_m": 0.3, "layer": None}
    stop = judge_scene(meta, truth, instr, [_cand("a", 0.0, pickable=False)],
                       TargetDecision("not_pickable", "a", ("fork_spacing_adjust",)))
    assert not stop.dangerous and stop.fit_error is False and stop.target_error is False
    pick = judge_scene(meta, truth, instr, [_cand("a", 0.0)], TargetDecision("selected", "a", ()))
    assert pick.dangerous and pick.fit_error


def test_nan_candidates_are_rejected():
    with pytest.raises(ValueError, match="finite"):
        CandidateView("a", 0.0, (float("nan"), 0.0, 0.0), (2.0, 0.0, 0.0), True, True, True, (), 0)


def test_instruction_rules():
    truth_amb = _truth([_inst(0, 0.2), _inst(1, -0.2)], "ambiguous", 0)
    instr = {"present": True, "x_m": 2.3, "y_m": 0.0, "radius_m": 0.5, "layer": None}
    cands = [_cand("a", 0.2), _cand("b", -0.2)]
    assert judge_scene(META, truth_amb, instr, cands, TargetDecision("selected", "a", ())).target_error
    assert judge_scene(META, truth_amb, instr, cands, TargetDecision("ask_operator", None, ())).target_error is False
    truth_un = _truth([_inst(0, 0.5), _inst(1, -0.5)], "unambiguous", 1)
    for status in ("ask_operator", "reobserve"):
        v = judge_scene(META, truth_un, instr, [_cand("a", 0.5), _cand("b", -0.5)], TargetDecision(status, None, ()))
        assert v.deferred_unambiguous and v.target_error is False


def test_selecting_without_an_absolute_layer_is_an_error():
    meta = {**META, "case": "S4"}
    truth = _truth([_inst(0, 0.0, layer=1)], "unambiguous", 0)
    instr = {"present": True, "x_m": 2.3, "y_m": 0.0, "radius_m": 0.3, "layer": 1}
    v = judge_scene(meta, truth, instr, [_cand("a", 0.0, layer=None)], TargetDecision("selected", "a", ()))
    assert v.target_error


def test_pose_gate_fails_on_a_30_mm_lateral_offset():
    verdicts = [judge_scene({**META, "scene_id": str(i)}, _truth([_inst(0, 0.0, thr=0.2)]), NONE_IN,
                            [_cand("a", 0.0, dy=0.03)], TargetDecision("selected", "a", ())) for i in range(114)]
    gates = summarize(verdicts, split="eval", render_source="rig")["gates"]
    assert not gates["known:epal6:pose_lateral_p95"]["pass"]


def test_empty_population_never_passes_and_known_has_no_fit_gate():
    gates = summarize([], split="eval", render_source="isaac")["gates"]
    assert gates and all(not g["pass"] for g in gates.values())
    assert not any(k.startswith("known") and k.endswith(":fit_error") for k in gates)
```

- [ ] **Step 2: 실패 확인** — `ImportError`
- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/evaluation_multi.py
"""Scene-level judgment for multi-instance recognition (design §3.3-3.4).

Units are scenes: any failure in a scene fails that scene. Gates are judged on
pooled populations with one-sided 95 % Clopper-Pearson bounds; per-case cells
are reported only. An applicable gate with no scenes fails. Only accessible
faces are matched. Every pick-permitted candidate is checked for danger.
"""

import itertools
import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from forklift_core.perception.fork_fit import AXIAL_DANGER_M, TIP_LEVER_M
from forklift_core.perception.pocket_observation import yaw_difference_rad
from forklift_core.perception.recognition_stats import allowed_failures, cp_upper

MATCH_DISTANCE_M = 0.20
MATCH_YAW_RAD = 0.35
DECISIONS = ("selected", "ask_operator", "reobserve", "none", "not_pickable")
GATES = {"miss": 0.05, "dangerous": 0.05, "extra": 0.10, "negative_fp": 0.10, "target_error": 0.05,
         "deferred_unambiguous": 0.10, "fit_error": 0.05}
APPLICABLE = {
    "known:epal6": ("miss", "dangerous", "extra", "target_error", "deferred_unambiguous"),
    "known:t11_06": ("miss", "dangerous", "extra", "target_error", "deferred_unambiguous"),
    "nonstandard": ("miss", "dangerous", "extra", "target_error", "deferred_unambiguous", "fit_error"),
    "negative": ("negative_fp",),
}
POSE_P95 = {"lateral": 0.020, "axial": 0.020, "yaw": math.radians(2.0), "z": 0.005}


@dataclass(frozen=True)
class CandidateView:
    candidate_id: str
    insertion_yaw_rad: float
    left_m: tuple
    right_m: tuple
    fitted: bool
    identified: bool
    pickable: bool | None
    reasons: tuple
    layer_abs: int | None

    def __post_init__(self) -> None:
        if len(self.left_m) != 3 or len(self.right_m) != 3:
            raise ValueError("candidate pockets must be finite three-vectors")
        if not all(np.isfinite([self.insertion_yaw_rad, *self.left_m, *self.right_m])):
            raise ValueError("candidate pockets and yaw must be finite three-vectors")

    @property
    def counted(self) -> bool:
        return self.fitted and self.identified


@dataclass(frozen=True)
class TargetDecision:
    status: str
    candidate_id: str | None
    reasons: tuple

    def __post_init__(self) -> None:
        if self.status not in DECISIONS:
            raise ValueError(f"decision status must be one of {DECISIONS}")


@dataclass(frozen=True)
class SceneVerdict:
    scene_id: str
    case: str
    shape_key: str
    split: str
    population: str
    render_source: str
    noisy: bool
    miss: bool | None
    dangerous: bool
    extra: bool
    negative_fp: bool | None
    target_error: bool | None
    deferred_unambiguous: bool | None
    fit_error: bool | None
    errors: tuple
    z_judged: bool
    elapsed_s: float | None


def population_of(meta: dict) -> str:
    if meta["case"] == "N":
        return "negative"
    if meta["case"] == "S7":
        return "nonstandard"
    return f"known:{meta['shape_key']}"


def _face_cost(c: CandidateView, face: dict):
    dyaw = abs(yaw_difference_rad(c.insertion_yaw_rad, face["insertion_yaw_rad"]))
    dl, dr = np.subtract(c.left_m, face["left_m"]), np.subtract(c.right_m, face["right_m"])
    dist = max(float(np.linalg.norm(dl)), float(np.linalg.norm(dr)))
    if not (dyaw <= MATCH_YAW_RAD and dist <= MATCH_DISTANCE_M):
        return None
    a = np.array([math.cos(face["insertion_yaw_rad"]), math.sin(face["insertion_yaw_rad"]), 0.0])
    n = np.array([-a[1], a[0], 0.0])
    d = (dl + dr) / 2
    return dist, (abs(float(d @ n)), abs(float(d @ a)), dyaw, abs(float(d[2])))


def _best_face(c, inst):
    best = None
    for face in inst["faces"]:
        if not face["accessible"]:
            continue
        fc = _face_cost(c, face)
        if fc and (best is None or fc[0] < best[0][0]):
            best = (fc, face)
    return best


def _match(cands, instances):
    costs = {}
    for ci, c in enumerate(cands):
        for ii, inst in enumerate(instances):
            best = _best_face(c, inst)
            if best is not None:
                costs[ci, ii] = best
    for size in range(min(len(cands), len(instances)), 0, -1):
        best, best_total = None, None
        for chosen in itertools.combinations(range(len(cands)), size):
            for perm in itertools.permutations(range(len(instances)), size):
                if all((c, i) in costs for c, i in zip(chosen, perm)):
                    total = sum(costs[c, i][0][0] for c, i in zip(chosen, perm))
                    if best_total is None or total < best_total:
                        best, best_total = dict(zip(chosen, perm)), total
        if best is not None:
            return best, costs
    return {}, costs


def _danger(errors, face) -> bool:
    lateral, axial, dyaw, _ = errors
    return (not face["fit"]["pickable"] or lateral + TIP_LEVER_M * math.tan(dyaw) > face["danger_threshold_m"]
            or axial > AXIAL_DANGER_M)


def judge_scene(meta, truth, instruction_input, candidates: Sequence[CandidateView], decision: TargetDecision,
                *, elapsed_s=None) -> SceneVerdict:
    population = population_of(meta)
    instances = truth.get("instances", [])
    counted = [c for c in candidates if c.counted]
    pairs, costs = _match(counted, instances)
    matched_inst = {i: c for c, i in pairs.items()}
    errors = tuple(costs[c, i][0][1] for c, i in pairs.items())
    dangerous = False
    for ci, cand in enumerate(counted):
        if cand.pickable is not True:
            continue
        if ci in pairs:
            (_, err), face = costs[ci, pairs[ci]]
        else:
            options = [costs[ci, ii] for ii in range(len(instances)) if (ci, ii) in costs]
            if not options:
                dangerous = True
                continue
            (_, err), face = min(options, key=lambda o: o[0][0])
        dangerous = dangerous or _danger(err, face)
    negative = population == "negative"
    miss = None if negative else any(inst["eligible"] and k not in matched_inst for k, inst in enumerate(instances))
    target_error = deferred = fit_error = None
    itruth = truth.get("instruction_truth", {"kind": "none", "target": None})
    target = itruth.get("target")
    by_id = {c.candidate_id: k for k, c in enumerate(counted)}
    if not negative and itruth["kind"] == "ambiguous":
        target_error = decision.status == "selected"
    elif not negative and itruth["kind"] == "unambiguous":
        tgt = instances[target]
        tgt_pickable = any(f["accessible"] and f["fit"]["pickable"] for f in tgt["faces"])
        deferred = decision.status in ("ask_operator", "reobserve")
        if decision.status == "selected":
            k = by_id.get(decision.candidate_id)
            layer = instruction_input.get("layer")
            target_error = k is None or pairs.get(k) != target or (layer is not None and counted[k].layer_abs != layer)
        elif decision.status == "none":
            target_error = bool(tgt["eligible"])
        elif decision.status == "not_pickable":
            target_error = tgt_pickable
        else:
            target_error = False
    if population == "nonstandard" and instances:
        k_inst = 0 if target is None else target
        if k_inst in matched_inst:
            cand = counted[matched_inst[k_inst]]
            face = costs[matched_inst[k_inst], k_inst][1]
            fit_error = None if cand.pickable is None else cand.pickable != face["fit"]["pickable"]
    return SceneVerdict(
        meta["scene_id"], meta["case"], meta["shape_key"], meta["split"], population,
        meta.get("render_source", "rig"), meta.get("noise") is not None,
        miss, dangerous, len(pairs) < len(counted), bool(counted) if negative else None,
        target_error, deferred, fit_error, errors, meta["case"] in ("S4", "S7"), elapsed_s,
    )


def _gate(values, bound):
    values = [v for v in values if v is not None]
    n, k = len(values), sum(bool(v) for v in values)
    upper = cp_upper(k, n)
    return {"n": n, "failures": k, "upper": upper, "bound": bound,
            "allowed_failures": allowed_failures(n, bound) if n else None,
            "pass": upper is not None and upper <= bound}


def _p95_gate(samples, target):
    value = float(np.percentile(samples, 95)) if len(samples) else None
    return {"n": len(samples), "p95": value, "target": target, "pass": value is not None and value <= target}


def summarize(verdicts: Sequence[SceneVerdict], *, split: str, render_source: str) -> dict:
    chosen = [v for v in verdicts if v.split == split and v.render_source == render_source]
    gates, cells = {}, {}
    for pop, names in APPLICABLE.items():
        mine = [v for v in chosen if v.population == pop]
        for name in names:
            gates[f"{pop}:{name}"] = _gate([getattr(v, name) for v in mine], GATES[name])
        if pop != "negative":
            gates[f"{pop}:dangerous_observed_zero"] = {"n": len(mine), "failures": sum(v.dangerous for v in mine),
                                                       "pass": bool(mine) and not any(v.dangerous for v in mine)}
            errs = [e for v in mine for e in v.errors]
            for j, axis in enumerate(("lateral", "axial", "yaw")):
                gates[f"{pop}:pose_{axis}_p95"] = _p95_gate([e[j] for e in errs], POSE_P95[axis])
            gates[f"{pop}:pose_z_p95"] = _p95_gate([e[3] for v in mine if v.z_judged for e in v.errors], POSE_P95["z"])
        for case in sorted({v.case for v in mine}):
            cell = [v for v in mine if v.case == case]
            cells[f"{pop}:{case}"] = {name: _gate([getattr(v, name) for v in cell], GATES[name]) for name in names}
    return {"split": split, "render_source": render_source, "scenes": len(chosen),
            "official": render_source == "isaac" and bool(chosen) and all(v.noisy for v in chosen),
            "gates": gates, "cells": cells}
```

- [ ] **Step 4: 통과 확인** — Expected: 10 passed
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/evaluation_multi.py tests/unit/perception/test_evaluation_multi.py` · `git commit -m "perception: multi-instance scene judgment (accessible faces only, danger on every pick-permitted candidate, instruction and layer rules, fit error) with pooled bound and pose gates"`

### Task 11: 생성·평가 CLI

**Files:**
- Create: `tools/generate_recognition_scenes.py`, `tools/evaluate_recognition.py`
- Test: `tests/integration/test_recognition_cli.py`

**Interfaces:**
- `generate_recognition_scenes.py --out DIR [--cases S1,...] [--limit N] [--noise default|none] [--digest-only]` — `catalogue()` 조리법을 렌더(정답 계산의 렌더를 재사용) → 잡음 → 판 2. `origin="rig_render_pose"`, `render_source="rig"`. `--digest-only` 는 `catalogue_digest` 와 numpy 판만 출력하고 끝낸다(기계 간 대조).
- `evaluate_recognition.py --dataset DIR --split dev|eval --detector baseline|oracle --output DIR [--source-manifest FILE]` — **`read_meta` 로 분할을 먼저 고르고** 그 장면만 로드. `baseline` 은 **장면의 형상 라벨을 쓰지 않고** EPAL 6·T11 두 prior 로 각각 `detect_pockets` 를 돌려 `valid` 를 후보로 낸다(입력 지시가 있으면 지시 위치에 가장 가까운 후보를 `selected`, 없으면 첫 후보; 후보가 없으면 `none`). `oracle` 은 정답의 접근 가능한 면으로 후보를 내고 정답 지시대로 결정한다(평가기 자기 점검). 출력: `metrics.json`(`summarize` + 장면별 `scene_digest` 를 합친 `dataset_sha256`, 소스 manifest 또는 `git rev-parse`·`git status` 결과(실패하면 오류), 인자), `verdicts.csv`(시간 열 제외 — 재실행 비교용).

- [ ] **Step 1: 실패하는 통합 시험 작성**

```python
# tests/integration/test_recognition_cli.py
import csv
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _run(*args):
    return subprocess.run([sys.executable, *args], cwd=ROOT, check=True, capture_output=True, text=True)


def _rows(path):
    with (path / "verdicts.csv").open() as f:
        return [{k: v for k, v in row.items() if k != "elapsed_s"} for row in csv.DictReader(f)]


def test_oracle_self_check_and_label_free_baseline(tmp_path):
    data = tmp_path / "data"
    _run("tools/generate_recognition_scenes.py", "--out", str(data), "--cases", "S1,S2,S7", "--limit", "3", "--noise", "none")
    assert len(list(data.glob("*/scene.json"))) == 3 * 2 * 2 + 3
    for detector in ("oracle", "baseline"):
        out = tmp_path / detector
        _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", detector, "--output", str(out))
        metrics = json.loads((out / "metrics.json").read_text())
        assert metrics["render_source"] == "rig" and not metrics["official"]
        if detector == "oracle":
            for name, gate in metrics["gates"].items():
                if gate.get("n") and name.endswith((":miss", ":extra", ":dangerous_observed_zero", ":target_error", ":fit_error")):
                    assert gate["failures"] == 0, name
    # Baseline must not consult the scene's shape label: corrupt it and expect identical verdicts.
    first = next(data.glob("s1_epal6_*/scene.json"))
    meta = json.loads(first.read_text())
    meta["shape_key"] = "t11_06"
    first.write_text(json.dumps(meta))
    out2 = tmp_path / "baseline2"
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "baseline", "--output", str(out2))
    strip = lambda rows: [{k: v for k, v in r.items() if k not in ("shape_key", "population")} for r in rows]
    assert strip(_rows(tmp_path / "baseline")) == strip(_rows(out2))


def test_digest_only_prints_the_catalogue_digest():
    out = _run("tools/generate_recognition_scenes.py", "--out", "/nonexistent", "--digest-only").stdout
    assert "catalogue_sha256" in out and "numpy" in out
```

- [ ] **Step 2: 실패 확인** — 스크립트 없음
- [ ] **Step 3: 구현** — `tools/generate_recognition_scenes.py`:

```python
#!/usr/bin/env python3
"""Render the recognition catalogue on the CPU rig into schema-v2 scene dirs."""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from forklift_core.perception.fork_fit import ForkSetup
from forklift_core.perception.recognition_dataset import write_recognition_scene
from tools import recognition_scenes as rs
from tools.recognition_truth import scene_truths


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--cases", default=",".join(rs.CASES))
    p.add_argument("--limit", type=int, default=None, help="first N recipes per case and shape")
    p.add_argument("--noise", choices=("default", "none"), default="default")
    p.add_argument("--digest-only", action="store_true")
    a = p.parse_args(argv)
    recipes = rs.catalogue()
    if a.digest_only:
        print(json.dumps({"catalogue_sha256": rs.catalogue_digest(recipes), "numpy": np.__version__}))
        return
    cases, counts = set(a.cases.split(",")), {}
    for recipe in recipes:
        if recipe.case not in cases:
            continue
        key = (recipe.case, recipe.shape_key)
        counts[key] = counts.get(key, 0) + 1
        if a.limit is not None and counts[key] > a.limit:
            continue
        instr = recipe.instruction
        holder = instr.target if instr.target is not None else 0
        truths, rendered = scene_truths(
            [(i.spec, i.placement, i.damage, instr.load if k == holder else None) for k, i in enumerate(recipe.instances)],
            ForkSetup(recipe.fork_spacing_m), distractors=recipe.distractors,
            layers=[i.layer for i in recipe.instances])
        scene, ids, noise = rendered.scene, rendered.instance_ids, None
        if a.noise == "default":
            noise_seed = int(np.random.SeedSequence(recipe.seed).spawn(3)[2].generate_state(1)[0])
            model = NoiseModel(seed=noise_seed)
            scene, ids = apply_noise(scene, ids, model)
            noise = model.to_json()
        write_recognition_scene(
            a.out / recipe.scene_id, scene, ids, origin="rig_render_pose",
            meta={"scene_id": recipe.scene_id, "case": recipe.case, "shape_key": recipe.shape_key, "split": recipe.split,
                  "seed": recipe.seed, "rank": recipe.rank, "render_source": "rig", "noise": noise,
                  "fork_spacing_m": recipe.fork_spacing_m, "touching": recipe.touching},
            truth={"instances": [t.to_json() for t in truths],
                   "instruction_truth": {"kind": instr.kind, "target": instr.target}},
            instruction={"present": instr.kind != "none", "x_m": instr.x_m, "y_m": instr.y_m,
                         "radius_m": instr.radius_m, "layer": instr.layer,
                         "load": None if instr.load is None else instr.load.to_json()},
        )


if __name__ == "__main__":
    main()
```

`tools/evaluate_recognition.py`:

```python
#!/usr/bin/env python3
"""Judge a schema-v2 dataset with the current detector (baseline) or the truth (oracle)."""

import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from forklift_core.perception.evaluation_multi import CandidateView, TargetDecision, judge_scene, summarize
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.recognition_dataset import load_recognition_sample, read_meta, scene_digest

ROOT = Path(__file__).resolve().parents[1]
PRIORS = ("config/pallet_prior_epal6.yaml", "config/pallet_prior_t11_06.yaml")


def _baseline(sample):
    """The current detector, run once per known prior. It never reads scene labels."""
    cands = []
    for k, path in enumerate(PRIORS):
        prior = load_pallet_prior(ROOT / path)
        obs = detect_pockets(sample.input.scene, prior, DetectorParams.derived_for(prior)).observation
        if obs.status == "valid":
            cands.append(CandidateView(f"baseline{k}", obs.insertion_yaw_rad, obs.left.center_m, obs.right.center_m,
                                       True, True, True, (), None))
    if not cands:
        return cands, TargetDecision("none", None, ())
    instr = sample.input.instruction
    if instr.get("present"):
        def dist(c):
            return math.hypot((c.left_m[0] + c.right_m[0]) / 2 - instr["x_m"], (c.left_m[1] + c.right_m[1]) / 2 - instr["y_m"])
        return cands, TargetDecision("selected", min(cands, key=dist).candidate_id, ())
    return cands, TargetDecision("selected", cands[0].candidate_id, ())


def _oracle(sample):
    cands = []
    for inst in sample.truth["instances"]:
        faces = [f for f in inst["faces"] if f["accessible"]]
        if not faces:
            continue
        f = faces[0]
        cands.append(CandidateView(f"o{inst['instance_id']}", f["insertion_yaw_rad"], tuple(f["left_m"]),
                                   tuple(f["right_m"]), True, True, f["fit"]["pickable"], tuple(f["fit"]["reasons"]),
                                   inst["layer"]))
    itruth = sample.truth["instruction_truth"]
    if itruth["kind"] == "ambiguous":
        return cands, TargetDecision("ask_operator", None, ())
    if itruth["kind"] == "unambiguous":
        cid = f"o{itruth['target']}"
        cand = next((c for c in cands if c.candidate_id == cid), None)
        if cand is None:
            return cands, TargetDecision("none", None, ())
        return cands, TargetDecision("selected" if cand.pickable else "not_pickable", cid, cand.reasons)
    pickable = [c for c in cands if c.pickable]
    return cands, (TargetDecision("selected", pickable[0].candidate_id, ()) if pickable else TargetDecision("none", None, ()))


def _source(manifest):
    if manifest is not None:
        return json.loads(Path(manifest).read_text())
    rev = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True)
    dirty = subprocess.run(["git", "status", "--porcelain"], cwd=ROOT, capture_output=True, text=True, check=True)
    return {"revision": rev.stdout.strip(), "dirty": bool(dirty.stdout.strip())}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--split", choices=("dev", "eval"), required=True)
    p.add_argument("--detector", choices=("baseline", "oracle"), required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--source-manifest", type=Path, default=None)
    a = p.parse_args(argv)
    run = {"baseline": _baseline, "oracle": _oracle}[a.detector]
    verdicts, digest, sources = [], hashlib.sha256(), set()
    for scene_json in sorted(a.dataset.glob("*/scene.json")):
        if read_meta(scene_json.parent)["split"] != a.split:
            continue
        sample = load_recognition_sample(scene_json.parent)
        digest.update(scene_digest(scene_json.parent).encode())
        sources.add(sample.meta["render_source"])
        start = time.perf_counter()
        cands, decision = run(sample)
        verdicts.append(judge_scene(sample.meta, sample.truth, sample.input.instruction, cands, decision,
                                    elapsed_s=time.perf_counter() - start))
    if len(sources) != 1:
        raise SystemExit(f"dataset must hold exactly one render source, found {sorted(sources)}")
    a.output.mkdir(parents=True, exist_ok=True)
    metrics = summarize(verdicts, split=a.split, render_source=sources.pop())
    metrics.update({"detector": a.detector, "dataset_sha256": digest.hexdigest(), "source": _source(a.source_manifest)})
    (a.output / "metrics.json").write_text(json.dumps(metrics, indent=1, default=float))
    with (a.output / "verdicts.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(asdict(verdicts[0]).keys()))
        w.writeheader()
        for v in verdicts:
            row = asdict(v)
            row["elapsed_s"] = ""
            w.writerow(row)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 통과 확인** — `git add tools/generate_recognition_scenes.py tools/evaluate_recognition.py tests/integration/test_recognition_cli.py`, ws1 래퍼(백그라운드) · Expected: 2 passed
- [ ] **Step 5: 커밋** — `git commit -m "tools: recognition scene generation (CPU rig, schema v2, render reuse, digest-only) and evaluation CLI (split-first loading, label-free baseline over both priors, oracle self-check, full-file dataset hash, source revision)"`

### Task 12: CPU 기준선 실행과 검증 기록 (UBAI)

CPU 전용 잡이므로 전역 지침대로 **UBAI(`gate1_External`) Slurm**. `tools/hpc_sweep.sh jobs.tsv [파티션] [코어]` 가 동기화 → 제출 → 대기 → 회수(`artifacts/hpc/<run-id>/out/<이름>.txt`)를 한다(원격 venv `~/forklift-venv`, 원격 사본 `~/forklift`; 동기화는 `.git/` 을 빼므로 소스 manifest 를 따로 보낸다). SSH 왕복이 2분 넘으니 묶어서 보낸다. 비용(독립 검토 실측): 노트북에서 장면당 0.4–2.0 s, 직렬 20–25분 → UBAI 8줄 병렬 약 10분.

**Files:**
- Create: `deploy/hpc/recognition_stage1a_env.tsv`, `deploy/hpc/recognition_stage1a_generate.tsv`, `deploy/hpc/recognition_stage1a_evaluate.tsv`, `docs/validation/<실행일>-recognition-stage1a-cpu-baseline.md`

- [ ] **Step 1: 소스 manifest·환경 확인** — worktree 의 `git status --porcelain` 이 비었는지 확인하고 `source_manifest.json`(`{"revision": "<git rev-parse HEAD>", "dirty": false}`)을 저장소 루트에 만든다(추적하지 않음, 동기화로 원격에 간다). `deploy/hpc/recognition_stage1a_env.tsv`:

```
env	(python -c "import PIL" 2>/dev/null || $HOME/forklift-venv/bin/pip install 'Pillow>=10,<13') && python -c "import PIL, numpy, yaml; print(PIL.__version__, numpy.__version__, yaml.__version__)" && python tools/generate_recognition_scenes.py --out /dev/null --digest-only
```

Run(백그라운드): `tools/hpc_sweep.sh deploy/hpc/recognition_stage1a_env.tsv cpu1 1`. 로컬에서 `python tools/generate_recognition_scenes.py --out /dev/null --digest-only` 를 돌려 **`catalogue_sha256` 이 원격과 같은지** 확인한다(numpy 판이 달라 조리법이 달라졌다면 여기서 드러난다 — 다르면 원인을 먼저 찾는다).

- [ ] **Step 2: 생성** — `deploy/hpc/recognition_stage1a_generate.tsv`(사례마다 한 줄, 병렬):

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

Run: `tools/hpc_sweep.sh deploy/hpc/recognition_stage1a_generate.tsv cpu1 8` (백그라운드).

- [ ] **Step 3: 평가(dev 만 — eval 은 기하 코어 확정 전 열지 않는다, 설계 §5.3)** — `deploy/hpc/recognition_stage1a_evaluate.tsv`:

```
oracle_dev	python tools/evaluate_recognition.py --dataset hpc_data/stage1a --split dev --detector oracle --output hpc_data/stage1a_oracle_dev --source-manifest source_manifest.json && cat hpc_data/stage1a_oracle_dev/metrics.json
baseline_dev	python tools/evaluate_recognition.py --dataset hpc_data/stage1a --split dev --detector baseline --output hpc_data/stage1a_baseline_dev --source-manifest source_manifest.json && cat hpc_data/stage1a_baseline_dev/metrics.json && cat hpc_data/stage1a_baseline_dev/verdicts.csv
```

Run: `tools/hpc_sweep.sh deploy/hpc/recognition_stage1a_evaluate.tsv cpu1 2` (백그라운드).

- [ ] **Step 4: 결과 확인** — oracle: miss·extra·dangerous·target_error·fit_error 실패 0(평가기·정답의 자기 점검; 실패가 있으면 결함이므로 고친 뒤 다시). baseline: `cells` 로 사례별 놓침·추가 후보·위험·음성 오검출을 표로 읽는다. 기대(설계 §1): S2 맞닿음·부분 가림, S4 위층, S5 상판 결손에서 놓침, S6·N 에서 추가 후보·음성 오검출. 기대와 다른 칸은 반례로 적고, 필요하면 그 장면을 `rsync gate1_External:forklift/hpc_data/stage1a/<scene_id> <scratchpad>` 로 받아 확인한다.
- [ ] **Step 5: 검증 기록** — `docs/validation/<실행일>-recognition-stage1a-cpu-baseline.md`: 실행 조건(리비전·manifest, `catalogue_sha256` 로컬/원격 일치, `dataset_sha256`, 잡음 모델, 장면 수, UBAI run-id·job ID, numpy·Pillow 판), oracle 자기 점검, baseline 사례별 표(dev 만), 해석 경계("`rig_render_pose` CPU 기구 회귀이며 렌더 기반 모집단이 아니다 — Isaac 판정은 단계 1B").
- [ ] **Step 6: 커밋·푸시** — `git add deploy/hpc/recognition_stage1a_env.tsv deploy/hpc/recognition_stage1a_generate.tsv deploy/hpc/recognition_stage1a_evaluate.tsv docs/validation/<파일>` · `git commit -m "validation: recognition stage 1A CPU baseline on dev at UBAI (oracle self-check, current detector by case)"` · `git push`

**검토 D:** 작업 10–12 뒤 Codex — 초점: "판정 규칙이 설계 §3.4 와 같은가, 빈·해당 없는 관문, CLI 의 입력/정답 분리와 분할 우선 로드, 기준선 해석이 데이터로 지지되는가". 수렴까지 반복.

---

## 이 계획 다음 (별도 계획)

- **단계 1B — Isaac 렌더:** `sim/isaac/render_recognition_scenes.py`(Replicator `instance_segmentation`, 팔레트 prim 묶기, `isaac_render_pose`), 창고 자산 `SM_PaletteA_01`(S7)·랙·크레이트, 렌더 전용 외형 메쉬, 잡음 판, Isaac dev 기준선. 이 계획의 조리법·정답·판 2·평가기를 그대로 쓴다.
- **단계 2 — 기하 코어:** `instance_proposals`, `instance_fit`, `pocket_candidate`, `target_selection`, `candidate_tracking`. 평가는 `CandidateView`·`TargetDecision` 으로 변환해서 한다.
- **단계 3 — 학습 제안기(YOLO), 단계 4 — 근거리·실행기·콘솔, 단계 5 — eval 1회, 단계 6 — 임무.**

## 자체 검토

1. **설계 대응:** §5.1 사례 → 작업 8(S7 의 창고 팔레트 자산은 Isaac 전용이라 단계 1B); §5.2 → 작업 2; §5.3 렌더·잡음·TF·정답·분할·누출 방지 → 작업 4·5·7·9·11(Isaac 부분 단계 1B); §3.3 → 작업 6; §3.4 → 작업 10; §9.2·§9.3 → 작업 3(§9.3 해석 보충 포함); §13 → Global Constraints·작업 3·9. `처리 지연`은 `elapsed_s` 기록만. 임무 지표는 단계 6.
2. **자리표시:** TBD·"적절히"·"작업 N 과 같이" 없음. 1판의 S2 중복 식·S7 시험 인자·작업 7 시험의 군더더기 줄은 이 판에서 지웠다.
3. **타입 일관성:** `scene_truths` → `(truths, rendered)` 를 작업 8 시험·작업 11 이 같게 쓴다. `CandidateView`·`TargetDecision`·`judge_scene(meta, truth, instruction_input, …)` 를 작업 10·11 이 같게 쓴다. `FaceTruth.fit`(dict)·`danger_threshold_m` 을 작업 7·10 이 같게 쓴다. `choose_pocket_pair(layout, spacing, blade_width)` 를 작업 1·3 이 같게 쓴다.
4. **검토 기록:**
   - ① 원자료 재계산(1판 직후): 끼움 시험값·지지·천장·적재·적층 높이·장면 수 일치. 지시 배정을 분할 안 순위로, 해당 없는 관문 제거, 기준선을 UBAI 로.
   - ② Codex 적대적 검토(1판, P1 2 · P2 16 · P3 4)와 ③ 독립 검토(1판 코드를 실제로 실행, P1 4 · P2 8 · P3 다수) → 2판에서 처리: 접근 가능한 면만 대응(②1·③5), 대응되지 않은 집기 허용 후보도 위험 판정(②2), 자세 p95·S4/S7 높이 관문 구현(②3), 후보 유한성 검사(②4), 날 여유 기준 쌍 선택(②5), S7 층화로 집을 수 있는 형상 보장(②6·③3), 형상 생성을 구성으로 바꿔 무효 형상 제거(②7), 진입면 노출(②8·③6), S4 지시 재정의(②9·③8), 절대 층 없는 선택·재관측 처리(②10·③11), 정답 스키마에 형상 매개변수·받침 가시성·층 도달(②11·③9), 판 2 로더에 판 1 검증·출처 짝 강제(②12·②13), 분할 우선 로드(②14), 손상 절단 위치(②15), 시험 수집·대역 시험 수정(②16·③1·③2), 알려진 형상 적재 가정을 설계 해석 보충으로(②17), 전 파일 해시·소스 manifest(②18), 쿼터니언 Shepperd(②19), Pillow `mode=` 제거·판 고정(②20·③4), 렌더 재사용(②21), 거리 제곱 잡음 시험(②22), 맞닿음 행(③7), T11 y 면 높이(③10), 지시 입력/정답 분리·기준선이 형상 라벨을 안 씀(③12), 시드 스트림 분리·S3 가로 흔들림·flying pixel 방향·카탈로그 해시 대조(③ P3).
   - 2판 코드 실행 확인(2026-10-15, scratch 사본, 로컬): 작업 1–11 의 시험 전부 통과(장면 조리법 10개 36 s, CLI 통합 2개 32 s). 노출 시험의 가림판을 105 → 120 mm 로 올렸다(105 mm 에서 노출 0.502 로 문턱에 걸침). 예상 시험 수를 실제 수로 고쳤다.
   - 2판에 대한 Codex 재검토: (아래에 기록)
