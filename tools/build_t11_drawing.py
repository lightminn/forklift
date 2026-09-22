"""Export unresolved T11 opening hypotheses as dimensioned SVG and Markdown.

Uses the existing PalletGeometry schema and pallet_boxes assembly. No standard
is fetched, no hypothesis is adopted, and an existing output is never replaced.
"""

import argparse
import hashlib
import inspect
import json
import math
import tempfile
from dataclasses import asdict
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

from forklift_core.perception.pallet_geometry import (
    CARRIAGE_INSERTION_LIMIT_M,
    INSERTION_DEPTH_FRACTION,
    INSERTION_RESERVE_M,
    PalletGeometry,
    check_fork_fit,
    load_pallet_geometry,
    target_insertion_depth_m,
)
from forklift_core.perception.pallet_prior import load_pallet_prior

if __package__:
    from .build_pallet_model import pallet_boxes
else:
    from build_pallet_model import pallet_boxes

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GEOMETRY = ROOT / "config/pallet_geometry_t11_06.yaml"
PARAMETERS = ROOT / "sim/models/dls08_provisional/parameters.yaml"
PRIOR = ROOT / "config/pallet_prior_t11_06.yaml"
ADR2 = ROOT / "docs/decisions/0002-test-pallet-and-geometry-generality.md"
ADR3 = ROOT / "docs/decisions/0003-target-selection-and-blind-zone-insertion.md"
GEOMETRY_CODE = ROOT / "src/forklift_core/perception/pallet_geometry.py"
ASSEMBLY_CODE = GEOMETRY_CODE
# ADR 0002 decision 1: the existing input YAML is already scaled by 0.6.
DEFAULT_SCALE = 0.6
SVG_NS = "http://www.w3.org/2000/svg"


def path_label(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def line_ref(path: Path, needle: str) -> str:
    """Resolve citations against current files, failing if an anchor disappeared."""
    for index, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if needle in line:
            return f"{path_label(path)}:{index}"
    raise ValueError(f"Source anchor missing: {path_label(path)} / {needle}")


def yaml_ref(path: Path, *keys: str) -> str:
    node = yaml.compose(path.read_text(encoding="utf-8"))
    for key in keys:
        key_node, node = next(pair for pair in node.value if pair[0].value == key)
    start = key_node.start_mark.line + 1
    end = node.end_mark.line + bool(node.end_mark.column)
    location = str(start) if start == end else f"{start}–{end}"
    return f"{path_label(path)}:{location} ({'.'.join(keys)})"


def fmt(value: float) -> str:
    return f"{value:.4f}".rstrip("0").rstrip(".") if value else "0"


def mm(value: float) -> float:
    return round(value * 1000, 8)


def serial_geometry(g: PalletGeometry) -> dict:
    data = asdict(g)
    for name in ("block_widths_m", "bottom_board_widths_m"):
        data[name] = list(data[name])
    return data


def derive_geometry(
    source: Path, reading: int, scale: float, source_scale: float
) -> PalletGeometry:
    """Transform the existing schema, then validate it with its existing loader."""
    for name, value in (("scale", scale), ("source_scale", source_scale)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if reading not in (235, 350):
        raise ValueError("opening reading must be 235 or 350")
    g = load_pallet_geometry(source)
    if not math.isclose(g.overall_width_m, g.overall_depth_m, abs_tol=1e-9):
        raise ValueError("T11 hypothesis requires a square input envelope")
    data = serial_geometry(g)
    for name, value in data.items():
        if name.endswith("_m"):
            data[name] = (
                [v * scale / source_scale for v in value]
                if isinstance(value, list)
                else value * scale / source_scale
            )
    # Both interpretations assume three equal supports and two clear openings.
    # Apply the same conditional equation in x and y; do not infer a standard.
    width = (data["overall_width_m"] - 2 * reading / 1000 * scale) / 3
    data["block_widths_m"] = [width] * 3
    data["bottom_board_widths_m"] = [width] * 3
    data["block_depth_m"] = width
    data["geometry_version"] = f"t11_reading_{reading}_scale_{scale:g}_unresolved"
    data["source_provenance"] = (
        "t11_unresolved_opening_hypothesis_not_standard_verified"
    )
    with tempfile.TemporaryDirectory(prefix="t11_geometry_") as temporary:
        candidate = Path(temporary) / "geometry.yaml"
        candidate.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
        return load_pallet_geometry(candidate)


def dimension_rows(
    g: PalletGeometry, source: Path, reading: int, scale: float, source_scale: float
) -> list[dict]:
    """A presentation of PalletGeometry, not a second geometry schema."""
    rows = []
    ratio = f" × ({fmt(scale)}/{fmt(source_scale)}) [--scale/--source-scale]"
    conditional = line_ref(ADR2, "1100 = 3b") + "; " + line_ref(ADR2, "235 로 읽으면")
    assembly = line_ref(ASSEMBLY_CODE, "def pallet_boxes(")

    def add(key, name, value, source_text, numeric_mm=None):
        rows.append(
            dict(id=key, name=name, value=value, source=source_text, mm=numeric_mm)
        )

    for key, name, field in (
        ("W", "전체 폭 y", "overall_width_m"),
        ("D", "전체 깊이 x", "overall_depth_m"),
        ("H", "전체 높이 z", "overall_height_m"),
        ("DB", "하부 판재 두께", "deck_bottom_m"),
        ("BH", "블록 높이 / 로더 개구 높이", "block_height_m"),
        ("ST", "스트링거 두께", "stringer_m"),
        ("TT", "상부 판재 두께", "top_board_thickness_m"),
        ("TW", "상부 판재 폭", "top_board_width_m"),
    ):
        value = mm(getattr(g, field))
        add(key, name, fmt(value), yaml_ref(source, field) + ratio, value)
    add(
        "DT",
        "상부 덱 합계 (스트링거+상판)",
        fmt(mm(g.deck_top_m)),
        "ST + TT",
        mm(g.deck_top_m),
    )
    add(
        "O",
        "두 개구 각각의 순폭 (조건부)",
        fmt(mm(g.opening_width_m)),
        f"CLI --opening-reading {reading} mm × --scale {fmt(scale)}; {conditional}",
        mm(g.opening_width_m),
    )
    add(
        "B",
        "각 블록 폭 y = 깊이 x",
        fmt(mm(g.block_depth_m)),
        f"(W − 2×O)/3; 같은 폭의 지지대 3개 가정; {conditional}",
        mm(g.block_depth_m),
    )
    add(
        "BW",
        "하부 판재 각각의 폭",
        fmt(mm(g.block_depth_m)),
        "B와 동일; " + line_ref(GEOMETRY_CODE, "Bottom boards must sit"),
        mm(g.block_depth_m),
    )
    add(
        "BC",
        "블록 수",
        "3 × 3 = 9",
        yaml_ref(source, "block_widths_m") + "; " + assembly,
    )
    add("SC", "스트링거 수", "3", assembly + "; 각 블록 x행에 하나")
    add("NC", "하부 판재 수", "3", assembly + "; 각 블록 y열에 하나")
    add(
        "TC",
        "상부 판재 수 (설계 선택값)",
        str(g.top_board_count),
        yaml_ref(source, "top_board_count"),
    )
    add(
        "TP",
        "상부 판재 중심 피치",
        fmt(mm(g.top_board_pitch_m)),
        "(W − TW)/(TC − 1)",
        mm(g.top_board_pitch_m),
    )
    gap = g.top_board_pitch_m - g.top_board_width_m
    add("TG", "상부 판재 사이 슬롯", fmt(mm(gap)), "TP − TW", mm(gap))
    add(
        "BZ",
        "블록 중심 z = 포켓 중심 z",
        fmt(mm(g.opening_centre_height_m)),
        "DB + BH/2",
        mm(g.opening_centre_height_m),
    )
    add(
        "P",
        "포켓 중심 y 절댓값",
        fmt(mm(g.opening_centre_offset_m)),
        "(B + O)/2",
        mm(g.opening_centre_offset_m),
    )
    add(
        "PS",
        "포켓 중심 간격",
        fmt(mm(g.opening_centre_spacing_m)),
        "2×P",
        mm(g.opening_centre_spacing_m),
    )
    add(
        "PX",
        "전면 포켓 중심 x",
        fmt(-mm(g.overall_depth_m) / 2),
        "−D/2; 삽입은 전면 −x에서 +x 방향",
        -mm(g.overall_depth_m) / 2,
    )
    add(
        "OZ",
        "로더 개구 z 대역",
        f"{fmt(mm(g.deck_bottom_m))}–{fmt(mm(g.deck_bottom_m + g.block_height_m))}",
        "[DB, DB+BH]; " + line_ref(GEOMETRY_CODE, "def opening_z_band_m"),
    )
    add(
        "FH",
        "정면 통로: 바닥부터 천장까지",
        fmt(mm(g.deck_bottom_m + g.block_height_m)),
        "DB+BH; 정면 개구 아래 판재 없음; " + assembly,
        mm(g.deck_bottom_m + g.block_height_m),
    )
    add(
        "SH",
        "측면 개구 순높이",
        fmt(mm(g.block_height_m)),
        "BH; 측면 아래에는 하부 판재가 가로지름; " + assembly,
        mm(g.block_height_m),
    )
    for key, name, values, formula in (
        (
            "BX",
            "블록 / 스트링거 x 중심",
            g.block_centres_x_m(),
            "[−(D−B)/2, 0, +(D−B)/2]",
        ),
        (
            "BY",
            "블록 / 하부 판재 y 중심",
            g.block_centres_y_m(),
            "[−(W−B)/2, 0, +(W−B)/2]",
        ),
        (
            "TY",
            "상부 판재 y 중심",
            g.top_board_centres_y_m(),
            "−(W−TW)/2 + i×TP; i=0…TC−1",
        ),
    ):
        add(key, name, ", ".join(fmt(mm(v)) for v in values), formula + "; " + assembly)
    return rows


def part_rows(g: PalletGeometry) -> list[dict]:
    source = line_ref(ASSEMBLY_CODE, "def pallet_boxes(")
    formulas = {
        "bottom": "size=(D,BW,DB); centre=(0,BY,DB/2)",
        "block": "size=(B,B,BH); centre=(BX,BY,BZ)",
        "stringer": "size=(B,W,ST); centre=(BX,0,DB+BH+ST/2)",
        "top": "size=(D,TW,TT); centre=(0,TY,H−TT/2)",
    }
    return [
        dict(
            name=box.name,
            size_mm=[mm(v) for v in box.size_m],
            centre_mm=[mm(v) for v in box.centre_m],
            source=formulas[box.name.split("_")[0]] + "; " + source,
        )
        for box in pallet_boxes(g)
    ]


def verification(g: PalletGeometry) -> dict:
    p = yaml.safe_load(PARAMETERS.read_text(encoding="utf-8"))
    d = p["dimensions"]
    fork = {
        "fork_spacing_m": d["fork_spacing_m"],
        "fork_width_m": d["fork_width_m"],
        "fork_thickness_m": d["fork_thickness_m"],
        "fork_centre_height_m": d["fork_center_height_m"],
        "fork_length_m": d["rear_extent_x_m"]
        + p["catalogue"]["overall_length_m"]
        - d["fork_root_x_m"],
        "lift_travel_m": d["lift_travel_m"],
    }
    fit = check_fork_fit(g, **fork)
    ceiling = g.opening_z_band_m[1] - (
        fork["fork_centre_height_m"]
        + fit.lift_required_m
        + fork["fork_thickness_m"] / 2
    )
    prior = load_pallet_prior(PRIOR)

    def range_result(value, low, high, key):
        return dict(
            nominal_m=value,
            min_m=low,
            max_m=high,
            within_range=low <= value <= high,
            outside_by_m=max(low - value, value - high, 0),
            source=yaml_ref(PRIOR, key),
        )

    candidate = g.overall_depth_m * INSERTION_DEPTH_FRACTION
    target = target_insertion_depth_m(g.overall_depth_m)
    return {
        "fork_inputs": fork,
        "fork_input_sources": {
            **{
                key: yaml_ref(
                    PARAMETERS,
                    "dimensions",
                    "fork_center_height_m" if key == "fork_centre_height_m" else key,
                )
                for key in fork
                if key != "fork_length_m"
            },
            "fork_length_m": "rear_extent_x_m + overall_length_m − fork_root_x_m; "
            + "; ".join(
                (
                    yaml_ref(PARAMETERS, "dimensions", "rear_extent_x_m"),
                    yaml_ref(PARAMETERS, "catalogue", "overall_length_m"),
                    yaml_ref(PARAMETERS, "dimensions", "fork_root_x_m"),
                )
            ),
        },
        "fork_fit": {
            **asdict(fit),
            "fits": fit.fits,
            "ceiling_margin_m": ceiling,
            "floor_clearance_m": inspect.signature(check_fork_fit)
            .parameters["floor_clearance_m"]
            .default,
            "source": line_ref(GEOMETRY_CODE, "def check_fork_fit("),
        },
        "prior": {
            "opening_width": range_result(
                g.opening_width_m,
                prior.opening_width_min_m,
                prior.opening_width_max_m,
                "opening_width_range",
            ),
            "centre_spacer": range_result(
                g.centre_block_width_m,
                prior.centre_spacer_min_m,
                prior.centre_spacer_max_m,
                "centre_spacer_range",
            ),
        },
        "insertion": {
            "depth_fraction": INSERTION_DEPTH_FRACTION,
            "depth_fraction_candidate_m": candidate,
            "carriage_limit_m": CARRIAGE_INSERTION_LIMIT_M,
            "reserve_m": INSERTION_RESERVE_M,
            "target_m": target,
            "remaining_to_carriage_m": CARRIAGE_INSERTION_LIMIT_M - target,
            "source": "사용자 지정 min(D×0.6, 406−46) mm; "
            + line_ref(ADR3, "최대 삽입 406 mm, 목표 깊이 360 mm")
            + "; 406 = 950−544; 46 = 406−360",
        },
    }


def markdown_table(headers: list[str], rows: list[list[str]]) -> str:
    return "\n".join(
        "| " + " | ".join(row) + " |"
        for row in [headers, ["---"] * len(headers), *rows]
    )


def dimensions_markdown(report: dict) -> str:
    rows, parts = report["dimensions"], report["parts"]
    by_id = {row["id"]: row for row in rows}
    fit, insertion = report["fork_fit"], report["insertion"]
    header = f"""# T11 ×{fmt(report["scale"])} — {report["opening_reading_mm"]} mm 순폭 해석 (미채택)

**어느 쪽도 채택하지 않았다. 표준 원문 확인 필요.** 아래 값은 기존 YAML의 설계 선택값과
CLI로 지정한 순폭 해석의 조건부 유도값이다. 표준 인증치·실측치·출력 허용공차가 아니다.
도면: [drawing.svg](drawing.svg), [차이표](comparison.md), [동일 스키마의 해석 사본](geometry.yaml).

단위는 mm. 계산은 SI, 표시만 mm로 변환(m × 1000). 좌표 원점은 바닥 외형 중심,
x는 삽입 방향, y는 전면 폭, z는 위다. 전면 x=−D/2에서 +x로 진입한다.
근거: {line_ref(DEFAULT_GEOMETRY, "# Axes:")}. SVG는 mm 치수값을 읽으며 인쇄 배율로 치수를 재지 않는다.

입력 축척 {fmt(report["source_scale"])} → 출력 축척 {fmt(report["scale"])}.
입력 YAML 자체가 이미 축소되어 있으므로 기존 길이에는 --scale/--source-scale만 곱한다.
삽입구 CLI 값은 원형 후보치이므로 --scale을 한 번 곱한다.
기본 축척의 근거: {line_ref(ADR2, "### 1. 시험용 팔레트")}.

입력 출처: `{report["sources"]["geometry"]}` (SHA-256 `{report["sources"]["geometry_sha256"]}`).
기존 YAML 주석에는 원형 높이 **144 mm**가 있지만 실제 필드 90/0.6은 **150 mm**다
({line_ref(DEFAULT_GEOMETRY, "1100 x 1100 x 144")}; {yaml_ref(DEFAULT_GEOMETRY, "overall_height_m")};
{line_ref(ADR2, "1100 × 1100 × 150")}). 이 불일치를 해결하지 않았으며 실제 필드를 사용한다.
판재 두께·상판 개수·폭은 기존 설계 선택값이다. 모서리 형상·체결부·분할 접합부·재료·공차는 미정이다.

## 치수와 출처

{markdown_table(["ID", "항목", "값 (mm, 개수는 별도 표기)", "출처 / 유도식"], [[r["id"], r["name"], r["value"], r["source"]] for r in rows])}

로더 개구 대역은 z={by_id["OZ"]["value"]} mm이고 높이는 BH다.
실제 정면 통로는 z=0–{by_id["FH"]["value"]} mm이며 아래 판재가 없다. 측면에는 아래 판재가 있으므로
개구 높이는 SH다. 정사각 외형과 같은 블록 배치만으로 덱까지 회전 대칭인 것은 아니다.
포켓 좌표는 (PX, ±P, BZ); 이는 로더 기준점이며 바닥까지 열린 통로의 면적 중심과 구별한다.

## 부품별 명목 치수와 배치

각 행은 조립 박스 하나다. size는 (x 길이, y 폭, z 높이), centre는 (x, y, z).
모든 위치와 모서리는 centre ± size/2로 재현한다. 판재의 방향과 아래 열린 통로를 바꾸지 않는다.

{markdown_table(["부품", "size (mm)", "centre (mm)", "출처 / 유도식"], [[p["name"], ", ".join(fmt(v) for v in p["size_mm"]), ", ".join(fmt(v) for v in p["centre_mm"]), p["source"]] for p in parts])}

## 포크 적합성 (정렬된 블레이드의 정적 기하)

{markdown_table(["입력", "값 (mm)", "출처"], [[k, fmt(mm(v)), report["fork_input_sources"][k]] for k, v in report["fork_inputs"].items()])}

포크 치수는 사진 비율 추정이며 실측이 아니다: {yaml_ref(PARAMETERS, "evidence", "dimensions")}.
`check_fork_fit` 호출 결과 fits={fit["fits"]}, lateral_ok={fit["lateral_ok"]}, vertical_ok={fit["vertical_ok"]}.
안쪽 여유 {fmt(mm(fit["lateral_inner_margin_m"]))} mm, 바깥쪽 여유 {fmt(mm(fit["lateral_outer_margin_m"]))} mm,
천장 여유 {fmt(mm(fit["ceiling_margin_m"]))} mm, 필요 승강 {fmt(mm(fit["lift_required_m"]))} mm.
근거: {fit["source"]}; 안쪽=(포크 중심간격−포크 폭−B)/2,
바깥쪽=B/2+O−(포크 중심간격+포크 폭)/2, 천장=DB+BH−(포크 중심높이+승강+포크 두께/2).
함수의 기본 floor_clearance={fmt(mm(fit["floor_clearance_m"]))} mm는 로더 개구 하단 기준의 계산 조건이며
출력 공차가 아니다. 블레이드 도달률 {fmt(fit["reach_fraction"])}=포크 길이/D는 캐리지 간섭을 검사하지 않는다.

## 기존 prior 범위 대조 (검출 실행 결과 아님)

{markdown_table(["항목", "해석 명목값 (mm)", "기존 범위 (mm)", "범위 안", "범위 밖 거리 (mm)", "출처"], [[k, fmt(mm(v["nominal_m"])), f"[{fmt(mm(v['min_m']))}, {fmt(mm(v['max_m']))}]", str(v["within_range"]), fmt(mm(v["outside_by_m"])), v["source"]] for k, v in report["prior"].items()])}

위 표는 고정된 기존 prior와 직접 비교한다. prior를 바꾸거나 실센서 검출을 실행한 결과가 아니다.
실제 검출기에는 격자 경계 보정도 있다: 중앙 지지대는 ±2×cell, 개구 하한은 −2×cell
({line_ref(ROOT / "src/forklift_core/perception/pocket_detector.py", "prior.centre_spacer_min_m - 2 * params.cell_m")};
{line_ref(ROOT / "src/forklift_core/perception/pocket_detector.py", "not prior.opening_width_min_m - 2 * params.cell_m")}).
인쇄 후 측정치로 prior를 재생성해야 한다({line_ref(ADR2, "prior 는 설계 YAML 이 아니라")}).

## 삽입 깊이

D×{fmt(insertion["depth_fraction"])}={fmt(mm(insertion["depth_fraction_candidate_m"]))} mm,
캐리지 한계−여유={fmt(mm(insertion["carriage_limit_m"]))}−{fmt(mm(insertion["reserve_m"]))}={fmt(mm(insertion["carriage_limit_m"] - insertion["reserve_m"]))} mm.
목표=min(두 값)={fmt(mm(insertion["target_m"]))} mm,
캐리지까지 남는 거리={fmt(mm(insertion["remaining_to_carriage_m"]))} mm.
출처: {insertion["source"]}. 삽입 비율과 팔레트 축척은 별개다.
이는 ADR의 잠정 차체·정렬 자세 규칙 적용이며 궤적·접촉·변형·하중·실물 승강 검증이 아니다.

## 채택·출력 전에 확인

- 표준 번호·판본·도면을 특정하고 그 도면의 235 / 350 mm가 재는 대상을 확인한다.
  개구 순폭인지 블록 간 중심거리인지 확정하기 전에는 어느 쪽도 회귀 정본으로 사용하지 않는다.
  출처: {line_ref(ADR2, "235 / 350 중 하나를 고르는 단계가 아니다")}.
- 원형 높이와 판재/블록 세부 치수는 표준 원문 확인 필요. 현재 판재 선택값을 표준치로 표시하지 않는다.
- 출력 공차: 장비·재료별 수축, 휨, 분할 접합 누적오차를 측정한다. 수치 공차는 아직 정하지 않았다.
  외형, 모든 부품의 size/centre, 개구 최소 폭·천장, 포켓 간격을 조립 후 검수한다.
- 적층 방향: 상판의 휨과 포크 접촉, 블록 접합부의 전단·층간 분리를 고려해 시험편으로 결정한다.
  이 도면에는 적층 방향·레이어 높이·인필·벽 수·접합 상세를 임의 지정하지 않았다.
- 하중 방향: 포크의 위쪽 지지력, 화물의 아래쪽 하중, 삽입 시 수평 접촉력을 나누어 확인한다.
  허용 하중·안전계수·내구 횟수는 미정이며 정적 기하 적합 판정으로 보증하지 않는다.
- 인쇄 전 실제 포크 두께·중심 높이·간격·승강 행정을 측정한다:
  {line_ref(ADR2, "인쇄 착수의 선행 조건")}.
"""
    return header


def comparison_markdown(reports: dict[int, dict]) -> str:
    left, right = reports[235], reports[350]
    r235 = {r["id"]: r for r in left["dimensions"]}
    r350 = {r["id"]: r for r in right["dimensions"]}
    rows = []
    for key in (
        "W",
        "D",
        "H",
        "DB",
        "BH",
        "ST",
        "TT",
        "DT",
        "B",
        "BW",
        "O",
        "P",
        "PS",
        "BZ",
        "TW",
        "TP",
        "TG",
        "FH",
        "SH",
    ):
        a, b = r235[key], r350[key]
        rows.append(
            [
                a["name"],
                a["value"],
                b["value"],
                fmt(a["mm"] - b["mm"]),
                f"{key}: {a['source']}"
                if a["source"] == b["source"]
                else f"235: {a['source']}; 350: {b['source']}",
            ]
        )
    for key in ("BX", "BY", "TY"):
        a, b = r235[key], r350[key]
        av, bv = (list(map(float, r["value"].split(", "))) for r in (a, b))
        rows.append(
            [
                a["name"],
                a["value"],
                b["value"],
                ", ".join(fmt(x - y) for x, y in zip(av, bv, strict=True)),
                a["source"],
            ]
        )
    for key, name in (
        ("lateral_inner_margin_m", "포크 안쪽 여유"),
        ("lateral_outer_margin_m", "포크 바깥쪽 여유"),
        ("ceiling_margin_m", "포크 천장 여유"),
    ):
        a, b = left["fork_fit"][key], right["fork_fit"][key]
        rows.append(
            [
                name,
                fmt(mm(a)),
                fmt(mm(b)),
                fmt(mm(a - b)),
                left["fork_fit"]["source"] + "; 각 dimensions.md의 포크 계산식",
            ]
        )
    for key, name in (
        ("depth_fraction_candidate_m", "깊이 비례 삽입 후보"),
        ("target_m", "삽입 목표"),
        ("remaining_to_carriage_m", "캐리지까지 잔여"),
    ):
        a, b = left["insertion"][key], right["insertion"][key]
        rows.append(
            [name, fmt(mm(a)), fmt(mm(b)), fmt(mm(a - b)), left["insertion"]["source"]]
        )
    return f"""# 두 순폭 해석의 차이 — 축척 {fmt(left["scale"])}

**어느 쪽도 채택하지 않았다. 표준 원문 확인 필요.** 두 값은 CLI로 선택한 가설이며 표준 회귀 상수가 아니다.
단위 mm; 차이는 235 해석 − 350 해석. 부품 수·상판 배치는 입력 YAML 그대로이며
블록과 하부 판재 폭, 블록 깊이 및 이에 연동된 스트링거 길이와 중심 위치가 바뀐다.
각 ID의 뜻과 모든 원자료는 같은 실행의 dimensions.md / verification.json에서 확인한다.

{markdown_table(["항목", "235 해석", "350 해석", "차이", "출처 / 유도식"], rows)}

포크 fits: 235={left["fork_fit"]["fits"]}, 350={right["fork_fit"]["fits"]}
(기존 check_fork_fit, 정렬된 잠정 블레이드 기하만).
기존 prior 개구 폭 범위 안: 235={left["prior"]["opening_width"]["within_range"]}, 350={right["prior"]["opening_width"]["within_range"]}.
기존 prior 중앙 지지대 범위 안: 235={left["prior"]["centre_spacer"]["within_range"]}, 350={right["prior"]["centre_spacer"]["within_range"]}.
출처: {yaml_ref(PRIOR, "opening_width_range")}; {yaml_ref(PRIOR, "centre_spacer_range")}.
"""


def drawing_svg(g: PalletGeometry, report: dict) -> str:
    """Orthographic projections of the shared boxes, dimensioned in millimetres."""
    ET.register_namespace("", SVG_NS)

    def element(parent, tag, **attrs):
        return ET.SubElement(
            parent,
            f"{{{SVG_NS}}}{tag}",
            {k.replace("_", "-"): str(v) for k, v in attrs.items()},
        )

    root = ET.Element(
        f"{{{SVG_NS}}}svg",
        {"viewBox": "0 0 1800 1450", "width": "1800", "height": "1450"},
    )
    element(
        root, "title"
    ).text = f"T11 opening reading {report['opening_reading_mm']} mm — unresolved"
    element(
        root, "desc"
    ).text = "Dimensioned plan, front and side projections. Nominal mm; not a certified standard or an approved print design. Each dimension references the dimension table; full sources are embedded below."
    element(root, "metadata").text = json.dumps(report, ensure_ascii=False)
    element(root, "style").text = """
text {font-family: 'DejaVu Sans',sans-serif; fill:#182d3b; font-size:18px}
.heading {font-size:26px; font-weight:bold} .small {font-size:16px}
.dimension {stroke:#18374b; stroke-width:1.2; fill:none}
.extension {stroke:#78909e; stroke-width:0.9}
.dim-label {font-size:17px; paint-order:stroke; stroke:white; stroke-width:5px; stroke-linejoin:round}
.hidden {stroke:#806040; stroke-width:1.4; stroke-dasharray:7 5; fill:none}
.outline {stroke:#182d3b; stroke-width:1.6}
.pocket {stroke:#007e85; stroke-width:2; fill:white}
.centre {stroke:#007e85; stroke-width:1.2; stroke-dasharray:12 4 2 4}
"""
    defs = element(root, "defs")
    for name, path in (
        ("start", "M 8,0 L 0,4 L 8,8 Z"),
        ("end", "M 0,0 L 8,4 L 0,8 Z"),
    ):
        marker = element(
            defs,
            "marker",
            id=name,
            markerWidth=8,
            markerHeight=8,
            refX=0 if name == "start" else 8,
            refY=4,
            orient="auto",
            markerUnits="userSpaceOnUse",
        )
        element(marker, "path", d=path, fill="#18374b")
    element(root, "rect", x=0, y=0, width=1800, height=1450, fill="white")
    rows = {row["id"]: row for row in report["dimensions"]}

    def text(parent, x, y, value, css="", **attrs):
        node = element(parent, "text", x=fmt(x), y=fmt(y), **attrs)
        if css:
            node.set("class", css)
        node.text = value
        return node

    def line(parent, x1, y1, x2, y2, css="extension", **attrs):
        node = element(
            parent, "line", x1=fmt(x1), y1=fmt(y1), x2=fmt(x2), y2=fmt(y2), **attrs
        )
        node.set("class", css)
        return node

    def dimension(parent, start, end, offset, key, label=None):
        """Parallel extension lines and inward arrows; offset in sheet pixels."""
        x1, y1 = start
        x2, y2 = end
        dx, dy = offset
        line(parent, x1, y1, x1 + dx, y1 + dy)
        line(parent, x2, y2, x2 + dx, y2 + dy)
        node = line(
            parent,
            x1 + dx,
            y1 + dy,
            x2 + dx,
            y2 + dy,
            "dimension",
            marker_start="url(#start)",
            marker_end="url(#end)",
        )
        node.set("data-source", f"{key}: {rows[key]['source']}")
        tx, ty = (x1 + x2) / 2 + dx, (y1 + y2) / 2 + dy
        attrs = {"text_anchor": "middle"}
        if abs(x1 - x2) < 1e-8:
            attrs["transform"] = f"rotate(-90 {fmt(tx - 8)} {fmt(ty)})"
            tx -= 8
        else:
            ty -= 9
        text(
            parent,
            tx,
            ty,
            f"{label or rows[key]['value']} [{key}]",
            "dim-label",
            **attrs,
        )

    text(
        root,
        55,
        58,
        f"T11 / opening reading {report['opening_reading_mm']} mm / scale {fmt(report['scale'])}",
        "heading",
    )
    text(
        root,
        55,
        94,
        "UNRESOLVED HYPOTHESIS — neither reading adopted. Nominal dimensions in mm; no print tolerances assigned.",
    )
    text(
        root,
        55,
        122,
        "Sources: [IDs] refer to dimensions.md; complete sources and part coordinates embedded in SVG metadata.",
        "small",
    )
    line(root, 45, 145, 1755, 145)
    line(root, 945, 165, 945, 1385)
    # Keep the drawing visually comparable at other article scales. Only labels
    # carry physical dimensions; sheet pixel units are never fabrication units.
    k = 580 / mm(g.overall_width_m)
    w, d, h = mm(g.overall_width_m), mm(g.overall_depth_m), mm(g.overall_height_m)
    cx, cy, front_floor, side_cx, side_floor = 455, 515, 1170, 1320, 375
    boxes = pallet_boxes(g)
    colours = {
        "bottom": "#c5dbe8",
        "block": "#e0c6a4",
        "stringer": "#b6d7c0",
        "top": "#e5edf1",
    }

    def project(view, centre):
        x, y, z = map(mm, centre)
        if view == "plan":
            return cx - y * k, cy - x * k
        if view == "front":
            return cx - y * k, front_floor - z * k
        return side_cx + x * k, side_floor - z * k

    groups = {}
    for view, title, title_y in (
        ("plan", "PLAN / viewed from +z; +x up, +y left", 172),
        ("front", "FRONT / viewed from -x; +y left, +z up", 1012),
        ("side", "SIDE / viewed from -y; +x right, +z up", 190),
    ):
        group = element(root, "g", id=view)
        groups[view] = group
        text(
            group,
            75 if view != "side" else 985,
            title_y,
            title,
            "heading" if view == "front" else "",
        )
        # Far-to-near for the selected view. Plan shows upper boards, followed
        # by a dashed overlay of hidden block/stringer edges.
        axis, reverse = {"plan": (2, False), "front": (0, True), "side": (1, True)}[
            view
        ]
        for box in sorted(boxes, key=lambda b: b.centre_m[axis], reverse=reverse):
            px, py = project(view, box.centre_m)
            sx, sy, sz = map(mm, box.size_m)
            width, height = {"plan": (sy, sx), "front": (sy, sz), "side": (sx, sz)}[
                view
            ]
            shape = element(
                group,
                "rect",
                x=fmt(px - width * k / 2),
                y=fmt(py - height * k / 2),
                width=fmt(width * k),
                height=fmt(height * k),
                fill=colours[box.name.split("_")[0]],
                data_part=box.name,
            )
            shape.set("class", "outline")
        if view == "plan":
            for box in boxes:
                if not box.name.startswith(("block_", "stringer_")):
                    continue
                px, py = project(view, box.centre_m)
                sx, sy, _ = map(mm, box.size_m)
                shape = element(
                    group,
                    "rect",
                    x=fmt(px - sy * k / 2),
                    y=fmt(py - sx * k / 2),
                    width=fmt(sy * k),
                    height=fmt(sx * k),
                )
                shape.set("class", "hidden")
        else:
            centre = cx if view == "front" else side_cx
            floor = front_floor if view == "front" else side_floor
            line(group, centre - w * k / 2 - 10, floor, centre + w * k / 2 + 10, floor)
    plan, front, side = (groups[v] for v in ("plan", "front", "side"))
    left, right, top, bottom = (
        cx - w * k / 2,
        cx + w * k / 2,
        cy - d * k / 2,
        cy + d * k / 2,
    )
    dimension(plan, (left, top), (right, top), (0, -26), "W")
    dimension(plan, (left, top), (left, bottom), (-72, 0), "D")
    line(plan, cx, top - 6, cx, bottom + 9, "centre")
    line(plan, left - 6, cy, right + 9, cy, "centre")
    text(plan, cx + 8, cy - 8, "O (floor-centred origin)", "small")
    text(
        plan,
        cx,
        bottom + 34,
        "FRONT (-x) / insertion toward +x",
        "small",
        text_anchor="middle",
    )
    # Along-depth block/opening chain, separate from the overall dimension.
    block, opening = mm(g.block_depth_m), mm(g.opening_width_m)
    position = top
    for value, key in (
        (block, "B"),
        (opening, "O"),
        (block, "B"),
        (opening, "O"),
        (block, "B"),
    ):
        dimension(plan, (right, position), (right, position + value * k), (56, 0), key)
        position += value * k
    # Top-board width is dimensioned; pitch and narrow slot use leaders to avoid
    # printing illegible labels inside a small slot.
    tw = mm(g.top_board_width_m)
    dimension(plan, (left, bottom), (left + tw * k, bottom), (0, 78), "TW")
    dimension(
        plan,
        (left + tw * k / 2, bottom),
        (left + tw * k / 2 + mm(g.top_board_pitch_m) * k, bottom),
        (0, 120),
        "TP",
    )
    slot_x = left + tw * k + mm(g.top_board_pitch_m - g.top_board_width_m) * k / 2
    line(plan, slot_x, top + 65, slot_x + 130, top + 100)
    text(plan, slot_x + 138, top + 107, f"slot {rows['TG']['value']} [TG]", "dim-label")
    text(
        plan,
        110,
        978,
        "Dashed: blocks / stringers below top boards. Colours identify parts, not materials.",
        "small",
    )
    # Front pocket landmarks are the loader's reference points, not the centroid
    # of the taller floor-open physical aperture.
    for sign in (-1, 1):
        px, py = project(
            "front", (0, sign * g.opening_centre_offset_m, g.opening_centre_height_m)
        )
        point = element(
            front,
            "circle",
            cx=fmt(px),
            cy=fmt(py),
            r=5,
            data_y_mm=fmt(sign * mm(g.opening_centre_offset_m)),
            data_z_mm=fmt(mm(g.opening_centre_height_m)),
        )
        point.set("class", "pocket")
        line(front, px - 12, py, px + 12, py, "centre")
        line(front, px, py - 12, px, py + 12, "centre")
    pocket_offset = mm(g.opening_centre_offset_m) * k
    pocket_z = front_floor - mm(g.opening_centre_height_m) * k
    dimension(
        front,
        (cx - pocket_offset, pocket_z),
        (cx + pocket_offset, pocket_z),
        (0, -94),
        "PS",
    )
    position = left
    for value, key in (
        (block, "B"),
        (opening, "O"),
        (block, "B"),
        (opening, "O"),
        (block, "B"),
    ):
        dimension(
            front,
            (position, front_floor),
            (position + value * k, front_floor),
            (0, 49),
            key,
        )
        position += value * k
    dimension(front, (left, front_floor), (left, front_floor - h * k), (-52, 0), "H")
    dimension(
        front,
        (right, front_floor),
        (right, front_floor - mm(g.deck_bottom_m + g.block_height_m) * k),
        (48, 0),
        "FH",
    )
    text(
        front,
        110,
        1270,
        f"Pocket datum: (x, y, z) = ({rows['PX']['value']}, +/-{rows['P']['value']}, {rows['BZ']['value']}) [PX,P,BZ]",
        "small",
    )
    text(
        front,
        110,
        1300,
        f"Front open to floor: z=0..{rows['FH']['value']} [FH]. Loader band: z={rows['OZ']['value']} [OZ].",
        "small",
    )
    text(
        front,
        110,
        1330,
        "No lower rail below the front openings; do not add a continuous bottom slab.",
        "small",
    )
    sl, sr = side_cx - d * k / 2, side_cx + d * k / 2
    dimension(side, (sl, side_floor), (sr, side_floor), (0, 53), "D")
    dimension(side, (sr, side_floor), (sr, side_floor - h * k), (88, 0), "H")
    # Layer stack: tiny dimensions get staggered, externally placed labels.
    z = 0
    for index, (height, key) in enumerate(
        (
            (g.deck_bottom_m, "DB"),
            (g.block_height_m, "BH"),
            (g.stringer_m, "ST"),
            (g.top_board_thickness_m, "TT"),
        )
    ):
        y1, y2 = side_floor - mm(z) * k, side_floor - mm(z + height) * k
        offset = 25 if index % 2 == 0 else 55
        line(side, sl, y1, sl - offset, y1)
        line(side, sl, y2, sl - offset, y2)
        arrow = line(
            side,
            sl - offset,
            y1,
            sl - offset,
            y2,
            "dimension",
            marker_start="url(#start)",
            marker_end="url(#end)",
        )
        arrow.set("data-source", rows[key]["source"])
        label_y = 215 + index * 23
        line(side, sl - offset, (y1 + y2) / 2, 1028, label_y - 5)
        text(
            side,
            1040,
            label_y,
            f"{rows[key]['value']} [{key}]",
            "dim-label",
            text_anchor="start",
        )
        z += height
    text(side, 1010, 470, "Side opening has a lower rail: height [SH] = [BH].", "small")
    line(root, 980, 502, 1740, 502)
    text(root, 985, 548, "PART SCHEDULE / size x × y × z (mm)", "heading")
    schedule = [
        (
            "BOTTOM",
            "NC",
            f"{rows['D']['value']} × {rows['BW']['value']} × {rows['DB']['value']}",
            "D, BW, DB",
        ),
        (
            "BLOCK",
            "BC",
            f"{rows['B']['value']} × {rows['B']['value']} × {rows['BH']['value']}",
            "B, BH",
        ),
        (
            "STRINGER",
            "SC",
            f"{rows['B']['value']} × {rows['W']['value']} × {rows['ST']['value']}",
            "B, W, ST",
        ),
        (
            "TOP",
            "TC",
            f"{rows['D']['value']} × {rows['TW']['value']} × {rows['TT']['value']}",
            "D, TW, TT",
        ),
    ]
    for index, (name, count, sizes, ids) in enumerate(schedule):
        y = 594 + index * 76
        text(root, 990, y, f"{name}   {rows[count]['value']} pcs [{count}]")
        text(root, 990, y + 27, f"{sizes}    [{ids}]", "small")
    text(root, 990, 927, "CENTRE COORDINATES (mm)", "heading")
    text(root, 990, 964, f"Blocks x [BX]: {rows['BX']['value']}")
    text(root, 990, 999, f"Blocks y [BY]: {rows['BY']['value']}")
    text(root, 990, 1034, f"Blocks z [BZ]: {rows['BZ']['value']}")
    text(
        root,
        990,
        1069,
        f"Top y [TY]: start -(W-TW)/2; pitch {rows['TP']['value']} [TP]",
    )
    text(
        root,
        990,
        1104,
        "All individual part centres: dimensions.md / SVG metadata.",
        "small",
    )
    line(root, 980, 1135, 1740, 1135)
    for y, value in (
        (1174, "Before adoption: identify standard, edition and drawing."),
        (1204, "Confirm what 235 / 350 measures; verify source height conflict."),
        (1234, "Measure printed size, warp, joints and fork dimensions."),
        (1264, "Layer direction, material, joints and load capacity: unresolved."),
        (1294, "These are nominal inspection views, not slicer files or load proof."),
        (1324, "See dimensions.md for assumptions, fit and insertion checks."),
    ):
        text(root, 990, y, value, "small")
    line(root, 45, 1385, 1755, 1385)
    text(
        root,
        55,
        1420,
        "T11 TEST ARTICLE / CONDITIONAL DRAWING     |     Floor-centred coordinates     |     Read dimension values; not a 1:1 print",
        "small",
    )
    ET.indent(root)
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        + ET.tostring(root, encoding="unicode")
        + "\n"
    )


def build_bundle(
    source: Path, reading: int, scale: float, source_scale: float, output: Path
) -> None:
    """Compute both hypotheses; write only the selected bundle into a new folder."""
    if output.exists():
        raise ValueError(f"Output already exists; refusing overwrite: {output}")
    reports, geometries = {}, {}
    sources = {
        "geometry": path_label(source),
        "geometry_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "files_sha256": {
            path_label(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (
                source,
                PARAMETERS,
                PRIOR,
                ADR2,
                ADR3,
                GEOMETRY_CODE,
                ASSEMBLY_CODE,
                Path(__file__),
            )
        },
    }
    for interpretation in (235, 350):
        g = derive_geometry(source, interpretation, scale, source_scale)
        geometries[interpretation] = g
        reports[interpretation] = {
            "opening_reading_mm": interpretation,
            "scale": scale,
            "source_scale": source_scale,
            "adopted": False,
            "evidence_kind": "conditional_static_geometry_not_standard_or_physical_validation",
            "sources": sources,
            "dimensions": dimension_rows(
                g, source, interpretation, scale, source_scale
            ),
            "parts": part_rows(g),
            **verification(g),
        }
    report, g = reports[reading], geometries[reading]
    contents = {
        "geometry.yaml": "# UNRESOLVED hypothesis; not adopted, not verified against a standard.\n# Derived only for drawing; do not replace the runtime geometry/prior.\n"
        + yaml.safe_dump(serial_geometry(g), sort_keys=False),
        "verification.json": json.dumps(
            report, indent=2, ensure_ascii=False, allow_nan=False
        )
        + "\n",
        "dimensions.md": dimensions_markdown(report),
        "comparison.md": comparison_markdown(reports),
        "drawing.svg": drawing_svg(g, report),
    }
    output.mkdir(parents=True, exist_ok=False)
    for name, content in contents.items():
        with (output / name).open("x", encoding="utf-8") as stream:
            stream.write(content)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--opening-reading",
        type=int,
        choices=(235, 350),
        required=True,
        help="Unresolved full-size clear-opening hypothesis in mm; neither is adopted",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=DEFAULT_SCALE,
        help="Output article scale (default: 0.6)",
    )
    parser.add_argument(
        "--geometry",
        type=Path,
        default=DEFAULT_GEOMETRY,
        help="Existing PalletGeometry YAML; default is the already scaled t11_06 input",
    )
    parser.add_argument(
        "--source-scale",
        type=float,
        default=DEFAULT_SCALE,
        help="Scale already applied in input YAML (default: 0.6), not inferred from its name",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="NEW output directory; existing paths are rejected",
    )
    args = parser.parse_args(argv)
    try:
        build_bundle(
            args.geometry,
            args.opening_reading,
            args.scale,
            args.source_scale,
            args.output,
        )
    except (ValueError, OSError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    print(f"Unresolved {args.opening_reading} reading: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
