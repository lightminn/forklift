"""The two unresolved readings remain separate, inspectable hypotheses."""

import hashlib
import json
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import yaml

from forklift_core.perception.pallet_geometry import load_pallet_geometry

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools/build_t11_drawing.py"
GEOMETRY = ROOT / "config/pallet_geometry_t11_06.yaml"
NS = {"s": "http://www.w3.org/2000/svg"}


def run_drawing(output, reading, *extra):
    return subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--opening-reading",
            str(reading),
            "--output",
            str(output),
            *extra,
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


@pytest.mark.parametrize(
    "reading,block,opening,offset,inner,outer,prior_ok",
    [
        (235, 0.126, 0.141, 0.1335, 0.0545, 0.0315, False),
        (350, 0.080, 0.210, 0.1450, 0.0775, 0.0775, True),
    ],
)
def test_both_hypotheses_export_dimensions_fit_and_real_svg(
    tmp_path, reading, block, opening, offset, inner, outer, prior_ok
):
    original = GEOMETRY.read_bytes()
    out = tmp_path / str(reading)
    result = run_drawing(out, reading)
    assert result.returncode == 0, result.stderr
    g = load_pallet_geometry(out / "geometry.yaml")
    assert g.overall_width_m == pytest.approx(0.660)
    assert g.overall_depth_m == pytest.approx(0.660)
    assert g.overall_height_m == pytest.approx(0.090)
    assert g.block_widths_m == pytest.approx([block] * 3)
    assert g.block_depth_m == pytest.approx(block)
    assert g.bottom_board_widths_m == pytest.approx([block] * 3)
    assert g.opening_width_m == pytest.approx(opening)
    assert g.opening_centre_offset_m == pytest.approx(offset)
    assert g.opening_z_band_m == pytest.approx((0.015, 0.060))
    assert "unresolved" in g.source_provenance
    report = json.loads((out / "verification.json").read_text())
    assert report["adopted"] is False
    assert report["fork_fit"]["fits"] is True
    assert report["fork_fit"]["lateral_inner_margin_m"] == pytest.approx(inner)
    assert report["fork_fit"]["lateral_outer_margin_m"] == pytest.approx(outer)
    assert report["fork_fit"]["ceiling_margin_m"] == pytest.approx(0.008)
    assert report["fork_fit"]["lift_required_m"] == pytest.approx(0)
    assert report["prior"]["opening_width"]["within_range"] is prior_ok
    assert report["prior"]["centre_spacer"]["within_range"] is prior_ok
    if reading == 235:
        assert report["prior"]["opening_width"]["outside_by_m"] == pytest.approx(0.049)
        assert report["prior"]["centre_spacer"]["outside_by_m"] == pytest.approx(0.031)
    assert report["insertion"]["depth_fraction_candidate_m"] == pytest.approx(0.396)
    assert report["insertion"]["target_m"] == pytest.approx(0.360)
    assert report["insertion"]["remaining_to_carriage_m"] == pytest.approx(0.046)
    assert report["sources"]["geometry_sha256"] == hashlib.sha256(original).hexdigest()
    assert GEOMETRY.read_bytes() == original
    assert all(row["source"] for row in report["dimensions"])
    assert all(part["source"] for part in report["parts"])
    assert len(report["parts"]) == 22
    # Check the actual exported boxes against the blade swept volume, independent
    # of check_fork_fit's scalar clearance calculation.
    for part in report["parts"]:
        centre, size = part["centre_mm"], part["size_mm"]
        bounds = [(c - s / 2, c + s / 2) for c, s in zip(centre, size, strict=True)]
        for fork_y in (-145, 145):
            blade = [(-330, 30), (fork_y - 27.5, fork_y + 27.5), (28, 52)]
            overlap = [
                min(a[1], b[1]) - max(a[0], b[0])
                for a, b in zip(bounds, blade, strict=True)
            ]
            assert not all(value > 0 for value in overlap), part["name"]
    svg = ET.parse(out / "drawing.svg").getroot()
    for view in ("plan", "front", "side"):
        assert svg.find(f".//s:g[@id='{view}']", NS) is not None
    lines = svg.findall(".//s:line[@class='dimension']", NS)
    assert len(lines) >= 20
    assert all(line.get("marker-start") and line.get("marker-end") for line in lines)
    assert all(line.get("data-source") for line in lines)
    assert svg.find(".//s:metadata", NS) is not None
    for pocket in svg.findall(".//s:circle[@class='pocket']", NS):
        assert abs(float(pocket.get("data-y-mm"))) == pytest.approx(offset * 1000)
        assert float(pocket.get("data-z-mm")) == pytest.approx(37.5)
    assert len(svg.findall(".//s:circle[@class='pocket']", NS)) == 2
    md = (out / "dimensions.md").read_text()
    assert "표준 원문 확인 필요" in md
    assert "144" in md and "150" in md
    assert "0–60" in md and "15–60" in md
    comparison = (out / "comparison.md").read_text()
    assert "235" in comparison and "350" in comparison and "-69" in comparison
    assert "어느 쪽도 채택하지 않았다" in comparison
    assert "CLI --opening-reading 235" in comparison
    assert "CLI --opening-reading 350" in comparison


def test_scale_changes_pallet_once_but_not_insertion_fraction_or_forks(tmp_path):
    out = tmp_path / "half"
    result = run_drawing(out, 235, "--scale", "0.5")
    assert result.returncode == 0, result.stderr
    g = load_pallet_geometry(out / "geometry.yaml")
    assert g.overall_depth_m == pytest.approx(0.55)
    assert g.opening_width_m == pytest.approx(0.1175)
    assert g.block_depth_m == pytest.approx(0.105)
    report = json.loads((out / "verification.json").read_text())
    assert report["fork_fit"]["fits"] is False
    assert report["fork_fit"]["ceiling_margin_m"] == pytest.approx(-0.002)
    assert report["insertion"]["target_m"] == pytest.approx(0.330)


def test_existing_schema_input_controls_chosen_top_boards(tmp_path):
    source = tmp_path / "custom.yaml"
    data = yaml.safe_load(GEOMETRY.read_text())
    data["top_board_count"] = 5
    data["top_board_width_m"] = 0.1
    source.write_text(yaml.safe_dump(data))
    result = run_drawing(tmp_path / "custom", 350, "--geometry", str(source))
    assert result.returncode == 0, result.stderr
    g = load_pallet_geometry(tmp_path / "custom/geometry.yaml")
    assert g.top_board_count == 5
    assert g.top_board_width_m == pytest.approx(0.1)
    assert str(source) in (tmp_path / "custom/dimensions.md").read_text()


def test_full_size_input_can_declare_its_source_scale(tmp_path):
    source = tmp_path / "full_size.yaml"
    data = yaml.safe_load(GEOMETRY.read_text())
    for key, value in data.items():
        if key.endswith("_m"):
            data[key] = (
                [v / 0.6 for v in value] if isinstance(value, list) else value / 0.6
            )
    source.write_text(yaml.safe_dump(data))
    result = run_drawing(
        tmp_path / "from_full_size",
        235,
        "--geometry",
        str(source),
        "--source-scale",
        "1",
    )
    assert result.returncode == 0, result.stderr
    g = load_pallet_geometry(tmp_path / "from_full_size/geometry.yaml")
    assert g.overall_depth_m == pytest.approx(0.660)
    assert g.block_depth_m == pytest.approx(0.126)
    assert g.top_board_width_m == pytest.approx(0.0825)


def test_refuses_overwrite_and_leaves_existing_files_byte_identical(tmp_path):
    out = tmp_path / "existing"
    out.mkdir()
    sentinel = out / "drawing.svg"
    sentinel.write_text("keep this")
    result = run_drawing(out, 235)
    assert result.returncode == 2
    assert "exist" in result.stderr.lower()
    assert sentinel.read_text() == "keep this"
    assert list(out.iterdir()) == [sentinel]


@pytest.mark.parametrize("scale", ["0", "-1", "nan", "inf"])
def test_invalid_scale_does_not_write_any_output(tmp_path, scale):
    out = tmp_path / "bad"
    result = run_drawing(out, 235, "--scale", scale)
    assert result.returncode == 2
    assert "scale" in result.stderr
    assert not out.exists()


def test_non_square_input_is_not_silently_promoted_to_t11(tmp_path):
    out = tmp_path / "epal"
    result = run_drawing(
        out, 235, "--geometry", str(ROOT / "config/pallet_geometry_epal6.yaml")
    )
    assert result.returncode == 2
    assert "square" in result.stderr
    assert not out.exists()
