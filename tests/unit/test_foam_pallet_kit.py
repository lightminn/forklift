# tests/unit/test_foam_pallet_kit.py
import csv
import json

import pytest

from tools import foam_pallet_kit as kit
from tools import recognition_scenes as rs


@pytest.mark.parametrize("target_mm, sheets, expected", [
    (22, (10, 20, 30), (20,)),          # nearest single sheet
    (22, (10,), (10, 10)),              # two layers when one is too thin
    (15, (10, 5), (10, 5)),             # exact stack
    (15, (10, 20), (20,)),              # tie at 5 mm off: the thicker, stronger choice
    (78, (30, 20), (30, 30, 20)),       # up to three layers
])
def test_the_closest_stack_is_chosen(target_mm, sheets, expected):
    assert kit.choose_stack(target_mm, sheets) == expected


def test_the_as_built_spec_keeps_the_pockets_and_changes_the_boards():
    nominal = rs.known_spec("epal6")
    build = kit.as_built(nominal, (20,))
    built, choice = build.spec, build.choice
    assert built.opening_height_m == nominal.opening_height_m  # printed blocks keep the pocket height
    assert (built.deck_bottom_m, built.stringer_m, built.top_thickness_m) == pytest.approx((0.020, 0.020, 0.020))
    assert built.overall_height_m == pytest.approx(nominal.overall_height_m - 0.006)
    assert choice["top"]["stack_mm"] == [20] and choice["top"]["nominal_mm"] == pytest.approx(22.0)


def test_the_cut_list_counts_every_board_and_layer(tmp_path):
    nominal = rs.known_spec("epal6")
    rows = kit.cut_list(kit.as_built(nominal, (10,)))
    by_group = {}
    for r in rows:
        by_group[r["group"]] = by_group.get(r["group"], 0) + r["quantity"]
    assert by_group == {"bottom_board": 3 * 2, "stringer": 3 * 2, "top_board": 7 * 2}  # 22 mm -> 10 + 10
    stringer = next(r for r in rows if r["group"] == "stringer")
    assert (stringer["length_mm"], stringer["width_mm"], stringer["sheet_mm"]) == (800.0, 145.0, 10)


def test_sheets_hold_every_piece():
    rows = kit.cut_list(kit.as_built(rs.known_spec("epal6"), (20,)))
    usage = kit.sheet_usage(rows, sheet_mm=(900, 1800))
    assert usage[20]["pieces"] == 3 + 3 + 7 and usage[20]["sheets"] >= 1
    placed = [p for s in usage[20]["layout"] for p in s]
    assert all(p["x"] + p["l"] <= 1800 + 1e-9 and p["y"] + p["w"] <= 900 + 1e-9 for p in placed)


def test_a_piece_larger_than_the_sheet_is_refused():
    rows = kit.cut_list(kit.as_built(rs.known_spec("epal6"), (20,)))
    with pytest.raises(ValueError, match="larger than the sheet"):
        kit.sheet_usage(rows, sheet_mm=(600, 700))


def test_the_cli_writes_a_kit_and_a_comparison(tmp_path):
    kit.main(["--shape", "t11_06", "--sheets", "10,20", "--out", str(tmp_path / "t11")])
    out = tmp_path / "t11"
    data = json.loads((out / "kit.json").read_text())
    assert data["as_built"]["opening_height_m"] == pytest.approx(rs.known_spec("t11_06").opening_height_m)
    assert sorted(p.name for p in (out / "blocks").glob("*.stl")) and len(list((out / "blocks").glob("*.stl"))) == 9
    with open(out / "cut_list.csv", newline="", encoding="utf-8") as f:
        assert len(list(csv.DictReader(f))) >= 3
    kit.main(["--shape", "epal6", "--compare", "10;20;30;10,20;10,20,30", "--out", str(tmp_path / "cmp")])
    table = json.loads((tmp_path / "cmp" / "compare.json").read_text())
    assert [row["sheets_mm"] for row in table] == [[10], [20], [30], [10, 20], [10, 20, 30]]
    assert all(row["pocket_height_mm"] == pytest.approx(78.0) for row in table)


def test_the_kit_draws_each_sheet_layout(tmp_path):
    kit.main(["--shape", "epal6", "--sheets", "10", "--out", str(tmp_path / "k")])
    svgs = sorted((tmp_path / "k").glob("sheet_layout_10mm_*.svg"))
    pieces = sum(svg.read_text().count('class="piece"') for svg in svgs)
    assert len(svgs) == json.loads((tmp_path / "k" / "kit.json").read_text())["sheet_usage"]["10"]["sheets"]
    assert pieces == 26  # 13 boards, two 10 mm layers each
