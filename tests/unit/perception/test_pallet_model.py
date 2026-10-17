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


def test_nonfinite_member_centres_are_rejected():
    with pytest.raises(ValueError):
        _spec(column_centres_y_m=(-0.57, float("nan"), 0.055, 0.57))
