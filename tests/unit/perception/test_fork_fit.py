# tests/unit/perception/test_fork_fit.py
from pathlib import Path

import pytest

from forklift_core.perception.fork_fit import (
    TEST_FORK_SPACING_M, ForkSetup, LoadDeclaration, danger_threshold_m, fork_fit, lane_reachable, load_site_defaults,
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


def _site(name):
    return load_site_defaults(ROOT / "config/site_load_defaults.yaml", {name: _known(name)})[name]


@pytest.mark.parametrize(("name", "margin"), [("epal6", 0.08625), ("t11_06", 0.0775)])
def test_known_shapes_at_their_test_spacing_with_the_site_default(name, margin):
    fit = fork_fit(_known(name), "x", ForkSetup(TEST_FORK_SPACING_M[name]), _site(name))
    assert fit.pickable and fit.reasons == () and fit.load_source == "site_default"
    assert fit.min_lateral_margin_m == pytest.approx(margin, abs=1e-9)


def test_known_shape_without_any_load_needs_confirmation():
    assert fork_fit(_known("epal6"), "x", ForkSetup(0.3725), None).reasons == ("support_unverified",)


@pytest.mark.parametrize("mass", [float("nan"), -1.0, 0.0])
def test_load_declarations_must_be_finite_and_positive(mass):
    with pytest.raises(ValueError):
        LoadDeclaration(mass, 0.0, 0.3, 0.0, "site_default")


def test_t11_at_the_epal_spacing_needs_adjustment():
    fit = fork_fit(_known("t11_06"), "x", ForkSetup(0.3725), _site("t11_06"))
    assert fit.min_lateral_margin_m == pytest.approx(0.03625, abs=1e-9)
    assert fit.reasons == ("fork_spacing_adjust",)


def test_known_shape_with_a_declared_heavy_load_hits_the_payload_limit():
    heavy = LoadDeclaration(9.0, 3.0, com_x_max_m=0.3, com_y_max_m=0.0)
    assert fork_fit(_known("epal6"), "x", ForkSetup(0.3725), heavy).reasons == ("payload_limit",)


def test_danger_threshold_is_margin_minus_tracking_allowance():
    fit = fork_fit(_known("t11_06"), "x", ForkSetup(0.290), _site("t11_06"))
    assert danger_threshold_m(fit) == pytest.approx(0.0437)


def test_a_valid_pair_is_found_when_the_widest_margin_pair_fails_support():
    # Codex counterexample: (0,1) has the widest margin (91.25 mm) but its fork
    # centre sits 215 mm off-centre; (1,2) fits with 86.25 mm and supports.
    spec = _ns(overall_width_m=1.2, column_centres_y_m=(-0.565, -0.21, 0.1575, 0.5275),
               column_widths_m=(0.07, 0.12, 0.135, 0.145))
    fit = fork_fit(spec, "x", ForkSetup(0.3725), CENTRED)
    assert fit.pickable and fit.pair == (1, 2)


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
    fit = fork_fit(spec, "x", ForkSetup(0.3725), _site("epal6"), support_z_m=0.288)
    assert "lift_alignment_needed" in fit.reasons
    # Lane 0.288-0.388 m; blades lifted 0.28 m sit at 0.3075-0.3325 m.
    assert lane_reachable(spec, "x", ForkSetup(0.3725), support_z_m=0.288)
    assert not lane_reachable(spec, "x", ForkSetup(0.3725), support_z_m=0.432)


def test_a_face_whose_boards_cross_the_insertion_axis_cannot_be_near_tracked():
    # T11 y-face: the top boards run along x, across the forks' path (design section 2).
    fit = fork_fit(_known("t11_06"), "y", ForkSetup(0.290), _site("t11_06"))
    assert "near_tracking_unavailable" in fit.reasons


def test_boards_without_gaps_cannot_be_near_tracked():
    seamless = _ns(top_board_count=2, top_board_width_m=0.41)  # 2 x 0.41 covers 0.82 m
    assert "near_tracking_unavailable" in fork_fit(seamless, "x", ForkSetup(0.3725), CENTRED).reasons


def test_a_site_default_is_not_accepted_for_a_nonstandard_pallet():
    assumed = LoadDeclaration(2.0, 6.0, 0.275, 0.0, "site_default")
    assert "support_unverified" in fork_fit(_ns(), "x", ForkSetup(0.3725), assumed).reasons


@pytest.mark.parametrize("kwargs", [{"spacing_m": float("nan")}, {"spacing_m": 0.3725, "blade_width_m": float("nan")},
                                    {"spacing_m": 0.3725, "lowest_centre_m": -0.01}])
def test_fork_setup_must_be_finite_and_positive(kwargs):
    with pytest.raises(ValueError):
        ForkSetup(**kwargs)


@pytest.mark.parametrize(("support", "m_req"), [(float("nan"), 0.066), (0.0, float("nan")), (-0.1, 0.066)])
def test_fit_rejects_nonfinite_or_negative_heights_and_thresholds(support, m_req):
    with pytest.raises(ValueError):
        fork_fit(_known("epal6"), "x", ForkSetup(0.3725), _site("epal6"), support_z_m=support, m_req_m=m_req)
