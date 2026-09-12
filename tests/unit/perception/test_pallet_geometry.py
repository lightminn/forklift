import math
from pathlib import Path

import pytest
import yaml

from forklift_core.perception.pallet_geometry import (
    check_fork_fit,
    load_pallet_geometry,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
EPAL6 = REPO_ROOT / "config" / "pallet_geometry_epal6.yaml"

# dls08_provisional, estimated from product images and not measured
PARAMETERS = yaml.safe_load(
    (REPO_ROOT / "sim/models/dls08_provisional/parameters.yaml").read_text()
)
DIMENSIONS = PARAMETERS["dimensions"]
FORKS = {
    "fork_spacing_m": DIMENSIONS["fork_spacing_m"],
    "fork_width_m": DIMENSIONS["fork_width_m"],
    "fork_thickness_m": DIMENSIONS["fork_thickness_m"],
    "fork_centre_height_m": DIMENSIONS["fork_center_height_m"],
    "fork_length_m": (
        DIMENSIONS["rear_extent_x_m"]
        + PARAMETERS["catalogue"]["overall_length_m"]
        - DIMENSIONS["fork_root_x_m"]
    ),
    "lift_travel_m": DIMENSIONS["lift_travel_m"],
}


def test_the_published_epal6_envelope_is_what_the_file_says():
    g = load_pallet_geometry(EPAL6)
    assert (g.overall_width_m, g.overall_depth_m, g.overall_height_m) == (
        0.800,
        0.600,
        0.144,
    )
    assert (g.block_width_m, g.block_depth_m, g.block_height_m) == (0.080, 0.073, 0.078)
    assert g.source_provenance == "epal6_published_standard"


def test_the_opening_geometry_is_derived_not_restated():
    g = load_pallet_geometry(EPAL6)
    assert g.opening_width_m == pytest.approx(0.280)
    assert g.opening_centre_offset_m == pytest.approx(0.180)
    assert g.opening_centre_spacing_m == pytest.approx(0.360)
    assert g.deck_top_m == pytest.approx(0.044)
    assert g.opening_z_band_m == pytest.approx((0.022, 0.100))
    assert g.opening_centre_height_m == pytest.approx(0.061)


def test_block_centres_are_symmetric_and_span_the_envelope():
    g = load_pallet_geometry(EPAL6)
    assert g.block_centres_y_m() == pytest.approx([-0.360, 0.0, 0.360])
    assert g.block_centres_x_m() == pytest.approx([-0.2635, 0.0, 0.2635])
    # the outer blocks end exactly at the envelope
    assert g.block_centres_y_m()[-1] + g.block_width_m / 2 == pytest.approx(
        g.overall_width_m / 2
    )


def test_the_estimated_dls08_forks_enter_this_pallet_without_lifting():
    fit = check_fork_fit(load_pallet_geometry(EPAL6), **FORKS)
    assert fit.fits
    assert fit.lateral_inner_margin_m == pytest.approx(0.0775)
    assert fit.lateral_outer_margin_m == pytest.approx(0.1475)
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


@pytest.mark.parametrize(
    "overrides",
    [
        {"overall_width_m": math.nan},
        {"block_width_m": math.inf},
        {"block_depth_m": 0.3},
        {"block_width_m": 0.3},
        {"deck_bottom_m": -0.01},
        {"deck_top_m": -0.01},
        {"block_count_across": 4},
        {"block_count_deep": 2.5},
        {"overall_depth_m": True},
    ],
)
def test_invalid_geometry_values_are_refused(tmp_path, overrides):
    data = {**yaml.safe_load(EPAL6.read_text()), **overrides}
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_pallet_geometry(bad)


def test_a_low_fork_requires_available_lift_and_floor_clearance():
    g = load_pallet_geometry(EPAL6)
    forks = {**FORKS, "fork_centre_height_m": 0.02}
    fit = check_fork_fit(g, **forks)
    assert fit.fits
    assert fit.lift_required_m == pytest.approx(0.018)
    assert not check_fork_fit(g, **{**forks, "lift_travel_m": 0.01}).fits
