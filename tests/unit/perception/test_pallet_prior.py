import dataclasses
from pathlib import Path

import pytest
import yaml

from forklift_core.perception.pallet_prior import PalletPrior, load_pallet_prior

REPO_ROOT = Path(__file__).resolve().parents[3]
REPO_PRIOR = REPO_ROOT / "config" / "pallet_prior_v1.yaml"
GOOD = {
    "height_m": 0.30,
    "deck_bottom_m": 0.05,
    "deck_top_m": 0.05,
    "opening_height_m": 0.20,
    "opening_width_range": [0.18, 0.30],
    "centre_spacer_range": [0.08, 0.12],
    "overall_width_m": 0.8,
    "overall_depth_m": 0.6,
    "source_provenance": "synthetic",
    "catalogue_version": "v1",
}


def write(tmp_path, **overrides):
    data = {**GOOD, **overrides}
    for key in [k for k, v in overrides.items() if v is None]:
        del data[key]
    path = tmp_path / "prior.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


def test_repository_prior_loads_and_reports_the_opening_centre_height():
    prior = load_pallet_prior(REPO_PRIOR)
    assert (
        prior.height_m,
        prior.deck_bottom_m,
        prior.deck_top_m,
        prior.opening_height_m,
    ) == (0.30, 0.05, 0.05, 0.20)
    assert prior.opening_centre_height_m == pytest.approx(0.15)
    assert prior.overall_depth_m == 0.6
    assert prior.source_provenance == "synthetic" and prior.catalogue_version == "v1"


def test_the_prior_dataclass_has_no_defaults_so_synthetic_sizes_cannot_leak_in():
    for field in dataclasses.fields(PalletPrior):
        assert field.default is dataclasses.MISSING
        assert field.default_factory is dataclasses.MISSING


@pytest.mark.parametrize(
    "overrides",
    [
        {"extra_key": 1},
        {"height_m": None},
        {"height_m": 0.31},
        {"deck_bottom_m": -0.05},
        {"opening_height_m": 0.0},
        {"opening_width_range": [0.30, 0.18]},
        {"opening_width_range": [0.18]},
        {"centre_spacer_range": [0.08, float("inf")]},
        {"overall_width_m": 0.5},
        {"overall_depth_m": 0.0},
        {"overall_depth_m": -0.6},
        {"overall_depth_m": float("nan")},
        {"source_provenance": "guess"},
    ],
)
def test_malformed_prior_files_are_rejected(tmp_path, overrides):
    with pytest.raises(ValueError):
        load_pallet_prior(write(tmp_path, **overrides))


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
    assert yaml.safe_load(out.read_text())["overall_depth_m"] == 0.6


def test_a_prior_without_overall_depth_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        load_pallet_prior(write(tmp_path, overall_depth_m=None))


def test_the_epal6_prior_carries_the_asymmetric_decks():
    prior = load_pallet_prior(REPO_ROOT / "config" / "pallet_prior_epal6.yaml")
    assert prior.deck_bottom_m == pytest.approx(0.022)
    assert prior.deck_top_m == pytest.approx(0.044)
    assert prior.opening_height_m == pytest.approx(0.078)
    assert prior.opening_centre_height_m == pytest.approx(0.061)
    assert (prior.opening_width_min_m, prior.opening_width_max_m) == pytest.approx(
        [0.2075, 0.2475]
    )
    assert (prior.centre_spacer_min_m, prior.centre_spacer_max_m) == pytest.approx(
        [0.130, 0.160]
    )
    assert prior.source_provenance == "epal6_published_standard_plus_cad_measurement"


def test_generated_cad_measured_prior_can_be_loaded(tmp_path):
    from tools import build_pallet_prior

    out = tmp_path / "prior.yaml"
    build_pallet_prior.main(
        [
            "--geometry",
            str(REPO_ROOT / "config/pallet_geometry_epal6.yaml"),
            "--output",
            str(out),
        ]
    )
    prior = load_pallet_prior(out)
    assert prior.source_provenance == "epal6_published_standard_plus_cad_measurement"
    assert prior.deck_top_m == pytest.approx(0.044)


def test_a_prior_whose_decks_and_opening_do_not_reach_the_height_is_refused(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(
        "source_provenance: synthetic\ncatalogue_version: test\n"
        "height_m: 0.144\ndeck_bottom_m: 0.022\ndeck_top_m: 0.044\n"
        "opening_height_m: 0.100\nopening_width_range: [0.16, 0.20]\n"
        "centre_spacer_range: [0.13, 0.16]\noverall_width_m: 0.8\n"
        "overall_depth_m: 0.6\n"
    )
    with pytest.raises(ValueError):
        load_pallet_prior(bad)


def test_generated_prior_tolerances_can_be_selected_and_are_recorded(capsys):
    from tools import build_pallet_prior

    assert (
        build_pallet_prior.main(
            [
                "--geometry",
                str(REPO_ROOT / "config/pallet_geometry_epal6.yaml"),
                "--opening-width-tolerance",
                "0.01",
                "--centre-spacer-tolerance",
                "0.005",
            ]
        )
        == 0
    )
    data = yaml.safe_load(capsys.readouterr().out)
    assert data["opening_width_range"] == pytest.approx([0.2175, 0.2375])
    assert data["centre_spacer_range"] == pytest.approx([0.140, 0.150])
    assert data["opening_width_tolerance_m"] == 0.01
    assert data["centre_spacer_tolerance_m"] == 0.005
    assert data["source_provenance"] == "epal6_published_standard_plus_cad_measurement"
    assert data["overall_depth_m"] == 0.6


def test_the_geometry_can_produce_a_prior_without_a_file_round_trip():
    from forklift_core.perception.pallet_geometry import load_pallet_geometry

    g = load_pallet_geometry(REPO_ROOT / "config/pallet_geometry_epal6.yaml")
    prior = g.to_pallet_prior()
    assert prior.opening_width_min_m == pytest.approx(0.2075)
    assert prior.opening_width_max_m == pytest.approx(0.2475)
    assert prior.centre_spacer_min_m == pytest.approx(0.130)
    assert prior.centre_spacer_max_m == pytest.approx(0.160)
    assert prior.overall_depth_m == g.overall_depth_m == 0.6


@pytest.mark.parametrize("tolerance", [-0.01, 0.0, 0.2, float("nan")])
def test_invalid_generation_tolerances_are_refused(tolerance):
    from forklift_core.perception.pallet_geometry import load_pallet_geometry

    g = load_pallet_geometry(REPO_ROOT / "config/pallet_geometry_epal6.yaml")
    with pytest.raises(ValueError):
        g.to_pallet_prior(opening_width_tolerance_m=tolerance)
