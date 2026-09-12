import dataclasses
from pathlib import Path

import pytest
import yaml

from forklift_core.perception.pallet_prior import PalletPrior, load_pallet_prior

REPO_PRIOR = Path(__file__).resolve().parents[3] / "config" / "pallet_prior_v1.yaml"
GOOD = {
    "height_m": 0.30,
    "deck_m": 0.05,
    "opening_height_m": 0.20,
    "opening_width_range": [0.18, 0.30],
    "centre_spacer_range": [0.08, 0.12],
    "overall_width_m": 0.8,
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
    assert (prior.height_m, prior.deck_m, prior.opening_height_m) == (0.30, 0.05, 0.20)
    assert prior.opening_centre_height_m == pytest.approx(0.15)
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
        {"deck_m": -0.05},
        {"opening_height_m": 0.0},
        {"opening_width_range": [0.30, 0.18]},
        {"opening_width_range": [0.18]},
        {"centre_spacer_range": [0.08, float("inf")]},
        {"overall_width_m": 0.5},
        {"source_provenance": "guess"},
    ],
)
def test_malformed_prior_files_are_rejected(tmp_path, overrides):
    with pytest.raises(ValueError):
        load_pallet_prior(write(tmp_path, **overrides))
