"""The simulated pallet's mass by shape (docs/plans/2026-11-01-pallet-mass.md, sim/isaac/pallet_mass.py)."""
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from pallet_mass import resolve_pallet_mass  # noqa: E402

from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402

SHAPES = {"epal6": 4.93, "t11_06": 2.49, "t11_06_d600": 2.49}


def _settings(name):
    return yaml.safe_load((ROOT / "config" / name).read_text())


@pytest.mark.parametrize("name", ["isaac_transport_measured.yaml", "isaac_transport_measured_tyre.yaml"])
def test_the_measured_settings_give_each_shape_its_printed_article_estimate(name):
    settings = _settings(name)
    for shape, kg in SHAPES.items():
        # the shape is the geometry's version, as the runner reads it
        version = load_pallet_geometry(ROOT / f"config/pallet_geometry_{shape}.yaml").geometry_version
        resolved, record = resolve_pallet_mass(settings, version)
        assert resolved["pallet_mass_kg"] == kg and record == {
            "kg": kg, "shape": shape, "source": "settings_by_shape", "settings_value": settings["pallet_mass_kg"]}
        assert settings["pallet_mass_kg"] == SHAPES  # the file's table is not changed in place


def test_the_site_defaults_equal_the_measured_settings_table():
    site = yaml.safe_load((ROOT / "config/site_load_defaults.yaml").read_text())["shapes"]
    assert {shape: entry["pallet_mass_kg"] for shape, entry in site.items()} == SHAPES
    assert _settings("isaac_transport_measured.yaml")["pallet_mass_kg"] == SHAPES


@pytest.mark.parametrize("name", ["isaac_transport.yaml", "isaac_transport_fast.yaml"])
def test_a_number_is_kept_as_read_for_every_shape(name):
    # the provisional settings keep 10 kg for their old baselines, recorded exactly as before
    settings = _settings(name)
    for shape in (*SHAPES, None):
        resolved, record = resolve_pallet_mass(settings, shape)
        assert resolved is settings and record["kg"] == 10.0 and record["source"] == "settings_number"


def test_a_table_needs_the_shape_and_an_entry_and_values_must_be_positive():
    settings = {"physics_hz": 120, "pallet_mass_kg": {"epal6": 4.93}}
    with pytest.raises(ValueError, match="loads no pallet geometry"):
        resolve_pallet_mass(settings, None)
    with pytest.raises(ValueError, match="no entry for the pallet shape 't11_06'"):
        resolve_pallet_mass(settings, "t11_06")
    for bad in (0.0, -1.0, float("nan"), float("inf"), True, "4.93", None):
        with pytest.raises(ValueError, match="positive finite"):
            resolve_pallet_mass({"pallet_mass_kg": {"epal6": bad}}, "epal6")
        with pytest.raises(ValueError, match="positive finite"):
            resolve_pallet_mass({"pallet_mass_kg": bad}, "epal6")
