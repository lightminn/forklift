"""EPAL 6 fork spacing plan, new plant candidate: the measured model's frame links (the fork-tip sites)
carry an explicit small inertial -- a URDF link without one becomes a PhysX body with the default 1 kg --
while the provisional model, without the optional key, keeps its output."""

import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def site_links(model: str) -> dict:
    root = ET.parse(ROOT / f"sim/models/{model}/forklift.urdf").getroot()
    return {l.get("name"): l for l in root.findall("link") if l.get("name", "").endswith("_fork_tip")}


def test_the_measured_fork_tip_frames_have_a_small_explicit_mass():
    links = site_links("dls08_measured")
    assert set(links) == {"left_fork_tip", "right_fork_tip"}
    for link in links.values():
        mass = float(link.find("inertial").find("mass").get("value"))
        assert 0 < mass <= 1e-3


def test_the_provisional_fork_tip_frames_are_unchanged():
    links = site_links("dls08_provisional")
    assert links and all(link.find("inertial") is None for link in links.values())


def test_the_optional_assumption_keys_are_accepted_only_as_extras(tmp_path):
    import pytest
    import yaml

    from tools import forklift_model_geometry as G

    data = yaml.safe_load((ROOT / "sim/models/dls08_measured/parameters.yaml").read_text())
    data["assumptions"]["not_a_key"] = 1.0
    path = tmp_path / "parameters.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="unexpected or missing assumptions"):
        G.load_parameters(path)
