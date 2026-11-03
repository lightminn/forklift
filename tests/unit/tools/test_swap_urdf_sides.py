"""tools/swap_urdf_sides.py: the steering left-asymmetry plan's A10 model swaps names, never geometry."""

import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("swap_urdf_sides", ROOT / "tools/swap_urdf_sides.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules["swap_urdf_sides"] = MODULE
SPEC.loader.exec_module(MODULE)
MEASURED = ROOT / "sim/models/dls08_measured/forklift.urdf"


def joints(text):
    root = ET.fromstring(text)
    return {j.get("name"): (j.find("parent").get("link"), j.find("child").get("link"),
                            j.find("origin").get("xyz") if j.find("origin") is not None else None)
            for j in root.iter("joint")}


def test_swap_name_exchanges_both_tokens():
    assert MODULE.swap_name("front_left_spin") == "front_right_spin"
    assert MODULE.swap_name("carriage_side_right_visual") == "carriage_side_left_visual"
    assert MODULE.swap_name("fork_lift") == "fork_lift"


def test_the_swapped_measured_model_keeps_every_origin_under_the_swapped_name():
    source = MEASURED.read_text()
    swapped = MODULE.swap_sides(source)
    before, after = joints(source), joints(swapped)
    assert len(before) == len(after)
    for name, (parent, child, xyz) in before.items():
        assert after[MODULE.swap_name(name)] == (MODULE.swap_name(parent), MODULE.swap_name(child), xyz)
    # the joint called left_steer now stands at y < 0 (it was right_steer)
    assert float(after["left_steer"][2].split()[1]) < 0 < float(after["right_steer"][2].split()[1])
    # nothing but names changed: swapping twice gives the source back
    assert MODULE.swap_sides(swapped) == source
