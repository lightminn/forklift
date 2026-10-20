"""The grounded lookalike for the Isaac check (plan 2026-10-19 task 5): the EPAL 6 pallet without its bottom
boards, every block column down to the floor -- the entry face unchanged."""

from xml.etree import ElementTree as ET

import pytest

from tools import build_grounded_pallet as G
from tools.build_grounded_pallet import ROOT

SOURCE = ROOT / "sim/models/epal6_pallet/pallet.urdf"


def boxes(root):
    out = {}
    for el in root.iter():
        if el.tag in ("visual", "collision"):
            o = [float(v) for v in el.find("origin").get("xyz").split()]
            s = [float(v) for v in el.find("geometry/box").get("size").split()]
            out[(el.tag, el.get("name"))] = (o, s)
    return out


def test_the_bottom_boards_go_and_the_blocks_reach_the_floor():
    src, out = boxes(ET.parse(SOURCE).getroot()), boxes(G.grounded(ET.parse(SOURCE).getroot()))
    assert not any(n.startswith("bottom_board") for _, n in out)
    assert any(n.startswith("bottom_board") for _, n in src)
    for key, (o, s) in src.items():
        if key[1].startswith("bottom_board"):
            continue
        o2, s2 = out[key]
        if key[1].startswith("block"):
            assert o2[2] - s2[2] / 2 == pytest.approx(0.0, abs=1e-9)          # down to the floor
            assert o2[2] + s2[2] / 2 == pytest.approx(o[2] + s[2] / 2)       # the same top
            assert (o2[:2], s2[:2]) == (o[:2], s[:2])
        else:
            assert (o2, s2) == (o, s)  # stringers and top boards unchanged


def test_the_written_model_is_reproducible(tmp_path):
    a, b = tmp_path / "a.urdf", tmp_path / "b.urdf"
    G.main(["--source", str(SOURCE), "--output", str(a)])
    G.main(["--source", str(SOURCE), "--output", str(b)])
    assert a.read_bytes() == b.read_bytes() and b"bottom_board" not in a.read_bytes()
