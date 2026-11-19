"""The closed-shelf test article for the Isaac S- check (docs/plans/2026-11-18-bottom-deck-structure-default.md 3-3):
the N2 catalogue's shelf_closed -- full-depth columns at the pallet's block rows up to a solid deck, no gap -- as a
single-link URDF the runner's named-box check accepts as --pallet-article shelf_closed."""

import sys
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from tools import build_shelf_article as S
from tools.build_shelf_article import ROOT

sys.path.insert(0, str(ROOT / "sim/isaac"))

import insertion_geometry as IG  # noqa: E402

from tools import negative_structures as N  # noqa: E402

SOURCE = ROOT / "sim/models/epal6_pallet/pallet.urdf"
GEOMETRY = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")


def test_the_boxes_are_the_n2_catalogues_shelf_closed():
    ours = sorted((tuple(np.round(b.centre_m, 9)), tuple(np.round(b.size_m, 9))) for b in IG.shelf_closed_boxes(GEOMETRY))
    theirs = sorted((tuple(np.round(b.centre_m, 9)), tuple(np.round(b.size_m, 9)))
                    for b in N.build(GEOMETRY, "shelf_closed", np.random.default_rng(0)))
    assert ours == theirs
    names = [b.name for b in IG.shelf_closed_boxes(GEOMETRY)]
    assert len(set(names)) == len(names) and not any(n.startswith(IG.PALLET_SUPPORT_PREFIXES) for n in names)


def test_the_written_article_passes_its_own_check_and_keeps_the_envelope(tmp_path):
    out = tmp_path / "pallet.urdf"
    S.main(["--source", str(SOURCE), "--geometry", str(ROOT / "config/pallet_geometry_epal6.yaml"),
            "--output", str(out)])
    IG.assert_pallet_urdf_matches_named_boxes(out, GEOMETRY, article="shelf_closed")
    IG.assert_pallet_urdf_matches_geometry(out, GEOMETRY.overall_depth_m, GEOMETRY.overall_width_m)
    with pytest.raises(ValueError):
        IG.assert_pallet_urdf_matches_named_boxes(out, GEOMETRY)  # not the nominal pallet
    with pytest.raises(ValueError):
        IG.assert_pallet_urdf_matches_named_boxes(SOURCE, GEOMETRY, article="shelf_closed")
    src, new = ET.parse(SOURCE).getroot(), ET.parse(out).getroot()
    assert new.get("name") == src.get("name")  # the run's semantic labels are the pallet's
    assert [link.get("name") for link in new.findall("link")] == [link.get("name") for link in src.findall("link")]
    src_inertial, new_inertial = src.find("link/inertial"), new.find("link/inertial")  # kept as it is (here: none)
    assert (src_inertial is None and new_inertial is None) or ET.tostring(new_inertial) == ET.tostring(src_inertial)


def test_the_committed_article_is_the_builders_output(tmp_path):
    out = tmp_path / "pallet.urdf"
    S.main(["--output", str(out)])
    assert out.read_bytes() == (ROOT / "sim/models/epal6_shelf_closed/pallet.urdf").read_bytes()


def test_the_runner_takes_the_article():
    text = (ROOT / "sim/isaac/run_transport.py").read_text()
    start = text.index('"--pallet-article"')
    assert '"shelf_closed"' in text[start:start + 200]  # among the argument's choices
