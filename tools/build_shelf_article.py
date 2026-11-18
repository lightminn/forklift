"""Write the closed-shelf test article URDF (docs/plans/2026-11-18-bottom-deck-structure-default.md 3-3).

The Isaac S- check needs a shelf where the mission expects a pallet: the N2 catalogue's ``shelf_closed`` -- columns at
the pallet's block rows running the full depth up to a solid deck (insertion_geometry.shelf_closed_boxes). The source
pallet URDF's robot and link names and its inertial are kept (the run's semantic labels and the rigid body stay the
pallet's); every visual and collision is replaced by the shelf's boxes. The runner checks the result box by box under
``--pallet-article shelf_closed``.
"""
from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "src", ROOT / "sim/isaac"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from insertion_geometry import shelf_closed_boxes  # noqa: E402

from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402

GREY = "0.55 0.57 0.60 1"


def _fmt(values) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def shelf(robot: ET.Element, geometry) -> ET.Element:
    out = copy.deepcopy(robot)
    links = out.findall("link")
    if len(links) != 1:
        raise ValueError("expected a single-link pallet URDF")
    link = links[0]
    for el in list(link):
        if el.tag in ("visual", "collision"):
            link.remove(el)
    for box in shelf_closed_boxes(geometry):
        for tag in ("visual", "collision"):
            el = ET.SubElement(link, tag, name=box.name)
            ET.SubElement(el, "origin", xyz=_fmt(box.centre_m), rpy="0 0 0")
            ET.SubElement(ET.SubElement(el, "geometry"), "box", size=_fmt(box.size_m))
            if tag == "visual":
                ET.SubElement(ET.SubElement(el, "material", name=f"grey_{box.name}"), "color", rgba=GREY)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--source", type=Path, default=ROOT / "sim/models/epal6_pallet/pallet.urdf")
    ap.add_argument("--geometry", type=Path, default=ROOT / "config/pallet_geometry_epal6.yaml")
    ap.add_argument("--output", type=Path, default=ROOT / "sim/models/epal6_shelf_closed/pallet.urdf")
    a = ap.parse_args(argv)
    tree = ET.ElementTree(shelf(ET.parse(a.source).getroot(), load_pallet_geometry(a.geometry)))
    ET.indent(tree, space="  ")
    a.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(a.output, encoding="unicode", xml_declaration=True)
    with a.output.open("a") as fh:
        fh.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
