"""Write the grounded lookalike of a pallet URDF (plan 2026-10-19 task 5).

"grounded": the pallet without its bottom boards, every block column run down to the floor -- from the entry face
the same pallet, and the structure the bottom-deck side evidence must not read as one. The bottom boards' visual
and collision elements are dropped and every ``block*`` box is extended to z = 0 keeping its top; stringers and top
boards are unchanged. The robot name is kept so the run's semantic labels are the pallet's.
"""
from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _fmt(values) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def grounded(robot: ET.Element) -> ET.Element:
    out = copy.deepcopy(robot)
    for link in out.iter("link"):
        for el in list(link):
            name = el.get("name") or ""
            if el.tag in ("visual", "collision") and name.startswith("bottom_board"):
                link.remove(el)
            elif el.tag in ("visual", "collision") and name.startswith("block"):
                origin, box = el.find("origin"), el.find("geometry/box")
                x, y, z = (float(v) for v in origin.get("xyz").split())
                sx, sy, sz = (float(v) for v in box.get("size").split())
                top = z + sz / 2
                origin.set("xyz", _fmt((x, y, top / 2)))
                box.set("size", _fmt((sx, sy, top)))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--source", type=Path, default=ROOT / "sim/models/epal6_pallet/pallet.urdf")
    ap.add_argument("--output", type=Path, default=ROOT / "sim/models/epal6_grounded/pallet.urdf")
    a = ap.parse_args(argv)
    tree = ET.ElementTree(grounded(ET.parse(a.source).getroot()))
    ET.indent(tree, space="  ")
    a.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(a.output, encoding="unicode", xml_declaration=True)
    with a.output.open("a") as fh:
        fh.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
