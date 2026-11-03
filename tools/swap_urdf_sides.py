#!/usr/bin/env python3
"""A diagnostic URDF with every left/right name swapped (steering left-asymmetry plan, docs/plans/
2026-11-03-steering-left-asymmetry.md, arm A10). Only names change -- every ``name`` attribute and every ``link``
reference (parent/child) -- the geometry stays where it is, so the link that stands on the truck's left is named
``right_*``. The Isaac URDF importer orders a link's children by joint name (a name-keyed map), so a symmetric model
swapped this way is the same truck with its left/right traversal order reversed.

  python tools/swap_urdf_sides.py sim/models/dls08_measured/forklift.urdf OUT.urdf
"""

import argparse
import hashlib
import re
from pathlib import Path

_ATTR = re.compile(r'\b(name|link)="([^"]*)"')
_TOKEN = re.compile(r"left|right")


def swap_name(name: str) -> str:
    return _TOKEN.sub(lambda m: "right" if m.group(0) == "left" else "left", name)


def swap_sides(text: str) -> str:
    return _ATTR.sub(lambda m: f'{m.group(1)}="{swap_name(m.group(2))}"', text)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("urdf", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    source = args.urdf.read_text()
    out = swap_sides(source)
    args.output.write_text(out)
    print("SWAPPED", args.urdf, hashlib.sha256(source.encode()).hexdigest()[:8], "->", args.output,
          hashlib.sha256(out.encode()).hexdigest()[:8])


if __name__ == "__main__":
    main()
