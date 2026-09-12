"""Generate static MJCF/URDF pallet boxes from authoritative geometry YAML.

The origin is the footprint centre at floor level. Continuous bottom and
combined top decks omit individual board gaps; no payload dynamics are claimed.
"""

import argparse
import hashlib
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET

from forklift_core.perception.pallet_geometry import (
    PalletGeometry,
    load_pallet_geometry,
)

SIMPLIFICATIONS = [
    "deck_board_gaps_omitted",
    "continuous_bottom_and_combined_top_decks",
]


@dataclass(frozen=True)
class PalletBox:
    """Full box size and centre in metres in the pallet's floor-centred frame."""

    name: str
    size_m: tuple[float, float, float]
    centre_m: tuple[float, float, float]


def pallet_boxes(geometry: PalletGeometry) -> list[PalletBox]:
    """Use one assembly for both exporters and geometric clearance consumers."""
    g = geometry
    boxes = [
        PalletBox(
            "deck_bottom",
            (g.overall_depth_m, g.overall_width_m, g.deck_bottom_m),
            (0.0, 0.0, g.deck_bottom_m / 2),
        )
    ]
    for xi, x in enumerate(g.block_centres_x_m()):
        for yi, y in enumerate(g.block_centres_y_m()):
            boxes.append(
                PalletBox(
                    f"block_x{xi}_y{yi}",
                    (g.block_depth_m, g.block_width_m, g.block_height_m),
                    (x, y, g.opening_centre_height_m),
                )
            )
    boxes.append(
        PalletBox(
            "deck_top",
            (g.overall_depth_m, g.overall_width_m, g.deck_top_m),
            (0.0, 0.0, g.overall_height_m - g.deck_top_m / 2),
        )
    )
    return boxes


def numbers(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:.12g}" for value in values)


def write_xml(root: ET.Element, path: Path) -> None:
    ET.indent(root, space="  ")
    path.write_text(
        '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n",
        encoding="utf-8",
    )


def build_models(geometry_path: Path, output: Path) -> None:
    """Write deterministic model geometry and a timestamped source manifest."""
    g = load_pallet_geometry(geometry_path)
    boxes = pallet_boxes(g)
    mjcf = ET.Element("mujoco", model=f"{g.geometry_version}_pallet")
    custom = ET.SubElement(mjcf, "custom")
    ET.SubElement(custom, "text", name="pallet_provenance", data=g.source_provenance)
    world = ET.SubElement(mjcf, "worldbody")
    body = ET.SubElement(world, "body", name="pallet", pos="0 0 0")
    urdf = ET.Element("robot", name=f"{g.geometry_version}_pallet")
    link = ET.SubElement(urdf, "link", name="pallet")
    for box in boxes:
        ET.SubElement(
            body,
            "geom",
            name=box.name,
            type="box",
            size=numbers(tuple(value / 2 for value in box.size_m)),
            pos=numbers(box.centre_m),
            rgba="0.62 0.40 0.20 1",
        )
        for kind in ("visual", "collision"):
            item = ET.SubElement(link, kind, name=box.name)
            ET.SubElement(item, "origin", xyz=numbers(box.centre_m), rpy="0 0 0")
            shape = ET.SubElement(item, "geometry")
            ET.SubElement(shape, "box", size=numbers(box.size_m))
            if kind == "visual":
                material = ET.SubElement(item, "material", name=f"wood_{box.name}")
                ET.SubElement(material, "color", rgba="0.62 0.40 0.20 1")
    output.mkdir(parents=True, exist_ok=True)
    write_xml(mjcf, output / "pallet.xml")
    write_xml(urdf, output / "pallet.urdf")
    manifest = {
        "geometry_file": geometry_path.name,
        "geometry_sha256": hashlib.sha256(geometry_path.read_bytes()).hexdigest(),
        "geometry_version": g.geometry_version,
        "source_provenance": g.source_provenance,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "origin": "footprint_centre_at_floor_z0",
        "simplifications": SIMPLIFICATIONS,
        "physical_dynamics_validated": False,
        "model_sha256": {
            name: hashlib.sha256((output / name).read_bytes()).hexdigest()
            for name in ("pallet.xml", "pallet.urdf")
        },
    }
    (output / "model_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument(
        "--output", type=Path, required=True, help="Model output directory"
    )
    args = parser.parse_args(argv)
    build_models(args.geometry, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
