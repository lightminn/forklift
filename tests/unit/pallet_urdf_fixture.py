"""Synthetic envelope and named T11 assembly fixtures; no contact claims."""

from pathlib import Path


def write_pallet_urdf(
    path: Path,
    *,
    x_m: float = 0,
    z_m: float = 0.045,
    depth_m: float = 0.66,
    width_m: float = 0.66,
) -> Path:
    path.write_text(f'''<robot name="synthetic"><link name="pallet">
<collision name="envelope"><origin xyz="{x_m} 0 {z_m}"/>
<geometry><box size="{depth_m} {width_m} 0.09"/></geometry>
</collision></link></robot>''')
    return path


def write_full_t11_pallet_urdf(
    path: Path, *, swap_all=False, swap_boards_only=False
) -> Path:
    """Write all 22 named T11 boxes, optionally swapping all boxes or only boards.

    Preserve names and zero rpy; rotate centres (x, y) to (-y, x) and swap
    full x/y sizes. Leaving blocks untouched isolates the board-layout trap.
    """
    from xml.etree import ElementTree as ET

    from forklift_core.perception.pallet_geometry import (
        load_pallet_geometry,
        pallet_boxes,
    )

    root = Path(__file__).resolve().parents[2]
    geometry = load_pallet_geometry(root / "config/pallet_geometry_t11_06.yaml")
    robot = ET.Element("robot", name="synthetic_t11")
    link = ET.SubElement(robot, "link", name="pallet")
    for box in pallet_boxes(geometry):
        x, y, z = box.centre_m
        sx, sy, sz = box.size_m
        if swap_all or (
            swap_boards_only
            and box.name.startswith(("bottom_board_", "stringer_", "top_board_"))
        ):
            x, y = -y, x
            sx, sy = sy, sx
        collision = ET.SubElement(link, "collision", name=box.name)
        ET.SubElement(collision, "origin", xyz=f"{x} {y} {z}", rpy="0 0 0")
        shape = ET.SubElement(collision, "geometry")
        ET.SubElement(shape, "box", size=f"{sx} {sy} {sz}")
    ET.ElementTree(robot).write(path, encoding="unicode")
    return path
