"""Move a scene catalogue onto a different pallet geometry, poses unchanged.

The v1 catalogue's 100 scenes carry poses, categories, splits, lighting,
surfaces and distractors that took a seeded draw to produce and that the
evaluation record is written against. Re-drawing them for a second pallet
would change the population as well as the shape, and nothing downstream could
tell the two effects apart. This copies every pose exactly and recomputes only
what the shape decides.

Recomputed, and nothing else:

* the header's ``catalogue_version``, ``generator`` and ``pallet`` block;
* ``pallet.opening_width_m``, which becomes the target's single value -- v1
  drew 78 distinct widths across 80 scenes, and a real pallet has one;
* ``ground_truth`` pocket centres, widths, heights. **The x component moves
  too**: a pocket sits on the approach face, which is half the pallet's depth
  in front of its centre, so a shallower or deeper pallet puts it elsewhere
  even at the same pose;
* the occluder's lateral size, which is a fraction of the opening width --
  ``fraction`` is a share of physical width, not of the image;
* ``visibility``: the projected corners move with the pockets, and so do
  ``left_in_view``, ``right_in_view``, ``occluded_side`` and
  ``occluded_fraction_image``.

The seed is kept rather than reset. It identifies the draw that produced these
poses, and the poses are exactly the ones it produced.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from generate_scene_catalogue import project_to_image  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from forklift_core.perception.pallet_geometry import (  # noqa: E402
    PalletGeometry,
    load_pallet_geometry,
)

MIN_DEPTH_M = 0.3
MARGIN_PX = 4


def header_pallet(geometry: PalletGeometry) -> dict:
    """The catalogue header block for this geometry.

    Split decks, not v1's single symmetric thickness: no real pallet has one.
    """
    return {
        "depth_m": geometry.overall_depth_m,
        "width_m": geometry.overall_width_m,
        "height_m": geometry.overall_height_m,
        "deck_bottom_m": geometry.deck_bottom_m,
        "deck_top_m": geometry.deck_top_m,
        "center_spacer_m": geometry.centre_block_width_m,
        "opening_height_m": geometry.block_height_m,
    }


def pocket_centres(geometry: PalletGeometry, x_m, y_m, yaw_rad):
    """(left, right) pocket centres in base_link, on the approach face."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    half_depth = geometry.overall_depth_m / 2
    front_x = x_m - half_depth * c
    front_y = y_m - half_depth * s
    offset = geometry.opening_centre_offset_m
    z = geometry.opening_centre_height_m
    return [
        (front_x - sign * offset * s, front_y + sign * offset * c, z)
        for sign in (1, -1)
    ]


def _corners(geometry, centre, yaw_rad):
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    width, height = geometry.opening_width_m, geometry.block_height_m
    return [
        (centre[0] - dy * s, centre[1] + dy * c, centre[2] + dz)
        for dy, dz in itertools.product(
            (-width / 2, width / 2), (-height / 2, height / 2)
        )
    ]


def _visibility(geometry, camera, x_m, y_m, yaw_rad):
    left, right = pocket_centres(geometry, x_m, y_m, yaw_rad)
    out = {"corners_px": {}}
    for side, centre in (("left", left), ("right", right)):
        projected = [
            project_to_image(p, camera) for p in _corners(geometry, centre, yaw_rad)
        ]
        out[f"{side}_in_view"] = all(
            z >= MIN_DEPTH_M
            and MARGIN_PX <= u <= camera["width"] - MARGIN_PX
            and MARGIN_PX <= v <= camera["height"] - MARGIN_PX
            for u, v, z in projected
        )
        out["corners_px"][side] = [[u, v] for u, v, _ in projected]
    return out


def _image_overlaps(occluder, corners_px, yaw_rad, camera):
    """Per-opening share of image width the occluder covers."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    centre, size = occluder["center_m"], occluder["size_m"]
    box_u = [
        project_to_image(
            (centre[0] + c * dx - s * dy, centre[1] + s * dx + c * dy, centre[2] + dz),
            camera,
        )[0]
        for dx, dy, dz in itertools.product(*[(-v / 2, v / 2) for v in size])
    ]
    overlaps = {}
    for side, corners in corners_px.items():
        low = min(p[0] for p in corners)
        high = max(p[0] for p in corners)
        overlaps[side] = max(0.0, min(high, max(box_u)) - max(low, min(box_u))) / (
            high - low
        )
    return overlaps


def _round(value, digits=6):
    if isinstance(value, dict):
        return {key: _round(item, digits) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_round(item, digits) for item in value]
    if isinstance(value, float):
        return round(value, digits)
    return value


def retarget(catalogue: dict, geometry: PalletGeometry) -> dict:
    camera = catalogue["camera"]
    width = geometry.opening_width_m
    out = dict(catalogue)
    out["catalogue_version"] = geometry.geometry_version
    out["generator"] = "tools/retarget_scene_catalogue.py"
    out["pallet"] = header_pallet(geometry)
    ranges = dict(catalogue["ranges"])
    # One opening width, not a drawn range: the article is manufactured.
    ranges["opening_width_m"] = [width, width]
    out["ranges"] = ranges

    scenes = []
    for scene in catalogue["scenes"]:
        scene = {key: (dict(v) if isinstance(v, dict) else v) for key, v in scene.items()}
        placed = scene["pallet"]
        if placed is None:
            scenes.append(_round(scene))
            continue
        placed["opening_width_m"] = width
        pose = (placed["x_m"], placed["y_m"], placed["yaw_rad"])
        left, right = pocket_centres(geometry, *pose)
        truth = dict(scene["ground_truth"])
        for side, centre in (("left", left), ("right", right)):
            pocket = dict(truth[side])
            pocket["center_m"] = list(centre)
            pocket["width_m"] = width
            pocket["height_m"] = geometry.block_height_m
            truth[side] = pocket
        scene["ground_truth"] = truth

        visibility = dict(scene["visibility"])
        recomputed = _visibility(geometry, camera, *pose)
        visibility["corners_px"] = recomputed["corners_px"]
        visibility["left_in_view"] = recomputed["left_in_view"]
        visibility["right_in_view"] = recomputed["right_in_view"]

        occluder = scene["occluder"]
        if occluder is not None:
            occluder = dict(occluder)
            # `fraction` is a share of physical opening width, not of the image.
            occluder["size_m"] = [
                occluder["size_m"][0],
                occluder["fraction"] * width,
                occluder["size_m"][2],
            ]
            side = occluder["side"]
            centre = left if side == "left" else right
            dx = camera["translation_m"][0] - centre[0]
            dy = camera["translation_m"][1] - centre[1]
            length = math.hypot(dx, dy)
            gap = occluder["gap_m"]
            occluder["center_m"] = [
                centre[0] + gap * dx / length,
                centre[1] + gap * dy / length,
                occluder["center_m"][2],
            ]
            scene["occluder"] = occluder
            overlaps = _image_overlaps(
                occluder, recomputed["corners_px"], pose[2], camera
            )
            visibility["occluded_side"] = side
            visibility["occluded_fraction_image"] = overlaps[side]
        else:
            visibility["occluded_side"] = None
            visibility["occluded_fraction_image"] = None
        scene["visibility"] = visibility
        scenes.append(_round(scene))
    out["scenes"] = scenes
    return _round(out)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--catalogue", type=Path, required=True)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--output", type=Path, help="default: stdout")
    args = parser.parse_args(argv)
    catalogue = yaml.safe_load(args.catalogue.read_text(encoding="utf-8"))
    geometry = load_pallet_geometry(args.geometry)
    text = yaml.safe_dump(retarget(catalogue, geometry), sort_keys=False)
    header = (
        "# Generated by tools/retarget_scene_catalogue.py from\n"
        f"# {args.catalogue.as_posix()} onto {args.geometry.as_posix()}.\n"
        "# Poses, categories, splits, lighting, surfaces and distractors are\n"
        "# copied exactly; only what the pallet's shape decides is recomputed.\n"
    )
    if args.output:
        args.output.write_text(header + text, encoding="utf-8")
    else:
        sys.stdout.write(header + text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
