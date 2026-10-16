#!/usr/bin/env python3
"""Export pallet models as 3D-printable STL parts (binary, millimetres).

The same box lists the simulator renders (forklift_core.perception.pallet_model and
pallet_damage, the stage-1A scene recipes) become printable parts, so a printed
pallet is the exact geometry of a simulated one: known shapes, damaged variants,
non-standard recipes, and a flat filament test plate for depth-noise captures.

A pallet larger than the print bed is cut into tiles that fit (even cuts along x
and y; glue them side by side) or exported member by member (each board and block
separately, members longer than the bed cut along their length, with positions for
assembly). The mesh is the exact union of the
axis-aligned boxes on their own edge grid, so touching members leave no inner faces.

  python tools/export_pallet_stl.py --shape epal6 --bed 256,256,256 --out prints/epal6
  python tools/export_pallet_stl.py --recipe s5_epal6_1000483 --instance 0 --scale 0.5 --out prints/s5
  python tools/export_pallet_stl.py --plate 200,200,5 --out prints/plate
"""

import argparse
import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from forklift_core.perception.pallet_damage import apply_damage
from forklift_core.perception.pallet_model import PalletBox, spec_boxes

PLA_DENSITY_G_CM3 = 1.24  # solid PLA; infill makes a print lighter


@dataclass(frozen=True)
class Part:
    name: str
    triangles: np.ndarray  # (n, 3, 3) metres, outward winding


def pallet_boxes(spec, damage) -> list[PalletBox]:
    return apply_damage(spec_boxes(spec), damage)


def _intervals(boxes):
    lo = np.array([[c - s / 2 for c, s in zip(b.centre_m, b.size_m)] for b in boxes])
    hi = np.array([[c + s / 2 for c, s in zip(b.centre_m, b.size_m)] for b in boxes])
    return lo, hi


def _grid(boxes, bounds=None):
    lo, hi = _intervals(boxes)
    axes = []
    for a in range(3):
        values = np.concatenate((lo[:, a], hi[:, a]))
        if bounds is not None:
            values = np.clip(np.concatenate((values, bounds[a])), *bounds[a])
        axes.append(np.unique(np.round(values, 12)))
    occ = np.zeros(tuple(len(g) - 1 for g in axes), bool)
    for l, h in zip(lo, hi):
        idx = []
        for a in range(3):
            l_a, h_a = (l[a], h[a]) if bounds is None else (max(l[a], bounds[a][0]), min(h[a], bounds[a][1]))
            if h_a <= l_a:
                break
            idx.append(slice(np.searchsorted(axes[a], round(l_a, 12)), np.searchsorted(axes[a], round(h_a, 12))))
        else:
            occ[tuple(idx)] = True
    return axes, occ


def union_mesh(boxes, *, bounds=None) -> np.ndarray:
    """Triangles (n, 3, 3) of the union of axis-aligned boxes, optionally clipped to bounds ((x0,x1),(y0,y1),(z0,z1))."""
    axes, occ = _grid(boxes, bounds)
    padded = np.pad(occ, 1)
    out = []
    for a in range(3):
        b, c = (a + 1) % 3, (a + 2) % 3
        # transitions along axis a between consecutive padded cells
        lower = np.take(padded, range(0, padded.shape[a] - 1), axis=a)
        upper = np.take(padded, range(1, padded.shape[a]), axis=a)
        for solid_low, normal in ((True, 1.0), (False, -1.0)):
            mask = (lower == solid_low) & (upper != solid_low)
            # trim the padding on the other two axes
            sl = [slice(None)] * 3
            sl[b] = slice(1, -1)
            sl[c] = slice(1, -1)
            mask = mask[tuple(sl)]
            for idx in zip(*np.nonzero(mask)):
                plane = axes[a][idx[a]]
                b0, b1 = axes[b][idx[b]], axes[b][idx[b] + 1]
                c0, c1 = axes[c][idx[c]], axes[c][idx[c] + 1]
                quad = []
                for vb, vc in ((b0, c0), (b1, c0), (b1, c1), (b0, c1)):
                    v = [0.0, 0.0, 0.0]
                    v[a], v[b], v[c] = plane, vb, vc
                    quad.append(v)
                q = np.array(quad)
                if normal < 0:
                    q = q[::-1]
                out.append([q[0], q[1], q[2]])
                out.append([q[0], q[2], q[3]])
    return np.array(out, dtype=float).reshape(-1, 3, 3)


def union_volume(boxes, bounds=None) -> float:
    axes, occ = _grid(boxes, bounds)
    widths = np.meshgrid(*[np.diff(g) for g in axes], indexing="ij")
    return float(np.sum(widths[0] * widths[1] * widths[2] * occ))


def _extent(boxes):
    lo, hi = _intervals(boxes)
    return lo.min(axis=0), hi.max(axis=0)


def tile_meshes(boxes, *, bed_m) -> list[Part]:
    lo, hi = _extent(boxes)
    if hi[2] - lo[2] > bed_m[2] + 1e-12:
        raise ValueError(f"pallet height {hi[2] - lo[2]:.3f} m exceeds the bed height {bed_m[2]:.3f} m")
    counts = [max(1, math.ceil((hi[a] - lo[a]) / bed_m[a] - 1e-9)) for a in range(2)]
    cuts = [np.linspace(lo[a], hi[a], counts[a] + 1) for a in range(2)]
    parts = []
    for i in range(counts[0]):
        for j in range(counts[1]):
            bounds = ((cuts[0][i], cuts[0][i + 1]), (cuts[1][j], cuts[1][j + 1]), (lo[2], hi[2]))
            tri = union_mesh(boxes, bounds=bounds)
            if len(tri):
                parts.append(Part(f"tile_x{i}_y{j}", tri))
    return parts


def write_stl(path: Path, triangles_m: np.ndarray, *, scale: float, name: str) -> None:
    tri = np.asarray(triangles_m, dtype=float) * 1000.0 * scale
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)
    header = f"forklift pallet print: {name}".encode()[:80].ljust(80, b" ")
    with open(path, "wb") as f:
        f.write(header + struct.pack("<I", len(tri)))
        for n, t in zip(normals, tri):
            f.write(struct.pack("<12fH", *n, *t.ravel(), 0))


def _part_record(path: Path, part: Part, scale: float) -> dict:
    pts = part.triangles.reshape(-1, 3) * 1000.0 * scale
    volume_m3 = float(np.sum(np.einsum("ij,ij->i", part.triangles[:, 0],
                                       np.cross(part.triangles[:, 1], part.triangles[:, 2]))) / 6.0)
    volume_cm3 = volume_m3 * 1e6 * scale ** 3
    return {"name": part.name, "file": path.name, "triangles": int(len(part.triangles)),
            "size_mm": [round(float(v), 3) for v in pts.max(axis=0) - pts.min(axis=0)],
            "position_mm": [round(float(v), 3) for v in pts.min(axis=0)],
            "volume_cm3": volume_cm3, "solid_pla_g": volume_cm3 * PLA_DENSITY_G_CM3}


def export(out: Path, boxes, *, scale: float, bed_m, split: str, source: dict) -> dict:
    out.mkdir(parents=True, exist_ok=False)
    if split == "members":
        parts = []
        for b in boxes:  # members longer than the bed are cut along their length
            pieces = tile_meshes([b], bed_m=[lim / scale for lim in bed_m])
            parts += [Part(b.name if len(pieces) == 1 else f"{b.name}_{piece.name}", piece.triangles)
                      for piece in pieces]
    else:
        parts = tile_meshes(boxes, bed_m=[lim / scale for lim in bed_m])
    records = []
    for part in parts:
        path = out / f"{part.name}.stl"
        write_stl(path, part.triangles, scale=scale, name=f"{source.get('name', 'pallet')} {part.name}")
        records.append(_part_record(path, part, scale))
    lo, hi = _extent(boxes)
    manifest = {"source": source, "scale": scale, "units": "mm", "split": split,
                "bed_mm": [v * 1000.0 for v in bed_m],
                "overall_mm": [round(float(v) * 1000.0 * scale, 3) for v in hi - lo],
                "volume_cm3": union_volume(boxes) * 1e6 * scale ** 3,
                "parts": records,
                "notes": "Members positions are in the pallet frame (x along the depth, y across, z up). Tiles are "
                         "cut evenly in x and y and glue side by side. Print in a matte, light filament: the "
                         "D435i's infrared pattern leaves holes on glossy, black or translucent plastic."}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
    return manifest


def main(argv=None):
    p = argparse.ArgumentParser()
    what = p.add_mutually_exclusive_group(required=True)
    what.add_argument("--shape", choices=("epal6", "t11_06"))
    what.add_argument("--recipe", help="a scene id from the recognition catalogue")
    what.add_argument("--plate", help="W,D,T mm: a flat filament test plate")
    p.add_argument("--instance", type=int, default=0, help="pallet instance of --recipe")
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--bed", default="256,256,256", help="print bed X,Y,Z mm")
    p.add_argument("--split", choices=("tiles", "members"), default="tiles")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    if not (math.isfinite(a.scale) and a.scale > 0):
        raise SystemExit("--scale must be positive")
    bed_m = [float(v) / 1000.0 for v in a.bed.split(",")]
    if a.plate:
        w, d, t = (float(v) / 1000.0 for v in a.plate.split(","))
        boxes, source = [PalletBox("plate", (w, d, t), (0.0, 0.0, t / 2))], {"name": "plate", "plate_mm": a.plate}
    else:
        from tools import recognition_scenes as rs

        if a.shape:
            spec, damage, source = rs.known_spec(a.shape), (), {"name": a.shape, "shape": a.shape}
        else:
            recipe = next((r for r in rs.catalogue() if r.scene_id == a.recipe), None)
            if recipe is None or not 0 <= a.instance < len(recipe.instances):
                raise SystemExit(f"no instance {a.instance} in recipe {a.recipe}")
            inst = recipe.instances[a.instance]
            spec, damage = inst.spec, inst.damage
            source = {"name": f"{a.recipe}_inst{a.instance}", "recipe": a.recipe, "instance": a.instance,
                      "case": recipe.case, "damage": [d.to_json() for d in damage]}
        source["spec"] = spec.to_json()
        boxes = pallet_boxes(spec, damage)
    manifest = export(a.out, boxes, scale=a.scale, bed_m=bed_m, split=a.split, source=source)
    print(json.dumps({"parts": len(manifest["parts"]), "overall_mm": manifest["overall_mm"],
                      "volume_cm3": round(manifest["volume_cm3"], 1)}))


if __name__ == "__main__":
    main()
