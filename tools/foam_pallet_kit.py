#!/usr/bin/env python3
"""Build a test pallet from XPS foam boards and 3D-printed blocks.

The boards (bottom boards, stringers, top boards) are plain rectangles, so they are
cut from foam sheets; the blocks are printed. Foam comes in fixed thicknesses (often
10, 20, 30 mm), so each board gets the closest stack of available sheets, and the
printed blocks absorb the bottom boards' change so the x-entry lane keeps its nominal
ceiling (the lane floor there is the ground). The y-entry lane (floor = bottom-board
top, ceiling = top-board underside) and the overall height still move with the
boards; kit.json reports every lane against the nominal one with the fork check of
forklift_core.perception.fork_fit. An optional facing (--facing-mm) under the boards
that form lane ceilings is the bottom layer of their stack, so it does not lower the
ceiling. kit.json records the as-built spec; after assembly the measured dimensions
are the truth.

  python tools/foam_pallet_kit.py --shape epal6 --sheets 10,20,30 --out prints/epal6_foam
  python tools/foam_pallet_kit.py --shape epal6 --compare "10;20;30;10,20;10,20,30" --out prints/epal6_compare
"""

import argparse
import csv
import itertools
import json
import sys
from dataclasses import dataclass, replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from forklift_core.perception.fork_fit import TEST_FORK_SPACING_M, VERTICAL_MARGIN_M, ForkSetup, lane_reachable  # noqa: E402,E501
from forklift_core.perception.pallet_model import PalletSpec, face_layout, spec_boxes  # noqa: E402
from tools import export_pallet_stl as ex  # noqa: E402

GROUPS = {"bottom": ("bottom_board", "deck_bottom_m"), "stringer": ("stringer", "stringer_m"),
          "top": ("top_board", "top_thickness_m")}
CEILING_GROUP = {"x": "stringer", "y": "top"}  # the board whose underside is the lane ceiling
KERF_MM = 3.0


@dataclass(frozen=True)
class FoamBuild:
    nominal: PalletSpec
    spec: PalletSpec  # as built: foam board thicknesses, nominal blocks
    sheets_mm: tuple
    stacks: dict      # group -> tuple of foam sheet thicknesses (mm), thickest first
    choice: dict      # group -> {"nominal_mm", "stack_mm", "facing_mm", "built_mm"}
    facing_mm: float = 0.0


def choose_stack(target_mm: float, sheets_mm, max_layers: int = 3) -> tuple:
    """Closest sum of up to max_layers sheets; ties go to fewer layers, then to the thicker stack."""
    best = None
    for n in range(1, max_layers + 1):
        for combo in itertools.combinations_with_replacement(sorted(set(sheets_mm), reverse=True), n):
            key = (round(abs(sum(combo) - target_mm), 9), n, -sum(combo))
            if best is None or key < best[0]:
                best = (key, tuple(sorted(combo, reverse=True)))
    return best[1]


def as_built(nominal: PalletSpec, sheets_mm, *, facing_mm: float = 0.0) -> FoamBuild:
    if not sheets_mm or any(s <= 0 for s in sheets_mm):
        raise ValueError("sheet thicknesses must be positive")
    if facing_mm < 0:
        raise ValueError("facing_mm must not be negative")
    faced = {CEILING_GROUP[axis] for axis in nominal.entry_axes} if facing_mm > 0 else set()
    stacks, choice, changes = {}, {}, {}
    for group, (_, field) in GROUPS.items():
        target = getattr(nominal, field) * 1000.0
        facing = facing_mm if group in faced else 0.0
        if target - facing <= 0:
            raise ValueError(f"the facing is thicker than the {group} board")
        stack = choose_stack(target - facing, sheets_mm)
        stacks[group] = stack
        choice[group] = {"nominal_mm": target, "stack_mm": list(stack), "facing_mm": facing,
                         "built_mm": float(sum(stack)) + facing}
        changes[field] = choice[group]["built_mm"] / 1000.0
    # printed blocks take up the bottom boards' change: the x lane ceiling stays where it was
    changes["opening_height_m"] = nominal.deck_bottom_m + nominal.opening_height_m - changes["deck_bottom_m"]
    if changes["opening_height_m"] <= 0:
        raise ValueError("the bottom boards are thicker than the x lane is tall")
    spec = replace(nominal, name=f"{nominal.name} foam {'+'.join(map(str, sorted(set(sheets_mm))))}", known=False,
                   **changes)
    return FoamBuild(nominal, spec, tuple(sheets_mm), stacks, choice, float(facing_mm))


def _lane(spec, axis, setup):
    layout = face_layout(spec, axis)
    lift = max(0.0, layout.lane_floor_m + VERTICAL_MARGIN_M - setup.blade_bottom_m)
    clearance = layout.lane_ceiling_m - (setup.blade_top_m + lift)
    return {"floor_mm": layout.lane_floor_m * 1000.0, "ceiling_mm": layout.lane_ceiling_m * 1000.0,
            "height_mm": (layout.lane_ceiling_m - layout.lane_floor_m) * 1000.0,
            "fork_clearance_mm": clearance * 1000.0,
            "fork_fits": lane_reachable(spec, axis, setup, support_z_m=0.0)}


def lanes(build: FoamBuild, shape: str) -> dict:
    """Fork lanes per entry axis, nominal and as built, with the project's test fork setup."""
    setup = ForkSetup(TEST_FORK_SPACING_M[shape])
    return {axis: {"nominal": _lane(build.nominal, axis, setup), "built": _lane(build.spec, axis, setup)}
            for axis in build.nominal.entry_axes}


def cut_list(build: FoamBuild) -> list[dict]:
    rows = {}
    for box in spec_boxes(build.spec):
        group = next((g for g, (prefix, _) in GROUPS.items() if box.name.startswith(prefix)), None)
        if group is None:
            continue
        length, width = sorted((box.size_m[0] * 1000.0, box.size_m[1] * 1000.0), reverse=True)
        layers = [("foam", s) for s in build.stacks[group]]
        if build.choice[group]["facing_mm"] > 0:
            layers.append(("facing", build.choice[group]["facing_mm"]))  # the bottom layer, under the foam
        for material, sheet in layers:
            key = (GROUPS[group][0], material, round(length, 3), round(width, 3), sheet)
            row = rows.setdefault(key, {"group": key[0], "material": material, "length_mm": key[2],
                                        "width_mm": key[3], "sheet_mm": sheet, "quantity": 0, "members": []})
            row["quantity"] += 1
            row["members"].append(box.name)
    return sorted(rows.values(), key=lambda r: (r["group"], r["material"], -r["length_mm"], -r["width_mm"],
                                                -r["sheet_mm"]))


def sheet_usage(rows, *, sheet_mm=(900, 1800), kerf_mm: float = KERF_MM) -> dict:
    """Shelf packing per sheet thickness (pieces may turn 90 degrees); an estimate, not an optimal nesting."""
    width, length = sorted(sheet_mm)
    out = {}
    rows = [r for r in rows if r.get("material", "foam") == "foam"]
    for thickness in sorted({r["sheet_mm"] for r in rows}):
        pieces = []
        for r in rows:
            if r["sheet_mm"] != thickness:
                continue
            for i in range(r["quantity"]):
                l, w = r["length_mm"], r["width_mm"]
                if l > length or w > width:
                    if l <= width and w <= length:
                        l, w = w, l
                    else:
                        raise ValueError(f"{r['group']} {l:.0f} x {w:.0f} mm is larger than the sheet "
                                         f"{length:.0f} x {width:.0f} mm")
                pieces.append({"member": r["members"][i % len(r["members"])], "l": l, "w": w})
        pieces.sort(key=lambda p: (-p["w"], -p["l"]))
        sheets, sheet, x, y, shelf = [], [], 0.0, 0.0, 0.0
        for p in pieces:
            if x + p["l"] > length:  # next shelf
                x, y, shelf = 0.0, y + shelf + kerf_mm, 0.0
            if y + p["w"] > width:   # next sheet
                sheets.append(sheet)
                sheet, x, y, shelf = [], 0.0, 0.0, 0.0
            sheet.append({**p, "x": x, "y": y})
            x += p["l"] + kerf_mm
            shelf = max(shelf, p["w"])
        if sheet:
            sheets.append(sheet)
        used = sum(p["l"] * p["w"] for p in pieces)
        out[thickness] = {"pieces": len(pieces), "sheets": len(sheets), "sheet_mm": [length, width],
                          "fill": used / (len(sheets) * length * width), "layout": sheets}
    return out


def summary(build: FoamBuild, shape: str) -> dict:
    n, b = build.nominal, build.spec
    return {
        "sheets_mm": sorted(set(build.sheets_mm)),
        "facing_mm": build.facing_mm,
        "boards": {g: dict(c) for g, c in build.choice.items()},
        "block_height_mm": {"nominal": n.opening_height_m * 1000.0, "built": b.opening_height_m * 1000.0},
        "lanes": lanes(build, shape),
        "overall_height_mm": {"nominal": n.overall_height_m * 1000.0, "built": b.overall_height_m * 1000.0},
    }


def write_kit(out: Path, build: FoamBuild, shape: str, *, sheet_mm, bed_mm) -> dict:
    rows = cut_list(build)
    usage = sheet_usage(rows, sheet_mm=sheet_mm)
    out.mkdir(parents=True, exist_ok=False)
    blocks = [b for b in spec_boxes(build.spec) if b.name.startswith("block")]
    ex.export(out / "blocks", blocks, scale=1.0, bed_m=[v / 1000.0 for v in bed_mm], split="members",
              source={"name": f"{build.spec.name} blocks", "spec": build.spec.to_json()})
    with open(out / "cut_list.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["group", "material", "length_mm", "width_mm", "sheet_mm", "quantity",
                                          "members"])
        w.writeheader()
        for r in rows:
            w.writerow({**r, "members": " ".join(r["members"])})
    assembly = [{"name": b.name, "material": "printed" if b.name.startswith("block") else "foam (+facing)"
                 if build.facing_mm and any(b.name.startswith(GROUPS[CEILING_GROUP[a]][0])
                                            for a in build.nominal.entry_axes) else "foam",
                 "size_mm": [round(v * 1000.0, 3) for v in b.size_m],
                 "centre_mm": [round(v * 1000.0, 3) for v in b.centre_m]} for b in spec_boxes(build.spec)]
    data = {**summary(build, shape), "nominal": build.nominal.to_json(), "as_built": build.spec.to_json(),
            "cut_list": rows, "sheet_usage": {str(k): v for k, v in usage.items()}, "assembly": assembly,
            "notes": ["as-built spec is a design value; measure the assembled pallet and use the measurements as truth",
                      "printed blocks are sized so the x-entry lane keeps its nominal ceiling; the y lane and the "
                      "overall height follow the boards (see lanes)",
                      "foam: extruded polystyrene (XPS); solvents melt it. XPS and printed plastic are non-porous, so "
                      "water-based glues cure poorly between them: use a polystyrene-safe adhesive rated for both "
                      "(two-part epoxy, polyurethane foam adhesive) or foam tape, and test a bonded XPS-XPS and "
                      "XPS-filament coupon before any load test",
                      "lane ceilings take the forks: --facing-mm puts a thin plastic facing as the bottom layer of the "
                      "ceiling boards without lowering the ceiling",
                      f"sheet layout is a shelf-packing estimate with {KERF_MM:.0f} mm kerf"]}
    (out / "kit.json").write_text(json.dumps(data, indent=1))
    for thickness, u in usage.items():
        for i, sheet in enumerate(u["layout"]):
            (out / f"sheet_layout_{thickness}mm_{i + 1}.svg").write_text(_svg(sheet, u["sheet_mm"], thickness, i + 1))
    return data


def _svg(sheet, size_mm, thickness, index) -> str:
    """One foam sheet with its pieces, drawn at 1 px per 2 mm with labels."""
    length, width = size_mm
    s = 0.5
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{length * s + 20:.0f}" height="{width * s + 40:.0f}">',
             f'<text x="10" y="16" font-size="14">sheet {index}: {length:.0f} x {width:.0f} mm, {thickness} mm XPS</text>',
             f'<rect x="10" y="25" width="{length * s:.1f}" height="{width * s:.1f}" fill="#fde" stroke="#000"/>']
    for p in sheet:
        x, y, w, h = 10 + p["x"] * s, 25 + p["y"] * s, p["l"] * s, p["w"] * s
        parts.append(f'<rect class="piece" x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
                     f'fill="#fff" stroke="#333"/>')
        parts.append(f'<text x="{x + 3:.1f}" y="{y + 12:.1f}" font-size="10">{p["member"]} '
                     f'{p["l"]:.0f}x{p["w"]:.0f}</text>')
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def compare(nominal: PalletSpec, shape: str, sets, *, sheet_mm, facing_mm: float = 0.0) -> list[dict]:
    table = []
    for sheets in sets:
        build = as_built(nominal, sheets, facing_mm=facing_mm)
        usage = sheet_usage(cut_list(build), sheet_mm=sheet_mm)
        table.append({**summary(build, shape), "sheets_needed": {str(t): u["sheets"] for t, u in usage.items()}})
    return table


def _markdown(shape: str, table) -> str:
    axes = list(table[0]["lanes"])

    def lane(r, a):
        b, n = r["lanes"][a]["built"], r["lanes"][a]["nominal"]
        ok = "fits" if b["fork_fits"] else "NO FIT"
        return (f"{b['floor_mm']:.0f}-{b['ceiling_mm']:.0f} (nominal {n['floor_mm']:.0f}-{n['ceiling_mm']:.0f}), "
                f"clearance {b['fork_clearance_mm']:.1f}, {ok}")

    head = " | ".join(f"{a} lane floor-ceiling (mm)" for a in axes)
    lines = [f"# {shape}: foam boards + printed blocks (facing {table[0]['facing_mm']:g} mm)", "",
             f"| available sheets (mm) | bottom / stringer / top (mm) | block height (mm) | {head} | "
             "overall height (mm) | sheets needed |",
             "|---|---|---|" + "---|" * len(axes) + "---|---|"]
    for r in table:
        def stack(g):
            c = r["boards"][g]
            return "+".join(map(str, c["stack_mm"])) + (f"+{c['facing_mm']:g}f" if c["facing_mm"] else "")
        lines.append(f"| {', '.join(map(str, r['sheets_mm']))} | {' / '.join(stack(g) for g in ('bottom', 'stringer', 'top'))} | "
                     f"{r['block_height_mm']['built']:.0f} (nominal {r['block_height_mm']['nominal']:.0f}) | "
                     + " | ".join(lane(r, a) for a in axes)
                     + f" | {r['overall_height_mm']['built']:.0f} (nominal {r['overall_height_mm']['nominal']:.0f}) | "
                     f"{', '.join(f'{t} mm: {n}' for t, n in r['sheets_needed'].items())} |")
    lines += ["", "Printed blocks keep the x lane ceiling; clearance is the space above the test forks in the lane "
                  "(fork_fit: at least 5 mm). 'f' marks the facing layer."]
    return "\n".join(lines) + "\n"


def _sizes(text):
    try:
        values = [float(v) for v in text.split(",") if v.strip()]
    except ValueError:
        raise SystemExit(f"bad size list {text!r}") from None
    if not values or any(v <= 0 for v in values):
        raise SystemExit(f"sizes must be positive: {text!r}")
    return [int(v) if v.is_integer() else v for v in values]


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--shape", choices=("epal6", "t11_06"), required=True)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sheets", help="available XPS thicknesses in mm, e.g. 10,20,30")
    mode.add_argument("--compare", help="several sets separated by ';', e.g. '10;20;10,20'")
    p.add_argument("--sheet-size", default="900,1800", help="foam sheet size in mm")
    p.add_argument("--facing-mm", type=float, default=0.0, help="plastic facing under the lane-ceiling boards")
    p.add_argument("--bed", default="325,320,320", help="print bed X,Y,Z mm for the blocks")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    from tools import recognition_scenes as rs

    nominal = rs.known_spec(a.shape)
    sheet_mm = tuple(_sizes(a.sheet_size))
    if len(sheet_mm) != 2:
        raise SystemExit("--sheet-size needs two numbers")
    if a.sheets:
        data = write_kit(a.out, as_built(nominal, _sizes(a.sheets), facing_mm=a.facing_mm), a.shape,
                         sheet_mm=sheet_mm, bed_mm=_sizes(a.bed))
        print(json.dumps({k: data[k] for k in ("boards", "block_height_mm", "lanes", "overall_height_mm")}))
    else:
        table = compare(nominal, a.shape, [_sizes(s) for s in a.compare.split(";")], sheet_mm=sheet_mm,
                        facing_mm=a.facing_mm)
        a.out.mkdir(parents=True, exist_ok=False)
        (a.out / "compare.json").write_text(json.dumps(table, indent=1))
        (a.out / "compare.md").write_text(_markdown(a.shape, table))
        print((a.out / "compare.md").read_text())


if __name__ == "__main__":
    main()
