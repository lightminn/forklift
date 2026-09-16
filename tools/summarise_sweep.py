#!/usr/bin/env python3
"""Recompute every summary statistic an ADR sentence might claim, from a sweep's output.

The 14th review of docs/decisions/0003 found ten falsehoods in one revision, eight
of which were refuted by recomputing from output that revision had already produced:
a minimum quoted as 0.60 that was 0.524, "five rejected cells" that were six, a
dead-cell count copied from a different mount's file, an all-seed distance copied
from the other quantisation condition.  Those are argmin/argmax/count claims made by
eye.  This tool makes them instead.

    tools/summarise_sweep.py grids artifacts/hpc/<run-id>/out
    tools/summarise_sweep.py evidence artifacts/hpc/<run-id>/out --markdown

`grids` reads every `grid` output and reports, per file: first non-zero cell, first
all-seed cell, all-seed/partial/dead counts, and the derived gap and blind zone.
Derived quantities are computed HERE AND ONLY HERE, so a row cannot say "0.28 m" in
one column and "no blind zone" in its annotation.

`evidence` reads every `evidence` output, computes upper/sum(supports) per cell, and
reports the true minimum margin, which cells the gate rejected, and which structures
differ -- with the argmin cell named, not just its value.
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

# Geometry of the derived columns.  One definition, used by every caller.
T11_06_HALF_DEPTH_M = 0.330   # config/pallet_geometry_t11_06.yaml overall_depth_m / 2
FORK_TIP_X_M = 0.950          # sim/models/dls08_provisional/forklift.xml:269 blade tip
TARGET_INSERTION_M = 0.360    # tools/preview_docking.py:417 default

GRID_ROW = re.compile(r"^\s*([\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+(\d+)/(\d+)")
CELLS = re.compile(r"all-seed (\d+), partial (\d+), dead (\d+)")
EV = re.compile(r"supports \((\d+), (\d+), (\d+)\)\s+lower\s+(\d+)\s+upper\s+(\d+)")
GATE = re.compile(r"gate min\s+\d+ vs min_band_points \d+\s+->\s+(\w+)")


def derived(first_detection_m: float, half_depth_m: float = T11_06_HALF_DEPTH_M) -> tuple[float, float]:
    """(gap from fork tip to pallet face, blind zone to full insertion), both in m."""
    gap = first_detection_m - half_depth_m - FORK_TIP_X_M
    return gap, gap + TARGET_INSERTION_M


def read_grid(path: Path) -> dict | None:
    rows, counts = [], None
    for line in path.read_text().splitlines():
        m = GRID_ROW.match(line)
        if m:
            rows.append((float(m.group(1)), int(m.group(4)), int(m.group(5))))
            continue
        c = CELLS.search(line)
        if c:
            counts = tuple(int(c.group(i)) for i in (1, 2, 3))
    if not rows:
        return None
    nonzero = next((r for r in rows if r[1] > 0), None)
    allseed = next((r for r in rows if r[1] == r[2]), None)
    out = {"name": path.stem, "cells": counts, "span": (rows[0][0], rows[-1][0]),
           "first_nonzero": nonzero, "first_allseed": allseed}
    if nonzero:
        out["gap_m"], out["blind_m"] = derived(nonzero[0])
    return out


def cmd_grids(args) -> int:
    print(f"{'name':<26} {'span':>13} {'first!=0':>14} {'first all':>10} "
          f"{'a/p/d':>14} {'gap_m':>8} {'blind_m':>8}")
    for path in sorted(Path(args.out_dir).glob("*.txt")):
        g = read_grid(path)
        if not g or g["first_nonzero"] is None and g["cells"] is None:
            continue
        nz = g["first_nonzero"]
        al = g["first_allseed"]
        print(f"{g['name']:<26} {g['span'][0]:.2f}-{g['span'][1]:>5.2f} "
              f"{(f'{nz[0]:.3f} ({nz[1]}/{nz[2]})' if nz else '-'):>14} "
              f"{(f'{al[0]:.3f}' if al else '-'):>10} "
              f"{('/'.join(map(str, g['cells'])) if g['cells'] else '-'):>14} "
              f"{g.get('gap_m', float('nan')):8.3f} {g.get('blind_m', float('nan')):8.3f}")
    return 0


def cmd_evidence(args) -> int:
    cells: dict[tuple[float, float, str], dict] = {}
    for path in sorted(Path(args.out_dir).glob("ev_*.txt")):
        m = re.match(r"ev_(\w+?)_x([\d.]+)_y([\d.]+)_(noq|q)$", path.stem)
        if not m:
            continue
        struct, x, yaw = m.group(1), float(m.group(2)), float(m.group(3))
        text = path.read_text()
        e = EV.search(text)
        if not e:
            continue
        sup = sum(int(e.group(i)) for i in (1, 2, 3))
        gate = GATE.search(text)
        cells[(x, yaw, struct)] = {
            "ratio": int(e.group(5)) / sup, "lower": int(e.group(4)),
            "upper": int(e.group(5)), "sup": sup,
            "pass": (gate.group(1) == "pass") if gate else None,
        }
    xs = sorted({k[0] for k in cells})
    yaws = sorted({k[1] for k in cells})
    structs = sorted({k[2] for k in cells})
    ref = args.reference
    others = [s for s in structs if s != ref]

    if args.markdown:
        print(f"| x_m | yaw | " + " | ".join(structs) + " | 관문(" + ref + ") |")
        print("|" + "---|" * (len(structs) + 3))
        for x in xs:
            for y in yaws:
                vals = " | ".join(
                    f"{cells[(x, y, s)]['ratio']:.3f}" if (x, y, s) in cells else "-"
                    for s in structs)
                g = cells.get((x, y, ref), {}).get("pass")
                print(f"| {x:.1f} | {y:.2f} | {vals} | {'통과' if g else '거부' if g is not None else '-'} |")

    print(f"\n# reference structure: {ref}")
    for other in others:
        pairs = [(x, y, cells[(x, y, ref)]["ratio"] - cells[(x, y, other)]["ratio"])
                 for x in xs for y in yaws if (x, y, ref) in cells and (x, y, other) in cells]
        if not pairs:
            continue
        passing = [p for p in pairs if cells[(p[0], p[1], ref)]["pass"]]
        lo = min(pairs, key=lambda p: p[2])
        print(f"{ref} - {other}: cells {len(pairs)}, "
              f"min margin {lo[2]:+.3f} at x={lo[0]} yaw={lo[1]}", end="")
        if passing:
            lp = min(passing, key=lambda p: p[2])
            print(f" | gate-passing only: {len(passing)} cells, "
                  f"min {lp[2]:+.3f} at x={lp[0]} yaw={lp[1]}", end="")
        differ = [p for p in pairs if abs(p[2]) > 1e-9]
        print(f" | differ in {len(differ)}/{len(pairs)} cells")
        ref_lo = min((cells[(x, y, ref)]["ratio"] for x, y in [(p[0], p[1]) for p in pairs]))
        oth_hi = max((cells[(x, y, other)]["ratio"] for x, y in [(p[0], p[1]) for p in pairs]))
        print(f"    fixed threshold: {ref} min {ref_lo:.3f} vs {other} max {oth_hi:.3f}"
              f" -> {'SEPARABLE' if ref_lo > oth_hi else 'OVERLAP'}")
    rejected = [(x, y) for x in xs for y in yaws
                if (x, y, ref) in cells and cells[(x, y, ref)]["pass"] is False]
    print(f"gate-rejected ({ref}): {len(rejected)} cells -> {rejected}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("grids"); g.add_argument("out_dir"); g.set_defaults(fn=cmd_grids)
    e = sub.add_parser("evidence"); e.add_argument("out_dir")
    e.add_argument("--markdown", action="store_true")
    e.add_argument("--reference", default="pallet")
    e.set_defaults(fn=cmd_evidence)
    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    raise SystemExit(main())
