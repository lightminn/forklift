"""Draw random factory floors from forklift_core.planning.world_gen.

    python tools/world_preview.py --seeds 100-102 --output <dir> [--trucks 10]

Plan: docs/plans/2026-10-09-fleet-random-worlds.md (W0). One PNG per
structure and seed plus a contact sheet, and worlds.json with the counts
(slots, holding places, dropped candidates). Geometry is the measured chassis,
as fleet_cpu_sim builds it.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from math import cos, sin
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import Rectangle as Patch  # noqa: E402

from forklift_core.planning.world_gen import (  # noqa: E402
    STRUCTURES,
    holding_box,
    make_world,
    pallet_box,
)

ROOT = Path(__file__).resolve().parents[1]
FONTS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
]
COLOURS = {
    "wall": "#5b6270",
    "outside": "#3a3f4a",
    "rack": "#d08a2e",
    "machine": "#4f7bbf",
    "column": "#8a8f99",
    "store": "#b9a27a",
}
NAMES = {"rack": "① 랙 창고", "shop": "② 공정 공장", "multi": "③ 복합동"}


def _fleet_tool():
    path = ROOT / "tools/fleet_cpu_sim.py"
    spec = importlib.util.spec_from_file_location("fleet_cpu_sim", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def draw(ax, world, geometry) -> None:
    for kind, b in world.obstacles:
        ax.add_patch(Patch((b.x0, b.y0), b.w, b.h, color=COLOURS.get(kind, "#777"), lw=0))
    for name, z in world.zones:
        if name in ("공장동",):
            continue
        ax.add_patch(
            Patch((z.x0, z.y0), z.w, z.h, fill=False, ls="--", lw=0.6, ec="#9aa3b5")
        )
        ax.text(z.x0 + 0.5, z.y1 - 0.5, name, fontsize=6, va="top", color="#c9d1e0")
    for d in world.doors:
        ax.add_patch(Patch((d.x0, d.y0), d.w, d.h, color="#7fd18b", lw=0))
    for slot in world.slots:
        b = pallet_box(slot.site, geometry)
        colour = "#5ec06b" if slot.role == "source" else "#f08a4b"
        ax.add_patch(Patch((b.x0, b.y0), b.w, b.h, color=colour, lw=0))
        ax.arrow(
            slot.site.x_m - 1.2 * cos(slot.site.yaw_rad),
            slot.site.y_m - 1.2 * sin(slot.site.yaw_rad),
            0.6 * cos(slot.site.yaw_rad),
            0.6 * sin(slot.site.yaw_rad),
            width=0.05,
            head_width=0.3,
            color=colour,
        )
    for pose in world.holding:
        b = holding_box(pose, geometry, 0.0)
        ax.add_patch(Patch((b.x0, b.y0), b.w, b.h, fill=False, ec="#6fb3ff", lw=0.8))
    bd = world.bounds
    ax.set_xlim(bd.x_min_m - 2, bd.x_max_m + 2)
    ax.set_ylim(bd.y_min_m - 2, bd.y_max_m + 2)
    ax.set_aspect("equal")
    ax.set_facecolor("#1d2129")
    ax.tick_params(labelsize=6, colors="#9aa3b5")
    counts = {r: sum(s.role == r for s in world.slots) for r in ("source", "sink")}
    ax.set_title(
        f"{NAMES[world.structure]} · seed {world.seed} · {world.size_m[0]:.0f}×{world.size_m[1]:.0f} m"
        f"\n출발 {counts['source']} · 도착 {counts['sink']} · 대기 {len(world.holding)}",
        fontsize=8,
        color="#e6e9ef",
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trucks", type=int, default=10)
    parser.add_argument(
        "--forklift-urdf", type=Path, default=ROOT / "sim/models/dls08_measured/forklift.urdf"
    )
    parser.add_argument(
        "--pallet-geometry", type=Path, default=ROOT / "config/pallet_geometry_epal6.yaml"
    )
    parser.add_argument("--insertion-reserve-m", type=float, default=0.016)
    args = parser.parse_args(argv)
    fleet = _fleet_tool()
    seeds = fleet._seeds(args.seeds)
    geometry = fleet.mission_geometry(args)
    for path in FONTS:
        if Path(path).exists():
            font_manager.fontManager.addfont(path)
            plt.rcParams["font.family"] = font_manager.FontProperties(fname=path).get_name()
            break
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    fig, axes = plt.subplots(
        len(STRUCTURES), len(seeds), figsize=(5.2 * len(seeds), 4.2 * len(STRUCTURES))
    )
    fig.patch.set_facecolor("#14171d")
    for i, structure in enumerate(STRUCTURES):
        for j, seed in enumerate(seeds):
            ax = axes[i][j] if len(seeds) > 1 else axes[i]
            try:
                world = make_world(structure, seed, geometry=geometry, trucks=args.trucks)
            except ValueError as error:
                ax.set_title(f"{structure} {seed}: {error}", fontsize=7, color="red")
                rows.append({"structure": structure, "seed": seed, "error": str(error)})
                continue
            draw(ax, world, geometry)
            single = plt.figure(figsize=(10, 8))
            single.patch.set_facecolor("#14171d")
            draw(single.add_subplot(111), world, geometry)
            single.savefig(args.output / f"{structure}_{seed}.png", dpi=130, bbox_inches="tight")
            plt.close(single)
            rows.append(
                {
                    "structure": structure,
                    "seed": seed,
                    "size_m": world.size_m,
                    "sources": len(world.sources()),
                    "sinks": len(world.sinks()),
                    "holding": len(world.holding),
                    "obstacles": len(world.obstacles),
                    "dropped": world.dropped,
                }
            )
    fig.tight_layout()
    fig.savefig(args.output / "contact_sheet.png", dpi=110)
    (args.output / "worlds.json").write_text(json.dumps(rows, indent=1, ensure_ascii=False) + "\n")
    for row in rows:
        print(json.dumps(row, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
