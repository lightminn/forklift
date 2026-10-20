"""Plan the Riibotics demo scenarios' narrow aisle and trailer on the CPU (planned paths only).

Riibotics sent five demo scenarios on 2026-10-19 (local reference, not in the repository).
Two are bounded by space: (1) a narrow aisle -- 3.8 m clear between the rack's pallet faces
and the blocks below, 3.4 m between those blocks -- where the truck picks a floor pallet from
the rack row and puts it in another rack slot, and (5) a trailer 2.6 m wide inside, walled
except for its entrance, holding two columns of pallets. The environment is scaled by our
truck over their 2.8 x 1.13 m forklift: 0.521 by length, 0.558 by width; both are planned.

Each case is a TransportScenario (racks, blocks, walls and neighbouring pallets as props, the
outer walls as Bounds) planned with plan_transport and run_transport's settings for the
measured chassis, back to the start pose (so a trailer case also has to leave the trailer).
Numbers the scenario does not give are assumptions, swept where they decide the outcome:
the gap between neighbouring floor pallets, the clearance behind the rack row, and the
alignment straight. The footprint the planner uses (half-width 0.36 m) is wider than the
measured body (0.315 m) and its steered tyres (0.336 m); a case planned with 0.336 m is a
control and is kept apart from the standard result.

A failure is classified by its stage and status: a goal pose in collision (invalid_goal or
invalid_start), a fixed straight in collision (straight_collision), or a search that found
nothing within its retries (no_path, expansion_limit, timeout) -- the last does not prove
that no path exists. Gear changes are counted over the whole mission in driving order,
stage boundaries included. A planned path is not a driven one: tracking, perception and
contact are not checked here.

    python tools/riibotics_scenario_paths.py --output artifacts/<run> [--jobs 8] [--timeout-s 900]
    python tools/riibotics_scenario_paths.py --output artifacts/<run> --dry   # scenes only
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import multiprocessing
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from bay_planning_sweep import mission_geometry  # noqa: E402

from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.planning import Footprint, Pose2D  # noqa: E402
from forklift_core.planning.geometry import Bounds, Rectangle  # noqa: E402
from forklift_core.planning.pallet_mission import (  # noqa: E402
    AssetSpec,
    PalletSite,
    PlacedProp,
    TransportScenario,
    make_transport_planner_config,
    plan_transport,
    site_poses,
)

URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"
PALLETS = {"epal6": ROOT / "config/pallet_geometry_epal6.yaml",
           "t11": ROOT / "config/pallet_geometry_t11_06.yaml"}
SCALES = {"length": 1.46 / 2.8, "width": 0.63 / 1.13}
RESERVE_M = 0.016
UP = math.pi / 2
STAGES = ("approach", "insert", "extract", "transport", "withdraw", "return_home")

# Scenario numbers (Riibotics 2026-10-19, version 2), real size
AISLE_M, BLOCK_GAP_M, TRAILER_M = 3.8, 3.4, 2.6
# Assumptions, real size (scaled) unless noted -- the drawing is not to scale
BLOCK_WIDTH_M = 3.4          # blocks drawn about as wide as the gaps between them
BLOCK_DEPTH_M = 5.0          # how far the blocks, and the gaps between them, run below the aisle
START_BEFORE_RACK_M = 3.0    # the truck enters the aisle this far before slot 0's left edge
SLOTS, FILLED = 12, 6        # rack slots; the first FILLED hold pallets
TRAILER_GAP_M = 0.10         # between pallet rows in the trailer, and before the back wall (scaled world)
STEERED_TYRE_HALF_WIDTH_M = 0.336   # measured-chassis tyre box at 15 deg (Codex 2026-10-20), control only


def prop(name: str, x: float, y: float, length: float, width: float, yaw: float = 0.0, h: float = 1.0):
    return PlacedProp(AssetSpec(f"riibotics:{name}", length, width, h), Rectangle(x, y, length, width, yaw))


def pallet_prop(name: str, x: float, y: float, pallet, yaw: float):
    """A pallet as an obstacle: its depth runs along the insertion heading."""
    return prop(name, x, y, pallet.overall_depth_m, pallet.overall_width_m, yaw, pallet.overall_height_m)


def aisle_case(scale: float, pallet, pick_over: str, drop_over: str, slot_gap: float, back_gap: float):
    """Pick from rack slot 1, put into the first empty slot over a block (or a gap).

    x runs along the aisle, y up towards the rack. The blocks and the gaps between them run
    BLOCK_DEPTH_M below the aisle; the aisle ends half a slot past the last slot (no way
    round the rack) and starts START_BEFORE_RACK_M plus 1.2 m before slot 0 (the entrance).
    """
    aisle = AISLE_M * scale
    width, depth = pallet.overall_width_m, pallet.overall_depth_m
    pitch = width + slot_gap
    rack_y = aisle / 2 + depth / 2
    slot_x = [slot_gap / 2 + pitch * (i + 0.5) for i in range(SLOTS)]
    block, gap = BLOCK_WIDTH_M * scale, BLOCK_GAP_M * scale
    period = block + gap
    start_x = -START_BEFORE_RACK_M * scale
    x_lo, x_hi = start_x - 1.2, slot_x[-1] + pitch / 2 + 0.5 * pitch
    # Shift the block pattern so slot 1 is over a block centre or a gap centre.
    centre = slot_x[1] if pick_over == "block" else slot_x[1] - period / 2
    first = centre - period * math.ceil((centre - x_lo) / period)
    blocks, x = [], first
    while x - block / 2 < x_hi:
        blocks.append(x)
        x += period

    def over(xc: float) -> str:
        return "block" if any(abs(xc - b) <= block / 2 for b in blocks) else "gap"

    empty = [i for i in range(FILLED, SLOTS) if over(slot_x[i]) == drop_over]
    if not empty:
        raise ValueError(f"no empty slot over a {drop_over}")
    drop = empty[0]
    props = [pallet_prop(f"rack_pallet_{i}", slot_x[i], rack_y, pallet, UP) for i in range(FILLED) if i != 1]
    y_low = -aisle / 2 - BLOCK_DEPTH_M * scale
    for b in blocks:
        lo, hi = max(b - block / 2, x_lo), min(b + block / 2, x_hi)
        if hi > lo:
            props.append(prop("block", (lo + hi) / 2, (-aisle / 2 + y_low) / 2, hi - lo, -aisle / 2 - y_low))
    bounds = Bounds(x_lo, x_hi, y_low, rack_y + depth / 2 + back_gap)
    scenario = TransportScenario(
        seed=0,
        start_rear=Pose2D(start_x, 0.0, 0.0),
        pickup=PalletSite(slot_x[1], rack_y, UP),
        destination=PalletSite(slot_x[drop], rack_y, UP),
        props=tuple(props),
        bounds=bounds,
    )
    return scenario, {"aisle_m": aisle, "pitch_m": pitch, "slot_gap_m": slot_gap, "back_gap_m": back_gap,
                      "pick_slot": 1, "drop_slot": drop, "pick_over": over(slot_x[1]),
                      "drop_over": over(slot_x[drop]), "blocks_x_m": blocks, "block_m": block,
                      "gap_m": gap, "bounds": asdict(bounds)}


# Trailer occupancy, columns (left, right) x rows (back, middle, front): 1 holds a pallet,
# T the target, 0 empty. The truck starts outside, the yard pallet site is outside.
TRAILER_CASES = {
    "a_front":   ("pick", ((1, 1), (1, 1), ("T", 1))),
    "b_middle":  ("pick", ((1, 1), ("T", 1), (0, 1))),
    "c_back":    ("pick", (("T", 1), (0, 1), (0, 1))),
    "d_putback": ("put",  (("T", 1), (0, 1), (0, 1))),
    "e_blocked": ("pick", (("T", 1), (1, 1), (0, 1))),   # negative control: back-left behind a pallet
}


def trailer_case(scale: float, pallet, case: str):
    """Forks enter along +y; outside the trailer at y >= 0 is the dock wall, the yard below is open."""
    inner = TRAILER_M * scale
    width, depth = pallet.overall_width_m, pallet.overall_depth_m
    columns = 2 if 2 * width < inner else 1
    clear = (inner - columns * width) / (columns + 1)
    col_x = [-(width + clear) / 2, (width + clear) / 2] if columns == 2 else [0.0]
    length = 3 * (depth + TRAILER_GAP_M) + TRAILER_GAP_M
    row_y = [length - TRAILER_GAP_M - depth / 2 - r * (depth + TRAILER_GAP_M) for r in range(3)]
    kind, grid = TRAILER_CASES[case]
    props, target = [], None
    for r, row in enumerate(grid):
        for c, cell in enumerate(row[:columns]):
            if cell == "T":
                target = (c, r)
            elif cell == 1:
                props.append(pallet_prop(f"trailer_pallet_{c}{r}", col_x[c], row_y[r], pallet, UP))
    side = 4.0
    for sign in (-1, 1):   # trailer wall and the dock face beside it, one block each side
        x_in, x_out = sign * inner / 2, sign * side
        props.append(prop("trailer_wall", (x_in + x_out) / 2, length / 2, abs(x_out - x_in), length))
    site = PalletSite(col_x[target[0]], row_y[target[1]], UP)
    yard = PalletSite(2.6, -4.2, 0.0)
    pickup, destination = (site, yard) if kind == "pick" else (yard, site)
    bounds = Bounds(-side, side, -6.5, length)   # the back wall is the trailer's end
    scenario = TransportScenario(
        seed=0,
        start_rear=Pose2D(0.0, -3.0, UP),
        pickup=pickup,
        destination=destination,
        props=tuple(props),
        bounds=bounds,
    )
    return scenario, {"inner_m": inner, "columns": columns, "side_clear_m": clear, "length_m": length,
                      "column_x_m": col_x, "row_y_m": row_y, "grid": grid, "kind": kind,
                      "target": target, "bounds": asdict(bounds)}


def cases(only: str | None) -> list[dict]:
    out = []
    for scale in SCALES:
        for pallet in PALLETS:
            base = {"scale": scale, "pallet": pallet, "kappa": 0.29, "align_m": 0.80, "footprint": "planner"}
            if only in (None, "aisle"):
                for pick in ("block", "gap"):
                    for drop in ("block", "gap"):
                        for slot_gap in (0.10, 0.20, 0.30):
                            # the standard settings, then a shorter alignment straight, then that with tracker kappa
                            for align, kappa in ((0.80, 0.29), (0.30, 0.29), (0.30, 0.36)):
                                out.append({**base, "scene": "aisle", "pick_over": pick, "drop_over": drop,
                                            "slot_gap_m": slot_gap, "back_gap_m": 0.15,
                                            "align_m": align, "kappa": kappa})
            if only in (None, "trailer"):
                for case in TRAILER_CASES:
                    for kappa, footprint in ((0.29, "planner"), (0.36, "planner"), (0.29, "tyres")):
                        out.append({**base, "scene": "trailer", "case": case, "kappa": kappa,
                                    "footprint": footprint})
    keys = ("pick_over", "drop_over", "slot_gap_m", "case", "kappa", "align_m", "footprint")
    for c in out:
        c["name"] = f"{c['scene']}_{c['scale']}_{c['pallet']}_" + "_".join(f"{k}{c[k]}" for k in keys if k in c)
    return out


def build(case: dict):
    pallet = load_pallet_geometry(PALLETS[case["pallet"]])
    scale = SCALES[case["scale"]]
    geometry = mission_geometry(URDF, pallet, RESERVE_M, alignment_straight_m=case["align_m"],
                                delivery_straight_m=0.70, withdrawal_m=0.55)
    if case["footprint"] == "tyres":
        fp = geometry.unloaded_footprint
        geometry = replace(geometry, unloaded_footprint=Footprint(fp.front_m, fp.rear_m, STEERED_TYRE_HALF_WIDTH_M))
    if case["scene"] == "aisle":
        scenario, info = aisle_case(scale, pallet, case["pick_over"], case["drop_over"],
                                    case["slot_gap_m"], case["back_gap_m"])
    else:
        scenario, info = trailer_case(scale, pallet, case["case"])
    return scenario, info, geometry


def classify(status: str) -> str:
    detail = status.split(":")[-1]
    if detail in ("invalid_goal", "invalid_start"):
        return "goal_in_collision"
    if detail == "straight_collision":
        return "straight_in_collision"
    if detail in ("no_path", "expansion_limit", "timeout"):
        return "search_found_nothing"
    return "success" if status == "success" else "other"


def gear_changes(plan) -> int:
    """Direction changes over every driven segment in mission order, stage boundaries included."""
    seq = []
    for name in STAGES:
        stage = getattr(plan, name, None)
        if stage is not None and stage.success:
            seq.extend(int(v) for v in stage.directions[1:])
    return sum(1 for a, b in zip(seq, seq[1:]) if a != b)


def _stage(plan) -> dict | None:
    if plan is None:
        return None
    d = [int(v) for v in plan.directions[1:]]
    return {"success": bool(plan.success), "status": plan.status, "length_m": round(float(plan.length_m), 3),
            "gear_changes_inside": sum(1 for a, b in zip(d, d[1:]) if a != b),
            "poses": [[round(float(v), 4) for v in p] for p in plan.poses],
            "directions": [int(v) for v in plan.directions]}


def scene_record(case: dict) -> dict:
    scenario, info, geometry = build(case)
    goals = {f"{side}_{n}": asdict(p) for side, site in (("pickup", scenario.pickup), ("drop", scenario.destination))
             for n, p in site_poses(site, geometry).items()}
    return {**case, "info": info, "scenario": asdict(scenario), "geometry": asdict(geometry), "goals": goals}


def run_case(case: dict, path: Path) -> None:
    try:
        record = scene_record(case)
    except ValueError as exc:
        path.write_text(json.dumps({**case, "outcome": "scene_failure", "reason": str(exc)}, indent=1))
        return
    path.with_suffix(".scene.json").write_text(json.dumps(record, indent=1, default=str))
    scenario, _, geometry = build(case)
    config = make_transport_planner_config(curvature_limit_inv_m=case["kappa"], clearance_m=0.10,
                                           max_expansions=30000)
    trace: list = []
    started = time.monotonic()
    plan = plan_transport(scenario, config, geometry=geometry, return_to=scenario.start_rear, trace=trace)
    record.update(outcome=classify(plan.status), status=plan.status, wall_s=round(time.monotonic() - started, 2),
                  gear_changes=gear_changes(plan) if plan.success else None, trace=trace,
                  stages={name: _stage(getattr(plan, name, None)) for name in STAGES})
    path.write_text(json.dumps(record, indent=1, default=str))


def _worker(args):
    case, out, timeout = args
    path = out / f"{case['name']}.json"
    if path.exists():
        return case["name"], json.loads(path.read_text()).get("outcome")
    proc = multiprocessing.get_context("spawn").Process(target=run_case, args=(case, path))
    proc.start()
    proc.join(timeout)
    if proc.is_alive():
        proc.kill()
        proc.join()
        path.write_text(json.dumps({**case, "outcome": "search_found_nothing", "status": "wall_timeout",
                                    "timeout_s": timeout}, indent=1))
    elif not path.exists():
        path.write_text(json.dumps({**case, "outcome": "crash", "exitcode": proc.exitcode}, indent=1))
    return case["name"], json.loads(path.read_text()).get("outcome")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--timeout-s", type=float, default=900.0)
    parser.add_argument("--only", choices=("aisle", "trailer"))
    parser.add_argument("--dry", action="store_true", help="write the scenes and goal poses, plan nothing")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    todo = cases(args.only)
    (args.output / "cases.json").write_text(json.dumps(todo, indent=1))
    if args.dry:
        for case in todo:
            (args.output / f"{case['name']}.scene.json").write_text(json.dumps(scene_record(case), indent=1, default=str))
        print(f"wrote {len(todo)} scenes")
        return
    # Threads only wait on the per-case processes (a pool worker may not start its own).
    with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
        for name, outcome in pool.map(_worker, [(c, args.output, args.timeout_s) for c in todo]):
            print(f"{outcome:22s} {name}", flush=True)


if __name__ == "__main__":
    main()
