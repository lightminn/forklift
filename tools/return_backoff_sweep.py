"""The return-start backoff against the neighbour estimate's offset (docs/plans/2026-10-27-return-start-withdraw-
extension.md C3; CPU).

From a recorded mission scenario (``state.mission_scenario`` of a session mission whose destination has a delivered
neighbour), the neighbour's ``@search`` rectangle is moved off its ``@slot`` toward the destination by dy (and dx, dyaw)
and the return is planned from the withdrawal end as the mission plan's return_home stage plans it, with the backoff
off and on. Each row: whether off and on succeed, the backoff record, and whether on equals off exactly where off
succeeded (the backoff never touches a valid start).

    python tools/return_backoff_sweep.py SCENARIO.json [...] --neighbour P-5 --out RESULT.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / "tools", ROOT / "sim/isaac", ROOT):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from bay_planning_sweep import mission_geometry  # noqa: E402
from riibotics_scenario_paths import PALLETS  # noqa: E402
from scene_file import scenario_from_record  # noqa: E402

from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.planning.pallet_mission import (  # noqa: E402
    D7_PLANNER_OPTIONS,
    RETURN_START_BACKOFF_CAP_M,
    SEARCH_TAG,
    SLOT_TAG,
    PlacedProp,
    make_transport_planner_config,
    plan_return_leg,
    site_poses,
)

# the run's planning inputs (tools/session_plan_check.py, the riib/CT-1 sessions)
URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"
STRAIGHTS = {"alignment_straight_m": 2.1, "delivery_straight_m": 1.5, "withdrawal_m": 0.55}
RESERVE_M, KAPPA, CLEARANCE_M, MAX_EXPANSIONS = 0.016, 0.29, 0.10, 30000
DY = [round(0.01 * k, 2) for k in range(36)]
DX = (-0.10, 0.0, 0.05)
DYAW = (-0.015, 0.0, 0.015)


def with_estimate(scenario, neighbour: str, dx: float, dy: float, dyaw: float):
    """``scenario`` with ``neighbour``'s @search at its @slot moved by (dx, dy toward the destination, dyaw)."""
    by_uri = {p.asset.uri: p for p in scenario.props}
    slot = by_uri[f"session:pallet:{neighbour}{SLOT_TAG}"].rectangle
    toward = math.copysign(1.0, scenario.destination.y_m - slot.y_m)
    moved = replace(slot, x_m=slot.x_m + dx, y_m=slot.y_m + toward * dy, yaw_rad=slot.yaw_rad + dyaw)
    search = by_uri[f"session:pallet:{neighbour}{SEARCH_TAG}"]
    props = tuple(PlacedProp(p.asset, moved) if p is search else p for p in scenario.props)
    return replace(scenario, props=props)


def same(a, b) -> bool:
    return all(np.array_equal(getattr(a, k), getattr(b, k)) for k in ("poses", "directions", "curvatures_inv_m"))


def sweep(record: dict, neighbour: str, geometry, config) -> list:
    base = scenario_from_record(record)
    start = site_poses(base.destination, geometry)["withdrawn"]
    rows = []
    for dyaw in DYAW:
        for dx in DX:
            for dy in DY:
                sc = with_estimate(base, neighbour, dx, dy, dyaw)
                off = plan_return_leg(sc, start, sc.start_rear, config, geometry=geometry)
                on = plan_return_leg(sc, start, sc.start_rear, config, geometry=geometry,
                                     return_start_backoff_m=RETURN_START_BACKOFF_CAP_M)
                rec = on.start_backoff or {}
                rows.append({"dx": dx, "dy": dy, "dyaw": dyaw, "off": off.status, "on": on.status,
                             "backoff": rec.get("status"), "advice_m": rec.get("advice_m"),
                             "length_m": rec.get("length_m"),
                             "on_equals_off": bool(off.success and on.success and same(off, on))})
    return rows


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scenarios", nargs="+", type=Path, help="JSON files holding a mission_scenario record")
    parser.add_argument("--neighbour", required=True, action="append", help="the neighbour pallet id, one per file")
    parser.add_argument("--pallet", default="epal6")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if len(args.neighbour) != len(args.scenarios):
        parser.error("one --neighbour per scenario file")
    geometry = mission_geometry(URDF, load_pallet_geometry(PALLETS[args.pallet]), RESERVE_M, **STRAIGHTS)
    config = make_transport_planner_config(curvature_limit_inv_m=KAPPA, clearance_m=CLEARANCE_M,
                                           max_expansions=MAX_EXPANSIONS, **D7_PLANNER_OPTIONS)
    out = []
    for path, neighbour in zip(args.scenarios, args.neighbour):
        record = json.loads(path.read_text())
        record = record.get("mission_scenario", record)
        out.append({"scenario": str(path), "neighbour": neighbour, "rows": sweep(record, neighbour, geometry, config)})
    args.out.write_text(json.dumps(out, indent=1) + "\n")
    for entry in out:
        rows = entry["rows"]
        blocked = [r for r in rows if r["off"] == "invalid_start"]
        fixed = [r for r in blocked if r["on"] == "success"]
        print(entry["scenario"], entry["neighbour"], "rows", len(rows), "off invalid_start", len(blocked),
              "on fixes", len(fixed), "valid-start rows unchanged",
              sum(r["on_equals_off"] for r in rows if r["off"] == "success"), "/",
              sum(1 for r in rows if r["off"] == "success"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
