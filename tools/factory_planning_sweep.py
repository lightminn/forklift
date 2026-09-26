"""Plan every requested seed on the factory hall floor, on the CPU.

For each seed: the transport mission (pickup-side stages inside the original
bay, transport and return legs across the hall with the obstacle heuristic)
and the SLAM survey loop. Asset sizes are the offline usd-core measurement in
sim/isaac/factory_assets.py; an Isaac run re-measures its own. Failures are
results: no seed is replaced or dropped.

    python tools/factory_planning_sweep.py --seeds 0-19 \
        --output artifacts/<UTC>_factory_planning
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import time
from dataclasses import replace
from pathlib import Path

import yaml

from forklift_core.planning.factory_layout import (
    load_factory_layout,
    make_factory_scenario,
    plan_survey_route,
)
from forklift_core.planning.pallet_mission import (
    SyntheticMissionGeometry,
    make_transport_planner_config,
    plan_transport,
)

ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props"
)
BAY_ASSETS = (
    "SM_BarelPlastic_A_01.usd",
    "SM_CratePlastic_D_01.usd",
    "SM_CardBoxA_02.usd",
)


def _factory_assets_module():
    path = ROOT / "sim/isaac/factory_assets.py"
    spec = importlib.util.spec_from_file_location("factory_assets", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _seeds(text: str) -> list[int]:
    seeds = []
    for part in text.split(","):
        first, _, last = part.partition("-")
        seeds.extend(range(int(first), int(last or first) + 1))
    return seeds


def planner_configs(settings: dict, heuristic_resolution_m: float, expansions: int):
    """Mission config as run_transport builds it, and the travel-leg variant."""
    config = make_transport_planner_config(
        curvature_limit_inv_m=settings["planner_curvature_inv_m"],
        clearance_m=settings["planning_clearance_m"],
        max_expansions=expansions,
    )
    return config, replace(
        config, obstacle_heuristic_resolution_m=heuristic_resolution_m
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=_seeds, required=True, help="e.g. 0-19")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--layout", type=Path, default=ROOT / "config/factory_south_hall.yaml"
    )
    parser.add_argument(
        "--settings", type=Path, default=ROOT / "config/isaac_transport.yaml"
    )
    parser.add_argument("--heuristic-resolution-m", type=float, default=0.25)
    parser.add_argument("--max-expansions", type=int, default=30000)
    parser.add_argument("--no-return", action="store_true")
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    assets = _factory_assets_module()
    specs = assets.offline_specs(ASSET_ROOT)
    bay_assets = [specs[name] for name in BAY_ASSETS]
    layout = load_factory_layout(args.layout)
    settings = yaml.safe_load(args.settings.read_text(encoding="utf-8"))
    config, travel = planner_configs(
        settings, args.heuristic_resolution_m, args.max_expansions
    )
    geometry = SyntheticMissionGeometry()
    records = []
    for seed in args.seeds:
        factory = make_factory_scenario(
            seed, layout, bay_assets, assets.factory_assets(specs), geometry=geometry
        )
        started = time.monotonic()
        plan = plan_transport(
            factory.transport,
            config,
            geometry=geometry,
            pickup_bounds=factory.pickup_bounds,
            travel_config=travel,
            return_to=None if args.no_return else factory.transport.start_rear,
        )
        mission_s = time.monotonic() - started
        started = time.monotonic()
        survey = plan_survey_route(
            factory, layout.survey_route, travel, geometry=geometry
        )
        survey_s = time.monotonic() - started
        record = {
            "seed": seed,
            "work_items": len(factory.work_items),
            "loads": len(factory.loads),
            "destination": [
                factory.transport.destination.x_m,
                factory.transport.destination.y_m,
                factory.transport.destination.yaw_rad,
            ],
            "mission_status": plan.status,
            "mission_wall_s": round(mission_s, 2),
            "legs": {
                name: {"length_m": leg.length_m, "expansions": leg.expanded_nodes}
                for name in (
                    "approach",
                    "insert",
                    "extract",
                    "transport",
                    "withdraw",
                    "return_home",
                )
                if (leg := getattr(plan, name)) is not None
            },
            "survey_status": survey.status,
            "survey_length_m": survey.length_m,
            "survey_expansions": survey.expanded_nodes,
            "survey_wall_s": round(survey_s, 2),
        }
        records.append(record)
        print(
            f"seed {seed:3d}  mission {plan.status:28s} {mission_s:6.1f}s   "
            f"survey {survey.status:22s} {survey.length_m:6.1f} m {survey_s:6.1f}s",
            flush=True,
        )
    summary = {
        "layout": str(args.layout),
        "layout_version": layout.layout_version,
        "settings": str(args.settings),
        "planner": {
            "max_expansions": args.max_expansions,
            "travel_obstacle_heuristic_resolution_m": args.heuristic_resolution_m,
            "return_home": not args.no_return,
        },
        "asset_dimensions": "offline usd-core measurement, sim/isaac/factory_assets.py",
        "seeds": args.seeds,
        "mission_success": sum(r["mission_status"] == "success" for r in records),
        "survey_success": sum(r["survey_status"] == "success" for r in records),
        "records": records,
    }
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    print(
        f"mission {summary['mission_success']}/{len(records)}  "
        f"survey {summary['survey_success']}/{len(records)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
