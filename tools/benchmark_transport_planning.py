"""Run every requested synthetic transport seed using measured warehouse bounds.

This portable launcher needs an installed forklift-core package, not a checkout
or Git. Planning failures are results, not grounds for replacing requested seeds.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import os
import platform
import sys
import time
import traceback
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.planning import Footprint
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    SyntheticMissionGeometry,
    make_scenario,
    make_transport_planner_config,
    plan_transport,
)

ASSET_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props"
)
ASSET_NAMES = (
    "SM_BarelPlastic_A_01.usd",
    "SM_CratePlastic_D_01.usd",
    "SM_CardBoxA_02.usd",
)


def _atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def _assets(path: Path) -> tuple[list[AssetSpec], bytes]:
    raw = path.read_bytes()
    records = json.loads(raw)
    if not isinstance(records, list):
        raise ValueError("asset evidence must be a list")
    by_name = {}
    for record in records:
        name = record["filename"]
        if name not in ASSET_NAMES or name in by_name:
            raise ValueError(f"unexpected or duplicate official asset: {name}")
        low, high = np.asarray(record["min"], float), np.asarray(record["max"], float)
        if (
            low.shape != (3,)
            or high.shape != (3,)
            or not np.isfinite([low, high]).all()
        ):
            raise ValueError(f"invalid measured bounds: {name}")
        by_name[name] = AssetSpec(ASSET_ROOT + "/" + name, *map(float, high - low))
    if set(by_name) != set(ASSET_NAMES):
        raise ValueError("all three official warehouse asset records are required")
    return [by_name[name] for name in ASSET_NAMES], raw


def main(argv: list[str] | None = None) -> int:
    """Save incremental evidence for all seeds; exit zero when the batch finishes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets-json", type=Path, required=True)
    parser.add_argument("--pallet-geometry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--start-seed", type=int, default=0)
    parser.add_argument("--obstacles", type=int, default=4)
    args = parser.parse_args(argv)
    if args.count <= 0 or args.start_seed < 0 or args.obstacles < 0:
        parser.error("count must be positive; start-seed and obstacles nonnegative")
    assets, raw_assets = _assets(args.assets_json)
    pallet_geometry = load_pallet_geometry(args.pallet_geometry)
    raw_pallet_geometry = args.pallet_geometry.read_bytes()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "assets.json").write_bytes(raw_assets)
    (args.output / "pallet_geometry.yaml").write_bytes(raw_pallet_geometry)
    config = make_transport_planner_config(
        curvature_limit_inv_m=0.5,
        max_expansions=30000,
    )
    geometry = SyntheticMissionGeometry(
        unloaded_footprint=Footprint(1.29, 0.17, 0.36),
        pallet_depth_m=pallet_geometry.overall_depth_m,
        pallet_width_m=pallet_geometry.overall_width_m,
    )
    sources = {}
    for name in (
        "forklift_core.planning.geometry",
        "forklift_core.planning.hybrid_astar",
        "forklift_core.planning.pallet_mission",
    ):
        module = importlib.import_module(name)
        path = Path(module.__file__)
        sources[name] = {
            "path": str(path),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    requested = list(range(args.start_seed, args.start_seed + args.count))
    report = {
        "benchmark_complete": False,
        "validation_kind": "synthetic geometric planning; not physical execution",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "python": sys.version,
        "platform": platform.platform(),
        "numpy": np.__version__,
        "launcher_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "installed_planning_sources": sources,
        "pallet_geometry_evidence": {
            "input_path": str(args.pallet_geometry),
            "sha256": hashlib.sha256(raw_pallet_geometry).hexdigest(),
            "copied_to": "pallet_geometry.yaml",
        },
        "asset_evidence": {
            "input_path": str(args.assets_json),
            "sha256": hashlib.sha256(raw_assets).hexdigest(),
            "copied_to": "assets.json",
        },
        "assets": [asdict(asset) for asset in assets],
        "planner_config": asdict(config),
        "approach_clearance_m": min(config.clearance_m, geometry.approach_gap_m / 2),
        "mission_geometry": asdict(geometry),
        "obstacles": args.obstacles,
        "requested_seeds": requested,
        "results": [],
    }
    started = time.monotonic()

    def checkpoint() -> None:
        rows = report["results"]
        finished = {row["seed"] for row in rows}
        progress = {
            "benchmark_complete": report["benchmark_complete"],
            "requested_count": len(requested),
            "completed_count": len(rows),
            "success_count": sum(row["success"] for row in rows),
            "failure_count": sum(not row["success"] for row in rows),
            "missing_seeds": [seed for seed in requested if seed not in finished],
            "elapsed_s": time.monotonic() - started,
        }
        report["progress"] = progress
        _atomic_json(args.output / "results.json", report)
        _atomic_json(args.output / "progress.json", progress)

    checkpoint()
    for seed in requested:
        seed_started = time.monotonic()
        row = {"seed": seed, "success": False}
        try:
            scenario = make_scenario(seed, assets, args.obstacles, geometry=geometry)
            row["scenario"] = asdict(scenario)
            result = plan_transport(scenario, config, geometry=geometry)
            row.update(success=result.success, status=result.status)
            if result.success:
                row["paths"] = {
                    name: {
                        "length_m": getattr(result, name).length_m,
                        "expanded_nodes": getattr(result, name).expanded_nodes,
                        "pose_count": len(getattr(result, name).poses),
                    }
                    for name in (
                        "approach",
                        "insert",
                        "extract",
                        "transport",
                        "withdraw",
                    )
                }
        except Exception as exc:
            row.update(
                status="exception",
                exception_type=type(exc).__name__,
                reason=str(exc),
                traceback=traceback.format_exc(),
            )
        row["elapsed_s"] = time.monotonic() - seed_started
        report["results"].append(row)
        checkpoint()
        print(
            json.dumps(
                {key: row[key] for key in ("seed", "success", "status", "elapsed_s")}
            ),
            flush=True,
        )
    report["benchmark_complete"] = True
    checkpoint()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
