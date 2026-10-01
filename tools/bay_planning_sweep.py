"""Plan every requested seed in the original transport bay, on the CPU.

Plan R1 of docs/plans/2026-10-01-measured-chassis-revalidation.md: the same
planning `run_transport.py --layout bay` does -- the three bay assets at their
offline full-precision sizes, mission planner config from the settings file,
no factory travel heuristic -- for one chassis model and insertion reserve.
Each seed runs in its own process under a wall-clock limit, writes its result
as soon as it finishes and is skipped on a rerun whose inputs hash the same.
Every planning step is appended to ``seed_N.trace.jsonl`` as it completes, so
a timed-out seed still shows how far it got. Failures are results: no seed is
replaced or dropped.

    python tools/bay_planning_sweep.py --seeds 0-24 \\
        --settings config/isaac_transport_measured.yaml \\
        --forklift-urdf sim/models/dls08_measured/forklift.urdf \\
        --insertion-reserve-m 0.016 --return-home --output artifacts/<run>
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import multiprocessing
import os
import sys
import time
from dataclasses import asdict
from pathlib import Path

import yaml

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.planning import Footprint
from forklift_core.planning.pallet_mission import (
    SyntheticMissionGeometry,
    make_scenario,
    make_transport_planner_config,
    plan_transport,
)

ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props"
)
# The bay's three props, in the order run_transport's catalogue uses.
BAY_ASSETS = (
    "SM_BarelPlastic_A_01.usd",
    "SM_CratePlastic_D_01.usd",
    "SM_CardBoxA_02.usd",
)
PROVISIONAL_URDF = ROOT / "sim/models/dls08_provisional/forklift.urdf"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _seeds(text: str) -> list[int]:
    seeds = []
    for part in text.split(","):
        first, _, last = part.partition("-")
        seeds.extend(range(int(first), int(last or first) + 1))
    return seeds


def mission_geometry(
    forklift_urdf: Path, pallet_geometry, reserve_m: float
) -> SyntheticMissionGeometry:
    """The geometry run_transport builds from its URDF and pallet geometry."""
    sys.path.insert(0, str(ROOT / "sim/isaac"))
    insertion = _load("insertion_geometry", ROOT / "sim/isaac/insertion_geometry.py")
    axle_to_tip, _ = insertion.read_chassis_reference_m(forklift_urdf)
    return SyntheticMissionGeometry(
        unloaded_footprint=Footprint(axle_to_tip, 0.17, 0.36),
        pallet_depth_m=pallet_geometry.overall_depth_m,
        pallet_width_m=pallet_geometry.overall_width_m,
        axle_to_fork_tip_m=axle_to_tip,
        carriage_limit_m=insertion.read_carriage_limit_m(forklift_urdf),
        insertion_reserve_m=reserve_m,
    )


class _TraceFile(list):
    """A list that also appends each entry to a JSON-lines file at once."""

    def __init__(self, path: Path):
        super().__init__()
        self.path = path
        path.write_text("")

    def append(self, entry) -> None:
        super().append(entry)
        with self.path.open("a") as handle:
            handle.write(json.dumps(entry) + "\n")


def _write_atomic(path: Path, record: dict) -> None:
    """Write JSON next to its target and rename, so a reader never sees half."""
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(record, indent=2))
    os.replace(tmp, path)


def _read(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def _plan_one(job: dict) -> None:
    """Child process: plan one seed into this attempt's own files."""
    assets = _load("factory_assets", ROOT / "sim/isaac/factory_assets.py")
    specs = assets.offline_specs(ASSET_ROOT)
    catalogue = [specs[name] for name in BAY_ASSETS]
    pallet_geometry = load_pallet_geometry(Path(job["pallet_geometry"]))
    geometry = mission_geometry(
        Path(job["forklift_urdf"]), pallet_geometry, job["reserve_m"]
    )
    scene_geometry = mission_geometry(
        Path(job["scene_urdf"]), pallet_geometry, job["scene_reserve_m"]
    )
    attempt = Path(job["attempt_stem"])
    record = {"seed": job["seed"], "input_sha256": job["input_sha256"]}
    try:
        scenario = make_scenario(
            job["seed"], catalogue, job["obstacles"], geometry=scene_geometry
        )
    except ValueError as exc:
        record.update(outcome="scene_failure", reason=str(exc))
        _write_atomic(attempt.with_suffix(".json"), record)
        return
    scene = json.dumps(asdict(scenario), sort_keys=True)
    record["scenario_sha256"] = hashlib.sha256(scene.encode()).hexdigest()
    record["geometry"] = asdict(geometry)
    # Saved before the search, so a timed-out seed still has its scene.
    _write_atomic(attempt.with_suffix(".scene.json"), record)
    config = make_transport_planner_config(
        curvature_limit_inv_m=job["curvature"],
        clearance_m=job["clearance"],
        max_expansions=job["max_expansions"],
    )
    trace = _TraceFile(attempt.with_suffix(".trace.jsonl"))
    started = time.monotonic()
    plan = plan_transport(
        scenario,
        config,
        geometry=geometry,
        return_to=scenario.start_rear if job["return_home"] else None,
        trace=trace,
    )
    record.update(
        outcome="success" if plan.success else "planning_failure",
        status=plan.status,
        wall_s=round(time.monotonic() - started, 3),
        trace=list(trace),
    )
    _write_atomic(attempt.with_suffix(".json"), record)


def run_seed(job: dict, timeout_s: float, final: Path) -> dict:
    """Run one seed in a child process and write ``final``.

    The child writes only to fresh attempt files; the parent accepts its
    result only on exit code 0 with this run's input hash. Timeouts and
    crashes are outcomes and keep whatever scene record the child saved.
    """
    # No dot in the stem: with_suffix would otherwise replace it and collide.
    attempt = final.with_name(final.stem + "_attempt")
    for suffix in (".json", ".scene.json", ".trace.jsonl"):
        attempt.with_suffix(suffix).unlink(missing_ok=True)
    context = multiprocessing.get_context("spawn")
    process = context.Process(
        target=_plan_one, args=({**job, "attempt_stem": str(attempt)},)
    )
    started = time.monotonic()
    process.start()
    process.join(timeout_s)
    timed_out = process.is_alive()
    if timed_out:
        process.kill()
        process.join()
    result = _read(attempt.with_suffix(".json"))
    if (
        not timed_out
        and process.exitcode == 0
        and result is not None
        and result.get("input_sha256") == job["input_sha256"]
    ):
        record = result
    else:
        record = {
            **(_read(attempt.with_suffix(".scene.json")) or {}),
            "seed": job["seed"],
            "input_sha256": job["input_sha256"],
            "outcome": "timeout" if timed_out else "crash",
            "exit_code": process.exitcode,
            "wall_s": round(time.monotonic() - started, 3),
        }
    trace = attempt.with_suffix(".trace.jsonl")
    final_trace = final.with_suffix(".trace.jsonl")
    if trace.exists():
        os.replace(trace, final_trace)
    else:
        # This attempt traced nothing; an older run's trace must not stay beside it.
        final_trace.unlink(missing_ok=True)
    _write_atomic(final, record)
    for suffix in (".json", ".scene.json"):
        attempt.with_suffix(suffix).unlink(missing_ok=True)
    return record


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=_seeds, required=True, help="e.g. 0-24")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--settings", type=Path, required=True)
    parser.add_argument("--forklift-urdf", type=Path, required=True)
    parser.add_argument("--insertion-reserve-m", type=float, default=0.046)
    parser.add_argument(
        "--pallet-geometry",
        type=Path,
        default=ROOT / "config/pallet_geometry_epal6.yaml",
    )
    parser.add_argument(
        "--curvature",
        type=float,
        default=None,
        help="Override the settings' planner curvature (a sensitivity condition)",
    )
    parser.add_argument(
        "--scene-from",
        choices=("self", "provisional"),
        default="self",
        help="Build each seed's scene with this model's geometry or the "
        "provisional model's (policy reserve), to hold the scene fixed",
    )
    parser.add_argument("--obstacles", type=int, default=4)
    parser.add_argument("--return-home", action="store_true")
    parser.add_argument("--max-expansions", type=int, default=30000)
    parser.add_argument(
        "--retry-expansions",
        type=int,
        default=100000,
        help="Plan an expansion_limit failure once more with this budget; 0 = off",
    )
    parser.add_argument("--timeout-s", type=float, default=600.0)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)
    settings = yaml.safe_load(args.settings.read_text(encoding="utf-8"))
    curvature = (
        args.curvature
        if args.curvature is not None
        else settings["planner_curvature_inv_m"]
    )
    scene_urdf = (
        PROVISIONAL_URDF if args.scene_from == "provisional" else (args.forklift_urdf)
    )
    scene_reserve = (
        0.046 if args.scene_from == "provisional" else (args.insertion_reserve_m)
    )
    inputs = {
        "settings_sha256": hashlib.sha256(args.settings.read_bytes()).hexdigest(),
        "forklift_urdf_sha256": hashlib.sha256(
            args.forklift_urdf.read_bytes()
        ).hexdigest(),
        "scene_urdf_sha256": hashlib.sha256(Path(scene_urdf).read_bytes()).hexdigest(),
        "pallet_geometry_sha256": hashlib.sha256(
            args.pallet_geometry.read_bytes()
        ).hexdigest(),
        "sources_sha256": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [
                *sorted((ROOT / "src/forklift_core/planning").glob("*.py")),
                ROOT / "src/forklift_core/perception/pallet_geometry.py",
                ROOT / "sim/isaac/factory_assets.py",
                ROOT / "sim/isaac/insertion_geometry.py",
                Path(__file__).resolve(),
            ]
        },
        "max_expansions": args.max_expansions,
        "timeout_s": args.timeout_s,
        "reserve_m": args.insertion_reserve_m,
        "scene_reserve_m": scene_reserve,
        "curvature": curvature,
        "clearance": settings["planning_clearance_m"],
        "obstacles": args.obstacles,
        "return_home": args.return_home,
    }
    run_hash = hashlib.sha256(json.dumps(inputs, sort_keys=True).encode()).hexdigest()
    (args.output / "inputs.json").write_text(
        json.dumps({**inputs, "run_sha256": run_hash, "argv": sys.argv}, indent=2)
    )
    records = []
    for seed in args.seeds:
        job = {
            **inputs,
            "seed": seed,
            "input_sha256": run_hash,
            "output": str(args.output),
            "forklift_urdf": str(args.forklift_urdf),
            "scene_urdf": str(scene_urdf),
            "pallet_geometry": str(args.pallet_geometry),
        }
        path = args.output / f"seed_{seed}.json"
        record = _read(path)
        if record is None or record.get("input_sha256") != run_hash:
            record = run_seed(job, args.timeout_s, path)
        # The budget retry is its own result, resumed on its own.
        if args.retry_expansions and "expansion_limit" in str(record.get("status", "")):
            retry_dir = args.output / "retry"
            retry_dir.mkdir(exist_ok=True)
            retry_hash = hashlib.sha256(
                (run_hash + f":retry:{args.retry_expansions}").encode()
            ).hexdigest()
            retry_path = retry_dir / f"seed_{seed}.json"
            retry = _read(retry_path)
            if retry is None or retry.get("input_sha256") != retry_hash:
                retry = run_seed(
                    {
                        **job,
                        "input_sha256": retry_hash,
                        "max_expansions": args.retry_expansions,
                    },
                    args.timeout_s,
                    retry_path,
                )
            record["budget_retry"] = {
                "max_expansions": args.retry_expansions,
                "outcome": retry.get("outcome"),
                "status": retry.get("status"),
            }
            _write_atomic(path, record)
        records.append(record)
        print(
            f"seed {seed}: {record.get('outcome')} {record.get('status', '')}"
            + (
                f" retry={record['budget_retry']['status']}"
                if "budget_retry" in record
                else ""
            ),
            flush=True,
        )
    outcomes: dict[str, int] = {}
    for record in records:
        outcomes[record.get("outcome", "?")] = (
            outcomes.get(record.get("outcome", "?"), 0) + 1
        )
    statuses: dict[str, int] = {}
    for record in records:
        key = record.get("status", record.get("outcome"))
        statuses[key] = statuses.get(key, 0) + 1
    summary = {
        "run_sha256": run_hash,
        "seeds": len(records),
        "outcomes": outcomes,
        "statuses": statuses,
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
