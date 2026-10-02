"""Choose observation waypoints to append to the runner's list (G4 plan).

Plan: docs/plans/2026-10-02-g4-observation-candidates.md. On design seeds
(never the G5 seeds 1000-1029, nor 100-129) this rebuilds each scenario offline, replays the
runner's observation order -- plan from the start to the first candidate whose
leg plans, and from a candidate whose view is too poor to the next one -- and
scores each view by how many approach-face pixels of the pallet the camera
sees past the props (the G3 tool's front-face definition). A seed is served
when some candidate in the sequence sees at least ``--proxy`` pixels. Then it
greedily appends grid candidates that serve the most unserved seeds, with the
plan's fixed tie-break, removal and stop rules.

The proxy only picks candidates; detection success is decided in Isaac (G2').

    python -m tools.observation_candidate_design \\
        --record artifacts/<g2 run>/seed_0/result.json --seeds 200:400 --json out.json
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import sys
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from forklift_core.planning import Footprint, Pose2D
from forklift_core.planning.hybrid_astar import PlannerConfig
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    SyntheticMissionGeometry,
    make_scenario,
    plan_observation_leg,
    plan_transport,
)
from tools import scene_rig
from tools.diagnose_detection import PalletTruth, pallet_parts, raycast, truth_in_base

ROOT = Path(__file__).resolve().parents[1]
FACTORY_PROPS = (
    "SM_BarelPlastic_A_01.usd",
    "SM_CratePlastic_D_01.usd",
    "SM_CardBoxA_02.usd",
)
# The runner's five candidates before G4 -- the list the design appends to.
DEFAULT_CANDIDATES = (
    (-0.10, 0.90, 0.0),
    (-1.20, 0.30, 0.0),
    (-0.10, -0.60, 0.0),
    (-1.50, -0.60, 0.0),
    (-2.00, -0.30, 0.0),
)
GRID_X = (-1.8, -1.2, -0.6, 0.0, 0.4)
GRID_Y = (1.2, 1.5, 1.8, 2.1, 2.4)
GRID_YAW = (0.0, -0.25)
# G5 seeds (closeout amendment 2026-10-02), and the range R1 already planned.
G5_SEEDS = range(1000, 1030)
RESERVED_SEEDS = (*G5_SEEDS, *range(100, 130))
REAR_TO_BASE_M = 0.34  # run_transport: base_link sits 0.34 m ahead of the rear axle


@dataclasses.dataclass(frozen=True)
class RunConfig:
    """What the runner planned with, rebuilt from one of its records."""

    catalogue: tuple[AssetSpec, ...]
    obstacles: int
    planner: PlannerConfig
    geometry: SyntheticMissionGeometry
    rear_axle_offset_m: float
    intrinsics: object

    @classmethod
    def from_record(cls, record: dict) -> RunConfig:
        specs = {}
        for prop in record["scenario"]["props"]:
            asset = prop["asset"]
            specs[asset["uri"].rsplit("/", 1)[1]] = AssetSpec(**asset)
        missing = [name for name in FACTORY_PROPS if name not in specs]
        if missing:
            raise ValueError(f"record lacks catalogue entries {missing}")
        catalogue = tuple(specs[name] for name in FACTORY_PROPS)
        planner = PlannerConfig(**record["planner_config"])
        g = dict(record["geometry"])
        for key in ("unloaded_footprint",):
            g[key] = Footprint(**g[key])
        fields = {
            f.name for f in dataclasses.fields(SyntheticMissionGeometry) if f.init
        }
        geometry = SyntheticMissionGeometry(
            **{k: v for k, v in g.items() if k in fields}
        )
        k = record["observation_attempts"][0]["capture_diagnostics"]["intrinsics"][
            "integer_index"
        ]["matrix"]
        intrinsics = dataclasses.replace(
            scene_rig.intrinsics(), fx=k[0][0], fy=k[1][1], cx=k[0][2], cy=k[1][2]
        )
        return cls(
            catalogue,
            int(record["arguments"]["obstacles"]),
            planner,
            geometry,
            abs(float(record["arguments"]["rear_axle_offset_m"])),
            intrinsics,
        )


PARTS = pallet_parts(ROOT / "sim/models/epal6_pallet/pallet.urdf")


def front_visible(scenario, config: RunConfig, rear: Pose2D) -> int:
    """Approach-face pixels the perception camera sees from this rear-axle pose."""
    bx = rear.x_m + config.rear_axle_offset_m * math.cos(rear.yaw_rad)
    by = rear.y_m + config.rear_axle_offset_m * math.sin(rear.yaw_rad)
    pose = (
        [bx, by, 0.0],
        [math.cos(rear.yaw_rad / 2), 0.0, 0.0, math.sin(rear.yaw_rad / 2)],
    )
    pickup = scenario.pickup
    truth = PalletTruth(
        pickup.x_m,
        pickup.y_m,
        0.0,
        pickup.yaw_rad,
        config.geometry.pallet_depth_m,
        PARTS,
    )
    boxes, _, front_point, front_normal = truth_in_base(truth, pose)
    c, s = math.cos(-rear.yaw_rad), math.sin(-rear.yaw_rad)
    props = []
    for prop in scenario.props:
        r = prop.rectangle
        dx, dy = r.x_m - bx, r.y_m - by
        props.append(
            scene_rig.Box(
                (c * dx - s * dy, s * dx + c * dy, prop.asset.height_m / 2),
                (r.length_m, r.width_m, prop.asset.height_m),
                r.yaw_rad - rear.yaw_rad,
            )
        )
    transform = scene_rig.Camera().base_from_optical()
    expected, index, rays, origin = raycast(boxes, config.intrinsics, transform)
    with_props, _, _, _ = raycast(boxes + props, config.intrinsics, transform)
    hit = origin + rays * np.where(np.isfinite(expected), expected, 0.0)[..., None]
    front = np.zeros(expected.shape, dtype=bool)
    for i, box in enumerate(boxes):
        mask = index == i
        if not mask.any():
            continue
        cc, ss = math.cos(box.yaw_rad), math.sin(box.yaw_rad)
        delta = hit[mask] - np.asarray(box.centre_m)
        local_x = cc * delta[:, 0] + ss * delta[:, 1]
        front[mask] = (np.abs(local_x + box.size_m[0] / 2) <= 1e-6) & (
            np.abs((hit[mask] - front_point) @ front_normal) <= 0.01
        )
    with np.errstate(invalid="ignore"):
        seen = front & (np.abs(with_props - expected) <= 1e-6)
    return int(seen.sum())


class Evaluator:
    """Caches each (seed, start, candidate) leg and each view."""

    def __init__(self, config: RunConfig, reserved: Sequence[int] = RESERVED_SEEDS):
        self.config = config
        # Seeds this caller may never generate; the G4 design keeps its own.
        self.reserved = frozenset(reserved)
        self._scenarios, self._legs, self._views, self._approaches = {}, {}, {}, {}

    def scenario(self, seed: int):
        if seed in self.reserved:
            raise ValueError(f"seed {seed} is reserved: not for design here")
        if seed not in self._scenarios:
            self._scenarios[seed] = make_scenario(
                seed,
                self.config.catalogue,
                self.config.obstacles,
                geometry=self.config.geometry,
            )
        return self._scenarios[seed]

    def leg(self, seed, start: Pose2D | None, candidate: tuple):
        key = (
            seed,
            None if start is None else (start.x_m, start.y_m, start.yaw_rad),
            candidate,
        )
        if key not in self._legs:
            result = plan_observation_leg(
                self.scenario(seed),
                Pose2D(*candidate),
                self.config.planner,
                geometry=self.config.geometry,
                start_rear=start,
            )
            self._legs[key] = (result.success, result.status, float(result.length_m))
        return self._legs[key]

    def approach(self, seed, candidate: tuple) -> str:
        """The mission plan from this viewpoint to the true pickup (a secondary check)."""
        key = (seed, candidate)
        if key not in self._approaches:
            result = plan_transport(
                self.scenario(seed),
                self.config.planner,
                geometry=self.config.geometry,
                start_rear=Pose2D(*candidate),
            )
            self._approaches[key] = result.status
        return self._approaches[key]

    def view(self, seed, candidate: tuple) -> int:
        key = (seed, candidate)
        if key not in self._views:
            self._views[key] = front_visible(
                self.scenario(seed), self.config, Pose2D(*candidate)
            )
        return self._views[key]

    def sequence(self, seed: int, candidates: Sequence[tuple], proxy: int) -> dict:
        """The runner's order: plan, look, move on from poor views from where you stand."""
        start, travelled, trace = None, 0.0, []
        for candidate in candidates:
            success, status, length = self.leg(seed, start, candidate)
            trace.append({"candidate": candidate, "plan": status})
            if not success:
                continue
            travelled += length
            pixels = self.view(seed, candidate)
            trace[-1]["front_visible"] = pixels
            if pixels >= proxy:
                # The runner plans the mission from here on a valid detection;
                # a view the mission cannot leave from does not count.
                status = self.approach(seed, candidate)
                trace[-1]["approach"] = status
                served = status == "success"
                return {
                    "served": served,
                    "by": candidate if served else None,
                    "travelled_m": travelled,
                    "trace": trace,
                }
            # The runner plans the next leg from the accepted pose here; the
            # nominal candidate stands in for it (a few cm off in Isaac).
            start = Pose2D(*candidate)
        return {"served": False, "by": None, "travelled_m": travelled, "trace": trace}


_WORKER: Evaluator | None = None


def _init_worker(record_path: str) -> None:
    global _WORKER
    _WORKER = Evaluator(
        RunConfig.from_record(json.loads(Path(record_path).read_text()))
    )


def _serving(job) -> tuple[int, list]:
    """For one seed: which grid candidates, appended to ``prefix``, serve it."""
    seed, prefix, grid, proxy = job
    return seed, [
        candidate
        for candidate in grid
        if _WORKER.sequence(seed, list(prefix) + [candidate], proxy)["served"]
    ]


def _trace(job) -> tuple[int, dict]:
    seed, candidates, proxy = job
    return seed, _WORKER.sequence(seed, list(candidates), proxy)


def _map(pool, function, jobs):
    return list(pool.map(function, jobs)) if pool else [function(j) for j in jobs]


def choose(seeds: Sequence[int], proxy: int, limit: int = 3, pool=None):
    """The plan's greedy rule: most newly served seeds, fixed tie-break, stop at zero.

    Tie-break: smaller x, then smaller y, then yaw 0; the chosen order is the
    append order.
    """
    grid = sorted(
        ((x, y, yaw) for x in GRID_X for y in GRID_Y for yaw in GRID_YAW),
        key=lambda c: (c[0], c[1], c[2] != 0.0),
    )
    chosen: list[tuple] = []
    base = list(DEFAULT_CANDIDATES)
    traces = dict(_map(pool, _trace, [(s, tuple(base), proxy) for s in seeds]))
    unserved = [s for s in seeds if not traces[s]["served"]]
    rounds = [{"unserved": list(unserved)}]
    while unserved and len(chosen) < limit:
        options = [c for c in grid if c not in chosen]
        serving = dict(
            _map(
                pool,
                _serving,
                [(s, tuple(base + chosen), options, proxy) for s in unserved],
            )
        )
        best, best_served = None, []
        for candidate in options:
            served = [s for s in unserved if candidate in serving[s]]
            if len(served) > len(best_served):
                best, best_served = candidate, served
        if best is None:
            break
        chosen.append(best)
        unserved = [s for s in unserved if s not in best_served]
        rounds.append(
            {"chose": best, "served": best_served, "unserved": list(unserved)}
        )
    return chosen, rounds


def parse_range(text: str) -> range:
    start, stop = (int(v) for v in text.split(":"))
    return range(start, stop)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--record", type=Path, required=True, help="a run_transport result.json"
    )
    parser.add_argument("--seeds", type=parse_range, default=range(200, 400))
    parser.add_argument("--proxy", type=int, default=1500)
    parser.add_argument("--limit", type=int, default=3)
    parser.add_argument("--json", type=Path, default=None)
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args(argv)
    if set(args.seeds) & set(RESERVED_SEEDS):
        parser.error("design seeds must not include the G5 seeds (1000-1029, 100-129)")
    from concurrent.futures import ProcessPoolExecutor

    seeds = list(args.seeds)
    pool = (
        ProcessPoolExecutor(
            args.workers, initializer=_init_worker, initargs=(str(args.record),)
        )
        if args.workers > 1
        else None
    )
    if pool is None:
        _init_worker(str(args.record))
    try:
        chosen, rounds = choose(seeds, args.proxy, args.limit, pool)
        final = list(DEFAULT_CANDIDATES) + chosen
        base_traces = dict(
            _map(pool, _trace, [(s, DEFAULT_CANDIDATES, args.proxy) for s in seeds])
        )
        final_traces = dict(
            _map(pool, _trace, [(s, tuple(final), args.proxy) for s in seeds])
        )
    finally:
        if pool:
            pool.shutdown()
    served = sum(final_traces[s]["served"] for s in seeds)
    base_served = sum(base_traces[s]["served"] for s in seeds)
    print(
        f"# design seeds {args.seeds.start}-{args.seeds.stop - 1}, proxy {args.proxy} px"
    )
    print(f"# served with the existing five: {base_served}/{len(args.seeds)}")
    for r in rounds[1:]:
        print(f"# + {r['chose']}: serves {len(r['served'])}, {len(r['unserved'])} left")
    print(f"# served with the appended list: {served}/{len(args.seeds)}")
    print(f"# appended: {chosen}")
    if args.json:
        args.json.write_text(
            json.dumps(
                {
                    "record": str(args.record),
                    "record_sha256": hashlib.sha256(
                        args.record.read_bytes()
                    ).hexdigest(),
                    "seeds": [args.seeds.start, args.seeds.stop],
                    "proxy": args.proxy,
                    "grid": {"x": GRID_X, "y": GRID_Y, "yaw": GRID_YAW},
                    "chosen": chosen,
                    "rounds": rounds,
                    "served": served,
                    "base_served": base_served,
                    # Per seed: candidate sequence, plans, views, approach, travel.
                    "base_traces": {str(k): v for k, v in base_traces.items()},
                    "final_traces": {str(k): v for k, v in final_traces.items()},
                },
                indent=2,
            )
            + "\n"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
