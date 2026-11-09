#!/usr/bin/env python3
"""Real-time plan G3-1d microbench (decision 2026-11-09 after #387): the 2D LiDAR's PhysX casting per scan, at one
physics state, by arm -- and whether each arm returns exactly what the runner's loop returns.

  A  planar_lidar.cast_scan_flags (the runner's loop; the oracle)
  B  planar_lidar.cast_scan_flags_fast (directions converted once, bound calls, a per-body own cache)
  T  B's loop with plain tuples for the vectors instead of carb.Float3, if the binding takes them

The query interface's other methods are listed (``query_methods``); a batch query, if the build has one, is reported
for a follow-up, not timed here (its signature is not known in advance). Adoption threshold (plan 5.20): an arm saves
at least 1 ms per scan against A with identical results at every pose.

The stage is a run's saved scene (``run/scene.usda``: the truck, the pallets and the environment as that run built
them), opened and waited for until it has loaded (``is_stage_loading``, --load-timeout-s), with the truck's
base_link and colliders checked. The saved physics settings are kept (``set_defaults=False``); after ``play`` and
two steps the physics is not stepped again: every arm casts the same state. Arm order rotates every repeat.

Validity (Codex bench review): the oracle A must return the same arrays at every repeat, and every arm's arrays must
equal A's; the colliders each ray hits (rigidBody, collision, distance) are compared once per pose outside the timing
for T against A's Float3 query. ``adopt`` needs all of it and a saving of at least 1 ms per scan. Any failure is
recorded and the process exits non-zero (no ``adopt`` left beside an error).

Limits: the sensor moves about a truck that does not; the rays repeat over a warm cache without physics or rendering
load -- a saving here is per scan at rest, not yet the live one. Poses: the truck's LiDAR origin (base_link + the bridge's mount -0.12, 0, 1.05) and --poses - 1 more, moved
in x/y by up to --spread-m and turned, so the rays meet the truck's own body, pallets and walls in different shares.

  $VENV_PY sim/isaac/lidar_bench.py --scene RUN/scene.usda --output OUT [--rays 1600 --range-m 12 --repeats 30]

Writes OUT/lidar_bench.json; touches nothing else.
"""

import argparse
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--scene", required=True)
ap.add_argument("--output", type=Path, required=True)
ap.add_argument("--rays", type=int, default=1600)
ap.add_argument("--range-m", type=float, default=12.0)
ap.add_argument("--repeats", type=int, default=30)
ap.add_argument("--poses", type=int, default=8)
ap.add_argument("--spread-m", type=float, default=0.6)
ap.add_argument("--base-link", default="/World/Forklift/base_link")
ap.add_argument("--mount", default="-0.12,0.0,1.05,0.0", help="x,y,z,yaw of the laser on base_link (the bridge's)")
ap.add_argument("--load-timeout-s", type=float, default=600.0)
args = ap.parse_args()
args.output.mkdir(parents=True, exist_ok=False)

from isaacsim import SimulationApp  # noqa: E402 -- before any other Isaac import

app = SimulationApp({"headless": True, "multi_gpu": False})
record = {"conditions": {k: str(v) for k, v in vars(args).items()}, "errors": []}


def same(a, b) -> bool:
    return all(np.array_equal(x, y, equal_nan=True) for x, y in zip(a, b))


def tuple_cast(query, origin, directions, range_m, prefixes, cache):
    """B's loop with tuples for the vectors (arm T)."""
    cast, reach = query.raycast_closest, float(range_m)
    start = tuple(map(float, origin))
    n = len(directions)
    distances, hits, own = np.full(n, np.nan), np.zeros(n, bool), np.zeros(n, bool)
    for i, d in enumerate(np.asarray(directions, float).tolist()):
        r = cast(start, tuple(d), reach, False)
        if not r.get("hit", False):
            continue
        hits[i] = True
        distances[i] = float(r["distance"])
        key = (prefixes, r.get("rigidBody", ""), r.get("collision", ""))
        f = cache.get(key)
        if f is None:
            body = str(key[1]) + str(key[2])
            f = cache[key] = any(p in body for p in prefixes)
        own[i] = f
    return distances, hits, own


def bodies(query, start, directions, range_m):
    """(hit, distance, rigidBody, collision) per ray, untimed."""
    out = []
    for d in np.asarray(directions, float).tolist():
        r = query.raycast_closest(start, d if isinstance(start, tuple) else type(start)(*d), float(range_m), False)
        out.append((bool(r.get("hit", False)), r.get("distance"), r.get("rigidBody"), r.get("collision")))
    return out


def main():
    import carb
    import omni.usd
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.utils.stage import is_stage_loading
    from omni.physx import get_physx_scene_query_interface
    from pxr import Usd, UsdGeom, UsdPhysics

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import planar_lidar as pl

    pl._PROFILE_EVERY = 0
    if not omni.usd.get_context().open_stage(args.scene):
        raise RuntimeError(f"open_stage failed: {args.scene}")
    t0 = time.monotonic()
    app.update()
    while is_stage_loading():
        if time.monotonic() - t0 > args.load_timeout_s:
            raise RuntimeError(f"the stage was still loading after {args.load_timeout_s} s")
        app.update()
    record["load_s"] = time.monotonic() - t0
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(args.base_link).IsValid():
        raise RuntimeError(f"no {args.base_link} on the stage")
    colliders = [p.GetPath().pathString for p in Usd.PrimRange(stage.GetPseudoRoot())
                 if p.HasAPI(UsdPhysics.CollisionAPI)]
    record["colliders"] = {"all": len(colliders), "truck": sum(c.startswith(pl.SELF_PREFIX) for c in colliders)}
    if not record["colliders"]["truck"] or record["colliders"]["all"] <= record["colliders"]["truck"]:
        raise RuntimeError(f"too few colliders: {record['colliders']}")
    sim = SimulationContext(stage_units_in_meters=1.0, set_defaults=False)  # the saved scene's settings, its dt too
    record["physics_dt"] = sim.get_physics_dt()
    sim.initialize_physics()
    sim.play()
    sim.step(render=False)
    sim.step(render=False)
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(args.base_link)).ComputeLocalToWorldTransform(0)
    base = np.array(xf.ExtractTranslation(), float)
    rot = np.array(xf.ExtractRotationMatrix(), float).T  # world_from_base (Gf rows are the basis vectors)
    mx, my, mz, myaw = map(float, args.mount.split(","))
    query = get_physx_scene_query_interface()
    record["query_methods"] = sorted(m for m in dir(query) if not m.startswith("_"))
    angles = np.linspace(-math.pi, math.pi, args.rays, endpoint=False)
    rng = np.random.default_rng(0)
    poses = []
    for k in range(args.poses):
        off = np.zeros(3) if k == 0 else np.array([*rng.uniform(-args.spread_m, args.spread_m, 2), 0.0])
        yaw = myaw + (0.0 if k == 0 else float(rng.uniform(-math.pi, math.pi)))
        origin = base + rot @ np.array([mx, my, mz]) + off
        in_base = np.column_stack((np.cos(angles + yaw), np.sin(angles + yaw), np.zeros(len(angles))))
        poses.append((origin, in_base @ rot.T))
    prefixes = (pl.SELF_PREFIX,)
    arms = {"A": lambda o, d, c: pl.cast_scan_flags(o, d, args.range_m, own_prefixes=prefixes),
            "B": lambda o, d, c: pl.cast_scan_flags_fast(o, d, args.range_m, own_prefixes=prefixes, query=query,
                                                         own_cache=c),
            "T": lambda o, d, c: tuple_cast(query, o, d, args.range_m, prefixes, c)}
    try:
        arms["T"](*poses[0], {})
    except Exception as exc:  # noqa: BLE001 -- the binding may not take tuples: the arm is reported, not timed
        record["T_unsupported"] = f"{type(exc).__name__}: {exc}"
        del arms["T"]
    out = []
    names = list(arms)
    for k, (origin, dirs) in enumerate(poses):
        oracle = arms["A"](origin, dirs, None)
        row = {"pose": k, "origin": origin.tolist(), "hits": int(oracle[1].sum()), "own": int(oracle[2].sum()),
               "equal": {}, "ms": {}}
        if "T" in arms:  # the colliders themselves, outside the timing: tuple vectors against Float3
            f3 = bodies(query, carb.Float3(*map(float, origin)), dirs, args.range_m)
            tp = bodies(query, tuple(map(float, origin)), dirs, args.range_m)
            row["bodies_equal_T"] = f3 == tp
        caches = {name: {} for name in arms}
        times = {name: [] for name in arms}
        for rep_ in range(args.repeats):
            order = names[rep_ % len(names):] + names[:rep_ % len(names)]  # rotated: no arm always first
            for name in order:
                fn = arms[name]
                t0 = time.perf_counter()
                res = fn(origin, dirs, caches[name])
                times[name].append((time.perf_counter() - t0) * 1e3)
                row["equal"].setdefault(name, True)
                row["equal"][name] &= same(res, oracle)
        row["ms"] = {n: {"median": float(np.median(v)), "p90": float(np.percentile(v, 90))} for n, v in times.items()}
        out.append(row)
    record["poses"] = out
    record["summary"] = {n: {"median_ms": float(np.median([r["ms"][n]["median"] for r in out])),
                             "all_equal": all(r["equal"][n] for r in out)} for n in arms}
    if "T" in arms:
        record["summary"]["T"]["bodies_equal"] = all(r.get("bodies_equal_T", False) for r in out)
    oracle_stable = record["summary"]["A"]["all_equal"]
    record["oracle_stable"] = oracle_stable
    base_ms = record["summary"]["A"]["median_ms"]
    for n, s in record["summary"].items():
        s["saving_ms_per_scan"] = base_ms - s["median_ms"]
        s["adopt"] = bool(n != "A" and oracle_stable and s["all_equal"] and s.get("bodies_equal", True)
                          and s["saving_ms_per_scan"] >= 1.0)
    sim.stop()


rc = 0
try:
    main()
except Exception:  # noqa: BLE001 -- the record keeps the failure; the exit code says it
    record["errors"].append(traceback.format_exc())
if record["errors"] or not record.get("oracle_stable", False):
    rc = 1
    for s in record.get("summary", {}).values():
        s["adopt"] = False  # no adoption beside an error or an unstable oracle
(args.output / "lidar_bench.json").write_text(json.dumps(record, indent=1, default=str))
print(json.dumps(record.get("summary", record["errors"]), indent=1, default=str), flush=True)
if rc:
    # app.close() may end the process with code 0 on its own (fast shutdown; run_transport saw it): a failure leaves
    # before it (Codex bench review round 2)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
app.close()
os._exit(0)
