#!/usr/bin/env python3
"""Offline bench of the two per-tick permissions a live TELEOP tick computes (docs/design/2026-11-12-teleop-pacing.md):
the core's watch (``ObstacleLayer.limit`` at the current curvature, both directions as ``_direction_limits`` asks)
and the arbiter's teleop permit (``BundlePermit.limit`` on a curvature bundle, both ends in full every call).

No Isaac: the live layer configuration (team_spec), a synthetic 2D scene of boxes ray-cast on the layer's own beams,
scans fed at 10 Hz while the truck stands, one refresh, then N calls with the pose creeping forward as when driving
(the bundle's cache keeps its arcs; the ends are asked in full each call). Reports per-call wall times and, with
``--profile``, the hottest functions. Times are this machine's: the same bench on another host gives the ratio.

    python tools/permit_bench.py [--calls 2000] [--profile] [--json out.json]
"""

from __future__ import annotations

import argparse
import cProfile
import importlib.util
import io
import json
import math
import pstats
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "sim" / "isaac"))


def _layer_module():
    mod = sys.modules.get("obstacle_layer")
    if mod is None:
        spec = importlib.util.spec_from_file_location("obstacle_layer", REPO / "sim/isaac/obstacle_layer.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules["obstacle_layer"] = mod
        spec.loader.exec_module(mod)
    return mod


# (cx, cy, half_x, half_y) boxes around the start pose (0, 0, 0): a wall behind, a rack to the left, a box ahead
BOXES = ((-1.2, 0.0, 0.05, 2.0), (1.5, 1.6, 2.0, 0.15), (3.0, -0.6, 0.3, 0.3), (6.0, 0.0, 0.05, 3.0))


def ray_boxes(origin, angles, rmax):
    """Distance to the nearest box face per ray (slab method); rmax and no hit where none."""
    ox, oy = origin
    dx, dy = np.cos(angles), np.sin(angles)
    best = np.full(len(angles), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for cx, cy, hx, hy in BOXES:
            t1x, t2x = (cx - hx - ox) / dx, (cx + hx - ox) / dx
            t1y, t2y = (cy - hy - oy) / dy, (cy + hy - oy) / dy
            tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
            tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
            ok = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0)
            best = np.where(ok & (tmin < best), tmin, best)
    hit = np.isfinite(best) & (best <= rmax)
    return np.where(hit, best, rmax), hit


def build(seed: int = 1):
    mod = _layer_module()
    from forklift_core.perception.obstacle_grid import AgeErrorTable
    from forklift_core.planning.geometry import Bounds, Footprint

    age = json.loads((REPO / "config/obstacle_odometry_age.json").read_text())
    table = AgeErrorTable(tuple(age["ages_s"]), tuple(age["cumulative_position_m"]), tuple(age["cumulative_yaw_rad"]))
    cfg = mod.load_layer_config(REPO / "config/obstacle_layer_single_team_spec.yaml")
    layer = mod.ObstacleLayer(cfg, hall=Bounds(-5, 15, -6, 6), error_table=table,
                              unloaded=Footprint(1.29, 0.17, 0.36), loaded=Footprint(1.53, 0.17, 0.40),
                              body_front_m=0.944, rear_axle_x_in_base_m=-0.34, noise_seed=seed)
    pose = (0.0, 0.0, 0.0)
    t = 0.0
    for _ in range(10):  # 1 s of scans at 10 Hz, standing
        raw = {}
        for sensor in layer.sensors:
            sx = pose[0] + (sensor.xyz_m[0] - layer.rear_x) * math.cos(pose[2])
            sy = pose[1] + (sensor.xyz_m[0] - layer.rear_x) * math.sin(pose[2])
            dist, hit = ray_boxes((sx, sy), layer.beam_angles + pose[2] + sensor.yaw_rad, layer.range_max_m)
            raw[sensor.name] = (dist, hit, np.zeros(len(dist), bool))
        layer.add_scans(t, raw, odom_rear=pose, loaded=False)
        t += 0.1
    layer.refresh(t, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=np.array([pose]), loaded=False)
    return layer, t


def bench(calls: int, profile: bool, kernels: str = "fast") -> dict:
    from forklift_core import kernels as kmod
    from forklift_core.safety.teleop_drive import BundlePermit, bundle

    if kernels == "fast":
        kmod.prefer_fast()  # as a real-time run does
    else:
        kmod.set_mode(kernels)
    warm_s = kmod.warm_up()
    layer, t = build()
    snap = layer.snapshot
    bp = BundlePermit(layer.permission.config)
    curv = bundle(0.0, 0.0, 0.05)
    out = {"curvatures": curv, "kernels": kmod.mode(), "warm_up_s": warm_s}
    prof = cProfile.Profile() if profile else None

    def core_watch(i):
        pose = (0.001 * i, 0.0, 0.0)
        for d in (1, -1):  # _direction_limits asks the driving direction and the other one the bound allows
            layer.limit(t, current_pose=pose, curvature_inv_m=0.0, direction=d, loaded=False, cap_mps=0.13)

    def teleop(i):
        pose = (0.001 * i, 0.0, 0.0)
        fp, own = layer.footprints(False, 1)
        bp.advance(0.001)
        bp.limit(snap, (id(snap), float(snap.stamp_s), 0, False), t, layer.last_scan_s, pose=pose, curvatures=curv,
                 direction=1, check_speed_mps=0.13, footprint=fp, own_footprint=own)

    def teleop_install(i):
        # a new snapshot (every scan, 10 Hz live): the bundle's arcs are installed again, then the ends asked
        pose = (0.001 * i, 0.0, 0.0)
        fp, own = layer.footprints(False, 1)
        BundlePermit(layer.permission.config).limit(snap, ("cold", i), t, layer.last_scan_s, pose=pose,
                                                     curvatures=curv, direction=1, check_speed_mps=0.13,
                                                     footprint=fp, own_footprint=own)

    for name, fn in (("core_watch", core_watch), ("teleop_permit", teleop), ("teleop_permit_install", teleop_install)):
        fn(0)  # warm-up (installs the bundle's arcs)
        times = []
        if prof is not None:
            prof.enable()
        for i in range(1, calls + 1):
            t0 = time.perf_counter()
            fn(i)
            times.append(time.perf_counter() - t0)
        if prof is not None:
            prof.disable()
        ms = np.array(times) * 1e3
        out[name] = {"mean_ms": float(ms.mean()), "p50_ms": float(np.percentile(ms, 50)),
                     "p99_ms": float(np.percentile(ms, 99)), "max_ms": float(ms.max())}
    if prof is not None:
        s = io.StringIO()
        pstats.Stats(prof, stream=s).sort_stats("tottime").print_stats(18)
        out["profile"] = s.getvalue()
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--calls", type=int, default=2000)
    ap.add_argument("--profile", action="store_true")
    ap.add_argument("--kernels", default="fast", choices=("fast", "reference", "numpy", "numba"),
                    help="fast: what a real-time run takes (numba, else numpy)")
    ap.add_argument("--json")
    args = ap.parse_args(argv)
    out = bench(args.calls, args.profile, args.kernels)
    prof = out.pop("profile", None)
    print(json.dumps(out, indent=1))
    if prof:
        print(prof)
    if args.json:
        Path(args.json).write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
