"""Low-object clearance report for riib runs (lead decision 2026-11-04 after the #285 correction #287): every run's
distance to the low objects twice -- to the planner's keep-out rectangles (``scenario.props`` below the LiDAR plane, what
the truth evaluator and the low-clearance guard measure) and to the physical scene objects (``scene_layout.obstacles``
below the plane, what collides in Isaac) -- with the reopening checks of the on-hold stage 1.5
(docs/validation/2026-11-04-riib-unpermitted-entries-low-block.md, hold section):

- a) the closest physical approach (< 0.05 m) and any physical stop-arc violation (the evaluator's stopping volume at the
  envelope, against the physical objects);
- b) any overlap with a keep-out rectangle (an overlap test, not a distance: the distance is 0 when they overlap);
- c) the keep-out rectangle approach (reported as a rate across the runs, an advisory, not a gate).

Records only: ``run/result.json`` (scenario, scene_layout, geometry, samples) and ``run/slam_control.npy`` (truth rear
pose every control tick). The runner and the evaluator are not touched.

Shapes and sampling (said in the output too):
- the truck: the nominal outline the guard uses -- ``geometry.loaded_footprint`` in the loaded phases, otherwise
  ``geometry.unloaded_footprint`` (the hull beside the blades; the runner's reversing body-and-blades parts shape is not
  rebuilt) -- at the truth rear pose of every control tick;
- the carried pallet (loaded phases): its recorded outline at the 0.1 s samples only, as its own row;
- the stop arc: at the 0.1 s samples with |speed| > 0.05 m/s, the sample's curvature command (the evaluator uses the
  held steering curvature), the stopping distance of the permission model given on the command line;
- unevaluated ranges: control-tick gaps over 3 ticks, and the time after the last sample for the sample-based rows.

    python tools/low_clearance_report.py OUT.json RUN_DIR [RUN_DIR ...]
        [--latency-s 0.15 --decel-mps2 1.5 --margin-m 0.05 --envelope-m 0.01]   (RUN_DIR holds run/result.json)"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

from forklift_core.control.drive_permission import StoppingModel, arc_poses, shape_meets
from forklift_core.control.low_clearance import shape_clearance
from forklift_core.planning.geometry import Footprint, Rectangle

LIDAR_PLANE_M = 1.05
LOADED_PHASES = ("lift", "extract", "transport", "settle", "lower")
NEAR_PHYSICAL_M = 0.05
GAP_TICKS = 3


def _rect(d: dict) -> Rectangle:
    return Rectangle(float(d["x_m"]), float(d["y_m"]), float(d["length_m"]), float(d["width_m"]), float(d["yaw_rad"]))


def low_objects(s: dict) -> tuple[list, list]:
    """(keep-out rectangles, physical objects), each as (index in its own list, Rectangle), below the LiDAR plane."""
    keepout = [(i, _rect(p["rectangle"])) for i, p in enumerate((s.get("scenario") or {}).get("props") or [])
               if float(p["asset"]["height_m"]) <= LIDAR_PLANE_M]
    physical = [(i, _rect(o)) for i, o in enumerate((s.get("scene_layout") or {}).get("obstacles") or [])
                if float(o["height_m"]) <= LIDAR_PLANE_M]
    return keepout, physical


def _nearest(pose, shape, objs, beat: float = math.inf) -> tuple[float, int | None]:
    """The nearest object's distance; its index only when it beats ``beat`` (the running minimum), which keeps the
    per-tick cost at one pruned distance call."""
    if not objs:
        return math.inf, None
    d = shape_clearance(pose, shape, [r for _, r in objs])
    if d >= beat:
        return d, None
    return d, min(objs, key=lambda o: shape_clearance(pose, shape, [o[1]]))[0]


def _phase_at(samples: list, t: float) -> str | None:
    """the phase of the last sample at or before t (samples are 0.1 s apart)"""
    ph = None
    for x in samples:
        if float(x["time_s"]) > t + 1e-9:
            break
        ph = x.get("phase")
    return ph


def _gaps(times: np.ndarray, dt: float) -> list:
    if len(times) < 2:
        return []
    d = np.diff(times)
    return [[float(times[k]), float(times[k + 1])] for k in np.nonzero(d > GAP_TICKS * dt + 1e-9)[0]]


def run_report(rundir: str, stopping: StoppingModel, envelope_m: float) -> dict:
    s = json.load(open(os.path.join(rundir, "result.json")))
    keepout, physical = low_objects(s)
    g = s.get("geometry") or {}
    loaded_fp, unloaded_fp = Footprint(**g["loaded_footprint"]), Footprint(**g["unloaded_footprint"])
    samples = [x for x in s.get("samples") or [] if x.get("phase")]
    out = {"run": rundir, "keepout_objects": len(keepout), "physical_objects": len(physical),
           "shapes": {"loaded": "geometry.loaded_footprint", "unloaded": "geometry.unloaded_footprint (hull)",
                      "pallet": "recorded outline at samples"},
           "unevaluated": []}
    if not samples:
        out["unevaluated"].append("no samples")
        return out
    cpath = os.path.join(rundir, "slam_control.npy")
    C = np.load(cpath) if os.path.exists(cpath) else None
    if C is None or not len(C):
        out["unevaluated"].append("no slam_control.npy: the truck rows use the 0.1 s samples' rear pose, which is the "
                                  "estimate, so they are not truth and are left out")
    rows = {k: {"min_m": math.inf, "t": None, "object": None, "phase": None}
            for k in ("truck_keepout", "truck_physical", "pallet_keepout", "pallet_physical")}
    overlaps, arc_hits = [], []

    def note(key, d, t, obj, ph):
        if d < rows[key]["min_m"]:
            rows[key].update(min_m=float(d), t=float(t), object=obj, phase=ph)

    if C is not None and len(C):
        dt = float(np.median(np.diff(C[:, 0]))) if len(C) > 1 else 1 / 120
        out["control_tick_s"] = dt
        out["unevaluated"] += [{"control_gap_s": gp} for gp in _gaps(C[:, 0], dt)]
        t0 = float(samples[0]["time_s"])
        for row in C:
            t = float(row[0])
            if t < t0:
                continue
            ph = _phase_at(samples, t)
            shape = loaded_fp if ph in LOADED_PHASES else unloaded_fp
            pose = tuple(float(v) for v in row[4:7])
            dk, ok = _nearest(pose, shape, keepout, rows["truck_keepout"]["min_m"])
            dp, op = _nearest(pose, shape, physical, rows["truck_physical"]["min_m"])
            note("truck_keepout", dk, t, ok, ph)
            note("truck_physical", dp, t, op, ph)
            for i, r in (keepout if dk <= 0.0 else ()):  # the distance is 0 when they meet; the overlap test decides
                if shape_meets(r, shape, pose, 0.0):
                    if not overlaps or overlaps[-1]["object"] != i or t - overlaps[-1]["to_s"] > 2 * dt:
                        overlaps.append({"object": i, "from_s": t, "to_s": t, "phase": ph})
                    else:
                        overlaps[-1]["to_s"] = t
    pd, pw = float(g.get("pallet_depth_m", 0.6)), float(g.get("pallet_width_m", 0.8))
    for x in samples:
        t, ph = float(x["time_s"]), x["phase"]
        if ph in LOADED_PHASES and x.get("pallet_position_m") is not None:
            pal = Rectangle(float(x["pallet_position_m"][0]), float(x["pallet_position_m"][1]), pd, pw,
                            float(x["pallet_yaw_rad"]))
            pal_shape = Footprint(pd / 2, pd / 2, pw / 2)
            pose = (pal.x_m, pal.y_m, pal.yaw_rad)
            dk, ok = _nearest(pose, pal_shape, keepout, rows["pallet_keepout"]["min_m"])
            dp, op = _nearest(pose, pal_shape, physical, rows["pallet_physical"]["min_m"])
            note("pallet_keepout", dk, t, ok, ph)
            note("pallet_physical", dp, t, op, ph)
        if C is None or not len(C):
            continue
        v = float(x.get("signed_speed_mps") or 0.0)
        if abs(v) <= 0.05:
            continue
        i = int(np.clip(np.searchsorted(C[:, 0], t), 0, len(C) - 1))
        pose = tuple(float(c) for c in C[i, 4:7])
        shape = loaded_fp if ph in LOADED_PHASES else unloaded_fp
        poses = arc_poses(pose, float(x.get("curvature_inv_m") or 0.0), 1 if v > 0 else -1,
                          stopping.distance_m(v), 0.025)[0]
        hit = [j for j, r in physical if any(shape_meets(r, shape, tuple(p), envelope_m) for p in poses[1:])]
        if hit:
            arc_hits.append({"t": t, "phase": ph, "objects": hit, "speed_mps": v})
    out["unevaluated"].append({"after_last_sample_s": float(samples[-1]["time_s"])})
    out["rows"] = {k: ({**r, "min_m": None} if math.isinf(r["min_m"]) else r) for k, r in rows.items()}
    out["keepout_overlaps"] = overlaps
    out["physical_arc_violations"] = arc_hits
    phys = [r["min_m"] for k, r in rows.items() if k.endswith("physical") and not math.isinf(r["min_m"])]
    out["reopen"] = {
        "a_physical_near": bool(phys and min(phys) < NEAR_PHYSICAL_M) or bool(arc_hits),
        "b_keepout_overlap": bool(overlaps),
    }
    return out


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--latency-s", type=float, default=0.15)
    ap.add_argument("--decel-mps2", type=float, default=1.5)
    ap.add_argument("--margin-m", type=float, default=0.05)
    ap.add_argument("--envelope-m", type=float, default=0.01)
    a = ap.parse_args(argv)
    stopping = StoppingModel(a.latency_s, a.decel_mps2, a.margin_m)
    reports = []
    for d in a.runs:
        run = os.path.join(d, "run") if os.path.isdir(os.path.join(d, "run")) else d
        try:
            reports.append(run_report(run, stopping, a.envelope_m))
        except Exception as e:  # a record this tool cannot read is listed, never silently dropped
            reports.append({"run": run, "error": repr(e)[:300]})
    near_keepout = [r for r in reports if (r.get("rows") or {}).get("truck_keepout", {}).get("min_m") is not None
                    and r["rows"]["truck_keepout"]["min_m"] < NEAR_PHYSICAL_M]
    summary = {"runs": len(reports), "errors": sum("error" in r for r in reports),
               "reopen_a": [r["run"] for r in reports if (r.get("reopen") or {}).get("a_physical_near")],
               "reopen_b": [r["run"] for r in reports if (r.get("reopen") or {}).get("b_keepout_overlap")],
               # advisory only (not a gate): runs whose truck came within the guard's 0.05 m of a keep-out rectangle
               "advisory_c_keepout_within_0p05_runs": len(near_keepout)}
    json.dump({"summary": summary, "runs": reports}, open(a.out, "w"), indent=1, default=str)
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
