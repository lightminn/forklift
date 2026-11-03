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
- the phase of a control tick comes from the recorded ``transitions`` (not from the 0.1 s samples);
- the truck: the nominal outline the guard uses -- ``geometry.loaded_footprint`` in the loaded phases, otherwise
  ``geometry.unloaded_footprint`` (the hull beside the blades) -- at the truth rear pose of every control tick;
- the carried pallet (loaded phases): its recorded outline at the 0.1 s samples only, as its own row, in the keep-out
  overlap test too;
- the stop arc: at every control tick, from the truth motion -- signed speed and path curvature from the truth rear
  poses of the neighbouring ticks (the evaluator uses the held steering curvature; the truth path curvature stands for it)
  -- the stopping distance of the permission model given on the command line, the evaluator's envelope, checked only
  where the truck is within reach of a physical object. Reversing unloaded, the evaluator drives the body-and-blades
  parts shape, which this tool does not rebuild: those ticks are listed as unevaluated for the arc;
- unevaluated ranges: control-tick gaps over 3 ticks, sample gaps over 0.15 s, the time after the last sample for the
  sample-based rows, and the unloaded reversing arc ticks.

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


def phase_timeline(s: dict) -> list:
    """[(from time, phase)] from the recorded transitions; the first phase is the first transition's 'from'."""
    tr = [t for t in s.get("transitions") or [] if isinstance(t, dict) and t.get("to")]
    if not tr:
        return []
    out = [(-math.inf, tr[0].get("from"))]
    out += [(float(t["time_s"]), t["to"]) for t in tr]
    return out


def phase_at(timeline: list, t: float) -> str | None:
    ph = None
    for t0, p in timeline:
        if t0 > t + 1e-12:
            break
        ph = p
    return ph


def truth_motion(C: np.ndarray, k: int, half: int = 2) -> tuple[float, float]:
    """(signed speed m/s, path curvature 1/m) at control row k from the truth rear poses of the neighbouring rows"""
    a, b = max(0, k - half), min(len(C) - 1, k + half)
    if b <= a:
        return 0.0, 0.0
    dx, dy = C[b, 4] - C[a, 4], C[b, 5] - C[a, 5]
    dt = C[b, 0] - C[a, 0]
    ds = math.hypot(dx, dy)
    yaw = C[k, 6]
    sign = 1.0 if dx * math.cos(yaw) + dy * math.sin(yaw) >= 0 else -1.0
    dyaw = math.atan2(math.sin(C[b, 6] - C[a, 6]), math.cos(C[b, 6] - C[a, 6]))
    v = sign * ds / dt if dt > 0 else 0.0
    kappa = dyaw / (sign * ds) if ds > 1e-4 else 0.0
    return float(v), float(kappa)


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
    timeline = phase_timeline(s)
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
    overlaps, arc_hits, arc_unevaluated = [], [], []

    def overlap(i, t, ph, part, step):
        if not overlaps or overlaps[-1]["object"] != i or overlaps[-1]["part"] != part or t - overlaps[-1]["to_s"] > 2 * step:
            overlaps.append({"object": i, "part": part, "from_s": t, "to_s": t, "phase": ph})
        else:
            overlaps[-1]["to_s"] = t

    def note(key, d, t, obj, ph):
        if d < rows[key]["min_m"]:
            rows[key].update(min_m=float(d), t=float(t), object=obj, phase=ph)

    if C is not None and len(C):
        dt = float(np.median(np.diff(C[:, 0]))) if len(C) > 1 else 1 / 120
        out["control_tick_s"] = dt
        out["unevaluated"] += [{"control_gap_s": gp} for gp in _gaps(C[:, 0], dt)]
        t0 = float(samples[0]["time_s"])
        for k, row in enumerate(C):
            t = float(row[0])
            if t < t0:
                continue
            ph = phase_at(timeline, t) if timeline else None
            loaded = ph in LOADED_PHASES
            shape = loaded_fp if loaded else unloaded_fp
            pose = tuple(float(v) for v in row[4:7])
            dk, ok = _nearest(pose, shape, keepout, rows["truck_keepout"]["min_m"])
            dp, op = _nearest(pose, shape, physical, rows["truck_physical"]["min_m"])
            note("truck_keepout", dk, t, ok, ph)
            note("truck_physical", dp, t, op, ph)
            for i, r in (keepout if dk <= 0.0 else ()):  # the distance is 0 when they meet; the overlap test decides
                if shape_meets(r, shape, pose, 0.0):
                    overlap(i, t, ph, "truck", dt)
            v, kappa = truth_motion(C, k)
            if abs(v) <= 0.05:
                continue
            stop_len = stopping.distance_m(v)
            if dp > stop_len + envelope_m + 0.05:
                continue  # out of reach of every physical object on this tick's arc
            if not loaded and v < 0:
                if not arc_unevaluated or t - arc_unevaluated[-1][1] > 2 * dt:
                    arc_unevaluated.append([t, t])
                else:
                    arc_unevaluated[-1][1] = t
                continue
            poses = arc_poses(pose, kappa, 1 if v > 0 else -1, stop_len, 0.025)[0]
            hit = [j for j, r in physical if any(shape_meets(r, shape, tuple(p), envelope_m) for p in poses[1:])]
            if hit:
                arc_hits.append({"t": t, "phase": ph, "objects": hit, "speed_mps": v, "curvature_inv_m": kappa})
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
            for i, r in (keepout if dk <= 0.0 else ()):
                if shape_meets(r, pal_shape, pose, 0.0):
                    overlap(i, t, ph, "pallet", 0.1)
    st = np.array([float(x["time_s"]) for x in samples])
    out["sample_interval_s"] = float(np.median(np.diff(st))) if len(st) > 1 else None
    out["unevaluated"] += [{"sample_gap_s": [float(st[k]), float(st[k + 1])]}
                           for k in np.nonzero(np.diff(st) > 0.15)[0]] if len(st) > 1 else []
    out["unevaluated"] += [{"arc_unloaded_reversing_s": seg} for seg in arc_unevaluated]
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
