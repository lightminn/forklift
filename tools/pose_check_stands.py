"""The two monitor candidates of the design revision, read from the shadow D records (plan
docs/plans/2026-11-06-unconfirmed-validation.md section 6 step 4; development data only).

- **window**: how far the correction moved the applied pose over the last L metres of odometry path, against today's
  cumulative Delta_int. A jump to a twin shows whatever the path before it; the drift the correction normally absorbs
  stays small.
- **stands**: the frozen relocalizer and the conservative check on the mission stands' scan sets (plan 1.5, a snapshot
  approximation: sets in a row, compared with the correction at the set's last scan), outcome by outcome -- renew (it
  would have set C_int again), revoke (confirmed but the correction disagrees past E_SLAM), no_evidence (the matcher or
  the check gave nothing) or not_estimable (no scan set: the stand ended first).

    python tools/pose_check_stands.py window SESSION... --out OUT
    python tools/pose_check_stands.py stands SESSION... --grid GRID.npz --e-slam M,RAD --out OUT
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "sim/isaac")]

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization import start_reloc as R  # noqa: E402
from forklift_core.localization.slam_pose import compose, invert  # noqa: E402

WINDOWS_M = (2.0, 5.0, 10.0, 20.0)
REAR_OFFSET_M = 0.34  # only the rows' metres: the base and the rear axle both, as pose_check.gap


def _gap(a, b, rear: float = REAR_OFFSET_M) -> tuple:
    ra, rb = R.rear_from_base(a, rear), R.rear_from_base(b, rear)
    return (max(math.hypot(a[0] - b[0], a[1] - b[1]), math.hypot(ra[0] - rb[0], ra[1] - rb[1])),
            abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))))


def window_changes(samples, window_m: float) -> list:
    """Each sample with window_m of path behind it: the move of the applied pose at this odometry between the
    correction then and now."""
    rows, path, s0, last = [], [], 0, None
    total = 0.0
    for s in samples:
        o = s.get("o")
        if o is None or s.get("applied") is None:
            continue
        if last is not None:
            total += math.hypot(o[0] - last[0], o[1] - last[1])
        last = o
        C = compose(tuple(s["applied"]), invert(tuple(o)))
        path.append((total, C, s["t"]))
        while s0 + 1 < len(path) and total - path[s0 + 1][0] >= window_m:
            s0 += 1
        if total - path[s0][0] < window_m:
            continue
        m, rad = _gap(compose(C, tuple(o)), compose(path[s0][1], tuple(o)))
        rows.append({"t": s["t"], "path_m": total, "m": m, "rad": rad})
    return rows


def _classify(pose, truth) -> str:
    m, rad = _gap(tuple(pose), tuple(truth), 0.0)
    return "success" if m <= 0.10 and rad <= 0.05 else ("wrong" if m > 0.30 or rad > 0.10 else "imprecise")


def rematch_stands(rec, scans, ctx, *, b, o0, e_slam) -> list:
    """One row a kept mission stand. ``scans``: the session's pose_check_stand_scans.npz; ``b`` at ``o0``: the estimate."""
    import pose_runtime as PR

    from forklift_core.localization import twin_check as T

    rows = []
    o_s = tuple(rec["start"]["o_scan"])
    hyps = rec.get("hyps") or []
    samples = rec.get("samples") or []
    for i, cap in enumerate(rec.get("stand_sets") or []):
        row = {"t": cap["t0"], "delta": cap.get("delta"), "sets": len(cap["sets"]), "sets_used": 0,
               "outcome": "not_estimable"}
        rows.append(row)
        rc = ctx.reloc_cfg
        relocalizer = R.StartRelocalizer(rc, time.monotonic)
        relocalizer.start()
        for k, st in enumerate(cap["sets"]):
            row["sets_used"] = k + 1
            row["outcome"] = "no_evidence"
            ranges = scans[f"s{i}_{k}"]
            sets = [((0.0, 0.0, 0.0), r, float(scans["angles"][0]), float(scans["angles"][1])) for r in ranges]
            rmin, rmax, mrange = (float(v) for v in scans["ranges_m"])
            points = G.stack_scans(sets, range_min_m=rmin, range_max_m=rmax, match_range_m=mrange)
            out = PR.run_job("relocalize", {"points": points, "deadline_at": relocalizer.deadline_at}, ctx)
            state = relocalizer.record(out["match"], out["decision"])
            if state == "rescan":
                continue
            if state != "chosen":
                break
            g = tuple(relocalizer.estimate.base)
            o_scan = tuple(st["o"])
            res = PR.run_job("rematch", {"phase": "confirmed", "points": points, "b": T.moved(tuple(b), tuple(o0), o_s),
                                         "hyps": hyps, "o0": o_s, "ot": o_scan, "decision": "confirmed", "g": g,
                                         "deadline_at": time.monotonic() + PR.DEADLINE_S}, ctx)
            at = st["stamps"][-1]
            near = min(samples, key=lambda s: abs(s["t"] - at))
            C = compose(tuple(near["applied"]), invert(tuple(near["o"])))
            row["agree_m"], row["agree_rad"] = _gap(compose(C, o_scan), g)
            row["truth_class"] = _classify(g, near["truth"])
            if res["outcome"] == "confirm":
                ok = row["agree_m"] <= e_slam[0] and row["agree_rad"] <= e_slam[1]
                row["outcome"] = "renew" if ok else "revoke"
            break
    return rows


def summary(by_session) -> dict:
    counts: dict = {}
    for rows in by_session.values():
        for r in rows:
            counts[r["outcome"]] = counts.get(r["outcome"], 0) + 1
    return {"sessions": len(by_session), "stands": sum(len(r) for r in by_session.values()), "outcomes": counts}


def _record(folder):
    return json.loads((folder / "epoch_1" / "run" / "result.json").read_text())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("mode", choices=("window", "stands"))
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--grid", type=Path)
    ap.add_argument("--e-slam", default="0.109,0.0137")
    args = ap.parse_args(argv)
    if args.mode == "window":
        out = {}
        for w in WINDOWS_M:
            per = {}
            for folder in args.sessions:
                rows = window_changes(_record(folder)["pose_check"]["samples"], w)
                if rows:
                    per[folder.name] = {"m": max(r["m"] for r in rows), "rad": max(r["rad"] for r in rows),
                                        "t": max(rows, key=lambda r: r["m"])["t"]}
            out[str(w)] = {"sessions": len(per), "m_max": max((v["m"] for v in per.values()), default=0.0),
                           "rad_max": max((v["rad"] for v in per.values()), default=0.0), "by_session": per}
    else:
        import pose_runtime as PR
        import reloc_phase

        from forklift_core.localization.slam_map import Grid

        cfg, reloc_cfg, _ = reloc_phase.frozen_settings(ROOT)
        ctx = PR.JobContext(Grid.from_npz(args.grid), cfg, reloc_cfg, PR.SHADOW_BOUNDS)
        e_slam = tuple(float(v) for v in args.e_slam.split(","))
        by = {}
        for folder in args.sessions:
            result = _record(folder)
            rec = result["pose_check"]
            npz = folder / "epoch_1" / "run" / "pose_check_stand_scans.npz"
            if rec.get("stand_sets") and npz.exists():
                est = tuple(result["start_reloc"]["estimate"])
                by[folder.name] = rematch_stands(rec, np.load(npz), ctx, b=est, o0=est, e_slam=e_slam)
                print(folder.name, summary({folder.name: by[folder.name]})["outcomes"], flush=True)
        out = {"summary": summary(by), "e_slam": e_slam, "rows": by}
    args.out.write_text(json.dumps(out, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
