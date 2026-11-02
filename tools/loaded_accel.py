#!/usr/bin/env python3
"""The horizontal acceleration a carried pallet sees in recorded Isaac runs (docs/plans/2026-11-01-contact-materials.md
B0 margin table): from the truth rear-axle pose at the control rate (``slam_control.npy`` of a batch run, the
``slam.control`` spool of a live run; columns 4-6 are the truth x, y, yaw), carried to the pallet's centre by rigid-body
kinematics -- the centre stands ``--pallet-offset-m`` ahead of the rear axle on the truck's axis, so its acceleration in
the truck frame is (a_long - omega^2 d, a_lat + alpha d) with a_lat = v omega.

The loaded window is from the lift transition to the lower transition in a batch run, and every sample labelled lift,
extract or transport in a live run. Speeds and accelerations are differentiated on the whole run and the window taken
after (a window cut first would smooth against the moving average's zero padding); the run's first and last 4 windows
are left out. Each run is reported at every ``--smooth-ticks`` (a moving average over that many control ticks, applied
to the position, the speed and the acceleration): max and p99 of the pallet-centre magnitude, and the time of the max.

  python tools/loaded_accel.py BATCH_DIR --pallet-offset-m 1.26 --smooth-ticks 12 3 --json OUT.json
"""

from __future__ import annotations

import argparse
import glob
import json
import math
from pathlib import Path

import numpy as np


def smooth(x: np.ndarray, n: int) -> np.ndarray:
    return x if n <= 1 else np.convolve(x, np.ones(n) / n, mode="same")


def pallet_acceleration(control: np.ndarray, d: float, n: int) -> tuple[np.ndarray, np.ndarray]:
    """(time, |a| at the pallet centre) over a whole run; rows of ``control`` are (t, ..., x, y, yaw) at columns 0, 4-6."""
    t, x, y = control[:, 0], smooth(control[:, 4], n), smooth(control[:, 5], n)
    th = np.unwrap(control[:, 6])
    dt = np.gradient(t)
    vx, vy = np.gradient(x) / dt, np.gradient(y) / dt
    v = smooth(vx * np.cos(th) + vy * np.sin(th), n)
    a_long = smooth(np.gradient(v) / dt, n)
    omega = smooth(np.gradient(smooth(th, n)) / dt, n)
    alpha = smooth(np.gradient(omega) / dt, n)
    ax = a_long - omega ** 2 * d
    ay = v * omega + alpha * d
    return t, np.hypot(ax, ay)


def window_stats(t: np.ndarray, a: np.ndarray, t0: float, t1: float, n: int) -> dict | None:
    keep = np.zeros(len(t), bool)
    keep[4 * max(n, 1):len(t) - 4 * max(n, 1)] = True
    sel = keep & (t >= t0) & (t <= t1)
    if sel.sum() < 3 * max(n, 1):
        return None
    i = int(np.argmax(a[sel]))
    return {"max": float(a[sel].max()), "p99": float(np.percentile(a[sel], 99)), "t_of_max": float(t[sel][i]),
            "loaded_s": float(t[sel][-1] - t[sel][0])}


def runs(batch: Path):
    """(name, control rows, loaded windows) of every run in ``batch``: batch runs ``<name>/run``, live runs
    ``<name>/live/epoch_1/run``."""
    for run in sorted(glob.glob(str(batch / "*/run"))):
        result = json.loads(Path(run, "result.json").read_text())
        to = {x.get("to"): x.get("time_s") for x in result.get("transitions") or []}
        if "lift" in to:
            yield Path(run).parent.name, np.load(Path(run, "slam_control.npy")), [(to["lift"], to.get("lower", math.inf))]
    for run in sorted(glob.glob(str(batch / "*/live/epoch_1/run"))):
        from forklift_core.live.spool import load_list

        control = np.asarray(load_list(run + "/spool", "slam.control", allow_incomplete=True), float).reshape(-1, 7)
        samples = load_list(run + "/spool", "state.samples", allow_incomplete=True)
        ts = [r["time_s"] for r in samples if r.get("phase") in ("lift", "extract", "transport")]
        if ts:
            yield Path(run).parents[2].name, control, [(min(ts), max(ts))]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("batch", type=Path)
    ap.add_argument("--pallet-offset-m", type=float, required=True,
                    help="the pallet centre ahead of the rear axle (axle-to-fork-tip - insertion + depth/2)")
    ap.add_argument("--smooth-ticks", type=int, nargs="+", default=[12, 3])
    ap.add_argument("--json", type=Path)
    a = ap.parse_args(argv)
    rows = []
    for name, control, windows in runs(a.batch):
        row = {"run": name}
        for n in a.smooth_ticks:
            t, acc = pallet_acceleration(control, a.pallet_offset_m, n)
            row[f"smooth_{n}"] = window_stats(t, acc, *windows[0], n)
        rows.append(row)
    for n in a.smooth_ticks:
        for kind, sel in (("batch", lambda r: "_live" not in r["run"]), ("live", lambda r: "_live" in r["run"])):
            for arm in sorted({r["run"].split("_")[1] for r in rows}):
                vals = [r[f"smooth_{n}"] for r in rows if sel(r) and r["run"].split("_")[1] == arm and r[f"smooth_{n}"]]
                if vals:
                    best = max(vals, key=lambda v: v["max"])
                    print(f"smooth {n} ticks  arm {arm} {kind:5s} runs {len(vals):2d}  max {best['max']:.3f}  "
                          f"p99(max over runs) {max(v['p99'] for v in vals):.3f} m/s^2")
    if a.json:
        a.json.write_text(json.dumps({"pallet_offset_m": a.pallet_offset_m, "rows": rows}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
