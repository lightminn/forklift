"""Latency test A, record diagnosis (no new runs): per run, the evidence_stale share of each phase's control ticks
and where in the 12-tick scan cycle the truth deceleration falls (the layer install k ticks after the scan shifts the
stale window by k if the 1e-0 mechanism holds).

    python lat_diag_phase.py A_OUT [--json OUT.json]
"""

import argparse
import glob
import json
import os
from pathlib import Path

import numpy as np


def phase_labels(t: np.ndarray, transitions: list, final_phase) -> np.ndarray:
    """Each control tick's phase: the first transition's ``from`` before it (the initial observe), each ``to`` after,
    and the run's final phase when there is no transition at all."""
    ph = np.array([final_phase] * len(t), dtype=object)
    marks = sorted((float(x["time_s"]), x.get("from"), x.get("to")) for x in transitions if "to" in x)
    if marks:
        ph[t < marks[0][0]] = marks[0][1]
        for i, (ts, _, name) in enumerate(marks):
            te = marks[i + 1][0] if i + 1 < len(marks) else np.inf
            ph[(t >= ts) & (t < te)] = name
    return ph


def phase_profile(C: np.ndarray, transitions: list, final_phase, phases=("transport", "return_home", "observe")) -> dict:
    """Truth speed between consecutive control rows (at their mid time) and the acceleration between consecutive
    speeds, set at the row between them (no one-tick lead). For rows in the given phases moving over 0.03 m/s: the mean
    acceleration by physics-tick phase mod 12 (scan ticks are multiples of 12), and over contiguous moving stretches of
    at least 2 s the share of the speed's power (0.5-30 Hz) in the 10 Hz bin (a stop-go at the scan period)."""
    t = C[:, 0]
    dt = np.diff(t)
    v = np.hypot(np.diff(C[:, 4]), np.diff(C[:, 5])) / np.maximum(dt, 1e-9)  # v[i]: rows i -> i+1
    acc = np.full(len(t), np.nan)
    acc[1:-1] = np.diff(v) / np.maximum((dt[:-1] + dt[1:]) / 2, 1e-9)  # at row i: v[i-1] -> v[i]
    tick = np.rint(t * 120).astype(int)
    ph = phase_labels(t, transitions, final_phase)
    vrow = np.r_[np.nan, v]  # the speed arriving at row i
    sel = np.isin(ph, phases) & (vrow > 0.03) & np.isfinite(acc)
    out = {"ticks": int(sel.sum())}
    if sel.sum() >= 120:
        m, a = tick[sel] % 12, acc[sel]
        prof = [float(a[m == k].mean()) if (m == k).any() else np.nan for k in range(12)]
        out["mean_acc_by_mod12"] = [round(x, 4) for x in prof]
        out["min_mod12"] = int(np.nanargmin(prof))
    # 10 Hz power share over contiguous moving stretches on the original tick axis
    moving = np.isin(ph[1:], phases) & (v > 0.03) & (np.abs(dt - 1 / 120) < 1e-4)
    shares, start = [], None
    for i in range(len(moving) + 1):
        if i < len(moving) and moving[i]:
            start = i if start is None else start
        elif start is not None:
            seg = v[start:i]
            if len(seg) >= 240:
                x = seg - seg.mean()
                f = np.fft.rfftfreq(len(x), 1 / 120)
                pw = np.abs(np.fft.rfft(x)) ** 2
                band = (f >= 0.5) & (f <= 30)
                peak = (f >= 9.5) & (f <= 10.5)
                if pw[band].sum() > 0:
                    shares.append((float(pw[peak].sum() / pw[band].sum()), len(seg)))
            start = None
    if shares:
        w = sum(n for _, n in shares)
        out["power10hz_share"] = round(sum(s * n for s, n in shares) / w, 3)
        out["stretches"] = len(shares)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("a_out", type=Path)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args(argv)
    rows = {}
    for d in sorted(glob.glob(str(a.a_out / "lat_*_s[012]"))):
        name = os.path.basename(d)
        run = Path(d) / "run"
        try:
            r = json.loads((run / "result.json").read_text())
            C = np.load(run / "slam_control.npy")
        except (OSError, ValueError):
            continue
        ol = r.get("obstacle_layer") or {}
        sr, ticks = ol.get("slowed_reasons") or {}, ol.get("control_ticks") or {}
        stale = {k.split(":")[0]: v for k, v in sr.items() if k.endswith(":evidence_stale")}
        share = {p: round(stale.get(p, 0) / ticks[p], 3) for p in ticks if ticks.get(p)}
        rows[name] = {"done": r.get("success"), "reason": r.get("failure_reason"),
                      "stale_ticks": stale, "stale_share": share, **phase_profile(C, r.get("transitions") or [], r.get("phase"))}
    if a.json:
        a.json.write_text(json.dumps(rows, indent=1) + "\n")
    for n, x in rows.items():
        print(n, x.get("min_mod12"), x.get("power10hz_share"), x.get("ticks"), x.get("stale_share"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
