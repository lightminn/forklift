"""Offline (docs/plans/2026-11-12-teleop-smooth-confirmation.md 1): the latency batch B's sessions (f 1.0, catch-up 6;
ws1 test_runs/lat/b_297e68d8, the 12 kept) replayed step by step -- each step's recorded work W (tick_trace.work) and
its wake overshoot eps, every step (the runner calls sleep() even for a zero wait: Codex P2-2) -- through the real
Pacer at a declared factor f and catch-up k. Reported over the TELEOP stretches (consecutive TELEOP tick records):
the ratio, the pacing ok share, the longest not-ok stretch, and per direct drive hold the ok share (the output share
while the hold is held: the arbiter refuses when pacing is not ok) and the time to the first ok.

Assumes a step's work does not depend on f or on pacing -- not so: while pacing refuses, the teleop permit is not
computed (teleop_drive.py 219), so a run whose pacing holds does more work per step (Codex P2-1). A support, not a
guarantee. Check row: k6 f1.0 reproduces the measured ratio, ok and hold shares.

    python factor_replay.py REPO DATA_ROOT [OUT_JSON]
"""
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(sys.argv[1])
sys.path.insert(0, str(REPO / "sim" / "isaac"))
sys.path.insert(0, str(REPO / "src"))
from tick_trace import COLUMNS, work  # noqa: E402
from forklift_core.live.pacing import Pacer  # noqa: E402

DT = 1 / 120
C = {n: i for i, n in enumerate(COLUMNS)}
KEPT = ["lat_b01_d0_r1", "lat_b02_d25_r1", "lat_b03_d50_r1", "lat_b04_d100_r1", "lat_b05_d500_r1", "lat_b06_d500_r2",
        "lat_b07_d100_r2", "lat_b08_d50_r2", "lat_b09_d25_r2", "lat_b10_d0_r2", "lat_b11_d0_r3", "lat_b12_d25_r3"]
DRIVE = ("forward", "reverse", "left", "right")


def session(root: Path, name: str):
    run = root / name / "s" / "live" / "epoch_1" / "run"
    with np.load(run / "tick_trace.npz") as z:
        data = z["data"]
    data = data[np.isfinite(data[:, C["S"]])]
    spans, prev = [], None  # (mode, first tick_t, last tick_t) of consecutive tick records
    with open(run / "live_log.jsonl") as fh:
        for line in fh:
            r = json.loads(line)
            if "tick_t" not in r or "mode" not in r:
                continue
            if r["mode"] != prev:
                spans.append([r["mode"], r["tick_t"], r["tick_t"]])
                prev = r["mode"]
            else:
                spans[-1][2] = r["tick_t"]
    t0 = [(a, b) for m, a, b in spans if m == "TELEOP"]
    t1 = None
    probe = json.loads((root / name / "s" / "probe.json").read_text())
    holds = [h for h in probe["confirm"]["holds"] if h["label"] in DRIVE]
    return data, t0, t1, holds


def replay(data, t0, t1, holds, f, k):
    sim = data[:, C["tick"]] * DT
    w = work(data)
    S, q, d = data[:, C["S"]], data[:, C["q"]], data[:, C["d"]]
    eps = np.nan_to_num(S - (q + d))
    sel = np.flatnonzero(np.isfinite(w))  # the whole session, so the pacer's state carries into TELEOP
    # hold windows in sim time, from the recorded wall clock S
    hs = [(lab, float(np.interp(a, S[sel], sim[sel])), float(np.interp(b, S[sel], sim[sel]))) for lab, a, b in
          ((h["label"], h["start_mono"], h["end_mono"]) for h in holds)]
    to_end = np.nan_to_num(data[:, C["step_end"]] - S)
    p = Pacer(f, DT, catchup_steps=k)
    t = 0.0
    rows = []  # (sim, wall start, wall dur, ok)
    for i in sel:
        sleep = p.wait_s(t)
        t += sleep + eps[i]  # the runner sleeps (sleep(0) too) every step: its overshoot always (Codex P2-2)
        p.started(t)
        p.after_step(t + min(max(to_end[i], 0.0), w[i]))
        rows.append((sim[i], t, p.ok))
        t += w[i]
    rows = np.array(rows, dtype=float)
    s, wall, ok = rows[:, 0], rows[:, 1], rows[:, 2] > 0
    dur = np.r_[np.diff(wall), w[sel[-1]]]
    T = np.zeros(len(s), bool)
    for a, b in t0:
        T |= (s >= a - 1e-9) & (s <= b + 1e-9)
    ratio = (T.sum() * DT) / dur[T].sum()
    okshare = float((dur[T] * ok[T]).sum() / dur[T].sum())
    # the longest wall stretch of not ok inside TELEOP
    longest, run_ = 0.0, 0.0
    for di, oi in zip(dur[T], ok[T]):
        run_ = 0.0 if oi else run_ + di
        longest = max(longest, run_)
    per = {}
    for lab, a, b in hs:
        m = (s >= a) & (s < b)
        if not m.any():
            continue
        share = float((dur[m] * ok[m]).sum() / dur[m].sum())
        first = np.flatnonzero(ok[m])
        first_ms = None if not len(first) else float((wall[m][first[0]] - wall[m][0]) * 1e3)
        per.setdefault(lab, []).append((share, first_ms))
    return ratio, okshare, longest, per


def main():
    root = Path(sys.argv[2])
    sessions = {n: session(root, n) for n in KEPT}
    out = {}
    for k in (6, 0):
        for f in ((1.0, 0.95, 0.9, 0.85) if k == 6 else (0.5, 0.45, 0.4, 0.35)):
            res = [replay(*sessions[n], f, k) for n in KEPT]
            per = {}
            for r in res:
                for lab, v in r[3].items():
                    per.setdefault(lab, []).extend(v)
            row = {"ratio": float(np.median([r[0] for r in res])), "ok": float(np.median([r[1] for r in res])),
                   "ok_min": float(min(r[1] for r in res)),
                   "longest_not_ok_s_max": float(max(r[2] for r in res)),
                   "hold_share_median": {lab: round(float(np.median([x[0] for x in v])), 3) for lab, v in per.items()},
                   "hold_share_min": {lab: round(float(min(x[0] for x in v)), 3) for lab, v in per.items()},
                   "first_output_ms_max": {lab: (None if any(x[1] is None for x in v) else round(max(x[1] for x in v)))
                                           for lab, v in per.items()}}
            out[f"k{k} f{f}"] = row
            print(f"k{k} f{f}: ratio {row['ratio']:.3f} ok {row['ok']:.3f} (min {row['ok_min']:.3f}) longest not-ok "
                  f"{row['longest_not_ok_s_max']:.2f}s holds {row['hold_share_median']} min {row['hold_share_min']} "
                  f"first {row['first_output_ms_max']}", flush=True)
    Path(sys.argv[3] if len(sys.argv) > 3 else "factor_replay.json").write_text(json.dumps(out, indent=1))


main()
