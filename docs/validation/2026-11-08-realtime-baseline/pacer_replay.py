"""Real-time plan 5.14 E1 (offline counterfactual, no Isaac): the recorded per-step work W_i of a live run's driving
period replayed through the real ``forklift_core.live.pacing.Pacer`` with other catch-up settings and with simple
transforms of the work, to rank what a change could buy before any Isaac run.

    python pacer_replay.py SNAP ANALYSIS_JSON TRACE_ROOT RUN [RUN ...]

The replay assumes each step's work does not depend on when it starts (no pacing feedback into the work); the
wake-up overshoot of a step that sleeps is its recorded eps, a step that does not sleep has none. The base case
(catch-up 0, work unchanged) must reproduce the measured ratio -- printed as the check.

Transforms of the render+scan steps (tick % 12 == 0; the work parts as tools/tick_trace_report.py splits them):
  split   the render's own cost (world.step above the plain steps' median, and the camera read) moved to the step six
          ticks later (a camera phase of 6, plan 5.12 D12b) -- other renders and their costs left as they are
  scanX   the scan processing (step_world after world.step) on scan steps cut to X of itself
  ctrlX   the control tick on every step cut to X of itself
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

SNAP = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(SNAP / "sim" / "isaac"))
sys.path.insert(0, str(SNAP / "src"))
from tick_trace import COLUMNS, FLAGS, work  # noqa: E402
from forklift_core.live.pacing import Pacer  # noqa: E402

DT = 1 / 120
C = {n: i for i, n in enumerate(COLUMNS)}
BIT = {n: 1 << i for i, n in enumerate(FLAGS)}


def load(trace_root: Path, run: dict):
    path = next(trace_root.rglob(f"{run['name']}/live/epoch_1/run/tick_trace.npz"))
    with np.load(path) as z:
        data = z["data"]
    t0, t1 = run["driving_sim_s"]
    sim = data[:, C["tick"]] * DT
    sim_next = np.r_[sim[1:], np.inf]
    w = work(data)
    S, q, d = data[:, C["S"]], data[:, C["q"]], data[:, C["d"]]
    interval = np.r_[S[1:] - S[:-1], np.nan]
    eps = S - (q + d)  # the start's own overshoot past its wait (Codex G3-0 review P2: S[i+1]-(q+d)[i+1] was one step late)
    K = (sim >= t0) & (sim_next <= t1) & np.isfinite(w) & np.isfinite(interval)
    nx = data[np.r_[np.arange(1, len(data)), len(data) - 1]]
    parts = {"to_step_end": data[:, C["step_end"]] - S,
             "world": data[:, C["world_end"]] - data[:, C["world_start"]],
             "step_rest": data[:, C["step_end"]] - data[:, C["world_end"]],
             "after": data[:, C["after_end"]] - data[:, C["step_end"]],
             "control": nx[:, C["ctrl_end"]] - nx[:, C["ctrl_start"]]}
    bits = np.nan_to_num(data[:, C["flags"]]).astype(int)
    rendered = (bits & BIT["rendered"]) != 0
    scan = (bits & BIT["scan"]) != 0
    idx = np.flatnonzero(K)
    return {"w": w[idx], "eps": np.nan_to_num(eps[idx]), "interval": interval[idx],
            "parts": {k: np.nan_to_num(v[idx]) for k, v in parts.items()},
            "rendered": rendered[idx], "scan": scan[idx]}


def replay(w, eps, catchup, to_step_end=None, with_ok=False):
    """The ratio over the replayed steps; with_ok: also the share of steps with ``ok`` after the first ``ok``.
    ``after_step`` is called where the runtime calls it -- after step_world, before the camera read and the next
    control tick (``to_step_end`` of each step, clipped to its W; Codex G1 review P2) -- and ``ok`` is read as the
    next control tick sees it."""
    p = Pacer(1.0, DT, catchup_steps=catchup)
    t = 0.0
    t_first = None
    oks = []
    for i, (wi, ei) in enumerate(zip(w, eps)):
        sleep = p.wait_s(t)
        t += sleep + (ei if sleep > 0 else 0.0)
        p.started(t)
        if t_first is None:
            t_first = t
        mid = wi if to_step_end is None else min(max(to_step_end[i], 0.0), wi)
        p.after_step(t + mid)
        oks.append(p.ok)
        t += wi
    ratio = len(w) * DT / (t - t_first)
    if not with_ok:
        return ratio
    oks = np.array(oks)
    first = int(np.argmax(oks)) if oks.any() else None
    return ratio, (None if first is None else float(oks[first:].mean()))


def transformed(r, split=False, scan=1.0, ctrl=1.0):
    """(W, to_step_end) after the transform: the time to step_world's end moves with every change that falls before
    it (the scan processing, the render's world.step share), not with the camera read or the control tick that come
    after it (Codex G1 round 2 P2)."""
    w = r["w"].copy()
    tse = r["parts"]["to_step_end"].copy()
    P = r["parts"]
    rs = r["rendered"] & r["scan"]
    if scan != 1.0:
        cut = np.where(r["scan"], (1 - scan) * P["step_rest"], 0.0)
        w -= cut
        tse -= cut
    if ctrl != 1.0:
        w -= (1 - ctrl) * P["control"]
    if split:
        plain_world = np.median(P["world"][~r["rendered"]])
        render = np.where(rs, np.maximum(P["world"] - plain_world, 0), 0.0)  # before step_world's end
        read = np.where(rs, P["after"], 0.0)  # the camera read, after it
        w -= render + read
        tse -= render
        w[6:] += render[:-6] + read[:-6]
        tse[6:] += render[:-6]
    return w, tse


def main() -> int:
    analysis = {r["name"]: r for r in json.loads(Path(sys.argv[2]).read_text())}
    root = Path(sys.argv[3])
    cases = [("base", {}), ("split", {"split": True}), ("scan0.5", {"scan": 0.5}), ("split+scan0.5",
             {"split": True, "scan": 0.5}), ("ctrl0.5", {"ctrl": 0.5}), ("split+scan0.5+ctrl0.5",
             {"split": True, "scan": 0.5, "ctrl": 0.5})]
    for name in sys.argv[4:]:
        run = analysis[name]
        r = load(root, run)
        measured = run["ratio"]
        check = replay(r["w"], r["eps"], 0)
        mean_w = float(np.mean(r["w"])) / DT
        print(f"{name}: measured {measured:.4f}, replay catch-up 0 {check:.4f} (check), mean W / h {mean_w:.3f}")
        for label, kw in cases:
            w, tse = transformed(r, **kw)
            row = " ".join(f"k{k}={replay(w, r['eps'], k):.3f}" for k in (0, 6, 12, 24, 120))
            ratio6, ok6 = replay(w, r["eps"], 6, tse, with_ok=True)
            row += f"  | k6 ok-share {ok6 if ok6 is None else round(ok6, 3)}"
            print(f"   {label:24s} mean W/h {np.mean(w) / DT:.3f}  {row}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
