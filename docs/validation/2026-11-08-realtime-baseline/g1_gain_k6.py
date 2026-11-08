"""Removal gains under the real Pacer with catch-up 6 for the G1 A6/A6P runs (Codex G1 result review P2-1)."""
import json, sys
from pathlib import Path
import numpy as np
SNAP = Path(sys.argv[1]); sys.path.insert(0, str(SNAP / "sim" / "isaac")); sys.path.insert(0, str(SNAP / "src"))
from tick_trace import COLUMNS, work
from forklift_core.live.pacing import Pacer
DT = 1 / 120; C = {n: i for i, n in enumerate(COLUMNS)}
G = json.load(open(sys.argv[2]))["runs"]; root = Path(sys.argv[3])
def replay(w, eps, k):
    p = Pacer(1.0, DT, catchup_steps=k); t = 0.0; t0 = None
    for wi, ei in zip(w, eps):
        s = p.wait_s(t); t += s + (ei if s > 0 else 0.0); p.started(t); t0 = t if t0 is None else t0; t += wi; p.after_step(t)
    return t - t0
for r in G:
    if r["arm"] not in ("A6", "A6P") or not r.get("driving_sim_s"):
        continue
    with np.load(root / "g1" / r["name"] / "live/epoch_1/run/tick_trace.npz") as z:
        d = z["data"]
    t0, t1 = r["driving_sim_s"]; sim = d[:, C["tick"]] * DT; sn = np.r_[sim[1:], np.inf]
    S, q, dd = d[:, C["S"]], d[:, C["q"]], d[:, C["d"]]
    w = work(d); itv = np.r_[S[1:] - S[:-1], np.nan]; eps = np.nan_to_num(np.r_[S[1:] - (q[1:] + dd[1:]), np.nan])
    K = (sim >= t0) & (sn <= t1) & np.isfinite(w) & np.isfinite(itv)
    nx = d[np.r_[np.arange(1, len(d)), len(d) - 1]]
    parts = {"world": d[:, C["world_end"]] - d[:, C["world_start"]], "step_rest": d[:, C["step_end"]] - d[:, C["world_end"]],
             "after": d[:, C["after_end"]] - d[:, C["step_end"]], "control": nx[:, C["ctrl_end"]] - nx[:, C["ctrl_start"]]}
    idx = np.flatnonzero(K); W = w[idx]; E = eps[idx]; n = len(idx); sim_s = n * DT
    base = replay(W, E, 6)
    out = {"replay_ratio": sim_s / base}
    for p, v in parts.items():
        out["G6_" + p] = (base - replay(W - np.nan_to_num(v[idx]), E, 6)) / sim_s
    print(r["name"], {k: round(x, 4) for k, x in out.items()})
