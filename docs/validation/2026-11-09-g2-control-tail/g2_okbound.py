"""G2 upper bound (agy review 7): the A6P runs' recorded work replayed through the real Pacer (catch-up 6) with every
control tick over 50 ms cut to the run's median control tick -- the ok share G2 could reach if the tail were gone."""
import json, sys
from pathlib import Path
import numpy as np
SNAP = Path(sys.argv[1]); sys.path.insert(0, str(SNAP / "sim" / "isaac")); sys.path.insert(0, str(SNAP / "src"))
from tick_trace import COLUMNS, work
from forklift_core.live.pacing import Pacer
DT = 1 / 120; C = {n: i for i, n in enumerate(COLUMNS)}
G = {r["name"]: r for r in json.load(open(sys.argv[2]))["runs"]}
def replay(w, eps, tse, k=6):
    p = Pacer(1.0, DT, catchup_steps=k); t = 0.0; t0 = None; oks = []
    for wi, ei, si in zip(w, eps, tse):
        s = p.wait_s(t); t += s + (ei if s > 0 else 0.0); p.started(t); t0 = t if t0 is None else t0
        p.after_step(t + min(max(si, 0.0), wi)); oks.append(p.ok); t += wi
    oks = np.array(oks); f = int(np.argmax(oks)) if oks.any() else None
    return len(w) * DT / (t - t0), (None if f is None else float(oks[f:].mean()))
for name in sys.argv[3:]:
    r = G[name]
    with np.load(Path(sys.argv[4 if False else 1]).parent / "x" if False else Path("/home/light/.claude/jobs/53111190/tmp/g1raw/g1") / name / "live/epoch_1/run/tick_trace.npz") as z:
        d = z["data"]
    t0, t1 = r["driving_sim_s"]; sim = d[:, C["tick"]] * DT; sn = np.r_[sim[1:], np.inf]
    S, q, dd = d[:, C["S"]], d[:, C["q"]], d[:, C["d"]]
    w = work(d); itv = np.r_[S[1:] - S[:-1], np.nan]; eps = np.nan_to_num(np.r_[S[1:] - (q[1:] + dd[1:]), np.nan])
    K = (sim >= t0) & (sn <= t1) & np.isfinite(w) & np.isfinite(itv)
    nx = d[np.r_[np.arange(1, len(d)), len(d) - 1]]
    ctrl = np.nan_to_num(nx[:, C["ctrl_end"]] - nx[:, C["ctrl_start"]])
    tse = d[:, C["step_end"]] - S
    idx = np.flatnonzero(K)
    base = replay(w[idx], eps[idx], tse[idx])
    med = float(np.median(ctrl[idx]))
    cut = np.where(ctrl > 0.05, ctrl - med, 0.0)
    flat = replay((w - cut)[idx], eps[idx], tse[idx])
    cut2 = np.where(ctrl > 0.02, ctrl - med, 0.0)
    flat2 = replay((w - cut2)[idx], eps[idx], tse[idx])
    print(f"{name}: measured ok {r['M2']['ok_share_auto']:.3f}; replay ratio/ok {base[0]:.4f}/{base[1]:.3f}; "
          f"control >50 ms flattened {flat[0]:.4f}/{flat[1]:.3f} ({int((ctrl[idx]>0.05).sum())} ticks); "
          f">20 ms flattened {flat2[0]:.4f}/{flat2[1]:.3f} ({int((ctrl[idx]>0.02).sum())} ticks)")
