"""G2 input: which calls make the long control ticks (live runs' tick trace + status-line timers)."""
import json, sys, collections
from pathlib import Path
import numpy as np
sys.path.insert(0, sys.argv[1] + "/sim/isaac")
from tick_trace import COLUMNS
C = {n: i for i, n in enumerate(COLUMNS)}
for run in sys.argv[2:]:
    run = Path(run)
    with np.load(run / "tick_trace.npz") as z:
        d = z["data"]
    ctrl = d[:, C["ctrl_end"]] - d[:, C["ctrl_start"]]
    sim_s = (np.nanmax(d[:, C["tick"]]) - np.nanmin(d[:, C["tick"]])) / 120
    timers = []
    for line in open(run / "live_log.jsonl"):
        r = json.loads(line)
        if isinstance(r.get("timers"), dict) and r.get("mono"):
            timers.append(r)
    monos = np.array([t["mono"] for t in timers])
    long_idx = np.flatnonzero(np.nan_to_num(ctrl) > 0.05)
    attrib = collections.Counter(); attrib_t = collections.Counter()
    for i in long_idx:
        m = d[i, C["ctrl_start"]]
        k = int(np.searchsorted(monos, m))
        if k >= len(timers):
            continue
        tm = timers[k]["timers"]
        cands = {name: v[2] for name, v in tm.items() if name.startswith(("core.", "plan.", "layer.", "grid.", "slam.", "near.", "far.", "recog")) and v[2] is not None and v[2] >= 0.5 * ctrl[i]}
        top = [n for n in cands if n.startswith("core.")]
        name = max(top, key=lambda n: cands[n]) if top else (max(cands, key=cands.get) if cands else "unattributed")
        attrib[name] += 1; attrib_t[name] += ctrl[i]
    print(f"== {run.parent.parent.parent.name if 'rtb' in str(run) else run.parent.parent.parent.name}: sim {sim_s:.0f} s, ctrl ticks >50 ms {len(long_idx)} ({len(long_idx)/sim_s*60:.1f}/min), >100 ms {int(np.sum(np.nan_to_num(ctrl)>0.1))}, >200 ms {int(np.sum(np.nan_to_num(ctrl)>0.2))}, total long ctrl {np.nansum(ctrl[long_idx]):.2f} s, all ctrl {np.nansum(ctrl):.1f} s")
    for n, c in attrib.most_common(12):
        print(f"   {n:40s} {c:4d} ticks  {attrib_t[n]:.2f} s")
    agg = collections.defaultdict(lambda: [0.0, 0, 0.0])
    for t in timers:
        for name, v in t["timers"].items():
            if name.startswith("core."):
                agg[name][0] += v[0]; agg[name][1] += v[1]; agg[name][2] = max(agg[name][2], v[2] or 0)
    print("   core.* totals (wall s / sim s, calls / sim s, max ms):")
    for name, (w, n, mx) in sorted(agg.items(), key=lambda x: -x[1][0])[:14]:
        print(f"     {name:38s} {w/sim_s:.4f}  {n/sim_s:7.1f}  {mx*1000:7.1f}")
