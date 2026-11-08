"""Independent recompute (plan 5.18/5.19 review 1) from raw live files only, no project tools.

usage: recompute_g2g3.py RUN_DIR [RUN_DIR ...]
  RUN_DIR holds live/epoch_1/run/tick_trace.npz (and live_log.jsonl when present).
Prints per run: the trace's loss state and span, control ticks over 50/100/200 ms, the start gaps over 0.25 s by two
clocks (S = pacer start after the wait, the 5.15 M3 / rtb_analyze definition; world_start = after the stop gate),
counted from the first control tick (whole, no exclusion) with each gap's control-tick content, gc.gen2 status lines,
and the low-clearance timer normalised by the session's simulated span.
"""
import glob, json, sys
import numpy as np

DT = 1 / 120
for d in sys.argv[1:]:
    tt = glob.glob(d + "/**/tick_trace.npz", recursive=True)
    ll = glob.glob(d + "/**/live_log.jsonl", recursive=True)
    z = np.load(tt[0]); cols = list(z["columns"]); D = z["data"]
    c = lambda n: D[:, cols.index(n)]
    S, ws, cs, ce = c("S"), c("world_start"), c("ctrl_start"), c("ctrl_end")
    dur = np.nan_to_num(np.where(np.isfinite(cs), ce - cs, 0.0))
    first = int(np.argmax(np.isfinite(cs)))
    t0 = ws[first]
    print(f"{d.rstrip('/').split('/')[-1]}: rows {len(D)} (sim {len(D)*DT:.0f}s) first ctrl row {first}"
          f" | ctrl >50 {int((dur>.05).sum())} >100 {int((dur>.1).sum())} >200 {int((dur>.2).sum())} max {np.nanmax(dur)*1e3:.0f}ms")
    for name, clk in (("S", S), ("world_start", ws)):
        g = np.diff(clk[first:]); big = np.where(g > 0.25)[0] + first
        items = []
        for j in big:
            inside = (cs >= clk[j]) & (cs < clk[j + 1])
            items.append(f"t+{clk[j]-t0:.1f}s {int((clk[j+1]-clk[j])*1e3)}ms ctrl {int(dur[inside].sum()*1e3)}ms")
        near = int(((g > 0.20) & (g <= 0.25)).sum())
        print(f"   gaps>0.25 by {name}: {len(big)} {items}  (0.20-0.25: {near})")
    gen2, lc, nstat, ts = [], 0.0, 0, []
    for line in (open(ll[0]) if ll else []):
        r = json.loads(line)
        if r.get("status") and "timers" in r:
            nstat += 1; tm = r["timers"]
            if "gc.gen2" in tm: gen2.append((nstat, round(tm["gc.gen2"][2], 3)))
            lc += sum(v[0] for k, v in tm.items() if "low_clearance" in k and "<" not in k)
            ts.append(r["tick_t"])
    if ts:
        print(f"   status lines {nstat} gc.gen2 {gen2} | low_clearance {lc:.1f}s / sim {ts[-1]-ts[0]:.0f}s = {lc/(ts[-1]-ts[0]):.3f}")
