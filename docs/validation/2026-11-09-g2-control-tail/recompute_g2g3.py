# Independent recompute (review 1) from raw files only, no project tools.
import numpy as np, json, glob, os
runs = {
 'A0_b1': 'g1raw/g1/g1_02_b1_A0', 'A6P_b1': 'g1raw/g1/g1_04_b1_A6P',
 'A6P_b2': 'g1raw/g1/g1_06_b2_A6P', 'A6P_b3': 'g1raw/g1/g1_08_b3_A6P', 'A0_b3':'g1raw/g1/g1_09_b3_A0',
 'riib_line': 'g2raw/riibv_21ce53df/s/ab_line_m_s0_A', 'riib_rand': 'g2raw/riibv_21ce53df/s/guard_random_m'}
for k, d in runs.items():
    tt = glob.glob(d + '/**/tick_trace.npz', recursive=True)
    ll = glob.glob(d + '/**/live_log.jsonl', recursive=True)
    if not tt: print(k, 'no trace'); continue
    z = np.load(tt[0]); cols = list(z['columns']); D = z['data']
    c = lambda n: D[:, cols.index(n)]
    cs, ce = c('ctrl_start'), c('ctrl_end'); m = np.isfinite(cs) & np.isfinite(ce) & (ce > cs)
    dur = (ce - cs)[m]
    ws = c('world_start'); span = np.nanmax(ws) - np.nanmin(ws)
    # status lines: gen2 occurrences, low_clearance totals, sim span
    gen2 = []; lc = 0.0; t0 = t1 = 0.0; nstat = 0
    if not ll: ll = ["/dev/null"]
    for line in open(ll[0]):
        r = json.loads(line)
        if r.get('status') and 'timers' in r:
            nstat += 1; tm = r['timers']
            if 'gc.gen2' in tm: gen2.append((nstat, round(tm['gc.gen2'][2], 3)))
            for kk, v in tm.items():
                if 'low_clearance' in kk and '<' not in kk: lc += v[0]
            t = r.get('tick_t'); t0 = t if nstat == 1 else t0; t1 = t
    print(f"{k}: trace wall span {span:.0f}s ticks {m.sum()} >50 {int((dur>.05).sum())} >100 {int((dur>.1).sum())} >200 {int((dur>.2).sum())} max {dur.max()*1e3:.0f}ms | status {nstat} gen2 {gen2} | sim {t0:.0f}-{t1:.0f}s low_clear {lc:.1f}s -> {lc/max(t1-t0,1e-9):.3f}/sim s")
