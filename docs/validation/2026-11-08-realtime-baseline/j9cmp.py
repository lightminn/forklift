import json, glob, numpy as np
from forklift_core.live.spool import iter_chunks, load_list
def run_dir(name):
    return glob.glob(f"*/{name}/live/epoch_1/run")[0]
def info(name):
    run = run_dir(name)
    res = json.load(open(run + "/missions/m1/result.json"))["result"]
    st = json.load(open(run + "/missions/m1/records.json"))["state"]
    cs = (st.get("cusp_stalls") or [{}])[-1]
    ctl = []
    for ch in iter_chunks(run + "/spool", "slam.control"):
        ctl.extend(ch)
    c = np.array(ctl, float).reshape(-1, 7)
    recs = load_list(run + "/spool", "slam.records")
    ph = {}
    for r in recs:
        ph.setdefault(r.get("phase"), [r["stamp_s"], r["stamp_s"]])[1] = r["stamp_s"]
    t_tr0 = ph.get("approach", [0])[0]; t_tr1 = ph.get("lower", [c[-1, 0]])[0]
    m = (c[:, 0] >= t_tr0) & (c[:, 0] <= t_tr1)
    ey = c[:, 2] - c[:, 5]
    e = np.hypot(c[:, 1] - c[:, 4], ey)
    proc = [r for r in recs if r.get("status") == "processed"]
    applied = sorted(r["applied_tick_s"] for r in proc if r.get("applied_tick_s") is not None)
    # share of the transport-window error change on ticks where a processed reply was applied
    tick_applied = np.zeros(len(c), bool)
    idx = np.searchsorted(c[:, 0], applied)
    tick_applied[np.clip(idx, 0, len(c) - 1)] = True
    dey = np.diff(ey[m]); ta = tick_applied[m][1:]
    delay = np.array([r["applied_tick_s"] - r["stamp_s"] for r in proc if r.get("applied_tick_s") is not None])
    ids = set(r["scan_id"] for r in proc)
    return {"outcome": res["outcome"], "reason": res["reason"], "stall_leg": cs.get("leg_index"), "stall_phase": cs.get("phase"),
            "stall_est_err": cs.get("position_error_m"),
            "stall_truth_err": None,
            "transport_err_p50": float(np.median(e[m])), "transport_dey": float(ey[m][-1] - ey[m][0]),
            "dey_applied_ticks": float(dey[ta].sum()), "dey_other": float(dey[~ta].sum()),
            "delay_p50": float(np.percentile(delay, 50)), "delay_p90": float(np.percentile(delay, 90)), "delay_max": float(delay.max()),
            "processed": len(ids), "_ids": ids, "end_err": float(e[-1])}
names = ["j9_f03_1_off", "j9_f03_2_off", "j4_i2_1_off", "j7_i3_1_off", "j8_i1b_3_off"]
I = {n: info(n) for n in names}
for n in names:
    d = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in I[n].items() if not k.startswith("_")}
    print(n, d)
for a in names[:2]:
    for b in names[2:]:
        A, B = I[a]["_ids"], I[b]["_ids"]
        print("scan-id overlap", a, b, round(len(A & B) / len(A | B), 3))
for n in names:
    r = json.load(open(run_dir(n) + "/result.json"))["arguments"]
    print(n, "live_factor", r.get("live_factor"))
