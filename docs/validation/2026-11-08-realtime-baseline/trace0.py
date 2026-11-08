import json
from forklift_core.live.spool import load_list, read_live_log
def ratio(name):
    out = f"g1/{name}"; run = f"{out}/live/epoch_1/run"
    recs = load_list(run + "/spool", "slam.records")
    t0 = min(r["stamp_s"] for r in recs if r.get("mode") == "tracking")
    t1 = json.load(open(out + "/probe.json"))["missions_run"][0]["result"]["ended_s"]
    st = [(r["tick_t"], r["mono"]) for r in read_live_log(run) if "pacing_ok" in r and r.get("tick_t") is not None]
    pts = [p for p in st if t0 + 2 <= p[0] <= t1 - 2]
    (a, ma), (b, mb) = pts[0], pts[-1]
    return (b - a) / (mb - ma), t0, t1
for n in ["g1_11_trace0_A0", "g1_01_warm_A0", "g1_02_b1_A0", "g1_07_b2_A0", "g1_09_b3_A0"]:
    r, t0, t1 = ratio(n); print(n, round(r, 4), round(t0, 2), round(t1, 1))
