import json
from forklift_core.live.spool import read_live_log
G = json.load(open("g1_analysis.json"))
by = {r["name"]: r for r in G["runs"]}
for row in G["m9"]:
    tgt = [n for n in by if n.split("_")[2] == row["block"] and by[n]["arm"] == row["target"]][0]
    r = by[tgt]; t0 = r["driving_sim_s"][0]; span = row["common_span_sim_s"]
    st = [x for x in read_live_log(f"g1/{tgt}/live/epoch_1/run") if "pacing_ok" in x and x.get("tick_t") is not None]
    st = [x for x in st if x["tick_t"] <= t0 + span]
    oks = [bool(x["pacing_ok"]) for x in st]
    i0 = oks.index(True) if True in oks else None
    auto = [bool(x["pacing_ok"]) for x in st[i0:] if x.get("mode") == "AUTO"] if i0 is not None else []
    print(row["block"], row["target"], "pred_ok_common", round(row["predicted_ok_common"], 3), "meas_ok_auto_common", round(sum(auto) / len(auto), 3) if auto else None, "meas_ok_auto_whole", round(row["measured_ok_auto"], 3))
