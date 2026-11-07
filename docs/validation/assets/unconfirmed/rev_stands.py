"""The lead's record table of 2026-11-07, from the records alone: python rev_stands.py OUT.json FOLDER...

Per session: the largest true error at open ticks, every revocation by gate, the final status. Per stand of the shadow
sessions: the heading odometry's drift, the true heading's change, the corrections applied inside the stand, the gyro's
standstill bias updates inside it.
"""
import collections
import glob
import json
import math
import os
import sys


def _wrap(a):
    return math.remainder(a, math.tau)


def _replies(s):
    rows = []
    for f in sorted(glob.glob(s + "/epoch_1/run/spool/slam.records/*.jsonl")):
        rows += [json.loads(line) for line in open(f)]
    return [r for r in rows if r.get("applied_after") is not None and "applied_tick_s" in r]


def gyro_history(r):
    for key, value in r.items():
        if "gyro" in key and isinstance(value, dict) and "history" in value:
            return value
    return None


def one(s):
    r = json.load(open(s + "/epoch_1/run/result.json"))
    pc = r.get("pose_check") or {}
    probe = json.load(open(s + "/probe.json"))
    m = (probe.get("reverse") or {}).get("mission") or {}
    sub, res = m.get("submit") or {}, m.get("result") or {}
    ends = [e for e in pc.get("events", []) if e.get("kind") in ("revoke", "ended")]
    open_ = (pc.get("ticks") or {}).get("open") or {}
    row = {"session": os.path.basename(s), "mode": pc.get("mode"), "status": pc.get("status"),
           "mission": sub.get("reason") if sub.get("t") == "reject" else res.get("outcome"),
           "mission_t": [res.get("started_s"), res.get("ended_s")], "damaged": r.get("scene_damaged") is not None,
           "open_max": [open_.get("max_m"), open_.get("max_rad")], "ends": ends}
    stands = []
    samples = [x for x in pc.get("samples", []) if x.get("o") and x.get("truth")]
    replies = _replies(s)
    gyro = gyro_history(r)
    history = (gyro or {}).get("history") or []
    for st in ((pc.get("raw") or {}).get("stands") or []):
        inside = [x for x in samples if st["t0"] <= x["t"] <= st["t1"]]
        if len(inside) < 2:
            continue
        dur = inside[-1]["t"] - inside[0]["t"]
        if dur <= 0:
            continue
        o = _wrap(inside[-1]["o"][2] - inside[0]["o"][2])
        truth = _wrap(inside[-1]["truth"][2] - inside[0]["truth"][2])
        steps = []
        last = None
        for q in replies:
            if q["applied_tick_s"] > st["t1"]:
                break
            c = q["applied_after"]
            if last is not None and q["applied_tick_s"] >= st["t0"] and c != last:
                steps.append(abs(_wrap(c[2] - last[2])))
            last = c
        stands.append({"session": row["session"], "t0": inside[0]["t"], "s": dur, "o_rad": o, "rate": o / dur, "truth_rad": truth,
                       "n_corr": len(steps), "corr_max": max(steps, default=0.0), "corr_sum": sum(steps),
                       "zupt": sum(st["t0"] <= h[0] <= st["t1"] for h in history), "mission": st["mission"],
                       "rebased_rad": st.get("rebased_rad")})
    row["gyro_updates"] = (gyro or {}).get("updates")
    row["bias_last"] = (gyro or {}).get("bias_rad_s")
    return row, stands


def main():
    out, folders = sys.argv[1], sys.argv[2:]
    rows, stands = [], []
    for folder in folders:
        for s in sorted(glob.glob(folder)):
            if os.path.exists(s + "/epoch_1/run/result.json") and os.path.exists(s + "/probe.json"):
                a, b = one(s)
                rows.append(a)
                stands += b
    bins = ((0, 2), (2, 5), (5, 10), (10, 30), (30, 1e9))
    table = []
    for lo, hi in bins:
        c = [x for x in stands if lo <= x["s"] < hi]
        if c:
            rates = sorted(abs(x["rate"]) for x in c)
            table.append({"s": [lo, hi], "n": len(c), "rate_p50": rates[len(rates) // 2], "rate_max": rates[-1],
                          "o_max": max(abs(x["o_rad"]) for x in c), "truth_max": max(abs(x["truth_rad"]) for x in c),
                          "n_corr_mean": sum(x["n_corr"] for x in c) / len(c), "corr_max": max(x["corr_max"] for x in c),
                          "zupt_mean": sum(x["zupt"] for x in c) / len(c), "zupt_zero": sum(x["zupt"] == 0 for x in c)})
    gates = collections.Counter()
    for r in rows:
        for e in r["ends"]:
            if e.get("kind") == "revoke":
                gates[(r["session"][0] if r["session"][0] in "tl" else "d", e.get("gate"))] += 1
    json.dump({"sessions": rows, "stands": stands, "table": table}, open(out, "w"), indent=0)
    print(json.dumps(table, indent=0)[:1800])
    print({str(k): v for k, v in gates.items()})
    print(collections.Counter(str(r["status"]).split(":", 1)[-1] if str(r["status"]).startswith("ended") else str(r["status"]) for r in rows))


if __name__ == "__main__":
    main()
