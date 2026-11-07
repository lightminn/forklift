"""The revision plan 9.19's E_SLAM from the audit table: python eslam_derive.py AUDIT.json INPUTS.json RESULTS_DIR... > OUT

A session belongs when it is in the closed input list and its start was not wrong (start estimate within 0.30 m and
0.10 rad of the truth). A row counts when it qualified (the conservative check and the applying rule passed) and lies
outside every contact interval. A g outside the success tolerance is not dropped: it is reported as a budget premise failure.
"""
import glob
import json
import math
import os
import sys

FACTOR = 1.5
CAP = (0.198, 0.048)


def start_wrong(result: dict) -> bool:
    pc = result.get("pose_check") or {}
    raw = (pc.get("raw") or {}).get("start") or {}
    truth = raw.get("truth") or {}
    if truth.get("class"):
        return truth["class"] == "wrong"
    sr = result.get("start_reloc") or {}
    est, tr = sr.get("estimate"), sr.get("truth")
    if not est or not tr:
        return True  # unknown is no basis
    return (math.hypot(est[0] - tr[0], est[1] - tr[1]) > 0.30
            or abs(math.remainder(est[2] - tr[2], math.tau)) > 0.10)


def main():
    audit, inputs, dirs = json.load(open(sys.argv[1])), json.load(open(sys.argv[2])), sys.argv[3:]
    listed = {r["session"] for r in inputs["sessions"]}
    results = {}
    for d in dirs:
        for s in glob.glob(d):
            f = os.path.join(s, "epoch_1", "run", "result.json")
            if os.path.basename(s) in listed and os.path.exists(f):
                results[os.path.basename(s)] = json.load(open(f))
    wrong = {n for n, r in results.items() if start_wrong(r)}
    rows = [r for r in audit["rows"] if r["session"] in listed]
    used = [r for r in rows if r["session"] not in wrong and r["qualified"] and not r["in_contact"]]
    premise = [r for r in used if r.get("g_outside_success")]
    out = {"sessions_listed": len(listed), "sessions_read": len(results), "start_wrong": sorted(wrong),
           "rows_listed": len(rows), "rows_used": len(used),
           "rows_out": {"start_wrong": sum(r["session"] in wrong for r in rows),
                        "not_qualified": sum(r["session"] not in wrong and not r["qualified"] for r in rows),
                        "in_contact": sum(r["session"] not in wrong and r["qualified"] and r["in_contact"] for r in rows)},
           "premise_failures": [{k: r[k] for k in ("session", "t", "kind", "g_err", "agree_m", "agree_rad")} for r in premise]}
    if used:
        m, rad = max(r["agree_m"] for r in used), max(r["agree_rad"] for r in used)
        out["max"] = [m, rad]
        out["e_slam"] = [FACTOR * m, FACTOR * rad]
        out["over_cap"] = out["e_slam"][0] > CAP[0] or out["e_slam"][1] > CAP[1]
        out["argmax"] = {"m": max(used, key=lambda r: r["agree_m"])["session"],
                         "rad": max(used, key=lambda r: r["agree_rad"])["session"]}
        out["by_group"] = {}
        for g in ("val3_", "t_val3_", "b_val3_"):
            sel = [r for r in used if r["session"].startswith(g) and not (g == "val3_" and r["session"].startswith("t_"))]
            if sel:
                out["by_group"][g] = {"rows": len(sel), "max": [max(r["agree_m"] for r in sel), max(r["agree_rad"] for r in sel)]}
        b = [r for r in used if r["session"].startswith("b_val3_032")]
        out["b032"] = [{k: r[k] for k in ("session", "kind", "ok", "generation", "attempts", "agree_m", "agree_rad", "g_err",
                                           "c_err", "wider", "reverse_m")} for r in b]
    json.dump(out, sys.stdout, indent=1)


if __name__ == "__main__":
    main()
