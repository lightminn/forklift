"""The lead's record audit of 2026-11-07 (plan 9.15), fixed before its values were read: the frozen gates together on the
shadow records, from the raw values only -- no status, outcome or renewals_of.

    python rev_audit.py OUT.json FOLDER...

Each session whose start the check confirmed (raw.start.confirmed, its agreement within E_SLAM) and that was not damaged
before its first drive command, from the confirmation to the end of the first mission (or of the session). Gates: the
window, the stand with its reference taken again at each release, the release itself, and the re-match where an on run
would have asked for one: a stand past 0.10 m of Delta_int outside the near field and a mission (the shadow's own trigger
holds those two), once per stand, its generation applied by the check's own conditions.
"""
import glob
import json
import math
import os
import sys

import numpy as np

import pose_check_stands as S
from forklift_core.localization.slam_pose import compose, invert

E_STILL, E_SLAM, E_WINDOW, E_RELEASE = (0.0347, 0.0079), (0.109, 0.0137), (0.153, 0.103), (0.065, 0.076)
REMATCH_M = 0.10


def _replies(s):
    rows = []
    for f in sorted(glob.glob(s + "/epoch_1/run/spool/slam.records/*.jsonl")):
        rows += [json.loads(line) for line in open(f)]
    return rows


def releases(pc, s, samples):
    if "releases" in pc:
        return [(r["t"], tuple(r["before"]), tuple(r["after"]), tuple(r["o"])) for r in pc["releases"]]
    return [(S.release_mark(samples, r), tuple(r["held"]), tuple(r["applied"]), tuple(r["o"]))
            for r in S.hold_releases(_replies(s))]


def first_drive(pc):
    """The first time the stroke or the mission moved the truck, from the samples' odometry."""
    samples = [x for x in pc["samples"] if x.get("o")]
    for a, b in zip(samples, samples[1:]):
        if math.hypot(b["o"][0] - a["o"][0], b["o"][1] - a["o"][1]) > 1e-3:
            return b["t"]
    return math.inf


def audit(s):
    r = json.load(open(s + "/epoch_1/run/result.json"))
    pc = r["pose_check"]
    start = (pc.get("raw") or {}).get("start") or {}
    name = os.path.basename(s)
    out = {"session": name, "t_start": start.get("t")}
    if not start.get("confirmed"):
        return {**out, "group": "start_not_confirmed"}
    if start.get("agree_m") is None or start["agree_m"] > E_SLAM[0] or start["agree_rad"] > E_SLAM[1]:
        return {**out, "group": "start_disagrees"}
    damaged = r.get("scene_damaged")
    early = damaged is not None and damaged.get("t", math.inf) <= first_drive(pc)
    probe = json.load(open(s + "/probe.json"))
    res = ((probe.get("reverse") or {}).get("mission") or {}).get("result") or {}
    end_t = res.get("ended_s") or math.inf
    samples = [x for x in pc["samples"] if x.get("o") and x.get("truth") and start["t"] <= x["t"] <= end_t]
    rel = releases(pc, s, [x for x in pc["samples"] if x.get("o")])
    c_int = compose(tuple(start["r"]), invert(tuple(pc["start"]["o_scan"])))
    gens = [g for g in pc["raw"].get("rematches") or [] if g["t"] >= start["t"] and g["t"] <= end_t]
    stands = [st for st in pc["raw"]["stands"] if st["t1"] >= start["t"] and st["t0"] <= end_t]
    # first revocation of each gate
    first = {}
    window, path, o_last = [], 0.0, None
    for x in samples:
        o, c = tuple(x["o"]), S._correction(x)
        if o_last is not None:
            path += math.hypot(o[0] - o_last[0], o[1] - o_last[1])
        o_last = o
        window.append((path, x["t"], c))
        while len(window) > 1 and (path - window[0][0] > 2.0 or x["t"] - window[0][1] > 10.0):
            window.pop(0)
        m, rad = S._gap(compose(c, o), compose(window[0][2], o))
        if (m > E_WINDOW[0] or rad > E_WINDOW[1]) and "window" not in first:
            first["window"] = (x["t"], m, rad)
    for at, before, after, o in rel:
        if start["t"] <= at <= end_t:
            m, rad = S._gap(compose(after, o), compose(before, o))
            if (m > E_RELEASE[0] or rad > E_RELEASE[1]) and "release" not in first:
                first["release"] = (at, m, rad)
    for st in stands:
        marks = sorted((at, after) for at, _, after, _ in rel if st["t0"] <= at <= st["t1"])
        ref, k = None, 0
        for x in samples:
            if not st["t0"] <= x["t"] <= st["t1"]:
                continue
            while k < len(marks) and x["t"] >= marks[k][0]:
                ref = marks[k][1]
                k += 1
            c = S._correction(x)
            ref = c if ref is None else ref
            m, rad = S._gap(compose(c, tuple(x["o"])), compose(ref, tuple(x["o"])))
            if (m > E_STILL[0] or rad > E_STILL[1]) and "still" not in first:
                first["still"] = (x["t"], m, rad, st["t1"] - st["t0"], bool(st["mission"]))
    asked = set()
    for g in gens:
        at = [x for x in samples if x["t"] <= g["t"]]
        if not at or g.get("g") is None or g.get("o_scan") is None:
            continue
        x = at[-1]
        o = tuple(x["o"])
        delta = S._gap(compose(S._correction(x), o), compose(c_int, o))[0]
        stand = next((i for i, st in enumerate(stands) if st["t0"] <= g["t"] <= st["t1"] + 1.0), None)
        if delta <= REMATCH_M or stand in asked:
            continue
        asked.add(stand)
        if not g.get("residual_confirmed") or not g.get("applicable") or g.get("agree_m") is None:
            continue  # no evidence: the check goes on
        if g["agree_m"] > E_SLAM[0] or g["agree_rad"] > E_SLAM[1]:
            first.setdefault("rematch", (g["t"], g["agree_m"], g["agree_rad"]))
        else:
            c_int = compose(tuple(g["g"]), invert(tuple(g["o_scan"])))
    order = sorted(first.items(), key=lambda kv: kv[1][0])
    err = [(math.hypot(x["applied"][0] - x["truth"][0], x["applied"][1] - x["truth"][1]),
            abs(math.remainder(x["applied"][2] - x["truth"][2], math.tau))) for x in samples] or [(0.0, 0.0)]
    return {**out, "group": "start_invalid" if early else "audit", "t_end": None if math.isinf(end_t) else end_t,
            "mission": res.get("outcome"), "first": first, "first_gate": order[0][0] if order else None,
            "first_t": order[0][1][0] if order else None, "truth_max": [max(e[0] for e in err), max(e[1] for e in err)],
            "generations": len(gens), "asked": len(asked)}


def main():
    out, folders = sys.argv[1], sys.argv[2:]
    rows = [audit(s) for f in folders for s in sorted(glob.glob(f)) if os.path.exists(s + "/epoch_1/run/result.json")]
    json.dump(rows, open(out, "w"), indent=0)
    audit_rows = [r for r in rows if r["group"] == "audit"]
    print({g: sum(r["group"] == g for r in rows) for g in {r["group"] for r in rows}})
    print("first revocation:", {k: sum(r["first_gate"] == k for r in audit_rows) for k in ("window", "still", "rematch", "release", None)})
    for r in rows:
        if r.get("first"):
            print(r["session"], r["group"], r["mission"], {k: [round(float(v), 4) if not isinstance(v, bool) else v for v in vals] for k, vals in r["first"].items()},
                  [round(a, 3) for a in r["truth_max"]])


if __name__ == "__main__":
    main()
