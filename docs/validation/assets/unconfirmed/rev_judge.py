"""The revision plan 9.11 and 9.13's numbers: run with tools/ and src/ on the path.

    python rev_judge.py D_FOLDER TAIL_FOLDER OUT.json

D_FOLDER holds D's val3_* sessions, TAIL_FOLDER the t_* sessions, each with epoch_1/run/result.json,
epoch_1/run/spool/slam.records and probe.json. The replay adds +-0.12 rad at each natural hold release of the correct
sessions and runs W2 on the record: the truck's motion after it is what it was.
"""
import glob
import json
import math
import os
import sys
from collections import Counter

import numpy as np

import pose_check_stands as S

POSITIVE = ("val3_016", "val3_055")
E_SLAM = (0.109, 0.0137)
LIMIT = (0.153, 0.103)
GATE = (0.065, 0.076)
ESTILL = (0.0347, 0.0079)
W2 = S.Monitor("W", window_m=2.0, window_s=10.0, limit=LIMIT)
OPEN = S.Monitor("W", window_m=2.0, window_s=10.0)


def _replies(s):
    rows = []
    for f in sorted(glob.glob(s + "/epoch_1/run/spool/slam.records/*.jsonl")):
        rows += [json.loads(line) for line in open(f)]
    return rows


def _outcome(s):
    m = (json.load(open(s + "/probe.json")).get("reverse") or {}).get("mission") or {}
    sub, res = m.get("submit") or {}, m.get("result") or {}
    return sub.get("reason") if sub.get("t") == "reject" else res.get("outcome")


def _releases(pc, s):
    """(time, C before, C after, O at it) of each natural release; the record's own, or the replies' in D."""
    if "releases" in pc:
        return [(r["t"], tuple(r["before"]), tuple(r["after"]), tuple(r["o"])) for r in pc["releases"]]
    samples = [x for x in pc["samples"] if x.get("o")]
    return [(S.release_mark(samples, r), tuple(r["held"]), tuple(r["applied"]), tuple(r["o"]))
            for r in S.hold_releases(_replies(s))]


def replay(name, s, ser, pc) -> list:
    out = []
    for at, before, after, o in _releases(pc, s):
        if at < ser.t[0] or at > ser.t[-1]:
            continue
        onset = int(np.searchsorted(ser.t, at, side="left"))
        if onset >= len(ser.t):
            continue
        natural = math.remainder(after[2] - before[2], math.tau)
        m0, r0 = S._gap(S.compose(after, o), S.compose(before, o))
        for sign in (1.0, -1.0):
            res = S.run_case(ser, onset, S.Injection("step_yaw", sign * 0.12), W2)
            pose = S.compose(after, o)
            pivoted = (pose[0], pose[1], pose[2] + sign * 0.12)
            m, rad = S._gap(pivoted, S.compose(before, o))
            out.append({"session": name, "t": at, "natural_rad": natural, "natural_m": m0, "sign": sign,
                        "same_sign": bool(natural * sign > 0), "w2": res["detected"], "gate": m > GATE[0] or rad > GATE[1],
                        "gate_m": m, "gate_rad": rad, "wrong": res["wrong_reached"], "delay_m": res["delay_m"]})
    return out


def session(s):
    r = json.load(open(s + "/epoch_1/run/result.json"))
    return r, r["pose_check"]


def main():
    d, t, out = sys.argv[1:4]
    rows, tail, bare, stands = [], {}, [], []
    for s in sorted(glob.glob(d + "/val3_*")):
        name = os.path.basename(s)
        if name in POSITIVE:
            continue
        r, pc = session(s)
        ren = S.renewals_of(pc, e_slam=E_SLAM)
        if ren:
            rows += replay(name, s, S.series([x for x in pc["samples"] if x.get("o")], ren), pc)
    info = {}
    for s in sorted(glob.glob(t + "/t_*")):
        name = os.path.basename(s)
        r, pc = session(s)
        done = _outcome(s) == "done"
        ren = S.renewals_of(pc, e_slam=E_SLAM)
        events = [e for e in pc.get("events", []) if e.get("kind") == "revoke"]
        err = pc.get("ticks", {}).get("all", {})
        info[name] = {"done": done, "outcome": _outcome(s), "damaged": r.get("scene_damaged") is not None,
                      "revoked": [(e.get("gate"), e["t"]) for e in events], "truth_max": [err.get("max_m"), err.get("max_rad")],
                      "link_failures": (r.get("slam_summary") or {}).get("link_failures"),
                      "heading": r["arguments"].get("odometry_heading"), "encoder": r["arguments"].get("encoder_model"),
                      "releases": len(pc.get("releases", []))}
        if not ren:
            continue
        ser = S.series([x for x in pc["samples"] if x.get("o")], ren)
        limit = S.fit_limit({name: ser}, OPEN)
        info[name]["window_max"] = [limit[0] / S.FACTOR, limit[1] / S.FACTOR]
        rows += replay(name, s, ser, pc)
        if done:
            tail[name] = ser
        bare += [{"session": name, "t": x["t"], "m": x["m"], "rad": x["rad"], "status": x["status"]} for x in pc.get("releases", [])]
        stands += [{"session": name, "m": x["rebased_m"], "rad": x["rebased_rad"], "s": x["t1"] - x["t0"], "mission": x["mission"]}
                   for x in pc["raw"]["stands"]]
    cells = {}
    for same in (True, False):
        for sign in (1.0, -1.0):
            c = [x for x in rows if x["same_sign"] == same and x["sign"] == sign]
            cells[f"{'same' if same else 'opposite'} {'+' if sign > 0 else '-'}"] = {
                "n": len(c), "w2_miss": sum(not x["w2"] for x in c), "gate_miss": sum(not x["gate"] for x in c),
                "both_miss": sum(not x["w2"] and not x["gate"] for x in c),
                "both_miss_wrong": sum(not x["w2"] and not x["gate"] and x["wrong"] for x in c)}
    lim = S.fit_limit(tail, OPEN) if tail else (0.0, 0.0)
    summary = {
        "outcomes": Counter(str(v["outcome"]) for v in info.values()), "eligible": len(tail),
        "tail_window_max": [lim[0] / S.FACTOR, lim[1] / S.FACTOR], "cells": cells, "replay_rows": len(rows),
        "replay_sessions": len({x["session"] for x in rows}), "w2_miss": sum(not x["w2"] for x in rows),
        "bare_release": {"n": len(bare), "m_max": max((x["m"] for x in bare), default=0.0),
                         "rad_max": max((x["rad"] for x in bare), default=0.0),
                         "over_gate": sum(x["m"] > GATE[0] or x["rad"] > GATE[1] for x in bare)},
        "stands": {"n": len(stands), "m_max": max((x["m"] for x in stands), default=0.0),
                   "rad_max": max((x["rad"] for x in stands), default=0.0),
                   "over_e_still": sum(x["m"] > ESTILL[0] or x["rad"] > ESTILL[1] for x in stands)}}
    json.dump({"summary": summary, "sessions": info, "replay": rows, "bare": bare, "stands": stands}, open(out, "w"), indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
