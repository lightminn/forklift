"""The revision plan 9.4 and 9.7's numbers from the D batch: run in the batch's session folder with this repository's
tools on the path. stands: every stand as the check reads it, with the reference taken again at the first sample on the
released correction, and each release's own step (tools/pose_check_stands.py hold_releases, from slam.records). caps: W2
with a time cap, as that tool's monitors.

    python d_extra.py stands OUT.json
    python d_extra.py caps OUT.json
"""
import glob
import json
import math
import os
import sys

import pose_check_stands as S

POSITIVE = ("val3_016", "val3_055")


def _records(s):
    rows = []
    for f in sorted(glob.glob(s + "/epoch_1/run/spool/slam.records/*.jsonl")):
        rows += [json.loads(line) for line in open(f)]
    return rows


def stands():
    rows, steps, total = [], [], 0
    for s in sorted(glob.glob("val3_*")):
        pc = json.load(open(s + "/epoch_1/run/result.json"))["pose_check"]
        smp = [x for x in pc["samples"] if x.get("o")]
        found = S.hold_releases(_records(s))
        steps += [{"session": s, **{k: r[k] for k in ("t0", "t1", "m", "rad")}} for r in found]
        if s in POSITIVE:
            continue
        rel = [S.release_mark(smp, r) for r in found]
        for st in pc["raw"]["stands"]:
            total += 1
            if sum(st["t0"] <= x["t"] <= st["t1"] for x in smp) < 2:
                continue
            m, r = S.stand_change(smp, t0=st["t0"], t1=st["t1"], releases=rel)
            rows.append({"session": s, "t0": st["t0"], "s": st["t1"] - st["t0"], "mission": st["mission"], "rad": r, "m": m,
                         "plain_rad": st["max_rad"], "plain_m": st["max_m"]})
    out = {"stands_in_correct_sessions": total, "read": len(rows),
           "rebased_max": [max(r["m"] for r in rows), max(r["rad"] for r in rows)],
           "plain_max": [max(r["plain_m"] for r in rows), max(r["plain_rad"] for r in rows)],
           "in_mission": sum(r["mission"] for r in rows),
           "release_steps": {"n": len(steps), "sessions": len({x["session"] for x in steps}),
                             "m_max": max(x["m"] for x in steps), "rad_max": max(x["rad"] for x in steps)}, "rows": rows}
    return out


def caps():
    correct, missions = {}, []
    for s in sorted(glob.glob("val3_*")):
        if s in POSITIVE:
            continue
        pc = json.load(open(s + "/epoch_1/run/result.json"))["pose_check"]
        ren = S.renewals_of(pc, e_slam=(0.109, 0.0137))
        if not ren:
            continue
        correct[s] = S.series([x for x in pc["samples"] if x.get("o")], ren)
        probe = json.load(open(s + "/probe.json"))
        if ((probe["reverse"].get("mission") or {}).get("result") or {}).get("outcome") == "done":
            missions.append(s)
    out = {}
    for cap in (math.inf, 20.0, 10.0, 5.0):
        base = S.Monitor("W", window_m=2.0, window_s=cap)
        mon = S.Monitor("W", window_m=2.0, window_s=cap, limit=S.fit_limit(correct, base))
        by = {}
        for n in missions:
            for _, on in S.onsets(correct[n], n, S._suffix(correct[n])):
                for c in S.kinds():
                    by.setdefault(f"{c.kind}:{abs(c.size):g}", []).append(S.run_case(correct[n], on, c, mon))
        out[str(cap)] = {"limit": list(mon.limit), "false": sum(S.first_alarm(v, mon) is not None for v in correct.values()),
                         "loo": sum(S.leave_one_out(correct, base).values()), "by_size": {k: S._tally(v) for k, v in by.items()}}
    return out


if __name__ == "__main__":
    json.dump({"stands": stands, "caps": caps}[sys.argv[1]](), open(sys.argv[2], "w"), indent=1)
