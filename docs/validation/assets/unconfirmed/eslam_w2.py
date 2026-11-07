"""The revision plan 9.19's W2 re-evaluation under the re-derived E_SLAM, the old results kept beside it:

    python eslam_w2.py OUT.json OLD_M,OLD_RAD NEW_M,NEW_RAD SESSION_DIR...

Per session: the renewals (count, first confirmation, last renewal) under each E_SLAM; then W2 at its fixed limit on the
correct sessions (false alarms, the window's largest change) and on the injected cases over the mission sessions (count,
caught), as tools/pose_check_stands.py's evaluation reads them.
"""
import json
import math
import sys
from pathlib import Path

import pose_check_stands as S

W2 = S.Monitor("W", window_m=2.0, window_s=10.0, limit=(0.153, 0.103))
OPEN = S.Monitor("W", window_m=2.0, window_s=10.0)


def load(folder):
    folder = Path(folder)
    result = json.loads((folder / "epoch_1" / "run" / "result.json").read_text())
    probe = json.loads((folder / "probe.json").read_text())
    return result["pose_check"], probe


def run(e_slam, folders):
    per, correct, missions = {}, {}, []
    for f in folders:
        rec, probe = load(f)
        samples = [s for s in rec["samples"] if s.get("o") is not None]
        wrong = next((s["t"] for s in samples if math.hypot(s["applied"][0] - s["truth"][0], s["applied"][1] - s["truth"][1]) > 0.30
                      or abs(math.remainder(s["applied"][2] - s["truth"][2], math.tau)) > 0.10), None)
        ren = S.renewals_of(rec, e_slam=e_slam)
        name = Path(f).name
        per[name] = {"renewals": len(ren), "first": ren[0][0] if ren else None, "last": ren[-1][0] if ren else None,
                     "wrong_at": wrong}
        if ren and wrong is None:
            correct[name] = S.series(samples, ren)
            if ((probe.get("reverse", {}).get("mission") or {}).get("result") or {}).get("outcome") == "done":
                missions.append(name)
    false = {n: S.first_alarm(s, W2) is not None for n, s in correct.items()}
    lim = S.fit_limit(correct, OPEN) if correct else (0.0, 0.0)
    cases = caught = 0
    by = {}
    for n in missions:
        ser = correct[n]
        for _, on in S.onsets(ser, n, S._suffix(ser)):
            for c in S.kinds():
                r = S.run_case(ser, on, c, W2)
                key = S.case_key(c)
                b = by.setdefault(key, [0, 0])
                b[0] += 1
                b[1] += bool(r["detected"])
                cases += 1
                caught += bool(r["detected"])
    return {"sessions": per, "correct": len(correct), "missions": len(missions), "false_alarms": sum(false.values()),
            "window_max": [lim[0] / S.FACTOR, lim[1] / S.FACTOR], "cases": cases, "caught": caught, "by_kind": by}


def main():
    out, old, new, folders = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:]
    parse = lambda v: tuple(float(x) for x in v.split(","))  # noqa: E731
    res = {"old": run(parse(old), folders), "new": run(parse(new), folders)}
    changed = [n for n in res["old"]["sessions"] if res["old"]["sessions"][n] != res["new"]["sessions"][n]]
    res["changed_sessions"] = changed
    json.dump(res, open(out, "w"), indent=0)
    for k in ("old", "new"):
        r = res[k]
        print(k, "correct", r["correct"], "missions", r["missions"], "false", r["false_alarms"], "window_max",
              [round(v, 4) for v in r["window_max"]], "cases", r["cases"], "caught", r["caught"])
    print("sessions whose renewals changed", len(changed), changed[:12])


if __name__ == "__main__":
    main()
