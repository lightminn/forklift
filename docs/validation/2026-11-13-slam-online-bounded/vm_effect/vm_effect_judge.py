"""VM-on effect judgment as registered in docs/validation/2026-11-13-slam-online-bounded/vm_effect_registration.md.
Runs on ws1 with python3 (numpy): python3 vm_effect_judge.py OFF_DIR ON_DIR"""
import json
import sys
from pathlib import Path

import numpy as np

off, on = Path(sys.argv[1]), Path(sys.argv[2])


def stats(run):
    w = np.array([float(json.loads(x)["slam_wall_s"]) for x in (run / "drive.jsonl").read_text().splitlines()
                  if x.strip()])
    return {"n": int(w.size), "mean": float(w.mean()), "p99": float(np.percentile(w, 99)), "max": float(w.max())}


out = {"arms": {}, "allowed": True}
for arm in ("r0", "p0"):
    o = [stats(off / f"{arm}_{i}") for i in (1, 2)]
    n = [stats(on / f"{arm}_{i}") for i in (1, 2)]
    om, nm = (o[0]["mean"] + o[1]["mean"]) / 2, (n[0]["mean"] + n[1]["mean"]) / 2
    op, np99 = (o[0]["p99"] + o[1]["p99"]) / 2, (n[0]["p99"] + n[1]["p99"]) / 2
    gap = abs(o[0]["p99"] - o[1]["p99"])
    mean_rel = (nm - om) / om
    p99_inc = np99 - op
    ok_mean, ok_p99 = mean_rel <= 0.05, p99_inc <= 2 * gap
    out["arms"][arm] = {"off": o, "on": n, "mean_rel": mean_rel, "p99_increase_s": p99_inc,
                        "p99_allowance_s": 2 * gap, "mean_ok": ok_mean, "p99_ok": ok_p99}
    out["allowed"] = out["allowed"] and ok_mean and ok_p99
print(json.dumps(out, indent=1))
