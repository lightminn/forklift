"""Offline diagnosis of the hall return-cusp stall (cusp_position_stall), from records only (lead assignment 2026-11-10).

For each run folder (``<root>/<batch>/<session>/live/epoch_1/run``): the mission's tracking failure and first cusp
stall (missions/m1/records.json), and per processed SLAM reply (spool/slam.records/*.jsonl) three pose errors against
the truth at the scan, map frame = world at the start:

- odometry: odom_base - truth_base (the raw odometry the SLAM link is sent);
- SLAM reply: compose(map_from_odom, odom_base) - truth_base (what the bridge estimates for that scan);
- applied: compose(applied correction, odom_base) - truth_base (what control used after that reply: the sync link's
  ``applied_map_from_odom`` or the async link's ``applied_after``).

Reported per session and phase: medians of |e|, e_y and e_yaw; the largest SLAM-reply error; every scan-to-scan jump of
the SLAM-reply error above --jump-m during withdraw / return_home with its time and truth position; the stall's time.

    python cusp_diag.py ROOT [--json out.json] [--jump-m 0.05]
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

PHASES = ("observe", "approach", "insert", "extract", "transport", "withdraw", "return_home")


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def wrap(x: float) -> float:
    return math.atan2(math.sin(x), math.cos(x))


def err(est, truth):
    return (est[0] - truth[0], est[1] - truth[1], wrap(est[2] - truth[2]))


def records(run: Path, group: str) -> list[dict]:
    out = []
    for f in sorted((run / "spool" / group).glob("*.jsonl")):
        out += [json.loads(line) for line in f.read_text().splitlines() if line.strip()]
    return out


def med(a) -> list | None:
    if not len(a):
        return None
    a = np.asarray(a, float)
    return [float(np.median(np.hypot(a[:, 0], a[:, 1]))), float(np.median(a[:, 1])), float(np.median(a[:, 2]))]


def session(run: Path, jump_m: float) -> dict:
    state = json.loads((run / "missions" / "m1" / "records.json").read_text()).get("state") or {}
    stall = (state.get("cusp_stalls") or [None])[0]
    tf = state.get("tracking_failure")
    by = {}
    prev, jumps, max_reply = None, [], 0.0
    for r in records(run, "slam.records"):
        if r.get("status") != "processed" or r.get("map_from_odom") is None or r.get("truth_base") is None:
            continue
        o, tr = r["odom_base"], r["truth_base"]
        applied = r.get("applied_after", r.get("applied_map_from_odom"))
        e_reply = err(compose(r["map_from_odom"], o), tr)
        row = by.setdefault(r.get("phase"), {"odometry": [], "reply": [], "applied": []})
        row["odometry"].append(err(o, tr))
        row["reply"].append(e_reply)
        if applied is not None:
            row["applied"].append(err(compose(applied, o), tr))
        max_reply = max(max_reply, math.hypot(e_reply[0], e_reply[1]))
        if prev is not None and r.get("phase") in ("withdraw", "return_home"):
            d = math.hypot(e_reply[0] - prev[0], e_reply[1] - prev[1])
            if d > jump_m:
                jumps.append({"t": float(r["stamp_s"]), "jump_m": d, "truth_xy": [float(tr[0]), float(tr[1])],
                              "error_before": [float(prev[0]), float(prev[1])],
                              "error_after": [float(e_reply[0]), float(e_reply[1])]})
        prev = e_reply
    return {"tracking_failure": (tf or {}).get("reason") if isinstance(tf, dict) else tf,
            "stall": None if stall is None else {k: stall.get(k) for k in ("time_s", "leg_index", "position_error_m",
                                                                           "rear", "phase")},
            "max_reply_error_m": max_reply, "return_jumps": jumps,
            "phases": {ph: {k: med(v) for k, v in by[ph].items()} | {"n": len(by[ph]["reply"])}
                       for ph in PHASES if ph in by}}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--jump-m", type=float, default=0.05)
    a = ap.parse_args()
    out = {}
    for run in sorted(a.root.glob("*/*/live/epoch_1/run")):
        out[run.parts[-4]] = session(run, a.jump_m)
    text = json.dumps(out, indent=1)
    if a.json:
        a.json.write_text(text)
    for name, s in out.items():
        j = s["return_jumps"]
        print(f"{name:24s} {s['tracking_failure'] or 'done':22s} stall_t={None if not s['stall'] else round(s['stall']['time_s'], 1)} "
              f"max_reply={s['max_reply_error_m']:.3f} transport_reply={s['phases'].get('transport', {}).get('reply')} "
              f"jumps={[(round(x['t'], 1), round(x['jump_m'], 3)) for x in j]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
