"""Hall SLAM replay reading (docs/validation/2026-11-10-hall-slam-replay.md), fixed before the replays' results.

For each session: the live run (EPOCH/run: spool/slam.records, the phase of each scan) and its two replays
(OUT/<session>_recorded, OUT/<session>_truth: replies.jsonl, status.json). Per processed reply the SLAM estimate error
compose(map_from_odom, odom) - truth_base, by phase; the transport phase's median |e| and e_y; the return-phase steps of
that error above 0.05 m between processed replies (as cusp_diag.py).

Validity (per session): both replays ok (status.json) and complete (every recorded sent scan answered up to the end), and
the recorded arm reproduces the live drift: its transport median |e| within 0.03 m of the live run's and its median
e_y of the same sign.

Reading (registered):
- per session, the truth arm's transport median |e| against the recorded arm's: "odometry-driven" when it is below 0.05 m
  and below half the recorded arm's; "SLAM's own" when it is within 0.03 m of the recorded arm's; otherwise "mixed";
- overall: the same reading in every valid session, else "split" (each session's reading reported);
- reported, not judged: each arm's return-phase steps (time, size, truth position) and the truth arm's e_yaw.

    python replay_eval.py OUT --live ROOT SESSION... [--json out.json]
    (ROOT/<batch>/<session>/live/epoch_1 holds each live epoch; SESSION as <batch>/<session>)
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

STEP_M = 0.05


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def wrap(x: float) -> float:
    return math.atan2(math.sin(x), math.cos(x))


def jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def spool(run: Path, group: str) -> list[dict]:
    out = []
    for f in sorted((run / "spool" / group).glob("*.jsonl")):
        out += jsonl(f)
    return out


def errors(rows: list[dict], phase_of: dict, odom_key: str) -> dict:
    by, steps, prev = {}, [], None
    for r in rows:
        if r.get("status") != "processed" or r.get("map_from_odom") is None:
            continue
        o, tr = r[odom_key], r["truth_base"]
        est = compose(r["map_from_odom"], o)
        e = (est[0] - tr[0], est[1] - tr[1], wrap(est[2] - tr[2]))
        ph = phase_of.get(r["scan_id"])
        by.setdefault(ph, []).append(e)
        if prev is not None and ph in ("withdraw", "return_home"):
            d = math.hypot(e[0] - prev[0], e[1] - prev[1])
            if d > STEP_M:
                steps.append({"t": float(r["stamp_s"]), "step_m": d, "truth_xy": [float(tr[0]), float(tr[1])]})
        prev = e
    out = {"steps": steps}
    for ph, v in by.items():
        a = np.asarray(v, float)
        out[str(ph)] = {"n": len(a), "abs_e_m": float(np.median(np.hypot(a[:, 0], a[:, 1]))),
                        "e_y_m": float(np.median(a[:, 1])), "e_yaw_rad": float(np.median(a[:, 2]))}
    return out


def session(out_root: Path, live_root: Path, name: str) -> dict:
    run = live_root / name / "live" / "epoch_1" / "run"
    live = spool(run, "slam.records")
    phase_of = {r["scan_id"]: r.get("phase") for r in live}
    sent = [r["scan_id"] for r in jsonl(live_root / name / "live" / "epoch_1" / "bridge" / "records.jsonl")
            if r.get("sent") != "local"]
    res = {"live": errors(live, phase_of, "odom_base"), "arms": {}, "invalid": []}
    for arm in ("recorded", "truth"):
        d = out_root / f"{name.split('/')[-1]}_{arm}"
        st = json.loads((d / "status.json").read_text()) if (d / "status.json").exists() else {}
        rows = jsonl(d / "replies.jsonl") if (d / "replies.jsonl").exists() else []
        if not st.get("ok"):
            res["invalid"].append(f"{arm}: status {st}")
        if [r["scan_id"] for r in rows] != sent:
            res["invalid"].append(f"{arm}: {len(rows)} replies for {len(sent)} sent scans")
        res["arms"][arm] = errors(rows, phase_of, "odom")
    lt, rt = res["live"].get("transport"), res["arms"]["recorded"].get("transport")
    if lt is None or rt is None:
        res["invalid"].append("no transport phase")
    elif abs(rt["abs_e_m"] - lt["abs_e_m"]) > 0.03 or np.sign(rt["e_y_m"]) != np.sign(lt["e_y_m"]):
        res["invalid"].append(f"the recorded arm does not reproduce the live drift: {rt} vs {lt}")
    tt = res["arms"]["truth"].get("transport")
    if not res["invalid"] and tt is not None:
        if tt["abs_e_m"] < 0.05 and tt["abs_e_m"] < rt["abs_e_m"] / 2:
            res["reading"] = "odometry-driven"
        elif abs(tt["abs_e_m"] - rt["abs_e_m"]) <= 0.03:
            res["reading"] = "SLAM's own"
        else:
            res["reading"] = "mixed"
    return res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--live", type=Path, required=True)
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    res = {n: session(a.out, a.live, n) for n in a.sessions}
    readings = {n: r.get("reading") for n, r in res.items() if not r["invalid"]}
    overall = None
    if readings:
        vals = set(readings.values())
        overall = vals.pop() if len(vals) == 1 else "split"
    out = {"sessions": res, "readings": readings, "overall": overall,
           "invalid": {n: r["invalid"] for n, r in res.items() if r["invalid"]}}
    text = json.dumps(out, indent=1)
    if a.json:
        a.json.write_text(text)
    print(json.dumps({"readings": readings, "overall": overall, "invalid": out["invalid"]}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
