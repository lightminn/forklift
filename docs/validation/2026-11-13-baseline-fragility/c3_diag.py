"""(나) C3 record diagnosis: the 2026-10-27 T2 B0 and C3 arms (t2d_1b60df21) through the seat decomposition, with C3
taken from its second near capture.

    python c3_diag.py RAW --out C3.json

RAW holds t2d_<b0|c3>_<scene>_s<n>/run/{result.json, slam_control.npy, slam_log.npz}. For a C3 run whose second
capture was adopted, the decomposition uses that capture: near_attempt -> near_capture.second.capture_attempt and the
Qe check against near_capture.second.target (the target the run then drove to). So S_const is what the bias at the
second capture would leave, and dB the bias change after it.

Per run: S, dB, S_const and the split terms; the course of dB(t) from the (last) capture at 0.1 / 0.3 / 0.6 m of true
travel, at the insert entry and at lift; the yaw part of the bias change at the insert entry; the steering from the
capture to the insert entry (largest |mean of the two joints| and the summed |change|); the redraw offsets of that
capture; the outcome (phase, failure reason, release jumps in transport and settle). Per pair (same scene and seed):
which arm has the smaller |S|. Reads only; descriptive.
"""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import math
import re
from pathlib import Path

import numpy as np

_spec = importlib.util.spec_from_file_location("seat_decompose", Path(__file__).with_name("seat_decompose.py"))
SD = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(SD)

NAME_RE = re.compile(r"^t2d_(?P<arm>b0|c3)_(?P<scene>.+)_s(?P<seed>\d+)$")
COURSE_AT_M = (0.1, 0.3, 0.6)


def last_capture(result: dict) -> dict:
    """The result with the decomposition pointed at the last adopted near capture (the second one for C3)."""
    sec = (result.get("near_capture") or {}).get("second") or {}
    if sec.get("status") != "adopted":
        return result
    out = copy.deepcopy(result)
    out["near_capture"]["near_attempt"] = sec["capture_attempt"]
    out["near_capture"]["near_estimate_m"] = {"x_m": sec["target"][0], "y_m": sec["target"][1]}
    return out


def course(result: dict, sc: np.ndarray, d: dict) -> dict:
    """dB(t) by true travel from the capture, at the insert entry and at lift, and the bias change's yaw at insert."""
    cap = result["observation_attempts"][result["near_capture"]["near_attempt"] - 1]["capture_control"]
    W = SD.se2(cap["control_rear"]) @ np.linalg.inv(SD.se2(cap["truth_rear"]))
    V = d["V"]
    Vw = (W @ np.array([V[0], V[1], 1.0]))[:2]
    rows = sc[(sc[:, 0] >= cap["time_s"] - 1e-6) & (sc[:, 0] <= d["t_lift"] + 1e-6)]
    c = np.array([SD.lateral(V, x[4:7]) - SD.lateral(Vw, x[1:4]) for x in rows])
    dist = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(rows[:, 4]), np.diff(rows[:, 5])))])
    i_ins = min(int(np.searchsorted(rows[:, 0], d["t_insert"] - 1e-6)), len(rows) - 1)
    yaw0 = cap["control_rear"][2] - cap["truth_rear"][2]
    return {"leg_m": float(dist[i_ins]), "insert_m": float(dist[-1] - dist[i_ins]),
            **{f"dB_at_{m}m": float(c[min(int(np.searchsorted(dist, m)), len(c) - 1)]) for m in COURSE_AT_M},
            "dB_at_insert": float(c[i_ins]), "dB_at_lift": float(c[-1]),
            "dyaw_at_insert_rad": math.remainder(rows[i_ins, 3] - rows[i_ins, 6] - yaw0, math.tau)}


def steering(log, t0: float, t1: float) -> dict:
    stamps = np.asarray(log["joint_stamps_s"], dtype=float)
    s = np.asarray(log["steering_rad"], dtype=float)[(stamps >= t0) & (stamps <= t1)].mean(axis=1)
    return {"max_abs_rad": float(np.abs(s).max()), "sum_abs_change_rad": float(np.abs(np.diff(s)).sum())}


def outcome(result: dict) -> dict:
    holds = (result.get("slam_summary") or {}).get("holds") or []
    releases = [{"phase": h["phase"], "event": h["event"], "jump_m": h.get("jump_m"), "jump_rad": h.get("jump_rad")}
                for h in holds if "release" in h.get("event", "") and h.get("phase") in ("transport", "settle")]
    reason = result.get("failure_reason") or (result.get("tracking_failure") or {}).get("reason")
    return {"success": result.get("success"), "phase": result.get("phase"), "reason": reason, "releases": releases}


def run(folder: Path) -> dict:
    m = NAME_RE.match(folder.name)
    result = json.loads((folder / "run" / "result.json").read_text())
    sc = np.load(folder / "run" / "slam_control.npy")
    used = last_capture(result)
    d = SD.decompose(used, sc)
    nc = used["near_capture"]
    sec = (result.get("near_capture") or {}).get("second") or {}
    offsets = sec.get("offsets") if sec.get("status") == "adopted" else nc.get("offsets_to_new_line")
    cap_t = used["observation_attempts"][nc["near_attempt"] - 1]["capture_control"]["time_s"]
    with np.load(folder / "run" / "slam_log.npz") as log:
        steer = steering(log, cap_t, d["t_insert"])
    return {"name": folder.name, "arm": m["arm"], "scene": m["scene"], "seed": int(m["seed"]),
            "capture": "second" if used is not result else "first", "second_status": sec.get("status"),
            "hold": d["hold"], "terms_m": d["terms_m"], "residuals_m": d["residuals_m"],
            "course": course(used, sc, d), "steering": steer, "offsets": offsets, "outcome": outcome(result)}


def analyse(raw: Path) -> dict:
    runs = [run(f) for f in sorted(raw.iterdir()) if NAME_RE.match(f.name) and (f / "run" / "result.json").is_file()]
    by = {(r["arm"], r["scene"], r["seed"]): r for r in runs}
    pairs = []
    for (arm, scene, seed), b in sorted(by.items()):
        c = by.get(("c3", scene, seed))
        if arm != "b0" or c is None:
            continue
        sb, scc = abs(b["terms_m"]["S"]), abs(c["terms_m"]["S"])
        pairs.append({"scene": scene, "seed": seed, "S_b0_m": b["terms_m"]["S"], "S_c3_m": c["terms_m"]["S"],
                      "dB_b0_m": b["terms_m"]["dB"], "dB_c3_m": c["terms_m"]["dB"],
                      "c3": "better" if scc < sb else "worse" if scc > sb else "tie"})

    def med(arm, key):
        return float(np.median([abs(r["terms_m"][key]) for r in runs if r["arm"] == arm]))

    return {"pairs": pairs, "c3_better": sum(p["c3"] == "better" for p in pairs),
            "c3_worse": sum(p["c3"] == "worse" for p in pairs),
            "median_abs_m": {a: {k: med(a, k) for k in ("S", "dB", "S_const")} for a in ("b0", "c3")},
            "runs": runs}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("raw", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    args.out.write_text(json.dumps(SD.jsonable(analyse(args.raw)), indent=1, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
