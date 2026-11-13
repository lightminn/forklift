"""The layer budget diagnosis' records-only checks (docs/plans/2026-11-13-layer-budget-diagnosis.md 2): compact facts
from a 58-run check's run folders, for the checks (1)-(4). Reads only; no judgement.

    python lat_diag_records.py RAW_ROOT --out FACTS.json

RAW_ROOT holds <run>/run/{result.json, slam_control.npy, paths.json} copies. Per run:
- (1) the last 10 s before a tracking failure (or the run's end): SLAM pose error (estimate - truth) over the window
  and at the end, the largest one-row jump of the estimate and of the truth with its time (a discontinuity shows in
  the estimate only), the tracker builds in the window (a near capture builds every later phase's tracker at once:
  a count, not active path changes), the tracking failure's own fields, the near-capture record;
- (2) the phase entries with the truth and estimate rear pose there (slam_control at the entry time) and the
  near-capture record's status and far / near errors (not the capture pose: a failed near capture has no near_error);
- (3) the delivery docking rounds (accepted, reason, goal_error_vs_truth -- the rear-axle goal's error, not the
  pallet's), the delivery error, the pallet's seat offset in the forks at lift (slip_watch baseline), and the pallet's
  planar error to the destination at the first lower sample and at the end (from the recorded samples);
- (4) the permission's slowed reasons, each divided by its own phase's control ticks (a count before the capping
  branch: not the applied reduction or a stop's length).
Fields the run did not save are reported as missing, never filled.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

WINDOW_S = 10.0


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def pose_error(row) -> dict:
    """slam_control row [t, est x y yaw, truth x y yaw] -> position and yaw error of the estimate."""
    dx, dy = row[1] - row[4], row[2] - row[5]
    return {"pos_m": float(math.hypot(dx, dy)), "yaw_rad": float(_wrap(row[3] - row[6]))}


def window_facts(sc: np.ndarray, t_end: float) -> dict:
    w = sc[(sc[:, 0] >= t_end - WINDOW_S) & (sc[:, 0] <= t_end + 1e-9)]
    if len(w) < 2:
        return {"rows": int(len(w))}
    errs = [pose_error(r) for r in w]
    est_jump = np.hypot(np.diff(w[:, 1]), np.diff(w[:, 2]))
    tru_jump = np.hypot(np.diff(w[:, 4]), np.diff(w[:, 5]))
    i = int(est_jump.argmax())
    return {"rows": int(len(w)), "t0": float(w[0, 0]), "t1": float(w[-1, 0]),
            "pos_err_max_m": max(e["pos_m"] for e in errs), "pos_err_end_m": errs[-1]["pos_m"],
            "yaw_err_absmax_rad": max(abs(e["yaw_rad"]) for e in errs), "yaw_err_end_rad": errs[-1]["yaw_rad"],
            "est_jump_max_m": float(est_jump.max()), "est_jump_t": float(w[i + 1, 0]),
            "truth_jump_at_est_jump_m": float(tru_jump[i]), "truth_jump_max_m": float(tru_jump.max()),
            "last_row_est_m": float(est_jump[-1]), "last_row_truth_m": float(tru_jump[-1])}


def pose_at(sc: np.ndarray, t: float) -> dict | None:
    if sc is None or not len(sc):
        return None
    i = int(np.searchsorted(sc[:, 0], t))
    i = min(max(i, 0), len(sc) - 1)
    r = sc[i]
    return {"t": float(r[0]), "est": [float(v) for v in r[1:4]], "truth": [float(v) for v in r[4:7]], **pose_error(r)}


def run_facts(run: Path) -> dict:
    res = json.loads((run / "result.json").read_text())
    scp = run / "slam_control.npy"
    sc = np.load(scp) if scp.exists() else None
    tf = res.get("tracking_failure") or {}
    t_end = float(tf["time_s"]) if tf.get("time_s") is not None else (float(sc[-1, 0]) if sc is not None else None)
    out = {"success": res.get("success"), "failure_reason": res.get("failure_reason"), "missing": []}
    if sc is None:
        out["missing"].append("slam_control.npy")
    # (1)
    if t_end is not None and sc is not None:
        out["window"] = window_facts(sc, t_end)
    tv = res.get("tracker_versions") or []
    out["tracker_builds_in_window"] = ([{"serial": v.get("serial"), "phase": v.get("phase"), "installed_s": v.get("installed_s")}
                                 for v in tv if v.get("installed_s") is not None and t_end is not None
                                 and t_end - WINDOW_S <= float(v["installed_s"]) <= t_end] if tv else None)
    if tf:
        tr = tf.get("tracking") or {}
        out["tracking_failure"] = {"time_s": tf.get("time_s"), "phase": tf.get("phase"), "leg": tf.get("leg"),
                                   "failure": tr.get("failure"), "position_error_m": tr.get("position_error_m"),
                                   "yaw_error_rad": tr.get("yaw_error_rad"), "off_path": tr.get("off_path"),
                                   "at_cusp": tr.get("at_cusp"), "speed_mps": tf.get("signed_speed_mps"),
                                   "remaining_to_leg_end_m": tf.get("remaining_to_leg_end_m")}
    nc = res.get("near_capture")
    if nc:
        out["near_capture"] = {k: nc.get(k) for k in ("status", "near_error", "far_error", "box", "near_attempt",
                                                      "recoveries")}
    # (2)
    out["entries"] = [{"phase": t.get("to") or t.get("phase"), "time_s": t.get("time_s"),
                       "pose": pose_at(sc, float(t["time_s"])) if sc is not None else None}
                      for t in res.get("transitions") or []]
    # (3)
    dd = res.get("delivery_docking")
    dest = (res.get("scenario") or {}).get("destination")
    seat = ((res.get("slip_watch") or {}).get("baseline") or {}).get("seat")
    samples = res.get("samples") or []
    lower = [x for x in samples if x.get("phase") == "lower"]

    def pallet_err(x):
        p = x.get("pallet_position_m")
        return None if p is None or dest is None else float(math.hypot(p[0] - dest["x_m"], p[1] - dest["y_m"]))

    out["pallet"] = {"seat_offset_at_lift": seat,
                     "error_at_first_lower_m": pallet_err(lower[0]) if lower else None,
                     "error_at_end_m": pallet_err(samples[-1]) if samples else None}
    if dd:
        out["delivery"] = {"delivery_error_m": res.get("delivery_error_m"),
                           "goal_error_vs_truth": dd.get("goal_error_vs_truth"),
                           "slam_goal_error_vs_truth": dd.get("slam_goal_error_vs_truth"),
                           "rounds": [{k: r.get(k) for k in ("accepted", "reason", "goal_error_vs_truth",
                                                             "estimate_vs_truth_at_match")}
                                      for r in dd.get("rounds") or []]}
    else:
        out["missing"].append("delivery_docking")
    # (4)
    ol = res.get("obstacle_layer") or {}
    ticks = ol.get("control_ticks") or {}
    sr = ol.get("slowed_reasons") or {}
    out["slowed_per_phase_tick"] = {k: round(v / ticks[k.split(":")[0]], 4) for k, v in sorted(sr.items())
                                    if isinstance(ticks, dict) and ticks.get(k.split(":")[0])}
    out["control_ticks"] = ticks
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("raw_root", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    facts = {p.parent.name: run_facts(p) for p in sorted(a.raw_root.glob("*/run")) if (p / "result.json").exists()}
    a.out.write_text(json.dumps(facts, indent=1, default=str) + "\n")
    print(len(facts), "runs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
