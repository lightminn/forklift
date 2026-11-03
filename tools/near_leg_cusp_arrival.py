"""Near-leg cusp arrival comparison (lead scope 2026-11-03 after docs/validation/2026-11-03-near-leg-cusp-miss-diagnosis.md):
records only, refused (K3 stall, near-confirmed) vs accepted near-leg cusps, at the stand on the cusp, in the cusp's
frame -- truth-path = (estimate-path) + (truth-estimate), signed along / lateral / heading. Not a contribution split:
estimate-path also holds the motion that answered earlier corrections.

Near-leg trackers are identified positively: those installed at a near-serial replan (within a tick) and the one
those replans replaced first. Runs without a near observe_miss replan have none and are reported as not covered.

    python tools/near_leg_cusp_arrival.py OUT.json BATCH_DIR [BATCH_DIR ...]   (numpy; run/slam_control.npy,
    run/slam_records.json and result.json samples are read where present)"""

from __future__ import annotations

import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import near_leg_cusp_miss as miss  # noqa: E402

TICK_S = miss.TICK_S
WINDOW_S = 3.0  # corrections and steering are read over this before the stand


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def in_frame(pose, ref) -> tuple[float, float, float]:
    """pose - ref in ref's frame: (along, lateral, heading)."""
    c, s = math.cos(ref[2]), math.sin(ref[2])
    dx, dy = pose[0] - ref[0], pose[1] - ref[1]
    return dx * c + dy * s, -dx * s + dy * c, wrap(pose[2] - ref[2])


def near_trackers(s: dict, near_serials: set) -> dict:
    """book serial -> tracker version, for the near-leg trackers this rule can prove."""
    versions = [v for v in s.get("tracker_versions") or [] if v.get("phase") == "observe"]
    times = sorted(float(r["time_s"]) for r in s.get("observe_replans") or [] if r.get("leg_serial") in near_serials)
    out = {}
    for v in versions:
        inst, repl = v.get("installed_s"), v.get("replaced_s")
        if inst is not None and any(abs(float(inst) - t) <= TICK_S + miss.EPS_S for t in times):
            out[v["serial"]] = v
        elif times and repl is not None and abs(float(repl) - times[0]) <= TICK_S + miss.EPS_S:
            out[v["serial"]] = v
    return out


def at(C: np.ndarray, t: float) -> tuple[list, list]:
    i = int(np.clip(np.searchsorted(C[:, 0], t), 0, len(C) - 1))
    return C[i, 1:4].tolist(), C[i, 4:7].tolist()


def corrections(records: list, t: float, cusp_pose=None) -> dict:
    """The applied map-from-odom over WINDOW_S before t: its largest single step and its net change, the net
    translation also in the cusp's frame (its lateral sign against the stand's lateral error)."""
    win = [r for r in records if t - WINDOW_S <= float(r["stamp_s"]) <= t and r.get("applied_map_from_odom")]
    if len(win) < 2:
        return {"n": len(win)}
    a = np.array([r["applied_map_from_odom"] for r in win], dtype=float)
    d = np.diff(a, axis=0)
    step = np.hypot(d[:, 0], d[:, 1])
    k = int(np.argmax(step))
    out = {"n": len(win), "max_step_m": float(step[k]), "max_step_yaw_rad": float(abs(d[k, 2])),
           "max_step_before_s": float(t - float(win[k + 1]["stamp_s"])),
           "net_m": float(math.hypot(*(a[-1, :2] - a[0, :2]))), "net_yaw_rad": float(abs(wrap(a[-1, 2] - a[0, 2])))}
    if cusp_pose is not None:
        c, s_ = math.cos(cusp_pose[2]), math.sin(cusp_pose[2])
        n = a[-1, :2] - a[0, :2]
        out["net_along_m"], out["net_lateral_m"] = float(n[0] * c + n[1] * s_), float(-n[0] * s_ + n[1] * c)
        out["net_yaw_signed_rad"] = float(wrap(a[-1, 2] - a[0, 2]))
    return out


def steering(samples: list, t: float) -> dict:
    """Command - actual over WINDOW_S before t: the mean (a steady loss or a side bias) and the largest gap."""
    w = [x for x in samples if t - WINDOW_S <= float(x["time_s"]) <= t and x.get("steering_command_rad")
         and x.get("steering_actual_rad")]
    if not w:
        return {"n": 0}
    d = np.array([np.subtract(x["steering_command_rad"], x["steering_actual_rad"]) for x in w], dtype=float)
    return {"n": len(w), "mean_left_rad": float(d[:, 0].mean()), "mean_right_rad": float(d[:, 1].mean()),
            "max_abs_rad": float(np.abs(d).max()),
            "speed_cmd_last_mps": float(w[-1].get("speed_command_mps") or 0.0),
            "speed_last_mps": float(w[-1].get("signed_speed_mps") or 0.0)}


def run_events(rundir: str, name: str) -> dict:
    s = json.load(open(os.path.join(rundir, "result.json")))
    have = {k: os.path.exists(os.path.join(rundir, k)) for k in ("slam_control.npy", "slam_records.json", "paths.json")}
    a = miss.analyse(s, name)
    near_serials = set(a["near_serials"])
    trackers = near_trackers(s, near_serials)
    out = {"run": name, "inputs": have, "near_trackers": sorted(trackers), "events": []}
    if not have["slam_control.npy"] or not trackers:
        return out
    C = np.load(os.path.join(rundir, "slam_control.npy"))
    records = json.load(open(os.path.join(rundir, "slam_records.json"))) if have["slam_records.json"] else []
    samples = s.get("samples") or []
    armed = ((s.get("near_end_injections") or {}).get("windows") or {}).get("injection")
    armed_s = None if not armed else float(armed[0])

    def around_step(t, corr, cusp_pose):
        """truth - estimate in the cusp's frame 0.1 s before and after the window's largest correction step."""
        if "max_step_before_s" not in corr:
            return None
        ts = t - corr["max_step_before_s"]
        out_ = {}
        for key, tt in (("before", ts - 0.1), ("after", ts + 0.1)):
            e_, tr_ = at(C, tt)
            a_, b_ = in_frame(tr_, cusp_pose), in_frame(e_, cusp_pose)
            out_[key] = [a_[0] - b_[0], a_[1] - b_[1], wrap(a_[2] - b_[2])]
        return out_

    def event(kind, t, cusp_pose, extra):
        est, tru = at(C, t)
        tp, ep, te = in_frame(tru, cusp_pose), in_frame(est, cusp_pose), in_frame(tru, est)
        corr = corrections(records, t, cusp_pose)
        out["events"].append({"kind": kind, "time_s": t, "truth_minus_path": tp, "estimate_minus_path": ep,
                              "truth_minus_estimate_in_cusp_frame": [tp[0] - ep[0], tp[1] - ep[1], wrap(tp[2] - ep[2])],
                              "truth_minus_estimate_own": te, "corrections": corr,
                              "truth_minus_estimate_around_step": around_step(t, corr, cusp_pose),
                              "steering": steering(samples, t), **extra})
    for w in s.get("leg_switches") or []:  # accepted near-leg cusps
        if (w.get("phase") == "observe" and w.get("via") in ("arrival", "release") and w.get("serial") in trackers
                and (armed_s is None or float(w["time_s"]) < armed_s)):
            event("accepted_" + w["via"], float(w["time_s"]), w["endpoint_pose"],
                  {"tracker": w["serial"], "position_error_m": w.get("position_error_m")})
    for r, st in zip(a["stalls"], [x for x in s.get("cusp_stalls") or [] if x.get("phase") == "observe"
                                   and (armed_s is None or float(x["time_s"]) < armed_s)]):
        if r["role"] != "near" or r["reason"] in ("released",):
            continue
        t = float(st["time_s"])
        owner = [v for v in trackers.values() if v.get("installed_s") is not None and float(v["installed_s"]) <= t
                 and (v.get("replaced_s") is None or t < float(v["replaced_s"]) + TICK_S)
                 or v.get("installed_s") is None and v.get("replaced_s") is not None and t < float(v["replaced_s"])]
        if len(owner) != 1:
            continue
        cusps = sorted(owner[0].get("cusps") or [], key=lambda c: c["index"])
        k = st.get("leg_index")
        if k is None or k >= len(cusps):
            continue
        event("refused_" + r["reason"], t, cusps[k]["pose"], {"tracker": owner[0]["serial"],
                                                               "last_scan_error": [r["last_position_error_m"],
                                                                                   r["last_yaw_error_rad"]]})
    return out


def main(argv: list) -> None:
    out_path, batches = argv[1], argv[2:]
    runs = []
    for b in batches:
        for d in sorted(glob.glob(os.path.join(b, "*random_1*", "run"))):
            if os.path.exists(os.path.join(d, "result.json")):
                runs.append(run_events(d, f"{os.path.basename(os.path.normpath(b))}/{d.split(os.sep)[-2]}"))
    ev = [e for r in runs for e in r["events"]]
    summary = {"runs": len(runs), "with_control": sum(r["inputs"]["slam_control.npy"] for r in runs),
               "with_near_trackers": sum(bool(r["near_trackers"]) for r in runs), "events": {}}
    for kind in sorted({e["kind"] for e in ev}):
        es = [e for e in ev if e["kind"] == kind]
        g = lambda f: np.array([f(e) for e in es], dtype=float)
        stat = lambda x: [round(float(np.min(x)), 4), round(float(np.median(x)), 4), round(float(np.max(x)), 4)]
        summary["events"][kind] = {
            "n": len(es),
            "truth_path_lateral_m": stat(g(lambda e: e["truth_minus_path"][1])),
            "truth_path_heading_rad": stat(g(lambda e: e["truth_minus_path"][2])),
            "est_path_lateral_m": stat(g(lambda e: e["estimate_minus_path"][1])),
            "est_path_heading_rad": stat(g(lambda e: e["estimate_minus_path"][2])),
            "truth_est_lateral_m": stat(g(lambda e: e["truth_minus_estimate_in_cusp_frame"][1])),
            "truth_est_heading_rad": stat(g(lambda e: e["truth_minus_estimate_in_cusp_frame"][2])),
            "corr_max_step_m": stat(g(lambda e: e["corrections"].get("max_step_m", float("nan")))),
            "steer_mean_left_rad": stat(g(lambda e: e["steering"].get("mean_left_rad", float("nan")))),
            "steer_mean_right_rad": stat(g(lambda e: e["steering"].get("mean_right_rad", float("nan")))),
        }
    json.dump({"summary": summary, "runs": runs}, open(out_path, "w"), indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main(sys.argv)
