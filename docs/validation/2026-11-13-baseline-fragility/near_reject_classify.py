"""Baseline fragility priority 2, classification only: runs that ended at the near capture, and how close the passing
runs came to the same limits.

    python near_reject_classify.py RAW_ROOT [RAW_ROOT ...] --out NEAR.json

Each RAW_ROOT holds <name>/run/result.json; repeats (_rep) and canaries (_can) are skipped. For every run with a near
capture record:
- the near capture's offsets to the new line (offsets_to_new_line: lateral, yaw), the box (0.05 m, box yaw) and, when
  the untracked re-draw recovery ran, its limit (max_lateral_m, the correction yaw limit) and refusal;
- the expected-site check of the last observation attempt: the expected site's distance from the true pallet centre,
  the nearest candidate's distance from the expected site (against radius_m) and from the true pallet centre (all in
  the truth base frame of that capture; the true centre is scenario.pickup);
- how the near observation stand was reached: a loose arrival accepted (observe_loose_arrivals, with its position and
  yaw error), a re-plan of the observe leg triggered by near_end_stand (observe_replans[].trigger) and the end-stand
  outcomes (observe_end_stands); result["near_end_stand"] itself is only the waiting state (since_s), not a re-plan;
  and the arrived estimate yaw minus the waypoint's yaw.
A run that ended at the near capture is labelled: 'perception' (the expected site was within the radius of the true
centre but no candidate near the truth was found), 'position' (the nearest candidate was within 0.02 m of the truth but
the truck stood off the line beyond the limit), or 'other'; 'by_mm' marks an exceedance of a limit by under 10 mm.
Reads only; descriptive.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

NEAR_FAILS = ("near_capture_failed", "near_capture_misaligned")
CORRECTION_MAX_YAW_RAD = 0.12  # near_tracking.CORRECTION_MAX_YAW_RAD, the re-draw's yaw limit
TRUTH_MATCH_M = 0.02
BY_MM_M = 0.010


def to_base(point, base) -> tuple[float, float]:
    x, y, yaw = base
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = point[0] - x, point[1] - y
    return c * dx + s * dy, -s * dx + c * dy


def truth_base(result: dict, attempt: dict) -> tuple[float, float, float] | None:
    cap = attempt.get("capture_control") or {}
    tr = cap.get("truth_rear")
    if not tr:
        return None
    off = abs(float(result["arguments"]["rear_axle_offset_m"]))
    return tr[0] + off * math.cos(tr[2]), tr[1] + off * math.sin(tr[2]), tr[2]


def classify(name: str, result: dict) -> dict | None:
    nc = result.get("near_capture") or {}
    if not nc or not result.get("observation_attempts"):
        return None
    reason = result.get("failure_reason") or (result.get("tracking_failure") or {}).get("reason") or ""
    attempt = result["observation_attempts"][-1]
    out: dict = {"name": name, "status": nc.get("status"), "failure": reason or None,
                 "ended_at_near": any(str(reason).startswith(k) for k in NEAR_FAILS)}
    off = nc.get("offsets_to_new_line")
    if off:
        out["offset_lateral_m"], out["offset_yaw_rad"] = off["lateral_m"], off["yaw_rad"]
    box = nc.get("box")
    if box:
        out["box"] = box
    redraw = [r for r in nc.get("recoveries") or [] if r.get("stage") == "redraw"]
    if redraw:
        last = redraw[-1]
        out["redraw"] = {"accepted": last.get("accepted"), "max_lateral_m": last.get("max_lateral_m"),
                         "max_yaw_rad": CORRECTION_MAX_YAW_RAD, "refused": (last.get("record") or {}).get("refused")}
    tb = truth_base(result, attempt)
    pk = result["scenario"]["pickup"]
    expected = attempt.get("expected") or {}
    if tb is not None and expected.get("site"):
        tp = to_base((pk["x_m"], pk["y_m"]), tb)
        site = expected["site"]
        cands = sorted(expected.get("candidates") or [], key=lambda c: c["distance_m"])
        out["site_vs_truth_m"] = math.hypot(site[0] - tp[0], site[1] - tp[1])
        out["radius_m"] = expected.get("radius_m")
        if cands:
            out["nearest_candidate_vs_site_m"] = cands[0]["distance_m"]
            out["nearest_candidate_vs_truth_m"] = math.hypot(cands[0]["centre_m"][0] - tp[0],
                                                             cands[0]["centre_m"][1] - tp[1])
            out["candidate_vs_truth_min_m"] = min(math.hypot(c["centre_m"][0] - tp[0], c["centre_m"][1] - tp[1])
                                                  for c in cands)
    out["near_end_stand_waiting"] = bool(result.get("near_end_stand"))
    cap_t = attempt.get("t_before_capture_s")
    loose = [a for a in result.get("observe_loose_arrivals") or []
             if cap_t is None or a.get("time_s", 0.0) <= cap_t + 1e-6]
    out["loose_arrival"] = bool(loose)
    if loose:
        out["loose_position_error_m"] = loose[-1].get("position_error_m")
        out["loose_yaw_error_rad"] = loose[-1].get("yaw_error_rad")
    out["near_end_stand_replans"] = sum(1 for r in result.get("observe_replans") or []
                                        if r.get("trigger") == "near_end_stand")
    out["end_stand_outcomes"] = [e.get("outcome") for e in result.get("observe_end_stands") or []]
    wp, arr = nc.get("waypoint"), attempt.get("arrived_pose")
    if wp and arr:
        out["arrival_yaw_minus_waypoint_rad"] = math.remainder(arr[2] - wp[2], math.tau)
    if out["ended_at_near"]:
        out["label"], out["by_mm"] = label(out)
    return out


def label(c: dict) -> tuple[str, bool]:
    if "no_expected_candidate" in (c.get("failure") or ""):
        near = c.get("candidate_vs_truth_min_m")
        exceed = (c.get("nearest_candidate_vs_site_m") or 0.0) - (c.get("radius_m") or 0.0)
        if c.get("site_vs_truth_m", 1.0) <= (c.get("radius_m") or 0.0) and (near is None or near > c["radius_m"]):
            return "perception", 0 < exceed < BY_MM_M
        return "other", 0 < exceed < BY_MM_M
    if "misaligned" in (c.get("failure") or ""):
        near = c.get("nearest_candidate_vs_truth_m")
        good = near is not None and near <= TRUTH_MATCH_M
        lim = (c.get("redraw") or {}).get("max_lateral_m") or (c.get("box") or {}).get("lateral_m") or 0.0
        exceed = abs(c.get("offset_lateral_m") or 0.0) - lim
        return ("position" if good else "other"), 0 < exceed < BY_MM_M
    return "other", False


def median(values: list) -> float:
    v = sorted(values)
    n = len(v)
    return v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2


def scene_of(name: str) -> str:
    body = name[len("lat_r_"):] if name.startswith("lat_r_") else name
    return body.split("_", 2)[2].rsplit("_s", 1)[0]


def analyse(roots: list[Path]) -> dict:
    runs = []
    for root in roots:
        for folder in sorted(p for p in root.iterdir() if (p / "run" / "result.json").is_file()):
            if folder.name.endswith(("_rep", "_can")):
                continue
            c = classify(folder.name, json.loads((folder / "run" / "result.json").read_text()))
            if c is not None:
                c["scene"], c["root"] = scene_of(folder.name), root.name
                runs.append(c)
    summary: dict = {}
    for r in runs:
        if "offset_lateral_m" not in r:
            continue
        key = f"{r['scene']}|loose_arrival={r['loose_arrival']}"
        s = summary.setdefault(key, {"n": 0, "rejected": 0, "lat": [], "yaw_arrival": []})
        s["n"] += 1
        s["rejected"] += int(r["ended_at_near"])
        s["lat"].append(abs(r["offset_lateral_m"]))
        if "arrival_yaw_minus_waypoint_rad" in r:
            s["yaw_arrival"].append(abs(r["arrival_yaw_minus_waypoint_rad"]))
    for s in summary.values():
        lat, ya = sorted(s.pop("lat")), sorted(s.pop("yaw_arrival"))
        s.update(lateral_median_m=median(lat), lateral_max_m=lat[-1],
                 over_0_05=sum(v > 0.05 for v in lat), over_0_08=sum(v > 0.08 for v in lat),
                 over_0_10=sum(v > 0.10 for v in lat),
                 arrival_yaw_median_rad=median(ya) if ya else None, arrival_yaw_max_rad=ya[-1] if ya else None)
    return {"ended_at_near": [r for r in runs if r["ended_at_near"]], "by_scene": summary, "runs": runs}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("roots", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    args.out.write_text(json.dumps(analyse(args.roots), indent=1, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
