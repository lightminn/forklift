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
- the lateral offset to the new line split into the waypoint's own offset from the new line (the far estimate -> near
  observation change of the target) and the truck's miss of the waypoint, and the observe path's end (curvature,
  direction, poses; result.paths.observe is the last version);
- how the near observation stand was reached: a loose arrival accepted on the near observation leg (observe_loose_arrivals
  after the far capture began and up to the near capture, with its position and yaw error), a re-plan of the observe leg triggered by near_end_stand (observe_replans[].trigger) and the end-stand
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
    # the near observation leg only: after the far capture began, up to the near capture (a loose arrival at the far
    # waypoint is a different leg, Codex round 5 P2)
    cap_t = attempt.get("t_before_capture_s")
    far_no = nc.get("far_attempt")
    attempts = result["observation_attempts"]
    far_t = (attempts[far_no - 1].get("t_before_capture_s")
             if isinstance(far_no, int) and 0 < far_no <= len(attempts) else None)
    loose = [a for a in result.get("observe_loose_arrivals") or []
             if (far_t is None or a.get("time_s", 0.0) > far_t + 1e-6)
             and (cap_t is None or a.get("time_s", 0.0) <= cap_t + 1e-6)]
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
    attempts = result["observation_attempts"]
    target = new_line_target(result, nc)
    if wp and arr and target and off:
        # the lateral offset to the new line, split: the waypoint's own offset from the new line (the far estimate ->
        # near observation change of the target) plus the truck's miss of the waypoint, both across the new line
        out["waypoint_off_new_line_m"] = lateral(wp, target)
        out["arrival_miss_across_new_line_m"] = lateral(arr, target) - lateral(wp, target)
        out["split_check_m"] = lateral(arr, target) - off["lateral_m"]
        # The waypoint share further split (Codex review): the waypoint was set in the far capture's estimate; with
        # W = estimate * truth^-1 at each capture, W_near * W_far^-1 moves it into the near capture's bias. The
        # remainder is what the two observations disagree on once the bias change is taken out.
        far_no = nc.get("far_attempt")
        far_cap = (attempts[far_no - 1].get("capture_control") if isinstance(far_no, int) and 0 < far_no <= len(attempts)
                   else None) or {}
        near_cap = attempt.get("capture_control") or {}
        if far_cap.get("truth_rear") and near_cap.get("truth_rear"):
            moved = bias_move(wp, far_cap, near_cap)
            out["waypoint_share_after_bias_change_m"] = lateral(moved, target)
            out["waypoint_share_bias_change_m"] = lateral(wp, target) - lateral(moved, target)
    path = (result.get("paths") or {}).get("observe") or {}
    if path.get("curvatures_inv_m"):
        out["observe_path_end"] = {"curvature_inv_m": path["curvatures_inv_m"][-1],
                                   "direction": (path.get("directions") or [None])[-1],
                                   "poses": len(path.get("poses") or [])}
    if out["ended_at_near"]:
        out["label"], out["by_mm"] = label(out)
    return out


def lateral(p, frame) -> float:
    return -math.sin(frame[2]) * (p[0] - frame[0]) + math.cos(frame[2]) * (p[1] - frame[1])


def se2(p):
    c, s_ = math.cos(p[2]), math.sin(p[2])
    return [[c, -s_, p[0]], [s_, c, p[1]], [0.0, 0.0, 1.0]]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def inv(m):
    c, s_, x, y = m[0][0], m[1][0], m[0][2], m[1][2]
    return [[c, s_, -(c * x + s_ * y)], [-s_, c, s_ * x - c * y], [0.0, 0.0, 1.0]]


def bias_move(pose, far_cap: dict, near_cap: dict) -> tuple[float, float, float]:
    """A pose expressed in the far capture's estimate, moved into the near capture's: W_near * W_far^-1 * pose with
    W = se2(control_rear) * se2(truth_rear)^-1 at each capture."""
    w_far = mat_mul(se2(far_cap["control_rear"]), inv(se2(far_cap["truth_rear"])))
    w_near = mat_mul(se2(near_cap["control_rear"]), inv(se2(near_cap["truth_rear"])))
    m = mat_mul(mat_mul(w_near, inv(w_far)), se2(pose))
    return m[0][2], m[1][2], math.atan2(m[1][0], m[0][0])


def new_line_target(result: dict, nc: dict):
    """The near observation's target the new line was drawn to (the perception estimate, or the last re-draw's)."""
    pe = (result.get("perception") or {}).get("perception_pickup_estimate_m")
    if pe:
        return pe["x_m"], pe["y_m"], pe["yaw_rad"]
    redraw = [r for r in nc.get("recoveries") or [] if r.get("target")]
    return tuple(redraw[-1]["target"]) if redraw else None


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
