"""Baseline fragility, priority 1 (docs/plans/2026-11-13-baseline-fragility.md 1, 3, 3.1): the pallet's lateral seat
offset at lift, split records-only by pose transforms at the same time and the same pallet origin.

    python seat_decompose.py RAW_ROOT [RAW_ROOT ...] --out DECOMP.json [--summary SUMMARY.md]

Each RAW_ROOT holds <name>/run/{result.json, slam_control.npy} copies; <name> is <prefix>_<arm>_<scene>_s<seed>.

g(X) is the lateral coordinate of the true pallet origin V (at the first lift sample) in pose X. A and E are the true
and estimated rear poses at the insert -> lift transition (slam_control columns: time, estimate x, y, yaw, truth x, y,
yaw), G is the insert path's end (result.paths.insert).

    S = g(A) = T + F + L,   T = g(G),   F = g(E) - g(G),   L = g(A) - g(E)
    T = D + C + P,   D = n.(V - Qd),   C = n.(Qd - Qe),   P = n.(Qe - G.xy)

n is G's left unit vector. q is the near capture's observed pallet centre in base_link (the pocket front midpoint
moved half a pallet depth along the observed insertion yaw, as sim/isaac/perception_adapter.py does); Qd and Qe are q
placed with the capture's true and estimated base pose (the capture_control rear pose moved |rear_axle_offset_m|
forward). Qe is checked against near_capture.near_estimate_m. S is checked against slip_watch.baseline.seat.

C + L is also reported, descriptively and outside the judgement: the change of the localisation error's lateral
effect from the capture to lift (the two terms cancel when the estimate's bias is the same at both times).

Runs that never reached lift are outside the data (plan 2) and are counted apart, not held. Holds (3.1): a term that cannot be computed, a residual (seat - g(A), T - (D + C + P), seat - leaf sum) above
max(2 mm, 0.1 |S|), pallet motion above 0.5 mm or 1 mrad from the first sample after the capture to the first lift
sample, an insert tracker replaced during insert, or Qe off near_estimate_m by more than 1e-6 m. Held runs stay in the
output with their reasons and are left out of the tallies. Nothing is judged beyond the plan's tally; no thresholds
change.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np

SELECT_M = 0.010
SENSITIVITY_M = (0.005, 0.010, 0.015)
RESIDUAL_FLOOR_M = 0.002
RESIDUAL_FRACTION = 0.1
MOTION_M = 0.0005
MOTION_RAD = 0.001
QE_TOL_M = 1e-6
NEGLIGIBLE_M = 1e-4  # the tables' 0.1 mm resolution: a smaller term is neither same-sign nor cancelling
LEAVES = ("D", "C", "P", "F", "L")
TOPS = ("T", "F", "L")
NAME_RE = re.compile(r"^(?P<prefix>[^_]+)_(?P<arm>[^_]+)_(?P<scene>.+)_s(?P<seed>\d+)$")


def lateral(point, pose) -> float:
    """The lateral (left +) coordinate of a world point in a planar pose (x, y, yaw)."""
    x, y, yaw = (float(v) for v in pose[:3])
    return -math.sin(yaw) * (float(point[0]) - x) + math.cos(yaw) * (float(point[1]) - y)


def base_from_rear(rear, offset_m: float) -> tuple[float, float, float]:
    """The base_link pose |offset| ahead of a rear-axle pose, same yaw (mission core's capture pose)."""
    x, y, yaw = (float(v) for v in rear[:3])
    d = abs(float(offset_m))
    return x + d * math.cos(yaw), y + d * math.sin(yaw), yaw


def observed_centre(observation: dict, depth_m: float) -> tuple[float, float]:
    """perception_adapter.estimate_pallet_center_m on a saved observation record."""
    left, right = observation["left"]["center_m"], observation["right"]["center_m"]
    yaw = float(observation["insertion_yaw_rad"])
    return ((left[0] + right[0]) / 2 + depth_m / 2 * math.cos(yaw),
            (left[1] + right[1]) / 2 + depth_m / 2 * math.sin(yaw))


def to_world(q, base) -> tuple[float, float]:
    """perception_adapter.estimate_world_pallet_site's position."""
    x, y, yaw = base
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * q[0] - s * q[1], y + s * q[0] + c * q[1]


def row_at(sc: np.ndarray, t: float) -> np.ndarray:
    return sc[int(np.argmin(np.abs(sc[:, 0] - t)))]


def transition_time(result: dict, to: str) -> float | None:
    times = [float(t["time_s"]) for t in result.get("transitions") or [] if t.get("to") == to]
    return times[0] if times else None


def pallet_motion(samples: list, t_from: float, t_lift: float) -> dict | None:
    """Largest planar move and yaw change of the true pallet from the first sample after t_from to the first lift
    sample, against the first of them."""
    window = [s for s in samples if s["time_s"] > t_from]
    end = next((i for i, s in enumerate(window) if s.get("phase") == "lift" and s["time_s"] >= t_lift), None)
    if not window or end is None:
        return None
    window = window[: end + 1]
    x0, y0 = window[0]["pallet_position_m"][:2]
    yaw0 = window[0]["pallet_yaw_rad"]
    return {
        "from_t": window[0]["time_s"], "to_t": window[-1]["time_s"], "samples": len(window),
        "max_move_m": max(math.hypot(s["pallet_position_m"][0] - x0, s["pallet_position_m"][1] - y0) for s in window),
        "max_yaw_rad": max(abs(math.remainder(s["pallet_yaw_rad"] - yaw0, math.tau)) for s in window),
    }


def insert_trackers(result: dict, t_insert: float, t_lift: float) -> int:
    """Insert-phase tracker versions alive at any time during insert."""
    n = 0
    for v in result.get("tracker_versions") or []:
        if v.get("phase") != "insert":
            continue
        installed = v.get("installed_s")
        replaced = v.get("replaced_s")
        if (installed is None or installed <= t_lift) and (replaced is None or replaced > t_insert):
            n += 1
    return n


def decompose(result: dict, sc: np.ndarray) -> dict:
    """One run's terms, residuals, checks and hold reasons. Missing fields are reported, never filled."""
    out: dict = {"missing": [], "hold": []}
    t_insert, t_lift = transition_time(result, "insert"), transition_time(result, "lift")
    if t_insert is None or t_lift is None:
        out["missing"].append("insert_or_lift_transition")
        out["reached_lift"] = False
        return out
    out["reached_lift"] = True
    out["t_insert"], out["t_lift"] = t_insert, t_lift
    lift_sample = next((s for s in result.get("samples") or [] if s.get("phase") == "lift" and s["time_s"] >= t_lift),
                       None)
    path = (result.get("paths") or {}).get("insert") or {}
    seat = ((result.get("slip_watch") or {}).get("baseline") or {}).get("seat")
    if lift_sample is None:
        out["missing"].append("lift_sample")
    if not path.get("poses"):
        out["missing"].append("insert_path")
    if seat is None:
        out["missing"].append("seat")
    if lift_sample is None or not path.get("poses"):
        out["hold"].append("missing_terms")
        return out
    V = lift_sample["pallet_position_m"][:2]
    G = path["poses"][-1]
    row = row_at(sc, t_lift)
    out["slam_row_dt_s"] = float(row[0] - t_lift)
    E, A = row[1:4], row[4:7]
    g_A, g_E, g_G = lateral(V, A), lateral(V, E), lateral(V, G)
    terms = {"S": g_A, "T": g_G, "F": g_E - g_G, "L": g_A - g_E}
    out["V"], out["G"], out["E"], out["A"] = [float(v) for v in V], [float(v) for v in G[:3]], \
        [float(v) for v in E], [float(v) for v in A]

    near = result.get("near_capture") or {}
    attempt_no = near.get("near_attempt")
    attempts = result.get("observation_attempts") or []
    offset = (result.get("arguments") or {}).get("rear_axle_offset_m")
    depth = (result.get("geometry") or {}).get("pallet_depth_m")
    attempt = attempts[attempt_no - 1] if isinstance(attempt_no, int) and 0 < attempt_no <= len(attempts) else None
    cap = (attempt or {}).get("capture_control") or {}
    obs = (attempt or {}).get("pocket_observation")
    if attempt is None:
        out["missing"].append("near_attempt")
    elif not cap.get("truth_rear") or not cap.get("control_rear"):
        out["missing"].append("capture_control")
    elif not obs or obs.get("status") != "valid":
        out["missing"].append("pocket_observation")
    elif offset is None or depth is None:
        out["missing"].append("offset_or_depth")
    else:
        q = observed_centre(obs, float(depth))
        Qd = to_world(q, base_from_rear(cap["truth_rear"], offset))
        Qe = to_world(q, base_from_rear(cap["control_rear"], offset))
        yaw = float(G[2])
        n = (-math.sin(yaw), math.cos(yaw))
        dot = lambda a, b: n[0] * (a[0] - b[0]) + n[1] * (a[1] - b[1])  # noqa: E731
        terms.update(D=dot(V, Qd), C=dot(Qd, Qe), P=dot(Qe, G))
        est = near.get("near_estimate_m")
        if est is not None:
            out["qe_vs_near_estimate_m"] = math.hypot(Qe[0] - est["x_m"], Qe[1] - est["y_m"])
            if out["qe_vs_near_estimate_m"] > QE_TOL_M:
                out["hold"].append("qe_mismatch")
        else:
            out["missing"].append("near_estimate_m")
        out["capture_t"] = cap.get("time_s")
    out["terms_m"] = terms
    if "C" in terms:
        out["C_plus_L_m"] = terms["C"] + terms["L"]
    S = terms["S"]
    bound = max(RESIDUAL_FLOOR_M, RESIDUAL_FRACTION * abs(S))
    out["residual_bound_m"] = bound
    res = {}
    if seat is not None:
        res["seat_minus_S"] = float(seat[1]) - S
        out["seat_lateral_m"] = float(seat[1])
    if all(k in terms for k in ("D", "C", "P")):
        res["T_minus_DCP"] = terms["T"] - (terms["D"] + terms["C"] + terms["P"])
        if seat is not None:
            res["seat_minus_leaves"] = float(seat[1]) - sum(terms[k] for k in LEAVES)
    out["residuals_m"] = res
    if any(abs(v) > bound for v in res.values()):
        out["hold"].append("residual")
    if out["missing"]:
        out["hold"].append("missing_terms")

    if out.get("capture_t") is not None:
        motion = pallet_motion(result.get("samples") or [], float(out["capture_t"]), t_lift)
        out["pallet_motion"] = motion
        if motion is None:
            out["hold"].append("missing_terms")
        elif motion["max_move_m"] > MOTION_M or motion["max_yaw_rad"] > MOTION_RAD:
            out["hold"].append("pallet_moved")
    out["insert_trackers"] = insert_trackers(result, t_insert, t_lift)
    if out["insert_trackers"] != 1:
        out["hold"].append("insert_tracker_replaced")
    out["hold"] = sorted(set(out["hold"]))
    return out


def contributions(terms: dict, keys) -> dict:
    """The largest term with S's sign, and the terms of the opposite sign (3.1); terms under NEGLIGIBLE_M are
    neither."""
    S = terms["S"]
    keys = [k for k in keys if abs(terms[k]) >= NEGLIGIBLE_M]
    same = [(k, terms[k]) for k in keys if terms[k] * S > 0]
    largest = max(same, key=lambda kv: abs(kv[1]))[0] if same else None
    return {"largest_same_sign": largest, "cancelling": {k: terms[k] for k in keys if terms[k] * S < 0}}


def judge(decomp: dict, select_m: float = SELECT_M) -> dict:
    """The plan 3.1 per-run reading: selected or not, held or judged, and the contribution if judged."""
    terms = decomp.get("terms_m") or {}
    if decomp.get("reached_lift") is False:
        return {"selected": None, "status": "not_reached_lift"}
    if "S" not in terms:
        return {"selected": None, "status": "held", "reasons": decomp.get("hold") or ["missing_terms"]}
    selected = abs(terms["S"]) > select_m
    if not selected:
        return {"selected": False, "status": "not_selected"}
    if decomp.get("hold"):
        return {"selected": True, "status": "held", "reasons": decomp["hold"]}
    return {"selected": True, "status": "judged", "leaf": contributions(terms, LEAVES),
            "top": contributions(terms, TOPS)}


def parse_name(name: str) -> dict:
    m = NAME_RE.match(name)
    return {"arm": m["arm"], "scene": m["scene"], "seed": int(m["seed"])} if m else {"arm": None, "scene": None,
                                                                                    "seed": None}


def arm_group(arm: str | None) -> str:
    return "B0" if arm == "B0" else "perturbed"


def tally(runs: list[dict], select_m: float) -> dict:
    """Counts by arm group: selected, held (with reasons), judged, the largest same-sign leaf, and per-term signed
    medians and largest magnitudes over the judged runs."""
    groups: dict = {}
    for r in runs:
        j = judge(r, select_m)
        g = groups.setdefault(arm_group(r.get("arm")), {"runs": 0, "not_reached_lift": 0, "selected": 0, "held": 0,
                                                        "judged": 0,
                                                        "held_reasons": {}, "largest_same_sign": {},
                                                        "_terms": {k: [] for k in LEAVES + ("T", "S", "C+L")}})
        g["runs"] += 1
        if j["status"] == "not_reached_lift":
            g["not_reached_lift"] += 1
            continue
        if j["status"] == "not_selected":
            continue
        if j["selected"]:
            g["selected"] += 1
        if j["status"] == "held":
            g["held"] += 1
            for reason in j["reasons"]:
                g["held_reasons"][reason] = g["held_reasons"].get(reason, 0) + 1
            continue
        g["judged"] += 1
        key = j["leaf"]["largest_same_sign"]
        g["largest_same_sign"][key] = g["largest_same_sign"].get(key, 0) + 1
        for k in g["_terms"]:
            g["_terms"][k].append(r["C_plus_L_m"] if k == "C+L" else r["terms_m"][k])
    for g in groups.values():
        vals = g.pop("_terms")
        g["terms_m"] = {k: {"median": float(np.median(v)), "max_abs_signed": float(max(v, key=abs))}
                        for k, v in vals.items() if v}
    return groups


def load_run(run_dir: Path) -> tuple[dict, np.ndarray]:
    return json.loads((run_dir / "result.json").read_text()), np.load(run_dir / "slam_control.npy")


def collect(roots: list[Path]) -> list[dict]:
    runs = []
    for root in roots:
        for folder in sorted(p for p in root.iterdir() if (p / "run" / "result.json").is_file()):
            result, sc = load_run(folder / "run")
            d = decompose(result, sc)
            d.update(name=folder.name, root=str(root), **parse_name(folder.name))
            d["judgement"] = judge(d)
            runs.append(d)
    return runs


def mm(v) -> str:
    return "—" if v is None else f"{1000 * v:+.1f}"


def summary_md(runs: list[dict]) -> str:
    lines = ["# Seat decomposition (records only)", "",
             f"Runs read: {len(runs)}. Terms in mm; S = T + F + L, T = D + C + P.", ""]
    for sel in SENSITIVITY_M:
        lines.append(f"## |S| > {1000 * sel:.0f} mm")
        lines.append("")
        lines.append("| group | runs | no lift | selected | held | judged | largest same-sign leaf | held reasons |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for name, g in sorted(tally(runs, sel).items()):
            lines.append(f"| {name} | {g['runs']} | {g['not_reached_lift']} | {g['selected']} | {g['held']} | {g['judged']} | "
                         f"{json.dumps(g['largest_same_sign'], sort_keys=True)} | "
                         f"{json.dumps(g['held_reasons'], sort_keys=True)} |")
        lines.append("")
    lines += ["## Runs", "", "| run | S | T | D | C | P | F | L | C+L | seat | judgement |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in runs:
        t = r.get("terms_m") or {}
        j = r["judgement"]
        verdict = (j["status"] if j["status"] != "judged"
                   else f"judged: {j['leaf']['largest_same_sign']}; cancelling "
                        f"{','.join(j['leaf']['cancelling']) or '-'}")
        if j["status"] == "held":
            verdict += f" ({','.join(j['reasons'])})"
        lines.append(f"| {r['name']} | " + " | ".join(mm(t.get(k)) for k in ("S", "T", "D", "C", "P", "F", "L"))
                     + f" | {mm(r.get('C_plus_L_m'))} | {mm(r.get('seat_lateral_m'))} | {verdict} |")
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("roots", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--summary", type=Path)
    args = ap.parse_args(argv)
    runs = collect(args.roots)
    payload = {"runs": runs, "tallies": {f"{1000 * s:.0f}mm": tally(runs, s) for s in SENSITIVITY_M}}
    args.out.write_text(json.dumps(payload, indent=1, sort_keys=True))
    if args.summary:
        args.summary.write_text(summary_md(runs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
