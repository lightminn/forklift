"""The global matcher's calibration on captured scans (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md
B4a).

    run      the matcher on every valid start of a capture: one JSON line per start (the kept candidates with their
             lattice and refined poses and scores, the top's endpoint fraction, the truth, the wall time)
    select   the pre-registered threshold choice on dev lines only: among (E_MIN, R_MAX, R_STRUCT) with no wrong and
             no imprecise confirmation, the most successes; ties to the higher E_MIN, the lower R_MAX, the lower
             R_STRUCT
    judge    validation lines under the frozen choice: per-start outcome, counts per map, the one-sided 95 % bound
    overlap  the recorded shared-wall metric: per validation start, the share of its endpoints within 5 cm of any dev
             start's endpoints, both placed in the hall frame by their truth poses

    python tools/global_match_eval.py run --capture CAP.npz --bundle BUNDLE --out rows.jsonl
    python tools/global_match_eval.py select --rows dev1.jsonl --rows dev2.jsonl --out choice.json
    python tools/global_match_eval.py judge --rows val.jsonl --choice choice.json --out verdict.json
    python tools/global_match_eval.py overlap --val CAP_VAL.npz --dev CAP_DEV1.npz --dev CAP_DEV2.npz --out o.json

Outcomes (base pose, B4a): success = confirmed within 0.10 m and 0.05 rad; wrong = confirmed beyond 0.3 m or 0.1 rad;
imprecise = confirmed in between; anything unconfirmed is a request to the operator, by its reason.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization.slam_map import Grid  # noqa: E402
from forklift_core.localization.slam_pose import compose  # noqa: E402

LASER_X_M = -0.12  # the LiDAR on base_link
SUCCESS_M, SUCCESS_RAD = 0.10, 0.05
WRONG_M, WRONG_RAD = 0.3, 0.1
E_GRID = [round(0.50 + 0.05 * i, 2) for i in range(10)]
R_GRID = [round(0.50 + 0.05 * i, 2) for i in range(10)]  # B4a
S_GRID = [0.4, 0.5, 0.6, 0.7, 0.8]  # B4a
# B4b: the region where keep_ratio 0.80 leaves the decision exact (r_struct >= 0.80, r_max > r_struct)
R_GRID_B4B = [0.85, 0.9, 0.95]
S_GRID_B4B = [0.8]
MARGIN = 0.05  # B4b: E_MIN >= W + this, W the fraction of a wrong top that only E_MIN refuses (an empirical margin)
DEADLINE_S, OVERRUN_S, TIMEOUT_SHARE = 30.0, 2.0, 0.10  # B4b time condition
LOW_OVERLAP, LOW_OVERLAP_MIN = 0.2, 30  # B4b: the low-overlap subset and when it is called short


def laser_to_base(pose) -> tuple:
    x, y, yaw = pose
    return (x - LASER_X_M * math.cos(yaw), y - LASER_X_M * math.sin(yaw), yaw)


def _gap(a, b) -> tuple:
    d = math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(d)


def outcome(confirmed: bool, est_base, truth_base) -> str:
    if not confirmed:
        return "request"
    m, rad = _gap(est_base, truth_base)
    if m <= SUCCESS_M and rad <= SUCCESS_RAD:
        return "success"
    if m > WRONG_M or rad > WRONG_RAD:
        return "wrong"
    return "imprecise"


def capture_points(cap, k: int) -> np.ndarray:
    meta = json.loads(str(cap["meta"]))
    scans = [((0.0, 0.0, 0.0), cap["sent"][k, s], meta["angle_min_rad"], meta["angle_increment_rad"])
             for s in range(cap["sent"].shape[1])]
    return G.stack_scans(scans, range_min_m=meta["range_min_m"], range_max_m=meta["range_max_m"])


def cmd_run(a) -> int:
    cap = np.load(a.capture, allow_pickle=False)
    meta = json.loads(str(cap["meta"]))
    manifest = json.loads((a.bundle / "manifest.json").read_text())
    if manifest["map_id"] != meta["map_id"]:
        raise SystemExit(f"the capture was taken for map {meta['map_id'][:12]}, not this bundle's")
    mfw = tuple(float(v) for v in manifest.get("map_from_world", (0.0, 0.0, 0.0)))
    grid = Grid.from_npz(a.bundle / "grid.npz")
    cfg = G.MatchConfig(**json.loads(a.config)) if a.config else G.MatchConfig()
    t0 = time.perf_counter()
    index = G.MapIndex(grid, cfg)
    built = {"build_s": time.perf_counter() - t0,
             "bytes": int(sum(x.nbytes for x in (index.field, index.cand, index.walls, *index.L, *index.C)))}
    Path(f"{a.out}.index.json").write_text(json.dumps(built) + "\n")
    choice = json.loads(Path(a.choice).read_text())["choice"] if a.choice else None
    with open(a.out, "w") as f:
        for k in range(len(cap["id"])):
            row = {"id": str(cap["id"][k]), "split": meta["split"], "map_index": meta["map_index"],
                   "map_id": meta["map_id"], "stratum": str(cap["stratum"][k]), "valid": bool(cap["valid"][k]),
                   "invalid_reason": str(cap["reason"][k]),
                   "truth_laser": list(compose(mfw, tuple(cap["laser_truth"][k]))),
                   "truth_base": list(compose(mfw, tuple(cap["base_truth"][k])))}
            if row["valid"]:
                pts = capture_points(cap, k)
                t0 = time.monotonic()
                if choice is not None:  # the whole request with the frozen choice (B4b: search + refine + decide)
                    match, decision = G.relocalize(grid, pts, cfg, e_min=choice["e_min"], r_max=choice["r_max"],
                                                   r_struct=choice["r_struct"], index=index, clock=time.monotonic)
                    row["decision"] = decision.reason
                else:  # dev: no thresholds yet; the decision is negligible next to the search
                    deadline = None if cfg.deadline_s is None else t0 + cfg.deadline_s
                    match = G.global_match(grid, pts, cfg, index=index, clock=time.monotonic, deadline=deadline)
                    match = G.refine(grid, match, pts, cfg, index=index, clock=time.monotonic, deadline=deadline)
                    if deadline is not None and time.monotonic() > deadline:  # as relocalize(): nothing comes late
                        match = G.GlobalMatch("timeout", work=match.work)
                row["wall_s"] = time.monotonic() - t0
                row["points"] = int(len(pts))
                row["status"] = match.status
                row["work"] = match.work
                row["candidates"] = [{"pose": list(c.pose), "score": c.score, "endpoint_fraction": c.endpoint_fraction,
                                      "refined": c.refined, "lattice_pose": list(c.search_pose),
                                      "lattice_score": c.search_score} for c in match.candidates]
            f.write(json.dumps(row) + "\n")
    return 0


def _rows(paths) -> list:
    out = []
    for p in paths:
        out += [json.loads(line) for line in Path(p).read_text().splitlines() if line.strip()]
    return out


def _match(row) -> G.GlobalMatch:
    cands = tuple(G.Candidate(tuple(c["pose"]), c["score"], c["endpoint_fraction"], c["refined"],
                              tuple(c["lattice_pose"]), c["lattice_score"]) for c in row.get("candidates", []))
    return G.GlobalMatch(row.get("status", "uninformative"), cands)


def judge_row(row, e_min: float, r_max: float, r_struct: float) -> dict:
    d = G.decide(_match(row), e_min=e_min, r_max=r_max, r_struct=r_struct)
    est = laser_to_base(row["candidates"][0]["pose"]) if d.confirmed else None
    return {"id": row["id"], "map_index": row["map_index"], "outcome": outcome(d.confirmed, est, row["truth_base"]),
            "reason": d.reason, "ratio": d.ratio}


def tally(rows, e_min, r_max, r_struct) -> dict:
    counts = {"success": 0, "wrong": 0, "imprecise": 0, "request": 0}
    for row in rows:
        counts[judge_row(row, e_min, r_max, r_struct)["outcome"]] += 1
    return counts


def _top_wrong(row) -> bool:
    cands = row.get("candidates") or []
    return bool(cands) and outcome(True, laser_to_base(cands[0]["pose"]), row["truth_base"]) == "wrong"


def margin_w(rows, r_max: float, r_struct: float) -> float:
    """B4b: the largest endpoint fraction among dev starts whose top is wrong and that every check but E_MIN lets
    through at (r_max, r_struct) -- the wrong tops only E_MIN stands in front of; 0 when there are none. Starts
    without a top (overflow, uninformative, timeout) are not in it."""
    w = 0.0
    for row in rows:
        if _top_wrong(row) and G.decide(_match(row), e_min=0.0, r_max=r_max, r_struct=r_struct).confirmed:
            w = max(w, float(row["candidates"][0]["endpoint_fraction"]))
    return w


def cmd_select(a) -> int:
    rows = [r for r in _rows(a.rows or []) if r["valid"]]
    if any(r["split"] != 0 for r in rows):
        raise SystemExit("select reads dev lines only (an opened validation set goes in by --opened-rows)")
    if a.opened_rows and a.rules != "b4b":
        raise SystemExit("--opened-rows is a B4b input")
    rows += [r for r in _rows(a.opened_rows or []) if r["valid"]]
    b4b = a.rules == "b4b"
    table = []
    for e in E_GRID:
        for r in R_GRID_B4B if b4b else R_GRID:
            for s in S_GRID_B4B if b4b else S_GRID:
                if not s < r:
                    continue
                t = {"e_min": e, "r_max": r, "r_struct": s, **tally(rows, e, r, s)}
                if b4b:
                    t["w"] = margin_w(rows, r, s)
                table.append(t)
    clean = [t for t in table if t["wrong"] == 0 and t["imprecise"] == 0
             and (not b4b or t["e_min"] >= t["w"] + MARGIN - 1e-9)]
    clean.sort(key=lambda t: (-t["success"], -t["e_min"], t["r_max"], t["r_struct"]))
    choice = clean[0] if clean else None
    out = {"dev_starts": len(rows), "choice": choice, "clean_combinations": len(clean), "table": table}
    if b4b:  # B4a's output keeps its keys
        out = {"rules": a.rules, **out}
        out["opened_rows"] = list(a.opened_rows or [])
        out["margin"] = {"delta": MARGIN, "w": choice["w"] if choice else None, "wrong_tops": [
            {"id": r["id"], "endpoint_fraction": r["candidates"][0]["endpoint_fraction"],
             "refused_by": judge_row(r, choice["e_min"], choice["r_max"], choice["r_struct"])["reason"]}
            for r in rows if _top_wrong(r)] if choice else None}
    Path(a.out).write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({"dev_starts": len(rows), "choice": choice}))
    return 0 if choice else 3


def bound_95(n: int) -> float | None:
    """One-sided 95 % upper bound on a rate with 0 events in n (Clopper-Pearson)."""
    return None if n <= 0 else 1 - 0.05 ** (1 / n)


def cmd_judge(a) -> int:
    choice = json.loads(Path(a.choice).read_text())["choice"]
    rows = _rows(a.rows)
    if any(r["split"] != 1 for r in rows):
        raise SystemExit("judge reads validation lines only")
    valid = [r for r in rows if r["valid"]]
    per = [judge_row(r, choice["e_min"], choice["r_max"], choice["r_struct"]) for r in valid]
    b4b = a.rules == "b4b"
    late = 0
    if b4b:  # anything that came out after the deadline is a timeout; a confirmation among them is dropped
        for r, p in zip(valid, per, strict=True):
            wall = r.get("wall_s")
            if not isinstance(wall, (int, float)) or not math.isfinite(wall) or wall < 0:
                raise SystemExit(f"{r['id']}: a valid row needs a finite, non-negative wall_s")
            if wall > DEADLINE_S and p["reason"] != "timeout":
                late += p["reason"] == "confirmed"
                p.update(outcome="request", reason="timeout")
    counts = {k: sum(p["outcome"] == k for p in per) for k in ("success", "wrong", "imprecise", "request")}
    by_map = {}
    for p in per:
        m = by_map.setdefault(p["map_index"], {"starts": 0, "wrong": 0})
        m["starts"] += 1
        m["wrong"] += p["outcome"] == "wrong"
    for m in by_map.values():
        m["bound_95"] = bound_95(m["starts"]) if m["wrong"] == 0 else None
    reasons = {}
    for p in per:
        if p["outcome"] == "request":
            reasons[p["reason"]] = reasons.get(p["reason"], 0) + 1
    walls = [r["wall_s"] for r in valid if "wall_s" in r]
    safety = len(valid) >= 60 and counts["wrong"] == 0 and (not b4b or counts["imprecise"] == 0)
    timeouts = sum(p["reason"] == "timeout" for p in per)
    over = sum(r.get("wall_s", 0.0) > DEADLINE_S + OVERRUN_S for r in valid)
    timely = not b4b or (over == 0 and timeouts <= TIMEOUT_SHARE * len(valid))
    extra = {}
    if b4b:
        extra = {"rules": "b4b", "safety_pass": safety, "time_pass": timely, "timeouts": timeouts,
                 "confirmed_after_deadline": late, "over_32_s": over}
        if a.overlap:
            share = {s["id"]: s["shared_share"] for s in json.loads(Path(a.overlap).read_text())["starts"]}
            low = [p for p in per if share.get(p["id"], math.inf) < LOW_OVERLAP]
            extra["low_overlap"] = {"threshold": LOW_OVERLAP, "starts": len(low),
                                    "wrong": sum(p["outcome"] == "wrong" for p in low),
                                    "short": len(low) < LOW_OVERLAP_MIN}
    out = {"choice": choice, "starts": len(valid), "invalid_captures": len(rows) - len(valid), **counts,
           "pass": safety and timely, **extra,
           "bound_95": bound_95(len(valid)) if counts["wrong"] == 0 else None, "by_map": by_map,
           "request_reasons": reasons,
           "wall_s": {"median": float(np.median(walls)), "p90": float(np.quantile(walls, 0.9, method="higher"))}
           if walls else None, "starts_detail": per}
    Path(a.out).write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: out[k] for k in ("starts", "success", "wrong", "imprecise", "request", "pass", "bound_95")}))
    return 0


def hall_endpoints(cap, k: int) -> np.ndarray:
    """A start's endpoints (first scan, pre-noise ranges) in the hall frame by its truth laser pose."""
    meta = json.loads(str(cap["meta"]))
    pts = G.scan_points(cap["raw"][k, 0], meta["angle_min_rad"], meta["angle_increment_rad"],
                        range_min_m=meta["range_min_m"], range_max_m=meta["range_max_m"], match_range_m=1e9)
    x, y, yaw = (float(v) for v in cap["laser_truth"][k])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], 1)


def matcher_points(cap, k: int, cell_m: float = 0.05) -> np.ndarray:
    """B4b: what the matcher reads of a start (the five stacked scans, <= match range, one point per cell) in the
    hall frame by its truth laser pose."""
    pts = G._thin(capture_points(cap, k), cell_m)
    x, y, yaw = (float(v) for v in cap["laser_truth"][k])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], 1)


def shared_share(points: np.ndarray, reference: np.ndarray, radius_m: float = 0.05) -> float:
    if not len(points):
        return math.nan
    if not len(reference):
        return 0.0
    from scipy.spatial import cKDTree  # noqa: PLC0415

    d, _ = cKDTree(reference).query(points, k=1)
    return float((d <= radius_m).mean())


def cmd_overlap(a) -> int:
    points = hall_endpoints if a.rules == "b4a" else matcher_points
    ref = []
    for p in a.dev:
        cap = np.load(p, allow_pickle=False)
        ref += [points(cap, k) for k in range(len(cap["id"])) if cap["valid"][k]]
    reference = np.vstack(ref) if ref else np.zeros((0, 2))
    cap = np.load(a.val, allow_pickle=False)
    rows = [{"id": str(cap["id"][k]), "shared_share": shared_share(points(cap, k), reference)}
            for k in range(len(cap["id"])) if cap["valid"][k]]
    shares = [r["shared_share"] for r in rows]
    out = {**({"rules": a.rules} if a.rules == "b4b" else {}), "radius_m": 0.05, "starts": rows,
           "summary": {"median": float(np.median(shares)), "max": float(np.max(shares))} if shares else None}
    Path(a.out).write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out["summary"]))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("run")
    p.add_argument("--capture", type=Path, required=True)
    p.add_argument("--bundle", type=Path, required=True)
    p.add_argument("--config", default=None, help="MatchConfig fields as JSON (dev only)")
    p.add_argument("--choice", type=Path, default=None,
                   help="B4b validation: the frozen choice; each start runs as one relocalize() request")
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("select")
    p.add_argument("--rules", choices=("b4a", "b4b"), default="b4b")
    p.add_argument("--rows", action="append")
    p.add_argument("--opened-rows", action="append", help="B4b: an opened validation set taken as dev data")
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("judge")
    p.add_argument("--rules", choices=("b4a", "b4b"), default="b4b")
    p.add_argument("--rows", action="append", required=True)
    p.add_argument("--choice", type=Path, required=True)
    p.add_argument("--overlap", type=Path, default=None, help="B4b: overlap output, for the low-overlap subset")
    p.add_argument("--out", type=Path, required=True)
    p = sub.add_parser("overlap")
    p.add_argument("--rules", choices=("b4a", "b4b"), default="b4b")
    p.add_argument("--val", type=Path, required=True)
    p.add_argument("--dev", action="append", required=True)
    p.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    return {"run": cmd_run, "select": cmd_select, "judge": cmd_judge, "overlap": cmd_overlap}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
