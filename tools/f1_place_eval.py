"""F1's offline evaluation on stationary captures (docs/plans/2026-10-31-operator-placed-start.md 2-4).

    run    one session per valid start under the registered policy: at most N_PLACE placements, placement k on
           capture k's scans of that start, its click drawn from the click model (base position uniform over the
           area of a disc of CLICK_M, heading uniform within +-CLICK_RAD, a fixed seed per start and placement), the
           local re-match with R_AMB disabled (its ambiguity ratio recorded): one JSON line per start
    sweep  every R_AMB of R_AMB_GRID rescored from those lines (a placement is ambiguous when its ratio >= R_AMB,
           checked before the edge as the matcher does); per value the sessions' outcomes -- the first confirmed
           placement ends a session (the first preview is confirmed, the plan's fixed policy) -- and the registered
           choice: no wrong and no imprecise session, then the most rescued, ties to the stricter (lower) value

    python tools/f1_place_eval.py run --bundle B --capture C1 --capture C2 --capture C3 --seed S --out rows.jsonl
    python tools/f1_place_eval.py sweep --rows rows.jsonl [--hard ids.json] --out sweep.json

A start is valid when its first capture passed the input check; a later capture of it that did not is that
placement's ``unsteady`` (a valid session's failure). Pose classes as B-P2 4.0: success <= 0.10 m and <= 0.05 rad,
wrong > 0.30 m or > 0.10 rad, any other confirmation imprecise.
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
sys.path.insert(0, str(ROOT / "tools"))

import global_match_eval as GE  # noqa: E402

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization import local_match as L  # noqa: E402
from forklift_core.localization.slam_map import Grid  # noqa: E402
from forklift_core.localization.slam_pose import compose  # noqa: E402

CLICK_ROOT = 20261031
CLICK_M, CLICK_RAD = 1.0, 0.35  # the click model (plan 2)
N_PLACE = 3
E_MIN = 0.80  # B-P1b, frozen
T_LOC_S = 10.0
R_AMB_GRID = [round(0.80 + 0.02 * i, 2) for i in range(11)]  # 0.80 .. 1.00 (plan 2: the leaf floor to 1)
NO_AMB = 2.0  # an R_AMB no ratio reaches: the run records the ratio, the sweep decides


def click(seed: int, start_index: int, k: int, truth) -> tuple:
    rng = np.random.default_rng([CLICK_ROOT, int(seed), int(start_index), int(k)])
    r = CLICK_M * math.sqrt(rng.uniform())  # uniform over the disc's area
    a = rng.uniform(0.0, 2 * math.pi)
    dyaw = rng.uniform(-CLICK_RAD, CLICK_RAD)
    return (float(truth[0] + r * math.cos(a)), float(truth[1] + r * math.sin(a)), float(L.wrap(truth[2] + dyaw)))


def placement_decision(rec: dict, r_amb: float) -> tuple:
    """(confirmed, reason) of a recorded placement at ``r_amb``: ambiguous when its ratio reaches it (the ratio is
    recorded only where the matcher reached its ambiguity check, which comes before the edge)."""
    if rec.get("amb_ratio") is not None and rec["amb_ratio"] >= r_amb:
        return False, "ambiguous"
    return rec["reason"] == "confirmed", rec["reason"]


def session(placements: list, r_amb: float, truth) -> dict:
    reasons = []
    for k, rec in enumerate(placements):
        confirmed, reason = placement_decision(rec, r_amb)
        if confirmed:
            return {"outcome": GE.outcome(True, tuple(rec["base"]), tuple(truth)), "placements": k + 1,
                    "reasons": reasons + ["confirmed"]}
        reasons.append(reason)
        if reason == "unsteady":  # the live phase ends the session there (Codex F1 implementation review P2)
            return {"outcome": "failed", "placements": k + 1, "reasons": reasons}
    return {"outcome": "failed", "placements": len(placements), "reasons": reasons}


def choose(table: list) -> dict | None:
    clean = [t for t in table if t["wrong"] == 0 and t["imprecise"] == 0]
    if not clean:
        return None
    return sorted(clean, key=lambda t: (-t["rescued"], t["r_amb"]))[0]


def _record(res: L.LocalResult, wall_s: float, click_base) -> dict:
    return {"click": list(click_base), "reason": res.reason, "status": res.status,
            "amb_ratio": res.amb_ratio, "base": None if res.base is None else list(res.base),
            "endpoint_fraction": res.endpoint_fraction, "leaves": res.leaves, "work": res.work,
            "wall_s": round(wall_s, 4)}


def cmd_run(a) -> int:
    caps = [np.load(p, allow_pickle=False) for p in a.capture]
    metas = [json.loads(str(c["meta"])) for c in caps]
    manifest = json.loads((a.bundle / "manifest.json").read_text())
    if any(m["map_id"] != manifest["map_id"] for m in metas):
        raise SystemExit("a capture was taken for another map than this bundle's")
    ids = [list(map(str, c["id"])) for c in caps]
    if any(i != ids[0] for i in ids):
        raise SystemExit("the captures do not list the same starts in the same order")
    mfw = tuple(float(v) for v in manifest.get("map_from_world", (0.0, 0.0, 0.0)))
    grid = Grid.from_npz(a.bundle / "grid.npz")
    cfg = G.MatchConfig(**json.loads(a.config)) if a.config else G.MatchConfig()
    index = G.MapIndex(grid, cfg)
    h, w = grid.data.shape
    x0, y0 = grid.origin[0], grid.origin[1]
    x1, y1 = x0 + w * grid.resolution_m, y0 + h * grid.resolution_m
    with open(a.out, "w") as f:
        for i, sid in enumerate(ids[0]):
            truth = tuple(compose(mfw, tuple(caps[0]["base_truth"][i])))
            row = {"id": sid, "stratum": str(caps[0]["stratum"][i]), "valid": bool(caps[0]["valid"][i]),
                   "truth_base": list(truth), "placements": []}
            if row["valid"]:
                for k in range(min(N_PLACE, len(caps))):
                    c = click(a.seed, i, k, truth)
                    # off the map first: live refuses the command before any scan (Codex F1 re-review P2)
                    if not (x0 <= c[0] < x1 and y0 <= c[1] < y1):  # one failed placement
                        row["placements"].append({"click": list(c), "reason": "invalid", "status": "invalid",
                                                  "amb_ratio": None, "base": None})
                        continue
                    if not bool(caps[k]["valid"][i]):
                        row["placements"].append({"click": list(c), "reason": "unsteady", "status": "unsteady",
                                                  "amb_ratio": None, "base": None})
                        continue
                    t0 = time.monotonic()  # T_LOC from the scans (here: their conversion) to the result
                    pts = GE.capture_points(caps[k], i)
                    res = L.local_match(grid, pts, L.Window(base=c), cfg, e_min=E_MIN, r_amb=NO_AMB, index=index,
                                        clock=time.monotonic, deadline=t0 + T_LOC_S)
                    row["placements"].append(_record(res, time.monotonic() - t0, c))
            f.write(json.dumps(row) + "\n")
    return 0


def _quantile(v: list, q: float):
    if not v:
        return None
    v = sorted(v)
    return v[min(len(v) - 1, math.ceil(q * (len(v) - 1)))]


def cmd_sweep(a) -> int:
    rows = [json.loads(line) for line in Path(a.rows).read_text().splitlines() if line.strip()]
    valid = [r for r in rows if r["valid"]]
    hard = set(json.loads(Path(a.hard).read_text())) if a.hard else None
    table = []
    for r_amb in R_AMB_GRID:
        out = {"r_amb": r_amb, "success": 0, "imprecise": 0, "wrong": 0, "failed": 0, "reasons": {}}
        hard_rescued = 0
        for row in valid:
            s = session(row["placements"], r_amb, row["truth_base"])
            out[s["outcome"]] += 1
            for reason in s["reasons"]:
                out["reasons"][reason] = out["reasons"].get(reason, 0) + 1
            if hard is not None and row["id"] in hard and s["outcome"] == "success":
                hard_rescued += 1
        out["rescued"] = out["success"]
        if hard is not None:
            out["hard"] = {"n": sum(r["id"] in hard for r in valid), "rescued": hard_rescued}
        table.append(out)
    walls = [p["wall_s"] for r in valid for p in r["placements"] if "wall_s" in p]
    summary = {"valid": len(valid), "invalid": len(rows) - len(valid), "table": table, "choice": choose(table),
               "time": {"placements": len(walls), "p95_s": _quantile(walls, 0.95), "max_s": max(walls, default=None)}}
    Path(a.out).write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: summary[k] for k in ("valid", "invalid", "choice", "time")}, indent=1))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--bundle", type=Path, required=True)
    r.add_argument("--capture", type=Path, action="append", required=True, help="one per placement, in order")
    r.add_argument("--seed", type=int, required=True)
    r.add_argument("--config", default=None, help="MatchConfig overrides (JSON), as the frozen match_config.json")
    r.add_argument("--out", type=Path, required=True)
    s = sub.add_parser("sweep")
    s.add_argument("--rows", type=Path, required=True)
    s.add_argument("--hard", type=Path, default=None, help="ids (JSON list) of the starts the unattended path did not")
    s.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    return cmd_run(a) if a.cmd == "run" else cmd_sweep(a)


if __name__ == "__main__":
    sys.exit(main())
