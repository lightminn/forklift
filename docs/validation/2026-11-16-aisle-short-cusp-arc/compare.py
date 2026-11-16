"""The short cusp arc comparison (docs/plans/2026-11-16-aisle-short-cusp-arc.md 2; fixed before the results):
arms A (off) and B (--short-cusp-arc 0.6,1.0), riib aisle_rack seeds 0-9, deterministic.

    python compare.py --a DIR_A --b DIR_B --out OUT.json

DIR_x holds <name>/run/{result.json, slam_control.npy} with names ending in _s<seed>. Per seed and summed: transport
cusp failures recovered by a replan and final failures (the #522 tool, cusp_heading.py, run by run), completion
(result.success), planning wall (the mission's first plan plus every cusp and scene replan: sum and max), transport
path length, and the short curved cusp legs of every planned path (all stages and replans) and of the transport.
The pre-registered verdict: adopt only if B's replan sum < A's, completion B >= A, final failures B <= A and the
planning wall max B <= 2 x A + 10 s.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
CH_FILE = HERE.parent / "2026-11-15-aisle-cusp-heading" / "cusp_heading.py"
SHORT_M = 0.6  # the registered L


def _cusp_heading():
    spec = importlib.util.spec_from_file_location("cusp_heading", CH_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def short_curved_cusps(path: dict, length_m: float = SHORT_M) -> int:
    """Legs of a recorded path (poses, directions, curvatures_inv_m) that end at a gear change (the cusp at i-1),
    shorter than length_m and curved at their end."""
    if not path or not path.get("success", True):
        return 0
    poses, dirs, curv = path.get("poses") or [], path.get("directions") or [], path.get("curvatures_inv_m") or []
    n, start, out = len(dirs), 0, 0
    for i in range(1, n):
        if dirs[i] != dirs[i - 1]:
            seg = poses[max(start - 1, 0):i]  # from the previous cusp (pose start-1) to this one (pose i-1)
            length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(seg, seg[1:]))
            if abs(curv[i - 1]) > 1e-9 and length < length_m - 1e-9:
                out += 1
            start = i
    return out


def run_measures(result: dict) -> dict:
    paths = result.get("paths") or {}
    replans = [*(result.get("cusp_replans") or []), *(result.get("scene_replans") or [])]
    walls = [float(result["planning_wall_s"])] if result.get("planning_wall_s") is not None else []
    walls += [float(r["planning_wall_s"]) for r in replans if r.get("planning_wall_s") is not None]
    all_paths = [p for p in paths.values() if isinstance(p, dict)] + [r["path"] for r in replans
                                                                      if isinstance(r.get("path"), dict)]
    transport = paths.get("transport") or {}
    return {"success": bool(result.get("success")), "planning_wall_sum_s": round(sum(walls), 3),
            "planning_wall_max_s": round(max(walls), 3) if walls else None,
            "transport_length_m": transport.get("length_m"),
            "short_curved_cusps_all": sum(short_curved_cusps(p) for p in all_paths),
            "short_curved_cusps_transport": short_curved_cusps(transport)
            + sum(short_curved_cusps(r["path"]) for r in result.get("cusp_replans") or [] if isinstance(r.get("path"), dict))}


def arm(root: Path, ch) -> dict:
    out = {}
    for run in ch.runs_from_raw([root], "aisle_rack"):
        m = re.search(r"_s(\d+)$", run["name"])
        if m is None:
            raise ValueError(f"{run['name']}: no seed in the name")
        seed = int(m.group(1))
        c = ch.analyse([run], "postfix", None)["counts"]
        res = json.loads((root / run["name"] / "run" / "result.json").read_text())
        out[seed] = {"name": run["name"], "fail_replan": c["failures_replan"], "fail_final": c["failures_final"],
                     "judgements": c["judgements"], **run_measures(res)}
    return out


def verdict(a: dict, b: dict) -> dict:
    seeds = sorted(set(a) & set(b))
    if set(a) != set(b):
        raise ValueError(f"the arms ran different seeds: {sorted(set(a) ^ set(b))}")
    s = lambda arm_, k: sum(arm_[x][k] for x in seeds)  # noqa: E731
    wmax = lambda arm_: max((arm_[x]["planning_wall_max_s"] or 0.0) for x in seeds)  # noqa: E731
    rules = {"replans_fewer": s(b, "fail_replan") < s(a, "fail_replan"),
             "completion_not_lower": s(b, "success") >= s(a, "success"),
             "final_not_higher": s(b, "fail_final") <= s(a, "fail_final"),
             "wall_max_within": wmax(b) <= 2 * wmax(a) + 10.0}
    return {"seeds": seeds, "sums": {k: {"A": s(a, k), "B": s(b, k)} for k in
                                     ("fail_replan", "fail_final", "success", "judgements", "planning_wall_sum_s",
                                      "short_curved_cusps_all", "short_curved_cusps_transport")},
            "wall_max_s": {"A": wmax(a), "B": wmax(b)}, "rules": rules, "adopt": all(rules.values())}


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", type=Path, required=True)
    ap.add_argument("--b", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    x = ap.parse_args(argv)
    ch = _cusp_heading()
    a, b = arm(x.a, ch), arm(x.b, ch)
    out = {"A": a, "B": b, "verdict": verdict(a, b)}
    x.out.write_text(json.dumps(out, indent=1, sort_keys=True, default=float) + "\n")
    print(json.dumps(out["verdict"], indent=1, sort_keys=True))


if __name__ == "__main__":
    main()
