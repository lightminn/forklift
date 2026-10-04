"""Tabulate S2 (online SLAM transport missions) against the plan's fixed criteria.

    python tools/summarise_slam_s2.py <artifacts>/20261004_slam_s2 [--json out.json]

Expects <root>/truth/seed_N/run/result.json (ground-truth control) and
<root>/slam/seed_N/run/result.json plus slam/seed_N/bridge/bridge_status.json.
Criteria (docs/plans/2026-10-04-online-slam-closed-loop.md, S2): safety --
no forbidden fork/pallet contact, no runtime overlap (any such failure reason);
success -- SLAM completions >= 3 of the seeds, and at most one seed the control
completed lost under SLAM; insertion, per completed SLAM run, from the true
geometry -- both forks inserted >= target - 20 mm (target = min(depth x 0.6,
carriage limit - reserve)), lateral clearance >= 10 mm at every fork check of
insert/extract/withdraw, insertion-axis yaw <= 3 deg. A SLAM run counts as
completed only when the runner succeeded and the bridge reported ok.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

SAFETY_WORDS = ("Forbidden fork/pallet contact", "footprint overlap", "Pallet dropped")


def load(path: Path) -> dict | None:
    return json.loads(path.read_text()) if path.exists() else None


def insertion(result: dict) -> dict:
    g = result["geometry"]
    target = min(g["pallet_depth_m"] * 0.6, g["carriage_limit_m"] - g["insertion_reserve_m"])
    measured = result.get("insertion_measured") or {}
    depths = [measured.get(side, {}).get("insertion_m") for side in ("left", "right")]
    lateral = [
        gap
        for phase in result.get("lateral_clearance_min_m", {}).values()
        for gap in phase.values()
        if gap is not None
    ]
    yaw = result.get("insertion_truth_yaw_rad")
    out = {
        "target_m": target,
        "depth_min_m": min(d for d in depths if d is not None) if all(d is not None for d in depths) else None,
        "lateral_min_m": min(lateral) if lateral else None,
        "yaw_deg": None if yaw is None else math.degrees(abs(yaw)),
    }
    out["depth_ok"] = out["depth_min_m"] is not None and out["depth_min_m"] >= target - 0.020
    out["lateral_ok"] = out["lateral_min_m"] is not None and out["lateral_min_m"] >= 0.010
    out["yaw_ok"] = out["yaw_deg"] is not None and out["yaw_deg"] <= 3.0
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(6)))
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    rows = []
    for seed in args.seeds:
        truth = load(args.root / f"truth/seed_{seed}/run/result.json")
        slam = load(args.root / f"slam/seed_{seed}/run/result.json")
        bridge = load(args.root / f"slam/seed_{seed}/bridge/bridge_status.json")
        row = {
            "seed": seed,
            "truth_success": bool(truth and truth.get("success")),
            "truth_reason": truth and truth.get("failure_reason"),
            "slam_runner_success": bool(slam and slam.get("success")),
            "slam_reason": slam and slam.get("failure_reason"),
            "bridge_ok": bool(bridge and bridge.get("ok")),
        }
        row["slam_success"] = row["slam_runner_success"] and row["bridge_ok"]
        reason = row["slam_reason"] or ""
        row["safety_violation"] = any(word in reason for word in SAFETY_WORDS) or bool(
            slam and slam.get("forbidden_pocket_contacts")
        )
        if slam:
            summary = slam.get("slam_summary") or {}
            row.update(
                rmse_m=summary.get("raw_position_rmse_m"),
                max_m=summary.get("raw_position_max_m"),
                holds=[
                    (h.get("event"), h.get("phase"), round(h.get("jump_m", 0.0), 4), h.get("replanned"))
                    for h in summary.get("holds", [])
                ],
                capture_error=[a.get("slam_error") for a in slam.get("observation_attempts", [])],
                insert_end_error=slam.get("slam_error_insert_end"),
            )
            if row["slam_success"]:
                row["insertion"] = insertion(slam)
        rows.append(row)
    completed = [r for r in rows if r["slam_success"]]
    lost = [r["seed"] for r in rows if r["truth_success"] and not r["slam_success"]]
    verdict = {
        "safety": not any(r["safety_violation"] for r in rows),
        "slam_completed": len(completed),
        "control_completed": sum(r["truth_success"] for r in rows),
        "lost_vs_control": lost,
        "success": len(completed) >= 3 and len(lost) <= 1,
        "insertion": all(
            r["insertion"]["depth_ok"] and r["insertion"]["lateral_ok"] and r["insertion"]["yaw_ok"]
            for r in completed
        ),
    }
    for r in rows:
        ins = r.get("insertion") or {}
        print(
            f"seed {r['seed']}: control {'OK' if r['truth_success'] else 'FAIL'}"
            f" | SLAM {'OK' if r['slam_success'] else 'FAIL'}"
            f" ({r['slam_reason'] or ''}{'' if r['bridge_ok'] or not r['slam_runner_success'] else ' bridge not ok'})"
            + (
                f" | depth {ins['depth_min_m']*1000:.1f}/{ins['target_m']*1000:.0f} mm"
                f" lateral {ins['lateral_min_m']*1000:.1f} mm yaw {ins['yaw_deg']:.2f} deg"
                if ins.get("depth_min_m") is not None and ins.get("lateral_min_m") is not None
                else ""
            )
            + (f" | raw RMSE {r['rmse_m']:.3f} max {r['max_m']:.3f}" if r.get("rmse_m") is not None else "")
        )
    print("VERDICT", json.dumps(verdict))
    if args.json:
        args.json.write_text(json.dumps({"rows": rows, "verdict": verdict}, indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
