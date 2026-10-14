"""The odometry applicability gate of the EPAL 6 fork spacing plan, (3) (b) 11th draft (no Isaac).

docs/plans/2026-10-13-epal6-fork-spacing.md: after the merged near-field bounds table is frozen,
every episode of the judged batch's near-tracking runs -- one per accepted near-capture tracker
build, as tools/export_near_field_bounds.py splits the supplement -- must have both pair sets (the
contiguous control-tick pairs and the anchor sample against every later tick, at the tracker's
lookup age) inside the table's e and psi at their ages, no gap in the control record and no age
beyond the table's range. Anything left holds S5 and the table's final promotion (the batch's own
group verdict is not changed by this check).

    python tools/odometry_coverage.py --bounds config/near_field_bounds_measured.json \
        --runs <batch>/seed_*/run --output <batch>/odometry_coverage.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from forklift_core.perception.near_field_bounds import NearFieldBounds  # noqa: E402
from tools import d8b_calibration as CAL  # noqa: E402
from tools import export_near_field_bounds as EX  # noqa: E402


def check_pairs(ages, e, psi, bounds: NearFieldBounds) -> dict:
    """Every pair against the table by the runtime's own lookup (NearFieldBounds.odometry, the
    tracker's 1 ns tolerance): the largest excess and its age, and how many pairs fall beyond the
    table's range whatever their error."""
    worst = {"e_excess_m": -math.inf, "e_age_s": None, "psi_excess_rad": -math.inf, "psi_age_s": None}
    beyond = 0
    for age, e_, psi_ in zip(ages, e, psi, strict=True):
        row = bounds.odometry(float(age))
        if row is None:
            beyond += 1
            continue
        if e_ - row[0] > worst["e_excess_m"]:
            worst.update(e_excess_m=float(e_ - row[0]), e_age_s=float(age))
        if psi_ - row[1] > worst["psi_excess_rad"]:
            worst.update(psi_excess_rad=float(psi_ - row[1]), psi_age_s=float(age))
    for k in ("e_excess_m", "psi_excess_rad"):
        if worst[k] == -math.inf:
            worst[k] = None
    return {**worst, "pairs": len(ages), "ages_beyond_table": beyond}


def run_coverage(run, bounds: NearFieldBounds) -> list[dict]:
    out = []
    for ep in EX.supplement_episodes(run):
        row = {k: v for k, v in ep.items() if k != "_anchor"}
        if ep["span_s"] is None:
            out.append({**row, "problems": ["an episode without approach ticks"]})
            continue
        problems = []
        try:
            e, psi = EX.odometry_running_max(run.control, *ep["span_s"])
            k = np.arange(1, len(e))
            row["ticks"] = check_pairs(k * EX.TICK_S, e[1:], psi[1:], bounds)
        except ValueError as exc:
            problems.append(f"tick pairs: {exc}")
        try:
            row["anchor"] = check_pairs(
                *EX.anchor_pairs(run.control, ep["_anchor"], ep["aligned_s"], ep["span_s"][1]),
                bounds,
            )
        except ValueError as exc:
            problems.append(f"anchor pairs: {exc}")
        for name in ("ticks", "anchor"):
            r = row.get(name)
            if r is None:
                continue
            if r["e_excess_m"] is not None and r["e_excess_m"] > 1e-12:
                problems.append(f"{name}: e over the table by {r['e_excess_m'] * 1e3:.3f} mm at {r['e_age_s']:.3f} s")
            if r["psi_excess_rad"] is not None and r["psi_excess_rad"] > 1e-12:
                problems.append(
                    f"{name}: psi over the table by {r['psi_excess_rad'] * 1e3:.3f} mrad at {r['psi_age_s']:.3f} s"
                )
            if r["ages_beyond_table"]:
                problems.append(f"{name}: {r['ages_beyond_table']} pairs beyond the table's ages")
        out.append({**row, "problems": problems})
    return out


def started(run) -> bool:
    """Whether the run shows a near-field episode in any record: a tracker start, an approach or
    insert tick, or a frame fed to the tracker."""
    near = run.result.get("near_tracking") or {}
    names = list(run.truth.phases)
    codes = [names.index(p) for p in EX.EPISODE_PHASES if p in names]
    return bool(
        near.get("starts")
        or near.get("start")
        or near.get("frames_fed")
        or np.any(np.isin(run.truth.phase, codes))
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bounds", type=Path, required=True)
    parser.add_argument("--runs", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    bounds = NearFieldBounds.load(args.bounds)  # finite, increasing ages, running maxima
    runs, problems = {}, 0
    for path in args.runs:
        key = EX.artifact_path(path)
        try:
            run = CAL.Run.load(path)
            if not (run.result.get("arguments") or {}).get("near_tracking"):
                continue  # only near-tracking runs have episodes
            if not started(run):
                # Ended before any near-capture tracker was built: no anchor-held span.
                runs[key] = {"episodes": [], "note": run.result.get("failure_reason")}
                continue
            if not (run.result.get("near_tracking") or {}).get("starts"):
                raise ValueError("a near-field episode is recorded without its tracker start")
            episodes = run_coverage(run, bounds)
        except (OSError, ValueError, KeyError) as exc:
            runs[key] = {"unverifiable": str(exc)}
            problems += 1
            continue
        runs[key] = {"episodes": episodes}
        problems += sum(bool(ep["problems"]) for ep in episodes)
    out = {
        "bounds_sha256": EX.sha256(args.bounds),
        "tool_sha256": EX.sha256(Path(__file__)),
        "pass": problems == 0,
        "episodes_or_runs_with_problems": problems,
        "runs": runs,
    }
    args.output.write_text(json.dumps(out, indent=1) + "\n")
    print(f"wrote {args.output}: {'pass' if out['pass'] else 'FAIL'} ({problems} with problems)")
    return 0 if out["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
