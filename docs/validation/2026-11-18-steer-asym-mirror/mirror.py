"""Mirror check of the steering error per wheel (docs/validation/2026-11-18-steer-asym-mirror.md), from the F2/I
re-registration's mechanism output (assets/f2i-reregistration/mechanism.json). Reads only.

In a stratum the two numbers of ``mabs`` are the time-weighted mean |command - actual| steering angle of the left and
the right front wheel (run_transport's steers order: left_steer, right_steer); a positive commanded curvature is a left
turn. The re-registration's A = |mabs_left - mabs_right| in one stratum is therefore inner wheel against outer wheel,
which a left-right symmetric truck also shows. The mirror check compares the same role across the two turn
directions: inner(left turn) = left wheel against inner(right turn) = right wheel, and the outer wheels likewise.

    python mirror.py [--mechanism PATH] [--arms O,F2]
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

HERE = Path(__file__).resolve().parent
MECH = HERE.parent / "assets" / "f2i-reregistration" / "mechanism.json"


def pairs(runs: dict, arm: str) -> list:
    """(run, stratum, within_left_mrad, within_right_mrad, inner_diff_mrad, outer_diff_mrad, exposure_l, exposure_r,
    ratio_l, ratio_r) for every direction x speed where both turn strata are valid in ``arm``."""
    out = []
    for run, arms in sorted(runs.items()):
        a = arms.get(arm)
        if not a:
            continue
        for d in ("fwd", "rev"):
            for v in ("slow", "fast"):
                lt, rt = a.get(f"{d}_left_{v}"), a.get(f"{d}_right_{v}")
                if not (lt and rt and lt.get("valid") and rt.get("valid")):
                    continue
                inner_l, outer_l = lt["mabs"][0], lt["mabs"][1]  # left turn: the left wheel is inside
                inner_r, outer_r = rt["mabs"][1], rt["mabs"][0]  # right turn: the right wheel is inside
                out.append({"run": run, "stratum": f"{d}_{v}",
                            "within_left_mrad": 1e3 * abs(lt["mabs"][0] - lt["mabs"][1]),
                            "within_right_mrad": 1e3 * abs(rt["mabs"][0] - rt["mabs"][1]),
                            "inner_diff_mrad": 1e3 * (inner_l - inner_r), "outer_diff_mrad": 1e3 * (outer_l - outer_r),
                            "exposure_s": [lt["exposure_s"], rt["exposure_s"]], "ratio": [lt["ratio"], rt["ratio"]]})
    return out


def summary(rows: list) -> dict:
    if not rows:
        return {"n": 0}
    med = lambda k: round(statistics.median(r[k] for r in rows), 2)  # noqa: E731
    rng = lambda k: [round(min(r[k] for r in rows), 2), round(max(r[k] for r in rows), 2)]  # noqa: E731
    return {"n": len(rows), "within_left_median_mrad": med("within_left_mrad"),
            "within_right_median_mrad": med("within_right_mrad"),
            "inner_diff_median_mrad": med("inner_diff_mrad"), "inner_diff_range_mrad": rng("inner_diff_mrad"),
            "outer_diff_median_mrad": med("outer_diff_mrad"), "outer_diff_range_mrad": rng("outer_diff_mrad"),
            "exposure_sum_s": [round(sum(r["exposure_s"][0] for r in rows), 1),
                               round(sum(r["exposure_s"][1] for r in rows), 1)],
            "ratio_median": [round(statistics.median(r["ratio"][0] for r in rows), 3),
                             round(statistics.median(r["ratio"][1] for r in rows), 3)]}


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mechanism", type=Path, default=MECH)
    ap.add_argument("--arms", default="O,F2")
    a = ap.parse_args(argv)
    runs = json.loads(a.mechanism.read_text())["runs"]
    print(json.dumps({arm: summary(pairs(runs, arm)) for arm in a.arms.split(",")}, indent=1, sort_keys=True))


if __name__ == "__main__":
    main()
