"""Score a localization replay (tools/slam_loc_replay_drive.py) against the truth and the survey grid (SLAM
localization plan, the V5 diagnosis).

Per processed reply: the pose the replay's localization gives the base (its ``map_from_odom`` composed with the
odometry that arm sent) against ``truth_base``, and the fraction of the scan's clean recorded endpoints on the survey's
occupied cells (one cell dilated, forklift_core.localization.slam_map.endpoint_fraction) at that pose and at the truth.
Summaries by ``--bin-s`` of sim time (median and max position error, max yaw error, median endpoint fractions). The
correction jumps: each processed reply's correction against the previous one, both composed with the same (this
reply's) odometry -- the pose change the correction alone makes. Complete (exit 0) only when the replies are every
scan the live bridge sent up to ``--until-s`` (ids and stamps), every one the live bridge expected a /pose for is
processed (a start-up drop may be skipped), none failed and every value finite (Codex P2).

    python tools/slam_loc_replay_eval.py RUN_DIR REPLIES_JSONL BUNDLE BRIDGE_RECORDS [--until-s T] [--bin-s 100]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from forklift_core.live.spool import load_slam_log  # noqa: E402
from forklift_core.localization import slam_map  # noqa: E402

LASER_X_M = -0.12  # the laser in base_link (tools/slam_bridge_container.sh)
ANG0, DANG, RMIN, RMAX = -math.pi, 2 * math.pi / 1600, 0.2, 12.0


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.remainder(a[2] + b[2], 2 * math.pi))


def laser(base):
    return compose(base, (LASER_X_M, 0.0, 0.0))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path)
    ap.add_argument("replies", type=Path)
    ap.add_argument("bundle", type=Path)
    ap.add_argument("bridge_records", type=Path)
    ap.add_argument("--until-s", type=float, default=float("inf"))
    ap.add_argument("--bin-s", type=float, default=100.0)
    a = ap.parse_args(argv)
    grid = slam_map.Grid.from_npz(a.bundle / "grid.npz")
    log = load_slam_log(a.run)
    ranges = log["scan_ranges_m"]
    rows, jumps = [], []
    replies = [json.loads(line) for line in a.replies.read_text().splitlines() if line]
    live = [json.loads(line) for line in a.bridge_records.read_text().splitlines() if line]
    sent = [r for r in live if r["sent"] != "local" and r["stamp_s"] <= a.until_s]
    expected = [(r["scan_id"], r["stamp_s"]) for r in sent]
    got = [(r["scan_id"], r["stamp_s"]) for r in replies]
    must = {r["scan_id"] for r in sent if r["sent"] == "expect_pose"}
    values = [v for r in replies for v in (*r["map_from_odom"], *r["odom"], *r["truth_base"])]
    complete = (len(got) == len(expected) and all(g[0] == e[0] and abs(g[1] - e[1]) < 1e-6 for g, e in zip(got, expected))
                and not any(r["status"] == "failed" for r in replies) and all(math.isfinite(v) for v in values)
                and all(r["status"] == "processed" for r in replies if r["scan_id"] in must))
    previous = None
    for r in replies:
        if r["status"] != "processed":
            continue
        if previous is not None:
            before, after = compose(previous, r["odom"]), compose(r["map_from_odom"], r["odom"])
            jumps.append((math.hypot(after[0] - before[0], after[1] - before[1]),
                          abs(math.remainder(after[2] - before[2], 2 * math.pi))))
        previous = r["map_from_odom"]
        est = compose(r["map_from_odom"], r["odom"])
        truth = r["truth_base"]
        scan = ranges[r["scan_id"]]
        fit_est = slam_map.endpoint_fraction(grid, laser(est), scan, ANG0, DANG, RMIN, RMAX)
        fit_truth = slam_map.endpoint_fraction(grid, laser(truth), scan, ANG0, DANG, RMIN, RMAX)
        rows.append((r["stamp_s"], math.hypot(est[0] - truth[0], est[1] - truth[1]),
                     abs(math.remainder(est[2] - truth[2], 2 * math.pi)), fit_est, fit_truth))
    if not rows:
        print(json.dumps({"processed": 0, "replies": len(replies)}))
        return 1
    m = np.asarray(rows, float)
    j = np.asarray(jumps, float).reshape(-1, 2)
    bins = []
    for lo in np.arange(0.0, m[:, 0].max() + a.bin_s, a.bin_s):
        k = (m[:, 0] >= lo) & (m[:, 0] < lo + a.bin_s)
        if k.any():
            bins.append({"from_s": float(lo), "n": int(k.sum()), "position_median_m": float(np.median(m[k, 1])),
                         "position_max_m": float(m[k, 1].max()), "yaw_max_rad": float(m[k, 2].max()),
                         "fit_estimate_median": float(np.nanmedian(m[k, 3])),
                         "fit_truth_median": float(np.nanmedian(m[k, 4]))})
    out = {"complete": bool(complete), "expected": len(expected), "processed": len(rows), "replies": len(replies),
           "jump": {"position_max_m": float(j[:, 0].max()) if len(j) else None,
                    "yaw_max_rad": float(j[:, 1].max()) if len(j) else None,
                    "position_over_0_02_m": int((j[:, 0] > 0.02).sum()), "position_over_0_05_m": int((j[:, 0] > 0.05).sum())},
           "failed": [r for r in replies if r["status"] == "failed"][:1],
           "position_max_m": float(m[:, 1].max()), "position_rmse_m": float(np.sqrt(np.mean(m[:, 1] ** 2))),
           "yaw_max_rad": float(m[:, 2].max()), "bins": bins}
    print(json.dumps(out, indent=1))
    return 0 if complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
