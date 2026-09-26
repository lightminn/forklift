"""Tabulate evaluation.json files from tools/evaluate_slam_replay.py.

    python tools/summarise_slam_runs.py <replay dir> [<replay dir> ...]

Prints one Markdown row per replay and a mean / max row per noise setting,
computed here from the files -- never copied by hand.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

COLUMNS = (
    ("slam", "first_pose_ate_rmse_m"),
    ("slam", "ate_max_m"),
    ("slam", "final_error_m"),
    ("odometry_same_samples", "first_pose_ate_rmse_m"),
    ("odometry_same_samples", "final_error_m"),
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replays", type=Path, nargs="+")
    args = parser.parse_args(argv)
    rows = []
    for replay in args.replays:
        report = json.loads((replay / "evaluation.json").read_text())
        meta = json.loads((Path(report["record"]) / "meta.json").read_text())
        noisy = any(report["noise"][k] for k in report["noise"] if k != "seed")
        values = [report[group][key] for group, key in COLUMNS]
        rows.append((meta["seed"], "합성 잡음" if noisy else "없음", report, values))
    print(
        "| seed | 잡음 | 경로 m | 짝지은 스캔 | SLAM ATE (시작 정렬) | SLAM 최대 | "
        "SLAM 최종 | 오도메트리 ATE | 오도메트리 최종 |"
    )
    print("|---|---|---:|---:|---:|---:|---:|---:|---:|")
    for seed, noise, report, values in rows:
        cells = " | ".join(f"{v:.3f}" for v in values)
        print(
            f"| {seed} | {noise} | {report['slam']['path_length_m']:.1f} | "
            f"{report['slam_poses_paired']}/{report['scans']} | {cells} |"
        )
    for noise in sorted({row[1] for row in rows}):
        table = np.array([row[3] for row in rows if row[1] == noise])
        for name, reduce in (("평균", np.mean), ("최대", np.max)):
            cells = " | ".join(f"{v:.3f}" for v in reduce(table, axis=0))
            print(f"| {name} ({len(table)}) | {noise} | | | {cells} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
