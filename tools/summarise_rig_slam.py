"""Tabulate rig SLAM evaluations: one row per replay, paired ratios per record.

    python tools/summarise_rig_slam.py --replays <experiment>/replays --output <dir>

Plan: docs/plans/2026-10-07-visual-slam-and-fusion.md D4. Reads every
``<record>/<config>_<noise>_<condition>/evaluation.json`` under ``--replays``
(written by tools/evaluate_rig_slam.py) and writes summary.json and
summary.md: per (record, condition, noise) the first-pose ATE of each config,
and for the plan's main comparisons the paired ratio F / L-ST and F / V4 on
the same record. A missing or failed replay stays in the table as such.
Records ``*_seed0`` are the development record (plan D3): listed, never in a
mean or a ratio.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

CONFIGS = ("lidar_st", "lidar", "vision_front", "vision", "vision_noodom", "fusion")
LABELS = {
    "lidar_st": "L-ST",
    "lidar": "L-RT",
    "vision_front": "V1",
    "vision": "V4",
    "vision_noodom": "V4-noodom",
    "fusion": "F",
}
CONDITIONS = (
    "nominal",
    "lidar_blackout",
    "lidar_short",
    "camera_blackout",
    "wheel_slip",
)
# F/L-ST: fusion against the existing system. F/L-RT: the cameras' share, same
# back end. L-RT/L-ST: the back end's share, same sensors. F/V4: the LiDAR's share.
PAIRS = (
    ("fusion", "lidar_st"),
    ("fusion", "lidar"),
    ("lidar", "lidar_st"),
    ("fusion", "vision"),
    ("vision", "lidar_st"),
)


def collect(root: Path) -> list[dict]:
    rows = []
    for record_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        if not record_dir.name.startswith(("nominal_seed", "fast_seed")):
            continue
        for replay in sorted(p for p in record_dir.iterdir() if p.is_dir()):
            # Longest config name first: "vision_front_..." is not "vision".
            names = [n for n in CONFIGS if replay.name.startswith(n + "_")]
            if not names:
                continue
            config = max(names, key=len)
            noise, condition = replay.name[len(config) + 1 :].split("_", 1)
            if condition not in CONDITIONS:
                continue  # development runs carry a tag after the condition
            # Synthetic noise is drawn with the record's own seed: one "noisy" group.
            noise = "none" if noise == "none" else "noisy"
            row = {
                "record": record_dir.name,
                "scenario": record_dir.name.split("_seed")[0],
                "config": config,
                "noise": noise,
                "condition": condition,
                "replay": str(replay),
            }
            evaluation = replay / "evaluation.json"
            if evaluation.exists():
                report = json.loads(evaluation.read_text())
                e = report["estimate"]
                row.update(
                    {
                        "status": "ok",
                        "first_pose_ate_m": e["first_pose_ate_rmse_m"],
                        "ls_ate_m": e["ate_rmse_m"],
                        "max_m": report["start_aligned_error_max_m"],
                        "final_m": e["final_error_m"],
                        "yaw_rmse_rad": e["yaw_rmse_rad"],
                        "odometry_first_pose_ate_m": report["odometry_same_frames"][
                            "first_pose_ate_rmse_m"
                        ],
                        "paired": f"{report['frames_paired']}/{report['frames']}",
                        "front_lost_frames": report.get("front_lost_frames"),
                        "frame_wall_s_median": report.get("frame_wall_s_median"),
                    }
                )
            else:
                row["status"] = "missing"
            rows.append(row)
    return rows


def paired_ratios(rows: list[dict]) -> list[dict]:
    table = defaultdict(dict)
    for row in rows:
        if row["status"] == "ok":
            table[(row["condition"], row["noise"], row["record"])][row["config"]] = row
    out = []
    for (condition, noise, record), configs in sorted(table.items()):
        for a, b in PAIRS:
            if a in configs and b in configs:
                out.append(
                    {
                        "scenario": record.split("_seed")[0],
                        "condition": condition,
                        "noise": noise,
                        "record": record,
                        "pair": f"{LABELS[a]}/{LABELS[b]}",
                        "ratio": configs[a]["first_pose_ate_m"]
                        / configs[b]["first_pose_ate_m"],
                        "a_m": configs[a]["first_pose_ate_m"],
                        "b_m": configs[b]["first_pose_ate_m"],
                    }
                )
    return out


def markdown(rows: list[dict], ratios: list[dict]) -> str:
    lines = []
    groups = defaultdict(lambda: defaultdict(dict))
    for row in rows:
        groups[(row["scenario"], row["condition"], row["noise"])][row["record"]][
            row["config"]
        ] = row
    for (scenario, condition, noise), records in sorted(groups.items()):
        present = [c for c in CONFIGS if any(c in r for r in records.values())]
        lines.append(
            f"\n### {scenario} / {condition} / noise {noise} -- first-pose ATE (m)\n"
        )
        lines.append(
            "| record | " + " | ".join(LABELS[c] for c in present) + " | odometry |"
        )
        lines.append("|---" * (len(present) + 2) + "|")
        means = defaultdict(list)
        for record, configs in sorted(records.items()):
            cells, odometry = [], None
            for c in present:
                row = configs.get(c)
                if row is None:
                    cells.append("")
                elif row["status"] != "ok":
                    cells.append("missing")
                else:
                    cells.append(f"{row['first_pose_ate_m']:.3f}")
                    if not record.endswith("_seed0"):  # development record
                        means[c].append(row["first_pose_ate_m"])
                    odometry = row["odometry_first_pose_ate_m"]
            odo = f"{odometry:.3f}" if odometry is not None else ""
            name = f"{record} (dev)" if record.endswith("_seed0") else record
            lines.append(f"| {name} | " + " | ".join(cells) + f" | {odo} |")
        lines.append(
            "| mean (eval) | "
            + " | ".join(
                f"{np.mean(means[c]):.3f}" if means[c] else "" for c in present
            )
            + " | |"
        )
    if ratios:
        lines.append("\n### Paired ratios (first-pose ATE, same record)\n")
        lines.append(
            "| condition | noise | pair | records | mean ratio | ratio range | a better |"
        )
        lines.append("|---|---|---|---|---|---|---|")
        grouped = defaultdict(list)
        for r in ratios:
            if r["record"].endswith("_seed0"):
                continue  # development record
            grouped[(r["scenario"], r["condition"], r["noise"], r["pair"])].append(r)
        for (scenario, condition, noise, pair), items in sorted(grouped.items()):
            values = np.array([i["ratio"] for i in items])
            lines.append(
                f"| {scenario} / {condition} | {noise} | {pair} | {len(items)} | {values.mean():.2f} | "
                f"{values.min():.2f}-{values.max():.2f} | {int((values < 1).sum())}/{len(items)} |"
            )
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replays", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    rows = collect(args.replays)
    ratios = paired_ratios(rows)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "summary.json").write_text(
        json.dumps({"rows": rows, "paired_ratios": ratios}, indent=1) + "\n"
    )
    text = markdown(rows, ratios)
    (args.output / "summary.md").write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
