"""Tabulate closed-loop survey runs driven by a SLAM estimate (plan 2026-10-07 D6).

    python tools/summarise_rig_online.py --online <experiment>/online --output <dir>

Each ``<config>_seed<N>`` (or ``dev_<config>_seed0``) directory holds the run of
deploy/slurm/vslam_online.sbatch: ``run/`` from sim/isaac/run_slam_drive.py
(result.json, slam_control.npy) and ``bridge/`` from the SLAM bridge. The error
is the one the 10/04 S1 record used: the rear-axle pose control actually used
against ground truth, with no alignment. Writes online.json, online.md and,
per run, ``error_series.npz`` (at 10 Hz: stamps, truth, estimate, error) for
the video.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

# dev_: the development seed; video_: re-runs with the chase camera for the
# video. Both stay out of every evaluation table (``development`` is true).
NAME = re.compile(r"^(dev_|video_)?(lidar_st|vision|fusion|lidar)_seed(\d+)(?:_(.+))?$")


def one(run_dir: Path) -> dict | None:
    match = NAME.match(run_dir.name)
    if not match:
        return None
    prefix, config, seed, condition = match.groups()
    row = {
        "name": run_dir.name,
        "config": config,
        "seed": int(seed),
        "development": bool(prefix),
        "purpose": (prefix or "evaluation_").rstrip("_"),
        "condition": condition or "nominal",
    }
    result_path = run_dir / "run" / "result.json"
    if not result_path.exists():
        row.update({"status": "missing", "completed": False})
        return row
    result = json.loads(result_path.read_text())
    summary = result.get("slam_summary") or {}
    bridge = run_dir / "bridge" / "bridge_status.json"
    bridge_ok = json.loads(bridge.read_text()).get("ok") if bridge.exists() else None
    row.update(
        {
            "status": "ok",
            "completed": bool(result.get("success")),
            "failure_reason": result.get("failure_reason"),
            "raw_rmse_m": summary.get("position_rmse_m"),
            "raw_max_m": summary.get("position_max_m"),
            "yaw_rmse_rad": summary.get("yaw_rmse_rad"),
            "arrival_truth_m": (result.get("truth_arrival_error") or {}).get(
                "position_m"
            ),
            "frames": summary.get("scans_sent"),
            "replies": summary.get("replies"),
            "simulated_time_s": result.get("simulated_time_s"),
            "wall_time_s": result.get("wall_time_s"),
            "bridge_ok": bridge_ok,
            "freshness_failures": len(
                (result.get("rig_freshness") or {}).get("failures", [])
            ),
        }
    )
    control_path = run_dir / "run" / "slam_control.npy"
    if control_path.exists():
        control = np.load(control_path).reshape(-1, 8)
        driving = control[np.isfinite(control[:, 1])]
        if len(driving):
            keep = np.arange(0, len(driving), 12)  # 120 Hz -> 10 Hz
            sample = driving[keep]
            error = np.hypot(sample[:, 1] - sample[:, 4], sample[:, 2] - sample[:, 5])
            np.savez(
                run_dir / "error_series.npz",
                stamps_s=sample[:, 0],
                truth=sample[:, 4:7],
                estimate=sample[:, 1:4],
                error_m=error,
                frame="rear axle, world (the control's own pose)",
            )
    return row


def markdown(rows: list[dict]) -> str:
    lines = [
        "| run | config | seed | completed | raw RMSE (m) | raw max (m) | arrival (m) | frames | sim (s) | note |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:

        def f(key, r=r):
            v = r.get(key)
            return f"{v:.3f}" if isinstance(v, float) else ""

        note = "" if r.get("completed") else (r.get("failure_reason") or r["status"])
        sim = r.get("simulated_time_s")
        dev = f" ({r['purpose']})" if r["development"] else ""
        lines.append(
            f"| {r['name']} | {r['config']} | {r['seed']}{dev} | "
            f"{'yes' if r.get('completed') else 'no'} | {f('raw_rmse_m')} | {f('raw_max_m')} | "
            f"{f('arrival_truth_m')} | {r.get('frames') or ''} | "
            f"{f'{sim:.1f}' if sim else ''} | {note} |"
        )
    eval_rows = [
        r for r in rows if not r["development"] and r["condition"] == "nominal"
    ]
    lines.append("")
    lines.append("| config | runs | completed | mean raw RMSE (m) | mean raw max (m) |")
    lines.append("|---|---|---|---|---|")
    for config in ("lidar_st", "lidar", "vision", "fusion"):
        group = [r for r in eval_rows if r["config"] == config]
        if not group:
            continue
        rmse = [r["raw_rmse_m"] for r in group if r.get("raw_rmse_m") is not None]
        peak = [r["raw_max_m"] for r in group if r.get("raw_max_m") is not None]
        lines.append(
            f"| {config} | {len(group)} | {sum(bool(r.get('completed')) for r in group)} | "
            f"{np.mean(rmse):.3f} | {np.mean(peak):.3f} |"
            if rmse
            else f"| {config} | {len(group)} | 0 | | |"
        )
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--online", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    rows = [
        r for r in (one(d) for d in sorted(args.online.iterdir()) if d.is_dir()) if r
    ]
    rows.sort(key=lambda r: (r["development"] is False, r["seed"], r["config"]))
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "online.json").write_text(json.dumps(rows, indent=1) + "\n")
    text = markdown(rows)
    (args.output / "online.md").write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
