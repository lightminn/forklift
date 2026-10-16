"""Pixel lag of the live runner's camera (M-c plan C5): from frames saved by the probe's marker scenario, where the
runner's red plate sweeps sideways once a sim second, read the sim phase each frame's pixels were drawn at and
compare it with the frame's capture stamp (the render metadata's sim time).

  python tools/live_marker_lag.py --report RUN/probe.json --frames-dir RUN/frames --factor 0.1

The phase is read from the plate's column, mapped linearly between the extreme columns seen over all frames (the
sweep covers its whole range every second). lag_sim = (stamp phase - pixel phase) mod 1; the wall lag is lag_sim / f.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


def plate_column(path: Path) -> float | None:
    rgb = np.asarray(Image.open(path).convert("RGB"), dtype=np.int16)
    red = (rgb[:, :, 0] > 150) & (rgb[:, :, 1] < 80) & (rgb[:, :, 2] < 80)
    if red.sum() < 4:
        return None
    return float(np.nonzero(red)[1].mean())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", type=Path, required=True)
    ap.add_argument("--frames-dir", type=Path, required=True)
    ap.add_argument("--factor", type=float, required=True)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    report = json.loads(args.report.read_text())
    rows = []
    for f in report["frames"]:
        path = args.frames_dir / f"frame_{f['frame_id']:06d}.jpg"
        if path.exists() and f.get("stamp_ns") is not None:
            col = plate_column(path)
            if col is not None:
                rows.append((f["frame_id"], f["stamp_ns"] / 1e9, col))
    cols = np.array([r[2] for r in rows])
    lo, hi = np.percentile(cols, 1), np.percentile(cols, 99)
    lags = []
    for frame_id, stamp, col in rows:
        pixel_phase = float(np.clip((col - lo) / (hi - lo), 0.0, 1.0))
        stamp_phase = stamp % 1.0
        lag = (stamp_phase - pixel_phase) % 1.0
        if lag > 0.5:
            lag -= 1.0  # pixels ahead of the stamp by a little reads as a small negative lag
        lags.append(lag)
    lags = np.array(lags)
    result = {"frames": len(rows), "lag_sim_s": {"median": float(np.median(lags)), "p99": float(np.percentile(lags, 99)),
                                                "max": float(lags.max()), "min": float(lags.min())},
              "lag_wall_s": {"median": float(np.median(lags)) / args.factor,
                             "p99": float(np.percentile(lags, 99)) / args.factor,
                             "max": float(lags.max()) / args.factor},
              "phase_resolution_sim_s": float(1.0 / max(1.0, hi - lo)), "factor": args.factor}
    print(json.dumps(result, indent=1))
    if args.out is not None:
        args.out.write_text(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
