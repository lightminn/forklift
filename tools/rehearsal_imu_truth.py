#!/usr/bin/env python3
"""The arrival-day rehearsal's IMU and truth stand-ins (docs/design/2026-11-15-arrival-day-rehearsal.md 2.5): format
checks only, never a measurement.

From a teleop bridge record made on the rehearsal bench (a synthetic record), writes
- ``imu.jsonl``: an ``imu_record/1`` file as tools/imu_record.py writes it (header, then ``imu`` rows for the gyro
  ``g`` at 400 Hz and the accelerometer ``a`` at 200 Hz, ``dev_us`` a 32-bit microsecond clock with its own offset),
  the gyro's body yaw rate taken from the STATE wheel rates (no slip -- the model's plant) on the D435i's down axis
  (y), plus white noise at the bench-measured angle random walk and a constant bias; gravity on y for the
  accelerometer. Header ``"synthetic": true``.
- ``truth.jsonl``: per trial, the standstill end poses (before the first moving segment and after the last) as the
  G9 branch A floor marks would give them, taken from the record's own odometry. Header ``"synthetic": true``.

The input must be a synthetic record (its header says so): a real record is refused.

usage: rehearsal_imu_truth.py --record TELEOP.jsonl --out DIR [--seed 0]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from forklift_core.live.synthetic import is_synthetic  # noqa: E402
from forklift_core.plant import m_analysis as ma  # noqa: E402
from forklift_core.plant.real_vehicle import load_real_vehicle  # noqa: E402
from forklift_core.plant.record_rows import sample_from_row  # noqa: E402

ARW_RAD_PER_SQRT_S = 2.054e-4  # the bench gyro y (registration 1.1)
GYRO_HZ, ACCEL_HZ = 400.0, 200.0
BIAS_RAD_S = 3.0e-4
G = 9.80665
VEHICLE = REPO / "config" / "real" / "vehicle.yaml"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--record", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)
    rows = [json.loads(x) for x in args.record.read_text(encoding="utf-8").splitlines() if x.strip()]
    header = next((r for r in rows if r.get("t") == "header"), {})
    if not is_synthetic(header):
        print("not a synthetic (rehearsal) record: the stand-ins are for the rehearsal only", file=sys.stderr)
        return 2
    cfg = load_real_vehicle(VEHICLE)
    states = [r for r in rows if r.get("t") == "state"]
    host = np.asarray([r["host_ns"] for r in states], dtype=np.int64)
    yaw_rate = ma.encoder_yaw_rate(np.asarray([r["wheel_rate_left_rad_s"] for r in states]),
                                   np.asarray([r["wheel_rate_right_rad_s"] for r in states]),
                                   radius_m=np.asarray(cfg.wheel_radius_m), track_m=cfg.track_m)
    rng = np.random.default_rng(args.seed)
    args.out.mkdir(parents=True, exist_ok=True)
    imu_header = {"t": "header", "schema": "imu_record/1", "tool": "rehearsal_imu_truth.py", "host_clock": "monotonic",
                  "host_boot_id": header.get("host_boot_id"), "synthetic": True, "source_record": str(args.record)}
    with (args.out / "imu.jsonl").open("w", encoding="utf-8") as f:
        f.write(json.dumps(imu_header) + "\n")
        dev_offset_us = 123_456_789
        for stream, hz in (("g", GYRO_HZ), ("a", ACCEL_HZ)):
            t = np.arange(host[0], host[-1], int(1e9 / hz), dtype=np.int64)
            if stream == "g":
                rate = np.interp(t, host, yaw_rate)
                sigma = ARW_RAD_PER_SQRT_S * np.sqrt(hz)  # white noise per sample
                x, y, z = (rng.normal(0, sigma, t.size), -rate + BIAS_RAD_S + rng.normal(0, sigma, t.size),
                           rng.normal(0, sigma, t.size))  # y down: a left (positive) yaw is negative on y
            else:
                x, y, z = rng.normal(0, 0.02, t.size), G + rng.normal(0, 0.02, t.size), rng.normal(0, 0.02, t.size)
            for i in range(t.size):
                dev = (dev_offset_us + int((t[i] - host[0]) / 1000)) % 2**32
                f.write(json.dumps({"t": "imu", "s": stream, "dev_us": dev, "ts_ms": dev / 1000.0, "dom": "global",
                                    "host_ns": int(t[i]), "x": float(x[i]), "y": float(y[i]), "z": float(z[i])}) + "\n")
    odo = ma.odometry_heading([sample_from_row(r).state for r in states], cfg, max_gap_s=0.2)
    t_mcu = odo["mcu_time_us"]
    out = []
    for tr in ma.trials(rows):
        moving = [label for label, _ in sorted(tr.spans.items(), key=lambda kv: kv[1][0])]
        spans = tr.spans
        first = min(spans.values(), key=lambda s: s[0])
        last = max(spans.values(), key=lambda s: s[1])

        def yaw_at(host_ns):
            i = int(np.clip(np.searchsorted(host, host_ns), 0, len(states) - 1))
            j = int(np.clip(np.searchsorted(t_mcu, states[i]["mcu_time_us"]), 0, len(t_mcu) - 1))
            return float(odo["yaw"][j])

        out.append({"t": "truth", "trial": tr.trial, "result": tr.result, "segments": moving,
                    "start_host_ns": first[0], "end_host_ns": last[1], "dyaw_rad": yaw_at(last[1]) - yaw_at(first[0]),
                    "method": "synthetic: the record's own odometry at the trial's ends"})
    with (args.out / "truth.jsonl").open("w", encoding="utf-8") as f:
        f.write(json.dumps({"t": "header", "schema": "rehearsal_truth/1", "synthetic": True,
                            "source_record": str(args.record)}) + "\n")
        for r in out:
            f.write(json.dumps(r) + "\n")
    print(json.dumps({"imu": str(args.out / "imu.jsonl"), "truth": str(args.out / "truth.jsonl"), "trials": len(out),
                      "synthetic": True}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
