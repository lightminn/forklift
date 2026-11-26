#!/usr/bin/env python3
"""The real SLAM bridge's ``laser_xyz_yaw`` from the files the operating run reads (docs/plans/2026-11-26-real-operating-
assembly.md 2.3, unlock U2b): base_link <- laser from ``lidar.yaml``'s mount (rear-axle frame) and the chassis's rear
axle offset (the chassis model ``vehicle.yaml`` names). The operating executor's hello carries the same value and the
bridge refuses another (1 mm / 0.1 deg), so the launcher computes it from the same files instead of typing it.

    python tools/real_bridge_laser.py [--lidar config/real/lidar.yaml] [--vehicle config/real/vehicle.yaml]
        (prints "[x, y, z, yaw]" for ``-p laser_xyz_yaw:=...``; exit 1 with the reason on a bad file)
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def laser_param(lidar_path: Path, vehicle_path: Path) -> list:
    from forklift_core.live import real_run as rr
    from forklift_core.plant.real_vehicle import chassis_model_path
    from forklift_core.sensors.lidar_cfg import load_lidar_config

    _tip, rear_offset_m, _carriage = rr.chassis_reference(chassis_model_path(vehicle_path).parent / "forklift.urdf")
    return [float(v) for v in rr.bridge_laser_xyz_yaw(load_lidar_config(lidar_path), rear_offset_m)]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--lidar", type=Path, default=REPO / "config/real/lidar.yaml")
    ap.add_argument("--vehicle", type=Path, default=REPO / "config/real/vehicle.yaml")
    a = ap.parse_args(argv)
    try:
        print(json.dumps(laser_param(a.lidar, a.vehicle)))
    except (OSError, ValueError, KeyError, TypeError) as err:
        print(f"real_bridge_laser: {type(err).__name__}: {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
