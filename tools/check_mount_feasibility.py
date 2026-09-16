#!/usr/bin/env python3
"""Reject camera mounts whose view of the pocket band passes through the robot itself.

`tools/scene_rig.render` puts three things in the scene: the pallet boxes, the floor
and a back wall.  It does not render the forklift.  So a mount can "see" the lower
band through its own blades and the rig will report a detection, which is what makes
the shortest blind zones in docs/decisions/0003 section 3.4 unreachable in practice:
every mount there with a blind zone under 0.5 m has the camera looking through the
blade slab.

This checks the two things the rig cannot: whether the sight line to the band crosses
the blades, and whether the mount is forward of the chassis at all.

    tools/check_mount_feasibility.py 0.53 0.15 1.200
    tools/check_mount_feasibility.py --table            # every mount the ADR prints

Geometry, all from sim/models/dls08_provisional: blades x 0.530 -> 0.950 at
z 0.028 -> 0.052 (forklift.xml:269, and parameters.yaml fork_center_height_m 0.04 -/+
fork_thickness_m 0.024 / 2); body front x 0.44, mast x 0.48, fork root x 0.53
(parameters.yaml:19-23).  Band height 15 mm is the lower band the gate counts.
"""
from __future__ import annotations

import argparse

# Derived quantities come from one place only (summarise_sweep.derived).  Computing
# them again here is exactly the error that machine: the first draft of this file
# double-counted the half depth and printed 1.36 m where the canonical figure is 1.03.
from tools.summarise_sweep import derived

BLADE_X = (0.530, 0.950)
BLADE_Z = (0.028, 0.052)
BODY_FRONT_X = 0.44
FORK_ROOT_X = 0.53
T11_06_HALF_DEPTH_M = 0.330
BAND_Z_M = 0.015

# The mounts the ADR's section 3.4 tables print: (label, camera x, camera z, first detection).
ADR_MOUNTS = [
    ("(0.75, 0.50) baseline", 0.75, 0.50, 1.950),
    ("(0.48, 0.50) brief mast x", 0.48, 0.50, 1.680),
    ("(0.75, 0.27) K-2", 0.75, 0.27, 1.560),
    ("(0.75, 0.27) K-2 + tilt", 0.75, 0.27, 1.390),
    ("(0.75, 0.15)", 0.75, 0.15, 1.460),
    ("(0.53, 0.15)", 0.53, 0.15, 1.320),
    ("(0.53, 0.15) + tilt 0.25", 0.53, 0.15, 1.200),
    ("(0.53, 0.15) + tilt 0.45", 0.53, 0.15, 1.200),
]


def blade_crossing(cam_x: float, cam_z: float, front_x: float,
                   band_z: float = BAND_Z_M) -> tuple[float, float] | None:
    """The x interval where the sight line lies inside the blade slab, or None."""
    dx, dz = front_x - cam_x, band_z - cam_z
    if dz == 0:
        return None
    xs = []
    for z in BLADE_Z:
        t = (z - cam_z) / dz
        if 0.0 < t < 1.0:
            x = cam_x + t * dx
            if BLADE_X[0] <= x <= BLADE_X[1]:
                xs.append(x)
    return (min(xs), max(xs)) if len(xs) == 2 else None


def report(label: str, cam_x: float, cam_z: float, detection_m: float) -> bool:
    front = detection_m - T11_06_HALF_DEPTH_M
    _, blind = derived(detection_m)
    crossing = blade_crossing(cam_x, cam_z, front)
    reasons = []
    if crossing:
        reasons.append(f"sight line inside blades over x {crossing[0]:.3f}-{crossing[1]:.3f}")
    if cam_x < BODY_FRONT_X:
        reasons.append(f"camera x {cam_x} is behind the body front {BODY_FRONT_X}")
    elif cam_x < FORK_ROOT_X:
        reasons.append(f"camera x {cam_x} is forward of the body but behind the fork root")
    ok = not reasons
    print(f"{label:<30} blind {blind:5.2f} m  {'OK' if ok else 'INFEASIBLE'}"
          + ("" if ok else "  -- " + "; ".join(reasons)))
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("camera_x", nargs="?", type=float)
    ap.add_argument("camera_z", nargs="?", type=float)
    ap.add_argument("detection_m", nargs="?", type=float)
    ap.add_argument("--table", action="store_true", help="check every mount the ADR prints")
    args = ap.parse_args()

    if args.table:
        feasible = [m for m in ADR_MOUNTS if report(*m)]
        print()
        if feasible:
            best = min(feasible, key=lambda m: m[3])
            print(f"best feasible: {best[0]} -> blind zone {derived(best[3])[1]:.2f} m")
        return 0
    if args.camera_x is None:
        ap.error("give camera_x camera_z detection_m, or --table")
    return 0 if report("mount", args.camera_x, args.camera_z, args.detection_m) else 1


if __name__ == "__main__":
    raise SystemExit(main())
