#!/usr/bin/env python3
"""Compare the rig's uniform metric rounding with a stereo disparity-domain step.

Appendix B-0 of docs/decisions/0003 asks whether a real D435i's quantisation
resembles ``tools/scene_rig.py``'s uniform ``QUANTIZE_STEP_M = 0.001``.  Half of
that question is answerable from the operating principle alone, without hardware:
a stereo pair recovers depth as z = f*B/d, so quantising disparity to a fixed
step dd gives a depth step that grows with the square of range,

    dz(z) = z^2 * dd / (f * B)

while the rig's step is flat.  This script prints both over the ranges the ADR
argues about, holding the rig's own camera model fixed so that only the
quantisation law changes (rule 4: match every other axis before comparing).

This is SPEC-DERIVED, NOT MEASURED.  The two sensor constants are cited below and
still need confirming against the delivered unit; what needs hardware is whether
a real D435i's post-processing leaves that step intact.

Sources for the constants:
  baseline 50 mm, subpixel 1/32 px  -- Intel RealSense D400 series documentation
  https://dev.intelrealsense.com/docs/white-paper-subpixel-linearity-improvement-for-intel-realsense-depth-cameras
  https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance
"""
from __future__ import annotations

import argparse

from tools.scene_rig import FOCAL_PX, QUANTIZE_STEP_M

D435I_BASELINE_M = 0.050
D435I_SUBPIXEL_PX = 1.0 / 32.0


def depth_step_m(z_m: float, focal_px: float, baseline_m: float, subpixel_px: float) -> float:
    """Depth step produced by one disparity step at range z."""
    return z_m * z_m * subpixel_px / (focal_px * baseline_m)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--focal-px", type=float, default=FOCAL_PX,
                    help="pixels; default is the rig's own focal length")
    ap.add_argument("--baseline-m", type=float, default=D435I_BASELINE_M)
    ap.add_argument("--subpixel-px", type=float, default=D435I_SUBPIXEL_PX)
    ap.add_argument("--band-margin-m", type=float, default=0.00225,
                    help="the derived band margin the gate compares against")
    ap.add_argument("--distances", default="1.0:4.5:0.25")
    args = ap.parse_args()

    lo, hi, step = (float(v) for v in args.distances.split(":"))
    print(f"# rig uniform step {QUANTIZE_STEP_M * 1000:.3f} mm (scene_rig.py:52)")
    print(f"# stereo: focal {args.focal_px:.3f} px, baseline {args.baseline_m * 1000:.1f} mm, "
          f"subpixel 1/{1 / args.subpixel_px:.0f} px  -> dz = z^2 * {args.subpixel_px:.5f} "
          f"/ ({args.focal_px:.3f} * {args.baseline_m})")
    print(f"# band margin {args.band_margin_m * 1000:.3f} mm")
    print(f"{'z_m':>6} {'rig_mm':>8} {'stereo_mm':>10} {'ratio':>7} {'steps_in_margin':>16}")
    z = lo
    while z <= hi + 1e-9:
        stereo = depth_step_m(z, args.focal_px, args.baseline_m, args.subpixel_px)
        rig = QUANTIZE_STEP_M
        print(f"{z:6.2f} {rig * 1000:8.3f} {stereo * 1000:10.3f} {stereo / rig:7.2f}"
              f" {args.band_margin_m / stereo:16.2f}")
        z += step
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
