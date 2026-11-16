#!/usr/bin/env bash
# 11-17 (plan A3): the angle convention. Put a box 30 cm to the RIGHT of the A2M12 first, then:
#   bash tools/hwday/a2m12_probe.sh right       (also: left | front | back)
# 20 s of scans; scan_check --probe says on which side the nearest return is (ROS: zero ahead, + to the left).
source "$(dirname -- "${BASH_SOURCE[0]}")/a2m12_lib.sh"
side="${1:-right}"
a_device
out=$(a_rundir laptop "probe_$side")
a_start_driver "$out/driver.log"
a_first_scans "$out"
"$A_PY" tools/scan_check.py --socket "$A_SOCK_DIR/scan.sock" --seconds 20 --probe "$side" --out "$out/summary.json" \
  > "$out/scan_check.out" 2>&1
"$A_PY" tools/hwday/judge.py probe "$out/summary.json" | tee "$out/verdict.txt"
