#!/usr/bin/env bash
# 11-17 (plan A4): the 7b scan permit on the real A2M12, standing. Put the A2M12 on the floor in an open area
# (about 1.5 m clear ahead, 1.2 m behind, 0.6 m to each side). Start a phone timer when you press Enter:
#   0-20 s nobody near;  20-40 s one person stands 0.5 m in front of the LiDAR;  40-70 s clear again (it ends at 70 s).
#   bash tools/hwday/a2m12_permit.sh [height_m]      height of the scan plane above the floor, default 0.3
t0=$(date +%s.%N)  # the phone timer's zero: everything below counts from here
source "$(dirname -- "${BASH_SOURCE[0]}")/a2m12_lib.sh"
height="${1:-0.3}"
a_device
out=$(a_rundir laptop permit)
a_start_driver "$out/driver.log"
"$A_PY" tools/hwday/scan_permit_check.py --socket "$A_SOCK_DIR/scan.sock" --t0 "$t0" --until 70 \
  --mount "0.9,0,${height},0" --out "$out/timeline.jsonl" | tee "$out/permit.out"
"$A_PY" tools/hwday/judge.py permit "$out/timeline.jsonl" | tee "$out/verdict.txt"
