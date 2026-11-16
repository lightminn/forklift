#!/usr/bin/env bash
# 11-17 (plan A4): the 7b scan permit on the real A2M12, standing. Put the A2M12 on the floor in an open area
# (about 1.5 m clear ahead, 1.2 m behind, 0.6 m to each side), start, then keep a phone timer:
#   0-20 s nobody near;  20-40 s one person stands 0.5 m in front of the LiDAR;  40-60 s clear again.
#   bash tools/hwday/a2m12_permit.sh [height_m]      height of the scan plane above the floor, default 0.3
set -euo pipefail
height="${1:-0.3}"
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
cd "$repo"
out="data/a2m12_check/2026-11-17_laptop"; mkdir -p "$out"
py=/home/light/anaconda3/bin/python; [[ -x "$py" ]] || py=python3
DEV="${DEV:-$(find /dev/serial/by-id -maxdepth 1 -type l | grep -viE 'forklift-mcu|stm|weact' | head -1)}"
name="hwday-permit-$$"; sock_dir="/tmp/forklift-scan-$$"; mkdir -p "$sock_dir"
cleanup() { docker stop -t 3 "$name" >/dev/null 2>&1 || true; rm -rf "$sock_dir"; }
trap cleanup EXIT INT TERM
FORKLIFT_DOCKER_NAME="$name" tools/ros2_dev.sh scan-a2m12 "$DEV" "$sock_dir" > "$out/permit_driver.log" 2>&1 &
rm -f "$out/permit_timeline.jsonl"
PYTHONPATH="$repo/src" "$py" tools/hwday/scan_permit_check.py --socket "$sock_dir/scan.sock" --seconds 60 \
  --mount "0.9,0,${height},0" --out "$out/permit_timeline.jsonl" | tee "$out/permit.out"
PYTHONPATH="$repo/src" "$py" tools/hwday/judge.py permit "$out/permit_timeline.jsonl" | tee "$out/permit_verdict.txt"
