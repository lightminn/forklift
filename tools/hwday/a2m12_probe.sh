#!/usr/bin/env bash
# 11-17 (plan A3): the angle convention. Put a box 30 cm to the RIGHT of the A2M12 first, then:
#   bash tools/hwday/a2m12_probe.sh right       (also: left | front | back)
# 20 s of scans; scan_check --probe says on which side the nearest return is (ROS: zero ahead, + to the left).
set -euo pipefail
side="${1:-right}"
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
cd "$repo"
out="data/a2m12_check/2026-11-17_laptop"; mkdir -p "$out"
py=/home/light/anaconda3/bin/python; [[ -x "$py" ]] || py=python3
DEV="${DEV:-$(find /dev/serial/by-id -maxdepth 1 -type l | grep -viE 'forklift-mcu|stm|weact' | head -1)}"
name="hwday-probe-$$"; sock_dir="/tmp/forklift-scan-$$"; mkdir -p "$sock_dir"
cleanup() { docker stop -t 3 "$name" >/dev/null 2>&1 || true; rm -rf "$sock_dir"; }
trap cleanup EXIT INT TERM
FORKLIFT_DOCKER_NAME="$name" tools/ros2_dev.sh scan-a2m12 "$DEV" "$sock_dir" > "$out/probe_driver.log" 2>&1 &
PYTHONPATH="$repo/src" "$py" tools/scan_check.py --socket "$sock_dir/scan.sock" --seconds 20 --probe "$side" \
  --out "$out/probe_${side}.json" > "$out/probe_${side}.out" 2>&1 || true
PYTHONPATH="$repo/src" "$py" tools/hwday/judge.py probe "$out/probe_${side}.json" | tee "$out/probe_${side}_verdict.txt"
