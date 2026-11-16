#!/usr/bin/env bash
# 11-17 (plan B7, only if time is left): the D435i on the Jetson, 120 s standing (tools/imu_record.py).
#   bash tools/hwday/jetson_imu.sh        (move the D435i's USB 3 cable to the Jetson first)
# Needs pyrealsense2 in the Jetson's venv; without it it stops (installing it is the user's call).
source "$(dirname -- "${BASH_SOURCE[0]}")/jetson_lib.sh"
jsync
jssh 'cd ~/forklift && .venv/bin/python -c "import pyrealsense2" 2>/dev/null' \
  || { echo "pyrealsense2 missing in the Jetson venv: skip (the laptop check of 2026-11-16 stands)"; exit 3; }
jssh 'cd ~/forklift && mkdir -p data/imu_check && PYTHONPATH=src .venv/bin/python tools/imu_record.py --out data/imu_check/2026-11-17_jetson.jsonl --seconds 120' 2>&1 | tail -5 | tee "$HW_OUT/imu.txt"
jssh 'cat ~/forklift/data/imu_check/2026-11-17_jetson.jsonl' > "$HW_OUT/imu_2026-11-17_jetson.jsonl"
