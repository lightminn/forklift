#!/usr/bin/env bash
# 11-17 (plan B6'): the H743 link check on the Jetson -- the firmware session's tool and criteria
# (docs/plans/2026-11-17-h743-board-check.md 6). Move the H743 (HIL image already flashed on the laptop) to the
# Jetson's USB first; 10 min, nothing touched, only DISARM is sent.
#   bash tools/hwday/jetson_h743.sh [seconds, default 600]
source "$(dirname -- "${BASH_SOURCE[0]}")/jetson_lib.sh"
secs="${1:-600}"
[[ -e "$HW_REPO/tools/h743_link_check.py" ]] || { echo "tools/h743_link_check.py is not in this checkout yet (the firmware PR): update it (tools/hwday/sync_main.sh) first"; exit 2; }
jsync
jssh 'ls -l /dev/forklift-mcu* 2>&1; id -nG | tr " " "\n" | grep -x dialout || echo "the account is NOT in dialout (the udev rule gives 0660)"'
rc=0
# one record per run, remote and local: a run refused by another executor touches no other run's files (design 8.1)
run="h743_link_$(date -u +%Y%m%dT%H%M%SZ)_$$"
remote="data/jetson_check/2026-11-17/$run.jsonl"
jssh "cd ~/forklift && mkdir -p data/jetson_check/2026-11-17 && .venv/bin/python tools/h743_link_check.py --device /dev/forklift-mcu --seconds $secs --out $remote" > "$HW_OUT/$run.txt" 2>&1 || rc=$?
if [ "$rc" -eq 2 ]; then
  tail -3 "$HW_OUT/$run.txt"  # refused (the board is held by another executor): no record was made
  exit 2
fi
if ! jssh "cat ~/forklift/$remote" > "$HW_OUT/$run.jsonl"; then
  echo "FAIL  copying the record back failed" >&2; rc=1
fi
tail -3 "$HW_OUT/$run.txt"
exit "$rc"
