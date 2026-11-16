#!/usr/bin/env bash
# 11-17 (plan B5): the bridge stall measurement on the Jetson (after jetson_setup.sh and jetson_power.sh).
#   bash tools/hwday/jetson_stall.sh [seconds per phase, default 300]      (about 11 min)
# Runs tools/hwday/stall_run.sh on the Jetson (MCU model on a pty, bridge unarmed, idle then every core busy) and
# copies the records and verdicts back to data/jetson_check/2026-11-17/stall/.
source "$(dirname -- "${BASH_SOURCE[0]}")/jetson_lib.sh"
secs="${1:-300}"
jsync
jssh "cd ~/forklift && bash tools/hwday/stall_run.sh ~/hwday_stall $secs" 2>&1 | tee "$HW_OUT/stall_run.txt"
mkdir -p "$HW_OUT/stall"
jssh 'cd ~ && tar -cf - hwday_stall' | tar -xf - -C "$HW_OUT/stall"
grep -h -E '^(PASS|FAIL|info|ALL|NOT)' "$HW_OUT"/stall/hwday_stall/*/verdict.txt
