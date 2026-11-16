#!/usr/bin/env bash
# 11-17 hardware day (docs/plans/2026-11-17-hw-day.md A2): the A2M12 measurement on THIS machine (the laptop, or the
# Jetson when tools/hwday/jetson_a2m12.sh runs it there).
#   bash tools/hwday/a2m12_run.sh <label> [minutes]      label: laptop | jetson; minutes default 10
# Finds the USB-serial device (one /dev/serial/by-id entry; DEV=... overrides), records its udev identity, starts the
# pinned driver + adapter in the ROS container, checks scans arrive, runs scan_check for the minutes with a raw record
# and judges it. Results: data/a2m12_check/2026-11-17_<label>/run_<time>/. Nothing drives; only the LiDAR spins.
source "$(dirname -- "${BASH_SOURCE[0]}")/a2m12_lib.sh"
label="${1:?usage: a2m12_run.sh laptop|jetson [minutes]}"; minutes="${2:-10}"
a_device
out=$(a_rundir "$label" run)
{ echo "device: $DEV -> $(readlink -f "$DEV")"
  udevadm info -q property -n "$DEV" | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_VENDOR|ID_MODEL|ID_USB_DRIVER)=' || true
  ls -l "$(readlink -f "$DEV")"; echo "groups: $(id -nG)"; } | tee "$out/device.txt"
a_start_driver "$out/driver.log"
a_first_scans "$out"
seconds=$(( minutes * 60 ))
echo "scan_check ${seconds} s -> $out ..."
"$A_PY" tools/scan_check.py --socket "$A_SOCK_DIR/scan.sock" --seconds "$seconds" --record "$out/scan.jsonl" \
  --out "$out/summary.json" > "$out/scan_check.out" 2>&1
"$A_PY" tools/hwday/judge.py a2m12 "$out/summary.json" --minutes "$minutes" | tee "$out/verdict.txt"
