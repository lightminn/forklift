#!/usr/bin/env bash
# 11-17 hardware day (docs/plans/2026-11-17-hw-day.md A): the A2M12 measurement on THIS machine (the laptop, or the
# Jetson when tools/hwday/jetson_a2m12.sh runs it there).
#   bash tools/hwday/a2m12_run.sh <label> [minutes]      label: laptop | jetson; minutes default 10
# 1) finds the USB-serial device (one /dev/serial/by-id entry; DEV=... overrides) and records its udev identity,
# 2) starts the pinned driver + scan adapter in the ROS container (tools/ros2_dev.sh scan-a2m12),
# 3) runs tools/scan_check.py for the given minutes with a raw record, 4) judges it (tools/hwday/judge.py a2m12).
# Results: data/a2m12_check/2026-11-17_<label>/. Nothing drives anything; only the LiDAR spins.
set -euo pipefail
label="${1:?usage: a2m12_run.sh laptop|jetson [minutes]}"
minutes="${2:-10}"
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
cd "$repo"
out="data/a2m12_check/2026-11-17_${label}"
mkdir -p "$out"
py="${FORKLIFT_PY:-python3}"
[[ -x "$repo/.venv/bin/python" ]] && py="$repo/.venv/bin/python"
[[ -x /home/light/anaconda3/bin/python && "$label" == laptop ]] && py=/home/light/anaconda3/bin/python

# 1) the device
if [[ -z "${DEV:-}" ]]; then
  mapfile -t cands < <(find /dev/serial/by-id -maxdepth 1 -type l 2>/dev/null | grep -viE 'forklift-mcu|stm|weact' || true)
  if (( ${#cands[@]} != 1 )); then
    echo "USB-serial candidates (${#cands[@]}):" >&2; printf '  %s\n' "${cands[@]}" >&2
    echo "give the A2M12's one: DEV=/dev/serial/by-id/<name> bash tools/hwday/a2m12_run.sh $label" >&2
    exit 2
  fi
  DEV="${cands[0]}"
fi
{ echo "device: $DEV -> $(readlink -f "$DEV")"; udevadm info -q property -n "$DEV" | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_VENDOR|ID_MODEL|ID_USB_DRIVER)=' || true
  ls -l "$(readlink -f "$DEV")"; id -nG; } > "$out/device.txt"
cat "$out/device.txt"

# 2) driver + adapter in the background (stopped by name on every exit), 3) the check in front
name="hwday-a2m12-$$"
sock_dir="/tmp/forklift-scan-$$"
mkdir -p "$sock_dir"
cleanup() { docker stop -t 3 "$name" >/dev/null 2>&1 || true; rm -rf "$sock_dir"; }
trap cleanup EXIT INT TERM
FORKLIFT_DOCKER_NAME="$name" tools/ros2_dev.sh scan-a2m12 "$DEV" "$sock_dir" > "$out/driver.log" 2>&1 &
seconds=$(( minutes * 60 ))
echo "scan_check ${seconds} s (record $out/scan.jsonl) ..."
PYTHONPATH="$repo/src" "$py" tools/scan_check.py --socket "$sock_dir/scan.sock" --seconds "$seconds" \
  --record "$out/scan.jsonl" --out "$out/summary.json" > "$out/scan_check.out" 2>&1 || true
PYTHONPATH="$repo/src" "$py" tools/hwday/judge.py a2m12 "$out/summary.json" --minutes "$minutes" | tee "$out/verdict.txt"
