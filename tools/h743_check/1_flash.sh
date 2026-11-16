#!/usr/bin/env bash
# H743 board check step 1 (docs/plans/2026-11-17-h743-board-check.md): DFU device present -> flash the staged
# ELF -> verify. Run from the staged folder (data/h743_check/, made by stage.sh), with the board in DFU mode
# (hold BOOT0, press and release RESET, release BOOT0). Writes only the sectors the image needs: no mass erase,
# no option bytes written (they are read and saved).
set -euo pipefail
H=$(cd "$(dirname "$0")" && pwd)
OUT="$H/$(date +%F)_flash"
mkdir -p "$OUT"

CLI=$(command -v STM32_Programmer_CLI || true)
if [ -z "$CLI" ]; then
  echo "STOP: STM32_Programmer_CLI is not on PATH."
  exit 1
fi

if ! (cd "$H/fw" && sha256sum -c --quiet SHA256SUMS); then
  echo "STOP: the staged image does not match fw/SHA256SUMS."
  exit 1
fi
echo "image: $(cat "$H/fw/BUILD.txt" | head -1)"

n=$(lsusb | grep -c '0483:df11' || true)
if [ "$n" -ne 1 ]; then
  echo "STOP: $n DFU devices (0483:df11) found, need exactly 1."
  echo "      Hold BOOT0, press and release RESET, release BOOT0; unplug any other STM32 board."
  exit 1
fi

"$CLI" -l usb > "$OUT/list.txt" 2>&1 || true
PORT=$(grep -oiE 'USB[0-9]+' "$OUT/list.txt" | head -1 | tr 'A-Z' 'a-z')
if [ -z "$PORT" ]; then
  echo "STOP: the programmer lists no USB DFU port. See $OUT/list.txt"
  exit 1
fi
echo "DFU port: $PORT"

set +e
"$CLI" -c port="$PORT" -ob displ > "$OUT/option_bytes.txt" 2>&1
ob=$?
"$CLI" -c port="$PORT" -w "$H/fw/fw_hil.elf" -v > "$OUT/flash.txt" 2>&1
fl=$?
set -e
echo "option bytes read: exit $ob (saved in $OUT/option_bytes.txt)"
if [ "$fl" -ne 0 ] || ! grep -qi 'verified successfully' "$OUT/flash.txt"; then
  echo "FAIL: flash or verify failed (exit $fl). See $OUT/flash.txt"
  exit 1
fi
echo "OK: flashed and verified. Now press RESET (BOOT0 not held) and run 2_enumerate.sh"
