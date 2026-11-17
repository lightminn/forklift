#!/usr/bin/env bash
# H743 board check step 2 (docs/plans/2026-11-17-h743-board-check.md): after RESET the board runs the image and
# shows up as the link device; its USB product string must be the staged image's identity.
set -euo pipefail
H=$(cd "$(dirname "$0")" && pwd)
WANT=$(head -1 "$H/fw/BUILD.txt")
DEV=${FORKLIFT_MCU_DEV:-/dev/forklift-mcu}  # tests point it at a stand-in

for _ in $(seq 1 15); do
  [ -e "$DEV" ] && break
  sleep 1
done
if [ ! -e "$DEV" ]; then
  echo "FAIL: $DEV did not appear in 15 s."
  echo "ttyACM devices now: $(find /dev -maxdepth 1 -name 'ttyACM*' -printf '%f ' 2>/dev/null)"
  echo "lsusb (0483): $(lsusb | grep 0483 || echo none)"
  exit 1
fi
# read the whole listing first, then pick the first match: `udevadm ... | grep -m1` closed the pipe while udevadm
# was still writing the parent devices' attributes, and with pipefail the script died of SIGPIPE (exit 141, 2026-11-17)
ATTRS=$(udevadm info -a -n "$DEV")
PRODUCT=$(sed -n '/ATTRS{product}==/{s/.*=="\(.*\)"/\1/p;q;}' <<<"$ATTRS")
SERIAL=$(sed -n '/ATTRS{serial}==/{s/.*=="\(.*\)"/\1/p;q;}' <<<"$ATTRS")
echo "device: $DEV -> $(readlink -f "$DEV")"
echo "product: $PRODUCT"
echo "serial:  $SERIAL"
if [ ! -r "$DEV" ] || [ ! -w "$DEV" ]; then
  echo "FAIL: no read/write permission on $DEV (uucp group? log out and in after adding it)."
  exit 1
fi
if [ "$PRODUCT" != "$WANT" ]; then
  echo "FAIL: product string is not the staged image ($WANT)."
  exit 1
fi
echo "OK: the board runs the staged image. Next: 3_link_check.sh (10 minutes)"
