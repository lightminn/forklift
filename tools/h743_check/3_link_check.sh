#!/usr/bin/env bash
# H743 board check step 3 (docs/plans/2026-11-17-h743-board-check.md): PC <-> board mcu-link/3 for 10 minutes.
# usage: 3_link_check.sh [seconds] [device]     (defaults: 600, /dev/forklift-mcu)
# Record: data/h743_check/<date>_link.jsonl; the last line printed is the one-line verdict.
set -euo pipefail
H=$(cd "$(dirname "$0")" && pwd)
SECONDS_RUN=${1:-600}
DEV=${2:-/dev/forklift-mcu}
PY=${PY:-/home/light/anaconda3/bin/python}
OUT="$H/$(date +%F)_link.jsonl"
if [ -e "$OUT" ]; then
  OUT="$H/$(date +%F_%H%M%S)_link.jsonl"
fi
# the staged copy must take the executor lock and send the poison (real safety wiring design 8.1): a copy staged
# before that change would open the board beside another executor -- stage again from main
if ! grep -q "before_config=" "$H/code/tools/h743_link_check.py" || ! grep -q "POISON" "$H/code/tools/h743_link_check.py"; then
  echo "STOP: the staged code predates the executor lock. Run tools/h743_check/stage.sh again from main."
  exit 1
fi
if [ ! -e "$DEV" ]; then
  echo "STOP: $DEV is not there. Run 2_enumerate.sh first."
  exit 1
fi
echo "listening $SECONDS_RUN s on $DEV (leave the board and cable alone) ..."
PYTHONPATH="$H/code/src" "$PY" "$H/code/tools/h743_link_check.py" --device "$DEV" --out "$OUT" --seconds "$SECONDS_RUN"
