#!/usr/bin/env bash
# 11-17 (plan B5): the teleop bridge's loop on THIS machine against the host-only MCU model (rehearsal bench, no
# hardware, nothing armed), idle and then with every core busy. Runs on the Jetson (jetson_stall.sh starts it there);
# can be tried on the laptop with short phases.
#   bash tools/hwday/stall_run.sh <out dir> [seconds per phase, default 300]
# Each phase: a fresh bench (mcu_model on a pty) + teleop_bridge.py --record for the given seconds; the load phase
# adds one busy process per core, all children of this script and killed on exit. The bridge's cycle starts are in
# the records; tools/hwday/judge.py stall reads them.
set -euo pipefail
out="${1:?usage: stall_run.sh <out dir> [seconds]}"; secs="${2:-300}"
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
cd "$repo"
py="$repo/.venv/bin/python"; [[ -x "$py" ]] || py="${FORKLIFT_PY:-python3}"
export PYTHONPATH="$repo/src"
mkdir -p "$out"
pids=()
cleanup() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM
phase() {
  local name="$1"; local d="$out/$name"; rm -rf "$d"; mkdir -p "$d"
  "$py" tools/rehearsal_bench.py --stage drive --out-dir "$d/bench" > "$d/bench.log" 2>&1 &
  local bench=$!; pids+=("$bench")
  for _ in $(seq 1 120); do [[ -f "$d/bench/bench.json" ]] && break; sleep 1; done
  [[ -f "$d/bench/bench.json" ]] || { echo "FAIL  $name: bench did not start (see $d/bench.log)" | tee "$d/verdict.txt"; return 1; }
  local dev; dev=$("$py" -c 'import json,sys; print(json.load(open(sys.argv[1]))["pts"])' "$d/bench/bench.json")
  local busy=()
  if [[ "$name" == load ]]; then
    for _ in $(seq 1 "$(nproc)"); do "$py" -c 'while True: pass' & busy+=("$!"); pids+=("$!"); done
  fi
  local brc=0
  "$py" tools/teleop_bridge.py --device "$dev" --bench "$d/bench/bench.json" --record "$d/teleop.jsonl" \
    --port 0 --duration-s "$secs" > "$d/bridge.log" 2>&1 || brc=$?
  for p in "${busy[@]}"; do kill "$p" 2>/dev/null || true; done
  kill "$bench" 2>/dev/null || true
  local jrc=0
  "$py" tools/hwday/judge.py stall "$d/teleop.jsonl" --phase "$name" --seconds "$secs" > "$d/verdict.txt" 2>&1 || jrc=$?
  (( brc == 0 )) || echo "FAIL  $name: the bridge exited $brc (see $d/bridge.log)" >> "$d/verdict.txt"
  cat "$d/verdict.txt"
  (( brc == 0 && jrc == 0 ))
}
{ echo "== $(hostname) $(date -Is) nproc $(nproc)"; command -v nvpmodel >/dev/null && nvpmodel -q 2>/dev/null | head -2 || true; } | tee "$out/host.txt"
rc=0
phase idle || rc=1   # the load phase runs even when idle fails: both are needed to read the result
phase load || rc=1
exit "$rc"
