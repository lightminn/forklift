#!/bin/bash
#SBATCH --job-name=slam_h
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=1G
#SBATCH --time=00:45:00
# Online-SLAM plan arm H (docs/plans/2026-11-13-slam-online-bounded.md, criterion 1-H): one spliced replay in two
# phases on one job -- the session's unit 0 into a mapping bridge with the graph dumped (deploy/slurm/
# slam_cost_replay.sbatch, --end-unit 1), a replay bundle from that dump (tools/slam_replay_bundle.py, source = this
# run, unit 0), then the rest of the stream (--start-unit 1) into a localization bridge on that bundle, started at the
# session's own start pose in the mapping's map frame (plan 1-H: parked at the start; tools/slam_h_switch.py pose,
# which also records the mapping's latest estimate there). The driver refuses any other bundle
# (--session-bundle). Step clocks (time.monotonic, the driver's clock) go to steps.json; tools/slam_h_switch.py
# summary gives the switch wait (last mapping reply -> first localization reply, <= 60 s) and the keyframe times after
# it (< 1 s). Both phases' cost summaries run after the switch, so they are not in the wait.
#   sbatch deploy/slurm/slam_h_switch.sbatch OUT RUN_DIR "key=value ..." [driver args: --noise-seed S
#       --splice-to-sim-s T [--splice-runs ...] ...]
#   LOC_SETS (environment): as slam_cost_replay.sbatch; BASELINE: an R0 replay's drive.jsonl of the same input
#   (the localization phase's excess error).
# Run from the repository root (a snapshot); docker image forklift/ros2-dev:jazzy.
set -u
OUT=$(realpath -m "$1"); RUN=$(realpath "$2"); SETS=$3; shift 3
ROOT=$(pwd)
PY=${VENV_PY:-/home/projects/forklift/.venv/bin/python}
if [ -e "$OUT" ] && [ -n "$(ls -A "$OUT" 2>/dev/null)" ]; then echo "output $OUT is not empty"; exit 2; fi
mkdir -p "$OUT"
echo '{"state": "pending"}' > "$OUT/status.json"
mono() { "$PY" -c 'import time; print(time.monotonic())'; }
step() { echo "\"$1\": $(mono)," >> "$OUT/steps.part"; }
case " $* " in *" --start-unit "*|*" --end-unit "*|*" --session-"*|*" --pace-sim "*)
  echo "the phases' units, bundle and pacing are this script's"; exit 2 ;; esac
SUMMARY=off DUMP_GRAPH=1 SLAM_NODE=sync bash "$ROOT/deploy/slurm/slam_cost_replay.sbatch" "$OUT/map" "$RUN" "$SETS" \
  "$@" --end-unit 1 > "$OUT/map.log" 2>&1
field() { "$PY" -c 'import json,sys; print(json.load(open(sys.argv[1])).get(sys.argv[2]))' "$1" "$2" 2>/dev/null || echo null; }
# the replay script itself always ends 0: a phase passed when its driver and its container (the bridge's own save,
# Codex H tools review P2-3) both ended 0
MAP=$(field "$OUT/map/status.json" driver_exit); MAP_CON=$(field "$OUT/map/status.json" container_exit)
step map_container_end
"$PY" "$ROOT/tools/slam_replay_bundle.py" "$OUT/map/bridge" "$OUT/bundle" --source-run "$RUN" --units 0 \
  > "$OUT/bundle.log" 2>&1
BUN=$?
MAP_ID=$(tail -1 "$OUT/bundle.log")
"$PY" "$ROOT/tools/slam_h_switch.py" pose "$OUT/map/drive.jsonl" "$OUT/map/drive.log" > "$OUT/start_pose.json" \
  2>>"$OUT/bundle.log"
POSE=$(field "$OUT/start_pose.json" pose)
[ "$POSE" = null ] && POSE=""
step bundle_end
LOC=null
LOC_CON=null
if [ "$MAP" = 0 ] && [ "$MAP_CON" = 0 ] && [ "$BUN" = 0 ] && [ -n "$POSE" ]; then
  SUMMARY=off SLAM_NODE=localization MAP_BUNDLE="$OUT/bundle" MAP_ID="$MAP_ID" MAP_START_POSE="$POSE" \
    bash "$ROOT/deploy/slurm/slam_cost_replay.sbatch" "$OUT/loc" "$RUN" "$SETS" "$@" --start-unit 1 \
    --session-bundle "$OUT/bundle" --session-map-id "$MAP_ID" > "$OUT/loc.log" 2>&1
  LOC=$(field "$OUT/loc/status.json" driver_exit); LOC_CON=$(field "$OUT/loc/status.json" container_exit)
fi
{ echo "{"; cat "$OUT/steps.part"; echo "\"_\": null}"; } | "$PY" -c \
  'import json,sys; d=json.load(sys.stdin); d.pop("_"); print(json.dumps(d))' > "$OUT/steps.json"
rm -f "$OUT/steps.part"
SW=null MAP_SUM=null LOC_SUM=null ALL_SUM=null
if [ "$LOC" = 0 ] && [ "$LOC_CON" = 0 ]; then
  "$PY" "$ROOT/tools/slam_h_switch.py" summary "$OUT/map/drive.jsonl" "$OUT/map/drive.log" "$OUT/loc/drive.jsonl" \
    "$OUT/loc/drive.log" --steps "$OUT/steps.json" > "$OUT/h_switch.json"
  SW=$?
  "$PY" "$ROOT/tools/slam_cost_summary.py" "$OUT/map/drive.jsonl" --graph-dump "$OUT/map/bridge/dump" \
    > "$OUT/map/summary.json"
  MAP_SUM=$?
  BASE=()
  [ -n "${BASELINE:-}" ] && BASE=(--baseline "$BASELINE")
  "$PY" "$ROOT/tools/slam_cost_summary.py" "$OUT/loc/drive.jsonl" "${BASE[@]}" > "$OUT/loc/summary.json"
  LOC_SUM=$?
  # the whole replay (criteria on the full sim range read both phases; Codex H tools review)
  cat "$OUT/map/drive.jsonl" "$OUT/loc/drive.jsonl" > "$OUT/drive.jsonl"
  "$PY" "$ROOT/tools/slam_cost_summary.py" "$OUT/drive.jsonl" "${BASE[@]}" > "$OUT/summary.json"
  ALL_SUM=$?
fi
nul() { echo "$1" | sed 's/None/null/'; }
printf '{"state": "done", "map_driver_exit": %s, "map_container_exit": %s, "bundle_exit": %s, "loc_driver_exit": %s, "loc_container_exit": %s, "switch_summary_exit": %s, "map_summary_exit": %s, "loc_summary_exit": %s, "summary_exit": %s, "map_id": "%s"}\n' \
  "$(nul "$MAP")" "$(nul "$MAP_CON")" "$BUN" "$(nul "$LOC")" "$(nul "$LOC_CON")" "$SW" "$MAP_SUM" "$LOC_SUM" "$ALL_SUM" \
  "$MAP_ID" > "$OUT/status.json"
cat "$OUT/status.json"
