#!/bin/bash
#SBATCH --job-name=slam_loc_replay
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=4G
#SBATCH --time=00:45:00
# Replay a live localization session's keyframes into a fresh localization bridge (tools/slam_loc_replay_drive.py) on
# its survey bundle, then score it (tools/slam_loc_replay_eval.py) -- the SLAM localization plan's V5 diagnosis.
#   sbatch deploy/slurm/slam_loc_replay.sbatch OUT EPOCH_DIR BUNDLE MAP_ID START_POSE ODOM UNTIL_S ["key=value ..."]
#   EPOCH_DIR: the live epoch (its run/ and bridge/); ODOM: recorded | truth; the params are the session's own
#   bridge/localization_params.yaml with the key=value changes. NOISE_SEED (environment, 0): the shared-scan noise.
# Run from the repository root (a snapshot); docker image forklift/ros2-dev:jazzy. Writes OUT/status.json, OUT/eval.json.
set -u
OUT=$(realpath -m "$1"); EP=$(realpath "$2"); BUNDLE=$(realpath "$3"); MAP_ID=$4; START=$5; ODOM=$6; UNTIL=$7
SETS=${8:-}
ROOT=$(pwd)
PY=${VENV_PY:-/home/projects/forklift/.venv/bin/python}
if [ -e "$OUT" ] && [ -n "$(ls -A "$OUT" 2>/dev/null)" ]; then echo "output $OUT is not empty"; exit 2; fi
mkdir -p "$OUT/bridge" "$OUT/sock"
echo '{"state": "pending"}' > "$OUT/status.json"
JOB=${SLURM_JOB_ID:-local$$}
CPUS=$(taskset -cp $$ | awk -F': ' '{print $2}')
CPU0=$(echo "$CPUS" | cut -d, -f1 | cut -d- -f1)
DRV_CPU=$(echo "$CPUS" | tr ',' '\n' | sed 's/-/\n/' | tail -1)
YAML=$OUT/localization_params.yaml
cp "$EP/bridge/localization_params.yaml" "$YAML"
for kv in $SETS; do
  k=${kv%%=*}; v=${kv#*=}
  grep -qE "^    $k:" "$YAML" || { echo "unknown param $k"; exit 2; }
  sed -i "s/^    $k: .*/    $k: $v/" "$YAML"
done
NAME=slam_loc_replay_${JOB}
trap 'docker rm -f "$NAME" >/dev/null 2>&1' EXIT
echo "job=$JOB epoch=$EP odom=$ODOM until=$UNTIL sets=[$SETS] start $(date -Is)"
docker run -d --name "$NAME" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -e SLAM_MAP_BUNDLE=/bundle -e SLAM_EXPECTED_MAP_ID="$MAP_ID" \
  -e SLAM_MAP_START_POSE="$START" -v "$ROOT:/workspace:ro" -v "$BUNDLE:/bundle:ro" -v "$YAML:/params.yaml:ro" \
  -v "$OUT/bridge:/out" -v "$OUT/sock:/run/slam" forklift/ros2-dev:jazzy \
  bash /workspace/tools/slam_loc_replay_container.sh /run/slam/slam.sock /out /params.yaml >/dev/null
taskset -c "$DRV_CPU" "$PY" "$ROOT/tools/slam_loc_replay_drive.py" "$EP/run" "$EP/bridge/records.jsonl" \
  "$OUT/sock/slam.sock" "$OUT/replies.jsonl" --noise-seed "${NOISE_SEED:-0}" --odom "$ODOM" --until-s "$UNTIL" \
  > "$OUT/drive.log" 2>&1
DRV=$?
CON=$(timeout 300 docker wait "$NAME")
docker logs "$NAME" > "$OUT/container.log" 2>&1
"$PY" "$ROOT/tools/slam_loc_replay_eval.py" "$EP/run" "$OUT/replies.jsonl" "$BUNDLE" > "$OUT/eval.json"
EV=$?
SENT=$(grep -oE "^sent [0-9]+" "$OUT/drive.log" | cut -d' ' -f2)
printf '{"state": "done", "driver_exit": %s, "container_exit": %s, "eval_exit": %s, "sent": %s}\n' \
  "$DRV" "${CON:-null}" "$EV" "${SENT:-null}" > "$OUT/status.json"
cat "$OUT/status.json"
