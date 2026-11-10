#!/bin/bash
#SBATCH --job-name=slam_map_replay
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=4G
#SBATCH --time=00:40:00
# Replay a live mapping session's sent scans into a fresh mapping bridge (tools/slam_loc_replay_drive.py, its odometry
# recorded or truth), for the hall return-cusp stall's first link (docs/validation/2026-11-10-hall-slam-replay.md).
#   sbatch deploy/slurm/slam_map_replay.sbatch OUT EPOCH_DIR ODOM [UNTIL_S]
#   EPOCH_DIR: the live epoch (its run/ and bridge/); ODOM: recorded | truth. NOISE_SEED (environment, 0): the shared-scan
#   noise. Run from the repository root (a snapshot); docker image forklift/ros2-dev:jazzy. Writes OUT/status.json.
set -u
OUT=$(realpath -m "$1"); EP=$(realpath "$2"); ODOM=$3; UNTIL=${4:-inf}
ROOT=$(pwd)
PY=${VENV_PY:-/home/projects/forklift/.venv/bin/python}
if [ -e "$OUT" ] && [ -n "$(ls -A "$OUT" 2>/dev/null)" ]; then echo "output $OUT is not empty"; exit 2; fi
mkdir -p "$OUT/bridge" "$OUT/sock"
echo '{"state": "pending"}' > "$OUT/status.json"
JOB=${SLURM_JOB_ID:-local$$}
CPUS=$(taskset -cp $$ | awk -F': ' '{print $2}')
CPU0=$(echo "$CPUS" | cut -d, -f1 | cut -d- -f1)
DRV_CPU=$(echo "$CPUS" | tr ',' '\n' | sed 's/-/\n/' | tail -1)
NAME=slam_map_replay_${JOB}
trap 'docker rm -f "$NAME" >/dev/null 2>&1' EXIT
echo "job=$JOB epoch=$EP odom=$ODOM until=$UNTIL start $(date -Is)"
docker run -d --name "$NAME" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$ROOT:/workspace:ro" -v "$OUT/bridge:/out" -v "$OUT/sock:/run/slam" \
  forklift/ros2-dev:jazzy bash /workspace/tools/slam_map_replay_container.sh /run/slam/slam.sock /out >/dev/null
docker inspect --format '{{.Image}}' "$NAME" > "$OUT/image_id"
taskset -c "$DRV_CPU" "$PY" "$ROOT/tools/slam_loc_replay_drive.py" "$EP/run" "$EP/bridge/records.jsonl" \
  "$OUT/sock/slam.sock" "$OUT/replies.jsonl" --noise-seed "${NOISE_SEED:-0}" --odom "$ODOM" --until-s "$UNTIL" \
  > "$OUT/drive.log" 2>&1
DRV=$?
CON=$(timeout 300 docker wait "$NAME")
docker logs "$NAME" > "$OUT/container.log" 2>&1
SENT=$(grep -oE "^sent [0-9]+" "$OUT/drive.log" | cut -d' ' -f2)
OK=$([ "$DRV" = 0 ] && [ "${CON:-1}" = 0 ] && echo true || echo false)
printf '{"state": "done", "ok": %s, "driver_exit": %s, "container_exit": %s, "sent": %s}\n' \
  "$OK" "$DRV" "${CON:-null}" "${SENT:-null}" > "$OUT/status.json"
cat "$OUT/status.json"
[ "$OK" = true ]
