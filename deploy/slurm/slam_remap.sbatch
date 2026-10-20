#!/bin/bash
#SBATCH --job-name=slam_remap
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=4G
#SBATCH --time=00:45:00
# Replay a run's recorded SLAM scans into a fresh mapping bridge (tools/slam_remap_drive.py) with a params variant, then
# V2' on the saved graph (plans docs/plans/2026-10-20-slam-localization-session.md r6 diagnosis, 2026-10-21 gyro-heading V-G0).
#   sbatch deploy/slurm/slam_remap.sbatch OUT RUN_DIR "key=value ..." [driver args]
#   ODOM_ARGS="--arm gyro --bundle A+ --seed 1000" (environment): build OUT/odom.npz with tools/gyro_heading_offline.py
#   first and replay with it. Writes OUT/status.json (exit codes, serialize, the driver's sent count) and OUT/v2.json.
# Run from the repository root (a snapshot); docker image forklift/ros2-dev:jazzy.
set -u
OUT=$(realpath -m "$1"); RUN=$(realpath "$2"); SETS=$3; shift 3
ROOT=$(pwd)
PY=${VENV_PY:-/home/projects/forklift/.venv/bin/python}
# a fresh directory only: an earlier run's status, graph or replies must never be read as this one's
if [ -e "$OUT" ] && [ -n "$(ls -A "$OUT" 2>/dev/null)" ]; then echo "output $OUT is not empty"; exit 2; fi
mkdir -p "$OUT/bridge" "$OUT/sock"
echo '{"state": "pending"}' > "$OUT/status.json"
JOB=${SLURM_JOB_ID:-local$$}
CPUS=$(taskset -cp $$ | awk -F': ' '{print $2}')
CPU0=$(echo "$CPUS" | cut -d, -f1 | cut -d- -f1)
DRV_CPU=$(echo "$CPUS" | tr ',' '\n' | sed 's/-/\n/' | tail -1)
YAML=$OUT/slam.yaml
cp "$ROOT/${BASE_PARAMS:-ros2/src/forklift_bringup/config/slam_toolbox_isaac_online.yaml}" "$YAML"
for kv in $SETS; do
  k=${kv%%=*}; v=${kv#*=}
  grep -qE "^    $k:" "$YAML" || { echo "unknown param $k"; exit 2; }
  sed -i "s/^    $k: .*/    $k: $v/" "$YAML"
done
ODOM=()
if [ -n "${ODOM_ARGS:-}" ]; then
  # shellcheck disable=SC2086 -- ODOM_ARGS is a list of arguments
  if ! "$PY" "$ROOT/tools/gyro_heading_offline.py" "$RUN" "$OUT/odom.npz" $ODOM_ARGS > "$OUT/odom.json" 2> "$OUT/odom.err"; then
    echo '{"state": "failed", "odom_exit": 1}' > "$OUT/status.json"; exit 3
  fi
  ODOM=(--odom-file "$OUT/odom.npz")
fi
NAME=slam_remap_${JOB}
trap 'docker rm -f "$NAME" >/dev/null 2>&1' EXIT
echo "job=$JOB run=$RUN sets=[$SETS] odom=[${ODOM_ARGS:-}] start $(date -Is)"
docker run -d --name "$NAME" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$ROOT:/workspace:ro" -v "$YAML:/params.yaml:ro" -v "$OUT/bridge:/out" \
  -v "$OUT/sock:/run/slam" forklift/ros2-dev:jazzy bash /workspace/tools/slam_remap_container.sh /run/slam/slam.sock /out /params.yaml >/dev/null
taskset -c "$DRV_CPU" "$PY" "$ROOT/tools/slam_remap_drive.py" "$RUN" "$OUT/sock/slam.sock" "$OUT/drive.jsonl" "${ODOM[@]}" "$@" \
  > "$OUT/drive.log" 2>&1
DRV=$?
CON=$(timeout 600 docker wait "$NAME")
docker logs "$NAME" > "$OUT/container.log" 2>&1
"$PY" "$ROOT/tools/slam_remap_eval.py" "$RUN" "$OUT/bridge/dump" > "$OUT/v2.json"
EV=$?
SENT=$(grep -oE "^sent [0-9]+" "$OUT/drive.log" | cut -d' ' -f2)
SER=$(grep -c "result=0" "$OUT/bridge/serialize.log" 2>/dev/null)
printf '{"state": "done", "driver_exit": %s, "container_exit": %s, "eval_exit": %s, "sent": %s, "serialize_ok": %s}\n' \
  "$DRV" "${CON:-null}" "$EV" "${SENT:-null}" "$([ "${SER:-0}" -ge 1 ] && echo true || echo false)" > "$OUT/status.json"
cat "$OUT/status.json" "$OUT/v2.json"
