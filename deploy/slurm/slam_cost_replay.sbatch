#!/bin/bash
#SBATCH --job-name=slam_cost
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=3G
#SBATCH --time=01:00:00
# slam_link_failed plan E1 (docs/plans/2026-10-25-slam-link-failed.md 3): a run's recorded scans replayed into a fresh
# mapping bridge (tools/slam_remap_drive.py) with a params variant, slam_toolbox on one CPU of the job and the driver
# on the other, then tools/slam_cost_summary.py. Records the CPU state (the SLAM CPU's SMT siblings, node load, the
# container's cpu.pressure) beside the result.
#   sbatch deploy/slurm/slam_cost_replay.sbatch OUT RUN_DIR "key=value ..." [driver args, e.g. --noise-seed 1
#       --allow-unanswered-tail 8 --pace-live --splice-to-sim-s 1500]
#   LIVE_BRIDGE (environment): the live run's bridge folder, for the summary's validity check
# Run from the repository root (a snapshot); docker image forklift/ros2-dev:jazzy.
set -u
OUT=$(realpath -m "$1"); RUN=$(realpath "$2"); SETS=$3; shift 3
ROOT=$(pwd)
PY=${VENV_PY:-/home/projects/forklift/.venv/bin/python}
if [ -e "$OUT" ] && [ -n "$(ls -A "$OUT" 2>/dev/null)" ]; then echo "output $OUT is not empty"; exit 2; fi
mkdir -p "$OUT/bridge" "$OUT/sock"
echo '{"state": "pending"}' > "$OUT/status.json"
JOB=${SLURM_JOB_ID:-local$$}
CPUS=$(taskset -cp $$ | awk -F': ' '{print $2}')
CPU0=$(echo "$CPUS" | cut -d, -f1 | cut -d- -f1)
DRV_CPU=$(echo "$CPUS" | tr ',' '\n' | sed 's/-/\n/' | tail -1)
YAML=$OUT/slam.yaml
cp "$ROOT/ros2/src/forklift_bringup/config/slam_toolbox_isaac_online.yaml" "$YAML"
for kv in $SETS; do
  k=${kv%%=*}; v=${kv#*=}
  grep -qE "^    $k:" "$YAML" || { echo "unknown param $k"; exit 2; }
  sed -i "s/^    $k: .*/    $k: $v/" "$YAML"
done
NAME=slam_cost_${JOB}
trap 'docker rm -f "$NAME" >/dev/null 2>&1' EXIT
{
  echo "job=$JOB run=$RUN sets=[$SETS] args=[$*] cpus=$CPUS slam_cpu=$CPU0 driver_cpu=$DRV_CPU"
  echo "slam_cpu_siblings=$(cat /sys/devices/system/cpu/cpu$CPU0/topology/thread_siblings_list 2>/dev/null)"
  echo "load_start=$(cut -d' ' -f1-3 /proc/loadavg) start=$(date -Is)"
} > "$OUT/env.txt"
docker run -d --name "$NAME" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$ROOT:/workspace:ro" -v "$YAML:/params.yaml:ro" -v "$OUT/bridge:/out" \
  -v "$OUT/sock:/run/slam" forklift/ros2-dev:jazzy bash /workspace/tools/slam_cost_container.sh /run/slam/slam.sock /out /params.yaml >/dev/null
CID=$(docker inspect -f '{{.Id}}' "$NAME" 2>/dev/null)
taskset -c "$DRV_CPU" "$PY" "$ROOT/tools/slam_remap_drive.py" "$RUN" "$OUT/sock/slam.sock" "$OUT/drive.jsonl" "$@" \
  > "$OUT/drive.log" 2>&1
DRV=$?
{
  echo "load_end=$(cut -d' ' -f1-3 /proc/loadavg) end=$(date -Is)"
  echo "cpu_pressure=$(cat /sys/fs/cgroup/system.slice/docker-$CID.scope/cpu.pressure 2>/dev/null | tr '\n' ' ')"
} >> "$OUT/env.txt"
CON=$(timeout 120 docker wait "$NAME")
docker logs "$NAME" > "$OUT/container.log" 2>&1
SUMMARY_ARGS=()
[ -n "${LIVE_BRIDGE:-}" ] && SUMMARY_ARGS=(--live-bridge "$LIVE_BRIDGE")
"$PY" "$ROOT/tools/slam_cost_summary.py" "$OUT/drive.jsonl" "${SUMMARY_ARGS[@]}" > "$OUT/summary.json"
SUM=$?
SENT=$(grep -oE "^sent [0-9]+" "$OUT/drive.log" | cut -d' ' -f2)
printf '{"state": "done", "driver_exit": %s, "container_exit": %s, "summary_exit": %s, "sent": %s}\n' \
  "$DRV" "${CON:-null}" "$SUM" "${SENT:-null}" > "$OUT/status.json"
cat "$OUT/status.json"
