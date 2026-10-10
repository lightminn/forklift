#!/bin/bash
#SBATCH --job-name=fleet
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=24G
#SBATCH --time=06:00:00
# Five-truck fleet run (docs/plans/2026-10-08-fleet-five-forklifts.md).
# Usage: EXP=<experiment dir> BASE_SCENE=<measured base scene.usda> PORTABLE_ROOT=<Kit cache> \
#        sbatch --output=<log> fleet_run.sbatch <name> <seed> <slam|truth> [run_fleet args...]
# EXP holds source/ (snapshot) and env/ (venv: the snapshot's src + the shared Isaac runtime).
# slam: one slam_toolbox container per truck (tools/slam_bridge_container.sh), each on
# one of the job's CPUs, sockets in a short /tmp path; Isaac keeps the rest.
set -uo pipefail
EXP=${EXP:?set EXP}; BASE_SCENE=${BASE_SCENE:?set BASE_SCENE}; PORTABLE_ROOT=${PORTABLE_ROOT:?set PORTABLE_ROOT}
NAME=$1; SEED=$2; MODE=$3; shift 3
TRUCKS=5
OUT="$EXP/runs/$NAME"
mkdir -p "$EXP/runs"
SOCK="/tmp/fleet_${SLURM_JOB_ID:-$$}"
mkdir -p "$SOCK"
mapfile -t CPUS < <(python3 -c 'import os; print("\n".join(map(str, sorted(os.sched_getaffinity(0)))))')
cleanup() {
  for k in $(seq 0 $((TRUCKS - 1))); do
    docker logs "fleet_${SLURM_JOB_ID:-$$}_$k" > "$OUT.bridges/truck_$k/container.log" 2>&1 || true
    docker rm -f "fleet_${SLURM_JOB_ID:-$$}_$k" > /dev/null 2>&1 || true
  done
  rm -rf "$SOCK"
}
trap cleanup EXIT
export OMNI_KIT_ACCEPT_EULA=YES OMP_NUM_THREADS=3
LASER="-0.12,0.0,1.05,0.0"
SLAM_ARGS=()
if [ "$MODE" = slam ]; then
  for k in $(seq 0 $((TRUCKS - 1))); do
    CPU=${CPUS[$((${#CPUS[@]} - 1 - k))]}
    mkdir -p "$OUT.bridges/truck_$k"
    docker run -d --name "fleet_${SLURM_JOB_ID:-$$}_$k" --network none --cpuset-cpus "$CPU" \
      --memory 2g --shm-size 512m -u "$(id -u):$(id -g)" --group-add 2000 -e HOME=/tmp \
      -e ROS_LOG_DIR=/out/ros_logs -v "$EXP/source:/workspace:ro" -v "$OUT.bridges/truck_$k:/out" \
      -v "$SOCK:/run/slam" forklift/ros2-dev:jazzy \
      bash /workspace/tools/slam_bridge_container.sh "/run/slam/truck_$k.sock" /out "$LASER" > /dev/null
  done
  for _ in $(seq 1 240); do
    ready=$(ls "$OUT".bridges/truck_*/bridge_ready 2> /dev/null | wc -l)
    [ "$ready" -ge "$TRUCKS" ] && break
    sleep 1
  done
  if [ "$ready" -lt "$TRUCKS" ]; then
    echo "only $ready of $TRUCKS bridges became ready" >&2
    exit 3
  fi
  SLAM_ARGS=(--slam-sockets "$SOCK" --slam-noise-seed "$SEED")
fi
echo "start $(date -Is) $NAME seed=$SEED mode=$MODE cpus=${CPUS[*]}" > "$EXP/runs/$NAME.job.txt"
cd "$EXP/source"
timeout 20000 "$EXP/env/bin/python" sim/isaac/run_fleet.py --base-scene "$BASE_SCENE" \
  --seed "$SEED" --output "$OUT" "${SLAM_ARGS[@]}" "$@" \
  --portable-root "$PORTABLE_ROOT" > "$EXP/runs/$NAME.log" 2>&1
STATUS=$?
echo "end $(date -Is) exit $STATUS" >> "$EXP/runs/$NAME.job.txt"
exit $STATUS
