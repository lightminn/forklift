#!/bin/bash
#SBATCH --job-name=vslam_replay
#SBATCH --partition=compute
#SBATCH --qos=admin
#SBATCH --cpus-per-task=4
#SBATCH --mem=5G
#SBATCH --time=03:00:00
# Offline SLAM replay of one rig record (docs/plans/2026-10-07-visual-slam-and-fusion.md).
# Usage: [TAG=<suffix>] sbatch --output=<log> vslam_replay.sbatch <experiment dir> <record> <config> <noise seed|none> <condition> [bridge ros-args...]
#   config: lidar_st (slam_toolbox, existing pipeline) | lidar | vision | vision_front
#           | vision_noodom | fusion (rig_slam_bridge + RTAB-Map)
# The experiment dir holds source/ (the snapshot), records/<record>/, and
# ros2_install/ (colcon install of source/ros2/src, for lidar_st).
set -uo pipefail
EXP=$1; RECORD=$2; CONFIG=$3; NOISE=$4; CONDITION=$5; shift 5
OUT="$EXP/replays/$RECORD/${CONFIG}_${NOISE}_${CONDITION}${TAG:-}"
mkdir -p "$OUT"
CONTAINER="vslam_replay_${SLURM_JOB_ID:-local$$}"
cleanup() { docker rm -f "$CONTAINER" >/dev/null 2>&1 || true; }
trap cleanup EXIT
if [ -n "${SLURM_JOB_ID:-}" ]; then
  CPUS=$(python3 -c 'import os; print(",".join(str(i) for i in sorted(os.sched_getaffinity(0))))')
  LIMIT=(--cpuset-cpus "$CPUS")
else
  # Run outside Slurm (the node drained for GPU maintenance): two CPUs' worth.
  CPUS="2 (no Slurm)"; LIMIT=(--cpus 2)
fi
RUN=(docker run --rm --name "$CONTAINER" --network none "${LIMIT[@]}" --memory 6g --shm-size 2g
     -u "$(id -u):$(id -g)" --group-add 2000 -e HOME=/tmp -e ROS_LOG_DIR=/out/ros_logs
     -v "$EXP/source:/workspace:ro" -v "$EXP/records/$RECORD:/rec:ro" -v "$OUT:/out"
     -v "$EXP/ros2_install:/ros2_install:ro" forklift/ros2-dev:jazzy-vslam)
echo "start $(date -Is) $CONFIG noise=$NOISE condition=$CONDITION cpus=$CPUS" > "$OUT/job.txt"
if [ "$CONFIG" = lidar_st ]; then
  NOISE_ARGS=""
  if [ "$NOISE" != none ]; then
    NOISE_ARGS="--range-noise-std-m 0.02 --wheel-rate-noise-std-rad-s 0.2 --steering-noise-std-rad 0.005 --seed $NOISE"
  fi
  "${RUN[@]}" bash -c "set +u; source /opt/ros/jazzy/setup.bash; source /ros2_install/setup.bash;
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONPATH=/workspace/src:\$PYTHONPATH;
    python3 -m forklift_ros.slam_replay --record /rec --output /out/replay $NOISE_ARGS --condition $CONDITION &&
    ros2 launch forklift_bringup isaac_slam_replay.launch.py bag:=/out/replay/bag output:=/out/slam rate:=1.0" \
    > "$OUT/run.log" 2>&1
  STATUS=$?
else
  MODE=$CONFIG; CAMERAS="[front,rear,left,right]"
  [ "$CONFIG" = vision_front ] && { MODE=vision; CAMERAS="[front]"; }
  SEED=-1; [ "$NOISE" != none ] && SEED=$NOISE
  "${RUN[@]}" bash /workspace/tools/rig_slam_container.sh /out "$MODE" \
    -p source:=record -p record_dir:=/rec -p noise_seed:=$SEED -p condition:=$CONDITION \
    -p "cameras:=$CAMERAS" "$@" > "$OUT/run.log" 2>&1
  STATUS=$?
fi
echo "end $(date -Is) exit $STATUS" >> "$OUT/job.txt"
exit $STATUS
