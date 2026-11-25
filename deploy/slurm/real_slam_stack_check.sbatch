#!/bin/bash
#SBATCH --job-name=real_slam_stack_check
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=4G
#SBATCH --time=00:20:00
# PR 7c-1a on the real SLAM stack: forklift_ros.real_slam_bridge + slam_toolbox in one container, driven by
# tools/real_slam_stack_check.py over the bridge's socket (docs/design/2026-11-25-real-sources-pr7.md 6.1, 9).
#   sbatch deploy/slurm/real_slam_stack_check.sbatch OUT BUNDLE_DIR
# Run from the repository root (a snapshot); docker image forklift/ros2-dev:jazzy. Writes OUT/stack_check.json.
set -u
OUT=$(realpath -m "$1"); BUNDLE=$(realpath "$2")
ROOT=$(pwd)
if [ -e "$OUT" ] && [ -n "$(ls -A "$OUT" 2>/dev/null)" ]; then echo "output $OUT is not empty"; exit 2; fi
mkdir -p "$OUT/bridge"
JOB=${SLURM_JOB_ID:-local$$}
CPUS=$(taskset -cp $$ | awk -F': ' '{print $2}')
NAME=real_slam_stack_check_${JOB}
trap 'docker rm -f "$NAME" >/dev/null 2>&1' EXIT
echo "job=$JOB bundle=$BUNDLE start $(date -Is)"
docker run --name "$NAME" --network none --cpuset-cpus "$CPUS" --memory 4g --group-add 2000 \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$ROOT:/workspace:ro" -v "$BUNDLE:/bundle:ro" -v "$OUT:/out" \
  forklift/ros2-dev:jazzy bash -c '
    source /opt/ros/jazzy/setup.bash
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=78 ROS_LOG_DIR=/tmp/roslog
    export PYTHONPATH=/workspace/ros2/src/forklift_ros:/workspace/src:${PYTHONPATH:-}
    python3 -m forklift_ros.real_slam_bridge --ros-args -p socket_path:=/tmp/slam.sock \
      -p localization_bundle:=/bundle -p "laser_xyz_yaw:=[0.0, 0.0, 0.0, 0.0]" \
      -p mapping_params:=/workspace/ros2/src/forklift_bringup/config/slam_toolbox_isaac_online.yaml \
      -p output_dir:=/out/bridge > /out/bridge.log 2>&1 &
    python3 /workspace/tools/real_slam_stack_check.py /tmp/slam.sock /bundle /out
    STATUS=$?
    wait
    exit $STATUS'
STATUS=$?
echo "STACK_EXIT $STATUS"
exit $STATUS
