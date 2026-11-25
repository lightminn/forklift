#!/bin/bash
#SBATCH --job-name=slam_instance_premise
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=4G
#SBATCH --time=00:15:00
# PR 7c-1a's first test (docs/design/2026-11-25-real-sources-pr7.md 6.1, 9): two bridge-owned slam_toolbox instances one
# after the other on a survey bundle, scans ray-cast from its grid (tools/slam_instance_premise.py).
#   sbatch deploy/slurm/slam_instance_premise.sbatch OUT BUNDLE_DIR MAP_ID
# Run from the repository root (a snapshot); docker image forklift/ros2-dev:jazzy. Writes OUT/premise.json.
set -u
OUT=$(realpath -m "$1"); BUNDLE=$(realpath "$2"); MAP_ID=$3
ROOT=$(pwd)
if [ -e "$OUT" ] && [ -n "$(ls -A "$OUT" 2>/dev/null)" ]; then echo "output $OUT is not empty"; exit 2; fi
mkdir -p "$OUT"
JOB=${SLURM_JOB_ID:-local$$}
CPUS=$(taskset -cp $$ | awk -F': ' '{print $2}')
NAME=slam_instance_premise_${JOB}
trap 'docker rm -f "$NAME" >/dev/null 2>&1' EXIT
echo "job=$JOB bundle=$BUNDLE start $(date -Is)"
docker run --name "$NAME" --network none --cpuset-cpus "$CPUS" --memory 4g --group-add 2000 \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$ROOT:/workspace:ro" -v "$BUNDLE:/bundle:ro" -v "$OUT:/out" \
  forklift/ros2-dev:jazzy bash -c "source /opt/ros/jazzy/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    ROS_DOMAIN_ID=77 ROS_LOG_DIR=/tmp/roslog && python3 /workspace/tools/slam_instance_premise.py /bundle $MAP_ID /out"
STATUS=$?
echo "PREMISE_EXIT $STATUS"
exit $STATUS
