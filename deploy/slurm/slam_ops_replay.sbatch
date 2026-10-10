#!/bin/bash
#SBATCH --job-name=slam_ops
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=8G
#SBATCH --time=01:00:00
# Saved-map replays for docs/plans/2026-10-08-slam-operating-conditions.md.
# Usage: EXP=<experiment dir> sbatch --output=<log> slam_ops_replay.sbatch <name> map <record> <noise seed> <condition>
#        EXP=... sbatch ... slam_ops_replay.sbatch <name> localize <record> <noise seed> <condition> \
#            <map name> <map record> <offset s> <error m> <error yaw rad>
# EXP holds source/ (snapshot), ros2_install/ (colcon install of source/ros2/src),
# records/<record>/ (Isaac records, read only) and maps/<map name>/posegraph.*.
# map: slam_toolbox mapping replay (the existing L-ST pipeline) that also saves
#      the pose graph to $EXP/maps/<name>/. localize: localization on a saved map,
#      the bag started <offset s> in, first pose = truth + the given error.
set -uo pipefail
EXP=${EXP:?set EXP to the experiment directory}
NAME=$1; MODE=$2; RECORD=$3; NOISE=$4; CONDITION=$5; shift 5
OUT="$EXP/replays/$NAME"
mkdir -p "$OUT"
CONTAINER="slam_ops_${SLURM_JOB_ID:-local$$}"
cleanup() { docker rm -f "$CONTAINER" >/dev/null 2>&1 || true; }
trap cleanup EXIT
CPUS=$(python3 -c 'import os; print(",".join(str(i) for i in sorted(os.sched_getaffinity(0))))')
MOUNTS=(-v "$EXP/source:/workspace:ro" -v "$EXP/records:/records:ro" -v "$OUT:/out"
        -v "$EXP/ros2_install:/ros2_install:ro")
NOISE_ARGS="--range-noise-std-m 0.02 --wheel-rate-noise-std-rad-s 0.2 --steering-noise-std-rad 0.005 --seed $NOISE"
SETUP="set +u; source /opt/ros/jazzy/setup.bash; source /ros2_install/setup.bash;
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONPATH=/workspace/src:\$PYTHONPATH; cd /workspace;
  python3 -m forklift_ros.slam_replay --record /records/$RECORD --output /out/replay $NOISE_ARGS --condition $CONDITION"
if [ "$MODE" = map ]; then
  mkdir -p "$EXP/maps/$NAME"
  MOUNTS+=(-v "$EXP/maps/$NAME:/map")
  INNER="$SETUP && ros2 launch forklift_bringup isaac_slam_replay.launch.py bag:=/out/replay/bag \
    output:=/out/slam rate:=1.0 serialize:=/map/posegraph"
  echo "map $RECORD noise=$NOISE condition=$CONDITION" > "$OUT/job.txt"
else
  MAP=$1; MAP_RECORD=$2; OFFSET=$3; ERR_M=$4; ERR_YAW=$5
  MOUNTS+=(-v "$EXP/maps/$MAP:/map:ro")
  INNER="$SETUP && POSE=\$(python3 tools/slam_localization.py start-pose --map-record /records/$MAP_RECORD \
      --record /records/$RECORD --offset-s $OFFSET --error-m $ERR_M --error-yaw-rad $ERR_YAW) &&
    python3 tools/slam_localization.py params --base ros2/src/forklift_bringup/config/slam_toolbox_isaac_replay.yaml \
      --map-file /map/posegraph --start-pose \$POSE --output /out/localization.yaml &&
    echo \"start pose \$POSE\" &&
    ros2 launch forklift_bringup isaac_slam_localization.launch.py bag:=/out/replay/bag output:=/out/slam \
      params:=/out/localization.yaml start_offset:=$OFFSET"
  echo "localize $RECORD on $MAP ($MAP_RECORD) noise=$NOISE condition=$CONDITION offset=$OFFSET error=$ERR_M,$ERR_YAW" > "$OUT/job.txt"
fi
echo "start $(date -Is) cpus=$CPUS" >> "$OUT/job.txt"
docker run --rm --name "$CONTAINER" --network none --cpuset-cpus "$CPUS" --memory 6g --shm-size 2g \
  -u "$(id -u):$(id -g)" --group-add 2000 -e HOME=/tmp -e ROS_LOG_DIR=/out/ros_logs "${MOUNTS[@]}" \
  forklift/ros2-dev:jazzy bash -c "$INNER" > "$OUT/run.log" 2>&1
STATUS=$?
echo "end $(date -Is) exit $STATUS" >> "$OUT/job.txt"
exit $STATUS
