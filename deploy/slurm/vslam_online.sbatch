#!/bin/bash
#SBATCH --job-name=vslam_online
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --qos=admin
#SBATCH --cpus-per-task=8
#SBATCH --mem=16G
#SBATCH --time=04:00:00
# Closed-loop survey on a SLAM estimate (docs/plans/2026-10-07-visual-slam-and-fusion.md D6).
# Usage: EXP=<experiment dir> sbatch --output=<log> vslam_online.sbatch <name> <seed> <config> <condition> [run_slam_drive args...]
#   config: lidar_st (isaac_slam_bridge + slam_toolbox, the 10/04 setup) | vision | fusion
#           (rig_slam_bridge + RTAB-Map). Noise: the plan's assumed noise, seed = layout seed.
set -uo pipefail
EXP=${EXP:?set EXP to the experiment directory}
NAME=$1; SEED=$2; CONFIG=$3; CONDITION=$4; shift 4
OUT="$EXP/online/$NAME"
# The socket lives in a short host path: AF_UNIX paths stop at 107 bytes and
# "$OUT/sock/slam.sock" passes that for long run names.
SOCK="/tmp/vslam_${SLURM_JOB_ID}"
mkdir -p "$OUT/bridge" "$SOCK"
CONTAINER="vslam_online_${SLURM_JOB_ID}"
cleanup() { docker rm -f "$CONTAINER" >/dev/null 2>&1 || true; rm -rf "$SOCK"; }
trap cleanup EXIT
export OMNI_KIT_ACCEPT_EULA=YES OMP_NUM_THREADS=4
CPUS=$(python3 -c 'import os; c=sorted(os.sched_getaffinity(0)); print(",".join(map(str, c[-3:])))')
LASER="-0.12,0.0,1.05,0.0"
if [ "$CONFIG" = lidar_st ]; then
  BACKEND=slam_toolbox
  INNER=(bash /workspace/tools/slam_bridge_container.sh /run/slam/slam.sock /out "$LASER")
else
  BACKEND=rig
  INNER=(bash /workspace/tools/rig_slam_container.sh /out "$CONFIG" -p source:=socket
         -p socket_path:=/run/slam/slam.sock -p rig_config:=/workspace/config/isaac_depth_rig.yaml
         -p "laser_xyz_yaw:=[$LASER]" -p condition:=$CONDITION -p "cameras:=[front,rear,left,right]")
fi
docker run -d --name "$CONTAINER" --network none --cpuset-cpus "$CPUS" --memory 6g --shm-size 2g \
  -u "$(id -u):$(id -g)" --group-add 2000 -e HOME=/tmp -e ROS_LOG_DIR=/out/ros_logs \
  -v "$EXP/source:/workspace:ro" -v "$OUT/bridge:/out" -v "$SOCK:/run/slam" \
  forklift/ros2-dev:jazzy-vslam "${INNER[@]}" > "$OUT/container_id.txt"
for _ in $(seq 1 240); do test -f "$OUT/bridge/bridge_ready" && break; sleep 1; done
if ! test -f "$OUT/bridge/bridge_ready"; then
  docker logs "$CONTAINER" > "$OUT/container_startup.log" 2>&1
  echo "bridge never became ready" > "$OUT/failed.txt"
  exit 3
fi
echo "simulation start $(date -Is) $CONFIG seed=$SEED condition=$CONDITION cpus=$CPUS" > "$OUT/stages.log"
cd "$EXP/source"
timeout 12000 "$EXP/env/bin/python" sim/isaac/run_slam_drive.py \
  --base-scene /home/projects/forklift/artifacts/base_scene_warehouse_forklift/scene.usda \
  --forklift-urdf sim/models/dls08_provisional/forklift.urdf \
  --output "$OUT/run" --seed "$SEED" --depth-rig config/isaac_depth_rig.yaml \
  --slam-feedback "$SOCK/slam.sock" --slam-backend "$BACKEND" --slam-noise-seed "$SEED" \
  --slam-condition "$CONDITION" "$@" \
  --portable-root /home/projects/forklift/cache/isaac_v2 > "$OUT/run.log" 2>&1
STATUS=$?
echo "simulation end $(date -Is) exit $STATUS" >> "$OUT/stages.log"
timeout 120 docker wait "$CONTAINER" > "$OUT/bridge_exit.txt" 2>&1 || true
docker logs "$CONTAINER" > "$OUT/container.log" 2>&1 || true
exit $STATUS
