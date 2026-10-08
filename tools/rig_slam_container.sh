#!/bin/bash
# Inside forklift/ros2-dev:jazzy-vslam: RTAB-Map nodes for one mode plus
# forklift_ros rig_slam_bridge, on a localhost-only DDS.
# Usage: rig_slam_container.sh <output_dir> <mode> <bridge ros-args...>
#   e.g. ... /out fusion -p source:=record -p record_dir:=/rec -p noise_seed:=0
# (docs/plans/2026-10-07-visual-slam-and-fusion.md)
set +u
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=/workspace/ros2/src/forklift_ros:/workspace/src:${PYTHONPATH}
OUT=$1; MODE=$2; shift 2
mkdir -p "$OUT" "${HOME:-/tmp}/.ros"
# VO_PARAMS / ICP_PARAMS / SLAM_PARAMS: JSON overrides for development runs only.
ros2 launch /workspace/ros2/src/forklift_bringup/launch/rig_slam.launch.py \
  mode:="$MODE" output:="$OUT" vo_params:="${VO_PARAMS:-{\}}" \
  icp_params:="${ICP_PARAMS:-{\}}" slam_params:="${SLAM_PARAMS:-{\}}" > "$OUT/rtabmap.log" 2>&1 &
SLAM=$!
python3 -m forklift_ros.rig_slam_bridge --ros-args -p mode:="$MODE" \
  -p output_dir:="$OUT" -p use_sim_time:=false "$@" > "$OUT/bridge.log" 2>&1
STATUS=$?
kill -INT $SLAM 2>/dev/null
for _ in $(seq 1 30); do kill -0 $SLAM 2>/dev/null || break; sleep 1; done
kill -9 $SLAM 2>/dev/null
wait $SLAM 2>/dev/null
exit $STATUS
