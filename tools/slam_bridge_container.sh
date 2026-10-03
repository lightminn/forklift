#!/bin/bash
# Inside forklift/ros2-dev:jazzy: slam_toolbox (online sync, lifecycle autostart)
# plus forklift_ros isaac_slam_bridge, both on a localhost-only DDS.
# Usage: slam_bridge_container.sh <socket_path> <output_dir> <laser x,y,z,yaw>
# (docs/plans/2026-10-04-online-slam-closed-loop.md)
set +u
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=/workspace/ros2/src/forklift_ros:/workspace/src:${PYTHONPATH}
SOCKET=$1; OUT=$2; LASER=$3
# Thresholds 0: slam_toolbox answers every scan; the bridge sends keyframes only (plan v3.4).
PARAMS=/workspace/ros2/src/forklift_bringup/config/slam_toolbox_isaac_online.yaml
mkdir -p "$OUT"
ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:="$PARAMS" \
  use_sim_time:=true autostart:=true > "$OUT/slam_toolbox.log" 2>&1 &
SLAM=$!
python3 -m forklift_ros.isaac_slam_bridge --ros-args \
  -p socket_path:="$SOCKET" -p output_dir:="$OUT" -p "laser_xyz_yaw:=[$LASER]" \
  > "$OUT/bridge.log" 2>&1
STATUS=$?
kill $SLAM 2>/dev/null
wait $SLAM 2>/dev/null
exit $STATUS
