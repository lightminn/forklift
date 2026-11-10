#!/bin/bash
# Hall SLAM replay (docs/validation/2026-11-10-hall-slam-replay.md) inside forklift/ros2-dev:jazzy: slam_toolbox in
# online mapping mode with the live sessions' params (online sync, the bridge's default) and the bridge with
# keyframe_gate every, so tools/slam_loc_replay_drive.py sends the recorded scans alone -- whatever odometry an arm gives.
#   slam_map_replay_container.sh <socket> <out>
set +u
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=/workspace/ros2/src/forklift_ros:/workspace/src:${PYTHONPATH}
SOCKET=$1; OUT=$2
PARAMS=/workspace/ros2/src/forklift_bringup/config/slam_toolbox_isaac_online.yaml  # tools/slam_bridge_container.sh's
mkdir -p "$OUT"
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:="$PARAMS" use_sim_time:=true autostart:=true \
  > "$OUT/slam_toolbox.log" 2>&1 &
SLAM=$!
python3 -m forklift_ros.isaac_slam_bridge --ros-args \
  -p socket_path:="$SOCKET" -p output_dir:="$OUT" -p "laser_xyz_yaw:=[-0.12,0.0,1.05,0.0]" -p keyframe_gate:=every \
  > "$OUT/bridge.log" 2>&1
STATUS=$?
kill $SLAM 2>/dev/null; wait $SLAM 2>/dev/null
exit $STATUS
