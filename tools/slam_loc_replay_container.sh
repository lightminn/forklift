#!/bin/bash
# SLAM localization replay (docs/plans/2026-10-20-slam-localization-session.md, the V5 diagnosis) inside
# forklift/ros2-dev:jazzy: slam_toolbox in localization mode on a survey bundle with the given params file, and the
# bridge with keyframe_gate every (tools/slam_loc_replay_drive.py sends the recorded keyframes alone).
#   slam_loc_replay_container.sh <socket> <out> <params>
#   environment: SLAM_MAP_BUNDLE (mounted bundle folder), SLAM_EXPECTED_MAP_ID, SLAM_MAP_START_POSE (x,y,yaw)
set +u
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=/workspace/ros2/src/forklift_ros:/workspace/src:${PYTHONPATH}
SOCKET=$1; OUT=$2; PARAMS=$3
: "${SLAM_MAP_BUNDLE:?bundle}" "${SLAM_EXPECTED_MAP_ID:?map id}" "${SLAM_MAP_START_POSE:?start pose}"
mkdir -p "$OUT"
ros2 launch slam_toolbox localization_launch.py slam_params_file:="$PARAMS" use_sim_time:=true autostart:=true \
  > "$OUT/slam_toolbox.log" 2>&1 &
SLAM=$!
python3 -m forklift_ros.isaac_slam_bridge --ros-args \
  -p socket_path:="$SOCKET" -p output_dir:="$OUT" -p "laser_xyz_yaw:=[-0.12,0.0,1.05,0.0]" -p keyframe_gate:=every \
  -p localization_bundle:="$SLAM_MAP_BUNDLE" -p expected_map_id:="$SLAM_EXPECTED_MAP_ID" \
  -p "map_start_pose:=[$SLAM_MAP_START_POSE]" > "$OUT/bridge.log" 2>&1
STATUS=$?
kill $SLAM 2>/dev/null; wait $SLAM 2>/dev/null
exit $STATUS
