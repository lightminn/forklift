#!/bin/bash
# slam_link_failed plan E1 (docs/plans/2026-10-25-slam-link-failed.md 3): slam_toolbox online sync + this snapshot's
# bridge inside forklift/ros2-dev:jazzy, for a cost replay (tools/slam_remap_drive.py) -- tools/slam_remap_container.sh
# without the graph serialize and dump, which a cost replay does not read.
#   slam_cost_container.sh <socket> <out> <params>
set +u
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=/workspace/ros2/src/forklift_ros:/workspace/src:${PYTHONPATH}
SOCKET=$1; OUT=$2; PARAMS=$3
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:="$PARAMS" use_sim_time:=true autostart:=true > "$OUT/slam_toolbox.log" 2>&1 &
SLAM=$!
python3 -m forklift_ros.isaac_slam_bridge --ros-args \
  -p socket_path:="$SOCKET" -p output_dir:="$OUT" -p "laser_xyz_yaw:=[-0.12,0.0,1.05,0.0]" \
  > "$OUT/bridge.log" 2>&1
STATUS=$?
kill $SLAM 2>/dev/null; wait $SLAM 2>/dev/null
exit $STATUS
