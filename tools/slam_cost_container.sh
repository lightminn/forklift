#!/bin/bash
# slam_link_failed plan E1 (docs/plans/2026-10-25-slam-link-failed.md 3): slam_toolbox online sync + this snapshot's
# bridge inside forklift/ros2-dev:jazzy, for a cost replay (tools/slam_remap_drive.py) -- tools/slam_remap_container.sh
# without the graph serialize and dump, which a cost replay does not read. persistent as the live bridge
# (deploy/console/isaac_live.sbatch): maps written as they come, not kept in memory; the sbatch stops the container.
# A fourth argument picks the node: sync (default, online_sync_launch.py) or lifelong (tools/slam_cost_launch.py with
# lifelong_slam_toolbox_node, slam-bounded-graph plan candidate B).
#   slam_cost_container.sh <socket> <out> <params> [sync|lifelong]
set +u
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=/workspace/ros2/src/forklift_ros:/workspace/src:${PYTHONPATH}
SOCKET=$1; OUT=$2; PARAMS=$3; NODE=${4:-sync}
case $NODE in
  sync) ros2 launch slam_toolbox online_sync_launch.py slam_params_file:="$PARAMS" use_sim_time:=true autostart:=true \
          > "$OUT/slam_toolbox.log" 2>&1 & ;;
  lifelong) ros2 launch /workspace/tools/slam_cost_launch.py executable:=lifelong_slam_toolbox_node \
              slam_params_file:="$PARAMS" use_sim_time:=true > "$OUT/slam_toolbox.log" 2>&1 & ;;
  *) echo "unknown node $NODE" > "$OUT/slam_toolbox.log"; exit 2 ;;
esac
SLAM=$!
python3 -m forklift_ros.isaac_slam_bridge --ros-args \
  -p socket_path:="$SOCKET" -p output_dir:="$OUT" -p "laser_xyz_yaw:=[-0.12,0.0,1.05,0.0]" -p persistent:=true \
  > "$OUT/bridge.log" 2>&1
STATUS=$?
kill $SLAM 2>/dev/null; wait $SLAM 2>/dev/null
exit $STATUS
