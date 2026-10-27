#!/bin/bash
# slam_link_failed plan E1 (docs/plans/2026-10-25-slam-link-failed.md 3): slam_toolbox online sync + this snapshot's
# bridge inside forklift/ros2-dev:jazzy, for a cost replay (tools/slam_remap_drive.py) -- tools/slam_remap_container.sh
# without the graph serialize and dump, which a cost replay does not read. persistent as the live bridge
# (deploy/console/isaac_live.sbatch): maps written as they come, not kept in memory; the sbatch stops the container.
# A fourth argument picks the node: sync (default, online_sync_launch.py), lifelong (tools/slam_cost_launch.py with
# lifelong_slam_toolbox_node, slam-bounded-graph plan candidate B) or localization (candidate A: the bundle mounted at
# /bundle, its map id and start pose in SLAM_EXPECTED_MAP_ID / SLAM_MAP_START_POSE, the localization params written
# by tools/slam_localization_params.py, then LOC_SETS "key=value ..." applied to them -- the PR #62 variants).
# A fifth argument "graph" also serializes and dumps the final graph (tools/slam_cost_dump.sh). The bridge serves one
# connection and returns when the driver closes it; the dump runs then, before slam_toolbox is stopped.
#   slam_cost_container.sh <socket> <out> <params> [sync|lifelong] [graph]
set +u
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=/workspace/ros2/src/forklift_ros:/workspace/src:${PYTHONPATH}
SOCKET=$1; OUT=$2; PARAMS=$3; NODE=${4:-sync}
LOC_ARGS=()
case $NODE in
  sync) ros2 launch slam_toolbox online_sync_launch.py slam_params_file:="$PARAMS" use_sim_time:=true autostart:=true \
          > "$OUT/slam_toolbox.log" 2>&1 & ;;
  lifelong) ros2 launch /workspace/tools/slam_cost_launch.py executable:=lifelong_slam_toolbox_node \
              slam_params_file:="$PARAMS" use_sim_time:=true > "$OUT/slam_toolbox.log" 2>&1 & ;;
  localization)
    : "${SLAM_EXPECTED_MAP_ID:?the bundle map id}" "${SLAM_MAP_START_POSE:?the start pose x,y,yaw}"
    python3 /workspace/tools/slam_localization_params.py "$PARAMS" "$OUT/localization_params.yaml" /bundle \
      "$SLAM_EXPECTED_MAP_ID" "$SLAM_MAP_START_POSE" > "$OUT/bundle_check.log" 2>&1 || exit 3
    for kv in ${LOC_SETS:-}; do
      k=${kv%%=*}; v=${kv#*=}
      grep -qE "^    $k:" "$OUT/localization_params.yaml" || { echo "unknown param $k" >> "$OUT/bundle_check.log"; exit 3; }
      sed -i "s/^    $k: .*/    $k: $v/" "$OUT/localization_params.yaml"
    done
    LOC_ARGS=(-p localization_bundle:=/bundle -p expected_map_id:="$SLAM_EXPECTED_MAP_ID"
              -p "map_start_pose:=[$SLAM_MAP_START_POSE]")
    ros2 launch slam_toolbox localization_launch.py slam_params_file:="$OUT/localization_params.yaml" \
      use_sim_time:=true autostart:=true > "$OUT/slam_toolbox.log" 2>&1 & ;;
  *) echo "unknown node $NODE" > "$OUT/slam_toolbox.log"; exit 2 ;;
esac
SLAM=$!
python3 -m forklift_ros.isaac_slam_bridge --ros-args \
  -p socket_path:="$SOCKET" -p output_dir:="$OUT" -p "laser_xyz_yaw:=[-0.12,0.0,1.05,0.0]" -p persistent:=true \
  "${LOC_ARGS[@]}" > "$OUT/bridge.log" 2>&1
STATUS=$?
bash /workspace/tools/slam_cost_dump.sh "$OUT" "${5:-}" > "$OUT/dump_run.log" 2>&1
echo "dump exit $?" >> "$OUT/dump_run.log"
kill $SLAM 2>/dev/null; wait $SLAM 2>/dev/null
exit $STATUS
