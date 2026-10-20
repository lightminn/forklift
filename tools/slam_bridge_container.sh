#!/bin/bash
# Inside forklift/ros2-dev:jazzy: slam_toolbox (online sync, lifecycle autostart)
# plus forklift_ros isaac_slam_bridge, both on a localhost-only DDS.
# Usage: slam_bridge_container.sh <socket_path> <output_dir> <laser x,y,z,yaw> [persistent]
# persistent: a live session left up for hours keeps no map or reply record in memory (persistent console plan D8b)
# SLAM_SERIALIZE=<prefix> (environment): once the bridge has ended -- the runner closed its socket, so no scan is
# sent or being processed -- slam_toolbox writes its pose graph to <prefix>.posegraph/.data before it is stopped
# (a survey epoch, docs/plans/2026-10-20-slam-localization-session.md L2)
# SLAM_PARAMS=<yaml> (environment): the mapping params, the online config when unset (a survey epoch passes its own)
# (docs/plans/2026-10-04-online-slam-closed-loop.md)
set +u
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=/workspace/ros2/src/forklift_ros:/workspace/src:${PYTHONPATH}
SOCKET=$1; OUT=$2; LASER=$3
PERSIST=false
[ "$4" = persistent ] && PERSIST=true
# Thresholds 0: slam_toolbox answers every scan; the bridge sends keyframes only (plan v3.4).
PARAMS=${SLAM_PARAMS:-/workspace/ros2/src/forklift_bringup/config/slam_toolbox_isaac_online.yaml}
mkdir -p "$OUT"
ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:="$PARAMS" \
  use_sim_time:=true autostart:=true > "$OUT/slam_toolbox.log" 2>&1 &
SLAM=$!
python3 -m forklift_ros.isaac_slam_bridge --ros-args \
  -p socket_path:="$SOCKET" -p output_dir:="$OUT" -p "laser_xyz_yaw:=[$LASER]" -p persistent:=$PERSIST \
  > "$OUT/bridge.log" 2>&1
STATUS=$?
if [ -n "${SLAM_SERIALIZE:-}" ]; then
  mkdir -p "$(dirname "$SLAM_SERIALIZE")"
  timeout 300 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$SLAM_SERIALIZE'}" > "$OUT/serialize.log" 2>&1
  echo "serialize exit $?" >> "$OUT/serialize.log"
fi
kill $SLAM 2>/dev/null
wait $SLAM 2>/dev/null
exit $STATUS
