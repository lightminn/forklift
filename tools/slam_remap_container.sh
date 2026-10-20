#!/bin/bash
# SLAM remap (plans 2026-10-20 L2 r6 diagnosis, 2026-10-21 gyro-heading V-G0) inside forklift/ros2-dev:jazzy: slam_toolbox online sync + the snapshot bridge; when the bridge
# ends, serialize the graph and dump its nodes with tools/slam_graph_dump.cpp.
#   slam_remap_container.sh <socket> <out> <params>
set +u
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=/workspace/ros2/src/forklift_ros:/workspace/src:${PYTHONPATH}
SOCKET=$1; OUT=$2; PARAMS=$3
mkdir -p "$OUT/bundle" "$OUT/dump" /tmp/build
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:="$PARAMS" use_sim_time:=true autostart:=true > "$OUT/slam_toolbox.log" 2>&1 &
SLAM=$!
python3 -m forklift_ros.isaac_slam_bridge --ros-args \
  -p socket_path:="$SOCKET" -p output_dir:="$OUT" -p "laser_xyz_yaw:=[-0.12,0.0,1.05,0.0]" -p persistent:=true \
  > "$OUT/bridge.log" 2>&1
STATUS=$?
timeout 300 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT/bundle/map'}" > "$OUT/serialize.log" 2>&1
echo "serialize exit $?" >> "$OUT/serialize.log"
kill $SLAM 2>/dev/null; wait $SLAM 2>/dev/null
INC=$(for d in /opt/ros/jazzy/include/*/; do printf -- '-I%s ' "$d"; done)
g++ -std=c++17 -O2 -I/opt/ros/jazzy/include $INC -I/usr/include/eigen3 /workspace/tools/slam_graph_dump.cpp \
  -o /tmp/build/slam_graph_dump -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib -lkartoSlamToolbox \
  -lboost_serialization -lboost_system -lboost_thread -ltbb > "$OUT/dump_build.log" 2>&1 || exit 4
read -r RES PASS THRESH < <(python3 - "$PARAMS" <<'PY'
import sys, yaml
p = yaml.safe_load(open(sys.argv[1]))["slam_toolbox"]["ros__parameters"]
print(p["resolution"], p["min_pass_through"], p["occupancy_threshold"])
PY
)
/tmp/build/slam_graph_dump "$OUT/bundle/map" "$OUT/dump" "$RES" "$PASS" "$THRESH" > "$OUT/dump.log" 2>&1 || exit 5
exit $STATUS
