#!/bin/bash
# Inside forklift/ros2-dev:jazzy, after a survey epoch (SLAM localization plan docs/plans/2026-10-20-slam-localization-session.md
# L2, S2): read the saved pose graph back in a fresh container.
#   1. slam_graph_dump (built here) writes the graph's nodes (corrected base poses by scan stamp), its vertex and edge
#      counts and the grid karto makes from it;
#   2. slam_toolbox loads the graph in localization mode, no scan sent, and slam_map_grab keeps the /map it publishes
#      from that graph alone -- the bundle's grid.
# Usage: slam_bundle_container.sh <bundle prefix> <output_dir> <localization params>
set +u
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=/workspace/ros2/src/forklift_ros:/workspace/src:${PYTHONPATH}
PREFIX=$1; OUT=$2; PARAMS=$3
mkdir -p "$OUT/dump" /tmp/build
dpkg-query -W -f '${Version}\n' ros-jazzy-slam-toolbox > "$OUT/slam_toolbox_version.txt" 2>&1
INC=$(for d in /opt/ros/jazzy/include/*/; do printf -- '-I%s ' "$d"; done)
# shellcheck disable=SC2086 -- INC is a list of flags
g++ -std=c++17 -O2 -I/opt/ros/jazzy/include $INC -I/usr/include/eigen3 /workspace/tools/slam_graph_dump.cpp \
  -o /tmp/build/slam_graph_dump -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib -lkartoSlamToolbox \
  -lboost_serialization -lboost_system -lboost_thread -ltbb > "$OUT/dump_build.log" 2>&1 || exit 4
# the grid parameters slam_toolbox's /map uses (resolution, min_pass_through, occupancy_threshold)
read -r RES PASS THRESH < <(python3 - "$PARAMS" <<'EOF'
import sys, yaml
p = yaml.safe_load(open(sys.argv[1]))["slam_toolbox"]["ros__parameters"]
print(p["resolution"], p["min_pass_through"], p["occupancy_threshold"])
EOF
)
/tmp/build/slam_graph_dump "$PREFIX" "$OUT/dump" "$RES" "$PASS" "$THRESH" > "$OUT/dump.log" 2>&1 || exit 5
ros2 launch slam_toolbox localization_launch.py slam_params_file:="$PARAMS" use_sim_time:=true autostart:=true \
  > "$OUT/slam_toolbox.log" 2>&1 &
SLAM=$!
python3 -m forklift_ros.slam_map_grab --ros-args -p output_dir:="$OUT" -p timeout_s:=120.0 > "$OUT/map_grab.log" 2>&1
STATUS=$?
kill $SLAM 2>/dev/null
wait $SLAM 2>/dev/null
exit $STATUS
