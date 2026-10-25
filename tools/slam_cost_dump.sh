#!/bin/bash
# Inside a running cost-replay container (docker exec, after the driver): the node's effective parameters, then, with
# a second argument "graph", the serialized pose graph dumped by tools/slam_graph_dump.cpp (nodes, edges, grid) for
# the slam-bounded-graph plan's connectivity check.
#   slam_cost_dump.sh <out> [graph]
set +u
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
OUT=$1
timeout 60 ros2 param dump /slam_toolbox > "$OUT/params_effective.yaml" 2> "$OUT/params_effective.err"
[ "${2:-}" = "graph" ] || exit 0
mkdir -p "$OUT/bundle" "$OUT/dump" /tmp/build
timeout 300 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT/bundle/map'}" \
  > "$OUT/serialize.log" 2>&1
echo "serialize exit $?" >> "$OUT/serialize.log"
INC=$(for d in /opt/ros/jazzy/include/*/; do printf -- '-I%s ' "$d"; done)
g++ -std=c++17 -O2 -I/opt/ros/jazzy/include $INC -I/usr/include/eigen3 /workspace/tools/slam_graph_dump.cpp \
  -o /tmp/build/slam_graph_dump -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib -lkartoSlamToolbox \
  -lboost_serialization -lboost_system -lboost_thread -ltbb > "$OUT/dump_build.log" 2>&1 || exit 4
read -r RES PASS THRESH < <(python3 - "$OUT/params_effective.yaml" <<'PY'
import sys, yaml
p = yaml.safe_load(open(sys.argv[1]))["/slam_toolbox"]["ros__parameters"]
print(p["resolution"], p["min_pass_through"], p["occupancy_threshold"])
PY
)
/tmp/build/slam_graph_dump "$OUT/bundle/map" "$OUT/dump" "$RES" "$PASS" "$THRESH" > "$OUT/dump.log" 2>&1 || exit 5
