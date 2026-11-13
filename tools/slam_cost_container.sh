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
# A source-built slam_toolbox (deploy/ros2/Dockerfile.slam_patched: the unpatched U0 or a patch series) is installed
# over the apt one; SLAM_IMAGE_KIND=u0|patched says one must be there -- never a silent fall back to apt
# (docs/design/2026-11-13-slam-patch-build.md 2).
if [ -n "${SLAM_IMAGE_KIND:-}" ]; then
  [ -f /opt/slam_patched/install/setup.bash ] || { echo "SLAM_IMAGE_KIND=$SLAM_IMAGE_KIND without /opt/slam_patched" >&2; exit 7; }
  source /opt/slam_patched/install/setup.bash
fi
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
    # every value a float (tools/ros_float_list.py; S2 a_1 2026-11-13: "-2.612,0,0" kept the bridge from starting)
    POSE_LIST=$(python3 /workspace/tools/ros_float_list.py 3 "$SLAM_MAP_START_POSE" 2>"$OUT/pose_list.log") || exit 3
    LOC_ARGS=(-p localization_bundle:=/bundle -p expected_map_id:="$SLAM_EXPECTED_MAP_ID"
              -p "map_start_pose:=[$POSE_LIST]")
    # online-SLAM plan criterion 4: keep /map after the start check and store every map (H's map measurement)
    [ "${KEEP_MAP_SUBSCRIPTION:-0}" = 1 ] && LOC_ARGS+=(-p keep_map_subscription:=true)
    ros2 launch slam_toolbox localization_launch.py slam_params_file:="$OUT/localization_params.yaml" \
      use_sim_time:=true autostart:=true > "$OUT/slam_toolbox.log" 2>&1 & ;;
  *) echo "unknown node $NODE" > "$OUT/slam_toolbox.log"; exit 2 ;;
esac
SLAM=$!
python3 -m forklift_ros.isaac_slam_bridge --ros-args \
  -p socket_path:="$SOCKET" -p output_dir:="$OUT" -p "laser_xyz_yaw:=[-0.12,0.0,1.05,0.0]" -p persistent:=true \
  "${LOC_ARGS[@]}" > "$OUT/bridge.log" 2>&1
STATUS=$?
# the libraries the slam_toolbox node really loaded (overlay or apt), read before it is stopped
for p in $(pgrep -f "slam_toolbox_node|slam_toolbox/lib" 2>/dev/null); do
  grep -oE "/[^ ]*(kartoSlamToolbox|toolbox_common|ceres|solver_plugin|_slam_toolbox)[^ ]*\.so[^ ]*" "/proc/$p/maps" 2>/dev/null \
    | sort -u | sed "s|^|$p |"
done > "$OUT/slam_libs.txt"
# the slam_toolbox this run asked for: image kind, loaded library, identification line and switches (design v3 2)
python3 /workspace/tools/slam_patch_check.py --kind "${SLAM_IMAGE_KIND:-}" --libs "$OUT/slam_libs.txt" \
  --log "$OUT/slam_toolbox.log" --out "$OUT/patch_check.json" > "$OUT/patch_check.log" 2>&1 \
  || { [ "$STATUS" = 0 ] && STATUS=8; }
bash /workspace/tools/slam_cost_dump.sh "$OUT" "${5:-}" > "$OUT/dump_run.log" 2>&1
DUMP=$?
echo "dump exit $DUMP" >> "$OUT/dump_run.log"
# a requested graph dump that failed fails the run (it was only logged; design v2 4)
[ "${5:-}" = graph ] && [ "$DUMP" != 0 ] && [ "$STATUS" = 0 ] && STATUS=6
# The node itself gets SIGINT and is waited for (up to 30 s), so it exits through its destructors, where a trace writes
# its tail; only then is ros2 launch stopped. SIGTERM to ros2 launch alone ended the launcher at once (exit 143) and
# left the node to die with the container, so no trace was written (S1 equivalence p0t, job 18838).
NODE_PIDS=$(pgrep -f "/lib/slam_toolbox/[a-z_]*slam_toolbox_node" 2>/dev/null || true)
for p in $NODE_PIDS; do kill -INT "$p" 2>/dev/null; done
for _ in $(seq 1 120); do
  alive=0; for p in $NODE_PIDS; do kill -0 "$p" 2>/dev/null && alive=1; done
  [ "$alive" = 0 ] && break; sleep 0.25
done
[ "$alive" = 0 ] || echo "slam_toolbox node still running 30 s after SIGINT: $NODE_PIDS" >> "$OUT/exit_check.log"
kill $SLAM 2>/dev/null; wait $SLAM 2>/dev/null
# a patched node that ended itself (abort or refusal) fails the run even after a clean bridge, and a trace must end
# with its end record: one without it lost its tail (Codex code review 2 P2-5; docs/design/2026-11-13-slam-patch-build.md 2)
if grep -qE "forklift_slam_patch (abort|refused)" "$OUT/slam_toolbox.log"; then
  echo "slam_toolbox ended itself: $(grep -m1 -E 'forklift_slam_patch (abort|refused)' "$OUT/slam_toolbox.log")" \
    >> "$OUT/exit_check.log"
  [ "$STATUS" = 0 ] && STATUS=9
fi
if [ -n "${FORKLIFT_SLAM_TRACE:-}" ] && ! tail -n 1 "$FORKLIFT_SLAM_TRACE" 2>/dev/null | grep -q '"kind":"end"'; then
  echo "trace $FORKLIFT_SLAM_TRACE does not end with its end record" >> "$OUT/exit_check.log"
  [ "$STATUS" = 0 ] && STATUS=9
fi
exit $STATUS
