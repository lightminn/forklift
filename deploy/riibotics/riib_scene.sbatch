#!/bin/bash
#SBATCH --job-name=riib_scene
#SBATCH --partition=compute
#SBATCH --gres=shard:2
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=02:00:00
# One Riibotics demo scene (tools/riibotics_isaac_scenes.py) as an Isaac autonomous mission (--layout scene: the
# scene's layout is the planner's prior map). Defaults (user decision 2026-10-22, deploy/riibotics/README.md):
#   MODE=slam      far recognition (learned proposals), SLAM feedback through the slam_toolbox bridge, D7 docking
#   NEAR=on        PR #88's near re-capture: --near-capture-expected on, --near-capture-lower waive_far_confirmed,
#                  --near-capture-redraw always, --near-capture-redraw-lateral-m 0.10 (slam mode only: they need the
#                  perception planning target)
#   MARKERS=on     line_* / random_* run their slot-marker copy <scene>_m.json (PR #99); the other scenes have none
#   RACK=on        aisle / aisle_out run their rack copy <scene>_rack.json: rack uprights, wider slot gaps and the
#                  0.20 m back gap (user decision 2026-10-22, docs/plans/2026-10-22-riibotics-aisle.md D-A, D-C)
#   KEEPOUT=0      --lidar-keepout-m: the planner's searches keep the LiDAR-visible props this much wider (P1c,
#                  docs/plans/2026-10-23-planner-permission-clearance.md); 0 plans as before. 0.15 by default for the
#                  aisle's rack copy only (aisle_rack, release off, within the same batches: transports passed 0/9 at 0
#                  against 7/14 at 0.15, docs/validation/2026-10-24-start-band-release.md); every other scene 0 (trailer:
#                  no gain shown). KEEPOUT=<metres> overrides it for any scene
#   NOISE=const    the obstacle layer config/obstacle_layer_single.yaml (sigma 0.02, cut 0.06 at every range). NOISE=a2
#                  runs config/obstacle_layer_single_a2.yaml: LiDAR range noise and the grid's range bound from Slamtec's A2
#                  spec rows (P3a, same plan). It also feeds SLAM and the scan docking (shared scan), whose acceptance under
#                  it is not settled -- const by default (user decision 2026-10-24, docs/validation/2026-10-24-p3a-a2-range-bound.md)
#   START_BAND=off --start-band-release: an occupied cell the stopped truck already stands within the permission's own
#                  start inflation of does not block moving away from it (docs/plans/2026-10-24-start-band-release.md);
#                  off: no gain on the trailer and one actual load overlap in the rack aisle with it on
#                  (docs/validation/2026-10-24-start-band-release.md)
#   RESTART_CONFIRM=0 --restart-confirm-scans: after an occupied stop, this many new scans in a row must allow a
#                  restart (docs/plans/2026-10-25-restart-confirmation.md); 0 (default) as before, until the Isaac comparison
#   LOW_CLEARANCE=on --low-clearance-guard: the props below the LiDAR plane watched along the stop arc of the
#                  estimated pose, the speed capped before the planned clearance is used up
#                  (docs/plans/2026-10-25-low-clearance-guard.md); on: no low-prop overlap and no fewer completions
#                  in the on/off comparison (docs/validation/2026-10-25-low-clearance-guard.md); LOW_CLEARANCE=off as before
# Regression runs choose the older combinations explicitly: MODE=truth (truck and pallet poses as truth, no camera,
# no SLAM -- the fleet demo setting), NEAR=off, MARKERS=off, RACK=off, KEEPOUT=0. Anything else goes through EXTRA_ARGS.
# shard:2 -- Isaac with the learned proposal worker peaks at 3.9 GB of a 4 GB shard (PR #98); a MODE=truth run may ask
# for one with sbatch --gres=shard:1.
# Several runs in one job (deploy/slurm/isaac_bundle.sbatch, docs/plans/2026-10-24-ws1-isaac-bundle.md): the bundle runner
# sets SESSION_ID (this run's bridge container name) and BRIDGE_CPU (its cpuset); sizes riib = 3 runs per GPU (a run
# with its video holds 4.15-4.47 GB). The runner goes in the background so that a TERM reaches the trap at once.
# DRY_RUN=1 prints the runner's arguments, one per line, and starts nothing.
#
# Usage (paths of this machine come from the caller, nothing host-specific is kept here):
#   SNAP=<code checkout> SCENE_DIR=<scene files> SCENE=<name> BASE_SCENE=<scene.usda> FORKLIFT_URDF=<urdf> \
#   VENV_PY=<python> [ONNX=<proposal weights> LEARNED_PY=<python with onnxruntime> BRIDGE_IMAGE=<ros2 image>] \
#   [MODE=slam|truth] [NEAR=on|off] [MARKERS=on|off] [RACK=on|off] [KEEPOUT=0] [NOISE=const|a2] [START_BAND=off|on] [SEED=0] [MAX_SIM=1500] [EXTRA_ARGS=...] \
#   sbatch --output=<dir>/riib-%j.log deploy/riibotics/riib_scene.sbatch OUT
set -u
OUT=${1:?the run directory}
: "${SNAP:?the code checkout to run}" "${SCENE_DIR:?the scene files}" "${SCENE:?the scene name}"
# Every path absolute before the cd into SNAP below (a relative one would name another directory there).
OUT=$(realpath -m "$OUT"); SNAP=$(realpath -m "$SNAP"); SCENE_DIR=$(realpath -m "$SCENE_DIR")
: "${BASE_SCENE:?the base scene (.usda)}" "${FORKLIFT_URDF:?the forklift URDF the scene was built from}"
: "${VENV_PY:?the Isaac python}"
MODE=${MODE:-slam}; NEAR=${NEAR:-on}; MARKERS=${MARKERS:-on}; RACK=${RACK:-on}; NOISE=${NOISE:-const}; START_BAND=${START_BAND:-off}; LOW_CLEARANCE=${LOW_CLEARANCE:-on}; RESTART_CONFIRM=${RESTART_CONFIRM:-0}
case "$MODE" in slam|truth) ;; *) echo "MODE must be slam or truth"; exit 2;; esac
case "$NEAR" in on|off) ;; *) echo "NEAR must be on or off"; exit 2;; esac
case "$MARKERS" in on|off) ;; *) echo "MARKERS must be on or off"; exit 2;; esac
case "$RACK" in on|off) ;; *) echo "RACK must be on or off"; exit 2;; esac
case "$NOISE" in a2) LAYER=config/obstacle_layer_single_a2.yaml;; const) LAYER=config/obstacle_layer_single.yaml;;
  *) echo "NOISE must be a2 or const"; exit 2;; esac
case "$START_BAND" in on|off) ;; *) echo "START_BAND must be on or off"; exit 2;; esac
case "$LOW_CLEARANCE" in on|off) ;; *) echo "LOW_CLEARANCE must be on or off"; exit 2;; esac
[[ "$RESTART_CONFIRM" =~ ^[0-9]+$ ]] || { echo "RESTART_CONFIRM must be 0 (off) or a number of scans"; exit 2; }
NAME=$SCENE
case "$SCENE" in
  *_m) [ "$MARKERS" = off ] && { echo "SCENE=$SCENE is a marker copy but MARKERS=off"; exit 2; } ;;
  line_*|random_*) [ "$MARKERS" = on ] && NAME=${SCENE}_m ;;
  *_rack) [ "$RACK" = off ] && { echo "SCENE=$SCENE is a rack copy but RACK=off"; exit 2; } ;;
  aisle|aisle_out) [ "$RACK" = on ] && NAME=${SCENE}_rack ;;
esac
# the aisle's rack copy keeps its uprights 0.15 m wider in the searches; every other scene plans as before
[ "$NAME" = aisle_rack ] && KEEPOUT=${KEEPOUT:-0.15}
KEEPOUT=${KEEPOUT:-0}
[[ "$KEEPOUT" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "KEEPOUT must be a non-negative number of metres"; exit 2; }
SCENE_FILE=$SCENE_DIR/$NAME.json
[ -f "$SCENE_FILE" ] || { echo "no scene file $SCENE_FILE (tools/riibotics_isaac_scenes.py --slot-markers / --rack write the _m / _rack copies)"; exit 2; }
export OMNI_KIT_ACCEPT_EULA=YES OMP_NUM_THREADS=2
# A run directory is used once: an earlier run's bridge_ready would pass for this bridge's, its records would mix.
if [ "${DRY_RUN:-0}" != 1 ] && { [ -e "$OUT/run" ] || [ -e "$OUT/bridge/bridge_ready" ]; }; then
  echo "refusing: $OUT holds an earlier run"; exit 2
fi
mkdir -p "$OUT/bridge" "$OUT/sock"
export PYTHONPATH=$SNAP/src
cd "$SNAP"
echo "gpu ${CUDA_VISIBLE_DEVICES:-unset} job ${SLURM_JOB_ID:-} snapshot $SNAP scene $NAME mode $MODE near $NEAR markers $MARKERS rack $RACK keepout $KEEPOUT noise $NOISE start_band $START_BAND $(date -Is)"
printf 'MODE=%s NEAR=%s MARKERS=%s RACK=%s KEEPOUT=%s NOISE=%s START_BAND=%s LOW_CLEARANCE=%s RESTART_CONFIRM=%s SCENE_FILE=%s\n' "$MODE" "$NEAR" \
  "$MARKERS" "$RACK" "$KEEPOUT" "$NOISE" "$START_BAND" "$LOW_CLEARANCE" "$RESTART_CONFIRM" "$SCENE_FILE" > "$OUT/riib_defaults.txt"
read QE QT < <("$VENV_PY" -c "import json,sys;v=json.load(open(sys.argv[1]))['views'];print(','.join(map(str,v['quarter_eye'])),','.join(map(str,v['quarter_target'])))" "$SCENE_FILE")
COMMON=(--base-scene "$BASE_SCENE"
  --pallet-urdf sim/models/epal6_pallet/pallet.urdf --pallet-geometry config/pallet_geometry_epal6.yaml
  --forklift-urdf "$FORKLIFT_URDF" --settings config/isaac_transport_measured.yaml
  --delivery-straight-m 1.5 --alignment-straight-m 2.1 --withdrawal-m 0.55 --insertion-reserve-m 0.016
  --layout scene --scene-file "$SCENE_FILE" --fps 60 --max-sim-seconds "${MAX_SIM:-1500}"
  --record-slam --return-home --obstacle-layer "$LAYER" --obstacle-act --seed "${SEED:-0}"
  --lidar-keepout-m "$KEEPOUT" --start-band-release "$START_BAND" --low-clearance-guard "$LOW_CLEARANCE"
  --restart-confirm-scans "$RESTART_CONFIRM"
  --video --extra-views quarter,chase --quarter-eye="$QE" --quarter-target="$QT")
if [ "$MODE" = truth ]; then
  MODE_ARGS=(--d7-planner)
  [ "$NEAR" = on ] && echo "NEAR=on has no effect in MODE=truth (the near re-capture needs the perception target)" >&2
else
  : "${ONNX:?the learned proposal weights (MODE=slam)}" "${LEARNED_PY:?the python with onnxruntime (MODE=slam)}"
  if [ "${DRY_RUN:-0}" != 1 ]; then
  NAME_B=slam_bridge_${SESSION_ID:-${SLURM_JOB_ID:-$$}}_riib
  CPU0=${BRIDGE_CPU:-$(taskset -cp $$ | awk -F': ' '{print $2}' | cut -d, -f1 | cut -d- -f1)}
  docker run -d --name "$NAME_B" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 -u "$(id -u):$(id -g)" \
    -e HOME=/tmp -v "$SNAP":/workspace:ro -v "$OUT/bridge":/out -v "$OUT/sock":/run/slam \
    "${BRIDGE_IMAGE:-forklift/ros2-dev:jazzy}" bash /workspace/tools/slam_bridge_container.sh /run/slam/slam.sock /out "-0.12,0.0,1.05,0.0" >/dev/null \
    || { echo "bridge container did not start"; exit 1; }
  # The bridge saves its records after the runner disconnects: stop it with time to do so before removing it.
  trap 'docker stop -t "${BRIDGE_STOP_S:-60}" "$NAME_B" >/dev/null 2>&1; docker rm -f "$NAME_B" >/dev/null 2>&1' EXIT
  for i in $(seq 1 120); do [ -f "$OUT/bridge/bridge_ready" ] && break; sleep 1; done
  [ -f "$OUT/bridge/bridge_ready" ] || { echo "bridge not ready"; exit 1; }
  fi
  MODE_ARGS=(--use-perception --pallet-prior config/pallet_prior_epal6.yaml --planning-target perception
    --perception-mount carriage_low_measured --depth-quantize-mm 1
    --slam-feedback "$OUT/sock/slam.sock" --slam-map-dir "$OUT/bridge" --slam-noise-seed "${SEED:-0}"
    --d7-planner --d7-docking --recognition far --recognition-proposals learned
    --recognition-onnx "$ONNX" --recognition-learned-python "$LEARNED_PY" --recognition-tau 0.3 --recognition-conf 0.05
    --recognition-pixel-lag-steps 24 --recognition-providers CUDAExecutionProvider,CPUExecutionProvider
    --recognition-device-id 0 --recognition-threads 4)
  if [ "$NEAR" = on ]; then
    MODE_ARGS+=(--near-capture-expected on --near-capture-lower waive_far_confirmed
      --near-capture-redraw always --near-capture-redraw-lateral-m 0.10)
  fi
fi
if [ "${DRY_RUN:-0}" = 1 ]; then
  # shellcheck disable=SC2086
  echo RUNNER_ARGS
  printf '%s\n' "${COMMON[@]}" "${MODE_ARGS[@]}" ${EXTRA_ARGS:-}
  exit 0
fi
# shellcheck disable=SC2086  # EXTRA_ARGS is a word list on purpose
timeout "${RUN_TIMEOUT:-6600}" "$VENV_PY" "$SNAP/sim/isaac/run_transport.py" "${COMMON[@]}" "${MODE_ARGS[@]}" ${EXTRA_ARGS:-} \
  --output "$OUT/run" > "$OUT/run.log" 2>&1 &
RUNNER=$!
trap 'kill -TERM "$RUNNER" 2>/dev/null' TERM INT  # timeout passes the TERM on to the runner
wait "$RUNNER"
rc=$?
while kill -0 "$RUNNER" 2>/dev/null; do wait "$RUNNER"; rc=$?; done  # a trapped TERM cut the wait
echo "runner exit $rc $(date -Is)"
echo "OUT_OF_DEVICE lines: $(grep -c OUT_OF_DEVICE "$OUT/run.log")"
# The runner also records its own result code as JSON {"code": .., "reason": ..} (Isaac can leave the process with 0
# after a failure).
if [ "$rc" -eq 0 ] && [ -f "$OUT/run/exit_code" ]; then
  recorded=$("$VENV_PY" -c 'import json,sys; print(int(json.load(open(sys.argv[1])).get("code") or 0))' "$OUT/run/exit_code") \
    || recorded=1
  if [ "$recorded" != 0 ]; then rc=$recorded; echo "runner recorded exit code $rc"; fi
fi
exit "$rc"
