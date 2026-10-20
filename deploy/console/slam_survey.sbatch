#!/bin/bash
#SBATCH --job-name=slam_survey
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=01:30:00
#SBATCH --signal=B:TERM@120
# A SLAM survey epoch (docs/plans/2026-10-20-slam-localization-session.md L2, S2): one epoch of the console's live
# runner -- the runner command is the one deploy/console/isaac_live.sbatch writes, word for word (a test holds the two
# together) -- in mapping mode, as every live session runs now. The console probe drives the survey's missions (CYCLE)
# and ends the session. Once the runner has closed its socket (no scan sent or being processed), the bridge has
# slam_toolbox write its pose graph (tools/slam_bridge_container.sh, SLAM_SERIALIZE); a fresh container then reads
# that graph back with no scan sent (tools/slam_bundle_container.sh) and tools/slam_survey_bundle.py writes the bundle
# (graph, grid from the read-back, manifest, map_id) and the survey's checks to OUT/bundle and OUT/survey_report.json.
#
#   BASE_SCENE=<scene.usda> FORKLIFT_URDF=<forklift.urdf> VENV_PY=<python> sbatch deploy/console/slam_survey.sbatch OUT
#   optional: PORT (8799), FACTOR (0.4), SEED (0), EXTRA_ARGS (split on spaces), CYCLE (P-1:D-1,P-1:D-0: the
#   missions, pallet:destination), REFERENCE_RUN (a run directory of operational missions for the coverage check)
#
# Run from the repository root.
set -u
OUT=$(realpath -m "${1:?output directory}")
ROOT=$(pwd)
: "${BASE_SCENE:?the base scene (.usda)}" "${FORKLIFT_URDF:?the forklift URDF the scene was built from}"
: "${VENV_PY:?the Isaac Python}"
JOB=${SLURM_JOB_ID:-local$$}
mkdir -p "$OUT"
export OMNI_KIT_ACCEPT_EULA=YES OMP_NUM_THREADS=2 PYTHONPATH=$ROOT/src
CPU0=$(taskset -cp $$ | awk -F': ' '{print $2}' | cut -d, -f1 | cut -d- -f1)
{
  printf 'ROOT=%q\nBASE_SCENE=%q\nFORKLIFT_URDF=%q\nVENV_PY=%q\nJOB=%q\nCPU0=%q\n' \
    "$ROOT" "$BASE_SCENE" "$FORKLIFT_URDF" "$VENV_PY" "$JOB" "$CPU0"
  printf 'PORT=%q\nFACTOR=%q\nSEED=%q\nEXTRA_ARGS=%q\n' "${PORT:-8799}" "${FACTOR:-0.4}" "${SEED:-0}" "${EXTRA_ARGS:-}"
} > "$OUT/env.sh"

cat > "$OUT/bridge.sh" <<'EOT'
#!/bin/bash
# epoch $1, its directory $2: the console's bridge, writing slam_toolbox's pose graph to bridge/bundle/map at its end
. "$(dirname "$0")/env.sh"
mkdir -p "$2/bridge" "$2/sock"
docker run -d --name "slam_survey_${JOB}_e$1" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -e SLAM_SERIALIZE=/out/bundle/map \
  -v "$ROOT:/workspace:ro" -v "$2/bridge:/out" -v "$2/sock:/run/slam" \
  forklift/ros2-dev:jazzy bash /workspace/tools/slam_bridge_container.sh /run/slam/slam.sock /out "-0.12,0.0,1.05,0.0" persistent >/dev/null
EOT
cat > "$OUT/stop.sh" <<'EOT'
#!/bin/bash
. "$(dirname "$0")/env.sh"
if [ -n "$(docker ps -aq --filter "name=^slam_survey_${JOB}_e$1\$")" ]; then
  # the bridge ends by itself once the runner closed its socket; then the graph is written (serialize_map up to 300 s)
  timeout 360 docker wait "slam_survey_${JOB}_e$1" >/dev/null 2>&1
  docker rm -f "slam_survey_${JOB}_e$1" >/dev/null
fi
EOT
cat > "$OUT/runner.sh" <<'EOT'
#!/bin/bash
# epoch $1, its directory $2, the port file $3
. "$(dirname "$0")/env.sh"
cd "$ROOT"
# shellcheck disable=SC2086 -- EXTRA_ARGS is a list of words by design
exec "$VENV_PY" "$ROOT/sim/isaac/run_transport.py" \
  --base-scene "$BASE_SCENE" \
  --pallet-urdf sim/models/epal6_pallet/pallet.urdf --pallet-geometry config/pallet_geometry_epal6.yaml \
  --forklift-urdf "$FORKLIFT_URDF" --settings config/isaac_transport_measured.yaml \
  --delivery-straight-m 1.5 --alignment-straight-m 2.1 --withdrawal-m 0.55 --insertion-reserve-m 0.016 \
  --min-obstacle-height-m 1.15 --prism-colliders --layout factory --fps 60 \
  --use-perception --pallet-prior config/pallet_prior_epal6.yaml --planning-target perception \
  --perception-mount carriage_low_measured --depth-quantize-mm 1 --record-slam --return-home \
  --obstacle-layer config/obstacle_layer_single.yaml --obstacle-act --grid-planning --seed "$SEED" \
  --slam-feedback "$2/sock/slam.sock" --slam-map-dir "$2/bridge" --slam-noise-seed "$SEED" \
  --d7-planner --d7-docking --live --live-supervised --live-port "$PORT" --live-port-file "$3" \
  --live-factor "$FACTOR" --live-web-root "$ROOT/console/web" --live-epoch "$1" $EXTRA_ARGS \
  --output "$2/run" > "$2/run.log" 2>&1
EOT
chmod +x "$OUT/bridge.sh" "$OUT/stop.sh" "$OUT/runner.sh"
json() { "$VENV_PY" -c 'import json, sys; print(json.dumps(sys.argv[1:]))' "$@"; }
trap 'for c in $(docker ps -aq --filter "name=slam_(survey|readback)_${JOB}"); do docker rm -f "$c" >/dev/null; done' EXIT
echo "host=$(hostname) job=$JOB out=$OUT start $(date -Is)"
# one epoch: a survey that failed is made again, not continued in a new scene
"$VENV_PY" "$ROOT/tools/live_supervisor.py" --out "$OUT" --port-file "$OUT/live_port.json" --max-epochs 1 \
  --bridge "$(json bash "$OUT/bridge.sh" '{epoch}' '{epoch_dir}')" \
  --bridge-stop "$(json bash "$OUT/stop.sh" '{epoch}')" \
  --bridge-ready "{epoch_dir}/bridge/bridge_ready" --exit-file "{epoch_dir}/run/exit_code" \
  --runner "$(json bash "$OUT/runner.sh" '{epoch}' '{epoch_dir}' '{port_file}')" > "$OUT/supervisor.log" 2>&1 &
SUP=$!
trap 'kill -TERM "$SUP" 2>/dev/null' TERM INT
"$VENV_PY" "$ROOT/tools/live_console_probe.py" --port-file "$OUT/live_port.json" --report "$OUT/probe.json" \
  --scenario cycle --cycle "${CYCLE:-P-1:D-1,P-1:D-0}" --end-session > "$OUT/probe.log" 2>&1
echo "probe exit $? $(date -Is)"
wait "$SUP"
rc=$?
if kill -0 "$SUP" 2>/dev/null; then
  wait "$SUP"
  rc=$?
fi
echo "supervisor exit $rc $(date -Is)"

# Read the saved graph back in a fresh container: no scan, the /map it makes from the graph alone is the grid.
EP=$OUT/epoch_1
mkdir -p "$OUT/readback"
LOC=$OUT/readback/localization.yaml
cp "$ROOT/ros2/src/forklift_bringup/config/slam_toolbox_isaac_online.yaml" "$LOC"
sed -i "s/^    mode: .*/    mode: localization\n    map_file_name: \/bundle\/map\n    map_start_pose: [0.0, 0.0, 0.0]/" "$LOC"
if [ -f "$EP/bridge/bundle/map.posegraph" ]; then
  docker run --rm --name "slam_readback_${JOB}" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 \
    -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$ROOT:/workspace:ro" -v "$EP/bridge/bundle:/bundle:ro" \
    -v "$LOC:/params.yaml:ro" -v "$OUT/readback:/out" \
    forklift/ros2-dev:jazzy bash /workspace/tools/slam_bundle_container.sh /bundle/map /out /params.yaml
  echo "readback exit $? $(date -Is)"
fi
# what can move in the survey's stage (the truck and the carried pallet only, or recorded)
[ -f "$EP/run/scene.usda" ] && "$VENV_PY" "$ROOT/tools/list_rigid_bodies.py" "$EP/run/scene.usda" "$OUT/rigid_bodies.json" \
  > "$OUT/rigid_bodies.log" 2>&1
"$VENV_PY" "$ROOT/tools/slam_survey_bundle.py" --epoch "$EP" --readback "$OUT/readback" \
  --rigid-bodies "$OUT/rigid_bodies.json" --slam-params "$ROOT/ros2/src/forklift_bringup/config/slam_toolbox_isaac_online.yaml" \
  --image "$(docker image inspect -f '{{.Id}}' forklift/ros2-dev:jazzy)" \
  ${REFERENCE_RUN:+--reference-run "$REFERENCE_RUN"} --out "$OUT/bundle" --report "$OUT/survey_report.json"
brc=$?
echo "bundle exit $brc $(date -Is)"
[ "$rc" -eq 0 ] && exit "$brc"
exit "$rc"
