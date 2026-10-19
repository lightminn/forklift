#!/bin/bash
#SBATCH --job-name=console_live
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=08:00:00
#SBATCH --signal=B:TERM@120
# A persistent operator console on the Isaac live runner (docs/plans/2026-10-18-persistent-live-console.md D9): the
# supervisor (tools/live_supervisor.py) runs one epoch after another -- its own SLAM bridge and runner -- until the
# operator ends the session (exit 0); a scene reset (exit 3) starts the next epoch at once, a failure after a
# backoff, three failures of one cause in a row stop it. The console keeps one port across epochs. Slurm's TERM
# (before the time limit) is passed to the supervisor, which ends the runner and its bridge and starts no more.
#
#   BASE_SCENE=<scene.usda> FORKLIFT_URDF=<forklift.urdf> VENV_PY=<python> sbatch deploy/console/isaac_live.sbatch OUT
#   optional: PORT (8798), FACTOR (0.4), SEED (0), EXTRA_ARGS (split on spaces)
#
# Run from the repository root. Open the console through   ssh -N -L 8798:127.0.0.1:$PORT <host>.
set -u
OUT=$(realpath -m "${1:?output directory}")
ROOT=$(pwd)
: "${BASE_SCENE:?the base scene (.usda)}" "${FORKLIFT_URDF:?the forklift URDF the scene was built from}"
: "${VENV_PY:?the Isaac Python}"
JOB=${SLURM_JOB_ID:-local$$}
mkdir -p "$OUT"
export OMNI_KIT_ACCEPT_EULA=YES OMP_NUM_THREADS=2 PYTHONPATH=$ROOT/src
CPU0=$(taskset -cp $$ | awk -F': ' '{print $2}' | cut -d, -f1 | cut -d- -f1)
# Every value the epoch scripts use, quoted for the shell once here; the scripts below expand nothing at writing.
{
  printf 'ROOT=%q\nBASE_SCENE=%q\nFORKLIFT_URDF=%q\nVENV_PY=%q\nJOB=%q\nCPU0=%q\n' \
    "$ROOT" "$BASE_SCENE" "$FORKLIFT_URDF" "$VENV_PY" "$JOB" "$CPU0"
  printf 'PORT=%q\nFACTOR=%q\nSEED=%q\nEXTRA_ARGS=%q\n' "${PORT:-8798}" "${FACTOR:-0.4}" "${SEED:-0}" "${EXTRA_ARGS:-}"
} > "$OUT/env.sh"

cat > "$OUT/bridge.sh" <<'EOT'
#!/bin/bash
# epoch $1, its directory $2: the SLAM bridge takes one connection, so each epoch has its own
. "$(dirname "$0")/env.sh"
mkdir -p "$2/bridge" "$2/sock"
docker run -d --name "slam_bridge_${JOB}_e$1" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$ROOT:/workspace:ro" -v "$2/bridge:/out" -v "$2/sock:/run/slam" \
  forklift/ros2-dev:jazzy bash /workspace/tools/slam_bridge_container.sh /run/slam/slam.sock /out "-0.12,0.0,1.05,0.0" persistent >/dev/null
EOT
cat > "$OUT/stop.sh" <<'EOT'
#!/bin/bash
. "$(dirname "$0")/env.sh"
# a missing container is no failure; a removal that fails is (the supervisor logs it)
if [ -n "$(docker ps -aq --filter "name=^slam_bridge_${JOB}_e$1\$")" ]; then
  # the bridge ends by itself once the runner closed its socket, writing its records and final status first
  # (persistent console plan D8b): give it that time, then remove the container either way
  timeout 45 docker wait "slam_bridge_${JOB}_e$1" >/dev/null 2>&1
  docker rm -f "slam_bridge_${JOB}_e$1" >/dev/null
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
trap 'for c in $(docker ps -aq --filter "name=slam_bridge_${JOB}_e"); do docker rm -f "$c" >/dev/null; done' EXIT
echo "host=$(hostname) job=$JOB out=$OUT"
"$VENV_PY" "$ROOT/tools/live_supervisor.py" --out "$OUT" --port-file "$OUT/live_port.json" \
  --bridge "$(json bash "$OUT/bridge.sh" '{epoch}' '{epoch_dir}')" \
  --bridge-stop "$(json bash "$OUT/stop.sh" '{epoch}')" \
  --bridge-ready "{epoch_dir}/bridge/bridge_ready" --exit-file "{epoch_dir}/run/exit_code" \
  --runner "$(json bash "$OUT/runner.sh" '{epoch}' '{epoch_dir}' '{port_file}')" &
SUP=$!
trap 'kill -TERM "$SUP" 2>/dev/null' TERM INT  # Slurm's TERM: the supervisor ends the runner and the bridge
wait "$SUP"
rc=$?
if kill -0 "$SUP" 2>/dev/null; then  # the wait was cut by the trap: wait for the supervisor's own end
  wait "$SUP"
  rc=$?
fi
echo "supervisor exit $rc $(date -Is)"
exit "$rc"
