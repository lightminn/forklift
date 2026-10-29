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
#   optional: PORT (8798), FACTOR (0.4), SEED (0), ENCODER_MODEL (team_spec_quantized, or reported; each takes its own
#   obstacle-layer age table), EXTRA_ARGS (split on spaces), STANDBY_LINGER_S (300), MAP_BUNDLE (the SLAM mode,
#   deploy/console/slam_choice.sh -- unset, the default since the user's decision of 2026-10-27 (live-default plan 5d):
#   localization on the accepted survey bundle of SCENE_FILE and ENCODER_MODEL from config/slam_map_bundles.yaml under
#   SLAM_BUNDLE_ROOT, its map id held against the folder, and online mapping as a recorded fallback where the registry
#   has no row; registry: that bundle or no launch; none: mapping; a folder: that bundle with MAP_START_POSE "x,y,yaw",
#   map <- base_link at the scene's start), SLAM_BUNDLE_ROOT (where the registry's bundle folders are),
#   SESSION_ID / BRIDGE_CPU (set by tools/isaac_bundle.py when several sessions share the job),
#   D7_HOLD (on: --d7-reapproach-hold, the SLAM corrections held over a D7 re-approach until its retry match -- its
#   registered V5 batch passed, docs/validation/2026-10-25-reapproach-slam-hold.md; off as before; an EXTRA_ARGS
#   --d7-reapproach-hold comes later and wins),
#   FAR_EXPECTED (on: --far-capture-expected on, the far capture without far recognition looks only in the mission's
#   adoption rectangle, so a neighbouring pallet is not adopted -- the lead's exception decision of 2026-10-28 after
#   docs/validation/2026-10-28-far-target-binding.md, which missed its pre-registered "no false refusal"; inert with
#   --recognition far; off as before; an EXTRA_ARGS --far-capture-expected comes later and wins; live records made
#   before this default -- CT-1, D7 and console baselines -- ran with it off: do not mix them with later ones),
#   START_POSE (B-P2 start relocalization, docs/plans/2026-10-29-start-relocalization.md: "x,y,yaw" world base_link --
#   the truck is placed there and the session finds itself on the survey map before its SLAM link opens: the runner
#   gets --start-pose/--start-relocalize, the supervisor --relocalize, the bridge the runner's estimate as its
#   map_start_pose instead of MAP_START_POSE; needs a survey map; the console shows the relocalization from the
#   start and takes the operator's pick or retry (B-P2c); RELOC_UNATTENDED=1: a start needing the operator ends the
#   session instead (unattended batches, B-P2d); RELOC_WAIT_S (600): the supervisor's whole wait, the operator's
#   time 60 s less), MAX_EPOCHS (unset: epochs until the session ends; 1 for a validation session that must not be
#   retried, B-P2d),
#   SCENE_FILE (a tools/riibotics_isaac_scenes.py scene: --layout scene on its layout as the prior map -- no
#   --grid-planning, --min-obstacle-height-m or --prism-colliders -- instead of the factory hall)
#   The far recognition (docs/plans/2026-10-22-live-far-recognition.md) goes in EXTRA_ARGS: --recognition far and,
#   for the learned proposals, the model and interpreter of that host -- then submit with --gres=shard:2: the
#   session's Isaac (3.4 GB) and the learned worker (0.5 GB) peaked at 3.9 GB of a 4 GB shard (2026-10-22), too
#   close for a GPU shared with other jobs (a worker's warm-up ran out of GPU memory once).
#
# Run from the repository root. Open the console through   ssh -N -L 8798:127.0.0.1:$PORT <host>.
set -u
OUT=$(realpath -m "${1:?output directory}")
ROOT=$(pwd)
: "${BASE_SCENE:?the base scene (.usda)}" "${FORKLIFT_URDF:?the forklift URDF the scene was built from}"
: "${VENV_PY:?the Isaac Python}"
# In a bundle (deploy/slurm/isaac_bundle.sbatch) several sessions share one Slurm job: SESSION_ID names this one's
# bridge containers and BRIDGE_CPU pins its bridge to a CPU of its own; alone, the job ID and the job's first CPU.
# The SLAM odometry's encoders (plan 2026-10-25-team-spec-default, lead decision): the joint-angle model by default,
# reported on request (its own baselines; never mixed with the default's), nothing else (no other age table).
ENCODER_MODEL=${ENCODER_MODEL:-team_spec_quantized}
case "$ENCODER_MODEL" in reported|team_spec_quantized) ;; *) echo "ENCODER_MODEL must be reported or team_spec_quantized"; exit 2;; esac
D7_HOLD=${D7_HOLD:-on}
case "$D7_HOLD" in on|off) ;; *) echo "D7_HOLD must be on or off"; exit 2;; esac
FAR_EXPECTED=${FAR_EXPECTED:-on}
case "$FAR_EXPECTED" in on|off) ;; *) echo "FAR_EXPECTED must be on or off"; exit 2;; esac
# The SLAM mode (live-default plan S3 D2, 5d): MAP_BUNDLE unset (the registry, mapping as a recorded fallback) / none /
# registry / a folder, deploy/console/slam_choice.sh; the choice goes to the supervisor's log (--slam-choice)
. "$ROOT/deploy/console/slam_choice.sh"
if [ -n "${MAP_BUNDLE:-}" ]; then
  # a bundle folder that is not there stops the launch (an empty realpath would have read as mapping)
  MAP_BUNDLE=$(realpath -e "$MAP_BUNDLE") || { echo "no SLAM bundle folder ${MAP_BUNDLE}" >&2; exit 2; }
  : "${MAP_START_POSE:?the start pose x,y,yaw the bridge tells slam_toolbox (map <- base_link)}"
fi
[[ -z "${MAX_EPOCHS:-}" || "$MAX_EPOCHS" =~ ^[1-9][0-9]*$ ]] || { echo "MAX_EPOCHS must be a whole number >= 1"; exit 2; }
RELOC_WAIT_S=${RELOC_WAIT_S:-600}
case "$RELOC_WAIT_S" in ''|*[!0-9]*) echo "RELOC_WAIT_S must be whole seconds"; exit 2;; esac
[ "$RELOC_WAIT_S" -gt 60 ] || { echo "RELOC_WAIT_S must be over 60 s (the operator's time is 60 s less)"; exit 2; }
if [ -n "${START_POSE:-}" ] && [ -z "${MAP_BUNDLE:-}" ]; then
  echo "START_POSE relocalizes on a survey map: it needs one (slam: $SLAM_CHOICE)" >&2
  exit 2
fi
echo "slam: $SLAM_CHOICE"
JOB=${SESSION_ID:-${SLURM_JOB_ID:-local$$}}
mkdir -p "$OUT"
export OMNI_KIT_ACCEPT_EULA=YES OMP_NUM_THREADS=2 PYTHONPATH=$ROOT/src
CPU0=${BRIDGE_CPU:-$(taskset -cp $$ | awk -F': ' '{print $2}' | cut -d, -f1 | cut -d- -f1)}
# Every value the epoch scripts use, quoted for the shell once here; the scripts below expand nothing at writing.
{
  printf 'ROOT=%q\nBASE_SCENE=%q\nFORKLIFT_URDF=%q\nVENV_PY=%q\nJOB=%q\nCPU0=%q\n' \
    "$ROOT" "$BASE_SCENE" "$FORKLIFT_URDF" "$VENV_PY" "$JOB" "$CPU0"
  printf 'PORT=%q\nFACTOR=%q\nSEED=%q\nEXTRA_ARGS=%q\n' "${PORT:-8798}" "${FACTOR:-0.4}" "${SEED:-0}" "${EXTRA_ARGS:-}"
  printf 'MAP_BUNDLE=%q\nMAP_START_POSE=%q\n' "${MAP_BUNDLE:-}" "${MAP_START_POSE:-}"
  printf 'SCENE_FILE=%q\n' "$([ -n "${SCENE_FILE:-}" ] && realpath "$SCENE_FILE")"
  printf 'ENCODER_MODEL=%q\n' "$ENCODER_MODEL"
  printf 'D7_HOLD=%q\n' "$D7_HOLD"
  printf 'FAR_EXPECTED=%q\n' "$FAR_EXPECTED"
  printf 'START_POSE=%q\n' "${START_POSE:-}"  # B-P2: the placed start (empty: the scene's, known)
  printf 'RELOC_UNATTENDED=%q\nRELOC_WAIT_S=%q\n' "${RELOC_UNATTENDED:-}" "$RELOC_WAIT_S"  # B-P2c
  printf 'SLAM_CHOICE=%q\n' "$SLAM_CHOICE"  # the runner shows it on the console (live-default plan 5d)
  printf 'REQUIREMENTS_PROFILE=%q\nSCENE_REQUIREMENTS_OVERRIDE=%q\n' "${REQUIREMENTS_PROFILE:-}" \
    "${SCENE_REQUIREMENTS_OVERRIDE:-}"  # scene requirements plan
} > "$OUT/env.sh"

cat > "$OUT/bridge.sh" <<'EOT'
#!/bin/bash
# epoch $1, its directory $2: the SLAM bridge takes one connection, so each epoch has its own; with MAP_BUNDLE the
# session's map id $3 and bundle $4 (the supervisor's) -- the bundle mounted read-only, localization (plan L4); with
# START_POSE the runner's estimate $5 (B-P2: the supervisor's {map_start_pose}) is the bridge's start pose
. "$(dirname "$0")/env.sh"
mkdir -p "$2/bridge" "$2/sock"
LOC=()
START=$MAP_START_POSE
[ -n "${START_POSE:-}" ] && START=${5:?the estimate x,y,yaw from the runner}
[ -n "${MAP_BUNDLE:-}" ] && LOC=(-v "$4:/bundle:ro" -e SLAM_MAP_BUNDLE=/bundle -e SLAM_EXPECTED_MAP_ID="$3"
                                 -e SLAM_MAP_START_POSE="$START")
docker run -d --name "slam_bridge_${JOB}_e$1" --network none --cpuset-cpus "$CPU0" --memory 4g --group-add 2000 \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$ROOT:/workspace:ro" -v "$2/bridge:/out" -v "$2/sock:/run/slam" "${LOC[@]}" \
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
# epoch $1, its directory $2, the port file $3, the session's map id $4 and survey bundle $5 (with MAP_BUNDLE)
. "$(dirname "$0")/env.sh"
cd "$ROOT"
LOC=()  # SLAM localization plan L7: localize on the survey bundle from the start (unset: online mapping)
[ -n "${MAP_BUNDLE:-}" ] && LOC=(--slam-map-bundle "$5" --expected-map-id "$4")
LAYOUT=(--min-obstacle-height-m 1.15 --prism-colliders --layout factory --grid-planning)
[ -n "${SCENE_FILE:-}" ] && LAYOUT=(--layout scene --scene-file "$SCENE_FILE")
# the SLAM odometry's encoders and the obstacle layer's age table drawn from the same model (the runner refuses a
# reported table for a team-spec model; PR #108's team-spec table)
LAYER=config/obstacle_layer_single.yaml
[ "$ENCODER_MODEL" = reported ] || LAYER=config/obstacle_layer_single_team_spec.yaml
RELOC=()  # B-P2b: placed at START_POSE, the start found on the survey map; the epoch folder holds the estimate
# one word: a pose with a negative x would read as an option (Codex B-P2b review P2)
[ -n "${START_POSE:-}" ] && RELOC=("--start-pose=$START_POSE" --start-relocalize --reloc-dir "$2"
                                  --reloc-operator-timeout-s "$((RELOC_WAIT_S - 60))")
[ -n "${START_POSE:-}" ] && [ "${RELOC_UNATTENDED:-}" = 1 ] && RELOC+=(--reloc-unattended)
REQ=()  # scene requirements plan: a profile to compare with and an override reason, both recorded by the runner
[ -n "${REQUIREMENTS_PROFILE:-}" ] && REQ+=(--requirements-profile "$REQUIREMENTS_PROFILE")
[ -n "${SCENE_REQUIREMENTS_OVERRIDE:-}" ] && REQ+=(--scene-requirements-override "$SCENE_REQUIREMENTS_OVERRIDE")
# shellcheck disable=SC2086 -- EXTRA_ARGS is a list of words by design
ARGS=(
  --base-scene "$BASE_SCENE" \
  --pallet-urdf sim/models/epal6_pallet/pallet.urdf --pallet-geometry config/pallet_geometry_epal6.yaml \
  --forklift-urdf "$FORKLIFT_URDF" --settings config/isaac_transport_measured.yaml \
  --delivery-straight-m 1.5 --alignment-straight-m 2.1 --withdrawal-m 0.55 --insertion-reserve-m 0.016 \
  "${LAYOUT[@]}" --fps 60 \
  --use-perception --pallet-prior config/pallet_prior_epal6.yaml --planning-target perception \
  --perception-mount carriage_low_measured --depth-quantize-mm 1 --record-slam --return-home \
  --obstacle-layer "$LAYER" --encoder-model "$ENCODER_MODEL" --obstacle-act --seed "$SEED" \
  --slam-feedback "$2/sock/slam.sock" --slam-map-dir "$2/bridge" --slam-noise-seed "$SEED" \
  --d7-planner --d7-docking --d7-reapproach-hold "$D7_HOLD" --far-capture-expected "$FAR_EXPECTED" \
  --live --live-supervised --live-port "$PORT" --live-port-file "$3" \
  --live-factor "$FACTOR" --live-web-root "$ROOT/console/web" --live-epoch "$1" --slam-choice "$SLAM_CHOICE" \
  $EXTRA_ARGS "${LOC[@]}" "${RELOC[@]}" "${REQ[@]}" \
  --output "$2/run")
if [ -n "${RUNNER_CHECK:-}" ]; then  # the launcher's preflight: these arguments, then the runner's own checks on them
  # an older runner would take --check-args for a Kit argument and start Isaac
  grep -q -- '"--check-args"' "$ROOT/sim/isaac/run_transport.py" \
    || { echo "preflight: $ROOT's runner has no --check-args: refused" >&2; exit 2; }
  echo RUNNER_ARGS
  printf '%s\n' "${ARGS[@]}"
  exec "$VENV_PY" "$ROOT/sim/isaac/run_transport.py" "${ARGS[@]}" --check-args
fi
exec "$VENV_PY" "$ROOT/sim/isaac/run_transport.py" "${ARGS[@]}" > "$2/run.log" 2>&1
EOT
chmod +x "$OUT/bridge.sh" "$OUT/stop.sh" "$OUT/runner.sh"
# Preflight (docs/plans/2026-10-27-ws1-preflight.md): the session's SLAM bundle as the supervisor will load it, and the
# runner's own checks on the arguments runner.sh builds -- before any container, supervisor or Isaac. DRY_RUN=1 ends
# here with their verdict (tools/isaac_bundle.py check); a real run goes on only when they pass, so a launch refused
# here ends the job in seconds instead of leaving the console probe waiting for a port file (lvram, d7oor 2026-10-27).
preflight() {
  local map_id="" sock="$OUT/epoch_999/sock/slam.sock"
  # an epoch's SLAM socket is a Unix socket: its path fits in 107 bytes or the runner dies at connect (AF_UNIX path
  # too long -- the lvram random_m refill, 11840, at 112)
  if [ "$(printf %s "$sock" | wc -c)" -gt 107 ]; then
    echo "preflight: the SLAM socket path $sock is $(printf %s "$sock" | wc -c) bytes (AF_UNIX allows 107): shorten OUT" >&2
    return 2
  fi
  if [ -n "${MAP_BUNDLE:-}" ]; then
    map_id=$("$VENV_PY" -c 'import sys; from pathlib import Path
from forklift_core.localization.slam_map import load_bundle
print(load_bundle(Path(sys.argv[1]), sys.argv[2] or None).map_id)' "$MAP_BUNDLE" "${EXPECTED_MAP_ID:-}") \
      || { echo "preflight: the SLAM bundle $MAP_BUNDLE does not load as map ${EXPECTED_MAP_ID:-(any)}" >&2; return 2; }
    echo "SLAM_BUNDLE $MAP_BUNDLE $map_id"
  fi
  echo "SLAM_CHOICE $SLAM_CHOICE"
  mkdir -p "$OUT/preflight"
  RUNNER_CHECK=1 bash "$OUT/runner.sh" 0 "$OUT/preflight" "$OUT/preflight/live_port.json" "$map_id" "${MAP_BUNDLE:-}"
}
if [ "${DRY_RUN:-0}" = 1 ]; then
  preflight
  exit $?
fi
if ! preflight > "$OUT/preflight.log" 2>&1; then
  echo "preflight: the launch is refused (its log follows)"
  cat "$OUT/preflight.log"
  exit 2
fi
json() { "$VENV_PY" -c 'import json, sys; print(json.dumps(sys.argv[1:]))' "$@"; }
trap 'for c in $(docker ps -aq --filter "name=slam_bridge_${JOB}_e"); do docker rm -f "$c" >/dev/null; done' EXIT
echo "host=$(hostname) job=$JOB out=$OUT"
SUP_LOC=()
[ -n "${MAP_BUNDLE:-}" ] && SUP_LOC=(--slam-map-bundle "$MAP_BUNDLE")
[ -n "${MAP_BUNDLE:-}" ] && [ -n "$EXPECTED_MAP_ID" ] && SUP_LOC+=(--expected-map-id "$EXPECTED_MAP_ID")
# B-P2: the runner first, the bridge at its estimate
BRIDGE=(bash "$OUT/bridge.sh" '{epoch}' '{epoch_dir}' '{expected_map_id}' '{map_bundle}')
[ -n "${START_POSE:-}" ] && SUP_LOC+=(--relocalize --reloc-wait-s "$RELOC_WAIT_S") && BRIDGE+=('{map_start_pose}')
# Between runners the supervisor serves the console's port itself (contract 6): the page and /api/v1/session, so a
# start check that fails before the runner's server opens is shown; after a failed end it stays up STANDBY_LINGER_S
# (300 s: the job holds its slot that long; 0 turns it off).
"$VENV_PY" "$ROOT/tools/live_supervisor.py" --out "$OUT" --port-file "$OUT/live_port.json" "${SUP_LOC[@]}" \
  ${MAX_EPOCHS:+--max-epochs "$MAX_EPOCHS"} \
  --slam-choice "$SLAM_CHOICE" \
  --standby-port "${PORT:-8798}" --web-root "$ROOT/console/web" --standby-linger-s "${STANDBY_LINGER_S:-300}" \
  --bridge "$(json "${BRIDGE[@]}")" \
  --bridge-stop "$(json bash "$OUT/stop.sh" '{epoch}')" \
  --bridge-ready "{epoch_dir}/bridge/bridge_ready" --exit-file "{epoch_dir}/run/exit_code" \
  --runner "$(json bash "$OUT/runner.sh" '{epoch}' '{epoch_dir}' '{port_file}' '{expected_map_id}' '{map_bundle}')" &
SUP=$!
# Progress watchdog (tools/stall_watch.py): when no epoch's run/live_log.jsonl has changed for STALL_MIN minutes
# (default 10, 0 off) the runner gets SIGUSR1 -- its stack goes to run.stall_stack.txt -- and the supervisor TERM, as
# on Slurm's TERM; $OUT/stall.jsonl says so. A live session never went 3.3 s without a record in 30 runs (first record
# 38-50 s after its start); 11119 (2026-10-26) sat silent for 34 min mid-transport until its time limit.
"$VENV_PY" "$ROOT/tools/stall_watch.py" --glob "$OUT/epoch_*/run/live_log.jsonl" --stall-min "${STALL_MIN:-10}" \
  --term-pid "$SUP" --marker "$OUT/stall.jsonl" &
trap 'kill -TERM "$SUP" 2>/dev/null' TERM INT  # Slurm's TERM: the supervisor ends the runner and the bridge
wait "$SUP"
rc=$?
if kill -0 "$SUP" 2>/dev/null; then  # the wait was cut by the trap: wait for the supervisor's own end
  wait "$SUP"
  rc=$?
fi
echo "supervisor exit $rc $(date -Is)"
exit "$rc"
