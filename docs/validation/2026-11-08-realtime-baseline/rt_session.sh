#!/bin/bash
# Real-time plan 5.13 (baseline re-measure, measurement only): one live console session on the factory hall driven by
# the console probe through one cycle mission (P-1:D-1, then the session ends), with the per-step trace on and a
# CPU/GPU sampler beside it. Run from a snapshot root, as a tools/isaac_bundle.py list entry:
#   LEVEL=off|video|views [FACTOR=1.0] bash .../rt_session.sh OUT
#   off:   no video (the live console's operating default)
#   video: --video (the overview video, no extra views)
#   views: --video --extra-views quarter,chase (the riib launcher's default views)
# DRY_RUN=1: this wrapper's checks, then isaac_live.sbatch's own DRY_RUN (it prints RUNNER_ARGS and ARGS_OK).
set -u
OUT=$(realpath -m "$1"); ROOT=$(pwd); mkdir -p "$OUT"
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
P=/home/projects/forklift
export VENV_PY=$P/.venv/bin/python
A=$P/artifacts/20261013_fork_spacing
export BASE_SCENE=$A/base_scene/scene.usda FORKLIFT_URDF=$A/dd_cfg/old/dls08_measured/forklift.urdf
export PORT=${PORT:-$((8840 + ${SLURM_JOB_ID:-0} % 100))} FACTOR=${FACTOR:-1.0} SEED=${SEED:-0}
case "${LEVEL:?LEVEL off|video|views}" in
  off) VID="" ;;
  video) VID="--video" ;;
  views) VID="--video --extra-views quarter,chase" ;;
  *) echo "LEVEL must be off, video or views" >&2; exit 2 ;;
esac
# the trace buffer is sized from --max-sim-seconds (the live loop itself is not bounded by it)
export EXTRA_ARGS="$VID --max-sim-seconds 1200 ${MORE:-}"
# every live-loop switch at its default (plan 5.13 P2-7): only the trace is set; one epoch, never restarted by the
# supervisor (a failed session is a missing run, not a retried one)
for v in $(env | grep -oE '^FORKLIFT_LIVE_[A-Z_]+' || true); do unset "$v"; done
export FORKLIFT_LIVE_TICK_TRACE=1 MAX_EPOCHS=1
if [ "${DRY_RUN:-0}" = 1 ]; then
  grep -q "DRY_RUN" "$ROOT/deploy/console/isaac_live.sbatch" || { echo "DRY_RUN: no DRY_RUN in isaac_live.sbatch"; exit 2; }
  # the launcher prints RUNNER_ARGS and ARGS_OK
  DRY_RUN=1 bash "$ROOT/deploy/console/isaac_live.sbatch" "$OUT/live"; exit $?
fi
echo "host=$(hostname) job=${SLURM_JOB_ID:-} session=${SESSION_ID:-} out=$OUT root=$ROOT level=$LEVEL factor=$FACTOR $(date -Is)"
printf 'LEVEL=%s\nFACTOR=%s\nSEED=%s\nEXTRA_ARGS=%s\nSESSION_ID=%s\nBRIDGE_CPU=%s\nSLURM_JOB_ID=%s\nCPUS=%s\n' \
  "$LEVEL" "$FACTOR" "$SEED" "$EXTRA_ARGS" "${SESSION_ID:-}" "${BRIDGE_CPU:-}" "${SLURM_JOB_ID:-}" \
  "$(taskset -cp $$ | awk -F': ' '{print $2}')" > "$OUT/rt_conditions.txt"
{ env | grep -E '^(FORKLIFT_|OMP_|CUDA_)' | sort
  sha256sum "$0" "$HERE/rt_monitor.py" "$BASE_SCENE" "$FORKLIFT_URDF" "$ROOT/sim/isaac/run_transport.py"
  scontrol show node "$(hostname)" | grep -E 'CfgTRES|AllocTRES'
  nvidia-smi --query-gpu=name,uuid,driver_version,clocks.max.sm,power.limit --format=csv,noheader
  docker image inspect -f '{{.Id}} {{.RepoTags}}' forklift/ros2-dev:jazzy 2>&1 | head -1
} >> "$OUT/rt_conditions.txt" 2>&1
squeue -h -o '%i %u %j %T %b %C' > "$OUT/squeue_start.txt" 2>&1
"$VENV_PY" "$HERE/rt_monitor.py" "$OUT/monitor" "slam_bridge_${SESSION_ID:-${SLURM_JOB_ID:-x}}_e1" &
MON=$!
bash "$ROOT/deploy/console/isaac_live.sbatch" "$OUT/live" > "$OUT/live.log" 2>&1 &
LIVE=$!
trap 'kill -TERM "$LIVE" 2>/dev/null' TERM INT
sleep 5
"$VENV_PY" "$ROOT/tools/live_console_probe.py" --port-file "$OUT/live/live_port.json" --report "$OUT/probe.json" \
  --launcher-pid "$LIVE" --scenario cycle --cycle P-1:D-1 --end-session \
  --mission-timeout-s 2400 --observe-timeout-s 900 > "$OUT/probe.log" 2>&1 &
PROBE_PID=$!
wait "$PROBE_PID"; prc=$?
while kill -0 "$PROBE_PID" 2>/dev/null; do wait "$PROBE_PID"; prc=$?; done
echo "probe exit $prc $(date -Is)"; echo "$prc" > "$OUT/probe_rc"
for _ in $(seq 60); do kill -0 "$LIVE" 2>/dev/null || break; sleep 2; done
kill -TERM "$LIVE" 2>/dev/null
wait "$LIVE"; lrc=$?; echo "live exit $lrc $(date -Is)"
squeue -h -o '%i %u %j %T %b %C' > "$OUT/squeue_end.txt" 2>&1
kill -TERM "$MON" 2>/dev/null; wait "$MON" 2>/dev/null
exit "$lrc"
