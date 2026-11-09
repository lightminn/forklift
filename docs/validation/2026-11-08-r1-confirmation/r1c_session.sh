#!/bin/bash
# Real-time plan 5.17 R1 confirmation run: one single live console session for the bundle runner (--slots 1).
# Run from the snapshot root:
#   SET=far|off|mark|wall ARM=L|K|KP FACTOR=0.4|0.3 [SEED=0] bash r1c_session.sh OUT
#   far:  line_m (registry localization), the console v2 far recognition (learned) and near settings, the confirm probe
#   off:  line_m, no far recognition, the near settings without the far-confirmed lower waiver, the confirm probe
#   mark: the hall, no mission, the pixel marker and a camera freeze, the marker probe (the marker overrides the lift,
#         so it never shares a session with a mission, a lift or a STOP test -- plan 5.17)
#   wall: the hall, no mission, the approach probe (teleop to the hall wall)
#   L:  FORKLIFT_LIVE_CATCHUP_STEPS=0, the legacy camera phase (variable unset) -- the profile pinned explicitly
#   K:  catch-up 6, the legacy phase;  KP: catch-up 6 + FORKLIFT_LIVE_CAMERA_PHASE=6 (never with far: plan 5.17 scope)
# DRY_RUN=1: this wrapper's checks, then the launcher's own DRY_RUN (it prints RUNNER_ARGS and ARGS_OK).
set -u
OUT=$(realpath -m "$1"); ROOT=$(pwd); mkdir -p "$OUT"
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
P=/home/projects/forklift
export VENV_PY=$P/.venv/bin/python
A=$P/artifacts/20261013_fork_spacing
export BASE_SCENE=$A/base_scene/scene.usda FORKLIFT_URDF=$A/dd_cfg/old/dls08_measured/forklift.urdf
export PORT=${PORT:-$((8860 + ${SLURM_JOB_ID:-0} % 100))} FACTOR=${FACTOR:?factor} SEED=${SEED:-0}
export SLAM_BUNDLE_ROOT=${SLAM_BUNDLE_ROOT:-$P/artifacts/slam_bundles}
export ENCODER_MODEL=${ENCODER_MODEL:-team_spec_quantized} MAX_EPOCHS=1
LINE_M=$P/test_runs/riibotics/scenes_ct1_a770031/line_m_session.json
# every live-loop switch at its default, then the arm's profile set explicitly (plan 5.17: validation pins k and phase)
for v in $(env | grep -oE '^FORKLIFT_LIVE_[A-Z_]+' || true); do unset "$v"; done
export FORKLIFT_LIVE_TICK_TRACE=1
# Real-time plan 5.22 (measurement run): INSTR=on turns on the long-call record and the sampled LiDAR split (PR #379,
# after the variables are cleared above); unset or off keeps every earlier run's environment exactly.
case "${INSTR:-off}" in
  on) export FORKLIFT_LIVE_LONG_CALLS_MS=20 FORKLIFT_LIVE_LIDAR_PROFILE_EVERY=20 ;;
  off) ;;
  *) echo "INSTR must be on or off" >&2; exit 2 ;;
esac
# Real-time plan 5.24: LIFECYCLE=on keeps the start marks (runner start, live start; FORKLIFT_LIVE_LIFECYCLE) without
# the long-call record -- the first ok from the runner's start (M6) needs them
case "${LIFECYCLE:-off}" in
  on) export FORKLIFT_LIVE_LIFECYCLE=1 ;;
  off) ;;
  *) echo "LIFECYCLE must be on or off" >&2; exit 2 ;;
esac
case ${ARM:?L, K or KP} in
  L) export FORKLIFT_LIVE_CATCHUP_STEPS=0 ;;
  K) export FORKLIFT_LIVE_CATCHUP_STEPS=6 ;;
  KP) export FORKLIFT_LIVE_CATCHUP_STEPS=6 FORKLIFT_LIVE_CAMERA_PHASE=6 ;;
  *) echo "ARM must be L, K or KP" >&2; exit 2 ;;
esac
FAR="--recognition far --recognition-proposals learned --recognition-onnx $P/test_runs/s3/yolo_runs/yolo11s-seg_p15/weights/best.onnx"
FAR="$FAR --recognition-learned-python $P/venvs/learned/bin/python --recognition-tau 0.3 --recognition-conf 0.05"
FAR="$FAR --recognition-pixel-lag-steps 12 --recognition-providers CUDAExecutionProvider,CPUExecutionProvider"
FAR="$FAR --recognition-device-id 0 --recognition-threads 4"
NEAR="--near-capture-redraw always --near-capture-redraw-lateral-m 0.10 --near-capture-expected on"
# Real-time plan 5.24 (G5-0 arms): PROPOSALS=oracle swaps the learned proposer for the oracle masks (the semantic
# render path stays); PREP_RENDERS / PREP_WAIT_S add the test arms on the far branch. Unset: as before.
[ "${PROPOSALS:-learned}" = oracle ] && FAR="--recognition far --recognition-proposals oracle --recognition-pixel-lag-steps 12"
case "${PROPOSALS:-learned}" in learned|oracle) ;; *) echo "PROPOSALS must be learned or oracle" >&2; exit 2 ;; esac
PREP=""
[ -n "${PREP_RENDERS:-}" ] && PREP="--live-test-prep-renders $PREP_RENDERS"
[ -n "${PREP_WAIT_S:-}" ] && PREP="$PREP --live-test-prep-wait-s $PREP_WAIT_S"
# only the registered combinations (plan 5.17): far L/K at 0.4 and 0.3; off L/KP at 0.4; mark L/K at 0.4 and 0.3 and
# KP at 0.4; wall L/K/KP at 0.4
case "${SET:-}:${ARM}:${FACTOR}" in
  far:L:0.4|far:K:0.4|far:L:0.3|far:K:0.3|off:L:0.4|off:KP:0.4|mark:L:0.4|mark:K:0.4|mark:KP:0.4|mark:L:0.3|mark:K:0.3) ;;
  wall:L:0.4|wall:K:0.4|wall:KP:0.4) ;;
  *) echo "SET:ARM:FACTOR ${SET:-}:${ARM}:${FACTOR} is not in plan 5.17's registration" >&2; exit 2 ;;
esac
TRACE="--max-sim-seconds 1500"  # the trace buffer (plan 5.17 T6: dropped must be 0); the live loop is not bounded by it
END=(--end-session)
case ${SET:?far, off, mark or wall} in
  far)
    [ "$ARM" = KP ] && { echo "KP is never run with far recognition (plan 5.17 scope)" >&2; exit 2; }
    export SCENE_FILE=$LINE_M
    export EXTRA_ARGS="$FAR $NEAR --near-capture-lower waive_far_confirmed $TRACE${PREP:+ $PREP}"
    PROBE=(--scenario confirm --live-factor "$FACTOR" --mission-timeout-s 2400) ;;
  off)
    [ "$ARM" = K ] && { echo "the far-off branch compares L and KP (plan 5.17)" >&2; exit 2; }
    export SCENE_FILE=$LINE_M
    export EXTRA_ARGS="$NEAR $TRACE"
    PROBE=(--scenario confirm --live-factor "$FACTOR" --mission-timeout-s 2400) ;;
  mark)
    unset SCENE_FILE
    # 200 frames (about 20 sim s at the 10 Hz live camera) for the pixel lag, then TELEOP before the freeze at 30-60 sim s:
    # video-mode commands across it must be refused stale_video (Codex tool review: 300 frames reached the freeze's end)
    export EXTRA_ARGS="--live-test-marker --live-test-camera-freeze 30:60 $TRACE"
    PROBE=(--scenario marker --frames-dir "$OUT/frames" --frames 200 --freeze-from 30) ;;
  wall)
    unset SCENE_FILE
    export EXTRA_ARGS="--live-test-permit-compare $TRACE"
    PROBE=(--scenario approach) ;;
  *) echo "SET must be far, off, mark or wall" >&2; exit 2 ;;
esac
{ env | grep -E '^(SET|ARM|FACTOR|SEED|PORT|SCENE_FILE|ENCODER_MODEL|MAX_EPOCHS|SLAM_BUNDLE_ROOT|SESSION_ID|BRIDGE_CPU|FORKLIFT_[A-Z_]+|BUNDLE_[A-Z]+)=' | sort
  printf 'EXTRA_ARGS=%q\n' "$EXTRA_ARGS"; printf 'PROBE='; printf '%q ' "${PROBE[@]}" "${END[@]}"; echo
  sha256sum "$0" "$HERE/../2026-11-08-realtime-baseline/rt_monitor.py" "$BASE_SCENE" "$FORKLIFT_URDF" \
    "$ROOT/sim/isaac/run_transport.py" "$ROOT/tools/live_console_probe.py" ${SCENE_FILE:+"$SCENE_FILE"}
} > "$OUT/args.txt" 2>&1
if [ "${DRY_RUN:-0}" = 1 ]; then
  grep -q "DRY_RUN" "$ROOT/deploy/console/isaac_live.sbatch" || { echo "DRY_RUN: no DRY_RUN in isaac_live.sbatch"; exit 2; }
  # the launcher prints RUNNER_ARGS and ARGS_OK
  DRY_RUN=1 bash "$ROOT/deploy/console/isaac_live.sbatch" "$OUT/live"; exit $?
fi
echo "host=$(hostname) job=${SLURM_JOB_ID:-} out=$OUT root=$ROOT set=$SET arm=$ARM factor=$FACTOR $(date -Is)"
{ scontrol show node "$(hostname)" | grep -E 'CfgTRES|AllocTRES'; squeue -h -o '%i %u %j %T %b %C'; } > "$OUT/node_start.txt" 2>&1
"$VENV_PY" "$HERE/../2026-11-08-realtime-baseline/rt_monitor.py" "$OUT/monitor" "slam_bridge_${SESSION_ID:-${SLURM_JOB_ID:-x}}_e1" &
MON=$!
bash "$ROOT/deploy/console/isaac_live.sbatch" "$OUT/live" > "$OUT/live.log" 2>&1 &
LIVE=$!
trap 'kill -TERM "$LIVE" 2>/dev/null' TERM INT
sleep 5
"$VENV_PY" "$ROOT/tools/live_console_probe.py" --port-file "$OUT/live/live_port.json" --report "$OUT/probe.json" \
  --launcher-pid "$LIVE" "${PROBE[@]}" "${END[@]}" > "$OUT/probe.log" 2>&1 &
PROBE_PID=$!
wait "$PROBE_PID"; prc=$?
while kill -0 "$PROBE_PID" 2>/dev/null; do wait "$PROBE_PID"; prc=$?; done
echo "probe exit $prc $(date -Is)"; echo "$prc" > "$OUT/probe_rc"
# every set ends the session through the probe (--end-session); a launcher TERM loses the end summary, the tick
# trace and the spool (plan 5.17: job 16866 lost all eight hall sessions so). The wait and the TERM below only
# follow a probe that could not end it.
for _ in $(seq 60); do kill -0 "$LIVE" 2>/dev/null || break; sleep 2; done
kill -TERM "$LIVE" 2>/dev/null
wait "$LIVE"; lrc=$?; echo "live exit $lrc $(date -Is)"
squeue -h -o '%i %u %j %T %b %C' > "$OUT/squeue_end.txt" 2>&1
kill -TERM "$MON" 2>/dev/null; wait "$MON" 2>/dev/null
[ "$prc" -ne 0 ] && exit "$prc"
exit "$lrc"
