#!/bin/bash
# Live riib-defaults validation (docs/plans/2026-11-08-live-riib-defaults-validation.md 3.1): a copy of
# test_runs/live/ct1_queue.sbatch for the bundle runner. The CT recognition settings are unchanged; the arm's runner
# arguments and the probe's options come from the run's env. Run from the snapshot root:
#   ARM=A|B|C2shadow|C2on [FIX=none|guard|boxed] SCENARIO=queue|cycle|main [CYCLE=..] [PROBE_EXTRA=..]
#   SCENE_FILE=.. SEED=.. FACTOR=.. PORT=.. bash riib_session.sh OUT
set -u
OUT=$(realpath -m "$1"); ROOT=$(pwd); mkdir -p "$OUT"
P=/home/projects/forklift
export VENV_PY=$P/.venv/bin/python
A=$P/artifacts/20261013_fork_spacing
export BASE_SCENE=$A/base_scene/scene.usda FORKLIFT_URDF=$A/dd_cfg/old/dls08_measured/forklift.urdf
export PORT=${PORT:-$((8820 + ${SLURM_JOB_ID:-0} % 100))} FACTOR=${FACTOR:?factor} SEED=${SEED:?seed}
export SCENE_FILE=${SCENE_FILE:?scene file}
export SLAM_BUNDLE_ROOT=${SLAM_BUNDLE_ROOT:-$P/artifacts/slam_bundles}
# 3.1: the registry's map (MAP_BUNDLE unset), team_spec, D7 hold, one epoch, the tick trace in every session
export ENCODER_MODEL=${ENCODER_MODEL:-team_spec_quantized} D7_HOLD=${D7_HOLD:-on} MAX_EPOCHS=${MAX_EPOCHS:-1}
export FORKLIFT_LIVE_TICK_TRACE=${FORKLIFT_LIVE_TICK_TRACE:-1}
FAR="--recognition far --recognition-proposals learned --recognition-onnx $P/test_runs/s3/yolo_runs/yolo11s-seg_p15/weights/best.onnx"
FAR="$FAR --recognition-learned-python $P/venvs/learned/bin/python --recognition-tau 0.3 --recognition-conf 0.05"
FAR="$FAR --recognition-pixel-lag-steps ${LAG:-12} --recognition-providers CUDAExecutionProvider,CPUExecutionProvider"
FAR="$FAR --recognition-device-id 0 --recognition-threads 4"
NEAR="--near-capture-redraw always --near-capture-redraw-lateral-m 0.10 --near-capture-expected on"
NEAR="$NEAR --near-capture-lower waive_far_confirmed"
case ${ARM:?A, B, C2shadow or C2on} in
  A) ARMX="";;  # the live launcher's current defaults
  B) ARMX="--low-clearance-guard on --return-start-backoff boxed --near-end-stand on";;  # A + 2 + 4 + 5
  C2shadow) ARMX="--near-end-stand shadow --inject-near-end-detour";;  # 2.3, 2 and 4 off
  C2on) ARMX="--near-end-stand on --inject-near-end-detour";;
  *) echo "ARM must be A, B, C2shadow or C2on"; exit 2;;
esac
case ${FIX:-none} in
  none) FIXX="";;
  guard) FIXX="--low-clearance-margin-m 0.30 --low-clearance-deviation-m 0.015";;  # 2.1
  boxed) FIXX="--withdrawal-m 0.40";;  # 2.2, after the launcher's 0.55 so it wins
  *) echo "FIX must be none, guard or boxed"; exit 2;;
esac
[ "${FIX:-none}" = none ] || [ "$ARM" = B ] || { echo "a fixture goes on arm B"; exit 2; }
RUNX=""; END=(--end-session)
case ${SCENARIO:?queue, cycle or main} in
  main) RUNX="--live-test-stop-in-capture"; PROBE=(--scenario main --expect-stop-in-capture); END=();;
  queue) PROBE=(--scenario queue --cycle "${CYCLE:?the order of the session}");;
  cycle) PROBE=(--scenario cycle --cycle "${CYCLE:?the order of the session}");;
  *) echo "SCENARIO must be queue, cycle or main"; exit 2;;
esac
read -ra PX <<< "${PROBE_EXTRA:-}"
PROBE+=("${PX[@]}")
export EXTRA_ARGS="$FAR $NEAR $RUNX $ARMX $FIXX"
# 3.2: every session's whole argument list kept in its result folder
{ env | grep -E '^(ARM|FIX|SCENARIO|CYCLE|PROBE_EXTRA|SCENE_FILE|SEED|FACTOR|PORT|ENCODER_MODEL|D7_HOLD|MAX_EPOCHS|FORKLIFT_LIVE_TICK_TRACE|START_RELOC|RELOC_UNATTENDED|SLAM_BUNDLE_ROOT|LAG|SESSION_ID|BUNDLE_[A-Z]+)=' | sort
  printf 'EXTRA_ARGS=%q\n' "$EXTRA_ARGS"; printf 'PROBE='; printf '%q ' "${PROBE[@]}" "${END[@]}"; echo; } > "$OUT/args.txt"
# DRY_RUN=1: no probe, no Isaac -- the launcher's own DRY_RUN (it prints RUNNER_ARGS and ARGS_OK, or exits 2) passes through
if [ "${DRY_RUN:-0}" = 1 ]; then
  grep -q "DRY_RUN" "$ROOT/deploy/console/isaac_live.sbatch" || { echo "DRY_RUN: this snapshot's isaac_live.sbatch has no DRY_RUN"; exit 2; }
  DRY_RUN=1 bash "$ROOT/deploy/console/isaac_live.sbatch" "$OUT/live"; exit $?
fi
echo "host=$(hostname) job=${SLURM_JOB_ID:-} out=$OUT root=$ROOT scene=$SCENE_FILE arm=$ARM fix=${FIX:-none} scenario=$SCENARIO $(date -Is)"
nvidia-smi --query-gpu=index,memory.used,memory.total --format=csv
bash "$ROOT/deploy/console/isaac_live.sbatch" "$OUT/live" > "$OUT/live.log" 2>&1 &
LIVE=$!
"$VENV_PY" /home/projects/forklift/test_runs/live/ct1_vram_sampler.py $$ "$OUT/vram.jsonl" 5 &
VRAM=$!
trap 'kill -TERM "$LIVE" 2>/dev/null' TERM INT
sleep 5
"$VENV_PY" "$ROOT/tools/live_console_probe.py" --port-file "$OUT/live/live_port.json" --report "$OUT/probe.json" \
  --launcher-pid "$LIVE" "${PROBE[@]}" "${END[@]}" --mission-timeout-s 2400 --observe-timeout-s 900 > "$OUT/probe.log" 2>&1 &
PROBE_PID=$!
wait "$PROBE_PID"; prc=$?
while kill -0 "$PROBE_PID" 2>/dev/null; do wait "$PROBE_PID"; prc=$?; done
echo "probe exit $prc $(date -Is)"
[ ${#END[@]} -eq 0 ] && kill -TERM "$LIVE" 2>/dev/null  # main: the probe does not end the session
for _ in $(seq 60); do kill -0 "$LIVE" 2>/dev/null || break; sleep 2; done
kill -TERM "$LIVE" 2>/dev/null
wait "$LIVE"; lrc=$?; echo "live exit $lrc $(date -Is)"
kill "$VRAM" 2>/dev/null
tail -5 "$OUT/live/supervisor.jsonl" 2>/dev/null
[ "$prc" -ne 0 ] && exit "$prc"
exit "$lrc"
