#!/bin/bash
#SBATCH --job-name=az_queue
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=7G
#SBATCH --time=01:15:00
# --time 01:15 (2026-10-26): completed 3-mission sessions took <= 30.5 min (39 bundle runs, single ct1 jobs <= 28.2 min);
# the probe gives each mission --mission-timeout-s 2400, so a slow but live session gets room. A session that stops
# ticking is not given more: 11119 stopped writing 23 min in and sat silent for 34 min until its limit.
#SBATCH --signal=B:TERM@120
# CT-1 PR 1 (docs/plans/2026-10-22-live-far-recognition.md section 4): the live console on a Riibotics scene with the
# far recognition (learned proposals, SLAM), driven by the console probe. Run from a snapshot root:
#   SCENE_FILE=<scene.json> SCENARIO=cycle|main|cancel|reject [LAG=12] [MORE=...] sbatch .../ct1_far.sbatch OUT
#   cycle:  P-1:D-1, P-1:D-0, P-1:D-1, then the session ends
#   main:   a STOP inside the first capture (the runner's test option), rearm and resume, teleop, resume to the end
#   cancel: the first mission cancelled 5 s into its observe leg, then P-1:D-1 submitted again
#   reject: the first mission looks 3 m off its pallet (recognition finds none, the mission fails), then P-1:D-1
set -u
OUT=$(realpath -m "$1"); ROOT=$(pwd); mkdir -p "$OUT"
P=/home/projects/forklift
export VENV_PY=$P/.venv/bin/python
A=$P/artifacts/20261013_fork_spacing
export BASE_SCENE=$A/base_scene/scene.usda FORKLIFT_URDF=$A/dd_cfg/old/dls08_measured/forklift.urdf
export PORT=${PORT:-$((8820 + ${SLURM_JOB_ID:-0} % 100))} FACTOR=${FACTOR:-0.4} SEED=${SEED:-0}
export SCENE_FILE=${SCENE_FILE:?scene file}
# live-default plan 5d (PR #167): a registered scene localizes on its survey bundle; MAP_BUNDLE=none maps
export SLAM_BUNDLE_ROOT=${SLAM_BUNDLE_ROOT:-$P/artifacts/slam_bundles}
FAR="--recognition far --recognition-proposals learned --recognition-onnx $P/test_runs/s3/yolo_runs/yolo11s-seg_p15/weights/best.onnx"
FAR="$FAR --recognition-learned-python $P/venvs/learned/bin/python --recognition-tau 0.3 --recognition-conf 0.05"
FAR="$FAR --recognition-pixel-lag-steps ${LAG:-12} --recognition-providers CUDAExecutionProvider,CPUExecutionProvider"
FAR="$FAR --recognition-device-id 0 --recognition-threads 4"
NEAR="--near-capture-redraw always --near-capture-redraw-lateral-m 0.10 --near-capture-expected on"
NEAR="$NEAR --near-capture-lower waive_far_confirmed"
# docs/plans/2026-10-28-adoption-zone-check.md 8a: RECOG=off -- no far recognition; the near capture keeps what a session
# scene requires (the redraw) and the expected site, without the far-confirmed lower waiver; RECOG=far (default) keeps
# ct1_queue's far recognition and near capture as they are
if [ "${RECOG:-far}" = off ]; then
  FAR=""; NEAR="--near-capture-redraw always --near-capture-redraw-lateral-m 0.10 --near-capture-expected on"
fi
END=(--end-session)
case ${SCENARIO:-cycle} in
  main) RUNX="--live-test-stop-in-capture"; PROBE=(--scenario main --expect-stop-in-capture); END=();;
  cancel) RUNX=""; PROBE=(--scenario cycle --cycle P-1:D-1,P-1:D-1 --cancel-at observe:5);;
  reject) RUNX="--live-test-far-zone-offset-m 3.0"; PROBE=(--scenario cycle --cycle P-1:D-1,P-1:D-1);;
  session) RUNX=""; PROBE=(--scenario cycle --cycle "${CYCLE:?the order of the session}");;
  # CT-1 PR 4: the console mission queue itself (tools/mission_queue.py, a port of queue.js)
  queue) RUNX=""; PROBE=(--scenario queue --cycle "${CYCLE:?the order of the session}" ${QUEUE_EXTRA:-});;
  *) RUNX=""; PROBE=(--scenario cycle --cycle P-1:D-1,P-1:D-0,P-1:D-1);;
esac
export EXTRA_ARGS="$FAR $NEAR $RUNX ${MORE:-}"
# DRY_RUN=1 (ws1-job-efficiency preflight, PR #181): no probe, no Isaac -- this wrapper's env checks above, then
# the launcher's own DRY_RUN (it prints RUNNER_ARGS and ARGS_OK, or exits 2); its output and exit code pass through
if [ "${DRY_RUN:-0}" = 1 ]; then
  grep -q "DRY_RUN" "$ROOT/deploy/console/isaac_live.sbatch" || { echo "DRY_RUN: this snapshot's isaac_live.sbatch has no DRY_RUN"; exit 2; }
  DRY_RUN=1 bash "$ROOT/deploy/console/isaac_live.sbatch" "$OUT/live"; exit $?
fi
echo "host=$(hostname) job=${SLURM_JOB_ID:-} out=$OUT root=$ROOT scene=$SCENE_FILE scenario=${SCENARIO:-cycle} $(date -Is)"
nvidia-smi --query-gpu=index,memory.used,memory.total --format=csv
bash "$ROOT/deploy/console/isaac_live.sbatch" "$OUT/live" > "$OUT/live.log" 2>&1 &
LIVE=$!
"$VENV_PY" /home/projects/forklift/test_runs/live/ct1_vram_sampler.py $$ "$OUT/vram.jsonl" 5 &
VRAM=$!
trap 'kill -TERM "$LIVE" 2>/dev/null' TERM INT
sleep 5
# the probe ends at once when the launcher ended before the session (ws1-job-efficiency PR); older snapshots lack the option
LP=(); grep -q -- "--launcher-pid" "$ROOT/tools/live_console_probe.py" && LP=(--launcher-pid "$LIVE")
"$VENV_PY" "$ROOT/tools/live_console_probe.py" --port-file "$OUT/live/live_port.json" --report "$OUT/probe.json" \
  "${LP[@]}" "${PROBE[@]}" "${END[@]}" --mission-timeout-s 2400 --observe-timeout-s 900 > "$OUT/probe.log" 2>&1 &
PROBE_PID=$!
wait "$PROBE_PID"; prc=$?  # in the background so that a TERM reaches the trap at once (bundle runner, Slurm)
while kill -0 "$PROBE_PID" 2>/dev/null; do wait "$PROBE_PID"; prc=$?; done
echo "probe exit $prc $(date -Is)"
[ ${#END[@]} -eq 0 ] && kill -TERM "$LIVE" 2>/dev/null  # main: the probe does not end the session
# CT-1 PR 4: a probe that stopped without ending the session (a lost link, a runner epoch change) leaves the
# supervisor serving a new epoch until the time limit -- end it after a short grace
for _ in $(seq 60); do kill -0 "$LIVE" 2>/dev/null || break; sleep 2; done
kill -TERM "$LIVE" 2>/dev/null
wait "$LIVE"; lrc=$?; echo "live exit $lrc $(date -Is)"
kill "$VRAM" 2>/dev/null
tail -5 "$OUT/live/supervisor.jsonl" 2>/dev/null
# the job's code: the probe's when it failed, else the live session's (the bundle runner and sacct read it)
[ "$prc" -ne 0 ] && exit "$prc"
exit "$lrc"
