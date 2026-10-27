#!/bin/bash
#SBATCH --job-name=scan_capture
#SBATCH --partition=compute
#SBATCH --gres=shard:2
#SBATCH --cpus-per-task=4
#SBATCH --mem=7G
#SBATCH --time=00:40:00
#SBATCH --signal=B:TERM@60
# Stationary LiDAR scans at listed poses on the world a survey bundle saw, for the global matcher's calibration
# (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md B4a): run_transport --scan-capture-poses with the
# arguments tools/global_match_capture.py command takes from the bundle's manifest; no mission, no SLAM link.
#
#   BUNDLE=<survey bundle> POSES=<pose list .json> VENV_PY=<python> sbatch deploy/slurm/scan_capture.sbatch OUT
#   DRY_RUN=1 ...: print RUNNER_ARGS (one per line) and run the runner's own --check-args (ARGS_OK or exit 2);
#   nothing starts. Run from the repository root (a snapshot).
set -u
OUT=$(realpath -m "${1:?the output folder}")
ROOT=$(pwd)
: "${BUNDLE:?the survey bundle}" "${POSES:?the pose list}" "${VENV_PY:?the python}"
BUNDLE=$(realpath -e "$BUNDLE") || { echo "no bundle folder $BUNDLE"; exit 2; }
POSES=$(realpath -e "$POSES") || { echo "no pose list $POSES"; exit 2; }
mapfile -t ARGS < <("$VENV_PY" "$ROOT/tools/global_match_capture.py" command --bundle "$BUNDLE" --poses "$POSES" \
  --out-npz "$OUT/capture.npz" --output "$OUT/run")
[ ${#ARGS[@]} -gt 0 ] || { echo "no runner arguments from the bundle's manifest"; exit 2; }
if [ "${DRY_RUN:-0}" = 1 ]; then
  echo RUNNER_ARGS
  printf '%s\n' "${ARGS[@]}"
  exec "$VENV_PY" "$ROOT/sim/isaac/run_transport.py" "${ARGS[@]}" --check-args
fi
mkdir -p "$OUT"
printf '%s\n' "${ARGS[@]}" > "$OUT/runner_args.txt"
echo "host=$(hostname) job=${SLURM_JOB_ID:-} bundle=$BUNDLE poses=$POSES $(date -Is)"
timeout "${RUN_TIMEOUT:-2100}" "$VENV_PY" "$ROOT/sim/isaac/run_transport.py" "${ARGS[@]}" > "$OUT/run.log" 2>&1
rc=$?
echo "runner exit $rc $(date -Is)"
[ -f "$OUT/capture.npz" ] || { echo "no capture written"; exit 1; }
exit "$rc"
